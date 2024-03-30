use frcrs::{alliance_station, deadzone};
use nalgebra::ComplexField;
use uom::si::{angle::{degree, radian}, f64::Angle};

use crate::{constants::drivetrain::{PODIUM_SHOT_ANGLE, SWERVE_TURN_KP}, subsystems::Drivetrain, telemetry};

use super::{Controllers, GamepadState};

#[derive(Default)]
pub struct DrivetrainControlState {
    saved_angle: Option<Angle>
}

pub async fn control_drivetrain(drivetrain: &mut Drivetrain, controllers: &mut Controllers, state: &mut DrivetrainControlState) {
    let right_drive = &mut controllers.right_drive;
    let left_drive = &mut controllers.left_drive;
    let saved_angle = &mut state.saved_angle;
    let gamepad = &mut controllers.gamepad;
    let gamepad_state = &mut controllers.gamepad_state;

    let joystick_range = 0.04..1.;
    let mut power_translate = if left_drive.get(1) { 0.0..0.3 }
    else { 0.0..1. };
    let mut power_rotate = if left_drive.get(1) { 0.0..0.2 }
    else { 0.0..1. };
    let mut deadly = deadzone(left_drive.get_y(), &joystick_range, &power_translate);
    let mut deadlx = deadzone(left_drive.get_x(), &joystick_range, &power_translate);
    let mut deadrz = deadzone(right_drive.get_z(), &joystick_range, &power_rotate);

    if matches!(gamepad_state, GamepadState::Drive) {
        let gamepad_range = 0. .. 1.;
        let pow = 2.;

        power_rotate.end *= 1. - gamepad.right_trigger() * 0.7;
        power_rotate.start = 0.1;
        power_translate.end *= 1. - gamepad.left_trigger() * 0.7;

        deadly += deadzone(gamepad.left_y().powf(pow), &gamepad_range, &power_translate);
        deadlx += deadzone(gamepad.left_x().powf(pow), &gamepad_range, &power_translate);
        deadrz += deadzone(gamepad.right_x().powf(pow), &gamepad_range, &power_rotate);
    }

    let hold_angle = deadrz == 0. && (right_drive.get(3) || matches!(gamepad_state, GamepadState::Drive));

    if !hold_angle { *saved_angle = Some(drivetrain.get_angle());
    }

    let rot = if right_drive.get(2) {
        let mut error = drivetrain.get_offset() + Angle::new::<degree>(PODIUM_SHOT_ANGLE);
        if alliance_station().blue() { error *= -1.; }
        -error.get::<radian>() * SWERVE_TURN_KP
    } else if hold_angle {
        if let Some(ref saved_angle) = (saved_angle).as_ref() {
            let error = drivetrain.get_angle() - **saved_angle;
            -error.get::<radian>() * SWERVE_TURN_KP
        } else {
            0.
        }
    } else if left_drive.get(2) {
        let angle = (drivetrain.get_angle() - drivetrain.offset).get::<degree>();
        let goal = (angle / 90.).round() * 90.;
        let error = angle - goal;
        -error.to_radians() * SWERVE_TURN_KP
    } else {
        deadrz
    };

    if matches!(gamepad_state, GamepadState::Manual) && gamepad.y() {
            drivetrain.zero_wheels()
    } else {
        drivetrain.set_speeds(deadly, deadlx, rot);
    }

    let angle = drivetrain.get_angle();

    if left_drive.get(4) {
        drivetrain.reset_heading();
    }

    telemetry::put_number("Odo X", drivetrain.odometry.position.x).await;
    telemetry::put_number("Odo Y", drivetrain.odometry.position.y).await;

    telemetry::put_number("Angle", angle.get::<degree>()).await;
}
