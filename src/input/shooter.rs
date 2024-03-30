
use frcrs::{alliance_station, deadzone};
use uom::si::{angle::{degree, radian}, f64::Angle};

use crate::{constants::drivetrain::{PODIUM_SHOT_ANGLE, SWERVE_TURN_KP}, subsystems::{Drivetrain, Shooter}, telemetry};

use super::{Controllers, GamepadState};

#[derive(Default)]
pub struct ShooterControlState {
    shooting: bool,
    last_loop: bool, // shoot button
    pub staging: bool,
    pub firing: bool,
    gamepad_spinning: bool,
    gamepad_spinning_last: bool,
}

pub async fn control_shooter(shooter: &mut Shooter, controllers: &mut Controllers, state: &mut ShooterControlState) {
    let right_drive = &mut controllers.right_drive;
    let left_drive = &mut controllers.left_drive;
    let gamepad = &mut controllers.gamepad;
    let gamepad_state = &mut controllers.gamepad_state;
    let operator = &mut controllers.operator;
    let shooting = &mut state.shooting;
    let staging = &mut state.staging;
    let firing = &mut state.firing;
    let last_loop = &mut state.last_loop;
    let gamepad_spinning = &mut state.gamepad_spinning;
    telemetry::put_number("flywheel speed", shooter.get_velocity()).await;
    telemetry::put_bool("beam break: {}", shooter.contains_note()).await;
    telemetry::put_bool("flywheel state", *shooting).await;

    if matches!(gamepad_state, GamepadState::Auto | GamepadState::Drive) {
        if gamepad.a() { // line shot
            if right_drive.get(2) {  // podium
                shooter.set_velocity(2080.);
                gamepad.rumble_right((2080.-shooter.get_velocity())/2000.);
            } else {
                shooter.set_velocity(5500.);
                gamepad.rumble_right((5500.-shooter.get_velocity())/2000.);
            }
            shooter.stow_amp();
            *gamepad_spinning = true;
        } else if gamepad.b() { // amp
            shooter.set_shooter(0.225);
            gamepad.rumble_right((1000.-shooter.get_velocity())/2000.);
            shooter.deploy_amp();
            *gamepad_spinning = true;
        } else if gamepad.y() { // pass over stage
            shooter.set_shooter(0.4);
            gamepad.rumble_right((2500.-shooter.get_velocity())/2000.);
            *gamepad_spinning = true;
        } else if gamepad.x() {
            shooter.stop_shooter();
            *gamepad_spinning = false;
        } else if matches!(gamepad_state, GamepadState::Manual) && gamepad.right_trigger() > 0. {
            shooter.set_shooter(gamepad.right_trigger());
            gamepad.rumble_right(shooter.get_velocity()/3000.);
            *gamepad_spinning = true;
        } else {
            gamepad.rumble_right(0.);
        }
    }

    if operator.get(2) && !*last_loop { 
        *shooting = !*shooting; 
    }
    *last_loop = operator.get(2);

    *firing = operator.get(1) || right_drive.get(1) || 
        matches!(gamepad_state, GamepadState::Auto | GamepadState::Manual | GamepadState::Drive) && gamepad.right_bumper();

    if *shooting && !*gamepad_spinning {
        if shooter.amp_deployed() && !operator.get(5) {
            shooter.set_shooter(0.225)
        } else if right_drive.get(2) {
            shooter.set_velocity(1917.)
        } else {
            shooter.set_shooter((operator.get_throttle() + 1.) / 2.);
        }
    } else if !*gamepad_spinning {
        shooter.stop_shooter();
    }

    if operator.get(5) {
        if operator.get(11) {
            shooter.set_amp_bar(-0.6);
        } else if operator.get(16) {
            shooter.set_amp_bar(0.6);
        } else {
            shooter.set_amp_bar(0.);
        }
    } else {
        if operator.get(11) {
            shooter.deploy_amp();
        } else if operator.get(16) {
            shooter.stow_amp();
        }
    }

    if !*staging {
        if *firing {
            shooter.set_feeder(-1.);
        } else if matches!(gamepad_state, GamepadState::Manual) && gamepad.left_trigger() > 0. {
            shooter.set_feeder(gamepad.left_trigger());
        } else if matches!(gamepad_state, GamepadState::Manual) && gamepad.left_bumper() {
            if shooter.contains_note() {
                shooter.set_feeder(0.1);
            } else {
                shooter.set_feeder(-0.3);
            }
        } else if operator.get(10) {
            shooter.set_feeder(0.5);
        } else {
            shooter.stop_feeder();
        }
    }
}
