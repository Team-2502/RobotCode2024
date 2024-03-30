use nalgebra::ComplexField;
use uom::si::{angle::degree, f64::Angle};

use crate::{constants::intake::{INTAKE_DOWN_GOAL, INTAKE_UP_GOAL}, subsystems::Intake, telemetry};

use super::{Controllers, GamepadState};

pub async fn control_intake(intake: &mut Intake, controllers: &mut Controllers) {
    let operator = &mut controllers.operator;
    let right_drive = &mut controllers.right_drive;
    let gamepad = &mut controllers.gamepad;
    let gamepad_state = &mut controllers.gamepad_state;
    telemetry::put_bool("intake at limit {}", intake.at_limit()).await;
    telemetry::put_number("intake position {}", intake.actuate_position().get::<degree>()).await;

    if matches!(gamepad_state, GamepadState::Manual | GamepadState::Auto) && gamepad.left_trigger() > 0. { 
        intake.set_rollers(gamepad.left_trigger());
    } else if operator.get(8) && operator.get(5)  {
        intake.set_rollers(1.);
    } else if (operator.get(7) && operator.get(5)) 
        || operator.get(1) || right_drive.get(1) 
        || matches!(gamepad_state, GamepadState::Manual | GamepadState::Auto) && gamepad.right_bumper() 
        || matches!(gamepad_state, GamepadState::Manual) && gamepad.left_bumper()
    {
        intake.set_rollers(-1.);
    } else {
        intake.stop_rollers();
    }

    if operator.get(5) || matches!(gamepad_state, GamepadState::Manual) {
        if operator.get(3) || gamepad.left_stick() {
            intake.set_actuate(0.3);
        } else if operator.get(4) || gamepad.right_stick() {
            intake.set_actuate(-0.3);
        } else {
            intake.stop_actuate();
        }
    } else {
        if operator.get(3) || matches!(gamepad_state, GamepadState::Auto) && gamepad.right_stick()  {
            intake.actuate_to(Angle::new::<degree>(INTAKE_UP_GOAL));
        } else if operator.get(4) || matches!(gamepad_state, GamepadState::Auto) && gamepad.left_stick() {
            intake.actuate_to(Angle::new::<degree>(INTAKE_DOWN_GOAL));
        }
    }
}

