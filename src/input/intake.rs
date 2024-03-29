use uom::si::{angle::degree, f64::Angle};

use crate::{constants::intake::{INTAKE_DOWN_GOAL, INTAKE_UP_GOAL}, subsystems::Intake, telemetry};

use super::Controllers;

pub async fn control_intake(intake: &mut Intake, controllers: &mut Controllers) {
    let operator = &mut controllers.operator;
    let right_drive = &mut controllers.right_drive;
    telemetry::put_bool("intake at limit {}", intake.at_limit()).await;
    telemetry::put_bool("intake at reverse limit {}", intake.at_reverse_limit()).await;
    telemetry::put_number("intake position {}", intake.actuate_position().get::<degree>()).await;

    if operator.get(8) && operator.get(5) {
        intake.set_rollers(1.);
    } else if (operator.get(7) && operator.get(5)) || operator.get(1) || right_drive.get(1) {
        intake.set_rollers(-1.);
    } else {
        intake.stop_rollers();
    }

    if operator.get(5) {
        if operator.get(3) && !intake.at_limit() {
            intake.set_actuate(0.3);
        } else if operator.get(4) && !intake.at_reverse_limit() {
            intake.set_actuate(-0.3);
        } else {
            intake.stop_actuate();
        }
    } else {
        if operator.get(3) {
            intake.actuate_to(Angle::new::<degree>(INTAKE_UP_GOAL));
        } else if operator.get(4) {
            intake.actuate_to(Angle::new::<degree>(INTAKE_DOWN_GOAL));
        }
    }
}

