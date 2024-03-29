
use uom::si::{angle::degree, f64::Angle};

use crate::{constants::intake::{INTAKE_DOWN_GOAL, INTAKE_UP_GOAL}, subsystems::{Climber, Intake}, telemetry};

use super::Controllers;

pub async fn control_climber(climber: &mut Climber, controllers: &mut Controllers) {
    let operator = &mut controllers.operator;
    let right_drive = &mut controllers.right_drive;
    let left_drive = &mut controllers.left_drive;

    let mut climbing = false;
    if left_drive.get(3) {
        climber.set(1.);
        climbing = true;
    } else {
        if operator.get(14) {
            climber.set_left(-1.);
            climbing = true;
        } else if operator.get(13) {
            climber.set_left(1.);
            climbing = true;
        } 

        if operator.get(15) {
            climber.set_right(1.);
            climbing = true;
        } else if operator.get(12) {
            climber.set_right(-1.);
            climbing = true;
        }
    };
    if !climbing { climber.stop(); }
}

