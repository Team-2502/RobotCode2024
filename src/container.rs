use frcrs::input::Joystick;
use frcrs::networktables::SmartDashboard;
use crate::subsystems::{Drivetrain, Intake};

pub fn container(left_drive: &Joystick, right_drive: &Joystick, operator: &Joystick, drivetrain: &Drivetrain, intake: &Intake) {
    drivetrain.set_speeds(-left_drive.get_y(), -left_drive.get_x(), right_drive.get_z());

    SmartDashboard::put_number("Angle".to_owned(), drivetrain.get_angle());

    if left_drive.get(1) {
        drivetrain.reset_angle();
    }

    if operator.get(1) {
        intake.set_rollers(1.);
    } else if operator.get(2) {
        intake.set_rollers(-1.);
    } else {
        intake.stop_rollers();
    }

    if operator.get(3) {
        intake.set_actuate(0.2);
    } else if operator.get(4) {
        intake.set_actuate(-0.2);
    } else {
        intake.stop_actuate();
    }
}

pub fn stop_all(drivetrain: &Drivetrain, intake: &Intake) {
    drivetrain.stop();
    intake.stop();
}