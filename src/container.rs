use frcrs::input::Joystick;
use frcrs::networktables::SmartDashboard;
use crate::subsystems::Drivetrain;

pub fn container(left_drive: &Joystick, right_drive: &Joystick, operator: &Joystick, drivetrain: &Drivetrain) {
    drivetrain.set_speeds(-left_drive.get_y(), -left_drive.get_x(), right_drive.get_z());

    SmartDashboard::put_number("Angle".to_owned(), drivetrain.get_angle());

    if left_drive.get(1) {
        drivetrain.reset_angle();
    }
}

pub fn stop_all(drivetrain: &Drivetrain) {
    drivetrain.stop()
}