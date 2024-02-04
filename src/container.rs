use frcrs::input::Joystick;
use frcrs::networktables::SmartDashboard;
use uom::si::angle::degree;
use crate::subsystems::{Climber, Drivetrain, Intake, Shooter};

pub fn container(left_drive: &Joystick, right_drive: &Joystick, operator: &Joystick, drivetrain: &Drivetrain, intake: &Intake,
                 shooter: &Shooter, climber: &Climber) {
    drivetrain.set_speeds(-left_drive.get_y(), -left_drive.get_x(), right_drive.get_z());

    SmartDashboard::put_number("Angle".to_owned(), drivetrain.get_angle().get::<degree>());

    let mut shooting = false;

    if left_drive.get(1) {
        drivetrain.reset_angle();
    }

    if operator.get(5) {
        intake.set_rollers(1.);
    } else if operator.get(6) {
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

    if operator.get(2) { shooting = !shooting; }

    if shooting {
        shooter.set_shooter((operator.get_throttle() + 1.) / 2.);
    } else {
        shooter.stop_shooter();
    }

    if operator.get(1) {
        shooter.set_feeder(1.);
    } else if operator.get(7) {
        shooter.set_feeder(-0.5);
    } else {
        shooter.stop_feeder();
    }

    if left_drive.get(2) {
        climber.set(0.5)
    } else if right_drive.get(2) {
        climber.set(-0.5);
    } else {
        climber.stop();
    }
}

pub fn stop_all(drivetrain: &Drivetrain, intake: &Intake, shooter: &Shooter, climber: &Climber) {
    drivetrain.stop();
    intake.stop();
    shooter.stop();
    climber.stop();
}
