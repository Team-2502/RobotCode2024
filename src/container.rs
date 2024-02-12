use frcrs::input::Joystick;
use frcrs::networktables::SmartDashboard;
use smol::{Task, LocalExecutor};
use uom::si::angle::degree;
use crate::subsystems::{Climber, Drivetrain, Intake, Shooter};
use frcrs::deadzone;

pub struct Ferris {
    drivetrain: Drivetrain, 
    intake: Intake,
    shooter: Shooter, 
    climber: Climber,
    intake_state: IntakeState,
    grab: Option<Task<()>>,
}

enum IntakeState { // TODO: use async/await instead of a state machine
    Not,
    Accelerating,
    Running,
    Stalled,
}

impl IntakeState {
    fn running(&self) -> bool {
        match self {
            IntakeState::Running | IntakeState::Accelerating => {
                true
            }
            _ => false
        }
    }

    fn start(&mut self) {
        match self {
            IntakeState::Not => {
                *self = IntakeState::Accelerating
            }
            _ => {}
        }
    }

    fn reset(&mut self) {
        match self {
            IntakeState::Stalled | IntakeState::Running | IntakeState::Accelerating => {
                *self = IntakeState::Not
            }
            _ => {}
        }
    }

    fn spinning(&mut self) {
        match self {
            IntakeState::Accelerating => {
                *self = IntakeState::Running
            }
            _ => {}
        }
    }

    fn stall(&mut self) {
        match self {
            IntakeState::Running => {
                *self = IntakeState::Stalled
            }
            _ => {}
        }
    }
}

impl Ferris {
    pub fn new() -> Self { 
        let drivetrain = Drivetrain::new();
        let intake = Intake::new();
        let shooter = Shooter::new();
        let climber = Climber::new();
        Self { drivetrain, intake, shooter, climber, intake_state: IntakeState::Not, grab: None} 
    }
}

pub fn container(left_drive: &mut Joystick, right_drive: &mut Joystick, operator: &mut Joystick, robot: &mut Ferris, executor: &LocalExecutor) {
    let drivetrain = &mut robot.drivetrain;
    let intake = &mut robot.intake;
    let shooter = &robot.shooter;
    let climber = &robot.climber;
    let joystick_range = 0.04..1.;
    let power_translate = if right_drive.get(3) { 0.0..0.3 }
    else { 0.0..1. };
    let power_rotate = if right_drive.get(3) { 0.0..0.2 }
    else { 0.0..1. };
    let deadly = deadzone(left_drive.get_y(), &joystick_range, &power_translate);
    let deadlx = deadzone(left_drive.get_x(), &joystick_range, &power_translate);
    let deadrz = deadzone(right_drive.get_z(), &joystick_range, &power_rotate);
    drivetrain.set_speeds(deadly, deadlx, deadrz);

    SmartDashboard::put_number("Angle".to_owned(), drivetrain.get_angle().get::<degree>());

    let mut shooting = false;

    if left_drive.get(3) {
        drivetrain.reset_heading();
    }

    if left_drive.get(1) {
        drivetrain.reset_angle();
    }

    if operator.get(10) && robot.grab.is_none() {
        robot.grab = Some(executor.spawn(robot.intake.grab()));
    } else if !operator.get(10) {
        if let Some(grab) = robot.grab.take() {
            drop(grab);
        }
    }

    if operator.get(8) {
        robot.intake_state.start();
    } else {
        robot.intake_state.reset();
    }

    if intake.running() {
        robot.intake_state.spinning();
    }
    if intake.stalled() {
        robot.intake_state.stall();
    }

    if robot.intake_state.running() {
        intake.set_rollers(0.4);
    } else if operator.get(7) {
        intake.set_rollers(-0.4);
    } else if operator.get(9) {
        intake.set_rollers(0.4);
    } else {
        intake.stop_rollers();
    }

    if operator.get(3) {
        intake.set_actuate(0.6);
    } else if operator.get(4) {
        intake.set_actuate(-0.6);
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
        shooter.set_feeder(-0.2);
    } else if operator.get(10) {
        shooter.set_feeder(0.5);
    } else {
        shooter.stop_feeder();
    }

    // Todo: cleanup
    if left_drive.get(2) {
        climber.set(-1.)
    } else if right_drive.get(2) {
        climber.set(1.);
    } else if operator.get(14) {
        climber.set_left(-1.);
    } else if operator.get(13) {
        climber.set_left(1.);
    } else if operator.get(15) {
        climber.set_right(1.);
    } else if operator.get(12) {
        climber.set_right(-1.);
    } else {
        climber.stop()
    }
}

pub fn stop_all(robot: &Ferris) {
    robot.drivetrain.stop();
    robot.intake.stop();
    robot.shooter.stop();
    robot.climber.stop();
}
