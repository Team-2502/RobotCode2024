use std::{cell::RefCell, borrow::BorrowMut, rc::Rc, ops::{Deref, DerefMut}};

use frcrs::{input::Joystick, };
use frcrs::networktables::SmartDashboard;
use tokio::task::{LocalSet, JoinHandle};
use uom::si::angle::{degree, radian};
use crate::{subsystems::{Climber, Drivetrain, Intake, Shooter}, constants::{BEAM_BREAK_SIGNAL, INTAKE_LIMIT, drivetrain::SWERVE_TURN_KP}};
use frcrs::deadzone;

#[derive(Clone)]
pub struct Ferris {
    pub drivetrain: Rc<RefCell<Drivetrain>>, 
    pub intake: Rc<RefCell<Intake>>,
    pub shooter: Rc<RefCell<Shooter>>, 
    pub climber: Rc<RefCell<Climber>>,
    grab: Rc<RefCell<Option<JoinHandle<()>>>>,
}

impl Ferris {
    pub fn new() -> Self { 
        let drivetrain = Rc::new(RefCell::new(Drivetrain::new()));
        let intake = Rc::new(RefCell::new(Intake::new()));
        let shooter = Rc::new(RefCell::new(Shooter::new()));
        let climber = Rc::new(RefCell::new(Climber::new()));
        Self { drivetrain, intake, shooter, climber, grab: Rc::new(RefCell::new(None))} 
    }
}

pub fn container<'a>(left_drive: &mut Joystick, right_drive: &mut Joystick, operator: &mut Joystick, robot: &'a Ferris, executor: &'a LocalSet) {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let shooter = robot.shooter.deref().borrow();
    let climber = robot.climber.deref().borrow();

    let joystick_range = 0.04..1.;
    let power_translate = if right_drive.get(3) { 0.0..0.3 }
    else { 0.0..1. };
    let power_rotate = if right_drive.get(3) { 0.0..0.2 }
    else { 0.0..1. };
    let deadly = deadzone(left_drive.get_y(), &joystick_range, &power_translate);
    let deadlx = deadzone(left_drive.get_x(), &joystick_range, &power_translate);
    let deadrz = deadzone(right_drive.get_z(), &joystick_range, &power_rotate);

    let rot = if left_drive.get(4) {
        -drivetrain.get_offset().get::<radian>() * SWERVE_TURN_KP
    } else {
        deadrz
    };

    drivetrain.set_speeds(deadly, deadlx, rot);

    SmartDashboard::put_number("Angle".to_owned(), drivetrain.get_angle().get::<degree>());

    let mut shooting = false;

    if left_drive.get(3) {
        drivetrain.reset_heading();
    }

    if left_drive.get(1) {
        drivetrain.reset_angle();
    }

    if operator.get(8) && robot.grab.deref().try_borrow().is_ok_and(|n| n.is_none()) {
        let intake = robot.intake.clone();
        robot.grab.replace(Some(executor.spawn_local(async move {
            intake.deref().borrow_mut().grab().await;
        })));
    } else if !operator.get(8) {
        if let Some(grab) = robot.grab.take() {
            grab.abort();
        }
    }
    
    if let Ok(intake) = robot.intake.try_borrow_mut() {
        SmartDashboard::put_bool("intake at limit {}".to_owned(), intake.at_limit());
        if operator.get(9) {
            intake.set_rollers(0.4);
        } else if operator.get(7) {
            intake.set_rollers(-0.4);
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

    SmartDashboard::put_bool("beam break: {}".to_owned(), shooter.contains_note());
    //println!("doo dad: {}", get_dio(INTAKE_LIMIT));
}

pub fn stop_all(robot: &Ferris) {
    robot.drivetrain.borrow().stop();
    robot.intake.borrow().stop();
    robot.shooter.borrow().stop();
    robot.climber.borrow().stop();
}
