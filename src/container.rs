use std::{borrow::BorrowMut, cell::RefCell, ops::{Deref, DerefMut}, rc::Rc, time::Duration, sync::Arc};

use frcrs::{input::Joystick, alliance_station, };
use frcrs::networktables::SmartDashboard;
use frcrs::networktables::set_position;
use tokio::{task::{JoinHandle, LocalSet}, time::sleep, join, sync::RwLock};
use uom::si::{angle::{degree, radian}, f64::Angle};
use crate::{constants::{drivetrain::{SWERVE_TURN_KP, self}, intake::{INTAKE_DOWN_GOAL, INTAKE_UP_GOAL}, BEAM_BREAK_SIGNAL, INTAKE_LIMIT}, subsystems::{wait, Climber, Drivetrain, Intake, Shooter}, auto::raise_intake, telemetry::{TelemetryStore, self}};
use frcrs::deadzone;

#[derive(Clone)]
pub struct Ferris {
    pub drivetrain: Rc<RefCell<Drivetrain>>, 
    pub intake: Rc<RefCell<Intake>>,
    pub shooter: Rc<RefCell<Shooter>>, 
    pub climber: Rc<RefCell<Climber>>,
    grab: Rc<RefCell<Option<JoinHandle<()>>>>,
    stage: Rc<RefCell<Option<JoinHandle<()>>>>,
    shooter_state: Rc<RefCell<(bool,bool)>>,
    pub telemetry: TelemetryStore,
}

impl Ferris {
    pub fn new() -> Self { 
        let drivetrain = Rc::new(RefCell::new(Drivetrain::new()));
        let intake = Rc::new(RefCell::new(Intake::new()));
        let shooter = Rc::new(RefCell::new(Shooter::new()));
        let climber = Rc::new(RefCell::new(Climber::new()));
        let shooter_state = Rc::new(RefCell::new((false,false)));
        let telemetry = Arc::new(RwLock::new(Default::default()));
        Self { drivetrain, intake, shooter, climber, grab: Rc::new(RefCell::new(None)), shooter_state, stage: Rc::new(RefCell::new(None)), telemetry } 
    }
}

pub fn container<'a>(left_drive: &mut Joystick, right_drive: &mut Joystick, operator: &mut Joystick, robot: &'a Ferris, executor: &'a LocalSet) {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let shooter = robot.shooter.deref().borrow();
    let climber = robot.climber.deref().borrow();
    let mut shooter_state = robot.shooter_state.deref().borrow_mut();
    let (shooting, last_loop) = &mut *shooter_state;

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
    let angle = drivetrain.get_angle();

    set_position(drivetrain.odometry.position, -angle);

    SmartDashboard::put_number("Odo X".to_owned(), drivetrain.odometry.position.x);
    SmartDashboard::put_number("Odo Y".to_owned(), drivetrain.odometry.position.y);

    SmartDashboard::put_number("Angle".to_owned(), angle.get::<degree>());
    let red = alliance_station().red();
    SmartDashboard::put_bool("red".to_owned(), red);

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

    let not = robot.stage.deref().try_borrow().is_ok_and(|n| n.is_none());
    let staging = robot.stage.deref().try_borrow().is_ok_and(|n| n.is_some());
    if operator.get(7) && not && !shooter.contains_note() {
        let intake = robot.intake.clone();
        let shooter = robot.shooter.clone();
        robot.grab.replace(Some(executor.spawn_local(async move {
            let intake = intake.deref().try_borrow_mut();
            let shooter = shooter.deref().try_borrow();

            if let (Ok(mut intake), Ok(shooter)) = (intake, shooter) {
                stage(&mut intake, &shooter).await;
            }
        })));
    } else if !operator.get(7) {
        if let Some(stage) = robot.stage.take() {
            stage.abort();
        }
    }
    
    if !staging {
        if let Ok(mut intake) = robot.intake.try_borrow_mut() {
            SmartDashboard::put_bool("intake at limit {}".to_owned(), intake.at_limit());
            SmartDashboard::put_bool("intake at reverse limit {}".to_owned(), intake.at_reverse_limit());
            SmartDashboard::put_number("intake position {}".to_owned(), intake.actuate_position().get::<degree>());

            if operator.get(9) {
                intake.set_rollers(0.4);
            } else if operator.get(6) {
                intake.set_rollers(-0.4);
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
    }

    if operator.get(2) && !*last_loop { 
        *shooting = !*shooting; 
    }
    *last_loop = operator.get(2);

    SmartDashboard::put_bool("flywheel state".to_owned(), *shooting);

    if *shooting {
        shooter.set_shooter((operator.get_throttle() + 1.) / 2.);
    } else {
        shooter.stop_shooter();
    }

    if operator.get(11) {
        shooter.set_amp_bar(-0.2);
    } else if operator.get(16) {
        shooter.set_amp_bar(0.2);
    } else {
        shooter.set_amp_bar(0.);
    }

    if !staging {
        if operator.get(1) {
            shooter.set_feeder(-0.2);
        } else if operator.get(10) {
            shooter.set_feeder(0.5);
        } else {
            shooter.stop_feeder();
        }
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

    let drivetrain = robot.drivetrain.clone();
    let telemetry = robot.telemetry.clone();
    executor.spawn_local(async move {
        if let Ok(mut dt) = drivetrain.deref().try_borrow_mut() {
            dt.odometry.update_from_vision(telemetry, red).await;
        }
    });
}

/// Transfer note from intake to shooter
pub async fn stage(intake: &mut Intake, shooter: &Shooter) {
    intake.set_rollers(1.);
    raise_intake(intake).await;
    intake.set_actuate(0.15);
    wait(|| intake.at_limit()).await;

    intake.set_rollers(-0.1);
    shooter.set_feeder(-0.24);
    wait(|| shooter.contains_note()).await;
    intake.set_rollers(0.0);
    intake.set_actuate(0.0);

    shooter.set_feeder(-0.10);
    wait(|| !shooter.contains_note()).await;
    shooter.set_feeder(0.0);
}

pub fn stop_all(robot: &Ferris) {
    robot.drivetrain.borrow().stop();
    robot.intake.borrow().stop();
    robot.shooter.borrow().stop();
    robot.climber.borrow().stop();
}
