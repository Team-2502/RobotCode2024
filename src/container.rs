use std::{borrow::BorrowMut, cell::RefCell, ops::{Deref, DerefMut}, rc::Rc, time::Duration, sync::Arc};

use frcrs::{alliance_station, input::Joystick, networktables::SmartDashboard };
use frcrs::networktables::set_position;
use tokio::{join, sync::RwLock, task::{JoinHandle, LocalSet}, time::{sleep, timeout}};
use uom::si::{angle::{degree, radian}, f64::Angle};
use crate::{constants::{drivetrain::{SWERVE_TURN_KP, self}, intake::{INTAKE_DOWN_GOAL, INTAKE_UP_GOAL}, BEAM_BREAK_SIGNAL, INTAKE_LIMIT}, subsystems::{wait, Climber, Drivetrain, Intake, Shooter}, auto::raise_intake, telemetry::{TelemetryStore, self, TELEMETRY}};
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
        let telemetry = TELEMETRY.clone();
        Self { drivetrain, intake, shooter, climber, grab: Rc::new(RefCell::new(None)), shooter_state, stage: Rc::new(RefCell::new(None)), telemetry } 
    }
}

pub async fn container<'a>(left_drive: &mut Joystick, right_drive: &mut Joystick, operator: &mut Joystick, robot: &'a Ferris, executor: &'a LocalSet) {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
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

    telemetry::put_number("Odo X", drivetrain.odometry.position.x).await;
    telemetry::put_number("Odo Y", drivetrain.odometry.position.y).await;

    telemetry::put_number("Angle", angle.get::<degree>()).await;
    let red = alliance_station().red();
    telemetry::put_bool("red", red).await;

    if left_drive.get(3) {
        drivetrain.reset_heading();
    }

    if left_drive.get(1) {
        drivetrain.reset_angle();
    }

    if operator.get(8) && robot.grab.deref().try_borrow().is_ok_and(|n| n.is_none()) && !operator.get(7) {
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
    if operator.get(7) && not && robot.shooter.try_borrow().is_ok_and(|s| !s.contains_note()) {

        // drop grab if done
        if robot.grab.deref().try_borrow().is_ok_and(|n| n.as_ref().is_some_and(|n| n.is_finished())) {
            if let Ok(mut grab) = robot.grab.deref().try_borrow_mut() {
                *grab = None;
            }
        }

        let intake = robot.intake.clone();
        let shooter = robot.shooter.clone();
        robot.stage.replace(Some(executor.spawn_local(async move {
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
            telemetry::put_bool("intake at limit {}", intake.at_limit()).await;
            telemetry::put_bool("intake at reverse limit {}", intake.at_reverse_limit()).await;
            telemetry::put_number("intake position {}", intake.actuate_position().get::<degree>()).await;

            if operator.get(9) {
                intake.set_rollers(1.);
            } else if operator.get(6) || operator.get(1) || right_drive.get(1) {
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
    }

    if operator.get(2) && !*last_loop { 
        *shooting = !*shooting; 
    }
    *last_loop = operator.get(2);

    telemetry::put_bool("flywheel state", *shooting).await;

    if let Ok(mut shooter) = robot.shooter.deref().try_borrow_mut() {
        telemetry::put_number("flywheel speed", shooter.get_velocity()).await;
        telemetry::put_bool("beam break: {}", shooter.contains_note()).await;

        if *shooting {
            if shooter.amp_deployed() && !operator.get(5) {
                shooter.set_shooter(0.225)
            } else {
                shooter.set_shooter((operator.get_throttle() + 1.) / 2.);
            }
        } else {
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

        if !staging {
            if operator.get(1) || right_drive.get(1) {
                shooter.set_feeder(-1.);
            } else if operator.get(10) {
                shooter.set_feeder(0.5);
            } else {
                shooter.stop_feeder();
            }
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

    //println!("doo dad: {}", get_dio(INTAKE_LIMIT));
}

/// Transfer note from intake to shooter
pub async fn stage(intake: &mut Intake, shooter: &Shooter) {
    intake.set_rollers(1.);
    raise_intake(intake).await;
    //intake.set_actuate(0.15);
    //let _ = timeout(Duration::from_millis(200), wait(|| intake.at_limit())).await;

    sleep(Duration::from_millis(200)).await;

    intake.set_rollers(-0.13);
    shooter.set_feeder(-0.14);
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
