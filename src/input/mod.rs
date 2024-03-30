use std::{
    borrow::BorrowMut, cell::RefCell, ops::Deref, rc::Rc, sync::atomic::AtomicBool, time::Duration,
};

use frcrs::{
    alliance_station,
    input::{Direction, Gamepad, Joystick},
};

use tokio::{
    task::{JoinHandle, LocalSet},
    time::sleep,
};

use crate::{
    auto::{lower_intake, raise_intake},
    subsystems::{wait, Climber, Drivetrain, Intake, Shooter},
    telemetry::{self, TelemetryStore, TELEMETRY},
};

use self::{
    climber::control_climber,
    drivetrain::{control_drivetrain, DrivetrainControlState},
    intake::control_intake,
    shooter::{control_shooter, ShooterControlState},
};

mod climber;
mod drivetrain;
mod intake;
mod shooter;

#[derive(Clone)]
pub struct Ferris {
    pub drivetrain: Rc<RefCell<Drivetrain>>,
    pub intake: Rc<RefCell<Intake>>,
    pub shooter: Rc<RefCell<Shooter>>,
    pub climber: Rc<RefCell<Climber>>,
    grab: Rc<RefCell<Option<JoinHandle<()>>>>,
    stage: Rc<RefCell<Option<JoinHandle<()>>>>,
    grab_full: Rc<RefCell<Option<JoinHandle<()>>>>,
    shooter_state: Rc<RefCell<(bool, bool)>>,
    teleop_state: Rc<RefCell<TeleopState>>,
    pub telemetry: TelemetryStore,
}

#[derive(Default)]
struct TeleopState {
    drivetrain_state: DrivetrainControlState,
    shooter_state: ShooterControlState,
}

pub struct Controllers {
    pub left_drive: Joystick,
    pub right_drive: Joystick,
    pub operator: Joystick,
    pub gamepad: Gamepad,
    pub gamepad_state: GamepadState,
}

#[derive(Clone, Copy, Debug)]
pub enum GamepadState {
    Auto,
    Manual,
    Climb,
    Drive,
}

impl Ferris {
    pub fn new() -> Self {
        let drivetrain = Rc::new(RefCell::new(Drivetrain::new()));
        let intake = Rc::new(RefCell::new(Intake::new()));
        let shooter = Rc::new(RefCell::new(Shooter::new()));
        let climber = Rc::new(RefCell::new(Climber::new()));
        let shooter_state = Rc::new(RefCell::new((false, false)));
        let telemetry = TELEMETRY.clone();
        Self {
            drivetrain,
            intake,
            shooter,
            climber,
            grab: Rc::new(RefCell::new(None)),
            grab_full: Rc::new(RefCell::new(None)),
            shooter_state,
            stage: Rc::new(RefCell::new(None)),
            teleop_state: Rc::new(RefCell::new(Default::default())),
            telemetry,
        }
    }
}

pub async fn container<'a>(
    controllers: &mut Controllers,
    robot: &'a Ferris,
    executor: &'a LocalSet,
    dt: Duration,
) {
    let TeleopState {
        ref mut drivetrain_state,
        ref mut shooter_state,
    } = *robot.teleop_state.deref().borrow_mut();

    if let Ok(mut drivetrain) = robot.drivetrain.try_borrow_mut() {
        control_drivetrain(&mut drivetrain, controllers, drivetrain_state).await;
    } else {
    }

    if let Ok(mut intake) = robot.intake.try_borrow_mut() {
        control_intake(&mut intake, controllers, &dt).await;
    }

    if let Ok(mut shooter) = robot.shooter.try_borrow_mut() {
        control_shooter(&mut shooter, controllers, shooter_state).await;
    }

    if let Ok(mut climber) = robot.climber.try_borrow_mut() {
        control_climber(&mut climber, controllers).await;
    }

    let red = alliance_station().red();
    telemetry::put_bool("red", red).await;

    let staging = &mut shooter_state.staging;
    let firing = &mut shooter_state.firing;

    let Controllers {
        left_drive: _,
        right_drive: _,
        ref mut operator,
        ref mut gamepad,
        ref mut gamepad_state,
    } = controllers;

    *gamepad_state = match gamepad.get_dpad_direction() {
        Direction::Left => GamepadState::Manual,
        Direction::Up => GamepadState::Climb,
        Direction::Down => GamepadState::Auto,
        Direction::Right => GamepadState::Drive,
        _ => *gamepad_state,
    };

    if operator.get(8)
        && robot.grab.deref().try_borrow().is_ok_and(|n| n.is_none())
        && !operator.get(7)
        && !operator.get(5)
    {
        let intake = robot.intake.clone();
        robot.grab.replace(Some(executor.spawn_local(async move {
            intake.deref().borrow_mut().grab().await;
        })));
    } else if !operator.get(8) || *firing {
        if let Some(grab) = robot.grab.take() {
            grab.abort();
        }
    }

    static CRASHED: AtomicBool = AtomicBool::new(false);
    if (operator.get(6)
        || matches!(gamepad_state, GamepadState::Auto | GamepadState::Drive)
            && gamepad.left_bumper())
        && (robot
            .grab_full
            .deref()
            .try_borrow()
            .is_ok_and(|n| n.is_none())
            || CRASHED.load(std::sync::atomic::Ordering::SeqCst))
        && !operator.get(7)
        && !operator.get(5)
    {
        let robot_ = robot.clone();
        CRASHED.store(false, std::sync::atomic::Ordering::SeqCst);
        robot
            .grab_full
            .replace(Some(executor.spawn_local(async move {
                if let Err(_) = grab_full(robot_).await {
                    CRASHED.store(true, std::sync::atomic::Ordering::SeqCst);
                }
            })));
    } else if !operator.get(6) && !matches!(gamepad_state, GamepadState::Auto)
        || *firing
        || matches!(gamepad_state, GamepadState::Auto) && !gamepad.left_bumper()
    {
        if let Some(grab_full) = robot.grab_full.take() {
            grab_full.abort();
        }
    }

    *staging = robot.stage.deref().try_borrow().is_ok_and(|n| n.is_some());
    if (operator.get(7)
        || (matches!(gamepad_state, GamepadState::Auto) && gamepad.right_trigger() > 0.3))
        && !operator.get(5)
        && robot.stage.deref().try_borrow().is_ok_and(|n| n.is_none())
        && robot.shooter.try_borrow().is_ok_and(|s| !s.contains_note())
    {
        let intake = robot.intake.clone();
        let shooter = robot.shooter.clone();
        robot.stage.replace(Some(executor.spawn_local(async move {
            let intake = intake.deref().try_borrow_mut();
            let shooter = shooter.deref().try_borrow();

            if let (Ok(mut intake), Ok(shooter)) = (intake, shooter) {
                stage(&mut intake, &shooter).await;
            }
        })));
    } else if (!operator.get(7) && !matches!(gamepad_state, GamepadState::Auto))
        || *firing
        || matches!(gamepad_state, GamepadState::Auto) && gamepad.right_trigger() < 0.2
    {
        if let Some(stage) = robot.stage.take() {
            stage.abort();
        }
    }

    if operator.get(9) || (matches!(gamepad_state, GamepadState::Climb) && gamepad.a()) {
        let intake = robot.intake.clone();
        executor.spawn_local(async move {
            if let Ok(mut intake) = intake.try_borrow_mut() {
                intake.zero().await;
            }
        });
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
    shooter.set_feeder(-0.34);
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

async fn grab_full(robot: Ferris) -> anyhow::Result<()> {
    let intake = robot.intake.clone();
    let shooter = robot.shooter.clone();
    let mut intake = intake.deref().try_borrow_mut()?;
    let shooter = shooter.deref().try_borrow()?;
    lower_intake(&mut intake).await;
    intake.set_rollers(0.6);
    wait(|| intake.running()).await;
    wait(|| intake.stalled()).await;
    intake.stop_rollers();
    stage(&mut intake, &shooter).await;
    Ok(())
}
