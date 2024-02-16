mod container;
mod subsystems;
mod constants;
mod swerve;

use std::thread;
use std::time::{Instant, Duration};

use container::Ferris;
use frcrs::ctre::{Falcon};
use frcrs::is_teleop;
use frcrs::networktables::SmartDashboard;
use frcrs::observe_user_program_starting;
use frcrs::refresh_data;
use j4rs_derive::call_from_java;
use j4rs::Jvm;
use j4rs::prelude::*;
use frcrs::init_hal;
use frcrs::hal_report;
use frcrs::input::{Joystick, RobotState};
use tokio::join;
use tokio::time::{sleep, timeout};
use crate::container::{container, stop_all};
use crate::subsystems::{Climber, Drivetrain, Intake, Shooter};
use tokio::task::{self, JoinHandle};
use std::ops::Deref;

#[call_from_java("frc.robot.Main.rustentry")]
fn entrypoint() {

    observe_user_program_starting();

    if !init_hal() {
        panic!("Failed to init HAL")
    }

    hal_report(2, 3, 0, "2024.2.1".to_string());

    SmartDashboard::init();

    let mut left_drive = Joystick::new(1);
    let mut right_drive = Joystick::new(0);
    let mut operator = Joystick::new(2);

    let mut robot = Ferris::new();

    let mut executor = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let mut auto = None;

    let mut last_loop = Instant::now();
    let controller = local.run_until(async { loop {
        refresh_data();

        let state = RobotState::get();

        if state.enabled() && state.teleop() {
            container(
                &mut left_drive,
                &mut right_drive,
                &mut operator,
                &mut robot,
                &local,
            );
        };

        if state.enabled() && state.auto() {
            if let None = auto {
                auto = Some(local.spawn_local(simple_auto(robot.clone())).abort_handle());
            }
        } else if let Some(auto) = auto.take() {
            auto.abort();
            
        };

        let elapsed = last_loop.elapsed().as_secs_f64();
        let left = (1./50. - elapsed).max(0.);
        
        sleep(Duration::from_secs_f64(left)).await;
        SmartDashboard::put_number("loop rate (hz)".to_owned(), 1./last_loop.elapsed().as_secs_f64());
        last_loop = Instant::now();
    }});
    executor.block_on(controller);
}

async fn simple_auto(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let shooter = robot.shooter.deref().borrow();

    shooter.set_shooter(0.4);
    drivetrain.set_speeds(-0.3, 0.0, 0.0);
    intake.set_rollers(-0.1);

    join!(
        async {if let Err(_) = timeout(Duration::from_secs_f64(1.4), shooter.load()).await {
            shooter.stop_feeder();
        }},
        async {
            sleep(Duration::from_secs_f64(1.5)).await;
            drivetrain.set_speeds(0.0, 0.0, 0.0);
        },
        sleep(Duration::from_secs_f64(2.0)),
    );
    intake.set_rollers(0.0);

    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(-0.0);
    shooter.set_shooter(0.0);
    //intake.grab().await;
}
