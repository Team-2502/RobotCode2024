#![feature(variant_count)]

mod container;
mod subsystems;
pub mod constants;
mod swerve;
mod auto;
pub mod telemetry;

use std::thread;
use std::time::{Instant, Duration};

use auto::{run_auto, Auto};
use constants::TELEMETRY_PORT;
use container::Ferris;
use frcrs::observe_user_program_starting;
use frcrs::refresh_data;
use frcrs::init_hal;
use frcrs::hal_report;
use frcrs::input::{Joystick, RobotState};
use lazy_static::lazy_static;
use num_traits::{FromPrimitive, ToPrimitive};
use once_cell::sync::OnceCell;
use telemetry::Data;
use tokio::join;
use tokio::time::{sleep, timeout};
use crate::container::{container, stop_all};
use crate::subsystems::{wait, Climber, Drivetrain, Intake, Shooter};
use tokio::task::{self, JoinHandle};
use std::ops::Deref;
use std::rc::Rc;
use send_wrapper::SendWrapper;


pub fn entrypoint() {

    observe_user_program_starting();

    if !init_hal() {
        panic!("Failed to init HAL")
    }

    const RUST: i32 = 7; // https://github.com/wpilibsuite/allwpilib/pull/6227
                         // TODO: bother national instruments
    hal_report(2, RUST, 0, "2024.2.1".to_string());

    let mut left_drive = Joystick::new(1);
    let mut right_drive = Joystick::new(0);
    let mut operator = Joystick::new(2);

    let mut robot = Ferris::new();

    let router = telemetry::server()
        .with_state(robot.telemetry.clone());

    let executor = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let mut auto = None;

    let mut last_loop = Instant::now();
    let controller = local.run_until(async { loop {
        refresh_data();

        let state = RobotState::get();

        if state.enabled() && state.teleop() && !state.test() {
            container(
                &mut left_drive,
                &mut right_drive,
                &mut operator,
                &mut robot,
                &local,
            ).await;
        };

        if state.enabled() && state.auto() {
            if let None = auto {
                let robot = robot.clone();

                let chosen = if let Data::Picker(picker) = robot.telemetry.read().await.data.get("auto chooser").unwrap() {
                    picker.selected.parse().unwrap()
                } else {
                    println!("auto chooser not found");
                    Auto::default().to_usize().unwrap()
                    
                };

                let chosen = Auto::from_usize(chosen).unwrap();

                let run = run_auto(chosen, robot);
                auto = Some(local.spawn_local(run).abort_handle());
                //auto = Some(local.spawn_local(auto_long(robot.clone())).abort_handle());
            }
        } else if let Some(auto) = auto.take() {
            auto.abort();
        };

        if state.test() && state.enabled() {
            robot.drivetrain.deref().borrow_mut().write_absolute();
        }


        let elapsed = last_loop.elapsed().as_secs_f64();
        let left = (1./50. - elapsed).max(0.);
        
        sleep(Duration::from_secs_f64(left)).await;
        telemetry::put_number("loop rate (hz)", 1./last_loop.elapsed().as_secs_f64()).await;
        last_loop = Instant::now();
    }});

    executor.spawn(async {
        let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{}", TELEMETRY_PORT)).await.unwrap();
        axum::serve(listener, router).await.unwrap();
    }).abort_handle();

    executor.block_on(controller);
}
