#![feature(variant_count)]

mod input;
mod subsystems;
pub mod constants;
mod swerve;
mod auto;
pub mod telemetry;


use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::sync::{Arc, Mutex};
use std::time::{Instant, Duration};

use auto::{autos, run_auto, Auto};
use constants::TELEMETRY_PORT;
use input::{Controllers, Ferris, GamepadState};

use frcrs::{alliance_station};
use frcrs::networktables::{set_position, SmartDashboard};
use frcrs::observe_user_program_starting;
use frcrs::refresh_data;
use j4rs_derive::call_from_java;
use j4rs::Jvm;
use j4rs::prelude::*;
use frcrs::init_hal;
use frcrs::hal_report;
use frcrs::input::{Gamepad, Joystick, RobotState};

use num_traits::{FromPrimitive, ToPrimitive};

use telemetry::{Data, TELEMETRY};

use tokio::time::sleep;
use crate::input::container;

use tokio::task::{self};
use std::ops::Deref;

//pub extern "system" fn entrypoint <'local>(mut env: JNIEnv<'local>, class: JClass<'local>) {

pub fn entrypoint() {
    let executor = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let controller = local.run_until(async { 

        if !init_hal() {
            panic!("Failed to init HAL")
        }

        hal_report(2, 3, 0, "2024.2.1".to_string());

        let left_drive = Joystick::new(1);
        let right_drive = Joystick::new(0);
        let operator = Joystick::new(2);
        let gamepad = Gamepad::new(3);
        let mut controllers = Controllers { left_drive, right_drive, operator, gamepad, gamepad_state: GamepadState::Auto };

        let mut robot = Ferris::new();
        observe_user_program_starting();

        let router = telemetry::server()
            .with_state(robot.telemetry.clone());

        executor.spawn(async move {
            let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{}", TELEMETRY_PORT)).await.unwrap();
            axum::serve(listener, router).await.unwrap();
        }).abort_handle();

    let mut auto = None;

    let mut last_loop = Instant::now();
        loop {
        
        refresh_data();

        let state = RobotState::get();

        if state.enabled() && state.teleop() && !state.test() {
            container(
                &mut controllers,
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

        //if let Ok(mut drivetrain) = robot.drivetrain.try_borrow_mut() {
        //    let red = alliance_station().red();
        //    drivetrain.odometry.update_from_vision(TELEMETRY.clone(), red).await;
        //    let angle = drivetrain.get_angle();
        //    set_position(drivetrain.odometry.position, -angle);
        //}
        
        //sleep(Duration::from_secs_f64(left)).await;
        telemetry::put_number("loop rate (hz)", 1./last_loop.elapsed().as_secs_f64()).await;
        last_loop = Instant::now();
        //println!("hz {}", 1./last_loop.elapsed().as_secs_f64());
    }});

    executor.block_on(controller);
}
