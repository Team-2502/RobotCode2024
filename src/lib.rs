mod container;
mod subsystems;
mod constants;
mod swerve;
mod auto;

use std::thread;
use std::time::{Instant, Duration};

use auto::{autos, AutoChooser};
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
use lazy_static::lazy_static;
use tokio::join;
use tokio::time::{sleep, timeout};
use crate::container::{container, stop_all};
use crate::subsystems::{wait, Climber, Drivetrain, Intake, Shooter};
use tokio::task::{self, JoinHandle};
use std::ops::Deref;
use std::rc::Rc;
use send_wrapper::SendWrapper;

static AUTOS: once_cell::sync::OnceCell<AutoChooser> = once_cell::unsync::OnceCell::new();

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

    let executor = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let mut auto = None;

    AUTOS.set(autos()).unwrap_or_else(|_| panic!("could not set"));

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
                let run = AUTOS.get().unwrap().run(robot.clone());
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
        SmartDashboard::put_number("loop rate (hz)".to_owned(), 1./last_loop.elapsed().as_secs_f64());
        last_loop = Instant::now();
    }});
    executor.block_on(controller);
}
