mod container;
mod subsystems;
mod constants;
mod swerve;

use std::time::Instant;

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
use frcrs::input::Joystick;
use crate::container::{container, stop_all};
use crate::subsystems::{Climber, Drivetrain, Intake, Shooter};

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

    let mut last_loop = Instant::now();
    loop {
        refresh_data();

        // Todo: use `get_keyword`
        match is_teleop() {
            true => {
                container(
                    &mut left_drive,
                    &mut right_drive,
                    &mut operator,
                    &mut robot,
                );
            }
            false => {
                stop_all(&robot);
            }
        };

        let elapsed = last_loop.elapsed().as_secs_f64();
        SmartDashboard::put_number("loop rate (hz)".to_owned(), 1./elapsed);
        last_loop = Instant::now();
    }
}
