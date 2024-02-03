use frcrs::is_teleop;
use frcrs::networktables::SmartDashboard;
use frcrs::observe_user_program_starting;
use frcrs::refresh_data;
use j4rs_derive::call_from_java;
use j4rs::Jvm;
use j4rs::prelude::*;
use frcrs::init_hal;
use frcrs::hal_report;

#[call_from_java("frc.robot.Main.rustentry")]
fn entrypoint() {

    observe_user_program_starting();

    if !init_hal() {
        panic!("Failed to init HAL")
    }

    hal_report(2, 3, 0, "2024.2.1".to_string());

    SmartDashboard::init();

    loop {
        refresh_data();

        // Todo: use `get_keyword`
        match is_teleop() {
            true => {

            }
            false => {

            }
        };
    }
}
