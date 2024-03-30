use std::{time::Duration};

use frcrs::{networktables::set_position, alliance_station};
use nalgebra::{Vector2};
use tokio::time::{Instant, sleep};
use uom::si::{angle::{radian}, f64::{Length, Time}, length::{foot, meter}, time::{millisecond, second}, velocity::meter_per_second};
use wpi_trajectory::{Path};

use crate::{constants::drivetrain::{SWERVE_DRIVE_IE, SWERVE_DRIVE_KD, SWERVE_DRIVE_KF, SWERVE_DRIVE_KFA, SWERVE_DRIVE_KI, SWERVE_DRIVE_KP, SWERVE_DRIVE_MAX_ERR, SWERVE_TURN_KP}, subsystems::Drivetrain};

pub async fn follow_path(drivetrain: &mut Drivetrain, path: Path) {
    follow_path_range(drivetrain, path, SWERVE_DRIVE_MAX_ERR).await
}
pub async fn follow_path_range(drivetrain: &mut Drivetrain, path: Path, max_err: f64) {
    let start = Instant::now();
    let red = alliance_station().red();

    let mut last_error = Vector2::zeros(); // TODO: delta t
    let mut last_loop = Instant::now();
    let mut i = Vector2::zeros();

    loop {
        let now = Instant::now();
        let dt = now-last_loop;
        last_loop = now;

        let elapsed = Time::new::<second>(start.elapsed().as_secs_f64());

        let mut setpoint = path.get(elapsed);
        let mut setpoint_next = path.get(elapsed + Time::new::<millisecond>(20.)); // bodge 

        // TODO: red-blu detection
        if red {
            for setpoint in [&mut setpoint, &mut setpoint_next] {
                setpoint.y = Length::new::<foot>(54./4.) - setpoint.y;
                setpoint.velocity_y = -setpoint.velocity_y;
                //setpoint.heading = Angle::new::<degree>(180.)- setpoint.heading;
                setpoint.heading = -setpoint.heading;
            }
        }

        let position = Vector2::new(setpoint.x.get::<meter>(), setpoint.y.get::<meter>());
        let angle = -setpoint.heading;

        //drivetrain.odometry.update_from_vision(telemetry.clone(), red).await;

        let mut error_position = position - drivetrain.odometry.position;
        let mut error_angle = (angle - drivetrain.get_angle()).get::<radian>();

        if error_position.abs().max() < SWERVE_DRIVE_IE {
            i += error_position;
        }

        if elapsed > path.length() && error_position.abs().max() < max_err && error_angle.abs() < 0.075  {
            break;
        }

        error_angle *= SWERVE_TURN_KP;
        error_position *= -SWERVE_DRIVE_KP;

        let mut speed = error_position;

        let velocity = Vector2::new(setpoint.velocity_x, setpoint.velocity_y);
        let velocity = velocity.map(|x| x.get::<meter_per_second>());

        let velocity_next = Vector2::new(setpoint.velocity_x, setpoint.velocity_y).map(|x| x.get::<meter_per_second>());
        
        let acceleration = (velocity_next - velocity) * 1000./20.;

        speed += velocity * -SWERVE_DRIVE_KF;
        speed += acceleration * -SWERVE_DRIVE_KFA;
        speed += i * -SWERVE_DRIVE_KI * dt.as_secs_f64() * 9.;

        let speed_s = speed;
        speed += (speed - last_error) * -SWERVE_DRIVE_KD * dt.as_secs_f64() * 9.;
        last_error =  speed_s;

        drivetrain.set_speeds(speed.x, speed.y, error_angle);

        //set_position(drivetrain.odometry.position, -drivetrain.get_angle());

        sleep(Duration::from_millis(20)).await;
    }
}
