use std::{time::Duration, f64::consts::FRAC_2_PI};

use frcrs::{networktables::set_position, alliance_station};
use nalgebra::{Vector2, Rotation2};
use tokio::time::{Instant, sleep};
use uom::si::{angle::{degree, radian}, f64::{Angle, Length, Time}, length::{foot, meter}, time::{millisecond, second}, velocity::meter_per_second};
use wpi_trajectory::{Path, Pose};

use crate::{constants::drivetrain::{SWERVE_DRIVE_KF, SWERVE_DRIVE_KFA, SWERVE_DRIVE_KP, SWERVE_TURN_KP}, subsystems::Drivetrain, telemetry::{self, TelemetryStore, TELEMETRY}};

pub async fn follow_path(drivetrain: &mut Drivetrain, path: Path) {
    let start = Instant::now();
    let red = alliance_station().red();

    loop {
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

        if elapsed > path.length() && error_position.abs().max() < 0.075 && error_angle.abs() < 0.075  {
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

        drivetrain.set_speeds(speed.x, speed.y, error_angle);

        set_position(drivetrain.odometry.position, -drivetrain.get_angle());

        sleep(Duration::from_millis(20)).await;
    }
}
