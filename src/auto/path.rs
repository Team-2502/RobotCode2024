use std::{time::Duration, f64::consts::FRAC_2_PI};

use frcrs::networktables::set_position;
use nalgebra::{Vector2, Rotation2};
use tokio::time::{Instant, sleep};
use uom::si::{f64::Time, time::second, length::meter, angle::{radian, degree}};
use wpi_trajectory::{Path, Pose};

use crate::{subsystems::Drivetrain, constants::drivetrain::SWERVE_TURN_KP};

pub async fn follow_path(drivetrain: &mut Drivetrain, path: Path) {
    let start = Instant::now();
    loop {
        let elapsed = Time::new::<second>(start.elapsed().as_secs_f64());

        let setpoint = path.get(elapsed);
        let position = Vector2::new(setpoint.x.get::<meter>(), setpoint.y.get::<meter>());
        let angle = -setpoint.heading;

        let mut error_position = position - drivetrain.odometry.position;
        let mut error_angle = (angle - drivetrain.get_angle()).get::<radian>();

        if elapsed > path.length() && error_position.abs().max() < 0.3 && error_angle.abs() < 0.2  {
            break;
        }

        error_angle *= SWERVE_TURN_KP;
        error_position *= -0.23;


        drivetrain.set_speeds(error_position.x, error_position.y, error_angle);

        set_position(drivetrain.odometry.position, -drivetrain.get_angle());

        sleep(Duration::from_millis(20)).await;
    }
}
