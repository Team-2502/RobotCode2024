use std::{time::Duration, f64::consts::FRAC_2_PI};

use frcrs::alliance_station;
use nalgebra::{Vector2, Rotation2};
use tokio::time::{Instant, sleep};
use uom::si::{f64::{Time, Angle, Length}, time::second, length::{meter, foot}, angle::{radian, degree}};
use wpi_trajectory::{Path, Pose};

use crate::{subsystems::Drivetrain, constants::drivetrain::SWERVE_TURN_KP, telemetry::{self, TelemetryStore, TELEMETRY, Data}};

pub async fn follow_path(drivetrain: &mut Drivetrain, path: Path) {
    let start = Instant::now();
    let red = alliance_station().red();
    let telemetry = TELEMETRY.clone();

    loop {
        let elapsed = Time::new::<second>(start.elapsed().as_secs_f64());

        let mut setpoint = path.get(elapsed);

        // TODO: red-blu detection
        if red {
            setpoint.y = Length::new::<foot>(54./4.) - setpoint.y;
            //setpoint.heading = Angle::new::<degree>(180.)- setpoint.heading;
            setpoint.heading = -setpoint.heading;
        }


        let position = Vector2::new(setpoint.x.get::<meter>(), setpoint.y.get::<meter>());
        let angle = -setpoint.heading;

        //drivetrain.odometry.update_from_vision(telemetry.clone(), red).await;

        let mut error_position = position - drivetrain.odometry.position;
        let mut error_angle = (angle - drivetrain.get_angle()).get::<radian>();


        if elapsed > path.length() && error_position.abs().max() < 0.3 && error_angle.abs() < 0.2  {
            break;
        }

        error_angle *= SWERVE_TURN_KP;
        error_position *= -0.23;


        drivetrain.set_speeds(error_position.x, error_position.y, error_angle);

        let mut telemetry = telemetry.write().await; 
        telemetry.data.insert("auto setpoint".to_owned(), Data::Pose(telemetry::Pose{
            x: position.x,
            y: position.y,
            theta: angle.get::<radian>(),
        }));

        telemetry.data.insert("position".to_owned(), Data::Pose(telemetry::Pose{
            x: drivetrain.odometry.position.x,
            y: drivetrain.odometry.position.y,
            theta: -drivetrain.get_angle().get::<radian>(),
        }));

        sleep(Duration::from_millis(20)).await;
    }
}
