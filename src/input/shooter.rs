
use frcrs::{alliance_station, deadzone};
use uom::si::{angle::{degree, radian}, f64::Angle};

use crate::{constants::drivetrain::{PODIUM_SHOT_ANGLE, SWERVE_TURN_KP}, subsystems::{Drivetrain, Shooter}, telemetry};

use super::Controllers;

#[derive(Default)]
pub struct ShooterControlState {
    saved_angle: Option<Angle>
}

pub async fn control_shooter(drivetrain: &mut Shooter, controllers: &mut Controllers, state: &mut ShooterControlState) {
    telemetry::put_number("flywheel speed", shooter.get_velocity()).await;
    telemetry::put_bool("beam break: {}", shooter.contains_note()).await;

    if *shooting {
        if shooter.amp_deployed() && !operator.get(5) {
            shooter.set_shooter(0.225)
        } else if right_drive.get(2) {
            shooter.set_velocity(1917.)
        } else {
            shooter.set_shooter((operator.get_throttle() + 1.) / 2.);
        }
    } else {
        shooter.stop_shooter();
    }

    if operator.get(5) {
        if operator.get(11) {
            shooter.set_amp_bar(-0.6);
        } else if operator.get(16) {
            shooter.set_amp_bar(0.6);
        } else {
            shooter.set_amp_bar(0.);
        }
    } else {
        if operator.get(11) {
            shooter.deploy_amp();
        } else if operator.get(16) {
            shooter.stow_amp();
        }
    }

    if !staging {
        if firing {
            shooter.set_feeder(-1.);
        } else if operator.get(10) {
            shooter.set_feeder(0.5);
        } else {
            shooter.stop_feeder();
        }
    }
}
