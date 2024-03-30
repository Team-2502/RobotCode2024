use crate::constants::*;
use frcrs::dio::DIO;
use frcrs::rev::MotorType::Brushless;
use frcrs::rev::Spark;
use uom::si::angle::revolution;
use uom::si::f64::Angle;

use super::wait;

pub struct Shooter {
    feeder_top: Spark,
    feeder_bottom: Spark,

    shooter_top: Spark,
    shooter_bottom: Spark,

    amp_bar: Spark,

    staged: DIO,
}

impl Shooter {
    pub fn new() -> Self {
        Self {
            feeder_top: Spark::new(SHOOTER_FEEDER_TOP, Brushless),
            feeder_bottom: Spark::new(SHOOTER_FEEDER_BOTTOM, Brushless),

            shooter_top: Spark::flex(SHOOTER_TOP),
            shooter_bottom: Spark::flex(SHOOTER_BOTTOM),

            amp_bar: Spark::new(AMP_BAR, Brushless),

            staged: DIO::new(BEAM_BREAK_SIGNAL),
        }
    }

    pub fn stop_feeder(&self) {
        self.feeder_top.stop();
        self.feeder_bottom.stop();
    }

    pub fn stop_shooter(&self) {
        self.shooter_top.stop();
        self.shooter_bottom.stop();
    }

    pub fn stop(&self) {
        self.feeder_top.stop();
        self.feeder_bottom.stop();

        self.shooter_top.stop();
        self.shooter_bottom.stop();
    }

    pub fn set_feeder(&self, value: f64) {
        self.feeder_top.set(value);
        self.feeder_bottom.set(-value);
    }

    pub fn stow_amp(&mut self) {
        self.amp_bar
            .set_position(Angle::new::<revolution>(amp::STOWED_POSITION));
    }

    pub fn deploy_amp(&mut self) {
        self.amp_bar
            .set_position(Angle::new::<revolution>(amp::DEPLOYED_POSITION));
    }

    pub fn amp_deployed(&mut self) -> bool {
        self.amp_bar.get_position().get::<revolution>()
            < (amp::DEPLOYED_POSITION + amp::STOWED_POSITION) / 2.
    }

    pub fn set_amp_bar(&self, value: f64) {
        self.amp_bar.set(value);
    }

    pub fn set_shooter(&self, value: f64) {
        self.shooter_top.set(value);
        self.shooter_bottom.set(-value);
    }

    pub fn set_velocity(&mut self, value: f64) {
        self.shooter_top
            .set_reference(value, frcrs::rev::ControlType::Velocity);
        self.shooter_bottom
            .set_reference(-value, frcrs::rev::ControlType::Velocity);
    }

    pub fn contains_note(&self) -> bool {
        !self.staged.get()
    }

    pub async fn load(&self) {
        self.set_feeder(-0.2);
        wait(|| self.contains_note()).await;
        self.stop_feeder();
    }

    pub fn get_velocity(&mut self) -> f64 {
        self.shooter_top
            .get_velocity()
            .abs()
            .min(self.shooter_bottom.get_velocity().abs())
    }
}
