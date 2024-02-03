use frcrs::rev::MotorType::Brushless;
use frcrs::rev::{Spark, SparkFlex, SparkMax};
use crate::constants::*;

pub struct Shooter {
    feeder_top: Spark,
    feeder_bottom: Spark,

    shooter_top: SparkFlex,
    shooter_bottom: SparkFlex,
}

impl Shooter {
    pub fn new() -> Self {
        Self {
            feeder_top: Spark::new(SHOOTER_FEEDER_TOP, Brushless),
            feeder_bottom: Spark::new(SHOOTER_FEEDER_BOTTOM, Brushless),

            shooter_top: SparkFlex::new(SHOOTER_TOP, Brushless),
            shooter_bottom: SparkFlex::new(SHOOTER_BOTTOM, Brushless)
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

    pub fn set_shooter(&self, value: f64) {
        self.shooter_top.set(value);
        self.shooter_bottom.set(-value);
    }
}