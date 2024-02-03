use frcrs::rev::{MotorType, Spark, SparkMax};
use crate::constants::*;

pub struct Intake {
    left_roller: Spark,
    right_roller: Spark,

    left_actuate: Spark,
    right_actuate: Spark,
}

impl Intake {
    pub fn new() -> Self {
        Self {
            left_roller: Spark::new(INTAKE_ROLLER_LEFT, MotorType::Brushless),
            right_roller: Spark::new(INTAKE_ROLLER_RIGHT, MotorType::Brushless),

            left_actuate: Spark::new(INTAKE_ACTUATE_LEFT, MotorType::Brushless),
            right_actuate: Spark::new(INTAKE_ACTUATE_RIGHT, MotorType::Brushless),
        }
    }

    pub fn stop(&self) {
        self.left_roller.stop();
        self.right_roller.stop();

        self.left_actuate.stop();
        self.right_actuate.stop();
    }

    pub fn stop_actuate(&self) {
        self.left_actuate.stop();
        self.right_actuate.stop();
    }

    pub fn stop_rollers(&self) {
        self.left_roller.stop();
        self.right_roller.stop();
    }

    pub fn set_rollers(&self, value: f64) {
        self.left_roller.set(value);
        self.right_roller.set(value);
    }

    pub fn set_actuate(&self, value: f64) {
        self.left_actuate.set(value);
        self.right_actuate.set(value);
    }
}