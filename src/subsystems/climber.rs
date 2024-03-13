use frcrs::rev::{Spark, SparkMax};
use frcrs::rev::MotorType::Brushless;
use crate::constants::*;

pub struct Climber {
    left: Spark,
    right: Spark,
}

impl Climber {
    pub fn new() -> Self {
        Self {
            left: Spark::new(CLIMBER_LEFT),
            right: Spark::new(CLIMBER_RIGHT),
        }
    }

    pub fn stop_left(&self) {
        self.left.stop();
    }

    pub fn stop_right(self) {
        self.right.stop();
    }

    pub fn stop(&self) {
        self.left.stop();
        self.right.stop();
    }

    pub fn set(&self, value: f32) {
        self.left.set(-value);
        self.right.set(value);
    }

    pub fn set_left(&self, value: f32) {
        self.left.set(value);
    }

    pub fn set_right(&self, value: f32) {
        self.right.set(value);
    }
}
