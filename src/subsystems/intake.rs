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
        let left_roller = Spark::new(INTAKE_ROLLER_LEFT, MotorType::Brushless);
        let right_roller = Spark::new(INTAKE_ROLLER_RIGHT, MotorType::Brushless);

        let left_actuate = Spark::new(INTAKE_ACTUATE_LEFT, MotorType::Brushless);
        let right_actuate = Spark::new(INTAKE_ACTUATE_RIGHT, MotorType::Brushless);

        Self {
            left_roller,
            right_roller,

            left_actuate,
            right_actuate
        }
    }

    pub fn stop(&self) {
        self.left_roller.stop();
        //self.right_roller.stop();

        self.left_actuate.stop();
        //self.right_actuate.stop();
    }

    pub fn stop_actuate(&self) {
        self.left_actuate.stop();
        //self.right_actuate.stop();
    }

    pub fn stop_rollers(&self) {
        self.left_roller.stop();
        //self.right_roller.stop();
    }

    pub fn set_rollers(&self, value: f64) {
        self.left_roller.set(value);
        println!("current is {}", self.left_roller.get_current());
        //self.right_roller.set(value);
    }

    pub fn set_actuate(&self, value: f64) {
        self.left_actuate.set(value);
        //self.right_actuate.set(value);
    }

    pub fn stalled(&self) -> bool {
        self.left_roller.get_current() > intake::INTAKE_OCCUPIED_CURRENT
    }
}
