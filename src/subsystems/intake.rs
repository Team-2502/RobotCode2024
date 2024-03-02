use std::time::Duration;

use frcrs::{rev::{MotorType, Spark, SparkMax}, dio::DIO};
use tokio::time::sleep;
use uom::si::{angle::degree, f64::Angle};
use crate::constants::*;

pub struct Intake {
    left_roller: Spark,
    right_roller: Spark,

    left_actuate: Spark,
    right_actuate: Spark,

    limit: DIO,
    reverse_limit: DIO,

    actuate_zero: Angle,
}

const COUNTS_PER_REVOLUTION: f64 = 41.6;

impl Intake {
    pub fn new() -> Self {
        let left_roller = Spark::new(INTAKE_ROLLER_LEFT, MotorType::Brushless);
        let right_roller = Spark::new(INTAKE_ROLLER_RIGHT, MotorType::Brushless);

        let left_actuate = Spark::new(INTAKE_ACTUATE_LEFT, MotorType::Brushless);
        let pid = left_actuate.get_pid();
        pid.set_p(0.08);
        pid.set_d(0.45);

        let right_actuate = Spark::new(INTAKE_ACTUATE_RIGHT, MotorType::Brushless);

        let limit = DIO::new(INTAKE_LIMIT);
        let reverse_limit = DIO::new(INTAKE_DOWN_LIMIT);

        Self {
            left_roller,
            right_roller,

            left_actuate,
            right_actuate,

            limit,
            reverse_limit,

            actuate_zero: Angle::new::<degree>(0.),
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
        //self.right_roller.set(value);
    }

    pub fn set_actuate(&self, value: f64) {
        self.left_actuate.set(value);
        //self.right_actuate.set(value);
    }

    pub fn stalled(&mut self) -> bool {
        self.left_roller.get_current() > intake::INTAKE_OCCUPIED_CURRENT &&
            self.left_roller.get_velocity() < intake::INTAKE_OCCUPIED_VELOCITY
    }

    /// finished accelerating
    pub fn running(&mut self) -> bool {
        self.left_roller.get_current() < intake::INTAKE_OCCUPIED_CURRENT &&
            self.left_roller.get_velocity() > intake::INTAKE_FREE_VELOCITY
    }

    pub fn at_reverse_limit(&self) -> bool {
        !self.reverse_limit.get()
    }

    pub fn at_limit(&self) -> bool {
        !self.limit.get()
    }

    pub fn actuate_position(&mut self) -> Angle {
        (self.left_actuate.get_position() - self.actuate_zero) / COUNTS_PER_REVOLUTION
    }

    pub fn constrained(&self) -> bool {
        self.at_limit() || self.at_reverse_limit()
    }

    pub async fn grab(&mut self) {
        self.set_rollers(0.4);
        wait(|| self.running()).await;
        wait(|| self.stalled()).await;
        self.stop_rollers();
    }

    pub async fn zero(&mut self) {
        self.set_actuate(0.3);
        wait(|| self.at_limit()).await;
        self.set_actuate(0.);
        sleep(Duration::from_millis(750)).await;
        wait(|| self.at_limit()).await;
        self.set_actuate(-0.15);
        wait(|| !self.at_limit()).await;
        self.set_actuate(0.);
        self.actuate_zero = self.left_actuate.get_position();
    }

    /// 0deg is stowed
    /// 180deg is out
    pub fn actuate_to(&self, angle: Angle) {
        self.left_actuate.set_position(angle * COUNTS_PER_REVOLUTION + self.actuate_zero)
    }
}

pub async fn wait<F>(mut condition: F) 
    where F: FnMut() -> bool {
        loop {
            if condition() { return };
            sleep(Duration::from_millis(20)).await;
        }
}
