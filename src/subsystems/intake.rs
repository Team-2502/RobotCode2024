use std::{sync::atomic::{AtomicI64, Ordering}, time::Duration};

use crate::{constants::*, subsystems::intake::intake::{INTAKE_DOWN_GOAL, INTAKE_UP_GOAL}};
use frcrs::{
    dio::DIO,
    rev::{MotorType, Spark},
};
use tokio::time::sleep;
use uom::si::{angle::degree, f64::Angle};

use self::intake::{INTAKE_DEGREES_PER_SECOND, INTAKE_ZERO_POINT};

pub struct Intake {
    left_roller: Spark,
    right_roller: Spark,

    left_actuate: Spark,
    right_actuate: Spark,

    limit: DIO,
    cam_limit: DIO,

    actuate_zero: Angle,
}

const COUNTS_PER_REVOLUTION: f64 = 41.6;

impl Intake {
    pub fn new() -> Self {
        let left_roller = Spark::new(INTAKE_ROLLER_LEFT, MotorType::Brushless);
        let right_roller = Spark::new(INTAKE_ROLLER_RIGHT, MotorType::Brushless);

        let mut left_actuate = Spark::new(INTAKE_ACTUATE_LEFT, MotorType::Brushless);
        let pid = left_actuate.get_pid();
        pid.set_p(0.08);
        pid.set_d(0.45);

        let right_actuate = Spark::new(INTAKE_ACTUATE_RIGHT, MotorType::Brushless);

        let limit = DIO::new(INTAKE_LIMIT);
        let cam_limit = DIO::new(INTAKE_CAM_LIMIT);

        Self {
            left_roller,
            right_roller,

            left_actuate,
            right_actuate,

            limit,
            cam_limit,

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

    pub fn roller_current(&mut self) -> f64 {
        self.left_roller.get_current()
    }

    pub fn stalled(&mut self) -> bool {
        self.left_roller.get_current() > intake::INTAKE_OCCUPIED_CURRENT
            && self.left_roller.get_velocity() < intake::INTAKE_OCCUPIED_VELOCITY
    }

    /// finished accelerating
    pub fn running(&mut self) -> bool {
        self.left_roller.get_current() < intake::INTAKE_OCCUPIED_CURRENT
            && self.left_roller.get_velocity() > intake::INTAKE_FREE_VELOCITY
    }

    pub fn cam_limit(&self) -> bool {
        !self.cam_limit.get()
    }

    pub fn at_limit(&self) -> bool {
        !self.limit.get()
    }

    pub fn actuate_position(&mut self) -> Angle {
        (self.left_actuate.get_position() - self.actuate_zero) / COUNTS_PER_REVOLUTION
    }

    pub fn constrained(&self) -> bool {
        self.at_limit()
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
        self.actuate_zero =
            self.left_actuate.get_position() - Angle::new::<degree>(INTAKE_ZERO_POINT);
    }

    /// 0deg is stowed
    /// 180deg is out
    pub fn actuate_to(&mut self, angle: Angle) {
        self.left_actuate
            .set_position(angle * COUNTS_PER_REVOLUTION + self.actuate_zero)
    }


    /// 0deg is stowed
    /// 180deg is out
    pub fn actuate_to_trapezoid_rdy(&mut self, angle: Angle, dt: &Duration) -> bool {
        // bodge
        static ANGLE: AtomicI64 = AtomicI64::new((0) as i64 * 100);

        let current = ANGLE.load(Ordering::SeqCst) as f64 / 100.;
        let max = dt.as_secs_f64() * INTAKE_DEGREES_PER_SECOND * 6.;
        let goal = angle.get::<degree>();

        let compromise = if goal > current {
            goal.min(current+max)
        } else {
            goal.max(current-max)
        };

        ANGLE.store((compromise * 100.) as i64, Ordering::SeqCst);

        let compromise = Angle::new::<degree>(compromise);
        self.left_actuate.set_position(compromise * COUNTS_PER_REVOLUTION + self.actuate_zero);

        goal == current
    }

    /// 0deg is stowed
    /// 180deg is out
    pub fn actuate_to_trapezoid(&mut self, angle: Angle, dt: &Duration) {
        self.actuate_to_trapezoid_rdy(angle, dt);
    }
}

pub async fn wait<F>(mut condition: F)
where
    F: FnMut() -> bool,
{
    loop {
        if condition() {
            return;
        };
        sleep(Duration::from_millis(20)).await;
    }
}
