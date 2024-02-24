use std::{ops::Deref, pin::Pin, time::Duration};

use frcrs::networktables::Chooser;
use futures_lite::Future;
use tokio::{join, time::{sleep, timeout}};

use crate::{container::Ferris, subsystems::wait};

use num_derive::FromPrimitive;    
use num_traits::FromPrimitive;

#[derive(Clone, FromPrimitive)]
pub enum Auto {
    Short = 1,
    Long = 2,
    Nop = 3,
}
pub struct AutoChooser(Chooser<Auto>);

impl AutoChooser {
    pub fn new() -> Self {
        Self(Chooser::new())
    }

    pub fn add(&mut self, name: &str, auto: Auto) 
    {
        self.0.add(name, auto);
    }

    pub fn get(&self) -> Auto {
        Auto::from_i32(self.0.get()).unwrap()
    }
}

pub async fn run_auto(auto: Auto, robot: Ferris) {
    match auto {
        Auto::Short => auto_short(robot).await,
        Auto::Long => auto_long(robot).await,
        Auto::Nop => {},

    }
}

pub fn autos() -> AutoChooser {
    let mut chooser = AutoChooser::new();

    //chooser.add("flat out", Auto::Short);
    //chooser.add("crooked", Auto::Long);
    //chooser.add("tk", Auto::Long);

    chooser
}

async fn auto_long(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    println!("auto!! waowzies!!!");

}

async fn auto_short(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    shooter.set_shooter(1.0);
    intake.set_rollers(-0.1);

    if let Err(_) = timeout(Duration::from_secs_f64(1.4), shooter.load()).await {
        shooter.stop_feeder();
    };
    wait(|| shooter.get_velocity() > 5000.).await;
    intake.set_rollers(0.0);

    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(-0.0);
    shooter.set_shooter(0.0);
    drivetrain.set_speeds(-0.3, 0.0, 0.0);
    sleep(Duration::from_secs_f64(0.8)).await;
    drivetrain.set_speeds(0.0, 0.0, 0.0);
}
