use std::{ops::Deref, pin::Pin, time::Duration};

use frcrs::networktables::Chooser;
use futures_lite::Future;
use tokio::{join, time::{sleep, timeout}};

use crate::{container::Ferris, subsystems::wait};

pub struct AutoChooser(Chooser<Box<dyn Fn(Ferris) -> Box<dyn Future<Output = ()>>>>);

impl AutoChooser {
    pub fn new() -> Self {
        Self(Chooser::new())
    }

    pub fn add<A,F>(&mut self, name: &str, auto: A) 
        where A: Fn(Ferris) -> F + 'static,
              F: Future<Output = ()> + 'static,
    {
        self.0.add(name, Box::new(move |robot| Box::new(auto(robot))));
    }

    pub async fn run(&self, robot: Ferris) {
        Box::into_pin(self.0.get()(robot)).await
    }
}

pub fn autos() -> AutoChooser {
    let mut chooser = AutoChooser::new();

    chooser.add("flat out", auto_long);
    chooser.add("crooked", auto_short);

    chooser
}

async fn auto_long(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    drivetrain.set_speeds(-0.3, 0.0, 0.0);
    intake.set_rollers(-0.1);

    join!(
        async {
            if let Err(_) = timeout(Duration::from_secs_f64(1.4), shooter.load()).await {
                shooter.stop_feeder();
            };
            wait(|| shooter.get_velocity() > 5000.).await;
        },
        async {
            sleep(Duration::from_secs_f64(0.3)).await;
            drivetrain.set_speeds(0.0, 0.0, 0.0);
        },
    );
    intake.set_rollers(0.0);

    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(-0.0);
    shooter.set_shooter(0.0);
    drivetrain.set_speeds(-0.3, 0.0, 0.0);
    sleep(Duration::from_secs_f64(1.4)).await;
    drivetrain.set_speeds(0.0, 0.0, 0.0);
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
