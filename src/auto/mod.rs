use std::{ops::{Deref, DerefMut}, pin::Pin, time::Duration};

use frcrs::networktables::Chooser;
use futures_lite::Future;
use nalgebra::Vector2;
use tokio::{join, time::{sleep, timeout}, fs::File, io::AsyncReadExt};
use wpi_trajectory::Path;

use crate::{container::{Ferris, stage}, subsystems::{wait, Intake, Shooter}};

use num_derive::FromPrimitive;    
use num_traits::FromPrimitive;

use self::path::follow_path;

pub mod path;

#[derive(Clone, FromPrimitive)]
pub enum Auto {
    Short = 1,
    PathTest = 2,
    Nop = 3,
    Top = 4,
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
        Auto::Top => top(robot).await,
        Auto::Nop => {},
        Auto::PathTest => {
            let name = "Example.1";
            let mut drivetrain = robot.drivetrain.borrow_mut();
            drive(name, &mut drivetrain).await;
        },

    }
}

async fn drive(name: &str, drivetrain: &mut crate::subsystems::Drivetrain) {
    let mut path = String::new();
    File::open(format!("/home/lvuser/deploy/choreo/{}.traj", name)).await.unwrap().read_to_string(&mut path).await.unwrap();
    let path = Path::from_trajectory(&path).unwrap();

    follow_path(drivetrain, path).await;
    drivetrain.set_speeds(0., 0., 0.)
}

pub fn autos() -> AutoChooser {
    let mut chooser = AutoChooser::new();

    //chooser.add("flat out", Auto::Short);
    //chooser.add("crooked", Auto::Long);
    //chooser.add("tk", Auto::Long);

    chooser
}

async fn top(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.odometry.position = Vector2::new(0.4550510048866272,7.067881107330322);
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    drive("Top.1", &mut drivetrain).await; // scoring position

    join!(
        async { // shoot
            wait(|| shooter.get_velocity() > 5000.).await;
            shooter.set_feeder(-0.4);
            sleep(Duration::from_secs_f64(0.3)).await;
            shooter.set_feeder(0.);
        },
        async { // lower intake
            intake.set_actuate(-0.4);
            timeout(Duration::from_millis(250), wait(|| intake.at_reverse_limit())).await.unwrap();
            intake.set_actuate(0.);
            intake.set_rollers(0.4);
        },
    );

    drive("Top.2", &mut drivetrain).await; // goto note
    intake.grab().await;
    intake.set_actuate(0.4);

    join!(
        stage(&intake, &shooter),
        drive("Top.3", &mut drivetrain) // scoring position
    );

    join!(
        shoot(&intake, &mut shooter),
        async { // lower intake
            intake.set_actuate(-0.4);
            timeout(Duration::from_millis(250), wait(|| intake.at_reverse_limit())).await.unwrap();
            intake.set_actuate(0.);
            intake.set_rollers(0.4);
        },
    );

    drive("Top.4", &mut drivetrain).await; // next note
    intake.grab().await;
    intake.set_actuate(0.4);

    join!(
        stage(&intake, &shooter),
        drive("Top.5", &mut drivetrain) // scoring position
    );
    shoot(&intake, &mut shooter).await;
}

async fn shoot(intake: &Intake, shooter: &mut Shooter) {
    wait(|| shooter.get_velocity() > 5000.).await;
    intake.set_rollers(-0.15);
    shooter.set_feeder(-0.3);
    sleep(Duration::from_secs_f64(0.4)).await;
    shooter.set_feeder(0.);
    intake.set_rollers(0.);
    shooter.set_shooter(0.);
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
