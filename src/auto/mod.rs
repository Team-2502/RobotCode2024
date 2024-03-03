use std::{ops::{Deref, DerefMut}, pin::Pin, time::Duration};

use frcrs::networktables::Chooser;
use futures_lite::Future;
use nalgebra::Vector2;
use tokio::{join, time::{sleep, timeout}, fs::File, io::AsyncReadExt};
use uom::si::{angle::degree, f64::Angle};
use wpi_trajectory::Path;

use crate::{container::{Ferris, stage}, subsystems::{wait, Intake, Shooter}, constants::intake::{INTAKE_DOWN_GOAL, INTAKE_DOWN_THRESHOLD, INTAKE_UP_GOAL, INTAKE_UP_THRESHOLD}};

use num_derive::{FromPrimitive, ToPrimitive};    
use num_traits::FromPrimitive;

use self::path::follow_path;

pub mod path;

#[derive(Clone, FromPrimitive, ToPrimitive)]
pub enum Auto {
    Short = 1,
    PathTest = 2,
    Nop = 3,
    Top = 4,
    Center = 5,
    Bottom = 6,
}

impl Auto {
    pub fn name(&self) -> &'static str {
        match self {
            Auto::Short => "close",
            Auto::PathTest => "test",
            Auto::Nop => "hit the bell with the glock a couple times",
            Auto::Top => "near amp 4 note",
            Auto::Center => "untested riley brain vomit",
            Auto::Bottom => "internal then far (2 note)",
        }
    }
    
}

impl Default for Auto {
    fn default() -> Self {
        Auto::Nop
    }
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
        Auto::Center => center(robot).await,
        Auto::Bottom => bottom(robot).await,
        Auto::Nop => {},
        Auto::PathTest => {
            let name = "Example.1";
            let mut drivetrain = robot.drivetrain.borrow_mut();
            //drive(name, &mut drivetrain).await;
            let mut shooter = robot.shooter.borrow_mut();
            let mut intake = robot.intake.borrow_mut();
            intake.zero().await;
            stage(&mut intake, &mut shooter).await;
            //raise_intake(&mut intake).await;
            //lower_intake(&mut intake).await;
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

    drivetrain.odometry.position = Vector2::new(0.4550510048866272,(8.2296/2.)-7.067881107330322);
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    join!(
        drive("Top.1", &mut drivetrain), // scoring position
        intake.zero(),
    );

    join!(
        async { // shoot
            wait(|| shooter.get_velocity() > 5000.).await;
            shooter.set_feeder(-0.4);
            sleep(Duration::from_secs_f64(0.3)).await;
            shooter.set_feeder(0.);
        },
        lower_intake(&mut intake)
    );

    intake.set_rollers(0.4);

    let mut failure = false;
    join!(
        drive("Top.2", &mut drivetrain), // goto note
        async {
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("Top.3", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    let mut failure = false;
    join!(
        drive("Top.4", &mut drivetrain), // next note
        async {
            lower_intake(&mut intake).await;
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        },
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("Top.5", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    let mut failure = false;
    join!(
        drive("Top.6", &mut drivetrain), // goto note
        async {
            lower_intake(&mut intake).await;
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("Top.7", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    shooter.set_shooter(0.);
}

async fn 3Note(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.odometry.position = Vector2::new(0.4550510048866272,(8.2296/2.)-7.067881107330322);
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    join!(
        drive("3Note.1", &mut drivetrain), // scoring position
        intake.zero(),
    );

    join!(
        async { // shoot
            wait(|| shooter.get_velocity() > 5000.).await;
            shooter.set_feeder(-0.4);
            sleep(Duration::from_secs_f64(0.3)).await;
            shooter.set_feeder(0.);
        },
        lower_intake(&mut intake)
    );

    intake.set_rollers(0.4);

    let mut failure = false;
    join!(
        drive("3Note.2", &mut drivetrain), // goto note
        async {
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("3Note.3", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    let mut failure = false;
    join!(
        drive("3Note.4", &mut drivetrain), // next note
        async {
            lower_intake(&mut intake).await;
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        },
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("3Note.5", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    let mut failure = false;
    join!(
        drive("3Note.6", &mut drivetrain), // goto note
        async {
            lower_intake(&mut intake).await;
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("3Note.7", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    shooter.set_shooter(0.);
}

async fn shoot(intake: &Intake, shooter: &mut Shooter) {
    wait(|| shooter.get_velocity() > 5000.).await;
    intake.set_rollers(-0.15);
    shooter.set_feeder(-0.3);
    sleep(Duration::from_secs_f64(0.4)).await;
    shooter.set_feeder(0.);
    intake.set_rollers(0.);
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

async fn center(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.odometry.position = Vector2::new(0.4550510048866272,7.067881107330322);
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    drive("4_Note_Center.1", &mut drivetrain).await; // scoring position

    join!(
        async { // shoot
            wait(|| shooter.get_velocity() > 5000.).await;
            shooter.set_feeder(-0.4);
            sleep(Duration::from_secs_f64(0.3)).await;
            shooter.set_feeder(0.);
        },
        async { // lower intake
            intake.set_actuate(-0.3);
            let _ = timeout(Duration::from_millis(1720), wait(|| intake.at_reverse_limit())).await;
            intake.set_actuate(0.);
            intake.set_rollers(0.4);
        },
    );

    let mut failure = false;
    join!(
        drive("Top.2", &mut drivetrain), // goto note
        async {
            failure = timeout(Duration::from_millis(1000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("Top.3", &mut drivetrain) // scoring position
    );

    join!(
        shoot(&intake, &mut shooter),
        async { // lower intake
            intake.set_actuate(-0.3);
            let _ = timeout(Duration::from_millis(1720), wait(|| intake.at_reverse_limit())).await;
            intake.set_actuate(0.);
            intake.set_rollers(0.4);
        },
    );

    let mut failure = false;
    join!(
        drive("Top.4", &mut drivetrain), // next note
        async {
            failure = timeout(Duration::from_millis(1000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("Top.5", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;
    shooter.set_shooter(0.);
}

async fn bottom(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.odometry.position = Vector2::new(0.399,(8.2296/2.)-4.098);
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    join!(
        drive("Bottom.1", &mut drivetrain), // scoring position
        intake.zero(),
    );

    // shoot
    wait(|| shooter.get_velocity() > 5000.).await;
    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(0.);

    let mut failure = false;
    join!(
        drive("Bottom.2", &mut drivetrain), // goto note
        async {
            sleep(Duration::from_millis(1750)).await;
            lower_intake(&mut intake).await;
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("Bottom.3", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;
    shooter.set_shooter(0.);
}

pub async fn lower_intake(intake: &mut Intake) {
    intake.actuate_to(Angle::new::<degree>(INTAKE_DOWN_GOAL));
    wait(|| intake.actuate_position().get::<degree>() < INTAKE_DOWN_THRESHOLD).await;
}

pub async fn raise_intake(intake: &mut Intake) {
    intake.actuate_to(Angle::new::<degree>(INTAKE_UP_GOAL));
    wait(|| intake.actuate_position().get::<degree>() > INTAKE_UP_THRESHOLD).await;
}
