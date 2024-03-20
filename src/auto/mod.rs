use std::{ops::{Deref, DerefMut}, pin::Pin, time::Duration, mem};

use frcrs::networktables::Chooser;
use futures_lite::Future;
use nalgebra::Vector2;
use tokio::{join, time::{sleep, timeout}, fs::File, io::AsyncReadExt};
use uom::si::{angle::degree, f64::Angle};
use wpi_trajectory::Path;

use crate::{constants::intake::{INTAKE_DOWN_GOAL, INTAKE_DOWN_THRESHOLD, INTAKE_UP_GOAL, INTAKE_UP_THRESHOLD}, container::{stage, Ferris}, subsystems::{wait, Intake, Shooter}, telemetry::{self, Picker, TelemetryStore}};

use num_derive::{FromPrimitive, ToPrimitive};    
use num_traits::{FromPrimitive, ToPrimitive};

use self::path::follow_path;

pub mod path;

#[derive(Clone, FromPrimitive, ToPrimitive)]
pub enum Auto {
    TopBlock,
    BottomCloseWait,
    TopStop,
    BottomClose,
    TopWait,
    SourceTwo,
    ZeroIntake,
    Short,
    PathTest,
    Bottom,
    Center,
    Top,
    TopCenter,
    BottomOut,
    BottomWait,
    StageCloseFar,
    OdoTest,
    Nop,
    Nop2,
    Nop3,
    Nop4,
}

impl Auto {
    pub fn name(&self) -> &'static str {
        match self {
            Auto::Short => "close",
            Auto::PathTest => "test",
            Auto::Nop => "hit the bell with the glock a couple times",
            Auto::Top => "near amp 4 note, swing b4 last",
            Auto::TopStop => "near amp 4 note, stop b4 last",
            Auto::Center => "untested riley brain vomit",
            Auto::Bottom => "internal then far (2 note)",
            Auto::ZeroIntake => "zero intake rotation",
            Auto::TopCenter => "anish triple (internal then far)",
            Auto::BottomOut => "internal then far not through stage (2 note)",
            Auto::BottomClose => "near stage start 4 note",
            Auto::BottomCloseWait => "near stage start 2 note, delayed start",
            Auto::SourceTwo => "near source start 2 note, second from bottom",
            Auto::BottomWait => "near stage wait 7s one note",
            Auto::TopWait => "near amp wait 7s one note",
            Auto::TopBlock => "near amp wait one note stop mid",
            Auto::StageCloseFar => "Stage close far",
            Auto::OdoTest => "Odo Test",
            _ => "shoot a chicken",
        }
    }

    pub fn len() -> usize {
        mem::variant_count::<Self>()
    }

    pub fn names() -> Vec<String> {
        (0..Self::len()).map(|n| Self::from_usize(n).unwrap().name().to_owned()).collect()
    }

    pub fn picker() -> Picker {
        Picker {
            options: Auto::names(),
            selected: Auto::default().to_usize().unwrap().to_string(),
        }
    }
}

impl Default for Auto {
    fn default() -> Self {
        Auto::ZeroIntake
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
        Auto::TopStop => top_stop(robot).await,
        Auto::Center => center(robot).await,
        Auto::Bottom => bottom(robot).await,
        Auto::BottomOut => bottom_out(robot).await,
        Auto::BottomClose => bottom_close(robot).await,
        Auto::BottomCloseWait => bottom_close_wait(robot).await,
        Auto::BottomWait => bottom_one(robot).await,
        Auto::TopWait => top_one(robot).await,
        Auto::TopBlock => top_one_block(robot).await,
        Auto::TopCenter => Triple_Note(robot).await,
        Auto::SourceTwo => source_out(robot).await,
        Auto::StageCloseFar => stage_close_far(robot).await,
        Auto::OdoTest => odo_test(robot).await,
        Auto::PathTest => {
            let name = "Example.1";
            let mut drivetrain = robot.drivetrain.borrow_mut();
            //drive(name, &mut drivetrain, telemetry.clone()).await;
            let mut shooter = robot.shooter.borrow_mut();
            let mut intake = robot.intake.borrow_mut();
            intake.zero().await;
            stage(&mut intake, &mut shooter).await;
            //raise_intake(&mut intake).await;
            //lower_intake(&mut intake).await;
        },
        Auto::ZeroIntake => {
            let mut intake = robot.intake.borrow_mut();
            intake.zero().await;
        },
        _ => {},
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


async fn top_stop(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.469,7.034497));
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
        async {
            drive("Top.6", &mut drivetrain).await;
            drive("Top.7", &mut drivetrain).await; // goto note
        },
        async {
            lower_intake(&mut intake).await;
            failure = timeout(Duration::from_millis(5000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("Top.8", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    shooter.set_shooter(0.);
}

async fn top(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.469,7.034497));
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

async fn source_out(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.4719328284263611,2.0507166385650635));
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    join!(
        drive("SourceTwo.1", &mut drivetrain), // scoring position
        intake.zero(),
    );

    // shoot
    wait(|| shooter.get_velocity() > 5000.).await;
    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(0.);

    let mut failure = false;
    join!(
        drive("SourceTwo.2", &mut drivetrain), // goto note
        async {
            sleep(Duration::from_millis(1500)).await;
            lower_intake(&mut intake).await;
            intake.set_rollers(0.4);
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("SourceTwo.3", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    shooter.set_shooter(0.);
}

async fn Triple_Note(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.4550510048866272,7.067881107330322));
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
    let telemetry = robot.telemetry.clone();

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
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.4550510048866272,7.067881107330322));
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
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.399,4.098));
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

async fn bottom_out(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.467,4.044));
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    join!(
        drive("BottomOut.1", &mut drivetrain), // scoring position
        intake.zero(),
    );

    // shoot
    wait(|| shooter.get_velocity() > 5000.).await;
    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(0.);

    let mut failure = false;
    join!(
        drive("BottomOut.2", &mut drivetrain), // goto note
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
        drive("BottomOut.3", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;
    shooter.set_shooter(0.);
}

async fn bottom_close(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.4808354377746582,4.043473720550537));
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);
    join!(
        drive("BottomClose.1", &mut drivetrain), // scoring position
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
        drive("BottomClose.2", &mut drivetrain), // goto note
        async {
            failure = timeout(Duration::from_millis(2500), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2000),stage(&mut intake, &shooter)),
        drive("BottomClose.3", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    let mut failure = false;
    join!(
        drive("BottomClose.4", &mut drivetrain), // next note
        async {
            lower_intake(&mut intake).await;
            failure = timeout(Duration::from_millis(2500), intake.grab()).await.is_err();
        },
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("BottomClose.5", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    let mut failure = false;

    join!(
        drive("BottomClose.6", &mut drivetrain), // goto note
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
        drive("BottomClose.7", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    shooter.set_shooter(0.);
}
async fn bottom_one(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.4808354377746582,4.043473720550537));
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    sleep(Duration::from_millis(7000)).await;

    shooter.set_shooter(1.0);
    join!(
        drive("BottomOne.1", &mut drivetrain), // scoring position
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

    shooter.set_shooter(0.);
    drive("BottomOne.2", &mut drivetrain).await;
}
async fn top_one(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.odometry.set(Vector2::new(0.46920153498649597,7.0344977378845215));
    drivetrain.reset_angle();
    drivetrain.reset_heading();


    shooter.set_shooter(1.0);
    join!(
        intake.zero(),
        sleep(Duration::from_millis(9000)),
    );

    drive("TopOne.1", &mut drivetrain).await; // scoring position

    // shoot
    wait(|| shooter.get_velocity() > 5000.).await;
    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(0.);

    shooter.set_shooter(0.);
    drive("TopOne.2", &mut drivetrain).await;
}

async fn top_one_block(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.odometry.set(Vector2::new(0.46920153498649597,7.0344977378845215));
    drivetrain.reset_angle();
    drivetrain.reset_heading();


    shooter.set_shooter(1.0);
    join!(
        intake.zero(),
        sleep(Duration::from_millis(6000)),
    );

    drive("TopOneBlock.1", &mut drivetrain).await; // scoring position

    // shoot
    wait(|| shooter.get_velocity() > 5000.).await;
    shooter.set_feeder(-0.4);
    sleep(Duration::from_secs_f64(0.3)).await;
    shooter.set_feeder(0.);

    shooter.set_shooter(0.);
    drive("TopOneBlock.2", &mut drivetrain).await;
}

async fn bottom_close_wait(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();

    drivetrain.odometry.set(Vector2::new(0.4808354377746582,4.043473720550537));
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);

    sleep(Duration::from_millis(4000)).await;
    join!(
        drive("BottomCloseWait.1", &mut drivetrain), // scoring position
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
        drive("BottomCloseWait.2", &mut drivetrain), // goto note
        async {
            failure = timeout(Duration::from_millis(3000), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(2500),stage(&mut intake, &shooter)),
        drive("BottomCloseWait.3", &mut drivetrain) // scoring position
    );

    shoot(&intake, &mut shooter).await;

    shooter.set_shooter(0.);
}

async fn stage_close_far(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(0.469, 4.09));
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    shooter.set_shooter(1.0);

    join!(
        drive("StageCloseFar.1", &mut drivetrain), // Shooting first
        intake.zero()
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

    intake.set_rollers(0.75);

    drive("StageCloseFar.2", &mut drivetrain).await; // rotate so straight

    let mut failure = false;
    join!(
        drive("StageCloseFar.3", &mut drivetrain), // drive to note
        async {
            failure = timeout(Duration::from_millis(2500), intake.grab()).await.is_err();
        }
    );

    if failure {
        println!("womp womp :(");
    }

    let _ = join!(
        timeout(Duration::from_millis(5000), stage(&mut intake, &shooter)),
        drive("StageCloseFar.4", &mut drivetrain) // Shooting second
    );

    shoot(&intake, &mut shooter).await;

    shooter.set_shooter(0.);

    /*drive("StageCloseFar.6", &mut drivetrain).await; // rotate
    drive("StageCloseFar.7", &mut drivetrain).await; // drive sideways*/
}

async fn odo_test(robot: Ferris) {
    let mut intake = robot.intake.deref().borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut shooter = robot.shooter.deref().borrow_mut();
    let telemetry = robot.telemetry.clone();

    drivetrain.odometry.set(Vector2::new(1.382, 5.572));
    drivetrain.reset_angle();
    drivetrain.reset_heading();

    drive("OdoTest.1", &mut drivetrain).await;
    drive("OdoTest.2", &mut drivetrain).await;
}