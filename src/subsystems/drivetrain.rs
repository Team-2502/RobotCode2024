use frcrs::ctre::{ControlMode, Talon};
use frcrs::drive::{ToTalonEncoder};
use frcrs::navx::NavX;
use nalgebra::Vector2;
use uom::si::angle::degree;
use uom::si::f64::{Angle, Length};
use uom::si::length::inch;
use crate::constants::*;
use crate::swerve::kinematics::Swerve;

pub struct Drivetrain {
    navx: NavX,

    fr_drive: Talon,
    fr_turn: Talon,

    fl_drive: Talon,
    fl_turn: Talon,

    bl_drive: Talon,
    bl_turn: Talon,

    br_drive: Talon,
    br_turn: Talon,

    kinematics: Swerve,
}

impl Drivetrain {
    pub fn new() -> Self {
        Self {
            navx: NavX::new(),

            fr_drive: Talon::new(FR_DRIVE, Some("can0".to_owned())),
            fr_turn: Talon::new(FR_TURN, Some("can0".to_owned())),

            fl_drive: Talon::new(FL_DRIVE, Some("can0".to_owned())),
            fl_turn: Talon::new(FL_TURN, Some("can0".to_owned())),

            bl_drive: Talon::new(BL_DRIVE, Some("can0".to_owned())),
            bl_turn: Talon::new(BL_TURN, Some("can0".to_owned())),

            br_drive: Talon::new(BR_DRIVE, Some("can0".to_owned())),
            br_turn: Talon::new(BR_TURN, Some("can0".to_owned())),

            kinematics: Swerve::rectangle(Length::new::<inch>(25.), Length::new::<inch>(25.)),
        }
    }

    pub fn stop(&self) {
        self.fr_drive.stop();
        self.fr_turn.stop();

        self.fl_drive.stop();
        self.fl_turn.stop();

        self.bl_drive.stop();
        self.bl_turn.stop();

        self.br_drive.stop();
        self.br_turn.stop();
    }

    pub fn set_speeds(&self, fwd: f64, str: f64, rot: f64) {
        let transform = Vector2::new(fwd, str);
        let wheel_speeds = self.kinematics.calculate(transform, rot);

        self.fr_drive.set(ControlMode::Percent, wheel_speeds[0].speed);
        self.fl_drive.set(ControlMode::Percent, wheel_speeds[1].speed);
        self.bl_drive.set(ControlMode::Percent, wheel_speeds[2].speed);
        self.br_drive.set(ControlMode::Percent, wheel_speeds[3].speed);

        self.fr_turn.set(ControlMode::Position, wheel_speeds[0].angle.get::<degree>().talon_encoder_ticks());
        self.fl_turn.set(ControlMode::Position, wheel_speeds[1].angle.get::<degree>().talon_encoder_ticks());
        self.bl_turn.set(ControlMode::Position, wheel_speeds[2].angle.get::<degree>().talon_encoder_ticks());
        self.br_turn.set(ControlMode::Position, wheel_speeds[3].angle.get::<degree>().talon_encoder_ticks());
    }

    pub fn get_angle(&self) -> Angle {
        Angle::new::<degree>(self.navx.get_angle())
    }

    pub fn reset_angle(&self) {
        self.navx.reset_angle()
    }
}
