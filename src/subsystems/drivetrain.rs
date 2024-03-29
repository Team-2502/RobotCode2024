use std::fs::File;
use std::io::{Read, Write};

use frcrs::ctre::{talon_encoder_tick, CanCoder, ControlMode, Talon};

use frcrs::navx::NavX;
use nalgebra::{Vector2, Rotation2};
use uom::si::angle::{degree, radian};
use uom::si::f64::{Angle, Length};
use uom::si::length::inch;
use crate::constants::*;
use crate::constants::drivetrain::SWERVE_ROTATIONS_TO_INCHES;
use crate::swerve::kinematics::{Swerve, ModuleState};
use crate::swerve::odometry::{ModuleReturn, Odometry};
use serde::Serialize;
use serde::Deserialize;

pub struct Drivetrain {
    navx: NavX,

    fr_drive: Talon,
    fr_turn: Talon,
    fr_encoder: CanCoder,

    fl_drive: Talon,
    fl_turn: Talon,
    fl_encoder: CanCoder,

    bl_drive: Talon,
    bl_turn: Talon,
    bl_encoder: CanCoder,

    br_drive: Talon,
    br_turn: Talon,
    br_encoder: CanCoder,

    kinematics: Swerve,
    pub odometry: Odometry,

    pub offset: Angle,

    absolute_offsets: Offsets,
}

#[derive(Serialize, Deserialize)]
struct Offsets {
    offsets: [f64; 4],
}

impl Offsets {
    const PATH: &'static str = "/home/lvuser/absolut_homosezual.json";
    fn load() -> Self {
        let mut file = File::open(Self::PATH).unwrap();
        let mut buf = String::new();
        file.read_to_string(&mut buf).unwrap();
        serde_json::from_str(&buf).unwrap_or(Self { offsets: [0.;4] })
    }
    fn store(&self) {
        let mut file = File::create(Self::PATH).unwrap();
        let buf = serde_json::to_string(&self).unwrap();
        file.write_all(buf.as_bytes()).unwrap();
    }
}

impl Drivetrain {
    pub fn new() -> Self {
        let mut absolute_offsets = Offsets::load();
        let fr_encoder = CanCoder::new(FR_ENCODER, Some("can0".to_owned()));
        let fl_encoder = CanCoder::new(FL_ENCODER, Some("can0".to_owned()));
        let bl_encoder = CanCoder::new(BL_ENCODER, Some("can0".to_owned()));
        let br_encoder = CanCoder::new(BR_ENCODER, Some("can0".to_owned()));

        let fr_turn = Talon::new(FR_TURN, Some("can0".to_owned()));
        let fl_turn = Talon::new(FL_TURN, Some("can0".to_owned()));
        let bl_turn = Talon::new(BL_TURN, Some("can0".to_owned()));
        let br_turn = Talon::new(BR_TURN, Some("can0".to_owned()));

        for (encoder, offset) in [&fr_encoder, &fl_encoder, &bl_encoder, &br_encoder].iter().zip(absolute_offsets.offsets.iter_mut()) {
            *offset -= encoder.get_absolute();
        }

        for (turn, offset) in [&fr_turn, &fl_turn, &bl_turn, &br_turn].iter().zip(absolute_offsets.offsets.iter_mut()) {
            *offset -= turn.get_position();
            *offset = 0.;
            dbg!(offset);
        }

        let dt = Self {
            navx: NavX::new(),

            fr_drive: Talon::new(FR_DRIVE, Some("can0".to_owned())),
            fr_turn,
            fr_encoder,

            fl_drive: Talon::new(FL_DRIVE, Some("can0".to_owned())),
            fl_turn,
            fl_encoder,

            bl_drive: Talon::new(BL_DRIVE, Some("can0".to_owned())),
            bl_turn,
            bl_encoder,

            br_drive: Talon::new(BR_DRIVE, Some("can0".to_owned())),
            br_turn,
            br_encoder,

            kinematics: Swerve::rectangle(Length::new::<inch>(22.5), Length::new::<inch>(23.5)),
            odometry: Odometry::new(),

            offset: Angle::new::<degree>(0.),

            absolute_offsets,
        };

        dt
    }

    pub fn write_absolute(&mut self) {
        let mut offsets = Offsets::load();
        for (encoder, offset) in [&self.fr_encoder, &self.fl_encoder, &self.bl_encoder, &self.br_encoder].iter().zip(offsets.offsets.iter_mut()) {
            *offset = encoder.get_absolute();
        }

        offsets.store();
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

    fn get_positions(&self, angles: &Vec<ModuleState>) -> Vec<ModuleReturn> {
        let mut speeds = Vec::new();

        for (module, offset) in [&self.fr_drive, &self.fl_drive, &self.bl_drive, &self.br_drive].iter().zip(angles.iter()) {
            let distance = module.get_position() * SWERVE_ROTATIONS_TO_INCHES;
            speeds.push(ModuleReturn {
                angle: offset.angle.clone(),
                distance:  Length::new::<inch>(distance),
            });
        }

        speeds
    }

    fn get_speeds(&self) -> Vec<ModuleState> {
        let mut speeds = Vec::new();

        for (module, offset) in [&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn].iter().zip(self.absolute_offsets.offsets.iter()) {
            speeds.push(ModuleState {
                speed: 0.,
                angle: Angle::new::<degree>(-module.get_position()) + Angle::new::<degree>(*offset),
            });
        }

        speeds
    }
    pub fn set_speeds(&mut self, fwd: f64, str: f64, rot: f64) {
        //println!("ODO X: {}", self.odometry.position.x);
        let mut transform = Vector2::new(str, -fwd);
        transform = Rotation2::new((self.get_angle() - self.offset).get::<radian>()) * transform;
        let wheel_speeds = self.kinematics.calculate(transform, rot);

        //self.fr_turn.set(control_mode, amount)

        //self.fr_turn.set(ControlMode::Position, (0.).talon_encoder_ticks());

        let measured = self.get_speeds();

        let positions = self.get_positions(&measured);

        let angle = self.get_angle();

        self.odometry.calculate(positions, angle);

        //println!("angle fr {}", measured[0].angle.get::<revolution>());

        let wheel_speeds: Vec<ModuleState> = wheel_speeds.into_iter().zip(measured.iter())
            .map(|(calculated,measured)| calculated.optimize(measured))
            .zip(self.absolute_offsets.offsets.iter())
            .map(|(mut state, offset)| {
                state.angle -= Angle::new::<degree>(*offset);
                state
            })
            .collect();

        self.fr_drive.set(ControlMode::Percent, wheel_speeds[0].speed);
        self.fl_drive.set(ControlMode::Percent, wheel_speeds[1].speed);
        self.bl_drive.set(ControlMode::Percent, wheel_speeds[2].speed);
        self.br_drive.set(ControlMode::Percent, wheel_speeds[3].speed);

        self.fr_turn.set(ControlMode::Position, -wheel_speeds[0].angle.get::<talon_encoder_tick>());
        self.fl_turn.set(ControlMode::Position, -wheel_speeds[1].angle.get::<talon_encoder_tick>());
        self.bl_turn.set(ControlMode::Position, -wheel_speeds[2].angle.get::<talon_encoder_tick>());
        self.br_turn.set(ControlMode::Position, -wheel_speeds[3].angle.get::<talon_encoder_tick>());
    }

    pub fn get_angle(&self) -> Angle {
        Angle::new::<degree>(self.navx.get_angle())
    }

    pub fn get_offset(&self) -> Angle {
        let mut difference = (self.get_angle() - self.offset).get::<degree>();

        difference = ( difference + 180. ) % 360. - 180.;
        if difference < -180. { difference += 360. };

        Angle::new::<degree>(difference)
    }

    pub fn reset_angle(&self) {
        self.navx.reset_angle()
    }

    pub fn reset_heading(&mut self) {
        self.offset = self.get_angle();
    }
}
