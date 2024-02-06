
/*use crate::{Encoder, Motor};

pub struct Motors {
    fl_drive: dyn Motor,
    fl_turn: dyn Motor,
    fr_drive: dyn Motor,
    fr_turn: dyn Motor,
    bl_drive: dyn Motor,
    bl_turn: dyn Motor,
    br_drive: dyn Motor,
    br_turn: dyn Motor,
}

pub struct Encoders {
    fl: dyn Encoder,
    fr: dyn Encoder,
    bl: dyn Encoder,
    br: dyn Encoder,
}

impl Encoders {
    pub fn new(fl: Box<dyn Encoder>, fr: Box<dyn Encoder>, bl: Box<dyn Encoder>, br: Box<dyn Encoder>) -> Box<Self> {
        Box::new(Self {
            fl,
            fr,
            bl,
            br
        })
    }
}

impl Motors {
    pub fn new(fl_drive: Box<dyn Motor>, fr_drive: Box<dyn Motor>, bl_drive: Box<dyn Motor>, br_drive: Box<dyn Motor>,
               fl_turn: Box<dyn Motor>, fr_turn: Box<dyn Motor>, bl_turn: Box<dyn Motor>, br_turn: Box<dyn Motor>) -> Box<Self> {
        Box::new(Self {
            fl_drive,
            fl_turn,
            fr_drive,
            fr_turn,
            bl_drive,
            bl_turn,
            br_drive,
            br_turn,
        })
    }
}

pub struct SwerveBuilder {
    motors: Motors,
    encoders: Encoders,
}

impl SwerveBuilder {
    pub fn new(fl_drive: Box<dyn Motor>, fr_drive: Box<dyn Motor>, bl_drive: Box<dyn Motor>, br_drive: Box<dyn Motor>,
               fl_turn: Box<dyn Motor>, fr_turn: Box<dyn Motor>, bl_turn: Box<dyn Motor>, br_turn: Box<dyn Motor>,
    fl_encoder: Box<dyn Encoder>, fr_encoder: Box<dyn Encoder>, bl_encoder: Box<dyn Encoder>, br_encoder: Box<dyn Encoder>) -> Box<Self> {

        Box::new(Self {
            motors: *Motors::new(fl_drive, fl_turn, fr_drive, fr_turn, bl_drive, bl_turn, br_drive, br_turn),
            encoders: *Encoders::new(fl_encoder, fr_encoder, bl_encoder, br_encoder)
        })
    }

    pub fn set_speeds(&self) {

    }
}*/

use std::f64::consts::{PI, FRAC_PI_2, FRAC_2_PI};
use uom::{num_traits::{Pow, PrimInt}, si::{angle::{degree, radian}, f64::Length, length::inch}};
use uom::si::f64::*;
use frcrs::networktables::SmartDashboard;
use nalgebra::{Vector2, Rotation2, ComplexField};

pub type WheelSpeeds = Vec<ModuleState>;
pub type ModulePosition = Vector2<f64>;
#[derive(Debug, PartialEq, PartialOrd)]
pub struct ModuleState {
    pub speed: f64,
    pub angle: Angle,
}

impl ModuleState {
    /// change this module to be equivalent, but close to other
    pub fn optimize(mut self, other: &ModuleState) -> Self {
        let angle = self.angle.get::<degree>();
        let other_angle = other.angle.get::<degree>();

        let cycles = other_angle as usize / 360;

        let mut difference = ( other_angle - angle + 180. ) % 360. - 180.;
        if difference < -180. { difference += 360. };


        let negate = difference.abs() > 90.;

        if negate {
            self.speed *= -1.;

            dbg!(difference);
            //if difference > 0. {
            //    difference = 180. - difference;
            //} else {
            //    difference = -180. - difference;
            //}
            difference = 180. - difference;

            difference %= 360.;
            dbg!(difference);
            self.angle = Angle::new::<degree>(difference);
        }

        dbg!(cycles);
        self.angle += Angle::new::<degree>((360 * cycles) as f64);

        self
    }
}

pub struct Swerve {
    positions: Vec<ModulePosition>,
}

impl Swerve {
    pub fn rectangle(width: Length, height: Length) -> Self { 
        let mut positions: Vec<ModulePosition> = Vec::new();

        let x = width.get::<inch>() / 2.0;
        let y = height.get::<inch>() / 2.0;
        positions.push(ModulePosition::new(x, y));
        positions.push(ModulePosition::new(-x, y));
        positions.push(ModulePosition::new(-x, -y));
        positions.push(ModulePosition::new(x, -y));

        Self { positions } 
    }

    /// Calculate module speeds from the given transform and rotation
    ///
    /// input units are from -1 to 1, with 1 being the maximum motor speed
    pub fn calculate(&self, transform: Vector2<f64>, rotation: f64) -> WheelSpeeds {
        let center_of_rotation = Vector2::zeros(); // relative to center of robot
        let rotation_transform = Rotation2::new(FRAC_PI_2); // 90 degrees

        let mut speeds = Vec::new();

        for module in self.positions.clone() {
            // constant component
            let mut vector = transform;

            let position = module - center_of_rotation;
            let unit_rotation = rotation_transform * position.normalize();

            // rotation
            vector += unit_rotation * rotation;

            let angle = f64::atan2(vector.x, vector.y);

            speeds.push(ModuleState {
                speed: vector.magnitude(),
                angle: Angle::new::<radian>(angle),
            });
        }

        // normalize speeds
        let max = speeds.iter().map(|m| m.speed.abs()).fold(0., f64::max);
        if max > 1. {
            speeds.iter_mut().for_each(|s| s.speed /= max);
        }

        /*SmartDashboard::put_number("fl turn".to_owned(), wa2);
        SmartDashboard::put_number("fr turn".to_owned(), wa1);
        SmartDashboard::put_number("bl turn".to_owned(), wa3);
        SmartDashboard::put_number("br turn".to_owned(), wa4);*/

        speeds
    }

    pub fn optimize(target_speed: f64, target_angle: f64, current_angle: f64) -> (f64, f64) {
        let mut target_angle = Self::place_in_appropriate_0_to_360_scope(current_angle, target_angle);
        //println!("{}", target_angle);

        let delta = target_angle - current_angle;
        let offset = if delta > 0. {
            180.
        } else { -180. };

        return if delta.abs() > 90. {
            (-target_speed, target_angle - offset)
        } else {
            (target_speed, target_angle - offset)
        }
    }

    pub fn place_in_appropriate_0_to_360_scope(scope_ref: f64, new_angle: f64) -> f64 {
        let mut lower_bound = 0.;
        let mut upper_bound = 0.;
        let mut _new_angle = new_angle;
        let lower_offset = scope_ref % 360.;

        if lower_offset >= 0. {
            lower_bound = scope_ref - lower_offset;
            upper_bound = scope_ref + (360. - lower_offset);
        } else {
            upper_bound = scope_ref - lower_offset;
            lower_bound = scope_ref - (360. + lower_offset);
        }
        while _new_angle < lower_bound {
            _new_angle += 360.;
        }
        while new_angle > upper_bound {
            _new_angle -= 360.;
        }
        if _new_angle - scope_ref > 180. {
            _new_angle -= 360.;
        } else if _new_angle - scope_ref < -180. {
            _new_angle += 360.;
        }

        _new_angle
    }
}

pub trait ToTalonEncoder {
    fn talon_encoder_ticks(&self) -> f64;
    fn from_talon_encoder_ticks(&self) -> f64;
}

impl ToTalonEncoder for f64 {
    fn talon_encoder_ticks(&self) -> f64 {
        self / ((360.) / (2048. * 12.8))
    }

    fn from_talon_encoder_ticks(&self) -> f64 {
        self * ((360.) / (2048. * 12.8))
    }
}

#[cfg(test)]
mod tests {
    use frcrs::drive::{Swerve, ToTalonEncoder};
    use uom::si::{f64::Angle, angle::degree};

    use crate::swerve::kinematics::ModuleState;

    #[test]
    fn opposite() {
        let this = ModuleState { speed: 1., angle: Angle::new::<degree>(180.) };
        let other = ModuleState { speed: 1., angle: Angle::new::<degree>(0.) };
        let goal = ModuleState { speed: -1., angle: Angle::new::<degree>(0.) };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn close_cycle() {
        let this = ModuleState { speed: 1., angle: Angle::new::<degree>(45.) };
        let other = ModuleState { speed: 1., angle: Angle::new::<degree>(360.) };
        let goal = ModuleState { speed: 1., angle: Angle::new::<degree>(360. + 45.) };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn close() {
        let this = ModuleState { speed: 1., angle: Angle::new::<degree>(45.) };
        let other = ModuleState { speed: 1., angle: Angle::new::<degree>(0.) };
        let goal = ModuleState { speed: 1., angle: Angle::new::<degree>(45.) };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn close_opposite() {
        let this = ModuleState { speed: 1., angle: Angle::new::<degree>(45. + 180.) };
        let other = ModuleState { speed: 1., angle: Angle::new::<degree>(0.) };
        let goal = ModuleState { speed: -1., angle: Angle::new::<degree>(45.) };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn place_in_scope() {
        let angle = Swerve::place_in_appropriate_0_to_360_scope(720., 90.);

        assert_eq!(810., angle);
    }

    #[test]
    fn opt_invert() {
        let angle = Swerve::optimize(1., 180., 0.);

        let in_range = Swerve::place_in_appropriate_0_to_360_scope(0., angle.1);
        assert_eq!((angle.0, in_range), (-1., 0.));
    }

    #[test]
    fn calc() {
        let wheel_speeds = Swerve::calculate(0.5, 0., 0., 0.);

        assert_eq!((wheel_speeds.ws1, wheel_speeds.wa1), (0.5, 0.));
    }

    #[test]
    fn opt() {
        let wheel_speeds = Swerve::calculate(0.5, 0., 0., 0.);

        let angle1 = Swerve::optimize(wheel_speeds.ws1, wheel_speeds.wa1, 0.);

        assert_eq!((angle1.0, angle1.1), (0., 10.));
    }

    #[test]
    fn opt_invert_high() {
        let angle = Swerve::optimize(1., 180., 725.);

        let in_range = Swerve::place_in_appropriate_0_to_360_scope(725., angle.1);
        assert_eq!((angle.0, in_range), (-1., 720.));
    }

    /*#[test]
    fn calculate_speed() {
        let start = Instant::now();

        let wheel_speeds = Swerve::calculate(0.5, 0.5,0.5, 142.15);

        let fr_speeds = Swerve::optimize(
            wheel_speeds.ws1, wheel_speeds.wa1, 2151.
        );

        let fl_speeds = Swerve::optimize(
            wheel_speeds.ws2, wheel_speeds.wa2, 2615.
        );

        let bl_speeds = Swerve::optimize(
            wheel_speeds.ws3, wheel_speeds.wa3, 2725.
        );

        let br_speeds = Swerve::optimize(
            wheel_speeds.ws4, wheel_speeds.wa4, 2409.
        );

        let fr_turn_pos =  Swerve::place_in_appropriate_0_to_360_scope(
            2151., fr_speeds.1).talon_encoder_ticks();

        let fl_turn_pos = Swerve::place_in_appropriate_0_to_360_scope(
            2615., fl_speeds.1).talon_encoder_ticks();

        let bl_turn_pos = Swerve::place_in_appropriate_0_to_360_scope(
            2725., bl_speeds.1).talon_encoder_ticks();

        let br_turn_pos = Swerve::place_in_appropriate_0_to_360_scope(
            2409., br_speeds.1).talon_encoder_ticks();

        println!("{}μs", start.elapsed().as_micros());
    }*/
}
