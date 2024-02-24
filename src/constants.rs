pub const FR_DRIVE: i32 = 1;
pub const FR_TURN: i32 = 2;
pub const FR_ENCODER: i32 = 3;

pub const FL_DRIVE: i32 = 4;
pub const FL_TURN: i32 = 5;
pub const FL_ENCODER: i32 = 6;

pub const BL_DRIVE: i32 = 7;
pub const BL_TURN: i32 = 8;
pub const BL_ENCODER: i32 = 9;

pub const BR_DRIVE: i32 = 10;
pub const BR_TURN: i32 = 11;
pub const BR_ENCODER: i32 = 12;


pub const INTAKE_ROLLER_LEFT: i32 = 1;
pub const INTAKE_ROLLER_RIGHT: i32 = 2;

pub const INTAKE_ACTUATE_LEFT: i32 = 3;
pub const INTAKE_ACTUATE_RIGHT: i32 = 4;


pub const SHOOTER_FEEDER_TOP: i32 = 5;
pub const SHOOTER_FEEDER_BOTTOM: i32 = 6;

pub const SHOOTER_TOP: i32 = 7;
pub const SHOOTER_BOTTOM: i32 = 8;

pub const AMP_BAR: i32 = 11;

pub const CLIMBER_LEFT: i32 = 9;
pub const CLIMBER_RIGHT: i32 = 10;

pub const INTAKE_LIMIT: i32 = 0;
pub const BEAM_BREAK_EMITTER: i32 = 2;
pub const BEAM_BREAK_SIGNAL: i32 = 1;
pub const INTAKE_DOWN_LIMIT: i32 = 3;

pub mod intake {
    pub const INTAKE_OCCUPIED_CURRENT: f64 = 30.;
    pub const INTAKE_OCCUPIED_VELOCITY: f64 = 1500.;
    /// velocity that intake acceleration is "over" at
    pub const INTAKE_FREE_VELOCITY: f64 = 2000.; 
}

pub mod drivetrain {
    use std::f64::consts::PI;

    pub const SWERVE_TURN_KP: f64 = 0.3;

    pub const SWERVE_ROTATIONS_TO_INCHES: f64 = (1./6.75) * (4. * PI);
}
