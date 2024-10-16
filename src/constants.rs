pub const FPS_LIMIT: f64 = 250.;
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

pub const BEAM_BREAK_EMITTER: i32 = 2;
pub const BEAM_BREAK_SIGNAL: i32 = 1;
pub const INTAKE_LIMIT: i32 = 0;
pub const INTAKE_CAM_LIMIT: i32 = 3;

pub const INDICATOR_PORT_LEFT: i32 = 4;
pub const INDICATOR_PORT_RIGHT: i32 = 5;

pub const TELEMETRY_PORT: i32 = 5807;
pub const HALF_FIELD_WIDTH_METERS: f64 = 4.1148; // 54/4 feet
pub const HALF_FIELD_LENGTH_METERS: f64 = 8.2296; // 54/2 feet

pub mod intake {
    pub const INTAKE_OCCUPIED_CURRENT: f64 = 20.;
    pub const INTAKE_OCCUPIED_VELOCITY: f64 = 2000.;
    /// velocity that intake acceleration is "over" at
    pub const INTAKE_FREE_VELOCITY: f64 = 2000.;

    pub const INTAKE_DOWN_THRESHOLD: f64 = -67.;
    pub const INTAKE_UP_THRESHOLD: f64 = 4.7;
    pub const INTAKE_DOWN_GOAL: f64 = -70.;
    pub const INTAKE_UP_GOAL: f64 = 7.6;

    pub const INTAKE_ZERO_POINT: f64 = 3.;

    pub const INTAKE_DEGREES_PER_SECOND: f64 = 145.;
}

pub mod drivetrain {
    use std::f64::consts::PI;

    pub const SWERVE_TURN_KP: f64 = 0.3;

    pub const SWERVE_ROTATIONS_TO_INCHES: f64 = (1. / 5.906) * (4. * PI);

    pub const SWERVE_DRIVE_KP: f64 = 0.3;
    pub const SWERVE_DRIVE_KI: f64 = 0.;
    pub const SWERVE_DRIVE_KD: f64 = 0.;
    pub const SWERVE_DRIVE_KF: f64 = 0.; // Velocity ff
    pub const SWERVE_DRIVE_KFA: f64 = 0.; // Acceleration ff

    pub const SWERVE_DRIVE_MAX_ERR: f64 = 0.15;
    pub const SWERVE_DRIVE_SUGGESTION_ERR: f64 = 0.35;
    pub const SWERVE_DRIVE_IE: f64 = 0.0; //0.175; // integral enable

    pub const PODIUM_SHOT_ANGLE: f64 = 34.34; // degrees
}

pub mod amp {
    pub const STOWED_POSITION: f64 = -3.;
    pub const DEPLOYED_POSITION: f64 = -30.4;
}
