use std::fs::File;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use frcrs::{alliance_station, AllianceStation};

use frcrs::ctre::{talon_encoder_tick, CanCoder, ControlMode, Talon};
use frcrs::input::Joystick;

use crate::constants::drivetrain::{SWERVE_DRIVE_IE, SWERVE_DRIVE_KD, SWERVE_DRIVE_KF, SWERVE_DRIVE_KFA, SWERVE_DRIVE_KI, SWERVE_DRIVE_KP, SWERVE_ROTATIONS_TO_INCHES, SWERVE_TURN_KP};
use crate::constants::*;
use crate::swerve::kinematics::{ModuleState, Swerve};
use crate::swerve::odometry::{ModuleReturn, Odometry};
use frcrs::navx::NavX;
use nalgebra::{Rotation2, Vector2};
use serde::Deserialize;
use serde::Serialize;
use tokio::time::sleep;
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{inch, meter};
use uom::si::time::Time;
use uom::si::velocity::meter_per_second;
use wpi_trajectory::Path;
use crate::subsystems::Vision;
use crate::telemetry;

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

    //vision: Vision,
}

#[derive(Serialize, Deserialize)]
struct Offsets {
    offsets: [f64; 4],
}

#[derive(Debug, PartialEq)]
struct Point {
    x: f64,
    y: f64,
    distance: f64,
    angle: f64
}

impl Offsets {
    const PATH: &'static str = "/home/lvuser/absolut_homosezual.json";
    fn load() -> Self {
        let mut file = File::open(Self::PATH).unwrap();
        let mut buf = String::new();
        file.read_to_string(&mut buf).unwrap();
        serde_json::from_str(&buf).unwrap_or(Self { offsets: [0.; 4] })
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

        //let vision = Vision::new("".to_owned());

        for (encoder, offset) in [&fr_encoder, &fl_encoder, &bl_encoder, &br_encoder]
            .iter()
            .zip(absolute_offsets.offsets.iter_mut())
        {
            *offset -= encoder.get_absolute();
        }

        for (turn, offset) in [&fr_turn, &fl_turn, &bl_turn, &br_turn]
            .iter()
            .zip(absolute_offsets.offsets.iter_mut())
        {
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

            //vision,
        };

        dt
    }

    /*pub async fn update_limelight(&mut self) {
        self.vision.update().await;
    }*/

    pub fn update_odo_vision(&mut self, vision: Vision) {
        if let Some(pose) = vision.get_position_from_tag_2d(self.get_angle()) {
            self.odometry.set(Vector2::new(pose.x.value, pose.y.value));
        }
    }

    pub fn update_odo(&mut self, pose: Option<Vector2<Length>>) {
        if let Some(p) = pose {
            self.odometry.set_abs(Vector2::new(p.x.value, p.y.value));
        }
    }

    pub fn write_absolute(&mut self) {
        let mut offsets = Offsets::load();
        for (encoder, offset) in [
            &self.fr_encoder,
            &self.fl_encoder,
            &self.bl_encoder,
            &self.br_encoder,
        ]
        .iter()
        .zip(offsets.offsets.iter_mut())
        {
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

        for (module, offset) in [
            &self.fr_drive,
            &self.fl_drive,
            &self.bl_drive,
            &self.br_drive,
        ]
        .iter()
        .zip(angles.iter())
        {
            let distance = module.get_position() * SWERVE_ROTATIONS_TO_INCHES;
            speeds.push(ModuleReturn {
                angle: offset.angle.clone(),
                distance: Length::new::<inch>(distance),
            });
        }

        speeds
    }

    fn get_speeds(&self) -> Vec<ModuleState> {
        let mut speeds = Vec::new();

        for (module, offset) in [&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn]
            .iter()
            .zip(self.absolute_offsets.offsets.iter())
        {
            speeds.push(ModuleState {
                speed: 0.,
                angle: Angle::new::<talon_encoder_tick>(-module.get_position())
                    + Angle::new::<degree>(*offset),
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

        let wheel_speeds: Vec<ModuleState> = wheel_speeds
            .into_iter()
            .zip(measured.iter())
            .map(|(calculated, measured)| calculated.optimize(measured))
            .zip(self.absolute_offsets.offsets.iter())
            .map(|(mut state, offset)| {
                state.angle -= Angle::new::<degree>(*offset);
                state
            })
            .collect();

        self.fr_drive
            .set(ControlMode::Percent, wheel_speeds[0].speed);
        self.fl_drive
            .set(ControlMode::Percent, wheel_speeds[1].speed);
        self.bl_drive
            .set(ControlMode::Percent, wheel_speeds[2].speed);
        self.br_drive
            .set(ControlMode::Percent, wheel_speeds[3].speed);

        self.fr_turn.set(
            ControlMode::Position,
            -wheel_speeds[0].angle.get::<talon_encoder_tick>(),
        );
        self.fl_turn.set(
            ControlMode::Position,
            -wheel_speeds[1].angle.get::<talon_encoder_tick>(),
        );
        self.bl_turn.set(
            ControlMode::Position,
            -wheel_speeds[2].angle.get::<talon_encoder_tick>(),
        );
        self.br_turn.set(
            ControlMode::Position,
            -wheel_speeds[3].angle.get::<talon_encoder_tick>(),
        );
    }

    pub fn dbg_set(&self, angle: f64) {
        println!(
            "front right {}",
            (Angle::new::<talon_encoder_tick>(-self.fr_turn.get_position())).get::<revolution>()
        );
        println!("front right setting {}", angle);
        let angle = Angle::new::<revolution>(angle);
        self.fr_turn
            .set(ControlMode::Position, angle.get::<talon_encoder_tick>());
        //self.fl_turn.set(ControlMode::Position, angle.get::<talon_encoder_tick>());
        //self.bl_turn.set(ControlMode::Position, angle.get::<talon_encoder_tick>());
        //self.br_turn.set(ControlMode::Position, angle.get::<talon_encoder_tick>());
    }

    pub fn zero_wheels(&self) {
        let measured = self.get_speeds();

        for (module, motor) in measured
            .into_iter()
            .zip([&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn].into_iter())
        {
            let remainder = module.angle % Angle::new::<degree>(360.);
            let angle = module.angle - remainder;
            motor.set(ControlMode::Position, -angle.get::<talon_encoder_tick>());
        }
    }

    pub fn get_angle(&self) -> Angle {
        Angle::new::<degree>(self.navx.get_angle())
    }

    pub fn get_offset(&self) -> Angle {
        let mut difference = (self.get_angle() - self.offset).get::<degree>();

        difference = (difference + 180.) % 360. - 180.;
        if difference < -180. {
            difference += 360.
        };

        Angle::new::<degree>(difference)
    }

    pub fn reset_angle(&self) {
        self.navx.reset_angle()
    }

    pub fn reset_heading(&mut self) {
        self.offset = self.get_angle();
    }

    fn closest_point_on_circle(robot_x: f64, robot_y: f64) -> Point {
        let circle_center_x = if alliance_station().red() { 16.5 } else { 0.0 };
        let circle_center_y = 5.5;
        let circle_radius = 2.3;

        let direction_x = robot_x - circle_center_x;
        let direction_y = robot_y - circle_center_y;

        let distance_to_center = (direction_x.powi(2) + direction_y.powi(2)).sqrt();

        // Avoid division by zero by ensuring the robot isn't exactly at the circle's center
        if distance_to_center == 0.0 {
            return Point {
                x: circle_radius,
                y: circle_center_y,
                distance: circle_radius,
                angle: 0.0
            };
        }

        let closest_x = circle_radius * (direction_x / distance_to_center);
        let closest_y = circle_center_y + circle_radius * (direction_y / distance_to_center);

        let min_distance = distance_to_center - circle_radius;

        let rotation_angle = f64::atan2(direction_y, direction_x);

        let rotation_angle_degrees = rotation_angle * (180.0 / std::f64::consts::PI);

        Point {
            x: closest_x,
            y: closest_y,
            distance: min_distance,
            angle: rotation_angle_degrees
        }
    }

    pub async fn follow_circle(drivetrain: &mut Drivetrain, dt: Duration) {
        let mut last_loop = Instant::now();
        let mut last_error = Vector2::zeros();
        let mut i = Vector2::zeros();

        let current_pos = drivetrain.odometry.position;
        let closest = Drivetrain::closest_point_on_circle(
            current_pos.x,
            current_pos.y
        );

        let position = Vector2::new(closest.x, closest.y);
        let target_angle = Angle::new::<degree>(closest.angle);

        let mut error_position = position - current_pos;
        let mut error_angle = (target_angle - drivetrain.get_angle()).get::<radian>();

        if error_position.abs().max() < SWERVE_DRIVE_IE {
            i += error_position;
        }

        error_angle *= SWERVE_TURN_KP;
        error_position *= -SWERVE_DRIVE_KP;

        let mut speed = error_position;
        speed += i * -SWERVE_DRIVE_KI * dt.as_secs_f64() * 9.;

        let speed_s = speed;
        speed += (speed - last_error) * -SWERVE_DRIVE_KD * dt.as_secs_f64() * 9.;
        last_error = speed_s;

        //drivetrain.set_speeds(speed.x, speed.y, error_angle);

        telemetry::put_number("cx", position.x).await;
        telemetry::put_number("cy", position.y).await;
        telemetry::put_number("cr", target_angle.value as f64).await;
        telemetry::put_number("ce", error_position.norm()).await;

        sleep(Duration::from_millis(20)).await;
    }
}

#[cfg(test)]
mod tests {
    use crate::subsystems::Drivetrain;
    use crate::subsystems::drivetrain::Point;

    #[test]
    fn closest_point_on_circle() {
        assert_eq!(Point {
            x: 2.1,
            y: 6.4,
            distance: 1.2,
            angle: 30.
        }, Drivetrain::closest_point_on_circle(3.2, 7.))
    }

    #[test]
    fn closest_point_on_circle_red() {
        assert_eq!(Point {
            x: 2.1,
            y: 6.4,
            distance: 1.2,
            angle: 30.
        }, Drivetrain::closest_point_on_circle(14.3, 4.5))
    }
}