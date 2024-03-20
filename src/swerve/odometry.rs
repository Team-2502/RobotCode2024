use std::{ops::Sub, time::Instant};

use frcrs::alliance_station;
use nalgebra::{Vector2, Rotation2};
use uom::si::{f64::{Length, Angle}, angle::radian, length::meter};

use crate::{telemetry::{self, TelemetryStore}, constants::{HALF_FIELD_WIDTH_METERS, HALF_FIELD_LENGTH_METERS}};

#[derive(Default, Clone)]
pub struct ModuleReturn {
    pub distance: Length,
    pub angle: Angle,
}

impl Into<Vector2<f64>> for ModuleReturn {
    fn into(self) -> Vector2<f64> {
        let angle = self.angle.get::<radian>();
        let distance = self.distance.get::<meter>();

        Rotation2::new(-angle) * Vector2::x() * distance
    }
}

impl Sub for ModuleReturn {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            distance: self.distance - rhs.distance,
            angle: self.angle
        }
    }
}


pub struct Odometry {
    last_modules: Vec<ModuleReturn>,
    pub position: Vector2<f64>,
    last_apriltag: Instant,
}

impl Odometry {
    pub fn new() -> Self { 
        let last_modules = Vec::new();
        let position = Vector2::new(0., 0.);
        let last_apriltag = Instant::now();
        Self { last_modules, position, last_apriltag } 
    }

    pub fn set(&mut self, position: Vector2<f64>) {
        if alliance_station().red() {
            self.position.x = position.x;
            self.position.y = HALF_FIELD_WIDTH_METERS - position.y;
        } else {
            self.position = position;
            
        }
    }

    pub async fn update_from_vision(&mut self, telemetry: TelemetryStore, mirror: bool) {
        if let Some((time, pose)) = &telemetry.read().await.apriltag_pose {
            if self.last_apriltag >= *time {
                return;
            }

            self.set(Vector2::new(pose.x, pose.y));
            self.last_apriltag = time.clone();
        }
    }

    pub fn calculate(&mut self, positions: Vec<ModuleReturn>, angle: Angle) {
        if positions.len() != self.last_modules.len() {
            self.last_modules = positions;
            return;
        }

        let mut deltas: Vec<ModuleReturn> = positions.iter().zip(self.last_modules.iter()).map(|(n,o)| {
            n.to_owned() - o.to_owned()
        }).collect();

        for module in &mut deltas {
            module.angle += angle;
        }

        let mut delta: Vector2<f64> = deltas.into_iter().map(|d| Into::<Vector2<f64>>::into(d)).sum();

        delta /= positions.len() as f64;

        self.position += delta;
        self.last_modules = positions;
    }
}
