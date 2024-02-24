use std::ops::Sub;

use nalgebra::{Vector2, Rotation2};
use uom::si::{f64::{Length, Angle}, angle::radian, length::meter};

#[derive(Default, Clone)]
pub struct ModuleReturn {
    pub distance: Length,
    pub angle: Angle,
}

impl Into<Vector2<f64>> for ModuleReturn {
    fn into(self) -> Vector2<f64> {
        let angle = self.angle.get::<radian>();
        let distance = self.distance.get::<meter>();

        Rotation2::new(angle) * Vector2::y() * distance
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
}

impl Odometry {
    pub fn new() -> Self { 
        let last_modules = Vec::new();
        let position = Vector2::new(0., 0.);
        Self { last_modules, position } 
    }

    pub fn calculate(&mut self, positions: Vec<ModuleReturn>, angle: Angle) {
        if positions.len() != self.last_modules.len() {
            self.last_modules = positions;
            return;
        }

        let mut deltas: Vec<ModuleReturn> = positions.into_iter().zip(self.last_modules.iter()).map(|(n,o)| {
            n - o.to_owned()
        }).collect();

        for module in &mut deltas {
            module.angle += angle;
        }

        let delta: Vector2<f64> = deltas.into_iter().map(|d| Into::<Vector2<f64>>::into(d)).sum();


        self.position += delta;
    }
}
