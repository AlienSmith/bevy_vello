pub mod tank {
    pub mod base;
    pub mod gun;
    pub mod particles;
    pub mod shell;
    pub mod tank_parts;
    pub mod turrent;
}

pub mod camera {
    pub mod edge_pan_camera;
}

pub use camera::edge_pan_camera::{update_edge_pan_camera, EdgePanCamera};

pub use tank::particles::{
    init_particles_player, spawn_particle_at, update_particle_scene, ParticleSceneAnim,
    ParticlesPlayer,
};

pub use tank::tank_parts::TankParts;
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(C)]
pub struct TankPartsType(pub u32);

impl TankPartsType {
    pub const BASE: Self = Self(0);
    pub const TURRENT: Self = Self(1);
    pub const GUN: Self = Self(2);
}

impl From<TankPartsType> for usize {
    fn from(value: TankPartsType) -> Self {
        value.0 as usize
    }
}
