pub mod tank {
    pub mod base;
    pub mod gun;
    pub mod particles;
    pub mod shell;
    pub mod tank_parts;
    pub mod turrent;
}

pub mod enemy {
    pub mod alien;
}

pub mod camera {
    pub mod edge_pan_camera;
}

pub mod collision;

pub use camera::edge_pan_camera::{update_edge_pan_camera, EdgePanCamera};
pub use collision::{handle_collisions, ColliderFlags, ColliderResponds, Health};
pub use enemy::alien::{spawn_static_enemy_at, static_alien_control_system};

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
