pub mod tank {
    pub mod base;
    pub mod gun;
    pub mod particles;
    pub mod shell;
    pub mod turrent;
}

pub mod enemy {
    pub mod alien;
    pub mod zombie;
}

pub mod camera {
    pub mod edge_pan_camera;
}

pub mod collision;
pub mod decor;
pub mod tank_parts;
pub mod text;

pub mod sprite_sheet;

pub use camera::edge_pan_camera::{update_edge_pan_camera, EdgePanCamera};
pub use collision::{handle_collisions, ColliderFlags, ColliderResponds, Health};
pub use enemy::alien::{spawn_static_enemy_at, static_alien_control_system};
pub use text::{pop_text_update, spawn_pop_text_at, PopTextAnim};

pub use tank::particles::{
    init_particles_player, spawn_particle_at, update_particle_scene, ParticleSceneAnim,
    ParticlesPlayer,
};

pub use decor::{spawn_stone_at, spawn_tree_at, update_tree};
pub use sprite_sheet::{make_sprite_sheet_scene_from_vello_replay_scene, spawn_sprite_sheet_at};
pub use tank_parts::TankGameAssets;
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(C)]
pub struct TankGameAssetsType(pub u32);

impl TankGameAssetsType {
    pub const BASE: Self = Self(0);
    pub const TURRENT: Self = Self(1);
    pub const GUN: Self = Self(2);
    pub const TREE: Self = Self(3);
    pub const STONE: Self = Self(4);
    pub const ZOMBIE_IDEL: Self = Self(5);
    pub const ZOMBIE_MOVE: Self = Self(6);
    pub const ZOMBIE_ATTACK: Self = Self(7);
}

#[derive(Clone, Default)]
pub enum TankGameAssetsMetaData {
    #[default]
    PBR,
    //we need the total frame counts
    SpriteSheet(u32),
}

impl From<TankGameAssetsType> for usize {
    fn from(value: TankGameAssetsType) -> Self {
        value.0 as usize
    }
}
