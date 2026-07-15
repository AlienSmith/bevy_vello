use bevy::prelude::*;
use vello_physics::{CollisionConstraintConfig, SoftBodyInitConfig};

mod observers;
pub mod plugin;

#[derive(Component)]
pub struct ColliderRoot {
    pub svg_asset_id: String,
    pub albedo_asset_id: String,
    pub normal_asset_id: String,
    pub metallic: f32,
    pub roughness: f32,
    pub softbody_config: SoftBodyInitConfig,
    pub collision_config: CollisionConstraintConfig,
    pub soft_body_init_transform: Transform,
    pub initial_velocity: Vec2,
}
