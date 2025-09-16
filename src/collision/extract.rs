use bevy::{prelude::*, render::Extract};
use vello::CollisionResult;

use crate::collision::{ExtractedVelloCollisionScene, GpuDataChannel, VelloCollisionScene};
//we are in render world
pub fn extract_collision_scene(
    mut render_scene: ResMut<ExtractedVelloCollisionScene>,
    game_scene: Extract<Res<VelloCollisionScene>>,
    channel: Extract<Res<GpuDataChannel<Vec<CollisionResult>>>>,
) {
    render_scene.scene.reset();
    render_scene.scene = game_scene.0.clone();
    if render_scene.sender.is_none() {
        render_scene.sender = Some(channel.sender.clone());
    }
}
