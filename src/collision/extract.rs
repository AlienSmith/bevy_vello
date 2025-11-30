use bevy::{prelude::*, render::Extract};

use crate::collision::{
    CollisionResults, CollisionSceneState, ExtractedVelloCollisionScene, GpuDataChannel,
    VelloCollisionScene,
};
//we are in render world
pub fn extract_collision_scene(
    mut render_scene: ResMut<ExtractedVelloCollisionScene>,
    game_scene: Extract<Res<VelloCollisionScene>>,
    channel: Extract<Res<GpuDataChannel<CollisionResults>>>,
) {
    if game_scene.state == CollisionSceneState::NeedExtract {
        info!("Extract Some Scene");
        render_scene.scene = game_scene.scene.clone();
        render_scene.pairs = game_scene.pair.clone();
        if render_scene.sender.is_none() {
            render_scene.sender = Some(channel.sender.clone());
        }
    } else {
        render_scene.scene.reset();
        render_scene.pairs.clear();
    }
}
