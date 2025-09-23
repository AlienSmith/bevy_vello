use bevy::prelude::*;
use bevy_vello::prelude::*;

use crate::{AssetManager, TankGameAssetsType};

pub fn make_sprite_sheet_scene_from_vello_replay_scene(
    scene: &mut VelloScene,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    parts: &Res<AssetManager>,
    part_type: TankGameAssetsType,
    is_loop: Option<bool>,
    start_time: Option<f32>,
    fps: Option<f32>,
) {
    custom_assets
        .get(&parts.get_index(part_type).unwrap())
        .unwrap()
        .player
        .apply_to_scene(scene);
    scene.overwrite_last_sprite_sheet_play_config(fps, start_time, is_loop);
}

pub fn spawn_sprite_sheet_at(
    commands: &mut Commands,
    parts: &Res<AssetManager>,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    transform: Transform,
    parts_type: TankGameAssetsType,
) {
    let mut b_s = VelloScene::default();
    make_sprite_sheet_scene_from_vello_replay_scene(
        &mut b_s,
        custom_assets,
        parts,
        parts_type,
        None,
        None,
        None,
    );
    commands.spawn(VelloSceneBundle {
        scene: b_s,
        transform,
        ..Default::default()
    });
}
