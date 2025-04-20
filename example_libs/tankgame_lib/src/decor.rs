use avian2d::prelude::*;
use bevy::prelude::*;
use bevy_vello::prelude::*;

use crate::{ColliderFlags, ColliderResponds, Health, TankGameAssets, TankGameAssetsType};
#[derive(Clone, Default, Component)]
pub struct Tree {}

pub fn make_scene_from_vello_replay_scene(
    scene: &mut VelloScene,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    parts: &Res<TankGameAssets>,
    part_type: TankGameAssetsType,
) {
    custom_assets
        .get(&parts.get_index(part_type).unwrap())
        .unwrap()
        .player
        .apply_to_scene(scene);
}

pub fn spawn_tree_at(
    commands: &mut Commands,
    parts: &Res<TankGameAssets>,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    translation: Vec3,
) {
    let mut b_s = VelloScene::default();
    make_scene_from_vello_replay_scene(&mut b_s, &custom_assets, &parts, TankGameAssetsType::TREE);
    commands.spawn((
        VelloSceneBundle {
            scene: b_s,
            transform: Transform::from_translation(translation),
            ..Default::default()
        },
        Collider::circle(40.0),
        Health { health: 1.0 },
        ColliderResponds {
            damage: 1.0,
            allowed_collider_masks: ColliderFlags::EXPLOSION,
            collider_type: ColliderFlags::DECOR,
            ..Default::default()
        },
        Tree::default(),
    ));
}

pub fn spawn_stone_at(
    commands: &mut Commands,
    parts: &Res<TankGameAssets>,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    translation: Vec3,
) {
    let mut b_s = VelloScene::default();
    make_scene_from_vello_replay_scene(&mut b_s, &custom_assets, &parts, TankGameAssetsType::STONE);
    commands.spawn((
        VelloSceneBundle {
            scene: b_s,
            transform: Transform::from_translation(translation),
            ..Default::default()
        },
        //have better shape for collisions
        Collider::rectangle(200.0, 200.0),
        Health { health: 1.0 },
        ColliderResponds {
            damage: 1000.0,
            allowed_collider_masks: ColliderFlags::None,
            collider_type: ColliderFlags::DECOR,
            ..Default::default()
        },
    ));
}

pub fn update_tree(mut commands: Commands, t_query: Query<(&Health, Entity), With<Tree>>) {
    for (health, entity) in t_query.iter() {
        if health.health <= 0.0 {
            commands.entity(entity).despawn();
        }
    }
}
