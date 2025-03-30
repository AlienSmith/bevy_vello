use avian2d::prelude::*;
use bevy::prelude::*;
use bevy_vello::prelude::*;

use crate::collision::{ColliderFlags, ColliderResponds, Health};

#[derive(Clone, Default, Component)]
pub struct StaticEnemy {}

pub fn spawn_static_enemy_at(
    commands: &mut Commands,
    transform: Transform,
    asset_server: &Res<AssetServer>,
) {
    commands.spawn((
        VelloAssetBundle {
            vector: asset_server.load("anim/alien.json"),
            debug_visualizations: DebugVisualizations::Hidden,
            transform,
            ..default()
        },
        PlaybackOptions {
            autoplay: true,
            ..Default::default()
        },
        Collider::circle(512.0),
        ColliderResponds {
            damage: 1.0,
            allowed_collider_masks: ColliderFlags::SHELL | ColliderFlags::EXPLOSION,
            collider_type: ColliderFlags::ALIEN,
            spawn_damage_text: true,
        },
        Health { health: 3.0 },
        StaticEnemy::default(),
    ));
}

pub fn static_alien_control_system(
    mut commands: Commands,
    a_query: Query<(&Health, Entity), With<StaticEnemy>>,
) {
    for (health, entity) in a_query.iter() {
        if health.health <= 0.0 {
            info!("Static Alien being killed");
            commands.entity(entity).despawn();
        }
    }
}
