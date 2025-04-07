use avian2d::prelude::Collider;
use bevy::prelude::*;
use bevy_vello::{
    vello::{
        kurbo::{Affine, BezPath, PathEl, Stroke},
        peniko::{Color, Gradient},
    },
    VelloScene, VelloSceneBundle,
};

use crate::{
    collision::{ColliderFlags, ColliderResponds, Health, SingleFrameCollider},
    spawn_particle_at, ParticlesPlayer,
};
#[derive(Clone, Component)]
pub struct Shell {
    movement_speed: f32,
    timer: Timer,
}
//width = 14.0
fn make_shell_scene(length: f64, width: f64, steps: usize) -> VelloScene {
    let mut path = BezPath::new();
    path.push(PathEl::MoveTo((0.0, 0.0).into()));
    path.push(PathEl::LineTo((length, 0.0).into()));
    let mut scene = VelloScene::default();
    let mut color_stops = vec![];
    let step = 1.0 / (steps as f64);
    for i in 0..steps {
        let value = 0.0 + (i as f64) * step;
        color_stops.push(Color::rgba((value * value) as f64, 0.0, 0.0, value));
    }

    let linear = Gradient::new_linear((0.0, 0.0), (length, 0.0)).with_stops(color_stops.as_slice());
    scene.stroke(
        &Stroke::new(width),
        Affine::translate((-1.0 * length, 0.0)),
        &linear,
        None,
        &path,
    );
    scene
}

pub fn spawn_sell(commands: &mut Commands, start: Vec2, target: Vec2, movement_speed: f32) {
    let diff = target - start;
    let length = (diff).length();
    let time = length / movement_speed;
    let from = Vec3::X;
    let to = Vec3::new(diff.x / length, diff.y / length, 0.0);
    let quat = Quat::from_rotation_arc(from, to).normalize();
    let transform =
        Transform::from_rotation(quat).with_translation(Vec3::new(start.x, start.y, 0.));
    commands.spawn((
        VelloSceneBundle {
            scene: make_shell_scene(400.0, 10.0, 4),
            transform,
            ..Default::default()
        },
        Shell {
            movement_speed,
            timer: Timer::from_seconds(time, TimerMode::Once),
        },
        Collider::circle(6.0),
        ColliderResponds {
            damage: 0.0,
            allowed_collider_masks: ColliderFlags::ALIEN | ColliderFlags::DECOR,
            collider_type: ColliderFlags::SHELL,
            ..Default::default()
        },
        Health { health: 1.0 },
    ));
}

pub fn spawn_shell_damage_collider(commands: &mut Commands, translate: Vec3, scale: f32) {
    //contact
    commands.spawn((
        Transform::from_translation(Vec3 {
            x: translate.x,
            y: translate.y,
            z: 0.0,
        }),
        Health { health: 0.0 },
        Collider::circle(scale),
        ColliderResponds {
            damage: 1.0,
            allowed_collider_masks: ColliderFlags::None,
            collider_type: ColliderFlags::EXPLOSION,
            ..Default::default()
        },
        SingleFrameCollider::default(),
    ));
    //explosion
    commands.spawn((
        Transform::from_translation(Vec3 {
            x: translate.x,
            y: translate.y,
            z: 0.0,
        }),
        Health { health: 1.0 },
        Collider::circle(80.0),
        ColliderResponds {
            damage: 2.0,
            allowed_collider_masks: ColliderFlags::None,
            collider_type: ColliderFlags::EXPLOSION,
            ..Default::default()
        },
        SingleFrameCollider::default(),
    ));
}

pub fn update_shell(
    mut commands: Commands,
    mut shell_query: Query<(
        &mut Transform,
        &mut Shell,
        &GlobalTransform,
        &Health,
        Entity,
    )>,
    player: Res<ParticlesPlayer>,
    time: Res<Time>,
) {
    for (mut transform, mut shell, global_transform, health, entity) in shell_query.iter_mut() {
        let movement_direction = transform.rotation * Vec3::X;
        let translation_delta = movement_direction * shell.movement_speed * time.delta_seconds();
        transform.translation += translation_delta;
        shell.timer.tick(time.delta());
        if shell.timer.finished() || health.health <= 0.0 {
            let pos = global_transform.translation();
            spawn_particle_at(&mut commands, &player, pos);
            commands.entity(entity).despawn();
            spawn_shell_damage_collider(&mut commands, pos, 6.0);
        }
    }
}
