use avian2d::collision::Collider;
use bevy::prelude::*;
use vello::{CollisionResult, CollisionScene};

use crate::{
    collision::{
        CollisionCoolDownPairManager, CollisionEventBatch, CollisionSceneState, RemovedColliders,
        VelloCollisionEvent, VelloCollisionScene, VelloCollisionTrigger, VelloCollisionWorld,
        VELLO_COLLISION_WORLD_RATIO,
    },
    mat4_to_affine, VelloCollider,
};

pub fn collect_removed_colliders(
    mut removed: RemovedComponents<VelloCollider>,
    mut collection: ResMut<RemovedColliders>,
) {
    collection.colliders = removed.read().collect();
}

pub fn make_collision_scene(
    query: Query<(&VelloCollider, &GlobalTransform)>,
    mut r: ResMut<VelloCollisionWorld>,
    mut scene: ResMut<VelloCollisionScene>,
) {
    if scene.state == CollisionSceneState::Extracted {
        scene.scene.reset();
        scene.pair.clear();
        scene.state = super::CollisionSceneState::Extracted;
        return;
    }
    // only inite a new collision test if the previous one has been consumed
    if r.update_collision_pairs_if_previous_one_has_been_consumed(&query) {
        let mut temp = CollisionScene::default();
        for (a, b) in &r.collision_pairs {
            let (c_a, t_a) = query.get(*a).unwrap();
            let (c_b, t_b) = query.get(*b).unwrap();
            //at this point all none soft body objects are static.
            let affine_a =
                mat4_to_affine(t_a.compute_matrix()).then_scale(VELLO_COLLISION_WORLD_RATIO as f64);
            let affine_b =
                mat4_to_affine(t_b.compute_matrix()).then_scale(VELLO_COLLISION_WORLD_RATIO as f64);
            temp.encode_colliders(
                (0.0, 1.0).into(),
                &c_a.shape,
                affine_a,
                &c_b.shape,
                affine_b,
            );
        }
        if !r.collision_pairs.is_empty() {
            scene.scene = temp;
            scene.pair = r.collision_pairs.clone();
            scene.state = super::CollisionSceneState::NeedExtract;
        } else {
            scene.scene.reset();
            scene.pair.clear();
            scene.state = super::CollisionSceneState::Extracted;
        }
    }
    r.collision_pairs_bvh.clear();
}

/// Build a [`VelloCollisionEvent`] with physics snapshot fields.
pub fn make_collision_event(
    entity_a: &Entity,
    entity_b: &Entity,
    result: &CollisionResult,
    scaling: f32,
    velocity_a: Vec2,
    velocity_b: Vec2,
    inv_mass_a: f32,
    inv_mass_b: f32,
) -> VelloCollisionEvent {
    VelloCollisionEvent {
        entity_a: *entity_a,
        entity_b: *entity_b,
        collision_point_a: Vec2::new(
            result.a_position_normal[0] * scaling,
            result.a_position_normal[1] * -scaling,
        ),
        collision_point_b: Vec2::new(
            result.b_position_normal[0] * scaling,
            result.b_position_normal[1] * -scaling,
        ),
        collision_normal_a: Vec2::new(result.a_position_normal[2], -result.a_position_normal[3]),
        collision_normal_b: Vec2::new(-result.a_position_normal[2], result.a_position_normal[3]),
        curve_index_a: result.b_position_normal[3] as u32,
        curve_index_b: result.b_position_normal[2] as u32,
        velocity_a,
        velocity_b,
        inv_mass_a,
        inv_mass_b,
    }
}

/// Read collision events from [`CollisionEventBatch`] and trigger per-entity
/// [`VelloCollisionTrigger`] observers with physics snapshots and batch indices.
pub fn collision_event_redistribute(
    batch: Res<CollisionEventBatch>,
    mut commands: Commands,
    mut cool_down_manager: ResMut<CollisionCoolDownPairManager>,
    query: Query<&VelloCollider>,
    time: Res<Time>,
) {
    let now = time.elapsed_secs();
    for (batch_index, entry) in batch.entries.iter().enumerate() {
        let event = &entry.event;
        let pos_a = event.collision_point_a;
        let pos_b = event.collision_point_b;
        let normal_a = event.collision_normal_a;
        let normal_b = event.collision_normal_b;
        let diff = pos_b - pos_a;
        if normal_a.dot(diff) > 0.0 || normal_b.dot(diff) < 0.0 {
            continue;
        }
        let gap0 = query.get(event.entity_a).unwrap().collision_cooled_down;
        let gap1 = query.get(event.entity_b).unwrap().collision_cooled_down;
        let gap = gap0.min(gap1);
        let key = CollisionCoolDownPairManager::pack_entity_pair(event.entity_a, event.entity_b);
        if let Some(item) = cool_down_manager.pairs.get_mut(&key) {
            if item.0 + item.1 < now {
                item.0 = now;
                item.1 = gap;
            } else {
                continue;
            }
        } else {
            cool_down_manager.pairs.insert(key, (now, gap));
        }
        let collision_point = (pos_a + pos_b) * 0.5;
        commands.trigger_targets(
            VelloCollisionTrigger {
                entity_self: event.entity_a,
                entity_other: event.entity_b,
                collision_point,
                normal_self: event.collision_normal_a,
                normal_other: event.collision_normal_b,
                batch_index,
                self_velocity: event.velocity_a,
                other_velocity: event.velocity_b,
                self_inv_mass: event.inv_mass_a,
                other_inv_mass: event.inv_mass_b,
            },
            event.entity_a,
        );
        commands.trigger_targets(
            VelloCollisionTrigger {
                entity_self: event.entity_b,
                entity_other: event.entity_a,
                collision_point,
                normal_self: event.collision_normal_b,
                normal_other: event.collision_normal_a,
                batch_index,
                self_velocity: event.velocity_b,
                other_velocity: event.velocity_a,
                self_inv_mass: event.inv_mass_b,
                other_inv_mass: event.inv_mass_a,
            },
            event.entity_b,
        );
    }

    //clean up the record where collision never happens again.
    if cool_down_manager.last_purge_time == 0.0 {
        cool_down_manager.last_purge_time = now;
    } else {
        if cool_down_manager.last_purge_time + cool_down_manager.purge_time_gaps < now {
            cool_down_manager.last_purge_time = now;
            cool_down_manager
                .pairs
                .retain(|_key, value| (value.0 + value.1) > now);
        }
    }
}
