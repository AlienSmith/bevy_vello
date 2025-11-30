use bevy::prelude::*;
use vello::{CollisionResult, CollisionScene};

use crate::{
    collision::{
        CollisionResults, CollisionSceneState, GpuDataChannel, RemovedColliders,
        VelloCollisionEvent, VelloCollisionScene, VelloCollisionWorld, VELLO_COLLISION_WORLD_RATIO,
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
    if r.paused || scene.state == CollisionSceneState::Extracted {
        scene.scene.reset();
        scene.pair.clear();
        scene.state = super::CollisionSceneState::Extracted;
        return;
    }
    // only inite a new collision test if the previous one has been consumed
    if r.update_collision_pairs_if_previous_one_has_been_consumed(&query) {
        // info!(
        //     "last collision_pairs registered count {}",
        //     r.collision_pairs.len()
        // );
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
            info!("collision scene {} {}", a, b);
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

fn make_collision_event(
    entity_a: &Entity,
    entity_b: &Entity,
    result: &CollisionResult,
    scaling: f32,
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
    }
}

pub fn collision_event_dispatch(
    collision_channel: Res<GpuDataChannel<CollisionResults>>,
    mut writer: EventWriter<VelloCollisionEvent>,
    collision_world: Res<VelloCollisionWorld>,
) {
    if collision_world.collision_pairs.len() == 0 {
        return;
    }
    //the following line would force a sync point between game thread and render thread.
    //match collision_channel.receiver.recv() {
    //notice the default behavious of the channel would consume the collision results.
    // Assumption is frame rate slower that fixed update we might not get a result, which is okay.
    match collision_channel.receiver.try_recv() {
        Ok(data) => {
            assert!(
                data.pairs.len() == collision_world.collision_pairs.len(),
                "pairs count {}, results count {}",
                data.pairs.len(),
                data.results.len()
            );
            info!("Recieved Something");
            let scaling = 1.0 / VELLO_COLLISION_WORLD_RATIO;
            for ((entity_a, entity_b), result) in data.pairs.iter().zip(data.results.iter()) {
                //valid surface normal means valid results other wise there are no collision.
                //the normal would be invalid if broad phase detects overlaps but narrow phase does not.
                if result.a_position_normal[2] != 0.0 || result.a_position_normal[3] != 0.0 {
                    writer.send(make_collision_event(entity_a, entity_b, result, scaling));
                }
            }
        }
        _ => {}
    }
}
