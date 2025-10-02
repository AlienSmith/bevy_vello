use bevy::prelude::*;

use crate::{
    collision::{
        RemovedColliders, VelloCollisionScene, VelloCollisionWorld, VELLO_COLLISION_WORLD_RATIO,
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
    scene.scene.reset();
    // only inite a new collision test if the previous one has been consumed
    if r.update_collision_pairs_if_previous_one_has_been_consumed(&query) {
        // info!(
        //     "last collision_pairs registered count {}",
        //     r.collision_pairs.len()
        // );
        for (a, b) in &r.collision_pairs {
            let (c_a, t_a) = query.get(*a).unwrap();
            let affine_a =
                mat4_to_affine(t_a.compute_matrix()).then_scale(VELLO_COLLISION_WORLD_RATIO as f64);
            let (c_b, t_b) = query.get(*b).unwrap();
            let affine_b =
                mat4_to_affine(t_b.compute_matrix()).then_scale(VELLO_COLLISION_WORLD_RATIO as f64);
            scene.scene.encode_colliders(
                (0.0, 1.0).into(),
                &c_a.shape,
                affine_a,
                &c_b.shape,
                affine_b,
            );
        }
        scene.pair = r.collision_pairs.clone();
    }
    r.collision_pairs_bvh.clear();
}
