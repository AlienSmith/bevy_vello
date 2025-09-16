use bevy::{ecs::entity, prelude::*};
use vello::{kurbo::Vec2, CollisionResult};

use crate::{
    collision::{GpuDataChannel, VelloCollisionScene, VelloCollisionWorld},
    mat4_to_affine, VelloCollider,
};

pub fn update_collision_world(
    query: Query<(&VelloCollider, Entity)>,
    mut r: ResMut<VelloCollisionWorld>,
) {
    let mut broad_phase_place_holder = vec![];
    for (_, entity) in &query {
        broad_phase_place_holder.push(entity);
    }
    //broad phase collision detection here
    if broad_phase_place_holder.len() == 2 {
        r.collision_pairs.clear();
        r.collision_pairs
            .push((broad_phase_place_holder[0], broad_phase_place_holder[1]));
    }
}

pub fn make_collision_scene(
    query: Query<(&VelloCollider, &GlobalTransform)>,
    r: Res<VelloCollisionWorld>,
    mut scene: ResMut<VelloCollisionScene>,
) {
    scene.reset();
    for (a, b) in &r.collision_pairs {
        let (c_a, t_a) = query.get(*a).unwrap();
        let affine_a = mat4_to_affine(t_a.compute_matrix());
        let (c_b, t_b) = query.get(*b).unwrap();
        let affine_b = mat4_to_affine(t_b.compute_matrix());
        scene.encode_colliders(
            (0.0, 1.0).into(),
            &c_a.shape,
            affine_a,
            &c_b.shape,
            affine_b,
        );
    }
}

pub fn print_collision_results(channel: Res<GpuDataChannel<Vec<CollisionResult>>>) {
    match channel.receiver.try_recv() {
        Ok(data) => {
            info!("{:?} \n", data);
        }
        _ => {}
    }
}
