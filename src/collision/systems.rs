use bevy::{ecs::entity, prelude::*};

use crate::{
    collision::{VelloCollisionScene, VelloCollisionWorld},
    VelloCollider,
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
    r.collision_pairs
        .push((broad_phase_place_holder[0], broad_phase_place_holder[1]));
}

pub fn make_collision_scene(
    query: Query<(&VelloCollider, &GlobalTransform)>,
    r: Res<VelloCollisionWorld>,
    mut scene: ResMut<VelloCollisionScene>,
) {
    scene.reset();
    for (a, b) in &r.collision_pairs {
        let (c_a, t_a) = query.get(*a).unwrap();
    }
}
