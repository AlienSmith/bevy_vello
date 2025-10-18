use std::vec;

use crate::{
    affine_to_mat4,
    collision::{
        CollisionResults, GpuDataChannel, RemovedColliders, VelloCollisionWorld,
        VELLO_COLLISION_WORLD_RATIO,
    },
    integrations::physics::VelloConstraintWorld,
    mat4_to_affine, VelloCollider, VelloScene,
};
use bevy::{ecs::entity, prelude::*};
use nalgebra::Vector2;
use vello::{
    kurbo::{self, Affine, BezPath, Shape, Stroke},
    peniko::{self, GlowColor},
    CollisionResult,
};
use vello_physics::{utility::path_to_ccw_quad_path, CoupledConstraintBreaker};

pub fn generate_soft_body_for_collider(
    query: Query<(Entity, &VelloCollider, &GlobalTransform), Added<VelloCollider>>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    for (entity, collider, transform) in query.iter() {
        if collider.inverse_mass != 0.0 {
            constraint_world.data.create_soft_body_from_path(
                &path_to_ccw_quad_path(&collider.shape),
                &mat4_to_affine(transform.compute_matrix()),
                nalgebra::Vector2::<f32>::new(
                    collider.initial_velocity.x,
                    -collider.initial_velocity.y,
                ),
                collider.inverse_mass,
                entity,
                collider.complexity_modifier,
            );
        }
    }
}

pub fn remove_soft_body(
    removed_colliders: Res<RemovedColliders>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    for item in &removed_colliders.colliders {
        constraint_world.data.remove_soft_body(*item);
    }
}

pub fn update_collider_from_soft_body(
    mut query: Query<(&mut VelloCollider, &mut Transform)>,
    constraint_world: Res<VelloConstraintWorld>,
) {
    constraint_world.data.get_colliders_from_soft_body(
        |index: Entity, path: BezPath, affine: Affine, rect: kurbo::Rect| {
            if let Ok((mut collider, mut transform)) = query.get_mut(index) {
                let target_matrix = affine_to_mat4(affine);
                *transform = Transform::from_matrix(target_matrix);
                collider.shape = path;
                collider.aabb = rect;
            }
        },
    );
}

//consume the collision result togather with the collision pairs.
//notice the collision results are from last frame so some entity could already been removed,
//hence we don't need to add collision constraints to them anymore.
pub fn make_collision_constraints(
    collision_channel: Res<GpuDataChannel<CollisionResults>>,
    query: Query<&VelloCollider>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    mut collision_world: ResMut<VelloCollisionWorld>,
) {
    if collision_world.collision_pairs.len() == 0 {
        return;
    }
    //the following line would force a sync point between game thread and render thread.
    //match collision_channel.receiver.recv() {
    //notice the default behavious of the channel would consume the collision results.
    match collision_channel.receiver.try_recv() {
        Ok(data) => {
            assert!(
                data.pairs.len() == collision_world.collision_pairs.len(),
                "pairs count {}, results count {}",
                data.pairs.len(),
                data.results.len()
            );
            let scaling = 1.0 / VELLO_COLLISION_WORLD_RATIO;
            assert!(data.pairs.len() != 0);
            for (index, (a_index, b_index)) in data.pairs.iter().enumerate() {
                let c = data.results[index];
                //valid surface normal means valid results
                if c.a_position_normal[2] != 0.0 || c.a_position_normal[3] != 0.0 {
                    let a_position = Vector2::<f32>::new(
                        c.a_position_normal[0] * scaling,
                        c.a_position_normal[1] * scaling,
                    );
                    let b_position = Vector2::<f32>::new(
                        c.b_position_normal[0] * scaling,
                        c.b_position_normal[1] * scaling,
                    );
                    let a_curve_index = c.b_position_normal[3] as u32;
                    let b_curve_index = c.b_position_normal[2] as u32;

                    let diff = a_position - b_position;
                    let a_normal =
                        Vector2::<f32>::new(c.a_position_normal[2], c.a_position_normal[3]);
                    let b_normal = -a_normal;

                    //consistent with COLLISION_MARGIN
                    if diff.dot(&a_normal) > 0.5 {
                        // if diff.magnitude() > 10.0 {
                        //     info!("triggers");
                        //     info!("{} wired {:?}", diff.magnitude(), c);
                        //     collision_world.paused = true;
                        // } else {
                        //     info!("{:?}", c);
                        // }
                        let mut constraints0 = None;
                        let mut is_soft_0 = false;
                        let mut constraints1 = None;
                        let mut is_soft_1 = false;
                        if let Ok(item) = query.get(*a_index) {
                            if item.is_soft_body() {
                                let collider_index = a_index;
                                let current_position = a_position;
                                let target_position = b_position;
                                let curve_index = a_curve_index;
                                constraints0 =
                                    constraint_world.data.add_one_time_collision_constraint(
                                        *collider_index,
                                        curve_index as usize,
                                        current_position,
                                        target_position,
                                        b_normal,
                                    );
                                is_soft_0 = true;
                            }
                        }
                        if let Ok(item) = query.get(*b_index) {
                            if item.is_soft_body() {
                                let collider_index = b_index;
                                let current_position = b_position;
                                let curve_index = b_curve_index;
                                let target_position = a_position;
                                constraints1 =
                                    constraint_world.data.add_one_time_collision_constraint(
                                        *collider_index,
                                        curve_index as usize,
                                        current_position,
                                        target_position,
                                        a_normal,
                                    );
                                is_soft_1 = true;
                            }
                        }

                        //deal with coupled constraints
                        if is_soft_0 && is_soft_1 {
                            if constraints0.is_some() && constraints1.is_some() {
                                let c0 = constraints0.take().unwrap();
                                let c1 = constraints1.take().unwrap();
                                constraint_world.data.add_constraints_breakers(Box::new(
                                    CoupledConstraintBreaker {
                                        constraints: vec![c0, c1],
                                    },
                                ));
                            } else {
                                //you couple has been denied so are you.
                                if let Some(c0) = constraints0.take() {
                                    constraint_world.data.remove_collision_constraint(c0);
                                }
                                if let Some(c1) = constraints1.take() {
                                    constraint_world.data.remove_collision_constraint(c1);
                                }
                            }
                        }
                    }
                }
            }
        }
        _ => {}
    }
}

pub fn update_constraint_world(
    collision_world: Res<VelloCollisionWorld>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    time: Res<Time>,
) {
    if !collision_world.paused {
        let delta = time.delta_seconds();
        constraint_world.data.step(delta);
    }
}

pub fn visualize_colliders(mut q: Query<(&mut VelloScene, &VelloCollider, &GlobalTransform)>) {
    for (mut s, c, _transform) in q.iter_mut() {
        s.reset();
        s.fill_with_shadow(
            peniko::Fill::NonZero,
            Affine::IDENTITY,
            c.debug_color,
            None,
            &c.shape,
            true,
        );
        //draw bbox
        // {
        //     let affine = mat4_to_affine(_transform.compute_matrix());
        //     let transform = Affine::translate(affine.translation()) * affine.inverse();
        //     s.stroke(
        //         &Stroke::new(1.0),
        //         transform,
        //         GlowColor {
        //             color: peniko::Color::rgba(1.0, 0.0, 0.0, 0.9),
        //             glow: 5.0,
        //         },
        //         None,
        //         &c.aabb.to_path(0.1),
        //     );
        // }
    }
}
