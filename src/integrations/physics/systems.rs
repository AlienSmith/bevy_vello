use std::{cmp::max, vec};

use crate::{
    affine_to_mat4,
    collision::{RemovedColliders, VelloCollisionEvent, VelloCollisionScene, VelloCollisionWorld},
    integrations::physics::{ColliderExternalImpulseEvent, VelloConstraintWorld},
    mat4_to_affine, VelloCollider, VelloScene,
};

use bevy::prelude::*;
use nalgebra::Vector2;
use vello::{
    kurbo::{self, Affine, BezPath, Shape, Stroke},
    peniko::{self, GlowColor},
};
use vello_physics::CoupledConstraintBreaker;
#[inline]
fn vec2_to_vector2_inverse_y(v: &Vec2) -> Vector2<f32> {
    Vector2::<f32>::new(v.x, -v.y)
}

pub fn generate_soft_body_for_collider(
    query: Query<(Entity, &VelloCollider, &GlobalTransform), Added<VelloCollider>>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    for (entity, collider, transform) in query.iter() {
        if collider.is_soft_body {
            constraint_world.data.create_soft_body_from_path_with_frame(
                &collider.shape,
                &mat4_to_affine(transform.compute_matrix()),
                nalgebra::Vector2::<f32>::new(
                    collider.initial_velocity.x,
                    -collider.initial_velocity.y,
                ),
                entity,
                collider.aabb.clone(),
                collider.soft_body_config.clone().unwrap(),
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
        |index: Entity, path: BezPath, affine: Affine, rect: kurbo::Rect, frame: BezPath| {
            if let Ok((mut collider, mut transform)) = query.get_mut(index) {
                let target_matrix = affine_to_mat4(affine);
                *transform = Transform::from_matrix(target_matrix);
                collider.shape = path;
                collider.aabb = rect;
                collider.shape_frame = frame;
            }
        },
    );
}

pub fn apply_explicit_impulse_on_softbody(
    mut constraint_world: ResMut<VelloConstraintWorld>,
    mut events: EventReader<ColliderExternalImpulseEvent>,
) {
    for event in events.read() {
        if let Some(particles) = constraint_world
            .data
            .get_all_frame_info(event.entity.clone())
        {
            for item in (event.filter)(particles, event.filter_data) {
                constraint_world.data.add_external_force(event.entity, item);
            }
        }
    }
}

//consume the collision result togather with the collision pairs.
//notice the collision results are from last frame so some entity could already been removed,
//hence we don't need to add collision constraints to them anymore.
pub fn make_collision_constraints(
    query: Query<&VelloCollider>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    mut events: EventReader<VelloCollisionEvent>,
) {
    for item in events.read() {
        //info!("{:?}", item);
        let a_index = item.entity_a;
        let b_index = item.entity_b;
        let a_position = vec2_to_vector2_inverse_y(&item.collision_point_a);
        let b_position = vec2_to_vector2_inverse_y(&item.collision_point_b);
        let a_curve_index = item.curve_index_a;
        let b_curve_index = item.curve_index_b;
        let a_normal = vec2_to_vector2_inverse_y(&item.collision_normal_a);
        let b_normal = vec2_to_vector2_inverse_y(&item.collision_normal_b);
        let diff = a_position - b_position;
        //consistent with COLLISION_MARGIN
        if diff.dot(&a_normal) > 0.0 {
            let mut constraints0 = None;
            let mut is_soft_0 = false;
            let mut constraints1 = None;
            let mut is_soft_1 = false;
            if let Ok(item) = query.get(a_index) {
                if item.is_soft_body() {
                    let collider_index = a_index;
                    let current_position = a_position;
                    let target_position = b_position;
                    let curve_index = a_curve_index;
                    let collision_config = item.collision_config.unwrap();
                    constraints0 = constraint_world.data.add_one_time_collision_constraint(
                        collider_index,
                        curve_index as usize,
                        current_position,
                        target_position,
                        b_normal,
                        collision_config,
                    );
                    is_soft_0 = true;
                }
            }
            if let Ok(item) = query.get(b_index) {
                if item.is_soft_body() {
                    let collider_index = b_index;
                    let current_position = b_position;
                    let curve_index = b_curve_index;
                    let target_position = a_position;
                    let collision_config = item.collision_config.unwrap();
                    constraints1 = constraint_world.data.add_one_time_collision_constraint(
                        collider_index,
                        curve_index as usize,
                        current_position,
                        target_position,
                        a_normal,
                        collision_config,
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

pub fn update_constraint_world(
    mut collision_scene: ResMut<VelloCollisionScene>,
    collision_world: Res<VelloCollisionWorld>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    time: Res<Time>,
) {
    if !collision_world.paused {
        let delta = time.delta_seconds();
        let substep = max(collision_world.substeps, 1);
        constraint_world.data.step(delta, substep);
        collision_scene.state = crate::collision::CollisionSceneState::Created;
    }
}

pub fn visualize_colliders(mut q: Query<(&mut VelloScene, &VelloCollider, &GlobalTransform)>) {
    for (mut s, c, transform) in q.iter_mut() {
        s.reset();
        s.fill_with_shadow_impl(
            peniko::Fill::NonZero,
            Affine::IDENTITY,
            &c.debug_color,
            None,
            c.uvs.clone(),
            &c.shape,
            true,
        );

        s.stroke(
            &Stroke::new(1.0),
            Affine::IDENTITY,
            GlowColor {
                color: peniko::Color::rgba(0.0, 1.0, 0.0, 0.9),
                glow: 5.0,
            },
            None,
            &c.shape_frame.to_path(0.1),
        );

        if c.is_selected {
            let affine = mat4_to_affine(transform.compute_matrix());
            let transform = Affine::translate(affine.translation()) * affine.inverse();
            s.stroke(
                &Stroke::new(1.0),
                transform,
                GlowColor {
                    color: peniko::Color::rgba(1.0, 0.0, 0.0, 0.9),
                    glow: 5.0,
                },
                None,
                &c.aabb.to_path(0.1),
            );
        }
    }
}
