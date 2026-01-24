use std::cmp::max;

use crate::{
    affine_to_mat4,
    collision::{RemovedColliders, VelloCollisionEvent, VelloCollisionScene, VelloCollisionWorld},
    integrations::physics::{
        CharacterPivotForceEvent, ColliderExternalImpulseEvent, JointExternalForceEvent,
        PivotVisualizer, VelloConstraintWorld, VelloJoint,
    },
    mat4_to_affine, VelloCollider, VelloScene, VelloSceneBundle,
};

use bevy::prelude::*;
use nalgebra::Vector2;
use vello::{
    kurbo::{self, Affine, BezPath, PathEl, Shape, Stroke},
    peniko::{self, GlowColor},
};
#[inline]
fn vec2_to_vector2_inverse_y(v: &Vec2) -> Vector2<f32> {
    Vector2::<f32>::new(v.x, -v.y)
}

pub fn generate_soft_body_for_collider(
    query: Query<(Entity, &VelloCollider), Added<VelloCollider>>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    for (entity, collider) in query.iter() {
        if collider.is_soft_body {
            constraint_world.data.create_soft_body_from_path_with_frame(
                &collider.shape,
                &mat4_to_affine(collider.soft_body_global_transform.compute_matrix()),
                nalgebra::Vector2::<f32>::new(
                    collider.initial_velocity.x,
                    -collider.initial_velocity.y,
                ),
                entity,
                collider.soft_body_config.clone().unwrap(),
            );
        }
    }
}

pub fn generate_connection_for_joint(
    query: Query<(Entity, &VelloJoint), Added<VelloJoint>>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    for (e, j) in query.iter() {
        let index_a = constraint_world
            .data
            .get_softbody_from_collider(j.init_config.entity_a)
            .unwrap();
        let index_b = constraint_world
            .data
            .get_softbody_from_collider(j.init_config.entity_b)
            .unwrap();
        constraint_world.data.add_connection(
            &j.init_config.connection_config,
            (index_a, index_b),
            e,
        );
    }
}

pub fn update_joint_from_connection(
    mut query: Query<(Entity, &mut VelloJoint)>,
    constraint_world: Res<VelloConstraintWorld>,
) {
    for (e, mut joint) in query.iter_mut() {
        if let Some(item) = constraint_world.data.get_particles_info_of_connection(e) {
            joint.particle_info = item;
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

pub fn remove_connection(
    mut removed: RemovedComponents<VelloJoint>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    removed.read().into_iter().for_each(|e| {
        constraint_world.data.remove_connection(e);
    });
}

pub fn update_collider_from_soft_body(
    mut query: Query<(&mut VelloCollider, &mut Transform)>,
    constraint_world: Res<VelloConstraintWorld>,
) {
    constraint_world.data.get_colliders_from_soft_body(
        |index: Entity, path: BezPath, affine: Affine, rect: kurbo::Rect, frame: BezPath| {
            if let Ok((mut collider, mut transform)) = query.get_mut(index) {
                let target_matrix = affine_to_mat4(affine);
                let temp = Transform::from_matrix(target_matrix);
                *transform = temp;
                collider.soft_body_global_transform = temp;
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

pub fn apply_explicit_impulse_on_joint(
    mut constraint_world: ResMut<VelloConstraintWorld>,
    mut events: EventReader<JointExternalForceEvent>,
) {
    for event in events.read() {
        if let Some(particles) = constraint_world
            .data
            .get_particles_info_of_connection(event.connection_index)
        {
            for item in (event.filter)(particles, event.filter_data) {
                constraint_world
                    .data
                    .add_external_force_connection(event.connection_index, item);
            }
        }
    }
}

pub fn apply_explicit_impulse_on_pivot(
    mut constraint_world: ResMut<VelloConstraintWorld>,
    mut events: EventReader<CharacterPivotForceEvent>,
) {
    for event in events.read() {
        constraint_world
            .data
            .add_external_force_connection(event.joint_entity, event.force.clone());
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

        let get_info = |entity: Entity| -> (f32, Vector2<f32>) {
            let mut inv_mass = 0.0;
            let mut velocity = Vector2::new(0.0, 0.0);
            if let Ok(item) = query.get(entity) {
                inv_mass = item._inverse_mass;
                if item.is_soft_body() {
                    velocity = constraint_world
                        .data
                        .get_velocity_of_softbody(entity)
                        .unwrap();
                }
            }
            return (inv_mass, velocity);
        };

        //consistent with COLLISION_MARGIN
        if diff.dot(&a_normal) > 0.0 {
            let (inv_mass_a, vel_a) = get_info(a_index);
            let (inv_mass_b, vel_b) = get_info(b_index);

            if let Ok(item) = query.get(a_index) {
                if item.is_soft_body() {
                    let collider_index = a_index;
                    let current_position = a_position;
                    let target_position = b_position;
                    let curve_index = a_curve_index;
                    let collision_config = item.collision_config.unwrap();
                    let _ = constraint_world.data.add_one_time_collision_constraint(
                        collider_index,
                        curve_index as usize,
                        current_position,
                        target_position,
                        b_normal,
                        vel_b,
                        inv_mass_b,
                        collision_config,
                    );
                }
            }
            if let Ok(item) = query.get(b_index) {
                if item.is_soft_body() {
                    let collider_index = b_index;
                    let current_position = b_position;
                    let curve_index = b_curve_index;
                    let target_position = a_position;
                    let collision_config = item.collision_config.unwrap();
                    let _ = constraint_world.data.add_one_time_collision_constraint(
                        collider_index,
                        curve_index as usize,
                        current_position,
                        target_position,
                        a_normal,
                        vel_a,
                        inv_mass_a,
                        collision_config,
                    );
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
        let delta = time.delta_secs();
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

        //draw a outline to make the body parts more obvious
        s.stroke(
            &Stroke::new(2.0),
            Affine::IDENTITY,
            peniko::GlowColor::new(peniko::Color::rgba(1.0, 0.0, 1.0, 0.9), 1.0),
            None,
            &c.shape,
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

pub fn create_update_pivot_visualizer(
    mut commands: Commands,
    mut q: Query<&mut VelloScene, With<PivotVisualizer>>,
    constraint_world: Res<VelloConstraintWorld>,
) {
    let mut pos = constraint_world.data.get_pivots_position();
    let mut path = BezPath::new();
    for item in pos.drain(..) {
        path.push(PathEl::MoveTo((item.0, item.1).into()));
        path.push(PathEl::LineTo((item.0 + 0.01, item.1).into()));
    }
    let mut scene = VelloScene::default();
    scene.stroke(
        &Stroke::new(8.0),
        Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.0, 0.0, 1.0, 0.9), 1.0),
        None,
        &path,
    );
    if q.is_empty() {
        commands.spawn((
            VelloSceneBundle {
                scene,
                transform: Transform::from_translation(Vec3::new(0.0, 0.0, 1000.0)),
                ..Default::default()
            },
            PivotVisualizer,
        ));
    } else {
        if let Ok(mut data) = q.single_mut() {
            *data = scene;
        }
    }
}
