use std::cmp::max;

use crate::{
    affine_to_mat4,
    collision::{RemovedColliders, VelloCollisionEvent, VelloCollisionScene, VelloCollisionWorld},
    integrations::physics::{
        CharacterAngularConstraintEvent, CharacterFrameForceEvent, CharacterPivotForceEvent,
        CharacterPivotPositionEvent, CharacterPivotVelocityEvent, ColliderExternalImpulseEvent,
        PivotVisualizer, VelloCharacterPhysicsRoot, VelloConstraintWorld, VelloJoint,
        VelloParticle,
    },
    mat4_to_affine, VelloCollider, VelloScene, VelloSceneBundle,
};

use bevy::{ecs::error::info, prelude::*};
use vello::{
    kurbo::{self, Affine, BezPath, PathEl, Shape, Stroke},
    peniko::{self, GlowColor},
};
use vello_physics::{
    utility::{vector2_to_kurbo_point, BalancedCoreFrame},
    Particle, FRAME_PARTICLES_COUNT,
};
#[inline]
fn vec2_to_vector2_inverse_y(v: &Vec2) -> Vec2 {
    Vec2::new(v.x, -v.y)
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
                Vec2::new(collider.initial_velocity.x, -collider.initial_velocity.y),
                entity,
                collider.soft_body_config.clone().unwrap(),
                Some(collider.aabb),
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
        |index: Entity,
         path: BezPath,
         affine: Affine,
         rect: kurbo::Rect,
         frame_particles: [Particle; FRAME_PARTICLES_COUNT]| {
            if let Ok((mut collider, mut transform)) = query.get_mut(index) {
                let target_matrix = affine_to_mat4(affine);
                let temp = Transform::from_matrix(target_matrix);
                *transform = temp;
                collider.soft_body_global_transform = temp;
                collider.shape = path;
                collider.aabb = rect;
                collider.frame_particles = frame_particles;
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
            particles.iter().zip(event.impulse).for_each(|(p, i)| {
                constraint_world.data.add_external_force(
                    event.entity,
                    vello_physics::soft_body::ExternalForce::Impulse(p.index, i.x, i.y),
                );
            });
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

        let get_info = |entity: Entity| -> (f32, Vec2) {
            let mut inv_mass = 0.0;
            let mut velocity = Vec2::new(0.0, 0.0);
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
        if diff.dot(a_normal) > 0.0 {
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

pub fn reset_visuzlie_colliders(mut q: Query<&mut VelloScene, With<VelloCollider>>) {
    for mut s in q.iter_mut() {
        s.reset()
    }
}

pub fn visualize_colliders(mut q: Query<(&mut VelloScene, &VelloCollider, &GlobalTransform)>) {
    for (mut s, c, transform) in q.iter_mut() {
        s.fill_with_shadow_impl(
            peniko::Fill::NonZero,
            Affine::IDENTITY,
            &c.debug_color,
            None,
            c.uvs.clone(),
            &c.shape,
            true,
        );

        //TODO: maybe move these visualize logic to visualizer instead of put them here.
        //draw a outline to make the body parts more obvious
        s.stroke(
            &Stroke::new(2.0),
            Affine::IDENTITY,
            peniko::GlowColor::new(peniko::Color::rgba(1.0, 0.0, 1.0, 0.9), 1.0),
            None,
            &c.shape,
        );

        let affine = mat4_to_affine(transform.compute_matrix()).inverse();

        let mut frame = vec![];
        let temp = c.frame_particles[0].pos;
        let point = vector2_to_kurbo_point(&temp);
        let p0 = PathEl::MoveTo(point);
        frame.push(p0);
        for i in 1..c.frame_particles.len() {
            let temp = c.frame_particles[i].pos;
            frame.push(PathEl::LineTo(vector2_to_kurbo_point(&temp)));
        }
        frame.push(PathEl::LineTo(point));

        s.stroke(
            &Stroke::new(1.0),
            affine,
            GlowColor {
                color: peniko::Color::rgba(0.0, 1.0, 0.0, 0.9),
                glow: 5.0,
            },
            None,
            &frame.into_path(0.1),
        );

        if c.is_selected {
            s.stroke(
                &Stroke::new(1.0),
                Affine::IDENTITY,
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

/////The following logic Works With softbody connection
pub fn generate_connection(
    query_r: Query<(Entity, &VelloCharacterPhysicsRoot), Added<VelloCharacterPhysicsRoot>>,
    query_p: Query<(Entity, &VelloParticle), Added<VelloParticle>>,
    query_c: Query<(Entity, &VelloJoint), Added<VelloJoint>>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    //create group object
    for (e, c) in query_r.iter() {
        constraint_world.data.add_group(e);
    }
    //all particles must be added before constraints
    for (e, p) in query_p.iter() {
        let group = constraint_world.data.get_group_mut(p.root_entity).unwrap();
        group.add_connect_particle(e, &p.particle);
    }
    //initliaze shape matching frame
    for (e, c) in query_r.iter() {
        constraint_world
            .data
            .initial_frame(
                &e,
                &c.frame_entities[0],
                &c.frame_entities[1],
                &c.frame_entities[2],
                &c.frame_entities[3],
                &c.shape_matching_frame_config,
            )
            .expect("character missing particles to form frame");
    }
    //add shape matching constraints
    for (e, p) in query_p.iter() {
        let group = constraint_world.data.get_group_mut(p.root_entity).unwrap();
        group.add_connect_particle_shape_matching(&e, &p.shape_matching_init);
    }
    //add onther constraints
    for (e, c) in query_c.iter() {
        constraint_world
            .data
            .add_connect_constraint(c.root_entity, e, c.init_config.clone())
            .unwrap();
    }
}

pub fn update_connection_particles(
    mut query: Query<(Entity, &mut VelloParticle)>,
    mut query_c: Query<(Entity, &mut VelloCharacterPhysicsRoot)>,
    mut query_j: Query<(Entity, &mut VelloJoint)>,
    constraint_world: Res<VelloConstraintWorld>,
) {
    for (e, mut joint) in query.iter_mut() {
        let character = joint.root_entity;
        let group = constraint_world.data.get_group_ref(character).unwrap();
        if let Some(item) = group.get_connect_particle(&e) {
            joint.particle = item;
        }
        if let Some(item) = group.get_connect_particle_shape_matching_config(&e) {
            if joint.shape_matching_init_local_pos.is_none() {
                joint.shape_matching_init_local_pos = Some(item.local_target);
            }
            joint.shape_matching = item;
        }
    }
    for (character, mut joint) in query_c.iter_mut() {
        let group = constraint_world.data.get_group_ref(character).unwrap();
        let item = group.get_frame_connect_particle();
        if item.len() == FRAME_PARTICLES_COUNT {
            let frame_coordinates =
                BalancedCoreFrame::new(item[0].pos, item[1].pos, item[2].pos, item[3].pos);
            if joint.initial_frame_coordinates.is_none() {
                joint.initial_frame_coordinates = Some(frame_coordinates.clone());
            }
            joint.frame_coordinates = frame_coordinates;
        }
    }
    for (e, mut joint) in query_j.iter_mut() {
        let character = joint.root_entity;
        let group = constraint_world.data.get_group_ref(character).unwrap();
        match &mut joint.constraint {
            vello_physics::ConnectionConstraint::Bilinear => {}
            vello_physics::ConnectionConstraint::Distance => {}
            vello_physics::ConnectionConstraint::Angular(angular_constraint_config) => {
                if let Some(item) = group.get_connect_angular_config(&e) {
                    *angular_constraint_config = item;
                    if joint.init_constrats.is_none() {
                        joint.init_constrats =
                            Some(vello_physics::ConnectionConstraint::Angular(item));
                    }
                }
            }
        }
    }
}

pub fn apply_explicit_impulse_on_connection_particle(
    mut constraint_world: ResMut<VelloConstraintWorld>,
    mut events: EventReader<CharacterPivotForceEvent>,
    mut v_events: EventReader<CharacterPivotVelocityEvent>,
    mut frame_events: EventReader<CharacterFrameForceEvent>,
    mut angular_event: EventReader<CharacterAngularConstraintEvent>,
    mut position_event: EventReader<CharacterPivotPositionEvent>,
) {
    for event in events.read() {
        let group = constraint_world
            .data
            .get_group_mut(event.character_entity)
            .unwrap();
        group.add_connect_external_force(
            &event.joint_entity,
            &Vec2::new(event.force.x, event.force.y),
        );
    }
    for event in v_events.read() {
        let group = constraint_world
            .data
            .get_group_mut(event.character_entity)
            .unwrap();
        group.queue_connect_particle_velocity(&event.joint_entity, event.velocity);
    }
    for event in frame_events.read() {
        let group = constraint_world
            .data
            .get_group_mut(event.character_entity)
            .unwrap();
        let nalgebra_vecs: Vec<Vec2> = event.forces.iter().map(|v| Vec2::new(v.x, v.y)).collect();
        group.add_connect_frame_external_force(&nalgebra_vecs);
    }
    for event in angular_event.read() {
        let group = constraint_world
            .data
            .get_group_mut(event.character_entity)
            .unwrap();
        group.set_connect_angular_config(&event.joint_entity, &event.config);
    }
    for event in position_event.read() {
        let group = constraint_world
            .data
            .get_group_mut(event.character_entity)
            .unwrap();
        group.set_connect_particle_shahep_matching_config(&event.joint_entity, &event.target);
    }
}

pub fn create_update_pivot_visualizer(
    mut commands: Commands,
    mut q: Query<&mut VelloScene, With<PivotVisualizer>>,
    q_p: Query<&VelloParticle>,
    q_f_p: Query<&VelloCharacterPhysicsRoot>,
) {
    let mut path = BezPath::new();
    for p in q_p.iter() {
        let item = p.particle.pos;
        path.push(PathEl::MoveTo((item.x, item.y).into()));
        path.push(PathEl::LineTo((item.x + 0.01, item.y).into()));
    }
    let mut scene = VelloScene::default();
    scene.stroke(
        &Stroke::new(8.0),
        Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.0, 0.0, 1.0, 0.9), 1.0),
        None,
        &path,
    );
    let mut frame_coordinate_path = BezPath::new();
    for f_p in q_f_p.iter() {
        let points = vec![
            Vec2::new(-100.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(0.0, -100.0),
            Vec2::new(0.0, 100.0),
        ];
        let world_point: Vec<Vec2> = points
            .iter()
            .map(|pos| f_p.frame_coordinates.local_to_world(*pos))
            .collect();
        frame_coordinate_path.push(PathEl::MoveTo((world_point[0].x, world_point[0].y).into()));
        frame_coordinate_path.push(PathEl::LineTo((world_point[1].x, world_point[1].y).into()));
        frame_coordinate_path.push(PathEl::MoveTo((world_point[2].x, world_point[2].y).into()));
        frame_coordinate_path.push(PathEl::LineTo((world_point[3].x, world_point[3].y).into()));
    }
    scene.stroke(
        &Stroke::new(4.0),
        Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.0, 1.0, 1.0, 0.9), 1.0),
        None,
        &frame_coordinate_path,
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
