use std::cmp::max;

use crate::{
    affine_to_mat4,
    collision::{
        CollisionEventBatch, CollisionEventEntry, CollisionOverride, GpuCollisionRunner,
        RemovedColliders, VelloCollisionBroadPhase, VelloCollisionEvent, VelloCollisionWorld,
        VELLO_COLLISION_WORLD_RATIO,
    },
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
    CollisionScene,
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
                collider.initilized_by_physics = true;
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
/// Resolve a `CollisionOverride` (game-level intent) into actual physics
/// parameters for `add_one_time_collision_constraint`.
///
/// # How it works
///
/// The solver computes:
/// ```text
/// total_inv_mass = inv_mass + other_inv_mass
/// frame_velocity = (frame_velocity - other_velocity) * inv_mass / total_inv_mass
/// ```
///
/// Given a desired `explosion_impulse` (non-physical energy injection), we solve
/// for `other_inv_mass` and `other_velocity` that produce that impulse.
///
/// With `other_inv_mass ≈ 0` (infinitely heavy opponent):
///   frame_velocity' ≈ frame_velocity - other_velocity
///   delta_v ≈ -other_velocity
///   impulse ≈ -other_velocity / inv_mass
///
/// So: other_velocity ≈ -explosion_impulse * inv_mass
fn resolve_collision_intent(
    intent: &CollisionOverride,
    actual_inv_mass: f32,
    _actual_frame_velocity: Vec2,
    opponent_actual_inv_mass: f32,
    opponent_actual_velocity: Vec2,
) -> (f32, Vec2) {
    // Start with opponent's actual physics values as base
    let base_inv_mass = opponent_actual_inv_mass;
    let base_velocity = opponent_actual_velocity;

    // Apply scale factors (default = 1.0 = use actual physics values)
    let inv_mass_scale = intent.inv_mass_scale.unwrap_or(1.0);
    let velocity_scale = intent.velocity_scale.unwrap_or(1.0);

    let mut other_inv_mass = base_inv_mass * inv_mass_scale;
    let mut other_velocity = base_velocity * velocity_scale;

    // If there's an explosion impulse, compute other_velocity to produce it.
    // Uses the "heavy opponent" hack (other_inv_mass = 0.0) so the explosion
    // impulse dominates the collision response.
    if let Some(explosion) = intent.explosion_impulse {
        other_inv_mass = 0.0;
        // Negative because other_velocity is subtracted in the solver:
        //   frame_velocity' = (frame_velocity - other_velocity) * ...
        // So to push the soft body in direction D, set other_velocity = -D * scale.
        other_velocity = -explosion * actual_inv_mass;
    }

    (other_inv_mass, other_velocity)
}

/// Create collision constraints from the [`CollisionEventBatch`].
/// Reads per-pair overrides written by game observers in PostUpdate,
/// resolves intent into physics parameters, and creates constraints
/// for the XPBD solver.
///
/// Uses physics snapshots captured at detection time (stored in the event)
/// rather than querying current physics state, since overrides may have
/// modified the intent based on those snapshots.
pub fn make_collision_constraints(
    query: Query<&VelloCollider>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    batch: Res<CollisionEventBatch>,
) {
    for entry in batch.entries.iter() {
        let item = &entry.event;
        let a_index = item.entity_a;
        let b_index = item.entity_b;
        let a_position = vec2_to_vector2_inverse_y(&item.collision_point_a);
        let b_position = vec2_to_vector2_inverse_y(&item.collision_point_b);
        let a_curve_index = item.curve_index_a;
        let b_curve_index = item.curve_index_b;
        let a_normal = vec2_to_vector2_inverse_y(&item.collision_normal_a);
        let b_normal = vec2_to_vector2_inverse_y(&item.collision_normal_b);
        let diff = a_position - b_position;

        // Use physics snapshots from detection time as defaults,
        // then apply per-pair overrides if set.
        let inv_mass_a = item.inv_mass_a;
        let vel_a = item.velocity_a;
        let inv_mass_b = item.inv_mass_b;
        let vel_b = item.velocity_b;

        if diff.dot(a_normal) > 0.0 {
            // For side A: check override_a (how A wants B to behave)
            let (effective_inv_mass_b, effective_vel_b) = if entry.override_a.is_active() {
                resolve_collision_intent(&entry.override_a, inv_mass_a, vel_a, inv_mass_b, vel_b)
            } else {
                (inv_mass_b, vel_b)
            };

            // For side B: check override_b (how B wants A to behave)
            let (effective_inv_mass_a, effective_vel_a) = if entry.override_b.is_active() {
                resolve_collision_intent(&entry.override_b, inv_mass_b, vel_b, inv_mass_a, vel_a)
            } else {
                (inv_mass_a, vel_a)
            };

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
                        effective_vel_b,
                        effective_inv_mass_b,
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
                        effective_vel_a,
                        effective_inv_mass_a,
                        collision_config,
                    );
                }
            }
        }
    }
}

/// Steps the physics simulation (XPBD solver).
pub fn update_constraint_world(
    mut constraint_world: ResMut<VelloConstraintWorld>,
    mut collision_world: ResMut<VelloCollisionWorld>,
    time: Res<Time>,
) {
    if collision_world.paused {
        return;
    }
    let delta = time.delta_secs();
    let substep = max(collision_world.substeps, 1);
    constraint_world.data.step(delta, substep);
}

/// Filters broad-phase pairs, builds the collision scene, runs GPU collision
/// synchronously, and populates [`CollisionEventBatch`] with entries that
/// include physics snapshots and empty per-pair overrides.
///
/// The batch is consumed by `make_collision_constraints` at the start of the
/// **next** FixedUpdate, giving PostUpdate observers time to write overrides.
pub fn run_gpu_collision(
    collision_runner: Res<GpuCollisionRunner>,
    mut batch: ResMut<CollisionEventBatch>,
    mut collision_world: ResMut<VelloCollisionWorld>,
    query: Query<(&VelloCollider, &Transform)>,
    constraint_world: Res<VelloConstraintWorld>,
) {
    if collision_world.paused {
        return;
    }

    // Step 1: Filter broad-phase pairs
    let pairs: Vec<(Entity, Entity)> = {
        let mut temp = vec![];
        for (e0, e1) in &collision_world.collision_pairs_bvh {
            if let (Ok((c, _)), Ok((c1, _))) = (query.get(*e0), query.get(*e1)) {
                if (c.is_soft_body() || c1.is_soft_body())
                    && (c.collision_group == 0 || (c.collision_group != c1.collision_group))
                {
                    temp.push((*e0, *e1));
                }
            }
        }
        temp
    };

    // Step 2: Build collision scene
    let mut scene = vello::CollisionScene::default();
    for (a, b) in &pairs {
        if let (Ok((c_a, t_a)), Ok((c_b, t_b))) = (query.get(*a), query.get(*b)) {
            let affine_a =
                mat4_to_affine(t_a.compute_matrix()).then_scale(VELLO_COLLISION_WORLD_RATIO as f64);
            let affine_b =
                mat4_to_affine(t_b.compute_matrix()).then_scale(VELLO_COLLISION_WORLD_RATIO as f64);
            scene.encode_colliders(
                (0.0, 1.0).into(),
                &c_a.shape,
                affine_a,
                &c_b.shape,
                affine_b,
            );
        }
    }

    // Step 3: Clear old batch and populate with fresh entries
    batch.entries.clear();

    // Step 4: Run GPU collision synchronously
    if !pairs.is_empty() {
        let results = collision_runner.run_collision(&scene);
        let scaling = 1.0 / VELLO_COLLISION_WORLD_RATIO;
        for ((entity_a, entity_b), result) in pairs.iter().zip(results.iter()) {
            if result.a_position_normal[2] != 0.0 || result.a_position_normal[3] != 0.0 {
                // Capture physics snapshot at detection time
                let (inv_mass_a, vel_a) = get_physics_state(*entity_a, &query, &constraint_world);
                let (inv_mass_b, vel_b) = get_physics_state(*entity_b, &query, &constraint_world);

                batch.entries.push(CollisionEventEntry {
                    event: VelloCollisionEvent {
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
                        collision_normal_a: Vec2::new(
                            result.a_position_normal[2],
                            -result.a_position_normal[3],
                        ),
                        collision_normal_b: Vec2::new(
                            -result.a_position_normal[2],
                            result.a_position_normal[3],
                        ),
                        curve_index_a: result.b_position_normal[3] as u32,
                        curve_index_b: result.b_position_normal[2] as u32,
                        velocity_a: vel_a,
                        velocity_b: vel_b,
                        inv_mass_a,
                        inv_mass_b,
                    },
                    override_a: CollisionOverride::default(),
                    override_b: CollisionOverride::default(),
                });
            }
        }
    }

    collision_world.collision_pairs_bvh.clear();
}

/// Helper to get the physics state (inv_mass, velocity) for a collider at
/// collision detection time. For soft bodies, queries the constraint world;
/// for static bodies, uses the collider's stored inverse mass.
fn get_physics_state(
    entity: Entity,
    query: &Query<(&VelloCollider, &Transform)>,
    constraint_world: &Res<VelloConstraintWorld>,
) -> (f32, Vec2) {
    if let Ok((collider, _)) = query.get(entity) {
        let inv_mass = collider.collision_inverse_mass;
        let velocity = if collider.is_soft_body() {
            constraint_world
                .data
                .get_velocity_of_softbody(entity)
                .unwrap_or(Vec2::ZERO)
        } else {
            Vec2::ZERO
        };
        (inv_mass, velocity)
    } else {
        (0.0, Vec2::ZERO)
    }
}

/// Runs the broad phase (BVH overlap detection) in FixedUpdate.
/// Populates `collision_pairs_bvh` with candidate pairs.
pub fn run_broad_phase(
    all_colliders: Query<(Entity, &VelloCollider)>,
    modified_colliders: Query<
        (Entity, &VelloCollider),
        Or<(Changed<VelloCollider>, Changed<GlobalTransform>)>,
    >,
    removed_collider: Res<RemovedColliders>,
    mut collision_world: ResMut<VelloCollisionWorld>,
    mut broad_phase: ResMut<VelloCollisionBroadPhase>,
) {
    broad_phase.broad_phase.update(
        &all_colliders,
        &modified_colliders,
        &removed_collider,
        &mut collision_world,
    );
}

pub fn reset_visuzlie_colliders(mut q: Query<&mut VelloScene, With<VelloCollider>>) {
    for mut s in q.iter_mut() {
        s.reset()
    }
}

pub fn visualize_colliders(mut q: Query<(&mut VelloScene, &VelloCollider, &GlobalTransform)>) {
    for (mut s, c, transform) in q.iter_mut() {
        if !c.initilized_by_physics {
            continue;
        }
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
