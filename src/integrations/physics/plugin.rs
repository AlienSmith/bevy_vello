use bevy::{
    app::{First, FixedUpdate, Plugin, PostUpdate},
    ecs::schedule::IntoScheduleConfigs,
    math::Vec2,
    time::{Fixed, Time},
};

use crate::{
    collision::CollisionSystems,
    integrations::physics::{
        systems::{
            apply_explicit_impulse_on_connection_particle, apply_explicit_impulse_on_softbody,
            create_update_pivot_visualizer, generate_connection, generate_soft_body_for_collider,
            make_collision_constraints, remove_soft_body, reset_visuzlie_colliders,
            run_broad_phase, update_collider_from_soft_body, update_connection_particles,
            update_constraint_world, visualize_colliders,
        },
        CharacterAngularConstraintEvent, CharacterFrameForceEvent, CharacterPivotForceEvent,
        CharacterPivotPositionEvent, CharacterPivotVelocityEvent, ColliderExternalImpulseEvent,
        VelloConstraintWorld,
    },
};

pub struct VelloCollisionResponsePlugin;

impl Plugin for VelloCollisionResponsePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(VelloConstraintWorld::new(Vec2::new(0.0, 0.0)))
            .insert_resource(Time::<Fixed>::from_hz(90.0))
            .add_event::<ColliderExternalImpulseEvent>()
            .add_event::<CharacterPivotForceEvent>()
            .add_event::<CharacterPivotVelocityEvent>()
            .add_event::<CharacterAngularConstraintEvent>()
            .add_event::<CharacterPivotPositionEvent>()
            .add_event::<CharacterFrameForceEvent>()
            // All physics-affecting systems run in FixedUpdate, ordered correctly
            .add_systems(
                FixedUpdate,
                (
                    // 1. Remove soft bodies for despawned entities
                    remove_soft_body,
                    // 2. Initialize new soft bodies from newly added colliders
                    generate_soft_body_for_collider,
                    generate_connection,
                    // 3. Apply external impulses from events
                    apply_explicit_impulse_on_softbody,
                    apply_explicit_impulse_on_connection_particle,
                    // 4. Process collision constraints from previous tick's events
                    make_collision_constraints,
                    // 5. Broad phase: BVH + AABB overlap
                    run_broad_phase,
                    // 6. Physics step + sync + GPU narrow phase
                    update_constraint_world,
                    // 7. Sync physics state back to connection particles
                    update_connection_particles,
                )
                    .chain(),
            )
            // Pure visualization systems stay in PostUpdate
            .add_systems(
                PostUpdate,
                (
                    update_collider_from_soft_body,
                    visualize_colliders,
                    create_update_pivot_visualizer,
                )
                    .chain()
                    .in_set(CollisionSystems::CollisionResponsePhysics),
            )
            .add_systems(First, reset_visuzlie_colliders);
    }
}
