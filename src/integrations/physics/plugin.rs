use bevy::{
    app::{First, FixedUpdate, Plugin, PostUpdate},
    ecs::schedule::IntoScheduleConfigs,
    math::Vec2,
    time::{Fixed, Time},
};

use crate::{
    collision::{CollisionEventBatch, CollisionSystems},
    integrations::physics::{
        systems::{
            apply_explicit_impulse_on_connection_particle, apply_explicit_impulse_on_softbody,
            create_update_pivot_visualizer, generate_connection, generate_soft_body_for_collider,
            make_collision_constraints, remove_soft_body, reset_visuzlie_colliders,
            run_broad_phase, run_gpu_collision, update_collider_from_soft_body,
            update_connection_particles, update_constraint_world, visualize_colliders,
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
            .insert_resource(CollisionEventBatch::default())
            .add_event::<ColliderExternalImpulseEvent>()
            .add_event::<CharacterPivotForceEvent>()
            .add_event::<CharacterPivotVelocityEvent>()
            .add_event::<CharacterAngularConstraintEvent>()
            .add_event::<CharacterPivotPositionEvent>()
            .add_event::<CharacterFrameForceEvent>()
            // Collision response runs FIRST, then collision detection runs LAST.
            // This introduces a one-frame pipeline delay between detection and
            // response, giving game systems (PostUpdate observers) time to write
            // CollisionOverride before the next frame's physics tick.
            //
            // Frame N:   run_gpu_collision detects collisions and populates
            //            CollisionEventBatch; PostUpdate observers modify
            //            per-pair overrides in the batch.
            // Frame N+1: make_collision_constraints reads overrides from the
            //            batch and creates physics constraints.
            .add_systems(
                FixedUpdate,
                (
                    // 1. Create collision constraints from PREVIOUS frame's batch.
                    //    Reads per-pair overrides written by PostUpdate observers.
                    make_collision_constraints,
                    // 2. Step the physics simulation (XPBD solver) with constraints.
                    update_constraint_world,
                    // 3. Sync physics state back to connection particles.
                    update_connection_particles,
                    // 4. Remove soft bodies for despawned entities.
                    remove_soft_body,
                    // 5. Initialize new soft bodies from newly added colliders.
                    generate_soft_body_for_collider,
                    generate_connection,
                    // 6. Apply external impulses from events.
                    apply_explicit_impulse_on_softbody,
                    apply_explicit_impulse_on_connection_particle,
                    // 7. Sync physics shapes back to Bevy transforms.
                    update_collider_from_soft_body,
                    // 8. Broad phase: BVH + AABB overlap.
                    run_broad_phase,
                    // 9. GPU narrow-phase collision detection; populates
                    //    CollisionEventBatch with physics snapshots.
                    run_gpu_collision,
                )
                    .chain(),
            )
            // Pure visualization systems stay in PostUpdate
            .add_systems(
                PostUpdate,
                (visualize_colliders, create_update_pivot_visualizer)
                    .chain()
                    .in_set(CollisionSystems::CollisionResponsePhysics),
            )
            .add_systems(First, reset_visuzlie_colliders);
    }
}
