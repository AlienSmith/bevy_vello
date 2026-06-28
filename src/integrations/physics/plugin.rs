use bevy::{
    app::{FixedUpdate, Plugin, PostUpdate},
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
            make_collision_constraints, remove_soft_body, update_collider_from_soft_body,
            update_connection_particles, update_constraint_world, visualize_colliders,
        },
        CharacterFrameForceEvent, CharacterPivotForceEvent, CharacterPivotVelocityEvent,
        ColliderExternalImpulseEvent, VelloConstraintWorld,
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
            .add_event::<CharacterFrameForceEvent>()
            .add_systems(FixedUpdate, update_constraint_world)
            .add_systems(
                PostUpdate,
                (
                    generate_soft_body_for_collider,
                    generate_connection,
                    apply_explicit_impulse_on_softbody,
                    apply_explicit_impulse_on_connection_particle,
                    make_collision_constraints,
                    remove_soft_body,
                    update_collider_from_soft_body,
                    update_connection_particles,
                    visualize_colliders,
                    create_update_pivot_visualizer,
                )
                    .chain()
                    .in_set(CollisionSystems::CollisionResponsePhysics),
            );
    }
}
