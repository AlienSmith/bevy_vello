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
            add_connection_on_softbody, apply_explicit_impulse_on_joint,
            apply_explicit_impulse_on_softbody, create_update_pivot_visualizer,
            generate_soft_body_for_collider, make_collision_constraints, remove_soft_body,
            update_collider_from_soft_body, update_constraint_world, visualize_colliders,
        },
        AddBodyConnectionEvent, ColliderExternalImpulseEvent, JointExternalForceEvent,
        PivotVisualizer, SoftBodyConnections, VelloConstraintWorld,
    },
};

pub struct VelloCollisionResponsePlugin;

impl Plugin for VelloCollisionResponsePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(VelloConstraintWorld::new(Vec2::new(0.0, -98.0)))
            .insert_resource(Time::<Fixed>::from_hz(90.0))
            .insert_resource(SoftBodyConnections::default())
            .add_event::<ColliderExternalImpulseEvent>()
            .add_event::<JointExternalForceEvent>()
            .add_event::<AddBodyConnectionEvent>()
            .add_systems(FixedUpdate, update_constraint_world)
            .add_systems(
                PostUpdate,
                (
                    generate_soft_body_for_collider,
                    apply_explicit_impulse_on_softbody,
                    add_connection_on_softbody,
                    apply_explicit_impulse_on_joint,
                    make_collision_constraints,
                    remove_soft_body,
                    update_collider_from_soft_body,
                    visualize_colliders,
                    create_update_pivot_visualizer,
                )
                    .chain()
                    .in_set(CollisionSystems::CollisionResponsePhysics),
            );
    }
}
