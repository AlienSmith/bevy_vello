use bevy::{
    app::{FixedUpdate, Plugin, PostUpdate},
    ecs::schedule::IntoSystemConfigs,
    math::Vec2,
    time::{Fixed, Time},
};

use crate::{
    collision::CollisionSystems,
    integrations::physics::{
        systems::{
            apply_explicit_impulse_on_softbody, generate_soft_body_for_collider,
            make_collision_constraints, remove_soft_body, update_collider_from_soft_body,
            update_constraint_world, visualize_colliders,
        },
        ColliderExternalImpulseEvent, VelloConstraintWorld,
    },
};

pub struct VelloCollisionResponsePlugin;

impl Plugin for VelloCollisionResponsePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(VelloConstraintWorld::new(Vec2::new(0.0, -98.0)))
            .insert_resource(Time::<Fixed>::from_hz(90.0))
            .add_event::<ColliderExternalImpulseEvent>()
            .add_systems(FixedUpdate, update_constraint_world)
            .add_systems(
                PostUpdate,
                (
                    generate_soft_body_for_collider,
                    apply_explicit_impulse_on_softbody,
                    make_collision_constraints,
                    remove_soft_body,
                    update_collider_from_soft_body,
                    visualize_colliders,
                )
                    .chain()
                    .in_set(CollisionSystems::CollisionResponsePhysics),
            );
    }
}
