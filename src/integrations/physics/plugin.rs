use bevy::{
    app::{Plugin, PostUpdate},
    ecs::schedule::IntoSystemConfigs,
    math::Vec2,
    transform::TransformSystem,
};

use crate::integrations::physics::{
    systems::{
        generate_soft_body_for_collider, make_collision_constraints,
        update_collider_from_soft_body, update_constraint_world, visualize_colliders,
    },
    VelloConstraintWorld,
};

pub struct VelloCollisionResponsePlugin;

impl Plugin for VelloCollisionResponsePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(VelloConstraintWorld::new(Vec2::new(0.0, 98.0)))
            .add_systems(
                PostUpdate,
                (
                    generate_soft_body_for_collider,
                    make_collision_constraints,
                    update_constraint_world,
                    update_collider_from_soft_body,
                    visualize_colliders,
                )
                    .chain()
                    .after(TransformSystem::TransformPropagate),
            );
    }
}
