mod plugin;
mod systems;

use bevy::{
    ecs::{entity::Entity, system::Resource},
    math::Vec2,
};

use vello_physics::*;

#[derive(Resource)]
pub struct VelloConstraintWorld {
    data: ConstraintWorld<Entity>,
}

impl VelloConstraintWorld {
    pub fn new(gravity: Vec2) -> Self {
        VelloConstraintWorld {
            data: ConstraintWorld {
                particles: Default::default(),
                constrains: Default::default(),
                self_resolved_constraints: Default::default(),
                softbodies: Default::default(),
                collider_to_body: Default::default(),
                body_to_collider: Default::default(),
                gravity: nalgebra::Vector2::<f32>::new(gravity.x, -gravity.y),
            },
        }
    }
}

pub use plugin::VelloCollisionResponsePlugin;
