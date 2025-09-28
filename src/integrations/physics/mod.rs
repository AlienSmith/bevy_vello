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
                softbodies: Default::default(),
                collider_to_body: Default::default(),
                body_to_collider: Default::default(),
                gravity: nalgebra::Vector2::<f32>::new(gravity.x, -gravity.y),
            },
        }
    }
    // vello coordinate is x right y down
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.data.gravity = nalgebra::Vector2::<f32>::new(gravity.x, -gravity.y);
    }
}

pub use plugin::VelloCollisionResponsePlugin;
