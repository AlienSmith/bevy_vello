mod plugin;
mod systems;

use bevy::{
    ecs::{component::Component, entity::Entity, event::Event, resource::Resource},
    math::Vec2,
};

#[derive(Resource)]
pub struct VelloConstraintWorld {
    data: ConstraintWorld<Entity>,
}

impl VelloConstraintWorld {
    pub fn new(gravity: Vec2) -> Self {
        VelloConstraintWorld {
            data: ConstraintWorld {
                gravity: nalgebra::Vector2::<f32>::new(gravity.x, -gravity.y),
                ..Default::default()
            },
        }
    }
    // vello coordinate is x right y down
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.data.gravity = nalgebra::Vector2::<f32>::new(gravity.x, -gravity.y);
    }
}

#[derive(Clone, Copy)]
pub struct FilterData {
    pub impulse: Vec2,
}

pub use plugin::VelloCollisionResponsePlugin;
pub use thunderdome::Index;
pub use vello_physics::collision_response::Particle;
pub use vello_physics::soft_body::ExternalForce;
pub use vello_physics::soft_body::ParticleInfo;
pub use vello_physics::soft_body_connection::ConnectionInitConfig;
use vello_physics::{ConnectionConstraintInitConfig, ConstraintWorld};

// #[derive(Event)]
// pub struct ColliderExternalImpulseEvent {
//     pub entity: Entity,
//     pub impulse: Vec2,
// }

#[derive(Event)]
pub struct ColliderExternalImpulseEvent {
    pub filter: fn(Vec<ParticleInfo>, FilterData) -> Vec<vello_physics::soft_body::ExternalForce>,
    pub entity: Entity,
    pub filter_data: FilterData,
}

#[derive(Event)]
pub struct CharacterPivotForceEvent {
    pub character_entity: Entity,
    pub joint_entity: Entity,
    pub force: Vec2,
}

#[derive(Component)]
pub struct PivotVisualizer;

#[derive(Component, Clone)]
pub struct VelloCharacterPhysicsRoot;

#[derive(Component, Clone)]
pub struct VelloJoint {
    pub init_config: ConnectionConstraintInitConfig<Entity>,
    pub root_entity: Entity,
}

impl VelloJoint {
    pub fn new(
        connection_config: ConnectionConstraintInitConfig<Entity>,
        character: Entity,
    ) -> Self {
        Self {
            init_config: connection_config,
            root_entity: character,
        }
    }
}

/// A simple newtype component wrapper for [`vello::Scene`] for rendering.
#[derive(Component, Clone)]
pub struct VelloParticle {
    pub particle: Particle,
    pub root_entity: Entity,
}

impl VelloParticle {
    pub fn new(particle: Particle, entity: Entity) -> Self {
        Self {
            particle,
            root_entity: entity,
        }
    }
}
