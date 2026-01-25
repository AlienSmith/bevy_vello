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
    pub joint_entity: Entity,
    pub force: Vec2,
}

#[derive(Component)]
pub struct PivotVisualizer;

#[derive(Component, Clone)]
pub struct VelloJoint {
    init_config: ConnectionConstraintInitConfig<Entity>,
}

impl VelloJoint {
    pub fn new(connection_config: ConnectionConstraintInitConfig<Entity>) -> Self {
        Self {
            init_config: connection_config,
        }
    }
}

/// A simple newtype component wrapper for [`vello::Scene`] for rendering.
#[derive(Component, Default, Clone)]
pub struct VelloParticle(Particle);

impl std::ops::Deref for VelloParticle {
    type Target = Particle;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for VelloParticle {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl VelloParticle {
    pub fn new() -> Self {
        Self::default()
    }
}

impl From<Particle> for VelloParticle {
    fn from(scene: Particle) -> Self {
        Self(scene)
    }
}
