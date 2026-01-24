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

    pub fn remove_connection(&mut self, joint: Entity) {
        self.data.remove_connection(joint);
    }
}

#[derive(Clone, Copy)]
pub struct FilterData {
    pub impulse: Vec2,
}

pub use plugin::VelloCollisionResponsePlugin;
pub use thunderdome::Index;
pub use vello_physics::collision_response::Particle as VelloParticle;
pub use vello_physics::soft_body::ExternalForce;
pub use vello_physics::soft_body::ParticleInfo;
pub use vello_physics::soft_body_connection::ConnectionInitConfig;
use vello_physics::ConstraintWorld;

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
    pub force: vello_physics::soft_body::ExternalForce,
}

#[derive(Event)]
pub struct JointExternalForceEvent {
    pub filter: fn(Vec<ParticleInfo>, FilterData) -> Vec<vello_physics::soft_body::ExternalForce>,
    pub connection_index: Entity,
    pub filter_data: FilterData,
}

#[derive(Component)]
pub struct PivotVisualizer;

pub struct VelloConnectionInitConfig {
    pub connection_config: ConnectionInitConfig,
    pub entity_a: Entity,
    pub entity_b: Entity,
}

#[derive(Component)]
pub struct VelloJoint {
    init_config: VelloConnectionInitConfig,
    pub particle_info: Vec<ParticleInfo>,
}

impl VelloJoint {
    pub fn new(
        connection_config: ConnectionInitConfig,
        entity_a: Entity,
        entity_b: Entity,
    ) -> Self {
        Self {
            init_config: VelloConnectionInitConfig {
                connection_config,
                entity_a,
                entity_b,
            },
            particle_info: vec![],
        }
    }
}
