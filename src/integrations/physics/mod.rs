mod plugin;
mod systems;

use bevy::{
    ecs::{
        entity::Entity,
        event::{Event, EventWriter},
        system::{In, ResMut, Resource},
    },
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

    pub fn remove_connection(&mut self, connection: Index) {
        self.data.softbody_connection.remove_connection(connection);
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
pub struct JointExternalForceEvent {
    pub filter: fn(Vec<ParticleInfo>, FilterData) -> Vec<vello_physics::soft_body::ExternalForce>,
    pub connection_index: Index,
    pub filter_data: FilterData,
}

#[derive(Event)]
pub struct AddBodyConnectionEvent {
    pub connection_config: ConnectionInitConfig,
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub connection_handle: Index,
}

#[derive(Copy, Clone)]
pub struct ConnectionHandle {
    connection: Index,
    initialized: bool,
}

impl Default for ConnectionHandle {
    fn default() -> Self {
        Self {
            connection: Index::DANGLING,
            initialized: false,
        }
    }
}

#[derive(Resource, Default)]
pub struct SoftBodyConnections {
    pub(crate) connections: thunderdome::Arena<ConnectionHandle>,
}

impl SoftBodyConnections {
    pub fn get_one(&mut self) -> Index {
        let index = self.connections.insert(ConnectionHandle::default());

        index
    }

    pub fn get(&self, index: &Index) -> Option<Index> {
        if let Some(item) = self.connections.get(*index) {
            if item.initialized {
                return Some(item.connection.clone());
            }
        }
        return None;
    }

    pub fn remove(&mut self, index: Index) -> Option<Index> {
        if let Some(item) = self.connections.remove(index) {
            if item.initialized {
                return Some(item.connection);
            }
        }
        return None;
    }
}

pub fn add_soft_body_connections(
    connections: &mut ResMut<SoftBodyConnections>,
    add_connection_events: &mut EventWriter<AddBodyConnectionEvent>,
    connection_config: ConnectionInitConfig,
    entity_a: Entity,
    entity_b: Entity,
) -> Index {
    let index = connections.get_one();
    add_connection_events.send(AddBodyConnectionEvent {
        connection_config,
        entity_a,
        entity_b,
        connection_handle: index.clone(),
    });
    return index;
}
