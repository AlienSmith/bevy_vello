use bevy::prelude::*;
use bevy_vello::integrations::physics::Index;

#[derive(Resource, Default)]
pub struct ConnectionStatus {
    pub(crate) last_connection: Option<Entity>,
}
