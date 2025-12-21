use bevy::ecs::system::Resource;
use bevy_vello::integrations::physics::Index;

#[derive(Resource, Default)]
pub struct ConnectionStatus {
    pub(crate) last_connection: Option<Index>,
}
