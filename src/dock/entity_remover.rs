use bevy::app::App;
use bevy::app::Plugin;
use bevy::prelude::*;

use super::commands::*;
use super::stream_factory::*;
use super::DockSystems;
use crossbeam_channel::Receiver;
pub struct EntityRemoverPlugin;
#[derive(Resource)]
pub struct EntityRemoverReciever {
    r: Receiver<u32>,
}

impl Plugin for EntityRemoverPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::RemoveEntity);
        app.insert_resource(EntityRemoverReciever { r })
            .add_systems(Update, despawn_entity.in_set(DockSystems::Remove));
    }
}

fn despawn_entity(mut commands: Commands, r: Res<EntityRemoverReciever>) {
    if let Ok(id) = r.r.try_recv() {
        bevy::log::info!("remove_dock {}", id.clone());
        let data = dock_get_command(id);
        if let DockCommand::RemoveEntity(entity_id) = &data.data {
            let entity = dock_get_entity_with_id(*entity_id);
            bevy::log::info!("remove entity id {}, {:?}", *entity_id, entity);
            commands.entity(entity).despawn();
            dock_remove_entitie(*entity_id);
            let _ = data.s.send(DockCommandResult::Ok(1));
        } else {
            let _ = data
                .s
                .send(DockCommandResult::NotOk("spawn entity failed".to_owned()));
        }
    }
}
