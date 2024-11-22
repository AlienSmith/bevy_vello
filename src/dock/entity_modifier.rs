use bevy::app::App;
use bevy::app::Plugin;
use bevy::prelude::*;

use crate::VelloAsset;

use super::commands::*;
use super::stream_factory::*;
use super::DockSystems;
use crossbeam_channel::Receiver;
pub struct EntityModifierPlugin;
#[derive(Resource)]
pub struct EntityModifierReciever {
    r: Receiver<u32>,
}

impl Plugin for EntityModifierPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::Transform);
        app.insert_resource(EntityModifierReciever { r })
            .add_systems(Update, modify_entity.in_set(DockSystems::Modify));
    }
}

fn modify_entity(
    r: Res<EntityModifierReciever>,
    mut query: Query<&mut Transform, With<Handle<VelloAsset>>>,
) {
    if let Ok(id) = r.r.try_recv() {
        bevy::log::info!("get_dock {}", id.clone());
        let data = dock_get_command(id);
        if let DockCommand::Transform(entity_id, transform) = &data.data {
            let entity = dock_get_entity_with_id(*entity_id);
            if let Ok(mut item) = query.get_mut(entity) {
                *item = *transform;
            }
            let _ = data.s.send(DockCommandResult::Ok(*entity_id));
        } else {
            let _ = data
                .s
                .send(DockCommandResult::NotOk("spawn entity failed".to_owned()));
        }
    }
}
