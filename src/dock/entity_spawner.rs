use bevy::app::App;
use bevy::app::Plugin;
use bevy::prelude::*;

use crate::VelloAssetBundle;

use super::commands::*;
use super::stream_factory::*;
use super::DockSystems;
use crossbeam_channel::Receiver;
pub struct EntitySpawnerPlugin;
#[derive(Resource)]
pub struct EntitySpawnerReciever {
    r: Receiver<u32>,
}

impl Plugin for EntitySpawnerPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::SpawnEntity);
        app.insert_resource(EntitySpawnerReciever { r })
            .add_systems(Update, spawn_vello_bundle.in_set(DockSystems::Spawn));
    }
}

fn spawn_vello_bundle(mut commands: Commands, r: Res<EntitySpawnerReciever>) {
    if let Ok(id) = r.r.try_recv() {
        bevy::log::info!("get_dock {}", id.clone());
        let data = dock_get_command(id);
        if let DockCommand::SpawnEntity(asset_id, transform) = &data.data {
            let asset = dock_get_asset_with_id(*asset_id);
            let entity = commands
                .spawn(VelloAssetBundle {
                    vector: asset,
                    transform: *transform,
                    ..default()
                })
                .id();
            let entity_id = dock_push_entitie(entity);
            let _ = data.s.send(DockCommandResult::Ok(entity_id));
        } else {
            let _ = data
                .s
                .send(DockCommandResult::NotOk("spawn entity failed".to_owned()));
        }
    }
}
