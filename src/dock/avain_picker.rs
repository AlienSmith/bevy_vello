use crate::dock::{commands::DockCommand, stream_factory::*};
use avian2d::prelude::*;
use bevy::prelude::*;
use crossbeam_channel::Receiver;
use futures::channel::oneshot;
use std::sync::{Arc, RwLock};

use super::commands::DockCommandResult;
pub struct AvainPickerPlugin;
#[derive(Component)]
pub struct PickerMarker {
    s: Arc<RwLock<Option<oneshot::Sender<DockCommandResult>>>>,
}

#[derive(Resource)]
pub struct AvainPickReceiver {
    r: Receiver<u32>,
}

impl Plugin for AvainPickerPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(super::commands::DockCommandDispatcherType::PickEntity);
        //let spawn_p = apply_deferred.after(spawn_picker);
        //let pick_p = (spawn_p, handle_pick).chain();
        app.insert_resource(AvainPickReceiver { r })
            .add_systems(Update, (handle_pick, spawn_picker).chain());
    }
}

fn spawn_picker(mut commands: Commands, r: Res<AvainPickReceiver>) {
    //we have to do this in two steps, spawn the sensor
    if let Ok(id) = r.r.try_recv() {
        bevy::log::info!("get_dock {}", id.clone());
        let data = dock_get_command(id);
        let sender = data.s;
        let data = data.data;
        if let DockCommand::PickEntity(pos, radius) = data {
            commands.spawn((
                Transform::from_translation(Vec3 {
                    x: pos.x,
                    y: pos.y,
                    z: 0.0,
                }),
                Sensor,
                RigidBody::Static,
                Collider::circle(radius),
                PickerMarker {
                    s: Arc::new(RwLock::new(Some(sender))),
                },
            ));
        }
    }
}

fn handle_pick(mut commands: Commands, query: Query<(&CollidingEntities, &PickerMarker, Entity)>) {
    //obtain existing sensor
    for (coll, pick, entity) in query.iter() {
        let mut index = 0;
        for item in coll.0.iter() {
            let i = item.clone();
            if i != entity {
                bevy::log::info!("entity is {}", i);
                index = dock_get_entity_id(i);
                break;
            }
        }
        let _ = pick
            .s
            .write()
            .unwrap()
            .take()
            .unwrap()
            .send(DockCommandResult::Ok(index));
        commands.entity(entity).despawn();
    }
}
