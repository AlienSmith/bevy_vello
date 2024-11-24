use crate::dock::stream_factory::*;
use bevy::prelude::*;
use crossbeam_channel::Receiver;

use super::commands::{DockCommand, DockCommandDispatcherType, DockCommandResult};

pub struct DockCameraPlugin;
#[derive(Resource)]
pub struct CameraModifiedReciever {
    r: Receiver<u32>,
}

impl Plugin for DockCameraPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::ModifyCamera);
        app.insert_resource(CameraModifiedReciever { r })
            .add_systems(Startup, setup_camera)
            .add_systems(Update, modify_camera);
    }
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

fn modify_camera(
    mut query: Query<(&mut Transform, &mut OrthographicProjection), With<Camera>>,
    r: Res<CameraModifiedReciever>,
) {
    if let Ok(index) = r.r.try_recv() {
        let data = dock_get_command(index);
        if let DockCommand::ModifyCamera(pos, scale) = data.data {
            bevy::log::info!("camera moved to {}", pos);
            let (mut transform, mut orth) = query.single_mut();
            transform.translation.x = pos.x;
            transform.translation.y = pos.y;
            orth.scale = scale;
            let _ = data.s.send(DockCommandResult::Ok(1));
        }
    }
}
