use crate::dock::stream_factory::*;
use avian2d::prelude::*;
use bevy::prelude::*;
use crossbeam_channel::Receiver;
use futures::{channel::oneshot, FutureExt};
use std::sync::{Arc, Mutex};

use super::camera_controller::CameraModifiedReciever;
pub struct AvainPickerPlugin;
#[derive(Component)]
pub struct AvainPickReceiver {
    r: Receiver<u32>,
}

impl Plugin for AvainPickerPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(super::commands::DockCommandDispatcherType::PickEntity);
        app.insert_resource(AvainPickReceiver { r })
    }
}
