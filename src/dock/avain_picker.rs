use crate::dock::stream_factory::*;
use avian2d::prelude::*;
use bevy::prelude::*;
use futures::{channel::oneshot, FutureExt};
use std::sync::{Arc, Mutex};
pub struct AvainPickerPlugin;
#[derive(Component)]
struct Picker {
    svg_s: Arc<Mutex<oneshot::Sender<Entity>>>,
}

impl Plugin for AvainPickerPlugin {
    fn build(&self, app: &mut App) {}
}
