use avain_picker::AvainPickerPlugin;
use avian2d::{prelude::PhysicsDebugPlugin, PhysicsPlugins};
use bevy::{
    app::{App, Plugin, Update},
    prelude::{IntoSystemSetConfigs, SystemSet},
};
use camera_controller::DockCameraPlugin;
use entity_modifier::EntityModifierPlugin;
use entity_remover::EntityRemoverPlugin;
use entity_spawner::EntitySpawnerPlugin;
use lottie_loader::LottieLoaderPlugin;
use svg_loader::SvgLoaderPlugin;

#[cfg(feature = "lottie")]
pub mod lottie_loader;
pub mod stream_factory;
#[cfg(feature = "svg")]
pub mod svg_loader;

pub mod avain_picker;

pub mod commands;

pub mod entity_spawner;

pub mod entity_remover;

pub mod entity_modifier;

pub mod camera_controller;

pub struct DockPlugin;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, SystemSet)]
pub enum DockSystems {
    Remove,
    Modify,
    Pick,
    Spawn,
    Load,
}

impl Plugin for DockPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(
            Update,
            (
                DockSystems::Remove,
                DockSystems::Modify,
                DockSystems::Pick,
                DockSystems::Spawn,
                DockSystems::Load,
            )
                .chain(),
        )
        .add_plugins(LottieLoaderPlugin)
        .add_plugins(SvgLoaderPlugin)
        .add_plugins(EntitySpawnerPlugin)
        .add_plugins(EntityRemoverPlugin)
        .add_plugins(EntityModifierPlugin)
        .add_plugins(DockCameraPlugin)
        .add_plugins(PhysicsPlugins::default().with_length_unit(20.0))
        .add_plugins(PhysicsDebugPlugin::default())
        .add_plugins(AvainPickerPlugin);
    }
}
