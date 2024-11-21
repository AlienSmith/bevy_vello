use bevy::{
    app::{App, Plugin, Update},
    prelude::{IntoSystemSetConfigs, SystemSet},
};
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
        .add_plugins(EntitySpawnerPlugin);
    }
}
