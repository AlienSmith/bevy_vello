mod character;
mod character_asset;
mod character_factory;
mod utility;
use bevy::prelude::*;

use crate::{
    character::plugin::GameCharacterPlugin, character_asset::plugin::CharacterLoaderPlugin,
    character_factory::plugin::CharacterFactoryPlugin,
};
pub struct VelloCharacterPlugin;
pub use crate::character_factory::CharacterRoot;

impl Plugin for VelloCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(GameCharacterPlugin)
            .add_plugins(CharacterFactoryPlugin)
            .add_plugins(CharacterLoaderPlugin);
    }
}
