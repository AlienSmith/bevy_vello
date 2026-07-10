mod character;
pub mod character_asset;
mod character_factory;
mod collider_factory;
mod utility;
use bevy::prelude::*;

use crate::{
    character::plugin::GameCharacterPlugin, character_asset::plugin::CharacterLoaderPlugin,
    character_factory::plugin::CharacterFactoryPlugin,
};
#[derive(Default)]
pub struct VelloCharacterPlugin;
pub use crate::{
    character::{
        ArmConfig, CharacterController, IkMode, LeftArmController, RightArmController, SpineConfig,
        SpineController,
    },
    character_factory::CharacterRoot,
};

impl Plugin for VelloCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(GameCharacterPlugin)
            .add_plugins(CharacterFactoryPlugin)
            .add_plugins(CharacterLoaderPlugin);
    }
}

pub use crate::{collider_factory::plugin::ColliderFactoryPlugin, collider_factory::ColliderRoot};
