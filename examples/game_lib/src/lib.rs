mod character;
pub mod character_asset;
mod character_factory;
mod collider_factory;
mod utility;
pub mod weapons;
use bevy::prelude::*;

use crate::{
    character::plugin::GameCharacterPlugin, character_asset::plugin::CharacterLoaderPlugin,
    character_factory::plugin::CharacterFactoryPlugin, weapons::plugin::WeaponPlugin,
};
#[derive(Default)]
pub struct VelloCharacterPlugin;
pub use crate::{
    character::{
        ArmConfig, CharacterController, ConnectivityRoot, IkMode, LeftArmController,
        ResetArmControlConstraintsEvent, RightArmController, SpineConfig, SpineController,
        StringPool, WhichArm,
    },
    character_factory::{CharacterPartEvent, CharacterRoot},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, SystemSet)]
pub enum GameLabSystems {
    WriteCharacterPartEvent,
    ReadCharacterPartEvent,
}

impl Plugin for VelloCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(GameCharacterPlugin)
            .add_plugins(CharacterFactoryPlugin)
            .add_plugins(CharacterLoaderPlugin)
            .add_plugins(ColliderFactoryPlugin)
            .add_plugins(WeaponPlugin)
            .configure_sets(
                Update,
                (
                    GameLabSystems::WriteCharacterPartEvent,
                    GameLabSystems::ReadCharacterPartEvent,
                )
                    .chain(),
            );
    }
}

pub use crate::{collider_factory::plugin::ColliderFactoryPlugin, collider_factory::ColliderRoot};
