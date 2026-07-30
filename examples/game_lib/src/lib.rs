mod character;
pub mod character_asset;
mod character_factory;
mod collider_factory;
mod damage;
mod death_channel;
mod health;
mod input_management;
mod utility;
pub mod weapons;
use bevy::prelude::*;

use crate::{
    character::plugin::GameCharacterPlugin,
    character_asset::plugin::CharacterLoaderPlugin,
    character_factory::plugin::CharacterFactoryPlugin,
    damage::DamagePlugin,
    death_channel::plugin::DeathChannelPlugin,
    health::HealthPlugin,
    utility::{tick_delayed_events, DelayedEventTrigger},
    weapons::plugin::WeaponPlugin,
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
    damage::{AttackStats, Damageable, PartKind, TotalHealth},
    health::{Die, Health},
    input_management::{
        actions::{default_input_map, PlayerAction},
        mouse::MouseWorldPosition,
        systems::Player as PlayerMarker,
        InputPlugin,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, SystemSet)]
pub enum GameLabSystems {
    WriteCharacterPartEvent,
    ReadCharacterPartEvent,
    ProcessDeathEvents,
    CheckHealth,
}

impl Plugin for VelloCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InputPlugin)
            .add_plugins(GameCharacterPlugin)
            .add_plugins(CharacterFactoryPlugin)
            .add_plugins(CharacterLoaderPlugin)
            .add_plugins(ColliderFactoryPlugin)
            .add_plugins(WeaponPlugin)
            .add_plugins(HealthPlugin)
            .add_plugins(DamagePlugin)
            .add_plugins(DeathChannelPlugin)
            .add_event::<DelayedEventTrigger>()
            .configure_sets(
                Update,
                (
                    GameLabSystems::WriteCharacterPartEvent,
                    GameLabSystems::ProcessDeathEvents,
                    GameLabSystems::ReadCharacterPartEvent,
                    GameLabSystems::CheckHealth,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                tick_delayed_events.in_set(GameLabSystems::WriteCharacterPartEvent),
            );
    }
}

pub use crate::{collider_factory::plugin::ColliderFactoryPlugin, collider_factory::ColliderRoot};
