use bevy::prelude::*;

use crate::{
    weapons::{system::attach_pistol, AttachPistolToCharacterEvent},
    GameLabSystems,
};

#[derive(Default)]
pub struct WeaponPlugin;

impl Plugin for WeaponPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<AttachPistolToCharacterEvent>();
        app.add_systems(
            Update,
            attach_pistol.in_set(GameLabSystems::WriteCharacterPartEvent),
        );
    }
}
