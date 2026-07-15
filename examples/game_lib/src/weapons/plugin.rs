use bevy::prelude::*;

use crate::{
    weapons::{
        system::{attach_pistol, process_fire_event, update_pistol_aim},
        AttachPistolToCharacterEvent, FireEvent,
    },
    GameLabSystems,
};

#[derive(Default)]
pub struct WeaponPlugin;

impl Plugin for WeaponPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<AttachPistolToCharacterEvent>()
            .add_event::<FireEvent>()
            .add_systems(
                Update,
                (attach_pistol, process_fire_event)
                    .chain()
                    .in_set(GameLabSystems::WriteCharacterPartEvent),
            )
            .add_systems(Update, update_pistol_aim);
    }
}
