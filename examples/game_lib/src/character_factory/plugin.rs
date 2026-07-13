use bevy::prelude::*;

use crate::character_factory::observers::{
    add_connectivity_to_parts, assemble_character, handle_unregister_part, spawn_character_parts,
};
use crate::character_factory::CharacterPartEvent;
use crate::GameLabSystems;
pub struct CharacterFactoryPlugin;

impl Plugin for CharacterFactoryPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<CharacterPartEvent>()
            .add_observer(assemble_character)
            .add_systems(
                Update,
                (
                    spawn_character_parts,
                    add_connectivity_to_parts,
                    handle_unregister_part,
                )
                    .chain()
                    .in_set(GameLabSystems::ReadCharacterPartEvent),
            );
    }
}
