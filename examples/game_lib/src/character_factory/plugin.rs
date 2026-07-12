use bevy::prelude::*;

use crate::character_factory::observers::{assemble_character, handle_character_part_events};
use crate::character_factory::CharacterPartEvent;
pub struct CharacterFactoryPlugin;

impl Plugin for CharacterFactoryPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<CharacterPartEvent>()
            .add_observer(assemble_character)
            .add_systems(Update, handle_character_part_events);
    }
}
