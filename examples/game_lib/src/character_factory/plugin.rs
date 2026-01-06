use bevy::prelude::*;

use crate::character_factory::observers::assemble_character;
pub struct CharacterFactoryPlugin;

impl Plugin for CharacterFactoryPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(assemble_character);
    }
}
