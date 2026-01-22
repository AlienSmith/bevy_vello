use bevy::prelude::*;

use crate::character::{
    observers::{on_remove_connectivity, on_remove_connectivity_root},
    systems::update_character_movement,
    StringPool,
};
pub struct GameCharacterPlugin;

impl Plugin for GameCharacterPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(StringPool::default())
            .add_observer(on_remove_connectivity)
            .add_observer(on_remove_connectivity_root)
            .add_systems(Update, update_character_movement);
    }
}
