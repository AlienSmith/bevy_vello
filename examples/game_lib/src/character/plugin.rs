use bevy::prelude::*;

use crate::character::system::clean_up_dead_body_parts;
pub struct GameCharacterPlugin;

impl Plugin for GameCharacterPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.add_systems(Update, clean_up_dead_body_parts);
    }
}
