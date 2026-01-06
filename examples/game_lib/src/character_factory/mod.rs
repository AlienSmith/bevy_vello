use bevy::prelude::*;

mod observers;
pub mod plugin;

#[derive(Component)]
pub struct CharacterRoot {
    pub svg_asset_id: String,
    pub blueprint_asset_id: String,
}
