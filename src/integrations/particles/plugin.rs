pub use bevy::prelude::*;

use crate::integrations::particles::systems::update_explosion_effects;
pub struct VelloPartclePlugin;
impl Plugin for VelloPartclePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.add_systems(Update, update_explosion_effects);
    }
}
