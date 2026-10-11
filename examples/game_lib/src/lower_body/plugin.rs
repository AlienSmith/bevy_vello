//! Plugin wiring for the lower-body kinematic sandbox.

use bevy::prelude::*;

use crate::lower_body::systems::{drive_lower_body, reset_lower_body, LowerBodyParams};

/// Registers the lower-body resolver ECS resources and systems.
///
/// Does **not** own rendering or UI — those live in the `collision_detection`
/// example crate (see `raibert_sandbox`), keeping the game_lib resolver reusable.
#[derive(Default)]
pub struct LowerBodySandboxPlugin;

impl Plugin for LowerBodySandboxPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LowerBodyParams>()
            .add_systems(Update, (drive_lower_body, reset_lower_body));
    }
}
