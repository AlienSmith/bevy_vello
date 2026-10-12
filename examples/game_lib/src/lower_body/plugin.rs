//! Plugin wiring for the lower-body kinematic sandbox.

use bevy::prelude::*;
use bevy::time::Fixed;

use crate::lower_body::systems::{drive_lower_body, reset_lower_body, LowerBodyParams};

/// Physics tick for the resolver. Matches the XPBD world's fixed rate so the
/// kinematic sandbox and the dynamics run on the same clock (`dt = 1/90 s`).
pub const SANDBOX_PHYSICS_HZ: f64 = 90.0;

/// Registers the lower-body resolver ECS resources and systems.
///
/// Does **not** own rendering or UI — those live in the `collision_detection`
/// example crate (see `raibert_sandbox`), keeping the game_lib resolver reusable.
#[derive(Default)]
pub struct LowerBodySandboxPlugin;

impl Plugin for LowerBodySandboxPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LowerBodyParams>()
            // Step the resolver on the fixed physics clock (90 Hz) instead of the
            // render-frame Δt, so its grounded spring can never see a coarse dt.
            .insert_resource(Time::<Fixed>::from_hz(SANDBOX_PHYSICS_HZ))
            .add_systems(FixedUpdate, drive_lower_body)
            .add_systems(Update, reset_lower_body);
    }
}
