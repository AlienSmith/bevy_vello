use bevy::prelude::*;
use bevy_vello::integrations::physics::VelloParticle;

use crate::{character::Connectivity, ConnectivityRoot, RightArmController, StringPool};
pub mod plugin;
mod system;
// ---------------------------------------------------------------------------
// PistolControl Component
// ---------------------------------------------------------------------------

/// Controls the attachment and aiming of a pistol for a character.
///
/// The pistol always points along the **X-axis** of its local space, and the
/// arm always points along the **X-axis** of the pistol's local space too.
///
/// # Fields
///
/// * `wrist_binding_point` — Attachment point of the wrist (PRLA) to the
///   pistol, in the pistol's local space. The wrist particle is constrained
///   to this point via a Bilinear joint.
///
/// * `gun_point` — Muzzle/tip of the gun in the pistol's local space.
///   The direction from `wrist_binding_point` to `gun_point` defines the
///   pistol's barrel direction, which is used to compute the aim target
///   for the character's `RightArmController`.
///
/// The elbow (P13) binding point is **calculated on the fly** because the
/// elbow–wrist distance varies per character.
#[derive(Component, Clone)]
pub struct PistolControl {
    /// Attachment point of the wrist (PRLA) in pistol local space.
    pub wrist_binding_uv: Vec2,
    /// Muzzle/tip of the gun in pistol local space.
    pub gun_point_uv: Vec2,
    ///
    pub world_aim_trarget: Option<Vec2>,
}

impl Default for PistolControl {
    fn default() -> Self {
        Self {
            wrist_binding_uv: Vec2::ZERO,
            gun_point_uv: Vec2::ZERO,
            world_aim_trarget: None,
        }
    }
}

#[derive(Event)]
pub struct AttachPistolToCharacterEvent {
    pub character: Entity,
    pub pistol: Entity,
}
