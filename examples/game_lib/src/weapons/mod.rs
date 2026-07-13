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
}

impl Default for PistolControl {
    fn default() -> Self {
        Self {
            wrist_binding_uv: Vec2::ZERO,
            gun_point_uv: Vec2::ZERO,
        }
    }
}

#[derive(Event)]
pub struct AttachPistolToCharacterEvent {
    pub character: Entity,
    pub pistol: Entity,
}
// ---------------------------------------------------------------------------
// System: update_pistol_aim
// ---------------------------------------------------------------------------

/// Updates the character's aim target based on the pistol's current orientation.
///
/// Every frame this system:
///
/// 1. Reads the wrist (PRLA) and elbow (P13) particle positions from the
///    physics world (in Vello coordinates: x-right, y-down).
///
/// 2. Computes the elbow binding point on the fly:
///    `elbow_binding_local = wrist_binding_point + (elbow_wrist_distance, 0)`
///    Since both the pistol and arm point along the X-axis, the elbow-wrist
///    distance determines how far along the pistol the elbow binds.
///
/// 3. Derives the pistol's world orientation from the wrist→elbow direction.
///
/// 4. Transforms the `gun_point` to world space and writes it to the
///    character's `RightArmController.target`.
///
/// The target is output in **Bevy coordinates** (x-right, y-up). The IK
/// system's [`calculate_arm_ik`] converts it to Vello coordinates internally
/// via `bevy_to_vello`.
pub fn update_pistol_aim(
    pistol_q: Query<(Entity, &PistolControl, &Connectivity)>,
    root_q: Query<&ConnectivityRoot>,
    particle_q: Query<&VelloParticle>,
    mut arm_q: Query<&mut RightArmController>,
    string_pool: Res<StringPool>,
) {
    for (_pistol_entity, control, connectivity) in &pistol_q {
        let character = connectivity.character;

        let Ok(root) = root_q.get(character) else {
            continue;
        };

        // Resolve PRLA (wrist) and P13 (elbow) particle entities
        let prla_name = string_pool.pool.intern("PRLA");
        let p13_name = string_pool.pool.intern("P13");

        let Some(&prla_entity) = root.parts.get(&prla_name) else {
            continue;
        };
        let Some(&p13_entity) = root.parts.get(&p13_name) else {
            continue;
        };

        let Ok(prla_particle) = particle_q.get(prla_entity) else {
            continue;
        };
        let Ok(p13_particle) = particle_q.get(p13_entity) else {
            continue;
        };

        // Particle positions are in Vello coordinates (x-right, y-down)
        let wrist_pos = prla_particle.particle.pos;
        let elbow_pos = p13_particle.particle.pos;

        // Elbow-wrist distance (in Vello space)
        let elbow_wrist_dist = (elbow_pos - wrist_pos).length();
        if elbow_wrist_dist <= f32::EPSILON {
            continue;
        }

        // The pistol's world direction is the wrist→elbow direction
        // (since the arm always points along the pistol's X-axis)
        let pistol_dir = (elbow_pos - wrist_pos) / elbow_wrist_dist;

        // Gun point in world space (Vello coordinates)
        // The offset along the pistol X-axis from wrist_binding to gun_point
        let barrel_offset = control.gun_point_uv.x - control.wrist_binding_uv.x;
        let gun_point_world_vello = wrist_pos + pistol_dir * barrel_offset;

        // Convert to Bevy coordinates (y-up) for the IK system
        // The IK system's calculate_arm_ik does bevy_to_vello internally,
        // so we need to give it Bevy coordinates.
        let gun_point_bevy = Vec2::new(gun_point_world_vello.x, -gun_point_world_vello.y);

        // Write the aim target to the character's right arm controller.
        // The IK mode (Aim / Reach / Disabled) is controlled separately by
        // the player's C key in `player_movement` — we only set the target
        // position here so the arm knows where to aim when IK is active.
        if let Ok(mut arm) = arm_q.single_mut() {
            arm.target = gun_point_bevy;
        }
    }
}
