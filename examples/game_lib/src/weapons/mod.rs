use std::collections::HashMap;

use bevy::prelude::*;
use bevy_vello::integrations::physics::VelloParticle;

use crate::{
    character::Connectivity, damage::components::AttackStats, ConnectivityRoot, RightArmController,
    StringPool,
};
pub(crate) mod observer;
pub mod plugin;
mod system;

/// Stores the latest ray trace hit point per pistol entity.
/// Written by [`observer::on_raytrace_hit`], read by [`system::update_pistol_aim`].
#[derive(Resource, Default)]
pub(crate) struct RayTraceHitPoints(pub HashMap<Entity, Vec2>);

// ---------------------------------------------------------------------------
// MeleeWeapon Component
// ---------------------------------------------------------------------------

/// Component for melee weapon entities.
/// Carries the game-level intent for collision response modification.
///
/// A melee hit is modelled as a single, intuitive `striking_force` scalar:
/// how hard this weapon shoves its target on contact.
///   * 0.0  = no push (target feels the blow physically but isn't knocked).
///   * 1.0  = neutral: the push is exactly proportional to the target's motion.
///   * >1.0 = extra-hard knockback.
///
/// It is applied purely on the VELOCITY channel of the collision override
/// (`velocity_scale = Some(striking_force)`) with real mass kept on both sides
/// (`inv_mass_scale = Some(1.0)`) and no explosion hack — so it never
/// launches a target instantly. Because it scales existing momentum, a
/// stationary target reacts less than a moving one; that is the documented
/// trade-off of the single-scale model.
///
/// The override is written to the [`CollisionEventBatch`] resource (indexed by
/// `batch_index` on the [`VelloCollisionTrigger`]) by the melee collision
/// observer, then consumed by `make_collision_constraints` in the next
/// FixedUpdate.
#[derive(Component, Clone)]
pub struct MeleeWeapon {
    /// How hard this weapon shoves its target. See the type-level docs.
    pub striking_force: f32,

    /// Attack stats for damage resolution.
    pub attack_stats: AttackStats,
}

impl Default for MeleeWeapon {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl MeleeWeapon {
    /// Build a melee weapon with the given knockback scale and a default
    /// attack profile (`blunt 30, cut 0, penetration 5`).
    #[must_use]
    pub fn new(striking_force: f32) -> Self {
        Self {
            striking_force,
            attack_stats: AttackStats::new(30.0, 0.0, 5.0),
        }
    }
}

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
    ///
    pub enable_aim_line: bool,

    pub last_fire_time: f32,
    pub fire_cool_down: f32,

    pub recoil_kickup: f32,
    pub recoil_kick_scale: f32,
}

impl Default for PistolControl {
    fn default() -> Self {
        Self {
            wrist_binding_uv: Vec2::ZERO,
            gun_point_uv: Vec2::ZERO,
            world_aim_trarget: None,
            enable_aim_line: true,
            last_fire_time: 0.0,
            fire_cool_down: 0.5,
            recoil_kickup: 0.5,
            recoil_kick_scale: 300.0,
        }
    }
}

#[derive(Event)]
pub struct AttachPistolToCharacterEvent {
    pub character: Entity,
    pub pistol: Entity,
}

#[derive(Event)]
pub struct FireEvent {
    pub weapon: Entity,
    pub projectile_collision_group: u32, //this should be consistent with the user
}

#[derive(Component, Default, Clone, Copy)]
pub struct Bullet;
