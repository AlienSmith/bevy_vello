use bevy::prelude::*;
use bevy_vello::integrations::physics::VelloParticle;

use crate::{
    character::Connectivity, damage::components::AttackStats, ConnectivityRoot, RightArmController,
    StringPool,
};
mod observer;
pub mod plugin;
mod system;

// ---------------------------------------------------------------------------
// MeleeWeapon Component
// ---------------------------------------------------------------------------

/// Component for melee weapon entities.
/// Carries the game-level intent for collision response modification.
///
/// The intent fields (`explosion_impulse`, `velocity_scale`, `inv_mass_scale`)
/// are written to `VelloCollider.collision_override` by the melee collision
/// observer, then consumed by `make_collision_constraints` in the next FixedUpdate.
#[derive(Component, Clone)]
pub struct MeleeWeapon {
    /// Desired "explosion" impulse at the contact point.
    /// This is a non-physical energy injection — like a tiny explosion
    /// that pushes the body part away from the weapon.
    /// Specified as a world-space impulse vector (force * time).
    pub explosion_impulse: Vec2,

    /// Scale factor for the opponent's velocity contribution.
    /// 1.0 = use actual opponent velocity.
    /// 0.0 = treat opponent as static.
    /// >1.0 = amplify opponent velocity (heavier feel).
    pub velocity_scale: f32,

    /// Scale factor for the opponent's inverse mass.
    /// 0.0 = treat opponent as infinitely heavy (like bullet hack).
    /// 1.0 = use actual opponent inv_mass.
    pub inv_mass_scale: f32,

    /// Attack stats for damage resolution.
    pub attack_stats: AttackStats,
}

impl Default for MeleeWeapon {
    fn default() -> Self {
        Self {
            explosion_impulse: Vec2::ZERO,
            velocity_scale: 1.0,
            inv_mass_scale: 0.0, // default: heavy hit like bullet
            attack_stats: AttackStats::new(30.0, 10.0, 5.0),
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
