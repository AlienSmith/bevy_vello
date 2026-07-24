use bevy::prelude::*;

/// Total health for the whole character, attached to the `CharacterRoot`.
///
/// Blunt damage (after passing the `penetration` vs `protection_level` gate)
/// propagates here via each part's `blunt_ratio`. When `current <= 0` the
/// character dies — the [`crate::health::Die`] event is fired on the
/// `CharacterRoot` entity (the same entity carrying `VelloCharacterPhysicsRoot`
/// + the controllers).
#[derive(Component, Clone)]
pub struct TotalHealth {
    pub current: f32,
    pub max: f32,
}

impl TotalHealth {
    pub fn new(max: f32) -> Self {
        Self { current: max, max }
    }
}

/// Unified damage component for BOTH body parts and armor.
///
/// Body parts use `protection_level = 0`; armor uses `protection_level > 0`.
/// They are distinguished only by field values, not by type.
///
/// * `cut_damage` is applied **uniformly** to the armor (degrades it) then the
///   body part — there is no penetration gate on cut.
/// * `blunt_damage` is gated by `penetration` vs `protection_level`; if it
///   passes, a fraction `blunt_ratio` propagates to [`TotalHealth`] and the
///   remainder is absorbed by this part.
#[derive(Component, Clone)]
pub struct Damageable {
    pub current: f32,
    pub max: f32,
    pub kind: PartKind,
    /// 0 for body parts; > 0 for armor (gates BLUNT only).
    pub protection_level: f32,
    /// Fraction of admitted blunt damage that propagates to `TotalHealth`;
    /// the rest `(1 - ratio)` is absorbed by this part.
    pub blunt_ratio: f32,
}

impl Damageable {
    /// Build a body part. Armor pieces use [`Damageable::armor`] instead.
    pub fn part(current: f32, kind: PartKind, blunt_ratio: f32) -> Self {
        Self {
            current,
            max: current,
            kind,
            protection_level: 0.0,
            blunt_ratio,
        }
    }

    /// Build an armor piece (shields part from blunt, absorbs/degrades from cut).
    pub fn armor(current: f32, protection_level: f32) -> Self {
        Self {
            current,
            max: current,
            kind: PartKind::NonVital,
            protection_level,
            blunt_ratio: 0.0,
        }
    }
}

/// Classifies a [`Damageable`] for death/detach behavior.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PartKind {
    /// Head: hitting 0 => instant death + decapitate (Die fired on CharacterRoot).
    Vital,
    /// Limbs/torso/armor: hitting 0 => detach (becomes free body).
    NonVital,
}

/// Attack stats carried by every damaging entity (bullet, fist, weapon, etc.).
///
/// `penetration` gates ONLY the blunt channel; cut damage always applies
/// uniformly to armor/body parts.
#[derive(Component, Clone, Copy)]
pub struct AttackStats {
    /// -> `TotalHealth` (via part `blunt_ratio`), gated by penetration.
    pub blunt_damage: f32,
    /// -> `Damageable.current` (armor then part), NO penetration gate.
    pub cut_damage: f32,
    /// Compared against `protection_level` to admit blunt damage.
    pub penetration: f32,
}

impl AttackStats {
    pub fn new(blunt_damage: f32, cut_damage: f32, penetration: f32) -> Self {
        Self {
            blunt_damage,
            cut_damage,
            penetration,
        }
    }
}
