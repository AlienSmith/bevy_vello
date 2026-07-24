pub mod components;
pub mod resolve;

use bevy::prelude::*;

use crate::{character_factory::CharacterRoot, health::Die};

pub use self::components::{AttackStats, Damageable, PartKind, TotalHealth};
pub use self::resolve::resolve_damage;

/// Plugin that registers the damage/health components and the system that
/// watches [`TotalHealth`] on the `CharacterRoot` to fire [`Die`].
///
/// Mirrors `input_management` layout: a standalone module folder with a
/// `plugin` registration point and submodules for components and the
/// resolution helper.
pub struct DamagePlugin;

impl Plugin for DamagePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, check_total_health);
    }
}

/// Watches `TotalHealth` on the `CharacterRoot`. When `current <= 0`, fires
/// the existing [`Die`] event on the `CharacterRoot` (the entity carrying
/// `VelloCharacterPhysicsRoot` + controllers), mirroring the single-death
/// design: parts/armor never fire `Die`; they detach via `UnregisterPart`.
pub fn check_total_health(
    mut commands: Commands,
    total_q: Query<(Entity, &TotalHealth), With<CharacterRoot>>,
) {
    for (entity, total) in total_q.iter() {
        if total.current <= 0.0 {
            commands.trigger_targets(Die, entity);
        }
    }
}
