use bevy::prelude::*;
use bevy_vello::collision::{CollisionEventBatch, CollisionOverride, VelloCollisionTrigger};

use crate::{
    character::Connectivity,
    damage::{
        components::{AttackStats, Damageable, TotalHealth},
        resolve::resolve_damage,
    },
    death_channel::{channel::ChannelMessage, components::Detached},
    health::{Die, Health},
    weapons::MeleeWeapon,
    CharacterPartEvent,
};

/// Observer fired when a bullet collides with something.
///
/// Reduces [`Damageable`] on the hit entity via `resolve_damage` when it is
/// still connected to a character (has [`Connectivity`] — the character's
/// [`TotalHealth`] is drained as well).  For detached body parts (with
/// [`Health`]) the damage is applied directly to both `Damageable` and `Health`;
/// [`check_health`] then handles writing `ChannelMessage<Die>` when `Health`
/// reaches zero, which triggers the second-hit observer.
///
/// This keeps the damage *math* unified — the cut/blunt channel model is
/// applied in both cases via [`resolve_damage`] or the inline path, but the
/// fan-out (character‑level death, unregister commands) only happens in the
/// connected path.
pub(crate) fn on_collision_bullet(
    trigger: Trigger<VelloCollisionTrigger>,
    mut part_q: Query<&mut Damageable>,
    mut total_q: Query<&mut TotalHealth>,
    connectivity_q: Query<&Connectivity>,
    mut detached_q: Query<&mut Health, (With<Detached>, Without<Connectivity>)>,
    bullet_q: Query<&AttackStats>,
    mut die_writer: EventWriter<ChannelMessage<Die>>,
    mut unreg_writer: EventWriter<CharacterPartEvent>,
) {
    let event = trigger.event();
    let Ok(attack_stats) = bullet_q.get(event.entity_self) else {
        return;
    };
    let target = event.entity_other;

    let Ok(mut part) = part_q.get_mut(target) else {
        return;
    };

    // ── Connected path ──────────────────────────────────────────────────
    if let Ok(connectivity) = connectivity_q.get(target) {
        let Ok(mut total) = total_q.get_mut(connectivity.character) else {
            return;
        };
        resolve_damage(
            connectivity.character,
            target,
            *attack_stats,
            None,
            &mut part,
            &mut total,
            &mut die_writer,
            &mut unreg_writer,
        );
        return;
    }

    // ── Detached path ───────────────────────────────────────────────────
    // No Connectivity → free physics body with its own Health.
    // Simple cut‑only reduction; blunt/protection gating is skipped for
    // detached parts.  `check_health` will fire `ChannelMessage<Die>`
    // when `Health.current <= 0`.
    if let Ok(mut health) = detached_q.get_mut(target) {
        let cut = attack_stats.cut_damage.min(part.current);
        part.current -= cut;
        health.current -= attack_stats.cut_damage - cut;
    }
}

/// Observer fired when a melee weapon collides with something.
///
/// Writes game-level intent into the [`CollisionEventBatch`] via `batch_index`
/// so that `make_collision_constraints` (in the next FixedUpdate) can resolve
/// per-pair overrides into actual physics parameters.
///
/// Also resolves damage via the existing damage system.
pub(crate) fn on_collision_melee(
    trigger: Trigger<VelloCollisionTrigger>,
    mut batch: ResMut<CollisionEventBatch>,
    melee_q: Query<&MeleeWeapon>,
    mut part_q: Query<&mut Damageable>,
    mut total_q: Query<&mut TotalHealth>,
    connectivity_q: Query<&Connectivity>,
    mut die_writer: EventWriter<ChannelMessage<Die>>,
    mut unreg_writer: EventWriter<CharacterPartEvent>,
) {
    let event = trigger.event();
    let Ok(melee) = melee_q.get(event.entity_self) else {
        return;
    };

    // 1. Write game-level intent into the per-pair batch entry.
    //    Determine whether the weapon is entity_a or entity_b in the
    //    batch entry, then write to the matching override field.
    let entry = &mut batch.entries[event.batch_index];
    let weapon_override = CollisionOverride {
        explosion_impulse: Some(melee.explosion_impulse),
        velocity_scale: Some(melee.velocity_scale),
        inv_mass_scale: Some(melee.inv_mass_scale),
    };
    if event.entity_self == entry.event.entity_a {
        entry.override_a = weapon_override;
    } else if event.entity_self == entry.event.entity_b {
        entry.override_b = weapon_override;
    }

    // 2. Resolve damage (same pattern as bullet)
    let target = event.entity_other;
    let Ok(mut part) = part_q.get_mut(target) else {
        return;
    };

    if let Ok(connectivity) = connectivity_q.get(target) {
        let Ok(mut total) = total_q.get_mut(connectivity.character) else {
            return;
        };
        resolve_damage(
            connectivity.character,
            target,
            melee.attack_stats,
            None,
            &mut part,
            &mut total,
            &mut die_writer,
            &mut unreg_writer,
        );
    }
}
