use bevy::prelude::*;
use bevy_vello::{
    collision::{CollisionOverride, VelloCollisionTrigger},
    VelloCollider,
};

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

    /// Observer fired when a melee weapon collides with something.
    /// Writes game-level intent to VelloCollider.collision_override for next frame's physics.
    /// Also resolves damage via the existing damage system.
    pub(crate) fn on_collision_melee(
        trigger: Trigger<VelloCollisionTrigger>,
        mut collider_q: Query<&mut VelloCollider>,
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

        // 1. Write game-level intent directly to the weapon's VelloCollider.
        //    make_collision_constraints will resolve this into actual
        //    other_inv_mass and other_velocity using the physics state.
        if let Ok(mut collider) = collider_q.get_mut(event.entity_self) {
            collider.collision_override = CollisionOverride {
                explosion_impulse: Some(melee.explosion_impulse),
                velocity_scale: Some(melee.velocity_scale),
                inv_mass_scale: Some(melee.inv_mass_scale),
            };
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
}
