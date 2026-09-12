use bevy::prelude::*;
use bevy_vello::{
    collision::{CollisionEventBatch, CollisionOverride, VelloCollisionTrigger},
    VelloRayTraceTrigger,
};

use crate::{
    character::Connectivity,
    damage::{
        components::{AttackStats, Damageable, TotalHealth},
        resolve::resolve_damage,
    },
    death_channel::{channel::ChannelMessage, components::Detached},
    health::{Die, Health},
    weapons::{MeleeWeapon, RayTraceHitPoints},
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
    //    Modeled as a single `striking_force` applied on the velocity channel.
    //    No explosion hack (would inject a massive absolute kick and wall the
    //    target); real mass is kept on both sides so the target `gives` instead
    //    of being rocket-launched.
    let entry = &mut batch.entries[event.batch_index];
    let weapon_override = CollisionOverride {
        explosion_impulse: None,
        velocity_scale: Some(melee.striking_force),
        inv_mass_scale: Some(1.0),
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

/// Observer fired when a character **body part** collides with another
/// character's **body part** (character-vs-character melee).
///
/// Fired once per side (both entities are character parts, so this runs for
/// each entity in the pair). It differs from [`on_collision_melee`] in two ways:
///
/// * **Identity gate:** both entities must carry [`Connectivity`] and belong to
///   *different* characters. Same character (e.g. own arm hitting own torso) is
///   treated as self-collision and skipped. Detached parts have no `Connectivity`,
///   so they are naturally excluded.
///
/// * **Mutual impact:** because this runs on both sides, each invocation strikes
///   the *opponent* using the *this* side as the attacker — so both characters
///   take damage, scaled by the opponent body part's importance via
///   [`resolve_damage`] (`PartKind::Vital` can kill, `NonVital` detaches).
///
/// Physics intent from **both** parts ([`MeleeWeapon`]) is written into the
/// symmetric per-pair [`CollisionOverride`] slots (`override_a` / `override_b`),
/// since both sides are striking parts — unlike [`on_collision_melee`], where one
/// side is the weapon. Each part carries a single [`MeleeWeapon::striking_force`],
/// so both slots are populated so the collision is pushed symmetrically on each
/// side independently.
pub(crate) fn on_collision_character(
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
    info!(
        "[char_collision] fired self={:?} other={:?} batch={}",
        event.entity_self, event.entity_other, event.batch_index
    );

    // ── Identity gate: both sides must be parts of DIFFERENT characters ──
    let Ok(self_conn) = connectivity_q.get(event.entity_self) else {
        return;
    };
    let Ok(other_conn) = connectivity_q.get(event.entity_other) else {
        return;
    };
    if self_conn.character == other_conn.character {
        info!("[char_collision] self-collision, skipping");
        return; // self-collision (own arm vs own torso) — ignore
    }
    info!(
        "[char_collision] chars self={:?} other={:?}",
        self_conn.character, other_conn.character
    );

    // ── Physics: MUTUAL mild repulsion — BOTH characters get nudged ──
    //    In `make_collision_constraints`, the two override slots are resolved
    //    like this:
    //      * override_a → fed with A's inv-mass → determines how B gets shoved.
    //      * override_b → fed with B's inv-mass → determines how A gets shoved.
    //    So to make BOTH characters move we populate BOTH slots.
    //
    //    A character-vs-character hit is a TWO-BODY exchange: each part is both
    //    striker and target. We use the VELOCITY branch (no `explosion_impulse`)
    //    and each part's single `MeleeWeapon.striking_force` as the velocity
    //    scale. Both sides keep real mass (`inv_mass_scale = 1.0`), so the blow
    //    scales existing momentum — a graze stays a graze, no fly-away.
    let entry = &mut batch.entries[event.batch_index];

    // Binding re-used below for damage. Each side shoves the other with ITS OWN
    // striking_force (default: a mild nudge if a part has no MeleeWeapon).
    let self_force = melee_q
        .get(event.entity_self)
        .map(|m| m.striking_force)
        .unwrap_or(1.4);
    let other_force = melee_q
        .get(event.entity_other)
        .map(|m| m.striking_force)
        .unwrap_or(1.4);

    let shove = |force: f32| CollisionOverride {
        explosion_impulse: None, // no wall hack → no fly-away
        velocity_scale: Some(force),
        inv_mass_scale: Some(1.0), // both keep real mass → mutual give
    };

    // override_a determines how A gets B shoved (A's striking force applies to
    // B); override_b determines how B gets A shoved (B's striking force applies
    // to A). The self/other roles flip between the two invocations.
    let (override_a, override_b) = if event.entity_self == entry.event.entity_a {
        (shove(self_force), shove(other_force))
    } else {
        (shove(other_force), shove(self_force))
    };

    entry.override_a = override_a;
    entry.override_b = override_b;
    info!(
        "[char_collision] self={:?} wrote override_a={} override_b={} (A pushes B with A's force, B pushes A with B's force)",
        event.entity_self, self_force, other_force
    );

    // ── Damage: this side's part strikes the opponent's part ──
    //    `other_conn.character` is copied (Entity is Copy) before the mutable
    //    query borrows below, so the immutable connectivity borrow ends here.
    let opponent_root = other_conn.character;
    let self_melee = melee_q.get(event.entity_self).ok();
    let Ok(mut target_part) = part_q.get_mut(event.entity_other) else {
        return;
    };
    let Ok(mut total) = total_q.get_mut(opponent_root) else {
        return;
    };
    resolve_damage(
        opponent_root,
        event.entity_other,
        self_melee
            .map(|m| m.attack_stats)
            .unwrap_or_else(|| AttackStats::new(30.0, 0.0, 5.0)),
        None,
        &mut target_part,
        &mut total,
        &mut die_writer,
        &mut unreg_writer,
    );
}

/// Observer fired when a [`VelloRayTraceTrigger`] is triggered on a pistol entity.
/// Stores the hit point so that [`update_pistol_aim`] can draw a red dot in the
/// pistol's own [`VelloScene`] on the next frame.
pub(crate) fn on_raytrace_hit(
    trigger: Trigger<VelloRayTraceTrigger>,
    mut hit_points: ResMut<RayTraceHitPoints>,
) {
    let event = trigger.event();
    let entity = event.source_entity;
    if let Some(hit_entity) = event.hit_entity {
        let _ = hit_entity; // keep for clarity
        hit_points.0.insert(entity, event.hit_point);
    } else {
        hit_points.0.remove(&entity);
    }
}
