use bevy::prelude::*;

use crate::{
    character_factory::CharacterPartEvent,
    damage::components::{AttackStats, Damageable, PartKind, TotalHealth},
    death_channel::channel::ChannelMessage,
    health::Die,
};

/// Outcome of resolving a single damage application. Pure data so it can be
/// unit-tested without Bevy command infrastructure.
pub struct DamageResolution {
    /// Remaining cut damage the part absorbed (after armor).
    pub part_cut_remaining: f32,
    /// Admitted blunt damage (after penetration gate). 0 if blocked.
    pub blunt_admitted: f32,
    /// Whether the character should die (total or vital part).
    pub dies: bool,
    /// Whether the part/armor should detach (nonvital depleted).
    pub detaches: bool,
}

/// Pure damage math for a single application. Centralizes the two-channel
/// model so player & AI share identical rules:
///
/// * **Cut channel (uniform, no penetration gate):** cut is applied to the
///   armor first (degrading it), then overflow hits the part.
/// * **Blunt channel (gated):** if `penetration > protection_level`, blunt is
///   admitted — `blunt_ratio` propagates to total health, the rest is absorbed
///   by the part. Otherwise blunt is blocked.
///
/// Returns the resolution; mutates `armor`/`part`/`total` in place.
pub fn apply_damage_math(
    attacker_stats: AttackStats,
    armor: Option<&mut Damageable>,
    part: &mut Damageable,
    total: &mut TotalHealth,
) -> DamageResolution {
    // Compute protection BEFORE consuming `armor` below.
    let protection = armor
        .as_ref()
        .map(|a| a.protection_level)
        .unwrap_or(part.protection_level);

    // Cut channel: UNIFORM, NO penetration gate. Armor absorbs first.
    let part_cut_remaining = match armor {
        Some(armor) => {
            let absorbed = armor.current.min(attacker_stats.cut_damage);
            armor.current -= absorbed;
            attacker_stats.cut_damage - absorbed
        }
        None => attacker_stats.cut_damage,
    };
    part.current -= part_cut_remaining;

    // Blunt channel: GATED by penetration vs protection_level (computed above).
    let blunt_admitted = if attacker_stats.penetration > protection {
        attacker_stats.blunt_damage
    } else {
        0.0
    };
    total.current -= blunt_admitted * part.blunt_ratio;
    part.current -= blunt_admitted * (1.0 - part.blunt_ratio);

    // Trigger conditions.
    let dies = total.current <= 0.0 || (part.kind == PartKind::Vital && part.current <= 0.0);
    let detaches = part.kind == PartKind::NonVital && part.current <= 0.0;

    DamageResolution {
        part_cut_remaining,
        blunt_admitted,
        dies,
        detaches,
    }
}

/// Resolves a single damage application and writes deferred channel messages
/// (ChannelMessage<Die> on the `CharacterRoot` when total HP hits zero, and
/// ChannelMessage<Die> on the part when it detaches).
///
/// Wraps [`apply_damage_math`]; parts/armor never fire `Die` themselves.
/// The channel messages are dispatched by `process_channel_system<T>`
/// in the `ProcessDeathEvents` schedule set.
///
/// Detach also writes a `CharacterPartEvent::UnregisterPart` so the part is
/// disconnected from the character **after** the `Die` observer has a chance
/// to spawn the delayed detach entity.
/// Because `ReadCharacterPartEvent` runs **after** `ProcessDeathEvents` in the
/// schedule, `Connectivity` is still present when the `Die` observer fires.
pub fn resolve_damage(
    character_root: Entity,
    target_part: Entity,
    attacker_stats: AttackStats,
    armor: Option<&mut Damageable>,
    part: &mut Damageable,
    total: &mut TotalHealth,
    die_writer: &mut EventWriter<ChannelMessage<Die>>,
    unreg_writer: &mut EventWriter<CharacterPartEvent>,
) {
    let resolution = apply_damage_math(attacker_stats, armor, part, total);

    if resolution.dies {
        die_writer.write(ChannelMessage {
            target: character_root,
            payload: Die,
        });
    }
    if resolution.detaches {
        // Fire Die on the body part itself so its observer can handle the first hit.
        die_writer.write(ChannelMessage {
            target: target_part,
            payload: Die,
        });
        // Schedule the part for unregistering AFTER the Die observer fires.
        // ReadCharacterPartEvent runs after ProcessDeathEvents in the schedule,
        // so Connectivity is still present when the observer runs.
        unreg_writer.write(CharacterPartEvent::UnregisterPart {
            character: character_root,
            part: target_part,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cut_applies_uniformly_to_armor_then_part() {
        let mut part = Damageable::part(10.0, PartKind::NonVital, 0.5);
        let mut armor = Damageable::armor(5.0, 2.0);
        let mut total = TotalHealth::new(100.0);
        // 8 cut damage: armor absorbs 5 (degrades to 0), 3 overflow hits part.
        let res = apply_damage_math(
            AttackStats::new(0.0, 8.0, 0.0),
            Some(&mut armor),
            &mut part,
            &mut total,
        );
        assert_eq!(armor.current, 0.0);
        assert_eq!(part.current, 7.0);
        assert_eq!(total.current, 100.0); // cut does not touch TotalHealth
        assert!(!res.dies);
        assert!(!res.detaches);
    }

    #[test]
    fn blunt_blocked_when_penetration_below_protection() {
        let mut part = Damageable::part(10.0, PartKind::NonVital, 0.5);
        let mut armor = Damageable::armor(5.0, 2.0);
        let mut total = TotalHealth::new(100.0);
        // penetration 1 <= protection 2 => blunt blocked.
        let res = apply_damage_math(
            AttackStats::new(20.0, 0.0, 1.0),
            Some(&mut armor),
            &mut part,
            &mut total,
        );
        assert_eq!(total.current, 100.0);
        assert_eq!(part.current, 10.0);
        assert_eq!(res.blunt_admitted, 0.0);
    }

    #[test]
    fn blunt_admitted_propagates_via_blunt_ratio() {
        let mut part = Damageable::part(10.0, PartKind::NonVital, 0.5);
        let mut armor = Damageable::armor(0.0, 2.0); // armor already depleted
        let mut total = TotalHealth::new(100.0);
        // penetration 3 > protection 2 => admitted. 20 blunt * 0.5 -> TotalHealth.
        let res = apply_damage_math(
            AttackStats::new(20.0, 0.0, 3.0),
            Some(&mut armor),
            &mut part,
            &mut total,
        );
        assert_eq!(total.current, 90.0);
        assert_eq!(part.current, 0.0); // 20 * (1 - 0.5) = 10 absorbed -> 0
        assert_eq!(res.blunt_admitted, 20.0);
        assert!(res.detaches);
    }

    #[test]
    fn vital_part_zero_sets_death_condition() {
        let mut part = Damageable::part(5.0, PartKind::Vital, 0.0);
        let mut total = TotalHealth::new(100.0);
        // No armor; cut 5 kills vital part.
        let res = apply_damage_math(AttackStats::new(0.0, 5.0, 0.0), None, &mut part, &mut total);
        assert_eq!(part.current, 0.0);
        assert_eq!(total.current, 100.0); // not dead yet, but vital condition met
        assert!(res.dies);
        assert!(!res.detaches);
    }

    #[test]
    fn nonvital_zero_triggers_detach() {
        let mut part = Damageable::part(5.0, PartKind::NonVital, 0.0);
        let mut total = TotalHealth::new(100.0);
        // No armor; cut 5 kills limb.
        let res = apply_damage_math(AttackStats::new(0.0, 5.0, 0.0), None, &mut part, &mut total);
        assert_eq!(part.current, 0.0);
        assert!(res.detaches);
        assert!(!res.dies);
    }
}
