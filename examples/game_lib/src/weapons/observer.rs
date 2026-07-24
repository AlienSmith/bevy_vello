use bevy::prelude::*;
use bevy_vello::collision::VelloCollisionTrigger;

use crate::{
    character::Connectivity,
    damage::{
        components::{AttackStats, Damageable, TotalHealth},
        resolve::resolve_damage,
    },
    utility::{DelayedEvent, DelayedEventTrigger},
    CharacterPartEvent,
};

/// Payload attached to a delayed-event entity. Read by the observer
/// when the timer fires to know which part to unregister.
#[derive(Component)]
pub(crate) struct UnregisterPartPayload {
    character: Entity,
    part: Entity,
}

pub(crate) fn on_collision_bullet(
    trigger: Trigger<VelloCollisionTrigger>,
    mut commands: Commands,
    mut part_q: Query<(Entity, &mut Damageable, &Connectivity)>,
    mut total_q: Query<&mut TotalHealth>,
    bullet_q: Query<&AttackStats>,
) {
    let event = trigger.event();
    // The bullet entity that triggered the collision carries the attack stats.
    let Ok(attack_stats) = bullet_q.get(event.entity_self) else {
        return;
    };
    // The part entity that was hit (entity_other).
    let Ok((part_entity, mut part, connectivity)) = part_q.get_mut(event.entity_other) else {
        return;
    };
    // The character this part belongs to — TotalHealth lives on the root.
    let Ok(mut total) = total_q.get_mut(connectivity.character) else {
        return;
    };
    let character_root = connectivity.character;

    // Resolve damage through the unified two-channel model. `resolve_damage`
    // handles the Die event on the CharacterRoot and part detachment. Armor is
    // not yet attached (pending the particle-binding design), so pass `None`.
    resolve_damage(
        &mut commands,
        character_root,
        part_entity,
        *attack_stats,
        None,
        &mut part,
        &mut total,
    );

    // Detach the collider from its character after the hit so it becomes a free
    // body (it will fly away via UnregisterPart).
    commands
        .spawn((
            DelayedEvent::new(0.05),
            UnregisterPartPayload {
                character: character_root,
                part: part_entity,
            },
        ))
        .observe(on_delayed_unregister);
}

pub(crate) fn on_delayed_unregister(
    trigger: Trigger<DelayedEventTrigger>,
    q: Query<&UnregisterPartPayload>,
    mut events: EventWriter<CharacterPartEvent>,
) {
    let Ok(payload) = q.get(trigger.target()) else {
        return;
    };
    events.write(CharacterPartEvent::UnregisterPart {
        character: payload.character,
        part: payload.part,
    });
}
