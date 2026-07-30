use bevy::prelude::*;

use crate::{death_channel::channel::ChannelMessage, GameLabSystems};

/// Tracks an entity's current and maximum health.
///
/// Attach this to any entity that can take damage. A separate system
/// (usually the one that applies damage) is responsible for mutating
/// `current`; [`check_health`] watches for the `current <= 0` condition
/// and fires the generic [`Die`] trigger.
#[derive(Component, Clone)]
pub struct Health {
    pub current: f32,
    pub max: f32,
}

impl Health {
    pub fn new(max: f32) -> Self {
        Self { current: max, max }
    }
}

/// Empty marker event triggered on an entity when its [`Health`] drops
/// to zero or below.
///
/// Because it carries no payload, any entity can attach its own observer
/// (`Trigger<Die>`) and use `trigger.target()` to discover which entity
/// died, letting different entities hook different death logic without a
/// shared payload. This mirrors the existing `Trigger`/`.observe()` idiom
/// used for `VelloCollisionTrigger` and `DelayedEventTrigger`.
#[derive(Event, Clone)]
pub struct Die;

/// Watches every entity with [`Health`]. When `current <= 0`, removes
/// `Health` (so the entity is not re-detected — guarantees single-fire) and
/// then fires [`Die`] on that entity via `trigger_targets` so per-entity
/// observers (e.g. one that despawns the entity) can react.
///
/// `Health` is removed *before* `Die` is triggered so that any despawn
/// performed by a `Die` observer runs after this command in the same flush,
/// avoiding a "entity does not exist" error from a redundant separate
/// `remove` command executing after the despawn.
pub fn check_health(
    mut commands: Commands,
    health_q: Query<(Entity, &Health)>,
    mut die_writer: EventWriter<ChannelMessage<Die>>,
) {
    for (entity, health) in health_q.iter() {
        if health.current <= 0.0 {
            commands.entity(entity).remove::<Health>();
            die_writer.write(ChannelMessage {
                target: entity,
                payload: Die,
            });
        }
    }
}

/// Plugin that registers the [`Die`] event and the [`check_health`] system.
pub struct HealthPlugin;

impl Plugin for HealthPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<Die>()
            .add_systems(Update, check_health.in_set(GameLabSystems::CheckHealth));
    }
}
