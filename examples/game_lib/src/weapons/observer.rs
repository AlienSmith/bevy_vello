use bevy::prelude::*;
use bevy_vello::collision::VelloCollisionTrigger;

use crate::{
    character::Connectivity,
    health::Health,
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
    quey_c: Query<&Connectivity>,
    mut health_q: Query<&mut Health>,
) {
    let event = trigger.event();
    // let pos = event.collision_point;
    // let mut scene = VelloScene::default();
    // scene.push_instance_with_transforms(&[]);
    // scene.fill(
    //     peniko::Fill::NonZero,
    //     kurbo::Affine::default(),
    //     peniko::Color::rgba(1.0, 0.0, 0.0, 0.5),
    //     None,
    //     &kurbo::Circle::new((0.0, 0.0), 20.0),
    // );
    // scene.pop_instance();

    // commands.spawn((
    //     VelloSceneBundle {
    //         scene,
    //         transform: Transform::from_translation(pos.extend(100.0)),
    //         ..Default::default()
    //     },
    //     ExplosionEffect::new(
    //         particles::GravityParticleConfig {
    //             gravity: Vec2::new(0.0, -98.0),
    //             drag: 0.0,
    //             persistent: false,
    //         },
    //         particles::BurstEmitterConfig {
    //             count: 100,
    //             speed_range: (10.0, 100.0),
    //             lifetime_range: (1.0, 1.2),
    //             origin: Vec2::new(0.0, 0.0),
    //         },
    //         500,
    //     ),
    // ));
    commands.entity(trigger.target()).despawn();
    // Colliders are spawned with Health of 2 in assemble_character. Each bullet
    // hit reduces health by 1. This must happen regardless of Connectivity:
    // after the first hit detaches the collider (via UnregisterPart) its
    // Connectivity is removed, but Health stays, so the second hit can still
    // drive health to zero and fire the Die event that despawns the entity.
    if let Ok(mut health) = health_q.get_mut(event.entity_other) {
        health.current -= 1.0;
    }
    // Only detach the collider from its character on the first hit (while it
    // still has Connectivity). After this it becomes a free body.
    if let Ok(c) = quey_c.get(event.entity_other) {
        commands
            .spawn((
                DelayedEvent::new(0.05),
                UnregisterPartPayload {
                    character: c.character,
                    part: event.entity_other,
                },
            ))
            .observe(on_delayed_unregister);
    }
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
