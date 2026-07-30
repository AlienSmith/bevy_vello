use bevy::prelude::*;

/// Generic wrapper event that acts as a deferred channel.
///
/// Systems write `ChannelMessage<T>` via `EventWriter<ChannelMessage<T>>`
/// instead of calling `commands.trigger_targets` directly.
/// The [`process_channel_system`](crate::death_channel::process_channel_system)
/// dispatches them at a controlled point in the frame schedule via
/// `commands.trigger_targets`.
#[derive(Event)]
pub struct ChannelMessage<T: Event> {
    pub target: Entity,
    pub payload: T,
}

/// Event payload: detach a part from its character.
///
/// When triggered on a body-part entity via `commands.trigger_targets`,
/// the `on_bodypart_detach` observer inserts `Health{1,1}` + [`Detached`]
/// marker, making it a free physics body that can be killed with a second hit.
#[derive(Event, Clone)]
pub struct Detach {
    pub character: Entity,
}
