use bevy::prelude::*;

use crate::death_channel::channel::ChannelMessage;

/// Generic channel processor. Reads `EventReader<ChannelMessage<T>>` and
/// dispatches each message via `commands.trigger_targets`, converting the
/// deferred event back into an immediate observer trigger on the target entity.
///
/// Instantiated for each event type `T` (e.g. [`Die`], [`Detach`]).
pub fn process_channel_system<T: Event + Clone>(
    mut events: EventReader<ChannelMessage<T>>,
    mut commands: Commands,
) {
    for msg in events.read() {
        commands.trigger_targets(msg.payload.clone(), msg.target);
    }
}
