use bevy::prelude::*;

use crate::{
    death_channel::{
        channel::{ChannelMessage, Detach},
        systems::process_channel_system,
    },
    health::Die,
    GameLabSystems,
};

/// Registers the [`ChannelMessage<T>`] events and the generic
/// [`process_channel_system`] processor.
///
/// Must run AFTER any system that writes `ChannelMessage<T>` so
/// that all channel messages queued during a frame are flushed
/// in a controlled order.
pub struct DeathChannelPlugin;

impl Plugin for DeathChannelPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<ChannelMessage<Die>>()
            .add_event::<ChannelMessage<Detach>>()
            .add_systems(
                Update,
                (
                    process_channel_system::<Die>,
                    process_channel_system::<Detach>,
                )
                    .chain()
                    .in_set(GameLabSystems::ProcessDeathEvents),
            );
    }
}
