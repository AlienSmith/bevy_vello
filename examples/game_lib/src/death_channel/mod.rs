pub mod channel;
pub mod components;
pub mod observers;
pub mod plugin;
pub mod systems;

pub use channel::{ChannelMessage, Detach};
pub use components::Detached;
pub use plugin::DeathChannelPlugin;
pub use systems::process_channel_system;
