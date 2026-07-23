pub mod actions;
pub mod mouse;
pub mod systems;

use bevy::prelude::*;
use leafwing_input_manager::prelude::*;

use self::actions::PlayerAction;
use self::mouse::update_mouse_world_position;
use self::systems::read_player_input;

pub use self::actions::default_input_map;
pub use self::systems::Player;

/// Plugin that sets up the `leafwing-input-manager` pipeline for character
/// control. Adds mouse world-position tracking and the system that translates
/// [`ActionState<PlayerAction>`] into writes to controller components.
///
/// The [`Player`] marker component is used by [`read_player_input`] to identify
/// which entity to read input for. Attach it to the character entity along with
/// an [`InputManagerBundle<PlayerAction>`].
pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<mouse::MouseWorldPosition>()
            .add_plugins(InputManagerPlugin::<PlayerAction>::default())
            .add_systems(PreUpdate, update_mouse_world_position)
            .add_systems(Update, read_player_input);
    }
}
