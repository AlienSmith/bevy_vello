use bevy::prelude::*;
use leafwing_input_manager::prelude::*;

/// Actions that can be triggered by player input (keyboard, gamepad, or AI via `seldom_state`).
#[derive(Actionlike, PartialEq, Eq, Clone, Copy, Hash, Debug, Reflect)]
pub enum PlayerAction {
    MoveUp,
    MoveDown,
    MoveLeft,
    MoveRight,
    Aim,
    Fire,
    DropWeapon,
}

/// Returns the default key bindings for [`PlayerAction`].
pub fn default_input_map() -> InputMap<PlayerAction> {
    let mut input_map = InputMap::<PlayerAction>::default();
    input_map.insert(PlayerAction::MoveUp, KeyCode::KeyW);
    input_map.insert(PlayerAction::MoveDown, KeyCode::KeyS);
    input_map.insert(PlayerAction::MoveLeft, KeyCode::KeyA);
    input_map.insert(PlayerAction::MoveRight, KeyCode::KeyD);
    input_map.insert(PlayerAction::Aim, KeyCode::KeyC);
    input_map.insert(PlayerAction::Fire, KeyCode::KeyF);
    input_map.insert(PlayerAction::DropWeapon, KeyCode::KeyV);
    input_map
}
