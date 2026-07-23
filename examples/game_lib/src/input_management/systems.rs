use bevy::prelude::*;
use leafwing_input_manager::prelude::*;

use crate::{
    character::{Connectivity, IkMode, RightArmController, SpineController, WhichArm},
    character_factory::CharacterRoot,
    weapons::{FireEvent, PistolControl},
    CharacterController, CharacterPartEvent, ResetArmControlConstraintsEvent,
};

use super::{actions::PlayerAction, mouse::MouseWorldPosition};

/// Marker component for the entity that receives input from [`read_player_input`].
///
/// Attach this to the character entity that should be controlled by the player.
/// The system queries for `ActionState<PlayerAction>` on the entity with this marker.
#[derive(Component, Default)]
pub struct Player;

/// Reads [`ActionState<PlayerAction>`] from each entity marked with [`Player`]
/// and translates it into writes to that entity's own controller components and events.
///
/// **Entity-centric**: iterates ALL entities with `ActionState<PlayerAction>` + `Player` marker.
/// For each entity, only its own `SpineController`, `RightArmController`, and connected
/// pistols are modified.
///
/// | Action | Effect |
/// |--------|--------|
/// | MoveUp/Down/Left/Right | Sets [`SpineController::move_vector`] |
/// | Aim (hold) | Sets [`PistolControl::world_aim_trarget`] to mouse position |
/// | Fire (hold) | Emits [`FireEvent`] (subject to cooldown in `process_fire_event`) |
/// | DropWeapon (press) | Emits [`CharacterPartEvent::UnregisterPart`], disables IK, resets arm constraints |
///
/// Legacy [`CharacterController`] is also updated for backward compatibility.
///
/// Future AI entities will NOT have the [`Player`] marker and will use a different
/// system to write to their own [`ActionState`] and [`PistolControl`].
pub fn read_player_input(
    mouse: Res<MouseWorldPosition>,
    action_state_q: Query<(Entity, &ActionState<PlayerAction>), With<Player>>,
    mut spine_q: Query<&mut SpineController>,
    mut pistol_q: Query<(Entity, &mut PistolControl, &Connectivity)>,
    mut arm_q: Query<&mut RightArmController>,
    mut legacy_q: Query<&mut CharacterController>,
    character_root_q: Query<&CharacterRoot>,
    mut events: EventWriter<CharacterPartEvent>,
    mut fire: EventWriter<FireEvent>,
    mut reset_event: EventWriter<ResetArmControlConstraintsEvent>,
) {
    for (character_entity, action_state) in action_state_q.iter() {
        // ---- Movement ----
        let mut direction = Vec2::ZERO;
        if action_state.pressed(&PlayerAction::MoveUp) {
            direction.y += 50.0;
        }
        if action_state.pressed(&PlayerAction::MoveDown) {
            direction.y -= 50.0;
        }
        if action_state.pressed(&PlayerAction::MoveLeft) {
            direction.x -= 50.0;
        }
        if action_state.pressed(&PlayerAction::MoveRight) {
            direction.x += 50.0;
        }

        // Set movement only on THIS character's spine controller
        if let Ok(mut spine) = spine_q.get_mut(character_entity) {
            spine.move_vector = direction;
        }

        // ---- Aim, Fire, Drop ----
        for (pistol_entity, mut pistol, connectivity) in pistol_q.iter_mut() {
            // Skip pistols not connected to this character
            if connectivity.character != character_entity {
                continue;
            }

            // Aim: hold C to aim at mouse cursor
            if action_state.pressed(&PlayerAction::Aim) {
                pistol.world_aim_trarget = Some(mouse.pos);
            } else {
                pistol.world_aim_trarget = None;
            }

            // Fire: hold F (cooldown enforced in process_fire_event)
            if action_state.pressed(&PlayerAction::Fire) {
                let collision_group = character_root_q
                    .get(character_entity)
                    .map(|cr| cr.collision_group)
                    .unwrap_or(0);
                fire.write(FireEvent {
                    weapon: pistol_entity,
                    projectile_collision_group: collision_group,
                });
            }

            // Drop weapon: press V
            if action_state.just_pressed(&PlayerAction::DropWeapon) {
                events.write(CharacterPartEvent::UnregisterPart {
                    character: character_entity,
                    part: pistol_entity,
                });
                if let Ok(mut right) = arm_q.get_mut(character_entity) {
                    right.config.ik_mode = IkMode::Disabled;
                    reset_event.write(ResetArmControlConstraintsEvent {
                        arm: WhichArm::Right,
                        character: character_entity,
                    });
                }
            }
        }

        // ---- Legacy CharacterController ----
        if let Ok(mut legacy) = legacy_q.get_mut(character_entity) {
            legacy.move_vector = direction;
            legacy.point_vector = mouse.pos;
        }
    }
}
