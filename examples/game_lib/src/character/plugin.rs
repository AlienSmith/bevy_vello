use bevy::prelude::*;
use bevy_vello::collision::CollisionSystems;
use bevy_vello::integrations::physics::systems::update_constraint_world;

use crate::{
    character::{
        observers::{on_remove_connectivity, on_remove_connectivity_root},
        systems::{
            reset_arm_constraint_event, tick_spine_drive, SpineControllerMode,
            update_character_movement,
        },
        StringPool,
    },
    ResetArmControlConstraintsEvent,
};
pub struct GameCharacterPlugin;

impl Plugin for GameCharacterPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        // Tick-native spine drive by default; VELLO_LEGACY_CONTROLLER=1
        // restores the old per-frame event bridge (kill-switch).
        let tick_native = std::env::var("VELLO_LEGACY_CONTROLLER").as_deref() != Ok("1");
        app.insert_resource(StringPool::default())
            .insert_resource(SpineControllerMode { tick_native })
            .add_event::<ResetArmControlConstraintsEvent>()
            .add_observer(on_remove_connectivity)
            .add_observer(on_remove_connectivity_root)
            .add_systems(
                FixedUpdate,
                tick_spine_drive.before(update_constraint_world),
            )
            .add_systems(
                PostUpdate,
                update_character_movement.before(CollisionSystems::CollisionResponsePhysics),
            );
        //we would intensionally make the reset event being resolved frame later so it won't got mixed with control event.
        app.add_systems(
            PostUpdate,
            reset_arm_constraint_event.after(CollisionSystems::CollisionResponsePhysics),
        );
    }
}
