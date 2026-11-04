use bevy::prelude::*;
use bevy_vello::collision::CollisionSystems;
use bevy_vello::integrations::physics::systems::update_constraint_world;

use crate::{
    character::{
        observers::{on_remove_connectivity, on_remove_connectivity_root},
        systems::{reset_limb_constraint_event, tick_spine_drive},
        StringPool,
    },
    ResetLimbControlConstraintsEvent,
};
pub struct GameCharacterPlugin;

impl Plugin for GameCharacterPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        // Tick-native spine drive by default; VELLO_LEGACY_CONTROLLER=1
        // restores the old per-frame event bridge (kill-switch).
        let tick_native = std::env::var("VELLO_LEGACY_CONTROLLER").as_deref() != Ok("1");
        app.insert_resource(StringPool::default())
            .add_event::<ResetLimbControlConstraintsEvent>()
            .add_observer(on_remove_connectivity)
            .add_observer(on_remove_connectivity_root)
            .add_systems(
                FixedUpdate,
                (
                    // TODO(walker-control): `update_character_movement` (the old
                    // limb PD driver) commands the same leg joints as the
                    // walker_control actuator and overwrites its stance targets —
                    // commented out for the walker_qa QA phase; it will move to an
                    // opt-in plugin or be removed. `tick_spine_drive` keeps its
                    // ordering against the constraint world directly.
                    // update_character_movement.before(update_constraint_world),
                    tick_spine_drive.before(update_constraint_world),
                ),
            );
        //we would intensionally make the reset event being resolved frame later so it won't got mixed with control event.
        app.add_systems(
            PostUpdate,
            reset_limb_constraint_event.after(CollisionSystems::CollisionResponsePhysics),
        );
    }
}
