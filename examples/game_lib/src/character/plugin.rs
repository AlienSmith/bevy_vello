use bevy::prelude::*;
use bevy_vello::collision::CollisionSystems;

use crate::{
    character::{
        observers::{on_remove_connectivity, on_remove_connectivity_root},
        systems::{reset_arm_constraint_event, update_character_movement},
        StringPool,
    },
    ResetArmControlConstraintsEvent,
};
pub struct GameCharacterPlugin;

impl Plugin for GameCharacterPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(StringPool::default())
            .add_event::<ResetArmControlConstraintsEvent>()
            .add_observer(on_remove_connectivity)
            .add_observer(on_remove_connectivity_root)
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
