use bevy::prelude::*;

use crate::collider_factory::observers::assemble_collider;
#[derive(Default)]
pub struct ColliderFactoryPlugin;

impl Plugin for ColliderFactoryPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(assemble_collider);
    }
}
