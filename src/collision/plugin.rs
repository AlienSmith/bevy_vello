use bevy::app::App;
use bevy::app::Plugin;

use crate::collision::VelloCollisionScene;
use crate::collision::VelloCollisionWorld;

pub struct VelloCollisionPlugin;

impl Plugin for VelloCollisionPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(VelloCollisionWorld::default())
            .insert_resource(VelloCollisionScene::default());
        
    }
    fn finish(&self, app: &mut App) {}
}
