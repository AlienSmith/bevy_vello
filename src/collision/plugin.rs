use bevy::app::App;
use bevy::app::Plugin;
use bevy::app::Update;
use bevy::ecs::schedule::IntoSystemConfigs;
use bevy::render::ExtractSchedule;
use bevy::render::RenderApp;
use vello::CollisionResult;

use crate::collision::extract::extract_collision_scene;
use crate::collision::systems::make_collision_scene;
use crate::collision::systems::print_collision_results;
use crate::collision::systems::update_collision_world;
use crate::collision::ExtractedVelloCollisionScene;
use crate::collision::GpuDataChannel;
use crate::collision::VelloCollisionScene;
use crate::collision::VelloCollisionWorld;
pub struct VelloCollisionPlugin;

impl Plugin for VelloCollisionPlugin {
    fn build(&self, app: &mut App) {
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        render_app
            .insert_resource(ExtractedVelloCollisionScene::default())
            .add_systems(ExtractSchedule, extract_collision_scene);
        app.insert_resource(VelloCollisionWorld::default())
            .insert_resource(VelloCollisionScene::default())
            .insert_resource(GpuDataChannel::<Vec<CollisionResult>>::new(1))
            .add_systems(
                Update,
                (
                    update_collision_world,
                    make_collision_scene.after(update_collision_world),
                    print_collision_results,
                ),
            );
    }
}
