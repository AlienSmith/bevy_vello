use bevy::ecs::schedule::IntoSystemConfigs;
use bevy::prelude::*;
use bevy::render::ExtractSchedule;
use bevy::render::RenderApp;
use bevy::transform::TransformSystem;
use vello::CollisionResult;

use crate::collision::broad_phase::update_broad_phase;
use crate::collision::extract::extract_collision_scene;
use crate::collision::systems::make_collision_scene;
use crate::collision::CollisionResults;
use crate::collision::CollisionSystems;
use crate::collision::ExtractedVelloCollisionScene;
use crate::collision::GpuDataChannel;
use crate::collision::VelloCollisionBroadPhase;
use crate::collision::VelloCollisionScene;
use crate::collision::VelloCollisionWorld;
use crate::integrations::svg_collider::SvgColliderPlugin;
pub struct VelloCollisionPlugin;

impl Plugin for VelloCollisionPlugin {
    fn build(&self, app: &mut App) {
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        render_app
            .insert_resource(ExtractedVelloCollisionScene::default())
            .add_systems(ExtractSchedule, extract_collision_scene);
        app.add_plugins(SvgColliderPlugin)
            .insert_resource(VelloCollisionWorld::default())
            .insert_resource(VelloCollisionScene::default())
            .insert_resource(VelloCollisionBroadPhase::default())
            .insert_resource(GpuDataChannel::<CollisionResults>::new(1))
            .configure_sets(
                PostUpdate,
                (
                    CollisionSystems::CollisionResponse,
                    CollisionSystems::Collision,
                )
                    .chain()
                    .after(TransformSystem::TransformPropagate),
            )
            .add_systems(
                PostUpdate,
                (
                    update_broad_phase,
                    make_collision_scene,
                    //print_collision_results,
                )
                    .chain()
                    .in_set(CollisionSystems::Collision),
            );
    }
}
