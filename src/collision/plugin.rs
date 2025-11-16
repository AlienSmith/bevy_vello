use crate::collision::broad_phase::update_broad_phase;
use crate::collision::extract::extract_collision_scene;
use crate::collision::systems::collect_removed_colliders;
use crate::collision::systems::collision_event_dispatch;
use crate::collision::systems::make_collision_scene;
use crate::collision::CollisionResults;
use crate::collision::CollisionSystems;
use crate::collision::ExtractedVelloCollisionScene;
use crate::collision::GpuDataChannel;
use crate::collision::RemovedColliders;
use crate::collision::VelloCollisionBroadPhase;
use crate::collision::VelloCollisionEvent;
use crate::collision::VelloCollisionScene;
use crate::collision::VelloCollisionWorld;
use crate::integrations::svg_collider::SvgColliderPlugin;
use bevy::ecs::schedule::IntoSystemConfigs;
use bevy::prelude::*;
use bevy::render::ExtractSchedule;
use bevy::render::RenderApp;
use bevy::transform::TransformSystem;
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
            .add_event::<VelloCollisionEvent>()
            .insert_resource(VelloCollisionWorld::default())
            .insert_resource(RemovedColliders::default())
            .insert_resource(VelloCollisionScene::default())
            .insert_resource(VelloCollisionBroadPhase::default())
            //.insert_resource(SimpleBroadPhase::default())
            .insert_resource(GpuDataChannel::<CollisionResults>::new(1))
            .configure_sets(
                PostUpdate,
                (
                    CollisionSystems::CollectRemovedColliders,
                    CollisionSystems::SendCollisionEvent,
                    CollisionSystems::CollisionResponsePhysics,
                    CollisionSystems::MakeCollisionScene,
                )
                    .chain()
                    .after(TransformSystem::TransformPropagate),
            )
            .add_systems(
                PostUpdate,
                collect_removed_colliders.in_set(CollisionSystems::CollectRemovedColliders),
            )
            .add_systems(PostUpdate, collision_event_dispatch)
            .add_systems(
                PostUpdate,
                (
                    update_broad_phase,
                    //update_broad_phase_simple,
                    make_collision_scene,
                    //print_collision_results,
                )
                    .chain()
                    .in_set(CollisionSystems::MakeCollisionScene),
            );
    }
}
