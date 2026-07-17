use crate::collision::broad_phase::update_broad_phase;
use crate::collision::systems::collect_removed_colliders;
use crate::collision::systems::collision_event_redistribute;
use crate::collision::systems::make_collision_scene;
use crate::collision::CollisionCoolDownPairManager;
use crate::collision::CollisionResults;
use crate::collision::CollisionSystems;
use crate::collision::RemovedColliders;
use crate::collision::VelloCollisionBroadPhase;
use crate::collision::VelloCollisionEvent;
use crate::collision::VelloCollisionScene;
use crate::collision::VelloCollisionTrigger;
use crate::collision::VelloCollisionWorld;
use crate::integrations::svg_collider::SvgColliderPlugin;
use bevy::prelude::*;
use bevy::render::ExtractSchedule;
use bevy::render::RenderApp;
use bevy::transform::TransformSystem;
pub struct VelloCollisionPlugin;

impl Plugin for VelloCollisionPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(SvgColliderPlugin)
            .add_event::<VelloCollisionEvent>()
            .add_event::<VelloCollisionTrigger>()
            .insert_resource(VelloCollisionWorld::default())
            .insert_resource(RemovedColliders::default())
            .insert_resource(VelloCollisionScene::default())
            .insert_resource(VelloCollisionBroadPhase::default())
            .insert_resource(CollisionCoolDownPairManager::default())
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
            .add_systems(
                PostUpdate,
                collision_event_redistribute.in_set(CollisionSystems::SendCollisionEvent),
            )
            .add_systems(
                PostUpdate,
                (update_broad_phase, make_collision_scene)
                    .chain()
                    .in_set(CollisionSystems::MakeCollisionScene),
            );
    }
}
