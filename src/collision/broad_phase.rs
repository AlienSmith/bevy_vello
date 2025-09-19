use bevy::prelude::*;
use parry2d::bounding_volume::Aabb;
use parry2d::math::Point;
use parry2d::partitioning::IndexedData;
use parry2d::partitioning::Qbvh;
use parry2d::partitioning::QbvhUpdateWorkspace;
use parry2d::query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor;

use crate::collision::VelloCollisionBroadPhase;
use crate::collision::VelloCollisionWorld;
use crate::VelloCollider;

pub fn compute_aabb_from_collider(collider: &VelloCollider) -> Aabb {
    let bbox = collider.get_aabb();
    let result = Aabb::new(Point::new(bbox.x, bbox.y), Point::new(bbox.z, bbox.w));
    result
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[repr(transparent)]
struct ColliderHandle(pub Entity);

impl Default for ColliderHandle {
    fn default() -> Self {
        Self(Entity::PLACEHOLDER)
    }
}

impl IndexedData for ColliderHandle {
    fn default() -> Self {
        Default::default()
    }

    fn index(&self) -> usize {
        self.0.index() as usize
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BroadPhaseQbvh {
    qbvh: Qbvh<ColliderHandle>,
    stack: Vec<(u32, u32)>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    workspace: QbvhUpdateWorkspace,
}

impl Default for BroadPhaseQbvh {
    fn default() -> Self {
        Self::new()
    }
}

impl BroadPhaseQbvh {
    pub fn new() -> Self {
        Self {
            qbvh: Qbvh::new(),
            stack: vec![],
            workspace: QbvhUpdateWorkspace::default(),
        }
    }
    pub fn update(
        &mut self,
        all_colliders: &Query<(Entity, &VelloCollider)>,
        modified_colliders: &Query<(Entity, &VelloCollider), Changed<VelloCollider>>,
        removed_collider: &mut RemovedComponents<VelloCollider>,
        collision_world: &mut ResMut<VelloCollisionWorld>,
    ) {
        let margin = 0.01;

        if modified_colliders.iter().count() == 0 {
            return;
        }

        let mut visitor = BoundingVolumeIntersectionsSimultaneousVisitor::new(
            |co1: &ColliderHandle, co2: &ColliderHandle| {
                if *co1 != *co2 {
                    collision_world.collision_pairs.push((co1.0, co2.0));
                }
                true
            },
        );

        let full_rebuild = self.qbvh.raw_nodes().is_empty();
        if full_rebuild {
            self.qbvh.clear_and_rebuild(
                all_colliders.iter().map(|(index, collider)| {
                    (ColliderHandle(index), compute_aabb_from_collider(collider))
                }),
                margin,
            );
            self.qbvh
                .traverse_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
        } else {
            for (entity, _collider) in modified_colliders.iter() {
                self.qbvh.pre_update_or_insert(ColliderHandle(entity));
            }

            for entity in removed_collider.read() {
                self.qbvh.remove(ColliderHandle(entity));
            }

            let _ = self.qbvh.refit(margin, &mut self.workspace, |handle| {
                let (_entity, collider) = all_colliders.get(handle.0).unwrap();
                compute_aabb_from_collider(collider)
            });
            // self.qbvh
            //     .traverse_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
            self.qbvh
                .traverse_modified_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
            self.qbvh.rebalance(margin, &mut self.workspace);
        }
    }
}

pub fn update_broad_phase(
    all_colliders: Query<(Entity, &VelloCollider)>,
    modified_colliders: Query<(Entity, &VelloCollider), Changed<VelloCollider>>,
    mut removed_collider: RemovedComponents<VelloCollider>,
    mut collision_world: ResMut<VelloCollisionWorld>,
    mut broad_phase: ResMut<VelloCollisionBroadPhase>,
) {
    broad_phase.broad_phase.update(
        &all_colliders,
        &modified_colliders,
        &mut removed_collider,
        &mut collision_world,
    );
}
