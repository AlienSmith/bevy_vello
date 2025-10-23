use core::f32;

use bevy::prelude::*;
use parry2d::bounding_volume::Aabb;
use parry2d::bounding_volume::SimdAabb;
use parry2d::math::Point;
use parry2d::math::SimdReal;
use parry2d::math::SIMD_WIDTH;
use parry2d::na::SimdValue;
use parry2d::partitioning::IndexedData;
use parry2d::partitioning::Qbvh;
use parry2d::partitioning::QbvhUpdateWorkspace;
use parry2d::partitioning::SimdBestFirstVisitStatus;
use parry2d::partitioning::SimdBestFirstVisitor;
use parry2d::query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor;

use crate::collision::RemovedColliders;
use crate::collision::SimpleBroadPhase;
use crate::collision::VelloCollisionBroadPhase;
use crate::collision::VelloCollisionWorld;
use crate::mat4_to_affine;
use crate::VelloCollider;
//aabb will only take the effect of position ignoring entity rotation and scale.
pub fn compute_aabb_from_collider(collider: &VelloCollider, transform: &GlobalTransform) -> Aabb {
    let position = transform.translation();
    let bbox = collider.get_aabb();
    let result = Aabb::new(
        Point::new(bbox.x + position.x, bbox.y + position.y),
        Point::new(bbox.z + position.x, bbox.w + position.y),
    );
    result
}

pub fn compute_aabb(collider: &VelloCollider, transform: &GlobalTransform) -> Vec4 {
    let pos = mat4_to_affine(transform.compute_matrix()).translation();
    return Vec4::new(
        (pos.x + collider.aabb.x0) as f32,
        (pos.y + collider.aabb.y0) as f32,
        (pos.x + collider.aabb.x1) as f32,
        (pos.y + collider.aabb.y1) as f32,
    );
}

pub fn check_overlaps(a: Vec4, b: Vec4) -> bool {
    let x_min = a.x.max(b.x);
    let y_min = a.y.max(b.y);
    let x_max = a.z.min(b.z);
    let y_max = a.w.min(b.w);
    return x_min <= x_max && y_min <= y_max;
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

    pub fn find_first_constains_point(&self, x: f32, y: f32) -> Option<Entity> {
        let mut visitor = FindFirstContainsPointVisitor {
            point: Point::new(SimdReal::splat(x), SimdReal::splat(y)),
        };
        if let Some((_, result)) = self.qbvh.traverse_best_first(&mut visitor) {
            Some(result.0)
        } else {
            None
        }
    }

    pub fn update(
        &mut self,
        all_colliders: &Query<(Entity, &VelloCollider, &GlobalTransform)>,
        modified_colliders: &Query<(Entity, &VelloCollider), Changed<VelloCollider>>,
        removed_collider: &Res<RemovedColliders>,
        collision_world: &mut ResMut<VelloCollisionWorld>,
    ) {
        collision_world.collision_pairs_bvh.clear();
        let margin = 0.01;

        if modified_colliders.iter().count() == 0 {
            return;
        }

        let mut visitor = BoundingVolumeIntersectionsSimultaneousVisitor::new(
            |co1: &ColliderHandle, co2: &ColliderHandle| {
                if *co1 != *co2 {
                    collision_world.collision_pairs_bvh.push((co1.0, co2.0));
                }
                true
            },
        );

        let full_rebuild = self.qbvh.raw_nodes().is_empty();
        if full_rebuild {
            self.qbvh.clear_and_rebuild(
                all_colliders.iter().map(|(index, collider, &transform)| {
                    (
                        ColliderHandle(index),
                        compute_aabb_from_collider(collider, &transform),
                    )
                }),
                margin,
            );
            self.qbvh
                .traverse_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
        } else {
            for (entity, _collider) in modified_colliders.iter() {
                self.qbvh.pre_update_or_insert(ColliderHandle(entity));
            }

            for entity in removed_collider.colliders.iter() {
                self.qbvh.remove(ColliderHandle(*entity));
            }

            let _ = self.qbvh.refit(margin, &mut self.workspace, |handle| {
                //TODO: Fix the removecompoents missing some entity problem
                if let Ok((_entity, collider, transform)) = all_colliders.get(handle.0) {
                    return compute_aabb_from_collider(collider, transform);
                } else {
                    Aabb::new(
                        Point::new(f32::INFINITY, f32::INFINITY),
                        Point::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
                    )
                }
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
    all_colliders: Query<(Entity, &VelloCollider, &GlobalTransform)>,
    modified_colliders: Query<(Entity, &VelloCollider), Changed<VelloCollider>>,
    removed_collider: Res<RemovedColliders>,
    mut collision_world: ResMut<VelloCollisionWorld>,
    mut broad_phase: ResMut<VelloCollisionBroadPhase>,
) {
    broad_phase.broad_phase.update(
        &all_colliders,
        &modified_colliders,
        &removed_collider,
        &mut collision_world,
    );
}

pub fn update_broad_phase_simple(
    all_colliders: Query<(Entity, &VelloCollider, &GlobalTransform)>,
    mut collision_world: ResMut<VelloCollisionWorld>,
    mut broad_phase: ResMut<SimpleBroadPhase>,
) {
    broad_phase
        .broad_phase
        .update(&all_colliders, &mut collision_world);
}

#[derive(Clone, Default)]
pub struct BroadPhaseSimple;
impl BroadPhaseSimple {
    pub fn update(
        &mut self,
        all_colliders: &Query<(Entity, &VelloCollider, &GlobalTransform)>,
        collision_world: &mut ResMut<VelloCollisionWorld>,
    ) {
        collision_world.collision_pairs_bvh.clear();
        let mut static_colliders: Vec<Entity> = vec![];
        let mut dynamic_colliders: Vec<Entity> = vec![];
        for (item, collider, _) in all_colliders.iter() {
            if collider.is_soft_body() {
                dynamic_colliders.push(item);
            } else {
                static_colliders.push(item);
            }
        }
        for i in 0..dynamic_colliders.len() {
            let (item, collider, transform) = all_colliders.get(dynamic_colliders[i]).unwrap();
            let aabb = compute_aabb(collider, transform);
            for j in (i + 1)..dynamic_colliders.len() {
                let (item1, collider1, transform1) =
                    all_colliders.get(dynamic_colliders[j]).unwrap();
                let aabb1 = compute_aabb(collider1, transform1);
                if check_overlaps(aabb, aabb1) {
                    collision_world.collision_pairs_bvh.push((item, item1));
                }
            }
        }
        for i in 0..dynamic_colliders.len() {
            let (item, collider, transform) = all_colliders.get(dynamic_colliders[i]).unwrap();
            let aabb = compute_aabb(collider, transform);
            for j in 0..static_colliders.len() {
                let (item1, collider1, transform1) =
                    all_colliders.get(static_colliders[j]).unwrap();
                let aabb1 = compute_aabb(collider1, transform1);
                if check_overlaps(aabb, aabb1) {
                    collision_world.collision_pairs_bvh.push((item, item1));
                }
            }
        }
    }
}

pub struct FindFirstContainsPointVisitor {
    point: Point<SimdReal>,
}

impl SimdBestFirstVisitor<ColliderHandle, SimdAabb> for FindFirstContainsPointVisitor {
    type Result = ColliderHandle;

    fn visit(
        &mut self,
        _best_cost_so_far: parry2d::math::Real,
        bv: &SimdAabb,
        value: Option<[Option<&ColliderHandle>; parry2d::math::SIMD_WIDTH]>,
    ) -> parry2d::partitioning::SimdBestFirstVisitStatus<Self::Result> {
        let contains_mask = bv.contains_local_point(&self.point); // Immutable borrow inside method
        if let Some(leaves) = value {
            for (i, &data) in leaves.iter().enumerate() {
                if contains_mask.extract(i) {
                    if let Some(leaf_idx) = data {
                        return SimdBestFirstVisitStatus::ExitEarly(Some(*leaf_idx));
                    }
                }
            }
        }
        SimdBestFirstVisitStatus::MaybeContinue {
            weights: SimdReal::splat(0.0),
            mask: contains_mask,
            results: [None; SIMD_WIDTH],
        }
    }
}
