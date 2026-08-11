use core::f32;

use avian2d::prelude::collider;
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
use vello_physics::soft_body_connection;

use crate::collision::CollisionSceneState;
use crate::collision::RemovedColliders;
use crate::collision::SimpleBroadPhase;
use crate::collision::VelloCollisionBroadPhase;
use crate::collision::VelloCollisionScene;
use crate::collision::VelloCollisionWorld;
use crate::mat4_to_affine;
use crate::VelloCollider;
//aabb will only take the effect of position ignoring entity rotation and scale.
pub fn compute_aabb_from_collider(collider: &VelloCollider, transform: &Transform) -> Aabb {
    let position = transform.translation;
    let bbox = collider.get_aabb();
    let result = Aabb::new(
        Point::new(bbox.x + position.x, bbox.y + position.y),
        Point::new(bbox.z + position.x, bbox.w + position.y),
    );
    result
}
#[allow(dead_code)]
pub fn compute_aabb(collider: &VelloCollider, transform: &Transform) -> Vec4 {
    let pos = mat4_to_affine(transform.compute_matrix()).translation();
    return Vec4::new(
        (pos.x + collider.aabb.x0) as f32,
        (pos.y + collider.aabb.y0) as f32,
        (pos.x + collider.aabb.x1) as f32,
        (pos.y + collider.aabb.y1) as f32,
    );
}

#[allow(dead_code)]
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

    /// Ray-cast against all colliders in the BVH, returning all (Entity, t_entry)
    /// pairs whose world-space AABB is intersected by the ray.
    ///
    /// Results are NOT sorted; the caller should sort by t_entry ascending.
    /// The ray parameters are in bevy world space (y-up).
    pub fn ray_cast(&self, origin: Vec2, direction: Vec2, max_distance: f32) -> Vec<(Entity, f32)> {
        if self.qbvh.raw_nodes().is_empty() {
            return Vec::new();
        }
        let mut visitor = RayCastVisitor::new(origin, direction, max_distance);
        self.qbvh.traverse_best_first(&mut visitor);
        visitor.hits
    }

    pub fn update(
        &mut self,
        all_colliders: &Query<(Entity, &VelloCollider)>,
        modified_colliders: &Query<
            (Entity, &VelloCollider),
            Or<(Changed<VelloCollider>, Changed<GlobalTransform>)>,
        >,
        removed_collider: &Res<RemovedColliders>,
        collision_world: &mut ResMut<VelloCollisionWorld>,
    ) {
        let margin = 0.01;
        let is_empty = self.qbvh.raw_nodes().is_empty();

        // If BVH is empty, always do a full rebuild
        if is_empty {
            self.qbvh.clear_and_rebuild(
                all_colliders.iter().map(|(index, collider)| {
                    (
                        ColliderHandle(index),
                        compute_aabb_from_collider(collider, &collider.soft_body_global_transform),
                    )
                }),
                margin,
            );
            collision_world.collision_pairs_bvh.clear();
            let mut visitor = BoundingVolumeIntersectionsSimultaneousVisitor::new(
                |co1: &ColliderHandle, co2: &ColliderHandle| {
                    if *co1 != *co2 {
                        collision_world.collision_pairs_bvh.push((co1.0, co2.0));
                    }
                    true
                },
            );
            self.qbvh
                .traverse_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
            return;
        }

        // If nothing changed, keep existing pairs from the last frame
        if modified_colliders.iter().count() == 0 {
            return;
        }

        collision_world.collision_pairs_bvh.clear();

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
                all_colliders.iter().map(|(index, collider)| {
                    (
                        ColliderHandle(index),
                        compute_aabb_from_collider(collider, &collider.soft_body_global_transform),
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
                //TODO: The assumption is missing removed component would be collect later, other wise this could be a potential memory leak
                if let Ok((_entity, collider)) = all_colliders.get(handle.0) {
                    return compute_aabb_from_collider(
                        collider,
                        &collider.soft_body_global_transform,
                    );
                } else {
                    Aabb::new_invalid()
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
    all_colliders: Query<(Entity, &VelloCollider)>,
    modified_colliders: Query<
        (Entity, &VelloCollider),
        Or<(Changed<VelloCollider>, Changed<GlobalTransform>)>,
    >,
    removed_collider: Res<RemovedColliders>,
    mut collision_world: ResMut<VelloCollisionWorld>,
    mut broad_phase: ResMut<VelloCollisionBroadPhase>,
    scene: Res<VelloCollisionScene>,
) {
    if scene.state == CollisionSceneState::Created {
        broad_phase.broad_phase.update(
            &all_colliders,
            &modified_colliders,
            &removed_collider,
            &mut collision_world,
        );
    }
}

#[allow(dead_code)]
pub fn update_broad_phase_simple(
    all_colliders: Query<(Entity, &VelloCollider)>,
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
        all_colliders: &Query<(Entity, &VelloCollider)>,
        collision_world: &mut ResMut<VelloCollisionWorld>,
    ) {
        collision_world.collision_pairs_bvh.clear();
        let mut static_colliders: Vec<Entity> = vec![];
        let mut dynamic_colliders: Vec<Entity> = vec![];
        for (item, collider) in all_colliders.iter() {
            if collider.is_soft_body() {
                dynamic_colliders.push(item);
            } else {
                static_colliders.push(item);
            }
        }
        for i in 0..dynamic_colliders.len() {
            let (item, collider) = all_colliders.get(dynamic_colliders[i]).unwrap();
            let aabb = compute_aabb(collider, &collider.soft_body_global_transform);
            for j in (i + 1)..dynamic_colliders.len() {
                let (item1, collider1) = all_colliders.get(dynamic_colliders[j]).unwrap();
                let aabb1 = compute_aabb(collider1, &collider1.soft_body_global_transform);
                if check_overlaps(aabb, aabb1) {
                    collision_world.collision_pairs_bvh.push((item, item1));
                }
            }
        }
        for i in 0..dynamic_colliders.len() {
            let (item, collider) = all_colliders.get(dynamic_colliders[i]).unwrap();
            let aabb = compute_aabb(collider, &collider.soft_body_global_transform);
            for j in 0..static_colliders.len() {
                let (item1, collider1) = all_colliders.get(static_colliders[j]).unwrap();
                let aabb1 = compute_aabb(collider1, &collider1.soft_body_global_transform);
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

/// Visitor that collects ALL collider handles whose AABBs intersect the ray,
/// along with the entry distance (t_entry) along the ray.
/// Uses the slab method (Kay & Kajiya) per SIMD lane for efficient traversal.
pub struct RayCastVisitor {
    /// Collected hits: (Entity, t_entry). Entry distance is clamped to >= 0.
    pub hits: Vec<(Entity, f32)>,
    origin: Vec2,
    inv_dir: Vec2,
    max_dist: f32,
}

impl RayCastVisitor {
    pub fn new(origin: Vec2, direction: Vec2, max_distance: f32) -> Self {
        let epsilon = 1e-10f32;
        let inv_x = if direction.x.abs() < epsilon {
            f32::MAX
        } else {
            1.0 / direction.x
        };
        let inv_y = if direction.y.abs() < epsilon {
            f32::MAX
        } else {
            1.0 / direction.y
        };
        Self {
            hits: Vec::new(),
            origin,
            inv_dir: Vec2::new(inv_x, inv_y),
            max_dist: max_distance,
        }
    }

    /// Slab test for a single aabb pair (min, max) returning `Some(t_entry)` or `None`.
    fn ray_aabb_slab(&self, min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Option<f32> {
        let t1_x = (min_x - self.origin.x) * self.inv_dir.x;
        let t2_x = (max_x - self.origin.x) * self.inv_dir.x;
        let t1_y = (min_y - self.origin.y) * self.inv_dir.y;
        let t2_y = (max_y - self.origin.y) * self.inv_dir.y;

        let t_entry_x = t1_x.min(t2_x);
        let t_exit_x = t1_x.max(t2_x);
        let t_entry_y = t1_y.min(t2_y);
        let t_exit_y = t1_y.max(t2_y);

        let t_entry = t_entry_x.max(t_entry_y);
        let t_exit = t_exit_x.min(t_exit_y);

        if t_entry <= t_exit && t_exit >= 0.0 && t_entry <= self.max_dist {
            Some(t_entry.max(0.0))
        } else {
            None
        }
    }
}

impl SimdBestFirstVisitor<ColliderHandle, SimdAabb> for RayCastVisitor {
    type Result = ();

    fn visit(
        &mut self,
        _best_cost_so_far: parry2d::math::Real,
        bv: &SimdAabb,
        value: Option<[Option<&ColliderHandle>; SIMD_WIDTH]>,
    ) -> SimdBestFirstVisitStatus<Self::Result> {
        let mut hit_mask = [false; SIMD_WIDTH];
        let mut weights_arr = [0.0f32; SIMD_WIDTH];

        // Test each SIMD lane individually using scalar slab method
        for i in 0..SIMD_WIDTH {
            let min_x = bv.mins.coords.x.extract(i);
            let min_y = bv.mins.coords.y.extract(i);
            let max_x = bv.maxs.coords.x.extract(i);
            let max_y = bv.maxs.coords.y.extract(i);

            if let Some(t_entry) = self.ray_aabb_slab(min_x, min_y, max_x, max_y) {
                hit_mask[i] = true;
                weights_arr[i] = t_entry;

                // Collect leaf entity handles that intersect the ray
                if let Some(leaves) = &value {
                    if let Some(handle) = leaves[i] {
                        self.hits.push((handle.0, t_entry));
                    }
                }
            }
        }

        SimdBestFirstVisitStatus::MaybeContinue {
            weights: SimdReal::from(weights_arr),
            mask: parry2d::math::SimdBool::from(hit_mask),
            results: [None; SIMD_WIDTH],
        }
    }
}
