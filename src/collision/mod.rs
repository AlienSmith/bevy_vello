use std::sync::{Arc, Mutex};

use avian2d::parry::utils::hashmap::HashMap;
use bevy::prelude::*;
use bevy::{
    ecs::{component::Component, entity::Entity, schedule::SystemSet},
    math::{Vec2, Vec4},
};
pub use plugin::VelloCollisionPlugin;
use vello::kurbo::Affine;
use vello::{
    kurbo::{self, BezPath},
    peniko, CollisionResult, CollisionScene, RendererOptions,
};

mod broad_phase;
mod extract;
mod plugin;
pub mod raytrace;
pub mod systems;

pub const VELLO_COLLISION_WORLD_RATIO: f32 = 4.0;
pub const VELLO_COLLISION_COOL_DOWN_TIME: f32 = 0.5;

use broad_phase::BroadPhaseQbvh;

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionScene {
    scene: CollisionScene,
    pair: Vec<(Entity, Entity)>,
    pub state: CollisionSceneState,
}

#[derive(Clone, Copy, Default, PartialEq)]
// our gpu collision logic which running in the render world of bevy runs at a different frequency with our phycis system.
// we use this state to avoid duplucated collision test(which would cause the programe to stuck since we used bounded channel to send collision result back)
pub enum CollisionSceneState {
    Created,
    NeedExtract,
    #[default]
    Extracted, // if it is extracted don't extract it again.
}

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionWorld {
    pub(crate) collision_pairs_bvh: Vec<(Entity, Entity)>,
    pub(crate) collision_pairs: Vec<(Entity, Entity)>,
    pub paused: bool,
    pub substeps: u32,
}

#[derive(Default, Resource, Clone)]
pub struct RemovedColliders {
    pub(crate) colliders: Vec<Entity>,
}

impl VelloCollisionWorld {
    pub fn update_collision_pairs_if_previous_one_has_been_consumed(
        &mut self,
        q: &Query<(&VelloCollider, &GlobalTransform)>,
    ) -> bool {
        self.collision_pairs.clear();
        let mut temp: Vec<(Entity, Entity)> = vec![];
        //TODO: Fix the removecompoents missing some entity problem
        //we could have a more complicated filter here
        for (e0, e1) in &self.collision_pairs_bvh {
            if let Ok((c, _t)) = q.get(*e0) {
                if let Ok((c1, _t1)) = q.get(*e1) {
                    if (c.is_soft_body() || c1.is_soft_body())
                        && (c.collision_group == 0 || (c.collision_group != c1.collision_group))
                    {
                        temp.push((*e0, *e1));
                    }
                }
            }
        }
        self.collision_pairs = temp;
        return true;
    }
}

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionBroadPhase {
    pub(crate) broad_phase: BroadPhaseQbvh,
}

impl VelloCollisionBroadPhase {
    pub fn find_first_constains_point(&self, world_position: Vec2) -> Option<Entity> {
        self.broad_phase
            .find_first_constains_point(world_position.x, world_position.y)
    }

    /// Ray-cast against the BVH, returning all (Entity, t_entry) pairs for AABBs
    /// intersected by the ray. Unsorted; caller should sort by t_entry ascending.
    pub fn ray_cast(&self, origin: Vec2, direction: Vec2, max_distance: f32) -> Vec<(Entity, f32)> {
        self.broad_phase.ray_cast(origin, direction, max_distance)
    }
}

#[derive(Default, Resource, Clone)]
pub struct SimpleBroadPhase {
    pub(crate) broad_phase: BroadPhaseSimple,
}

/// Describes the *intent* of a collision response modification.
/// Game systems write per-pair overrides into `CollisionEventBatch`
/// (in PostUpdate observers). `make_collision_constraints` (in the next
/// FixedUpdate) resolves this intent into actual `other_inv_mass` and
/// `other_velocity` values using the physics state captured at detection time.
///
/// This separation exists because game systems do NOT have access to
/// the constraint world's internal physics state (inv_mass, frame_velocity
/// of the soft body). The physics snapshot is captured once by
/// `run_gpu_collision` and stored in `VelloCollisionEvent`.
#[derive(Clone, Debug, Default)]
pub struct CollisionOverride {
    /// Desired "explosion" impulse applied to the soft body side of the collision.
    /// This is a non-physical energy injection -- like a tiny explosion at the
    /// contact point. Specified as a world-space impulse vector (force * time).
    ///
    /// None = no explosion effect (use default collision params).
    pub explosion_impulse: Option<Vec2>,

    /// Scale factor for the opponent's velocity contribution.
    /// 1.0 = use actual opponent velocity (default behavior).
    /// 0.0 = treat opponent as static (no velocity transfer).
    /// >1.0 = amplify opponent velocity (makes hit feel heavier).
    pub velocity_scale: Option<f32>,

    /// Scale factor for the opponent's inverse mass.
    /// 1.0 = use actual opponent inv_mass (default behavior).
    /// 0.0 = treat opponent as infinitely heavy (like bullet hack).
    /// >1.0 = treat opponent as lighter (less reaction).
    pub inv_mass_scale: Option<f32>,
}

impl CollisionOverride {
    /// Returns true if this override has any non-default values set.
    pub fn is_active(&self) -> bool {
        self.explosion_impulse.is_some()
            || self.velocity_scale.is_some()
            || self.inv_mass_scale.is_some()
    }
}

// ── Collision Event Batch ──────────────────────────────────────────────────
/// A single entry in the collision event batch, pairing a raw collision event
/// with per-pair game-level overrides. Game systems modify `override_a` and
/// `override_b` via the batch resource (indexed by `batch_index` on the trigger).
#[derive(Clone, Debug)]
pub struct CollisionEventEntry {
    pub event: VelloCollisionEvent,
    /// Override for how entity_a wants entity_b to respond.
    pub override_a: CollisionOverride,
    /// Override for how entity_b wants entity_a to respond.
    pub override_b: CollisionOverride,
}

/// Resource holding all collision events and their per-pair overrides for the
/// current frame. Populated by `run_gpu_collision` at the end of FixedUpdate,
/// read by game observers in PostUpdate, and consumed by
/// `make_collision_constraints` at the start of the next FixedUpdate.
#[derive(Resource, Default, Clone)]
pub struct CollisionEventBatch {
    pub entries: Vec<CollisionEventEntry>,
}

//Use the debug_color and soft_body_global_transform in here to initialize this entity, instead of using the transform and scene
//in the VelloBundle.

//at this point we only use the translation from bevy global transform. and since we are not allowed to modify the global transform
//we need to make sure the item is initialized at the right location. otherwise the first frame of rendering is wrong.
#[derive(Clone, Default, Component)]
pub struct VelloCollider {
    pub(crate) shape: BezPath,
    pub frame_particles: [Particle; FRAME_PARTICLES_COUNT], //particles are always in world space.
    pub(crate) initilized_by_physics: bool, //some data are calculated in engine for softbody, this marks the entity is ready.
    pub(crate) aabb: kurbo::Rect, //use for coarse collision detection and actually in local space.
    pub(crate) initial_velocity: Vec2,
    pub(crate) collision_inverse_mass: f32,
    pub(crate) debug_color: peniko::Brush,
    pub(crate) is_soft_body: bool,
    pub(crate) uvs: Option<Vec<f32>>,
    pub(crate) soft_body_config: Option<SoftBodyInitConfig>,
    pub(crate) collision_config: Option<CollisionConstraintConfig>,
    pub(crate) soft_body_global_transform: Transform,
    pub(crate) collision_group: u32, //item in the same collision group won't collide against each other
    pub collision_cooled_down: f32,
    pub is_selected: bool,
    pub initial_scale: Vec2,
}

impl VelloCollider {
    pub fn is_soft_body(&self) -> bool {
        self.is_soft_body
    }

    pub fn new(
        path: &BezPath,
        frame_path: &BezPath,
        aabb: &kurbo::Rect,
        initial_velocity: Vec2,
        color: peniko::Brush,
        collision_inverse_mass: f32,
        is_soft_body: bool,
        uvs: Option<Vec<f32>>,
        soft_body_init_config: Option<SoftBodyInitConfig>,
        collision_constraint_config: Option<CollisionConstraintConfig>,
        collision_group: u32,
        transform: Transform,
        collision_cooled_down: f32,
    ) -> Self {
        let scale_x = (aabb.x1 - aabb.x0) as f32 * transform.scale.x;
        let scale_y = (aabb.y1 - aabb.y0) as f32 * transform.scale.y;
        Self {
            shape: path.clone(),
            frame_particles: Default::default(),
            aabb: *aabb,
            initial_velocity,
            collision_inverse_mass,
            debug_color: color,
            is_soft_body,
            uvs,
            is_selected: false,
            soft_body_config: soft_body_init_config,
            collision_config: collision_constraint_config,
            collision_group,
            soft_body_global_transform: transform,
            collision_cooled_down,
            initial_scale: Vec2::new(scale_x, scale_y),
            initilized_by_physics: !is_soft_body,
        }
    }

    pub fn get_aabb(&self) -> Vec4 {
        Vec4::new(
            self.aabb.x0 as f32,
            self.aabb.y0 as f32,
            self.aabb.x1 as f32,
            self.aabb.y1 as f32,
        )
    }
}

pub use vello_physics::CollisionConstraintConfig;
pub use vello_physics::SoftBodyInitConfig;
use vello_physics::{Particle, FRAME_PARTICLES_COUNT};

use crate::collision::broad_phase::BroadPhaseSimple;

/// Runs GPU collision detection synchronously from the main world.
/// Shares the `vello::Renderer` with the rendering pipeline via `Arc<Mutex<>>`
/// to avoid duplicating GPU buffer allocations.
#[derive(Resource, Clone)]
pub struct GpuCollisionRunner {
    renderer: Arc<Mutex<vello::Renderer>>,
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
}

impl GpuCollisionRunner {
    pub fn new(
        renderer: Arc<Mutex<vello::Renderer>>,
        device: wgpu::Device,
        queue: wgpu::Queue,
    ) -> Self {
        Self {
            renderer,
            device: Arc::new(device),
            queue: Arc::new(queue),
        }
    }

    /// Run GPU collision detection synchronously.
    /// Blocks until the GPU results are available.
    pub fn run_collision(&self, scene: &CollisionScene) -> Vec<CollisionResult> {
        let mut renderer = self.renderer.lock().unwrap();
        vello::util::block_on_wgpu(
            &self.device,
            renderer.render_collision_async(&self.device, &self.queue, scene),
        )
        .unwrap()
        .unwrap_or_default()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, SystemSet)]
pub enum CollisionSystems {
    CollectRemovedColliders,  //collect removed colliders
    SendCollisionEvent,       //send collision event
    CollisionResponsePhysics, // response to collsion event physics logic
    MakeCollisionScene, // this would collect collision paires from broad phase and prepare it for collision on gpu.
}

#[derive(Clone, Default)]
pub struct CollisionResults {
    pub pairs: Vec<(Entity, Entity)>,
    pub results: Vec<CollisionResult>,
}

pub use vello_physics::utility::generate_uvs;
pub use vello_physics::utility::path_to_ccw_quad_path;
//in bevy space use a y up x right coordinate.
#[derive(Event, Debug, Clone)]
pub struct VelloCollisionEvent {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub collision_point_a: Vec2,
    pub collision_point_b: Vec2,
    pub collision_normal_a: Vec2,
    pub collision_normal_b: Vec2,
    pub curve_index_a: u32,
    pub curve_index_b: u32,
    /// Physics snapshot: velocity of entity_a at collision detection time.
    pub velocity_a: Vec2,
    /// Physics snapshot: velocity of entity_b at collision detection time.
    pub velocity_b: Vec2,
    /// Physics snapshot: inverse mass of entity_a.
    pub inv_mass_a: f32,
    /// Physics snapshot: inverse mass of entity_b.
    pub inv_mass_b: f32,
}

//in bevy space use a y up x right coordinate.
#[derive(Event, Debug, Clone)]
pub struct VelloCollisionTrigger {
    pub entity_self: Entity,
    pub entity_other: Entity,
    pub collision_point: Vec2,
    pub normal_self: Vec2,
    pub normal_other: Vec2,
    /// Index into `CollisionEventBatch.entries` for this collision pair.
    pub batch_index: usize,
    /// Physics snapshot: velocity of self at detection time.
    pub self_velocity: Vec2,
    /// Physics snapshot: velocity of other at detection time.
    pub other_velocity: Vec2,
    /// Physics snapshot: inverse mass of self.
    pub self_inv_mass: f32,
    /// Physics snapshot: inverse mass of other.
    pub other_inv_mass: f32,
}

#[derive(Resource)]
pub struct CollisionCoolDownPairManager {
    pairs: HashMap<u64, (f32, f32)>,
    last_purge_time: f32,
    purge_time_gaps: f32,
}

impl Default for CollisionCoolDownPairManager {
    fn default() -> Self {
        Self {
            pairs: Default::default(),
            last_purge_time: 0.0,
            purge_time_gaps: 1.0,
        }
    }
}

impl CollisionCoolDownPairManager {
    pub fn pack_entity_pair(entity_a: Entity, entity_b: Entity) -> u64 {
        let id_a = entity_a.index();
        let id_b = entity_b.index();
        let min = std::cmp::min(id_a, id_b) as u64;
        let max = std::cmp::max(id_a, id_b) as u64;
        (min << 32) | max
    }
}

// ── Ray Trace Pipeline ────────────────────────────────────────────────────

/// A GPU ray trace command emitted by game systems.
///
/// Collected in `FixedUpdate`, run through BVH broad phase + GPU raytrace,
/// and results are redistributed as [`VelloRayTraceTrigger`] in `PostUpdate`.
#[derive(Event, Clone)]
pub struct VelloRayTraceCommand {
    /// The entity that emitted this ray (e.g., a gun).
    pub source_entity: Entity,
    /// Ray origin in bevy world space (y-up, x-right).
    pub origin: Vec2,
    /// Normalized ray direction in bevy world space (y-up, x-right).
    pub direction: Vec2,
    /// Maximum ray length; intersections beyond this are culled at broad phase.
    pub max_distance: f32,
}

/// Triggered on the source entity after GPU raytrace completes.
///
/// Game systems observe this to react to ray hits (e.g., apply damage).
/// `hit_entity` is `None` when the ray missed all geometry.
#[derive(Event, Clone)]
pub struct VelloRayTraceTrigger {
    /// The entity that emitted the ray.
    pub source_entity: Entity,
    /// The entity that was hit, or `None` if the ray missed.
    pub hit_entity: Option<Entity>,
    /// Hit point in bevy world space (y-up, x-right).
    pub hit_point: Vec2,
    /// Hit normal in bevy world space (y-up, x-right).
    pub hit_normal: Vec2,
    /// Distance along the ray to the hit point. Negative if miss.
    pub distance: f32,
    /// Index of the cubic segment that was hit.
    pub cubic_index: u32,
}

/// A single entry in the ray trace batch, pairing a command with its GPU result.
#[derive(Clone)]
pub struct RayTraceBatchEntry {
    /// The original ray trace command.
    pub command: VelloRayTraceCommand,
    /// The GPU ray trace result (one per (ray, candidate_shape) pair submitted).
    pub result: vello::RayTraceResult,
    /// The entity that was hit, resolved from the BVH candidate list.
    pub hit_entity: Option<Entity>,
}

/// Resource holding all ray trace results for the current frame.
///
/// Populated by `run_gpu_raytrace` in `FixedUpdate`, read by
/// `redistribute_raytrace_results` in `PostUpdate`.
#[derive(Resource, Default, Clone)]
pub struct RayTraceBatch {
    /// Per-ray entries with GPU results.
    pub entries: Vec<RayTraceBatchEntry>,
}

/// Runs GPU ray tracing synchronously from the main world.
///
/// Shares the `vello::Renderer` with the rendering pipeline via `Arc<Mutex<>>`
/// (same renderer used by [`GpuCollisionRunner`]).
#[derive(Resource, Clone)]
pub struct GpuRayTraceRunner {
    renderer: Arc<Mutex<vello::Renderer>>,
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
}

impl GpuRayTraceRunner {
    pub fn new(
        renderer: Arc<Mutex<vello::Renderer>>,
        device: wgpu::Device,
        queue: wgpu::Queue,
    ) -> Self {
        Self {
            renderer,
            device: Arc::new(device),
            queue: Arc::new(queue),
        }
    }

    /// Run GPU ray tracing synchronously on the given scene.
    /// Blocks until the GPU results are available.
    /// Returns one [`vello::RayTraceResult`] per encoded (ray, shape) pair.
    pub fn run_raytrace(&self, scene: &vello::RayTraceScene) -> Vec<vello::RayTraceResult> {
        let mut renderer = self.renderer.lock().unwrap();
        vello::util::block_on_wgpu(
            &self.device,
            renderer.render_raytrace_async(&self.device, &self.queue, scene.data()),
        )
        .unwrap()
        .unwrap_or_default()
    }
}
