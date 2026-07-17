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
mod systems;

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
}

#[derive(Default, Resource, Clone)]
pub struct SimpleBroadPhase {
    pub(crate) broad_phase: BroadPhaseSimple,
}

//Use the debug_color and soft_body_global_transform in here to initialize this entity, instead of using the transform and scene
//in the VelloBundle.

//at this point we only use the translation from bevy global transform. and since we are not allowed to modify the global transform
//we need to make sure the item is initialized at the right location. otherwise the first frame of rendering is wrong.
#[derive(Clone, Default, Component)]
pub struct VelloCollider {
    pub(crate) shape: BezPath,
    pub frame_particles: [Particle; FRAME_PARTICLES_COUNT], //particles are always in world space.
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
        let scale_y = (aabb.y1 - aabb.x1) as f32 * transform.scale.y;
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
/// Holds its own `vello::Renderer` instance (separate from the rendering pipeline).
#[derive(Resource)]
pub struct GpuCollisionRunner {
    renderer: Arc<Mutex<vello::Renderer>>,
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
}

impl GpuCollisionRunner {
    pub fn new(device: wgpu::Device, queue: wgpu::Queue) -> Self {
        let renderer = vello::Renderer::new(
            &device,
            &RendererOptions {
                surface_format: None,
                timestamp_period: queue.get_timestamp_period(),
                use_cpu: false,
            },
        )
        .unwrap();
        Self {
            renderer: Arc::new(Mutex::new(renderer)),
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
}

//in bevy space use a y up x right coordinate.
#[derive(Event, Debug, Clone)]
pub struct VelloCollisionTrigger {
    pub entity_self: Entity,
    pub entity_other: Entity,
    pub collision_point: Vec2,
    pub normal_self: Vec2,
    pub normal_other: Vec2,
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
        // Extract the raw u32 internal index numbers
        let id_a = entity_a.index();
        let id_b = entity_b.index();

        // Sort them so that pack(A, B) yields the exact same key as pack(B, A)
        let min = std::cmp::min(id_a, id_b) as u64;
        let max = std::cmp::max(id_a, id_b) as u64;

        // Shift the smaller ID to the left 32 bits, then merge it with the larger ID
        (min << 32) | max
    }
}
