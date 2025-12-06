use bevy::prelude::*;
use bevy::{
    ecs::{component::Component, entity::Entity, schedule::SystemSet, system::Resource},
    math::{Vec2, Vec4},
};
pub use plugin::VelloCollisionPlugin;
use vello::{
    kurbo::{self, BezPath},
    peniko, CollisionResult, CollisionScene,
};

mod broad_phase;
mod extract;
mod plugin;
mod systems;

pub const VELLO_COLLISION_WORLD_RATIO: f32 = 4.0;

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
pub struct ExtractedVelloCollisionScene {
    pub(crate) scene: CollisionScene,
    pub(crate) pairs: Vec<(Entity, Entity)>,
    pub(crate) sender: Option<Sender<CollisionResults>>,
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
                    if c.is_soft_body() || c1.is_soft_body() {
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

//TODO: remvoe this component from entity could cause memory leak in the xpbd softbody system and qbvh broad phase of collision detection
// make this component private by hide it in a bundle, do not expose it to user.
#[derive(Clone, Default, Component)]
pub struct VelloCollider {
    pub(crate) shape: BezPath,
    pub(crate) aabb: kurbo::Rect, //aabb will only take the effect of position ignoring entity rotation and scale.
    pub(crate) initial_velocity: Vec2,
    pub(crate) _inverse_mass: f32,
    pub(crate) debug_color: peniko::Brush,
    pub(crate) is_soft_body: bool,
    pub(crate) uvs: Option<Vec<f32>>,
    pub(crate) soft_body_config: Option<SoftBodyInitConfig>,
    pub(crate) collision_config: Option<CollisionConstraintConfig>,
    pub is_selected: bool,
}

impl VelloCollider {
    pub fn is_soft_body(&self) -> bool {
        self.is_soft_body
    }

    pub fn new(
        path: &BezPath,
        aabb: &kurbo::Rect,
        initial_velocity: Vec2,
        color: peniko::Brush,
        inverse_mass: f32,
        is_soft_body: bool,
        uvs: Option<Vec<f32>>,
        soft_body_init_config: Option<SoftBodyInitConfig>,
        collision_constraint_config: Option<CollisionConstraintConfig>,
    ) -> Self {
        Self {
            shape: path.clone(),
            aabb: *aabb,
            initial_velocity,
            _inverse_mass: inverse_mass,
            debug_color: color,
            is_soft_body,
            uvs,
            is_selected: false,
            soft_body_config: soft_body_init_config,
            collision_config: collision_constraint_config,
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

use crossbeam_channel::{bounded, Receiver, Sender};
pub use vello_physics::CollisionConstraintConfig;
pub use vello_physics::SoftBodyInitConfig;

use crate::collision::broad_phase::BroadPhaseSimple;

// Thread-safe channel for GPU → Main thread communication
#[derive(Resource)]
pub struct GpuDataChannel<T: Send + 'static> {
    pub sender: Sender<T>,
    pub receiver: Receiver<T>,
}

impl<T: Send + 'static> GpuDataChannel<T> {
    pub fn new(capacity: usize) -> Self {
        let (sender, receiver) = bounded(capacity);
        Self { sender, receiver }
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
