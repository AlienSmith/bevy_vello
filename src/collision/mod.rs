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
mod prepare;
mod systems;

pub const VELLO_COLLISION_WORLD_RATIO: f32 = 2.0;

use broad_phase::BroadPhaseQbvh;

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionScene {
    scene: CollisionScene,
    pair: Vec<(Entity, Entity)>,
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
}

impl VelloCollisionWorld {
    pub fn update_collision_pairs_if_previous_one_has_been_consumed(
        &mut self,
        q: &Query<(&VelloCollider, &GlobalTransform)>,
    ) -> bool {
        if self.collision_pairs.is_empty() {
            let mut temp: Vec<(Entity, Entity)> = vec![];
            for (e0, e1) in &self.collision_pairs_bvh {
                let (c, _t) = q.get(*e0).unwrap();
                let (c1, _t1) = q.get(*e1).unwrap();
                if c.inverse_mass > 0.0 || c1.inverse_mass > 0.0 {
                    temp.push((*e0, *e1));
                }
            }
            self.collision_pairs = temp;
            return true;
        }
        return false;
    }
}

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionBroadPhase {
    pub(crate) broad_phase: BroadPhaseQbvh,
}

#[derive(Clone, Default, Component)]
pub struct VelloCollider {
    pub(crate) shape: BezPath,
    pub(crate) aabb: kurbo::Rect, //aabb will only take the effect of position ignoring entity rotation and scale.
    pub(crate) initial_velocity: Vec2,
    pub(crate) inverse_mass: f32,
    pub(crate) debug_color: peniko::GlowColor,
}

impl VelloCollider {
    pub fn new(
        path: &BezPath,
        aabb: &kurbo::Rect,
        initial_velocity: Vec2,
        color: peniko::GlowColor,
        inverse_mass: f32,
    ) -> Self {
        Self {
            shape: path.clone(),
            aabb: *aabb,
            initial_velocity,
            inverse_mass,
            debug_color: color,
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
    Collision,         // Your first phase
    CollisionResponse, // Your second phase (runs after Phase1)
}

#[derive(Clone, Default)]
pub struct CollisionResults {
    pub pairs: Vec<(Entity, Entity)>,
    pub results: Vec<CollisionResult>,
}
