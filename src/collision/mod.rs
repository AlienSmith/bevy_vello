use bevy::{
    ecs::{component::Component, entity::Entity, system::Resource},
    utils::PassHash,
};
pub use plugin::VelloCollisionPlugin;
use vello::{
    kurbo::{self, BezPath},
    CollisionResult, CollisionScene,
};

mod extract;
mod plugin;
mod prepare;
mod systems;

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionScene(CollisionScene);

#[derive(Default, Resource, Clone)]
pub struct ExtractedVelloCollisionScene {
    pub(crate) scene: CollisionScene,
    pub(crate) sender: Option<Sender<Vec<CollisionResult>>>,
}

impl std::ops::Deref for VelloCollisionScene {
    type Target = CollisionScene;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for VelloCollisionScene {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionWorld {
    collision_pairs: Vec<(Entity, Entity)>,
}

#[derive(Clone, Default, Component)]
pub struct VelloCollider {
    shape: BezPath,
    aabb: kurbo::Rect,
}

impl VelloCollider {
    pub fn new(path: &BezPath, aabb: &kurbo::Rect) -> Self {
        Self {
            shape: path.clone(),
            aabb: *aabb,
        }
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
