use bevy::{
    ecs::{component::Component, entity::Entity, system::Resource},
    utils::PassHash,
};
use vello::{
    kurbo::{self, BezPath},
    CollisionScene,
};

mod extract;
mod plugin;
mod prepare;
mod systems;

#[derive(Default, Resource, Clone)]
pub struct VelloCollisionScene(CollisionScene);

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
