use bevy::{
    ecs::intern::{Interned, Interner},
    platform::collections::{HashMap, HashSet},
    prelude::*,
};
mod observers;
pub mod plugin;
mod systems;

#[derive(Component, Clone)]
pub struct Connectivity {
    pub name: Interned<str>,
    pub character: Entity,
    pub parts: HashSet<Entity>,
    pub death_propegate: bool,
}

#[derive(Component, Default, Clone)]
pub struct ConnectivityRoot {
    pub parts: HashMap<Interned<str>, Entity>,
}

impl Connectivity {
    pub fn new(character: Entity, death_propegate: bool, name: Interned<str>) -> Self {
        Connectivity {
            name,
            character,
            parts: HashSet::default(),
            death_propegate,
        }
    }
}

#[derive(Resource, Default)]
pub struct StringPool {
    pub pool: Interner<str>,
}

#[derive(Clone)]
pub struct SpineController {
    /// How aggressively the spine rotates toward the desired direction.
    /// Higher values snap faster (e.g. 4.0–8.0 for aim-first).
    pub rotation_gain: f32,
    /// Forward velocity scale when aligned.
    pub velocity_scale: f32,
}

impl Default for SpineController {
    fn default() -> Self {
        Self {
            rotation_gain: 6.0,
            velocity_scale: 5.0,
        }
    }
}

#[derive(Component, Clone, Default)]
pub struct CharacterController {
    pub move_vector: Vec2,
    pub spine_controller: SpineController,
}

impl CharacterController {
    pub fn new(move_vector: Vec2) -> Self {
        Self {
            move_vector,
            spine_controller: SpineController::default(),
        }
    }
}
