use bevy::{platform::collections::HashSet, prelude::*};
use bevy_vello::{integrations::physics::VelloJoint, VelloCollider};
pub mod plugin;
mod system;
#[derive(Default, Clone, Copy, Debug)]
pub enum BodyPartType {
    #[default]
    Body,
    Joint,
}

#[derive(Component, Default, Clone)]
pub struct Connectivity {
    character: Option<Entity>,
    parts: HashSet<Entity>,
    mark_of_death: bool,
    death_propegate: bool,
}

impl Connectivity {
    pub fn new(character: Option<Entity>, parts: &[Entity], death_propegate: bool) -> Self {
        Connectivity {
            character,
            parts: parts.iter().cloned().collect(),
            mark_of_death: false,
            death_propegate,
        }
    }

    pub fn mark_as_dead(&mut self, _through_propergate: bool) {
        self.mark_of_death = true;
    }
}

#[derive(Bundle)]
pub struct BodyColliderBundle {
    pub connect: Connectivity,
    pub collider: VelloCollider,
}

impl BodyColliderBundle {
    pub fn new(character: Option<Entity>, parts: &[Entity], collider: VelloCollider) -> Self {
        Self {
            connect: Connectivity::new(character, parts, true),
            collider,
        }
    }
}

#[derive(Bundle)]
pub struct BodyJointBundle {
    pub connect: Connectivity,
    pub joint: VelloJoint,
}

impl BodyJointBundle {
    pub fn new(character: Option<Entity>, parts: &[Entity], joint: VelloJoint) -> Self {
        Self {
            connect: Connectivity::new(character, parts, true),
            joint,
        }
    }
}
