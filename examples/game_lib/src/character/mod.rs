use bevy::{platform::collections::HashSet, prelude::*};
pub mod plugin;
mod system;

#[derive(Component, Default, Clone)]
pub struct Connectivity {
    pub character: Option<Entity>,
    pub parts: HashSet<Entity>,
    pub mark_of_death: bool,
    pub death_propegate: bool,
}

impl Connectivity {
    pub fn new(character: Option<Entity>, death_propegate: bool) -> Self {
        Connectivity {
            character,
            parts: HashSet::default(),
            mark_of_death: false,
            death_propegate,
        }
    }

    pub fn mark_as_dead(&mut self, _through_propergate: bool) {
        self.mark_of_death = true;
    }
}
