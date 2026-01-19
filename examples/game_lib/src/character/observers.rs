use bevy::prelude::*;

use crate::character::{Connectivity, ConnectivityRoot};

pub fn on_remove_connectivity(
    trigger: Trigger<OnRemove, Connectivity>,
    mut commands: Commands,
    mut parts_query: Query<(Entity, &mut Connectivity)>,
    mut character_query: Query<&mut ConnectivityRoot>,
) {
    let part_entity = trigger.target(); // trigger.entity() is more common than .target() for lifecycle

    // 1. Extract data (Cloning Interned<str> is very cheap)
    let Ok((_, connectivity)) = parts_query.get(part_entity) else {
        return; // Entity might already be gone in complex despawn chains
    };

    // We clone because we need to iterate over 'parts' while modifying the world
    let mut c = connectivity.clone();

    // 2. Remove from the character root
    if let Ok(mut root) = character_query.get_mut(c.character) {
        root.parts.remove(&c.name);
    }

    // 3. Handle Neighbors
    for e in c.parts.drain() {
        if c.death_propegate {
            // Despawning is safe here as commands are deferred
            if let Ok(mut entity_cmd) = commands.get_entity(e) {
                entity_cmd.despawn();
            }
        } else {
            // Remove 'this' entity from the neighbor's connectivity list
            if let Ok((_, mut neighbor_conn)) = parts_query.get_mut(e) {
                neighbor_conn.parts.remove(&part_entity);
            }
        }
    }
}

pub fn on_remove_connectivity_root(
    trigger: Trigger<OnRemove, ConnectivityRoot>,
    mut commands: Commands,
    character_query: Query<&ConnectivityRoot>,
) {
    let entity = trigger.target(); // trigger.entity() is more common than .target() for lifecycle

    // 1. Extract data (Cloning Interned<str> is very cheap)
    let Ok(connectivity) = character_query.get(entity) else {
        return; // Entity might already be gone in complex despawn chains
    };

    // We clone because we need to iterate over 'parts' while modifying the world
    let mut c = connectivity.clone();
    // 3. Handle Neighbors
    for (_, e) in c.parts.drain() {
        // Despawning is safe here as commands are deferred
        if let Ok(mut entity_cmd) = commands.get_entity(e) {
            entity_cmd.despawn();
        }
    }
}
