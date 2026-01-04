use bevy::prelude::*;

use crate::character::Connectivity;

pub fn clean_up_dead_body_parts(
    mut command: Commands,
    mut query: Query<(Entity, &mut Connectivity)>,
) {
    let to_remove: Vec<_> = query
        .iter_mut()
        .filter_map(|(e, mut c)| {
            if c.mark_of_death {
                Some((e, std::mem::take(&mut *c)))
            } else {
                None
            }
        })
        .collect();

    for (e, c) in to_remove {
        if let Some(character) = c.character {
            if let Ok((_, mut connect)) = query.get_mut(character) {
                connect.parts.remove(&e);
            }
        }
        for item in c.parts {
            if let Ok((_, mut connect)) = query.get_mut(item) {
                connect.parts.remove(&e);
                if c.death_propegate {
                    connect.mark_as_dead(true);
                }
            }
        }
        command.entity(e).despawn();
    }
}
