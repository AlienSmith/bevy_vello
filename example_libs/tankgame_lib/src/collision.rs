use avian2d::prelude::*;
use bevy::prelude::*;
use bitflags::bitflags;
bitflags! {
    /// Represents a set of flags.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct ColliderFlags: u32 {
        const None = 0b00000000;
        const PLAYER = 0b00000001;
        const SHELL = 0b00000010;
        const ALIEN = 0b00000100;
        const EXPLOSION = 0b00001000;
    }
}

pub fn match_tag_to_mask(tag: ColliderFlags, mask: ColliderFlags) -> bool {
    (tag & mask).bits() != 0
}

use bevy::prelude::Component;
#[derive(Component, Clone)]
pub struct ColliderResponds {
    pub damage: f32,
    pub allowed_collider_masks: ColliderFlags,
    pub collider_type: ColliderFlags,
}

#[derive(Component, Clone)]
pub struct Health {
    pub health: f32,
}

pub fn handle_collisions(
    mut collision_events: EventReader<Collision>,
    mut c_query: Query<(&mut Health, &ColliderResponds)>,
) {
    for Collision(contacts) in collision_events.read() {
        if let Ok([(mut health1, responds1), (mut health2, responds2)]) =
            c_query.get_many_mut([contacts.entity1, contacts.entity2])
        {
            if match_tag_to_mask(responds2.collider_type, responds1.allowed_collider_masks) {
                health1.health -= responds2.damage;
            }
            if match_tag_to_mask(responds1.collider_type, responds2.allowed_collider_masks) {
                health2.health -= responds1.damage;
            }
        }
    }
}
