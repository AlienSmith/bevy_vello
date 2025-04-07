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
        const DECOR = 0b00010000;
    }
}

impl Default for ColliderFlags {
    fn default() -> Self {
        ColliderFlags::None
    }
}

pub fn match_tag_to_mask(tag: ColliderFlags, mask: ColliderFlags) -> bool {
    (tag & mask).bits() != 0
}

use bevy::prelude::Component;

use crate::{spawn_pop_text_at, text::DefaultFonts};
#[derive(Component, Clone)]
pub struct ColliderResponds {
    pub damage: f32,
    pub allowed_collider_masks: ColliderFlags,
    pub collider_type: ColliderFlags,
    pub spawn_damage_text: bool,
}

impl Default for ColliderResponds {
    fn default() -> Self {
        Self {
            damage: Default::default(),
            allowed_collider_masks: Default::default(),
            collider_type: Default::default(),
            spawn_damage_text: Default::default(),
        }
    }
}
#[derive(Component, Clone, Default)]

pub struct SingleFrameCollider;

#[derive(Component, Clone)]
pub struct Health {
    pub health: f32,
}

pub fn handle_collisions(
    mut commands: Commands,
    mut collision_events: EventReader<Collision>,
    mut c_query: Query<(&mut Health, &ColliderResponds)>,
    s_query: Query<Entity, With<SingleFrameCollider>>,
    fonts: Res<DefaultFonts>,
) {
    for Collision(contacts) in collision_events.read() {
        if let Ok([(mut health1, responds1), (mut health2, responds2)]) =
            c_query.get_many_mut([contacts.entity1, contacts.entity2])
        {
            //Only one of them would be enemy.
            let pos = contacts.manifolds[0].contacts[0].point1;
            let mut damage: Option<String> = None;
            if match_tag_to_mask(responds2.collider_type, responds1.allowed_collider_masks) {
                health1.health -= responds2.damage;
                if responds1.spawn_damage_text && responds2.damage > 0.0 {
                    damage = Some(responds2.damage.to_string());
                }
            }
            if match_tag_to_mask(responds1.collider_type, responds2.allowed_collider_masks) {
                health2.health -= responds1.damage;
                if responds2.spawn_damage_text && responds1.damage > 0.0 {
                    damage = Some(responds1.damage.to_string());
                }
            }
            if let Some(damage_string) = damage {
                spawn_pop_text_at(
                    &mut commands,
                    &fonts,
                    Vec3 {
                        x: pos.x,
                        y: pos.y,
                        z: 1000.0,
                    },
                    &damage_string,
                    20.0,
                );
            }
        }
        //clean up single frame colliders
        for entity in s_query.iter() {
            commands.entity(entity).despawn();
        }
    }
}
