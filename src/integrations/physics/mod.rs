mod plugin;
mod systems;

use bevy::{
    ecs::{
        component::{Component, ComponentHooks, HookContext, Mutable, StorageType},
        entity::Entity,
        event::Event,
        resource::Resource,
        world::DeferredWorld,
    },
    math::Vec2,
};

use bevy::prelude::*;

#[derive(Resource)]
pub struct VelloConstraintWorld {
    data: ConstraintWorld<Entity>,
}

impl VelloConstraintWorld {
    pub fn new(gravity: Vec2) -> Self {
        VelloConstraintWorld {
            data: ConstraintWorld {
                gravity: nalgebra::Vector2::<f32>::new(gravity.x, -gravity.y),
                ..Default::default()
            },
        }
    }
    // vello coordinate is x right y down
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.data.gravity = nalgebra::Vector2::<f32>::new(gravity.x, -gravity.y);
    }
}

#[derive(Clone, Copy)]
pub struct FilterData {
    pub impulse: Vec2,
}

pub use plugin::VelloCollisionResponsePlugin;
pub use thunderdome::Index;
pub use vello_physics::collision_response::Particle;
pub use vello_physics::soft_body::ExternalForce;
pub use vello_physics::soft_body::ParticleInfo;
pub use vello_physics::soft_body_connection::ConnectionInitConfig;
use vello_physics::{
    ConnectionConstraintInitConfig, ConstraintWorld, FrameBilinearConstraintConfig, FrameInitConfig,
};

// #[derive(Event)]
// pub struct ColliderExternalImpulseEvent {
//     pub entity: Entity,
//     pub impulse: Vec2,
// }

#[derive(Event)]
pub struct ColliderExternalImpulseEvent {
    pub filter: fn(Vec<ParticleInfo>, FilterData) -> Vec<vello_physics::soft_body::ExternalForce>,
    pub entity: Entity,
    pub filter_data: FilterData,
}

#[derive(Event)]
pub struct CharacterPivotForceEvent {
    pub character_entity: Entity,
    pub joint_entity: Entity,
    pub force: Vec2,
}

#[derive(Component)]
pub struct PivotVisualizer;

#[derive(Clone)]
pub struct VelloJoint {
    pub init_config: ConnectionConstraintInitConfig<Entity>,
    pub root_entity: Entity,
}

impl VelloJoint {
    pub fn new(
        connection_config: ConnectionConstraintInitConfig<Entity>,
        character: Entity,
    ) -> Self {
        Self {
            init_config: connection_config,
            root_entity: character,
        }
    }
}

impl Component for VelloJoint {
    const STORAGE_TYPE: StorageType = StorageType::Table;
    type Mutability = Mutable;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        // Match the signature: (DeferredWorld, HookContext)
        hooks.on_remove(|mut world: DeferredWorld, context: HookContext| {
            let entity = context.entity; // Entity ID is now inside the context

            // 1. Read the data
            let character = {
                let joint = world.get::<VelloJoint>(entity).unwrap();
                joint.root_entity
            };

            // 2. Queue the mutation
            world.commands().queue(move |world: &mut World| {
                if let Some(mut cv) = world.get_resource_mut::<VelloConstraintWorld>() {
                    if let Ok(group) = cv.data.get_group_mut(character) {
                        group.remove_connect_constraint(&entity);
                    }
                }
            });
        });
    }
}

/// A simple newtype component wrapper for [`vello::Scene`] for rendering.
#[derive(Clone)]
pub struct VelloParticle {
    pub particle: Particle,
    pub root_entity: Entity,
    pub frame_connect_config: FrameBilinearConstraintConfig,
}

impl VelloParticle {
    pub fn new(
        particle: Particle,
        entity: Entity,
        frame_connect_config: FrameBilinearConstraintConfig,
    ) -> Self {
        Self {
            particle,
            root_entity: entity,
            frame_connect_config,
        }
    }
}

impl Component for VelloParticle {
    const STORAGE_TYPE: StorageType = StorageType::Table;
    type Mutability = Mutable;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        hooks.on_remove(|mut world: DeferredWorld, context: HookContext| {
            let entity = context.entity;
            // 1. Read the data
            let character = {
                let joint = world.get::<VelloJoint>(entity).unwrap();
                joint.root_entity
            };
            world.commands().queue(move |world: &mut World| {
                if let Some(mut cv) = world.get_resource_mut::<VelloConstraintWorld>() {
                    if let Ok(group) = cv.data.get_group_mut(character) {
                        group.remove_connect_particle(&entity);
                    }
                }
            });
        });
    }
}

#[derive(Clone)]
pub struct VelloCharacterPhysicsRoot {
    pub shape_matching_frame_config: FrameInitConfig,
}

impl VelloCharacterPhysicsRoot {
    pub fn new(config: FrameInitConfig) -> Self {
        Self {
            shape_matching_frame_config: config,
        }
    }
}

impl Component for VelloCharacterPhysicsRoot {
    const STORAGE_TYPE: StorageType = StorageType::Table;
    type Mutability = Mutable;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        hooks.on_remove(|mut world: DeferredWorld, context: HookContext| {
            let entity = context.entity;
            world.commands().queue(move |world: &mut World| {
                if let Some(mut cv) = world.get_resource_mut::<VelloConstraintWorld>() {
                    cv.data.remove_group(entity);
                }
            });
        });
    }
}
