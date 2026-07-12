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
                gravity,
                ..Default::default()
            },
        }
    }
    // vello coordinate is x right y down
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.data.gravity = gravity;
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
pub use vello_physics::ConnectionConstraintInitConfig;
use vello_physics::{
    collision_response::PartcileShapeMatchingConfig, utility::BalancedCoreFrame,
    AngularConstraintConfig, ConnectionConstraint, ConstraintWorld, FrameInitConfig,
    FramePositionConstraintConfig, FRAME_PARTICLES_COUNT,
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

#[derive(Event)]
pub struct CharacterFrameForceEvent {
    pub character_entity: Entity,
    pub forces: Vec<Vec2>,
}

#[derive(Event)]
pub struct CharacterPivotVelocityEvent {
    pub character_entity: Entity,
    pub joint_entity: Entity,
    pub velocity: Vec2,
}

#[derive(Event)]
pub struct CharacterAngularConstraintEvent {
    pub character_entity: Entity,
    pub joint_entity: Entity,
    pub config: AngularConstraintConfig,
}

#[derive(Event)]
pub struct CharacterPivotPositionEvent {
    pub character_entity: Entity,
    pub joint_entity: Entity,
    pub target: PartcileShapeMatchingConfig,
}

#[derive(Component)]
pub struct PivotVisualizer;

#[derive(Clone)]
pub struct VelloJoint {
    pub init_config: ConnectionConstraintInitConfig<Entity>,
    pub root_entity: Entity,
    pub constraint: ConnectionConstraint,
}

impl VelloJoint {
    pub fn new(
        connection_config: ConnectionConstraintInitConfig<Entity>,
        character: Entity,
    ) -> Self {
        let constraint = match connection_config {
            ConnectionConstraintInitConfig::Bilinear(_, _, _) => ConnectionConstraint::Bilinear,
            ConnectionConstraintInitConfig::Distance(_, _, _) => ConnectionConstraint::Distance,
            ConnectionConstraintInitConfig::Angular(_, _, _, _) => {
                ConnectionConstraint::Angular(AngularConstraintConfig::default())
            }
        };
        Self {
            init_config: connection_config,
            root_entity: character,
            constraint,
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
    pub particle_init: Particle,
    pub particle: Particle,
    pub shape_matching_init: FramePositionConstraintConfig,
    pub shape_matching: PartcileShapeMatchingConfig,
    pub root_entity: Entity,
}

impl VelloParticle {
    pub fn new(
        particle: Particle,
        entity: Entity,
        shape_matching_init: FramePositionConstraintConfig,
    ) -> Self {
        Self {
            particle_init: particle,
            particle,
            shape_matching_init,
            shape_matching: Default::default(),
            root_entity: entity,
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
                let joint = world.get::<VelloParticle>(entity).unwrap();
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
    pub frame_entities: [Entity; FRAME_PARTICLES_COUNT],
    pub frame_coordinates: BalancedCoreFrame,
    pub initial_frame_coordinates: Option<BalancedCoreFrame>, //this is used to indicate that the physics world is in sync with game world.
}

impl VelloCharacterPhysicsRoot {
    pub fn new(config: FrameInitConfig, frame_entities: [Entity; FRAME_PARTICLES_COUNT]) -> Self {
        Self {
            shape_matching_frame_config: config,
            frame_entities,
            initial_frame_coordinates: None,
            frame_coordinates: Default::default(),
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
