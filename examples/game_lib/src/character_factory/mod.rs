use bevy::prelude::*;
use vello::kurbo::{BezPath, Rect};
use vello_physics::{
    collision_response::Particle, CollisionConstraintConfig, ConnectionConstraintInitConfig,
    FramePositionConstraintConfig, SoftBodyInitConfig,
};

mod observers;
pub mod plugin;

#[derive(Component)]
pub struct CharacterRoot {
    pub svg_asset_id: String,
    pub blueprint_asset_id: String,
    pub collision_group: u32,
}

/// Events for runtime modification of a character's physics parts.
///
/// Fired by game code and processed in `handle_character_part_events` (Update schedule).
/// Spawned entities are picked up by `generate_connection` / `generate_soft_body_for_collider`
/// in the same frame's PostUpdate via their `Added<T>` queries.
#[derive(Event)]
pub enum CharacterPartEvent {
    /// Add a soft-body collider to a character.
    AddCollider {
        character: Entity,
        path_id: String,
        svg_path: BezPath,
        rect: Rect,
        inv_mass: f32,
        soft_body_config: SoftBodyInitConfig,
        collision_config: CollisionConstraintConfig,
        transform: Transform,
    },
    /// Add a particle (connection particle) to a character.
    ///
    /// Particles are spawned in pass 1 (alongside colliders) so that joints
    /// referencing them in pass 2 can resolve their entity handles.
    AddParticle {
        character: Entity,
        path_id: String,
        /// Initial particle state (position, inverse mass, etc.).
        particle: Particle,
        /// Shape-matching constraint config for this particle.
        shape_matching: FramePositionConstraintConfig,
    },
    /// Add a joint where the constraint's entity references are given as string
    /// path_ids (e.g. "PRLA", "pistol") instead of raw entity handles.
    ///
    /// The handler resolves them from the character's [`ConnectivityRoot`] in pass 2,
    /// which is useful when the collider/particle entity is not yet known at the
    /// call site (e.g. it was just created by an `AddCollider` event in pass 1).
    ///
    /// The entities the joint connects to are derived from the `config` field's
    /// string path_ids, so there is no need to provide them separately.
    AddJoint {
        character: Entity,
        path_id: String,
        /// Connection config using string path_ids that will be resolved to
        /// entities from `ConnectivityRoot.parts`.
        config: ConnectionConstraintInitConfig<String>,
    },
    /// Register an existing free entity (e.g. a weapon collider) as a part of
    /// a character by adding a [`Connectivity`] component and inserting it into
    /// the character's [`ConnectivityRoot`].
    ///
    /// Use this for "pick up" — the entity already exists as a standalone
    /// physics body and should now be attached to the character.
    RegisterPart {
        /// The character root entity.
        character: Entity,
        /// The existing entity to register.
        entity: Entity,
        /// Name for `ConnectivityRoot.parts` lookup.
        path_id: String,
    },
    /// Unregister a part from its character by removing its [`Connectivity`]
    /// component. The `on_remove_connectivity` observer will clean up the
    /// [`ConnectivityRoot`] and despawn any dependent joints.
    ///
    /// Use this for "throw/drop" — the entity remains alive as a free physics
    /// body, no longer connected to the character.
    ///
    /// The part is identified by its string `path_id` (e.g. "pistol") rather
    /// than an entity handle, since the external world knows parts by name.
    UnregisterPart {
        /// The character root entity whose part to unregister.
        character: Entity,
        /// The string path_id of the part to unregister (e.g. "pistol").
        /// This is looked up from [`ConnectivityRoot.parts`] to find the entity.
        path_id: String,
    },
}
