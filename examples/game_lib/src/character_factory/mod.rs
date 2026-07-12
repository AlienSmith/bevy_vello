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
    AddJoint {
        character: Entity,
        path_id: String,
        /// The entities this joint connects to (already-resolved handles for
        /// connectivity tracking).
        connected_entities: Vec<Entity>,
        /// Connection config using string path_ids that will be resolved to
        /// entities from `ConnectivityRoot.parts`.
        config: ConnectionConstraintInitConfig<String>,
    },
}
