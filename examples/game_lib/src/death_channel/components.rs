use bevy::prelude::*;

/// Marker indicating this entity is a detached physics body (first hit).
/// A second hit (Die + Detached) will despawn it.
#[derive(Component, Clone, Debug)]
pub struct Detached;
