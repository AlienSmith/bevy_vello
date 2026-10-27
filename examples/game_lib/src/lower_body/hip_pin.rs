//! Hip pin: step the Raibert resolver and advance `P1/P2/P3` by the hip's
//! per-tick displacement with three external position constraints.
//!
//! The target is the live particle position plus the resolver hip delta, a
//! centroid velocity servo toward the resolver hip velocity, and the
//! `support_share` fraction of gravity. There is no absolute pose term.
//!
//! The loop is closed: at the top of each step the solved `P3` (synced into
//! [`VelloParticle`] by `update_connection_particles` at the end of the
//! previous fixed step) is written back into the resolver state, so the next
//! step starts from where physics actually left the body. While the `P3` pin
//! is stiff this is a near-identity correction; it becomes the real feedback
//! path once the hard pin is later removed. `FEEDBACK=0` disables it for A/B
//! against the feed-forward baseline.
//!
//! The spine drive must stay disabled while the pin runs
//! ([`crate::character::SpineDriveEnabled`]).
//!
//! # Layout
//! Split into submodules under `hip_pin/`:
//! - [`params`] — `HipPinParams`, env parsing, gait-authority parsing.
//! - [`ground`] — foot contact debouncing and the `GroundSupport` resource.
//! - [`state`] — the `HipPin` component, attitude/frame math helpers.
//! - [`seeding`] — particle-position seeding and body measurement.
//! - [`drive`] — the `drive_hip_pin` system, loop-3 angular tick, reset.
//! Public names are re-exported here so downstream imports are unchanged.

mod drive;
mod ground;
mod params;
mod seeding;
mod state;

pub use drive::*;
pub use ground::*;
pub use params::*;
pub use seeding::*;
pub use state::*;

#[cfg(test)]
mod tests;
