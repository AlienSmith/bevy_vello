//! Walker control bridge — observe / control / actuate (see
//! `plans/walker-control-rewrite.md`).
//!
//! Build status (per plan §8): the pure **actuate** core is implemented and
//! unit-tested here first; the Bevy emission adapter, `observe`, and the
//! swappable `control` seam land on top of these contracts next.

pub mod actuate;
pub mod control;
pub mod emit;
pub mod observe;
pub mod plugin;
pub mod script;
pub mod telemetry;
pub mod walker;
