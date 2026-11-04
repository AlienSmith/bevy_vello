//! Pure Raibert lower-body pose resolver (kinematic, no XPBD).
//!
//! This module is the game-layer mathematical core of the lower-body sandbox
//! defined in `plans/raibert_sandbox.md`. It implements the forward step
//! planner and the backward velocity clamp from `plans/Raibert.md` term-for-term,
//! then resolves each leg (hip → knee → foot) with law-of-cosines 2-bone IK.
//!
//! Everything here is a deterministic pure function of `(input, config, state,
//! dt)` so it can be unit-tested without a running `App`, mirroring the arm IK
//! test style in `character/ik.rs`.
//!
//! # Coordinate convention
//! Bevy world coordinates: **x right, y up**. The flat floor is the horizontal
//! line `y = floor_y`. The hip travels along +x when the player pushes right.
//!
//! # Layout
//! Split into submodules under `resolver/` (each section keeps its original
//! doc header):
//! - [`types`] — config and state/pose types.
//! - [`vertical`] — math primitives + loop 2 (height/thrust state machine).
//! - [`attitude`] — loop 3 (torso tilt PD).
//! - [`laws`] — the gait scalar laws: reach/speed bounds, support velocity,
//!   floors, [`GaitLimits`], neutral point.
//! - [`ik`] — 2-bone IK and knee winding selection.
//! - [`placement`] — the DCM placement law family (capture point, horizon,
//!   gain, deadbeat) and the [`StepPlan`] it produces.
//! - [`stepping`] — swing trajectory profile and the per-leg gait FSM.
//! - [`walk`] — `step_lower_body`, one full resolver tick.
//! All public names are re-exported here so `use crate::lower_body::resolver::*`
//! and the `lower_body::mod` re-exports are unchanged from before the split.

mod attitude;
mod ik;
mod laws;
mod placement;
mod stepping;
mod types;
mod vertical;
mod walk;

pub use attitude::*;
pub use ik::*;
pub use laws::*;
pub use placement::*;
// `stepping` exposes only pub(crate) internals (the gait FSM); it is used by
// `walk` and the tests, nothing public to re-export.
pub use types::*;
pub use vertical::*;
pub use walk::*;

#[cfg(test)]
mod tests;
