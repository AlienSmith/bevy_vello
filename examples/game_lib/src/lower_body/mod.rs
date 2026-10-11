//! Lower-body Raibert pose resolver (kinematic sandbox core).
//!
//! [`resolver`] holds the pure, unit-testable math ([`step_lower_body`],
//! [`solve_leg_ik`], and the forward/backward Raibert primitives). [`systems`]
//! wraps it in Bevy ECS (input → resolver → cached pose), and [`plugin`]
//! registers the sandbox plugin.

mod plugin;
mod resolver;
mod systems;

pub use plugin::LowerBodySandboxPlugin;
pub use resolver::{
    compute_forward_target, compute_support_velocity, compute_v_max, compute_x_max, solve_leg_ik,
    DebugInfo, GaitLimits, GaitPhase, LegState, LowerBodyConfig, LowerBodyPose, ResolverState,
};
pub use systems::{drive_lower_body, reset_lower_body, LowerBodyController, LowerBodyParams};
