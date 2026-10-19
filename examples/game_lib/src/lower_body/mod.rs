//! Lower-body Raibert pose resolver (kinematic sandbox core).
//!
//! [`resolver`] holds the pure, unit-testable math ([`step_lower_body`],
//! [`solve_leg_ik`], and the forward/backward Raibert primitives). [`systems`]
//! wraps it in Bevy ECS (input → resolver → cached pose), and [`plugin`]
//! registers the sandbox plugin.

mod hip_pin;
mod plugin;
mod resolver;
mod systems;

pub use hip_pin::{
    drive_hip_pin, reset_hip_pin, update_ground_support, GroundSupport, HipPin, HipPinParams,
};

pub use plugin::LowerBodySandboxPlugin;
pub use resolver::{
    apex_to_launch_speed, compute_forward_target, compute_support_velocity, compute_v_max,
    compute_x_max, landing_speed, predict_apex_rise, solve_leg_ik, update_delta_l, DebugInfo,
    GaitLimits, GaitPhase, LegState, LowerBodyConfig, LowerBodyPose, ResolverState, VerticalPhase,
    VerticalState,
};
pub use systems::{
    drive_lower_body, reset_lower_body, LowerBodyController, LowerBodyInput, LowerBodyParams,
};
