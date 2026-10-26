//! Lower-body Raibert pose resolver (kinematic sandbox core).
//!
//! [`resolver`] holds the pure, unit-testable math ([`step_lower_body`],
//! [`solve_leg_ik`], and the forward/backward Raibert primitives). [`systems`]
//! wraps it in Bevy ECS (input → resolver → cached pose), and [`plugin`]
//! registers the sandbox plugin.

mod hip_pin;
mod measure;
mod plugin;
mod resolver;
mod systems;

pub use hip_pin::{
    drive_hip_pin, measure_body, parse_gait_authority, reset_hip_pin, tick_loop3_angular,
    update_ground_support, FootContact, GroundSupport, HipPin, HipPinParams,
};
pub use measure::{
    center_of_mass, com_velocity, ema_alpha, torso_pitch, update_measurements, BodyMeasureFilter,
    BodyMeasurements, ParticleSample,
};

pub use plugin::LowerBodySandboxPlugin;
pub use resolver::{
    apex_to_launch_speed, attitude_lean_pd, attitude_pd, capture_point, compute_forward_target,
    compute_support_velocity, compute_v_max, compute_x_max, enforce_knee_winding, facing_sign,
    landing_speed, predict_apex_rise, select_anterior_knee, solve_leg_ik, tick_attitude,
    update_delta_l, AttitudeState, CaptureMeasure, DebugInfo, GaitAuthority, GaitLimits, GaitPhase,
    LegContactInput, LegState, LegThrust, LowerBodyConfig, LowerBodyPose, ResolverState,
    VerticalPhase, VerticalState,
};
pub use systems::{
    drive_lower_body, reset_lower_body, LowerBodyController, LowerBodyInput, LowerBodyParams,
};
