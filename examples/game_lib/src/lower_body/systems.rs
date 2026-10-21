//! Bevy ECS glue for the lower-body resolver: input → step → cached output.

use bevy::prelude::*;
use bevy::time::Fixed;

use crate::lower_body::resolver::{
    step_lower_body, DebugInfo, LowerBodyConfig, LowerBodyPose, ResolverState, VerticalPhase,
};

/// Runtime-tuneable resolver config, exposed as a single UI-editable Resource.
#[derive(Resource)]
pub struct LowerBodyParams {
    pub config: LowerBodyConfig,
}

impl Default for LowerBodyParams {
    fn default() -> Self {
        Self {
            config: LowerBodyConfig::default(),
        }
    }
}

/// Persistent resolver state + last computed pose, attached to the sandbox entity.
#[derive(Component)]
pub struct LowerBodyController {
    pub state: ResolverState,
    pub pose: LowerBodyPose,
    pub debug: DebugInfo,
}

impl LowerBodyController {
    pub fn new(config: &LowerBodyConfig) -> Self {
        let state = ResolverState::new(config);
        let pose = LowerBodyPose {
            hip: state.hip_pos,
            left_knee: state.hip_pos,
            left_foot: state.left.foot_pos,
            right_knee: state.hip_pos,
            right_foot: state.right.foot_pos,
            upper: state.hip_pos + Vec2::new(0.0, config.torso_len),
            hip_vel: Vec2::ZERO,
            left_knee_vel: Vec2::ZERO,
            left_foot_vel: Vec2::ZERO,
            right_knee_vel: Vec2::ZERO,
            right_foot_vel: Vec2::ZERO,
        };
        let debug = DebugInfo {
            x_max: 0.0,
            v_max: 0.0,
            v_desired: 0.0,
            v_clamped: 0.0,
            left_phase: state.left.phase,
            right_phase: state.right.phase,
            left_target_x: state.left.swing_to.x,
            right_target_x: state.right.swing_to.x,
            a_max: 0.0,
            v_abs: 0.0,
            v_support: 0.0,
            capture_x: 0.0,
            capture_saturated: false,
            v_phase: VerticalPhase::Stand,
            hip_height: config.hip_height,
            hip_vel_y: 0.0,
            h_desired: config.jump_apex,
            h_apex: 0.0,
            delta_l: 0.0,
            theta: 0.0,
            omega: 0.0,
            attitude_torque: 0.0,
            hip_accel_x: 0.0,
        };
        Self { state, pose, debug }
    }
}

/// Persistent jump-edge latch for [`drive_lower_body`].
///
/// The resolver runs in `FixedUpdate` (90 Hz), but the jump must be
/// *edge-triggered*: arming the buffer from the held level every tick re-fires
/// the hop the instant [`tick_vertical()`](crate::lower_body::resolver) returns
/// to `Stand` after landing, producing an endless double-hop while the key is
/// held.
///
/// `ButtonInput::just_pressed` is the wrong edge for this: it is a **render
/// frame** event, and at high (uncapped) frame rates most render frames run
/// zero fixed steps, so the single-frame edge is routinely missed and taps are
/// dropped. Instead we fetch the rising edge from the **held level sampled on
/// the fixed clock**: the key is physically down for tens of ms, so several
/// fixed steps observe it, and the latch turns that level into exactly one edge
/// per press.
#[derive(Resource, Default)]
pub struct LowerBodyInput {
    /// Whether any jump key was held at the previous fixed step.
    jump_was_held: bool,
}

/// Read arrow keys / WASD and advance the resolver one fixed timestep.
///
/// Runs in `FixedUpdate` with `Time<Fixed>`, so the resolver always steps with
/// the fixed physics `dt` (1/90 s), never the render-frame time. That keeps the
/// grounded [`regulate_height()`](crate::lower_body::resolver) spring inside its
/// stability band regardless of the achieved frame rate.
pub fn drive_lower_body(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Fixed>>,
    params: Res<LowerBodyParams>,
    mut input: ResMut<LowerBodyInput>,
    mut controllers: Query<&mut LowerBodyController>,
) {
    let mut input_dir = Vec2::ZERO;
    if keys.pressed(KeyCode::ArrowRight) || keys.pressed(KeyCode::KeyD) {
        input_dir.x += 1.0;
    }
    if keys.pressed(KeyCode::ArrowLeft) || keys.pressed(KeyCode::KeyA) {
        input_dir.x -= 1.0;
    }
    // Vertical channel: `y > 0.5` is a buffered jump request. Both Space and
    // Up/W raise it, but only on the key's *rising edge*, derived at fixed rate
    // from the held level so a high render frame rate cannot skip it (see
    // [`LowerBodyInput`]). The resolver consumes the request at the next
    // grounded `Stand → Load` edge, so one press ⇒ one hop and holding the key
    // no longer re-hops on landing.
    let jump_held = keys.pressed(KeyCode::Space)
        || keys.pressed(KeyCode::ArrowUp)
        || keys.pressed(KeyCode::KeyW);
    if jump_held && !input.jump_was_held {
        input_dir.y += 1.0;
    }
    input.jump_was_held = jump_held;
    if keys.pressed(KeyCode::ArrowDown) || keys.pressed(KeyCode::KeyS) {
        input_dir.y -= 1.0;
    }

    for mut controller in controllers.iter_mut() {
        let (pose, debug) = step_lower_body(
            input_dir,
            &params.config,
            &mut controller.state,
            time.delta_secs(),
        );
        controller.pose = pose;
        controller.debug = debug;
    }
}

/// Reset the resolver to the resting symmetric stance (press `R`).
pub fn reset_lower_body(
    keys: Res<ButtonInput<KeyCode>>,
    params: Res<LowerBodyParams>,
    mut controllers: Query<&mut LowerBodyController>,
) {
    if !keys.just_pressed(KeyCode::KeyR) {
        return;
    }
    for mut controller in controllers.iter_mut() {
        controller.state = ResolverState::new(&params.config);
    }
}
