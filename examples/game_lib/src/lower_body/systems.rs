//! Bevy ECS glue for the lower-body resolver: input → step → cached output.

use bevy::prelude::*;

use crate::lower_body::resolver::{
    step_lower_body, DebugInfo, LowerBodyConfig, LowerBodyPose, ResolverState,
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
        };
        Self { state, pose, debug }
    }
}

/// Read arrow keys / WASD and advance the resolver one timestep.
///
/// The resolver is fully deterministic and framerate-independent (`dt` is
/// passed through), so this can run in `Update` with `Time::delta_secs()`.
pub fn drive_lower_body(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    params: Res<LowerBodyParams>,
    mut controllers: Query<&mut LowerBodyController>,
) {
    let mut input_dir = Vec2::ZERO;
    if keys.pressed(KeyCode::ArrowRight) || keys.pressed(KeyCode::KeyD) {
        input_dir.x += 1.0;
    }
    if keys.pressed(KeyCode::ArrowLeft) || keys.pressed(KeyCode::KeyA) {
        input_dir.x -= 1.0;
    }
    // y is ignored by the flat-floor resolver, but read it for completeness
    // (future non-flat gait patterns may use it for speed modulation).
    if keys.pressed(KeyCode::ArrowUp) || keys.pressed(KeyCode::KeyW) {
        input_dir.y += 1.0;
    }
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
