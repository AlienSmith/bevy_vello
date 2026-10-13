//! Hip-pin bridge: drive the character's rigid `P1/P2/P3` frame from the
//! Raibert lower-body resolver.
//!
//! The resolver's hip output — **position and velocity** — is the model; the
//! pin realizes it on the three pinned particles with a **PD command + a stiff
//! external position constraint**:
//!
//! * the resolver is stepped with the live fixed `dt` and the game input map
//!   ([`PlayerAction`]),
//! * its hip is mapped into the character's **Vello (y-down) world frame**,
//!   anchored to the live `P3` captured once when the pin first arms,
//! * the `P1`/`P2`/`P3` *pose* targets are derived from the captured rest
//!   offsets by a **pure rigid transform** (translation + the `theta` attitude),
//! * each particle's *command* target is that pose advanced by a PD
//!   acceleration: `a = Kp·(pose − x) + Kd·(v_hip − v)`, integrated to a
//!   position offset `target = pose + a·dt²`. The velocity term uses the
//!   resolver's hip velocity as feed-forward, so the pin tracks a moving hip
//!   without lag ([`pd_target`]),
//! * the constraint's stiffness `1/compliance` is **softened in speed space**
//!   — `compliance_eff = compliance·(1 + s·(soft_scale−1))`,
//!   `s = min(|v|/v_ref, 1)` — so a fast frame yields instead of fighting the
//!   motion, mirroring the limb PD's ω-scheduled compliance
//!   ([`softened_compliance`]),
//! * exactly **three** [`CharacterExternalPositionConstraintEvent`]s are
//!   emitted (`P1`, `P2`, `P3`); **nothing angular** is emitted, so the
//!   `P1_P2_P3` angular joint is left at rest and cannot fight the pin.
//!
//! The spine drive ([`crate::character::systems::tick_spine_drive`]) must be
//! switched off while this runs — the code is **retained but disabled**, not
//! removed, via [`SpineDriveEnabled`](crate::character::SpineDriveEnabled).

use bevy::prelude::*;
use bevy_vello::integrations::physics::{
    CharacterExternalPositionConstraintEvent, VelloCharacterPhysicsRoot, VelloConstraintWorld,
    VelloParticle,
};
use leafwing_input_manager::prelude::ActionState;
use vello_physics::ExternalPositionConstraintConfig;

use crate::character::SpineController;
use crate::input_management::actions::PlayerAction;
use crate::lower_body::resolver::{step_lower_body, LowerBodyConfig, ResolverState};

/// Runtime knobs for the hip pin. Kept as a Resource (mirroring
/// [`crate::lower_body::LowerBodyParams`]) so the tuning UI can edit it live.
#[derive(Resource)]
pub struct HipPinParams {
    /// The Raibert resolver config (thigh/calf/hip-height/speeds/gravity...).
    pub config: LowerBodyConfig,
    /// PD **position** gain of the command, in `1/s²` (`a += kp·(pose − x)`).
    /// `0` disables the P term; the pin then tracks `pose` through `compliance`
    /// alone (the original position-only behaviour). Start small and raise
    /// until the frame snaps without ringing (ω ≈ √kp).
    pub kp: f32,
    /// PD **velocity-matching** gain of the command, in `1/s`
    /// (`a += kd·(v_hip − v)`). Uses the resolver's hip velocity as the
    /// feed-forward target, so the pin tracks a moving hip without lag. `0`
    /// disables the velocity term. A good start is `kd ≈ 2√kp`.
    pub kd: f32,
    /// Speed (px/s) at which the compliance softening saturates: `s` reaches `1`
    /// at `|v| = v_ref`.
    pub v_ref: f32,
    /// Softening factor: at `s = 1` the emitted compliance is multiplied by this
    /// (larger ⇒ softer-on-fast). `1.0` disables softening (constant
    /// compliance).
    pub soft_scale: f32,
    /// External position constraint compliance **at rest** (lower = stiffer).
    /// The stock `ExternalPositionConstraint` is position-only, so this is the
    /// base stiffness of the pin; it is scaled up (softer) with speed by
    /// [`Self::soft_scale`].
    pub compliance: f32,
    /// External position constraint damping (0..1). Legacy raw velocity damper;
    /// prefer [`Self::kd`] once tuning.
    pub damping: f32,
}

impl Default for HipPinParams {
    fn default() -> Self {
        Self {
            config: LowerBodyConfig::default(),
            // Neutral defaults: `kp = kd = 0` reproduces the position-only pin,
            // so the PD is opt-in at tuning time.
            kp: 0.0,
            kd: 0.0,
            v_ref: 300.0,
            soft_scale: 1.0, // 1 ⇒ constant compliance (softening off)
            // Snaps the (now-light) particles to the commanded pose.
            compliance: 1e-7,
            damping: 0.01,
        }
    }
}

/// Per-character hip pin. Attach to the character root entity. All fields are
/// seeded lazily on the first tick the physics root reports ready.
#[derive(Component)]
pub struct HipPin {
    /// Persistent Raibert resolver state (owns `hip_pos`/`hip_vel`).
    pub state: ResolverState,
    /// `P1`/`P2`/`P3` rest offsets **relative to `P3`**, in Vello (y-down)
    /// world space. `rest_offsets[2]` (P3 relative to itself) is always zero.
    pub rest_offsets: [Vec2; 3],
    /// Resolver hip at seed time (local y-up). Only the *delta* from this is
    /// mapped into the world, so the absolute resolver frame is irrelevant.
    pub seed_hip: Vec2,
    /// Live `P3` world (Vello y-down) position at seed time — the anchor the
    /// rigid frame is pinned to.
    pub anchor_p3: Vec2,
    /// Set once [`Self::rest_offsets`]/[`Self::seed_hip`]/[`Self::anchor_p3`]
    /// have been captured from the assembled particles.
    pub ready: bool,
    /// Master enable. When false the pin emits nothing (spine drive may run).
    pub enabled: bool,
    /// Rigid-frame attitude in radians. Step 1 uses `0.0` (upright). Reserved
    /// for the later PD-attitude phase.
    pub theta: f32,
    /// Whether a jump action was held at the previous step. Turns the held
    /// level into an edge at the fixed `dt` (a render-frame `just_pressed` is
    /// dropped whenever a render frame runs zero fixed steps).
    jump_was_held: bool,
}

impl HipPin {
    pub fn new(config: &LowerBodyConfig) -> Self {
        Self {
            state: ResolverState::new(config),
            rest_offsets: [Vec2::ZERO; 3],
            seed_hip: Vec2::ZERO,
            anchor_p3: Vec2::ZERO,
            ready: false,
            enabled: true,
            theta: 0.0,
            jump_was_held: false,
        }
    }
}

/// Pure rigid-frame derivation: place `P1/P2/P3` by translating the anchor by
/// `world_delta` and rotating each rest offset by `theta`. No per-tick
/// re-anchoring, so the three targets are a fixed rigid body. Exposed for
/// unit testing (bone lengths must be preserved exactly).
pub fn rigid_frame_targets(
    anchor_p3: Vec2,
    rest_offsets: [Vec2; 3],
    world_delta: Vec2,
    theta: f32,
) -> [Vec2; 3] {
    let (sin, cos) = theta.sin_cos();
    rest_offsets.map(|o| {
        let rotated = Vec2::new(o.x * cos - o.y * sin, o.x * sin + o.y * cos);
        anchor_p3 + world_delta + rotated
    })
}

/// PD command target for one pinned particle: the pose target advanced by the
/// commanded acceleration `a = Kp·(pose − x) + Kd·(v_hip − v)`, integrated to a
/// position offset (`target = pose + a·dt²`). `v_hip` is the resolver's hip
/// velocity (the rigid frame translates as one, so all three particles carry
/// it). Pure; exposed for unit testing.
pub fn pd_target(
    pose_t: Vec2,
    pos: Vec2,
    vel: Vec2,
    v_hip: Vec2,
    dt: f32,
    kp: f32,
    kd: f32,
) -> Vec2 {
    let a = kp * (pose_t - pos) + kd * (v_hip - vel);
    pose_t + a * (dt * dt)
}

/// Speed-softened compliance: `base·(1 + s·(soft_scale−1))` with
/// `s = min(speed/v_ref, 1)`. `soft_scale = 1` (or `v_ref ≤ 0`) returns `base`
/// unchanged. Pure; exposed for unit testing.
pub fn softened_compliance(base: f32, soft_scale: f32, v_ref: f32, speed: f32) -> f32 {
    let s = if v_ref > 0.0 {
        (speed / v_ref).min(1.0)
    } else {
        0.0
    };
    base * (1.0 + s * (soft_scale - 1.0))
}

/// Seed the pin from the assembled character: capture the `P3` world anchor and
/// the `P1/P2` rest offsets. No-op once `ready` or if the character is not yet
/// assembled.
fn try_seed(
    pin: &mut HipPin,
    spine: &SpineController,
    root: &VelloCharacterPhysicsRoot,
    p_q: &Query<&VelloParticle>,
) {
    if pin.ready || root.initial_frame_coordinates.is_none() {
        return;
    }
    // spine.particles == [PH, P0, P1, P2, P3]; we drive the last three.
    let p1 = match p_q.get(spine.particles[2]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    let p2 = match p_q.get(spine.particles[3]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    let p3 = match p_q.get(spine.particles[4]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    pin.rest_offsets = [p1 - p3, p2 - p3, Vec2::ZERO];
    pin.anchor_p3 = p3;
    pin.seed_hip = pin.state.hip_pos;
    pin.ready = true;
}

/// Step the Raibert resolver and emit the three stiff external position
/// constraints that pin `P1/P2/P3` to the commanded rigid frame.
///
/// Order this **before** the physics constraint world is stepped (the tuning
/// app schedules it `.before(update_constraint_world)`), mirroring how the
/// spine drive used to queue its events.
pub fn drive_hip_pin(
    time: Res<Time>,
    params: Res<HipPinParams>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<(&SpineController, &VelloCharacterPhysicsRoot)>,
    p_q: Query<&VelloParticle>,
    action_q: Query<&ActionState<PlayerAction>>,
    mut world: ResMut<VelloConstraintWorld>,
) {
    let dt = time.delta_secs();

    for (entity, mut pin) in pin_q.iter_mut() {
        if !pin.enabled {
            continue;
        }
        let Ok((spine, root)) = spine_q.get(entity) else {
            continue;
        };
        try_seed(&mut pin, spine, root, &p_q);
        if !pin.ready {
            continue;
        }

        // --- Game input → resolver input (Vello y-down action map). ---
        let mut input_dir = Vec2::ZERO;
        if let Ok(action) = action_q.get(entity) {
            if action.pressed(&PlayerAction::MoveRight) {
                input_dir.x += 1.0;
            }
            if action.pressed(&PlayerAction::MoveLeft) {
                input_dir.x -= 1.0;
            }
            // Resolver convention: `input_dir.y > 0.5` is a buffered jump. It
            // must be *edge-triggered*: the resolver re-enters `Stand` while the
            // key may still be held, so a held-level arm re-fires the hop on
            // every landing (the double-hop). Derive the rising edge from the
            // held level here at the fixed `dt` rather than via the action's
            // render-frame `just_pressed`, which a high frame rate drops.
            let jump_held = action.pressed(&PlayerAction::MoveUp);
            if jump_held && !pin.jump_was_held {
                input_dir.y += 1.0;
            }
            pin.jump_was_held = jump_held;
            if action.pressed(&PlayerAction::MoveDown) {
                input_dir.y -= 1.0;
            }
        }

        // --- Step the resolver (deterministic, fixed-cadence). ---
        let (pose, _debug) = step_lower_body(input_dir, &params.config, &mut pin.state, dt);

        // --- Map resolver hip (y-up) → Vello world (y-down) translation. ---
        // Only the delta from the seed hip is used, so the resolver's absolute
        // frame never leaks into the world. The resolver's hip velocity is the
        // PD feed-forward target (all three frame particles translate as one).
        let world_delta = Vec2::new(pose.hip.x - pin.seed_hip.x, -(pose.hip.y - pin.seed_hip.y));
        let v_hip = Vec2::new(pin.state.hip_vel.x, -pin.state.hip_vel.y);

        // --- Rigid pose (attitude theta; 0 = upright). ---
        let pose_targets =
            rigid_frame_targets(pin.anchor_p3, pin.rest_offsets, world_delta, pin.theta);

        // --- Gravity ownership on the pinned frame. ---
        // The XPBD step applies world gravity to P1/P2/P3 *before* this
        // constraint solves (and `inv_mass` is now 1.0, so that term is no
        // longer negligible). The resolver already integrates the same gravity
        // into `pose`, so the pinned particle would receive `g` twice. Feed the
        // same `g·dt²` forward into the target to hand gravity ownership to the
        // sandbox; the constraint then cancels the physics gravity exactly and
        // no `-g` bias is left on the frame. This is inert at zero gravity.
        let g_vello = world.gravity();
        let g_dt2 = g_vello * (dt * dt);

        // --- Per-particle PD command + speed-softened compliance. ---
        // Exactly 3 position constraints, nothing angular.
        for (i, pose_t) in [2usize, 3, 4].into_iter().zip(pose_targets.iter()) {
            let (pos, vel) = match p_q.get(spine.particles[i]) {
                Ok(p) => (p.particle.pos, p.particle.velocity),
                Err(_) => continue,
            };
            let target = pd_target(*pose_t, pos, vel, v_hip, dt, params.kp, params.kd) + g_dt2;
            let compliance = softened_compliance(
                params.compliance,
                params.soft_scale,
                params.v_ref,
                vel.length(),
            );

            world.queue_character_one_time_external_position_constraint(
                &CharacterExternalPositionConstraintEvent {
                    character_entity: entity,
                    joint_entity: spine.particles[i],
                    config: ExternalPositionConstraintConfig {
                        target,
                        compliance,
                        damping: params.damping,
                    },
                },
            );
        }
    }
}

/// Reset the pin to its resting symmetric stance and re-arm the seed (press `R`).
pub fn reset_hip_pin(
    keys: Res<ButtonInput<KeyCode>>,
    params: Res<HipPinParams>,
    mut pin_q: Query<&mut HipPin>,
) {
    if !keys.just_pressed(KeyCode::KeyR) {
        return;
    }
    for mut pin in pin_q.iter_mut() {
        pin.state = ResolverState::new(&params.config);
        pin.ready = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rigid_frame_preserves_bone_lengths() {
        // y-down Vello offsets: P1 above-right of P3, P2 near-above of P3.
        let rest = [Vec2::new(12.0, -60.0), Vec2::new(-3.0, -30.0), Vec2::ZERO];
        let anchor = Vec2::new(347.0, 346.0);
        let delta = Vec2::new(55.0, -18.0);

        for theta in [0.0_f32, 0.4, -1.1] {
            let t = rigid_frame_targets(anchor, rest, delta, theta);
            // P1–P3 and P2–P3 lengths must match the rest offsets exactly.
            assert!((t[0] - t[2]).length() - rest[0].length() < 1e-3);
            assert!((t[1] - t[2]).length() - rest[1].length() < 1e-3);
            // P3 is the anchor translated by the world delta.
            assert!((t[2] - (anchor + delta)).length() < 1e-3);
        }
    }

    #[test]
    fn rigid_frame_theta_zero_is_pure_translation() {
        let rest = [Vec2::new(12.0, -60.0), Vec2::new(-3.0, -30.0), Vec2::ZERO];
        let anchor = Vec2::new(10.0, 20.0);
        let delta = Vec2::new(4.0, 5.0);
        let t = rigid_frame_targets(anchor, rest, delta, 0.0);
        for k in 0..3 {
            assert!((t[k] - (anchor + delta + rest[k])).length() < 1e-4);
        }
    }

    /// End-to-end gate for the *mapping* (resolver → rigid P1/P2/P3 frame):
    /// run the real resolver for a walking episode across several fixed `dt`s
    /// (including an extreme 0.2 s step) and assert the derived frame (a) stays
    /// finite, (b) preserves both bone lengths exactly, and (c) puts P3 at the
    /// anchor translated by the resolver's hip delta. Deterministic and
    /// CI-runnable; the visual tracking gate (measured particles → targets under
    /// the XPBD solver) stays a user-run check in the tuning app, per plan §8.
    #[test]
    fn rigid_frame_tracks_resolver_across_dts() {
        let cfg = LowerBodyConfig::default();
        let rest = [Vec2::new(12.0, -60.0), Vec2::new(-3.0, -30.0), Vec2::ZERO];
        let anchor = Vec2::new(347.0, 346.0);
        let l1 = rest[0].length();
        let l2 = rest[1].length();

        for dt in [1.0_f32 / 90.0, 1.0 / 15.0, 0.2] {
            let mut state = ResolverState::new(&cfg);
            let seed = state.hip_pos;
            for step in 0..240 {
                // Hold "move right" — the resolver walks (bounded). No jump: the
                // vertical loop diverges at a coarse dt, which would blow f32
                // precision and mask the mapping under test.
                let input = Vec2::new(1.0, 0.0);
                let (pose, _debug) = step_lower_body(input, &cfg, &mut state, dt);
                let delta = Vec2::new(pose.hip.x - seed.x, -(pose.hip.y - seed.y));
                let t = rigid_frame_targets(anchor, rest, delta, 0.0);

                assert!(
                    t.iter().all(|p| p.x.is_finite() && p.y.is_finite()),
                    "non-finite frame at step {step}, dt {dt}"
                );
                let d1 = (t[0] - t[2]).length();
                let d2 = (t[1] - t[2]).length();
                assert!(
                    (d1 - l1).abs() < 1e-3,
                    "P1-P3 drift, dt {dt} step {step}: got {d1} want {l1} t={t:?}"
                );
                assert!(
                    (d2 - l2).abs() < 1e-3,
                    "P2-P3 drift, dt {dt} step {step}: got {d2} want {l2} t={t:?}"
                );
                assert!(
                    (t[2] - (anchor + delta)).length() < 1e-4,
                    "P3 not anchor+delta, dt {dt}"
                );
            }
        }
    }

    #[test]
    fn pd_target_is_pose_when_gains_are_zero() {
        let pose = Vec2::new(10.0, -5.0);
        let t = pd_target(
            pose,
            Vec2::new(0.0, 0.0),
            Vec2::ZERO,
            Vec2::ZERO,
            1.0 / 90.0,
            0.0,
            0.0,
        );
        assert!((t - pose).length() < 1e-6);
    }

    #[test]
    fn pd_target_leads_toward_pose_and_commanded_velocity() {
        let dt = 1.0 / 90.0;
        let pose = Vec2::new(100.0, 0.0);
        let pos = Vec2::new(90.0, 0.0); // pose ahead of pos in +x
                                        // P term alone pushes the target past `pose` (in +x).
        let tp = pd_target(pose, pos, Vec2::ZERO, Vec2::ZERO, dt, 100.0, 0.0);
        assert!(tp.x > pose.x, "P term should lead ahead of pose: {tp:?}");
        // D term: particle at rest, hip moving +x ⇒ target pushed +x.
        let td = pd_target(pose, pos, Vec2::ZERO, Vec2::new(200.0, 0.0), dt, 0.0, 10.0);
        assert!(
            td.x > pose.x,
            "D term should lead in the hip direction: {td:?}"
        );
        // D term with the particle already matching the hip ⇒ no shift.
        let tm = pd_target(
            pose,
            pos,
            Vec2::new(200.0, 0.0),
            Vec2::new(200.0, 0.0),
            dt,
            0.0,
            10.0,
        );
        assert!(
            (tm - pose).length() < 1e-6,
            "matched velocity ⇒ no shift: {tm:?}"
        );
    }

    #[test]
    fn compliance_softens_monotonically_with_speed_and_clamps() {
        let base = 1e-7;
        let slow = softened_compliance(base, 10.0, 300.0, 0.0);
        let mid = softened_compliance(base, 10.0, 300.0, 150.0);
        let fast = softened_compliance(base, 10.0, 300.0, 300.0);
        let satur = softened_compliance(base, 10.0, 300.0, 9999.0);
        assert!((slow - base).abs() < 1e-12, "at rest ⇒ base");
        assert!(
            mid > slow && fast > mid,
            "softer as speed grows: {slow} {mid} {fast}"
        );
        assert!(
            (fast - base * 10.0).abs() < 1e-12,
            "saturated ⇒ base·soft_scale"
        );
        assert!((satur - fast).abs() < 1e-12, "clamped at v_ref");
        // soft_scale = 1 and v_ref = 0 are both no-ops.
        assert!((softened_compliance(base, 1.0, 300.0, 9999.0) - base).abs() < 1e-12);
        assert!((softened_compliance(base, 10.0, 0.0, 9999.0) - base).abs() < 1e-12);
    }
}
