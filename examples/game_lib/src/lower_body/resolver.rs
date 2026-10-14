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

use bevy::math::Vec2;

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

/// Runtime-tuneable knobs for the lower-body resolver.
///
/// All fields are editable through the sandbox egui window. The leg-length
/// defaults are seeded from the v10 blueprint (`thigh`/`calf` ≈ 55 px after the
/// character root's `0.5` world scale in the tuning example), but the math is
/// fully parametric and does not depend on that blueprint.
#[derive(Debug, Clone)]
pub struct LowerBodyConfig {
    /// Thigh (hip→knee) length, px.
    pub thigh_len: f32,
    /// Calf (knee→foot) length, px.
    pub calf_len: f32,
    /// Standing hip height above the floor, px.
    pub hip_height: f32,
    /// Crouched hip height above the floor, px (drives `X_max` down).
    pub crouch_height: f32,
    /// Target cruise speed magnitude, px/s (`v_desired`).
    pub target_speed: f32,
    /// Horizontal acceleration toward `v_desired`, px/s².
    pub accel: f32,
    /// Stance duration `Ts` (seconds): how long a foot stays pinned. The default
    /// equals the swing duration, i.e. a **duty factor β = 0.5**: the airborne
    /// half of a running/jumping cycle lasts exactly as long as the grounded half.
    pub stance_duration_ts: f32,
    /// Swing duration (seconds): how long a foot is in the air. Equal to
    /// `stance_duration_ts` by default (β = 0.5).
    pub swing_duration_ts: f32,
    /// Raibert proportional velocity-feedback gain (units: **seconds**). See
    /// the derivation in `plans/Raibert.md`.
    pub kv: f32,
    /// Backward-solve blend gain (0..1]. `1.0` = deadbeat: at touchdown the hip
    /// velocity fully adopts the support velocity inverted from the actual
    /// plant offset. Lower values damp the correction, which is what the XPBD
    /// integration will need once feet lag their kinematic targets.
    pub beta: f32,
    /// Fraction of the geometric leg reach `X_max` that is actually usable
    /// (0..1). Lower = more conservative steps.
    pub safety_margin: f32,
    /// Fraction of `X_max` (0..1) reserved for prediction error. The planner
    /// keeps `½·a·T_swing² ≤ prediction_margin·X_max`, which bounds the hip
    /// acceleration at `a_max` (see [`GaitLimits`]).
    pub prediction_margin: f32,
    /// Swing arc apex height above the plant/floor line, px.
    pub step_height: f32,
    /// World y of the flat floor line.
    pub floor_y: f32,
    /// Knee bend direction: +1 = knee in front of the hip→foot line, -1 = behind.
    pub bend_sign: f32,
    /// `true` while crouching (squash height).
    pub crouch: bool,

    // --- Vertical (loop 1) knobs. See `plans/raibert_vertical_loop.md`. ---
    /// Effective gravity, px/s². The ECS layer overwrites this from the live
    /// world each tick; the default assumes the `100 px = 1 m` mapping (980).
    pub gravity: f32,
    /// Target jump apex **rise** above the standing hip height, px. This is the
    /// `h_desired` of the per-hop displacement law (doc §4).
    pub jump_apex: f32,
    /// Apex-error proportional gain for the per-hop displacement law
    /// `Δl = Δl_prev + k_p·(h_desired − h_apex)`. Dimensionless.
    pub k_p_apex: f32,
    /// Upward acceleration the virtual leg pin can command while grounded,
    /// px/s². This is a **force authority**, not a stiffness: the loop still
    /// closes on measured velocity, never on this number (doc §2.2, §4.1).
    pub launch_accel: f32,
}

impl Default for LowerBodyConfig {
    fn default() -> Self {
        Self {
            thigh_len: 55.0,
            calf_len: 55.0,
            hip_height: 90.0,
            crouch_height: 55.0,
            target_speed: 120.0,
            accel: 800.0,
            stance_duration_ts: 0.28,
            // β = 0.5: airborne half-cycle == grounded half-cycle.
            swing_duration_ts: 0.28,
            kv: 0.12,
            beta: 1.0,
            safety_margin: 0.85,
            prediction_margin: 0.5,
            step_height: 28.0,
            floor_y: 0.0,
            bend_sign: 1.0,
            crouch: false,
            gravity: 980.0,
            jump_apex: 60.0,
            k_p_apex: 0.5,
            launch_accel: 2200.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Gait phase
// ---------------------------------------------------------------------------

/// Discrete phase of a single leg.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaitPhase {
    /// Foot is pinned to the floor.
    Stance,
    /// Foot is travelling through the air toward its next plant.
    Swing,
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Per-leg persistent state.
///
/// `foot_pos` is the current (output) foot position; `plant_pos` is the
/// anchored world position the foot holds while in stance (so a moving hip
/// leaves the stance foot pinned, stretching the leg through IK).
#[derive(Debug, Clone, Copy)]
pub struct LegState {
    pub phase: GaitPhase,
    /// Seconds remaining in the current phase.
    pub phase_timer: f32,
    /// Current foot world position (output cache).
    pub foot_pos: Vec2,
    /// Anchored foot world position used while in stance.
    pub plant_pos: Vec2,
    /// Swing lift-off position (world). Holds the foot's world position at
    /// lift-off for a normal ground swing.
    pub swing_from: Vec2,
    /// Swing plant target.
    pub swing_to: Vec2,
}

/// Discrete phase of the vertical (loop 1) controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerticalPhase {
    /// Regulating to the standing height.
    Stand,
    /// Banking displacement: crouching by the commanded `Δl`.
    Load,
    /// Spending displacement: thrusting up until the measured h-dot cutoff.
    Launch,
    /// Ballistic, unactuated.
    Flight,
    /// Absorbing the landing back to the standing height.
    Land,
}

/// Loop-1 (vertical) persistent state. See `plans/raibert_vertical_loop.md`.
///
/// The whole algorithm is one line — the displacement law
/// `Δl = Δl_prev + k_p·(h_desired − h_apex)` — plus the phase machine that
/// decides *when* to bank `Δl` (Load) and when to spend it (Launch). On a
/// one-time jump `Δl_prev` starts at zero, so the first Load depth is simply
/// `k_p·h_desired`; thereafter the same law runs every hop.
#[derive(Debug, Clone, Copy)]
pub struct VerticalState {
    pub phase: VerticalPhase,
    /// `Δl`: extra crouch displacement commanded for the current hop.
    pub delta_l: f32,
    /// Running max rise above stand this hop.
    pub h_apex: f32,
    /// Latched apex rise of the last completed hop (the *measurement*).
    pub h_apex_prev: f32,
    /// True once the current hop's apex has been latched (edge, not level).
    pub apex_latched: bool,
    /// Buffered jump intent, consumed at a `Stand → Load` edge.
    pub jump_requested: bool,
    /// Time spent in `Land`, used for the settle exit.
    pub land_timer: f32,
    /// Vertical-lift amount of the airborne foot swing built at the `Launch →
    /// Flight` edge: `FLIGHT_SWING_LIFT_FRACTION · step_height` (or a reduced
    /// value if the hop is too shallow to clear it). Frozen for the whole hop.
    pub swing_lift: f32,
    /// Total airborne (swing) half-cycle time for the current hop, `2·v0/g`,
    /// latched at the `Launch → Flight` edge from the launch speed. This is the
    /// time base for the flight foot arc (it equals the observed flight time,
    /// the descent ending at the same height it launched from).
    pub swing_period: f32,
    /// Elapsed time in the airborne swing, advanced every `Flight` tick. The
    /// flight arc maps `swing_t / swing_period → [0, 1]`.
    pub swing_t: f32,
    /// Left / right foot **hip-relative offset** latched at the `Launch → Flight`
    /// edge (clamped into the IK bend zone). The flight swing sweeps each leg from
    /// its lift-off offset to the shared landing offset (see §6b in
    /// [`step_lower_body`]). Held here — not in [`LegState::swing_from`] — because
    /// `step_leg` overwrites `swing_from` with a world position if a stance leg
    /// lifts off mid-flight; these fields are touched only by the flight swing.
    pub swing_from_off_left: Vec2,
    pub swing_from_off_right: Vec2,
}

impl Default for VerticalState {
    fn default() -> Self {
        Self {
            phase: VerticalPhase::Stand,
            delta_l: 0.0,
            h_apex: 0.0,
            h_apex_prev: 0.0,
            apex_latched: false,
            jump_requested: false,
            land_timer: 0.0,
            swing_lift: 0.0,
            swing_period: 0.0,
            swing_t: 0.0,
            swing_from_off_left: Vec2::ZERO,
            swing_from_off_right: Vec2::ZERO,
        }
    }
}

/// The resolver's full persistent state.
#[derive(Debug, Clone)]
pub struct ResolverState {
    pub hip_pos: Vec2,
    pub hip_vel: Vec2,
    /// Current (possibly interpolated) hip height `H`.
    pub hip_height: f32,
    pub left: LegState,
    pub right: LegState,
    /// Loop-1 (vertical) controller state.
    pub vs: VerticalState,
    /// True on the most recent tick if the body was airborne (`Flight`). Used to
    /// detect the `Launch→Flight` (arm) and `Flight→Land` (seed) edges of the
    /// jump's airborne swing.
    pub airborne_prev: bool,
    /// `true` when the jump was armed from a *walking* lift-off. The airborne
    /// swing then reaches to the ground-reference forward target (a running
    /// stride, one step ahead); on a stand-still jump it is `false` and the
    /// swing lands on top of the lift-off footprint instead (no forward step).
    pub jump_arm_lead_leg: bool,
}

impl ResolverState {
    /// Build a resting, symmetric stance: hip centred, both feet planted either
    /// side of the hip at a reachable offset.
    pub fn new(config: &LowerBodyConfig) -> Self {
        let hip = Vec2::new(0.0, config.floor_y + config.hip_height);
        let spread = 30.0_f32
            .min(config.thigh_len + config.calf_len - 5.0)
            .max(5.0);
        let left_plant = Vec2::new(hip.x - spread, config.floor_y);
        let right_plant = Vec2::new(hip.x + spread, config.floor_y);

        let left = LegState {
            phase: GaitPhase::Stance,
            phase_timer: config.stance_duration_ts,
            foot_pos: left_plant,
            plant_pos: left_plant,
            swing_from: left_plant,
            swing_to: left_plant,
        };
        // Right leg starts half a stance out of phase with the left, so the two
        // legs alternate once the gait begins; both stay planted while standing
        // still (no idle stance→swing hop).
        let right = LegState {
            phase: GaitPhase::Stance,
            phase_timer: config.stance_duration_ts * 0.5,
            foot_pos: right_plant,
            plant_pos: right_plant,
            swing_from: right_plant,
            swing_to: right_plant,
        };

        Self {
            hip_pos: hip,
            hip_vel: Vec2::ZERO,
            hip_height: config.hip_height,
            left,
            right,
            vs: VerticalState::default(),
            airborne_prev: false,
            jump_arm_lead_leg: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

/// The resolved particle positions: 1 hip + 2 knees + 2 feet.
#[derive(Debug, Clone, Copy)]
pub struct LowerBodyPose {
    pub hip: Vec2,
    pub left_knee: Vec2,
    pub left_foot: Vec2,
    pub right_knee: Vec2,
    pub right_foot: Vec2,
}

/// Telemetry for the UI / debugging; the forward/backward math intermediates.
#[derive(Debug, Clone, Copy)]
pub struct DebugInfo {
    /// Geometric max reach at the current hip height (`X_max`).
    pub x_max: f32,
    /// Velocity clamp ceiling actually applied this tick (`v_ceil`).
    pub v_max: f32,
    /// Desired horizontal speed (`v_desired`), signed, after the plan cap.
    pub v_desired: f32,
    /// Clamped horizontal velocity actually applied this tick.
    pub v_clamped: f32,
    pub left_phase: GaitPhase,
    pub right_phase: GaitPhase,
    /// Forward step target x for each leg (world x), the last time each swung.
    pub left_target_x: f32,
    pub right_target_x: f32,
    /// Max feasible hip acceleration from the prediction-error budget.
    pub a_max: f32,
    /// Absolute speed ceiling from the feasibility envelope.
    pub v_abs: f32,
    /// Backward-solved support velocity at the latest touchdown this tick
    /// (0 when no touchdown event fired).
    pub v_support: f32,
    /// Signed capture-point offset from the hip while braking to a stop
    /// (0 when not stopping).
    pub capture_x: f32,

    // --- Loop 1 (vertical) telemetry. See `plans/raibert_vertical_loop.md`. ---
    /// Current vertical-controller phase.
    pub v_phase: VerticalPhase,
    /// Measured hip height above the floor, px.
    pub hip_height: f32,
    /// Measured vertical hip velocity `ḣ`, px/s.
    pub hip_vel_y: f32,
    /// Target apex rise `h_desired`, px.
    pub h_desired: f32,
    /// Latched apex rise of the last completed hop (the measurement), px.
    pub h_apex: f32,
    /// Current commanded extra crouch displacement `Δl`, px.
    pub delta_l: f32,
}

// ---------------------------------------------------------------------------
// Math primitives
// ---------------------------------------------------------------------------

/// Move `current` toward `target` by at most `max_delta`, without overshooting.
#[inline]
pub fn approach(current: f32, target: f32, max_delta: f32) -> f32 {
    let d = target - current;
    if d.abs() <= max_delta {
        target
    } else {
        current + d.signum() * max_delta
    }
}

/// Rotate a vector by `angle` radians (CCW in a y-up frame).
#[inline]
pub fn rotate(v: Vec2, angle: f32) -> Vec2 {
    let (sin, cos) = angle.sin_cos();
    Vec2::new(v.x * cos - v.y * sin, v.x * sin + v.y * cos)
}

// ---------------------------------------------------------------------------
// Loop 1: the vertical displacement controller (pure math)
// ---------------------------------------------------------------------------

/// Forward: ballistic apex **rise** for a measured launch speed.
/// `Δh = ḣ₀² / (2·g̃)` — kinematics only; subtracts out the standing height.
#[inline]
pub fn predict_apex_rise(h_dot0: f32, gravity: f32) -> f32 {
    if gravity <= 0.0 {
        0.0
    } else {
        h_dot0 * h_dot0 / (2.0 * gravity)
    }
}

/// Backward: launch speed needed to reach an apex rise `Δh*`.
/// `ḣ₀* = sqrt(2·g̃·Δh*)` — no mass and no stiffness, by construction.
#[inline]
pub fn apex_to_launch_speed(apex_rise: f32, gravity: f32) -> f32 {
    (2.0 * gravity * apex_rise.max(0.0)).sqrt()
}

/// Landing speed magnitude after falling `fall` px (energy conservation).
/// Retained for the absorb/landing path (`Land`) and for symmetry with the
/// launch helpers; consumed today by the loop-1 tests.
#[inline]
#[allow(dead_code)]
pub fn landing_speed(fall: f32, gravity: f32) -> f32 {
    (2.0 * gravity * fall.max(0.0)).sqrt()
}

/// The per-hop displacement (stroke) law — the whole of loop 1:
///
/// ```text
/// Δl = Δl_prev + k_p · ( h_desired − h_apex )
/// ```
///
/// `h_desired` and `h_apex` are **apex rises** above the standing height, so
/// their difference is itself a displacement and `k_p` is dimensionless. The
/// `Δl_prev` term is an integrator: it learns the baseline energy needed to
/// fight gravity, which is why no mass or stiffness ever enters the loop.
#[inline]
pub fn update_delta_l(delta_l_prev: f32, h_desired: f32, h_apex: f32, k_p: f32) -> f32 {
    delta_l_prev + k_p * (h_desired - h_apex)
}

/// Grounded height-regulator spring (1/s²) and damping (1/s). Critically
/// damped (`k_d = 2·√k_s`) so a height step settles without ringing.
const VERTICAL_K_S: f32 = 400.0;
const VERTICAL_K_D: f32 = 40.0;

/// Drive the grounded hip toward `y_target` under a critically-damped spring,
/// with ground support cancelling gravity — the "held" pin of
/// `plans/gravity_four_layer.md`. The vertical loop closes on the *measured*
/// resulting state, never on this request.
fn regulate_height(state: &mut ResolverState, y_target: f32, dt: f32) {
    let dy = y_target - state.hip_pos.y;
    let a = VERTICAL_K_S * dy - VERTICAL_K_D * state.hip_vel.y;
    state.hip_vel.y += a * dt;
    state.hip_pos.y += state.hip_vel.y * dt;
}

/// One tick of the loop-1 (vertical) controller.
///
/// State-based triggers only: crouch depth reached, measured h-dot cutoff,
/// full extension, contact loss, settle band. No timers gate the launch.
fn tick_vertical(state: &mut ResolverState, config: &LowerBodyConfig, dt: f32) {
    let g = config.gravity.max(1.0);
    let floor = config.floor_y;
    let stand = if config.crouch {
        config.crouch_height
    } else {
        config.hip_height
    };
    let h = state.hip_pos.y - floor;
    let grounded = state.left.phase == GaitPhase::Stance || state.right.phase == GaitPhase::Stance;
    let full_ext = config.thigh_len + config.calf_len - 2.0;

    match state.vs.phase {
        VerticalPhase::Stand => {
            regulate_height(state, floor + stand, dt);
            if state.vs.jump_requested && grounded {
                state.vs.jump_requested = false;
                // Apply the displacement law for this hop, using the last hop's
                // measured apex (`0` on the very first jump → `Δl = k_p·h_desired`).
                state.vs.delta_l = update_delta_l(
                    state.vs.delta_l,
                    config.jump_apex,
                    state.vs.h_apex_prev,
                    config.k_p_apex,
                );
                state.vs.phase = VerticalPhase::Load;
                state.vs.h_apex = 0.0;
                state.vs.apex_latched = false;
                // Reset the airborne-swing bookkeeping for the new hop and arm
                // the flight reach: a lift-off with forward speed steps forward
                // (a running stride), a stand-still jump lands back on its own
                // footprint instead.
                state.vs.swing_lift = 0.0;
                state.vs.swing_period = 0.0;
                state.vs.swing_t = 0.0;
                state.vs.swing_from_off_left = Vec2::ZERO;
                state.vs.swing_from_off_right = Vec2::ZERO;
                state.jump_arm_lead_leg = state.hip_vel.x.abs() > SWING_TRIGGER_SPEED;
            }
        }
        VerticalPhase::Load => {
            // Launch window (doc §5.7): a hop is committed from a stance. If
            // contact is lost before the crouch completes, abort to Stand and
            // wait for the next stance window rather than crouching mid-air.
            if !grounded {
                state.vs.phase = VerticalPhase::Stand;
            } else {
                // Bank displacement: crouch by `Δl`, never below the crouch stop.
                let target = (stand - state.vs.delta_l).max(config.crouch_height.min(stand));
                regulate_height(state, floor + target, dt);
                state.vs.delta_l = (stand - target).max(0.0);
                if h <= target + 1.0 {
                    state.vs.phase = VerticalPhase::Launch;
                }
            }
        }
        VerticalPhase::Launch => {
            // Spend displacement: thrust up. Release on the **measured** h-dot
            // (doc §4.1) once it is just enough to coast from the CURRENT height
            // to the target apex `floor + stand + jump_apex` — i.e. the cutoff
            // is **altitude-aware**, so any height already gained during Launch
            // is credited. That makes the measured apex land on `stand +
            // h_desired` regardless of how deep the crouch was, which is what
            // lets the per-hop `Δl` law close its loop (a fixed cutoff would
            // overshoot and feed the integrator the wrong sign). The fallback
            // ends Launch on full extension with h-dot still short (a shallow
            // crouch saturating the leg).
            state.hip_vel.y += (config.launch_accel - g) * dt;
            state.hip_pos.y += state.hip_vel.y * dt;
            let y_goal = floor + stand + config.jump_apex;
            let v_req = apex_to_launch_speed(y_goal - state.hip_pos.y, g);
            if state.hip_vel.y >= v_req || (state.hip_pos.y - floor) >= full_ext {
                state.vs.phase = VerticalPhase::Flight;
                // Size the airborne swing lift once, from the hop's ballistic
                // rise `v²/(2g)`. A shallow hop cannot clear the full tuck, so
                // the lift is capped to `⅔` of the rise; a normal hop clears
                // `FLIGHT_SWING_LIFT_FRACTION · step_height` outright.
                let v0 = state.hip_vel.y.max(0.0);
                let rise = v0 * v0 / (2.0 * g);
                state.vs.swing_lift =
                    (FLIGHT_SWING_LIFT_FRACTION * config.step_height).min((2.0 / 3.0) * rise);
                state.vs.swing_period = (2.0 * v0 / g).max(f32::EPSILON);
                state.vs.swing_t = 0.0;
            }
        }
        VerticalPhase::Flight => {
            // Ballistic, unactuated — the anti-hover guarantee.
            state.hip_vel.y -= g * dt;
            state.hip_pos.y += state.hip_vel.y * dt;
            // Advance the airborne-swing clock (drives the flight foot arc).
            state.vs.swing_t += dt;
            let rise = (state.hip_pos.y - floor) - stand;
            if rise > state.vs.h_apex {
                state.vs.h_apex = rise;
            }
            // Latch the apex at the sign change of vertical velocity. This is
            // the boundary measurement the per-hop law consumes.
            if state.hip_vel.y <= 0.0 && !state.vs.apex_latched {
                state.vs.h_apex_prev = state.vs.h_apex;
                state.vs.apex_latched = true;
            }
            // The hop ends only when the hip is truly back down on the planted
            // floor line (`hip_pos.y ≤ floor + stand`), so the airborne foot
            // swing has a full landing half-cycle to reach ahead: either the
            // feet catch the body (the normal, cyclic running landing) or a
            // slightly slow flight hands a small residual to `Land`'s settle.
            // Exit on the hip, never on a pose-triggered foot lift.
            if state.hip_vel.y < 0.0 && state.hip_pos.y <= floor + stand {
                state.vs.phase = VerticalPhase::Land;
                state.vs.land_timer = 0.0;
            }
        }
        VerticalPhase::Land => {
            regulate_height(state, floor + stand, dt);
            state.vs.land_timer += dt;
            let settled = (h - stand).abs() < 4.0 && state.hip_vel.y.abs() < 20.0;
            if state.vs.land_timer > 0.1 && settled {
                state.vs.phase = VerticalPhase::Stand;
            }
        }
    }
}

/// The backward velocity clamp. Isolates the maximum horizontal speed the hip
/// may have at the current height without outrunning its own feet.
///
/// `X_max` is the (safety-margined) geometric reach at height `H`:
/// `sqrt((L1 + L2)^2 - H^2) * margin`.
#[inline]
pub fn compute_x_max(thigh: f32, calf: f32, hip_height: f32, margin: f32) -> f32 {
    let full = thigh + calf;
    let inner = full * full - hip_height * hip_height;
    if inner <= 0.0 {
        0.0
    } else {
        inner.sqrt() * margin
    }
}

/// Cruise ceiling from the two-sided stance-reach constraint.
///
/// A pinned stance foot's offset from the hip sweeps monotonically from
/// `+x_target` at touchdown to `−(v·Ts − x_target)` at lift-off, and the swing
/// foot's offset interpolates between the same bounds, so the reach constraint
/// is `|foot − hip| ≤ X_max` at both ends of the sweep:
///
/// `v·Ts/2 + kv·|v − v_desired| ≤ X_max`.
///
/// Solving for `v` on each side of `v_desired` gives two ceilings; the binding
/// one is their minimum:
/// - forward side (v ≥ v_d): `v ≤ (X_max + kv·v_d) / (Ts/2 + kv)`
/// - backward side (v ≤ v_d): `v ≤ (X_max − kv·v_d) / (Ts/2 − kv)`
///
/// Note `T_swing` does **not** enter: the foot−hip offset is bounded by the
/// same extremes during swing as during stance. The original "floating foot"
/// bug came from an uncapped, infeasible `v_desired` (e.g. 600 px/s, where
/// `v·Ts/2 = 168 > X_max`) violating the end-of-stance side — fixed by capping
/// the desired speed itself (see [`GaitLimits::v_plan_cap`]).
///
/// `v_desired_mag` is the magnitude of the desired cruise speed (non-negative);
/// the result is a symmetric positive speed ceiling for both directions.
#[inline]
pub fn compute_v_max(x_max: f32, kv: f32, stance_ts: f32, v_desired_mag: f32) -> f32 {
    let half = stance_ts * 0.5;
    let fwd_denom = half + kv;
    if fwd_denom <= f32::EPSILON {
        return 0.0;
    }
    let fwd = (x_max + kv * v_desired_mag) / fwd_denom;
    let back_denom = half - kv;
    let back = if back_denom > f32::EPSILON {
        ((x_max - kv * v_desired_mag) / back_denom).max(0.0)
    } else if x_max >= kv * v_desired_mag {
        // kv feedback never pushes past reach on the slow side.
        f32::INFINITY
    } else {
        0.0
    };
    fwd.min(back)
}

/// The generalized backward solve (foot → hip). Given the signed offset
/// `delta_x` from the hip **at touchdown** to the support foot, returns the hip
/// velocity whose forward plan would land exactly on that offset:
///
/// `v_support = (delta_x + kv * v_desired) / (Ts/2 + kv)`.
///
/// [`compute_v_max`] (old form) is this evaluated at `delta_x = X_max`.
/// Composing backward∘forward is the identity: a plant error `e` maps to a
/// velocity correction `e / (Ts/2 + kv)` that the next stride annihilates
/// (deadbeat, one-stride convergence). `v_desired` is *signed*.
#[inline]
pub fn compute_support_velocity(delta_x: f32, kv: f32, stance_ts: f32, v_desired: f32) -> f32 {
    let denom = stance_ts * 0.5 + kv;
    if denom <= f32::EPSILON {
        return 0.0;
    }
    (delta_x + kv * v_desired) / denom
}

/// Precomputed feasibility envelope for the gait at a given hip height.
///
/// All four limits are closed-form in the config, so the sandbox can clamp the
/// planner *before* it produces unreachable targets instead of reacting to IK
/// clamping after the fact. Derivations live in `plans/raibert_redesign.md`.
#[derive(Debug, Clone, Copy)]
pub struct GaitLimits {
    /// Safety-margined geometric reach: `sqrt((L1+L2)² − H²) · margin`.
    pub x_max: f32,
    /// Max hip acceleration whose swing-phase prediction error `½·a·T_swing²`
    /// still fits inside `prediction_margin · x_max`.
    pub a_max: f32,
    /// Absolute speed ceiling: at this speed the prediction error budget alone
    /// saturates the reach, and the stop decel floor `v²/(2·X_max)` equals
    /// `a_max` exactly.
    pub v_abs: f32,
    /// Cruise ceiling from the two-sided stance-reach constraint
    /// `v·Ts/2 + kv·|v − v_d| ≤ X_max` (see [`compute_v_max`]).
    pub v_cruise_max: f32,
    /// Cap on the *desired* speed fed to the planner: the self-consistent
    /// cruise ceiling `2·X_max / Ts` where the kv feedback term vanishes
    /// (`v == v_d` ⇒ offset peaks at `v·Ts/2`). Requesting more would leave a
    /// persistent kv bias `kv·(v − v_d)` that pushes late-stance feet out of
    /// reach.
    pub v_plan_cap: f32,
}

impl GaitLimits {
    /// Evaluate the envelope for `config` at hip height `hip_height`, with
    /// desired cruise speed magnitude `v_desired_mag`.
    pub fn from_config(config: &LowerBodyConfig, hip_height: f32, v_desired_mag: f32) -> Self {
        let x_max = compute_x_max(
            config.thigh_len,
            config.calf_len,
            hip_height,
            config.safety_margin,
        );
        let t_swing = config.swing_duration_ts.max(f32::EPSILON);
        let m_e = config.prediction_margin.clamp(0.05, 1.0);
        // ½·a·T_swing² ≤ m_e·X_max  ⇒  a_max = 2·m_e·X_max / T_swing²
        let a_max = 2.0 * m_e * x_max / (t_swing * t_swing);
        // v_abs: speed at which the stop floor v²/(2·X_max) equals a_max.
        let v_abs = 2.0 * x_max * m_e.sqrt() / t_swing;
        let t_stance = config.stance_duration_ts.max(f32::EPSILON);
        // v == v_d ⇒ offset peaks at v·Ts/2 ≤ X_max  ⇒  v ≤ 2·X_max/Ts.
        let v_plan_cap = 2.0 * x_max / t_stance;
        let v_d_eff = v_desired_mag.min(v_plan_cap);
        let v_cruise_max = compute_v_max(x_max, config.kv, config.stance_duration_ts, v_d_eff);
        Self {
            x_max,
            a_max,
            v_abs,
            v_cruise_max,
            v_plan_cap,
        }
    }
}

/// The forward step planner: horizontal distance from the (moving) hip to the
/// next foot plant.
///
/// `x_target = (v * Ts) / 2 + kv * (v - v_desired)`.
///
/// `v` is the clamped horizontal hip velocity, `v_desired` the *signed* desired
/// cruise velocity (positive = moving +x).
#[inline]
pub fn compute_forward_target(v: f32, v_desired: f32, kv: f32, stance_ts: f32) -> f32 {
    (v * stance_ts) * 0.5 + kv * (v - v_desired)
}

/// Law-of-cosines 2-bone IK: returns `(knee, foot)` for the leg reaching from
/// `hip` toward `foot_target` given `thigh_len` and `calf_len`.
///
/// `bend_sign` selects which side of the hip→foot line the knee sits on (+1 or
/// -1).
///
/// The returned `foot` is the **effective** foot position: when `foot_target`
/// is out of reach it is clamped to the nearest reachable point along the
/// hip→target ray, so `|knee - hip| == thigh_len` and `|foot - knee| ==
/// calf_len` always hold exactly. Callers must render/use this effective foot,
/// otherwise the leg bones visually stretch. If the hip and target coincide, a
/// horizontal default direction is used.
pub fn solve_leg_ik(
    hip: Vec2,
    foot_target: Vec2,
    thigh_len: f32,
    calf_len: f32,
    bend_sign: f32,
) -> (Vec2, Vec2) {
    let to = foot_target - hip;
    let dist = to.length();
    let dmax = thigh_len + calf_len;
    let dmin = (thigh_len - calf_len).abs();

    // Clamp the reach into [dmin + eps, dmax - eps] so the acos argument is
    // always well-conditioned.
    let eps = 1e-3;
    let lo = (dmin + eps).max(eps);
    let hi = (dmax - eps).max(lo + eps);
    let d = dist.clamp(lo, hi);

    let dir = if dist > f32::EPSILON {
        to / dist
    } else {
        Vec2::X
    };

    let cos_a = ((thigh_len * thigh_len + d * d - calf_len * calf_len) / (2.0 * thigh_len * d))
        .clamp(-1.0, 1.0);
    let a = cos_a.acos();

    let thigh_dir = rotate(dir, a * bend_sign);
    let knee = hip + thigh_dir * thigh_len;
    // The effective foot is the clamped point along the hip→target ray; when
    // the target is reachable this is the target itself.
    let foot = hip + dir * d;
    (knee, foot)
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

/// Minimum `|v_desired|` (move intent, px/s) required to initiate a step. With
/// no intent the gait freezes: both feet stay planted and the stop-settle servo
/// in [`step_lower_body`] glides the hip to the support centre. Gating on
/// *intent* rather than actual velocity also prevents the settle motion itself
/// from triggering phantom steps that would walk the character off with no
/// input held.
const SWING_TRIGGER_SPEED: f32 = 1.0;

/// Maximum fraction of the full leg reach `thigh + calf` that the airborne swing
/// may use for the hip→foot offset. Keeps the knee off the 2-bone IK's
/// `acos(reach/(thigh+calf))` singularity: with `thigh == calf` a reach of
/// `0.96·(L1+L2)` holds the knee half-angle at `α = acos(0.96) ≈ 16°`, so a fast
/// hip during Flight can never straighten (and thereby snap) the knee.
const LIFT_REACH_SAFE_FRACTION: f32 = 0.96;

/// Horizontal reach the lead foot may extend ahead of the hip at the landing
/// plant, as a fraction of the ground-reference gait reach `X_max`. The jump's
/// airborne (swing) half drives both feet forward from behind the hip to ahead
/// of it; this caps how far ahead so the landing pose not only sits inside the
/// IK reach but also leaves the leg *visibly bent* — with `thigh == calf` the
/// knee half-angle is `α = acos(reach / (thigh + calf))`, so keeping the
/// standing-height reach at `0.5·X_max` (≈ 35 px ≪ 110 px) holds `α` far from
/// the IK's `acos(reach/110)` singularity and no knee can ever snap straight.
const FLIGHT_SWING_REACH_FRACTION: f32 = 0.5;

/// How far the airborne swing lifts the foot off the planted floor line, as a
/// fraction of the walking `step_height` — the vertical amplitude of the jump's
/// full-circle foot arc. Kept below 1 so the airborne swing reads as a running
/// stride rather than a knee-to-chest tuck.
const FLIGHT_SWING_LIFT_FRACTION: f32 = 0.8;

/// Per-tick planning context shared by both legs.
#[derive(Debug, Clone, Copy)]
struct StepPlan {
    /// Safety-margined reach; swing targets are clamped to `hip ± x_max` so a
    /// plant can never be aimed outside the workspace (which the IK would
    /// ray-clamp into a foot floating off the floor).
    x_max: f32,
    /// True while braking to a stop: no move intent but the hip is still
    /// travelling faster than the swing trigger.
    stopping: bool,
    /// Capture-point offset from the *current* hip while `stopping`:
    /// `v·|v| / (2·a_eff)`. Aimed at the point where the hip will come to rest
    /// under the effective deceleration, so the hip brakes *onto* its foot.
    capture_offset: f32,
}

/// World-x swing target for a foot, given `t_flight` seconds remaining in the
/// air. Used both at lift-off (`t_flight = T_swing`) and by the per-frame
/// re-planning during swing.
///
/// While stopping, targets the capture point instead of the forward plan.
/// The result is always clamped to `hip ± x_max`.
fn plan_target_x(
    hip_x: f32,
    v: f32,
    v_desired: f32,
    t_flight: f32,
    config: &LowerBodyConfig,
    plan: &StepPlan,
) -> f32 {
    let raw = if plan.stopping {
        hip_x + plan.capture_offset
    } else {
        let x_target = compute_forward_target(v, v_desired, config.kv, config.stance_duration_ts);
        // Raibert's `x_target` is the offset from the hip **at touchdown**, not
        // at lift-off: the hip keeps travelling while the foot is airborne, so
        // predict that motion. Re-planning every frame with the shrinking
        // `t_flight` keeps the world-space target stationary at constant
        // velocity (hip advance `v·dt` cancels the prediction shrink) and makes
        // the target track acceleration/braking transients mid-swing.
        hip_x + v * t_flight + x_target
    };
    raw.clamp(hip_x - plan.x_max, hip_x + plan.x_max)
}

/// Advance a single leg through the stance → swing → plant state machine.
///
/// Updates `leg` in place. `v_desired` is the *signed* desired cruise velocity
/// (from the current input, not inferred from `v`), so at standstill it is zero
/// and the foot plants straight under the hip rather than behind it.
///
/// `other_swinging` is true while the opposite leg is airborne. A stance leg may
/// only lift off when the opposite foot is planted, which enforces a strict
/// anti-phase alternating gait: at most one foot is airborne at any instant, so
/// the two legs can never swing into each other.
///
/// Returns `Some(delta_x)` on the tick the foot touches down, where `delta_x`
/// is the signed offset from the hip to the new support foot — the event that
/// drives the backward solve in [`step_lower_body`].
#[allow(clippy::too_many_arguments)]
fn step_leg(
    leg: &mut LegState,
    hip_x: f32,
    v: f32,
    v_desired: f32,
    other_swinging: bool,
    config: &LowerBodyConfig,
    plan: &StepPlan,
    dt: f32,
) -> Option<f32> {
    match leg.phase {
        GaitPhase::Stance => {
            // The stance foot is pinned in world space; the hip moves away and
            // the leg stretches through the IK solve.
            leg.foot_pos = leg.plant_pos;
            leg.phase_timer -= dt;
            // Step while there is move *intent*, or while braking to a stop
            // (the capture-point plan needs a foot to land on). With neither,
            // the gait freezes and the hip settles over the planted feet (see
            // the stop-settle servo in `step_lower_body`). Steps also require
            // the opposite foot to be planted (anti-phase swap).
            let wants_step = v_desired.abs() > SWING_TRIGGER_SPEED
                || (plan.stopping && v.abs() > SWING_TRIGGER_SPEED);
            if leg.phase_timer <= 0.0 && wants_step && !other_swinging {
                let mut swing_to = leg.plant_pos;
                // The jump never retargets a step: every swing targets its own
                // forward Raibert landing point, exactly as in a normal walk, so a
                // foot always leads ahead of the hip for the next step.
                swing_to.x =
                    plan_target_x(hip_x, v, v_desired, config.swing_duration_ts, config, plan);
                swing_to.y = config.floor_y;
                leg.phase = GaitPhase::Swing;
                leg.phase_timer = config.swing_duration_ts;
                leg.swing_from = leg.plant_pos;
                leg.swing_to = swing_to;
            }
            None
        }
        GaitPhase::Swing => {
            leg.phase_timer -= dt;
            // Continuous re-planning: refresh the target from the current hip
            // position, velocity and remaining flight time. Open-loop targets
            // frozen at lift-off are what made sudden stops land both feet on
            // one side of the hip.
            leg.swing_to.x =
                plan_target_x(hip_x, v, v_desired, leg.phase_timer.max(0.0), config, plan);
            // Progress 1→0 over the swing duration (elapsed fraction).
            let total = config.swing_duration_ts.max(f32::EPSILON);
            let t = (1.0 - (leg.phase_timer / total).clamp(0.0, 1.0)).clamp(0.0, 1.0);

            let x = leg.swing_from.x + (leg.swing_to.x - leg.swing_from.x) * t;
            let y = leg.swing_from.y + (leg.swing_to.y - leg.swing_from.y) * t;
            let lift = (t * (1.0 - t) * 4.0) * config.step_height;
            leg.foot_pos = Vec2::new(x, y + lift);

            if leg.phase_timer <= 0.0 {
                leg.foot_pos = leg.swing_to;
                leg.plant_pos = leg.swing_to;
                leg.phase = GaitPhase::Stance;
                leg.phase_timer = config.stance_duration_ts;
                // Touchdown event: signed hip→foot offset for the backward solve.
                return Some(leg.plant_pos.x - hip_x);
            }
            None
        }
    }
}

/// Run one full resolver tick.
///
/// # Pipeline (mirrors `plans/raibert_redesign.md`)
/// 1. Read target hip height (normal vs crouch).
/// 2. Precomputed feasibility envelope ([`GaitLimits`]) at the current `H`.
/// 3. Velocity shaping: accel clamped to `a_max`, stop decel floor
///    `v²/(2·X_max)`, ceiling `min(v_abs, v_cruise_max)`.
/// 4. Stop-settle servo once braking has bled the speed off.
/// 5. Forward math with per-frame re-planning + capture-point stop; targets
///    clamped to `hip ± X_max`.
/// 6. Backward solve at each touchdown event, blended with gain `beta`.
/// 7. Flat floor ⇒ plant y is `floor_y`; parabolic lift during swing.
/// 8. 2-bone IK for each leg → knees/feet.
pub fn step_lower_body(
    input_dir: Vec2,
    config: &LowerBodyConfig,
    state: &mut ResolverState,
    dt: f32,
) -> (LowerBodyPose, DebugInfo) {
    let dir = input_dir.normalize_or_zero();

    // 0. --- Vertical loop (loop 1) ---
    // Buffer jump intent from the input's vertical channel. A jump fires at the
    // next `Stand → Load` edge (see `tick_vertical`), which is the launch
    // window that keeps load + launch inside one stance (doc §5.7).
    if input_dir.y > 0.5 {
        state.vs.jump_requested = true;
    }

    // 1. --- Vertical integration ---
    // `tick_vertical` OWNS `hip_pos.y` / `hip_vel.y`: it regulates the grounded
    // hip spring (Stand/Load/Land) and integrates ballistically in Flight. The
    // loop closes on this *measured* state — never on the requested pose.
    tick_vertical(state, config, dt);
    // Measured height above the floor. Read back from the integrated position
    // (not the old kinematic target) so the horizontal planner sees the true
    // height, including mid-flight.
    let h = (state.hip_pos.y - config.floor_y).max(1.0);
    state.hip_height = h;

    // Airborne = no ground authority. The horizontal planner must not shape
    // velocity in the air: `v_x` carries ballistically (doc §6.2, §5A).
    let airborne = state.vs.phase == VerticalPhase::Flight;

    // 2. --- Feasibility envelope at the current height ---
    // The gait envelope (and therefore the swing targets, the velocity caps and
    // the `hip ± X_max` clamps) is evaluated at the GROUND reference height, not
    // the live hip height. A jump is a purely vertical event, so the horizontal
    // gait keeps its normal walking workspace even while the hip is far above the
    // floor. Using the live `h` here would make the envelope collapse to zero
    // once the hip rises past `thigh + calf` — `x_max = sqrt((L1+L2)² − H²)`
    // becomes imaginary — which clamped every foot target to the hip and pulled
    // both feet in behind the advancing body (the reported "legs lag behind").
    let h_plan = if config.crouch {
        config.crouch_height
    } else {
        config.hip_height
    };
    let limits = GaitLimits::from_config(config, h_plan, config.target_speed);
    let x_max = limits.x_max;
    // Cap the desired speed at the self-consistent cruise ceiling so the kv
    // feedback term can never develop a persistent bias larger than the
    // workspace (which would push late-stance feet out of reach).
    let v_desired_scalar = dir.x * config.target_speed.min(limits.v_plan_cap);

    // 3. --- Velocity shaping ---
    // While braking to a stop, raise the deceleration to at least the stop
    // floor `v²/(2·X_max)`: this guarantees the capture point stays within
    // reach, so the hip can always brake onto its next foot instead of
    // overshooting both plants.
    let v_prev = state.hip_vel.x;
    let stopping = v_desired_scalar == 0.0 && v_prev.abs() > SWING_TRIGGER_SPEED;
    let a_cmd = config.accel.min(limits.a_max).max(f32::EPSILON);
    let a_eff = if stopping {
        let a_stop_floor = v_prev * v_prev / (2.0 * x_max.max(f32::EPSILON));
        a_cmd.max(a_stop_floor)
    } else {
        a_cmd
    };
    // Ceiling: while braking, allow up to the absolute envelope (the hip may be
    // faster than cruise from external pushes); under power, cap at the
    // corrected cruise ceiling so plants stay in reach. Releasing input at
    // speed therefore never snaps the velocity.
    let v_ceil = if v_desired_scalar == 0.0 {
        limits.v_abs
    } else {
        limits.v_abs.min(limits.v_cruise_max)
    };
    let mut v = approach(v_prev, v_desired_scalar, a_eff * dt).clamp(-v_ceil, v_ceil);

    // 4. --- Stop settle: the feet own the hip ---
    // Once braking has bled the speed off and both feet are planted, glide the
    // hip to the support centre (midpoint of the planted feet) so the
    // character comes to rest balanced between them. The settle velocity is
    // capped by the kinematic braking limit `sqrt(2·a·d)` so the hip
    // decelerates smoothly into the midpoint without overshooting.
    if v_desired_scalar == 0.0
        && !stopping
        && state.left.phase == GaitPhase::Stance
        && state.right.phase == GaitPhase::Stance
    {
        let mid_x = (state.left.plant_pos.x + state.right.plant_pos.x) * 0.5;
        let to_mid = mid_x - state.hip_pos.x;
        if to_mid.abs() > 0.5 {
            let settle_time = config.stance_duration_ts.max(f32::EPSILON);
            let brake_limit = (2.0 * a_cmd * to_mid.abs()).sqrt();
            let v_settle = (to_mid / settle_time).clamp(-brake_limit, brake_limit);
            v = approach(v, v_settle, a_cmd * dt);
        } else {
            v = approach(v, 0.0, a_cmd * dt);
        }
    }

    // In Flight there is no ground authority: horizontal velocity carries
    // ballistically (no shaping, no braking). This is what makes walk → jump →
    // walk one continuous loop at a constant `v_x` (doc §6.2, §5A).
    if airborne {
        v = state.hip_vel.x;
    }
    state.hip_vel.x = v;

    // Integrate the horizontal hip position. The vertical position/velocity are
    // owned by `tick_vertical` above — there is no height pin here.
    state.hip_pos.x += v * dt;

    // 4c. --- Stance constraint: the hip may never outrun its pinned feet ---
    // A stance foot is pinned in world space. If the hip travels beyond the
    // (margined) horizontal reach of a planted foot — which the anti-phase
    // gate can provoke during hard transients, when the rear foot's lift-off
    // is delayed until the swinging foot plants — the IK ray-clamp would lift
    // that foot off the floor. Clamp the hip into the intersection of the
    // stance feet's windows `[plant ± X_max]`; the position stays continuous
    // and the constraint releases as soon as the trailing foot re-plants.
    // Gated OFF in Flight: a ballistic body has no ground to push against, so
    // there are no pinned feet to respect. Body axes freeze until touchdown.
    if !airborne {
        let mut lo = f32::NEG_INFINITY;
        let mut hi = f32::INFINITY;
        for leg in [&state.left, &state.right] {
            if leg.phase == GaitPhase::Stance {
                lo = lo.max(leg.plant_pos.x - x_max);
                hi = hi.min(leg.plant_pos.x + x_max);
            }
        }
        if lo <= hi {
            state.hip_pos.x = state.hip_pos.x.clamp(lo, hi);
        }
    }

    // 5. --- Capture point + per-leg foot placement ---
    // While stopping, aim the swing foot at the point where the hip will come
    // to rest under `a_eff` (Pratt capture point), so the hip brakes onto its
    // foot rather than past it.
    let capture_offset = if stopping {
        v * v.abs() / (2.0 * a_eff)
    } else {
        0.0
    };
    let plan = StepPlan {
        x_max,
        stopping,
        capture_offset,
    };

    // 5b. --- Jump landing seed (Flight→Land edge): both feet plant together ---
    // The jump's airborne half drives BOTH feet forward to one shared landing
    // point (a two-foot grounded stance), so on the first grounded frame both
    // legs are re-seeded onto a common footprint in `Stance` with a full stance
    // window. The seed preserves each foot's *rendered* x and drops it onto the
    // floor, and since the flight swing already left both feet on that point the
    // write is a no-op on the render: the Flight→Land boundary is continuous.
    // Done BEFORE `step_leg` so its `foot_pos = plant_pos` write is a no-op.
    if !airborne && state.airborne_prev {
        let floor = config.floor_y;
        for leg in [&mut state.left, &mut state.right] {
            let planted = Vec2::new(leg.foot_pos.x, floor);
            leg.foot_pos = planted;
            leg.plant_pos = planted;
            leg.phase = GaitPhase::Stance;
            leg.phase_timer = config.stance_duration_ts;
        }
    }

    // A jump is a VERTICAL event: it must not alter the horizontal gait. The legs
    // keep their normal alternating swing (each targets its own forward
    // `plan_target_x`), so at any instant one foot leads ahead of the hip for the
    // next step — exactly as while walking — and the character lands and walks
    // straight on. (The old two-foot-sync retarget forced BOTH feet onto one
    // shared, stale plant point, which is what made both feet lag behind — or,
    // over-corrected, both lunge ahead. Neither is a normal gait.)
    let left_td = step_leg(
        &mut state.left,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.right.phase == GaitPhase::Swing,
        config,
        &plan,
        dt,
    );
    let right_td = step_leg(
        &mut state.right,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.left.phase == GaitPhase::Swing,
        config,
        &plan,
        dt,
    );

    // 6. --- Backward solve at touchdown (foot → hip) ---
    // Invert the *actual* plant offset to get the hip velocity whose next
    // forward plan lands exactly on it, then blend with gain `beta` (1 =
    // deadbeat). Skipped while stopping: capture plants encode "brake to rest
    // here", which the cruise inversion would misread as a velocity command.
    let mut v_support = 0.0;
    // Skipped in Flight: no touchdown can fire, and `v_x` must carry unchanged.
    if !airborne && v_desired_scalar != 0.0 {
        if let Some(delta) = left_td.or(right_td) {
            v_support = compute_support_velocity(
                delta,
                config.kv,
                config.stance_duration_ts,
                v_desired_scalar,
            );
            let beta = config.beta.clamp(0.0, 1.0);
            v = (v + beta * (v_support - v)).clamp(-v_ceil, v_ceil);
            state.hip_vel.x = v;
        }
    }

    // 6b. --- Jump airborne swing: both feet drive forward to the landing point ---
    // The jump's airborne half (Flight) is the *swing* phase of the leading leg's
    // full cycle: with `swing_duration == stance_duration` the airborne half is
    // exactly half the cycle (duty factor β = 0.5). Both legs are SYNCED — the
    // trailing leg simply mirrors the leading leg — so they reach forward together
    // and land on the SAME point.
    //
    // The hip is far above the floor here, so the leg is inherently near full
    // extension and the 2-bone IK sits on its `acos(reach/(L1+L2))` singularity.
    // The foot is therefore driven as a hip-relative OFFSET, swept from its
    // lift-off offset (behind the hip) to the shared landing offset (ahead of the
    // hip), with the offset length CLAMPED to `LIFT_REACH_SAFE_FRACTION·(L1+L2)`.
    // That holds the knee at `α = acos(0.96) ≈ 16°` or more, so a fast hip can
    // never snap the knee straight. (A world-space floor target is useless here:
    // it is unreachable while the hip is high, so the IK ray-clamps it onto the
    // very same singularity.)
    if airborne {
        let full = (config.thigh_len + config.calf_len).max(f32::EPSILON);
        let reach_cap = full * LIFT_REACH_SAFE_FRACTION;
        // Shared landing offset target: one forward Raibert reach ahead of the hip
        // for a running (walking) lift-off, or straight down onto the lift-off
        // footprint for a stand-still jump. Capped at `FLIGHT_SWING_REACH_FRACTION
        // ·x_max` so the landing pose stays well inside the safe bend zone.
        let t_step = plan_target_x(state.hip_pos.x, v, v_desired_scalar, 0.0, config, &plan);
        let max_reach = x_max * FLIGHT_SWING_REACH_FRACTION;
        let land_x = (if state.jump_arm_lead_leg {
            t_step
        } else {
            state.hip_pos.x
        })
        .clamp(state.hip_pos.x - max_reach, state.hip_pos.x + max_reach);
        // Lift-off (first airborne frame): latch each foot's hip-relative offset
        // (clamped into the bend zone) as the sweep start, so the first airborne
        // frame is continuous with the planted pose. Stored in `vs` so a stance
        // leg's `step_leg` lift-off mid-flight cannot clobber it.
        if !state.airborne_prev {
            let mut lo = state.left.foot_pos - state.hip_pos;
            let lr0 = lo.length();
            if lr0 > reach_cap {
                lo *= reach_cap / lr0;
            }
            state.vs.swing_from_off_left = lo;
            let mut ro = state.right.foot_pos - state.hip_pos;
            let rr0 = ro.length();
            if rr0 > reach_cap {
                ro *= reach_cap / rr0;
            }
            state.vs.swing_from_off_right = ro;
        }
        // Time base: `swing_t / swing_period → [0, 1]` across the whole airborne
        // half-cycle (both latched / advanced by `tick_vertical`).
        let prog = if state.vs.swing_period > f32::EPSILON {
            (state.vs.swing_t / state.vs.swing_period).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let lift = if state.vs.swing_lift > 0.0 {
            state.vs.swing_lift
        } else {
            config.step_height * FLIGHT_SWING_LIFT_FRACTION
        };
        let arc = prog * (1.0 - prog) * 4.0 * lift;
        let floor = config.floor_y;
        // Hip → shared landing point, clamped into the bend zone.
        let mut land_off = Vec2::new(land_x - state.hip_pos.x, floor - state.hip_pos.y);
        let lr = land_off.length();
        if lr > reach_cap {
            land_off *= reach_cap / lr;
        }
        for (leg, from_off) in [
            (&mut state.left, state.vs.swing_from_off_left),
            (&mut state.right, state.vs.swing_from_off_right),
        ] {
            let mut off = from_off.lerp(land_off, prog);
            off.y += arc;
            let r = off.length();
            if r > reach_cap {
                off *= reach_cap / r;
            }
            let mut p = state.hip_pos + off;
            p.y = p.y.max(floor);
            leg.foot_pos = p;
        }
    }
    state.airborne_prev = airborne;

    // 7-8. --- IK ---

    // IK returns the *effective* foot (clamped to the reachable annulus), so the
    // rendered bones always satisfy thigh/calf length even when the hip briefly
    // outruns a pinned stance foot.
    let (left_knee, left_foot) = solve_leg_ik(
        state.hip_pos,
        state.left.foot_pos,
        config.thigh_len,
        config.calf_len,
        config.bend_sign,
    );
    let (right_knee, right_foot) = solve_leg_ik(
        state.hip_pos,
        state.right.foot_pos,
        config.thigh_len,
        config.calf_len,
        config.bend_sign,
    );

    let pose = LowerBodyPose {
        hip: state.hip_pos,
        left_knee,
        right_knee,
        left_foot,
        right_foot,
    };

    let debug = DebugInfo {
        x_max,
        v_max: v_ceil,
        v_desired: v_desired_scalar,
        v_clamped: v,
        left_phase: state.left.phase,
        right_phase: state.right.phase,
        left_target_x: state.left.swing_to.x,
        right_target_x: state.right.swing_to.x,
        a_max: limits.a_max,
        v_abs: limits.v_abs,
        v_support,
        capture_x: capture_offset,
        v_phase: state.vs.phase,
        hip_height: h,
        hip_vel_y: state.hip_vel.y,
        h_desired: config.jump_apex,
        h_apex: state.vs.h_apex_prev,
        delta_l: state.vs.delta_l,
    };

    (pose, debug)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> LowerBodyConfig {
        LowerBodyConfig::default()
    }

    #[test]
    fn test_compute_x_max_pythagorean() {
        // (L1 + L2) = 110, H = 90 → sqrt(110² - 90²) = sqrt(4000) ≈ 66.33
        let cfg = default_config();
        let raw = compute_x_max(cfg.thigh_len, cfg.calf_len, 90.0, 1.0);
        let expected = (110.0f32 * 110.0 - 90.0 * 90.0).sqrt();
        assert!(
            (raw - expected).abs() < 1e-3,
            "raw={raw} expected={expected}"
        );
        // margin scales X_max linearly.
        let margined = compute_x_max(cfg.thigh_len, cfg.calf_len, 90.0, 0.5);
        assert!((margined - expected * 0.5).abs() < 1e-3);
        // Impossibly high hip → clamped to zero reach.
        assert_eq!(compute_x_max(50.0, 50.0, 200.0, 1.0), 0.0);
    }

    /// Non-stopping plan context for the direct `step_leg` unit tests.
    fn cruise_plan(cfg: &LowerBodyConfig) -> StepPlan {
        StepPlan {
            x_max: compute_x_max(
                cfg.thigh_len,
                cfg.calf_len,
                cfg.hip_height,
                cfg.safety_margin,
            ),
            stopping: false,
            capture_offset: 0.0,
        }
    }

    #[test]
    fn test_compute_v_max_formula() {
        // Two-sided constraint v·Ts/2 + kv·|v − v_d| ≤ X_max; the forward
        // side binds when v ≥ v_d: (X_max + kv·v_d) / (Ts/2 + kv).
        let v_max = compute_v_max(66.33, 0.12, 0.28, 120.0);
        let fwd = (66.33 + 0.12 * 120.0) / (0.28 * 0.5 + 0.12);
        let back = (66.33 - 0.12 * 120.0) / (0.28 * 0.5 - 0.12);
        assert!(fwd < back, "forward side should bind at these params");
        assert!((v_max - fwd).abs() < 1e-3, "v_max={v_max} fwd={fwd}");
        // With kv = 0 both sides collapse to the geometric cruise limit
        // 2·X_max/Ts.
        let geo = compute_v_max(66.33, 0.0, 0.28, 120.0);
        assert!((geo - 2.0 * 66.33 / 0.28).abs() < 1e-3, "geo={geo}");
        // Guard against degenerate denominator.
        assert_eq!(compute_v_max(50.0, 0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn test_gait_limits_envelope() {
        let cfg = default_config();
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        // Hand-computed defaults: X_max = √(110²−90²)·0.85 ≈ 53.76
        let x_max = (110.0f32 * 110.0 - 90.0 * 90.0).sqrt() * 0.85;
        assert!(
            (limits.x_max - x_max).abs() < 1e-2,
            "x_max={}",
            limits.x_max
        );
        // a_max = 2·m_e·X_max / T_swing²
        let a_max = 2.0 * cfg.prediction_margin * x_max / (cfg.swing_duration_ts.powi(2));
        assert!(
            (limits.a_max - a_max).abs() < 1e-1,
            "a_max={}",
            limits.a_max
        );
        // v_abs = 2·X_max·√m_e / T_swing
        let v_abs = 2.0 * x_max * cfg.prediction_margin.sqrt() / cfg.swing_duration_ts;
        assert!(
            (limits.v_abs - v_abs).abs() < 1e-1,
            "v_abs={}",
            limits.v_abs
        );
        // Consistency: at v_abs the stop decel floor v²/(2·X_max) equals a_max.
        let a_stop_at_v_abs = v_abs * v_abs / (2.0 * x_max);
        assert!(
            (a_stop_at_v_abs - a_max).abs() / a_max < 1e-3,
            "a_stop(v_abs)={a_stop_at_v_abs} != a_max={a_max}"
        );
        // v_plan_cap = 2·X_max / Ts (self-consistent cruise ceiling, kv term 0).
        let v_plan_cap = 2.0 * x_max / cfg.stance_duration_ts;
        assert!(
            (limits.v_plan_cap - v_plan_cap).abs() < 1e-2,
            "v_plan_cap={} expected={v_plan_cap}",
            limits.v_plan_cap
        );
        // Degenerate height (hip above full reach) collapses the envelope.
        let degen = GaitLimits::from_config(&cfg, 200.0, cfg.target_speed);
        assert_eq!(degen.x_max, 0.0);
        assert_eq!(degen.a_max, 0.0);
        assert_eq!(degen.v_abs, 0.0);
    }

    #[test]
    fn test_backward_forward_identity() {
        // backward∘forward = identity: the support velocity inverted from a
        // forward-planned offset recovers the original velocity exactly.
        let cfg = default_config();
        for v in [-200.0, -50.0, 0.0, 37.5, 120.0, 300.0] {
            for v_d in [-120.0, 0.0, 120.0] {
                let delta = compute_forward_target(v, v_d, cfg.kv, cfg.stance_duration_ts);
                let v_back = compute_support_velocity(delta, cfg.kv, cfg.stance_duration_ts, v_d);
                assert!(
                    (v_back - v).abs() < 1e-2,
                    "v={v} v_d={v_d} → v_back={v_back}"
                );
            }
        }
        // Degenerate denominator guard.
        assert_eq!(compute_support_velocity(10.0, 0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn test_support_velocity_corrects_plant_error() {
        // A plant error `e` at touchdown maps to the velocity correction
        // `e / (Ts/2 + kv)`, annihilated in one stride (deadbeat).
        let cfg = default_config();
        let v = cfg.target_speed;
        let e = 7.5;
        let delta = v * cfg.stance_duration_ts * 0.5 + e;
        let v_support = compute_support_velocity(delta, cfg.kv, cfg.stance_duration_ts, v);
        let expected = v + e / (cfg.stance_duration_ts * 0.5 + cfg.kv);
        assert!(
            (v_support - expected).abs() < 1e-3,
            "v_support={v_support} expected={expected}"
        );
    }

    #[test]
    fn test_compute_forward_target_neutral_point() {
        // At v == v_desired, x_target = v·Ts/2 (the neutral stance midpoint).
        let cfg = default_config();
        let v = 120.0;
        let x = compute_forward_target(v, v, cfg.kv, cfg.stance_duration_ts);
        assert!((x - v * cfg.stance_duration_ts * 0.5).abs() < 1e-3);
        // At v == 0, x_target = -kv·v_desired (foot steps behind the hip).
        let stop = compute_forward_target(0.0, 120.0, cfg.kv, cfg.stance_duration_ts);
        assert!((stop - (-cfg.kv * 120.0)).abs() < 1e-3);
    }

    #[test]
    fn test_landing_compensates_hip_motion_during_swing() {
        // Raibert neutral point: the foot must land `v·Ts/2` ahead of the hip
        // *at touchdown*, so the hip passes over the foot at mid-stance. This
        // requires predicting the hip's travel during the swing phase.
        let cfg = default_config();
        let v = cfg.target_speed; // cruise: kv term vanishes
        let mut leg = LegState {
            phase: GaitPhase::Stance,
            phase_timer: 0.0,
            foot_pos: Vec2::ZERO,
            plant_pos: Vec2::ZERO,
            swing_from: Vec2::ZERO,
            swing_to: Vec2::ZERO,
        };

        // Lift off with the hip at x = 0.
        let plan = cruise_plan(&cfg);
        let _ = step_leg(&mut leg, 0.0, v, v, false, &cfg, &plan, 0.001);
        assert_eq!(leg.phase, GaitPhase::Swing);

        // Fly through the swing while the hip keeps moving.
        let dt = 0.01;
        let mut hip_x = 0.0;
        let mut touchdown = None;
        for _ in 0..((cfg.swing_duration_ts / dt).ceil() as usize + 2) {
            hip_x += v * dt;
            touchdown = step_leg(&mut leg, hip_x, v, v, false, &cfg, &plan, dt);
            if leg.phase == GaitPhase::Stance {
                break;
            }
        }
        assert_eq!(leg.phase, GaitPhase::Stance, "swing never completed");

        let offset = leg.plant_pos.x - hip_x;
        assert!(
            offset > 0.0,
            "foot landed behind the hip: offset={offset:.1}px"
        );
        let expected = v * cfg.stance_duration_ts * 0.5;
        assert!(
            (offset - expected).abs() < v * dt * 2.0,
            "offset={offset:.1}px expected≈{expected:.1}px"
        );
        // The touchdown event reports the same signed hip→foot offset.
        assert!(
            touchdown.is_some_and(|d| (d - offset).abs() < 1e-3),
            "touchdown event missing or wrong: {touchdown:?} vs {offset}"
        );
    }

    #[test]
    fn test_gait_alternates_phases() {
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        // Both legs begin planted (no idle hop).
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);

        // Right leg is half a stance out of phase: after half a stance of
        // forward motion it swings while the left is still in stance.
        let dt = cfg.stance_duration_ts * 0.5;
        let plan = cruise_plan(&cfg);
        let _ = step_leg(
            &mut state.left,
            state.hip_pos.x,
            40.0,
            120.0,
            state.right.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        let _ = step_leg(
            &mut state.right,
            state.hip_pos.x,
            40.0,
            120.0,
            state.left.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Swing);
    }

    #[test]
    fn test_no_step_when_stationary() {
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        // At standstill (v = 0, v_desired = 0) the stance never opens a swing.
        let dt = cfg.stance_duration_ts + cfg.swing_duration_ts + 0.1;
        let plan = cruise_plan(&cfg);
        let _ = step_leg(
            &mut state.left,
            state.hip_pos.x,
            0.0,
            0.0,
            state.right.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        let _ = step_leg(
            &mut state.right,
            state.hip_pos.x,
            0.0,
            0.0,
            state.left.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
    }

    #[test]
    fn test_stance_foot_holds_position() {
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let planted = state.left.plant_pos;

        // A stance leg keeps its foot pinned even as the hip moves. Step in
        // small increments so the phase never times out into a swing.
        let dt = cfg.stance_duration_ts * 0.25;
        let plan = cruise_plan(&cfg);
        let mut hip_x = state.hip_pos.x;
        for _ in 0..3 {
            hip_x += 40.0 * dt;
            let _ = step_leg(
                &mut state.left,
                hip_x,
                40.0,
                120.0,
                state.right.phase == GaitPhase::Swing,
                &cfg,
                &plan,
                dt,
            );
            assert!(
                (state.left.foot_pos - planted).length() < 1e-3,
                "stance foot drifted: {:?} vs {:?}",
                state.left.foot_pos,
                planted
            );
        }
    }

    #[test]
    fn test_solve_leg_ik_length_preservation() {
        let cfg = default_config();
        let hip = Vec2::new(0.0, cfg.hip_height);

        // In-reach, exactly-at-max-reach, out-of-reach, too-close and coincident
        // targets: bone lengths must hold for the *effective* foot in all cases.
        let targets = [
            Vec2::new(30.0, cfg.floor_y),          // in reach
            Vec2::new(80.0, cfg.floor_y),          // dist ≈ 114 > L1+L2 = 110
            Vec2::new(110.0, cfg.hip_height),      // exactly at max reach
            Vec2::new(200.0, cfg.floor_y + 200.0), // far out of reach
            Vec2::new(0.0, cfg.floor_y),           // straight down (dist = H)
            Vec2::new(0.0, cfg.hip_height),        // coincident with hip
        ];

        for bend in [1.0, -1.0] {
            for target in targets {
                let (knee, foot) = solve_leg_ik(hip, target, cfg.thigh_len, cfg.calf_len, bend);
                let thigh = (knee - hip).length();
                let calf = (foot - knee).length();
                assert!(
                    (thigh - cfg.thigh_len).abs() < 1e-2,
                    "thigh={thigh} bend={bend} target={target:?}"
                );
                assert!(
                    (calf - cfg.calf_len).abs() < 1e-2,
                    "calf={calf} bend={bend} target={target:?}"
                );
                // The effective foot lies on the hip→target ray, never farther
                // than the target itself.
                if (target - hip).length_squared() > 1e-6 {
                    let ray = (target - hip).normalize();
                    let offset = foot - hip;
                    assert!(
                        offset.dot(ray) >= offset.length() - 1e-2,
                        "foot off the hip→target ray: bend={bend} target={target:?}"
                    );
                    assert!(
                        offset.length() <= (target - hip).length() + 1e-3,
                        "foot beyond target: bend={bend} target={target:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_pose_bones_preserve_length_while_walking() {
        // End-to-end invariant: whatever the gait does (accelerate hard, let the
        // hip outrun a pinned stance foot), the *rendered* pose must always have
        // thigh/calf bones of the configured length.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let input = Vec2::X;
        // Run several full strides at full throttle.
        let frames = ((cfg.stance_duration_ts + cfg.swing_duration_ts) * 60.0 * 4.0) as usize;
        for frame in 0..frames {
            let (pose, _debug) = step_lower_body(input, &cfg, &mut state, dt);
            for (knee, foot, side) in [
                (pose.left_knee, pose.left_foot, "left"),
                (pose.right_knee, pose.right_foot, "right"),
            ] {
                let thigh = (knee - pose.hip).length();
                let calf = (foot - knee).length();
                assert!(
                    (thigh - cfg.thigh_len).abs() < 1e-2,
                    "frame {frame} {side} thigh={thigh}"
                );
                assert!(
                    (calf - cfg.calf_len).abs() < 1e-2,
                    "frame {frame} {side} calf={calf}"
                );
            }
        }
    }

    #[test]
    fn test_hip_stays_between_feet_at_cruise() {
        // The reported symptom: while walking, both feet trailed behind the hip.
        // With the swing-flight prediction, at steady cruise the feet must
        // straddle the hip (one ahead, one behind) for most of the stride.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let input = Vec2::X;

        // Spin up to cruise speed and past the initial transient (3 full
        // stride cycles).
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let warmup = (stride * 3.0 / dt) as usize;
        for _ in 0..warmup {
            step_lower_body(input, &cfg, &mut state, dt);
        }

        // Measure one more full stride: count frames where the hip is strictly
        // between the two feet.
        let mut straddled = 0usize;
        let total = (stride / dt) as usize;
        for _ in 0..total {
            let (pose, _debug) = step_lower_body(input, &cfg, &mut state, dt);
            let lo = pose.left_foot.x.min(pose.right_foot.x);
            let hi = pose.left_foot.x.max(pose.right_foot.x);
            if pose.hip.x > lo && pose.hip.x < hi {
                straddled += 1;
            }
        }
        let frac = straddled as f32 / total as f32;
        assert!(
            frac >= 0.5,
            "hip only straddled by the feet on {:.0}% of the stride at cruise",
            frac * 100.0
        );
    }

    #[test]
    fn test_sudden_stop_hip_settles_between_feet() {
        // Symptom: on a sudden stop both feet ended up ahead of the hip and
        // the gait froze there. With the stop-settle servo the hip must glide
        // to the support centre and come to rest between the planted feet.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;

        // Cruise for 3 strides.
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 3.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        let hip_at_stop = state.hip_pos.x;

        // Sudden stop: release input and let it settle.
        for _ in 0..(3.0 / dt) as usize {
            step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
        }

        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
        let mid = (state.left.plant_pos.x + state.right.plant_pos.x) * 0.5;
        assert!(
            (state.hip_pos.x - mid).abs() < 2.0,
            "hip did not settle between the feet: hip={:.1} mid={mid:.1}",
            state.hip_pos.x
        );
        assert!(
            state.hip_vel.x.abs() < 5.0,
            "hip still sliding after settle: v={}",
            state.hip_vel.x
        );
        // The settle must have actually moved the hip toward the feet (they
        // were planted ahead of it at cruise-stop).
        assert!(
            (state.hip_pos.x - hip_at_stop).abs() > 1.0,
            "hip never moved during settle"
        );
    }

    #[test]
    fn test_no_phantom_steps_after_stop() {
        // With no move intent and the hip fully settled, the gait must freeze:
        // the plants stay put, so the settle servo can never chain into a
        // phantom walk-off. (Braking steps *while* the hip is still moving are
        // legal — they are the capture-point stop — so settle fully first.)
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 3.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        // Brake + settle completely.
        for _ in 0..(3.0 / dt) as usize {
            step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
        }
        assert!(state.hip_vel.x.abs() < SWING_TRIGGER_SPEED);
        let (l_plant, r_plant) = (state.left.plant_pos, state.right.plant_pos);
        for _ in 0..(2.0 / dt) as usize {
            step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
        }
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
        assert!(
            (state.left.plant_pos - l_plant).length() < 1e-3,
            "left foot re-planted with no input"
        );
        assert!(
            (state.right.plant_pos - r_plant).length() < 1e-3,
            "right foot re-planted with no input"
        );
    }

    #[test]
    fn test_no_floating_feet_at_max_speed() {
        // Symptom: at high speed the stance foot floated in mid-air because
        // the plant target exceeded the leg reach and the IK ray-clamped it
        // off the floor. With the corrected ceiling + workspace clamp, every
        // stance foot must stay on the floor and within reach at all times.
        let mut cfg = default_config();
        cfg.target_speed = 600.0; // far above the feasible envelope
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let reach = cfg.thigh_len + cfg.calf_len;
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let frames = (stride * 6.0 / dt) as usize;
        let mut v_peak = 0.0f32;
        for frame in 0..frames {
            let (pose, debug) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
            v_peak = v_peak.max(debug.v_clamped.abs());
            for (foot, phase, side) in [
                (pose.left_foot, debug.left_phase, "left"),
                (pose.right_foot, debug.right_phase, "right"),
            ] {
                if phase == GaitPhase::Stance {
                    assert!(
                        (foot.y - cfg.floor_y).abs() < 1e-2,
                        "frame {frame} {side} stance foot off the floor: y={}",
                        foot.y
                    );
                    let d = (foot - pose.hip).length();
                    assert!(
                        d <= reach + 1e-2,
                        "frame {frame} {side} stance foot out of reach: d={d}"
                    );
                }
            }
        }
        // The velocity must have been clamped to the feasible cruise ceiling.
        assert!(
            v_peak <= limits.v_cruise_max + 1e-2,
            "v_peak={v_peak} exceeded cruise ceiling {}",
            limits.v_cruise_max
        );
        assert!(
            v_peak > 0.5 * limits.v_cruise_max,
            "never reached a meaningful speed: v_peak={v_peak}"
        );
    }

    #[test]
    fn test_sudden_stop_from_high_speed() {
        // Symptom: on a sudden stop from speed both feet ended up on one side
        // of the hip. With the capture-point stop + decel floor the hip must
        // brake onto its feet and come to rest between them, and the capture
        // point must never leave the workspace.
        let mut cfg = default_config();
        cfg.target_speed = 350.0;
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 4.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        assert!(
            state.hip_vel.x > 100.0,
            "did not reach speed before the stop: v={}",
            state.hip_vel.x
        );

        // Sudden stop: release input, verify the capture point stays in reach
        // while braking, then let it settle.
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        for _ in 0..(4.0 / dt) as usize {
            let (_pose, debug) = step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
            assert!(
                debug.capture_x.abs() <= limits.x_max + 1e-2,
                "capture point out of reach: {}",
                debug.capture_x
            );
        }

        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
        let mid = (state.left.plant_pos.x + state.right.plant_pos.x) * 0.5;
        assert!(
            (state.hip_pos.x - mid).abs() < 2.0,
            "hip did not settle between the feet: hip={:.1} mid={mid:.1}",
            state.hip_pos.x
        );
        assert!(
            state.hip_vel.x.abs() < 5.0,
            "hip still sliding after settle: v={}",
            state.hip_vel.x
        );
    }

    #[test]
    fn test_velocity_continuity() {
        // The hip velocity must change continuously: bounded by the effective
        // acceleration (command accel clamped to a_max, or the stop decel
        // floor) on every non-touchdown frame. Touchdown frames apply the
        // backward-solve blend and are exempt.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        let a_bound = cfg.accel.min(limits.a_max);
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;

        let mut v_prev = 0.0;
        let frames = (stride * 8.0 / dt) as usize;
        for frame in 0..frames {
            // Cruise, then sudden stop halfway through.
            let input = if frame < frames / 2 {
                Vec2::X
            } else {
                Vec2::ZERO
            };
            let (_pose, debug) = step_lower_body(input, &cfg, &mut state, dt);
            if debug.v_support == 0.0 {
                // Stop decel floor: v²/(2·X_max), evaluated at the frame's
                // starting speed.
                let a_stop = v_prev * v_prev / (2.0 * limits.x_max);
                let dv_max = a_bound.max(a_stop) * dt * 1.05 + 1e-3;
                assert!(
                    (debug.v_clamped - v_prev).abs() <= dv_max,
                    "frame {frame}: velocity jumped {} → {} (dv_max={dv_max})",
                    v_prev,
                    debug.v_clamped
                );
            }
            v_prev = debug.v_clamped;
        }
    }

    // --- Loop 1 (vertical) ---

    #[test]
    fn test_vertical_pure_math() {
        let g = 980.0;
        // `apex_to_launch_speed` and `predict_apex_rise` are exact inverses.
        let v = apex_to_launch_speed(50.0, g);
        assert!((v - (2.0f32 * g * 50.0).sqrt()).abs() < 1e-3);
        assert!((predict_apex_rise(v, g) - 50.0).abs() < 1e-2);
        // Energy conservation: falling the same height recovers launch speed.
        assert!((landing_speed(50.0, g) - v).abs() < 1e-3);
        // The Δl law: a one-time jump starts from Δl_prev = 0.
        assert_eq!(update_delta_l(0.0, 60.0, 0.0, 0.5), 30.0);
        // With the error at zero the integrator holds its memory.
        assert!((update_delta_l(30.0, 60.0, 60.0, 0.5) - 30.0).abs() < 1e-6);
        // A shortfall banks more displacement; an overshoot gives some back.
        assert!((update_delta_l(30.0, 60.0, 40.0, 0.5) - 40.0).abs() < 1e-6);
        assert!((update_delta_l(30.0, 60.0, 80.0, 0.5) - 20.0).abs() < 1e-6);
    }

    /// Drive the resolver until one jump request completes a full hop; return
    /// the latched apex of that hop.
    fn run_one_jump(cfg: &LowerBodyConfig, state: &mut ResolverState, dt: f32) -> f32 {
        state.vs.jump_requested = true;
        for _ in 0..4000 {
            let _ = step_lower_body(Vec2::ZERO, cfg, state, dt);
            if state.vs.phase == VerticalPhase::Stand && state.vs.h_apex_prev > 0.0 {
                return state.vs.h_apex_prev;
            }
        }
        state.vs.h_apex_prev
    }

    #[test]
    fn test_vertical_jump_and_integrator() {
        // A one-time jump must leave the ground, bank displacement, latch its
        // apex, and — repeated — keep the apex in a bounded band around the
        // target via the Δl integrator (never diverging).
        let cfg = LowerBodyConfig::default();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let target = cfg.jump_apex;

        let apex1 = run_one_jump(&cfg, &mut state, dt);
        assert!(apex1 > 0.4 * target, "first hop too low: {apex1}");
        assert!(state.vs.delta_l > 0.0, "no displacement was banked");
        assert!(state.vs.h_apex_prev > 0.0, "apex was not latched");

        let mut apexes = vec![apex1];
        for _ in 0..7 {
            apexes.push(run_one_jump(&cfg, &mut state, dt));
            assert!(
                state.vs.delta_l.is_finite() && state.vs.delta_l < 4.0 * target,
                "Δl diverged: {}",
                state.vs.delta_l
            );
        }
        let late = &apexes[apexes.len() - 4..];
        let mean = late.iter().sum::<f32>() / late.len() as f32;
        assert!(
            (mean - target).abs() < 0.45 * target,
            "integrator did not converge near target {target}: mean={mean} apexes={apexes:?}"
        );
    }

    #[test]
    fn test_vertical_flight_is_ballistic() {
        // Anti-hover: Flight is unactuated, so the vertical velocity must fall
        // by exactly g·dt every frame — never rising, never held.
        let cfg = LowerBodyConfig::default();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        state.vs.jump_requested = true;

        let mut prev_hdot = 0.0f32;
        let mut in_flight = false;
        let mut saw_flight = false;
        for _ in 0..2000 {
            let (_p, d) = step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
            if d.v_phase == VerticalPhase::Flight {
                if in_flight {
                    let dv = d.hip_vel_y - prev_hdot;
                    assert!(
                        (dv + cfg.gravity * dt).abs() < 1e-2,
                        "Flight not ballistic: dḣ={dv}, expected {}",
                        -cfg.gravity * dt
                    );
                }
                saw_flight = true;
                in_flight = true;
                prev_hdot = d.hip_vel_y;
            } else if saw_flight {
                break;
            }
        }
        assert!(saw_flight, "never entered Flight");
    }

    #[test]
    fn test_vertical_preserves_horizontal_velocity() {
        // walk → jump → walk is one continuous loop: v_x is conserved through
        // the air (no ground shaping in Flight) and the gait resumes at it.
        let cfg = LowerBodyConfig::default();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 3.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        assert!(
            state.hip_vel.x > 100.0,
            "did not reach cruise before the jump: {}",
            state.hip_vel.x
        );

        state.vs.jump_requested = true;
        let mut v_in_flight: Option<f32> = None;
        let mut saw_flight = false;
        for _ in 0..2000 {
            let (_p, d) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
                match v_in_flight {
                    None => v_in_flight = Some(d.v_clamped),
                    Some(v0) => assert!(
                        (d.v_clamped - v0).abs() < 1e-3,
                        "v_x changed in Flight: {v0} → {}",
                        d.v_clamped
                    ),
                }
            } else if saw_flight && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw_flight, "never entered Flight");
        assert!(
            state.hip_vel.x > 100.0,
            "lost speed across the jump: {}",
            state.hip_vel.x
        );
    }

    #[test]
    fn test_jump_syncs_both_legs_onto_one_landing_point() {
        // The jump's airborne half is a SYNCED full-circle swing: the trailing leg
        // mirrors the leading leg, so both feet reach forward together and land on
        // the SAME point (a two-foot grounded stance). This is the opposite of the
        // walk gait, where the two feet stay a stride apart (alternating
        // anti-phase). The horizontal foot gap must therefore COLLAPSE to ~0 across
        // Flight — unlike the old freeze, which held the lift-off split, and the
        // old in-air stride, which kept the anti-phase spread.
        let cfg = LowerBodyConfig::default();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 3.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        // Measured walking spread just before the hop (plants on either side of
        // the hip at the anti-phase extremes).
        let walk_spread = (state.left.plant_pos.x - state.right.plant_pos.x).abs();
        assert!(
            walk_spread > 10.0,
            "not walking before the hop: {walk_spread}"
        );

        state.vs.jump_requested = true;
        let mut min_gap = f32::INFINITY;
        let mut end_gap = f32::INFINITY;
        let mut saw_flight = false;
        for _ in 0..4000 {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
                let gap = (pose.left_foot.x - pose.right_foot.x).abs();
                min_gap = min_gap.min(gap);
                end_gap = gap;
            }
            if saw_flight && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw_flight, "hop never went airborne");
        // The synced swing must bring the two feet together ...
        assert!(
            min_gap < 2.0,
            "the synced swing never brought the feet together (min gap = {min_gap:.1}px)"
        );
        // ... and leave them on a single, shared landing point for touchdown.
        assert!(
            end_gap < 2.0,
            "the two legs did not land on the same point (end-of-flight gap = {end_gap:.1}px)"
        );
    }

    // --- Loop 1 numeric: the whole walking → jumping → walking episode ---
    //
    // The resolver is pure math over 5 points (1 hip + 2 knees + 2 feet), so the
    // entire episode is a headless numeric trace. These tests check that the
    // NUMBERS fit the kinematics rather than just that the phases advance.

    /// Run a hop to completion; return the latched apex rise and the number of
    /// ballistic ticks spent with no ground support. `run_one_jump` discards the
    /// timing, which the numeric loop tests need.
    fn run_hop_measured(cfg: &LowerBodyConfig, state: &mut ResolverState, dt: f32) -> (f32, u32) {
        state.vs.jump_requested = true;
        let mut flight_ticks = 0u32;
        for _ in 0..6000 {
            let (_p, d) = step_lower_body(Vec2::ZERO, cfg, state, dt);
            if d.v_phase == VerticalPhase::Flight {
                flight_ticks += 1;
            }
            if state.vs.phase == VerticalPhase::Stand && state.vs.h_apex_prev > 0.0 {
                return (state.vs.h_apex_prev, flight_ticks);
            }
        }
        (state.vs.h_apex_prev, flight_ticks)
    }

    #[test]
    fn test_vertical_apex_matches_target_and_ballistic_timing() {
        // The NUMBERS must fit. The altitude-aware launch cutoff releases the hip
        // at a measured h-dot that balances to `stand + h_desired`, so the latched
        // apex equals the target, and the observed Flight duration closes the
        // ballistic relation `t_flight ≈ 2·ḣ₀/g̃` for `ḣ₀ = sqrt(2·g̃·h_apex)`.
        let cfg = LowerBodyConfig::default();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let g = cfg.gravity;
        let target = cfg.jump_apex;

        // The release is a per-tick threshold, so Launch can overshoot the ideal
        // cutoff by up to one tick of exit speed, `launch_accel·dt`. That is an
        // apex jitter of `ḣ₀*·(launch_accel·dt)/g̃` that no integrator can remove
        // (it is the quantization of the cutoff, not a modeling error). The
        // per-hop band must admit exactly one such quantum; the MEAN, which the
        // Δl law centres, must fit far tighter.
        let v_ideal = apex_to_launch_speed(target, g);
        let quantum = v_ideal * (cfg.launch_accel * dt) / g;

        let mut apexes = Vec::new();
        for hop in 0..8 {
            let (apex, flight_ticks) = run_hop_measured(&cfg, &mut state, dt);
            apexes.push(apex);
            assert!(apex.is_finite(), "hop {hop}: apex not finite");
            assert!(
                (apex - target).abs() <= quantum + 2.0,
                "hop {hop}: apex {apex} outside one launch-quantum of target {target} \
                 (quantum={quantum}, Δl={})",
                state.vs.delta_l
            );

            // Ballistic timing closes on the *measured* apex: t_flight ≈ 2·ḣ₀/g̃.
            let h_dot0 = apex_to_launch_speed(apex, g);
            let t_pred = 2.0 * h_dot0 / g;
            let t_meas = flight_ticks as f32 * dt;
            assert!(
                (t_meas - t_pred).abs() <= 0.15 * t_pred + dt,
                "hop {hop}: flight time {t_meas}s vs ballistic {t_pred}s"
            );
        }
        let mean = apexes.iter().sum::<f32>() / apexes.len() as f32;
        assert!(
            (mean - target).abs() <= 0.08 * target,
            "mean apex {mean} does not fit target {target}: {apexes:?}"
        );

        // Δl is a bounded integrator, and the hip resets to stand between hops.
        assert!(
            state.vs.delta_l >= 0.0 && state.vs.delta_l < 4.0 * target,
            "Δl out of band: {}",
            state.vs.delta_l
        );
        assert!(
            (state.hip_pos.y - (cfg.floor_y + cfg.hip_height)).abs() < 8.0,
            "hip did not reset to stand: {}",
            state.hip_pos.y
        );
    }

    #[test]
    fn test_walk_jump_walk_full_episode_is_consistent() {
        // The whole episode — walking → jump → walking — as one headless trace,
        // asserting the 5 tracked points never violate their invariants and the
        // vertical numbers stay physical on every single tick.
        let cfg = LowerBodyConfig::default();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;

        let mut saw_flight = false;
        let mut max_h = 0.0f32;
        let mut landed_h = state.hip_pos.y;
        let mut flight_vx: Option<f32> = None;

        for frame in 0..4000 {
            // Jump once, at cruise, while still holding the walk input.
            let jump_now =
                state.vs.phase == VerticalPhase::Stand && !saw_flight && state.hip_vel.x > 100.0;
            let input = if jump_now {
                state.vs.jump_requested = true;
                Vec2::new(1.0, 1.0)
            } else {
                Vec2::X
            };

            let (pose, d) = step_lower_body(input, &cfg, &mut state, dt);

            // --- The 5 points: finite, on-bone, on-floor ---
            for p in [
                pose.hip,
                pose.left_knee,
                pose.left_foot,
                pose.right_knee,
                pose.right_foot,
            ] {
                assert!(
                    p.x.is_finite() && p.y.is_finite(),
                    "frame {frame}: NaN in pose: {p:?}"
                );
            }
            let lt = (pose.left_knee - pose.hip).length();
            let lc = (pose.left_foot - pose.left_knee).length();
            let rt = (pose.right_knee - pose.hip).length();
            let rc = (pose.right_foot - pose.right_knee).length();
            assert!(
                (lt - cfg.thigh_len).abs() < 0.05
                    && (lc - cfg.calf_len).abs() < 0.05
                    && (rt - cfg.thigh_len).abs() < 0.05
                    && (rc - cfg.calf_len).abs() < 0.05,
                "frame {frame}: bone length violated: L={lt}/{lc} R={rt}/{rc}"
            );
            assert!(
                pose.left_foot.y >= cfg.floor_y - 0.5 && pose.right_foot.y >= cfg.floor_y - 0.5,
                "frame {frame}: foot sank below the floor"
            );

            // --- Vertical numbers ---
            max_h = max_h.max(d.hip_height);
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
                match flight_vx {
                    None => flight_vx = Some(d.v_clamped),
                    Some(v0) => assert!(
                        (d.v_clamped - v0).abs() < 1e-3,
                        "frame {frame}: v_x changed in Flight: {v0} → {}",
                        d.v_clamped
                    ),
                }
            }
            if saw_flight && d.v_phase == VerticalPhase::Stand {
                landed_h = state.hip_pos.y;
                break;
            }
        }

        assert!(saw_flight, "episode never went airborne");
        // The apex must clear the target band even while walking.
        assert!(
            max_h - cfg.hip_height >= 0.7 * cfg.jump_apex,
            "jump apex too low: peak h={max_h}, stand={}",
            cfg.hip_height
        );
        // And the hip must settle back onto the stand.
        assert!(
            (landed_h - (cfg.floor_y + cfg.hip_height)).abs() < 8.0,
            "did not settle back to stand: h={landed_h}"
        );
        // The gait resumes and keeps advancing after the landing.
        for _ in 0..(stride / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        assert!(
            state.hip_vel.x > 100.0,
            "did not resume the gait after the jump: v_x={}",
            state.hip_vel.x
        );
    }

    #[test]
    fn test_walk_jump_walk_gait_resumes_and_progresses() {
        // After landing, the two legs must return to a valid, alternating gait
        // (never both Swing) and holding the input must keep the hip advancing.
        let cfg = LowerBodyConfig::default();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 3.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        let x_before = state.hip_pos.x;

        state.vs.jump_requested = true;
        // Ride through the whole hop, checking the gait gate while grounded.
        for _ in 0..2000 {
            let (_p, d) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
            if d.v_phase != VerticalPhase::Flight {
                assert!(
                    !(d.left_phase == GaitPhase::Swing && d.right_phase == GaitPhase::Swing),
                    "both feet swinging while grounded"
                );
            }
            if d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        // Resume walking and confirm forward progress with a valid alternation.
        for _ in 0..((stride * 2.0) / dt) as usize {
            let (_p, d) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
            assert!(
                !(d.left_phase == GaitPhase::Swing && d.right_phase == GaitPhase::Swing),
                "both feet swinging during post-jump walking"
            );
        }
        assert!(
            state.hip_pos.x > x_before + 20.0,
            "gait did not advance after the jump: {x_before} → {}",
            state.hip_pos.x
        );
    }

    #[test]
    fn test_walk_jump_walk_deterministic_replay() {
        // Pure math over the 5 points means identical inputs replay to identical
        // numbers — the property that makes these headless tests meaningful.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let approach = ((stride * 3.0) / dt) as usize;

        let run = || {
            let mut s = ResolverState::new(&cfg);
            let mut trace: Vec<[f32; 7]> = Vec::new();
            for i in 0..(approach + 600) {
                if i == approach {
                    s.vs.jump_requested = true;
                }
                let (_p, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
                trace.push([
                    s.hip_pos.x,
                    s.hip_pos.y,
                    d.v_clamped,
                    d.hip_vel_y,
                    d.delta_l,
                    d.h_apex,
                    d.hip_height,
                ]);
            }
            trace
        };

        let first = run();
        let second = run();
        assert_eq!(first.len(), second.len());
        for (frame, (a, b)) in first.iter().zip(second.iter()).enumerate() {
            assert_eq!(a, b, "frame {frame} diverged between identical runs");
        }
    }

    // --- State-machine transitions: the whole mode graph ---
    //
    // The two behaviours the character is built from are IDLE and WALK, plus the
    // JUMP action available from both. These tests walk every directed edge of
    // that graph — idle→walk, walk→idle, idle→jump, jump→idle, walk→jump,
    // jump→walk and their round trips — asserting the pose invariants on every
    // tick and the meaningful numeric outcome at each edge. A seeded fuzzer then
    // hammers the edges with random switch timing (reproducibly).

    /// Assert the 5-point pose and loop-1 state stay physical on one tick. This
    /// is the shared contract every transition must uphold:
    /// * all 5 points finite;
    /// * both legs exactly on-bone (IK ray-clamp guarantees this);
    /// * no foot ever sinks below the floor (the clamp only ever lifts);
    /// * never both feet airborne at once (the anti-phase gait gate);
    /// * vertical state finite and bounded.
    fn assert_pose_invariants(
        cfg: &LowerBodyConfig,
        pose: &LowerBodyPose,
        d: &DebugInfo,
        ctx: &str,
        frame: usize,
    ) {
        for p in [
            pose.hip,
            pose.left_knee,
            pose.left_foot,
            pose.right_knee,
            pose.right_foot,
        ] {
            assert!(
                p.x.is_finite() && p.y.is_finite(),
                "{ctx} frame {frame}: NaN in pose: {p:?}"
            );
        }
        let lt = (pose.left_knee - pose.hip).length();
        let lc = (pose.left_foot - pose.left_knee).length();
        let rt = (pose.right_knee - pose.hip).length();
        let rc = (pose.right_foot - pose.right_knee).length();
        assert!(
            (lt - cfg.thigh_len).abs() < 0.05
                && (lc - cfg.calf_len).abs() < 0.05
                && (rt - cfg.thigh_len).abs() < 0.05
                && (rc - cfg.calf_len).abs() < 0.05,
            "{ctx} frame {frame}: bone length violated: L={lt}/{lc} R={rt}/{rc}"
        );
        assert!(
            pose.left_foot.y >= cfg.floor_y - 0.5 && pose.right_foot.y >= cfg.floor_y - 0.5,
            "{ctx} frame {frame}: foot sank below the floor"
        );
        assert!(
            !(d.left_phase == GaitPhase::Swing && d.right_phase == GaitPhase::Swing),
            "{ctx} frame {frame}: both feet airborne at once"
        );
        assert!(
            d.delta_l.is_finite()
                && d.hip_height.is_finite()
                && d.hip_vel_y.is_finite()
                && d.delta_l < 100.0 * cfg.jump_apex,
            "{ctx} frame {frame}: loop-1 state out of band: Δl={}, h={}, ḣ={}",
            d.delta_l,
            d.hip_height,
            d.hip_vel_y
        );
    }

    /// Hold `walk`/`idle` input for `frames`, checking invariants every tick.
    fn drive(
        cfg: &LowerBodyConfig,
        state: &mut ResolverState,
        dt: f32,
        frames: usize,
        walk: bool,
        ctx: &str,
    ) {
        for frame in 0..frames {
            let input = if walk { Vec2::X } else { Vec2::ZERO };
            let (pose, d) = step_lower_body(input, cfg, state, dt);
            assert_pose_invariants(cfg, &pose, &d, ctx, frame);
        }
    }

    /// Request a jump, then step (holding `input`) until the vertical FSM settles
    /// back to `Stand` after going airborne. Returns `(saw_flight, max_rise,
    /// flight_ticks)` where `max_rise` is the peak measured height above stand.
    fn run_hop(
        cfg: &LowerBodyConfig,
        state: &mut ResolverState,
        dt: f32,
        input: Vec2,
        ctx: &str,
    ) -> (bool, f32, u32) {
        state.vs.jump_requested = true;
        let mut saw_flight = false;
        let mut max_rise = 0.0f32;
        let mut flight_ticks = 0u32;
        for frame in 0..4000 {
            let (pose, d) = step_lower_body(input, cfg, state, dt);
            assert_pose_invariants(cfg, &pose, &d, ctx, frame);
            max_rise = max_rise.max(d.hip_height - cfg.hip_height);
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
                flight_ticks += 1;
            }
            if saw_flight && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        (saw_flight, max_rise, flight_ticks)
    }

    #[test]
    fn test_transition_idle_to_walk() {
        // idle → walk: from rest the gait must break into a step and reach cruise.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;

        // Start at rest, gait frozen, both feet planted.
        drive(&cfg, &mut s, dt, (stride / dt) as usize, false, "idle-pre");
        assert!(
            s.left.phase == GaitPhase::Stance && s.right.phase == GaitPhase::Stance,
            "idle gait is not frozen"
        );
        let x0 = s.hip_pos.x;

        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 3.0) / dt) as usize,
            true,
            "walk",
        );
        assert!(
            s.hip_vel.x > 100.0,
            "did not reach cruise: v={}",
            s.hip_vel.x
        );
        assert!(
            s.hip_pos.x > x0 + 50.0,
            "did not advance: {x0} → {}",
            s.hip_pos.x
        );
    }

    #[test]
    fn test_transition_walk_to_idle() {
        // walk → idle: release the input and the hip must brake, settle between the
        // feet, and then stay put (no phantom steps).
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;

        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 3.0) / dt) as usize,
            true,
            "walk",
        );
        assert!(s.hip_vel.x > 100.0, "not cruising before the stop");

        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 4.0) / dt) as usize,
            false,
            "brake",
        );
        assert!(s.hip_vel.x.abs() < 8.0, "did not stop: v={}", s.hip_vel.x);
        let mid = (s.left.plant_pos.x + s.right.plant_pos.x) * 0.5;
        assert!(
            (s.hip_pos.x - mid).abs() < 8.0,
            "hip did not settle over the feet: hx={} mid={mid}",
            s.hip_pos.x
        );

        // Idle hold: no drift, both feet planted.
        let x1 = s.hip_pos.x;
        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 2.0) / dt) as usize,
            false,
            "idle-hold",
        );
        assert!(
            (s.hip_pos.x - x1).abs() < 3.0,
            "phantom drift while idle: {x1} → {}",
            s.hip_pos.x
        );
        assert!(
            s.left.phase == GaitPhase::Stance && s.right.phase == GaitPhase::Stance,
            "gait not frozen after stopping"
        );
    }

    #[test]
    fn test_transition_idle_to_jump() {
        // idle → jump: a standing jump must clear the target band and land back on
        // the stand with zero horizontal drift.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        drive(&cfg, &mut s, dt, (stride / dt) as usize, false, "idle-pre");
        let x0 = s.hip_pos.x;

        let (saw_flight, max_rise, _ft) = run_hop(&cfg, &mut s, dt, Vec2::ZERO, "idle-jump");
        assert!(saw_flight, "standing jump never left the ground");
        assert!(
            max_rise >= 0.7 * cfg.jump_apex,
            "standing jump too low: rise={max_rise}"
        );
        assert!(
            (s.hip_pos.y - (cfg.floor_y + cfg.hip_height)).abs() < 8.0,
            "did not return to the stand: {}",
            s.hip_pos.y
        );
        assert!(
            (s.hip_pos.x - x0).abs() < 2.0,
            "vertical jump drifted horizontally: {x0} → {}",
            s.hip_pos.x
        );
        assert!(s.vs.delta_l >= 0.0, "Δl went negative: {}", s.vs.delta_l);
    }

    #[test]
    fn test_transition_jump_to_idle() {
        // jump → idle: after an idle jump lands, the system must be a clean idle
        // again — both feet planted, gait frozen, no residual velocity.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let (_saw, _rise, _ft) = run_hop(&cfg, &mut s, dt, Vec2::ZERO, "jump-idle");
        // Let it fully settle, still idle.
        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 2.0) / dt) as usize,
            false,
            "idle-settle",
        );
        assert!(s.hip_vel.x.abs() < 5.0, "residual v after landing");
        assert!(
            s.left.phase == GaitPhase::Stance && s.right.phase == GaitPhase::Stance,
            "not an idle stance after the jump"
        );
    }

    #[test]
    fn test_transition_walk_to_jump() {
        // walk → jump: at cruise the jump must not perturb v_x (ballistic carry),
        // and the apex numbers must still fit while moving.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 3.0) / dt) as usize,
            true,
            "walk",
        );
        let v_before = s.hip_vel.x;
        assert!(v_before > 100.0, "not cruising before the jump");

        let (saw_flight, max_rise, _ft) = run_hop(&cfg, &mut s, dt, Vec2::X, "walk-jump");
        assert!(saw_flight, "walking jump never left the ground");
        assert!(
            max_rise >= 0.7 * cfg.jump_apex,
            "walking jump too low: rise={max_rise}"
        );
        assert!(
            (s.hip_vel.x - v_before).abs() < 0.15 * v_before,
            "v_x was perturbed by the jump: {v_before} → {}",
            s.hip_vel.x
        );
    }

    #[test]
    fn test_transition_jump_to_walk() {
        // jump → walk: an idle jump that lands and then receives walk input must
        // break straight into a cruise gait.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let (saw, _rise, _ft) = run_hop(&cfg, &mut s, dt, Vec2::ZERO, "jump");
        assert!(saw, "never went airborne");

        let x0 = s.hip_pos.x;
        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 3.0) / dt) as usize,
            true,
            "walk",
        );
        assert!(
            s.hip_vel.x > 100.0,
            "did not resume walking after the jump: v={}",
            s.hip_vel.x
        );
        assert!(
            s.hip_pos.x > x0 + 50.0,
            "did not advance after the jump: {x0} → {}",
            s.hip_pos.x
        );
    }

    #[test]
    fn test_transition_round_trips() {
        // The compound edges: idle→walk→idle, idle→jump→idle, and
        // walk→jump→walk, each asserted end-to-end on one state.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let n = |mults: f32| ((stride * mults) / dt) as usize;

        // idle → walk → idle
        let x_idle0 = s.hip_pos.x;
        drive(&cfg, &mut s, dt, n(3.0), true, "rt walk");
        assert!(s.hip_vel.x > 100.0, "rt: no cruise");
        drive(&cfg, &mut s, dt, n(4.0), false, "rt idle");
        assert!(s.hip_vel.x.abs() < 8.0, "rt: did not stop");
        assert!(s.hip_pos.x > x_idle0 + 50.0, "rt: no net travel");

        // idle → jump → idle
        let x_before_jump = s.hip_pos.x;
        let (saw, rise, _ft) = run_hop(&cfg, &mut s, dt, Vec2::ZERO, "rt jump");
        assert!(saw && rise >= 0.7 * cfg.jump_apex, "rt: bad idle jump");
        drive(&cfg, &mut s, dt, n(2.0), false, "rt idle2");
        assert!(
            (s.hip_pos.x - x_before_jump).abs() < 3.0,
            "rt: idle jump drifted"
        );

        // walk → jump → walk
        drive(&cfg, &mut s, dt, n(3.0), true, "rt walk2");
        let v_before = s.hip_vel.x;
        let (saw2, rise2, _ft2) = run_hop(&cfg, &mut s, dt, Vec2::X, "rt walkjump");
        assert!(saw2 && rise2 >= 0.7 * cfg.jump_apex, "rt: bad walking jump");
        drive(&cfg, &mut s, dt, n(2.0), true, "rt walk3");
        assert!(
            s.hip_vel.x > 100.0 && (s.hip_vel.x - v_before).abs() < 0.2 * v_before,
            "rt: walking gait not maintained across the jump: {v_before} → {}",
            s.hip_vel.x
        );
    }

    #[test]
    fn test_transition_chained_jumps_while_walking() {
        // walk → jump → walk → jump → walk … : repeated hops at cruise must each
        // fit the target and keep the horizontal speed steady (the walk→jump→walk
        // loop is stable, not just a one-shot).
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let g = cfg.gravity;
        let quantum = apex_to_launch_speed(cfg.jump_apex, g) * (cfg.launch_accel * dt) / g;

        drive(
            &cfg,
            &mut s,
            dt,
            ((stride * 3.0) / dt) as usize,
            true,
            "walk",
        );
        let v_cruise = s.hip_vel.x;
        assert!(v_cruise > 100.0, "not cruising");

        for cycle in 0..5 {
            let (saw, rise, _ft) = run_hop(&cfg, &mut s, dt, Vec2::X, &format!("chain {cycle}"));
            assert!(saw, "cycle {cycle}: never airborne");
            assert!(
                (rise - cfg.jump_apex).abs() <= quantum + 2.0,
                "cycle {cycle}: apex {rise} outside one launch-quantum of {}",
                cfg.jump_apex
            );
            // Keep walking between hops.
            drive(
                &cfg,
                &mut s,
                dt,
                ((stride * 2.0) / dt) as usize,
                true,
                "walk-between",
            );
            assert!(
                (s.hip_vel.x - v_cruise).abs() < 0.2 * v_cruise,
                "cycle {cycle}: cruise speed drifted: {v_cruise} → {}",
                s.hip_vel.x
            );
        }
        assert!(
            s.vs.delta_l >= 0.0 && s.vs.delta_l < 4.0 * cfg.jump_apex,
            "Δl out of band after chaining: {}",
            s.vs.delta_l
        );
    }

    /// A tiny deterministic LCG so randomized switch-timing tests are fully
    /// reproducible (same seed → same schedule → same numbers). This is what lets
    /// randomization live in a pure-math unit test without flakiness.
    struct Lcg(u64);
    impl Lcg {
        fn new(seed: u64) -> Self {
            Self(seed ^ 0x9E37_79B9_7F4A_7C15)
        }
        fn next_u32(&mut self) -> u32 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 33) as u32
        }
        /// True with probability `1/n`.
        fn chance(&mut self, n: u32) -> bool {
            self.next_u32() % n.max(1) == 0
        }
        /// A frame count in `[lo, hi]`.
        fn span(&mut self, lo: u32, hi: u32) -> u32 {
            lo + self.next_u32() % (hi.saturating_sub(lo).max(1))
        }
    }

    /// Drive a random schedule of idle/walk toggles and jump requests, returning a
    /// compact trace of the numbers for the determinism check. Invariants are
    /// asserted on every tick.
    fn fuzz_schedule(seed: u64) -> Vec<[f32; 6]> {
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let mut rng = Lcg::new(seed);
        let mut s = ResolverState::new(&cfg);
        let mut walk = false;
        let mut trace = Vec::with_capacity(9000);
        // Random hold lengths keep switches landing at arbitrary gait phases.
        let mut walk_hold = rng.span(1, 120);
        let mut jump_hold = rng.span(1, 240);

        for frame in 0..9000 {
            if walk_hold == 0 {
                walk = !walk;
                walk_hold = rng.span(1, 120);
            }
            walk_hold -= 1;
            if jump_hold == 0 {
                s.vs.jump_requested = true;
                jump_hold = rng.span(30, 240);
            }
            jump_hold -= 1;

            let input = if walk { Vec2::X } else { Vec2::ZERO };
            let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, &format!("fuzz seed {seed}"), frame);
            // Global boundedness: nothing may run away under arbitrary switching.
            assert!(
                s.hip_vel.x.abs() < 1.0e4 && d.hip_vel_y.abs() < 1.0e4,
                "seed {seed} frame {frame}: velocity ran away: vx={} vy={}",
                s.hip_vel.x,
                d.hip_vel_y
            );
            trace.push([
                s.hip_pos.x,
                s.hip_pos.y,
                d.v_clamped,
                d.hip_vel_y,
                d.delta_l,
                d.hip_height,
            ]);
        }
        trace
    }

    #[test]
    fn test_transition_random_switch_timing() {
        // Randomize the timing of every state switch (idle↔walk toggles and jump
        // requests) across many seeds. The point is not one trajectory but that NO
        // schedule — including switches that land mid-stride, mid-crouch or
        // mid-air — can violate the pose invariants or drive the loop unbounded.
        for seed in 1..=12 {
            let trace = fuzz_schedule(seed);
            assert_eq!(trace.len(), 9000, "seed {seed}: short trace");
        }
    }

    #[test]
    fn test_transition_random_switch_timing_is_deterministic() {
        // Reproducibility of the fuzzer: the same seed must yield byte-identical
        // numbers, so a failure can always be replayed exactly.
        for seed in [7u64, 1234, 999_999] {
            let a = fuzz_schedule(seed);
            let b = fuzz_schedule(seed);
            assert_eq!(a.len(), b.len(), "seed {seed}: length mismatch");
            for (frame, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                assert_eq!(x, y, "seed {seed} frame {frame}: nondeterministic");
            }
        }
        // And different seeds must actually explore different schedules.
        assert_ne!(
            fuzz_schedule(1),
            fuzz_schedule(2),
            "the fuzzer is not varying its schedule"
        );
    }

    // --- Design-contract probes ---
    //
    // These encode what `plans/raibert_vertical_loop.md` claims the loop DOES, so a
    // refactor that silently changes the behaviour is caught here rather than only
    // once it is ported onto the physics body.

    #[test]
    fn test_design_apex_fits_target_across_amplitude_sweep() {
        // §4: the commanded apex must actually be reached (within one launch
        // quantum) for every target the leg has the stroke to buy. This is the
        // core numeric contract — "the number fits".
        let dt = 1.0 / 60.0;
        for &target in &[20.0f32, 30.0, 40.0, 50.0, 60.0] {
            let mut cfg = LowerBodyConfig::default();
            cfg.jump_apex = target;
            let g = cfg.gravity;
            let quantum = apex_to_launch_speed(target, g) * (cfg.launch_accel * dt) / g;
            let mut s = ResolverState::new(&cfg);
            let mut apexes = Vec::new();
            for _ in 0..8 {
                let (apex, _ft) = run_hop_measured(&cfg, &mut s, dt);
                apexes.push(apex);
            }
            let mean = apexes.iter().sum::<f32>() / apexes.len() as f32;
            assert!(
                (mean - target).abs() <= 0.12 * target + quantum,
                "target {target}: mean apex {mean} off ({apexes:?})"
            );
            assert!(
                s.vs.delta_l.is_finite() && s.vs.delta_l.abs() < 4.0 * target,
                "target {target}: Δl out of band: {}",
                s.vs.delta_l
            );
        }
    }

    #[test]
    fn test_design_no_model_fitting_gravity_sweep() {
        // §4A: the loop holds "no mass estimate, no stiffness estimate, no leg
        // model" — only measured state and g. Changing g must not break
        // convergence: the same one-line law still reaches the target.
        let dt = 1.0 / 60.0;
        for &g in &[490.0f32, 980.0, 1960.0] {
            let mut cfg = LowerBodyConfig::default();
            cfg.gravity = g;
            cfg.jump_apex = 30.0;
            let mut s = ResolverState::new(&cfg);
            let mut apexes = Vec::new();
            for _ in 0..10 {
                let (apex, _ft) = run_hop_measured(&cfg, &mut s, dt);
                apexes.push(apex);
            }
            let last = &apexes[apexes.len() - 4..];
            let mean = last.iter().sum::<f32>() / last.len() as f32;
            assert!(
                (mean - cfg.jump_apex).abs() <= 0.25 * cfg.jump_apex,
                "g={g}: apex not reached: mean={mean} ({apexes:?})"
            );
            assert!(
                s.vs.delta_l.is_finite() && s.vs.delta_l.abs() < 200.0,
                "g={g}: Δl diverged: {}",
                s.vs.delta_l
            );
        }
    }

    #[test]
    fn test_design_unreachable_target_saturates_without_diverging() {
        // A target beyond what the stroke can buy must SATURATE: the crouch clamps
        // at the mechanical stop, the apex plateaus, and the PI integrator must not
        // wind up past the stop (that would be the divergence bug in disguise).
        let cfg = {
            let mut c = LowerBodyConfig::default();
            c.jump_apex = 200.0;
            c
        };
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stop = cfg.hip_height - cfg.crouch_height; // 35 px mechanical stop
        let mut apexes = Vec::new();
        for hop in 0..15 {
            let (apex, _ft) = run_hop_measured(&cfg, &mut s, dt);
            apexes.push(apex);
            assert!(apex.is_finite(), "hop {hop}: non-finite apex");
        }
        let last = &apexes[apexes.len() - 5..];
        let mx = last.iter().cloned().fold(f32::MIN, f32::max);
        let mn = last.iter().cloned().fold(f32::MAX, f32::min);
        assert!(mx < cfg.jump_apex, "unreachable target reached: {apexes:?}");
        assert!(mx - mn < 5.0, "apex did not plateau: {last:?}");
        assert!(
            s.vs.delta_l <= stop + 1.0,
            "Δl wound past the crouch stop: {} > {stop}",
            s.vs.delta_l
        );
    }

    #[test]
    fn test_design_controller_has_no_airborne_authority() {
        // §3.1 / anti-hover: the controller must never ADD upward velocity while
        // airborne. On every Flight tick ḣ must strictly decrease (gravity only).
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        s.vs.jump_requested = true;
        let mut prev: Option<f32> = None;
        let mut saw = false;
        for _ in 0..3000 {
            let (_p, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            if d.v_phase == VerticalPhase::Flight {
                saw = true;
                if let Some(p) = prev {
                    assert!(
                        d.hip_vel_y < p,
                        "controller added height in the air: {p} → {}",
                        d.hip_vel_y
                    );
                }
                prev = Some(d.hip_vel_y);
            } else {
                prev = None;
            }
            if saw && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw, "never flew");
    }

    #[test]
    fn test_design_vertical_phase_order_is_respected() {
        // §5: the FSM must traverse Stand→Load→Launch→Flight→Land→Stand in order —
        // never skipping a phase and never reversing (the one legal backward edge
        // is the launch-window abort Load→Stand).
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        s.vs.jump_requested = true;
        let mut seen: Vec<VerticalPhase> = vec![VerticalPhase::Stand];
        let mut last = VerticalPhase::Stand;
        for _ in 0..3000 {
            let (_p, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            if d.v_phase != last {
                let ok = matches!(
                    (last, d.v_phase),
                    (VerticalPhase::Stand, VerticalPhase::Load)
                        | (VerticalPhase::Load, VerticalPhase::Launch)
                        | (VerticalPhase::Load, VerticalPhase::Stand)
                        | (VerticalPhase::Launch, VerticalPhase::Flight)
                        | (VerticalPhase::Flight, VerticalPhase::Land)
                        | (VerticalPhase::Land, VerticalPhase::Stand)
                );
                assert!(ok, "illegal vertical transition {last:?} → {:?}", d.v_phase);
                seen.push(d.v_phase);
                last = d.v_phase;
            }
            if last == VerticalPhase::Stand && seen.len() > 1 {
                break;
            }
        }
        for need in [
            VerticalPhase::Load,
            VerticalPhase::Launch,
            VerticalPhase::Flight,
            VerticalPhase::Land,
        ] {
            assert!(seen.contains(&need), "never entered {need:?}: {seen:?}");
        }
    }

    #[test]
    fn test_design_tolerates_variable_dt() {
        // Porting reality: the frame dt is not fixed. A PAL-style ±40 % jitter
        // around 1/60 must not destabilize the loop — the apex still fits, the
        // invariants still hold, and the hop still lands.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let mut rng = Lcg::new(0xDEAD_BEEF);
        s.vs.jump_requested = true;
        let mut saw = false;
        let mut max_h = 0.0f32;
        for frame in 0..6000 {
            // dt ∈ [0.0100, 0.0200] (~1/60 ± 20 % in time → the physics rate jitters).
            let dt = 0.0100 + (rng.next_u32() % 100) as f32 * 0.0001;
            let (pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "vdt", frame);
            max_h = max_h.max(d.hip_height);
            if d.v_phase == VerticalPhase::Flight {
                saw = true;
            }
            if saw && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw, "variable-dt loop never flew");
        assert!(
            max_h - cfg.hip_height >= 0.6 * cfg.jump_apex,
            "variable-dt jump too low: rise={}",
            max_h - cfg.hip_height
        );
        assert!(
            (s.hip_pos.y - (cfg.floor_y + cfg.hip_height)).abs() < 10.0,
            "variable-dt hop did not settle: {}",
            s.hip_pos.y
        );
    }

    #[test]
    fn test_design_crouch_mode_still_jumps() {
        // With `crouch = true` the stand height is `crouch_height`, so the loop
        // must settle low and still buy the full transition to
        // `crouch_height + jump_apex`. Exercises the `stand` selector and the
        // `crouch_height.min(stand)` Load clamp.
        let mut cfg = LowerBodyConfig::default();
        cfg.crouch = true;
        let stand = cfg.crouch_height; // 55
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        s.vs.jump_requested = true;

        let mut saw = false;
        let mut max_h = 0.0f32;
        for frame in 0..6000 {
            let (pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "crouch", frame);
            max_h = max_h.max(d.hip_height);
            if d.v_phase == VerticalPhase::Flight {
                saw = true;
            }
            if saw && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw, "crouch-mode jump never flew");
        assert!(
            max_h >= stand + 0.5 * cfg.jump_apex,
            "crouch-mode apex too low: peak={max_h}, stand={stand}"
        );
        assert!(
            (s.hip_pos.y - (cfg.floor_y + stand)).abs() < 8.0,
            "crouch-mode did not settle to stand: {} (want {})",
            s.hip_pos.y,
            cfg.floor_y + stand
        );
    }

    #[test]
    fn test_design_nonzero_floor_offset() {
        // The whole loop must be translation-invariant: a floor at y = 100 has to
        // behave exactly like y = 0 (all heights are measured above `floor_y`).
        let mut cfg = LowerBodyConfig::default();
        cfg.floor_y = 100.0;
        let base = cfg.floor_y + cfg.hip_height; // 190
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        s.vs.jump_requested = true;

        let mut saw = false;
        let mut max_h = 0.0f32;
        for frame in 0..6000 {
            let (pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "floor", frame);
            max_h = max_h.max(d.hip_height);
            if d.v_phase == VerticalPhase::Flight {
                saw = true;
            }
            if saw && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw, "floor-offset jump never flew");
        assert!(
            max_h - cfg.hip_height >= 0.6 * cfg.jump_apex,
            "floor-offset jump too low: rise={}",
            max_h - cfg.hip_height
        );
        assert!(
            (s.hip_pos.y - base).abs() < 8.0,
            "did not settle at the offset stand: {} (want {base})",
            s.hip_pos.y
        );
    }

    /// Fuzz with random jump requests AND per-tick dt jitter; returns a trace for
    /// the determinism check. Invariants asserted every tick.
    fn fuzz_jitter(seed: u64) -> Vec<[f32; 6]> {
        let cfg = LowerBodyConfig::default();
        let mut rng = Lcg::new(seed);
        let mut s = ResolverState::new(&cfg);
        let mut walk = false;
        let mut trace = Vec::with_capacity(8000);
        let mut walk_hold = rng.span(1, 90);
        let mut jump_hold = rng.span(1, 150);

        for frame in 0..8000 {
            if walk_hold == 0 {
                walk = !walk;
                walk_hold = rng.span(1, 90);
            }
            walk_hold -= 1;
            if jump_hold == 0 {
                s.vs.jump_requested = true;
                jump_hold = rng.span(20, 180);
            }
            jump_hold -= 1;
            let dt = 0.0100 + (rng.next_u32() % 100) as f32 * 0.0001; // 0.01..0.02

            let input = if walk { Vec2::X } else { Vec2::ZERO };
            let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, &format!("fuzz-jitter {seed}"), frame);
            assert!(
                s.hip_vel.x.abs() < 1.0e4 && d.hip_vel_y.abs() < 1.0e4,
                "seed {seed} frame {frame}: runaway vx={} vy={}",
                s.hip_vel.x,
                d.hip_vel_y
            );
            trace.push([
                s.hip_pos.x,
                s.hip_pos.y,
                d.v_clamped,
                d.hip_vel_y,
                d.delta_l,
                d.hip_height,
            ]);
        }
        trace
    }

    #[test]
    fn test_fuzz_jitter_is_stable_and_deterministic() {
        // Random dt + random switching, across seeds: never panics, never
        // diverges, and replays identically for a given seed.
        for seed in 1..=6 {
            let a = fuzz_jitter(seed);
            let b = fuzz_jitter(seed);
            assert_eq!(a.len(), b.len(), "seed {seed}: length");
            for (frame, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                assert_eq!(
                    x, y,
                    "seed {seed} frame {frame}: nondeterministic under jitter"
                );
            }
        }
    }

    #[test]
    fn test_design_launch_full_extension_branch() {
        // §4.1 fallback: if the hip reaches full extension with ḣ still short of
        // the cutoff, Launch must still release to Flight (a shallow-crouch
        // saturation). `jump_apex = 200` guarantees the cutoff is unreachable, so
        // the ONLY way out of Launch is the full-extension branch.
        let cfg = {
            let mut c = LowerBodyConfig::default();
            c.jump_apex = 200.0;
            c
        };
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        s.vs.jump_requested = true;
        let full_ext = cfg.thigh_len + cfg.calf_len - 2.0;
        let mut max_h = 0.0f32;
        let mut released = false;
        for _ in 0..4000 {
            let (_p, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            max_h = max_h.max(d.hip_height);
            if d.v_phase == VerticalPhase::Flight {
                released = true;
                break;
            }
            assert!(
                d.v_phase != VerticalPhase::Land,
                "locked in Load/Launch without ever flying"
            );
        }
        assert!(released, "never left Launch for an unreachable target");
        assert!(
            max_h >= full_ext - 4.0,
            "released short of full extension: {max_h} vs {full_ext}"
        );
    }

    #[test]
    fn test_design_launch_commit_latency_is_measured() {
        // §5.7 says a jump intent is *buffered* and committed at the next launch
        // window. Measure the real latency: request a jump at the worst moment
        // (mid-swing) and record how many ticks elapse before Load is entered.
        // This pins the current behaviour; a change to the window policy trips it.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        // Walk until exactly one leg is mid-swing.
        let mut ticks = 0usize;
        while !(s.left.phase == GaitPhase::Swing || s.right.phase == GaitPhase::Swing) {
            step_lower_body(Vec2::X, &cfg, &mut s, dt);
            ticks += 1;
            assert!(
                ticks < (stride * 4.0 / dt) as usize,
                "never got a swing leg"
            );
        }
        s.vs.jump_requested = true;
        let mut latency = 0usize;
        for _ in 0..2000 {
            let (_p, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            if d.v_phase == VerticalPhase::Load {
                break;
            }
            latency += 1;
        }
        // The intent never drops: it must commit within a couple of strides.
        assert!(
            latency < (stride * 3.0 / dt) as usize,
            "jump intent was dropped (latency {latency} ticks)"
        );
    }

    // --- Sudden stop from an arbitrary phase ---
    //
    // The user's hypothesis: a sudden stop (release the move input) must "finish
    // the last walking/jumping circle and then stand still" and it must not
    // matter WHERE in the cycle the stop is triggered. These sweeps slide the
    // release frame across a whole stride (walk) and a whole hop (jump) and
    // assert the SAME settled end-state every time: the in-progress step/jump
    // completes (never frozen mid-swing / never cancelled mid-air), speed reaches
    // zero, both feet are planted, and nothing drifts or re-steps afterward.

    #[test]
    fn test_stop_finishes_walk_cycle_from_any_phase() {
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let stride_frames = (stride / dt).round() as usize;
        let cruise_frames = (stride * 3.0 / dt) as usize;
        let settle_budget = (stride * 8.0 / dt) as usize;

        let mut worst_settle = 0usize;
        let mut stopped_mid_swing = 0usize;
        for k in (0..stride_frames).step_by(2) {
            let ctx = format!("walk-stop k={k}");
            let mut s = ResolverState::new(&cfg);
            // Cruise to a steady gait …
            drive(&cfg, &mut s, dt, cruise_frames, true, &ctx);
            assert!(
                s.hip_vel.x > 50.0,
                "{ctx}: not at cruise before the stop (v={})",
                s.hip_vel.x
            );
            if s.left.phase == GaitPhase::Swing || s.right.phase == GaitPhase::Swing {
                stopped_mid_swing += 1;
            }

            // … walk `k` more frames (sliding the stop across the stride) …
            let mut settle = None;
            for frame in 0..(k + settle_budget) {
                let input = if frame < k { Vec2::X } else { Vec2::ZERO };
                let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
                assert_pose_invariants(&cfg, &pose, &d, &ctx, frame);
                if frame >= k
                    && s.hip_vel.x.abs() < 5.0
                    && s.left.phase == GaitPhase::Stance
                    && s.right.phase == GaitPhase::Stance
                {
                    settle = Some(frame - k);
                    break;
                }
            }
            let settle = settle.unwrap_or_else(|| panic!("{ctx}: never reached a standstill"));
            worst_settle = worst_settle.max(settle);

            // … and once settled it must stay put: no phantom steps, no drift.
            let x_settled = s.hip_pos.x;
            let (l_plant, r_plant) = (s.left.plant_pos, s.right.plant_pos);
            drive(&cfg, &mut s, dt, (2.0 / dt) as usize, false, &ctx);
            assert!(
                s.left.phase == GaitPhase::Stance && s.right.phase == GaitPhase::Stance,
                "{ctx}: stepping resumed after the standstill"
            );
            assert!(
                (s.hip_pos.x - x_settled).abs() < 4.0,
                "{ctx}: drifted after the standstill: {x_settled} -> {}",
                s.hip_pos.x
            );
            assert!(
                (s.left.plant_pos - l_plant).length() < 1e-3
                    && (s.right.plant_pos - r_plant).length() < 1e-3,
                "{ctx}: a foot re-planted with no input"
            );
        }
        // Every stop must settle promptly, and the sweep must have actually hit
        // the mid-swing case (otherwise the test is not probing the hard phase).
        assert!(
            worst_settle < settle_budget,
            "slowest stop settle was {worst_settle} frames (budget {settle_budget})"
        );
        assert!(
            stopped_mid_swing > 0,
            "the sweep never triggered a stop during swing"
        );
    }

    #[test]
    fn test_stop_finishes_jump_cycle_from_any_phase() {
        // Releasing the move input at any point of a hop — including mid-Load,
        // mid-Launch and mid-Flight — must NOT cancel a committed jump: it still
        // goes airborne and settles to a standstill on landing.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let cruise_frames = (stride * 3.0 / dt) as usize;
        let settle_budget = (stride * 16.0 / dt) as usize;
        // The sweep spans the whole hop (Load → Launch → Flight → Land).
        let hop_frames = 120usize;

        let mut released_mid_air = 0usize;
        for k in (0..hop_frames).step_by(3) {
            let ctx = format!("jump-stop k={k}");
            let mut s = ResolverState::new(&cfg);
            drive(&cfg, &mut s, dt, cruise_frames, true, &ctx);

            s.vs.jump_requested = true;
            let mut saw_flight = false;
            let mut settled = false;
            for frame in 0..(k + settle_budget) {
                let input = if frame < k { Vec2::X } else { Vec2::ZERO };
                let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
                assert_pose_invariants(&cfg, &pose, &d, &ctx, frame);
                if d.v_phase == VerticalPhase::Flight {
                    saw_flight = true;
                    if frame >= k {
                        released_mid_air += 1;
                    }
                }
                if frame >= k
                    && saw_flight
                    && d.v_phase == VerticalPhase::Stand
                    && s.hip_vel.x.abs() < 5.0
                    && s.left.phase == GaitPhase::Stance
                    && s.right.phase == GaitPhase::Stance
                {
                    settled = true;
                    break;
                }
            }
            assert!(
                saw_flight,
                "{ctx}: releasing the move input cancelled the jump (never flew)"
            );
            assert!(settled, "{ctx}: never reached a standstill after the jump");
        }
        // The sweep must have exercised a release while already airborne.
        assert!(
            released_mid_air > 0,
            "the sweep never released the input during Flight"
        );
    }

    #[test]
    fn test_stop_from_random_phase_always_settles() {
        // Reproducible fuzz of the stop phase: cruise, then release after a random
        // number of extra frames, optionally requesting a jump at the same moment.
        // Whatever the phase, the character must finish the cycle and stand still.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let cruise_frames = (stride * 3.0 / dt) as usize;
        let settle_budget = (stride * 20.0 / dt) as usize;
        let mut rng = Lcg::new(0x00C0_FFEE);

        for trial in 0..24u32 {
            let ctx = format!("stop-fuzz trial {trial}");
            let mut s = ResolverState::new(&cfg);
            drive(&cfg, &mut s, dt, cruise_frames, true, &ctx);

            let k = rng.span(0, (stride / dt) as u32);
            let jump = rng.chance(3);
            if jump {
                s.vs.jump_requested = true;
            }

            let mut saw_flight = false;
            let mut settled = false;
            for frame in 0..(k as usize + settle_budget) {
                let input = if (frame as u32) < k {
                    Vec2::X
                } else {
                    Vec2::ZERO
                };
                let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
                assert_pose_invariants(&cfg, &pose, &d, &ctx, frame);
                if d.v_phase == VerticalPhase::Flight {
                    saw_flight = true;
                }
                if (frame as u32) >= k
                    && s.hip_vel.x.abs() < 5.0
                    && s.left.phase == GaitPhase::Stance
                    && s.right.phase == GaitPhase::Stance
                    && (!jump || (saw_flight && d.v_phase == VerticalPhase::Stand))
                {
                    settled = true;
                    break;
                }
            }
            assert!(
                settled,
                "{ctx}: never settled (jump={jump}, k={k}, v={})",
                s.hip_vel.x
            );
        }
    }

    // --- The "middle" of a walk↔jump edge: an idle, or not? ---
    //
    // Design §5.7/§5.8 answer this precisely, and it is worth pinning as a test
    // because the intuitive answer (a middle idle) is the WRONG one:
    //
    // * walk → jump does NOT insert an idle/stop. "The horizontal loop keeps
    //   running … the launch is `Stance-start → Launch` with `v_x` continuous;
    //   only the height reference changes." Freezing `v_x` through the window was
    //   explicitly withdrawn (§5.7). The only "middle" is the *vertical* FSM
    //   passing through `Stand → Load → Launch → Flight → Land → Stand` while the
    //   gait never stops.
    // * idle → jump is a different edge: from a true standstill the physical
    //   "middle" is the `Load` crouch — it banks displacement in height only, so
    //   the body must NOT translate during it.
    //
    // These three tests treat that as the contract.

    /// Append `p` only if it differs from the previous entry, so the vector holds
    /// the *ordered distinct* phase sequence (consecutive repeats collapsed).
    fn push_phase(seq: &mut Vec<VerticalPhase>, p: VerticalPhase) {
        if seq.last() != Some(&p) {
            seq.push(p);
        }
    }

    /// Build a state cruised to steady speed with move input held.
    fn walk_state(cfg: &LowerBodyConfig, dt: f32) -> ResolverState {
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let mut s = ResolverState::new(cfg);
        drive(
            cfg,
            &mut s,
            dt,
            ((stride * 3.0) / dt) as usize,
            true,
            "cruise",
        );
        s
    }

    #[test]
    fn test_walk_to_jump_phase_order() {
        // The vertical phase sequence of a walking jump is fixed, and — per
        // §5.7 — there is NO idle interposed: it is `Stance-start → Launch`
        // (here expressed as Stand → Load → Launch → Flight → Land → Stand)
        // while the horizontal gait keeps running.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let mut s = walk_state(&cfg, dt);
        assert!(s.hip_vel.x > 100.0, "not cruising before the jump");

        s.vs.jump_requested = true;
        let mut seq = vec![s.vs.phase];
        let mut saw_flight = false;
        for frame in 0..4000 {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "walk-jump", frame);
            push_phase(&mut seq, d.v_phase);
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
            }
            if saw_flight && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw_flight, "never went airborne");
        assert_eq!(
            seq,
            vec![
                VerticalPhase::Stand,
                VerticalPhase::Load,
                VerticalPhase::Launch,
                VerticalPhase::Flight,
                VerticalPhase::Land,
                VerticalPhase::Stand,
            ],
            "walk→jump vertical phase order changed: {seq:?}"
        );
    }

    #[test]
    fn test_walk_to_jump_preserves_vx_design() {
        // §5.7/§5.8 claim: "the horizontal loop keeps running … the launch is
        // `Stance-start → Launch` with `v_x` continuous; only the height
        // reference changes." Freezing `v_x` was explicitly withdrawn (§5.7).
        // So |v_x| must stay a large fraction of cruise for the WHOLE transition.
        //
        // RESOLVED (2026-10-14). This was an `#[ignore]`d known deviation: a
        // walk→jump committed mid-swing used to ring `v_x` (120 → 4.6 → 120 → …)
        // as touchdown backward-solves fired DURING the jump and the
        // two-foot-landing follower planted *behind* the hip. The "legs lag
        // behind" fix (envelope evaluated at the GROUND reference height
        // `h_plan`, `step_lower_body` §2) removed that: the horizontal gait now
        // keeps its normal swing through the hop, no in-jump touchdown fires, and
        // `v_x` stays at cruise. Un-ignored and green (8/8 runs at dt = 1/60).
        // Per-frame continuity — no sudden dv_x without a touchdown source — is
        // asserted by `test_jump_to_walk_vx_continuity`.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let mut s = walk_state(&cfg, dt);
        let v_cruise = s.hip_vel.x;

        s.vs.jump_requested = true;
        let mut v_min = f32::INFINITY;
        let mut v_min_phase = VerticalPhase::Stand;
        let mut v_min_support = 0.0f32;
        let mut saw_flight = false;
        for frame in 0..4000 {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "walk-jump", frame);
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
            }
            let vx = s.hip_vel.x.abs();
            if vx < v_min {
                v_min = vx;
                v_min_phase = d.v_phase;
                v_min_support = d.v_support;
            }
            if saw_flight && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw_flight, "never went airborne");
        assert!(
            v_min > 0.5 * v_cruise,
            "walking jump did not preserve v_x (§5.7): min |v_x|={v_min} at {v_min_phase:?} \
             (touchdown v_support={v_min_support}), cruise={v_cruise}"
        );
    }

    #[test]
    fn test_jump_to_walk_has_no_middle_idle() {
        // The reverse edge: an idle hop lands (Land → Stand) and then walk input
        // must break straight into the gait. The vertical phase must be `Stand` at
        // the hand-off and stay `Stand` — no accidental re-jump — while the
        // character accelerates from the standstill.
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;

        let (saw, _rise, _ft) = run_hop(&cfg, &mut s, dt, Vec2::ZERO, "idle-hop");
        assert!(saw, "never went airborne");
        assert!(
            s.vs.phase == VerticalPhase::Stand,
            "did not settle to Stand after the hop: {:?}",
            s.vs.phase
        );
        let x0 = s.hip_pos.x;

        for frame in 0..((stride * 3.0) / dt) as usize {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "jump-walk", frame);
            assert_eq!(
                d.v_phase,
                VerticalPhase::Stand,
                "re-entered the jump while transitioning to walk"
            );
        }
        assert!(s.hip_vel.x > 100.0, "did not resume walking");
        assert!(s.hip_pos.x > x0 + 50.0, "did not advance after the hop");
    }

    #[test]
    fn test_idle_to_jump_crouch_does_not_translate() {
        // The true idle→jump edge. From a standstill the ordered sequence is
        // Stand → Load → Launch → Flight → Land → Stand, and the `Load` crouch —
        // the physical "middle" this edge *does* have — must bank displacement in
        // height only, never translating the body (no phantom step).
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        drive(&cfg, &mut s, dt, (1.0 / dt) as usize, false, "rest");
        assert!(s.hip_vel.x.abs() < 1.0, "not at rest");
        let x0 = s.hip_pos.x;

        let mut seq = vec![s.vs.phase];
        let mut crouch_x_min = f32::INFINITY;
        let mut crouch_x_max = f32::NEG_INFINITY;
        let mut saw_flight = false;
        s.vs.jump_requested = true;
        for frame in 0..4000 {
            let (pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "idle-jump", frame);
            push_phase(&mut seq, d.v_phase);
            if d.v_phase == VerticalPhase::Load {
                crouch_x_min = crouch_x_min.min(s.hip_pos.x);
                crouch_x_max = crouch_x_max.max(s.hip_pos.x);
            }
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
            }
            if saw_flight && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw_flight, "idle jump never flew");
        assert_eq!(
            seq,
            vec![
                VerticalPhase::Stand,
                VerticalPhase::Load,
                VerticalPhase::Launch,
                VerticalPhase::Flight,
                VerticalPhase::Land,
                VerticalPhase::Stand,
            ],
            "idle→jump vertical phase order changed: {seq:?}"
        );
        assert!(
            crouch_x_max - crouch_x_min < 1.0,
            "the crouch translated the body: {crouch_x_min} … {crouch_x_max}"
        );
        assert!(
            (s.hip_pos.x - x0).abs() < 3.0,
            "idle jump drifted: {x0} → {}",
            s.hip_pos.x
        );
        assert!(
            s.hip_vel.x.abs() < 5.0,
            "not at a standstill after the idle jump: v={}",
            s.hip_vel.x
        );
    }

    #[test]
    fn test_jump_to_walk_symmetric_no_vx_hidden_stop() {
        // Symmetry probe for the reverse edge. The walk→jump finding was an
        // in-jump touchdown solve that damped `v_x`. Here we check the mirror:
        // does landing (and the resume into walk) inject an analogous *hidden*
        // horizontal stop, or reverse `v_x`? Holding walk input through the hop,
        // we require (a) landing does not silently floor the carried speed,
        // (b) `v_x` never goes backward during Land/resume, and (c) walk speed
        // recovers. (Measured: `v_x` rises monotonically 30 → 120 through Land,
        // `v_support = 0` — the landing is clean, unlike the jump-in edge.)
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let mut s = walk_state(&cfg, dt);
        let v_cruise = s.hip_vel.x;

        s.vs.jump_requested = true;
        let mut saw_flight = false;
        let mut land_seen = false;
        let mut v_carry = 0.0f32; // last |v_x| while airborne (momentum carried)
        let mut v_min_land = f32::INFINITY;
        let mut v_min_signed = f32::INFINITY;
        let mut land_frames = 0usize;
        for _ in 0..4000 {
            let (_p, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            match d.v_phase {
                VerticalPhase::Flight => {
                    saw_flight = true;
                    v_carry = s.hip_vel.x.abs();
                }
                VerticalPhase::Land => {
                    land_seen = true;
                    land_frames += 1;
                    v_min_land = v_min_land.min(s.hip_vel.x.abs());
                }
                _ => {}
            }
            if land_seen {
                v_min_signed = v_min_signed.min(s.hip_vel.x);
            }
            if saw_flight && land_seen && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw_flight && land_seen, "never flew and/or never landed");
        // (a) Landing must not silently floor the carried speed to a standstill.
        assert!(
            v_min_land > 0.4 * v_carry.max(1.0),
            "landing killed the carried v_x: carry={v_carry:.2}, min |v_x| during Land=\
             {v_min_land:.2} over {land_frames} land frames"
        );
        // (b) No backward stutter on touchdown.
        assert!(
            v_min_signed > -1.0,
            "v_x reversed during landing/resume: min signed v_x={v_min_signed:.2}"
        );

        // (c) Continue holding walk input: speed must come back to cruise.
        let mut v_recover = 0.0f32;
        for _ in 0..((stride * 4.0) / dt) as usize {
            let (_p, _d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            v_recover = v_recover.max(s.hip_vel.x);
            if s.hip_vel.x > 0.9 * v_cruise {
                break;
            }
        }
        assert!(
            v_recover > 0.8 * v_cruise,
            "did not resume walking after the hop: peak v={v_recover:.2}, cruise={v_cruise:.2}"
        );
    }

    #[test]
    fn test_jump_to_walk_vx_continuity() {
        // Hypothesis under test: at the jump→walk handoff the Raibert resolver
        // injects a *sudden* change in hip horizontal velocity — a jolt on the
        // hip particle — instead of carrying `v_x` continuously through the hop.
        //
        // We hold walk input across the whole walk→jump→walk episode and, per
        // frame, record the single-frame change |Δv_x|. Outside a touchdown
        // backward-solve the horizontal loop may only move `v_x` by its accel
        // budget `a_eff·dt`; a larger "free" |Δv_x| is a jolt with no physical
        // source. On touchdown frames the backward solve legitimately steps
        // `v_x` — telemetry exposes it as `v_support != 0` — so those are
        // reported separately rather than lumped into the free-jolt bound.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let v_cruise = walk_state(&cfg, dt).hip_vel.x;
        let mut s = walk_state(&cfg, dt);

        s.vs.jump_requested = true;
        let mut saw_flight = false;
        let mut land_seen = false;
        let mut prev_v = s.hip_vel.x;

        let mut worst_free = 0.0f32; // largest jolt on a no-touchdown frame
        let mut worst_free_phase = VerticalPhase::Stand;
        let mut worst_free_frame = 0usize;
        let mut worst_td = 0.0f32; // largest jolt on a touchdown frame
        let mut worst_td_phase = VerticalPhase::Stand;
        let mut worst_td_support = 0.0f32;
        let mut worst_td_frame = 0usize;
        let mut worst = 0.0f32; // largest jolt anywhere
        let mut worst_desc = String::new();

        // Accel budget per frame with a fat margin: under power `step_lower_body`
        // approaches `v_desired` by at most `accel·dt` per tick.
        let free_budget = cfg.accel * dt * 1.5;

        for frame in 0..4000 {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "jump-walk", frame);
            match d.v_phase {
                VerticalPhase::Flight => saw_flight = true,
                VerticalPhase::Land => land_seen = true,
                _ => {}
            }
            let v = s.hip_vel.x;
            let jolt = (v - prev_v).abs();
            if jolt > worst {
                worst = jolt;
                worst_desc = format!(
                    "phase {:?} v_support={:.3} {:.3}->{:.3} (dv={:.3})",
                    d.v_phase, d.v_support, prev_v, v, jolt
                );
            }
            if d.v_support == 0.0 {
                if jolt > worst_free {
                    worst_free = jolt;
                    worst_free_phase = d.v_phase;
                    worst_free_frame = frame;
                }
            } else if jolt > worst_td {
                worst_td = jolt;
                worst_td_phase = d.v_phase;
                worst_td_support = d.v_support;
                worst_td_frame = frame;
            }
            prev_v = v;
            if saw_flight && land_seen && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }
        assert!(saw_flight && land_seen, "never flew and/or never landed");

        println!(
            "[jump->walk vx] cruise={v_cruise:.2} | worst dv={worst:.3} @ {worst_desc} | \
             free worst dv={worst_free:.3} @ {worst_free_phase:?} frame {worst_free_frame} | \
             touchdown worst dv={worst_td:.3} @ {worst_td_phase:?} frame {worst_td_frame} \
             (v_support={worst_td_support:.3})"
        );

        // DESIGN: outside an explicit touchdown solve the horizontal loop may
        // only move `v_x` by its accel budget. A violation is the hypothesized
        // sudden hip-velocity change with no physical source.
        assert!(
            worst_free <= free_budget,
            "sudden dv_x with NO touchdown source: {worst_free:.3} at {worst_free_phase:?} \
             frame {worst_free_frame} (budget {free_budget:.3}/frame); cruise={v_cruise:.2}"
        );
    }

    #[test]
    fn test_exit_jump_into_walk_vx_no_sudden_change() {
        // The literal "exit jumping into walking": jump from an idle stand and
        // only press forward AFTER touchdown. The hand-off is a step input on the
        // velocity *target*, but the horizontal loop may still only move `v_x` by
        // its accel budget per frame — except on a touchdown frame (`v_support`
        // != 0), which is the one legitimate source of a velocity step.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let mut s = ResolverState::new(&cfg);
        drive(&cfg, &mut s, dt, (1.0 / dt) as usize, false, "rest");

        s.vs.jump_requested = true;
        let mut saw_flight = false;
        let mut land_seen = false;
        let mut walking = false; // flips on once we press forward after landing
        let mut worst_free = 0.0f32;
        let mut free_phase = VerticalPhase::Stand;
        let mut free_frame = 0usize;
        let mut worst_td = 0.0f32;
        let mut prev_v = s.hip_vel.x;
        let free_budget = cfg.accel * dt * 1.5;

        for frame in 0..4000 {
            if land_seen && s.vs.phase == VerticalPhase::Stand {
                walking = true;
            }
            let input = if walking { Vec2::X } else { Vec2::ZERO };
            let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "exit-jump-walk", frame);
            match d.v_phase {
                VerticalPhase::Flight => saw_flight = true,
                VerticalPhase::Land => land_seen = true,
                _ => {}
            }
            let jolt = (s.hip_vel.x - prev_v).abs();
            if d.v_support == 0.0 {
                if jolt > worst_free {
                    worst_free = jolt;
                    free_phase = d.v_phase;
                    free_frame = frame;
                }
            } else if jolt > worst_td {
                worst_td = jolt;
            }
            prev_v = s.hip_vel.x;
            if walking && s.hip_vel.x > 0.8 * cfg.target_speed {
                break;
            }
        }
        assert!(saw_flight && land_seen, "never flew and/or never landed");

        println!(
            "[exit-jump->walk vx] worst free dv={worst_free:.3} @ {free_phase:?} frame \
             {free_frame} | touchdown dv={worst_td:.3} (budget {free_budget:.3}/frame)"
        );
        assert!(
            worst_free <= free_budget,
            "sudden dv_x when exiting a jump into walking (NO touchdown source): \
             {worst_free:.3} at {free_phase:?} frame {free_frame} (budget {free_budget:.3})"
        );
    }

    // --- Weird-but-legal condition probes ---
    //
    // These deliberately push degenerate or unusual inputs through the pure
    // resolver: magnitudes, directions, frame rates, zero dt, nonsense tuning.
    // They are not new features — they pin the *existing* contract at the edges,
    // which is exactly where a physics port diverges first.

    #[test]
    fn test_input_normalization_is_magnitude_invariant() {
        // `step_lower_body` normalizes the input direction (`normalize_or_zero`).
        // Only the unit direction may matter, so a tiny or a huge stick
        // magnitude must produce the very same cruise as a unit push. This
        // guards a refactor that starts scaling speed by raw magnitude.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let cruise_with = |input: Vec2| -> f32 {
            let mut s = ResolverState::new(&cfg);
            for frame in 0..((stride * 4.0) / dt) as usize {
                let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
                assert_pose_invariants(&cfg, &pose, &d, "normalize", frame);
            }
            s.hip_vel.x
        };
        let unit = cruise_with(Vec2::X);
        let huge = cruise_with(Vec2::new(700.0, 0.0));
        let tiny = cruise_with(Vec2::new(1.0e-3, 0.0));
        assert!(
            unit > 0.5 * cfg.target_speed,
            "unit push did not cruise: {unit}"
        );
        assert!(
            (unit - huge).abs() < 1.0e-3,
            "magnitude leaked into speed: unit={unit:.4} huge={huge:.4}"
        );
        assert!(
            (unit - tiny).abs() < 1.0e-3,
            "tiny magnitude did not normalize to +X: unit={unit:.4} tiny={tiny:.4}"
        );
    }

    #[test]
    fn test_reverse_direction_mirror_and_turnaround_are_bounded() {
        // Every other test walks in +X. Walk in -X (the mirror) and then reverse
        // mid-stride *without* idling. The mirror must reach exactly -cruise; the
        // reversal must stay finite, hold the invariants, and settle into -X.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let v_forward = walk_state(&cfg, dt).hip_vel.x;

        // (a) -X must be the exact mirror of +X.
        let mut s = ResolverState::new(&cfg);
        for frame in 0..((stride * 4.0) / dt) as usize {
            let (pose, d) = step_lower_body(Vec2::NEG_X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "reverse", frame);
        }
        let v_reverse = s.hip_vel.x;
        assert!(
            (v_reverse + v_forward).abs() < 0.05 * v_forward.abs().max(1.0),
            "reverse is not the mirror of forward: forward={v_forward:.3}, reverse={v_reverse:.3}"
        );

        // (b) Reversing mid-stride must stay bounded and settle into -X.
        let mut s = walk_state(&cfg, dt);
        let mut saw_negative = false;
        for frame in 0..((stride * 6.0) / dt) as usize {
            let (pose, d) = step_lower_body(Vec2::NEG_X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "turnaround", frame);
            if s.hip_vel.x < -0.5 * v_forward.abs() {
                saw_negative = true;
            }
        }
        assert!(
            saw_negative,
            "never reversed into -X: end v={:.3}",
            s.hip_vel.x
        );
    }

    #[test]
    fn test_held_jump_input_rehops_and_never_relaunches_midair() {
        // `input.y > 0.5` re-arms `jump_requested` every tick. Holding a walk+jump
        // stick (raw y = 1.0) must (a) never commit a hop except from a grounded
        // stand, and (b) re-hop after each landing — a bunny hop. The phase
        // predecessor is the contract: `Load` may only be entered from `Stand`,
        // and `Launch` only from `Load`.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        let mut s = ResolverState::new(&cfg);
        let input = Vec2::new(1.0, 1.0);
        let mut prev = VerticalPhase::Stand;
        let mut flights = 0usize;
        let mut was_flight = false;
        for frame in 0..8000 {
            let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "held-jump", frame);
            match d.v_phase {
                VerticalPhase::Load => assert!(
                    matches!(prev, VerticalPhase::Stand | VerticalPhase::Load),
                    "jump committed mid-air: Load entered from {prev:?} at frame {frame}"
                ),
                VerticalPhase::Launch => assert!(
                    matches!(prev, VerticalPhase::Load | VerticalPhase::Launch),
                    "launch without a grounded crouch: Launch entered from {prev:?} at frame {frame}"
                ),
                _ => {}
            }
            let flight = d.v_phase == VerticalPhase::Flight;
            if flight && !was_flight {
                flights += 1;
            }
            was_flight = flight;
            prev = d.v_phase;
            if flights >= 3 {
                break;
            }
        }
        assert!(
            flights >= 3,
            "held jump did not re-hop after each landing: flights={flights}"
        );
    }

    #[test]
    fn test_production_frame_rates_stay_stable_and_bounded() {
        // The grounded height spring is integrated with explicit Euler
        // (`regulate_height`), so stability is dt-limited. Every real frame rate
        // we ship must stay inside the stable band across a full walk+hop episode.
        let cfg = LowerBodyConfig::default();
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for &rate in &[30.0f32, 60.0, 120.0, 144.0, 240.0] {
            let dt = 1.0 / rate;
            let mut s = ResolverState::new(&cfg);
            let frames = ((stride * 8.0) / dt) as usize;
            for frame in 0..frames {
                let input = if (frame / 200) % 2 == 0 {
                    Vec2::X
                } else {
                    Vec2::ZERO
                };
                if frame % 400 == 100 {
                    s.vs.jump_requested = true;
                }
                let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
                assert_pose_invariants(&cfg, &pose, &d, &format!("rate {rate}"), frame);
                assert!(
                    s.hip_vel.x.abs() < 1.0e4 && d.hip_vel_y.abs() < 1.0e4,
                    "rate {rate} frame {frame}: runaway vx={} vy={}",
                    s.hip_vel.x,
                    d.hip_vel_y
                );
            }
        }
    }

    #[test]
    fn test_paused_dt_zero_is_a_noop() {
        // A paused frame (`dt = 0`) must not move the body, divide by zero, or
        // emit NaN. (Callers must not build loop bounds as `x / dt`, but the
        // resolver itself must be safe under a zero step.)
        let cfg = LowerBodyConfig::default();
        let mut s = ResolverState::new(&cfg);
        let (_p0, _d0) = step_lower_body(Vec2::X, &cfg, &mut s, 0.0);
        let x0 = s.hip_pos.x;
        let y0 = s.hip_pos.y;
        let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, 0.0);
        assert_pose_invariants(&cfg, &pose, &d, "dt0", 0);
        assert!(
            (s.hip_pos.x - x0).abs() < 1.0e-4 && (s.hip_pos.y - y0).abs() < 1.0e-4,
            "dt=0 moved the body: ({x0},{y0}) -> ({},{})",
            s.hip_pos.x,
            s.hip_pos.y
        );
    }

    #[test]
    fn test_degenerate_jump_apex_zero_and_negative_are_bounded() {
        // `jump_apex = 0` (no rise requested) and a negative `jump_apex`
        // (nonsense input) must not run away, NaN, or trap the FSM. The negative
        // rise is clamped to 0 by `apex_to_launch_speed`, so both collapse to a
        // tiny/no hop that still returns to `Stand`.
        let dt = 1.0 / 60.0;
        for apex in [0.0f32, -30.0] {
            let mut cfg = LowerBodyConfig::default();
            cfg.jump_apex = apex;
            let mut s = ResolverState::new(&cfg);
            // Drive idle, then request a hop with no move input, using inline
            // checks that do NOT band `Δl` by `jump_apex` — the shared helper's
            // `Δl < 100·jump_apex` band degenerates to `≤ 0` when `jump_apex ≤ 0`.
            let check = |pose: &LowerBodyPose, d: &DebugInfo, frame: usize| {
                for p in [
                    pose.hip,
                    pose.left_knee,
                    pose.left_foot,
                    pose.right_knee,
                    pose.right_foot,
                ] {
                    assert!(
                        p.x.is_finite() && p.y.is_finite(),
                        "apex={apex} frame {frame}: NaN pose {p:?}"
                    );
                }
                assert!(
                    !(d.left_phase == GaitPhase::Swing && d.right_phase == GaitPhase::Swing),
                    "apex={apex} frame {frame}: both feet airborne"
                );
                assert!(
                    d.delta_l.is_finite()
                        && d.hip_height.is_finite()
                        && d.hip_vel_y.is_finite()
                        && d.delta_l.abs() < 1.0e4,
                    "apex={apex} frame {frame}: loop-1 out of band: Δl={} h={} ḣ={}",
                    d.delta_l,
                    d.hip_height,
                    d.hip_vel_y
                );
            };
            for frame in 0..60 {
                let (pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
                check(&pose, &d, frame);
            }
            s.vs.jump_requested = true;
            let mut saw_flight = false;
            for frame in 0..3000 {
                let (pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
                check(&pose, &d, frame);
                if d.v_phase == VerticalPhase::Flight {
                    saw_flight = true;
                }
                if saw_flight && d.v_phase == VerticalPhase::Stand {
                    break;
                }
            }
            let h = s.hip_pos.y - cfg.floor_y;
            assert!(
                h.is_finite() && (h - cfg.hip_height).abs() < 5.0,
                "apex={apex}: did not settle to stand: h={h} (flight={saw_flight})"
            );
            assert!(
                s.hip_vel.y.abs() < 50.0,
                "apex={apex}: vertical runaway v={}",
                s.hip_vel.y
            );
        }
    }

    #[test]
    fn test_extreme_speed_and_reach_clamps_hold() {
        // An absurd `target_speed` must be clamped by the envelope (the shaped
        // velocity never exceeds `v_abs`), and a tiny `safety_margin`
        // (aggressive reach) must not break the invariants. Both are weird but
        // legal tuning values.
        let dt = 1.0 / 60.0;
        let stride = LowerBodyConfig::default().stance_duration_ts
            + LowerBodyConfig::default().swing_duration_ts;

        let mut cfg = LowerBodyConfig::default();
        cfg.target_speed = 100_000.0;
        let mut s = ResolverState::new(&cfg);
        for frame in 0..((stride * 6.0) / dt) as usize {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "hi-speed", frame);
            assert!(
                d.v_clamped.abs() <= d.v_abs + 1.0e-3,
                "speed exceeded the envelope: v={} v_abs={}",
                d.v_clamped,
                d.v_abs
            );
        }

        let mut cfg = LowerBodyConfig::default();
        cfg.safety_margin = 0.05;
        let mut s = ResolverState::new(&cfg);
        for frame in 0..((stride * 6.0) / dt) as usize {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "tiny-margin", frame);
        }
    }

    #[test]
    fn test_degenerate_step_timers_stay_finite() {
        // Sub-tick stance/swing timers and a zero stance timer are legal (if
        // nonsensical) configs. `compute_support_velocity` divides by
        // `Ts/2 + kv`; with `Ts = 0` that is just `kv`. The gait must still
        // produce finite, bounded poses (the legs flip every tick).
        let dt = 1.0 / 60.0;
        for (stance, swing) in [(0.0f32, 0.22f32), (0.001, 0.001), (0.001, 0.22)] {
            let mut cfg = LowerBodyConfig::default();
            cfg.stance_duration_ts = stance;
            cfg.swing_duration_ts = swing;
            let mut s = ResolverState::new(&cfg);
            for frame in 0..3000 {
                let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
                assert_pose_invariants(
                    &cfg,
                    &pose,
                    &d,
                    &format!("degenerate Ts={stance} Tsw={swing}"),
                    frame,
                );
                assert!(
                    s.hip_vel.x.abs() < 1.0e5 && d.hip_vel_y.abs() < 1.0e5,
                    "Ts={stance} Tsw={swing} frame {frame}: runaway vx={} vy={}",
                    s.hip_vel.x,
                    d.hip_vel_y
                );
            }
        }
    }

    #[test]
    fn test_config_fuzz_weird_parameters_stay_bounded_and_deterministic() {
        // Randomize the *config* (not just the schedule), sampling deeply weird
        // but legal tuning values, then drive a random walk/jump schedule. The
        // structural contracts must survive every draw: finite poses, no
        // double-swing, bounded velocity, and byte-exact replay per seed.
        fn draw(rng: &mut Lcg) -> LowerBodyConfig {
            let f = |rng: &mut Lcg, lo: f32, hi: f32| {
                lo + (rng.next_u32() % 10_000) as f32 / 10_000.0 * (hi - lo)
            };
            let mut c = LowerBodyConfig::default();
            c.target_speed = f(rng, 40.0, 220.0);
            c.accel = f(rng, 150.0, 2500.0);
            c.kv = f(rng, 0.05, 0.5);
            c.beta = f(rng, 0.2, 1.0);
            c.safety_margin = f(rng, 0.55, 0.95);
            c.prediction_margin = f(rng, 0.25, 0.75);
            c.step_height = f(rng, 8.0, 70.0);
            c.stance_duration_ts = f(rng, 0.12, 0.38);
            c.swing_duration_ts = f(rng, 0.10, 0.38);
            c.gravity = f(rng, 500.0, 1800.0);
            c.jump_apex = f(rng, 10.0, 120.0);
            c.k_p_apex = f(rng, 0.1, 1.0);
            c.launch_accel = f(rng, 800.0, 4000.0);
            c
        }
        fn run(seed: u64) -> Vec<[f32; 3]> {
            let dt = 1.0 / 60.0;
            let mut rng = Lcg::new(seed);
            let cfg = draw(&mut rng);
            let mut s = ResolverState::new(&cfg);
            let mut walk = false;
            let mut walk_hold = rng.span(1, 120);
            let mut jump_hold = rng.span(20, 200);
            let mut trace = Vec::with_capacity(4000);
            for frame in 0..4000 {
                if walk_hold == 0 {
                    walk = !walk;
                    walk_hold = rng.span(1, 120);
                }
                walk_hold -= 1;
                if jump_hold == 0 {
                    s.vs.jump_requested = true;
                    jump_hold = rng.span(20, 200);
                }
                jump_hold -= 1;
                let input = if walk { Vec2::X } else { Vec2::ZERO };
                let (pose, d) = step_lower_body(input, &cfg, &mut s, dt);
                for p in [
                    pose.hip,
                    pose.left_knee,
                    pose.left_foot,
                    pose.right_knee,
                    pose.right_foot,
                ] {
                    assert!(
                        p.x.is_finite() && p.y.is_finite(),
                        "cfg-fuzz {seed} frame {frame}: NaN pose {p:?}"
                    );
                }
                assert!(
                    !(d.left_phase == GaitPhase::Swing && d.right_phase == GaitPhase::Swing),
                    "cfg-fuzz {seed} frame {frame}: both feet airborne"
                );
                assert!(
                    s.hip_vel.x.abs() < 1.0e5 && d.hip_vel_y.abs() < 1.0e5,
                    "cfg-fuzz {seed} frame {frame}: runaway vx={} vy={}",
                    s.hip_vel.x,
                    d.hip_vel_y
                );
                trace.push([s.hip_pos.x, s.hip_pos.y, d.v_clamped]);
            }
            trace
        }
        for seed in 1..=20u64 {
            let a = run(seed);
            let b = run(seed);
            assert_eq!(a.len(), b.len(), "cfg-fuzz {seed}: length");
            for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                assert_eq!(x, y, "cfg-fuzz {seed} frame {i}: nondeterministic");
            }
        }
        assert_ne!(run(1), run(2), "cfg-fuzz: configs are not varying");
    }

    #[test]
    fn test_jump_is_a_synced_full_circle() {
        // Animation contract for the jump: it is a full gait cycle, not a freeze.
        //   * Load + Launch are the leading leg's grounded (stance) half;
        //   * Flight + Land are its airborne (swing) half, in which the foot
        //     reaches FORWARD from behind the hip to the landing point;
        //   * with β = 0.5 those two halves last the same time;
        //   * both legs are SYNCED (the trailing leg mirrors the leading leg), so
        //     they drive forward together and land on the SAME point.
        //
        // Regression guards, both of which the old behaviours failed:
        //   * the old freeze held each foot's offset from the hip constant for the
        //     whole hop (a "moon-walk" — no forward swing);
        //   * the older two-foot-sync retarget left both feet clustered BEHIND the
        //     advancing hip, so nothing reached ahead to catch the landing.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 60.0;
        // Per-frame particle-continuity bound: a smooth swing arc (the foot's own
        // forward reach plus the hip's `target_speed·dt ≈ 2 px` travel) stays well
        // under this, while a pose teleport would trip it.
        let step_bound = 12.0_f32;

        // Cruise to a steady walk so the hip and gait are both at speed.
        let mut s = walk_state(&cfg, dt);
        assert!(s.hip_vel.x > 100.0, "not cruising before the jump");

        let mut flight_lead_end = f32::NEG_INFINITY; // min foot reach at last flight frame
        let mut flight_gap_end = f32::INFINITY; // |left.x − right.x| at last flight frame
                                                // Hip→foot forward offsets at the FIRST and LAST airborne frames. The
                                                // total forward sweep is their difference (the per-frame delta is small
                                                // because the swing is deliberately smooth — and that smoothness is the
                                                // no-teleport guarantee checked by `max_step` below).
        let mut first_flight_off: Option<(f32, f32)> = None;
        let mut last_flight_off = (0.0_f32, 0.0_f32);
        let mut saw_flight = false;
        let mut prev: Option<LowerBodyPose> = None;
        let mut max_step = 0.0_f32;
        s.vs.jump_requested = true;
        for frame in 0..4000 {
            let (pose, d) = step_lower_body(Vec2::X, &cfg, &mut s, dt);
            assert_pose_invariants(&cfg, &pose, &d, "jump-swing", frame);
            let l = pose.left_foot.x - pose.hip.x;
            let r = pose.right_foot.x - pose.hip.x;
            if d.v_phase == VerticalPhase::Flight {
                saw_flight = true;
                if first_flight_off.is_none() {
                    first_flight_off = Some((l, r));
                }
                last_flight_off = (l, r);
                // Remember the last flight frame's reach and foot gap — the pose
                // the swing is committing to just before touchdown.
                flight_lead_end = l.min(r);
                flight_gap_end = (pose.left_foot.x - pose.right_foot.x).abs();
            }
            if let Some(p) = prev {
                for (name, a, b) in [
                    ("hip", p.hip, pose.hip),
                    ("left_knee", p.left_knee, pose.left_knee),
                    ("right_knee", p.right_knee, pose.right_knee),
                    ("left_foot", p.left_foot, pose.left_foot),
                    ("right_foot", p.right_foot, pose.right_foot),
                ] {
                    let step = (b - a).length();
                    max_step = max_step.max(step);
                    assert!(
                        step <= step_bound,
                        "particle `{name}` teleported {step:.1}px in one frame at f{frame} \
                         ({:?}→{:?}); bound={step_bound:.1}px",
                        p,
                        pose
                    );
                }
            }
            prev = Some(pose);
            if saw_flight && d.v_phase == VerticalPhase::Stand {
                break;
            }
        }

        assert!(saw_flight, "walking jump never went airborne");
        // How far the hip→foot offset swept forward over the airborne half (the
        // smaller leg, so BOTH legs must reach forward).
        let swing_forward = match first_flight_off {
            Some((l0, r0)) => (last_flight_off.0 - l0).min(last_flight_off.1 - r0),
            None => 0.0,
        };
        println!(
            "jump full circle: flight_lead_end={flight_lead_end:.1}px  \
             flight_gap_end={flight_gap_end:.1}px  \
             swing_forward={swing_forward:.1}px  max_step={max_step:.1}px"
        );
        // The swing must carry the legs FORWARD relative to the hip — the opposite
        // of the old freeze, where the offset (and so this sweep) was exactly 0.
        assert!(
            swing_forward > 4.0,
            "the jump legs were frozen, not swinging: the hip→foot offset moved only \
             {swing_forward:.1}px forward across Flight (expected a forward swing)"
        );
        // By the last flight frame the swing has carried BOTH feet ahead of the
        // hip, ready to catch the landing ...
        assert!(
            flight_lead_end > 0.0,
            "the airborne swing never reached ahead of the hip \
             (last-flight reach = {flight_lead_end:.1}px)"
        );
        // ... and the two legs have converged onto the SAME landing point.
        assert!(
            flight_gap_end < 12.0,
            "the two legs did not land on the same point (gap = {flight_gap_end:.1}px)"
        );
    }

    #[test]
    fn test_stand_still_jump_has_no_pose_teleport() {
        // Regression: a jump must never pop the knee particles to a different
        // position for a single frame and then snap them back — that discontinuity
        // would wreck the physics system that consumes this pose.
        //
        // The jump's airborne half is a forward swing arc, driven per-frame from
        // the ballistic flight clock (a sine arc, not a rate limit or a hard
        // clamp). Composed with the frictionless landing seed it delivers a pose
        // that is continuous across the Launch→Flight and Flight→Land boundaries:
        // the largest per-frame particle step is dominated by the hip's own
        // ballistic motion, never by a snapping knee. The airborne reach is also
        // capped (`FLIGHT_SWING_REACH_FRACTION·X_max`) so the landing pose stays
        // well inside the IK's safe bend zone and no knee straightens onto the
        // `acos` singularity.
        let cfg = LowerBodyConfig::default();
        let dt = 1.0 / 90.0;
        // Generous upper bound on per-frame particle travel. The reported pop was
        // ~21-23 px in one frame; a smooth arc keeps every particle well under
        // this.
        let bound = 10.0_f32;

        let mut s = ResolverState::new(&cfg);
        for _ in 0..40 {
            step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
        }

        let mut jumped = false;
        let mut prev: Option<LowerBodyPose> = None;
        let mut max_step = 0.0_f32;
        let mut saw_launch = false;
        let mut saw_flight = false;
        let mut saw_land = false;

        for frame in 0..600 {
            if !jumped && frame == 5 {
                s.vs.jump_requested = true;
                jumped = true;
            }
            let (pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
            match d.v_phase {
                VerticalPhase::Launch => saw_launch = true,
                VerticalPhase::Flight => saw_flight = true,
                VerticalPhase::Land => saw_land = true,
                _ => {}
            }
            if let Some(p) = prev {
                for (name, a, b) in [
                    ("hip", p.hip, pose.hip),
                    ("left_knee", p.left_knee, pose.left_knee),
                    ("right_knee", p.right_knee, pose.right_knee),
                    ("left_foot", p.left_foot, pose.left_foot),
                    ("right_foot", p.right_foot, pose.right_foot),
                ] {
                    let step = (b - a).length();
                    max_step = max_step.max(step);
                    assert!(
                        step <= bound,
                        "particle `{name}` teleported {step:.1}px in one frame at f{frame} \
                         ({:?}→{:?}); bound={bound:.1}px",
                        p,
                        pose
                    );
                }
            }
            prev = Some(pose);
            if saw_flight && d.v_phase == VerticalPhase::Stand && frame > 40 {
                break;
            }
        }

        assert!(
            saw_launch && saw_flight && saw_land,
            "jump never completed a full cycle"
        );
        assert!(
            max_step <= bound,
            "pose moved too fast: {max_step:.1}px > {bound:.1}px"
        );
    }
}
