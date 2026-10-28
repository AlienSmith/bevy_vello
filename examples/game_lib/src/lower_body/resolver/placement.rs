use super::laws::compute_forward_target;
use super::types::*;
// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

/// Minimum `|v_desired|` (move intent, px/s) required to initiate a step. With
/// no intent the gait freezes: both feet stay planted and the stop-settle servo
/// in [`step_lower_body`] glides the hip to the support centre. Gating on
/// *intent* rather than actual velocity also prevents the settle motion itself
/// from triggering phantom steps that would walk the character off with no
/// input held.
pub(crate) const SWING_TRIGGER_SPEED: f32 = 1.0;

/// Maximum fraction of the full leg reach `thigh + calf` that the airborne swing
/// may use for the hip→foot offset. Keeps the knee off the 2-bone IK's
/// `acos(reach/(thigh+calf))` singularity: with `thigh == calf` a reach of
/// `0.96·(L1+L2)` holds the knee half-angle at `α = acos(0.96) ≈ 16°`, so a fast
/// hip during Flight can never straighten (and thereby snap) the knee.
pub(crate) const LIFT_REACH_SAFE_FRACTION: f32 = 0.96;

/// Horizontal reach the lead foot may extend ahead of the hip at the landing
/// plant, as a fraction of the ground-reference gait reach `X_max`. The jump's
/// airborne (swing) half drives both feet forward from behind the hip to ahead
/// of it; this caps how far ahead so the landing pose not only sits inside the
/// IK reach but also leaves the leg *visibly bent* — with `thigh == calf` the
/// knee half-angle is `α = acos(reach / (thigh + calf))`, so keeping the
/// standing-height reach at `0.5·X_max` (≈ 35 px ≪ 110 px) holds `α` far from
/// the IK's `acos(reach/110)` singularity and no knee can ever snap straight.
pub(crate) const FLIGHT_SWING_REACH_FRACTION: f32 = 0.5;

/// How far the airborne swing lifts the foot off the planted floor line, as a
/// fraction of the walking `step_height` — the vertical amplitude of the jump's
/// full-circle foot arc. Kept below 1 so the airborne swing reads as a running
/// stride rather than a knee-to-chest tuck.
pub(crate) const FLIGHT_SWING_LIFT_FRACTION: f32 = 0.8;

/// Resolve the deadbeat convergence deadline `t_c` from the config.
///
/// `dcm_tc > 0` is the window in seconds (`DCM_TC=<seconds>`); `dcm_tc == 0`
/// auto-selects the stance duration (the physical deadline, since a foot can
/// only act while planted); negative disables the law (`t_c = 0`, which makes
/// [`capture_point_deadbeat`] return `None`).
pub fn resolve_deadbeat_tc(dcm_tc: f32, stance_ts: f32) -> f32 {
    match dcm_tc {
        t if t > 0.0 => t,
        0.0 => stance_ts,
        _ => 0.0,
    }
}

/// Measured whole-body CoM capture state (loop 1 balance, plan §2), fed into
/// the resolver each tick by the ECS layer. `com_x` / `v_com` are already in
/// the resolver's y-up frame (horizontal x is unaffected by the Vello y-down
/// flip); `z_com` is the CoM height above the floor, positive-up.
#[derive(Debug, Clone, Copy)]
pub struct CaptureMeasure {
    /// CoM x in the resolver's y-up frame (equal to Vello x).
    pub com_x: f32,
    /// EMA-filtered CoM horizontal velocity, px/s (equal to Vello x).
    pub v_com: f32,
    /// CoM height above the floor line, px (positive-up).
    pub z_com: f32,
}

/// LIPM pendulum frequency `ω₀ = √(g / z_CoM)` (rad/s), or `None` when
/// undefined (`z_com ≤ 0` or `gravity ≤ 0`).
#[inline]
pub fn omega0(z_com: f32, gravity: f32) -> Option<f32> {
    if z_com > 0.0 && gravity > 0.0 {
        Some((gravity / z_com).sqrt())
    } else {
        None
    }
}

/// LIPM capture point shifted by the desired CoM velocity (plan §2):
/// `x_foot = x_CoM + (v_CoM − v_des) / ω₀`, with `ω₀ = √(g / z_CoM)`.
///
/// Returns `None` when the pendulum frequency is undefined (`z_com ≤ 0` or
/// `gravity ≤ 0`), so callers fall back to the Raibert neutral point.
#[inline]
pub fn capture_point(
    com_x: f32,
    v_com: f32,
    v_desired: f32,
    z_com: f32,
    gravity: f32,
) -> Option<f32> {
    if z_com <= 0.0 || gravity <= 0.0 {
        return None;
    }
    let omega0 = (gravity / z_com).sqrt();
    Some(com_x + (v_com - v_desired) / omega0)
}

/// Per-tick planning context shared by both legs.
#[derive(Debug, Clone, Copy)]
pub(crate) struct StepPlan {
    /// Safety-margined reach; swing targets are clamped to `hip ± x_max` so a
    /// plant can never be aimed outside the workspace (which the IK would
    /// ray-clamp into a foot floating off the floor).
    pub(crate) x_max: f32,
    /// True while braking to a stop: no move intent but the hip is still
    /// travelling faster than the swing trigger.
    pub(crate) stopping: bool,
    /// Capture-point offset from the *current* hip while `stopping`:
    /// `v·|v| / (2·a_eff)`. Aimed at the point where the hip will come to rest
    /// under the effective deceleration, so the hip brakes *onto* its foot.
    pub(crate) capture_offset: f32,
    /// Measured whole-body capture state (plan §2). `None` in the pure
    /// kinematic sandbox; `plan_target_x` then falls back to the Raibert
    /// neutral point.
    pub(crate) measured: Option<CaptureMeasure>,
    /// Signed DCM/capture-point state `xi = x_CoM + v_CoM/ω₀` for this tick,
    /// with `ω₀ = √(g / z_CoM)`. `Some` only when a measurement exists and the
    /// pendulum frequency is defined; drives both the §3 DCM-margin liftoff
    /// trigger (`dcm_crossed`) and the §4 fixed-point touchdown prediction.
    pub(crate) xi: Option<f32>,
    /// Pendulum frequency `ω₀` (rad/s) matching `xi` (defined exactly when
    /// `xi` is `Some`). Used by the §4 prediction's exponential lead.
    pub(crate) omega0: f32,
    /// Enable the §3 DCM-margin liftoff trigger (`dcm_crossed`). Off = the
    /// committed baseline (`com_crossed` only). The margin trigger is a
    /// **matched pair** with the §4 exponential touchdown lead: firing the
    /// trigger early while the placement still evaluates the law at the
    /// *current* state lets the foot chase a target it cannot reach in the
    /// capped flight — measured to worsen the touchdown lead (−18/−32 px vs
    /// the baseline −7/+21). Stays off until §4 lands.
    pub(crate) mpc_trigger: bool,
}

/// World-x swing target for a foot, given `t_flight` seconds remaining in the
/// air. Used both at lift-off (`t_flight = T_swing`) and by the per-frame
/// re-planning during swing.
///
/// While stopping, targets the capture point instead of the forward plan.
/// The result is always clamped to `hip ± x_max`.
pub(crate) fn plan_target_x(
    hip_x: f32,
    v: f32,
    v_desired: f32,
    t_flight: f32,
    config: &LowerBodyConfig,
    plan: &StepPlan,
) -> f32 {
    let target = plan_target_x_raw(hip_x, v, v_desired, t_flight, config, plan);
    target.clamp(hip_x - plan.x_max, hip_x + plan.x_max)
}

/// The placement law itself, without the reach clamp ([`plan_target_x`] applies
/// it).
pub(crate) fn plan_target_x_raw(
    hip_x: f32,
    v: f32,
    v_desired: f32,
    t_flight: f32,
    config: &LowerBodyConfig,
    plan: &StepPlan,
) -> f32 {
    if plan.stopping {
        // Braking-to-stop keeps the kinematic capture-point form: `hip + the
        // point where the hip comes to rest under `a_eff`. (v_des → 0 makes the
        // general law a capture-point target too, but `stopping` covers the
        // no-measurement and settle paths, so the branch stays.)
        return (hip_x + plan.capture_offset).clamp(hip_x - plan.x_max, hip_x + plan.x_max);
    }
    // Loop 1 balance (plan §2, §4): foot placement from the LIPM capture point
    // state. This is a WORLD-space target — `x_CoM` is the measured CoM
    // position — so it needs no `v·t_flight` hip prediction (unlike the
    // hip-relative Raibert neutral point, which must predict where the hip will
    // be at touchdown). If the measurement is degenerate (`z_com ≤ 0` or
    // `gravity ≤ 0`), fall back to the Raibert neutral point below.
    //
    // Placement law: the analytic deadbeat solution (`DCM_TC`), the exact
    // two-coefficient LIPM form, evaluated with the **horizon-stretched**
    // absolute deadline `t_c' = t_c + t_flight.max(0)` on the live state:
    // every swing frame promises `v = v_desired` one stance after the
    // *actual* touchdown, so the promise never slides forward with the tick
    // (the receding-horizon re-anchor was removed 2026-10-28 — it is the
    // as-if-now fiction; the state-propagation variant was measured worse:
    // clamp-saturated mid-flight arcs). The stretch softens `(α, β)` early in
    // flight and converges to the plain law as `t_flight → 0` (the planting
    // tick). Cruise lead `tanh(ω₀·t_c'/2)·v/ω₀` stays under the capture limit
    // `v/ω₀ < x_max`, so the mid-flight aim never saturates the reach clamp.
    //
    // `p = ξ + (v·e^{−ω₀·t_c} − v_des)/(ω₀·sinh(ω₀·t_c)) = x + α·v − β·v_des`.
    // `DCM_TC > 0` is the window in seconds; `DCM_TC == 0` (the default)
    // auto-selects the stance duration (see [`resolve_deadbeat_tc`]); negative
    // disables the law, falling back to the Raibert neutral point below. The
    // velocity-error-gain and horizon truncations of this family were removed
    // (2026-10-28) — deadbeat contains both as its limits.
    let t_c = resolve_deadbeat_tc(config.dcm_tc, config.stance_duration_ts);
    let t_c_eff = if t_c > 0.0 {
        t_c + t_flight.max(0.0)
    } else {
        t_c
    };
    if let Some(x) = capture_point_deadbeat(plan.xi, v, v_desired, plan.omega0, t_c_eff) {
        return x.clamp(hip_x - plan.x_max, hip_x + plan.x_max);
    }
    let x_target = compute_forward_target(v, v_desired, config.kv, config.stance_duration_ts);
    // Raibert's `x_target` is the offset from the hip **at touchdown**, not at
    // lift-off: the hip keeps travelling while the foot is airborne, so predict
    // that motion. Re-planning every frame with the shrinking `t_flight` keeps
    // the world-space target stationary at constant velocity (hip advance `v·dt`
    // cancels the prediction shrink) and makes the target track
    // acceleration/braking transients mid-swing.
    (hip_x + v * t_flight + x_target).clamp(hip_x - plan.x_max, hip_x + plan.x_max)
}

/// Analytic (deadbeat) foot placement — the **exact** solution, not a hand-tuned
/// form of it. The only runtime placement law; its two single-dof truncations
/// (the zero-lead horizon form and the `ξ − Kp·(v_des − v)` gain form) were
/// removed 2026-10-28 and survive only as the `t_c → ∞` / `t_c → 0` limits
/// pinned by `test_deadbeat_placement_is_the_exact_solution`.
///
/// Ask: with the foot planted at `p` and the body at `(x, v)`, at what `p` does
/// the LIPM deliver `v(T_c) = v_desired` exactly `T_c` seconds from now? The
/// stance solution is `x(T) = p + (x−p)·cosh(ω₀T) + (v/ω₀)·sinh(ω₀T)` with
/// `v(T) = ω₀(x−p)·sinh(ω₀T) + v·cosh(ω₀T)`; eliminating `(x−p)` and using
/// `xi = x + v/ω₀` gives
///
/// ```text
/// p = xi + (v·e^(−ω₀·t_c) − v_desired) / (ω₀·sinh(ω₀·t_c))
///   = x + α·v − β·v_desired,   α = 1/(ω₀·tanh(ω₀t_c)),  β = 1/(ω₀·sinh(ω₀t_c))
/// ```
///
/// Two coefficients from one physical parameter. A single-dof form (fixed `α`
/// or welded `α = 1/ω₀ + Kp`) can get the velocity gain right or the lead
/// right, not both — this expression has both, which is why it is the shipped
/// law.
///
/// Limits: `t_c → ∞` ⇒ `p → xi` (plain capture point, asymptotic stop);
/// `t_c → 0` ⇒ `p → x + (v − v_des)/(ω₀²·t_c)`, the stiff velocity-error spring.
///
/// `t_c` is a **deadline**, so it must cover the flight it is planned through
/// (`t_c ≥ T_swing`), otherwise the foot is asked for a velocity during its own
/// swing. `t_c <= 0` disables it (the caller falls back to the Raibert neutral
/// point).
#[inline]
pub fn capture_point_deadbeat(
    xi: Option<f32>,
    v: f32,
    v_desired: f32,
    omega0: f32,
    t_c: f32,
) -> Option<f32> {
    let x = xi?;
    if omega0 <= f32::EPSILON || t_c <= 0.0 {
        return None;
    }
    let s = omega0 * (omega0 * t_c).sinh();
    if s.abs() <= f32::EPSILON {
        return None;
    }
    Some(x + (v * (-omega0 * t_c).exp() - v_desired) / s)
}
