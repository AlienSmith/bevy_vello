use super::types::*;
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

/// Stage 1 sign-flip floor (`plans/walk-improvement-plan.md`). Given a solved
/// support velocity `v_support` and the signed command `v_desired`, allow the
/// hip to brake toward `floor · v_desired` but never past it, so a lagged
/// plant can never invert the travel direction against intent. `floor < 0`
/// disables the clamp (committed behavior). Forward/backward symmetric.
#[inline]
pub fn apply_support_floor(v_support: f32, v_desired: f32, floor: f32) -> f32 {
    if floor < 0.0 {
        return v_support;
    }
    let bound = floor * v_desired;
    if v_desired > 0.0 {
        v_support.max(bound)
    } else if v_desired < 0.0 {
        v_support.min(bound)
    } else {
        v_support
    }
}

/// Bootstrap **gas guard** (`plans/walk-improvement-plan.md` §2b): while
/// bootstrapping, a swing foot may never be aimed at or behind its own lift-off
/// pivot. At `v ≈ 0` the placement law is blind — `xi = x_CoM` — so it emits
/// `x_CoM − v_desired/(T_conv·ω₀)`, a target *behind* the hip. Stepping there
/// throws away the rear-foot push-off stance that is the only actuator able to
/// create forward velocity from rest. Flooring the target at `pivot + toe_len`
/// keeps the step forward of the pivot (never a backward step), while a `v = 0`
/// command leaves the target untouched (no phantom step intent).
///
/// `toe_len` is the support-edge proxy already used by the §3 trigger, so the
/// floored target sits exactly at the stance foot's forward edge — the smallest
/// forward step the gait can ask for. Direction follows the *signed* command.
#[inline]
pub fn apply_bootstrap_floor(target_x: f32, pivot_x: f32, v_desired: f32, toe_len: f32) -> f32 {
    if v_desired > 0.0 {
        target_x.max(pivot_x + toe_len.max(0.0))
    } else if v_desired < 0.0 {
        target_x.min(pivot_x - toe_len.max(0.0))
    } else {
        target_x
    }
}

/// Whether the bootstrap gates are active this tick: `v_boot >= 0` (the flag is
/// enabled) **and** the measured speed is still below it. Reads the measured
/// velocity on purpose — the resolver's own reference is what must not
/// bootstrap itself (see the `v_boot` doc comment).
#[inline]
pub fn bootstrap_gate_active(v_boot: f32, measured_vx: f32) -> bool {
    v_boot >= 0.0 && measured_vx.abs() < v_boot
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
