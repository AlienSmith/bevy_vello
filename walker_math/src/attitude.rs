use super::types::*;
// ---------------------------------------------------------------------------
// Loop 3: the attitude (tilt) PD controller (pure math)
// ---------------------------------------------------------------------------

/// The loop-3 control input. With `θ_desired = 0` (upright) this is the
/// textbook rotational spring-damper:
///
/// ```text
/// u = Kp · (θ_desired − θ) − Kd · ω
/// ```
///
/// `u` is an **angular acceleration** (rad/s²) — inertia is normalized out
/// (`I = m·L²`), so `Kp` is rad/s² per rad and `Kd` is 1/s. Positive `u`
/// torques the torso forward.
#[inline]
pub fn attitude_pd(theta: f32, omega: f32, theta_desired: f32, kp: f32, kd: f32) -> f32 {
    kp * (theta_desired - theta) - kd * omega
}

/// Loop-3 in its **leg-delivery** form: the PD output is not a torque about a
/// waist joint, but the *lean of the leg PD's world-upright reference axis*.
///
/// The body above `P3` is a straight, stiff stick; the only external forces are
/// gravity, foot contact and friction, so the stick's verticality is decided by
/// the hip joints `P2–P3–P40/P41` — not by `P1–P2–P3`. Raibert's loop 3 has no
/// concept of `P1` either. So the measured world-frame attitude (`theta_meas`,
/// `omega_meas`) is delivered by tilting the reference axis the leg PD aims the
/// thigh at:
///
/// ```text
/// lean = bias + Kp · (bias − θ_meas) − Kd · ω_meas
/// ```
///
/// i.e. exactly [`attitude_pd`] toward `bias`, plus `bias` added so the
/// reference axis itself carries the correction: if the stick leans forward
/// (`θ_meas > 0`) the legs aim *backward* (`lean < bias`) and push the stick
/// upright, and vice-versa. This is a **position** command (radians), not an
/// acceleration — the leg sprints resolve it against the contacts.
///
/// `bias` is the desired torso lean (radians, `+` = forward / `+x`). The result
/// is clamped to `±max_angle` so a bad measurement can never command the legs
/// to an impossible reference. Non-finite inputs degrade to `0.0` (straight).
#[inline]
pub fn attitude_lean_pd(
    theta_meas: f32,
    omega_meas: f32,
    bias: f32,
    kp: f32,
    kd: f32,
    max_angle: f32,
) -> f32 {
    let lean = bias + attitude_pd(theta_meas, omega_meas, bias, kp, kd);
    if lean.is_finite() {
        lean.clamp(-max_angle.abs(), max_angle.abs())
    } else {
        0.0
    }
}

/// One tick of the loop-3 (attitude) controller: the inverted-pendulum EOM at
/// the hip plus the PD, integrated with semi-implicit (symplectic) Euler.
///
/// ```text
/// θ̈ =  (g̃ · sinθ) / L_upper · GRV      ← gravity: DESTABILIZING
///    − (a_x · cosθ) / L_upper · ACC      ← hip horizontal accel: tilt source
///    +  u                                 ← loop-3 control input
/// ```
///
/// In `Flight` the torso is weightless in its own frame, so `GRV` is dropped
/// (no `g·sinθ` term) and `a_x` is ~0 (ballistic carry) — the only authority
/// left is the PD, which acts as air reorientation toward upright.
///
/// See `plans/raibert_sandbox_attitude_loop.md` §3.2–§4.
#[inline]
pub fn tick_attitude(
    state: &mut AttitudeState,
    config: &LowerBodyConfig,
    hip_accel_x: f32,
    dt: f32,
    airborne: bool,
) -> f32 {
    if dt <= 0.0 {
        // No step: nothing moves (the caller still reports the current PD output).
        return attitude_pd(
            state.theta,
            state.omega,
            0.0,
            config.attitude_kp,
            config.attitude_kd,
        );
    }

    let theta = state.theta;
    let omega = state.omega;
    let (sin_t, cos_t) = theta.sin_cos();

    // Gravity term: destabilizing pole about the hip. Dropped in flight
    // (weightless torso) and when the coupling toggle is off.
    let gravity_term = if config.attitude_gravity_coupling && !airborne {
        (config.gravity * sin_t) / config.torso_len.max(f32::EPSILON)
    } else {
        0.0
    };

    // Inertial-coupling term: hip horizontal acceleration tilts the torso
    // backwards under forward acceleration, forwards under braking.
    let accel_term =
        (config.attitude_accel_coupling * hip_accel_x * cos_t) / config.torso_len.max(f32::EPSILON);

    // PD control input toward upright.
    let u = attitude_pd(theta, omega, 0.0, config.attitude_kp, config.attitude_kd);

    // Semi-implicit Euler: update ω first, then θ from the new ω.
    let theta_ddot = gravity_term - accel_term + u;
    let omega_new = omega + theta_ddot * dt;
    let theta_new = theta + omega_new * dt;

    // Clamp so the torso can never spin past "fallen over" in one tick.
    let theta_max = config.theta_max.max(0.05);
    state.omega = omega_new;
    state.theta = theta_new.clamp(-theta_max, theta_max);

    u
}
