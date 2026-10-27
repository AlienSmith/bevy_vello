use bevy::math::Vec2;
use super::placement::{FLIGHT_SWING_LIFT_FRACTION, SWING_TRIGGER_SPEED};
use super::types::*;
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
pub(crate) fn regulate_height(state: &mut ResolverState, y_target: f32, dt: f32) {
    let dy = y_target - state.hip_pos.y;
    let a = VERTICAL_K_S * dy - VERTICAL_K_D * state.hip_vel.y;
    state.hip_vel.y += a * dt;
    state.hip_pos.y += state.hip_vel.y * dt;
}

/// One tick of the loop-1 (vertical) controller.
///
/// State-based triggers only: crouch depth reached, measured h-dot cutoff,
/// full extension, contact loss, settle band. No timers gate the launch.
pub(crate) fn tick_vertical(state: &mut ResolverState, config: &LowerBodyConfig, dt: f32) {
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
            // the boundary measurement the per-hop law consumes. When the ECS
            // layer feeds a measured whole-body CoM apex (plan §4), prefer that
            // physics measurement over the resolver's internal ballistic model,
            // so the `Δl` integrator closes on the real ground reaction.
            if state.hip_vel.y <= 0.0 && !state.vs.apex_latched {
                state.vs.h_apex_prev = match state.measured_apex {
                    Some(measured) => measured.max(0.0),
                    None => state.vs.h_apex,
                };
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
