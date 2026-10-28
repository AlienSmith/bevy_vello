use super::placement::*;
use super::types::*;
use bevy::math::Vec2;
/// Three-phase swing profile (plan §3): maps normalized swing progress `s ∈
/// [0,1]` to the foot's world position along a C¹-continuous piecewise path from
/// the lift-off plant `from` to the touchdown plant `to`:
///
/// - **Lift** (s 0 → 0.25): knee flex — the foot rises steeply to `step_height`
///   while advancing only ~16% of the step length, so it clears the toe before
///   moving forward.
/// - **Advance** (s 0.25 → 0.8): hip flex — the foot sweeps most of the step
///   length forward at roughly constant height.
/// - **Extend / plant** (s 0.8 → 1.0): knee extend — the foot descends to the
///   touchdown x, decelerating to a gentle plant.
///
/// Each segment is a cubic Hermite spline through endpoint positions AND
/// derivatives, with the junction derivatives matched across segments, so the
/// path is C¹-continuous: no velocity discontinuity (and no touchdown "kick")
/// at any segment joint. `+y` is up (resolver frame); the profile returns an
/// absolute foot position.
#[inline]
pub(crate) fn swing_profile_foot(s: f32, from: Vec2, to: Vec2, step_height: f32) -> Vec2 {
    let s = s.clamp(0.0, 1.0);
    let dx = to.x - from.x;
    let h = step_height.max(0.0);

    // Control points (relative to `from`) and their s-space derivatives
    // (dP/ds across the full swing) encoding the choreography above.
    let p0 = Vec2::new(0.0, 0.0);
    let v0 = Vec2::new(0.30 * dx, 2.0 * h);
    let p1 = Vec2::new(0.16 * dx, h);
    let v1 = Vec2::new(1.20 * dx, 0.0);
    let p2 = Vec2::new(0.86 * dx, 0.50 * h);
    let v2 = Vec2::new(0.90 * dx, -1.50 * h);
    let p3 = Vec2::new(dx, 0.0);
    let v3 = Vec2::new(0.12 * dx, 0.0);

    // Cubic Hermite basis over local `u ∈ [0,1]` (position + derivative at both
    // endpoints). The passed derivative is dP/du; the caller converts the
    // s-space velocity by the segment width (dP/du = dP/ds * width).
    let hermite = |u: f32, pa: Vec2, va: Vec2, pb: Vec2, vb: Vec2| -> Vec2 {
        let u2 = u * u;
        let u3 = u2 * u;
        let h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
        let h10 = u3 - 2.0 * u2 + u;
        let h01 = -2.0 * u3 + 3.0 * u2;
        let h11 = u3 - u2;
        pa * h00 + va * h10 + pb * h01 + vb * h11
    };

    const S_LIFT: f32 = 0.25;
    const S_ADV: f32 = 0.80;
    let off = if s <= S_LIFT {
        let w = S_LIFT;
        let u = s / w;
        hermite(u, p0, v0 * w, p1, v1 * w)
    } else if s <= S_ADV {
        let w = S_ADV - S_LIFT;
        let u = (s - S_LIFT) / w;
        hermite(u, p1, v1 * w, p2, v2 * w)
    } else {
        let w = 1.0 - S_ADV;
        let u = (s - S_ADV) / w;
        hermite(u, p2, v2 * w, p3, v3 * w)
    };
    from + off
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
pub(crate) fn step_leg(
    leg: &mut LegState,
    hip_x: f32,
    v: f32,
    v_desired: f32,
    other_swinging: bool,
    config: &LowerBodyConfig,
    plan: &StepPlan,
    dt: f32,
    contact: Option<LegContactInput>,
    authority: GaitAuthority,
    max_swing_wait: f32,
    early_lift_count: &mut u32,
    swing_wait_count: &mut u32,
) -> Option<f32> {
    // `Contact` and above may trust the measured foot position. `Observe` and
    // `Timer` must not — that is what separates them from `Contact`.
    let plant_correction = authority >= GaitAuthority::Contact;
    match leg.phase {
        GaitPhase::Stance => {
            // Plan §6 stance correction (`Contact` and above): re-anchor the
            // kinematic plant to where the foot ACTUALLY landed. Without this the
            // resolver keeps drawing the foot from `plant_pos` — from where it
            // *asked* the foot to be — so a landing error silently invalidates
            // every downstream angle, and the stretch measured here is never what
            // the physics delivered.
            //
            // Corrected **once per stance**, on the first tick contact confirms
            // the foot is down. Not every tick: see `LegState::plant_anchored`
            // for the feedback loop that creates. One correction removes the
            // landing error; repeated correction transfers the stride to the
            // foot's slip.
            if plant_correction && !leg.plant_anchored {
                if let Some(c) = contact {
                    if c.touching {
                        if let Some(x) = c.foot_x {
                            if x.is_finite() {
                                leg.plant_pos.x = x;
                                leg.plant_anchored = true;
                            }
                        }
                    }
                }
            }
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
            // Plan §6 early lift-off (`Full` only): a timer that says Stance
            // while the foot is airborne makes loop 2 extend a leg against
            // nothing, and makes loop 1 believe it has support it does not have.
            //
            // Requires ALL THREE of: contact truth exists, the foot is not
            // `hovered_in` (i.e. this stance began with a genuine touchdown), and
            // the foot is not touching right now. Omit `hovered_in` and a swing
            // that lands high never produces a contact report, so its stance is
            // cut immediately, forever — that is the degenerate rapid-cycle
            // oscillator described in `physics_tuning_progress.md` §11.
            //
            // The `lost_contact_early` guard is also what keeps a foot that
            // legitimately leaves the floor just before its timer expires from
            // being double-counted.
            let lost_contact_early = if authority == GaitAuthority::Full && !leg.hovered_in {
                match contact {
                    Some(c) => !c.touching && leg.phase_timer > 0.0,
                    None => false,
                }
            } else {
                false
            };
            // Event-driven liftoff — two triggers, primary then emergency
            // (`plans/swing-phase-timing.md` §3):
            //
            // - `dcm_crossed` (PRIMARY): step when the DCM `xi` crosses this
            //   stance foot's forward support edge **minus one swing's travel**
            //   (`v·T_nominal`). The swing leg needs that much lead time: if we
            //   only lift off when `xi` reaches the toe edge, the flight takes
            //   another `v·T` before the new foot can plant, and the CoM has
            //   passed the plant before it arrives — the "feet never lead the
            //   hip" failure. `margin` is exactly that lead payment.
            // - `com_crossed` (EMERGENCY): the measured CoM has already
            //   advanced past the plant in the direction of motion. Late, but
            //   the right call at very low speed where the margin is tiny;
            //   logged as `trigger_reason == 3` so tuning can count how often
            //   it fires during normal walking (it should be ~never).
            //
            // The stance timer is a *nominal* scheduler and a fallover cap: it
            // still steps a stuck body whose `xi` never crosses, but it no
            // longer schedules the gait whenever a measurement is present.
            // `wants_step` (move intent) is required so the settle motion
            // never triggers a phantom step with no input held.
            let dir_sign = if v_desired > 0.0 { 1.0 } else { -1.0 };
            let dcm_crossed = if plan.mpc_trigger {
                match (plan.xi, plan.measured) {
                    (Some(xi), Some(m)) if v_desired.abs() > SWING_TRIGGER_SPEED => {
                        let margin = m.v_com * config.swing_duration_ts;
                        (xi - leg.plant_pos.x) * dir_sign >= config.toe_len - margin
                    }
                    _ => false,
                }
            } else {
                false
            };
            let com_crossed = match plan.measured {
                Some(m) => {
                    let rel = m.com_x - leg.plant_pos.x;
                    if v_desired.abs() > SWING_TRIGGER_SPEED {
                        (v_desired > 0.0 && rel > 0.0) || (v_desired < 0.0 && rel < 0.0)
                    } else {
                        false
                    }
                }
                None => false,
            };
            let stance_over =
                leg.phase_timer <= 0.0 || lost_contact_early || dcm_crossed || com_crossed;
            if stance_over && wants_step && !other_swinging {
                // Log which condition ended the stance (telemetry only).
                leg.trigger_reason = if dcm_crossed {
                    2
                } else if com_crossed {
                    3
                } else {
                    1
                };
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
                leg.early_lift = lost_contact_early;
                leg.swing_wait = 0.0;
                leg.flight_span = config.swing_duration_ts.max(f32::EPSILON);
                leg.swing_prog = 0.0;
                // New stance will get its own single chance to anchor.
                leg.plant_anchored = false;
            }
            None
        }
        GaitPhase::Swing => {
            leg.phase_timer -= dt;
            // Continuous re-planning (§6 schedule tracking): refresh the target
            // from the current hip position, velocity and remaining flight
            // time. Open-loop targets frozen at lift-off are what made sudden
            // stops land both feet on one side of the hip.
            leg.swing_to.x =
                plan_target_x(hip_x, v, v_desired, leg.phase_timer.max(0.0), config, plan);
            // §19.2.1 deadbeat swing timing: traverse the spline so the foot
            // reaches the capture point about when the CoM does, instead of on
            // the fixed swing timer. `T_remain = |x_land − x_com| / v_com`
            // (with a floor so near-zero speed doesn't divide by zero or swing
            // forever); the swing timer remains only as a fallover cap.
            let swing_frac = if plan.measured.is_some() {
                // Pace the arrival over the time the hip/CoM takes to cross the
                // residual distance to the plant, capped by the swing timer (the
                // fallover bound). Slow body → longer effective swing; fast body
                // → quicker swing.
                let t_target = {
                    let dist = (leg.swing_to.x - leg.plant_pos.x).abs();
                    let speed = v.abs().max(1.0);
                    (dist / speed)
                        .min(config.swing_duration_ts)
                        .max(f32::EPSILON)
                };
                // Fraction elapsed toward touchdown scaled by the paced time.
                (1.0 - (leg.phase_timer / t_target).clamp(0.0, 1.0)).clamp(0.0, 1.0)
            } else {
                // No measurement: fall back to the timer-paced profile exactly as
                // the pre-§19 resolver.
                let total = config.swing_duration_ts.max(f32::EPSILON);
                (1.0 - (leg.phase_timer / total).clamp(0.0, 1.0)).clamp(0.0, 1.0)
            };
            let t = swing_frac;
            leg.swing_prog = swing_frac;
            // Three-phase swing profile (plan §3): lift → advance → extend,
            // C¹-continuous through the IK foot path. Replaces the old single
            // symmetric parabola, which cleared no toe before advancing and
            // arrived at the plant at full swing speed.
            leg.foot_pos = swing_profile_foot(t, leg.swing_from, leg.swing_to, config.step_height);

            // Plan §6 swing extension (`Full` only): the swing is over when the
            // timer expires AND the foot has actually touched down (or the wait
            // cap is hit). A long flight from loop 2's vertical control
            // otherwise lands the pose while the foot is still in the air, and
            // the next stance begins with no support at all. Holding the final
            // profile point while waiting is deliberate: it parks the foot
            // exactly on the plant target and lets the leg PD close the last
            // millimetres.
            //
            // The cap must be generous relative to the report latency: the
            // contact batch is one physics tick stale, so a foot that lands on
            // the last frame of the window has not been reported yet. A cap near
            // zero cancels the mechanism instead of applying it.
            let waiting = if authority == GaitAuthority::Full {
                match contact {
                    Some(c) => leg.phase_timer <= 0.0 && !c.touching,
                    None => false,
                }
            } else {
                false
            };
            if waiting {
                leg.swing_wait += dt;
                leg.foot_pos = leg.swing_to;
                if leg.swing_wait < max_swing_wait.max(0.0) {
                    return None;
                }
                *swing_wait_count = swing_wait_count.saturating_add(1);
            }
            if leg.phase_timer <= 0.0 {
                leg.foot_pos = leg.swing_to;
                leg.plant_pos = leg.swing_to;
                leg.phase = GaitPhase::Stance;
                leg.phase_timer = config.stance_duration_ts;
                leg.swing_prog = 0.0;
                // Remember whether this stance can trust contact truth: a
                // touchdown that never got a contact report is a hover, and
                // `Full` must not then veto the whole stance on the very next
                // tick for the same absence of contact.
                leg.hovered_in = match contact {
                    Some(c) => !c.touching,
                    None => false,
                };
                leg.swing_wait = 0.0;
                // Touchdown event: signed hip→foot offset for the backward solve.
                return Some(leg.plant_pos.x - hip_x);
            }
            None
        }
    }
}
