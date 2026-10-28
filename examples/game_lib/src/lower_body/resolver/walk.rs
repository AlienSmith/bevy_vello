use super::attitude::*;
use super::ik::*;
use super::laws::*;
use super::placement::*;
use super::stepping::*;
use super::types::*;
use super::vertical::*;
use bevy::math::Vec2;
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
    let v_cmd = dir.x * config.target_speed.min(limits.v_plan_cap);

    // 2b. --- Rate-limit the reference (§19.2.2/§15.1) ---
    // Ramp the commanded speed into `v_ref` so the capture point and backward
    // solve see a reference that rises smoothly. A step `0 → 120` is exactly
    // the overreach that feeds loop 2 into a one-leg hop at startup.
    state.v_ref = approach(state.v_ref, v_cmd, config.accel * dt);
    let v_desired_scalar = state.v_ref;

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
    let xi = state
        .measured
        .and_then(|m| omega0(m.z_com, config.gravity).map(|w| m.com_x + m.v_com / w));
    let omega0 = state
        .measured
        .and_then(|m| omega0(m.z_com, config.gravity))
        .unwrap_or(0.0);
    let plan = StepPlan {
        support_x: None,
        stretch: config.dcm_propagate == DcmPropagate::Stretch,
        x_max,
        stopping,
        capture_offset,
        measured: state.measured,
        xi,
        omega0,
        mpc_trigger: config.mpc_trigger,
    };
    // Per-leg support pivot for the propagated placement law. The anti-phase
    // gate makes the opposite foot the ONLY possible pivot during this leg's
    // flight: it cannot lift off until this foot lands, so it is planted for
    // the whole remaining flight whether it is mid-stance or its timer has
    // already expired. `None` (body airborne, opposite leg swinging too, or
    // unmeasured) makes `plan_target_x` use the live state directly —
    // bit-identical pre-propagation behavior.
    let support_of = |other: &LegState| -> Option<f32> {
        (config.dcm_propagate == DcmPropagate::Land
            && !airborne
            && other.phase == GaitPhase::Stance
            && state.measured.is_some())
        .then_some(other.plant_pos.x)
    };
    let plan_l = StepPlan {
        support_x: support_of(&state.right),
        ..plan
    };
    let plan_r = StepPlan {
        support_x: support_of(&state.left),
        ..plan
    };

    // Loop 1 "in trouble" signal (plan §2): the reachability clamp fired on the
    // unclamped capture point — the foot the balance law demands is outside the
    // leg's geometric workspace `hip ± x_max`. `false` when no measurement is
    // active (pure kinematic fallback) or the capture point is within reach.
    let capture_saturated = match state.measured {
        Some(m) => match capture_point(m.com_x, m.v_com, v_desired_scalar, m.z_com, config.gravity)
        {
            Some(raw) => {
                let sat_a = raw < state.hip_pos.x - x_max || raw > state.hip_pos.x + x_max;
                // Report saturation for the active placement law (propagated
                // analytic deadbeat; `None` when `DCM_TC` is negative, i.e. the
                // law is off and the Raibert fallback runs instead). Same law,
                // same per-leg pivot and remaining-flight horizon as
                // `plan_target_x` — "in trouble" describes the placement that
                // is actually being steered, for either leg.
                let t_c = resolve_deadbeat_tc(config.dcm_tc, config.stance_duration_ts);
                let raw_b = |support_x: Option<f32>, t_flight: f32| -> Option<f32> {
                    let t_c_eff = if plan.stretch && t_c > 0.0 {
                        t_c + t_flight.max(0.0)
                    } else {
                        t_c
                    };
                    let (xi_in, v_in) = match support_x {
                        Some(p_s) if t_flight > 0.0 => {
                            propagate_to_landing(plan.xi, v, plan.omega0, p_s, t_flight)
                        }
                        _ => (plan.xi, v),
                    };
                    capture_point_deadbeat(xi_in, v_in, v_desired_scalar, plan.omega0, t_c_eff)
                };
                let t_l = if state.left.phase == GaitPhase::Swing {
                    state.left.phase_timer.max(0.0)
                } else {
                    config.swing_duration_ts
                };
                let t_r = if state.right.phase == GaitPhase::Swing {
                    state.right.phase_timer.max(0.0)
                } else {
                    config.swing_duration_ts
                };
                let out_of_reach =
                    |x: f32| x < state.hip_pos.x - x_max || x > state.hip_pos.x + x_max;
                let sat_b = [raw_b(plan_l.support_x, t_l), raw_b(plan_r.support_x, t_r)]
                    .into_iter()
                    .flatten()
                    .any(out_of_reach);
                sat_a || sat_b
            }
            None => false,
        },
        None => false,
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
    let mut lift_count_l = 0u32;
    let mut lift_count_r = 0u32;
    let mut wait_count_l = 0u32;
    let mut wait_count_r = 0u32;
    // Plan §6: per-leg ground truth, when the ECS contact layer supplies it and
    // the authority level actually wants it. `Timer` gets `None` so the pre-§6
    // behaviour is bit-identical rather than merely "equivalent".
    let authority = state.gait_authority;
    let contact_pair = if authority == GaitAuthority::Timer {
        None
    } else {
        state.foot_contact
    };
    let contact_left = contact_pair.map(|c| c[0]);
    let contact_right = contact_pair.map(|c| c[1]);
    let max_wait = state.max_swing_wait;
    let left_td = step_leg(
        &mut state.left,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.right.phase == GaitPhase::Swing,
        config,
        &plan_l,
        dt,
        contact_left,
        authority,
        max_wait,
        &mut lift_count_l,
        &mut wait_count_l,
    );
    let right_td = step_leg(
        &mut state.right,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.left.phase == GaitPhase::Swing,
        config,
        &plan_r,
        dt,
        contact_right,
        authority,
        max_wait,
        &mut lift_count_r,
        &mut wait_count_r,
    );
    state.early_lift_count[0] = state.early_lift_count[0].saturating_add(lift_count_l);
    state.early_lift_count[1] = state.early_lift_count[1].saturating_add(lift_count_r);
    state.swing_wait_count[0] = state.swing_wait_count[0].saturating_add(wait_count_l);
    state.swing_wait_count[1] = state.swing_wait_count[1].saturating_add(wait_count_r);

    // 6. --- Backward solve at touchdown (foot → hip) ---
    // Invert the *actual* plant offset to get the hip velocity whose next
    // forward plan lands exactly on it, then blend with gain `beta` (1 =
    // deadbeat). Skipped while stopping: capture plants encode "brake to rest
    // here", which the cruise inversion would misread as a velocity command.
    //
    // Stage 1 fulfillment gate (`plans/walk-improvement-plan.md`): the deadbeat
    // inversion is correct control only while the landing error is noise. Under
    // the zero-lead cruise law the error is *systematic* (feet land behind the
    // hip), and inverting a systematic lag through `beta = 1` writes a reversed
    // hip velocity — the measured walk-backwards oscillation. `td_solve_floor`
    // forbids that sign flip: the inversion may brake the hip down to
    // `floor·v_desired` but never past it, so direction changes go through
    // the *command*, not through landing error. (`hip_pos.x` is anchored to
    // the physical P3 by the ECS layer, so a real lag shows up in `delta`
    // directly; a plan-vs-achieved tolerance at the landing tick would be
    // vacuous because the kinematic plant equals the same-tick plan.)
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
            let v_sup_eff = apply_support_floor(v_support, v_desired_scalar, config.td_solve_floor);
            let beta = config.beta.clamp(0.0, 1.0);
            v = (v + beta * (v_sup_eff - v)).clamp(-v_ceil, v_ceil);
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

    // 6c. --- Attitude (loop 3) ---
    // Derive the measured hip horizontal acceleration by finite-differencing
    // the resolver's own integrated `hip_vel.x` (the physics-like tilt source:
    // start/stop walking leans the torso, ballistic flight does not). Then tick
    // the inverted-pendulum + PD and expose the state for the pose/debug.
    let hip_accel_x = match state.prev_hip_vel_x {
        Some(prev_vx) if dt > 0.0 => (state.hip_vel.x - prev_vx) / dt,
        _ => 0.0,
    };
    state.prev_hip_vel_x = Some(state.hip_vel.x);
    let attitude_torque = tick_attitude(&mut state.attitude, config, hip_accel_x, dt, airborne);

    // 7-8. --- IK ---

    // IK returns the *effective* foot (clamped to the reachable annulus), so the
    // rendered bones always satisfy thigh/calf length even when the hip briefly
    // outruns a pinned stance foot.
    let (left_knee_raw, left_foot) = solve_leg_ik(
        state.hip_pos,
        state.left.foot_pos,
        config.thigh_len,
        config.calf_len,
        config.bend_sign,
    );
    let (right_knee_raw, right_foot) = solve_leg_ik(
        state.hip_pos,
        state.right.foot_pos,
        config.thigh_len,
        config.calf_len,
        config.bend_sign,
    );
    // Pick the anatomical knee from the IK's two mirror solutions (facing right
    // ⇒ the more anterior one). The raw chord-relative `bend_sign` winds the
    // opposite way in vello's y-down frame (goat leg), and its world side flips
    // whenever the chord swings past vertical; selection on the facing x axis is
    // unambiguous in both y conventions and preserves both bone lengths exactly.
    let left_knee = select_anterior_knee(state.hip_pos, left_foot, left_knee_raw, config.facing);
    let right_knee = select_anterior_knee(state.hip_pos, right_foot, right_knee_raw, config.facing);

    // Finite-difference the pose into joint velocities for the leg PD's
    // velocity-error (kd) term. The hip reuses the exact integrated `hip_vel`
    // (loop-1 owns it); the knees/feet are differenced from the previous tick.
    let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };
    let (hip_vel, left_knee_vel, left_foot_vel, right_knee_vel, right_foot_vel) =
        match state.prev_positions {
            Some([ph, plk, plf, prk, prf]) => (
                state.hip_vel,
                (left_knee - plk) * inv_dt,
                (left_foot - plf) * inv_dt,
                (right_knee - prk) * inv_dt,
                (right_foot - prf) * inv_dt,
            ),
            None => (
                state.hip_vel,
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::ZERO,
            ),
        };

    // Upper-body point: `torso_len` above the hip along the tilt axis, in the
    // y-up travel frame (`hip + L·(sinθ, cosθ)`). At rest (θ = 0) it is exactly
    // `torso_len` above the hip; a forward lean (θ > 0) carries it in +x.
    let upper = state.hip_pos
        + Vec2::new(
            config.torso_len * state.attitude.theta.sin(),
            config.torso_len * state.attitude.theta.cos(),
        );

    let pose = LowerBodyPose {
        hip: state.hip_pos,
        left_knee,
        right_knee,
        left_foot,
        right_foot,
        upper,
        hip_vel,
        left_knee_vel,
        left_foot_vel,
        right_knee_vel,
        right_foot_vel,
    };

    state.prev_positions = Some([state.hip_pos, left_knee, left_foot, right_knee, right_foot]);

    // Loop 2 delivery (plan §4): map the vertical FSM phase to a virtual-leg
    // thrust command. During stance the banked stroke `Δl` is spent as knee/
    // ankle EXTENSION (the foot is pinned, so extension pushes the CoM up via
    // ground reaction); in Flight the knee RETRACTS for swing clearance; on
    // Landing it FLEXES to absorb. The ECS layer converts these px demands
    // into angular-constraint targets through the leg PD.
    let thrust = match state.vs.phase {
        VerticalPhase::Load | VerticalPhase::Launch => {
            // Push-off: the deeper the crouch stroke, the harder the extension.
            let extend = state.vs.delta_l.max(0.0);
            LegThrust {
                stance_extend: extend,
                flight_retract: 0.0,
                land_absorb: 0.0,
            }
        }
        VerticalPhase::Flight => {
            // Retract: tuck the knee, scaled by swing progress (more tuck as
            // the foot comes through).
            let prog = if state.vs.swing_period > f32::EPSILON {
                (state.vs.swing_t / state.vs.swing_period).clamp(0.0, 1.0)
            } else {
                1.0
            };
            LegThrust {
                stance_extend: 0.0,
                flight_retract: config.step_height * FLIGHT_SWING_LIFT_FRACTION * prog,
                land_absorb: 0.0,
            }
        }
        VerticalPhase::Land => {
            // Absorb: controlled flexion that decays as the landing settles.
            let absorb =
                state.vs.delta_l.max(0.0) * (1.0 - (state.vs.land_timer * 3.0).clamp(0.0, 1.0));
            LegThrust {
                stance_extend: 0.0,
                flight_retract: 0.0,
                land_absorb: absorb,
            }
        }
        VerticalPhase::Stand => LegThrust::default(),
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
        left_trigger_reason: state.left.trigger_reason,
        right_trigger_reason: state.right.trigger_reason,
        capture_saturated,
        v_phase: state.vs.phase,
        hip_height: h,
        hip_vel_y: state.hip_vel.y,
        h_desired: config.jump_apex,
        h_apex: state.vs.h_apex_prev,
        delta_l: state.vs.delta_l,
        thrust,
        theta: state.attitude.theta,
        omega: state.attitude.omega,
        attitude_torque,
        hip_accel_x,
    };

    (pose, debug)
}
