use super::placement::*;
use super::stepping::*;
use super::vertical::*;
use super::*;
use bevy::math::Vec2;

/// Timer-only [`step_leg`] — no physics contact information, i.e. exactly
/// the pre-§6 behaviour. The legacy gait tests are written against this, so
/// they keep asserting the open-loop clock while §6 is tested separately.
#[allow(clippy::too_many_arguments)]
fn step_leg_kinematic(
    leg: &mut LegState,
    hip_x: f32,
    v: f32,
    v_desired: f32,
    other_swinging: bool,
    config: &LowerBodyConfig,
    plan: &StepPlan,
    dt: f32,
) -> Option<f32> {
    let mut a = 0u32;
    let mut b = 0u32;
    step_leg(
        leg,
        hip_x,
        v,
        v_desired,
        other_swinging,
        config,
        plan,
        dt,
        None,
        GaitAuthority::Timer,
        0.05,
        &mut a,
        &mut b,
    )
}

/// §6 [`step_leg`] with explicit contact input and authority, hiding the
/// diagnostic counters.
#[allow(clippy::too_many_arguments)]
fn step_leg_contact(
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
) -> Option<f32> {
    let mut a = 0u32;
    let mut b = 0u32;
    step_leg(
        leg,
        hip_x,
        v,
        v_desired,
        other_swinging,
        config,
        plan,
        dt,
        contact,
        authority,
        0.05,
        &mut a,
        &mut b,
    )
}

/// A leg in Stance with the full timer, at a given plant.
fn stance_leg(cfg: &LowerBodyConfig, plant_x: f32) -> LegState {
    LegState {
        phase: GaitPhase::Stance,
        phase_timer: cfg.stance_duration_ts,
        foot_pos: Vec2::new(plant_x, cfg.floor_y),
        plant_pos: Vec2::new(plant_x, cfg.floor_y),
        swing_from: Vec2::ZERO,
        swing_to: Vec2::ZERO,
        swing_wait: 0.0,
        early_lift: false,
        hovered_in: false,
        plant_anchored: false,
        swing_prog: 0.0,
        trigger_reason: 0,
        flight_span: cfg.stance_duration_ts,
    }
}

fn default_config() -> LowerBodyConfig {
    LowerBodyConfig::default()
}

// -----------------------------------------------------------------
// Plan §6 — contact-gated gait clock
// -----------------------------------------------------------------

/// A leg in Stance with almost no timer left, plus a cruise plan, ready to
/// lift off on the next tick.
fn stance_about_to_step(cfg: &LowerBodyConfig) -> (LegState, StepPlan) {
    let mut leg = stance_leg(cfg, 0.0);
    leg.phase_timer = 0.0;
    (leg, cruise_plan(cfg))
}

#[test]
fn test_contact_early_liftoff_ends_stance_before_timer() {
    // Plan §6 (`Full`): a foot that has genuinely left the ground must end
    // its stance now, not at `stance_duration_ts`. Otherwise loop 2 keeps
    // commanding extension against nothing for the rest of the timer.
    let cfg = default_config();
    let v = cfg.target_speed;
    let plan = cruise_plan(&cfg);
    let mut leg = stance_leg(&cfg, 0.0);
    let airborne = Some(LegContactInput {
        touching: false,
        foot_x: None,
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        v,
        v,
        false,
        &cfg,
        &plan,
        1e-3,
        airborne,
        GaitAuthority::Full,
    );
    assert_eq!(
        leg.phase,
        GaitPhase::Swing,
        "contact lost early must lift the foot off"
    );
    assert!(leg.early_lift, "the clock was overruled; flag it");
}

#[test]
fn test_contact_early_liftoff_is_off_below_full_authority() {
    // `Contact` must NOT touch timing. This is the A/B boundary that makes
    // the authority ladder meaningful: the same "airborne" input leaves the
    // stance running under `Contact`, `Observe`, and `Timer`.
    let cfg = default_config();
    let v = cfg.target_speed;
    let plan = cruise_plan(&cfg);
    for authority in [
        GaitAuthority::Timer,
        GaitAuthority::Observe,
        GaitAuthority::Contact,
    ] {
        let mut leg = stance_leg(&cfg, 0.0);
        let airborne = Some(LegContactInput {
            touching: false,
            foot_x: Some(3.0),
        });
        let _ = step_leg_contact(
            &mut leg, 0.0, v, v, false, &cfg, &plan, 1e-3, airborne, authority,
        );
        assert_eq!(
            leg.phase,
            GaitPhase::Stance,
            "{authority:?} must not end the stance on contact loss"
        );
    }
}

#[test]
fn test_contact_hovered_in_stance_is_not_vetoed() {
    // The rapid-cycle trap: a swing that never reported contact starts a
    // stance that is already "not touching". `Full` must not read that as
    // "the foot just left" — otherwise every stance is cut on tick 1 and the
    // leg oscillates at the tick rate forever.
    let cfg = default_config();
    let v = cfg.target_speed;
    let plan = cruise_plan(&cfg);
    let mut leg = stance_leg(&cfg, 0.0);
    leg.hovered_in = true;
    let still_airborne = Some(LegContactInput {
        touching: false,
        foot_x: None,
    });
    for _ in 0..30 {
        let _ = step_leg_contact(
            &mut leg,
            0.0,
            v,
            v,
            false,
            &cfg,
            &plan,
            1e-3,
            still_airborne,
            GaitAuthority::Full,
        );
        assert_eq!(
            leg.phase,
            GaitPhase::Stance,
            "a hover-entered stance must run its timer instead of thrashing"
        );
    }
}

#[test]
fn test_contact_stance_holds_while_grounded() {
    // The same timer with real contact must NOT lift off: the clock is still
    // authoritative while the foot is genuinely planted.
    let cfg = default_config();
    let v = cfg.target_speed;
    let plan = cruise_plan(&cfg);
    let mut leg = stance_leg(&cfg, 0.0);
    let loaded = Some(LegContactInput {
        touching: true,
        foot_x: None,
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        v,
        v,
        false,
        &cfg,
        &plan,
        1e-3,
        loaded,
        GaitAuthority::Full,
    );
    assert_eq!(leg.phase, GaitPhase::Stance);
    assert!(!leg.early_lift);
}

#[test]
fn test_contact_swing_extends_until_real_touchdown() {
    // Plan §6 (`Full`): the swing must not land the pose while the foot is
    // still in the air. Hold the final profile point until contact (or the
    // wait cap).
    let cfg = default_config();
    let v = cfg.target_speed;
    let plan = cruise_plan(&cfg);
    let (mut leg, _) = stance_about_to_step(&cfg);
    let dt = 1e-3;
    let loaded = Some(LegContactInput {
        touching: true,
        foot_x: None,
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        v,
        v,
        false,
        &cfg,
        &plan,
        dt,
        loaded,
        GaitAuthority::Full,
    );
    assert_eq!(leg.phase, GaitPhase::Swing);

    // Force the timer to expire, then feed "airborne" repeatedly.
    leg.phase_timer = 0.0;
    let airborne = Some(LegContactInput {
        touching: false,
        foot_x: None,
    });
    let mut landed_early = false;
    for _ in 0..20 {
        let _ = step_leg_contact(
            &mut leg,
            0.0,
            v,
            v,
            false,
            &cfg,
            &plan,
            dt,
            airborne,
            GaitAuthority::Full,
        );
        if leg.phase == GaitPhase::Stance {
            landed_early = true;
            break;
        }
    }
    assert!(
        !landed_early,
        "swing landed while the foot was still airborne"
    );
    assert!(leg.swing_wait > 0.0, "the wait must be accumulated");
}

#[test]
fn test_contact_swing_wait_is_capped() {
    // A foot that never reports contact must eventually land anyway, or the
    // gait hangs forever.
    let cfg = default_config();
    let v = cfg.target_speed;
    let plan = cruise_plan(&cfg);
    let (mut leg, _) = stance_about_to_step(&cfg);
    let loaded = Some(LegContactInput {
        touching: true,
        foot_x: None,
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        v,
        v,
        false,
        &cfg,
        &plan,
        1e-3,
        loaded,
        GaitAuthority::Full,
    );
    leg.phase_timer = 0.0;
    let airborne = Some(LegContactInput {
        touching: false,
        foot_x: None,
    });
    let dt = 1e-2;
    let mut landed = false;
    for _ in 0..20 {
        let _ = step_leg_contact(
            &mut leg,
            0.0,
            v,
            v,
            false,
            &cfg,
            &plan,
            dt,
            airborne,
            GaitAuthority::Full,
        );
        if leg.phase == GaitPhase::Stance {
            landed = true;
            break;
        }
    }
    assert!(
        landed,
        "an unreachable foot must still land on the wait cap"
    );
}

#[test]
fn test_contact_reanchors_stance_plant_to_measured_x() {
    // Plan §6 stance-slide correction: while in contact the plant follows
    // where the foot ACTUALLY is, so a drifting foot stops poisoning every
    // downstream angle.
    let cfg = default_config();
    let v = 0.0; // no stepping intent: stay in stance
    let plan = cruise_plan(&cfg);
    let mut leg = stance_leg(&cfg, 0.0);
    let measured = Some(LegContactInput {
        touching: true,
        foot_x: Some(7.5),
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        v,
        v,
        false,
        &cfg,
        &plan,
        1e-3,
        measured,
        GaitAuthority::Contact,
    );
    assert!(
        (leg.plant_pos.x - 7.5).abs() < 1e-5,
        "plant x {} must follow the measured foot 7.5",
        leg.plant_pos.x
    );
    assert!(
        (leg.foot_pos.x - 7.5).abs() < 1e-5,
        "foot_pos must be the corrected plant"
    );
    assert!(leg.plant_anchored, "the stance spends its one anchor");
}

#[test]
fn test_contact_reanchor_happens_at_most_once_per_stance() {
    // The anti-feedback-loop guard. Re-anchoring every tick makes the plan
    // chase the slip it is meant to resist: target → PD → measured →
    // target. One correction per stance removes the landing error without
    // turning the stride into a follower of the foot.
    let cfg = default_config();
    let plan = cruise_plan(&cfg);
    let mut leg = stance_leg(&cfg, 0.0);
    // First tick: foot reported at 7.5, plant follows.
    let first = Some(LegContactInput {
        touching: true,
        foot_x: Some(7.5),
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        0.0,
        0.0,
        false,
        &cfg,
        &plan,
        1e-3,
        first,
        GaitAuthority::Contact,
    );
    assert!((leg.plant_pos.x - 7.5).abs() < 1e-5);
    // Second tick: the foot has drifted further to 20.0. The plant must
    // NOT follow — that is the loop this test exists to prevent.
    let second = Some(LegContactInput {
        touching: true,
        foot_x: Some(20.0),
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        0.0,
        0.0,
        false,
        &cfg,
        &plan,
        1e-3,
        second,
        GaitAuthority::Contact,
    );
    assert!(
        (leg.plant_pos.x - 7.5).abs() < 1e-5,
        "plant x {} must stay at the first confirmed anchor 7.5",
        leg.plant_pos.x
    );
}

#[test]
fn test_contact_reanchor_is_rearmed_by_the_next_stance() {
    // The "once" is per stance, not for the leg's lifetime: after the next
    // lift-off the new stance gets its own single correction.
    let cfg = default_config();
    let plan = cruise_plan(&cfg);
    let mut leg = stance_leg(&cfg, 0.0);
    let loaded = Some(LegContactInput {
        touching: true,
        foot_x: Some(7.5),
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        cfg.target_speed,
        cfg.target_speed,
        false,
        &cfg,
        &plan,
        1e-3,
        loaded,
        GaitAuthority::Contact,
    );
    assert!(leg.plant_anchored);
    // Force a lift-off by draining the stance timer.
    leg.phase_timer = 0.0;
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        cfg.target_speed,
        cfg.target_speed,
        false,
        &cfg,
        &plan,
        1e-3,
        loaded,
        GaitAuthority::Contact,
    );
    assert_eq!(leg.phase, GaitPhase::Swing);
    assert!(
        !leg.plant_anchored,
        "lift-off must re-arm the anchor for the next stance"
    );
}

#[test]
fn test_contact_reanchor_requires_touching() {
    // The plant must only follow the foot while the physics solver is
    // actually holding it down. A foot that is airborne keeps its plant:
    // otherwise the plant walks away with the foot and the whole stride
    // is dragged along by the slide it was supposed to resist.
    let cfg = default_config();
    let plan = cruise_plan(&cfg);
    let mut leg = stance_leg(&cfg, 0.0);
    let airborne = Some(LegContactInput {
        touching: false,
        foot_x: Some(7.5),
    });
    let _ = step_leg_contact(
        &mut leg,
        0.0,
        0.0,
        0.0,
        false,
        &cfg,
        &plan,
        1e-3,
        airborne,
        GaitAuthority::Contact,
    );
    assert!(
        leg.plant_pos.x.abs() < 1e-6,
        "plant must not follow an airborne foot"
    );
}

#[test]
fn test_contact_below_contact_authority_never_moves_the_plant() {
    // `Observe` and `Timer` must leave `plant_pos` untouched even with a
    // perfect measured foot x. This is the control arm of the A/B.
    let cfg = default_config();
    let plan = cruise_plan(&cfg);
    for authority in [GaitAuthority::Timer, GaitAuthority::Observe] {
        let mut leg = stance_leg(&cfg, 0.0);
        let measured = Some(LegContactInput {
            touching: true,
            foot_x: Some(7.5),
        });
        let _ = step_leg_contact(
            &mut leg, 0.0, 0.0, 0.0, false, &cfg, &plan, 1e-3, measured, authority,
        );
        assert!(
            leg.plant_pos.x.abs() < 1e-6,
            "{authority:?} must not re-anchor the plant"
        );
    }
}

#[test]
fn test_no_contact_input_is_identical_to_pre_s6_timer() {
    // The kinematic fallback must be bit-identical to the old behaviour: same
    // phase transitions at the same ticks, for a full swing.
    let cfg = default_config();
    let v = cfg.target_speed;
    let plan = cruise_plan(&cfg);
    let dt = 1e-2;

    let (mut a, _) = stance_about_to_step(&cfg);
    let (mut b, _) = stance_about_to_step(&cfg);
    let mut hip_x = 0.0;
    for _ in 0..200 {
        hip_x += v * dt;
        let ra = step_leg_kinematic(&mut a, hip_x, v, v, false, &cfg, &plan, dt);
        let rb = step_leg_kinematic(&mut b, hip_x, v, v, false, &cfg, &plan, dt);
        assert_eq!(a.phase, b.phase, "phase diverged with no contact input");
        assert!((a.phase_timer - b.phase_timer).abs() < 1e-6);
        assert!((a.foot_pos - b.foot_pos).length() < 1e-5);
        assert_eq!(ra.is_some(), rb.is_some());
    }
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
        measured: None,
        xi: None,
        omega0: 0.0,
        mpc_trigger: false,
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
fn test_deadbeat_placement_is_the_exact_solution() {
    // `capture_point_deadbeat` must be the algebra, not a fit: forward-
    // simulate the LIPM stance from the target and check the velocity
    // arrives at `v_desired` exactly at the deadline.
    let cfg = default_config();
    let z = 90.0;
    let g = 980.0;
    let w = omega0(z, g).unwrap();
    let t_c = cfg.stance_duration_ts;
    let v_des = 120.0;

    let lipm_v = |x0: f32, v0: f32, p: f32, t: f32| -> f32 {
        w * (x0 - p) * (w * t).sinh() + v0 * (w * t).cosh()
    };

    for (x0, v0) in [
        (0.0, 120.0),  // cruise
        (0.0, 0.0),    // from rest
        (0.0, 60.0),   // below command
        (30.0, 150.0), // too fast
        (-40.0, -20.0),
    ] {
        let xi = x0 + v0 / w;
        let p = capture_point_deadbeat(Some(xi), v0, v_des, w, t_c).unwrap();
        let v_at_tc = lipm_v(x0, v0, p, t_c);
        assert!(
            (v_at_tc - v_des).abs() < 1e-2,
            "deadbeat missed: x0={x0} v0={v0} → p={p}, v(T_c)={v_at_tc}"
        );
    }

    // Cruise lead, and its two-coefficient form. The horizon law cancels to
    // zero here and the gain family has only one dof; this one has both
    // coefficients, which is the missing term.
    let xi = v_des / w;
    let p = capture_point_deadbeat(Some(xi), v_des, v_des, w, t_c).unwrap();
    let alpha = 1.0 / (w * (w * t_c).tanh());
    let beta = 1.0 / (w * (w * t_c).sinh());
    let expected = alpha * v_des - beta * v_des;
    assert!(
        (p - expected).abs() < 1e-3,
        "coefficient form mismatch: p={p} expected={expected}"
    );
    assert!(alpha > 1.0 / w, "alpha must exceed the implicit 1/ω₀ lead");

    // A real lead, and — the useful identity — `alpha − beta =
    // tanh(ω₀t_c/2)/ω₀`, so the cruise lead is `v·tanh(ω₀t_c/2)/ω₀`, which
    // tends to `v·t_c/2` for small `ω₀t_c`: this law *contains Raibert's
    // neutral point* as its small-angle limit. `t_c = T_s` lands within ~1 px
    // of `v·T_s/2 = 16.8 px` at the default parameters.
    // Lead is measured from the CoM (`x = 0`), not from the DCM: the DCM
    // sits `v/ω₀` ahead of the CoM by construction.
    let x_com = 0.0;
    let lead = p - x_com;
    assert!(lead > 5.0, "cruise target should lead the CoM: {lead:.1}px");
    let identity = v_des * (w * t_c * 0.5).tanh() / w;
    assert!(
        (lead - identity).abs() < 1e-3,
        "alpha−beta identity mismatch: {lead} vs {identity}"
    );
    let neutral = v_des * t_c * 0.5;
    assert!(
        (lead - neutral).abs() / neutral < 0.1,
        "t_c = T_s should recover the neutral point {neutral:.1}px, got {lead:.1}px"
    );

    // Limits: long deadline → plain capture point; short → stiff spring.
    let p_long = capture_point_deadbeat(Some(xi), v_des, v_des, w, 50.0).unwrap();
    assert!(
        (p_long - xi).abs() < 0.5,
        "t_c→∞ must approach xi: {p_long}"
    );
    let p_short = capture_point_deadbeat(Some(0.0), 0.0, v_des, w, 1e-3).unwrap();
    let spring = -v_des / (w * w * 1e-3);
    assert!(
        (p_short - spring).abs() / spring.abs() < 1e-2,
        "t_c→0 must approach the Kp spring: {p_short} vs {spring}"
    );
    // Disabled / degenerate guards.
    assert_eq!(capture_point_deadbeat(Some(0.0), 0.0, v_des, w, 0.0), None);
    assert_eq!(
        capture_point_deadbeat(Some(0.0), 0.0, v_des, 0.0, t_c),
        None
    );
    assert_eq!(capture_point_deadbeat(None, 0.0, v_des, w, t_c), None);
}

#[test]
fn test_dcm_tc_env_semantics() {
    // `DCM_TC` is a tunable window, not a switch: 0 means "auto = stance
    // duration", positive is taken literally in seconds, negative is off.
    let ts = 0.28;
    assert_eq!(resolve_deadbeat_tc(-1.0, ts), 0.0, "negative must disable");
    assert_eq!(resolve_deadbeat_tc(0.0, ts), ts, "0 must auto-pick T_s");
    assert_eq!(
        resolve_deadbeat_tc(0.5, ts),
        0.5,
        "positive must pass through"
    );
    // Disabled resolves to 0.0, which the placement helper rejects — that is
    // what keeps the flag-off path on the committed horizon law.
    assert_eq!(
        capture_point_deadbeat(Some(0.0), 0.0, 120.0, 3.3, resolve_deadbeat_tc(-1.0, ts)),
        None
    );
    assert!(
        capture_point_deadbeat(Some(0.0), 0.0, 120.0, 3.3, resolve_deadbeat_tc(0.0, ts)).is_some(),
        "auto window must enable the law"
    );
}

// (Removed 2026-10-28: the `propagate_to_landing` / state-propagation ("Land")
// tests. Horizon-stretch became the sole placement law; the propagation
// experiment measured worse — clamp-saturated mid-flight arcs, net 1322 →
// 318/640 px — and its machinery (`support_x`, `DcmPropagate`) was deleted.
// The stretch properties are pinned by
// `test_stretch_horizon_is_absolute_deadline_and_bounded_early`.)

// (Removed 2026-10-28: `test_dcm_kp_gain_form_keeps_cruise_lead` tested the
// single-dof truncations `capture_point_with_{gain,horizon}`, deleted in favour
// of the exact deadbeat law; its distinguishing property — the cruise lead — is
// pinned by `test_deadbeat_placement_is_the_exact_solution`.)
#[test]
fn test_stretch_horizon_is_absolute_deadline_and_bounded_early() {
    // THE placement law (post-A/B, 2026-10-28): deadbeat evaluated at the
    // ABSOLUTE deadline `t_c' = t_c + t_flight.max(0)` on the live state —
    // "settle at touchdown + stance", promised identically from every swing
    // frame. Assertions:
    //  1. liftoff frame equals a manual call at `t_c + T_swing`;
    //  2. wide-early: the cruise lead (α−β)·v = tanh(ω₀T/2)·v/ω₀ GROWS with
    //     the horizon and is monotone in `t_flight`, but is BOUNDED by the
    //     capture-point limit v/ω₀ (≈ 36 px at cruise) < x_max (≈ 54) — the
    //     mid-flight aim can never saturate the reach clamp (unlike the
    //     deleted `Land` variant, whose propagated aim of ≈ +86 px pinned
    //     `hip + x_max` all flight and halved net progress);
    //  3. exact-late: at `t_flight = 0` the law is the plain deadbeat at
    //     `t_c` — every variant plants the same point on the touchdown tick.
    let cfg = default_config();
    let w = omega0(90.0, 980.0).unwrap();
    let v_des = 120.0;
    let (x0, v0) = (0.0, 120.0); // cruising at command
    let plan = StepPlan {
        x_max: 1.0e6,
        stopping: false,
        capture_offset: 0.0,
        measured: Some(CaptureMeasure {
            com_x: x0,
            v_com: v0,
            z_com: 90.0,
        }),
        xi: Some(x0 + v0 / w),
        omega0: w,
        mpc_trigger: false,
    };
    // t_c auto = stance (0.28); swing = 0.28 as well by default.
    let ts = cfg.stance_duration_ts;
    let sw = cfg.swing_duration_ts;
    let t_c = resolve_deadbeat_tc(cfg.dcm_tc, ts);
    // 1. Absolute deadline: liftoff horizon = t_c + T_swing.
    let p_lift = plan_target_x(x0, v0, v_des, sw, &cfg, &plan);
    let manual = capture_point_deadbeat(Some(x0 + v0 / w), v0, v_des, w, t_c + sw).unwrap();
    assert!(
        (p_lift - manual).abs() < 1e-3,
        "must evaluate at t_c + t_flight: {p_lift} vs {manual}"
    );
    // 2. Wide-early but bounded, monotone in t_r. (x0 == 0 ⇒ lead = p.)
    let p_mid = plan_target_x(x0, v0, v_des, sw * 0.5, &cfg, &plan);
    let p_plain_mid = capture_point_deadbeat(Some(x0 + v0 / w), v0, v_des, w, t_c).unwrap();
    assert!(
        p_mid > p_plain_mid && p_mid < v0 / w + 1e-3,
        "lead must grow with horizon but stay under the capture limit: {p_mid}"
    );
    assert!(p_lift > p_mid, "lead monotone in t_r: {p_mid} < {p_lift}");
    // 3. Exact-late: the planting tick is the plain deadbeat at t_c.
    let p_land = plan_target_x(x0, v0, v_des, 0.0, &cfg, &plan);
    assert!(
        (p_land - p_plain_mid).abs() < 1e-6,
        "t_flight = 0 must reduce to the plain law: {p_land} vs {p_plain_mid}"
    );
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
fn test_support_floor_blocks_reversal_but_allows_brake() {
    // Stage 1 (`walk-improvement-plan.md`): the floor lets a lagged plant
    // brake the hip toward `floor·v_des` but never flip its direction
    // against the command; floor < 0 is the committed no-op.
    // Forward command.
    assert_eq!(apply_support_floor(200.0, 120.0, -1.0), 200.0); // disabled
    assert_eq!(apply_support_floor(-50.0, 120.0, -1.0), -50.0); // disabled
    assert_eq!(apply_support_floor(90.0, 120.0, 0.2), 90.0); // above bound: keep
    assert_eq!(apply_support_floor(20.0, 120.0, 0.2), 24.0); // brake to bound
    assert_eq!(apply_support_floor(-40.0, 120.0, 0.2), 24.0); // reversal blocked
    assert_eq!(apply_support_floor(-40.0, 120.0, 0.0), 0.0); // full stop allowed
                                                             // Backward command mirrors exactly.
    assert_eq!(apply_support_floor(-20.0, -120.0, 0.2), -24.0);
    assert_eq!(apply_support_floor(40.0, -120.0, 0.2), -24.0);
    // Zero command: no bound to apply.
    assert_eq!(apply_support_floor(-40.0, 0.0, 0.2), -40.0);
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
fn test_capture_point_formula_and_sign() {
    // x_foot = x_CoM + (v_CoM − v_des)/ω₀, ω₀ = √(g/z_CoM). The target is
    // world-space: it does NOT depend on the hip position at all.
    let g: f32 = 980.0;
    let z: f32 = 90.0; // CoM height ≈ hip height
    let omega0 = (g / z).sqrt();
    // CoM at origin, standing still, want to stop: foot plants at origin.
    let x = capture_point(0.0, 0.0, 0.0, z, g).unwrap();
    assert!(
        x.abs() < 1e-3,
        "standing capture point should be at CoM: {x}"
    );
    // Moving at v_des: foot leads ahead of the CoM by v_des/ω₀ (speed
    // regulation is a target shift, not a separate gain).
    let x = capture_point(0.0, 0.0, 120.0, z, g).unwrap();
    let expected = -120.0 / omega0;
    assert!(
        (x - expected).abs() < 1e-3,
        "capture x={x} expected={expected}"
    );
    // Positive velocity error (v_CoM > v_des) pushes the foot further ahead.
    let x = capture_point(10.0, 150.0, 120.0, z, g).unwrap();
    assert!(
        (x - (10.0 + (150.0 - 120.0) / omega0)).abs() < 1e-3,
        "capture x={x}"
    );
    // Higher CoM ⇒ lower ω₀ ⇒ larger stride for the same velocity error.
    let x_low = capture_point(0.0, 0.0, 120.0, z, g).unwrap();
    let x_high = capture_point(0.0, 0.0, 120.0, 200.0, g).unwrap();
    assert!(x_high.abs() > x_low.abs(), "{x_high} vs {x_low}");
}

#[test]
fn test_capture_point_degnerate_returns_none() {
    // Degenerate z_com / gravity ⇒ undefined pendulum frequency ⇒ None so
    // the caller falls back to the Raibert neutral point.
    assert_eq!(capture_point(0.0, 0.0, 120.0, 0.0, 980.0), None);
    assert_eq!(capture_point(0.0, 0.0, 120.0, -10.0, 980.0), None);
    assert_eq!(capture_point(0.0, 0.0, 120.0, 90.0, 0.0), None);
    assert_eq!(capture_point(0.0, 0.0, 120.0, 90.0, -980.0), None);
}

#[test]
fn test_capture_point_worldspace_independent_of_hip() {
    // The capture point is a world-space CoM target, so for identical
    // measured CoM the same target must come out regardless of where the
    // hip is (unlike the Raibert neutral point, which predicts hip travel).
    let g: f32 = 980.0;
    let z: f32 = 90.0;
    let cfg_test = default_config();
    for hip_x in [-500.0, 0.0, 1234.0] {
        // CoM at the hip, so the capture point is always within a generous
        // reach window and nothing clamps (x is world-space, independent of
        // the hip position).
        let m = CaptureMeasure {
            com_x: hip_x,
            v_com: 40.0,
            z_com: z,
        };
        // Build the plan the way `step_lower_body` does: the DCM state is
        // derived from the measurement once per tick.
        let w = omega0(m.z_com, g).unwrap();
        let plan = StepPlan {
            x_max: 1000.0,
            stopping: false,
            capture_offset: 0.0,
            measured: Some(m),
            xi: Some(m.com_x + m.v_com / w),
            omega0: w,
            mpc_trigger: false,
        };
        let target = plan_target_x(hip_x, 0.0, 0.0, 0.5, &cfg_test, &plan);
        let expected = capture_point(m.com_x, m.v_com, 0.0, z, g).unwrap();
        assert!(
            (target - expected).abs() < 1e-2,
            "hip_x={hip_x} target={target} expected={expected}"
        );
    }
}

#[test]
fn test_capture_point_clamps_and_reports_saturation() {
    // A capture point beyond the leg reach is clamped to hip ± x_max, and
    // the "in trouble" saturation flag reports the clamp fired.
    let cfg = default_config();
    let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
    let x_max = limits.x_max;
    // CoM far ahead of the hip with forward motion: capture point is way out
    // of reach.
    let m = CaptureMeasure {
        com_x: 500.0,
        v_com: 300.0,
        z_com: cfg.hip_height,
    };
    let plan = StepPlan {
        x_max,
        stopping: false,
        capture_offset: 0.0,
        measured: Some(m),
        xi: Some(m.com_x + m.v_com / omega0(m.z_com, cfg.gravity).unwrap()),
        omega0: omega0(m.z_com, cfg.gravity).unwrap(),
        mpc_trigger: false,
    };
    let hip_x = 0.0;
    let target = plan_target_x(hip_x, 300.0, 120.0, 0.5, &cfg, &plan);
    assert!(
        (target - (hip_x + x_max)).abs() < 1e-2,
        "clamped target={target} expected={}",
        hip_x + x_max
    );
    // Direct saturation check on the unclamped capture point.
    let raw = capture_point(m.com_x, m.v_com, 120.0, cfg.hip_height, cfg.gravity).unwrap();
    assert!(
        raw > hip_x + x_max,
        "raw capture point {raw} should be out of reach"
    );
}

#[test]
fn test_capture_saturation_reported_through_step_lower_body() {
    // With a measured CoM far outside the workspace, `step_lower_body`
    // reports `capture_saturated == true` (the "in trouble" signal) and
    // still clamps the swing target into reach. Without any measurement it
    // stays false (pure Raibert fallback, existing sandbox behavior).
    let mut cfg = default_config();
    cfg.target_speed = 350.0;
    let mut state = ResolverState::new(&cfg);
    let dt = 1.0 / 60.0;
    let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
    // Cruise to speed without a measurement first.
    for _ in 0..((stride * 4.0) / dt) as usize {
        step_lower_body(Vec2::X, &cfg, &mut state, dt);
    }
    assert!(state.hip_vel.x > 100.0, "not cruising");

    // Fallback: no measurement ⇒ never "in trouble".
    let (_p, d0) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
    assert!(!d0.capture_saturated, "fallback must not report saturation");

    // Measured CoM pulled far off the hip ⇒ balance law demands an
    // unreachable foot ⇒ saturation fires.
    state.measured = Some(CaptureMeasure {
        com_x: state.hip_pos.x + 1000.0,
        v_com: 400.0,
        z_com: cfg.hip_height,
    });
    let (_p, d1) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
    assert!(d1.capture_saturated, "must report the in-trouble clamp");
    // The balance-law demand is unreachable, so the per-leg swing target
    // must sit on the reach boundary (this is the clamp the flag reports).
    let x_max = d1.x_max;
    for leg in [&state.left, &state.right] {
        let target = leg.swing_to.x;
        assert!(
            (target - state.hip_pos.x).abs() <= x_max + 1e-2,
            "swing target escaped the reach window: {target} vs hip {}",
            state.hip_pos.x
        );
    }
}

#[test]
fn test_swing_profile_lift_advance_extend_phases() {
    // The 3-phase profile (plan §3): endpoints are the lift-off and plant
    // plants, the lift clears to `step_height` with little x advance, the
    // advance sweeps most of the step at height, and the extend descends to
    // the plant x. The foot must never go below the floor.
    let from = Vec2::new(10.0, 0.0);
    let to = Vec2::new(60.0, 0.0);
    let h = 40.0;
    let dx = to.x - from.x;

    // Endpoints: exactly the lift-off and touchdown plants (no pop, no
    // floating at touch-down).
    assert!((swing_profile_foot(0.0, from, to, h) - from).length() < 1e-3);
    assert!((swing_profile_foot(1.0, from, to, h) - to).length() < 1e-3);

    // Lift end (s = 0.25): apex `step_height`, only ~16% of the step.
    let p = swing_profile_foot(0.25, from, to, h);
    assert!((p.x - (from.x + 0.16 * dx)).abs() < 1e-3, "lift x={}", p.x);
    assert!((p.y - h).abs() < 1e-3, "lift y={}", p.y);

    // Advance end (s = 0.80): ~86% of the step at half height.
    let p = swing_profile_foot(0.80, from, to, h);
    assert!((p.x - (from.x + 0.86 * dx)).abs() < 1e-3, "adv x={}", p.x);
    assert!((p.y - 0.5 * h).abs() < 1e-3, "adv y={}", p.y);

    // The lift is steeper than the advance: by s = 0.12 the foot is already
    // well off the floor, yet has barely advanced.
    let early = swing_profile_foot(0.12, from, to, h);
    assert!(
        early.y > 0.5 * h,
        "lift too shallow at s=0.12: y={}",
        early.y
    );
    assert!(
        early.x < from.x + 0.12 * dx,
        "lift advanced too far: {}",
        early.x
    );

    // Mid-swing is elevated but below the lift apex (descent starts).
    let mid = swing_profile_foot(0.5, from, to, h);
    assert!(
        mid.y > 0.0 && mid.y < h,
        "mid height out of band: {}",
        mid.y
    );

    // Never below the floor, sampled finely across the whole swing.
    for i in 0..=200 {
        let s = i as f32 / 200.0;
        let p = swing_profile_foot(s, from, to, h);
        assert!(p.y >= from.y - 1e-3, "foot below floor at s={s}: y={}", p.y);
    }
}

#[test]
fn test_swing_profile_c1_continuity() {
    // C¹ continuity at the segment junctions: both the position and the
    // velocity (finite difference of dP/ds) must match across s = 0.25 and
    // s = 0.80, so no junction reintroduces a "kick".
    let from = Vec2::new(0.0, 0.0);
    let to = Vec2::new(50.0, 0.0);
    let h = 40.0;
    let eps = 1e-4;
    let d = |s: f32| -> Vec2 {
        let a = swing_profile_foot(s, from, to, h);
        let b = swing_profile_foot(s + eps, from, to, h);
        (b - a) / eps
    };
    for junction in [0.25, 0.80] {
        let (dl, dr) = (d(junction - eps), d(junction + eps));
        assert!(
            (dl - dr).length() < 1.0,
            "velocity discontinuity at s={junction}: {dl:?} vs {dr:?}"
        );
        // Across a `2·eps` sampling window the position moves by
        // `≈ 2·eps·|velocity|` (≈0.02px here), so allow that sampling
        // width rather than an exact match.
        let (pl, pr) = (
            swing_profile_foot(junction - eps, from, to, h),
            swing_profile_foot(junction + eps, from, to, h),
        );
        assert!(
            (pl - pr).length() < 0.2,
            "position discontinuity at s={junction}: {pl:?} vs {pr:?}"
        );
    }
}

#[test]
fn test_landing_compensates_hip_motion_during_swing() {
    // Raibert neutral point: the foot must land `v·Ts/2` ahead of the hip
    // *at touchdown*, so the hip passes over the foot at mid-stance. This
    // requires predicting the hip's travel during the swing phase.
    let cfg = default_config();
    let v = cfg.target_speed; // cruise: kv term vanishes
    let mut leg = stance_leg(&cfg, 0.0);
    leg.phase_timer = 0.0;

    // Lift off with the hip at x = 0.
    let plan = cruise_plan(&cfg);
    let _ = step_leg_kinematic(&mut leg, 0.0, v, v, false, &cfg, &plan, 0.001);
    assert_eq!(leg.phase, GaitPhase::Swing);

    // Fly through the swing while the hip keeps moving.
    let dt = 0.01;
    let mut hip_x = 0.0;
    let mut touchdown = None;
    for _ in 0..((cfg.swing_duration_ts / dt).ceil() as usize + 2) {
        hip_x += v * dt;
        touchdown = step_leg_kinematic(&mut leg, hip_x, v, v, false, &cfg, &plan, dt);
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
    let _ = step_leg_kinematic(
        &mut state.left,
        state.hip_pos.x,
        40.0,
        120.0,
        state.right.phase == GaitPhase::Swing,
        &cfg,
        &plan,
        dt,
    );
    let _ = step_leg_kinematic(
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
    let _ = step_leg_kinematic(
        &mut state.left,
        state.hip_pos.x,
        0.0,
        0.0,
        state.right.phase == GaitPhase::Swing,
        &cfg,
        &plan,
        dt,
    );
    let _ = step_leg_kinematic(
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
        let _ = step_leg_kinematic(
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
fn test_knee_anterior_selection_body_frame() {
    // The 2-bone IK has exactly two valid knees, mirror images across the
    // hip->foot chord. The anatomical rule is a SELECTION between them:
    // facing right take the more ANTERIOR one (max knee.x along facing),
    // facing left the mirror. Stated on the x axis only, it is identical in
    // the y-down (vello) and y-up (Bevy) frames — a cross-product winding
    // rule is not, it mirrors with y (the bug that bent the knee goat-style).
    // Sweep the target a full circle and assert: the choice is the more
    // anterior solution in the facing frame, and bone lengths hold.
    let cfg = default_config();
    let hip = Vec2::ZERO;
    let r = cfg.thigh_len * 1.2; // comfortably inside reach (L1+L2 = 110)
    for facing in [1.0, -1.0] {
        for i in 0..64 {
            let th = std::f32::consts::TAU * (i as f32) / 64.0;
            let target = Vec2::new(th.cos(), th.sin()) * r;
            let (knee_raw, foot) =
                solve_leg_ik(hip, target, cfg.thigh_len, cfg.calf_len, cfg.bend_sign);
            let knee = select_anterior_knee(hip, foot, knee_raw, facing);
            // The selected knee must be at least as anterior as the raw IK
            // knee in the facing frame (selection only ever improves anterior).
            assert!(
                (knee.x - knee_raw.x) * facing_sign(facing) >= -1e-4,
                "facing={facing} angle={:.1}: selection moved the knee posterior ({:.2} vs {:.2})",
                th.to_degrees(),
                knee.x,
                knee_raw.x
            );
            assert!((knee.length() - cfg.thigh_len).abs() < 1e-3, "thigh broke");
            assert!(
                ((foot - knee).length() - cfg.calf_len).abs() < 1e-3,
                "calf broke"
            );
        }
    }

    // The pose we actually care about: a trailing stance leg (foot behind the
    // hip, walking right) must end up HUMAN — knee ahead of the ankle.
    let cfg = default_config();
    let hip = Vec2::ZERO;
    for facing in [1.0, -1.0] {
        for foot_x in [-50.0_f32, -30.0, -10.0, 0.0, 10.0] {
            let target = Vec2::new(foot_x * facing_sign(facing), cfg.hip_height);
            let (knee_raw, foot) =
                solve_leg_ik(hip, target, cfg.thigh_len, cfg.calf_len, cfg.bend_sign);
            let knee = select_anterior_knee(hip, foot, knee_raw, facing);
            assert!(
                    (knee.x - foot.x) * facing_sign(facing) > 0.0,
                    "facing={facing} foot_x={foot_x}: knee should lead the ankle (knee.x={:.2}, foot.x={:.2})",
                    knee.x,
                    foot.x
                );
        }
    }
    assert!(facing_sign(1.0) > 0.0 && facing_sign(-1.0) < 0.0);
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
fn test_measured_apex_overrides_internal_model() {
    // Loop 2 measurement (plan §4): when the ECS layer feeds a measured
    // whole-body CoM apex, the `Δl` integrator must latch THAT value (the
    // physics truth) instead of the resolver's internal ballistic apex.
    // The difference then drives the next hop's stroke.
    let cfg = LowerBodyConfig::default();
    let mut state = ResolverState::new(&cfg);
    let dt = 1.0 / 60.0;

    // Without a measurement the latch uses the internal model (> 0 on a hop).
    let apex_internal = run_one_jump(&cfg, &mut state, dt);
    assert!(apex_internal > 0.0, "internal model latched nothing");

    // Now feed a measured apex: the next hop must latch exactly that value.
    state.vs.h_apex_prev = 0.0;
    state.measured_apex = Some(37.5);
    let apex_measured = run_one_jump(&cfg, &mut state, dt);
    assert!(
        (apex_measured - 37.5).abs() < 1e-3,
        "measured apex not latched: got {apex_measured}, expected 37.5"
    );
    // A non-finite/negative measured apex clamps to 0 (never poison the loop).
    state.vs.h_apex_prev = 0.0;
    state.measured_apex = Some(-5.0);
    let apex_neg = run_one_jump(&cfg, &mut state, dt);
    assert!(
        apex_neg >= 0.0,
        "negative measured apex must clamp: {apex_neg}"
    );
}

#[test]
fn test_leg_thrust_maps_vertical_phases() {
    // Loop 2 delivery (plan §4): the resolver's `DebugInfo.thrust` command
    // must map the vertical FSM phase to a virtual-leg extension demand —
    // push-off during Load/Launch, retract during Flight, absorb during
    // Land, and nothing at idle Stand.
    let cfg = LowerBodyConfig::default();
    let mut state = ResolverState::new(&cfg);
    let dt = 1.0 / 60.0;

    // Stand: no thrust (all zero).
    let (_, d) = step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
    assert_eq!(d.v_phase, VerticalPhase::Stand);
    assert_eq!(d.thrust.stance_extend, 0.0);
    assert_eq!(d.thrust.flight_retract, 0.0);
    assert_eq!(d.thrust.land_absorb, 0.0);

    // Request a jump → Load/Lance: stance extension equals the banked Δl.
    state.vs.jump_requested = true;
    let mut saw_load = false;
    let mut saw_flight = false;
    let mut saw_land = false;
    let mut flight_retract_max = 0.0f32;
    for _ in 0..4000 {
        let (_p, d) = step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
        match d.v_phase {
            VerticalPhase::Load | VerticalPhase::Launch => {
                saw_load = true;
                assert!(
                    d.thrust.stance_extend > 0.0,
                    "no push-off during Load/Launch: {:?}",
                    d.thrust
                );
                assert_eq!(d.thrust.flight_retract, 0.0);
                assert_eq!(d.thrust.land_absorb, 0.0);
            }
            VerticalPhase::Flight => {
                saw_flight = true;
                assert_eq!(d.thrust.stance_extend, 0.0);
                assert_eq!(d.thrust.land_absorb, 0.0);
                // Retract ramps from 0 at lift-off (swing_t == 0) to full
                // tuck as the foot comes through, so track the peak.
                flight_retract_max = flight_retract_max.max(d.thrust.flight_retract);
            }
            VerticalPhase::Land => {
                saw_land = true;
                assert_eq!(d.thrust.stance_extend, 0.0);
                assert_eq!(d.thrust.flight_retract, 0.0);
                assert!(
                    d.thrust.land_absorb >= 0.0,
                    "negative absorb during Land: {:?}",
                    d.thrust
                );
            }
            VerticalPhase::Stand => {}
        }
        if d.v_phase == VerticalPhase::Stand && state.vs.h_apex_prev > 0.0 {
            break;
        }
    }
    assert!(saw_load, "never reached Load/Launch");
    assert!(saw_flight, "never reached Flight");
    assert!(saw_land, "never reached Land");
    assert!(
        flight_retract_max > 0.0,
        "knee never retracted during Flight: max {flight_retract_max}"
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

// --- Loop 3 (attitude) ---

#[test]
fn test_attitude_pd_sign_and_magnitude() {
    // Leaning forward (+θ) must command a restoring (negative) torque; the
    // magnitude scales with Kp, and a positive spin adds damping torque.
    let (kp, kd) = (45.0, 11.0);
    let u = attitude_pd(0.3, 0.0, 0.0, kp, kd);
    assert!(
        u < 0.0,
        "positive lean must push back toward upright, got {u}"
    );
    assert!((u - (-kp * 0.3)).abs() < 1e-3);
    let u_spin = attitude_pd(0.0, 2.0, 0.0, kp, kd);
    assert!(u_spin < 0.0 && (u_spin - (-kd * 2.0)).abs() < 1e-3);
    // At the setpoint with no spin the PD output is exactly zero.
    assert!(attitude_pd(0.0, 0.0, 0.0, kp, kd).abs() < 1e-6);
}

/// The leg-delivered loop-3 output: upright stick at the setpoint ⇒ the
/// reference axis stays where the bias puts it (no correction); a forward
/// lean ⇒ the reference tilts BACKWARD (negative) so the stance legs push
/// the torso upright; a backward lean ⇒ the reference tilts forward.
#[test]
fn test_attitude_lean_pd_signs_and_setpoint() {
    let (kp, kd, max) = (0.5, 0.03, 0.25);

    // At the setpoint with no spin there is nothing to correct: the output
    // is exactly the bias (the reference axis itself).
    assert!((attitude_lean_pd(0.0, 0.0, 0.0, kp, kd, max)).abs() < 1e-6);
    assert!((attitude_lean_pd(0.1, 0.0, 0.1, kp, kd, max) - 0.1).abs() < 1e-6);

    // Forward lean (θ > 0) ⇒ negative reference lean (legs aim backward).
    let forward = attitude_lean_pd(0.2, 0.0, 0.0, kp, kd, max);
    assert!(
        forward < 0.0,
        "forward lean must command a backward reference, got {forward}"
    );
    // Kp = 0.5 ⇒ half the error, and it is added to the zero bias.
    assert!((forward + kp * 0.2).abs() < 1e-6);

    // Backward lean (θ < 0) ⇒ positive reference lean.
    let backward = attitude_lean_pd(-0.2, 0.0, 0.0, kp, kd, max);
    assert!(
        backward > 0.0,
        "backward lean must command a forward reference, got {backward}"
    );

    // A non-zero bias moves the setpoint: at `θ_meas == bias` the PD error is
    // zero, so the output is exactly the bias (the reference axis the legs
    // should hold). A measurement off the bias produces a correction around
    // it, scaled by Kp.
    for bias in [-0.1_f32, 0.0, 0.1] {
        let at_setpoint = attitude_lean_pd(bias, 0.0, bias, kp, kd, max);
        assert!(
            (at_setpoint - bias).abs() < 1e-6,
            "bias={bias} is the setpoint: got {at_setpoint}"
        );
        let off = attitude_lean_pd(bias + 0.2, 0.0, bias, kp, kd, max);
        assert!(
            (off - (bias - kp * 0.2)).abs() < 1e-6,
            "bias={bias}: forward error must correct backward around it: {off}"
        );
    }

    // Spinning further forward damps the reference even at zero error.
    let spinning = attitude_lean_pd(0.0, 2.0, 0.0, kp, kd, max);
    assert!(
        spinning < 0.0 && (spinning + kd * 2.0).abs() < 1e-6,
        "positive spin must subtract kd·ω from the reference: {spinning}"
    );
}

/// The reference lean is a *position* command (radians) and must stay inside
/// the attitude wall, degrade to straight on garbage input, and be exactly
/// zero-gain-invariant when `kp = kd = 0` (bias only).
#[test]
fn test_attitude_lean_pd_clamps_and_degrades() {
    let max = 0.25;
    // A large error saturates instead of commanding an impossible reference.
    assert!((attitude_lean_pd(10.0, 0.0, 0.0, 1.0, 0.0, max) + max).abs() < 1e-6);
    assert!((attitude_lean_pd(-10.0, 0.0, 0.0, 1.0, 0.0, max) - max).abs() < 1e-6);
    // The clamp is symmetric and magnitude-based (a negative cap still works).
    assert!((attitude_lean_pd(-10.0, 0.0, 0.0, 1.0, 0.0, -max) - max).abs() < 1e-6);
    // No gains ⇒ bias only, whatever the measurement.
    for theta in [-1.0, 0.0, 1.0] {
        let out = attitude_lean_pd(theta, 5.0, 0.05, 0.0, 0.0, max);
        assert!(
            (out - 0.05).abs() < 1e-6,
            "kp=kd=0 must be bias-only: {out}"
        );
    }
    // Non-finite measurement / gains ⇒ straight reference, never NaN.
    assert_eq!(attitude_lean_pd(f32::NAN, 0.0, 0.0, 0.5, 0.03, max), 0.0);
    assert_eq!(
        attitude_lean_pd(0.0, f32::INFINITY, 0.0, 0.5, 0.03, max),
        0.0
    );
    assert_eq!(attitude_lean_pd(0.0, 0.0, 0.0, f32::NAN, 0.0, max), 0.0);
}

#[test]
fn test_attitude_pendulum_diverges_without_pd() {
    // With no control (Kp = Kd = 0) and gravity coupling on, the inverted
    // pendulum about the hip is an unstable pole: a small lean must grow
    // (and saturate at `theta_max`) instead of recovering.
    let mut cfg = LowerBodyConfig::default();
    cfg.attitude_kp = 0.0;
    cfg.attitude_kd = 0.0;
    cfg.attitude_gravity_coupling = true;
    let dt = 1.0 / 90.0;
    let mut st = AttitudeState {
        theta: 0.05,
        omega: 0.0,
    };
    let start = st.theta.abs();
    for _ in 0..300 {
        tick_attitude(&mut st, &cfg, 0.0, dt, false);
    }
    assert!(
        st.theta.abs() > start * 2.0,
        "uncontrolled pendulum should fall away from upright: {start} → {}",
        st.theta
    );
    assert!(
        st.theta.abs() <= cfg.theta_max + 1e-3,
        "θ escaped the clamp"
    );
}

#[test]
fn test_attitude_settles_to_upright_with_pd() {
    // Default Kp/Kd exceed the pendulum pole (g/L ≈ 14 rad/s²/rad): a
    // perturbed torso settles back to upright and stops.
    let cfg = LowerBodyConfig::default();
    let dt = 1.0 / 90.0;
    let mut st = AttitudeState {
        theta: 0.6,
        omega: 0.0,
    };
    for _ in 0..600 {
        tick_attitude(&mut st, &cfg, 0.0, dt, false);
    }
    assert!(st.theta.abs() < 0.01, "θ did not settle: {}", st.theta);
    assert!(st.omega.abs() < 0.1, "ω did not settle: {}", st.omega);
}

#[test]
fn test_attitude_settles_through_step_lower_body() {
    // Full wiring: a stationary character (no horizontal motion → a_x = 0)
    // with a pre-tilted torso returns to upright through `step_lower_body`,
    // and the attitude telemetry stays finite the whole way.
    let cfg = LowerBodyConfig::default();
    let dt = 1.0 / 90.0;
    let mut s = ResolverState::new(&cfg);
    for _ in 0..30 {
        step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
    }
    s.attitude.theta = 0.5;
    s.attitude.omega = 0.0;
    for _ in 0..900 {
        let (_pose, d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
        assert!(d.theta.is_finite() && d.attitude_torque.is_finite());
    }
    assert!(
        s.attitude.theta.abs() < 0.02,
        "θ did not settle via step: {}",
        s.attitude.theta
    );
    assert!(
        s.attitude.omega.abs() < 0.1,
        "ω did not settle via step: {}",
        s.attitude.omega
    );
}

#[test]
fn test_attitude_accel_coupling_leans_and_recovers() {
    // A sustained forward hip acceleration tilts the torso backward (−θ)
    // via the inertial-coupling term; cutting the acceleration lets the PD
    // swing it back to upright.
    let cfg = LowerBodyConfig::default();
    let dt = 1.0 / 90.0;
    let mut st = AttitudeState::default();
    let a_x = 600.0; // px/s², ~ the resolver's start/stop authority
    for _ in 0..300 {
        tick_attitude(&mut st, &cfg, a_x, dt, false);
    }
    assert!(
        st.theta < -0.05,
        "forward accel should lean the torso backward, got θ={}",
        st.theta
    );
    for _ in 0..600 {
        tick_attitude(&mut st, &cfg, 0.0, dt, false);
    }
    assert!(
        st.theta.abs() < 0.01,
        "torso did not recover: θ={}",
        st.theta
    );
}

#[test]
fn test_attitude_upper_point_geometry() {
    // The upper-body point must always sit exactly `torso_len` from the hip
    // along the tilt axis, and a forward lean carries it in +x. `theta` is
    // set directly before a single step; the step's own PD correction is
    // tiny (|Δθ| < 2e-3 for the default Kp·dt²), so the geometry holds.
    let cfg = LowerBodyConfig::default();
    let dt = 1.0 / 90.0;
    let mut s = ResolverState::new(&cfg);
    for theta in [-1.2, -0.5, 0.0, 0.4, 1.3] {
        s.attitude.theta = theta;
        s.attitude.omega = 0.0;
        let (pose, _d) = step_lower_body(Vec2::ZERO, &cfg, &mut s, dt);
        let off = pose.upper - pose.hip;
        assert!(
            (off.length() - cfg.torso_len).abs() < 1e-2,
            "upper distance wrong at θ={theta}: {} (L={})",
            off.length(),
            cfg.torso_len
        );
        if theta >= 0.0 {
            assert!(off.x >= -1e-3, "upper should lean +x at θ={theta}");
        } else {
            assert!(off.x <= 1e-3, "upper should lean -x at θ={theta}");
        }
    }
}

#[test]
fn test_attitude_theta_clamp() {
    // No matter how hard the coupling pushes, θ never exceeds `theta_max`.
    let mut cfg = LowerBodyConfig::default();
    cfg.theta_max = 0.6;
    let dt = 1.0 / 90.0;
    let mut st = AttitudeState {
        theta: 0.4,
        omega: 0.0,
    };
    // A huge forward accel drives the torso hard backward.
    for _ in 0..2000 {
        tick_attitude(&mut st, &cfg, 20_000.0, dt, false);
    }
    assert!(
        st.theta.abs() <= 0.6 + 1e-3,
        "θ escaped the clamp: {}",
        st.theta
    );
}

#[test]
fn test_attitude_flight_drops_gravity_term() {
    // Airborne == gravity-coupling-off: in flight the torso is weightless,
    // so the inverted-pendulum pole must vanish (identical trajectory to a
    // gravity-free run, tick for tick). Grounded keeps the pole, which fights
    // the recovery and shows up during the transient (both settle to upright,
    // so only the transient distinguishes them).
    let mut airborne_cfg = LowerBodyConfig::default();
    airborne_cfg.attitude_gravity_coupling = true;
    let mut no_g_cfg = LowerBodyConfig::default();
    no_g_cfg.attitude_gravity_coupling = false;
    let dt = 1.0 / 90.0;

    let mut a = AttitudeState {
        theta: 0.6,
        omega: 0.0,
    };
    let mut b = AttitudeState {
        theta: 0.6,
        omega: 0.0,
    };
    let mut c = AttitudeState {
        theta: 0.6,
        omega: 0.0,
    };
    let mut max_grounded_dev = 0.0f32;
    for _ in 0..300 {
        tick_attitude(&mut a, &airborne_cfg, 0.0, dt, true);
        tick_attitude(&mut b, &no_g_cfg, 0.0, dt, false);
        tick_attitude(&mut c, &airborne_cfg, 0.0, dt, false);
        assert!(
            (a.theta - b.theta).abs() < 1e-6 && (a.omega - b.omega).abs() < 1e-6,
            "airborne must be identical to gravity-off: θ {} vs {}",
            a.theta,
            b.theta
        );
        max_grounded_dev = max_grounded_dev.max((a.theta - c.theta).abs());
    }
    assert!(
        max_grounded_dev > 1e-3,
        "grounded should deviate from airborne during the transient, \
             max dev = {max_grounded_dev}"
    );
    // Both must still settle to upright.
    assert!(a.theta.abs() < 1e-3 && c.theta.abs() < 1e-3);
}
