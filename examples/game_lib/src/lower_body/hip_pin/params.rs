use bevy::prelude::*;

use crate::lower_body::resolver::{GaitAuthority, LowerBodyConfig};

#[derive(Resource)]
pub struct HipPinParams {
    /// Raibert resolver config.
    pub config: LowerBodyConfig,
    /// Speed at which compliance softening saturates, px/s.
    pub v_ref: f32,
    /// Compliance multiplier at full speed; `1.0` disables softening.
    pub soft_scale: f32,
    /// Fraction of gravity cancelled while grounded (`0..=1`).
    pub support_share: f32,
    /// External position constraint compliance at rest (lower = stiffer).
    pub compliance: f32,
    /// External position constraint damping.
    pub damping: f32,
    /// Also write the resolver's foot poses into the leg controllers.
    pub hip_drives_legs: bool,
    /// Vertical offset of the whole pinned body in px, positive = down.
    pub hip_drop: f32,
    /// Close the XPBD↔Raibert loop: write the solved `P3` back into the
    /// resolver state each step (`FEEDBACK=0` to disable).
    pub feedback: bool,
    /// Emit the `P1/P2/P3` external position constraints. Off for normal floor
    /// walking: the skeleton is then driven by angular constraints only.
    pub external_pin: bool,
    /// Allow the external pin as a short mid-air direction-change assist.
    pub air_control: bool,
    /// Leg angular-constraint compliance (lower = stiffer).
    pub leg_compliance: f32,
    /// Leg angular-constraint damping ratio (`1.0` = critical).
    pub leg_zeta: f32,
    /// Leg rest-angle chase fraction per tick (`1.0` = full target).
    pub leg_alpha: f32,
    /// Leg rest-angle chase rate cap, rad/s.
    pub leg_max_angle_rate: f32,
    /// Enable loop-3 attitude control.
    pub attitude_enabled: bool,
    /// Compliance of the loop-3 actuator when the external pin is active. In
    /// the default (angular-only) mode loop 3 rides the leg PD instead, so this
    /// is only used for the mid-air pin assist and for the `P1_P2_P3`
    /// posture hold.
    pub attitude_compliance: f32,
    /// Damping ratio of the angular-only loop-3 joint (`1.0` = critical).
    pub attitude_zeta: f32,
    /// Additional desired torso lean, radians (`+` = forward).
    pub attitude_target_bias: f32,
    /// Hard cap on the commanded torso lean, radians.
    pub attitude_max_angle: f32,
    /// Loop-3 proportional gain, forwarded to
    /// [`attitude_lean_pd`](crate::lower_body::resolver::attitude_lean_pd).
    ///
    /// Loop 3 is delivered by tilting the leg PD's world-upright reference axis,
    /// so this gain is in "radians of leg-reference lean per radian of measured
    /// torso error". `0.5` = half correction (gentle). Raise toward 1.0+ to
    /// stiffen the torso, but watch for foot chatter — the legs are the only
    /// actuator and the correction competes with the swing/foot-placement
    /// targets.
    pub attitude_lean_kp: f32,
    /// Loop-3 derivative gain (radians of leg-reference lean per rad/s of
    /// measured torso rate). Damps the reference tilt; `0.0` disables.
    pub attitude_lean_kd: f32,
    /// EXPERIMENT (`VEL_FEEDBACK=1`, default off): overwrite the fiction hip
    /// velocity `state.hip_vel.x` with the measured `P3vx` every tick — the
    /// hard velocity feedback reverted in bf65865 (A/B: walk died, net +6 px,
    /// thrashing ±140). Re-added behind a flag to re-measure its behavior now
    /// that the `PLAN_MEAS_V` placement brake exists. See the NOTE in
    /// `drive_hip_pin`.
    pub vel_feedback: bool,
    /// Boot FSM (`BOOT_LUNGE`): magnitude of the open-loop launch lean,
    /// radians. Written into the hip-joint reference (sign-corrected in
    /// `drive_hip_pin`) for the lunge window while the balance control is
    /// idle. `BOOT_LEAN` env; default 0.30 (~17°).
    pub boot_lean: f32,
    /// How much authority the plan §6 contact layer has over the gait clock.
    /// `GAIT_AUTHORITY=timer|observe|contact|full` (default `timer`).
    ///
    /// Written into [`ResolverState::gait_authority`] every tick, so it is
    /// switchable at runtime exactly like the other tuning knobs. Kept here
    /// rather than in `LowerBodyConfig` because the config is shared by every
    /// character and is copied by value; the authority is per-pin state.
    pub gait_authority: GaitAuthority,
}

/// Parse a `GAIT_AUTHORITY` value. Accepts the enum name or a single-letter /
/// numeric prefix, so `c`, `contact`, `Contact`, and `2` all mean
/// [`GaitAuthority::Contact`]. Anything unrecognised falls back to `Timer`,
/// which is the safe control arm.
pub fn parse_gait_authority(raw: &str) -> GaitAuthority {
    match raw.trim().to_ascii_lowercase().as_str() {
        "full" | "f" | "3" => GaitAuthority::Full,
        "contact" | "c" | "2" => GaitAuthority::Contact,
        "observe" | "o" | "1" => GaitAuthority::Observe,
        _ => GaitAuthority::Timer,
    }
}

impl HipPinParams {
    /// Read the shared harness/tuning environment: `TARGET_SPEED`, `LEGS`,
    /// `COMPLIANCE`, `DAMPING`, `HIP_DROP`, `SUPPORT_SHARE`, `FEEDBACK`,
    /// `EXTERNAL_PIN`, `AIR_CONTROL`, `LEG_COMPLIANCE`, `LEG_ZETA`,
    /// `LEG_ALPHA`, `LEG_MAX_ANG_RATE`, `ATTITUDE`, `ATT_COMPLIANCE`,
    /// `ATT_ZETA`, `ATT_TARGET_DEG`, `ATT_MAX_DEG`, `ATT_LEAN_KP`,
    /// `ATT_LEAN_KD`, and the gait timing / swing-arc shape: `STANCE`,
    /// `SWING`, `STEP_HEIGHT`.
    pub fn from_env() -> Self {
        let mut p = Self::default();
        p.config.target_speed = env_f32("TARGET_SPEED", p.config.target_speed);
        p.vel_feedback = env_bool("VEL_FEEDBACK", p.vel_feedback);
        // Gait shape. `SWING` shorter = the foot must travel the step in less
        // time (a faster swing); `STEP_HEIGHT` is the swing arc apex above the
        // plant line, i.e. how high the foot lifts to clear the floor.
        p.config.stance_duration_ts = env_f32("STANCE", p.config.stance_duration_ts).max(1e-3);
        p.config.swing_duration_ts = env_f32("SWING", p.config.swing_duration_ts).max(1e-3);
        p.config.step_height = env_f32("STEP_HEIGHT", p.config.step_height).max(0.0);
        // Event-driven gait (`plans/swing-phase-timing.md` §3/§4): the
        // support-edge proxy for the DCM-margin liftoff trigger.
        // Analytic deadbeat placement (`DCM_TC`): the exact two-coefficient law
        // `p = xi + (v·e^(−ω₀·t_c) − v_des)/(ω₀·sinh(ω₀·t_c))` — the ONLY
        // runtime placement law. `t_c` is the convergence deadline in seconds;
        // `DCM_TC=0` (default) auto-picks the stance duration; negative
        // disables it (Raibert neutral-point fallback). The former single-dof
        // truncations (`T_CONV` horizon, `DCM_KP` gain) were removed 2026-10-28
        // — deadbeat contains both as its `t_c → ∞` / `t_c → 0` limits.
        p.config.dcm_tc = env_f32("DCM_TC", p.config.dcm_tc);
        // (Removed 2026-10-28: the `PROPAGATE=off|stretch|land` A/B switch.
        // Horizon-stretch — `t_c' = t_c + t_flight`, the absolute deadline — is
        // now THE placement law; the receding `off` is the as-if-now fiction
        // and `land` saturated the reach clamp mid-flight.)
        p.config.toe_len = env_f32("TOE_LEN", p.config.toe_len).max(0.0);
        // §3 DCM-margin liftoff trigger. Off by default (the committed gait);
        // see `LowerBodyConfig::mpc_trigger` for why it stays experimental
        // until the §4 exponential placement lands.
        p.config.mpc_trigger = env_bool("MPC_TRIGGER", p.config.mpc_trigger);
        // Stage 1 sign-flip floor (`plans/walk-improvement-plan.md`): the
        // touchdown backward solve may brake only down to TD_FLOOR·v_des,
        // never past it (no reversal against the command). Negative = off
        // (committed gait). Try TD_FLOOR=0.2.
        p.config.td_solve_floor = env_f32("TD_FLOOR", p.config.td_solve_floor);
        // Boot FSM (Stage 4d): stand→lunge→walk (`BOOT_LUNGE=1`). Default off
        // ⇒ committed behavior, bit-identical. `BOOT_LUNGE_HOLD=1` is the
        // falsification run: the lunge window never releases, so the feet stay
        // locked after the hip is thrown forward — the character MUST fall over
        // (proves the Walk switch is the real gait door). See BootPhase.
        p.config.boot_lunge = env_bool("BOOT_LUNGE", p.config.boot_lunge);
        p.config.boot_lunge_hold = env_bool("BOOT_LUNGE_HOLD", p.config.boot_lunge_hold);
        p.config.boot_lunge_v = env_f32("BOOT_LUNGE_V", p.config.boot_lunge_v).max(0.0);
        // Route measured `v_com` into the deadbeat LEAD term (the brake is
        // computed against the TRUE speed, not the fiction). Default off ⇒
        // committed gait bit-identical. Pairs with the boot kick to stop the
        // post-launch cruise runaway.
        p.config.plan_meas_v = env_bool("PLAN_MEAS_V", p.config.plan_meas_v);
        p.config.v_meas_tau = env_f32("V_MEAS_TAU", p.config.v_meas_tau).max(1e-3);
        p.compliance = env_f32("COMPLIANCE", p.compliance);
        p.damping = env_f32("DAMPING", p.damping);
        p.hip_drop = env_f32("HIP_DROP", p.hip_drop);
        p.support_share = env_f32("SUPPORT_SHARE", p.support_share).clamp(0.0, 1.0);
        p.hip_drives_legs = !matches!(
            std::env::var("LEGS").ok().as_deref().map(str::trim),
            Some("off") | Some("0") | Some("false")
        );
        p.feedback = env_bool("FEEDBACK", true);
        p.external_pin = env_bool("EXTERNAL_PIN", p.external_pin);
        p.air_control = env_bool("AIR_CONTROL", p.air_control);
        p.leg_compliance = env_f32("LEG_COMPLIANCE", p.leg_compliance).max(1e-12);
        p.leg_zeta = env_f32("LEG_ZETA", p.leg_zeta).max(0.0);
        p.leg_alpha = env_f32("LEG_ALPHA", p.leg_alpha).clamp(0.0, 1.0);
        p.leg_max_angle_rate = env_f32("LEG_MAX_ANG_RATE", p.leg_max_angle_rate).max(0.0);
        p.attitude_enabled = env_bool("ATTITUDE", p.attitude_enabled);
        p.attitude_compliance = env_f32("ATT_COMPLIANCE", p.attitude_compliance).max(1e-12);
        p.attitude_zeta = env_f32("ATT_ZETA", p.attitude_zeta).max(0.0);
        p.attitude_target_bias =
            env_f32("ATT_TARGET_DEG", p.attitude_target_bias.to_degrees()).to_radians();
        p.attitude_max_angle = env_f32("ATT_MAX_DEG", p.attitude_max_angle.to_degrees())
            .to_radians()
            .max(0.0);
        p.attitude_lean_kp = env_f32("ATT_LEAN_KP", p.attitude_lean_kp).max(0.0);
        p.attitude_lean_kd = env_f32("ATT_LEAN_KD", p.attitude_lean_kd).max(0.0);
        p.boot_lean = env_f32("BOOT_LEAN", p.boot_lean).max(0.0);
        if let Ok(raw) = std::env::var("GAIT_AUTHORITY") {
            p.gait_authority = parse_gait_authority(&raw);
        }
        // Facing direction for the knee winding pin (+1 = facing +x/right, the
        // default; -1 = facing -x/left). `FACING=left`/`-1` mirrors the bend side.
        p.config.facing = match std::env::var("FACING").ok().as_deref().map(str::trim) {
            Some("left") | Some("-1") | Some("-") => -1.0,
            _ => 1.0,
        };
        p
    }
}

/// Parse a float env var; malformed or missing ⇒ `default`.
fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|s| s.trim().parse::<f32>().ok())
        .unwrap_or(default)
}

/// Parse a boolean env var (`1`/`true`/`yes`/`on` ⇒ true); missing ⇒ `default`.
fn env_bool(key: &str, default: bool) -> bool {
    match std::env::var(key).ok().as_deref().map(str::trim) {
        Some("1") | Some("true") | Some("yes") | Some("on") => true,
        Some("0") | Some("false") | Some("no") | Some("off") => false,
        _ => default,
    }
}

/// Ground-contact state gating the pin's vertical authority.
///
/// Per-foot ground truth (plan §6).
///
/// The collision batch gives per-pair entity ids, contact points and normals;

impl Default for HipPinParams {
    fn default() -> Self {
        Self {
            config: LowerBodyConfig::default(),
            v_ref: 300.0,
            soft_scale: 1.0,
            support_share: 0.5,
            compliance: 1e-7,
            damping: 0.01,
            hip_drives_legs: true,
            hip_drop: 0.0,
            feedback: true,
            // Normal walking is angular-only; the external pin is reserved for
            // explicit mid-air direction changes.
            external_pin: false,
            air_control: true,
            // The pin OWNS the leg angular compliance and rewrites it every
            // frame (see `drive_hip_pin`), so the asset's leg `Angular`
            // compliance is dead at runtime — THIS is the value that matters.
            //
            // `1e-6` is the edge of stability, not a soft choice: the measured
            // sweep (`tools/physics-qa/sweep_leg_compliance.py`,
            // `physics_tuning_progress.md` §8) shows 5e-7 and below drive the
            // body BACKWARDS at up to -193 px/s with the pinned frame tearing
            // (`rig max_rel_dev` 0.05 -> 0.38). Stiffening the loop-3 attitude
            // (`ATT_COMPLIANCE`) or damping the rest-angle chase (`LEG_ALPHA`,
            // `LEG_MAX_ANG_RATE`) does not tame it. So the stiffness knob is
            // not the fix for the non-stepping gait until the per-frame
            // rest-angle jump (alpha 1.0, rate 1e6) is solved properly.
            // `LEG_COMPLIANCE` still overrides for experiments.
            leg_compliance: 1e-6,
            leg_zeta: 1.0,
            leg_alpha: 1.0,
            leg_max_angle_rate: 1e6,
            attitude_enabled: true,
            attitude_compliance: 1e-7,
            attitude_zeta: 1.0,
            attitude_target_bias: 0.0,
            attitude_max_angle: 0.25, // ~14°, inside the 15° joint wall
            // Loop 3 now rides the leg PD's world-upright reference axis, so
            // these are *leg-reference* gains, not torques. `0.5` corrects half
            // the measured lean in one tick; `0.03` damps the reference tilt.
            attitude_lean_kp: 0.5,
            attitude_lean_kd: 0.03,
            boot_lean: 0.30,
            vel_feedback: false,
            // §6 is opt-in and off by default: see `GaitAuthority`.
            gait_authority: GaitAuthority::Timer,
        }
    }
}
