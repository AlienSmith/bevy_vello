use bevy::math::Vec2;

use super::placement::CaptureMeasure;
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
    /// Analytic (deadbeat) convergence window for foot placement, seconds.
    ///
    /// The exact LIPM solution of "with the foot planted, reach `v_desired`
    /// after `t_c` seconds" is
    ///
    /// ```text
    /// p = xi + (v·e^(−ω₀·t_c) − v_desired) / (ω₀·sinh(ω₀·t_c))
    /// ```
    ///
    /// i.e. `p = x + α·v − β·v_desired` with `α = 1/(ω₀·tanh(ω₀t_c))` and
    /// `β = 1/(ω₀·sinh(ω₀t_c))`. Two coefficients from ONE physical parameter —
    /// the single-dof truncations (horizon form, `ξ − Kp·(v_des − v)` gain form)
    /// could get the velocity gain right or the lead right, not both, and were
    /// removed 2026-10-28.
    ///
    /// `t_c` is a **deadline**, so the physical choice is the stance duration
    /// `t_c = T_s` (`DCM_TC=0` → auto). `t_c → ∞` recovers the plain capture
    /// point `p = ξ` (asymptotic stop); `t_c → 0` recovers the stiff
    /// velocity-error spring `p = x + (v − v_des)/(ω₀²·t_c)`.
    ///
    /// Env: `DCM_TC=<seconds>` tunes the window directly; `DCM_TC=0` (the
    /// default) auto-picks the stance duration; negative disables this law so
    /// the Raibert neutral-point fallback runs unchanged.
    pub dcm_tc: f32,
    /// Foot half-length ahead of the plant anchor, px (support-edge proxy for
    /// the §3 DCM-margin trigger; there are no heel/toe particles yet).
    pub toe_len: f32,
    /// Enable the §3 DCM-margin liftoff trigger. Off (default) keeps the
    /// committed `com_crossed`-only gait. See [`StepPlan::mpc_trigger`] for why
    /// this is gated until the §4 exponential placement lands.
    pub mpc_trigger: bool,
    /// Stage 1 sign-flip floor (`plans/walk-improvement-plan.md`), as a
    /// fraction of `v_desired`: the touchdown backward solve may brake the
    /// hip toward this floor but never past it — a lagged plant (the zero-lead
    /// law's systematic landing error) otherwise gets *inverted* into a
    /// reversed velocity command, the measured walk-backwards oscillator.
    /// Negative (default) = disabled, committed behavior. (A plan-vs-achieved
    /// *tolerance* gate was considered and removed: at the landing tick the
    /// kinematic plant equals the same-tick plan by construction, so such a
    /// check is vacuous; the physical lag enters through `delta` itself,
    /// because `hip_pos` is anchored to the real P3 — the floor is the
    /// mechanism that reads it.)
    pub td_solve_floor: f32,
    /// Swing arc apex height above the plant/floor line, px.
    pub step_height: f32,
    /// World y of the flat floor line.
    pub floor_y: f32,
    /// Knee bend direction: +1 = knee in front of the hip→foot line, -1 = behind.
    pub bend_sign: f32,
    /// Which way the body faces: +1 = facing +x (right, default), -1 = facing -x.
    /// The knee's **world winding order** is pinned to this, not to the hip→foot
    /// chord: facing right the knee must always sit clockwise (in screen terms)
    /// around the hip→foot chord, facing left counter-clockwise. The chord-relative
    /// `bend_sign` alone cannot express that — when the chord swings past vertical
    /// (deep swing tuck, saturated reach near the `acos` singularity) the same
    /// `bend_sign` produces the opposite world side, which is what made the leg
    /// "sometimes bend left, sometimes right" while walking right.
    pub facing: f32,
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

    // --- Attitude (loop 3) knobs. See `plans/raibert_sandbox_attitude_loop.md`. ---
    /// Upper-body pivot-to-COM distance above the hip, px.
    pub torso_len: f32,
    /// Rotational spring gain for the attitude PD, rad/s² per rad.
    pub attitude_kp: f32,
    /// Rotational damper gain for the attitude PD, 1/s.
    pub attitude_kd: f32,
    /// Gain (0..1) on the hip-accel inertial-coupling term (the tilt source).
    pub attitude_accel_coupling: f32,
    /// Toggle the destabilizing gravity term (true = inverted-pendulum pole).
    pub attitude_gravity_coupling: bool,
    /// Tilt clamp, rad (default 1.30 ≈ 75° — never spin past "fallen over").
    pub theta_max: f32,

    // --- Boot FSM (`BOOT_LUNGE`, Stage 4d). All `false` = committed behavior. ---
    /// Stand→lunge→walk boot switch. `false` (default): the whole FSM is inert
    /// and the resolver is bit-identical to the committed build. `true`: the
    /// character starts locked (`Stand`), converts the first move intent into a
    /// short scripted hip lunge, and only then opens the door to the gait.
    pub boot_lunge: bool,
    /// Falsification knob (`BOOT_LUNGE_HOLD`): the `Lunge` window never
    /// releases — feet stay locked after the hip has been thrown forward at
    /// `BOOT_LUNGE_V`. The character MUST fall over; if it does not, the lock
    /// is not airtight and the `Walk` switch is doing nothing.
    pub boot_lunge_hold: bool,
    /// Launch hip-translation speed during the Lunge window, px/s
    /// (`BOOT_LUNGE_V`). The kick is delivered through the P3 external
    /// position constraint (see `drive_hip_pin`). Default 160.
    pub boot_lunge_v: f32,
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
            toe_len: 8.0,
            mpc_trigger: false,
            td_solve_floor: -1.0,
            // Analytic deadbeat placement is the shipped law; `0` auto-picks
            // the stance duration as the convergence deadline.
            dcm_tc: 0.0,
            step_height: 28.0,
            floor_y: 0.0,
            bend_sign: 1.0,
            facing: 1.0,
            crouch: false,
            gravity: 980.0,
            jump_apex: 60.0,
            k_p_apex: 0.5,
            launch_accel: 2200.0,
            torso_len: 70.0,
            attitude_kp: 45.0,
            attitude_kd: 11.0,
            attitude_accel_coupling: 1.0,
            attitude_gravity_coupling: true,
            theta_max: 1.30,
            boot_lunge: false,
            boot_lunge_hold: false,
            boot_lunge_v: 160.0,
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
// Boot FSM (Stage 4d, `BOOT_LUNGE`)
// ---------------------------------------------------------------------------

/// Whole-body horizontal gait mode. The per-leg [`GaitPhase`] machine is
/// untouched; this switch only *vetoes* liftoffs (feet locked) while it is not
/// `Walk`, and scripts the hip through a short forward lunge.
///
/// Why a scripted lunge instead of a weight shift (the three failed attempts
/// of Stage 4c): this plant has no ankle actuator, so a *posture* (lean or
/// backward plant-drag) cannot migrate the CoM while both feet are pinned.
/// The leg PD can, however, *drag the hip particle itself* toward an
/// advancing reference: translate the resolver hip forward through a short
/// window and the physics pipeline gives the body real forward velocity. The
/// DCM law (`xi = x + v/ω₀`) is blind at `v = 0`; after the lunge it reads
/// positive `v_com` on its very first tick, so the first swing targets land
/// ahead of the hip and the 2 s catch-up scramble disappears.
///
/// Release is a *timer by intent*: the window IS the maneuver (like a jump's
/// launch phase), so unlike the Stage 4c cap it is not an escape hatch for a
/// weight shift that failed to arrive. `BOOT_LUNGE_HOLD` refuses the release
/// to prove the state switch is real (a locked body thrown forward falls).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BootPhase {
    /// Feet locked (no liftoff, stance timers frozen), hip sitting. Initial
    /// armed state while `boot_lunge` is on; inert (`Walk`) when off.
    Stand,
    /// Scripted forward hip translation in progress: `dir` is the signed lunge
    /// direction captured at entry, `t` is the time held so far (seconds).
    /// Feet stay locked; the hip reference advances at `BOOT_LUNGE_V`.
    Lunge { dir: f32, t: f32 },
    /// The gait door is open: the DCM law, the triggers and the timers own
    /// everything. Sticky — once released, never re-armed (a stopped walker
    /// does not re-stand; the committed stop-settle owns that).
    Walk,
}

impl Default for BootPhase {
    fn default() -> Self {
        BootPhase::Stand
    }
}

impl BootPhase {
    /// True while the feet-lock veto is active (`Stand` or `Lunge`).
    #[inline]
    pub fn is_locked(&self) -> bool {
        !matches!(self, BootPhase::Walk)
    }
}

/// How much authority the physics contact layer has over the gait clock.
///
/// Plan §6 is a spectrum, not a switch, and the levels are **additive**: each
/// one includes everything below it. They exist because the three mechanisms
/// have very different failure modes, and enabling all three at once made the
/// walk measurably worse than the pre-§6 timer (`physics_tuning_progress.md`
/// §11). Having them separable is what makes that A/B possible.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum GaitAuthority {
    /// Pre-§6: the timers own the gait clock outright. Physics contact is
    /// ignored entirely. This is the control arm.
    Timer,
    /// **Observation only.** Read and record contact, decide nothing. Proves
    /// the contact layer is wired without perturbing the walk.
    Observe,
    /// **Plant correction.** The stance plant re-anchors to the measured foot
    /// x, so the resolver draws the foot where it actually is instead of where
    /// it asked. Fixes the stance-slide feedback error; touches no timing.
    Contact,
    /// **Full authority** (`Contact` + the two timing mechanisms). A stance
    /// ends early if its loaded foot has left the ground, and a swing extends
    /// until its foot really lands.
    ///
    /// This is the level that regressed, and the reason is documented in
    /// `physics_tuning_progress.md` §11: the mechanisms are individually
    /// sound but the *timing* ones fight each other through the anti-phase
    /// interlock, which only lets one foot swing at a time.
    Full,
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Ground truth for one leg, fed in by the ECS contact layer (plan §6).
///
/// `None` in [`ResolverState::foot_contact`] means *no physics contact
/// information at all* (the pure kinematic sandbox), and the resolver then
/// keeps its old timer-only behaviour exactly. This is deliberately an input
/// rather than something the resolver measures: `resolver.rs` stays pure math
/// and all collision knowledge lives in `hip_pin.rs`.
///
/// Note what is **not** here: a debounced `grounded` flag. The resolver wants
/// the instantaneous truth — "did the foot collide with the floor on this
/// tick?" — because every §6 decision is a comparison against a phase boundary.
/// The ECS layer keeps its own hysteretic flag for the *thrust* gate, where a
/// stable signal matters more than a one-tick-late one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegContactInput {
    /// The foot collider reported a ground contact on the most recent physics
    /// tick. No hysteresis: see the type docs.
    pub touching: bool,
    /// Realized foot x (resolver space) to re-anchor the stance plant to, or
    /// `None` to leave `plant_pos` alone. Only used under
    /// [`GaitAuthority::Contact`].
    pub foot_x: Option<f32>,
}

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
    /// Seconds spent in Swing *after* `phase_timer` ran out while waiting for
    /// real contact (plan §6, swing extension). Reset on every phase change.
    pub swing_wait: f32,
    /// True when this leg's last Stance was cut short by measured lift-off
    /// (contact lost before the timer expired) rather than by the timer.
    /// Telemetry only; lets the UI/HUD tell a "gated" step from a clocked one.
    pub early_lift: bool,
    /// True when the swing that *entered* the current stance never received a
    /// contact report, i.e. the foot arrived (per the pose) without ever
    /// touching. Contact absence is then uninformative — the foot was already
    /// not touching when the stance began — so [`GaitAuthority::Full`] must not
    /// read it as "the foot just left" and veto the stance on its first tick.
    ///
    /// Without this guard that is exactly what happens: a swing that lands high
    /// produces a stance with no support, `Full` cuts it immediately, the leg
    /// swings again, lands high again — an unbounded rapid-cycle oscillator.
    pub hovered_in: bool,
    /// Set once the stance plant has been re-anchored to the measured foot.
    /// Cleared on lift-off, so the plant is corrected **at most once per
    /// stance**.
    ///
    /// The "once" is the whole point. Re-anchoring every tick while contact
    /// holds is a positive feedback loop: `plant_pos` sets the resolver's foot
    /// target, the leg PD drives the foot there, the next tick measures the
    /// (drifting) foot and moves the plant to it — so the plan is dragged along
    /// by the very slip it was supposed to correct. Measured cost of the
    /// per-tick version: stance drift 61 px → 130 px
    /// (`physics_tuning_progress.md` §11).
    pub plant_anchored: bool,
    /// Monotone swing-profile progress achieved so far (`s` of the current
    /// flight, `0` in stance). Re-timing (§6.2) may stretch the schedule, but
    /// the reference foot never rewinds — this clamp makes that structural.
    pub swing_prog: f32,
    /// Code of the condition that ended the most recent stance for this leg
    /// (`plans/swing-phase-timing.md` §9 "log every transition with its
    /// triggering condition"): `0` = none yet, `1` = timer expiry (the nominal
    /// gait clock), `2` = the §3 DCM-margin liftoff trigger (`dcm_crossed`),
    /// `3` = the §3 emergency trigger (measured CoM past the plant,
    /// `com_crossed`). Telemetry only; decides nothing.
    pub trigger_reason: u8,
    /// Full scheduled swing duration (seconds) latched at lift-off and stretched
    /// by continuous re-timing during swing — the denominator that converts
    /// `phase_timer` into profile progress. This is the schedule's time base,
    /// never the landing trigger itself.
    pub flight_span: f32,
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

/// Loop-3 (attitude) persistent state. See `plans/raibert_sandbox_attitude_loop.md`.
///
/// The torso is a balancing pole about the hip: `θ = 0` is upright, positive θ is
/// leaning forward (y-up travel frame). `omega` is the angular velocity. The PD in
/// [`tick_attitude`] holds `θ` at `0` against the destabilizing gravity term and
/// the inertial coupling of hip horizontal acceleration.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeState {
    /// Torso lean relative to world vertical, rad. `0` = upright; `+` = forward.
    pub theta: f32,
    /// Angular velocity of the torso, rad/s.
    pub omega: f32,
}

impl Default for AttitudeState {
    fn default() -> Self {
        Self {
            theta: 0.0,
            omega: 0.0,
        }
    }
}

/// The resolver's full persistent state.
#[derive(Debug, Clone)]
pub struct ResolverState {
    pub hip_pos: Vec2,
    pub hip_vel: Vec2,
    /// Rate-limited velocity *reference* (`cmd_vx → v_des`, §19.2.2/§15.1). The
    /// command presented to the capture point and backward solve ramps instead
    /// of stepping, so a soft body starting at rest is never asked to jump to
    /// full speed in one tick.
    pub v_ref: f32,
    /// Current (possibly interpolated) hip height `H`.
    pub hip_height: f32,
    pub left: LegState,
    pub right: LegState,
    /// Loop-1 (vertical) controller state.
    pub vs: VerticalState,
    /// Loop-3 (attitude) controller state: torso lean + angular velocity.
    pub attitude: AttitudeState,
    /// Previous tick's horizontal hip velocity, for finite-differencing the
    /// hip acceleration `a_x` that feeds loop 3's inertial-coupling term.
    /// `None` on the first tick (and when `dt ≤ 0`).
    pub prev_hip_vel_x: Option<f32>,
    /// True on the most recent tick if the body was airborne (`Flight`). Used to
    /// detect the `Launch→Flight` (arm) and `Flight→Land` (seed) edges of the
    /// jump's airborne swing.
    pub airborne_prev: bool,
    /// `true` when the jump was armed from a *walking* lift-off. The airborne
    /// swing then reaches to the ground-reference forward target (a running
    /// stride, one step ahead); on a stand-still jump it is `false` and the
    /// swing lands on top of the lift-off footprint instead (no forward step).
    pub jump_arm_lead_leg: bool,
    /// Previous tick's resolved joint positions, aligned to
    /// `[hip, left_knee, left_foot, right_knee, right_foot]`. `None` on the
    /// first tick. Used to finite-difference the pose into velocities for the
    /// leg PD controller's velocity-error term.
    pub prev_positions: Option<[Vec2; 5]>,
    /// Measured whole-body capture state (loop 1 balance, plan §2). Fed in by
    /// the ECS layer from [`measure_body`]; `None` in the pure kinematic
    /// sandbox, where `plan_target_x` falls back to the Raibert neutral point.
    pub measured: Option<CaptureMeasure>,
    /// Measured whole-body CoM apex **rise** above the standing height, px
    /// (loop 2, plan §4). Fed in by the ECS layer from the §1 measurement each
    /// tick. When present, `Flight` latches `h_apex_prev` from this physics
    /// measurement instead of the resolver's internal ballistic model, so the
    /// `Δl` integrator closes on the real ground reaction. `None` in the pure
    /// kinematic sandbox (internal model only).
    pub measured_apex: Option<f32>,

    // --- Plan §6: contact-gated gait clock authority ---
    /// Per-leg ground truth from the ECS contact layer, `[left, right]`.
    /// `None` = no physics contact information at all (pure kinematic
    /// sandbox), and the gait clock then behaves exactly like the pre-§6
    /// timer. This is an *input* so `resolver.rs` stays pure math; all
    /// collision knowledge lives in `hip_pin.rs`.
    pub foot_contact: Option<[LegContactInput; 2]>,
    /// How much authority the contact layer above is granted. Defaults to
    /// [`GaitAuthority::Timer`] so the pre-§6 walk is the default and every
    /// step up is an explicit, measurable decision.
    pub gait_authority: GaitAuthority,
    /// Maximum seconds a swing may overrun its timer while waiting for real
    /// contact before landing anyway (§6 swing extension). Without a cap, a
    /// foot that never touches would swing forever.
    pub max_swing_wait: f32,
    /// Diagnostic: per-leg count of stances that `GaitAuthority::Full` ended
    /// before their timer expired, and swings that overran waiting for
    /// contact. Exposed because these are the two mechanisms that changed the
    /// gait's rhythm, and the counters make that visible without telemetry.
    pub early_lift_count: [u32; 2],
    pub swing_wait_count: [u32; 2],
    /// Boot FSM (Stage 4d): the stand→lunge→walk switch. `Walk` when
    /// `config.boot_lunge` is off (the committed gait, bit-identical).
    pub boot: BootPhase,
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
            swing_wait: 0.0,
            early_lift: false,
            hovered_in: false,
            plant_anchored: false,
            swing_prog: 0.0,
            trigger_reason: 0,
            flight_span: config.stance_duration_ts,
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
            swing_wait: 0.0,
            early_lift: false,
            hovered_in: false,
            plant_anchored: false,
            swing_prog: 0.0,
            trigger_reason: 0,
            flight_span: config.stance_duration_ts,
        };

        Self {
            hip_pos: hip,
            hip_vel: Vec2::ZERO,
            // Reference starts at rest and is ramped toward `v_des` each tick
            // (see `step_lower_body` §2). Starting it at 0 keeps the bootstrap
            // from commanding a step-change in velocity (§15.1/§19.2.2).
            v_ref: 0.0,
            hip_height: config.hip_height,
            left,
            right,
            vs: VerticalState::default(),
            attitude: AttitudeState::default(),
            prev_hip_vel_x: None,
            airborne_prev: false,
            jump_arm_lead_leg: false,
            prev_positions: None,
            measured: None,
            measured_apex: None,
            foot_contact: None,
            // The pre-§6 timer is the default. Each §6 mechanism is opt-in.
            gait_authority: GaitAuthority::Timer,
            // ~4 ticks at 90 Hz. Long enough to ride out a single missed
            // collision frame, short enough that a genuinely stuck foot lands
            // instead of hanging the gait.
            max_swing_wait: 0.05,
            early_lift_count: [0; 2],
            swing_wait_count: [0; 2],
            // Off ⇒ inert `Walk`; on ⇒ armed `Stand` (feet locked until the
            // lunge releases).
            boot: if config.boot_lunge {
                BootPhase::Stand
            } else {
                BootPhase::Walk
            },
        }
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

/// The resolved particle positions and velocities: 1 hip + 2 knees + 2 feet.
#[derive(Debug, Clone, Copy)]
pub struct LowerBodyPose {
    pub hip: Vec2,
    pub left_knee: Vec2,
    pub left_foot: Vec2,
    pub right_knee: Vec2,
    pub right_foot: Vec2,
    /// Loop-3 upper-body point: `torso_len` above the hip along the tilt axis
    /// (`hip + L·(sinθ, cosθ)` in the y-up travel frame). This is the visual
    /// torso tip / upper-body marker the frame tilt rotates about the hip.
    pub upper: Vec2,
    /// Finite-differenced joint velocities (per-second), aligned with the
    /// position fields. The hip uses the exact integrated `hip_vel`; knees/feet
    /// are `(current − previous)/dt`.
    pub hip_vel: Vec2,
    pub left_knee_vel: Vec2,
    pub left_foot_vel: Vec2,
    pub right_knee_vel: Vec2,
    pub right_foot_vel: Vec2,
}

/// Virtual-leg thrust command (plan §4, loop 2 delivery).
///
/// The banked stroke `Δl` is spent as knee/ankle **extension** during stance
/// (the foot is pinned by friction, so extension pushes the CoM up via ground
/// reaction), as **retraction** during flight (shorten the virtual leg so the
/// swing clears), and as controlled **flexion** on landing (absorb the
/// touchdown). All values are px of extra leg-extension demand; the ECS layer
/// converts them into angular-constraint targets through the leg PD.
#[derive(Debug, Clone, Copy, Default)]
pub struct LegThrust {
    /// Stance push-off extension demand, px. `> 0` during `Load`/`Launch`
    /// (the crouch stroke being spent), `0` at idle.
    pub stance_extend: f32,
    /// Flight retract demand, px (`> 0` tucks the knee for swing clearance).
    pub flight_retract: f32,
    /// Landing absorb flexion demand, px (`> 0` bends the knee to absorb).
    pub land_absorb: f32,
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
    /// Per-leg code of the condition that ended the most recent stance:
    /// `0` = none yet, `1` = timer, `2` = `dcm_crossed` (the §3 primary
    /// trigger), `3` = `com_crossed` (the §3 emergency). Telemetry only.
    pub left_trigger_reason: u8,
    pub right_trigger_reason: u8,
    /// `true` when the unclamped §2 capture point fell outside the reachable
    /// `hip ± x_max` window this tick — the leg-geometry clamp fired, i.e. the
    /// "in trouble" signal. `false` when there is no measurement (pure
    /// kinematic fallback) or the capture point is within reach.
    pub capture_saturated: bool,

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
    /// Virtual-leg thrust command (plan §4, loop 2 delivery): how the banked
    /// stroke `Δl` is spent as knee/ankle extension each tick. Read by the ECS
    /// layer to bias the leg-PD angular targets — stance pushes off (extend),
    /// flight retracts for clearance, landing absorbs.
    pub thrust: LegThrust,

    // --- Loop 3 (attitude) telemetry. See `plans/raibert_sandbox_attitude_loop.md`. ---
    /// Torso lean relative to world vertical, rad (`+` = forward in travel frame).
    pub theta: f32,
    /// Angular velocity of the torso, rad/s.
    pub omega: f32,
    /// Loop-3 control input `u` (an angular acceleration), rad/s².
    pub attitude_torque: f32,
    /// Measured hip horizontal acceleration (finite-differenced), px/s².
    pub hip_accel_x: f32,
}
