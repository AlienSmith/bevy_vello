//! The **control** module — fiction + command → [`ControlOut`] (plan §1).
//!
//! This module is the *only* place the walking algorithm lives. It is split
//! into:
//!
//! - **Contracts** ([`Command`], [`ControlOut`], [`LocoMode`]) — data-only,
//!   no ECS. The pose itself is [`crate::actuate::Pose`]; control builds it
//!   and wraps it in [`ControlOut`] with the non-actuation fields (`hip_vel`,
//!   `mode`, `debug`).
//! - **[`LocoState`]** — the state machine whose *only* job is deciding which
//!   controller is active. It reads command + fiction and nothing else.
//! - **[`Controller`]** — an enum of three controllers sharing one slot:
//!   [`StandingCtrl`] (rest-pose hold), [`WalkingCtrl`] (the `walker_math`
//!   gait seam), [`BoostStartCtrl`] (the leaned-stance launch, plan §5).
//!
//! # Law 2 — controller state is schedule + reference only
//! Every *body* field the controller needs arrives in [`Fiction`] (measured by
//! observe). The controller may integrate only what physics cannot measure:
//! the `v_ref` ramp, foot-placement plans, FSM timers. There is no
//! `hip_pos += v·dt` here, by construction — there is nowhere to write it.
//!
//! # Frame
//! Vello **y-down**, same as [`crate::observe`] and [`crate::actuate`]. The
//! `walker_math` seam (y-up) flips at the stamp; nothing else does.
//!
//! # The standing controller is deliberately a dummy
//! The default pose *is* standing, so standing commands essentially "keep
//! doing what you are doing, only balanced": hold the measured feet, hold the
//! measured hip height, drift the hip horizontally toward the foot midpoint at
//! a capped rate, and command an upright attitude. Emitted constraint targets
//! then land inside [`crate::actuate`]'s dead zone whenever the character
//! already stands balanced — zero actuation ticks, which is the correct
//! behaviour for a body at rest.

use glam::Vec2;
use walker_math::{DebugInfo, GaitAuthority, LegContactInput, LowerBodyConfig, ResolverState};

use crate::actuate::{FootPose, Pose};
use crate::observe::Fiction;

// ---------------------------------------------------------------------------
// Contracts
// ---------------------------------------------------------------------------

/// Player/script intent, written by the input system. Units are px/s and
/// Vello y-down. A `Command` is a **held move** (a script cell's duration);
/// "no command" is `None` in [`crate::walker::Walker::command`], which is what
/// ends a run and returns the machine to standing.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    /// Commanded horizontal direction, unit-ish (`+x` = travel right).
    pub dir: Vec2,
    /// Commanded speed magnitude, px/s. `0` = stop / stay put.
    pub speed: f32,
    /// This move is a **launch**: start it through [`LocoMode::BoostStart`]
    /// instead of entering the gait directly. The launch is an **upright**
    /// held stance — squash, then push the back foot backwards — held while
    /// this flag is set; the machine hands over to [`LocoMode::Walking`] when
    /// the launch schedule has played out *and* the DCM has passed the front
    /// foot (see [`boost_handover_due`]).
    pub boost: bool,
}

impl Command {
    /// Convenience: a pure horizontal move command (no launch boost).
    pub fn walk(speed: f32) -> Self {
        let dir = Vec2::new(speed.signum(), 0.0);
        Self {
            dir,
            speed,
            boost: false,
        }
    }

    /// Convenience: a boosted launch move — the kick start of the gait.
    pub fn boost(speed: f32) -> Self {
        let dir = Vec2::new(speed.signum(), 0.0);
        Self {
            dir,
            speed,
            boost: true,
        }
    }

    /// Signed horizontal command speed (`dir.x · speed`).
    pub fn vx(&self) -> f32 {
        self.dir.x * self.speed
    }

    /// True when this command actually asks for locomotion: a nonzero signed
    /// speed above `threshold`. A command below it counts as *ended* — the
    /// machine treats it exactly like no command (return to standing).
    pub fn is_move(&self, threshold: f32) -> bool {
        self.vx().abs() > threshold
    }
}

/// The coarse locomotion mode. Deliberately **three** entries: one per
/// controller. Finer phases (`GaitPhase`, `VerticalPhase`) belong to the
/// controller that owns them; they are not modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocoMode {
    /// Balanced quiet stance.
    Standing,
    /// The gait schedule (Raibert / deadbeat DCM).
    Walking,
    /// The swappable launch used to start moving (`plans/walker-control-rewrite.md` §5).
    BoostStart,
}

impl Default for LocoMode {
    /// A fresh character stands. This also lets [`crate::telemetry::TickRecord`]
    /// derive `Default` without a hand-written initializer.
    fn default() -> Self {
        LocoMode::Standing
    }
}

/// One control tick's output.
///
/// The **pose is a field, not a copy**. Control produces [`Pose`] — actuate's
/// own contract type, with no second struct mirroring it — and wraps it with
/// the three values actuate must never see because they are not actuation:
///
/// | field | consumer |
/// |---|---|
/// | [`pose`](Self::pose) | [`crate::actuate::actuate_tick`] (the only input it reads) |
/// | `hip_vel` | telemetry (`cmd_vx`, `tP3vx`) and prediction bookkeeping |
/// | `mode` | diagnostics; [`LocoState::mode`] is the authority |
/// | `debug` | CSV passthrough (plan §3b); `None` outside the gait controller |
///
/// Keeping `Pose` as the actuate layer's own type is deliberate: the bottom
/// layer stays self-contained and knows nothing about commands, modes, or
/// telemetry, while control cannot silently feed it extra fields because the
/// only path is `out.pose`.
///
/// Note the pose has **no torso-tip field**. The old architecture aimed a
/// *position* there (the spine's external position constraint); this one
/// commands the stick's **orientation** as an angle through the leg hip
/// joints (`P40-P3-P2` / `P41-P3-P2`), so a torso point would be write-only.
/// Loop-3's pendulum model still needs a torso length, but that lives in
/// [`walker_math::LowerBodyConfig::torso_len`] — a balance-law parameter, not
/// an actuation target.
#[derive(Debug, Clone, Copy)]
pub struct ControlOut {
    /// Exactly what actuate consumes this tick.
    pub pose: Pose,
    /// Root velocity target (telemetry + prediction).
    pub hip_vel: Vec2,
    /// The **placement law's own swing target** per foot, as a Vello world
    /// point (same frame as [`Pose::left_foot`]); `NaN` outside `WalkingCtrl`.
    ///
    /// `pose.*_foot.pos` is the *profile* position the actuator is commanded to
    /// (the Hermite output, IK-clamped); this is the plant the law asked for
    /// (`DebugInfo::left/right_target_x`, planted at the config floor) before
    /// the profile and the pose map touched it. Exporting both separates "the
    /// law aimed wrong" from "the leg did not reach the aim", and lets the QA
    /// plot draw the aim on BOTH axes (x target + the plant's floor height).
    pub plan: [Vec2; 2],
    /// Which controller produced this.
    pub mode: LocoMode,
    /// Walking-internals telemetry; `None` outside the gait controller.
    pub debug: Option<DebugInfo>,
}

/// Tuning for the control layer. Everything here is a *decision* knob;
/// solver hardware (compliance/ζ) lives in [`crate::actuate::ActuatorConfig`],
/// and rig geometry (bone lengths, torso length, hip height, floor) lives in
/// [`walker_math::LowerBodyConfig`] — seeded once from the rig at spawn, never
/// guessed here.
#[derive(Debug, Clone, Copy)]
pub struct ControlConfig {
    /// Max horizontal hip drift toward the foot midpoint while standing, px/s.
    pub stand_settle_rate: f32,
    /// Speed magnitude (px/s) below which a move command means "stop".
    pub move_threshold: f32,
    /// Boost: seconds to slide the FRONT foot (the one ahead of the hip in
    /// travel) to directly under the hip, then hold. The gather only needs to
    /// outlast the handover event (see [`boost_handover_due`]). The back foot
    /// stays planted: support is always BEHIND ξ while walking (see
    /// [`BoostStartCtrl`]).
    pub boost_gather_ts: f32,
    /// Boost→Walking handover lead, seconds: the DCM approximation
    /// `ξ ≈ com + v_com·τ` (τ ≈ 1/ω₀ at standing height). The launch releases
    /// when this leads past the frontmost foot — [`boost_handover_due`].
    pub boost_lead_tau: f32,
    /// Boost crouch time, seconds: the plant depths ease from the blueprint
    /// rest depths to `LowerBodyConfig::hip_height` over this window instead
    /// of stepping there on the first boost tick. A step on a planted chord is
    /// a velocity kick — a 5 px squash (`HIP_HEIGHT=98`) drove the hip
    /// 7→31 px/s DOWN and −11→−61 px/s BACKWARD in a single tick, walked the
    /// hip 9 px behind its start, and because ξ = com + v·τ then never passed
    /// the frontmost foot the launch never released: the boost ran its whole
    /// cell and the gait handed over with the CoM behind both plants.
    pub boost_crouch_ts: f32,
    /// Boost push-off, px: extend the BACK leg's chord (the one pointing
    /// against travel) this far along its own hip→foot direction, ramped in
    /// from half a gather window. The pushing foot is flagged `stance` for the
    /// duration — a swing foot rides the idle 1e-2 tether and has no authority
    /// to push with. Clamped to the reach the leg actually has left
    /// (`thigh + calf − chord`): the v10 back foot already sits at 111 px of a
    /// 110 px leg, so a push only has room AFTER a crouch — crouch first, then
    /// extend, exactly like a real launch. Default **0** (off).
    pub boost_push: f32,
    /// Boost push-off ramp, seconds (smoothstep; a step is a velocity kick).
    pub boost_push_ts: f32,
    /// Boost **step two**: move **both** plant targets this far BACKWARDS along
    /// travel, so both legs extend behind the body and the reaction pushes the
    /// hips FORWARD. 100 % of this displacement is forward travel, unlike the
    /// ~30 % a chord extension gives (the chord is ~18° off vertical).
    ///
    /// The launch is explicitly two steps, in this order:
    /// 1. **squash** — `rest_shift` eases both plant depths toward
    ///    `hip_height` over `boost_crouch_ts` (vertical only, nothing moves in
    ///    x), with both feet support so the HIP is pulled down;
    /// 2. **move both feet backwards** — this knob, starting only once step 1
    ///    has landed, because the legs need the crouch's reach room before they
    ///    can translate: each foot is clamped by its own
    ///    `sqrt(reach² − y²) − |x|` (~50 px for the gathered front foot, ~15 px
    ///    for the back one at `hip_height = 96`, none at the v10 rest depth).
    ///
    /// Release is unchanged: the DCM `ξ = com + v_com·τ` passing the frontmost
    /// foot ([`boost_handover_due`]). Default 15 px, **back foot only** — see
    /// [`Self::boost_kick_front`].
    pub boost_kick: f32,
    /// Boost step two, the **front** foot's share of the backward translation,
    /// px. Default **0**: the gathered front foot is the support the body is
    /// standing on, so dragging it backwards removes that support and tips the
    /// body back (measured: the two-feet version fell backwards). Keep this at 0
    /// and let the back leg do the pushing.
    pub boost_kick_front: f32,
    /// Boost kick-off ramp, seconds. Short = a kick, long = a shove: at 0.04 s
    /// a 15 px drive is ~375 px/s of foot speed, fast enough to beat the slip
    /// that turned the first (0.12 s, 8 px) attempt into a drag.
    pub boost_kick_ts: f32,
    /// Boost **hip thrust**, px/s² (`BOOST_HIP_ACCEL`). While the launch is
    /// pushing (`since ≥ boost_crouch_ts`), `actuate_stage` adds one tick of
    /// horizontal `Δv = dir · accel · dt` to the **hip particle `P3`**, as an
    /// idempotent velocity overwrite (last write wins, so a lost or duplicated
    /// application can never accumulate energy).
    ///
    /// This is the one place the launch puts force where it belongs. Every
    /// pose-level push can only move FEET, and a foot that slides delivers
    /// almost nothing (measured: 6 px of foot drag → 0.6 px of hip travel). The
    /// speed the gait inherits is the speed of the BODY, so it is set directly.
    /// At 300 px/s² over the 0.29 s launch that is ~87 px/s of forward speed —
    /// the cruise the plan asks for. 0 disables it.
    pub boost_hip_accel: f32,
    /// Attitude-loop closure: position gain on the MEASURED lean error
    /// (radians → radians). `0.0` keeps `upper_angle` a pure echo of the
    /// solver's model lean — the open loop whose damper (`−kd·ω`) sees fiction
    /// ω≈0 while the gait kicks the real torso, so the wobble grows until the
    /// body falls. Nonzero routes the command through
    /// [`walker_math::attitude_lean_pd`]: model lean = bias, correction PD on
    /// the measured lean/rate. This keeps the model's integral memory;
    /// hard-stamping θ/ω from reality instead is a gain-1 echo (zero restoring
    /// force) and collapsed FASTER (sync120: clamp by t=2.5, down by t=3).
    /// Gains are angle→angle, so they are ~450x smaller than the model's 45/11
    /// acceleration-domain pair. Sweep on walk_in_120 (10s track, same rig):
    /// (0,0) fell at ~9.5s with v_filt runaway 163→209; (0.4,0.2) fell at 5.5s
    /// — hard position feedback destabilizes; (0,0.15) damped wobble but still
    /// ran away to 202 and fell at 11.1s; (0.2,0.3) fell at 4.9s. The baked
    /// (0.1,0.15): v_filt steady ~150 through t=3–11, lean sd 0.02 (10x the
    /// baseline), survived the whole walk cell.
    pub att_closure_kp: f32,
    /// Attitude-loop closure: rate gain on the MEASURED lean rate (rad/s →
    /// radians). Damps the real wobble the fiction damper cannot see.
    pub att_closure_kd: f32,
    /// Stance-foot press, px (`STANCE_DEPTH`). Default **0** (off ⇒ the pose is
    /// bit-identical to the solver output). When nonzero, `WalkingCtrl::step`
    /// extends the STANCE leg's commanded foot a few px FURTHER along the
    /// hip→foot chord as the very last act of the controller, before the
    /// actuator reads the pose. Pressing along the leg axis (not straight down)
    /// loads the foot into the floor the way the strut actually pushes, and
    /// leaves the planner's geometry (`floor_y`, plants, swing targets) and the
    /// vertical loop completely untouched — it is a pure actuation preload.
    ///
    /// Shaped by a stance-phase envelope: mild at touchdown and liftoff,
    /// strongest mid-stance (`sin(π·age/T_stance)`), so it never yanks at the
    /// stance boundaries where the contact is being made/broken.
    pub stance_press_depth: f32,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            stand_settle_rate: 40.0,
            move_threshold: 5.0,
            // Deliberately BELOW the gait's own cruise lean (~0.15 rad): the
            // gathered feet already put ξ ahead of both plants at handover, so
            // the lean only needs to nudge, not launch. A hard tilt (0.25)
            // overshoots 120 px/s, sags z_com, and ω₀ = √(g/z_com) inflates
            // x_max — the runaway that collapsed the hip ~100 px. Measured
            // lean 0.05: symmetric steps, bounded sag, avg ~94 px/s.
            // Front foot sweeps 39.5 px inward over this window; the handover
            // event fires partway through it (~0.3 s), so the gather only has
            // to be fast enough to MEET ξ, never overshoot past it.
            boost_gather_ts: 0.4,
            // 1/ω₀ at hip_height ≈ 104 (ω₀ = √(980/104) ≈ 3.07): the DCM's
            // velocity lead over the CoM.
            boost_lead_tau: 0.33,
            // Long enough to be a ramp at 90 Hz (≈ 23 ticks), short enough
            // that the crouch is in place before the handover event.
            boost_crouch_ts: 0.25,
            // Off by default: the launch stays a pure leaned stance until a
            // sweep says the push helps.
            boost_push: 0.0,
            boost_push_ts: 0.3,
            // On by default, but the reach clamp makes it a no-op unless the
            // launch is crouched (see the field docs).
            boost_kick: 15.0,
            boost_kick_front: 0.0,
            boost_kick_ts: 0.04,
            // ~90 px/s of forward speed over the 0.29 s launch.
            boost_hip_accel: 300.0,
            // Validated on the 10s track (see the field docs): full-walk
            // survival + 10x wobble damping vs the open loop.
            att_closure_kp: 0.1,
            att_closure_kd: 0.15,
            // Off by default: the walking pose stays bit-identical to the
            // solver output until a sweep says otherwise.
            stance_press_depth: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/// A pose that commands exactly what physics currently has: measured hip,
/// measured feet, measured attitude. Emitted targets collide with the live
/// pose inside actuate's dead zone, so a hold is genuinely inert.
///
/// Used by the two unimplemented controllers so the wiring, telemetry, and
/// mode machine can be exercised end-to-end before their algorithms land.
#[inline]
pub fn hold_pose(
    fiction: &Fiction,
    _cfg: &ControlConfig,
    mode: LocoMode,
    facing: f32,
) -> ControlOut {
    ControlOut {
        pose: Pose {
            hip: fiction.hip,
            left_foot: FootPose {
                pos: fiction.feet[0].pos,
                stance: fiction.feet[0].touching,
            },
            right_foot: FootPose {
                pos: fiction.feet[1].pos,
                stance: fiction.feet[1].touching,
            },
            facing,
            upper_angle: fiction.lean,
            stance_split: false,
        },
        hip_vel: fiction.v_hip,
        plan: [Vec2::new(f32::NAN, f32::NAN); 2],
        mode,
        debug: None,
    }
}

// ---------------------------------------------------------------------------
// Controllers
// ---------------------------------------------------------------------------

/// The controller contract. One call per fixed tick.
///
/// `facing` is the character's travel orientation (`+1` = `+x`), a spawn-time
/// constant rather than command state — but it is one of the five pose elements
/// control produces, so it is passed here instead of being patched in later.
pub trait Step {
    fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut;
    fn mode(&self) -> LocoMode;
}

/// Balanced quiet stance: the **v10 rest pose**, hardcoded.
///
/// # The pose is translation-invariant, so only offsets matter
/// The actuate layer never reads a world position: it consumes only the
/// hip→foot chord (IK knee), the hip→knee→foot angle (knee joint), and the
/// `P3→P2` axis rotated by `upper_angle` (hip joint). The hip particle is free
/// (no position constraint). So `pose.hip` can be anything — moving it in world
/// space changes *nothing*. What actuates the legs is the **shape**: the two
/// hip→foot offset vectors and the lean.
///
/// That is why this controller feeds **no physics feedback**: every value it
/// emits is a constant offset from the blueprint's rest pose. Reading the
/// measured hip or feet here would close a loop and let a sag walk away.
///
/// # The constants
/// Taken from `v10.character.json` (SVG units, then × 0.5 spawn scale), all
/// measured from `P3` (the hip):
///
/// | particle | SVG offset from `P3` | scale-0.5 px |
/// |---|---|---|
/// | `P2` (waist ref) | `( 0.0, −70.0)` | `( 0.0, −35.0)` |
/// | `P40` (L knee) | `(−33.0, 106.0)` | `(−16.5, 53.0)` |
/// | `P41` (R knee) | `( 39.0, 103.0)` | `( 19.5, 51.5)` |
/// | `PLLL` (L foot) | `(−66.0, 212.0)` | `(−33.0, 106.0)` |
/// | `PRLL` (R foot) | `( 79.0, 206.0)` | `( 39.5, 103.0)` |
///
/// Note the rest leg is **fully extended** (hip→foot ≈ 111 px ≈
/// `thigh + calf`), not a bent-knee drop pose: that is what the rig stands on
/// with no actuation at all. Commanding any shorter chord asks the knee to fold
/// against two rigid distance bones, which is what collapsed the stance.
#[derive(Debug, Clone, Copy)]
pub struct StandingCtrl {
    /// Seconds spent standing (telemetry; also lets later hysteresis gates read
    /// a settle time).
    pub since: f32,
    /// Hardcoded `P3 → PLLL` offset, px, vello y-down. Left foot.
    left_foot_offset: Vec2,
    /// Hardcoded `P3 → PRLL` offset, px, vello y-down. Right foot.
    right_foot_offset: Vec2,
}

/// v10 rest offsets under `P3`, px at spawn scale 0.5 (y-down: +y is below).
const REST_LEFT_FOOT: Vec2 = Vec2::new(-33.0, 106.0);
const REST_RIGHT_FOOT: Vec2 = Vec2::new(39.5, 103.0);

impl StandingCtrl {
    pub fn new(_lower: &walker_math::LowerBodyConfig) -> Self {
        Self {
            since: 0.0,
            left_foot_offset: REST_LEFT_FOOT,
            right_foot_offset: REST_RIGHT_FOOT,
        }
    }
}

impl Step for StandingCtrl {
    fn step(
        &mut self,
        _cmd: &Command,
        fiction: &Fiction,
        _cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        if dt > 0.0 {
            self.since += dt;
        }
        // Anchor the frame at the measured hip so the *world* pose is right, but
        // emit only hardcoded offsets: the shape is constant, so the emitted
        // joint rests coincide with the v10 rest pose and land in actuate's dead
        // zone. The body then stands on its own rigid bones, exactly as it does
        // with no actuation at all.
        let hip = fiction.hip;
        let left_foot = hip + self.left_foot_offset;
        let right_foot = hip + self.right_foot_offset;

        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: left_foot,
                    stance: fiction.feet[0].touching,
                },
                right_foot: FootPose {
                    pos: right_foot,
                    stance: fiction.feet[1].touching,
                },
                facing,
                // The rest pose's `P3→P2` axis is straight up (offset
                // `(0, −35)`), so the attitude command is upright.
                upper_angle: 0.0,
                stance_split: false,
            },
            hip_vel: Vec2::ZERO,
            plan: [Vec2::new(f32::NAN, f32::NAN); 2],
            mode: LocoMode::Standing,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Standing
    }
}

/// The gait controller: [`walker_math::step_lower_body`] behind the seam.
///
/// # The seed frame (why an anchor, not absolute coords)
/// `walker_math` runs in its own **y-up** fiction space: floor at `y = 0`, hip
/// seeded at `(0, hip_height)`. Vello is y-down and the absolute world origin
/// is meaningless to the legs (the pose is translation-invariant — only the
/// hip→foot chord and lean actuate). So this controller keeps a one-time
/// *anchor* pair recorded on its first tick: the measured vello hip and the
/// solver's seed hip. Every tick:
///
/// - **stamp (Law 2, position-only)**: the solver's fiction `hip_pos.x` is
///   re-anchored to the measured horizontal displacement, exactly the old
///   `FEEDBACK=1` path. `hip_vel` is deliberately NOT overwritten — the
///   measured-velocity hard feedback is documented (old drive.rs) to stop the
///   body dead by cancelling the command-vs-realized lead; the solver owns
///   that fiction integrator.
/// - **measured CoM**: `Fiction.com/v_com` enter `CaptureMeasure` through the
///   same seed-relative x-map, with `z_com` = height above the anchor floor
///   line (`anchor.y + hip_height` in vello y-down). With no measurement the
///   solver falls back to its Raibert neutral point; we always have one.
/// - **schedule state stays in the solver**: the vertical spring
///   (`tick_vertical` owns `hip_pos.y`), the gait FSM timers, `v_ref`, and
///   loop-3's pendulum are fiction the physics cannot replace. Attitude is
///   *seeded once* from the fiction lean so the handover from BoostStart is
///   continuous, then solver-owned.
/// - **event-driven gait** (`GaitAuthority::Full`, no stance stopwatch):
///   `reality_v` stays `None` (open-loop vertical fiction), but `foot_contact`
///   is stamped every tick from the RAW one-tick `Fiction.feet[i].touching`
///   flags — no hysteresis, exactly what [`LegContactInput`] asks for — with
///   `foot_x` riding the same seed-relative map as the plants. The solver
///   lifts off only when the DCM crosses the stance toe (never because a
///   stopwatch expired), and ends a swing only on measured touchdown (bounded
///   by `max_swing_wait`, whose overrun plants with `hovered_in`). The Timer
///   build's measured failure — the clock claiming stance while both feet
///   floated (`plans/vertical-stabilizer.md` §0) — is gone by construction:
///   a leg cannot leave the ground before its partner is contact-confirmed.
///
/// The output pose is mapped back by hip-relative **offsets**: solver
/// `(foot − hip)` with `y` negated, re-anchored at the measured hip — the same
/// trick [`StandingCtrl`] uses, so a solver drift in absolute position can
/// never detach a leg. All solver→vello vector mappings mirror x by `facing`
/// (the solver's +x is the TRAVEL direction) and negate y (y-up → y-down).
#[derive(Debug, Clone)]
pub struct WalkingCtrl {
    pub state: ResolverState,
    /// Working copy of the rig/gait config: `target_speed` is written from the
    /// held command each tick (the old `MoveSpeed` resource pattern, now a
    /// field of the controller that owns the gait).
    cfg: LowerBodyConfig,
    /// Measured vello hip at seed time (first tick). `None` until then.
    anchor: Option<Vec2>,
    /// Solver seed hip at the same moment (`ResolverState::new`'s position).
    seed_hip: Vec2,
}

impl WalkingCtrl {
    pub fn new(cfg: &LowerBodyConfig) -> Self {
        let mut state = ResolverState::new(cfg);
        // Close the VELOCITY loop: the deadbeat's anchor ξ already uses the
        // measured CoM speed, but its brake-lead term `v_law` defaults to the
        // fiction integrator (`approach(v_prev, v_des)`) that never leaves the
        // command setpoint — measured: fiction pinned at 120 while reality ran
        // to 181, so the brake saw "no correction needed." `plan_meas_v` routes
        // the filtered measured `v_com` into `v_law` (and the stop path), so an
        // overspeeding body is braked against its TRUE speed.
        let mut cfg = cfg.clone();
        cfg.plan_meas_v = true;
        // Event-driven gait: liftoff on the DCM crossing, landing on measured
        // touchdown (`step_leg`'s `Full` branch). `step` stamps
        // `state.foot_contact` every tick; without a stamp (`None`) the solver
        // behaves like `Timer`.
        state.gait_authority = GaitAuthority::Full;
        // The default (0.05) only rides out report latency. Measured: a swing
        // foot needs up to ~0.2 s of leg dynamics to close the last px to the
        // floor (base.csv t=2.3-2.5), so give the wait a nominal-swing budget.
        // A genuinely stuck foot still lands after the cap (hovered_in), and
        // under the event-only liftoff a wait overrun is SAFE: the stance leg
        // simply does not lift until the partner is down or its own DCM
        // crosses — the both-feet-airborne window cannot open.
        state.max_swing_wait = cfg.swing_duration_ts;
        let seed_hip = state.hip_pos;
        Self {
            state,
            cfg,
            anchor: None,
            seed_hip,
        }
    }
}

impl Step for WalkingCtrl {
    fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        let anchor = match self.anchor {
            Some(a) => a,
            None => {
                // First tick: anchor the seed frame at the measured hip and
                // carry the REAL state across the handover — the BoostStart
                // exit fires because the body measurably moves, so the gait
                // fiction must start at that speed, not at zero (a fiction
                // ramping from 0 would plan its first steps blind to the
                // seeded velocity; the deadbeat lead `v_law` is fiction-owned).
                self.anchor = Some(fiction.hip);
                self.state.hip_vel.x = fiction.v_hip.x * facing;
                self.state.attitude.theta = fiction.lean * facing;
                self.state.attitude.omega = fiction.lean_rate * facing;
                // Seed the stance plants from the MEASURED feet. Without this
                // the solver's default `seed_hip ± 30` plants disagree with
                // reality (the boost leans the body ~0.25 rad without moving
                // its feet; any handover where the measured offsets differ
                // from ±30 is worse): the first gait tick yanks the planted
                // feet forward, the legs stop dead and the body trips over
                // them (measured: lean jumped 0.48 -> 1.06 rad in one tick).
                // With the plants seeded, tick 1 commands the feet where they
                // actually are and the capture-point law owns the catch.
                for (leg, foot) in [
                    (&mut self.state.left, &fiction.feet[0]),
                    (&mut self.state.right, &fiction.feet[1]),
                ] {
                    let off = Vec2::new(
                        (foot.pos.x - fiction.hip.x) * facing,
                        -(foot.pos.y - fiction.hip.y),
                    );
                    let p = self.state.hip_pos + off;
                    leg.plant_pos = p;
                    leg.foot_pos = p;
                    leg.swing_from = p;
                    leg.swing_to = p;
                }
                fiction.hip
            }
        };

        // --- Stamp: measured horizontal displacement into the fiction ---
        // The solver's x axis is the TRAVEL frame (+x = facing), so vello
        // displacements enter multiplied by `facing`.
        let dx = (fiction.hip.x - anchor.x) * facing;
        self.state.hip_pos.x = self.seed_hip.x + dx;

        // --- Stamp: whole-body capture measure (DCM's reality input) ---
        // Floor line in the vello frame, implied by the seed anchor.
        let floor_vello_y = anchor.y + self.cfg.hip_height;
        self.state.measured = Some(walker_math::CaptureMeasure {
            com_x: self.state.hip_pos.x + (fiction.com.x - fiction.hip.x) * facing,
            v_com: fiction.v_com.x * facing,
            z_com: (floor_vello_y - fiction.com.y).max(1.0),
        });

        // --- Stamp: per-foot contact truth (the gait's landing event) ---
        // Measured foot x enters through the same hip-relative seed map as the
        // plants on the first tick, so `foot_x` re-anchors the stance plant in
        // the frame the solver actually draws in.
        let solver_hip_x = self.state.hip_pos.x;
        let to_solver_foot_x =
            |foot: &crate::observe::FootState| (foot.pos.x - fiction.hip.x) * facing + solver_hip_x;
        self.state.foot_contact = Some([
            LegContactInput {
                touching: fiction.feet[0].touching,
                foot_x: Some(to_solver_foot_x(&fiction.feet[0])),
            },
            LegContactInput {
                touching: fiction.feet[1].touching,
                foot_x: Some(to_solver_foot_x(&fiction.feet[1])),
            },
        ]);

        // --- One solver tick ---
        self.cfg.target_speed = cmd.speed.abs();
        // Command dir in the solver travel frame (+x = facing direction).
        let dir = Vec2::new((cmd.vx() * facing).signum(), 0.0);
        let (pose, debug) = walker_math::step_lower_body(dir, &self.cfg, &mut self.state, dt);

        // --- Map back: measured hip + travel-frame offsets to vello ---
        // The stance gate for the hip-foot strut is the gait FSM's own phase
        // (now event-driven: stance begins on real touchdown, ends on the DCM
        // crossing): a Stance leg holds its pinned plant.
        let hip = fiction.hip;
        // --- Attitude loop: close loop 3 against reality, or stay open ---
        // Travel frame like the first-tick seed: measured lean/rate × facing.
        // `attitude_lean_pd` commands `bias + kp·(bias−θ_meas) − kd·ω_meas`:
        // the model lean rides along as the bias (its integral memory stays
        // intact — no echo), and the PD term tilts the legs' reference axis
        // BACK against a measured forward lean. With zero gains this is
        // exactly the old `debug.theta * facing`.
        let lean = if cfg.att_closure_kp != 0.0 || cfg.att_closure_kd != 0.0 {
            walker_math::attitude_lean_pd(
                fiction.lean * facing,
                fiction.lean_rate * facing,
                debug.theta,
                cfg.att_closure_kp,
                cfg.att_closure_kd,
                self.cfg.theta_max,
            )
        } else {
            debug.theta
        };
        let upper_angle = lean * facing;
        let left_stance = self.state.left.phase == walker_math::GaitPhase::Stance;
        let right_stance = self.state.right.phase == walker_math::GaitPhase::Stance;
        let to_vello = |off: Vec2| Vec2::new(off.x * facing, -off.y);
        //
        // NOTE (tried and REVERTED 2026-11-17): mapping this pose's y through
        // the floor line instead of the measured hip. It fixes the hop (the
        // stance foot stops riding the hip's vault — see
        // `plans/actuator-frame-divergence.md` for the numbers) but the walk
        // gets WORSE end to end, and it needs a matching `pose.hip` command
        // plus a live-pivot hip target to avoid a kneel. Left untouched until
        // the hop can be addressed without re-anchoring the whole pose.
        // --- Stance press (`STANCE_DEPTH`): the LAST act before the actuator ---
        // Extend a STANCE leg's commanded foot along the hip→foot chord,
        // envelope-shaped across stance (mild at touchdown/liftoff, strongest
        // mid-stance); swing feet and `depth == 0` stay bit-identical. See
        // [`stance_press`]; nothing feeds back into the solver, so the pose
        // is a pure output preload.
        let ts = self.cfg.stance_duration_ts.max(f32::EPSILON);
        let depth = cfg.stance_press_depth;
        // NOTE (tried and REVERTED 2026-11-17): clamping a stance foot's y to
        // the floor line (`min(floor_vello_y)`) so `STANCE_DEPTH = 0` never
        // commands a plant below the ground. The penetration is real (the pose
        // rides the measured hip, so a low body buries the target by 10-16 px)
        // but the clamp breaks the walk: the leg loses the over-extension the
        // body was standing on. Same story as the floor-frame attempt — see
        // `plans/actuator-frame-divergence.md`.
        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: hip
                        + to_vello(stance_press(
                            depth,
                            ts,
                            left_stance,
                            self.state.left.phase_timer,
                            pose.left_foot - pose.hip,
                        )),
                    stance: left_stance,
                },
                right_foot: FootPose {
                    pos: hip
                        + to_vello(stance_press(
                            depth,
                            ts,
                            right_stance,
                            self.state.right.phase_timer,
                            pose.right_foot - pose.hip,
                        )),
                    stance: right_stance,
                },
                facing,
                upper_angle,
                stance_split: true,
            },
            // The solver's own integrated hip velocity, travel frame → vello
            // (x mirrored by facing, y negated for y-down).
            hip_vel: Vec2::new(pose.hip_vel.x * facing, -pose.hip_vel.y),
            // The law's target in the same Vello frame as the pose feet: the
            // solver's travel frame maps by `facing` about the solver hip, and
            // every plan is planted on the config floor line.
            plan: [
                hip + to_vello(Vec2::new(debug.left_target_x, self.cfg.floor_y) - pose.hip),
                hip + to_vello(Vec2::new(debug.right_target_x, self.cfg.floor_y) - pose.hip),
            ],
            mode: LocoMode::Walking,
            debug: Some(debug),
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Walking
    }
}

/// The launch (plan §5) — the stand→walk bridge: **tilt forward and gather
/// the feet under the hip.** Two jobs, in parallel, inside the boost cell.
///
/// # Why lean instead of push
/// The deadbeat DCM cannot bootstrap from a dead stop with an upright body:
/// ξ ≈ x_CoM sits under the hip, its first steps merely re-place the feet
/// under the CoM, and nothing falls. The old boost tried to create speed with
/// a skate push (sliding one foot toward under-hip), and the boost stopped
/// pushing entirely once it became a static leaned stance. But the measured
/// handover (`evt104.csv`) exposed the rest-pose geometry: the v10 blueprint
/// stands asymmetric (left foot 33 px BEHIND the hip, right foot 39.5 px
/// AHEAD of it). At handover ξ ≈ hip, so the DCM event lifts the *left*
/// (behind-ξ) foot — and the remaining support is the right foot, 39.5 px
/// AHEAD of ξ: a foot ahead of the capture point tips the body BACKWARD. The
/// first step was committed the wrong way, and the gait fell before it ever
/// had forward velocity.
///
/// # The gather
/// So the boost cell does two things: it tilts the stick AND slides the
/// FRONT foot — the one ahead of the hip in travel, right at +39.5 for an
/// eastward boost — inward to directly under the hip (`boost_gather_ts`
/// seconds, then holds). The back foot never moves (it is already behind the
/// hip, zero drag). At handover the lean has put ξ ahead of BOTH plants, and
/// the gathered front foot is the one ξ crossed most recently — so the DCM
/// event lifts *it* first, a step-in-place, and the remaining support is the
/// back foot 33 px BEHIND ξ: the body is planted to fall FORWARD. Gathering
/// both feet instead would leave ξ near-center on lift, inviting the
/// backward-tip / negative-velocity window the `evt104` trace showed; with
/// the back foot held, a foot is always strictly behind ξ while walking.
///
/// Handover is **command-driven**: the pose holds while the command carries
/// the `boost` flag, so for a script the `boost` cell's duration *is* the
/// gather+lean window (it must exceed `boost_gather_ts`), and a plain `move`
/// cell (or a released keyboard modifier) ends it.
#[derive(Debug, Clone, Copy)]
pub struct BoostStartCtrl {
    pub since: f32,
    /// Front-foot gather progress 0..1: 0 = rest offset, 1 = directly under
    /// the hip (x offset 0, rest depth). Ramps over `boost_gather_ts`. The
    /// back foot holds its rest offset throughout.
    pub gather: f32,
    /// Rig standing hip height (`LowerBodyConfig::hip_height`, vello y-down
    /// px). The support (right) plant is placed this depth below the hip
    /// instead of the blueprint's hardcoded 103 px, so `HIP_HEIGHT` crouches
    /// the launch exactly as it crouches the gait. `<= 0` / non-finite falls
    /// back to the rest depth.
    pub hip_height: f32,
    /// Bone reach (`thigh_len + calf_len`, px). The push-off is clamped to
    /// `reach − chord`, so it can never command a leg past full extension.
    pub reach: f32,
}

impl BoostStartCtrl {
    pub fn new(lower: &LowerBodyConfig) -> Self {
        Self {
            since: 0.0,
            gather: 0.0,
            hip_height: lower.hip_height,
            reach: lower.thigh_len + lower.calf_len,
        }
    }
}

impl Default for BoostStartCtrl {
    fn default() -> Self {
        Self::new(&LowerBodyConfig::default())
    }
}

impl Step for BoostStartCtrl {
    fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        self.since += dt.max(0.0);
        // Travel sign: command direction if moving, else facing (a boost
        // always carries a nonzero vx through the machine's entry gate).
        let s = if cmd.vx() != 0.0 {
            cmd.vx().signum()
        } else if facing < 0.0 {
            -1.0
        } else {
            1.0
        };
        // Gather only the FRONT foot (offset pointing along travel) to under
        // the hip — x offset 0, rest depth, chord 103-106 of a 110 leg, a
        // slight fold off the full-extension singularity — then hold. The
        // back foot stays planted at its rest offset; the tilt is unchanged.
        self.gather = (self.since / cfg.boost_gather_ts.max(1e-3)).clamp(0.0, 1.0);
        let gather = |off: Vec2| {
            if off.x * s > 0.0 {
                Vec2::new(off.x * (1.0 - self.gather), off.y)
            } else {
                off
            }
        };
        //
        // The RIGHT foot is the support (stance) foot: it is the v10 FRONT
        // foot for an eastward boost and the one the gather pulls under the
        // hip, so it carries the launch. Flagged stance, its strut is active
        // and it also takes the stance press (`STANCE_DEPTH`) — the same
        // chord-aligned extension `WalkingCtrl` applies — so the launch loads
        // the support leg into the floor instead of standing on a relaxed
        // chord that lets the leaned CoM keep rolling forward.
        //
        // The LEFT foot is the swing foot: it holds its rest offset behind the
        // hip and is placed by the angular joints only. Keeping one foot
        // strictly BEHIND the hip is what makes the handover commit the fall
        // forward, so the back foot stays put — unless the push-off is armed,
        // in which case the back leg is the one that pushes (see below).
        let hip = fiction.hip;
        // Rig hip height: the support plant sits at the commanded standing
        // height below the hip, not the blueprint's hardcoded 103 px, so
        // `HIP_HEIGHT` crouches the launch like it crouches the gait. The
        // whole rest pose shifts by the same amount, which keeps the v10
        // asymmetry (left foot 3 px lower than the right) intact.
        let hip_h = if self.hip_height.is_finite() && self.hip_height > 0.0 {
            self.hip_height
        } else {
            REST_RIGHT_FOOT.y
        };
        // The crouch is EASED, never stepped: the plants are holding the whole
        // body, so a depth step is a chord-velocity kick straight through the
        // struts (see `boost_crouch_ts`). At `since = 0` the pose is the
        // blueprint rest pose, i.e. what the body already has.
        let crouch = smoothstep01(self.since / cfg.boost_crouch_ts.max(1e-3));
        let rest_shift = Vec2::new(0.0, (hip_h - REST_RIGHT_FOOT.y) * crouch);
        let ts = cfg.boost_gather_ts.max(1e-3);
        // --- STEP 1: SQUASH (vertical only) -------------------------------
        // `rest_shift` eases both plant targets toward the hip over
        // `boost_crouch_ts`; nothing moves in x yet, and both feet are support
        // so the crouch pulls the HIP down onto planted feet.
        let left_off = gather(REST_LEFT_FOOT + rest_shift);
        let right_off = gather(REST_RIGHT_FOOT + rest_shift);
        // --- STEP 2: MOVE BOTH FEET BACKWARDS -----------------------------
        // Only after the squash has landed. BOTH plant targets translate
        // backwards along travel by the same amount, so the hips are pushed
        // forward whichever foot carries the load — a single back-foot kick
        // only pushes if that one foot grips (measured: it slid 6 px for 0.6 px
        // of hip travel). Each foot is limited by ITS OWN horizontal reach room
        // `sqrt(reach² − y²) − |x|`: the gathered front foot has ~50 px, the
        // back one only ~15 px at 96 and none at the v10 rest depth, which is
        // why the squash comes first.
        let step2_age = (self.since - cfg.boost_crouch_ts.max(1e-3)).max(0.0);
        let step2 = smoothstep01(step2_age / cfg.boost_kick_ts.max(1e-3));
        let reach = self.reach;
        // The BACK leg is the pusher (`boost_kick`); the FRONT foot is the
        // support and stays put (`boost_kick_front`, default 0). Dragging the
        // support backwards removes what the body is standing on.
        let left_back = REST_LEFT_FOOT.x * s <= 0.0;
        let kick_of = |back: bool| {
            let base = if back {
                cfg.boost_kick
            } else {
                cfg.boost_kick_front
            };
            base * step2
        };
        let push_age = step2_age;
        let push_off = |off: Vec2, kick: f32| {
            let room = (reach * reach - off.y * off.y).max(0.0).sqrt() - off.x.abs();
            let kicked = off + Vec2::new(-s * kick.min(room.max(0.0)), 0.0);
            // `BOOST_PUSH` (default 0) rides on top: extend the kicked chord
            // along itself, which reacts on the hip up-and-forward and lifts the
            // body out of the crouch. Clamped to the reach left too.
            let push = cfg.boost_push.min((reach - kicked.length()).max(0.0))
                * smoothstep01(push_age / cfg.boost_push_ts.max(1e-3));
            chord_extend(kicked, push)
        };
        let left_off = push_off(left_off, kick_of(left_back));
        let right_off = push_off(right_off, kick_of(!left_back));
        // BOTH feet are SUPPORT for the whole launch. A swing-flagged foot
        // rides the idle 1e-2 tether, so the crouch ramp merely lifts it off
        // the floor and the kick then drags it through the air — the back leg
        // must be a stance leg from the entry tick for its strut to carry load.
        let left_stance = true;
        let right_stance = true;
        // Press envelope: `stance_press` peaks at `age = t_stance / 2`, so the
        // launch ramps in over the first half of the gather window and then
        // holds at full depth (`self.since` is clamped, pinning `age`).
        let press_timer = ts - self.since.min(ts * 0.5);
        let depth = cfg.stance_press_depth;
        // Never command a chord past the bone reach: the back leg is already at
        // 111 px of a 110 px leg at the v10 rest depth, and the press adds to it.
        let left_off = clamp_len(
            stance_press(depth, ts, left_stance, press_timer, left_off),
            self.reach,
        );
        let right_off = clamp_len(
            stance_press(depth, ts, right_stance, press_timer, right_off),
            self.reach,
        );
        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: hip + left_off,
                    stance: left_stance,
                },
                right_foot: FootPose {
                    pos: hip + right_off,
                    stance: right_stance,
                },
                facing,
                // No pre-lean: the launch is upright and the KICK supplies the
                // forward drive (`boost_kick`). The old 0.05 rad tilt only
                // bought ~1.8 px of CoM offset and, because the torso target is
                // referenced to the live thigh, it also had to be paid for in
                // pitch. Upright keeps the attitude loop's reference vertical
                // for the whole cell.
                upper_angle: 0.0,
                stance_split: false,
            },
            // The boost commands no velocity: it is a held pose. Telemetry
            // reads the real forward creep only after the walking handover.
            hip_vel: Vec2::ZERO,
            plan: [Vec2::new(f32::NAN, f32::NAN); 2],
            mode: LocoMode::BoostStart,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::BoostStart
    }
}

/// The three controllers, one shared slot (the component field is this enum).
#[derive(Debug, Clone)]
pub enum Controller {
    Standing(StandingCtrl),
    Walking(WalkingCtrl),
    BoostStart(BoostStartCtrl),
}

impl Controller {
    /// Build the controller for a mode. `lower` supplies the gait config.
    pub fn for_mode(mode: LocoMode, lower: &LowerBodyConfig) -> Self {
        match mode {
            LocoMode::Standing => Controller::Standing(StandingCtrl::new(lower)),
            LocoMode::Walking => Controller::Walking(WalkingCtrl::new(lower)),
            LocoMode::BoostStart => Controller::BoostStart(BoostStartCtrl::new(lower)),
        }
    }

    /// Dispatch through the enum; one tick of control.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        match self {
            Controller::Standing(c) => c.step(cmd, fiction, cfg, dt, facing),
            Controller::Walking(c) => c.step(cmd, fiction, cfg, dt, facing),
            Controller::BoostStart(c) => c.step(cmd, fiction, cfg, dt, facing),
        }
    }

    pub fn mode(&self) -> LocoMode {
        match self {
            Controller::Standing(c) => c.mode(),
            Controller::Walking(c) => c.mode(),
            Controller::BoostStart(c) => c.mode(),
        }
    }
}

impl Default for Controller {
    fn default() -> Self {
        Controller::Standing(StandingCtrl::new(&LowerBodyConfig::default()))
    }
}

// ---------------------------------------------------------------------------
// State machine: decides only WHICH controller runs
// ---------------------------------------------------------------------------

/// The mode machine. It owns no gait state — only the current mode and how
/// long it has held; it reads nothing but the command and the fiction.
///
/// Three states, one per controller:
///
/// ```text
///               move + boost flag
///  Standing ────────────────────────► BoostStart
///    ▲  ▲                                  │  boost flag dropped
///    │  │  command ended (None / stop)     │  (plain move still held)
///    │  │                                  ▼
///    │  └──────────── Walking ◄───────── (leaned stance → gait)
///    │                 ▲   │
///    └─────────────────┘   └── command ended → Standing
///               plain move (no boost) goes Standing → Walking directly
/// ```
///
/// The boost exit is **event-driven, and scheduled**: the launch holds while
/// the `boost` flag is set AND the launch's own schedule has not played out
/// (squash `boost_crouch_ts`, then push `boost_kick_ts`) AND the DCM has not
/// yet reached the gathered front foot ([`boost_handover_due`]). Once all three
/// hold, the handover fires the instant ξ crosses the frontmost foot — the very
/// condition WalkingCtrl's own first `dcm_crossed` liftoff waits for — so the
/// gait opens with a step in place. A timer handover (the old cell-end release)
/// overshoots: the gather sweeps the foot *past* ξ, storing a gap the first step
/// must spend as a giant leap (measured v_filt ran to 220 px/s against a 120
/// command, past the reach envelope). A plain `move` cell (flag drop) still
/// releases as a fallback when the event never fires. Command end — `None`, or a
/// signed speed under [`ControlConfig::move_threshold`] — always returns to
/// Standing from any mode.
///
/// The schedule gate exists because the DCM event can fire *during* the squash
/// or a few ticks into the push: the body is given to the gait before the push
/// has delivered its speed, and the walk then starts from rest against the full
/// planned speed.
#[derive(Debug, Clone, Copy)]
pub struct LocoState {
    pub mode: LocoMode,
    /// Seconds in the current mode.
    pub since: f32,
}

impl Default for LocoState {
    fn default() -> Self {
        Self {
            mode: LocoMode::Standing,
            since: 0.0,
        }
    }
}

impl LocoState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Decide the mode for this tick.
    pub fn decide(
        &mut self,
        cmd: Option<Command>,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
    ) -> LocoMode {
        self.since += dt.max(0.0);
        // A held move command; anything else counts as no command at all.
        let active = cmd.filter(|c| c.is_move(cfg.move_threshold));

        let next = match (self.mode, active) {
            // Command ended (or never started): stand, from any mode.
            (_, None) => LocoMode::Standing,
            // Launch: a boosted move kicks the gait through BoostStart.
            (LocoMode::Standing, Some(c)) if c.boost => LocoMode::BoostStart,
            // A plain move enters the gait directly (already up to speed).
            (LocoMode::Standing, Some(_)) => LocoMode::Walking,
            (LocoMode::BoostStart, Some(c)) => {
                // Event handover: hold the launch while the boost flag is
                // set, until the DCM crosses the gathered front foot; the
                // first plain move releases it even if the event never fires.
                if c.boost && !boost_handover_due(&c, fiction, cfg, self.since) {
                    LocoMode::BoostStart
                } else {
                    LocoMode::Walking
                }
            }
            (LocoMode::Walking, Some(_)) => LocoMode::Walking,
        };

        if next != self.mode {
            self.mode = next;
            self.since = 0.0;
        }
        self.mode
    }
}

/// Stance-foot press (`STANCE_DEPTH`): extend a STANCE leg's commanded
/// hip→foot offset by `depth · sin(π · age / T_stance)` px **along the chord**,
/// i.e. away from the hip in the same direction the strut already pushes.
/// Mild at the stance boundaries (`age → 0` and `age → T_stance`, where the
/// envelope vanishes), strongest mid-stance. Returns `off` untouched when the
/// leg is in swing, when `depth == 0` (keeps the pose bit-identical), or for a
/// degenerate zero-length chord. Pure and side-effect free so it can be tested
/// directly. Two callers: `WalkingCtrl::step` (both feet, each gated by its own
/// stance flag) and `BoostStartCtrl::step` (the right/support foot only, with
/// its timer pinned at the envelope peak so the launch holds full depth).
fn stance_press(depth: f32, t_stance: f32, stance: bool, timer: f32, off: Vec2) -> Vec2 {
    if !stance || depth == 0.0 {
        return off;
    }
    // `timer` counts DOWN from `t_stance` after the solver tick; age is the
    // fraction of stance already spent, clamped so a stretched/early state can
    // never push the envelope outside [0, 1] cycles.
    let age = (t_stance - timer).clamp(0.0, t_stance);
    let env = ((age / t_stance) * std::f32::consts::PI).sin();
    chord_extend(off, depth * env)
}

/// Extend `off` along its own direction by `delta` px (`delta < 0` shortens),
/// i.e. push the foot away from the hip down the chord the strut already lies
/// on. The single definition of "push along the hip→foot chord" for both the
/// stance press and the boost push-off; returns `off` untouched for a
/// degenerate chord or a zero delta.
fn chord_extend(off: Vec2, delta: f32) -> Vec2 {
    let len = off.length();
    if delta == 0.0 || len <= f32::EPSILON {
        return off;
    }
    off * (1.0 + delta / len)
}

/// Scale `off` down to at most `max` px (direction preserved). The pose may
/// never command a chord longer than the leg — the solver would only fold it.
fn clamp_len(off: Vec2, max: f32) -> Vec2 {
    let l = off.length();
    if l > max && l > f32::EPSILON {
        off * (max / l)
    } else {
        off
    }
}

/// Smooth 0→1 with zero slope at both ends. Every planted-chord length change
/// goes through this: a step is a velocity kick through the whole body.
fn smoothstep01(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// The boost release event: has the DCM `ξ ≈ com + v_com·boost_lead_tau`
/// (vello x; the boost body is barely moving, so the CoM term dominates) led
/// past the FRONTMOST foot in travel? Walking's gather sweeps that foot to
/// meet ξ; at the crossing both feet are behind ξ — the state in which the
/// walking gait wants to liftoff the front foot *now* — and the launch
/// converts to the fall.
fn boost_handover_due(cmd: &Command, fiction: &Fiction, cfg: &ControlConfig, since: f32) -> bool {
    // The launch has a SCHEDULE — squash (`boost_crouch_ts`), then push
    // (`boost_kick_ts`) — and the DCM event alone can fire during the squash or
    // a few ticks into the push, which hands the body to the gait BEFORE the
    // push has delivered its speed. The walk then starts from (almost) rest
    // while the plan asks for the full speed, and the first step is a leap.
    // Hold the launch until the schedule has played out.
    let launch_done = cfg.boost_crouch_ts.max(0.0) + cfg.boost_kick_ts.max(0.0);
    if since < launch_done {
        return false;
    }
    // Travel sign: a boost always carries a nonzero vx through the entry gate.
    let s = cmd.vx().signum();
    let xi = fiction.com.x + fiction.v_com.x * cfg.boost_lead_tau;
    // No foot may remain ahead of ξ in the travel direction.
    fiction.feet.iter().all(|f| (f.pos.x - xi) * s <= 0.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::FootState;

    fn fiction(hip: Vec2, feet: [Vec2; 2], touching: [bool; 2], lean: f32, v: Vec2) -> Fiction {
        Fiction {
            dt: 1.0 / 90.0,
            hip,
            v_hip: v,
            lean,
            lean_rate: 0.0,
            com: hip,
            v_com: v,
            feet: [
                FootState {
                    pos: feet[0],
                    vel: Vec2::ZERO,
                    touching: touching[0],
                },
                FootState {
                    pos: feet[1],
                    vel: Vec2::ZERO,
                    touching: touching[1],
                },
            ],
            grounded: touching[0] || touching[1],
        }
    }

    fn centered() -> Fiction {
        // Feet at ±STANCE_SPREAD, exactly on the x-stance: standing holds x and
        // only lifts the hip toward the stand height.
        fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-10.0, -90.0), Vec2::new(10.0, -90.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        )
    }

    #[test]
    fn standing_emits_the_hardcoded_v10_rest_offsets() {
        // The frame is the v10 rest pose: constant offsets under the measured
        // `P3`, identical whatever the measured feet do and whether grounded.
        // This is what makes the emitted joint rests land in actuate's dead
        // zone — no feedback, no sag walk-away.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        for touching in [[true, true], [false, false]] {
            let f = fiction(
                Vec2::new(3.0, -120.0),
                [Vec2::new(-45.0, -11.0), Vec2::new(12.0, -80.0)],
                touching,
                0.0,
                Vec2::new(11.0, -7.0),
            );
            let mut c = StandingCtrl::new(&walker_math::LowerBodyConfig::default());
            let out = c.step(&Command::default(), &f, &cfg, dt, 1.0);
            assert_eq!(out.mode, LocoMode::Standing);
            assert_eq!(
                out.pose.hip, f.hip,
                "anchor: measured hip (world placement)"
            );
            assert_eq!(
                out.pose.left_foot.pos,
                f.hip + REST_LEFT_FOOT,
                "left foot is the hardcoded P3→PLLL offset"
            );
            assert_eq!(
                out.pose.right_foot.pos,
                f.hip + REST_RIGHT_FOOT,
                "right foot is the hardcoded P3→PRLL offset"
            );
            assert_eq!(out.pose.left_foot.stance, touching[0]);
            assert_eq!(out.pose.right_foot.stance, touching[1]);
            assert_eq!(out.pose.upper_angle, 0.0, "the rest P3→P2 axis is upright");
            assert_eq!(out.pose.facing, 1.0);
            assert!(out.debug.is_none(), "no gait internals in standing");
        }
    }

    #[test]
    fn the_rest_offsets_match_the_v10_blueprint_at_scale_half() {
        // Regression guard on the hardcoded numbers: blueprint SVG offsets
        // scaled by the 0.5 spawn factor.
        assert_eq!(REST_LEFT_FOOT, Vec2::new(-33.0, 106.0));
        assert_eq!(REST_RIGHT_FOOT, Vec2::new(39.5, 103.0));
    }

    #[test]
    fn control_out_carries_the_pose_and_the_actuator_never_sees_the_rest() {
        // `pose` is a field, not a copy: the ControlOut IS what actuate reads,
        // and hip_vel/mode/debug sit beside it where actuate cannot reach them.
        let out = ControlOut {
            pose: Pose {
                hip: Vec2::new(4.0, -140.0),
                left_foot: FootPose {
                    pos: Vec2::new(-10.0, -90.0),
                    stance: true,
                },
                right_foot: FootPose {
                    pos: Vec2::new(10.0, -90.0),
                    stance: false,
                },
                facing: 1.0,
                upper_angle: 0.1,
                stance_split: true,
            },
            hip_vel: Vec2::new(50.0, 0.0),
            plan: [Vec2::new(f32::NAN, f32::NAN); 2],
            mode: LocoMode::Standing,
            debug: None,
        };
        assert_eq!(out.pose.hip, Vec2::new(4.0, -140.0));
        assert_eq!(out.pose.left_foot.pos, Vec2::new(-10.0, -90.0));
        assert!(out.pose.left_foot.stance);
        assert!(!out.pose.right_foot.stance);
        assert_eq!(out.pose.facing, 1.0);
        assert_eq!(out.pose.upper_angle, 0.1);
        assert_eq!(
            out.hip_vel.x, 50.0,
            "velocity is beside the pose, not in it"
        );
    }

    #[test]
    fn hold_pose_stubs_are_inert_and_carry_their_mode() {
        let cfg = ControlConfig::default();
        let f = centered();
        let out = hold_pose(&f, &cfg, LocoMode::Walking, 1.0);
        assert_eq!(out.mode, LocoMode::Walking);
        assert_eq!(out.pose.hip, f.hip);
        assert_eq!(
            out.pose.upper_angle, f.lean,
            "hold must not invent an attitude"
        );
        assert_eq!(out.pose.facing, 1.0);
    }

    #[test]
    fn state_machine_starts_standing_and_stays_without_a_command() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let f = centered();
        for _ in 0..50 {
            let m = st.decide(None, &f, &cfg, 1.0 / 90.0);
            assert_eq!(m, LocoMode::Standing);
        }
        assert!(st.since > 0.5, "since accumulates while standing");
    }

    #[test]
    fn a_plain_move_starts_the_gait_directly() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let f = centered();
        let m = st.decide(Some(Command::walk(120.0)), &f, &cfg, 1.0 / 90.0);
        assert_eq!(m, LocoMode::Walking, "no boost flag ⇒ no launch window");
    }

    #[test]
    fn a_boosted_move_hands_over_when_the_boost_flag_drops() {
        // The script (or keyboard) owns the lean window: BoostStart holds the
        // pose exactly while the command carries `boost`; the first plain move
        // — same speed, same direction, flag gone — is the handover tick.
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let rest = centered();
        let dt = 1.0 / 90.0;
        let m = st.decide(Some(Command::boost(120.0)), &rest, &cfg, dt);
        assert_eq!(m, LocoMode::BoostStart);

        // Held boost keeps holding, however long (the pose is as stable as
        // standing; there is no timer to expire).
        for i in 0..900 {
            let m = st.decide(Some(Command::boost(120.0)), &rest, &cfg, dt);
            assert_eq!(m, LocoMode::BoostStart, "tick {i} still leaning");
        }

        // The plain move cell takes over on its very first tick.
        let m = st.decide(Some(Command::walk(120.0)), &rest, &cfg, dt);
        assert_eq!(m, LocoMode::Walking, "boost flag dropped => gait");
    }

    #[test]
    fn the_boost_holds_until_the_dcm_passes_the_front_foot() {
        // Event handover: a foot still AHEAD of ξ = com + v·τ keeps the
        // launch; the instant the frontmost foot falls behind ξ (the gather
        // swept it under, or the body raced past it) the gait takes over —
        // that crossing is exactly WalkingCtrl's first `dcm_crossed` liftoff.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        // Rest-like pose: front (right) foot 39.5 AHEAD of the hip/CoM.
        let holding = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut st = LocoState::new();
        st.decide(Some(Command::boost(120.0)), &holding, &cfg, dt);
        for i in 0..300 {
            assert_eq!(
                st.decide(Some(Command::boost(120.0)), &holding, &cfg, dt),
                LocoMode::BoostStart,
                "tick {i}: a foot ahead of ξ must keep the launch"
            );
        }
        // The gathered front foot sits under the hip — right with ξ: released.
        let arrived = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(0.0, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        assert_eq!(
            st.decide(Some(Command::boost(120.0)), &arrived, &cfg, dt),
            LocoMode::Walking,
            "ξ past every foot => the fall converts to the gait"
        );
        // A zooming body (ξ = 400·0.33 = 132) releases even with feet at hip —
        // but only once the launch SCHEDULE (squash + push) has played out.
        let mut st = LocoState::new();
        st.decide(Some(Command::boost(120.0)), &holding, &cfg, dt);
        let zooming = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::new(400.0, 0.0),
        );
        // Right at entry the schedule is not done: even a body that is already
        // past ξ must hold, or the walk starts before the push has delivered.
        assert_eq!(
            st.decide(Some(Command::boost(120.0)), &zooming, &cfg, dt),
            LocoMode::BoostStart,
            "the launch schedule outranks the DCM event"
        );
        for _ in 0..40 {
            st.decide(Some(Command::boost(120.0)), &holding, &cfg, dt);
        }
        assert_eq!(
            st.decide(Some(Command::boost(120.0)), &zooming, &cfg, dt),
            LocoMode::Walking,
            "velocity feeds the event through ξ, once the schedule is done"
        );
        // The flag-drop fallback still releases even when the event never
        // fires (front foot stays ahead of ξ):
        let m = st.decide(Some(Command::walk(120.0)), &holding, &cfg, dt);
        assert_eq!(m, LocoMode::Walking, "plain move lifts the flag anyway");
    }

    #[test]
    fn command_end_returns_any_mode_to_standing() {
        let cfg = ControlConfig::default();
        let f = centered();
        // From Walking: command gone ⇒ Standing.
        let mut st = LocoState::new();
        st.decide(Some(Command::walk(120.0)), &f, &cfg, 1.0 / 90.0);
        assert_eq!(st.mode, LocoMode::Walking);
        assert_eq!(st.decide(None, &f, &cfg, 1.0 / 90.0), LocoMode::Standing);

        // A sub-threshold command counts as ended, not held.
        let mut st = LocoState::new();
        st.decide(Some(Command::walk(120.0)), &f, &cfg, 1.0 / 90.0);
        let stop = Command {
            dir: Vec2::X,
            speed: cfg.move_threshold * 0.5,
            boost: false,
        };
        assert_eq!(
            st.decide(Some(stop), &f, &cfg, 1.0 / 90.0),
            LocoMode::Standing,
            "speed under the threshold = command ended"
        );
    }

    #[test]
    fn mode_entry_resets_since() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let f = centered();
        for _ in 0..10 {
            st.decide(None, &f, &cfg, 1.0 / 90.0);
        }
        st.decide(Some(Command::boost(120.0)), &f, &cfg, 1.0 / 90.0);
        assert!(
            st.since < 0.02,
            "entering BoostStart restarts the timeout clock"
        );
    }

    #[test]
    fn controller_slot_swaps_with_the_mode_and_dispatches() {
        let low = LowerBodyConfig::default();
        let cfg = ControlConfig::default();
        let mut c = Controller::for_mode(LocoMode::Standing, &low);
        assert_eq!(c.mode(), LocoMode::Standing);
        let f = centered();
        let out = c.step(&Command::default(), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(out.mode, LocoMode::Standing);

        // Swapping the slot is just a reassignment — the enum shares one field.
        c = Controller::for_mode(LocoMode::BoostStart, &low);
        assert_eq!(c.mode(), LocoMode::BoostStart);
        let out = c.step(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(out.mode, LocoMode::BoostStart);

        c = Controller::for_mode(LocoMode::Walking, &low);
        assert_eq!(c.mode(), LocoMode::Walking);
        let out = c.step(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(out.mode, LocoMode::Walking);
    }

    /// The boost's blueprint assertions assume the rig stands at the v10 rest
    /// depth (support foot 103 px below the hip): `HIP_HEIGHT` replaces that
    /// hardcoded depth, and `LowerBodyConfig::default()` is the sandbox's
    /// crouched 90. Pinning the height to the blueprint keeps the geometry
    /// tests on the unmodified rest pose.
    fn boost_lower() -> LowerBodyConfig {
        LowerBodyConfig {
            hip_height: REST_RIGHT_FOOT.y,
            ..LowerBodyConfig::default()
        }
    }

    #[test]
    fn boost_stays_upright_and_gathers_the_feet_under_the_hip() {
        // Kick off: this test is about the gather + the upright torso only.
        // The launch is UPRIGHT (the kick supplies the forward drive) and the
        // FRONT foot slides from its rest offset to directly under the hip over
        // `boost_gather_ts`, then holds. Both feet are support; a held pose
        // commands no hip velocity.
        let mut cfg = ControlConfig::default();
        cfg.boost_kick = 0.0; // step two has its own test
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new(&boost_lower());
        let out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        assert_eq!(out.mode, LocoMode::BoostStart);
        assert_eq!(out.pose.hip, f.hip, "frame anchored at measured hip");
        assert_eq!(
            out.pose.upper_angle, 0.0,
            "the launch is upright — the kick is the drive"
        );
        assert_eq!(
            (out.pose.left_foot.stance, out.pose.right_foot.stance),
            (true, true),
            "both feet are support: the back leg must be loaded before it kicks"
        );
        assert_eq!(out.hip_vel, Vec2::ZERO, "a held pose commands no velocity");
        // Tick 1 starts from the rest offsets and the FRONT foot (right, +x
        // travel) creeps inward; one more tick moves it <1 px toward hip x,
        // depth unchanged. The BACK foot (left) never moves.
        let o1 = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        let r0 = out.pose.right_foot.pos.x - f.hip.x;
        let r1 = o1.pose.right_foot.pos.x - f.hip.x;
        assert!(r0 < 40.0 && r0 > 38.0, "starts near the rest offset: {r0}");
        assert!(
            r1 < r0 && r0 - r1 < 1.5,
            "front foot gathering inward ~1 px/tick: {r0} -> {r1}"
        );
        assert_eq!(
            o1.pose.right_foot.pos.y,
            f.hip.y + REST_RIGHT_FOOT.y,
            "depth held at the rest depth"
        );
        assert!(
            (o1.pose.left_foot.pos - (f.hip + REST_LEFT_FOOT)).length() < 1.5,
            "back foot stays at its rest offset (chord clamped to the reach): {:?}",
            o1.pose.left_foot.pos - f.hip
        );

        // Hold for the gather window + slack: the FRONT foot (right, +x travel)
        // converges UNDER the hip (x offset 0, rest depth), the BACK foot stays
        // at its rest offset, and the pose then never changes.
        let mut last = out;
        for _ in 0..300 {
            last = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        assert!(
            (last.pose.right_foot.pos.x - f.hip.x).abs() < 0.01,
            "front foot directly under the hip: {:?}",
            last.pose.right_foot.pos.x
        );
        assert_eq!(
            last.pose.right_foot.pos.y,
            f.hip.y + REST_RIGHT_FOOT.y,
            "gathered at the rest depth (chord 103 of 110: a fold, off the singularity)"
        );
        assert!(
            (last.pose.left_foot.pos - (f.hip + REST_LEFT_FOOT)).length() < 1.5,
            "back foot stays at its rest offset (chord clamped to the reach)"
        );
        let settled = last;
        for _ in 0..60 {
            let o = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
            assert_eq!(o.pose.left_foot.pos, settled.pose.left_foot.pos);
            assert_eq!(o.pose.right_foot.pos, settled.pose.right_foot.pos);
            assert_eq!(o.pose.upper_angle, settled.pose.upper_angle);
        }
    }

    #[test]
    fn boost_gathers_the_front_foot_toward_the_command_travel_sign() {
        // A -x launch gathers the other foot: the FRONT foot (the offset
        // pointing along travel) moves under the hip, the back foot never does.
        // The torso stays upright either way; the step-two translation is off
        // here so the gather is what is measured.
        let mut cfg = ControlConfig::default();
        cfg.boost_kick = 0.0;
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new(&boost_lower());
        let west = Command {
            dir: Vec2::new(-1.0, 0.0),
            speed: 120.0,
            boost: true,
        };
        let mut out = c.step(&west, &f, &cfg, dt, 1.0);
        assert_eq!(
            out.pose.upper_angle, 0.0,
            "upright in both travel directions"
        );
        for _ in 0..300 {
            out = c.step(&west, &f, &cfg, dt, 1.0);
        }
        assert!(
            (out.pose.left_foot.pos.x - f.hip.x).abs() < 0.01,
            "westward: the left foot is the front foot, it gathers: {:?}",
            out.pose.left_foot.pos.x
        );
        let back = out.pose.right_foot.pos - f.hip;
        assert!(
            (back - REST_RIGHT_FOOT).length() < 0.5,
            "the back foot (right) stays at its rest offset (chord clamped to \
             the reach): {back:?}"
        );
        assert_eq!(
            (out.pose.left_foot.stance, out.pose.right_foot.stance),
            (true, true),
            "both feet are support, independent of travel"
        );
    }

    #[test]
    fn boost_applies_the_stance_press_to_the_right_foot_only() {
        // `STANCE_DEPTH` in the launch: the support (right) foot's chord is
        // extended along its own hip->foot direction, exactly like a walking
        // stance, while the swing (left) foot is left untouched and the press
        // ramps in (no step) over the first half of the gather window.
        let mut cfg = ControlConfig::default();
        cfg.stance_press_depth = 4.0;
        cfg.boost_kick = 0.0; // the press alone (no step-two translation)
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new(&boost_lower());
        let first = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        let first_r = first.pose.right_foot.pos - f.hip;
        let gathered_first = Vec2::new(
            REST_RIGHT_FOOT.x * (1.0 - dt / cfg.boost_gather_ts),
            REST_RIGHT_FOOT.y,
        );
        assert!(
            (first_r.length() - gathered_first.length()).abs() < 1.0,
            "the press ramps in rather than stepping: {first_r:?}"
        );

        // Hold through the full gather window: the gather lands under the hip
        // at rest depth, and the press adds exactly `depth` px along the chord.
        let mut out = first;
        for _ in 0..300 {
            out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        let right = out.pose.right_foot.pos - f.hip;
        assert!(right.x.abs() < 0.01, "gathered under the hip: {right:?}");
        assert!(
            (right.length() - (REST_RIGHT_FOOT.y.abs() + 4.0)).abs() < 0.01,
            "support chord pressed by STANCE_DEPTH: {right:?}"
        );
        assert!(right.y > 0.0, "the press extends away from the hip");
        // The back foot is a support foot too, so it takes the press — and its
        // chord is clamped to the reach (111 px + press would be 115 of 110).
        assert!(out.pose.left_foot.stance);
        assert!(
            (out.pose.left_foot.pos - f.hip).length()
                <= boost_lower().thigh_len + boost_lower().calf_len + 1e-3,
            "the back chord is pressed only up to the reach"
        );
        assert_eq!(
            (out.pose.left_foot.stance, out.pose.right_foot.stance),
            (true, true)
        );

        // Depth 0 (the default) keeps the gathered chord bit-identical.
        let mut plain = ControlConfig::default();
        plain.boost_kick = 0.0;
        let mut c0 = BoostStartCtrl::new(&boost_lower());
        let mut g = c0.step(&Command::boost(120.0), &f, &plain, dt, 1.0);
        for _ in 0..300 {
            g = c0.step(&Command::boost(120.0), &f, &plain, dt, 1.0);
        }
        let gathered = g.pose.right_foot.pos - f.hip;
        assert!(
            (gathered.length() - REST_RIGHT_FOOT.y.abs()).abs() < 0.01,
            "depth 0 keeps the gathered chord: {gathered:?}"
        );
    }

    #[test]
    fn boost_crouches_the_support_plant_to_the_configured_hip_height() {
        // `HIP_HEIGHT` reaches the launch: a lower standing height deepens the
        // gather (feet closer to the hip) while the v10 x offsets and the 3 px
        // left/right depth skew are preserved. An unseeded (zero) height falls
        // back to the blueprint depth. The kick is off here so this test sees
        // the crouch alone.
        let mut cfg = ControlConfig::default();
        cfg.boost_kick = 0.0;
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut low = boost_lower();
        low.hip_height = 96.0;
        let mut c = BoostStartCtrl::new(&low);
        let mut out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        for _ in 0..300 {
            out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        let right = out.pose.right_foot.pos - f.hip;
        let left = out.pose.left_foot.pos - f.hip;
        assert!(right.x.abs() < 0.01, "gathered under the hip: {right:?}");
        assert!(
            (right.y - 96.0).abs() < 1e-3,
            "the support plant depth IS the hip height: {right:?}"
        );
        assert!(
            (left.y - 99.0).abs() < 1e-3,
            "the 3 px blueprint skew is kept: {left:?}"
        );
        assert!(
            (left.x - REST_LEFT_FOOT.x).abs() < 1e-3,
            "the back foot's x offset never changes: {left:?}"
        );

        // Zero / unseeded height: the rest depth, exactly as before.
        let mut zero = boost_lower();
        zero.hip_height = 0.0;
        let mut c0 = BoostStartCtrl::new(&zero);
        let mut g = c0.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        for _ in 0..300 {
            g = c0.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        assert!(
            (g.pose.right_foot.pos.y - (f.hip.y + REST_RIGHT_FOOT.y)).abs() < 1e-3,
            "an unseeded height falls back to the blueprint depth"
        );
    }

    #[test]
    fn boost_eases_into_the_crouch_instead_of_stepping_it() {
        // A squash applied as a step yanks a planted chord (measured: a 5 px
        // step at HIP_HEIGHT=98 moved the hip 7→31 px/s down and −11→−61 px/s
        // BACKWARD in ONE tick, and the launch then never released). The depth
        // must ramp out of the rest pose the body already has.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut low = boost_lower();
        low.hip_height = 98.0;
        let mut c = BoostStartCtrl::new(&low);
        let first = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        let d0 = first.pose.right_foot.pos.y - f.hip.y;
        assert!(
            (d0 - REST_RIGHT_FOOT.y).abs() < 0.5,
            "no depth step on the entry tick: {d0}"
        );
        let mut out = first;
        let ticks = ((cfg.boost_crouch_ts * 1.5) / dt) as usize;
        for _ in 0..ticks {
            out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        let d1 = out.pose.right_foot.pos.y - f.hip.y;
        assert!(
            (d1 - 98.0).abs() < 0.02,
            "the crouch arrives at the rig hip height: {d1}"
        );
    }

    #[test]
    fn boost_push_extends_the_back_chord_up_to_the_reach_left() {
        // `BOOST_PUSH` extends the BACK (left, −x travel) leg along its own
        // hip→foot chord and flags it stance so its strut can push. The
        // extension is clamped to `thigh + calf − chord`: at the v10 rest
        // depth the back leg already sits at 111 px of a 110 px leg, so there
        // is no room to push until the crouch has bought some.
        let mut low = boost_lower();
        low.hip_height = 96.0;
        let mut cfg = ControlConfig::default();
        cfg.boost_kick = 0.0; // this test is about the chord extension alone
        cfg.boost_push = 20.0; // deliberately more than the headroom
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new(&low);
        let mut out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        for _ in 0..300 {
            out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        let left = out.pose.left_foot.pos - f.hip;
        let reach = low.thigh_len + low.calf_len;
        assert!(
            (left.length() - reach).abs() < 0.01,
            "clamped to the reach left: {} of {reach}",
            left.length()
        );
        let crouched = REST_LEFT_FOOT.y + 96.0 - REST_RIGHT_FOOT.y;
        assert!(
            (left.y / left.x - crouched / REST_LEFT_FOOT.x).abs() < 1e-4,
            "the push keeps the chord's own direction: {left:?}"
        );
        assert!(out.pose.left_foot.stance, "the pushing leg must be stance");
        // `BOOST_PUSH` extends BOTH chords along themselves (it is applied after
        // the step-two translation), so both land on the reach here.
        let right = out.pose.right_foot.pos - f.hip;
        assert!(
            (right.length() - reach).abs() < 0.01,
            "the front chord is extended to the reach too: {right:?}"
        );
        // Off (the default): both feet stay at their rest offsets.
        let plain = ControlConfig::default();
        let mut c0 = BoostStartCtrl::new(&boost_lower());
        let mut g = c0.step(&Command::boost(120.0), &f, &plain, dt, 1.0);
        for _ in 0..300 {
            g = c0.step(&Command::boost(120.0), &f, &plain, dt, 1.0);
        }
        assert!(
            g.pose.left_foot.stance,
            "both feet are support in the launch"
        );
        assert!(
            (g.pose.left_foot.pos - (f.hip + REST_LEFT_FOOT)).length() < 1.5,
            "no push ⇒ the back foot is the rest offset (chord clamped to reach)"
        );
    }

    #[test]
    fn boost_squashes_first_then_pushes_the_back_foot() {
        // Two steps, in this order: (1) squash vertically — both depths ease
        // toward `hip_height`, nothing moves in x; (2) the BACK foot translates
        // backwards along travel, so its leg extends behind the body and the
        // reaction pushes the hips forward (100 % of that displacement is
        // forward travel). The FRONT foot is the support and does not move:
        // dragging it backwards removes what the body is standing on.
        let cfg = ControlConfig::default(); // kick 15 px, ramp 0.04 s
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut low = boost_lower();
        low.hip_height = 96.0;
        let reach = low.thigh_len + low.calf_len;
        // Step 1 only: one tick after entry, before the squash has landed, the
        // plants have not moved in x at all.
        let mut c = BoostStartCtrl::new(&low);
        let first = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        let r0 = first.pose.right_foot.pos - f.hip;
        let l0 = first.pose.left_foot.pos - f.hip;
        assert!(
            r0.x > 38.0 && (l0.x - REST_LEFT_FOOT.x).abs() < 1.0,
            "step 1 is vertical only: no plant has been translated backwards \
             yet (only the gather has started): {r0:?} {l0:?}"
        );

        // Both steps settled.
        let mut out = first;
        for _ in 0..300 {
            out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        let right = out.pose.right_foot.pos - f.hip; // front foot (gathered)
        let left = out.pose.left_foot.pos - f.hip; // back foot
        assert!(
            right.x.abs() < 0.02,
            "the front foot is the support and stays under the hip: {right:?}"
        );
        assert!(
            left.x < REST_LEFT_FOOT.x - 14.0,
            "the back foot is driven well behind its rest offset (clamped to its \
             own ~15 px of room): {left:?}"
        );
        assert!(
            left.length() <= reach + 1e-3 && right.length() <= reach + 1e-3,
            "neither chord is commanded past the reach: {} {}",
            left.length(),
            right.length()
        );
        assert!(
            (right.y - 96.0).abs() < 0.02 && (left.y - 99.0).abs() < 0.02,
            "the squash depths survive step 2: {right:?} {left:?}"
        );
        assert!(
            out.pose.left_foot.stance && out.pose.right_foot.stance,
            "both legs are support through both steps"
        );

        // No squash, no reach room: step 2 cannot move the back foot at all.
        let mut c0 = BoostStartCtrl::new(&boost_lower());
        let mut g = c0.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        for _ in 0..300 {
            g = c0.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        let back = g.pose.left_foot.pos - f.hip;
        assert!(
            (back - REST_LEFT_FOOT).length() < 1.5,
            "no room ⇒ the back foot cannot translate: {back:?}"
        );
        // …and the front (support) foot never moves in x, squash or not.
        let front = g.pose.right_foot.pos - f.hip;
        assert!(
            front.x.abs() < 1.0,
            "the front foot is support: it does not translate: {front:?}"
        );
    }

    // --- WalkingCtrl: the walker_math seam ---

    /// A fiction with the hip at the solver's standing height, feet planted on
    /// the implied floor line (vello y-down: below the hip by +hip_height).
    fn standing_fiction(hip_x: f32, v: Vec2) -> Fiction {
        let hip = Vec2::new(hip_x, -140.0);
        fiction(
            hip,
            [hip + Vec2::new(-33.0, 106.0), hip + Vec2::new(39.5, 103.0)],
            [true, true],
            0.0,
            v,
        )
    }

    #[test]
    fn walking_anchors_the_pose_at_the_measured_hip() {
        // The solver runs in its own seed frame; the emitted pose rides on the
        // MEASURED hip (translation-invariant, like StandingCtrl), and the
        // feet land BELOW the hip (vello +y) within leg reach.
        let low = LowerBodyConfig::default();
        let cfg = ControlConfig::default();
        let mut c = WalkingCtrl::new(&low);
        let f = standing_fiction(0.0, Vec2::new(90.0, 0.0));
        let out = c.step(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(out.mode, LocoMode::Walking);
        assert_eq!(out.pose.hip, f.hip, "pose anchored at measured hip");
        assert!(out.debug.is_some(), "the gait tick carries solver debug");
        let l = out.pose.left_foot.pos - f.hip;
        let r = out.pose.right_foot.pos - f.hip;
        assert!(l.y > 0.0 && r.y > 0.0, "feet below the hip in y-down");
        let leg = low.thigh_len + low.calf_len;
        assert!(
            l.length() <= leg + 1.0 && r.length() <= leg + 1.0,
            "chords within reach: {l:?}, {r:?}"
        );
    }

    #[test]
    fn walking_seeds_the_fiction_velocity_from_the_measured_hip() {
        // The BoostStart handover fires at a real 90+ px/s. The solver's
        // integrator must START there, not at 0 (the deadbeat lead is
        // fiction-owned; a fiction-blind first plan re-creates the stall).
        let low = LowerBodyConfig::default();
        let cfg = ControlConfig::default();
        let mut c = WalkingCtrl::new(&low);
        let v = Vec2::new(95.0, 0.0);
        let f = standing_fiction(0.0, v);
        c.step(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0, 1.0);
        // hip_vel.x is solver-owned after the seed: it may only have been
        // shaped toward the plan cap this tick, never reset to zero.
        assert!(
            c.state.hip_vel.x > 50.0,
            "the seeded velocity survives the first tick: {}",
            c.state.hip_vel.x
        );
    }

    #[test]
    fn the_attitude_closure_is_inert_at_zero_gains() {
        // Zero gains must reproduce the old open-loop mapping exactly:
        // `upper_angle = debug.theta * facing`, bit for bit.
        let low = LowerBodyConfig::default();
        let mut cfg = ControlConfig::default();
        cfg.att_closure_kp = 0.0;
        cfg.att_closure_kd = 0.0;
        let mut c = WalkingCtrl::new(&low);
        let f = standing_fiction(0.0, Vec2::new(90.0, 0.0));
        let out = c.step(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(
            out.pose.upper_angle,
            out.debug.unwrap().theta,
            "zero gains echo the model lean exactly"
        );
    }

    #[test]
    fn the_attitude_closure_leans_the_command_back_from_a_measured_forward_lean() {
        // The body is MEASURABLY at 0.5 rad forward lean while the model (no
        // per-tick stamp) converges toward its own, smaller θ. The closure must
        // command LESS lean than the model — negative feedback, not the gain-1
        // echo that collapsed sync120. Identical streams keep `debug.theta`
        // equal across both controllers: the closure only maps the output, it
        // never perturbs the solver state.
        let low = LowerBodyConfig::default();
        let dt = 1.0 / 90.0;
        let mut open = ControlConfig::default();
        open.att_closure_kp = 0.0;
        open.att_closure_kd = 0.0;
        let mut closed = ControlConfig::default();
        closed.att_closure_kp = 0.4;
        closed.att_closure_kd = 0.0;
        let f = Fiction {
            lean: 0.5,
            ..standing_fiction(0.0, Vec2::new(90.0, 0.0))
        };
        let mut co = WalkingCtrl::new(&low);
        let mut cc = WalkingCtrl::new(&low);
        let mut last_open = 0.0f32;
        let mut last_closed = 0.0f32;
        for _ in 0..30 {
            let ao = co.step(&Command::walk(120.0), &f, &open, dt, 1.0);
            let ac = cc.step(&Command::walk(120.0), &f, &closed, dt, 1.0);
            assert_eq!(
                ao.debug.unwrap().theta,
                ac.debug.unwrap().theta,
                "the closure must not perturb the solver state"
            );
            last_open = ao.pose.upper_angle;
            last_closed = ac.pose.upper_angle;
        }
        assert!(
            last_closed < last_open - 0.05,
            "correction backs the command off the measured lean: open={last_open:.3}, closed={last_closed:.3}"
        );
    }

    #[test]
    fn walking_opens_the_gait_once_the_fiction_passes_the_trigger() {
        // Event-driven schedule: a swing opens when the DCM ξ = CoM + v/ω₀
        // reaches the stance toe — never because a stopwatch expired. So the
        // fixture simulates a TRACKING body: each tick the fiction hip
        // advances with the solver's own emitted velocity, letting v (and ξ's
        // lead over the plants) build until each plant is crossed in turn.
        // Direction is not assumed: from rest the deadbeat law plants behind
        // the CoM to speed up, so the check is "a foot left its seed plant".
        let low = LowerBodyConfig::default();
        let cfg = ControlConfig::default();
        let mut c = WalkingCtrl::new(&low);
        let leg = low.thigh_len + low.calf_len;
        let dt = 1.0 / 90.0;
        let mut hip = Vec2::new(0.0, -140.0);
        let mut v = Vec2::ZERO;
        let mut saw_swing = false;
        let mut moved = 0.0f32;
        for _ in 0..90 {
            let f = fiction(
                hip,
                [hip + Vec2::new(-33.0, 106.0), hip + Vec2::new(39.5, 103.0)],
                [true, true],
                0.0,
                v,
            );
            let out = c.step(&Command::walk(120.0), &f, &cfg, dt, 1.0);
            let l = out.pose.left_foot.pos - f.hip;
            let r = out.pose.right_foot.pos - f.hip;
            assert!(l.is_finite() && r.is_finite());
            assert!(
                l.length() <= leg + 1.0 && r.length() <= leg + 1.0,
                "chords within reach: {l:?}, {r:?}"
            );
            if !out.pose.left_foot.stance || !out.pose.right_foot.stance {
                saw_swing = true;
            }
            // Displacement from the seeded plants (±30 at hip_height 90).
            moved = moved.max((l.x + 30.0).abs()).max((r.x - 30.0).abs());
            // Perfect-tracking plant: the body follows the solver's fiction.
            v = out.hip_vel;
            hip.x += v.x * dt;
        }
        assert!(saw_swing, "a rolling body crossed its toe and swung");
        assert!(
            moved > 10.0,
            "a plant moved under the placement law: {moved}"
        );
    }

    #[test]
    fn walking_mirrors_the_travel_frame_into_vello_space() {
        // facing = -1: the solver's +x TRAVEL is vello −x. A −x command on a
        // body measurably moving −x seeds +95 in the travel frame; the mapped
        // `hip_vel` must keep reporting vello −x travel.
        let low = LowerBodyConfig::default();
        let cfg = ControlConfig::default();
        let mut c = WalkingCtrl::new(&low);
        let f = standing_fiction(0.0, Vec2::new(-95.0, 0.0));
        let cmd = Command {
            dir: Vec2::new(-1.0, 0.0),
            speed: 120.0,
            boost: false,
        };
        let mut out = hold_pose(&f, &cfg, LocoMode::Walking, -1.0);
        for _ in 0..20 {
            out = c.step(&cmd, &f, &cfg, 1.0 / 90.0, -1.0);
        }
        assert!(
            out.hip_vel.x < -20.0,
            "facing -1 travel maps to vello -x: {}",
            out.hip_vel.x
        );
        assert!(out.pose.left_foot.pos.is_finite() && out.pose.right_foot.pos.is_finite());
    }

    #[test]
    fn control_is_deterministic_for_identical_input_streams() {
        let cfg = ControlConfig::default();
        let low = LowerBodyConfig::default();
        let run = || {
            let mut c = Controller::for_mode(LocoMode::Standing, &low);
            let mut st = LocoState::new();
            let mut outs = Vec::new();
            for i in 0..40 {
                let v = Vec2::new(i as f32, 0.0);
                let f = fiction(
                    Vec2::new(i as f32, -140.0),
                    [Vec2::new(-20.0, -90.0), Vec2::new(20.0, -90.0)],
                    [i % 4 != 0, i % 5 != 0],
                    0.01 * i as f32,
                    v,
                );
                let cmd = if i % 3 == 0 {
                    None
                } else if i < 20 {
                    Some(Command::boost(100.0))
                } else {
                    Some(Command::walk(100.0))
                };
                let mode = st.decide(cmd, &f, &cfg, 1.0 / 90.0);
                if mode != c.mode() {
                    c = Controller::for_mode(mode, &low);
                }
                let out = c.step(&cmd.unwrap_or_default(), &f, &cfg, 1.0 / 90.0, 1.0);
                outs.push((
                    out.mode,
                    (out.pose.hip.x * 1000.0).round() as i64,
                    (out.pose.upper_angle * 1000.0).round() as i64,
                ));
            }
            outs
        };
        let a = run();
        let b = run();
        assert_eq!(a, b, "same inputs ⇒ same outputs (D1)");
    }

    #[test]
    fn stance_press_is_chord_aligned_envelope_shaped_and_stance_only() {
        // Geometry: hip at solver origin, foot 100 px directly BELOW it in the
        // solver's y-UP frame (a perfectly vertical leg). The press must lengthen
        // the hip→foot chord — never tilt it — so the x component stays 0.
        let off = Vec2::new(0.0, -100.0);
        let t = 0.28;
        let depth = 6.0;

        // Swing foot: untouched, bit-identical.
        assert_eq!(
            stance_press(depth, t, false, t * 0.5, off),
            off,
            "swing leg not pressed"
        );

        // depth == 0: inert even in stance (bit-identical pose when off).
        assert_eq!(
            stance_press(0.0, t, true, t * 0.5, off),
            off,
            "zero depth is inert"
        );

        // Mid-stance (age = t/2): envelope peaks at sin(π/2) = 1 ⇒ +depth px,
        // purely along the chord (direction unchanged, length +depth).
        let mid = stance_press(depth, t, true, t - t * 0.5, off);
        assert_eq!(mid.x, 0.0, "press stays on the vertical chord");
        assert!(
            (mid.length() - (100.0 + depth)).abs() < 1e-3,
            "mid-stance press = depth along chord: {}",
            mid.length()
        );
        // Direction preserved (still straight down in solver frame).
        assert!(
            mid.y < 0.0 && mid.y < off.y,
            "foot pushed FURTHER from hip, not lifted"
        );

        // Boundaries: touchdown (age 0) and liftoff (age t) envelope → 0.
        let td = stance_press(depth, t, true, t, off);
        assert!(
            (td - off).length() < 1e-4,
            "touchdown is mild (envelope→0): {td:?}"
        );
        let lo = stance_press(depth, t, true, 0.0, off);
        assert!(
            (lo - off).length() < 1e-4,
            "liftoff is mild (envelope→0): {lo:?}"
        );

        // Monotone ramp-up on the first half, ramp-down on the second.
        let quarter = stance_press(depth, t, true, t - t * 0.25, off).length();
        let three_quarter = stance_press(depth, t, true, t - t * 0.75, off).length();
        assert!(
            quarter > td.length() && quarter < mid.length(),
            "quarter-stance rises toward the peak: {quarter}"
        );
        assert!(
            (three_quarter - quarter).abs() < 1e-2,
            "envelope is symmetric about mid-stance: {quarter} vs {three_quarter}"
        );

        // Degenerate zero-length chord must not divide by zero.
        assert_eq!(
            stance_press(depth, t, true, t * 0.5, Vec2::ZERO),
            Vec2::ZERO
        );
    }
}
