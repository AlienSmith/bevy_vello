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
    /// instead of entering the gait directly. The boost is a static leaned
    /// stance ([`ControlConfig::boost_lean`]) held **while this flag is set**
    /// — for a script, the `boost` cell's duration is the lean window; the
    /// machine hands over to [`LocoMode::Walking`] the moment the flag drops
    /// (typically a following `move` cell), whose first steps place the feet
    /// under the tilted CoM and start the fall forward.
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
    /// Boost: commanded upper-body forward lean, rad (vello frame: `+` leans
    /// toward `+x`), held against the gathered feet — the CoM (and with it ξ)
    /// ends the boost cell AHEAD of both plants, so the first handover step
    /// commits the fall forward.
    pub boost_lean: f32,
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
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            stand_settle_rate: 40.0,
            move_threshold: 5.0,
            // Past the gait's own cruise lean (~0.15 rad at 120 px/s): with
            // the feet gathered under the hip, ξ = CoM + v/ω₀ ≈ hip + 20 px
            // ahead of both plants at handover — the first step falls FORWARD.
            boost_lean: 0.25,
            // Front foot sweeps 39.5 px inward over this window; the handover
            // event fires partway through it (~0.3 s), so the gather only has
            // to be fast enough to MEET ξ, never overshoot past it.
            boost_gather_ts: 0.4,
            // 1/ω₀ at hip_height ≈ 104 (ω₀ = √(980/104) ≈ 3.07): the DCM's
            // velocity lead over the CoM.
            boost_lead_tau: 0.33,
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
        },
        hip_vel: fiction.v_hip,
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
            },
            hip_vel: Vec2::ZERO,
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
            cfg: cfg.clone(),
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
        _cfg: &ControlConfig,
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
        let upper_angle = debug.theta * facing;
        let left_stance = self.state.left.phase == walker_math::GaitPhase::Stance;
        let right_stance = self.state.right.phase == walker_math::GaitPhase::Stance;
        let to_vello = |off: Vec2| Vec2::new(off.x * facing, -off.y);
        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: hip + to_vello(pose.left_foot - pose.hip),
                    stance: left_stance,
                },
                right_foot: FootPose {
                    pos: hip + to_vello(pose.right_foot - pose.hip),
                    stance: right_stance,
                },
                facing,
                upper_angle,
            },
            // The solver's own integrated hip velocity, travel frame → vello
            // (x mirrored by facing, y negated for y-down).
            hip_vel: Vec2::new(pose.hip_vel.x * facing, -pose.hip_vel.y),
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
}

impl BoostStartCtrl {
    pub fn new() -> Self {
        Self {
            since: 0.0,
            gather: 0.0,
        }
    }
}

impl Default for BoostStartCtrl {
    fn default() -> Self {
        Self::new()
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
        // Both feet are commanded as STANCE (strut active), regardless of the
        // measured touch: the lean puts the CoM past the static margin, and a
        // foot flagged swing rides the idle tether (1e-2 = no authority), so
        // the measured lean crept through a self-reinforcing forward rock
        // (0.25 -> 0.70 rad over 2 s, left foot peeling at ~1.7 s). The active
        // strut holds the chord planted through the pull-in.
        let hip = fiction.hip;
        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: hip + gather(REST_LEFT_FOOT),
                    stance: true,
                },
                right_foot: FootPose {
                    pos: hip + gather(REST_RIGHT_FOOT),
                    stance: true,
                },
                facing,
                upper_angle: s * cfg.boost_lean,
            },
            // The boost commands no velocity: it is a held pose. Telemetry
            // reads the real forward creep only after the walking handover.
            hip_vel: Vec2::ZERO,
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
            LocoMode::BoostStart => Controller::BoostStart(BoostStartCtrl::new()),
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
/// The boost exit is **event-driven**: the launch holds while the `boost`
/// flag is set AND the DCM has not yet reached the gathered front foot
/// ([`boost_handover_due`]). The handover fires the instant ξ crosses the
/// frontmost foot — the very condition WalkingCtrl's own first `dcm_crossed`
/// liftoff waits for — so the gait opens with a step in place. A timer
/// handover (the old cell-end release) overshoots: the gather sweeps the foot
/// *past* ξ, storing a gap the first step must spend as a giant leap (measured
/// v_filt ran to 220 px/s against a 120 command, past the reach envelope).
/// A plain `move` cell (flag drop) still releases as a fallback when the
/// event never fires. Command end — `None`, or a signed speed under
/// [`ControlConfig::move_threshold`] — always returns to Standing from any
/// mode.
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
                if c.boost && !boost_handover_due(&c, fiction, cfg) {
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

/// The boost release event: has the DCM `ξ ≈ com + v_com·boost_lead_tau`
/// (vello x; the boost body is barely moving, so the CoM term dominates) led
/// past the FRONTMOST foot in travel? Walking's gather sweeps that foot to
/// meet ξ; at the crossing both feet are behind ξ — the state in which the
/// walking gait wants to liftoff the front foot *now* — and the launch
/// converts to the fall.
fn boost_handover_due(cmd: &Command, fiction: &Fiction, cfg: &ControlConfig) -> bool {
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
            },
            hip_vel: Vec2::new(50.0, 0.0),
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
        // A zooming body (ξ = 400·0.33 = 132) releases even with feet at hip:
        let mut st = LocoState::new();
        st.decide(Some(Command::boost(120.0)), &holding, &cfg, dt);
        let zooming = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::new(400.0, 0.0),
        );
        assert_eq!(
            st.decide(Some(Command::boost(120.0)), &zooming, &cfg, dt),
            LocoMode::Walking,
            "velocity feeds the event through ξ"
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

    #[test]
    fn boost_tilts_and_gathers_the_feet_under_the_hip() {
        // Two jobs in the boost cell: the stick tilts toward travel AND both
        // feet slide from their rest offsets to directly under the hip over
        // `boost_gather_ts`, then hold. Feet stay stance:true (strut active)
        // throughout; a held pose commands no hip velocity.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new();
        let out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        assert_eq!(out.mode, LocoMode::BoostStart);
        assert_eq!(out.pose.hip, f.hip, "frame anchored at measured hip");
        assert_eq!(out.pose.upper_angle, cfg.boost_lean, "the stick tilts");
        assert!(out.pose.left_foot.stance && out.pose.right_foot.stance);
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
        assert_eq!(
            o1.pose.left_foot.pos,
            f.hip + REST_LEFT_FOOT,
            "back foot never moves"
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
        assert_eq!(
            last.pose.left_foot.pos,
            f.hip + REST_LEFT_FOOT,
            "back foot never moves"
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
    fn boost_lean_and_gather_toward_the_command_travel_sign() {
        // A -x boost mirrors the lean AND gathers the other foot: the FRONT
        // foot (the offset pointing along travel) moves under the hip, the
        // back foot never does.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new();
        let west = Command {
            dir: Vec2::new(-1.0, 0.0),
            speed: 120.0,
            boost: true,
        };
        let mut out = c.step(&west, &f, &cfg, dt, 1.0);
        assert_eq!(out.pose.upper_angle, -cfg.boost_lean, "lean mirrors travel");
        for _ in 0..300 {
            out = c.step(&west, &f, &cfg, dt, 1.0);
        }
        assert!(
            (out.pose.left_foot.pos.x - f.hip.x).abs() < 0.01,
            "westward: the left foot is the front foot, it gathers: {:?}",
            out.pose.left_foot.pos.x
        );
        assert_eq!(
            out.pose.right_foot.pos,
            f.hip + REST_RIGHT_FOOT,
            "the back foot (right) never moves"
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
}
