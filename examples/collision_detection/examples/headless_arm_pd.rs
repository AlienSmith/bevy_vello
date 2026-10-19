//! Headless **command-driven** character harness: the limb-PD waggle gate **and**
//! the whole-body hip-pin / Raibert traversal, in **one** binary.
//!
//! One harness drives any single limb of the **V10** character — right/left arm
//! or right/left leg — through a **command script** and reports whether the limb
//! PD controller **settles** or **waggles**, so good `Kp`/`Kd` values can be
//! found. Arms and legs are structurally identical (`[ref, pivot, mid, end]` /
//! `[joint0, joint1]`) and share [`drive_limb`](game_lib) + `calculate_limb_ik`,
//! so they share this one tool; the only thing that differs is the input.
//!
//! ## Input: a command script, not code edits
//!
//! The playhead is a flat, ordered list of **self-contained commands** — each
//! groups everything it needs (which limb, where to, its gains, how long to
//! hold). No phase machine, no cross-entry coupling. Commands are deserialized
//! from JSON (env `SCRIPT=path.json`); when absent a built-in script is used.
//!
//! ```json
//! { "script": [
//!   { "reach": { "limb": "right_leg", "dir": [1.0, -0.1], "dur": 1.2 } },
//!   { "wait":  { "dur": 0.6 } },
//!   { "reach": { "limb": "right_leg", "dir": [-1.0, -0.1], "kp": 0.5, "kd": 0.065, "dur": 1.2 } },
//!   { "aim":   { "limb": "right_arm", "dir": [1.0, 0.0], "dur": 1.2 } },
//!   { "set":   { "limb": "right_arm", "kp": 0.4, "kd": 0.05 } },
//!   { "reset": { "limb": "right_leg" } },
//!   { "wait":  { "dur": 0.9 } }
//! ] }
//! ```
//!
//! | Command | Effect |
//! |---|---|
//! | `reach` | Position IK (`IkMode::Reach`): drive `limb` end to `to` (world Bevy y-up) or `anchor + dir*dist`. |
//! | `aim`   | Direction IK (`IkMode::Aim`): aim `limb` end along `dir` at `dist` from the pivot. |
//! | `set`   | Persistently set `limb` gains (`kp`/`kd`) without moving. |
//! | `reset` | Emit a limb constraint reset (restores authored rest angles). |
//! | `wait`  | Hold the current command (let it settle) for `dur` seconds. |
//! | `move`  | Hold a horizontal body input for `dur` s (`dir[0]` sign → left/right) — drives the hip pin. |
//! | `jump`  | Hold the jump input for `dur` s (a rising edge → one hop) — drives the hip pin. |
//!
//! `to`/`dir` are `[x, y]`; when `to`/`dist` are omitted the target is placed
//! `dist` (default = the limb's characteristic length) from the limb pivot along
//! `dir`, so a script needs no knowledge of the assembled rest pose.
//!
//! ## What is real here
//!
//! - character assembly from `v10.character.svg` + `v10.character.json` (game_lib)
//! - the real limb controller (`update_character_movement`, `FixedUpdate`) which
//!   reads `*Controller.target` + `config.ik_mode` and runs `calculate_limb_ik` —
//!   the PD under test
//! - the real XPBD connection/angular physics (`FixedUpdate` @ 90 Hz)
//!
//! ## Two modes (chosen by the script)
//!
//! - **Limb mode** (no `move`/`jump`): the spine drive is silenced and there is
//!   **no** `HipPin`, so the `P1/P2/P3` frame is passive; zero gravity, no
//!   colliders — the isolated-limb PD measurement. `SWEEP` applies here.
//! - **Whole-body mode** (a `move`/`jump` script, or `HIPPIN=1`): a `HipPin` is
//!   attached, [`drive_hip_pin`](game_lib) runs in the fixed chain before the leg
//!   IK, and the row carries the pin's `cmd_vx`/`vphase`. The run then reports the
//!   pinned-frame **rigidity**, the **jump→walk** commanded-vs-realized hip
//!   velocity verdict, the per-leg **foot-lag/wobble** verdict, and writes an
//!   `<csv>.events.csv` input timeline. This is the retired `headless_pin` gate,
//!   now command-driven.
//!
//! ## The PD knobs (verified, in `calculate_limb_ik`)
//!
//! | Knob | Field | Default |
//! |---|---|---|
//! | **Kp** | `LimbConfig::rest_chase_alpha` | 0.5 |
//! | **Kd** | `LimbConfig::kd` | 0.065 s |
//!
//! `SWEEP=1` replays the **same script** once per `(Kp,Kd)` cell, overriding the
//! gains on every gain-bearing command; a single CSV row is written per fixed tick
//! carrying all **13** v10 particle channels. Each cell is then classified.
//!
//! ## Verdict (per cell)
//!
//! Measured on the **focus** limb (env `LIMB`) end-bone angle over the script's
//! final hold window:
//!
//! - `DIVERGED` — `|theta|` range > 360° / non-finite
//! - `NO_SWING` — the measured arc is < 60° (the limb never moved — a rig problem)
//! - `WAGGLE`   — settled-tail RMS > ~1° or ≥ 4 angular-rate sign-reversals
//!   (measured only over the last 0.6 s, so a monotonic catch-up is not mistaken
//!   for oscillation)
//! - `SLOW`     — settles too late (or not within the final window)
//! - `PASS`     — settles quickly and quietly
//!
//! ## Env
//!
//! `LIMB` = `right_arm`|`left_arm`|`right_leg`|`left_leg` (focus limb + default
//! script; default `right_arm`), `SCRIPT` (path to a JSON script), `KP`, `KD`
//! (≤0 = auto/unset), `TARGET_DIST` (px; ≤0 = auto, aim only), `OUT_CSV`
//! (main CSV), `SWEEP=1`, `GRID="kp:kd,kp:kd,..."`.
//! Whole-body: `HIPPIN=1` (force even without a `move`/`jump`), `GRAVITY`
//! (`on`|`off`|`x,y` vello frame), `LEGS` (`off` = pin only `P1/P2/P3`),
//! `COMPLIANCE`, `DAMPING`. `COLLISION=1` adds a headless wgpu/vello GPU
//! narrow phase, the CPU BVH broad phase, and a static floor at the resolver
//! floor line. Output defaults under `/home/ubuntu/bender/tmp/headless/`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use bevy::{
    app::ScheduleRunnerPlugin,
    asset::{AssetMetaCheck, AssetPlugin},
    input::{ButtonInput, InputSystem},
    prelude::*,
    time::TimeUpdateStrategy,
    transform::TransformPlugin,
    MinimalPlugins,
};

use leafwing_input_manager::{plugin::InputManagerSystem, prelude::ActionState};

use serde::Deserialize;

use bevy_vello::vello::kurbo::Shape as _;

use bevy_vello::{
    collision::{
        path_to_ccw_quad_path, CollisionEventBatch, GpuCollisionRunner, RemovedColliders,
        VelloCollider, VelloCollisionBroadPhase, VelloCollisionWorld, VelloRayTraceCommand,
        VELLO_COLLISION_COOL_DOWN_TIME,
    },
    integrations::physics::{
        systems::{
            apply_explicit_impulse_on_connection_particle, apply_explicit_impulse_on_softbody,
            generate_connection, generate_soft_body_for_collider, make_collision_constraints,
            remove_soft_body, run_broad_phase, run_gpu_collision, update_collider_from_soft_body,
            update_connection_particles, update_constraint_world,
        },
        CharacterAngularConstraintEvent, CharacterFrameForceEvent, CharacterPivotForceEvent,
        CharacterPivotImpulseEvent, CharacterPivotVelocityEvent, ColliderExternalImpulseEvent,
        VelloCharacterPhysicsRoot, VelloConstraintWorld, VelloParticle,
    },
    prelude::*,
};

use wgpu;

use game_lib::{
    capture_telemetry, cell_verdict,
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
    },
    check_bone_rigidity, check_invariants, default_input_map, drive_hip_pin, flush_csv, flush_rows,
    rows_hash, tail_verdict, update_character_movement, update_ground_support, Bone,
    CharacterController, CharacterRoot, GroundSupport, HeadlessTelemetryPlugin, HipPin,
    HipPinParams, IkMode, InvariantConfig, LeftArmController, LeftLegController, LimbConfig,
    ParticleTelemetryConfig, PlayerAction, PlayerMarker, ResetLimbControlConstraintsEvent,
    RightArmController, RightLegController, SpineDriveEnabled, TelemetryBuffer, TelemetryRow,
    VelloCharacterPlugin, VerticalPhase, WhichLimb,
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Asset root override (for local Windows runs). Falls back to the CI/Linux
/// absolute path when unset, so the existing Linux pipeline is unchanged.
fn asset_root() -> String {
    std::env::var("ASSET_ROOT")
        .ok()
        .filter(|p| !p.trim().is_empty())
        .unwrap_or_else(|| {
            "/home/ubuntu/bender/bevy_vello/examples/collision_detection/assets".to_string()
        })
}
const SVG_ASSET: &str = "v10.character.svg";
const BP_ASSET: &str = "v10.character.json";
const SVG_PATH: &str = "character/v10.character.svg";
const BP_PATH: &str = "character/v10.character.json";

/// The physics cadence the controller + XPBD step at.
const FIXED_HZ: f64 = 90.0;
/// Hard cap on cells (safety against a typo'd grid).
const MAX_CELLS: usize = 64;
/// Angle band (deg) that counts as "settled".
const SETTLE_BAND_DEG: f32 = 1.5;
/// Minimum arc (deg) for the motion to count as a real swing.
const MIN_SWING_DEG: f32 = 60.0;
/// Width (s) of the settled tail measured at the end of the final hold window.
const TAIL_S: f32 = 0.6;
/// Residual RMS (deg, about the tail mean) above which the tail is judged noisy.
const TAIL_RMS_DEG: f32 = 1.0;
/// Angular-rate sign-reversals (with a deadband) within the tail above which the
/// limb is judged to oscillate — a direct, transient-immune wobble detector.
const WAGGLE_REVERSALS: u32 = 4;
/// `|omega|` (rad/s) below which a sample is ignored when counting reversals.
const REVERSAL_DEADBAND: f32 = 0.05;
/// Settle time (s after the final hold starts) above which the response is "SLOW".
const SLOW_SETTLE_S: f32 = 1.5;

const PI: f32 = std::f32::consts::PI;

// ---------------------------------------------------------------------------
// Whole-body (hip-pin) mode constants
// ---------------------------------------------------------------------------

/// Gravity magnitude the tuning app uses (100 px = 1 m ⇒ 980 px/s²), Vello frame
/// (x right, y down ⇒ down is +y).
const GRAVITY_ON: Vec2 = Vec2::new(0.0, 980.0);
/// Bone-length tolerance for the pinned `P1/P2/P3` frame (and, when the legs are
/// driven, both legs' thigh + calf).
const PIN_RIGID_TOL: f32 = 0.02;
/// Foot-tracking tail window (s) + wobble thresholds (matches the retired
/// `headless_pin` leg verdict).
const LEG_TAIL_S: f32 = 1.0;
const LEG_RMS_MAX: f32 = 6.0;
const LEG_REV_MAX: u32 = 6;
const LEG_RATE_DEADBAND: f32 = 20.0;
/// Held-key mask bits written into the events sidecar (`keys` column).
const KEY_RIGHT: u8 = 1;
const KEY_LEFT: u8 = 2;
const KEY_UP: u8 = 4;
/// The physical keys `default_input_map()` binds for (Right, Left, Up). The probe
/// entity uses `default_input_map()` (L/J/I), **not** WASD.
const KEYS: [KeyCode; 3] = [KeyCode::KeyL, KeyCode::KeyJ, KeyCode::KeyI];

// ---------------------------------------------------------------------------
// Command script (serde JSON)
// ---------------------------------------------------------------------------

/// The four drivable limbs (also the JSON `limb` tag).
#[derive(Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[serde(rename_all = "snake_case")]
enum LimbId {
    RightArm,
    LeftArm,
    RightLeg,
    LeftLeg,
}

impl LimbId {
    fn idx(self) -> usize {
        match self {
            LimbId::RightArm => 0,
            LimbId::LeftArm => 1,
            LimbId::RightLeg => 2,
            LimbId::LeftLeg => 3,
        }
    }

    fn which(self) -> WhichLimb {
        match self {
            LimbId::RightArm => WhichLimb::Right,
            LimbId::LeftArm => WhichLimb::Left,
            LimbId::RightLeg => WhichLimb::RightLeg,
            LimbId::LeftLeg => WhichLimb::LeftLeg,
        }
    }

    fn from_env() -> Self {
        match std::env::var("LIMB").ok().as_deref() {
            Some("left_arm") => LimbId::LeftArm,
            Some("right_leg") => LimbId::RightLeg,
            Some("left_leg") => LimbId::LeftLeg,
            _ => LimbId::RightArm,
        }
    }

    fn label(self) -> &'static str {
        match self {
            LimbId::RightArm => "right_arm",
            LimbId::LeftArm => "left_arm",
            LimbId::RightLeg => "right_leg",
            LimbId::LeftLeg => "left_leg",
        }
    }
}

/// Position-IK reach: drive `limb` end to `to` (world Bevy y-up) or, when `to`
/// is omitted, to `anchor + normalize(dir) * dist`.
#[derive(Deserialize, Clone)]
struct ReachArgs {
    limb: LimbId,
    #[serde(default)]
    to: Option<[f32; 2]>,
    #[serde(default)]
    dir: Option<[f32; 2]>,
    #[serde(default)]
    dist: Option<f32>,
    #[serde(default)]
    kp: Option<f32>,
    #[serde(default)]
    kd: Option<f32>,
    dur: f32,
}

/// Direction-IK aim: aim `limb` end along `dir` at `dist` from the pivot.
#[derive(Deserialize, Clone)]
struct AimArgs {
    limb: LimbId,
    dir: [f32; 2],
    #[serde(default)]
    dist: Option<f32>,
    #[serde(default)]
    kp: Option<f32>,
    #[serde(default)]
    kd: Option<f32>,
    #[serde(default)]
    weapon_offset_y: Option<f32>,
    dur: f32,
}

/// Persistently set a limb's gains without moving it.
#[derive(Deserialize, Clone)]
struct SetArgs {
    limb: LimbId,
    #[serde(default)]
    kp: Option<f32>,
    #[serde(default)]
    kd: Option<f32>,
}

/// Emit a limb constraint reset.
#[derive(Deserialize, Clone)]
struct ResetArgs {
    limb: LimbId,
}

/// Hold the current command for `dur` seconds.
#[derive(Deserialize, Clone)]
struct WaitArgs {
    dur: f32,
}

/// Whole-body walk: hold a horizontal body input for `dur` seconds. Only `dir[0]`
/// (the x sign) is used — `+x` walks right, `-x` walks left — matching the
/// resolver's `input_dir.x`.
#[derive(Deserialize, Clone)]
struct MoveArgs {
    dir: [f32; 2],
    dur: f32,
}

#[derive(Deserialize, Clone)]
struct JumpArgs {
    #[serde(default = "default_jump_dur")]
    dur: f32,
    /// Optional held movement during the jump, matching how the app is actually
    /// driven (move key held while jump is tapped). `None` (or `[0,0]`) releases
    /// the move key for the jump duration — the legacy behaviour.
    #[serde(default)]
    dir: Option<[f32; 2]>,
}

fn default_jump_dur() -> f32 {
    0.8
}

/// One script command. Externally tagged: `{ "reach": { .. } }`.
#[derive(Deserialize, Clone)]
#[serde(rename_all = "snake_case")]
enum Command {
    Reach(ReachArgs),
    Aim(AimArgs),
    Set(SetArgs),
    Reset(ResetArgs),
    Wait(WaitArgs),
    Move(MoveArgs),
    Jump(JumpArgs),
}

impl Command {
    /// Kind label for the CSV `kind` column.
    fn kind(&self) -> &'static str {
        match self {
            Command::Reach(_) => "reach",
            Command::Aim(_) => "aim",
            Command::Set(_) => "set",
            Command::Reset(_) => "reset",
            Command::Wait(_) => "wait",
            Command::Move(_) => "move",
            Command::Jump(_) => "jump",
        }
    }

    /// How long the playhead dwells on this command.
    fn dur(&self) -> f32 {
        match self {
            Command::Reach(a) => a.dur,
            Command::Aim(a) => a.dur,
            Command::Wait(a) => a.dur,
            Command::Move(a) => a.dur,
            Command::Jump(a) => a.dur,
            // Instant: applied for exactly one tick.
            Command::Set(_) | Command::Reset(_) => 0.0,
        }
    }
}

#[derive(Deserialize, Clone)]
struct ScriptFile {
    script: Vec<Command>,
}

/// Built-in scripts used when `SCRIPT` is not given: a big step to one extreme,
/// a settle, a big step back through DC, then a settle — the minimum that exposes
/// waggle for either an arm (aim) or a leg (reach).
fn default_script(focus: LimbId) -> Vec<Command> {
    match focus {
        LimbId::RightArm | LimbId::LeftArm => vec![
            Command::Aim(AimArgs {
                limb: focus,
                dir: [1.0, 0.0],
                dist: None,
                kp: None,
                kd: None,
                weapon_offset_y: Some(0.0),
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.6 }),
            Command::Aim(AimArgs {
                limb: focus,
                dir: [-1.0, 0.0],
                dist: None,
                kp: None,
                kd: None,
                weapon_offset_y: Some(0.0),
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.9 }),
        ],
        LimbId::RightLeg | LimbId::LeftLeg => vec![
            Command::Reach(ReachArgs {
                limb: focus,
                to: None,
                dir: Some([1.0, -0.1]),
                dist: None,
                kp: None,
                kd: None,
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.6 }),
            Command::Reach(ReachArgs {
                limb: focus,
                to: None,
                dir: Some([-1.0, -0.1]),
                dist: None,
                kp: None,
                kd: None,
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.9 }),
        ],
    }
}

/// Load the script: `SCRIPT=path.json` if set, else the built-in default.
fn load_script(focus: LimbId) -> (Vec<Command>, String) {
    match std::env::var("SCRIPT") {
        Ok(path) if !path.trim().is_empty() => match std::fs::read_to_string(&path) {
            Ok(text) => match serde_json::from_str::<ScriptFile>(&text) {
                Ok(sf) if !sf.script.is_empty() => (sf.script, format!("file:{path}")),
                Ok(_) => {
                    error!("SCRIPT '{path}' parsed but is empty; using built-in default");
                    (default_script(focus), "default(empty-file)".into())
                }
                Err(e) => {
                    error!("SCRIPT '{path}' failed to parse ({e}); using built-in default");
                    (default_script(focus), "default(parse-error)".into())
                }
            },
            Err(e) => {
                error!("SCRIPT '{path}' unreadable ({e}); using built-in default");
                (default_script(focus), "default(unreadable)".into())
            }
        },
        _ => (default_script(focus), "default".into()),
    }
}

// ---------------------------------------------------------------------------
// Sample / report
// ---------------------------------------------------------------------------

/// One telemetry sample (Bevy y-up frame) — the minimum the classifier reads.
#[derive(Clone, Copy)]
struct Sample {
    tau: f32,
    /// Script action index this sample belongs to.
    act: usize,
    theta: f32, // end-bone angle (rad), Bevy frame
    omega: f32, // end-bone angular velocity (rad/s)
}

/// The classified result of one `(Kp,Kd)` cell — the deliverable readout.
struct CellReport {
    cell: usize,
    kp: f32,
    kd: f32,
    verdict: String,
    swing_deg: f32,
    rms_deg: f32,
    catchup_deg: f32,
    reversals: u32,
    settle_s: Option<f32>,
    max_omega: f32,
    inv_ok: bool,
    inv_detail: String,
    hash: u64,
}

#[derive(Resource)]
struct Probe {
    character: Option<Entity>,
    /// Monotonic fixed-tick counter (the harness clock).
    tick: u64,
    spawn_tick: Option<u64>,
    ever_ready: bool,

    // --- rig (captured once, when the focus limb is first ready) ---
    focus: LimbId,
    ready: bool,
    ticks_ready: u64,
    /// Per-limb pivot anchor (world Bevy y-up) = the limb's `particles[1]`.
    anchors: [Vec2; 4],
    /// Per-limb characteristic length = upper + lower bone, in px.
    limb_len: [f32; 4],
    anchored: bool,
    /// Resolved `TARGET_DIST` override for `aim` (px; ≤0 = auto).
    target_dist: f32,

    // --- script ---
    script: Vec<Command>,
    script_src: String,
    action: usize,
    action_start_tick: u64,
    applied_action: Option<usize>,
    script_done: bool,

    // --- sweep ---
    grid: Vec<(f32, f32)>,
    cell: usize,

    // --- per-cell accumulation ---
    kp: f32,
    kd: f32,
    samples: Vec<Sample>,
    reports: Vec<CellReport>,
    done: bool,
    csv_path: String,
    sweep: bool,

    // --- whole-body (hip-pin + legs) mode ---
    /// True when the script drives the whole body (`move`/`jump` present, or
    /// `HIPPIN` set): the pin is attached, `drive_hip_pin` runs, and the
    /// track/rigidity/jump→walk telemetry is folded into the row.
    body: bool,
    /// Previous held key mask, for edge-detecting the scripted body input.
    held_prev: u8,
    /// Events sidecar rows (`t,event_id,label,keys,kind`).
    events: Vec<String>,
    next_event_id: u32,
    /// Cached `P3` particle entity (the realized hip) for the jump→walk trace.
    p3_entity: Option<Entity>,
    /// The jump→walk window `(begin, end)` in seconds, derived from the script
    /// (the `move` immediately after a `jump`); `None` disables the verdict.
    jw_window: Option<(f32, f32)>,
    jw_prev_cmd_vx: f32,
    jw_prev_real_vx: f32,
    jw_primed: bool,
    jw_cmd_step: f32,
    jw_cmd_step_t: f32,
    jw_real_step: f32,
    jw_real_step_t: f32,
    jw_lag: f32,
    jw_lag_t: f32,
    jw_phase: u8,
    nan_rows: u32,
    /// Per-leg realized foot lag vs the commanded IK target.
    leg_active: bool,
    max_leg_err: f32,
    leg_peak_t: f32,
    max_leg_err_lr: [f32; 2],
    max_leg_err_lr_t: [f32; 2],
    leg_t: Vec<f32>,
    leg_err: Vec<f32>,
    leg_err_lr: Vec<[f32; 2]>,
    /// Signed world-frame x offset `foot.x - target.x` per leg, so a peak can
    /// be classified as lag (foot behind target, negative) vs overshoot (foot
    /// ahead of target, positive) for the +x walking direction.
    leg_dx_lr: Vec<[f32; 2]>,
    /// Commanded leg-target velocity (bevy y-up) from the previous tick, one per
    /// lower-body particle in `[P3, P40, P41, PLLL, PRLL]` order, for the
    /// finite-difference target acceleration export.
    prev_tgt_vel: [Vec2; 5],
    /// Whether `prev_tgt_vel` holds a real previous tick (false on the first).
    tgt_primed: bool,
}

// ---------------------------------------------------------------------------
// Small math / env helpers
// ---------------------------------------------------------------------------

/// Engine Vello (y-down) → Bevy (y-up).
#[inline]
fn to_bevy(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

/// Bevy (y-up) → engine Vello (y-down). The inverse of [`to_bevy`]; used to
/// export the leg controllers' bevy y-up world targets in the same y-down frame
/// as the realized particle channels, so `t<name>x/y` can be subtracted from
/// `<name>x/y` directly.
#[inline]
fn to_vello(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

#[inline]
fn cross(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Signed angular velocity (rad/s) of a bone `arm` (far end − pivot) whose far
/// end moves with relative velocity `dvel`: `ω = cross(arm, dvel) / |arm|²`.
#[inline]
fn bone_ang_vel(arm: Vec2, dvel: Vec2) -> f32 {
    let l2 = arm.length_squared();
    if l2 > f32::EPSILON {
        cross(arm, dvel) / l2
    } else {
        0.0
    }
}

fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn env_flag(key: &str) -> bool {
    matches!(
        std::env::var(key).as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

fn parse_grid(spec: &str) -> Vec<(f32, f32)> {
    spec.split(',')
        .filter_map(|cell| {
            let mut it = cell.split(':');
            let kp: f32 = it.next()?.trim().parse().ok()?;
            let kd: f32 = it.next()?.trim().parse().ok()?;
            Some((kp, kd))
        })
        .take(MAX_CELLS)
        .collect()
}

/// Default sweep grid when `SWEEP=1` and no `GRID` is given.
fn default_grid() -> Vec<(f32, f32)> {
    let kps = [0.2, 0.35, 0.5, 0.65, 0.8];
    let kds = [0.0, 0.03, 0.065, 0.1, 0.14];
    let mut g = Vec::new();
    for kp in kps {
        for kd in kds {
            g.push((kp, kd));
        }
    }
    g
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    let focus = LimbId::from_env();
    let (script, script_src) = load_script(focus);

    // Whole-body mode: the script drives `move`/`jump` (or `HIPPIN` forces it).
    // Then the hip pin owns P1/P2/P3 + the legs and the body telemetry runs; a
    // pure-limb script keeps the limb gate byte-identical (no pin, no pin rows).
    let body = env_flag("HIPPIN")
        || script
            .iter()
            .any(|c| matches!(c, Command::Move(_) | Command::Jump(_)));
    let jw_window = if body { jumpwalk_window(&script) } else { None };

    // Sweep is a limb-PD axis; the whole-body mode runs a single pass.
    let sweep = env_flag("SWEEP") && !body;

    let csv_path = std::env::var("OUT_CSV")
        .unwrap_or_else(|_| "/home/ubuntu/bender/tmp/headless/limb_pd_log.csv".to_string());

    let grid: Vec<(f32, f32)> = match std::env::var("GRID") {
        Ok(g) if !g.trim().is_empty() => parse_grid(&g),
        _ if sweep => default_grid(),
        _ => vec![(env_f32("KP", 0.5), env_f32("KD", 0.065))],
    };

    // Seed the first cell's gains from the grid (also the single-cell default).
    let (kp0, kd0) = grid.first().copied().unwrap_or((0.5, 0.065));

    info!(
        "character harness: focus={} script={} ({} command(s)), {} cell(s), sweep={}, body={}",
        focus.label(),
        script_src,
        script.len(),
        grid.len(),
        sweep,
        body
    );

    let probe = Probe {
        character: None,
        tick: 0,
        spawn_tick: None,
        ever_ready: false,
        focus,
        ready: false,
        ticks_ready: 0,
        anchors: [Vec2::ZERO; 4],
        limb_len: [0.0; 4],
        anchored: false,
        target_dist: env_f32("TARGET_DIST", 0.0),
        script,
        script_src,
        action: 0,
        action_start_tick: 0,
        applied_action: None,
        script_done: false,
        kp: kp0,
        kd: kd0,
        grid,
        cell: 0,
        samples: Vec::new(),
        reports: Vec::new(),
        done: false,
        csv_path,
        sweep,
        body,
        held_prev: 0,
        events: Vec::new(),
        next_event_id: 0,
        p3_entity: None,
        jw_window,
        jw_prev_cmd_vx: 0.0,
        jw_prev_real_vx: 0.0,
        jw_primed: false,
        jw_cmd_step: 0.0,
        jw_cmd_step_t: 0.0,
        jw_real_step: 0.0,
        jw_real_step_t: 0.0,
        jw_lag: 0.0,
        jw_lag_t: 0.0,
        jw_phase: 0,
        nan_rows: 0,
        leg_active: false,
        max_leg_err: 0.0,
        leg_peak_t: 0.0,
        max_leg_err_lr: [0.0, 0.0],
        max_leg_err_lr_t: [0.0, 0.0],
        leg_t: Vec::new(),
        leg_err: Vec::new(),
        leg_err_lr: Vec::new(),
        leg_dx_lr: Vec::new(),
        prev_tgt_vel: [Vec2::ZERO; 5],
        tgt_primed: false,
    };

    let gravity = if body { gravity_from_env() } else { Vec2::ZERO };

    let mut app = App::default();
    // Colliders need `GlobalTransform` propagation for the broad-phase BVH.
    if !app.is_plugin_added::<TransformPlugin>() {
        app.add_plugins(TransformPlugin);
    }
    app.add_plugins(
        MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
            1.0 / 240.0,
        ))),
    )
    .add_plugins(bevy::log::LogPlugin::default())
    .add_plugins(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        file_path: asset_root().into(),
        ..default()
    })
    .add_plugins(bevy::input::InputPlugin)
    // Deterministic clock: one fixed step per frame (no wall-clock batching), so
    // replays are byte-identical instead of merely metric-stable.
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / FIXED_HZ,
    )))
    .add_plugins(VelloCharacterPlugin)
    .add_plugins(HeadlessTelemetryPlugin)
    // Physics resources (mirrors VelloCollisionResponsePlugin, minus GPU parts).
    // No colliders; gravity per `GRAVITY` in body mode, else zero (the limb case).
    // `SUBSTEPS=<n>` overrides the XPBD substep count (default 0 → 1). The two
    // angular constraints on the leg share the knee particle; with a single
    // Gauss-Seidel pass per frame they never converge to a mutually-consistent
    // bent rest (hip held tight by the pin, fight spills onto knee+foot), so
    // more substeps is the convergence lever for foot lift.
    .insert_resource(VelloConstraintWorld::new(gravity))
    .insert_resource(Time::<Fixed>::from_hz(FIXED_HZ))
    .insert_resource({
        let mut world = VelloCollisionWorld::default();
        if let Ok(s) = std::env::var("SUBSTEPS") {
            if let Ok(n) = s.trim().parse::<u32>() {
                world.substeps = n;
            }
        }
        world
    })
    .insert_resource(RemovedColliders::default())
    .insert_resource(CollisionEventBatch::default())
    // Ground gate for the pin's vertical authority; refreshed only when the
    // collision pipeline runs, otherwise it keeps its grounded default.
    .insert_resource(GroundSupport::default())
    // The spine drive must stay silent: in body mode the pin owns P1/P2/P3, in
    // limb mode the frame must be passive (absent resource = ON ⇒ insert false).
    .insert_resource(SpineDriveEnabled(false))
    .add_event::<ColliderExternalImpulseEvent>()
    .add_event::<CharacterPivotForceEvent>()
    .add_event::<CharacterPivotVelocityEvent>()
    .add_event::<CharacterPivotImpulseEvent>()
    .add_event::<CharacterAngularConstraintEvent>()
    .add_event::<CharacterFrameForceEvent>()
    .add_event::<VelloRayTraceCommand>()
    .add_event::<ResetLimbControlConstraintsEvent>()
    .insert_resource(probe);

    if body {
        let params = hip_pin_params_from_env();
        let substeps = std::env::var("SUBSTEPS")
            .ok()
            .and_then(|s| s.trim().parse::<u32>().ok())
            .unwrap_or(1)
            .max(1);
        info!("pin controller: delta velocity servo, substeps={substeps}");
        app.insert_resource(params);
    }

    // Real physics stepping, same order as VelloCollisionResponsePlugin minus the
    // GPU tail. `update_character_movement` is injected by `GameCharacterPlugin`
    // in `FixedUpdate` before `update_constraint_world`. In body mode the pin is
    // inserted before it so the leg IK reads the foot targets the pin just wrote.
    if body {
        if collision_enabled() {
            app.add_systems(FixedUpdate, update_ground_support.before(drive_hip_pin));
        }
        if std::env::var("HIT").is_ok() {
            app.add_systems(
                FixedUpdate,
                inject_test_hit.before(apply_explicit_impulse_on_connection_particle),
            );
        }
        app.add_systems(
            FixedUpdate,
            (
                make_collision_constraints,
                drive_hip_pin.before(update_character_movement),
                update_constraint_world,
                update_connection_particles,
                remove_soft_body,
                generate_soft_body_for_collider,
                generate_connection,
                apply_explicit_impulse_on_softbody,
                apply_explicit_impulse_on_connection_particle,
                update_collider_from_soft_body,
            )
                .chain(),
        );
    } else {
        app.add_systems(
            FixedUpdate,
            (
                make_collision_constraints,
                update_constraint_world,
                update_connection_particles,
                remove_soft_body,
                generate_soft_body_for_collider,
                generate_connection,
                apply_explicit_impulse_on_softbody,
                apply_explicit_impulse_on_connection_particle,
                update_collider_from_soft_body,
            )
                .chain(),
        );
    }

    app.add_systems(Startup, register_character_assets);
    if collision_enabled() {
        // Headless GPU: no window/surface, but a real wgpu device + vello
        // renderer, so the same GPU narrow phase as the visual app runs.
        app.add_systems(Startup, setup_headless_collision);
        app.add_systems(
            FixedUpdate,
            (spawn_collision_floor, run_broad_phase, run_gpu_collision)
                .chain()
                .after(update_collider_from_soft_body),
        );
        app.add_systems(FixedPostUpdate, log_collision_stats);
    }
    app.add_systems(Update, mark_assets_loaded);
    // Apply the current script command in `FixedPreUpdate`, i.e. before the
    // controller reads the target this same fixed step (FixedPreUpdate runs
    // before FixedUpdate in the FixedMain schedule).
    app.add_systems(FixedPreUpdate, drive_script);
    // In body mode, translate the active `move`/`jump` command into the real
    // `ButtonInput` — after raw input (which clears per-frame edges) and before
    // leafwing turns it into `ActionState`, so the pin reads it this same frame.
    if body {
        app.add_systems(
            PreUpdate,
            drive_body_keys
                .after(InputSystem)
                .before(InputManagerSystem::Update),
        );
    }
    // Sample after the whole fixed step (physics + controller), append the row via
    // the shared telemetry plugin, then finalize the cell. Gated so no stray row
    // lands in the gap between cells (the rig is briefly not "ready").
    app.add_systems(
        FixedPostUpdate,
        (limb_sample, capture_telemetry, limb_finalize)
            .chain()
            .run_if(telemetry_active),
    );
    app.run();
}

/// Headless collision is opt-in so the deterministic kinematic gates stay
/// byte-identical.
fn collision_enabled() -> bool {
    matches!(
        std::env::var("COLLISION").ok().as_deref(),
        Some("1") | Some("true") | Some("yes")
    )
}

/// Build a headless wgpu device + vello renderer for the GPU narrow phase.
/// No surface/window: the collision path only needs a device and queue.
fn setup_headless_collision(mut commands: Commands) {
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });
    let adapter = bevy::tasks::block_on(wgpu::util::initialize_adapter_from_env_or_default(
        &instance, None,
    ))
    .expect("headless collision: no wgpu adapter");
    let (device, queue) = bevy::tasks::block_on(adapter.request_device(
        &wgpu::DeviceDescriptor {
            label: Some("headless collision"),
            required_features: adapter.features(),
            required_limits: adapter.limits(),
            memory_hints: wgpu::MemoryHints::default(),
        },
        None,
    ))
    .expect("headless collision: no wgpu device");
    let renderer = bevy_vello::vello::Renderer::new(
        &device,
        &bevy_vello::vello::RendererOptions {
            surface_format: None,
            timestamp_period: queue.get_timestamp_period(),
            use_cpu: false,
        },
    )
    .expect("headless collision: vello renderer");
    commands.insert_resource(GpuCollisionRunner::new(
        Arc::new(Mutex::new(renderer)),
        device,
        queue,
    ));
    commands.insert_resource(VelloCollisionBroadPhase::default());
    info!("headless collision: GPU runner + CPU BVH enabled");
}

/// Spawn the static floor at the resolver's floor line once the pin is seeded.
/// `anchor_p3` is Vello y-down; `-anchor_p3.y` is the Bevy y of the hip, and
/// the resolver floor is `hip_height` below it.
fn spawn_collision_floor(
    mut commands: Commands,
    mut done: Local<bool>,
    pin_q: Query<&HipPin>,
    params: Option<Res<HipPinParams>>,
) {
    if *done {
        return;
    }
    let Some(params) = params else {
        return;
    };
    let Ok(pin) = pin_q.get_single() else {
        return;
    };
    if !pin.ready {
        return;
    }
    let hip_y = -pin.anchor_p3.y;
    // The pose-aware bridge preserves the *live* leg, so the floor must sit at
    // the live hip→foot distance, not the resolver's `hip_height` (the resolver
    // stands with a bent knee; the character's seed pose is straight).
    let stance_h = pin.stance_height().unwrap_or(params.config.hip_height);
    // `GROUND_DROP=<px>` lowers the floor below the seed foot line. With the
    // rigid pin the frame cannot follow, so the character floats; with the delta
    // pin gravity is real and the body must fall by the drop. This is the
    // gravity/free-fall gate.
    let floor_drop = env_f32("GROUND_DROP", 0.0);
    // The collider is a 40px-thick rect (local y -20..20). Its TOP surface must
    // be at the foot line, so the center is one half-thickness below it
    // (Bevy y-up: subtract; the earlier version put the feet 20px inside).
    const FLOOR_HALF_THICKNESS: f32 = 20.0;
    let floor_surface_y = hip_y - stance_h - floor_drop;
    let floor_y = floor_surface_y - FLOOR_HALF_THICKNESS;
    let rect = kurbo::Rect::new(-2400.0, -20.0, 2400.0, 20.0);
    let shape = path_to_ccw_quad_path(&rect.to_path(0.1));
    let transform = Transform::from_xyz(0.0, floor_y, 0.0);
    commands.spawn((
        VelloCollider::new(
            &shape,
            None,
            &rect,
            Vec2::ZERO,
            peniko::Brush::Solid(peniko::Color::TRANSPARENT),
            0.0,
            false,
            None,
            None,
            None,
            0,
            transform,
            VELLO_COLLISION_COOL_DOWN_TIME,
        ),
        transform,
        GlobalTransform::from(transform),
    ));
    *done = true;
    info!(
        "headless collision: floor center y={floor_y:.2}, surface y={floor_surface_y:.2} (hip y={hip_y:.2}, live_stance_h={stance_h:.2}, ground_drop={floor_drop:.2}, resolver_hip_height={:.2}, hip_drop={:.2})",
        params.config.hip_height, params.hip_drop
    );
}

fn log_collision_stats(
    time: Res<Time<Fixed>>,
    batch: Res<CollisionEventBatch>,
    colliders: Query<&VelloCollider>,
    mut ticks: Local<u64>,
) {
    *ticks += 1;
    if *ticks % FIXED_HZ as u64 != 0 {
        return;
    }
    let total = colliders.iter().count();
    let soft = colliders.iter().filter(|c| c.is_soft_body()).count();
    info!(
        "collision t={:.1}s colliders={total} soft={soft} contacts={}",
        time.elapsed_secs(),
        batch.entries.len()
    );
}

/// `HIT=<px/s>` injects a one-shot lateral velocity into the pinned frame at
/// `HIT_T` seconds (default 2.0), for the "heavy strike" gate. Uses
/// [`CharacterFrameForceEvent`] on the three frame particles; the solver applies
/// `pos += impulse * inv_mass * dt`, so an impulse of `dv/dt` produces `dv`.
fn inject_test_hit(
    time: Res<Time<Fixed>>,
    mut done: Local<bool>,
    pin_q: Query<Entity, With<HipPin>>,
    mut events: EventWriter<CharacterFrameForceEvent>,
) {
    if *done {
        return;
    }
    let dv = match std::env::var("HIT") {
        Ok(s) => s.trim().parse::<f32>().unwrap_or(0.0),
        Err(_) => return,
    };
    if dv.abs() < 1e-3 {
        return;
    }
    let t_hit = env_f32("HIT_T", 2.0);
    if time.elapsed_secs() < t_hit {
        return;
    }
    let Ok(character) = pin_q.get_single() else {
        return;
    };
    let dt = time.delta_secs().max(1e-6);
    let impulse = Vec2::new(dv / dt, 0.0);
    events.write(CharacterFrameForceEvent {
        character_entity: character,
        forces: vec![impulse; 3],
    });
    info!(
        "test hit: dv={dv:.0} px/s on P1/P2/P3 at t={:.2}",
        time.elapsed_secs()
    );
    *done = true;
}

fn register_character_assets(
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
    asset_server: Res<AssetServer>,
) {
    character_svg.push(
        asset_server.load(SVG_PATH),
        SvgCharacterAssetMetaData::default(),
        SVG_ASSET,
    );
    character_blueprint.push(
        asset_server.load(BP_PATH),
        BlueprintCharacterAssetMetaData::default(),
        BP_ASSET,
    );
}

fn mark_assets_loaded(
    mut cs: EventReader<AssetEvent<SvgCharacterAsset>>,
    mut cb: EventReader<AssetEvent<BlueprintCharacterAsset>>,
    mut svg: ResMut<SvgCharacterAssetManager>,
    mut bp: ResMut<BlueprintCharacterAssetManager>,
) {
    for e in cs.read() {
        if let AssetEvent::LoadedWithDependencies { id } = e {
            svg.mark_as_loaded(id);
        }
    }
    for e in cb.read() {
        if let AssetEvent::LoadedWithDependencies { id } = e {
            bp.mark_as_loaded(id);
        }
    }
}

/// The resolved intent of the current command — everything the dispatcher needs
/// to write, independent of which of the four controller components it lands on.
enum Intent {
    /// Drive `limb` to `target` in `mode` with `(kp, kd)`.
    Move {
        limb: LimbId,
        target: Vec2,
        mode: IkMode,
        kp: f32,
        kd: f32,
    },
    /// Persistently set gains only.
    Set {
        limb: LimbId,
        kp: Option<f32>,
        kd: Option<f32>,
    },
    /// Emit a constraint reset for `limb`.
    Reset { limb: LimbId },
    /// Do nothing this tick.
    Idle,
    /// Whole-body verb (`move`/`jump`): the hip pin is driven by
    /// [`drive_body_keys`] from the playhead, not by a limb controller here.
    Body,
}

/// Fold the current command + the probe's rig state into an [`Intent`].
fn resolve_intent(cmd: &Command, probe: &Probe) -> Intent {
    // Per-command gains override the cell/env default only in single-cell mode;
    // in sweep mode the cell's gains always win (that IS the experiment axis).
    let gains = |kp: Option<f32>, kd: Option<f32>| -> (f32, f32) {
        if probe.sweep {
            (probe.kp, probe.kd)
        } else {
            (kp.unwrap_or(probe.kp), kd.unwrap_or(probe.kd))
        }
    };

    match cmd {
        Command::Reach(a) => {
            let idx = a.limb.idx();
            let anchor = probe.anchors[idx];
            let len = probe.limb_len[idx].max(1.0);
            let target = match a.to {
                Some([x, y]) => Vec2::new(x, y),
                None => {
                    let dir = a
                        .dir
                        .map(|[x, y]| Vec2::new(x, y))
                        .unwrap_or(Vec2::NEG_Y)
                        .normalize_or(Vec2::NEG_Y);
                    anchor + dir * a.dist.unwrap_or(len)
                }
            };
            let (kp, kd) = gains(a.kp, a.kd);
            Intent::Move {
                limb: a.limb,
                target,
                mode: IkMode::Reach,
                kp,
                kd,
            }
        }
        Command::Aim(a) => {
            let idx = a.limb.idx();
            let anchor = probe.anchors[idx];
            let len = probe.limb_len[idx].max(1.0);
            // Auto distance: 3× the end bone (matches the arm harness heuristic),
            // unless `TARGET_DIST` or the command's `dist` says otherwise.
            let auto = 3.0 * len * 0.5;
            let dist = if probe.target_dist > 0.0 {
                probe.target_dist
            } else {
                a.dist.unwrap_or(auto.max(40.0))
            };
            let dir = Vec2::new(a.dir[0], a.dir[1]).normalize_or(Vec2::X);
            let target = anchor + dir * dist;
            let (kp, kd) = gains(a.kp, a.kd);
            Intent::Move {
                limb: a.limb,
                target,
                mode: IkMode::Aim {
                    weapon_offset_y: a.weapon_offset_y.unwrap_or(0.0),
                },
                kp,
                kd,
            }
        }
        Command::Set(a) => {
            let kp = if probe.sweep { Some(probe.kp) } else { a.kp };
            let kd = if probe.sweep { Some(probe.kd) } else { a.kd };
            Intent::Set {
                limb: a.limb,
                kp,
                kd,
            }
        }
        Command::Reset(a) => Intent::Reset { limb: a.limb },
        Command::Wait(_) => Intent::Idle,
        Command::Move(_) | Command::Jump(_) => Intent::Body,
    }
}

#[allow(clippy::too_many_arguments)]
fn drive_script(
    mut commands: Commands,
    mut probe: ResMut<Probe>,
    character_svg: Res<SvgCharacterAssetManager>,
    character_blueprint: Res<BlueprintCharacterAssetManager>,
    mut right_arm_q: Query<&mut RightArmController>,
    mut left_arm_q: Query<&mut LeftArmController>,
    mut right_leg_q: Query<&mut RightLegController>,
    mut left_leg_q: Query<&mut LeftLegController>,
    root_q: Query<&VelloCharacterPhysicsRoot>,
    particle_q: Query<&VelloParticle>,
    mut reset_ev: EventWriter<ResetLimbControlConstraintsEvent>,
    mut exit: EventWriter<AppExit>,
    pin_params: Option<Res<HipPinParams>>,
) {
    probe.tick += 1;
    let tick = probe.tick;

    // ---- Spawn V10 once both assets are parsed. ----
    if probe.character.is_none() {
        if character_svg.all_loaded() && character_blueprint.all_loaded() {
            let mut ent = commands.spawn((
                VelloSceneBundle {
                    transform: Transform {
                        translation: Vec3::new(0.0, 0.0, 100.0),
                        scale: Vec3::new(0.5, 0.5, 1.0),
                        ..Default::default()
                    },
                    ..Default::default()
                },
                CharacterRoot {
                    svg_asset_id: SVG_ASSET.to_owned(),
                    blueprint_asset_id: BP_ASSET.to_owned(),
                    collision_group: 1,
                },
                CharacterController::default(),
            ));
            if probe.body {
                // The pin needs the real input path: `PlayerMarker` + the default
                // action map + the live `ActionState` the pin reads, plus the
                // `HipPin` itself (seeded lazily on the first ready tick). The
                // resolver config comes from the env-built `HipPinParams`
                // resource (TARGET_SPEED/LEGS/...), so the spawn and the
                // resource can't drift apart.
                let hip_cfg = match &pin_params {
                    Some(p) => p.config.clone(),
                    None => HipPinParams::default().config,
                };
                ent.insert((
                    PlayerMarker,
                    default_input_map(),
                    ActionState::<PlayerAction>::default(),
                    HipPin::new(&hip_cfg),
                ));
            }
            let entity = ent.id();
            probe.character = Some(entity);
            probe.spawn_tick = Some(tick);
            info!("V10 spawned, harness clock starts (body={})", probe.body);
        }
        return;
    }

    let character = probe.character.unwrap();

    // ---- Bail out if the rig never comes up (never silently hang). ----
    if !probe.ready && !probe.ever_ready {
        if let Some(st) = probe.spawn_tick {
            if tick.saturating_sub(st) > (FIXED_HZ as u64) * 10 {
                error!("rig never became ready after 10 sim-seconds; aborting");
                probe.done = true;
                exit.write(AppExit::error());
            }
        }
    }
    if probe.done {
        return;
    }

    // ---- Wait for the physics root (same gate as `update_character_movement`). ----
    let Ok(root) = root_q.get(character) else {
        return;
    };
    if root.initial_frame_coordinates.is_none() {
        return;
    }

    // ---- Capture the rig once: every limb's pivot anchor + characteristic length. ----
    if !probe.ready {
        // The focus limb must exist; otherwise there is nothing to measure.
        let focus_ok = match probe.focus {
            LimbId::RightArm => right_arm_q.get(character).is_ok(),
            LimbId::LeftArm => left_arm_q.get(character).is_ok(),
            LimbId::RightLeg => right_leg_q.get(character).is_ok(),
            LimbId::LeftLeg => left_leg_q.get(character).is_ok(),
        };
        if !focus_ok {
            return;
        }

        probe.anchored = seed_anchors(
            &mut probe,
            character,
            &right_arm_q,
            &left_arm_q,
            &right_leg_q,
            &left_leg_q,
            &particle_q,
        );
        if !probe.anchored {
            return;
        }

        // ---- A/B levers for the leg-controller tracking bandwidth (env-gated,
        // applied once at ready; body mode never calls `apply_to_rleg`/`apply_to_lleg`
        // so the legs otherwise hold `LimbConfig::default()`). Both target the
        // leg-chain divergence now that the hip frame is proven tight (~0.7px).
        //   `LEG_NO_CHASE=1`   command the resolver's rest angle directly — no
        //                      first-order chase (`rest_chase_alpha=1.0`) and no
        //                      `max_angle_rate` clamp (huge) — the XPBD ζ-damped
        //                      spring is the only smoothing left.
        //   `LEG_NO_SCHEDULE=1` disable the ω-softening compliance schedule
        //                      (`soft_scale=1.0`) so stiffness holds during swing.
        //   `LEG_ZETA=<ζ>`     override the solver damping ratio (default 1.0 —
        //                      proven over-damping in the arm ζ-sweep; the large
        //                      `gamma` starves the position correction). ζ=0.2 was
        //                      the arm sweet spot.
        //   `LEG_SIGMA=<σ>`    override the velocity-projection strength
        //                      (default 1.0 = full projection). σ=0 disables the
        //                      projection entirely — recovers the pre-projection
        //                      positional-solve-only behaviour (37px floor), σ<1
        //                      weakens it (soft velocity constraint).
        //   `LEG_COMPLIANCE=<k>` override the angular stiffness (default
        //                      5e-8 at 30px ref length). Smaller = stiffer, larger
        //                      = softer; this is the Kp of the XPBD angular
        //                      spring (ζ damping scales with it automatically).
        // Baseline = none set (default chase + schedule + ζ=1.0 + σ=1.0 + k=5e-8).
        //   `LEG_C_DOT_SCALE=<s>` override the `angular_velocity` gain
        //                      (default 1.0). The game layer now emits rad/s
        //                      (substep-count-independent) and the solver
        //                      converts internally; this is a pure authority
        //                      knob on the commanded joint rate. Root-cause probe.
        if probe.body
            && (env_flag("LEG_NO_CHASE")
                || env_flag("LEG_NO_SCHEDULE")
                || std::env::var("LEG_ZETA")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty())
                || std::env::var("LEG_SIGMA")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty())
                || std::env::var("LEG_COMPLIANCE")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty())
                || std::env::var("LEG_C_DOT_SCALE")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty()))
        {
            let no_chase = env_flag("LEG_NO_CHASE");
            let no_schedule = env_flag("LEG_NO_SCHEDULE");
            let zeta_opt = std::env::var("LEG_ZETA")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let sigma_opt = std::env::var("LEG_SIGMA")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let compliance_opt = std::env::var("LEG_COMPLIANCE")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let c_dot_scale_opt = std::env::var("LEG_C_DOT_SCALE")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let apply = |cfg: &mut LimbConfig| {
                if no_chase {
                    cfg.rest_chase_alpha = 1.0;
                    cfg.max_angle_rate = 1e6;
                }
                if no_schedule {
                    cfg.soft_scale = 1.0;
                }
                if let Some(z) = zeta_opt {
                    cfg.zeta = z;
                }
                if let Some(s) = sigma_opt {
                    cfg.sigma = s;
                }
                if let Some(k) = compliance_opt {
                    cfg.angular_compliance = k;
                }
                if let Some(s) = c_dot_scale_opt {
                    cfg.c_dot_scale = s;
                }
            };
            if let Ok(mut c) = right_leg_q.get_mut(character) {
                apply(&mut c.config);
            }
            if let Ok(mut c) = left_leg_q.get_mut(character) {
                apply(&mut c.config);
            }
            info!(
                "body A/B leg levers: no_chase={} no_schedule={} zeta={:?} sigma={:?} compliance={:?} c_dot_scale={:?}",
                no_chase, no_schedule, zeta_opt, sigma_opt, compliance_opt, c_dot_scale_opt
            );
        }

        let first_ready = !probe.ever_ready;
        probe.ticks_ready = tick;
        probe.ready = true;
        probe.ever_ready = true;
        probe.action = 0;
        probe.action_start_tick = tick;
        probe.applied_action = None;
        probe.script_done = false;
        probe.samples.clear();
        if first_ready {
            commands.insert_resource(ParticleTelemetryConfig::v10(character));
        }
        info!(
            "cell {} (kp={:.3}, kd={:.3}): rig ready, focus={} anchor={:?}",
            probe.cell,
            probe.kp,
            probe.kd,
            probe.focus.label(),
            probe.anchors[probe.focus.idx()],
        );
    }

    if probe.script.is_empty() {
        error!("empty script; aborting");
        probe.done = true;
        exit.write(AppExit::error());
        return;
    }

    // ---- Playhead: one entry per command, advanced when its `dur` elapses. ----
    let cur = probe.action.min(probe.script.len() - 1);
    let cur_dur = probe.script[cur].dur();
    let elapsed = (tick.saturating_sub(probe.action_start_tick)) as f32 / FIXED_HZ as f32;

    // Apply the current command (idempotent for continuous verbs; once for
    // one-shot verbs via `applied_action`).
    let cmd = probe.script[cur].clone();
    let first_touch = probe.applied_action != Some(cur);
    let intent = resolve_intent(&cmd, &probe);
    match intent {
        Intent::Move {
            limb,
            target,
            mode,
            kp,
            kd,
        } => match limb {
            LimbId::RightArm => apply_to_arm(&mut right_arm_q, character, target, mode, kp, kd),
            LimbId::LeftArm => apply_to_larm(&mut left_arm_q, character, target, mode, kp, kd),
            LimbId::RightLeg => apply_to_rleg(&mut right_leg_q, character, target, mode, kp, kd),
            LimbId::LeftLeg => apply_to_lleg(&mut left_leg_q, character, target, mode, kp, kd),
        },
        Intent::Set { limb, kp, kd } => {
            if first_touch {
                apply_set(
                    &mut right_arm_q,
                    &mut left_arm_q,
                    &mut right_leg_q,
                    &mut left_leg_q,
                    character,
                    limb,
                    kp,
                    kd,
                );
            }
        }
        Intent::Reset { limb } => {
            if first_touch {
                reset_ev.write(ResetLimbControlConstraintsEvent {
                    limb: limb.which(),
                    character,
                });
            }
        }
        // `wait` (Idle) does nothing; `move`/`jump` (Body) are driven by the
        // playhead in `drive_body_keys`, so no limb write lands here.
        Intent::Idle | Intent::Body => {}
    }
    if first_touch {
        probe.applied_action = Some(cur);
    }

    // Advance the playhead (after applying, so a `dur == 0` verb still lands once).
    if elapsed >= cur_dur {
        if cur + 1 < probe.script.len() {
            probe.action = cur + 1;
            probe.action_start_tick = tick;
            probe.applied_action = None;
        } else {
            probe.script_done = true;
            // The telemetry system owns finalization (so it can grab the last
            // sample); the drive simply holds the last target and waits.
        }
    }
}

/// Seed each limb's pivot anchor (world Bevy y-up) + characteristic length,
/// resolved from the controller components. Returns false if a limb's particles
/// are not yet resolvable (retry next tick). Missing limbs keep zeroed defaults.
#[allow(clippy::too_many_arguments)]
fn seed_anchors(
    probe: &mut Probe,
    character: Entity,
    right_arm_q: &Query<&mut RightArmController>,
    left_arm_q: &Query<&mut LeftArmController>,
    right_leg_q: &Query<&mut RightLegController>,
    left_leg_q: &Query<&mut LeftLegController>,
    particle_q: &Query<&VelloParticle>,
) -> bool {
    let mut ok = true;
    let mut seed = |idx: usize, parts: &[Entity; 4]| {
        let (Ok(p1), Ok(p2), Ok(p3)) = (
            particle_q.get(parts[1]),
            particle_q.get(parts[2]),
            particle_q.get(parts[3]),
        ) else {
            ok = false;
            return;
        };
        let pivot = to_bevy(p1.particle.pos);
        let mid = to_bevy(p2.particle.pos);
        let end = to_bevy(p3.particle.pos);
        probe.anchors[idx] = pivot;
        probe.limb_len[idx] = (mid - pivot).length() + (end - mid).length();
    };
    if let Ok(c) = right_arm_q.get(character) {
        seed(0, &c.particles);
    }
    if let Ok(c) = left_arm_q.get(character) {
        seed(1, &c.particles);
    }
    if let Ok(c) = right_leg_q.get(character) {
        seed(2, &c.particles);
    }
    if let Ok(c) = left_leg_q.get(character) {
        seed(3, &c.particles);
    }
    ok
}

#[inline]
fn apply_to_arm(
    q: &mut Query<&mut RightArmController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[inline]
fn apply_to_larm(
    q: &mut Query<&mut LeftArmController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[inline]
fn apply_to_rleg(
    q: &mut Query<&mut RightLegController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[inline]
fn apply_to_lleg(
    q: &mut Query<&mut LeftLegController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[allow(clippy::too_many_arguments)]
fn apply_set(
    right_arm_q: &mut Query<&mut RightArmController>,
    left_arm_q: &mut Query<&mut LeftArmController>,
    right_leg_q: &mut Query<&mut RightLegController>,
    left_leg_q: &mut Query<&mut LeftLegController>,
    character: Entity,
    limb: LimbId,
    kp: Option<f32>,
    kd: Option<f32>,
) {
    let tweak = |config: &mut LimbConfig| {
        if let Some(kp) = kp {
            config.rest_chase_alpha = kp;
        }
        if let Some(kd) = kd {
            config.zeta = kd;
        }
    };
    match limb {
        LimbId::RightArm => {
            if let Ok(mut c) = right_arm_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
        LimbId::LeftArm => {
            if let Ok(mut c) = left_arm_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
        LimbId::RightLeg => {
            if let Ok(mut c) = right_leg_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
        LimbId::LeftLeg => {
            if let Ok(mut c) = left_leg_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
    }
}

/// Whether telemetry should collect this tick: only while the rig is up and a
/// cell is in progress, so no row lands in the brief not-ready gap between cells.
fn telemetry_active(probe: Res<Probe>) -> bool {
    probe.character.is_some() && probe.ready && !probe.done
}

/// The focus limb's end-bone (particles[2] → particles[3]) angle + rate, in the
/// Bevy frame. Returns `None` if the focus limb's controller is absent.
fn focus_bone(
    probe: &Probe,
    character: Entity,
    right_arm_q: &Query<&RightArmController>,
    left_arm_q: &Query<&LeftArmController>,
    right_leg_q: &Query<&RightLegController>,
    left_leg_q: &Query<&LeftLegController>,
    particle_q: &Query<&VelloParticle>,
) -> Option<(f32, f32)> {
    let parts: [Entity; 4] = match probe.focus {
        LimbId::RightArm => right_arm_q.get(character).ok()?.particles,
        LimbId::LeftArm => left_arm_q.get(character).ok()?.particles,
        LimbId::RightLeg => right_leg_q.get(character).ok()?.particles,
        LimbId::LeftLeg => left_leg_q.get(character).ok()?.particles,
    };
    let mid = particle_q.get(parts[2]).ok()?;
    let end = particle_q.get(parts[3]).ok()?;
    let arm = to_bevy(end.particle.pos) - to_bevy(mid.particle.pos);
    let dvel = to_bevy(end.particle.velocity) - to_bevy(mid.particle.velocity);
    let theta = arm.y.atan2(arm.x);
    let omega = bone_ang_vel(arm, dvel);
    Some((theta, omega))
}

/// Collect one sample: the harness scalars (into [`TelemetryRow`], in a fixed
/// key order) and the theta/omega the classifier needs (into `Probe::samples`).
fn limb_sample(
    mut probe: ResMut<Probe>,
    mut row: ResMut<TelemetryRow>,
    params: Option<Res<HipPinParams>>,
    pin_q: Query<&HipPin>,
    right_arm_q: Query<&RightArmController>,
    left_arm_q: Query<&LeftArmController>,
    right_leg_q: Query<&RightLegController>,
    left_leg_q: Query<&LeftLegController>,
    particle_q: Query<&VelloParticle>,
) {
    let Some(character) = probe.character else {
        return;
    };
    if probe.done || !probe.ready {
        return;
    }
    let Some((theta, omega)) = focus_bone(
        &probe,
        character,
        &right_arm_q,
        &left_arm_q,
        &right_leg_q,
        &left_leg_q,
        &particle_q,
    ) else {
        return;
    };

    let tick = probe.tick;
    let tau = (tick.saturating_sub(probe.ticks_ready)) as f32 / FIXED_HZ as f32;
    let act = probe.action.min(probe.script.len().saturating_sub(1));
    let kind = probe.script.get(act).map(|c| c.kind()).unwrap_or("none");

    let (kp, kd) = (probe.kp, probe.kd);
    probe.samples.push(Sample {
        tau,
        act,
        theta,
        omega,
    });

    row.clear();
    row.set("tau", format!("{:.4}", tau));
    row.set("t", format!("{:.4}", tick as f32 / FIXED_HZ as f32));
    row.set("act", act.to_string());
    row.set("kind", kind.to_string());
    row.set("kp", format!("{:.3}", kp));
    row.set("kd", format!("{:.4}", kd));
    row.set("theta_deg", format!("{:.3}", theta.to_degrees()));
    row.set("omega", format!("{:.3}", omega));

    // --- Whole-body (hip-pin) telemetry, folded into the row as extra scalars.
    // The verbatim commanded hip vx (`cmd_vx`) + vertical phase (`vphase`), the
    // jump→walk handoff accumulators, and the per-leg foot lag vs the commanded
    // IK target. All body timing uses the **script-relative** `tau` (the playhead
    // resets when the rig is ready), so it lines up with `jumpwalk_window`'s
    // cumulative `dur` — not the spawn-relative `t` column the limb gate uses.
    if probe.body {
        let tb = tau;
        // The pin seeds lazily; degrade to zeros rather than dropping the row, so
        // the frozen header (scalars set in this exact order on the first capture)
        // always matches the values.
        let (cmd_vx, vphase) = match pin_q.get(character) {
            Ok(pin) => (pin.state.hip_vel.x, vphase_code(pin.state.vs.phase)),
            Err(_) => (0.0, 0),
        };
        row.set("cmd_vx", format!("{cmd_vx:.2}"));
        row.set("vphase", vphase.to_string());

        // --- Loop 3 (attitude/tilt) telemetry, from the same pin. The resolver's
        // persistent `theta`/`omega` (rad, rad/s) plus the per-tick control input
        // `u` (rad/s²) and measured hip acceleration (px/s²) latched by
        // `drive_hip_pin`. Keyed `body_*`/`att_*` so they never collide with the
        // limb classifier's `theta_deg`/`omega` columns above. Order is fixed so
        // the frozen header stays stable.
        let (body_theta, body_omega, att_torque, hip_accel_x) = match pin_q.get(character) {
            Ok(pin) => (
                pin.state.attitude.theta,
                pin.state.attitude.omega,
                pin.last_attitude_torque,
                pin.last_hip_accel_x,
            ),
            Err(_) => (0.0, 0.0, 0.0, 0.0),
        };
        row.set("body_theta", format!("{body_theta:.4}"));
        row.set("body_omega", format!("{body_omega:.4}"));
        row.set("att_torque", format!("{att_torque:.2}"));
        row.set("hip_accel_x", format!("{hip_accel_x:.2}"));

        // Realized hip vx = the pinned `P3` (each leg controller's `pivot`,
        // `particles[1]`). Cached on the first tick it resolves.
        if probe.p3_entity.is_none() {
            probe.p3_entity = left_leg_q.get(character).ok().map(|l| l.particles[1]);
        }
        let p3_vx = match probe.p3_entity.and_then(|e| particle_q.get(e).ok()) {
            Some(p) => to_bevy(p.particle.velocity).x,
            None => cmd_vx,
        };

        // Jump→walk handoff: the *commanded* hip vx (Raibert resolver) vs the
        // *realized* hip vx (the pinned `P3`). A jagged `real_step` over a smooth
        // `cmd_step` ⇒ the physics/pin layer injects a jolt the controller never
        // asked for; a persistent `lag` ⇒ the frame is not tracking the command.
        if let Some((b, e)) = probe.jw_window {
            if (b..e).contains(&tb) {
                if !probe.jw_primed {
                    probe.jw_phase = vphase;
                    probe.jw_primed = true;
                } else {
                    let s = (cmd_vx - probe.jw_prev_cmd_vx).abs();
                    if s > probe.jw_cmd_step {
                        probe.jw_cmd_step = s;
                        probe.jw_cmd_step_t = tb;
                    }
                    let r = (p3_vx - probe.jw_prev_real_vx).abs();
                    if r > probe.jw_real_step {
                        probe.jw_real_step = r;
                        probe.jw_real_step_t = tb;
                    }
                }
                let lag = (p3_vx - cmd_vx).abs();
                if lag > probe.jw_lag {
                    probe.jw_lag = lag;
                    probe.jw_lag_t = tb;
                }
                probe.jw_prev_cmd_vx = cmd_vx;
                probe.jw_prev_real_vx = p3_vx;
            }
        }

        // Per-leg foot lag: the realized foot (`PRLL`/`PLLL`) vs the commanded IK
        // `target` the pin wrote into each leg controller. A large value = a foot
        // lagging far from (or fighting) its commanded pose.
        let legs_on = params
            .as_deref()
            .map(|p| p.hip_drives_legs)
            .unwrap_or(false);
        if legs_on {
            if let (Ok(left_leg), Ok(right_leg)) =
                (left_leg_q.get(character), right_leg_q.get(character))
            {
                probe.leg_active = true;
                let mut emax_leg = 0.0f32;
                let mut lr = [0.0f32, 0.0f32];
                for (li, target) in [left_leg.target, right_leg.target].into_iter().enumerate() {
                    let foot_ent = if li == 0 {
                        left_leg.particles[3]
                    } else {
                        right_leg.particles[3]
                    };
                    if let Ok(p) = particle_q.get(foot_ent) {
                        let err = (to_bevy(p.particle.pos) - target).length();
                        lr[li] = err;
                        if err > probe.max_leg_err_lr[li] {
                            probe.max_leg_err_lr[li] = err;
                            probe.max_leg_err_lr_t[li] = tb;
                        }
                        emax_leg = emax_leg.max(err);
                    }
                }
                if emax_leg > probe.max_leg_err {
                    probe.max_leg_err = emax_leg;
                    probe.leg_peak_t = tb;
                }
                let mut dx_lr = [0.0f32, 0.0f32];
                for (li, target) in [left_leg.target, right_leg.target].into_iter().enumerate() {
                    let foot_ent = if li == 0 {
                        left_leg.particles[3]
                    } else {
                        right_leg.particles[3]
                    };
                    if let Ok(p) = particle_q.get(foot_ent) {
                        // Signed world-frame x offset: foot.x - target.x. Negative
                        // = lagging behind the +x command, positive = overshoot.
                        // Both are bevy y-up world, so the x is directly comparable.
                        dx_lr[li] = to_bevy(p.particle.pos).x - target.x;
                    }
                }
                probe.leg_t.push(tb);
                probe.leg_err.push(emax_leg);
                probe.leg_err_lr.push(lr);
                probe.leg_dx_lr.push(dx_lr);

                // --- Commanded lower-body target state, exported in the same
                // vello y-down frame as the realized particle channels (which
                // `capture_telemetry` emits raw), so `<name>x - t<name>x` is
                // directly the tracking error: negative x = lagging behind the
                // +x command, positive x = overshoot. Velocity is the
                // resolver/controller command; acceleration is a per-tick finite
                // difference (fixed cadence `1/FIXED_HZ`). Order matches the 5
                // lower-body particles: P3 (hip), P40 (left knee), P41 (right
                // knee), PLLL (left foot), PRLL (right foot). NOTE: the pin
                // currently writes only the foot `target` + the joint
                // velocities; `hip_target`/`knee_target` are frozen at spawn,
                // so a stationary `tP3`/`tP40`/`tP41` is expected and is itself
                // diagnostic.
                let lb_tgt = [
                    ("tP3", left_leg.hip_target, left_leg.hip_vel),
                    ("tP40", left_leg.knee_target, left_leg.knee_vel),
                    ("tP41", right_leg.knee_target, right_leg.knee_vel),
                    ("tPLLL", left_leg.target, left_leg.target_vel),
                    ("tPRLL", right_leg.target, right_leg.target_vel),
                ];
                let dt_tgt = 1.0 / FIXED_HZ as f32;
                for (i, (nm, tgt_pos, tgt_vel)) in lb_tgt.iter().enumerate() {
                    let p = to_vello(*tgt_pos);
                    let v = to_vello(*tgt_vel);
                    let (ax, ay) = if probe.tgt_primed {
                        let dv = v - probe.prev_tgt_vel[i];
                        (dv.x / dt_tgt, dv.y / dt_tgt)
                    } else {
                        (0.0, 0.0)
                    };
                    let key_x = format!("{}x", nm);
                    let key_y = format!("{}y", nm);
                    let key_vx = format!("{}vx", nm);
                    let key_vy = format!("{}vy", nm);
                    let key_ax = format!("{}ax", nm);
                    let key_ay = format!("{}ay", nm);
                    row.set(&key_x, format!("{:.2}", p.x));
                    row.set(&key_y, format!("{:.2}", p.y));
                    row.set(&key_vx, format!("{:.2}", v.x));
                    row.set(&key_vy, format!("{:.2}", v.y));
                    row.set(&key_ax, format!("{:.1}", ax));
                    row.set(&key_ay, format!("{:.1}", ay));
                    probe.prev_tgt_vel[i] = v;
                }
                probe.tgt_primed = true;
            }
        }
    }
}

/// Close out a cell once the script has run to completion: classify, flush the
/// per-cell CSV from the shared buffer, then advance to the next cell or write
/// the summary and exit.
fn limb_finalize(
    mut probe: ResMut<Probe>,
    mut buf: ResMut<TelemetryBuffer>,
    params: Option<Res<HipPinParams>>,
    mut reset_ev: EventWriter<ResetLimbControlConstraintsEvent>,
    mut exit: EventWriter<AppExit>,
) {
    if probe.done || !probe.ready || !probe.script_done {
        return;
    }

    let mut report = finalize_cell(&probe);

    let inv = check_invariants(&buf.header, &buf.rows, &InvariantConfig::default());
    report.inv_ok = inv.ok;
    report.inv_detail = inv.detail.clone();
    report.hash = rows_hash(&buf.header, &buf.rows);

    // Whole-body verdicts + the events sidecar (no-op in limb mode).
    if probe.body {
        report_body(&probe, &buf, params.as_deref());
    }

    info!(
        "cell {} (kp={:.3}, kd={:.4}): {} swing={:.1}deg rms={:.2}deg rev={} settle={} inv={} hash={:016x}",
        report.cell,
        report.kp,
        report.kd,
        report.verdict,
        report.swing_deg,
        report.rms_deg,
        report.reversals,
        report
            .settle_s
            .map(|t| format!("{t:.2}s"))
            .unwrap_or_else(|| "none".into()),
        if report.inv_ok {
            "ok"
        } else {
            report.inv_detail.as_str()
        },
        report.hash,
    );

    let cell_path = if probe.sweep {
        let p = std::path::Path::new(&probe.csv_path);
        let dir = p.parent().map(|d| d.to_path_buf());
        let name = format!(
            "limb_pd_cell{:02}_kp{:.3}_kd{:.4}.csv",
            report.cell, report.kp, report.kd
        );
        match dir {
            Some(d) => d.join(name).to_string_lossy().to_string(),
            None => name,
        }
    } else {
        probe.csv_path.clone()
    };
    flush_csv(&cell_path, &buf);
    probe.reports.push(report);

    probe.cell += 1;
    if probe.cell >= probe.grid.len() {
        finish(&probe);
        probe.done = true;
        exit.write(AppExit::Success);
    } else {
        let (kp, kd) = probe.grid[probe.cell];
        probe.kp = kp;
        probe.kd = kd;
        // Reset the playhead for the next cell and restore rest angles so the
        // next cell starts from the same pose.
        if let Some(character) = probe.character {
            reset_ev.write(ResetLimbControlConstraintsEvent {
                limb: probe.focus.which(),
                character,
            });
        }
        probe.action = 0;
        probe.action_start_tick = probe.tick + 1;
        probe.applied_action = None;
        probe.script_done = false;
        probe.samples.clear();
        buf.clear_rows();
    }
}

/// Classify one cell from its samples: the transported arc over the whole script
/// plus the settled-tail quality over the **final** action's hold window.
fn finalize_cell(probe: &Probe) -> CellReport {
    let samples = &probe.samples;
    let mut rep = CellReport {
        cell: probe.cell,
        kp: probe.kp,
        kd: probe.kd,
        verdict: "SHORT".into(),
        swing_deg: 0.0,
        rms_deg: 0.0,
        catchup_deg: 0.0,
        reversals: 0,
        settle_s: None,
        max_omega: 0.0,
        inv_ok: true,
        inv_detail: String::new(),
        hash: 0,
    };
    if samples.len() < 5 {
        return rep;
    }

    // Unwrap theta so a swing across ±π doesn't fold.
    let mut un: Vec<f32> = Vec::with_capacity(samples.len());
    let mut prev: Option<f32> = None;
    for s in samples {
        let mut th = s.theta;
        if let Some(p) = prev {
            while th - p > PI {
                th -= 2.0 * PI;
            }
            while th - p < -PI {
                th += 2.0 * PI;
            }
        }
        prev = Some(th);
        un.push(th);
    }

    let mut mn = f32::INFINITY;
    let mut mx = f32::NEG_INFINITY;
    let mut max_omega = 0.0f32;
    for (i, s) in samples.iter().enumerate() {
        mn = mn.min(un[i]);
        mx = mx.max(un[i]);
        max_omega = max_omega.max(s.omega.abs());
    }
    let swing_deg = (mx - mn).to_degrees();
    rep.swing_deg = swing_deg;
    rep.max_omega = max_omega;

    if !swing_deg.is_finite() || !max_omega.is_finite() || swing_deg > 360.0 {
        rep.verdict = "DIVERGED".into();
        return rep;
    }
    if swing_deg < MIN_SWING_DEG {
        rep.verdict = "NO_SWING".into();
        return rep;
    }

    // The settled window = the final action's samples (the script should end on a
    // hold). Fall back to the whole run if the last action has too few samples.
    let last_act = samples.last().map(|s| s.act).unwrap_or(0);
    let mut win: Vec<(f32, f32, f32)> = samples
        .iter()
        .enumerate()
        .filter(|(_, s)| s.act == last_act)
        .map(|(i, s)| (s.tau, un[i].to_degrees(), s.omega))
        .collect();
    if win.len() < 5 {
        win = samples
            .iter()
            .enumerate()
            .map(|(i, s)| (s.tau, un[i].to_degrees(), s.omega))
            .collect();
    }
    if win.len() < 5 {
        return rep; // stays SHORT
    }

    let taus: Vec<f32> = win.iter().map(|r| r.0).collect();
    let thetas: Vec<f32> = win.iter().map(|r| r.1).collect();
    let omegas: Vec<f32> = win.iter().map(|r| r.2).collect();
    let v = tail_verdict(
        &taus,
        &thetas,
        &omegas,
        TAIL_S,
        TAIL_RMS_DEG,
        WAGGLE_REVERSALS,
        REVERSAL_DEADBAND,
    );
    let tail_mean = v.mean;
    rep.rms_deg = v.rms_deg;
    rep.reversals = v.reversals;

    let mut catchup = 0.0f32;
    for (_, th, _) in &win {
        catchup = catchup.max((th - tail_mean).abs());
    }
    rep.catchup_deg = catchup;

    // Settle time: first instant after the final window opens at which every
    // remaining sample stays in band about the settled tail mean.
    let rel_t0 = win[0].0;
    let nrels = win.len();
    let mut last_out = None;
    for (i, (_, th, _)) in win.iter().enumerate() {
        if (th - tail_mean).abs() > SETTLE_BAND_DEG {
            last_out = Some(i);
        }
    }
    rep.settle_s = if let Some(li) = last_out {
        if li + 1 < nrels {
            Some(win[li + 1].0 - rel_t0)
        } else {
            None
        }
    } else {
        Some(0.0)
    };

    rep.verdict = cell_verdict(v.pass, rep.settle_s, SLOW_SETTLE_S).to_string();
    rep
}

fn finish(probe: &Probe) {
    let p = std::path::Path::new(&probe.csv_path);
    let stem = p
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("limb_pd_log");
    let dir = p.parent().map(|d| d.to_path_buf());
    let name = format!("{stem}.summary.csv");
    let path = match dir {
        Some(d) => d.join(name).to_string_lossy().to_string(),
        None => name,
    };
    let header =
        "cell,kp,kd,verdict,swing_deg,rms_deg,catchup_deg,reversals,settle_s,max_omega,inv_ok,hash";
    let mut rows: Vec<String> = Vec::with_capacity(probe.reports.len());
    for r in &probe.reports {
        rows.push(format!(
            "{},{:.3},{:.4},{},{:.2},{:.2},{:.2},{},{},{:.3},{},{:016x}",
            r.cell,
            r.kp,
            r.kd,
            r.verdict,
            r.swing_deg,
            r.rms_deg,
            r.catchup_deg,
            r.reversals,
            r.settle_s
                .map(|t| format!("{t:.3}"))
                .unwrap_or_else(|| "none".into()),
            r.max_omega,
            r.inv_ok as u8,
            r.hash,
        ));
    }
    flush_rows(&path, header, &rows);
    info!(
        "done: {} cell(s) on {} ({}); summary: {}",
        probe.grid.len(),
        probe.focus.label(),
        probe.script_src,
        path
    );
}

// ---------------------------------------------------------------------------
// Whole-body (hip-pin) helpers
// ---------------------------------------------------------------------------

/// Compact code for the vertical phase, for the `vphase` CSV column (matches the
/// `VerticalPhase` declaration order).
fn vphase_code(p: VerticalPhase) -> u8 {
    match p {
        VerticalPhase::Stand => 0,
        VerticalPhase::Load => 1,
        VerticalPhase::Launch => 2,
        VerticalPhase::Flight => 3,
        VerticalPhase::Land => 4,
    }
}

/// The held-key mask the active command produces: `move` → left/right by the sign
/// of `dir[0]`, `jump` → the jump key, everything else → none.
fn active_mask(cmd: &Command) -> u8 {
    match cmd {
        Command::Move(a) => {
            if a.dir[0] > 0.0 {
                KEY_RIGHT
            } else if a.dir[0] < 0.0 {
                KEY_LEFT
            } else {
                0
            }
        }
        Command::Jump(a) => {
            // `dir` holds the move key through the jump (app-faithful: keep
            // running while you leap). Without it the move key is released for
            // the jump duration.
            let move_bit = match a.dir {
                Some(d) if d[0] > 0.0 => KEY_RIGHT,
                Some(d) if d[0] < 0.0 => KEY_LEFT,
                _ => 0,
            };
            move_bit | KEY_UP
        }
        _ => 0,
    }
}

/// The jump→walk window: the `move` immediately following a `jump` (its start/end
/// seconds, computed by cumulative `dur`). `None` when the script has no such
/// pattern, which disables the jump→walk verdict.
fn jumpwalk_window(script: &[Command]) -> Option<(f32, f32)> {
    let mut t = 0.0f32;
    let mut saw_jump = false;
    for c in script {
        let d = c.dur();
        if saw_jump && matches!(c, Command::Move(_)) {
            return Some((t, t + d));
        }
        if matches!(c, Command::Jump(_)) {
            saw_jump = true;
        }
        t += d;
    }
    None
}

/// Gravity for the harness, from `GRAVITY`: `on`/`1` → [`GRAVITY_ON`]; `off`/`0`
/// → zero; `x,y` → a raw vello-frame vector (x right, y down).
fn gravity_from_env() -> Vec2 {
    match std::env::var("GRAVITY").ok().as_deref().map(str::trim) {
        None | Some("") | Some("off") | Some("0") => Vec2::ZERO,
        Some("on") | Some("1") => GRAVITY_ON,
        Some(spec) => {
            let mut it = spec.split(',');
            let x = it.next().and_then(|v| v.trim().parse::<f32>().ok());
            let y = it.next().and_then(|v| v.trim().parse::<f32>().ok());
            match (x, y) {
                (Some(x), Some(y)) => Vec2::new(x, y),
                _ => {
                    warn!("GRAVITY=`{spec}` not understood; falling back to zero");
                    Vec2::ZERO
                }
            }
        }
    }
}

/// Pin params from env (body mode). `LEGS=off` keeps the pin on `P1/P2/P3` only;
/// otherwise the pin also drives the legs (the whole-body default).
fn hip_pin_params_from_env() -> HipPinParams {
    // Shared with the tuning app (`SUPPORT_SHARE`, `HIP_DROP`, ...).
    HipPinParams::from_env()
}

/// `pin_log.csv` + `.events.csv` → `pin_log.events.csv` (same dir, suffix before
/// extension).
fn path_with_suffix(main: &str, suffix: &str) -> String {
    let p = std::path::Path::new(main);
    let stem = p.file_stem().and_then(|s| s.to_str()).unwrap_or("log");
    match p.parent() {
        Some(dir) => dir
            .join(format!("{stem}{suffix}"))
            .to_string_lossy()
            .to_string(),
        None => format!("{stem}{suffix}"),
    }
}

/// Translate the active `move`/`jump` command into the real `ButtonInput<KeyCode>`
/// (edge transitions only, so held keys stay held) — the whole
/// `keyboard → leafwing ActionState → hip pin` path, same as the retired
/// `headless_pin` harness, but driven by the command playhead. Also records the
/// input timeline for the events sidecar.
fn drive_body_keys(mut keys: ResMut<ButtonInput<KeyCode>>, mut probe: ResMut<Probe>) {
    if !probe.body || probe.character.is_none() {
        return;
    }
    let cur = probe.action.min(probe.script.len().saturating_sub(1));
    let mask = if probe.script_done {
        0
    } else {
        probe.script.get(cur).map(active_mask).unwrap_or(0)
    };
    let kind: &'static str = if probe.script_done {
        "done"
    } else {
        probe.script.get(cur).map(|c| c.kind()).unwrap_or("none")
    };
    let t = probe.tick as f32 / FIXED_HZ as f32;
    for i in 0..3 {
        let bit = 1u8 << i;
        let now = mask & bit != 0;
        let was = probe.held_prev & bit != 0;
        if now && !was {
            keys.press(KEYS[i]);
            let id = probe.next_event_id;
            probe.next_event_id += 1;
            probe.events.push(format!("{t:.4},{id},{kind},{mask},0"));
        } else if !now && was {
            keys.release(KEYS[i]);
            let id = probe.next_event_id;
            probe.next_event_id += 1;
            probe.events.push(format!("{t:.4},{id},{kind},{mask},1"));
        }
    }
    probe.held_prev = mask;
}

/// Whole-body verdicts, computed from the captured buffer + the accumulators:
/// the pinned-frame (and leg) **bone rigidity**, the **jump→walk** commanded-vs-
/// realized hip-velocity comparison, the **leg foot-lag/wobble** verdict, and the
/// `<csv>.events.csv` input timeline. Pure post-processing (no ECS writes).
fn report_body(probe: &Probe, buf: &TelemetryBuffer, params: Option<&HipPinParams>) {
    // Rigid bones that must keep their length: the pinned `P1/P2/P3` frame (the
    // pin's job) and, when the pin drives the legs, both legs' thigh + calf.
    let mut bones = vec![
        Bone {
            a: ("P1x".into(), "P1y".into()),
            b: ("P2x".into(), "P2y".into()),
            rest_len: 0.0,
        },
        Bone {
            a: ("P2x".into(), "P2y".into()),
            b: ("P3x".into(), "P3y".into()),
            rest_len: 0.0,
        },
        Bone {
            a: ("P1x".into(), "P1y".into()),
            b: ("P3x".into(), "P3y".into()),
            rest_len: 0.0,
        },
    ];
    let mut labels: Vec<String> = vec!["P1-P2".into(), "P2-P3".into(), "P1-P3".into()];
    let legs_on = params.map(|p| p.hip_drives_legs).unwrap_or(false) && probe.leg_active;
    if legs_on {
        for (a, b) in [
            ("P3", "P40"),
            ("P40", "PLLL"),
            ("P3", "P41"),
            ("P41", "PRLL"),
        ] {
            bones.push(Bone {
                a: (format!("{a}x"), format!("{a}y")),
                b: (format!("{b}x"), format!("{b}y")),
                rest_len: 0.0,
            });
            labels.push(format!("{a}-{b}"));
        }
    }
    let rig = check_bone_rigidity(&buf.header, &buf.rows, &bones, PIN_RIGID_TOL);
    let worst_rel = rig.max_rel_dev.iter().copied().fold(0.0f32, f32::max);
    let mut rig_detail = String::new();
    for (i, nm) in labels.iter().enumerate() {
        let tt = buf
            .rows
            .get(rig.worst_row[i])
            .and_then(|r| r.split(',').next())
            .unwrap_or("?");
        rig_detail += &format!(" {nm}={:.4}@t{tt}", rig.max_rel_dev[i]);
    }
    info!(
        "body: rows={} nan_rows={} rig={} max_rel_dev={:.4}",
        buf.rows.len(),
        probe.nan_rows,
        if rig.ok { "ok" } else { "FAIL" },
        worst_rel,
    );
    info!("rigidity:{}", rig_detail);

    if let Some((b, e)) = probe.jw_window {
        info!(
            "jumpwalk(t{:.1}-{:.1}s): handoff_phase={} cmd_step={:.2}px/s@t{:.2} real_step={:.2}px/s@t{:.2} cmd_vs_real_gap={:.2}px/s@t{:.2}",
            b, e, probe.jw_phase, probe.jw_cmd_step, probe.jw_cmd_step_t,
            probe.jw_real_step, probe.jw_real_step_t, probe.jw_lag, probe.jw_lag_t,
        );
    }

    if legs_on {
        let mut rate = vec![0.0f32; probe.leg_err.len()];
        for i in 1..probe.leg_err.len() {
            let d = (probe.leg_t[i] - probe.leg_t[i - 1]).max(1e-6);
            rate[i] = (probe.leg_err[i] - probe.leg_err[i - 1]) / d;
        }
        let v = tail_verdict(
            &probe.leg_t,
            &probe.leg_err,
            &rate,
            LEG_TAIL_S,
            LEG_RMS_MAX,
            LEG_REV_MAX,
            LEG_RATE_DEADBAND,
        );
        info!(
            "leg: peak left={:.1}px@t{:.2} right={:.1}px@t{:.2} combined={:.1}px@t{:.2} tail={}(rms={:.3}px rev={})",
            probe.max_leg_err_lr[0], probe.max_leg_err_lr_t[0],
            probe.max_leg_err_lr[1], probe.max_leg_err_lr_t[1],
            probe.max_leg_err, probe.leg_peak_t,
            if v.pass { "PASS" } else { "WAGGLE" }, v.rms_deg, v.reversals,
        );
    }

    // The input timeline sidecar, so any CSV slice can be tied to a key event.
    let ev_path = path_with_suffix(&probe.csv_path, ".events.csv");
    let mut ev_out = String::from("t,event_id,label,keys,kind\n");
    if !probe.events.is_empty() {
        ev_out += &(probe.events.join("\n") + "\n");
    }
    if let Some(dir) = std::path::Path::new(&ev_path).parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    match std::fs::write(&ev_path, ev_out) {
        Ok(_) => info!("wrote {ev_path}"),
        Err(e) => error!("write failed ({ev_path}): {e}"),
    }

    // Per-leg foot-lag trace so the worst lag can be tied to a gait phase. The
    // signed `*_dx_px` columns are `foot.x - target.x`: negative = lagging
    // behind the +x command, positive = overshooting ahead of it.
    if legs_on && !probe.leg_t.is_empty() {
        let leg_path = path_with_suffix(&probe.csv_path, ".leg.csv");
        let mut leg_out = String::from("t,left_err_px,right_err_px,left_dx_px,right_dx_px\n");
        for (i, tt) in probe.leg_t.iter().enumerate() {
            let lr = probe.leg_err_lr.get(i).copied().unwrap_or([0.0, 0.0]);
            let dx = probe.leg_dx_lr.get(i).copied().unwrap_or([0.0, 0.0]);
            leg_out += &format!(
                "{tt:.4},{:.3},{:.3},{:.3},{:.3}\n",
                lr[0], lr[1], dx[0], dx[1]
            );
        }
        if let Some(dir) = std::path::Path::new(&leg_path).parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        match std::fs::write(&leg_path, leg_out) {
            Ok(_) => info!("wrote {leg_path}"),
            Err(e) => error!("write failed ({leg_path}): {e}"),
        }
    }
}
