//! Headless **limb**-PD waggle-tuning gate (arm **or** leg), script-driven.
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
//! | `move`  | **Reserved** — no actuator in this harness yet (logs once, acts as `wait`). |
//! | `jump`  | **Reserved** — no actuator in this harness yet (logs once, acts as `wait`). |
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
//! ## What is configured for a clean, isolated measurement
//!
//! - The spine drive is silenced (`SpineDriveEnabled(false)`) and there is **no**
//!   `HipPin`, so the `P1/P2/P3` frame is passive and never fights the limb.
//! - Zero gravity, no colliders. The frame particles are authored near-static, so
//!   only the driven limb moves under the PD.
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
//! (main CSV), `SWEEP=1`, `GRID="kp:kd,kp:kd,..."`. Output defaults under
//! `/home/ubuntu/bender/tmp/headless/`.

use std::time::Duration;

use bevy::{
    app::ScheduleRunnerPlugin,
    asset::{AssetMetaCheck, AssetPlugin},
    prelude::*,
    time::TimeUpdateStrategy,
    MinimalPlugins,
};

use serde::Deserialize;

use bevy_vello::{
    collision::{CollisionEventBatch, RemovedColliders, VelloCollisionWorld, VelloRayTraceCommand},
    integrations::physics::{
        systems::{
            apply_explicit_impulse_on_connection_particle, apply_explicit_impulse_on_softbody,
            generate_connection, generate_soft_body_for_collider, make_collision_constraints,
            remove_soft_body, update_collider_from_soft_body, update_connection_particles,
            update_constraint_world,
        },
        CharacterAngularConstraintEvent, CharacterFrameForceEvent, CharacterPivotForceEvent,
        CharacterPivotImpulseEvent, CharacterPivotVelocityEvent, ColliderExternalImpulseEvent,
        VelloCharacterPhysicsRoot, VelloConstraintWorld, VelloParticle,
    },
    prelude::*,
};

use game_lib::{
    capture_telemetry, cell_verdict,
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
    },
    check_invariants, flush_csv, flush_rows, rows_hash, tail_verdict, CharacterController,
    CharacterRoot, HeadlessTelemetryPlugin, IkMode, InvariantConfig, LeftArmController,
    LeftLegController, LimbConfig, ParticleTelemetryConfig, ResetLimbControlConstraintsEvent,
    RightArmController, RightLegController, SpineDriveEnabled, TelemetryBuffer, TelemetryRow,
    VelloCharacterPlugin, WhichLimb,
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const ASSET_ROOT: &str = "/home/ubuntu/bender/bevy_vello/examples/collision_detection/assets";
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

/// Reserved whole-body verbs (no actuator in this harness yet).
#[derive(Deserialize, Clone)]
struct MoveArgs {
    /// Reserved: no whole-body actuator in this harness yet.
    #[allow(dead_code)]
    dir: [f32; 2],
    dur: f32,
}

#[derive(Deserialize, Clone)]
struct JumpArgs {
    #[serde(default = "default_jump_dur")]
    dur: f32,
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
}

// ---------------------------------------------------------------------------
// Small math / env helpers
// ---------------------------------------------------------------------------

/// Engine Vello (y-down) → Bevy (y-up).
#[inline]
fn to_bevy(p: Vec2) -> Vec2 {
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
    let sweep = env_flag("SWEEP");
    let focus = LimbId::from_env();
    let (script, script_src) = load_script(focus);

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
        "limb-PD harness: focus={} script={} ({} command(s)), {} cell(s), sweep={}",
        focus.label(),
        script_src,
        script.len(),
        grid.len(),
        sweep
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
    };

    let mut app = App::default();
    app.add_plugins(
        MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
            1.0 / 240.0,
        ))),
    )
    .add_plugins(bevy::log::LogPlugin::default())
    .add_plugins(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        file_path: ASSET_ROOT.into(),
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
    // Zero gravity + no colliders = the isolated-limb case.
    .insert_resource(VelloConstraintWorld::new(Vec2::ZERO))
    .insert_resource(Time::<Fixed>::from_hz(FIXED_HZ))
    .insert_resource(VelloCollisionWorld::default())
    .insert_resource(RemovedColliders::default())
    .insert_resource(CollisionEventBatch::default())
    // No hip pin, and the spine drive must stay silent so the P1/P2/P3 frame is
    // passive (absent resource = ON, so this must be inserted explicitly).
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

    // Real physics stepping, same order as VelloCollisionResponsePlugin minus the
    // GPU tail. `update_character_movement` is injected by `GameCharacterPlugin`
    // in `FixedUpdate` before `update_constraint_world`.
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

    app.add_systems(Startup, register_character_assets);
    app.add_systems(Update, mark_assets_loaded);
    // Apply the current script command in `FixedPreUpdate`, i.e. before the
    // controller reads the target this same fixed step (FixedPreUpdate runs
    // before FixedUpdate in the FixedMain schedule).
    app.add_systems(FixedPreUpdate, drive_script);
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
        Command::Wait(_) | Command::Move(_) | Command::Jump(_) => Intent::Idle,
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
) {
    probe.tick += 1;
    let tick = probe.tick;

    // ---- Spawn V10 once both assets are parsed. ----
    if probe.character.is_none() {
        if character_svg.all_loaded() && character_blueprint.all_loaded() {
            let entity = commands
                .spawn((
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
                ))
                .id();
            probe.character = Some(entity);
            probe.spawn_tick = Some(tick);
            info!("V10 spawned, limb-PD clock starts");
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
        Intent::Idle => {
            if first_touch {
                if let Command::Move(_) | Command::Jump(_) = probe.script[cur] {
                    warn!(
                        "'{}' command has no actuator in this harness yet; acting as wait",
                        probe.script[cur].kind()
                    );
                }
            }
        }
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
        c.config.kd = kd;
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
        c.config.kd = kd;
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
        c.config.kd = kd;
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
        c.config.kd = kd;
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
            config.kd = kd;
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
}

/// Close out a cell once the script has run to completion: classify, flush the
/// per-cell CSV from the shared buffer, then advance to the next cell or write
/// the summary and exit.
fn limb_finalize(
    mut probe: ResMut<Probe>,
    mut buf: ResMut<TelemetryBuffer>,
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
