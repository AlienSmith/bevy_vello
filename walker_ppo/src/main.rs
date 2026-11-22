//! The PPO training app (`plans/ppo-learned-walker.md` §6).
//!
//! Same engine as `walker_qa` — real Bevy app, softbody colliders, GPU narrow
//! phase, XPBD solver, 90 Hz fixed clock — but N characters share ONE policy
//! (multi-env) and the harness closes transitions after every observe tick.
//!
//! First milestone: a **straight vertical jump** (plan §0). The held command
//! is `Command::jump`; the reward is hip height above the standing line; an
//! episode ends on the horizon or a fall, then the character is teleported
//! back to its blueprint rest pose with stiff one-time external position
//! constraints + zeroed velocities (the engine's own public channels — the
//! same Law-1 idiom as the hip-pin drive), and re-parked for a settle window.
//!
//! ```sh
//! WIN_HIDDEN=1 cargo run -p walker_ppo --release
//! PPO_SMOKE=1 cargo run -p walker_ppo            # fast exit + sanity check
//! N=8 ROLLOUT=120 HORIZON=180 cargo run -p walker_ppo --release
//! # Point training at the exact rig QA uses (default is the repo's v10):
//! ./target/release/walker_ppo --character \
//!     examples/collision_detection/assets/character/v10.character.json
//! ```
//!
//! Outputs (relative to the working directory):
//! - `tmp/walker_ppo/train.csv`    — one line per PPO update (trainer stats)
//! - `tmp/walker_ppo/episodes.csv` — one line per finished episode
//! - `tmp/walker_ppo/policy.json`  — latest checkpoint (deploy with
//!   `POLICY=tmp/walker_ppo/policy.json cargo run -p walker_qa`)

mod reward;

use std::sync::{Arc, RwLock};

use bevy::{
    asset::AssetMetaCheck,
    prelude::*,
    window::{PresentMode, PrimaryWindow},
};
use bevy_vello::{
    collision::{path_to_ccw_quad_path, VelloCollisionWorld, VELLO_COLLISION_COOL_DOWN_TIME},
    integrations::physics::VelloConstraintWorld,
    prelude::*,
    vello::{
        kurbo::{Rect, Shape},
        peniko,
    },
    VelloCollider, VelloCollisionResponsePlugin, VelloPlugin,
};
use game_lib::{
    character_asset::{
        paths::CharacterAssetPaths, BlueprintCharacterAsset, BlueprintCharacterAssetManager,
        BlueprintCharacterAssetMetaData, SvgCharacterAsset, SvgCharacterAssetManager,
        SvgCharacterAssetMetaData,
    },
    CharacterRoot, Connectivity, LeftLegController, RightLegController, SpineController,
    SpineDriveEnabled, VelloCharacterPlugin,
};
use walker_control::{
    control::Command,
    emit::LegHandles,
    jump_residual::{JumpResidual, JumpResidualConfig},
    learned::LearnMode,
    observe::ObserveHandles,
    plugin::{actuate_system, observe_system, AttachWalker, WalkerControlPlugin, WalkerControlSet},
    walker::Walker,
    walker::WalkerConfig,
};
use walker_nn::{PolicyBundle, PpoConfig, PpoTrainer, Rollout, Transition};

// ---------------------------------------------------------------------------
// Scene constants — the QA rig (the env MUST be the QA env so a trained policy
// drops into `walker_qa` with zero retuning).
//
// The *rig files themselves* are not constants: `CharacterAssetPaths` resolves
// them at startup (default `character/v10.character.*` under the repo's
// `examples/collision_detection/assets`), so `--character`/`CHARACTER_JSON`
// points training and QA at the very same SVG + blueprint.
// ---------------------------------------------------------------------------

const GRAVITY: Vec2 = Vec2::new(0.0, 980.0);
const SPAWN_TRANSLATION: Vec3 = Vec3::new(0.0, -30.0, 100.0);
const SPAWN_SCALE: Vec3 = Vec3::new(0.5, 0.5, 1.0);
const FOOT_SURFACE_Y: f32 = 312.0;
const FLOOR_HALF_THICKNESS: f32 = 20.0;
/// Minimum floor half-length (the old fixed value, 4800 px end to end). The
/// real floor is `max(this, layout extent + margin)` — see [`floor_half_len`].
/// It was a constant and became a bug at large `N`: 32 bodies at 420 px span
/// 13 020 px, so most of the row stood over empty space and could never land.
const FLOOR_HALF_LEN_MIN: f64 = 2400.0;

/// Hip-to-hip spacing between characters. The rig is ~140 px wide at scale
/// 0.5; 420 px keeps every pair disjoint in the GPU narrow phase (they share
/// collision_group 1 with the QA convention, so bodies must never touch).
const CHAR_SPACING: f32 = 420.0;

/// Leftmost character x for `n` bodies laid out centred on the origin — the
/// same formula [`spawn_characters`] uses, kept in one place so the floor can
/// be sized to cover it.
fn layout_x0(n: usize) -> f32 {
    -((n.max(1) as f32 - 1.0) * CHAR_SPACING) / 2.0
}

/// Floor half-length, in world px, sized to the **actual** character layout so
/// every body always has ground under it (plus one spacing of margin on each
/// end). Never smaller than [`FLOOR_HALF_LEN_MIN`].
fn floor_half_len(n: usize) -> f64 {
    let half = ((n.max(1) as f32 - 1.0) * CHAR_SPACING) / 2.0;
    // One full spacing of slack past the outermost body.
    (half as f64 + CHAR_SPACING as f64).max(FLOOR_HALF_LEN_MIN)
}

/// Standing hip height (measured, same value QA sets on `config.lower`).
const HIP_HEIGHT: f32 = 104.0;
/// Leg length `thigh + calf` at scale 0.5.
const LEG_LEN: f32 = 110.0;

/// Jump-intent magnitude fed into `Command::jump` (obs dim 13, normalized by
/// `v_scale`). Fixed this milestone — the intent channel just says "jump".
const DEFAULT_JUMP_SPEED: f32 = 100.0;

/// Settle window before an episode opens: **the idle standing pose**.
///
/// This must be long enough for the drop onto the floor to finish relaxing,
/// because the start pose decides the stroke: the feet are seen at different
/// heights for ~1 s after spawn (`d(L−R)` swings ±3 px), and the stroke always
/// lifts the HIGHER foot — so a short settle picks the support leg by phase.
/// Measured with the same checkpoint (policy 70), rise vs settle:
///
/// | settle | ticks | rise (px) |
/// |---|---|---|
/// | 0.5 s | 45 | 52.6 – 67.3 (all left-footed) |
/// | 1.2 s | 110 | 23.0 – 61.7, QA-shaped (23–39 typical) |
///
/// `walker_qa` stands ≥1 s before a scripted jump, so 110 ticks (1.22 s) is
/// the value that makes training start where the game idles. Override with
/// `SETTLE`.
const SETTLE_TICKS: u32 = 110;

/// Teleport stiffness (external position constraint compliance; the hip-pin
/// rest value is the order of `1e-6`).

// ---------------------------------------------------------------------------
// Env-overridable tunables (plan §5)
// ---------------------------------------------------------------------------

fn env_f32(key: &str) -> Option<f32> {
    std::env::var(key)
        .ok()
        .filter(|v| !v.trim().is_empty())
        .and_then(|v| v.trim().parse::<f32>().ok())
}
fn env_usize(key: &str) -> Option<usize> {
    std::env::var(key)
        .ok()
        .filter(|v| !v.trim().is_empty())
        .and_then(|v| v.trim().parse::<usize>().ok())
}
fn env_flag(key: &str) -> bool {
    std::env::var(key)
        .map(|v| !v.trim().is_empty() && v != "0")
        .unwrap_or(false)
}

fn win_hidden() -> bool {
    env_flag("WIN_HIDDEN")
}
/// Episode horizon (ticks). 180 ≈ 2 s.
///
/// There are **two** episode-end triggers in [`ppo_collect_system`]: this
/// horizon (a timeout) and the measured-fall gate. The reset is a *teleport*
/// (stiff position constraints in [`ppo_reset_system`]), which is what looks
/// like the body "zipping" back to standing — that is the respawn, and in
/// training it is exactly what should happen. The inconsistent timing is the
/// fall trigger firing early on some strokes while the rest ride to the horizon.
///
/// In `EXPERT_ONLY` (viewing) the timeout is disabled: the expert's own re-hop
/// rebases the clock after `flight_end + 0.25 s` while the jump intent is held,
/// so the stroke loops crouch → hop → land → stand continuously and only a real
/// fall respawns it. Set `HORIZON` explicitly to cap an episode if wanted.
fn horizon() -> u32 {
    if env_flag("EXPERT_ONLY") {
        env_usize("HORIZON").unwrap_or(u32::MAX as usize) as u32
    } else {
        // The episode must be long enough to contain the whole stroke PLUS the
        // landing window, or it times out before `landing_complete` can fire and
        // the landing reward is never paid. Stroke ≈ 1.15 s (clip + a little
        // flight), then `land_hold` seconds of grounded stabilization, plus
        // slack. Derive it so widening `JUMP_LAND_HOLD` can never silently
        // truncate the episode.
        env_usize("HORIZON").unwrap_or_else(|| {
            let secs = 1.15 + land_hold().max(0.0) + 0.55;
            (secs * 90.0).ceil() as usize
        }) as u32
    }
}
/// Collection window per update, summed across envs: `ROLLOUT` ticks/char.
fn rollout_len(n: usize) -> usize {
    env_usize("ROLLOUT").unwrap_or(180).max(16) * n
}
/// |lean| beyond which the episode is over (fallen).
fn fall_lean() -> f32 {
    env_f32("FALL_LEAN").unwrap_or(0.7)
}
/// Consecutive failed ticks before the fall gate fires. A real collapse
/// accelerates and stays failed for ≫ this window; a raw-exploration spike
/// (the random policy yanking `upper_angle` for one tick) recovers in 1–2,
/// so without the hold every episode dies at t=2 and PPO never sees the
/// crouch→extend sequence it needs to discover.
fn fall_hold() -> u32 {
    env_usize("FALL_HOLD").unwrap_or(6) as u32
}
/// Hip height (fraction of standing) below which the body has sunk — one of
/// the two fall conditions.
fn min_height() -> f32 {
    env_f32("MIN_HEIGHT").unwrap_or(0.5)
}
/// Hip |v_x| under which the body is considered "vertical". Above `3·` the
/// gate the episode counts as failing toward a fall termination (out of
/// control is out of control). The dense climb/lateral shaping that used this
/// gate now lives in [`reward::JumpReward`]; the collector keeps it only for
/// the measured-fall termination check.
fn v_gate() -> f32 {
    env_f32("V_GATE").unwrap_or(60.0)
}
/// Landing hold, seconds (`JUMP_LAND_HOLD`). Grounded time — counted from
/// **touchdown**, so it includes the authored absorb — the body must put in
/// before the landing counts as complete. **1.5 s** by default: on touchdown the
/// body is still extended and carries residual momentum, and 1 s was judged too
/// short to settle. A negative value disables the hold and restores the re-hop
/// loop. NOTE: [`horizon`] derives the episode length from this, so widening it
/// cannot truncate the episode.
/// Settle window before an episode opens, ticks (`SETTLE`, default
/// [`SETTLE_TICKS`]). The idle standing pose is what the jump must start from
/// (it is the game's idle state), and 45 ticks (0.5 s) stops *mid-relaxation*:
/// the feet are still swinging, and which of them sits higher — hence which one
/// the stroke lifts — depends on the phase. `walker_qa` stands for ≥1 s, so a
/// `SETTLE` of ~110 ticks (1.2 s) matches it.
fn settle_ticks() -> u32 {
    env_usize("SETTLE").unwrap_or(SETTLE_TICKS as usize) as u32
}

fn land_hold() -> f32 {
    env_f32("JUMP_LAND_HOLD").unwrap_or(1.5)
}
/// Landing-hold stability band, radians (`JUMP_LAND_LEAN`). The hold only
/// accrues while `|lean| ≤ this`.
fn land_lean() -> f32 {
    env_f32("JUMP_LAND_LEAN").unwrap_or(0.35)
}
/// Assemble the task-outcome [`reward::RewardConfig`] from env knobs. Every unset
/// knob falls back to [`reward::RewardConfig::default`], so
/// `cargo run -p walker_ppo` behaves without any env.
///
/// **Three knobs**, and that is the whole reward:
///   `FOOT_MARGIN`    how far (px) the feet may stray above the floor pre-liftoff
///   `FOOT_PENALTY`   cost per px per tick of straying beyond it
///   `R_LAND`         the one positive number: paid for landing after a lift-off
fn reward_cfg() -> reward::RewardConfig {
    let mut c = reward::RewardConfig::default();
    if let Some(v) = env_f32("FOOT_MARGIN") {
        c.foot_margin = v;
    }
    if let Some(v) = env_f32("FOOT_PENALTY") {
        c.foot_penalty = v;
    }
    if let Some(v) = env_f32("R_LAND") {
        c.r_land = v;
    }
    c
}

// ---------------------------------------------------------------------------
// Harness state
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Perf meter — is the simulation actually keeping up?
// ---------------------------------------------------------------------------

/// Render FPS **and** the physics tick rate, side by side.
///
/// Render FPS alone is not enough to judge this build: the PPO update blocks
/// the main thread, and Bevy's fixed-step scheduler then runs a burst of
/// catch-up ticks. A 60 fps readout can therefore sit on top of a physics
/// timeline that stalls and then fast-forwards — which is precisely the "is the
/// simulation broken?" question. So the meter tracks:
///
/// - `fps`  — `Update` iterations per wall second (what the window shows);
/// - `sim`  — `FixedUpdate` iterations per wall second. The loop targets 90 Hz
///   (`DT`), so a healthy run reads ≈ 90; a sustained drop means physics is not
///   keeping up, and a spike above 90 is catch-up after a stall.
/// - `stop` — the last PPO update's main-thread block, ms (0 if none yet).
#[derive(Resource)]
struct PerfMeter {
    last: std::time::Instant,
    frames: u32,
    sim_ticks: u32,
    pub fps: f32,
    pub sim: f32,
    /// Last `ppo_update_system` block, ms — the "freeze" cost, on screen.
    pub stop: f32,
    /// Process start, so the title can show how long the run has been going.
    pub start: std::time::Instant,
    /// Wall-clock seconds since the last log line.
    log_accum: f32,
}

impl Default for PerfMeter {
    fn default() -> Self {
        Self {
            last: std::time::Instant::now(),
            frames: 0,
            sim_ticks: 0,
            fps: 0.0,
            sim: 0.0,
            stop: 0.0,
            start: std::time::Instant::now(),
            log_accum: 0.0,
        }
    }
}

/// Count one physics tick. Lives in `FixedUpdate`, so it measures the scheduler
/// under load — not the render loop.
fn perf_count_sim(mut meter: ResMut<PerfMeter>) {
    meter.sim_ticks += 1;
}

/// Once a second: fold the counts into rates, show them in the window title, and
/// log them. The title needs no font asset (bevy_ui text would), so it works in
/// every build.
fn perf_report(
    mut meter: ResMut<PerfMeter>,
    mut windows: Query<&mut Window, With<PrimaryWindow>>,
    n: Res<NChars>,
    state: Res<PpoState>,
    exit: Res<ExitAfter>,
) {
    meter.frames += 1;
    let elapsed = meter.last.elapsed().as_secs_f32();
    meter.log_accum += elapsed;
    meter.last = std::time::Instant::now();
    if meter.log_accum < 1.0 {
        return;
    }
    let secs = meter.log_accum;
    meter.fps = meter.frames as f32 / secs;
    meter.sim = meter.sim_ticks as f32 / secs;
    meter.frames = 0;
    meter.sim_ticks = 0;
    meter.log_accum = 0.0;

    let (fps, sim, stop) = (meter.fps, meter.sim, meter.stop);
    // Wall-clock time since the process started, `m:ss` (or `h:mm:ss`).
    let secs = meter.start.elapsed().as_secs();
    let (h, m, s) = (secs / 3600, (secs / 60) % 60, secs % 60);
    let elapsed = if h > 0 {
        format!("{h}:{m:02}:{s:02}")
    } else {
        format!("{m}:{s:02}")
    };
    if let Ok(mut window) = windows.single_mut() {
        window.title = format!(
            "Walker PPO | N={} | upd {}/{} | {elapsed} | fps {fps:.0} | sim {sim:.0} Hz",
            n.0, state.updates, exit.0,
        );
    }
    info!(
        "perf: upd {}/{} | {elapsed} | fps {fps:.1} | sim {sim:.1} Hz (target 90) | \
         update_blocked {stop:.1} ms",
        state.updates, exit.0,
    );
}

/// Ignore the artifact of process startup: the first frame's `elapsed` is huge.
fn perf_reset(mut meter: ResMut<PerfMeter>) {
    meter.last = std::time::Instant::now();
    meter.log_accum = 0.0;
}

/// Shared training state: the one trainer, the rollout buffer, and the policy
/// snapshot behind the `Arc<RwLock<..>>` every character's `LearnedCtrl` reads.
#[derive(Resource)]
struct PpoState {
    trainer: PpoTrainer,
    rollout: Rollout,
    bundle: Arc<RwLock<PolicyBundle>>,
    updates: usize,
    tick: usize,
    /// Episode accumulators for the current update window.
    ep_rewards: Vec<f32>,
    ep_lens: Vec<u32>,
    ep_peaks: Vec<f32>,
    /// 1.0 if the episode was paid for landing.
    ep_lands: Vec<f32>,
    /// 1.0 if the episode produced a real launch.
    ep_launch: Vec<f32>,
    /// The foot-margin cost of the episode (positive number = charged).
    ep_footpen: Vec<f32>,
    /// `TRACE=1` buffer: one env's per-tick particle state, first episode only.
    trace_rows: Vec<String>,
    /// The env being traced (fixed at its first collected tick).
    trace_env: Option<Entity>,
    /// Trace finished + written; never trace again.
    trace_done: bool,
    /// Episodes traced so far, stand hip of the current one, and its lowest hip.
    trace_ep: usize,
    trace_stand: f32,
    trace_min: f32,
    /// One row per traced episode: `ep,ticks,stand_hip,min_hip,rise,landed,fallen`.
    /// The spread of `rise` is what "is this start state stable?" means.
    trace_summary: Vec<String>,
    train_rows: Vec<String>,
    ep_rows: Vec<String>,
    best_mean_reward: f32,
    /// Highest `mean_peak_height` seen (kept for the log; it SATURATES at 1.0
    /// once the apex target is calibrated, so it is no longer a selector).
    best_mean_peak: f32,
    /// Highest composite outcome score (`landed_pct + 0.5·mean_stand`) — the
    /// `policy_best` key. Named for what it replaced; it is the *score*, not the
    /// landing fraction.
    best_landed: f32,
    /// The mean `R_stand` credit at the best checkpoint (for the log line).
    best_stand: f32,
    /// **Generation swap handshake.** The update runs once, at a barrier where
    /// every env has finished exactly one episode (see
    /// [`ppo_update_system`]/[`ppo_reset_system`]). `ppo_update_system` sets
    /// this the instant the new weights are broadcast; the NEXT tick's
    /// `ppo_reset_system` despawns and respawns **all** bodies together and
    /// clears it. Splitting the two means the swap lands on a fully parked
    /// generation, never mid-stroke on a live body.
    swap_pending: bool,
}

/// Per-character episode bookkeeping (env state the harness owns).
#[derive(Component)]
struct Env {
    /// True while the body is settling (initial spawn or respawn); no
    /// collection, command is None.
    parked: bool,
    settle: u32,
    /// Set when an episode ended. The body then waits at the **generation
    /// barrier** — parked, standing, uncollected — until every other env has
    /// finished too; `ppo_update_system` then swaps the weights and
    /// `ppo_reset_system` tears this body down and rebuilds it in the same pass
    /// as all the others. A freshly assembled body (flag clear) just settles and
    /// unparks.
    respawn_pending: bool,
    /// World x the character spawns at; a respawn reuses it so the env keeps its
    /// place in the scene.
    spawn_x: f32,
    /// Episode tick counter (collection ticks only).
    t: u32,
    ep_reward: f32,
    peak: f32,
    /// Consecutive ticks of the fall condition (see [`fall_hold`]).
    fall_streak: u32,
    /// The task-outcome jump reward. Owns the phase machine, the one-tick CoM
    /// rise-rate history, and the episode's measured standing CoM reference.
    reward: reward::JumpReward,
    root: Entity,
}

#[derive(States, Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
enum PpoPhase {
    #[default]
    Loading,
    Game,
}

#[derive(Resource)]
struct NChars(usize);
#[derive(Resource)]
struct ExitAfter(usize);

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    // Which rig this run opens is a startup decision, not a compile-time one.
    // Resolving it before Bevy boots keeps `AssetPlugin.file_path` and the
    // `CharacterRoot` ids in sync, and turns a missing file into an immediate
    // error here instead of a permanent hang in `PpoPhase::Loading` (Bevy 0.16
    // has no `AssetEvent::Failed`). The same resolver runs in `walker_qa`, so
    // both harnesses open the identical SVG + blueprint.
    let assets = match CharacterAssetPaths::from_env_and_args()
        .and_then(|paths| paths.validate().map(|()| paths))
    {
        Ok(paths) => paths,
        Err(err) => {
            eprintln!("walker_ppo: {err}");
            std::process::exit(2);
        }
    };
    eprintln!(
        "walker_ppo: character assets (shared with walker_qa)\n{}",
        assets.describe()
    );

    let hidden = win_hidden();
    // `EXPERT_ONLY=1` is a **viewing** mode: the residual runs in `Deploy`
    // mode against a zero-init actor, so `δ ≡ 0` and the controller replays the
    // authored stroke exactly (`zero_residual_executes_the_expert`). No PPO
    // update runs and no checkpoint is written — nothing is trained.
    let expert_only = env_flag("EXPERT_ONLY");
    let n_chars = env_usize("N").unwrap_or(if env_flag("PPO_SMOKE") || expert_only {
        1
    } else {
        8
    });
    let exit_updates = env_usize("UPDATES").unwrap_or(if env_flag("PPO_SMOKE") { 3 } else { 400 });

    // Build the trainer *before* the app: its shapes define the policy every
    // controller adopts at attach time. These are the **JumpResidual** dims
    // (obs 22 / act 5 in the polar action space), NOT the walking `LearnedCtrl`
    // dims (16 / 7) — the residual is what this app trains
    // (`plans/jump-ppo-residual-polar-action.md` §3).
    let jcfg = JumpResidualConfig {
        hip_height: HIP_HEIGHT,
        leg_len: LEG_LEN,
        ..JumpResidualConfig::default()
    };
    let mut ppo_cfg = PpoConfig::default();
    // The residual acts in a *normalized* space (deltas are already divided by
    // `r_scale`/`s_scale`/`theta_scale` in `jump_residual.rs`), so a 0.5 std is
    // half the full action range per dimension — far too noisy for a residual
    // that should stay near the expert. 0.3 keeps early exploration inside the
    // scales; override with `INIT_STD` (`plans/jump-ppo-residual-polar-action.md`
    // §3, blocker 4).
    ppo_cfg.init_std = 0.3;
    if let Some(v) = env_f32("GAMMA") {
        ppo_cfg.gamma = v;
    }
    if let Some(v) = env_f32("LAMBDA") {
        ppo_cfg.gae_lambda = v;
    }
    if let Some(v) = env_f32("CLIP") {
        ppo_cfg.clip_eps = v;
    }
    if let Some(v) = env_f32("ENT") {
        ppo_cfg.ent_coef = v;
    }
    if let Some(v) = env_f32("EPOCHS") {
        ppo_cfg.epochs = v as usize;
    }
    if let Some(v) = env_f32("INIT_STD") {
        ppo_cfg.init_std = v;
    }
    let seed = env_usize("SEED").unwrap_or(0x5A17) as u64;
    let fresh = PpoTrainer::new(jcfg.obs_dim, jcfg.action_dim, &jcfg.hidden, ppo_cfg, seed);
    // ------------------------------------------------------------------
    // RESUME — continue a run from a checkpoint.
    // ------------------------------------------------------------------
    // Weights are restored; the Adam moments are NOT (see
    // `PpoTrainer::save_json`: `#[serde(skip)]`), so the first update after a
    // resume takes a slightly different step than the uninterrupted run would
    // have. That is a documented, accepted approximation — valuable because a
    // run that dies at update 3000 can still be continued instead of restarted.
    let mut trainer = match std::env::var("RESUME") {
        Ok(path) if !path.trim().is_empty() => match PpoTrainer::load_json(path.trim()) {
            // A shape mismatch means the checkpoint was trained against a
            // different obs/action space; silently loading it would feed the
            // policy garbage, so refuse and start fresh.
            Ok(t) if t.actor.dims == fresh.actor.dims && t.critic.dims == fresh.critic.dims => {
                // `eprintln!`, not `info!` — Bevy's LogPlugin is not installed
                // until `DefaultPlugins` builds, which is AFTER this point, so
                // an `info!` here is silently dropped and the operator cannot
                // tell whether a resume actually happened.
                eprintln!("walker_ppo: RESUMED from {path} (Adam moments reset)");
                t
            }
            Ok(t) => {
                eprintln!(
                    "walker_ppo: {path} is {}→{} but this env is {}→{}; starting FRESH",
                    t.actor.dims[0],
                    t.actor.dims[t.actor.dims.len() - 1],
                    jcfg.obs_dim,
                    jcfg.action_dim
                );
                fresh
            }
            Err(e) => {
                eprintln!("walker_ppo: RESUME load failed ({e}); starting FRESH");
                fresh
            }
        },
        _ => fresh,
    };
    if let Some(v) = env_f32("LR") {
        trainer.set_lr(v);
    }
    // Fresh policy starts at μ(s) ≡ 0 = the **standing pose** (both feet
    // planted via the stance polarity in `action_to_pose`, neutral depth, no
    // lean). A random-init agent therefore *stands* instead of toppling within
    // a few ticks, so episodes are long enough to explore the crouch→extend
    // sequence. Gradients still flow (the output layer's weight gradient rides
    // the hidden activations), so learning is unaffected.
    //
    // Zero-init applies to a **fresh** policy only (opt out with `ZERO_INIT=0`):
    // it is what makes a random-init agent stand instead of topple. It must NEVER
    // run on a loaded checkpoint — measured: `RESUME=policy_best.json` was
    // silently zeroed here and the character replayed the expert instead
    // (3/96 landings, and the identical `lean −1.65 / h 0.17` collapse signature
    // as the zero-residual baseline), so a loaded policy could not be observed at
    // all. Deploy/viewing a checkpoint is exactly when this matters most.
    let resume_path = std::env::var("RESUME").unwrap_or_default();
    let resumed = !resume_path.trim().is_empty();
    if resumed {
        eprintln!(
            "walker_ppo: policy loaded as-is (zero-init SKIPPED for {}; \
             pass ZERO_INIT=1 only if you really want to erase the actor)",
            resume_path.trim()
        );
    } else if std::env::var("ZERO_INIT").map(|v| v != "0").unwrap_or(true) {
        trainer.actor.zero_last_layer();
    }
    if expert_only {
        info!(
            "walker_ppo: EXPERT_ONLY — Deploy + zero-init ⇒ δ ≡ 0, so the residual \
             replays the authored stroke. No update, no checkpoint write."
        );
    }
    let bundle = Arc::new(RwLock::new(PolicyBundle::from_trainer(&trainer)));

    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Walker PPO".into(),
                        present_mode: PresentMode::AutoNoVsync,
                        visible: !hidden,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    file_path: assets.root_string(),
                    ..default()
                }),
        )
        .add_plugins(VelloCharacterPlugin::default())
        .add_plugins(WalkerControlPlugin)
        .add_plugins(VelloPlugin)
        .add_plugins(VelloCollisionResponsePlugin)
        .insert_resource(SpineDriveEnabled(false))
        .init_state::<PpoPhase>()
        .insert_resource(assets)
        .insert_resource(NChars(n_chars))
        .insert_resource(ExitAfter(exit_updates))
        .init_resource::<PerfMeter>()
        .insert_resource(PpoState {
            trainer,
            rollout: Rollout::default(),
            bundle,
            updates: 0,
            tick: 0,
            ep_rewards: Vec::new(),
            ep_lens: Vec::new(),
            ep_peaks: Vec::new(),
            ep_lands: Vec::new(),
            ep_launch: Vec::new(),
            ep_footpen: Vec::new(),
            trace_rows: Vec::new(),
            trace_env: None,
            trace_done: false,
            trace_ep: 0,
            trace_stand: f32::NAN,
            trace_min: f32::INFINITY,
            trace_summary: Vec::new(),
            train_rows: Vec::new(),
            ep_rows: Vec::new(),
            best_mean_reward: f32::NEG_INFINITY,
            best_mean_peak: -1.0,
            best_landed: -1.0,
            best_stand: -1.0,
            swap_pending: false,
        })
        // `perf_reset` seeds the meter's clock ONCE: running it per-frame in
        // `First` would zero `log_accum` before it could ever reach one second
        // (and the readout would never print).
        .add_systems(
            Startup,
            (
                setup_scene,
                apply_gravity,
                pause_physics,
                clamp_virtual_time,
                perf_reset,
            )
                .chain(),
        )
        .add_systems(
            Update,
            (
                register_character_assets,
                mark_assets_loaded,
                check_character_loaded.run_if(in_state(PpoPhase::Loading)),
                attach_walkers.run_if(in_state(PpoPhase::Game)),
                flush_rows_periodic.run_if(in_state(PpoPhase::Game)),
                // Runs unconditionally (also while Loading) so the readout is
                // live from the first second — a stalled or never-ticking sim is
                // then visible immediately.
                perf_report,
                run_exit.run_if(in_state(PpoPhase::Game)),
            ),
        )
        .add_systems(OnEnter(PpoPhase::Game), spawn_characters)
        // Fixed-step harness, slotted into the walker plugin's chain:
        //   Input   (runs with intent/script; ours is the command source)
        //   Observe (after observe_system: close transitions on fresh fiction)
        //   Actuate (after actuate_system: teleport-reset constraint queue)
        //   Assemble(after assemble_system: the PPO update + broadcast)
        .add_systems(
            FixedUpdate,
            (
                // Counts scheduler ticks, whatever else is (or is not) running —
                // the cheapest possible "is the sim still advancing?" probe.
                perf_count_sim,
                ppo_command_system.in_set(WalkerControlSet::Input),
                ppo_collect_system
                    .in_set(WalkerControlSet::Observe)
                    .after(observe_system),
                ppo_reset_system
                    .in_set(WalkerControlSet::Actuate)
                    .after(actuate_system),
                ppo_update_system.in_set(WalkerControlSet::Assemble),
            ),
        )
        .run();
}

/// Gravity on (the world boots at zero — same as QA).
fn apply_gravity(mut world: ResMut<VelloConstraintWorld>) {
    world.set_gravity(GRAVITY);
}

/// Freeze the solver until every walker is attached; the first physics tick is
/// then the first walker tick.
fn pause_physics(mut collision: ResMut<VelloCollisionWorld>) {
    collision.paused = true;
}

/// Clamp the virtual clock so one long frame can never fast-forward the fixed
/// clock. The PPO update blocks the main thread for hundreds of ms; without a
/// cap the real delta is fed to `Time<Virtual>`, the fixed schedule then runs a
/// catch-up **burst** of sim ticks right after the freeze (the "lurch" the perf
/// log shows as a >90 Hz second). One sim tick of headroom is plenty: a normal
/// frame advances 0–1 ticks, so this only trims the pathological post-stall
/// frame. Tune with `MAX_FRAME_MS` (default ≈ 2 ticks at 90 Hz ≈ 22 ms).
fn clamp_virtual_time(mut time: ResMut<Time<Virtual>>) {
    let max_ms = env_f32("MAX_FRAME_MS").unwrap_or(1000.0 / 90.0 * 2.0);
    time.set_max_delta(std::time::Duration::from_secs_f32(max_ms / 1000.0));
}

/// Camera at the scene origin: the rig sits at y ≈ 208 physics ⇒ −208 in Bevy
/// y-up, so lift the view to frame bodies + floor.
fn setup_scene(mut commands: Commands, n: Res<NChars>) {
    commands.spawn((Camera2d::default(), Transform::from_xyz(0.0, -60.0, 0.0)));

    // Size the floor to the ACTUAL layout so every one of the N bodies always
    // has ground under it — a fixed half-length silently stranded most of a
    // 32-body row (see [`floor_half_len`]).
    let half_len = floor_half_len(n.0);
    let floor_y = -(FOOT_SURFACE_Y + FLOOR_HALF_THICKNESS);
    let rect = Rect::new(
        -half_len,
        -(FLOOR_HALF_THICKNESS as f64),
        half_len,
        FLOOR_HALF_THICKNESS as f64,
    );
    let shape = path_to_ccw_quad_path(&rect.to_path(0.1));
    let transform = Transform::from_xyz(0.0, floor_y, 0.0);

    let mut bg = VelloScene::default();
    bg.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        peniko::Color::rgb(1.0, 1.0, 1.0),
        None,
        &kurbo::Rect::new(-2048.0, -2048.0, 2048.0, 2048.0),
    );
    commands.spawn(VelloSceneBundle {
        scene: bg,
        ..default()
    });

    let mut floor_scene = VelloScene::default();
    floor_scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        peniko::Color::rgb(0.6, 0.6, 0.6),
        None,
        &rect,
    );
    commands.spawn((
        VelloSceneBundle {
            scene: floor_scene,
            transform,
            ..default()
        },
        VelloCollider::new(
            &shape,
            None,
            &rect,
            Vec2::ZERO,
            peniko::Brush::Solid(peniko::Color::rgb(1.0, 0.0, 0.0)),
            0.0,
            false,
            None,
            None,
            None,
            0,
            transform,
            VELLO_COLLISION_COOL_DOWN_TIME,
        ),
    ));
    let (x0, x1) = (layout_x0(n.0), -layout_x0(n.0));
    info!(
        "walker_ppo: floor surface y={FOOT_SURFACE_Y:.0}, spans x=[{:.0}, {:.0}] \
         (N={} bodies span [{x0:.0}, {x1:.0}])",
        -half_len, half_len, n.0
    );
}

/// Spawn the N character roots across the floor.
fn spawn_characters(
    mut commands: Commands,
    n: Res<NChars>,
    assets: Res<CharacterAssetPaths>,
    mut done: Local<bool>,
) {
    if *done {
        return;
    }
    *done = true;
    let total = n.0;
    let x0 = layout_x0(total);
    for i in 0..total {
        let x = x0 + i as f32 * CHAR_SPACING;
        commands.spawn((
            VelloSceneBundle {
                transform: Transform {
                    translation: Vec3::new(x, SPAWN_TRANSLATION.y, SPAWN_TRANSLATION.z),
                    scale: SPAWN_SCALE,
                    ..default()
                },
                ..default()
            },
            CharacterRoot {
                svg_asset_id: assets.svg_id().to_owned(),
                blueprint_asset_id: assets.json_id().to_owned(),
                collision_group: 1,
            },
        ));
    }
    info!("walker_ppo: spawned {total} characters");
}

// ---------------------------------------------------------------------------
// Assets (the same load flow as walker_qa)
// ---------------------------------------------------------------------------

fn register_character_assets(
    mut svg: ResMut<SvgCharacterAssetManager>,
    mut bp: ResMut<BlueprintCharacterAssetManager>,
    assets: Res<CharacterAssetPaths>,
    server: Res<AssetServer>,
    mut done: Local<bool>,
) {
    if *done {
        return;
    }
    *done = true;
    svg.push(
        server.load(assets.svg_path()),
        SvgCharacterAssetMetaData::default(),
        assets.svg_id(),
    );
    bp.push(
        server.load(assets.json_path()),
        BlueprintCharacterAssetMetaData::default(),
        assets.json_id(),
    );
    info!(
        "character assets: svg={} json={} (root {})",
        assets.svg_path(),
        assets.json_path(),
        assets.root().display()
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

fn check_character_loaded(
    svg: Res<SvgCharacterAssetManager>,
    bp: Res<BlueprintCharacterAssetManager>,
    mut next: ResMut<NextState<PpoPhase>>,
) {
    if svg.all_loaded() && bp.all_loaded() {
        next.set(PpoPhase::Game);
    }
}

// ---------------------------------------------------------------------------
// Attach
// ---------------------------------------------------------------------------

/// Attach a Train-mode walker to every assembled character; un-pause physics
/// once all N are attached. Also inserts each character's `Env` (parked, so
/// the first settle teleport restores the rest pose before collection opens).
fn attach_walkers(
    mut commands: Commands,
    mut collision: ResMut<VelloCollisionWorld>,
    roots: Query<(
        Entity,
        &SpineController,
        &LeftLegController,
        &RightLegController,
        &Transform,
    )>,
    colliders: Query<(Entity, &Connectivity, &VelloCollider)>,
    walker_q: Query<(), With<Walker>>,
    state: Res<PpoState>,
    n: Res<NChars>,
    mut attached: Local<std::collections::HashSet<Entity>>,
) {
    for (root, _spine, left, right, root_tf) in &roots {
        if attached.contains(&root) || walker_q.get(root).is_ok() {
            continue;
        }
        let mut foot_colliders: [Option<Entity>; 2] = [None, None];
        for (e, conn, _c) in &colliders {
            if conn.character != root {
                continue;
            }
            match &*conn.name {
                "PLLL" | "LLL" => foot_colliders[0] = Some(e),
                "PRLL" | "RLL" => foot_colliders[1] = Some(e),
                _ => {}
            }
        }
        let (Some(plll), Some(prll)) = (foot_colliders[0], foot_colliders[1]) else {
            continue; // feet not assembled yet — retry next frame
        };
        let handles = ObserveHandles {
            hip: left.particles[1],
            waist: left.particles[0],
            feet: [left.particles[3], right.particles[3]],
            foot_colliders: [plll, prll],
        };
        let left_handles = LegHandles {
            parts: left.particles,
            hip_joint: left.joints[0],
            knee_joint: left.joints[1],
            strut: left.strut,
        };
        let right_handles = LegHandles {
            parts: right.particles,
            hip_joint: right.joints[0],
            knee_joint: right.joints[1],
            strut: right.strut,
        };

        let mut config = WalkerConfig::default();
        config.lower.hip_height = HIP_HEIGHT;
        // The **residual** is what trains here, not the walking policy:
        // `use_learned = false` keeps the FSM on the authored paths, and
        // `control_stage` swaps in `JumpResidual` whenever the machine enters
        // `LocoMode::Jumping` (`plans/jump-ppo-residual-polar-action.md` §3).
        // The dims must match the trainer built in `main()`.
        let jcfg = JumpResidualConfig {
            hip_height: HIP_HEIGHT,
            leg_len: LEG_LEN,
            ..JumpResidualConfig::default()
        };
        // `EXPERT_ONLY` runs the same controller deterministically: Deploy mode
        // takes μ with no exploration noise, so a zero-init actor gives δ ≡ 0
        // and the expert is what you see. Training mode would sample them.
        let learn_mode = if env_flag("EXPERT_ONLY") {
            LearnMode::Deploy
        } else {
            LearnMode::Train
        };
        config.control.use_learned = false;
        // A jump episode ends on a **completed landing** (absorb + stabilization
        // hold), not on touchdown — see `JumpCtrl::landing_complete`.
        config.control.jump_land_hold = land_hold();
        config.control.jump_land_lean = land_lean();
        config.jump_residual = Some(JumpResidual::with_policy(
            state.bundle.clone(),
            learn_mode,
            jcfg,
            &config.lower,
        ));

        commands.trigger_targets(
            AttachWalker {
                config,
                facing: 1.0,
                handles,
                left: left_handles,
                right: right_handles,
            },
            root,
        );
        // Freshly assembled: settle briefly (a *respawn* body is already built
        // at rest, so nothing is dragged), then open the first episode.
        commands.entity(root).insert(Env {
            parked: true,
            settle: settle_ticks(),
            respawn_pending: false,
            spawn_x: root_tf.translation.x,
            t: 0,
            ep_reward: 0.0,
            peak: 0.0,
            fall_streak: 0,
            reward: reward::JumpReward::new(reward_cfg()),
            root,
        });
        attached.insert(root);
    }

    if attached.len() == n.0 && collision.paused {
        collision.paused = false;
        info!("walker_ppo: all walkers attached — physics live");
    }
}

// ---------------------------------------------------------------------------
// Fixed-step harness
// ---------------------------------------------------------------------------

/// Command source: unparked characters hold the jump intent; a parked one gets
/// `None` (the FSM holds Standing while the teleport settles it). The walker
/// plugin's `intent_system` stays inert (no `WalkerIntent` components here).
fn ppo_command_system(mut walkers: Query<(&mut Walker, &Env)>) {
    let speed = env_f32("JUMP_SPEED").unwrap_or(DEFAULT_JUMP_SPEED);
    let jump = Command::jump_vertical(speed);
    for (mut walker, env) in walkers.iter_mut() {
        walker.command = if env.parked { None } else { Some(jump) };
    }
}

/// Close the PREVIOUS tick's transition with THIS tick's fiction.
///
/// `observe_system` just stamped the fiction from the solver's result of the
/// last control action, and the controller's `last` still holds that action's
/// `(obs, a, logp, value)` — the Law-2 pairing (controller reports, harness
/// collects reward/done).
fn ppo_collect_system(
    mut walkers: Query<(&mut Walker, &mut Env)>,
    mut state: ResMut<PpoState>,
    mut trace_on: Local<Option<bool>>,
) {
    let horizon = horizon();
    let fall = fall_lean();
    // Read `TRACE` once, not 90×N times a second.
    let trace_on = *trace_on.get_or_insert_with(|| env_flag("TRACE"));
    // Determinism (D1): query order is unstable across runs, so buffer the
    // tick's transitions and push them in sorted root order — the rollout (and
    // thus the float-summed gradients) then reproduces bit-for-bit.
    let mut staged: Vec<(u32, Transition)> = Vec::new();
    for (mut walker, mut env) in walkers.iter_mut() {
        if env.parked {
            continue;
        }
        let Some(last) = walker.controller.take_last() else {
            continue; // Standing recorded nothing
        };
        if walker.command.is_none() {
            continue; // shouldn't happen while unparked; drop the record
        }
        let fiction = *walker.fiction();
        let height = (FOOT_SURFACE_Y - fiction.hip.y) / HIP_HEIGHT;
        let lateral = fiction.v_hip.x.abs();
        // The **task-outcome reward**: takeoff CoM velocity, a monotone CoM apex
        // gate, and a landing window that pays for a settled capture. Standing
        // still pays exactly 0, so there is no do-nothing optimum to fall into;
        // the only charges are the harness's measured-fall gate below.
        // The reward has no per-tick term (that is the point): `step` only detects
        // the launch event and returns 0.
        let (mut reward, _events) = env.reward.step(&fiction);
        // Termination: **measured fall only** (§12.5) — no predicted-doom cut,
        // which would invite the degenerate optimum of never launching.
        // A **vertical hop** is not a fall: while airborne the torso can lean or
        // drift (the launch impulse torques the free body), and judging it there
        // cut every episode at the apex — so the descent and landing were never
        // rendered. Fall is a GROUNDED condition: the body has come back down and
        // is not recovering. `height` also only means "sunk" when grounded (in
        // flight the hip is legitimately far above the standing line).
        // **Landing grace window**: between touchdown and the end of the hold the
        // body is *supposed* to be folding, wobbling and recovering. Cutting it
        // there is the "respawns too soon on landing / responds too fast after
        // hitting the floor" failure. The stroke owns that window; the fall gate
        // stays silent until it closes (`landing_window_open`).
        let landing_grace = walker
            .controller
            .landing_window_open(&walker.config.control)
            .unwrap_or(false);
        let lean_bad = !landing_grace && fiction.grounded && fiction.lean.abs() > fall;
        // A DEEP CROUCH is not a fall. The absorb after touchdown legitimately
        // sinks the hip (measured: the landing is the wobblies part of the whole
        // stroke), so "sunk" only counts together with a lean — otherwise the
        // gate fires on a body that is merely folding its knees.
        let height_bad = !landing_grace
            && fiction.grounded
            && height < min_height()
            && fiction.lean.abs() > land_lean();
        let lateral_bad = !landing_grace && fiction.grounded && lateral > 3.0 * v_gate();
        let bad = lean_bad || height_bad || lateral_bad;
        env.fall_streak = if bad { env.fall_streak + 1 } else { 0 };
        let fallen = env.fall_streak >= fall_hold();
        let over = env.t + 1 >= horizon;
        // **Landed** is its own terminal: the stroke played its absorb and the
        // body then held a stable grounded stance for `JUMP_LAND_HOLD`
        // (`JumpCtrl::landing_complete`). Touchdown alone is NOT the end of the
        // episode — the body still has to land. The hold's own lean band means
        // this cannot fire on a toppling body; that failure stays with the
        // fall gate below.
        let landed = walker
            .controller
            .landing_complete(&walker.config.control)
            .unwrap_or(false);
        let done = fallen || over || landed;
        if done && env_flag("EXPERT_ONLY") {
            // Which trigger ended the episode — a timeout, or the fall gate and
            // *which* of its conditions. The reset is a stiff teleport, so an
            // early `done` mid-flight is exactly what looks like the body
            // "zipping" back to standing.
            info!(
                "EXPERT_ONLY episode end @t={} grounded={} over={over} fallen={fallen} \
                 landed={landed} lean={:.3}({}) hip_y={:.1} h={:.3}({}) vx={:.1}({}) phase={:?}",
                env.t,
                fiction.grounded,
                fiction.lean,
                lean_bad,
                fiction.hip.y,
                height,
                height_bad,
                fiction.v_hip.x,
                lateral_bad,
                walker.controller.mode(),
            );
        }
        // ---- the only payment ----
        // `R_LAND` if the launch was real and the episode did not end in a fall.
        // A fall pays 0 — there is no penalty term at all, so no policy can be
        // pushed into "never jump" to dodge one (that happened twice before).
        let mut land_pay = 0.0;
        if done {
            // Keyed to the harness's own landing verdict, not "did not fall": a body
            // that lifts off and wobbles to the horizon never landed.
            land_pay = env.reward.pay_land(fallen, landed);
            reward += land_pay;
        }
        env.t += 1;
        env.ep_reward += reward;
        // ---- optional per-tick trace (`TRACE=1`) ----
        // One env, its first episode, every body channel — so a deploy in
        // `walker_qa` can be diffed against what the trainer actually did.
        if trace_on && !state.trace_done && state.trace_rows.len() < TRACE_MAX_ROWS {
            // A respawn DESPAWNS and rebuilds the character, so the traced
            // entity changes every episode: re-target at each episode start
            // (rows are empty exactly then).
            let mut mine = state.trace_env == Some(env.root);
            if !mine && state.trace_rows.is_empty() {
                state.trace_env = Some(env.root);
                mine = true;
            }
            if mine {
                let tick = state.tick;
                if state.trace_rows.is_empty() {
                    state.trace_stand = fiction.hip.y;
                    state.trace_min = fiction.hip.y;
                }
                state.trace_min = state.trace_min.min(fiction.hip.y);
                state.trace_rows.push(format!(
                    "{},{},{:.2},{:.2},{:.1},{:.2},{:.2},{:.1},{:.2},{:.2},{},{},{}",
                    env.t,
                    tick,
                    fiction.hip.x,
                    fiction.hip.y,
                    fiction.v_hip.y,
                    fiction.com.x,
                    fiction.com.y,
                    fiction.v_com.y,
                    fiction.feet[0].pos.y,
                    fiction.feet[1].pos.y,
                    fiction.feet[0].touching as u8,
                    fiction.feet[1].touching as u8,
                    env.reward.lifted_off() as u8,
                ));
            }
        }
        // Log-only diagnostic (no reward consults it): the CoM height as a fraction
        // of the standing hip height, so a run's actual jump height is still visible.
        env.peak = env.peak.max((FOOT_SURFACE_Y - fiction.com.y) / HIP_HEIGHT);
        staged.push((
            env.root.index(),
            Transition {
                obs: last.obs,
                action: last.action,
                old_logp: last.logp,
                reward,
                done,
                value: last.value,
            },
        ));

        if done {
            if state.trace_env == Some(env.root) && !state.trace_done && trace_on {
                let (ep, ticks) = (state.trace_ep, state.trace_rows.len());
                let (stand_hip, min_hip) = (state.trace_stand, state.trace_min);
                let rise = stand_hip - min_hip;
                state.trace_summary.push(format!(
                    "{ep},{ticks},{stand_hip:.2},{min_hip:.2},{rise:.1},{},{}",
                    landed as u8, fallen as u8,
                ));
                write_rows(TRACE_PATH, TRACE_HEADER, &state.trace_rows);
                info!(
                    "walker_ppo: TRACE ep {ep}: {ticks} ticks, stand hip {stand_hip:.1}, \
                     min {min_hip:.1}, rise {rise:.1}, landed={landed} fallen={fallen}"
                );
                state.trace_ep += 1;
                let want = env_usize("TRACE_EPISODES").unwrap_or(8);
                if state.trace_ep >= want {
                    state.trace_done = true;
                    write_rows(
                        TRACE_SUMMARY_PATH,
                        TRACE_SUMMARY_HEADER,
                        &state.trace_summary,
                    );
                    info!("walker_ppo: TRACE summary ({want} episodes) -> {TRACE_SUMMARY_PATH}");
                } else {
                    // Next episode starts fresh; the start state is the respawn's.
                    state.trace_rows.clear();
                }
            }
            let tick = state.tick;
            state.ep_rewards.push(env.ep_reward);
            state.ep_lens.push(env.t);
            state.ep_peaks.push(env.peak);
            state.ep_lands.push(if land_pay > 0.0 { 1.0 } else { 0.0 });
            state
                .ep_launch
                .push(if env.reward.lifted_off() { 1.0 } else { 0.0 });
            state.ep_footpen.push(env.reward.foot_pen);
            state.ep_rows.push(format!(
                "{},{},{:.4},{},{},{:.3},{:.3}",
                tick,
                env.root.index(),
                env.ep_reward,
                env.t,
                if env.reward.lifted_off() { 1 } else { 0 },
                env.reward.foot_pen,
                land_pay,
            ));
            // Park and flag for a full **respawn**: `ppo_reset_system` despawns
            // this character and the factory rebuilds it at its rest pose. That
            // is a real reset — no stiff pin dragging a live soft body.
            env.parked = true;
            env.respawn_pending = true;
            env.settle = 0;
            env.t = 0;
            env.ep_reward = 0.0;
            env.peak = 0.0;
            env.fall_streak = 0;
            // Don't carry the phase machine or the action stream across the
            // respawn: the first tick of a new episode starts grounded with
            // no Δμ penalty.
            env.reward.reset();
        }
    }
    staged.sort_by_key(|(idx, _)| *idx);
    for (_, tr) in staged {
        state.rollout.push(tr);
    }
    state.tick += 1;
}

/// The episode reset: **despawn and respawn the whole character**.
///
/// The previous version pinned every particle back to `particle_init.pos` with a
/// `1e-6` compliance ("teleport"). That is not a respawn — the pins are stiff
/// enough to *drag* a live soft body, so a reset mid-flight visibly deformed the
/// character as it snapped home.
///
/// Despawning the root is the engine-sanctioned teardown:
///  - removing `ConnectivityRoot` fires `on_remove_connectivity_root`, which
///    despawns every part (and the joints, via `death_propegate`);
///  - removing `VelloCharacterPhysicsRoot` tears down the physics group
///    (`remove_group`), and removed colliders reach `remove_soft_body`, so no
///    stale soft body survives.
///
/// A fresh `CharacterRoot` at the same spawn x then goes through the normal
/// assembly path, so the new body is built at its blueprint rest pose — a true
/// respawn, with nothing to drag.
fn ppo_reset_system(
    mut commands: Commands,
    mut walkers: Query<(&mut Walker, &mut Env)>,
    roots: Query<&CharacterRoot>,
    mut state: ResMut<PpoState>,
) {
    // Respawning is gated on the **generation barrier**: when every env has
    // finished its episode, `ppo_update_system` swapped the weights and set
    // `swap_pending`. Only then does this system rebuild the whole generation at
    // once — so the new policy always starts from identical fresh bodies, never
    // mid-stroke. `EXPERT_ONLY` has no updates, so it respawns immediately.
    let expert_only = env_flag("EXPERT_ONLY");
    let reset_now = state.swap_pending || expert_only;
    for (mut walker, mut env) in walkers.iter_mut() {
        if !env.parked {
            continue;
        }

        // A freshly assembled body (initial spawn or the replacement we just
        // spawned) only needs to settle. Never respawn it again — that would
        // loop forever. Once the settle ticks out, open the episode with a
        // fresh observer so the EMA/debounce cannot smear the spawn transient
        // into the first observation.
        if !env.respawn_pending {
            if env.settle > 0 {
                env.settle -= 1;
            } else {
                env.parked = false;
                walker.observer = Default::default();
            }
            continue;
        }

        // Finished, but the generation is incomplete: stand at the barrier.
        if !reset_now {
            continue;
        }

        // `Walker`/`Env` live on the character ROOT entity (`on_attach_walker`
        // inserts onto the trigger target), so one despawn retires the whole
        // thing. Guard on the root still existing: deferred commands mean a
        // parked walker can be observed once after its root is already gone.
        let Ok(root_cfg) = roots.get(env.root) else {
            continue;
        };
        let (svg_id, bp_id, group) = (
            root_cfg.svg_asset_id.clone(),
            root_cfg.blueprint_asset_id.clone(),
            root_cfg.collision_group,
        );

        // 1. Tear the old body down through the engine's own teardown hooks:
        //    `ConnectivityRoot` removal despawns every part + joints, and
        //    `VelloCharacterPhysicsRoot` removal frees the physics group.
        commands.entity(env.root).despawn();

        // 2. Spawn a brand-new character at the same place. The factory's
        //    `OnAdd<CharacterRoot>` observer rebuilds particles, joints and
        //    colliders at the blueprint rest pose, then `attach_walkers` gives
        //    it a fresh `Walker` + `Env` (parked, so its settle runs first).
        commands.spawn((
            VelloSceneBundle {
                transform: Transform {
                    translation: Vec3::new(env.spawn_x, SPAWN_TRANSLATION.y, SPAWN_TRANSLATION.z),
                    scale: SPAWN_SCALE,
                    ..default()
                },
                ..default()
            },
            CharacterRoot {
                svg_asset_id: svg_id,
                blueprint_asset_id: bp_id,
                collision_group: group,
            },
        ));
    }
    // The whole generation has been despawned+re-spawned together; the barrier
    // is discharged until the next full rollout is collected.
    if state.swap_pending {
        state.swap_pending = false;
    }
}

/// The PPO update + weight broadcast + trainer CSV, once per complete
/// **generation**.
///
/// Runs in `Assemble` (after `assemble_system`). The trigger is a **barrier, not
/// a tick count**: every env must have finished its episode (all parked) before
/// the update fires. Collecting to a barrier is what makes the swap safe — the
/// new weights land on a generation that is entirely done, and
/// `ppo_reset_system` then rebuilds all bodies at once (this system only sets
/// `swap_pending`). No body is ever mid-crouch or mid-flight across a weight
/// change.
fn ppo_update_system(
    mut state: ResMut<PpoState>,
    walkers: Query<&Env>,
    n: Res<NChars>,
    mut meter: ResMut<PerfMeter>,
) {
    // Viewing mode: never learn, never overwrite the checkpoint.
    if env_flag("EXPERT_ONLY") {
        return;
    }
    // Already swapped and waiting for the simultaneous respawn — do not collect
    // or update again until the barrier is discharged.
    if state.swap_pending {
        return;
    }
    // The barrier: nothing is collectible until the last env finishes. A parked
    // env with `respawn_pending` has completed its episode; a parked one without
    // it is a fresh body still settling after the last respawn.
    let n_envs = walkers.iter().count();
    if n_envs == 0 {
        return;
    }
    let generation_done = walkers.iter().all(|env| env.parked && env.respawn_pending);
    if !generation_done {
        return;
    }
    // Guard against a degenerate empty rollout (e.g. an env that never ran).
    if state.rollout.len() < n.0 {
        return;
    }
    let rollout = std::mem::take(&mut state.rollout);
    // This runs **on the main thread inside FixedUpdate**, so it stalls the
    // render loop for its whole duration — the visible "freeze" every
    // generation. `Time<Virtual>` is clamped in `clamp_virtual_time` so the
    // stall can never fast-forward the fixed clock (no catch-up burst after).
    let t0 = std::time::Instant::now();
    let stats = state.trainer.update(&rollout);
    let update_ms = t0.elapsed().as_secs_f64() * 1000.0;
    // Publish the block to the on-screen perf readout, so the periodic "freeze"
    // is visible rather than mysterious.
    meter.stop = update_ms as f32;
    state.updates += 1;
    let (updates, tick, n_ep) = (state.updates, state.tick, rollout.len());
    let target = rollout_len(n.0);

    // Broadcast: freeze the new weights into the shared bundle (on-policy: the
    // write happens once, between generations — never during collection).
    if let Ok(mut guard) = state.bundle.write() {
        *guard = PolicyBundle::from_trainer(&state.trainer);
    }
    // Raise the generation barrier: the next `ppo_reset_system` (Actuate) tears
    // down and rebuilds **every** body together, so the new policy starts from a
    // clean, uniform generation instead of inheriting half-finished strokes.
    state.swap_pending = true;

    let mean_ep = mean(&state.ep_rewards);
    let mean_len = mean(
        &state
            .ep_lens
            .iter()
            .map(|&v| v as f32)
            .collect::<Vec<f32>>(),
    );
    let mean_peak = mean(&state.ep_peaks);
    // ---- outcome metrics: the thing the task is actually about ----
    // `landed_pct` is the fraction of episodes that completed the 1.5 s landing
    // window; `stand_pct` is the fraction that ended the episode on their feet
    // without a fall. Together with `launch_pct` these are what tell you, from the
    // csv alone, whether a run is on a wrong path — reward alone cannot.
    let landed_pct = mean(&state.ep_lands);
    // Fraction of episodes that produced a **real launch** (the precondition for
    // any payment). This is the first thing to look at: if it collapses, the policy
    // has learned to suppress the jump, whatever the reward says.
    let launch_pct = mean(&state.ep_launch);
    // Mean foot-margin cost (positive = charged). This is the constraint working:
    // it should be small once the policy stops flicking the feet.
    let mean_footpen = mean(&state.ep_footpen);
    // Composite outcome score for the deployable checkpoint: landings first, then
    // "did it actually jump". `landed_pct` alone is 1/32 coarse and noisy.
    let success = landed_pct + 0.5 * launch_pct;
    state.train_rows.push(format!(
        "{updates},{tick},{:.4},{:.1},{:.3},{:.3},{:.3},{:.4},{:.5},{:.5},{:.4},{:.4},{:.3}",
        mean_ep,
        mean_len,
        mean_peak,
        launch_pct,
        landed_pct,
        mean_footpen,
        stats.policy_loss,
        stats.value_loss,
        stats.entropy,
        stats.approx_kl,
        stats.clip_frac
    ));
    // ---- checkpointing ----
    // Three artifacts, because each answers a different question:
    //   policy.json          — ALWAYS the latest. Warm-resume a dead run.
    //   policy_snapshot.json — every `SNAPSHOT_EVERY` updates (default 100), so
    //                          an interrupted or diverged run loses at most
    //                          that many updates, not everything.
    //   policy_best.json     — the best policy by a **hop** criterion. The
    //                          artifact you actually DEPLOY. Without it a late
    //                          collapse silently overwrites the good policy.
    //
    // The selector must NOT be pure `mean_ep_reward`: that is the quantity the
    // policy games. Rank on the **task outcome** instead — completed landings,
    // then "did it launch at all", then reward.
    let is_best = if state.ep_rewards.is_empty() {
        false
    } else {
        let better = success > state.best_landed + 1e-4;
        let tied = (success - state.best_landed).abs() <= 1e-4;
        better || (tied && mean_ep > state.best_mean_reward)
    };
    if is_best {
        state.best_mean_reward = mean_ep;
        state.best_mean_peak = mean_peak;
        state.best_landed = success;
        state.best_stand = launch_pct;
    }
    info!(
        "update {updates} @tick {tick}/{target}: reward {mean_ep:.3} len {mean_len:.0} \
         peak {mean_peak:.3} launch {:.0}% landed {:.0}% footpen {mean_footpen:.3} succ {success:.3} \
         pi {:.5} v {:.5} kl {:.4} clip {:.2} | buffer {n_ep} \
         | update_blocked {update_ms:.1} ms",
        launch_pct * 100.0,
        landed_pct * 100.0,
        stats.policy_loss,
        stats.value_loss,
        stats.approx_kl,
        stats.clip_frac,
    );
    state.ep_rewards.clear();
    state.ep_lens.clear();
    state.ep_peaks.clear();
    state.ep_lands.clear();
    state.ep_launch.clear();
    state.ep_footpen.clear();

    flush_state(&state);
    // Default 10, not 100: the interesting part of a jump run is the FIRST ~50
    // updates (measured: real landings appear around update 40-55 and the policy
    // degenerates by ~100), so a 100-update snapshot interval captures none of it.
    let snapshot_every = env_usize("SNAPSHOT_EVERY").unwrap_or(10).max(1);
    let mut saves = vec!["tmp/walker_ppo/policy.json"];
    if is_best {
        saves.push("tmp/walker_ppo/policy_best.json");
    }
    let do_snapshot = updates % snapshot_every == 0;
    for path in saves {
        if let Err(e) = state.trainer.save_json(path) {
            warn!("walker_ppo: checkpoint save failed ({path}): {e}");
        }
    }
    if do_snapshot {
        // **Indexed history.** The rolling `policy_snapshot.json` alone is useless
        // for diagnosis: it holds only the most recent snapshot, so a 406-update
        // run left update 400 and nothing earlier — the early policy that actually
        // jumped was unrecoverable. Keep the rolling file (compat) AND one file
        // per snapshot, so any point of the run can be loaded in the viewer.
        let dir = "tmp/walker_ppo/snap";
        let _ = std::fs::create_dir_all(dir);
        let indexed = format!("{dir}/policy_{updates:04}.json");
        for path in ["tmp/walker_ppo/policy_snapshot.json", indexed.as_str()] {
            if let Err(e) = state.trainer.save_json(path) {
                warn!("walker_ppo: snapshot save failed ({path}): {e}");
            }
        }
        info!(
            "walker_ppo: snapshot @update {updates} -> {indexed} (every {snapshot_every}); \
             best outcome score {:.3}",
            state.best_landed
        );
    }
}

fn mean(v: &[f32]) -> f32 {
    if v.is_empty() {
        0.0
    } else {
        v.iter().sum::<f32>() / v.len() as f32
    }
}

const TRAIN_HEADER: &str = "update,tick,mean_ep_reward,mean_ep_len,mean_peak_height,launch_pct,landed_pct,mean_foot_pen,policy_loss,value_loss,entropy,approx_kl,clip_frac";
const EP_HEADER: &str = "tick,env,ep_reward,ep_len,launch,foot_pen,land_pay";
/// `TRACE=1`: per-tick particle state of ONE env, for its first episode. The
/// trainer otherwise writes only per-update / per-episode aggregates, so a
/// checkpoint's actual stroke could not be inspected at all — this is the
/// trainer-side counterpart of `walker_qa`'s telemetry, same channels.
const TRACE_HEADER: &str =
    "t,tick,hip_x,hip_y,hip_vy,com_x,com_y,com_vy,foot_l_y,foot_r_y,touch_l,touch_r,lifted_off";
const TRACE_PATH: &str = "tmp/walker_ppo/trace.csv";
/// One row per traced episode (see `PpoState::trace_summary`).
const TRACE_SUMMARY_PATH: &str = "tmp/walker_ppo/trace_summary.csv";
const TRACE_SUMMARY_HEADER: &str = "ep,ticks,stand_hip,min_hip,rise,landed,fallen";
/// 900 ticks = 10 s at 90 Hz, well past the horizon.
const TRACE_MAX_ROWS: usize = 900;

fn write_rows(path: &str, header: &str, rows: &[String]) {
    if let Some(dir) = path.rsplit_once('/').map(|(d, _)| d) {
        let _ = std::fs::create_dir_all(dir);
    }
    let mut out = String::from(header);
    out.push('\n');
    for r in rows {
        out.push_str(r);
        out.push('\n');
    }
    if let Err(e) = std::fs::write(path, out) {
        warn!("walker_ppo: write {path}: {e}");
    }
}

fn flush_state(state: &PpoState) {
    write_rows("tmp/walker_ppo/train.csv", TRAIN_HEADER, &state.train_rows);
    write_rows("tmp/walker_ppo/episodes.csv", EP_HEADER, &state.ep_rows);
}

/// Kill-safe periodic flush (between updates).
fn flush_rows_periodic(state: Res<PpoState>, mut last: Local<usize>) {
    if state.updates != *last || state.ep_rows.len() % 20 == 1 {
        *last = state.updates;
        flush_state(&state);
    }
}

/// The run exit: after `UPDATES` updates (default 400; 3 under `PPO_SMOKE`),
/// flush the CSVs and close the app. `PPO_SMOKE=1` additionally asserts every
/// logged stat stayed finite and prints PASS/FAIL.
///
/// This used to bail out unless `PPO_SMOKE` was set, so `UPDATES=100` was
/// silently ignored in a real (windowed) run and it never stopped.
fn run_exit(
    state: Res<PpoState>,
    exit_after: Res<ExitAfter>,
    mut exit: EventWriter<AppExit>,
    mut done: Local<bool>,
) {
    if *done {
        return;
    }
    if state.updates < exit_after.0 {
        return;
    }
    *done = true;
    flush_state(&state);
    if !env_flag("PPO_SMOKE") {
        info!(
            "walker_ppo: finished {}/{} updates | best succ {:.3} (landed {:.3} + launch {:.3}) | reward {:.4}",
            state.updates, exit_after.0, state.best_landed, state.best_landed, state.best_stand, state.best_mean_reward,
        );
        exit.write(AppExit::Success);
        return;
    }
    let finite = state
        .train_rows
        .iter()
        .all(|r| !r.contains("NaN") && !r.contains("inf") && !r.contains("-inf"));
    if finite && state.updates > 0 {
        info!(
            "PPO_SMOKE PASS: best mean_ep_reward = {:.4} over {} updates",
            state.best_mean_reward, state.updates
        );
        exit.write(AppExit::Success);
    } else {
        error!("PPO_SMOKE FAIL: non-finite or empty stats");
        exit.write(AppExit::Error(
            std::num::NonZeroU8::new(1).expect("nonzero"),
        ));
    }
}
