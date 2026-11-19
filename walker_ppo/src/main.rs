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
//! ```
//!
//! Outputs (relative to the working directory):
//! - `tmp/walker_ppo/train.csv`    — one line per PPO update (trainer stats)
//! - `tmp/walker_ppo/episodes.csv` — one line per finished episode
//! - `tmp/walker_ppo/policy.json`  — latest checkpoint (deploy with
//!   `POLICY=tmp/walker_ppo/policy.json cargo run -p walker_qa`)

mod reward;

use std::sync::{Arc, RwLock};

use bevy::{asset::AssetMetaCheck, prelude::*, window::PresentMode};
use bevy_vello::{
    collision::{path_to_ccw_quad_path, VelloCollisionWorld, VELLO_COLLISION_COOL_DOWN_TIME},
    integrations::physics::{
        CharacterExternalPositionConstraintEvent, VelloConstraintWorld, VelloParticle,
    },
    prelude::*,
    vello::{
        kurbo::{Rect, Shape},
        peniko,
    },
    VelloCollider, VelloCollisionResponsePlugin, VelloPlugin,
};
use game_lib::{
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
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
    plugin::{
        actuate_system, observe_system, AttachWalker, WalkerControlPlugin, WalkerControlSet,
        WalkerRig,
    },
    walker::Walker,
    walker::WalkerConfig,
};
use walker_nn::{PolicyBundle, PpoConfig, PpoTrainer, Rollout, Transition};

// ---------------------------------------------------------------------------
// Scene constants — the v10 QA rig verbatim (the env MUST be the QA env so a
// trained policy drops into `walker_qa` with zero retuning).
// ---------------------------------------------------------------------------

const SVG_ASSET: &str = "v10.character.svg";
const BP_ASSET: &str = "v10.character.json";
const SVG_PATH: &str = "character/v10.character.svg";
const BP_PATH: &str = "character/v10.character.json";

const GRAVITY: Vec2 = Vec2::new(0.0, 980.0);
const SPAWN_TRANSLATION: Vec3 = Vec3::new(0.0, -30.0, 100.0);
const SPAWN_SCALE: Vec3 = Vec3::new(0.5, 0.5, 1.0);
const FOOT_SURFACE_Y: f32 = 312.0;
const FLOOR_HALF_THICKNESS: f32 = 20.0;
const FLOOR_HALF_LEN: f64 = 2400.0;

/// Hip-to-hip spacing between characters. The rig is ~140 px wide at scale
/// 0.5; 420 px keeps every pair disjoint in the GPU narrow phase (they share
/// collision_group 1 with the QA convention, so bodies must never touch).
const CHAR_SPACING: f32 = 420.0;

/// Standing hip height (measured, same value QA sets on `config.lower`).
const HIP_HEIGHT: f32 = 104.0;
/// Leg length `thigh + calf` at scale 0.5.
const LEG_LEN: f32 = 110.0;

/// Jump-intent magnitude fed into `Command::jump` (obs dim 13, normalized by
/// `v_scale`). Fixed this milestone — the intent channel just says "jump".
const DEFAULT_JUMP_SPEED: f32 = 100.0;

/// Teleport settle window (ticks of stiff position constraints before the
/// episode re-opens). ~0.5 s at 90 Hz.
const SETTLE_TICKS: u32 = 45;

/// Teleport stiffness (external position constraint compliance; the hip-pin
/// rest value is the order of `1e-6`).
const RESET_COMPLIANCE: f32 = 1e-6;

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
fn horizon() -> u32 {
    env_usize("HORIZON").unwrap_or(180) as u32
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
/// Fall-termination penalty.
fn fall_penalty() -> f32 {
    env_f32("FALL_PENALTY").unwrap_or(0.5)
}
/// Hip |v_x| under which the body is considered "vertical". Above `3·` the
/// gate the episode counts as failing toward a fall termination (out of
/// control is out of control). The dense climb/lateral shaping that used this
/// gate now lives in [`reward::JumpReward`]; the collector keeps it only for
/// the measured-fall termination check.
fn v_gate() -> f32 {
    env_f32("V_GATE").unwrap_or(60.0)
}
/// End-of-episode bonus per unit of peak height above standing. Prices the
/// *event* (the apex we render) as a terminal term, gated on survival. Kept
/// separate from the per-tick two-checkpoint reward because it is the one
/// term still keyed to the whole-episode peak.
fn r_peak() -> f32 {
    env_f32("R_PEAK").unwrap_or(3.0)
}

/// Assemble the two-checkpoint [`reward::RewardConfig`] from the shared env
/// knobs. Every unset knob falls back to [`reward::RewardConfig::default`], so
/// `cargo run -p walker_ppo` behaves without any env.
fn reward_cfg() -> reward::RewardConfig {
    let mut c = reward::RewardConfig {
        leg_len: LEG_LEN,
        hip_height: HIP_HEIGHT,
        gravity: GRAVITY.y,
        foot_surface_y: FOOT_SURFACE_Y,
        land_height_target: HIP_HEIGHT,
        ..reward::RewardConfig::default()
    };
    if let Some(v) = env_f32("W_ALIVE") {
        c.w_alive = v;
    }
    if let Some(v) = env_f32("W_APEX") {
        c.w_apex = v;
    }
    if let Some(v) = env_f32("APEX_TARGET") {
        c.apex_target = v;
    }
    if let Some(v) = env_f32("W_LIFTOFF") {
        c.w_liftoff = v;
    }
    if let Some(v) = env_f32("W_LIFTOFF_SPIN") {
        c.w_liftoff_spin = v;
    }
    if let Some(v) = env_f32("W_DCM") {
        c.w_dcm = v;
    }
    if let Some(v) = env_f32("W_XI_DOT") {
        c.w_xi_dot = v;
    }
    if let Some(v) = env_f32("W_POST") {
        c.w_post = v;
    }
    if let Some(v) = env_f32("W_LAND_SPIN") {
        c.w_land_spin = v;
    }
    if let Some(v) = env_f32("W_LATERAL") {
        c.w_lateral = v;
    }
    if let Some(v) = env_f32("W_ACT") {
        c.w_act = v;
    }
    if let Some(v) = env_f32("PEN_CAP") {
        c.pen_cap = v;
    }
    if let Some(v) = env_f32("V_GATE") {
        c.v_gate = v;
    }
    if let Some(v) = env_f32("V_SCALE") {
        c.v_scale = v;
    }
    if let Some(v) = env_f32("OMEGA_SCALE") {
        c.omega_scale = v;
    }
    if let Some(v) = env_f32("LAND_WINDOW_TS") {
        c.land_window_ts = v;
    }
    if let Some(v) = env_f32("LAND_WINDOW_E") {
        c.land_window_e = v;
    }
    if let Some(v) = env_f32("LAND_HEIGHT_TARGET") {
        c.land_height_target = v;
    }
    c
}

// ---------------------------------------------------------------------------
// Harness state
// ---------------------------------------------------------------------------

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
    train_rows: Vec<String>,
    ep_rows: Vec<String>,
    best_mean_reward: f32,
}

/// Per-character episode bookkeeping (env state the harness owns).
#[derive(Component)]
struct Env {
    /// True while teleporting/settling — no collection; command is None.
    parked: bool,
    settle: u32,
    /// Episode tick counter (collection ticks only).
    t: u32,
    ep_reward: f32,
    peak: f32,
    /// Consecutive ticks of the fall condition (see [`fall_hold`]).
    fall_streak: u32,
    /// The two-checkpoint jump reward (`plans/jump-ppo-residual-polar-action.md`
    /// §12). Owns the phase machine, the Δμ anti-dither state, and the
    /// finite-differenced `ξ̇` history.
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
    let hidden = win_hidden();
    let n_chars = env_usize("N").unwrap_or(if env_flag("PPO_SMOKE") { 1 } else { 8 });
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
    let mut trainer = PpoTrainer::new(jcfg.obs_dim, jcfg.action_dim, &jcfg.hidden, ppo_cfg, seed);
    if let Some(v) = env_f32("LR") {
        trainer.set_lr(v);
    }
    // Fresh policy starts at μ(s) ≡ 0 = the **standing pose** (both feet
    // planted via the stance polarity in `action_to_pose`, neutral depth, no
    // lean). A random-init agent therefore *stands* instead of toppling within
    // a few ticks, so episodes are long enough to explore the crouch→extend
    // sequence. Gradients still flow (the output layer's weight gradient rides
    // the hidden activations), so learning is unaffected.
    // Zero-init by default (opt out with ZERO_INIT=0): a fresh policy that
    // starts standing is the point of the milestone-1 de-risking.
    if std::env::var("ZERO_INIT").map(|v| v != "0").unwrap_or(true) {
        trainer.actor.zero_last_layer();
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
                    file_path: asset_root().into(),
                    ..default()
                }),
        )
        .add_plugins(VelloCharacterPlugin::default())
        .add_plugins(WalkerControlPlugin)
        .add_plugins(VelloPlugin)
        .add_plugins(VelloCollisionResponsePlugin)
        .insert_resource(SpineDriveEnabled(false))
        .init_state::<PpoPhase>()
        .insert_resource(NChars(n_chars))
        .insert_resource(ExitAfter(exit_updates))
        .insert_resource(PpoState {
            trainer,
            rollout: Rollout::default(),
            bundle,
            updates: 0,
            tick: 0,
            ep_rewards: Vec::new(),
            ep_lens: Vec::new(),
            ep_peaks: Vec::new(),
            train_rows: Vec::new(),
            ep_rows: Vec::new(),
            best_mean_reward: f32::NEG_INFINITY,
        })
        .add_systems(Startup, (setup_scene, apply_gravity, pause_physics).chain())
        .add_systems(
            Update,
            (
                register_character_assets,
                mark_assets_loaded,
                check_character_loaded.run_if(in_state(PpoPhase::Loading)),
                attach_walkers.run_if(in_state(PpoPhase::Game)),
                flush_rows_periodic.run_if(in_state(PpoPhase::Game)),
                smoke_exit.run_if(in_state(PpoPhase::Game)),
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

fn asset_root() -> String {
    std::env::var("ASSET_ROOT")
        .ok()
        .filter(|p| !p.trim().is_empty())
        .unwrap_or_else(|| {
            "/home/ubuntu/bender/bevy_vello/examples/collision_detection/assets".to_string()
        })
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

/// Camera at the scene origin: the rig sits at y ≈ 208 physics ⇒ −208 in Bevy
/// y-up, so lift the view to frame bodies + floor.
fn setup_scene(mut commands: Commands) {
    commands.spawn((Camera2d::default(), Transform::from_xyz(0.0, -60.0, 0.0)));

    let floor_y = -(FOOT_SURFACE_Y + FLOOR_HALF_THICKNESS);
    let rect = Rect::new(
        -FLOOR_HALF_LEN,
        -(FLOOR_HALF_THICKNESS as f64),
        FLOOR_HALF_LEN,
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
    info!("walker_ppo: floor surface y={FOOT_SURFACE_Y:.0}");
}

/// Spawn the N character roots across the floor.
fn spawn_characters(mut commands: Commands, n: Res<NChars>, mut done: Local<bool>) {
    if *done {
        return;
    }
    *done = true;
    let total = n.0;
    let x0 = -((total as f32 - 1.0) * CHAR_SPACING) / 2.0;
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
                svg_asset_id: SVG_ASSET.to_owned(),
                blueprint_asset_id: BP_ASSET.to_owned(),
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
    server: Res<AssetServer>,
    mut done: Local<bool>,
) {
    if *done {
        return;
    }
    *done = true;
    svg.push(
        server.load(SVG_PATH),
        SvgCharacterAssetMetaData::default(),
        SVG_ASSET,
    );
    bp.push(
        server.load(BP_PATH),
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
    )>,
    colliders: Query<(Entity, &Connectivity, &VelloCollider)>,
    walker_q: Query<(), With<Walker>>,
    state: Res<PpoState>,
    n: Res<NChars>,
    mut attached: Local<std::collections::HashSet<Entity>>,
) {
    for (root, _spine, left, right) in &roots {
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
        config.control.use_learned = false;
        config.jump_residual = Some(JumpResidual::with_policy(
            state.bundle.clone(),
            LearnMode::Train,
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
        // Parked: settle-reset to the spawn pose before the first episode.
        commands.entity(root).insert(Env {
            parked: true,
            settle: SETTLE_TICKS,
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
fn ppo_collect_system(mut walkers: Query<(&mut Walker, &mut Env)>, mut state: ResMut<PpoState>) {
    let horizon = horizon();
    let fall = fall_lean();
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
        // The **two-checkpoint reward** (`plans/jump-ppo-residual-polar-action.md`
        // §12): liftoff prices the conserved purchase, the Land window judges
        // capture, flight is alive-only. All the physics reuse lives in
        // [`reward::JumpReward`]; this site only recomputes μ (against the
        // FROZEN bundle — identical to the controller's forward) so the
        // anti-dither term rides the same mean the action was sampled from.
        let mu = state.bundle.read().unwrap().act_deterministic(&last.obs);
        let (mut reward, _events) = env.reward.step(&fiction, Some(&mu));
        // Termination: **measured fall only** (§12.5) — no predicted-doom cut,
        // which would invite the degenerate optimum of never launching.
        let bad = fiction.lean.abs() > fall || height < min_height() || lateral > 3.0 * v_gate();
        env.fall_streak = if bad { env.fall_streak + 1 } else { 0 };
        let fallen = env.fall_streak >= fall_hold();
        let over = env.t + 1 >= horizon;
        let done = fallen || over;
        if fallen {
            reward -= fall_penalty();
        }
        // Terminal peak-height bonus: pay for the EVENT, not the rate — the
        // apex is too brief for dense terms to price it, and the flail cannot
        // earn it without also paying the lateral charge. Gated on SURVIVAL:
        // run 7 paid it on crashes too, so "hop, then topple" earned the same
        // bonus as "hop, then land" — with exploration noise tripping most
        // episodes, the crash-hop was the practiced behavior and the policy
        // plateaued at a small hop followed by a forward face-plant.
        if done && !fallen {
            reward += env.peak.max(0.0) * r_peak();
        }

        env.t += 1;
        env.ep_reward += reward;
        env.peak = env.peak.max(height - 1.0);
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
            let tick = state.tick;
            state.ep_rewards.push(env.ep_reward);
            state.ep_lens.push(env.t);
            state.ep_peaks.push(env.peak);
            state.ep_rows.push(format!(
                "{},{},{:.4},{},{:.3}",
                tick,
                env.root.index(),
                env.ep_reward,
                env.t,
                env.peak
            ));
            // Reset on the spot (next tick's settle window).
            env.parked = true;
            env.settle = SETTLE_TICKS;
            env.t = 0;
            env.ep_reward = 0.0;
            env.peak = 0.0;
            env.fall_streak = 0;
            // Don't carry the phase machine or the action stream across the
            // teleport: the first tick of a new episode starts grounded with
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

/// The teleport reset: while parked, pin every rig particle at its blueprint
/// spawn pose (`particle_init`) with stiff one-time external position
/// constraints and zero its velocity — the engine's own public channels (same
/// Law-1 queue-only idiom as the hip-pin drive), one pass per settle tick.
fn ppo_reset_system(
    mut walkers: Query<(&mut Walker, &mut Env)>,
    particles: Query<&VelloParticle>,
    mut world: ResMut<VelloConstraintWorld>,
    rigs: Query<&WalkerRig>,
) {
    for (mut walker, mut env) in walkers.iter_mut() {
        if !env.parked {
            continue;
        }
        if env.settle == 0 {
            // Settle done: re-open collection with a FRESH observer so the
            // EMA/debounce cannot smear the teleport discontinuity into the
            // first observation of the new episode.
            env.parked = false;
            walker.observer = Default::default();
            continue;
        }
        env.settle -= 1;
        let Ok(rig) = rigs.get(env.root) else {
            continue;
        };
        // Every particle the rig knows: legs, feet, waist/hip (deduped).
        let mut roster: Vec<Entity> = Vec::new();
        roster.extend_from_slice(&rig.left.parts);
        roster.extend_from_slice(&rig.right.parts);
        roster.push(rig.handles.waist);
        roster.push(rig.handles.hip);
        roster.extend_from_slice(&rig.handles.feet);
        roster.sort();
        roster.dedup();
        for entity in roster {
            let Ok(p) = particles.get(entity) else {
                continue;
            };
            world.queue_character_one_time_external_position_constraint(
                &CharacterExternalPositionConstraintEvent {
                    character_entity: env.root,
                    joint_entity: entity,
                    config: vello_physics::ExternalPositionConstraintConfig {
                        target: p.particle_init.pos,
                        compliance: RESET_COMPLIANCE,
                        damping: 1.0,
                    },
                },
            );
            world.queue_connect_particle_velocity(env.root, &entity, Vec2::ZERO);
        }
    }
}

/// The PPO update + weight broadcast + trainer CSV, once per full window.
/// Runs in `Assemble` (after `assemble_system`), so telemetry stays coherent
/// and the next tick's controllers read the new snapshot.
fn ppo_update_system(mut state: ResMut<PpoState>, n: Res<NChars>) {
    let target = rollout_len(n.0);
    if state.rollout.len() < target {
        return;
    }
    let rollout = std::mem::take(&mut state.rollout);
    let stats = state.trainer.update(&rollout);
    state.updates += 1;
    let (updates, tick, n_ep) = (state.updates, state.tick, rollout.len());

    // Broadcast: freeze the new weights into the shared bundle (on-policy: the
    // write happens once, between rollouts — never during collection).
    if let Ok(mut guard) = state.bundle.write() {
        *guard = PolicyBundle::from_trainer(&state.trainer);
    }

    let mean_ep = mean(&state.ep_rewards);
    let mean_len = mean(
        &state
            .ep_lens
            .iter()
            .map(|&v| v as f32)
            .collect::<Vec<f32>>(),
    );
    let mean_peak = mean(&state.ep_peaks);
    state.train_rows.push(format!(
        "{updates},{tick},{:.4},{:.1},{:.3},{:.5},{:.5},{:.4},{:.4},{:.3}",
        mean_ep,
        mean_len,
        mean_peak,
        stats.policy_loss,
        stats.value_loss,
        stats.entropy,
        stats.approx_kl,
        stats.clip_frac
    ));
    if !state.ep_rewards.is_empty() {
        state.best_mean_reward = state.best_mean_reward.max(mean_ep);
    }
    info!(
        "update {updates} @tick {tick}/{target}: reward {mean_ep:.3} len {mean_len:.0} \
         peak {mean_peak:.3} pi {:.5} v {:.5} kl {:.4} clip {:.2} | buffer {n_ep}",
        stats.policy_loss, stats.value_loss, stats.approx_kl, stats.clip_frac,
    );
    state.ep_rewards.clear();
    state.ep_lens.clear();
    state.ep_peaks.clear();

    flush_state(&state);
    if let Err(e) = state.trainer.save_json("tmp/walker_ppo/policy.json") {
        warn!("walker_ppo: checkpoint save failed: {e}");
    }
}

fn mean(v: &[f32]) -> f32 {
    if v.is_empty() {
        0.0
    } else {
        v.iter().sum::<f32>() / v.len() as f32
    }
}

const TRAIN_HEADER: &str =
    "update,tick,mean_ep_reward,mean_ep_len,mean_peak_height,policy_loss,value_loss,entropy,approx_kl,clip_frac";
const EP_HEADER: &str = "tick,env,ep_reward,ep_len,peak_height";

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

/// The smoke exit: with `PPO_SMOKE=1`, after `UPDATES` (default 3) updates
/// assert every logged stat stayed finite, print PASS/FAIL, close the app.
/// A real run never exits — Ctrl-C / window close ends it, and the periodic
/// flush means everything through the last update is on disk.
fn smoke_exit(
    state: Res<PpoState>,
    exit_after: Res<ExitAfter>,
    mut exit: EventWriter<AppExit>,
    mut done: Local<bool>,
) {
    if *done || !env_flag("PPO_SMOKE") {
        return;
    }
    if state.updates < exit_after.0 {
        return;
    }
    *done = true;
    flush_state(&state);
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
