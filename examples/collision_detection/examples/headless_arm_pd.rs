//! Headless limb-PD waggle-tuning gate.
//!
//! Drives the **V10** character's **right arm** through a ~180° target swing and
//! reports whether the limb PD controller **settles** or **waggles**, so good
//! `Kp`/`Kd` values can be found. All of it runs the REAL game logic — the only
//! things removed are the GPU-bound tail of the physics chain (there are no
//! colliders, so collision detection would produce no contacts regardless).
//!
//! What is real here:
//! - character assembly from `v10.character.svg` + `v10.character.json` (game_lib)
//! - the real limb controller ([`update_character_movement`], `FixedUpdate`) which
//!   reads `RightArmController.target` + `config.ik_mode` and runs
//!   `calculate_limb_ik` — the PD under test
//! - the real XPBD connection/angular physics (`FixedUpdate` @ 90 Hz)
//!
//! What is configured for a clean, isolated measurement:
//! - **`IkMode::Aim { weapon_offset_y: 0.0 }`** (the user's choice — the mode that
//!   tends to waggle, and the one that makes a clean ~180° wrist rotate easy).
//! - The spine drive is silenced (`SpineDriveEnabled(false)`) and there is **no**
//!   `HipPin`, so the `P1/P2/P3` frame is passive and never fights the arm.
//! - Zero gravity, no colliders. `P1` is authored `inv_mass = 1e-5` (near-static
//!   anchor), so the shoulder (`P1`) stays put and only `P13` (elbow) / `PRLA`
//!   (wrist) move under the PD.
//!
//! ## The PD knobs (verified, in `calculate_limb_ik`)
//!
//! | Knob | Field | Default |
//! |---|---|---|
//! | **Kp** | `LimbConfig::rest_chase_alpha` | 0.5 |
//! | **Kd** | `LimbConfig::kd` | 0.065 s |
//!
//! The harness sweeps `Kp`/`Kd` live (env or a grid) and writes one CSV row per
//! fixed tick — every row carries all **13** v10 particle channels (position +
//! velocity, in JSON order, so `P2` is always index 2). It then classifies each
//! `(Kp,Kd)` cell.
//!
//! ## The swing plan
//!
//! `base` (rest) → `swing1` (ease to the opposite ~180° point) → `hold1` (settle)
//! → `flip` (slow ease back, sweeping through DC) → `release` (hold, measure the
//! residual ring). The target sits at a fixed large distance along a direction
//! that rotates 0→180°, so the Aim solver drives the **whole** forearm through a
//! ~180° arc.
//!
//! ## Verdict (per cell)
//!
//! - `DIVERGED` — `|theta|` range > 360° / non-finite
//! - `NO_SWING` — the measured arc is < 60° (the arm never moved — a rig problem)
//! - `WAGGLE`   — settled-tail RMS > ~1° or ≥ 4 angular-rate sign-reversals
//!   (measured only over the last 0.6 s, so the monotonic flip-tail catch-up is
//!   not mistaken for oscillation)
//! - `SLOW`     — settles too late (or not within the release window)
//! - `PASS`     — settles quickly and quietly
//!
//! ## Env
//!
//! `KP`, `KD`, `TARGET_DIST` (px; ≤0 = auto 3×forearm), `SWING_HZ` (half-swing
//! rate), `HOLD`, `DUR` (per cell), `OUT_CSV` (main CSV), `SWEEP=1`,
//! `GRID="kp:kd,kp:kd,..."`. Output defaults under
//! `/home/ubuntu/bender/tmp/headless/`.

use std::time::Duration;

use bevy::{
    app::ScheduleRunnerPlugin,
    asset::{AssetMetaCheck, AssetPlugin},
    prelude::*,
    time::TimeUpdateStrategy,
    MinimalPlugins,
};

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
    CharacterRoot, HeadlessTelemetryPlugin, IkMode, InvariantConfig, ParticleTelemetryConfig,
    RightArmController, SpineDriveEnabled, TelemetryBuffer, TelemetryRow, VelloCharacterPlugin,
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
/// Minimum arc (deg) for the swing to count as a real motion.
const MIN_SWING_DEG: f32 = 60.0;
/// Width (s) of the settled tail measured at the end of the release window.
/// The release window opens while the flip is still coasting to rest, so the
/// oscillation metrics are taken only from this final slice — a monotonic
/// catch-up then cannot masquerade as a wobble.
const TAIL_S: f32 = 0.6;
/// Residual RMS (deg, about the tail mean) above which the tail is judged noisy.
const TAIL_RMS_DEG: f32 = 1.0;
/// Angular-rate sign-reversals (with a deadband) within the tail above which the
/// arm is judged to oscillate — a direct, transient-immune wobble detector.
const WAGGLE_REVERSALS: u32 = 4;
/// `|omega|` (rad/s) below which a sample is ignored when counting reversals.
const REVERSAL_DEADBAND: f32 = 0.05;
/// Settle time (s after release start) above which the response is "SLOW".
const SLOW_SETTLE_S: f32 = 1.5;

const PI: f32 = std::f32::consts::PI;

/// Phase ids (also the CSV `phase` column via [`phase_label`]).
const PH_BASE: u8 = 0;
const PH_SWING: u8 = 1;
const PH_HOLD: u8 = 2;
const PH_FLIP: u8 = 3;
const PH_RELEASE: u8 = 4;

/// One telemetry sample (Bevy y-up frame) — the minimum the classifier reads.
/// Per-particle positions/velocities now arrive via the CSV channels (see
/// [`game_lib::capture_telemetry`]), so only the derived forearm angle/rate are
/// kept here.
#[derive(Clone, Copy)]
struct Sample {
    tau: f32,
    phase: u8,
    theta: f32, // forearm angle (rad), Bevy frame
    omega: f32, // forearm angular velocity (rad/s)
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
    /// `judge::check_invariants` verdict over the whole cell's rows.
    inv_ok: bool,
    /// First invariant failure detail (empty when `inv_ok`).
    inv_detail: String,
    /// Determinism fingerprint of the cell's rows (`judge::rows_hash`).
    hash: u64,
}

#[derive(Resource)]
struct Probe {
    character: Option<Entity>,
    /// Monotonic fixed-tick counter (the harness clock).
    tick: u64,
    /// Tick at which the character was spawned (for the initial up-time guard).
    spawn_tick: Option<u64>,
    /// True once the rig came up at least once; after that, later cells only
    /// need a short re-capture, so the initial up-time guard no longer applies.
    ever_ready: bool,

    // --- rig (captured once, when the arm is first ready) ---
    ready: bool,
    ticks_ready: u64,
    shoulder: Vec2, // P1 (P12), Bevy
    dir_a: Vec2,    // unit, Bevy — rest-pointing swing extreme
    target_dist: f32,

    // --- schedule ---
    base: f32,
    t_half: f32,
    hold: f32,
    dur: f32,

    // --- sweep ---
    grid: Vec<(f32, f32)>,
    cell: usize,

    // --- per-cell accumulation ---
    kp: f32,
    kd: f32,
    samples: Vec<Sample>,

    /// One classified report per completed cell.
    reports: Vec<CellReport>,
    done: bool,
    csv_path: String,
    sweep: bool,
}

fn phase_label(p: u8) -> &'static str {
    match p {
        PH_BASE => "base",
        PH_SWING => "swing1",
        PH_HOLD => "hold1",
        PH_FLIP => "flip",
        _ => "release",
    }
}

/// `(phase, s)` for the swing schedule at time `tau` seconds after ready.
/// `s` ∈ [0,1] drives the target direction `rotate(dir_a, s·π)`: 0 = rest side,
/// 1 = the opposite (~180°) side.
fn phase_and_s(tau: f32, base: f32, t_half: f32, hold: f32) -> (u8, f32) {
    if tau < base {
        return (PH_BASE, 0.0);
    }
    let e1 = base + t_half;
    if tau < e1 {
        let u = ((tau - base) / t_half).clamp(0.0, 1.0);
        return (PH_SWING, 0.5 - 0.5 * (PI * u).cos());
    }
    let e2 = e1 + hold;
    if tau < e2 {
        return (PH_HOLD, 1.0);
    }
    let e3 = e2 + t_half;
    if tau < e3 {
        let u = ((tau - e2) / t_half).clamp(0.0, 1.0);
        return (PH_FLIP, 0.5 + 0.5 * (PI * u).cos());
    }
    (PH_RELEASE, 0.0)
}

#[inline]
fn rotate(v: Vec2, a: f32) -> Vec2 {
    let (s, c) = a.sin_cos();
    Vec2::new(v.x * c - v.y * s, v.x * s + v.y * c)
}

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

fn main() {
    let sweep = env_flag("SWEEP");
    let csv_path = std::env::var("OUT_CSV")
        .unwrap_or_else(|_| "/home/ubuntu/bender/tmp/headless/arm_pd_log.csv".to_string());

    let grid: Vec<(f32, f32)> = match std::env::var("GRID") {
        Ok(g) if !g.trim().is_empty() => parse_grid(&g),
        _ if sweep => default_grid(),
        _ => vec![(env_f32("KP", 0.5), env_f32("KD", 0.065))],
    };

    let swing_hz = env_f32("SWING_HZ", 1.5);
    let probe = Probe {
        character: None,
        tick: 0,
        spawn_tick: None,
        ever_ready: false,
        ready: false,
        ticks_ready: 0,
        shoulder: Vec2::ZERO,
        dir_a: Vec2::X,
        target_dist: env_f32("TARGET_DIST", 0.0),
        base: 0.5,
        t_half: 0.5 / swing_hz.max(0.1),
        hold: env_f32("HOLD", 1.0),
        dur: env_f32("DUR", 6.0),
        kp: grid.first().map(|c| c.0).unwrap_or(0.5),
        kd: grid.first().map(|c| c.1).unwrap_or(0.065),
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
    // Zero gravity + no colliders = the isolated-arm case.
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
    // Write `RightArmController.target` + the PD gains in `FixedPreUpdate`, i.e.
    // before the controller reads them this same fixed step (FixedPreUpdate runs
    // before FixedUpdate in the FixedMain schedule).
    app.add_systems(FixedPreUpdate, arm_drive);
    // Sample after the whole fixed step (physics + controller), append the row via
    // the shared telemetry plugin, then finalize the cell. Gated so no stray row
    // lands in the gap between cells (the rig is briefly not "ready").
    app.add_systems(
        FixedPostUpdate,
        (arm_sample, capture_telemetry, arm_finalize)
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

#[allow(clippy::too_many_arguments)]
fn arm_drive(
    mut commands: Commands,
    mut probe: ResMut<Probe>,
    character_svg: Res<SvgCharacterAssetManager>,
    character_blueprint: Res<BlueprintCharacterAssetManager>,
    mut arm_q: Query<&mut RightArmController>,
    root_q: Query<&VelloCharacterPhysicsRoot>,
    particle_q: Query<&VelloParticle>,
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
            info!("V10 spawned, arm-PD clock starts");
        }
        return;
    }

    let character = probe.character.unwrap();

    // ---- Bail out if the rig never comes up (never silently hang). ----
    // Only guards the *initial* bring-up; between cells the rig is already up.
    if !probe.ready && !probe.ever_ready {
        if let Some(st) = probe.spawn_tick {
            if tick.saturating_sub(st) > (FIXED_HZ as u64) * 10 {
                error!("arm never became ready after 10 sim-seconds; aborting");
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
    let Ok(mut arm) = arm_q.get_mut(character) else {
        return;
    };

    // ---- Capture the rig once: shoulder anchor + the rest forearm direction. ----
    if !probe.ready {
        let Ok(p_p1) = particle_q.get(arm.particles[1]) else {
            return;
        };
        let Ok(p_p13) = particle_q.get(arm.particles[2]) else {
            return;
        };
        let Ok(p_prla) = particle_q.get(arm.particles[3]) else {
            return;
        };
        let shoulder = to_bevy(p_p1.particle.pos);
        let elbow = to_bevy(p_p13.particle.pos);
        let wrist = to_bevy(p_prla.particle.pos);
        let forearm = (wrist - elbow).normalize_or_zero();
        if forearm == Vec2::ZERO {
            error!("degenerate forearm; aborting");
            probe.done = true;
            exit.write(AppExit::error());
            return;
        }
        // The swing extreme is the rest forearm rotated 90°; `dir_a` and its
        // 180° opposite bracket the full sweep.
        probe.shoulder = shoulder;
        probe.dir_a = rotate(forearm, PI / 2.0);
        let f_len = (wrist - elbow).length();
        if probe.target_dist <= 0.0 {
            probe.target_dist = (3.0 * f_len).max(40.0);
        }
        let first_ready = !probe.ever_ready;
        probe.ticks_ready = tick;
        probe.ready = true;
        probe.ever_ready = true;
        probe.samples.clear();
        if first_ready {
            // The rig exists now (particles spawned + connectivity named), so the
            // telemetry system can resolve every v10 particle by name. The resolve
            // latch means this must NOT happen before the particles exist.
            commands.insert_resource(ParticleTelemetryConfig::v10(character));
        }
        info!(
            "cell {} (kp={:.3}, kd={:.3}): rig ready, swing dist {:.1}px",
            probe.cell, probe.kp, probe.kd, probe.target_dist
        );
    }

    let tau = (tick.saturating_sub(probe.ticks_ready)) as f32 / FIXED_HZ as f32;
    let (phase, s) = phase_and_s(tau, probe.base, probe.t_half, probe.hold);
    if phase == PH_RELEASE && tau >= probe.dur {
        // The telemetry system owns finalization (so it can grab the last
        // sample); the drive simply holds the rest target and waits.
        arm.target = probe.shoulder + probe.dir_a * probe.target_dist;
        return;
    }

    // ---- Command: Aim mode, target on the rotating swing direction. ----
    // `RightArmController.target` is Bevy world (y-up) coords; `calculate_limb_ik`
    // converts it to Vello internally via `bevy_to_vello`, so we feed it Bevy.
    let dir = rotate(probe.dir_a, s * PI);
    let target_bevy = probe.shoulder + dir * probe.target_dist;
    arm.config.ik_mode = IkMode::Aim {
        weapon_offset_y: 0.0,
    };
    arm.config.rest_chase_alpha = probe.kp;
    arm.config.kd = probe.kd;
    arm.target = target_bevy;

    let _ = (phase, s); // logged by telemetry
}

/// Whether telemetry should collect this tick: only while the rig is up and a
/// cell is in progress, so no row lands in the brief not-ready gap between cells.
fn telemetry_active(probe: Res<Probe>) -> bool {
    probe.character.is_some() && probe.ready && !probe.done
}

/// Collect one sample: the harness scalars (into [`TelemetryRow`], in a fixed
/// key order) and the theta/omega the classifier needs (into `Probe::samples`).
///
/// Runs before [`capture_telemetry`], which appends this tick's scalars plus all
/// 13 v10 particle channels. Elbow (`P13`) and wrist (`PRLA`) positions arrive via
/// those channels, so they are no longer duplicated as scalars.
fn arm_sample(
    mut probe: ResMut<Probe>,
    mut row: ResMut<TelemetryRow>,
    arm_q: Query<&RightArmController>,
    particle_q: Query<&VelloParticle>,
) {
    let Some(character) = probe.character else {
        return;
    };
    if probe.done || !probe.ready {
        return;
    }
    let Ok(arm) = arm_q.get(character) else {
        return;
    };
    let (Ok(p_p13), Ok(p_prla)) = (
        particle_q.get(arm.particles[2]),
        particle_q.get(arm.particles[3]),
    ) else {
        return;
    };

    let tick = probe.tick;
    let tau = (tick.saturating_sub(probe.ticks_ready)) as f32 / FIXED_HZ as f32;
    let (phase, s) = phase_and_s(tau, probe.base, probe.t_half, probe.hold);

    let elbow = to_bevy(p_p13.particle.pos);
    let wrist = to_bevy(p_prla.particle.pos);
    let v_elbow = to_bevy(p_p13.particle.velocity);
    let v_wrist = to_bevy(p_prla.particle.velocity);
    let forearm = wrist - elbow;
    let theta = forearm.y.atan2(forearm.x);
    let omega = bone_ang_vel(forearm, v_wrist - v_elbow);

    // Copy the scalars out before the `push`, so the mutable borrow of
    // `probe.samples` cannot overlap a read of `probe.kp/kd`.
    let (kp, kd) = (probe.kp, probe.kd);
    probe.samples.push(Sample {
        tau,
        phase,
        theta,
        omega,
    });

    // Same keys, same order, every tick (the header is frozen from these on the
    // first capture).
    row.clear();
    row.set("tau", format!("{:.4}", tau));
    row.set("t", format!("{:.4}", tick as f32 / FIXED_HZ as f32));
    row.set("phase", phase_label(phase));
    row.set("kp", format!("{:.3}", kp));
    row.set("kd", format!("{:.4}", kd));
    row.set("s", format!("{:.4}", s));
    row.set("theta_deg", format!("{:.3}", theta.to_degrees()));
    row.set("omega", format!("{:.3}", omega));
}

/// Close out a cell once the swing window has elapsed: classify, flush the
/// per-cell CSV from the shared buffer, then advance to the next cell or write
/// the summary and exit.
fn arm_finalize(
    mut probe: ResMut<Probe>,
    mut buf: ResMut<TelemetryBuffer>,
    mut exit: EventWriter<AppExit>,
) {
    if probe.done || !probe.ready {
        return;
    }
    let tau = (probe.tick.saturating_sub(probe.ticks_ready)) as f32 / FIXED_HZ as f32;
    let (phase, _) = phase_and_s(tau, probe.base, probe.t_half, probe.hold);
    if !(phase == PH_RELEASE && tau >= probe.dur) {
        return;
    }

    let mut report = finalize_cell(&probe);

    // Judge the cell's raw telemetry before it is flushed: a NaN row, a ragged
    // row or an out-of-ceiling magnitude is a physics bug, not a tuning result.
    // The hash fingerprints the whole cell for determinism replay.
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

    // Per-cell path: one file per cell in sweep mode, else the single out path.
    let cell_path = if probe.sweep {
        let p = std::path::Path::new(&probe.csv_path);
        let dir = p.parent().map(|d| d.to_path_buf());
        let name = format!(
            "arm_pd_cell{:02}_kp{:.3}_kd{:.4}.csv",
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

    // Advance to the next cell, or finish.
    probe.cell += 1;
    if probe.cell >= probe.grid.len() {
        finish(&probe);
        probe.done = true;
        exit.write(AppExit::Success);
    } else {
        let (kp, kd) = probe.grid[probe.cell];
        probe.kp = kp;
        probe.kd = kd;
        probe.ready = false;
        probe.samples.clear();
        // Keep the frozen header (particle resolution is stable across cells) but
        // drop this cell's rows so the next cell starts clean.
        buf.clear_rows();
    }
}

/// Classify one cell from its samples.
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

    // Release window samples, carrying omega so the tail detector can read the
    // sign of the angular rate directly.
    let rel: Vec<(f32, f32, f32)> = samples
        .iter()
        .enumerate()
        .filter(|(_, s)| s.phase == PH_RELEASE)
        .map(|(i, s)| (s.tau, un[i].to_degrees(), s.omega))
        .collect();
    if rel.len() < 5 {
        return rep; // stays SHORT
    }

    // The settled tail (last `TAIL_S` of the release window) is judged by the
    // shared detector in `judge`: residual RMS about the tail mean + deadbanded
    // angular-rate sign-reversals. Slicing the tail — not the whole window — is
    // what keeps a monotonic flip-tail catch-up from reading as a wobble.
    let taus: Vec<f32> = rel.iter().map(|r| r.0).collect();
    let thetas: Vec<f32> = rel.iter().map(|r| r.1).collect();
    let omegas: Vec<f32> = rel.iter().map(|r| r.2).collect();
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

    // Catch-up excursion (informational): how far the settled pose lay from the
    // furthest flip-tail sample. Large is fine — it only means a long flip.
    let mut catchup = 0.0f32;
    for (_, th, _) in &rel {
        catchup = catchup.max((th - tail_mean).abs());
    }
    rep.catchup_deg = catchup;

    // Settle time: first release instant after which every sample stays in band
    // about the settled tail mean.
    let rel_t0 = rel[0].0;
    let mut settle: Option<f32> = None;
    let nrels = rel.len();
    let mut last_out = None;
    for (i, (_, th, _)) in rel.iter().enumerate() {
        if (th - tail_mean).abs() > SETTLE_BAND_DEG {
            last_out = Some(i);
        }
    }
    if let Some(li) = last_out {
        if li + 1 < nrels {
            settle = Some(rel[li + 1].0 - rel_t0);
        }
    } else {
        settle = Some(0.0);
    }
    rep.settle_s = settle;

    rep.verdict = cell_verdict(v.pass, settle, SLOW_SETTLE_S).to_string();
    rep
}

fn finish(probe: &Probe) {
    let p = std::path::Path::new(&probe.csv_path);
    let stem = p
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("arm_pd_log");
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
    info!("done: {} cell(s); summary: {}", probe.grid.len(), path);
}
