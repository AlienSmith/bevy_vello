//! Headless hip-pin tracking gate.
//!
//! Runs the REAL game logic for the **V10** character (13 particles) with the
//! **hip pin** owning `P1/P2/P3` and the spine drive silenced, on a
//! **floors-removed** world (no static colliders, gravity off by default) — so
//! the only actuator acting on the spine frame is the stiff external position
//! constraint the pin queues. Set `GRAVITY=on` to load that same frame with
//! the world's `g` and prove the (now light, `inv_mass 1.0`) `P1/P2/P3` still
//! hold. No window, no GPU.
//!
//! What is real here:
//! - character assembly from `v10.character.svg` + `v10.character.json` (game_lib)
//! - the Raibert resolver (`drive_hip_pin`, FixedUpdate @ 90 Hz) and the three
//!   `CharacterExternalPositionConstraintEvent`s it queues
//! - the real XPBD connection physics (FixedUpdate @ 90 Hz)
//! - the real input path: `ButtonInput<KeyCode>` → leafwing `ActionState` →
//!   the pin's resolver input (the pin reads `ActionState::pressed` directly)
//!
//! What is omitted (safe): the GPU-bound tail of the physics chain
//! (`run_broad_phase`, `run_gpu_collision`, raytrace). There are no colliders,
//! so collision detection would produce no contacts regardless.
//!
//! ## Telemetry
//!
//! One flat CSV, sampled **every fixed tick** (90 Hz) once the pin is `ready`.
//! **Every** particle that belongs to the character is tracked (all 13 for V10:
//! `PH,P0,P1,P2,P3,P40,P41,P10,P13,PLLA,PRLA,PRLL,PLLL`), discovered by
//! `VelloParticle.root_entity` + `Connectivity.character` and labelled from each
//! particle's `Connectivity` name, so the header is version-independent. For
//! each particle it logs world-space **position**, **velocity** and
//! **acceleration**:
//!
//! ```text
//! t,ev,held,<P> x,y,vx,vy,ax,ay,fx,fy, <P2> ...
//! ```
//! - `t`     — seconds since spawn
//! - `ev`    — id of the most recent event (see the events sidecar), or -1
//! - `held`  — key mask currently held: 1=Right(D) 2=Left(A) 4=Jump(W)
//! - `x,y`   — position, world (y-up px)
//! - `vx,vy` — velocity, world (px/s), read from `VelloParticle.particle.velocity`
//! - `ax,ay` — acceleration, world (px/s²), finite-differenced from the velocity
//!             between consecutive fixed ticks (raw — noisy by nature)
//! - `fx,fy` — the same acceleration passed through a light EMA (τ≈60 ms) for a
//!             far less noisy signal; prefer this for post-hoc analysis
//!
//! Positions are converted from the engine's Vello y-down frame to a world
//! y-up frame (`world = (x, -y)`) so acceleration signs read correctly.
//!
//! A sidecar `<name>.events.csv` carries the input timeline so any CSV slice can
//! be tied to a key event:
//!
//! ```text
//! t,event_id,label,keys,kind
//! ```
//! - `kind` — 0 = hold begin, 1 = hold end, 2 = edge tap
//! - `keys` — same mask as `held`
//!
//! Env: `PROBE_FPS` (default 60), `OUT_CSV` (default
//! `/home/ubuntu/bender/tmp/headless/pin_log.csv`), `GRAVITY` (default off;
//! `on`/`off`, or a raw `x,y` vello-frame vector).

use std::collections::HashSet;
use std::time::Duration;

use bevy::{
    app::ScheduleRunnerPlugin,
    asset::{AssetMetaCheck, AssetPlugin},
    input::{ButtonInput, InputSystem},
    prelude::*,
    time::TimeUpdateStrategy,
    MinimalPlugins,
};
use leafwing_input_manager::plugin::InputManagerSystem;

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
        VelloConstraintWorld, VelloParticle,
    },
    prelude::*,
};
use game_lib::{
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
    },
    check_bone_rigidity, check_invariants, default_input_map, drive_hip_pin, rigid_frame_targets,
    rows_hash, Bone, CharacterController, CharacterRoot, Connectivity, HipPin, HipPinParams,
    InvariantConfig, PlayerAction, PlayerMarker, SpineDriveEnabled, VelloCharacterPlugin,
};
use leafwing_input_manager::prelude::ActionState;

const ASSET_ROOT: &str = "/home/ubuntu/bender/bevy_vello/examples/collision_detection/assets";
/// Names the asset managers key on (filenames, as in the tuning app).
const SVG_ASSET: &str = "v10.character.svg";
const BP_ASSET: &str = "v10.character.json";
/// Paths relative to the assets root (AssetPlugin `file_path`).
const SVG_PATH: &str = "character/v10.character.svg";
const BP_PATH: &str = "character/v10.character.json";
/// The physics cadence the resolver + XPBD step at.
const FIXED_HZ: f64 = 90.0;
const EXIT_AT: f32 = 6.0;

/// Key mask bits (match the `held`/`keys` CSV columns).
const KEY_RIGHT: u8 = 1;
const KEY_LEFT: u8 = 2;
const KEY_UP: u8 = 4;

/// EMA time constant (seconds) for the smoothed acceleration column `f*`.
const FILT_TAU: f32 = 0.06;

/// Relative bone-length tolerance (fraction) for the rigid `P1/P2/P3` frame. The
/// pin should hold these near-exactly; anything larger is a physics bug.
const PIN_RIGID_TOL: f32 = 0.02;

/// The input timeline: `(label, begin, end, keys, kind)`.
/// `kind`: 0 = hold begin, 1 = hold end, 2 = edge tap.
/// Windows exercise the resolver: forward walk, stop, turn-around, then a hop.
fn event_script() -> Vec<(String, f32, f32, u8, u8)> {
    vec![
        ("walk".to_string(), 0.5, 2.5, KEY_RIGHT, 0), // forward walk
        ("turn".to_string(), 3.5, 4.5, KEY_LEFT, 0),  // turn-around
        ("jump".to_string(), 5.0, 5.12, KEY_UP, 2),   // jump edge
    ]
}

fn out_path() -> String {
    std::env::var("OUT_CSV")
        .unwrap_or_else(|_| "/home/ubuntu/bender/tmp/headless/pin_log.csv".to_string())
}

/// The sidecar events CSV path derived from the main one (`pin_log.csv` →
/// `pin_log.events.csv`).
fn events_path(main: &str) -> String {
    let p = std::path::Path::new(main);
    let stem = p.file_stem().and_then(|s| s.to_str()).unwrap_or("pin_log");
    match p.parent() {
        Some(dir) => dir
            .join(format!("{stem}.events.csv"))
            .to_string_lossy()
            .to_string(),
        None => format!("{stem}.events.csv"),
    }
}

/// Same magnitude the tuning app uses (`GRAVITY_ON`): 100 px = 1 m, so 980
/// px/s². Vello frame is x right, y **down**, so "down" is `+y`.
const GRAVITY_ON: Vec2 = Vec2::new(0.0, 980.0);

/// Gravity for the harness, from the `GRAVITY` env var. The harness exists to
/// prove the pin holds the frame; gravity is exactly what makes the now-light
/// `P1/P2/P3` (inv_mass 1.0) fight for their position, so it is a first-class
/// knob here.
/// - unset / `off` / `0` → `Vec2::ZERO` (the original floors-removed world)
/// - `on` / `1`          → [`GRAVITY_ON`] (matches the tuning app)
/// - `x,y`               → a raw vello-frame vector (x right, y down)
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

fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.trim().parse::<f32>().ok())
        .unwrap_or(default)
}

/// Pin params from env so the stiffness can be swept without a rebuild.
/// `COMPLIANCE` overrides the base constraint compliance; `DAMPING` overrides
/// the raw velocity damper. Softening (`kp`/`kd`/`soft_scale`) defaults to the
/// neutral [`HipPinParams`] values — the pin is a pure position constraint
/// unless a PD term is set, so the steady-state sag under gravity is the clean
/// `sag ≈ g·dt² + compliance·g` and scales linearly with `compliance`.
fn params_from_env() -> HipPinParams {
    let d = HipPinParams::default();
    HipPinParams {
        compliance: env_f32("COMPLIANCE", d.compliance),
        damping: env_f32("DAMPING", d.damping),
        ..d
    }
}

/// Engine Vello (y-down) → world (y-up) so acceleration signs read correctly.
#[inline]
fn vello_to_world(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

/// Sort key for the tracked-particle list: spine frame first (`PH,P0,P1,P2,P3`,
/// which puts the `P1/P2/P3` frame at indices `2..5`), then numeric `Pnn` ids,
/// then a stable alphabetical tail for the named particles.
fn particle_order(name: &str) -> (u8, u32) {
    const SPINE: [&str; 5] = ["PH", "P0", "P1", "P2", "P3"];
    if let Some(i) = SPINE.iter().position(|s| *s == name) {
        return (0, i as u32);
    }
    if let Some(num) = name.strip_prefix('P').and_then(|n| n.parse::<u32>().ok()) {
        return (1, num);
    }
    (2, 0)
}

#[derive(Resource)]
struct Probe {
    character: Option<Entity>,
    /// Fixed-step tick counter — the deterministic harness clock. The script
    /// windows and the exit gate both key off `tick * dt`, never wall-clock
    /// `Time`: otherwise the event edges land on different ticks run to run and
    /// the same seed produces a different run fingerprint.
    tick: u64,
    dt: f32,
    exited: bool,
    script: Vec<(String, f32, f32, u8, u8)>,
    /// Which windows were active last tick (per-event edge detection).
    event_active: Vec<bool>,
    /// Tracked particles in a stable order: `(label, entity)`.
    particles: Vec<(String, Entity)>,
    /// Indices into `particles` for the spine frame `[P1, P2, P3]`.
    spine_idx: [usize; 3],
    /// Previous velocity per tracked particle (px/s, Vello frame), for accel FD.
    prev_vel: Vec<Vec2>,
    /// EMA-smoothed acceleration per tracked particle (px/s², Vello frame).
    filt_acc: Vec<Vec2>,
    /// False until the first sample primes `prev_vel` (avoids a bogus first accel).
    primed: bool,
    /// The main CSV header, built once the particle list is known.
    head: Option<String>,
    log: Vec<String>,
    events: Vec<String>,
    next_event_id: u32,
    last_event: Option<u32>,
    max_err: f32,
    nan_rows: u32,
}

fn main() {
    let fps: f64 = std::env::var("PROBE_FPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(60.0);
    let mut app = App::default();
    app.add_plugins(
        MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
            1.0 / fps,
        ))),
    )
    .add_plugins(bevy::log::LogPlugin::default())
    .add_plugins(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        file_path: ASSET_ROOT.into(),
        ..default()
    })
    .add_plugins(bevy::input::InputPlugin)
    // Deterministic clock: advance exactly one fixed step per frame instead of
    // measuring wall-clock dt. With frame == fixed the fixed-step batching can no
    // longer vary run to run, so input injection, physics and the run fingerprint
    // are locked. Without this, `run_loop(1/60)` against a 90 Hz sim batches a
    // variable number of steps per frame and the scripted keys land on different
    // ticks each run (the source of the flaky `rows_hash`).
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / FIXED_HZ,
    )))
    .add_plugins(VelloCharacterPlugin)
    // Physics resources (mirrors VelloCollisionResponsePlugin, minus GPU parts).
    // No colliders + gravity per `GRAVITY` (default zero) = the floors-removed
    // case; `GRAVITY=on` loads the pin with the same `g` the tuning app uses.
    .insert_resource(VelloConstraintWorld::new(gravity_from_env()))
    .insert_resource(Time::<Fixed>::from_hz(FIXED_HZ))
    .insert_resource(VelloCollisionWorld::default())
    .insert_resource(RemovedColliders::default())
    .insert_resource(CollisionEventBatch::default())
    // Pin mode: the pin owns P1/P2/P3, so the spine drive must stay silent
    // (absent resource = ON, so this must be inserted explicitly).
    .insert_resource(SpineDriveEnabled(false))
    .insert_resource(params_from_env())
    .add_event::<ColliderExternalImpulseEvent>()
    .add_event::<CharacterPivotForceEvent>()
    .add_event::<CharacterPivotVelocityEvent>()
    .add_event::<CharacterPivotImpulseEvent>()
    .add_event::<CharacterAngularConstraintEvent>()
    .add_event::<CharacterFrameForceEvent>()
    .add_event::<VelloRayTraceCommand>()
    .insert_resource(Probe {
        character: None,
        tick: 0,
        dt: 1.0 / FIXED_HZ as f32,
        exited: false,
        script: event_script(),
        event_active: Vec::new(),
        particles: Vec::new(),
        spine_idx: [2, 3, 4],
        prev_vel: Vec::new(),
        filt_acc: Vec::new(),
        primed: false,
        head: None,
        log: vec![],
        events: vec![],
        next_event_id: 0,
        last_event: None,
        max_err: 0.0,
        nan_rows: 0,
    });

    // Real physics stepping, same order as VelloCollisionResponsePlugin minus
    // the GPU tail. `drive_hip_pin` steps the resolver and queues the three
    // constraints immediately before the constraint world is stepped.
    app.add_systems(
        FixedUpdate,
        (
            make_collision_constraints,
            drive_hip_pin,
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
    // Drive the real keyboard resource after bevy's raw-input system (which
    // clears per-frame edges) and before leafwing turns it into `ActionState`.
    app.add_systems(
        PreUpdate,
        drive_scripted_keys
            .after(InputSystem)
            .before(InputManagerSystem::Update),
    );
    // The event ledger and telemetry sample on the **fixed** clock (90 Hz), after
    // the physics chain has written this tick's particle state — so the velocity
    // finite-difference is exact (÷ the true 1/90 s step) and every CSV row is
    // exactly one physics step. The ledger runs before telemetry so each row can
    // stamp the latest event id.
    app.add_systems(
        FixedUpdate,
        (drive_event_script, probe_driver)
            .chain()
            .after(update_collider_from_soft_body),
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

/// Write the scripted window into the real `ButtonInput<KeyCode>` so the whole
/// `keyboard → leafwing ActionState → pin` path is exercised (only edge
/// transitions touch the resource, so held keys stay held and `just_pressed`
/// behaves like a real tap).
fn drive_scripted_keys(
    mut keys: ResMut<ButtonInput<KeyCode>>,
    probe: Res<Probe>,
    mut prev: Local<u8>,
) {
    if probe.character.is_none() {
        return;
    }
    let t = probe.tick as f32 * probe.dt;
    let mut mask = 0u8;
    for &(_, a, b, k, _) in &probe.script {
        if t >= a && t < b {
            mask |= k;
        }
    }
    let codes = [KeyCode::KeyD, KeyCode::KeyA, KeyCode::KeyW];
    for i in 0..3 {
        let bit = 1u8 << i;
        let now = mask & bit != 0;
        let was = *prev & bit != 0;
        if now && !was {
            keys.press(codes[i]);
        } else if !now && was {
            keys.release(codes[i]);
        }
    }
    *prev = mask;
}

/// Record the input timeline as discrete events (one row each) so any slice of
/// the main CSV can be tied to a key event. Runs before `probe_driver`.
fn drive_event_script(mut probe: ResMut<Probe>) {
    if probe.character.is_none() {
        return;
    }
    if probe.event_active.is_empty() {
        probe.event_active = vec![false; probe.script.len()];
    }
    // Advance the deterministic clock once per fixed step, before the telemetry
    // system reads it, so both agree on this tick's `t`.
    probe.tick += 1;
    let t = probe.tick as f32 * probe.dt;
    for i in 0..probe.script.len() {
        let (label, a, b, k, kind) = {
            let e = &probe.script[i];
            (e.0.clone(), e.1, e.2, e.3, e.4)
        };
        let active = t >= a && t < b;
        let log_it = match kind {
            0 => active && !probe.event_active[i], // hold begin
            1 => !active && probe.event_active[i], // hold end
            2 => active && !probe.event_active[i], // edge tap
            _ => false,
        };
        if log_it {
            let id = probe.next_event_id;
            probe.next_event_id += 1;
            probe.last_event = Some(id);
            probe.events.push(format!("{t:.4},{id},{label},{k},{kind}"));
        }
        probe.event_active[i] = active;
    }
}

#[allow(clippy::too_many_arguments)]
fn probe_driver(
    mut commands: Commands,
    mut probe: ResMut<Probe>,
    character_svg: Res<SvgCharacterAssetManager>,
    character_blueprint: Res<BlueprintCharacterAssetManager>,
    conn_q: Query<&Connectivity>,
    pin_q: Query<&HipPin>,
    particle_q: Query<(Entity, &VelloParticle)>,
    root_q: Query<Entity, With<CharacterRoot>>,
    mut exit: EventWriter<AppExit>,
) {
    // Spawn V10 once both assets are parsed (same gate as the tuning app).
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
                    CharacterController {
                        move_vector: Vec2::ZERO,
                        point_vector: Vec2::ZERO,
                        ..Default::default()
                    },
                    PlayerMarker,
                    default_input_map(),
                    ActionState::<PlayerAction>::default(),
                    HipPin::new(&HipPinParams::default().config),
                ))
                .id();
            probe.character = Some(entity);
            info!("V10 spawned, gate clock starts");
        }
        return;
    }

    let Some(character) = probe.character else {
        return;
    };
    let t = probe.tick as f32 * probe.dt;

    // Wait for the pin to seed before emitting any telemetry: before that the
    // pin owns nothing and the frame is meaningless.
    let Ok(pin) = pin_q.get(character) else {
        return;
    };
    if !pin.ready {
        return;
    }
    // Build the tracked-particle list once, discovering **every** particle that
    // belongs to this character (all 13 for V10) via `VelloParticle.root_entity`
    // + `Connectivity.character`, and labelling each from its `Connectivity`
    // name so the header is version-independent and self-describing.
    if probe.particles.is_empty() {
        let mut seen: HashSet<String> = HashSet::new();
        let mut found: Vec<(String, Entity)> = Vec::new();
        for (ent, p) in particle_q.iter() {
            if p.root_entity != character {
                continue;
            }
            let Ok(c) = conn_q.get(ent) else { continue };
            if c.character != character {
                continue;
            }
            let name = c.name.to_string();
            if seen.insert(name.clone()) {
                found.push((name, ent));
            }
        }
        // Deterministic order: spine frame first (PH,P0,P1,P2,P3 → the P1/P2/P3
        // frame lands at indices 2..5), then numeric `Pnn` by id, then the
        // remaining names alphabetically.
        found.sort_by(|a, b| {
            particle_order(&a.0)
                .cmp(&particle_order(&b.0))
                .then_with(|| a.0.cmp(&b.0))
        });
        probe.spine_idx = [
            found.iter().position(|(n, _)| n == "P1").unwrap_or(2),
            found.iter().position(|(n, _)| n == "P2").unwrap_or(3),
            found.iter().position(|(n, _)| n == "P3").unwrap_or(4),
        ];
        probe.particles = found;
        let n = probe.particles.len();
        probe.prev_vel = vec![Vec2::ZERO; n];
        probe.filt_acc = vec![Vec2::ZERO; n];
        let mut head = String::from("t,ev,held");
        for (name, _) in &probe.particles {
            head +=
                &format!(",{name}x,{name}y,{name}vx,{name}vy,{name}ax,{name}ay,{name}fx,{name}fy");
        }
        probe.head = Some(head);
        info!("tracking {} particles", probe.particles.len());
    }

    // --- Kinematics for every tracked particle (world y-up). ---
    let dt = probe.dt;
    let alpha = dt / FILT_TAU; // EMA weight for the smoothed acceleration
    let mut row_s = format!(
        "{t:.4},{},{}",
        probe.last_event.map(|v| v as i64).unwrap_or(-1),
        current_held(&probe, t)
    );

    let mut spine_realized = [Vec2::ZERO; 3];
    let n = probe.particles.len();
    for i in 0..n {
        let (_, ent) = probe.particles[i].clone();
        let Ok((_, p)) = particle_q.get(ent) else {
            continue;
        };
        let pos = p.particle.pos;
        let vel = p.particle.velocity;

        // Acceleration by finite difference of velocity at the fixed dt, plus an
        // EMA-smoothed copy (`f*`) that is far less noisy for post-hoc analysis.
        let (acc, filt) = if probe.primed {
            let acc = (vel - probe.prev_vel[i]) / dt;
            let f = probe.filt_acc[i] + (acc - probe.filt_acc[i]) * alpha;
            (acc, f)
        } else {
            (Vec2::ZERO, Vec2::ZERO)
        };
        probe.prev_vel[i] = vel;
        probe.filt_acc[i] = filt;

        // Spine P1/P2/P3 realized positions, located by name at list-build time.
        if let Some(si) = probe.spine_idx.iter().position(|&j| j == i) {
            spine_realized[si] = pos;
        }

        let wp = vello_to_world(pos);
        let wv = vello_to_world(vel);
        let wa = vello_to_world(acc);
        let wf = vello_to_world(filt);
        row_s += &format!(
            ",{:.2},{:.2},{:.2},{:.2},{:.1},{:.1},{:.1},{:.1}",
            wp.x, wp.y, wv.x, wv.y, wa.x, wa.y, wf.x, wf.y
        );
        let finite = wp.is_finite() && wv.is_finite() && wa.is_finite() && wf.is_finite();
        if !finite {
            probe.nan_rows += 1;
        }
    }
    probe.primed = true;

    // --- Tracking error (realized vs commanded rigid frame, Vello y-down). ---
    let world_delta = Vec2::new(
        pin.state.hip_pos.x - pin.seed_hip.x,
        -(pin.state.hip_pos.y - pin.seed_hip.y),
    );
    let targets = rigid_frame_targets(pin.anchor_p3, pin.rest_offsets, world_delta, pin.theta);
    let mut emax = 0.0f32;
    for i in 0..3 {
        emax = emax.max((spine_realized[i] - targets[i]).length());
    }
    probe.max_err = probe.max_err.max(emax);

    probe.log.push(row_s);

    if t >= EXIT_AT && !probe.exited {
        probe.exited = true;
        let main_path = out_path();
        let ev_path = events_path(&main_path);

        let head = probe
            .head
            .clone()
            .unwrap_or_else(|| "t,ev,held".to_string());
        // Judge the whole run: finiteness + magnitude ceilings, and the rigid
        // `P1/P2/P3` frame must hold its bone lengths (that is the pin's job).
        // The hash fingerprints the run for determinism replay.
        let inv = check_invariants(&head, &probe.log, &InvariantConfig::default());
        let bones = [
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
        let rig = check_bone_rigidity(&head, &probe.log, &bones, PIN_RIGID_TOL);
        let hash = rows_hash(&head, &probe.log);
        let worst_rel = rig.max_rel_dev.iter().copied().fold(0.0f32, f32::max);
        // Per-bone detail with the tick where each peaked — so a failure points
        // at the offending event rather than just a magnitude.
        let bone_names = ["P1-P2", "P2-P3", "P1-P3"];
        let mut rig_detail = String::new();
        for (i, nm) in bone_names.iter().enumerate() {
            let t = probe
                .log
                .get(rig.worst_row[i])
                .and_then(|r| r.split(',').next())
                .unwrap_or("?");
            rig_detail += &format!(" {nm}={:.4}@t{t}", rig.max_rel_dev[i]);
        }

        let mut out = String::from(head + "\n");
        out += &(probe.log.join("\n") + "\n");

        let mut ev_out = String::from("t,event_id,label,keys,kind\n");
        if !probe.events.is_empty() {
            ev_out += &(probe.events.join("\n") + "\n");
        }

        let write = |path: &str, data: String| {
            if let Some(dir) = std::path::Path::new(path).parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            match std::fs::write(path, data) {
                Ok(_) => info!("wrote {path}"),
                Err(e) => error!("write failed ({path}): {e}"),
            }
        };
        write(&main_path, out);
        write(&ev_path, ev_out);

        info!(
            "summary: rows={} events={} nan_rows={} max_track_err={:.3}px inv={} rig={} max_rel_dev={:.4} hash={:016x}",
            probe.log.len(),
            probe.next_event_id,
            probe.nan_rows,
            probe.max_err,
            if inv.ok { "ok".to_string() } else { inv.detail.clone() },
            if rig.ok { "ok".to_string() } else { "FAIL".to_string() },
            worst_rel,
            hash,
        );
        info!("rigidity:{}", rig_detail);
        let _ = root_q.iter().count(); // keep the query live for parity with the demo
        exit.write(AppExit::Success);
    }
}

/// Current held key mask (bits 1=Right 2=Left 4=Jump), from the script windows.
fn current_held(probe: &Probe, t: f32) -> u8 {
    let mut mask = 0u8;
    for &(_, a, b, k, _) in &probe.script {
        if t >= a && t < b {
            mask |= k;
        }
    }
    mask
}
