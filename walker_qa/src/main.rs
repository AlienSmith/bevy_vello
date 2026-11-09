//! Minimal walker scene: the existing character-assembly + physics + render
//! plugins, a static floor, the v10 character spawned on top of it, and the
//! `walker_control` plugin attached to the root once the rig is assembled.
//!
//! The observer (reality) and actuator (constraints) are wired to the character
//! by [`WalkerControlPlugin`]; the keyboard only writes [`WalkerIntent`]. Run
//! with:
//!
//! ```sh
//! cargo run -p walker_qa
//! ```
//!
//! This opens a window like the tuning app. The character stands — the walking
//! controller is a stub until the gait seam lands (plan §12).

use bevy::{asset::AssetMetaCheck, input::keyboard::KeyCode, prelude::*, window::PresentMode};
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
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
    },
    flush_rows, CharacterRoot, Connectivity, LeftLegController, RightLegController,
    SpineController, SpineDriveEnabled, VelloCharacterPlugin,
};
use walker_control::{
    emit::LegHandles,
    observe::ObserveHandles,
    plugin::{
        AttachWalker, ScriptResource, WalkerControlPlugin, WalkerIntent, WalkerTelemetryBuffer,
    },
    script::ScriptTrack,
    walker::{Walker, WalkerConfig},
};

/// Character asset ids (must match `CharacterRoot` and the blueprint controller
/// bindings: `left_leg`/`right_leg` particles + `PLLL`/`PRLL` colliders).
const SVG_ASSET: &str = "v10.character.svg";
const BP_ASSET: &str = "v10.character.json";
const SVG_PATH: &str = "character/v10.character.svg";
const BP_PATH: &str = "character/v10.character.json";

/// Gravity magnitude (px/s²), Vello y-down. 100 px = 1 m ⇒ Earth-like. The
/// fixed clock (90 Hz) is owned by `VelloCollisionResponsePlugin` itself.
const GRAVITY: Vec2 = Vec2::new(0.0, 980.0);

/// Spawn pose. The root's scale is baked into the particle positions by
/// `assemble_character`, so the hip lands near world origin. The y-lift
/// matters: the foot collider wraps the knee→foot segment and reaches ~10 px
/// BELOW the foot particle, so spawning flush starts the colliders inside the
/// floor and the XPBD penetration kick topples the body. A small lift clears
/// that pre-penetration without a big impact speed; [`StandingCtrl`] spreads
/// the feet throughout the drop.
const SPAWN_TRANSLATION: Vec3 = Vec3::new(0.0, -30.0, 100.0);
const SPAWN_SCALE: Vec3 = Vec3::new(0.5, 0.5, 1.0);

/// Floor surface line in physics space (y-down, below the origin). The v10
/// seed pose at scale 0.5 puts the foot particles near y = +310; with the
/// 30 px spawn lift the colliders clear the floor on spawn and land after a
/// short drop. Collider Transforms are in Bevy y-up space: the floor spawns
/// at y = −(this + half thickness).
const FOOT_SURFACE_Y: f32 = 312.0;
/// The floor collider's half thickness (40 px rect, same as the harness).
const FLOOR_HALF_THICKNESS: f32 = 20.0;
/// Half-length of the floor rect (4800 px total).
const FLOOR_HALF_LEN: f64 = 2400.0;

/// Commanded speed for a held direction key (px/s).
const WALK_SPEED: f32 = 120.0;

/// Commanded speed when the boost key (Shift) is held with a direction.
const BOOST_SPEED: f32 = 120.0;

/// Where the telemetry CSV lands (relative to the working directory; the
/// repo's QA tools read e.g. `python3 tools/physics-qa/plot_p3.py
/// tmp/walker_qa/walk.csv out.png`).
const CSV_PATH: &str = "tmp/walker_qa/walk.csv";
/// Flush cadence: rewrite the whole buffer every ~1 s of ticks (90 Hz). The
/// app has no clean teardown hook in a windowed run, so up to the last flush
/// interval of rows may be dropped at close — acceptable for QA, and the
/// buffer stays whole while the app runs.
const FLUSH_EVERY_ROWS: usize = 90;

/// `WIN_HIDDEN=1` runs the full pipeline (render + physics + walker) with no
/// OS window — the same idiom as `visual_arm_pd`. Only the surface is
/// suppressed; every system still runs on the fixed clock.
fn win_hidden() -> bool {
    std::env::var("WIN_HIDDEN")
        .map(|v| !v.trim().is_empty() && v != "0")
        .unwrap_or(false)
}

fn main() {
    let hidden = win_hidden();
    App::default()
        .insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Walker QA".into(),
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
        // Existing character assembly + factories + input.
        .add_plugins(VelloCharacterPlugin::default())
        // The walk control: AttachWalker observer + the
        // Input→Observe→Control→Actuate→Assemble chain (before the solver).
        .add_plugins(WalkerControlPlugin)
        // Optional scripted command track: `SCRIPT=path.json` drives the state
        // machine through the JSON cells (boost/move/wait). Absent ⇒ keyboard.
        .init_resource::<ScriptSlot>()
        // Physics + collision (owns the XPBD step, GPU collision, 90 Hz clock).
        .add_plugins(VelloPlugin)
        .add_plugins(VelloCollisionResponsePlugin)
        // The walker drives the legs; the game-side spine drive must stay off so
        // it cannot fight the actuator (same rule as the hip-pin tuning app).
        .insert_resource(SpineDriveEnabled(false))
        .init_state::<WalkerState>()
        .add_systems(Startup, (setup_scene, apply_gravity, pause_physics).chain())
        .add_systems(
            Update,
            (
                register_character_assets,
                mark_assets_loaded,
                check_character_loaded.run_if(in_state(WalkerState::Loading)),
                attach_walker,
                install_script,
                keyboard_intent,
                flush_telemetry,
                script_exit,
                camera_follow,
            ),
        )
        .add_systems(OnEnter(WalkerState::Game), spawn_character)
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

/// Optional env override of an f32 (empty/invalid ⇒ `None`).
fn env_f32(key: &str) -> Option<f32> {
    std::env::var(key)
        .ok()
        .filter(|v| !v.trim().is_empty())
        .and_then(|v| v.trim().parse::<f32>().ok())
}

/// Tune the boost shape from env so a lean sweep can test how much stored
/// fall-energy the handover inherits. Unknown keys ignored.
fn apply_control_env(cfg: &mut walker_control::control::ControlConfig) {
    if let Some(v) = env_f32("BOOST_LEAN") {
        cfg.boost_lean = v;
    }
    if let Some(v) = env_f32("BOOST_GATHER_TS") {
        cfg.boost_gather_ts = v;
    }
    if let Some(v) = env_f32("ATT_KP") {
        cfg.att_closure_kp = v;
    }
    if let Some(v) = env_f32("ATT_KD") {
        cfg.att_closure_kd = v;
    }
}

/// Tune the actuator stiffness from env so a sweep can test whether a
/// commanded foot tracks (vs. the leg simply folding). Unknown keys ignored.
fn apply_actuator_env(cfg: &mut walker_control::actuate::ActuatorConfig) {
    if let Some(v) = env_f32("HIP_COMPLIANCE") {
        cfg.hip_compliance = v;
    }
    if let Some(v) = env_f32("KNEE_COMPLIANCE") {
        cfg.knee_compliance = v;
    }
    if let Some(v) = env_f32("ZETA") {
        cfg.zeta = v;
    }
    if let Some(v) = env_f32("STRUT_ACTIVE") {
        cfg.strut_active = v;
    }
}

#[derive(States, Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
enum WalkerState {
    #[default]
    Loading,
    Game,
}

/// The physics world boots at zero gravity; open it.
fn apply_gravity(mut world: ResMut<VelloConstraintWorld>) {
    world.set_gravity(GRAVITY);
}

/// Freeze the XPBD step until the walker is attached. Assembly takes a handful
/// of frames; without the pause the body falls (and the CSV's first row is a
/// few ticks into the fall) before the actuator loop ever opens. With it, the
/// very first physics tick is the first walker tick — row 1 is the spawn pose.
fn pause_physics(mut collision: ResMut<VelloCollisionWorld>) {
    collision.paused = true;
}

/// Marker: the script (if any) has been installed (or declined).
#[derive(Resource, Default)]
struct ScriptSlot {
    decided: bool,
}

/// Install the `SCRIPT=path.json` command track once, before any walker tick
/// needs it. An unreadable/broken script logs and falls back to keyboard.
fn install_script(mut commands: Commands, mut slot: ResMut<ScriptSlot>) {
    if slot.decided {
        return;
    }
    slot.decided = true;
    let Some(path) = std::env::var("SCRIPT")
        .ok()
        .filter(|p| !p.trim().is_empty())
    else {
        info!("walker_qa: no SCRIPT set; keyboard intent drives the walker");
        return;
    };
    match ScriptTrack::from_path(&path) {
        Ok(track) => {
            info!("walker_qa: script '{}' loaded", path);
            commands.insert_resource(ScriptResource(track));
        }
        Err(e) => {
            warn!("walker_qa: SCRIPT '{e}'; falling back to keyboard intent");
        }
    }
}

fn setup_scene(mut commands: Commands) {
    commands.spawn((
        Camera2d::default(),
        CamFollow {
            pos: Vec2::ZERO,
            vel: Vec2::ZERO,
            live: false,
        },
    ));

    // Static floor: a 40 px-thick rect whose TOP surface sits on the live foot
    // line. Collider transforms are Bevy y-up, so "below the feet" (physics
    // y-down, larger y) = more negative Bevy y; the center is one
    // half-thickness below the surface (same convention as the headless harness).
    let floor_y = -(FOOT_SURFACE_Y + FLOOR_HALF_THICKNESS);
    let rect = Rect::new(
        -FLOOR_HALF_LEN,
        -(FLOOR_HALF_THICKNESS as f64),
        FLOOR_HALF_LEN,
        FLOOR_HALF_THICKNESS as f64,
    );
    let shape = path_to_ccw_quad_path(&rect.to_path(0.1));
    let transform = Transform::from_xyz(0.0, floor_y, 0.0);

    // White backdrop at identity (covers the whole viewport).
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

    // The collider alone is invisible: `visualize_colliders` repaints the
    // `VelloScene` of entities that carry BOTH a scene and a collider, and a
    // scene-less collider entity is never drawn. The tuning app's static shapes
    // co-spawn a scene authored in the collider's LOCAL frame (the same `rect`);
    // do the same here so the engine paints this slab (red debug brush) every
    // frame, and author one identical fill so frame 0 shows it too.
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
    info!("floor: surface y={FOOT_SURFACE_Y:.0}, center y={floor_y:.0}");
}

/// Spawn the character root; the assembly factories do the rest (particles,
/// joints, colliders are inserted `Added` and picked up by the physics plugin).
fn spawn_character(mut commands: Commands) {
    commands.spawn((
        VelloSceneBundle {
            transform: Transform {
                translation: SPAWN_TRANSLATION,
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
        WalkerIntent::default(),
    ));
    info!("character root spawned; waiting for assembly");
}

// ---------------------------------------------------------------------------
// Assets
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
    mut next: ResMut<NextState<WalkerState>>,
) {
    if svg.all_loaded() && bp.all_loaded() {
        next.set(WalkerState::Game);
    }
}

// ---------------------------------------------------------------------------
// Attach
// ---------------------------------------------------------------------------

/// When a freshly assembled character has both leg controllers (the blueprint
/// declares them), fire [`AttachWalker`] so the plugin inserts the component
/// pair. The leg controllers' `[P2, P3, knee, foot]` particle arrays map
/// straight onto [`LegHandles`]; the foot *collider* entities are found by
/// `Connectivity` name. Runs once per character (tracked in `attached`).
fn attach_walker(
    mut commands: Commands,
    mut collision: ResMut<VelloCollisionWorld>,
    roots: Query<(
        Entity,
        &SpineController,
        &LeftLegController,
        &RightLegController,
    )>,
    colliders: Query<(Entity, &Connectivity, &VelloCollider)>,
    walker_q: Query<(), With<walker_control::walker::Walker>>,
    mut attached: Local<std::collections::HashSet<Entity>>,
) {
    for (root, _spine, left, right) in &roots {
        if attached.contains(&root) || walker_q.get(root).is_ok() {
            continue;
        }
        // Foot collider entities, keyed by the blueprint's PLLL/PRLL names.
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
            // Feet not assembled yet — retry next frame.
            continue;
        };
        let handles = ObserveHandles {
            hip: left.particles[1],
            waist: left.particles[0],
            feet: [left.particles[3], right.particles[3]],
            foot_colliders: [plll, prll],
        };
        // The two controllers are distinct component types, so build each leg's
        // handles explicitly rather than through one shared closure.
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
        // QA stiffness override: the default actuator compliance lets the
        // stance collapse under the body weight (measured: hip sinks to the
        // floor while feet hover ~60 px above it). Env levers let a sweep
        // tighten the leg/strut until a commanded foot actually tracks.
        let mut config = WalkerConfig::default();
        apply_actuator_env(&mut config.actuator);
        apply_control_env(&mut config.control);
        // Rig geometry for the gait math (v10 at scale 0.5): the bones
        // measure ~55 px (matching walker_math's defaults), but the STANDING
        // hip height is the measured hip→floor distance (~104 px: hip y≈203,
        // foot line ≈307), not the sandbox's crouched 90. The walking
        // controller's floor line is anchored from `hip_height`, so this must
        // match the rig or the commanded feet float above the ground.
        config.lower.hip_height = 104.0;
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
        attached.insert(root);
        // Rig is complete — let physics run from the frozen spawn pose.
        collision.paused = false;
        info!("attached walker to character {root:?}");
    }
}

/// Write the telemetry CSV to disk. The buffer's header is the full legacy
/// format (`legacy_header()`), so the file loads directly in the existing
/// analysis tools.
fn flush_telemetry(buf: Res<WalkerTelemetryBuffer>, mut last: Local<usize>) {
    if buf.rows.len() >= *last + FLUSH_EVERY_ROWS {
        flush_rows(CSV_PATH, &buf.header, &buf.rows);
        *last = buf.rows.len();
    }
}

/// The scripted run ends when the command track runs out: flush every row,
/// then close the app. Without a script this system never fires (the keyboard
/// run is interactive until the window closes).
fn script_exit(
    script: Option<Res<ScriptResource>>,
    buf: Res<WalkerTelemetryBuffer>,
    mut exit: EventWriter<AppExit>,
    mut done: Local<bool>,
) {
    if *done {
        return;
    }
    let Some(script) = script else { return };
    if script.0.done() {
        *done = true;
        flush_rows(CSV_PATH, &buf.header, &buf.rows);
        info!("walker_qa: script finished — exiting");
        exit.write(AppExit::Success);
    }
}

/// Key → intent. No keys ⇒ no command (standing); a direction key holds a
/// move, and holding Shift with it marks the move as a boost launch. This
/// path is inert while a `ScriptResource` is installed (the plugin gates it).
fn keyboard_intent(keys: Res<ButtonInput<KeyCode>>, mut intents: Query<&mut WalkerIntent>) {
    for mut intent in &mut intents {
        let mut dir = Vec2::ZERO;
        if keys.pressed(KeyCode::ArrowRight) || keys.pressed(KeyCode::KeyD) {
            dir.x += 1.0;
        }
        if keys.pressed(KeyCode::ArrowLeft) || keys.pressed(KeyCode::KeyA) {
            dir.x -= 1.0;
        }
        let boost = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
        intent.dir = dir;
        intent.speed = if dir.length() > 0.0 {
            if boost {
                BOOST_SPEED
            } else {
                WALK_SPEED
            }
        } else {
            0.0
        };
        intent.boost = boost && dir.length() > 0.0;
    }
}

/// Spring-follow camera state: position + velocity in Bevy world coords
/// (y-up; the walker's measured hip lives in physics y-down, negated here).
#[derive(Component)]
struct CamFollow {
    pos: Vec2,
    vel: Vec2,
    live: bool,
}

/// Keep the walker on screen: critically damped spring toward the measured
/// hip (`fiction().hip`, the same observation the controller reads). `live`
/// snaps the camera to the target on the first frame a Walker exists (the
/// spawn pose), then springs: `a = k·(target−pos) − 2√k·vel` settles in
/// ~0.3 s without overshoot, so a boost start pans smoothly instead of
/// racing the character off the window edge.
fn camera_follow(
    mut cam_q: Query<(&mut CamFollow, &mut Transform)>,
    walkers: Query<&Walker>,
    time: Res<Time>,
) {
    let Some(walker) = walkers.iter().next() else {
        return;
    };
    // Physics y-down → Bevy y-up (what the render pass shows for the particle).
    let hip = walker.fiction().hip;
    let target = Vec2::new(hip.x, -hip.y);
    let dt = time.delta_secs().min(1.0 / 30.0);
    for (mut cam, mut transform) in cam_q.iter_mut() {
        if !cam.live {
            cam.pos = target;
            cam.vel = Vec2::ZERO;
            cam.live = true;
        } else {
            let k = 12.0f32; // spring stiffness (1/s²); damping = critical
            let (p0, v0) = (cam.pos, cam.vel);
            let accel = (target - p0) * k - v0 * (2.0 * k.sqrt());
            let v1 = v0 + accel * dt;
            cam.pos = p0 + v1 * dt;
            cam.vel = v1;
        }
        transform.translation.x = cam.pos.x;
        transform.translation.y = cam.pos.y;
    }
}
