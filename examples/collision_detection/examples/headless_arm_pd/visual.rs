//! Visual (rendered) twin of the headless command-driven character harness.
//!
//! Same [`Probe`], same command-script driver, same 90 Hz XPBD physics and
//! telemetry — but booted under `DefaultPlugins` + `VelloPlugin` so the
//! character (which [`drive_script`] already spawns with a `VelloSceneBundle`)
//! is actually rendered, and [`VelloCollisionResponsePlugin`] supplies the full
//! GPU collision chain from the **shared** vello renderer instead of the manual
//! chain the headless bin wires.
//!
//! This is a separate example *binary* that declares the exact same `mod`
//! blocks as the headless [`main.rs`](./main.rs) (assets/collision/command/
//! config/dispatch/math/probe/report/sample). Each binary is its own crate, so
//! the harness brain is compiled once per binary with **zero duplication** and
//! byte-identical `crate::` resolution — no lib refactor, no path churn.
//!
//! Run:
//! ```text
//! cargo run -p collision_detection --example visual_arm_pd --release
//! ```
//!
//! All headless env levers apply (`LIMB`, `SCRIPT`, `KP`, `KD`, `GRAVITY`,
//! `HIPPIN`, `OUT_CSV`, `HIT`, `SUBSTEPS`, ...). Collision is always on here
//! (that is the point — watch the character hit the floor and walk). The run
//! ends with `AppExit` exactly like the headless bin once the summary CSV is
//! written, so the window closes when the script finishes.
//!
//! `#![allow(dead_code)]` because each binary uses a subset of the shared
//! module surface (e.g. `setup_headless_collision` is headless-only; the
//! modules themselves are the same files).

#![allow(dead_code)]

mod assets;
mod collision;
mod command;
mod config;
mod dispatch;
mod math;
mod probe;
mod report;
mod sample;

// Reuse the tuning app's edge-pan camera verbatim instead of duplicating it.
#[path = "../../src/edge_pan_camera.rs"]
mod edge_pan_camera;

use bevy::asset::{AssetMetaCheck, AssetPlugin};
use bevy::input::InputSystem;
use bevy::prelude::*;
use bevy::window::PresentMode;

use leafwing_input_manager::plugin::InputManagerSystem;

use bevy_vello::collision::VelloCollisionWorld;
use bevy_vello::integrations::physics::systems::{
    apply_explicit_impulse_on_connection_particle, run_gpu_collision, update_constraint_world,
};
use bevy_vello::integrations::physics::VelloConstraintWorld;
use bevy_vello::vello::{kurbo, peniko};
use bevy_vello::{prelude::*, VelloCollisionResponsePlugin, VelloPlugin};

use game_lib::{
    capture_telemetry, drive_hip_pin, tick_loop3_angular, update_character_movement,
    update_ground_support, GroundSupport, HeadlessTelemetryPlugin, SpineDriveEnabled,
    VelloCharacterPlugin,
};

use crate::command::{jumpwalk_window, load_script, Command, LimbId};
use crate::config::{
    asset_root, default_grid, gravity_from_env, hip_pin_params_from_env, parse_grid, FIXED_HZ,
};
use crate::math::{env_f32, env_flag};
use crate::probe::{Probe, ProbeArgs};

use crate::assets::{mark_assets_loaded, register_character_assets};
use crate::collision::{inject_test_hit, log_collision_stats, spawn_collision_floor};
use crate::dispatch::drive_script;
use crate::report::drive_body_keys;
use crate::sample::{limb_finalize, limb_sample, telemetry_active};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    let focus = LimbId::from_env();
    let (script, script_src) = load_script(focus);

    // Whole-body mode: the script drives `move`/`jump` (or `HIPPIN` forces it).
    // Mirrors the headless bin exactly.
    let body = env_flag("HIPPIN")
        || script
            .iter()
            .any(|c| matches!(c, Command::Move(_) | Command::Jump(_)));
    let jw_window = if body { jumpwalk_window(&script) } else { None };
    let sweep = env_flag("SWEEP") && !body;

    // Distinct default stem from the headless bin so the rendered run's
    // telemetry (main CSV + .summary.csv + .events.csv) does not clobber the
    // headless output — the whole point is diffing the two for divergence.
    let csv_path = std::env::var("OUT_CSV")
        .unwrap_or_else(|_| "/home/ubuntu/bender/tmp/headless/visual_limb_pd_log.csv".to_string());

    let grid: Vec<(f32, f32)> = match std::env::var("GRID") {
        Ok(g) if !g.trim().is_empty() => parse_grid(&g),
        _ if sweep => default_grid(),
        _ => vec![(env_f32("KP", 0.5), env_f32("KD", 0.065))],
    };
    let (kp0, kd0) = grid.first().copied().unwrap_or((0.5, 0.065));

    info!(
        "visual harness: focus={} script={} ({} command(s)), {} cell(s), sweep={}, body={}",
        focus.label(),
        script_src,
        script.len(),
        grid.len(),
        sweep,
        body
    );

    let probe = Probe::new(ProbeArgs {
        focus,
        script,
        script_src,
        body,
        sweep,
        grid,
        kp: kp0,
        kd: kd0,
        csv_path,
        target_dist: env_f32("TARGET_DIST", 0.0),
        jw_window,
    });

    let gravity = if body { gravity_from_env() } else { Vec2::ZERO };
    let substeps = std::env::var("SUBSTEPS")
        .ok()
        .and_then(|s| s.trim().parse::<u32>().ok())
        .unwrap_or(2)
        .max(1);

    let mut app = App::default();
    app.insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Vello Character Harness (Visual)".into(),
                        // Best for Linux/NVIDIA (same as the tuning app).
                        present_mode: PresentMode::AutoNoVsync,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    file_path: asset_root().into(),
                    ..default()
                })
                .set(bevy::log::LogPlugin { ..default() }),
        )
        .add_plugins(VelloCharacterPlugin)
        .add_plugins(HeadlessTelemetryPlugin)
        // Rendering + collision: `VelloPlugin` → `VelloRenderPlugin::finish`
        // creates the ONE shared vello::Renderer used by both the render graph
        // and the GPU narrow phase (`GpuCollisionRunner`), so no separate
        // headless wgpu device is needed.
        .add_plugins(VelloPlugin)
        // Full physics + GPU collision chain at 90 Hz FixedUpdate (with the
        // PostUpdate collision-event redistribution) — this replaces the
        // manual chain the headless bin wires.
        .add_plugins(VelloCollisionResponsePlugin)
        // The plugin boots a zero-gravity world; override with the harness
        // gravity (vello frame: x right, y down).
        .insert_resource(VelloConstraintWorld::new(gravity))
        .insert_resource(Time::<Fixed>::from_hz(FIXED_HZ))
        .insert_resource({
            // Match the headless harness: 2 XPBD substeps by default.
            let mut world = VelloCollisionWorld::default();
            world.substeps = substeps;
            world
        })
        // Ground gate for the pin's vertical authority (same as the harness).
        .insert_resource(GroundSupport::default())
        // The spine drive must stay silent (the pin owns P1/P2/P3 in body mode;
        // the frame is passive in limb mode).
        .insert_resource(SpineDriveEnabled(false))
        .insert_resource(probe)
        .add_systems(
            Startup,
            (register_character_assets, setup_camera, add_scene),
        )
        .add_systems(Update, mark_assets_loaded);

    if body {
        // Self-identify the run: `LEG_COMPLIANCE` is env-only (the asset's leg
        // angular compliance is overwritten every frame by the pin) and it is
        // the difference between walking forwards (1e-6) and walking backwards
        // (1e-9), so print what this process actually got.
        let params = hip_pin_params_from_env();
        info!(
            "walk params: leg_compliance={:.1e} leg_zeta={:.2} leg_alpha={:.2} \
             target_speed={:.0}px/s external_pin={} attitude={}",
            params.leg_compliance,
            params.leg_zeta,
            params.leg_alpha,
            params.config.target_speed,
            params.external_pin,
            params.attitude_enabled,
        );
        app.insert_resource(params)
            .add_systems(FixedUpdate, update_ground_support.before(drive_hip_pin))
            .add_systems(
                FixedUpdate,
                drive_hip_pin
                    .before(update_character_movement)
                    .before(update_constraint_world),
            )
            .add_systems(
                FixedUpdate,
                tick_loop3_angular
                    .after(drive_hip_pin)
                    .before(update_constraint_world),
            );
        if std::env::var("HIT").is_ok() {
            app.add_systems(
                FixedUpdate,
                inject_test_hit.before(apply_explicit_impulse_on_connection_particle),
            );
        }
    }

    // The static floor at the resolver foot line (self-gates on the pin). The
    // broad phase + GPU narrow phase already run inside the plugin's chain, so
    // the floor only needs to spawn before the GPU collision picks it up.
    app.add_systems(FixedUpdate, spawn_collision_floor.before(run_gpu_collision));

    // Apply the current script command in `FixedPreUpdate` (before the
    // controller reads the target this same fixed step).
    app.add_systems(FixedPreUpdate, drive_script);
    // Body mode: translate the active `move`/`jump` command into the real
    // `ButtonInput` — after raw input, before leafwing's `ActionState`.
    if body {
        app.add_systems(
            PreUpdate,
            drive_body_keys
                .after(InputSystem)
                .before(InputManagerSystem::Update),
        );
    }
    // Sample after the whole fixed step, append via the shared telemetry
    // plugin, then finalize the cell. `limb_finalize` writes the summary and
    // emits `AppExit` (window closes) when the run is done.
    app.add_systems(
        FixedPostUpdate,
        (limb_sample, capture_telemetry, limb_finalize)
            .chain()
            .run_if(telemetry_active),
    );
    // One contact-stat line per sim-second (same as the headless bin).
    app.add_systems(FixedPostUpdate, log_collision_stats);
    // Pan/zoom camera controls.
    app.add_systems(Update, edge_pan_camera::update_edge_pan_camera);

    app.run();
}

/// 2D camera with edge-pan zoom (mirrors `character_tuning_main::setup_back_ground`).
fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera2d::default(),
        edge_pan_camera::EdgePanCamera {
            edge_margin: -10.0,
            ..Default::default()
        },
    ));
}

/// Dark background plane + a point light so the rendered character is visible.
fn add_scene(mut commands: Commands) {
    let mut scene: VelloScene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgb(0.12, 0.12, 0.15),
        None,
        &kurbo::Rect::new(-4096.0, -4096.0, 4096.0, 4096.0),
    );
    commands.spawn((VelloSceneBundle {
        scene,
        ..Default::default()
    },));

    // Point light (same as `character_tuning_main::add_light`).
    let mut light_scene: VelloScene = VelloScene::default();
    let light_radius = 800.0;
    light_scene.push_point_light(
        kurbo::Affine::scale(light_radius * 2.0),
        &[1.0, 1.0, 1.0],
        200.0 / (light_radius as f32),
    );
    commands.spawn((VelloSceneBundle {
        scene: light_scene,
        ..Default::default()
    },));
}
