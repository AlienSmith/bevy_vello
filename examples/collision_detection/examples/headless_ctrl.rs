//! Headless controller probe.
//!
//! Runs the REAL game logic without a window or GPU:
//! - character assembly from V8.character.svg + v8.character.json (game_lib)
//! - the real controller (`update_character_movement`, PostUpdate)
//! - the real XPBD connection physics (FixedUpdate @ 90 Hz)
//!
//! Only the GPU-bound tail of the physics chain (`run_broad_phase`,
//! `run_gpu_collision`, raytrace) is omitted — this test walks the character
//! through empty space, so collision detection would produce no contacts.
//!
//! Phases: 0–2 s baseline (no input) → 2–4.4 s `move_vector = (50, 0)`
//! (equivalent of holding D) → 4.4–6.5 s release. Logs spine-particle
//! positions/velocities to CSV every 50 ms, then exits.

use std::time::Duration;

use bevy::{
    app::ScheduleRunnerPlugin,
    asset::{AssetMetaCheck, AssetPlugin},
    prelude::*,
    MinimalPlugins,
};

use bevy_vello::{
    collision::{
        CollisionEventBatch, RemovedColliders, VelloCollider, VelloCollisionWorld,
        VelloRayTraceCommand,
    },
    integrations::physics::{
        systems::{
            apply_explicit_impulse_on_connection_particle, apply_explicit_impulse_on_softbody,
            generate_connection, generate_soft_body_for_collider, make_collision_constraints,
            remove_soft_body, update_collider_from_soft_body, update_connection_particles,
            update_constraint_world,
        },
        CharacterAngularConstraintEvent, CharacterFrameForceEvent, CharacterPivotForceEvent,
        CharacterPivotImpulseEvent, CharacterPivotPositionEvent, CharacterPivotVelocityEvent,
        ColliderExternalImpulseEvent, VelloConstraintWorld, VelloParticle,
    },
    prelude::*,
};
use game_lib::{
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
    },
    CharacterController, CharacterRoot, SpineController, VelloCharacterPlugin,
};

const D_INPUT: Vec2 = Vec2::new(50.0, 0.0); // what read_player_input writes for KeyD
const BASELINE_END: f32 = 2.0;
const LOG_PERIOD: f32 = 0.05;

/// Instability scenarios (`SCENARIO` env):
/// - `walk` (default): hold D 2.4 s — perpendicular walk in the spawn facing
///   (character spawns facing up; D is a 90° turn-then-walk case).
/// - `up`:    hold W 4 s — move_vector (0, 50): ALIGNED walk (straight-line
///   quality test, requirement 1).
/// - `down`:  hold S 8 s — move_vector (0, -50): 180° turn-then-walk
///   (requirement 2: turn within a small travel distance).
/// - `turn`:  hold A 6 s — 180° alignment command.
/// - `tap`:   single 0.3 s D press, then 5 s of observation.
fn scenario() -> (Vec2, f32, f32, f32) {
    // (input vector, input_end, exit_at, log_period)
    match std::env::var("SCENARIO").as_deref() {
        Ok("up") => (Vec2::new(0.0, 50.0), 6.0, 8.0, 0.02),
        Ok("down") => (Vec2::new(0.0, -50.0), 8.0, 10.0, 0.02),
        Ok("turn") => (Vec2::new(-50.0, 0.0), 8.0, 10.0, 0.02),
        Ok("tap") => (Vec2::new(50.0, 0.0), 2.3, 8.0, 0.02),
        _ => (D_INPUT, 4.4, 6.5, 0.05),
    }
}

fn out_path() -> String {
    std::env::var("OUT_CSV").unwrap_or_else(|_| "/tmp/x11get/headless/ctrl_log.csv".to_string())
}

#[derive(Resource)]
struct Probe {
    character: Option<Entity>,
    t0: f32,
    log: Vec<String>,
    next_log: f32,
    exited: bool,
    input_vec: Vec2,
    input_end: f32,
    exit_at: f32,
    log_period: f32,
}

fn main() {
    // Frame rate is a first-class experimental variable: the controller runs
    // per rendered frame while physics is fixed 90 Hz, so the pump/beat
    // regime depends on it. PROBE_FPS overrides the default 60.
    let fps: f64 = std::env::var("PROBE_FPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(60.0);
    let mut app = App::default();
    app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
        1.0 / fps, // controller runs per app update; override with PROBE_FPS
    ))))
        .add_plugins(bevy::log::LogPlugin::default())
        .add_plugins(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            file_path: "/home/ubuntu/bender/bevy_vello/examples/collision_detection/assets".into(),
            ..default()
        })
        .add_plugins(bevy::input::InputPlugin)
        .add_plugins(VelloCharacterPlugin)
        // Physics resources (mirrors VelloCollisionResponsePlugin, minus GPU parts)
        .insert_resource(VelloConstraintWorld::new(Vec2::new(0.0, 0.0)))
        .insert_resource(Time::<Fixed>::from_hz(90.0))
        .insert_resource(VelloCollisionWorld::default())
        .insert_resource(RemovedColliders::default())
        .insert_resource(CollisionEventBatch::default())
        .add_event::<ColliderExternalImpulseEvent>()
        .add_event::<CharacterPivotForceEvent>()
        .add_event::<CharacterPivotVelocityEvent>()
        .add_event::<CharacterPivotImpulseEvent>()
        .add_event::<CharacterAngularConstraintEvent>()
        .add_event::<CharacterPivotPositionEvent>()
        .add_event::<CharacterFrameForceEvent>()
        .add_event::<VelloRayTraceCommand>()
        .insert_resource(Probe {
            character: None,
            t0: 0.0,
            log: vec![],
            next_log: 0.0,
            exited: false,
            input_vec: scenario().0,
            input_end: scenario().1,
            exit_at: scenario().2,
            log_period: scenario().3,
        });

    // Real physics stepping, same order as VelloCollisionResponsePlugin,
    // minus run_broad_phase / run_gpu_collision / raytrace.
    app.add_systems(
        FixedUpdate,
        (
            make_collision_constraints,
            synth_hit_inject,
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
    app.add_systems(Update, (mark_assets_loaded, probe_driver));
    app.run();
}

fn register_character_assets(
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
    asset_server: Res<AssetServer>,
) {
    character_svg.push(
        asset_server.load("character/V8.character.svg"),
        SvgCharacterAssetMetaData::default(),
        "V8.character.svg",
    );
    character_blueprint.push(
        asset_server.load("character/v8.character.json"),
        BlueprintCharacterAssetMetaData::default(),
        "v8.character.json",
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

/// Test-support: inject one synthetic collision hit (no GPU needed) so the
/// collision→skeleton channel can be exercised headless.
/// Env `SYNTH_HIT="t,px,py,vx,vy"` — fires once when the fixed-clock time
/// reaches `t` (seconds from app start), on the first-spawned body part.
/// `px,py` is a position offset in px; `vx,vy` is a velocity in px/s,
/// converted internally to the channel's px-per-tick units (÷ fixed Hz).
fn synth_hit_inject(
    time: Res<Time>,
    fixed_time: Res<Time<Fixed>>,
    mut world: ResMut<VelloConstraintWorld>,
    mut fired: Local<bool>,
) {
    if *fired {
        return;
    }
    let Some(spec) = std::env::var("SYNTH_HIT").ok().filter(|s| !s.is_empty()) else {
        return;
    };
    let parts: Vec<f32> = spec.split(',').filter_map(|s| s.trim().parse().ok()).collect();
    if parts.len() != 5 {
        error!("SYNTH_HIT must be \"t,px,py,vx,vy\"");
        *fired = true;
        return;
    }
    if time.elapsed_secs() >= parts[0] {
        *fired = true;
        // Deterministic pick: lowest entity index = first-spawned collider.
        let keys = world.softbody_collider_keys();
        if let Some(key) = keys.into_iter().min_by_key(|e| e.index()) {
            let dt = fixed_time.delta_secs();
            let pos = Vec2::new(parts[1], parts[2]);
            let vel = Vec2::new(parts[3], parts[4]) * dt; // px/s → px/tick
            world.queue_test_hit(key, pos, vel);
            info!(
                "synth hit queued at t={:.3} on collider {:?}: pos=({:.2},{:.2})px vel=({:.2},{:.2})px/s = ({:.3},{:.3})px/tick",
                time.elapsed_secs(),
                key,
                pos.x,
                pos.y,
                parts[3],
                parts[4],
                vel.x,
                vel.y
            );
        } else {
            error!("synth hit skipped: no soft bodies registered yet");
            *fired = false;
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn probe_driver(
    mut commands: Commands,
    time: Res<Time>,
    mut probe: ResMut<Probe>,
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
    mut spine_q: Query<&mut SpineController>,
    root_q: Query<Entity, With<CharacterRoot>>,
    particle_q: Query<&VelloParticle>,
    collider_tq: Query<&Transform, With<VelloCollider>>,
    mut exit: EventWriter<AppExit>,
) {
    // Spawn once both character assets are parsed.
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
                        svg_asset_id: "V8.character.svg".to_owned(),
                        blueprint_asset_id: "v8.character.json".to_owned(),
                        collision_group: 1,
                    },
                    CharacterController::default(),
                ))
                .id();
            probe.character = Some(entity);
            probe.t0 = time.elapsed_secs();
            info!("character spawned, test clock starts");
        }
        return;
    }

    let Some(character) = probe.character else { return };
    let t = time.elapsed_secs() - probe.t0;

    // Physics root must be initialized before the controller acts (same gate
    // as update_character_movement: initial_frame_coordinates.is_some()).
    let phase = if t < BASELINE_END {
        "baseline"
    } else if t < probe.input_end {
        "input_held"
    } else {
        "released"
    };

    if let Ok(mut spine) = spine_q.get_mut(character) {
        // Optional A/B overrides for tuning sweeps (no game-default changes).
        if let Ok(g) = std::env::var("ROT_GAIN") {
            if let Ok(v) = g.parse::<f32>() {
                spine.config.rotation_gain = v;
            }
        }
        if let Ok(g) = std::env::var("V_SCALE") {
            if let Ok(v) = g.parse::<f32>() {
                spine.config.velocity_scale = v;
            }
        }
        if let Ok(g) = std::env::var("BRAKE") {
            if let Ok(v) = g.parse::<f32>() {
                spine.config.brake_blending = v;
            }
        }
        if let Ok(g) = std::env::var("VEL_BLEND") {
            if let Ok(v) = g.parse::<f32>() {
                spine.config.velocity_blending = v;
            }
        }
        spine.move_vector = if t >= BASELINE_END && t < probe.input_end {
            probe.input_vec
        } else {
            Vec2::ZERO
        };
    }

    // Log spine particle state (vello coords: x right, y down).
    if t >= probe.next_log {
        probe.next_log = t + probe.log_period;
        if let Ok(spine) = spine_q.get(character) {
            let names = ["PH", "P0", "P1", "P2", "P3"];
            let mut row = format!("{t:.3},{phase}");
            let mut center = Vec2::ZERO;
            let mut n = 0.0;
            for (i, e) in spine.particles.iter().enumerate() {
                if let Ok(p) = particle_q.get(*e) {
                    row += &format!(
                        ",{:.2},{:.2},{:.2},{:.2}",
                        p.particle.pos.x, p.particle.pos.y, p.particle.velocity.x, p.particle.velocity.y
                    );
                    if i < 4 {
                        center += p.particle.pos;
                        n += 1.0;
                    }
                } else {
                    row += ",nan,nan,nan,nan";
                }
            }
            let _ = names;
            let (cx, cy) = if n > 0.0 { (center.x / n, center.y / n) } else { (f32::NAN, f32::NAN) };
            row += &format!(",{cx:.2},{cy:.2}");
            // Mean body-part (collider) position: measures how the 16 soft-body
            // parts follow the skeleton (follow-lag metric for one-way coupling).
            let mut cc = Vec2::ZERO;
            let mut cn = 0.0;
            for tr in collider_tq.iter() {
                cc += Vec2::new(tr.translation.x, tr.translation.y);
                cn += 1.0;
            }
            let (ccx, ccy) = if cn > 0.0 { (cc.x / cn, cc.y / cn) } else { (f32::NAN, f32::NAN) };
            row += &format!(",{ccx:.2},{ccy:.2},{cn:.0}");
            probe.log.push(row);
        }
    }

    if t >= probe.exit_at && !probe.exited {
        probe.exited = true;
        let mut out = String::from(
            "t,phase,PH_x,PH_y,PH_vx,PH_vy,P0_x,P0_y,P0_vx,P0_vy,P1_x,P1_y,P1_vx,P1_vy,P2_x,P2_y,P2_vx,P2_vy,P3_x,P3_y,P3_vx,P3_vy,center_x,center_y,col_cx,col_cy,col_n\n",
        );
        out += &(probe.log.join("\n") + "\n");
        let out_path = out_path();
        if let Some(dir) = std::path::Path::new(&out_path).parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        match std::fs::write(&out_path, out) {
            Ok(_) => info!("wrote {out_path}"),
            Err(e) => error!("write failed: {e}"),
        }
        exit.write(AppExit::Success);
    }
}
