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

mod assets;
mod collision;
mod command;
mod config;
mod dispatch;
mod math;
mod probe;
mod report;
mod sample;

use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::asset::{AssetMetaCheck, AssetPlugin};
use bevy::input::InputSystem;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy::transform::TransformPlugin;
use bevy::MinimalPlugins;

use leafwing_input_manager::plugin::InputManagerSystem;

use bevy_vello::collision::{
    CollisionEventBatch, RemovedColliders, VelloCollisionWorld, VelloRayTraceCommand,
};
use bevy_vello::integrations::physics::systems::{
    apply_explicit_impulse_on_connection_particle, apply_explicit_impulse_on_softbody,
    generate_connection, generate_soft_body_for_collider, make_collision_constraints,
    remove_soft_body, run_broad_phase, run_gpu_collision, update_collider_from_soft_body,
    update_connection_particles, update_constraint_world,
};
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterFrameForceEvent, CharacterPivotForceEvent,
    CharacterPivotImpulseEvent, CharacterPivotVelocityEvent, ColliderExternalImpulseEvent,
    VelloConstraintWorld,
};

use game_lib::{
    capture_telemetry, drive_hip_pin, measure_body, tick_loop3_angular, update_character_movement,
    update_ground_support, GroundSupport, HeadlessTelemetryPlugin,
    ResetLimbControlConstraintsEvent, SpineDriveEnabled, VelloCharacterPlugin,
};

use crate::command::{jumpwalk_window, load_script, Command, LimbId};
use crate::config::{
    asset_root, collision_enabled, default_grid, gravity_from_env, hip_pin_params_from_env,
    parse_grid, FIXED_HZ,
};
use crate::math::{env_f32, env_flag};
use crate::probe::{Probe, ProbeArgs};

use crate::assets::{mark_assets_loaded, register_character_assets};
use crate::collision::{
    inject_test_hit, log_collision_stats, setup_headless_collision, spawn_collision_floor,
};
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
        // Match the tuning app: 2 XPBD substeps by default. `SUBSTEPS` overrides.
        let mut world = VelloCollisionWorld::default();
        world.substeps = 2;
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
    // Per-command walk speed override, published by `drive_body_keys`.
    .init_resource::<game_lib::MoveSpeed>()
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
            .unwrap_or(2)
            .max(1);
        info!(
            "pin controller: delta velocity servo, substeps={substeps}, \
             leg_compliance={:.1e}, target_speed={:.0}px/s",
            params.leg_compliance, params.config.target_speed
        );
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
                measure_body,
                drive_hip_pin.before(update_character_movement),
                tick_loop3_angular.after(drive_hip_pin),
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
