//! Headless collision bring-up: the wgpu device + vello renderer for the GPU
//! narrow phase, the static floor at the resolver's foot line, the periodic
//! contact stats, and the env-gated `HIT` one-shot strike on the pinned frame.

use std::sync::{Arc, Mutex};

use bevy::prelude::{
    info, Commands, Entity, EventWriter, Fixed, GlobalTransform, Local, Query, Res, Time,
    Transform, Vec2, With,
};

use wgpu;

use bevy_vello::collision::{
    path_to_ccw_quad_path, CollisionEventBatch, GpuCollisionRunner, VelloCollider,
    VelloCollisionBroadPhase, VELLO_COLLISION_COOL_DOWN_TIME,
};
use bevy_vello::integrations::physics::CharacterFrameForceEvent;
use bevy_vello::vello::kurbo::{Rect, Shape as _};
use bevy_vello::vello::peniko;

use game_lib::{HipPin, HipPinParams};

use crate::config::FIXED_HZ;
use crate::math::env_f32;

/// Build a headless wgpu device + vello renderer for the GPU narrow phase.
/// No surface/window: the collision path only needs a device and queue.
pub fn setup_headless_collision(mut commands: Commands) {
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
pub fn spawn_collision_floor(
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
    // `FLOOR_HALF_LEN=<px>` widens the single-rect floor collider (default 2400 ⇒
    // 4800 px total). A long walk (~40 s × ~150 px/s ≈ 6000 px) walks off the
    // default edge and free-falls, truncating the test; widen it to keep the body
    // supported for the whole run. EXPERIMENT: a single very wide quad path can
    // stress the GPU narrow phase / BVH, so this is also a collision-system
    // stress test for a long floor.
    let floor_half_len = env_f32("FLOOR_HALF_LEN", 2400.0).max(2400.0) as f64;
    let rect = Rect::new(
        -floor_half_len,
        -(FLOOR_HALF_THICKNESS as f64),
        floor_half_len,
        FLOOR_HALF_THICKNESS as f64,
    );
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

/// One log line per sim-second with the collider/contact counts.
pub fn log_collision_stats(
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
pub fn inject_test_hit(
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
