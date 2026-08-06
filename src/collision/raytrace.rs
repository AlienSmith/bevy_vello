use bevy::prelude::*;

use crate::{
    collision::{
        GpuRayTraceRunner, RayTraceBatch, RayTraceBatchEntry, VelloRayTraceCommand,
        VELLO_COLLISION_WORLD_RATIO,
    },
    mat4_to_affine, VelloCollider,
};

/// Collects [`VelloRayTraceCommand`] events emitted since the last fixed step.
pub fn collect_raytrace_commands(
    mut commands: EventReader<VelloRayTraceCommand>,
    mut queue: ResMut<RayTraceCommandQueue>,
) {
    for cmd in commands.read() {
        queue.commands.push(cmd.clone());
    }
}

/// Temporary resource that buffers ray trace commands between
/// `collect_raytrace_commands` and `run_gpu_raytrace`.
#[derive(Resource, Default, Clone)]
pub struct RayTraceCommandQueue {
    pub commands: Vec<VelloRayTraceCommand>,
}

/// Ray-vs-AABB intersection test using the slab method (Kay & Kajiya).
/// Returns the entry distance (t_entry) if the ray hits the AABB, or `None` if it misses.
/// `aabb` uses bevy y-up coordinates where: (x0, y0) = min, (x1, y1) = max.
fn ray_aabb_intersect(
    origin: Vec2,
    direction: Vec2,
    max_distance: f32,
    aabb_min: Vec2,
    aabb_max: Vec2,
) -> Option<f32> {
    let epsilon = 1e-10f32;

    // Precompute inverse direction
    let inv_x = if direction.x.abs() < epsilon {
        f32::MAX
    } else {
        1.0 / direction.x
    };
    let inv_y = if direction.y.abs() < epsilon {
        f32::MAX
    } else {
        1.0 / direction.y
    };

    let t1_x = (aabb_min.x - origin.x) * inv_x;
    let t2_x = (aabb_max.x - origin.x) * inv_x;
    let t1_y = (aabb_min.y - origin.y) * inv_y;
    let t2_y = (aabb_max.y - origin.y) * inv_y;

    let t_entry_x = t1_x.min(t2_x);
    let t_exit_x = t1_x.max(t2_x);
    let t_entry_y = t1_y.min(t2_y);
    let t_exit_y = t1_y.max(t2_y);

    let t_entry = t_entry_x.max(t_entry_y);
    let t_exit = t_exit_x.min(t_exit_y);

    // Valid hit: t_entry <= t_exit, t_exit >= 0, t_entry <= max_distance
    if t_entry <= t_exit && t_exit >= 0.0 && t_entry <= max_distance {
        Some(t_entry.max(0.0))
    } else {
        None
    }
}

/// Runs broad phase ray-AABB scan for each queued ray against all colliders,
/// builds a [`vello::RayTraceScene`], executes GPU ray tracing synchronously,
/// and populates [`RayTraceBatch`] with the closest hit per ray.
pub fn run_gpu_raytrace(
    ray_runner: Res<GpuRayTraceRunner>,
    mut queue: ResMut<RayTraceCommandQueue>,
    mut batch: ResMut<RayTraceBatch>,
    collider_query: Query<(Entity, &VelloCollider)>,
) {
    let commands = std::mem::take(&mut queue.commands);
    if commands.is_empty() {
        return;
    }

    // For each ray, collect all collider AABB candidates with entry distances.
    // Sorted by t so the closest is first.
    struct RayCandidates {
        cmd: VelloRayTraceCommand,
        candidates: Vec<(Entity, f32)>,
    }

    let mut ray_candidates: Vec<RayCandidates> = Vec::with_capacity(commands.len());

    for (ray_idx, cmd) in commands.into_iter().enumerate() {
        let mut candidates: Vec<(Entity, f32)> = Vec::new();
        for (entity, collider) in collider_query.iter() {
            let aabb = collider.get_aabb(); // (x0, y0, x1, y1) in local space
            let transform = &collider.soft_body_global_transform;
            // Convert local AABB to world-space AABB (using translation only for
            // conservative broad phase)
            let world_min = Vec2::new(
                aabb.x + transform.translation.x,
                aabb.y + transform.translation.y,
            );
            let world_max = Vec2::new(
                aabb.z + transform.translation.x,
                aabb.w + transform.translation.y,
            );
            if let Some(t_entry) = ray_aabb_intersect(
                cmd.origin,
                cmd.direction,
                cmd.max_distance,
                world_min,
                world_max,
            ) {
                candidates.push((entity, t_entry));
            }
        }
        // Sort by entry distance (ascending)
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        ray_candidates.push(RayCandidates { cmd, candidates });
    }

    // Build the RayTraceScene: encode one (ray, shape) pair per candidate,
    // tracking which ray each pair belongs to for result reconstruction.
    let mut scene = vello::RayTraceScene::new();
    let mut pair_ray_indices: Vec<usize> = Vec::new();
    let mut pair_entities: Vec<Option<Entity>> = Vec::new();

    for (ray_idx, rc) in ray_candidates.iter().enumerate() {
        if rc.candidates.is_empty() {
            continue;
        }
        // Convert ray to vello space (y-down, scaled)
        let ray_origin_vello = vello::kurbo::Vec2::new(
            rc.cmd.origin.x as f64 * VELLO_COLLISION_WORLD_RATIO as f64,
            -(rc.cmd.origin.y as f64) * VELLO_COLLISION_WORLD_RATIO as f64,
        );
        let ray_dir_vello =
            vello::kurbo::Vec2::new(rc.cmd.direction.x as f64, -(rc.cmd.direction.y as f64));

        for &(entity, _t) in &rc.candidates {
            if let Ok((_, collider)) = collider_query.get(entity) {
                let affine = mat4_to_affine(collider.soft_body_global_transform.compute_matrix())
                    .then_scale(VELLO_COLLISION_WORLD_RATIO as f64);
                scene.encode_ray_shape(ray_origin_vello, ray_dir_vello, &collider.shape, affine);
                pair_ray_indices.push(ray_idx);
                pair_entities.push(Some(entity));
            }
        }
    }

    // GPU execution.
    //
    // Guard against an empty scene: when no ray has any collider candidate,
    // `scene` encodes zero bytes and the buffer pool quantizes size 0 up to a
    // 2-byte buffer (`size_class(0, 1) == 2`). Binding that to a
    // `var<storage> scene: array<u32>` binding (which needs at least 4 bytes)
    // trips wgpu validation ("Buffer is bound with size 2 where the shader
    // expects 4 in group[0] compact index 1"). Skip the GPU entirely in that
    // case; every ray simply misses.
    let num_rays = ray_candidates.len();
    let results: Vec<vello::RayTraceResult> = if pair_ray_indices.is_empty() {
        Vec::new()
    } else {
        ray_runner.run_raytrace(&scene)
    };

    // results.len() == number of encoded (ray, shape) pairs.
    // For each ray, pick the closest positive t.
    let mut best_per_ray: Vec<Option<(f32, Entity, vello::RayTraceResult)>> = vec![None; num_rays];

    for (pair_idx, result) in results.iter().enumerate() {
        if pair_idx >= pair_ray_indices.len() {
            break;
        }
        let ray_idx = pair_ray_indices[pair_idx];
        if ray_idx >= num_rays {
            continue;
        }
        // t >= 0 means a hit; take the smallest positive t
        if result.t >= 0.0 {
            let replace = match &best_per_ray[ray_idx] {
                None => true,
                Some((best_t, _, _)) => result.t < *best_t,
            };
            if replace {
                best_per_ray[ray_idx] = pair_entities[pair_idx].map(|e| (result.t, e, *result));
            }
        }
    }

    // Populate batch
    batch.entries.clear();
    for (ray_idx, rc) in ray_candidates.iter().enumerate() {
        let cmd = rc.cmd.clone();
        match &best_per_ray[ray_idx] {
            Some((t, entity, result)) => {
                batch.entries.push(RayTraceBatchEntry {
                    command: cmd,
                    result: *result,
                    hit_entity: Some(*entity),
                });
            }
            None => {
                // Miss: produce an entry with no hit
                batch.entries.push(RayTraceBatchEntry {
                    command: cmd,
                    result: vello::RayTraceResult {
                        cubic_index: 0,
                        t: -1.0,
                        point_x: 0.0,
                        point_y: 0.0,
                    },
                    hit_entity: None,
                });
            }
        }
    }
}
