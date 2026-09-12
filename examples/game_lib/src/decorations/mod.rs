//! Frame-based, non-colliding decorations.
//!
//! A [`Decoration`] is a purely cosmetic rendering entity that rides a host body
//! part — the `VelloCollider` entity it is attached to. It carries **no mass, no
//! collision, no `Connectivity`, no `Damageable`**; it never enters the GPU
//! collision solver and never fires a collision observer. It lives and dies with
//! its host.
//!
//! # Pose model (GPU-transform reuse)
//!
//! The decoration's `VelloScene` is authored **once**, in the host frame's local
//! space (the frame spanned by `p0`→`p1` and `p0`→`p3`, commonly normalized to
//! unit `[0,1]^2`). Every physics frame we derive a **single** frame→world affine
//! from the host's four `frame_particles` and write it to the decoration's
//! `Transform`. The render pipeline ([`scene_affine`](../../../src/render/prepare.rs))
//! uploads that transform as a `PreparedAffine` and the **GPU applies it to every
//! vertex** — the CPU does O(1) work per decoration per frame, no per-vertex
//! recomposition.
//!
//! Cardinal rule: the host's `Transform`/`GlobalTransform` is **never** read for
//! decoration pose. Only [`VelloCollider::frame_particles`] is authoritative.
//!
//! # Lifecycle
//!
//! A decoration is a top-level entity (not a Bevy child), so it must clean itself
//! up when its host dies. Every frame [`resolve_decoration_anchors`] tries to
//! read `host.frame_particles`; when that lookup fails the host is gone, and the
//! decoration is despawned **in the same system** — the authoritative death
//! signal. (The host's own `commands.entity(host).despawn()` is deferred to the
//! end of the schedule, so reacting to `RemovedComponents` would run a frame late
//! and leave the decoration frozen at its last pose for one frame.)

use bevy::prelude::*;
use bevy_vello::{affine_to_mat4, CoordinateSpace, VelloCollider, VelloScene, VelloSceneBundle};
use vello::kurbo::{Affine, BezPath, Rect, Shape};
use vello::peniko;
use vello_physics::{DecorationAnchorMode, Particle, FRAME_PARTICLES_COUNT};

/// A rendering-only decoration riding a host [`VelloCollider`]'s frame.
///
/// The decoration's scene is authored in the host frame's local space once, and
/// each frame we re-derive the single frame→world transform from
/// `host.frame_particles`, so the GPU can carry the per-vertex multiply.
#[derive(Component, Clone)]
pub struct Decoration {
    /// The `VelloCollider` entity whose frame this decoration rides.
    pub host: Entity,
    /// How the frame particles are mapped to the decoration each frame.
    pub anchor: DecorationAnchor,
}

/// Describes how a [`Decoration`] follows its host frame each frame.
#[derive(Clone)]
pub enum DecorationAnchor {
    /// Rigid parallelogram affine (R1). Maps the frame-local scene to world using
    /// the raw frame basis (`p1-p0`, `p3-p0`), so the decoration keeps its shape
    /// and scales with the frame. A pure matrix map — rides the GPU transform.
    ///
    /// `local_pose` is an optional extra in-frame pose (identity by default).
    Rigid {
        /// Extra local pose applied in frame-local space before the frame map.
        local_pose: Affine,
    },
    /// Rigid orientation-follow (R2). Like [`Self::Rigid`] but normalises the
    /// frame basis so the decoration follows the frame's rotation **without**
    /// scaling with it (keeps constant size). Also a pure matrix map.
    RigidRotation {
        /// Extra local pose applied in frame-local space before the frame map.
        local_pose: Affine,
    },
    /// Bilinear shear-follow. Uses all four corners, capturing frame shear/twist.
    ///
    /// Unlike the rigid modes, true per-vertex shear is a per-point weighted
    /// average, **not** a single global affine — full shear recomposition of the
    /// scene is deferred. This mode places the decoration at the bilinear
    /// reconstructed `uv` point plus `pose` (a cheap opt-in standing in for the
    /// full effect).
    Bilinear {
        /// Fractional point within the frame `[0,1]^2`.
        uv: Vec2,
        /// Extra local pose offset applied on top of the constructed point.
        pose: Affine,
    },
}

impl DecorationAnchor {
    /// Map a blueprint anchor mode to a runtime [`DecorationAnchor`] using the
    /// default local pose (identity). This lets the character factory spawn
    /// decorations straight from `CharacterBlueprint.decorations` without
    /// needing per-entry pose/uv authored at load time.
    pub fn from_mode(mode: &DecorationAnchorMode) -> Self {
        match mode {
            DecorationAnchorMode::Rigid => DecorationAnchor::Rigid {
                local_pose: Affine::IDENTITY,
            },
            DecorationAnchorMode::RigidRotation => DecorationAnchor::RigidRotation {
                local_pose: Affine::IDENTITY,
            },
            DecorationAnchorMode::Bilinear => DecorationAnchor::Bilinear {
                uv: Vec2::splat(0.5),
                pose: Affine::IDENTITY,
            },
        }
    }
}

/// Bakes an authored decoration shape so that, at rest, it renders at exactly
/// the position it was authored at in the character SVG — it does **not** center
/// on the host.
///
/// `shape` is the raw world/author-space geometry extracted by the loader
/// (e.g. `"Hp_LLA"` from `SvgCharacterAsset.decoration_shapes`). `host_aabb` is
/// the host collider's rest AABB in the same author space. `mode` is the anchor
/// mode from the blueprint decoration config.
///
/// The scene is authored in the host frame's local space. Each frame the engine
/// builds a single frame→world affine from the host's `frame_particles` and the
/// GPU applies it to the scene. To reproduce the authored world position at
/// rest, we pre-apply the **inverse of the host's rest frame** so that
/// `frame_affine * bake(scene) == authored`:
/// - **Rigid** (raw basis): the frame spans the host AABB, so the inverse is
///   `scale(1/w, 1/h) · translate(-p0)` where `p0` is the host's min corner.
/// - **RigidRotation** (normalized, unit-vector basis): the inverse is just
///   `translate(-p0)` — the decoration keeps its authored pixel size and
///   position, following the host's rotation each frame.
/// - **Bilinear**: reconstructs a single world point from `uv`, then places the
///   scene there; center the shape on the local origin and let `uv` carry the
///   frame-relative placement.
///
/// `p0` is the host's rest AABB min corner — the frame origin for these
/// axis-aligned body parts. No uv/offset ever appears in the loader — the bake
/// is the factory's job.
/// `host_scale` is the character root's world scale. `frame_particles` live in
/// world (scaled) space, while `host_aabb` is in author space.
///
/// - **Rigid** folds the world scale into its raw basis (`p1-p0` is the scaled
///   diagonal), so the bake only needs the author→frame local offset.
/// - **Bilinear** interpolates world corners at an author-normalized `uv`, so it
///   too is scale-correct with just the author offset.
/// - **RigidRotation** rides a **unit** (normalized) basis that carries no
///   scale, so the local scene must be pre-scaled by `host_scale` to land at the
///   authored world position (and size) over the character's frame.
pub fn bake_authored_decoration(
    shape: &BezPath,
    host_aabb: &Rect,
    mode: &DecorationAnchorMode,
    host_scale: f32,
) -> (VelloScene, DecorationAnchor) {
    let shape_center = shape.bounding_box().center();
    // TEMP-DIAG: print the actual bake inputs so we can see the real numbers.
    info!(
        "DECORATION bake inputs: shape_bbox={:?} (x0={:.1},y0={:.1},x1={:.1},y1={:.1}) \
         host_aabb={:?} host_scale={:.3} mode={:?}",
        shape.bounding_box(),
        shape.bounding_box().x0,
        shape.bounding_box().y0,
        shape.bounding_box().x1,
        shape.bounding_box().y1,
        host_aabb,
        host_scale,
        mode
    );
    // The host frame spans its rest AABB, so the frame origin is its min corner.
    // Bring the authored shape into host-frame-local space by applying the
    // inverse: subtract that corner (and, for the raw/rigid basis, divide by the
    // size so the raw unit-span frame maps back to world pixels).
    let (bake_affine, anchor) = match mode {
        DecorationAnchorMode::Rigid => (
            Affine::scale_non_uniform(1.0 / host_aabb.width(), 1.0 / host_aabb.height())
                * Affine::translate((-host_aabb.x0, -host_aabb.y0)),
            DecorationAnchor::Rigid {
                local_pose: Affine::IDENTITY,
            },
        ),
        DecorationAnchorMode::RigidRotation => (
            Affine::scale_non_uniform(host_scale as f64, host_scale as f64)
                * Affine::translate((-host_aabb.x0, -host_aabb.y0)),
            DecorationAnchor::RigidRotation {
                local_pose: Affine::IDENTITY,
            },
        ),
        // Bilinear reconstructs a single world point `bilinear_reconstruct(uv)`
        // from the four corners and places the scene centered there. To land the
        // shape back at its authored position at rest, bake its center onto the
        // local origin and encode the authored offset as `uv` in the rest frame's
        // bilinear coords (axis-aligned over the host AABB at rest).
        DecorationAnchorMode::Bilinear => {
            let uv = Vec2::new(
                ((shape_center.x - host_aabb.x0) / host_aabb.width()) as f32,
                ((shape_center.y - host_aabb.y0) / host_aabb.height()) as f32,
            );
            (
                Affine::translate((-shape_center.x, -shape_center.y)),
                DecorationAnchor::Bilinear {
                    uv,
                    pose: Affine::IDENTITY,
                },
            )
        }
    };
    let mut scene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        bake_affine,
        peniko::Color::rgba(1.0, 0.0, 0.0, 0.9),
        None,
        shape,
    );
    (scene, anchor)
}

/// Builds the frame→world affine from the host's `frame_particles` using the raw
/// parallelogram basis (R1): `world = p0 + u·(p1-p0) + v·(p3-p0)`.
fn r1_affine(corners: &[Particle; FRAME_PARTICLES_COUNT]) -> Affine {
    frame_basis_affine(corners, false)
}

/// Builds the frame→world affine using the normalized basis (R2) — rotation-follow
/// with unit scale.
fn r2_affine(corners: &[Particle; FRAME_PARTICLES_COUNT]) -> Affine {
    frame_basis_affine(corners, true)
}

fn frame_basis_affine(corners: &[Particle; FRAME_PARTICLES_COUNT], normalized: bool) -> Affine {
    let p0 = corners[0].pos;
    let bu = corners[1].pos - p0; // +x axis of the frame (world, vello y-down)
    let bv = corners[3].pos - p0; // +y axis of the frame
    let bu = if normalized {
        bu.normalize_or_zero()
    } else {
        bu
    };
    let bv = if normalized {
        bv.normalize_or_zero()
    } else {
        bv
    };
    Affine::new([
        bu.x as f64,
        bu.y as f64,
        bv.x as f64,
        bv.y as f64,
        p0.x as f64,
        p0.y as f64,
    ])
}

/// Bilinear reconstructed point of `uv` against the four frame corners. Matches
/// the engine's `bilinear_reconstruct` used by the pistol path.
fn bilinear_reconstruct(uv: Vec2, corners: &[Particle; FRAME_PARTICLES_COUNT]) -> Vec2 {
    let p0 = corners[0].pos;
    let p1 = corners[1].pos;
    let p2 = corners[2].pos;
    let p3 = corners[3].pos;
    let bottom = p0.lerp(p1, uv.x);
    let top = p3.lerp(p2, uv.x);
    bottom.lerp(top, uv.y)
}

fn affine_to_transform(affine: Affine) -> Transform {
    let mut transform = Transform::from_matrix(affine_to_mat4(affine));
    // Draw decorations above the body parts they overlap (which sit at z=0).
    // The render queues items by z, so a positive offset ensures the decoration
    // is composited on top and not hidden behind an arm.
    transform.translation.z = 100.0;
    transform
}

/// Spawns a decoration riding `host`'s frame.
///
/// - `host`: the `VelloCollider` entity whose `frame_particles` this decoration
///   follows.
/// - `scene`: the decoration's scene, **authored in the host frame's local
///   space** (e.g. unit `[0,1]^2`). Drawn unchanged each frame; only the
///   `Transform` changes.
/// - `anchor`: how the frame corners map to the scene.
///
/// The decoration is spawned as a **top-level entity** (no `ChildOf`); its world
/// `Transform` is written each frame from the host's `frame_particles`.
pub fn spawn_decoration(
    commands: &mut Commands,
    host: Entity,
    scene: VelloScene,
    anchor: DecorationAnchor,
) -> Entity {
    commands
        .spawn((
            VelloSceneBundle {
                scene,
                coordinate_space: CoordinateSpace::WorldSpace,
                transform: default(),
                ..default()
            },
            Decoration { host, anchor },
        ))
        .id()
}

/// Computes this decoration's world `Transform` from its host's `frame_particles`.
fn decorate_transform(
    anchor: &DecorationAnchor,
    corners: &[Particle; FRAME_PARTICLES_COUNT],
) -> Transform {
    match anchor {
        DecorationAnchor::Rigid { local_pose } => {
            let affine = r1_affine(corners) * *local_pose;
            affine_to_transform(affine)
        }
        DecorationAnchor::RigidRotation { local_pose } => {
            let affine = r2_affine(corners) * *local_pose;
            affine_to_transform(affine)
        }
        DecorationAnchor::Bilinear { uv, pose } => {
            let p = bilinear_reconstruct(*uv, corners);
            // A translation in vello y-down space carried as the decoration's
            // local origin. The scene is place-relative to `p` via `pose`.
            let affine = *pose * Affine::translate((p.x as f64, p.y as f64));
            affine_to_transform(affine)
        }
    }
}

/// Writes each decoration's world `Transform` from its host's `frame_particles`.
///
/// Runs after the physics solve (so `frame_particles` is current). Reads only
/// `host.frame_particles` — never the host's `Transform`/`GlobalTransform`.
///
/// Also handles the host's death: whenever `hosts.get(host)` fails, the host is
/// gone, so the decoration is despawned *in this same system*. This is the
/// authoritative death signal (the host's `commands.entity(host).despawn()` is
/// deferred, so checking `RemovedComponents` runs a frame late and leaves the
/// decoration frozen at its last pose for one frame).
pub fn resolve_decoration_anchors(
    mut commands: Commands,
    mut decorations: Query<(Entity, &Decoration, &mut Transform)>,
    hosts: Query<&VelloCollider>,
) {
    let mut orphans: Vec<Entity> = Vec::new();
    for (entity, decoration, mut transform) in decorations.iter_mut() {
        let Ok(collider) = hosts.get(decoration.host) else {
            orphans.push(entity); // host gone → collect for despawn after the loop
            continue;
        };
        *transform = decorate_transform(&decoration.anchor, &collider.frame_particles);
        // TEMP-DIAG: print the actual frame particles and computed world transform
        // so we can see where the decoration is really landing.
        let v = transform.translation;
        info!(
            "DECORATION runtime host={:?} frame_p0=({:.1},{:.1}) p1=({:.1},{:.1}) \
             p2=({:.1},{:.1}) p3=({:.1},{:.1}) -> world=({:.2},{:.2},{:.2}) anchor={:?}",
            decoration.host,
            collider.frame_particles[0].pos.x,
            collider.frame_particles[0].pos.y,
            collider.frame_particles[1].pos.x,
            collider.frame_particles[1].pos.y,
            collider.frame_particles[2].pos.x,
            collider.frame_particles[2].pos.y,
            collider.frame_particles[3].pos.x,
            collider.frame_particles[3].pos.y,
            v.x,
            v.y,
            v.z,
            std::mem::discriminant(&decoration.anchor),
        );
    }
    for orphan in orphans.drain(..) {
        commands.entity(orphan).despawn();
    }
}

/// Plugin that registers the decoration systems.
pub struct DecorationPlugin;

impl Plugin for DecorationPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, resolve_decoration_anchors);
    }
}
