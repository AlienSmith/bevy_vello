//! Spine indicator / spine-drive input handle.
//!
//! A single scene entity the user commanders with the keyboard: `Q`/`E`/`A`/`D`
//! rotate the *desired heading* absolutely and the arrow keys translate the
//! *desired centre* in the current facing direction. It draws a small triangle +
//! three linking circles in a local, origin-centred frame that mirrors the
//! relative positions of the character's quad spine particles P1/P2/P3 (a
//! vertical pole in the character blueprint).
//!
//! The *shape* is baked once at spawn; position and rotation live on the entity's
//! [`Transform`]. This makes it a convenient, moving/rotating target that the
//! spine controller pulls the real particles towards (external position constraint
//! with damping), instead of driving them by impulse.
//!
//! ## Spawning
//! The indicator is **not** spawned lazily by the draw system. It is spawned by
//! [`spawn_spine_indicator`], an observer on `OnAdd<SpineController>` — i.e. exactly
//! when a character is assembled. This lets us derive the three circle offsets
//! (`local_points`) directly from the real, already-scaled physics particle
//! positions, so there is no second hard-coded copy of the particle layout to
//! keep in sync.
//!
//! ## Coordinates
//! The particle rest positions in [`VelloParticle::particle_init`] are in Vello
//! (y-down) world space, already scaled by the character root's transform during
//! assembly. We store `local_points` centred on P2 in the same Vello y-down space;
//! the baked scene and `calculate_spine_drive` (game_lib) understand that
//! convention, so the indicator shape and the constraint targets both land
//! exactly on the particles.

use bevy::prelude::*;
use bevy_vello::{
    integrations::physics::VelloParticle,
    prelude::{kurbo, peniko},
    VelloScene, VelloSceneBundle,
};
use game_lib::{SpineConfig, SpineController, SpineIndicator};

/// Consolidated, runtime-tuneable spine parameters (a single UI-editable Resource).
///
/// All of the values the mouse/egui tuning UI can edit live in this one Resource:
/// the physics drive config ([`SpineConfig`] — compliance/damping/max speed) and
/// the arrow-command reach (`command_reach` — how far the desired target sits
/// from the current P2). The UI edits this resource directly; the [`apply_spine_config`]
/// system copies the `SpineConfig` portion onto the live [`SpineIndicator`] each frame,
/// so changes take effect immediately with no restarts.
///
/// Only the *control* keys (arrow keys = desired-centre translation, `P` =
/// visibility toggle) remain on the keyboard; value tuning is mouse/UI driven.
#[derive(Resource)]
pub struct SpineTuneParams {
    /// Physics drive config pushed onto the [`SpineIndicator`] by [`apply_spine_config`].
    pub config: SpineConfig,
    /// How far the desired target sits from the current P2, in px. Fed into
    /// `SpineIndicator::command_reach`, so higher = target farther ahead.
    pub command_reach: f32,
}

impl Default for SpineTuneParams {
    fn default() -> Self {
        Self {
            config: SpineConfig::default(),
            command_reach: 400.0,
        }
    }
}

/// Whether the spine indicator visualization is shown (toggled with the `P` key).
#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug)]
pub struct IndicatorVisibility(pub bool);

impl Default for IndicatorVisibility {
    fn default() -> Self {
        Self(true)
    }
}

/// Indices of the driven quad spine particles within
/// [`SpineController::particles`] (`[PH, P0, P1, P2, P3]`) → P1/P2/P3 at 2, 3, 4.
const QUAD_INDICES: [usize; 3] = [2, 3, 4];
/// The quad index that is the shape centre (P2 → `particles[3]`).
const CENTRE_INDEX: usize = QUAD_INDICES[1];

/// Input state for the single spine-indicator entity.
///
/// The `SpineIndicator` component type now lives in `game_lib` (it owns the
/// pose + linear/angular speed fields used by `calculate_spine_drive`). This
/// example keeps the Vello scene/visual spawn only.

/// Convert a Vello (y-down) point to Bevy (y-up).
#[inline]
fn vello_to_bevy(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

/// Radius of the per-particle circles, in local Vello units.
const CIRCLE_RADIUS: f32 = 12.0;
/// Stroke width of the connecting lines, in local Vello units.
const LINE_WIDTH: f64 = 4.0;

/// Marker on the *desired-goal* outline scene entity, so [`draw_spine_indicator`]
/// can update it alongside the virtual-pose entity without confusing the two.
#[derive(Component)]
pub struct DesiredIndicator;

/// Build the fixed "virtual pose" indicator shape once (local coords, centred at
/// the origin). Solid triangle line + filled circles → reads as the *current*
/// commanded pose the physics are pulling toward.
fn build_indicator_scene(points: &[Vec2; 3]) -> VelloScene {
    let mut scene = VelloScene::default();

    // Connecting line: P1→P2→P3→P1 (a closed triangle, degenerate here).
    let mut line_path = kurbo::BezPath::new();
    line_path.push(kurbo::PathEl::MoveTo((points[0].x, points[0].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[1].x, points[1].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[2].x, points[2].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[0].x, points[0].y).into()));
    scene.stroke(
        &kurbo::Stroke::new(LINE_WIDTH),
        kurbo::Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.0, 0.8, 1.0, 0.9), 1.0),
        None,
        &line_path,
    );

    // Three filled circles at the particle positions, one distinct color each.
    let circle_colors = [
        peniko::Color::rgba(1.0, 0.2, 0.2, 0.9), // P1 — red
        peniko::Color::rgba(0.2, 0.8, 0.3, 0.9), // P2 — green (centre)
        peniko::Color::rgba(0.2, 0.4, 1.0, 0.9), // P3 — blue
    ];
    for (point, color) in points.iter().zip(circle_colors.iter()) {
        let circle = kurbo::Circle::new((point.x as f64, point.y as f64), CIRCLE_RADIUS as f64);
        scene.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            *color,
            None,
            &circle,
        );
    }

    scene
}

/// Build the "desired goal" indicator shape (local coords, centred at origin).
/// Same P1/P2/P3 geometry but drawn as an *outline* in orange so it clearly
/// reads as the latched target the virtual pose interpolates toward (distinct
/// from the cyan/red/green/blue virtual pose).
fn build_desired_scene(points: &[Vec2; 3]) -> VelloScene {
    let mut scene = VelloScene::default();

    // Outline triangle (P1→P2→P3→P1) in orange.
    let mut line_path = kurbo::BezPath::new();
    line_path.push(kurbo::PathEl::MoveTo((points[0].x, points[0].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[1].x, points[1].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[2].x, points[2].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[0].x, points[0].y).into()));
    scene.stroke(
        &kurbo::Stroke::new(LINE_WIDTH),
        kurbo::Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(1.0, 0.6, 0.0, 0.9), 1.0),
        None,
        &line_path,
    );

    // Outlined circles at the particle positions, one distinct colour each
    // (matching the virtual pose's P1/P2/P3 red/green/blue scheme) so the goal
    // pose stays readable even when it overlaps the virtual pose.
    let circle_colors = [
        peniko::Color::rgba(1.0, 0.2, 0.2, 0.9), // P1 — red
        peniko::Color::rgba(0.2, 0.8, 0.3, 0.9), // P2 — green (centre)
        peniko::Color::rgba(0.2, 0.4, 1.0, 0.9), // P3 — blue
    ];
    for (point, color) in points.iter().zip(circle_colors.iter()) {
        let circle = kurbo::Circle::new((point.x as f64, point.y as f64), CIRCLE_RADIUS as f64);
        scene.stroke(
            &kurbo::Stroke::new(LINE_WIDTH * 0.75),
            kurbo::Affine::IDENTITY,
            *color,
            None,
            &circle,
        );
    }

    scene
}

/// Spawn the spine indicator when a character is assembled (`OnAdd<SpineController>`).
///
/// Reads the real P1/P2/P3 particle rest positions from [`VelloParticle`] so the
/// indicator's geometry (and later the constraint targets) exactly match the
/// assembled character — no hard-coded particle offsets.
pub fn spawn_spine_indicator(
    trigger: Trigger<OnAdd, SpineController>,
    mut commands: Commands,
    spine_q: Query<&SpineController>,
    particle_q: Query<&VelloParticle>,
) {
    let Ok(spine) = spine_q.get(trigger.target()) else {
        return;
    };

    // Centre (P2) rest position, in Vello (y-down) world space.
    let Ok(centre) = particle_q.get(spine.particles[CENTRE_INDEX]) else {
        return;
    };
    let centre_pos = centre.particle_init.pos;

    // Offsets relative to the centre — already in world/scaled space, so they are
    // the true on-screen spacing (no extra scale factor needed at render/constraint).
    let mut local_points = [Vec2::ZERO; 3];
    for (k, i) in QUAD_INDICES.iter().enumerate() {
        if let Ok(p) = particle_q.get(spine.particles[*i]) {
            local_points[k] = p.particle_init.pos - centre_pos;
        }
    }

    // The *virtual pose* scene (solid cyan/red/green/blue) — what the physics
    // are actually pulling toward this frame.
    let scene = build_indicator_scene(&local_points);
    commands.spawn((
        VelloSceneBundle {
            scene,
            transform: Transform::from_translation(vello_to_bevy(centre_pos).extend(1000.0)),
            ..Default::default()
        },
        SpineIndicator {
            angle: 0.0,
            center: centre_pos,
            // Latched goal starts at the rest pose (no input yet).
            desired_angle: 0.0,
            desired_center: centre_pos,
            local_points,
            character: trigger.target(),
            ..Default::default()
        },
    ));

    // The *desired goal* scene (orange outline) — the latched target the virtual
    // pose interpolates toward. Tracked separately, so arrow-key input is visible
    // even before the body has caught up.
    commands.spawn((
        VelloSceneBundle {
            scene: build_desired_scene(&local_points),
            transform: Transform::from_translation(vello_to_bevy(centre_pos).extend(1000.0)),
            ..Default::default()
        },
        DesiredIndicator,
    ));
}

/// Snap the indicator to the mouse and rotate it about its centre with Q/E.
///
/// The indicator entity itself is spawned by [`spawn_spine_indicator`]; this
/// system only moves/rotates the existing entity, and is a no-op if it is absent.
pub fn draw_spine_indicator(
    visibility: Res<IndicatorVisibility>,
    mut q_indicator: Query<(Entity, &mut SpineIndicator, &mut Transform, &mut Visibility)>,
    mut q_desired: Query<
        (&mut Transform, &mut Visibility),
        (With<DesiredIndicator>, Without<SpineIndicator>),
    >,
) {
    let Ok((_entity, indicator, mut transform, mut vis)) = q_indicator.single_mut() else {
        return;
    };
    // Apply the P-key visibility toggle to both the virtual-pose and desired-goal
    // rendered entities.
    let vis_value = if visibility.0 {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };
    *vis = vis_value;
    for (mut d_transform, mut d_vis) in q_desired.iter_mut() {
        *d_vis = vis_value;
        d_transform.translation = vello_to_bevy(indicator.desired_center).extend(990.0);
        d_transform.rotation = Quat::from_rotation_z(indicator.desired_angle);
    }

    // ---- Render the virtual pose ----
    // `tick_spine_drive` wrote the interpolated, capped virtual pose back onto
    // the indicator; the visual simply mirrors it (Vello y-down → Bevy y-up).
    transform.translation = vello_to_bevy(indicator.center).extend(1000.0);
    transform.rotation = Quat::from_rotation_z(indicator.angle);
}

/// Spine control input: translate the *desired centre* with the arrow keys.
///
/// The arrow keys set a unit-length *direction* (Vello y-down world coords, so
/// screen up = Vello -y, screen right = Vello +x) on the single `SpineIndicator`.
/// When any arrow is held, `command_active` is set and `tick_spine_drive`
/// re-anchors the desired centre to the live P2 (`P2 + commanded_dir *
/// command_reach`) every fixed tick. That keeps the goal a *constant distance*
/// ahead of the spine while held — it never drifts away from or clamps down
/// onto the body. When no arrow is held, `command_active` clears and the last
/// re-anchored `desired_center` stays frozen (virtual pose keeps decaying
/// toward it).
///
/// The aim distance `command_reach` is driven by the `SpineTuneParams` resource
/// (exposed as the "arrow move speed" slider in the UI): higher move speed ⇒ the
/// commanded target sits farther ahead of the current P2.
pub fn spine_control_input(
    keys: Res<ButtonInput<KeyCode>>,
    params: Res<SpineTuneParams>,
    mut visibility: ResMut<IndicatorVisibility>,
    mut q_indicator: Query<&mut SpineIndicator>,
) {
    let Ok(mut indicator) = q_indicator.single_mut() else {
        return;
    };

    // Toggle the indicator visualization on/off with the P key.
    if keys.just_pressed(KeyCode::KeyP) {
        visibility.0 = !visibility.0;
    }

    indicator.command_reach = params.command_reach;
    let mut dir = Vec2::ZERO;
    if keys.pressed(KeyCode::ArrowUp) {
        dir.y -= 1.0;
    }
    if keys.pressed(KeyCode::ArrowDown) {
        dir.y += 1.0;
    }
    if keys.pressed(KeyCode::ArrowLeft) {
        dir.x -= 1.0;
    }
    if keys.pressed(KeyCode::ArrowRight) {
        dir.x += 1.0;
    }
    if dir.length_squared() > 0.0 {
        indicator.commanded_dir = dir.normalize();
        indicator.command_active = true;
    } else {
        indicator.command_active = false;
    }
}

/// Copy the UI-tuned [`SpineConfig`] from the [`SpineTuneParams`] resource onto
/// the live [`SpineIndicator`] each frame, so slider edits take effect
/// immediately (no restarts).
pub fn apply_spine_config(
    params: Res<SpineTuneParams>,
    mut q_indicator: Query<&mut SpineIndicator>,
) {
    let Ok(mut indicator) = q_indicator.single_mut() else {
        return;
    };
    indicator.config = params.config.clone();
}
