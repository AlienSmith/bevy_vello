//! Debug overlay: the *real* collider floor vs the *resolver's* floor line, plus
//! the live DCM (capture point) ξ and the commanded foot targets. Enabled with
//! `DCM_LINES=1` in the visual harness (`visual_arm_pd`).
//!
//! Everything is drawn in absolute Vello y-down world coordinates (identity
//! transform, rebuilt every fixed tick from the same quantities the placement
//! law uses) — so what you see IS what the controller believes:
//!
//!   magenta  the real collision floor surface  (anchor hip y + live stance h)
//!   yellow   the resolver's floor/plant line   (CoM y + measured z_com)
//!   cyan     DCM ξ = com_x + v_com/ω₀ on the yellow line, joined to the CoM
//!   red      measured CoM (mass-weighted, whole body)
//!   green    left foot commanded IK target
//!   blue     right foot commanded IK target
//!   orange   left KNEE commanded target (resolver knee position)
//!   violet   right KNEE commanded target
//!   white    realized leg skeleton: hip(P3) -> knee(P40/P41) -> foot particle,
//!            with a dot on the realized knee. Target ticks far from the white
//!            joints = IK command vs physics diverged.
//!
//! Read it like this: if yellow sits above magenta, the walker plans against a
//! floor that is not the collider (the stance target floats, the foot grazes).
//! If the red CoM dot stays far behind the stance feet, the body is running
//! over its own plant points (accelerating geometry).

use bevy::prelude::*;
use bevy_vello::integrations::physics::VelloParticle;
use bevy_vello::vello::{kurbo, peniko};
use bevy_vello::{VelloScene, VelloSceneBundle};
use game_lib::{HipPin, LeftLegController, RightLegController};

#[derive(Component)]
pub struct DcmOverlay;

/// LIPM nominal gravity (same constant the resolver config carries).
const G: f32 = 980.0;

fn stroke_color(r: f64, g: f64, b: f64) -> peniko::GlowColor {
    peniko::GlowColor::new(peniko::Color::rgba(r, g, b, 0.9), 1.0)
}

fn fill_color(r: f64, g: f64, b: f64) -> peniko::Color {
    peniko::Color::rgba(r, g, b, 0.9)
}

/// Foot world target (controller, bevy y-up) → Vello y-down.
#[inline]
fn foot_to_vello(p: Vec2) -> (f64, f64) {
    (p.x as f64, -p.y as f64)
}

/// Draw one leg: the commanded knee target as a horizontal tick + dot in
/// `color`, and the realized skeleton hip(P3) -> knee -> foot from live
/// particles in white (thigh) and `color` (calf), with dots on the knee and
/// realized foot. Particles are already in the Vello y-down frame.
fn draw_leg(
    s: &mut VelloScene,
    particle_q: &Query<&VelloParticle>,
    knee_target: Vec2,
    particles: &[Entity; 4],
    color: (f64, f64, f64),
) {
    let (kx, ky) = foot_to_vello(knee_target);
    s.stroke(
        &kurbo::Stroke::new(3.0),
        kurbo::Affine::IDENTITY,
        stroke_color(color.0, color.1, color.2),
        None,
        &kurbo::Line::new((kx - 10.0, ky), (kx + 10.0, ky)),
    );
    s.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        fill_color(color.0, color.1, color.2),
        None,
        &kurbo::Circle::new((kx, ky), 5.0),
    );
    let pos = |i: usize| {
        particle_q
            .get(particles[i])
            .ok()
            .map(|p| (p.particle.pos.x as f64, p.particle.pos.y as f64))
    };
    let (Some(hip), Some(knee), Some(foot)) = (pos(1), pos(2), pos(3)) else {
        return;
    };
    s.stroke(
        &kurbo::Stroke::new(2.0),
        kurbo::Affine::IDENTITY,
        stroke_color(1.0, 1.0, 1.0),
        None,
        &kurbo::Line::new(hip, knee),
    );
    s.stroke(
        &kurbo::Stroke::new(2.0),
        kurbo::Affine::IDENTITY,
        stroke_color(color.0, color.1, color.2),
        None,
        &kurbo::Line::new(knee, foot),
    );
    s.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        fill_color(1.0, 1.0, 1.0),
        None,
        &kurbo::Circle::new(knee, 5.0),
    );
    s.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        fill_color(1.0, 1.0, 1.0),
        None,
        &kurbo::Circle::new(foot, 4.0),
    );
}

pub fn dcm_overlay(
    mut commands: Commands,
    pin_q: Query<&HipPin>,
    left_q: Query<&LeftLegController>,
    right_q: Query<&RightLegController>,
    particle_q: Query<&VelloParticle>,
    overlay_q: Query<Entity, With<DcmOverlay>>,
    mut scene_q: Query<&mut VelloScene, With<DcmOverlay>>,
) {
    let Ok(pin) = pin_q.single() else { return };
    if !pin.ready {
        return;
    }

    // Spawn the overlay entity once (first tick the pin is ready).
    if let Ok(entity) = overlay_q.single() {
        // fall through to the update path below
        let _ = entity;
    } else {
        commands.spawn((
            VelloSceneBundle {
                scene: VelloScene::default(),
                transform: Transform::from_xyz(0.0, 0.0, 1010.0),
                ..Default::default()
            },
            DcmOverlay,
        ));
        return;
    }

    let Ok(mut scene) = scene_q.single_mut() else {
        return;
    };
    let mut s = VelloScene::default();

    let m = pin.measured; // Vello y-down

    // Resolver floor line: CoM y + its measured height above the floor.
    let y_res = m.com.y + m.z_com;
    // Real collider floor surface: seed hip y + live stance height
    // (mirrors `spawn_collision_floor`: collider top at hip_y − stance_h in
    //  bevy y-up  ⇔  anchor_p3.y + stance_h in Vello y-down).
    let y_real = pin.anchor_p3.y + pin.stance_height().unwrap_or(m.z_com);

    // Real collider floor: a 40 px-thick slab whose TOP face sits at `y_real`
    // (mirrors `spawn_collision_floor`: center = surface + half thickness).
    // Drawn as a translucent magenta rect + a solid top-edge line so the
    // collision volume is legible, not just its surface.
    const FLOOR_THICK: f64 = 40.0;
    let half_len = crate::math::env_f32("FLOOR_HALF_LEN", 2400.0).max(2400.0) as f64;
    let slab = kurbo::Rect::new(
        -half_len,
        y_real as f64,
        half_len,
        y_real as f64 + FLOOR_THICK,
    );
    s.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        peniko::Color::rgba(1.0, 0.0, 1.0, 0.30),
        None,
        &slab,
    );
    s.stroke(
        &kurbo::Stroke::new(5.0),
        kurbo::Affine::IDENTITY,
        stroke_color(1.0, 0.0, 1.0),
        None,
        &kurbo::Line::new((-half_len, y_real as f64), (half_len, y_real as f64)),
    );
    // Resolver floor line: yellow, drawn as short dash segments so it stays
    // readable where it coincides with the magenta top face.
    {
        let (dash, gap) = (14.0_f64, 10.0_f64);
        let mut x = -5000.0_f64;
        while x < 5000.0 {
            s.stroke(
                &kurbo::Stroke::new(3.0),
                kurbo::Affine::IDENTITY,
                stroke_color(1.0, 1.0, 0.0),
                None,
                &kurbo::Line::new((x, y_res as f64), (x + dash, y_res as f64)),
            );
            x += dash + gap;
        }
    }

    // Commanded foot IK targets (controller frame is bevy y-up → flip).
    if let Ok(l) = left_q.single() {
        let (x, y) = foot_to_vello(l.target);
        s.stroke(
            &kurbo::Stroke::new(3.0),
            kurbo::Affine::IDENTITY,
            stroke_color(0.0, 1.0, 0.0),
            None,
            &kurbo::Line::new((x, y), (x, y + 24.0)),
        );
        s.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            fill_color(0.0, 1.0, 0.0),
            None,
            &kurbo::Circle::new((x, y), 6.0),
        );
    }
    if let Ok(r) = right_q.single() {
        let (x, y) = foot_to_vello(r.target);
        s.stroke(
            &kurbo::Stroke::new(3.0),
            kurbo::Affine::IDENTITY,
            stroke_color(0.2, 0.4, 1.0),
            None,
            &kurbo::Line::new((x, y), (x, y + 24.0)),
        );
        s.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            fill_color(0.2, 0.4, 1.0),
            None,
            &kurbo::Circle::new((x, y), 6.0),
        );
    }

    // Commanded KNEE targets (controller frame is bevy y-up -> flip), plus the
    // realized leg skeleton from live particles (already vello y-down):
    // hip P3 = particles[1], knee P40/P41 = particles[2], foot = particles[3].
    // A target tick that sits far from its white joint = command vs physics gap.
    if let Ok(leg) = left_q.single() {
        draw_leg(
            &mut s,
            &particle_q,
            leg.knee_target,
            &leg.particles,
            (1.0, 0.55, 0.0),
        );
    }
    if let Ok(leg) = right_q.single() {
        draw_leg(
            &mut s,
            &particle_q,
            leg.knee_target,
            &leg.particles,
            (0.7, 0.2, 1.0),
        );
    }

    // DCM marker on the resolver floor line (needs a capture measurement).
    // NOTE: `CaptureMeasure` is in the resolver's seed-relative frame:
    //   x_res = x_world − P3.x + state.hip_pos.x
    // so the world position of the planned capture point is
    //   ξ_world = ξ_res + P3.x − state.hip_pos.x
    // where (P3.x − state.hip_pos.x) is exactly the fiction-vs-reality hip gap.
    if let Some(cap) = pin.state.measured {
        let z = cap.z_com.max(1.0);
        let omega = (G / z).sqrt();
        let xi_res = cap.com_x + cap.v_com / omega;
        // P3 = the hip particle, left controller's particles[1].
        let p3_x = left_q
            .single()
            .ok()
            .and_then(|l| particle_q.get(l.particles[1]).ok())
            .map(|p| p.particle.pos.x)
            .unwrap_or(m.com.x);
        let xi_world = (xi_res + p3_x - pin.state.hip_pos.x) as f64;
        // The fiction hip position itself, as a white tick on the floor line:
        // its distance from the red CoM = how far the fiction has drifted from
        // the real body this tick.
        let fiction_x = (pin.state.hip_pos.x + p3_x - m.com.x) as f64;
        s.stroke(
            &kurbo::Stroke::new(3.0),
            kurbo::Affine::IDENTITY,
            stroke_color(1.0, 1.0, 1.0),
            None,
            &kurbo::Line::new(
                (fiction_x, y_res as f64 - 14.0),
                (fiction_x, y_res as f64 + 14.0),
            ),
        );
        // connector CoM -> ξ
        s.stroke(
            &kurbo::Stroke::new(2.0),
            kurbo::Affine::IDENTITY,
            stroke_color(0.0, 1.0, 1.0),
            None,
            &kurbo::Line::new((m.com.x as f64, m.com.y as f64), (xi_world, y_res as f64)),
        );
        // ξ on the floor line
        s.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            fill_color(0.0, 1.0, 1.0),
            None,
            &kurbo::Circle::new((xi_world, y_res as f64), 8.0),
        );
    }

    // CoM (red).
    s.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        fill_color(1.0, 0.0, 0.0),
        None,
        &kurbo::Circle::new((m.com.x as f64, m.com.y as f64), 8.0),
    );

    *scene = s;
}
