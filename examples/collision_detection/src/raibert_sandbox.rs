//! Raibert lower-body kinematic sandbox (pure math, no XPBD).
//!
//! A single VelloScene draws the flat floor plus the five resolver particles
//! (1 hip + 2 knees + 2 feet) and the thigh/calf connecting lines. The pose is
//! produced entirely by [`game_lib::lower_body`] each frame; this module only
//! owns rendering, camera-follow, and an egui tuning window.
//!
//! Coordinate convention:
//! - The resolver works in **Bevy y-up** world space (`floor_y = 0`, hip above).
//! - Vello scenes are y-down, so every point is converted with `to_vello` before
//!   being written into the scene.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_vello::{
    prelude::{kurbo, peniko},
    VelloScene, VelloSceneBundle,
};
use game_lib::{LowerBodyConfig, LowerBodyController, LowerBodyParams};

/// Marker on the sandbox render entity (owns the rebuilt [`VelloScene`] and the
/// [`LowerBodyController`] the game_lib systems drive each Update).
#[derive(Component)]
pub struct RaibertSandbox;

/// Marker on the follow camera so [`update_sandbox`] can recentre it on the hip.
#[derive(Component)]
pub struct SandboxCamera;

/// The one visual entity (scene + controller) and the follow camera.
pub fn setup_sandbox(mut commands: Commands) {
    commands.spawn((
        Camera2d::default(),
        SandboxCamera,
        Transform::from_translation(Vec3::new(0.0, 150.0, 0.0)),
    ));

    commands.spawn((
        RaibertSandbox,
        LowerBodyController::new(&LowerBodyConfig::default()),
        VelloSceneBundle {
            transform: Transform::from_translation(Vec3::ZERO),
            ..Default::default()
        },
    ));
}

/// Convert a resolver (Bevy y-up) point to Vello (y-down) space.
#[inline]
fn to_vello(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

fn point(p: Vec2) -> kurbo::Point {
    kurbo::Point::new(p.x as f64, p.y as f64)
}

/// Rebuild the whole scene from the resolver pose and, if a follow camera
/// exists, recentre it on the hip.
pub fn update_sandbox(
    mut q_scene: Query<(Entity, &LowerBodyController, &mut VelloScene), With<RaibertSandbox>>,
    params: Res<LowerBodyParams>,
    mut q_camera: Query<&mut Transform, (With<SandboxCamera>, Without<RaibertSandbox>)>,
) {
    let Ok((_entity, controller, mut scene)) = q_scene.single_mut() else {
        return;
    };

    let pose = controller.pose;
    let hip = pose.hip;
    let cfg = &params.config;

    let mut s = VelloScene::default();

    // Background (dark panel).
    s.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        peniko::Color::rgb(0.05, 0.06, 0.09),
        None,
        &kurbo::Rect::new(-100_000.0, -100_000.0, 100_000.0, 100_000.0),
    );

    // Floor line, drawn wide enough to always span the viewport around the hip.
    let floor_lo = to_vello(Vec2::new(hip.x - 2000.0, cfg.floor_y));
    let floor_hi = to_vello(Vec2::new(hip.x + 2000.0, cfg.floor_y));
    let mut floor_path = kurbo::BezPath::new();
    floor_path.move_to(point(floor_lo));
    floor_path.line_to(point(floor_hi));
    s.stroke(
        &kurbo::Stroke::new(4.0),
        kurbo::Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.5, 0.55, 0.6, 0.9), 1.0),
        None,
        &floor_path,
    );

    // Hip height reference line (dashed feel via low alpha).
    let h_lo = to_vello(Vec2::new(hip.x - 2000.0, hip.y));
    let h_hi = to_vello(Vec2::new(hip.x + 2000.0, hip.y));
    let mut h_path = kurbo::BezPath::new();
    h_path.move_to(point(h_lo));
    h_path.line_to(point(h_hi));
    s.stroke(
        &kurbo::Stroke::new(1.5),
        kurbo::Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.4, 0.7, 0.5, 0.35), 1.0),
        None,
        &h_path,
    );

    // Leg bones: hip → knee → foot for each side.
    let thigh_color = peniko::GlowColor::new(peniko::Color::rgba(0.9, 0.4, 0.2, 0.95), 1.0);
    let calf_color = peniko::GlowColor::new(peniko::Color::rgba(0.2, 0.6, 0.95, 0.95), 1.0);
    for (knee, foot) in [
        (pose.left_knee, pose.left_foot),
        (pose.right_knee, pose.right_foot),
    ] {
        let mut thigh_path = kurbo::BezPath::new();
        thigh_path.move_to(point(to_vello(hip)));
        thigh_path.line_to(point(to_vello(knee)));
        s.stroke(
            &kurbo::Stroke::new(6.0),
            kurbo::Affine::IDENTITY,
            thigh_color,
            None,
            &thigh_path,
        );

        let mut calf_path = kurbo::BezPath::new();
        calf_path.move_to(point(to_vello(knee)));
        calf_path.line_to(point(to_vello(foot)));
        s.stroke(
            &kurbo::Stroke::new(6.0),
            kurbo::Affine::IDENTITY,
            calf_color,
            None,
            &calf_path,
        );
    }

    // Particles: hip white, knees red, feet green.
    let hip_circle = kurbo::Circle::new(point(to_vello(hip)), 11.0);
    s.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        peniko::Color::rgba(1.0, 1.0, 1.0, 0.95),
        None,
        &hip_circle,
    );
    for knee in [pose.left_knee, pose.right_knee] {
        let c = kurbo::Circle::new(point(to_vello(knee)), 8.0);
        s.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            peniko::Color::rgba(1.0, 0.25, 0.25, 0.95),
            None,
            &c,
        );
    }
    for foot in [pose.left_foot, pose.right_foot] {
        let c = kurbo::Circle::new(point(to_vello(foot)), 8.0);
        s.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            peniko::Color::rgba(0.2, 0.9, 0.4, 0.95),
            None,
            &c,
        );
    }

    *scene = s;

    // Follow the hip horizontally so a long run stays on screen.
    if let Ok(mut cam) = q_camera.single_mut() {
        cam.translation.x = hip.x;
    }
}

/// egui window to tune the resolver config live, and to show the Raibert
/// forward/backward intermediates (`X_max`, `v_max`, clamped velocity).
pub fn tuning_ui(
    mut contexts: EguiContexts,
    mut params: ResMut<LowerBodyParams>,
    controller_q: Query<&LowerBodyController, With<RaibertSandbox>>,
) {
    let cfg = &mut params.config;
    egui::Window::new("Raibert Lower Body Tuning").show(contexts.ctx_mut(), |ui| {
        ui.label("← → or A/D walk; R resets to the resting stance.");
        ui.separator();

        ui.heading("Leg lengths & height");
        ui.add(egui::Slider::new(&mut cfg.thigh_len, 20.0..=160.0).text("thigh_len (px)"));
        ui.add(egui::Slider::new(&mut cfg.calf_len, 20.0..=160.0).text("calf_len (px)"));
        ui.add(egui::Slider::new(&mut cfg.hip_height, 40.0..=200.0).text("hip_height (px)"));
        ui.add(egui::Slider::new(&mut cfg.crouch_height, 30.0..=160.0).text("crouch_height (px)"));
        ui.checkbox(&mut cfg.crouch, "crouch");
        ui.separator();

        ui.heading("Gait timing");
        ui.add(egui::Slider::new(&mut cfg.stance_duration_ts, 0.05..=1.0).text("stance Ts (s)"));
        ui.add(egui::Slider::new(&mut cfg.swing_duration_ts, 0.02..=0.8).text("swing (s)"));
        ui.add(egui::Slider::new(&mut cfg.step_height, 0.0..=80.0).text("step_height (px)"));
        ui.separator();

        ui.heading("Raibert gains & speed");
        ui.add(egui::Slider::new(&mut cfg.target_speed, 0.0..=600.0).text("target_speed (px/s)"));
        ui.add(egui::Slider::new(&mut cfg.accel, 10.0..=4000.0).text("accel (px/s²)"));
        ui.add(egui::Slider::new(&mut cfg.kv, 0.0..=0.5).text("kv (s)"));
        ui.add(egui::Slider::new(&mut cfg.safety_margin, 0.1..=1.0).text("safety_margin"));
        ui.separator();

        ui.heading("Knee bend");
        let mut bend_positive = cfg.bend_sign > 0.0;
        if ui
            .checkbox(&mut bend_positive, "knees forward (+x)")
            .changed()
        {
            cfg.bend_sign = if bend_positive { 1.0 } else { -1.0 };
        }

        if let Ok(controller) = controller_q.single() {
            let d = &controller.debug;
            let hip = controller.state.hip_pos;
            ui.separator();
            ui.heading("Telemetry");
            ui.label(format!("v_desired : {:.1} px/s", d.v_desired));
            ui.label(format!("v_clamped : {:.1} px/s", d.v_clamped));
            ui.label(format!("X_max     : {:.1} px", d.x_max));
            ui.label(format!("v_max     : {:.1} px/s", d.v_max));
            ui.label(format!("hip pos   : {:.1}, {:.1}", hip.x, hip.y));
            ui.label(format!(
                "phases    : L={:?} R={:?}",
                d.left_phase, d.right_phase
            ));
        }
    });
}
