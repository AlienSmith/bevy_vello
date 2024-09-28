//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

use bevy::{
    prelude::*,
    render::{render_resource::WgpuFeatures, settings::WgpuSettings, RenderPlugin},
};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_hanabi::prelude::*;

use bevy::asset::AssetMetaCheck;
use bevy_vello::{
    integrations::{HanabiIntegrationPlugin, VelloSceneSubBundle},
    vello::{kurbo, peniko},
};
use bevy_vello::{prelude::*, VelloPlugin};

const BOUNDS: Vec2 = Vec2::new(1200.0, 640.0);

/// player component
#[derive(Component, Default)]
struct Player {
    /// linear speed in meters per second
    movement_speed: f32,
    /// rotation speed in radians per second
    rotation_speed: f32,

    spawn_count: u32,

    last_spawn_time: f32,

    spawn_count_limits: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut wgpu_settings = WgpuSettings::default();
    wgpu_settings
        .features
        .set(WgpuFeatures::VERTEX_WRITABLE_STORAGE, true);

    let mut app = App::default();
    app.insert_resource(ClearColor(Color::DARK_GRAY))
        .add_plugins(
            DefaultPlugins
                .set(RenderPlugin {
                    render_creation: wgpu_settings.into(),
                    synchronous_pipeline_compilation: false,
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "🎆 Hanabi — 2d".to_string(),
                        ..default()
                    }),
                    ..default()
                }),
        )
        .add_plugins(HanabiIntegrationPlugin);

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.insert_resource(AssetMetaCheck::Never)
        .add_plugins(VelloPlugin)
        .add_systems(Startup, setup_vector_graphics)
        .add_systems(Update, player_control_system)
        .run();

    Ok(())
}

fn make_default_rect_particles(scene: &mut VelloScene, particle_index: u32) {
    let value = (96.0 + (particle_index as f32) * 8.0) / 256.0;
    let mut color: peniko::Color = peniko::Color::rgb(value as f64, 0.0, 0.0);
    *scene = VelloScene::default();
    scene.push_instance(0, 0);
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        color,
        None,
        &kurbo::Rect::new(-2.5, -2.5, 2.5, 2.5),
    );
    scene.pop_instance();
}

fn default_effect(effects: &mut ResMut<Assets<EffectAsset>>, count: f32) -> Handle<EffectAsset> {
    let mut gradient = Gradient::new();
    gradient.add_key(0.0, Vec4::new(0.5, 0.5, 1.0, 1.0));
    gradient.add_key(1.0, Vec4::new(0.5, 0.5, 1.0, 0.0));

    let writer = ExprWriter::new();

    let age = writer.lit(0.).expr();
    let init_age = SetAttributeModifier::new(Attribute::AGE, age);

    let lifetime = writer.lit(10.).expr();
    let init_lifetime = SetAttributeModifier::new(Attribute::LIFETIME, lifetime);

    let init_pos = SetPositionCircleModifier {
        center: writer.lit(Vec3::ZERO).expr(),
        axis: writer.lit(Vec3::Z).expr(),
        radius: writer.lit(0.05).expr(),
        dimension: ShapeDimension::Surface,
    };

    let init_vel = SetVelocityCircleModifier {
        center: writer.lit(Vec3::ZERO).expr(),
        axis: writer.lit(Vec3::Z).expr(),
        speed: writer.lit(30.0).expr(),
    };

    let module = writer.finish();

    // Create a new effect asset spawning 30 particles per second from a circle
    // and slowly fading from blue-ish to transparent over their lifetime.
    // By default the asset spawns the particles at Z=0.
    let spawner = Spawner::rate(30.0.into());
    effects.add(
        EffectAsset::new(vec![4096], spawner, module)
            .with_name("2d")
            .init(init_pos)
            .init(init_vel)
            .init(init_age)
            .init(init_lifetime)
            .render(SizeOverLifetimeModifier {
                gradient: Gradient::constant(Vec2::splat(2.0)),
                screen_space_size: false,
            })
            .with_simulation_space(SimulationSpace::Local), // .render(ColorOverLifetimeModifier { gradient })
                                                            // .render(round),
    )
}

fn spawn_particles_at(
    commands: &mut Commands,
    effects: &mut ResMut<Assets<EffectAsset>>,
    translate: Vec3,
    count: f32,
    particle_index: u32,
) {
    // Create a color gradient for the particles
    let effect = default_effect(effects, count);
    let mut scene = VelloScene::default();
    make_default_rect_particles(&mut scene, particle_index);
    // Spawn an instance of the particle effect, and override its Z layer to
    // be above the reference white square previously spawned.
    let id = commands
        .spawn((
            ParticleEffectBundle {
                // Assign the Z layer so it appears in the egui inspector and can be modified at runtime
                effect: ParticleEffect::new(effect).with_z_layer_2d(Some(0.1)),
                transform: Transform::from_translation(translate),
                ..default()
            },
            VelloSceneSubBundle {
                scene,
                ..Default::default()
            },
        ))
        .id();
}

fn setup_vector_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
    let mut scene: VelloScene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgb(1.0, 1.0, 1.0),
        None,
        &kurbo::Rect::new(-2.5, -5.0, 2.5, 5.0),
    );
    commands.spawn((
        VelloSceneBundle {
            scene,
            ..Default::default()
        },
        Player {
            movement_speed: 500.0,                  // meters per second
            rotation_speed: f32::to_radians(360.0), // degrees per second
            spawn_count_limits: 20,
            ..Default::default()
        },
    ));
}

/// Demonstrates applying rotation and movement based on keyboard input.
fn player_control_system(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut Player, &mut Transform)>,
    mut commands: Commands,
    mut effects: ResMut<Assets<EffectAsset>>,
) {
    let (mut ship, mut transform) = query.single_mut();

    let mut rotation_factor = 0.0;
    let mut movement_factor = 0.0;

    if keyboard_input.pressed(KeyCode::ArrowLeft) {
        rotation_factor += 1.0;
    }

    if keyboard_input.pressed(KeyCode::ArrowRight) {
        rotation_factor -= 1.0;
    }

    if keyboard_input.pressed(KeyCode::ArrowUp) {
        movement_factor += 1.0;
    }

    if keyboard_input.pressed(KeyCode::ArrowDown) {
        movement_factor -= 1.0;
    }

    if keyboard_input.pressed(KeyCode::Space) {
        if ship.spawn_count < ship.spawn_count_limits
            && time.elapsed_seconds() - ship.last_spawn_time > 0.5
        {
            let count = 1;
            spawn_particles_at(
                &mut commands,
                &mut effects,
                transform.translation,
                count as f32,
                ship.spawn_count,
            );
            ship.spawn_count += 1;
            ship.last_spawn_time = time.elapsed_seconds();
        }
    }

    // update the ship rotation around the Z axis (perpendicular to the 2D plane of the screen)
    transform.rotate_z(rotation_factor * ship.rotation_speed * time.delta_seconds());

    // get the ship's forward vector by applying the current rotation to the ships initial facing
    // vector
    let movement_direction = transform.rotation * Vec3::Y;
    // get the distance the ship will move based on direction, the ship's movement speed and delta
    // time
    let movement_distance = movement_factor * ship.movement_speed * time.delta_seconds();
    // create the change in translation using the new movement direction and distance
    let translation_delta = movement_direction * movement_distance;
    // update the ship translation with our new translation delta
    transform.translation += translation_delta;

    // bound the ship within the invisible level bounds
    let extents = Vec3::from((BOUNDS / 2.0, 0.0));
    transform.translation = transform.translation.min(extents).max(-extents);
}
