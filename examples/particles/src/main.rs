//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

use bevy::{
    log::LogPlugin,
    prelude::*,
    render::{
        camera::ScalingMode, render_resource::WgpuFeatures, settings::WgpuSettings, RenderPlugin,
    },
    sprite::{MaterialMesh2dBundle, Mesh2dHandle},
};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_hanabi::prelude::*;

use bevy::asset::AssetMetaCheck;
use bevy::prelude::*;
use bevy_vello::vello::{kurbo, peniko};
use bevy_vello::{prelude::*, VelloPlugin};

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
        .add_plugins(HanabiPlugin);

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.add_systems(Startup, setup);
    
    app.insert_resource(AssetMetaCheck::Never)
        .add_plugins(VelloPlugin)
        .add_systems(Startup, setup_vector_graphics)
        .add_systems(Update, simple_animation)
        .run();

    Ok(())
}

fn setup(
    mut commands: Commands,
    mut effects: ResMut<Assets<EffectAsset>>,
    // mut meshes: ResMut<Assets<Mesh>>,
    // mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Spawn a 2D camera
    // let mut camera = Camera2dBundle::default();
    // camera.projection.scale = 1.0;
    // camera.projection.scaling_mode = ScalingMode::FixedVertical(1.);
    // commands.spawn(camera);

    // Spawn a reference white square in the center of the screen at Z=0
    // commands
    //     .spawn(MaterialMesh2dBundle {
    //         mesh: meshes
    //             .add(Rectangle {
    //                 half_size: Vec2::splat(0.1),
    //             })
    //             .into(),
    //         material: materials.add(ColorMaterial {
    //             color: Color::WHITE,
    //             ..Default::default()
    //         }),
    //         ..Default::default()
    //     })
    //     .insert(Name::new("square"));

    // Create a color gradient for the particles
    let mut gradient = Gradient::new();
    gradient.add_key(0.0, Vec4::new(0.5, 0.5, 1.0, 1.0));
    gradient.add_key(1.0, Vec4::new(0.5, 0.5, 1.0, 0.0));

    let writer = ExprWriter::new();

    let age = writer.lit(0.).expr();
    let init_age = SetAttributeModifier::new(Attribute::AGE, age);

    let lifetime = writer.lit(5.).expr();
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
        speed: writer.lit(0.1).expr(),
    };

    let mut module = writer.finish();

    let round = RoundModifier::constant(&mut module, 2.0 / 3.0);

    // Create a new effect asset spawning 30 particles per second from a circle
    // and slowly fading from blue-ish to transparent over their lifetime.
    // By default the asset spawns the particles at Z=0.
    let spawner = Spawner::rate(30.0.into());
    let effect = effects.add(
        EffectAsset::new(vec![4096], spawner, module)
            .with_name("2d")
            .init(init_pos)
            .init(init_vel)
            .init(init_age)
            .init(init_lifetime)
            .render(SizeOverLifetimeModifier {
                gradient: Gradient::constant(Vec2::splat(0.02)),
                screen_space_size: false,
            })
            .render(ColorOverLifetimeModifier { gradient })
            .render(round),
    );

    // Spawn an instance of the particle effect, and override its Z layer to
    // be above the reference white square previously spawned.
    commands
        .spawn(ParticleEffectBundle {
            // Assign the Z layer so it appears in the egui inspector and can be modified at runtime
            effect: ParticleEffect::new(effect).with_z_layer_2d(Some(0.1)),
            ..default()
        })
        .insert(Name::new("effect:2d"));
}

fn update_plane(time: Res<Time>, mut query: Query<&mut Transform, With<Mesh2dHandle>>) {
    let mut transform = query.single_mut();
    // Move the plane back and forth to show particles ordering relative to it
    transform.translation.z = (time.elapsed_seconds() * 2.5).sin() * 0.045 + 0.1;
}

fn setup_vector_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
    commands.spawn(VelloSceneBundle::default());
}

fn simple_animation(mut query_scene: Query<(&mut Transform, &mut VelloScene)>, time: Res<Time>) {
    let sin_time = time.elapsed_seconds().sin().mul_add(0.5, 0.5);
    let (mut transform, mut scene) = query_scene.single_mut();

    // Reset scene every frame
    *scene = VelloScene::default();

    // Animate color green to blue
    let c = Vec3::lerp(
        Vec3::new(-1.0, 1.0, -1.0),
        Vec3::new(-1.0, 1.0, 1.0),
        sin_time + 0.5,
    );

    // Animate the corner radius
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgb(c.x as f64, c.y as f64, c.z as f64),
        None,
        &kurbo::RoundedRect::new(-50.0, -50.0, 50.0, 50.0, (sin_time as f64) * 50.0),
    );

    transform.scale = Vec3::lerp(Vec3::ONE * 0.5, Vec3::ONE * 1.0, sin_time);
    transform.translation = Vec3::lerp(Vec3::X * -100.0, Vec3::X * 100.0, sin_time);
    transform.rotation = Quat::from_rotation_z(-std::f32::consts::TAU * sin_time);
}
