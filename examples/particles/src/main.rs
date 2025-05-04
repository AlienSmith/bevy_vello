//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

use std::time::Duration;

use bevy::{ecs::entity, prelude::*};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_hanabi::prelude::*;

use bevy::asset::AssetMetaCheck;
use bevy_vello::{
    add_default_light,
    integrations::{HanabiIntegrationPlugin, VelloSceneSubBundle},
    vello::{
        kurbo,
        peniko::{self, GlowColor},
        scene::StorkeExpand,
    },
};
use bevy_vello::{prelude::*, VelloPlugin};
use ron::value::Float;

#[derive(Clone, Default, Component)]
pub struct ExplosionFading {
    timer: Timer,
    init_color: Vec3,
    end_color: Vec3,
    particle_scales: f32,
}

const BOUNDS: Vec2 = Vec2::new(1200.0, 640.0);

const DEFAULT_PARTICLES: &[u8] = include_bytes!("../2d_default.particles");

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

    effect: Option<Handle<EffectAsset>>,
}
//Notic without "meta_check: AssetMetaCheck::Never" bevy would complain about the HanabiNode.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = App::default();
    app.insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    ..default()
                })
                .set(bevy::log::LogPlugin {
                    // Uncomment this to override the default log settings:
                    // level: bevy::log::Level::TRACE,
                    // filter: "wgpu=warn,bevy_ecs=info".to_string(),
                    ..default()
                }),
        )
        .add_plugins(HanabiIntegrationPlugin);

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.add_plugins(VelloPlugin)
        .add_systems(Startup, setup_vector_graphics)
        .add_systems(Startup, add_default_light)
        .add_systems(Update, player_control_system)
        .add_systems(Update, simple_animation)
        .run();

    Ok(())
}

fn make_default_rect_particles(scene: &mut VelloScene) {
    use vello::kurbo::*;
    let color = GlowColor {
        color: peniko::Color::rgb(0.5, 0.0, 0.0),
        glow: 4.0,
    };
    *scene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        color,
        None,
        &Circle::new(Point { x: -5.0, y: 0.0 }, 10.0),
    );
}

fn _make_default_effect() -> EffectAsset {
    let mut gradient = Gradient::new();
    gradient.add_key(0.0, Vec4::new(0.5, 0.5, 1.0, 1.0));
    gradient.add_key(1.0, Vec4::new(0.5, 0.5, 1.0, 0.0));

    let writer = ExprWriter::new();

    let age = writer.lit(0.).expr();
    let init_age = SetAttributeModifier::new(Attribute::AGE, age);

    let lifetime = writer.lit(2.0).expr();
    let init_lifetime = SetAttributeModifier::new(Attribute::LIFETIME, lifetime);

    let init_pos = SetPositionCircleModifier {
        center: writer.lit(Vec3::ZERO).expr(),
        axis: writer.lit(Vec3::Z).expr(),
        radius: writer.lit(0.05).expr(),
        dimension: ShapeDimension::Surface,
    };

    let speed = writer.add_property("speed", Value::Scalar(ScalarValue::Float(100.0)));
    let speed = writer.prop(speed);

    let init_vel = SetVelocityCircleModifier {
        center: writer.lit(Vec3::ZERO).expr(),
        axis: writer.lit(Vec3::Z).expr(),
        speed: (writer.rand(ValueType::Scalar(ScalarType::Float))
            * (writer.lit(3.0)
                - writer.lit(2.0) * writer.rand(ValueType::Scalar(ScalarType::Float)))
            * speed)
            .expr(),
    };

    let drag = writer.add_property("drag", Value::Scalar(ScalarValue::Float(4.0)));
    let drag = writer.prop(drag).expr();

    let update_drag = LinearDragModifier::new(drag);

    let module = writer.finish();

    let spawner = Spawner::once(255.0.into(), true);
    EffectAsset::new(vec![2048], spawner, module)
        .with_name("2d_default")
        .init(init_pos)
        .init(init_vel)
        .init(init_age)
        .init(init_lifetime)
        .update(update_drag)
        .render(SizeOverLifetimeModifier {
            gradient: Gradient::constant(Vec2::splat(2.0)),
            screen_space_size: false,
        })
        .with_simulation_space(SimulationSpace::Local)
        .build()
}

fn default_effect(effects: &mut ResMut<Assets<EffectAsset>>) -> Handle<EffectAsset> {
    //let custom_asset = ron::de::from_bytes::<EffectAsset>(&DEFAULT_PARTICLES).unwrap();
    let custom_asset = _make_default_effect();
    effects.add(
        custom_asset, // .render(ColorOverLifetimeModifier { gradient })
                      // .render(round),
    )
}

fn spawn_particles_at(
    commands: &mut Commands,
    effect: Handle<EffectAsset>,
    translate: Vec3,
    particle_index: u32,
) {
    // Create a color gradient for the particles
    let mut scene = VelloScene::default();
    make_default_rect_particles(&mut scene);
    // Spawn an instance of the particle effect, and override its Z layer to
    // be above the reference white square previously spawned.
    bevy::log::info!("asset {:?}", effect);
    let mut ep1 = EffectProperties::default();
    ep1.set("speed", (60.0).into());

    let mut ep0 = EffectProperties::default();
    ep0.set("speed", (140.0).into());

    let mut scene = VelloScene::default();
    make_default_rect_particles(&mut scene);
    commands.spawn((
        ParticleEffectBundle {
            // Assign the Z layer so it appears in the egui inspector and can be modified at runtime
            effect: ParticleEffect::new(effect.clone()).with_z_layer_2d(Some(0.1)),
            transform: Transform::from_translation(Vec3 {
                x: translate.x,
                y: translate.y,
                z: 0.0,
            }),
            effect_properties: ep0,
            ..default()
        },
        VelloSceneSubBundle {
            scene,
            ..Default::default()
        },
        ExplosionFading {
            timer: Timer::from_seconds(2.0, TimerMode::Once),
            init_color: Vec3::new(3.0, 1.8, 0.6),
            end_color: Vec3::new(0.5, 0.5, 0.5),
            particle_scales: 10.0,
        },
    ));

    // let mut scene = VelloScene::default();
    // make_default_rect_particles(&mut scene);
    // commands.spawn((
    //     ParticleEffectBundle {
    //         // Assign the Z layer so it appears in the egui inspector and can be modified at runtime
    //         effect: ParticleEffect::new(effect.clone()).with_z_layer_2d(Some(0.1)),
    //         transform: Transform::from_translation(Vec3 {
    //             x: translate.x,
    //             y: translate.y,
    //             z: 1.0,
    //         }),
    //         ..Default::default()
    //     },
    //     VelloSceneSubBundle {
    //         scene,
    //         ..Default::default()
    //     },
    //     ExplosionFading {
    //         timer: Timer::from_seconds(2.0, TimerMode::Once),
    //         init_color: Vec3::new(1.0, 1.0, 1.0),
    //         end_color: Vec3::new(0.2, 0.2, 0.2),
    //         particle_scales: 15.0,
    //     },
    // ));
    // let mut scene = VelloScene::default();
    // make_default_rect_particles(&mut scene);
    // commands.spawn((
    //     ParticleEffectBundle {
    //         // Assign the Z layer so it appears in the egui inspector and can be modified at runtime
    //         effect: ParticleEffect::new(effect).with_z_layer_2d(Some(0.1)),
    //         transform: Transform::from_translation(Vec3 {
    //             x: translate.x,
    //             y: translate.y,
    //             z: 2.0,
    //         }),
    //         effect_properties: ep1,
    //         ..default()
    //     },
    //     VelloSceneSubBundle {
    //         scene,
    //         ..Default::default()
    //     },
    //     ExplosionFading {
    //         timer: Timer::from_seconds(2.0, TimerMode::Once),
    //         init_color: Vec3::new(3.0, 1.8, 0.6),
    //         end_color: Vec3::new(0.5, 0.5, 0.5),
    //         particle_scales: 6.0,
    //     },
    // ));
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
            spawn_count_limits: 99,
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
            if ship.effect.is_none() {
                ship.effect = Some(default_effect(&mut effects));
            }
            let effect = ship.effect.as_ref().unwrap().clone();
            spawn_particles_at(
                &mut commands,
                effect,
                transform.translation,
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

#[cfg(test)]
mod test {
    use ron::ser::PrettyConfig;
    use std::fs::File;
    use std::io::Write;
    #[test]
    fn export_default_effect() {
        let effect = crate::_make_default_effect();
        let s = ron::ser::to_string_pretty(&effect, PrettyConfig::new().new_line("\n".to_string()))
            .unwrap();
        let mut file = File::create("2d_default.particles").unwrap();
        file.write_all(s.as_bytes()).unwrap();
    }
}

fn simple_animation(
    mut commands: Commands,
    mut query_scene: Query<(&mut VelloScene, &mut ExplosionFading, Entity)>,
    time: Res<Time>,
) {
    for (mut scene, mut e_timer, entity) in query_scene.iter_mut() {
        e_timer.timer.tick(time.delta());
        let time = e_timer.timer.elapsed_secs() / 2.0;
        let (radius, alpha) = if time < 0.7 {
            (e_timer.particle_scales, 1.0)
        } else {
            let tt = (time - 0.7) / 0.3;
            let ttt = tt * tt * (3.0 - 2.0 * tt);
            let r = e_timer.particle_scales * 0.5 + e_timer.particle_scales * 0.5 * (1.0 - ttt);
            (r, (1.0 - ttt))
        };
        let color = e_timer.init_color * (1.0 - time) + e_timer.end_color * time;
        let glow = color.length();
        let color = GlowColor {
            color: peniko::Color::rgba(
                (color.x / glow) as f64,
                (color.y / glow) as f64,
                (color.z / glow) as f64,
                alpha.into(),
            ),
            glow,
        };
        if let Some((particle_index, particle_size)) = scene.get_instance_index_in_export_buffer() {
            *scene = VelloScene::default();
            scene.fill(
                peniko::Fill::NonZero,
                kurbo::Affine::default(),
                color,
                None,
                &kurbo::Circle::new(kurbo::Point { x: -5.0, y: 0.0 }, radius.into()),
            );
            scene.set_instance_index_in_export_buffer(particle_index, particle_size);
        }
        if e_timer.timer.finished() {
            commands.entity(entity).despawn();
            info!("particle despawned");
        }
    }
}
