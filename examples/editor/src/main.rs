//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

use bevy::{color::palettes::css::WHITE, prelude::*};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy::asset::AssetMetaCheck;
use bevy_vello::dock::{
    commands::{DockCommand, DockCommandResult, EntityType},
    stream_factory::dock_push_commands,
    DockPlugin,
};
use bevy_vello::{add_default_light, VelloPlugin};
use futures::{channel::oneshot, FutureExt};
use std::sync::{Arc, Mutex};

const SVG_DATA: &[u8] = include_bytes!("./assets/fountain.svg");
const LOTTIE_DATA: &[u8] = include_bytes!("./assets/tiger.json");
const DEFAULT_PARTICLES: &[u8] = include_bytes!("./assets/2d_default.particles");
#[derive(Resource)]
struct SVGReceivers {
    svg_r: Arc<Mutex<oneshot::Receiver<DockCommandResult>>>,
}

#[derive(Resource)]
struct ParticleRecievers {
    p_r: Arc<Mutex<oneshot::Receiver<DockCommandResult>>>,
}

#[derive(Resource)]
struct LOTTIEReceivers {
    lottie_r: Arc<Mutex<oneshot::Receiver<DockCommandResult>>>,
}

#[derive(Resource)]
struct ParticleVelloIndexes {
    svg_id: u32,
    particle_id: u32,
    spawned: bool,
}

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
        .add_plugins(DockPlugin)
        .add_plugins(VelloPlugin)
        .add_systems(Startup, setup_vector_graphics)
        .add_systems(Startup, add_default_light)
        .add_systems(Update, another_particle_system)
        .add_systems(Update, particle_system)
        .add_systems(Update, svg_system)
        //.add_systems(Update, lottie_system)
        //.add_systems(Update, draw_cursor)
        .run();

    Ok(())
}

fn setup_vector_graphics(mut commands: Commands) {
    let svg_r = dock_push_commands(DockCommand::LoadSVGAssets(SVG_DATA.to_vec()));
    let lottie_r = dock_push_commands(DockCommand::LoadLottieAssets(LOTTIE_DATA.to_vec()));
    commands.insert_resource(SVGReceivers {
        svg_r: Arc::new(Mutex::new(svg_r)),
    });
    commands.insert_resource(LOTTIEReceivers {
        lottie_r: Arc::new(Mutex::new(lottie_r)),
    });
    //commands.spawn(Camera2dBundle::default());
    let particle_r =
        dock_push_commands(DockCommand::LoadParticleAssets(DEFAULT_PARTICLES.to_vec()));
    commands.insert_resource(ParticleRecievers {
        p_r: Arc::new(Mutex::new(particle_r)),
    });
    commands.insert_resource(ParticleVelloIndexes {
        svg_id: 0,
        particle_id: 0,
        spawned: false,
    });
}

fn another_particle_system(mut pv: ResMut<ParticleVelloIndexes>) {
    if !pv.spawned && pv.svg_id != 0 && pv.particle_id != 0 {
        pv.spawned = true;
        let _ = dock_push_commands(DockCommand::SpawnEntity(
            pv.particle_id,
            Transform::from_translation(Vec3 {
                x: 50.0,
                y: 50.0,
                z: 1.0,
            }),
            EntityType::Particle,
            pv.svg_id,
        ));
    }
}

/// Demonstrates applying rotation and movement based on keyboard input.
fn particle_system(
    mut commands: Commands,
    particle_recievers: Option<ResMut<ParticleRecievers>>,
    mut pv: ResMut<ParticleVelloIndexes>,
) {
    if let Some(l_rs) = particle_recievers {
        let p_rs = &mut *l_rs.p_r.lock().unwrap();
        if let Some(r) = p_rs.now_or_never() {
            match r {
                Ok(v) => {
                    commands.remove_resource::<ParticleRecievers>();
                    match v {
                        DockCommandResult::Ok(index) => {
                            println!("particle loaded with value {}", index);
                            pv.particle_id = index;
                            // let effect = dock_get_particle_asset_with_id(index);
                            // let _ = dock_push_commands(DockCommand::SpawnEntity(
                            //     index,
                            //     Transform::from_translation(Vec3 {
                            //         x: 0.0,
                            //         y: 0.0,
                            //         z: 1.0,
                            //     }),
                            //     EntityType::Particle,
                            //     0,
                            // ));
                            // spawn_particles_at(
                            //     &mut commands,
                            //     effect,
                            //     transform.translation,
                            //     ship.spawn_count,
                            // );
                            // ship.spawn_count += 1;
                            // ship.last_spawn_time = time.elapsed_seconds();
                        }
                        DockCommandResult::NotOk(s) => {
                            println!("{}", s);
                        }
                    }
                }
                Err(_) => println!("lottie not loaded"),
            }
        } else {
            // If now_or_never returns None, the sender has been dropped without sending a value.
            println!("The sender was dropped without sending a value.");
        }
    }
}

fn svg_system(
    mut commands: Commands,
    svg_recivers: Option<ResMut<SVGReceivers>>,
    mut pv: ResMut<ParticleVelloIndexes>,
) {
    if let Some(rcs) = svg_recivers {
        let rs = &mut *rcs.svg_r.lock().unwrap();
        if let Some(value) = rs.now_or_never() {
            match value {
                Ok(v) => {
                    commands.remove_resource::<SVGReceivers>();
                    match v {
                        DockCommandResult::Ok(index) => {
                            println!("svg loaded with value {}", index);
                            pv.svg_id = index;
                            // let _ = dock_push_commands(DockCommand::SpawnEntity(
                            //     index,
                            //     Transform::from_translation(Vec3 {
                            //         x: 100.0,
                            //         y: 100.0,
                            //         z: 0.0,
                            //     }),
                            //     EntityType::Vello,
                            //     0,
                            // ));
                            // let _ = dock_push_commands(DockCommand::PickEntity(
                            //     Vec2 {
                            //         x: -100.0,
                            //         y: -100.0,
                            //     },
                            //     100.0,
                            // ));
                        }
                        DockCommandResult::NotOk(s) => {
                            println!("{}", s);
                        }
                    }
                }
                Err(_) => {
                    println!("svg not loaded");
                }
            }
        } else {
            // If now_or_never returns None, the sender has been dropped without sending a value.
            println!("The sender was dropped without sending a value.");
        }
    }
}

fn lottie_system(mut commands: Commands, lottie_recivers: Option<ResMut<LOTTIEReceivers>>) {
    if let Some(l_rs) = lottie_recivers {
        let l_rs = &mut *l_rs.lottie_r.lock().unwrap();
        if let Some(r) = l_rs.now_or_never() {
            match r {
                Ok(v) => {
                    commands.remove_resource::<LOTTIEReceivers>();
                    match v {
                        DockCommandResult::Ok(index) => {
                            println!("lottie loaded with value {}", index);
                            let _ = dock_push_commands(DockCommand::SpawnEntity(
                                index,
                                Transform::from_translation(Vec3 {
                                    x: 100.0,
                                    y: 100.0,
                                    z: 0.0,
                                })
                                .with_scale(Vec3 {
                                    x: 0.5,
                                    y: 0.5,
                                    z: 1.0,
                                }),
                                EntityType::Vello,
                                0,
                            ));
                        }
                        DockCommandResult::NotOk(s) => {
                            println!("{}", s);
                        }
                    }
                }
                Err(_) => println!("lottie not loaded"),
            }
        } else {
            // If now_or_never returns None, the sender has been dropped without sending a value.
            println!("The sender was dropped without sending a value.");
        }
    }
}
fn draw_cursor(
    camera_query: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    mut gizmos: Gizmos,
) {
    let (camera, camera_transform) = camera_query.single();

    let Some(cursor_position) = windows.single().cursor_position() else {
        return;
    };

    // Calculate a world position based on the cursor's position.
    let Some(point) = camera.viewport_to_world_2d(camera_transform, cursor_position) else {
        return;
    };

    gizmos.circle_2d(point, 10., WHITE);
}
