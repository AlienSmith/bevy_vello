use bevy::{asset::AssetMetaCheck, prelude::*};
use bevy_vello::{
    dock::{commands::*, stream_factory::*, DockPlugin},
    VelloPlugin,
};
const SVG_DATA: &[u8] = include_bytes!("./assets/fountain.svg");
const LOTTIE_DATA: &[u8] = include_bytes!("./assets/tiger.json");
use futures::{channel::oneshot, FutureExt};
use std::sync::{Arc, Mutex};
//Reciver is Send but not sync, add mutex make a send type also sync
#[derive(Resource)]
struct SVGReceivers {
    svg_r: Arc<Mutex<oneshot::Receiver<DockCommandResult>>>,
}

#[derive(Resource)]
struct LOTTIEReceivers {
    lottie_r: Arc<Mutex<oneshot::Receiver<DockCommandResult>>>,
}

use bevy::color::palettes::css::WHITE;

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        ..default()
    }))
    .add_plugins(VelloPlugin)
    .add_plugins(DockPlugin)
    .add_systems(Startup, receive)
    .add_systems(Update, recieve_check)
    .add_systems(Update, draw_cursor);
    app.run();
}

fn receive(mut commands: Commands) {
    let svg_r = dock_push_commands(DockCommand::LoadSVGAssets(SVG_DATA.to_vec()));
    let lottie_r = dock_push_commands(DockCommand::LoadLottieAssets(LOTTIE_DATA.to_vec()));
    commands.insert_resource(SVGReceivers {
        svg_r: Arc::new(Mutex::new(svg_r)),
    });
    commands.insert_resource(LOTTIEReceivers {
        lottie_r: Arc::new(Mutex::new(lottie_r)),
    });
}

fn recieve_check(
    mut commands: Commands,
    svg_recivers: Option<ResMut<SVGReceivers>>,
    lottie_recivers: Option<ResMut<LOTTIEReceivers>>,
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
                            for i in 0..10 {
                                for j in 0..10 {
                                    let _ = dock_push_commands(DockCommand::SpawnEntity(
                                        index,
                                        Transform::from_translation(Vec3 {
                                            x: i as f32 * 50.0,
                                            y: j as f32 * 50.0,
                                            z: 0.0,
                                        }),
                                    ));
                                }
                            }
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
    if let Some(l_rs) = lottie_recivers {
        let l_rs = &mut *l_rs.lottie_r.lock().unwrap();
        if let Some(r) = l_rs.now_or_never() {
            match r {
                Ok(v) => {
                    commands.remove_resource::<LOTTIEReceivers>();
                    match v {
                        DockCommandResult::Ok(index) => {
                            println!("lottie loaded with value {}", index);
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
