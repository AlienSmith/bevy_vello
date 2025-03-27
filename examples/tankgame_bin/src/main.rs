use std::str;

use bevy::{asset::AssetMetaCheck, prelude::*};
use bevy_vello::{
    add_default_light,
    integrations::HanabiIntegrationPlugin,
    prelude::*,
    vello::{kurbo::Affine, peniko::PBRImages},
    VelloPlugin, VelloSceneRepalyer,
};
use tankgame_lib::{
    init_particles_player,
    tank::{self, shell::update_shell},
    update_particle_scene, ParticlesPlayer,
};

const BASE: &[u8] = include_bytes!("../base.txt");
const GUN: &[u8] = include_bytes!("../gun.txt");
const TURRENT: &[u8] = include_bytes!("../turrent.txt");

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .add_plugins(HanabiIntegrationPlugin)
        .add_plugins(VelloPlugin)
        .insert_resource(ParticlesPlayer::default())
        .add_systems(Startup, setup_vector_graphics)
        .add_systems(Startup, add_default_light)
        .add_systems(Startup, init_particles_player)
        .add_systems(Update, tank::base::control_system)
        .add_systems(Update, tank::turrent::control_system)
        .add_systems(Update, tank::gun::control_system)
        .add_systems(Update, update_particle_scene)
        .add_systems(Update, update_shell)
        .run();
    bevy::log::warn!("Initialize");
}

fn setup_vector_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
    let (b_s, b_r) = spawn_pbr_scene(BASE);
    let (g_s, g_r) = spawn_pbr_scene(GUN);
    let (t_s, t_r) = spawn_pbr_scene(TURRENT);
    commands
        .spawn((
            VelloSceneBundle {
                scene: b_s,
                transform: Transform::from_translation(Vec3::new(0., 0., 0.)),
                ..Default::default()
            },
            b_r,
            tank::base::Base::new(200.0, 50.0),
        ))
        .with_children(|parent| {
            parent
                .spawn((
                    VelloSceneBundle {
                        scene: t_s,
                        transform: Transform::from_translation(Vec3::new(0., 0., 2.)),
                        ..Default::default()
                    },
                    t_r,
                    tank::turrent::Turrent::new(20.0),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        VelloSceneBundle {
                            scene: g_s,
                            transform: Transform::from_translation(Vec3::new(0., 0., -1.)),
                            ..Default::default()
                        },
                        g_r,
                        tank::gun::Gun::default(),
                    ));
                });
        });
}

fn spawn_pbr_scene(data: &[u8]) -> (VelloScene, VelloSceneRepalyer) {
    let mut replayer = vello::SceneReplayer::default();
    let mut scene = vello::Scene::new();
    replayer.load_trace_to_scene(&mut scene, data);
    let replayer: VelloSceneRepalyer = replayer.into();
    let scene: VelloScene = scene.into();
    (scene, replayer)
}

fn _export_default_pbr_scene(albedo: &[u8], normal: &[u8], metallic: f32, roughness: f32) {
    let base_image = vello::decode_image(albedo).unwrap();
    let base_normal_image = vello::decode_image(normal).unwrap();
    let width = base_image.width as f64;
    let height = base_image.height as f64;
    let pbr = PBRImages::new(base_image, base_normal_image, metallic, roughness);
    let mut scene = VelloScene::default();
    scene.write_trace = true;
    scene.draw_image_with_normal(&pbr, Affine::translate((-0.5 * width, -0.5 * height)));
}

fn _export_default_pbr_scene_with_name(name: &str, metallic: f32, roughness: f32) {
    use std::env;
    let exe_dir = env::current_dir()
        .expect("Failed to get executable path")
        .parent()
        .expect("Failed to get parent directory")
        .to_path_buf();
    let temp = exe_dir.join("assets\\pbr");

    let base_name = format!("{}{}", name, ".png");
    let normal_name = format!("{}{}", name, "_normal.png");

    let base = temp.join(base_name);
    let normal = temp.join(normal_name);
    println!("{:?},\n {:?}", base, normal);
    use std::fs;
    let tank = fs::read(base).expect("Failed to read file");
    let tank_normals = fs::read(normal).expect("Failed to read file");
    _export_default_pbr_scene(&tank, &tank_normals, metallic, roughness);
}

#[test]
fn export_default_effect() {
    _export_default_pbr_scene_with_name("turrent", 0.1, 0.5);
}
