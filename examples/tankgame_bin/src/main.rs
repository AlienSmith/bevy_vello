use bevy::prelude::*;
use bevy::{asset::AssetMetaCheck, utils::HashMap};
use bevy_vello::{
    add_default_light,
    integrations::HanabiIntegrationPlugin,
    prelude::*,
    vello::{kurbo::Affine, peniko::PBRImages},
    VelloPlugin,
};
use std::str;
use tankgame_lib::{
    init_particles_player,
    tank::{self, shell::update_shell},
    update_particle_scene, ParticlesPlayer,
};

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(C)]
pub struct TankPartsType(pub u32);

impl TankPartsType {
    pub const BASE: Self = Self(0);
    pub const TURRENT: Self = Self(1);
    pub const GUN: Self = Self(2);
}

impl From<TankPartsType> for usize {
    fn from(value: TankPartsType) -> Self {
        value.0 as usize
    }
}

#[derive(Resource, Clone, Default)]
pub struct TankParts {
    pub id_to_index: HashMap<AssetId<VelloReplaySceneAsset>, usize>,
    pub parts: Vec<Handle<VelloReplaySceneAsset>>,
    pub load_state: Vec<bool>,
}

impl TankParts {
    pub fn push(&mut self, handle: Handle<VelloReplaySceneAsset>) {
        let index = self.parts.len();
        let id = handle.id();
        self.id_to_index.insert(id, index);
        self.parts.push(handle);
        self.load_state.push(false);
    }

    pub fn get_index<T: Into<usize>>(&self, index: T) -> Option<Handle<VelloReplaySceneAsset>> {
        let index = index.into();
        if self.parts.len() > index && self.load_state[index] {
            return Some(self.parts[index].clone());
        }
        None
    }

    pub fn mark_as_loaded(&mut self, id: &AssetId<VelloReplaySceneAsset>) {
        if let Some(index) = self.id_to_index.get(id) {
            self.load_state[*index] = true;
        }
    }

    pub fn all_loaded(&self) -> bool {
        self.load_state.iter().all(|&loaded| loaded)
    }
}

#[derive(Clone, Eq, PartialEq, Debug, Hash, Default, States)]
enum GameState {
    #[default]
    Loading,
    Game,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .add_plugins(HanabiIntegrationPlugin)
        .add_plugins(VelloPlugin)
        .insert_resource(ParticlesPlayer::default())
        .insert_resource(TankParts::default())
        .init_state::<GameState>()
        .add_systems(Startup, setup_tank_parts)
        .add_systems(
            Update,
            check_assets_loaded.run_if(in_state(GameState::Loading)),
        )
        .add_systems(
            OnEnter(GameState::Game),
            (
                setup_vector_graphics,
                add_default_light,
                init_particles_player,
            ),
        )
        .add_systems(
            Update,
            (
                tank::base::control_system,
                tank::turrent::control_system,
                tank::gun::control_system,
                update_particle_scene,
                update_shell,
            )
                .run_if(in_state(GameState::Game)),
        )
        .run();
    bevy::log::warn!("Initialize");
}

fn check_assets_loaded(
    mut ev_asset: EventReader<AssetEvent<VelloReplaySceneAsset>>,
    mut tank_parts: ResMut<TankParts>,
    mut next_state: ResMut<NextState<GameState>>,
) {
    for ev in ev_asset.read() {
        match ev {
            AssetEvent::LoadedWithDependencies { id } => {
                tank_parts.mark_as_loaded(id);
                if tank_parts.all_loaded() {
                    next_state.set(GameState::Game);
                }
            }
            _ => {}
        }
    }
}

fn setup_tank_parts(mut tank_parts: ResMut<TankParts>, asset_server: Res<AssetServer>) {
    tank_parts.push(asset_server.load("scenes/base.scene"));
    tank_parts.push(asset_server.load("scenes/turrent.scene"));
    tank_parts.push(asset_server.load("scenes/gun.scene"));
}

fn setup_vector_graphics(
    mut commands: Commands,
    parts: ResMut<TankParts>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    commands.spawn(Camera2dBundle::default());
    let mut b_s = VelloScene::default();
    custom_assets
        .get(&parts.get_index(TankPartsType::BASE).unwrap())
        .unwrap()
        .player
        .apply_to_scene(&mut b_s);

    let mut t_s = VelloScene::default();
    custom_assets
        .get(&parts.get_index(TankPartsType::TURRENT).unwrap())
        .unwrap()
        .player
        .apply_to_scene(&mut t_s);

    let mut g_s = VelloScene::default();
    custom_assets
        .get(&parts.get_index(TankPartsType::GUN).unwrap())
        .unwrap()
        .player
        .apply_to_scene(&mut g_s);

    commands
        .spawn((
            VelloSceneBundle {
                scene: b_s,
                transform: Transform::from_translation(Vec3::new(0., 0., 0.)),
                ..Default::default()
            },
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
                    tank::turrent::Turrent::new(20.0),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        VelloSceneBundle {
                            scene: g_s,
                            transform: Transform::from_translation(Vec3::new(0., 0., -1.)),
                            ..Default::default()
                        },
                        tank::gun::Gun::default(),
                    ));
                });
        });
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
    let temp = exe_dir.join("tankgame_bin\\assets\\pbr");

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
