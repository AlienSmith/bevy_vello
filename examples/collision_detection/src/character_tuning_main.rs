//! Standalone character-tuning app.
//!
//! A slimmed-down variant of the collision_detection demo, with all UI
//! (egui), mouse-drag sandbox, and preview code stripped out. It keeps only
//! the character, the static collision walls, the light, and the physics
//! chain — so the character can be driven/tuned via input and env-gated
//! experiments (e.g. `VELLO_TICK_DRAG`).
//!
//! Run:
//!   cargo run -p collision_detection --bin character_tuning_main --release

mod cannon;
mod edge_pan_camera;

use bevy::{asset::AssetMetaCheck, prelude::*, window::PresentMode};
use bevy_egui::{egui, EguiContexts, EguiPlugin};

use bevy_vello::integrations::physics::systems::update_constraint_world;
use bevy_vello::{
    collision::{
        generate_uvs, path_to_ccw_quad_path, CollisionConstraintConfig, CollisionSystems,
        SoftBodyInitConfig, VelloCollisionTrigger, VELLO_COLLISION_COOL_DOWN_TIME,
    },
    integrations::{
        particles::{self, ExplosionEffect},
        physics::{VelloConstraintWorld, VelloParticle},
        svg_collider::{
            SvgColliderAsset, SvgColliderAssetManager, VelloColliderAssetMetaData, VelloImageAsset,
            VelloImageAssetManager, VelloImageAssetMetaData,
        },
    },
    vello::{
        kurbo::{self, BezPath, Shape},
        peniko::{self, GlowColor},
    },
    VelloCollider, VelloCollisionResponsePlugin,
};
use bevy_vello::{prelude::*, VelloPlugin};
use game_lib::{
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
    },
    default_input_map, drive_hip_pin, reset_hip_pin, update_character_movement,
    weapons::{AttachPistolToCharacterEvent, PistolControl},
    CharacterController, CharacterRoot, ColliderRoot, ConnectivityRoot, GameLabSystems, HipPin,
    HipPinParams, PlayerAction, PlayerMarker, SpineController, VelloCharacterPlugin,
};
use leafwing_input_manager::prelude::*;

const PLAYER_COLLISION_GROUP: u32 = 1;

/// Gravity applied when the toggle is on. Vello coordinates: x right, y down
/// (matches the XPBD default `GRAVITY` in `collision_response.rs`).
/// We are using 100px = 1m and the character in this scene is about 170px hight.
const GRAVITY_ON: Vec2 = Vec2::new(0.0, 980.0);

#[derive(Component)]
pub struct Player;

/// DEBUG EXPERIMENT (env-gated): a tick-native drag that replaces the spine
/// controller entirely.
///
/// When `VELLO_TICK_DRAG=1`, every FixedUpdate tick pulls the spine particles
/// toward `DRAG_VY` (px/s) with per-tick blend `DRAG_GAIN`. No per-frame event
/// bridge — the drag lives entirely inside the fixed step, so its behavior
/// cannot depend on render fps.
fn tick_drag(
    spine_q: Query<&SpineController>,
    particle_q: Query<&VelloParticle>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    roots: Query<Entity, With<CharacterRoot>>,
) {
    if std::env::var("VELLO_TICK_DRAG").as_deref() != Ok("1") {
        return;
    }
    let target_y: f32 = std::env::var("DRAG_VY")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(500.0);
    let gain: f32 = std::env::var("DRAG_GAIN")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.1);
    let Ok(root) = roots.single() else {
        return;
    };
    // Match each physics frame particle to the mirrored Bevy particle by
    // position (the mirror is one sync stale, which is fine for a drag).
    let Some(infos) = constraint_world.frame_info(root) else {
        return;
    };
    for info in &infos {
        // find the mirrored velocity for this physics particle
        let mut vel = None;
        for spine in spine_q.iter() {
            for e in spine.particles.iter() {
                if let Ok(vp) = particle_q.get(*e) {
                    let p = vp.particle.pos;
                    if (p.x - info.pos_x).abs() < 1.0 && (p.y - info.pos_y).abs() < 1.0 {
                        vel = Some(vp.particle.velocity);
                        break;
                    }
                }
            }
            if vel.is_some() {
                break;
            }
        }
        let Some(v) = vel else { continue };
        // Δv per tick toward the target; ExternalForce::Impulse applies
        // delta_pos = impulse * inv_mass * dt, so impulse = Δv / inv_mass.
        let dvx = (0.0 - v.x) * gain;
        let dvy = (target_y - v.y) * gain;
        constraint_world.queue_external_force(
            root,
            vello_physics::soft_body::ExternalForce::Impulse(
                info.index,
                dvx / info.inv_mass.max(f32::EPSILON),
                dvy / info.inv_mass.max(f32::EPSILON),
            ),
        );
    }
}

#[derive(Clone, Eq, PartialEq, Debug, Hash, Default, States)]
enum GameState {
    #[default]
    Loading,
    Game,
}

/// Tracks whether the pistol collider has been registered to the character.
/// Used to guard the UnregisterPart event so we don't try to unregister twice.
#[derive(Resource, Default)]
struct PistolState {
    registered: bool,
    character: Option<Entity>,
    pistol: Option<Entity>,
}

/// Open/close toggle for gravity. Kept as a standalone resource so the UI
/// checkbox can persist its state across frames and the world gravity only
/// changes when the user actually flips the checkbox.
#[derive(Resource)]
struct GravityToggle {
    enabled: bool,
    /// Downward acceleration magnitude (px/s², vello `+y`). Matches `GRAVITY_ON.y`.
    magnitude: f32,
}

impl Default for GravityToggle {
    fn default() -> Self {
        // On by default: the physics plugin boots the world at zero gravity, so
        // `apply_default_gravity` (Startup) must push this into the world — the
        // checkbox only writes on a *change*, so without it the box would read
        // "checked" while the character still floats.
        Self {
            enabled: true,
            magnitude: GRAVITY_ON.y,
        }
    }
}
//cargo run --package collision_detection --bin character_tuning_main --release
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = App::default();
    app.insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Vello Character Tuning".into(),
                        // PresentMode::AutoNoVsync is usually the best for Linux/NVIDIA.
                        present_mode: PresentMode::AutoNoVsync,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    ..default()
                })
                .set(bevy::log::LogPlugin { ..default() }),
        )
        .init_state::<GameState>()
        .add_plugins(VelloCharacterPlugin::default())
        .insert_resource(PistolState::default())
        .insert_resource(cannon::CannonParams::default())
        .insert_resource(GravityToggle::default())
        // Same env-driven controller as the headless harness: `PIN_MODE`,
        // `KV`, `HIP_DROP`, `TARGET_SPEED`, `LEGS`. Default = rigid pin.
        .insert_resource(HipPinParams::from_env())
        .add_plugins(EguiPlugin {
            enable_multipass_for_primary_context: false,
        })
        .add_plugins(VelloPlugin)
        .add_plugins(VelloCollisionResponsePlugin)
        .add_plugins(particles::VelloPartclePlugin)
        .add_systems(
            FixedUpdate,
            tick_drag.before(CollisionSystems::CollisionResponsePhysics),
        )
        // Hip pin: step the Raibert resolver, queue the three P1/P2/P3 external
        // position constraints, and (when `hip_drives_legs`) write the foot
        // targets for the legs — all *before* `update_character_movement` runs
        // the leg IK, and before the constraint world is stepped.
        .add_systems(
            FixedUpdate,
            drive_hip_pin
                .before(update_character_movement)
                .before(update_constraint_world),
        )
        .add_systems(Startup, setup_back_ground)
        .add_systems(Startup, add_light)
        .add_systems(Startup, setup_resources)
        // The world boots at zero gravity; open it now to match the default.
        .add_systems(Startup, apply_default_gravity)
        .add_systems(Startup, cannon::spawn_cannon_indicator)
        .add_systems(
            Update,
            check_assets_loaded.run_if(in_state(GameState::Loading)),
        )
        .add_systems(
            OnEnter(GameState::Game),
            (
                setup_entity,
                setup_pistol
                    .after(setup_entity)
                    .before(GameLabSystems::WriteCharacterPartEvent),
            ),
        )
        .add_systems(
            Update,
            (
                edge_pan_camera::update_edge_pan_camera,
                arm_tuning_ui,
                gravity_toggle_ui,
                cannon::cannon_ui,
                cannon::move_cannon_with_wasd,
                cannon::update_cannon_indicator,
                cannon::fire_cannon,
                hip_pin_ui,
                reset_hip_pin,
            )
                .run_if(in_state(GameState::Game)),
        )
        .run();

    Ok(())
}

//make a white background
fn setup_back_ground(mut commands: Commands) {
    commands.spawn((
        Camera2d::default(),
        edge_pan_camera::EdgePanCamera {
            edge_margin: -10.0,
            ..Default::default()
        },
    ));
    let mut scene: VelloScene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgb(1.0, 1.0, 1.0),
        None,
        &kurbo::Rect::new(-2048.0, -2048.0, 2048.0, 2048.0),
    );

    commands.spawn((VelloSceneBundle {
        scene,
        ..Default::default()
    },));
}

fn setup_entity(mut commands: Commands, mut pistol_state: ResMut<PistolState>) {
    make_static_scene(&mut commands);
    let mut scene: VelloScene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgb(1.0, 0.0, 0.0),
        None,
        &kurbo::Rect::new(-10.0, -10.0, 10.0, 10.0),
    );

    pistol_state.character = Some(
        commands
            .spawn((
                VelloSceneBundle {
                    transform: Transform {
                        translation: Vec3::new(0.0, 0.0, 100.0),
                        scale: Vec3::new(0.5, 0.5, 1.0),
                        ..Default::default()
                    },
                    scene,
                    ..Default::default()
                },
                CharacterRoot {
                    svg_asset_id: "v10.character.svg".to_owned(),
                    blueprint_asset_id: "v10.character.json".to_owned(),
                    collision_group: PLAYER_COLLISION_GROUP,
                },
                CharacterController {
                    move_vector: Vec2::ZERO,
                    point_vector: Vec2::ZERO,
                    ..Default::default()
                },
                Player,
                PlayerMarker,
                default_input_map(),
                ActionState::<PlayerAction>::default(),
                HipPin::new(&HipPinParams::from_env().config),
            ))
            .id(),
    );

    let _player_entity = pistol_state
        .character
        .expect("player must be spawned first");
}

/// Spawn a pistol collider and connect it to the character's PRLA particle
/// via a bilinear joint. Runs after [`setup_entity`] so the character and its
/// particles are fully assembled.
fn setup_pistol(
    mut commands: Commands,
    mut events: EventWriter<AttachPistolToCharacterEvent>,
    character_q: Query<(Entity, &ConnectivityRoot), With<CharacterRoot>>,
    mut pistol_state: ResMut<PistolState>,
) {
    // There is only one character — get its entity and ConnectivityRoot.
    let (character_entity, _root) = character_q.get(pistol_state.character.unwrap()).unwrap();

    let soft_body_init_transform = Transform {
        translation: Vec3::new(325.0, -90.0, 0.0),
        rotation: Quat::from_rotation_z(0.0_f32.to_radians()),
        scale: Vec3::new(0.1, 0.1, 1.0),
    };

    let softbody_config = SoftBodyInitConfig::default();

    let pistol_entity = commands
        .spawn((
            VelloSceneBundle {
                ..Default::default()
            },
            ColliderRoot {
                svg_asset_id: "pistol.collider.svg".to_string(),
                albedo_asset_id: "pistol_albedo.png".to_string(),
                normal_asset_id: "pistol_normal.png".to_string(),
                metallic: 0.9,
                roughness: 0.2,
                softbody_config,
                collision_config: CollisionConstraintConfig::default(),
                soft_body_init_transform,
                initial_velocity: Vec2::ZERO,
                collision_group: PLAYER_COLLISION_GROUP,
                collision_inverse_mass: softbody_config.total_inv_mass,
            },
            PistolControl {
                wrist_binding_uv: Vec2::new(0.15, 0.75),
                gun_point_uv: Vec2::new(1.0, 0.125),
                ..Default::default()
            },
        ))
        //.observe(on_collision_spawn_particle)
        .id();
    events.write(AttachPistolToCharacterEvent {
        character: character_entity,
        pistol: pistol_entity,
    });
    pistol_state.registered = true;
    pistol_state.pistol = Some(pistol_entity);
}

fn make_static_scene(commands: &mut Commands) {
    let make_long_rect = || {
        let rect: kurbo::Rect = kurbo::Rect::new(-1940.0, -20.0, 1940.0, 20.0);
        let rect_path = rect.to_path(0.1);
        (rect_path, rect)
    };
    //aabb will only take the effect of position ignoring entity rotation and scale.
    //in other words if your static collider contains rotation or scaling you need to account for that
    let make_short_rect = || {
        let rect = kurbo::Rect::new(-20.0, -1120.0, 20.0, 1120.0);
        let rect_path = rect.to_path(0.1);
        (rect_path, rect)
    };
    // GROUND WALL DISABLED for the hip-pin bridge (see hip_pin_integration.md
    // §7.2). The character stands at world y≈0, so this wall at y=540 is the
    // true "ground"; the pin places the rigid P1/P2/P3 frame, so foot↔floor
    // collision would fight it. Dropping it scene-locally is enough — the leg
    // soft-body colliders share the character-wide group and never self-collide,
    // and the joint graph is untouched.
    //
    // make_static_collision_shape(
    //     commands,
    //     Vec4::new(0.0, 540.0, 0.0, 1.0),
    //     make_long_rect,
    //     GlowColor {
    //         color: peniko::Color::rgb(1.0, 0.0, 0.0),
    //         glow: 1.0,
    //     },
    //     Vec2::new(0.0, 0.0),
    //     0.0,
    //     false,
    // );

    // Ground line matching the headless walk.
    //
    // `assemble_character` bakes the root transform into the particles
    // (`transform_to_affine`, observers.rs), so the root's scale 0.5 is already
    // applied there. Blueprint feet are y=558, and the headless live stance
    // reports the foot particles at Vello y=+279 (hip +173, stance 106) — that
    // is the *world* physics position, not a pre-scale value. Static colliders
    // are authored in Bevy space, and `mat4_to_affine` flips the y translation
    // when building the physics frame, so the matching Bevy surface is -279.
    //
    // The collider is a 40px-thick rect, so its center sits one half-thickness
    // below the surface: -279 - 20 = -299 (exactly the headless floor).
    // `GROUND_DROP=<px>` lowers the floor below the seed foot line (1 m = 100 px
    // in this world). With the rigid pin the frame cannot follow and the
    // character floats; with the delta pin gravity must make it fall. Used as
    // the gravity/free-fall gate.
    let ground_drop = std::env::var("GROUND_DROP")
        .ok()
        .and_then(|s| s.trim().parse::<f32>().ok())
        .unwrap_or(0.0);
    let ground_surface_y = -279.0 - ground_drop;
    const GROUND_HALF_THICKNESS: f32 = 20.0;
    make_static_collision_shape(
        commands,
        Vec4::new(
            0.0,
            ground_surface_y - GROUND_HALF_THICKNESS,
            0.0,
            1.0,
        ),
        make_long_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(0.0, 0.0),
        0.0,
        false,
    );
    info!(
        "tuning ground: surface y={ground_surface_y:.1} (drop={ground_drop:.1}), rect center y={:.1}",
        ground_surface_y - GROUND_HALF_THICKNESS
    );

    make_static_collision_shape(
        commands,
        Vec4::new(-960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
        false,
    );

    make_static_collision_shape(
        commands,
        Vec4::new(960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
        false,
    );
}

fn make_collision_shape(
    commands: &mut Commands,
    transform: Vec4,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    color: peniko::Brush,
    velocity: Vec2,
    inverse_mass: f32,
    is_soft_body: bool,
    soft_body_init_config: Option<SoftBodyInitConfig>,
    collision_config: Option<CollisionConstraintConfig>,
    collision_group: u32,
) -> Entity {
    let mut scene: VelloScene = VelloScene::default();
    let (s, rect) = f();
    let shape = path_to_ccw_quad_path(&s);
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgba(0.0, 1.0, 0.0, 0.7),
        None,
        &shape,
    );
    let uvs = match &color {
        peniko::Brush::Image(_) | peniko::Brush::PBRImage(_) => Some(generate_uvs(&shape, &rect)),
        _ => None,
    };
    let soft_body_init_transform = Transform {
        translation: Vec3::new(transform.x, transform.y, 0.0),
        rotation: Quat::from_rotation_z(transform.z.to_radians()),
        scale: Vec3::new(transform.w, transform.w, 1.0),
    };
    let entity = commands
        .spawn((
            VelloSceneBundle {
                scene,
                transform: Transform {
                    translation: Vec3::new(transform.x, transform.y, 0.0),
                    ..Default::default()
                },
                ..Default::default()
            },
            VelloCollider::new(
                &shape,
                None,
                &rect,
                velocity,
                color,
                inverse_mass,
                is_soft_body,
                uvs,
                soft_body_init_config,
                collision_config,
                collision_group,
                soft_body_init_transform,
                VELLO_COLLISION_COOL_DOWN_TIME,
            ),
        ))
        .id();
    entity
}

fn make_static_collision_shape(
    commands: &mut Commands,
    transform: Vec4,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    color: peniko::GlowColor,
    velocity: Vec2,
    inverse_mass: f32,
    _is_soft_body: bool,
) {
    let _entity = make_collision_shape(
        commands,
        transform,
        f,
        peniko::Brush::SolidGlow(color),
        velocity,
        inverse_mass,
        false,
        None,
        None,
        0,
    );
}

fn check_assets_loaded(
    mut sc_asset: EventReader<AssetEvent<SvgColliderAsset>>,
    mut im_asset: EventReader<AssetEvent<VelloImageAsset>>,
    mut cs_asset: EventReader<AssetEvent<SvgCharacterAsset>>,
    mut cb_asset: EventReader<AssetEvent<BlueprintCharacterAsset>>,
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut images: ResMut<VelloImageAssetManager>,
    mut next_state: ResMut<NextState<GameState>>,
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
) {
    for sc in sc_asset.read() {
        match sc {
            AssetEvent::LoadedWithDependencies { id } => {
                colliders.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    for im in im_asset.read() {
        match im {
            AssetEvent::LoadedWithDependencies { id } => {
                images.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    for cs in cs_asset.read() {
        match cs {
            AssetEvent::LoadedWithDependencies { id } => {
                character_svg.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    for cb in cb_asset.read() {
        match cb {
            AssetEvent::LoadedWithDependencies { id } => {
                character_blueprint.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    if colliders.all_loaded()
        && images.all_loaded()
        && character_svg.all_loaded()
        && character_blueprint.all_loaded()
    {
        next_state.set(GameState::Game);
    }
}

fn setup_resources(
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut images: ResMut<VelloImageAssetManager>,
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
    asset_server: Res<AssetServer>,
) {
    let mut collider = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        colliders.push(
            asset_server.load(path),
            VelloColliderAssetMetaData::default(),
            name,
        );
    };
    let mut image = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        images.push(
            asset_server.load(path),
            VelloImageAssetMetaData::default(),
            name,
        );
    };
    let mut c_svg = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        character_svg.push(
            asset_server.load(path),
            SvgCharacterAssetMetaData::default(),
            name,
        );
    };
    let mut c_blueprint = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        character_blueprint.push(
            asset_server.load(path),
            BlueprintCharacterAssetMetaData::default(),
            name,
        );
    };
    collider("colliders/ammo.collider.svg");
    collider("colliders/pistol.collider.svg");
    image("image/ammo_albedo.png");
    image("image/ammo_normal.png");
    image("image/pistol_albedo.png");
    image("image/pistol_normal.png");
    c_svg("character/v10.character.svg");
    c_blueprint("character/v10.character.json");
}

/// egui window that edits the live `LimbConfig` on the player's right arm
/// directly (no resource mirror needed — the controller component holds the
/// config and `update_character_movement` reads it each FixedUpdate).
///
/// Exposes the ω-scheduled compliance knobs (`omega_ref`, `soft_scale`) plus
/// the base stiffness pair (`angular_compliance` at `compliance_ref_length`)
/// so arm waggle vs. aim crispness can be tuned live.
fn arm_tuning_ui(
    mut contexts: EguiContexts,
    player_q: Query<Entity, With<Player>>,
    mut arm_q: Query<&mut game_lib::RightArmController>,
) {
    let Ok(player) = player_q.get_single() else {
        return;
    };
    let Ok(mut arm) = arm_q.get_mut(player) else {
        return;
    };
    let config = &mut arm.config;
    egui::Window::new("Arm IK Tuning").show(contexts.ctx_mut(), |ui| {
        ui.heading("Base stiffness (length-normalized)");
        ui.add(
            egui::Slider::new(&mut config.angular_compliance, 1e-9..=1e-4)
                .logarithmic(true)
                .text("angular_compliance @ ref"),
        );
        ui.add(
            egui::Slider::new(&mut config.compliance_ref_length, 5.0..=100.0)
                .text("compliance_ref_length (px)"),
        );
        ui.separator();

        ui.heading("ω-scheduled softening (back-EMF style)");
        ui.label("Softer compliance while the joint's bone spins fast; stiff at rest.");
        ui.add(egui::Slider::new(&mut config.omega_ref, 0.0..=40.0).text("omega_ref (rad/s)"));
        ui.add(egui::Slider::new(&mut config.soft_scale, 1.0..=50.0).text("soft_scale (×)"));
        ui.separator();

        ui.heading("Solver ζ damping (implicit spring-damper)");
        ui.label(
            "Damping ratio baked into the XPBD angular solver. 1.0 = critical (snap, no wobble); <1 springy; >1 heavy.",
        );
        ui.add(egui::Slider::new(&mut config.zeta, 0.0..=2.0).text("zeta (ζ)"));
        ui.separator();

        ui.heading("Rest-angle chase");
        ui.label("Kp: fraction of the gap closed per tick, pre-clamp (shared with spine).");
        ui.add(
            egui::Slider::new(&mut config.rest_chase_alpha, 0.0..=1.0)
                .text("rest_chase_alpha (Kp)"),
        );
        ui.add(
            egui::Slider::new(&mut config.max_angle_rate, 0.0..=40.0)
                .text("max_angle_rate (rad/s)"),
        );
    });
}

/// Push the default toggle state into the world at startup. `VelloCollisionResponsePlugin`
/// boots at zero gravity and the checkbox only writes on a *change*, so without
/// this the "on"-by-default toggle would silently never open gravity.
fn apply_default_gravity(
    toggle: Res<GravityToggle>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    constraint_world.set_gravity(if toggle.enabled {
        Vec2::new(0.0, toggle.magnitude)
    } else {
        Vec2::ZERO
    });
}

/// Gravity toggle: an open/close checkbox plus a magnitude slider (so the app
/// can be driven at the same `g` the `headless_arm_pd` body gate uses). Writes
/// the world gravity only when something actually changes.
fn gravity_toggle_ui(
    mut contexts: EguiContexts,
    mut toggle: ResMut<GravityToggle>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
) {
    egui::Window::new("Gravity").show(contexts.ctx_mut(), |ui| {
        let mut enabled = toggle.enabled;
        let mut magnitude = toggle.magnitude;
        let mut changed = ui.checkbox(&mut enabled, "gravity on / off").changed();
        changed |= ui
            .add(egui::Slider::new(&mut magnitude, 0.0..=4000.0).text("g (px/s², +y down)"))
            .changed();
        if changed {
            toggle.enabled = enabled;
            toggle.magnitude = magnitude;
            constraint_world.set_gravity(if enabled {
                Vec2::new(0.0, magnitude)
            } else {
                Vec2::ZERO
            });
        }
    });
}

/// Hip-pin tuning window: master enable plus the live Raibert resolver +
/// pin-stiffness knobs, and an input readout that shows the W/A/S/D vector the
/// pin is actually reading from `ActionState<PlayerAction>` (proving the hip
/// controller is hooked to input).
fn hip_pin_ui(
    mut contexts: EguiContexts,
    mut params: ResMut<HipPinParams>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    action_q: Query<&ActionState<PlayerAction>>,
) {
    // Snapshot the single character's pin (all Copy) so no borrow is held across
    // the UI closure, which needs `&mut` access to edit the pin below.
    let snapshot = pin_q
        .iter_mut()
        .next()
        .map(|(e, p)| (e, p.enabled, p.ready, p.state.hip_pos));

    egui::Window::new("Hip Pin").show(contexts.ctx_mut(), |ui| {
        let Some((entity, mut enabled, ready, hip_pos)) = snapshot else {
            ui.label("no character / hip pin yet");
            return;
        };

        // Input readout: the exact `input_dir` the pin feeds the resolver.
        // `drive_hip_pin` reads `MoveLeft/Right` (x) and `MoveUp/Down` (y); if
        // this vector reacts to your keys, input is reaching the pin.
        let (mut ix, mut iy) = (0.0_f32, 0.0_f32);
        if let Ok(action) = action_q.get(entity) {
            if action.pressed(&PlayerAction::MoveRight) {
                ix += 1.0;
            }
            if action.pressed(&PlayerAction::MoveLeft) {
                ix -= 1.0;
            }
            if action.pressed(&PlayerAction::MoveUp) {
                iy += 1.0;
            }
            if action.pressed(&PlayerAction::MoveDown) {
                iy -= 1.0;
            }
        }
        ui.colored_label(
            if ix != 0.0 || iy != 0.0 {
                egui::Color32::LIGHT_GREEN
            } else {
                egui::Color32::GRAY
            },
            format!(
                "input WASD: ({ix:+.0}, {iy:+.0})   ready: {ready}   hip: ({:.1}, {:.1})",
                hip_pos.x, hip_pos.y
            ),
        );
        ui.label("Drive with W/A/S/D (leafwing PlayerAction). Arrow keys are unused.");
        ui.separator();

        if ui
            .checkbox(&mut enabled, "hip pin enabled (drives P1/P2/P3)")
            .changed()
        {
            for (_, mut pin) in pin_q.iter_mut() {
                pin.enabled = enabled;
            }
        }
        if let Some((_, mut pin)) = pin_q.iter_mut().next() {
            ui.add(egui::Slider::new(&mut pin.theta, -1.5..=1.5).text("theta (attitude)"));
        }
        if let Some((_, mut pin)) = pin_q.iter_mut().next() {
            let mut drives = params.hip_drives_legs;
            if ui
                .checkbox(&mut drives, "hip pin drives legs (feet follow sandbox)")
                .changed()
            {
                params.hip_drives_legs = drives;
                pin.reseat_leg_bridge(); // re-capture the foot seed from the live pose
            }
        }
        ui.separator();
        ui.label("Raibert lower-body resolver → rigid P1/P2/P3 frame.");
        ui.add(
            egui::Slider::new(&mut params.config.target_speed, 0.0..=600.0).text("target_speed"),
        );
        ui.add(egui::Slider::new(&mut params.config.hip_height, 40.0..=200.0).text("hip_height"));
        ui.add(egui::Slider::new(&mut params.config.thigh_len, 20.0..=120.0).text("thigh_len"));
        ui.add(egui::Slider::new(&mut params.config.calf_len, 20.0..=120.0).text("calf_len"));
        ui.add(egui::Slider::new(&mut params.config.gravity, 0.0..=2000.0).text("gravity"));
        ui.separator();
        ui.label("PD command  a = kp·(pose−x) + kd·(v_hip−v)");
        ui.add(egui::Slider::new(&mut params.kp, 0.0..=4000.0).text("pin kp (P term)"));
        ui.add(egui::Slider::new(&mut params.kd, 0.0..=200.0).text("pin kd (v match)"));
        ui.separator();
        ui.label("Speed-softened compliance (softer when moving fast)");
        ui.add(egui::Slider::new(&mut params.v_ref, 1.0..=2000.0).text("v_ref (px/s)"));
        ui.add(egui::Slider::new(&mut params.soft_scale, 1.0..=50.0).text("soft_scale"));
        ui.separator();
        ui.add(
            egui::Slider::new(&mut params.compliance, 1e-9..=1e-5)
                .logarithmic(true)
                .text("pin compliance"),
        );
        ui.add(egui::Slider::new(&mut params.damping, 0.0..=1.0).text("pin damping"));
    });
}

pub fn add_light(mut commands: Commands) {
    let mut light_scene: VelloScene = VelloScene::default();
    let light_radius = 800.0;
    //let light_shape_ratio = 1.0 / 40.0;
    info!("Add Light");
    light_scene.push_point_light(
        kurbo::Affine::scale(light_radius * 2.0),
        &[1.0, 1.0, 1.0],
        200.0 / (light_radius as f32),
    );
    commands.spawn((VelloSceneBundle {
        scene: light_scene,
        ..Default::default()
    },));
}

fn on_collision_spawn_particle(trigger: Trigger<VelloCollisionTrigger>, mut commands: Commands) {
    let event = trigger.event();
    let pos = event.collision_point;
    let mut scene = VelloScene::default();
    scene.push_instance_with_transforms(&[]);
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgba(1.0, 0.0, 0.0, 0.5),
        None,
        &kurbo::Circle::new((0.0, 0.0), 20.0),
    );
    scene.pop_instance();

    commands.spawn((
        VelloSceneBundle {
            scene,
            transform: Transform::from_translation(pos.extend(100.0)),
            ..Default::default()
        },
        ExplosionEffect::new(
            particles::GravityParticleConfig {
                gravity: Vec2::new(0.0, -98.0),
                drag: 0.0,
                persistent: false,
            },
            particles::BurstEmitterConfig {
                count: 100,
                speed_range: (10.0, 100.0),
                lifetime_range: (1.0, 1.2),
                origin: Vec2::new(0.0, 0.0),
            },
            500,
        ),
    ));
}
