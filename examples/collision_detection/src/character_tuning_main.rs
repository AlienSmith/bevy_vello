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
        SoftBodyInitConfig, VelloCollisionTrigger, VelloCollisionWorld,
        VELLO_COLLISION_COOL_DOWN_TIME,
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
    default_input_map, drive_hip_pin, measure_body, reset_hip_pin, tick_loop3_angular,
    update_character_movement, update_ground_support,
    weapons::{AttachPistolToCharacterEvent, PistolControl},
    CharacterController, CharacterRoot, ColliderRoot, ConnectivityRoot, GameLabSystems,
    GroundSupport, HipPin, HipPinParams, PlayerAction, PlayerMarker, SpineController,
    SpineDriveEnabled, VelloCharacterPlugin, VerticalPhase,
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
/// The pistol is a body part with mass/contacts attached to the right arm. It
/// is off by default so the tuning app matches the pistol-free headless walk;
/// `PISTOL=1` re-enables it.
#[derive(Resource)]
struct PistolEnabled(bool);

fn pistol_enabled(enabled: Res<PistolEnabled>) -> bool {
    enabled.0
}

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
        .insert_resource(PistolEnabled(matches!(
            std::env::var("PISTOL").ok().as_deref().map(str::trim),
            Some("1") | Some("true") | Some("yes")
        )))
        .insert_resource(cannon::CannonParams::default())
        .insert_resource(GravityToggle::default())
        // Same env-driven controller as the headless harness: `SUPPORT_SHARE`,
        // `HIP_DROP`, `TARGET_SPEED`, `LEGS`.
        // Default = angular-only loop 3; the hip pin owns the resolver/leg
        // targets, so the spine drive must stay silent or it will emit
        // conflicting external P1/P2/P3 position constraints and snap the spine.
        .insert_resource(HipPinParams::from_env())
        .insert_resource(SpineDriveEnabled(false))
        // Ground gate: vertical authority and gravity cancellation only while a
        // character collider is in floor contact.
        .insert_resource(GroundSupport::default())
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
        .add_systems(FixedUpdate, update_ground_support.before(drive_hip_pin))
        // Measurement layer: whole-body CoM / attitude, EMA-filtered, gathered
        // before the resolver runs so the loops read `pin.measured` this tick.
        .add_systems(
            FixedUpdate,
            measure_body
                .after(update_ground_support)
                .before(drive_hip_pin),
        )
        .add_systems(
            FixedUpdate,
            drive_hip_pin
                .before(update_character_movement)
                .before(update_constraint_world),
        )
        // Angular-only loop 3: drive P1_P2_P3 when the external pin is off.
        .add_systems(
            FixedUpdate,
            tick_loop3_angular
                .after(drive_hip_pin)
                .before(update_constraint_world),
        )
        // Match the headless harness exactly: 90 Hz fixed physics and the same
        // `SUBSTEPS` env (default 1). Without this the two apps can silently
        // run different XPBD convergence settings.
        .add_systems(Startup, configure_physics)
        .add_systems(Startup, setup_back_ground)
        .add_systems(Startup, add_light)
        .add_systems(Startup, setup_resources)
        // The world boots at zero gravity; open it now to match the default.
        .add_systems(Startup, apply_default_gravity)
        .add_systems(Startup, log_pin_setup)
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
                    .before(GameLabSystems::WriteCharacterPartEvent)
                    .run_if(pistol_enabled),
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

/// Align the real-time tuning app with the headless harness:
/// `Time<Fixed> = 90 Hz`, `VelloCollisionWorld::substeps = SUBSTEPS` (default 1).
fn configure_physics(
    mut collision_world: ResMut<VelloCollisionWorld>,
    mut fixed: ResMut<Time<Fixed>>,
) {
    let substeps = std::env::var("SUBSTEPS")
        .ok()
        .and_then(|s| s.trim().parse::<u32>().ok())
        .unwrap_or(2)
        .max(1);
    collision_world.substeps = substeps;
    *fixed = Time::<Fixed>::from_hz(90.0);
    info!("tuning physics: fixed 90 Hz, substeps={substeps}");
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
        Vec4::new(0.0, ground_surface_y - GROUND_HALF_THICKNESS, 0.0, 1.0),
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

    // Small ramp + raised platform for climb / step / fall testing:
    //   ramp   x = 120..270, rises 30 px (≈20°)
    //   platform x = 270..470, top 30 px above the ground
    //   right edge at x = 470 is a 30 px drop back to the ground.
    // The 30 px rise keeps the hip inside the leg workspace, so the pose-aware
    // bridge can still reach without terrain-aware foot targets.
    //
    // Off by default for angular-only flat-floor tuning (`RAMP=1` re-enables).
    // There is no terrain-aware foot target yet, so walking into the ramp
    // collapses the body and produces the move-time snap.
    let ramp_enabled = matches!(
        std::env::var("RAMP").ok().as_deref().map(str::trim),
        Some("1") | Some("true") | Some("yes")
    );
    if ramp_enabled {
        const RAMP_RISE: f32 = 30.0;
        const RAMP_X0: f32 = 120.0;
        const RAMP_X1: f32 = 270.0;
        const PLATFORM_X1: f32 = 470.0;
        let ramp = {
            let half_x = ((RAMP_X1 - RAMP_X0) * 0.5) as f64;
            let half_y = (RAMP_RISE * 0.5) as f64;
            let mut path = BezPath::new();
            path.move_to((-half_x, -half_y));
            path.line_to((half_x, half_y));
            path.line_to((half_x, -half_y));
            path.close_path();
            (path, kurbo::Rect::new(-half_x, -half_y, half_x, half_y))
        };
        make_static_collision_shape(
            commands,
            Vec4::new(
                (RAMP_X0 + RAMP_X1) * 0.5,
                ground_surface_y + RAMP_RISE * 0.5,
                0.0,
                1.0,
            ),
            move || ramp.clone(),
            GlowColor {
                color: peniko::Color::rgb(0.1, 0.8, 1.0),
                glow: 1.0,
            },
            Vec2::new(0.0, 0.0),
            0.0,
            false,
        );
        make_static_collision_shape(
            commands,
            Vec4::new(
                (RAMP_X1 + PLATFORM_X1) * 0.5,
                ground_surface_y + RAMP_RISE * 0.5,
                0.0,
                1.0,
            ),
            || {
                let half_w = ((PLATFORM_X1 - RAMP_X1) * 0.5) as f64;
                let half_h = (RAMP_RISE * 0.5) as f64;
                let rect = kurbo::Rect::new(-half_w, -half_h, half_w, half_h);
                (rect.to_path(0.1), rect)
            },
            GlowColor {
                color: peniko::Color::rgb(0.1, 0.8, 1.0),
                glow: 1.0,
            },
            Vec2::new(0.0, 0.0),
            0.0,
            false,
        );
        info!(
        "tuning ramp: x={RAMP_X0:.0}..{RAMP_X1:.0} rise {RAMP_RISE:.0} px, platform x={RAMP_X1:.0}..{PLATFORM_X1:.0}"
    );
    }

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

fn log_pin_setup() {
    info!("tuning pin controller: delta velocity servo");
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
    support: Option<Res<GroundSupport>>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<&SpineController>,
    p_q: Query<&VelloParticle>,
    action_q: Query<&ActionState<PlayerAction>>,
) {
    // Snapshot the single character's pin (all Copy) so no borrow is held across
    // the UI closure, which needs `&mut` access to edit the pin below.
    let snapshot = pin_q.iter_mut().next().map(|(e, p)| {
        let live_y = spine_q.get(e).ok().and_then(|spine| {
            let p1 = p_q.get(spine.particles[2]).ok()?.particle.pos.y;
            let p2 = p_q.get(spine.particles[3]).ok()?.particle.pos.y;
            let p3 = p_q.get(spine.particles[4]).ok()?.particle.pos.y;
            Some((p1, p2, p3))
        });
        (
            e,
            p.enabled,
            p.ready,
            p.state.hip_pos,
            p.last_ref_p3,
            p.last_sag_y,
            p.state.vs.phase,
            p.theta,
            p.last_meas_theta,
            p.last_att_target,
            p.last_att_rest_cos,
            p.last_att_rest_sin,
            p.last_external_pin,
            live_y,
        )
    });

    egui::Window::new("Hip Pin").show(contexts.ctx_mut(), |ui| {
        let Some((
            entity,
            mut enabled,
            ready,
            hip_pos,
            ref_p3,
            sag_y,
            vphase,
            model_theta,
            meas_theta,
            att_target,
            rest_cos,
            rest_sin,
            ext_pin,
            live_y,
        )) = snapshot
        else {
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

        ui.add(
            egui::Slider::new(&mut params.support_share, 0.0..=1.0)
                .text("support_share (gravity cancelled while grounded)"),
        );

        // Live gate readout: the master switch is only one input. The effective
        // vertical authority also depends on ground contact (or a ballistic
        // launch/flight reference), and gravity cancellation is forced to 0 in
        // the air. This line is the ground truth while jumping.
        let grounded = support.as_deref().map(|s| s.grounded).unwrap_or(true);
        let contacts = support.as_deref().map(|s| s.contacts).unwrap_or(0);
        let ballistic = matches!(vphase, VerticalPhase::Launch | VerticalPhase::Flight);
        let vertical_active = grounded || ballistic;
        let gravity_cancel = if grounded { params.support_share } else { 0.0 };
        ui.colored_label(
            if grounded {
                egui::Color32::LIGHT_GREEN
            } else {
                egui::Color32::LIGHT_RED
            },
            format!(
                "gate: grounded={grounded} (contacts {contacts})  phase={vphase:?}\n\
                 vertical command active={vertical_active}  gravity cancelled={:.0}%",
                gravity_cancel * 100.0
            ),
        );
        ui.colored_label(
            egui::Color32::LIGHT_BLUE,
            format!(
                "P3 actual y {:.1}, resolver ref y {:.1}, sag {sag_y:+.1} px",
                ref_p3.y + sag_y,
                ref_p3.y
            ),
        );
        ui.separator();

        if ui
            .checkbox(
                &mut enabled,
                "hip controller enabled (resolver + leg targets)",
            )
            .changed()
        {
            for (_, mut pin) in pin_q.iter_mut() {
                pin.enabled = enabled;
            }
        }
        ui.checkbox(&mut params.external_pin, "external pin (debug / hard pin)");
        ui.checkbox(
            &mut params.air_control,
            "mid-air direction assist (external pin)",
        );
        ui.colored_label(
            if ext_pin {
                egui::Color32::LIGHT_RED
            } else {
                egui::Color32::LIGHT_GREEN
            },
            format!("external pin active this tick: {ext_pin}"),
        );
        if let Some((p1, p2, p3)) = live_y {
            // Vello y-down: floor surface is y = +279 in this scene.
            const FLOOR_Y: f32 = 279.0;
            ui.label(format!(
                "live y: P1={p1:.1} P2={p2:.1} P3={p3:.1}  floor={FLOOR_Y:.1}"
            ));
            ui.colored_label(
                if p3 > FLOOR_Y - 5.0 {
                    egui::Color32::LIGHT_RED
                } else {
                    egui::Color32::LIGHT_GREEN
                },
                format!("hip clearance above floor: {:.1} px", FLOOR_Y - p3),
            );
        } else {
            ui.colored_label(egui::Color32::YELLOW, "live P1/P2/P3 unavailable");
        }

        // Loop 3: delivered through the leg PD's world-upright reference axis.
        // `P1_P2_P3` is only held straight (posture), never used as the actuator.
        ui.checkbox(
            &mut params.attitude_enabled,
            "loop 3: leg-delivered attitude",
        );
        let rest_angle = rest_sin.atan2(rest_cos).to_degrees();
        ui.colored_label(
            egui::Color32::LIGHT_YELLOW,
            format!(
                "model θ {:+.2}°  measured θ {:+.2}°  target {:+.2}°  rest {:+.2}°\n\
                 rest cos/sin {:+.4}/{:+.4}",
                model_theta.to_degrees(),
                meas_theta.to_degrees(),
                att_target.to_degrees(),
                rest_angle,
                rest_cos,
                rest_sin,
            ),
        );
        ui.add(
            egui::Slider::new(&mut params.attitude_lean_kp, 0.0..=2.0)
                .text("loop-3 lean Kp (leg ref)"),
        );
        ui.add(
            egui::Slider::new(&mut params.attitude_lean_kd, 0.0..=0.2)
                .text("loop-3 lean Kd (leg ref)"),
        );
        ui.add(
            egui::Slider::new(&mut params.attitude_target_bias, -0.5..=0.5)
                .text("loop-3 target bias (rad)"),
        );
        ui.add(
            egui::Slider::new(&mut params.attitude_max_angle, 0.0..=1.3)
                .text("loop-3 max lean (rad)"),
        );
        ui.label("P1_P2_P3 posture hold (external pin / waist stiffness)");
        ui.add(
            egui::Slider::new(&mut params.attitude_compliance, 1e-9..=1e-5)
                .logarithmic(true)
                .text("waist hold compliance"),
        );
        ui.add(egui::Slider::new(&mut params.attitude_zeta, 0.0..=2.0).text("waist hold zeta"));
        ui.separator();
        ui.label("Leg angular solver (world-upright reference when pin off)");
        ui.add(
            egui::Slider::new(&mut params.leg_compliance, 1e-12..=1e-6)
                .logarithmic(true)
                .text("leg compliance"),
        );
        ui.add(egui::Slider::new(&mut params.leg_zeta, 0.0..=2.0).text("leg zeta"));
        ui.add(egui::Slider::new(&mut params.leg_alpha, 0.0..=1.0).text("leg alpha"));
        ui.add(
            egui::Slider::new(&mut params.leg_max_angle_rate, 0.5..=20.0)
                .text("leg max angle rate (rad/s)"),
        );
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
        ui.add(
            egui::Slider::new(&mut params.config.jump_apex, 10.0..=200.0)
                .text("jump_apex (px above stand)"),
        );
        ui.add(egui::Slider::new(&mut params.config.hip_height, 40.0..=200.0).text("hip_height"));
        ui.add(egui::Slider::new(&mut params.config.thigh_len, 20.0..=120.0).text("thigh_len"));
        ui.add(egui::Slider::new(&mut params.config.calf_len, 20.0..=120.0).text("calf_len"));
        ui.add(egui::Slider::new(&mut params.config.gravity, 0.0..=2000.0).text("gravity"));
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
