use bevy::{ecs::intern::Interned, platform::collections::HashMap, prelude::*};
use bevy_vello::{
    collision::path_to_ccw_quad_path,
    integrations::physics::{VelloCharacterPhysicsRoot, VelloJoint, VelloParticle},
    VelloCollider, VelloScene, VelloSceneBundle,
};
use vello::{
    kurbo::{self, Affine, BezPath, Shape},
    peniko::{self, GlowColor},
};
use vello_physics::{
    collision_response::Particle, generate_uvs, soft_body_connection::ConnectionInitConfig,
    utility::bilinear_interpolate, CollisionConstraintConfig, ConnectionConstraintInitConfig,
    FramePositionConstraintConfig, SoftBodyInitConfig, FRAME_PARTICLES_COUNT,
};

use crate::{
    character::{
        ArmConfig, Connectivity, ConnectivityRoot, LeftArmController, RightArmController,
        SpineConfig, SpineController, StringPool,
    },
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, SvgCharacterAsset,
        SvgCharacterAssetManager,
    },
    character_factory::CharacterRoot,
};
pub fn assemble_character(
    trigger: Trigger<OnAdd, CharacterRoot>,
    mut commands: Commands,
    query: Query<(&CharacterRoot, &Transform)>,
    svg_manager: Res<SvgCharacterAssetManager>,
    blueprint_manager: Res<BlueprintCharacterAssetManager>,
    svg_assets: Res<Assets<SvgCharacterAsset>>,
    blueprint_assets: Res<Assets<BlueprintCharacterAsset>>,
    string_pool: ResMut<StringPool>,
) {
    let root_entity = trigger.target();
    // 1. Get the specific asset IDs for this character
    let (config, transform) = query.get(root_entity).unwrap();

    let affine = transform_to_affine(transform);
    let apply_transform_to_particle = |p: &mut Particle| {
        let point = kurbo::Point::new(p.pos.x as f64, p.pos.y as f64);
        let result = affine * point;
        p.pos = Vec2::new(result.x as f32, result.y as f32);
        return;
    };

    let Some(blueprint_handle) = blueprint_manager.get_index_from_name(&config.blueprint_asset_id)
    else {
        warn!(
            "could not found character blueprint asset {:?}",
            config.blueprint_asset_id
        );
        return;
    };
    let Some(svg_handle) = svg_manager.get_index_from_name(&config.svg_asset_id) else {
        warn!(
            "could not found character svg asset {:?}",
            config.svg_asset_id
        );
        return;
    };
    let blueprint = blueprint_assets.get(blueprint_handle.id()).unwrap();
    let svgs = svg_assets.get(svg_handle.id()).unwrap();
    let mut character_connectivity = ConnectivityRoot::default();
    let mut colliders_particle_entity: HashMap<String, (Entity, Connectivity)> = HashMap::new();

    for item in blueprint.data.colliders.iter() {
        let Some((s, rect)) = svgs.data.get(&item.path_id) else {
            warn!(
                "could not found character svg path {:?}, {:?}",
                config.svg_asset_id, item.path_id
            );
            return;
        };
        let inv_mass = item.softbody.total_inv_mass;

        //TODO: pass these info
        let color = bevy_vello::prelude::peniko::Brush::SolidGlow(GlowColor {
            color: peniko::Color::PINK,
            glow: 1.0,
        });
        let collision_group = 1;

        let entity = make_collision_shape(
            &mut commands,
            transform,
            s,
            rect,
            color,
            inv_mass,
            true,
            Some(item.softbody),
            Some(item.collision),
            collision_group,
        );
        let collider_name = string_pool.pool.intern(&item.path_id);
        colliders_particle_entity.insert(
            item.path_id.to_string(),
            (entity, Connectivity::new(root_entity, true, collider_name)),
        );
        character_connectivity.parts.insert(collider_name, entity);
    }

    for item in blueprint.data.particles.iter() {
        let mut particle = item.particle.clone();
        apply_transform_to_particle(&mut particle);
        let mut config = item.frame_conn.clone();
        let entity = make_particle(&mut commands, particle, &root_entity, config);
        let particle_name = string_pool.pool.intern(&item.path_id);
        colliders_particle_entity.insert(
            item.path_id.to_string(),
            (entity, Connectivity::new(root_entity, true, particle_name)),
        );
        character_connectivity.parts.insert(particle_name, entity);
    }
    let mut sibling: Vec<(String, Entity)> = vec![];
    let process_name = |colliders_particle_entity: &HashMap<String, (Entity, Connectivity)>,
                        sibling: &mut Vec<(String, Entity)>,
                        name: &String,
                        path_name: &String|
     -> Entity {
        let Some((entity_a, _)) = colliders_particle_entity.get(name) else {
            panic!(
                "could not found collider {:?}, constraint {:?}, in {:?}",
                name, path_name, config.svg_asset_id
            );
        };
        sibling.push((name.clone(), *entity_a));
        *entity_a
    };

    for item in blueprint.data.joints.iter() {
        sibling.clear();
        let joint_config = match &item.config {
            ConnectionInitConfig::Bilinear(s0, s1, compliance) => {
                let e0 = process_name(&colliders_particle_entity, &mut sibling, s0, &item.path_id);
                let e1 = process_name(&colliders_particle_entity, &mut sibling, s1, &item.path_id);
                ConnectionConstraintInitConfig::<Entity>::Bilinear(e0, e1, *compliance)
            }
            ConnectionInitConfig::Distance(s0, s1, compliance) => {
                let e0 = process_name(&colliders_particle_entity, &mut sibling, s0, &item.path_id);
                let e1 = process_name(&colliders_particle_entity, &mut sibling, s1, &item.path_id);
                ConnectionConstraintInitConfig::<Entity>::Distance(e0, e1, *compliance)
            }
            ConnectionInitConfig::Angular(s0, s1, s2, compliance) => {
                let e0 = process_name(&colliders_particle_entity, &mut sibling, s0, &item.path_id);
                let e1 = process_name(&colliders_particle_entity, &mut sibling, s1, &item.path_id);
                let e2 = process_name(&colliders_particle_entity, &mut sibling, s2, &item.path_id);
                ConnectionConstraintInitConfig::<Entity>::Angular(e0, e1, e2, *compliance)
            }
        };
        let joint_name = string_pool.pool.intern(&item.path_id);
        let joint_entity = make_joint(
            &mut commands,
            joint_config,
            &sibling,
            root_entity,
            joint_name,
        );

        for (s, _) in sibling.iter() {
            colliders_particle_entity
                .get_mut(s)
                .unwrap()
                .1
                .parts
                .insert(joint_entity);
        }

        character_connectivity
            .parts
            .insert(joint_name, joint_entity);
    }

    // Look up controller entity handles BEFORE character_connectivity is moved.
    let intern = |name: &str| string_pool.pool.intern(name);
    let get = |name: &str| -> Entity {
        *character_connectivity
            .parts
            .get(&intern(name))
            .unwrap_or_else(|| panic!("missing entity for {name}"))
    };

    // Maybe make this into json file too.

    // Spine particles: [PH, P0, P1, P2, P3]
    let spine = SpineController {
        particles: [get("PH"), get("P0"), get("P1"), get("P2"), get("P3")],
        config: SpineConfig::default(),
        move_vector: Vec2::ZERO,
    };

    // Right arm particles: [P1, P12, P13, PRLA]
    // Right arm joints:    [P1_P12_P13, P12_P13_PRLA]
    let right_arm = RightArmController {
        particles: [get("P1"), get("P12"), get("P13"), get("PRLA")],
        joints: [get("P1_P12_P13"), get("P12_P13_PRLA")],
        config: ArmConfig {
            bend_sign: 1.0,
            ..Default::default()
        },
        target: Vec2::ZERO,
    };

    // Left arm particles: [P1, P11, P10, PLLA]
    // Left arm joints:    [P1_P11_P10, P11_P10_PLLA]
    let left_arm = LeftArmController {
        particles: [get("P1"), get("P11"), get("P10"), get("PLLA")],
        joints: [get("P1_P11_P10"), get("P11_P10_PLLA")],
        config: ArmConfig {
            bend_sign: 1.0,
            ..Default::default()
        },
        target: Vec2::ZERO,
    };

    commands.entity(root_entity).insert(character_connectivity);
    let frame_config = blueprint.data.frame.init_config.clone();
    //to do read these from the FrameConfig.
    let left_hip = colliders_particle_entity
        .get(&frame_config.left_hip)
        .unwrap()
        .0
        .clone();
    let right_hip = colliders_particle_entity
        .get(&frame_config.right_hip)
        .unwrap()
        .0
        .clone();
    let base_spine = colliders_particle_entity
        .get(&frame_config.spine_base)
        .unwrap()
        .0
        .clone();
    let mid_spine = colliders_particle_entity
        .get(&frame_config.spine_mid)
        .unwrap()
        .0
        .clone();
    let frame_entites: [Entity; FRAME_PARTICLES_COUNT] =
        [left_hip, right_hip, base_spine, mid_spine];

    commands.entity(root_entity).insert((
        VelloCharacterPhysicsRoot::new(frame_config, frame_entites),
        spine,
        right_arm,
        left_arm,
    ));

    for (_, (e, c)) in colliders_particle_entity.drain() {
        commands.entity(e).insert(c);
    }

    // The CharacterRoot remains on the entity as your permanent marker.
}

fn make_particle(
    commands: &mut Commands,
    particle: Particle,
    root_entity: &Entity,
    frame_connect_config: FramePositionConstraintConfig,
) -> Entity {
    commands
        .spawn(VelloParticle::new(
            particle,
            *root_entity,
            frame_connect_config,
        ))
        .id()
}

fn make_joint(
    commands: &mut Commands,
    connection_config: ConnectionConstraintInitConfig<Entity>,
    entity: &[(String, Entity)],
    character: Entity,
    name: Interned<str>,
) -> Entity {
    let mut connectivity = Connectivity::new(character, false, name);
    for item in entity {
        connectivity.parts.insert(item.1.clone());
    }
    commands
        .spawn((VelloJoint::new(connection_config, character), connectivity))
        .id()
}

fn make_collision_shape(
    commands: &mut Commands,
    transform: &Transform,
    s: &BezPath,
    rect: &kurbo::Rect,
    color: peniko::Brush,
    inverse_mass: f32,
    is_soft_body: bool,
    soft_body_init_config: Option<SoftBodyInitConfig>,
    collision_config: Option<CollisionConstraintConfig>,
    collision_group: u32,
) -> Entity {
    let mut scene: VelloScene = VelloScene::default();
    let shape = path_to_ccw_quad_path(&s);
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgba(0.0, 1.0, 0.0, 0.7),
        None,
        &shape,
    );
    let uvs = match &color {
        peniko::Brush::Image(_) | peniko::Brush::PBRImage(_) => Some(generate_uvs(&shape, rect)),
        _ => None,
    };
    let frame_path = rect.to_path(0.1);
    let soft_body_init_transform = *transform;
    let translation = transform.translation;
    commands
        .spawn((
            VelloSceneBundle {
                scene,
                transform: Transform {
                    translation: Vec3::new(translation.x, translation.y, 0.0),
                    ..Default::default()
                },
                ..Default::default()
            },
            VelloCollider::new(
                &shape,
                &frame_path,
                &rect,
                Vec2::new(0.0, 0.0),
                color,
                inverse_mass,
                is_soft_body,
                uvs,
                soft_body_init_config,
                collision_config,
                collision_group,
                soft_body_init_transform,
            ),
        ))
        .id()
}

pub fn transform_to_affine(transform: &Transform) -> Affine {
    let mut model_matrix = transform.compute_matrix();
    model_matrix.w_axis.y *= -1.0;

    let transform: [f32; 16] = model_matrix.to_cols_array();

    // | a c e |
    // | b d f |
    // | 0 0 1 |
    let transform: [f64; 6] = [
        transform[0] as f64,  // a
        -transform[1] as f64, // b
        -transform[4] as f64, // c
        transform[5] as f64,  // d
        transform[12] as f64, // e
        transform[13] as f64, // f
    ];
    Affine::new(transform)
}
