use bevy::{platform::collections::HashMap, prelude::*};
use bevy_vello::{
    collision::path_to_ccw_quad_path, integrations::physics::VelloJoint, VelloCollider, VelloScene,
    VelloSceneBundle,
};
use nalgebra::Vector2;
use vello::{
    kurbo::{self, Affine, BezPath, Shape},
    peniko::{self, GlowColor},
};
use vello_physics::{
    generate_uvs, soft_body_connection::ConnectionInitConfig, CollisionConstraintConfig,
    SoftBodyInitConfig,
};

use crate::{
    character::Connectivity,
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
) {
    let root_entity = trigger.target();
    // 1. Get the specific asset IDs for this character
    let (config, transform) = query.get(root_entity).unwrap();
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
    let mut character_connectivity = Connectivity::new(None, true);
    let mut colliders_entity: HashMap<String, (Entity, Connectivity)> = HashMap::new();
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
        colliders_entity.insert(
            item.path_id.to_string(),
            (entity, Connectivity::new(Some(root_entity), true)),
        );
        character_connectivity.parts.insert(entity);
    }
    let affine = transform_to_affine(transform);
    let apply_transform_to_pos = |p: Vector2<f32>| -> Vector2<f32> {
        let point = kurbo::Point::new(p.x as f64, p.y as f64);
        let result = affine * point;
        return Vector2::new(result.x as f32, result.y as f32);
    };
    for item in blueprint.data.joints.iter() {
        let joint_config = match item.config {
            ConnectionInitConfig::HingeJoint(
                mut particle,
                mut particle1,
                mut particle2,
                complaince,
                min,
                max,
                clamp,
            ) => {
                particle.pos = apply_transform_to_pos(particle.pos);
                particle1.pos = apply_transform_to_pos(particle1.pos);
                particle2.pos = apply_transform_to_pos(particle2.pos);
                ConnectionInitConfig::HingeJoint(
                    particle, particle1, particle2, complaince, min, max, clamp,
                )
            }
            _ => {
                todo!()
            }
        };

        let (entity_a, entity_b) = {
            let Some((entity_a, _)) = colliders_entity.get(&item.collider_a_id) else {
                warn!(
                    "could not found collider {:?}, for joint {:?}, in {:?}",
                    item.collider_a_id, item.path_id, config.svg_asset_id
                );
                return;
            };
            let Some((entity_b, _)) = colliders_entity.get(&item.collider_b_id) else {
                warn!(
                    "could not found collider {:?}, for joint {:?}, in {:?}",
                    item.collider_b_id, item.path_id, config.svg_asset_id,
                );
                return;
            };
            (*entity_a, *entity_b)
        };
        let joint_entity = make_joint(&mut commands, joint_config, entity_a, entity_b, root_entity);
        character_connectivity.parts.insert(joint_entity);
        colliders_entity
            .get_mut(&item.collider_a_id)
            .unwrap()
            .1
            .parts
            .insert(joint_entity);
        colliders_entity
            .get_mut(&item.collider_b_id)
            .unwrap()
            .1
            .parts
            .insert(joint_entity);
    }

    commands.entity(root_entity).insert(character_connectivity);
    for (_, (e, c)) in colliders_entity.drain() {
        commands.entity(e).insert(c);
    }

    // The CharacterRoot remains on the entity as your permanent marker.
}

fn make_joint(
    commands: &mut Commands,
    connection_config: ConnectionInitConfig,
    entity_a: Entity,
    entity_b: Entity,
    character: Entity,
) -> Entity {
    let mut connectivity = Connectivity::new(Some(character), false);
    connectivity.parts.insert(entity_a);
    connectivity.parts.insert(entity_b);
    commands
        .spawn((
            VelloJoint::new(connection_config, entity_a, entity_b),
            connectivity,
        ))
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
