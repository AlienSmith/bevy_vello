use bevy::prelude::*;
use bevy_vello::{
    collision::{path_to_ccw_quad_path, VELLO_COLLISION_COOL_DOWN_TIME},
    integrations::svg_collider::{
        SvgColliderAsset, SvgColliderAssetManager, VelloImageAsset, VelloImageAssetManager,
    },
    VelloCollider, VelloScene,
};
use vello::{
    kurbo::{self, Shape},
    peniko, Quad,
};
use vello_physics::generate_uvs;

use crate::collider_factory::ColliderRoot;

pub fn assemble_collider(
    trigger: Trigger<OnAdd, ColliderRoot>,
    mut commands: Commands,
    svgs: Res<SvgColliderAssetManager>,
    images: Res<VelloImageAssetManager>,
    image_assets: Res<Assets<VelloImageAsset>>,
    svg_assets: Res<Assets<SvgColliderAsset>>,
    mut query: Query<(&ColliderRoot, &mut VelloScene)>,
) {
    let root_entity = trigger.target();
    let (config, mut vllo_scene) = query.get_mut(root_entity).unwrap();
    //make brush
    let albedo = image_assets
        .get(&images.get_index_from_name(&config.albedo_asset_id).unwrap())
        .unwrap()
        .image
        .clone()
        .with_usage(peniko::ImageUsageType::MASKED);
    let normals = image_assets
        .get(&images.get_index_from_name(&config.normal_asset_id).unwrap())
        .unwrap()
        .image
        .clone()
        .with_usage(peniko::ImageUsageType::NORMAL);
    let brush = bevy_vello::prelude::peniko::Brush::PBRImage(peniko::PBRImages::new(
        albedo,
        normals,
        config.metallic,
        config.roughness,
    ));
    //make shape
    let svg_collider = svg_assets
        .get(&svgs.get_index_from_name(&config.svg_asset_id).unwrap())
        .unwrap();
    let p = svg_collider.shape.clone();
    let rect = svg_collider.aabb.clone();
    let shape = path_to_ccw_quad_path(&p);
    let uvs = Some(generate_uvs(&shape, &rect));
    // The pistol/weapon asset only carries an axis-aligned `Rect` (no engine
    // quad), but its frame must NOT fall back to an OBB. Derive a quad from
    // the rect so the engine builds an AABB-style (non-OBB) coarse frame.
    let frame = Some(Quad::from_rect(rect));
    //make rendering sence
    vllo_scene.reset();
    //make collider
    commands.entity(root_entity).insert((VelloCollider::new(
        &shape,
        frame,
        &rect,
        config.initial_velocity,
        brush,
        config.collision_inverse_mass,
        true,
        uvs,
        Some(config.softbody_config),
        Some(config.collision_config),
        1,
        config.soft_body_init_transform,
        VELLO_COLLISION_COOL_DOWN_TIME,
    ),));
}
