use crate::integrations::VelloSceneSubBundle;
use crate::VelloAsset;
use crate::VelloAssetBundle;
use crate::VelloScene;
use avian2d::prelude::*;
use bevy::app::App;
use bevy::app::Plugin;
use bevy::prelude::*;
use bevy_hanabi::ParticleEffect;
use bevy_hanabi::ParticleEffectBundle;
use vello::scene::StorkeExpand;

use super::commands::*;
use super::stream_factory::*;
use super::DockSystems;
use crossbeam_channel::Receiver;
pub struct EntitySpawnerPlugin;
#[derive(Resource)]
pub struct EntitySpawnerReciever {
    r: Receiver<u32>,
}

impl Plugin for EntitySpawnerPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::SpawnEntity);
        app.insert_resource(EntitySpawnerReciever { r })
            .add_systems(Update, spawn_vello_bundle.in_set(DockSystems::Spawn));
    }
}

fn spawn_vello_bundle(
    mut commands: Commands,
    r: Res<EntitySpawnerReciever>,
    vello_assets: Res<Assets<VelloAsset>>,
) {
    if let Ok(id) = r.r.try_recv() {
        let data = dock_get_command(id);
        if let DockCommand::SpawnEntity(asset_id, transform, entity_type, second_asset_id) =
            &data.data
        {
            match entity_type {
                EntityType::Vello => {
                    let asset = dock_get_asset_with_id(*asset_id);
                    let entity = commands
                        .spawn((
                            VelloAssetBundle {
                                vector: asset,
                                transform: *transform,
                                ..default()
                            },
                            Sensor,
                            RigidBody::Static,
                            Collider::rectangle(100.0, 100.0),
                        ))
                        .id();
                    let entity_id = dock_push_entitie(entity);
                    let _ = data.s.send(DockCommandResult::Ok(entity_id));
                    bevy::log::info!(
                        "spawn entity with vello asset {}, id {}, entity{:?}",
                        id.clone(),
                        entity_id,
                        entity
                    );
                }
                EntityType::Particle => {
                    let effect = dock_get_particle_asset_with_id(*asset_id);
                    bevy::log::info!(
                        "spawn entity with id {} particle asset {:?}",
                        id.clone(),
                        effect
                    );
                    let (scene, local_transform_center) = if *second_asset_id != 0 {
                        let vello_asset = dock_get_asset_with_id(*second_asset_id);
                        if let Some(
                            _asset @ VelloAsset {
                                file: _file @ crate::VectorFile::Svg(scene_ptr),
                                local_transform_center,
                                ..
                            },
                        ) = vello_assets.get(&vello_asset)
                        {
                            ((**scene_ptr).clone().into(), *local_transform_center)
                        } else {
                            (make_default_rect_particles(), Transform::default())
                        }
                    } else {
                        (make_default_rect_particles(), Transform::default())
                    };
                    //let totoal_transform = transform.mul_transform(local_transform_center);
                    let entity = spawn_particles(commands, effect, scene, transform);
                    let entity_id = dock_push_entitie(entity);
                    let _ = data.s.send(DockCommandResult::Ok(entity_id));
                    bevy::log::info!(
                        "spawn entity with particle asset {}, id {}, entity{:?}",
                        id.clone(),
                        entity_id,
                        entity
                    );
                }
            }
        } else {
            let _ = data
                .s
                .send(DockCommandResult::NotOk("spawn entity failed".to_owned()));
        }
    }
}

fn make_default_rect_particles() -> VelloScene {
    let mut scene = VelloScene::default();
    use vello::kurbo::*;
    use vello::peniko::*;
    let color = Color::rgb(0.8 as f64, 0.0, 0.0);
    let color1 = Color::rgb(0.0, 1.0, 1.0);
    scene.stroke(
        &Stroke::new(12.0).with_solid_ratio(0.0),
        Affine::default(),
        color,
        None,
        &Circle::new(Point { x: -5.0, y: 0.0 }, 10.0),
    );
    scene.stroke(
        &Stroke::new(2.0),
        Affine::default(),
        color1,
        None,
        &Circle::new(Point { x: -5.0, y: 0.0 }, 10.0),
    );
    let mut path = BezPath::new();
    path.push(PathEl::MoveTo(Point { x: -5.0, y: 0.0 }));
    path.push(PathEl::LineTo(Point { x: 5.0, y: 0.0 }));
    scene.stroke(
        &Stroke::new(12.0).with_solid_ratio(0.0),
        Affine::default(),
        color,
        None,
        &path,
    );
    scene.stroke(&Stroke::new(2.0), Affine::default(), color1, None, &path);
    scene
}

fn spawn_particles(
    mut commands: Commands,
    effect: Handle<bevy_hanabi::EffectAsset>,
    scene: VelloScene,
    transform: &Transform,
) -> Entity {
    // Create a color gradient for the particles
    // Spawn an instance of the particle effect, and override its Z layer to
    // be above the reference white square previously spawned.
    commands
        .spawn((
            ParticleEffectBundle {
                // Assign the Z layer so it appears in the egui inspector and can be modified at runtime
                effect: ParticleEffect::new(effect).with_z_layer_2d(Some(0.1)),
                transform: *transform,
                ..default()
            },
            VelloSceneSubBundle {
                scene,
                ..Default::default()
            },
            Sensor,
            RigidBody::Static,
            Collider::rectangle(100.0, 100.0),
        ))
        .id()
}
