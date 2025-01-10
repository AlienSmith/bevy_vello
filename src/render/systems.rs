use super::extract::{ExtractedRenderAsset, ExtractedRenderText, SSRenderTarget};
use super::plugin::simulate_graph::VelloSimulateGraph;
use super::prepare::PreparedAffine;
use crate::render::extract::ExtractedRenderScene;
use crate::{CoordinateSpace, VelloCanvasMaterial, VelloFont};
use bevy::ecs::system::lifetimeless::Read;
use bevy::prelude::*;
use bevy::render::mesh::Indices;
use bevy::render::render_asset::{RenderAssetUsages, RenderAssets};
use bevy::render::render_graph::{NodeRunError, RenderGraphContext, SlotInfo};
use bevy::render::render_resource::{
    Extent3d, PrimitiveTopology, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue};
use bevy::render::texture::GpuImage;
use bevy::render::view::NoFrustumCulling;
use bevy::sprite::{MaterialMesh2dBundle, Mesh2dHandle};
use bevy::window::{WindowResized, WindowResolution};
use vello::kurbo::Affine;
use vello::{RenderParams, RendererOptions, Scene};

use std::sync::{Arc, Mutex};
#[derive(Component)]
pub struct VelloRenderBatches {
    should_render: bool,
    scene: vello::Scene,
    image: Option<Handle<Image>>,
}

pub fn setup_image(images: &mut Assets<Image>, window: &WindowResolution) -> Handle<Image> {
    let size = Extent3d {
        width: window.physical_width(),
        height: window.physical_height(),
        ..default()
    };

    // This is the texture that will be rendered to.
    let mut image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size,
            dimension: TextureDimension::D2,
            format: TextureFormat::Rgba8Unorm,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING
                | TextureUsages::COPY_DST
                | TextureUsages::STORAGE_BINDING
                | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };

    // fill image.data with zeroes
    image.resize(size);
    images.add(image)
}

/// Transforms all the vectors extracted from the game world and places them in
/// a scene, and renders the scene to a texture with WGPU
#[allow(clippy::complexity)]
pub fn prepare_scene(
    mut commands: Commands,
    ss_render_target: Query<&SSRenderTarget>,
    query_render_vectors: Query<(&PreparedAffine, &ExtractedRenderAsset)>,
    query_render_scenes: Query<(&PreparedAffine, &ExtractedRenderScene)>,
    query_render_texts: Query<(&PreparedAffine, &ExtractedRenderText)>,
    mut font_render_assets: ResMut<RenderAssets<VelloFont>>,
    #[cfg(feature = "lottie")] mut velato_renderer: ResMut<super::VelatoRenderer>,
    render_batches_query: Query<Entity, With<VelloRenderBatches>>,
) {
    for item in render_batches_query.iter() {
        if let Some(entity_commands) = commands.get_entity(item) {
            entity_commands.despawn_recursive();
        }
    }
    let mut batch = VelloRenderBatches {
        should_render: false,
        scene: vello::Scene::default(),
        image: None,
    };
    if let Ok(SSRenderTarget(render_target_image)) = ss_render_target.get_single() {
        //let gpu_image = gpu_images.get(render_target_image).unwrap();

        enum RenderItem<'a> {
            Asset(&'a ExtractedRenderAsset),
            Scene(&'a ExtractedRenderScene),
            Text(&'a ExtractedRenderText),
        }
        let mut render_queue: Vec<(f32, CoordinateSpace, (Affine, RenderItem))> =
            query_render_vectors
                .iter()
                .map(|(&affine, asset)| {
                    (
                        asset.transform.translation().z,
                        asset.render_mode,
                        (*affine, RenderItem::Asset(asset)),
                    )
                })
                .collect();
        render_queue.extend(query_render_scenes.iter().map(|(&affine, scene)| {
            (
                scene.transform.translation().z,
                scene.render_mode,
                (*affine, RenderItem::Scene(scene)),
            )
        }));
        render_queue.extend(query_render_texts.iter().map(|(&affine, text)| {
            (
                text.transform.translation().z,
                text.render_mode,
                (*affine, RenderItem::Text(text)),
            )
        }));

        // Sort by render mode with screen space on top, then by z-index
        render_queue.sort_by(
            |(a_z_index, a_coord_space, _), (b_z_index, b_coord_space, _)| {
                let z_index = a_z_index
                    .partial_cmp(b_z_index)
                    .unwrap_or(std::cmp::Ordering::Equal);
                let render_mode = a_coord_space.cmp(b_coord_space);
                render_mode.then(z_index)
            },
        );

        // Apply transforms to the respective fragments and add them to the
        // scene to be rendered
        let mut scene_buffer = Scene::new();
        for (_, _, (affine, render_item)) in render_queue.iter_mut() {
            match render_item {
                RenderItem::Asset(ExtractedRenderAsset {
                    asset,
                    #[cfg(feature = "lottie")]
                    alpha,
                    #[cfg(feature = "lottie")]
                    theme,
                    #[cfg(feature = "lottie")]
                    playhead,
                    ..
                }) => match &asset.file {
                    #[cfg(feature = "svg")]
                    crate::VectorFile::Svg(scene) => {
                        // TODO: Apply alpha
                        scene_buffer.append(scene, Some(*affine));
                    }
                    #[cfg(feature = "lottie")]
                    crate::VectorFile::Lottie(composition) => {
                        velato_renderer.render(
                            {
                                theme
                                    .as_ref()
                                    .map(|cs| cs.recolor(composition))
                                    .as_ref()
                                    .unwrap_or(composition)
                            },
                            *playhead as f64,
                            *affine,
                            *alpha as f64,
                            &mut scene_buffer,
                        );
                    }
                    #[cfg(not(any(feature = "svg", feature = "lottie")))]
                    _ => unimplemented!(),
                },
                RenderItem::Scene(ExtractedRenderScene { scene, .. }) => {
                    scene_buffer.append(scene, Some(*affine));
                }
                RenderItem::Text(ExtractedRenderText {
                    font,
                    text,
                    alignment,
                    ..
                }) => {
                    if let Some(font) = font_render_assets.get_mut(font) {
                        font.render(&mut scene_buffer, *affine, text, *alignment);
                    }
                }
            }
        }

        // TODO: Vello should be ignoring 0-sized buffers in the future, so this could go away.
        // Prevent a panic in the vello renderer if all the items contain empty encoding data
        let empty_encodings = render_queue
            .iter()
            .filter(|(_, _, (_, item))| match item {
                RenderItem::Asset(a) => match &a.asset.file {
                    #[cfg(feature = "svg")]
                    crate::VectorFile::Svg(scene) => scene.encoding().is_empty(),
                    #[cfg(feature = "lottie")]
                    crate::VectorFile::Lottie(composition) => composition.layers.is_empty(),
                    #[cfg(not(any(feature = "svg", feature = "lottie")))]
                    _ => unimplemented!(),
                },
                RenderItem::Scene(s) => s.scene.encoding().is_empty(),
                RenderItem::Text(t) => t.text.content.is_empty(),
            })
            .count()
            == render_queue.len();
        let should_render = !render_queue.is_empty() && !empty_encodings;
        batch = VelloRenderBatches {
            should_render,
            scene: scene_buffer,
            image: Some(render_target_image.clone()),
        };
    }
    commands.spawn(batch);
}

pub fn resize_rendertargets(
    mut window_resize_events: EventReader<WindowResized>,
    mut query: Query<(&mut SSRenderTarget, &Handle<VelloCanvasMaterial>)>,
    mut images: ResMut<Assets<Image>>,
    mut target_materials: ResMut<Assets<VelloCanvasMaterial>>,
    windows: Query<&Window>,
) {
    let Ok(window) = windows.get_single() else {
        return;
    };
    if window_resize_events.read().last().is_some() {
        let size = Extent3d {
            width: window.resolution.physical_width(),
            height: window.resolution.physical_height(),
            ..default()
        };
        if size.width == 0 || size.height == 0 {
            return;
        }
        for (mut target, target_mat_handle) in query.iter_mut() {
            let image = setup_image(&mut images, &window.resolution);
            if let Some(mat) = target_materials.get_mut(target_mat_handle) {
                target.0 = image.clone();
                mat.texture = image;
            }
            debug!(
                size = format!(
                    "Resized Vello render image to {:?}",
                    (size.width, size.height)
                )
            );
        }
    }
}

pub fn setup_ss_rendertarget(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut custom_materials: ResMut<Assets<VelloCanvasMaterial>>,
    windows: Query<&Window>,
    mut render_target_mesh_handle: Local<Option<Handle<Mesh>>>,
) {
    let Ok(window) = windows.get_single() else {
        return;
    };

    let mesh_handle = render_target_mesh_handle.get_or_insert_with(|| {
        let mut rendertarget_quad = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );

        // Rectangle of the screen
        let verts = vec![
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 1.0, 0.0],
        ];
        rendertarget_quad.insert_attribute(Mesh::ATTRIBUTE_POSITION, verts);

        let uv_pos = vec![[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [1.0, 1.0]];
        rendertarget_quad.insert_attribute(Mesh::ATTRIBUTE_UV_0, uv_pos);

        let indices = vec![0, 1, 2, 0, 2, 3];
        rendertarget_quad.insert_indices(Indices::U32(indices));

        meshes.add(rendertarget_quad)
    });
    let texture_image = setup_image(&mut images, &window.resolution);
    let render_target = SSRenderTarget(texture_image.clone());
    let mesh = Mesh2dHandle(mesh_handle.clone());
    let material = custom_materials.add(VelloCanvasMaterial {
        texture: texture_image,
    });

    commands
        .spawn(MaterialMesh2dBundle {
            mesh,
            material,
            transform: Transform::from_translation(0.001 * Vec3::NEG_Z), /* Make sure the vello
                                                                          * canvas renders behind
                                                                          * Gizmos */
            ..Default::default()
        })
        .insert(NoFrustumCulling)
        .insert(render_target);
}

/// Hide the render target canvas if there is nothing to render
pub fn clear_when_empty(
    mut query_render_target: Query<&mut Visibility, With<SSRenderTarget>>,
    render_items: Query<(&mut CoordinateSpace, &ViewVisibility)>,
) {
    if let Ok(mut visibility) = query_render_target.get_single_mut() {
        if render_items.is_empty() {
            *visibility = Visibility::Hidden;
        } else {
            *visibility = Visibility::Inherited;
        }
    }
}

pub(crate) struct VelloRenderDriverNode;

impl bevy::render::render_graph::Node for VelloRenderDriverNode {
    fn run(
        &self,
        graph: &mut RenderGraphContext,
        _render_context: &mut RenderContext,
        _world: &World,
    ) -> Result<(), NodeRunError> {
        graph.run_sub_graph(VelloSimulateGraph, vec![], None)?;
        Ok(())
    }
}

pub(crate) struct VelloRenderNode {
    renderer: Arc<Mutex<vello::Renderer>>,
    render_query: QueryState<(Entity, Read<VelloRenderBatches>)>,
}

impl VelloRenderNode {
    const _UTILITY_WORKGROUP_SIZE: u32 = 256;

    /// Output particle buffer for that view. TODO - how to handle multiple
    /// buffers?! Should use Entity instead??
    // pub const OUT_PARTICLE_BUFFER: &'static str = "particle_buffer";

    /// Create a new node for simulating the effects of the given world.
    pub fn new(world: &mut World) -> Self {
        let device = world.resource::<RenderDevice>();
        let queue = world.resource::<RenderQueue>();
        Self {
            renderer: Arc::new(Mutex::new(
                vello::Renderer::new(
                    device.wgpu_device(),
                    &(RendererOptions {
                        surface_format: None,
                        timestamp_period: queue.0.get_timestamp_period(),
                        use_cpu: false,
                    }),
                )
                .unwrap(),
            )),
            render_query: QueryState::new(world),
        }
    }
}

impl bevy::render::render_graph::Node for VelloRenderNode {
    fn input(&self) -> Vec<SlotInfo> {
        vec![]
    }

    fn update(&mut self, world: &mut World) {
        self.render_query.update_archetypes(&world);
    }

    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        _render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let gpu_images = world.resource::<RenderAssets<GpuImage>>();
        let device = world.resource::<RenderDevice>();
        let queue = world.resource::<RenderQueue>();
        #[cfg(feature = "particles")]
        let effect_cache = world.resource::<bevy_hanabi::EffectCache>();

        for (_entity, batches) in self.render_query.iter_manual(world) {
            if let Some(image) = &batches.image {
                let gpu_image = gpu_images.get(image).unwrap();
                if batches.should_render {
                    #[cfg(feature = "particles")]
                    self.renderer
                        .lock()
                        .unwrap()
                        .render_to_texture_with_external_particle_buffer(
                            device.wgpu_device(),
                            &queue,
                            &batches.scene,
                            &gpu_image.texture_view,
                            &(RenderParams {
                                base_color: vello::peniko::Color::TRANSPARENT,
                                width: gpu_image.size.x as u32,
                                height: gpu_image.size.y as u32,
                            }),
                            &*effect_cache.obtain_export_buffer(),
                        )
                        .unwrap();
                    #[cfg(not(feature = "particles"))]
                    self.renderer
                        .lock()
                        .unwrap()
                        .render_to_texture(
                            device.wgpu_device(),
                            &queue,
                            &batches.scene,
                            &gpu_image.texture_view,
                            &(RenderParams {
                                base_color: vello::peniko::Color::TRANSPARENT,
                                width: gpu_image.size.x as u32,
                                height: gpu_image.size.y as u32,
                            }),
                        )
                        .unwrap();
                }
            }
        }

        Ok(())
    }
}
