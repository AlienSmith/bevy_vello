use std::sync::{Arc, Mutex};

use super::extract::{self, ExtractedPixelScale, SSRenderTarget};
use super::systems::{VelloRenderDriverNode, VelloRenderNode};
use super::{prepare, systems};
use crate::integrations::VelloAssetSource;
use crate::render::extract::ExtractedRenderText;
use crate::render::SSRT_SHADER_HANDLE;
use crate::{VelloAsset, VelloScene};
use crate::{VelloCanvasMaterial, VelloFont};
use bevy::render::render_graph::RenderGraph;
use bevy::{
    asset::load_internal_asset,
    prelude::*,
    render::{
        extract_component::ExtractComponentPlugin,
        render_asset::RenderAssetPlugin,
        renderer::{RenderDevice, RenderQueue},
        view::{check_visibility, VisibilitySystems},
        Render, RenderApp, RenderSet,
    },
    sprite::Material2dPlugin,
};
use vello::RendererOptions;
pub struct VelloRenderPlugin;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, SystemSet)]
pub enum VelloPrepareSystems {
    PrepareAssects,
    PrepareScene,
}

pub mod main_graph {
    pub mod node {
        use bevy::render::render_graph::RenderLabel;

        /// Label for the simulation driver node running the simulation graph.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, RenderLabel)]
        pub struct VelloDriverNode;
    }
}

pub mod simulate_graph {
    use bevy::render::render_graph::RenderSubGraph;

    /// Name of the simulation sub-graph.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, RenderSubGraph)]
    pub struct VelloSimulateGraph;

    pub mod node {
        use bevy::render::render_graph::RenderLabel;

        /// Label for the simulation node (init and update compute passes;
        /// view-independent).
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, RenderLabel)]
        pub struct VelloSimulateNode;
    }
}

impl Plugin for VelloRenderPlugin {
    fn build(&self, app: &mut App) {
        load_internal_asset!(
            app,
            SSRT_SHADER_HANDLE,
            "../../shaders/vello_ss_rendertarget.wgsl",
            Shader::from_wgsl
        );

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        #[cfg(feature = "svg")]
        render_app.add_systems(ExtractSchedule, extract::extract_svg_instances);
        #[cfg(feature = "lottie")]
        render_app
            .init_resource::<super::VelatoRenderer>()
            .add_systems(ExtractSchedule, extract::extract_lottie_instances);

        render_app
            .insert_resource(ExtractedPixelScale(1.0))
            .add_systems(
                ExtractSchedule,
                (
                    extract::extract_pixel_scale.in_set(RenderSet::ExtractCommands),
                    extract::scene_instances,
                ),
            )
            .configure_sets(
                Render,
                (
                    VelloPrepareSystems::PrepareAssects
                        .before(VelloPrepareSystems::PrepareScene)
                        .after(RenderSet::Prepare),
                    VelloPrepareSystems::PrepareScene.before(RenderSet::Render),
                ),
            )
            .add_systems(
                Render,
                (
                    prepare::prepare_vector_affines,
                    prepare::prepare_scene_affines,
                    prepare::prepare_text_affines,
                )
                    .in_set(VelloPrepareSystems::PrepareAssects),
            );
        render_app.add_systems(
            Render,
            systems::prepare_scene
                .in_set(VelloPrepareSystems::PrepareScene)
                .run_if(resource_exists::<RenderDevice>),
        );

        app.add_plugins((
            Material2dPlugin::<VelloCanvasMaterial>::default(),
            ExtractComponentPlugin::<ExtractedRenderText>::default(),
            ExtractComponentPlugin::<SSRenderTarget>::default(),
            RenderAssetPlugin::<VelloFont>::default(),
        ))
        .add_systems(Startup, systems::setup_ss_rendertarget)
        .add_systems(
            Update,
            (systems::resize_rendertargets, systems::clear_when_empty),
        );
        // .add_systems(
        //     PostUpdate,
        //     check_visibility.in_set(VisibilitySystems::CheckVisibility),
        // );
    }

    fn finish(&self, app: &mut App) {
        // Step 1: Create a SINGLE vello::Renderer shared between rendering and collision.
        // Using Arc<Mutex<>> because Bevy's render world and main world run on separate threads.
        // The Mutex serializes access: collision in FixedUpdate, rendering in render graph.
        let shared_renderer = {
            let render_app = app.sub_app_mut(RenderApp);
            let device = render_app.world().resource::<RenderDevice>();
            let queue = render_app.world().resource::<RenderQueue>();
            Arc::new(Mutex::new(
                vello::Renderer::new(
                    device.wgpu_device(),
                    &(RendererOptions {
                        surface_format: None,
                        timestamp_period: queue.0.get_timestamp_period(),
                        use_cpu: false,
                    }),
                )
                .unwrap(),
            ))
        };

        // Step 2: Create GpuCollisionRunner with a cloned Arc (same renderer, same GPU buffers)
        let (device, queue) = {
            let render_app = app.sub_app_mut(RenderApp);
            let device = render_app.world().resource::<RenderDevice>();
            let queue = render_app.world().resource::<RenderQueue>();
            (
                device.wgpu_device().clone(),
                queue.0.as_ref().clone().into_inner(),
            )
        };
        let collision_runner = crate::collision::GpuCollisionRunner::new(
            shared_renderer.clone(),
            device.clone(),
            queue.clone(),
        );
        app.world_mut().insert_resource(collision_runner);

        // Step 2b: Create GpuRayTraceRunner with the same cloned renderer
        let raytrace_runner =
            crate::collision::GpuRayTraceRunner::new(shared_renderer.clone(), device, queue);
        app.world_mut().insert_resource(raytrace_runner);

        // Step 3: Set up the render graph with the SAME shared renderer
        let render_app = app.sub_app_mut(RenderApp);
        let mut simulate_graph = RenderGraph::default();
        let simulate_node =
            VelloRenderNode::new_with_renderer(&mut render_app.world_mut(), shared_renderer);
        simulate_graph.add_node(simulate_graph::node::VelloSimulateNode, simulate_node);
        let mut graph = render_app
            .world_mut()
            .get_resource_mut::<RenderGraph>()
            .unwrap();
        graph.add_sub_graph(simulate_graph::VelloSimulateGraph, simulate_graph);

        // Add the simulation driver node which executes the simulation sub-graph. It
        // runs before the camera driver, since rendering needs to access simulated
        // particles.
        graph.add_node(main_graph::node::VelloDriverNode, VelloRenderDriverNode {});

        graph.add_node_edge(
            main_graph::node::VelloDriverNode,
            bevy::render::graph::CameraDriverLabel,
        );
    }
}
