use super::extract::{self, ExtractedPixelScale, SSRenderTarget};
use super::systems::{VelloRenderDriverNode, VelloRenderNode};
use super::{prepare, systems};
use crate::render::extract::ExtractedRenderText;
use crate::render::SSRT_SHADER_HANDLE;
use crate::{VelloCanvasMaterial, VelloFont};
use bevy::asset::load_internal_asset;
use bevy::prelude::*;
use bevy::render::extract_component::ExtractComponentPlugin;
use bevy::render::render_asset::RenderAssetPlugin;
use bevy::render::render_graph::RenderGraph;
#[cfg(not(feature = "particles"))]
use bevy::render::renderer::RenderDevice;
use bevy::render::{Render, RenderApp, RenderSet};
use bevy::sprite::Material2dPlugin;
use bevy_hanabi::HanabiDriverNode;
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

        let Ok(render_app) = app.get_sub_app_mut(RenderApp) else {
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
        #[cfg(feature = "particles")]
        render_app.add_systems(
            Render,
            systems::prepare_scene
                .in_set(VelloPrepareSystems::PrepareScene)
                .run_if(resource_exists::<bevy_hanabi::render::EffectCache>),
        );
        #[cfg(not(feature = "particles"))]
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
    }

    fn finish(&self, app: &mut App) {
        // Add the simulation sub-graph. This render graph runs once per frame no matter
        // how many cameras/views are active (view-independent).
        let render_app = app.sub_app_mut(RenderApp);
        let mut simulate_graph = RenderGraph::default();
        let simulate_node = VelloRenderNode::new(&mut render_app.world);
        simulate_graph.add_node(simulate_graph::node::VelloSimulateNode, simulate_node);
        let mut graph = render_app.world.get_resource_mut::<RenderGraph>().unwrap();
        graph.add_sub_graph(simulate_graph::VelloSimulateGraph, simulate_graph);

        // Add the simulation driver node which executes the simulation sub-graph. It
        // runs before the camera driver, since rendering needs to access simulated
        // particles.
        graph.add_node(main_graph::node::VelloDriverNode, VelloRenderDriverNode {});
        graph.add_node_edge(HanabiDriverNode, main_graph::node::VelloDriverNode);
    }
}
