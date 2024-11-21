use bevy::{asset::AssetMetaCheck, prelude::*};
use bevy_vello::{
    dock::{lottie_loader::LottieLoaderPlugin, svg_loader::SvgLoaderPlugin},
    VelloPlugin,
};

mod wasm {
    extern crate wasm_bindgen;
    use bevy_vello::dock::commands::{DockCommand, DockCommandResult};
    use wasm_bindgen::prelude::*;
    use wasm_bindgen_futures::future_to_promise;
    #[wasm_bindgen(start)]
    pub fn runner() {
        crate::run();
    }
    fn result_to_js_value(result: DockCommandResult) -> JsValue {
        match result {
            DockCommandResult::Ok(i) => JsValue::from(i),
            DockCommandResult::NotOk(s) => JsValue::from(s),
        }
    }
    #[wasm_bindgen]
    pub fn load_svg_assets_from_bytes(data: Vec<u8>) -> js_sys::Promise {
        let receiver =
            bevy_vello::dock::stream_factory::dock_push_commands(DockCommand::LoadSVGAssets(data));
        future_to_promise(async move {
            match receiver.await {
                Ok(value) => Ok(result_to_js_value(value)), // Resolve the `Promise` with the `u32` value
                Err(_) => Err(JsValue::from("Connection failed")), // Resolve the `Promise` with `undefined` if the sender is closed
            }
        })
    }
    #[wasm_bindgen]
    pub fn load_lottie_assets_from_bytes(data: Vec<u8>) -> js_sys::Promise {
        let receiver = bevy_vello::dock::stream_factory::dock_push_commands(
            DockCommand::LoadLottieAssets(data),
        );
        future_to_promise(async move {
            match receiver.await {
                Ok(value) => Ok(result_to_js_value(value)), // Resolve the `Promise` with the `u32` value
                Err(_) => Err(JsValue::from("Connection failed")), // Resolve the `Promise` with `undefined` if the sender is closed
            }
        })
    }
}

pub fn run() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        ..default()
    }))
    .add_plugins(VelloPlugin)
    .add_plugins(SvgLoaderPlugin)
    .add_plugins(LottieLoaderPlugin)
    .add_systems(Startup, recieve_svg);
    app.run();
}

fn recieve_svg(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}
