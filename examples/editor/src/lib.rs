use bevy::{asset::AssetMetaCheck, color::palettes::css::WHITE, prelude::*};
use bevy_vello::{dock::DockPlugin, VelloPlugin};

mod wasm {
    extern crate wasm_bindgen;
    use bevy::{
        math::{Quat, Vec2, Vec3},
        prelude::Transform,
    };
    use bevy_vello::dock::commands::{DockCommand, DockCommandResult};
    use bevy_vello::dock::stream_factory::*;
    use futures::channel::oneshot::Receiver;
    use wasm_bindgen::prelude::*;
    use wasm_bindgen_futures::future_to_promise;
    #[wasm_bindgen]
    #[derive(Clone, Copy, Debug)]
    pub struct Transform2D {
        pub t0: f32,
        pub t1: f32,
        pub r: f32,
        pub s0: f32,
        pub s1: f32,
        pub z: f32,
    }
    #[wasm_bindgen]
    impl Transform2D {
        #[wasm_bindgen(constructor)]
        pub fn new(
            transform_x: f32,
            transform_y: f32,
            rotation: f32,
            scale_x: f32,
            scale_y: f32,
            depth: f32,
        ) -> Self {
            Transform2D {
                t0: transform_x,
                t1: transform_y,
                r: rotation,
                s0: scale_x,
                s1: scale_y,
                z: depth,
            }
        }
    }
    impl From<Transform2D> for Transform {
        fn from(v: Transform2D) -> Self {
            Transform {
                translation: Vec3 {
                    x: v.t0,
                    y: v.t1,
                    z: v.z,
                },
                rotation: Quat::from_rotation_z(v.r),
                scale: Vec3 {
                    x: v.s0,
                    y: v.s1,
                    z: 1.0,
                },
            }
        }
    }

    #[wasm_bindgen]
    pub fn start() {
        crate::run();
    }
    fn result_to_js_value(result: DockCommandResult) -> JsValue {
        match result {
            DockCommandResult::Ok(i) => JsValue::from(i),
            DockCommandResult::NotOk(s) => JsValue::from(s),
        }
    }
    fn pack_reciever(receiver: Receiver<DockCommandResult>) -> js_sys::Promise {
        future_to_promise(async move {
            match receiver.await {
                Ok(value) => Ok(result_to_js_value(value)), // Resolve the `Promise` with the `u32` value
                Err(_) => Err(JsValue::from("Connection failed")), // Resolve the `Promise` with `undefined` if the sender is closed
            }
        })
    }
    #[wasm_bindgen]
    pub fn load_svg_assets_from_bytes(data: Vec<u8>) -> js_sys::Promise {
        pack_reciever(dock_push_commands(DockCommand::LoadSVGAssets(data)))
    }
    #[wasm_bindgen]
    pub fn load_lottie_assets_from_bytes(data: Vec<u8>) -> js_sys::Promise {
        pack_reciever(dock_push_commands(DockCommand::LoadLottieAssets(data)))
    }
    #[wasm_bindgen]
    pub fn spawn_entity(asset_id: u32, transform: Transform2D) -> js_sys::Promise {
        pack_reciever(dock_push_commands(DockCommand::SpawnEntity(
            asset_id,
            transform.into(),
        )))
    }
    #[wasm_bindgen]
    pub fn remove_entity(entity_id: u32) -> js_sys::Promise {
        pack_reciever(dock_push_commands(DockCommand::RemoveEntity(entity_id)))
    }
    #[wasm_bindgen]
    pub fn modify_entity(entity_id: u32, transform: Transform2D) -> js_sys::Promise {
        pack_reciever(dock_push_commands(DockCommand::Transform(
            entity_id,
            transform.into(),
        )))
    }
    #[wasm_bindgen]
    pub fn modify_camera(x: f32, y: f32, scale: f32) -> js_sys::Promise {
        pack_reciever(dock_push_commands(DockCommand::ModifyCamera(
            Vec2::new(x, y),
            scale,
        )))
    }
}

pub fn run() {
    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .set(AssetPlugin {
                meta_check: AssetMetaCheck::Never,
                ..default()
            })
            .set(WindowPlugin {
                primary_window: Some(Window {
                    canvas: Some("#mygame-canvas".into()),
                    ..default()
                }),
                ..default()
            }),
    )
    .add_plugins(VelloPlugin)
    .add_plugins(DockPlugin);
    app.run();
}
