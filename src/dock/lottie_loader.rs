use crate::dock::commands::*;
use crate::dock::stream_factory::*;
use crate::integrations::lottie::load_lottie_from_bytes;
use crate::VelloAsset;
use bevy::prelude::*;
use crossbeam_channel::Receiver;

use super::DockSystems;
pub struct LottieLoaderPlugin;
#[derive(Resource)]
pub struct LottieReciever {
    r: Receiver<u32>,
}

impl Plugin for LottieLoaderPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::LoadLottieAssets);
        app.insert_resource(LottieReciever { r })
            .add_systems(Update, load_lottie.in_set(DockSystems::Load));
    }
}
fn load_lottie(mut assets: ResMut<Assets<VelloAsset>>, r: Res<LottieReciever>) {
    if let Ok(id) = r.r.try_recv() {
        bevy::log::info!("get_dock {}", id.clone());
        let data = dock_get_command(id);
        if let DockCommand::LoadLottieAssets(asset) = &data.data {
            let handle = assets.add(load_lottie_from_bytes(asset).unwrap());
            let index = dock_push_asset(handle);
            let _ = data.s.send(DockCommandResult::Ok(index));
        } else {
            let _ = data
                .s
                .send(DockCommandResult::NotOk("Load svg failed".to_owned()));
        }
    }
}
