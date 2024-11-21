use crate::dock::stream_factory::*;
use crate::integrations::svg::load_svg_from_bytes;
use crate::VelloAsset;
use bevy::prelude::*;
use crossbeam_channel::Receiver;

use super::{commands::*, DockSystems};
pub struct SvgLoaderPlugin;
#[derive(Resource)]
pub struct SvgReciever {
    r: Receiver<u32>,
}

impl Plugin for SvgLoaderPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::LoadSVGAssets);
        app.insert_resource(SvgReciever { r })
            .add_systems(Update, load_svg.in_set(DockSystems::Load));
    }
}
fn load_svg(mut assets: ResMut<Assets<VelloAsset>>, r: Res<SvgReciever>) {
    if let Ok(id) = r.r.try_recv() {
        bevy::log::info!("get_dock {}", id.clone());
        let data = dock_get_command(id);
        if let DockCommand::LoadSVGAssets(asset) = &data.data {
            let handle = assets.add(load_svg_from_bytes(asset).unwrap());
            let index = dock_push_asset(handle);
            let _ = data.s.send(DockCommandResult::Ok(index));
        } else {
            let _ = data
                .s
                .send(DockCommandResult::NotOk("Load svg failed".to_owned()));
        }
    }
}
