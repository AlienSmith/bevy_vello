use crate::dock::stream_factory::*;
use bevy::prelude::*;
use bevy_hanabi::EffectAsset;
use crossbeam_channel::Receiver;

use super::{commands::*, DockSystems};
pub struct ParticleLoaderPlugin;
#[derive(Resource)]
pub struct ParticleReciever {
    r: Receiver<u32>,
}

impl Plugin for ParticleLoaderPlugin {
    fn build(&self, app: &mut App) {
        let r = dock_register_loader(DockCommandDispatcherType::LoadParticleAssets);
        app.insert_resource(ParticleReciever { r })
            .add_systems(Update, load_particle.in_set(DockSystems::Load));
    }
}
fn load_particle(mut assets: ResMut<Assets<EffectAsset>>, r: Res<ParticleReciever>) {
    if let Ok(id) = r.r.try_recv() {
        bevy::log::info!("get_dock {}", id.clone());
        let data = dock_get_command(id);
        if let DockCommand::LoadParticleAssets(asset) = &data.data {
            let custom_asset = ron::de::from_bytes::<EffectAsset>(asset).unwrap();
            let handle = assets.add(custom_asset);
            bevy::log::info!("id {} with {:?}", id.clone(), handle);
            let index = dock_push_particle_asset(handle);
            let _ = data.s.send(DockCommandResult::Ok(index));
        } else {
            let _ = data
                .s
                .send(DockCommandResult::NotOk("Load particle failed".to_owned()));
        }
    }
}
