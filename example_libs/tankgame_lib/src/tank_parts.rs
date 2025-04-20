use bevy::{prelude::*, utils::HashMap};
use bevy_vello::prelude::*;

use crate::TankGameAssetsMetaData;
#[derive(Clone, Default)]
pub struct Part {
    pub handle: Handle<VelloReplaySceneAsset>,
    pub meta: TankGameAssetsMetaData,
}

#[derive(Resource, Clone, Default)]
pub struct TankGameAssets {
    pub id_to_index: HashMap<AssetId<VelloReplaySceneAsset>, usize>,
    pub parts: Vec<Part>,
    pub load_state: Vec<bool>,
}

impl TankGameAssets {
    pub fn push(&mut self, handle: Handle<VelloReplaySceneAsset>, meta: TankGameAssetsMetaData) {
        let index = self.parts.len();
        let id = handle.id();
        self.id_to_index.insert(id, index);
        self.parts.push(Part { handle, meta });
        self.load_state.push(false);
    }

    pub fn get_index<T: Into<usize>>(&self, index: T) -> Option<Handle<VelloReplaySceneAsset>> {
        let index = index.into();
        if self.parts.len() > index && self.load_state[index] {
            return Some(self.parts[index].clone().handle);
        }
        None
    }

    pub fn get_part_at_index<T: Into<usize>>(&self, index: T) -> Option<Part> {
        let index = index.into();
        if self.parts.len() > index && self.load_state[index] {
            return Some(self.parts[index].clone());
        }
        None
    }

    pub fn mark_as_loaded(&mut self, id: &AssetId<VelloReplaySceneAsset>) {
        if let Some(index) = self.id_to_index.get(id) {
            self.load_state[*index] = true;
        }
    }

    pub fn all_loaded(&self) -> bool {
        self.load_state.iter().all(|&loaded| loaded)
    }
}
