use bevy::{prelude::*, utils::HashMap};
use bevy_vello::prelude::*;
#[derive(Resource, Clone, Default)]
pub struct TankParts {
    pub id_to_index: HashMap<AssetId<VelloReplaySceneAsset>, usize>,
    pub parts: Vec<Handle<VelloReplaySceneAsset>>,
    pub load_state: Vec<bool>,
}

impl TankParts {
    pub fn push(&mut self, handle: Handle<VelloReplaySceneAsset>) {
        let index = self.parts.len();
        let id = handle.id();
        self.id_to_index.insert(id, index);
        self.parts.push(handle);
        self.load_state.push(false);
    }

    pub fn get_index<T: Into<usize>>(&self, index: T) -> Option<Handle<VelloReplaySceneAsset>> {
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
