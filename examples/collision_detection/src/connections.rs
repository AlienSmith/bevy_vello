use bevy::prelude::*;
use bevy_vello::integrations::physics::Index;

#[derive(Resource, Default)]
pub struct ConnectionStatus {
    pub(crate) last_connection: Option<Index>,
    pub(crate) all_connectiion: Vec<Index>,
}

impl ConnectionStatus {
    pub fn push(&mut self, index: Index) {
        if index != Index::DANGLING {
            self.last_connection = Some(index);
            self.all_connectiion.push(index);
        }
    }
    pub fn reset(&mut self) -> Vec<Index> {
        let mut results = vec![];
        results.append(&mut self.all_connectiion);
        self.last_connection = None;
        return results;
    }
}
