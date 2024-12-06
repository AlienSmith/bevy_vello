use bevy::{prelude::*, utils::HashMap};
use bevy_hanabi::EffectAsset;
use crossbeam_channel::{unbounded, SendError};
use futures::channel::oneshot;

use crate::{dock::commands::*, VelloAsset};

#[derive(Debug, Clone)]
struct DockDispatcher {
    sender: crossbeam_channel::Sender<u32>,
}

impl DockDispatcher {
    fn send(&self, id: u32) -> Result<(), SendError<u32>> {
        self.sender.send(id)
    }
}
#[derive(Debug)]
pub struct DockData {
    pub data: DockCommand,
    pub s: oneshot::Sender<DockCommandResult>,
}
#[derive(Clone, Debug)]
pub struct IDGen {
    command: u32,
    entities: u32,
    assets: u32,
    particle_assets: u32,
}
impl Default for IDGen {
    fn default() -> Self {
        //start all id with 1 leaving 0 as invalid index
        Self {
            command: 1,
            entities: 1,
            assets: 1,
            particle_assets: 1,
        }
    }
}
impl IDGen {
    fn fetch_add(value: &mut u32) -> u32 {
        let current = *value;
        *value += 1;
        current
    }
    pub fn next_command_id(&mut self) -> u32 {
        IDGen::fetch_add(&mut self.command)
    }
    pub fn next_entity_id(&mut self) -> u32 {
        IDGen::fetch_add(&mut self.entities)
    }
    pub fn next_assets_id(&mut self) -> u32 {
        IDGen::fetch_add(&mut self.assets)
    }
    pub fn next_particle_assets_id(&mut self) -> u32 {
        IDGen::fetch_add(&mut self.particle_assets)
    }
}

#[derive(Debug, Default)]
struct Dock {
    commands: HashMap<u32, DockData>,
    entities: HashMap<u32, Entity>,
    inver_entities: HashMap<Entity, u32>,
    assets: HashMap<u32, Handle<VelloAsset>>,
    particle_assets: HashMap<u32, Handle<EffectAsset>>,
    messenger: HashMap<u32, DockDispatcher>,
    id_generator: IDGen,
}

impl Dock {
    pub(crate) fn register(
        &mut self,
        ext: DockCommandDispatcherType,
    ) -> crossbeam_channel::Receiver<u32> {
        let (s, r) = unbounded();
        self.messenger
            .insert(ext.to_index(), DockDispatcher { sender: s });
        r
    }

    pub(crate) fn push_entitie(&mut self, entity: Entity) -> u32 {
        let id = self.id_generator.next_entity_id();
        self.entities.insert(id, entity);
        self.inver_entities.insert(entity, id);
        id
    }

    pub(crate) fn remove_entitie(&mut self, id: u32) {
        let entity = self.get_entity_with_id(id);
        let _ = self.inver_entities.remove(&entity);
        let _ = self.entities.remove(&id);
    }

    pub(crate) fn get_entity_with_id(&self, id: u32) -> Entity {
        self.entities
            .get(&id)
            .expect("none existing entity")
            .clone()
    }

    pub(crate) fn get_entity_id(&self, entity: Entity) -> u32 {
        self.inver_entities
            .get(&entity)
            .expect("none existing entity id")
            .clone()
    }

    pub(crate) fn push_asset(&mut self, asset: Handle<VelloAsset>) -> u32 {
        let id = self.id_generator.next_assets_id();
        self.assets.insert(id, asset);
        id
    }

    pub(crate) fn remove_asset(&mut self, id: u32) {
        self.assets.remove(&id);
    }

    pub(crate) fn get_asset_with_id(&self, id: u32) -> Handle<VelloAsset> {
        self.assets.get(&id).expect("none existing entity").clone()
    }

    pub(crate) fn push_particle_asset(&mut self, asset: Handle<EffectAsset>) -> u32 {
        let id = self.id_generator.next_particle_assets_id();
        self.particle_assets.insert(id, asset);
        id
    }

    pub(crate) fn remove_particle_asset(&mut self, id: u32) {
        self.particle_assets.remove(&id);
    }

    pub(crate) fn get_particle_asset_with_id(&self, id: u32) -> Handle<EffectAsset> {
        self.particle_assets
            .get(&id)
            .expect("none existing entity")
            .clone()
    }

    pub fn push_commands(&mut self, command: DockCommand) -> oneshot::Receiver<DockCommandResult> {
        let (s, r) = oneshot::channel();
        let id = self.id_generator.next_command_id();
        let dispatcher = command_to_dispatcher(&command);
        self.commands.insert(id, DockData { data: command, s });
        let _ = self
            .messenger
            .get(&dispatcher.to_index())
            .expect("missing dock woker to do add specific type")
            .send(id);
        r
    }

    pub fn get_command(&mut self, index: u32) -> DockData {
        self.commands
            .remove(&index)
            .expect("try to pop none existing data")
    }
}

use once_cell::sync::Lazy;
use std::sync::{Arc, RwLock};
static EXTERNAL_DATA_DOCK: Lazy<Arc<RwLock<Dock>>> =
    Lazy::new(|| Arc::new(RwLock::new(Dock::default())));

pub(crate) fn dock_register_loader(
    ext: DockCommandDispatcherType,
) -> crossbeam_channel::Receiver<u32> {
    EXTERNAL_DATA_DOCK.write().unwrap().register(ext)
}

pub(crate) fn dock_push_entitie(entity: Entity) -> u32 {
    EXTERNAL_DATA_DOCK.write().unwrap().push_entitie(entity)
}

pub(crate) fn dock_remove_entitie(id: u32) {
    EXTERNAL_DATA_DOCK.write().unwrap().remove_entitie(id)
}

pub(crate) fn dock_get_entity_with_id(id: u32) -> Entity {
    EXTERNAL_DATA_DOCK.read().unwrap().get_entity_with_id(id)
}

pub(crate) fn dock_get_entity_id(entity: Entity) -> u32 {
    EXTERNAL_DATA_DOCK.read().unwrap().get_entity_id(entity)
}

pub(crate) fn dock_push_asset(asset: Handle<VelloAsset>) -> u32 {
    EXTERNAL_DATA_DOCK.write().unwrap().push_asset(asset)
}

pub(crate) fn dock_remove_asset(id: u32) {
    EXTERNAL_DATA_DOCK.write().unwrap().remove_asset(id)
}

pub(crate) fn dock_get_asset_with_id(id: u32) -> Handle<VelloAsset> {
    EXTERNAL_DATA_DOCK.read().unwrap().get_asset_with_id(id)
}

pub(crate) fn dock_push_particle_asset(asset: Handle<EffectAsset>) -> u32 {
    EXTERNAL_DATA_DOCK
        .write()
        .unwrap()
        .push_particle_asset(asset)
}

pub(crate) fn dock_remove_particle_asset(id: u32) {
    EXTERNAL_DATA_DOCK
        .write()
        .unwrap()
        .remove_particle_asset(id)
}

pub(crate) fn dock_get_particle_asset_with_id(id: u32) -> Handle<EffectAsset> {
    EXTERNAL_DATA_DOCK
        .read()
        .unwrap()
        .get_particle_asset_with_id(id)
}

pub fn dock_push_commands(command: DockCommand) -> oneshot::Receiver<DockCommandResult> {
    EXTERNAL_DATA_DOCK.write().unwrap().push_commands(command)
}

pub(crate) fn dock_get_command(index: u32) -> DockData {
    EXTERNAL_DATA_DOCK.write().unwrap().get_command(index)
}

#[test]
pub fn test_dock() {
    let svg_name = "something.svg";
    let json_name = "something.json";
    let svg_data = svg_name.as_bytes().to_vec();
    let json_data = json_name.as_bytes().to_vec();
    let svg_r = dock_register_loader(DockCommandDispatcherType::LoadSVGAssets);
    let json_r = dock_register_loader(DockCommandDispatcherType::LoadLottieAssets);

    dock_push_commands(DockCommand::LoadSVGAssets(svg_data));
    dock_push_commands(DockCommand::LoadLottieAssets(json_data));

    if let Ok(id) = svg_r.try_recv() {
        println!("svg provided with {:?}", dock_get_command(id));
    }

    if let Ok(id) = json_r.try_recv() {
        println!("json provided with {:?}", dock_get_command(id));
    }
}
