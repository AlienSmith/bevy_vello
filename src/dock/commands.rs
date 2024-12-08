use bevy::{math::Vec2, prelude::Transform};

#[derive(Clone, Debug)]
pub enum EntityType {
    Vello,
    Particle,
}

#[derive(Clone, Debug)]
pub enum DockCommand {
    LoadSVGAssets(Vec<u8>),
    LoadLottieAssets(Vec<u8>),
    RemoveEntity(u32),
    SpawnEntity(u32, Transform, EntityType, u32),
    Transform(u32, Transform),
    ModifyCamera(Vec2, f32),
    PickEntity(Vec2, f32),
    LoadParticleAssets(Vec<u8>),
}

pub(crate) fn command_to_dispatcher(command: &DockCommand) -> DockCommandDispatcherType {
    match command {
        DockCommand::LoadSVGAssets(_) => DockCommandDispatcherType::LoadSVGAssets,
        DockCommand::LoadLottieAssets(_) => DockCommandDispatcherType::LoadLottieAssets,
        DockCommand::RemoveEntity(_) => DockCommandDispatcherType::RemoveEntity,
        DockCommand::SpawnEntity(_, _, _, _) => DockCommandDispatcherType::SpawnEntity,
        DockCommand::Transform(_, _) => DockCommandDispatcherType::Transform,
        DockCommand::ModifyCamera(_, _) => DockCommandDispatcherType::ModifyCamera,
        DockCommand::PickEntity(_, _) => DockCommandDispatcherType::PickEntity,
        DockCommand::LoadParticleAssets(_) => DockCommandDispatcherType::LoadParticleAssets,
    }
}

#[derive(Clone, Debug, Copy)]
pub(crate) enum DockCommandDispatcherType {
    LoadSVGAssets = 0,
    LoadLottieAssets = 1,
    RemoveEntity = 2,
    SpawnEntity = 3,
    Transform = 4,
    ModifyCamera = 5,
    PickEntity = 6,
    LoadParticleAssets = 7,
}

impl DockCommandDispatcherType {
    pub(crate) fn to_index(&self) -> u32 {
        *self as u32
    }
}

#[derive(Clone, Debug)]
pub enum DockCommandResult {
    Ok(u32),
    NotOk(String),
}
