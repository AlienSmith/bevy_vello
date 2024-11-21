use bevy::prelude::Transform;

#[derive(Clone, Debug)]
pub enum DockCommand {
    LoadSVGAssets(Vec<u8>),
    LoadLottieAssets(Vec<u8>),
    RemoveAssets(u32),
    SpawnEntity(u32, Transform),
    Transform(u32, Transform),
}

pub(crate) fn command_to_dispatcher(command: &DockCommand) -> DockCommandDispatcherType {
    match command {
        DockCommand::LoadSVGAssets(_) => DockCommandDispatcherType::LoadSVGAssets,
        DockCommand::LoadLottieAssets(_) => DockCommandDispatcherType::LoadLottieAssets,
        DockCommand::RemoveAssets(_) => DockCommandDispatcherType::RemoveAssets,
        DockCommand::SpawnEntity(_, _) => DockCommandDispatcherType::SpawnEntity,
        DockCommand::Transform(_, _) => DockCommandDispatcherType::Transform,
    }
}

#[derive(Clone, Debug, Copy)]
pub(crate) enum DockCommandDispatcherType {
    LoadSVGAssets = 0,
    LoadLottieAssets = 1,
    RemoveAssets = 2,
    SpawnEntity = 3,
    Transform = 4,
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
