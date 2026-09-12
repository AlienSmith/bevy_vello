use bevy::prelude::*;

use crate::{
    weapons::{
        observer::on_raytrace_hit,
        system::{attach_pistol, process_fire_event, update_pistol_aim},
        AttachPistolToCharacterEvent, FireEvent, RayTraceHitPoints,
    },
    GameLabSystems,
};

#[derive(Default)]
pub struct WeaponPlugin;

impl Plugin for WeaponPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<AttachPistolToCharacterEvent>()
            .add_event::<FireEvent>()
            .init_resource::<RayTraceHitPoints>()
            .add_observer(on_raytrace_hit)
            .add_systems(
                Update,
                (attach_pistol, process_fire_event)
                    .chain()
                    .in_set(GameLabSystems::WriteCharacterPartEvent),
            )
            .add_systems(Update, update_pistol_aim);
    }
}
