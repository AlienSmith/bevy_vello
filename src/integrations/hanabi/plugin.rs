use bevy::{ecs::query, prelude::*};
use bevy_hanabi::prelude::*;

use crate::{CoordinateSpace, VelloScene};
pub struct HanabiIntegrationPlugin;

#[derive(Bundle, Default)]
pub struct VelloSceneSubBundle {
    pub scene: VelloScene,
    /// The coordinate space in which this scene should be rendered.
    pub coordinate_space: CoordinateSpace,
}

impl Plugin for HanabiIntegrationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(HanabiPlugin);
        app.add_systems(
            PostUpdate,
            add_remove_effect.in_set(EffectSystems::BeforeCompile),
        );
    }
}

fn add_remove_effect(
    effect_assets: Res<Assets<EffectAsset>>,
    mut asset_counter: ResMut<EffectAssetCounter>,
    mut query: Query<
        (Entity, &mut ParticleEffect, &mut VelloScene),
        (Added<CompiledParticleEffect>, Added<VelloScene>),
    >,
    mut removed_effects: RemovedComponents<ParticleEffect>,
) {
    for (entity, mut effect, mut scene) in query.iter_mut() {
        let size = effect_assets
            .get(effect.handle.clone())
            .unwrap()
            .capacities()
            .len();
        let token = asset_counter.alloc(entity.clone(), size as u32);
        println!("initialized {}", token.index);
        effect.token = token;
        scene.set_instance_index_in_export_buffer(token.index, token.size);
    }
    let entities: Vec<Entity> = removed_effects.read().collect();
    for entity in &entities {
        asset_counter.free(entity);
    }
}
