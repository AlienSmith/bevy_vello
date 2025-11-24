pub use bevy::prelude::*;

use crate::integrations::particles::ExplosionEffect;
pub fn update_explosion_effects(
    mut commands: Commands,
    mut query: Query<(Entity, &mut ExplosionEffect)>,
    time: Res<Time>,
) {
    let mut rng = rand::thread_rng();
    let delta = time.delta_seconds();

    for (entity, mut effect) in query.iter_mut() {
        effect.0.update(delta, &mut rng);
        if !effect.0.active && effect.0.is_empty() {
            commands.entity(entity).despawn();
        }
    }
}
