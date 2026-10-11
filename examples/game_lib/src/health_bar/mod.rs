use bevy::prelude::*;
use bevy_vello::integrations::physics::VelloParticle;
use bevy_vello::VelloScene;
use vello::kurbo::{Affine, Rect};
use vello::peniko;

use crate::{damage::TotalHealth, CharacterRoot};

/// A 2D health-bar overlay that follows a character's right-arm wrist particle.
///
/// The bar is drawn into its own [`VelloScene`] each frame, positioned at the
/// wrist. The fill color starts **pink** (full health) and shifts to **red** as
/// the character's [`TotalHealth`] decreases.
#[derive(Component)]
pub struct HealthBar {
    /// The character root entity carrying the [`TotalHealth`].
    pub character: Entity,
    /// The wrist particle entity (e.g. `RightArmController.particles[3]`) the
    /// bar tracks for position.
    pub wrist_entity: Entity,
    /// Full width of the bar in world units.
    pub width: f32,
    /// Height of the bar in world units.
    pub height: f32,
}

/// Rebuilds the health bar scene each frame: positions it at the wrist and
/// re-draws the background + fill with the current health ratio.
pub fn update_health_bar(
    mut q: Query<(&HealthBar, &mut VelloScene, &mut Transform)>,
    total_q: Query<&TotalHealth, With<CharacterRoot>>,
    particle_q: Query<&VelloParticle>,
) {
    for (bar, mut scene, mut transform) in q.iter_mut() {
        let Ok(total) = total_q.get(bar.character) else {
            continue;
        };
        let Ok(particle) = particle_q.get(bar.wrist_entity) else {
            continue;
        };

        // Vello particle pos is world-space y-down; convert to bevy y-up.
        let pos = particle.particle.pos;
        transform.translation = Vec3::new(pos.x, -pos.y, transform.translation.z);

        let ratio = (total.current / total.max).clamp(0.0, 1.0);

        // Full health = pink, empty = red.
        //   pink: (1.0, 0.6, 0.8), red: (1.0, 0.0, 0.0)
        let color = peniko::Color::rgba(1.0, 0.6 * ratio as f64, 0.8 * ratio as f64, 1.0);

        let hw = (bar.width * 0.5) as f64;
        let hh = (bar.height * 0.5) as f64;

        scene.reset();

        // Dark background outline.
        scene.fill(
            peniko::Fill::NonZero,
            Affine::IDENTITY,
            peniko::Color::rgba(0.0, 0.0, 0.0, 0.6),
            None,
            &Rect::new(-hw, -hh, hw, hh),
        );

        // Colored fill, width scales with remaining health.
        let fill_hw = hw * ratio as f64;
        scene.fill(
            peniko::Fill::NonZero,
            Affine::IDENTITY,
            color,
            None,
            &Rect::new(-fill_hw, -hh, fill_hw, hh),
        );
    }
}

/// Plugin that registers the health-bar update system.
pub struct HealthBarPlugin;

impl Plugin for HealthBarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, update_health_bar);
    }
}
