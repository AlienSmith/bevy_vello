use bevy::prelude::*;
use bevy_vello::{
    collision::VelloCollisionTrigger,
    integrations::particles::{self, ExplosionEffect},
    VelloScene, VelloSceneBundle,
};
use vello::{kurbo, peniko};

pub(crate) fn on_collision_bullet(trigger: Trigger<VelloCollisionTrigger>, mut commands: Commands) {
    // let event = trigger.event();
    // let pos = event.collision_point;
    // let mut scene = VelloScene::default();
    // scene.push_instance_with_transforms(&[]);
    // scene.fill(
    //     peniko::Fill::NonZero,
    //     kurbo::Affine::default(),
    //     peniko::Color::rgba(1.0, 0.0, 0.0, 0.5),
    //     None,
    //     &kurbo::Circle::new((0.0, 0.0), 20.0),
    // );
    // scene.pop_instance();

    // commands.spawn((
    //     VelloSceneBundle {
    //         scene,
    //         transform: Transform::from_translation(pos.extend(100.0)),
    //         ..Default::default()
    //     },
    //     ExplosionEffect::new(
    //         particles::GravityParticleConfig {
    //             gravity: Vec2::new(0.0, -98.0),
    //             drag: 0.0,
    //             persistent: false,
    //         },
    //         particles::BurstEmitterConfig {
    //             count: 100,
    //             speed_range: (10.0, 100.0),
    //             lifetime_range: (1.0, 1.2),
    //             origin: Vec2::new(0.0, 0.0),
    //         },
    //         500,
    //     ),
    // ));
    commands.entity(trigger.target()).despawn();
}
