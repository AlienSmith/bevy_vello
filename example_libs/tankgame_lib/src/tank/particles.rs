use bevy::prelude::*;
use bevy_hanabi::prelude::*;
use bevy_vello::{
    integrations::VelloSceneSubBundle,
    vello::{
        kurbo,
        peniko::{self, Color, GlowColor},
    },
    VelloScene,
};
#[derive(Resource, Clone, Default)]
pub struct ParticlesPlayer {
    pub explosion: Option<Handle<EffectAsset>>,
}

pub fn init_particles_player(
    mut effects: ResMut<Assets<EffectAsset>>,
    mut player: ResMut<ParticlesPlayer>,
) {
    if player.explosion.is_none() {
        let handle = effects.add(particle_lib::make_explosion_effect());
        player.explosion = Some(handle);
    }
}

use bevy_vello::velato::model::Value;

#[derive(Component, Clone, Default)]
pub struct ParticleSceneAnim {
    timer: Timer,
    radius: Value<f64>,
    color: Value<GlowColor>,
}

impl ParticleSceneAnim {
    pub fn new(
        duration: f32,
        mode: TimerMode,
        radius: Value<f64>,
        color: Value<GlowColor>,
    ) -> Self {
        Self {
            timer: Timer::from_seconds(duration, mode),
            radius,
            color,
        }
    }

    pub fn get_radius(&self, time_in_seconds: f64) -> f64 {
        let duration = self.timer.duration().as_secs_f64();
        let frame = time_in_seconds / duration;
        self.radius.evaluate(frame)
    }

    pub fn get_glow_color(&self, time_in_seconds: f64) -> GlowColor {
        let duration = self.timer.duration().as_secs_f64();
        let frame = time_in_seconds / duration;
        self.color.evaluate(frame)
    }
}

pub fn update_particle_scene(
    mut commands: Commands,
    mut query_scene: Query<(&mut VelloScene, &mut ParticleSceneAnim, Entity)>,
    time: Res<Time>,
) {
    for (mut scene, mut anim, entity) in query_scene.iter_mut() {
        anim.timer.tick(time.delta());
        let time = anim.timer.elapsed_secs() as f64;
        let radius = anim.get_radius(time);
        let glow_color = anim.get_glow_color(time);
        if let Some((particle_index, particle_size)) = scene.get_instance_index_in_export_buffer() {
            *scene = VelloScene::default();
            scene.fill(
                peniko::Fill::NonZero,
                kurbo::Affine::default(),
                glow_color,
                None,
                &kurbo::Circle::new(kurbo::Point { x: -5.0, y: 0.0 }, radius.into()),
            );
            scene.set_instance_index_in_export_buffer(particle_index, particle_size);
        }
        if anim.timer.finished() && anim.timer.mode() == TimerMode::Once {
            commands.entity(entity).despawn();
            info!("particle despawned");
        }
    }
}

pub fn spawn_particle_at(commands: &mut Commands, player: &Res<ParticlesPlayer>, translate: Vec3) {
    let effect = player.explosion.clone().unwrap();
    let anim = ParticleSceneAnim {
        timer: Timer::from_seconds(2.0, TimerMode::Once),
        radius: Value::Fixed(10.0),
        color: Value::Fixed(GlowColor {
            color: Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        }),
    };
    let radius = anim.get_radius(0.0);
    let glow_color = anim.get_glow_color(0.0);
    let mut scene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        glow_color,
        None,
        &kurbo::Circle::new(kurbo::Point { x: -5.0, y: 0.0 }, radius.into()),
    );
    commands.spawn((
        ParticleEffectBundle {
            // Assign the Z layer so it appears in the egui inspector and can be modified at runtime
            effect: ParticleEffect::new(effect).with_z_layer_2d(Some(0.1)),
            transform: Transform::from_translation(Vec3 {
                x: translate.x,
                y: translate.y,
                z: 100.0,
            }),
            ..default()
        },
        VelloSceneSubBundle {
            scene,
            ..Default::default()
        },
        anim,
    ));
}
