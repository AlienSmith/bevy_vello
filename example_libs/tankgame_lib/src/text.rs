use bevy::prelude::*;
use bevy_vello::prelude::*;
use bevy_vello::velato::model::{Animated, Value};
use bevy_vello::vello::peniko::GlowColor;

#[derive(Clone, Default, Component)]
pub struct PopTextAnim {
    timer: Timer,
    pos: Value<kurbo::Point>,
    scale: Value<f64>,
    color: Value<GlowColor>,
}
impl PopTextAnim {
    pub fn new(
        duration: f32,
        pos: Value<kurbo::Point>,
        scale: Value<f64>,
        color: Value<GlowColor>,
    ) -> Self {
        Self {
            timer: Timer::from_seconds(duration, TimerMode::Once),
            pos,
            scale,
            color,
        }
    }

    pub fn get_pos(&self, time_in_seconds: f64) -> kurbo::Point {
        let duration = self.timer.duration().as_secs_f64();
        let frame = time_in_seconds / duration;
        self.pos.evaluate(frame)
    }

    pub fn get_scale(&self, time_in_seconds: f64) -> f64 {
        let duration = self.timer.duration().as_secs_f64();
        let frame = time_in_seconds / duration;
        self.scale.evaluate(frame)
    }

    pub fn get_glow_color(&self, time_in_seconds: f64) -> GlowColor {
        let duration = self.timer.duration().as_secs_f64();
        let frame = time_in_seconds / duration;
        self.color.evaluate(frame)
    }
}
fn linear_move_upward(base: kurbo::Point, size: f32) -> Value<kurbo::Point> {
    let x_offset: f64 = rand::random();
    let size = size as f64;
    use bevy_vello::velato::model::Time;
    use bevy_vello::velato::runtime::SimpleEasing;
    let times = vec![
        Time::new(0.0, SimpleEasing::EaseOut, false),
        Time::new(0.7, SimpleEasing::None, true),
    ];
    let values = vec![base, base + (x_offset * size, size * 3.0)];
    Value::Animated(Animated { times, values })
}

pub fn spawn_pop_text_at(
    commands: &mut Commands,
    fonts: &Res<DefaultFonts>,
    translate: Vec3,
    string: &str,
    size: f32,
) {
    let base_x = translate.x as f64;
    let base_y = translate.y as f64;
    let anim = PopTextAnim::new(
        2.0,
        linear_move_upward((base_x, base_y).into(), size),
        Value::Fixed(1.0),
        Value::Fixed(GlowColor {
            color: peniko::Color::rgb(0.0, 1.0, 0.0),
            glow: 1.0,
        }),
    );
    let color = anim.get_glow_color(0.0);

    commands.spawn((
        VelloTextBundle {
            font: fonts.default_font.clone(),
            text: VelloText {
                content: string.to_string(),
                size,
                brush: Some(peniko::Brush::SolidGlow(color)),
            },
            transform: Transform::from_xyz(translate.x, translate.y, 1000.0),
            debug_visualizations: DebugVisualizations::Hidden,
            alignment: VelloTextAlignment::Center,
            ..default()
        },
        anim,
    ));
}

pub fn pop_text_update(
    mut command: Commands,
    mut query: Query<(&mut Transform, &mut VelloText, &mut PopTextAnim, Entity)>,
    time: Res<Time>,
) {
    for (mut transform, mut vellotext, mut anim, entity) in query.iter_mut() {
        anim.timer.tick(time.delta());
        let time = anim.timer.elapsed_secs() as f64;
        let pos = anim.get_pos(time);
        let scale = anim.get_scale(time) as f32;
        let color = anim.get_glow_color(time);
        transform.translation.x = pos.x as f32;
        transform.translation.y = pos.y as f32;
        transform.scale = Vec3::new(scale, scale, 1.0);
        vellotext.brush = Some(peniko::Brush::SolidGlow(color));
        if anim.timer.finished() {
            info!("text despawn");
            command.entity(entity).despawn();
        }
    }
}

#[derive(Resource, Clone, Default)]
pub struct DefaultFonts {
    pub default_font: Handle<VelloFont>,
}
