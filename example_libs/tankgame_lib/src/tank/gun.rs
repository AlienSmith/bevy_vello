use bevy::prelude::*;
#[derive(Clone, Component)]
pub struct Gun {
    timer: Option<Timer>,
    time_in_seconds: f32,
    recoil_distance: f32,
}
impl Default for Gun {
    fn default() -> Self {
        Self {
            timer: None,
            time_in_seconds: 1.0,
            recoil_distance: 17.0,
        }
    }
}

impl Gun {
    pub fn new(time_in_seconds: f32, recoil_distance: f32) -> Self {
        Self {
            timer: None,
            time_in_seconds,
            recoil_distance,
        }
    }
}

pub fn control_system(
    time: Res<Time>,
    button: Res<ButtonInput<MouseButton>>,
    mut query_scene: Query<(&mut Transform, &mut Gun, &GlobalTransform)>,
) {
    let (mut transform, mut gun, _global_transform) = query_scene.single_mut();
    if button.pressed(MouseButton::Left) {
        if gun.timer.is_none() {
            gun.timer = Some(Timer::from_seconds(gun.time_in_seconds, TimerMode::Once));
        }
    }
    let time_in_seconds = gun.time_in_seconds;
    let recoil = gun.recoil_distance * -1.0;
    if let Some(timer) = &mut gun.timer {
        timer.tick(time.delta());
        let t = timer.elapsed_secs();
        let middle = time_in_seconds * 0.3;
        let end = time_in_seconds;
        let y_offset = if t < middle {
            let t0 = t / middle;
            //take the slow down half part of smoothstep
            let t1 = t0 * 0.5 + 0.5;
            let ss = t1 * t1 * (3.0 - 2.0 * t1);
            ss * recoil
        } else {
            let t1 = (t - middle) / (end * 0.7);
            let ss = t1 * t1 * (3.0 - 2.0 * t1);
            (recoil) * (1.0 - ss)
        };
        transform.translation.y = y_offset;
        if timer.finished() {
            gun.timer = None;
        }
    }
}
