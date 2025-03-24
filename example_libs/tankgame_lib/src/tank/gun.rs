use bevy::prelude::*;
#[derive(Clone, Default, Component)]
pub struct Gun {}

pub fn control_system(
    _time: Res<Time>,
    button: Res<ButtonInput<MouseButton>>,
    mut _query_scene: Query<(&mut Transform, &Gun, &GlobalTransform)>,
) {
    //let (mut transform, gun, _global_transform) = query_scene.single_mut();
    if button.pressed(MouseButton::Left) {
        info!("fire");
    }
}
