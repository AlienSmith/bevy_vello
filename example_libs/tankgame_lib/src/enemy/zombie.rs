use crate::{
    collision::{ColliderFlags, ColliderResponds, Health, SingleFrameCollider},
    make_sprite_sheet_scene_from_vello_replay_scene, TankGameAssets, TankGameAssetsType,
};
use avian2d::prelude::*;
use bevy::{
    math::{vec2, vec3},
    prelude::*,
    utils::HashMap,
};
use bevy_vello::prelude::*;
use seldom_state::prelude::*;

#[derive(Clone, Component, Default)]
pub struct ZombieInputComponent {
    pub event: Option<ZombieInputEvent>,
}

#[derive(Clone, Copy)]
pub enum ZombieInputEvent {
    Attack,
    MoveTo(Vec2),
}

#[derive(Clone, Component)]
pub struct Zombie;

#[derive(Clone, Component)]
#[component(storage = "SparseSet")]
pub struct Idle;

#[derive(Clone, Copy, Component)]
#[component(storage = "SparseSet")]
pub struct GoToSelection {
    speed: f32,
    target: Vec2,
}

#[derive(Clone, Component)]
#[component(storage = "SparseSet")]
pub struct Attack {
    anim_timer: Option<Timer>,
    collider_spawn_timer: Option<Timer>,
}

pub fn spawn_zombie_at(
    commands: &mut Commands,
    parts: &Res<TankGameAssets>,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    translation: Vec3,
    scale: f32,
) {
    let mut b_s = VelloScene::default();
    make_sprite_sheet_scene_from_vello_replay_scene(
        &mut b_s,
        &custom_assets,
        &parts,
        TankGameAssetsType::ZOMBIE_IDEL,
        None,
        None,
        None,
    );

    let move_trigger = move |In(entity): In<Entity>, inputs: Query<&ZombieInputComponent>| {
        if let Ok(component) = inputs.get(entity) {
            if let Some(event) = component.event {
                match event {
                    ZombieInputEvent::MoveTo(pos) => {
                        info!("zombie moved");
                        return Ok(pos);
                    }
                    _ => {}
                }
            }
        }
        return Err(());
    };

    let attack_trigger = move |In(entity): In<Entity>, inputs: Query<&ZombieInputComponent>| {
        if let Ok(component) = inputs.get(entity) {
            if let Some(event) = component.event {
                match event {
                    ZombieInputEvent::Attack => {
                        info!("zombie attacked");
                        return Ok(());
                    }
                    _ => {}
                }
            }
        }
        return Err(());
    };
    let circle_with_offset = vec![(
        Position(Vec2 { x: -20.0, y: 0.0 }),
        Rotation::default(),
        Collider::circle(70.0),
    )];
    // commands.spawn();
    commands.spawn((
        ZombieInputComponent::default(),
        Zombie,
        Idle,
        StateMachine::default()
            .trans_builder(move_trigger, |_: &Idle, pos| {
                Some(GoToSelection {
                    speed: 200.,
                    target: pos,
                })
            })
            // When the player clicks, go there
            // `done` triggers when the `Done` component is added to the entity. When they're done
            // going to the selection, idle.
            .trans::<GoToSelection, _>(done(Some(Done::Success)), Idle)
            .trans::<AnyState, _>(
                attack_trigger,
                Attack {
                    anim_timer: None,
                    collider_spawn_timer: None,
                },
            )
            .trans::<Attack, _>(done(Some(Done::Success)), Idle)
            .set_trans_logging(true),
        VelloSceneBundle {
            scene: b_s,
            transform: Transform::from_scale(Vec3 {
                x: scale,
                y: scale,
                z: 1.0,
            })
            .with_translation(translation),
            ..Default::default()
        },
        Collider::compound(circle_with_offset),
        ColliderResponds {
            damage: 1.0,
            allowed_collider_masks: ColliderFlags::SHELL | ColliderFlags::EXPLOSION,
            collider_type: ColliderFlags::ALIEN,
            spawn_damage_text: true,
        },
        Health { health: 3.0 },
    ));
}

pub fn on_add_attack_to_zombie(
    time: Res<Time>,
    mut query: Query<(&mut VelloScene, &mut Attack), (Added<Attack>, With<Zombie>)>,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    let mut temp: Option<VelloScene> = None;
    for (mut scene, mut attack) in query.iter_mut() {
        if temp.is_none() {
            let start_time = time.elapsed_seconds();
            let mut b_s = VelloScene::default();
            make_sprite_sheet_scene_from_vello_replay_scene(
                &mut b_s,
                &custom_assets,
                &parts,
                TankGameAssetsType::ZOMBIE_ATTACK,
                Some(false),
                Some(start_time),
                Some(9.0),
            );
            temp = Some(b_s);
        }
        *scene = temp.clone().unwrap();
        attack.anim_timer = Some(Timer::from_seconds(1.1, TimerMode::Once));
        attack.collider_spawn_timer = Some(Timer::from_seconds(6.5 / 9.0, TimerMode::Once));
    }
}

pub fn update_zombie_attack(
    mut commands: Commands,
    time: Res<Time>,
    mut query: Query<(Entity, &GlobalTransform, &mut Attack), With<Zombie>>,
) {
    for (entity, transform, mut attack) in query.iter_mut() {
        if let Some(timer) = &mut attack.anim_timer {
            timer.tick(time.delta());
            if timer.finished() {
                commands.entity(entity).insert(Done::Success);
                info!("Done!")
            }
        }
        if let Some(timer) = &mut attack.collider_spawn_timer {
            timer.tick(time.delta());
            if timer.finished() {
                attack.collider_spawn_timer = None;
                // these infomation are extracted from the image
                let collider_center = vec2(60.0, 0.0);
                let radius: f32 = 60.0;
                let translate = transform.transform_point(collider_center.extend(0.0));
                let (scale, _, _) = transform.to_scale_rotation_translation();
                let radius = scale.x * radius;
                spawn_zombie_damage_collider(&mut commands, translate, radius);
            }
        }
    }
}

pub fn on_add_move_to_zombie(
    time: Res<Time>,
    mut query: Query<&mut VelloScene, (Added<GoToSelection>, With<Zombie>)>,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    let mut temp: Option<VelloScene> = None;
    for mut scene in query.iter_mut() {
        if temp.is_none() {
            let start_time = time.elapsed_seconds();
            let mut b_s = VelloScene::default();
            make_sprite_sheet_scene_from_vello_replay_scene(
                &mut b_s,
                &custom_assets,
                &parts,
                TankGameAssetsType::ZOMBIE_MOVE,
                None,
                Some(start_time),
                None,
            );
            temp = Some(b_s);
        }
        *scene = temp.clone().unwrap();
    }
}

pub fn on_add_idle_to_zombie(
    time: Res<Time>,
    mut query: Query<&mut VelloScene, (Added<Idle>, With<Zombie>)>,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    let mut temp: Option<VelloScene> = None;
    for mut scene in query.iter_mut() {
        if temp.is_none() {
            let start_time = time.elapsed_seconds();
            let mut b_s = VelloScene::default();
            make_sprite_sheet_scene_from_vello_replay_scene(
                &mut b_s,
                &custom_assets,
                &parts,
                TankGameAssetsType::ZOMBIE_IDEL,
                None,
                Some(start_time),
                None,
            );
            temp = Some(b_s);
        }
        *scene = temp.clone().unwrap();
    }
}

pub fn zombie_go_to_target_update(
    mut commands: Commands,
    mut go_to_selections: Query<
        (
            Entity,
            &mut Transform,
            &GlobalTransform,
            &mut GoToSelection,
            &ZombieInputComponent,
        ),
        With<Zombie>,
    >,
    time: Res<Time>,
) {
    for (entity, mut transform, global_transform, mut go_to_selection, input_component) in
        &mut go_to_selections
    {
        if let Some(event) = input_component.event {
            match event {
                ZombieInputEvent::MoveTo(pos) => go_to_selection.target = pos,
                _ => {}
            }
        }

        let target = go_to_selection.target;
        let delta = target - transform.translation.truncate();
        let movement = delta.normalize_or_zero() * go_to_selection.speed * time.delta_seconds();
        let global_to_local = global_transform.compute_matrix().inverse();
        let dif_3_local = global_to_local.transform_vector3(movement.extend(0.0));
        let x = vec3(1.0, 0.0, 0.0);
        let cross = x.cross(dif_3_local).z;
        let angle = x.angle_between(dif_3_local);
        let rotate_direction = cross.signum();
        if movement.length() > delta.length() {
            transform.translation = target.extend(transform.translation.z);
            // The player has reached the target! Add the `Done` component to the player, causing
            // `done` to trigger. It will be automatically removed later this frame.
            commands.entity(entity).insert(Done::Success);
            info!("Done!")
        } else {
            transform.translation += movement.extend(0.);
        }
        transform.rotate_z(rotate_direction * angle);
    }
}

pub fn spawn_zombie_damage_collider(commands: &mut Commands, translate: Vec3, scale: f32) {
    //damage
    commands.spawn((
        Transform::from_translation(Vec3 {
            x: translate.x,
            y: translate.y,
            z: 0.0,
        }),
        Health { health: 1.0 },
        Collider::circle(scale),
        ColliderResponds {
            damage: 2.0,
            allowed_collider_masks: ColliderFlags::None,
            collider_type: ColliderFlags::ENEMY_DAMAGE,
            ..Default::default()
        },
        SingleFrameCollider::default(),
    ));
}
