use std::ops::Mul;

use bevy::ecs::intern::Interned;

/// Default bullet damage profile. `cut_damage` applies uniformly to armor/part;
/// `blunt_damage` is gated by `penetration` vs the target's `protection_level`.
const BULLET_BLUNT_DAMAGE: f32 = 20.0;
const BULLET_CUT_DAMAGE: f32 = 8.0;
const BULLET_PENETRATION: f32 = 3.0;

use bevy::{prelude::*, tasks::block_on, transform};
use bevy_egui::egui::Key::W;
use bevy_vello::{
    integrations::physics::{ColliderExternalImpulseEvent, VelloParticle},
    mat4_to_affine, VelloCollider, VelloScene, VelloSceneBundle,
};
use vello::{
    kurbo::{Affine, BezPath, PathEl, Shape, Stroke},
    peniko::{self, GlowColor},
};
use vello_physics::{
    utility::{bilinear_distribute, bilinear_reconstruct, vector2_to_kurbo_point},
    CollisionConstraintConfig, ConnectionConstraintInitConfig, SoftBodyInitConfig,
};

use crate::{
    character::Connectivity,
    damage::components::AttackStats,
    utility::mat4_to_affine2,
    weapons::{
        observer::on_collision_bullet, AttachPistolToCharacterEvent, Bullet, FireEvent,
        MeleeWeapon, PistolControl,
    },
    CharacterPartEvent, ColliderRoot, ConnectivityRoot, LeftArmController, RightArmController,
    StringPool,
};

pub fn attach_pistol(
    mut reader: EventReader<AttachPistolToCharacterEvent>,
    mut writer: EventWriter<CharacterPartEvent>,
    query: Query<(&PistolControl, &VelloCollider)>,
    q_p: Query<&VelloParticle>,
    q_controller: Query<(&LeftArmController, &RightArmController)>,
) {
    for item in reader.read() {
        let (control, collider) = query.get(item.pistol).unwrap();
        let (controller_left, controller_right) = q_controller.get(item.character).unwrap();
        let pos_elbow = q_p
            .get(controller_right.particles[2])
            .unwrap()
            .particle_init
            .pos;
        let pos_wrist = q_p
            .get(controller_right.particles[3])
            .unwrap()
            .particle_init
            .pos;
        let length = (pos_wrist - pos_elbow).length();
        let u_offset = length / collider.initial_scale.x;
        let elbow_uv = control.wrist_binding_uv - Vec2::new(u_offset, 0.0);
        writer.write(CharacterPartEvent::RegisterPart {
            character: item.character,
            entity: item.pistol,
            path_id: "pistol".to_string(),
        });

        writer.write(CharacterPartEvent::AddJoint {
            character: item.character,
            path_id: "pistol_prla".to_string(),
            config: ConnectionConstraintInitConfig::Bilinear(
                "PRLA".to_string(),
                "pistol".to_string(),
                0.0,
                Some(control.wrist_binding_uv),
            ),
        });

        // Also connect the pistol collider to P13 (right arm elbow) via a bilinear
        // joint for additional stability.
        writer.write(CharacterPartEvent::AddJoint {
            character: item.character,
            path_id: "pistol_p13".to_string(),
            config: ConnectionConstraintInitConfig::Bilinear(
                "P13".to_string(),
                "pistol".to_string(),
                0.0,
                //None,
                Some(elbow_uv),
            ),
        });
    }
}

pub fn update_pistol_aim(
    mut pistol_q: Query<(
        &PistolControl,
        &Connectivity,
        &VelloCollider,
        &mut VelloScene,
        &GlobalTransform,
    )>,
    mut arm_q: Query<(&mut LeftArmController, &mut RightArmController)>,
) {
    // the connectivity component means we are connected to some character
    for (control, connectivity, collider, mut scene, transform) in pistol_q.iter_mut() {
        let character = connectivity.character;
        let y_scale = collider.initial_scale.y;
        let y_offset = (control.gun_point_uv.y - control.wrist_binding_uv.y) * y_scale;
        // set the target for controller
        if let Ok((_left, mut right)) = arm_q.get_mut(character) {
            if let Some(target) = control.world_aim_trarget {
                right.config.ik_mode = crate::IkMode::Aim {
                    weapon_offset_y: y_offset,
                };
                right.target = target;
            } else {
                right.config.ik_mode = crate::IkMode::Disabled;
            }
        }
        if control.enable_aim_line {
            // to local space
            let affine = mat4_to_affine2(transform.compute_matrix()).inverse();
            let frame_position: Vec<Vec2> =
                collider.frame_particles.iter().map(|p| p.pos).collect();
            let gun_point_world_pos =
                bilinear_reconstruct(control.gun_point_uv, &frame_position.as_slice());
            let gun_rear_world_pos = bilinear_reconstruct(
                Vec2::new(control.wrist_binding_uv.x, control.gun_point_uv.y),
                &frame_position.as_slice(),
            );
            let l_p = affine.transform_point2(gun_point_world_pos);
            let l_r = affine.transform_point2(gun_rear_world_pos);
            let ray = (l_p - l_r).normalize();
            let end = l_p + ray * 1000.0;
            let mut frame = vec![];
            frame.push(PathEl::MoveTo(vector2_to_kurbo_point(&l_p)));
            frame.push(PathEl::LineTo(vector2_to_kurbo_point(&end)));
            scene.stroke(
                &Stroke::new(0.5),
                Affine::IDENTITY,
                GlowColor {
                    color: peniko::Color::rgba(1.0, 0.0, 0.0, 0.9),
                    glow: 5.0,
                },
                None,
                &frame.into_path(0.1),
            );
        }
    }
}

pub fn process_fire_event(
    mut commands: Commands,
    mut reader: EventReader<FireEvent>,
    mut pistol_q: Query<(Entity, &mut PistolControl, &VelloCollider)>,
    mut writer: EventWriter<ColliderExternalImpulseEvent>,
    time: Res<Time>,
) {
    let now = time.elapsed_secs();
    for fire in reader.read() {
        if let Ok((entity, mut control, collider)) = pistol_q.get_mut(fire.weapon) {
            if (control.last_fire_time + control.fire_cool_down) > now {
                continue;
            }
            control.last_fire_time = now;
            let frame_position: Vec<Vec2> =
                collider.frame_particles.iter().map(|p| p.pos).collect();
            //interpolate the vello world position
            let w_p = bilinear_reconstruct(control.gun_point_uv, &frame_position.as_slice());
            let w_r = bilinear_reconstruct(
                Vec2::new(control.wrist_binding_uv.x, control.gun_point_uv.y),
                &frame_position.as_slice(),
            );
            // convert to bevy world position
            let b_p = Vec2::new(w_p.x, -w_p.y);
            let b_r = Vec2::new(w_r.x, -w_r.y);
            let x_ray = (b_p - b_r).normalize();
            let angle = x_ray.y.atan2(x_ray.x);
            let init_bevy_transform = Transform::from_translation(b_p.extend(0.0));
            let soft_body_init_transform = Transform {
                translation: b_p.extend(0.0),
                rotation: Quat::from_rotation_z(angle),
                scale: Vec3::new(0.05, 0.05, 1.0),
            };
            let softbody_config = SoftBodyInitConfig::default();
            commands
                .spawn((
                    VelloSceneBundle {
                        transform: init_bevy_transform,
                        ..Default::default()
                    },
                    ColliderRoot {
                        svg_asset_id: "ammo.collider.svg".to_string(),
                        albedo_asset_id: "ammo_albedo.png".to_string(),
                        normal_asset_id: "ammo_normal.png".to_string(),
                        metallic: 0.9,
                        roughness: 0.2,
                        softbody_config,
                        collision_config: CollisionConstraintConfig::default(),
                        soft_body_init_transform,
                        initial_velocity: x_ray * 1000.0,
                        collision_group: fire.projectile_collision_group,
                        collision_inverse_mass: 0.0,
                    },
                    Bullet,
                    // Tunable bullet damage profile. Cut applies uniformly to
                    // armor/part; blunt is gated by penetration vs protection.
                    AttackStats::new(BULLET_BLUNT_DAMAGE, BULLET_CUT_DAMAGE, BULLET_PENETRATION),
                ))
                .observe(on_collision_bullet);
            let back = -x_ray;
            let up = Vec2::new(x_ray.y, -x_ray.x);
            let recoil =
                (back + control.recoil_kickup * up).normalize() * control.recoil_kick_scale;
            let weights = bilinear_distribute(control.gun_point_uv);
            writer.write(ColliderExternalImpulseEvent {
                entity,
                impulse: [
                    recoil * weights.x,
                    recoil * weights.y,
                    recoil * weights.z,
                    recoil * weights.w,
                ],
            });
        }
    }
}

/// Update melee weapon impulse based on swing state.
/// Runs in Update, before collision observers fire in PostUpdate.
/// Computes the explosion impulse from the weapon's frame particle velocities.
pub fn update_melee_weapon_impulse(mut melee_q: Query<(&mut MeleeWeapon, &VelloCollider)>) {
    for (mut melee, collider) in melee_q.iter_mut() {
        // Compute swing direction from frame particle velocities
        let frame_velocity: Vec2 = collider.frame_particles.iter().map(|p| p.velocity).sum();
        let speed = frame_velocity.length();

        if speed > 10.0 {
            // Weapon is swinging — set explosion impulse in swing direction
            let direction = frame_velocity.normalize();
            melee.explosion_impulse = direction * speed * 50.0; // tunable scale
        } else {
            melee.explosion_impulse = Vec2::ZERO;
        }
    }
}
