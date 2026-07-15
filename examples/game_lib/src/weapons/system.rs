use std::ops::Mul;

use bevy::{prelude::*, transform};
use bevy_vello::{integrations::physics::VelloParticle, mat4_to_affine, VelloCollider, VelloScene};
use vello::{
    kurbo::{Affine, BezPath, PathEl, Shape, Stroke},
    peniko::{self, GlowColor},
};
use vello_physics::{
    utility::{bilinear_reconstruct, vector2_to_kurbo_point},
    ConnectionConstraintInitConfig,
};

use crate::{
    character::Connectivity,
    utility::mat4_to_affine2,
    weapons::{AttachPistolToCharacterEvent, PistolControl},
    CharacterPartEvent, ConnectivityRoot, LeftArmController, RightArmController, StringPool,
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
            info!("draw aim line {:?}, {:?}", l_p, l_r);
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
