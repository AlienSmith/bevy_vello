use bevy::prelude::*;
use bevy_vello::{integrations::physics::VelloParticle, VelloCollider};
use vello_physics::ConnectionConstraintInitConfig;

use crate::{
    character::Connectivity,
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
            .get(controller_left.particles[2])
            .unwrap()
            .particle_init
            .pos;
        let pos_wrist = q_p
            .get(controller_left.particles[3])
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
    pistol_q: Query<(&PistolControl, &Connectivity, &VelloCollider)>,
    mut arm_q: Query<(&mut LeftArmController, &mut RightArmController)>,
) {
    // the connectivity component means we are connected to some character
    for (control, connectivity, collider) in &pistol_q {
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
    }
}
