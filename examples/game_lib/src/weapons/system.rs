use bevy::prelude::*;
use bevy_vello::{integrations::physics::VelloParticle, VelloCollider};
use vello_physics::ConnectionConstraintInitConfig;

use crate::{
    weapons::{AttachPistolToCharacterEvent, PistolControl},
    CharacterPartEvent, ConnectivityRoot, LeftArmController, RightArmController,
};
pub fn attach_pistol(
    mut reader: EventReader<AttachPistolToCharacterEvent>,
    mut writer: EventWriter<CharacterPartEvent>,
    query: Query<(&PistolControl, &VelloCollider)>,
    q_p: Query<&VelloParticle>,
    q_controller: Query<(&LeftArmController, &RightArmController)>,
) {
    for item in reader.read() {
        let pivot = query.get(item.pistol).unwrap();
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
                None,
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
                None,
            ),
        });
    }
}
