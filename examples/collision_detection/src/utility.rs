use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    math::VectorSpace,
    prelude::*,
    window::PrimaryWindow,
};
use bevy_egui::{egui, EguiContexts};
use bevy_vello::{
    collision::{
        CollisionConstraintConfig, SoftBodyInitConfig, VelloCollisionBroadPhase,
        VelloCollisionWorld,
    },
    integrations::{
        particles::Particle,
        physics::{
            ColliderExternalImpulseEvent, ConnectionInitConfig,
            ExternalForce::{self, Impulse},
            FilterData, ParticleInfo, VelloJoint, VelloParticle,
        },
    },
    vello::{
        kurbo::{self, Affine, Stroke},
        peniko,
    },
    VelloCollider, VelloScene, VelloSceneBundle,
};

use crate::connections::ConnectionStatus;

#[derive(Component)]
pub struct StaticSceneComponent;

const GRAVITY: f32 = 0.0;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(u32)]
pub enum ColliderType {
    #[default]
    Rect = 0,
    Circle = 1,
    Star = 2,
    Heart = 3,
    Key = 4,
    Shield = 5,
    Knife = 6,
    Capsule = 7,
    Ammo = 8,
    PISTOL = 9,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(u32)]
pub enum ExteralEffectType {
    #[default]
    Selection = 0,
    DragFrameAll = 1,
    AddPin = 2, //Add a Pivot to a single entity
    DragJoint = 3,
    AddPinJoint = 4, //Add a Single Point Joint to two Entity
    SnapJointToMouse = 5,
}

pub fn get_default_parameters(collider: ColliderType) -> (peniko::Color, f32, i32) {
    match collider {
        ColliderType::Rect => (peniko::Color::GREEN, 1.0, 1),
        ColliderType::Circle => (peniko::Color::WHITE, 1.0, 0),
        ColliderType::Star => (peniko::Color::ORANGE, 0.3, 1),
        ColliderType::Heart => (peniko::Color::RED, 0.5, 1),
        ColliderType::Key => (peniko::Color::GREEN, 0.1, 0),
        ColliderType::Shield => (peniko::Color::CYAN, 0.1, 1),
        ColliderType::Knife => (peniko::Color::YELLOW, 0.3, 1),
        ColliderType::Ammo => (peniko::Color::GREEN, 0.1, 1),
        ColliderType::PISTOL => (peniko::Color::GREEN, 0.1, 1),
        ColliderType::Capsule => (peniko::Color::PINK, 0.5, 1),
    }
}

#[derive(PartialEq, Clone)]
pub(crate) struct ExternalImpulseConfig {
    pub(crate) scale: f32,
    pub(crate) drag_type: ExteralEffectType,
}

#[derive(PartialEq, Clone)]
pub(crate) struct EntityConfig {
    pub(crate) pos_x: f32,
    pub(crate) pos_y: f32,
    pub(crate) vec_x: f32,
    pub(crate) vec_y: f32,
    pub(crate) rotation: f32,
    pub(crate) scale: f32,
    pub(crate) scale_modifier: f32,
    pub(crate) connection_complaince: f32,
    pub(crate) collider_type: ColliderType,
}

#[derive(PartialEq, Clone)]
pub(crate) struct VelloConstraintWorldConfig {
    pub(crate) gravity_x: f32,
    pub(crate) gravity_y: f32,

    pub(crate) pre_gravity_x: f32,
    pub(crate) pre_gravity_y: f32,
}

impl Default for ExternalImpulseConfig {
    fn default() -> Self {
        Self {
            scale: 1.0,
            drag_type: ExteralEffectType::Selection,
        }
    }
}

impl Default for VelloConstraintWorldConfig {
    fn default() -> Self {
        Self {
            gravity_x: 0.0,
            gravity_y: GRAVITY,
            pre_gravity_x: 0.0,
            pre_gravity_y: GRAVITY,
        }
    }
}

impl Default for EntityConfig {
    fn default() -> Self {
        EntityConfig {
            scale: 1.0,
            scale_modifier: 1.0,
            pos_x: 0.0,
            pos_y: 0.0,
            vec_x: 0.0,
            vec_y: 0.0,
            rotation: 0.0,
            connection_complaince: 0.01,
            collider_type: Default::default(),
        }
    }
}

#[derive(Default, Component)]
pub struct Preview;

#[derive(Default, Resource)]

pub(crate) struct UiState {
    pub(crate) current: EntityConfig,
    pub(crate) just_spawn: bool,
    pub(crate) c_config: VelloConstraintWorldConfig,
    pub(crate) just_modified: bool,
    pub(crate) e_config: ExternalImpulseConfig,
    pub(crate) soft_body_config: SoftBodyInitConfig,
    pub(crate) collision_config: CollisionConstraintConfig,
    pub(crate) delete_all_dynamic: bool,
    pub(crate) spawn_particle_effect: bool,
    pub(crate) preview_state: bool,
    pub(crate) preview_state_just_modified: bool,
}

pub fn ui_example_system(
    mut ui_state: ResMut<UiState>,
    mut contexts: EguiContexts,
    diagnostics: Res<DiagnosticsStore>,
    mut r: ResMut<VelloCollisionWorld>,
) {
    egui::Window::new("SoftBodyConfig").show(contexts.ctx_mut(), |ui| {
        ui.add(
            egui::Slider::new(&mut ui_state.soft_body_config.max_complexity, 0..=10)
                .text("max_complexity"),
        );
        ui.add(
            egui::Slider::new(&mut ui_state.soft_body_config.resititution, 0.01..=1.0)
                .text("resititution"),
        );
        ui.add(
            egui::Slider::new(
                &mut ui_state
                    .soft_body_config
                    .velocity_against_nromal_damping_threhold,
                1.0..=100.0,
            )
            .text("velocity_threhold"),
        );
        ui.add(
            egui::Slider::new(&mut ui_state.soft_body_config.total_inv_mass, 0.1..=10.0)
                .text("total_inv_mass"),
        );
        ui.add(
            egui::Slider::new(
                &mut ui_state.soft_body_config.inner_constraints_scaler,
                1e-3..=1e3,
            )
            .text("inner_c"),
        );
        ui.add(
            egui::Slider::new(
                &mut ui_state.soft_body_config.frame_constraints_scaler,
                1e-3..=1e3,
            )
            .text("frame_c"),
        );
        ui.add(egui::Slider::new(&mut ui_state.soft_body_config.substeps, 1..=10).text("substeps"));
        // ui.add(
        //     egui::Slider::new(
        //         &mut ui_state.soft_body_config.self_collision_complaince,
        //         0.0..=1.0,
        //     )
        //     .text("selfc_c"),
        // );
        // ui.add(
        //     egui::Slider::new(
        //         &mut ui_state.soft_body_config.self_collision_distance_threhold,
        //         0.1..=10.0,
        //     )
        //     .text("selfc_distance"),
        // );
        ui.add(
            egui::Slider::new(
                &mut ui_state.soft_body_config.shape_matching_damping,
                0.01..=1.0,
            )
            .text("sm_damping"),
        );
    });

    r.substeps = ui_state.soft_body_config.substeps;

    egui::Window::new("CollisionConstraintConfig").show(contexts.ctx_mut(), |ui| {
        ui.add(
            egui::Slider::new(
                &mut ui_state.collision_config.push_compliance_penetration_scaler,
                0.0001..=1.0,
            )
            .text("resititution"),
        );
        ui.add(
            egui::Slider::new(
                &mut ui_state.collision_config.friction_compliance_scaler,
                0.0001..=1.0,
            )
            .text("friction"),
        );
        ui.add(
            egui::Slider::new(
                &mut ui_state.collision_config.pull_compliance_scaler,
                0.0001..=1.0,
            )
            .text("anti_rotate"),
        );
    });

    egui::Window::new("General").show(contexts.ctx_mut(), |ui| {
        ui_state.just_spawn = false;
        ui_state.just_modified = false;
        ui_state.delete_all_dynamic = false;
        ui_state.preview_state_just_modified = false;
        // Get the FPS diagnostic path
        let fps_path = FrameTimeDiagnosticsPlugin::FPS;

        // Fetch the FPS value
        if let Some(fps) = diagnostics.get(&fps_path) {
            if let Some(value) = fps.value() {
                ui.label(format!("FPS: {:.1}", value));
            }
            if let Some(avg) = fps.average() {
                ui.label(format!("Avg FPS: {:.1}", avg));
            }
        }
        ui.add(egui::Slider::new(&mut ui_state.c_config.gravity_x, -100.0..=100.0).text("gra_x"));
        ui.add(egui::Slider::new(&mut ui_state.c_config.gravity_y, -100.0..=100.0).text("gra_y"));

        ui.add(egui::Slider::new(&mut ui_state.current.pos_x, -900.0..=900.0).text("pox_x"));
        ui.add(egui::Slider::new(&mut ui_state.current.pos_y, -501.0..=500.0).text("pox_y"));
        ui.add(egui::Slider::new(&mut ui_state.current.vec_x, -500.0..=500.0).text("vec_x"));
        ui.add(egui::Slider::new(&mut ui_state.current.vec_y, -500.0..=500.0).text("vec_y"));

        ui.add(egui::Slider::new(&mut ui_state.current.rotation, -360.0..=360.0).text("rotation"));
        ui.add(egui::Slider::new(&mut ui_state.current.scale, 0.01..=10.0).text("scale"));
        ui.add(
            egui::Slider::new(&mut ui_state.current.connection_complaince, 0.001..=1.0)
                .text("connection_compliance"),
        );
        let last_spawn_type = ui_state.current.collider_type;
        egui::ComboBox::from_label("Collider Type")
            .selected_text(format!("{:?}", ui_state.current.collider_type))
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Rect,
                    "Rect",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Circle,
                    "Circle",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Star,
                    "Star",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Heart,
                    "Heart",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Key,
                    "Key",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Shield,
                    "Shield",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Knife,
                    "Knife",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Capsule,
                    "Capsule",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::Ammo,
                    "Ammo",
                );
                ui.selectable_value(
                    &mut ui_state.current.collider_type,
                    ColliderType::PISTOL,
                    "Pistol",
                );
            });

        if last_spawn_type != ui_state.current.collider_type {
            ui_state.preview_state_just_modified = true;
        }
        ui.checkbox(&mut ui_state.spawn_particle_effect, "Spawn Particles");
        if ui.button("TogglePreview").clicked() {
            ui_state.preview_state = !ui_state.preview_state;
            ui_state.preview_state_just_modified = true;
        }

        if ui.button("StackSpawn").clicked() {
            ui_state.current.pos_y += 40.0;
            ui_state.just_spawn = true;
        }
        if ui.button("Spawn").clicked() {
            ui_state.just_spawn = true;
        }
        if ui.button("Nuke").clicked() {
            ui_state.delete_all_dynamic = true;
        }

        ui.add(egui::Slider::new(&mut ui_state.e_config.scale, 0.001..=1.0).text("impulse_scale"));
        egui::ComboBox::from_label("drag Type")
            .selected_text(format!("{:?}", ui_state.e_config.drag_type))
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut ui_state.e_config.drag_type,
                    ExteralEffectType::Selection,
                    "Selection",
                );
                ui.selectable_value(
                    &mut ui_state.e_config.drag_type,
                    ExteralEffectType::DragFrameAll,
                    "DragFrameAll",
                );
                ui.selectable_value(
                    &mut ui_state.e_config.drag_type,
                    ExteralEffectType::AddPin,
                    "AddPin",
                );
                ui.selectable_value(
                    &mut ui_state.e_config.drag_type,
                    ExteralEffectType::AddPinJoint,
                    "AddJoint",
                );
                ui.selectable_value(
                    &mut ui_state.e_config.drag_type,
                    ExteralEffectType::DragJoint,
                    "DragJoint",
                );
                ui.selectable_value(
                    &mut ui_state.e_config.drag_type,
                    ExteralEffectType::SnapJointToMouse,
                    "SnapJointToMouse",
                );
            });

        if ui.button("Quit").clicked() {
            std::process::exit(0);
        }
        if ui.button("Pause").clicked() {
            r.paused = !r.paused;
        }

        if ui_state.c_config.gravity_x != ui_state.c_config.pre_gravity_x
            || ui_state.c_config.gravity_y != ui_state.c_config.pre_gravity_y
        {
            ui_state.just_modified = true;
            ui_state.c_config.pre_gravity_x = ui_state.c_config.gravity_x;
            ui_state.c_config.pre_gravity_y = ui_state.c_config.gravity_y;
        }
    });
}

#[derive(Resource, Default)]
pub struct ColliderStatus {
    pub selected: Option<Entity>,
    pub secondary_selected: Option<Entity>,
    pub to_unselect: Vec<Entity>,
    pub applied_impulse: Vec2,
    pub need_update: bool,
}

impl ColliderStatus {
    pub fn select_entity(&mut self, entity: Entity) {
        if let Some(old) = self.secondary_selected.take() {
            if old != entity {
                self.to_unselect.push(old);
            }
        }
        self.secondary_selected = self.selected.take();
        self.selected = Some(entity);
        self.need_update = true;
    }

    pub fn select_nothing(&mut self) {
        if let Some(old) = self.secondary_selected.take() {
            self.to_unselect.push(old);
        }
        if let Some(old) = self.selected.take() {
            self.to_unselect.push(old);
        }
        self.need_update = true;
    }

    pub fn apply_impulse_to_selected_entity(&mut self, impulse: Vec2) {
        self.applied_impulse = impulse;
    }
}

pub fn drag_all_particles(particles: Vec<ParticleInfo>, data: FilterData) -> Vec<ExternalForce> {
    let impulse = bevy_to_vello(data.impulse) * 0.25;
    let mut result = vec![];
    for item in particles {
        result.push(ExternalForce::Impulse(item.index, impulse.x, impulse.y));
    }
    result
}

pub fn drag_particle_to_point(
    particles: Vec<ParticleInfo>,
    data: FilterData,
) -> Vec<ExternalForce> {
    let target = bevy_to_vello(data.impulse);
    let mut result = vec![];
    let item = particles[0];
    let delta = (target - Vec2::new(item.pos_x, item.pos_y));
    result.push(ExternalForce::Impulse(item.index, delta.x, delta.y));
    result
}

#[inline]
pub fn bevy_to_vello(point: Vec2) -> Vec2 {
    Vec2::new(point.x, -point.y)
}

pub fn update_collider_from_mouse(
    mut commands: Commands,
    mut query: Query<&mut VelloCollider>,
    mut status: ResMut<ColliderStatus>,
    mut force_on_frame_events: EventWriter<ColliderExternalImpulseEvent>,
    mouse_position: Res<MouseStatus>,
    mut connection_status: ResMut<ConnectionStatus>,
    ui_state: Res<UiState>,
) {
    let effect = ui_state.e_config.drag_type;
    // if effect == ExteralEffectType::SnapJointToMouse && mouse_position.pressed {
    //     if let Some(handle) = &connection_status.last_connection {
    //         force_on_joint_events.write(JointExternalForceEvent {
    //             filter: drag_particle_to_point,
    //             connection_index: *handle,
    //             filter_data: FilterData {
    //                 impulse: mouse_position.world_pos,
    //             },
    //         });
    //     }
    // }
    if status.applied_impulse != Vec2::ZERO {
        match effect {
            ExteralEffectType::DragFrameAll => {
                let impulse = status.applied_impulse * ui_state.e_config.scale;
                let vello_impulse = Vec2::new(impulse.x, -impulse.y);
                if let Some(entity) = &status.selected {
                    force_on_frame_events.write(ColliderExternalImpulseEvent {
                        entity: *entity,
                        impulse: [vello_impulse; 4],
                    });
                    status.applied_impulse = Vec2::ZERO
                }
            }
            ExteralEffectType::Selection => {
                status.applied_impulse = Vec2::ZERO;
            }
            _ => {} // ExteralEffectType::AddPin => {
                    //     let position = bevy_to_vello(mouse_position.world_pos);
                    //     let pos = Vector2::new(position.x, position.y);
                    //     if let Some(entity) = &status.selected {
                    //         let id = commands
                    //             .spawn(VelloJoint::new(
                    //                 ConnectionInitConfig::SinglePivot(
                    //                     VelloParticle {
                    //                         previous_pos: pos,
                    //                         pos,
                    //                         velocity: Vector2::new(0.0, 0.0),
                    //                         inv_mass: 1.0,
                    //                     },
                    //                     0.01,
                    //                 ),
                    //                 *entity,
                    //                 *entity,
                    //             ))
                    //             .id();
                    //         connection_status.last_connection = Some(id);
                    //     }
                    //     status.applied_impulse = Vec2::ZERO;
                    // }
                    // ExteralEffectType::AddPinJoint => {
                    //     let position = bevy_to_vello(mouse_position.world_pos);
                    //     let pos = Vector2::new(position.x, position.y);
                    //     if status.selected.is_some() && status.secondary_selected.is_some() {
                    //         let e1 = status.selected.clone().unwrap();
                    //         let e2 = status.secondary_selected.clone().unwrap();

                    //         if e1 != e2 {
                    //             let id = commands
                    //                 .spawn(VelloJoint::new(
                    //                     ConnectionInitConfig::SingleJoint(
                    //                         VelloParticle {
                    //                             previous_pos: pos,
                    //                             pos,
                    //                             velocity: Vector2::new(0.0, 0.0),
                    //                             inv_mass: 1.0,
                    //                         },
                    //                         0.01,
                    //                     ),
                    //                     e1,
                    //                     e2,
                    //                 ))
                    //                 .id();
                    //             connection_status.last_connection = Some(id);
                    //         } else {
                    //             info!("can not add joint to the same entity");
                    //         }
                    //     }
                    //     status.applied_impulse = Vec2::ZERO;
                    // }
                    // ExteralEffectType::DragJoint => {
                    //     if let Some(handle) = &connection_status.last_connection {
                    //         force_on_joint_events.write(JointExternalForceEvent {
                    //             filter: drag_all_particles,
                    //             connection_index: handle.clone(),
                    //             filter_data: FilterData {
                    //                 impulse: status.applied_impulse * ui_state.e_config.scale * 16.0,
                    //             },
                    //         });
                    //     }
                    //     status.applied_impulse = Vec2::ZERO;
                    // }
                    //ExteralEffectType::SnapJointToMouse => {}
        }
    }

    if status.need_update {
        for item in status.to_unselect.drain(..) {
            if let Ok(mut collider) = query.get_mut(item) {
                collider.is_selected = false;
            }
        }
        if let Some(entity) = &status.selected {
            if let Ok(mut collider) = query.get_mut(*entity) {
                collider.is_selected = true;
            }
        }
        status.need_update = false;
    }
}

#[derive(Resource, Default)]
pub struct MouseStatus {
    pub world_pos: Vec2,
    pub last_pos: Vec2,
    pub pressed: bool,
}

pub fn update_mouse_position(
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut p: ResMut<MouseStatus>,
) {
    let (camera, camera_transform) = camera_query.single().unwrap();
    if let Some(mouse_position) = windows
        .iter()
        .next()
        .and_then(|window| window.cursor_position())
    {
        if let Ok(world_pos) = camera.viewport_to_world_2d(camera_transform, mouse_position) {
            p.world_pos = world_pos;
        }
    }
}

pub fn update_mouse(
    mut commands: Commands,
    mut q_indicator: Query<(Entity, &mut VelloScene), With<DragIndicator>>,
    broad_phase: Res<VelloCollisionBroadPhase>,
    button: Res<ButtonInput<MouseButton>>,
    mut mouse_position: ResMut<MouseStatus>,
    mut collider_status: ResMut<ColliderStatus>,
    ui_state: Res<UiState>,
) {
    if button.just_pressed(MouseButton::Left) {
        if ui_state.e_config.drag_type == ExteralEffectType::Selection {
            if let Some(entity) = broad_phase.find_first_constains_point(mouse_position.world_pos) {
                collider_status.select_entity(entity);
            }
        }
        mouse_position.last_pos = mouse_position.world_pos;
        mouse_position.pressed = true;
    };
    if button.just_pressed(MouseButton::Right) {
        if ui_state.e_config.drag_type == ExteralEffectType::Selection {
            collider_status.select_nothing();
        }
    }
    if button.just_released(MouseButton::Left) {
        collider_status
            .apply_impulse_to_selected_entity(mouse_position.world_pos - mouse_position.last_pos);
        mouse_position.pressed = false;
        if let Ok((e, _)) = q_indicator.single() {
            commands.entity(e).despawn();
        }
    }
    if mouse_position.pressed {
        let mut temp = VelloScene::default();
        draw_drag_indicator(&mut temp, mouse_position.last_pos, mouse_position.world_pos);
        if let Ok((_, mut scene)) = q_indicator.single_mut() {
            *scene = temp;
        } else {
            commands.spawn((
                VelloSceneBundle {
                    scene: temp,
                    transform: Transform::from_translation(Vec3::new(0.0, 0.0, 100.0)),
                    ..Default::default()
                },
                DragIndicator,
            ));
        }
    }
}

#[derive(Clone, Component)]
pub struct DragIndicator;

pub fn draw_drag_indicator(s: &mut VelloScene, start: Vec2, end: Vec2) {
    let line = kurbo::Line::new((start.x, -start.y), (end.x, -end.y));
    s.stroke(
        &Stroke::new(8.0),
        Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(1.0, 0.0, 1.0, 0.9), 1.0),
        None,
        &line,
    );
}
