//! Script dispatch: fold the current command + rig state into an [`Intent`], then
//! write it onto the right limb controller (or, for whole-body verbs, hand off to
//! [`crate::report::drive_body_keys`]).
//!
//! Also owns the one-time rig seeding ([`seed_anchors`]) and the env-gated A/B
//! levers for leg tracking bandwidth, applied once the rig is ready in body mode.

use bevy::app::AppExit;
use bevy::ecs::event::EventWriter;
use bevy::prelude::{error, info, Commands, Entity, Query, Res, ResMut, Vec2, Vec3};

use bevy_vello::integrations::physics::{VelloCharacterPhysicsRoot, VelloParticle};
use bevy_vello::prelude::VelloSceneBundle;

use leafwing_input_manager::prelude::ActionState;

use game_lib::character_asset::{BlueprintCharacterAssetManager, SvgCharacterAssetManager};
use game_lib::{
    default_input_map, CharacterController, CharacterRoot, HipPin, HipPinParams, IkMode,
    LeftArmController, LeftLegController, LimbConfig, ParticleTelemetryConfig, PlayerAction,
    PlayerMarker, ResetLimbControlConstraintsEvent, RightArmController, RightLegController,
};

use crate::command::{Command, LimbId};
use crate::config::{BP_ASSET, FIXED_HZ, SVG_ASSET};
use crate::math::{env_f32, env_flag, to_bevy};
use crate::probe::Probe;

/// The resolved intent of the current command — everything the dispatcher needs
/// to write, independent of which of the four controller components it lands on.
pub enum Intent {
    /// Drive `limb` to `target` in `mode` with `(kp, kd)`.
    Move {
        limb: LimbId,
        target: Vec2,
        mode: IkMode,
        kp: f32,
        kd: f32,
    },
    /// Persistently set gains only.
    Set {
        limb: LimbId,
        kp: Option<f32>,
        kd: Option<f32>,
    },
    /// Emit a constraint reset for `limb`.
    Reset { limb: LimbId },
    /// Do nothing this tick.
    Idle,
    /// Whole-body verb (`move`/`jump`): the hip pin is driven by
    /// [`crate::report::drive_body_keys`] from the playhead, not by a limb
    /// controller here.
    Body,
}

/// Fold the current command + the probe's rig state into an [`Intent`].
pub fn resolve_intent(cmd: &Command, probe: &Probe) -> Intent {
    // Per-command gains override the cell/env default only in single-cell mode;
    // in sweep mode the cell's gains always win (that IS the experiment axis).
    let gains = |kp: Option<f32>, kd: Option<f32>| -> (f32, f32) {
        if probe.sweep {
            (probe.kp, probe.kd)
        } else {
            (kp.unwrap_or(probe.kp), kd.unwrap_or(probe.kd))
        }
    };

    match cmd {
        Command::Reach(a) => {
            let idx = a.limb.idx();
            let anchor = probe.anchors[idx];
            let len = probe.limb_len[idx].max(1.0);
            let target = match a.to {
                Some([x, y]) => Vec2::new(x, y),
                None => {
                    let dir = a
                        .dir
                        .map(|[x, y]| Vec2::new(x, y))
                        .unwrap_or(Vec2::NEG_Y)
                        .normalize_or(Vec2::NEG_Y);
                    anchor + dir * a.dist.unwrap_or(len)
                }
            };
            let (kp, kd) = gains(a.kp, a.kd);
            Intent::Move {
                limb: a.limb,
                target,
                mode: IkMode::Reach,
                kp,
                kd,
            }
        }
        Command::Aim(a) => {
            let idx = a.limb.idx();
            let anchor = probe.anchors[idx];
            let len = probe.limb_len[idx].max(1.0);
            // Auto distance: 3× the end bone (matches the arm harness heuristic),
            // unless `TARGET_DIST` or the command's `dist` says otherwise.
            let auto = 3.0 * len * 0.5;
            let dist = if probe.target_dist > 0.0 {
                probe.target_dist
            } else {
                a.dist.unwrap_or(auto.max(40.0))
            };
            let dir = Vec2::new(a.dir[0], a.dir[1]).normalize_or(Vec2::X);
            let target = anchor + dir * dist;
            let (kp, kd) = gains(a.kp, a.kd);
            Intent::Move {
                limb: a.limb,
                target,
                mode: IkMode::Aim {
                    weapon_offset_y: a.weapon_offset_y.unwrap_or(0.0),
                },
                kp,
                kd,
            }
        }
        Command::Set(a) => {
            let kp = if probe.sweep { Some(probe.kp) } else { a.kp };
            let kd = if probe.sweep { Some(probe.kd) } else { a.kd };
            Intent::Set {
                limb: a.limb,
                kp,
                kd,
            }
        }
        Command::Reset(a) => Intent::Reset { limb: a.limb },
        Command::Wait(_) => Intent::Idle,
        Command::Move(_) | Command::Jump(_) => Intent::Body,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn drive_script(
    mut commands: Commands,
    mut probe: ResMut<Probe>,
    character_svg: Res<SvgCharacterAssetManager>,
    character_blueprint: Res<BlueprintCharacterAssetManager>,
    mut right_arm_q: Query<&mut RightArmController>,
    mut left_arm_q: Query<&mut LeftArmController>,
    mut right_leg_q: Query<&mut RightLegController>,
    mut left_leg_q: Query<&mut LeftLegController>,
    root_q: Query<&VelloCharacterPhysicsRoot>,
    particle_q: Query<&VelloParticle>,
    mut reset_ev: EventWriter<ResetLimbControlConstraintsEvent>,
    mut exit: EventWriter<AppExit>,
    pin_params: Option<Res<HipPinParams>>,
) {
    probe.tick += 1;
    let tick = probe.tick;

    // ---- Spawn V10 once both assets are parsed. ----
    if probe.character.is_none() {
        if character_svg.all_loaded() && character_blueprint.all_loaded() {
            let mut ent = commands.spawn((
                VelloSceneBundle {
                    transform: bevy::prelude::Transform {
                        translation: Vec3::new(0.0, env_f32("DROP_Y", 0.0), 100.0),
                        scale: Vec3::new(0.5, 0.5, 1.0),
                        ..Default::default()
                    },
                    ..Default::default()
                },
                CharacterRoot {
                    svg_asset_id: SVG_ASSET.to_owned(),
                    blueprint_asset_id: BP_ASSET.to_owned(),
                    collision_group: 1,
                },
                CharacterController::default(),
            ));
            if probe.body {
                // The pin needs the real input path: `PlayerMarker` + the default
                // action map + the live `ActionState` the pin reads, plus the
                // `HipPin` itself (seeded lazily on the first ready tick). The
                // resolver config comes from the env-built `HipPinParams`
                // resource (TARGET_SPEED/LEGS/...), so the spawn and the
                // resource can't drift apart.
                let hip_cfg = match &pin_params {
                    Some(p) => p.config.clone(),
                    None => HipPinParams::default().config,
                };
                ent.insert((
                    PlayerMarker,
                    default_input_map(),
                    ActionState::<PlayerAction>::default(),
                    HipPin::new(&hip_cfg),
                ));
            }
            let entity = ent.id();
            probe.character = Some(entity);
            probe.spawn_tick = Some(tick);
            info!("V10 spawned, harness clock starts (body={})", probe.body);
        }
        return;
    }

    let character = probe.character.unwrap();

    // ---- Bail out if the rig never comes up (never silently hang). ----
    if !probe.ready && !probe.ever_ready {
        if let Some(st) = probe.spawn_tick {
            if tick.saturating_sub(st) > (FIXED_HZ as u64) * 10 {
                error!("rig never became ready after 10 sim-seconds; aborting");
                probe.done = true;
                exit.write(AppExit::error());
            }
        }
    }
    if probe.done {
        return;
    }

    // ---- Wait for the physics root (same gate as `update_character_movement`). ----
    let Ok(root) = root_q.get(character) else {
        return;
    };
    if root.initial_frame_coordinates.is_none() {
        return;
    }

    // ---- Capture the rig once: every limb's pivot anchor + characteristic length. ----
    if !probe.ready {
        // The focus limb must exist; otherwise there is nothing to measure.
        let focus_ok = match probe.focus {
            LimbId::RightArm => right_arm_q.get(character).is_ok(),
            LimbId::LeftArm => left_arm_q.get(character).is_ok(),
            LimbId::RightLeg => right_leg_q.get(character).is_ok(),
            LimbId::LeftLeg => left_leg_q.get(character).is_ok(),
        };
        if !focus_ok {
            return;
        }

        probe.anchored = seed_anchors(
            &mut probe,
            character,
            &right_arm_q,
            &left_arm_q,
            &right_leg_q,
            &left_leg_q,
            &particle_q,
        );
        if !probe.anchored {
            return;
        }

        // ---- A/B levers for the leg-controller tracking bandwidth (env-gated,
        // applied once at ready; body mode never calls `apply_to_rleg`/`apply_to_lleg`
        // so the legs otherwise hold `LimbConfig::default()`). Both target the
        // leg-chain divergence now that the hip frame is proven tight (~0.7px).
        //   `LEG_NO_CHASE=1`   command the resolver's rest angle directly — no
        //                      first-order chase (`rest_chase_alpha=1.0`) and no
        //                      `max_angle_rate` clamp (huge) — the XPBD ζ-damped
        //                      spring is the only smoothing left.
        //   `LEG_NO_SCHEDULE=1` disable the ω-softening compliance schedule
        //                      (`soft_scale=1.0`) so stiffness holds during swing.
        //   `LEG_ZETA=<ζ>`     override the solver damping ratio (default 1.0 —
        //                      proven over-damping in the arm ζ-sweep; the large
        //                      `gamma` starves the position correction). ζ=0.2 was
        //                      the arm sweet spot.
        //   `LEG_SIGMA=<σ>`    override the velocity-projection strength
        //                      (default 1.0 = full projection). σ=0 disables the
        //                      projection entirely — recovers the pre-projection
        //                      positional-solve-only behaviour (37px floor), σ<1
        //                      weakens it (soft velocity constraint).
        //   `LEG_COMPLIANCE=<k>` override the angular stiffness (default
        //                      5e-8 at 30px ref length). Smaller = stiffer, larger
        //                      = softer; this is the Kp of the XPBD angular
        //                      spring (ζ damping scales with it automatically).
        // Baseline = none set (default chase + schedule + ζ=1.0 + σ=1.0 + k=5e-8).
        //   `LEG_C_DOT_SCALE=<s>` override the `angular_velocity` gain
        //                      (default 1.0). The game layer now emits rad/s
        //                      (substep-count-independent) and the solver
        //                      converts internally; this is a pure authority
        //                      knob on the commanded joint rate. Root-cause probe.
        if probe.body
            && (env_flag("LEG_NO_CHASE")
                || env_flag("LEG_NO_SCHEDULE")
                || std::env::var("LEG_ZETA")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty())
                || std::env::var("LEG_SIGMA")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty())
                || std::env::var("LEG_COMPLIANCE")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty())
                || std::env::var("LEG_C_DOT_SCALE")
                    .ok()
                    .is_some_and(|s| !s.trim().is_empty()))
        {
            let no_chase = env_flag("LEG_NO_CHASE");
            let no_schedule = env_flag("LEG_NO_SCHEDULE");
            let zeta_opt = std::env::var("LEG_ZETA")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let sigma_opt = std::env::var("LEG_SIGMA")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let compliance_opt = std::env::var("LEG_COMPLIANCE")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let c_dot_scale_opt = std::env::var("LEG_C_DOT_SCALE")
                .ok()
                .and_then(|s| s.trim().parse::<f32>().ok());
            let apply = |cfg: &mut LimbConfig| {
                if no_chase {
                    cfg.rest_chase_alpha = 1.0;
                    cfg.max_angle_rate = 1e6;
                }
                if no_schedule {
                    cfg.soft_scale = 1.0;
                }
                if let Some(z) = zeta_opt {
                    cfg.zeta = z;
                }
                if let Some(s) = sigma_opt {
                    cfg.sigma = s;
                }
                if let Some(k) = compliance_opt {
                    cfg.angular_compliance = k;
                }
                if let Some(s) = c_dot_scale_opt {
                    cfg.c_dot_scale = s;
                }
            };
            if let Ok(mut c) = right_leg_q.get_mut(character) {
                apply(&mut c.config);
            }
            if let Ok(mut c) = left_leg_q.get_mut(character) {
                apply(&mut c.config);
            }
            info!(
                "body A/B leg levers: no_chase={} no_schedule={} zeta={:?} sigma={:?} compliance={:?} c_dot_scale={:?}",
                no_chase, no_schedule, zeta_opt, sigma_opt, compliance_opt, c_dot_scale_opt
            );
        }

        let first_ready = !probe.ever_ready;
        probe.ticks_ready = tick;
        probe.ready = true;
        probe.ever_ready = true;
        probe.action = 0;
        probe.action_start_tick = tick;
        probe.applied_action = None;
        probe.script_done = false;
        probe.samples.clear();
        if first_ready {
            commands.insert_resource(ParticleTelemetryConfig::v10(character));
        }
        info!(
            "cell {} (kp={:.3}, kd={:.3}): rig ready, focus={} anchor={:?}",
            probe.cell,
            probe.kp,
            probe.kd,
            probe.focus.label(),
            probe.anchors[probe.focus.idx()],
        );
    }

    if probe.script.is_empty() {
        error!("empty script; aborting");
        probe.done = true;
        exit.write(AppExit::error());
        return;
    }

    // ---- Playhead: one entry per command, advanced when its `dur` elapses. ----
    let cur = probe.action.min(probe.script.len() - 1);
    let cur_dur = probe.script[cur].dur();
    let elapsed = (tick.saturating_sub(probe.action_start_tick)) as f32 / FIXED_HZ as f32;

    // Apply the current command (idempotent for continuous verbs; once for
    // one-shot verbs via `applied_action`).
    let cmd = probe.script[cur].clone();
    let first_touch = probe.applied_action != Some(cur);
    let intent = resolve_intent(&cmd, &probe);
    match intent {
        Intent::Move {
            limb,
            target,
            mode,
            kp,
            kd,
        } => match limb {
            LimbId::RightArm => apply_to_arm(&mut right_arm_q, character, target, mode, kp, kd),
            LimbId::LeftArm => apply_to_larm(&mut left_arm_q, character, target, mode, kp, kd),
            LimbId::RightLeg => apply_to_rleg(&mut right_leg_q, character, target, mode, kp, kd),
            LimbId::LeftLeg => apply_to_lleg(&mut left_leg_q, character, target, mode, kp, kd),
        },
        Intent::Set { limb, kp, kd } => {
            if first_touch {
                apply_set(
                    &mut right_arm_q,
                    &mut left_arm_q,
                    &mut right_leg_q,
                    &mut left_leg_q,
                    character,
                    limb,
                    kp,
                    kd,
                );
            }
        }
        Intent::Reset { limb } => {
            if first_touch {
                reset_ev.write(ResetLimbControlConstraintsEvent {
                    limb: limb.which(),
                    character,
                });
            }
        }
        // `wait` (Idle) does nothing; `move`/`jump` (Body) are driven by the
        // playhead in `drive_body_keys`, so no limb write lands here.
        Intent::Idle | Intent::Body => {}
    }
    if first_touch {
        probe.applied_action = Some(cur);
    }

    // Advance the playhead (after applying, so a `dur == 0` verb still lands once).
    if elapsed >= cur_dur {
        if cur + 1 < probe.script.len() {
            probe.action = cur + 1;
            probe.action_start_tick = tick;
            probe.applied_action = None;
        } else {
            probe.script_done = true;
            // The telemetry system owns finalization (so it can grab the last
            // sample); the drive simply holds the last target and waits.
        }
    }
}

/// Seed each limb's pivot anchor (world Bevy y-up) + characteristic length,
/// resolved from the controller components. Returns false if a limb's particles
/// are not yet resolvable (retry next tick). Missing limbs keep zeroed defaults.
#[allow(clippy::too_many_arguments)]
fn seed_anchors(
    probe: &mut Probe,
    character: Entity,
    right_arm_q: &Query<&mut RightArmController>,
    left_arm_q: &Query<&mut LeftArmController>,
    right_leg_q: &Query<&mut RightLegController>,
    left_leg_q: &Query<&mut LeftLegController>,
    particle_q: &Query<&VelloParticle>,
) -> bool {
    let mut ok = true;
    let mut seed = |idx: usize, parts: &[Entity; 4]| {
        let (Ok(p1), Ok(p2), Ok(p3)) = (
            particle_q.get(parts[1]),
            particle_q.get(parts[2]),
            particle_q.get(parts[3]),
        ) else {
            ok = false;
            return;
        };
        let pivot = to_bevy(p1.particle.pos);
        let mid = to_bevy(p2.particle.pos);
        let end = to_bevy(p3.particle.pos);
        probe.anchors[idx] = pivot;
        probe.limb_len[idx] = (mid - pivot).length() + (end - mid).length();
    };
    if let Ok(c) = right_arm_q.get(character) {
        seed(0, &c.particles);
    }
    if let Ok(c) = left_arm_q.get(character) {
        seed(1, &c.particles);
    }
    if let Ok(c) = right_leg_q.get(character) {
        seed(2, &c.particles);
    }
    if let Ok(c) = left_leg_q.get(character) {
        seed(3, &c.particles);
    }
    ok
}

#[inline]
fn apply_to_arm(
    q: &mut Query<&mut RightArmController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[inline]
fn apply_to_larm(
    q: &mut Query<&mut LeftArmController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[inline]
fn apply_to_rleg(
    q: &mut Query<&mut RightLegController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[inline]
fn apply_to_lleg(
    q: &mut Query<&mut LeftLegController>,
    character: Entity,
    target: Vec2,
    mode: IkMode,
    kp: f32,
    kd: f32,
) {
    if let Ok(mut c) = q.get_mut(character) {
        c.target = target;
        c.config.ik_mode = mode;
        c.config.rest_chase_alpha = kp;
        c.config.zeta = kd;
    }
}

#[allow(clippy::too_many_arguments)]
fn apply_set(
    right_arm_q: &mut Query<&mut RightArmController>,
    left_arm_q: &mut Query<&mut LeftArmController>,
    right_leg_q: &mut Query<&mut RightLegController>,
    left_leg_q: &mut Query<&mut LeftLegController>,
    character: Entity,
    limb: LimbId,
    kp: Option<f32>,
    kd: Option<f32>,
) {
    let tweak = |config: &mut LimbConfig| {
        if let Some(kp) = kp {
            config.rest_chase_alpha = kp;
        }
        if let Some(kd) = kd {
            config.zeta = kd;
        }
    };
    match limb {
        LimbId::RightArm => {
            if let Ok(mut c) = right_arm_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
        LimbId::LeftArm => {
            if let Ok(mut c) = left_arm_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
        LimbId::RightLeg => {
            if let Ok(mut c) = right_leg_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
        LimbId::LeftLeg => {
            if let Ok(mut c) = left_leg_q.get_mut(character) {
                tweak(&mut c.config);
            }
        }
    }
}
