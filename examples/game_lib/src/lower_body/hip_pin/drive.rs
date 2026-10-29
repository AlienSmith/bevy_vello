use bevy::{prelude::*, time::Fixed};
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, VelloCharacterPhysicsRoot, VelloConstraintWorld, VelloParticle,
};
use leafwing_input_manager::prelude::ActionState;
use vello_physics::AngularConstraintConfig;

use super::ground::*;
use super::params::*;
use super::seeding::*;
use super::state::*;
use crate::character::{LeftLegController, RightLegController, SpineController};
use crate::input_management::actions::PlayerAction;
use crate::lower_body::resolver::{
    attitude_lean_pd, step_lower_body, BootPhase, CaptureMeasure, GaitAuthority, GaitPhase,
    ResolverState, VerticalPhase,
};

/// Step the resolver and queue the three stiff external position constraints
/// that pin `P1/P2/P3` to the commanded frame.
///
/// Run before `update_character_movement` (the legs read the targets) and
/// before the constraint world is stepped.
pub fn drive_hip_pin(
    time: Res<Time<Fixed>>,
    params: Res<HipPinParams>,
    ground_support: Option<Res<GroundSupport>>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<(&SpineController, &VelloCharacterPhysicsRoot)>,
    mut right_leg_q: Query<&mut RightLegController>,
    mut left_leg_q: Query<&mut LeftLegController>,
    p_q: Query<&VelloParticle>,
    action_q: Query<&ActionState<PlayerAction>>,
    world: ResMut<VelloConstraintWorld>,
    mut log_tick: Local<u64>,
) {
    let dt = time.delta_secs();

    for (entity, mut pin) in pin_q.iter_mut() {
        if !pin.enabled {
            continue;
        }
        let Ok((spine, root)) = spine_q.get(entity) else {
            continue;
        };
        try_seed(&mut pin, spine, root, params.hip_drop, &p_q);
        if !pin.ready {
            continue;
        }

        // Feedback: close the XPBD↔Raibert loop *scoped to the horizontal
        // channel*. `update_connection_particles` synced the solved physics
        // into `VelloParticle` at the end of the previous fixed step, so `P3`
        // (spine.particles[4], the hip anchor) holds the realized pose.
        //
        // Only the realized horizontal position is written back into the
        // resolver: foot placement must start from where physics actually left
        // the body. The vertical channel is left to the resolver's own
        // `tick_vertical` authority — its `regulate_height` spring already
        // closes that loop against the measured state, and re-injecting the
        // measured y (with its steady `g_eff·dt²` sag) would accumulate into
        // drift. The commanded hip *velocity* is likewise preserved, because
        // `delta_frame_targets` uses it as the servo reference
        // (`(v_hip − v_c)·dt`); overwriting it with the measured velocity would
        // cancel that command-vs-realized correction. `FEEDBACK=0` disables it
        // for A/B against the feed-forward baseline.
        if params.feedback {
            if let Ok(p3) = p_q.get(spine.particles[4]) {
                // Vello y-down → resolver y-up (flip y). Horizontal only.
                let measured_x = p3.particle.pos.x;
                let anchor_x = pin.anchor_p3.x;
                let world_dx = measured_x - anchor_x;
                pin.state.hip_pos.x = pin.seed_hip.x + world_dx;
            }
        }

        // Loop 1 balance feedback (plan §2): feed the measured whole-body CoM
        // into the resolver so `plan_target_x` uses the LIPM capture point
        // `x_foot = x_CoM + (v_CoM − v_des)/ω₀` instead of the Raibert neutral
        // point. `pin.measured` is in the Vello y-down frame: horizontal x is
        // unaffected by the flip, and `z_com` is already positive-up. The CoM x
        // is mapped into the resolver's seed-relative frame the same way the P3
        // feedback above maps the hip. When no valid measurement exists
        // (`n == 0`, e.g. the pure kinematic sandbox path) the resolver falls
        // back to its Raibert neutral point.
        if pin.measured.n > 0 {
            if let Ok(p3) = p_q.get(spine.particles[4]) {
                let com_offset_x = pin.measured.com.x - p3.particle.pos.x;
                pin.state.measured = Some(CaptureMeasure {
                    com_x: pin.state.hip_pos.x + com_offset_x,
                    v_com: pin.measured.v_com.x,
                    z_com: pin.measured.z_com,
                });
                // Loop 2 (plan §4): close the `Δl` integrator on the measured
                // whole-body CoM apex instead of the resolver's internal
                // ballistic model. `z_com` is the CoM height above the floor;
                // the resolver's `h_apex_prev` is an apex RISE above the
                // standing height, so feed `z_com − standing_hip_height`.
                pin.state.measured_apex = Some(pin.measured.z_com - pin.standing_height);
            }
        } else {
            pin.state.measured = None;
            pin.state.measured_apex = None;
        }

        // Game input → resolver input. The jump is edge-triggered here because
        // a held key would re-arm the hop on every landing.
        let mut input_dir = Vec2::ZERO;
        if let Ok(action) = action_q.get(entity) {
            if action.pressed(&PlayerAction::MoveRight) {
                input_dir.x += 1.0;
            }
            if action.pressed(&PlayerAction::MoveLeft) {
                input_dir.x -= 1.0;
            }
            let jump_held = action.pressed(&PlayerAction::MoveUp);
            if jump_held && !pin.jump_was_held {
                input_dir.y += 1.0;
            }
            pin.jump_was_held = jump_held;
            if action.pressed(&PlayerAction::MoveDown) {
                input_dir.y -= 1.0;
            }
        }

        // External pin policy: normal floor walking is angular-only. The pin is
        // allowed only behind the explicit debug switch, or as a mid-air
        // direction-change assist (airborne + horizontal input).
        let grounded = ground_support
            .as_deref()
            .map(|s| s.grounded)
            .unwrap_or(true);
        let ext_pin = use_external_pin(
            params.external_pin,
            params.air_control,
            grounded,
            input_dir.x,
        );
        pin.last_external_pin = ext_pin;

        // Plan §6: hand the resolver real per-foot ground truth. `None` when
        // the contact layer has never seen a foot collider (pure kinematic
        // sandbox, or a character without feet), which makes the gait clock fall
        // back to the pre-§6 timer exactly.
        //
        // The measured foot x is mapped into the resolver's seed-relative frame
        // exactly like the P3 feedback above (`seed_hip.x + (world − anchor)`),
        // so re-anchoring a stance plant is in the same units the foot targets
        // are planned in.
        // §6 authority is pushed every tick so the tuning UI can switch levels
        // without a respawn.
        pin.state.gait_authority = params.gait_authority;
        if let Some(support) = ground_support.as_deref() {
            if support.feet_valid {
                // Foot particles: `[P2, P3, knee, foot]` → index 3 is the foot.
                let foot_world_x = |leg_particles: Option<[Entity; 4]>| -> Option<f32> {
                    let e = leg_particles?[3];
                    p_q.get(e).ok().map(|p| p.particle.pos.x)
                };
                let left_parts = left_leg_q.get(entity).ok().map(|l| l.particles);
                let right_parts = right_leg_q.get(entity).ok().map(|r| r.particles);
                let anchor_x = pin.anchor_p3.x;
                let seed_x = pin.seed_hip.x;
                let to_resolver = |wx: Option<f32>| wx.map(|x| seed_x + (x - anchor_x));
                let lx = to_resolver(foot_world_x(left_parts));
                let rx = to_resolver(foot_world_x(right_parts));
                // Raw (`touching`), not debounced: the resolver compares
                // contact against phase boundaries, where one tick matters. The
                // hysteretic `grounded` flag stays in `GroundSupport` for the
                // thrust gate below.
                pin.state.foot_contact = Some([
                    crate::lower_body::resolver::LegContactInput {
                        touching: support.feet[0].touching,
                        foot_x: if support.feet[0].touching { lx } else { None },
                    },
                    crate::lower_body::resolver::LegContactInput {
                        touching: support.feet[1].touching,
                        foot_x: if support.feet[1].touching { rx } else { None },
                    },
                ]);
            } else {
                pin.state.foot_contact = None;
            }
        } else {
            pin.state.foot_contact = None;
        }

        // Per-pin config: the resolver's ground-referenced geometry is computed
        // against the realized standing height (latched from the live leg seed),
        // not the shared model default, so the virtual floor coincides with the
        // real collision floor. Everything downstream that reads `hip_height`
        // (reach envelope, LIPM, stand regulator, plant) sees this override.
        let mut cfg = params.config.clone();
        cfg.hip_height = pin.standing_height;
        let (pose, debug) = step_lower_body(input_dir, &cfg, &mut pin.state, dt);
        // Loop-3 telemetry latch: the torque/accel are per-tick `DebugInfo`
        // values (not persistent state), so keep the last tick's for the
        // headless harness's whole-body row.
        pin.last_attitude_torque = debug.attitude_torque;
        pin.last_hip_accel_x = debug.hip_accel_x;

        // Loop 3 → physics: the resolver's attitude is the desired torso lean.
        // Sign is consistent with no flip: resolver positive θ = forward lean in
        // its y-up travel frame (+x), `world_delta` maps resolver +x → Vello +x,
        // and an up-on-screen axis `(0, −L)` rotated by positive θ becomes
        // `(L·sinθ, −L·cosθ)` — also a lean toward +x.
        pin.theta = pin.state.attitude.theta;

        // Live P1/P2/P3 states, in the same order as `idx`.
        let idx = [2usize, 3, 4];
        let mut states = [(Vec2::ZERO, Vec2::ZERO); 3];
        let mut all_ready = true;
        for (k, i) in idx.iter().enumerate() {
            match p_q.get(spine.particles[*i]) {
                Ok(p) => states[k] = (p.particle.pos, p.particle.velocity),
                Err(_) => {
                    all_ready = false;
                    break;
                }
            }
        }
        if !all_ready {
            continue;
        }

        // Loop 3: rotate the whole [P1,P2,P3] rigid frame about P2. The
        // angular-joint path is *not* used here: with P2/P3 hard-pinned and the
        // rigid P1-P3 frame distance, changing only the P1_P2_P3 angle would
        // over-constrain the triangle (the solver can flip P1 through the
        // collinear singularity). Rotating all three by the same R(theta) about
        // P2 preserves every frame distance and makes the hip follow the torso.
        let p1_live = states[0].0;
        let p2_live = states[1].0;
        let p3_live = states[2].0;
        pin.last_meas_theta = measured_attitude(p1_live, p2_live);
        let theta = if params.attitude_enabled {
            clamp_attitude(
                pin.theta + params.attitude_target_bias,
                params.attitude_max_angle,
            )
        } else {
            0.0
        };
        pin.last_att_target = theta;
        if let Some((rest_cos, rest_sin)) = attitude_rest_basis(p1_live, p2_live, p3_live, theta) {
            pin.last_att_rest_cos = rest_cos;
            pin.last_att_rest_sin = rest_sin;
        }

        // Resolver hip (y-up) → Vello world (y-down). Only the delta from the
        // seed is used, so the resolver's absolute frame never leaks.
        let world_delta = Vec2::new(pose.hip.x - pin.seed_hip.x, -(pose.hip.y - pin.seed_hip.y));
        let hip_delta = world_delta - pin.prev_world_delta;
        pin.prev_world_delta = world_delta;
        let v_hip = Vec2::new(pin.state.hip_vel.x, -pin.state.hip_vel.y);

        let g_vello = world.gravity();

        // Vertical authority: supported, or a ballistic resolver phase (where
        // tracking the reference preserves the jump).
        let vertical_tracking = matches!(
            pin.state.vs.phase,
            VerticalPhase::Launch | VerticalPhase::Flight
        );
        let vertical_allowed = grounded || vertical_tracking;
        let support_share = if grounded { params.support_share } else { 0.0 };

        let targets = delta_frame_targets(
            states,
            hip_delta,
            v_hip,
            g_vello,
            dt,
            vertical_allowed,
            support_share,
        );

        // P2 carries the delta/velocity/gravity servo. P1/P3 are the rest
        // offsets from P2, rotated by theta; this rotates the whole rigid frame
        // about P2 and preserves all three frame distances.
        let p2_target = targets[1];
        let pose_targets = if params.attitude_enabled {
            p2_rigid_frame_targets(p2_target, pin.rest_offsets, theta)
        } else {
            // Legacy translation-only path: no attitude authority.
            targets
        };

        // All three frame particles are commanded to the rigid, P2-centered
        // pose. P3 remains hard-pinned, but to the attitude-adjusted target.
        for (k, i) in idx.iter().enumerate() {
            let (pos, vel) = states[k];
            let target = pose_targets[k];
            if *i == 4 {
                pin.last_ref_p3 = target;
                pin.last_sag_y = pos.y - target.y;
            }
            if !ext_pin {
                continue;
            }
            // While loop 3 is active the whole frame is a P2-centred attitude
            // pin, so the attitude compliance is the relevant stiffness.
            let base_compliance = if params.attitude_enabled {
                params.attitude_compliance
            } else {
                params.compliance
            };
            let _compliance = softened_compliance(
                base_compliance,
                params.soft_scale,
                params.v_ref,
                vel.length(),
            );

            // world.queue_character_one_time_external_position_constraint(
            //     &CharacterExternalPositionConstraintEvent {
            //         character_entity: entity,
            //         joint_entity: spine.particles[*i],
            //         config: ExternalPositionConstraintConfig {
            //             target,
            //             compliance,
            //             damping: params.damping,
            //         },
            //     },
            // );
        }

        // Opt-in telemetry (`PIN_LOG=1`).
        let pin_log = std::env::var("PIN_LOG")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);
        if pin_log {
            *log_tick += 1;
            if *log_tick % 90 == 0 {
                let p3 = p_q
                    .get(spine.particles[4])
                    .map(|p| p.particle.pos)
                    .unwrap_or(pin.last_ref_p3);
                info!(
                    "pin vertical={} grounded={} share={:.2} P3 actual=({:.1},{:.1}) ref=({:.1},{:.1}) sag_y={:.1}",
                    vertical_allowed,
                    grounded,
                    support_share,
                    p3.x,
                    p3.y,
                    pin.last_ref_p3.x,
                    pin.last_ref_p3.y,
                    pin.last_sag_y,
                );
            }
        }

        // Sandbox → legs: write the resolver poses into the leg controllers.
        // `update_character_movement` (ordered after) reads them for the IK.
        if params.hip_drives_legs {
            // Boot FSM (Stage 4d) — the physics-side launch authority. The body
            // above P3 is a straight stiff stick, and the ONLY actuator that
            // pitches it is the hip-joint rest angle (the world-upright lean
            // reference; see `leg_pd.rs` §loop-3 delivery). The two measured
            // failures of the scripted hip-translation lunge: (a) the hip target
            // follows the LIVE P3 x (`hip_t.x = actual_p3.x`), so an advancing
            // resolver reference never drags the particle — it only stretches
            // the drawn legs, whose PD then pulls the pinned feet and the body
            // drifts BACKWARD; (b) the measured-attitude rebalancer stayed live
            // and caught every tip, so the HOLD falsification run stayed
            // upright. The fix follows the user's directive literally:
            //   * Stand  = totally idle: no rebalancer, no lean PD — 0.0 lean
            //     aims the stick at true world vertical and nothing else, so
            //     whatever momentum the kick injects is unopposed;
            //   * Lunge  = DIRECT override of the angular constraint: an
            //     open-loop forward lean written straight into the hip-joint
            //     reference for the short window, bypassing the measured-attitude
            //     PD entirely. SIGN: `leg_pd.rs` documents the actuator as
            //     "leaning the reference backward makes the stance legs push the
            //     trunk forward" (the rebalancer corrects a forward lean with a
            //     NEGATIVE lean) — so a forward KICK uses `-dir * params.boot_lean`.
            //     (A +lean command measured `body_theta < 0`: trunk went backward.)
            // On release (`Walk`) the rebalancer resumes automatically because
            // these fields are rewritten from `params` every tick.
            let boot_locked = params.config.boot_lunge && pin.state.boot.is_locked();
            let boot_lunge_dir = match pin.state.boot {
                BootPhase::Lunge { dir, .. } if params.config.boot_lunge => Some(dir),
                _ => None,
            };
            let (Ok(mut right_leg), Ok(mut left_leg)) =
                (right_leg_q.get_mut(entity), left_leg_q.get_mut(entity))
            else {
                continue;
            };
            // Only the external pin owns P2/P3. In angular-only mode the leg
            // PD uses the full mass matrix and references the desired upright
            // spine, so it supports the torso instead of following it.
            right_leg.config.anchor_chain = ext_pin;
            left_leg.config.anchor_chain = ext_pin;
            right_leg.config.upright_spine_reference = !ext_pin;
            left_leg.config.upright_spine_reference = !ext_pin;

            // Loop 3 (plan §5, corrected delivery): the body above P3 is a
            // straight stiff stick, and the only external forces on it are
            // gravity, foot contact and friction — so its world verticality is
            // decided by the hip joints `P2–P3–P40/P41`, NOT by the waist
            // `P1–P2–P3`. Deliver the measured-attitude PD by tilting the
            // world-upright reference axis the leg PD aims the thigh at.
            //
            // Gating:
            //   * `attitude_enabled` — master switch;
            //   * `!ext_pin` — with the external pin active the pin itself owns
            //     the frame attitude (mid-air assist), so the leg axis must
            //     stay world-vertical;
            //   * `grounded` — in Flight there is no contact and therefore no
            //     leg authority; the stick must stay ballistic, so the
            //     reference is held straight instead of trimming;
            //   * `measured.n > 0` — no valid whole-body measurement yet
            //     (startup / detached parts) means no error signal to act on.
            let upright_lean = if let Some(ldir) = boot_lunge_dir {
                // Launch: the direct override. A/B-calibrated sign: `+lean`
                // advanced the hip (net +37.9 px), `−lean` toppled it backward
                // (hip collapsed 173→87 px) — so the forward kick is `+dir`.
                ldir * params.boot_lean
            } else if !boot_locked
                && params.attitude_enabled
                && !ext_pin
                && grounded
                && pin.measured.n > 0
            {
                attitude_lean_pd(
                    pin.measured.theta,
                    pin.measured.omega,
                    params.attitude_target_bias,
                    params.attitude_lean_kp,
                    params.attitude_lean_kd,
                    params.attitude_max_angle,
                )
            } else {
                0.0
            };
            right_leg.config.upright_reference_lean = upright_lean;
            left_leg.config.upright_reference_lean = upright_lean;

            right_leg.config.angular_compliance = params.leg_compliance.max(1e-12);
            left_leg.config.angular_compliance = params.leg_compliance.max(1e-12);
            right_leg.config.zeta = params.leg_zeta.max(0.0);
            left_leg.config.zeta = params.leg_zeta.max(0.0);
            right_leg.config.rest_chase_alpha = params.leg_alpha.clamp(0.0, 1.0);
            left_leg.config.rest_chase_alpha = params.leg_alpha.clamp(0.0, 1.0);
            right_leg.config.max_angle_rate = params.leg_max_angle_rate.max(0.0);
            left_leg.config.max_angle_rate = params.leg_max_angle_rate.max(0.0);

            let vello_to_bevy = |p: &VelloParticle| Vec2::new(p.particle.pos.x, -p.particle.pos.y);
            let live_pose = |particles: [Entity; 4],
                             hip_fb: Vec2,
                             knee_fb: Vec2,
                             foot_fb: Vec2|
             -> LegPoseSeed {
                let get =
                    |e: Entity, fallback: Vec2| p_q.get(e).map(&vello_to_bevy).unwrap_or(fallback);
                LegPoseSeed {
                    hip: get(particles[1], hip_fb),
                    knee: get(particles[2], knee_fb),
                    foot: get(particles[3], foot_fb),
                }
            };
            let live_left = live_pose(
                left_leg.particles,
                left_leg.hip_target,
                left_leg.knee_target,
                left_leg.target,
            );
            let live_right = live_pose(
                right_leg.particles,
                right_leg.hip_target,
                right_leg.knee_target,
                right_leg.target,
            );
            // Capture the seam seed and the realized standing height together,
            // then attach them. `leg_seed` and `standing_height` are separate
            // fields, so seeding one while writing the other must be done on a
            // local to satisfy the borrow checker.
            let seed = if let Some(seed) = pin.leg_seed.as_ref() {
                *seed
            } else {
                // Latch the realized standing height from the live leg the first
                // time the bridge is seeded. The character may stand taller than
                // the resolver's model `hip_height` (e.g. 106 px vs 90 px); the
                // ground-referenced resolver geometry is computed against this
                // per-pin value so the virtual floor lines up with the real one.
                pin.standing_height = (live_left.hip.y - live_left.foot.y).max(1.0);
                LegBridgeSeed {
                    live: [live_left, live_right],
                    resolver: [
                        LegPoseSeed {
                            hip: pose.hip,
                            knee: pose.left_knee,
                            foot: pose.left_foot,
                        },
                        LegPoseSeed {
                            hip: pose.hip,
                            knee: pose.right_knee,
                            foot: pose.right_foot,
                        },
                    ],
                }
            };
            pin.leg_seed = Some(seed);
            // Hip target: x follows the live P3 (so impulses move the body),
            // but y follows the resolver's vertical command relative to the
            // seed. Without the external pin, using live y here makes the foot
            // targets sink with the body and the leg PD then drives the feet
            // through the floor.
            let actual_p3_bevy = Vec2::new(states[2].0.x, -states[2].0.y);
            let seed_hip_y = 0.5 * (seed.live[0].hip.y + seed.live[1].hip.y);
            let desired_hip_y = seed_hip_y + (pose.hip.y - pin.seed_hip.y);
            let hip_t = Vec2::new(actual_p3_bevy.x, desired_hip_y) - Vec2::Y * params.hip_drop;
            let mut knee_l = hip_t
                + transfer_segment(
                    seed.live[0].knee - seed.live[0].hip,
                    seed.resolver[0].knee - seed.resolver[0].hip,
                    pose.left_knee - pose.hip,
                );
            let mut foot_l = knee_l
                + transfer_segment(
                    seed.live[0].foot - seed.live[0].knee,
                    seed.resolver[0].foot - seed.resolver[0].knee,
                    pose.left_foot - pose.left_knee,
                );
            let mut knee_r = hip_t
                + transfer_segment(
                    seed.live[1].knee - seed.live[1].hip,
                    seed.resolver[1].knee - seed.resolver[1].hip,
                    pose.right_knee - pose.hip,
                );
            let mut foot_r = knee_r
                + transfer_segment(
                    seed.live[1].foot - seed.live[1].knee,
                    seed.resolver[1].foot - seed.resolver[1].knee,
                    pose.right_foot - pose.right_knee,
                );
            // Safety clamp: never command a foot below its seed floor line.
            // (Bevy y-up, so the floor is the seed foot y.)
            foot_l.y = foot_l.y.max(seed.live[0].foot.y);
            foot_r.y = foot_r.y.max(seed.live[1].foot.y);

            // Pick the anatomical knee (facing right ⇒ the more anterior of the
            // IK's two mirror solutions). `transfer_segment` rotates the live seed
            // segment by the resolver's angle *change*, so a leg near full extension
            // (the `acos` singularity, reach ≈ L1+L2) lets the knee drift to
            // whichever side float noise lands on — the "sometimes bends left,
            // sometimes right" failure. The test uses only the x axis, identical in
            // the Bevy y-up frame here and the vello y-down resolver frame, so no
            // handedness flip is needed (the earlier cross-product winding rule was
            // wrong exactly because the two frames mirror the y axis).
            knee_l = crate::lower_body::select_anterior_knee(
                hip_t,
                foot_l,
                knee_l,
                params.config.facing,
            );
            knee_r = crate::lower_body::select_anterior_knee(
                hip_t,
                foot_r,
                knee_r,
                params.config.facing,
            );

            // Loop 2 delivery (plan §4): the resolver's virtual-leg thrust
            // command biases the leg-PD targets. In Bevy y-up, `stance_extend`
            // pushes the stance foot DOWN (extend the leg → ground reaction
            // lifts the CoM), `flight_retract` tucks the knee UP (swing
            // clearance), and `land_absorb` bends the knee DOWN (controlled
            // flexion to absorb the touchdown). Each is applied only to the leg
            // in the matching gait phase, so a swing leg is never extended into
            // the ground and a stance leg is never retracted.
            let thrust = debug.thrust;
            // Plan §6: stance authority requires REAL contact. A timer that
            // says Stance while the foot is airborne makes loop 2 extend a leg
            // against nothing — which is how a "stance" foot that has already
            // left the ground turns into slide instead of push. When the
            // contact layer has no foot data (pure kinematic sandbox), fall back
            // to the phase test so the offline animation still runs.
            //
            // Gated at `Contact` and above for the same reason the gait clock
            // is: this is the "is there ground under this leg" question, and
            // below `Contact` the resolver is not yet trusted with the answer.
            // It uses the DEBOUNCED flag, not raw contact, because losing thrust
            // authority for a single flicker tick is what makes the leg PD
            // chatter.
            let contact_authority = params.gait_authority >= GaitAuthority::Contact;
            let (left_stance_loaded, right_stance_loaded) = match ground_support.as_deref() {
                Some(s) if s.feet_valid && contact_authority => {
                    (s.feet[0].grounded, s.feet[1].grounded)
                }
                _ => (
                    debug.left_phase != GaitPhase::Swing,
                    debug.right_phase != GaitPhase::Swing,
                ),
            };
            if debug.left_phase == GaitPhase::Swing {
                knee_l.y = knee_l.y + thrust.flight_retract; // tuck knee up
            } else if left_stance_loaded {
                // stance with ground under it: extend/absorb as commanded
                foot_l.y = foot_l.y - thrust.stance_extend.max(thrust.land_absorb);
            }
            // else: the clock says Stance but the foot is not on the ground.
            // Grant no thrust authority — pushing against air only adds target
            // error for the leg PD to fight, and it is not producing ground
            // reaction anyway.
            if debug.right_phase == GaitPhase::Swing {
                knee_r.y = knee_r.y + thrust.flight_retract;
            } else if right_stance_loaded {
                foot_r.y = foot_r.y - thrust.stance_extend.max(thrust.land_absorb);
            }

            left_leg.hip_target = hip_t;
            right_leg.hip_target = hip_t;
            left_leg.knee_target = knee_l;
            right_leg.knee_target = knee_r;
            left_leg.target = foot_l;
            right_leg.target = foot_r;
            // Joint-rate feed-forward for the leg PD (bevy y-up, px/s).
            left_leg.hip_vel = pose.hip_vel;
            left_leg.knee_vel = pose.left_knee_vel;
            left_leg.target_vel = pose.left_foot_vel;
            right_leg.hip_vel = pose.hip_vel;
            right_leg.knee_vel = pose.right_knee_vel;
            right_leg.target_vel = pose.right_foot_vel;
        } else {
            // No pin ⇒ the frame is free; use the full mass matrix.
            if let Ok(mut right_leg) = right_leg_q.get_mut(entity) {
                right_leg.config.anchor_chain = false;
                right_leg.config.upright_spine_reference = false;
            }
            if let Ok(mut left_leg) = left_leg_q.get_mut(entity) {
                left_leg.config.anchor_chain = false;
                left_leg.config.upright_spine_reference = false;
            }
        }
    }
}

/// Posture hold for the `P1_P2_P3` waist joint: keep the spine **straight**.
///
/// This used to be the loop-3 actuator (the resolver's model `θ` was commanded
/// here). It is not: the body above `P3` is a straight, stiff stick whose
/// verticality is set by the hip joints `P2–P3–P40/P41`, and the measured
/// attitude is now delivered through the leg PD's upright reference axis (see
/// [`drive_hip_pin`]). What is left for this joint is posture — hold `P1`, `P2`
/// and `P3` collinear (`theta = 0.0`) so the waist never folds while the legs
/// do the balancing.
///
/// Runs after [`drive_hip_pin`] (which updates `pin.measured` and
/// `last_external_pin`) and before the constraint world is stepped.
pub fn tick_loop3_angular(
    params: Res<HipPinParams>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<&SpineController>,
    p_q: Query<&VelloParticle>,
    mut world: ResMut<VelloConstraintWorld>,
) {
    for (entity, mut pin) in pin_q.iter_mut() {
        // The external pin, when active, already owns the frame pose.
        if !pin.enabled || pin.last_external_pin || !params.attitude_enabled {
            continue;
        }
        let Ok(spine) = spine_q.get(entity) else {
            continue;
        };
        let (Ok(p1), Ok(p2), Ok(p3)) = (
            p_q.get(spine.particles[2]),
            p_q.get(spine.particles[3]),
            p_q.get(spine.particles[4]),
        ) else {
            continue;
        };
        let p1 = p1.particle.pos;
        let p2 = p2.particle.pos;
        let p3 = p3.particle.pos;
        pin.last_meas_theta = measured_attitude(p1, p2);
        // Straight stick: no model replay, no extra lean. The measured-attitude
        // correction is delivered by the legs, not here.
        let theta = 0.0;
        pin.last_att_target = theta;
        let Some((rest_cos, rest_sin)) = attitude_rest_basis(p1, p2, p3, theta) else {
            continue;
        };
        pin.last_att_rest_cos = rest_cos;
        pin.last_att_rest_sin = rest_sin;
        world.queue_character_angular_constraints(&CharacterAngularConstraintEvent {
            character_entity: entity,
            joint_entity: spine.angulars[2], // P1_P2_P3, pivot at P2
            config: AngularConstraintConfig {
                rest_cos,
                rest_sin,
                compliance: params.attitude_compliance.max(1e-12),
                zeta: params.attitude_zeta.max(0.0),
                angular_velocity: None,
                sigma: 1.0,
                // No external pin: all three frame particles are free to
                // receive the corrective torque.
                free_mask: 0b111,
                max_angle: None,
            },
        });
    }
}

/// Reset the pin to its resting stance and re-arm the seed (`R`).
pub fn reset_hip_pin(
    keys: Res<ButtonInput<KeyCode>>,
    params: Res<HipPinParams>,
    mut pin_q: Query<&mut HipPin>,
) {
    if !keys.just_pressed(KeyCode::KeyR) {
        return;
    }
    for mut pin in pin_q.iter_mut() {
        pin.state = ResolverState::new(&params.config);
        pin.ready = false;
        pin.leg_seed = None;
        // Re-seed will re-latch the realized height from the live leg.
        pin.standing_height = params.config.hip_height;
        pin.prev_world_delta = Vec2::ZERO;
    }
}
