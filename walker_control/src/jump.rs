//! The **hand-rolled jump** — a scripted crouch→extend stroke.
//!
//! # Why this lives here and not in `game_lib`
//! `game_lib`'s loop-2 vertical resolver (`examples/game_lib/src/lower_body/
//! resolver/vertical.rs`) owns a jump FSM (`Stand→Load→Launch→Flight→Land`)
//! with the per-hop law `Δl = Δl_prev + k_p·(h_desired − h_apex)` — but it
//! drives the legacy `drive_hip_pin` stack, which `walker_qa` disables
//! (`SpineDriveEnabled(false)`). The walker_control stack actuates through a
//! different channel, so the stroke needs its own port here.
//!
//! # The actuation channel this exploits
//! [`crate::actuate`] consumes only the **shape**: the two hip→foot chord
//! offsets and the lean. `pose.hip` is a free anchor (no position constraint),
//! and a `stance: true` foot gets a stiff distance strut pulling hip↔foot to
//! the commanded chord length. So:
//!
//! - **Load** shortens the chord (crouch) against planted feet ⇒ the solver
//!   pulls the hip down. The crouch keeps the **rest x-offsets**: sweeping the
//!   feet inward while they are friction-planted drags them and kicks the
//!   torso off-center (measured: it spun the body a −95° backflip).
//! - **Launch** re-extends the chord to full rest fast ⇒ the strut pushes the
//!   hip up against the friction-planted feet. (This exact channel lifted the
//!   hip ~25 px in the PPO deploy — the physics is proven, only the clock is
//!   ours now.)
//! - **Flight** holds the extended pose. No tuck: the leg hip angular
//!   constraints are the torso's ONLY joints, so swinging the airborne legs
//!   (the first attempt swept them 45 px through an arc) reaction-rotates the
//!   torso — a pencil jump keeps the shape and therefore keeps the attitude.
//! - **Land** re-arms the stroke with the feet at the rest offsets, then (while
//!   the jump intent is held) **re-arms Load for the next hop** — bunny-
//!   hopping, matching loop-2's held-command semantics.
//!
//! The stroke is time-phased and deliberately **open-loop** (no height
//! feedback yet): the authored expert PPO will later be trained against. When
//! the measured hop is right, the same phases get a closed Δl law on top.

use glam::Vec2;

use crate::actuate::{FootPose, Pose};
use crate::control::{Command, ControlConfig, ControlOut, LocoMode, Step};
use crate::observe::Fiction;

/// Full-extension rest chord (hip→foot) offsets — the same constants
/// [`StandingCtrl`](crate::control::StandingCtrl) stands on (v10 blueprint,
/// spawn scale 0.5). Chord ≈ 111 px ≈ `thigh + calf`.
const REST_LEFT: Vec2 = Vec2::new(-33.0, 106.0);
const REST_RIGHT: Vec2 = Vec2::new(39.5, 103.0);

/// Crouch chord: depth 106 → 78 px with the **rest x-offsets kept**. The knee
/// folds (chord ≈ 85 < thigh+calf, the bones are rigid); the hip descends.
/// Gathering inward was measured to drag the planted feet and kick the torso
/// (see module docs) — a pure vertical stroke moves only the depth.
const CROUCH_LEFT: Vec2 = Vec2::new(-33.0, 78.0);
const CROUCH_RIGHT: Vec2 = Vec2::new(39.5, 78.0);

/// One stroke cycle. Time-phased; `Land` re-arms `Load` while the command is
/// held (bunny-hop) — the mode machine (`LocoState::decide`) ends the stroke
/// externally when the jump intent goes away.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JumpPhase {
    /// Chord rest → crouch, feet stance-pinned, hip descends.
    Load,
    /// Crouch → rest fast, feet pinned: the power stroke.
    Launch,
    /// Airborne: stance off (soft struts), shape held extended (pencil).
    Flight,
    /// Touchdown recovery: stance on, chord ramps back to rest.
    Land,
}

/// The scripted jump controller — the authored expert for the jump intent
/// (`Command::is_jump`). See the module docs for the stroke.
#[derive(Debug, Clone, Copy)]
pub struct JumpCtrl {
    phase: JumpPhase,
    /// Seconds inside the current phase.
    t: f32,
    /// Completed touchdown count (telemetry/tests).
    hops: u32,
    /// The hip target the stroke started from, and its measured velocity at
    /// that moment (see the ride-along in [`Step::step`]).
    anchor: Vec2,
    anchor_v: f32,
    /// True once the ride-along has been seeded (on the first Load entry).
    seeded: bool,
    // The active ramp: offsets are `lerp(*_from, *_to, smoothstep(t/dur))`.
    left_from: Vec2,
    left_to: Vec2,
    right_from: Vec2,
    right_to: Vec2,
    // Phase durations, seconds.
    load_ts: f32,
    launch_ts: f32,
    flight_ts: f32,
    land_ts: f32,
}

impl JumpCtrl {
    /// `lower` is unused for now (the chord constants are the v10 blueprint
    /// rest pose, like [`StandingCtrl`](crate::control::StandingCtrl)); it is
    /// taken for constructor symmetry and future reach-limited crouch depth.
    pub fn new(_lower: &walker_math::LowerBodyConfig) -> Self {
        Self {
            phase: JumpPhase::Load,
            t: 0.0,
            hops: 0,
            anchor: Vec2::ZERO,
            anchor_v: 0.0,
            seeded: false,
            left_from: REST_LEFT,
            left_to: CROUCH_LEFT,
            right_from: REST_RIGHT,
            right_to: CROUCH_RIGHT,
            load_ts: 0.22,
            launch_ts: 0.12,
            flight_ts: 0.45,
            land_ts: 0.25,
        }
    }

    /// Completed hops (for tests and later telemetry).
    pub fn hops(&self) -> u32 {
        self.hops
    }

    /// Ease the ramps: smoothstep on the normalized phase time.
    fn ramp_k(&self, dur: f32) -> f32 {
        if dur <= 0.0 {
            return 1.0;
        }
        let u = (self.t / dur).clamp(0.0, 1.0);
        u * u * (3.0 - 2.0 * u)
    }

    /// Begin a new ramp to `to_l`/`to_r` starting from the current offsets.
    fn begin(&mut self, phase: JumpPhase, cur_l: Vec2, cur_r: Vec2, to_l: Vec2, to_r: Vec2) {
        self.phase = phase;
        self.t = 0.0;
        self.left_from = cur_l;
        self.left_to = to_l;
        self.right_from = cur_r;
        self.right_to = to_r;
    }
}

impl Step for JumpCtrl {
    fn step(
        &mut self,
        _cmd: &Command,
        fiction: &Fiction,
        _cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        let dt = dt.max(0.0);
        self.t += dt;

        // Phase clock and the shape it commands.
        let dur = match self.phase {
            JumpPhase::Load => self.load_ts,
            JumpPhase::Launch => self.launch_ts,
            JumpPhase::Flight => self.flight_ts,
            JumpPhase::Land => self.land_ts,
        };
        let k = self.ramp_k(dur);
        let left = self.left_from.lerp(self.left_to, k);
        let right = self.right_from.lerp(self.right_to, k);
        // Stance (stiff strut) everywhere except the airborne phase.
        let stance = self.phase != JumpPhase::Flight;

        // Transitions: timers out the ground phases; Flight ends early on a
        // real touchdown (contact is measured, the ballistic arc is not).
        match self.phase {
            JumpPhase::Load if self.t >= dur => {
                self.begin(JumpPhase::Launch, left, right, REST_LEFT, REST_RIGHT);
            }
            JumpPhase::Launch if self.t >= dur => {
                // Pencil: hold the extended shape through the arc (see module
                // docs — swinging airborne legs reaction-rotates the torso).
                self.begin(JumpPhase::Flight, left, right, REST_LEFT, REST_RIGHT);
            }
            JumpPhase::Flight if self.t >= dur || fiction.feet.iter().any(|f| f.touching) => {
                self.begin(JumpPhase::Land, left, right, REST_LEFT, REST_RIGHT);
            }
            JumpPhase::Land if self.t >= dur && fiction.grounded => {
                // Held jump intent re-hops (bunny-hop semantics).
                self.hops += 1;
                self.begin(
                    JumpPhase::Load,
                    REST_LEFT,
                    REST_RIGHT,
                    CROUCH_LEFT,
                    CROUCH_RIGHT,
                );
            }
            _ => {}
        }

        // Anchor: the pose is translation-invariant in WORLD x — only the
        // chord shape actuates (see [`crate::actuate`]). But while a stance
        // foot is planted the strut drags the hip toward that foot's
        // commanded position, so a *stationary* hip target becomes a brake:
        // walking into a jump would dead-stop the 120 px/s gait mid-stroke
        // (measured on the first run-up attempt). The ground phases therefore
        // ride the target along at the velocity captured at stroke entry
        // (measured `fiction.v_hip.x`, constant through the arc); during the
        // airborne phase the pose floats — re-anchored to the measured hip,
        // which also re-syncs before the next stroke.
        let hip = if stance {
            if !self.seeded {
                self.anchor = fiction.hip;
                self.anchor_v = fiction.v_hip.x;
                self.seeded = true;
            } else {
                self.anchor.x += self.anchor_v * dt;
            }
            Vec2::new(self.anchor.x, fiction.hip.y)
        } else {
            self.seeded = false;
            fiction.hip
        };
        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: hip + left,
                    stance,
                },
                right_foot: FootPose {
                    pos: hip + right,
                    stance,
                },
                facing,
                // Upright throughout: the hand-rolled jump is a pure vertical
                // hop; any lean here is drift we would have to fight later.
                upper_angle: 0.0,
                stance_split: false,
            },
            // No horizontal intent: the hip_vel target is zero so telemetry
            // reads the stroke as in-place.
            hip_vel: Vec2::ZERO,
            jump_impulse: 0.0,
            plan: [Vec2::new(f32::NAN, f32::NAN); 2],
            mode: LocoMode::Jumping,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Jumping
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::FootState;

    /// Standing fiction: feet at the rest offsets under a hip at (0, −140),
    /// both touching.
    fn standing() -> Fiction {
        let hip = Vec2::new(0.0, -140.0);
        Fiction {
            dt: 1.0 / 90.0,
            hip,
            v_hip: Vec2::ZERO,
            lean: 0.0,
            lean_rate: 0.0,
            com: hip,
            v_com: Vec2::ZERO,
            ang_momentum: 0.0,
            omega_com: 0.0,
            feet: [
                FootState {
                    pos: hip + REST_LEFT,
                    vel: Vec2::ZERO,
                    touching: true,
                },
                FootState {
                    pos: hip + REST_RIGHT,
                    vel: Vec2::ZERO,
                    touching: true,
                },
            ],
            grounded: true,
        }
    }

    fn airborne() -> Fiction {
        let mut f = standing();
        f.feet[0].touching = false;
        f.feet[1].touching = false;
        f.grounded = false;
        f
    }

    fn run(
        cfg: &ControlConfig,
        ctrl: &mut JumpCtrl,
        fiction: &Fiction,
        ticks: usize,
    ) -> Vec<ControlOut> {
        let cmd = Command::jump_vertical(100.0);
        (0..ticks)
            .map(|_| ctrl.step(&cmd, fiction, cfg, 1.0 / 90.0, 1.0))
            .collect()
    }

    #[test]
    fn the_stroke_crouches_then_extends_then_tucks() {
        let cfg = ControlConfig::default();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        let depth = |o: &ControlOut| o.pose.left_foot.pos.y - o.pose.hip.y;

        // Load (grounded) shortens the chord depth toward the crouch. At
        // tick 20 (t = .222 ≥ load_ts) the stroke hands over to Launch.
        let outs = run(&cfg, &mut ctrl, &standing(), 20);
        assert!(depth(&outs[19]) < 100.0, "crouching during load");

        // Launch ramps back to the full rest chord; at its tick 11 it enters
        // Flight (launch_ts = .12 s).
        let outs = run(&cfg, &mut ctrl, &standing(), 11);
        assert!(
            (depth(&outs[10]) - 106.0).abs() < 2.0,
            "fully extended after the power stroke, got {}",
            depth(&outs[10])
        );

        // Flight (airborne fiction — ANY contact would land early): stance
        // dropped, shape held extended (pencil: no leg swing to react
        // against).
        let outs = run(&cfg, &mut ctrl, &airborne(), 26);
        let flight = &outs[25];
        assert!(!flight.pose.left_foot.stance, "flight is airborne");
        assert!(!flight.pose.right_foot.stance);
        assert!(depth(flight) > 100.0, "shape held extended in flight");
        assert!(
            (flight.pose.left_foot.pos.x - flight.pose.hip.x - REST_LEFT.x).abs() < 2.0,
            "flight keeps the rest x-offsets"
        );

        // Land (touchdown) brings stance back on.
        let outs = run(&cfg, &mut ctrl, &standing(), 10);
        assert!(outs[5].pose.left_foot.stance, "stance re-arms at land");
    }

    #[test]
    fn flight_ends_early_on_real_contact() {
        // The ballistic clock may not expire, but a touchdown does: land.
        let cfg = ControlConfig::default();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Drive Load + Launch grounded until Flight begins (tick 31), a few
        // ticks airborne, then the fiction reports contact again.
        run(&cfg, &mut ctrl, &standing(), 20);
        run(&cfg, &mut ctrl, &standing(), 11);
        assert_eq!(ctrl.phase, JumpPhase::Flight, "power stroke ended");
        run(&cfg, &mut ctrl, &airborne(), 2);
        assert_eq!(ctrl.phase, JumpPhase::Flight, "airborne: still flying");
        ctrl.step(
            &Command::jump_vertical(100.0),
            &standing(),
            &cfg,
            1.0 / 90.0,
            1.0,
        );
        assert_eq!(ctrl.phase, JumpPhase::Land, "contact ends the flight");
    }

    #[test]
    fn a_held_command_re_hops() {
        let cfg = ControlConfig::default();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // One full hop ≈ load .22 + launch .12 + short flight + land .25.
        run(&cfg, &mut ctrl, &standing(), 26); // grounded throughout
                                               // Let flight terminate on contact immediately (feet always touching).
        run(&cfg, &mut ctrl, &standing(), 40);
        assert!(ctrl.hops() >= 1, "the stroke re-armed after landing");
    }

    #[test]
    fn land_waits_for_the_ground() {
        // Commanded contact gone (airborne) ⇒ Land must not hand over;
        // stance-on in the air does no harm (strut target vs free feet).
        let cfg = ControlConfig::default();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Run to the end of flight without any contact.
        run(&cfg, &mut ctrl, &airborne(), 75);
        assert_eq!(
            ctrl.phase,
            JumpPhase::Land,
            "flight clock expired into Land (airborne)"
        );
        ctrl.step(
            &Command::jump_vertical(100.0),
            &airborne(),
            &cfg,
            1.0 / 90.0,
            1.0,
        );
        assert_eq!(ctrl.phase, JumpPhase::Land, "still no ground, no re-arm");
    }

    #[test]
    fn the_stroke_is_deterministic() {
        let cfg = ControlConfig::default();
        let mut a = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        let mut b = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        for _ in 0..120 {
            let oa = a.step(
                &Command::jump_vertical(100.0),
                &standing(),
                &cfg,
                1.0 / 90.0,
                1.0,
            );
            let ob = b.step(
                &Command::jump_vertical(100.0),
                &standing(),
                &cfg,
                1.0 / 90.0,
                1.0,
            );
            assert_eq!(oa.pose.left_foot.pos, ob.pose.left_foot.pos);
            assert_eq!(oa.pose.right_foot.pos, ob.pose.right_foot.pos);
        }
    }
}
