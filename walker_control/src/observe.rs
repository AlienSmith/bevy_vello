//! The **observe** module — physics reality → [`Fiction`] (plan §1: `Observe`).
//!
//! One snapshot per fixed tick. The fiction is *measured input*, never a
//! parallel story (plan Law 2): every field is read from the physics world,
//! filtered for noise, and handed to the controller. The controller may not
//! measure anything itself; the observer may not decide anything.
//!
//! Derivation was audited against every measured-physics consumer in
//! [`walker_math::step_lower_body`](../../walker_math/src/walk.rs):
//!
//! | consumer | fiction field |
//! |---|---|
//! | `CaptureMeasure { com_x, v_com, z_com }` | `com`, `v_com` (height from config floor) |
//! | `RealityVertical { grounded, h_meas, vy_up }` | `grounded`, `hip`, `v_hip` |
//! | `foot_contact[i] { touching, foot_x }` | `feet[i].touching`, `feet[i].pos.x` |
//! | loop-3 PD `(theta_meas, omega_meas)` | `lean`, `lean_rate` |
//! | hip fiction stamp | `hip`, `v_hip` |
//!
//! Deliberately **not** here: swing/stance durations (config), move/jump
//! intent (the `Command` input), waist position (the controller needs the
//! *derived* attitude, not the raw shoulder), contact normals (the old thrust
//! gate retired with the pin architecture).
//!
//! # Frame
//! Vello **y-down** — the raw physics frame, like [`crate::actuate`]. The
//! controller's stamp maps signs/origin when feeding `walker_math`. Attitude
//! convention follows loop 3: the stick axis is `hip(P3) → waist(P2)`;
//! upright is `(0, -1)` ⇒ 0 rad, `+` = forward lean toward `+x`.
//!
//! Ported from [`measure.rs`](../../examples/game_lib/src/lower_body/measure.rs)
//! (CoM mass-weighting, EMA, torso pitch) and
//! [`ground.rs`](../../examples/game_lib/src/lower_body/hip_pin/ground.rs)
//! (foot contact debounce), with the pin-era extras stripped.

use glam::Vec2;

// ---------------------------------------------------------------------------
// The fiction: what observe submits to control
// ---------------------------------------------------------------------------

/// One per-tick snapshot of physics status. Pure input data (plan §1).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fiction {
    /// Fixed-step duration this snapshot was taken over, seconds.
    pub dt: f32,
    /// Measured hip (`P3`) position, Vello y-down.
    pub hip: Vec2,
    /// Measured hip velocity, EMA-filtered ([`ObserveConfig::tau`]).
    pub v_hip: Vec2,
    /// Measured stick attitude `θ` (hip→waist axis vs vertical, `+` = lean `+x`).
    pub lean: f32,
    /// EMA-filtered `dθ/dt`, rad/s. The attitude PD needs the rate, not just θ.
    pub lean_rate: f32,
    /// Mass-weighted whole-body CoM position (falls back to `hip` for an empty
    /// sample set, per the §7 P3-heavy budget where CoM ≈ hip anyway).
    pub com: Vec2,
    /// EMA-filtered mass-weighted CoM velocity.
    pub v_com: Vec2,
    /// Measured foot state `[left, right]`. Velocities are unfiltered (the
    /// math does not consume them; slip telemetry and gates do).
    pub feet: [FootState; 2],
    /// Debounced ground truth: any foot touching this tick or continuously
    /// grounded. NOT the gait clock — the measured failure mode it replaces:
    /// the clock claimed stance while both feet floated 48.9 % of ticks
    /// (`plans/vertical-stabilizer.md` §0).
    pub grounded: bool,
}

impl Default for Fiction {
    fn default() -> Self {
        Self {
            dt: 0.0,
            hip: Vec2::ZERO,
            v_hip: Vec2::ZERO,
            lean: 0.0,
            lean_rate: 0.0,
            com: Vec2::ZERO,
            v_com: Vec2::ZERO,
            feet: [FootState::default(); 2],
            grounded: false,
        }
    }
}

/// One measured foot: position, velocity, raw 1-tick contact flag.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FootState {
    pub pos: Vec2,
    pub vel: Vec2,
    /// Raw contact this tick (no hysteresis — the gait clock compares contact
    /// against phase boundaries, where one tick matters; the hysteretic flag
    /// is the body-level `grounded`).
    pub touching: bool,
}

// ---------------------------------------------------------------------------
// Pure sampling input (built by the Bevy adapter, or by hand in tests)
// ---------------------------------------------------------------------------

/// One body particle for the CoM average. `inv_mass` follows the XPBD
/// convention (`m_i = 1 / inv_mass`); `inv_mass <= 0` ⇒ infinite mass, skipped.
#[derive(Debug, Clone, Copy, Default)]
pub struct ParticleSample {
    pub pos: Vec2,
    pub vel: Vec2,
    pub inv_mass: f32,
}

/// The measured hip particle plus the waist reference (`P2`) that defines the
/// stick axis. Only the *derived* attitude reaches the controller.
#[derive(Debug, Clone, Copy)]
pub struct HipSample {
    pub pos: Vec2,
    pub vel: Vec2,
    pub waist: Vec2,
}

/// One measured foot with its raw contact flag.
#[derive(Debug, Clone, Copy)]
pub struct FootSample {
    pub pos: Vec2,
    pub vel: Vec2,
    pub touching: bool,
}

/// One full tick's raw measurements (borrowed body slice — the owned variant
/// is [`OwnedRealitySample`], built by the adapter).
#[derive(Debug, Clone)]
pub struct RealitySample<'a> {
    pub dt: f32,
    pub hip: HipSample,
    pub feet: [FootSample; 2],
    /// All character particles (arms included), for the mass-weighted CoM.
    pub body: &'a [ParticleSample],
}

/// Owned [`RealitySample`]: the adapter builds this from the ECS, then
/// [`Observer::observe`] borrows it for one tick.
#[derive(Debug, Clone)]
pub struct OwnedRealitySample {
    pub dt: f32,
    pub hip: HipSample,
    pub feet: [FootSample; 2],
    pub body: Vec<ParticleSample>,
}

impl OwnedRealitySample {
    /// Borrow as the slice-based sample for one [`Observer::observe`] call.
    pub fn as_sample(&self) -> RealitySample<'_> {
        RealitySample {
            dt: self.dt,
            hip: self.hip,
            feet: self.feet,
            body: &self.body,
        }
    }
}

// ---------------------------------------------------------------------------
// Filter + debounce state
// ---------------------------------------------------------------------------

/// Observer tuning. The EMA time constant applies to `v_hip`, `v_com`, and
/// `lean_rate` together (one knob, matching the committed `MEASURE_TAU`).
#[derive(Debug, Clone, Copy)]
pub struct ObserveConfig {
    /// EMA time constant, seconds. Default `0.03` (~3 ticks at 90 Hz): raw
    /// `v_com` carries the ±140 px/s gait-cycle ripple, which rang every
    /// consumer when fed directly (measured, 2026-10-29).
    pub tau: f32,
}

impl Default for ObserveConfig {
    fn default() -> Self {
        Self { tau: 0.03 }
    }
}

/// Ticks of continuous contact before a foot counts as grounded. Two at 90 Hz
/// ≈ 22 ms: rejects single-frame flicker at touchdown without delaying the
/// stance→swing decision noticeably.
pub const CONTACT_ON_TICKS: u32 = 2;
/// Ticks of continuous *no* contact before grounded drops. One: losing support
/// must act immediately, and the asymmetric `on` debounce already prevents
/// chatter.
pub const CONTACT_OFF_TICKS: u32 = 1;

/// Per-foot debounced contact state (ported from
/// [`ground.rs`](../../examples/game_lib/src/lower_body/hip_pin/ground.rs)).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FootDebounce {
    pub grounded: bool,
    pub on_ticks: u32,
    pub off_ticks: u32,
}

/// `alpha = dt / (tau + dt)`; `dt ≤ 0` or `tau ≤ 0` ⇒ `1.0` (snap).
#[inline]
pub fn ema_alpha(dt: f32, tau: f32) -> f32 {
    if dt <= 0.0 || tau <= 0.0 {
        1.0
    } else {
        dt / (tau + dt)
    }
}

/// Mass-weighted CoM over the samples; skips non-finite and `inv_mass ≤ 0`
/// (infinite-mass / static). Returns `(com, total_mass, n)` or `None`.
pub fn center_of_mass(samples: &[ParticleSample]) -> Option<(Vec2, f32, u32)> {
    let mut total = 0.0f32;
    let mut com = Vec2::ZERO;
    let mut n = 0u32;
    for s in samples {
        if !s.pos.is_finite() || !s.inv_mass.is_finite() || s.inv_mass <= 0.0 {
            continue;
        }
        let m = 1.0 / s.inv_mass;
        total += m;
        com += s.pos * m;
        n += 1;
    }
    if total <= 0.0 {
        return None;
    }
    Some((com / total, total, n))
}

/// Mass-weighted CoM velocity; `None` when nothing contributed a finite value.
pub fn com_velocity(samples: &[ParticleSample]) -> Option<Vec2> {
    let mut total = 0.0f32;
    let mut v = Vec2::ZERO;
    for s in samples {
        if !s.vel.is_finite() || !s.inv_mass.is_finite() || s.inv_mass <= 0.0 {
            continue;
        }
        let m = 1.0 / s.inv_mass;
        total += m;
        v += s.vel * m;
    }
    if total <= 0.0 {
        return None;
    }
    Some(v / total)
}

/// Stick attitude from the `hip → waist` axis against world vertical, Vello
/// y-down. Upright `(0, -1)` ⇒ 0 rad; `+` = forward lean toward `+x`.
/// Degenerate (zero-length) axes ⇒ `0`.
#[inline]
pub fn attitude_pitch(hip: Vec2, waist: Vec2) -> f32 {
    let axis = waist - hip;
    if axis.length_squared() < 1e-6 {
        0.0
    } else {
        axis.x.atan2(-axis.y)
    }
}

/// Advance one foot's debounce by one raw contact tick. Asymmetric by design:
/// gaining contact needs [`CONTACT_ON_TICKS`], losing it is immediate
/// ([`CONTACT_OFF_TICKS`]).
pub fn debounce_contact(prev: FootDebounce, touching: bool) -> FootDebounce {
    let on_ticks = if touching {
        prev.on_ticks.saturating_add(1)
    } else {
        0
    };
    let off_ticks = if touching {
        0
    } else {
        prev.off_ticks.saturating_add(1)
    };
    let grounded = if touching {
        on_ticks >= CONTACT_ON_TICKS
    } else if prev.grounded {
        off_ticks < CONTACT_OFF_TICKS
    } else {
        false
    };
    FootDebounce {
        grounded,
        on_ticks,
        off_ticks,
    }
}

/// Persistent observer state: EMA filters + foot hysteresis. The first tick
/// **snaps** each filter to its raw value (`primed`), so boot never blends
/// from zero.
#[derive(Debug, Clone, Copy, Default)]
pub struct Observer {
    /// The current snapshot — observe's product, control's input. Lives here
    /// (not as a free component field) because the observer is its only
    /// producer; `Default` is a zeroed fiction until the first tick.
    pub fiction: Fiction,
    v_hip: Vec2,
    v_com: Vec2,
    theta_prev: f32,
    omega: f32,
    primed: bool,
    feet: [FootDebounce; 2],
    /// Pre-filter reads of the most recent tick, for the reality telemetry
    /// columns (plan §3b): the CSV must show what the filter smoothed.
    pub raw: RawReality,
}

/// The unfiltered signal behind the current [`Fiction`]. Telemetry only;
/// nothing in the control path reads it.
#[derive(Debug, Clone, Copy, Default)]
pub struct RawReality {
    pub v_hip: Vec2,
    pub v_com: Vec2,
    pub lean: f32,
    pub lean_rate: f32,
    pub com: Vec2,
}

impl Observer {
    pub fn new() -> Self {
        Self::default()
    }

    /// One observe tick: raw measurements in, [`Fiction`] out.
    ///
    /// Filtering contract (plan D2): velocities and the attitude rate are
    /// EMAs with time constant `cfg.tau`; positions and the raw touch flag
    /// pass through untouched. The first tick snaps each filter to the raw
    /// sample so boot transients cannot smear into the fiction.
    pub fn observe(&mut self, s: &RealitySample, cfg: &ObserveConfig) -> Fiction {
        let alpha = ema_alpha(s.dt, cfg.tau);

        // --- attitude: raw θ, finite-differenced + EMA'd rate ---
        let theta = attitude_pitch(s.hip.pos, s.hip.waist);
        let omega_raw = if self.primed && s.dt > 0.0 {
            (theta - self.theta_prev) / s.dt
        } else {
            0.0
        };
        self.omega = if self.primed {
            self.omega + (omega_raw - self.omega) * alpha
        } else {
            omega_raw
        };
        self.theta_prev = theta;

        // --- filtered velocities ---
        let v_hip_raw = if s.hip.vel.is_finite() {
            s.hip.vel
        } else {
            Vec2::ZERO
        };
        self.v_hip = if self.primed {
            self.v_hip + (v_hip_raw - self.v_hip) * alpha
        } else {
            v_hip_raw
        };

        let v_com_raw = com_velocity(s.body).unwrap_or(v_hip_raw);
        self.v_com = if self.primed {
            self.v_com + (v_com_raw - self.v_com) * alpha
        } else {
            v_com_raw
        };
        self.primed = true;

        // --- CoM position: mass-weighted, hip fallback when degenerate ---
        let com = center_of_mass(s.body)
            .map(|(c, _, _)| c)
            .unwrap_or(s.hip.pos);

        // --- feet + hysteresis ---
        let mut feet = [FootState::default(); 2];
        for i in 0..2 {
            self.feet[i] = debounce_contact(self.feet[i], s.feet[i].touching);
            feet[i] = FootState {
                pos: s.feet[i].pos,
                vel: s.feet[i].vel,
                touching: s.feet[i].touching,
            };
        }
        // Support truth: touching *this tick* (raw — the same way the old
        // `GroundSupport` counted live contacts) or continuously grounded.
        let grounded =
            feet[0].touching || feet[1].touching || self.feet[0].grounded || self.feet[1].grounded;

        self.raw = RawReality {
            v_hip: v_hip_raw,
            v_com: v_com_raw,
            lean: theta,
            lean_rate: omega_raw,
            com,
        };

        let fiction = Fiction {
            dt: s.dt,
            hip: s.hip.pos,
            v_hip: self.v_hip,
            lean: theta,
            lean_rate: self.omega,
            com,
            v_com: self.v_com,
            feet,
            grounded,
        };
        self.fiction = fiction;
        fiction
    }

    /// Debounced per-foot support state (telemetry; the fiction exposes only
    /// the aggregate `grounded`).
    pub fn foot_debounce(&self) -> [FootDebounce; 2] {
        self.feet
    }
}

// ---------------------------------------------------------------------------
// Bevy adapter — particles + collision batch → OwnedRealitySample → Fiction
// ---------------------------------------------------------------------------

/// Entity handles for one observer: the hip/waist/foot particles (physics
/// entities) and the foot *collider* entities used to attribute contacts in
/// [`CollisionEventBatch`].
#[derive(Debug, Clone, Copy)]
pub struct ObserveHandles {
    /// Hip particle (`P3`).
    pub hip: bevy::prelude::Entity,
    /// Waist particle (`P2`) — defines the stick axis with the hip.
    pub waist: bevy::prelude::Entity,
    /// Foot particles `[left, right]`.
    pub feet: [bevy::prelude::Entity; 2],
    /// Foot colliders `[left, right]` for contact attribution.
    pub foot_colliders: [bevy::prelude::Entity; 2],
}

/// One observe tick end-to-end: gather from the ECS, filter, submit fiction.
pub fn observe_tick(
    handles: &ObserveHandles,
    character: bevy::prelude::Entity,
    p_q: &bevy::prelude::Query<&bevy_vello::integrations::physics::VelloParticle>,
    colliders: &bevy::prelude::Query<&bevy_vello::VelloCollider>,
    batch: Option<&bevy_vello::collision::CollisionEventBatch>,
    observer: &mut Observer,
    cfg: &ObserveConfig,
    dt: f32,
) -> Fiction {
    let mut owned = sample(handles, character, p_q, colliders, batch);
    owned.dt = dt;
    observer.observe(&owned.as_sample(), cfg)
}

/// Gather one tick into the owned sample: read every character particle
/// (mass-weighted CoM body), the hip/waist/foot particles, and the raw
/// per-foot contacts from last tick's [`CollisionEventBatch`].
///
/// The batch is written by `run_gpu_collision` at the end of FixedUpdate and
/// read here at the start of the next tick — the intended 1-tick detect→respond
/// lag, unchanged from the old bridge. `dt` is left `0.0`; the caller stamps it.
pub fn sample(
    handles: &ObserveHandles,
    character: bevy::prelude::Entity,
    p_q: &bevy::prelude::Query<&bevy_vello::integrations::physics::VelloParticle>,
    colliders: &bevy::prelude::Query<&bevy_vello::VelloCollider>,
    batch: Option<&bevy_vello::collision::CollisionEventBatch>,
) -> OwnedRealitySample {
    let touching = batch
        .map(|b| contacts_for_feet(b, colliders, handles.foot_colliders))
        .unwrap_or([false; 2]);

    // Whole-character particle sweep (CoM body). `VelloParticle.root_entity`
    // identifies the character, so no game-side component types are needed.
    let mut body = Vec::new();
    for p in p_q.iter() {
        if p.root_entity != character {
            continue;
        }
        let part = p.particle;
        body.push(ParticleSample {
            pos: part.pos,
            vel: part.velocity,
            inv_mass: part.inv_mass,
        });
    }

    let g = |e: bevy::prelude::Entity| p_q.get(e).ok().map(|p| p.particle);
    let (hip_pos, hip_vel) = g(handles.hip)
        .map(|p| (p.pos, p.velocity))
        .unwrap_or((Vec2::ZERO, Vec2::ZERO));
    let waist = g(handles.waist).map(|p| p.pos).unwrap_or(Vec2::ZERO);
    let mut feet = [
        FootSample {
            pos: Vec2::ZERO,
            vel: Vec2::ZERO,
            touching: touching[0],
        },
        FootSample {
            pos: Vec2::ZERO,
            vel: Vec2::ZERO,
            touching: touching[1],
        },
    ];
    for i in 0..2 {
        if let Some(p) = g(handles.feet[i]) {
            feet[i].pos = p.pos;
            feet[i].vel = p.velocity;
        }
    }

    OwnedRealitySample {
        dt: 0.0,
        hip: HipSample {
            pos: hip_pos,
            vel: hip_vel,
            waist,
        },
        feet,
        body,
    }
}

/// Raw-contact attribution for the two foot colliders from last tick's batch:
/// a foot counts when its soft body collided with a NON-soft entity (floor).
/// Same-side or soft-vs-soft pairs are ignored (mirrors the old ground layer).
pub fn contacts_for_feet(
    batch: &bevy_vello::collision::CollisionEventBatch,
    colliders: &bevy::prelude::Query<&bevy_vello::VelloCollider>,
    foot_colliders: [bevy::prelude::Entity; 2],
) -> [bool; 2] {
    let mut out = [false; 2];
    for entry in &batch.entries {
        let ev = &entry.event;
        let a_soft = colliders
            .get(ev.entity_a)
            .map(|c| c.is_soft_body())
            .unwrap_or(false);
        let b_soft = colliders
            .get(ev.entity_b)
            .map(|c| c.is_soft_body())
            .unwrap_or(false);
        if a_soft == b_soft {
            continue; // character vs character (or static vs static)
        }
        let soft_entity = if a_soft { ev.entity_a } else { ev.entity_b };
        for (slot, foot) in out.iter_mut().zip(foot_colliders) {
            if soft_entity == foot {
                *slot = true;
            }
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Tests (pure core; the adapter is exercised at the harness level)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn standing_body() -> Vec<ParticleSample> {
        // Hip-heavy budget (§7): P3 carries most of the mass.
        vec![
            ParticleSample {
                pos: Vec2::new(0.0, -140.0),
                vel: Vec2::new(10.0, 0.0),
                inv_mass: 1.0 / 8.0,
            },
            ParticleSample {
                pos: Vec2::new(0.0, -180.0),
                vel: Vec2::new(10.0, 0.0),
                inv_mass: 1.0 / 2.0,
            },
            ParticleSample {
                pos: Vec2::new(-20.0, -120.0),
                vel: Vec2::new(10.0, 0.0),
                inv_mass: 1.0,
            },
        ]
    }

    fn sample_at(
        dt: f32,
        hip: Vec2,
        v_hip: Vec2,
        waist: Vec2,
        touching: [bool; 2],
        body: &[ParticleSample],
    ) -> RealitySample<'_> {
        RealitySample {
            dt,
            hip: HipSample {
                pos: hip,
                vel: v_hip,
                waist,
            },
            feet: [
                FootSample {
                    pos: Vec2::new(-20.0, -90.0),
                    vel: v_hip,
                    touching: touching[0],
                },
                FootSample {
                    pos: Vec2::new(20.0, -90.0),
                    vel: v_hip,
                    touching: touching[1],
                },
            ],
            body,
        }
    }

    #[test]
    fn first_tick_snaps_filters_to_raw_values() {
        let mut o = Observer::new();
        let cfg = ObserveConfig::default();
        let f = o.observe(
            &sample_at(
                1.0 / 90.0,
                Vec2::new(0.0, -140.0),
                Vec2::new(100.0, 0.0),
                Vec2::new(0.0, -180.0),
                [false, false],
                &standing_body(),
            ),
            &cfg,
        );
        // Snap, not blend-from-zero: a boot step at 100 px/s reads 100, not ~3.
        assert!((f.v_hip.x - 100.0).abs() < 1e-6, "v_hip {}", f.v_hip.x);
        assert_eq!(f.lean_rate, 0.0, "no rate can exist on the first tick");
    }

    #[test]
    fn ema_smooths_gait_ripple_in_com_velocity() {
        let mut o = Observer::new();
        let cfg = ObserveConfig { tau: 0.1 };
        let base = ParticleSample {
            pos: Vec2::ZERO,
            vel: Vec2::ZERO,
            inv_mass: 1.0,
        };
        let dt = 1.0 / 90.0;
        let mut last = Fiction {
            dt,
            hip: Vec2::ZERO,
            v_hip: Vec2::ZERO,
            lean: 0.0,
            lean_rate: 0.0,
            com: Vec2::ZERO,
            v_com: Vec2::ZERO,
            feet: [FootState::default(); 2],
            grounded: false,
        };
        // Ripple: alternating 400 / 200 → true average 300.
        for i in 0..90 {
            let speed = if i % 2 == 0 { 400.0 } else { 200.0 };
            let body = [ParticleSample {
                vel: Vec2::new(speed, 0.0),
                ..base
            }];
            last = o.observe(
                &sample_at(
                    dt,
                    Vec2::ZERO,
                    Vec2::new(speed, 0.0),
                    Vec2::ZERO,
                    [false, false],
                    &body,
                ),
                &cfg,
            );
        }
        assert!(
            (last.v_com.x - 300.0).abs() < 40.0,
            "EMA must center the ripple on its mean, got {}",
            last.v_com.x
        );
        assert!(
            (last.v_com.x - 400.0).abs() > 50.0,
            "the filtered signal must not be the raw ripple"
        );
    }

    #[test]
    fn ema_alpha_formula_and_guards() {
        assert!((ema_alpha(1.0, 1.0) - 0.5).abs() < 1e-6);
        assert_eq!(ema_alpha(0.0, 1.0), 1.0);
        assert_eq!(ema_alpha(1.0, 0.0), 1.0);
        assert_eq!(ema_alpha(-1.0, -1.0), 1.0);
    }

    #[test]
    fn com_mass_weights_and_skips_statics() {
        let samples = [
            ParticleSample {
                pos: Vec2::new(0.0, 0.0),
                vel: Vec2::new(0.0, 0.0),
                inv_mass: 1.0, // m = 1
            },
            ParticleSample {
                pos: Vec2::new(10.0, 0.0),
                vel: Vec2::new(0.0, 0.0),
                inv_mass: 0.5, // m = 2, twice the pull
            },
            ParticleSample {
                pos: Vec2::new(500.0, 500.0),
                vel: Vec2::new(f32::NAN, 0.0),
                inv_mass: 0.0, // infinite mass — skipped
            },
        ];
        let (com, total, n) = center_of_mass(&samples).unwrap();
        assert_eq!(n, 2);
        assert!((total - 3.0).abs() < 1e-5, "total {total}");
        assert!((com.x - 20.0 / 3.0).abs() < 1e-4, "com {com}");
    }

    #[test]
    fn attitude_upright_is_zero_and_forward_lean_is_positive() {
        // y-down frame: waist above the hip is (0, -L) ⇒ 0 rad.
        assert!(attitude_pitch(Vec2::new(0.0, -140.0), Vec2::new(0.0, -180.0)).abs() < 1e-6);
        // Waist tilted toward +x (forward lean) ⇒ positive θ.
        let t = attitude_pitch(Vec2::new(0.0, -140.0), Vec2::new(20.0, -180.0));
        assert!(t > 0.0, "forward lean must be positive, got {t}");
        // Degenerate axis ⇒ 0, never NaN.
        assert_eq!(attitude_pitch(Vec2::splat(7.0), Vec2::splat(7.0)), 0.0);
    }

    #[test]
    fn lean_and_rate_reach_the_fiction() {
        let mut o = Observer::new();
        let cfg = ObserveConfig { tau: 1e-3 };
        let body = [ParticleSample::default()];
        // Snap the axis upright.
        o.observe(
            &sample_at(
                1.0 / 90.0,
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::new(0.0, -40.0),
                [false, false],
                &body,
            ),
            &cfg,
        );
        // Tip it forward by ~0.2 rad in one tick: θ follows, ω reads positive.
        let f = o.observe(
            &sample_at(
                1.0 / 90.0,
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::new(8.0, -40.0),
                [false, false],
                &body,
            ),
            &cfg,
        );
        assert!(f.lean > 0.15 && f.lean < 0.25, "lean {}", f.lean);
        assert!(f.lean_rate > 0.0, "rate {}", f.lean_rate);
    }

    #[test]
    fn contact_debounce_arms_after_two_ticks_and_drops_at_once() {
        let mut o = Observer::new();
        let cfg = ObserveConfig::default();
        let body = [ParticleSample::default()];
        let f1 = o.observe(
            &sample_at(
                1.0 / 90.0,
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::ZERO,
                [true, false],
                &body,
            ),
            &cfg,
        );
        // One tick of flicker still counts as support (raw path), but the
        // per-foot debounced flag has not armed.
        assert!(f1.grounded, "raw contact must count as support");
        assert!(
            !o.foot_debounce()[0].grounded,
            "1 tick must not arm debounce"
        );
        let _ = o.observe(
            &sample_at(
                1.0 / 90.0,
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::ZERO,
                [true, false],
                &body,
            ),
            &cfg,
        );
        assert!(o.foot_debounce()[0].grounded, "2 ticks must arm debounce");
        // Loss of contact is immediate.
        let f3 = o.observe(
            &sample_at(
                1.0 / 90.0,
                Vec2::ZERO,
                Vec2::ZERO,
                Vec2::ZERO,
                [false, false],
                &body,
            ),
            &cfg,
        );
        assert!(!f3.grounded, "support must drop the tick contact is lost");
    }

    #[test]
    fn feet_pass_positions_and_raw_touch_through_untouched() {
        let mut o = Observer::new();
        let cfg = ObserveConfig::default();
        let body = [ParticleSample::default()];
        let f = o.observe(
            &sample_at(
                1.0 / 90.0,
                Vec2::new(5.0, -140.0),
                Vec2::new(30.0, 0.0),
                Vec2::ZERO,
                [true, false],
                &body,
            ),
            &cfg,
        );
        assert!(f.feet[0].touching);
        assert!(!f.feet[1].touching);
        assert_eq!(f.feet[0].pos, Vec2::new(-20.0, -90.0));
        assert_eq!(f.dt, 1.0 / 90.0);
        assert_eq!(f.hip, Vec2::new(5.0, -140.0));
    }

    #[test]
    fn empty_body_falls_back_to_hip_for_com_and_raw_velocity() {
        // The §7 P3-heavy budget makes CoM ≈ hip; a degenerate sweep must not
        // poison the fiction with zeros.
        let mut o = Observer::new();
        let cfg = ObserveConfig::default();
        let hip = Vec2::new(0.0, -140.0);
        let f = o.observe(
            &sample_at(
                1.0 / 90.0,
                hip,
                Vec2::new(12.0, 0.0),
                Vec2::ZERO,
                [false, false],
                &[],
            ),
            &cfg,
        );
        assert_eq!(f.com, hip);
        assert!((f.v_com.x - 12.0).abs() < 1e-6);
    }

    #[test]
    fn raw_telemetry_exposes_the_pre_filter_signal() {
        // Plan §3b: the reality CSV columns must show what the filter smoothed.
        let mut o = Observer::new();
        let cfg = ObserveConfig { tau: 0.1 };
        let body = [ParticleSample {
            pos: Vec2::new(1.0, 2.0),
            vel: Vec2::new(400.0, 0.0),
            inv_mass: 1.0,
        }];
        let dt = 1.0 / 90.0;
        o.observe(
            &sample_at(
                dt,
                Vec2::ZERO,
                Vec2::new(400.0, 0.0),
                Vec2::ZERO,
                [false, false],
                &body,
            ),
            &cfg,
        );
        // Prime once so the second tick blends.
        let body2 = [ParticleSample {
            vel: Vec2::new(200.0, 0.0),
            ..body[0]
        }];
        let f = o.observe(
            &sample_at(
                dt,
                Vec2::ZERO,
                Vec2::new(200.0, 0.0),
                Vec2::ZERO,
                [false, false],
                &body2,
            ),
            &cfg,
        );
        assert!(
            (o.raw.v_hip.x - 200.0).abs() < 1e-3,
            "raw = the tick's sample"
        );
        assert!(f.v_hip.x > 200.0, "filtered still pulling down from 400");
        assert!((o.raw.com.x - 1.0).abs() < 1e-6);
    }

    #[test]
    fn fiction_is_deterministic_for_identical_sample_streams() {
        let mk = |o: &mut Observer| {
            let cfg = ObserveConfig::default();
            let body = standing_body();
            for i in 0..30 {
                let touching = [i % 3 != 0, i % 5 != 0];
                let v = Vec2::new(50.0 + i as f32, 0.0);
                let _ = o.observe(
                    &sample_at(
                        1.0 / 90.0,
                        Vec2::new(i as f32, -140.0),
                        v,
                        Vec2::new(2.0, -180.0),
                        touching,
                        &body,
                    ),
                    &cfg,
                );
            }
            o.observe(
                &sample_at(
                    1.0 / 90.0,
                    Vec2::new(30.0, -140.0),
                    Vec2::new(80.0, 0.0),
                    Vec2::new(3.0, -180.0),
                    [true, false],
                    &body,
                ),
                &cfg,
            )
        };
        let a = mk(&mut Observer::new());
        let b = mk(&mut Observer::new());
        assert_eq!(
            a, b,
            "same inputs ⇒ same fiction (D1 determinism prerequisite)"
        );
    }
}
