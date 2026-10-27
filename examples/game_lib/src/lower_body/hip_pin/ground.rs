use bevy::prelude::*;
use bevy_vello::collision::CollisionEventBatch;
use bevy_vello::VelloCollider;



#[derive(Debug, Clone, Copy, Default)]
pub struct FootContact {
    /// Debounced: in contact for at least `CONTACT_ON_TICKS` consecutive ticks.
    pub grounded: bool,
    /// Raw contact this tick (before the debounce).
    pub touching: bool,
    /// Consecutive ticks in contact (saturating).
    pub on_ticks: u32,
    /// Consecutive ticks *out* of contact (saturating).
    pub off_ticks: u32,
    /// Upward-pointing contact normal (vello y-down), zero when not touching.
    pub normal: Vec2,
    /// Contact point in vello world space, or zero when not touching.
    pub point: Vec2,
    /// Closing speed along the contact normal, px/s (positive = approaching).
    pub normal_speed: f32,
}

/// The pin may cancel gravity and command y only while supported; in flight
/// gravity is ballistic. Register [`update_ground_support`] before
/// [`drive_hip_pin`].
#[derive(Resource, Debug, Clone)]
pub struct GroundSupport {
    /// True when **any** character collider touches non-character geometry.
    /// Derived from the per-foot layer plus body contacts, so it is strictly
    /// more accurate than the old global count.
    pub grounded: bool,
    /// Number of near-vertical mixed contacts seen this frame.
    pub contacts: usize,
    /// `[left, right]` foot colliders (`PLLL`, `RLL`), when known.
    pub feet: [FootContact; 2],
    /// True when at least one foot collider was identified in the character.
    /// `false` ⇒ the per-foot data is meaningless and callers must fall back to
    /// the global flags (this happens for characters without foot colliders).
    pub feet_valid: bool,
    /// A body part *other than a foot* is in contact (knee, hand, head…).
    pub body_contact: bool,
}

impl Default for GroundSupport {
    fn default() -> Self {
        // No collision pipeline ⇒ keep the kinematic behaviour. `feet_valid`
        // stays false so the resolver's §6 gating is skipped entirely and the
        // pre-§6 timer behaviour is preserved exactly.
        Self {
            grounded: true,
            contacts: 0,
            feet: [FootContact::default(); 2],
            feet_valid: false,
            body_contact: false,
        }
    }
}

/// Ticks of continuous contact before a foot counts as `grounded`. Two ticks at
/// 90 Hz ≈ 22 ms: enough to reject the single-frame flicker at lift-off and
/// touchdown, short enough not to delay the stance→swing decision noticeably.
const CONTACT_ON_TICKS: u32 = 2;
/// Ticks of continuous *no* contact before a foot stops being `grounded`. One
/// tick, because losing contact matters more urgently than gaining it (loop 2
/// must stop pushing immediately) — and the `on` debounce already prevents the
/// symmetric chatter case.
const CONTACT_OFF_TICKS: u32 = 1;
/// A contact counts as support when its normal is within this of vertical
/// (`|n.y| >= COS(60°) = 0.5`). Kept as a magnitude test for backwards
/// compatibility with the old global flag.
const CONTACT_NORMAL_Y_MIN: f32 = 0.5;

/// Returns true if `name` identifies the left foot collider.
pub(crate) fn is_left_foot(name: &str) -> bool {
    name == "PLLL" || name == "LLL"
}

/// Returns true if `name` identifies the right foot collider.
pub(crate) fn is_right_foot(name: &str) -> bool {
    name == "PRLL" || name == "RLL"
}

/// Advance one foot's contact state by one tick.
///
/// The asymmetry is deliberate and is the whole point of the debounce:
/// * gaining contact needs [`CONTACT_ON_TICKS`] consecutive ticks, so a
///   single-frame flicker at touchdown cannot arm loop 2;
/// * losing contact is immediate ([`CONTACT_OFF_TICKS`]), because a foot that
///   has left the ground must not be granted thrust authority for even one more
///   tick — that is exactly the "pushing against nothing" case.
///
/// Pure so it can be unit-tested without an ECS.
pub(crate) fn debounce_contact(prev: FootContact, touching: bool, raw: FootContact) -> FootContact {
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
    FootContact {
        grounded,
        touching,
        on_ticks,
        off_ticks,
        normal: raw.normal,
        point: raw.point,
        normal_speed: raw.normal_speed,
    }
}

/// Refresh [`GroundSupport`] from the previous tick's [`CollisionEventBatch`].
///
/// This is the §6 contact layer. It resolves *which* collider touched, keeps a
/// debounce per foot, and records the contact normal/point so the loops can be
/// gated on real ground truth instead of the resolver's timers.
///
/// Ordering note: the batch is written by `run_gpu_collision` at the end of
/// FixedUpdate and read here at the start of the next tick, so this is *last
/// tick's* contact. That is the intended semantics — the pin acts on the most
/// recent measurement it can have.
pub fn update_ground_support(
    batch: Option<Res<CollisionEventBatch>>,
    colliders: Query<&VelloCollider>,
    connectivity: Query<&crate::character::Connectivity>,
    mut support: ResMut<GroundSupport>,
) {
    let Some(batch) = batch else {
        // No collision pipeline (pure kinematic sandbox): keep the pre-§6
        // behaviour alive and make sure the per-foot layer reports "unknown".
        support.grounded = true;
        support.contacts = 0;
        support.feet = [FootContact::default(); 2];
        support.feet_valid = false;
        support.body_contact = false;
        return;
    };

    // Accumulate this tick's raw contacts.
    let mut raw = [FootContact::default(); 2];
    let mut raw_touching = [false; 2];
    let mut contacts = 0usize;
    let mut body_contact = false;
    let mut saw_foot_collider = false;

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
        // Take the normal/point from the soft-body side.
        let (normal, point, vel) = if a_soft {
            (ev.collision_normal_a, ev.collision_point_a, ev.velocity_a)
        } else {
            (ev.collision_normal_b, ev.collision_point_b, ev.velocity_b)
        };
        if normal.y.abs() < CONTACT_NORMAL_Y_MIN {
            continue; // not a near-horizontal floor contact
        }
        contacts += 1;

        // Attribute to a foot when the soft body is a foot collider.
        let soft_entity = if a_soft { ev.entity_a } else { ev.entity_b };
        let name = connectivity.get(soft_entity).ok().map(|c| c.name);
        let slot = match name.as_deref() {
            Some(n) if is_left_foot(n) => Some(0),
            Some(n) if is_right_foot(n) => Some(1),
            Some(_) => {
                body_contact = true;
                None
            }
            None => {
                // Names can legitimately be unavailable (e.g. a bare collider
                // without `Connectivity`); treat it as a body contact rather
                // than silently crediting a foot.
                body_contact = true;
                None
            }
        };
        if name.is_some() {
            saw_foot_collider = true;
        }
        if let Some(i) = slot {
            raw_touching[i] = true;
            raw[i].touching = true;
            raw[i].normal = normal;
            raw[i].point = point;
            // Closing speed along the normal (positive = moving into the floor).
            raw[i].normal_speed = vel.dot(normal);
            saw_foot_collider = true;
        }
    }

    // Debounce each foot against its previous state.
    let mut feet = support.feet;
    for i in 0..2 {
        feet[i] = debounce_contact(support.feet[i], raw_touching[i], raw[i]);
    }

    support.feet = feet;
    // Footer colliders are identified once ANY foot collider has ever been seen
    // in the batch. If a character genuinely has none, the per-foot layer must
    // not be trusted, so the resolver keeps its timer behaviour.
    support.feet_valid = saw_foot_collider || support.feet_valid;
    support.contacts = contacts;
    support.body_contact = body_contact;
    support.grounded = contacts > 0 || feet[0].grounded || feet[1].grounded;
}
