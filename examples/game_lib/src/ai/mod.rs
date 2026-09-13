//! AI behavior for enemy characters using `bevy_behave` behavior trees.
//!
//! The AI drives a `CHARGE → FLEE → IDLE → CHARGE` loop that is purely
//! **distance/facing driven** — there are no `Wait` timers. Transitions are
//! decided live each frame by trigger conditions that read real physics state.
//!
//! # Behaviour
//!
//! - **CHARGE** — the enemy moves head-first (tail→head along its spine)
//!   toward the player. It keeps charging until:
//!     * **fist contact** ([`AiState::contact_landed`] set by the weapons
//!       observer when a player body part touches the enemy), or
//!     * **over-charge / miss** — after committing to some travel distance,
//!       if the enemy now requires a large turn (>60°) to keep facing the
//!       player it has overshot and would expose its back, so it retreats.
//!   Either way it transitions immediately to **FLEE**.
//! - **FLEE** — move directly away from the player until it reaches a safe
//!   distance ([`SAFE_DISTANCE`]).
//! - **IDLE** — stand still and keep its distance, re-preparing for a new
//!   charge. When the player closes back into engagement range
//!   ([`ADVANCE_RANGE`]) it transitions to **CHARGE** again.
//!
//! # Architecture
//!
//! - **Behavior tree** (`Forever → Sequence → [While(Charge), While(Flee),
//!   While(Idle)]`). Each phase is a single-child `While(trigger(cond))` node.
//!   Because a single-child `While` resets and re-fires its condition trigger
//!   after every successful loop, the condition observers are polled **every
//!   frame** — giving live decisions with no `Wait` node.
//! - Condition observers (`on_while_charge/flee/idle`) read the enemy's
//!   [`AiState`], real physics particle positions, and the player's position;
//!   they (a) apply the correct steering marker ([`AiChaseTarget`] /
//!   [`AiFleeTarget`]) and (b) report `ctx.success()` to keep looping or
//!   `ctx.failure()` when the phase is done. Each gate is wrapped in
//!   [`Behave::Invert`] so `failure` (phase done) becomes `success` and the
//!   parent `Sequence` advances to the next phase.
//! - **`ai_travel_system`** accumulates how far the enemy has physically
//!   traveled within the current phase ([`AiState::travel`]), resetting on a
//!   phase change. This is the distance basis for the over-charge decision.
//! - **`ai_steer_system`** (per-frame, `Update`) reads the markers, reads
//!   actual physics particle positions (`VelloParticle` on P3), computes
//!   direction + wall avoidance, and writes `SpineController.move_vector`.

use bevy::prelude::*;
use bevy_behave::prelude::*;

use bevy_vello::integrations::physics::VelloParticle;

use crate::character::SpineController;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Movement speed applied to the character's `move_vector`.
const AI_SPEED: f32 = 80.0;

/// Strength of the wall-avoidance repulsion force.
const AVOID_FORCE: f32 = 200.0;

/// Distance from a wall boundary at which avoidance kicks in.
const AVOID_MARGIN: f32 = 120.0;

/// Arena safe area (bevy y-up). Walls are at ±1920 x / ±1080 y.
const ARENA_MIN_X: f32 = -1880.0;
const ARENA_MAX_X: f32 = 1880.0;
const ARENA_MIN_Y: f32 = -1040.0;
const ARENA_MAX_Y: f32 = 1040.0;

/// Distance from the player at which the enemy stops fleeing. Once far enough
/// it moves to the IDLE phase.
const SAFE_DISTANCE: f32 = 420.0;

/// Distance from the player at which a defensive (IDLE) enemy re-engages and
/// starts a new CHARGE. Kept below [`SAFE_DISTANCE`] so the envelope has
/// hysteresis: flee out past `SAFE_DISTANCE`, then only come back when the
/// player closes within `ADVANCE_RANGE`.
const ADVANCE_RANGE: f32 = 380.0;

/// How long (seconds) the enemy will stand still in the IDLE phase before it
/// re-engages and starts a fresh CHARGE on its own — even if the player never
/// closes back into [`ADVANCE_RANGE`]. Keeps the AI cycling continuously.
const IDLE_DURATION: f32 = 1.5;

/// Travel (within the current CHARGE phase) that must be accumulated before a
/// facing-mismatch is treated as an *over-charge / miss*. This prevents an
/// enemy that is merely starting to turn from instantly retreating.
const CHARGE_MIN_TRAVEL: f32 = 60.0;

/// Minimum dot product between the enemy's facing direction and the direction
/// to the player to consider a charge still viable. `cos(60°) = 0.5`; below
/// that the enemy needs a >60° turn and is over-charged (dangerous back
/// exposure), so it flees.
const CHARGE_ANGLE_COS: f32 = 0.5;

// ---------------------------------------------------------------------------
// AI state (on the character root entity)
// ---------------------------------------------------------------------------

/// The current phase an enemy AI character is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AiPhase {
    /// Moving head-first toward the player.
    Charge,
    /// Moving directly away from the player to a safe distance.
    Flee,
    /// Standing still, keeping distance, preparing for the next charge.
    Idle,
}

/// Per-enemy AI state stored on the character root.
///
/// This is the single source of truth for the AI's current phase. The
/// behavior-tree condition observers read and mutate it, and it drives the
/// steering markers via the condition observers.
#[derive(Component)]
pub struct AiState {
    /// Current AI phase.
    pub phase: AiPhase,
    /// Set to `true` by [`crate::weapons`] when a player body part (fist)
    /// contacts this enemy — the signal to stop a charge and flee.
    pub contact_landed: bool,
    /// Distance traveled within the current phase (bevy y-up units).
    pub travel: f32,
    /// Accumulated time (seconds) the enemy has stood still in the IDLE phase.
    /// Once it exceeds [`crate::ai::IDLE_DURATION`] the enemy re-engages on its
    /// own, so it cycles continuously even with a stationary player.
    pub idle_timer: f32,
    /// Last observed position (bevy y-up) used to accumulate [`Self::travel`].
    last_pos: Option<Vec2>,
    /// The phase the travel accumulator was recording for, so it can detect a
    /// phase change and reset the accumulator.
    recorded_phase: AiPhase,
}

impl Default for AiState {
    fn default() -> Self {
        Self {
            phase: AiPhase::Charge,
            contact_landed: false,
            travel: 0.0,
            idle_timer: 0.0,
            last_pos: None,
            recorded_phase: AiPhase::Charge,
        }
    }
}

// ---------------------------------------------------------------------------
// Marker components (on the character root entity)
// ---------------------------------------------------------------------------

/// Marker inserted on the character root while the AI is in chase mode.
/// Read by [`ai_steer_system`] to drive movement toward the player.
#[derive(Component)]
pub struct AiChaseTarget {
    /// The player entity to chase.
    pub player: Entity,
}

/// Marker inserted on the character root while the AI is in flee mode.
#[derive(Component)]
pub struct AiFleeTarget {
    /// The player entity to flee from.
    pub player: Entity,
}

// ---------------------------------------------------------------------------
// Behavior-tree condition payloads (fired by `Behave::trigger`)
// ---------------------------------------------------------------------------

// Each payload tags a distinct `BehaveTrigger<T>` event type, so each has its
// own observer below. They only need `Clone + Send + Sync` (bevy_behave
// requirement for `Behave::trigger`); the character root is obtained from the
// trigger's `BehaveCtx`.

/// Condition: "keep charging?". Emitted by the CHARGE gate.
#[derive(Clone)]
pub struct WhileCharging {
    /// The player entity being charged.
    pub player: Entity,
}

/// Condition: "keep fleeing?". Emitted by the FLEE gate.
#[derive(Clone)]
pub struct WhileFleeing {
    /// The player entity being fled from.
    pub player: Entity,
}

/// Condition: "keep idling?". Emitted by the IDLE gate.
#[derive(Clone)]
pub struct WhileIdling {
    /// The player entity to prepare to re-engage.
    pub player: Entity,
}

// ---------------------------------------------------------------------------
// Condition observers (polled every frame by the single-child `While` nodes)
// ---------------------------------------------------------------------------

fn handle_charge(
    root: Entity,
    player: Entity,
    ai_q: &mut Query<&mut AiState>,
    spine_q: &Query<&SpineController>,
    particle_q: &Query<&VelloParticle>,
    commands: &mut Commands,
    ctx: &BehaveCtx,
) {
    let Ok(mut ai) = ai_q.get_mut(root) else {
        commands.trigger(ctx.failure());
        return;
    };
    if ai.phase != AiPhase::Charge {
        // Not supposed to be charging — bail out of this gate.
        commands.trigger(ctx.failure());
        return;
    }

    // Apply steering: chase the player, cancel any leftover flee.
    commands.entity(root).insert(AiChaseTarget { player });
    commands.entity(root).remove::<AiFleeTarget>();

    // Decide whether the charge is over (→ FLEE).
    let Some(enemy_base) = spine_q
        .get(root)
        .ok()
        .and_then(|s| get_bevy_pos(&s, particle_q))
    else {
        commands.trigger(ctx.success());
        return;
    };
    let Some(player_pos) = get_player_bevy_pos(player, spine_q, particle_q) else {
        commands.trigger(ctx.success());
        return;
    };

    // 1) Fist contact — the player hit us, flee immediately.
    if ai.contact_landed {
        ai.phase = AiPhase::Flee;
        ai.contact_landed = false;
        commands.entity(root).remove::<AiChaseTarget>();
        commands.trigger(ctx.failure());
        return;
    }

    // 2) Over-charge / miss — after committing to travel, if we now need a
    //    >60° turn to keep facing the player we've overshot and our back is
    //    exposed, so retreat.
    let mut over_charged = false;
    if ai.travel > CHARGE_MIN_TRAVEL {
        if let Some(facing) = enemy_facing(root, spine_q, particle_q) {
            let to_player = (player_pos - enemy_base).normalize_or_zero();
            if facing.length_squared() > 1e-4 && to_player.length_squared() > 1e-4 {
                over_charged = facing.dot(to_player) < CHARGE_ANGLE_COS;
            }
        }
    }
    if over_charged {
        ai.phase = AiPhase::Flee;
        commands.entity(root).remove::<AiChaseTarget>();
        commands.trigger(ctx.failure());
        return;
    }

    // Otherwise keep charging.
    commands.trigger(ctx.success());
}

fn handle_flee(
    root: Entity,
    player: Entity,
    ai_q: &mut Query<&mut AiState>,
    spine_q: &Query<&SpineController>,
    particle_q: &Query<&VelloParticle>,
    commands: &mut Commands,
    ctx: &BehaveCtx,
) {
    let Ok(mut ai) = ai_q.get_mut(root) else {
        commands.trigger(ctx.failure());
        return;
    };
    if ai.phase != AiPhase::Flee {
        commands.trigger(ctx.failure());
        return;
    }

    // Apply steering: flee away, cancel any leftover chase.
    commands.entity(root).insert(AiFleeTarget { player });
    commands.entity(root).remove::<AiChaseTarget>();

    let Some(enemy_base) = spine_q
        .get(root)
        .ok()
        .and_then(|s| get_bevy_pos(&s, particle_q))
    else {
        commands.trigger(ctx.success());
        return;
    };
    let Some(player_pos) = get_player_bevy_pos(player, spine_q, particle_q) else {
        commands.trigger(ctx.success());
        return;
    };

    let dist = enemy_base.distance(player_pos);
    if dist > SAFE_DISTANCE {
        // Reached a safe distance → IDLE.
        ai.phase = AiPhase::Idle;
        commands.entity(root).remove::<AiFleeTarget>();
        commands.trigger(ctx.failure());
        return;
    }

    // Otherwise keep fleeing.
    commands.trigger(ctx.success());
}

fn handle_idle(
    root: Entity,
    player: Entity,
    ai_q: &mut Query<&mut AiState>,
    spine_q: &Query<&SpineController>,
    particle_q: &Query<&VelloParticle>,
    commands: &mut Commands,
    ctx: &BehaveCtx,
) {
    let Ok(mut ai) = ai_q.get_mut(root) else {
        commands.trigger(ctx.failure());
        return;
    };
    if ai.phase != AiPhase::Idle {
        commands.trigger(ctx.failure());
        return;
    }

    // Idle: stand still (no steering marker), keep distance, re-prepare.
    // `ai_steer_system` detects the missing markers and zeroes `move_vector`
    // so the character actually stops (no stale flee thrust into the wall).
    commands.entity(root).remove::<AiChaseTarget>();
    commands.entity(root).remove::<AiFleeTarget>();

    let Some(enemy_base) = spine_q
        .get(root)
        .ok()
        .and_then(|s| get_bevy_pos(&s, particle_q))
    else {
        commands.trigger(ctx.success());
        return;
    };
    let Some(player_pos) = get_player_bevy_pos(player, spine_q, particle_q) else {
        commands.trigger(ctx.success());
        return;
    };

    // Re-engage once the player closes back into range → fresh CHARGE.
    // (The self-driven idle-timeout re-charge is handled separately in the
    // every-frame [`ai_travel_system`], because the tree's IDLE gate only runs
    // `handle_idle` once and never re-polls its timer.)
    let dist = enemy_base.distance(player_pos);
    if !ai.contact_landed && dist < ADVANCE_RANGE {
        ai.phase = AiPhase::Charge;
        ai.travel = 0.0;
        commands.trigger(ctx.failure());
        return;
    }

    // Otherwise keep waiting at a defensive distance.
    commands.trigger(ctx.success());
}

/// `OnAdd<(WhileCharging,)>` observer: entry point for the CHARGE gate.
pub fn on_while_charging(
    trigger: Trigger<BehaveTrigger<WhileCharging>>,
    mut ai_q: Query<&mut AiState>,
    spine_q: Query<&SpineController>,
    particle_q: Query<&VelloParticle>,
    mut commands: Commands,
) {
    let ctx = trigger.event().ctx();
    let root = ctx.target_entity();
    let player = trigger.event().inner().player;
    handle_charge(
        root,
        player,
        &mut ai_q,
        &spine_q,
        &particle_q,
        &mut commands,
        ctx,
    );
}

/// `OnAdd<(WhileFleeing,)>` observer: entry point for the FLEE gate.
pub fn on_while_fleeing(
    trigger: Trigger<BehaveTrigger<WhileFleeing>>,
    mut ai_q: Query<&mut AiState>,
    spine_q: Query<&SpineController>,
    particle_q: Query<&VelloParticle>,
    mut commands: Commands,
) {
    let ctx = trigger.event().ctx();
    let root = ctx.target_entity();
    let player = trigger.event().inner().player;
    handle_flee(
        root,
        player,
        &mut ai_q,
        &spine_q,
        &particle_q,
        &mut commands,
        ctx,
    );
}

/// `OnAdd<(WhileIdling,)>` observer: entry point for the IDLE gate.
pub fn on_while_idling(
    trigger: Trigger<BehaveTrigger<WhileIdling>>,
    mut ai_q: Query<&mut AiState>,
    spine_q: Query<&SpineController>,
    particle_q: Query<&VelloParticle>,
    mut commands: Commands,
) {
    let ctx = trigger.event().ctx();
    let root = ctx.target_entity();
    let player = trigger.event().inner().player;
    handle_idle(
        root,
        player,
        &mut ai_q,
        &spine_q,
        &particle_q,
        &mut commands,
        ctx,
    );
}

// ---------------------------------------------------------------------------
// Per-frame systems: travel accumulator + steering
// ---------------------------------------------------------------------------

/// Accumulates the physical distance the enemy has traveled within its current
/// phase. Resets the accumulator whenever the phase changes. Used by the
/// CHARGE gate to decide over-charge.
pub fn ai_travel_system(
    mut ai_q: Query<(&mut AiState, &SpineController)>,
    particle_q: Query<&VelloParticle>,
    time: Res<Time>,
) {
    for (mut ai, spine) in &mut ai_q {
        let Some(pos) = get_bevy_pos(&spine, &particle_q) else {
            continue;
        };

        // Phase change → reset accumulator and origin.
        if ai.phase != ai.recorded_phase {
            ai.recorded_phase = ai.phase;
            ai.travel = 0.0;
            ai.idle_timer = 0.0;
            ai.last_pos = None;
        }

        match ai.phase {
            AiPhase::Charge | AiPhase::Flee => {
                if let Some(last) = ai.last_pos {
                    ai.travel += (pos - last).length();
                }
                ai.last_pos = Some(pos);
            }
            AiPhase::Idle => {
                // Standing still — no travel accumulation, but keep clocking
                // the idle timer so the enemy eventually re-engages on its own.
                ai.idle_timer += time.delta_secs();
                if ai.idle_timer >= IDLE_DURATION {
                    // Self-driven re-engage: flip back to CHARGE. The tree's
                    // phase gates re-read `phase` from scratch every frame (the
                    // `SequenceFlow` re-ticks its first child each tick), so the
                    // while-charging gate picks this up and drives the enemy
                    // toward the player again — even if the player stands still.
                    ai.phase = AiPhase::Charge;
                    ai.idle_timer = 0.0;
                }
                ai.last_pos = None;
            }
        }
    }
}

/// Drives `SpineController.move_vector` each frame for all AI-controlled
/// characters, applying chase/flee direction and wall avoidance.
///
/// Particle positions (P3 = spine base) are read from `VelloParticle` in
/// vello y-down space and converted to bevy y-up.
///
/// A single `Query<&mut SpineController>` is used for both the player's and
/// the enemies' spines (readable via `Query::get`), avoiding the B0001
/// read/write conflict of two separate `SpineController` queries.
pub fn ai_steer_system(
    ai_q: Query<(Entity, Option<&AiChaseTarget>, Option<&AiFleeTarget>), With<AiState>>,
    particle_q: Query<&VelloParticle>,
    mut all_spines: Query<&mut SpineController>,
) {
    for (enemy, chase_target, flee_target) in &ai_q {
        // No active steering marker (e.g. IDLE phase): the character must stop.
        // Zeroing here guarantees a stale flee `move_vector` can't keep pushing
        // the enemy into a wall after the AI gate flips to IDLE.
        let (player, toward_player) = if let Some(chase) = chase_target {
            (chase.player, true)
        } else if let Some(flee) = flee_target {
            (flee.player, false)
        } else {
            if let Ok(mut spine) = all_spines.get_mut(enemy) {
                spine.move_vector = Vec2::ZERO;
            }
            continue;
        };

        // Read positions first while the mutable spine borrow is not held.
        let Some(enemy_pos) = all_spines
            .get(enemy)
            .ok()
            .and_then(|spine| get_bevy_pos(&spine, &particle_q))
        else {
            continue;
        };

        // Read the player's P3 position (player spine is separate from the
        // enemies' spines, so reading it here while holding the enemy's spine
        // borrow is fine).
        let Some(player_pos) = all_spines
            .get(player)
            .ok()
            .and_then(|spine| get_bevy_pos(&spine, &particle_q))
        else {
            continue;
        };

        let dir = if toward_player {
            (player_pos - enemy_pos).normalize_or_zero()
        } else {
            (enemy_pos - player_pos).normalize_or_zero()
        };

        let avoid = compute_wall_avoidance(enemy_pos);

        // Flee by heading to the safe anchor currently farthest from the
        // player, so the enemy never aims into a wall.
        let flee_dir = choose_flee_direction(enemy_pos, player_pos, avoid);

        let Ok(mut spine) = all_spines.get_mut(enemy) else {
            continue;
        };
        spine.move_vector = if toward_player {
            dir * AI_SPEED + avoid * AVOID_FORCE
        } else {
            flee_dir * AI_SPEED + avoid * AVOID_FORCE
        };
    }
}

// ---------------------------------------------------------------------------
// Position / facing helpers
// ---------------------------------------------------------------------------

/// Read a spine particle's world position (bevy y-up) by index.
///
/// `particles[0]` is the head (PH), `particles[4]` is the base (P3). Converts
/// from vello y-down to bevy y-up.
fn get_part_pos(
    spine: &SpineController,
    idx: usize,
    particle_q: &Query<&VelloParticle>,
) -> Option<Vec2> {
    let entity = spine.particles.get(idx)?;
    let particle = particle_q.get(*entity).ok()?;
    let vello = particle.particle.pos;
    Some(Vec2::new(vello.x, -vello.y))
}

/// Read the P3 (tail/base) particle position from an entity's
/// [`SpineController`], converting from vello y-down to bevy y-up.
fn get_bevy_pos(spine: &SpineController, particle_q: &Query<&VelloParticle>) -> Option<Vec2> {
    get_part_pos(spine, 4, particle_q)
}

/// The enemy's facing direction (head → base), in bevy y-up space.
fn enemy_facing(
    root: Entity,
    spine_q: &Query<&SpineController>,
    particle_q: &Query<&VelloParticle>,
) -> Option<Vec2> {
    let spine = spine_q.get(root).ok()?;
    let head = get_part_pos(&spine, 0, particle_q)?;
    let base = get_part_pos(&spine, 4, particle_q)?;
    Some((head - base).normalize_or_zero())
}

/// Read the P3 position of the player entity.
fn get_player_bevy_pos(
    player: Entity,
    spine_q: &Query<&SpineController>,
    particle_q: &Query<&VelloParticle>,
) -> Option<Vec2> {
    let spine = spine_q.get(player).ok()?;
    get_bevy_pos(&spine, particle_q)
}

// ---------------------------------------------------------------------------
// Wall avoidance
// ---------------------------------------------------------------------------

/// Computes a repulsion force that pushes `pos` away from the arena edges.
fn compute_wall_avoidance(pos: Vec2) -> Vec2 {
    let mut avoid = Vec2::ZERO;

    if pos.x < ARENA_MIN_X + AVOID_MARGIN {
        avoid.x = 1.0 - (pos.x - ARENA_MIN_X) / AVOID_MARGIN;
    } else if pos.x > ARENA_MAX_X - AVOID_MARGIN {
        avoid.x = -(1.0 - (ARENA_MAX_X - pos.x) / AVOID_MARGIN);
    }

    if pos.y < ARENA_MIN_Y + AVOID_MARGIN {
        avoid.y = 1.0 - (pos.y - ARENA_MIN_Y) / AVOID_MARGIN;
    } else if pos.y > ARENA_MAX_Y - AVOID_MARGIN {
        avoid.y = -(1.0 - (ARENA_MAX_Y - pos.y) / AVOID_MARGIN);
    }

    avoid
}

/// Lightweight "nav graph" of safe retreat anchors, placed on the arena
/// interior (outside the wall-avoidance margin) for each side and corner.
///
/// Fleeing steers toward whichever anchor is currently farthest from the
/// player. Because anchors are always safely inside the arena, the enemy's flee
/// heading can never aim into — and get pinned against — a wall. This is the
/// cheap box-only analog of a full navmesh; it generalizes later to an actual
/// node graph if the scene grows interior obstacles.
fn retreat_anchors() -> [Vec2; 4] {
    let m = AVOID_MARGIN * 2.0;
    [
        Vec2::new(0.0, ARENA_MAX_Y - m), // top
        Vec2::new(0.0, ARENA_MIN_Y + m), // bottom
        Vec2::new(ARENA_MIN_X + m, 0.0), // left
        Vec2::new(ARENA_MAX_X - m, 0.0), // right
    ]
}

/// Picks the fleet direction for the current frame.
///
/// Chooses the retreat anchor farthest from the player (growing separation) and
/// steers toward it. If the enemy is already closer to a wall than the chosen
/// anchor's heading would allow, the reactive [`compute_wall_avoidance`] force
/// still peels it off — the anchor handles the general case, avoidance only the
/// transient tail when the anchor just flipped sides.
fn choose_flee_direction(enemy_pos: Vec2, player_pos: Vec2, avoid: Vec2) -> Vec2 {
    let mut best = Vec2::ZERO;
    let mut best_dist = -1.0_f32;
    for anchor in retreat_anchors() {
        let dist = anchor.distance(player_pos);
        if dist > best_dist {
            best_dist = dist;
            best = anchor;
        }
    }

    let dir = (best - enemy_pos).normalize_or_zero();
    // If we're already *at* the anchor, rely on the avoidance force to nudge us
    // back into open space rather than oscillating around a single point.
    if dir.length_squared() < 1e-6 {
        return avoid.normalize_or_zero();
    }
    dir
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Registers the behaviour-tree plugin, AI observers, and the per-frame
/// steering/travel systems.
pub struct AiPlugin;

impl Plugin for AiPlugin {
    fn build(&self, app: &mut App) {
        // Tick behaviour trees (BehavePlugin default schedule).
        app.add_plugins(BehavePlugin::default());

        // Observers for the phase condition triggers.
        app.add_observer(on_while_charging);
        app.add_observer(on_while_fleeing);
        app.add_observer(on_while_idling);

        // Per-frame systems: travel accumulation + steering run in Update so
        // they write move_vector before update_character_movement (PostUpdate).
        app.add_systems(Update, (ai_travel_system, ai_steer_system).chain());
    }
}

// ---------------------------------------------------------------------------
// Tree builder
// ---------------------------------------------------------------------------

/// Builds the behaviour tree for an enemy character.
///
/// The tree is a `CHARGE → FLEE → IDLE → CHARGE` loop with **no `Wait` nodes**.
/// Each phase is a single-child `While(trigger(cond))` gate wrapped in
/// `Invert`:
///
/// ```text
/// Forever →
///   Sequence →
///     Invert( While( trigger(WhileCharging) ) )   // block while charging
///     Invert( While( trigger(WhileFleeing)  ) )   // block while fleeing
///     Invert( While( trigger(WhileIdling)   ) )   // block while idling
/// ```
///
/// A `While` holds the phase while its condition reports `success()` (loop) and
/// reports `failure()` when the phase is complete. The outer `Invert` turns
/// that *phase-complete* `failure` into a `success` so the parent `Sequence`
/// advances to the next gate.
pub fn build_enemy_ai_tree(player: Entity) -> Tree<Behave> {
    tree! {
        Behave::Forever => {
            Behave::Sequence => {
                Behave::Invert => {
                    Behave::While => {
                        Behave::trigger(WhileCharging { player })
                    }
                },
                Behave::Invert => {
                    Behave::While => {
                        Behave::trigger(WhileFleeing { player })
                    }
                },
                Behave::Invert => {
                    Behave::While => {
                        Behave::trigger(WhileIdling { player })
                    }
                },
            }
        }
    }
}
