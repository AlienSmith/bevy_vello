//! AI behavior for enemy characters using `bevy_behave` behavior trees.
//!
//! The AI alternates between chasing and fleeing the player with random
//! durations (1–4 s), applying soft wall avoidance inside the arena.
//!
//! # Architecture
//!
//! - **Behavior tree** (`Forever → Sequence → [Chase, Wait, Flee, Wait]`)
//!   handles phase timing via `Wait` nodes.
//! - **`OnAdd`/`OnRemove` observers** fire once when `Chase`/`Flee` task
//!   entities are spawned/despawned, inserting/removing marker components
//!   on the character root.
//! - **`ai_steer_system`** (per-frame, `Update`) reads the markers, reads
//!   actual physics particle positions (`VelloParticle` on P3), computes
//!   direction + wall avoidance, and writes `SpineController.move_vector`.

use bevy::prelude::*;
use bevy_behave::prelude::*;
use rand::Rng;

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

/// Arena safe area (bevy y-up). Walls are at ±960 x / ±540 y.
const ARENA_MIN_X: f32 = -920.0;
const ARENA_MAX_X: f32 = 920.0;
const ARENA_MIN_Y: f32 = -500.0;
const ARENA_MAX_Y: f32 = 500.0;

// ---------------------------------------------------------------------------
// Behaviour-tree task components (spawned on task child entities)
// ---------------------------------------------------------------------------

/// Task component: tells the AI to chase the player.
///
/// When the tree spawns this on a task child entity, the [`on_add_chase`]
/// observer inserts [`AiChaseTarget`] on the character root.
/// When the tree despawns it, [`on_remove_chase`] cleans up.
#[derive(Component, Clone)]
pub struct Chase {
    /// The player entity to chase.
    pub player: Entity,
}

/// Task component: tells the AI to flee from the player.
#[derive(Component, Clone)]
pub struct Flee {
    /// The player entity to flee from.
    pub player: Entity,
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
// Observers (fire once on task entity when Chase/Flee is added/removed)
// ---------------------------------------------------------------------------

/// `OnAdd<Chase>` observer: inserts [`AiChaseTarget`] on the character root.
pub fn on_add_chase(
    trigger: Trigger<OnAdd, Chase>,
    q: Query<&Chase>,
    ctx_q: Query<&BehaveCtx>,
    mut commands: Commands,
) {
    let Ok(chase) = q.get(trigger.target()) else {
        return;
    };
    let Ok(ctx) = ctx_q.get(trigger.target()) else {
        return;
    };
    let character_root = ctx.target_entity();
    commands.entity(character_root).insert(AiChaseTarget {
        player: chase.player,
    });
}

/// `OnRemove<Chase>` observer: removes [`AiChaseTarget`] and zeroes
/// `move_vector`.
pub fn on_remove_chase(
    trigger: Trigger<OnRemove, Chase>,
    ctx_q: Query<&BehaveCtx>,
    mut spine_q: Query<&mut SpineController>,
    mut commands: Commands,
) {
    let Ok(ctx) = ctx_q.get(trigger.target()) else {
        return;
    };
    let character_root = ctx.target_entity();
    commands.entity(character_root).remove::<AiChaseTarget>();
    if let Ok(mut spine) = spine_q.get_mut(character_root) {
        spine.move_vector = Vec2::ZERO;
    }
}

/// `OnAdd<Flee>` observer: inserts [`AiFleeTarget`] on the character root.
pub fn on_add_flee(
    trigger: Trigger<OnAdd, Flee>,
    q: Query<&Flee>,
    ctx_q: Query<&BehaveCtx>,
    mut commands: Commands,
) {
    let Ok(flee) = q.get(trigger.target()) else {
        return;
    };
    let Ok(ctx) = ctx_q.get(trigger.target()) else {
        return;
    };
    let character_root = ctx.target_entity();
    commands.entity(character_root).insert(AiFleeTarget {
        player: flee.player,
    });
}

/// `OnRemove<Flee>` observer: removes [`AiFleeTarget`] and zeroes
/// `move_vector`.
pub fn on_remove_flee(
    trigger: Trigger<OnRemove, Flee>,
    ctx_q: Query<&BehaveCtx>,
    mut spine_q: Query<&mut SpineController>,
    mut commands: Commands,
) {
    let Ok(ctx) = ctx_q.get(trigger.target()) else {
        return;
    };
    let character_root = ctx.target_entity();
    commands.entity(character_root).remove::<AiFleeTarget>();
    if let Ok(mut spine) = spine_q.get_mut(character_root) {
        spine.move_vector = Vec2::ZERO;
    }
}

// ---------------------------------------------------------------------------
// Per-frame system: reads markers + particle positions, writes move_vector
// ---------------------------------------------------------------------------

/// Drives `SpineController.move_vector` each frame for all AI-controlled
/// characters, applying chase/flee direction and wall avoidance.
///
/// Particle positions (P3 = spine base) are read from `VelloParticle` in
/// vello y-down space and converted to bevy y-up.
pub fn ai_steer_system(
    ai_q: Query<
        (Entity, Option<&AiChaseTarget>, Option<&AiFleeTarget>),
        Or<(With<AiChaseTarget>, With<AiFleeTarget>)>,
    >,
    particle_q: Query<&VelloParticle>,
    player_spine_q: Query<&SpineController>,
    mut all_spines: Query<&mut SpineController>,
) {
    for (enemy, chase_target, flee_target) in &ai_q {
        let Ok(mut spine) = all_spines.get_mut(enemy) else {
            continue;
        };

        let Some(enemy_pos) = get_bevy_pos(&spine, &particle_q) else {
            continue;
        };

        // Determine player entity and whether we chase (true) or flee (false).
        let (player, toward_player) = if let Some(chase) = chase_target {
            (chase.player, true)
        } else if let Some(flee) = flee_target {
            (flee.player, false)
        } else {
            continue;
        };

        let Some(player_pos) = get_player_bevy_pos(player, &player_spine_q, &particle_q) else {
            continue;
        };

        let dir = if toward_player {
            (player_pos - enemy_pos).normalize_or_zero()
        } else {
            (enemy_pos - player_pos).normalize_or_zero()
        };

        let avoid = compute_wall_avoidance(enemy_pos);
        spine.move_vector = dir * AI_SPEED + avoid * AVOID_FORCE;
    }
}

/// Read the P3 particle position from an entity's [`SpineController`],
/// converting from vello y-down to bevy y-up.
fn get_bevy_pos(spine: &SpineController, particle_q: &Query<&VelloParticle>) -> Option<Vec2> {
    let p3_entity = spine.particles.get(4)?;
    let particle = particle_q.get(*p3_entity).ok()?;
    let vello = particle.particle.pos;
    Some(Vec2::new(vello.x, -vello.y))
}

/// Read the P3 position of the player entity.
fn get_player_bevy_pos(
    player: Entity,
    player_spine_q: &Query<&SpineController>,
    particle_q: &Query<&VelloParticle>,
) -> Option<Vec2> {
    let spine = player_spine_q.get(player).ok()?;
    get_bevy_pos(spine, particle_q)
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

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Registers the behaviour-tree plugin, AI observers, and the per-frame
/// steering system.
pub struct AiPlugin;

impl Plugin for AiPlugin {
    fn build(&self, app: &mut App) {
        // Tick behaviour trees in FixedPreUpdate (BehavePlugin default).
        app.add_plugins(BehavePlugin::default());

        // Observers for Chase/Flee lifecycle.
        app.add_observer(on_add_chase);
        app.add_observer(on_remove_chase);
        app.add_observer(on_add_flee);
        app.add_observer(on_remove_flee);

        // Per-frame steering system: runs in Update so it writes
        // move_vector before update_character_movement (PostUpdate).
        app.add_systems(Update, ai_steer_system);
    }
}

// ---------------------------------------------------------------------------
// Tree builder
// ---------------------------------------------------------------------------

/// Builds the behaviour tree for an enemy character.
///
/// Random chase/flee durations are baked in at construction time
/// (1–4 seconds each).
pub fn build_enemy_ai_tree(player: Entity) -> Tree<Behave> {
    let mut rng = rand::thread_rng();
    let chase_dur: f32 = rng.gen::<f32>() * 3.0 + 1.0;
    let flee_dur: f32 = rng.gen::<f32>() * 3.0 + 1.0;

    tree! {
        Behave::Forever => {
            Behave::Sequence => {
                Behave::spawn_named("Chase", Chase { player }),
                Behave::Wait(chase_dur),
                Behave::spawn_named("Flee", Flee { player }),
                Behave::Wait(flee_dur),
            }
        }
    }
}
