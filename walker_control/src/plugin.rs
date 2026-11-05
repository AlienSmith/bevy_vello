//! The **plugin** — attach a [`Walker`] to a character root and run the
//! observe → control → actuate chain on the fixed physics clock.
//!
//! ```text
//! AttachWalker (event, triggered on the character root)
//!         │
//!         └─ on_attach_walker  ⇒  insert (Walker, WalkerRig)
//!
//! FixedUpdate, **before the physics step** (`update_constraint_world`):
//!   Input      intent_system     host input → Walker.command
//!   Observe    observe_system    fiction + reality block of telemetry
//!   Control    control_system    ControlOut + command block of telemetry
//!   Actuate    actuate_system    constraint queues + knee columns
//!   Assemble   assemble_system   the only formatter, in sorted-entity order
//! ```
//!
//! The whole set is anchored `.before(update_constraint_world)`: `bevy_vello`'s
//! physics plugin runs `make_collision_constraints → update_constraint_world →
//! …` as one `FixedUpdate` chain, and `update_constraint_world` consumes the
//! queued [`crate::emit`] events in the *same* tick. Observe reads the particle
//! positions before they move and last tick's [`CollisionEventBatch`] — the
//! intended 1-tick detect→respond lag — so the chain sits ahead of the solver.
//!
//! Four constraints shape this file:
//!
//! 1. **Entity handles ride the event, not a component query.**
//!    `walker_control` must not name `game_lib`'s `LeftLegController` /
//!    `RightLegController` / spine components, so it cannot look the handles up
//!    itself. The spawner already has them; it puts them in [`AttachWalker`].
//!    That keeps `game_lib → walker_control` a one-way dependency.
//! 2. **One writer per telemetry block.** Each set writes only its own columns,
//!    and `assemble_system` is the only formatter, so no stage can print a
//!    value it did not produce.
//! 3. **Determinism under multiple characters.** `assemble_system` visits
//!    walkers in sorted `Entity` order, so row order is stable across runs.
//! 4. **Attach is an event, not a plugin-wide scan.** A character whose legs
//!    have not finished spawning has nothing to attach yet; the factory decides
//!    when to fire.

use bevy::prelude::*;
use bevy::time::Fixed;
use bevy_vello::collision::CollisionEventBatch;
use bevy_vello::integrations::physics::systems::update_constraint_world;
use bevy_vello::integrations::physics::{VelloConstraintWorld, VelloParticle};
use bevy_vello::VelloCollider;

use crate::emit::LegHandles;
use crate::observe::ObserveHandles;
use crate::script::ScriptTrack;
use crate::telemetry::legacy_header;
use crate::walker::{actuate_stage, control_stage, observe_stage, Walker, WalkerConfig};

/// Spawn-time payload for one character. Trigger it on the character root.
///
/// The observer/leg handles are copied from the game-side controllers that
/// already hold them (constraint 1 above).
#[derive(Event, Debug, Clone)]
pub struct AttachWalker {
    /// All tuning, typed. Clone it from the character's blueprint.
    pub config: WalkerConfig,
    /// `+1` = facing `+x`. Spawn-time constant, not a per-tick command.
    pub facing: f32,
    /// Hip/waist/foot particles plus foot colliders (contact attribution).
    pub handles: ObserveHandles,
    /// Left leg: `[P2, P3, knee, foot]` particles, both joints, optional strut.
    pub left: LegHandles,
    /// Right leg, same shape.
    pub right: LegHandles,
}

/// The entity handles a [`Walker`] needs every tick, stored beside it.
///
/// Separate from [`Walker`] so `walker.rs` stays free of handle plumbing, and
/// so a respawned character can re-point its rig without rebuilding controller
/// state.
#[derive(Component, Debug, Clone, Copy)]
pub struct WalkerRig {
    pub handles: ObserveHandles,
    pub left: LegHandles,
    pub right: LegHandles,
}

/// Desired movement for one character this tick. Written by the host (keyboard,
/// AI) and consumed by [`intent_system`], which turns it into a held
/// [`Command`]. Absent ⇒ [`intent_system`] leaves the walker alone.
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct WalkerIntent {
    /// Travel direction (only `x` matters on the loop-3 model).
    pub dir: Vec2,
    /// Desired speed, world units per second.
    pub speed: f32,
    /// Launch this move through BoostStart before the gait takes over.
    pub boost: bool,
}

/// The fixed-tick chain, in order. Observe must see the physics world before
/// control reads the fiction, and actuate must queue its constraints before the
/// XPBD chain consumes them.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WalkerControlSet {
    Input,
    Observe,
    Control,
    Actuate,
    Assemble,
}

/// The command-track resource: a JSON [`ScriptTrack`] playhead the host loads
/// at startup. When present, [`script_system`] advances it once per fixed tick
/// and writes the active command onto every [`Walker`], exactly like keyboard
/// intent — but scripted, so the harness runs deterministic, replayable moves
/// and a `wait` cell cleanly ends a command (the machine returns to Standing).
/// Absent ⇒ the walkers keep whatever the [`WalkerIntent`] path (or callers)
/// set.
#[derive(Resource, Debug)]
pub struct ScriptResource(pub ScriptTrack);

/// The formatted CSV stream. `assemble_system` appends rows; the host writes the
/// file at teardown (never per tick). The header is this crate's because only it
/// knows the walker's own columns; a host that appends particle channels extends
/// `header` before flushing.
#[derive(Resource, Debug)]
pub struct WalkerTelemetryBuffer {
    /// Comma-separated `SCALAR_COLUMNS`, built once.
    pub header: String,
    /// One CSV row per formatted walker tick, in tick-then-entity order.
    pub rows: Vec<String>,
}

impl WalkerTelemetryBuffer {
    /// The legacy-format header: the frozen scalar block plus the measured
    /// particle channels, so rows drop straight into `plot_p3.py` etc.
    pub fn new() -> Self {
        Self {
            header: legacy_header(),
            rows: Vec::new(),
        }
    }

    /// Drop the rows but keep the header (used between harness cells).
    pub fn clear_rows(&mut self) {
        self.rows.clear();
    }
}

impl Default for WalkerTelemetryBuffer {
    /// `Default` means "a usable empty buffer", not "an empty header": the
    /// header is a contract with every consumer of the CSV.
    fn default() -> Self {
        Self::new()
    }
}

/// Attach handler: insert the component pair. Falling back to the rig-free
/// defaults would be wrong — without handles there is nothing to observe.
pub fn on_attach_walker(trigger: Trigger<AttachWalker>, mut commands: Commands) {
    let event = trigger.event();
    let root = trigger.target();
    commands
        .entity(root)
        .insert(Walker::new(event.config.clone(), event.facing))
        .insert(WalkerRig {
            handles: event.handles,
            left: event.left,
            right: event.right,
        });
}

/// Advance the [`ScriptResource`] playhead and write the active command onto
/// every walker. Runs only when the host inserted a script **and** at least
/// one walker is attached — the playhead must not burn cells while the rig is
/// still assembling. The command is *held* for the cell's duration and
/// becomes `None` after the script ends (the command-end that parks the
/// machine in Standing; a done track keeps returning `None`).
pub fn script_system(
    time: Res<Time<Fixed>>,
    mut script: ResMut<ScriptResource>,
    mut walkers: Query<&mut Walker>,
) {
    let dt = time.delta_secs();
    let cmd = script.0.tick(dt);
    for mut walker in walkers.iter_mut() {
        walker.command = cmd;
    }
}

/// Turn the optional [`WalkerIntent`] into a held [`Command`]. While a
/// [`ScriptResource`] is loaded the script owns the command slot, so this
/// system no-ops (exactly one command source per tick). No intent component ⇒
/// no change, so a character with no controller still holds its last command.
pub fn intent_system(
    script: Option<Res<ScriptResource>>,
    mut walkers: Query<(&mut Walker, Option<&mut WalkerIntent>)>,
) {
    use crate::control::Command;
    if script.is_some() {
        return;
    }
    for (mut walker, intent) in walkers.iter_mut() {
        if let Some(intent) = intent {
            if intent.dir.length() > 0.0 || intent.speed > 0.0 {
                walker.command = Some(Command {
                    dir: intent.dir,
                    speed: intent.speed,
                    boost: intent.boost,
                });
            } else {
                walker.command = None;
            }
        }
    }
}

/// Observe stage for every walker.
#[allow(clippy::too_many_arguments)]
pub fn observe_system(
    time: Res<Time<Fixed>>,
    batch: Option<Res<CollisionEventBatch>>,
    particles: Query<&VelloParticle>,
    colliders: Query<&VelloCollider>,
    mut walkers: Query<(Entity, &mut Walker, &WalkerRig)>,
) {
    let dt = time.delta_secs();
    for (entity, mut walker, rig) in walkers.iter_mut() {
        observe_stage(
            &mut walker,
            &rig.handles,
            entity,
            &particles,
            &colliders,
            batch.as_deref(),
            dt,
        );
    }
}

/// Control stage for every walker. The dt comes from the fiction the observe
/// stage just stamped, so the two stages can never disagree about the tick.
pub fn control_system(mut walkers: Query<&mut Walker>) {
    for mut walker in walkers.iter_mut() {
        control_stage(&mut walker);
    }
}

/// Actuate stage for every walker. Queues only angular + distance constraints
/// (plan Law 1) and records the knee targets it aimed at.
pub fn actuate_system(
    particles: Query<&VelloParticle>,
    mut world: ResMut<VelloConstraintWorld>,
    mut walkers: Query<(Entity, &mut Walker, &WalkerRig)>,
) {
    for (entity, mut walker, rig) in walkers.iter_mut() {
        actuate_stage(
            &mut walker,
            entity,
            &rig.left,
            &rig.right,
            &particles,
            &mut world,
        );
    }
}

/// The only formatter: one row per walker per tick, entities sorted so a
/// multi-character scene has a stable row order.
pub fn assemble_system(
    time: Res<Time<Fixed>>,
    mut buffer: ResMut<WalkerTelemetryBuffer>,
    mut walkers: Query<(Entity, &mut Walker)>,
) {
    let dt = time.delta_secs();
    let mut ordered: Vec<(Entity, Mut<Walker>)> = walkers.iter_mut().collect();
    ordered.sort_by_key(|(entity, _)| *entity);

    for (_, mut walker) in ordered {
        let pairs = walker.telemetry.format(dt);
        let row: Vec<&str> = pairs.iter().map(|(_, value)| value.as_str()).collect();
        buffer.rows.push(row.join(","));
    }
}

/// The plugin. Hosts install the fixed clock (`Time<Fixed>` at the physics rate)
/// themselves, exactly as `LowerBodySandboxPlugin` does.
pub struct WalkerControlPlugin;

impl Plugin for WalkerControlPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<WalkerTelemetryBuffer>()
            .add_observer(on_attach_walker)
            .configure_sets(
                FixedUpdate,
                (
                    WalkerControlSet::Input,
                    WalkerControlSet::Observe,
                    WalkerControlSet::Control,
                    WalkerControlSet::Actuate,
                    WalkerControlSet::Assemble,
                )
                    // Whole pipeline ahead of the solver. `update_constraint_world`
                    // is the one system that consumes the queued
                    // `CharacterAngularConstraintEvent` /
                    // `CharacterDistanceConstraintEvent`; anchoring here (not on
                    // `add_systems`) means the order is a property of the sets, so
                    // any system a host adds into a set inherits it. Mirrors the
                    // host idiom `drive_hip_pin.before(update_constraint_world)`.
                    .chain()
                    .before(update_constraint_world),
            )
            .add_systems(
                FixedUpdate,
                (
                    // Exactly one command source writes `Walker::command` per
                    // tick: the script track when the host loaded one, else
                    // the keyboard/AI path via `WalkerIntent` (`intent_system`
                    // no-ops while a `ScriptResource` exists).
                    (
                        script_system
                            .run_if(resource_exists::<ScriptResource>)
                            .run_if(any_with_component::<Walker>),
                        intent_system,
                    )
                        .chain()
                        .in_set(WalkerControlSet::Input),
                    observe_system.in_set(WalkerControlSet::Observe),
                    control_system.in_set(WalkerControlSet::Control),
                    actuate_system.in_set(WalkerControlSet::Actuate),
                    assemble_system.in_set(WalkerControlSet::Assemble),
                ),
            );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::Command;
    use crate::telemetry::SCALAR_COLUMNS;

    /// A walker with no rig attached must not be touched by the stage systems:
    /// `Query<(Entity, &mut Walker, &WalkerRig)>` simply skips it, which is why
    /// the plugin can be installed before any character spawns.
    #[test]
    fn header_matches_the_frozen_scalar_columns() {
        let buffer = WalkerTelemetryBuffer::new();
        assert_eq!(buffer.header, legacy_header());
        assert_eq!(
            buffer.header.split(',').count(),
            SCALAR_COLUMNS.len() + 4 * crate::telemetry::REALITY_PARTICLES.len()
        );
        assert!(buffer.rows.is_empty());
    }

    #[test]
    fn clear_rows_keeps_the_header() {
        let mut buffer = WalkerTelemetryBuffer::new();
        let header = buffer.header.clone();
        buffer.rows.push("0".to_string());
        buffer.clear_rows();
        assert!(buffer.rows.is_empty());
        assert_eq!(buffer.header, header);
    }

    #[test]
    fn the_set_chain_is_declared_in_pipeline_order() {
        // The enum's discriminants are the contract the plugin relies on; a
        // reordering here would silently break the chain's intent.
        let order = [
            WalkerControlSet::Input,
            WalkerControlSet::Observe,
            WalkerControlSet::Control,
            WalkerControlSet::Actuate,
            WalkerControlSet::Assemble,
        ];
        for (i, set) in order.iter().enumerate() {
            assert_eq!(*set as u8, i as u8, "{set:?} is at its stage index");
        }
    }

    #[test]
    fn a_walker_starts_standing_and_intentless() {
        let walker = Walker::new(WalkerConfig::default(), 1.0);
        assert!(walker.command.is_none());
        assert_eq!(walker.facing, 1.0);
        assert!(walker.out().is_none(), "no control tick has run yet");
    }

    #[test]
    fn walker_intent_launch_flag_reaches_the_command() {
        let mut w = Walker::new(WalkerConfig::default(), 1.0);
        w.command = Some(Command::boost(120.0));
        assert!(w.command.unwrap().boost);
    }
}
