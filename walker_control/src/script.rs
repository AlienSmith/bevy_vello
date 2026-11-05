//! The **command track** — a JSON script that drives the state machine.
//!
//! A script is a flat list of timed cells; the track is a playhead over them.
//! Each tick it hands the machine one [`Command`] (a held move) or `None`
//! (no active command — which is what ends a run and returns the character to
//! standing). The machine ([`crate::control::LocoState`]) decides which
//! controller runs; the track only decides *what is commanded*:
//!
//! ```json
//! { "script": [
//!   { "stand": { "dur": 6.0 } }
//! ] }
//! ```
//!
//! - `stand` — no command for `dur` seconds: the machine sits in Standing and
//!   the character holds the rigid rest pose. This is the only cell that is
//!   meaningful today, because `WalkingCtrl` and `BoostStartCtrl` are still
//!   [`crate::control::hold_pose`] stubs.
//! - `move` — (schema ready, controller pending) hold a plain gait command.
//! - `boost` — (schema ready, controller pending) hold a **launch** command:
//!   the machine runs it through [`crate::control::LocoMode::BoostStart`]
//!   until the *measured* speed clears the exit threshold.
//!
//! The playhead is tick-clocked (plan Law 3): it advances by the fixed-step
//! `dt`, exactly like the walker chain it feeds. A finished track keeps
//! returning `None`, so every run ends parked in Standing.

use glam::Vec2;
use serde::Deserialize;

use crate::control::Command;

/// A `move` / `boost` cell: dir × speed held for `dur` seconds.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct MoveArgs {
    /// Travel direction (only `x` matters on this model).
    pub dir: [f32; 2],
    /// Commanded speed magnitude, px/s.
    #[serde(default)]
    pub speed: f32,
    /// Hold duration, seconds.
    pub dur: f32,
}

/// A `stand` cell: no command, `dur` seconds.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct StandArgs {
    pub dur: f32,
}

/// One script cell. Externally tagged, matching the harness's existing
/// `command.rs` style: `{ "stand": { .. } }`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Cell {
    /// No command: the machine stands (and ends any prior move command).
    Stand(StandArgs),
    Move(MoveArgs),
    /// A move that launches through BoostStart (the `boost` verb).
    Boost(MoveArgs),
}

impl Cell {
    /// How long the playhead dwells on this cell.
    pub fn dur(&self) -> f32 {
        match self {
            Cell::Stand(a) => a.dur,
            Cell::Move(a) | Cell::Boost(a) => a.dur,
        }
    }

    /// The command this cell holds while active (`None` = no command).
    pub fn command(&self) -> Option<Command> {
        match self {
            Cell::Stand(_) => None,
            Cell::Move(a) => Some(Command {
                dir: Vec2::new(a.dir[0], a.dir[1]),
                speed: a.speed,
                boost: false,
            }),
            Cell::Boost(a) => Some(Command {
                dir: Vec2::new(a.dir[0], a.dir[1]),
                speed: a.speed,
                boost: true,
            }),
        }
    }

    /// Kind label (for logs/telemetry symmetry with the old harness).
    pub fn kind(&self) -> &'static str {
        match self {
            Cell::Stand(_) => "stand",
            Cell::Move(_) => "move",
            Cell::Boost(_) => "boost",
        }
    }
}

/// The parsed file shape: `{ "script": [ ... ] }`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ScriptFile {
    pub script: Vec<Cell>,
}

/// The playhead: an ordered [`Cell`] list plus the active index and the tick
/// count spent in it. Lives as a Bevy resource; the harness system advances it
/// once per fixed tick and writes the active command onto every
/// [`crate::walker::Walker`] — exactly like keyboard intent, but scripted, so
/// runs are deterministic and replayable.
///
/// A cell of `dur` seconds covers `ceil(dur · rate)` ticks, where `rate` is
/// derived from the first non-zero `dt` the track sees. Rollover is an
/// **integer** tick count, not a float time accumulator, so `1/90`-second
/// boundaries never drift with float rounding.
#[derive(Debug, Clone, Default)]
pub struct ScriptTrack {
    cells: Vec<Cell>,
    index: usize,
    ticks_in_cell: u64,
    ticks_per_sec: u64,
}

impl ScriptTrack {
    /// Parse a JSON track from a file. Keeps the path in the error so the
    /// harness can print the reason without re-reading.
    pub fn from_path(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
        Self::from_json(&text).map_err(|e| format!("{path}: {e}"))
    }

    /// Parse from JSON text (unit-testable without a filesystem).
    pub fn from_json(text: &str) -> Result<Self, String> {
        let file: ScriptFile =
            serde_json::from_str(text).map_err(|e| format!("parse failed: {e}"))?;
        if file.script.is_empty() {
            return Err("script is empty".to_string());
        }
        Ok(Self::from_cells(file.script))
    }

    /// A track from already-built cells (tests, future AI planners).
    pub fn from_cells(cells: Vec<Cell>) -> Self {
        Self {
            cells,
            index: 0,
            ticks_in_cell: 0,
            ticks_per_sec: 0,
        }
    }

    pub fn done(&self) -> bool {
        self.index >= self.cells.len()
    }

    /// The active cell's command *before* advancing (idempotent read).
    pub fn peek(&self) -> Option<Command> {
        self.cells.get(self.index).and_then(|c| c.command())
    }

    /// The active cell's kind label (`"done"` after the script finishes).
    pub fn kind(&self) -> &'static str {
        self.cells
            .get(self.index)
            .map(|c| c.kind())
            .unwrap_or("done")
    }

    /// Advance the playhead by one fixed tick of `dt` and return the command
    /// to hold this tick. The command returned is the cell that was active
    /// *during* this tick; rollover happens after it, so a cell of `dur`
    /// seconds covers `ceil(dur · rate)` ticks. When the script finishes,
    /// every further tick returns `None` — the "command end" that parks the
    /// machine in Standing.
    pub fn tick(&mut self, dt: f32) -> Option<Command> {
        if dt <= 0.0 {
            return self.peek();
        }
        // Learn the tick rate from the first real tick (the fixed clock is
        // constant, so one measurement is exact).
        if self.ticks_per_sec == 0 {
            self.ticks_per_sec = (1.0 / dt).round().max(1.0) as u64;
        }
        // Rollover on ENTRY: this tick belongs to the cell that still has
        // room for it. A cell of `dur` s covers exactly `ceil(dur·rate)`
        // ticks; a zero-dur cell is instantaneous (occupies no tick, so it
        // can never leak a phantom one).
        let mut budget = self.ticks_in_cell + 1;
        while !self.done() {
            let span = self.span_ticks(self.index);
            if budget <= span {
                break;
            }
            budget -= span;
            self.index += 1;
        }
        self.ticks_in_cell = if self.done() { 0 } else { budget };
        self.peek()
    }

    /// Cell `index`'s span in ticks: `ceil(dur · rate)`.
    fn span_ticks(&self, index: usize) -> u64 {
        (self.cells[index].dur() * self.ticks_per_sec as f32)
            .ceil()
            .max(0.0) as u64
    }

    /// Rewind to the first cell (used between harness cells / replays).
    pub fn rewind(&mut self) {
        self.index = 0;
        self.ticks_in_cell = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"{
        "script": [
            { "boost": { "dir": [1.0, 0.0], "speed": 120.0, "dur": 2.0 } },
            { "stand": { "dur": 1.0 } },
            { "move":  { "dir": [-1.0, 0.0], "speed": 90.0, "dur": 4.0 } }
        ]
    }"#;

    #[test]
    fn json_tracks_the_three_cell_kinds() {
        let t = ScriptTrack::from_json(SAMPLE).expect("sample parses");
        assert_eq!(t.cells.len(), 3);
        assert!(matches!(t.cells[0], Cell::Boost(_)));
        assert!(matches!(t.cells[1], Cell::Stand(_)));
        assert_eq!(t.cells[1].command(), None, "stand means no command");
        assert_eq!(
            t.cells[2].command(),
            Some(Command {
                dir: Vec2::new(-1.0, 0.0),
                speed: 90.0,
                boost: false
            })
        );
    }

    #[test]
    fn the_boost_cell_holds_a_boost_command() {
        let t = ScriptTrack::from_json(SAMPLE).unwrap();
        let c = t.cells[0].command().unwrap();
        assert!(c.boost);
        assert_eq!(c.vx(), 120.0);
    }

    #[test]
    fn a_standing_script_is_the_only_sane_default_today() {
        // Only StandingCtrl is implemented; a stand-only track must simply
        // never produce a command, in any length of run.
        let mut t =
            ScriptTrack::from_json(r#"{ "script": [ { "stand": { "dur": 10.0 } } ] }"#).unwrap();
        for _ in 0..900 {
            assert_eq!(t.tick(1.0 / 90.0), None);
        }
    }

    #[test]
    fn playhead_advances_by_dur_then_goes_quiet() {
        let mut t = ScriptTrack::from_json(SAMPLE).unwrap();
        let dt = 1.0 / 90.0;
        // Boost cell: 2 s ⇒ exactly 180 held ticks.
        for _ in 0..180 {
            assert_eq!(
                t.tick(dt).map(|c| c.boost),
                Some(true),
                "boost held for its full duration"
            );
        }
        // Stand cell: 1 s of None.
        for _ in 0..90 {
            assert_eq!(t.tick(dt), None, "stand means no command");
        }
        // Move cell: 4 s of a plain move.
        for _ in 0..360 {
            let c = t.tick(dt).expect("move active");
            assert!(!c.boost);
            assert_eq!(c.vx(), -90.0);
        }
        // Finished: quiet forever (the command-end that parks Standing).
        for _ in 0..90 {
            assert_eq!(t.tick(dt), None);
        }
        assert!(t.done());
    }

    #[test]
    fn zero_dur_cells_are_instantaneous() {
        let mut t = ScriptTrack::from_cells(vec![
            Cell::Stand(StandArgs { dur: 0.0 }),
            Cell::Move(MoveArgs {
                dir: [1.0, 0.0],
                speed: 50.0,
                dur: 1.0,
            }),
        ]);
        // A zero-dur stand occupies no tick: the move owns the very first one.
        for _ in 0..90 {
            let c = t.tick(1.0 / 90.0).expect("move is active");
            assert_eq!(c.speed, 50.0);
        }
        // Then the playhead finishes and never looks back.
        assert_eq!(t.tick(1.0 / 90.0), None);
    }

    #[test]
    fn empty_or_broken_scripts_are_rejected() {
        assert!(ScriptTrack::from_json(r#"{ "script": [] }"#).is_err());
        assert!(ScriptTrack::from_json(r#"{ "script": [ { "flee": {} } ] }"#).is_err());
    }

    #[test]
    fn speed_defaults_to_zero_when_omitted() {
        let t = ScriptTrack::from_json(
            r#"{ "script": [ { "move": { "dir": [1.0, 0.0], "dur": 1.0 } } ] }"#,
        )
        .unwrap();
        assert_eq!(t.cells[0].command().unwrap().speed, 0.0);
    }

    #[test]
    fn rewind_replays_the_track() {
        let mut t = ScriptTrack::from_json(SAMPLE).unwrap();
        // 2 s + 1 s + 4 s = 630 ticks, +1 until the playhead flips to done
        // (rollover happens on the next tick's entry).
        for _ in 0..631 {
            t.tick(1.0 / 90.0);
        }
        assert!(t.done());
        t.rewind();
        assert_eq!(t.tick(1.0 / 90.0).map(|c| c.boost), Some(true));
    }
}
