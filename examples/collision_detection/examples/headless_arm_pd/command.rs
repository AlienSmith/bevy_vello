//! The command-script schema (serde JSON) + the built-in default scripts.
//!
//! The playhead is a flat, ordered list of **self-contained commands** — each
//! groups everything it needs (which limb, where to, its gains, how long to
//! hold). No phase machine, no cross-entry coupling. Commands are deserialized
//! from JSON (env `SCRIPT=path.json`); when absent a built-in script is used.
//!
//! ```json
//! { "script": [
//!   { "reach": { "limb": "right_leg", "dir": [1.0, -0.1], "dur": 1.2 } },
//!   { "wait":  { "dur": 0.6 } },
//!   { "aim":   { "limb": "right_arm", "dir": [1.0, 0.0], "dur": 1.2 } },
//!   { "set":   { "limb": "right_arm", "kp": 0.4, "kd": 0.05 } },
//!   { "reset": { "limb": "right_leg" } },
//!   { "wait":  { "dur": 0.9 } }
//! ] }
//! ```

use bevy::log::error;

use serde::Deserialize;

use game_lib::WhichLimb;

/// The four drivable limbs (also the JSON `limb` tag).
#[derive(Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[serde(rename_all = "snake_case")]
pub enum LimbId {
    RightArm,
    LeftArm,
    RightLeg,
    LeftLeg,
}

impl LimbId {
    pub fn idx(self) -> usize {
        match self {
            LimbId::RightArm => 0,
            LimbId::LeftArm => 1,
            LimbId::RightLeg => 2,
            LimbId::LeftLeg => 3,
        }
    }

    pub fn which(self) -> WhichLimb {
        match self {
            LimbId::RightArm => WhichLimb::Right,
            LimbId::LeftArm => WhichLimb::Left,
            LimbId::RightLeg => WhichLimb::RightLeg,
            LimbId::LeftLeg => WhichLimb::LeftLeg,
        }
    }

    pub fn from_env() -> Self {
        match std::env::var("LIMB").ok().as_deref() {
            Some("left_arm") => LimbId::LeftArm,
            Some("right_leg") => LimbId::RightLeg,
            Some("left_leg") => LimbId::LeftLeg,
            _ => LimbId::RightArm,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            LimbId::RightArm => "right_arm",
            LimbId::LeftArm => "left_arm",
            LimbId::RightLeg => "right_leg",
            LimbId::LeftLeg => "left_leg",
        }
    }
}

/// Position-IK reach: drive `limb` end to `to` (world Bevy y-up) or, when `to`
/// is omitted, to `anchor + normalize(dir) * dist`.
#[derive(Deserialize, Clone)]
pub struct ReachArgs {
    pub limb: LimbId,
    #[serde(default)]
    pub to: Option<[f32; 2]>,
    #[serde(default)]
    pub dir: Option<[f32; 2]>,
    #[serde(default)]
    pub dist: Option<f32>,
    #[serde(default)]
    pub kp: Option<f32>,
    #[serde(default)]
    pub kd: Option<f32>,
    pub dur: f32,
}

/// Direction-IK aim: aim `limb` end along `dir` at `dist` from the pivot.
#[derive(Deserialize, Clone)]
pub struct AimArgs {
    pub limb: LimbId,
    pub dir: [f32; 2],
    #[serde(default)]
    pub dist: Option<f32>,
    #[serde(default)]
    pub kp: Option<f32>,
    #[serde(default)]
    pub kd: Option<f32>,
    #[serde(default)]
    pub weapon_offset_y: Option<f32>,
    pub dur: f32,
}

/// Persistently set a limb's gains without moving it.
#[derive(Deserialize, Clone)]
pub struct SetArgs {
    pub limb: LimbId,
    #[serde(default)]
    pub kp: Option<f32>,
    #[serde(default)]
    pub kd: Option<f32>,
}

/// Emit a limb constraint reset.
#[derive(Deserialize, Clone)]
pub struct ResetArgs {
    pub limb: LimbId,
}

/// Hold the current command for `dur` seconds.
#[derive(Deserialize, Clone)]
pub struct WaitArgs {
    pub dur: f32,
}

/// Whole-body walk: hold a horizontal body input for `dur` seconds. Only `dir[0]`
/// (the x sign) is used — `+x` walks right, `-x` walks left — matching the
/// resolver's `input_dir.x`.
/// `speed` optionally overrides the global `target_speed` (px/s, magnitude) for
/// the duration of this command: a mid-walk speed change is just two `move`
/// entries with different `speed`. `None` = the global config speed.
#[derive(Deserialize, Clone)]
pub struct MoveArgs {
    pub dir: [f32; 2],
    #[serde(default)]
    pub speed: Option<f32>,
    pub dur: f32,
}

#[derive(Deserialize, Clone)]
pub struct JumpArgs {
    #[serde(default = "default_jump_dur")]
    pub dur: f32,
    /// Optional held movement during the jump, matching how the app is actually
    /// driven (move key held while jump is tapped). `None` (or `[0,0]`) releases
    /// the move key for the jump duration — the legacy behaviour.
    #[serde(default)]
    pub dir: Option<[f32; 2]>,
}

fn default_jump_dur() -> f32 {
    0.8
}

/// One script command. Externally tagged: `{ "reach": { .. } }`.
#[derive(Deserialize, Clone)]
#[serde(rename_all = "snake_case")]
pub enum Command {
    Reach(ReachArgs),
    Aim(AimArgs),
    Set(SetArgs),
    Reset(ResetArgs),
    Wait(WaitArgs),
    Move(MoveArgs),
    Jump(JumpArgs),
}

impl Command {
    /// Kind label for the CSV `kind` column.
    pub fn kind(&self) -> &'static str {
        match self {
            Command::Reach(_) => "reach",
            Command::Aim(_) => "aim",
            Command::Set(_) => "set",
            Command::Reset(_) => "reset",
            Command::Wait(_) => "wait",
            Command::Move(_) => "move",
            Command::Jump(_) => "jump",
        }
    }

    /// How long the playhead dwells on this command.
    pub fn dur(&self) -> f32 {
        match self {
            Command::Reach(a) => a.dur,
            Command::Aim(a) => a.dur,
            Command::Wait(a) => a.dur,
            Command::Move(a) => a.dur,
            Command::Jump(a) => a.dur,
            // Instant: applied for exactly one tick.
            Command::Set(_) | Command::Reset(_) => 0.0,
        }
    }

    /// The per-command walk speed override (px/s, magnitude): the `speed` of an
    /// active `move`, or `None` for every other command (use the global).
    pub fn move_speed(&self) -> Option<f32> {
        match self {
            Command::Move(a) => a.speed,
            _ => None,
        }
    }

    /// The held-key mask the active command produces: `move` → left/right by the
    /// sign of `dir[0]`, `jump` → the jump key, everything else → none.
    pub fn active_mask(&self) -> u8 {
        match self {
            Command::Move(a) => {
                if a.dir[0] > 0.0 {
                    crate::config::KEY_RIGHT
                } else if a.dir[0] < 0.0 {
                    crate::config::KEY_LEFT
                } else {
                    0
                }
            }
            Command::Jump(a) => {
                // `dir` holds the move key through the jump (app-faithful: keep
                // running while you leap). Without it the move key is released for
                // the jump duration.
                let move_bit = match a.dir {
                    Some(d) if d[0] > 0.0 => crate::config::KEY_RIGHT,
                    Some(d) if d[0] < 0.0 => crate::config::KEY_LEFT,
                    _ => 0,
                };
                move_bit | crate::config::KEY_UP
            }
            _ => 0,
        }
    }
}

#[derive(Deserialize, Clone)]
pub struct ScriptFile {
    pub script: Vec<Command>,
}

/// Built-in scripts used when `SCRIPT` is not given: a big step to one extreme,
/// a settle, a big step back through DC, then a settle — the minimum that exposes
/// waggle for either an arm (aim) or a leg (reach).
pub fn default_script(focus: LimbId) -> Vec<Command> {
    match focus {
        LimbId::RightArm | LimbId::LeftArm => vec![
            Command::Aim(AimArgs {
                limb: focus,
                dir: [1.0, 0.0],
                dist: None,
                kp: None,
                kd: None,
                weapon_offset_y: Some(0.0),
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.6 }),
            Command::Aim(AimArgs {
                limb: focus,
                dir: [-1.0, 0.0],
                dist: None,
                kp: None,
                kd: None,
                weapon_offset_y: Some(0.0),
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.9 }),
        ],
        LimbId::RightLeg | LimbId::LeftLeg => vec![
            Command::Reach(ReachArgs {
                limb: focus,
                to: None,
                dir: Some([1.0, -0.1]),
                dist: None,
                kp: None,
                kd: None,
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.6 }),
            Command::Reach(ReachArgs {
                limb: focus,
                to: None,
                dir: Some([-1.0, -0.1]),
                dist: None,
                kp: None,
                kd: None,
                dur: 1.2,
            }),
            Command::Wait(WaitArgs { dur: 0.9 }),
        ],
    }
}

/// Load the script: `SCRIPT=path.json` if set, else the built-in default.
pub fn load_script(focus: LimbId) -> (Vec<Command>, String) {
    match std::env::var("SCRIPT") {
        Ok(path) if !path.trim().is_empty() => match std::fs::read_to_string(&path) {
            Ok(text) => match serde_json::from_str::<ScriptFile>(&text) {
                Ok(sf) if !sf.script.is_empty() => (sf.script, format!("file:{path}")),
                Ok(_) => {
                    error!("SCRIPT '{path}' parsed but is empty; using built-in default");
                    (default_script(focus), "default(empty-file)".into())
                }
                Err(e) => {
                    error!("SCRIPT '{path}' failed to parse ({e}); using built-in default");
                    (default_script(focus), "default(parse-error)".into())
                }
            },
            Err(e) => {
                error!("SCRIPT '{path}' unreadable ({e}); using built-in default");
                (default_script(focus), "default(unreadable)".into())
            }
        },
        _ => (default_script(focus), "default".into()),
    }
}

/// The jump→walk window: the `move` immediately following a `jump` (its start/end
/// seconds, computed by cumulative `dur`). `None` when the script has no such
/// pattern, which disables the jump→walk verdict.
pub fn jumpwalk_window(script: &[Command]) -> Option<(f32, f32)> {
    let mut t = 0.0f32;
    let mut saw_jump = false;
    for c in script {
        let d = c.dur();
        if saw_jump && matches!(c, Command::Move(_)) {
            return Some((t, t + d));
        }
        if matches!(c, Command::Jump(_)) {
            saw_jump = true;
        }
        t += d;
    }
    None
}
