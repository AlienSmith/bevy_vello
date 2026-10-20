//! Constants + env-derived harness configuration (whole-body mode, sweep grid,
//! gravity, collision). The limb-PD verdict constants live here too.

use bevy::prelude::{warn, Vec2};

// ---------------------------------------------------------------------------
// Asset paths
// ---------------------------------------------------------------------------

pub const SVG_ASSET: &str = "v10.character.svg";
pub const BP_ASSET: &str = "v10.character.json";
pub const SVG_PATH: &str = "character/v10.character.svg";
pub const BP_PATH: &str = "character/v10.character.json";

/// Asset root override (for local Windows runs). Falls back to the CI/Linux
/// absolute path when unset, so the existing Linux pipeline is unchanged.
pub fn asset_root() -> String {
    std::env::var("ASSET_ROOT")
        .ok()
        .filter(|p| !p.trim().is_empty())
        .unwrap_or_else(|| {
            "/home/ubuntu/bender/bevy_vello/examples/collision_detection/assets".to_string()
        })
}

// ---------------------------------------------------------------------------
// Physics / verdict cadence
// ---------------------------------------------------------------------------

/// The physics cadence the controller + XPBD step at.
pub const FIXED_HZ: f64 = 90.0;
/// Hard cap on cells (safety against a typo'd grid).
pub const MAX_CELLS: usize = 64;
/// Angle band (deg) that counts as "settled".
pub const SETTLE_BAND_DEG: f32 = 1.5;
/// Minimum arc (deg) for the motion to count as a real swing.
pub const MIN_SWING_DEG: f32 = 60.0;
/// Width (s) of the settled tail measured at the end of the final hold window.
pub const TAIL_S: f32 = 0.6;
/// Residual RMS (deg, about the tail mean) above which the tail is judged noisy.
pub const TAIL_RMS_DEG: f32 = 1.0;
/// Angular-rate sign-reversals (with a deadband) within the tail above which the
/// limb is judged to oscillate — a direct, transient-immune wobble detector.
pub const WAGGLE_REVERSALS: u32 = 4;
/// `|omega|` (rad/s) below which a sample is ignored when counting reversals.
pub const REVERSAL_DEADBAND: f32 = 0.05;
/// Settle time (s after the final hold starts) above which the response is "SLOW".
pub const SLOW_SETTLE_S: f32 = 1.5;

pub const PI: f32 = std::f32::consts::PI;

// ---------------------------------------------------------------------------
// Whole-body (hip-pin) mode constants
// ---------------------------------------------------------------------------

/// Gravity magnitude the tuning app uses (100 px = 1 m ⇒ 980 px/s²), Vello frame
/// (x right, y down ⇒ down is +y).
pub const GRAVITY_ON: Vec2 = Vec2::new(0.0, 980.0);
/// Bone-length tolerance for the pinned `P1/P2/P3` frame (and, when the legs are
/// driven, both legs' thigh + calf).
pub const PIN_RIGID_TOL: f32 = 0.02;
/// Foot-tracking tail window (s) + wobble thresholds (matches the retired
/// `headless_pin` leg verdict).
pub const LEG_TAIL_S: f32 = 1.0;
pub const LEG_RMS_MAX: f32 = 6.0;
pub const LEG_REV_MAX: u32 = 6;
pub const LEG_RATE_DEADBAND: f32 = 20.0;
/// Held-key mask bits written into the events sidecar (`keys` column).
pub const KEY_RIGHT: u8 = 1;
pub const KEY_LEFT: u8 = 2;
pub const KEY_UP: u8 = 4;
/// The physical keys `default_input_map()` binds for (Right, Left, Up). The probe
/// entity uses `default_input_map()` (L/J/I), **not** WASD.
pub const KEYS: [KeyCode; 3] = [KeyCode::KeyL, KeyCode::KeyJ, KeyCode::KeyI];

use bevy::prelude::KeyCode;

/// Gravity for the harness, from `GRAVITY`: `on`/`1` → [`GRAVITY_ON`]; `off`/`0`
/// → zero; `x,y` → a raw vello-frame vector (x right, y down).
pub fn gravity_from_env() -> Vec2 {
    match std::env::var("GRAVITY").ok().as_deref().map(str::trim) {
        // Default ON in body mode (this fn is only called when `body` is true):
        // the character must fall onto the floor to test standing. Explicit
        // `off`/`0` still opt out.
        None | Some("") => GRAVITY_ON,
        Some("off") | Some("0") => Vec2::ZERO,
        Some("on") | Some("1") => GRAVITY_ON,
        Some(spec) => {
            let mut it = spec.split(',');
            let x = it.next().and_then(|v| v.trim().parse::<f32>().ok());
            let y = it.next().and_then(|v| v.trim().parse::<f32>().ok());
            match (x, y) {
                (Some(x), Some(y)) => Vec2::new(x, y),
                _ => {
                    warn!("GRAVITY=`{spec}` not understood; falling back to zero");
                    Vec2::ZERO
                }
            }
        }
    }
}

/// Headless collision is opt-in so the deterministic kinematic gates stay
/// byte-identical.
pub fn collision_enabled() -> bool {
    // Default ON: the angular-only walking/standing target needs the floor's
    // normal reaction + the soft-body contact. `COLLISION=0|off|false` opts out
    // (keeps the deterministic limb-PD gates byte-identical).
    !matches!(
        std::env::var("COLLISION").ok().as_deref().map(str::trim),
        Some("0") | Some("off") | Some("false") | Some("no")
    )
}

/// Parse a `kp:kd,kp:kd,...` sweep grid, capped at [`MAX_CELLS`].
pub fn parse_grid(spec: &str) -> Vec<(f32, f32)> {
    spec.split(',')
        .filter_map(|cell| {
            let mut it = cell.split(':');
            let kp: f32 = it.next()?.trim().parse().ok()?;
            let kd: f32 = it.next()?.trim().parse().ok()?;
            Some((kp, kd))
        })
        .take(MAX_CELLS)
        .collect()
}

/// Default sweep grid when `SWEEP=1` and no `GRID` is given.
pub fn default_grid() -> Vec<(f32, f32)> {
    let kps = [0.2, 0.35, 0.5, 0.65, 0.8];
    let kds = [0.0, 0.03, 0.065, 0.1, 0.14];
    let mut g = Vec::new();
    for kp in kps {
        for kd in kds {
            g.push((kp, kd));
        }
    }
    g
}

/// Pin params from env (body mode). `LEGS=off` keeps the pin on `P1/P2/P3` only;
/// otherwise the pin also drives the legs (the whole-body default).
pub fn hip_pin_params_from_env() -> game_lib::HipPinParams {
    // Shared with the tuning app (`SUPPORT_SHARE`, `HIP_DROP`, ...).
    game_lib::HipPinParams::from_env()
}
