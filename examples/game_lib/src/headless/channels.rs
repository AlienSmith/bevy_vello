//! Telemetry channels: which particles to log, the harness-written scalar row,
//! and the capture system that assembles one CSV row per tick.

use bevy::prelude::*;
use bevy_vello::integrations::physics::VelloParticle;

use crate::character::Connectivity;

/// The **v10** particle order — exactly the order of the `particles` array in
/// [`v10.character.json`](../../../collision_detection/assets/character/v10.character.json).
///
/// `P2` is index 2. Because v10 is the only character we track and this order is
/// fixed, we resolve by name (never by ECS iteration order) and emit columns in
/// this order. Positions/velocities are logged raw in the Vello engine frame
/// (x right, y down); no transform is applied.
pub const V10_PARTICLES: [&str; 13] = [
    "P0", "P1", "P2", "P3", "P40", "P41", "P10", "P13", "PH", "PLLA", "PRLA", "PRLL", "PLLL",
];

/// What to log and how. Inserted by the harness once the character entity is
/// known; [`capture_telemetry`] reads it every tick.
#[derive(Resource, Clone)]
pub struct ParticleTelemetryConfig {
    /// The character root whose particles are tracked.
    pub character: Entity,
    /// Particle names in output order (default [`V10_PARTICLES`]).
    pub names: Vec<String>,
    /// Emit `vx,vy` per particle.
    pub emit_vel: bool,
    /// Emit the raw finite-difference acceleration (`ax,ay`) **and** its
    /// EMA-smoothed copy (`fx,fy`) per particle. Implies velocity tracking.
    pub emit_accel: bool,
    /// EMA time constant (seconds) for the smoothed acceleration, used only when
    /// [`Self::emit_accel`] is set.
    pub filt_tau: f32,
    /// Sampling period (seconds) used for the acceleration finite difference.
    /// Must match the schedule the harness runs [`capture_telemetry`] in.
    pub dt: f32,
}

impl ParticleTelemetryConfig {
    /// Track all of v10, positions + velocities, no acceleration.
    pub fn v10(character: Entity) -> Self {
        Self {
            character,
            names: V10_PARTICLES.iter().map(|s| (*s).to_string()).collect(),
            emit_vel: true,
            emit_accel: false,
            filt_tau: 0.06,
            dt: 1.0 / 90.0,
        }
    }

    /// Whether the velocity columns are emitted (accel needs velocities too).
    fn want_vel(&self) -> bool {
        self.emit_vel || self.emit_accel
    }
}

/// Harness-written scalar columns for the current tick, in header order. Values
/// are **pre-formatted** by the harness so it keeps full control of precision
/// (e.g. `row.set("tau", format!("{:.4}", tau))`).
///
/// The harness must call [`TelemetryRow::clear`] then re-`set` the same keys,
/// in the same order, every tick before `capture_telemetry` runs.
#[derive(Resource, Default)]
pub struct TelemetryRow {
    pub scalars: Vec<(String, String)>,
}

impl TelemetryRow {
    pub fn set(&mut self, key: &str, value: impl Into<String>) {
        self.scalars.push((key.to_string(), value.into()));
    }
    pub fn clear(&mut self) {
        self.scalars.clear();
    }
}

/// The assembled CSV: `header` is frozen on the first capture; `rows` grow one
/// per tick. The harness owns flushing (see [`crate::headless::flush_csv`]).
#[derive(Resource, Default)]
pub struct TelemetryBuffer {
    pub header: String,
    pub rows: Vec<String>,
}

impl TelemetryBuffer {
    /// Drop the rows for a new section/cell but keep the frozen header.
    pub fn clear_rows(&mut self) {
        self.rows.clear();
    }
    /// Drop everything, including the header (re-resolve next capture).
    pub fn reset(&mut self) {
        self.header.clear();
        self.rows.clear();
    }
}

/// Name→entity resolution + finite-difference state, owned by the plugin.
#[derive(Resource, Default)]
pub struct ResolvedParticles {
    resolved: bool,
    /// One slot per [`ParticleTelemetryConfig::names`] entry; `None` = absent
    /// (its columns are emitted as `nan`).
    entities: Vec<Option<Entity>>,
    prev_vel: Vec<Vec2>,
    filt_acc: Vec<Vec2>,
    primed: bool,
}

/// Append one row: the harness scalars (from [`TelemetryRow`]) followed by the
/// resolved particle channels, in [`ParticleTelemetryConfig::names`] order.
///
/// On the first call it resolves every name to its particle entity (scanning
/// `VelloParticle.root_entity == character` and matching `Connectivity.name`)
/// and freezes [`TelemetryBuffer::header`]. Resolution is by-key, so it does not
/// depend on ECS iteration order.
pub fn capture_telemetry(
    config: Option<Res<ParticleTelemetryConfig>>,
    mut row: ResMut<TelemetryRow>,
    mut buf: ResMut<TelemetryBuffer>,
    mut resolved: ResMut<ResolvedParticles>,
    particle_q: Query<(Entity, &VelloParticle)>,
    conn_q: Query<&Connectivity>,
) {
    // The harness inserts the config when the rig first becomes ready; before
    // then there is nothing to log. Running as a no-op keeps the system safe to
    // schedule unconditionally from frame 0.
    let Some(config) = config else { return };
    let names = config.names.clone();

    if !resolved.resolved {
        let mut entities = vec![None; names.len()];
        for (ent, p) in particle_q.iter() {
            if p.root_entity != config.character {
                continue;
            }
            let Ok(c) = conn_q.get(ent) else { continue };
            let name = c.name.to_string();
            if let Some(i) = names.iter().position(|nm| *nm == name) {
                entities[i] = Some(ent);
            }
        }
        resolved.entities = entities;
        resolved.prev_vel = vec![Vec2::ZERO; names.len()];
        resolved.filt_acc = vec![Vec2::ZERO; names.len()];
        resolved.primed = false;

        // Freeze the header from the harness scalar keys (first capture) + the
        // particle channels implied by the config.
        let mut cols: Vec<String> = row.scalars.iter().map(|(k, _)| k.clone()).collect();
        for name in &names {
            cols.push(format!("{name}x"));
            cols.push(format!("{name}y"));
            if config.want_vel() {
                cols.push(format!("{name}vx"));
                cols.push(format!("{name}vy"));
            }
            if config.emit_accel {
                cols.push(format!("{name}ax"));
                cols.push(format!("{name}ay"));
                cols.push(format!("{name}fx"));
                cols.push(format!("{name}fy"));
            }
        }
        buf.header = cols.join(",");
        resolved.resolved = true;
    }

    let want_vel = config.want_vel();
    let alpha = if config.emit_accel && config.filt_tau > 0.0 {
        (config.dt / config.filt_tau).clamp(0.0, 1.0)
    } else {
        0.0
    };

    // Scalars first (header order == the order the harness `set` them).
    let mut vals: Vec<String> = row.scalars.iter().map(|(_, v)| v.clone()).collect();

    for (i, _name) in names.iter().enumerate() {
        let got = resolved.entities[i].and_then(|e| particle_q.get(e).ok());
        match got {
            Some((_, p)) => {
                let pos = p.particle.pos;
                let vel = p.particle.velocity;
                vals.push(format!("{:.2}", pos.x));
                vals.push(format!("{:.2}", pos.y));
                if want_vel {
                    vals.push(format!("{:.2}", vel.x));
                    vals.push(format!("{:.2}", vel.y));
                }
                if config.emit_accel {
                    let (acc, filt) = if resolved.primed {
                        let a = (vel - resolved.prev_vel[i]) / config.dt;
                        let f = resolved.filt_acc[i] + (a - resolved.filt_acc[i]) * alpha;
                        (a, f)
                    } else {
                        (Vec2::ZERO, Vec2::ZERO)
                    };
                    resolved.prev_vel[i] = vel;
                    resolved.filt_acc[i] = filt;
                    vals.push(format!("{:.1}", acc.x));
                    vals.push(format!("{:.1}", acc.y));
                    vals.push(format!("{:.1}", filt.x));
                    vals.push(format!("{:.1}", filt.y));
                } else if want_vel {
                    resolved.prev_vel[i] = vel;
                }
            }
            None => {
                vals.push("nan".into());
                vals.push("nan".into());
                if want_vel {
                    vals.push("nan".into());
                    vals.push("nan".into());
                }
                if config.emit_accel {
                    vals.push("nan".into());
                    vals.push("nan".into());
                    vals.push("nan".into());
                    vals.push("nan".into());
                }
            }
        }
    }

    if config.emit_accel {
        resolved.primed = true;
    }

    buf.rows.push(vals.join(","));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v10_particles_index_and_order() {
        // P2 is index 2 — the whole point of the fixed order.
        assert_eq!(V10_PARTICLES[2], "P2");
        assert_eq!(V10_PARTICLES.len(), 13);
        // First entries follow the JSON array; spine frame P0..P3 up front.
        assert_eq!(&V10_PARTICLES[..4], &["P0", "P1", "P2", "P3"]);
    }

    #[test]
    fn header_is_scalars_then_particle_channels() {
        let mut row = TelemetryRow::default();
        row.set("t", "0.0000");
        row.set("phase", "base");
        let mut cols: Vec<String> = row.scalars.iter().map(|(k, _)| k.clone()).collect();
        for name in ["P0", "P2"] {
            cols.push(format!("{name}x"));
            cols.push(format!("{name}y"));
            cols.push(format!("{name}vx"));
            cols.push(format!("{name}vy"));
        }
        assert_eq!(
            cols.join(","),
            "t,phase,P0x,P0y,P0vx,P0vy,P2x,P2y,P2vx,P2vy"
        );
    }
}
