//! A particle explosion system simulating blood splatter/debris with cone-shaped spread.
//! Uses Rust's built-in random number generation and vector math.

use bevy::color::palettes::css;
use bevy_vello::vello::kurbo::{self, Vec2};
use rand::Rng;
use std::f32::consts::PI;

/// Converts degrees to radians (for trigonometric functions)
const DEG_TO_RAD: f32 = PI / 180.0;

/// Represents a single particle in the explosion
#[derive(Debug, Clone)]
struct Particle {
    /// Current position (x,y)
    position: [f32; 2],
    /// Velocity vector (vx, vy)
    velocity: [f32; 2],
    ///
    orientation: [f32; 2],
    /// Size of the particle
    size: f32,
    /// Current transparency/scale ratio (1.0 = full size, 0.0 = faded out)
    ratio: f32,
    /// If true, particle leaves a permanent mark
    is_trace: bool,
}
#[derive(Clone, Default)]
/// Manages a particle explosion effect
pub struct Explosion {
    /// All active particles
    particles: Vec<Particle>,
    /// Precomputed random directions (cos/sin pairs)
    precomputed_directions: Vec<[f32; 2]>,
    /// How quickly particles fade out (per update)
    decay_rate: f32,
}

impl Explosion {
    /// Precomputes 1000 random direction vectors (cos/sin pairs)
    /// This avoids expensive trig calculations during particle spawning
    fn precompute_directions() -> Vec<[f32; 2]> {
        let mut rng = rand::thread_rng();
        (0..1000)
            .map(|_| {
                // Random angle between 0-2π radians
                let angle = rng.gen_range(0.0..2.0 * PI);
                // Convert to direction vector
                [angle.cos(), angle.sin()]
            })
            .collect()
    }

    pub fn set_decay_rate(&mut self, rate: f32) {
        self.decay_rate = rate;
    }

    /// Creates a new explosion effect
    ///
    /// # Arguments
    /// * `x`, `y` - Spawn position
    /// * `cone_angle` - Total angle of spread in degrees (e.g. 90.0 for a 90-degree cone)
    /// * `direction` - Central direction in degrees (0 = right, 90 = up)
    /// * `speed` - Base velocity of particles
    /// * `size` - Base size of particles
    /// * `count` - Number of particles to spawn
    /// * `is_trace` - Whether particles leave permanent marks
    pub fn new(
        x: f32,
        y: f32,
        cone_angle: f32,
        direction: f32,
        speed: f32,
        size: f32,
        count: usize,
        is_trace: bool,
    ) -> Self {
        let mut rng = rand::thread_rng();
        let precomputed = Self::precompute_directions();

        // Convert angles to radians
        let direction_rad = direction * DEG_TO_RAD;
        let half_cone_rad = (cone_angle / 2.0) * DEG_TO_RAD;

        let particles = (0..count)
            .map(|_| {
                let speed = rng.gen_range(0.0..speed);
                // 1. Get random direction from precomputed set
                let orientation = precomputed[rng.gen_range(0..1000)];

                // 2. Apply cone spread by rotating the direction
                //    - Random angle within the cone's bounds
                let a = rng.gen_range(-half_cone_rad..half_cone_rad) + direction_rad;
                let size = rng.gen_range(0.0..size) + 1.0;
                //let orientation = [1.0, 0.0];
                Particle {
                    position: [x, y],
                    velocity: [speed * a.cos(), speed * a.sin()],
                    orientation,
                    size,
                    ratio: 1.0,
                    is_trace,
                }
            })
            .collect();

        Explosion {
            particles,
            precomputed_directions: precomputed,
            decay_rate: 0.01,
        }
    }

    /// Updates all particles (position and fade-out)
    pub fn update(&mut self, delta_time: f32) {
        for p in &mut self.particles {
            // Update position based on velocity
            p.position[0] += p.velocity[0] * delta_time;
            p.position[1] += p.velocity[1] * delta_time;

            // Apply decay (unless it's a permanent trace)
            if !p.is_trace {
                p.ratio = (p.ratio - self.decay_rate * delta_time).max(0.0);
            }
        }
    }

    pub fn get_transforms(&self, transforms: &mut Vec<kurbo::Affine>) {
        self.particles.iter().for_each(|p| {
            let scale = p.size * p.ratio;
            //35 * 30
            let cos = p.orientation[0] * scale;
            let sin = p.orientation[1] * scale;
            let x_offset = 35.0 * cos - sin * 30.0;
            let y_offset = 35.0 * sin + cos * 30.0;
            transforms.push(kurbo::Affine::new([
                cos as f64,
                sin as f64,
                -sin as f64,
                cos as f64,
                (p.position[0] - x_offset) as f64,
                (p.position[1] - y_offset) as f64,
            ]));

            // transforms.push(kurbo::Affine::new([
            //     2.0 as f64,
            //     0.0 as f64,
            //     0.0 as f64,
            //     2.0 as f64,
            //     p.position[0] as f64,
            //     p.position[1] as f64,
            // ]));
        });
    }
}
