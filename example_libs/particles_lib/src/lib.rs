pub mod tank_particles {
    pub mod blood_paint;
    pub mod explosion;
}

pub use tank_particles::blood_paint::Explosion;
pub use tank_particles::explosion::make_explosion_effect;
