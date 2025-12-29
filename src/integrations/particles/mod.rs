mod plugin;
mod systems;

use crate::VelloScene;
use bevy::prelude::*;
use rand::Rng;
use thunderdome::Arena;
use vello::kurbo;

struct PersistentBuffer {
    data: Vec<kurbo::Affine>,
    write_index: usize,
}

impl PersistentBuffer {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
            write_index: 0,
        }
    }

    fn push(&mut self, affine: kurbo::Affine) {
        if self.data.len() < self.data.capacity() {
            self.data.push(affine);
        } else {
            self.data[self.write_index] = affine;
            self.write_index = (self.write_index + 1) % self.data.capacity();
        }
    }

    fn append_to(&self, output: &mut Vec<kurbo::Affine>) {
        output.extend_from_slice(&self.data);
    }

    fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, SystemSet)]
pub enum ParticleSystemState {
    UpdateParticleScene,
    UpdateParticleInstances,
}

// ───── Shared Types ───────────────────────────────────────

#[derive(Clone, Copy, Debug)]
pub struct SpawnParams {
    pub position: Vec2,
    pub velocity: Vec2,
    pub lifetime: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct InstanceData {
    pub position: [f32; 2],
    pub scale: f32,
    pub rotation: f32,
    pub custom: [f32; 2],
}

// ───── Traits ─────────────────────────────────────────────

pub trait Particle: Clone + Send + Sync + 'static {
    type Config: Clone + Send + Sync + 'static;

    fn spawn(params: SpawnParams, config: &Self::Config) -> Self;
    fn tick(&mut self, delta: f32, config: &Self::Config);
    fn is_alive(&self, config: &Self::Config) -> bool;
    fn is_persistent(&self, config: &Self::Config) -> bool;
    // notice vello is y down hence you want to do -y and -rotation.
    fn as_instance(&self) -> kurbo::Affine;
}

pub trait Emitter<P: Particle>: Clone + Send + Sync + 'static {
    type Config: Clone + Send + Sync + 'static;

    fn emit(&mut self, config: &Self::Config, rng: &mut impl Rng) -> Option<SpawnParams>;
    fn is_active(&self, config: &Self::Config) -> bool;
    fn from_config(config: &Self::Config) -> Self;
}

// ───── Concrete Particle: GravityParticle ──────────────────

#[derive(Clone)]
pub struct GravityParticle {
    pos: Vec2,
    vel: Vec2,
    age: f32,
    lifetime: f32,
}

#[derive(Clone)]
pub struct GravityParticleConfig {
    pub gravity: Vec2,
    pub drag: f32,
    pub persistent: bool,
}

impl Particle for GravityParticle {
    type Config = GravityParticleConfig;

    fn spawn(params: SpawnParams, _config: &Self::Config) -> Self {
        Self {
            pos: params.position,
            vel: params.velocity,
            age: 0.0,
            lifetime: params.lifetime,
        }
    }

    fn tick(&mut self, delta: f32, config: &Self::Config) {
        self.age += delta;
        self.vel += config.gravity * delta;
        self.vel *= 1.0 - config.drag * delta;
        self.pos += self.vel * delta;
    }

    fn is_alive(&self, _config: &Self::Config) -> bool {
        self.age < self.lifetime
    }

    fn as_instance(&self) -> kurbo::Affine {
        kurbo::Affine::translate((self.pos.x as f64, -self.pos.y as f64))
    }

    fn is_persistent(&self, config: &Self::Config) -> bool {
        config.persistent
    }
}

// ───── Concrete Emitter: BurstEmitter ──────────────────────

#[derive(Clone, Copy, Default)]
pub struct BurstEmitter {
    spawned: u32,
}

#[derive(Clone, Copy)]
pub struct BurstEmitterConfig {
    pub count: u32,
    pub speed_range: (f32, f32),
    pub lifetime_range: (f32, f32),
    pub origin: Vec2,
}

impl<P: Particle> Emitter<P> for BurstEmitter {
    type Config = BurstEmitterConfig;

    fn emit(&mut self, config: &Self::Config, rng: &mut impl Rng) -> Option<SpawnParams> {
        if self.spawned >= config.count {
            return None;
        }
        self.spawned += 1;

        let speed = rng.gen_range(config.speed_range.0..=config.speed_range.1);
        let angle = rng.gen_range(0.0..std::f32::consts::TAU);
        let vel = Vec2::new(angle.cos(), angle.sin()) * speed;
        let lifetime = rng.gen_range(config.lifetime_range.0..=config.lifetime_range.1);

        Some(SpawnParams {
            position: config.origin,
            velocity: vel,
            lifetime,
        })
    }

    fn is_active(&self, config: &Self::Config) -> bool {
        self.spawned < config.count
    }

    fn from_config(_config: &Self::Config) -> Self {
        Self { spawned: 0 }
    }
}

// ───── Generic Particle Group ─────────────────────────────

pub struct ParticleGroup<P: Particle, E: Emitter<P>> {
    pub particle_config: P::Config,
    pub emitter_config: E::Config,
    pub emitter: E,
    pub particles: Arena<P>,
    pub active: bool,
    persistents: PersistentBuffer,
}

impl<P: Particle, E: Emitter<P>> ParticleGroup<P, E> {
    pub fn new(
        particle_config: P::Config,
        emitter_config: E::Config,
        persistent_capacity: usize,
    ) -> Self {
        Self {
            emitter: E::from_config(&emitter_config),
            particle_config,
            emitter_config,
            particles: Arena::new(),
            active: true,
            persistents: PersistentBuffer::with_capacity(persistent_capacity),
        }
    }

    pub fn update(&mut self, delta: f32, rng: &mut impl Rng) {
        // Spawn
        while let Some(params) = self.emitter.emit(&self.emitter_config, rng) {
            let particle = P::spawn(params, &self.particle_config);
            self.particles.insert(particle);
        }

        // Update & cull
        let mut dead = Vec::new();
        for (idx, particle) in &mut self.particles {
            particle.tick(delta, &self.particle_config);
            if !particle.is_alive(&self.particle_config) {
                if particle.is_persistent(&self.particle_config) {
                    self.persistents.push(particle.as_instance());
                }
                dead.push(idx);
            }
        }
        for idx in dead {
            self.particles.remove(idx);
        }

        self.active = self.emitter.is_active(&self.emitter_config) || !self.particles.is_empty();
    }

    pub fn collect_instances(&self) -> Vec<kurbo::Affine> {
        let mut alive = self
            .particles
            .iter()
            .map(|(_, p)| p.as_instance())
            .collect();
        self.persistents.append_to(&mut alive);
        alive
    }

    pub fn is_empty(&self) -> bool {
        self.particles.is_empty() && self.persistents.is_empty()
    }
}

#[macro_export]
macro_rules! define_particle_effect {
    ($vis:vis $name:ident, $particle:ty, $emitter:ty) => {
        #[derive(bevy::prelude::Component)]
        $vis struct $name(pub ParticleGroup<$particle, $emitter>);

        impl $name {
            pub fn new(
                pc: <$particle as Particle>::Config,
                ec: <$emitter as Emitter<$particle>>::Config,
                persistent_capacity: usize,
            ) -> Self {
                Self(ParticleGroup::new(pc, ec, persistent_capacity))
            }

            pub fn update_system(
                mut commands: Commands,
                mut query: Query<(Entity, &mut $name)>,
                time: Res<Time>
            ){
                let delta = time.delta_secs();
                let mut rng = rand::thread_rng();
                for (entity, mut effect) in query.iter_mut() {
                    effect.0.update(delta, &mut rng);
                    if !effect.0.active && effect.0.is_empty() {
                        commands.entity(entity).despawn();
                    }
                }
            }

            pub fn update_particle_instances(mut query: Query<(&mut VelloScene, & $name)>){
                for (mut scene, particle) in query.iter_mut(){
                    let affines = particle.0.collect_instances();
                    scene.update_instance_data_only(&affines);
                }
            }


            pub fn register(app: &mut App) {
                app.add_systems(Update, Self::update_system);
                app.add_systems(Update, Self::update_particle_instances.in_set(ParticleSystemState::UpdateParticleInstances));
            }

        }
    };
}

// Now use it in the same crate!
define_particle_effect!(pub ExplosionEffect, GravityParticle, BurstEmitter);

pub struct VelloPartclePlugin;
impl Plugin for VelloPartclePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.configure_sets(
            Update,
            (
                ParticleSystemState::UpdateParticleScene,
                ParticleSystemState::UpdateParticleInstances,
            )
                .chain(),
        );
        ExplosionEffect::register(app);
    }
}
