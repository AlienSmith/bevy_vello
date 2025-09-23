use crate::VectorFile;
use bevy::{
    asset::{io::Reader, AssetLoader, AsyncReadExt, LoadContext},
    prelude::*,
    reflect::TypePath,
};
use thiserror::Error;
use vello::ReuseSceneReplayer;
#[derive(Asset, TypePath, Clone)]
pub struct VelloReplaySceneAsset {
    pub player: ReuseSceneReplayer,
}

#[derive(Default)]
pub struct VelloReplaySceneAssetLoader;

/// Possible errors that can be produced by [`VelloReplaySceneAssetLoader`]
#[non_exhaustive]
#[derive(Debug, Error)]
pub enum VelloReplaySceneAssetLoaderError {
    /// An [IO](std::io) Error
    #[error("Could not load asset: {0}")]
    Io(#[from] std::io::Error),
    /// A [RON](ron) Error
    #[error("Could not parse RON: {0}")]
    ReplayerError(#[from] vello::SceneReplayerError),
}

impl AssetLoader for VelloReplaySceneAssetLoader {
    type Asset = VelloReplaySceneAsset;
    type Settings = ();
    type Error = VelloReplaySceneAssetLoaderError;
    async fn load<'a>(
        &'a self,
        reader: &'a mut Reader<'_>,
        _settings: &'a (),
        _load_context: &'a mut LoadContext<'_>,
    ) -> Result<VelloReplaySceneAsset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let player = vello::ReuseSceneReplayer::new(&bytes)?;
        Ok(VelloReplaySceneAsset { player })
    }

    fn extensions(&self) -> &[&str] {
        &["scene"]
    }
}

#[derive(Asset, TypePath, Clone)]
pub struct VelloAsset {
    pub file: VectorFile,
    pub local_transform_center: Transform,
    pub width: f32,
    pub height: f32,
    pub alpha: f32,
}

impl VelloAsset {
    /// Returns the bounding box in world space
    pub fn bb_in_world_space(&self, gtransform: &GlobalTransform) -> Rect {
        // Convert local coordinates to world coordinates
        let local_min = Vec3::new(-self.width / 2.0, -self.height / 2.0, 0.0).extend(1.0);
        let local_max = Vec3::new(self.width / 2.0, self.height / 2.0, 0.0).extend(1.0);

        let min_world = gtransform.compute_matrix() * local_min;
        let max_world = gtransform.compute_matrix() * local_max;

        // Calculate the distance between the vertices to get the size in world space
        let min = Vec2::new(min_world.x, min_world.y);
        let max = Vec2::new(max_world.x, max_world.y);
        Rect { min, max }
    }

    /// Returns the bounding box in screen space
    pub fn bb_in_screen_space(
        &self,
        gtransform: &GlobalTransform,
        camera: &Camera,
        camera_transform: &GlobalTransform,
    ) -> Option<Rect> {
        let Rect { min, max } = self.bb_in_world_space(gtransform);
        camera
            .viewport_to_world_2d(camera_transform, min)
            .zip(camera.viewport_to_world_2d(camera_transform, max))
            .map(|(min, max)| Rect { min, max })
    }
}

/// Describes how to position the asset from the origin
#[derive(Component, Default, Clone, Copy, PartialEq, Eq)]
pub enum VelloAssetAlignment {
    /// Bounds start from the render position and advance up and to the right.
    BottomLeft,
    /// Bounds start from the render position and advance up.
    Bottom,
    /// Bounds start from the render position and advance up and to the left.
    BottomRight,

    /// Bounds start from the render position and advance right.
    Left,
    /// Bounds start from the render position and advance equally on both axes.
    #[default]
    Center,
    /// Bounds start from the render position and advance left.
    Right,

    /// Bounds start from the render position and advance down and to the right.
    TopLeft,
    /// Bounds start from the render position and advance down.
    Top,
    /// Bounds start from the render position and advance down and to the left.
    TopRight,
}

impl VelloAssetAlignment {
    pub(crate) fn compute(
        &self,
        asset: &VelloAsset,
        transform: &GlobalTransform,
    ) -> GlobalTransform {
        let (width, height) = (asset.width, asset.height);
        // Apply alignment
        let adjustment = match self {
            VelloAssetAlignment::TopLeft => Vec3::new(width / 2.0, -height / 2.0, 0.0),
            VelloAssetAlignment::Left => Vec3::new(width / 2.0, 0.0, 0.0),
            VelloAssetAlignment::BottomLeft => Vec3::new(width / 2.0, height / 2.0, 0.0),
            VelloAssetAlignment::Top => Vec3::new(0.0, -height / 2.0, 0.0),
            VelloAssetAlignment::Center => Vec3::new(0.0, 0.0, 0.0),
            VelloAssetAlignment::Bottom => Vec3::new(0.0, height / 2.0, 0.0),
            VelloAssetAlignment::TopRight => Vec3::new(-width / 2.0, -height / 2.0, 0.0),
            VelloAssetAlignment::Right => Vec3::new(-width / 2.0, 0.0, 0.0),
            VelloAssetAlignment::BottomRight => Vec3::new(-width / 2.0, height / 2.0, 0.0),
        };
        let new_translation: Vec3 = (transform.compute_matrix() * adjustment.extend(1.0)).xyz();
        GlobalTransform::from(
            transform
                .compute_transform()
                .with_translation(new_translation),
        )
    }
}

#[derive(Clone, Default)]
pub enum TankGameAssetsMetaData {
    #[default]
    PBR,
    //we need the total frame counts
    SpriteSheet(u32),
}

use bevy::utils::HashMap;

pub trait AssetWithMeta: Asset {
    type Meta: Clone + Default; // Metadata type for this asset
}

//pub type TankAssetManager = AssetManager<VelloReplaySceneAsset>;

#[derive(Default)]
pub struct AssetEntry<T: AssetWithMeta> {
    pub handle: Handle<T>,
    pub meta: T::Meta,
}
//The compiler will wants T: Clone if we use derived clone method
impl<T: AssetWithMeta> Clone for AssetEntry<T> {
    fn clone(&self) -> Self {
        Self {
            handle: self.handle.clone(), // Handle<T> is always Clone
            meta: self.meta.clone(),     // T::Meta is Clone per your trait bound
        }
    }
}

#[derive(Resource)]
pub struct VelloAssetManager<T: AssetWithMeta> {
    pub id_to_index: HashMap<AssetId<T>, usize>,
    pub parts: Vec<AssetEntry<T>>,
    pub load_state: Vec<bool>,
}

impl<T: AssetWithMeta> Default for VelloAssetManager<T> {
    fn default() -> Self {
        Self {
            id_to_index: Default::default(),
            parts: Default::default(),
            load_state: Default::default(),
        }
    }
}

impl<T: AssetWithMeta> Clone for VelloAssetManager<T> {
    fn clone(&self) -> Self {
        Self {
            id_to_index: self.id_to_index.clone(),
            parts: self.parts.clone(),
            load_state: self.load_state.clone(),
        }
    }
}

impl<T: AssetWithMeta> VelloAssetManager<T> {
    pub fn push(&mut self, handle: Handle<T>, meta: T::Meta) {
        let index = self.parts.len();
        let id = handle.id();
        self.id_to_index.insert(id, index);
        self.parts.push(AssetEntry { handle, meta });
        self.load_state.push(false);
    }

    pub fn get_index<R: Into<usize>>(&self, index: R) -> Option<Handle<T>> {
        let index = index.into();
        if self.parts.len() > index && self.load_state[index] {
            return Some(self.parts[index].clone().handle);
        }
        None
    }

    pub fn get_entry_at_index<R: Into<usize>>(&self, index: R) -> Option<AssetEntry<T>> {
        let index = index.into();
        if self.parts.len() > index && self.load_state[index] {
            return Some(self.parts[index].clone());
        }
        None
    }

    pub fn mark_as_loaded(&mut self, id: &AssetId<T>) {
        if let Some(index) = self.id_to_index.get(id) {
            self.load_state[*index] = true;
        }
    }

    pub fn all_loaded(&self) -> bool {
        self.load_state.iter().all(|&loaded| loaded)
    }
}

impl AssetWithMeta for VelloReplaySceneAsset {
    type Meta = TankGameAssetsMetaData;
}
