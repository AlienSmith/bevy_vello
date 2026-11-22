//! Resolve **which** character rig (SVG + blueprint JSON) a harness opens.
//!
//! The QA harness and the PPO training app used to hardcode
//! `character/v10.character.svg` / `.json` *and* carry a stale
//! `/home/ubuntu/...` fallback for `ASSET_ROOT`. A release binary started
//! without the environment set therefore either loaded nothing (and hung in the
//! `Loading` state forever, because Bevy 0.16 has no `AssetEvent::Failed`) or
//! loaded whatever snapshot happened to sit next to it. Training and QA could
//! silently run on different rigs.
//!
//! Both binaries now resolve their assets through this one type, so they can be
//! pointed at the *same* files explicitly:
//!
//! ```sh
//! # same rig for QA and training
//! tools/physics-qa/run_walker_qa.sh
//! ./target/release/walker_ppo --character \
//!     examples/collision_detection/assets/character/v10.character.json
//! ```
//!
//! Precedence (highest first):
//!
//! | knob | meaning |
//! | --- | --- |
//! | `--character <file.svg\|file.json>` | the rig; the sibling is derived |
//! | `--character-svg <file.svg>` | explicit SVG only |
//! | `--character-json <file.json>` | explicit blueprint only |
//! | `--asset-root <dir>` | Bevy's `AssetPlugin.file_path` root |
//! | `CHARACTER_ASSET` | same as `--character` |
//! | `CHARACTER_SVG` / `CHARACTER_JSON` | one file — the sibling (`svg` ↔ `json`) is derived, like `--character`, so the two halves can never disagree |
//! | `ASSET_ROOT` | same as `--asset-root` |
//! | *(nothing)* | `<repo>/examples/collision_detection/assets` + `character/v10.character.*` |
//!
//! The default root is derived from `CARGO_MANIFEST_DIR` at compile time, so it
//! is correct in every checkout and never points at another machine's home
//! directory. An explicit path that is not under the asset root is an error —
//! Bevy cannot load assets outside `file_path`, and silently falling back would
//! recreate exactly the bug this module exists to prevent.
use std::{
    env, fmt,
    path::{Path, PathBuf},
};

use bevy::prelude::Resource;

/// Asset-root-relative SVG path used when nothing is given.
pub const DEFAULT_SVG: &str = "character/v10.character.svg";
/// Asset-root-relative blueprint path used when nothing is given.
pub const DEFAULT_JSON: &str = "character/v10.character.json";

const ENV_ROOT: &str = "ASSET_ROOT";
const ENV_SVG: &str = "CHARACTER_SVG";
const ENV_JSON: &str = "CHARACTER_JSON";
const ENV_CHARACTER: &str = "CHARACTER_ASSET";

/// `examples/collision_detection/assets`, resolved at compile time.
pub fn default_asset_root() -> PathBuf {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.pop(); // examples/game_lib -> examples
    root.push("collision_detection");
    root.push("assets");
    root
}

/// Raw request: every field is `None` unless the caller demanded it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CharacterAssetSpec {
    pub asset_root: Option<PathBuf>,
    /// A single rig file; the sibling (`svg` <-> `json`) is derived.
    pub character: Option<PathBuf>,
    pub svg: Option<PathBuf>,
    pub json: Option<PathBuf>,
}

/// What a harness should hand to Bevy and to the asset managers.
///
/// Inserted as a resource: `AssetPlugin.file_path`, the asset-manager keys and
/// `CharacterRoot` all read the same value.
#[derive(Resource, Clone, Debug, PartialEq, Eq)]
pub struct CharacterAssetPaths {
    root: PathBuf,
    svg_abs: PathBuf,
    json_abs: PathBuf,
    svg_rel: String,
    json_rel: String,
    svg_id: String,
    json_id: String,
}

/// Why a request could not be turned into loadable paths.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CharacterAssetError {
    /// `--character` / `CHARACTER_ASSET` needs a `.svg` or `.json`.
    BadExtension(PathBuf),
    /// The file is outside the asset root, so `server.load` could never read it.
    OutsideRoot { root: PathBuf, file: PathBuf },
    /// Valid paths, but the file is not on disk — fail before the app hangs.
    Missing {
        role: &'static str,
        file: PathBuf,
        root: PathBuf,
    },
}

impl fmt::Display for CharacterAssetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadExtension(path) => write!(
                f,
                "character asset `{}` must be a .svg or .json file",
                path.display()
            ),
            Self::OutsideRoot { root, file } => write!(
                f,
                "character asset `{}` is outside the asset root `{}`; \
                 Bevy can only load files under the root.\n\
                 Pass --asset-root/ASSET_ROOT so the file sits below it.",
                file.display(),
                root.display()
            ),
            Self::Missing { role, file, root } => write!(
                f,
                "character {role} not found: `{}` (asset root `{}`).\n\
                 Pass --character-svg/--character-json, --character, \
                 CHARACTER_SVG/CHARACTER_JSON/CHARACTER_ASSET, or \
                 --asset-root/ASSET_ROOT.",
                file.display(),
                root.display()
            ),
        }
    }
}

impl std::error::Error for CharacterAssetError {}

impl CharacterAssetPaths {
    /// Resolve from everywhere: CLI flags first, then environment, then defaults.
    pub fn from_env_and_args() -> Result<Self, CharacterAssetError> {
        let mut spec = parse_args(env::args().skip(1));
        spec.asset_root = spec.asset_root.or_else(|| env_path(ENV_ROOT));
        spec.character = spec.character.or_else(|| env_path(ENV_CHARACTER));
        spec.svg = spec.svg.or_else(|| env_path(ENV_SVG));
        spec.json = spec.json.or_else(|| env_path(ENV_JSON));
        Self::resolve(spec)
    }

    /// Turn a request into loadable paths. Does not check that the files exist
    /// (that is [`Self::validate`]), but does use the filesystem to disambiguate
    /// a relative character path between "asset-root-relative" (the natural
    /// reading of `character/v10.character.json`) and "cwd-relative".
    pub fn resolve(spec: CharacterAssetSpec) -> Result<Self, CharacterAssetError> {
        let mut svg_in = spec.svg;
        let mut json_in = spec.json;
        if let Some(character) = spec.character {
            match character.extension().and_then(|e| e.to_str()) {
                Some("svg") => {
                    svg_in.get_or_insert(character);
                }
                Some("json") => {
                    json_in.get_or_insert(character);
                }
                _ => return Err(CharacterAssetError::BadExtension(character)),
            }
        }
        // A single file always implies the rig *pair*: derive the sibling so a
        // lone `CHARACTER_JSON` (what run_walker_qa.sh pins) loads the matching
        // SVG too, and vice-versa — the two halves can never disagree. An
        // explicit pair with different stems is still respected (both slots are
        // already filled, so `get_or_insert` is a no-op).
        if let Some(json) = json_in.as_ref() {
            svg_in.get_or_insert_with(|| sibling_with_extension(json, "svg"));
        }
        if let Some(svg) = svg_in.as_ref() {
            json_in.get_or_insert_with(|| sibling_with_extension(svg, "json"));
        }

        let explicit_root = spec.asset_root.map(absolutize);
        let root = explicit_root
            .or_else(|| {
                // `--character /some/assets/character/v10.character.json` should
                // not also require `--asset-root`.
                given_files(&svg_in, &json_in).find_map(|p| root_for_existing_file(p))
            })
            .or_else(default_asset_root_if_present)
            .unwrap_or_else(default_asset_root);

        let svg_abs = resolve_file(&root, svg_in, DEFAULT_SVG);
        let json_abs = resolve_file(&root, json_in, DEFAULT_JSON);

        let svg_rel = rel_to_root(&root, &svg_abs)?;
        let json_rel = rel_to_root(&root, &json_abs)?;

        Ok(Self {
            svg_id: file_name(&svg_abs),
            json_id: file_name(&json_abs),
            root,
            svg_abs,
            json_abs,
            svg_rel,
            json_rel,
        })
    }

    /// Fail fast when the resolved files are not actually there. Called before
    /// the app starts: Bevy would otherwise sit in `Loading` forever.
    pub fn validate(&self) -> Result<(), CharacterAssetError> {
        for (role, file) in [("svg", &self.svg_abs), ("blueprint", &self.json_abs)] {
            if !file.is_file() {
                return Err(CharacterAssetError::Missing {
                    role,
                    file: file.clone(),
                    root: self.root.clone(),
                });
            }
        }
        Ok(())
    }

    /// Bevy `AssetPlugin.file_path`.
    pub fn root_string(&self) -> String {
        self.root.to_string_lossy().into_owned()
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    pub fn svg_abs(&self) -> &Path {
        &self.svg_abs
    }

    pub fn json_abs(&self) -> &Path {
        &self.json_abs
    }

    /// Asset-root-relative, `/`-separated: what `AssetServer::load` wants.
    pub fn svg_path(&self) -> &str {
        &self.svg_rel
    }

    pub fn json_path(&self) -> &str {
        &self.json_rel
    }

    /// Manager key for `CharacterRoot::svg_asset_id`.
    pub fn svg_id(&self) -> &str {
        &self.svg_id
    }

    /// Manager key for `CharacterRoot::blueprint_asset_id`.
    pub fn json_id(&self) -> &str {
        &self.json_id
    }

    /// Multi-line report for startup logs — the record of which rig was opened.
    pub fn describe(&self) -> String {
        format!(
            "  root {}\n  svg  {}  (id {})\n  json {}  (id {})",
            self.root.display(),
            self.svg_abs.display(),
            self.svg_id,
            self.json_abs.display(),
            self.json_id
        )
    }
}

/// Parse only the flags this module owns; everything else is left alone so the
/// binaries can add their own arguments later.
pub fn parse_args(args: impl Iterator<Item = String>) -> CharacterAssetSpec {
    let mut spec = CharacterAssetSpec::default();
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        let (key, inline) = match arg.split_once('=') {
            Some((k, v)) => (k.to_string(), Some(PathBuf::from(v))),
            None => (arg, None),
        };
        let slot = match key.as_str() {
            "--asset-root" => &mut spec.asset_root,
            "--character" => &mut spec.character,
            "--character-svg" => &mut spec.svg,
            "--character-json" => &mut spec.json,
            _ => continue,
        };
        if let Some(v) = inline {
            *slot = Some(v);
        } else if let Some(v) = args.next() {
            *slot = Some(PathBuf::from(v));
        }
    }
    spec
}

fn env_path(key: &str) -> Option<PathBuf> {
    env::var_os(key)
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
}

/// Default root only when it exists — keeps unit tests and odd installs from
/// silently pointing at a directory that is not there.
fn default_asset_root_if_present() -> Option<PathBuf> {
    let root = default_asset_root();
    root.is_dir().then_some(root)
}

fn absolutize(path: PathBuf) -> PathBuf {
    if path.is_absolute() {
        path
    } else {
        env::current_dir()
            .map(|cwd| cwd.join(&path))
            .unwrap_or(path)
    }
}

/// The rig sibling: same parent and stem, only the final extension changes —
/// `v8.character.json` → `v8.character.svg` (not `v8.svg`).
fn sibling_with_extension(path: &Path, ext: &str) -> PathBuf {
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    path.with_file_name(format!("{stem}.{ext}"))
}

fn given_files<'a>(
    svg: &'a Option<PathBuf>,
    json: &'a Option<PathBuf>,
) -> impl Iterator<Item = &'a PathBuf> {
    svg.iter().chain(json.iter())
}

/// Derive the asset root from a single explicitly given file, when no
/// `--asset-root`/`ASSET_ROOT` was supplied: `.../assets/character/rig.json`
/// implies the root is `.../assets`. The file must exist, otherwise there is
/// nothing to derive from and the default root (with its clear error) wins.
fn root_for_existing_file(path: &Path) -> Option<PathBuf> {
    // Relative character paths are asset-relative by convention here, so read
    // them against the repo default first.
    let candidate = if path.is_absolute() {
        path.to_path_buf()
    } else {
        default_asset_root().join(path)
    };
    if !candidate.is_file() {
        return None;
    }
    let default = default_asset_root();
    if candidate.starts_with(&default) {
        return Some(default);
    }
    let parent = candidate.parent()?;
    // `<root>/character/<file>` is this project's layout; anything else is
    // treated as a file sitting directly in the root.
    if parent
        .file_name()
        .map(|n| n == "character")
        .unwrap_or(false)
    {
        parent.parent().map(Path::to_path_buf)
    } else {
        Some(parent.to_path_buf())
    }
}

fn resolve_file(root: &Path, given: Option<PathBuf>, default_rel: &str) -> PathBuf {
    match given {
        Some(path) => {
            let root_relative = root.join(&path);
            if path.is_absolute() {
                path
            } else if root_relative.is_file() {
                // `character/v10.character.json` ⇒ relative to the asset root.
                root_relative
            } else {
                absolutize(path)
            }
        }
        None => root.join(default_rel),
    }
}

/// Strip the root and join with `/` so the result is a valid Bevy asset path.
fn rel_to_root(root: &Path, file: &Path) -> Result<String, CharacterAssetError> {
    let rel = file
        .strip_prefix(root)
        .map_err(|_| CharacterAssetError::OutsideRoot {
            root: root.to_path_buf(),
            file: file.to_path_buf(),
        })?;
    Ok(rel
        .components()
        .filter_map(|c| match c {
            std::path::Component::Normal(part) => Some(part.to_string_lossy().into_owned()),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join("/"))
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(root: &str) -> CharacterAssetSpec {
        CharacterAssetSpec {
            asset_root: Some(PathBuf::from(root)),
            ..Default::default()
        }
    }

    #[test]
    fn default_root_is_the_repo_assets_dir() {
        let root = default_asset_root();
        assert!(
            root.ends_with("examples/collision_detection/assets"),
            "{root:?}"
        );
        assert!(!root.to_string_lossy().contains("/home/ubuntu"));
    }

    #[test]
    fn defaults_are_the_v10_rig_relative_to_the_root() {
        let p = CharacterAssetPaths::resolve(spec("/tmp/assets")).unwrap();
        assert_eq!(p.root(), Path::new("/tmp/assets"));
        assert_eq!(p.svg_path(), DEFAULT_SVG);
        assert_eq!(p.json_path(), DEFAULT_JSON);
        assert_eq!(p.svg_id(), "v10.character.svg");
        assert_eq!(p.json_id(), "v10.character.json");
    }

    #[test]
    fn character_flag_derives_the_sibling_and_keeps_the_root() {
        let spec = CharacterAssetSpec {
            asset_root: Some(PathBuf::from("/tmp/assets")),
            character: Some(PathBuf::from("/tmp/assets/character/v8.character.json")),
            ..Default::default()
        };
        let p = CharacterAssetPaths::resolve(spec).unwrap();
        assert_eq!(
            p.svg_abs(),
            Path::new("/tmp/assets/character/v8.character.svg")
        );
        assert_eq!(
            p.json_abs(),
            Path::new("/tmp/assets/character/v8.character.json")
        );
        assert_eq!(p.svg_id(), "v8.character.svg");
        assert_eq!(p.json_id(), "v8.character.json");
    }

    #[test]
    fn lone_json_env_style_derives_the_svg_sibling() {
        let p = CharacterAssetPaths::resolve(CharacterAssetSpec {
            asset_root: Some(PathBuf::from("/tmp/assets")),
            json: Some(PathBuf::from("/tmp/assets/character/v8.character.json")),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(
            p.svg_abs(),
            Path::new("/tmp/assets/character/v8.character.svg")
        );
        assert_eq!(
            p.json_abs(),
            Path::new("/tmp/assets/character/v8.character.json")
        );
    }

    #[test]
    fn explicit_svg_and_json_can_have_different_stems() {
        let spec = CharacterAssetSpec {
            asset_root: Some(PathBuf::from("/tmp/assets")),
            svg: Some(PathBuf::from("/tmp/assets/rig/shape.svg")),
            json: Some(PathBuf::from("/tmp/assets/rig/physics.json")),
            ..Default::default()
        };
        let p = CharacterAssetPaths::resolve(spec).unwrap();
        assert_eq!(p.svg_path(), "rig/shape.svg");
        assert_eq!(p.json_path(), "rig/physics.json");
    }

    #[test]
    fn outside_the_root_is_an_error() {
        let spec = CharacterAssetSpec {
            asset_root: Some(PathBuf::from("/tmp/assets")),
            character: Some(PathBuf::from("/elsewhere/v10.character.json")),
            ..Default::default()
        };
        assert!(matches!(
            CharacterAssetPaths::resolve(spec),
            Err(CharacterAssetError::OutsideRoot { .. })
        ));
    }

    #[test]
    fn bad_extension_is_an_error() {
        let spec = CharacterAssetSpec {
            asset_root: Some(PathBuf::from("/tmp/assets")),
            character: Some(PathBuf::from("/tmp/assets/character/v10.character.txt")),
            ..Default::default()
        };
        assert!(matches!(
            CharacterAssetPaths::resolve(spec),
            Err(CharacterAssetError::BadExtension(_))
        ));
    }

    #[test]
    fn args_win_over_their_equals_and_space_forms() {
        let args = [
            "--asset-root=/tmp/a",
            "--character",
            "/tmp/a/character/v10.character.json",
            "--character-svg",
            "/tmp/a/character/other.svg",
            "--unrelated",
            "x",
        ]
        .into_iter()
        .map(String::from);
        let spec = parse_args(args);
        assert_eq!(spec.asset_root, Some(PathBuf::from("/tmp/a")));
        assert_eq!(
            spec.character,
            Some(PathBuf::from("/tmp/a/character/v10.character.json"))
        );
        assert_eq!(spec.svg, Some(PathBuf::from("/tmp/a/character/other.svg")));
        assert_eq!(spec.json, None);
    }

    #[test]
    fn relative_paths_in_asset_layout_are_read_from_the_root() {
        let p = CharacterAssetPaths::resolve(CharacterAssetSpec {
            asset_root: Some(default_asset_root()),
            svg: Some(PathBuf::from(DEFAULT_SVG)),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(p.svg_rel, DEFAULT_SVG);
    }
}
