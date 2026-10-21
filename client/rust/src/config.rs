use std::fs;
use std::path::{Path, PathBuf};

use crate::error::{Result, SimError};

/// Strips JavaScript-style comments (`// line` and `/* block */`) from JSONC text,
/// preserving strings and escape sequences.
pub fn strip_jsonc_comments(input: &str) -> String {
    #[derive(Copy, Clone, PartialEq, Eq)]
    enum State {
        Normal,
        InString,
        InEscape,
        SawSlash,
        InLineComment,
        InBlockComment,
        SawStarInBlock,
    }

    let mut out = String::with_capacity(input.len());
    let mut state = State::Normal;

    for ch in input.chars() {
        match state {
            State::Normal => match ch {
                '"' => {
                    out.push('"');
                    state = State::InString;
                }
                '/' => {
                    state = State::SawSlash;
                }
                other => {
                    out.push(other);
                }
            },
            State::InString => match ch {
                '\\' => {
                    out.push('\\');
                    state = State::InEscape;
                }
                '"' => {
                    out.push('"');
                    state = State::Normal;
                }
                other => {
                    out.push(other);
                }
            },
            State::InEscape => {
                out.push(ch);
                state = State::InString;
            }
            State::SawSlash => match ch {
                '/' => {
                    state = State::InLineComment;
                }
                '*' => {
                    state = State::InBlockComment;
                }
                '"' => {
                    out.push('/');
                    out.push('"');
                    state = State::InString;
                }
                other => {
                    out.push('/');
                    out.push(other);
                    state = State::Normal;
                }
            },
            State::InLineComment => {
                if ch == '\n' {
                    out.push('\n');
                    state = State::Normal;
                }
            }
            State::InBlockComment => {
                if ch == '*' {
                    state = State::SawStarInBlock;
                }
            }
            State::SawStarInBlock => match ch {
                '/' => {
                    state = State::Normal;
                }
                '*' => {
                    // remain in SawStarInBlock
                }
                _ => {
                    state = State::InBlockComment;
                }
            },
        }
    }

    if state == State::SawSlash {
        out.push('/');
    }

    out
}

/// Parses a JSONC (JSON with Comments) string into a `serde_json::Value`.
pub fn parse_jsonc(input: &str) -> Result<serde_json::Value> {
    let clean = strip_jsonc_comments(input);
    serde_json::from_str(&clean).map_err(|e| {
        SimError::SerializationError(format!("Failed to parse JSONC content: {e}"))
    })
}

/// Loads a JSONC file from disk, stripping comments and returning parsed JSON.
pub fn load_jsonc_file(path: impl AsRef<Path>) -> Result<serde_json::Value> {
    let path_ref = path.as_ref();
    let content = fs::read_to_string(path_ref).map_err(|e| {
        SimError::SerializationError(format!(
            "Failed to read file '{}': {e}",
            path_ref.display()
        ))
    })?;
    parse_jsonc(&content)
}

/// Loads and recursively resolves a ProjectAirSim scene configuration file.
///
/// Parallels `World::Impl::LoadSceneConfig` in the C++ client:
/// - Strips JSONC comments from the scene config file.
/// - Appends optional `sim_instance_idx` to `id`.
/// - Recursively inlines referenced `robot-config` files in `actors`.
/// - Recursively inlines referenced `env-actor-config` files in `environment-actors`.
/// - Resolves relative `tiles-dir` to absolute path if `tiles-dir-is-client-relative` is true.
pub fn load_scene_config(
    scene_config_path: impl AsRef<Path>,
    sim_config_path: Option<impl AsRef<Path>>,
    sim_instance_idx: Option<i32>,
) -> Result<serde_json::Value> {
    let scene_path = scene_config_path.as_ref();
    let canonical_scene = fs::canonicalize(scene_path).map_err(|e| {
        SimError::SerializationError(format!(
            "Cannot find scene configuration '{}': {e}",
            scene_path.display()
        ))
    })?;

    let scene_dir = canonical_scene
        .parent()
        .unwrap_or_else(|| Path::new("."));

    let base_sim_config: PathBuf = match sim_config_path {
        Some(p) => {
            let p_ref = p.as_ref();
            if p_ref.is_absolute() {
                p_ref.to_path_buf()
            } else {
                scene_dir.join(p_ref)
            }
        }
        None => scene_dir.join("sim_config"),
    };

    let mut scene_json = load_jsonc_file(&canonical_scene)?;

    // Append sim_instance_idx if given and != -1
    if let Some(idx) = sim_instance_idx {
        if idx >= 0 {
            if let Some(id_val) = scene_json.get("id").and_then(|v| v.as_str()) {
                scene_json["id"] = serde_json::Value::String(format!("{id_val}-{idx}"));
            }
        }
    }

    // Expand robot configs in actors
    if let Some(actors) = scene_json.get_mut("actors").and_then(|a| a.as_array_mut()) {
        for actor in actors {
            let is_robot = actor.get("type").and_then(|t| t.as_str()) == Some("robot");
            if is_robot {
                if let Some(cfg_str) = actor.get("robot-config").and_then(|c| c.as_str()) {
                    let sub_path = resolve_config_path(cfg_str, &base_sim_config, scene_dir);
                    let sub_json = load_jsonc_file(&sub_path)?;
                    actor["robot-config"] = sub_json;
                }
            }
        }
    }

    // Expand env-actor configs
    if let Some(env_actors) = scene_json
        .get_mut("environment-actors")
        .and_then(|a| a.as_array_mut())
    {
        for actor in env_actors {
            let is_env = actor.get("type").and_then(|t| t.as_str()) == Some("env_actor");
            if is_env {
                if let Some(cfg_str) = actor.get("env-actor-config").and_then(|c| c.as_str()) {
                    let sub_path = resolve_config_path(cfg_str, &base_sim_config, scene_dir);
                    let sub_json = load_jsonc_file(&sub_path)?;
                    actor["env-actor-config"] = sub_json;
                }
            }
        }
    }

    // Resolve client-relative tiles-dir
    if scene_json
        .get("tiles-dir-is-client-relative")
        .and_then(|v| v.as_bool())
        == Some(true)
    {
        if let Some(tiles_dir) = scene_json.get("tiles-dir").and_then(|v| v.as_str()) {
            let p = Path::new(tiles_dir);
            let abs_path = if p.is_absolute() {
                p.to_path_buf()
            } else {
                scene_dir.join(p)
            };
            if let Ok(canon) = fs::canonicalize(&abs_path) {
                scene_json["tiles-dir"] =
                    serde_json::Value::String(canon.to_string_lossy().to_string());
            } else {
                scene_json["tiles-dir"] =
                    serde_json::Value::String(abs_path.to_string_lossy().to_string());
            }
        }
    }

    Ok(scene_json)
}

fn resolve_config_path(cfg_str: &str, base_sim_config: &Path, scene_dir: &Path) -> PathBuf {
    let p = Path::new(cfg_str);
    if p.is_absolute() && p.exists() {
        return p.to_path_buf();
    }
    let sim_joined = base_sim_config.join(cfg_str);
    if sim_joined.exists() {
        return sim_joined;
    }
    let scene_joined = scene_dir.join(cfg_str);
    if scene_joined.exists() {
        return scene_joined;
    }
    // Default to sim_config joined
    sim_joined
}
