//! The Valhalla config JSON, merged the way the scripts merge `--config` and `--inline-config`.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use serde_json::{Map, Value};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid --inline-config JSON")]
    InlineConfig(#[source] serde_json::Error),
    #[error("{0} must be a JSON object")]
    NotObject(String),
    #[error("I/O error on {}", path.display())]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("invalid JSON in {}", path.display())]
    Json {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
}

/// Reads the config file, then lets the inline config replace its top-level keys, like
/// Python's `dict.update`.
pub fn merge(file: Option<&Path>, inline: Option<&str>) -> Result<Map<String, Value>, Error> {
    let mut config = Map::new();
    if let Some(path) = file {
        let text = fs::read_to_string(path).map_err(|source| Error::Io {
            path: path.to_path_buf(),
            source,
        })?;
        let value = serde_json::from_str(&text).map_err(|source| Error::Json {
            path: path.to_path_buf(),
            source,
        })?;
        config = into_object(value, || path.display().to_string())?;
    }
    if let Some(inline) = inline {
        let value = serde_json::from_str(inline).map_err(Error::InlineConfig)?;
        config.extend(into_object(value, || "--inline-config".to_owned())?);
    }
    Ok(config)
}

/// A top-level section of the merged config, `None` when the config has none.
pub fn section(
    file: Option<&Path>,
    inline: Option<&str>,
    key: &str,
) -> Result<Option<Value>, Error> {
    Ok(merge(file, inline)?.remove(key))
}

fn into_object(value: Value, origin: impl FnOnce() -> String) -> Result<Map<String, Value>, Error> {
    match value {
        Value::Object(object) => Ok(object),
        _ => Err(Error::NotObject(origin())),
    }
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use std::io::Write;

    use serde_json::json;

    use super::*;

    #[test]
    fn the_inline_config_replaces_whole_top_level_sections() {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        file.write_all(br#"{"a": {"x": 1, "y": 2}, "b": 3}"#)
            .unwrap();

        let config = merge(Some(file.path()), Some(r#"{"a": {"x": 4}}"#)).unwrap();
        assert_eq!(Value::Object(config), json!({"a": {"x": 4}, "b": 3}));
    }

    #[test]
    fn a_config_file_that_is_not_an_object_is_an_error() {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        file.write_all(b"[1, 2]").unwrap();
        assert!(matches!(
            merge(Some(file.path()), None),
            Err(Error::NotObject(_))
        ));
    }

    #[test]
    fn invalid_inline_json_is_an_error() {
        assert!(matches!(
            merge(None, Some("{")),
            Err(Error::InlineConfig(_))
        ));
    }

    #[test]
    fn a_missing_config_file_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let result = merge(Some(&dir.path().join("missing.json")), None);
        assert!(matches!(result, Err(Error::Io { .. })));
    }
}
