use std::path::{Path, PathBuf};

use serde_json::{Map, Value};
use tracing::warn;

use super::Error;

/// The `mjolnir` paths the extract reads; empty strings count as unset.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct Mjolnir {
    pub(crate) tile_dir: Option<PathBuf>,
    pub(crate) tile_extract: Option<PathBuf>,
    pub(crate) traffic_extract: Option<PathBuf>,
}

/// Reads the config file, then lets the inline config replace its top-level keys.
pub(crate) fn load(file: Option<&Path>, inline: Option<&str>) -> Result<Mjolnir, Error> {
    if file.is_none() && inline.is_none_or(str::is_empty) {
        return Err(Error::NoConfig);
    }
    if file.is_none() {
        warn!("Only inline-config will be used.");
    }
    mjolnir(&crate::config::merge(file, inline)?)
}

fn mjolnir(config: &Map<String, Value>) -> Result<Mjolnir, Error> {
    let mjolnir = config
        .get("mjolnir")
        .and_then(Value::as_object)
        .ok_or(Error::MissingMjolnir)?;
    let path = |key: &'static str| match mjolnir.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(path)) if path.is_empty() => Ok(None),
        Some(Value::String(path)) => Ok(Some(PathBuf::from(path))),
        Some(_) => Err(Error::InvalidConfigValue { key }),
    };
    Ok(Mjolnir {
        tile_dir: path("tile_dir")?,
        tile_extract: path("tile_extract")?,
        traffic_extract: path("traffic_extract")?,
    })
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use std::io::Write;

    use super::*;

    #[test]
    fn an_inline_config_alone_is_enough() {
        let mjolnir = load(None, Some(r#"{"mjolnir": {"tile_dir": "tiles"}}"#)).unwrap();
        assert_eq!(mjolnir.tile_dir, Some(PathBuf::from("tiles")));
    }

    #[test]
    fn the_inline_config_replaces_whole_top_level_sections() {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        let file_config = r#"{"mjolnir": {"tile_dir": "tiles", "tile_extract": "tiles.tar"}}"#;
        file.write_all(file_config.as_bytes()).unwrap();

        let inline = r#"{"mjolnir": {"tile_extract": "other.tar"}}"#;
        let mjolnir = load(Some(file.path()), Some(inline)).unwrap();
        let expected = Mjolnir {
            tile_extract: Some(PathBuf::from("other.tar")),
            ..Mjolnir::default()
        };
        assert_eq!(mjolnir, expected);
    }

    #[test]
    fn no_config_at_all_is_an_error() {
        assert!(matches!(load(None, None), Err(Error::NoConfig)));
    }

    #[test]
    fn an_empty_inline_config_alone_is_no_config() {
        assert!(matches!(load(None, Some("")), Err(Error::NoConfig)));
    }

    #[test]
    fn a_config_without_mjolnir_is_an_error() {
        assert!(matches!(load(None, Some("{}")), Err(Error::MissingMjolnir)));
    }

    #[test]
    fn a_non_string_path_is_an_error() {
        let result = load(None, Some(r#"{"mjolnir": {"tile_dir": 3}}"#));
        assert!(matches!(
            result,
            Err(Error::InvalidConfigValue { key: "tile_dir" })
        ));
    }

    #[test]
    fn empty_paths_count_as_unset() {
        let mjolnir = load(None, Some(r#"{"mjolnir": {"tile_extract": ""}}"#)).unwrap();
        assert_eq!(mjolnir.tile_extract, None);
    }
}
