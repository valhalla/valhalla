use std::{io::Write, path::PathBuf};

use miniserde::{Serialize, json};
use tempfile::NamedTempFile;

#[cfg(feature = "proto")]
use valhalla::Actor;
use valhalla::{Config, ConfigBuilder, GraphReader};

/// Small subset of `valhalla::ConfigBuilder` to test building a config from JSON.
#[derive(Serialize)]
struct ValhallaConfig {
    mjolnir: MjolnirConfig,
}

#[derive(Serialize)]
struct MjolnirConfig {
    tile_extract: String,
    traffic_extract: String,
}

const ANDORRA_TILES: &str = "tests/andorra/tiles.tar";
const ANDORRA_TRAFFIC: &str = "tests/andorra/traffic.tar";

#[test]
fn from_file() {
    assert!(Config::from_file(PathBuf::default()).is_err());

    // bad json
    let mut file = NamedTempFile::new().expect("Failed to create temp file for config");
    file.write_all("{{{".as_bytes())
        .expect("Failed to write config");
    assert!(Config::from_file(file.path()).is_err());

    // bad config
    let config = ValhallaConfig {
        mjolnir: MjolnirConfig {
            tile_extract: "bad_path_to_tile_extract".into(),
            traffic_extract: "bad_path_to_traffic_extract".into(),
        },
    };
    let mut file = NamedTempFile::new().expect("Failed to create temp file for config");
    file.write_all(json::to_string(&config).as_bytes())
        .expect("Failed to write config");
    // Config just ensures that the file is valid JSON. `GraphReader` will check the paths.
    let config = Config::from_file(file.path()).unwrap();
    assert!(GraphReader::new(&config).is_err());

    // tiles only
    let config = ValhallaConfig {
        mjolnir: MjolnirConfig {
            tile_extract: ANDORRA_TILES.into(),
            traffic_extract: "bad_path_to_traffic_extract".into(), // Bad traffic extract
        },
    };
    let mut file = NamedTempFile::new().expect("Failed to create temp file for config");
    file.write_all(json::to_string(&config).as_bytes())
        .expect("Failed to write config");
    // Config just ensures that the file is valid JSON. `GraphReader` will check the paths.
    let config = Config::from_file(file.path()).unwrap();
    assert!(GraphReader::new(&config).is_ok());

    // traffic only
    let config = ValhallaConfig {
        mjolnir: MjolnirConfig {
            tile_extract: "bad_path_to_tile_extract".into(), // No tile extract
            traffic_extract: ANDORRA_TRAFFIC.into(),
        },
    };
    let mut file = NamedTempFile::new().expect("Failed to create temp file for config");
    file.write_all(json::to_string(&config).as_bytes())
        .expect("Failed to write config");
    // Config just ensures that the file is valid JSON. `GraphReader` will check the paths.
    let config = Config::from_file(file.path()).unwrap();
    assert!(GraphReader::new(&config).is_err());

    // tiles and traffic
    let config = ValhallaConfig {
        mjolnir: MjolnirConfig {
            tile_extract: ANDORRA_TILES.into(),
            traffic_extract: ANDORRA_TRAFFIC.into(),
        },
    };
    let mut file = NamedTempFile::new().expect("Failed to create temp file for config");
    file.write_all(json::to_string(&config).as_bytes())
        .expect("Failed to write config");
    // Config just ensures that the file is valid JSON. `GraphReader` will check the paths.
    let config = Config::from_file(file.path()).unwrap();
    assert!(GraphReader::new(&config).is_ok());
}

#[test]
fn from_json() {
    assert!(Config::from_json("").is_err());
    assert!(Config::from_json("{").is_err());
    assert!(Config::from_json("}").is_err());
    assert!(Config::from_json("{}").is_ok());

    // bad config
    let config = ValhallaConfig {
        mjolnir: MjolnirConfig {
            tile_extract: "bad_path_to_tile_extract".into(),
            traffic_extract: "bad_path_to_traffic_extract".into(),
        },
    };
    let config = Config::from_json(&json::to_string(&config)).unwrap();
    assert!(GraphReader::new(&config).is_err());

    // tiles and traffic ok
    let config = ValhallaConfig {
        mjolnir: MjolnirConfig {
            tile_extract: ANDORRA_TILES.into(),
            traffic_extract: ANDORRA_TRAFFIC.into(),
        },
    };
    let config = Config::from_json(&json::to_string(&config)).unwrap();
    assert!(GraphReader::new(&config).is_ok());
}

#[test]
fn from_tile_extract() {
    assert!(GraphReader::new(&Config::from_tile_extract("").unwrap()).is_err());
    assert!(
        GraphReader::new(&Config::from_tile_extract("bad_path_to_tile_extract").unwrap()).is_err()
    );
    assert!(GraphReader::new(&Config::from_tile_extract(ANDORRA_TILES).unwrap()).is_ok());
    #[cfg(feature = "proto")]
    assert!(Actor::new(&Config::from_tile_extract(ANDORRA_TILES).unwrap()).is_ok());
}

#[test]
fn config_builder() {
    // Verify defaults match valhalla_build_config
    let builder = ConfigBuilder::default();
    assert_eq!(builder.mjolnir.tile_extract, "/data/valhalla/tiles.tar");
    assert_eq!(
        builder.mjolnir.traffic_extract,
        "/data/valhalla/traffic.tar"
    );
    assert!(builder.mjolnir.hierarchy);
    assert!(builder.mjolnir.shortcuts);
    assert_eq!(builder.mjolnir.max_cache_size, 1000000000);
    assert!(!builder.loki.actions.is_empty());
    assert!(!builder.meili.customizable.is_empty());

    // Default paths don't exist, so GraphReader should fail, but config itself is valid
    let config = builder.build();
    assert!(GraphReader::new(&config).is_err());

    // bad config
    let mut builder = ConfigBuilder::default();
    builder.mjolnir.tile_extract = "bad_path_to_tile_extract".into();
    builder.mjolnir.traffic_extract = "bad_path_to_traffic_extract".into();
    let config = builder.build();
    assert!(GraphReader::new(&config).is_err());

    // tiles only
    let mut builder = ConfigBuilder::default();
    builder.mjolnir.tile_extract = ANDORRA_TILES.into();
    builder.mjolnir.traffic_extract = "bad_path_to_traffic_extract".into();
    let config = builder.build();
    assert!(GraphReader::new(&config).is_ok());

    // traffic only
    let mut builder = ConfigBuilder::default();
    builder.mjolnir.tile_extract = "bad_path_to_tile_extract".into();
    builder.mjolnir.traffic_extract = ANDORRA_TRAFFIC.into();
    let config = builder.build();
    assert!(GraphReader::new(&config).is_err());

    // tiles and traffic
    let mut builder = ConfigBuilder::default();
    builder.mjolnir.tile_extract = ANDORRA_TILES.into();
    builder.mjolnir.traffic_extract = ANDORRA_TRAFFIC.into();
    let config = builder.build();
    assert!(GraphReader::new(&config).is_ok());
    #[cfg(feature = "proto")]
    assert!(Actor::new(&config).is_ok());

    // build() takes &self, so the builder is reusable
    let config2 = builder.build();
    #[cfg(feature = "proto")]
    assert!(Actor::new(&config2).is_ok());
    let _ = config2;

    // Break the config by pointing to non-existent tiles
    builder.mjolnir.tile_extract = "bad_path_to_tile_extract".into();
    let config = builder.build();
    assert!(GraphReader::new(&config).is_err());
    #[cfg(feature = "proto")]
    assert!(Actor::new(&config).is_err());
}
