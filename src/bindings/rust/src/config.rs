use std::{os::unix::ffi::OsStrExt, path::Path};

use crate::Error;

#[cxx::bridge]
pub(crate) mod ffi {
    unsafe extern "C++" {
        include!("valhalla/src/config.hpp");

        #[namespace = "boost::property_tree"]
        type ptree;
        fn from_file(path: &[u8]) -> Result<UniquePtr<ptree>>;
        fn from_json(config: &str) -> Result<UniquePtr<ptree>>;

        fn ptree_new() -> UniquePtr<ptree>;
        fn ptree_put_str(pt: Pin<&mut ptree>, path: &str, value: &str);
        fn ptree_put_bool(pt: Pin<&mut ptree>, path: &str, value: bool);
        fn ptree_put_int(pt: Pin<&mut ptree>, path: &str, value: i64);
        fn ptree_put_float(pt: Pin<&mut ptree>, path: &str, value: f64);
        fn ptree_put_str_array(pt: Pin<&mut ptree>, path: &str, values: &[String]);
        fn ptree_put_int_array(pt: Pin<&mut ptree>, path: &str, values: &[i64]);

        /// Applies the config's `logging` section to Valhalla's logger.
        fn configure_logging(pt: &ptree);
    }
}

/// Wrapper around Valhalla configuration.
///
/// The recommended way to create a `Config` is via [`ConfigBuilder`], which
/// auto-generates all Valhalla defaults at build time:
///
/// ```
/// let config = valhalla::ConfigBuilder {
///     mjolnir: valhalla::config::Mjolnir {
///         tile_extract: "./tests/andorra/tiles.tar".into(),
///         traffic_extract: "./tests/andorra/traffic.tar".into(),
///         ..Default::default()
///     },
///     ..Default::default()
/// }
/// .build();
/// ```
///
/// Alternatively, [`Config::from_file`] or [`Config::from_json`] can be used to parse an existing
/// config JSON, created by [`valhalla_build_config`] script:
///
/// ```
/// let Ok(config) = valhalla::Config::from_file("path/to/config.json") else {
///     return; // Handle error appropriately
/// };
/// ```
///
/// N.B.: [`Config::from_file`] and [`Config::from_json`] are provided for convenience but their
/// use is **fragile**: the expected JSON schema may change between Valhalla versions, causing
/// hard-to-diagnose runtime failures.
/// Prefer [`ConfigBuilder`] for forward-compatible configuration.
///
/// `valhalla_build_config`: https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config
pub struct Config(cxx::UniquePtr<ffi::ptree>);

impl Config {
    /// Reads configuration from the given Valhalla configuration file.
    ///
    /// **Warning**: The JSON configuration format may change between Valhalla versions.
    /// Prefer [`ConfigBuilder`] which automatically tracks Valhalla's defaults.
    ///
    /// # Examples
    ///
    /// ```
    /// let config = valhalla::Config::from_file("path/to/config.json");
    /// ```
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, Error> {
        Ok(Config::new(ffi::from_file(
            path.as_ref().as_os_str().as_bytes(),
        )?))
    }

    /// Reads configuration from Valhalla configuration JSON string.
    ///
    /// **Warning**: The JSON configuration format may change between Valhalla versions.
    /// Prefer [`ConfigBuilder`] which automatically tracks Valhalla's defaults.
    ///
    /// # Examples
    ///
    /// ```
    /// let json = r#"{"mjolnir":{"tile_extract":"path/to/tiles.tar","traffic_extract":"path/to/traffic.tar"}}"#;
    /// let config = valhalla::Config::from_json(&json);
    /// ```
    pub fn from_json(config: &str) -> Result<Self, Error> {
        Ok(Config::new(ffi::from_json(config)?))
    }

    /// Creates a new Valhalla configuration from path to the tiles tar extract.
    ///
    /// This is a convenience shortcut for the common case of only needing tile data.
    /// For more control, use [`ConfigBuilder`].
    ///
    /// # Examples
    ///
    /// ```
    /// let config = valhalla::Config::from_tile_extract("path/to/tiles.tar");
    /// ```
    pub fn from_tile_extract(tile_extract: impl AsRef<Path>) -> Result<Self, Error> {
        let config = ConfigBuilder {
            mjolnir: Mjolnir {
                tile_extract: tile_extract.as_ref().display().to_string(),
                ..Default::default()
            },
            ..Default::default()
        }
        .build();
        Ok(config)
    }

    pub(crate) fn new(pt: cxx::UniquePtr<ffi::ptree>) -> Self {
        // Valhalla has static global state for logging so this is the earliest we can call it. No-op for subsequent calls.
        ffi::configure_logging(pt.as_ref().unwrap());
        Config(pt)
    }

    /// Reference to the inner Valhalla configuration object.
    pub(crate) fn inner(&self) -> &ffi::ptree {
        self.0.as_ref().unwrap()
    }
}

// ConfigBuilder and all supporting types.
include!(concat!(env!("OUT_DIR"), "/config_builder.rs"));
