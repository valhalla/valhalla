//! Valhalla data tools that do not need libvalhalla: the libraries behind the
//! `valhalla_build_extract`, `valhalla_build_elevation` and `valhalla_build_config` binaries.
#![forbid(unsafe_code)]
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented
)]

pub mod build_config;
pub mod config;
pub mod elevation;
pub mod extract;
pub mod logging;

mod geojson;
mod geometry;
mod http;
mod paths;
mod python;
