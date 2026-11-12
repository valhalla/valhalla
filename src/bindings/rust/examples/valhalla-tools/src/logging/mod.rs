//! Log line formats and destinations shared by the binaries, which install the subscriber.

mod format;
mod valhalla;

use std::path::Path;

pub use format::{LineFormat, LogFormat};
pub use valhalla::{LineWriter, LogFile, LogTarget, Logger, from_section};

/// Where the given format logs: the Python format writes to stderr, the Valhalla format follows
/// the config's top-level `logging` section as the C++ tools do.
pub fn logger(
    format: LogFormat,
    config: Option<&Path>,
    inline_config: Option<&str>,
) -> Result<Logger, String> {
    if format == LogFormat::Python {
        return Ok(Logger::stderr());
    }
    match crate::config::section(config, inline_config, "logging") {
        Ok(section) => from_section(section.as_ref()),
        // The run reports the unreadable config, through the logger the C++ would start with.
        Err(_) => Ok(Logger::default_console()),
    }
}
