//! Generates a Valhalla config, as `scripts/valhalla_build_config` does: its defaults, every leaf
//! overridable by an option, optionally merged into an existing config.
//!
//! The defaults and help texts are read from the script at build time.

mod args;
mod json;
mod merge;
mod table;

// The build script's parser, tested here as build scripts have no tests.
#[cfg(test)]
#[path = "../../build/py_literal.rs"]
mod py_literal;

use std::fs;
use std::io::{self, Write};
use std::path::PathBuf;

use indexmap::IndexMap;

pub use args::{Arguments, Parsed, UsageError, command, parse_args};
pub use json::{MAX_DEPTH, ParseError, Value, dumps, parse};
pub use merge::{MergeReport, merge_json, should_remove_key};
pub use table::{DefaultValue, LEAVES, Leaf, Literal, OptionalKind};

use crate::paths::pure_path;

/// Where the script has a message for the same situation, the wording is the script's.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Config file not found: {0}")]
    NotFound(String),
    #[error("cannot read {path}: {source}")]
    Read { path: String, source: io::Error },
    #[error("{path} is not valid UTF-8: {source}")]
    NotUtf8 {
        path: String,
        source: std::string::FromUtf8Error,
    },
    #[error("Invalid JSON in config: {0}")]
    InvalidJson(#[from] ParseError),
    #[error("cannot write {path}: {source}")]
    Write { path: String, source: io::Error },
    #[error("cannot write to stdout: {0}")]
    Stdout(io::Error),
    #[error("cannot write to stderr: {0}")]
    Stderr(io::Error),
}

/// The defaults with the given options applied, Optional leaves left out unless given: the
/// script's `override_config`.
pub fn config(values: &[Option<Value>]) -> Value {
    let mut root = IndexMap::new();
    for (leaf, value) in LEAVES.iter().zip(values) {
        let value = match (value, &leaf.default) {
            (Some(value), _) => Some(value.clone()),
            (None, DefaultValue::Value(literal)) => Some(literal.to_value()),
            (None, DefaultValue::Optional(_)) => None,
        };
        insert(&mut root, leaf.path, value);
    }
    Value::Dict(root)
}

/// Creates the dicts of `path` even when `value` is `None`, as the script keeps a dict whose
/// leaves are all left out.
fn insert(node: &mut IndexMap<String, Value>, path: &[&str], value: Option<Value>) {
    match path {
        [] => {}
        [last] => {
            if let Some(value) = value {
                node.insert((*last).to_owned(), value);
            }
        }
        [first, rest @ ..] => {
            let child = node
                .entry((*first).to_owned())
                .or_insert_with(|| Value::Dict(IndexMap::new()));
            // Never a leaf: the script's leaves are never above other leaves.
            if let Value::Dict(child) = child {
                insert(child, rest, value);
            }
        }
    }
}

/// Builds the config the arguments describe and writes it to stdout or `--output`, with the
/// merge report or warning on `stderr`.
pub fn run(
    arguments: &Arguments,
    stdout: &mut dyn Write,
    stderr: &mut dyn Write,
) -> Result<(), Error> {
    let mut output_config = config(&arguments.values);
    let mut report = None;
    if let Some(path) = &arguments.merge_config {
        let path = pure_path(path);
        let existing = read_config(&path)?;
        let mut merge_report = MergeReport::default();
        output_config = merge_json(
            &existing,
            &output_config,
            &mut merge_report,
            arguments.remove_old.as_deref(),
            "",
        );
        report = Some(merge_report);
    }
    if arguments.report {
        let text = match &report {
            Some(report) => report.render(),
            None => "Warn: --report requires --merge-config\n".to_owned(),
        };
        stderr.write_all(text.as_bytes()).map_err(Error::Stderr)?;
    }
    let mut text = dumps(&output_config);
    text.push('\n');
    match &arguments.output {
        Some(path) => {
            let path = pure_path(path);
            // Python's text mode writes `\r\n` on Windows.
            if cfg!(windows) {
                text = text.replace('\n', "\r\n");
            }
            fs::write(&path, text).map_err(|source| Error::Write {
                path: path.clone(),
                source,
            })?;
            writeln!(stderr, "Config written to: {path}").map_err(Error::Stderr)?;
        }
        None => {
            stdout.write_all(text.as_bytes()).map_err(Error::Stdout)?;
            stdout.flush().map_err(Error::Stdout)?;
        }
    }
    Ok(())
}

fn read_config(path: &str) -> Result<Value, Error> {
    let bytes = fs::read(PathBuf::from(path)).map_err(|source| {
        if source.kind() == io::ErrorKind::NotFound {
            Error::NotFound(path.to_owned())
        } else {
            Error::Read {
                path: path.to_owned(),
                source,
            }
        }
    })?;
    let text = String::from_utf8(bytes).map_err(|source| Error::NotUtf8 {
        path: path.to_owned(),
        source,
    })?;
    Ok(parse(&text)?)
}
