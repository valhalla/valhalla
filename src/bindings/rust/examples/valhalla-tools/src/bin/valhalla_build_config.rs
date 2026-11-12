//! Generates a Valhalla config JSON from the defaults of `scripts/valhalla_build_config`, with
//! one `--<section>-<key>` option per config leaf, optionally merged into an existing config.
//!
//! Exit codes, as with the Python script: 0 on success, 1 when the config to merge cannot be
//! read or the output cannot be written, 2 when the arguments cannot be parsed.
#![forbid(unsafe_code)]
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented
)]

use std::env;
use std::io;
use std::process::ExitCode;

use valhalla_tools::build_config::{self, Parsed, UsageError};

fn main() -> ExitCode {
    let args: Result<Vec<String>, UsageError> = env::args_os()
        .skip(1)
        .map(|arg| {
            arg.into_string()
                .map_err(|arg| UsageError::NotUtf8(arg.to_string_lossy().into_owned()))
        })
        .collect();
    let arguments = match args.and_then(|args| build_config::parse_args(&args)) {
        Ok(Parsed::Run(arguments)) => arguments,
        Ok(Parsed::Help) => {
            return match build_config::command().print_help() {
                Ok(()) => ExitCode::SUCCESS,
                Err(_) => ExitCode::FAILURE,
            };
        }
        Err(error) => build_config::command().error(error.kind(), error).exit(),
    };
    match build_config::run(
        &arguments,
        &mut io::stdout().lock(),
        &mut io::stderr().lock(),
    ) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("Error: {error}");
            ExitCode::FAILURE
        }
    }
}
