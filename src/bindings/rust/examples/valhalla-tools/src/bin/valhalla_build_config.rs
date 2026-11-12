//! Generates a Valhalla config JSON from the defaults of `scripts/valhalla_build_config`, with
//! one `--<section>-<key>` option per config leaf, optionally merged into an existing config.
//!
//! Exit codes, as with the Python script: 0 on success, 1 when the config to merge cannot be
//! read or the output cannot be written, 2 when the arguments cannot be parsed.
//!
//! Where it differs from the script:
//! - `Optional(bool)` options such as `--mjolnir-tile-url-gz` take the same spellings as the
//!   other bool options, so `false` is false; anything else, an empty value included, is a usage
//!   error. The script took any non-empty value as true.
//! - Arguments that are not UTF-8 are a usage error, except the paths of `-m` and `-o`.
//! - Paths are shown as Python shows POSIX paths; Windows path display is not emulated.
//! - A config to merge that the script cannot read, is not UTF-8, or that the output cannot be
//!   written to, is an `Error:` line and exit 1 where the script printed a traceback.
//! - The config to merge is refused, exit 1, when it nests deeper than 1000 levels, or holds an
//!   unpaired `\uD800`-`\uDFFF` escape, which Rust strings cannot hold.
//! - `--help` is laid out by clap.
#![forbid(unsafe_code)]
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented
)]

use std::env;
use std::ffi::OsString;
use std::io;
use std::process::ExitCode;

use valhalla_tools::build_config::{self, Parsed};

fn main() -> ExitCode {
    let args: Vec<OsString> = env::args_os().skip(1).collect();
    let arguments = match build_config::parse_args(&args) {
        Ok(Parsed::Run(arguments)) => arguments,
        Ok(Parsed::Help) => {
            return match build_config::command().print_help() {
                Ok(()) => ExitCode::SUCCESS,
                Err(_) => ExitCode::FAILURE,
            };
        }
        Err(error) => {
            // Printed as is: clap's error output would strip control characters of the message.
            let usage = build_config::command().render_usage();
            eprintln!("error: {error}\n\n{usage}\n\nFor more information, try '--help'.");
            return ExitCode::from(2);
        }
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
