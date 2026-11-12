//! Generates the option table of `valhalla_build_config` from `scripts/valhalla_build_config`,
//! which stays the single source of defaults and help texts.
use std::path::PathBuf;
use std::{env, fs};

#[path = "build/py_literal.rs"]
mod py_literal;

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

fn run() -> py_literal::Result<()> {
    let manifest_dir =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").ok_or("no CARGO_MANIFEST_DIR")?);
    let script = manifest_dir
        .ancestors()
        .nth(5)
        .ok_or("the crate is not 5 levels below the repository root")?
        .join("scripts/valhalla_build_config");
    println!("cargo:rerun-if-changed={}", script.display());
    let source = fs::read_to_string(&script)
        .map_err(|error| format!("cannot read {}: {error}", script.display()))?;
    let table =
        py_literal::generate(&source).map_err(|error| format!("{}: {error}", script.display()))?;
    let out = PathBuf::from(env::var_os("OUT_DIR").ok_or("no OUT_DIR")?).join("config_table.rs");
    fs::write(&out, table)?;
    Ok(())
}
