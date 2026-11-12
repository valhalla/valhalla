use std::path::Path;

/// Path to a file of the tileset built by the `utrecht_tiles` CMake target.
pub fn utrecht_tiles(file: &str) -> String {
    let path = format!(
        "{}/build/test/data/utrecht_tiles/{file}",
        env!("CARGO_MANIFEST_DIR")
    );
    assert!(
        Path::new(&path).exists(),
        "{path} is missing, build the `utrecht_tiles` CMake target first"
    );
    path
}
