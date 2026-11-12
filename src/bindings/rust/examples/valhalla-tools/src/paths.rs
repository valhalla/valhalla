use std::fs;
use std::path::{Path, PathBuf};

/// The absolute path, with symlinks resolved when it exists, like Python's `Path.resolve()`.
pub(crate) fn resolved(path: &Path) -> PathBuf {
    fs::canonicalize(path)
        .or_else(|_| std::path::absolute(path))
        .unwrap_or_else(|_| path.to_path_buf())
}

/// Python's `Path(text)`, where an empty path means the current directory.
pub(crate) fn python_path(path: &Path) -> PathBuf {
    if path.as_os_str().is_empty() {
        PathBuf::from(".")
    } else {
        path.to_path_buf()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_path_is_the_current_directory() {
        assert_eq!(python_path(Path::new("")), PathBuf::from("."));
    }
}
