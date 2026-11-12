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

/// `str(PurePosixPath(text))`: repeated slashes, `.` components and a trailing slash dropped,
/// `.` for nothing left, and a leading `//`, not more, kept as POSIX allows.
pub(crate) fn pure_path(text: &str) -> String {
    let root = if text.starts_with("//") && !text.starts_with("///") {
        "//"
    } else if text.starts_with('/') {
        "/"
    } else {
        ""
    };
    let parts: Vec<&str> = text
        .split('/')
        .filter(|part| !part.is_empty() && *part != ".")
        .collect();
    let path = format!("{root}{}", parts.join("/"));
    if path.is_empty() {
        ".".to_owned()
    } else {
        path
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_path_is_the_current_directory() {
        assert_eq!(python_path(Path::new("")), PathBuf::from("."));
    }

    #[test]
    fn pure_paths_are_normalized_like_pathlib() {
        assert_eq!(
            ["", "a//b/./c/", "./x", "///a", "//a/", "/", "../a/.."].map(pure_path),
            [".", "a/b/c", "x", "/a", "//a", "/", "../a/.."]
        );
    }
}
