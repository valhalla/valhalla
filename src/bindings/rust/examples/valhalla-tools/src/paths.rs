use std::fs;
use std::path::{Component, Path, PathBuf};

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

/// `PurePosixPath(path)`: repeated separators, `.` components and a trailing separator dropped,
/// `.` for nothing left, and a leading `//`, not more, kept as POSIX allows. Windows paths get
/// the same treatment: `PureWindowsPath` is not emulated.
pub(crate) fn pure_path(path: &Path) -> PathBuf {
    let text = path.as_os_str().as_encoded_bytes();
    let double_root = text.starts_with(b"//") && !text.starts_with(b"///");
    let mut pure = PathBuf::from(if double_root { "//" } else { "" });
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::RootDir if double_root => {}
            component => pure.push(component),
        }
    }
    if pure.as_os_str().is_empty() {
        PathBuf::from(".")
    } else {
        pure
    }
}

/// `str(path)` as Python prints it to stderr: bytes that are not UTF-8 become `\udcXX`.
#[cfg(unix)]
pub(crate) fn python_display(path: &Path) -> String {
    use std::os::unix::ffi::OsStrExt;
    let mut text = String::new();
    for chunk in path.as_os_str().as_bytes().utf8_chunks() {
        text.push_str(chunk.valid());
        for byte in chunk.invalid() {
            text.push_str(&format!("\\udc{byte:02x}"));
        }
    }
    text
}

#[cfg(not(unix))]
pub(crate) fn python_display(path: &Path) -> String {
    path.display().to_string()
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
        let paths = ["", "a//b/./c/", "./x", "///a", "//a/", "/", "../a/..", "//"];
        assert_eq!(
            paths.map(|path| pure_path(Path::new(path))),
            [".", "a/b/c", "x", "/a", "//a", "/", "../a/..", "//"].map(PathBuf::from)
        );
    }

    #[cfg(unix)]
    #[test]
    fn paths_that_are_not_utf8_display_as_surrogate_escapes() {
        use std::ffi::OsStr;
        use std::os::unix::ffi::OsStrExt;
        let path = Path::new(OsStr::from_bytes(b"caf\xe9/\xff.json"));
        assert_eq!(python_display(path), "caf\\udce9/\\udcff.json");
    }
}
