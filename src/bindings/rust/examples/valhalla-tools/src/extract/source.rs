use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{self, Read, Seek, SeekFrom, Take};
use std::path::{Path, PathBuf};

use super::Error;
use super::archive::Member;
use super::tile::Tile;

#[derive(Debug)]
pub(crate) enum TileSource {
    Dir(PathBuf),
    Tar(PathBuf),
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct SourceTile {
    /// Path relative to the tile directory, with `/` separators.
    pub(crate) name: String,
    pub(crate) position: Tile,
    pub(crate) size: u64,
    /// Where the tile data starts in its file: 0 in a tile directory, past the member header in a tar.
    data_offset: u64,
}

impl TileSource {
    pub(crate) fn path(&self) -> &Path {
        match self {
            Self::Dir(path) | Self::Tar(path) => path,
        }
    }

    /// All `.gph` tiles of the source, sorted by path component.
    pub(crate) fn list_tiles(&self) -> Result<Vec<SourceTile>, Error> {
        let mut tiles = match self {
            Self::Dir(dir) => list_dir(dir)?,
            Self::Tar(tar) => list_tar(tar)?,
        };
        tiles.sort_by(|a, b| a.name.split('/').cmp(b.name.split('/')));
        Ok(tiles)
    }

    pub(crate) fn read_tile(&self, tile: &SourceTile) -> Result<Member<Take<File>>, Error> {
        let path = match self {
            Self::Dir(dir) => dir.join(&tile.name),
            Self::Tar(tar) => tar.clone(),
        };
        let open = || -> io::Result<Take<File>> {
            let mut file = File::open(&path)?;
            file.seek(SeekFrom::Start(tile.data_offset))?;
            Ok(file.take(tile.size))
        };
        let content = open().map_err(Error::io(&path))?;
        Ok(Member {
            name: tile.name.clone(),
            tile_id: tile.position.id(),
            size: tile.size,
            content,
        })
    }
}

fn list_dir(root: &Path) -> Result<Vec<SourceTile>, Error> {
    let mut tiles = Vec::new();
    let mut pending = vec![root.to_path_buf()];
    while let Some(dir) = pending.pop() {
        for entry in fs::read_dir(&dir).map_err(Error::io(&dir))? {
            let entry = entry.map_err(Error::io(&dir))?;
            let path = entry.path();
            if entry.file_type().map_err(Error::io(&path))?.is_dir() {
                pending.push(path);
                continue;
            }
            if path.extension().is_none_or(|extension| extension != "gph") {
                continue;
            }
            let metadata = fs::metadata(&path).map_err(Error::io(&path))?;
            if !metadata.is_file() {
                continue;
            }
            let name = relative_name(root, &path);
            tiles.push(SourceTile {
                position: Tile::from_path(&name)?,
                name,
                size: metadata.len(),
                data_offset: 0,
            });
        }
    }
    Ok(tiles)
}

fn relative_name(root: &Path, path: &Path) -> String {
    let relative = path.strip_prefix(root).unwrap_or(path);
    let parts: Vec<_> = relative
        .components()
        .map(|part| part.as_os_str().to_string_lossy())
        .collect();
    parts.join("/")
}

fn list_tar(path: &Path) -> Result<Vec<SourceTile>, Error> {
    let file = File::open(path).map_err(Error::io(path))?;
    let mut archive = tar::Archive::new(file);
    // A later member replaces an earlier one with the same name, as when extracting.
    let mut by_name = HashMap::new();
    for entry in archive.entries().map_err(Error::io(path))? {
        let entry = entry.map_err(Error::io(path))?;
        let name = String::from_utf8_lossy(&entry.path_bytes()).into_owned();
        if !entry.header().entry_type().is_file() || !name.ends_with(".gph") {
            continue;
        }
        let tile = SourceTile {
            position: Tile::from_path(&name)?,
            name: name.clone(),
            size: entry.size(),
            data_offset: entry.raw_file_position(),
        };
        by_name.insert(name, tile);
    }
    Ok(by_name.into_values().collect())
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    fn write_tile(root: &Path, name: &str, size: usize) {
        let path = root.join(name);
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, vec![7; size]).unwrap();
    }

    #[test]
    fn tiles_are_listed_in_path_component_order() {
        let root = tempfile::tempdir().unwrap();
        for name in ["2/000/818/660.gph", "0/003/196.gph", "1/051/305.gph"] {
            write_tile(root.path(), name, 64);
        }
        let tiles = TileSource::Dir(root.path().to_path_buf())
            .list_tiles()
            .unwrap();
        let names: Vec<_> = tiles.iter().map(|tile| tile.name.as_str()).collect();
        assert_eq!(
            names,
            ["0/003/196.gph", "1/051/305.gph", "2/000/818/660.gph"]
        );
    }

    #[test]
    fn files_other_than_tiles_are_ignored() {
        let root = tempfile::tempdir().unwrap();
        write_tile(root.path(), "0/003/196.gph", 64);
        write_tile(root.path(), "0/003/196.gph.lock", 1);
        write_tile(root.path(), "README", 1);
        let tiles = TileSource::Dir(root.path().to_path_buf())
            .list_tiles()
            .unwrap();
        assert_eq!(tiles.len(), 1);
    }

    #[test]
    fn a_misplaced_tile_is_an_error() {
        let root = tempfile::tempdir().unwrap();
        write_tile(root.path(), "backup/196.gph", 64);
        let listed = TileSource::Dir(root.path().to_path_buf()).list_tiles();
        assert!(matches!(listed, Err(Error::InvalidTilePath { .. })));
    }
}
