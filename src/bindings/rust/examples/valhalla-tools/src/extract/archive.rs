use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use tempfile::NamedTempFile;
use tracing::debug;

use super::Error;

const BLOCK_SIZE: u64 = 512;
const INDEX_NAME: &str = "index.bin";
const INDEX_ENTRY_SIZE: u64 = 16;
/// Two zero blocks mark the end of a tar.
const END_OF_ARCHIVE: [u8; 1024] = [0; 1024];

/// One `index.bin` record, laid out like `tile_index_entry` in baldr/graphreader.cc.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct IndexEntry {
    /// Absolute offset of the tile data in the tar.
    pub(crate) offset: u64,
    pub(crate) tile_id: u32,
    pub(crate) size: u32,
}

impl IndexEntry {
    pub(crate) fn to_bytes(self) -> [u8; 16] {
        let mut bytes = [0; 16];
        bytes[..8].copy_from_slice(&self.offset.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.tile_id.to_le_bytes());
        bytes[12..].copy_from_slice(&self.size.to_le_bytes());
        bytes
    }
}

/// A tile to store: `content` must yield exactly `size` bytes.
pub(crate) struct Member<R> {
    pub(crate) name: String,
    pub(crate) tile_id: u32,
    pub(crate) size: u64,
    pub(crate) content: R,
}

#[derive(Debug)]
pub(crate) struct ArchivedTile {
    pub(crate) name: String,
    pub(crate) entry: IndexEntry,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum WriteMode {
    /// Build next to the output and rename it over the output once complete.
    Replace,
    /// Truncate and rewrite the output, following symlinks and keeping its owner and permissions.
    InPlace,
}

enum OutputFile {
    Temporary(NamedTempFile),
    InPlace(File),
}

impl OutputFile {
    fn create(output: &Path, mode: WriteMode) -> io::Result<Self> {
        fs::create_dir_all(parent_dir(output))?;
        match mode {
            WriteMode::Replace => create_temp_file(output).map(Self::Temporary),
            WriteMode::InPlace => OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open(output)
                .map(Self::InPlace),
        }
    }

    fn file_mut(&mut self) -> &mut File {
        match self {
            Self::Temporary(temp) => temp.as_file_mut(),
            Self::InPlace(file) => file,
        }
    }

    fn finish(self, output: &Path) -> io::Result<()> {
        match self {
            Self::Temporary(temp) => temp.persist(output).map(drop).map_err(|error| error.error),
            Self::InPlace(_) => Ok(()),
        }
    }
}

/// Writes `index.bin`, then one member per tile in order.
pub(crate) fn write_indexed_tar<T, R: Read>(
    output: &Path,
    mode: WriteMode,
    tiles: &[T],
    mut member: impl FnMut(&T) -> Result<Member<R>, Error>,
) -> Result<Vec<ArchivedTile>, Error> {
    let mut output_file = OutputFile::create(output, mode).map_err(Error::io(output))?;
    let file = output_file.file_mut();
    let mtime = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since_epoch| since_epoch.as_secs());

    // Reserve index.bin as the first member; it is filled in once the offsets are known.
    let index_size = INDEX_ENTRY_SIZE * tiles.len() as u64;
    append(file, INDEX_NAME, index_size, io::repeat(0), mtime).map_err(Error::io(output))?;

    let mut archived = Vec::with_capacity(tiles.len());
    for tile in tiles {
        let Member {
            name,
            tile_id,
            size,
            content,
        } = member(tile)?;
        let Ok(indexed_size) = u32::try_from(size) else {
            return Err(Error::TileTooLarge { name, size });
        };
        let offset = match append(file, &name, size, content, mtime) {
            Ok(offset) => offset,
            Err(source) => {
                return Err(Error::CopyTile {
                    name,
                    path: output.to_path_buf(),
                    source,
                });
            }
        };
        debug!(tile = name, offset, size, "added tile");
        archived.push(ArchivedTile {
            name,
            entry: IndexEntry {
                offset,
                tile_id,
                size: indexed_size,
            },
        });
    }

    let index: Vec<u8> = archived
        .iter()
        .flat_map(|tile| tile.entry.to_bytes())
        .collect();
    let finish = |file: &mut File| -> io::Result<()> {
        file.write_all(&END_OF_ARCHIVE)?;
        file.seek(SeekFrom::Start(BLOCK_SIZE))?;
        file.write_all(&index)?;
        file.sync_all()
    };
    finish(file).map_err(Error::io(output))?;
    output_file.finish(output).map_err(Error::io(output))?;
    Ok(archived)
}

/// Appends a plain ustar member and returns the offset of its data.
fn append(
    file: &mut File,
    name: &str,
    size: u64,
    content: impl Read,
    mtime: u64,
) -> io::Result<u64> {
    let mut header = tar::Header::new_ustar();
    header.set_path(name)?;
    header.set_size(size);
    header.set_mode(0o644);
    header.set_uid(0);
    header.set_gid(0);
    header.set_mtime(mtime);
    header.set_entry_type(tar::EntryType::Regular);
    header.set_cksum();
    file.write_all(header.as_bytes())?;

    let offset = file.stream_position()?;
    let copied = io::copy(&mut content.take(size), file)?;
    if copied != size {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            format!("expected {size} bytes, got {copied}"),
        ));
    }
    let padding = size.next_multiple_of(BLOCK_SIZE) - size;
    io::copy(&mut io::repeat(0).take(padding), file)?;
    Ok(offset)
}

/// Whether both paths name the same existing file, through symlinks and hard links.
pub(crate) fn is_same_file(a: &Path, b: &Path) -> bool {
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        match (fs::metadata(a), fs::metadata(b)) {
            (Ok(a), Ok(b)) => a.dev() == b.dev() && a.ino() == b.ino(),
            _ => false,
        }
    }
    #[cfg(not(unix))]
    {
        match (fs::canonicalize(a), fs::canonicalize(b)) {
            (Ok(a), Ok(b)) => a == b,
            _ => false,
        }
    }
}

fn parent_dir(output: &Path) -> &Path {
    match output.parent() {
        Some(parent) if !parent.as_os_str().is_empty() => parent,
        _ => Path::new("."),
    }
}

fn create_temp_file(output: &Path) -> io::Result<NamedTempFile> {
    let file_name = output
        .file_name()
        .map(|name| name.to_string_lossy())
        .unwrap_or_default();
    let prefix = format!(".{file_name}.");
    let mut builder = tempfile::Builder::new();
    builder.prefix(&prefix).suffix(".tmp");
    // Same mode as `File::create`, so the umask decides the permissions as usual.
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        builder.permissions(fs::Permissions::from_mode(0o666));
    }
    builder.tempfile_in(parent_dir(output))
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    #[test]
    fn an_index_entry_is_offset_then_tile_id_then_size_in_little_endian() {
        let entry = IndexEntry {
            offset: 0x0102_0304_0506_0708,
            tile_id: 6_549_282,
            size: 0x0a0b_0c0d,
        };
        let mut expected = Vec::new();
        expected.extend_from_slice(&[0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01]);
        expected.extend_from_slice(&6_549_282_u32.to_le_bytes());
        expected.extend_from_slice(&[0x0d, 0x0c, 0x0b, 0x0a]);
        assert_eq!(entry.to_bytes().as_slice(), expected.as_slice());
    }

    #[test]
    fn a_member_shorter_than_announced_is_an_error() {
        let mut file = tempfile::tempfile().unwrap();
        let result = append(&mut file, "0/003/196.gph", 100, &[1_u8; 10][..], 0);
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::UnexpectedEof);
    }

    #[test]
    fn member_data_is_padded_to_a_whole_block() {
        let mut file = tempfile::tempfile().unwrap();
        append(&mut file, "0/003/196.gph", 700, io::repeat(1), 0).unwrap();
        assert_eq!(file.stream_position().unwrap(), 512 + 1024);
    }

    #[test]
    fn a_tile_too_large_for_the_index_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let output = dir.path().join("tiles.tar");
        let result = write_indexed_tar(&output, WriteMode::Replace, &[()], |_| {
            Ok(Member {
                name: "0/003/196.gph".to_owned(),
                tile_id: 25568,
                size: u64::from(u32::MAX) + 1,
                content: io::empty(),
            })
        });
        assert!(matches!(result, Err(Error::TileTooLarge { .. })));
    }

    #[test]
    fn a_failed_write_leaves_neither_output_nor_temporary_file() {
        let dir = tempfile::tempdir().unwrap();
        let output = dir.path().join("tiles.tar");
        let result = write_indexed_tar(&output, WriteMode::Replace, &[()], |_| {
            Ok(Member {
                name: "0/003/196.gph".to_owned(),
                tile_id: 25568,
                size: 100,
                content: io::empty(),
            })
        });
        assert!(result.is_err());
        assert_eq!(fs::read_dir(dir.path()).unwrap().count(), 0);
    }
}
