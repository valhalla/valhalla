use std::fs;
use std::io::{self, BufWriter, Read, Write};
use std::num::NonZeroUsize;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use flate2::read::MultiGzDecoder;
use tracing::{debug, error, info, info_span};

use super::Tile;

/// The skadi tiles of the AWS Terrain Tiles bucket, each at `{dir}/{name}.gz`.
pub const ELEVATION_TILES_URL: &str =
    "https://elevation-tiles-prod.s3.us-east-1.amazonaws.com/skadi";

/// One attempt per delay, each waited before its attempt: `(i**2) / 2 * 30` seconds in the script.
pub const RETRY_DELAYS: [Duration; 5] = [
    Duration::ZERO,
    Duration::from_secs(15),
    Duration::from_secs(60),
    Duration::from_secs(135),
    Duration::from_secs(240),
];

/// Errors that leave a tile undownloadable whatever the retries, such as a full disk; logged
/// like the binary's fatal errors, so that they show without `-v`.
pub const CRITICAL_TARGET: &str = "valhalla_tools::elevation::critical";

/// LZ4 frame settings of the script, which skadi's `LZ4F_decompress` reads like any frame.
const LZ4_BLOCK_SIZE: lz4::BlockSize = lz4::BlockSize::Max4MB;
const LZ4_LEVEL: u32 = 6;

/// How downloaded tiles are stored; skadi reads all three, picking the format by extension.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compression {
    /// `.hgt.gz`, the bytes as served.
    Gzip,
    /// `.hgt`, which skadi maps without decompressing.
    Uncompressed,
    /// `.hgt.lz4`, larger than gzip but several times faster to decompress.
    Lz4,
}

impl Compression {
    fn extension(self) -> &'static str {
        match self {
            Self::Gzip => ".gz",
            Self::Uncompressed => "",
            Self::Lz4 => ".lz4",
        }
    }
}

#[derive(Debug, Clone)]
pub struct DownloadOptions<'a> {
    pub outdir: &'a Path,
    pub compression: Compression,
    pub parallelism: NonZeroUsize,
    /// Replaces [`ELEVATION_TILES_URL`], for tests and mirrors.
    pub base_url: &'a str,
    /// Waited before each attempt; their number is the number of attempts.
    pub retry_delays: &'a [Duration],
}

/// Tiles already on disk are neither downloaded nor failed.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Summary {
    pub downloaded: usize,
    pub failed: usize,
}

/// Downloads the tiles missing from the output directory with `parallelism` threads. Per-tile
/// failures are logged and counted, never fatal to the other tiles.
pub fn download(tiles: &[Tile], options: &DownloadOptions<'_>) -> Summary {
    let agent = crate::http::agent();
    let next = AtomicUsize::new(0);
    let downloaded = AtomicUsize::new(0);
    let failed = AtomicUsize::new(0);
    let workers = options.parallelism.get().min(tiles.len());
    thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                while let Some(tile) = tiles.get(next.fetch_add(1, Ordering::Relaxed)) {
                    let counter = match download_tile(&agent, tile, options) {
                        Outcome::Downloaded => &downloaded,
                        Outcome::Skipped => continue,
                        Outcome::Failed => &failed,
                    };
                    counter.fetch_add(1, Ordering::Relaxed);
                }
            });
        }
    });
    Summary {
        downloaded: downloaded.into_inner(),
        failed: failed.into_inner(),
    }
}

enum Outcome {
    Downloaded,
    Skipped,
    Failed,
}

/// Why an attempt failed; only a local error is not worth retrying.
enum AttemptError {
    HttpStatus(u16, &'static str),
    Transport(String),
    Corrupted(io::Error),
    Local(io::Error),
}

fn download_tile(agent: &ureq::Agent, tile: &Tile, options: &DownloadOptions<'_>) -> Outcome {
    let _span = info_span!("elevation_tile", tile = %tile.name).entered();
    let dir = options.outdir.join(&tile.dir);
    if let Err(source) = fs::create_dir_all(&dir) {
        error!(target: CRITICAL_TARGET, "Could not create directory {}: {source}", dir.display());
        return Outcome::Failed;
    }
    let file_name = format!("{}{}", tile.name, options.compression.extension());
    remove_stale_temporary_files(&dir, &file_name);
    if dir.join(&file_name).is_file() {
        return Outcome::Skipped;
    }
    let base_url = options.base_url.trim_end_matches('/');
    let url = format!("{base_url}/{}/{}.gz", tile.dir, tile.name);
    let tile_path = format!("{}/{}", tile.dir, tile.name);

    info!("Downloading tile {}", tile.name);
    // Never reset: the script reports a tile corrupted once as corrupted, whatever follows.
    let mut corrupted = false;
    for (attempt, delay) in options.retry_delays.iter().enumerate() {
        debug!("Downloading tile {tile_path} for the {attempt}th time.");
        thread::sleep(*delay);
        match fetch(agent, &url, &dir, &file_name, options.compression) {
            Ok(()) => {
                debug!("Successfully downloaded tile {tile_path}");
                return Outcome::Downloaded;
            }
            Err(AttemptError::HttpStatus(code, reason)) => {
                error!("Download failed with HTTP error {code}: {reason}.\nTrying again...");
            }
            Err(AttemptError::Transport(reason)) => {
                error!("Download failed of elevation tile {tile_path}: {reason}.\nTrying again..");
            }
            Err(AttemptError::Corrupted(reason)) => {
                corrupted = true;
                error!(
                    "Decompression error on tile {tile_path}: {reason}. Likely a corrupted download."
                );
            }
            Err(AttemptError::Local(reason)) => {
                error!(target: CRITICAL_TARGET, "Tile {tile_path} couldn't be written: {reason}");
                return Outcome::Failed;
            }
        }
    }
    if corrupted {
        // Nothing to remove: a corrupted download never left its temporary file.
        error!("Tile {tile_path} was corrupted, removing it...");
    } else {
        error!("Tile {tile_path} couldn't be downloaded...");
    }
    Outcome::Failed
}

/// The temporary files of `file_name`, named by `fetch`, that an interrupted run left behind.
fn remove_stale_temporary_files(dir: &Path, file_name: &str) {
    let prefix = format!(".{file_name}.");
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let name = entry.file_name();
        let is_stale = name
            .to_str()
            .and_then(|name| name.strip_prefix(&prefix))
            .and_then(|rest| rest.strip_suffix(TEMPORARY_SUFFIX))
            .is_some_and(|random| {
                random.len() == TEMPORARY_RANDOM_LENGTH
                    && random.bytes().all(|byte| byte.is_ascii_alphanumeric())
            });
        if is_stale && let Err(source) = fs::remove_file(entry.path()) {
            debug!("Could not remove {}: {source}", entry.path().display());
        }
    }
}

const TEMPORARY_SUFFIX: &str = ".part";
const TEMPORARY_RANDOM_LENGTH: usize = 6;

/// One attempt: writes the tile to a temporary file next to it, renamed over the tile once
/// complete, so that no partial tile is ever skipped as already downloaded.
fn fetch(
    agent: &ureq::Agent,
    url: &str,
    dir: &Path,
    file_name: &str,
    compression: Compression,
) -> Result<(), AttemptError> {
    let response = agent
        .get(url)
        .call()
        .map_err(|error| AttemptError::Transport(error.to_string()))?;
    let status = response.status();
    if !status.is_success() {
        return Err(AttemptError::HttpStatus(
            status.as_u16(),
            status.canonical_reason().unwrap_or_default(),
        ));
    }
    let mut body = TrackedReader {
        inner: response.into_body().into_reader(),
        failed: false,
    };

    // The leading dot and the suffix keep skadi, which scans the directory, off this file.
    let prefix = format!(".{file_name}.");
    let mut builder = tempfile::Builder::new();
    builder
        .prefix(&prefix)
        .suffix(TEMPORARY_SUFFIX)
        .rand_bytes(TEMPORARY_RANDOM_LENGTH);
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        builder.permissions(fs::Permissions::from_mode(0o666));
    }
    let temporary = builder.tempfile_in(dir).map_err(AttemptError::Local)?;
    let mut writer = BufWriter::new(temporary.as_file());

    match compression {
        // Stored as served, but decoded on the way, so that a body cut short without a
        // Content-Length or an error page is retried instead of skipped by later runs.
        Compression::Gzip => {
            let mut tee = Tee {
                reader: &mut body,
                writer: &mut writer,
                write_error: None,
            };
            let decoded = io::copy(&mut MultiGzDecoder::new(&mut tee), &mut io::sink())
                .and_then(|_| io::copy(&mut tee, &mut io::sink()));
            if let Some(error) = tee.write_error.take() {
                return Err(AttemptError::Local(error));
            }
            decoded.map_err(|error| body.decode_error(error))?;
        }
        Compression::Uncompressed => {
            let hgt = gunzip(&mut body)?;
            writer.write_all(&hgt).map_err(AttemptError::Local)?;
        }
        Compression::Lz4 => {
            let hgt = gunzip(&mut body)?;
            write_lz4(&mut writer, &hgt).map_err(AttemptError::Local)?;
        }
    }
    writer.flush().map_err(AttemptError::Local)?;
    drop(writer);
    temporary
        .as_file()
        .sync_all()
        .map_err(AttemptError::Local)?;
    temporary
        .persist(dir.join(file_name))
        .map_err(|error| AttemptError::Local(error.error))?;
    Ok(())
}

fn gunzip<R: Read>(body: &mut TrackedReader<R>) -> Result<Vec<u8>, AttemptError> {
    let mut hgt = Vec::new();
    MultiGzDecoder::new(&mut *body)
        .read_to_end(&mut hgt)
        .map_err(|error| body.decode_error(error))?;
    Ok(hgt)
}

/// The content size the script writes is that of every hgt tile; the actual size keeps the
/// frame valid for any other.
fn write_lz4<W: Write>(writer: W, hgt: &[u8]) -> io::Result<()> {
    let mut encoder = lz4::EncoderBuilder::new()
        .block_size(LZ4_BLOCK_SIZE)
        .block_checksum(lz4::liblz4::BlockChecksum::NoBlockChecksum)
        .checksum(lz4::ContentChecksum::NoChecksum)
        .level(LZ4_LEVEL)
        .content_size(hgt.len() as u64)
        .build(writer)?;
    encoder.write_all(hgt)?;
    let (_, result) = encoder.finish();
    result
}

/// Remembers whether the response body failed, to tell a broken connection from bad data.
struct TrackedReader<R> {
    inner: R,
    failed: bool,
}

impl<R> TrackedReader<R> {
    /// A decoding error caused by reading the body is a transport error, otherwise bad data.
    fn decode_error(&self, error: io::Error) -> AttemptError {
        if self.failed {
            AttemptError::Transport(error.to_string())
        } else {
            AttemptError::Corrupted(error)
        }
    }
}

impl<R: Read> Read for TrackedReader<R> {
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        let result = self.inner.read(buffer);
        self.failed |= result.is_err();
        result
    }
}

/// Writes what it reads, keeping the write error apart from the read errors.
struct Tee<R, W> {
    reader: R,
    writer: W,
    write_error: Option<io::Error>,
}

impl<R: Read, W: Write> Read for Tee<R, W> {
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        let read = self.reader.read(buffer)?;
        if let Err(error) = self.writer.write_all(&buffer[..read]) {
            self.write_error = Some(error);
            return Err(io::Error::other("the tile could not be written"));
        }
        Ok(read)
    }
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    #[test]
    fn retry_delays_follow_the_scripts_formula() {
        let script: Vec<_> = (0..5_u64)
            .map(|i| Duration::from_secs_f64((i * i) as f64 / 2.0 * 30.0))
            .collect();
        assert_eq!(RETRY_DELAYS.to_vec(), script);
    }

    #[test]
    fn only_this_tiles_temporary_files_are_removed() {
        let dir = tempfile::tempdir().unwrap();
        let names = [
            ".N52E005.hgt.gz.a1B2c3.part",
            ".N52E005.hgt.gz.toolong.part",
            ".N52E005.hgt.a1B2c3.part",
            ".N52E006.hgt.gz.a1B2c3.part",
            "N52E005.hgt.gz",
        ];
        for name in names {
            fs::write(dir.path().join(name), b"").unwrap();
        }
        remove_stale_temporary_files(dir.path(), "N52E005.hgt.gz");
        let kept: Vec<_> = names
            .iter()
            .map(|name| dir.path().join(name).exists())
            .collect();
        assert_eq!(kept, [false, true, true, true, true]);
    }

    #[test]
    fn the_lz4_frame_has_the_scripts_header() {
        let mut frame = Vec::new();
        write_lz4(&mut frame, &[7; 1000]).unwrap();
        // Magic; version 1, linked blocks, content size, no checksums; 4 MB blocks; size.
        let mut expected = vec![0x04, 0x22, 0x4d, 0x18, 0x48, 0x70];
        expected.extend_from_slice(&1000_u64.to_le_bytes());
        assert_eq!(frame[..14], expected);
    }

    #[test]
    fn the_lz4_frame_decodes_to_the_input() {
        let hgt: Vec<u8> = (0..100_000_u32).map(|i| (i / 7 % 256) as u8).collect();
        let mut frame = Vec::new();
        write_lz4(&mut frame, &hgt).unwrap();
        let mut decoded = Vec::new();
        lz4::Decoder::new(&frame[..])
            .unwrap()
            .read_to_end(&mut decoded)
            .unwrap();
        assert_eq!(decoded, hgt);
    }

    struct Broken;

    impl Read for Broken {
        fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
            Err(io::Error::from(io::ErrorKind::ConnectionReset))
        }
    }

    #[test]
    fn a_body_read_error_while_decoding_is_a_transport_error() {
        let mut body = TrackedReader {
            inner: Broken,
            failed: false,
        };
        assert!(matches!(gunzip(&mut body), Err(AttemptError::Transport(_))));
    }

    #[test]
    fn bad_gzip_data_is_corrupted() {
        let mut body = TrackedReader {
            inner: &b"not gzip"[..],
            failed: false,
        };
        assert!(matches!(gunzip(&mut body), Err(AttemptError::Corrupted(_))));
    }

    #[test]
    fn a_truncated_gzip_stream_is_corrupted() {
        let mut gz = Vec::new();
        let mut encoder = flate2::write::GzEncoder::new(&mut gz, flate2::Compression::default());
        encoder.write_all(&[1; 5000]).unwrap();
        encoder.finish().unwrap();
        let mut body = TrackedReader {
            inner: &gz[..gz.len() / 2],
            failed: false,
        };
        assert!(matches!(gunzip(&mut body), Err(AttemptError::Corrupted(_))));
    }
}
