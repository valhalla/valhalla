//! The `logging` config section, read the way src/midgard/logging.cc reads it.

use std::ffi::OsString;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::{Mutex, PoisonError};
use std::time::{Duration, Instant};

use serde_json::{Map, Value};
use tracing_subscriber::fmt::MakeWriter;

/// Where log lines go and whether their level is coloured.
pub struct Logger {
    pub target: LogTarget,
    pub color: bool,
    /// Whether to warn, as the C++ does, that the config has no `logging` section.
    pub announce_default: bool,
}

impl Logger {
    pub fn stderr() -> Self {
        Self {
            target: LogTarget::Stderr,
            color: false,
            announce_default: false,
        }
    }

    /// The C++ default logger, `{"type": "std_out", "color": "true"}`.
    pub fn default_console() -> Self {
        Self {
            target: LogTarget::Stdout,
            color: true,
            announce_default: false,
        }
    }
}

/// The logger of a parsed config's `logging` section; without one, the announced C++ default.
pub fn from_section(section: Option<&Value>) -> Result<Logger, String> {
    let empty = Map::new();
    let section = match section {
        None => {
            return Ok(Logger {
                announce_default: true,
                ..Logger::default_console()
            });
        }
        Some(Value::Object(section)) => section,
        // A scalar section has no keys, so no logger type either.
        Some(_) => &empty,
    };
    // The C++ reads every value as the text of a property tree node.
    let text = |key: &str| {
        section.get(key).map(|value| match value {
            Value::String(text) => text.clone(),
            Value::Object(_) | Value::Array(_) => String::new(),
            other => other.to_string(),
        })
    };
    let color = text("color").as_deref() == Some("true");
    let target = match text("type").as_deref() {
        None => return Err("Logging factory configuration requires a type of logger".to_owned()),
        Some("") => LogTarget::Null,
        Some("std_out") => LogTarget::Stdout,
        Some("std_err") => LogTarget::Stderr,
        Some("file") => {
            let path = text("file_name").ok_or("No output file provided to file logger")?;
            let reopen_interval = match text("reopen_interval") {
                Some(text) => strtoul(&text)
                    .ok_or_else(|| format!("{text} is not a valid reopen interval"))?,
                None => 300,
            };
            let max_file_size = match text("max_file_size").filter(|text| !text.is_empty()) {
                Some(text) => strtol(&text)
                    .and_then(|size| u64::try_from(size).ok())
                    .ok_or_else(|| format!("{text} is not a valid max_file_size"))?,
                None => 0,
            };
            let max_archived_files =
                match text("max_archived_files").filter(|text| !text.is_empty()) {
                    // `midgard::to_int<int>`, which does not skip whitespace.
                    Some(text) => leading_integer(&text)
                        .and_then(|count| i32::try_from(count).ok())
                        .and_then(|count| u64::try_from(count).ok())
                        .ok_or_else(|| format!("{text} is not a valid max_archived_files"))?,
                    None => 0,
                };
            // Rolling is only enabled when both limits are set.
            let rolling = (max_file_size > 0 && max_archived_files > 0)
                .then_some((max_file_size, max_archived_files));
            let file = LogFile::open(
                PathBuf::from(path),
                Duration::from_secs(reopen_interval),
                rolling,
            )?;
            LogTarget::File(Mutex::new(file))
        }
        Some(other) => return Err(format!("Couldn't produce logger for type: {other}")),
    };
    // Only the console loggers colour their output.
    let color = color && matches!(target, LogTarget::Stdout | LogTarget::Stderr);
    Ok(Logger {
        target,
        color,
        announce_default: false,
    })
}

/// The integer at the start of `text`, after an optional sign, ignoring what follows the
/// digits, as `strtol` and `std::from_chars` read it. `None` without digits.
fn leading_integer(text: &str) -> Option<i128> {
    let (negative, digits) = match text.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, text.strip_prefix('+').unwrap_or(text)),
    };
    let end = digits
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(digits.len());
    let magnitude: i128 = digits[..end].parse().ok()?;
    Some(if negative { -magnitude } else { magnitude })
}

/// `std::stol`: leading whitespace skipped, `None` out of the `long` range.
fn strtol(text: &str) -> Option<i64> {
    leading_integer(text.trim_start_matches(|c: char| c.is_ascii_whitespace()))
        .and_then(|value| i64::try_from(value).ok())
}

/// `std::stoul`, which also accepts a negative number and wraps it around.
fn strtoul(text: &str) -> Option<u64> {
    let value = leading_integer(text.trim_start_matches(|c: char| c.is_ascii_whitespace()))?;
    if value >= 0 {
        u64::try_from(value).ok()
    } else {
        u64::try_from(-value).ok().map(u64::wrapping_neg)
    }
}

pub enum LogTarget {
    Null,
    Stdout,
    Stderr,
    File(Mutex<LogFile>),
}

impl<'a> MakeWriter<'a> for LogTarget {
    type Writer = LineWriter<'a>;

    fn make_writer(&'a self) -> Self::Writer {
        LineWriter(self)
    }
}

/// Receives each formatted line in a single `write` call.
pub struct LineWriter<'a>(&'a LogTarget);

impl Write for LineWriter<'_> {
    fn write(&mut self, line: &[u8]) -> io::Result<usize> {
        match self.0 {
            LogTarget::Null => {}
            LogTarget::Stdout => io::stdout().lock().write_all(line)?,
            LogTarget::Stderr => io::stderr().lock().write_all(line)?,
            LogTarget::File(file) => file
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .write_line(line)?,
        }
        Ok(line.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// The C++ file logger: appends, reopens the file every interval and optionally rolls it.
pub struct LogFile {
    path: PathBuf,
    file: File,
    reopen_interval: Duration,
    last_reopen: Instant,
    /// Maximum size in bytes and number of archives `path.1` .. `path.N`.
    rolling: Option<(u64, u64)>,
}

impl LogFile {
    fn open(
        path: PathBuf,
        reopen_interval: Duration,
        rolling: Option<(u64, u64)>,
    ) -> Result<Self, String> {
        let file = open_append(&path)?;
        Ok(Self {
            path,
            file,
            reopen_interval,
            last_reopen: Instant::now(),
            rolling,
        })
    }

    fn write_line(&mut self, line: &[u8]) -> io::Result<()> {
        if let Some((max_size, archives)) = self.rolling
            && self.file.metadata()?.len() > max_size
        {
            self.roll(archives)?;
        }
        self.file.write_all(line)?;
        if self.last_reopen.elapsed() > self.reopen_interval {
            self.reopen()?;
        }
        Ok(())
    }

    /// Shifts `path.1` .. up to `path.N`, dropping the oldest, then moves `path` to `path.1`.
    fn roll(&mut self, archives: u64) -> io::Result<()> {
        let archive = |number: u64| {
            let mut name = OsString::from(self.path.as_os_str());
            name.push(format!(".{number}"));
            PathBuf::from(name)
        };
        let highest = (1..archives)
            .take_while(|&number| archive(number).exists())
            .last()
            .unwrap_or(0);
        for number in (1..=highest).rev() {
            let newer = archive(number + 1);
            if newer.exists() {
                fs::remove_file(&newer)?;
            }
            fs::rename(archive(number), newer)?;
        }
        if self.path.exists() {
            fs::rename(&self.path, archive(1))?;
        }
        self.reopen()
    }

    fn reopen(&mut self) -> io::Result<()> {
        self.file = open_append(&self.path).map_err(io::Error::other)?;
        self.last_reopen = Instant::now();
        Ok(())
    }
}

fn open_append(path: &Path) -> Result<File, String> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)
            .map_err(|_| format!("Cannot create directory for log file: {}", parent.display()))?;
    }
    OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .map_err(|_| format!("Cannot create log file: {}", path.display()))
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    fn write_lines(file: &mut LogFile, lines: &[&str]) {
        for line in lines {
            file.write_line(format!("{line}\n").as_bytes()).unwrap();
        }
    }

    /// Writes "first", moves the log away as logrotate would, then writes "second" and "third".
    fn rotate_between_lines(reopen_interval: Duration) -> (String, Option<String>) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("build.log");
        let mut file = LogFile::open(path.clone(), reopen_interval, None).unwrap();
        write_lines(&mut file, &["first"]);
        fs::rename(&path, dir.path().join("build.log.old")).unwrap();
        write_lines(&mut file, &["second", "third"]);
        let rotated = fs::read_to_string(dir.path().join("build.log.old")).unwrap();
        (rotated, fs::read_to_string(&path).ok())
    }

    #[test]
    fn a_zero_reopen_interval_follows_a_rotated_log_after_one_line() {
        let expected = ("first\nsecond\n".to_owned(), Some("third\n".to_owned()));
        assert_eq!(rotate_between_lines(Duration::ZERO), expected);
    }

    #[test]
    fn the_default_reopen_interval_keeps_writing_to_the_rotated_log() {
        let expected = ("first\nsecond\nthird\n".to_owned(), None);
        assert_eq!(rotate_between_lines(Duration::from_secs(300)), expected);
    }

    #[test]
    fn rolling_at_the_archive_limit_drops_the_oldest_archive() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("build.log");
        let archive = |number: u32| dir.path().join(format!("build.log.{number}"));
        fs::write(archive(1), "old 1\n").unwrap();
        fs::write(archive(2), "old 2\n").unwrap();
        fs::write(&path, "current\n").unwrap();
        let mut file = LogFile::open(path.clone(), Duration::from_secs(300), Some((1, 2))).unwrap();

        write_lines(&mut file, &["new"]);

        let contents = [path, archive(1), archive(2)].map(|path| fs::read_to_string(path).unwrap());
        assert_eq!(contents, ["new\n", "current\n", "old 1\n"]);
    }

    #[test]
    fn an_uncreatable_log_directory_is_reported_like_the_cpp() {
        let dir = tempfile::tempdir().unwrap();
        let blocker = dir.path().join("file");
        fs::write(&blocker, "").unwrap();
        let path = blocker.join("logs/build.log");
        let message = LogFile::open(path, Duration::ZERO, None).err();
        let expected = format!(
            "Cannot create directory for log file: {}",
            blocker.join("logs").display()
        );
        assert_eq!(message, Some(expected));
    }

    #[test]
    fn stoul_skips_whitespace_and_stops_at_the_first_non_digit() {
        assert_eq!(
            [" 30", "300.0", "10MB"].map(strtoul),
            [Some(30), Some(300), Some(10)]
        );
    }

    #[test]
    fn stoul_wraps_negative_values_around() {
        assert_eq!(strtoul("-1"), Some(u64::MAX));
    }

    #[test]
    fn numbers_without_digits_are_invalid() {
        assert_eq!(["", "MB", "+-1", " "].map(leading_integer), [None; 4]);
    }

    #[test]
    fn from_chars_does_not_skip_whitespace() {
        assert_eq!(leading_integer(" 3"), None);
    }
}
