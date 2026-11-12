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
}

/// Builds the logger of the `logging` section; `None` gives the C++ default, coloured stdout.
pub fn from_section(section: Option<&Map<String, Value>>) -> Result<Logger, String> {
    let Some(section) = section else {
        return Ok(Logger {
            target: LogTarget::Stdout,
            color: true,
        });
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
                Some(seconds) => seconds
                    .parse()
                    .map_err(|_| format!("{seconds} is not a valid reopen interval"))?,
                None => 300,
            };
            let max_file_size = count(text("max_file_size"), "max_file_size")?;
            let max_archived_files = count(text("max_archived_files"), "max_archived_files")?;
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
    Ok(Logger { target, color })
}

/// A non-negative limit; absent or empty means 0, that is disabled.
fn count(value: Option<String>, key: &str) -> Result<u64, String> {
    match value.as_deref() {
        None | Some("") => Ok(0),
        Some(text) => text
            .parse()
            .map_err(|_| format!("{text} is not a valid {key}")),
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
        let file = open_append(&path)
            .map_err(|_| format!("Cannot create log file: {}", path.display()))?;
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
        self.file = open_append(&self.path)?;
        self.last_reopen = Instant::now();
        Ok(())
    }
}

fn open_append(path: &Path) -> io::Result<File> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }
    OpenOptions::new().create(true).append(true).open(path)
}
