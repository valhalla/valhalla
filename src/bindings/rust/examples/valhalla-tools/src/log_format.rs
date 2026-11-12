//! Log line formats of the binary: the Python script's or Valhalla's C++ logger's.

use std::fmt::{self, Write};

use chrono::{DateTime, Local, TimeZone, Utc};
use clap::ValueEnum;
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::fmt::format::Writer;
use tracing_subscriber::fmt::{FmtContext, FormatEvent, FormatFields};
use tracing_subscriber::registry::LookupSpan;

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum LogFormat {
    /// `2026-11-12 12:59:04,737  INFO: message`, in local time.
    Python,
    /// `2026-11-12 11:59:04.669982123 [INFO] message`, in UTC.
    Valhalla,
}

/// Writes the timestamp and level of the format, then the message and its fields as `key=value`.
pub struct LineFormat {
    pub format: LogFormat,
    /// Colours the level as the C++ console loggers do; only used by the Valhalla format.
    pub color: bool,
}

impl<S, N> FormatEvent<S, N> for LineFormat
where
    S: Subscriber + for<'a> LookupSpan<'a>,
    N: for<'a> FormatFields<'a> + 'static,
{
    fn format_event(
        &self,
        _context: &FmtContext<'_, S, N>,
        mut writer: Writer<'_>,
        event: &Event<'_>,
    ) -> fmt::Result {
        let level = *event.metadata().level();
        match self.format {
            LogFormat::Python => {
                let now = Local::now();
                write_date_time(&mut writer, &now)?;
                // Truncated, not rounded, like `logging.Formatter.formatTime`.
                let millis = now.timestamp_subsec_millis();
                write!(writer, ",{millis:03} {:>5}: ", python_level_name(level))?;
            }
            LogFormat::Valhalla => {
                let now = Utc::now();
                write_date_time(&mut writer, &now)?;
                write!(writer, ".{:09} ", now.timestamp_subsec_nanos())?;
                match (self.color, color_code(level)) {
                    (true, code) => write!(writer, "\x1b[{code};1m[{level}]\x1b[0m ")?,
                    (false, _) => write!(writer, "[{level}] ")?,
                }
            }
        }
        let mut line = Line::default();
        event.record(&mut line);
        writeln!(writer, "{}{}", line.message, line.fields)
    }
}

fn write_date_time<Z: TimeZone>(writer: &mut Writer<'_>, time: &DateTime<Z>) -> fmt::Result
where
    Z::Offset: fmt::Display,
{
    write!(writer, "{}", time.format("%Y-%m-%d %H:%M:%S"))
}

/// ANSI colours of the `colored` levels in src/midgard/logging.cc.
fn color_code(level: Level) -> u8 {
    match level {
        Level::ERROR => 31,
        Level::WARN => 33,
        Level::INFO => 32,
        Level::DEBUG => 34,
        Level::TRACE => 37,
    }
}

/// Python's `logging` level names; errors are fatal here, which the script logs as critical.
fn python_level_name(level: Level) -> &'static str {
    match level {
        Level::ERROR => "CRITICAL",
        Level::WARN => "WARNING",
        Level::INFO => "INFO",
        Level::DEBUG => "DEBUG",
        Level::TRACE => "TRACE",
    }
}

#[derive(Default)]
struct Line {
    message: String,
    fields: String,
}

impl Visit for Line {
    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.message.push_str(value);
        } else {
            let _ = write!(self.fields, " {}={value}", field.name());
        }
    }

    fn record_debug(&mut self, field: &Field, value: &dyn fmt::Debug) {
        if field.name() == "message" {
            let _ = write!(self.message, "{value:?}");
        } else {
            let _ = write!(self.fields, " {}={value:?}", field.name());
        }
    }
}
