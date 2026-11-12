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

/// Writes the timestamp and level of the format, then the message; events carry no other field.
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
                let timestamp = python_timestamp(&Local::now());
                write!(writer, "{timestamp} {:>5}: ", python_level_name(level))?;
            }
            LogFormat::Valhalla => {
                write!(writer, "{} ", valhalla_timestamp(&Utc::now()))?;
                match (self.color, color_code(level)) {
                    (true, code) => write!(writer, "\x1b[{code};1m[{level}]\x1b[0m ")?,
                    (false, _) => write!(writer, "[{level}] ")?,
                }
            }
        }
        let mut message = Message::default();
        event.record(&mut message);
        writeln!(writer, "{}", message.0)
    }
}

/// `%(asctime)s` of Python's `logging`: milliseconds are truncated, not rounded.
fn python_timestamp<Z: TimeZone>(time: &DateTime<Z>) -> String
where
    Z::Offset: fmt::Display,
{
    let millis = time.timestamp_subsec_millis();
    format!("{},{millis:03}", time.format("%Y-%m-%d %H:%M:%S"))
}

/// `append_timestamp` of src/midgard/logging.cc: `system_clock` printed with `%F %T`.
fn valhalla_timestamp(time: &DateTime<Utc>) -> String {
    let nanos = time.timestamp_subsec_nanos();
    format!("{}.{nanos:09}", time.format("%Y-%m-%d %H:%M:%S"))
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
struct Message(String);

impl Visit for Message {
    fn record_debug(&mut self, field: &Field, value: &dyn fmt::Debug) {
        if field.name() == "message" {
            let _ = write!(self.0, "{value:?}");
        }
    }
}

#[cfg(test)]
mod tests {
    use chrono::{FixedOffset, TimeZone};

    use super::*;

    #[test]
    fn python_milliseconds_are_truncated_not_rounded() {
        let time = Utc.timestamp_opt(1_700_000_000, 999_900_000).single();
        assert_eq!(
            time.map(|time| python_timestamp(&time)).as_deref(),
            Some("2023-11-14 22:13:20,999")
        );
    }

    #[test]
    fn python_timestamps_use_the_given_offset() {
        let tokyo = FixedOffset::east_opt(9 * 3600);
        let time = tokyo.and_then(|zone| zone.timestamp_opt(1_700_000_000, 0).single());
        assert_eq!(
            time.map(|time| python_timestamp(&time)).as_deref(),
            Some("2023-11-15 07:13:20,000")
        );
    }

    #[test]
    fn valhalla_timestamps_have_nanoseconds() {
        let time = Utc.timestamp_opt(1_700_000_000, 5).single();
        assert_eq!(
            time.map(|time| valhalla_timestamp(&time)).as_deref(),
            Some("2023-11-14 22:13:20.000000005")
        );
    }
}
