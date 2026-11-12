use std::time::Duration;

/// Applies to connecting and to each read or write, not to the whole download.
const TIMEOUT: Duration = Duration::from_secs(30);

/// An agent that returns HTTP error statuses as responses, so callers can report them, and
/// cannot hang on an unresponsive server.
pub(crate) fn agent() -> ureq::Agent {
    ureq::Agent::config_builder()
        .http_status_as_error(false)
        .timeout_connect(Some(TIMEOUT))
        .timeout_per_read(Some(TIMEOUT))
        .timeout_per_write(Some(TIMEOUT))
        .build()
        .into()
}
