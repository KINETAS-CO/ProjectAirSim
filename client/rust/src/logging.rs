//! Logging subsystem with `tracing` integration and C++ parity log sinks.
//!
//! Provides structured logging via the [`tracing`] crate, convenient initialization
//! via [`init_logging`], and custom log sink callbacks via [`set_log_sink`].

use std::fmt;
use std::io::Write;
use std::sync::{Arc, Once, RwLock};
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::Context;
use tracing_subscriber::prelude::*;
use tracing_subscriber::{EnvFilter, Layer};

/// Log message severity levels, maintaining semantic parity with the C++ client's
/// `microsoft::projectairsim::client::Log::Severity`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Severity {
    /// Detailed diagnostic information.
    Trace,
    /// Diagnostic information useful during development.
    Debug,
    /// Normal operational messages.
    Info,
    /// Warnings for unexpected situations that do not prevent execution.
    Warning,
    /// Error conditions where an operation failed but execution can continue.
    Error,
    /// Critical failure conditions where execution cannot safely proceed.
    Critical,
}

impl Severity {
    /// Returns the string representation of the severity.
    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::Trace => "TRACE",
            Self::Debug => "DEBUG",
            Self::Info => "INFO",
            Self::Warning => "WARN",
            Self::Error => "ERROR",
            Self::Critical => "CRITICAL",
        }
    }
}

impl fmt::Display for Severity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

impl From<Level> for Severity {
    fn from(level: Level) -> Self {
        match level {
            Level::TRACE => Self::Trace,
            Level::DEBUG => Self::Debug,
            Level::INFO => Self::Info,
            Level::WARN => Self::Warning,
            Level::ERROR => Self::Error,
        }
    }
}

impl From<Severity> for Level {
    fn from(severity: Severity) -> Self {
        match severity {
            Severity::Trace => Level::TRACE,
            Severity::Debug => Level::DEBUG,
            Severity::Info => Level::INFO,
            Severity::Warning => Level::WARN,
            Severity::Error | Severity::Critical => Level::ERROR,
        }
    }
}

/// Callback type for receiving log messages.
pub type LogSink = Arc<dyn Fn(Severity, &str) + Send + Sync + 'static>;

/// Global storage for custom log sink callback.
static LOG_SINK: RwLock<Option<LogSink>> = RwLock::new(None);

/// Global initialization guard for the tracing subscriber.
static INIT: Once = Once::new();

/// Registers a custom log sink callback matching the C++ client's `pasc::log.SetLogSink(...)`.
///
/// When a log sink is registered, all log events are forwarded to the callback,
/// and default terminal output is suppressed to prevent duplicated console logs.
/// Calling this automatically initializes the logging subsystem if it has not
/// already been initialized.
pub fn set_log_sink<F>(sink: F)
where
    F: Fn(Severity, &str) + Send + Sync + 'static,
{
    init_logging();
    let mut guard = LOG_SINK.write().expect("LOG_SINK lock poisoned");
    *guard = Some(Arc::new(sink));
}

/// Clears any registered log sink callback, restoring default console output.
pub fn clear_log_sink() {
    let mut guard = LOG_SINK.write().expect("LOG_SINK lock poisoned");
    *guard = None;
}

/// Checks whether a custom log sink is currently active.
pub fn has_log_sink() -> bool {
    LOG_SINK.read().map(|g| g.is_some()).unwrap_or(false)
}

/// Dispatches an event directly to the active sink, or logs via tracing.
pub fn log(severity: Severity, message: &str) {
    match severity {
        Severity::Trace => tracing::trace!("{}", message),
        Severity::Debug => tracing::debug!("{}", message),
        Severity::Info => tracing::info!("{}", message),
        Severity::Warning => tracing::warn!("{}", message),
        Severity::Error => tracing::error!("{}", message),
        Severity::Critical => tracing::error!(critical = true, "{}", message),
    }
}

/// Logs an informational message.
pub fn info(message: &str) {
    log(Severity::Info, message);
}

/// Logs a warning message.
pub fn warn(message: &str) {
    log(Severity::Warning, message);
}

/// Logs an error message.
pub fn error(message: &str) {
    log(Severity::Error, message);
}

/// Logs a critical message.
pub fn critical(message: &str) {
    log(Severity::Critical, message);
}

/// Visitor to extract the primary message and metadata from a tracing Event.
struct EventVisitor {
    message: String,
    critical: bool,
}

impl EventVisitor {
    fn new() -> Self {
        Self {
            message: String::new(),
            critical: false,
        }
    }
}

impl Visit for EventVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn fmt::Debug) {
        if field.name() == "message" {
            use fmt::Write;
            let _ = write!(self.message, "{:?}", value);
        } else if field.name() == "critical" {
            if let Ok(b) = format!("{:?}", value).parse::<bool>() {
                self.critical = b;
            }
        } else {
            use fmt::Write;
            if !self.message.is_empty() {
                let _ = write!(self.message, " {}={:?}", field.name(), value);
            } else {
                let _ = write!(self.message, "{}={:?}", field.name(), value);
            }
        }
    }

    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.message.push_str(value);
        } else {
            use fmt::Write;
            if !self.message.is_empty() {
                let _ = write!(self.message, " {}={}", field.name(), value);
            } else {
                let _ = write!(self.message, "{}={}", field.name(), value);
            }
        }
    }

    fn record_bool(&mut self, field: &Field, value: bool) {
        if field.name() == "critical" {
            self.critical = value;
        } else {
            use fmt::Write;
            if !self.message.is_empty() {
                let _ = write!(self.message, " {}={}", field.name(), value);
            } else {
                let _ = write!(self.message, "{}={}", field.name(), value);
            }
        }
    }
}

/// A tracing layer that invokes the registered `LogSink` callback when set.
struct SinkLayer;

impl<S> Layer<S> for SinkLayer
where
    S: Subscriber + for<'a> tracing_subscriber::registry::LookupSpan<'a>,
{
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        if let Ok(guard) = LOG_SINK.read() {
            if let Some(ref sink) = *guard {
                let mut visitor = EventVisitor::new();
                event.record(&mut visitor);

                let severity = if visitor.critical {
                    Severity::Critical
                } else {
                    Severity::from(*event.metadata().level())
                };

                sink(severity, &visitor.message);
            }
        }
    }
}

/// Writer for `tracing_subscriber::fmt` that only writes to stdout if no custom sink is set.
#[derive(Clone, Copy, Debug, Default)]
struct ConditionalStdoutWriter;

impl Write for ConditionalStdoutWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        if !has_log_sink() {
            std::io::stdout().write(buf)
        } else {
            Ok(buf.len())
        }
    }

    fn flush(&mut self) -> std::io::Result<()> {
        if !has_log_sink() {
            std::io::stdout().flush()
        } else {
            Ok(())
        }
    }
}

impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for ConditionalStdoutWriter {
    type Writer = ConditionalStdoutWriter;

    fn make_writer(&'a self) -> Self::Writer {
        *self
    }
}

/// Initializes the ProjectAirSim client logging subsystem with default formatting and environment filtering.
///
/// Reads the filter level from the `RUST_LOG` environment variable if present;
/// otherwise defaults to `info`. Output is written to standard output unless a custom sink
/// is registered via [`set_log_sink`].
///
/// This function is safe to call multiple times; subsequent calls are no-ops.
pub fn init_logging() {
    INIT.call_once(|| {
        let env_filter = EnvFilter::try_from_default_env()
            .unwrap_or_else(|_| EnvFilter::new("info"));

        let fmt_layer = tracing_subscriber::fmt::layer()
            .with_writer(ConditionalStdoutWriter)
            .with_target(true);

        let subscriber = tracing_subscriber::registry()
            .with(env_filter)
            .with(SinkLayer)
            .with(fmt_layer);

        // Ignore failure if a global subscriber is already registered (e.g. in test harness)
        let _ = subscriber.try_init();
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn test_severity_display_and_conversion() {
        assert_eq!(Severity::Trace.as_str(), "TRACE");
        assert_eq!(Severity::Debug.as_str(), "DEBUG");
        assert_eq!(Severity::Info.as_str(), "INFO");
        assert_eq!(Severity::Warning.as_str(), "WARN");
        assert_eq!(Severity::Error.as_str(), "ERROR");
        assert_eq!(Severity::Critical.as_str(), "CRITICAL");

        assert_eq!(Severity::from(Level::INFO), Severity::Info);
        assert_eq!(Severity::from(Level::WARN), Severity::Warning);
        assert_eq!(Severity::from(Level::ERROR), Severity::Error);

        assert_eq!(Level::from(Severity::Info), Level::INFO);
        assert_eq!(Level::from(Severity::Critical), Level::ERROR);
    }

    #[test]
    fn test_custom_log_sink() {
        let count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&count);

        set_log_sink(move |severity, msg| {
            if severity == Severity::Info && msg.contains("test message") {
                count_clone.fetch_add(1, Ordering::SeqCst);
            }
        });

        assert!(has_log_sink());

        info("test message 1");
        info("test message 2");
        assert_eq!(count.load(Ordering::SeqCst), 2);

        clear_log_sink();
        assert!(!has_log_sink());

        info("test message 3");
        assert_eq!(count.load(Ordering::SeqCst), 2);
    }
}
