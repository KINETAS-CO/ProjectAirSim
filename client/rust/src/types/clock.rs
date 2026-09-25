use serde::{Deserialize, Serialize};

/// Simulation clock execution mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClockType {
    Steppable,
    Wallclock,
    Realtime,
}
