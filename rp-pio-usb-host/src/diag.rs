//! Diagnostic counters for bus-level anomalies, read (and reset) by the application.

use core::sync::atomic::{AtomicU32, Ordering};

/// SM0 did not raise its EOP IRQ within the TX wait timeout.
pub(crate) static TX_EOP_TIMEOUT: AtomicU32 = AtomicU32::new(0);
/// D+/D- were still driven shortly after a packet's EOP; the bus was released by force.
pub(crate) static TX_BUS_HELD: AtomicU32 = AtomicU32::new(0);
/// Handshake after an OUT/SETUP: no reply at all.
pub(crate) static HS_NO_REPLY: AtomicU32 = AtomicU32::new(0);
/// Handshake after an OUT/SETUP: a one-byte (truncated) reply.
pub(crate) static HS_SHORT: AtomicU32 = AtomicU32::new(0);
/// Handshake after an OUT/SETUP: STALL.
pub(crate) static HS_STALL: AtomicU32 = AtomicU32::new(0);
/// Handshake after an OUT/SETUP: a PID other than ACK, NAK or STALL.
pub(crate) static HS_OTHER: AtomicU32 = AtomicU32::new(0);
/// The last such PID byte.
pub(crate) static HS_OTHER_LAST: AtomicU32 = AtomicU32::new(0);

#[inline(always)]
pub(crate) fn count(c: &AtomicU32) {
    c.fetch_add(1, Ordering::Relaxed);
}

/// Counter values since the previous [`take`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    pub tx_eop_timeout: u32,
    pub tx_bus_held: u32,
    pub hs_no_reply: u32,
    pub hs_short: u32,
    pub hs_stall: u32,
    pub hs_other: u32,
    pub hs_other_last_pid: u8,
}

impl Counters {
    /// No anomaly counted.
    pub fn is_clean(&self) -> bool {
        self.tx_eop_timeout == 0
            && self.tx_bus_held == 0
            && self.hs_no_reply == 0
            && self.hs_short == 0
            && self.hs_stall == 0
            && self.hs_other == 0
    }
}

/// Read and reset the counters.
pub fn take() -> Counters {
    let t = |c: &AtomicU32| c.swap(0, Ordering::Relaxed);
    Counters {
        tx_eop_timeout: t(&TX_EOP_TIMEOUT),
        tx_bus_held: t(&TX_BUS_HELD),
        hs_no_reply: t(&HS_NO_REPLY),
        hs_short: t(&HS_SHORT),
        hs_stall: t(&HS_STALL),
        hs_other: t(&HS_OTHER),
        hs_other_last_pid: HS_OTHER_LAST.load(Ordering::Relaxed) as u8,
    }
}
