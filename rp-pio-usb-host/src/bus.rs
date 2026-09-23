//! Direct root-port USB bus built from one RP PIO block and two adjacent GPIOs.
//!
//! This layer owns packet transmission, packet reception, speed detection, debounce,
//! reset, and low-/full-speed keep-alives. Higher-level adapters can build control,
//! bulk, and interrupt transfers on top of these primitives.

use crate::pio_instance::UsbPioInstance;
use crate::ram::now_us;
use crate::rx_driver::{RxDriver, RxPacketStatus};
use crate::tx_driver::TxDriver;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::{Common, Instance, InterruptHandler, Pin, Pio, PioPin};
use embassy_time::{Duration, Timer};
use embassy_usb_driver::Speed;
use embassy_usb_driver::host::{DeviceEvent, PipeError};

/// Root-port reset SE0 duration.
///
/// USB 2.0 §7.1.7.5 "Reset Signaling": a reset from a *root port* lasts at least
/// 50 ms (T_DRSTR, table 7-14); the 10–20 ms T_DRST applies to downstream hub ports.
/// The longer reset gives a suspended device time to wake and finish the high-speed
/// detection handshake before falling back to full speed.
const RESET_SE0_US: u64 = 50_000;

/// Reset-recovery hold: after releasing reset, emit SOF-only frames (no transactions)
/// for this many frames before talking to the device. USB 2.0 §7.1.7.5 reset-recovery
/// interval T_RSTRCY (table 7-14) is ≥10 ms; at one frame/ms (below) 15 frames ≈ 15 ms.
const RESET_RECOVERY_FRAMES: u32 = 15;

/// Debounce counter
///
/// A device has to be plugged in for at least this many frames (1 ms each) before we
/// consider it "attached".
const DEBOUNCE_FRAMES: u32 = 15;

/// Debounce cap
///
/// A device has to be unplugged for at least this many frames (1 ms each), but longer
/// than the time spent plugged in, before we consider it "detached".
const DEBOUNCE_CAP: u32 = 60;

/// Full-speed frame interval in microseconds.
///
/// USB 2.0 §7.1.12 and §8.4.3 define one full-speed frame every 1.000 ms
/// ±0.0005 ms. This is used as the SOF/low-speed keep-alive period, presence-poll
/// period, and retry cadence. The bus-idle test uses the RAM-safe [`now_us`]
/// helper because it runs in the synchronous transaction path.
const FRAME_INTERVAL_US: u32 = 1000;

/// Minimum spacing between two SOFs.
///
/// SOFs are aligned to 1 ms timer slots; if one goes out late in its slot, the next
/// slot's SOF is held back until at least this long after it.
const MIN_SOF_SPACING_US: u32 = FRAME_INTERVAL_US / 2;

/// End-of-frame guard: a transaction is not started within this many microseconds
/// of the next frame boundary.
///
/// Covers the longest full-speed transaction this host issues: token, turnaround,
/// a 64-byte DATA packet with worst-case bit stuffing (~53 µs), and the handshake.
const FRAME_GUARD_US: u32 = 80;

/// Continuous idle (J) that marks the end of a device packet. Inside a packet the line
/// holds a state for at most 7 bit times (0.6 µs at full speed).
const BUS_IDLE_US: u32 = 2;

/// Upper bound for [`Bus::settle_after_bad_reply`]: longer than a maximum-size
/// full-speed packet with worst-case bit stuffing (~56 µs).
const BUS_IDLE_TIMEOUT_US: u32 = 100;

/// Timer slot of the most recent SOF, readable without the bus lock.
///
/// Lets the frame-timer interrupt tell, when a transfer holds the bus at a frame
/// boundary, whether that transfer already sent the frame's SOF. One global: there is
/// one frame alarm (`embassy::FRAME_ALARM`), so one bus uses the frame timer.
pub(crate) static LAST_SOF_SLOT: AtomicU32 = AtomicU32::new(u32::MAX);

/// Set while the root port is held in reset (SE0): no SOF can be sent, so the
/// frame-timer interrupt does not retry.
pub(crate) static SOF_PAUSED: AtomicBool = AtomicBool::new(false);

/// Pull-down configuration for the USB bus.
///
/// The USB spec mandates 15k pull-downs on the D+/D- lines to detect device presence.
/// The PIO-USB host transport can either drive internal pull-downs
/// on the D+/D- lines or leave it to external pull-down resistors. External pull-downs
/// are preferable, as the internal ones do not meet the USB spec.
///
/// Using the internal pull-downs may let you get away with wiring the D+/D- lines
/// directly to a socket, but it is not recommended for production use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pulldown {
    /// Enable the RP's internal D+/D- pull-downs.
    Internal,
    /// Leave D+/D- pull-downs to external resistors.
    External,
}

/// Classified response to an IN token.
enum InReply {
    /// No response was captured after the IN token.
    NoReply,
    /// Device returned a NAK handshake.
    Nak,
    /// Device returned DATA0/DATA1.
    Data {
        /// Raw PID byte (`DATA0` or `DATA1`).
        pid: u8,
        /// Whether CRC16 and DATA PID checks passed.
        valid_crc: bool,
        /// Payload bytes, excluding SYNC, PID, and CRC16.
        payload_len: usize,
    },
    /// Any other non-terminal response.
    Other,
}

/// Physical USB bus, implemented with a PIO block and two GPIO pins.
pub struct Bus<'a, PIO: UsbPioInstance> {
    /// Shared PIO ownership state kept alive for the loaded programs and claimed pins.
    _common: Common<'a, PIO>,
    /// D+ PIO pin
    dp: Pin<'a, PIO>,
    /// D- PIO pin
    dm: Pin<'a, PIO>,

    /// The speed the transport is currently configured for.
    speed: Speed,

    /// Whether a device is currently attached to the bus.
    attached: bool,

    /// Saturating attach/detach debounce accumulator.
    debounce: u32,

    /// Timestamp of the last packet put on the bus, in microseconds from [`now_us`].
    ///
    /// For LS devices, keepalives are sent 1 ms after the last bus activity
    /// (the deadline is 3 ms so we try to keep comfortable headroom).
    /// This is updated by [`Self::mark_activity`] from every TX path, including
    /// keep-alive itself.
    last_activity: u32,

    /// Timestamp of the last SOF packet sent, in microseconds from [`now_us`].
    ///
    /// For FS devices, we need to send one SOF frame every millisecond,
    /// including while a transfer is in progress (see [`Self::sof_if_due`]).
    last_sof: u32,

    /// 1 ms timer slot (`now_us / 1000`) of the last SOF sent; its low 11 bits are the
    /// frame number.
    sof_slot: u32,

    /// PIO state machine 3, until handed to [`Self::enable_hw_sof`].
    #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
    sof_sm: Option<embassy_rp::pio::StateMachine<'a, PIO, 3>>,

    /// Hardware-timed SOF, once enabled.
    #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
    hw_sof: Option<crate::hw_sof::HwSof<'a, PIO>>,

    /// Scratch buffer for the NRZI/bit-stuff encoder.
    enc: [u8; crate::encoding::MAX_ENCODED_PACKET_BYTES],

    /// Transmit driver for sending packets on the bus.
    tx: TxDriver<'a, PIO>,

    /// Receive driver for receiving packets from the bus.
    rx: RxDriver<'a, PIO>,
}

impl<'a, PIO: UsbPioInstance> Bus<'a, PIO> {
    /// Construct a root-port bus from a PIO block, adjacent D+/D- pins, and IRQ binding.
    ///
    /// The PIO state machines are assigned as TX on SM0, RX edge detection on SM1, and
    /// RX decoding on SM2. `dp` and `dm` must be adjacent GPIOs; the constructor panics
    /// otherwise.
    pub fn new<Irq0>(
        pio: Peri<'a, PIO>,
        dp: Peri<'a, impl PioPin>,
        dm: Peri<'a, impl PioPin>,
        irq0: Irq0,
        pulldown: Pulldown,
    ) -> Self
    where
        Irq0: Binding<<PIO as Instance>::Interrupt, InterruptHandler<PIO>>,
    {
        let (dpn, dmn) = (dp.pin(), dm.pin());
        assert_eq!(
            dpn.abs_diff(dmn),
            1,
            "PIO USB bus requires adjacent USB pins"
        );

        // All three USB state machines on the chosen PIO: TX = sm0, detector = sm1,
        // decoder = sm2.
        let Pio {
            mut common,
            sm0: tx_sm,
            sm1: rx_det_sm,
            sm2: rx_dec_sm,
            sm3: sof_sm,
            ..
        } = Pio::new(pio, irq0);
        #[cfg(not(any(feature = "rp235xa", feature = "rp235xb")))]
        let _ = sof_sm;

        // Convert GPIO peripherals into PIO-owned pins before configuring pads.
        let mut dp = common.make_pio_pin(dp);
        let mut dm = common.make_pio_pin(dm);

        let pulldown_cfg = match pulldown {
            Pulldown::Internal => embassy_rp::gpio::Pull::Down,
            Pulldown::External => embassy_rp::gpio::Pull::None,
        };
        dp.set_pull(pulldown_cfg);
        dm.set_pull(pulldown_cfg);

        // Full-speed drivers must switch in 4–20 ns into the ~45 Ω single-ended cable
        // impedance (USB 2.0 §7.1.2). The pad reset default (2/4 mA, slow slew) is too
        // weak for that; use the strongest setting, as Pico-PIO-USB does
        // (`port_pin_drive_setting`).
        for pin in [&mut dp, &mut dm] {
            pin.set_drive_strength(embassy_rp::gpio::Drive::_12mA);
            pin.set_slew_rate(embassy_rp::gpio::SlewRate::Fast);
        }

        // The PIO programs are written for inverted line sense.
        // Most of the interesting states are SE0 (both low) and J/K (one high, one low).
        // PIO only supports waiting/conditionally jumping when a pin is high,
        // so invert the inputs to simplify the PIO programs.
        crate::chip::set_gpio_input_inversion(dpn, true);
        crate::chip::set_gpio_input_inversion(dmn, true);

        let gpio_high_window = crate::chip::configure_pio_gpio_base::<PIO>(dpn, dmn);

        let tx = TxDriver::init(&mut common, tx_sm, &dp, &dm, gpio_high_window);
        let rx = RxDriver::init(
            &mut common,
            rx_det_sm,
            rx_dec_sm,
            &dp,
            &dm,
            gpio_high_window,
        );

        let now = now_us();

        Self {
            _common: common,
            dp,
            dm,
            speed: Speed::Full,
            attached: false,
            debounce: 0,
            last_activity: now,
            last_sof: now,
            sof_slot: now / FRAME_INTERVAL_US,
            enc: [0u8; crate::encoding::MAX_ENCODED_PACKET_BYTES],
            tx,
            rx,
            #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
            sof_sm: Some(sof_sm),
            #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
            hw_sof: None,
        }
    }

    /// Switch to hardware-timed SOFs (see [`crate::hw_sof`]) using PIO state machine 3,
    /// `pwm` and `dma`. Once only; later calls return without effect.
    #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
    pub(crate) fn enable_hw_sof<S: embassy_rp::pwm::Slice, C: embassy_rp::dma::ChannelInstance>(
        &mut self,
        pwm: Peri<'a, S>,
        dma: Peri<'a, C>,
    ) {
        let Some(sm) = self.sof_sm.take() else {
            return;
        };
        self.hw_sof = Some(crate::hw_sof::HwSof::new(sm, pwm, dma));
        if self.attached && self.speed == Speed::Full {
            self.hw_sof_arm(true);
        }
    }

    /// Whether SOFs are currently sent by the hardware path.
    #[inline(always)]
    fn hw_sof_active(&self) -> bool {
        #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
        {
            self.hw_sof.as_ref().is_some_and(|h| h.armed())
        }
        #[cfg(not(any(feature = "rp235xa", feature = "rp235xb")))]
        {
            false
        }
    }

    /// Arm / disarm the hardware SOF (no-op without one).
    fn hw_sof_arm(&mut self, on: bool) {
        #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
        if let Some(h) = self.hw_sof.as_mut() {
            h.arm(on, self.tx.config());
            if on {
                h.service(self.tx.start_instr());
            }
        }
        #[cfg(not(any(feature = "rp235xa", feature = "rp235xb")))]
        let _ = on;
    }

    /// Prepare the next hardware SOF if due (no-op without one).
    #[inline(always)]
    fn hw_sof_service(&mut self) {
        #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
        if let Some(h) = self.hw_sof.as_mut() {
            h.service(self.tx.start_instr());
        }
    }

    /// Microseconds into the current frame, from the hardware SOF timebase.
    #[inline(always)]
    fn hw_frame_pos_us(&self) -> u32 {
        #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
        if let Some(h) = self.hw_sof.as_ref() {
            return h.frame_pos_us();
        }
        now_us() % FRAME_INTERVAL_US
    }

    /// Hardware-SOF frame guard: no token before this frame's SOF is out, none within
    /// [`FRAME_GUARD_US`] of the next boundary.
    #[inline(always)]
    fn hw_frame_guard(&self) {
        #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
        loop {
            let pos = self.hw_frame_pos_us();
            if pos >= crate::hw_sof::SOF_DONE_US && FRAME_INTERVAL_US - pos >= FRAME_GUARD_US {
                break;
            }
        }
    }

    /// Sense the attached device speed from the idle line state.
    ///
    /// A full-speed device pulls D+ high; a low-speed device pulls D- high
    /// (USB 2.0 §7.1.5.1). Both GPIO inputs are inverted for the RX PIO programs,
    /// so a real high level reads as `false` through [`crate::chip::gpio_input_level`].
    /// Returns `None` for SE0/no-device or transient invalid states; callers use
    /// [`Self::poll_device_event`] to debounce that raw state.
    fn detect_speed(&self) -> Option<Speed> {
        let dp_high = !crate::chip::gpio_input_level(self.dp.pin()); // real D+ high (inverted input reads 0)
        let dm_high = !crate::chip::gpio_input_level(self.dm.pin()); // real D- high
        match (dp_high, dm_high) {
            (true, false) => Some(Speed::Full), // D+ pulled up ⇒ full-speed device
            (false, true) => Some(Speed::Low),  // D- pulled up ⇒ low-speed device
            _ => None,                          // SE0 (no device) or transient/invalid
        }
    }

    fn set_speed(&mut self, speed: Speed) {
        // Re-armed by the next bus reset if the new speed is full speed.
        self.hw_sof_arm(false);
        self.speed = speed;
        self.tx.set_speed(speed);
        self.rx.set_speed(speed);
    }

    /// Send the full-speed SOF for the current 1 ms frame if it has not been sent yet.
    ///
    /// Frames are slots of the hardware microsecond timer (`now_us / 1000`), so the mean
    /// SOF period is exactly 1 ms regardless of executor latency, and the 11-bit frame
    /// number is the slot index. Called from the idle keep-alive *and* before every
    /// token, so a long control transfer (many NAK polls, or several transfers issued
    /// back to back without yielding) keeps emitting SOFs.
    ///
    /// USB 2.0 §8.4.3.1: the host issues an SOF at the start of every full-speed frame,
    /// whether or not other traffic keeps the bus busy.
    fn sof_if_due(&mut self) {
        if !self.attached || self.speed != Speed::Full {
            return;
        }
        let now = now_us();
        let slot = now / FRAME_INTERVAL_US;
        // The spacing guard stops a late SOF at the end of one slot from being followed
        // almost immediately by the next slot's SOF.
        let gap = now.wrapping_sub(self.last_sof);
        if slot != self.sof_slot && gap >= MIN_SOF_SPACING_US {
            self.sof_slot = slot;
            self.last_sof = now;
            LAST_SOF_SLOT.store(slot, Ordering::Relaxed);
            let sof = crate::encoding::build_sof((slot & 0x7ff) as u16);
            self.tx.transmit(&sof);
        }
    }

    /// Do not start a transaction that could still be on the wire at the next frame
    /// boundary; wait for the boundary instead so the SOF goes out on time (the host's
    /// end-of-frame guard, USB 2.0 §11.2.5).
    ///
    /// This also means the frame-timer interrupt (see `Bus::start_frame_timer`) never
    /// lands in the middle of a transaction's reply/handshake turnaround.
    #[inline(always)]
    fn wait_frame_guard(&self) {
        if !self.attached || self.speed != Speed::Full {
            return;
        }
        let now = now_us();
        if FRAME_INTERVAL_US - now % FRAME_INTERVAL_US < FRAME_GUARD_US {
            let slot = now / FRAME_INTERVAL_US;
            while now_us() / FRAME_INTERVAL_US == slot {}
        }
    }

    /// Low-speed keep-alive: send a single low-speed **EOP** via the TX player. Encoding
    /// an empty payload yields just `[SE0, COMP]`, so the player drives SE0 for its EOP
    /// slot — `irq 0 side 0b00 [7]` = 8 SM cycles = **1.33 µs at the LS clock = exactly 2
    /// LS bit-times** — then releases. A spec-correct keep-alive (USB 2.0 §7.1.7.4 /
    /// §11.8.4.1); LS devices have no SOF, so this per-frame EOP is what keeps them awake
    /// and gives them a bus-derived frame timebase.
    ///
    /// The precise width matters because reset detection can begin after roughly
    /// 2.5 µs of SE0 (T_DETRST, USB 2.0 table 7-14). Letting the PIO player time
    /// the EOP from the low-speed divider keeps it at two bit-times.
    fn ls_keepalive(&mut self) {
        self.mark_activity();
        self.tx.transmit(&crate::encoding::LS_KEEPALIVE_PACKET);
        self.mark_activity();
    }

    /// Emit the SOF / low-speed keep-alive if one is due.
    ///
    /// Returns the number of microseconds until the next one is due, so the idle task can
    /// sleep exactly until the next frame boundary.
    pub(crate) fn keepalive(&mut self) -> u32 {
        if !self.attached {
            return FRAME_INTERVAL_US;
        }

        match self.speed {
            Speed::Full if self.hw_sof_active() => {
                self.hw_sof_service();
                FRAME_INTERVAL_US - self.hw_frame_pos_us()
            }
            Speed::Full => {
                self.sof_if_due();
                FRAME_INTERVAL_US - now_us() % FRAME_INTERVAL_US
            }
            Speed::Low => {
                if now_us().wrapping_sub(self.last_activity) >= FRAME_INTERVAL_US {
                    self.ls_keepalive()
                }
                FRAME_INTERVAL_US.saturating_sub(now_us().wrapping_sub(self.last_activity))
            }
            _ => FRAME_INTERVAL_US, // only FS and LS are supported by the PIO host transport
        }
    }

    /// Sample and debounce the root-port line state once without waiting.
    ///
    /// A connected event leaves reset to the caller so adapters can keep the
    /// shared bus locked across the state transition and reset, while releasing
    /// it between ordinary debounce samples.
    pub(crate) fn poll_device_event(&mut self) -> Option<DeviceEvent> {
        let speed = self.detect_speed();
        if speed.is_some() {
            self.debounce = (self.debounce + 1).min(DEBOUNCE_CAP);
        } else {
            self.debounce = self.debounce.saturating_sub(1);
        }

        if !self.attached
            && self.debounce >= DEBOUNCE_FRAMES
            && let Some(speed) = speed
        {
            self.set_speed(speed);
            self.attached = true;

            return Some(DeviceEvent::Connected(speed));
        }

        if self.attached && self.debounce == 0 && speed.is_none() {
            self.attached = false;
            self.hw_sof_arm(false);
            self.tx.release_bus();
            return Some(DeviceEvent::Disconnected);
        }

        None
    }

    #[inline(always)]
    pub(crate) async fn wait_for_next_frame() {
        Timer::after_micros(FRAME_INTERVAL_US as u64).await;
    }

    pub(crate) async fn bus_reset(&mut self) {
        SOF_PAUSED.store(true, Ordering::Relaxed);
        // The hardware SOF's state machine must not write the pins over the SE0.
        self.hw_sof_arm(false);
        self.tx.drive_reset_se0();
        Timer::after(Duration::from_micros(RESET_SE0_US)).await;
        self.tx.release_reset();
        SOF_PAUSED.store(false, Ordering::Relaxed);
        if self.speed == Speed::Full {
            self.hw_sof_arm(true);
        }

        for _ in 0..RESET_RECOVERY_FRAMES {
            let next_us = self.keepalive();
            Timer::after_micros(u64::from(next_us)).await;
        }
    }

    /// Record that a packet was just transmitted.
    ///
    /// Any bus activity — SOF, keep-alive, or a transaction's token/DATA — resets
    /// the attached device's 3 ms suspend timer (USB 2.0 §7.1.7.6), so every TX
    /// path stamps this. Uses the RAM-safe [`now_us`] helper because it can run
    /// at transaction boundaries.
    #[inline(always)]
    fn mark_activity(&mut self) {
        self.last_activity = now_us();
    }

    /// Send an IN token, catch the device reply, and ACK valid DATA before returning.
    ///
    /// This spans the TX→RX turnaround (`transmit_for_reply` returns with RX armed, then
    /// `receive_data_and_ack` pre-stages/fires the host ACK), so keep the wrapper itself in
    /// RAM as well as the transmit/receive helpers it calls.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    fn in_reply(&mut self, in_tok: &[u32], pkt: &mut [u8]) -> Result<InReply, PipeError> {
        self.transmit_for_reply(in_tok, None);
        let (n, status, ack_sent) = self.receive_data_and_ack(pkt);
        if status == RxPacketStatus::Overflow {
            self.settle_after_bad_reply();
            return Err(PipeError::Babble);
        }
        if n < 2 {
            if n == 1 {
                self.settle_after_bad_reply();
            }
            return Ok(InReply::NoReply);
        }

        use crate::pid;

        match (ack_sent, pkt[1]) {
            (_, pid::USB_PID_STALL) => Err(PipeError::Stall),
            (_, pid::USB_PID_NAK) => Ok(InReply::Nak),
            (_, pid @ (pid::USB_PID_DATA0 | pid::USB_PID_DATA1)) => {
                let valid_crc = status == RxPacketStatus::ValidData;
                if ack_sent && valid_crc {
                    self.tx.wait();
                }
                if !valid_crc {
                    self.settle_after_bad_reply();
                }
                Ok(InReply::Data {
                    pid,
                    valid_crc,
                    payload_len: n.saturating_sub(4),
                })
            }
            _ => {
                self.settle_after_bad_reply();
                Ok(InReply::Other)
            }
        }
    }

    /// After a reply the receiver could not take in whole (bad CRC, cut short, unknown
    /// PID), the device may still be transmitting: wait until the line has been idle for
    /// [`BUS_IDLE_US`] before anything else goes on the bus (the next token, or the SOF
    /// from the frame-timer interrupt once the bus lock is released).
    ///
    /// Transmitting over a device packet is a collision; a hub between host and device
    /// sees its port still busy at the end of the frame and disables it (babble/LOA,
    /// USB 2.0 §11.8.1).
    fn settle_after_bad_reply(&mut self) {
        let start = now_us();
        let mut idle_since = start;
        loop {
            let now = now_us();
            if self.detect_speed() != Some(self.speed) {
                idle_since = now;
            } else if now.wrapping_sub(idle_since) >= BUS_IDLE_US {
                break;
            }
            if now.wrapping_sub(start) >= BUS_IDLE_TIMEOUT_US {
                break;
            }
        }
        self.mark_activity();
    }

    /// Transmit one or two packets, then arm RX for the device reply.
    ///
    /// Used for token-only IN transactions and token+DATA OUT/SETUP transactions.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    pub(crate) fn transmit_for_reply(&mut self, first: &[u32], second: Option<&[u32]>) {
        self.transmit_for_reply_inner(first, second);
    }

    /// Shared RAM-inlined body for [`transmit_for_reply`](Self::transmit_for_reply)
    /// and [`transmit_and_check_ack`](Self::transmit_and_check_ack).
    #[inline(always)]
    fn transmit_for_reply_inner(&mut self, first: &[u32], second: Option<&[u32]>) {
        // Before the token, not between token and reply: the reply turnaround is the
        // timing-critical part.
        if self.hw_sof_active() {
            self.hw_frame_guard();
            self.hw_sof_service();
        } else {
            self.wait_frame_guard();
            self.sof_if_due();
        }
        self.rx.prepare_for_receive();
        self.tx.transmit(first);
        if let Some(second) = second {
            self.tx.transmit(second);
        }
        self.rx.start_receive();
    }

    /// Transmit a token or token+DATA pair and interpret the device handshake.
    ///
    /// Returns `Ok(true)` for ACK, `Ok(false)` for no reply/NAK/other non-ACK
    /// response, and `Err(PipeError::Stall)` for STALL.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    pub(crate) fn transmit_and_check_ack(
        &mut self,
        first: &[u32],
        second: Option<&[u32]>,
    ) -> Result<bool, PipeError> {
        self.transmit_for_reply_inner(first, second);
        let mut hbuf = [0u8; 8];
        let (hlen, _) = self.rx.receive(&mut hbuf);
        self.mark_activity();
        if hlen < 2 {
            if hlen == 1 {
                self.settle_after_bad_reply();
            }
            return Ok(false);
        }
        match hbuf[1] {
            crate::pid::USB_PID_ACK => Ok(true),
            crate::pid::USB_PID_STALL => Err(PipeError::Stall),
            crate::pid::USB_PID_NAK => Ok(false),
            _ => {
                self.settle_after_bad_reply();
                Ok(false)
            }
        }
    }

    /// Receive one device DATA packet and ACK it immediately if valid.
    ///
    /// `out` receives `[SYNC, PID, payload..., CRC16_lo, CRC16_hi]`. Returns the
    /// packet length, receive status, and whether the pre-staged ACK was sent.
    ///
    /// USB handshakes have tight response timing after EOP (USB 2.0 §7.1.18.2), so
    /// the ACK is preloaded before receiving and fired with a single SM-enable write.
    /// CRC16 is updated per byte as data arrives, letting the EOP path decide validity
    /// with a residue comparison instead of a post-packet CRC pass.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    pub(crate) fn receive_data_and_ack(&mut self, out: &mut [u8]) -> (usize, RxPacketStatus, bool) {
        // Pre-stage the ACK off the EOP→ACK path. Release the bus last so the
        // device reply is not collided with during capture.
        self.tx.prepare_ack_and_release_bus();

        let (n, status) = self.rx.receive(out);
        if status == RxPacketStatus::ValidData {
            // Fire the pre-staged ACK with one MMIO write.
            self.tx.start_tx();
        }
        self.mark_activity();
        (n, status, status == RxPacketStatus::ValidData)
    }

    /// One **IN** transaction (interrupt/bulk): send IN, catch the device reply.
    /// Returns `Ok(Some(payload_len))` on a valid ACKed DATA packet (payload copied
    /// into `out`, SYNC/PID/CRC stripped), `Ok(None)` on NAK / no reply (caller
    /// should poll again), or `Err(Stall)` on a stalled endpoint.
    pub(crate) fn in_once(
        &mut self,
        addr: u8,
        ep: u8,
        expect_data1: bool,
        out: &mut [u8],
    ) -> Result<Option<usize>, PipeError> {
        self.mark_activity(); // an IN transaction is bus activity — stamp before the TX
        let in_tok = crate::encoding::build_token(crate::pid::USB_PID_IN, addr, ep);

        let mut pkt = [0u8; crate::encoding::MAX_DATA_PACKET_BYTES];
        match self.in_reply(&in_tok, &mut pkt)? {
            InReply::NoReply => Ok(None), // no reply caught
            InReply::Data {
                pid,
                valid_crc: true,
                payload_len,
            } => {
                let is_data1 = pid == crate::pid::USB_PID_DATA1;
                if is_data1 != expect_data1 {
                    return Ok(None);
                }
                if payload_len > out.len() {
                    return Err(PipeError::BufferOverflow);
                }
                let copy = payload_len.min(out.len());
                out[..copy].copy_from_slice(&pkt[2..2 + copy]);
                Ok(Some(copy))
            }
            InReply::Data {
                valid_crc: false, ..
            } => Ok(None), // DATA caught but CRC failed
            InReply::Nak => Ok(None),
            InReply::Other => Ok(None),
        }
    }

    /// One **OUT** transaction (interrupt/bulk or one control-OUT data packet): OUT token + DATA →
    /// device handshake. `data1` selects the DATA1/DATA0 toggle. Returns `Ok(true)`
    /// on device ACK, `Ok(false)` on NAK (caller retries), `Err(Stall)` on stall.
    ///
    /// This helper sends a single DATA packet; callers that need multi-packet OUT
    /// transfers must split the payload and manage toggles.
    pub(crate) fn out_once(
        &mut self,
        addr: u8,
        ep: u8,
        data1: bool,
        data: &[u8],
    ) -> Result<bool, PipeError> {
        let out_tok = crate::encoding::build_token(crate::pid::USB_PID_OUT, addr, ep);
        let pid = if data1 {
            crate::pid::USB_PID_DATA1
        } else {
            crate::pid::USB_PID_DATA0
        };
        let mut dbuf = [0u8; crate::encoding::MAX_DATA_PACKET_BYTES];

        let mut data_w = [0u32; crate::encoding::MAX_DATA_PACKET_WORDS];
        let data_w = crate::encoding::build_data(pid, data, &mut dbuf, &mut self.enc, &mut data_w)
            .ok_or(PipeError::BufferOverflow)?;

        self.transmit_and_check_ack(&out_tok, Some(data_w))
    }

    /// Send the SETUP token and DATA0 setup packet, expecting an ACK handshake.
    pub(crate) fn control_setup(
        &mut self,
        addr: u8,
        ep: u8,
        setup: &[u8; 8],
    ) -> Result<(), PipeError> {
        let setup_tok = crate::encoding::build_token(crate::pid::USB_PID_SETUP, addr, ep);
        let mut data0 = [0u8; crate::encoding::MAX_DATA_PACKET_BYTES];
        let mut data_w = [0u32; crate::encoding::MAX_DATA_PACKET_WORDS];
        let data_w = crate::encoding::build_data(
            crate::pid::USB_PID_DATA0,
            setup,
            &mut data0,
            &mut self.enc,
            &mut data_w,
        )
        .ok_or(PipeError::BufferOverflow)?;

        if self.transmit_and_check_ack(&setup_tok, Some(data_w))? {
            Ok(())
        } else {
            Err(PipeError::Timeout)
        }
    }

    /// One DATA-stage IN poll of a control-IN transfer.
    ///
    /// `Ok(Some(n))`: DATA with the expected toggle, ACKed; its payload is `out[..n]`.
    /// `Ok(None)`: NAK, no reply, an undecodable packet, or a repeat of the previous
    /// packet (wrong toggle, ACKed again): poll again later, the stage is unchanged.
    ///
    /// Retrying (and giving up) is the caller's job; see `embassy::PioPipe`.
    pub(crate) fn control_data_in(
        &mut self,
        addr: u8,
        ep: u8,
        expect_data1: bool,
        out: &mut [u8],
    ) -> Result<Option<usize>, PipeError> {
        let in_tok = crate::encoding::build_token(crate::pid::USB_PID_IN, addr, ep);
        let mut pkt = [0u8; crate::encoding::MAX_DATA_PACKET_BYTES];
        match self.in_reply(&in_tok, &mut pkt)? {
            InReply::Data {
                pid,
                valid_crc: true,
                payload_len,
            } => {
                if (pid == crate::pid::USB_PID_DATA1) != expect_data1 {
                    return Ok(None);
                }
                if payload_len > out.len() {
                    return Err(PipeError::BufferOverflow);
                }
                out[..payload_len].copy_from_slice(&pkt[2..2 + payload_len]);
                Ok(Some(payload_len))
            }
            InReply::Nak | InReply::NoReply | InReply::Data { .. } | InReply::Other => Ok(None),
        }
    }

    /// One STATUS-stage IN poll of a control-OUT transfer (the device returns a
    /// zero-length DATA1).
    ///
    /// `Ok(true)`: status received, ACKed. `Ok(false)`: NAK (the device is still
    /// processing the request), no reply or an undecodable packet: poll again later.
    /// USB 2.0 §8.5.3.1 mandates DATA1; DATA0 is accepted too, for non-compliant devices.
    pub(crate) fn control_status_in(&mut self, addr: u8, ep: u8) -> Result<bool, PipeError> {
        let in_tok = crate::encoding::build_token(crate::pid::USB_PID_IN, addr, ep);
        let mut pkt = [0u8; 8];
        match self.in_reply(&in_tok, &mut pkt)? {
            InReply::Data {
                valid_crc: true,
                payload_len: 0,
                ..
            } => Ok(true),
            InReply::Data {
                valid_crc: true, ..
            } => Err(PipeError::BadResponse),
            InReply::Nak | InReply::NoReply | InReply::Data { .. } | InReply::Other => Ok(false),
        }
    }

    /// Single nonblocking step of a bulk/interrupt IN transfer.
    pub(crate) fn request_in(
        &mut self,
        addr: u8,
        ep: u8,
        out: &mut [u8],
        toggle_data1: &mut bool,
    ) -> Result<usize, PipeError> {
        match self.in_once(addr, ep, *toggle_data1, out) {
            Ok(Some(n)) => {
                *toggle_data1 = !*toggle_data1;
                Ok(n)
            }
            Ok(None) => Err(PipeError::Timeout),
            Err(err) => Err(err),
        }
    }

    /// Single nonblocking step of a bulk/interrupt OUT transfer.
    pub(crate) fn request_out(
        &mut self,
        addr: u8,
        ep: u8,
        data: &[u8],
        toggle_data1: &mut bool,
    ) -> Result<(), PipeError> {
        match self.out_once(addr, ep, *toggle_data1, data) {
            Ok(true) => {
                *toggle_data1 = !*toggle_data1;
                Ok(())
            }
            Ok(false) => Err(PipeError::Timeout),
            Err(err) => Err(err),
        }
    }
}
