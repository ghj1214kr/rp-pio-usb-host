//! Async `embassy` integration for sharing the direct PIO USB bus.
//!
//! The wrapper serializes access to the physical root port, provides an idle task
//! for frame keep-alives, and exposes the host-controller/allocator types used by
//! `embassy-usb-host`.

use crate::bus::{Bus as PioUsbBus, Pulldown};
use crate::pio_instance::UsbPioInstance;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_rp::Peri;
use embassy_rp::interrupt::InterruptExt;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{Instance, InterruptHandler, PioPin};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator,
    UsbHostController, UsbPipe, pipe,
};
use embassy_usb_driver::{EndpointInfo, EndpointType};

const FRAME_INTERVAL_US: u32 = 1000; // 1 ms

/// TIMER alarm used by [`Bus::start_frame_timer`].
const FRAME_ALARM: usize = 1;

#[cfg(feature = "rp2040")]
use embassy_rp::interrupt::TIMER_IRQ_1 as FRAME_TIMER_IRQ;
#[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
use embassy_rp::interrupt::TIMER0_IRQ_1 as FRAME_TIMER_IRQ;

/// Arm the frame alarm for the next 1 ms boundary of the microsecond timer.
fn arm_frame_alarm() {
    let offset = if HW_SOF.load(Ordering::Relaxed) {
        HW_SERVICE_OFFSET_US
    } else {
        0
    };
    let alarm = crate::ram::timer().alarm(FRAME_ALARM);
    let since = crate::ram::now_us().wrapping_sub(offset);
    let next = (since / FRAME_INTERVAL_US + 1)
        .wrapping_mul(FRAME_INTERVAL_US)
        .wrapping_add(offset);
    alarm.write_value(next);
    // Alarms fire only on an exact match with the timer's low word; if the target
    // passed while arming, take the one after.
    if crate::ram::now_us().wrapping_sub(next) as i32 >= 0 {
        alarm.write_value(next.wrapping_add(FRAME_INTERVAL_US));
    }
}

/// Set once SOFs come from hardware: the frame-timer interrupt then fires
/// [`HW_SERVICE_OFFSET_US`] after each boundary and only prepares the next SOF.
static HW_SOF: AtomicBool = AtomicBool::new(false);

/// With hardware SOFs, when after the boundary the frame-timer interrupt prepares the
/// next SOF (after the current one is out).
const HW_SERVICE_OFFSET_US: u32 = 20;

/// With hardware SOFs, retry interval when the bus was busy (not timing-critical).
const HW_SERVICE_RETRY_US: u32 = 20;

/// With hardware SOFs, the last point in the frame the interrupt tries to prepare.
const HW_SERVICE_WINDOW_US: u32 = 900;

/// Retry delay of the frame alarm when the bus was busy at the frame boundary.
///
/// Short: a holder that is not a transfer (e.g. the idle task's own check) releases
/// the bus within microseconds, and every retry step is SOF jitter.
/// Hubs derive their end-of-frame points from the SOF period, and USB 2.0 §7.1.12
/// allows the host ±0.5 µs; a transfer that holds the bus sends the SOF itself, which
/// stops the retries (see `LAST_SOF_SLOT`).
const FRAME_RETRY_US: u32 = 2;

/// Retries stop this far into the frame; later, the next boundary is armed instead.
/// (Matches the SOF spacing guard: an SOF this late would hold back the next one.)
const FRAME_RETRY_WINDOW_US: u32 = FRAME_INTERVAL_US / 2;

/// Re-arm the frame alarm after the bus was busy at a frame boundary: retry shortly
/// while this frame's SOF is still missing and early enough to send it, otherwise wait
/// for the next boundary.
fn arm_retry_alarm() {
    let now = crate::ram::now_us();
    if HW_SOF.load(Ordering::Relaxed) {
        // Preparing the next hardware SOF can wait; the SOF itself is not late.
        if now % FRAME_INTERVAL_US >= HW_SERVICE_WINDOW_US {
            arm_frame_alarm();
        } else {
            let target = now.wrapping_add(HW_SERVICE_RETRY_US);
            crate::ram::timer().alarm(FRAME_ALARM).write_value(target);
            if crate::ram::now_us().wrapping_sub(target) as i32 >= 0 {
                FRAME_TIMER_IRQ.pend();
            }
        }
        return;
    }
    let slot = now / FRAME_INTERVAL_US;
    let sent = crate::bus::LAST_SOF_SLOT.load(core::sync::atomic::Ordering::Relaxed) == slot;
    let paused = crate::bus::SOF_PAUSED.load(core::sync::atomic::Ordering::Relaxed);
    if sent || paused || now % FRAME_INTERVAL_US >= FRAME_RETRY_WINDOW_US {
        arm_frame_alarm();
    } else {
        let alarm = crate::ram::timer().alarm(FRAME_ALARM);
        let target = now.wrapping_add(FRAME_RETRY_US);
        alarm.write_value(target);
        // Alarms fire only on an exact match with the timer's low word: if the target
        // passed while arming, it would not fire for ~71 minutes. Re-enter now instead.
        if crate::ram::now_us().wrapping_sub(target) as i32 >= 0 {
            FRAME_TIMER_IRQ.pend();
        }
    }
}

/// Shared, interior-mutable bus state referenced by the controller and all pipes.
///
/// Construct once (typically into a `StaticCell`) and hand out a controller via
/// [`Bus::controller`]; pipes obtained through the controller's allocator
/// borrow the same `&'d` shared bus.
pub struct Bus<'d, PIO: UsbPioInstance = PIO0> {
    /// The single physical bus shared by the controller and all allocated pipes.
    bus: Mutex<CriticalSectionRawMutex, PioUsbBus<'d, PIO>>,
}

impl<'d, PIO: UsbPioInstance> Bus<'d, PIO> {
    /// Construct a shared async bus from the raw PIO peripheral, USB pins, and IRQ binding.
    pub fn new<Irq0>(
        pio: Peri<'d, PIO>,
        dp: Peri<'d, impl PioPin>,
        dm: Peri<'d, impl PioPin>,
        irq0: Irq0,
        pulldown: Pulldown,
    ) -> Self
    where
        Irq0: Binding<<PIO as Instance>::Interrupt, InterruptHandler<PIO>>,
    {
        let bus = PioUsbBus::new(pio, dp, dm, irq0, pulldown);
        Self {
            bus: Mutex::new(bus),
        }
    }

    /// Obtain the root-port controller handle.
    pub fn controller<'a>(&'a self) -> PioUsbController<'a, 'd, PIO> {
        PioUsbController { shared: self }
    }

    /// Perform one non-blocking keep-alive check and return the microseconds until the
    /// next one is due.
    ///
    /// If an in-flight transfer owns the bus, it emits the SOF itself before its next
    /// token, so this check only has to cover an idle bus. [`Self::idle_task`] calls this
    /// once per frame.
    fn tick(&self) -> u32 {
        match self.bus.try_lock() {
            Ok(mut bus) => bus.keepalive(),
            Err(_) => FRAME_INTERVAL_US / 4,
        }
    }

    /// Start timer-driven SOFs.
    ///
    /// Arms TIMER alarm 1 (the embassy-rp time driver uses alarm 0) to fire at every
    /// 1 ms frame boundary and enables its interrupt — `TIMER0_IRQ_1` on RP235x,
    /// `TIMER_IRQ_1` on RP2040 — on the calling core. The application's handler for
    /// that interrupt must call [`Self::on_frame_timer`].
    ///
    /// Without it, SOFs are sent by [`Self::idle_task`] and inherit executor latency
    /// (tens of µs to milliseconds of jitter) instead of the 1.000 ms ± 500 ppm frame
    /// period of USB 2.0 §7.1.12. Keep running `idle_task` as well; it covers
    /// low-speed keep-alives.
    pub fn start_frame_timer(&self) {
        let timer = crate::ram::timer();
        timer.intr().write(|w| w.set_alarm(FRAME_ALARM, true));
        timer.inte().modify(|w| w.set_alarm(FRAME_ALARM, true));
        arm_frame_alarm();
        // SAFETY: the handler is provided by the application (see above).
        unsafe { FRAME_TIMER_IRQ.enable() };
    }

    /// Send SOFs from hardware (PIO state machine 3, paced by `pwm` through `dma`) instead
    /// of from the frame-timer interrupt, for an SOF period free of CPU latency. The
    /// frame-timer interrupt (see [`Self::start_frame_timer`]) then only prepares the
    /// next SOF, shortly after each frame boundary. Returns `false` if a transfer holds
    /// the bus (call it before starting the host stack).
    #[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
    pub fn enable_hw_sof(
        &self,
        pwm: Peri<'d, impl embassy_rp::pwm::Slice>,
        dma: Peri<'d, impl embassy_rp::dma::ChannelInstance>,
    ) -> bool {
        let Ok(mut bus) = self.bus.try_lock() else {
            return false;
        };
        bus.enable_hw_sof(pwm, dma);
        HW_SOF.store(true, Ordering::Relaxed);
        true
    }

    /// Frame-timer interrupt body: send this frame's SOF and re-arm for the next frame.
    ///
    /// If a transfer holds the bus, it sends the SOF itself before its next token (its
    /// end-of-frame guard keeps transactions clear of the boundary).
    pub fn on_frame_timer(&self) {
        crate::ram::timer()
            .intr()
            .write(|w| w.set_alarm(FRAME_ALARM, true));
        match self.bus.try_lock() {
            Ok(mut bus) => {
                bus.keepalive();
                arm_frame_alarm();
            }
            // Someone holds the bus at the frame boundary. A transfer sends the SOF
            // before its next token, but the holder may be about to release it (e.g. the
            // idle task's own tick); giving up here would leave the whole frame without an
            // SOF, so retry shortly.
            Err(_) => arm_retry_alarm(),
        }
    }

    /// Run the keep-alive task for as long as the bus is in use.
    ///
    /// This future never returns. Spawn it once beside the USB host stack so full-speed
    /// devices receive SOFs and low-speed devices receive keep-alive EOPs between
    /// transfers.
    pub async fn idle_task(&self) {
        loop {
            let next_us = self.tick();
            Timer::after_micros(u64::from(next_us)).await;
        }
    }
}

/// A single endpoint pipe; implements [`UsbPipe`].
///
/// Carries the addressing it needs to build tokens at runtime plus its own data-toggle and timeout state.
pub struct PioPipe<'a, 'd, T: pipe::Type, D: pipe::Direction, PIO: UsbPioInstance = PIO0> {
    /// Shared root-port bus used to execute this pipe's transactions.
    shared: &'a Bus<'d, PIO>,
    /// USB device address.
    addr: u8,
    /// Endpoint number without direction bit.
    ep: u8,
    /// Endpoint maximum packet size.
    mps: u16,
    /// Polling interval for interrupt endpoints, in microseconds.
    interval_us: u32,
    /// Next OUT/IN data toggle (`true` ⇒ DATA1). Initialised to DATA0.
    toggle_data1: bool,
    /// Transfer timeouts supplied by the host stack.
    timeout: TimeoutConfig,
    /// Type-level endpoint kind and direction markers.
    _markers: PhantomData<(T, D)>,
}

fn needs_terminating_zlp(len: usize, mps: usize, ensure_transaction_end: bool) -> bool {
    ensure_transaction_end && len != 0 && len.is_multiple_of(mps)
}

impl<'a, 'd: 'a, T: pipe::Type, D: pipe::Direction, PIO: UsbPioInstance>
    PioPipe<'a, 'd, T, D, PIO>
{
    /// Send one OUT packet, retrying NAK/no-response until cancellation.
    async fn request_out_packet(&mut self, data: &[u8]) -> Result<(), PipeError> {
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.request_out(self.addr, self.ep, data, &mut self.toggle_data1)
            };

            match res {
                Err(PipeError::Timeout) => {
                    Timer::after(Duration::from_micros(self.interval_us as u64)).await;
                }
                _ => return res,
            }
        }
    }

    /// SETUP stage: resent only while the device does not ACK it. Once ACKed the request
    /// is running in the device, and sending it again would restart it (a hub's
    /// SET_FEATURE(PORT_RESET), SET_CONFIGURATION, ...).
    async fn setup_stage(&mut self, setup: &[u8; 8], deadline: Instant) -> Result<(), PipeError> {
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.control_setup(self.addr, self.ep, setup)
            };
            match res {
                Err(PipeError::Timeout) if Instant::now() < deadline => stage_retry_wait().await,
                _ => return res,
            }
        }
    }

    /// One DATA-stage IN packet, polled until it arrives or `deadline` passes.
    async fn data_in_packet(
        &mut self,
        expect_data1: bool,
        out: &mut [u8],
        deadline: Instant,
    ) -> Result<usize, PipeError> {
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.control_data_in(self.addr, self.ep, expect_data1, out)
            };
            match res? {
                Some(n) => return Ok(n),
                None if Instant::now() < deadline => stage_retry_wait().await,
                None => return Err(PipeError::Timeout),
            }
        }
    }

    /// One OUT packet of a control DATA or STATUS stage, retried on NAK / no handshake.
    async fn control_out_packet(
        &mut self,
        data1: bool,
        data: &[u8],
        deadline: Instant,
    ) -> Result<(), PipeError> {
        loop {
            let acked = {
                let mut bus = self.shared.bus.lock().await;
                bus.out_once(self.addr, self.ep, data1, data)
            }?;
            if acked {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(PipeError::Timeout);
            }
            stage_retry_wait().await;
        }
    }

    /// STATUS stage of a control write: IN polled until the zero-length packet arrives.
    async fn status_in(&mut self, deadline: Instant) -> Result<(), PipeError> {
        loop {
            let done = {
                let mut bus = self.shared.bus.lock().await;
                bus.control_status_in(self.addr, self.ep)
            }?;
            if done {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(PipeError::Timeout);
            }
            stage_retry_wait().await;
        }
    }
}

/// Pause before retrying a NAKed (or unanswered) control-transfer stage: the next
/// frame, as Pico-PIO-USB / TinyUSB schedule it. The bus is released meanwhile.
async fn stage_retry_wait() {
    Timer::after(Duration::from_micros(FRAME_INTERVAL_US as u64)).await;
}

impl<'a, 'd: 'a, T: pipe::Type, D: pipe::Direction, PIO: UsbPioInstance> UsbPipe<T, D>
    for PioPipe<'a, 'd, T, D, PIO>
{
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        // Stage by stage, each retried in the next frame on NAK until the transfer's
        // deadline; never by resending the SETUP once it was ACKed.
        let deadline =
            Instant::now() + Duration::from_millis(self.timeout.data_timeout.as_millis() as u64);
        // wLength also ends the DATA stage (not only a short packet): needed when `mps`
        // differs from the device's real mps0, e.g. the first 8-byte descriptor read.
        let wlen = usize::from(u16::from_le_bytes([setup[6], setup[7]]));
        if wlen > buf.len() {
            return Err(PipeError::BufferOverflow);
        }
        let mps = usize::from(self.mps).clamp(1, crate::encoding::MAX_DATA_PAYLOAD_BYTES);

        self.setup_stage(setup, deadline).await?;

        let mut total = 0;
        let mut data1 = true; // the first DATA packet of a control transfer is DATA1
        let mut pkt = [0u8; crate::encoding::MAX_DATA_PAYLOAD_BYTES];
        while total < wlen {
            let n = self
                .data_in_packet(data1, &mut pkt[..mps], deadline)
                .await?;
            if total + n > buf.len() {
                return Err(PipeError::BufferOverflow);
            }
            buf[total..total + n].copy_from_slice(&pkt[..n]);
            total += n;
            data1 = !data1;
            if n < mps {
                break;
            }
        }

        // STATUS: OUT zero-length DATA1.
        self.control_out_packet(true, &[], deadline).await?;
        Ok(total)
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        // No-data control writes use `no_data_timeout`; writes with an OUT data stage
        // use `data_timeout`.
        let timeout = if buf.is_empty() {
            self.timeout.no_data_timeout
        } else {
            self.timeout.data_timeout
        };
        let deadline = Instant::now() + Duration::from_millis(timeout.as_millis() as u64);
        let mps = usize::from(self.mps).max(1);

        self.setup_stage(setup, deadline).await?;

        let mut data1 = true;
        for chunk in buf.chunks(mps) {
            self.control_out_packet(data1, chunk, deadline).await?;
            data1 = !data1;
        }

        // STATUS: IN, the device answers with a zero-length packet once the request is
        // done (it NAKs while processing, e.g. a hub port reset).
        self.status_in(deadline).await
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        // Interrupt/bulk IN: NAK means no data yet; wait one endpoint polling
        // interval (minimum one frame) and retry. Callers impose cancellation by
        // dropping this future.
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.request_in(self.addr, self.ep, buf, &mut self.toggle_data1)
            };

            match res {
                Err(PipeError::Timeout) => {
                    Timer::after(Duration::from_micros(self.interval_us as u64)).await;
                }
                _ => return res,
            }
        }
    }

    async fn request_out(
        &mut self,
        buf: &[u8],
        ensure_transaction_end: bool,
    ) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        let mps = usize::from(self.mps);
        for chunk in buf.chunks(mps) {
            self.request_out_packet(chunk).await?;
        }
        if buf.is_empty() || needs_terminating_zlp(buf.len(), mps, ensure_transaction_end) {
            self.request_out_packet(&[]).await?;
        }
        Ok(())
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        self.toggle_data1 = false;
    }
}

/// Pipe allocator handle; implements [`UsbHostAllocator`].
pub struct PioUsbAllocator<'a, 'd, PIO: UsbPioInstance = PIO0> {
    /// Shared root-port bus state.
    shared: &'a Bus<'d, PIO>,
}

// Manual `Clone` (not `#[derive]`): the only field is a shared reference, so cloning
// never touches `PIO` — deriving would spuriously require `PIO: Clone`, which the PIO
// peripheral singleton is not.
impl<'a, 'd, PIO: UsbPioInstance> Clone for PioUsbAllocator<'a, 'd, PIO> {
    fn clone(&self) -> Self {
        Self {
            shared: self.shared,
        }
    }
}

impl<'a, 'd: 'a, PIO: UsbPioInstance> UsbHostAllocator<'a> for PioUsbAllocator<'a, 'd, PIO> {
    type Pipe<T: pipe::Type, D: pipe::Direction> = PioPipe<'a, 'd, T, D, PIO>;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        if split.is_some() {
            // `split == None` covers root-port devices AND full-speed devices behind a
            // full-speed hub (the host stack's hub routing yields no split there), so
            // those work. A `Some(split)` is either low-speed-behind-a-hub (legacy PRE)
            // or a high-speed Transaction-Translator split — neither implemented.
            return Err(HostError::Other(
                "split transactions (LS-via-hub PRE / HS TT) unsupported",
            ));
        }
        if endpoint.ep_type != T::ep_type() {
            return Err(HostError::Other("pipe and endpoint types do not match"));
        }
        if endpoint.ep_type == EndpointType::Isochronous {
            return Err(HostError::Other("isochronous endpoints unsupported"));
        }
        if endpoint.max_packet_size == 0
            || usize::from(endpoint.max_packet_size) > crate::encoding::MAX_DATA_PAYLOAD_BYTES
        {
            return Err(HostError::Other("unsupported endpoint maximum packet size"));
        }
        Ok(PioPipe {
            shared: self.shared,
            addr,
            ep: endpoint.addr.index() as u8,
            mps: endpoint.max_packet_size,
            interval_us: (u32::from(endpoint.interval_ms) * 1000).max(FRAME_INTERVAL_US),
            toggle_data1: false,
            timeout: TimeoutConfig::default(),
            _markers: PhantomData,
        })
    }
}

/// Root-port controller; implements [`UsbHostController`].
pub struct PioUsbController<'a, 'd, PIO: UsbPioInstance = PIO0> {
    /// Shared root-port bus state.
    shared: &'a Bus<'d, PIO>,
}

impl<'a, 'd: 'a, PIO: UsbPioInstance> UsbHostController<'a> for PioUsbController<'a, 'd, PIO> {
    type Allocator = PioUsbAllocator<'a, 'd, PIO>;

    fn allocator(&self) -> Self::Allocator {
        PioUsbAllocator {
            shared: self.shared,
        }
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        loop {
            let event = {
                let mut bus = self.shared.bus.lock().await;
                let event = bus.poll_device_event();
                if matches!(event, Some(DeviceEvent::Connected(_))) {
                    bus.bus_reset().await;
                }
                event
            };
            if let Some(event) = event {
                return event;
            }
            PioUsbBus::<PIO>::wait_for_next_frame().await;
        }
    }

    async fn bus_reset(&mut self) {
        let mut bus = self.shared.bus.lock().await;
        bus.bus_reset().await
    }
}
