//! Hardware-timed full-speed SOF (RP235x).
//!
//! USB 2.0 §7.1.12 gives the host a 1.000 ms ± 0.5 µs frame, and hubs derive their
//! end-of-frame points (EOF1/EOF2, §11.2.5) from the SOF period. An SOF sent by an
//! interrupt competes with transfers for the bus and with other interrupts for the
//! core, and was measured tens to hundreds of microseconds late; a hub then disabled
//! its port for babble. Here no CPU is on the timing path:
//!
//! - a PWM slice wraps exactly every 1 ms (from `clk_sys`, aligned once to the 1 ms
//!   boundaries of the microsecond timer);
//! - each wrap paces one transfer of a DMA channel that writes the SOF state machine's
//!   enable bit to the PIO `CTRL` atomic-set alias;
//! - PIO state machine 3 runs the same TX player program and configuration as the
//!   transfer state machine (SM0) and is kept prepared, between frames, with the next
//!   frame's SOF in its FIFO and its program counter at the player's `start:` slot.
//!
//! The CPU only has to prepare SM3 once per frame, any time between the end of the SOF
//! and shortly before the next boundary ([`HwSof::service`]), and transfers keep clear
//! of the SOF: no token within [`crate::bus`]'s end-of-frame guard, none before the SOF
//! is out ([`SOF_DONE_US`]). Preparing SM3 executes a side-set on the shared pins, so
//! it is only done by the holder of the bus lock, i.e. never during a transfer.

use crate::pio_instance::UsbPioInstance;
use crate::ram;
use core::ptr::addr_of;
use embassy_rp::pio::{Config, StateMachine};
use embassy_rp::pwm::{Pwm, Slice};
use embassy_rp::{Peri, dma};
use rp_pac as pac;
use rp_pac::dma::vals::{DataSize, TransCountMode, TreqSel};

/// Full-speed frame, in microseconds.
const FRAME_US: u32 = 1000;

/// The SOF (32 bit times ≈ 2.7 µs plus EOP) is on the wire within this long of the
/// frame boundary; transfers wait for it and SM3 is not prepared before it.
pub(crate) const SOF_DONE_US: u32 = 8;

/// SM3 is not prepared this close to the next boundary (the DMA could enable it while
/// its FIFO is half filled).
const PREPARE_GUARD_US: u32 = 30;

/// State machine index used for SOFs.
const SOF_SM: usize = 3;

/// Written by the DMA channel to `CTRL` (atomic-set alias): SM_ENABLE bit of SM3.
static SOF_SM_ENABLE: u32 = 1 << SOF_SM;

/// Offset of the atomic bit-set alias of an RP peripheral register.
const REG_ALIAS_SET: u32 = 0x2000;

/// DREQ of PWM slice 0's wrap; slice `n` is `+ n` (RP2350 datasheet, DMA DREQ table).
const DREQ_PWM_WRAP0: u8 = 0x20;

pub(crate) struct HwSof<'d, PIO: UsbPioInstance> {
    sm: StateMachine<'d, PIO, SOF_SM>,
    /// Keeps the slice owned (and running) for the bus's lifetime.
    _pwm: Pwm<'d>,
    slice: usize,
    dma_ch: usize,
    counts_per_us: u32,
    armed: bool,
    /// Timer slot (`now_us / 1000`) whose SOF SM3 currently holds, or `None`.
    prepared_slot: Option<u32>,
}

impl<'d, PIO: UsbPioInstance> HwSof<'d, PIO> {
    /// Take SM3, a PWM slice and a DMA channel. The slice starts wrapping at the 1 ms
    /// boundaries right away; SOFs start once [`Self::arm`] is called.
    ///
    /// Panics if `clk_sys` is not a multiple of 1 kHz reachable with an integer PWM
    /// divider (any MHz-multiple system clock up to 255 × 65.536 MHz is).
    pub(crate) fn new<S: Slice, C: dma::ChannelInstance>(
        sm: StateMachine<'d, PIO, SOF_SM>,
        pwm: Peri<'d, S>,
        _dma: Peri<'d, C>,
    ) -> Self {
        let slice = pwm.number();
        let pwm = Pwm::new_free(pwm, Default::default());

        let clk = embassy_rp::clocks::clk_sys_freq();
        let per_frame = clk / 1000;
        let div = (1..=255u32)
            .find(|&d| per_frame.is_multiple_of(d) && per_frame / d <= 65536)
            .expect("clk_sys: no integer PWM divider gives a 1 ms period");
        let counts = per_frame / div;
        assert!(
            counts.is_multiple_of(1000),
            "clk_sys / PWM divider must be a whole number of counts per microsecond"
        );

        let ch = pac::PWM.ch(slice);
        ch.csr().write(|w| {
            w.set_en(false);
            w.set_ph_correct(false);
        });
        ch.div().write(|w| {
            w.set_int(div as u8);
            w.set_frac(0);
        });
        ch.top().write(|w| w.set_top((counts - 1) as u16));

        let this = Self {
            sm,
            _pwm: pwm,
            slice,
            dma_ch: C::number() as usize,
            counts_per_us: counts / 1000,
            armed: false,
            prepared_slot: None,
        };
        this.align();
        this
    }

    /// Start the PWM counter so that it wraps on the microsecond timer's 1 ms
    /// boundaries (to within a microsecond; both run from the same crystal).
    fn align(&self) {
        let ch = pac::PWM.ch(self.slice);
        ch.csr().modify(|w| w.set_en(false));
        let into_frame = ram::now_us() % FRAME_US;
        ch.ctr()
            .write(|w| w.set_ctr((into_frame * self.counts_per_us) as u16));
        ch.csr().modify(|w| w.set_en(true));
    }

    /// Microseconds since the start of the current frame.
    #[inline(always)]
    pub(crate) fn frame_pos_us(&self) -> u32 {
        u32::from(pac::PWM.ch(self.slice).ctr().read().ctr()) / self.counts_per_us
    }

    pub(crate) fn armed(&self) -> bool {
        self.armed
    }

    /// Start (or stop) sending SOFs at every frame boundary.
    ///
    /// `txcfg`/`start_instr` are the transfer state machine's current configuration and
    /// player entry instruction (full speed). Disarm while the port is reset or not
    /// running full speed: an SM enabled by the DMA would write its side-set to the
    /// shared pins (e.g. over a reset's SE0).
    pub(crate) fn arm(&mut self, on: bool, txcfg: &Config<'d, PIO>) {
        self.stop_dma();
        ram::pio_sm_disable::<PIO, SOF_SM>();
        self.prepared_slot = None;
        self.armed = on;
        if !on {
            return;
        }
        self.sm.set_config(txcfg);
        let ch = pac::DMA.ch(self.dma_ch);
        ch.read_addr().write_value(addr_of!(SOF_SM_ENABLE) as u32);
        ch.write_addr()
            .write_value(PIO::REGS.ctrl().as_ptr() as u32 + REG_ALIAS_SET);
        ch.trans_count().write(|w| {
            w.set_mode(TransCountMode::ENDLESS);
            w.set_count(1);
        });
        ch.ctrl_trig().write(|w| {
            w.set_en(true);
            w.set_data_size(DataSize::SIZE_WORD);
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_chain_to(self.dma_ch as u8); // chaining to itself = none
            w.set_treq_sel(TreqSel::from_bits(DREQ_PWM_WRAP0 + self.slice as u8));
            w.set_irq_quiet(true);
        });
    }

    fn stop_dma(&self) {
        pac::DMA
            .chan_abort()
            .write(|w| w.set_chan_abort(1 << self.dma_ch));
        while pac::DMA.chan_abort().read().chan_abort() & (1 << self.dma_ch) != 0 {}
    }

    /// Prepare SM3 with the next frame's SOF, if it is not already and the timing
    /// allows. Call with the bus lock held (no transfer on the wire).
    pub(crate) fn service(&mut self, start_instr: u16) {
        if !self.armed {
            return;
        }
        let pos = self.frame_pos_us();
        if !(SOF_DONE_US..FRAME_US - PREPARE_GUARD_US).contains(&pos) {
            return;
        }
        let next = ram::now_us() / FRAME_US + 1;
        if self.prepared_slot == Some(next) {
            return;
        }

        let sof = crate::encoding::build_sof((next & 0x7ff) as u16);
        ram::pio_sm_disable::<PIO, SOF_SM>();
        ram::pio_sm_clear_fifos::<PIO, SOF_SM>();
        ram::pio_sm_restart::<PIO, SOF_SM>();
        // Program counter to the player's `start:` slot. The instruction carries the
        // idle-J side-set, harmless here: nobody else drives the pins.
        ram::pio_sm_exec_instr::<PIO, SOF_SM>(start_instr);
        for w in sof {
            ram::pio_sm_push_tx::<PIO, SOF_SM>(w);
        }
        self.prepared_slot = Some(next);
    }
}
