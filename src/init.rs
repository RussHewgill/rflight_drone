use dwt_systick_monotonic::DwtSystick;
use stm32f4::stm32f401::{self, RCC};
use stm32f401::{CorePeripherals, Peripherals};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Clocks;

use crate::uart::UART;

use cortex_m::peripheral::{syst::SystClkSource, SYST};

pub struct Systick<const TIMER_HZ: u32> {
    systick: SYST,
    cnt: u64,
}

impl<const TIMER_HZ: u32> Systick<TIMER_HZ> {
    pub fn new(mut systick: SYST, sysclk: u32) -> Self {
        // + TIMER_HZ / 2 provides round to nearest instead of round to 0.
        // - 1 as the counter range is inclusive [0, reload]
        let reload = (sysclk + TIMER_HZ / 2) / TIMER_HZ - 1;

        assert!(reload <= 0x00ff_ffff);
        assert!(reload > 0);

        systick.disable_counter();
        systick.set_clock_source(SystClkSource::Core);
        systick.set_reload(reload);

        Systick { systick, cnt: 0 }
    }
}

impl<const TIMER_HZ: u32> rtic::Monotonic for Systick<TIMER_HZ> {
    const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = false;

    type Instant = fugit::TimerInstantU64<TIMER_HZ>;
    type Duration = fugit::TimerDurationU64<TIMER_HZ>;

    fn now(&mut self) -> Self::Instant {
        if self.systick.has_wrapped() {
            self.cnt += 1;
        }

        Self::Instant::from_ticks(self.cnt)
    }

    unsafe fn reset(&mut self) {
        self.systick.clear_current();
        self.systick.enable_counter();
    }

    #[inline(always)]
    fn set_compare(&mut self, _val: Self::Instant) {
        // No need to do something here, we get interrupts anyway.
    }

    #[inline(always)]
    fn clear_compare_flag(&mut self) {
        // NOOP with SysTick interrupt
    }

    #[inline(always)]
    fn zero() -> Self::Instant {
        Self::Instant::from_ticks(0)
    }

    fn on_interrupt(&mut self) {
        if self.systick.has_wrapped() {
            self.cnt += 1;
        }
    }
}

// pub fn init_all(mut cp: CorePeripherals, mut dp: Peripherals) -> (UART, Clocks, DwtSystick<1_000>) {
pub fn init_all(mut cp: CorePeripherals, mut dp: Peripherals) -> (UART, Clocks, Systick<1_000>) {
    /// Enable GPIOA + GPIOB + GPIOC
    dp.RCC.ahb1enr.modify(|r, w| {
        w.gpioaen()
            .set_bit()
            .gpioben()
            .set_bit()
            .gpiocen()
            .set_bit()
    });

    // init_hse(&mut dp.RCC);

    let clocks = init_clocks(dp.RCC);

    let mut gpioa = dp.GPIOA.split();

    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.sysclk().raw());
    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk().raw());

    let mono = Systick::new(cp.SYST, clocks.sysclk().raw());

    (uart, clocks, mono)
}

fn init_clocks(rcc: RCC) -> Clocks {
    /// default sysclk = 16 MHz
    let mut rcc = rcc.constrain();
    // let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(32.MHz()).freeze();
    let clocks = rcc.cfgr.sysclk(32.MHz()).freeze();
    // let clocks = rcc.cfgr.freeze();

    clocks
}

// /// https://nercury.github.io/rust/embedded/experiments/2019/01/27/rust-embedded-02-measuring-the-clock.html
// fn init_hse(rcc: &mut RCC) {
//     /// Enable HSE
//     rcc.cr.modify(|r, w| w.hseon().set_bit());
//     /// Wait until HSE is ready
//     while rcc.cr.read().hserdy().bit_is_clear() {
//         cortex_m::asm::nop();
//     }
//     const RCC_CFGR2_DIV2: u8 = 0b0001; // bits = divider - 1 (1 = 2 - 1)
//     // rcc.plli2scfgr
// }
