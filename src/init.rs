// use dwt_systick_monotonic::DwtSystick;
use stm32f4::stm32f401::{self, RCC, TIM2};
use stm32f401::{CorePeripherals, Peripherals};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Clocks;

// use crate::time::MonoTimer;
use crate::uart::UART;

use systick_monotonic::{ExtU64, Systick};

// pub fn init_all(mut cp: CorePeripherals, mut dp: Peripherals) -> (UART, Clocks, DwtSystick<1_000>) {
pub fn init_all(mut cp: CorePeripherals, mut dp: Peripherals) -> (UART, Clocks, Systick<1_000>) {
    // pub fn init_all(
    //     mut cp: CorePeripherals,
    //     mut dp: Peripherals,
    // ) -> (UART, Clocks, MonoTimer<TIM2, 1_000>) {
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

    // let (clocks, mono) = init_clocks(dp.TIM2, dp.RCC);
    let clocks = init_clocks(dp.RCC);

    let mut gpioa = dp.GPIOA.split();

    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.sysclk().raw());
    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk().raw());

    let mono = Systick::new(cp.SYST, clocks.sysclk().raw());

    (uart, clocks, mono)
}

fn init_clocks(rcc: RCC) -> Clocks {
    let mut rcc = rcc.constrain();
    // let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(32.MHz()).freeze();
    // let clocks = rcc.cfgr.sysclk(32.MHz()).freeze();
    let clocks = rcc.cfgr.freeze();
    clocks
}

// fn init_clocks(tim2: TIM2, rcc: RCC) -> (Clocks, MonoTimer<TIM2, 1_000>) {
//     /// enable and reset TIM2 for MonoTimer
//     rcc.apb1enr.modify(|r, w| w.tim2en().set_bit());
//     rcc.apb1rstr.modify(|r, w| w.tim2rst().set_bit());
//     rcc.apb1rstr.modify(|r, w| w.tim2rst().clear_bit());
//     /// default sysclk = 16 MHz
//     let mut rcc = rcc.constrain();
//     let clocks = rcc.cfgr.freeze();
//     let mono = MonoTimer::new(tim2, &clocks);
//     (clocks, mono)
// }

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
