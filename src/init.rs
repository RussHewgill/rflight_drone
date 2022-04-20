// use dwt_systick_monotonic::DwtSystick;
use stm32f4::stm32f401::{self, RCC, TIM2};
use stm32f401::{CorePeripherals, Peripherals};
use stm32f4xx_hal::gpio::{Pull, Speed, PA4, PA5, PA6, PA7, PB2};
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::{gpio::PB0, prelude::*};

// use crate::time::MonoTimer;
use crate::{bt_control::BTController, uart::UART};

use systick_monotonic::{ExtU64, Systick};

pub fn init_all_pre(cp: &mut CorePeripherals, dp: &mut Peripherals) {
    /// Enable GPIOA + GPIOB + GPIOC
    dp.RCC.ahb1enr.modify(|r, w| {
        w.gpioaen()
            .set_bit()
            .gpioben()
            .set_bit()
            .gpiocen()
            .set_bit()
    });

    /// Enable SPI1 clock
    dp.RCC.apb2enr.modify(|r, w| w.spi1en().set_bit());

    /// Enable SPI2 clock
    dp.RCC.apb1enr.write(|w| w.spi2en().set_bit());
}

pub struct InitStruct {
    pub uart: UART,
    pub clocks: Clocks,
    pub mono: Systick<1_000>,
    pub bt: BTController<'static>,
}

pub fn init_all(mut cp: CorePeripherals, mut dp: Peripherals) -> InitStruct {
    // let (clocks, mono) = init_clocks(dp.TIM2, dp.RCC);
    let clocks = init_clocks(dp.RCC);

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.sysclk().raw());
    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk().raw());

    let mono = Systick::new(cp.SYST, clocks.sysclk().raw());

    // (uart, clocks, mono)

    let bt = init_bt(
        gpiob.pb0, gpiob.pb2, gpioa.pa4, gpioa.pa5, gpioa.pa6, gpioa.pa7,
    );

    InitStruct {
        uart,
        clocks,
        mono,
        bt,
    }
}

pub fn init_bt(
    cs: PB0,
    reset: PB2,
    input: PA4,
    sck: PA5,
    miso: PA6,
    mosi: PA7,
) -> BTController<'static> {
    let mut cs = cs
        .internal_resistor(Pull::Up)
        .into_push_pull_output()
        .speed(Speed::High);
    cs.set_high();

    let reset = reset
        .internal_resistor(stm32f4xx_hal::gpio::Pull::Up)
        .into_push_pull_output()
        .speed(Speed::Low);

    unimplemented!()
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
