#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_doc_comments)]
#![no_std]
#![no_main]

// pick a panicking behavior
// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

// use cortex_m::asm;
use cortex_m::{iprintln, peripheral::ITM};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use stm32f4::stm32f401::{self, SPI2};

use embedded_hal::spi::*;
use embedded_time::rate::{Kilohertz, Megahertz};
use stm32f4xx_hal::{gpio::NoPin, prelude::*};

#[inline(never)]
fn delay(tim9: &stm32f401::tim9::RegisterBlock, ms: u16) {
    unsafe {
        tim9.arr.write(|w| w.arr().bits(ms));
    }

    tim9.cr1.modify(|_, w| w.cen().set_bit());

    while !tim9.sr.read().uif().bit_is_set() {}

    tim9.sr.modify(|_, w| w.uif().clear_bit());
}

fn enable_bt(spi2: &SPI2) {
    unimplemented!()
}

#[entry]
fn main_imu() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    // // LED
    // dp.RCC.ahb1enr.write(|w| w.gpioben())
    // let mut gpiob = dp.GPIOB.split();
    // let mut pb5 = gpiob.pb5;

    /// Enable SPI1
    dp.RCC.apb2enr.write(|w| w.spi1en().set_bit());

    loop {}
}

// #[entry]
fn main() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    // hprintln!("Hello, world!, {}", 1);

    // let spi1 = ps.SPI1

    // let gpioa = dp.GPIOA.split();

    // let mut rcc = dp.RCC.constrain();
    // let clocks = rcc.cfgr.freeze();

    // let spi1 = Spi::new(ps.SPI2, ());
    // let spi = ps.SPI1.spi(
    //     // (NoPin, NoPin, NoPin),
    // );

    // let spi = dp.SPI1.spi(pins, mode, Megahertz(10), clocks);

    loop {}
}

// #[entry]
fn main2() -> ! {
    // hprintln!("Hello, world!, {}", 1);

    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut ps = stm32f401::Peripherals::take().unwrap();

    // ps.RCC.apb2enr.modify(|_, w| w.tim9en().set_bit());
    // ps.TIM9.cr1.write(|w| w.opm().set_bit().cen().clear_bit());
    // ps.TIM9.psc.write(|w| w.psc().bits(7_999));
    //
    // hprintln!("wat 0");
    // delay(&ps.TIM9, 1000);
    // hprintln!("wat 1");
    // delay(&ps.TIM9, 1000);
    // hprintln!("wat 2");

    // enable TPIU and ITM
    cp.DCB.enable_trace();

    // prescaler
    let swo_freq = 2_000_000;
    unsafe {
        // cp.TPIU.acpr.write((stm32f4xx_hal::rcc::Clocks::sysclk().0 / swo_freq) - 1);

        // SWO NRZ
        cp.TPIU.sppr.write(2);

        cp.TPIU.ffcr.modify(|r| r | (1 << 1));
    }

    // SWO NRZ
    ps.DBGMCU.cr.modify(|_, w| w.trace_ioen().set_bit());

    unsafe {
        cp.ITM.lar.write(0xC5ACCE55);

        cp.ITM.tcr.write(
            (0b000001 << 16) // TraceBusID
            | (1 << 3) // enable SWO output ??
            | (1 << 0), // Enable ITM
        );

        // enable stimulus port 0
        cp.ITM.ter[0].write(1);
    }

    iprintln!(&mut cp.ITM.stim[0], "wat");

    // let mut itm = init();
    // iprintln!(&mut itm.stim[0], "wat");

    // // exit QEMU
    // // NOTE do not run this on hardware; it can corrupt OpenOCD state
    // debug::exit(debug::EXIT_SUCCESS);

    loop {}

    // asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

    // loop {
    //     // your code goes here
    // }
}

// pub fn init() -> ITM {
//     let p = cortex_m::Peripherals::take().unwrap();
//     p.ITM
// }
