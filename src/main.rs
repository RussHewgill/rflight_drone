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

use stm32f4::stm32f401;

#[inline(never)]
fn delay(tim9: &stm32f401::tim9::RegisterBlock, ms: u16) {
    unsafe {
        tim9.arr.write(|w| w.arr().bits(ms));
    }

    tim9.cr1.modify(|_, w| w.cen().set_bit());

    while !tim9.sr.read().uif().bit_is_set() {}

    tim9.sr.modify(|_, w| w.uif().clear_bit());
}

#[entry]
fn main() -> ! {
    // hprintln!("Hello, world!, {}", 1);

    // let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut ps = stm32f401::Peripherals::take().unwrap();

    ps.RCC.apb2enr.modify(|_, w| w.tim9en().set_bit());

    ps.TIM9.cr1.write(|w| w.opm().set_bit().cen().clear_bit());
    ps.TIM9.psc.write(|w| w.psc().bits(7_999));

    hprintln!("wat 0");
    delay(&ps.TIM9, 1000);

    hprintln!("wat 1");
    delay(&ps.TIM9, 1000);

    hprintln!("wat 2");

    // cp.DCB.enable_trace();

    // let swo_freq = 8_000_000;
    // cp.TPIU.acpr

    // enable stimulus port 0
    // cp.ITM.ter[0].write(1);

    // iprintln!(&mut cp.ITM.stim[0], "wat");

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

pub fn init() -> ITM {
    let p = cortex_m::Peripherals::take().unwrap();
    p.ITM
}
