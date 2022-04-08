#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![no_std]
#![no_main]

pub mod pid;
pub mod sensors;

use sensors::imu::*;

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
// use embedded_time::rate::{Kilohertz, Megahertz};
use stm32f4xx_hal::{
    gpio::{Pin, PinExt},
    prelude::*,
    spi::NoMiso,
    time::*,
};

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

// #[entry]
fn main_led() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    // #define LED1_PIN                         GPIO_PIN_5
    // #define LED1_GPIO_PORT                   GPIOB
    // #define LED1_GPIO_CLK_ENABLE()           __GPIOB_CLK_ENABLE()
    // #define LED1_GPIO_CLK_DISABLE()          __GPIOB_CLK_DISABLE()

    // dp.RCC
    //     .ahb1enr
    //     .write(|w| w.gpioben().set_bit());

    // let mut gpiob = dp.GPIOB.split();

    // dp.GPIOB.moder

    loop {}
}

#[entry]
fn main_adc() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    loop {}
}

// #[entry]
fn main_imu() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    // #define STEVAL_FCU001_V1_SENSORS_SPI               SPI2
    // #define STEVAL_FCU001_V1_SENSORS_SPI_MOSI_Pin      GPIO_PIN_15 // pb15
    // #define STEVAL_FCU001_V1_SENSORS_SPI_SCK_Pin       GPIO_PIN_13 // pb13
    // #define STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port	      GPIOA
    // #define STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin     	  GPIO_PIN_8  // pa8

    hprintln!("wat 0");

    /// Enable SPI2 clock
    dp.RCC.apb1enr.write(|w| w.spi2en().set_bit());

    /// Enable GPIOA + GPIOB
    dp.RCC.ahb1enr.write(|w| {
        w.gpioben()
            .set_bit()
            .gpioaen()
            .set_bit()
            .gpiocen()
            .set_bit()
    });

    dp.GPIOA.moder.modify(|r, w| w.moder8().output());
    dp.GPIOB
        .moder
        .modify(|r, w| w.moder13().output().moder14().output().moder15().output());

    dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr8().high_speed());
    dp.GPIOB
        .ospeedr
        .modify(|r, w| w.ospeedr13().high_speed().ospeedr15().high_speed());
    dp.GPIOC.ospeedr.modify(|r, w| w.ospeedr13().high_speed());

    dp.GPIOB
        .pupdr
        .modify(|r, w| w.pupdr13().floating().pupdr15().floating());

    dp.GPIOA.pupdr.modify(|r, w| w.pupdr8().floating()); // IMU CS
    dp.GPIOB.pupdr.modify(|r, w| w.pupdr12().floating()); // Magno
    dp.GPIOC.pupdr.modify(|r, w| w.pupdr13().floating()); // Baro

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    // let mode = Mode {
    //     polarity: Polarity::IdleHigh,
    //     phase: Phase::CaptureOnFirstTransition,
    // };
    let mode = MODE_3;

    let pb13 = gpiob.pb13.into_push_pull_output().into_alternate::<5>();
    let pb15 = gpiob.pb15.into_push_pull_output().into_alternate::<5>();

    let sck = pb13;

    // let miso = gpiob.pb14.into_alternate();
    let miso = NoMiso {};

    let mosi = pb15;

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    hprintln!("wat 1");

    dp.SPI2.cr1.modify(|_, w| {
        w.bidimode()
            .set_bit()
            .br()
            .div16()
            .mstr()
            .set_bit()
            .ssm()
            .set_bit()
            .dff()
            .eight_bit()
            .lsbfirst()
            .clear_bit()
            .crcen()
            .clear_bit()
    });

    dp.SPI2
        .cr2
        .modify(|_, w| w.ssoe().set_bit().frf().clear_bit());

    // let spi = dp.SPI2.spi((sck, miso, mosi), mode, 10.MHz(), &clocks);
    let spi = dp.SPI2.spi_bidi((sck, miso, mosi), mode, 10.MHz(), &clocks);

    let mut cs_imu = gpioa.pa8.into_push_pull_output();
    cs_imu.set_high();

    let mut cs_magno = gpiob.pb12.into_push_pull_output();
    cs_magno.set_high();

    let mut cs_baro = gpioc.pc13.into_push_pull_output();
    cs_baro.set_high();

    let mut imu = IMU::new(spi, cs_imu);

    /// Init done
    let mut buf = [0u8; 1];

    // imu.read_reg(0x0F, &mut buf);

    hprintln!("result: {:#010b}", buf[0]);

    loop {}
}

// #[entry]
fn main() -> ! {
    // let mut cp = stm32f401::CorePeripherals::take().unwrap();
    // let mut dp = stm32f401::Peripherals::take().unwrap();

    hprintln!("Hello, world!, {}", 1);

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
