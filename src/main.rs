#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![no_std]
#![no_main]

pub mod bluetooth;
pub mod pid;
pub mod sensors;
pub mod spi;

use bluetooth::*;
use sensors::barometer::*;
use sensors::imu::*;
use spi::*;

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

use embedded_hal as hal;
use hal::spi::*;
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
fn main_bluetooth() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    /// Reset  = PB2
    /// BLE_CS = PB0
    /// clk    = PA5
    /// MISO   = PA6
    /// MOSI   = PA7
    /// IRQ    = PA4

    /// Enable SPI1 clock
    dp.RCC.apb2enr.write(|w| w.spi1en().set_bit());

    /// Enable GPIOA + GPIOB
    dp.RCC
        .ahb1enr
        .write(|w| w.gpioaen().set_bit().gpioben().set_bit());

    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    /// Reset pin config
    {
        dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr2().low_speed());
        // dp.GPIOB.pupdr.modify(|_, w| w.pupdr2().pull_up());
    }
    /// CS pin config
    {
        dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr0().high_speed());
    }

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    let cs = gpiob.pb0.into_push_pull_output();
    let reset = gpiob.pb2.into_push_pull_output();

    let sck = gpioa.pa5.into_push_pull_output().into_alternate::<5>();
    let miso = gpioa.pa6.into_push_pull_output().into_alternate::<5>();
    let mosi = gpioa.pa7.into_push_pull_output().into_alternate::<5>();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let spi = dp.SPI1.spi((sck, miso, mosi), mode, 8.MHz(), &clocks);

    let mut bt = BluetoothSpi::new(spi, cs, reset);

    loop {}
}

// #[entry]
fn main_adc() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    dp.RCC.apb2enr.write(|w| w.adc1en().set_bit());

    // dp.GPIOB.pupdr.modify(|r, w| w.pupdr8().floating());

    // dp.ADC1.cr2.modify(|r, w| w.adon().set_bit());

    loop {}
}

// #[entry]
fn main_imu_i2c() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    /// scl = PB13
    /// sda = PB15
    // dp.RCC.apb1enr.write(|w| w.i2c1en().set_bit());
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // let i2c = dp.I2C

    let mut gpiob = dp.GPIOB.split();

    // let scl = gpiob.pb13.into_alternate();
    // let sda = gpiob.pb15.into_alternate();

    loop {}
}

// #[entry]
fn main_imu2() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    /// Enable GPIOA + GPIOB + GPIOC
    dp.RCC.ahb1enr.write(|w| {
        w.gpioaen()
            .set_bit()
            .gpioben()
            .set_bit()
            .gpiocen()
            .set_bit()
    });

    /// set PA8 to output
    dp.GPIOA.moder.modify(|r, w| w.moder8().output());
    /// set PA8 to high speed
    dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr8().high_speed());

    /// set PB12 to high speed
    dp.GPIOB.ospeedr.modify(|r, w| w.ospeedr12().high_speed());
    /// set PC13 to high speed
    dp.GPIOC.ospeedr.modify(|r, w| w.ospeedr13().high_speed());

    /// set CS pins to No PUPD
    dp.GPIOA.pupdr.modify(|r, w| w.pupdr8().floating()); // IMU CS
    dp.GPIOB.pupdr.modify(|r, w| w.pupdr12().floating()); // Magno
    dp.GPIOC.pupdr.modify(|r, w| w.pupdr13().floating()); // Baro

    let mut gpioa = dp.GPIOA.split();
    let mut gpioc = dp.GPIOC.split();

    /// set CS pins to No PUPD
    /// Magno is done in Spi3::new
    let mut cs_imu = gpioa.pa8.into_push_pull_output();
    cs_imu.set_high();
    let mut cs_baro = gpioc.pc13.into_push_pull_output();
    cs_baro.set_high();

    let mut spi = Spi3::new(&dp.RCC, dp.GPIOB, dp.SPI2, MODE_3, 10.MHz());

    // let reg = IMURegisters::CTRL3_C.to_addr();
    let reg = 0x12;

    // Sensor_IO_Write(*handle, LSM6DSL_ACC_GYRO_CTRL3_C, &data, 1)

    /// SIM = 1, 3 wire mode
    // let val = 0b0000_0100 | 0b1000;
    let val = 0b0000_0000
        | (1 << 2) // IF_INC = 1
        | (1 << 3); // SIM = 1

    // /// BLE = 1, use data MSB @ lower address
    // let val = val | 0b0010;

    const IMU_SPI_READ: u8 = 0x80; // 0x01 << 7
    const IMU_SPI_WRITE: u8 = 0x00;

    // let mut bytes0 = [(reg << 1) | IMU_SPI_WRITE, val];
    let mut bytes0 = [reg | IMU_SPI_WRITE, val];

    cs_imu.set_low();
    let e = spi.send(&bytes0);
    cs_imu.set_high();

    hprintln!("e: {:?}", e);

    // let reg = IMURegisters::WHO_AM_I.to_addr();

    // let bytes1 = [(reg << 1) | IMU_SPI_READ];
    let bytes1 = [reg | IMU_SPI_READ];
    let mut bytes2 = 0b1111_1111;

    cs_imu.set_low();
    cortex_m::asm::delay(200);
    let e1 = spi.send(&bytes1);

    cortex_m::asm::delay(200);
    cortex_m::asm::dsb();

    let e2 = spi.read(&mut bytes2);
    cortex_m::asm::delay(200);
    cs_imu.set_high();

    hprintln!("e1: {:?}", e1);
    hprintln!("e2: {:?}", e2);

    // match e2 {
    //     Ok(()) => hprintln!("Ok"),
    //     Err(e) => hprintln!("Err"),
    // }

    hprintln!("b2: {:#010b}", bytes2);

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

    /// PA8  = IMU
    /// PB12 = Magno
    /// PC13 = Baro

    /// Enable SPI2 clock
    dp.RCC.apb1enr.write(|w| w.spi2en().set_bit());

    /// Enable GPIOA + GPIOB
    dp.RCC.ahb1enr.write(|w| {
        w.gpioaen()
            .set_bit()
            .gpioben()
            .set_bit()
            .gpiocen()
            .set_bit()
    });

    /// set PA8 to output
    dp.GPIOA.moder.modify(|r, w| w.moder8().output());

    /// set PB13, PB15 to output
    dp.GPIOB
        .moder
        .modify(|r, w| w.moder13().output().moder15().output());

    /// set PA8 to high speed
    dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr8().high_speed());
    /// set PB12, PB13, PB15 to high speed
    dp.GPIOB.ospeedr.modify(|r, w| {
        w.ospeedr12()
            .high_speed()
            .ospeedr13()
            .high_speed()
            .ospeedr15()
            .high_speed()
    });
    /// set PC13 to high speed
    dp.GPIOC.ospeedr.modify(|r, w| w.ospeedr13().high_speed());

    /// set PB13, PB15 to No Pull-up, No Pull-down (No PUPD)
    dp.GPIOB
        .pupdr
        .modify(|r, w| w.pupdr13().floating().pupdr15().floating());

    /// set CS pins to No PUPD
    dp.GPIOA.pupdr.modify(|r, w| w.pupdr8().floating()); // IMU CS
    dp.GPIOB.pupdr.modify(|r, w| w.pupdr12().floating()); // Magno
    dp.GPIOC.pupdr.modify(|r, w| w.pupdr13().floating()); // Baro

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    /// Mode3 = IdleHigh, Capture 2nd
    let mode = MODE_3;

    let pb13 = gpiob.pb13.into_push_pull_output().into_alternate::<5>();
    let pb15 = gpiob.pb15.into_push_pull_output().into_alternate::<5>();

    let sck = pb13;
    let miso = NoMiso {};
    let mosi = pb15;

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    hprintln!("wat 1");

    dp.SPI2.cr1.modify(|_, w| {
        w.bidimode() // bidirectional half duplex mode
            .set_bit()
            .br() // baud rate = 1/16 f_PCLK
            .div16()
            .mstr() // master mode
            .set_bit()
            .ssm() // software slave management
            .set_bit()
            .dff() // 8 bit data frame format
            .eight_bit()
            .lsbfirst() // MSB first
            .clear_bit()
            .crcen() // hardware CRC disabled (?)
            .clear_bit()
    });

    dp.SPI2.cr2.modify(|_, w| {
        w.ssoe()
            .set_bit() // SS output enabled
            .frf()
            .clear_bit() // Motorola frame format (not TI)
    });

    let mut spi = dp.SPI2.spi_bidi((sck, miso, mosi), mode, 10.MHz(), &clocks);

    let mut cs_imu = gpioa.pa8.into_push_pull_output();
    cs_imu.set_high();

    let mut cs_magno = gpiob.pb12.into_push_pull_output();
    cs_magno.set_high();

    let mut cs_baro = gpioc.pc13.into_push_pull_output();
    cs_baro.set_high();

    // let mut imu = IMU::new(spi, cs_imu);

    // let mut baro = Barometer::new(spi, cs_baro);

    /// Init done
    let mut buf = [0u8; 1];

    // imu.read_reg(0x0F, &mut buf);

    // /// 0b0100_0000
    // /// If endianess is wrong, will only set LPF1 Bandwidth selection
    // let val = 0b00000000 | AccelPowerModes::Normal104.to_val();

    let reg = IMURegisters::CTRL3_C.to_addr();

    /// SIM = 1, 3 wire mode
    let val = 0b0000_0100 | 0b1000;

    const IMU_SPI_READ: u8 = 0x01;
    const IMU_SPI_WRITE: u8 = 0x00;

    let mut bytes0 = [(reg << 1) | IMU_SPI_WRITE, val];

    // while spi.is_txe() {}

    /// start tx
    cs_imu.set_low();

    // let e = hal::spi::nb::FullDuplex::write(&mut spi, bytes0[0]);
    // hprintln!("e: {:?}", e);

    // let e = hal::spi::nb::FullDuplex::write(&mut spi, bytes0[1]);
    // hprintln!("e: {:?}", e);

    /// send data
    let e0 = spi.write(&mut bytes0).ok();

    // let e = hal::spi::blocking::Read::read(&mut spi, &mut bytes0);
    // hprintln!("e: {:?}", e);

    // while spi.is_txe() {}

    /// end tx
    cs_imu.set_high();

    hprintln!("e0: {:?}", e0);

    // hprintln!("r0: {:#010b}", bytes0[0]);
    // hprintln!("r1: {:#010b}", bytes0[1]);

    let reg = IMURegisters::WHO_AM_I.to_addr();

    let bytes1 = [(reg << 1) | IMU_SPI_READ];
    let mut bytes2 = [0];

    cs_imu.set_low();

    let e1 = spi.write(&bytes1).ok();

    let e2 = hal::spi::blocking::Read::read(&mut spi, &mut bytes2);

    cs_imu.set_high();

    hprintln!("e1: {:?}", e1);
    hprintln!("e2: {:?}", e2);

    hprintln!("r0: {:#010b}", bytes2[0]);

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
