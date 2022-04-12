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

use core::cell::RefCell;

use bluetooth::*;
use sensors::barometer::*;
use sensors::imu::*;
use sensors::magneto::*;
use sensors::*;
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

use crate::sensors::magneto::Magnetometer;

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

// #[entry]
fn main_test() -> ! {
    // let gpio_mode = 0x00000003;
    // let exti_mode = 0x10000000;
    // let gpio_mode_it = 0x00010000;
    // let gpio_mode_evt = 0x00020000;
    // let rising_edge = 0x00100000;
    // let falling_edge = 0x00200000;
    // let gpio_output_type = 0x00000010;

    // let gpio_mode_it_rising = 0x10110000;

    // if (gpio_mode_it_rising & exti_mode) == exti_mode {
    //     unimplemented!()
    // }

    let scale = 2;

    // let x_h: u8 = 0x16;
    // let x_l: u8 = 0x69;

    // let x_h: u8 = 0x40;
    // let x_l: u8 = 0x09;

    // let x_h: u8 = 0xff;
    // let x_l: u8 = 0x1d;

    let x_h: u8 = 0xE9;
    let x_l: u8 = 0x97;

    // let mut vs = [0i16; 2];

    // let v0 = x_l as u16;
    // let v0 = v0 | ((x_h as u16) << 8);
    // let v1 = v0 as i16;

    let v1 = ((x_h as i16) << 8) | x_l as i16;

    // let v2 = (v1 as f32) / (scale as f32 * 2.0);

    let v2 = ((v1 as f32) / (i16::MAX as f32)) * scale as f32;

    // vs[0] |= ((x_h as i16) << 8) | x_l as i16;
    // vs[1] |= ((y_h as i16) << 8) | y_l as i16;

    // hprintln!("v0: {:?}", v0);
    hprintln!("v1: {:?}", v1);
    hprintln!("v2: {:?}", v2);

    loop {}
}

// #[entry]
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

    /// Reset pin config, PB2
    {
        // dp.GPIOB.moder.modify(|_, w| w.moder2().output());
        dp.GPIOB.pupdr.modify(|_, w| w.pupdr2().pull_up());
        dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr2().low_speed());
        /// Added to avoid spurious interrupt from the BlueNRG
        dp.GPIOB.bsrr.write(|w| w.br2().set_bit());
    }
    /// SCLK pin config, PA5
    {
        dp.GPIOA.ospeedr.modify(|_, w| w.ospeedr5().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr5().pull_down());
        // dp.GPIOA.moder.modify(|r, w| w.moder5().alternate());
        // dp.GPIOA.afrl.modify(|r, w| w.afrl5().af5());
    }
    /// MISO pin config, PA6
    {
        dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr6().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr6().floating());
    }
    /// MOSI pin config, PA7
    {
        dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr7().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr7().floating());
    }
    /// CS pin config, PB0
    {
        dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr0().high_speed());
        dp.GPIOB.pupdr.modify(|r, w| w.pupdr0().pull_up());
    }
    /// IRQ, PA4
    {
        dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr4().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr4().floating());
    }

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    let mut cs = gpiob.pb0.into_push_pull_output();
    cs.set_high();
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

#[entry]
#[allow(unreachable_code)]
fn main_imu2() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    dp.GPIOA.odr.modify(|_, w| w.odr8().set_bit());
    dp.GPIOB.odr.modify(|_, w| w.odr12().set_bit());
    dp.GPIOC.odr.modify(|_, w| w.odr13().set_bit());

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

    let mode = MODE_3;

    let (mut cs_magno, mut spi) = Spi3::new(&dp.RCC, dp.GPIOB, dp.SPI2, mode, 10.MHz());

    {
        let mut mag = Magnetometer::new(cs_magno);
        let b = mag.read_reg(&mut spi, MagRegister::WHO_AM_I);
        hprintln!("b2: {:#010b}", b.unwrap());

        // let mut bs = [0u8; 2];

        // mag.read_
    }

    loop {}

    #[cfg(feature = "nope")]
    {
        let mut imu = IMU::new(cs_imu);

        // imu.write_reg(&mut spi, IMURegister::CTRL3_C, 0x0C).unwrap();
        imu.init(&mut spi).unwrap();

        let b = imu.read_reg(&mut spi, IMURegister::WHO_AM_I).unwrap();
        hprintln!("b2: {:#010b}", b);

        let ready = imu.read_new_data_available(&mut spi).unwrap();
        hprintln!("r: {:?}", ready);

        let acc = imu.read_accel_data(&mut spi).unwrap();
        let gyro = imu.read_gyro_data(&mut spi).unwrap();

        imu.power_down(&mut spi).unwrap();

        loop {}
    }

    #[cfg(feature = "nope")]
    {
        let mag = Magnetometer::new(cs_magno);

        let baro = Barometer::new(cs_baro);

        let imu = IMU::new(cs_imu);

        let mut sensors = Sensors::new(spi, imu, mag, baro);

        {
            let (spi, mag) = sensors.get_mag();
            let b = mag.read_reg(spi, MagRegister::WHO_AM_I).unwrap();
            hprintln!("b0: {:#010b}", b);
        }

        {
            // let (spi, baro) = sensors.get_baro();
            // let b = mag.read_reg(spi, MagRegister::WHO_AM_I).unwrap();
            // hprintln!("b1: {:#010b}", b);
        }

        {
            let (spi, imu) = sensors.get_imu();
            // let b = imu.read_reg(spi, IMURegisters::WHO_AM_I).unwrap();
            // hprintln!("b2: {:#010b}", b);
        }

        loop {}
    }

    #[cfg(feature = "nope")]
    {
        // let spi = RefCell::new(spi);

        let mut mag = Magnetometer::new(spi, cs_magno);

        let b = mag.read_reg(MagRegister::WHO_AM_I);
        hprintln!("b2: {:#010b}", b.unwrap());

        loop {}

        mag.init().unwrap();

        // let mut rcc = dp.RCC.constrain();
        // let clocks = rcc.cfgr.freeze();
        // let mut delay = cp.SYST.delay(&clocks);

        // mag.reset().unwrap();
        // delay.delay_ms(10u32);

        // mag.init().unwrap();

        mag.read_new_data_available().unwrap();
        let temp = mag.read_temp().unwrap();

        let data = mag.read_data().unwrap();
        hprintln!("x: {:?}", data[0]);
        hprintln!("y: {:?}", data[1]);
        hprintln!("z: {:?}", data[2]);

        // let b = mag.read_reg(MagRegister::WHO_AM_I).unwrap();

        // let reg = 0x4F;
        // let addr = reg | 0x80;
        // let mut b = 0u8;
        // cs_magno.set_low();
        // let e1 = spi.send(addr);
        // let e2 = spi.read(&mut b);
        // cs_magno.set_high();
        // hprintln!("b: {:#010b}", b);

        // loop {
        //     if mag.read_new_data_available().unwrap() {
        //         let data = mag.read_data().unwrap();
        //         hprintln!("x: {:?}", data[0]);
        //         hprintln!("y: {:?}", data[1]);
        //         hprintln!("z: {:?}", data[2]);
        //     }
        // }

        loop {}
    }

    #[cfg(feature = "nope")]
    {
        let reg = 0x4F; // WHO_AM_I

        const SPI_READ: u8 = 0x80; // 0x01 << 7
        const SPI_WRITE: u8 = 0x00;

        let addr = reg | SPI_READ;

        // let bytes1 = [addr];
        let bytes1 = addr;
        let mut bytes2 = 0b1111_1111;

        cs_magno.set_low();
        // cortex_m::asm::delay(200);
        let e1 = spi.send(bytes1);

        spi.enable(false);
        spi.set_bidi_input();
        spi.enable(true);

        // cortex_m::asm::delay(200);
        let e2 = spi.read(&mut bytes2);
        // cortex_m::asm::delay(200);
        cs_magno.set_high();

        spi.enable(false);
        spi.set_bidi_output();
        spi.enable(true);

        // hprintln!("e1: {:?}", e1);
        // hprintln!("e2: {:?}", e2);

        hprintln!("b2: {:#010b}", bytes2);

        // if bytes2 == 0b0100_0000 {
        //     hprintln!("wat 0");
        // } else {
        //     hprintln!("wat 1");
        // }

        let mut mag = Magnetometer::new(&mut spi, cs_magno);

        loop {}
    }

    #[cfg(feature = "nope")]
    {
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
        let e0 = spi.send(bytes0[0]);
        let e1 = spi.send(bytes0[1]);
        cs_imu.set_high();

        // hprintln!("e0: {:?}", e0);
        // hprintln!("e1: {:?}", e1);

        let reg = IMURegister::WHO_AM_I.to_addr();

        // let bytes1 = [(reg << 1) | IMU_SPI_READ];
        let bytes1 = [reg | IMU_SPI_READ];
        let mut bytes2 = 0b1111_1111;

        cs_imu.set_low();
        cortex_m::asm::delay(200);
        let e1 = spi.send(bytes1[0]);

        cortex_m::asm::delay(200);
        // cortex_m::asm::dsb();

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
}

// #[entry]
#[cfg(feature = "nope")]
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

    let reg = IMURegister::CTRL3_C.to_addr();

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

    let reg = IMURegister::WHO_AM_I.to_addr();

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
#[cfg(feature = "nope")]
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
#[cfg(feature = "nope")]
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
