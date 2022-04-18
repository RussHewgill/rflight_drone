#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![no_std]
#![no_main]

pub mod adc;
pub mod bluetooth;
pub mod bt_control;
pub mod pid;
pub mod sensors;
pub mod spi;
// pub mod time;
pub mod init;
pub mod uart;

use adc::*;
use bluetooth::*;
use bt_control::*;
use sensors::barometer::*;
use sensors::imu::*;
use sensors::magneto::*;
use sensors::*;
use spi::*;
use uart::*;

use byteorder::ByteOrder;

// pick a panicking behavior
// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

// use cortex_m::asm;
use cortex_m::{iprintln, peripheral::ITM};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use embedded_hal as hal;
use hal::spi::*;
use stm32f4::stm32f401::{self, SPI2};
use stm32f4xx_hal::block;
use stm32f4xx_hal::gpio::Speed;
use stm32f4xx_hal::serial::Serial;
use stm32f4xx_hal::{
    gpio::{Pin, PinExt},
    prelude::*,
    spi::NoMiso,
    time::*,
};

use rtic::app;

// #[cfg(feature = "nope")]
#[app(device = stm32f4xx_hal::pac)]
mod app {
    use cortex_m_semihosting::debug;
    // use embedded_time::{clock, duration::*, rate::*, Instant, Timer};
    use dwt_systick_monotonic::DwtSystick;
    use fugit::{Duration, ExtU32};
    use stm32f4::stm32f401;
    use stm32f4xx_hal::prelude::*;

    #[shared]
    struct Shared {
        //
    }

    #[local]
    struct Local {
        //
    }

    // #[monotonic(binds = SysTick, default = true)]
    // type MonoTick = DwtSystick<1_000>; // 1000 Hz

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // debug::exit(debug::EXIT_SUCCESS); // Exit QEMU simulator

        // let cp: stm32f401::CorePeripherals = cx.core;
        // let dp: stm32f401::Peripherals = cx.device;

        // /// Enable HSE
        // dp.RCC.cr.modify(|r, w| w.hseon().set_bit());

        // /// Wait until HSE is ready
        // while dp.RCC.cr.read().hserdy().bit_is_clear() {
        //     cortex_m::asm::nop();
        // }

        // let mut rcc = dp.RCC.constrain();
        // let clocks = rcc.cfgr.freeze();

        // let mono = MonoTick::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk());

        (Shared {}, Local {}, init::Monotonics())
        // (Shared {}, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        loop {}
    }
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
    dp.RCC.apb2enr.modify(|r, w| w.spi1en().set_bit());

    /// Enable GPIOA + GPIOB
    dp.RCC
        .ahb1enr
        .modify(|r, w| w.gpioaen().set_bit().gpioben().set_bit());

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    /// Reset pin config, PB2
    {
        // dp.GPIOB.moder.modify(|_, w| w.moder2().output());
        dp.GPIOB.pupdr.modify(|_, w| w.pupdr2().pull_up());
        dp.GPIOB.otyper.modify(|r, w| w.ot2().push_pull());
        dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr2().low_speed());
        /// Added to avoid spurious interrupt from the BlueNRG
        dp.GPIOB.bsrr.write(|w| w.br2().clear_bit());
    }
    /// SCLK pin config, PA5
    {
        dp.GPIOA.ospeedr.modify(|_, w| w.ospeedr5().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr5().pull_down());

        // XXX: ?
        dp.GPIOA.otyper.modify(|r, w| w.ot5().push_pull());
        dp.GPIOA.moder.modify(|r, w| w.moder5().alternate());
        dp.GPIOA.afrl.modify(|r, w| w.afrl5().af5());
    }
    /// MISO pin config, PA6
    {
        dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr6().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr6().floating());

        // // XXX: ?
        // dp.GPIOA.otyper.modify(|r, w| w.ot6().push_pull());
        // dp.GPIOA.moder.modify(|r, w| w.moder6().alternate());
        // dp.GPIOA.afrl.modify(|r, w| w.afrl6().af5());
    }
    /// MOSI pin config, PA7
    {
        dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr7().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr7().floating());

        // XXX: ?
        dp.GPIOA.otyper.modify(|r, w| w.ot7().push_pull());
        dp.GPIOA.moder.modify(|r, w| w.moder7().alternate());
        dp.GPIOA.afrl.modify(|r, w| w.afrl7().af5());
    }
    /// CS pin config, PB0
    {
        dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr0().high_speed());
        dp.GPIOB.pupdr.modify(|r, w| w.pupdr0().pull_up());

        // XXX: ?
        dp.GPIOB.moder.modify(|r, w| w.moder0().output());
    }
    /// IRQ, PA4
    #[cfg(feature = "nope")]
    {
        /// Enable SYSCFG clock
        dp.RCC.apb2enr.modify(|r, w| w.syscfgen().set_bit());

        /// Set source for EXTI4 external interrupt to PA4
        unsafe {
            dp.SYSCFG
                .exticr2
                .modify(|r, w| w.bits(r.bits() & !(0b1111)));
        }

        dp.EXTI.imr.modify(|r, w| {
            // w.mr4().
            w
        });
    }
    {
        dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr4().high_speed());
        dp.GPIOA.pupdr.modify(|r, w| w.pupdr4().floating());
    }

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    let mut cs = gpiob.pb0.into_push_pull_output().speed(Speed::High);
    cs.set_high();
    let mut reset = gpiob.pb2.into_push_pull_output().speed(Speed::Low);
    // reset.set_low();

    let input = gpioa.pa4.into_pull_down_input();

    let sck = gpioa.pa5.into_push_pull_output().into_alternate::<5>();
    // let miso = gpioa.pa6.into_push_pull_output().into_alternate::<5>();
    // let mosi = gpioa.pa7.into_push_pull_output().into_alternate::<5>();
    let miso = gpioa.pa6.into_alternate::<5>();
    let mosi = gpioa.pa7.into_push_pull_output().into_alternate::<5>();

    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    let mut spi = dp.SPI1.spi((sck, miso, mosi), mode, 1.MHz(), &clocks);

    // dp.SPI1.cr1.modify(|r, w| {
    //     w.cpha() // clock phase
    //         .bit(mode.phase == Phase::CaptureOnSecondTransition)
    //         .cpol() // clock polarity
    //         .bit(mode.polarity == Polarity::IdleHigh)
    //         .bidimode() // bidirectional half duplex mode
    //         .clear_bit()
    //         .bidioe() // bidi output mode
    //         .clear_bit()
    //         .br() // baud rate = 1/16 f_PCLK
    //         .div16()
    //         .mstr() // master mode enabled
    //         .set_bit()
    //         .ssm() // software slave management
    //         .set_bit()
    //         .ssi()
    //         .set_bit()
    //         .dff() // 8 bit data frame format
    //         .eight_bit()
    //         .lsbfirst() // MSB first
    //         .clear_bit()
    //         .crcen() // hardware CRC disabled (?)
    //         .clear_bit()
    // });
    // dp.SPI1.cr2.modify(|_, w| {
    //     w.ssoe()
    //         .set_bit() // SS output enabled
    //         .frf()
    //         .clear_bit() // Motorola frame format (not TI)
    // });
    // let spi = Spi4::new(dp.SPI1, sck, miso, mosi);

    // let k = reset.get_state();
    // hprintln!("k: {:?}", k);

    uprintln!(uart, "wat 0");

    let mut buffer = [0u8; 512];

    let mut delay = cp.SYST.delay(&clocks);

    let mut bt = BluetoothSpi::new(spi, cs, reset, input, &mut buffer);

    bt.reset_with_delay(&mut delay, 5u32).unwrap();

    uprintln!(uart, "wat 1");

    // uprintln!(uart, "c = {:?}", crate::bluetooth::opcode::GATT_INIT);

    use bluetooth_hci::host::uart::Hci as HciUart;
    use bluetooth_hci::host::Hci;
    use bluetooth_hci::Controller;

    bt.read_event(&mut uart).unwrap();

    // let e = block!(bt.read_local_version_information()).unwrap();
    // // let e = block!(bt.read_local_supported_commands()).unwrap();
    // uprintln!(uart, "e = {:?}", e);

    use crate::bluetooth::gatt::Commands as GattCommands;

    // block!(bt.init_gatt()).unwrap();
    // block!(bt.read_event(&mut uart)).unwrap();

    // block!(bt.le_set_random_address()).unwrap();
    // block!(bt.read_event(&mut uart)).unwrap();

    // static ble_name: &'static [u8; 7] = b"DRN1120";

    // let ps = crate::bluetooth::gatt::UpdateCharacteristicValueParameters {
    //     service_handle,
    //     characteristic_handle,
    //     offset: 0,
    //     value: &ble_name,
    // };

    // block!(bt.update_characteristic_value(ps)).unwrap();
    // block!(bt.read_event(&mut uart).unwrap());

    loop {
        block!(bt.read_event(&mut uart)).unwrap();
    }

    // uprintln!(uart, "bt._data_ready() = {:?}", bt._data_ready().unwrap());

    // let param_len = block!(bt.test1(&mut uart)).unwrap();
    // uprintln!(uart, "bt._data_ready() = {:?}", bt._data_ready().unwrap());

    // for c in buffer.chunks(32) {
    //     for b in c {
    //         uprint!(uart, "{:#04x} ", b);
    //     }
    //     uprintln!(uart, " === ");
    // }

    // loop {}
}

// #[entry]
fn main_uart2() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    /// Enable GPIOA + GPIOB
    dp.RCC
        .ahb1enr
        .modify(|r, w| w.gpioaen().set_bit().gpioben().set_bit());
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut delay = dp.TIM1.delay_ms(&clocks);
    let mut gpioa = dp.GPIOA.split();
    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    use bluetooth_hci::host::HciHeader;

    const HEADER_LEN: usize = 4;
    let mut header = [0; HEADER_LEN];

    let opcode = bluetooth_hci::Opcode::new(0x04, 0x01);
    bluetooth_hci::host::uart::CommandHeader::new(opcode, 0).copy_into_slice(&mut header);

    let x = byteorder::LittleEndian::read_u16(&header[..]);

    for b in header {
        uprint!(uart, "{:#04x} ", b);
    }

    uprintln!(uart, "opcode = {:?}", opcode);
    uprintln!(uart, "x = {:#x}", x);

    // // uprintln!(uart, "c = {:?}", crate::bluetooth::opcode::GATT_INIT);
    // uprintln!(
    //     uart, "c = {:?}", // bluetooth_hci::opcode::READ_LOCAL_VERSION_INFO
    //     c
    // );

    loop {}
}

// #[entry]
fn main_adc2() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    use stm32f4xx_hal::adc::config::*;
    use stm32f4xx_hal::adc::*;

    let gpiob = dp.GPIOB.split();

    let voltage = gpiob.pb1.into_analog();

    let adc_cfg = AdcConfig::default()
        .clock(Clock::Pclk2_div_4)
        .resolution(Resolution::Twelve)
        .align(Align::Right)
        .scan(Scan::Disabled)
        // .external_trigger(TriggerMode::Disabled, Exte)
        .continuous(Continuous::Single);

    let mut adc = Adc::adc1(dp.ADC1, true, adc_cfg);

    let sample = adc.convert(&voltage, SampleTime::Cycles_3);

    let mv = adc.sample_to_millivolts(sample);

    hprintln!("v = {:?}", mv);

    // adc.configure_channel(&voltage, Sequence::One, SampleTime::Cycles_3);
    // adc.enable_temperature_and_vref();

    loop {}
}

// #[entry]
fn main_adc() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    dp.RCC.apb2enr.write(|w| w.adc1en().set_bit());

    /// Enable GPIOB
    dp.RCC.ahb1enr.write(|w| w.gpioben().set_bit());

    dp.GPIOB.pupdr.modify(|r, w| w.pupdr1().floating());

    dp.GPIOB.moder.modify(|r, w| w.moder1().analog());

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // let uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // let pb1 = gpiob.pb1.into_analog();

    dp.ADC1
        .cr1
        .modify(|r, w| w.scan().clear_bit().res().twelve_bit().discen().disabled());

    dp.ADC1.cr2.modify(|r, w| {
        w.align()
            .right()
            .exten()
            .disabled()
            .cont()
            .single()
            .dma()
            .disabled()
            .eocs()
            .each_conversion()
    });

    dp.ADC1.sqr1.modify(|r, w| w.l().bits(0b0000));

    dp.ADC1.smpr2.modify(|r, w| w.smp9().cycles3());

    unsafe {
        dp.ADC1.sqr3.modify(|r, w| w.sq1().bits(9));
    }

    dp.ADC_COMMON.ccr.modify(|r, w| w.adcpre().div4());

    /// enable ADC
    dp.ADC1.cr2.modify(|r, w| w.adon().set_bit());

    /// start single conversion
    dp.ADC1.cr2.modify(|r, w| w.swstart().set_bit());

    let adc_val: u16 = dp.ADC1.dr.read().data().bits();

    /// disable ADC
    dp.ADC1.cr2.modify(|r, w| w.adon().clear_bit());

    let v_ref = 3.3;

    let r_up = 10_000.0;
    let r_down = 20_000.0;

    let d = 2u32.pow(12) as f32;

    let vbat = ((adc_val as f32 * v_ref) / d) * ((r_up + r_down) / r_down);

    hprintln!("v = {:?}", vbat);

    loop {}
}

// #[entry]
#[allow(unreachable_code)]
// #[cfg(feature = "nope")]
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

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // let mut sensors = Sensors::new(spi, imu, magnetometer, barometer)

    // #[cfg(feature = "nope")]
    {
        let mut mag = Magnetometer::new(cs_magno);
        let b = mag.read_reg(&mut spi, MagRegister::WHO_AM_I).unwrap();
        uprintln!(uart, "b2: {:#010b}", b);

        // mag.init_single(&mut spi).unwrap();
        mag.init_continuous(&mut spi).unwrap();

        loop {
            match mag.read_new_data_available(&mut spi) {
                Ok(true) => {
                    let data = mag.read_data(&mut spi).unwrap();
                    uprintln!(uart, "data: {:>6} {:>6} {:>6}", data[0], data[1], data[2]);
                    // uprintln!(uart, "y: {:?}", data[1]);
                    // uprintln!(uart, "z: {:?}", data[2]);
                }
                _ => {
                    // uprintln!(uart, "no data");
                    cortex_m::asm::nop();
                }
            }
        }

        // loop {}
    }

    #[cfg(feature = "nope")]
    {
        let mut imu = IMU::new(cs_imu);

        // imu.write_reg(&mut spi, IMURegister::CTRL3_C, 0x0C).unwrap();
        imu.init(&mut spi).unwrap();

        let b = imu.read_reg(&mut spi, IMURegister::WHO_AM_I).unwrap();
        hprintln!("b2: {:#010b}", b);

        let ready = imu.read_new_data_available(&mut spi).unwrap();
        hprintln!("r: {:?}", ready);

        if ready[1] {
            // let acc = imu.read_accel_data(&mut spi).unwrap();
            // let gyro = imu.read_gyro_data(&mut spi).unwrap();

            let (gyro, acc) = imu.read_data(&mut spi).unwrap();

            imu.power_down(&mut spi).unwrap();

            for (n, g) in gyro.iter().enumerate() {
                hprintln!("g{:?}: {:.4}", n, g);
            }

            for (n, a) in acc.iter().enumerate() {
                hprintln!("a{:?}: {:.4}", n, a);
            }
        } else {
            hprintln!("wat");
        }

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
