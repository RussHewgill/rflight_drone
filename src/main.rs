#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![no_std]
#![no_main]

pub mod adc;
pub mod bluetooth;
pub mod bluetooth2;
pub mod bt_control;
pub mod flight_control;
pub mod init;
pub mod leds;
pub mod math;
pub mod pid;
pub mod sensors;
pub mod spi;
pub mod time;
pub mod uart;
pub mod utils;

use adc::*;
use bluetooth::*;
use bluetooth2::*;
use bt_control::*;
use flight_control::*;
use init::init_all_pre;
use math::*;
use sensors::barometer::*;
use sensors::imu::*;
use sensors::magneto::*;
use sensors::*;
use spi::*;
use stm32f4xx_hal::dwt::DwtExt;
use time::*;
use uart::*;
use utils::*;

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

#[cfg(feature = "nope")]
// #[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]

mod app {

    use cortex_m_semihosting::{debug, hprintln};
    use fugit::MillisDurationU32;
    use stm32f4::stm32f401::{self, EXTI, TIM10, TIM2, TIM3, TIM5, TIM9};

    use stm32f4xx_hal::{
        block,
        dwt::Dwt,
        gpio::{Output, Pin},
        prelude::*,
        timer::{CounterHz, CounterMs, DelayMs},
    };

    use crate::{
        bluetooth::gap::Commands as GapCommands,
        bluetooth::gatt::Commands as GattCommands,
        bluetooth::{events::BlueNRGEvent, hal_bt::Commands as HalCommands},
        bt_control::BTState,
        sensors::ahrs::{FlightData, AHRS},
        sensors::{SensorData, Sensors, UQuat},
        time::MonoTimer,
        utils::*,
    };

    use bluetooth_hci::host::{uart::Hci as HciUart, Hci};

    use nalgebra as na;

    use heapless::spsc::Queue;

    use crate::{
        bt_control::{BTController, BTEvent},
        init::*,
        math::*,
        uart::*,
        uprint, uprintln,
    };

    #[shared]
    struct Shared {
        dwt:         Dwt,
        uart:        UART,
        exti:        EXTI,
        ahrs:        AHRS,
        sens_data:   SensorData,
        flight_data: FlightData,
        // bt_chan:  heapless::spsc::Queue<>
        bt:          BTController<'static>,
        // bt_queue:    heapless::spsc::Queue<bluetooth_hci::Event<BlueNRGEvent>, 10>,
        // delay_bt: DelayMs<TIM2>,
        // delay_bt: TIM2,
        // delay_bt:  CounterMs<TIM2>,
        tim9_flag:   bool,
    }

    #[local]
    struct Local {
        //
        sensors: Sensors,
        tim3:    CounterHz<TIM3>,
        // interval: MillisDurationU32,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MonoTick = MonoTimer<TIM5, 1_000_000>; // 1 MHz

    #[init(local = [bt_buf: [u8; 512] = [0u8; 512]])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // debug::exit(debug::EXIT_SUCCESS); // Exit QEMU simulator

        let mut cp: stm32f401::CorePeripherals = cx.core;
        let mut dp: stm32f401::Peripherals = cx.device;

        let bt_buf = cx.local.bt_buf;

        // let sensor_period: stm32f4xx_hal::time::Hertz = 800.Hz();
        let sensor_period: stm32f4xx_hal::time::Hertz = 200.Hz();
        // let sensor_period: stm32f4xx_hal::time::Hertz = 50.Hz();

        // let main_period: stm32f4xx_hal::time::Hertz = 50.Hz();

        let mut init_struct = init_all(cp, dp, &mut bt_buf[..]);

        let mut uart = init_struct.uart;
        let clocks = init_struct.clocks;
        let mono = init_struct.mono;
        let mut exti = init_struct.exti;
        let mut sensors = init_struct.sensors;
        let mut bt = init_struct.bt;
        let mut dwt = init_struct.dwt;

        // uprintln!(uart, "hclk()     = {:?}", clocks.hclk());
        // uprintln!(uart, "sysclk()   = {:?}", clocks.sysclk());
        // uprintln!(uart, "pclk1()    = {:?}", clocks.pclk1());
        // uprintln!(uart, "pclk2()    = {:?}", clocks.pclk2());
        // uprintln!(uart, "pll48clk() = {:?}", clocks.pll48clk());
        // uprintln!(uart, "i2s_clk()  = {:?}", clocks.i2s_clk());

        // uprintln!(uart, "main_period = {:?}", main_period.to_Hz());
        uprintln!(uart, "sensor_period = {:?}", sensor_period.to_Hz());

        uart.pause();
        bt.pause_interrupt(&mut exti);
        // match bt.init_bt(&mut uart, &mut delay_bt) {
        match bt.init_bt(&mut uart) {
            Ok(()) => {}
            e => {
                uprintln!(uart, "init_bt error = {:?}", e);
            }
        }
        uart.unpause();

        bt.unpause_interrupt(&mut exti);

        loop {
            if !bt.data_ready().unwrap() {
                break;
            }
            uprintln!(uart, "wat 0");
            bt.read_event_uart(&mut uart).unwrap();
        }

        // bt.clear_interrupt();
        // bt.unpend();

        let mut tim3: stm32f4xx_hal::timer::CounterHz<TIM3> =
            init_struct.tim3.counter_hz(&clocks);

        /// start timer
        tim3.start(sensor_period).unwrap();
        tim3.listen(stm32f4xx_hal::timer::Event::Update);

        /// enable sensors and configure settings
        init_sensors(&mut sensors);

        // /// No debug
        // uart.pause();

        // let t = 1.0 / (sensor_period.raw() as f32);
        // uprintln!(uart, "t = {:?}", t);

        let ahrs = AHRS::new(
            // 1.0 / 800.0, // 1.25 ms
            // 1.0 / 200.0, // 5 ms
            1.0 / (sensor_period.raw() as f32),
            0.5,
        );

        // let interval = (((1.0 / main_period.raw() as f32) * 1000.0) as u32).millis();
        // uprintln!(uart, "interval = {:?}", interval);

        let shared = Shared {
            dwt,
            uart,
            exti,
            ahrs,
            sens_data: SensorData::default(),
            flight_data: FlightData::default(),
            bt,
            // bt_queue: Queue::new(),
            // delay_bt,
            tim9_flag: false,
        };

        let local = Local {
            sensors,
            tim3,
            // interval,
        };

        // timer_sensors::spawn_after(100.millis()).unwrap();

        // main_loop::spawn_after(100.millis()).unwrap();

        (shared, local, init::Monotonics(mono))
    }

    #[cfg(feature = "nope")]
    // #[task(
    // binds = TIM3,
    // shared = [ahrs, sens_data, flight_data, tim9_flag],
    // local = [tim3, sensors],
    // priority = 4
    // )]
    fn timer_sensors(mut cx: timer_sensors::Context) {
        cx.local
            .tim3
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);
        (
            cx.shared.ahrs,
            cx.shared.sens_data,
            cx.shared.flight_data,
            cx.shared.tim9_flag,
            // cx.shared.uart,
        )
            // .lock(|ahrs, sd, fd, tim9_flag, uart| {
            .lock(|ahrs, sd, fd, tim9_flag| {
                /// Read sensor data
                cx.local.sensors.read_data_mag(sd);
                cx.local.sensors.read_data_imu(sd, false);

                /// update AHRS
                let gyro = sd.imu_gyro.read_and_reset();
                let acc = sd.imu_acc.read_and_reset();
                let mag = sd.magnetometer.read_and_reset();

                // // print_v3(uart, gyro, 2);
                // // print_v3(uart, acc, 2);
                // print_v3(uart, mag, 3);
                // // uprintln!(uart, "");

                // uprintln!(uart, "d = {:0>4.2}", d);

                ahrs.update(gyro, acc, mag);

                /// update FlightData
                fd.update(&ahrs);

                *tim9_flag = true;
            });
        // timer_sensors::spawn_after(100.millis()).unwrap();
    }

    #[task(
        // binds = TIM3,
        shared = [bt, exti, flight_data, uart, dwt, tim9_flag],
        local = [counter: u32 = 0, buf: [u8; 16] = [0; 16], cnt: (u32,u32) = (0,0)],
        priority = 3
    )]
    fn main_loop(mut cx: main_loop::Context) {
        *cx.local.counter += 1;
        if *cx.local.counter >= 10 {
            *cx.local.counter = 0;

            (
                cx.shared.uart,
                cx.shared.flight_data,
                cx.shared.bt,
                cx.shared.exti,
            )
                .lock(|uart, fd, bt, exti| {
                    // let qq = fd.quat.coords;

                    // cx.local.buf[0..4].copy_from_slice(&qq[0].to_be_bytes());
                    // cx.local.buf[4..8].copy_from_slice(&qq[1].to_be_bytes());
                    // cx.local.buf[8..12].copy_from_slice(&qq[2].to_be_bytes());
                    // cx.local.buf[12..16].copy_from_slice(&qq[3].to_be_bytes());

                    // if bt.data_ready().unwrap() {
                    //     uprintln!(uart, "data ready?");
                    // }

                    // if bt.state.is_connected() {

                    // uprint!(uart, "sending ({:?})...", *cx.local.counter);
                    // bt.clear_interrupt();
                    bt.pause_interrupt(exti);
                    uprint!(uart, "0..");
                    match bt.log_write(uart, true, &cx.local.buf[..]) {
                        Ok(true) => {
                            // uprintln!(uart, "sent log write command");
                            cx.local.cnt.0 += 1;
                        }
                        Ok(false) => {
                            // uprintln!(uart, "failed to write");
                            cx.local.cnt.1 += 1;
                        }
                        Err(e) => {
                            // uprintln!(uart, "error 0 = {:?}", e);
                        }
                    }
                    bt.unpause_interrupt(exti);
                    uprintln!(uart, "1");

                    uprintln!(
                        uart,
                        "{:?}, {:?} = {:.2}",
                        cx.local.cnt.0,
                        cx.local.cnt.1,
                        cx.local.cnt.0 as f32 / (cx.local.cnt.0 + cx.local.cnt.1) as f32,
                    );
                    // //
                    // unimplemented!()
                });
        }
    }

    #[cfg(feature = "nope")]
    // #[task(
    // shared = [bt, exti, flight_data, uart, dwt, tim9_flag],
    // local = [counter: u32 = 0, buf: [u8; 16] = [0; 16]],
    // priority = 3
    // )]
    fn main_loop(mut cx: main_loop::Context) {
        cx.shared.tim9_flag.lock(|tim9_flag| {
            if *tim9_flag {
                *tim9_flag = false;

                *cx.local.counter += 1;
                if *cx.local.counter >= 10 {
                    *cx.local.counter = 0;
                    (
                        cx.shared.uart,
                        cx.shared.flight_data,
                        cx.shared.bt,
                        cx.shared.exti,
                    )
                        .lock(|uart, fd, bt, exti| {
                            let qq = fd.quat.coords;

                            cx.local.buf[0..4].copy_from_slice(&qq[0].to_be_bytes());
                            cx.local.buf[4..8].copy_from_slice(&qq[1].to_be_bytes());
                            cx.local.buf[8..12].copy_from_slice(&qq[2].to_be_bytes());
                            cx.local.buf[12..16].copy_from_slice(&qq[3].to_be_bytes());

                            // uprint!(uart, "sending ({:?})...", *cx.local.counter);
                            // bt.clear_interrupt();
                            bt.pause_interrupt(exti);
                            uprint!(uart, "0..");
                            match bt.log_write(uart, false, &cx.local.buf[..]) {
                                Ok(true) => {
                                    // uprintln!(uart, "sent log write command");
                                }
                                Ok(false) => {
                                    // uprintln!(uart, "failed to write");
                                }
                                Err(e) => {
                                    // uprintln!(uart, "error 0 = {:?}", e);
                                }
                            }
                            bt.unpause_interrupt(exti);
                            uprintln!(uart, "1");

                            // //
                            // unimplemented!()
                        });
                }
            }
        });

        // main_loop::spawn_after(100.millis()).unwrap();
        main_loop::spawn().unwrap();
    }

    // #[cfg(feature = "nope")]
    #[task(binds = EXTI4, shared = [bt, uart, exti], priority = 8)]
    fn bt_irq(mut cx: bt_irq::Context) {
        // cortex_m_semihosting::hprint!("b");
        (cx.shared.uart, cx.shared.bt, cx.shared.exti).lock(|uart, bt, exti| {
            uprintln!(uart, "bt_irq");

            bt.clear_interrupt();
            bt.pause_interrupt(exti);

            let event: BTEvent = match bt._read_event(uart) {
                Ok(ev) => ev,
                Err(e) => {
                    uprintln!(uart, "read event error = {:?}", e);
                    unimplemented!()
                }
            };
            bt.state.handle_event(uart, event);

            // if !bt.data_ready().unwrap() {
            //     break;
            // }

            bt.unpause_interrupt(exti);

            // match block!(bt.read_event(uart)) {
            //     Ok(_) => {
            //         uprintln!(uart, "read event");
            //     }
            //     Err(e) => {
            //         uprintln!(uart, "error 1 = {:?}", e);
            //     }
            // }
        });
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
}

// #[cfg(feature = "nope")]
#[entry]
fn main_bluetooth() -> ! {
    use stm32f4xx_hal::gpio::Pull;

    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    /// Enable SPI1 clock
    dp.RCC.apb2enr.modify(|r, w| w.spi1en().set_bit());

    /// Enable GPIOA + GPIOB
    dp.RCC
        .ahb1enr
        .modify(|r, w| w.gpioaen().set_bit().gpioben().set_bit());

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    let cs = gpiob.pb0;
    let reset = gpiob.pb2;
    let input = gpioa.pa4;
    let sck = gpioa.pa5;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;

    let mut cs = cs
        .internal_resistor(Pull::Up)
        .into_push_pull_output()
        .speed(Speed::High);
    cs.set_high();

    let mut reset = reset
        .internal_resistor(Pull::Up)
        .into_push_pull_output()
        .speed(Speed::Low);
    reset.set_high();

    // let input = input.into_pull_down_input();

    let sck = sck
        .internal_resistor(Pull::Down)
        .into_push_pull_output()
        .speed(Speed::High)
        .into_alternate::<5>();
    // let miso = miso.into_alternate::<5>();
    let miso = miso
        .internal_resistor(Pull::None)
        // .internal_resistor(Pull::Down) // XXX: nope
        .into_push_pull_output()
        .into_alternate::<5>()
        .speed(Speed::High);
    let mosi = mosi
        .internal_resistor(Pull::None)
        .into_push_pull_output()
        .speed(Speed::High)
        .into_alternate::<5>();

    let mode = Mode {
        polarity: stm32f4xx_hal::spi::Polarity::IdleLow,
        phase:    stm32f4xx_hal::spi::Phase::CaptureOnFirstTransition,
    };

    loop {}
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

    let mut gpiob = dp.GPIOB.split();

    let mut led1_pin = gpiob
        .pb5
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(stm32f4xx_hal::gpio::Pull::None);
    let mut led2_pin = gpiob
        .pb4
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(stm32f4xx_hal::gpio::Pull::None);

    led1_pin.set_high();
    led2_pin.set_high();

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
fn main_imu3() -> ! {
    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    init_all_pre(&mut dp.RCC);

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // let mut sensors = init_sensors(
    //     dp.SPI2, gpiob.pb13, gpiob.pb15, gpioa.pa8, gpiob.pb12, gpioc.pc13, &clocks,
    // );

    // sensors.with_spi_mag(|spi, mag| {
    //     //
    //     unimplemented!()
    // });

    loop {}
}

// #[entry]
#[allow(unreachable_code)]
#[cfg(feature = "nope")]
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

    let (mut cs_magno, mut spi) =
        Spi3::new_full_config(&dp.RCC, dp.GPIOB, dp.SPI2, mode, 10.MHz());

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
