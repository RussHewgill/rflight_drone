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
pub mod init;
pub mod math;
pub mod pid;
pub mod sensors;
pub mod spi;
pub mod time;
pub mod uart;

use adc::*;
use bluetooth::*;
use bt_control::*;
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

// #[cfg(feature = "nope")]
#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
mod app {

    use cortex_m_semihosting::{debug, hprintln};
    use stm32f4::stm32f401::{self, EXTI, TIM2, TIM5, TIM9};
    // use stm32f4xx_hal::prelude::*;

    use stm32f4xx_hal::dma::StreamsTuple;
    use stm32f4xx_hal::dwt::Dwt;
    use stm32f4xx_hal::gpio::{Output, Pin};
    use stm32f4xx_hal::timer::{CounterHz, CounterMs};
    use stm32f4xx_hal::{block, prelude::*, timer::DelayMs};

    // // use dwt_systick_monotonic::{DwtSystick, ExtU32};
    // use systick_monotonic::{ExtU64, Systick};

    use crate::bluetooth::gatt::Commands as GattCommands;
    use crate::bluetooth::hal_bt::Commands as HalCommands;
    use crate::sensors::ahrs::AHRS;
    use crate::sensors::{Sensors, UQuat};
    use crate::time::MonoTimer;
    use crate::{bluetooth::gap::Commands as GapCommands, bt_control::BTState};
    use bluetooth_hci::host::{uart::Hci as HciUart, Hci};

    use nalgebra as na;

    use crate::{
        bt_control::{BTController, BTEvent},
        init::*,
        math::*,
        uart::*,
        uprint, uprintln,
    };

    #[shared]
    struct Shared {
        dwt:      Dwt,
        uart:     UART,
        exti:     EXTI,
        ahrs:     AHRS,
        bt:       BTController<'static>,
        delay_bt: DelayMs<TIM2>,
    }

    #[local]
    struct Local {
        //
        sensors: Sensors,
        tim9:    CounterHz<TIM9>,
    }

    // #[monotonic(binds = SysTick, default = true)]
    // type MonoTick = DwtSystick<1_000>; // 1000 Hz

    // #[monotonic(binds = SysTick, default = true)]
    // type MonoTick = Systick<1_000>; // 1000 Hz

    #[monotonic(binds = TIM5, default = true)]
    type MonoTick = MonoTimer<TIM5, 1_000_000>; // 1 MHz

    #[init(local = [bt_buf: [u8; 512] = [0u8; 512]])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // debug::exit(debug::EXIT_SUCCESS); // Exit QEMU simulator

        let mut cp: stm32f401::CorePeripherals = cx.core;
        let mut dp: stm32f401::Peripherals = cx.device;

        let bt_buf = cx.local.bt_buf;

        let init_struct = init_all(cp, dp, &mut bt_buf[..]);

        let mut uart = init_struct.uart;
        let clocks = init_struct.clocks;
        let mono = init_struct.mono;
        let mut exti = init_struct.exti;

        // hprintln!("hclk() = {:?}", clocks.hclk());
        // uprintln!(uart, "hclk() = {:?}", clocks.hclk());
        // uprintln!(uart, "sysclk() = {:?}", clocks.sysclk());

        let mut bt = init_struct.bt;
        let mut delay_bt = init_struct.delay_bt;

        // uart.pause();
        // bt.pause_interrupt(&mut exti);
        match bt.init_bt(&mut uart, &mut delay_bt) {
            Ok(()) => {
                test_sens::spawn_after(2.secs()).unwrap();
            }
            _ => unimplemented!(),
        }
        // bt.unpause_interrupt(&mut exti);
        bt.clear_interrupt();

        // uart.unpause();

        bt.unpend();

        let ahrs = AHRS::new(
            // 1.0 / 800.0, // 1.25 ms
            1.0 / 200.0, // 5 ms
            0.5,
        );

        let shared = Shared {
            dwt: init_struct.dwt,
            uart,
            exti,
            ahrs,
            bt,
            delay_bt,
        };

        let local = Local {
            sensors: init_struct.sensors,
            tim9:    init_struct.tim9,
        };

        // setup_bt::spawn().unwrap();

        // test_uart::spawn().unwrap();
        // test_uart::spawn_after(2.secs()).unwrap();

        // test_sens::spawn_after(2.secs()).unwrap();

        // tim9_sensors::spawn().unwrap();

        // (shared, Local {}, init::Monotonics())
        (shared, local, init::Monotonics(mono))
    }

    #[cfg(feature = "nope")]
    // #[task(binds = TIM1_BRK_TIM9, shared = [bt, uart, exti, ahrs], local = [tim9,sensors], priority = 2)]
    fn tim9_sensors(mut cx: tim9_sensors::Context) {
        cx.local
            .tim9
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);
        let sensors: &mut Sensors = cx.local.sensors;

        sensors.read_data_mag();
        sensors.read_data_imu(false);

        // let mut quat = UQuat::default();
        let mut quat: Option<UQuat> = None;
        let q = &mut quat;

        // (cx.shared.uart, cx.shared.ahrs, cx.shared.bt).lock(|uart, ahrs, bt| {
        (cx.shared.uart, cx.shared.ahrs, cx.shared.bt, cx.shared.exti).lock(
            |uart, ahrs, bt, exti| {
                let gyro = sensors.data.imu_gyro.read_and_reset();
                let acc = sensors.data.imu_acc.read_and_reset();
                let mag = sensors.data.magnetometer.read_and_reset();

                if let Some(new_quat) = ahrs.update_uart(uart, gyro, acc, mag) {
                    *q = Some(*new_quat);
                }

                // uprintln!(
                //     uart,
                //     "({:.5},{:.5},{:.5}), ({:.5},{:.5},{:.5})",
                //     gyro.x,
                //     gyro.y,
                //     gyro.z,
                //     acc.x,
                //     acc.y,
                //     acc.z,
                // );

                // let (roll, pitch, yaw) = quat.euler_angles();

                // uprintln!(uart, "roll  = {:.1}", rad_to_deg(roll));
                // uprintln!(uart, "pitch = {:.1}", rad_to_deg(pitch));
                // uprintln!(uart, "yaw   = {:.1}", rad_to_deg(yaw));

                // let xs = sensors.data.magnetometer.read_and_reset();
                // uprintln!(uart, "xs[0] = {:.4}", xs[0]);
                // uprintln!(uart, "xs[1] = {:.4}", xs[1]);
                // uprintln!(uart, "xs[2] = {:.4}", xs[2]);

                let qq = q.clone().unwrap().coords;

                let mut buf = [0; 16];

                buf[0..4].copy_from_slice(&qq[0].to_be_bytes());
                buf[4..8].copy_from_slice(&qq[1].to_be_bytes());
                buf[8..12].copy_from_slice(&qq[2].to_be_bytes());
                buf[12..16].copy_from_slice(&qq[3].to_be_bytes());

                // let buf = [0];
                // let buf = q.to_be_bytes();

                bt.pause_interrupt(exti);
                match bt.log_write(uart, &buf) {
                    Ok(_) => {
                        uprintln!(uart, "sent log write command");
                        // let i = bt.check_interrupt();
                        // uprintln!(uart, "i = {:?}", i);
                    }
                    Err(e) => {
                        uprintln!(uart, "error 0 = {:?}", e);
                    }
                }
                bt.unpause_interrupt(exti);
            },
        );

        // tim9_sensors::spawn_after(1.secs()).unwrap();
    }

    #[task(shared = [uart, exti, bt], local = [x: f32 = 0.0, once: bool = true], priority = 8)]
    fn test_sens(mut cx: test_sens::Context) {
        (cx.shared.uart, cx.shared.bt, cx.shared.exti).lock(|uart, bt, exti| {
            if bt.state.is_connected() {
                uprintln!(uart, "test_sens");

                *cx.local.x += 1.0;

                let qq = na::Quaternion::<f32>::new(*cx.local.x, 2.0, 3.0, 4.0);
                let qq = qq.coords;

                let mut buf = [0; 16];

                buf[0..4].copy_from_slice(&qq[0].to_be_bytes());
                buf[4..8].copy_from_slice(&qq[1].to_be_bytes());
                buf[8..12].copy_from_slice(&qq[2].to_be_bytes());
                buf[12..16].copy_from_slice(&qq[3].to_be_bytes());

                if *cx.local.once {
                    uprintln!(uart, "&buf[0..4] = {:?}", &buf[0..4]);
                    uprintln!(uart, "&buf[4..8] = {:?}", &buf[4..8]);
                    *cx.local.once = false;
                }

                bt.pause_interrupt(exti);
                match bt.log_write(uart, &buf) {
                    Ok(_) => {
                        uprintln!(uart, "sent log write command");
                        // let i = bt.check_interrupt();
                        // uprintln!(uart, "i = {:?}", i);
                    }
                    Err(e) => {
                        uprintln!(uart, "error 0 = {:?}", e);
                    }
                }
                bt.unpause_interrupt(exti);
            } else {
                uprintln!(uart, "not connected yet");
            }
        });

        test_sens::spawn_after(1.secs()).unwrap();
    }

    #[cfg(feature = "nope")]
    // #[task(capacity = 3, shared = [uart, dwt], local = [sensors], priority = 8)]
    fn test_sens(mut cx: test_sens::Context) {
        (cx.shared.uart, cx.shared.dwt).lock(|uart, dwt| {
            uprintln!(uart, "test_sens");
            let sensors: &mut Sensors = &mut cx.local.sensors;

            // sensors.with_spi_mag(|spi, mag| {
            //     mag.init_continuous(spi).unwrap();
            //     while !mag.read_new_data_available(spi).unwrap() {
            //         cortex_m::asm::nop();
            //     }
            // });

            // sensors.with_spi_mag(|spi, mag| {
            //     let t = dwt.measure(|| {
            //         let data = mag.read_data(spi).unwrap();
            //     });
            //     uprintln!(uart, "test_sens done, t = {:?}", t.as_micros());
            // });

            // sensors.with_spi_mag(|spi, mag| {
            //     mag.init_single(spi).unwrap();
            //     while !mag.read_new_data_available(spi).unwrap() {
            //         cortex_m::asm::nop();
            //     }
            // });

            // sensors.with_spi_mag(|spi, mag| {
            //     /// 32 MHz = 39 us
            //     let t = dwt.measure(|| {
            //         let data = mag.read_data(spi).unwrap();
            //     });
            //     // uprintln!(uart, "x = {:?}", data[0]);
            //     // uprintln!(uart, "y = {:?}", data[1]);
            //     // uprintln!(uart, "z = {:?}", data[2]);
            //     uprintln!(uart, "test_sens done, t = {:?}", t.as_micros());
            //     // let b = mag
            //     //     .read_reg(spi, crate::sensors::magneto::MagRegister::WHO_AM_I)
            //     //     .unwrap();
            //     // uprintln!(uart, "b2: {:#010b}", b);
            // });

            // sensors.with_spi_mag(|spi, mag| {
            //     // let streams = StreamsTuple::new(dp.DMA1)
            //     unimplemented!()
            // });

            //
        });
    }

    // #[cfg(feature = "nope")]
    #[task(binds = EXTI4, shared = [bt, uart, exti], priority = 9)]
    fn bt_irq(mut cx: bt_irq::Context) {
        (cx.shared.uart, cx.shared.bt, cx.shared.exti).lock(|uart, bt, exti| {
            uprintln!(uart, "bt_irq");

            // exti.pr.modify(|r, w| w.pr4().set_bit());
            bt.clear_interrupt();
            bt.pause_interrupt(exti);

            let event: BTEvent = match block!(bt._read_event(uart)) {
                Ok(ev) => ev,
                Err(e) => {
                    uprintln!(uart, "read event error = {:?}", e);
                    unimplemented!()
                }
            };

            bt.state.handle_event(uart, event);

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

    #[cfg(feature = "nope")]
    // #[task(capacity = 3, shared = [bt, uart, exti], priority = 8)]
    fn test_uart(mut cx: test_uart::Context) {
        (cx.shared.uart, cx.shared.bt, cx.shared.exti).lock(|uart, bt, exti| {
            uprintln!(uart, "test_uart start");

            // let buf = [1, 2, 3, 4];
            let buf = "asdf";

            // bt.clear_interrupt();
            bt.pause_interrupt(exti);
            // match block!(bt.log_write(uart, buf.as_bytes())) {
            match bt.log_write(uart, buf.as_bytes()) {
                Ok(_) => {
                    uprintln!(uart, "sent log write command");
                    // let i = bt.check_interrupt();
                    // uprintln!(uart, "i = {:?}", i);
                }
                Err(e) => {
                    uprintln!(uart, "error 0 = {:?}", e);
                }
            }
            bt.unpause_interrupt(exti);

            uprintln!(uart, "test_uart done");
            test_uart::spawn_after(1.secs()).unwrap();

            // if !bt.state.is_connected() {
            //     // uprintln!(uart, "not connected yet");
            //     test_uart::spawn_after(1.secs()).unwrap();
            // } else {
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
        phase:    Phase::CaptureOnFirstTransition,
    };

    /// Reset pin config, PB2
    {
        // // dp.GPIOB.moder.modify(|_, w| w.moder2().output());
        // dp.GPIOB.pupdr.modify(|_, w| w.pupdr2().pull_up());
        // dp.GPIOB.otyper.modify(|r, w| w.ot2().push_pull());
        // dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr2().low_speed());
        // /// Added to avoid spurious interrupt from the BlueNRG
        // dp.GPIOB.bsrr.write(|w| w.br2().clear_bit());
    }
    /// SCLK pin config, PA5
    {
        // dp.GPIOA.ospeedr.modify(|_, w| w.ospeedr5().high_speed());
        // dp.GPIOA.pupdr.modify(|r, w| w.pupdr5().pull_down());

        // // XXX: ?
        // dp.GPIOA.otyper.modify(|r, w| w.ot5().push_pull());
        // dp.GPIOA.moder.modify(|r, w| w.moder5().alternate());
        // dp.GPIOA.afrl.modify(|r, w| w.afrl5().af5());
    }
    /// MISO pin config, PA6
    {
        // dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr6().high_speed());
        // dp.GPIOA.pupdr.modify(|r, w| w.pupdr6().floating());

        // // // XXX: ?
        // // dp.GPIOA.otyper.modify(|r, w| w.ot6().push_pull());
        // // dp.GPIOA.moder.modify(|r, w| w.moder6().alternate());
        // // dp.GPIOA.afrl.modify(|r, w| w.afrl6().af5());
    }
    /// MOSI pin config, PA7
    {
        // dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr7().high_speed());
        // dp.GPIOA.pupdr.modify(|r, w| w.pupdr7().floating());

        // // XXX: ?
        // dp.GPIOA.otyper.modify(|r, w| w.ot7().push_pull());
        // dp.GPIOA.moder.modify(|r, w| w.moder7().alternate());
        // dp.GPIOA.afrl.modify(|r, w| w.afrl7().af5());
    }
    /// CS pin config, PB0
    {
        // dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr0().high_speed());
        // dp.GPIOB.pupdr.modify(|r, w| w.pupdr0().pull_up());

        // // XXX: ?
        // dp.GPIOB.moder.modify(|r, w| w.moder0().output());
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
        // dp.GPIOA.ospeedr.modify(|r, w| w.ospeedr4().high_speed());
        // dp.GPIOA.pupdr.modify(|r, w| w.pupdr4().floating());
    }

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    let mut cs = gpiob
        .pb0
        .internal_resistor(stm32f4xx_hal::gpio::Pull::Up)
        .into_push_pull_output()
        .speed(Speed::High);
    cs.set_high();

    let mut reset = gpiob
        .pb2
        .internal_resistor(stm32f4xx_hal::gpio::Pull::Up)
        .into_push_pull_output()
        .speed(Speed::Low);
    // reset.set_low();

    /// Speed is only for inputs
    let input = gpioa.pa4.into_pull_down_input();

    let sck = gpioa
        .pa5
        .internal_resistor(stm32f4xx_hal::gpio::Pull::Down)
        .into_push_pull_output()
        .speed(Speed::High)
        .into_alternate::<5>();
    // let miso = gpioa.pa6.into_push_pull_output().into_alternate::<5>();
    // let mosi = gpioa.pa7.into_push_pull_output().into_alternate::<5>();
    let miso = gpioa.pa6.into_alternate::<5>();
    let mosi = gpioa
        .pa7
        .into_push_pull_output()
        .speed(Speed::High)
        .into_alternate::<5>();

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

    let mut bt: BTController<'_> = BluetoothSpi::new(spi, cs, reset, input, &mut buffer);

    // bt.reset_with_delay(&mut delay, 5u32).unwrap();
    // block!(bt.read_event(&mut uart)).unwrap();

    // // let id: &'static [u8; 12] = unsafe { &*DEVICE_ID_PTR.cast::<[u8; 12]>() };
    // pub fn device_id() -> &'static [u8; 12] {
    //     const DEVICE_ID_PTR: *const u8 = 0x1FF0_7A10 as _;
    //     unsafe { &*DEVICE_ID_PTR.cast::<[u8; 12]>() }
    // }
    // let id = device_id();

    // /// Device CF:74:D9:3D:C6:C3 DRN1120
    // bt.init_bt(&mut uart, &mut delay).unwrap();

    loop {
        // block!(bt.read_event(&mut uart)).unwrap();
    }

    #[cfg(feature = "nope")]
    {
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

        block!(bt.init_gatt()).unwrap();
        block!(bt.read_event(&mut uart)).unwrap();

        let role = crate::bluetooth::gap::Role::PERIPHERAL;
        block!(bt.init_gap(role, false, 7)).unwrap();
        let gap = block!(bt.read_event_gap_init(&mut uart)).unwrap();

        // block!(bt.le_set_random_address()).unwrap();
        // block!(bt.read_event(&mut uart)).unwrap();

        // let stm32_uuid = 0x1FFF7A10;

        // static BLE_NAME: &'static [u8; 7] = b"DRN1120";

        // let ps = crate::bluetooth::gatt::UpdateCharacteristicValueParameters {
        //     service_handle: gap.service_handle,
        //     characteristic_handle: CharacteristicHandle(0),
        //     offset: 0,
        //     value: &BLE_NAME[..],
        // };

        // block!(bt.update_characteristic_value(&ps)).unwrap();
        // block!(bt.read_event(&mut uart)).unwrap();

        block!(bt.set_tx_power_level(hal_bt::PowerLevel::DbmNeg2_1)).unwrap();
        block!(bt.read_event(&mut uart)).unwrap();

        loop {
            block!(bt.read_event(&mut uart)).unwrap();
        }
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
