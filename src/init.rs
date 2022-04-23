use cortex_m::peripheral::NVIC;
use embedded_hal::spi::MODE_3;
// use dwt_systick_monotonic::DwtSystick;
use stm32f4::stm32f401::{self, EXTI, RCC, SPI1, SPI2, TIM2, TIM5, TIM9};
use stm32f401::{CorePeripherals, Peripherals};
use stm32f4xx_hal::dwt::{Dwt, DwtExt};
use stm32f4xx_hal::gpio::{
    Alternate, Input, Output, Pin, Pull, Speed, PA4, PA5, PA6, PA7, PA8, PB12, PB13,
    PB15, PB2, PC13,
};
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::spi::{Mode, NoMiso};
use stm32f4xx_hal::syscfg::SysCfg;
use stm32f4xx_hal::timer::{CounterMs, DelayMs, SysDelay, Timer};
use stm32f4xx_hal::{gpio::PB0, prelude::*};

use crate::bluetooth::BluetoothSpi;
use crate::sensors::barometer::Barometer;
use crate::sensors::imu::IMU;
use crate::sensors::magneto::Magnetometer;
use crate::sensors::Sensors;
use crate::spi::Spi3;
use crate::time::MonoTimer;
// use crate::time::MonoTimer;
use crate::{bt_control::BTController, uart::UART};

// use systick_monotonic::{ExtU64, Systick};

// pub fn init_all_pre(cp: &mut CorePeripherals, dp: &mut Peripherals) {
pub fn init_all_pre(rcc: &mut RCC) {
    /// Enable GPIOA + GPIOB + GPIOC
    rcc.ahb1enr.modify(|r, w| {
        w.gpioaen()
            .set_bit()
            .gpioben()
            .set_bit()
            .gpiocen()
            .set_bit()
    });

    /// Enable SPI1 clock
    rcc.apb2enr.modify(|r, w| w.spi1en().set_bit());

    /// Enable SPI2 clock
    rcc.apb1enr.write(|w| w.spi2en().set_bit());

    /// Enable SYSCFG clock
    rcc.apb2enr.modify(|r, w| w.syscfgen().set_bit());
}

pub struct InitStruct {
    pub dwt:      Dwt,
    pub uart:     UART,
    pub exti:     EXTI,
    pub tim9:     CounterMs<TIM9>,
    pub clocks:   Clocks,
    // pub mono:     Systick<1_000>,
    pub mono:     MonoTimer<TIM5, 1_000_000>,
    pub sensors:  Sensors,
    // pub sensors:  (SensSpi, Pin<'B', 12, Output>),
    pub bt:       BTController<'static>,
    pub delay_bt: DelayMs<TIM2>,
}

pub fn init_all(
    mut cp: CorePeripherals,
    mut dp: Peripherals,
    bt_buf: &'static mut [u8],
) -> InitStruct {
    init_all_pre(&mut dp.RCC);

    // let (clocks, mono) = init_clocks(dp.TIM2, dp.RCC);
    let clocks = init_clocks(dp.RCC);

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.sysclk().raw());
    // let mono = DwtSystick::<1_000>::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk().raw());

    // let mut bt_delay = cp.SYST.delay(&clocks);
    let bt_delay = dp.TIM2.delay_ms(&clocks);

    // let mono = Systick::new(cp.SYST, clocks.sysclk().raw());

    let mono = MonoTimer::<TIM5, 1_000_000>::new(dp.TIM5, &clocks);

    // let mut tim9 = Timer::new(dp.TIM9, &clocks);
    // tim9.sta
    // tim9.listen(stm32f4xx_hal::timer::Event::Update);

    /// TIM9: periodic sensor polling
    let mut tim9: stm32f4xx_hal::timer::CounterMs<TIM9> = dp.TIM9.counter(&clocks);
    tim9.start(1.secs()).unwrap();
    tim9.listen(stm32f4xx_hal::timer::Event::Update);

    // (uart, clocks, mono)

    let mut syscfg = dp.SYSCFG.constrain();

    let bt_irq = init_bt_interrupt(&mut dp.EXTI, &mut syscfg, gpioa.pa4);

    uart.pause();
    let bt = init_bt(
        dp.SPI1, gpiob.pb0, gpiob.pb2, bt_irq, gpioa.pa5, gpioa.pa6, gpioa.pa7, &clocks,
        bt_buf,
    );
    uart.unpause();

    let mut sensors = init_sensors(
        dp.SPI2, gpiob.pb13, gpiob.pb15, gpioa.pa8, gpiob.pb12, gpioc.pc13, &clocks,
    );

    let dwt = cp.DWT.constrain(cp.DCB, &clocks);

    InitStruct {
        dwt,
        uart,
        exti: dp.EXTI,
        // tim9: dp.TIM9,
        tim9,
        clocks,
        mono,
        sensors,
        bt,
        delay_bt: bt_delay,
    }
}

fn init_sensors(
    spi2: SPI2,
    pb13: PB13,
    pb15: PB15,
    pa8: PA8,
    pb12: PB12,
    pc13: PC13,
    clocks: &Clocks,
) -> Sensors {
    let mut cs_imu = pa8
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(Pull::None);
    cs_imu.set_high();

    let mut cs_mag = pb12
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(Pull::None);
    cs_mag.set_high();

    let mut cs_baro = pc13
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(Pull::None);
    cs_baro.set_high();

    let sck = pb13
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(Pull::None)
        .into_alternate::<5>();
    let mosi = pb15
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(Pull::None)
        .into_alternate::<5>();

    /// Mode3 = IdleHigh, Capture 2nd
    let mode = MODE_3;

    // let mut spi = spi2.spi_bidi((sck, NoMiso {}, mosi), mode, 10.MHz(), &clocks);

    let mut spi = Spi3::new(spi2, mode, sck, mosi);

    let imu = IMU::new(cs_imu);
    let mag = Magnetometer::new(cs_mag);
    let baro = Barometer::new(cs_baro);

    let mut out = Sensors::new(spi, imu, mag, baro);

    out.with_spi_mag(|spi, mag| {
        // mag.init_continuous(spi).unwrap();
        mag.init_single(spi).unwrap();
    });

    out
}

fn init_bt_interrupt(
    exti: &mut EXTI,
    syscfg: &mut SysCfg,
    mut pa4: PA4,
) -> Pin<'A', 4, Input> {
    pa4.make_interrupt_source(syscfg);
    pa4.enable_interrupt(exti);
    pa4.trigger_on_edge(exti, stm32f4xx_hal::gpio::Edge::Rising);
    pa4.clear_interrupt_pending_bit();
    pa4.into_input()
}

fn init_bt(
    spi1: SPI1,
    cs: PB0,
    reset: PB2,
    input: PA4,
    sck: PA5,
    miso: PA6,
    mosi: PA7,
    clocks: &Clocks,
    buf: &'static mut [u8],
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

    // let input = input.into_pull_down_input();

    let sck = sck
        .internal_resistor(stm32f4xx_hal::gpio::Pull::Down)
        .into_push_pull_output()
        .speed(Speed::High)
        .into_alternate::<5>();
    let miso = miso.into_alternate::<5>();
    let mosi = mosi
        .into_push_pull_output()
        .speed(Speed::High)
        .into_alternate::<5>();

    let mode = Mode {
        polarity: stm32f4xx_hal::spi::Polarity::IdleLow,
        phase:    stm32f4xx_hal::spi::Phase::CaptureOnFirstTransition,
    };

    let mut spi = spi1.spi((sck, miso, mosi), mode, 1.MHz(), &clocks);

    // let mut buffer = [0u8; 512];

    // let mut bt: BTController<'_> = BluetoothSpi::new(spi, cs, reset, input, &mut buffer);
    let mut bt: BTController<'_> = BluetoothSpi::new(spi, cs, reset, input, buf);

    bt
    // unimplemented!()
}

fn init_clocks(rcc: RCC) -> Clocks {
    let mut rcc = rcc.constrain();
    // let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(32.MHz()).freeze();
    // let clocks = rcc.cfgr.sysclk(32.MHz()).freeze();

    // /// Works, any change makes UART stop working
    // let clocks = rcc.cfgr.sysclk(16.MHz()).use_hse(16.MHz()).freeze();

    let clocks = rcc
        .cfgr
        //
        .use_hse(16.MHz())
        .sysclk(32.MHz())
        // .require_pll48clk()
        .freeze();

    // /// max speed
    // let clocks = rcc
    //     .cfgr
    //     //
    //     .use_hse(25.MHz())
    //     .sysclk(84.MHz())
    //     // .require_pll48clk()
    //     .freeze();

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
