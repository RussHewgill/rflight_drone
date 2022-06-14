use cortex_m::peripheral::NVIC;
use embedded_hal::spi::MODE_3;
// use dwt_systick_monotonic::DwtSystick;
use stm32f4::stm32f401::{
    self, ADC1, EXTI, RCC, SPI1, SPI2, TIM10, TIM2, TIM3, TIM4, TIM5, TIM9,
};
use stm32f401::{CorePeripherals, Peripherals};
use stm32f4xx_hal::adc::Adc;
use stm32f4xx_hal::dwt::{Dwt, DwtExt};
use stm32f4xx_hal::gpio::{
    Alternate, Input, Output, Pin, Pull, Speed, PA4, PA5, PA6, PA7, PA8, PB1, PB12, PB13,
    PB15, PB2, PB4, PB5, PC13,
};
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::spi::{Mode, NoMiso};
use stm32f4xx_hal::syscfg::SysCfg;
use stm32f4xx_hal::timer::{
    CounterHz, CounterMs, CounterUs, DelayMs, FTimerMs, SysDelay, Timer,
};
use stm32f4xx_hal::{gpio::PB0, prelude::*};

use crate::battery::BatteryAdc;
use crate::bluetooth::BluetoothSpi;
use crate::flight_control::ControlInputs;
use crate::leds::LEDs;
use crate::motors::MotorsPWM;
use crate::sensors::barometer::Barometer;
use crate::sensors::imu::config::ImuConfig;
use crate::sensors::imu::IMU;
use crate::sensors::magneto::Magnetometer;
use crate::sensors::Sensors;
use crate::spi::Spi3;
use crate::time::MonoTimer;
// use crate::time::MonoTimer;
use crate::bt_control::BTController;

/// Timers:
/// 2: BT Delay
/// 3: PID interrupt
/// 4: Motor PWM
/// 5: Main monotonic
/// 9: Main Loop interrupt
/// 10: Sensors interrupt

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
    /// peripherals
    pub dwt:     Dwt,
    pub exti:    EXTI,
    /// clocks and timers
    pub clocks:  Clocks,
    pub tim3:    TIM3,
    pub tim9:    TIM9,
    pub tim10:   TIM10,
    pub mono:    MonoTimer<TIM5, 1_000_000>,
    /// wrappers
    pub sensors: Sensors,
    pub bt:      BTController,
    pub adc:     BatteryAdc,
    pub leds:    LEDs,
    pub motors:  MotorsPWM,
}

pub fn init_all(mut cp: CorePeripherals, mut dp: Peripherals) -> InitStruct {
    init_all_pre(&mut dp.RCC);

    // let (clocks, mono) = init_clocks(dp.TIM2, dp.RCC);
    let clocks = init_clocks(dp.RCC);

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    // let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    let mono = MonoTimer::<TIM5, 1_000_000>::new(dp.TIM5, &clocks);

    let mut syscfg = dp.SYSCFG.constrain();

    // // let _rtc = hal::rtc::Rtc::rtc(dp.RTC, &mut rcc);
    // let _rtc = stm32f4xx_hal::rtc::Rtc::new(dp.RTC, &mut dp.PWR);

    let bt_delay = dp.TIM2.counter_us(&clocks);

    let bt_irq = init_bt_interrupt(&mut dp.EXTI, &mut syscfg, gpioa.pa4);

    let bt = init_bt(
        dp.SPI1, gpiob.pb0, gpiob.pb2, bt_irq, gpioa.pa5, gpioa.pa6, gpioa.pa7, &clocks,
        // bt_buf,
        bt_delay,
    );

    let mut sensors = init_sensors_spi(
        dp.SPI2, gpiob.pb13, gpiob.pb15, gpioa.pa8, gpiob.pb12, gpioc.pc13, &clocks,
    );

    let dwt = cp.DWT.constrain(cp.DCB, &clocks);

    let adc = init_adc(dp.ADC1, gpiob.pb1);

    let leds = init_leds(gpiob.pb4, gpiob.pb5);

    let motors =
        MotorsPWM::new(dp.TIM4, gpiob.pb6, gpiob.pb7, gpiob.pb8, gpiob.pb9, &clocks);

    InitStruct {
        dwt,
        exti: dp.EXTI,
        clocks,
        tim3: dp.TIM3,
        tim9: dp.TIM9,
        tim10: dp.TIM10,
        mono,
        sensors,
        bt,
        adc,
        leds,
        motors,
    }
}

// fn init_adc(adc1: ADC1, pb1: PB1) -> BatteryAdc {
pub fn init_adc(adc1: ADC1, pb1: PB1) -> BatteryAdc {
    use stm32f4xx_hal::adc::config::*;
    use stm32f4xx_hal::adc::*;

    let voltage = pb1.into_analog();

    let adc_cfg = AdcConfig::default().resolution(Resolution::Twelve);

    let mut adc = Adc::adc1(adc1, true, adc_cfg);

    BatteryAdc::new(adc, voltage)
}

fn init_sensors_spi(
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

/// Gyroscope is configured a narrow bandwidth low-pass filter
/// Accelerometer is configured to use an anti-aliasing analog low-pass filter,
///     a digital low-pass filter, and a composite filter
/// Mag without filter
pub fn init_sensors(sensors: &mut Sensors) {
    use crate::sensors::imu::config::*;

    let mut imu_cfg = ImuConfig::default();
    imu_cfg.block_data_update = true;

    // imu_cfg.acc_power = AccelPowerModes::Normal104;
    // imu_cfg.acc_power = AccelPowerModes::Normal208;
    // imu_cfg.acc_power = AccelPowerModes::High833; // matches 800 Hz update rate

    /// 1660 Hz or greater needed for analog filter chain
    // imu_cfg.acc_power = AccelPowerModes::High1660; // default in ST firmware
    // imu_cfg.acc_power = AccelPowerModes::High1660;
    // imu_cfg.acc_power = AccelPowerModes::High3330; // gyro doesn't work at 6660
    imu_cfg.acc_power = AccelPowerModes::High6660; // fastest possible

    imu_cfg.acc_scale = AccelScaleFactor::S4;

    imu_cfg.acc_analog_lp_bandwidth = AccelAnalogBandwidth::BW1500;
    imu_cfg.acc_filter_input_composite = AccelInputComposite::LowLatency;
    imu_cfg.acc_digital_filter_config = AccelDigFilterConfig::OdrLowPass400;

    // imu_cfg.gyro_power = GyroPowerModes::High416; // default in ST firmware
    // imu_cfg.gyro_power = GyroPowerModes::High833; // matches 800 Hz update rate
    // imu_cfg.gyro_power = GyroPowerModes::High1660;
    // imu_cfg.gyro_power = GyroPowerModes::High3330;

    imu_cfg.gyro_power = GyroPowerModes::High6660; // fastest possible
    imu_cfg.gyro_scale = GyroScaleFactor::S2000;

    // gyro low-pass filter
    imu_cfg.gyro_lp_filter_enable = true;
    // imu_cfg.gyro_lp_bandwidth = GyroLpBandwidth::Narrow;
    imu_cfg.gyro_lp_bandwidth = GyroLpBandwidth::VeryNarrow;

    sensors.with_spi_imu(|spi, imu| {
        imu.reset(spi).unwrap();
        imu.init(spi, imu_cfg).unwrap();
    });

    sensors.with_spi_mag(|spi, mag| {
        mag.init_continuous(spi, crate::sensors::magneto::MagDataRate::R100)
            .unwrap();
    });

    // sensors.with_spi_baro(|spi, baro| {
    //     baro.init(spi).unwrap();
    //     baro.set_data_rate(spi, crate::sensors::barometer::BaroDataRate::R75)
    //         .unwrap();
    // });
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
    // pa4.into_input().internal_resistor(Pull::Down)
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
    // buf: &'static mut [u8],
    delay: CounterUs<TIM2>,
    // delay: FTimerMs<TIM2>,
    // uart: &mut UART,
    // ) -> BTController<'static> {
) -> BTController {
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

    // let mut spi = spi1.spi((sck, miso, mosi), mode, 1.MHz(), &clocks);
    // let mut spi = spi1.spi((sck, miso, mosi), mode, 2.MHz(), &clocks);
    // let mut spi = spi1.spi((sck, miso, mosi), mode, 4.MHz(), &clocks);

    /// 8 MHz seems to work most reliably (PCLK / 8)
    let mut spi = spi1.spi((sck, miso, mosi), mode, 8.MHz(), &clocks);

    // let mut bt: BTController<'_> = BluetoothSpi::new(spi, cs, reset, input, buf, delay);
    let mut bt: BTController = BluetoothSpi::new(spi, cs, reset, input, delay);

    bt
}

pub fn init_leds(pb4: Pin<'B', 4, Alternate<0>>, pb5: PB5) -> LEDs {
    let mut led1_pin = pb5
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(stm32f4xx_hal::gpio::Pull::None);
    let mut led2_pin = pb4
        .into_push_pull_output()
        .speed(Speed::High)
        .internal_resistor(stm32f4xx_hal::gpio::Pull::None);
    led1_pin.set_high();
    led2_pin.set_high();

    LEDs::new(led1_pin, led2_pin)
}

fn init_clocks(rcc: RCC) -> Clocks {
    // /// Power interface clock enable
    // rcc.apb1enr.modify(|r, w| w.pwren().set_bit());

    let mut rcc = rcc.constrain();

    let clocks = rcc
        .cfgr
        //
        .use_hse(16.MHz())
        // .sysclk(32.MHz())
        // .sysclk(64.MHz())
        .sysclk(84.MHz())
        // .require_pll48clk()
        .freeze();

    clocks
}

#[cfg(feature = "nope")]
fn init_clocks(rcc: RCC) -> Clocks {
    let mut rcc = rcc.constrain();
    // let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(32.MHz()).freeze();
    // let clocks = rcc.cfgr.sysclk(32.MHz()).freeze();

    // /// Works, any change makes UART stop working
    // let clocks = rcc.cfgr.sysclk(16.MHz()).use_hse(16.MHz()).freeze();

    // /// works
    // let clocks = rcc.cfgr.use_hse(16.MHz()).sysclk(32.MHz()).freeze();

    let speed: stm32f4xx_hal::time::Hertz = 32.MHz();
    // let speed: stm32f4xx_hal::time::Hertz = 48.MHz();

    let clocks = rcc
        .cfgr
        //
        .use_hse(16.MHz())
        // .sysclk(32.MHz())
        .sysclk(speed)
        .hclk(speed)
        // .pclk1(42.MHz())
        // .pclk2(84.MHz())
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
