#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![no_std]
#![no_main]
// XXX: Features:
// for doc comments on var assignments
#![feature(stmt_expr_attributes)]
// XXX: Clippy
#![allow(
    // clippy::all,
    // clippy::restriction,
    clippy::pedantic,
    // clippy::cargo,
    // clippy::complexity,
    // clippy::correctness,
    // clippy::nursery,
    // clippy::restriction,
    clippy::style,
    // clippy::suspicious,
    // clippy::perf,

    clippy::type_complexity,
    clippy::useless_conversion,
    clippy::too_many_arguments,
)]

pub mod battery;
pub mod bluetooth;
pub mod bt_control;
pub mod bt_state;
pub mod consts;
pub mod flight_control;
pub mod init;
pub mod leds;
pub mod math;
pub mod motors;
pub mod pid;
pub mod sensors;
pub mod spi;
pub mod time;
// pub mod uart;
pub mod misc;
pub mod utils;

use crate::{
    battery::*, bluetooth::*, bt_control::*, consts::*, flight_control::*,
    init::init_all_pre, math::*, sensors::barometer::*, sensors::imu::*,
    sensors::magneto::*, sensors::*, spi::*, time::*, utils::*,
};

use bt_control::service_log::BTMessQueue;
use byteorder::ByteOrder;

// pick a panicking behavior
// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use panic_probe as _;

/// defmt global_logger
use defmt_rtt as _;

use defmt::println as rprintln;

// use cortex_m::asm;
// use cortex_m::{iprintln, peripheral::ITM};

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

// use rtt_target::{rprint, rprintln, rtt_init_print};

use embedded_hal as hal;
use hal::spi::*;
use stm32f4::stm32f401::{self, SPI2};
use stm32f4xx_hal::{
    block,
    dwt::DwtExt,
    gpio::{Pin, PinExt, Speed},
    nb,
    prelude::*,
    serial::Serial,
    spi::NoMiso,
    time::*,
};

use crate::bluetooth::{
    events::BlueNRGEvent, gap::Commands as GapCommands, gatt::Commands as GattCommands,
    hal_bt::Commands as HalCommands,
};
use bluetooth_hci_defmt::{host::uart::Hci as HciUart, host::Hci};
use core::convert::Infallible;

static BT_MESS_QUEUE: BTMessQueue = BTMessQueue::new();

// #[cfg(feature = "nope")]
#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3,SPI4,EXTI0])]
mod app {

    use core::f32::consts::PI;

    use fugit::{HertzU32, MillisDurationU32};
    use stm32f4::stm32f401::{self, EXTI, TIM10, TIM2, TIM3, TIM4, TIM5, TIM9};

    // use rtt_target::{rprint, rprintln, rtt_init_print};
    use defmt::println as rprintln;

    use stm32f4xx_hal::{
        block,
        dwt::Dwt,
        gpio::{Output, Pin},
        prelude::*,
        timer::{CounterHz, CounterMs, DelayMs},
    };

    use crate::{
        battery::BatteryAdc,
        bluetooth::gap::Commands as GapCommands,
        bluetooth::{events::BlueNRGEvent, hal_bt::Commands as HalCommands},
        bluetooth::{gap::ConnectionUpdateParameters, gatt::Commands as GattCommands},
        bt_control::{
            service_log::{BTMessQueue, BTMessage},
            BTController, BTEvent,
        },
        bt_state::{BTState, ConnectionChange},
        consts::*,
        flight_control::{ControlInputs, DroneController, IdPID, MotorOutputs},
        init::*,
        leds::LEDs,
        math::*,
        motors::MotorsPWM,
        pid::PID,
        sensors::{ahrs::*, V3},
        sensors::{filtering::SensorFilters, SensorData, Sensors, UQuat},
        time::MonoTimer,
        utils::round_to,
        utils::*,
        BT_MESS_QUEUE,
    };

    use bluetooth_hci_defmt::{
        event::CommandStatus,
        host::{
            uart::Hci as HciUart, ConnectionIntervalBuilder, ExpectedConnectionLength,
            Hci,
        },
        ConnectionHandle,
    };

    use nalgebra as na;

    #[shared]
    struct Shared {
        dwt:           Dwt,
        exti:          EXTI,
        // ahrs:          AhrsController<AhrsComplementary>,
        // ahrs:          AhrsController<AhrsMahony>,
        ahrs:          AhrsController<AhrsFusion>,
        // ahrs:          AhrsController<AhrsExtKalman>,
        // ahrs:          AhrsController<AhrsMadgwick>,
        sens_data:     SensorData,
        flight_data:   FlightData,
        sens_filters:  SensorFilters,
        bt:            BTController,
        // tim9_flag:     bool,
        adc:           BatteryAdc,
        leds:          LEDs,
        motors:        MotorsPWM,
        motor_outputs: MotorOutputs,
        inputs:        ControlInputs,
        controller:    DroneController,
        dbg_gyro:      bool,
    }

    #[local]
    struct Local {
        //
        sensors: Sensors,
        tim3:    CounterHz<TIM3>,
        tim9:    CounterHz<TIM9>,
        tim10:   CounterHz<TIM10>,
        // t_imu:   u32,
        // t_mag:   u32,
        // interval: MillisDurationU32,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MonoTick = MonoTimer<TIM5, 1_000_000>; // 1 MHz

    #[init(local = [])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut cp: stm32f401::CorePeripherals = cx.core;
        let mut dp: stm32f401::Peripherals = cx.device;

        let mut init_struct = init_all(cp, dp);

        let clocks = init_struct.clocks;
        let mono = init_struct.mono;
        let mut exti = init_struct.exti;
        let mut sensors = init_struct.sensors;
        let mut bt = init_struct.bt;
        let mut dwt = init_struct.dwt;

        rprintln!("sensor_period = {:?}", SENSOR_FREQ);
        rprintln!("pid_period    = {:?}", PID_FREQ);

        bt.pause_interrupt(&mut exti);
        match bt.init_bt() {
            Ok(()) => {}
            e => {
                // rprintln!("init_bt error = {:?}", e);
                rprintln!("init_bt error ???");
            }
        }
        // bt.unpause_interrupt(&mut exti);
        loop {
            if !bt.data_ready().unwrap() {
                break;
            }
            rprintln!("wat 0");
            bt.read_event_uart().unwrap();
        }
        bt.unpause_interrupt(&mut exti);

        // bt.clear_interrupt();
        // bt.unpend();

        let mut tim3: stm32f4xx_hal::timer::CounterHz<TIM3> =
            init_struct.tim3.counter_hz(&clocks);

        let mut tim10: stm32f4xx_hal::timer::CounterHz<TIM10> =
            init_struct.tim10.counter_hz(&clocks);

        let mut tim9: stm32f4xx_hal::timer::CounterHz<TIM9> =
            init_struct.tim9.counter_hz(&clocks);

        /// enable sensors and configure settings
        init_sensors(&mut sensors);

        let mut controller = DroneController::new_default_params();

        // controller.pid_pitch_rate.kp = 0.01;
        // controller.pid_pitch_rate.kd = 0.01;
        // controller.pid_roll_rate.kp = 0.01;
        // controller.pid_roll_rate.kd = 0.01;

        // controller
        //     .pid_pitch_rate
        //     .set_d_lowpass(PID_FREQ.to_Hz() as f32, 100.0);

        /// Fusion
        let mut ahrs_alg = AhrsFusion::new(
            SENSOR_FREQ,
            // 0.5,
            7.5,
        );
        ahrs_alg.cfg_acc_rejection = 10.0;
        ahrs_alg.cfg_mag_rejection = 20.0;

        let mut ahrs = AhrsController::new(ahrs_alg, SENSOR_FREQ);

        ahrs.calibration.hard_iron_offset = V3::new(
            -167175.0, //
            -17325.0,  //
            101025.0,
        );

        let sens_filters = SensorFilters::new();

        // /// NED
        // ahrs.ahrs.set_mag_ref(V3::new(
        //     17611.0, //
        //     5026.6,  //
        //     50522.0, //
        // ));

        /// start Sensor timer
        tim10.start(SENSOR_FREQ).unwrap();
        tim10.listen(stm32f4xx_hal::timer::Event::Update);

        /// start PID timer
        tim3.start(PID_FREQ).unwrap();
        tim3.listen(stm32f4xx_hal::timer::Event::Update);

        /// start Main Loop timer
        tim9.start(MAIN_LOOP_FREQ).unwrap();
        tim9.listen(stm32f4xx_hal::timer::Event::Update);

        // let bt_period = 200.Hz();
        // /// start bt_test timer
        // tim9.start(bt_period).unwrap();
        // tim9.listen(stm32f4xx_hal::timer::Event::Update);

        let v = init_struct.adc.sample();
        rprintln!("Battery = {:?} V", v);

        // let v = init_struct.adc.sample_avg(5);
        // rprintln!("Battery/5 = {:?} V", v);

        let shared = Shared {
            dwt,
            exti,
            ahrs,
            sens_data: SensorData::default(),
            flight_data: FlightData::default(),
            sens_filters,
            bt,
            // tim9_flag: false,
            adc: init_struct.adc,
            leds: init_struct.leds,
            motors: init_struct.motors,
            motor_outputs: MotorOutputs::default(),
            inputs: ControlInputs::new(),
            controller,
            dbg_gyro: false,
        };

        let local = Local {
            sensors,
            tim3,
            tim9,
            tim10,
            // t_imu,
            // t_mag,
            // interval,
        };

        // timer_sensors::spawn_after(100.millis()).unwrap();
        // timer_pid::spawn_after(100.millis()).unwrap();

        // main_loop::spawn_after(100.millis()).unwrap();

        // set_dbg_gyro::spawn(true).unwrap();
        // test_motors::spawn(0.05).unwrap();
        // test_motors::spawn_after(2_500.millis(), 0.1).unwrap();
        // test_motors::spawn_after(5_000.millis(), 0.15).unwrap();
        // test_motors::spawn_after(7_500.millis(), 0.2).unwrap();
        // kill_motors::spawn_after(8_000.millis()).unwrap();

        // let throttle = 0.1;
        // test_motors::spawn(throttle).unwrap();
        // set_dbg_gyro::spawn(true).unwrap();
        // kill_motors::spawn_after(5_000.millis()).unwrap();

        // bt_test::spawn_after(100.millis()).unwrap();

        (shared, local, init::Monotonics(mono))
    }

    #[task(shared = [dbg_gyro], priority = 7)]
    fn set_dbg_gyro(mut cx: set_dbg_gyro::Context, enable: bool) {
        cx.shared.dbg_gyro.lock(|d| *d = enable);
    }

    #[task(shared = [adc], priority = 7)]
    fn show_battery(mut cx: show_battery::Context) {
        cx.shared.adc.lock(|adc| {
            let v0 = adc.sample();
            // let v = adc.sample_avg(5);
            // rprintln!("v0 = {:?}\nv = {:?}", v0, v);
            rprintln!("v0 = {:?}", v0);
        });
        show_battery::spawn_after(250.millis()).unwrap();
    }

    #[task(shared = [motors, inputs, dbg_gyro], priority = 6, capacity = 5)]
    fn test_motors(mut cx: test_motors::Context, throttle: f32) {
        (cx.shared.motors, cx.shared.inputs, cx.shared.dbg_gyro).lock(
            |motors, inputs, dbg_gyro| {
                // *dbg_gyro = true;
                inputs.set_motors_armed(true);
                // inputs.throttle = 0.225;
                inputs.set_throttle(throttle);
                motors.set_armed_unchecked(true);
                motors.set_all_f32(throttle);
            },
        );
    }

    #[task(shared = [motors, dbg_gyro], priority = 7)]
    fn kill_motors(mut cx: kill_motors::Context) {
        rprintln!("disarming motors");
        (cx.shared.motors, cx.shared.dbg_gyro).lock(|motors, dbg_gyro| {
            *dbg_gyro = false;
            motors.set_disarmed();
        });
    }

    #[task(
        binds = TIM1_UP_TIM10,
        shared = [sens_data, sens_filters, dbg_gyro],
        local = [tim10, sensors, counter: (u32,u32,u32) = (0,0,0)],
        priority = 4
    )]
    fn timer_sensors(mut cx: timer_sensors::Context) {
        cx.local
            .tim10
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);

        /// mag: 23394 ns, 42.75 kHz
        /// imu: 33955 ns, 29.5 kHz
        // (cx.shared.sens_data, cx.shared.sens_filters).lock(|sd, filters| {
        (
            cx.shared.sens_data,
            cx.shared.sens_filters,
            cx.shared.dbg_gyro,
        )
            .lock(|sd, filters, dbg_gyro| {
                /// Read sensor data
                cx.local.sensors.read_data_mag(sd, filters);
                let (gyro_rdy, acc_rdy) = cx.local.sensors.read_data_imu(sd, filters);

                // if gyro_rdy {
                //     let gyro = sd.imu_gyro.read_and_reset();
                //     rprintln!("{}\n{}\n{}", gyro.x, gyro.y, gyro.z);
                // } else {
                //     rprintln!("no Gyro");
                // }

                // let gyro_rdy = if gyro_rdy { 1 } else { 0 };
                // let acc_rdy = if acc_rdy { 1 } else { 0 };
                // rprintln!("gyro_rdy, acc_rdy = {:?}, {:?}", gyro_rdy, acc_rdy);

                // if gyro_rdy {
                //     cx.local.counter.0 += 1;
                //     // print_v3("gyro = ", sd.imu_gyro.read_and_reset(), 5);
                // }
                // if acc_rdy {
                //     cx.local.counter.1 += 1;
                // }
                // cx.local.counter.2 += 1;
                // if cx.local.counter.2 > 3330 * 2 {
                //     rprintln!(
                //         "gyro: {:?}\nacc: {:?}\ntotal: {:?}",
                //         cx.local.counter.0 as f32 / cx.local.counter.2 as f32,
                //         cx.local.counter.1 as f32 / cx.local.counter.2 as f32,
                //         cx.local.counter.2,
                //     );
                //     *cx.local.counter = (0, 0, 0);
                // }

                // if *dbg_gyro {
                //     let gyro = sd.imu_gyro.read_and_reset();
                //     rprintln!("{}", gyro.x);
                //     // let acc = sd.imu_acc.read_and_reset();
                //     // rprintln!("{}", acc.x);
                // }

                // // TODO: read baro
                // let (pressure_rdy, temp_rdy) =
                //     cx.local.sensors.read_data_baro(sd, filters);

                // rprintln!("pressure, temp = {:?}, {:?}", pressure_rdy, temp_rdy);
                // let pressure = sd.baro_pressure.read_and_reset();
                // let temp = sd.baro_temperature.read_and_reset();
                // rprintln!("pressure = {:?}\ntemp = {:?}", pressure, temp);

                //
            });
    }

    #[task(
        binds = TIM3,
        shared = [ahrs, sens_data, flight_data, motors, inputs, controller, dwt],
        local = [tim3, counter: u32 = 0, pitch: (f32,f32) = (0.0, 0.0)],
        // priority = 5,
        priority = 3,
    )]
    fn timer_pid(mut cx: timer_pid::Context) {
        cx.local
            .tim3
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);

        /// ahrs update: 34.5 - 37.8 us, 27.6 kHz
        /// PID updates: ~120 us, 8.3 kHz
        (
            cx.shared.ahrs,
            cx.shared.sens_data,
            cx.shared.flight_data,
            // cx.shared.tim9_flag,
            cx.shared.motors,
            cx.shared.inputs,
            cx.shared.controller,
            cx.shared.dwt,
        )
            .lock(
                // |ahrs, sd, fd, tim9_flag, motors, inputs, controller| {
                |ahrs, sd, fd, motors, inputs, controller, dwt| {
                    /// update AHRS
                    let gyro = sd.imu_gyro.read_and_reset();
                    let acc = sd.imu_acc.read_and_reset();
                    let mag = sd.magnetometer.read_and_reset();

                    // let t0 = dwt.measure(|| {
                    // });
                    // rprintln!("t0 = {:?} ns", t0.as_nanos());

                    // let gyro0 = gyro;
                    // let gyro = ahrs.update(gyro, acc, mag);
                    let gyro2 = ahrs.update(gyro, acc, mag);

                    #[cfg(feature = "nope")]
                    if sd.baro_pressure.is_changed() && sd.baro_temperature.is_changed() {
                        let pressure = sd.baro_pressure.read_and_reset();
                        let temp = sd.baro_temperature.read_and_reset();

                        // let (alt, (pressure0, temp0)) =
                        let alt = ahrs.altitude.update_altitude(pressure, temp);

                        // if let Some(alt) = alt {
                        //     rprintln!("pressure = {:?}\nalt = {:?}", pressure, alt);
                        // }

                        // rprintln!(
                        //     "pressure = {:?}\npressure0 = {:?}\ntemp = {:?}\ntemp0 = {:?}\nalt = {:?}",
                        //     pressure,
                        //     pressure0,
                        //     temp,
                        //     temp0,
                        //     alt,
                        // );
                    }

                    // cx.local.pitch.0 += gyro.y * (1.0 / PID_FREQ.to_Hz() as f32);
                    // cx.local.pitch.1 += gyro2.y * (1.0 / PID_FREQ.to_Hz() as f32);

                    // print_v3("gyro = ", gyro, 5);

                    /// update FlightData
                    fd.update(ahrs);

                    /// update PIDs
                    /// XXX: use offset adjusted gyro?
                    // let motor_outputs = controller.update(*inputs, &fd.quat, gyro);
                    let motor_outputs = controller.update(inputs, &fd.quat, gyro2);

                    // if motors.is_armed() {
                    //     rprintln!("{:?}", gyro2.x);
                    //     // rprintln!("{:?}", controller.pid_pitch_rate.prev_output.d);
                    // }

                    // if motors.is_armed() {
                    //     let (roll, pitch, yaw) = fd.get_euler_angles();
                    //     rprintln!(
                    //         "{:?}, {:?}, {:?}, {:?}, {:?}",
                    //         controller.pid_pitch_rate.prev_output.p,
                    //         controller.pid_pitch_rate.prev_output.i,
                    //         controller.pid_pitch_rate.prev_output.d,
                    //         gyro2.x,
                    //         pitch,
                    //     );
                    // }

                    /// apply mixed PID outputs to motors
                    motor_outputs.apply(motors);

                    /// print at FREQ / X Hz
                    // #[cfg(feature = "nope")]
                    if *cx.local.counter >= PID_FREQ.to_Hz() / 100 {
                        *cx.local.counter = 0;

                        // let (roll, pitch, yaw) = fd.get_euler_angles();
                        // rprintln!(
                        //     "gyro: {}\nahrs: {}\nsum1:  {}\nsum2:  {}",
                        //     gyro2.x,
                        //     rad_to_deg(pitch), //
                        //     cx.local.pitch.0,  //
                        //     cx.local.pitch.1,  //
                        // );

                        // let (roll, pitch, yaw) = fd.get_euler_angles();
                        // rprintln!(
                        //     "{:08}, {:08}\n{:08}, {:08}\n(r,p,y) = {:08}, {:08}, {:08}",
                        //     // "  3,   2\n{:08}, {:08}\n{:08}, {:08}\n  4,
                        //     // 1\n(r,p,y) = {:08}, {:08}, {:08}",
                        //     round_to(motor_outputs.back_right, 4),
                        //     round_to(motor_outputs.back_left, 4),
                        //     round_to(motor_outputs.front_right, 4), // XXX: rotate 180
                        //     round_to(motor_outputs.front_left, 4), // to match position on table
                        //     r(rad_to_deg(roll)),
                        //     r(rad_to_deg(pitch)),
                        //     r(rad_to_deg(yaw)),
                        // );

                        // let (roll, pitch, yaw) = fd.get_euler_angles();
                        // rprintln!(
                        //     "(r,p,y) = \n\t{:08}\n\t{:08}\n\t{:08}",
                        //     r(rad_to_deg(roll)),
                        //     r(rad_to_deg(pitch)),
                        //     r(rad_to_deg(yaw)),
                        // );

                        //
                    } else {
                        *cx.local.counter += 1;
                    }

                    // *tim9_flag = true;
                },
            );
    }

    #[task(
        binds = TIM1_BRK_TIM9,
        // shared = [bt, exti, flight_data, sens_data, dwt, adc, tim9_flag, inputs, controller, motors, ahrs],
        shared = [bt, exti, flight_data, sens_data, dwt, adc, inputs, controller, motors, ahrs],
        local = [tim9, batt_counter: u32 = 0, batt_warn: u32 = 0],
        priority = 2
    )]
    fn main_loop(mut cx: main_loop::Context) {
        cx.local
            .tim9
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);

        // const BATT_TIMES: u32 = 20;
        const BATT_TIMES: u32 = 10;

        let flight_data = cx.shared.flight_data;
        let sens_data = cx.shared.sens_data;
        let bt = cx.shared.bt;
        let exti = cx.shared.exti;
        // let inputs = cx.shared.inputs;
        let motors = cx.shared.motors;
        let controller = cx.shared.controller;

        let ahrs = cx.shared.ahrs;

        (flight_data, sens_data, bt, controller, motors, ahrs).lock(
            |fd, sd, bt, controller, motors, ahrs| {
                /// Write data to Bluetooth

                /// send orientation
                bt.log_write_quat(&fd.quat).unwrap();

                // /// send battery voltage
                // if *cx.local.batt_counter >= BATT_TIMES {
                //     *cx.local.batt_counter = 0;
                //     cx.shared.adc.lock(|adc| {
                //         let v = adc.sample();
                //         // let v = adc.sample_avg(3);
                //         if motors.is_armed() && v <= adc.min_voltage {
                //             rprintln!("Low Battery, disarming motors");
                //             motors.set_disarmed();
                //         }
                //         bt.pause_interrupt(exti);
                //         bt.log_write_batt(v).unwrap();
                //         bt.unpause_interrupt(exti);
                //     });
                // } else {
                //     *cx.local.batt_counter += 1;
                // }

                /// send battery voltage
                let send_bt = if *cx.local.batt_counter >= BATT_TIMES {
                    *cx.local.batt_counter = 0;
                    // rprintln!("sending battery");
                    // BT_MESS_QUEUE.enqueue(BTMessage::Test(*cx.local.batt_counter as u8));
                    true
                } else {
                    *cx.local.batt_counter += 1;
                    false
                };

                /// check battery levels, disarm if below threshold
                cx.shared.adc.lock(|adc| {
                    // rprintln!("checking battery, sending = {}", send_bt);
                    if adc.battery_warning(
                        bt,
                        cx.local.batt_warn,
                        motors.is_armed(),
                        send_bt,
                    ) {
                        rprintln!("Low Battery, disarming motors");
                        motors.set_disarmed();
                    }
                });

                // let alt = ahrs.altitude.get_altitude();
                // let alt = alt * 100.0;
                // let gyro0 = V3::new(alt, alt, 0.0);

                let gyro0 = sd.imu_gyro.read_and_reset();
                // let acc0 = sd.imu_acc.read_and_reset();
                // let mag0 = sd.magnetometer.read_and_reset();

                // bt.log_write_sens(gyro0, acc0, mag0).unwrap();
                bt.log_write_sens_gyro(gyro0).unwrap();

                // let pids = [IdPID::PitchStab];
                // let pids = [IdPID::PitchRate];
                let pids = [IdPID::PitchRate, IdPID::PitchStab];
                // let pids = [IdPID::YawRate];
                for id in pids {
                    bt.log_write_pid(id, &controller[id]).unwrap();
                }

                while let Some(mess) = BT_MESS_QUEUE.dequeue() {
                    rprintln!("sending mess = {:?}", mess);
                    bt.log_write_mess(mess).unwrap();
                }

                // unimplemented!()
            },
        );

        // main_loop::spawn_after(100.millis()).unwrap();
        // main_loop::spawn().unwrap();
    }

    #[cfg(feature = "nope")]
    // #[task(
    //     binds = TIM1_BRK_TIM9,
    //     shared = [bt, exti, flight_data, sens_data, dwt, adc, tim9_flag, inputs, controller, motors],
    //     local = [tim9, roll: f32 = 0.0],
    //     priority = 2
    // )]
    fn bt_test(mut cx: bt_test::Context) {
        cx.local
            .tim9
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);

        (cx.shared.bt, cx.shared.exti).lock(|bt, exti| {
            let quat = UQuat::from_euler_angles(*cx.local.roll, 0.0, 0.0);
            *cx.local.roll = *cx.local.roll + 1.0;

            /// send orientation
            bt.pause_interrupt(exti);
            bt.log_write_quat(&quat).unwrap();
            bt.unpause_interrupt(exti);
        });
    }

    #[cfg(feature = "nope")]
    // #[task(shared = [bt, exti], priority = 9)]
    fn bt_conn(mut cx: bt_conn::Context, conn_handle: ConnectionHandle) {
        cx.shared.bt.lock(|bt| {
            rprintln!("conn_handle = {:?}", conn_handle);

            rprintln!("conn_handle2 = {:?}", conn_handle);
            let conn_interval = ConnectionIntervalBuilder::new()
                .with_range(
                    core::time::Duration::from_micros(7500 * 2),
                    core::time::Duration::from_micros(7500 * 2),
                )
                .with_latency(0)
                .with_supervision_timeout(core::time::Duration::from_secs_f32(1.0))
                .build()
                .unwrap();
            rprintln!("wat -1, conn_interval = {:?}", conn_interval);
            let params = bluetooth_hci_defmt::host::ConnectionUpdateParameters {
                // crate::bluetooth::gap::ConnectionUpdateParameters {
                conn_handle,
                conn_interval,
                expected_connection_length: ExpectedConnectionLength::new(
                    core::time::Duration::from_millis(32),
                    core::time::Duration::from_millis(32),
                )
                .unwrap(),
            };
            rprintln!("wat 0");
            block!(bt.le_connection_update(&params)).unwrap();
            // block!(bt.start_connection_update(&params)).unwrap();
            let ev0 = bt._read_event();
            rprintln!("ev0 = {:?}", ev0);
            let ev1 = bt._read_event();
            rprintln!("ev1 = {:?}", ev1);
            bt.read_event_uart().unwrap();
            rprintln!("wat 3");

            #[cfg(feature = "nope")]
            for x in 0u16..0x0EFF {
                let conn_handle = ConnectionHandle(x);
                rprintln!("conn_handle2 = {:?}", conn_handle);
                let conn_interval = ConnectionIntervalBuilder::new()
                    .with_range(
                        core::time::Duration::from_micros(7500 * 2),
                        core::time::Duration::from_micros(7500 * 2),
                    )
                    .with_latency(0)
                    .with_supervision_timeout(core::time::Duration::from_secs_f32(1.0))
                    .build()
                    .unwrap();
                rprintln!("wat -1, conn_interval = {:?}", conn_interval);
                let params = bluetooth_hci_defmt::host::ConnectionUpdateParameters {
                    // crate::bluetooth::gap::ConnectionUpdateParameters {
                    conn_handle,
                    conn_interval,
                    expected_connection_length: ExpectedConnectionLength::new(
                        core::time::Duration::from_millis(32),
                        core::time::Duration::from_millis(32),
                    )
                    .unwrap(),
                };
                rprintln!("wat 0");
                block!(bt.le_connection_update(&params)).unwrap();
                // block!(bt.start_connection_update(&params)).unwrap();
                let ev0 = bt._read_event();
                rprintln!("ev0 = {:?}", ev0);

                match ev0 {
                    Ok(bluetooth_hci_defmt::Event::CommandStatus(CommandStatus {
                        status,
                        ..
                    })) => match status {
                        bluetooth_hci_defmt::Status::Success => {
                            rprintln!("Success");
                            break;
                        }
                        bluetooth_hci_defmt::Status::UnknownConnectionId => {
                            rprintln!("failed");
                        }
                        _ => {
                            rprintln!("other = {:?}", status);
                        }
                    },
                    _ => {}
                }

                // let ev1 = bt._read_event();
                // rprintln!("ev1 = {:?}", ev1);
                // bt.read_event_uart().unwrap();
                // rprintln!("wat 3");
            }
        });
    }

    #[task(
        binds = EXTI4,
        shared = [bt, exti, controller, motors, inputs, leds, adc, flight_data],
        priority = 8
    )]
    fn bt_irq(mut cx: bt_irq::Context) {
        (
            cx.shared.bt,
            // cx.shared.exti,
            cx.shared.controller,
            cx.shared.motors,
            cx.shared.inputs,
            cx.shared.adc,
            cx.shared.flight_data,
        )
            // .lock(|bt, exti, controller, motors, inputs, adc, fd| {
            .lock(|bt, controller, motors, inputs, adc, fd| {
                // rprintln!("bt_irq");

                // bt.clear_interrupt();
                // bt.pause_interrupt(exti);

                if !bt.data_ready().unwrap() {
                    bt.clear_interrupt();
                    // rprintln!("bt_irq: no interrupt");
                    return;
                }

                loop {
                    // let event: BTEvent = match bt._read_event() {
                    let event: BTEvent = match bt._read_event_timeout(10.millis()) {
                        Ok(Some(ev)) => {
                            // rprintln!("ev = {:?}", ev);
                            bt.clear_interrupt();
                            ev
                        }
                        Ok(None) => {
                            rprintln!("read event timeout");
                            return;
                        }
                        Err(e) => {
                            rprintln!("read event error = {:?}", e);
                            // unimplemented!()

                            // if bt.data_ready().unwrap() {
                            //     bt.set_pending();
                            // }

                            // bt.unpause_interrupt(exti);
                            return;
                        }
                    };

                    /// TODO: update connection params
                    match bt.handle_connect_disconnect(&event) {
                        // Some(ConnectionChange::NewConnection(conn_handle)) => {
                        //     bt_conn::spawn_after(250.millis(), conn_handle).unwrap();
                        // }
                        #[cfg(feature = "nope")]
                        Some(ConnectionChange::NewConnection(conn_handle)) => {
                            rprintln!("conn_handle = {:?}", conn_handle);
                            // rprintln!("wat -2");
                            let conn_interval = ConnectionIntervalBuilder::new()
                                .with_range(
                                    core::time::Duration::from_micros(7500 * 2),
                                    core::time::Duration::from_micros(7500 * 2),
                                )
                                .with_latency(0)
                                .with_supervision_timeout(
                                    core::time::Duration::from_secs_f32(1.0),
                                )
                                .build()
                                .unwrap();
                            rprintln!("wat -1, conn_interaval = {:?}", conn_interval);
                            let params =
                                bluetooth_hci_defmt::host::ConnectionUpdateParameters {
                                    // crate::bluetooth::gap::ConnectionUpdateParameters {
                                    conn_handle,
                                    conn_interval,
                                    expected_connection_length:
                                        ExpectedConnectionLength::new(
                                            core::time::Duration::from_millis(32),
                                            core::time::Duration::from_millis(32),
                                        )
                                        .unwrap(),
                                };
                            rprintln!("wat 0");
                            block!(bt.le_connection_update(&params)).unwrap();
                            // block!(bt.start_connection_update(&params)).unwrap();
                            let ev0 = bt._read_event();
                            rprintln!("ev0 = {:?}", ev0);
                            let ev1 = bt._read_event();
                            rprintln!("ev1 = {:?}", ev1);
                            bt.read_event_uart().unwrap();
                            rprintln!("wat 3");
                        }
                        Some(ConnectionChange::Disconnect) => {
                            rprintln!("Disconnected, disarming motors");
                            // motors.set_armed(false, &bt.state, *inputs, fd.quat);
                            motors.set_disarmed();
                        }
                        _ => {}
                    }

                    // if let Some(ConnectionChange::Disconnect) =
                    //     bt.handle_connect_disconnect(&event)
                    // {
                    //     rprintln!("Disconnected, disarming motors");
                    //     motors.set_armed(false);
                    // }

                    /// XXX: For safely testing unplugged: kill motors, delayed after arming
                    match bt.handle_input(
                        motors,
                        inputs,
                        controller,
                        fd.quat,
                        adc.last_reading,
                        &event,
                    ) {
                        Some(true) => {
                            // kill_motors::spawn_after(1_000.millis()).unwrap();
                        }
                        _ => {}
                    }

                    if !bt.data_ready().unwrap() {
                        break;
                    }
                }

                // bt.unpause_interrupt(exti);
                // rprintln!("bt_irq done");
            });
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
}

// #[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
#[cfg(feature = "nope")]
mod app {
    use cortex_m_semihosting::{debug, hprintln};
    use fugit::MillisDurationU32;
    use stm32f4::stm32f401::{self, EXTI, TIM10, TIM2, TIM3, TIM4, TIM5, TIM9};

    // use rtt_target::{rprint, rprintln, rtt_init_print};
    use defmt::println as rprintln;

    use stm32f4xx_hal::{
        adc::{config::AdcConfig, Adc},
        block,
        dwt::Dwt,
        gpio::{Output, Pin},
        prelude::*,
        rcc::BusTimerClock,
        timer::{CounterHz, CounterMs, DelayMs},
    };

    use crate::{
        battery::BatteryAdc,
        bluetooth::gap::Commands as GapCommands,
        bluetooth::gatt::Commands as GattCommands,
        bluetooth::{events::BlueNRGEvent, hal_bt::Commands as HalCommands},
        init::init_leds,
        sensors::ahrs::{FlightData, AHRS},
        sensors::{SensorData, Sensors, UQuat},
        time::MonoTimer,
        utils::*,
    };

    use bluetooth_hci_defmt::host::{uart::Hci as HciUart, Hci};

    use nalgebra as na;

    #[shared]
    struct Shared {
        // adc:
    }

    #[local]
    struct Local {
        timer: CounterHz<TIM10>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MonoTick = MonoTimer<TIM5, 1_000_000>; // 1 MHz

    static Q: heapless::mpmc::Q4<u32> = heapless::mpmc::Q4::new();

    #[allow(unreachable_code)]
    #[cfg(feature = "nope")]
    // #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // foo::spawn().unwrap();

        let mut cp: stm32f401::CorePeripherals = cx.core;
        let mut dp: stm32f401::Peripherals = cx.device;

        let mut rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(16.MHz()).sysclk(84.MHz()).freeze();
        rprintln!("sysclk()   core = {:?}", clocks.sysclk());
        rprintln!("hclk()     AHB1 = {:?}", clocks.hclk());

        let mut delay = cp.SYST.delay(&clocks);

        use crate::motors::*;

        let gb = dp.GPIOB.split();

        let mut motors = MotorsPWM::new(dp.TIM4, gb.pb6, gb.pb7, gb.pb8, gb.pb9, &clocks);

        let mut adc = crate::init::init_adc(dp.ADC1, gb.pb1);
        let v = adc.sample();
        rprintln!("v = {:?}", v);

        // let mut leds = init_leds(gb.pb4, gb.pb5);

        // return (Shared {}, Local {}, init::Monotonics());

        motors.set_armed(true);

        // let m0 = MotorSelect::Motor3;
        // let m1 = MotorSelect::Motor4;

        // let pwm = 700;

        // let pwm_f32 = pwm as f32 / MotorsPWM::MOTOR_MAX_PWM;
        // rprintln!("pwm_f32 = {:?}", pwm_f32);

        let pwm = 0.1;

        rprintln!("pwm = {:?}", pwm);
        let pwm_u16 = (pwm * MotorsPWM::MOTOR_MAX_PWM) as u16;
        rprintln!("pwm_u16 = {:?}", pwm_u16);

        // motors.enable_motor(m0);
        // motors.enable_motor(m1);
        motors.enable_all();
        rprintln!("wat 0");

        // motors.set_all_u16(pwm);
        motors.set_all_f32(pwm);
        // motors.set_motor_u16(m0, pwm);
        // motors.set_motor_u16(m1, pwm);

        delay.delay(1000.millis());

        // motors.disable_motor(m0);
        // motors.disable_motor(m1);
        motors.disable_all();
        rprintln!("wat 1");

        (Shared {}, Local {}, init::Monotonics())
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut cp: stm32f401::CorePeripherals = cx.core;
        let mut dp: stm32f401::Peripherals = cx.device;

        let mut rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(16.MHz()).sysclk(84.MHz()).freeze();
        rprintln!("sysclk()   core = {:?}", clocks.sysclk());
        rprintln!("hclk()     AHB1 = {:?}", clocks.hclk());

        let mono = MonoTimer::<TIM5, 1_000_000>::new(dp.TIM5, &clocks);

        let mut tim3: stm32f4xx_hal::timer::CounterHz<TIM10> =
            dp.TIM10.counter_hz(&clocks);

        tim3.start(2.Hz()).unwrap();
        tim3.listen(stm32f4xx_hal::timer::Event::Update);

        wat::spawn().unwrap();

        (Shared {}, Local { timer: tim3 }, init::Monotonics(mono))
    }

    #[task(binds = TIM1_UP_TIM10, shared = [], local = [timer])]
    fn foo(cx: foo::Context) {
        cx.local
            .timer
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);

        rprintln!("foo:");
        while let Some(x) = Q.dequeue() {
            rprintln!("x = {:?}", x);
        }

        // rprintln!("wat");
    }

    #[task(shared = [], local = [x: u32 = 0])]
    fn wat(cx: wat::Context) {
        Q.enqueue(*cx.local.x).unwrap();
        *cx.local.x += 1;

        wat::spawn_after(250.millis()).unwrap();
        // wat::spawn().unwrap();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }
}

#[cfg(feature = "nope")]
// #[entry]
fn main_bluetooth() -> ! {
    use crate::bluetooth::{AccessByte, BTError, BTServices};
    use crate::spi::{Spi4, SpiError};
    use stm32f4xx_hal::gpio::Pull;

    // rtt_init_print!();

    let mut cp = stm32f401::CorePeripherals::take().unwrap();
    let mut dp = stm32f401::Peripherals::take().unwrap();

    /// Enable SPI1 clock
    dp.RCC.apb2enr.modify(|r, w| w.spi1en().set_bit());

    /// Enable GPIOA + GPIOB
    dp.RCC
        .ahb1enr
        .modify(|r, w| w.gpioaen().set_bit().gpioben().set_bit());

    // /// Power interface clock enable
    // dp.RCC.apb1enr.modify(|r, w| w.pwren().set_bit());

    // dp.FLASH.acr.modify(|r, w| {
    //     w.latency() // wait states
    //         .ws2()
    //         .icen() // instruction cache
    //         .set_bit()
    //         .dcen() // data cache
    //         .set_bit()
    //         .prften() // prefetch
    //         .set_bit()
    // });

    // unsafe {
    //     dp.PWR.cr.modify(|r,w| w.vos().bits(0b));
    // }

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(16.MHz())
        // .bypass_hse_oscillator()
        // .sysclk(32.MHz())
        .sysclk(64.MHz())
        // .hclk(64.MHz())
        // .require_pll48clk()
        // .pclk1(32.MHz())
        // .pclk2(64.MHz())
        .freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    let mut syscfg = dp.SYSCFG.constrain();
    let bt_delay = dp.TIM2.counter_ms(&clocks);
    // let mut uart = UART::new(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);
    let mut exti = dp.EXTI;

    // defmt::println!("sysclk()   core = {:?}", clocks.sysclk());
    // defmt::println!("hclk()     AHB1 = {:?}", clocks.hclk());
    rprintln!("sysclk()   core = {:?}", clocks.sysclk());
    rprintln!("hclk()     AHB1 = {:?}", clocks.hclk());

    // uprintln!(uart, "sysclk()   core = {:?}", clocks.sysclk());
    // uprintln!(uart, "hclk()     AHB1 = {:?}", clocks.hclk());
    // uprintln!(uart, "pclk1()    APB1 = {:?}", clocks.pclk1());
    // uprintln!(uart, "pclk2()    APB2 = {:?}", clocks.pclk2());
    // uprintln!(uart, "pll48clk() = {:?}", clocks.pll48clk());
    // uprintln!(uart, "i2s_clk()  = {:?}", clocks.i2s_clk());
    // uprintln!(uart, "ppre1()    = {:?}", clocks.ppre1());
    // uprintln!(uart, "ppre2()    = {:?}", clocks.ppre2());

    let cs = gpiob.pb0;
    let reset = gpiob.pb2;
    let mut input = gpioa.pa4;
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

    // let miso = miso
    //     .internal_resistor(Pull::None)
    //     // .internal_resistor(Pull::Down) // XXX: nope
    //     .into_push_pull_output()
    //     .into_alternate::<5>()
    //     .speed(Speed::High);

    let miso = miso
        .internal_resistor(Pull::None)
        .into_input()
        .into_alternate::<5>()
        .speed(Speed::High);

    let mosi = mosi
        .internal_resistor(Pull::None)
        .into_push_pull_output()
        .speed(Speed::High)
        .into_alternate::<5>();

    let mode = stm32f4xx_hal::spi::Mode {
        polarity: stm32f4xx_hal::spi::Polarity::IdleLow,
        phase:    stm32f4xx_hal::spi::Phase::CaptureOnFirstTransition,
    };

    // let mut input = input.into_input();
    input.make_interrupt_source(&mut syscfg);
    input.enable_interrupt(&mut exti);
    input.trigger_on_edge(&mut exti, stm32f4xx_hal::gpio::Edge::Rising);
    input.clear_interrupt_pending_bit();
    let input = input.into_input();

    // uprintln!(uart, "wat 0");
    // let spi = Spi4::new(dp.SPI1, mode, sck, miso, mosi);

    // let spi = dp.SPI1.spi((sck, miso, mosi), mode, 8.MHz(), &clocks);
    let spi = dp.SPI1.spi((sck, miso, mosi), mode, 2.MHz(), &clocks);

    // static mut BLE_BUFFER: [u8; 512] = [0u8; 512];
    // let buffer = unsafe { &mut BLE_BUFFER[..] };

    let mut bt = BluetoothSpi::new(spi, cs, reset, input, bt_delay);

    // bt.pause_interrupt(&mut exti);

    // use bluetooth_hci_defmt::host::HciHeader;
    // let mut header: [u8; 4] = [0; 4];
    // let opcode = bluetooth_hci_defmt::Opcode::new(0x04, 0x01);
    // bluetooth_hci_defmt::host::uart::CommandHeader::new(opcode, 0).copy_into_slice(&mut header);
    // let header: &[u8] = &header;
    // uprintln!(uart, "header[0] = {:#0x}", header[0]);
    // uprintln!(uart, "header[1] = {:#0x}", header[1]);
    // uprintln!(uart, "header[2] = {:#0x}", header[2]);
    // uprintln!(uart, "header[3] = {:#0x}", header[3]);
    // loop {}

    #[cfg(feature = "nope")]
    loop {
        // uprintln!(uart, "loop");
        rprintln!(uart, "loop");
        block!(bt.read_local_version_information()).unwrap();
        // block!(bt.safe_read_local_ver(&mut uart)).unwrap();

        // uprintln!(uart, "wat 0");
        // while !bt.data_ready().unwrap() {
        //     cortex_m::asm::nop();
        // }
        // uprintln!(uart, "wat 1");

        while bt.data_ready().unwrap() {
            // match bt.read()
            let x: stm32f4xx_hal::nb::Result<
                bluetooth_hci_defmt::host::uart::Packet<BlueNRGEvent>,
                bluetooth_hci_defmt::host::uart::Error<
                    BTError<SpiError, Infallible>,
                    crate::bluetooth::events::BlueNRGError,
                >,
            > = bt.read();
            match x {
                Ok(ev) => {
                    rprintln!("ev = {:?}", ev);
                    break;
                }
                Err(nb::Error::WouldBlock) => {
                    if bt.is_ovr() {
                        rprintln!("overrun");
                    }
                    if bt.is_modf() {
                        rprintln!("modf");
                    }
                }
                Err(e) => {
                    panic!("error = {:?}", e);
                }
            }
        }
    }

    // #[cfg(feature = "nope")]
    {
        // uart.pause();
        bt.pause_interrupt(&mut exti);
        match bt.init_bt() {
            Ok(()) => {}
            e => {
                // uprintln!(uart, "init_bt error = {:?}", e);
                rprintln!("init_bt error = {:?}", e);
            }
        }
        // uart.unpause();
        // bt.unpause_interrupt(&mut exti);

        // bt.wait_ms(100.millis());
        // loop {
        //     if !bt.data_ready().unwrap() {
        //         break;
        //     }
        //     uprintln!(uart, "wat 4");
        //     bt.read_event_uart(&mut uart).unwrap();
        // }

        let mut buf = [0u8; 16];

        let logger = if let Some(logger) = bt.services.logger {
            logger
        } else {
            // uprintln!(uart, "no logger?");
            // uprintln!(uart, "");
            // return Ok(false);
            panic!("no logger?");
        };

        loop {
            for b in buf.iter_mut() {
                *b += 1;
            }

            // uprint!(uart, "0..");
            rprint!("0..");

            let val = crate::bluetooth::gatt::UpdateCharacteristicValueParameters {
                service_handle:        logger.service_handle,
                characteristic_handle: logger.char_handle,
                offset:                0,
                value:                 &buf,
            };
            block!(bt.update_characteristic_value(&val)).unwrap();

            // while !bt.data_ready().unwrap() {
            //     cortex_m::asm::nop();
            // }

            loop {
                // match bt.read()
                let x: stm32f4xx_hal::nb::Result<
                    bluetooth_hci_defmt::host::uart::Packet<BlueNRGEvent>,
                    bluetooth_hci_defmt::host::uart::Error<
                        BTError<SpiError, Infallible>,
                        crate::bluetooth::events::BlueNRGError,
                    >,
                > = bt.read();
                match x {
                    Ok(ev) => {
                        // uprintln!(uart, "ev = {:?}", ev);
                        rprintln!("ev = {:?}", ev);
                        break;
                    }
                    Err(nb::Error::WouldBlock) => {
                        if bt.is_ovr() {
                            rprintln!("overrun");
                        }
                        if bt.is_modf() {
                            rprintln!("modf");
                        }
                    }
                    Err(e) => {
                        panic!("error = {:?}", e);
                    }
                }
                if !bt.data_ready().unwrap() {
                    break;
                }
            }

            // match bt.log_write(&mut uart, false, &buf) {
            //     Ok(true) => {
            //         // uprintln!(uart, "sent log write command");
            //     }
            //     Ok(false) => {
            //         uprintln!(uart, "failed to write");
            //     }
            //     Err(e) => {
            //         uprintln!(uart, "error 0 = {:?}", e);
            //     }
            // }

            bt.unpause_interrupt(&mut exti);
            rprintln!("1");
        }
    }

    // loop {}

    // bt.pause_interrupt(&mut exti);
    // block!(bt.read_local_version_information()).unwrap();
    // uprintln!(uart, "wat -1");
    // bt.read_events_while_ready(&mut uart).unwrap();
    // uprintln!(uart, "wat 0");
    // bt.reset().unwrap();
    // uprintln!(uart, "wat 1");
    // // bt.read_event_uart(&mut uart).unwrap();
    // uprintln!(uart, "wat 2");
    // bt.read_events_while_ready(&mut uart).unwrap();
    // uprintln!(uart, "wat 3");
    // bt.unpause_interrupt(&mut exti);

    // // uart.pause();
    // bt.pause_interrupt(&mut exti);
    // match bt.init_bt(&mut uart) {
    //     Ok(()) => {}
    //     e => {
    //         uprintln!(uart, "init_bt error = {:?}", e);
    //     }
    // }
    // // uart.unpause();
    // bt.unpause_interrupt(&mut exti);

    // uprintln!(uart, "wat 2");
    // let e = bt.block_until_ready(AccessByte::Read, Some(&mut uart));
    // uprintln!(uart, "e 1 = {:?}", e);

    // bt.reset().unwrap();
    // uprintln!(uart, "wat 2");
    // bt.read_event_uart(&mut uart).unwrap();
    // uprintln!(uart, "wat 3");

    // block!(bt.read_local_version_information()).unwrap();
    // uprintln!(uart, "wat 4");
    // bt.read_event_uart(&mut uart).unwrap();
    // uprintln!(uart, "wat 5");

    // loop {}

    //
}

#[cfg(feature = "nope")]
// #[entry]
fn main_test() -> ! {
    let x = bluetooth_hci_defmt::event::command::LmpFeatures::ENCRYPTION;

    rprintln!("x = {:?}", x);
    rprintln!("x = {:?}", defmt::Debug2Format(&x));

    loop {}
}

#[cfg(feature = "nope")]
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

#[cfg(feature = "nope")]
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

#[cfg(feature = "nope")]
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

#[cfg(feature = "nope")]
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
