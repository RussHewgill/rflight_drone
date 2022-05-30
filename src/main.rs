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

pub mod battery;
pub mod bluetooth;
pub mod bt_control;
pub mod bt_state;
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
pub mod utils;

use crate::{
    battery::*, bluetooth::*, bt_control::*, flight_control::*, init::init_all_pre,
    math::*, sensors::barometer::*, sensors::imu::*, sensors::magneto::*, sensors::*,
    spi::*, time::*, utils::*,
};

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

// #[cfg(feature = "nope")]
#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3,EXTI0])]
mod app {

    use cortex_m_semihosting::{debug, hprintln};
    use fugit::MillisDurationU32;
    use stm32f4::stm32f401::{self, EXTI, TIM10, TIM2, TIM3, TIM5, TIM9};

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
        bt_state::{BTState, ConnectionChange},
        flight_control::{ControlInputs, DroneController, IdPID, MotorOutputs},
        leds::LEDs,
        motors::MotorsPWM,
        pid::PID,
        sensors::{ahrs::*, V3},
        sensors::{SensorData, Sensors, UQuat},
        time::MonoTimer,
        utils::*,
    };

    use bluetooth_hci_defmt::host::{
        uart::Hci as HciUart, ConnectionIntervalBuilder, ExpectedConnectionLength, Hci,
    };

    use nalgebra as na;

    use crate::{
        bt_control::{BTController, BTEvent},
        init::*,
        math::*,
        utils::round_to,
    };

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
        bt:            BTController,
        tim9_flag:     bool,
        adc:           BatteryAdc,
        leds:          LEDs,
        motors:        MotorsPWM,
        motor_outputs: MotorOutputs,
        inputs:        ControlInputs,
        controller:    DroneController,
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

    #[init(local = [])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut cp: stm32f401::CorePeripherals = cx.core;
        let mut dp: stm32f401::Peripherals = cx.device;

        let sensor_period: stm32f4xx_hal::time::Hertz = 800.Hz();
        // let sensor_period: stm32f4xx_hal::time::Hertz = 200.Hz();
        // let sensor_period: stm32f4xx_hal::time::Hertz = 100.Hz();
        // let sensor_period: stm32f4xx_hal::time::Hertz = 50.Hz();
        // let sensor_period: stm32f4xx_hal::time::Hertz = 5.Hz();

        // let main_period: stm32f4xx_hal::time::Hertz = 50.Hz();

        // let mut init_struct = init_all(cp, dp, &mut bt_buf[..]);
        let mut init_struct = init_all(cp, dp);

        let clocks = init_struct.clocks;
        let mono = init_struct.mono;
        let mut exti = init_struct.exti;
        let mut sensors = init_struct.sensors;
        let mut bt = init_struct.bt;
        let mut dwt = init_struct.dwt;

        rprintln!("sensor_period = {:?}", sensor_period.to_Hz());

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

        /// enable sensors and configure settings
        init_sensors(&mut sensors);

        let controller = DroneController::new_default_params();

        /// Fusion
        let mut ahrs_alg = AhrsFusion::new(
            sensor_period,
            // 0.5,
            7.5,
        );
        ahrs_alg.cfg_acc_rejection = 10.0;
        ahrs_alg.cfg_mag_rejection = 20.0;

        // ahrs.calibration.hard_iron_offset = V3::new(
        //     -5.1751766, //
        //     -0.5539105, //
        //     3.117466,
        // );

        // /// Kalman
        // let mut ahrs_alg = AhrsExtKalman::new(
        //     sensor_period,
        //     0.5, // XXX: ???
        // );

        // /// complementary
        // let gain = 0.1;
        // let ahrs_alg = AhrsComplementary::new(sensor_period, gain);

        // /// Madgwick
        // let ahrs_alg = AhrsMadgwick::new(sensor_period, 40.0);

        // /// Mahony
        // let ahrs_alg = AhrsMahony::new(sensor_period);

        // /// ST ahrs
        // let ahrs_alg = AhrsST::new(sensor_period);

        let mut ahrs = AhrsController::new(ahrs_alg, sensor_period);

        ahrs.calibration.hard_iron_offset = V3::new(
            -5.1751766, //
            -0.5539105, //
            3.117466,
        );

        // /// NED
        // ahrs.ahrs.set_mag_ref(V3::new(
        //     17611.0, //
        //     5026.6,  //
        //     50522.0, //
        // ));

        // let interval = (((1.0 / main_period.raw() as f32) * 1000.0) as u32).millis();
        // uprintln!(uart, "interval = {:?}", interval);

        // rprintln!(
        //     "core::mem::size_of::<PID>() = {:?}",
        //     core::mem::size_of::<PID>()
        // );

        /// start timer
        tim3.start(sensor_period).unwrap();
        tim3.listen(stm32f4xx_hal::timer::Event::Update);

        let shared = Shared {
            dwt,
            exti,
            ahrs,
            sens_data: SensorData::default(),
            flight_data: FlightData::default(),
            bt,
            tim9_flag: false,
            adc: init_struct.adc,
            leds: init_struct.leds,
            motors: init_struct.motors,
            motor_outputs: MotorOutputs::default(),
            inputs: ControlInputs::default(),
            controller,
        };

        let local = Local {
            sensors,
            tim3,
            // interval,
        };

        // timer_sensors::spawn_after(100.millis()).unwrap();

        // main_loop::spawn_after(100.millis()).unwrap();

        // bt_test::spawn_after(100.millis()).unwrap();

        (shared, local, init::Monotonics(mono))
    }

    /// fusion
    #[cfg(feature = "nope")]
    // #[task(
    //     binds = TIM3,
    //     shared = [ahrs, sens_data, flight_data, tim9_flag, motors, inputs, controller],
    //     local = [tim3, sensors],
    //     priority = 4
    // )]
    fn timer_sensors(mut cx: timer_sensors::Context) {
        cx.local
            .tim3
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);
        let ahrs = cx.shared.ahrs;
        let flight_data = cx.shared.flight_data;
        let sens_data = cx.shared.sens_data;
        let tim9_flag = cx.shared.tim9_flag;
        let motors = cx.shared.motors;
        let inputs = cx.shared.inputs;
        let controller = cx.shared.controller;

        /// update sensors, ahrs
        (
            ahrs,
            sens_data,
            flight_data,
            tim9_flag,
            motors,
            inputs,
            controller,
        )
            .lock(|ahrs, sd, fd, tim9_flag, motors, inputs, controller| {
                // rprintln!("timer_sensors lock 0");

                /// Read sensor data
                cx.local.sensors.read_data_mag(sd);
                cx.local.sensors.read_data_imu(sd, false);
                cx.local.sensors.read_data_baro(sd);

                /// update AHRS
                let gyro0 = sd.imu_gyro.read_and_reset();
                let acc0 = sd.imu_acc.read_and_reset();
                let mag0 = sd.magnetometer.read_and_reset();

                // let pressure = sd.baro_pressure.read_and_reset();
                // let temp = sd.baro_temperature.read_and_reset();
                // let alt = ahrs.update_baro(pressure, temp);
                // rprintln!("alt = {:?}", alt);

                // if ahrs.calibration.initializing {
                //     ahrs.calibration.update(gyro0, acc0, mag0);
                // }

                let gyro1 = ahrs.calibration.calibrate_gyro(gyro0);
                let acc = ahrs.calibration.calibrate_acc(acc0);
                let mag = ahrs.calibration.calibrate_mag(mag0);

                // let gyro = gyro0;
                // let acc = acc0;
                // let mag = mag0;

                let gyro = ahrs.offset.update(gyro1);
                // let gyro = gyro1;

                #[cfg(feature = "nope")]
                {
                    // print_v3("gyro = ", gyro, 4);
                    // print_v3("acc  = ", acc, 4);
                    // print_v3("mag  = ", mag, 6);

                    // print_v3("mag0 = ", mag0, 6);

                    // use na::RealField;
                    // let heading = rad_to_deg(f32::atan2(mag.y, mag.x));
                    // rprintln!(
                    //     "heading = {:?}, mag(x,y,z) = ({:?}, {:?}, {:?})",
                    //     round_to(heading, 1),
                    //     round_to(mag.x, 6),
                    //     round_to(mag.y, 6),
                    //     round_to(mag.z, 6),
                    // );

                    // /// expected magnetic field:
                    // /// Declination (+E) = 15.9 deg
                    // /// Inclination (+D) = 70.1 deg
                    // /// total:      53_743 nT
                    // /// +North:     17_611 nT
                    // /// +East:      5_028  nT
                    // /// +Vertical:  50_527 nT
                    // let mag_strength = mag.magnitude();
                    // let mag_horiz = {
                    //     let v = V3::new(mag.x, mag.y, 0.0);
                    //     v.magnitude()
                    // };

                    // rprintln!(
                    //     "mag total: [{=f32:08}]\nhorizontal = {=f32:08}\n{=f32:08}, {=f32:08}, {=f32:08}",
                    //     mag_strength,
                    //     mag_horiz,
                    //     r(mag.x),
                    //     r(mag.y),
                    //     r(mag.z)
                    // );
                }

                ahrs.update(gyro, acc, mag);
                // ahrs.update_no_mag(gyro, acc);

                /// update FlightData
                fd.update(ahrs);

                /// update PIDs
                let motor_outputs = controller.update(*inputs, &fd.quat, gyro);

                /// apply mixed PID outputs to motors
                motor_outputs.apply(motors);

                let (roll, pitch, yaw) = fd.get_euler_angles();

                // rprintln!(
                //     "(r,p,y) = {:?}, {:?}, {:?}",
                //     r(rad_to_deg(roll)),
                //     r(rad_to_deg(pitch)),
                //     r(rad_to_deg(yaw)),
                // );

                use na::{ComplexField, RealField};
                let heading = rad_to_deg(f32::atan2(mag.y, mag.x));

                let xh = mag.x * pitch.cos() + mag.y * pitch.sin() * roll.sin()
                    - mag.z * roll.cos() * pitch.sin();
                let yh = mag.y * roll.cos() + mag.z * roll.sin();

                let heading2 = rad_to_deg(f32::atan2(yh, xh));

                // let heading2 = AhrsFusion::compass_calc_heading(acc, mag);

                // let heading2 =
                // let heading2 = rad_to_deg()

                // rprintln!(
                //     "yaw = {:?}\nheading  = {:?}\nheading2 = {:?}",
                //     rad_to_deg(yaw),
                //     heading,
                //     heading2
                // );

                // let q = fd.quat.as_ref();

                // use na::{Quaternion, Vector2};
                // let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
                // let b = Quaternion::new(0.0, Vector2::new(h[0], h[1]).norm(), 0.0, h[2]);
                // let b = UQuat::from_quaternion(b);

                // fn f(x: f32) -> f32 {
                //     (x * 10.0).round() / 10.0
                // }

                // rprintln!(
                //     "heading = {=f32:04}, yaw = {=f32:04}",
                //     f(heading),
                //     f(rad_to_deg(yaw))
                // );

                *tim9_flag = true;
                // rprintln!("timer_sensors lock 1");
            });
    }

    #[task(
        binds = TIM3,
        shared = [ahrs, sens_data, flight_data, tim9_flag, motors, inputs, controller],
        local = [tim3, sensors],
        priority = 4
    )]
    fn timer_sensors(mut cx: timer_sensors::Context) {
        cx.local
            .tim3
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);

        /// update sensors, ahrs
        (
            cx.shared.ahrs,
            cx.shared.sens_data,
            cx.shared.flight_data,
            cx.shared.tim9_flag,
            cx.shared.motors,
            cx.shared.inputs,
            cx.shared.controller,
        )
            .lock(|ahrs, sd, fd, tim9_flag, motors, inputs, controller| {
                /// Read sensor data
                cx.local.sensors.read_data_mag(sd);
                cx.local.sensors.read_data_imu(sd, false);
                cx.local.sensors.read_data_baro(sd);

                /// update AHRS
                let gyro = sd.imu_gyro.read_and_reset();
                let acc = sd.imu_acc.read_and_reset();
                let mag = sd.magnetometer.read_and_reset();

                // let m_ref = V3::new(
                //     17611.0, //
                //     5026.6,  //
                //     50522.0, //
                // );

                // let mag = mag - ahrs.calibration.hard_iron_offset;
                // print_v3("mag   = ", mag, 6);
                // print_v3("mag_n = ", mag.normalize(), 6);
                // print_v3("m_ref = ", m_ref.normalize(), 6);

                ahrs.update(gyro, acc, mag);

                /// update FlightData
                fd.update(ahrs);

                // /// update PIDs
                // let motor_outputs = controller.update(*inputs, &fd.quat, gyro);

                // /// apply mixed PID outputs to motors
                // motor_outputs.apply(motors);

                let (roll, pitch, yaw) = fd.get_euler_angles();
                rprintln!(
                    "(r,p,y) = {:?}, {:?}, {:?}",
                    r(rad_to_deg(roll)),
                    r(rad_to_deg(pitch)),
                    r(rad_to_deg(yaw)),
                );

                *tim9_flag = true;
            });
    }

    #[task(
        shared = [bt, exti, flight_data, sens_data, dwt, adc, tim9_flag, inputs, controller],
        local = [counter: u32 = 0, bat_counter: u32 = 0, buf: [u8; 16] = [0; 16]],
        priority = 3
    )]
    fn main_loop(mut cx: main_loop::Context) {
        // const COUNTER_TIMES: u32 = 10; // 200 hz => 20 hz
        // const COUNTER_TIMES: u32 = 5; // 100 hz => 20 hz
        const COUNTER_TIMES: u32 = 30; // 800 hz => 26.7 hz

        let flight_data = cx.shared.flight_data;
        let sens_data = cx.shared.sens_data;
        let bt = cx.shared.bt;
        let exti = cx.shared.exti;
        // let inputs = cx.shared.inputs;
        let controller = cx.shared.controller;

        // *cx.local.bat_counter += 1;
        // if *cx.local.bat_counter

        /// send telemetry over bluetooth, at 20-30 Hz
        cx.shared.tim9_flag.lock(|tim9_flag| {
            if *tim9_flag {
                *tim9_flag = false;

                *cx.local.counter += 1;
                if *cx.local.counter >= COUNTER_TIMES {
                    *cx.local.counter = 0;
                    (flight_data, sens_data, bt, exti, controller).lock(
                        |fd, sd, bt, exti, controller| {
                            // bt.pause_interrupt(exti);
                            // bt.log_write_quat(&fd.quat).unwrap();
                            // bt.unpause_interrupt(exti);

                            let gyro0 = sd.imu_gyro.read_and_reset();
                            let acc0 = sd.imu_acc.read_and_reset();
                            let mag0 = sd.magnetometer.read_and_reset();

                            // rprintln!("0");
                            bt.pause_interrupt(exti);
                            bt.log_write_sens(gyro0, acc0, mag0).unwrap();
                            bt.unpause_interrupt(exti);
                            // rprintln!("1");

                            // // rprintln!("writing PID");
                            // for id in [IdPID::RollRate] {
                            //     bt.pause_interrupt(exti);
                            //     bt.log_write_pid(id, &controller[id]).unwrap();
                            //     bt.unpause_interrupt(exti);
                            // }

                            // #[cfg(feature = "nope")]
                            // for id in IdPID::ITER {
                            //     bt.pause_interrupt(exti);
                            //     bt.log_write_pid(id, &controller[id]).unwrap();
                            //     bt.unpause_interrupt(exti);
                            // }

                            // //
                            // unimplemented!()
                        },
                    );
                }
            }
        });

        // main_loop::spawn_after(100.millis()).unwrap();
        main_loop::spawn().unwrap();
    }

    #[cfg(feature = "nope")]
    // #[task(
    //     // binds = TIM3,
    //     // shared = [bt, exti, dwt, inputs, controller],
    //     shared = [bt, exti, dwt, ahrs, sens_data, flight_data, motors, tim9_flag, inputs, controller],
    //     local = [x: f32 = 0.0, tim3, sensors],
    //     priority = 3)
    // ]
    fn bt_test(mut cx: bt_test::Context) {
        #[cfg(feature = "nope")]
        (cx.shared.bt, cx.shared.exti, cx.shared.controller).lock(
            |bt, exti, controller| {
                controller.pid_altitude_rate.step(-1.0);

                rprintln!("sending: {:?}", controller.pid_altitude_rate);
                bt.pause_interrupt(exti);
                bt.log_write_pid(IdPID::AltitudeRate, &controller.pid_altitude_rate)
                    .unwrap();
                bt.unpause_interrupt(exti);

                #[cfg(feature = "nope")]
                if let BTState::Connected(conn) = bt.state {
                    // let handle = match bt.services.input {
                    //     Some(i) => i.throttle_char,
                    //     _ => panic!("no throttle handle"),
                    // };

                    // let quat = UQuat::from_euler_angles(deg_to_rad(*cx.local.x), 0.0, 0.0);
                    // *cx.local.x += 1.0;
                    // rprintln!("cx.local.x = {:?}", *cx.local.x);

                    // // rprintln!("sending");
                    // bt.log_write_quat(&quat).unwrap();

                    controller.pid_altitude_rate.step(-1.0);

                    rprintln!("sending: {:?}", controller.pid_altitude_rate);
                    bt.pause_interrupt(exti);
                    bt.log_write_pid(IdPID::AltitudeRate, &controller.pid_altitude_rate)
                        .unwrap();
                    bt.unpause_interrupt(exti);

                    // bt.read_event_uart().unwrap();
                    // rprintln!("finished read");
                }
            },
        );

        let ahrs = cx.shared.ahrs;
        let flight_data = cx.shared.flight_data;
        let sens_data = cx.shared.sens_data;
        let tim9_flag = cx.shared.tim9_flag;
        let motors = cx.shared.motors;
        let inputs = cx.shared.inputs;
        let controller = cx.shared.controller;

        cx.local
            .tim3
            .clear_interrupt(stm32f4xx_hal::timer::Event::Update);

        cx.shared.dwt.lock(|dwt| {
            let t = dwt.measure(|| {
                (
                    ahrs,
                    sens_data,
                    flight_data,
                    tim9_flag,
                    motors,
                    inputs,
                    controller,
                )
                    .lock(
                        |ahrs, sd, fd, tim9_flag, motors, inputs, controller| {
                            // (cx.shared.dwt, cx.shared.controller, cx.shared.inputs).lock(
                            // |dwt, controller, inputs| {
                            //

                            // XXX: takes 50 us without, reading data
                            // XXX: or 204 with

                            // cx.local.sensors.read_data_mag(sd); // 15-30 us
                            // cx.local.sensors.read_data_imu(sd, false); // 110 us

                            // cx.local.sensors.read_data_baro(sd); // 10-20 us

                            // for _ in 0..15 {
                            //     cx.local.sensors.read_data_mag(sd); // 15-30 us
                            // }

                            // for _ in 0..5 {
                            //     cx.local.sensors.read_data_imu(sd, false); // 110 us
                            // }

                            // let gyro = sd.imu_gyro.read_and_reset();
                            // let acc = sd.imu_acc.read_and_reset();
                            // let mag = sd.magnetometer.read_and_reset();

                            let gyro_acc = cx.local.sensors._read_data_imu(); // 110 us
                            let mag: Option<V3> = cx.local.sensors._read_data_mag();

                            // let gyro = [0.0, 0.0, 0.0].into();
                            // let acc = [0.0, 0.0, 0.0].into();
                            // let mag = [0.0, 0.0, 0.0].into();

                            match (gyro_acc, mag) {
                                (Some((gyro, acc)), Some(mag)) => {
                                    rprintln!("wat 0");
                                    ahrs.update(gyro, acc, mag);

                                    /// update FlightData
                                    fd.update(ahrs);

                                    /// update PIDs
                                    let motor_outputs =
                                        controller.update(*inputs, &fd.quat, gyro);
                                }
                                (Some((gyro, _)), _) => {
                                    rprintln!("wat 1");
                                    /// update PIDs
                                    let motor_outputs =
                                        controller.update(*inputs, &fd.quat, gyro);
                                }
                                _ => {
                                    rprintln!("wat 2");
                                }
                            }

                            // let t = dwt.measure(|| {
                            //     let out0_roll = controller.pid_roll_stab.step(err0_roll);
                            //     let out1_roll = controller.pid_pitch_stab.step(err0_roll);
                            //     let out2_roll = controller.pid_yaw_stab.step(err0_roll);
                            //     let k = out0_roll + out1_roll + out2_roll;
                            //     defmt::trace!("k: {:?}", k);
                            // });

                            // rprintln!("t = {:?} ns", t.as_nanos());
                        },
                    );
            });

            rprintln!("t = {:?} ns", t.as_nanos());
        });

        // bt_test::spawn_after(250.millis()).unwrap();
    }

    #[task(binds = EXTI4, shared = [bt, exti, controller, motors, inputs, leds], priority = 8)]
    fn bt_irq(mut cx: bt_irq::Context) {
        (
            cx.shared.bt,
            cx.shared.exti,
            cx.shared.controller,
            cx.shared.motors,
            cx.shared.inputs,
        )
            .lock(|bt, exti, controller, motors, inputs| {
                rprintln!("bt_irq");

                bt.clear_interrupt();
                bt.pause_interrupt(exti);

                loop {
                    let event: BTEvent = match bt._read_event() {
                        Ok(ev) => {
                            rprintln!("ev = {:?}", ev);
                            ev
                        }
                        Err(e) => {
                            rprintln!("read event error = {:?}", e);
                            unimplemented!()
                        }
                    };

                    // /// TODO: update connection params
                    // match bt.state.handle_connect_disconnect(&event) {
                    //     Some(ConnectionChange::NewConnection(conn_handle)) => {
                    //         // let conn_interval = ConnectionIntervalBuilder::new()
                    //         //     .with_latency()
                    //         //     .build()
                    //         //     .unwrap();
                    //         // let params = ConnectionUpdateParameters {
                    //         //     conn_handle,
                    //         //     conn_interval,
                    //         //     expected_connection_length:
                    //         //         ExpectedConnectionLength::new(
                    //         //             core::time::Duration::from_secs_f32(1.0),
                    //         //             core::time::Duration::from_secs_f32(1200.0), // 20 minutes
                    //         //         )
                    //         //         .unwrap(),
                    //         // };
                    //         // // block!(bt.le_connection_update(params)).unwrap();
                    //         // block!(bt.(params)).unwrap();
                    //         // bt.read_event_uart().unwrap();
                    //     }
                    //     _ => {}
                    // }

                    bt.handle_connect_disconnect(&event);

                    bt.handle_input(motors, inputs, controller, &event);

                    if !bt.data_ready().unwrap() {
                        break;
                    }
                }

                bt.unpause_interrupt(exti);
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
    struct Local {}

    #[cfg(feature = "nope")]
    // #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut cp: stm32f401::CorePeripherals = cx.core;
        let mut dp: stm32f401::Peripherals = cx.device;

        let mut rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(16.MHz()).sysclk(84.MHz()).freeze();

        dp.TIM4.cr1.modify(|r, w| {
            w.dir()
                .up() // direction
                .cms()
                .edge_aligned() //
                .ckd()
                .div1() // clock division
                .udis()
                .enabled() // update generation
        });

        // dp.TIM4.cr1.modify(|r, w| {
        //     w.mms()
        //         .
        // });

        // TODO: TRGO Reset mode

        /// auto-reload value
        dp.TIM4.arr.modify(|r, w| w.arr().bits(1999));

        /// prescaler value
        dp.TIM4.psc.modify(|r, w| w.psc().bits(84));

        dp.TIM4.smcr.modify(|r, w| {
            w.msm()
                .no_sync() // slave mode disabled
                .ts()
                .itr0() // internal trigger 0
                .etp()
                .not_inverted() // ETR noninverted
                .ece()
                .disabled() // external clock mode 2
                .etps()
                .div1() // external trigger prescaler disabled
                .etf()
                .no_filter() // external trigger filter disabled
        });

        dp.TIM4.ccer.modify(|r, w| {
            w.cc1np()
                .clear_bit() // CC1 channel configured as output
                .cc1p()
                .clear_bit() //CC1 active high
        });

        // TODO: output compare PWM mode 1
        dp.TIM4.ccmr1_output().modify(|r, w| {
            w.oc1pe()
                .enabled() // OC1 preload register enabled
                .oc1fe()
                .set_bit() // fast mode enable
                .cc1s()
                .output() // C1 channel configured as output
        });

        (Shared {}, Local {}, init::Monotonics())
    }

    #[allow(unreachable_code)]
    // #[cfg(feature = "nope")]
    #[init]
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

        let pwm = 0.25;

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

        delay.delay(2000.millis());

        // motors.disable_motor(m0);
        // motors.disable_motor(m1);
        motors.disable_all();
        rprintln!("wat 1");

        (Shared {}, Local {}, init::Monotonics())
    }

    #[task(shared = [], local = [])]
    fn foo(cx: foo::Context) {
        foo::spawn().unwrap();
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
