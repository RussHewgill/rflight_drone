use core::f32::consts::PI;

use fugit::{HertzU32, MillisDurationU32, RateExtU32};

pub const SENSOR_FREQ: HertzU32 = HertzU32::Hz(6660);
// pub const SENSOR_FREQ: HertzU32 = HertzU32::Hz(10);

pub const PID_FREQ: HertzU32 = HertzU32::Hz(3330);
// pub const PID_PERIOD: HertzU32 = HertzU32::Hz(6660);

pub const MAIN_LOOP_FREQ: HertzU32 = HertzU32::Hz(15);
// pub const MAIN_LOOP_FREQ: HertzU32 = HertzU32::Hz(10);
// pub const MAIN_LOOP_FREQ: HertzU32 = HertzU32::Hz(20);

pub const SENSOR_FREQ_GYRO: HertzU32 = HertzU32::Hz(6660);
pub const SENSOR_FREQ_ACC: HertzU32 = HertzU32::Hz(6660);
pub const SENSOR_FREQ_MAG: HertzU32 = HertzU32::Hz(100);
pub const SENSOR_FREQ_BARO: HertzU32 = HertzU32::Hz(75);
