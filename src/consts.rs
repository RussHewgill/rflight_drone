use core::f32::consts::PI;

use fugit::{HertzU32, MillisDurationU32, RateExtU32};

pub const SENSOR_FREQ: HertzU32 = HertzU32::Hz(6660);
// pub const SENSOR_FREQ: HertzU32 = HertzU32::Hz(10);

pub const PID_FREQ: HertzU32 = HertzU32::Hz(3330);
// pub const PID_PERIOD: HertzU32 = HertzU32::Hz(6660);

pub const MAIN_LOOP_FREQ: HertzU32 = HertzU32::Hz(15);
// pub const MAIN_LOOP_FREQ: HertzU32 = HertzU32::Hz(100);
