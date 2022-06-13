use core::f32::consts::PI;

use fugit::{HertzU32, MillisDurationU32, RateExtU32};

pub const SENSOR_PERIOD: HertzU32 = HertzU32::Hz(6660);
pub const PID_PERIOD: HertzU32 = HertzU32::Hz(6660);
pub const MAIN_LOOP_PERIOD: HertzU32 = HertzU32::Hz(15);
