use core::f32::consts::PI;

use fugit::{HertzU32, MillisDurationU32, RateExtU32};

pub const SENSOR_FREQ: HertzU32 = HertzU32::Hz(6660);
// pub const SENSOR_FREQ: HertzU32 = HertzU32::Hz(10);

pub const PID_FREQ: HertzU32 = HertzU32::Hz(3330);
// pub const PID_PERIOD: HertzU32 = HertzU32::Hz(6660);

pub const MAIN_LOOP_FREQ: HertzU32 = HertzU32::Hz(15);
// pub const MAIN_LOOP_FREQ: HertzU32 = HertzU32::Hz(20);

pub const SENSOR_FREQ_GYRO: HertzU32 = HertzU32::Hz(6660);
pub const SENSOR_FREQ_ACC: HertzU32 = HertzU32::Hz(6660);
pub const SENSOR_FREQ_MAG: HertzU32 = HertzU32::Hz(100);
pub const SENSOR_FREQ_BARO: HertzU32 = HertzU32::Hz(75);

pub mod pid_vals {
    // /// set 1
    // pub const PID_PITCH_RATE_P: f32 = 0.000_06;
    // pub const PID_PITCH_RATE_I: f32 = 0.000_185;
    // pub const PID_PITCH_RATE_D: f32 = 0.000_004_9;

    /// set 2: ZN
    /// ku = 0.0000_9;
    /// tu = 1.6;
    pub const PID_PITCH_RATE_P: f32 = 0.000_054;
    pub const PID_PITCH_RATE_I: f32 = 0.000_0675;
    pub const PID_PITCH_RATE_D: f32 = 0.000_0108;
}
