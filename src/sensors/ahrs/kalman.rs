use core::f32::consts::PI;

use fugit::HertzU32;
use nalgebra::{self as na, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

use super::{Rot3, UQuat, AHRS, V3};
use crate::{math::*, utils::print_v3};
use defmt::println as rprintln;

#[derive(Debug, Clone, Copy)]
pub struct AhrsExtKalman {
    quat:       UQuat,
    delta_time: f32,
}

impl AhrsExtKalman {
    pub fn new(sample_rate: HertzU32) -> Self {
        let delta_time = 1.0 / (sample_rate.to_Hz() as f32);
        Self {
            quat: UQuat::default(),
            delta_time,
        }
    }
}

impl AHRS for AhrsExtKalman {
    fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
        unimplemented!()
    }
    fn get_quat(&self) -> &UQuat {
        &self.quat
    }
}
