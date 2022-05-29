use core::f32::consts::PI;

use fugit::HertzU32;
use nalgebra::{self as na, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

use super::{Rot3, UQuat, AHRS, V3};
use crate::{math::*, utils::print_v3};
use defmt::println as rprintln;

pub type Mat4 = na::Matrix4<f32>;

#[derive(Debug, Clone, Copy)]
pub struct AhrsExtKalman {
    quat:       UQuat,
    delta_time: f32,

    gain: f32,

    pp: Mat4,
}

/// new
impl AhrsExtKalman {
    pub fn new(sample_rate: HertzU32, gain: f32) -> Self {
        let delta_time = 1.0 / (sample_rate.to_Hz() as f32);
        Self {
            quat: UQuat::default(),
            delta_time,

            gain,

            pp: Mat4::identity(),
        }
    }
}

/// omega, dfdq, dhdq, f, h
impl AhrsExtKalman {
    fn omega(x: V3) -> Mat4 {
        unimplemented!()
    }

    fn dfdq(omega: V3, dt: f32) -> Mat4 {
        unimplemented!()
    }

    fn dhdq(quat: UQuat, normal_mode: bool) -> na::Matrix6x4<f32> {
        unimplemented!()
    }

    fn f(quat: UQuat, omega: V3, dt: f32) -> Mat4 {
        unimplemented!()
    }

    // fn h(quat: UQuat) -> (V3, V3) {
    fn h(quat: UQuat) -> na::Vector6<f32> {
        unimplemented!()
    }
}

impl AHRS for AhrsExtKalman {
    fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
        /// * Prediction

        /// Predicted State
        let q_t = Self::f(self.quat, gyro, self.delta_time);
        /// Linearized Fundamental Matrix
        let f = Self::dfdq(gyro, self.delta_time);

        /// Jacobian W = df/dω
        // W   = 0.5*dt * np.r_[[-q[1:]], q[0]*np.identity(3) + skew(q[1:])]
        let w: na::Matrix4x3<f32> = {
            let q = self.quat.coords;
            let w = na::Matrix3x4::from([
                [-q.x, -q.y, -q.z], //
                [q.w, -q.z, q.y],   //
                [q.z, q.w, -q.x],
                [-q.y, q.x, q.w],
            ])
            .transpose();
            0.5 * self.delta_time * w
        };

        let g_noise = Mat4::identity();

        // TODO: self.g_noise
        /// Process Noise Covariance
        let qq_t: Mat4 = 0.5 * self.delta_time * g_noise * w * w.transpose();

        /// Predicted Covariance Matrix
        let pp_t: Mat4 = f * self.pp * f.transpose() + qq_t;

        /// * Correction
        let mag_n = mag.normalize();
        let z: na::Vector6<f32> =
            na::Vector6::new(acc.x, acc.y, acc.z, mag_n.x, mag_n.y, mag_n.z);

        /// Expected Measurement function
        // let (y_acc, y_mag) = Self::h(self.quat);
        let y: na::Vector6<f32> = Self::h(self.quat);

        /// Innovation (Measurement Residual)
        let v: na::Vector6<f32> = z - y;

        /// Linearized Measurement Matrix
        let hh: na::Matrix6x4<f32> = Self::dhdq(self.quat, true);

        // TODO: self.rr
        /// Measurement Prediction Covariance
        // let ss = h * pp_t * hh.transpose() + self.rr;
        let ss: na::Matrix6<f32> = hh * pp_t * hh.transpose();

        /// Kalman Gain
        let kk: na::Matrix4x6<f32> = pp_t * hh.transpose() * ss.try_inverse().unwrap();

        self.pp = (Mat4::identity() - kk * hh) * pp_t;

        // let q = q_t + kk * v;

        // self.quat =

        unimplemented!()
    }

    fn get_quat(&self) -> &UQuat {
        &self.quat
    }
}

pub fn skew(x: V3) -> na::Matrix3<f32> {
    [
        [0.0, -x.z, x.y], //
        [x.z, 0.0, -x.x], //
        [-x.y, x.x, 0.0],
    ]
    .into()
}
