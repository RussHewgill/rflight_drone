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

    /// Gravitational Reference Vector
    a_ref: V3,
    /// Magnetic Reference Vector, normalized
    m_ref: V3,
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

            /// NED
            a_ref: V3::new(0.0, 0.0, -9.81),
            m_ref: V3::zeros(),
        }
    }
    pub fn set_mag_ref(&mut self, m_ref: V3) {
        self.m_ref = m_ref.normalize();
    }
}

/// omega, dfdq, dhdq, f, h
#[cfg(feature = "nope")]
impl AhrsExtKalman {
    fn omega(x: V3) -> Mat4 {
        Mat4::from([
            [0.0, -x[0], -x[1], -x[2]],
            [x[0], 0.0, x[2], -x[1]],
            [x[1], -x[2], 0.0, x[0]],
            [x[2], x[1], -x[0], 0.0],
        ])
    }

    /// omega: Angular velocity in rad/s.
    /// dt: Time step, in seconds, between consecutive Quaternions.
    fn dfdq(omega: V3, dt: f32) -> Mat4 {
        let x = 0.5 * dt * omega;
        Mat4::identity() + Self::omega(x)
    }

    fn dhdq(&self, quat: UQuat, normal_mode: bool) -> na::Matrix6x4<f32> {
        if !normal_mode {
            panic!();
        }

        let v = na::Vector6::new(
            self.a_ref.x,
            self.a_ref.y,
            self.a_ref.z,
            self.m_ref.x,
            self.m_ref.y,
            self.m_ref.z,
        );

        let q = quat.coords;

        let hh = na::Matrix4x6::from([
            [
                -q.x * v[2] + q.x * v[1],
                q.x * v[1] + q.x * v[2],
                -q.x * v[2] + q.x * v[1] - 2.0 * q.x * v[0],
                q.x * v[1] + q.x * v[2] - 2.0 * q.x * v[0],
            ],
            [
                q.x * v[2] - q.x * v[0],
                q.x * v[2] - 2.0 * q.x * v[1] + q.x * v[0],
                q.x * v[0] + q.x * v[2],
                -q.x * v[0] + q.x * v[2] - 2.0 * q.x * v[1],
            ],
            [
                -q.x * v[1] + q.x * v[0],
                -q.x * v[1] - 2.0 * q.x * v[2] + q.x * v[0],
                q.x * v[0] - 2.0 * q.x * v[2] + q.x * v[1],
                q.x * v[0] + q.x * v[1],
            ],
            [
                -q.x * v[5] + q.x * v[4],
                q.x * v[4] + q.x * v[5],
                -q.x * v[5] + q.x * v[4] - 2.0 * q.x * v[3],
                q.x * v[4] + q.x * v[5] - 2.0 * q.x * v[3],
            ],
            [
                q.x * v[5] - q.x * v[3],
                q.x * v[5] - 2.0 * q.x * v[4] + q.x * v[3],
                q.x * v[3] + q.x * v[5],
                -q.x * v[3] + q.x * v[5] - 2.0 * q.x * v[4],
            ],
            [
                -q.x * v[4] + q.x * v[3],
                -q.x * v[4] - 2.0 * q.x * v[5] + q.x * v[3],
                q.x * v[3] - 2.0 * q.x * v[5] + q.x * v[4],
                q.x * v[3] + q.x * v[4],
            ],
        ]);

        2.0 * hh.transpose()
    }

    fn f(quat: UQuat, omega: V3, dt: f32) -> UQuat {
        let omega_t = Self::omega(omega);
        let m: Mat4 = Mat4::identity() + 0.5 * dt * omega_t;
        let x = m * quat.coords;
        UQuat::from_quaternion(Quaternion::from(x))
    }

    // fn h(quat: UQuat) -> (V3, V3) {
    fn h(&self, quat: UQuat) -> na::Vector6<f32> {
        // gx(0.5−q2y−q2z)+gy(qwqz+qxqy)+gz(qxqz−qwqy)
        // gx(qxqy−qwqz)+gy(0.5−q2x−q2z)+gz(qwqx+qyqz)
        // gx(qwqy+qxqz)+gy(qyqz−qwqx)+gz(0.5−q2x−q2y)
        // rx(0.5−q2y−q2z)+ry(qwqz+qxqy)+rz(qxqz−qwqy)
        // rx(qxqy−qwqz)+ry(0.5−q2x−q2z)+rz(qwqx+qyqz)
        // rx(qwqy+qxqz)+ry(qyqz−qwqx)+rz(0.5−q2x−q2y)

        let c = quat.to_rotation_matrix().transpose();

        let a = c * self.a_ref;
        let m = c * self.m_ref;

        na::Vector6::new(
            a.x, a.y, a.z, //
            m.x, m.y, m.z, //
        )
    }

    // /// Direction Cosine Matrix from given quaternion.
    // fn q2r(q: UQuat) -> () {
    //     unimplemented!()
    // }
}

impl AhrsExtKalman {
    fn update_strapdown_equations_NED(&mut self) {
        unimplemented!()
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

#[cfg(feature = "nope")]
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
        let y: na::Vector6<f32> = self.h(q_t);

        /// Innovation (Measurement Residual)
        let v: na::Vector6<f32> = z - y;

        /// Linearized Measurement Matrix
        let hh: na::Matrix6x4<f32> = self.dhdq(q_t, true);

        // TODO: self.rr
        /// Measurement Prediction Covariance
        // let ss = h * pp_t * hh.transpose() + self.rr;
        let ss: na::Matrix6<f32> = hh * pp_t * hh.transpose();

        let ss2 = if let Some(m) = ss.try_inverse() {
            m
        } else {
            // ss.try_inverse().unwrap()
            rprintln!("can't invert ss = {:?}", defmt::Debug2Format(&ss));
            return;
        };

        /// Kalman Gain
        // let kk: na::Matrix4x6<f32> = pp_t * hh.transpose() * ss.try_inverse().unwrap();
        let kk: na::Matrix4x6<f32> = pp_t * hh.transpose() * ss2;

        self.pp = (Mat4::identity() - kk * hh) * pp_t;

        // let q = q_t.coords + kk * v;

        self.quat = UQuat::from_quaternion(q_t.as_ref() + Quaternion::from(kk * v));
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
