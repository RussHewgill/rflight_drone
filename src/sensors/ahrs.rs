// use ahrs::{Ahrs, Madgwick};
use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

use crate::{math::*, uart::*, uprintln};

use super::{UQuat, V3};

// pub use self::madgwick_prev::*;

#[derive(Debug, Clone)]
pub struct AHRS {
    quat:          UQuat,
    sample_period: f32,
    beta:          f32,

    cfg_gain:              f32,
    cfg_acc_rejection:     f32,
    cfg_mag_rejection:     f32,
    cfg_rejection_timeout: u32,

    initializing:     bool,
    ramped_gain:      f32,
    ramped_gain_step: f32,

    half_acc_feedback: V3,
    half_mag_feedback: V3,

    acc_ignored:           bool,
    acc_rejection_timer:   u32,
    acc_rejection_timeout: bool,
    mag_ignored:           bool,
    mag_rejection_timer:   u32,
    mag_rejection_timeout: bool,
}

/// new
impl AHRS {
    pub const INIT_TIME: f32 = 3.0;
    pub const INITIAL_GAIN: f32 = 10.0;

    pub fn new(sample_period: f32, beta: f32) -> Self {
        Self {
            quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            sample_period,
            beta,

            cfg_gain: 0.5,
            cfg_acc_rejection: 90.0,
            cfg_mag_rejection: 90.0,
            cfg_rejection_timeout: 0,

            initializing: true,
            ramped_gain: Self::INITIAL_GAIN,
            ramped_gain_step: (Self::INITIAL_GAIN - beta) / Self::INIT_TIME,

            half_acc_feedback: V3::zeros(),
            half_mag_feedback: V3::zeros(),

            acc_ignored: false,
            acc_rejection_timer: 0,
            acc_rejection_timeout: false,
            mag_ignored: false,
            mag_rejection_timer: 0,
            mag_rejection_timeout: false,
        }
    }
}

/// update
impl AHRS {
    pub fn update_uart(
        &mut self,
        uart: &mut UART,
        gyro: V3,
        acc: V3,
        mag: V3,
    ) -> Option<&UQuat> {
        if self.initializing {
            self.ramped_gain -= self.ramped_gain_step * self.sample_period;

            if self.ramped_gain < self.cfg_gain {
                self.ramped_gain = self.cfg_gain;
                self.initializing = false;
                self.acc_rejection_timeout = false; // XXX: ?
            }
        }

        /// Calculate direction of gravity indicated by algorithm
        let half_gravity = {
            let q = self.quat.coords;
            V3::new(
                q.x * q.z - q.w * q.y,
                q.w * q.x + q.y * q.z,
                q.w * q.w - 0.5 + q.z * q.z,
            )
        };
        uprintln!(uart, "half_gravity = {:?}", half_gravity);

        // {
        //     let q = self.quat.as_ref();
        //     // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
        //     let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
        //     let b = Quaternion::new(0.0, Vector2::new(h[0], h[1]).norm(), 0.0, h[2]);
        //     uprintln!(uart, "b = {:?}", b);
        // }

        let mut half_acc_feedback = V3::zeros();
        self.acc_ignored = true;
        /// Calculate accelerometer feedback
        if !(acc == V3::zeros()) {
            /// Enter acceleration recovery state if acceleration rejection times out
            if self.acc_rejection_timer >= self.cfg_rejection_timeout {
                let q = self.quat;
                self.reset();
                self.quat = q;
                self.acc_rejection_timer = 0;
                self.acc_rejection_timeout = true;
            }

            // Calculate accelerometer feedback scaled by 0.5
            self.half_acc_feedback = acc.normalize().cross(&half_gravity);

            /// Ignore accelerometer if acceleration distortion detected
            if self.initializing
                || half_acc_feedback.norm_squared() <= self.cfg_acc_rejection
            {
                half_acc_feedback = self.half_acc_feedback;
                self.acc_ignored = false;
                if self.acc_rejection_timer >= 10 {
                    self.acc_rejection_timer -= 10;
                }
            } else {
                self.acc_rejection_timer += 1;
            }
        }

        let mut half_mag_feedback = V3::zeros();
        self.mag_ignored = true;
        /// Calculate magnetometer feedback
        if !(mag == V3::zeros()) {
            self.mag_rejection_timeout = false;
            /// Set to compass heading if magnetic rejection times out
            if self.mag_rejection_timer >= self.cfg_rejection_timeout {
                self.set_heading(Self::compass_calc_heading(acc, mag));
                self.mag_rejection_timer = 0;
                self.mag_rejection_timeout = true;
            }

            /// Compute direction of west indicated by algorithm
            let half_west = {
                let q = self.quat.coords;
                V3::new(
                    q.x * q.y + q.w * q.z,
                    q.w * q.w - 0.5 + q.y * q.y,
                    q.y * q.z - q.w * q.x,
                )
            };

            // Calculate magnetometer feedback scaled by 0.5
            self.half_mag_feedback =
                half_gravity.cross(&mag).normalize().cross(&half_west);

            // Ignore magnetometer if magnetic distortion detected
            if self.initializing
                || self.half_mag_feedback.norm_squared() <= self.cfg_mag_rejection
            {
                half_mag_feedback = self.half_mag_feedback;
                self.mag_ignored = false;
                if self.mag_rejection_timer >= 10 {
                    self.mag_rejection_timer -= 10;
                }
            } else {
                self.mag_rejection_timer += 1;
            }
        }

        /// XXX: ??
        /// Convert gyroscope to radians per second scaled by 0.5
        let half_gyro = gyro * deg_to_rad(0.5);

        /// Apply feedback to gyroscope
        let adjusted_half_gyro =
            half_gyro + ((half_acc_feedback + half_mag_feedback) * self.ramped_gain);

        /// Integrate rate of change of quaternion
        /// Normalise quaternion
        let v: V3 = adjusted_half_gyro * self.sample_period;
        self.quat = UQuat::from_quaternion(*self.quat + quat_mult_vec(*self.quat, v));

        Some(&self.quat)
    }

    pub fn set_heading(&mut self, heading: f32) {
        use nalgebra::{ComplexField, RealField}; // for sin, atan2

        let q = self.quat.coords;

        /// Euler angle of conjugate
        let inv_heading = f32::atan2(q.x * q.y + q.w * q.z, q.w * q.w - 0.5 + q.x * q.x);

        let half_inv_heading_minus_offset = 0.5 * (inv_heading - deg_to_rad(heading));

        let inv_heading_quat = Quaternion::from_parts(
            half_inv_heading_minus_offset.cos(),
            V3::new(0.0, 0.0, -1.0 * half_inv_heading_minus_offset.sin()),
        );

        self.quat = self.quat * UQuat::from_quaternion(inv_heading_quat);
    }

    pub fn compass_calc_heading(acc: V3, mag: V3) -> f32 {
        use nalgebra::RealField; // for atan2
        let magnetic_west = acc.cross(&mag).normalize();
        let magnetic_north = magnetic_west.cross(&acc).normalize();
        rad_to_deg(f32::atan2(magnetic_west.x, magnetic_north.x))
    }

    pub fn reset(&mut self) {
        self.quat = UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        self.initializing = true;
        self.ramped_gain = Self::INITIAL_GAIN;

        self.half_acc_feedback = V3::zeros();
        self.half_mag_feedback = V3::zeros();

        self.acc_ignored = false;
        self.acc_rejection_timer = 0;
        self.acc_rejection_timeout = false;
        self.mag_ignored = false;
        self.mag_rejection_timer = 0;
        self.mag_rejection_timeout = false;
    }
}

impl Default for AHRS {
    fn default() -> Self {
        let sample_period = 1.0 / 256.0;
        let beta = 0.1;
        Self::new(sample_period, beta)
    }
}

#[cfg(feature = "nope")]
mod madgwick_prev {
    use nalgebra::{Matrix6, Quaternion, UnitQuaternion, Vector2, Vector3, Vector6};

    use crate::{uart::*, uprintln};

    use super::{UQuat, V3};

    pub struct AHRS {
        sample_period: f32,
        beta:          f32,
        quat:          UQuat,
    }

    /// new
    impl AHRS {
        pub fn new(sample_period: f32, beta: f32) -> Self {
            Self {
                sample_period,
                beta,
                quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            }
        }
    }

    /// update
    impl AHRS {
        /// https://crates.io/crates/ahrs
        pub fn update_uart(
            &mut self,
            uart: &mut UART,
            gyro: V3,
            acc: V3,
            mag: V3,
        ) -> Option<&UQuat> {
            let acc = if let Some(acc) = acc.try_normalize(0.0) {
                acc
            } else {
                uprintln!(uart, "acc norm div by zero");
                return None;
            };

            let mag = if let Some(mag) = mag.try_normalize(0.0) {
                mag
            } else {
                uprintln!(uart, "mag norm div by zero");
                return None;
            };

            let q = self.quat.as_ref();

            /// Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
            let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
            let b = Quaternion::new(0.0, Vector2::new(h[0], h[1]).norm(), 0.0, h[2]);

            // Gradient descent algorithm corrective step
            let f = Vector6::new(
                2.0 * (q[0] * q[2] - q[3] * q[1]) - acc[0],
                2.0 * (q[3] * q[0] + q[1] * q[2]) - acc[1],
                2.0 * (0.5 - q[0] * q[0] - q[1] * q[1]) - acc[2],
                2.0 * b[0] * (0.5 - q[1] * q[1] - q[2] * q[2])
                    + 2.0 * b[2] * (q[0] * q[2] - q[3] * q[1])
                    - mag[0],
                2.0 * b[0] * (q[0] * q[1] - q[3] * q[2])
                    + 2.0 * b[2] * (q[3] * q[0] + q[1] * q[2])
                    - mag[1],
                2.0 * b[0] * (q[3] * q[1] + q[0] * q[2])
                    + 2.0 * b[2] * (0.5 - q[0] * q[0] - q[1] * q[1])
                    - mag[2],
            );

            let j_t = Matrix6::new(
                -2.0 * q[1],
                2.0 * q[0],
                0.0,
                -2.0 * b[2] * q[1],
                -2.0 * b[0] * q[2] + 2.0 * b[2] * q[0],
                2.0 * b[0] * q[1],
                2.0 * q[2],
                2.0 * q[3],
                -4.0 * q[0],
                2.0 * b[2] * q[2],
                2.0 * b[0] * q[1] + 2.0 * b[2] * q[3],
                2.0 * b[0] * q[2] - 4.0 * b[2] * q[0],
                -2.0 * q[3],
                2.0 * q[2],
                -4.0 * q[1],
                -4.0 * b[0] * q[1] - 2.0 * b[2] * q[3],
                2.0 * b[0] * q[0] + 2.0 * b[2] * q[2],
                2.0 * b[0] * q[3] - 4.0 * b[2] * q[1],
                2.0 * q[0],
                2.0 * q[1],
                0.0,
                -4.0 * b[0] * q[2] + 2.0 * b[2] * q[0],
                -2.0 * b[0] * q[3] + 2.0 * b[2] * q[1],
                2.0 * b[0] * q[0],
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            );

            let step = (j_t * f).normalize();

            // Compute rate of change for quaternion
            let q_dot = q * Quaternion::from_parts(0.0, gyro) * 0.5
                - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

            // Integrate to yield quaternion
            self.quat = UnitQuaternion::from_quaternion(q + q_dot * self.sample_period);

            Some(&self.quat)
        }
    }

    impl Default for AHRS {
        fn default() -> Self {
            let sample_period = 1.0 / 256.0;
            let beta = 0.1;
            Self::new(sample_period, beta)
        }
    }
}
