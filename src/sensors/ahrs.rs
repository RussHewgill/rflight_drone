use ahrs::{Ahrs, Madgwick};
use nalgebra::{UnitQuaternion, Vector3};

use super::{Quat, V3};

pub use self::madgwick::*;

// #[derive(Debug, Clone)]
// pub struct AHRS {
//     mag: Madgwick<f32>,
//     est_velocity: V3,
// }

// impl Default for AHRS {
//     fn default() -> Self {
//         let sample_period = 1.0 / 256.0;
//         let beta = 0.1;
//         Self {
//             mag:          Madgwick::new(sample_period, beta),
//             est_velocity: V3::new(0.0, 0.0, 0.0),
//         }
//     }
// }

// impl AHRS {
//     pub fn update(&mut self, gyro: V3, acc: V3, mag: V3) -> Option<&Quat> {
//         self.mag.update(&gyro, &acc, &mag).ok()
//     }
//     fn update_velocity(&mut self) {
//     }
// }

mod madgwick {
    use nalgebra::{Matrix6, Quaternion, UnitQuaternion, Vector2, Vector3, Vector6};

    use crate::{uart::*, uprintln};

    use super::{Quat, V3};

    pub struct AHRS {
        sample_period: f32,
        beta:          f32,
        quat:          Quat,
    }

    /// new
    impl AHRS {
        pub fn new(sample_period: f32, beta: f32) -> Self {
            Self {
                sample_period,
                beta,
                quat: Quat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
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
        ) -> Option<&Quat> {
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
