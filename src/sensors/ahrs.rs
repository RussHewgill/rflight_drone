// use ahrs::{Ahrs, Madgwick};
use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

use crate::{math::*, uart::*, uprintln};
use defmt::println as rprintln;

use super::{Rot3, UQuat, V3};

pub use self::complementary::*;
pub use self::fusion::*;
pub use self::madgwick_prev::*;

pub trait AHRS {
    fn update(&mut self, gyro: V3, acc: V3, mag: V3);

    fn get_quat(&self) -> &UQuat;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct FlightData {
    /// Rotation using North-West-Up convention
    pub quat: UQuat,
}

impl FlightData {
    pub fn update<T: AHRS>(&mut self, ahrs: &T) {
        self.quat = *ahrs.get_quat();
    }
    /// roll, pitch, yaw
    pub fn get_euler_angles(&self) -> (f32, f32, f32) {
        self.quat.euler_angles()
    }
}

mod complementary {
    use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

    use super::{UQuat, AHRS, V3};
    use crate::{math::*, uart::*, uprintln};
    use defmt::println as rprintln;

    #[derive(Debug, Clone, Copy)]
    pub struct AhrsComplementary {
        quat:          UQuat,
        sample_period: f32,
    }

    /// new
    impl AhrsComplementary {
        pub fn new(sample_period: f32) -> Self {
            Self {
                quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
                sample_period,
            }
        }
    }

    /// update
    impl AHRS for AhrsComplementary {
        fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
            unimplemented!()
        }

        fn get_quat(&self) -> &UQuat {
            &self.quat
        }
    }

    // impl AhrsComplementary {
    //     pub fn attitude_propagation(&self, gyro: V3) -> UQuat {
    //         unimplemented!()
    //     }
    //     pub fn am_estimation(&self, acc: V3, mag: V3) -> UQuat {
    //         unimplemented!()
    //     }
    // }
}

mod fusion {
    use core::f32::consts::PI;

    use nalgebra::{self as na, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

    use super::{Rot3, UQuat, AHRS, V3};
    use crate::{math::*, uart::*, uprintln};
    use defmt::println as rprintln;

    #[derive(Debug, Clone, Copy)]
    pub struct AhrsFusion {
        quat:       UQuat,
        delta_time: f32,

        pub offset:      FusionOffset,
        pub calibration: FusionCalibration,

        // prev_acc: V3,
        pub cfg_gain:              f32,
        pub cfg_acc_rejection:     f32,
        pub cfg_mag_rejection:     f32,
        pub cfg_rejection_timeout: u32,

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
    impl AhrsFusion {
        pub const INIT_TIME: f32 = 3.0;
        pub const INITIAL_GAIN: f32 = 10.0;

        pub fn new(delta_time: f32, gain: f32) -> Self {
            Self {
                // quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
                quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
                delta_time,

                offset: FusionOffset::default(),
                calibration: FusionCalibration::default(),

                cfg_gain: gain,
                cfg_acc_rejection: 10.0,
                cfg_mag_rejection: 20.0,

                cfg_rejection_timeout: (5.0 / delta_time) as u32, // 5 seconds

                initializing: true,
                ramped_gain: Self::INITIAL_GAIN,
                ramped_gain_step: (Self::INITIAL_GAIN - gain) / Self::INIT_TIME,

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

        pub fn reset(&mut self) {
            // self.offset.reset();
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

    impl AHRS for AhrsFusion {
        fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
            if self.initializing {
                rprintln!("initializing");
                self.ramped_gain -= self.ramped_gain * self.delta_time;
                if self.ramped_gain < self.cfg_gain {
                    rprintln!("initializing complete");
                    self.ramped_gain = self.cfg_gain;
                    self.initializing = false;
                    self.acc_rejection_timeout = false;
                }
            }

            // self.offset.update(self.delta_time, gyro, acc);
            // let gyro = gyro - self.offset.gyro_offset;
            // // let acc = acc - self.offset.acc_offset;
            // let mut acc = acc;
            // acc.x -= self.offset.acc_offset.x;
            // acc.y -= self.offset.acc_offset.y;
            // acc.z -= self.offset.acc_offset.z - 1000.0;
            // rprintln!(
            //     "self.offset.gyro_offset = {:?}",
            //     defmt::Debug2Format(&self.offset.gyro_offset)
            // );
            // rprintln!(
            //     "self.offset.acc_offset = {:?}",
            //     defmt::Debug2Format(&self.offset.acc_offset)
            // );

            /// Calculate direction of gravity indicated by algorithm
            let half_gravity = {
                let q = self.quat.coords;
                V3::new(
                    q.x * q.z - q.w * q.y,
                    q.w * q.x + q.y * q.z,
                    q.w * q.w - 0.5 + q.z * q.z,
                )
            };
            // rprintln!("half_gravity = {:?}", defmt::Debug2Format(&half_gravity));

            // /// equal to 3rd column of rotation matrix representation scaled by 0.5
            // let half_gravity2 = self.quat.to_rotation_matrix().matrix().column(2) * 0.5;
            // rprintln!("half_gravity2 = {:?}", defmt::Debug2Format(&half_gravity2));

            let mut half_acc_feedback = V3::zeros();
            self.acc_ignored = true;
            /// Calculate accelerometer feedback
            if acc != V3::zeros() {
                /// Enter acceleration recovery state if acceleration rejection times out
                if self.acc_rejection_timer >= self.cfg_rejection_timeout {
                    rprintln!("acc recovery");
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
                    rprintln!("ignoring acc, distorted");
                    half_acc_feedback = self.half_acc_feedback;
                    self.acc_ignored = false;

                    if self.acc_rejection_timer >= 10 {
                        self.acc_rejection_timer -= 10;
                    }
                    // self.acc_rejection_timer -= if self.acc_rejection_timer >= 10 {
                    //     10
                    // } else {
                    //     0
                    // };
                } else {
                    self.acc_rejection_timer += 1;
                }
            }

            let mut half_mag_feedback = V3::zeros();
            self.mag_ignored = true;
            /// Calculate magnetometer feedback
            if mag != V3::zeros() {
                self.mag_rejection_timeout = false;
                /// Set to compass heading if magnetic rejection times out
                if self.mag_rejection_timer >= self.cfg_rejection_timeout {
                    rprintln!("mag timeout, setting to compass heading");
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
                // rprintln!("half_west = {:?}", defmt::Debug2Format(&half_west));

                // /// equal to 2nd column of rotation matrix representation scaled by 0.5
                // let half_west2 = self.quat.to_rotation_matrix().matrix().column(1) * 0.5;
                // rprintln!("half_west2 = {:?}", defmt::Debug2Format(&half_west2));

                // Calculate magnetometer feedback scaled by 0.5
                self.half_mag_feedback =
                    half_gravity.cross(&mag).normalize().cross(&half_west);

                // Ignore magnetometer if magnetic distortion detected
                if self.initializing
                    || self.half_mag_feedback.norm_squared() <= self.cfg_mag_rejection
                {
                    rprintln!("ignoring mag, distorted");
                    half_mag_feedback = self.half_mag_feedback;
                    self.mag_ignored = false;
                    if self.mag_rejection_timer >= 10 {
                        self.mag_rejection_timer -= 10;
                    }
                } else {
                    self.mag_rejection_timer += 1;
                }
            }

            /// Convert gyroscope to radians per second scaled by 0.5
            let half_gyro = gyro * deg_to_rad(0.5);

            /// Apply feedback to gyroscope
            let adjusted_half_gyro =
                half_gyro + ((half_acc_feedback + half_mag_feedback) * self.ramped_gain);

            /// Integrate rate of change of quaternion
            /// Normalise quaternion
            let v: V3 = adjusted_half_gyro * self.delta_time;
            self.quat = UQuat::from_quaternion(*self.quat + quat_mult_vec(*self.quat, v));
        }

        fn get_quat(&self) -> &UQuat {
            &self.quat
        }
    }

    /// set_heading, compass
    impl AhrsFusion {
        pub fn set_heading(&mut self, heading: f32) {
            use nalgebra::{ComplexField, RealField}; // for sin, atan2

            let q = self.quat.coords;

            /// Euler angle of conjugate
            let inv_heading =
                f32::atan2(q.x * q.y + q.w * q.z, q.w * q.w - 0.5 + q.x * q.x);

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
    }

    #[derive(Debug, Default, Clone, Copy)]
    pub struct FusionCalibration {
        /// gyro
        pub gyro_misalignment: Rot3,
        pub gyro_sens:         V3,
        pub gyro_offset:       V3,
        /// acc
        pub acc_misalignment:  Rot3,
        pub acc_sens:          V3,
        pub acc_offset:        V3,
        /// mag
        pub soft_iron_rot:     Rot3,
        pub hard_iron_offset:  V3,
    }

    /// calibrate
    impl FusionCalibration {
        pub fn calibrate_gyro(&self, uncalibrated: V3) -> V3 {
            Self::_calibrate_imu(
                uncalibrated,
                self.gyro_misalignment,
                self.gyro_sens,
                self.gyro_offset,
            )
        }
        pub fn calibrate_acc(&self, uncalibrated: V3) -> V3 {
            Self::_calibrate_imu(
                uncalibrated,
                self.acc_misalignment,
                self.acc_sens,
                self.acc_offset,
            )
        }
        pub fn calibrate_mag(&self, uncalibrated: V3) -> V3 {
            Self::_calibrate_mag(uncalibrated, self.soft_iron_rot, self.hard_iron_offset)
        }

        fn _calibrate_imu(
            uncalibrated: V3,
            misalignment: na::Rotation3<f32>,
            sensitivity: V3,
            offset: V3,
        ) -> V3 {
            let v: V3 = (uncalibrated - offset).component_mul(&sensitivity);
            misalignment * v
        }

        fn _calibrate_mag(
            uncalibrated: V3,
            soft_iron_matrix: na::Rotation3<f32>,
            hard_iron_offset: V3,
        ) -> V3 {
            (soft_iron_matrix * uncalibrated) - hard_iron_offset
        }
    }

    #[derive(Debug, Default, Clone, Copy)]
    pub struct FusionOffset {
        filter_coef: f32,
        timeout:     u32,
        timer:       u32,
        gyro_offset: V3,
    }

    impl FusionOffset {
        /// Cutoff freq in Hz
        const CUTOFF_FREQ: f32 = 0.02;

        /// Timeout in seconds
        const TIMEOUT: u32 = 5;

        /// Threshold in degrees / second
        const THRESHOLD: f32 = 3.0;

        pub fn init(&mut self, sample_rate: u32) {
            self.filter_coef = 2.0 * PI * Self::CUTOFF_FREQ * (1.0 / sample_rate as f32);
            self.timeout = Self::TIMEOUT * sample_rate;
            self.timer = 0;
            self.gyro_offset = V3::zeros();
        }

        pub fn update(&mut self, gyro: V3) -> V3 {
            let gyro = gyro - self.gyro_offset;

            /// Reset timer if gyroscope not stationary
            if gyro.x > Self::THRESHOLD
                || gyro.y > Self::THRESHOLD
                || gyro.z > Self::THRESHOLD
            {
                self.timer = 0;
                return gyro;
            }

            /// Increment timer while gyroscope stationary
            if self.timer < self.timeout {
                self.timer += 1;
                return gyro;
            }

            // Adjust offset if timer has elapsed
            self.gyro_offset += gyro * self.filter_coef;

            gyro
        }
    }
}

#[cfg(feature = "nope")]
mod fusion {
    use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

    use super::{UQuat, AHRS, V3};
    use crate::{math::*, uart::*, uprintln};
    use defmt::println as rprintln;

    // #[derive(Debug, Default, Clone, Copy)]
    // pub struct FlightData {
    //     /// Rotation using North-West-Up convention
    //     pub quat:        UQuat,
    //     /// accelerometer measurement with the 1 g of gravity removed
    //     pub lin_accel:   V3,
    //     /// accelerometer measurement in the Earth coordinate frame with the 1 g of gravity removed
    //     pub earth_accel: V3,
    //     // pub prev_acc:     V3,
    //     // /// integrated accel to estimate velocity
    //     // pub est_velocity: V3,
    // }

    // impl FlightData {
    //     pub fn update(&mut self, ahrs: &AHRS) {
    //         self.quat = ahrs.get_quat();
    //         self.lin_accel = ahrs.get_linear_accel();
    //         self.earth_accel = ahrs.get_earth_accel();
    //         // self.prev_acc = ahrs.prev_acc;
    //         // self.est_velocity += self.lin_accel * ahrs.sample_period;
    //         // // self.est_velocity += self.earth_accel * ahrs.sample_period;
    //     }
    //     /// roll, pitch, yaw
    //     pub fn get_euler_angles(&self) -> (f32, f32, f32) {
    //         self.quat.euler_angles()
    //     }
    // }

    #[derive(Debug, Clone, Copy)]
    pub struct AhrsFusion {
        quat:          UQuat,
        sample_period: f32,
        beta:          f32,

        prev_acc: V3,

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
    impl AhrsFusion {
        pub const INIT_TIME: f32 = 3.0;
        pub const INITIAL_GAIN: f32 = 10.0;

        pub fn new(sample_period: f32, beta: f32) -> Self {
            Self {
                quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
                sample_period,
                beta,

                prev_acc: V3::zeros(),

                cfg_gain: 0.5,
                cfg_acc_rejection: 10.0,
                cfg_mag_rejection: 20.0,
                cfg_rejection_timeout: (5.0 / sample_period) as u32,

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

    impl AHRS for AhrsFusion {
        fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
            self.prev_acc = acc;

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

            // Some(&self.quat)
        }

        fn get_quat(&self) -> &UQuat {
            &self.quat
        }
    }

    impl AhrsFusion {
        pub fn set_heading(&mut self, heading: f32) {
            use nalgebra::{ComplexField, RealField}; // for sin, atan2

            let q = self.quat.coords;

            /// Euler angle of conjugate
            let inv_heading =
                f32::atan2(q.x * q.y + q.w * q.z, q.w * q.w - 0.5 + q.x * q.x);

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

    /// get quat, accel
    impl AhrsFusion {
        // pub fn get_quat(&self) -> UQuat {
        //     self.quat
        // }

        /// Returns the linear acceleration measurement equal to the accelerometer
        /// measurement with the 1 g of gravity removed.
        pub fn get_linear_accel(&self) -> V3 {
            let q = self.quat.coords;
            let gravity = V3::new(
                2.0 * (q.x * q.z - q.w * q.y),
                2.0 * (q.w * q.x + q.y * q.z),
                2.0 * (q.w * q.w - 0.5 + q.z * q.z),
            );
            self.prev_acc - gravity
        }

        /// Returns the Earth acceleration measurement equal to accelerometer
        /// measurement in the Earth coordinate frame with the 1 g of gravity removed.
        pub fn get_earth_accel(&self) -> V3 {
            let q = self.quat.coords;
            let qwqw = q.w * q.w; // calculate common terms to avoid repeated operations
            let qwqx = q.w * q.x;
            let qwqy = q.w * q.y;
            let qwqz = q.w * q.z;
            let qxqy = q.x * q.y;
            let qxqz = q.x * q.z;
            let qyqz = q.y * q.z;

            let acc = &self.prev_acc;

            /// transpose of a rotation matrix representation
            /// multiplied with the accelerometer, with 1 g subtracted
            V3::new(
                2.0 * ((qwqw - 0.5 + q.x * q.x) * acc.x
                    + (qxqy - qwqz) * acc.y
                    + (qxqz + qwqy) * acc.z),
                2.0 * ((qxqy + qwqz) * acc.x
                    + (qwqw - 0.5 + q.y * q.y) * acc.y
                    + (qyqz - qwqx) * acc.z),
                (2.0 * ((qxqz - qwqy) * acc.x
                    + (qyqz + qwqx) * acc.y
                    + (qwqw - 0.5 + q.z * q.z) * acc.z))
                    - 1.0,
            )
        }
    }
}

// #[cfg(feature = "nope")]
mod madgwick_prev {
    use nalgebra::{Matrix6, Quaternion, UnitQuaternion, Vector2, Vector3, Vector6};

    use super::{UQuat, AHRS, V3};
    use defmt::println as rprintln;

    pub struct AhrsMadgwick {
        sample_period: f32,
        beta:          f32,
        quat:          UQuat,
    }

    /// new
    impl AhrsMadgwick {
        pub fn new(sample_period: f32, beta: f32) -> Self {
            Self {
                sample_period,
                beta,
                quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            }
        }
    }

    /// update
    impl AHRS for AhrsMadgwick {
        /// https://crates.io/crates/ahrs
        fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
            let acc = if let Some(acc) = acc.try_normalize(0.0) {
                acc
            } else {
                rprintln!("acc norm div by zero");
                return;
            };

            let mag = if let Some(mag) = mag.try_normalize(0.0) {
                mag
            } else {
                rprintln!("mag norm div by zero");
                return;
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

            // Some(&self.quat)
        }

        fn get_quat(&self) -> &UQuat {
            &self.quat
        }
    }

    impl Default for AhrsMadgwick {
        fn default() -> Self {
            let sample_period = 1.0 / 256.0;
            let beta = 0.1;
            Self::new(sample_period, beta)
        }
    }
}
