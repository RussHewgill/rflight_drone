use core::f32::consts::PI;

use fugit::HertzU32;
use nalgebra::{self as na, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

use super::{Rot3, UQuat, AHRS, V3};
use crate::{math::*, utils::print_v3};
use defmt::println as rprintln;

pub use self::{calibration::*, offset::*};

#[derive(Debug, Clone, Copy)]
pub struct AhrsFusion {
    quat:       UQuat,
    delta_time: f32,

    alt_ref_pressure:    f32,
    alt_ref_temperature: f32,
    altitude:            f32,

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

/// new, reset
impl AhrsFusion {
    pub const INIT_PERIOD: f32 = 3.0;

    pub const INITIAL_GAIN: f32 = 10.0;

    // pub const INITIAL_GAIN: f32 = 20.0;

    pub fn new(sample_rate: HertzU32, gain: f32) -> Self {
        let delta_time = 1.0 / (sample_rate.to_Hz() as f32);

        // rprintln!("delta_time = {:?}", delta_time);
        // let step = (Self::INITIAL_GAIN - gain) / Self::INIT_PERIOD;
        // rprintln!("step = {:?}", step);

        let mut offset = FusionOffset::default();
        offset.init(sample_rate);

        let calibration = FusionCalibration::new(delta_time);

        Self {
            quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            delta_time,

            alt_ref_pressure: 0.0,
            alt_ref_temperature: 0.0,
            altitude: 0.0,

            offset,
            calibration,

            cfg_gain: gain,
            cfg_acc_rejection: 10.0,
            cfg_mag_rejection: 20.0,

            cfg_rejection_timeout: (5.0 / delta_time) as u32, // 5 seconds

            initializing: true,
            ramped_gain: Self::INITIAL_GAIN,
            // ramped_gain_step: (Self::INITIAL_GAIN - gain) / Self::INIT_TIME,
            ramped_gain_step: (Self::INITIAL_GAIN - gain) / Self::INIT_PERIOD,

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
        // defmt::debug!("AHRS Fusion resetting");
        rprintln!("AHRS Fusion resetting");
        // self.offset.reset();
        self.quat = UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        self.altitude = 0.0;
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

/// update_no_mag
impl AhrsFusion {
    pub fn update_no_mag(&mut self, gyro: V3, acc: V3) {
        self.update(gyro, acc, V3::zeros());
        /// Zero heading during initialisation
        if self.initializing && !self.acc_rejection_timeout {
            self.set_heading(0.0);
        }
    }
}

/// altitude
impl AhrsFusion {
    pub fn get_altitude(&self) -> f32 {
        self.altitude
    }

    pub fn update_baro(&mut self, pressure: f32, temp: f32) -> f32 {
        use nalgebra::ComplexField;

        if self.alt_ref_pressure == 0.0 && self.alt_ref_temperature == 0.0 {
            self.alt_ref_pressure = pressure;
            self.alt_ref_temperature = temp;
        }

        // self.altitude = (self.alt_ref_temperature )

        const R: f32 = 8.31432; // universal gas constant
        const G0: f32 = 9.80665;
        const M: f32 = 0.0289644; // molar mass of Earth’s air (kg/mol)

        self.altitude =
            (R * self.alt_ref_temperature * f32::ln(pressure / self.alt_ref_pressure))
                / (-G0 * M);

        self.altitude
    }
}

impl AHRS for AhrsFusion {
    fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
        /// converge on good quat over 3 seconds
        if self.initializing {
            // rprintln!("initializing");
            self.ramped_gain -= self.ramped_gain_step * self.delta_time;
            // rprintln!("initializing, ramped_gain = {:?}", self.ramped_gain);
            if self.ramped_gain < self.cfg_gain {
                rprintln!("AhrsFusion: initializing complete");
                self.ramped_gain = self.cfg_gain;
                self.initializing = false;
                self.acc_rejection_timeout = false;
            }
        }

        /// Calculate direction of gravity indicated by algorithm
        /// third column of transposed rotation matrix scaled by 0.5
        let half_gravity = {
            let q = self.quat.coords;
            V3::new(
                q.x * q.z - q.w * q.y,
                q.w * q.x + q.y * q.z,
                q.w * q.w - 0.5 + q.z * q.z,
            )
        };

        // print_v3("half_gravity = ", half_gravity, 4);
        // print_v3("acc          = ", acc * 0.5, 4);
        // print_v3("diff         = ", acc * 0.5 - half_gravity, 4);
        // rprintln!("diff.mag = {:?}", (acc * 0.5 - half_gravity).magnitude());

        // /// equal to 3rd column of rotation matrix representation scaled by 0.5
        // let half_grav2 = self.quat.to_rotation_matrix().matrix().transpose()
        //     .column(2) * 0.5;

        // rprintln!("half_gravity  = {:?}", defmt::Debug2Format(&half_gravity));
        // rprintln!("half_gravity2 = {:?}", defmt::Debug2Format(&half_gravity2));

        // assert_eq!(half_gravity, half_gravity2);

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
                rprintln!("ignoring acc, distorted");
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
            // print_v3("half_west = ", half_west, 4);

            // let q = self.quat.as_ref();
            // let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
            // let b = Quaternion::new(0.0, Vector2::new(h[0], h[1]).norm(), 0.0, h[2]);

            // /// equal to 2nd column of rotation matrix representation scaled by 0.5
            // let half_west2 = self.quat.to_rotation_matrix().matrix().transpose()
            //     .column(1) * 0.5;

            // rprintln!("half_west  = {:?}", defmt::Debug2Format(&half_west));
            // rprintln!("half_west2 = {:?}", defmt::Debug2Format(&half_west2));

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
                rprintln!("ignoring mag, distorted");
                self.mag_rejection_timer += 1;
            }
        }

        /// Convert gyroscope to radians per second scaled by 0.5
        let half_gyro = gyro * deg_to_rad(0.5);
        // print_v3("half_gyro  = ", half_gyro, 3);

        /// Apply feedback to gyroscope
        let adjusted_half_gyro =
            half_gyro + ((half_acc_feedback + half_mag_feedback) * self.ramped_gain);
        // print_v3("adjusted_half_gyro  = ", adjusted_half_gyro, 3);

        /// Integrate rate of change of quaternion
        /// Normalise quaternion
        let v: V3 = adjusted_half_gyro * self.delta_time;

        let v = Quaternion::from_parts(0.0, v);
        self.quat = UQuat::from_quaternion(*self.quat + (*self.quat * v));

        // self.quat = UQuat::from_quaternion(*self.quat + quat_mult_vec(*self.quat, v));

        if self.is_acc_warning() {
            rprintln!("acc warning");
        }
        if self.is_acc_timeout() {
            rprintln!("acc timeout");
        }
        if self.is_mag_warning() {
            rprintln!("mag warning");
        }
        if self.is_mag_timeout() {
            rprintln!("mag timeout");
        }

        //
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
}

/// get flags
impl AhrsFusion {
    pub fn is_acc_warning(&self) -> bool {
        self.acc_rejection_timer > self.cfg_rejection_timeout / 4
    }
    pub fn is_mag_warning(&self) -> bool {
        self.mag_rejection_timer > self.cfg_rejection_timeout / 4
    }

    pub fn is_acc_timeout(&self) -> bool {
        self.acc_rejection_timeout
    }
    pub fn is_mag_timeout(&self) -> bool {
        self.mag_rejection_timeout
    }

    // pub fn is_acc_ignored(&self) -> bool {
    //     self.acc_ignored
    // }
    // pub fn is_mag_ignored(&self) -> bool {
    //     self.mag_ignored
    // }
}

mod calibration {
    use nalgebra::{self as na, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

    use super::{Rot3, UQuat, AHRS, V3};
    use defmt::println as rprintln;

    #[derive(Debug, Clone, Copy)]
    pub struct FusionCalibration {
        pub initializing: bool,
        delta_time:       f32,

        /// gyro
        pub gyro_misalignment: Rot3,
        pub gyro_sens:         V3,
        pub gyro_offset:       V3,
        /// acc
        pub acc_misalignment:  Rot3,
        pub acc_sens:          V3,
        pub acc_offset:        V3,
        /// mag
        pub soft_iron_rot:     Option<Rot3>,
        pub hard_iron_offset:  V3,
    }

    /// new
    impl FusionCalibration {
        pub fn new(delta_time: f32) -> Self {
            Self {
                    initializing: true,
                    delta_time,

                    /// gyro
                    gyro_misalignment: Rot3::default(),
                    gyro_sens: V3::new(1.0, 1.0, 1.0),
                    gyro_offset: V3::default(),
                    /// acc
                    acc_misalignment: Rot3::default(),
                    acc_sens: V3::new(1.0, 1.0, 1.0),
                    acc_offset: V3::default(),
                    /// mag
                    // soft_iron_rot: Rot3::default(),
                    soft_iron_rot: None,
                    hard_iron_offset: V3::default(),
                }
        }
    }

    /// calibrate
    impl FusionCalibration {
        #[cfg(feature = "nope")]
        pub fn calc_calibration_mag(
            &mut self,
            geo: GeoMagField,
            mags: &[V3],
            accs: &[V3],
        ) {
            let mag_min_x: f32 = mags
                .iter()
                .map(|v| v.x)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();
            let mag_min_y: f32 = mags
                .iter()
                .map(|v| v.y)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();
            let mag_min_z: f32 = mags
                .iter()
                .map(|v| v.z)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();

            let mag_max_x: f32 = mags
                .iter()
                .map(|v| v.x)
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();
            let mag_max_y: f32 = mags
                .iter()
                .map(|v| v.y)
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();
            let mag_max_z: f32 = mags
                .iter()
                .map(|v| v.z)
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();

            self.hard_iron_offset = V3::new(
                (mag_min_x + mag_max_x) / 2.0,
                (mag_min_y + mag_max_y) / 2.0,
                // (mag_min_z + mag_max_z) / 2.0,
                0.0,
            );
        }

        pub fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
            if !self.initializing {
                return;
            }
            rprintln!("updating calibration");
        }

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
            soft_iron_matrix: Option<na::Rotation3<f32>>,
            hard_iron_offset: V3,
        ) -> V3 {
            if let Some(soft_iron_matrix) = soft_iron_matrix {
                (soft_iron_matrix * uncalibrated) - hard_iron_offset
            } else {
                uncalibrated - hard_iron_offset
            }
        }
    }
}

mod offset {
    use core::f32::consts::PI;

    use fugit::HertzU32;
    use nalgebra::{self as na, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

    use super::{Rot3, UQuat, AHRS, V3};
    use defmt::println as rprintln;

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

        pub fn init(&mut self, sample_rate: HertzU32) {
            self.filter_coef =
                2.0 * PI * Self::CUTOFF_FREQ * (1.0 / sample_rate.raw() as f32);
            // rprintln!("self.filter_coef = {:?}", self.filter_coef);
            self.timeout = Self::TIMEOUT * sample_rate.raw();
            // rprintln!("self.timeout = {:?}", self.timeout);
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
                // rprintln!("gyro not stationary, resetting timer");
                self.timer = 0;
                return gyro;
            }

            /// Increment timer while gyroscope stationary
            if self.timer < self.timeout {
                self.timer += 1;
                // rprintln!("gyro stationary, ticking: {:?}", self.timer);
                return gyro;
            }

            // rprintln!("Adjusting gyro offset");
            // Adjust offset if timer has elapsed
            self.gyro_offset += gyro * self.filter_coef;

            // rprintln!(
            //     "offset = {=f32:08}, {=f32:08}, {=f32:08}",
            //     self.gyro_offset.x,
            //     self.gyro_offset.y,
            //     self.gyro_offset.z
            // );

            gyro
        }
    }
}

// #[cfg(feature = "nope")]
mod mag_offset {
    use core::f32::consts::PI;

    use fugit::HertzU32;
    use nalgebra::{self as na, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

    use super::{Rot3, UQuat, AHRS, V3};
    use defmt::println as rprintln;

    #[derive(Debug, Default, Clone, Copy)]
    pub struct MagOffset {
        pub mag_field_earth:                 V3,
        pub rot_prev:                        Rot3,
        pub mag_field_earth_normalized_prev: V3,
        pub mag_alignment:                   V3,
        pub mag_field_body_magnitude_prev:   f32,
        pub mag_field_body_prev:             V3,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct GeoMagField {
        pub declination:   f32,
        pub inclination:   f32,
        pub horizontal_nt: f32,
        pub north_nt:      f32,
        pub east_nt:       f32,
        pub vertical_nt:   f32,
        pub total_nt:      f32,
    }

    impl MagOffset {
        pub fn update(&mut self) {
            unimplemented!()
        }
    }
}
