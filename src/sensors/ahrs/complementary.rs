use fugit::HertzU32;
use nalgebra::{
    self as na, ComplexField, Quaternion, RealField, Rotation3, UnitQuaternion, Vector2,
    Vector3,
};

use super::{UQuat, AHRS, V3};
use crate::{math::*, sensors::Rot3};
use defmt::println as rprintln;

#[derive(Debug, Clone, Copy)]
pub struct AhrsComplementary {
    quat:           UQuat,
    pub delta_time: f32,
    gain:           f32,
}

/// new
impl AhrsComplementary {
    pub fn new(sample_rate: HertzU32, gain: f32) -> Self {
        let delta_time = 1.0 / (sample_rate.to_Hz() as f32);
        Self {
            quat: UQuat::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            delta_time,
            gain,
        }
    }
}

/// update
impl AHRS for AhrsComplementary {
    /// Attitude from gravity
    #[cfg(feature = "nope")]
    fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
        let roll = f32::atan2(acc.x, acc.z);
        let pitch = f32::atan2(-acc.y, (acc.x.powi(2) + acc.z.powi(2)).sqrt());
        self.quat = UQuat::from_euler_angles(roll, pitch, 0.0);
    }

    #[cfg(feature = "nope")]
    fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
        let q_omega = self.attitude_propagation(gyro);

        let roll = f32::atan2(acc.x, acc.z);
        let pitch = f32::atan2(-acc.y, (acc.x.powi(2) + acc.z.powi(2)).sqrt());

        /// theta, phi
        let (t, p) = (roll, pitch);

        // #[rustfmt::skip]
        // let b = na::Matrix3::<f32>::new(
        //     t.cos(),  t.sin() * p.sin(), t.sin() * p.cos(),
        //     0.0,      p.cos(),           -p.sin(),
        //     -t.sin(), t.cos() * p.sin(), t.cos() * p.cos(),
        // );
        // let b: V3 = b * mag;
        // let yaw0 = f32::atan2(-b.y, b.x);

        let yaw = f32::atan2(
            mag.z * p.sin() - mag.y * p.cos(),
            mag.x * t.cos() + mag.y * t.sin() * p.sin() + mag.z * t.sin() * p.cos(),
        );

        let q_am = UQuat::from_euler_angles(roll, pitch, yaw);

        // self.quat = q_omega.nlerp(&q_am, self.gain);
        self.quat = q_omega;
    }

    fn update(&mut self, gyro: V3, acc: V3, mag: V3) {
        let q_omega = self.attitude_propagation(gyro);

        let q_am = self.am_estimation(acc, mag);

        let q = if (q_omega.coords + q_am.coords).norm() < f32::sqrt(2.0) {
            (1.0 - self.gain) * q_omega.coords - self.gain * q_am.coords
        } else {
            (1.0 - self.gain) * q_omega.coords + self.gain * q_am.coords
        };

        self.quat = UQuat::from_quaternion(q.into());
    }

    fn get_quat(&self) -> UQuat {
        self.quat
    }
}

/// attitude_propagation, am_estimation
impl AhrsComplementary {
    pub fn attitude_propagation(&self, gyro: V3) -> UQuat {
        let w: V3 = -0.5 * self.delta_time * gyro;

        #[rustfmt::skip]
            let a = na::Matrix4::<f32>::new(
                1.0,  -w[0], -w[1], -w[2],
                w[0],   1.0,  w[2], -w[1],
                w[1], -w[2],   1.0,  w[0],
                w[2],  w[1], -w[0],   1.0);

        let q_omega: Quaternion<f32> = (a * self.quat.coords).into();

        // q_omega / q_omega.norm()
        UQuat::from_quaternion(q_omega)
    }

    pub fn am_estimation(&self, acc: V3, mag: V3) -> UQuat {
        // let na = acc.normalize();
        let nm = mag.normalize();

        let r_z = acc.normalize();

        let r_y = r_z.cross(&nm).normalize();
        let r_x = r_y.cross(&r_z).normalize();

        let r = na::Matrix3::from([
            [r_x.x, r_x.y, r_x.z],
            [r_y.x, r_y.y, r_y.z],
            [r_z.x, r_z.y, r_z.z],
        ]);

        UQuat::from_rotation_matrix(&na::Rotation3::from_matrix(&r))
    }

    #[cfg(feature = "nope")]
    pub fn am_estimation(&self, acc: V3, mag: V3) -> UQuat {
        // m /= np.linalg.norm(m)

        // Rz = a/np.linalg.norm(a)

        // if frame.upper() == 'NED':
        //     Ry = np.cross(Rz, m)
        //     Rx = np.cross(Ry, Rz)
        // else:
        //     Rx = np.cross(m, Rz)
        //     Ry = np.cross(Rz, Rx)

        // Rx /= np.linalg.norm(Rx)
        // Ry /= np.linalg.norm(Ry)
        // R = np.c_[Rx, Ry, Rz].T

        // if representation.lower() == 'quaternion':
        //     return chiaverini(R)

        // if representation.lower() == 'rpy':
        //     phi = np.arctan2(R[1, 2], R[2, 2])    # Roll Angle
        //     theta = -np.arcsin(R[0, 2])           # Pitch Angle
        //     psi = np.arctan2(R[0, 1], R[0, 0])    # Yaw Angle
        //     return np.array([phi, theta, psi])
        // if representation.lower() == 'axisangle':
        //     angle = np.arccos((R.trace()-1)/2)
        //     axis = np.zeros(3)
        //     if angle != 0:
        //         S = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
        //         axis = S/(2*np.sin(angle))
        //     return (axis, angle)
        // return R

        unimplemented!()
    }
}
