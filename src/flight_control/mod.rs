mod control_inputs;
mod control_rates;

use crate::{
    math::rad_to_deg,
    motors::MotorsPWM,
    pid::PID,
    sensors::{UQuat, V3},
};

use defmt::{println as rprintln, Format};

pub use self::control_inputs::*;
pub use self::control_rates::*;

#[derive(Clone, Copy, Format)]
#[repr(u8)]
pub enum IdPID {
    RollStab,
    RollRate,
    PitchStab,
    PitchRate,
    YawStab,
    YawRate,
    AltitudeStab,
    AltitudeRate,
}

impl IdPID {
    pub const ITER: [IdPID; 8] = [
        IdPID::RollStab,
        IdPID::RollRate,
        IdPID::PitchStab,
        IdPID::PitchRate,
        IdPID::YawStab,
        IdPID::YawRate,
        IdPID::AltitudeStab,
        IdPID::AltitudeRate,
    ];

    pub fn from_u8(p: u8) -> Option<Self> {
        match p {
            0 => Some(Self::RollStab),
            1 => Some(Self::RollRate),
            2 => Some(Self::PitchStab),
            3 => Some(Self::PitchRate),
            4 => Some(Self::YawStab),
            5 => Some(Self::YawRate),
            6 => Some(Self::AltitudeStab),
            7 => Some(Self::AltitudeRate),
            _ => None,
        }
    }
}

/// size: each PID is 60 bytes
#[derive(Clone, Copy)]
pub struct DroneController {
    /// Roll
    pub pid_roll_stab:     PID,
    pub pid_roll_rate:     PID,
    /// Pitch
    pub pid_pitch_stab:    PID,
    pub pid_pitch_rate:    PID,
    /// Yaw
    pub pid_yaw_stab:      PID,
    pub pid_yaw_rate:      PID,
    /// Throttle
    pub pid_altitude_stab: PID,
    pub pid_altitude_rate: PID,
    /// Rates
    pub rates:             ControlRates,
    /// Config
    pub config:            FlightConfig,
}

/// new
impl DroneController {
    pub fn new_default_params() -> Self {
        let mut pid_roll_stab = PID::new_limited(0.0, 0.0, 0.0);
        let mut pid_roll_rate = PID::new_limited(0.0, 0.0, 0.0);

        let mut pid_pitch_stab = PID::new_limited(0.0, 0.0, 0.0);
        let mut pid_pitch_rate = PID::new_limited(0.0, 0.0, 0.0);

        let mut pid_yaw_stab = PID::new_limited(0.0, 0.0, 0.0);
        let mut pid_yaw_rate = PID::new_limited(0.0, 0.0, 0.0);

        let mut pid_altitude_stab = PID::new_limited(0.0, 0.0, 0.0);
        let mut pid_altitude_rate = PID::new_limited(0.0, 0.0, 0.0);

        /// output meanings:
        /// pid_pitch_stab:  desired pitch rate in rad/s

        // pid_pitch_rate.kp = 0.1;
        // pid_pitch_stab.kp = 0.1;

        // #[cfg(feature = "nope")]
        /// starting point
        {
            //

            // pid_roll_rate.kp = 1.0;
            // pid_roll_rate.ki = 0.0;
            // pid_roll_rate.kd = 0.0;
            // pid_roll_rate.i_limit = 0.0;

            // pid_roll_stab.kp = 1.0;
            // pid_roll_stab.ki = 0.0;
            // pid_roll_stab.kd = 0.0;
            // pid_roll_stab.i_limit = 0.0;

            // pid_pitch_rate.kp = 0.1;
            // pid_pitch_stab.kp = 0.1;

            // pid_pitch_stab.kp = 0.001579; // kp1
            // pid_pitch_stab.i_limit = 0.001053; // XXX: ST firmware says this is 5 degrees ??

            // pid_pitch_rate.kp = 0.04211; // kp2
            // pid_pitch_rate.ki = 0.04211; // ki2
            // pid_pitch_rate.kd = 0.005263; // kd2
            // pid_pitch_rate.i_limit = 0.01053;

            // pid_yaw_rate.kp = 1.0;

            // pid_yaw_stab.kp = 1.0;

            // /// PID and roll should be the same
            // pid_roll_rate.copy_settings_to(&mut pid_pitch_rate);
            // pid_roll_stab.copy_settings_to(&mut pid_pitch_stab);

            //
        }

        #[cfg(feature = "nope")]
        /// ST values, divided by 1900 (from PWM)
        /// too low
        {
            pid_roll_stab.kp = 0.001579; // kp1
            pid_roll_stab.i_limit = 0.001053; // XXX: ST firmware says this is 5 degrees ??

            pid_roll_rate.kp = 0.04211; // kp2
            pid_roll_rate.ki = 0.04211; // ki2
            pid_roll_rate.kd = 0.005263; // kd2
            pid_roll_rate.i_limit = 0.01053;

            // pid_pitch_stab.kp = pid_roll_stab.kp;
            // pid_pitch_stab.ki = pid_roll_stab.ki;
            // pid_pitch_stab.i_limit = pid_roll_stab.i_limit;

            // pid_pitch_rate.kp = pid_roll_rate.kp;
            // pid_pitch_rate.ki = pid_roll_rate.ki;
            // pid_pitch_rate.kd = pid_roll_rate.kd;
            // pid_pitch_rate.i_limit = pid_roll_rate.i_limit;

            // pid_yaw_stab.kp = 0.002105;
            // pid_yaw_stab.ki = 0.0;
            // pid_yaw_stab.i_limit = 0.001053;

            // pid_yaw_rate.kp = 0.4737;
            // pid_yaw_rate.ki = 0.001579;
            // pid_yaw_rate.kd = 0.001579;
            // pid_yaw_rate.i_limit = 0.001053;
        }

        #[cfg(feature = "nope")]
        /// ST values
        {
            pid_roll_stab.kp = 3.0; // kp1
            pid_roll_stab.i_limit = 2.0; // XXX: ST firmware says this is 5 degrees ??

            pid_roll_rate.kp = 80.0; // kp2
            pid_roll_rate.ki = 80.0; // ki2
            pid_roll_rate.kd = 10.0; // kd2
            pid_roll_rate.i_limit = 20.0;

            pid_pitch_stab.kp = pid_roll_stab.kp;
            pid_pitch_stab.ki = pid_roll_stab.ki;
            pid_pitch_stab.i_limit = pid_roll_stab.i_limit;

            pid_pitch_rate.kp = pid_roll_rate.kp;
            pid_pitch_rate.ki = pid_roll_rate.ki;
            pid_pitch_rate.kd = pid_roll_rate.kd;
            pid_pitch_rate.i_limit = pid_roll_rate.i_limit;

            pid_yaw_stab.kp = 4.0;
            pid_yaw_stab.ki = 0.0;
            pid_yaw_stab.i_limit = 2.0;

            pid_yaw_rate.kp = 900.0;
            pid_yaw_rate.ki = 3.0;
            pid_yaw_rate.kd = 3.0;
            pid_yaw_rate.i_limit = 2.0;
        }

        Self {
            pid_roll_stab,
            pid_roll_rate,
            pid_pitch_stab,
            pid_pitch_rate,
            pid_yaw_stab,
            pid_yaw_rate,
            pid_altitude_stab,
            pid_altitude_rate,
            rates: ControlRates::default(),
            config: FlightConfig::default(),
        }
    }

    #[cfg(feature = "nope")]
    pub fn new(
        pid_roll_stab: PID,
        pid_roll_rate: PID,
        pid_pitch_stab: PID,
        pid_pitch_rate: PID,
        pid_yaw_stab: PID,
        pid_yaw_rate: PID,
        pid_altitude_stab: PID,
        pid_altitude_rate: PID,
    ) -> Self {
        Self {
            pid_roll_stab,
            pid_roll_rate,
            pid_pitch_stab,
            pid_pitch_rate,
            pid_yaw_stab,
            pid_yaw_rate,
            pid_altitude_stab,
            pid_altitude_rate,
            rates: ControlRates::default(),
            config: FlightConfig::default(),
        }
    }
}

/// update PIDs
impl DroneController {
    /// +roll  = left wing up
    /// +pitch = nose up
    /// +yaw   = nose right
    pub fn update(
        &mut self,
        inputs: ControlInputs,
        ahrs_quat: &UQuat,
        gyro: V3,
    ) -> MotorOutputs {
        if inputs.get_level_mode() {
            self.update_level_mode(inputs, ahrs_quat, gyro)
        } else {
            self.update_acro_mode(inputs, ahrs_quat, gyro)
        }

        // defmt::warn!("overriding level mode, using acro mode");
        // self.update_acro_mode(inputs, ahrs_quat, gyro)
    }
}

/// Level mode
impl DroneController {
    pub fn update_level_mode(
        &mut self,
        inputs: ControlInputs,
        ahrs_quat: &UQuat,
        gyro: V3,
    ) -> MotorOutputs {
        /// in radians
        let (ahrs_roll, ahrs_pitch, ahrs_yaw) = ahrs_quat.euler_angles();
        /// convert to degrees since gyro is in deg/s
        let (ahrs_roll, ahrs_pitch, ahrs_yaw) = (
            rad_to_deg(ahrs_roll),
            rad_to_deg(ahrs_pitch),
            rad_to_deg(ahrs_yaw),
        );

        let (i_roll, i_pitch, i_yaw, i_throttle) = inputs.get_values(&self.rates);

        // /// XXX: testing PID
        // let i_pitch = 15.0;

        let err0_roll = i_roll - ahrs_roll;
        let err0_pitch = i_pitch - ahrs_pitch;
        // let err0_yaw = i_yaw - ahrs_yaw;

        let out0_roll = self.pid_roll_stab.step(err0_roll);
        let out0_pitch = self.pid_pitch_stab.step(err0_pitch);
        // let out0_yaw = self.pid_yaw_stab.step(err0_yaw);

        // let out0_roll = 0.0f32;
        // let out0_pitch = 0.0f32;
        // let out0_yaw = 0.0f32;

        // let out0_roll = self.config.limit_roll_rate(out0_roll);
        // let out0_pitch = self.config.limit_pitch_rate(out0_pitch);
        // let out0_yaw = self.config.limit_yaw_rate(i_yaw);

        /// Roll  = y
        /// Pitch = x
        let err1_roll = out0_roll - gyro.y;
        let err1_pitch = out0_pitch - gyro.x;
        // let err1_yaw = out0_yaw - gyro.z;
        let err1_yaw = i_yaw - gyro.z;

        let out1_roll = self.pid_roll_rate.step(err1_roll);
        let out1_pitch = self.pid_pitch_rate.step(err1_pitch);
        let out1_yaw = self.pid_yaw_rate.step(err1_yaw);

        // let out1_yaw = 0.1;
        // let out1_yaw = -0.1;

        // let out1_roll = 0.0;
        // let out1_pitch = 0.0;
        // let out1_yaw = 0.0;

        use crate::math::rad_to_deg;
        use crate::utils::r;

        // rprintln!(
        //     "err0_pitch = {:?}\nout0_pitch = {:?}\nerr1_pitch = {:?}\nout1_pitch = {:?}",
        //     err0_pitch,
        //     out0_pitch,
        //     err1_pitch,
        //     out1_pitch
        // );

        // rprintln!(
        //     "yaw = {:?}\nerr1_yaw = {:?}\nout1_yaw = {:?}",
        //     r(ahrs_yaw),
        //     r(err1_yaw),
        //     r(out1_yaw),
        // );

        // rprintln!(
        //     "out1_roll, out1_pitch, out1_yaw = {:?}, {:?}, {:?}",
        //     out1_roll,
        //     out1_pitch,
        //     out1_yaw
        // );

        /// TODO: add pitch, roll limits
        self.mix(i_throttle, out1_roll, out1_pitch, out1_yaw)
    }
}

/// Acro mode
impl DroneController {
    pub fn update_acro_mode(
        &mut self,
        inputs: ControlInputs,
        ahrs_quat: &UQuat,
        gyro: V3,
    ) -> MotorOutputs {
        /// in radians
        let (ahrs_roll, ahrs_pitch, ahrs_yaw) = ahrs_quat.euler_angles();
        /// convert to degrees since gyro is in deg/s
        let (ahrs_roll, ahrs_pitch, ahrs_yaw) = (
            rad_to_deg(ahrs_roll),
            rad_to_deg(ahrs_pitch),
            rad_to_deg(ahrs_yaw),
        );

        let (i_roll, i_pitch, i_yaw, i_throttle) = inputs.get_values(&self.rates);

        // let i_roll = self.config.limit_roll_rate(i_roll);
        // let i_pitch = self.config.limit_pitch_rate(i_pitch);
        // let i_yaw = self.config.limit_yaw_rate(i_yaw);

        /// Roll  = y
        /// Pitch = x
        let err1_roll = i_roll - gyro.y;
        let err1_pitch = i_pitch - gyro.x;
        let err1_yaw = i_yaw - gyro.z;

        let out1_roll = self.pid_roll_rate.step(err1_roll);
        let out1_pitch = self.pid_pitch_rate.step(err1_pitch);
        let out1_yaw = self.pid_yaw_rate.step(err1_yaw);

        // rprintln!(
        //     "err1_pitch = {:?}\nout1_pitch = {:?}",
        //     err1_pitch,
        //     out1_pitch
        // );

        // rprintln!(
        //     "yaw = {:?}\ngyro = {:?}\nerr1_yaw = {:?}\nout1_yaw = {:?}",
        //     ahrs_yaw,
        //     gyro.z,
        //     err1_yaw,
        //     out1_yaw,
        // );

        self.mix(i_throttle, out1_roll, out1_pitch, out1_yaw)
    }
}

/// mix
impl DroneController {
    /// +roll  = left wing up
    /// +pitch = nose up
    /// +yaw   = nose right
    pub fn mix(&self, throttle: f32, roll: f32, pitch: f32, yaw: f32) -> MotorOutputs {
        self._mix(throttle, roll, pitch, yaw, true)
    }

    pub fn _mix(
        &self,
        throttle: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
        once: bool,
    ) -> MotorOutputs {
        /// Roll
        /// ++  --
        /// ++  --
        /// Pitch
        /// ++ ++
        /// -- --
        /// Yaw
        /// -- ++
        /// ++ --
        let front_left = throttle - yaw + pitch + roll; // M3
        let front_right = throttle + yaw + pitch - roll; // M2
        let back_left = throttle + yaw - pitch + roll; // M4
        let back_right = throttle - yaw - pitch - roll; // M1

        let outs = [front_left, front_right, back_left, back_right];
        let excess_output = *outs
            .iter()
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();

        if once && excess_output > 1.0 {
            return self._mix(throttle - (excess_output - 1.0), roll, pitch, yaw, false);
        }

        let front_left = front_left.clamp(0.0, 1.0);
        let front_right = front_right.clamp(0.0, 1.0);
        let back_left = back_left.clamp(0.0, 1.0);
        let back_right = back_right.clamp(0.0, 1.0);

        MotorOutputs {
            input_roll: roll,
            input_pitch: pitch,
            input_yaw: yaw,

            front_left,
            front_right,
            back_left,
            back_right,
        }
    }
}

impl core::ops::Index<IdPID> for DroneController {
    type Output = PID;
    fn index(&self, index: IdPID) -> &Self::Output {
        use self::IdPID::*;
        match index {
            RollStab => &self.pid_roll_stab,
            RollRate => &self.pid_roll_rate,
            PitchStab => &self.pid_pitch_stab,
            PitchRate => &self.pid_pitch_rate,
            YawStab => &self.pid_yaw_stab,
            YawRate => &self.pid_yaw_rate,
            AltitudeStab => &self.pid_altitude_stab,
            AltitudeRate => &self.pid_altitude_rate,
        }
    }
}

impl core::ops::IndexMut<IdPID> for DroneController {
    fn index_mut(&mut self, index: IdPID) -> &mut Self::Output {
        use self::IdPID::*;
        match index {
            RollStab => &mut self.pid_roll_stab,
            RollRate => &mut self.pid_roll_rate,
            PitchStab => &mut self.pid_pitch_stab,
            PitchRate => &mut self.pid_pitch_rate,
            YawStab => &mut self.pid_yaw_stab,
            YawRate => &mut self.pid_yaw_rate,
            AltitudeStab => &mut self.pid_altitude_stab,
            AltitudeRate => &mut self.pid_altitude_rate,
        }
    }
}

#[derive(Clone, Copy, Format)]
pub struct FlightConfig {
    pub max_pitch: f32,
    pub max_roll:  f32,

    pub max_rate_roll:  f32,
    pub max_rate_pitch: f32,
    pub max_rate_yaw:   f32,
}

impl Default for FlightConfig {
    fn default() -> Self {
        Self {
            max_pitch: 60.0,
            max_roll:  60.0,

            max_rate_roll:  180.0,
            max_rate_pitch: 180.0,
            // max_rate_pitch: 250.0,
            // max_rate_roll: 250.0,
            max_rate_yaw:   270.0,
        }
    }
}

impl FlightConfig {
    pub fn limit_roll_rate(&self, roll: f32) -> f32 {
        roll.clamp(-self.max_rate_roll, self.max_rate_roll)
    }
    pub fn limit_pitch_rate(&self, pitch: f32) -> f32 {
        pitch.clamp(-self.max_rate_pitch, self.max_rate_pitch)
    }
    pub fn limit_yaw_rate(&self, yaw: f32) -> f32 {
        yaw.clamp(-self.max_rate_yaw, self.max_rate_yaw)
    }
}

/// Output of PID after mixing
/// PWM values: from 0-1900
#[derive(Default, Clone, Copy, Format)]
pub struct MotorOutputs {
    /// for logging
    pub input_roll:  f32,
    pub input_pitch: f32,
    pub input_yaw:   f32,

    pub front_left:  f32,
    pub front_right: f32,
    pub back_left:   f32,
    pub back_right:  f32,
}

/// apply
impl MotorOutputs {
    pub fn apply(&self, motors: &mut MotorsPWM) {
        use crate::motors::MotorSelect::*;
        motors.set_motor_f32(Motor1, self.back_right);
        motors.set_motor_f32(Motor2, self.front_right);
        motors.set_motor_f32(Motor3, self.front_left);
        motors.set_motor_f32(Motor4, self.back_left);
    }
}
