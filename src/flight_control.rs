use crate::{
    motors::MotorsPWM,
    pid::PID,
    sensors::{UQuat, V3},
};

use defmt::{println as rprintln, Format};

pub use self::control_inputs::*;

#[cfg(feature = "nope")]
mod prev_inputs {

    /// Received from remote control
    #[derive(Default, Clone, Copy, Format)]
    pub struct ControlInputs {
        pub roll:         i16,
        pub pitch:        i16,
        pub yaw:          i16,
        pub throttle:     i16,
        pub takeoff:      bool,
        pub calibrate:    bool,
        pub motors_armed: bool,
    }

    /// deserialize
    impl ControlInputs {
        pub fn deserialize(data: &[u8]) -> Option<ControlInputs> {
            if data.len() != 9 {
                return None;
            }
            let throttle = i16::from_be_bytes(data[0..2].try_into().unwrap());
            let roll = i16::from_be_bytes(data[2..4].try_into().unwrap());
            let pitch = i16::from_be_bytes(data[4..6].try_into().unwrap());
            let yaw = i16::from_be_bytes(data[6..8].try_into().unwrap());

            let takeoff = (data[8] & 0b0001) != 0;
            let calibrate = (data[8] & 0b0010) != 0;
            let motors_armed = (data[8] & 0b0100) != 0;

            Some(ControlInputs {
                roll,
                pitch,
                yaw,
                throttle,
                takeoff,
                calibrate,
                motors_armed,
            })
        }

        pub fn as_f32(&self) -> (f32, f32, f32, f32) {
            (
                Self::to_f32(self.roll),
                Self::to_f32(self.pitch),
                Self::to_f32(self.yaw),
                Self::to_f32(self.throttle),
            )
        }

        pub fn to_f32(x: i16) -> f32 {
            x as f32 / (i16::MAX as f32)
        }
        // pub fn from_f32(x: f32) -> i16 {
        //     let x = x.clamp(-1.0, 1.0);
        //     (x * i16::MAX as f32) as i16
        // }
    }

    impl ControlInputs {
        pub fn set_roll(&mut self, roll: f32) {
            self.roll = Self::from_f32(roll);
        }
    }
}

mod control_inputs {
    use core::f32::consts::PI;

    use defmt::{println as rprintln, Format};

    /// Received from remote control
    #[derive(Default, Clone, Copy, Format)]
    pub struct ControlInputs {
        /// Range: -1.0 to 1.0
        pub roll:         f32,
        /// Range: -1.0 to 1.0
        pub pitch:        f32,
        /// Range: -2Pi to 2Pi
        pub yaw:          f32,
        /// Range: 0.0 to 1.0
        pub throttle:     f32,
        /// Toggles
        pub takeoff:      bool,
        pub calibrate:    bool,
        pub motors_armed: bool,
        pub level_mode:   bool,
    }

    /// deserialize
    impl ControlInputs {
        pub fn deserialize(data: &[u8]) -> Option<ControlInputs> {
            if data.len() != 9 {
                return None;
            }
            let roll = i16::from_be_bytes(data[0..2].try_into().unwrap());
            let pitch = i16::from_be_bytes(data[2..4].try_into().unwrap());
            let yaw = i16::from_be_bytes(data[4..6].try_into().unwrap());
            let throttle = i16::from_be_bytes(data[6..8].try_into().unwrap());

            let takeoff = (data[8] & 0b0001) != 0;
            let calibrate = (data[8] & 0b0010) != 0;
            let motors_armed = (data[8] & 0b0100) != 0;
            let level_mode = (data[8] & 0b1000) != 0;

            /// negative throttle probably won't work with motors?
            let throttle = Self::to_f32(throttle).clamp(0.0, 1.0);

            Some(ControlInputs {
                roll: Self::to_f32(roll),
                pitch: Self::to_f32(pitch),
                yaw: Self::to_f32(yaw),
                throttle,
                takeoff,
                calibrate,
                motors_armed,
                level_mode,
            })
        }

        pub fn to_f32(x: i16) -> f32 {
            x as f32 / (i16::MAX as f32)
        }
    }

    /// set values
    impl ControlInputs {
        pub fn set_roll(&mut self, roll: f32) {
            self.roll = roll.clamp(-1.0, 1.0);
        }
        pub fn set_pitch(&mut self, pitch: f32) {
            self.pitch = pitch.clamp(-1.0, 1.0);
        }
        pub fn set_yaw(&mut self, yaw: f32) {
            // self.yaw = yaw.clamp(-1.0, 1.0);
            self.yaw = yaw % (2.0 * PI);
        }
        pub fn set_throttle(&mut self, throttle: f32) {
            self.throttle = throttle.clamp(-1.0, 1.0);
        }
    }
}

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
#[derive(Clone, Copy, Format)]
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
    /// Config
    pub config:            FlightConfig,
}

/// new
impl DroneController {
    pub fn new_default_params() -> Self {
        let mut pid_roll_stab = PID::new(0.0, 0.0, 0.0);
        let mut pid_roll_rate = PID::new(0.0, 0.0, 0.0);

        let mut pid_pitch_stab = PID::new(0.0, 0.0, 0.0);
        let mut pid_pitch_rate = PID::new(0.0, 0.0, 0.0);

        let mut pid_yaw_stab = PID::new(0.0, 0.0, 0.0);
        let mut pid_yaw_rate = PID::new(0.0, 0.0, 0.0);

        let mut pid_altitude_stab = PID::new(0.0, 0.0, 0.0);
        let mut pid_altitude_rate = PID::new(0.0, 0.0, 0.0);

        // #[cfg(feature = "nope")]
        /// starting point
        {
            //

            pid_roll_rate.kp = 1.0;
            pid_roll_rate.ki = 0.0;
            pid_roll_rate.kd = 0.0;
            pid_roll_rate.i_limit = 0.0;

            pid_roll_stab.kp = 1.0;
            pid_roll_stab.ki = 0.0;
            pid_roll_stab.kd = 0.0;
            pid_roll_stab.i_limit = 0.0;

            // pid_yaw_rate.kp = 1.0;

            // pid_yaw_stab.kp = 1.0;

            /// PID and roll should be the same
            pid_roll_rate.copy_settings_to(&mut pid_pitch_rate);
            pid_roll_stab.copy_settings_to(&mut pid_pitch_stab);

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
            config: FlightConfig::default(),
        }
    }

    #[rustfmt::skip]
    #[cfg(feature = "nope")]
    pub fn new_default_params() -> Self {
        let pid_stab_roll = PID::new(
            3.0,
            0.0,
            0.0,
        );
        let pid_rate_roll = PID::new(
            80.0,
            0.0,
            0.0
        );
        let pid_stab_pitch = PID::new(
            0.0,
            0.0,
            0.0
        );
        let pid_rate_pitch = PID::new(
            0.0,
            0.0,
            0.0
        );
        let pid_stab_yaw = PID::new(
            0.0,
            0.0,
            0.0
        );
        let pid_rate_yaw = PID::new(
            0.0,
            0.0,
            0.0
        );
        let pid_throttle = PID::new(
            0.0,
            0.0,
            0.0
        );

        Self {
            pid_stab_roll,
            pid_rate_roll,
            pid_stab_pitch,
            pid_rate_pitch,
            pid_stab_yaw,
            pid_rate_yaw,
            pid_throttle,
            config: FlightConfig::default(),
        }
    }

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
            config: FlightConfig::default(),
        }
    }
}

/// update
impl DroneController {
    // #[cfg(feature = "nope")]
    pub fn update(
        &mut self,
        // inputs: &ControlInputs,
        inputs: ControlInputs,
        ahrs_quat: &UQuat,
        gyro: V3,
    ) -> MotorOutputs {
        let (ahrs_roll, ahrs_pitch, ahrs_yaw) = ahrs_quat.euler_angles();

        let err0_roll = inputs.roll - ahrs_roll;
        let err0_pitch = inputs.pitch - ahrs_pitch;
        let err0_yaw = inputs.yaw - ahrs_yaw;

        let out0_roll = self.pid_roll_stab.step(err0_roll);
        let out0_pitch = self.pid_pitch_stab.step(err0_pitch);
        let out0_yaw = self.pid_yaw_stab.step(err0_yaw);

        // let out0_roll = 0.0;
        // let out0_pitch = 0.0;
        // let out0_yaw = 0.0;

        /// Roll  = y
        /// Pitch = x
        let err1_roll = out0_roll - gyro.y;
        let err1_pitch = out0_pitch - gyro.x;
        let err1_yaw = out0_yaw - gyro.z;

        let out1_roll = self.pid_roll_rate.step(err1_roll);
        let out1_pitch = self.pid_pitch_rate.step(err1_pitch);
        let out1_yaw = self.pid_yaw_rate.step(err1_yaw);

        // let out1_roll = 0.0;
        // let out1_pitch = 0.0;

        use crate::math::rad_to_deg;
        use crate::utils::r;

        // rprintln!("err0_yaw = {:?}", rad_to_deg(err0_yaw));
        // rprintln!("out0_yaw = {:?}", out0_yaw);
        // rprintln!("err1_yaw = {:?}", err1_yaw);
        // rprintln!("out1_yaw = {:?}", out1_yaw);
        // rprintln!("");

        // rprintln!(
        //     "yaw = {:?}\nout0 = {:?}\nout1 = {:?}",
        //     r(rad_to_deg(ahrs_yaw)),
        //     r(out0_yaw),
        //     r(out1_yaw),
        // );

        // rprintln!(
        //     "out1_roll, out1_pitch, out1_yaw = {:?}, {:?}, {:?}",
        //     out1_roll,
        //     out1_pitch,
        //     out1_yaw
        // );

        /// TODO: add pitch, roll limits
        self.mix(inputs.throttle, out1_roll, out1_pitch, out1_yaw)
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

    pub max_rate_pitch: f32,
    pub max_rate_roll:  f32,
    pub max_rate_yaw:   f32,
}

impl Default for FlightConfig {
    fn default() -> Self {
        Self {
            max_pitch: 60.0,
            max_roll:  60.0,

            max_rate_pitch: 180.0,
            max_rate_roll:  180.0,
            // max_rate_pitch: 250.0,
            // max_rate_roll: 250.0,
            max_rate_yaw:   270.0,
        }
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
