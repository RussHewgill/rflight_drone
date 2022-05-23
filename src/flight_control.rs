use crate::{
    motors::MotorsPWM,
    pid::PID,
    sensors::{UQuat, V3},
};

/// Received from remote control
#[derive(Debug, Default, Clone, Copy)]
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
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum IdPID {
    StabRoll,
    RateRoll,
    StabPitch,
    RatePitch,
    StabYaw,
    RateYaw,
    AltitudeRate,
}

impl IdPID {
    pub fn from_u8(p: u8) -> Option<Self> {
        match p {
            0 => Some(Self::StabRoll),
            1 => Some(Self::RateRoll),
            2 => Some(Self::StabPitch),
            3 => Some(Self::RatePitch),
            4 => Some(Self::StabYaw),
            5 => Some(Self::RateYaw),
            6 => Some(Self::AltitudeRate),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DroneController {
    /// roll
    pub pid_stab_roll:     PID,
    pub pid_rate_roll:     PID,
    /// pitch
    pub pid_stab_pitch:    PID,
    pub pid_rate_pitch:    PID,
    /// yaw
    pub pid_stab_yaw:      PID,
    pub pid_rate_yaw:      PID,
    /// throttle
    pub pid_altitude_rate: PID,
    /// config
    pub config:            FlightConfig,
}

/// new
impl DroneController {
    pub fn new_default_params() -> Self {
        let mut pid_stab_roll = PID::new(0.0, 0.0, 0.0);
        let mut pid_rate_roll = PID::new(0.0, 0.0, 0.0);

        let mut pid_stab_pitch = PID::new(0.0, 0.0, 0.0);
        let mut pid_rate_pitch = PID::new(0.0, 0.0, 0.0);

        let mut pid_stab_yaw = PID::new(0.0, 0.0, 0.0);
        let mut pid_rate_yaw = PID::new(0.0, 0.0, 0.0);

        let mut pid_altitude = PID::new(0.0, 0.0, 0.0);

        pid_stab_roll.kp = 3.0; // kp1
        pid_stab_roll.p_limit = 2.0; // XXX: ST firmware says this is 5 degrees ??

        pid_rate_roll.kp = 80.0; // kp2
        pid_rate_roll.ki = 80.0; // ki2
        pid_rate_roll.kd = 10.0; // kd2
        pid_rate_roll.p_limit = 20.0;

        // pid_

        Self {
            pid_stab_roll,
            pid_rate_roll,
            pid_stab_pitch,
            pid_rate_pitch,
            pid_stab_yaw,
            pid_rate_yaw,
            pid_altitude_rate: pid_altitude,
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
        pid_stab_roll: PID,
        pid_rate_roll: PID,
        pid_stab_pitch: PID,
        pid_rate_pitch: PID,
        pid_stab_yaw: PID,
        pid_rate_yaw: PID,
        pid_throttle: PID,
    ) -> Self {
        Self {
            pid_stab_roll,
            pid_rate_roll,
            pid_stab_pitch,
            pid_rate_pitch,
            pid_stab_yaw,
            pid_rate_yaw,
            pid_altitude_rate: pid_throttle,
            config: FlightConfig::default(),
        }
    }
}

/// update, mix
impl DroneController {
    pub fn update(
        &mut self,
        inputs: ControlInputs,
        ahrs_quat: &UQuat,
        gyro: V3,
    ) -> MotorOutputs {
        let (ahrs_roll, ahrs_pitch, ahrs_yaw) = ahrs_quat.euler_angles();

        let (i_roll, i_pitch, i_yaw, i_throttle) = inputs.as_f32();

        let err0_roll = i_roll - ahrs_roll;
        let err0_pitch = i_pitch - ahrs_pitch;
        let err0_yaw = i_yaw - ahrs_yaw;

        // let err_throttle = i_throttle - throttle;

        let out0_roll = self.pid_stab_roll.step(err0_roll);
        let out0_pitch = self.pid_stab_pitch.step(err0_pitch);
        let out0_yaw = self.pid_stab_yaw.step(err0_yaw);

        let err1_roll = out0_roll - gyro.y;
        let err1_pitch = out0_pitch - gyro.x;
        let err1_yaw = out0_pitch - gyro.z;

        let out1_roll = self.pid_rate_roll.step(err1_roll);
        let out1_pitch = self.pid_rate_pitch.step(err1_pitch);
        let out1_yaw = self.pid_rate_yaw.step(err1_yaw);

        self.mix(i_throttle, out1_roll, out1_pitch, out1_yaw)
    }

    /// +roll  = left wing up
    /// +pitch = nose up
    /// +yaw   = nose right
    pub fn mix(&self, throttle: f32, roll: f32, pitch: f32, yaw: f32) -> MotorOutputs {
        let front_left = throttle - yaw + pitch + roll; // M3
        let front_right = throttle + yaw + pitch - roll; // M2
        let back_left = throttle + yaw - pitch + roll; // M4
        let back_right = throttle - yaw - pitch - roll; // M1

        let outs = [front_left, front_right, back_left, back_right];
        let excess_output = *outs
            .iter()
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();

        if excess_output > 1.0 {
            return self.mix(throttle - (excess_output - 1.0), roll, pitch, yaw);
        }

        MotorOutputs {
            front_left,
            front_right,
            back_left,
            back_right,
        }
    }
}

#[derive(Debug, Clone, Copy)]
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
#[derive(Debug, Default, Clone, Copy)]
pub struct MotorOutputs {
    front_left:  f32,
    front_right: f32,
    back_left:   f32,
    back_right:  f32,
}

impl MotorOutputs {
    pub fn apply(&self, motors: &mut MotorsPWM) {
        use crate::motors::MotorSelect::*;
        motors.set_motor_f32(Motor1, self.back_right);
        motors.set_motor_f32(Motor2, self.front_right);
        motors.set_motor_f32(Motor3, self.front_left);
        motors.set_motor_f32(Motor4, self.back_left);
    }

    // pub fn mix(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> Self {
    //     Self {
    //         front_left:  0,
    //         front_right: 0,
    //         back_left:   0,
    //         back_right:  0,
    //     }
    // }
}
