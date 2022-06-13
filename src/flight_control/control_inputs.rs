use core::f32::consts::PI;

use defmt::{println as rprintln, Format};

use crate::math::rad_to_deg;

use super::{ControlRates, FlightLimits};

/// Received from remote control
#[derive(Clone, Copy, Format)]
pub struct ControlInputs {
    /// Range: -1.0 to 1.0
    roll:         f32,
    /// Range: -1.0 to 1.0
    pitch:        f32,
    /// Range: -2Pi to 2Pi
    yaw:          f32,
    /// Range: 0.0 to 1.0
    throttle:     f32,
    /// Toggles
    takeoff:      bool,
    calibrate:    bool,
    motors_armed: bool,
    level_mode:   bool,
}

impl ControlInputs {
    pub fn set_motors_armed(&mut self, armed: bool) {
        self.motors_armed = armed;
    }
    pub fn get_motors_armed(&self) -> bool {
        self.motors_armed
    }
    pub fn get_level_mode(&self) -> bool {
        self.level_mode
    }
}

impl Default for ControlInputs {
    fn default() -> Self {
        Self {
            roll:         0.0,
            pitch:        0.0,
            yaw:          0.0,
            throttle:     0.0,
            takeoff:      false,
            calibrate:    false,
            motors_armed: false,
            level_mode:   true,
        }
    }
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

        // eprintln!("motors_armed = {:?}", motors_armed);

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

/// get_values /// TODO: not correct
impl ControlInputs {
    pub fn get_roll(&self, rates: &ControlRates) -> f32 {
        rates.roll.apply(self.roll)
    }
    pub fn get_pitch(&self, rates: &ControlRates) -> f32 {
        rates.pitch.apply(self.pitch)
    }
    pub fn get_yaw(&self, rates: &ControlRates) -> f32 {
        rates.yaw.apply(self.yaw)
    }
    pub fn get_throttle(&self, rates: &ControlRates) -> f32 {
        rates.throttle.apply(self.throttle)
    }
    pub fn get_raw_throttle(&self) -> f32 {
        self.throttle
    }

    pub fn get_values(
        &self,
        rates: &ControlRates,
        // limits: &FlightLimits,
    ) -> (f32, f32, f32, f32) {
        let roll = rates.roll.apply(self.roll);
        let pitch = rates.pitch.apply(self.pitch);
        let yaw = rates.yaw.apply(self.yaw);
        let throttle = rates.throttle.apply(self.throttle);

        // let roll = limits.limit_roll_rate(roll);
        // let pitch = limits.limit_pitch_rate(pitch);
        // let yaw = limits.limit_yaw_rate(yaw);

        (roll, pitch, yaw, throttle)
        // (self.roll, self.pitch, self.yaw, self.throttle)
    }

    /// gyro is in deg/s, so pid inputs/outputs must be as well
    #[cfg(feature = "nope")]
    pub fn get_values(&self) -> (f32, f32, f32, f32) {
        (
            rad_to_deg(self.roll),
            rad_to_deg(self.pitch),
            rad_to_deg(self.yaw),
            // self.roll,
            // self.pitch,
            // self.yaw,
            self.throttle,
        )
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
        self.yaw = yaw.clamp(-1.0, 1.0);
        // self.yaw = yaw % (2.0 * PI);
    }
    pub fn set_throttle(&mut self, throttle: f32) {
        // self.throttle = throttle.clamp(-1.0, 1.0);
        self.throttle = throttle.clamp(0.0, 1.0);
    }
}
