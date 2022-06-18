use core::f32::consts::PI;

use defmt::{println as rprintln, Format};

use biquad::*;

use signalo::filters::convolve::{
    savitzky_golay::SavitzkyGolay, Config as ConvolveConfig, Convolve,
};
use signalo::filters::mean::mean::Mean;
use signalo::traits::{Filter, WithConfig};

use crate::{math::rad_to_deg, motors::MotorsPWM};

use super::{ControlRates, FlightLimits};

/// Received from remote control
#[derive(Clone)]
pub struct ControlInputs {
    state: ControlInputState,
    // lowpass_pitch: DirectForm2Transposed<f32>,
    // filter_roll:  Convolve<f32, 3>,
    // filter_pitch: Convolve<f32, 3>,
    // filter_yaw:   Convolve<f32, 3>,
}

/// Instantaneous input settings
#[derive(Clone, Copy, Format)]
pub struct ControlInputState {
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

/// new
impl ControlInputs {
    pub fn new() -> Self {
        // let sampling_freq =

        // let coeffs = Coefficients::<f32>::from_params(
        //     Type::LowPass,
        //     sampling_freq.hz(),
        //     cutoff_freq.hz(),
        //     Q_BUTTERWORTH_F32,
        //     // q_value,
        // )
        //     .unwrap();
        // let lowpass_pitch = DirectForm2Transposed::<f32>::new(coeffs);

        Self {
            state: ControlInputState::default(),
            // lowpass_pitch,
            // filter_roll:  Convolve::savitzky_golay(),
            // filter_pitch: Convolve::savitzky_golay(),
            // filter_yaw:   Convolve::savitzky_golay(),
        }
    }
}

/// update
impl ControlInputs {
    pub fn update(&mut self, data: &[u8]) -> Option<bool> {
        let mut input = if let Some(input) = ControlInputState::deserialize(data) {
            input
        } else {
            rprintln!("ControlInputs update: failed deserialize");
            // let mut input = self.state;
            // input.roll = self.filter_roll.filter(input.roll);
            // input.pitch = self.filter_pitch.filter(input.pitch);
            // input.yaw = self.filter_yaw.filter(input.yaw);
            // self.state = input;
            return None;
        };

        // input.roll = self.filter_roll.filter(input.roll);
        // input.pitch = self.filter_pitch.filter(input.pitch);
        // input.yaw = self.filter_yaw.filter(input.yaw);

        // self.state = input;

        self.state.roll = input.roll;
        self.state.pitch = input.pitch;
        self.state.yaw = input.yaw;
        self.state.throttle = input.throttle;

        if self.state.level_mode != input.level_mode {
            rprintln!("setting level_mode = {:?}", input.level_mode);
            self.state.level_mode = input.level_mode;
        }

        if !self.get_motors_armed() && input.motors_armed {
            return Some(true);
        } else if self.get_motors_armed() && !input.motors_armed {
            return Some(false);
        }
        None
    }
}

/// deserialize
impl ControlInputState {
    pub fn deserialize(data: &[u8]) -> Option<Self> {
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

        Some(Self {
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

/// get_values /// TODO: maybe not correct?
impl ControlInputs {
    pub fn get_roll(&self, rates: &ControlRates) -> f32 {
        rates.roll.apply(self.state.roll)
    }
    pub fn get_pitch(&self, rates: &ControlRates) -> f32 {
        rates.pitch.apply(self.state.pitch)
    }
    pub fn get_yaw(&self, rates: &ControlRates) -> f32 {
        rates.yaw.apply(self.state.yaw)
    }
    pub fn get_throttle(&self, rates: &ControlRates) -> f32 {
        rates.throttle.apply(self.state.throttle)
    }
    pub fn get_raw_throttle(&self) -> f32 {
        self.state.throttle
    }
    pub fn get_motors_armed(&self) -> bool {
        self.state.motors_armed
    }
    pub fn get_level_mode(&self) -> bool {
        self.state.level_mode
    }

    pub fn get_values(
        &self,
        rates: &ControlRates,
        // limits: &FlightLimits,
    ) -> (f32, f32, f32, f32) {
        let roll = rates.roll.apply(self.state.roll);
        let pitch = rates.pitch.apply(self.state.pitch);
        let yaw = rates.yaw.apply(self.state.yaw);
        let throttle = rates.throttle.apply(self.state.throttle);

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
    pub fn set_motors_armed(&mut self, armed: bool) {
        self.state.motors_armed = armed;
    }
    pub fn set_roll(&mut self, roll: f32) {
        self.state.roll = roll.clamp(-1.0, 1.0);
    }
    pub fn set_pitch(&mut self, pitch: f32) {
        self.state.pitch = pitch.clamp(-1.0, 1.0);
    }
    pub fn set_yaw(&mut self, yaw: f32) {
        self.state.yaw = yaw.clamp(-1.0, 1.0);
        // self.state.yaw = yaw % (2.0 * PI);
    }
    pub fn set_throttle(&mut self, throttle: f32) {
        // self.state.throttle = throttle.clamp(-1.0, 1.0);
        self.state.throttle = throttle.clamp(0.0, 1.0);
    }
}

// impl Default for ControlInputs {
//     fn default() -> Self {
//         Self {
//             state: ControlInputState::default(),
//         }
//     }
// }

impl Default for ControlInputState {
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
