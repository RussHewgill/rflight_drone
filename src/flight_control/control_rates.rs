use defmt::{println as rprintln, Format};

use crate::misc::Axis;

#[derive(Default, Clone, Copy, Format)]
pub struct ControlRates {
    pub roll:     ControlRateAxis,
    pub pitch:    ControlRateAxis,
    pub yaw:      ControlRateAxis,
    pub throttle: ControlRateThrottle,
}

impl ControlRates {
    pub fn get_axis(&self, axis: Axis) -> Option<&ControlRateAxis> {
        match axis {
            Axis::Roll => Some(&self.roll),
            Axis::Pitch => Some(&self.pitch),
            Axis::Yaw => Some(&self.yaw),
            Axis::Throttle => None,
        }
    }
}

#[derive(Clone, Copy, Format)]
pub struct ControlRateAxis {
    pub rc_rate:    f32,
    pub super_rate: f32,
    pub rc_expo:    f32,
}

impl Default for ControlRateAxis {
    fn default() -> Self {
        Self {
            rc_rate:    1.0,
            super_rate: 0.0,
            rc_expo:    0.0,
        }
    }
}

impl ControlRateAxis {
    pub fn apply(&self, input: f32) -> f32 {
        use nalgebra::ComplexField;
        /// BF rcCommand factor
        let q = (input.powi(4) * self.rc_expo) + input * (1.0 - self.rc_expo);
        /// BF expo factor
        let r = 200.0 * q * self.rc_rate;
        /// BF super factor
        let p = 1.0 / (1.0 - (input * self.super_rate));
        r * p
    }
}

#[derive(Clone, Copy, Format)]
pub struct ControlRateThrottle {
    pub mid:  f32,
    pub expo: f32,
}

impl Default for ControlRateThrottle {
    fn default() -> Self {
        Self {
            mid:  0.5,
            expo: 0.0,
        }
    }
}

impl ControlRateThrottle {
    pub fn apply(&self, input: f32) -> f32 {
        input
    }
}

#[derive(Clone, Copy, Format)]
pub struct FlightLimits {
    pub max_pitch: f32,
    pub max_roll:  f32,

    pub max_rate_roll:  f32,
    pub max_rate_pitch: f32,
    pub max_rate_yaw:   f32,
}

impl Default for FlightLimits {
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

impl FlightLimits {
    pub fn get_roll_angle(&self, input: f32) -> f32 {
        // debug_assert!(input >= -1.0);
        // debug_assert!(input <= 1.0);
        (input * self.max_roll).clamp(-self.max_roll, self.max_roll)
    }
    pub fn get_pitch_angle(&self, input: f32) -> f32 {
        // debug_assert!(input >= -1.0);
        // debug_assert!(input <= 1.0);
        (input * self.max_pitch).clamp(-self.max_pitch, self.max_pitch)
    }
}

impl FlightLimits {
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
