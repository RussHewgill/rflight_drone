/// Received from remote control
#[derive(Debug, Default, Clone, Copy)]
pub struct ControlInputs {
    pub roll:     u32,
    pub pitch:    u32,
    pub yaw:      u32,
    pub throttle: u32,
}

/// Output of PID after mixing
/// PWM values: from 0-1900
#[derive(Debug, Default, Clone, Copy)]
pub struct MotorOutputs {
    front_left:  u16,
    front_right: u16,
    back_left:   u16,
    back_right:  u16,
}

impl MotorOutputs {
    pub fn mix(thrust: f32, roll: f32, pitch: f32, yaw: f32) -> Self {
        Self {
            front_left:  0,
            front_right: 0,
            back_left:   0,
            back_right:  0,
        }
    }
}
