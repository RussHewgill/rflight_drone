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
