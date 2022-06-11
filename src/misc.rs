use defmt::{println as rprintln, Format};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Format)]
pub enum Axis {
    Roll,
    Pitch,
    Yaw,
    Throttle,
}
