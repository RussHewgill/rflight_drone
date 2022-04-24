pub fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / core::f32::consts::PI
}

pub fn deg_to_rad(deg: f32) -> f32 {
    deg * core::f32::consts::PI / 180.0
}
