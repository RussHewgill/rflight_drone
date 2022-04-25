use nalgebra::{Quaternion, UnitQuaternion, Vector2, Vector3};

pub fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / core::f32::consts::PI
}

pub fn deg_to_rad(deg: f32) -> f32 {
    deg * core::f32::consts::PI / 180.0
}

pub fn quat_mult_vec(q: Quaternion<f32>, v: Vector3<f32>) -> Quaternion<f32> {
    let q = q.coords;
    let mut result = Quaternion::default();
    result.coords.w = -q.x * v.x - q.y * v.y - q.z * v.z;
    result.coords.x = q.w * v.x + q.y * v.z - q.z * v.y;
    result.coords.y = q.w * v.y - q.x * v.z + q.z * v.x;
    result.coords.z = q.w * v.z + q.x * v.y - q.y * v.x;
    result
}
