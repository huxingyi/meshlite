use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Deg;

pub fn norm(p1: Point3<f32>, p2: Point3<f32>, p3: Point3<f32>) -> Vector3<f32> {
    let v1 = p1 - p2;
    let v2 = p1 - p3;
    let n = Vector3 {x: v1.y * v2.z - v2.y * v1.z, 
        y: v2.x * v1.z - v1.x * v2.z,
        z: v1.x * v2.y - v2.x * v1.y};
    n.normalize();
    return n;
}

/*
pub fn angle(v1: Vector3<f32>, v2: Vector3<f32>) -> f32 {
    let dot = v1.dot(v2);
    let mag1 = v1.magnitude();
    let mag2 = v2.magnitude();
    let mag1_x_mag2 = mag1 * mag2;
    if mag1_x_mag2 == 0.0 {
        return 0.0;
    }
    let mut dot_div_mags = dot / mag1_x_mag2;
    if dot_div_mags < -1.0 {
        dot_div_mags = -1.0;
    } else if dot_div_mags > 1.0 {
        dot_div_mags = 1.0;
    }
    Deg::acos(dot_div_mags).0
}
*/
