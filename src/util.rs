use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;

pub fn norm(p1: Point3<f32>, p2: Point3<f32>, p3: Point3<f32>) -> Vector3<f32> {
    let v1 = p1 - p2;
    let v2 = p1 - p3;
    let n = Vector3 {x: v1.y * v2.z - v2.y * v1.z, 
        y: v2.x * v1.z - v1.x * v2.z,
        z: v1.x * v2.y - v2.x * v1.y};
    n.normalize();
    return n;
}
