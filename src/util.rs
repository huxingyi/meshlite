use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Deg;

pub fn norm(p1: Point3<f32>, p2: Point3<f32>, p3: Point3<f32>) -> Vector3<f32> {
    let side1 = p2 - p1;
    let side2 = p3 - p1;
    let perp = side1.cross(side2);
    perp.normalize()
}

pub fn almost_eq(v1: Vector3<f32>, v2: Vector3<f32>) -> bool {
    (v1.x - v2.x).abs() <= 0.01 &&
        (v1.y - v2.y).abs() <= 0.01 &&
        (v1.z - v2.z).abs() <= 0.01
}
