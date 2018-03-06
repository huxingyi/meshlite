use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Deg;
use cgmath::Rad;

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

// Modifed from Reza Nourai's C# version: PointInTriangle
// https://blogs.msdn.microsoft.com/rezanour/2011/08/07/barycentric-coordinates-and-point-in-triangle-tests/
pub fn point_in_triangle(a: Point3<f32>, b: Point3<f32>, c: Point3<f32>, p: Point3<f32>) -> bool {
    let u = b - a;
    let v = c - a;
    let w = p - a;
    let v_cross_w = v.cross(w);
    let v_cross_u = v.cross(u);
    if v_cross_w.dot(v_cross_u) < 0.0 {
        return false;
    }
    let u_cross_w = u.cross(w);
    let u_cross_v = u.cross(v);
    if u_cross_w.dot(u_cross_v) < 0.0 {
        return false;
    }
    let denom = u_cross_v.magnitude();
    let r = v_cross_w.magnitude() / denom;
    let t = u_cross_w.magnitude() / denom;
    r + t <= 1.0
}

// Modified from Cyranose's answer
// https://www.opengl.org/discussion_boards/showthread.php/159385-Deriving-angles-from-0-to-360-from-Dot-Product
pub fn angle360(a: Vector3<f32>, b: Vector3<f32>, direct: Vector3<f32>) -> f32 {
    let angle = Rad::acos(a.dot(b));
    let c = a.cross(b);
    if c.dot(direct) < 0.0 {
        180.0 + Deg::from(angle).0
    } else {
        Deg::from(angle).0
    }
}

#[derive(PartialEq)]
#[derive(Debug)]
pub enum PointSide {
    Front,
    Back,
    Coincident
}

pub fn point_side_on_plane(pt: Point3<f32>, pt_on_plane: Point3<f32>, norm: Vector3<f32>) -> PointSide {
    let line = pt - pt_on_plane;
    let dot = line.dot(norm);
    if dot > 0.0 {
        PointSide::Front
    } else if dot < 0.0 {
        PointSide::Back
    } else {
        PointSide::Coincident
    }
}

#[derive(PartialEq)]
#[derive(Debug)]
pub enum SegmentPlaneIntersect {
    NoIntersection,
    Parallel,
    LiesIn,
    Intersection(Point3<f32>),
}

const SMALL_NUM : f32 = 0.00000001;

// Modfied from the C++ version intersect3D_SegmentPlane
// http://geomalgorithms.com/a05-_intersect-1.html
pub fn intersect_of_segment_and_plane(p0: Point3<f32>, p1: Point3<f32>, pt_on_plane: Point3<f32>, norm: Vector3<f32>) -> SegmentPlaneIntersect {
    let u = p1 - p0;
    let w = p0 - pt_on_plane;
    let d = norm.dot(u);
    let n = -norm.dot(w);
    if d.abs() < SMALL_NUM {
        if n == 0.0 {
            return SegmentPlaneIntersect::LiesIn;
        }
        return SegmentPlaneIntersect::Parallel;
    }
    let s_i = n / d;
    if s_i < 0.0 || s_i > 1.0 {
        return SegmentPlaneIntersect::NoIntersection;
    }
    SegmentPlaneIntersect::Intersection(p0 + (s_i * u))
}
