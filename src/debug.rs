use cgmath::{Point3, Vector3};
use mesh::Mesh;
use util::*;

pub trait Debug {
    fn add_debug_norm(&mut self, origin: Point3<f32>, norm: Vector3<f32>);
}

impl Debug for Mesh {
    fn add_debug_norm(&mut self, origin: Point3<f32>, norm: Vector3<f32>) {
        let mut m = Mesh::new();
        let quad = make_quad(origin, norm, 0.01, norm);
        let face_id = m.add_positions(quad);
        m.extrude_face(face_id, norm, 0.5);
        self.add_mesh(&m);
    }
}
