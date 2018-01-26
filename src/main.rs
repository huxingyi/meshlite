extern crate cgmath;

mod mesh;
mod subdiv;
mod iterator;
mod util;
mod wrap;

use cgmath::Matrix4;
use cgmath::prelude::*;
use cgmath::Vector3;
use cgmath::Deg;
use cgmath::Rad;

use wrap::GiftWrapper;

use mesh::Mesh;

fn cube() -> Mesh {
    let mut m = Mesh::new();
    let face_id = m.add_plane(1.0, 1.0);
    let normal = m.face_norm(face_id);
    m.extrude_face(face_id, normal, 1.0);
    //m.remove_face(face_id);
    m
}

fn main() {
    //let mut m = Mesh::new();
    //let face_id = m.add_plane(2.0, 1.0);
    //let normal = m.face_norm(face_id);
    //m.extrude_face(face_id, normal, 1.0);
    //m.save_obj("test.obj").expect("save file failed");

    let mut m1 = cube();
    let v1 = Vector3 {x: 0.0, y: -1.0, z: 0.0};
    let mut mat1 = Matrix4::from_translation(v1);
    let matr = Matrix4::from_angle_x(Rad::from(Deg(-90.0)));
    mat1 = mat1 * matr;
    m1.transform(&mat1);

    let mut m2 = cube();
    let v2 = Vector3 {x: 0.0, y: 2.0, z: 0.0};
    let mut mat2 = Matrix4::from_translation(v2);
    let matr = Matrix4::from_angle_x(Rad::from(Deg(90.0)));
    mat2 = mat2 * matr;
    m2.transform(&mat2);

    let mut m3 = m1 + m2;

    let mut gw = GiftWrapper::new();
    gw.stitch_two_faces(&mut m3, 1, 7);

    m3.save_obj("test.obj").expect("save file failed");

    //m.load_obj("/Users/jeremy/Repositories/dust3d/gourd.obj").expect("load file failed");
    //let mut sm = CatmullClarkSubdivider::new(&mut m);
    //sm.generated_mesh_mut().save_obj("test.obj").expect("save file failed");
    //println!("Mesh debug info: {:?}", m);
}
