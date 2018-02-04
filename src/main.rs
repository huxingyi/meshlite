extern crate cgmath;
extern crate petgraph;

mod mesh;
mod subdiv;
mod iterator;
mod util;
mod wrap;
mod bmesh;

use cgmath::Matrix4;
use cgmath::prelude::*;
use cgmath::Vector3;
use cgmath::Point3;
use cgmath::Deg;
use cgmath::Rad;

use wrap::GiftWrapper;
use subdiv::CatmullClarkSubdivider;

use mesh::Mesh;
use bmesh::Bmesh;

fn cube() -> Mesh {
    let mut m = Mesh::new();
    let face_id = m.add_plane(1.0, 1.0);
    let normal = m.face_norm(face_id);
    m.extrude_face(face_id, normal, 1.0);
    //m.remove_face(face_id);
    m
}

fn plane() -> Mesh {
    let mut m = Mesh::new();
    let face_id = m.add_plane(1.0, 1.0);
    m
}

fn main() {
    //let mut m = Mesh::new();
    //let face_id = m.add_plane(2.0, 1.0);
    //let normal = m.face_norm(face_id);
    //m.extrude_face(face_id, normal, 1.0);
    //m.save_obj("test.obj").expect("save file failed");

    /*
    let mut m1 = cube();
    let v1 = Vector3 {x: 0.0, y: -1.0, z: 0.0};
    let mut mat1 = Matrix4::from_translation(v1);
    let matr = Matrix4::from_angle_x(Rad::from(Deg(-90.0)));
    mat1 = mat1 * matr;
    m1.transform(&mat1);

    let mut m2 = plane();
    let v2 = Vector3 {x: 0.0, y: 2.0, z: 0.0};
    let mut mat2 = Matrix4::from_translation(v2);
    let matr = Matrix4::from_angle_x(Rad::from(Deg(90.0)));
    mat2 = mat2 * matr;
    m2.transform(&mat2);

    let mut m3 = m1 + m2;

    let mut gw = GiftWrapper::new();
    gw.stitch_two_faces(&mut m3, 1, 7);

    m3.save_obj("test.obj").expect("save file failed");
    */

    //m.load_obj("/Users/jeremy/Repositories/dust3d/gourd.obj").expect("load file failed");
    //let mut sm = CatmullClarkSubdivider::new(&mut m);
    //sm.generated_mesh_mut().save_obj("test.obj").expect("save file failed");
    //println!("Mesh debug info: {:?}", m);

    let mut bmesh = Bmesh::new();
    /*
    let node1 = bmesh.add_node(Point3 {x: -1.0, y: 1.5, z: 1.0}, 0.25);
    let node2 = bmesh.add_node(Point3 {x: 0.0, y: 0.0, z: 0.0}, 0.3);
    let node3 = bmesh.add_node(Point3 {x: 1.0, y: -1.5, z: -1.0}, 0.5);
    let node4 = bmesh.add_node(Point3 {x: 1.0, y: 1.5, z: -1.0}, 0.2);
    bmesh.add_edge(node1, node2);
    bmesh.add_edge(node2, node3);
    bmesh.add_edge(node2, node4);
    let mut mesh = bmesh.generate_mesh(node2);
    */
    let node0 = bmesh.add_node(Point3 {x: -2.07575, y: 1.53902, z: 0.04122}, 0.25);                                    
    let node1 = bmesh.add_node(Point3 {x: 2.40837, y: 2.34882, z: 0.48585}, 0.3);
    let node2 = bmesh.add_node(Point3 {x: -0.91403, y: 0.77069, z: 0.62299}, 0.5);         
    let node3 = bmesh.add_node(Point3 {x: 2.25224, y: 0.74973, z: 0.85115}, 0.5);
    let node4 = bmesh.add_node(Point3 {x: 0.0, y: 0.0, z: 0.0}, 0.82);
    let node5 = bmesh.add_node(Point3 {x: 0.00920, y: -0.66115, z: -2.04601}, 0.5);
    let node6 = bmesh.add_node(Point3 {x: 0.01726, y: -0.88224, z: -2.87471}, 0.2);
    let node7 = bmesh.add_node(Point3 {x: 0.0, y: -2.0, z: 0.00}, 0.2);
    let node8 = bmesh.add_node(Point3 {x: -0.3, y: -2.8, z: 0.13}, 0.5);
    let node9 = bmesh.add_node(Point3 {x: -0.3, y: -3.8, z: 1.13}, 0.6);
    bmesh.add_edge(node0, node2);
    bmesh.add_edge(node2, node4);
    bmesh.add_edge(node4, node3);
    bmesh.add_edge(node3, node1);
    bmesh.add_edge(node4, node5);
    bmesh.add_edge(node5, node6);
    bmesh.add_edge(node4, node7);
    bmesh.add_edge(node7, node8);
    bmesh.add_edge(node8, node9);
    let mut mesh = bmesh.generate_mesh(node4);
    mesh.save_obj("test.obj").expect("save file failed");

    //let mut sm = CatmullClarkSubdivider::new(&mut mesh);
    //sm.generated_mesh_mut().save_obj("test.obj").expect("save file failed");
}

