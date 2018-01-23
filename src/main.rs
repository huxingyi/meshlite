extern crate cgmath;

mod mesh;
mod subdiv;
mod iterator;
mod util;
mod wrap;

use mesh::Mesh;
use subdiv::CatmullClarkSubdivider;
use wrap::GiftWrapper;

fn main() {
    let mut m = Mesh::new();
    let face_id = m.add_plane(2.0, 1.0);
    let normal = m.face_norm(face_id);
    m.extrude_face(face_id, normal, 1.0);
    m.save_obj("test.obj").expect("save file failed");
    //m.load_obj("/Users/jeremy/Repositories/dust3d/gourd.obj").expect("load file failed");
    //let mut sm = CatmullClarkSubdivider::new(&mut m);
    //sm.generated_mesh_mut().save_obj("test.obj").expect("save file failed");
    //println!("Mesh debug info: {:?}", m);
}
