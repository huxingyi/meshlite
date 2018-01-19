extern crate cgmath;

mod mesh;
mod subdiv;
mod iterator;

use mesh::Mesh;
use subdiv::CatmullClarkSubdivider;

fn main() {
    println!("Hello, world!");
    let mut m = Mesh::new();
    m.load_obj("/Users/jeremy/Repositories/dust3d/gourd.obj").expect("load file failed");
    let mut sm = CatmullClarkSubdivider::new(&mut m);
    sm.generated_mesh_mut().save_obj("test.obj").expect("save file failed");
    //println!("Mesh debug info: {:?}", m);
}
