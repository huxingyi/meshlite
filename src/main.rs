extern crate cgmath;

mod mesh;
mod subdiv;
mod iterator;
mod wrap;
mod util;

use mesh::Mesh;
use subdiv::CatmullClarkSubdivider;
use wrap::GiftWrapper;

fn main() {
    println!("Hello, world!");
    let mut m = Mesh::new();
    m.load_obj("/Users/jeremy/Repositories/dust3d/gourd.obj").expect("load file failed");
    let mut sm = CatmullClarkSubdivider::new(&mut m);
    let mut m1 = sm.generated_mesh_mut();
    let mut sm2 = CatmullClarkSubdivider::new(&mut m1);
    let mut m2 = sm2.generated_mesh_mut();
    m2.save_obj("test.obj").expect("save file failed");
    //println!("Mesh debug info: {:?}", m);
}
