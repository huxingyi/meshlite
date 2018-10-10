extern crate meshlite;

use meshlite::primitives::cube;
use meshlite::subdivide::Subdivide;
use meshlite::triangulate::Triangulate;
use std::time::{Duration, Instant};
use std::vec::Vec;

fn main() {
    let mut quad_mesh = cube();
    let mut all_vps = Vec::new(); // Vertices Per Second
    println!(concat!(
        "faces     | ",
        "vertices  | ",
        "halfedges | ",
        "edges     | ",
        "verts/s   | ",
        "time (ms)"
    ));
    println!("----------+-----------+-----------+-----------+-----------+-----------");
    let last = 8;
    for i in 0..=last {
        let now = Instant::now();
        let tri_mesh = quad_mesh.triangulate();
        let seconds = to_seconds_f64(&now.elapsed());
        let verts_per_second = (tri_mesh.vertex_count as f64 / seconds).round();
        all_vps.push(verts_per_second);
        println!(
            "{: <9} | {: <9} | {: <9} | {: <9} | {: <9} | {: <9.2}",
            tri_mesh.face_count,
            tri_mesh.vertex_count,
            tri_mesh.halfedge_count,
            tri_mesh.edges.len(),
            verts_per_second,
            seconds * 1000.0
        );
        if i != last {
            quad_mesh = quad_mesh.subdivide();
        }
    }
    println!(
        "Vertices per second, min: {}, max: {}, avg: {}",
        all_vps.iter().cloned().fold(0. / 0., f64::min),
        all_vps.iter().cloned().fold(0. / 0., f64::max),
        (all_vps.iter().sum::<f64>() / all_vps.len() as f64).round()
    );
}

fn to_seconds_f64(d: &Duration) -> f64 {
    d.as_secs() as f64 + d.subsec_nanos() as f64 * 1e-9
}

