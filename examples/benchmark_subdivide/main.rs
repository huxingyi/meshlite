extern crate meshlite;

use meshlite::primitives::cube;
use meshlite::subdivide::Subdivide;
use std::time::{Duration, Instant};
use std::vec::Vec;

fn main() {
    let mut mesh = cube();
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
    for _ in 0..9 {
        let now = Instant::now();
        let new_mesh = mesh.subdivide();
        let seconds = to_seconds_f64(&now.elapsed());
        let verts_per_second = (new_mesh.vertex_count as f64 / seconds).round();
        all_vps.push(verts_per_second);
        println!(
            "{: <9} | {: <9} | {: <9} | {: <9} | {: <9} | {: <9.2}",
            new_mesh.face_count,
            new_mesh.vertex_count,
            new_mesh.halfedge_count,
            new_mesh.edges.len(),
            verts_per_second,
            seconds * 1000.0
        );
        mesh = new_mesh; // drop the old mesh outside the benchmark timer
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

