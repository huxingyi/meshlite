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
        let verts_before = mesh.vertex_count;
        mesh = mesh.subdivide();
        let added_verts = mesh.vertex_count - verts_before;
        let seconds = to_seconds_f64(&now.elapsed());
        let verts_per_second = (added_verts as f64 / seconds).round();
        all_vps.push(verts_per_second);
        println!(
            "{: <9} | {: <9} | {: <9} | {: <9} | {: <9} | {: <9.2}",
            mesh.face_count,
            mesh.vertex_count,
            mesh.halfedge_count,
            mesh.edges.len(),
            verts_per_second,
            seconds * 1000.0
        );
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

