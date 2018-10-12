extern crate meshlite;

use meshlite::primitives::cube;
use meshlite::subdivide::Subdivide;
use meshlite::triangulate::Triangulate;

/// Test added for https://github.com/huxingyi/meshlite/pull/2
#[test]
fn verify_cube_subdivision() {
    let mesh = cube();
    let sub = mesh.subdivide();
    assert_eq!(26, sub.vertex_count);
    assert_eq!(24, sub.face_count);
}

#[test]
fn verify_cube_triangulation() {
    let cube = cube();
    let tri = cube.triangulate();
    assert_eq!(8, tri.vertex_count);
    assert_eq!(12, tri.face_count);
}
