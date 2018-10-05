extern crate meshlite;

use meshlite::mesh::Mesh;
use meshlite::primitives::cube;
use meshlite::subdivide::Subdivide;

/// Test added for https://github.com/huxingyi/meshlite/pull/2
#[test]
fn verify_cube_subdivision() {
    let mut mesh = cube();
    let sub = mesh.subdivide();
    assert_eq!(26, sub.vertex_count);
    assert_eq!(24, sub.face_count);
}
