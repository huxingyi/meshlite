extern crate meshlite;

use meshlite::mesh::Mesh;
use meshlite::subdivide::Subdivide;

/// Test added for https://github.com/huxingyi/meshlite/pull/2
#[test]
fn verify_cube_subdivision() {
    let mut mesh = cube();
    let sub = mesh.subdivide();
    assert_eq!(26, sub.vertex_count);
    assert_eq!(24, sub.face_count);
}

/// Should this be part of the public API? (currently defined in two other places)
fn cube() -> Mesh {
    let mut m = Mesh::new();
    let face_id = m.add_plane(1.0, 1.0);
    let normal = m.face_norm(face_id);
    m.extrude_face(face_id, normal, 1.0).translate(0.0, 0.0, -0.5);
    m
}
