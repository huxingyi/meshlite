use mesh::Mesh;

pub fn cube() -> Mesh {
    let mut m = Mesh::new();
    let face_id = m.add_plane(1.0, 1.0);
    let normal = m.face_norm(face_id);
    m.extrude_face(face_id, normal, 1.0).translate(0.0, 0.0, -0.5);
    m
}

