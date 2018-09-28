use mesh::Mesh;
use mesh::Id;
use std::collections::HashMap;
use iterator::FaceIterator;
use iterator::FaceHalfedgeIterator;
use util::*;

pub trait Triangulate {
    fn triangulate(&self) -> Self;
}

impl Triangulate for Mesh {
    fn triangulate(&self) -> Self {
        let mut tri_mesh = Mesh::new();
        let mut new_vertices : HashMap<Id, Id> = HashMap::new();
        let mut tri_faces = Vec::new();
        for face_id in FaceIterator::new(self) {
            let mut vertices = Vec::new();
            let first_halfedge_id = self.face_first_halfedge_id(face_id).unwrap();
            for halfedge_id in FaceHalfedgeIterator::new(self, first_halfedge_id) {
                let vertex = self.halfedge_start_vertex(halfedge_id).unwrap();
                new_vertices.entry(vertex.id).or_insert(tri_mesh.add_vertex(vertex.position));
                vertices.push(vertex.id);
            }
            if vertices.len() > 3 {
                let direct = self.face_norm(face_id);
                while vertices.len() > 3 {
                    let mut new_face_generated = false;
                    for i in 0..vertices.len() {
                        let i_next = (i + 1) % vertices.len();
                        let enter = vertices[i];
                        let cone = vertices[i_next];
                        let leave = vertices[(i + 2) % vertices.len()];
                        let cone_v = self.vertex(cone).unwrap();
                        let enter_v = self.vertex(enter).unwrap();
                        let leave_v = self.vertex(leave).unwrap();
                        let angle = angle360(cone_v.position - enter_v.position,
                            leave_v.position - cone_v.position,
                            direct);
                        if angle >= 1.0 && angle <= 179.0 {
                            let mut is_ear = true;
                            for j in 0..(vertices.len() - 3) {
                                let fourth = vertices[(i + 3 + j) % vertices.len()];
                                let fourth_v = self.vertex(fourth).unwrap();
                                if point_in_triangle(enter_v.position, cone_v.position, leave_v.position, fourth_v.position) {
                                    is_ear = false;
                                    break;
                                }
                            }
                            if is_ear {
                                tri_faces.push((enter, cone, leave));
                                vertices.remove(i_next);
                                new_face_generated = true;
                                break;
                            }
                        }
                    }
                    if !new_face_generated {
                        break;
                    }
                }
            }
            if vertices.len() == 3 {
                tri_faces.push((vertices[0], vertices[1], vertices[2]));
            }
        }
        for tri_faces in tri_faces {
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            added_halfedges.push((tri_mesh.add_halfedge(), new_vertices[&tri_faces.0]));
            added_halfedges.push((tri_mesh.add_halfedge(), new_vertices[&tri_faces.1]));
            added_halfedges.push((tri_mesh.add_halfedge(), new_vertices[&tri_faces.2]));
            tri_mesh.add_halfedges_and_vertices(added_halfedges);
        }
        tri_mesh
    }
}
