use iterator::FaceHalfedgeIterator;
use iterator::FaceIterator;
use mesh::Id;
use mesh::Mesh;
use util::*;

pub trait Triangulate {
    fn triangulate(&self) -> Self;
}

struct PolygonEdgeCounter {
    /// Each element represents the number of polygons in the mesh with the same
    /// number of edges as the index in this sequence. Eg. index 3 contains the
    /// number of triangles in the mesh, index 4 contains the number of quads in
    /// the mesh, and so on.
    pub polygon_edge_counts: Vec<usize>,
}

impl PolygonEdgeCounter {
    pub fn new() -> Self {
        Self {
            polygon_edge_counts: Vec::new(),
        }
    }

    pub fn add_polygon(&mut self, edge_count: usize) {
        if edge_count >= self.polygon_edge_counts.len() {
            self.polygon_edge_counts.resize(edge_count + 1, 0);
        }
        self.polygon_edge_counts[edge_count] += 1;
    }
}

fn predict_triangle_count(pec: &PolygonEdgeCounter) -> usize {
    let mut sum = 0;
    for i in 3..pec.polygon_edge_counts.len() {
        let polygons = pec.polygon_edge_counts[i];
        sum += polygons * (i - 2);
    }
    sum
}

struct TriangulationPrediction {
    vertex_count: usize,
    triangle_count: usize,
    halfedge_count: usize,
    edge_count: usize,
}

impl TriangulationPrediction {
    pub fn new(input: &Mesh) -> Self {
        let mut pec = PolygonEdgeCounter::new();
        for face_id in FaceIterator::new(input) {
            let halfedge = input.face_first_halfedge_id(face_id).unwrap();
            let count = FaceHalfedgeIterator::new(input, halfedge).count();
            pec.add_polygon(count);
        }
        let triangle_count = predict_triangle_count(&pec);
        let halfedge_count = triangle_count * 3;
        Self {
            triangle_count,
            halfedge_count,
            edge_count: halfedge_count / 2,
            vertex_count: input.vertex_count,
        }
    }
}

impl Triangulate for Mesh {
    /// Triangulate without knowing stats about the input mesh.
    fn triangulate(&self) -> Self {
        let tp = TriangulationPrediction::new(self);
        let mut tri_mesh = Mesh::new();
        tri_mesh.vertices.reserve(tp.vertex_count);
        tri_mesh.faces.reserve(tp.triangle_count);
        tri_mesh.halfedges.reserve(tp.halfedge_count);
        tri_mesh.edges.reserve(tp.edge_count);
        let mut new_vertices: Vec<Option<Id>> =
            vec![None; self.vertices.len() + 1];
        let mut tri_faces = Vec::new();
        tri_faces.reserve(tp.triangle_count);
        let mut vertices = Vec::new();
        for face_id in FaceIterator::new(self) {
            vertices.clear();
            let first_halfedge_id =
                self.face_first_halfedge_id(face_id).unwrap();
            for halfedge_id in
                FaceHalfedgeIterator::new(self, first_halfedge_id)
            {
                let vertex = self.halfedge_start_vertex(halfedge_id).unwrap();
                let new_vert = &mut new_vertices[vertex.id];
                if new_vert.is_none() {
                    *new_vert = Some(tri_mesh.add_vertex(vertex.position));
                }
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
                        let angle = angle360(
                            cone_v.position - enter_v.position,
                            leave_v.position - cone_v.position,
                            direct,
                        );
                        if angle >= 1.0 && angle <= 179.0 {
                            let mut is_ear = true;
                            for j in 0..(vertices.len() - 3) {
                                let fourth =
                                    vertices[(i + 3 + j) % vertices.len()];
                                let fourth_v = self.vertex(fourth).unwrap();
                                if point_in_triangle(
                                    enter_v.position,
                                    cone_v.position,
                                    leave_v.position,
                                    fourth_v.position,
                                ) {
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
        for tf in tri_faces {
            let array = [
                (tri_mesh.add_halfedge(), new_vertices[tf.0].unwrap()),
                (tri_mesh.add_halfedge(), new_vertices[tf.1].unwrap()),
                (tri_mesh.add_halfedge(), new_vertices[tf.2].unwrap()),
            ];
            tri_mesh.add_halfedges_and_vertices(&array);
        }
        tri_mesh
    }
}
