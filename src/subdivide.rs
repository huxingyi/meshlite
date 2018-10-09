use cgmath::EuclideanSpace;
use cgmath::Point3;
use iterator::FaceHalfedgeIterator;
use iterator::FaceIterator;
use mesh::Id;
use mesh::Mesh;

/// Derives Clone to allow initializing a vec with the vec![value; length]
/// macro.
#[derive(Clone)]
struct FaceData {
    /// The center point of the original face in the input mesh.
    average_of_points: Point3<f32>,

    /// The new vertex in the output mesh.
    generated_vertex_id: Id,
}

/// Derives Clone to allow initializing a vec with the vec![value; length]
/// macro.
#[derive(Clone)]
struct EdgeData {
    mid_point: Point3<f32>,
    generated_vertex_id: Id,
}

// A vertex Id in the output mesh.
// This is a simple id wrapper so we derive value semantics.
#[derive(Copy, Clone)]
struct VertexData {
    generated_vertex_id: Id,
}

impl VertexData {
    pub fn new() -> Self {
        VertexData {
            generated_vertex_id: 0,
        }
    }
}

fn face_data_mut<'a>(
    input: &Mesh,
    id: Id,
    face_data_set: &'a mut Vec<Option<FaceData>>,
    output: &mut Mesh,
) -> &'a mut FaceData {
    if face_data_set[id].is_some() {
        return face_data_set[id].as_mut().unwrap();
    }
    let average_of_points = input.face_center(id);
    face_data_set[id] = Some(FaceData {
        average_of_points,
        generated_vertex_id: output.add_vertex(average_of_points),
    });
    face_data_set[id].as_mut().unwrap()
}

fn edge_data_mut<'a>(
    input: &Mesh,
    id: Id,
    face_data_set: &mut Vec<Option<FaceData>>,
    edge_data_set: &'a mut Vec<Option<EdgeData>>,
    output: &mut Mesh,
) -> &'a mut EdgeData {
    let id = input.peek_same_halfedge(id);
    if edge_data_set[id].is_some() {
        return edge_data_set[id].as_mut().unwrap();
    }
    let mid_point = input.edge_center(id);
    let (
        halfedge_face_id,
        opposite_face_id,
        next_halfedge_vertex_id,
        start_vertex_position,
    ) = {
        let halfedge = input.halfedge(id).unwrap();
        (
            halfedge.face,
            input.halfedge(halfedge.opposite).unwrap().face,
            input.halfedge(halfedge.next).unwrap().vertex,
            input.vertex(halfedge.vertex).unwrap().position,
        )
    };
    let stop_vertex_position =
        input.vertex(next_halfedge_vertex_id).unwrap().position;
    let f1_data_average =
        face_data_mut(input, halfedge_face_id, face_data_set, output)
            .average_of_points;
    let f2_data_average =
        face_data_mut(input, opposite_face_id, face_data_set, output)
            .average_of_points;
    let center = Point3::centroid(&[
        f1_data_average,
        f2_data_average,
        start_vertex_position,
        stop_vertex_position,
    ]);
    edge_data_set[id] = Some(EdgeData {
        mid_point,
        generated_vertex_id: output.add_vertex(center),
    });
    edge_data_set[id].as_mut().unwrap()
}

/// A context for subdivision, providing temporary memory buffers.
pub struct CatmullClarkSubdivider<'a> {
    /// Maps HALFEDGE ID in the input mesh to EdgeData.
    edge_data_set: Vec<Option<EdgeData>>,

    /// Maps FACE ID in the INPUT mesh to FaceData.
    face_data_set: Vec<Option<FaceData>>,

    /// Destination mesh
    output: Mesh,

    /// Source mesh
    input: &'a Mesh,

    // Temporary and reusable memory buffer for vertex_data_mut().
    tmp_avg_of_faces: Vec<Point3<f32>>,

    // Temporary and reusable memory buffer for vertex_data_mut().
    tmp_avg_of_edge_mids: Vec<Point3<f32>>,

    /// Maps VERTEX ID in the INPUT mesh to VertexData.
    vertex_data_set: Vec<Option<VertexData>>,
}

impl<'a> CatmullClarkSubdivider<'a> {
    pub fn new(input: &'a Mesh) -> Self {
        CatmullClarkSubdivider {
            edge_data_set: Vec::new(),
            face_data_set: Vec::new(),
            output: Mesh::new(),
            input,
            tmp_avg_of_edge_mids: Vec::new(),
            tmp_avg_of_faces: Vec::new(),
            vertex_data_set: Vec::new(),
        }
    }

    pub fn generate(mut self) -> Mesh {
        self.reserve_internal_memory();
        for face_id in FaceIterator::new(self.input) {
            let face_vertex_id = face_data_mut(
                &self.input,
                face_id,
                &mut self.face_data_set,
                &mut self.output,
            ).generated_vertex_id;
            let face_halfedge = self.input.face(face_id).unwrap().halfedge;
            let face_halfedge_id_vec =
                FaceHalfedgeIterator::new(self.input, face_halfedge).into_vec();
            for halfedge_id in face_halfedge_id_vec {
                let (next_halfedge_id, vertex_id) = {
                    let halfedge = self.input.halfedge(halfedge_id).unwrap();
                    let next_halfedge_id = halfedge.next;
                    let next_halfedge_start =
                        self.input.halfedge(next_halfedge_id).unwrap().vertex;
                    (next_halfedge_id, next_halfedge_start)
                };
                let e1_vertex_id =
                    self.edge_data_mut(halfedge_id).generated_vertex_id;
                let e2_vertex_id =
                    self.edge_data_mut(next_halfedge_id).generated_vertex_id;
                let vertex_generated_id =
                    self.vertex_data_mut(vertex_id).generated_vertex_id;
                let added_face_id = self.output.add_face();
                let mut added_halfedges = [
                    (self.output.add_halfedge(), face_vertex_id),
                    (self.output.add_halfedge(), e1_vertex_id),
                    (self.output.add_halfedge(), vertex_generated_id),
                    (self.output.add_halfedge(), e2_vertex_id),
                ];
                for &(added_halfedge_id, added_vertex_id) in
                    added_halfedges.iter()
                {
                    self.output
                        .vertex_mut(added_vertex_id)
                        .unwrap()
                        .halfedges
                        .insert(added_halfedge_id);
                    self.output.halfedge_mut(added_halfedge_id).unwrap().face =
                        added_face_id;
                    self.output
                        .halfedge_mut(added_halfedge_id)
                        .unwrap()
                        .vertex = added_vertex_id;
                }
                self.output.face_mut(added_face_id).unwrap().halfedge =
                    added_halfedges[0].0;
                for i in 0..added_halfedges.len() {
                    let first = added_halfedges[i].0;
                    let second =
                        added_halfedges[(i + 1) % added_halfedges.len()].0;
                    self.output.link_halfedges(first, second);
                }
            }
        }
        self.output
    }

    /// Should be called once, internally, at subdivision start.
    fn reserve_internal_memory(&mut self) {
        // Each halfedge produce 3 new
        let halfedge_prediction = self.input.halfedge_count * 4;
        self.output.halfedges.reserve(halfedge_prediction);

        self.output.vertices.reserve(
            self.input.vertex_count         // No vertices are removed
            + self.input.halfedge_count / 2 // Each edge produce a new point
            + self.input.face_count, // Each face produce a new point
        );
        self.output.faces.reserve(
            self.input.face_count * 4, // Optimize for quads
        );

        // Is this true for all meshes? If false, this is probably still ok
        // since the worst-case here is degraded performance or
        // overallocation.
        self.output.edges.reserve(halfedge_prediction / 2);

        // input.rs is using 1-based indexing so we need + 1 for the length of
        // each Vec below. It is also not enough to use input.halfedge_count,
        // the count represents "living" elements and may be less than the
        // largest id (index).
        //
        // Using Vecs here may consume more memory compared to hash maps when
        // the source mesh has been heavily edited with many deletions, but
        // should be faster in most cases.
        self.face_data_set = vec![None; self.input.faces.len() + 1];
        self.edge_data_set = vec![None; self.input.halfedges.len() + 1];
        self.vertex_data_set = vec![None; self.input.vertices.len() + 1];
    }

    /// Helps to reduce the syntax noise when a Self is available. Splits Self
    /// into multiple mutable borrows.
    fn edge_data_mut(&mut self, halfedge_id: Id) -> &EdgeData {
        edge_data_mut(
            &self.input,
            halfedge_id,
            &mut self.face_data_set,
            &mut self.edge_data_set,
            &mut self.output,
        )
    }

    /// Get or create a vertex in the new mesh.
    /// The vertex_id paremeter refers to a vertex in the input mesh.
    /// The returned VertexData (vertex id) refers to the output mesh.
    fn vertex_data_mut(&mut self, vertex_id: Id) -> VertexData {
        if let Some(data) = self.vertex_data_set[vertex_id] {
            return data;
        }
        self.tmp_avg_of_faces.clear();
        self.tmp_avg_of_edge_mids.clear();
        let vertex = self.input.vertex(vertex_id).unwrap();
        for halfedge_id in vertex.halfedges.iter() {
            let halfedge_face_id =
                self.input.halfedge(*halfedge_id).unwrap().face;
            self.tmp_avg_of_faces.push(
                face_data_mut(
                    &self.input,
                    halfedge_face_id,
                    &mut self.face_data_set,
                    &mut self.output,
                ).average_of_points,
            );
            self.tmp_avg_of_edge_mids.push(
                edge_data_mut(
                    &self.input,
                    *halfedge_id,
                    &mut self.face_data_set,
                    &mut self.edge_data_set,
                    &mut self.output,
                ).mid_point,
            );
        }
        let barycenter = Point3::centroid(&self.tmp_avg_of_faces);
        let average_of_edge = Point3::centroid(&self.tmp_avg_of_edge_mids);
        let position = (((average_of_edge * 2.0) + barycenter.to_vec())
            + (vertex.position.to_vec()
                * ((self.tmp_avg_of_faces.len() as i32 - 3).abs() as f32)))
            / (self.tmp_avg_of_faces.len() as f32);
        let mut data = VertexData::new();
        data.generated_vertex_id = self.output.add_vertex(position);
        self.vertex_data_set[vertex_id] = Some(data);
        data
    }
}

pub trait Subdivide {
    fn subdivide(&self) -> Self;
}

impl Subdivide for Mesh {
    fn subdivide(&self) -> Self {
        CatmullClarkSubdivider::new(self).generate()
    }
}
