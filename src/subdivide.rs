use cgmath::Point3;
use cgmath::EuclideanSpace;
use fnv::FnvHashMap;
use mesh::Mesh;
use mesh::Id;
use iterator::FaceIterator;
use iterator::FaceHalfedgeIterator;
use std::mem;

struct FaceData {
    /// The center point of the original face in the input mesh.
    average_of_points: Point3<f32>,

    /// The new vertex in the output mesh.
    generated_vertex_id: Id,
}

struct EdgeData {
    mid_point: Point3<f32>,
    generated_vertex_id: Id,
}

impl EdgeData {
    pub fn new() -> Self {
        EdgeData {
            mid_point: Point3::new(0.0, 0.0, 0.0),
            generated_vertex_id: 0,
        }
    }
}

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

/// Allows efficient usage of the hash map entry API by splitting
/// CatmullClarkSubdivider::Self into multiple borrows.
fn face_data_mut<'a>(
    input: &Mesh,
    id: Id,
    face_data_set: &'a mut FnvHashMap<Id, FaceData>,
    output: &mut Mesh)
-> &'a mut FaceData {
    face_data_set.entry(id).or_insert_with(|| {
        let average_of_points = input.face_center(id);
        FaceData {
            average_of_points,
            generated_vertex_id: output.add_vertex(average_of_points)
        }
    })
}

/// A context for subdivision, providing temporary memory buffers.
pub struct CatmullClarkSubdivider<'a> {
    /// Temporary buffer
    /// TODO: Describe purpose.
    edge_data_set: FnvHashMap<Id, EdgeData>,

    /// Maps FACE ID in the INPUT mesh to FaceData.
    face_data_set: FnvHashMap<Id, FaceData>,

    /// TODO: Investigate if this is needed or if this struct should be consumed
    ///       by .generate() instead.
    finished: bool,

    /// Destination mesh
    generated_mesh: Mesh,

    /// Source mesh
    mesh: &'a Mesh,

    /// Temporary buffer, TODO: Describe purpose.
    vertex_data_set: FnvHashMap<Id, VertexData>,
}

impl<'a> CatmullClarkSubdivider<'a> {
    pub fn new(mesh: &'a Mesh) -> Self {
        CatmullClarkSubdivider {
            edge_data_set: FnvHashMap::default(),   // TODO: Preallocate! :)
            face_data_set: FnvHashMap::default(),   // TODO: Preallocate! :)
            finished: false,
            generated_mesh: Mesh::new(),
            mesh: mesh,
            vertex_data_set: FnvHashMap::default(), // TODO: Preallocate! :)
        }
    }

    fn edge_data_real_mut(&mut self, id: Id) -> &mut EdgeData {
        let mesh = &mut self.mesh;
        let generated_mesh = &mut self.generated_mesh;
        let face_data_set = &mut self.face_data_set;
        let edge_data_set = &mut self.edge_data_set;
        edge_data_set.entry(id).or_insert_with(|| {
            let mid_point = mesh.edge_center(id);
            let (halfedge_face_id, opposite_face_id, next_halfedge_vertex_id, start_vertex_position) = {
                let halfedge = mesh.halfedge(id).unwrap();
                (halfedge.face, 
                    mesh.halfedge(halfedge.opposite).unwrap().face, 
                    mesh.halfedge(halfedge.next).unwrap().vertex,
                    mesh.vertex(halfedge.vertex).unwrap().position)
            };
            let stop_vertex_position = mesh.vertex(next_halfedge_vertex_id).unwrap().position;
            let f1_data_average = face_data_mut(
                mesh,
                halfedge_face_id,
                face_data_set,
                generated_mesh).average_of_points;
            let f2_data_average = face_data_mut(
                mesh,
                opposite_face_id,
                face_data_set,
                generated_mesh).average_of_points;
            let mut data = EdgeData::new();
            data.mid_point = mid_point;
            let center = Point3::centroid(&[
               f1_data_average,
               f2_data_average,
               start_vertex_position,
               stop_vertex_position,
            ]);
            data.generated_vertex_id = generated_mesh.add_vertex(center);
            data
        })
    }

    fn edge_data_mut(&mut self, id: Id) -> &mut EdgeData {
        let peek_id = self.mesh.peek_same_halfedge(id);
        self.edge_data_real_mut(peek_id)
    }

    fn vertex_data_mut(
        &mut self,
        id: Id,
        tmp_avg_of_faces: &mut Vec<Point3<f32>>,
        tmp_avg_of_edge_mids: &mut Vec<Point3<f32>>)
    -> &mut VertexData {
        tmp_avg_of_faces.clear();
        tmp_avg_of_edge_mids.clear();
        let vertex = self.mesh.vertex(id).unwrap();
        for halfedge_id in vertex.halfedges.iter() {
            let halfedge_face_id = self.mesh.halfedge(*halfedge_id).unwrap().face;
            tmp_avg_of_faces.push(face_data_mut(
                &self.mesh,
                halfedge_face_id,
                &mut self.face_data_set,
                &mut self.generated_mesh).average_of_points);
            tmp_avg_of_edge_mids.push(self.edge_data_mut(*halfedge_id).mid_point);
        }
        let bury_center = Point3::centroid(tmp_avg_of_faces);
        let average_of_edge = Point3::centroid(tmp_avg_of_edge_mids);
        let position = (((average_of_edge * 2.0) + bury_center.to_vec()) + 
                        (vertex.position.to_vec() * ((tmp_avg_of_faces.len() as i32 - 3).abs() as f32))) /
            (tmp_avg_of_faces.len() as f32);
        let internal_borrow = &mut self.generated_mesh;
        self.vertex_data_set.entry(id).or_insert_with(|| {
            let mut data = VertexData::new();
            data.generated_vertex_id = internal_borrow.add_vertex(position);
            data
        })
    }

    pub fn generate(&mut self) -> Mesh {
        mem::replace(&mut self.generated_mesh_mut(), Mesh::new())
    }
    
    fn generated_mesh_mut(&mut self) ->&mut Mesh {
        if self.finished {
            return &mut self.generated_mesh;
        }
        {
            // Each halfedge produce 3 new
            let halfedge_prediction = self.mesh.halfedge_count * 4;
            self.generated_mesh.halfedges.reserve(halfedge_prediction);
            self.generated_mesh.vertices.reserve(
                self.mesh.vertex_count         // No vertices are removed
                + self.mesh.halfedge_count / 2 // Each edge produce a new point
                + self.mesh.face_count         // Each face produce a new point
            );
            self.generated_mesh.faces.reserve(
                self.mesh.face_count * 4       // Optimized for quad meshes
            );
            // Is this true for all meshes? If false, this is probably still ok
            // since the worst-case here is degraded performance or 
            // overallocation.
            self.generated_mesh.edges.reserve(halfedge_prediction / 2);
        }
        // Temporary and reusable memory buffers for self.vertex_data_mut().
        let mut tmp_avg_of_faces: Vec<Point3<f32>> = Vec::new();
        let mut tmp_avg_of_edge_mids: Vec<Point3<f32>> = Vec::new();
        for face_id in FaceIterator::new(self.mesh) {
            let face_vertex_id = face_data_mut(
                &self.mesh,
                face_id,
                &mut self.face_data_set,
                &mut self.generated_mesh).generated_vertex_id;
            let face_halfedge = self.mesh.face(face_id).unwrap().halfedge;
            let face_halfedge_id_vec = FaceHalfedgeIterator::new(self.mesh, face_halfedge).into_vec();
            for halfedge_id in face_halfedge_id_vec {
                let (next_halfedge_id, vertex_id) = {
                    let halfedge = self.mesh.halfedge(halfedge_id).unwrap();
                    let next_halfedge_id = halfedge.next;
                    let next_halfedge_start = self.mesh.halfedge(next_halfedge_id).unwrap().vertex;
                    (next_halfedge_id, next_halfedge_start)
                };
                let e1_vertex_id = self.edge_data_mut(halfedge_id).generated_vertex_id;
                let e2_vertex_id = self.edge_data_mut(next_halfedge_id).generated_vertex_id;
                let vertex_generated_id = self.vertex_data_mut(
                    vertex_id,
                    &mut tmp_avg_of_faces,
                    &mut tmp_avg_of_edge_mids).generated_vertex_id;
                let added_face_id = self.generated_mesh.add_face();
                let mut added_halfedges = [
                    (self.generated_mesh.add_halfedge(), face_vertex_id),
                    (self.generated_mesh.add_halfedge(), e1_vertex_id),
                    (self.generated_mesh.add_halfedge(), vertex_generated_id),
                    (self.generated_mesh.add_halfedge(), e2_vertex_id)
                ];
                for &(added_halfedge_id, added_vertex_id) in added_halfedges.iter() {
                    self.generated_mesh.vertex_mut(added_vertex_id).unwrap().halfedges.insert(added_halfedge_id);
                    self.generated_mesh.halfedge_mut(added_halfedge_id).unwrap().face = added_face_id;
                    self.generated_mesh.halfedge_mut(added_halfedge_id).unwrap().vertex = added_vertex_id;
                }
                self.generated_mesh.face_mut(added_face_id).unwrap().halfedge = added_halfedges[0].0;
                for i in 0..added_halfedges.len() {
                    let first = added_halfedges[i].0;
                    let second = added_halfedges[(i + 1) % added_halfedges.len()].0;
                    self.generated_mesh.link_halfedges(first, second);
                }
            }
        }
        self.finished = true;
        &mut self.generated_mesh
    }
}

pub trait Subdivide {
    fn subdivide(&self) -> Self;
}

impl Subdivide for Mesh {
    fn subdivide(&self) -> Self {
        let mut cc = CatmullClarkSubdivider::new(self);
        cc.generate()
    }
}
