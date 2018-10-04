use cgmath::Point3;
use cgmath::EuclideanSpace;
use std::collections::HashMap;
use mesh::Mesh;
use mesh::Id;
use iterator::FaceIterator;
use iterator::FaceHalfedgeIterator;
use std::mem;

struct FaceData {
    average_of_points: Point3<f32>,
    generated_vertex_id: Id,
}

impl FaceData {
    pub fn new() -> Self {
        FaceData {
            average_of_points: Point3::new(0.0, 0.0, 0.0),
            generated_vertex_id: 0,
        }
    }
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

pub struct CatmullClarkSubdivider<'a> {
    mesh: &'a Mesh,
    face_data_set: HashMap<Id, Box<FaceData>>,
    edge_data_set: HashMap<Id, Box<EdgeData>>,
    vertex_data_set: HashMap<Id, Box<VertexData>>,
    generated_mesh: Mesh,
    finished: bool,
}

impl<'a> CatmullClarkSubdivider<'a> {
    pub fn new(mesh: &'a Mesh) -> Self {
        CatmullClarkSubdivider {
            mesh: mesh,
            face_data_set: HashMap::new(),
            edge_data_set: HashMap::new(),
            vertex_data_set: HashMap::new(),
            generated_mesh: Mesh::new(),
            finished: false,
        }
    }

    fn face_data_mut(&mut self, id: Id) -> &mut FaceData {
        let center = self.mesh.face_center(id);
        let internal_borrow = &mut self.generated_mesh;
        self.face_data_set.entry(id).or_insert_with(|| {
            let mut data = FaceData::new();
            data.average_of_points = center;
            data.generated_vertex_id = internal_borrow.add_vertex(center);
            Box::new(data)
        })
    }

    fn edge_data_real_mut(&mut self, id: Id) -> &mut EdgeData {
        let mid_point = self.mesh.edge_center(id);
        let (halfedge_face_id, opposite_face_id, next_halfedge_vertex_id, start_vertex_position) = {
            let halfedge = self.mesh.halfedge(id).unwrap();
            (halfedge.face, 
                self.mesh.halfedge(halfedge.opposite).unwrap().face, 
                self.mesh.halfedge(halfedge.next).unwrap().vertex,
                self.mesh.vertex(halfedge.vertex).unwrap().position)
        };
        let stop_vertex_position = self.mesh.vertex(next_halfedge_vertex_id).unwrap().position;
        let f1_data_average = self.face_data_mut(halfedge_face_id).average_of_points;
        let f2_data_average = self.face_data_mut(opposite_face_id).average_of_points;
        let array = vec![
           f1_data_average,
           f2_data_average,
           start_vertex_position,
           stop_vertex_position,
        ];
        let internal_borrow = &mut self.generated_mesh;
        self.edge_data_set.entry(id).or_insert_with(|| {
            let mut data = EdgeData::new();
            data.mid_point = mid_point;
            let center = Point3::centroid(&array);
            data.generated_vertex_id = internal_borrow.add_vertex(center);
            Box::new(data)
        })
    }

    fn edge_data_mut(&mut self, id: Id) -> &mut EdgeData {
        let peek_id = self.mesh.peek_same_halfedge(id);
        self.edge_data_real_mut(peek_id)
    }

    fn vertex_data_mut(&mut self, id: Id) -> &mut VertexData {
        let mut average_of_faces = Vec::new();
        let mut average_of_edge_mids = Vec::new();
        let mut vec = Vec::new();
        let vertex_position = self.mesh.vertex(id).unwrap().position;
        {
            let mut edge_iter = self.mesh.vertex(id).unwrap().halfedges.iter();
            while let Some(&halfedge_id) = edge_iter.next() {
                let halfedge_face_id = self.mesh.halfedge(halfedge_id).unwrap().face;
                vec.push((halfedge_id, halfedge_face_id));
            }
        }
        for (halfedge_id, halfedge_face_id) in vec {
            average_of_faces.push(self.face_data_mut(halfedge_face_id).average_of_points);
            average_of_edge_mids.push(self.edge_data_mut(halfedge_id).mid_point);
        }
        let bury_center = Point3::centroid(&average_of_faces);
        let average_of_edge = Point3::centroid(&average_of_edge_mids);
        let position = (((average_of_edge * 2.0) + bury_center.to_vec()) + 
                        (vertex_position.to_vec() * ((average_of_faces.len() as i32 - 3).abs() as f32))) / 
            (average_of_faces.len() as f32);
        let internal_borrow = &mut self.generated_mesh;
        self.vertex_data_set.entry(id).or_insert_with(|| {
            let mut data = VertexData::new();
            data.generated_vertex_id = internal_borrow.add_vertex(position);
            Box::new(data)
        })
    }

    pub fn generate(&mut self) -> Mesh {
        mem::replace(&mut self.generated_mesh_mut(), Mesh::new())
    }

    fn generated_mesh_mut(&mut self) ->&mut Mesh {
        if self.finished {
            return &mut self.generated_mesh;
        }
        let face_id_vec = FaceIterator::new(self.mesh).into_vec();
        for face_id in face_id_vec {
            let face_vertex_id = self.face_data_mut(face_id).generated_vertex_id;
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
                let vertex_generated_id = self.vertex_data_mut(vertex_id).generated_vertex_id;
                let added_face_id = self.generated_mesh.add_face();
                let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
                added_halfedges.push((self.generated_mesh.add_halfedge(), face_vertex_id));
                added_halfedges.push((self.generated_mesh.add_halfedge(), e1_vertex_id));
                added_halfedges.push((self.generated_mesh.add_halfedge(), vertex_generated_id));
                added_halfedges.push((self.generated_mesh.add_halfedge(), e2_vertex_id));
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
