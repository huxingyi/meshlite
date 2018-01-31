use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Deg;

use std::collections::HashMap;
use std::collections::LinkedList;
use iterator::FaceHalfedgeIterator;
use iterator::FaceHalfedgeCollection;
use mesh::Id;
use mesh::Mesh;
use util::*;

#[derive(Hash, Eq, PartialEq, Debug)]
struct WrapItemKey {
    p1: usize,
    p2: usize,
}

#[derive(Clone)]
pub struct WrapItem {
    base_normal: Vector3<f32>,
    pub p1: usize,
    pub p2: usize,
    pub p3: usize,
    processed: bool,
}

pub struct Face3 {
    pub p1: usize,
    pub p2: usize,
    pub p3: usize,
}

#[derive(Clone)]
pub struct SourceVertex {
    pub position: Point3<f32>,
    pub source_plane: Id,
    pub index: usize,
    pub tag: Id,
}

#[derive(Hash, Eq, PartialEq, Debug)]
struct RoundVector3 {
    round_x: i32,
    round_y: i32,
    round_z: i32,
}

impl RoundVector3 {
    pub fn new(vec: Vector3<f32>) -> Self {
        RoundVector3 {
            round_x: (vec.x * 100.0).round() as i32,
            round_y: (vec.y * 100.0).round() as i32,
            round_z: (vec.z * 100.0).round() as i32,
        }
    }
}

pub struct GiftWrapper {
    items: Vec<WrapItem>,
    items_map: HashMap<WrapItemKey, usize>,
    items_list: LinkedList<usize>,
    candidates: Vec<usize>,
    pub source_vertices: Vec<SourceVertex>,
    pub generated_faces: Vec<Face3>,
    generated_face_edges_map: HashMap<WrapItemKey, bool>,
    generated_face_norm_map: HashMap<RoundVector3, Vec<usize>>,
    generated_vertex_edges_map: HashMap<usize, Vec<usize>>,
}

impl GiftWrapper {
    pub fn new() -> Self {
        GiftWrapper {
            items: Vec::new(),
            items_map: HashMap::new(),
            items_list: LinkedList::new(),
            source_vertices: Vec::new(),
            candidates: Vec::new(),
            generated_faces: Vec::new(),
            generated_face_edges_map: HashMap::new(),
            generated_face_norm_map: HashMap::new(),
            generated_vertex_edges_map: HashMap::new(),
        }
    }

    pub fn add_source_vertex(&mut self, position: Point3<f32>, source_plane: Id, tag: Id) -> usize {
        let added_index = self.source_vertices.len();
        self.source_vertices.push(SourceVertex {position: position, source_plane: source_plane, tag: tag, index: added_index});
        self.candidates.push(added_index);
        added_index
    }

    fn calculate_face_vector(&self, p1: usize, p2: usize, base_normal: Vector3<f32>) -> Vector3<f32> {
        let v1 = &self.source_vertices[p1];
        let v2 = &self.source_vertices[p2];
        let seg = v2.position - v1.position;
        seg.cross(base_normal)
    }

    fn add_item(&mut self, p1: usize, p2: usize, base_normal: Vector3<f32>) {
        {
            let v1 = &self.source_vertices[p1];
            let v2 = &self.source_vertices[p2];
            if !self.items.is_empty() && v1.source_plane == v2.source_plane {
                return;
            }
        }
        if !self.find_item(p1, p2).is_none() || !self.find_item(p2, p1).is_none() {
            return;
        }
        if self.is_edge_generated(p1, p2) || self.is_edge_generated(p2, p1) {
            return;
        }
        let index = self.items.len();
        self.items.push(WrapItem {p3: 0, p1: p1, p2: p2, base_normal: base_normal, processed: false});
        self.items_map.insert(WrapItemKey {p1: p1, p2: p2}, index);
        self.items_list.push_front(index);
    }

    pub fn find_item(&self, p1: usize, p2: usize) -> Option<&usize> {
        let key = WrapItemKey {p1: p1, p2: p2};
        self.items_map.get(&key)
    }

    pub fn add_startup(&mut self, p1: usize, p2: usize, base_normal: Vector3<f32>) {
        if self.items.len() == 0 {
            self.add_item(p1, p2, base_normal);
        }
        self.generated_face_edges_map.insert(WrapItemKey {p1: p2, p2: p1}, true);
    }

    fn is_edge_generated(&self, p1: usize, p2: usize) -> bool {
        let key = WrapItemKey {p1: p1, p2: p2};
        if self.generated_face_edges_map.get(&key).is_none() {
            return false;
        }
        true
    }

    fn angle_of_base_face_and_point(&self, item_index: usize, vertex_index: usize) -> f32 {
        let item = &self.items[item_index].clone();
        if item.p1 == vertex_index || item.p2 == vertex_index {
            return 0.0;
        }
        let v1 = &self.source_vertices[item.p1].clone();
        let v2 = &self.source_vertices[item.p2].clone();
        let vp = &self.source_vertices[vertex_index].clone();
        if v1.source_plane == v2.source_plane && v1.source_plane == vp.source_plane {
            return 0.0;
        }
        let vd1 = self.calculate_face_vector(item.p1, item.p2, item.base_normal);
        let normal = norm(v2.position, v1.position, vp.position);
        let vd2 = self.calculate_face_vector(item.p1, item.p2, normal);
        let angle = Deg::from(vd2.angle(vd1));
        angle.0
    }

    fn find_best_vertex_on_the_left(&mut self, item_index: usize) -> Option<usize> {
        let mut max_angle = 0 as f32;
        let mut choosen_it = None;
        let mut rm_vec : Vec<usize> = Vec::new();
        for (i, &it) in self.candidates.iter().enumerate() {
            if self.is_vertex_closed(it) {
                rm_vec.push(i);
                continue;
            }
            let mut angle = self.angle_of_base_face_and_point(item_index, it);
            if angle > max_angle {
                max_angle = angle;
                choosen_it = Some(it);
            }
        }
        for &i in rm_vec.iter().rev() {
            self.candidates.swap_remove(i);
        }
        //println!("find_best_vertex_on_the_left angle:{:?}", max_angle);
        choosen_it
    }

    pub fn peek_item(&self) -> Option<usize> {
        for &item_index in self.items_list.iter() {
            if !self.items[item_index].processed {
                return Some(item_index);
            }
        }
        None
    }

    fn is_edge_closed(&self, p1: usize, p2: usize) -> bool {
        self.generated_face_edges_map.contains_key(&WrapItemKey {p1: p1, p2: p2}) &&
            self.generated_face_edges_map.contains_key(&WrapItemKey {p1: p2, p2: p1})
    }

    fn is_vertex_closed(&self, vertex_index: usize) -> bool {
        let map = self.generated_vertex_edges_map.get(&vertex_index);
        if map.is_none() {
            return false;
        }
        for &other_index in map.unwrap() {
            if !self.is_edge_closed(vertex_index, other_index) {
                return false;
            }
        }
        true
    }

    fn generate(&mut self) {
        while let Some(item_index) = self.peek_item() {
            self.items[item_index].processed = true;
            let p1 = self.items[item_index].p1;
            let p2 = self.items[item_index].p2;
            if self.is_edge_closed(p1, p2) {
                continue;
            }
            let p3 = self.find_best_vertex_on_the_left(item_index);
            if !p3.is_none() {
                let p3 = p3.unwrap();
                self.items[item_index].p3 = p3;
                let base_normal = norm(self.source_vertices[p1].position, 
                    self.source_vertices[p2].position,
                    self.source_vertices[p3].position);
                let new_face_index = self.generated_faces.len();
                //println!("----- new face[{:?}] p1:{:?} p2:{:?} p3:{:?} normal<x:{:?} y:{:?} z:{:?}>", 
                //    new_face_index, p1, p2, p3, base_normal.x, base_normal.y, base_normal.z);
                self.generated_faces.push(Face3 {p1: p1, p2: p2, p3: p3});
                self.add_item(p3, p2, base_normal);
                self.add_item(p1, p3, base_normal);
                self.generated_face_norm_map.entry(RoundVector3::new(base_normal)).or_insert(Vec::new()).push(new_face_index);
                self.generated_face_edges_map.insert(WrapItemKey {p1: p1, p2: p2}, true);
                self.generated_face_edges_map.insert(WrapItemKey {p1: p2, p2: p3}, true);
                self.generated_face_edges_map.insert(WrapItemKey {p1: p3, p2: p1}, true);
                self.generated_vertex_edges_map.entry(p1).or_insert(Vec::new()).push(p2);
                self.generated_vertex_edges_map.entry(p1).or_insert(Vec::new()).push(p3);
                self.generated_vertex_edges_map.entry(p2).or_insert(Vec::new()).push(p3);
                self.generated_vertex_edges_map.entry(p2).or_insert(Vec::new()).push(p1);
                self.generated_vertex_edges_map.entry(p3).or_insert(Vec::new()).push(p1);
                self.generated_vertex_edges_map.entry(p3).or_insert(Vec::new()).push(p2);
            }
        }
    }

    fn add_candidate_face(&mut self, mesh: &mut Mesh, face_id: Id) {
        let halfedge_collection = FaceHalfedgeCollection::new(mesh, mesh.face_first_halfedge_id(face_id).unwrap());
        let mut vertices_index_set : HashMap<Id, usize> = HashMap::new();
        for &halfedge_id in halfedge_collection.as_vec() {
            let vertex = mesh.halfedge_start_vertex(halfedge_id).unwrap();
            vertices_index_set.entry(vertex.id).or_insert(self.add_source_vertex(vertex.position, face_id, vertex.id));
        }
        for &halfedge_id in halfedge_collection.as_vec() {
            let halfedge_next_id = mesh.halfedge_next_id(halfedge_id).unwrap();
            let next_vertex_id = mesh.halfedge_start_vertex_id(halfedge_next_id).unwrap();
            let &next_vertex_index = vertices_index_set.get(&next_vertex_id).unwrap();
            let vertex_id = mesh.halfedge_start_vertex_id(halfedge_id).unwrap();
            let &vertex_index = vertices_index_set.get(&vertex_id).unwrap();
            self.add_startup(next_vertex_index,
                vertex_index,
                mesh.face_norm(face_id));
        }
    }

    fn finalize(&mut self, mesh: &mut Mesh) {
        for f in self.generated_faces.iter() {
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p1].tag));
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p2].tag));
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p3].tag));
            mesh.add_halfedges_and_vertices(added_halfedges);
        }
    }

    pub fn stitch_two_faces(&mut self, mesh: &mut Mesh, face1: Id, face2: Id) {
        let mut remove_faces = Vec::new();
        self.add_candidate_face(mesh, face1);
        if !mesh.face_adj_id(face1).is_none() {
            remove_faces.push(face1);
        }
        self.add_candidate_face(mesh, face2);
        if !mesh.face_adj_id(face2).is_none() {
            remove_faces.push(face2);
        }
        self.generate();
        for face_id in remove_faces {
            mesh.remove_face(face_id);
        }
        self.finalize(mesh);
    }

    pub fn wrap_faces(&mut self, mesh: &mut Mesh, faces: &Vec<Id>) {
        for &face_id in faces {
            self.add_candidate_face(mesh, face_id);
        }
        self.generate();
        self.finalize(mesh);
    }
}
