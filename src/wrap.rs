use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Deg;

use std::collections::HashMap;
use std::collections::LinkedList;
use iterator::FaceHalfedgeIterator;
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
    pub norm: Vector3<f32>,
    pub index: usize,
}

pub struct Face4 {
    pub p1: usize,
    pub p2: usize,
    pub p3: usize,
    pub p4: usize,
}

#[derive(Clone)]
pub struct SourceVertex {
    pub position: Point3<f32>,
    pub source_plane: Id,
    pub index: usize,
    pub tag: Id,
}

pub struct GiftWrapper {
    items: Vec<WrapItem>,
    items_map: HashMap<WrapItemKey, usize>,
    items_list: LinkedList<usize>,
    candidates: Vec<usize>,
    pub source_vertices: Vec<SourceVertex>,
    pub generated_faces: Vec<Face3>,
    generated_face_edges_map: HashMap<WrapItemKey, Option<usize>>,
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
        self.generated_face_edges_map.insert(WrapItemKey {p1: p2, p2: p1}, None);
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
                let face_index = self.generated_faces.len();
                self.generated_faces.push(Face3 {p1: p1, p2: p2, p3: p3, norm: base_normal, index: face_index});
                self.add_item(p3, p2, base_normal);
                self.add_item(p1, p3, base_normal);
                self.generated_face_edges_map.insert(WrapItemKey {p1: p1, p2: p2}, Some(face_index));
                self.generated_face_edges_map.insert(WrapItemKey {p1: p2, p2: p3}, Some(face_index));
                self.generated_face_edges_map.insert(WrapItemKey {p1: p3, p2: p1}, Some(face_index));
                self.generated_vertex_edges_map.entry(p1).or_insert(Vec::new()).push(p2);
                self.generated_vertex_edges_map.entry(p1).or_insert(Vec::new()).push(p3);
                self.generated_vertex_edges_map.entry(p2).or_insert(Vec::new()).push(p3);
                self.generated_vertex_edges_map.entry(p2).or_insert(Vec::new()).push(p1);
                self.generated_vertex_edges_map.entry(p3).or_insert(Vec::new()).push(p1);
                self.generated_vertex_edges_map.entry(p3).or_insert(Vec::new()).push(p2);
            }
        }
    }

    fn add_candidate_face(&mut self, mesh: &mut Mesh, face_id: Id, reverse: bool) {
        let mut vertices_index_set : HashMap<Id, usize> = HashMap::new();
        for halfedge_id in FaceHalfedgeIterator::new(mesh, mesh.face_first_halfedge_id(face_id).unwrap()) {
            let vertex = mesh.halfedge_start_vertex(halfedge_id).unwrap();
            vertices_index_set.entry(vertex.id).or_insert(self.add_source_vertex(vertex.position, face_id, vertex.id));
        }
        for halfedge_id in FaceHalfedgeIterator::new(mesh, mesh.face_first_halfedge_id(face_id).unwrap()) {
            let halfedge_next_id = mesh.halfedge_next_id(halfedge_id).unwrap();
            let next_vertex_id = mesh.halfedge_start_vertex_id(halfedge_next_id).unwrap();
            let &next_vertex_index = vertices_index_set.get(&next_vertex_id).unwrap();
            let vertex_id = mesh.halfedge_start_vertex_id(halfedge_id).unwrap();
            let &vertex_index = vertices_index_set.get(&vertex_id).unwrap();
            if reverse {
                self.add_startup(vertex_index,
                    next_vertex_index,
                    -mesh.face_norm(face_id));
            } else {
                self.add_startup(next_vertex_index,
                    vertex_index,
                    mesh.face_norm(face_id));
            }
        }
    }

    fn another_vertex_index_of_face3(&self, f: &Face3, p1: usize, p2: usize) -> usize {
        let indices = vec![f.p1, f.p2, f.p3];
        for index in indices {
            if index != p1 && index != p2 {
                return index;
            }
        }
        0
    }

    fn find_pair_face3(&self, f: &Face3, used_ids: &HashMap<usize, bool>, q: &mut Vec<Face4>) -> Option<usize> {
        let indices = vec![f.p1, f.p2, f.p3];
        for i in 0..indices.len() {
            let next_i = (i + 1) % indices.len();
            let next_next_i = (i + 2) % indices.len();
            let paired_face3_id = self.generated_face_edges_map.get(&WrapItemKey {p1: indices[next_i], p2: indices[i]});
            if !paired_face3_id.is_none() && !paired_face3_id.unwrap().is_none() {
                let paired_face3_id = paired_face3_id.unwrap().unwrap();
                if used_ids.contains_key(&paired_face3_id) {
                    continue;
                }
                let paired_face3 = &self.generated_faces[paired_face3_id];
                if !almost_eq(paired_face3.norm, f.norm) {
                    continue;
                }
                let another_index = self.another_vertex_index_of_face3(paired_face3, indices[next_i], indices[i]);
                let merged_f = Face4 {p1: indices[i], p2: another_index, p3: indices[next_i], p4: indices[next_next_i]};
                q.push(merged_f);
                return Some(paired_face3_id);
            }
        }
        None
    }

    fn finalize(&mut self, mesh: &mut Mesh) {
        let mut quards : Vec<Face4> = Vec::new();
        let mut used_ids: HashMap<usize, bool> = HashMap::new();
        for f in self.generated_faces.iter() {
            if used_ids.contains_key(&f.index) {
                continue;
            }
            used_ids.insert(f.index, true);
            let paired = self.find_pair_face3(&f, &used_ids, &mut quards);
            if !paired.is_none() {
                used_ids.insert(paired.unwrap(), true);
                continue;
            }
            let mut added_vertices = Vec::new();
            added_vertices.push(self.source_vertices[f.p1].tag);
            added_vertices.push(self.source_vertices[f.p2].tag);
            added_vertices.push(self.source_vertices[f.p3].tag);
            mesh.add_vertices(added_vertices);
        }
        for f in quards.iter() {
            let mut added_vertices = Vec::new();
            added_vertices.push(self.source_vertices[f.p1].tag);
            added_vertices.push(self.source_vertices[f.p2].tag);
            added_vertices.push(self.source_vertices[f.p3].tag);
            added_vertices.push(self.source_vertices[f.p4].tag);
            mesh.add_vertices(added_vertices);
        }
    }

    pub fn stitch_two_faces(&mut self, mesh: &mut Mesh, face1: Id, face2: Id) {
        let mut remove_faces = Vec::new();
        self.add_candidate_face(mesh, face1, false);
        if !mesh.face_adj_id(face1).is_none() {
            remove_faces.push(face1);
        }
        self.add_candidate_face(mesh, face2, false);
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
            self.add_candidate_face(mesh, face_id, true);
        }
        self.generate();
        self.finalize(mesh);
    }
}
