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

pub struct GiftWrapper {
    items: Vec<WrapItem>,
    items_map: HashMap<WrapItemKey, usize>,
    items_list: LinkedList<usize>,
    candidates: Vec<usize>,
    pub source_vertices: Vec<SourceVertex>,
    pub generated_faces: Vec<Face3>,
    generated_face_edges_map: HashMap<WrapItemKey, bool>,
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
        /*
        {
            let v1 = &self.source_vertices[p1];
            let v2 = &self.source_vertices[p2];
            if !self.items.is_empty() && v1.source_plane == v2.source_plane {
                return;
            }
        }*/
        if !self.find_item(p1, p2).is_none() || !self.find_item(p2, p1).is_none() {
            return;
        }
        if self.is_edge_generated(p1, p2) || self.is_edge_generated(p2, p1) {
            return;
        }
        let index = self.items.len();
        self.items.push(WrapItem {p3: 0, p1: p1, p2: p2, base_normal: base_normal, processed: false});
        self.items_map.insert(WrapItemKey {p1: p1, p2: p2}, index);
        self.items_list.push_back(index);
    }

    pub fn find_item(&self, p1: usize, p2: usize) -> Option<&usize> {
        let key = WrapItemKey {p1: p1, p2: p2};
        self.items_map.get(&key)
    }

    pub fn add_startup(&mut self, p1: usize, p2: usize, base_normal: Vector3<f32>) {
        self.add_item(p1, p2, base_normal);
    }

    fn is_edge_generated(&self, p1: usize, p2: usize) -> bool {
        let key = WrapItemKey {p1: p1, p2: p2};
        if self.generated_face_edges_map.get(&key).is_none() {
            return false;
        }
        true
    }

    fn can_make_face(&self, p1: usize, p2: usize, p3: usize) -> bool {
        let b1 = self.is_edge_generated(p1, p2);
        let b2 = self.is_edge_generated(p2, p3);
        let b3 = self.is_edge_generated(p3, p1);
        if b1 || b2 || b3 {
            return false;
        }
        true
    }

    fn angle_of_base_face_and_point(&self, item_index: usize, vertex_index: usize) -> f32 {
        let item = &self.items[item_index].clone();
        let v1 = &self.source_vertices[item.p1].clone();
        let v2 = &self.source_vertices[item.p2].clone();
        let vp = &self.source_vertices[vertex_index].clone();
        if v1.source_plane == v2.source_plane && v1.source_plane == vp.source_plane {
            return 0.0;
        }
        if !self.find_item(item.p2, vertex_index).is_none() || 
                !self.find_item(vertex_index, item.p2).is_none() ||
                !self.find_item(vertex_index, item.p1).is_none() ||
                !self.find_item(item.p1, vertex_index).is_none() {
            return 0.0;
        }
        let vd1 = self.calculate_face_vector(item.p2, item.p1, item.base_normal);
        let normal = norm(v2.position, v1.position, vp.position);
        let vd2 = self.calculate_face_vector(item.p2, item.p1, normal);
        let angle = Deg::from(vd2.angle(vd1));
        angle.0
    }

    fn find_best_vertex_on_the_left(&mut self, item_index: usize) -> Option<usize> {
        let mut max_angle = 0 as f32;
        let mut choosen_it = None;
        for &it in self.candidates.iter() {
            let mut angle = self.angle_of_base_face_and_point(item_index, it);
            if angle > max_angle {
                max_angle = angle;
                choosen_it = Some(it);
            }
        }
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

    fn generate(&mut self) {
        while let Some(item_index) = self.peek_item() {
            self.items[item_index].processed = true;
            let p3 = self.find_best_vertex_on_the_left(item_index);
            if !p3.is_none() {
                let p1 = self.items[item_index].p1;
                let p2 = self.items[item_index].p2;
                let p3 = p3.unwrap();
                if self.can_make_face(p1, p2, p3) {
                    self.items[item_index].p3 = p3;
                    let base_normal = norm(self.source_vertices[p1].position, 
                        self.source_vertices[p2].position,
                        self.source_vertices[p3].position);
                    self.generated_faces.push(Face3 {p1: p1, p2: p2, p3: p3});
                    self.generated_face_edges_map.insert(WrapItemKey {p1: p1, p2: p2}, true);
                    self.generated_face_edges_map.insert(WrapItemKey {p1: p2, p2: p3}, true);
                    self.generated_face_edges_map.insert(WrapItemKey {p1: p3, p2: p1}, true);
                    self.add_item(p3, p2, base_normal);
                    self.add_item(p1, p3, base_normal);
                }
            }
        }
    }

    fn add_candidate_face(&mut self, mesh: &mut Mesh, face_id: Id) {
        let halfedge_collection = FaceHalfedgeCollection::new(mesh, mesh.face_first_halfedge_id(face_id).unwrap());
        for &halfedge_id in halfedge_collection.as_vec() {
            let mut vertex = mesh.halfedge_start_vertex_mut(halfedge_id).unwrap();
            vertex.index = self.add_source_vertex(vertex.position, face_id, vertex.id);
        }
        for &halfedge_id in halfedge_collection.as_vec() {
            let opposite_id = mesh.halfedge_opposite_id(halfedge_id).unwrap();
            let opposite_face_id = mesh.halfedge(opposite_id).unwrap().face;
            let normal = mesh.face_norm(opposite_face_id);
            let halfedge_next_id = mesh.halfedge_next_id(halfedge_id).unwrap();
            self.add_startup(mesh.halfedge_start_vertex(halfedge_id).unwrap().index,
                mesh.halfedge_start_vertex(halfedge_next_id).unwrap().index,
                normal);
        }
    }

    pub fn stitch_two_faces(&mut self, mesh: &mut Mesh, face1: Id, face2: Id) {
        self.add_candidate_face(mesh, face1);
        self.add_candidate_face(mesh, face2);
        self.generate();
        mesh.remove_face(face1);
        mesh.remove_face(face2);
        for f in self.generated_faces.iter() {
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p1].tag));
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p2].tag));
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p3].tag));
            mesh.add_halfedges_and_vertices(added_halfedges);
        }
    }
}
