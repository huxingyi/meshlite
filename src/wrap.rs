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
use util::norm;

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
            if (!self.items.is_empty() && v1.source_plane == v2.source_plane) {
                return;
            }
        }
        if !self.find_item(p1, p2).is_none() || !self.find_item(p2, p1).is_none() {
            return;
        }
        let index = self.items.len();
        self.items.push(WrapItem {p3: 0, p1: p1, p2: p2, base_normal: base_normal, processed: false});
        self.items_map.insert(WrapItemKey {p1: p2, p2: p2}, index);
        self.items_list.push_back(index);
    }

    pub fn find_item(&self, p1: usize, p2: usize) -> Option<&usize> {
        let key = WrapItemKey {p1: p2, p2: p2};
        self.items_map.get(&key)
    }

    pub fn add_startup(&mut self, p1: usize, p2: usize, base_normal: Vector3<f32>) {
        self.add_item(p1, p2, base_normal);
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
        let angle = Deg::from(vd1.angle(vd2));
        angle.0
    }

    fn find_best_vertex_on_the_left(&mut self, item_index: usize) -> Option<usize> {
        let mut max_angle = 0 as f32;
        let mut choosen_it = None;
        let mut remove_i = 0;
        {
            let iter = self.candidates.iter();
            for (i, it) in iter.enumerate() {
                let angle = self.angle_of_base_face_and_point(item_index, *it);
                if angle > max_angle {
                    max_angle = angle;
                    remove_i = i;
                    choosen_it = Some(*it);
                }
            }
        }
        if !choosen_it.is_none() {
            self.candidates.swap_remove(remove_i);
        }
        choosen_it
    }

    pub fn peek_item(&self) -> Option<usize> {
        for &item_index in self.candidates.iter() {
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
            if p3.is_none() {
                self.items[item_index].p3 = p3.unwrap();
                let p1 = self.items[item_index].p1;
                let p2 = self.items[item_index].p2;
                let p3 = p3.unwrap();
                let base_normal = norm(self.source_vertices[p1].position, 
                    self.source_vertices[p2].position,
                    self.source_vertices[p3].position);
                self.generated_faces.push(Face3 {p1: p1, p2: p2, p3: p3});
                self.add_item(p3, p2, base_normal);
                self.add_item(p1, p3, base_normal);
            }
        }
    }

    fn add_candidate_face(&mut self, mesh: &mut Mesh, face_id: Id) {
        for halfedge_id in FaceHalfedgeCollection::new(mesh, mesh.face_first_halfedge_id(face_id).unwrap()).into_iter() {
            let mut vertex = mesh.halfedge_start_vertex_mut(halfedge_id).unwrap();
            vertex.index = self.add_source_vertex(vertex.position, face_id, vertex.id);
        }
    }

    pub fn stitch_two_faces(&mut self, mesh: &mut Mesh, face1: Id, face2: Id) {
        self.add_candidate_face(mesh, face1);
        self.add_candidate_face(mesh, face2);
        let face_adj_id = mesh.face_adj_id(face1).unwrap();
        let normal = mesh.face_norm(face_adj_id);
        let halfedge_id = mesh.face_first_halfedge_id(face1).unwrap();
        let halfedge_next_id = mesh.halfedge_next_id(halfedge_id).unwrap();
        self.add_startup(mesh.halfedge_start_vertex(halfedge_id).unwrap().index,
            mesh.halfedge_start_vertex(halfedge_next_id).unwrap().index,
            normal);
        self.generate();
        mesh.remove_face(face1);
        mesh.remove_face(face2);
        for f in self.generated_faces.iter() {
            let added_face_id = mesh.add_face();
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p1].tag));
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p2].tag));
            added_halfedges.push((mesh.add_halfedge(), self.source_vertices[f.p2].tag));
            for &(added_halfedge_id, added_vertex_id) in added_halfedges.iter() {
                mesh.vertex_mut(added_vertex_id).unwrap().halfedge = added_halfedge_id;
                mesh.halfedge_mut(added_halfedge_id).unwrap().face = added_face_id;
                mesh.halfedge_mut(added_halfedge_id).unwrap().vertex = added_vertex_id;
            }
            mesh.face_mut(added_face_id).unwrap().halfedge = added_halfedges[0].0;
            for i in 0..added_halfedges.len() {
                let first = added_halfedges[i].0;
                let second = added_halfedges[(i + 1) % added_halfedges.len()].0;
                mesh.link_halfedges(first, second);
            }
        }
    }
}
