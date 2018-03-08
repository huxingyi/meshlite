use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Matrix4;
use std::option::Option;
use std::fs::File;
use std::io::prelude::*;
use std::string::String;
use std::str::FromStr;
use std::io;
use std::vec::Vec;
use std::collections::HashMap;
use std::collections::HashSet;
use iterator::FaceHalfedgeIterator;
use iterator::FaceIterator;
use util::*;
use std::ops::Add;
use std::ops::AddAssign;

pub type Id = usize;

#[derive(Debug)]
pub struct Vertex {
    pub id: Id,
    pub position: Point3<f32>,
    pub halfedge: Id,
    pub prev: Id,
    pub next: Id,
    pub alive: bool,
}

#[derive(Debug)]
pub struct Face {
    pub id: Id,
    pub halfedge: Id,
    pub prev: Id,
    pub next: Id,
    pub alive: bool,
}

#[derive(Debug)]
pub struct Halfedge {
    pub id: Id,
    pub vertex: Id,
    pub face: Id,
    pub prev: Id,
    pub next: Id,
    pub opposite: Id,
    pub alive: bool,
}

#[derive(Hash, Eq, PartialEq, Debug, Clone)]
pub struct Point3Key {
    x: u32,
    y: u32,
    z: u32,
}

impl Point3Key {
    pub fn new(point: Point3<f32>) -> Self {
        Point3Key {
            x: (point.x * 1000.0).round() as u32,
            y: (point.y * 1000.0).round() as u32,
            z: (point.z * 1000.0).round() as u32,
        }
    }
}

#[derive(Hash, Eq, PartialEq, Debug, Clone)]
pub struct EdgeEndpoints {
    low: Id,
    high: Id,
}

impl EdgeEndpoints {
    pub fn new(first: Id, second: Id) -> Self {
        if first < second {
            EdgeEndpoints {
                low: first,
                high: second
            }
        } else {
            EdgeEndpoints {
                low: second,
                high: first
            }
        }
    }
}

pub type FacePair = EdgeEndpoints;

#[derive(Debug)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub vertex_count: usize,
    pub faces: Vec<Face>,
    pub face_count: usize,
    pub halfedges: Vec<Halfedge>,
    pub halfedge_count: usize,
    pub edges: HashMap<EdgeEndpoints, Id>,
    pub edge_count: usize,
}

impl Mesh {
    pub fn new() -> Self {
        Mesh {
            vertices: Vec::new(),
            vertex_count: 0,
            faces: Vec::new(),
            face_count: 0,
            halfedges: Vec::new(),
            halfedge_count: 0,
            edges: HashMap::new(),
            edge_count: 0,
        }
    }

    pub fn vertex(&self, id: Id) -> Option<&Vertex> {
        if 0 == id {
            return None;
        }
        {
            let vertex = &self.vertices[id - 1];
            if !vertex.alive {
                return None;
            }
        }
        Some(&self.vertices[id - 1])
    }

    pub fn vertex_mut(&mut self, id: Id) -> Option<&mut Vertex> {
        if 0 == id {
            return None;
        }
        {
            let vertex = &self.vertices[id - 1];
            if !vertex.alive {
                return None;
            }
        }
        Some(&mut self.vertices[id - 1])
    }

    pub fn peek_same_halfedge(&self, any_paired_id: Id) -> Id {
        let halfedge = self.halfedge(any_paired_id).unwrap();
        if halfedge.opposite > 0 && halfedge.opposite < any_paired_id {
            return halfedge.opposite;
        }
        any_paired_id
    }

    pub fn edge_center(&self, id: Id) -> Point3<f32> {
        let halfedge = self.halfedge(id).unwrap();
        let next = self.halfedge(halfedge.next).unwrap();
        Point3::midpoint(self.vertex(halfedge.vertex).unwrap().position,
            self.vertex(next.vertex).unwrap().position)
    }

    pub fn face_center(&self, id: Id) -> Point3<f32> {
        let face = self.face(id).unwrap();
        let mut face_halfedge_iter = FaceHalfedgeIterator::new(self, face.halfedge);
        let mut points = Vec::new();
        while let Some(halfedge_id) = face_halfedge_iter.next() {
            let halfedge = self.halfedge(halfedge_id).unwrap();
            let vertex = self.vertex(halfedge.vertex).unwrap();
            points.push(vertex.position);
        }
        Point3::centroid(&points)
    }

    pub fn face_norm(&self, id: Id) -> Vector3<f32> {
        let face = self.face(id).unwrap();
        let mut points = Vec::new();
        for halfedge_id in FaceHalfedgeIterator::new(self, face.halfedge) {
            let halfedge = self.halfedge(halfedge_id).unwrap();
            let vertex = self.vertex(halfedge.vertex).unwrap();
            points.push(vertex.position);
        }
        if points.len() < 3 {
            return Vector3::zero();
        } else if points.len() == 3 {
            return norm(points[0], points[1], points[2]);
        }
        let mut total = Vector3::zero();
        for i in 0..points.len() {
            let n = norm(points[i], points[(i + 1) % points.len()], points[(i + 2) % points.len()]);
            total += n;
        }
        total / points.len() as f32
    }

    pub fn face(&self, id: Id) -> Option<&Face> {
        if 0 == id {
            return None;
        }
        {
            let face = &self.faces[id - 1];
            if !face.alive {
                return None;
            }
        }
        Some(&self.faces[id - 1])
    }

    pub fn face_adj_id(&self, id: Id) -> Option<Id> {
        self.face(id)
            .and_then(|f: &Face| self.halfedge(f.halfedge))
            .and_then(|h: &Halfedge| self.halfedge(h.opposite))
            .and_then(|o: &Halfedge| if 0 != o.face { Some(o.face) } else { None })
    }

    pub fn face_adj(&self, id: Id) -> Option<&Face> {
        self.face_adj_id(id).and_then(|id: Id| self.face(id))
    }

    pub fn halfedge_next_id(&self, id: Id) -> Option<Id> {
        self.halfedge(id)
            .and_then(|h: &Halfedge| if 0 != h.next { Some(h.next) } else { None })
    }

    pub fn halfedge_prev_id(&self, id: Id) -> Option<Id> {
        self.halfedge(id)
            .and_then(|h: &Halfedge| if 0 != h.prev { Some(h.prev) } else { None })
    }

    pub fn halfedge_opposite_id(&self, id: Id) -> Option<Id> {
        self.halfedge(id)
            .and_then(|h: &Halfedge| if 0 != h.opposite { Some(h.opposite) } else { None })
    }

    pub fn face_first_halfedge_id(&self, id: Id) -> Option<Id> {
        self.face(id)
            .and_then(|f: &Face| if 0 != f.halfedge { Some(f.halfedge) } else { None })
    }

    pub fn halfedge_start_vertex_id(&self, id: Id) -> Option<Id> {
        self.halfedge(id)
            .and_then(|h: &Halfedge| if 0 != h.vertex { Some(h.vertex) } else { None })
    }

    pub fn halfedge_face_id(&self, id: Id) -> Option<Id> {
        self.halfedge(id)
            .and_then(|h: &Halfedge| if 0 != h.face { Some(h.face) } else { None })
    }

    pub fn halfedge_opposite_face_id(&self, id: Id) -> Option<Id> {
        let opposite_id = self.halfedge_opposite_id(id);
        if opposite_id.is_none() {
            return None;
        }
        self.halfedge_face_id(opposite_id.unwrap())
    }

    pub fn halfedge_direct(&self, id: Id) -> Vector3<f32> {
        let begin_pos = self.halfedge_start_vertex(id).unwrap().position;
        let end_pos = self.halfedge_start_vertex(self.halfedge_next_id(id).unwrap()).unwrap().position;
        end_pos - begin_pos
    }

    pub fn set_halfedge_start_vertex_id(&mut self, halfedge_id: Id, vertex_id: Id) {
        self.halfedge_mut(halfedge_id).unwrap().vertex = vertex_id;
    }

    pub fn halfedge_start_vertex_mut(&mut self, id: Id) -> Option<&mut Vertex> {
        let vertex_id = self.halfedge_start_vertex_id(id)?;
        self.vertex_mut(vertex_id)
    }

    pub fn halfedge_start_vertex(&self, id: Id) -> Option<&Vertex> {
        let vertex_id = self.halfedge_start_vertex_id(id)?;
        self.vertex(vertex_id)
    }

    pub fn vertex_first_halfedge_id(&self, id: Id) -> Option<Id> {
        self.vertex(id)
            .and_then(|v: &Vertex| if 0 != v.halfedge { Some(v.halfedge) } else { None })
    }

    pub fn vertex_first_halfedge(&self, id: Id) -> Option<&Halfedge> {
        let halfedge_id = self.vertex_first_halfedge_id(id)?;
        self.halfedge(halfedge_id)
    }

    pub fn vertex_first_halfedge_mut(&mut self, id: Id) -> Option<&mut Halfedge> {
        let halfedge_id = self.vertex_first_halfedge_id(id)?;
        self.halfedge_mut(halfedge_id)
    }

    pub fn set_vertex_first_halfedge_id(&mut self, vertex_id: Id, halfedge_id: Id) {
        self.vertex_mut(vertex_id).unwrap().halfedge = halfedge_id;
    }

    pub fn set_halfedge_opposite_id(&mut self, halfedge_id: Id, opposite_id: Id) {
        self.halfedge_mut(halfedge_id).unwrap().opposite = opposite_id;
    }

    fn remove_halfedges_from_edges(&mut self, halfedges: &Vec<Id>) {
        for &halfedge_id in halfedges {
            let opposite = self.halfedge_opposite_id(halfedge_id);
            let next_halfedge_id = self.halfedge_next_id(halfedge_id).unwrap();
            let endpoints = EdgeEndpoints::new(self.halfedge_start_vertex_id(halfedge_id).unwrap(), 
                self.halfedge_start_vertex_id(next_halfedge_id).unwrap());
            if let Some(&id) = self.edges.get(&endpoints) {
                if id == halfedge_id {
                    if !opposite.is_none() {
                        *self.edges.get_mut(&endpoints).unwrap() = opposite.unwrap();
                    } else {
                        self.edges.remove(&endpoints);
                    }
                }
            }
        }
    }

    pub fn halfedge_start_vertex_alt_halfedge_id(&self, halfedge_id: Id) -> Option<Id> {
        let opposite = self.halfedge_opposite_id(halfedge_id);
        if !opposite.is_none() {
            return self.halfedge_next_id(opposite.unwrap());
        }
        let prev = self.halfedge_prev_id(halfedge_id);
        if !prev.is_none() {
            return self.halfedge_opposite_id(prev.unwrap());
        }
        None
    }

    pub fn remove_face(&mut self, id: Id) {
        let halfedge_collection = FaceHalfedgeIterator::new(self, self.face_first_halfedge_id(id).unwrap()).into_vec();
        self.remove_halfedges_from_edges(&halfedge_collection);
        for halfedge_id in halfedge_collection {
            let vertex_id = self.halfedge_start_vertex_id(halfedge_id).unwrap();
            let opposite = self.halfedge_opposite_id(halfedge_id);
            if !opposite.is_none() {
                self.set_halfedge_opposite_id(opposite.unwrap(), 0);
            }
            if self.vertex_first_halfedge_id(vertex_id).unwrap() == halfedge_id {
                let alt_id = self.halfedge_start_vertex_alt_halfedge_id(halfedge_id);
                if alt_id.is_none() {
                    self.set_vertex_first_halfedge_id(vertex_id, 0);
                    self.vertex_mut(vertex_id).unwrap().alive = false;
                } else {
                    self.set_vertex_first_halfedge_id(vertex_id, alt_id.unwrap());
                }
            }
            self.halfedge_mut(halfedge_id).unwrap().alive = false;
        }
        self.face_mut(id).unwrap().alive = false;
    }

    pub fn face_mut(&mut self, id: Id) -> Option<&mut Face> {
        if 0 == id {
            return None;
        }
        {
            let face = &self.faces[id - 1];
            if !face.alive {
                return None;
            }
        }
        Some(&mut self.faces[id - 1])
    }

    pub fn halfedge(&self, id: Id) -> Option<&Halfedge> {
        if 0 == id {
            return None;
        }
        {
            let halfedge = &self.halfedges[id - 1];
            if !halfedge.alive {
                return None;
            }
        }
        Some(&self.halfedges[id - 1])
    }

    pub fn halfedge_mut(&mut self, id: Id) -> Option<&mut Halfedge> {
        if 0 == id {
            return None;
        }
        {
            let halfedge = &self.halfedges[id - 1];
            if !halfedge.alive {
                return None;
            }
        }
        Some(&mut self.halfedges[id - 1])
    }

    pub fn add_vertex(&mut self, position: Point3<f32>) -> usize {
        let new_id = self.vertices.len() + 1;
        self.vertices.push(Vertex {
            id: new_id,
            halfedge: 0,
            prev: 0,
            next: 0,
            position : position,
            alive: true,
        });
        new_id
    }

    pub fn add_halfedge(&mut self) -> Id {
        let new_id = self.halfedges.len() + 1;
        self.halfedges.push(Halfedge {
            id: new_id,
            vertex: 0,
            face: 0,
            prev: 0,
            next: 0,
            opposite: 0,
            alive: true,
        });
        new_id
    }

    pub fn pair_halfedges(&mut self, first: Id, second: Id) {
        self.halfedge_mut(first).unwrap().opposite = second;
        self.halfedge_mut(second).unwrap().opposite = first;
    }

    pub fn unpair_halfedges(&mut self, first: Id, second: Id) {
        self.halfedge_mut(first).unwrap().opposite = 0;
        self.halfedge_mut(second).unwrap().opposite = 0;
    }

    pub fn link_halfedges(&mut self, first: Id, second: Id) {
        self.halfedge_mut(first).unwrap().next = second;
        self.halfedge_mut(second).unwrap().prev = first;
        let endpoints = EdgeEndpoints::new(self.halfedge(first).unwrap().vertex,
            self.halfedge(second).unwrap().vertex);
        match self.edges.get(&endpoints) {
            Some(&halfedge) => self.pair_halfedges(first, halfedge),
            _ => {
                self.edges.insert(endpoints, first);
                self.edge_count += 1; 
            }
        };
    }

    pub fn add_face(&mut self) -> Id {
        let new_id = self.faces.len() + 1;
        self.faces.push(Face {
            id: new_id,
            halfedge: 0,
            prev: 0,
            next: 0,
            alive: true,
        });
        new_id
    }

    pub fn add_linked_vertices(&mut self, linked_vertices: HashMap<Id, Id>) -> Id {
        let (&first_id, _) = linked_vertices.iter().next().unwrap();
        let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
        let mut vert = first_id;
        added_halfedges.push((self.add_halfedge(), first_id));
        while linked_vertices.contains_key(&vert) && linked_vertices[&vert] != first_id {
            vert = linked_vertices[&vert];
            added_halfedges.push((self.add_halfedge(), vert));
        }
        self.add_halfedges_and_vertices(added_halfedges)
    }

    pub fn add_vertices(&mut self, added_vertices : Vec<Id>) -> Id {
        let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
        for i in 0..added_vertices.len() {
            added_halfedges.push((self.add_halfedge(), added_vertices[i]));
        }
        self.add_halfedges_and_vertices(added_halfedges)
    }

    pub fn add_halfedges_and_vertices(&mut self, added_halfedges : Vec<(Id, Id)>) -> Id {
        let added_face_id = self.add_face();
        for &(added_halfedge_id, added_vertex_id) in added_halfedges.iter() {
            self.vertex_mut(added_vertex_id).unwrap().halfedge = added_halfedge_id;
            self.halfedge_mut(added_halfedge_id).unwrap().face = added_face_id;
            self.halfedge_mut(added_halfedge_id).unwrap().vertex = added_vertex_id;
        }
        self.face_mut(added_face_id).unwrap().halfedge = added_halfedges[0].0;
        for i in 0..added_halfedges.len() {
            let first = added_halfedges[i].0;
            let second = added_halfedges[(i + 1) % added_halfedges.len()].0;
            self.link_halfedges(first, second);
        }
        added_face_id
    }

    pub fn extrude_halfedges(&mut self, halfedges: &Vec<Id>, normal: Vector3<f32>, amount: f32) {
        let mut downside_halfedges: Vec<Id> = Vec::new();
        let mut downside_vertices: Vec<Id> = Vec::new();
        let direct = normal * amount;
        let mut need_fill_downside = false;
        for &halfedge_id in halfedges {
            let opposite = self.halfedge_opposite_id(halfedge_id);
            if opposite.is_none() {
                let old_position = {
                    let mut vertex = self.halfedge_start_vertex_mut(halfedge_id).unwrap();
                    let position = vertex.position;
                    vertex.position += direct;
                    position
                };
                downside_vertices.push(self.add_vertex(old_position));
                downside_halfedges.push(self.add_halfedge());
                need_fill_downside = true;
            } else {
                let old_vertex_id = self.halfedge_start_vertex_id(halfedge_id).unwrap();
                let copy_position = self.halfedge_start_vertex(halfedge_id).unwrap().position;
                let copy_vertex = self.add_vertex(copy_position);
                if self.vertex_first_halfedge_id(old_vertex_id).unwrap() == halfedge_id {
                    let opposite_next_halfedge_id = self.halfedge_next_id(opposite.unwrap()).unwrap();
                    self.set_vertex_first_halfedge_id(old_vertex_id, opposite_next_halfedge_id);
                }
                self.set_halfedge_start_vertex_id(halfedge_id, copy_vertex);
                self.unpair_halfedges(halfedge_id, opposite.unwrap());
                downside_vertices.push(old_vertex_id);
                downside_halfedges.push(opposite.unwrap());
            }
        }
        for i in 0..halfedges.len() {
            let halfedge_id = halfedges[i];
            let i_next = (i + 1) % halfedges.len();
            let next_halfedge_id = halfedges[i_next];
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            let left_bottom = downside_vertices[i];
            let right_bottom = downside_vertices[i_next];
            let right_top = self.halfedge_start_vertex_id(next_halfedge_id).unwrap();
            let left_top = self.halfedge_start_vertex_id(halfedge_id).unwrap();
            added_halfedges.push((self.add_halfedge(), left_bottom));
            added_halfedges.push((self.add_halfedge(), right_bottom));
            added_halfedges.push((self.add_halfedge(), right_top));
            added_halfedges.push((self.add_halfedge(), left_top));
            self.add_halfedges_and_vertices(added_halfedges);
        }
        if need_fill_downside {
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            for i in 0..halfedges.len() {
                let j = (halfedges.len() - i) % halfedges.len();
                added_halfedges.push((downside_halfedges[j], downside_vertices[j]));
            }
            self.add_halfedges_and_vertices(added_halfedges);
        }
    }

    pub fn extrude_face(&mut self, face_id: Id, normal: Vector3<f32>, amount: f32) -> &mut Self {
        let mut new_halfedges : Vec<Id> = Vec::new();
        for halfedge_id in FaceHalfedgeIterator::new(self, face_id) {
            new_halfedges.push(halfedge_id);
        }
        self.extrude_halfedges(&new_halfedges, normal, amount);
        self
    }

    pub fn add_plane(&mut self, width: f32, depth: f32) -> Id {
        let x = width / 2.0;
        let y = depth / 2.0;
        let points = vec![Point3 {x: -x, y: -y, z: 0.0},
            Point3 {x: x, y: -y, z: 0.0},
            Point3 {x: x, y: y, z: 0.0},
            Point3 {x: -x, y: y, z: 0.0}];
        let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
        for i in 0..points.len() {
            added_halfedges.push((self.add_halfedge(), self.add_vertex(points[i])));
        }
        self.add_halfedges_and_vertices(added_halfedges)
    }

    pub fn transform(&mut self, mat: &Matrix4<f32>) -> &mut Self {
        for vertex in self.vertices.iter_mut() {
            vertex.position = mat.transform_point(vertex.position);
        }
        self
    }

    pub fn translate(&mut self, x: f32, y: f32, z: f32) -> &mut Self {
        let mat = Matrix4::from_translation(Vector3::new(x, y, z));
        self.transform(&mat)
    }

    pub fn scale(&mut self, value: f32) -> &mut Self {
        let mat = Matrix4::from_scale(value);
        self.transform(&mat)
    }

    pub fn weld(&self) -> Self {
        let mut mesh = Mesh::new();
        mesh.add_mesh(&self);
        mesh
    }

    pub fn add_mesh(&mut self, other: &Mesh) {
        let mut vertices_set : HashMap<Point3Key, Id> = HashMap::new();
        for face_id in FaceIterator::new(&other) {
            let face = other.face(face_id).unwrap();
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            for halfedge_id in FaceHalfedgeIterator::new(&other, face.halfedge) {
                let vertex = other.halfedge_start_vertex(halfedge_id).unwrap();
                let key = Point3Key::new(vertex.position);
                if let Some(&new_vertex_id) = vertices_set.get(&key) {
                    added_halfedges.push((self.add_halfedge(), new_vertex_id));
                } else {
                    let new_vertex_id = self.add_vertex(vertex.position);
                    vertices_set.insert(key, new_vertex_id);
                    added_halfedges.push((self.add_halfedge(), new_vertex_id));
                }
            }
            self.add_halfedges_and_vertices(added_halfedges);
        }
    }

    pub fn flip_mesh(&self) -> Mesh {
        let mut new_mesh = Mesh::new();
        let mut new_vert_map = HashMap::new();
        for face_id in FaceIterator::new(self) {
            let mut verts = Vec::new();
            for halfedge_id in FaceHalfedgeIterator::new(self, self.face_first_halfedge_id(face_id).unwrap()) {
                let old_vert = self.halfedge_start_vertex(halfedge_id).unwrap();
                let new_vert_id = new_vert_map.entry(old_vert.id).or_insert_with(|| {
                    new_mesh.add_vertex(old_vert.position)
                });
                verts.push(*new_vert_id);
            }
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            for new_vert_id in verts.iter().rev() {
                added_halfedges.push((new_mesh.add_halfedge(), *new_vert_id));
            }
            new_mesh.add_halfedges_and_vertices(added_halfedges);
        }
        new_mesh
    }

    pub fn split_mesh_by_other(&self, other: &Mesh) -> (Mesh, Mesh) {
        let mut inner_mesh = Mesh::new();
        let mut outter_mesh = Mesh::new();
        inner_mesh.add_mesh(self);
        for face_id in FaceIterator::new(other) {
            let norm = other.face_norm(face_id);
            let point = other.halfedge_start_vertex(other.face_first_halfedge_id(face_id).unwrap()).unwrap().position;
            let (sub_front, sub_back) = inner_mesh.split_mesh_by_plane(point, norm, false);
            inner_mesh = sub_back;
            outter_mesh.add_mesh(&sub_front);
        }
        (outter_mesh, inner_mesh)
    }

    pub fn union_mesh(&self, other: &Mesh) -> Mesh {
        let (other_outter, _) = other.split_mesh_by_other(self);
        let (my_outter, _) = self.split_mesh_by_other(other);
        let mesh = other_outter + my_outter;
        mesh.weld().fix_tjunction().combine_adj_faces()
    }

    pub fn diff_mesh(&self, other: &Mesh) -> Mesh {
        let (_, other_inner) =  other.split_mesh_by_other(self);
        let (my_outter, _) = self.split_mesh_by_other(other);
        let mesh = other_inner.flip_mesh() + my_outter;
        mesh.weld().fix_tjunction().combine_adj_faces()
    }

    pub fn intersect_mesh(&self, other: &Mesh) -> Mesh {
        let (_, other_inner) =  other.split_mesh_by_other(self);
        let (_, my_inner) = self.split_mesh_by_other(other);
        let mesh = other_inner + my_inner;
        mesh.weld().fix_tjunction().combine_adj_faces()
    }

    pub fn split_mesh_by_plane(&self, pt_on_plane: Point3<f32>, norm: Vector3<f32>, fill_cut: bool) -> (Mesh, Mesh) { 
        let mut vert_side_map : HashMap<Id, PointSide> = HashMap::new();
        for face_id in FaceIterator::new(self) {
            for halfedge_id in FaceHalfedgeIterator::new(self, self.face_first_halfedge_id(face_id).unwrap()) {
                let vert = self.halfedge_start_vertex(halfedge_id).unwrap();
                vert_side_map.entry(vert.id).or_insert(point_side_on_plane(vert.position, pt_on_plane, norm));
            }
        }
        let mut front_mesh = Mesh::new();
        let mut back_mesh = Mesh::new();
        let mut front_vert_map : HashMap<Id, Id> = HashMap::new();
        let mut back_vert_map : HashMap<Id, Id> = HashMap::new();
        let mut intersect_map : HashMap<EdgeEndpoints, SegmentPlaneIntersect> = HashMap::new();
        let mut front_intersect_map : HashMap<EdgeEndpoints, Id> = HashMap::new();
        let mut back_intersect_map : HashMap<EdgeEndpoints, Id> = HashMap::new();
        let mut front_cut_map : HashMap<Id, Id> = HashMap::new();
        let mut back_cut_map : HashMap<Id, Id> = HashMap::new();
        for face_id in FaceIterator::new(self) {
            let mut front_new_verts = Vec::new();
            let mut back_new_verts = Vec::new();
            let mut front_new_vert_set = HashSet::new();
            let mut back_new_vert_set = HashSet::new();
            let mut front_intersects = vec![0, 0];
            let mut back_intersects = vec![0, 0];
            for halfedge_id in FaceHalfedgeIterator::new(self, self.face_first_halfedge_id(face_id).unwrap()) {
                let from_vert_id = self.halfedge_start_vertex_id(halfedge_id).unwrap();
                let to_vert_id = self.halfedge_start_vertex_id(self.halfedge_next_id(halfedge_id).unwrap()).unwrap();
                let from_is_front = vert_side_map[&from_vert_id] != PointSide::Back;
                let to_is_front = vert_side_map[&to_vert_id] != PointSide::Back;
                let from_is_back = vert_side_map[&from_vert_id] != PointSide::Front;
                let to_is_back = vert_side_map[&to_vert_id] != PointSide::Front;
                let edge = EdgeEndpoints::new(from_vert_id, to_vert_id);
                if from_is_front {
                    let new_vert_id = *front_vert_map.entry(from_vert_id).or_insert_with(|| {
                        front_mesh.add_vertex(self.vertex(from_vert_id).unwrap().position)
                    });
                    if front_new_vert_set.insert(new_vert_id) {
                        front_new_verts.push(new_vert_id);
                    }
                }
                if from_is_back {
                    let new_vert_id = *back_vert_map.entry(from_vert_id).or_insert_with(|| {
                        back_mesh.add_vertex(self.vertex(from_vert_id).unwrap().position)
                    });
                    if back_new_vert_set.insert(new_vert_id) {
                        back_new_verts.push(new_vert_id);
                    }
                }
                if (from_is_front && to_is_back) || (from_is_back && to_is_front) {
                    let intersect = intersect_map.entry(edge.clone()).or_insert_with(|| {
                        let p0 = self.vertex(from_vert_id).unwrap().position;
                        let p1 = self.vertex(to_vert_id).unwrap().position;
                        intersect_of_segment_and_plane(p0, p1, pt_on_plane, norm)
                    });
                    if let SegmentPlaneIntersect::Intersection(intersect_pt) = *intersect {
                        if from_is_front || to_is_front {
                            let new_vert_id = *front_intersect_map.entry(edge.clone()).or_insert_with(|| {
                                front_mesh.add_vertex(intersect_pt)
                            });
                            if front_new_vert_set.insert(new_vert_id) {
                                front_new_verts.push(new_vert_id);
                            }
                            if from_is_front {
                                front_intersects[0] = new_vert_id;
                            }
                            if to_is_front {
                                front_intersects[1] = new_vert_id;
                            }
                        }
                        if from_is_back || to_is_back {
                            let new_vert_id = *back_intersect_map.entry(edge.clone()).or_insert_with(|| {
                                back_mesh.add_vertex(intersect_pt)
                            });
                            if back_new_vert_set.insert(new_vert_id) {
                                back_new_verts.push(new_vert_id);
                            }
                            if from_is_front {
                                back_intersects[0] = new_vert_id;
                            }
                            if to_is_front {
                                back_intersects[1] = new_vert_id;
                            }
                        }
                    }
                }
                if to_is_front {
                    let new_vert_id = *front_vert_map.entry(to_vert_id).or_insert_with(|| {
                        front_mesh.add_vertex(self.vertex(to_vert_id).unwrap().position)
                    });
                    if front_new_vert_set.insert(new_vert_id) {
                        front_new_verts.push(new_vert_id);
                    }
                }
                if to_is_back {
                    let new_vert_id = *back_vert_map.entry(to_vert_id).or_insert_with(|| {
                        back_mesh.add_vertex(self.vertex(to_vert_id).unwrap().position)
                    });
                    if back_new_vert_set.insert(new_vert_id) {
                        back_new_verts.push(new_vert_id);
                    }
                }
            }
            if front_new_verts.len() >= 3 {
                front_mesh.add_vertices(front_new_verts);
            }
            if back_new_verts.len() >= 3 {
                back_mesh.add_vertices(back_new_verts);
            }
            if front_intersects[0] > 0 && front_intersects[1] > 0 {
                front_cut_map.insert(front_intersects[1], front_intersects[0]);
            }
            if back_intersects[0] > 0 && back_intersects[1] > 0 {
                back_cut_map.insert(back_intersects[0], back_intersects[1]);
            }
        }
        if fill_cut {
            if front_cut_map.len() >= 3 {
                front_mesh.add_linked_vertices(front_cut_map);
            }
            if back_cut_map.len() >= 3 {
                back_mesh.add_linked_vertices(back_cut_map);
            }
        }
        (front_mesh, back_mesh)
    }

    fn split_halfedge(&mut self, halfedge_id: Id, vertex_id: Id) -> Id {
        let (face_id, next_halfedge_id) = {
            let halfedge = self.halfedge(halfedge_id).unwrap();
            (halfedge.face, halfedge.next)
        };
        let new_halfedge_id = self.add_halfedge();
        {
            let new_halfedge = self.halfedge_mut(new_halfedge_id).unwrap();
            new_halfedge.vertex = vertex_id;
            new_halfedge.face = face_id;
        }
        self.link_halfedges(new_halfedge_id, next_halfedge_id);
        self.link_halfedges(halfedge_id, new_halfedge_id);
        new_halfedge_id
    }

    pub fn fix_tjunction(&mut self) -> &mut Self {
        let mut may_broken_halfedges = Vec::new();
        for face_id in FaceIterator::new(self) {
            for halfedge_id in FaceHalfedgeIterator::new(self, self.face_first_halfedge_id(face_id).unwrap()) {
                if self.halfedge_opposite_id(halfedge_id).is_none() {
                    may_broken_halfedges.push(halfedge_id);
                }
            }
        }
        let mut i = 0;
        'outer: while i < may_broken_halfedges.len() {
            'inner: for j in 0..may_broken_halfedges.len() {
                let long_id = may_broken_halfedges[i];
                let short_id = may_broken_halfedges[j];
                if long_id == short_id {
                    continue;
                }
                if !self.halfedge_opposite_id(long_id).is_none() {
                    continue;
                }
                let long_begin = self.halfedge_start_vertex(long_id).unwrap().position;
                let long_end = self.halfedge_start_vertex(self.halfedge_next_id(long_id).unwrap()).unwrap().position;
                
                let (short_begin_pos, short_begin_vert_id) = {
                    let vert = self.halfedge_start_vertex(short_id).unwrap();
                    (vert.position, vert.id)
                };
                if is_point_on_segment(short_begin_pos, long_begin, long_end) {
                    may_broken_halfedges.push(self.split_halfedge(long_id, short_begin_vert_id));
                    continue 'outer;
                }

                let (short_end_pos, short_end_vert_id) = {
                    let vert = self.halfedge_start_vertex(self.halfedge_next_id(short_id).unwrap()).unwrap();
                    (vert.position, vert.id)
                };
                if is_point_on_segment(short_end_pos, long_begin, long_end) {
                    may_broken_halfedges.push(self.split_halfedge(long_id, short_end_vert_id));
                    continue 'outer;
                }
            }
            i += 1;
        }
        self
    }

    pub fn combine_adj_faces_round(&self) -> (bool, Self) {
        let from_mesh = self;
        let mut to_mesh = Mesh::new();
        let mut ignore_faces = HashSet::new();
        let mut new_vert_map : HashMap<Id, Id> = HashMap::new();
        let mut ignore_vert_ids : HashSet<Id> = HashSet::new();
        let mut pending_old_verts : Vec<Vec<Id>> = Vec::new();
        let mut face_pair_map : HashSet<FacePair> = HashSet::new();
        for (_, &halfedge_id) in self.edges.iter() {
            let face_id = from_mesh.halfedge_face_id(halfedge_id).unwrap();
            if ignore_faces.contains(&face_id) {
                continue;
            }
            if let Some(opposite_id) = from_mesh.halfedge_opposite_id(halfedge_id) {
                let opposite_face_id = from_mesh.halfedge_face_id(opposite_id).unwrap();
                if ignore_faces.contains(&opposite_face_id) {
                    continue;
                }
                let prev_id = from_mesh.halfedge_prev_id(halfedge_id).unwrap();
                let next_id = from_mesh.halfedge_next_id(halfedge_id).unwrap();
                let opposite_prev_id = from_mesh.halfedge_prev_id(opposite_id).unwrap();
                let opposite_next_id = from_mesh.halfedge_next_id(opposite_id).unwrap();
                if !almost_eq(from_mesh.halfedge_direct(prev_id).normalize(), from_mesh.halfedge_direct(opposite_next_id).normalize()) {
                    continue;
                }
                if !almost_eq(from_mesh.halfedge_direct(opposite_prev_id).normalize(), from_mesh.halfedge_direct(next_id).normalize()) {
                    continue;
                }
                let first_face_id = from_mesh.halfedge_opposite_face_id(prev_id);
                let second_face_id = from_mesh.halfedge_opposite_face_id(opposite_next_id);
                if (first_face_id == second_face_id) || 
                        (!first_face_id.is_none() && !second_face_id.is_none() && 
                            face_pair_map.contains(&FacePair::new(first_face_id.unwrap(), second_face_id.unwrap()))) {
                    ignore_vert_ids.insert(from_mesh.halfedge_start_vertex_id(halfedge_id).unwrap());
                }
                let first_face_id = from_mesh.halfedge_opposite_face_id(opposite_prev_id);
                let second_face_id = from_mesh.halfedge_opposite_face_id(next_id);
                if (first_face_id == second_face_id) || 
                        (!first_face_id.is_none() && !second_face_id.is_none() && 
                            face_pair_map.contains(&FacePair::new(first_face_id.unwrap(), second_face_id.unwrap()))) {
                    ignore_vert_ids.insert(from_mesh.halfedge_start_vertex_id(opposite_id).unwrap());
                }
                ignore_faces.insert(face_id);
                ignore_faces.insert(opposite_face_id);
                face_pair_map.insert(FacePair::new(face_id, opposite_face_id));
                let mut old_vertices = Vec::new();
                let mut loop_id = next_id;
                while loop_id != halfedge_id {
                    old_vertices.push(from_mesh.halfedge_start_vertex_id(loop_id).unwrap());
                    loop_id = from_mesh.halfedge_next_id(loop_id).unwrap();
                }
                loop_id = opposite_next_id;
                while loop_id != opposite_id {
                    old_vertices.push(from_mesh.halfedge_start_vertex_id(loop_id).unwrap());
                    loop_id = from_mesh.halfedge_next_id(loop_id).unwrap();
                }
                pending_old_verts.push(old_vertices);
            }
        }
        for face_id in FaceIterator::new(from_mesh) {
            if ignore_faces.contains(&face_id) {
                continue;
            }
            let mut old_vertices = Vec::new();
            for halfedge_id in FaceHalfedgeIterator::new(from_mesh, from_mesh.face_first_halfedge_id(face_id).unwrap()) {
                old_vertices.push(from_mesh.halfedge_start_vertex_id(halfedge_id).unwrap());
            }
            pending_old_verts.push(old_vertices);
        }
        for verts in pending_old_verts.iter() {
            let mut added_vertices = Vec::new();
            for old_vert_id in verts.iter() {
                if !ignore_vert_ids.contains(old_vert_id) {
                    let new_vert_id = new_vert_map.entry(*old_vert_id).or_insert_with(|| {
                        to_mesh.add_vertex(from_mesh.vertex(*old_vert_id).unwrap().position)
                    });
                    added_vertices.push(*new_vert_id);
                }
            }
            to_mesh.add_vertices(added_vertices);
        }
        (!ignore_faces.is_empty(), to_mesh)
    }

    pub fn combine_adj_faces(&self) -> Self {
        let mut from_mesh = self.clone();
        let mut combined = true;
        let mut to_mesh = Mesh::new();
        while combined {
            let (sub_combined, sub_to_mesh) = from_mesh.combine_adj_faces_round();
            combined = sub_combined;
            from_mesh = sub_to_mesh.clone();
            to_mesh = sub_to_mesh;
        }
        to_mesh
    }
}

impl Add for Mesh {
    type Output = Mesh;

    fn add(self, other: Mesh) -> Mesh {
        let mut new_mesh = Mesh::new();
        new_mesh.add_mesh(&self);
        new_mesh.add_mesh(&other);
        new_mesh
    }
}

impl AddAssign for Mesh {
    fn add_assign(&mut self, other: Mesh) {
        self.add_mesh(&other);
    }
}

pub trait Export {
    fn export(&self, filename: &str) -> io::Result<()>;
}

pub trait Import {
    fn import(&mut self, filename: &str) -> io::Result<()>;
}

impl Clone for Mesh {
    fn clone(&self) -> Self {
        let mut mesh = Mesh::new();
        mesh.add_mesh(self);
        mesh
    }
}
