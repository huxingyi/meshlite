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
use iterator::FaceHalfedgeIterator;
use iterator::FaceIterator;
use iterator::FaceHalfedgeCollection;
use util::*;
use std::ops::Add;

pub type Id = usize;

#[derive(Debug)]
pub struct Vertex {
    pub id: Id,
    pub position: Point3<f32>,
    pub index: usize,
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


#[derive(Hash, Eq, PartialEq, Debug)]
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
        let halfedge_collection = FaceHalfedgeCollection::new(self, self.face_first_halfedge_id(id).unwrap());
        self.remove_halfedges_from_edges(halfedge_collection.as_vec());
        for halfedge_id in halfedge_collection.into_iter() {
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
            index: 0,
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

    pub fn save_obj(&self, filename: &str) -> io::Result<()> {
        let mut f = File::create(filename)?;
        let mut i = 0;
        let mut vertices = Vec::new();
        let mut vertices_index_set : HashMap<Id, usize> = HashMap::new();
        {
            let mut face_iter = FaceIterator::new(self);
            while let Some(face_id) = face_iter.next() {
                let face = self.face(face_id).unwrap();
                let mut face_halfedge_iter = FaceHalfedgeIterator::new(self, face.halfedge);
                while let Some(halfedge_id) = face_halfedge_iter.next() {
                    let halfedge = self.halfedge(halfedge_id).unwrap();
                    vertices.push(halfedge.vertex);
                }
            }
        }
        for vertex_id in vertices {
            if vertices_index_set.get(&vertex_id).is_none() {
                i += 1;
                vertices_index_set.insert(vertex_id, i);
                let vertex = self.vertex(vertex_id).unwrap();
                writeln!(f, "v {} {} {}", vertex.position.x, vertex.position.y, vertex.position.z)?;
            }
        }
        let mut face_iter = FaceIterator::new(self);
        while let Some(face_id) = face_iter.next() {
            let face = self.face(face_id).unwrap();
            let mut face_halfedge_iter = FaceHalfedgeIterator::new(self, face.halfedge);
            write!(f, "f")?;
            while let Some(halfedge_id) = face_halfedge_iter.next() {
                let halfedge = self.halfedge(halfedge_id).unwrap();
                let vertex = self.vertex(halfedge.vertex).unwrap();
                write!(f, " {}", vertices_index_set.get(&vertex.id).unwrap())?;
            }
            writeln!(f, "")?;
        }
        Ok(())
    }

    pub fn load_obj(&mut self, filename: &str) -> io::Result<()> {
        let mut f = File::open(filename)?;
        let mut contents = String::new();
        f.read_to_string(&mut contents)?;
        let lines = contents.lines();
        let mut vertex_array = Vec::new();
        for line in lines {
            let mut words = line.split_whitespace().filter(|s| !s.is_empty());
            match words.next() {
                Some("v") => {
                    let (x, y, z) = (f32::from_str(words.next().unwrap()).unwrap(),
                        f32::from_str(words.next().unwrap()).unwrap(),
                        f32::from_str(words.next().unwrap()).unwrap());
                    vertex_array.push(self.add_vertex(Point3::new(x, y, z)));
                },
                Some("f") => {
                    let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
                    while let Some(index_str) = words.next() {
                        let index = usize::from_str(index_str).unwrap() - 1;
                        added_halfedges.push((self.add_halfedge(), vertex_array[index]));
                    }
                    self.add_halfedges_and_vertices(added_halfedges);
                },
                _ => ()
            }
        }
        Ok(())
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

    pub fn extrude_face(&mut self, face_id: Id, normal: Vector3<f32>, amount: f32) {
        let mut new_halfedges : Vec<Id> = Vec::new();
        for halfedge_id in FaceHalfedgeIterator::new(self, face_id) {
            new_halfedges.push(halfedge_id);
        }
        self.extrude_halfedges(&new_halfedges, normal, amount);
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

    pub fn transform(&mut self, mat: &Matrix4<f32>) {
        for vertex in self.vertices.iter_mut() {
            vertex.position = mat.transform_point(vertex.position);
        }
    }

    pub fn add_mesh(&mut self, other: &Mesh) {
        let mut vertices_set : HashMap<Id, Id> = HashMap::new();
        for face_id in FaceIterator::new(&other) {
            let face = other.face(face_id).unwrap();
            let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
            for halfedge_id in FaceHalfedgeIterator::new(&other, face.halfedge) {
                let vertex = other.halfedge_start_vertex(halfedge_id).unwrap();
                if let Some(&new_vertex_id) = vertices_set.get(&vertex.id) {
                    added_halfedges.push((self.add_halfedge(), new_vertex_id));
                } else {
                    let new_vertex_id = self.add_vertex(vertex.position);
                    vertices_set.insert(vertex.id, new_vertex_id);
                    added_halfedges.push((self.add_halfedge(), new_vertex_id));
                }
            }
            self.add_halfedges_and_vertices(added_halfedges);
        }
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
