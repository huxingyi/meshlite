use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
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
use util::*;

pub type Id = usize;

#[derive(Debug)]
pub struct Vertex {
    pub id: Id,
    pub position: Point3<f32>,
    pub index: usize,
    pub halfedge: Id,
    pub previous: Id,
    pub next: Id,
    pub alive: bool,
}

#[derive(Debug)]
pub struct Face {
    pub id: Id,
    pub halfedge: Id,
    pub previous: Id,
    pub next: Id,
    pub alive: bool,
}

#[derive(Debug)]
pub struct Halfedge {
    pub id: Id,
    pub vertex: Id,
    pub face: Id,
    pub previous: Id,
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
            .and_then(|o: &Halfedge| Some(o.face))
    }

    pub fn face_adj(&self, id: Id) -> Option<&Face> {
        self.face_adj_id(id).and_then(|id: Id| self.face(id))
    }

    pub fn halfedge_next_id(&self, id: Id) -> Option<Id> {
        self.halfedge(id)
            .and_then(|h: &Halfedge| Some(h.next))
    }

    pub fn face_first_halfedge_id(&self, id: Id) -> Option<Id> {
        self.face(id)
            .and_then(|f: &Face| Some(f.halfedge))
    }

    pub fn halfedge_start_vertex_id(&self, id: Id) -> Option<Id> {
        self.halfedge(id)
            .and_then(|h: &Halfedge| Some(h.vertex))
    }

    pub fn halfedge_start_vertex_mut(&mut self, id: Id) -> Option<&mut Vertex> {
        let vertex_id = self.halfedge_start_vertex_id(id)?;
        self.vertex_mut(vertex_id)
    }

    pub fn halfedge_start_vertex(&self, id: Id) -> Option<&Vertex> {
        let vertex_id = self.halfedge_start_vertex_id(id)?;
        self.vertex(vertex_id)
    }

    pub fn remove_face(&self, id: Id) {
        // TODO:
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
            previous: 0,
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
            previous: 0,
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

    pub fn link_halfedges(&mut self, first: Id, second: Id) {
        self.halfedge_mut(first).unwrap().next = second;
        self.halfedge_mut(second).unwrap().previous = first;
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
            previous: 0,
            next: 0,
            alive: true,
        });
        new_id
    }

    pub fn save_obj(&mut self, filename: &str) -> io::Result<()> {
        let mut f = File::create(filename)?;
        let mut i = 0;
        let mut vertices = Vec::new();
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
            let mut vertex = self.vertex_mut(vertex_id).unwrap();
            if 0 == vertex.index {
                i += 1;
                vertex.index = i;
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
                write!(f, " {}", vertex.index)?;
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
                    let face_id = self.add_face();
                    let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
                    while let Some(index_str) = words.next() {
                        let index = usize::from_str(index_str).unwrap() - 1;
                        added_halfedges.push((self.add_halfedge(), vertex_array[index]));
                    }
                    for &(halfedge_id, vertex_id) in added_halfedges.iter() {
                        self.vertex_mut(vertex_id).unwrap().halfedge = halfedge_id;
                        self.halfedge_mut(halfedge_id).unwrap().face = face_id;
                        self.halfedge_mut(halfedge_id).unwrap().vertex = vertex_id;
                    }
                    self.face_mut(face_id).unwrap().halfedge = added_halfedges[0].0;
                    for i in 0..added_halfedges.len() {
                        let first = added_halfedges[i].0;
                        let second = added_halfedges[(i + 1) % added_halfedges.len()].0;
                        self.link_halfedges(first, second);
                    }
                },
                _ => ()
            }
        }
        Ok(())
    }
}
