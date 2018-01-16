use cgmath::Vector3;
use std::option::Option;
use std::fs::File;
use std::io::prelude::*;
use std::string::String;
use std::str::FromStr;
use std::io;
use std::vec::Vec;
use std::collections::HashMap;

type Id = usize;

#[derive(Debug)]
pub struct Vertex {
    id: Id,
    position: Vector3<f32>,
    index: usize,
    halfedge: Id,
    previous: Id,
    next: Id,
    alive: bool,
}

#[derive(Debug)]
pub struct Face {
    id: Id,
    halfedge: Id,
    previous: Id,
    next: Id,
    alive: bool,
}

#[derive(Debug)]
pub struct Halfedge {
    id: Id,
    vertex: Id,
    face: Id,
    previous: Id,
    next: Id,
    opposite: Id,
    alive: bool,
}


#[derive(Hash, Eq, PartialEq, Debug)]
struct EdgeEndpoints {
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

struct FaceIterator<'a> {
    index: usize,
    mesh: &'a Mesh,
}

impl<'a> Iterator for FaceIterator<'a> {
    type Item = Id;

    fn next(&mut self) -> Option<Id> {
        while self.index < self.mesh.faces.len() {
            if self.mesh.faces[self.index].alive {
                self.index += 1;
                return Some(self.mesh.faces[self.index].id);
            }
        }
        None
    }
}

struct FaceHalfedgeIterator<'a> {
    stop_id: Id,
    current_id: Id,
    mesh: &'a Mesh,
}

impl<'a> Iterator for FaceHalfedgeIterator<'a> {
    type Item = Id;

    fn next(&mut self) -> Option<Id> {
        let current_halfedge = self.mesh.halfedge(self.current_id);
        match current_halfedge {
            Some(halfedge) => {
                if halfedge.next == self.stop_id {
                    return None;
                }
                Some(halfedge.next)
            },
            _ => None
        }
    }
}

#[derive(Debug)]
pub struct Mesh {
    vertices: Vec<Vertex>,
    vertex_count: usize,
    faces: Vec<Face>,
    face_count: usize,
    halfedges: Vec<Halfedge>,
    halfedge_count: usize,
    edges: HashMap<EdgeEndpoints, Id>,
    edge_count: usize,
}

impl Mesh {
    pub fn face_iter<'a>(&'a self) -> FaceIterator<'a> {
        FaceIterator {
            index: 0,
            mesh: self,
        }
    }

    pub fn face_halfedge_iter<'a>(&'a self, start_id: Id) -> FaceHalfedgeIterator<'a> {
        FaceHalfedgeIterator {
            stop_id: start_id,
            current_id: start_id,
            mesh: self,
        }
    }

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

    pub fn add_vertex(&mut self, position: Vector3<f32>) -> usize {
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

    pub fn pair_halfeges(&mut self, first: Id, second: Id) {
        self.halfedge_mut(first).unwrap().opposite = second;
        self.halfedge_mut(second).unwrap().opposite = first;
    }

    pub fn link_halfedges(&mut self, first: Id, second: Id) {
        self.halfedge_mut(first).unwrap().next = second;
        self.halfedge_mut(second).unwrap().previous = first;
        let endpoints = EdgeEndpoints::new(self.halfedge_mut(first).unwrap().vertex,
            self.halfedge_mut(second).unwrap().vertex);
        match self.edges.get(&endpoints) {
            Some(&halfedge) => self.pair_halfeges(first, halfedge),
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
        let mut face_iter = self.face_iter();
        while let Some(face_id) = face_iter.next() {
            let face = self.face(face_id).unwrap();
            let mut face_halfedge_iter = self.face_halfedge_iter(face.id);
            while let Some(halfedge_id) = face_halfedge_iter.next() {
                let halfedge = self.halfedge(halfedge_id).unwrap();
                let vertex = self.vertex(halfedge.vertex).unwrap();
                writeln!(f, "v {} {} {}", vertex.position.x, vertex.position.y, vertex.position.z)?;
            }
        }
        let mut face_iter = self.face_iter();
        while let Some(face_id) = face_iter.next() {
            let face = self.face(face_id).unwrap();
            let mut face_halfedge_iter = self.face_halfedge_iter(face.id);
            while !face_halfedge_iter.next().is_none() {
                i += 1;
                write!(f, " {}", i)?;
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
                    vertex_array.push(self.add_vertex(Vector3::new(x, y, z)));
                },
                Some("f") => {
                    let face_id = self.add_face();
                    let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
                    while let Some(index_str) = words.next() {
                        let index = usize::from_str(index_str).unwrap() - 1;
                        added_halfedges.push((self.add_halfedge(), vertex_array[index]));
                    }
                    let mut i = 0;
                    for (halfedge_id, vertex_id) in added_halfedges.clone() {
                        let first = added_halfedges[i].0;
                        let second = added_halfedges[(i + 1) % added_halfedges.len()].0;
                        self.link_halfedges(first, second);
                        self.vertex_mut(vertex_id).unwrap().halfedge = halfedge_id;
                        self.halfedge_mut(halfedge_id).unwrap().face = face_id;
                        self.halfedge_mut(halfedge_id).unwrap().vertex = vertex_id;
                        i += 1;
                    }
                },
                _ => ()
            }
        }
        Ok(())
    }
}
