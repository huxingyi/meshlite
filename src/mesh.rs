use cgmath::Vector3;
use std::option::Option;
use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::string::String;
use std::str::FromStr;
use std::io;
use std::vec::Vec;

type VertexMutPtr = *mut Vertex;
type HalfedgeMutPtr = *mut Halfedge;
type FaceMutPtr = *mut Face;

pub struct Vertex {
    position: Vector3<f32>,
    index: usize,
    halfedge: Option<HalfedgeMutPtr>,
    previous: Option<VertexMutPtr>,
    next: Option<VertexMutPtr>,
}

pub struct Face {
    halfedge: Option<HalfedgeMutPtr>,
    previous: Option<FaceMutPtr>,
    next: Option<FaceMutPtr>,
}

pub struct Halfedge {
    vertex: Option<VertexMutPtr>,
    face: Option<FaceMutPtr>,
    previous: Option<HalfedgeMutPtr>,
    next: Option<HalfedgeMutPtr>,
    opposite: Option<HalfedgeMutPtr>,
}

#[derive(Debug)]
pub struct Mesh {
    first_vertex: Option<VertexMutPtr>,
    last_vertex: Option<VertexMutPtr>,
    vertex_count: usize,
    first_face: Option<FaceMutPtr>,
    last_face: Option<FaceMutPtr>,
    face_count: usize,
}

impl Vertex {
    pub fn boxed_new(position: Vector3<f32>) -> Box<Vertex> {
        Box::new(Vertex {
            halfedge: None,
            previous: None,
            next: None,
            position : position,
            index: 0,
        })
    }

    fn into_position(self: Box<Self>) -> Vector3<f32> {
        self.position
    }
}

impl Halfedge {
    pub fn boxed_new() -> Box<Halfedge> {
        Box::new(Halfedge {
            vertex: None,
            face: None,
            previous: None,
            next: None,
            opposite: None,
        })
    }
}

impl Face {
    pub fn boxed_new() -> Box<Face> {
        Box::new(Face {
            halfedge: None,
            previous: None,
            next: None,
        })
    }
}

impl Mesh {
    pub fn new() -> Self {
        Mesh {
            first_vertex: None,
            last_vertex: None,
            vertex_count: 0,
            first_face: None,
            last_face: None,
            face_count: 0,
        }
    }

    fn add_halfedge(&mut self, halfedge: Box<Halfedge>) -> HalfedgeMutPtr {
        Box::into_raw(halfedge)
    }

    fn add_face(&mut self, mut face: Box<Face>) -> FaceMutPtr {
        face.next = self.first_face;
        face.previous = None;
        let face_mut_ptr : FaceMutPtr = Box::into_raw(face);
        match self.first_face {
            None => self.last_face = Some(face_mut_ptr),
            Some(first_face) => unsafe {
                (*first_face).previous = Some(face_mut_ptr)
            },
        }
        self.first_face = Some(face_mut_ptr);
        self.face_count += 1;
        face_mut_ptr
    }

    fn add_vertex(&mut self, mut vertex: Box<Vertex>) -> VertexMutPtr {
        vertex.next = self.first_vertex;
        vertex.previous = None;
        let vertex_mut_ptr : VertexMutPtr = Box::into_raw(vertex);
        match self.first_vertex {
            None => self.last_vertex = Some(vertex_mut_ptr),
            Some(first_vertex) => unsafe {
                (*first_vertex).previous = Some(vertex_mut_ptr)
            },
        }
        self.first_vertex = Some(vertex_mut_ptr);
        self.vertex_count += 1;
        vertex_mut_ptr
    }

    pub fn save_obj(&mut self, filename: &str) -> io::Result<()> {
        let mut f = File::create(filename)?;
        let mut i = 0;
        let mut face = self.first_face;
        while let Some(face_mut_ptr) = face {
            unsafe {
                let mut halfedge = (*face_mut_ptr).halfedge;
                let stop = halfedge;
                loop {
                    i += 1;
                    let halfedge_mut_ptr = halfedge.unwrap();
                    let vertex_mut_ptr = (*halfedge_mut_ptr).vertex.unwrap();
                    (*vertex_mut_ptr).index = i;
                    let position = (*vertex_mut_ptr).position;
                    writeln!(f, "v {} {} {}", position.x, position.y, position.z)?;
                    halfedge = (*halfedge_mut_ptr).next;
                    if halfedge == stop {
                        break;
                    }
                }
                face = (*face_mut_ptr).next;
            }
        }
        face = self.first_face;
        while let Some(face_mut_ptr) = face {
            unsafe {
                let mut halfedge = (*face_mut_ptr).halfedge;
                let stop = halfedge;
                write!(f, "f")?;
                loop {
                    let halfedge_mut_ptr = halfedge.unwrap();
                    let vertex_mut_ptr = (*halfedge_mut_ptr).vertex.unwrap();
                    write!(f, " {}", (*vertex_mut_ptr).index)?;
                    halfedge = (*halfedge_mut_ptr).next;
                    if halfedge == stop {
                        break;
                    }
                }
                writeln!(f, "")?;
                face = (*face_mut_ptr).next;
            }
        }
        Ok(())
    }

    pub fn load_obj(&mut self, filename: &str) -> io::Result<()> {
        let mut f = File::open(filename)?;
        let mut contents = String::new();
        f.read_to_string(&mut contents)?;
        let lines = contents.lines();
        let mut vertex_index = 0;
        let mut vertex_array = Vec::new();
        for line in lines {
            let mut words = line.split_whitespace().filter(|s| !s.is_empty());
            match words.next() {
                Some("v") => {
                    let (x, y, z) = (f32::from_str(words.next().unwrap()).unwrap(),
                        f32::from_str(words.next().unwrap()).unwrap(),
                        f32::from_str(words.next().unwrap()).unwrap());
                    let mut boxed_vertex = Vertex::boxed_new(Vector3::new(x, y, z));
                    vertex_index += 1;
                    boxed_vertex.index = vertex_index;
                    vertex_array.push(self.add_vertex(boxed_vertex));
                },
                Some("f") => {
                    let mut face = Face::boxed_new();
                    let mut added_halfedges : Vec<(HalfedgeMutPtr, VertexMutPtr)> = Vec::new();
                    let mut face_mut_ptr = self.add_face(face);
                    while let Some(index_str) = words.next() {
                        let index = usize::from_str(index_str).unwrap() - 1;
                        added_halfedges.push((self.add_halfedge(Halfedge::boxed_new()),
                            vertex_array[index]));
                    }
                    let mut i = 0;
                    unsafe {
                        (*face_mut_ptr).halfedge = Some(added_halfedges[0].0);
                    }
                    for (h, v) in added_halfedges.iter().cloned() {
                        unsafe {
                            let mut first = added_halfedges[i].0;
                            let mut second = added_halfedges[(i + 1) % added_halfedges.len()].0;
                            (*first).next = Some(second);
                            (*second).previous = Some(first);
                            (*v).halfedge = Some(h);
                            (*h).face = Some(face_mut_ptr);
                            (*h).vertex = Some(v);
                        }
                        i += 1;
                    }
                },
                _ => ()
            }
        }
        Ok(())
    }
}
