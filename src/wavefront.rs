use cgmath::Point3;
use mesh::Mesh;
use mesh::Id;
use mesh::Export;
use mesh::Import;
use std::fs::File;
use std::io::prelude::*;
use std::io;
use std::vec::Vec;
use std::collections::HashMap;
use iterator::FaceHalfedgeIterator;
use iterator::FaceIterator;
use std::string::String;
use std::str::FromStr;

impl Export for Mesh {
    fn export(&self, filename: &str) -> io::Result<()> {
        let mut f = File::create(filename)?;
        let mut i = 0;
        let mut vertices = Vec::new();
        let mut vertices_index_set : HashMap<Id, usize> = HashMap::new();
        writeln!(f, "# Export by meshlite")?;
        writeln!(f, "# https://github.com/huxingyi/meshlite")?;
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
        /*
        let mut face_iter = FaceIterator::new(self);
        while let Some(face_id) = face_iter.next() {
            let normal = self.face_norm(face_id);
            writeln!(f, "vn {} {} {}", normal.x, normal.y, normal.z)?;
        }*/
        let mut face_iter = FaceIterator::new(self);
        //let mut face_index = 0;
        while let Some(face_id) = face_iter.next() {
            let face = self.face(face_id).unwrap();
            let mut face_halfedge_iter = FaceHalfedgeIterator::new(self, face.halfedge);
            write!(f, "f")?;
            //face_index += 1;
            while let Some(halfedge_id) = face_halfedge_iter.next() {
                let halfedge = self.halfedge(halfedge_id).unwrap();
                let vertex = self.vertex(halfedge.vertex).unwrap();
                write!(f, " {}", vertices_index_set.get(&vertex.id).unwrap())?;
                //write!(f, " {}//{}", vertices_index_set.get(&vertex.id).unwrap(), face_index)?;
            }
            writeln!(f, "")?;
        }
        Ok(())
    }
}

impl Import for Mesh {
    fn import(&mut self, filename: &str) -> io::Result<()> {
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
}

