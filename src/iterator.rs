use mesh::Mesh;
use mesh::Id;

pub struct FaceIterator<'a> {
    index: usize,
    mesh: &'a Mesh,
}

impl<'a> Iterator for FaceIterator<'a> {
    type Item = Id;

    fn next(&mut self) -> Option<Id> {
        while self.index < self.mesh.faces.len() {
            let face = &self.mesh.faces[self.index];
            self.index += 1;
            if face.alive {
                return Some(face.id)
            }
        }
        None
    }
}

impl<'a> FaceIterator<'a> {
    pub fn new(mesh: &'a Mesh) -> FaceIterator<'a> {
        FaceIterator {
            index: 0,
            mesh: mesh,
        }
    }

    pub fn into_vec(self) -> Vec<Id> {
        let mut vec = Vec::new();
        for id in self {
            vec.push(id);
        }
        vec
    }
}

pub struct FaceHalfedgeIterator<'a> {
    stop_id: Id,
    current_id: Id,
    index: usize,
    mesh: &'a Mesh,
}

impl<'a> Iterator for FaceHalfedgeIterator<'a> {
    type Item = Id;

    fn next(&mut self) -> Option<Id> {
        let id = self.current_id;
        if self.mesh.halfedge_start_vertex(id).is_none() {
            return None;
        }
        self.current_id = self.mesh.halfedge(self.current_id).unwrap().next;
        if id == self.stop_id && self.index > 0 {
            return None;
        }
        self.index += 1;
        Some(id)
    }
}

impl<'a> FaceHalfedgeIterator<'a> {
    pub fn new(mesh: &'a Mesh, start_id: Id) -> FaceHalfedgeIterator<'a> {
        FaceHalfedgeIterator {
            stop_id: start_id,
            current_id: start_id,
            index: 0,
            mesh: mesh,
        }
    }

    pub fn into_vec(self) -> Vec<Id> {
        let mut vec = Vec::new();
        for id in self {
            vec.push(id);
        }
        vec
    }
}

pub struct VertexHalfedgeIterator<'a> {
    stop_id: Id,
    current_id: Id,
    index: usize,
    mesh: &'a Mesh,
}

impl<'a> Iterator for VertexHalfedgeIterator<'a> {
    type Item = Id;

    fn next(&mut self) -> Option<Id> {
        let id = self.current_id;
        if self.mesh.halfedge_start_vertex(id).is_none() {
            return None;
        }
        self.current_id = {
            if 0 == self.index % 2 {
                self.mesh.halfedge(self.current_id).unwrap().opposite
            } else {
                self.mesh.halfedge(self.current_id).unwrap().next
            }
        };
        if id == self.stop_id && self.index > 0 {
            return None;
        }
        self.index += 1;
        Some(id)
    }
}

impl<'a> VertexHalfedgeIterator<'a> {
    pub fn new(mesh: &'a Mesh, start_id: Id) -> VertexHalfedgeIterator<'a> {
        VertexHalfedgeIterator {
            stop_id: start_id,
            current_id: start_id,
            index: 0,
            mesh: mesh,
        }
    }

    pub fn into_vec(self) -> Vec<Id> {
        let mut vec = Vec::new();
        for id in self {
            vec.push(id);
        }
        vec
    }
}
