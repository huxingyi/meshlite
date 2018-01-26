use mesh::Mesh;
use mesh::Id;

pub struct VertexEdgeIterator<'a> {
    stop_id: Id,
    current_id: Id,
    index: usize,
    mesh: &'a Mesh,
}

impl<'a> Iterator for VertexEdgeIterator<'a> {
    type Item = Id;

    fn next(&mut self) -> Option<Id> {
        let id = self.current_id;
        self.current_id = self.mesh.halfedge(self.mesh.halfedge(self.current_id).unwrap().prev).unwrap().opposite;
        if id == self.stop_id && self.index > 0 {
            return None;
        }
        self.index += 1;
        Some(id)
    }
}

impl<'a> VertexEdgeIterator<'a> {
    pub fn new(mesh: &'a Mesh, start_id: Id) -> VertexEdgeIterator<'a> {
        VertexEdgeIterator {
            stop_id: start_id,
            current_id: start_id,
            index: 0,
            mesh: mesh,
        }
    }
}

pub struct VertexEdgeCollection(Vec<Id>);

impl VertexEdgeCollection {
    pub fn new(mesh: &Mesh, start_id: Id) -> VertexEdgeCollection {
        let mut halfedge_iter = VertexEdgeIterator::new(mesh, start_id);
        let mut collection = VertexEdgeCollection(Vec::new());
        while let Some(face_id) = halfedge_iter.next() {
            collection.0.push(face_id);
        }
        collection
    }
}

impl IntoIterator for VertexEdgeCollection {
    type Item = Id;
    type IntoIter = ::std::vec::IntoIter<Id>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

pub struct FaceIterator<'a> {
    index: usize,
    mesh: &'a Mesh,
}

impl<'a> Iterator for FaceIterator<'a> {
    type Item = Id;

    fn next(&mut self) -> Option<Id> {
        while self.index < self.mesh.faces.len() {
            if self.mesh.faces[self.index].alive {
                self.index += 1;
                return Some(self.mesh.faces[self.index - 1].id);
            }
            self.index += 1;
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
}

pub struct FaceCollection(Vec<Id>);

impl FaceCollection {
    pub fn new(mesh: &Mesh) -> FaceCollection {
        let mut face_iter = FaceIterator::new(mesh);
        let mut collection = FaceCollection(Vec::new());
        while let Some(face_id) = face_iter.next() {
            collection.0.push(face_id);
        }
        collection
    }
}

impl IntoIterator for FaceCollection {
    type Item = Id;
    type IntoIter = ::std::vec::IntoIter<Id>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

pub struct FaceHalfedgeCollection(Vec<Id>);

impl FaceHalfedgeCollection {
    pub fn new(mesh: &Mesh, start_id: Id) -> FaceHalfedgeCollection {
        let mut edge_iter = FaceHalfedgeIterator::new(mesh, start_id);
        let mut collection = FaceHalfedgeCollection(Vec::new());
        while let Some(halfedge_id) = edge_iter.next() {
            collection.0.push(halfedge_id);
        }
        collection
    }

    pub fn as_vec(&self) -> &Vec<Id> {
        &self.0
    }
}

impl IntoIterator for FaceHalfedgeCollection {
    type Item = Id;
    type IntoIter = ::std::vec::IntoIter<Id>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
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
}

