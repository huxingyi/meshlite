use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Matrix4;
use mesh::Mesh;
use mesh::Id;
use petgraph::prelude::*;
use petgraph::Graph;
use std::collections::HashMap;
use wrap::GiftWrapper;

struct Node {
    radius: f32,
    position: Point3<f32>,
    generated: bool,
}

struct Face4 {
    a: Point3<f32>,
    b: Point3<f32>,
    c: Point3<f32>,
    d: Point3<f32>,
}

struct Edge {
    cuts: HashMap<NodeIndex, Face4>,
}

pub struct Bmesh {
    graph : Graph<Node, Edge, Undirected>,
    mesh: Mesh,
}

impl Bmesh {
    pub fn new() -> Self {
        Bmesh {
            graph: Graph::new_undirected(),
            mesh: Mesh::new(),
        }
    }

    pub fn add_node(&mut self, position: Point3<f32>, radius: f32) -> usize {
        let node = Node::new(radius, position);
        self.graph.add_node(node).index()
    }

    pub fn add_edge(&mut self, first_node_id: usize, second_node_id: usize) -> usize {
        let edge = Edge::new();
        self.graph.add_edge(NodeIndex::new(first_node_id), NodeIndex::new(second_node_id), edge).index()
    }

    fn direct_of_nodes(&self, first_node_index: NodeIndex, second_node_index: NodeIndex) -> Vector3<f32> {
        let first_node = self.graph.node_weight(first_node_index).unwrap();
        let second_node = self.graph.node_weight(second_node_index).unwrap();
        let direct = second_node.position - first_node.position;
        direct.normalize()
    }

    fn make_cut(&self, position: Point3<f32>, direct: Vector3<f32>, radius: f32) -> Face4 {
        let world_y_axis = Vector3 {x: 0.0, y: 1.0, z: 0.0};
        let local_y = world_y_axis.cross(direct);
        let local_z = local_y.cross(direct);
        let y = local_y * radius;
        let z = local_z * radius;
        let origin = position + direct * radius;
        Face4 {a: origin - y + z,
            b: origin + y + z,
            c: origin + y - z,
            d: origin - y - z}
    }

    fn generate_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().generated {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().generated = true;
        let node_position = self.graph.node_weight(node_index).unwrap().position;
        let node_radius = self.graph.node_weight(node_index).unwrap().radius;
        let mut new_faces : Vec<(EdgeIndex, Face4)> = Vec::new();
        let mut new_indices : Vec<NodeIndex> = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            let mut node_around_mesh = Mesh::new();
            let mut added_faces = Vec::new();
            for other_index in neighbors {
                let direct = self.direct_of_nodes(node_index, other_index);
                let edge_index = self.graph.find_edge(node_index, other_index).unwrap();
                let face = self.make_cut(node_position, direct, node_radius);
                let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
                added_halfedges.push((node_around_mesh.add_halfedge(), node_around_mesh.add_vertex(face.a)));
                added_halfedges.push((node_around_mesh.add_halfedge(), node_around_mesh.add_vertex(face.b)));
                added_halfedges.push((node_around_mesh.add_halfedge(), node_around_mesh.add_vertex(face.c)));
                added_halfedges.push((node_around_mesh.add_halfedge(), node_around_mesh.add_vertex(face.d)));
                let new_face_id = node_around_mesh.add_halfedges_and_vertices(added_halfedges);
                added_faces.push(new_face_id);
                new_faces.push((edge_index, face));
                new_indices.push(other_index);
            }
            if added_faces.len() > 1 {
                let mut wrapper = GiftWrapper::new();
                wrapper.wrap_faces(&mut node_around_mesh, added_faces);
                self.mesh += node_around_mesh;
            }
        }
        for (edge_index, face) in new_faces {
            let ref mut edge = self.graph.edge_weight_mut(edge_index).unwrap();
            edge.cuts.insert(node_index, face);
        }
        for other_index in new_indices {
            self.generate_from_node(other_index);
        }
    }

    pub fn generate_mesh(&mut self, root: usize) -> &Mesh {
        self.generate_from_node(NodeIndex::new(root));
        &self.mesh
    }
}

impl Node {
    fn new(radius: f32, position: Point3<f32>) -> Self {
        Node {
            radius: radius,
            position: position,
            generated: false,
        }
    }
}

impl Edge {
    fn new() -> Self {
        Edge {
            cuts: HashMap::new(),
        }
    }
}
