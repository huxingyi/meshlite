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
use util::*;

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
    cuts: Vec<Id>,
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
        let mut u = world_y_axis.cross(direct);
        let mut v = u.cross(direct);
        if u == Vector3::zero() {
            u = Vector3 {x: 1.0, y: 0.0, z: 0.0};
        }
        if v == Vector3::zero() {
            v = Vector3 {x: 0.0, y: 0.0, z: 1.0};
        }
        println!("u: {:?} v: {:?} direct: {:?}", u, v, direct);
        let u = u * radius * 0.6;
        let v = v * radius * 0.6;
        let origin = position + direct * radius;
        let mut f = Face4 {a: origin - u - v,
            b: origin + u - v,
            c: origin + u + v,
            d: origin - u + v};
        if direct.cross(norm(f.a, f.b, f.c)) == Vector3::zero() {
            f = Face4 {a: origin - u + v,
                b: origin + u + v,
                c: origin + u - v,
                d: origin - u - v};
        }
        f
    }

    fn generate_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().generated {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().generated = true;
        let node_position = self.graph.node_weight(node_index).unwrap().position;
        let node_radius = self.graph.node_weight(node_index).unwrap().radius;
        let mut new_faces : Vec<(EdgeIndex, Id)> = Vec::new();
        let mut new_indices : Vec<NodeIndex> = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            let mut added_faces = Vec::new();
            for other_index in neighbors {
                let direct = self.direct_of_nodes(node_index, other_index);
                let edge_index = self.graph.find_edge(node_index, other_index).unwrap();
                let face = self.make_cut(node_position, direct, node_radius);
                let mut added_halfedges : Vec<(Id, Id)> = Vec::new();
                added_halfedges.push((self.mesh.add_halfedge(), self.mesh.add_vertex(face.a)));
                added_halfedges.push((self.mesh.add_halfedge(), self.mesh.add_vertex(face.b)));
                added_halfedges.push((self.mesh.add_halfedge(), self.mesh.add_vertex(face.c)));
                added_halfedges.push((self.mesh.add_halfedge(), self.mesh.add_vertex(face.d)));
                let new_face_id = self.mesh.add_halfedges_and_vertices(added_halfedges);
                added_faces.push(new_face_id);
                new_faces.push((edge_index, new_face_id));
                new_indices.push(other_index);
            }
            if added_faces.len() > 1 {
                let mut wrapper = GiftWrapper::new();
                wrapper.wrap_faces(&mut self.mesh, &added_faces);
            }
        }
        for (edge_index, face_id) in new_faces {
            let ref mut edge = self.graph.edge_weight_mut(edge_index).unwrap();
            edge.cuts.push(face_id);
        }
        for other_index in new_indices {
            self.generate_from_node(other_index);
        }
    }

    fn stitch_by_edges(&mut self) {
        for edge in self.graph.edge_weights_mut() {
            assert!(edge.cuts.len() == 2);
            let mut wrapper = GiftWrapper::new();
            wrapper.stitch_two_faces(&mut self.mesh, edge.cuts[0], edge.cuts[1]);
        }
    }

    pub fn generate_mesh(&mut self, root: usize) -> &mut Mesh {
        self.generate_from_node(NodeIndex::new(root));
        self.stitch_by_edges();
        &mut self.mesh
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
            cuts: Vec::new(),
        }
    }
}


