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
    thickness: f32,
    position: Point3<f32>,
    generated: bool,
}

struct Edge {
    cuts: Vec<(Vec<Id>, Vector3<f32>)>,
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

    pub fn add_node(&mut self, position: Point3<f32>, radius: f32, thickness: f32) -> usize {
        let node = Node::new(radius, thickness, position);
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

    fn make_cut(&self, position: Point3<f32>, direct: Vector3<f32>, radius: f32, thickness: f32, base_plane_norm: Vector3<f32>) -> Vec<Point3<f32>> {
        let world_y_axis = {
            if base_plane_norm == Vector3::zero() {
                Vector3 {x: 0.0, y: 1.0, z: 0.0}
            } else {
                base_plane_norm
            }
        };
        let mut u = world_y_axis.cross(direct);
        let mut v = u.cross(direct);
        if u == Vector3::zero() {
            u = Vector3 {x: 1.0, y: 0.0, z: 0.0};
        }
        if v == Vector3::zero() {
            v = Vector3 {x: 0.0, y: 0.0, z: 1.0};
        }
        let u = u.normalize() * radius;
        let v = v.normalize() * thickness;
        let origin = position + direct * radius;
        let mut f = vec![origin - u - v,
            origin + u - v,
            origin + u + v,
            origin - u + v];
        if direct.cross(norm(f[0], f[1], f[2])) == Vector3::zero() {
            f = vec![origin - u + v,
                origin + u + v,
                origin + u - v,
                origin - u - v];
        }
        f
    }

    fn find_edge_plane_norm(&self, node_index: NodeIndex, other_index: NodeIndex) -> Vector3<f32> {
        let mut directs = Vec::new();
        let edge_direct = self.direct_of_nodes(node_index, other_index);
        for another_index in self.graph.neighbors_undirected(node_index) {
            if another_index == other_index {
                continue;
            }
            directs.push(self.direct_of_nodes(node_index, another_index));
        }
        if directs.len() == 0 {
            for another_index in self.graph.neighbors_undirected(other_index) {
                if another_index == node_index {
                    continue;
                }
                directs.push(self.direct_of_nodes(other_index, another_index));
            }
            if directs.len() == 0 {
                return Vector3::zero();
            }
            return find_average_plane_norm(edge_direct, directs);
        }
        find_average_plane_norm(edge_direct, directs)
    }

    fn generate_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().generated {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().generated = true;
        let node_position = self.graph.node_weight(node_index).unwrap().position;
        let node_radius = self.graph.node_weight(node_index).unwrap().radius;
        let node_thickness = self.graph.node_weight(node_index).unwrap().thickness;
        let mut new_cuts : Vec<(EdgeIndex, (Vec<Id>, Vector3<f32>))> = Vec::new();
        let mut new_indices : Vec<NodeIndex> = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            let mut neighbors_count = 0;
            let mut directs = Vec::new();
            let mut other_factors : HashMap<NodeIndex, f32> = HashMap::new();
            for other_index in neighbors.clone() {
                let direct = self.direct_of_nodes(node_index, other_index);
                directs.push(direct);
                new_indices.push(other_index);
                other_factors.entry(other_index).or_insert(0.0);
                neighbors_count += 1;
            }
            if neighbors_count == 1 {
                let direct = directs[0];
                let face = self.make_cut(node_position - direct * node_radius, direct, node_radius, node_thickness,
                    self.find_edge_plane_norm(node_index, new_indices[0]));
                let mut vert_ids = Vec::new();
                for vert in face {
                    vert_ids.push(self.mesh.add_vertex(vert));
                }
                for other_index in neighbors.clone() {
                    let edge_index = self.graph.find_edge(node_index, other_index).unwrap();
                    new_cuts.push((edge_index, (vert_ids.clone(), -direct)));
                }
                self.mesh.add_vertices(vert_ids);
            } else if neighbors_count == 2 {
                let mut order = 0;
                let direct = (directs[0] - directs[1]) / 2.0;
                let face = self.make_cut(node_position - direct * node_radius, direct, node_radius, node_thickness,
                    self.find_edge_plane_norm(node_index, new_indices[0]));
                let mut vert_ids = Vec::new();
                for vert in face {
                    vert_ids.push(self.mesh.add_vertex(vert));
                }
                let mut rev_vert_ids = vert_ids.clone();
                rev_vert_ids.reverse();
                let cut_faces = vec![vert_ids, rev_vert_ids];
                let cut_directs = vec![direct, -direct];
                for other_index in neighbors.clone() {
                    let edge_index = self.graph.find_edge(node_index, other_index).unwrap();
                    new_cuts.push((edge_index, (cut_faces[order].clone(), -cut_directs[order])));
                    order += 1;
                }
            } else if neighbors_count >= 3 {
                let mut cuts : Vec<(Vec<Point3<f32>>, EdgeIndex, NodeIndex, Vector3<f32>)> = Vec::new();
                let max_round : usize = 10;
                let factor_step = 1.0 / max_round as f32;
                for round in 0..max_round {
                    for other_index in neighbors.clone() {
                        let mut direct = self.direct_of_nodes(node_index, other_index);
                        let edge_index = self.graph.find_edge(node_index, other_index).unwrap();
                        let mut create_origin = node_position;
                        let other_position = self.graph.node_weight(other_index).unwrap().position;
                        let other_radius = self.graph.node_weight(other_index).unwrap().radius;
                        let distance = other_position.distance(create_origin);
                        let mut origin_moved_distance = 0.0;
                        if round > 0 {
                            let factor = other_factors[&other_index];
                            origin_moved_distance = distance * factor;
                            create_origin = create_origin + direct * origin_moved_distance;
                        }
                        let dist_factor = (node_radius + origin_moved_distance) / distance;
                        let create_radius = node_radius * (1.0 - dist_factor) + other_radius * dist_factor;
                        let face = self.make_cut(create_origin, direct, create_radius, node_thickness,
                            self.find_edge_plane_norm(node_index, other_index));
                        cuts.push((face, edge_index, other_index, direct.normalize()));
                    }
                    let mut intersects = false;
                    for j in 0..cuts.len() {
                        for k in j + 1..cuts.len() {
                            let first = &cuts[j].0;
                            let second = &cuts[k].0;
                            if is_two_quads_intersect(&vec![first[0], first[1], first[2], first[3]],
                                    &vec![second[0], second[1], second[2], second[3]]) {
                                intersects = true;
                                *other_factors.get_mut(&cuts[j].2).unwrap() += factor_step;
                                *other_factors.get_mut(&cuts[k].2).unwrap() += factor_step;
                            }
                        }
                    }
                    if !intersects {
                        break;
                    }
                    cuts = Vec::new();
                }
                if cuts.len() > 0 {
                    let mut added_loops : Vec<(Vec<Id>, Vector3<f32>)> = Vec::new();
                    for (face, edge_index, other_index, direct) in cuts {
                        let mut vert_ids = Vec::new();
                        for vert in face {
                            vert_ids.push(self.mesh.add_vertex(vert));
                        }
                        let mut rev_vert_ids = vert_ids.clone();
                        rev_vert_ids.reverse();
                        added_loops.push((rev_vert_ids, direct));
                        new_cuts.push((edge_index, (vert_ids, -direct)));
                    }
                    if added_loops.len() > 1 {
                        let mut wrapper = GiftWrapper::new();
                        wrapper.wrap_vertices(&mut self.mesh, &added_loops);
                    }
                }
            }
        }
        for (edge_index, cut) in new_cuts {
            let ref mut edge = self.graph.edge_weight_mut(edge_index).unwrap();
            edge.cuts.push(cut);
        }
        for other_index in new_indices {
            self.generate_from_node(other_index);
        }
    }

    fn stitch_by_edges(&mut self) {
        for edge in self.graph.edge_weights_mut() {
            match edge.cuts.len() {
                2 => {
                    let mut wrapper = GiftWrapper::new();
                    wrapper.wrap_vertices(&mut self.mesh, &edge.cuts);
                },
                _ => {}
            }
        }
    }

    pub fn generate_mesh(&mut self, root: usize) -> &mut Mesh {
        self.generate_from_node(NodeIndex::new(root));
        self.stitch_by_edges();
        &mut self.mesh
    }
}

impl Node {
    fn new(radius: f32, thickness: f32, position: Point3<f32>) -> Self {
        Node {
            radius: radius,
            thickness: thickness,
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
