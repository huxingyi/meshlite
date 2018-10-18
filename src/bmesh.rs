use cgmath::InnerSpace;
use cgmath::Point3;
use cgmath::Vector3;
use debug::Debug;
use mesh::Id;
use mesh::Mesh;
use petgraph::Graph;
use petgraph::prelude::*;
use std::collections::HashMap;
use subdivide::Subdivide;
use triangulate::Triangulate;
use util::*;
use wrap::GiftWrapper;

struct Node {
    radius: f32,
    position: Point3<f32>,
    base_norm: Vector3<f32>,
    generated: bool,
    base_norm_resolved: bool,
    base_norm_searched: bool,
    cut_subdiv_count: Option<usize>,
    round_way: Option<i32>,
    generate_from_node_id: Option<usize>,
    seam_resolved: bool,
    generated_vertices: Vec<Vec<Id>>,
    insert_order: isize,
}

struct Edge {
    cuts: Vec<(Vec<Id>, Vector3<f32>)>,
}

pub struct Bmesh {
    graph : Graph<Node, Edge, Undirected>,
    mesh: Mesh,
    neighbor_count_map: HashMap<usize, usize>,
    neighbor_count_vec: Vec<(usize, usize, isize)>,
    wrap_error_count: i32,
    node_count: usize,
    debug_enabled: bool,
    cut_subdiv_count: usize,
    round_way: i32,
    deform_thickness: f32,
    deform_width: f32,
    vertex_node_map: HashMap<Id, NodeIndex>,
    vertex_cut_direct_map: HashMap<Id, Vector3<f32>>,
    round_steps: usize,
    pub seams: Vec<Vec<usize>>,
    seam_required: bool,
    last_node_id: usize,
}

impl Bmesh {
    pub fn new() -> Self {
        Bmesh {
            graph: Graph::new_undirected(),
            mesh: Mesh::new(),
            neighbor_count_map: HashMap::new(),
            neighbor_count_vec: Vec::new(),
            wrap_error_count: 0,
            node_count: 0,
            debug_enabled: false,
            cut_subdiv_count: 0,
            round_way: 0,
            deform_thickness: 1.0,
            deform_width: 1.0,
            vertex_node_map: HashMap::new(),
            vertex_cut_direct_map: HashMap::new(),
            round_steps: 1,
            seams: Vec::new(),
            seam_required: false,
            last_node_id: 0,
        }
    }

    pub fn set_cut_subdiv_count(&mut self, count: usize) {
        self.cut_subdiv_count = count;
    }

    pub fn set_round_way(&mut self, round_way: i32) {
        self.round_way = round_way;
    }

    pub fn set_deform_thickness(&mut self, thickness: f32) {
        self.deform_thickness = thickness;
    }

    pub fn set_deform_width(&mut self, width: f32) {
        self.deform_width = width;
    }

    pub fn enable_debug(&mut self, enable: bool) {
        self.debug_enabled = enable;
    }

    pub fn add_seam_requirement(&mut self) {
        self.seam_required = true;
    }

    pub fn get_node_base_norm(&self, node_id: usize) -> Vector3<f32> {
        self.graph.node_weight(NodeIndex::new(node_id)).unwrap().base_norm
    }

    pub fn set_node_cut_subdiv_count(&mut self, node_id: usize, subdiv_count: usize) {
        self.graph.node_weight_mut(NodeIndex::new(node_id)).unwrap().cut_subdiv_count = Some(subdiv_count);
    }

    pub fn set_node_round_way(&mut self, node_id: usize, round_way: i32) {
        self.graph.node_weight_mut(NodeIndex::new(node_id)).unwrap().round_way = Some(round_way);
    }

    fn resolve_base_norm_for_leaves_from_node(&mut self, node_index: NodeIndex, base_norm: Vector3<f32>) {
        if self.graph.node_weight(node_index).unwrap().base_norm_resolved {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().base_norm_resolved = true;
        self.graph.node_weight_mut(node_index).unwrap().base_norm = base_norm;
        let mut indicies = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            for other_index in neighbors {
                indicies.push(other_index);
            }
        }
        for other_index in indicies {
            match self.neighbor_count_map[&other_index.index()] {
                1 => self.resolve_base_norm_for_leaves_from_node(other_index, base_norm),
                2 => {
                    let edge_base_norm = self.calculate_node_base_norm(other_index);
                    if edge_base_norm.is_none() {
                        self.resolve_base_norm_for_leaves_from_node(other_index, base_norm)
                    } else {
                        self.resolve_base_norm_for_leaves_from_node(other_index, edge_base_norm.unwrap())
                    }
                },
                _ => {},
            }
        }
    }

    fn calculate_node_base_norm(&self, node_index: NodeIndex) -> Option<Vector3<f32>> {
        let mut directs : Vec<Vector3<f32>> = Vec::new();
        let mut positions : Vec<Point3<f32>> = Vec::new();
        let mut weights : Vec<f32> = Vec::new();
        let neighbors = self.graph.neighbors_undirected(node_index);
        for other_index in neighbors {
            let direct = self.direct_of_nodes(node_index, other_index);
            let other = self.graph.node_weight(other_index).unwrap();
            directs.push(direct);
            positions.push(other.position);
            weights.push(other.radius);
        }
        pick_base_plane_norm(directs, positions, weights)
    }

    fn search_a_no_none_base_norm_from_neighbors(&mut self, node_index: NodeIndex) -> Option<Vector3<f32>> {
        let mut indicies = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            for other_index in neighbors {
                indicies.push(other_index);
            }
        }
        for other_index in indicies.clone() {
            let base_norm = self.calculate_node_base_norm(other_index);
            if base_norm.is_some() {
                return base_norm;
            }
        }
        for other_index in indicies {
            if self.graph.node_weight(other_index).unwrap().base_norm_searched {
                continue;
            }
            self.graph.node_weight_mut(other_index).unwrap().base_norm_searched = true;
            let base_norm = self.search_a_no_none_base_norm_from_neighbors(other_index);
            if base_norm.is_some() {
                return base_norm;
            }
        }
        None
    }

    fn resolve_base_norm_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().base_norm_resolved {
            return;
        }
        let mut base_norm = self.calculate_node_base_norm(node_index);
        if base_norm.is_none() {
            // Find a no none base norm from neighbors
            self.graph.node_weight_mut(node_index).unwrap().base_norm_searched = true;
            let base_norm_of_some_direct_or_indirect_neighbor = self.search_a_no_none_base_norm_from_neighbors(node_index);
            // If all the base norm are none, then we use the Z axis
            base_norm = if base_norm_of_some_direct_or_indirect_neighbor.is_none() {
                const WORLD_Z_AXIS : Vector3<f32> = Vector3 {x: 0.0, y: 0.0, z: 1.0};
                Some(WORLD_Z_AXIS)
            } else {
                base_norm_of_some_direct_or_indirect_neighbor
            }
        }
        self.resolve_base_norm_for_leaves_from_node(node_index, base_norm.unwrap());
    }

    fn reorder_nodes_by_neighbor_count(&mut self) {
        let mut vec : Vec<(usize, usize, isize)> = Vec::new();
        for (&k, &v) in self.neighbor_count_map.iter() {
            vec.push((k, v, self.graph.node_weight(NodeIndex::new(k)).unwrap().insert_order));
        }
        vec.sort_by(|a, b| {
            if b.1 == a.1 {
                a.2.cmp(&b.2)
            } else {
                b.1.cmp(&a.1)
            }
        });
        self.neighbor_count_vec = vec;
    }

    fn resolve_base_norm(&mut self) {
        for &(node_id, _neighbor_count, _) in self.neighbor_count_vec.clone().iter() {
            self.resolve_base_norm_from_node(NodeIndex::new(node_id));
        }
    }

    fn output_debug_info_if_enabled(&mut self) {
        if self.debug_enabled {
            for &(node_id, _, _) in self.neighbor_count_vec.iter() {
                let node_index = NodeIndex::new(node_id);
                let node_origin = self.graph.node_weight(node_index).unwrap().position;
                let base_norm = self.graph.node_weight(node_index).unwrap().base_norm;
                self.mesh.add_debug_norm(node_origin, base_norm);
            }
        }
    }

    pub fn add_node(&mut self, position: Point3<f32>, radius: f32) -> usize {
        //println!("add_node position:{:?} radius:{:?}", position, radius);
        let node = Node::new(radius, position);
        let node_index = self.graph.add_node(node);
        let node_id = node_index.index();
        self.graph.node_weight_mut(node_index).unwrap().insert_order = self.node_count as isize;
        self.node_count += 1;
        self.last_node_id = node_id;
        node_id
    }

    pub fn add_edge(&mut self, first_node_id: usize, second_node_id: usize) -> usize {
        let edge = Edge::new();
        *self.neighbor_count_map.entry(first_node_id).or_insert(0) += 1;
        *self.neighbor_count_map.entry(second_node_id).or_insert(0) += 1;
        self.graph.add_edge(NodeIndex::new(first_node_id), NodeIndex::new(second_node_id), edge).index()
    }

    fn direct_of_nodes(&self, first_node_index: NodeIndex, second_node_index: NodeIndex) -> Vector3<f32> {
        let first_node = self.graph.node_weight(first_node_index).unwrap();
        let second_node = self.graph.node_weight(second_node_index).unwrap();
        let direct = second_node.position - first_node.position;
        direct.normalize()
    }

    fn make_cut(&self, position: Point3<f32>, direct: Vector3<f32>, radius: f32, base_norm: Vector3<f32>, subdiv_count: usize) -> Vec<Point3<f32>> {
        let mut cut : Vec<Point3<f32>> = make_quad(position, direct, radius, base_norm);
        let origin = position + direct * radius;
        for _ in 0..subdiv_count {
            let mut middle_cut : Vec<Point3<f32>> = Vec::new();
            let mut final_cut : Vec<Point3<f32>> = Vec::new();
            let length = (cut[0] - origin).magnitude() * 0.8;
            for i in 0..cut.len() {
                let a = cut[i] - origin;
                let b = cut[(i + 1) % cut.len()] - origin;
                let c = a + b;
                let new_point = origin + c.normalize_to(length);
                middle_cut.push(new_point);
            }
            let length = (middle_cut[0] - origin).magnitude();
            for i in 0..middle_cut.len() {
                let a = middle_cut[i] - origin;
                let b = middle_cut[(i + 1) % middle_cut.len()] - origin;
                let c = a + b;
                let new_point = origin + c.normalize_to(length);
                final_cut.push(middle_cut[i]);
                final_cut.push(new_point);
            }
            cut = final_cut;
        }
        cut
    }

    pub fn resolve_round(&mut self) {
        let mut new_end_starts : Vec<(NodeIndex, Vector3<f32>)> = Vec::new();
        for (&k, &v) in self.neighbor_count_map.iter() {
            if 1 == v {
                let node_index = NodeIndex::new(k);
                let neighbors = self.graph.neighbors_undirected(node_index);
                for other_index in neighbors {
                    let direct = self.direct_of_nodes(node_index, other_index);
                    new_end_starts.push((node_index, -direct));
                }
            }
        }
        for end_start in new_end_starts {
            let node_origin = self.graph.node_weight(end_start.0).unwrap().position;
            let node_radius = self.graph.node_weight(end_start.0).unwrap().radius;
            let node_round_way = self.graph.node_weight(end_start.0).unwrap().round_way;
            let node_cut_subdiv_count = self.graph.node_weight(end_start.0).unwrap().cut_subdiv_count;
            let round_way = {
                if node_round_way.is_none() {
                    self.round_way
                } else {
                    node_round_way.unwrap()
                }
            };
            if 0 != round_way {
                let mut step_radius = node_radius;
                let mut step_from = node_origin;
                let mut wait_connect_node_ids : Vec<Id> = Vec::new();
                let step_direct = if round_way > 0 {
                    end_start.1
                } else {
                    // FIXME: The stitch step will encounter flipped normal problem because we flipped the edge direction here.
                    -end_start.1
                };
                wait_connect_node_ids.push(end_start.0.index());
                for _ in 0..self.round_steps {
                    step_radius *= 0.5;
                    step_from = step_from + step_direct * step_radius;
                    let new_node_id = self.add_node(step_from, step_radius);
                    self.graph.node_weight_mut(NodeIndex::new(new_node_id)).unwrap().generate_from_node_id = Some(end_start.0.index());
                    wait_connect_node_ids.push(new_node_id);
                }
                for i in 1..wait_connect_node_ids.len() {
                    let from_node_id = wait_connect_node_ids[i - 1];
                    let to_node_id = wait_connect_node_ids[i];
                    if node_cut_subdiv_count.is_some() {
                        self.set_node_cut_subdiv_count(to_node_id, node_cut_subdiv_count.unwrap());
                    }
                    self.add_edge(from_node_id, to_node_id);
                }
            }
        }
    }

    fn generate_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().generated {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().generated = true;
        let user_node_id = {
            let from_id = self.graph.node_weight(node_index).unwrap().generate_from_node_id;
            if from_id.is_none() {
                node_index.index()
            } else {
                from_id.unwrap()
            }
        };
        let node_base_norm = self.graph.node_weight(node_index).unwrap().base_norm;
        let node_position = self.graph.node_weight(node_index).unwrap().position;
        let node_radius = self.graph.node_weight(node_index).unwrap().radius;
        let node_cut_subdiv_count = self.graph.node_weight(node_index).unwrap().cut_subdiv_count;
        let cut_subdiv_count = {
            if node_cut_subdiv_count.is_none() {
                self.cut_subdiv_count
            } else {
                node_cut_subdiv_count.unwrap()
            }
        };
        let mut new_cuts : Vec<(EdgeIndex, (Vec<Id>, Vector3<f32>))> = Vec::new();
        let mut other_node_indices : Vec<NodeIndex> = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            let mut neighbors_count = 0;
            let mut directs = Vec::new();
            let mut rev_directs = Vec::new();
            for other_index in neighbors.clone() {
                let direct = self.direct_of_nodes(node_index, other_index);
                directs.push(direct);
                rev_directs.push(-direct);
                other_node_indices.push(other_index);
                neighbors_count += 1;
            }
            if neighbors_count == 1 {
                let direct = directs[0];
                let face = self.make_cut(node_position - direct * node_radius, direct, node_radius, node_base_norm, cut_subdiv_count);
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
                let face = self.make_cut(node_position - direct * node_radius, direct, node_radius, node_base_norm, cut_subdiv_count);
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
                let max_round : usize = 25;
                let factor_step = 1.0 / max_round as f32;
                const DIRECT_INITIAL_AFFECT_FACTOR : f32 = 0.5;
                for round in 0..max_round {
                    for other_index in neighbors.clone() {
                        let factor = factor_step * round as f32;
                        let mut direct = self.direct_of_nodes(node_index, other_index);
                        let mut ave_direct = direct;
                        for &rev_direct in rev_directs.iter() {
                            ave_direct += rev_direct;
                        }
                        ave_direct = ave_direct / (rev_directs.len() - 1) as f32;
                        let mut direct_affect_factor = DIRECT_INITIAL_AFFECT_FACTOR;
                        let mut create_radius = node_radius;
                        let mut create_origin = node_position;
                        if round > 0 {
                            direct_affect_factor += (1.0 - DIRECT_INITIAL_AFFECT_FACTOR) * factor;
                            create_radius = node_radius * (1.0 - factor);
                        }
                        direct = (ave_direct * direct_affect_factor + direct * (1.0 - direct_affect_factor)).normalize();
                        if round > 0 {
                            create_origin += direct * (node_radius * factor);
                        }
                        let edge_index = self.graph.find_edge(node_index, other_index).unwrap();
                        //println!("round: {:?} other_index:{:?} r:{:?} direct:{:?}", round, other_index, create_radius, direct);
                        let face = self.make_cut(create_origin, direct, create_radius, node_base_norm, cut_subdiv_count);
                        cuts.push((face, edge_index, other_index, direct));
                    }
                    let wrap_ok = {
                        // test wrap
                        let mut added_loops : Vec<(Vec<Id>, Vector3<f32>)> = Vec::new();
                        let mut test_mesh = Mesh::new();
                        for (face, _edge_index, _other_index, direct) in cuts.clone() {
                            let mut vert_ids = Vec::new();
                            for vert in face {
                                vert_ids.push(test_mesh.add_vertex(vert));
                            }
                            let mut rev_vert_ids = vert_ids.clone();
                            rev_vert_ids.reverse();
                            test_mesh.add_vertices(vert_ids);
                            added_loops.push((rev_vert_ids, direct));
                        }
                        if added_loops.len() > 1 {
                            let mut wrapper = GiftWrapper::new();
                            wrapper.wrap_vertices(&mut test_mesh, &added_loops);
                            wrapper.finished() && test_mesh.broken_face_set().is_empty() && test_mesh.triangulate().is_triangulated_mesh_manifold()
                        } else {
                            false
                        }
                    };
                    if wrap_ok {
                        break;
                    }
                    cuts = Vec::new();
                }
                if cuts.len() > 0 {
                    // real wrap
                    let mut added_loops : Vec<(Vec<Id>, Vector3<f32>)> = Vec::new();
                    for (face, edge_index, _other_index, direct) in cuts {
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
                        wrapper.finished();
                    }
                } else {
                    self.wrap_error_count += 1;
                }
            }
        }
        for (edge_index, cut) in new_cuts {
            let mut generated_vertices = Vec::new();
            for &vertex_id in cut.0.iter() {
                self.vertex_node_map.insert(vertex_id, node_index);
                self.vertex_cut_direct_map.insert(vertex_id, cut.1);
                self.mesh.vertex_mut(vertex_id).unwrap().source = user_node_id as i32;
                generated_vertices.push(vertex_id);
            }
            {
                let ref mut edge = self.graph.edge_weight_mut(edge_index).unwrap();
                edge.cuts.push(cut);
            }
            self.graph.node_weight_mut(node_index).unwrap().generated_vertices.push(generated_vertices);
        }
        for other_index in other_node_indices {
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

    pub fn error_count(&self) -> usize {
        self.wrap_error_count as usize
    }

    fn resolve_deform(&mut self) {
        for vert in self.mesh.vertices.iter_mut() {
            let node_index = if self.vertex_node_map.is_empty() {
                NodeIndex::new(self.last_node_id)
            } else {
                self.vertex_node_map[&vert.id]
            };
            let node_base_norm = self.graph.node_weight(node_index).unwrap().base_norm;
            let node_position = self.graph.node_weight(node_index).unwrap().position;
            let vert_ray = vert.position - node_position;
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut sum_z = 0.0;
            let mut num = 0;
            if (self.deform_thickness - 1.0).abs() > SMALL_NUM {
                let thickness_deformed_position = calculate_deform_position(vert.position,
                    vert_ray, node_base_norm, self.deform_thickness);
                sum_x += thickness_deformed_position.x;
                sum_y += thickness_deformed_position.y;
                sum_z += thickness_deformed_position.z;
                num += 1;
            }
            if (self.deform_width - 1.0).abs() > SMALL_NUM {
                if let Some(&cut_direct) = self.vertex_cut_direct_map.get(&vert.id) {
                    let width_deformed_position = calculate_deform_position(vert.position,
                        vert_ray, node_base_norm.cross(cut_direct), self.deform_width);
                    sum_x += width_deformed_position.x;
                    sum_y += width_deformed_position.y;
                    sum_z += width_deformed_position.z;
                    num += 1;
                }
            }
            if num > 0 {
                vert.position.x = sum_x / num as f32;
                vert.position.y = sum_y / num as f32;
                vert.position.z = sum_z / num as f32;
            }
        }
    }

    fn resolve_seam_from_node(&mut self, node_index: NodeIndex, from_vertex_id: Id, seam: &mut Vec<Id>) {
        // Check seam_resolved in caller, don't check it here
        self.graph.node_weight_mut(node_index).unwrap().seam_resolved = true;
        seam.push(from_vertex_id);
        let halfedges = self.mesh.vertex(from_vertex_id).unwrap().halfedges.clone();
        let mut ids = Vec::new();
        for halfedge_id in halfedges {
            let next_halfedge_id = self.mesh.halfedge_next_id(halfedge_id);
            if next_halfedge_id.is_none() {
                continue;
            }
            let vertex_id = self.mesh.halfedge_start_vertex_id(next_halfedge_id.unwrap()).unwrap();
            let next_node_id = self.mesh.vertex_mut(vertex_id).unwrap().source as usize;
            if next_node_id == node_index.index() {
                continue;
            }
            if self.graph.node_weight_mut(NodeIndex::new(next_node_id)).unwrap().seam_resolved {
                continue;
            }
            ids.push(vertex_id);
        }
        let mut positions = Vec::new();
        for &vert_id in ids.iter() {
            positions.push(self.mesh.vertex(vert_id).unwrap().position);
        }
        if positions.is_empty() {
            return;
        }
        let vertex_index = pick_most_not_obvious_vertex(positions);
        let next_from_vertex_id = ids[vertex_index];
        let next_node_index = NodeIndex::new(self.mesh.vertex_mut(next_from_vertex_id).unwrap().source as usize);
        if self.neighbor_count_map[&next_node_index.index()] != 2 {
            return;
        }
        self.resolve_seam_from_node(next_node_index, next_from_vertex_id, seam);
    }

    fn resolve_seam(&mut self) {
        for (k, v) in self.neighbor_count_map.clone() {
            if 2 != v {
                let node_index = NodeIndex::new(k);
                {
                    if self.graph.node_weight(node_index).unwrap().seam_resolved {
                        continue;
                    }
                }
                let generated_vertices = self.graph.node_weight(node_index).unwrap().generated_vertices.clone();
                for vertices in generated_vertices {
                    let mut positions = Vec::new();
                    for &vert_id in vertices.iter() {
                        positions.push(self.mesh.vertex(vert_id).unwrap().position);
                    }
                    if positions.is_empty() {
                        continue;
                    }
                    let vertex_index = pick_most_not_obvious_vertex(positions);
                    let mut seam : Vec<Id> = Vec::new();
                    self.resolve_seam_from_node(node_index, vertices[vertex_index], &mut seam);
                    if seam.len() > 1 {
                        self.seams.push(seam);
                    }
                }
            }
        }
    }

    pub fn generate_mesh(&mut self) -> &mut Mesh {
        if self.node_count > 1 {
            self.reorder_nodes_by_neighbor_count();
            let root_node_id = self.neighbor_count_vec[0].0;
            let root_node = NodeIndex::new(root_node_id);
            self.resolve_round();
            self.resolve_base_norm();
            self.generate_from_node(root_node);
            if self.seam_required {
                self.resolve_seam();
            }
            if 0 == self.wrap_error_count {
                self.stitch_by_edges();
            }
            if (self.deform_thickness - 1.0).abs() > SMALL_NUM ||
                    (self.deform_width - 1.0).abs() > SMALL_NUM {
                self.resolve_deform();
            }
            self.output_debug_info_if_enabled();
        } else {
            let root_node_id = self.last_node_id;
            let root_node = NodeIndex::new(root_node_id);
            let node_position = self.graph.node_weight(root_node).unwrap().position;
            let node_radius = self.graph.node_weight(root_node).unwrap().radius;
            let face_id = self.mesh.add_plane(node_radius, node_radius);
            let normal = self.mesh.face_norm(face_id);
            self.mesh.extrude_face(face_id, normal, node_radius).translate(node_position.x, 
                node_position.y, 
                node_position.z - node_radius * 0.5);
            if self.cut_subdiv_count > 0 {
                let subdived_mesh = self.mesh.subdivide();
                self.mesh = subdived_mesh;
            }
            for vertex in self.mesh.vertices.iter_mut() {
                vertex.source = root_node_id as i32;
            }
            if (self.deform_thickness - 1.0).abs() > SMALL_NUM ||
                    (self.deform_width - 1.0).abs() > SMALL_NUM {
                self.resolve_deform();
            }
        }
        &mut self.mesh
    }
}

impl Node {
    fn new(radius: f32, position: Point3<f32>) -> Self {
        Node {
            radius: radius,
            position: position,
            base_norm: Vector3 {x:0.0, y:0.0, z:1.0},
            generated: false,
            base_norm_resolved: false,
            base_norm_searched: false,
            cut_subdiv_count: None,
            round_way: None,
            generate_from_node_id: None,
            seam_resolved: false,
            generated_vertices: Vec::new(),
            insert_order: 0
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
