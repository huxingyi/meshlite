use cgmath::Point3;
use cgmath::Vector3;
use cgmath::prelude::*;
use cgmath::Matrix4;
use mesh::Mesh;
use mesh::Id;
use petgraph::prelude::*;
use petgraph::Graph;
use std::collections::HashMap;
use std::collections::HashSet;
use wrap::GiftWrapper;
use triangulate::Triangulate;
use util::*;
use iterator::FaceIterator;
use iterator::FaceHalfedgeIterator;
use debug::Debug;
use subdivide::CatmullClarkSubdivider;
use subdivide::Subdivide;

struct Node {
    radius: f32,
    position: Point3<f32>,
    base_norm: Vector3<f32>,
    generated: bool,
    triangle_ring_resolved: bool,
    quad_ring_resolved: bool,
    base_norm_resolved: bool,
}

struct Edge {
    cuts: Vec<(Vec<Id>, Vector3<f32>)>,
}

struct Ring {
    pub key: Vec<NodeIndex>,
    pub nodes: Vec<NodeIndex>
}

impl Ring {
    pub fn new(first: NodeIndex, second: NodeIndex, third: NodeIndex, fourth: NodeIndex) -> Self {
        let mut nodes = Vec::new();
        nodes.push(first);
        nodes.push(second);
        nodes.push(third);
        if fourth != NodeIndex::end() {
            nodes.push(fourth);
        }
        let mut key = nodes.clone();
        key.sort();
        Ring {
            key: key,
            nodes: nodes,
        }
    }
}

pub struct Bmesh {
    graph : Graph<Node, Edge, Undirected>,
    mesh: Mesh,
    neighbor_count_map: HashMap<usize, usize>,
    neighbor_count_vec: Vec<(usize, usize)>,
    resolve_ring_map: HashSet<Vec<NodeIndex>>,
    resolve_ring_list: Vec<Vec<NodeIndex>>,
    wrap_error_count: i32,
    node_count: usize,
    debug_enabled: bool,
    generate_from_node_id: usize,
    cut_subdiv_count: usize,
    deform_thickness: f32,
    deform_width: f32,
    vertex_node_map: HashMap<Id, NodeIndex>,
}

impl Bmesh {
    pub fn new() -> Self {
        Bmesh {
            graph: Graph::new_undirected(),
            mesh: Mesh::new(),
            neighbor_count_map: HashMap::new(),
            neighbor_count_vec: Vec::new(),
            resolve_ring_map: HashSet::new(),
            resolve_ring_list: Vec::new(),
            wrap_error_count: 0,
            node_count: 0,
            debug_enabled: false,
            generate_from_node_id: 0,
            cut_subdiv_count: 0,
            deform_thickness: 1.0,
            deform_width: 1.0,
            vertex_node_map: HashMap::new(),
        }
    }

    pub fn set_cut_subdiv_count(&mut self, count: usize) {
        self.cut_subdiv_count = count;
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

    pub fn get_node_base_norm(&self, node_id: usize) -> Vector3<f32> {
        self.graph.node_weight(NodeIndex::new(node_id)).unwrap().base_norm
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

    fn resolve_base_norm_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().base_norm_resolved {
            return;
        }
        let base_norm = self.calculate_node_base_norm(node_index);
        if base_norm.is_none() {
            const WORLD_Y_AXIS : Vector3<f32> = Vector3 {x: 0.0, y: 1.0, z: 0.0};
            self.resolve_base_norm_for_leaves_from_node(node_index, WORLD_Y_AXIS);
            return;
        }
        self.resolve_base_norm_for_leaves_from_node(node_index, base_norm.unwrap());
    }

    fn resolve_base_norm(&mut self) {
        let mut vec : Vec<(usize, usize)> = Vec::new();
        for (&k, &v) in self.neighbor_count_map.iter() {
            vec.push((k, v));
        }
        vec.sort_by(|a, b| b.1.cmp(&a.1));
        for &(node_id, neighbor_count) in vec.iter() {
            println!("resolve_base_norm node_id:{:?} neighbor_count:{:?}", node_id, neighbor_count);
            self.resolve_base_norm_from_node(NodeIndex::new(node_id));
        }
        self.neighbor_count_vec = vec;
    }

    fn output_debug_info_if_enabled(&mut self) {
        if self.debug_enabled {
            for &(node_id, _) in self.neighbor_count_vec.iter() {
                let node_index = NodeIndex::new(node_id);
                let node_origin = self.graph.node_weight(node_index).unwrap().position;
                let base_norm = self.graph.node_weight(node_index).unwrap().base_norm;
                self.mesh.add_debug_norm(node_origin, base_norm);
            }
        }
    }

    pub fn add_node(&mut self, position: Point3<f32>, radius: f32) -> usize {
        let node = Node::new(radius, position);
        let node_id = self.graph.add_node(node).index();
        self.node_count += 1;
        self.generate_from_node_id = node_id;
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

    fn make_cut(&self, position: Point3<f32>, direct: Vector3<f32>, radius: f32, base_norm: Vector3<f32>) -> Vec<Point3<f32>> {
        let mut cut : Vec<Point3<f32>> = make_quad(position, direct, radius, base_norm);
        let origin = position + direct * radius;
        for _ in 0..self.cut_subdiv_count {
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

    fn resolve_triangle_ring_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().triangle_ring_resolved {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().triangle_ring_resolved = true;
        let mut other_node_indices : Vec<NodeIndex> = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            for other_index in neighbors.clone() {
                other_node_indices.push(other_index);
            }
        }
        for i in 0..other_node_indices.len() {
            let first_index = other_node_indices[i];
            for j in i + 1..other_node_indices.len() {
                let second_index = other_node_indices[j];
                    if !self.graph.find_edge_undirected(first_index, second_index).is_none() {
                        // found triangle: first_index, node_index, second_index
                        let ring = Ring::new(first_index, node_index, second_index, NodeIndex::end());
                        if !self.resolve_ring_map.contains(&ring.key) {
                            self.resolve_ring_map.insert(ring.key);
                            self.resolve_ring_list.push(ring.nodes);
                        }
                    }
            }
        }
        for other_index in other_node_indices {
            self.resolve_triangle_ring_from_node(other_index);
        }
    }

    fn resolve_quad_ring_from_node(&mut self, node_index: NodeIndex) {
        if self.graph.node_weight(node_index).unwrap().quad_ring_resolved {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().quad_ring_resolved = true;
        let mut other_node_indices : Vec<NodeIndex> = Vec::new();
        {
            let neighbors = self.graph.neighbors_undirected(node_index);
            for other_index in neighbors.clone() {
                other_node_indices.push(other_index);
            }
        }
        for i in 0..other_node_indices.len() {
            let first_index = other_node_indices[i];
            for j in i + 1..other_node_indices.len() {
                let second_index = other_node_indices[j];
                    if self.graph.find_edge_undirected(first_index, second_index).is_none() {
                        let mut found_index = NodeIndex::end();
                        for third_index in self.graph.neighbors_undirected(first_index) {
                            if third_index != node_index && !self.graph.find_edge_undirected(third_index, second_index).is_none() {
                                found_index = third_index;
                                break;
                            }
                        }
                        if found_index != NodeIndex::end() {
                            // found quad: first_index, node_index, second_index, found_index
                            let ring = Ring::new(first_index, node_index, second_index, found_index);
                            if !self.resolve_ring_map.contains(&ring.key) &&
                                    !self.resolve_ring_map.contains(&Ring::new(first_index, node_index, second_index, NodeIndex::end()).key) &&
                                    !self.resolve_ring_map.contains(&Ring::new(node_index, second_index, found_index, NodeIndex::end()).key) &&
                                    !self.resolve_ring_map.contains(&Ring::new(second_index, found_index, first_index, NodeIndex::end()).key) &&
                                    !self.resolve_ring_map.contains(&Ring::new(found_index, first_index, node_index, NodeIndex::end()).key) {
                                self.resolve_ring_map.insert(ring.key);
                                self.resolve_ring_list.push(ring.nodes);
                            }
                        }
                    }
            }
        }
        for other_index in other_node_indices {
            self.resolve_quad_ring_from_node(other_index);
        }
    }

    fn get_ring_center(&self, ring: &Vec<NodeIndex>) -> Point3<f32> {
        let mut positions = Vec::new();
        for &node_index in ring {
            positions.push(self.graph.node_weight(node_index).unwrap().position);
        }
        Point3::centroid(&positions)
    }

    fn do_resolve_ring(&mut self, ring: Vec<NodeIndex>) {
        let mut shared_indices = HashSet::new();
        let center = self.get_ring_center(&ring);
        for i in 0..ring.len() {
            let first_node = ring[i];
            let second_node = ring[(i + 1) % ring.len()];
            let edge_index = self.graph.find_edge_undirected(first_node, second_node).unwrap().0;
            let ref edge = self.graph.edge_weight(edge_index).unwrap();
            for cut in &edge.cuts {
                for &vert_id in cut.0.iter() {
                    shared_indices.insert(vert_id);
                }
            }
        }
        let mut remove_face_id_list : Vec<Id> = Vec::new();
        for face_id in FaceIterator::new(&self.mesh) {
            let mut need_remove = true;
            let mut any_point_on_plane = Point3{x:0.0, y:0.0, z:0.0};
            for halfedge_id in FaceHalfedgeIterator::new(&self.mesh, self.mesh.face_first_halfedge_id(face_id).unwrap()) {
                let vert_id = self.mesh.halfedge_start_vertex_id(halfedge_id).unwrap();
                if !shared_indices.contains(&vert_id) {
                    need_remove = false;
                    break;
                }
                any_point_on_plane = self.mesh.vertex(vert_id).unwrap().position;
            }
            if need_remove {
                need_remove = PointSide::Front == point_side_on_plane(center, any_point_on_plane, self.mesh.face_norm(face_id));
            }
            if need_remove {
                remove_face_id_list.push(face_id);
            }
        }
        for face_id in remove_face_id_list {
            self.mesh.remove_face(face_id);
        }
        let mut linked_vertices : HashMap<Id, Id> = HashMap::new();
        for face_id in FaceIterator::new(&self.mesh) {
            for halfedge_id in FaceHalfedgeIterator::new(&self.mesh, self.mesh.face_first_halfedge_id(face_id).unwrap()) {
                if self.mesh.halfedge_opposite_face_id(halfedge_id).is_some() {
                    continue;
                }
                let vert_id = self.mesh.halfedge_start_vertex_id(halfedge_id).unwrap();
                let next_vert_id = self.mesh.halfedge_start_vertex_id(self.mesh.halfedge_next_id(halfedge_id).unwrap()).unwrap();
                if shared_indices.contains(&vert_id) && shared_indices.contains(&next_vert_id) {
                    linked_vertices.entry(next_vert_id).or_insert(vert_id);
                    println!("link {:?} -> {:?}", next_vert_id, vert_id);
                }
            }
        }
        while self.mesh.add_linked_vertices(&mut linked_vertices) > 0 {};
    }

    fn resolve_ring_from_node(&mut self, node_index: NodeIndex) {
        {
            self.resolve_triangle_ring_from_node(node_index);
            self.resolve_quad_ring_from_node(node_index);
        }
        for ring in self.resolve_ring_list.clone() {
            self.do_resolve_ring(ring);
        }
    }

    fn generate_from_node(&mut self, node_index: NodeIndex) {
        println!("generate_from_node:{:?}", node_index);
        if self.graph.node_weight(node_index).unwrap().generated {
            return;
        }
        self.graph.node_weight_mut(node_index).unwrap().generated = true;
        let node_base_norm = self.graph.node_weight(node_index).unwrap().base_norm;
        let node_position = self.graph.node_weight(node_index).unwrap().position;
        let node_radius = self.graph.node_weight(node_index).unwrap().radius;
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
                let face = self.make_cut(node_position - direct * node_radius, direct, node_radius, node_base_norm);
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
                let face = self.make_cut(node_position - direct * node_radius, direct, node_radius, node_base_norm);
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
                        println!("round: {:?} other_index:{:?} r:{:?} direct:{:?}", round, other_index, create_radius, direct);
                        let face = self.make_cut(create_origin, direct, create_radius, node_base_norm);
                        cuts.push((face, edge_index, other_index, direct));
                    }
                    let wrap_ok = {
                        // test wrap
                        let mut added_loops : Vec<(Vec<Id>, Vector3<f32>)> = Vec::new();
                        let mut test_mesh = Mesh::new();
                        for (face, edge_index, other_index, direct) in cuts.clone() {
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
                        wrapper.finished();
                    }
                } else {
                    self.wrap_error_count += 1;
                }
            }
        }
        for (edge_index, cut) in new_cuts {
            for &vertex_id in cut.0.iter() {
                self.vertex_node_map.insert(vertex_id, node_index);
            }
            let ref mut edge = self.graph.edge_weight_mut(edge_index).unwrap();
            edge.cuts.push(cut);
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
            let node_index = self.vertex_node_map[&vert.id];
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
            if (self.deform_thickness - 1.0).abs() > SMALL_NUM {
                let width_deformed_position = calculate_deform_position(vert.position,
                    vert_ray, world_perp(node_base_norm), self.deform_width);
                sum_x += width_deformed_position.x;
                sum_y += width_deformed_position.y;
                sum_z += width_deformed_position.z;
                num += 1;
            }
            if num > 0 {
                vert.position.x = sum_x / num as f32;
                vert.position.y = sum_y / num as f32;
                vert.position.z = sum_z / num as f32;
            }
        }
    }

    pub fn generate_mesh(&mut self) -> &mut Mesh {
        let root_node = NodeIndex::new(self.generate_from_node_id);
        if self.node_count > 1 {
            self.resolve_base_norm();
            self.generate_from_node(root_node);
            if 0 == self.wrap_error_count {
                self.stitch_by_edges();
                self.resolve_ring_from_node(root_node);
            }
            if (self.deform_thickness - 1.0).abs() > SMALL_NUM ||
                    (self.deform_thickness - 1.0).abs() > SMALL_NUM {
                self.resolve_deform();
            }
            self.output_debug_info_if_enabled();
        } else {
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
        }
        &mut self.mesh
    }
}

impl Node {
    fn new(radius: f32, position: Point3<f32>) -> Self {
        Node {
            radius: radius,
            position: position,
            base_norm: Vector3::zero(),
            generated: false,
            triangle_ring_resolved: false,
            quad_ring_resolved: false,
            base_norm_resolved: false,
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
