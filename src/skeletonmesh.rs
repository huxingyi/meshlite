use cgmath::Point3;
use cgmath::prelude::*;
use mesh::Mesh;
use util::*;
use subdivide::Subdivide;
use wrap::GiftWrapper;

#[derive(Clone)]
struct Bone {
    from: Point3<f32>,
    to: Point3<f32>,
}

pub struct SkeletonMesh {
    bones: Vec<Bone>,
    mesh: Mesh,
    end_radius: f32,
    max_radius: f32,
}

impl SkeletonMesh {
    pub fn new() -> Self {
        SkeletonMesh {
            bones: Vec::new(),
            mesh: Mesh::new(),
            end_radius: 0.005,
            max_radius: 0.025
        }
    }

    pub fn set_end_radius(&mut self, radius: f32) {
        self.end_radius = radius;
    }

    pub fn add_bone(&mut self, from: Point3<f32>, to: Point3<f32>) {
        let bone = Bone {from: from, to: to};
        self.bones.push(bone);
    }

    fn add_sphere(&mut self, position: Point3<f32>, radius: f32) {
        let mut mesh = Mesh::new();
        let face_id = mesh.add_plane(radius, radius);
        let normal = mesh.face_norm(face_id);
        mesh.extrude_face(face_id, normal, radius).translate(position.x, 
            position.y, 
            position.z - radius * 0.5);
        self.mesh += mesh.subdivide();
    }

    pub fn generate_mesh(&mut self) -> &mut Mesh {
        let end_radius = self.end_radius;
        for bone in self.bones.clone() {
            self.add_sphere(bone.from, end_radius);
            self.add_sphere(bone.to, end_radius);
            let bone_vector = bone.to - bone.from;
            let norm = bone_vector.normalize();
            let mut big_radius = bone_vector.magnitude() * 0.15;
            if big_radius > self.max_radius {
                big_radius = self.max_radius;
            }
            let big_end_quad = make_quad(bone.from, norm, end_radius, norm);
            let small_end_quad = make_quad(bone.to, -norm, end_radius, -norm);
            let middle_for_big_quad = make_quad((bone.from + bone_vector * 0.2) - (-norm) * big_radius, -norm, big_radius, -norm);
            let middle_for_small_quad = make_quad((bone.from + bone_vector * 0.2) - norm * big_radius, norm, big_radius, norm);
            let big_end_face = self.mesh.add_positions(big_end_quad);
            let small_end_face = self.mesh.add_positions(small_end_quad);
            let middle_for_big_face = self.mesh.add_positions(middle_for_big_quad);
            let middle_for_small_face = self.mesh.add_positions(middle_for_small_quad);
            {
                let mut wrapper = GiftWrapper::new();
                wrapper.stitch_two_faces(&mut self.mesh, big_end_face, middle_for_big_face);
            }
            {
                let mut wrapper = GiftWrapper::new();
                wrapper.stitch_two_faces(&mut self.mesh, small_end_face, middle_for_small_face);
            }
        }
        &mut self.mesh
    }
}
