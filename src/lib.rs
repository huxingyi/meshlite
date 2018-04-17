extern crate cgmath;
extern crate petgraph;

use std::ffi::CStr;
use std::os::raw::{c_int, c_uint, c_char, c_float};

mod mesh;
mod subdivide;
mod iterator;
mod util;
mod wrap;
mod bmesh;
mod triangulate;
mod wavefront;
mod debug;

use cgmath::Point3;

use mesh::Mesh;
use bmesh::Bmesh;
use mesh::Export;
use mesh::Import;
use mesh::Id;

use wrap::GiftWrapper;
use subdivide::CatmullClarkSubdivider;
use triangulate::Triangulate;
use subdivide::Subdivide;
use iterator::FaceHalfedgeIterator;
use debug::Debug;

use std::cmp;

const MAGIC_NUM: u32 = 12345678;

#[repr(C)]
pub struct RustContext {
    magic: u32,
    meshes: Vec<Mesh>,
    free_mesh_ids: Vec<i32>,
    bmeshes: Vec<Bmesh>,
    free_bmesh_ids: Vec<i32>,
}

fn alloc_mesh_id(ctx: &mut RustContext) -> i32 {
    let mut id = 0;
    if ctx.free_mesh_ids.len() > 0 {
        id = ctx.free_mesh_ids[0];
        ctx.free_mesh_ids.swap_remove(0);
    } else {
        ctx.meshes.push(Mesh::new());
        id = ctx.meshes.len() as i32;
    }
    id
}

fn free_mesh_id(ctx: &mut RustContext, id: i32) {
    ctx.meshes[id as usize - 1] = Mesh::new();
    ctx.free_mesh_ids.push(id);
}

fn alloc_bmesh_id(ctx: &mut RustContext) -> i32 {
    let mut id = 0;
    if ctx.free_bmesh_ids.len() > 0 {
        id = ctx.free_bmesh_ids[0];
        ctx.free_bmesh_ids.swap_remove(0);
    } else {
        ctx.bmeshes.push(Bmesh::new());
        id = ctx.bmeshes.len() as i32;
    }
    id
}

fn free_bmesh_id(ctx: &mut RustContext, id: i32) {
    ctx.bmeshes[id as usize - 1] = Bmesh::new();
    ctx.free_bmesh_ids.push(id);
}

#[no_mangle]
pub extern "C" fn meshlite_create_context() -> *mut RustContext {
    Box::into_raw(Box::new(RustContext {
        magic: MAGIC_NUM,
        meshes: Vec::new(),
        free_mesh_ids: Vec::new(),
        bmeshes: Vec::new(),
        free_bmesh_ids: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn meshlite_destroy_context(context: *mut RustContext) -> c_int {
    if context.is_null() { 
        return 0;
    }
    unsafe { 
        Box::from_raw(context); 
    }
    0
}

#[no_mangle]
pub extern "C" fn meshlite_import(context: *mut RustContext, filename: *const c_char) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let c_str = unsafe {
        assert!(!filename.is_null());
        CStr::from_ptr(filename)
    };
    let new_mesh_id = alloc_mesh_id(ctx);
    if !ctx.meshes.get_mut((new_mesh_id - 1) as usize).unwrap().import(c_str.to_str().unwrap()).is_ok() {
        free_mesh_id(ctx, new_mesh_id);
        return 0;
    }
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_export(context: *mut RustContext, mesh_id: c_int, filename: *const c_char) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let c_str = unsafe {
        assert!(!filename.is_null());
        CStr::from_ptr(filename)
    };
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    if !mesh.export(c_str.to_str().unwrap()).is_ok() {
        return -1;
    }
    0
}

#[no_mangle]
pub extern "C" fn meshlite_merge(context: *mut RustContext, first_mesh_id: c_int, second_mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let mut new_mesh = Mesh::new();
    new_mesh.add_mesh(ctx.meshes.get((first_mesh_id - 1) as usize).unwrap());
    new_mesh.add_mesh(ctx.meshes.get((second_mesh_id - 1) as usize).unwrap());
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_clone(context: *mut RustContext, from_mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh = ctx.meshes.get((from_mesh_id - 1) as usize).unwrap().clone();
    let new_mesh_id = alloc_mesh_id(ctx);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_triangulate(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().triangulate();
    let new_mesh_id = alloc_mesh_id(ctx);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_is_triangulated_manifold(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let is_manifold = ctx.meshes.get((mesh_id - 1) as usize).unwrap().is_triangulated_mesh_manifold();
    is_manifold as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_subdivide(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().subdivide();
    let new_mesh_id = alloc_mesh_id(ctx);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_union(context: *mut RustContext, first_mesh_id: c_int, second_mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh = ctx.meshes.get((first_mesh_id - 1) as usize).unwrap().union_convex_mesh(ctx.meshes.get((second_mesh_id - 1) as usize).unwrap());
    let new_mesh_id = alloc_mesh_id(ctx);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_diff(context: *mut RustContext, first_mesh_id: c_int, second_mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh = ctx.meshes.get((first_mesh_id - 1) as usize).unwrap().diff_convex_mesh(ctx.meshes.get((second_mesh_id - 1) as usize).unwrap());
    let new_mesh_id = alloc_mesh_id(ctx);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_intersect(context: *mut RustContext, first_mesh_id: c_int, second_mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh = ctx.meshes.get((first_mesh_id - 1) as usize).unwrap().intersect_convex_mesh(ctx.meshes.get((second_mesh_id - 1) as usize).unwrap());
    let new_mesh_id = alloc_mesh_id(ctx);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_scale(context: *mut RustContext, mesh_id: c_int, value: c_float) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    ctx.meshes.get_mut((mesh_id - 1) as usize).unwrap().scale(value);
    0
}

#[no_mangle]
pub extern "C" fn meshlite_get_vertex_count(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    ctx.meshes.get((mesh_id - 1) as usize).unwrap().vertices.len() as i32
}

#[no_mangle]
pub extern "C" fn meshlite_get_face_count(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    ctx.meshes.get((mesh_id - 1) as usize).unwrap().face_count as i32
}

#[no_mangle]
pub extern "C" fn meshlite_get_halfedge_count(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    ctx.meshes.get((mesh_id - 1) as usize).unwrap().halfedge_count as i32
}

#[no_mangle]
pub extern "C" fn meshlite_get_edge_count(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    ctx.meshes.get((mesh_id - 1) as usize).unwrap().edges.len() as i32
}

#[no_mangle]
pub extern "C" fn meshlite_get_vertex_position_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_float, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = cmp::min((mesh.vertices.len() * 3) as usize, max_buffer_len as usize) as isize;
    let mut i : isize = 0;
    for vert_idx in 0..mesh.vertices.len() {
        let position = mesh.vertices[vert_idx].position;
        if i + 3 > count {
            break;
        }
        unsafe {
            *buffer.offset(i + 0) = position.x;
            *buffer.offset(i + 1) = position.y;
            *buffer.offset(i + 2) = position.z;
        }
        i += 3;
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_get_halfedge_index_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_int, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = cmp::min((mesh.halfedge_count * 2) as usize, max_buffer_len as usize) as isize;
    let mut i : isize = 0;
    for face in mesh.faces.iter() {
        if !face.alive {
            continue;
        }
        let mut vert_ids = Vec::new();
        for halfedge_id in FaceHalfedgeIterator::new(mesh, mesh.face_first_halfedge_id(face.id).unwrap()) {
            vert_ids.push(mesh.halfedge_start_vertex_id(halfedge_id).unwrap());
        }
        if i + vert_ids.len() as isize * 2 > count {
            break;
        }
        for j in 0..vert_ids.len() {
            let cur_id = vert_ids[j] as c_int;
            let next_id = vert_ids[(j + 1) % vert_ids.len()] as c_int;
            unsafe {
                *buffer.offset(i + 0) = cur_id - 1;
                *buffer.offset(i + 1) = next_id - 1;
            }
            i += 2;
        }
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_get_halfedge_normal_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_float, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = cmp::min((mesh.halfedge_count * 3) as usize, max_buffer_len as usize) as isize;
    let mut i : isize = 0;
    for face in mesh.faces.iter() {
        if !face.alive {
            continue;
        }
        let norm = mesh.face_norm(face.id);
        let face_halfedge_count = FaceHalfedgeIterator::new(mesh, mesh.face_first_halfedge_id(face.id).unwrap()).into_vec().len();
        if i + face_halfedge_count as isize * 3 > count {
            break;
        }
        for j in 0..face_halfedge_count {
            unsafe {
                *buffer.offset(i + 0) = norm.x;
                *buffer.offset(i + 1) = norm.y;
                *buffer.offset(i + 2) = norm.z;
            }
            i += 3;
        }
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_get_triangle_index_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_int, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = cmp::min((mesh.face_count * 3) as usize, max_buffer_len as usize) as isize;
    let mut i : isize = 0;
    for face in mesh.faces.iter() {
        if !face.alive {
            continue;
        }
        if i + 3 > count {
            break;
        }
        let mut j = 0;
        for halfedge_id in FaceHalfedgeIterator::new(mesh, mesh.face_first_halfedge_id(face.id).unwrap()) {
            if j >= 3 {
                break;
            }
            let vert_id = mesh.halfedge_start_vertex_id(halfedge_id).unwrap() as c_int;
            unsafe {
                *buffer.offset(i + j) = vert_id - 1;
            }
            j += 1;
        }
        i += j;
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_get_face_index_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_int, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = max_buffer_len as isize;
    let mut i : isize = 0;
    for face in mesh.faces.iter() {
        if !face.alive {
            continue;
        }
        let mut vert_ids = Vec::new();
        for halfedge_id in FaceHalfedgeIterator::new(mesh, mesh.face_first_halfedge_id(face.id).unwrap()) {
            let vert_id = mesh.halfedge_start_vertex_id(halfedge_id).unwrap() as c_int;
            vert_ids.push(vert_id);
        }
        if vert_ids.len() == 0 {
            continue;
        }
        if i + 1 + vert_ids.len() as isize > count {
            break;
        }
        unsafe {
            *buffer.offset(i) = vert_ids.len() as i32;
        }
        for j in 0..vert_ids.len() as isize {
            unsafe {
                *buffer.offset(i + 1 + j) = vert_ids[j as usize] - 1;
            }
        }
        i += 1 + vert_ids.len() as isize;
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_get_triangle_normal_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_float, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = cmp::min((mesh.face_count * 3) as usize, max_buffer_len as usize) as isize;
    let mut i : isize = 0;
    for face in mesh.faces.iter() {
        if !face.alive {
            continue;
        }
        if i + 3 > count {
            break;
        }
        let norm = mesh.face_norm(face.id);
        unsafe {
            *buffer.offset(i + 0) = norm.x;
            *buffer.offset(i + 1) = norm.y;
            *buffer.offset(i + 2) = norm.z;
        }
        i += 3;
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_get_edge_index_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_int, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = cmp::min((mesh.edges.len() * 2) as usize, max_buffer_len as usize) as isize;
    let mut i : isize = 0;
    for (edge, _) in mesh.edges.iter() {
        if i + 2 > count {
            break;
        }
        unsafe {
            *buffer.offset(i + 0) = edge.low as c_int - 1;
            *buffer.offset(i + 1) = edge.high as c_int - 1;
        }
        i += 2;
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_get_edge_normal_array(context: *mut RustContext, mesh_id: c_int, buffer: *mut c_float, max_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    let count : isize = cmp::min((mesh.edges.len() * 3) as usize, max_buffer_len as usize) as isize;
    let mut i : isize = 0;
    for (_, &halfedge_id) in mesh.edges.iter() {
        if i + 3 > count {
            break;
        }
        let face_id = mesh.halfedge_face_id(halfedge_id).unwrap();
        let norm = mesh.face_norm(face_id);
        unsafe {
            *buffer.offset(i + 0) = norm.x;
            *buffer.offset(i + 1) = norm.y;
            *buffer.offset(i + 2) = norm.z;
        }
        i += 3;
    }
    i as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_build(context: *mut RustContext, vertex_position_buffer: *mut c_float, vertex_count: c_int, face_index_buffer: *mut c_int, face_index_buffer_len: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let mesh = ctx.meshes.get_mut((new_mesh_id - 1) as usize).unwrap();
    let mut offset = 0;
    let mut vertex_ids = Vec::new();
    for i in 0..vertex_count {
        vertex_ids.push(mesh.add_vertex(unsafe {
            Point3 {
                x: *vertex_position_buffer.offset(offset + 0),
                y: *vertex_position_buffer.offset(offset + 1),
                z: *vertex_position_buffer.offset(offset + 2),
            }
        }));
        offset += 3;
    }
    offset = 0;
    while offset < face_index_buffer_len as isize {
        let index_count = unsafe {
            *face_index_buffer.offset(offset)
        };
        offset += 1;
        let mut added_vertices = Vec::new();
        for i in 0..index_count {
            let index = unsafe {
                *face_index_buffer.offset(offset)
            };
            added_vertices.push(vertex_ids[index as usize] as Id);
            offset += 1;
        }
        mesh.add_vertices(added_vertices);
    }
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_create(context: *mut RustContext) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_bmesh_id = alloc_bmesh_id(ctx);
    new_bmesh_id as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_add_node(context: *mut RustContext, bmesh_id: c_int, x: c_float, y: c_float, z: c_float, radius: c_float) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    bmesh.add_node(Point3 {x: x, y: y, z: z}, radius) as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_add_edge(context: *mut RustContext, bmesh_id: c_int, first_node_id: c_int, second_node_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    println!("rust bmesh_id:{:?} first_node_id:{:?} second_node_id:{:?}", bmesh_id, first_node_id, second_node_id);
    bmesh.add_edge(first_node_id as usize, second_node_id as usize) as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_generate_mesh(context: *mut RustContext, bmesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    println!("rust bmesh_id:{:?}", bmesh_id);
    let new_mesh_id = alloc_mesh_id(ctx);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    let mesh = bmesh.generate_mesh();
    ctx.meshes.insert((new_mesh_id - 1) as usize, mesh.clone());
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_destroy(context: *mut RustContext, bmesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    free_bmesh_id(ctx, bmesh_id);
    0
}

#[no_mangle]
pub extern "C" fn meshlite_combine_adj_faces(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().combine_adj_faces();
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_combine_coplanar_faces(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().combine_coplanar_faces();
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_trim(context: *mut RustContext, mesh_id: c_int, normalize: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().trim(0 != normalize);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_get_node_base_norm(context: *mut RustContext, bmesh_id: c_int, node_id: c_int, norm_buffer: *mut c_float) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    let base_norm = bmesh.get_node_base_norm(node_id as usize);
    unsafe {
        *norm_buffer.offset(0) = base_norm.x;
        *norm_buffer.offset(1) = base_norm.y;
        *norm_buffer.offset(2) = base_norm.z;
    }
    0
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_enable_debug(context: *mut RustContext, bmesh_id: c_int, enable: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    bmesh.enable_debug(enable != 0);
    0
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_set_cut_subdiv_count(context: *mut RustContext, bmesh_id: c_int, subdiv_count: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    bmesh.set_cut_subdiv_count(subdiv_count as usize);
    0
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_set_deform_thickness(context: *mut RustContext, bmesh_id: c_int, thickness: c_float) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    bmesh.set_deform_thickness(thickness);
    0
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_set_deform_width(context: *mut RustContext, bmesh_id: c_int, width: c_float) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    bmesh.set_deform_width(width);
    0
}

#[no_mangle]
pub extern "C" fn meshlite_bmesh_error_count(context: *mut RustContext, bmesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let bmesh = ctx.bmeshes.get_mut((bmesh_id - 1) as usize).unwrap();
    bmesh.error_count() as c_int
}

#[no_mangle]
pub extern "C" fn meshlite_mirror_in_x(context: *mut RustContext, mesh_id: c_int, center_x: c_float) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().mirror_in_x(center_x);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_mirror_in_z(context: *mut RustContext, mesh_id: c_int, center_z: c_float) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().mirror_in_z(center_z);
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_fix_hole(context: *mut RustContext, mesh_id: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, MAGIC_NUM);
    let new_mesh_id = alloc_mesh_id(ctx);
    let new_mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap().fix_hole();
    ctx.meshes.insert((new_mesh_id - 1) as usize, new_mesh);
    new_mesh_id
}
