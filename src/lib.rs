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

use mesh::Mesh;
use mesh::Export;
use mesh::Import;

use wrap::GiftWrapper;
use subdivide::CatmullClarkSubdivider;
use triangulate::Triangulate;
use subdivide::Subdivide;

const MAGIC_NUM: u32 = 12345678;

#[repr(C)]
pub struct RustContext {
    magic: u32,
    meshes: Vec<Mesh>,
    free_mesh_ids: Vec<i32>,
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
    ctx.free_mesh_ids.push(id);
}

#[no_mangle]
pub extern "C" fn meshlite_create_context() -> *mut RustContext {
    Box::into_raw(Box::new(RustContext {
        magic: MAGIC_NUM,
        meshes: Vec::new(),
        free_mesh_ids: Vec::new(),
    }))
}

pub extern "C" fn meshlite_destroy_context(context: *mut RustContext) {
    if context.is_null() { 
        return 
    }
    unsafe { 
        Box::from_raw(context); 
    }
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
    let new_mesh = ctx.meshes.get((first_mesh_id - 1) as usize).unwrap().union_mesh(ctx.meshes.get((second_mesh_id - 1) as usize).unwrap());
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
    let new_mesh = ctx.meshes.get((first_mesh_id - 1) as usize).unwrap().diff_mesh(ctx.meshes.get((second_mesh_id - 1) as usize).unwrap());
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
    let new_mesh = ctx.meshes.get((first_mesh_id - 1) as usize).unwrap().intersect_mesh(ctx.meshes.get((second_mesh_id - 1) as usize).unwrap());
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