extern crate cgmath;
extern crate petgraph;

mod mesh;
mod subdivide;
mod iterator;
mod util;
mod wrap;
mod bmesh;
mod triangulate;

use std::ffi::CStr;
use std::os::raw::{c_int, c_uint, c_char};
use mesh::Mesh;

use std::collections::HashMap;

use subdivide::CatmullClarkSubdivider;

#[repr(C)]
pub struct RustContext {
    magic: u32,
    meshes: Vec<Mesh>,
    free_mesh_ids: Vec<i32>,
}

fn alloc_mesh_id(ctx: &mut RustContext) -> i32 {
    if ctx.free_mesh_ids.len() > 0 {
        ctx.free_mesh_ids.swap_remove(0)
    } else {
        ctx.meshes.push(Mesh::new());
        ctx.meshes.len() as i32
    }
}

fn free_mesh_id(ctx: &mut RustContext, id: i32) {
    ctx.free_mesh_ids.push(id);
}

#[no_mangle]
pub extern "C" fn meshlite_create_context() -> *mut RustContext {
    Box::into_raw(Box::new(RustContext {
        magic: 12345678,
        meshes: Vec::new(),
        free_mesh_ids: Vec::new(),
    }))
}

pub extern "C" fn meshlite_destroy_context(context: *mut RustContext) {
    if context.is_null() { return }
    unsafe { Box::from_raw(context); }
}

#[no_mangle]
pub extern "C" fn meshlite_subdivide_mesh(context: *mut RustContext, mesh_id: c_int, loops: c_int) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, 12345678);
    if loops <= 0 {
        return 0;
    }
    let new_mesh_id = alloc_mesh_id(ctx);
    let mut old_mesh_id = mesh_id;
    for i in 0..loops {
        let new_mesh = {
            let mut mesh = ctx.meshes.get_mut((old_mesh_id - 1) as usize).unwrap();
            let mut cc = CatmullClarkSubdivider::new(&mut mesh);
            cc.generate()
        };
        ctx.meshes[(new_mesh_id - 1) as usize] = new_mesh;
        old_mesh_id = new_mesh_id;
    }
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_load_obj_as_mesh(context: *mut RustContext, filename: *const c_char) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, 12345678);
    let c_str = unsafe {
        assert!(!filename.is_null());
        CStr::from_ptr(filename)
    };
    let new_mesh_id = alloc_mesh_id(ctx);
    if !ctx.meshes.get_mut((new_mesh_id - 1) as usize).unwrap().load_obj(c_str.to_str().unwrap()).is_ok() {
        free_mesh_id(ctx, new_mesh_id);
        return 0;
    }
    new_mesh_id
}

#[no_mangle]
pub extern "C" fn meshlite_save_mesh_as_obj(context: *mut RustContext, mesh_id: c_int, filename: *const c_char) -> c_int {
    let ctx = unsafe {
        assert!(!context.is_null());
        &mut *context
    };
    assert_eq!(ctx.magic, 12345678);
    let c_str = unsafe {
        assert!(!filename.is_null());
        CStr::from_ptr(filename)
    };
    let mesh = ctx.meshes.get((mesh_id - 1) as usize).unwrap();
    if !mesh.save_obj(c_str.to_str().unwrap()).is_ok() {
        return -1;
    }
    0
}

