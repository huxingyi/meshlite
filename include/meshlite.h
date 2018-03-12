#ifndef MESHLITE_H
#define MESHLITE_H

#ifdef __cplusplus
extern "C" {
#endif

void *meshlite_create_context(void);
int meshlite_destroy_context(void *context);
int meshlite_import(void *context, const char *filename);
int meshlite_export(void *context, int mesh_id, const char *filename);
int meshlite_clone(void *context, int from_mesh_id);
int meshlite_triangulate(void *context, int mesh_id);
int meshlite_subdivide(void *context, int mesh_id);
int meshlite_union(void *context, int first_mesh_id, int second_mesh_id);
int meshlite_diff(void *context, int first_mesh_id, int second_mesh_id);
int meshlite_intersect(void *context, int first_mesh_id, int second_mesh_id);
int meshlite_scale(void *context, int mesh_id, float value);
int meshlite_get_vertex_count(void *context, int mesh_id);
int meshlite_get_vertex_position_array(void *context, int mesh_id, float *buffer, int max_buffer_len);
int meshlite_get_face_count(void *context, int mesh_id);
int meshlite_get_triangle_index_array(void *context, int mesh_id, int *buffer, int max_buffer_len);
int meshlite_get_triangle_normal_array(void *context, int mesh_id, float *buffer, int max_buffer_len);
int meshlite_get_edge_count(void *context, int mesh_id);
int meshlite_get_edge_index_array(void *context, int mesh_id, int *buffer, int max_buffer_len);
int meshlite_bmesh_create(void *context);
int meshlite_bmesh_add_node(void *context, int bmesh_id, float x, float y, float z, float radius);
int meshlite_bmesh_add_edge(void *context, int bmesh_id, int first_node_id, int second_node_id);
int meshlite_bmesh_generate_mesh(void *context, int bmesh_id, int root_node_id);
int meshlite_bmesh_destroy(void *context, int bmesh_id);

#ifdef __cplusplus
}
#endif

#endif
