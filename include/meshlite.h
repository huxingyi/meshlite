#ifndef MESHLITE_H
#define MESHLITE_H

#ifdef __cplusplus
extern "C" {
#endif

void *meshlite_create_context(void);
int meshlite_import(void *context, const char *filename);
int meshlite_export(void *context, int mesh_id, const char *filename);
int meshlite_clone(void *context, int from_mesh_id);
int meshlite_triangulate(void *context, int mesh_id);
int meshlite_subdivide(void *context, int mesh_id);
int meshlite_union(void *context, int first_mesh_id, int second_mesh_id);
int meshlite_diff(void *context, int first_mesh_id, int second_mesh_id);
int meshlite_intersect(void *context, int first_mesh_id, int second_mesh_id);
int meshlite_scale(void *context, int mesh_id, float value);

#ifdef __cplusplus
}
#endif

#endif
