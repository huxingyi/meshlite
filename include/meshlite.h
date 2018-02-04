#ifndef MESHLITE_H
#define MESHLITE_H

#ifdef __cplusplus
extern "C" {
#endif

void *meshlite_create_context(void);
int meshlite_load_obj_as_mesh(void *context, const char *filename);
int meshlite_save_mesh_as_obj(void *context, int mesh_id, const char *filename);
int meshlite_subdivide_mesh(void *context, int mesh_id, int loops);

#ifdef __cplusplus
}
#endif

#endif
