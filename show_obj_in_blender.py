# Changed by: Xingyi HU
# Original author: ideasman42 (https://blenderartists.org/forum/member.php?6112-ideasman42)
# Original source: https://blenderartists.org/forum/showthread.php?238284-Import-obj-file-automatically-when-starting-Blender

import bpy, sys

def select_mesh():
    for o in bpy.data.objects:
        if o.type == 'MESH':
            o.select = True
            bpy.context.scene.objects.active = o
        else:
            o.select = False

def make_transparent():
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            area.spaces.active.use_occlude_geometry = False

bpy.context.user_preferences.view.show_splash = False
select_mesh()
bpy.ops.object.delete()
bpy.ops.import_scene.obj(filepath=sys.argv[-1])
select_mesh()
bpy.ops.object.mode_set(mode='EDIT')
make_transparent()
bpy.context.scene.tool_settings.normal_size = 0.4
bpy.context.object.data.show_normal_face = True