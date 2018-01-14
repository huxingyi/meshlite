# Changed by: Xingyi HU
# Origin author: ideasman42 (https://blenderartists.org/forum/member.php?6112-ideasman42)
# Origin source: https://blenderartists.org/forum/showthread.php?238284-Import-obj-file-automatically-when-starting-Blender

import bpy, sys
for o in bpy.data.objects:
    if o.type == 'MESH':
        o.select = True
    else:
        o.select = False
bpy.ops.object.delete()
bpy.ops.import_scene.obj(filepath=sys.argv[-1])
bpy.context.user_preferences.view.show_splash = False
