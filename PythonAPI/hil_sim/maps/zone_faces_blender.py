"""k x 1 m panels for ue_zone_faces.py (Blender, -b -P ... -- <out_dir>).
long side on X, centred, normal +Z / ゾーン面パネル（Blender）"""
import os
import sys

import bpy

out = sys.argv[sys.argv.index("--") + 1]
os.makedirs(out, exist_ok=True)
for k in (1, 2, 4, 8, 16, 32):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.mesh.primitive_plane_add(size=1.0)
    ob = bpy.context.active_object
    ob.name = "ZoneFacePanel_%d" % k
    ob.scale = (k, 1, 1)
    bpy.ops.object.transform_apply(scale=True)
    bpy.ops.export_scene.fbx(filepath=os.path.join(out, "ZoneFacePanel_%d.fbx" % k),
                             use_selection=False, mesh_smooth_type="FACE")
