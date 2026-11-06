"""Blender: the glowing plate under a street-lamp head (街灯の発光面).

  blender -b --python osm_city/jp_gaito.py -- <out dir>   -> SM_JpGaitoGlow.fbx, layout.json

A thin 0.66 x 0.26 x 0.012 m box just under props.m_lamp's 0.7 x 0.3 m head,
its top 2 mm below the actor origin (= the head's underside), long side along
+X (the arm). ue_jp_gaito.py puts it on BP_JpGaito's lamp mesh slot, where
CARLA's light component swaps in a glowing material when the lamp is on.
Names avoid "light"/"sign" (guide ch. 24).
"""

import json
import os
import sys

import bpy
import bmesh
from mathutils import Matrix

L, W, T, GAP = 0.66, 0.26, 0.012, 0.002


def build():
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    # UE (x, y, z) -> Blender (x, -y, z); the box is symmetric in y
    bmesh.ops.transform(bm, matrix=Matrix.Translation((0, 0, -GAP - T / 2)) @ Matrix.Diagonal((L, W, T, 1)),
                        verts=bm.verts)
    uv = bm.loops.layers.uv.verify()
    for f in bm.faces:
        for lp in f.loops:
            lp[uv].uv = (lp.vert.co.x / L + 0.5, lp.vert.co.y / W + 0.5)
    me = bpy.data.meshes.new("SM_JpGaitoGlow")
    bm.to_mesh(me)
    bm.free()
    me.materials.append(bpy.data.materials.new("jpg_glow"))
    ob = bpy.data.objects.new("SM_JpGaitoGlow", me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = os.path.abspath(argv[0] if argv else "build/JpGaito")
    os.makedirs(out, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    ob = build()
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    bpy.ops.export_scene.fbx(filepath=os.path.join(out, "SM_JpGaitoGlow.fbx"), use_selection=True,
                             object_types={"MESH"}, apply_unit_scale=True, bake_space_transform=False,
                             mesh_smooth_type="FACE")
    json.dump({"plate_m": [L, W, T], "gap_m": GAP}, open(os.path.join(out, "layout.json"), "w"))
    print("[jp_gaito] wrote SM_JpGaitoGlow.fbx to", out)


main()
