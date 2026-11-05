"""Blender previews of the pedestrian head in each state (the same rule as
the UE lamp materials: R = stop figure, G = walk figure, B = flash, A =
countdown fraction; the lit lamp's column (u < 0.5 lower/walk, u > 0.5
upper/stop) lit while v < A).

  blender -b --python jp_hokousha_render.py -- <sample_dir> <out_dir>

歩行者用灯器の各状態のプレビュー（UE のランプ材質と同じ規則）。
"""

import os
import sys

import bpy
import numpy as np

argv = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT = argv[0], argv[1]
os.makedirs(OUT, exist_ok=True)
NPZ = os.path.join(SRC, "meshes", "Terrain_JpHokousha_Sample_000.npz")
BASE = {"jph_body": (0.30, 0.31, 0.32), "jph_face": (0.008, 0.008, 0.008), "jph_pole": (0.30, 0.31, 0.32),
        "jph_stop": (0.05, 0.008, 0.006), "jph_walk": (0.006, 0.035, 0.032), "jph_count": (0.02, 0.02, 0.02)}
RED = (1.0, 0.03, 0.015)
CYAN = (0.0, 0.85, 0.72)
STATES = {"stop": (1, 0, 0, 6 / 10), "walk": (0, 1, 0, 7 / 10), "flash_on": (0, 1, 1, 0.0),
          "flash_off": (0, 1, 1, 0.0), "dark": (0, 0, 0, 0.0)}


def emissive(mat, uv, state, flash_off):
    r, g, b, a = state
    if mat == "jph_stop":
        return RED if r else None
    if mat == "jph_walk":
        return CYAN if g and not (b and flash_off) else None
    if mat == "jph_count":
        walk_col = uv[0] < 0.5
        return (CYAN if walk_col else RED) if (g if walk_col else r) and uv[1] < a else None
    return None


def build(state, flash_off):
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o)
    d = np.load(NPZ)
    mats = sorted({k.split("|")[0] for k in d.files})
    for mat in mats:
        V, I, UV = d[mat + "|V"], d[mat + "|I"], d[mat + "|UV"]
        # one object per triangle group with its own emission (count: per segment) / 区画ごと
        groups = {}
        for tri in I:
            key = tuple(np.round(UV[tri[0]], 3)) if mat == "jph_count" else None
            groups.setdefault(key, []).append(tri)
        for key, tris in groups.items():
            em = emissive(mat, key if key else (0, 0), state, flash_off)
            me = bpy.data.meshes.new(mat)
            me.from_pydata([tuple(v) for v in V], [], [tuple(t) for t in tris])
            ob = bpy.data.objects.new(mat, me)
            bpy.context.scene.collection.objects.link(ob)
            m = bpy.data.materials.new(mat)
            m.use_nodes = True
            bs = m.node_tree.nodes["Principled BSDF"]
            bs.inputs["Base Color"].default_value = BASE[mat] + (1,)
            bs.inputs["Roughness"].default_value = 0.4
            if em:
                bs.inputs["Emission Color"].default_value = em + (1,)
                bs.inputs["Emission Strength"].default_value = 2.5
            me.materials.append(m)


def render(name):
    sc = bpy.context.scene
    cam = bpy.data.cameras.new("cam")
    cam.lens = 70
    co = bpy.data.objects.new("cam", cam)
    sc.collection.objects.link(co)
    co.location = (0.4, -3.0, 3.1)
    co.rotation_euler = (1.5708, 0, 0.18)
    sc.camera = co
    li = bpy.data.lights.new("sun", "SUN")
    li.energy = 2.0
    lo = bpy.data.objects.new("sun", li)
    lo.rotation_euler = (0.9, 0.2, 0.6)
    sc.collection.objects.link(lo)
    sc.world = sc.world or bpy.data.worlds.new("w")
    sc.world.use_nodes = True
    sc.world.node_tree.nodes["Background"].inputs[0].default_value = (0.35, 0.45, 0.6, 1)
    sc.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in [e.identifier for e in
                                                                      bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items] else "BLENDER_EEVEE"
    sc.render.resolution_x, sc.render.resolution_y = 420, 640
    sc.render.filepath = os.path.join(OUT, "hokousha_%s.png" % name)
    bpy.ops.render.render(write_still=True)
    print("[jp_hokousha_render] wrote", sc.render.filepath)


for nm, st in STATES.items():
    build(st, nm == "flash_off")
    render(nm)


def export_fbx(path):
    """One mesh object with one slot per material (as the map FBX has), for
    the UE test. / UE 試験用に材質スロット付きの 1 オブジェクトを書き出す"""
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o)
    d = np.load(NPZ)
    mats = sorted({k.split("|")[0] for k in d.files})
    verts, faces, mat_idx, uvs = [], [], [], []
    for k, mat in enumerate(mats):
        V, I, UV = d[mat + "|V"], d[mat + "|I"], d[mat + "|UV"]
        off = len(verts)
        verts += [tuple(v) for v in V]
        faces += [tuple(int(i) + off for i in t) for t in I]
        mat_idx += [k] * len(I)
        uvs += [UV[t].tolist() for t in I]
    me = bpy.data.meshes.new("Terrain_JpHokousha_Sample_000")
    me.from_pydata(verts, [], faces)
    uvl = me.uv_layers.new(name="UVMap")
    for poly, uv3 in zip(me.polygons, uvs):
        poly.material_index = mat_idx[poly.index]
        for li, uv in zip(poly.loop_indices, uv3):
            uvl.data[li].uv = uv
    for mat in mats:
        me.materials.append(bpy.data.materials.new(mat))
    ob = bpy.data.objects.new("Terrain_JpHokousha_Sample_000", me)
    bpy.context.scene.collection.objects.link(ob)
    ob.select_set(True)
    bpy.ops.export_scene.fbx(filepath=path, use_selection=True, object_types={"MESH"}, apply_unit_scale=True,
                             mesh_smooth_type="FACE")
    print("[jp_hokousha_render] wrote", path)


if len(argv) > 2 and argv[2] == "--fbx":
    export_fbx(os.path.join(OUT, "Terrain_JpHokousha_Sample_000.fbx"))
