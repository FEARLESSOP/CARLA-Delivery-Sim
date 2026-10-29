"""Turn a downloaded circuit model into a CARLA map mesh (Blender).

  blender -b -P prepare_map.py -- <maps.json> <map key> <out_dir>

From the map's entry in maps.json: drops the materials listed under "drop"
(sky backdrop), moves the scene by the track's offset (the same shift the
road logic uses, so mesh and lanes line up), splits the mesh by material
and names each piece so CARLA's importer tags it for semantic segmentation
(Road_Road_*, Road_Marking_*, Road_Grass_*, everything else Terrain), writes
the textures packed in the model to PNG and exports <Name>.fbx.

ダウンロードしたサーキットのモデルを CARLA 用マップメッシュにする（Blender）。
"""

import json
import os
import sys

import bpy
import bmesh
from mathutils import Matrix


def main():
    cfg_path, key, out_dir = sys.argv[sys.argv.index("--") + 1:][:3]
    cfg = json.load(open(cfg_path))[key]
    name = cfg["name"]
    track = json.load(open(os.path.join(out_dir, "track.json")))
    ox, oy, oz = track["offset"]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=os.path.expanduser(cfg["source"]))
    objs = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    for o in objs:
        o.data.transform(Matrix.Translation((ox, oy, oz)) @ o.matrix_world)
        o.parent = None
        o.matrix_world = Matrix.Identity(4)
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    mesh = bpy.context.view_layer.objects.active
    # drop the sky backdrop and degenerate faces / 背景の空や面積0の縮退面を削除
    drop = set(cfg.get("drop", []))
    bm = bmesh.new()
    bm.from_mesh(mesh.data)
    names = [m.name if m else "" for m in mesh.data.materials]
    dead = [f for f in bm.faces if names[f.material_index] in drop or f.calc_area() < 1e-6]
    bmesh.ops.delete(bm, geom=dead, context="FACES")
    bmesh.ops.dissolve_degenerate(bm, dist=0.0001, edges=bm.edges)
    bm.to_mesh(mesh.data)
    bm.free()
    print("dropped %d faces (backdrop / degenerate)" % len(dead))
    # one object per material, named for CARLA's semantic tags / マテリアルごとに分割・命名
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.separate(type="MATERIAL")
    bpy.ops.object.mode_set(mode="OBJECT")
    groups = {m: g for g, mats in cfg.get("tags", {}).items() for m in mats}
    parts = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    count = {}
    for o in parts:
        if not o.data.polygons:
            bpy.data.objects.remove(o)
            continue
        mat = o.data.materials[o.data.polygons[0].material_index].name
        tag = groups.get(mat, "SM")
        o.name = o.data.name = "%s_%s" % (tag, mat)
        count[tag] = count.get(tag, 0) + 1
    print("pieces", count)
    # textures packed in the glb -> files the FBX can embed / 内蔵テクスチャをファイルに
    tex_dir = os.path.join(out_dir, name + "_textures")
    os.makedirs(tex_dir, exist_ok=True)
    for img in bpy.data.images:
        if img.packed_file is not None:
            safe = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in img.name)
            img.filepath_raw = os.path.join(tex_dir, safe + ".png")
            img.file_format = "PNG"
            img.save()
            img.unpack(method="REMOVE")
            img.filepath = os.path.join(tex_dir, safe + ".png")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.export_scene.fbx(filepath=os.path.join(out_dir, name + ".fbx"), use_selection=True,
                             object_types={"MESH"}, apply_unit_scale=True, bake_space_transform=False,
                             mesh_smooth_type="FACE", path_mode="COPY", embed_textures=True)
    print("wrote", os.path.join(out_dir, name + ".fbx"))


main()
