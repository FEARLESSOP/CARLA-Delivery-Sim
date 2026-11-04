"""Assemble the map in Blender and export the FBX for CARLA's importer.

  blender -b -P blender_build.py -- <out_dir> [--fbx] [--render name:x,y,z:tx,ty,tz ...]

Reads <out_dir>/meshes (build.py) and <out_dir>/textures. --fbx writes
<out_dir>/<Name>.fbx (x east, y north, metres, the same export settings as
the other maps: the importer turns it into CARLA's frame). --render makes
preview images (Cycles) from cameras at x,y,z looking at tx,ty,tz into
<out_dir>/previews/<name>.png.

Blender で地図を組み立て、CARLA 用 FBX を書き出す。プレビュー画像も作る。
"""

import json
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Vector


def args():
    a = sys.argv[sys.argv.index("--") + 1:]
    out = a[0]
    fbx = "--fbx" in a
    renders = []
    # --exposure E: preview camera exposure in stops (the Nishita sky's sun disc
    # over-exposes sunlit asphalt; Street View cameras expose for the scene)
    # プレビューの露出（段）
    exposure = float(a[a.index("--exposure") + 1]) if "--exposure" in a else 0.0
    if "--render" in a:
        for spec in a[a.index("--render") + 1:]:
            if spec.startswith("--"):
                break
            if ":" not in spec:
                continue
            n, p, t = spec.split(":")
            renders.append((n, [float(v) for v in p.split(",")], [float(v) for v in t.split(",")]))
    return out, fbx, renders, exposure


def image(tex_dir, fname, colour=True):
    path = os.path.join(tex_dir, fname)
    if not os.path.exists(path):
        return None
    img = bpy.data.images.load(path, check_existing=True)
    if not colour:
        img.colorspace_settings.name = "Non-Color"
    return img


def material(name, spec, tex_dir, ue):
    """Principled material with colour, roughness, metallic and normal maps.
    For the FBX (ue=True) the maps go where Unreal's FBX importer looks for
    them (UnFbx::FFbxImporter: FBX SpecularFactor -> Roughness, Shininess ->
    Metallic, NormalMap -> Normal; Blender exports Specular -> SpecularFactor
    and Roughness -> Shininess), so the roughness map is plugged into
    Specular and the metallic map into Roughness. Plain values never reach
    Unreal, only textures. For previews (ue=False) the maps go where they
    belong. / UE の FBX 取り込みの対応に合わせてマップを接続する。"""
    tex, col, rough, metal, smooth = spec
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if tex:
        node = nt.nodes.new("ShaderNodeTexImage")
        node.image = image(tex_dir, tex + ".png")
        nt.links.new(node.outputs["Color"], bsdf.inputs["Base Color"])
        cut = image(tex_dir, tex + "_summer.png") if not ue else None
        if cut is not None:
            # previews only: leaf cards cut out by the summer texture's alpha (in
            # Unreal the foliage material does it, maps/ue_foliage.py)
            # プレビューのみ: 葉カードをアルファで切り抜く（UE では植生マテリアル）
            node.image = cut
            nt.links.new(node.outputs["Alpha"], bsdf.inputs["Alpha"])
            m.blend_method = "CLIP"
    elif col:
        bsdf.inputs["Base Color"].default_value = (col[0] / 255.0, col[1] / 255.0, col[2] / 255.0, 1.0)
    r_img = image(tex_dir, name + "_r.png", False)
    m_img = image(tex_dir, name + "_m.png", False)
    n_img = image(tex_dir, tex + "_n.png", False) if tex else None
    if r_img:
        rn = nt.nodes.new("ShaderNodeTexImage")
        rn.image = r_img
        nt.links.new(rn.outputs["Color"], bsdf.inputs["Specular IOR Level" if ue else "Roughness"])
    if m_img:
        mn = nt.nodes.new("ShaderNodeTexImage")
        mn.image = m_img
        nt.links.new(mn.outputs["Color"], bsdf.inputs["Roughness" if ue else "Metallic"])
    if n_img:
        tn = nt.nodes.new("ShaderNodeTexImage")
        tn.image = n_img
        nm = nt.nodes.new("ShaderNodeNormalMap")
        nt.links.new(tn.outputs["Color"], nm.inputs["Color"])
        nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    return m


def build_object(name, path, mats, smooth_mats):
    d = np.load(path)
    names = sorted({k.split("|")[0] for k in d.files})
    Vs, Is, UVs, Ms = [], [], [], []
    off = 0
    me = bpy.data.meshes.new(name)
    for k, mat in enumerate(names):
        V, I, UV = d[mat + "|V"], d[mat + "|I"], d[mat + "|UV"]
        Vs.append(V)
        Is.append(I + off)
        UVs.append(UV[I.ravel()])
        Ms.append(np.full(len(I), k, np.int32))
        off += len(V)
        me.materials.append(mats[mat])
    V = np.concatenate(Vs)
    I = np.concatenate(Is)
    UV = np.concatenate(UVs)
    M = np.concatenate(Ms)
    nt = len(I)
    me.vertices.add(len(V))
    me.vertices.foreach_set("co", V.ravel())
    me.loops.add(nt * 3)
    me.loops.foreach_set("vertex_index", I.ravel().astype(np.int32))
    me.polygons.add(nt)
    me.polygons.foreach_set("loop_start", np.arange(0, nt * 3, 3, dtype=np.int32))
    me.polygons.foreach_set("loop_total", np.full(nt, 3, np.int32))
    me.polygons.foreach_set("material_index", M)
    sm = np.array([names[m] in smooth_mats for m in M], bool)
    me.polygons.foreach_set("use_smooth", sm)
    uv = me.uv_layers.new(name="UVMap")
    uv.data.foreach_set("uv", UV.ravel().astype(np.float32))
    me.update()
    me.validate(clean_customdata=False)
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def setup_render(out, exposure=0.0):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = 32
    sc.cycles.use_denoising = False
    sc.cycles.transparent_max_bounces = 128     # stacked leaf cards go black at 8 / 葉カードの重なり
    sc.render.resolution_x, sc.render.resolution_y = 1280, 720
    w = bpy.data.worlds.new("sky")
    sc.world = w
    w.use_nodes = True
    sky = w.node_tree.nodes.new("ShaderNodeTexSky")
    sky.sun_elevation = math.radians(55)
    sky.sun_rotation = math.radians(120)
    w.node_tree.links.new(sky.outputs["Color"], w.node_tree.nodes["Background"].inputs["Color"])
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.35
    sun = bpy.data.lights.new("sun", "SUN")
    sun.energy = 2.6
    so = bpy.data.objects.new("sun", sun)
    so.rotation_euler = (math.radians(35), 0, math.radians(120))
    sc.collection.objects.link(so)
    sc.view_settings.view_transform = "Filmic"
    sc.view_settings.exposure = exposure


def render(out, name, pos, tgt, lens=24):
    sc = bpy.context.scene
    cam = bpy.data.cameras.new(name)
    cam.lens = lens
    cam.clip_end = 3000
    co = bpy.data.objects.new(name, cam)
    sc.collection.objects.link(co)
    co.location = Vector(pos)
    co.rotation_euler = (Vector(tgt) - Vector(pos)).to_track_quat("-Z", "Y").to_euler()
    sc.camera = co
    os.makedirs(os.path.join(out, "previews"), exist_ok=True)
    sc.render.filepath = os.path.join(out, "previews", name + ".png")
    bpy.ops.render.render(write_still=True)


def main():
    out, fbx, renders, exposure = args()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    man = json.load(open(os.path.join(out, "meshes", "manifest.json")))
    tex_dir = os.path.join(out, "textures")
    mats = {k: material(k, v, tex_dir, ue=fbx) for k, v in man["materials"].items()}
    smooth = {k for k, v in man["materials"].items() if v[4]}
    for o in man["objects"]:
        build_object(o["name"], os.path.join(out, "meshes", o["name"] + ".npz"), mats, smooth)
    print("objects", len(man["objects"]))
    if fbx:
        path = os.path.join(out, man["name"] + ".fbx")
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.export_scene.fbx(filepath=path, use_selection=True, object_types={"MESH"},
                                 apply_unit_scale=True, bake_space_transform=False,
                                 mesh_smooth_type="FACE", path_mode="COPY", embed_textures=True)
        print("wrote", path, os.path.getsize(path) // (1 << 20), "MB")
    if renders:
        setup_render(out, exposure)
        for n, p, t in renders:
            render(out, n, p, t)


main()
