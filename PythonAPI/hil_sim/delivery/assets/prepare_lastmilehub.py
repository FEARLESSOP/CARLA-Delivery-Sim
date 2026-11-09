"""Split the LastMile Hub LMH-3 export into CARLA props + a drivable vehicle (Blender).

Input: the glTF files of ~/Downloads/LastMileHub_FBX_Maya_Unreal (metres, full
PBR materials; the FBX files are the same geometry in cm). Every part is its
own named node with its pivot where it turns or slides (README.md there), and
the meshes are named <Node>__<Material>.

Makes, in <out_dir>:

  LMHVehicle.fbx   the hub without its pods, parcels and animated parts,
                   wheels skinned to Prius-style wheel bones (root
                   Vehicle_Base), built in centimetres (see prepare_lastmile.py)
  LMH<Part>.fbx    one prop per moving-part type, origin on its pivot, axes
                   as in the vehicle (or pod) frame at rest
  textures/        every texture written to PNG (glb textures are packed and
                   nameless: without files the FBX embeds nothing)
  lmh_materials.json   PBR spec per material (base colour/texture, metallic,
                   roughness, emissive, glass) for lmh_materials.py in UE,
                   because UE's FBX import keeps only the base colour
  lmh_measured.json    pivots, sizes, wheel centres, parcel slots, dock
                   positions in CARLA frames; compose_lmh_parts.py turns it
                   into delivery/assets/lmh_parts.json (the contract)
  LMHChargeStation.fbx  the whole charging station (bays 03 + 04, pads, drone
                   deck) as one static mesh without its moving parts, origin =
                   the hub parked in bay 04; the moving parts are the plug,
                   pad and LMHDeckPlug props ("station" in lmh_measured.json)

Frames: Blender (x, y, z) arrives in CARLA as (x, -y, z). The glb's x is the
hub's front and +y (FBX -Z) its curb side with the bay doors, which is the
vehicle's LEFT in CARLA (keep-left maps: doors face the kerb).

Material and mesh names must not contain "light" or "sign" (CARLA's
commandlet drops such meshes): they become "lamp" / "sgn".

Run headless (downloaded files: --disable-autoexec):
  blender -b --disable-autoexec -P prepare_lastmilehub.py -- \
      <LastMileHub.glb> <LastMileHub_MiniPod.glb> <LastMileHub_ChargeDock.glb> <out_dir> \
      [<LastMileHub_Neighbourhood.glb>]   (the parcel locker with the robot port)

LastMile Hub LMH-3 を CARLA 用の車両とプロップに分割する（Blender）。
"""

import json
import math
import os
import re
import sys

import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

WHEEL_BONES = {"Wheel_Front_Left": "Hub_Wheel_FrontLeft", "Wheel_Front_Right": "Hub_Wheel_FrontRight",
               "Wheel_Rear_Left": "Hub_Wheel_RearLeft", "Wheel_Rear_Right": "Hub_Wheel_RearRight"}
# nodes of LastMileHub.glb that are not part of the vehicle body (props or
# loose cargo); a trailing "*" is a prefix / 車両本体に含めないノード
NOT_VEHICLE = ("MiniPod_P*", "Parcel_*", "Bay_1_RampDoor", "Bay_2_RampDoor", "Bay_3_RampDoor",
               "Lift_*", "Diverter_Gate_*", "Bay_1_ChargePlug", "Bay_2_ChargePlug",
               "Bay_3_ChargePlug", "Bay_1_LockPins", "Bay_2_LockPins", "Bay_3_LockPins",
               "Hub_FrontChargePort_Cover")
# hub props: prop -> (node its meshes come from, instances in the hub frame)
HUB_PARTS = {
    "LMHRampDoor": ("Bay_1_RampDoor", ["Bay_1_RampDoor"]),
    "LMHRampDoor2": ("Bay_2_RampDoor", ["Bay_2_RampDoor"]),     # door number decal 2
    "LMHRampDoor3": ("Bay_3_RampDoor", ["Bay_3_RampDoor"]),
    "LMHLiftCarriage": ("Lift_1_Carriage", ["Lift_%d_Carriage" % i for i in (1, 2, 3)]),
    "LMHDiverterGate": ("Diverter_Gate_1", ["Diverter_Gate_%d" % i for i in (1, 2, 3)]),
    "LMHChargePlug": ("Bay_1_ChargePlug", ["Bay_%d_ChargePlug" % i for i in (1, 2, 3)]),
    "LMHLockPins": ("Bay_1_LockPins", ["Bay_%d_LockPins" % i for i in (1, 2, 3)]),
    "LMHPortCover": ("Hub_FrontChargePort_Cover", ["Hub_FrontChargePort_Cover"]),
}
# pod props from LastMileHub_MiniPod.glb (pod frame: nose +x, origin on the
# ground at the footprint centre, flag raised) / ポッド単体ファイルから
POD = "MiniPod_P1"
POD_PARTS = {
    "LMHPodWheel": "MiniPod_P1_Wheel1",
    "LMHPodLid": "MiniPod_P1_Lid",
    "LMHPodShutter": "MiniPod_P1_Shutter",
    "LMHFlagMast": "MiniPod_P1_FlagMast",
    "LMHFlag": "MiniPod_P1_Flag",
}
POD_SUBNODES = ["MiniPod_P1_Wheel%d" % i for i in range(1, 7)] + [
    "MiniPod_P1_Lid", "MiniPod_P1_Shutter", "MiniPod_P1_FlagMast", "MiniPod_P1_Flag"]
SHUTTER_ROLLED = 0.04          # README: shutter scale Y 1 -> 0.04 rolls it up
# charging station (LastMileHub_ChargeDock.glb, hub parked in bay 04 at 0,0,0);
# bay 03 / the drone deck belong to the LMP-4 and are left out
DOCK_PARTS = {
    "LMHDockBay": (["HubBay04", "HubBay04_Cabinet", "HubBay04_Screen"], "origin", ["HubBay04"]),
    "LMHDockPad": (["MiniPodPad_M1"], "node", ["MiniPodPad_M%d" % i for i in (1, 2, 3)]),
    "LMHDockPadPins": (["MiniPodPad_M1_LockPins"], "node", ["MiniPodPad_M%d_LockPins" % i for i in (1, 2, 3)]),
    "LMHDockPadPlug": (["MiniPodPad_M1_Plug"], "node", ["MiniPodPad_M%d_Plug" % i for i in (1, 2, 3)]),
    "LMHPlugBase": (["HubPlugUnit_Base"], "node", ["HubPlugUnit_Base"]),
    "LMHPlugZ": (["HubPlugUnit_ZCarriage"], "node", ["HubPlugUnit_ZCarriage"]),
    "LMHPlugY": (["HubPlugUnit_YCarriage"], "node", ["HubPlugUnit_YCarriage"]),
    "LMHPlugBoom": (["HubPlugUnit_BoomStage"], "node", ["HubPlugUnit_BoomStage"]),
    "LMHPlugHead": (["HubPlugUnit_Head"], "node", ["HubPlugUnit_Head"]),
}
DOCK_NODES = set(n for v in DOCK_PARTS.values() for n in v[0] + v[2]) | {
    "MiniPodPad_M%d%s" % (i, s) for i in (1, 2, 3) for s in ("", "_LockPins", "_Plug")}
DOCK_SKIP = ("PlugUnit_", "DroneDeck", "ChargeDock")      # LMP-4 bay 03 and its deck
# the whole station as one static prop: everything but these moving subtrees,
# which are spawned as the plug / pad / deck-plug props / 可動部（別プロップ）を除いた駅全体
STATION_MOVING = ["PlugUnit_ZCarriage", "HubPlugUnit_ZCarriage"] + [
    "MiniPodPad_M%d_%s" % (i, s) for i in (1, 2, 3) for s in ("LockPins", "Plug")] + [
    "DroneDeck_Pad%d_Plug" % i for i in (1, 2, 3)]
PLUG_CHAIN = ("Base", "ZCarriage", "YCarriage", "BoomStage", "Head")
# bay -> (node of the parked vehicle or None = origin, plug-unit prefix, painted
# bay outline mesh) / 区画：駐車位置ノード・プラグユニット・区画線メッシュ
STATION_BAYS = {"lmp": ("ChargeDock", "PlugUnit_", "ChargeDock__M_Standard_f0c020"),
                "lmh": (None, "HubPlugUnit_", "HubBay04__M_Standard_f0c020")}
CLEAR_FROM_Z, CLEAR_STEP = 0.05, 0.1      # roof clearance rays (m) / 天井高を測るレイ

OUT = {"props": {}, "hub": {}, "pod": {}, "dock": {}}
SPEC = {}
OUT_DIR = ""


# ---------------------------------------------------------------- scene helpers

def load(path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=path)
    fix_materials()
    return [o for o in bpy.context.scene.objects if o.type == "MESH"]


def ancestors(o):
    out, p = [], o.parent
    while p is not None:
        out.append(p.name)
        p = p.parent
    return out


def match(name, pat):
    return name.startswith(pat[:-1]) if pat.endswith("*") else name == pat


def under(o, pats):
    return any(match(n, p) for n in ancestors(o) for p in pats)


def own(meshes, node, part_nodes):
    """Meshes of ``node`` itself: below it, not below another part node in it."""
    out = []
    for o in meshes:
        names = ancestors(o)
        if node not in names:
            continue
        if not any(n in part_nodes for n in names[:names.index(node)]):
            out.append(o)
    return out


def bounds(objs, frame=None):
    inv = frame.inverted() if frame is not None else Matrix.Identity(4)
    vs = [inv @ (o.matrix_world @ v.co) for o in objs for v in o.data.vertices]
    lo = Vector([min(v[i] for v in vs) for i in range(3)])
    hi = Vector([max(v[i] for v in vs) for i in range(3)])
    return lo, hi


def carla(v):
    return [round(v.x, 4), round(-v.y, 4), round(v.z, 4)]


def node(name):
    return bpy.data.objects[name]


def bake(objs, name, frame):
    """One joined mesh of copies of ``objs`` expressed in ``frame`` (its origin
    becomes the prop's pivot) / ``frame`` を原点とした結合メッシュ。"""
    inv = frame.inverted()
    copies = []
    for o in objs:
        c = o.copy()
        c.data = o.data.copy()
        bpy.context.scene.collection.objects.link(c)
        c.data.transform(inv @ o.matrix_world)
        c.parent = None
        c.matrix_world = Matrix.Identity(4)
        copies.append(c)
    bpy.ops.object.select_all(action="DESELECT")
    for c in copies:
        c.select_set(True)
    bpy.context.view_layer.objects.active = copies[0]
    if len(copies) > 1:
        bpy.ops.object.join()
    obj = bpy.context.view_layer.objects.active
    obj.name = obj.data.name = name
    return obj


def export_prop(obj, size_note=None):
    lo = Vector([min(v.co[i] for v in obj.data.vertices) for i in range(3)])
    hi = Vector([max(v.co[i] for v in obj.data.vertices) for i in range(3)])
    mats = sorted({s.material.name for s in obj.material_slots if s.material})
    OUT["props"][obj.name] = {"size": [round(x, 4) for x in (hi - lo)],
                              "bbox_min": carla(Vector((lo.x, hi.y, lo.z))),
                              "bbox_max": carla(Vector((hi.x, lo.y, hi.z))),
                              "materials": mats}
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.export_scene.fbx(filepath=os.path.join(OUT_DIR, obj.name + ".fbx"),
                             use_selection=True, object_types={"MESH"}, apply_unit_scale=True,
                             bake_space_transform=False, mesh_smooth_type="FACE",
                             path_mode="COPY", embed_textures=True)
    print("PROP %-16s size %.3f x %.3f x %.3f  verts %d  mats %d" % (
        (obj.name,) + tuple(hi - lo) + (len(obj.data.vertices), len(mats))))
    bpy.data.objects.remove(obj)


# ---------------------------------------------------------------- materials

def clean(name):
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    s = re.sub(r"(?i)light", "lamp", s)
    return re.sub(r"(?i)sign", "sgn", s)


def upstream_image(socket, seen=None):
    """The first image texture feeding ``socket`` (through mix nodes)."""
    seen = seen or set()
    for link in socket.links:
        n = link.from_node
        if n.name in seen:
            continue
        seen.add(n.name)
        if n.type == "TEX_IMAGE" and n.image:
            return n, link.from_socket.name
        for inp in n.inputs:
            r = upstream_image(inp, seen)
            if r:
                return r
    return None


def tint_of(socket):
    """Colour a MULTIPLY mix applies to the texture (three.js map x color)."""
    for link in socket.links:
        n = link.from_node
        if n.type == "MIX" and getattr(n, "blend_type", "") == "MULTIPLY":
            for inp in n.inputs:
                if inp.type == "RGBA" and not inp.links:
                    return [round(x, 4) for x in list(inp.default_value)[:3]]
    return None


def rgb(v):
    return [round(x, 4) for x in list(v)[:3]]


def analyse(m):
    nt = m.node_tree
    p = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
    spec = {"base": [0.8, 0.8, 0.8], "metallic": 0.0, "roughness": 0.5, "emissive": None,
            "opacity": 1.0, "mode": "opaque", "texture": None, "image": None}
    if p is not None:
        spec["base"] = rgb(p.inputs["Base Color"].default_value)
        spec["metallic"] = round(p.inputs["Metallic"].default_value, 3)
        spec["roughness"] = round(p.inputs["Roughness"].default_value, 3)
        img = upstream_image(p.inputs["Base Color"])
        if img:
            spec["image"] = img[0].image
            spec["base"] = [1.0, 1.0, 1.0]
        a = p.inputs["Alpha"]
        if a.links:
            spec["mode"] = "masked"
        elif a.default_value < 0.99:
            spec["mode"] = "translucent"
            spec["opacity"] = round(a.default_value, 3)
        es = p.inputs["Emission Strength"].default_value
        ec = rgb(p.inputs["Emission Color"].default_value)
        if es > 0 and max(ec) > 0:
            spec["emissive"] = [round(c * es, 4) for c in ec]
    else:
        # three.js MeshBasicMaterial (labels, screens): an emission shader with
        # the texture; shown as a lit printed surface in CARLA, where unlit
        # value 1 reads nearly black under the physical sun
        # 印刷面（ラベル・画面）は CARLA では通常の印刷面として扱う
        em = next((n for n in nt.nodes if n.type == "EMISSION"), None)
        spec["roughness"] = 0.6
        if em is not None:
            img = upstream_image(em.inputs["Color"])
            if img:
                spec["image"] = img[0].image
                spec["base"] = tint_of(em.inputs["Color"]) or [1.0, 1.0, 1.0]
            else:
                spec["base"] = rgb(em.inputs["Color"].default_value)
        if any(n.type == "TEX_IMAGE" and any(l.to_node.type == "MIX_SHADER" for l in n.outputs["Alpha"].links)
               for n in nt.nodes):
            spec["mode"] = "masked"
        spec["printed"] = True
    return spec


def fix_materials():
    """Clean the names, write each texture to PNG, record the PBR spec and
    leave a plain Principled material (base colour texture) that the FBX
    exporter understands. / 名前・テクスチャ・PBR 仕様を整える。"""
    tex_dir = os.path.join(OUT_DIR, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    for m in list(bpy.data.materials):
        if not m.use_nodes:
            continue
        name = clean(m.name)
        spec = analyse(m)
        img = spec.pop("image")
        nt = m.node_tree
        nt.nodes.clear()
        out = nt.nodes.new("ShaderNodeOutputMaterial")
        p = nt.nodes.new("ShaderNodeBsdfPrincipled")
        nt.links.new(p.outputs["BSDF"], out.inputs["Surface"])
        p.inputs["Base Color"].default_value = list(spec["base"]) + [1.0]
        p.inputs["Metallic"].default_value = spec["metallic"]
        p.inputs["Roughness"].default_value = spec["roughness"]
        if img is not None:
            tname = "T_LMH_" + name
            path = os.path.join(tex_dir, tname + ".png")
            own_img = img.copy()                 # one file per material / 材質ごとに 1 枚
            own_img.name = tname
            own_img.filepath_raw = path
            own_img.file_format = "PNG"
            own_img.save()
            if own_img.packed_file is not None:
                own_img.unpack(method="REMOVE")
            own_img.filepath = path
            t = nt.nodes.new("ShaderNodeTexImage")
            t.image = own_img
            nt.links.new(t.outputs["Color"], p.inputs["Base Color"])
            if spec["mode"] == "masked":
                nt.links.new(t.outputs["Alpha"], p.inputs["Alpha"])
            spec["texture"] = tname
        if spec["mode"] == "translucent":
            p.inputs["Alpha"].default_value = spec["opacity"]
        if spec["mode"] != "opaque":
            m.blend_method = "BLEND" if spec["mode"] == "translucent" else "CLIP"
        if spec["emissive"]:
            p.inputs["Emission Color"].default_value = [min(1.0, c) for c in spec["emissive"]] + [1.0]
            p.inputs["Emission Strength"].default_value = 3.0
        m.name = name
        if name in SPEC and SPEC[name] != spec:
            print("WARNING material %s differs between files; keeping the first" % name)
        SPEC.setdefault(name, spec)


# ---------------------------------------------------------------- vehicle

def make_vehicle(meshes):
    keep = [o for o in meshes if not under(o, NOT_VEHICLE)]
    wheel_of = {}
    for o in keep:
        for bone, group in WHEEL_BONES.items():
            if group in ancestors(o):
                wheel_of[o] = bone
    centres, radius, width = {}, 0.0, 0.0
    for bone in WHEEL_BONES:
        lo, hi = bounds([o for o in keep if wheel_of.get(o) == bone])
        centres[bone] = (lo + hi) / 2.0
        radius = max(radius, (hi.z - lo.z) / 2.0)
        width = max(width, hi.y - lo.y)
    lo, hi = bounds(keep)
    copies = []
    for o in keep:
        c = o.copy()
        c.data = o.data.copy()
        bpy.context.scene.collection.objects.link(c)
        c.data.transform(o.matrix_world)
        c.parent = None
        c.matrix_world = Matrix.Identity(4)
        if o in wheel_of:
            vg = c.vertex_groups.new(name=wheel_of[o])
            vg.add(list(range(len(c.data.vertices))), 1.0, "REPLACE")
        copies.append(c)
    for o in list(bpy.context.scene.objects):
        if o not in copies:
            bpy.data.objects.remove(o)
    bpy.ops.object.select_all(action="DESELECT")
    for c in copies:
        c.select_set(True)
    bpy.context.view_layer.objects.active = copies[0]
    bpy.ops.object.join()
    mesh = bpy.context.view_layer.objects.active
    mesh.name = mesh.data.name = "LMHVehicle"
    # centimetres: a metre FBX leaves a 100x scale on the root bone in UE
    # cm で作る（m だとルートボーンに 100 倍のスケールが残る）
    bpy.context.scene.unit_settings.scale_length = 0.01
    mesh.data.transform(Matrix.Scale(100.0, 4))
    arm = bpy.data.objects.new("Vehicle_Base", bpy.data.armatures.new("Vehicle_Base"))
    bpy.context.scene.collection.objects.link(arm)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    for bone, c in centres.items():
        b = arm.data.edit_bones.new(bone)
        b.head = c * 100.0
        b.tail = c * 100.0 + Vector((0.0, 1.0, 0.0))
    bpy.ops.object.mode_set(mode="OBJECT")
    mesh.parent = arm
    mesh.modifiers.new("Armature", "ARMATURE").object = arm
    bpy.ops.object.select_all(action="DESELECT")
    mesh.select_set(True)
    arm.select_set(True)
    bpy.ops.export_scene.fbx(filepath=os.path.join(OUT_DIR, "LMHVehicle.fbx"),
                             use_selection=True, object_types={"ARMATURE", "MESH"},
                             apply_unit_scale=True, bake_space_transform=False,
                             mesh_smooth_type="FACE", path_mode="COPY", embed_textures=True,
                             add_leaf_bones=False, bake_anim=False, armature_nodetype="NULL",
                             use_armature_deform_only=False)
    mats = sorted({s.material.name for s in mesh.material_slots if s.material})
    OUT["hub"]["vehicle"] = {
        "wheels": {b: carla(c) for b, c in centres.items()},
        "wheel_radius": round(radius, 4), "wheel_width": round(width, 4),
        "bbox_min": carla(Vector((lo.x, hi.y, lo.z))), "bbox_max": carla(Vector((hi.x, lo.y, hi.z))),
        "size": [round(x, 4) for x in (hi - lo)], "materials": mats, "verts": len(mesh.data.vertices)}
    print("VEHICLE LMHVehicle %d verts, %d materials, size %s, wheels %s r %.3f w %.3f" % (
        len(mesh.data.vertices), len(mats), OUT["hub"]["vehicle"]["size"],
        OUT["hub"]["vehicle"]["wheels"], radius, width))


# ---------------------------------------------------------------- hub

def make_hub_props(meshes):
    part_nodes = {n for v in HUB_PARTS.values() for n in v[1]} | {
        "MiniPod_P%d" % i for i in (1, 2, 3)}
    for prop, (src, insts) in HUB_PARTS.items():
        objs = own(meshes, src, part_nodes)
        if not objs:
            raise RuntimeError("no meshes for %s (%s)" % (prop, src))
        pivot = node(src).matrix_world.translation.copy()
        export_prop(bake(objs, prop, Matrix.Translation(pivot)))
        for n in insts:
            OUT["hub"].setdefault("pivots", {})[n] = carla(node(n).matrix_world.translation)
    # pods docked in the hub: their frames, and bodies/lids 2 and 3 (number decals)
    # 格納中のポッドの座標系と、番号違いの本体・蓋
    sub = {"MiniPod_P%d_%s" % (i, s) for i in (1, 2, 3)
           for s in ["Wheel%d" % k for k in range(1, 7)] + ["Lid", "Shutter", "FlagMast", "Flag"]}
    for i in (1, 2, 3):
        pn = node("MiniPod_P%d" % i)
        m = pn.matrix_world
        yaw = math.degrees(m.to_euler().z)
        OUT["hub"].setdefault("pods", {})["P%d" % i] = {
            "loc": carla(m.translation), "yaw": round(-yaw, 2)}
        if i == 1:
            continue
        body = own(meshes, pn.name, sub)
        export_prop(bake(body, "LMHMiniPod%d" % i, m.copy()))
        lid = own(meshes, "MiniPod_P%d_Lid" % i, sub)
        lid_pivot = m.inverted() @ node("MiniPod_P%d_Lid" % i).matrix_world.translation
        export_prop(bake(lid, "LMHPodLid%d" % i, m @ Matrix.Translation(lid_pivot)))
    # parcels: bottom centre, in each parcel's own frame / 荷物：底面中心
    slots = {}
    for kind, src in (("LMHParcelL", "Parcel_P1"), ("LMHParcelS", "Parcel_S1")):
        objs = own(meshes, src, set())
        nm = node(src).matrix_world
        lo, hi = bounds(objs, nm)
        bc = Vector(((lo.x + hi.x) / 2, (lo.y + hi.y) / 2, lo.z))
        export_prop(bake(objs, kind, nm @ Matrix.Translation(bc)))
    for o in bpy.context.scene.objects:
        if o.type == "EMPTY" and o.name.startswith("Parcel_"):
            objs = own(meshes, o.name, set())
            nm = o.matrix_world
            lo, hi = bounds(objs, nm)
            bc = nm @ Vector(((lo.x + hi.x) / 2, (lo.y + hi.y) / 2, lo.z))
            slots[o.name] = {"loc": carla(bc), "yaw": round(-math.degrees(nm.to_euler().z), 2),
                             "size": [round(x, 3) for x in (hi - lo)]}
    OUT["hub"]["parcel_slots"] = slots
    lo, hi = bounds(meshes)
    OUT["hub"]["model_bbox_min"] = carla(Vector((lo.x, hi.y, lo.z)))
    OUT["hub"]["model_bbox_max"] = carla(Vector((hi.x, lo.y, hi.z)))


# ---------------------------------------------------------------- pod

def make_pod_props(meshes):
    sub = set(POD_SUBNODES)
    body = own(meshes, POD, sub)
    export_prop(bake(body, "LMHMiniPod", Matrix.Identity(4)))
    lo, hi = bounds(meshes)
    OUT["pod"]["bbox_min"] = carla(Vector((lo.x, hi.y, lo.z)))
    OUT["pod"]["bbox_max"] = carla(Vector((hi.x, lo.y, hi.z)))
    lo, hi = bounds(body)
    OUT["pod"]["body_size"] = [round(x, 4) for x in (hi - lo)]
    pivots = {}
    for n in POD_SUBNODES:
        pivots[n.replace("MiniPod_P1_", "")] = carla(node(n).matrix_world.translation)
    for prop, src in POD_PARTS.items():
        objs = own(meshes, src, sub)
        pivot = node(src).matrix_world.translation.copy()
        if prop == "LMHFlagMast":
            # pivot at the foot of the mast (the node sits at its middle,
            # where the web viewer scales it) / 支柱の根元を支点にする
            lo, hi = bounds(objs)
            pivot = Vector((pivot.x, pivot.y, lo.z))
            pivots["FlagMastFoot"] = carla(pivot)
            OUT["pod"]["flag_mast_length"] = round(hi.z - lo.z, 4)
        export_prop(bake(objs, prop, Matrix.Translation(pivot)))
        if prop == "LMHPodShutter":
            # the rolled-up shutter as its own prop (CARLA can't scale a prop
            # at run time): swap it in when open / 巻き上げ状態の別プロップ
            o = bake(objs, "LMHPodShutterUp", Matrix.Translation(pivot))
            o.data.transform(Matrix.Diagonal((1.0, 1.0, SHUTTER_ROLLED, 1.0)))
            export_prop(o)
    lo, hi = bounds(own(meshes, "MiniPod_P1_Shutter", sub))
    OUT["pod"]["shutter_box"] = {"min": carla(Vector((lo.x, hi.y, lo.z))), "max": carla(Vector((hi.x, lo.y, hi.z)))}
    OUT["pod"]["pivots"] = pivots
    w = own(meshes, "MiniPod_P1_Wheel1", sub)
    lo, hi = bounds(w)
    OUT["pod"]["wheel_radius"] = round((hi.z - lo.z) / 2, 4)


# ---------------------------------------------------------------- dock

def make_dock_props(meshes):
    part_nodes = DOCK_NODES
    for prop, (srcs, rule, insts) in DOCK_PARTS.items():
        objs = []
        for s in srcs:
            objs += own(meshes, s, part_nodes - {s})
        if prop == "LMHDockBay":
            objs = [o for o in objs if not any(n.startswith(DOCK_SKIP) for n in ancestors(o)[:1])]
        if not objs:
            raise RuntimeError("no meshes for %s" % prop)
        pivot = Vector((0, 0, 0)) if rule == "origin" else node(srcs[0]).matrix_world.translation.copy()
        export_prop(bake(objs, prop, Matrix.Translation(pivot)))
        for n in insts:
            OUT["dock"].setdefault("pivots", {})[n] = carla(Vector((0, 0, 0)) if rule == "origin"
                                                            else node(n).matrix_world.translation)


def clearance(obj, x0, x1, y0, y1):
    """Lowest surface over the strip x0..x1, y0..y1 (Blender frame, ``obj``
    baked at the origin): vertical rays from just above the floor.
    進入路上の最低の天井高（床のすぐ上から鉛直レイ）。"""
    bvh = BVHTree.FromPolygons([v.co for v in obj.data.vertices], [p.vertices for p in obj.data.polygons])
    low, nx, ny = None, int((x1 - x0) / CLEAR_STEP) + 1, int((y1 - y0) / CLEAR_STEP) + 1
    for i in range(nx + 1):
        for j in range(ny + 1):
            o = Vector((x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny, CLEAR_FROM_Z))
            hit = bvh.ray_cast(o, Vector((0.0, 0.0, 1.0)))[0]
            if hit is not None and (low is None or hit.z < low):
                low = hit.z
    return None if low is None else round(low, 4)


def make_station(meshes):
    """LMHChargeStation (one static mesh of the whole station without its
    moving parts, origin = Blender 0,0,0 = the hub parked in bay 04) and
    LMHDeckPlug (drone-deck pad plug, pivot on its node); OUT["station"]
    holds the bay poses, plug-unit / pad / deck pivots and roof clearances
    in CARLA frame. 充電ステーション全体（静的）とドローンデッキのプラグ。"""
    st = {}
    objs = [o for o in meshes if not under(o, STATION_MOVING)]
    obj = bake(objs, "LMHChargeStation", Matrix.Identity(4))
    lo, hi = bounds([obj])
    st["bbox_min"] = carla(Vector((lo.x, hi.y, lo.z)))
    st["bbox_max"] = carla(Vector((hi.x, lo.y, hi.z)))
    st["size"] = [round(x, 4) for x in (hi - lo)]
    st["entrance_x"] = round(lo.x, 4)
    names = {o.name: o for o in meshes}
    for bay, (vnode, prefix, outline) in STATION_BAYS.items():
        if vnode is None:
            pose = [0.0, 0.0, 0.0]
        else:
            m = node(vnode).matrix_world
            pose = carla(m.translation)[:2] + [round(-math.degrees(m.to_euler().z), 2) + 0.0]
        st.setdefault("bays", {})[bay] = pose
        st.setdefault("plug_units", {})[bay] = {
            k: carla(node(prefix + k).matrix_world.translation) for k in PLUG_CHAIN}
        if outline in names:
            # painted bay box, and the lowest structure over the drive-in strip
            # (box width) from the entrance edge to the stop bar: the canopy's
            # cross beams / 区画線の箱と進入路上の天井高（梁の下端）
            blo, bhi = bounds([names[outline]])
            st.setdefault("bay_boxes", {})[bay] = {"min": carla(Vector((blo.x, bhi.y, 0.0)))[:2],
                                                   "max": carla(Vector((bhi.x, blo.y, 0.0)))[:2]}
            st.setdefault("roof_clearance", {})[bay] = clearance(obj, lo.x, bhi.x, blo.y, bhi.y)
        else:
            print("WARNING no bay outline %s: no roof clearance for %s" % (outline, bay))

    def pad(fmt):
        return [carla(node(fmt % i).matrix_world.translation) for i in (1, 2, 3)]
    st["pads"] = pad("MiniPodPad_M%d")
    st["pad_pins"] = pad("MiniPodPad_M%d_LockPins")
    st["pad_plugs"] = pad("MiniPodPad_M%d_Plug")
    st["deck_pads"] = pad("DroneDeck_Pad%d")
    st["deck_plugs"] = pad("DroneDeck_Pad%d_Plug")
    print("STATION entrance x %.3f, roof clearance %s" % (lo.x, st.get("roof_clearance")))
    export_prop(obj)
    src = "DroneDeck_Pad1_Plug"
    export_prop(bake(own(meshes, src, set()), "LMHDeckPlug",
                     Matrix.Translation(node(src).matrix_world.translation.copy())))
    st["apron"] = make_apron(objs)
    st["walk"] = make_walk()
    OUT["station"] = st


# the footpath round a charging station, in the map's own pavement look:
# interlocking blocks between granite kerb edges (osm_city/textures_jp.py's
# pave_block / kerb_jp, made by our map generator) / 周回歩道（地図の歩道と同じ見た目）
MAP_TEX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "maps", "build", "OitaKosen", "textures")
WALK_W, WALK_KERB, WALK_T, WALK_UP = 1.5, 0.15, 0.06, 0.04   # paving width, kerb width, thickness, kerb upstand
WALK_L = 2.0                  # segment length = the paving texture's repeat / 直線部の長さ


def walk_material(name, image, tile, rough):
    """A material on one of the map generator's pavement textures, recorded
    in SPEC like the model's own (UE takes the texture). / 地図の歩道テクスチャの材質。"""
    tex_dir = os.path.join(OUT_DIR, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    tname = "T_LMH_" + name
    dst = os.path.join(tex_dir, tname + ".png")
    import shutil
    shutil.copyfile(os.path.join(MAP_TEX, image), dst)
    img = bpy.data.images.load(dst)
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    pr = nt.nodes.new("ShaderNodeBsdfPrincipled")
    nt.links.new(pr.outputs["BSDF"], out.inputs["Surface"])
    pr.inputs["Roughness"].default_value = rough
    t = nt.nodes.new("ShaderNodeTexImage")
    t.image = img
    nt.links.new(t.outputs["Color"], pr.inputs["Base Color"])
    SPEC[name] = {"base": [1.0, 1.0, 1.0], "metallic": 0.0, "roughness": rough, "emissive": None,
                  "opacity": 1.0, "mode": "opaque", "texture": tname}
    return m, tile


def _box(bm, uv, x0, x1, y0, y1, z0, z1, mi, tile, skip=()):
    """An axis-aligned box into ``bm`` (material slot ``mi``), UVs projected
    per face at ``tile`` metres per texture repeat. ``skip``: the side faces
    to leave out ("-x", "+x", "-y", "+y", Blender axes) where the next piece
    joins: a face there renders as a dark seam. / 直方体（継ぎ目の面は省く）。"""
    import bmesh
    vs = [bm.verts.new((x, y, z)) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    sides = [None, None, "-y", "+y", "-x", "+x"]
    faces = [f for f, sd in zip(faces, sides) if sd not in skip]
    for f in faces:
        face = bm.faces.new([vs[i] for i in f])
        face.material_index = mi
        face.normal_update()
        n = face.normal
        for loop in face.loops:
            c = loop.vert.co
            u, v = ((c.x, c.y) if abs(n.z) > 0.5 else (c.x + c.y, c.z) if abs(n.x) < 0.5 else (c.y, c.z))
            loop[uv].uv = (u / tile, v / tile)


def make_walk():
    """LMHWalk (a 2 m footpath segment along +x: block paving between kerb
    edges, origin bottom centre) and LMHWalkCorner (a square of paving with
    kerbs on its +x and CARLA +y edges, for the turns). / 歩道の直線部と角。"""
    import bmesh
    pave, pt = walk_material("walk_pave", "pave_block.png", 2.0, 0.8)
    kerb, kt = walk_material("walk_kerb", "kerb_jp.png", 1.0, 0.7)
    h, k = WALK_W / 2.0, WALK_KERB
    out = {}
    for name in ("LMHWalk", "LMHWalkCorner"):
        bm = bmesh.new()
        uv = bm.loops.layers.uv.new("map1")
        if name == "LMHWalk":
            # 2 m long = exactly one repeat of the 2 m paving texture: the
            # block pattern runs on across the joints / 2 m = テクスチャ 1 周期
            ends = ("-x", "+x")                     # the next segments join there / 継ぎ目
            _box(bm, uv, -WALK_L / 2, WALK_L / 2, -h, h, 0.0, WALK_T, 0, pt, ends + ("-y", "+y"))
            _box(bm, uv, -WALK_L / 2, WALK_L / 2, h, h + k, 0.0, WALK_T + WALK_UP, 1, kt, ends)
            _box(bm, uv, -WALK_L / 2, WALK_L / 2, -h - k, -h, 0.0, WALK_T + WALK_UP, 1, kt, ends)
        else:
            a = h + k                               # half the full width / 全幅の半分
            # segments join on -x and Blender +y (CARLA -y): no faces there
            _box(bm, uv, -a, a - k, -(a - k), a, 0.0, WALK_T, 0, pt, ("-x", "+y", "+x", "-y"))
            _box(bm, uv, a - k, a, -a, a, 0.0, WALK_T + WALK_UP, 1, kt, ("+y",))        # +x kerb
            _box(bm, uv, -a, a - k, -a, -(a - k), 0.0, WALK_T + WALK_UP, 1, kt, ("-x",))  # Blender -y = CARLA +y
        me = bpy.data.meshes.new(name)
        bm.to_mesh(me)
        bm.free()
        me.materials.append(pave)
        me.materials.append(kerb)
        obj = bpy.data.objects.new(name, me)
        bpy.context.scene.collection.objects.link(obj)
        export_prop(obj)
    print("WALK footpath props: width %.2f m (paving %.2f + kerbs), segment %.1f m" % (WALK_W + 2 * k, WALK_W, WALK_L))
    return {"prop": "static.prop.lmhwalk", "corner": "static.prop.lmhwalkcorner",
            "width": WALK_W + 2 * k, "segment": WALK_L, "kerb_up": WALK_UP, "thickness": WALK_T}


APRON = (2.0, 2.0, 0.04)       # driveway slab to a charging station: x, y, thickness / 進入路の舗装板


def make_apron(objs):
    """LMHApron: a 2 m x 2 m paving slab (4 cm, origin at its bottom centre)
    in the station floor's own material, laid in tiles by the sim from the
    kerb to the station's open edge as its access road. The floor material
    is the one covering the most upward-facing area at z 0.
    充電ステーションへの進入路用の舗装板（床と同じ材料）。"""
    import bmesh
    area = {}
    for o in objs:
        mw = o.matrix_world
        for poly in o.data.polygons:
            n = (mw.to_3x3() @ poly.normal).normalized()
            c = mw @ poly.center
            if n.z > 0.95 and abs(c.z) < 0.03 and poly.material_index < len(o.material_slots):
                m = o.material_slots[poly.material_index].material
                if m is not None:
                    area[m.name] = area.get(m.name, 0.0) + poly.area
    mat = bpy.data.materials[max(area, key=area.get)]
    sx, sy, t = APRON
    me = bpy.data.meshes.new("LMHApron")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=(sx, sy, t), verts=bm.verts)
    bmesh.ops.translate(bm, vec=(0.0, 0.0, t / 2.0), verts=bm.verts)
    uv = bm.loops.layers.uv.new("map1")
    for f in bm.faces:                       # 1 UV unit per metre, top projection / 1 m = 1 UV
        for loop in f.loops:
            loop[uv].uv = (loop.vert.co.x, loop.vert.co.y)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    obj = bpy.data.objects.new("LMHApron", me)
    bpy.context.scene.collection.objects.link(obj)
    print("APRON material %s (%.0f m2 of floor)" % (mat.name, area[mat.name]))
    export_prop(obj)
    return {"prop": "static.prop.lmhapron", "size": [sx, sy, t], "material": mat.name}


# ---------------------------------------------------------------- parcel locker

LOCKER, LOCKER_FLAP = "AptB_ParcelLocker", "AptB_Locker_RobotPortFlap"
UPPER_PORT_Z = 1.185          # LMP-4 port centre over the pavement / 上段ポート中心高さ
UPPER_FLAP_OUT = 0.03         # its flap stands this far out of the door faces / 扉面より手前


def make_locker_props(meshes):
    """Apartment B's parcel locker with its robot port (mini.js m_aptB), from
    LastMileHub_Neighbourhood.glb: LMHLocker (origin = the locker node on the
    pavement, front +y in CARLA) and LMHLockerFlap (pivot = its top hinge;
    README: +77 deg about X opens it inward). Measures the port, the hinge
    and the docking tag in the locker frame.
    ロボットポート付き宅配ロッカー（近隣モデルの Apartment B）。"""
    nm = node(LOCKER).matrix_world.copy()
    fm = node(LOCKER_FLAP).matrix_world.copy()
    body = own(meshes, LOCKER, {LOCKER_FLAP})
    chute = [o for o in body if o.name.endswith("__M_black")]
    frame = [o for o in body if o.name.endswith("__M_hazard")]
    tag = [o for o in body if "Printed" in o.name]
    lo, hi = bounds(body, nm)
    clo, chi = bounds(chute, nm)
    flo, _ = bounds(frame, nm)
    out = {"size": [round(x, 4) for x in (hi - lo)],
           "bbox_min": carla(Vector((lo.x, hi.y, lo.z))), "bbox_max": carla(Vector((hi.x, lo.y, hi.z))),
           # port opening centre on the front face (the frame's outer plane) / 投入口中心
           "port": carla(Vector(((clo.x + chi.x) / 2, flo.y, (clo.z + chi.z) / 2))),
           "port_size": [round(chi.x - clo.x, 4), round(chi.z - clo.z, 4)],
           "flap_pivot": carla(nm.inverted() @ fm.translation), "flap_open_deg": 77.0}
    if tag:
        tlo, thi = bounds(tag, nm)
        out["tag"] = carla((tlo + thi) / 2)
    # a second robot port straight above, at the LMP-4 side-port tray height
    # (lmp_rig: parcel 1.215-1.415 m over the road = ~1.09-1.29 m over the
    # pavement), so the drone pods deliver into the same locker. It sits on
    # the compartment doors: chute just in front of them, frame in front.
    # LMP-4 用の上段ポート（トレイ高さ）を同じロッカーに追加
    up = UPPER_PORT_Z - (clo.z + chi.z) / 2
    extra = []
    for o, dy in [(c, flo.y + 0.001 - clo.y) for c in chute] + [(h, -0.012) for h in frame]:
        cpy = o.copy()
        cpy.data = o.data.copy()
        bpy.context.scene.collection.objects.link(cpy)
        cpy.matrix_world = Matrix.Translation(nm.to_quaternion() @ Vector((0.0, dy, up))) @ o.matrix_world
        extra.append(cpy)
    out["port_upper"] = carla(Vector(((clo.x + chi.x) / 2, flo.y - 0.012, UPPER_PORT_Z)))
    fp = nm.inverted() @ fm.translation
    out["flap_pivot_upper"] = carla(Vector((fp.x, fp.y - UPPER_FLAP_OUT, fp.z + up)))
    export_prop(bake(body + extra, "LMHLocker", nm))
    export_prop(bake(own(meshes, LOCKER_FLAP, set()), "LMHLockerFlap", fm))
    OUT["locker"] = out


def main():
    global OUT_DIR
    args = sys.argv[sys.argv.index("--") + 1:]
    hub_src, pod_src, dock_src, OUT_DIR = args[:4]
    town_src = args[4] if len(args) > 4 else "-"
    os.makedirs(OUT_DIR, exist_ok=True)

    make_hub_props(load(hub_src))
    make_vehicle(load(hub_src))
    make_pod_props(load(pod_src))
    if dock_src != "-":
        dock = load(dock_src)
        make_dock_props(dock)
        make_station(dock)
    if town_src != "-":
        make_locker_props(load(town_src))

    with open(os.path.join(OUT_DIR, "lmh_measured.json"), "w") as f:
        json.dump(OUT, f, indent=1, sort_keys=True)
    with open(os.path.join(OUT_DIR, "lmh_materials.json"), "w") as f:
        json.dump(SPEC, f, indent=1, sort_keys=True)
    print("wrote %d props + LMHVehicle, %d materials" % (len(OUT["props"]), len(SPEC)))


if __name__ == "__main__":
    main()
