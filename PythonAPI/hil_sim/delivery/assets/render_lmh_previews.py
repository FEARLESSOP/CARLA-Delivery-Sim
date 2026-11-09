"""Preview renders of the LastMile Hub LMH-3 and its Mini Pod (Blender, Cycles).

  lmh_hub_open.png       the source model: bay doors folded down to the sidewalk,
                         P1 out on the sidewalk lane, P2 on its ramp, flags up
  lmh_minipod_front.png  the Mini Pod alone, front three-quarter view
  lmh_minipod_rear.png   rear three-quarter view, lid and shutter open
  lmh_contract_check.png the BUILT assets (stage LMHVehicle.fbx + LMH*.fbx props)
                         placed only from lmh_parts.json with every motion
                         applied, the pods at their ramp feet: checks pivots,
                         axes and motion signs of the contract

CARLA -> Blender: (x, y, z) -> (x, -y, z); roll -> +X, pitch -> -Y, yaw -> -Z.

  blender -b --disable-autoexec -P render_lmh_previews.py -- <model_dir> <stage_dir> <lmh_parts.json> <out_dir>

LastMile Hub と Mini Pod のプレビュー画像を作る（Blender）。
"""

import json
import math
import os
import sys

import bpy
from mathutils import Euler, Matrix, Vector

DOOR_OPEN = 93.33


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 96
    sc.cycles.use_denoising = False
    sc.render.resolution_x, sc.render.resolution_y = 1400, 900
    sc.render.film_transparent = False
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.62, 0.7, 0.8, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.8
    sc.world = world
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = Euler((math.radians(50), math.radians(10), math.radians(-35)))
    sc.collection.objects.link(sun)


def box(name, lo, hi, rgb):
    bpy.ops.mesh.primitive_cube_add()
    o = bpy.context.active_object
    o.name = name
    o.scale = [(h - l) / 2 for l, h in zip(lo, hi)]
    o.location = [(h + l) / 2 for l, h in zip(lo, hi)]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    m.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = list(rgb) + [1]
    m.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.9
    o.data.materials.append(m)
    return o


def street():
    box("road", (-8, -8, -0.02), (8, 0.92, 0.0), (0.09, 0.09, 0.1))
    box("curb", (-8, 0.92, 0.0), (8, 1.06, 0.15), (0.55, 0.55, 0.52))
    box("sidewalk", (-8, 1.06, 0.0), (8, 6, 0.15), (0.45, 0.44, 0.42))


def camera(loc, target, lens=35):
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    cam.data.lens = lens
    cam.location = loc
    d = Vector(target) - Vector(loc)
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.collection.objects.link(cam)
    bpy.context.scene.camera = cam


def render(path):
    bpy.context.scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print("rendered", path)


def set_world(o, m):
    o.matrix_world = m


def hub_open(model_dir, out):
    reset()
    bpy.ops.import_scene.gltf(filepath=os.path.join(model_dir, "LastMileHub.glb"))
    ob = bpy.data.objects
    for i in (1, 2, 3):
        d = ob["Bay_%d_RampDoor" % i]
        d.matrix_world = d.matrix_world @ Matrix.Rotation(math.radians(-DOOR_OPEN), 4, "X")
    # pods: P1 out on the sidewalk lane, P2 halfway down its ramp, raised flags
    for name, loc in (("MiniPod_P1", (-0.66, 2.0, 0.15)), ("MiniPod_P2", (0.0, 1.25, 0.17))):
        p = ob[name]
        mast, flag = ob[name + "_FlagMast"], ob[name + "_Flag"]
        p.matrix_world = Matrix.Translation(loc) @ p.matrix_world.to_quaternion().to_matrix().to_4x4()
        bpy.context.view_layer.update()
        for n, up in ((mast, 0.259), (flag, 0.517)):
            loc_w, rot_w, _ = n.matrix_world.decompose()
            n.matrix_world = Matrix.Translation(loc_w + Vector((0, 0, up))) @ rot_w.to_matrix().to_4x4()
    street()
    camera((3.6, 6.2, 2.6), (0.0, 0.6, 0.6), 30)
    render(os.path.join(out, "lmh_hub_open.png"))


def pod_views(model_dir, out):
    reset()
    bpy.ops.import_scene.gltf(filepath=os.path.join(model_dir, "LastMileHub_MiniPod.glb"))
    box("ground", (-3, -3, -0.01), (3, 3, 0.0), (0.4, 0.4, 0.38))
    camera((1.6, 1.2, 0.85), (0.0, 0.0, 0.38), 40)
    render(os.path.join(out, "lmh_minipod_front.png"))
    ob = bpy.data.objects
    lid = ob["MiniPod_P1_Lid"]
    lid.matrix_world = lid.matrix_world @ Matrix.Rotation(math.radians(66), 4, "Y")   # pitch -66
    sh = ob["MiniPod_P1_Shutter"]
    sh.matrix_world = sh.matrix_world @ Matrix.Diagonal((1, 1, 0.04, 1))
    camera((-1.6, -1.3, 1.0), (0.0, 0.0, 0.38), 40)
    render(os.path.join(out, "lmh_minipod_rear.png"))


def c2b_matrix(loc, rot):
    """CARLA loc + [pitch, yaw, roll] -> Blender world matrix."""
    p, y, r = [math.radians(a) for a in rot]
    R = (Matrix.Rotation(-y, 4, "Z") @ Matrix.Rotation(-p, 4, "Y") @ Matrix.Rotation(r, 4, "X"))
    return Matrix.Translation((loc[0], -loc[1], loc[2])) @ R


def opened(part):
    loc, rot, prop = list(part["loc"]), list(part["rot"]), part["prop"]
    m = part.get("motion") or {}
    if m.get("type") == "slide":
        loc["xyz".index(m["axis"])] += m["amount"]
    elif m.get("type") == "rotate" and m.get("per") is None:
        rot[{"y": 0, "z": 1, "x": 2}[m["axis"]]] += m["amount"]
    elif m.get("type") == "scale" and m.get("runtime", {}).get("swap_prop"):
        prop = m["runtime"]["swap_prop"]
    return loc, rot, prop


def contract_check(stage, parts_path, out):
    reset()
    doc = json.load(open(parts_path))
    files = {f[:-4].lower(): os.path.join(stage, f) for f in os.listdir(stage) if f.endswith(".fbx")}
    cache = {}

    def place(prop, world):
        key = prop.replace("static.prop.", "")
        before = set(bpy.data.objects)
        if key not in cache:
            bpy.ops.import_scene.fbx(filepath=files[key])
            cache[key] = [o for o in bpy.data.objects if o not in before and o.type == "MESH"][0]
            o = cache[key]
        else:
            o = cache[key].copy()
            bpy.context.scene.collection.objects.link(o)
        o.matrix_world = world @ Matrix.Diagonal(o.matrix_world.to_scale().to_4d())
        return o

    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=files["lmhvehicle"])
    hub = doc["hub"]
    for part in hub["parts"]:
        loc, rot, prop = opened(part)
        place(prop, c2b_matrix(loc, rot))
    pod = doc["minipod"]
    for bay in hub["bays"]:
        x, y, z = bay["ramp_foot"]
        # out of its bay on the lane, nose toward the curb side (left, -y): yaw -90
        pw = c2b_matrix([x, y, z], [0, -90, 0])
        place(bay["pod_body"], pw)
        for part in pod["parts"]:
            loc, rot, prop = opened(part)
            if part["name"] == "Lid":
                prop = bay["pod_lid"]
            place(prop, pw @ c2b_matrix(loc, rot))
    pc = doc["parcels"]
    place(pc["large"], c2b_matrix([-0.66 - 0.85, -2.0, 0.15], [0, 0, 0]))
    street()
    camera((3.8, 6.4, 2.8), (0.0, 0.9, 0.5), 30)
    render(os.path.join(out, "lmh_contract_check.png"))
    camera((0.0, -6.5, 2.2), (0.0, 0.0, 0.8), 30)
    render(os.path.join(out, "lmh_contract_check_street.png"))


def main():
    model_dir, stage, parts, out = sys.argv[sys.argv.index("--") + 1:][:4]
    os.makedirs(out, exist_ok=True)
    which = sys.argv[sys.argv.index("--") + 5:] or ["hub", "pod", "check"]
    if "hub" in which:
        hub_open(model_dir, out)
    if "pod" in which:
        pod_views(model_dir, out)
    if "check" in which:
        contract_check(stage, parts, out)


if __name__ == "__main__":
    main()
