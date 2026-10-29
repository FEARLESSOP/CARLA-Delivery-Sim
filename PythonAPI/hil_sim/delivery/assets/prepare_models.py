"""Turn the artist's Pod.fbx / Drone.fbx into CARLA-importable props (Blender).

CARLA's importer (``make import``) wants one static mesh per prop, named like
the file. These exports are dozens of part meshes under scaled (cm) empties,
so this script joins them into one mesh per prop, with the origin on the
part's pivot. The parts that move are props of their own: the delivery code
moves them (delivery/rig.py has the same pivot numbers).

  Pod            body without the moving parts; ground, midway between axles
  PodJack        one jack (all four are the same); bottom of its foot pad
  PodHatchFront  front hatch door; on its hinge (front edge), bottom face
  PodHatchRear   rear hatch door;  on its hinge (rear edge), bottom face
  PodSideDoor    side-port door (PodV2); bottom centre
  PodSlideStage1..3, PodSlideCarrier
                 the side-port arm's telescoping stages and the tray (PodV2)
  PodVehicle     the pod as a drivable CARLA vehicle: the body and wheels on
                 the Toyota Prius's skeleton (root Vehicle_Base + four wheel
                 bones), so CARLA's wheel animation and physics apply
  Drone          drone without rotors; bottom of its legs, centred
  DroneRotor     one rotor blade (all four are the same); its hub
  Parcel         a small cardboard box made here; bottom centre

All face +X. Blender +X/+Y/+Z arrives in CARLA as +X forward, -Y -> +Y right,
+Z up (checked with a raycast height map of the cooked pod). Collision is
removed later, in Unreal, by strip_collision.py.

Run headless:
  blender -b -P prepare_models.py -- <Pod.fbx> <Drone.fbx> <out_dir>

アーティスト提供の Pod.fbx / Drone.fbx を CARLA のインポートできるプロップに
変換する（Blender）。動く部品（ジャッキ・ハッチ扉・ローター）は別プロップとし、
原点を回転・移動の支点に置く。
"""

import os
import sys

import bpy
from mathutils import Matrix, Vector

DRONE_SCALE = 1.0   # as modelled (0.77 m): it must fit through the 1.2 m hatch
PARCEL_SIZE = (0.32, 0.32, 0.24)   # m
PARCEL_COLOR = (0.42, 0.27, 0.13, 1.0)  # cardboard


def load(path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=path)


def group_of(obj):
    """Name of the first ancestor empty, e.g. 'Jack_FrontLeft'."""
    p = obj.parent
    names = []
    while p is not None:
        names.append(p.name)
        p = p.parent
    return names


def world_bounds(meshes):
    lo = Vector((1e9,) * 3)
    hi = Vector((-1e9,) * 3)
    for o in meshes:
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    return lo, hi


def keep_join(name, keep):
    """Delete every mesh not chosen by ``keep``, bake and join the rest."""
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    chosen = [o for o in meshes if keep(o)]
    if not chosen:
        raise RuntimeError("no meshes for %s" % name)
    for o in chosen:  # bake the cm->m empties into the vertices
        o.data = o.data.copy()
        o.data.transform(o.matrix_world)
    for o in [o for o in bpy.context.scene.objects if o not in chosen]:
        bpy.data.objects.remove(o)
    for o in chosen:
        o.parent = None
        o.matrix_world = Matrix.Identity(4)
    bpy.ops.object.select_all(action="DESELECT")
    for o in chosen:
        o.select_set(True)
    bpy.context.view_layer.objects.active = chosen[0]
    bpy.ops.object.join()
    obj = bpy.context.view_layer.objects.active
    obj.name = obj.data.name = name
    return obj


def shift(obj, origin, scale=1.0):
    obj.data.transform(Matrix.Scale(scale, 4) @ Matrix.Translation(-origin))


def bounds(obj):
    vs = [v.co for v in obj.data.vertices]
    return (Vector([min(v[i] for v in vs) for i in range(3)]),
            Vector([max(v[i] for v in vs) for i in range(3)]))


def export(path, obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.ops.export_scene.fbx(filepath=path, use_selection=True,
                             object_types={"MESH"}, apply_unit_scale=True,
                             bake_space_transform=False, mesh_smooth_type="FACE",
                             path_mode="STRIP")


def report(name, obj, origin, ref):
    lo, hi = bounds(obj)
    rel = origin - ref
    # pivot in CARLA's frame (y flips) relative to the parent prop's origin
    print("%-14s size %.3f x %.3f x %.3f m   pivot in parent (carla) "
          "x=%.3f y=%.3f z=%.3f" % ((name,) + tuple(hi - lo) + (rel.x, -rel.y, rel.z)))


def part(src, out_dir, name, keep, origin_fn, ref_fn, scale=1.0):
    load(src)
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    ref = ref_fn(meshes)
    obj = keep_join(name, keep)
    origin = origin_fn(obj)
    shift(obj, origin, scale)
    report(name, obj, origin * scale, ref * scale)
    export(os.path.join(out_dir, name + ".fbx"), obj)


def in_group(prefix):
    return lambda o: any(n.startswith(prefix) for n in group_of(o))


def bottom_centre(obj):
    lo, hi = bounds(obj)
    return Vector(((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0, lo.z))


def centre(obj):
    lo, hi = bounds(obj)
    return (lo + hi) / 2.0


def body(src, out_dir, name, keep, scale=1.0):
    """The non-moving part, with the origin of the whole model (bottom centre)."""
    load(src)
    ref = model_origin([o for o in bpy.context.scene.objects if o.type == "MESH"])
    obj = keep_join(name, keep)
    shift(obj, ref, scale)
    report(name, obj, ref, ref)
    export(os.path.join(out_dir, name + ".fbx"), obj)


def model_origin(meshes):
    """Ground (lowest point), centred: for the pod that is between the axles."""
    lo, hi = world_bounds(meshes)
    return Vector(((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0, lo.z))


def in_group(prefix):
    return lambda o: any(n.startswith(prefix) for n in group_of(o))


MOVING = ("Jack_", "Roof_HatchDoor_", "SidePort_Door", "SidePort_Stage", "SidePort_Carrier")
SIDEPORT = {"PodSideDoor": "SidePort_Door", "PodSlideStage1": "SidePort_Stage1",
            "PodSlideStage2": "SidePort_Stage2", "PodSlideStage3": "SidePort_Stage3",
            "PodSlideCarrier": "SidePort_Carrier"}
# Prius bone -> the model's wheel group (exact names: a prefix would pull in
# the steering knuckle, which must not spin) / プリウスのボーン → 車輪グループ
WHEEL_BONES = {"Wheel_Front_Left": "Wheel_FrontLeft", "Wheel_Front_Right": "Wheel_FrontRight",
               "Wheel_Rear_Left": "Wheel_RearLeft", "Wheel_Rear_Right": "Wheel_RearRight"}


def is_moving(o):
    return any(in_group(p)(o) for p in MOVING)


def prepare_pod(src, out_dir):
    moving = [in_group(p) for p in MOVING]
    body(src, out_dir, "Pod", lambda o: not any(m(o) for m in moving))

    def hinge(front):
        def f(obj):
            lo, hi = bounds(obj)
            return Vector((hi.x if front else lo.x, (lo.y + hi.y) / 2.0, lo.z))
        return f

    part(src, out_dir, "PodJack", in_group("Jack_FrontLeft"), bottom_centre, model_origin)
    for jack in ("Jack_FrontRight", "Jack_RearLeft", "Jack_RearRight"):  # pivots only
        load(src)
        ref = model_origin([o for o in bpy.context.scene.objects if o.type == "MESH"])
        obj = keep_join("PodJack", in_group(jack))
        report("  " + jack, obj, bottom_centre(obj), ref)
    part(src, out_dir, "PodHatchFront", in_group("Roof_HatchDoor_Front"), hinge(True),
         model_origin)
    part(src, out_dir, "PodHatchRear", in_group("Roof_HatchDoor_Rear"), hinge(False),
         model_origin)
    load(src)
    if any(in_group("SidePort_Door")(o) for o in bpy.context.scene.objects if o.type == "MESH"):
        for name, group in SIDEPORT.items():
            part(src, out_dir, name, in_group(group), bottom_centre, model_origin)


def in_group_exact(name):
    return lambda o: name in group_of(o)[:1]


def prepare_vehicle(src, out_dir):
    """PodVehicle.fbx: body + wheels skinned to a Prius-style armature.

    The armature object is named Vehicle_Base, so Unreal imports it as the
    root bone; each wheel's tyre and rim go to its wheel bone (bone at the
    wheel centre, pointing +y like the Prius's). Everything else -- moving
    parts excluded, they stay props -- is unweighted, i.e. on the root.
    アーマチュア名 Vehicle_Base が Unreal のルートボーンになる。各車輪のタイヤと
    ホイールは車輪ボーンに、それ以外はルートに付く（可動部はプロップのまま）。
    """
    load(src)
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    ref = model_origin(meshes)
    wheel_of = {}
    for o in meshes:
        for bone, group in WHEEL_BONES.items():
            if in_group_exact(group)(o):
                wheel_of[o] = bone
    keep = [o for o in meshes if not is_moving(o)]
    for o in keep:
        o.data = o.data.copy()
        o.data.transform(o.matrix_world)
    for o in [o for o in bpy.context.scene.objects if o not in keep]:
        bpy.data.objects.remove(o)
    for o in keep:
        o.parent = None
        o.matrix_world = Matrix.Identity(4)
        o.data.transform(Matrix.Translation(-ref))
        if o in wheel_of:
            vg = o.vertex_groups.new(name=wheel_of[o])
            vg.add(list(range(len(o.data.vertices))), 1.0, "REPLACE")
    centres = {}
    for bone in WHEEL_BONES:
        vs = [v.co for o in keep if wheel_of.get(o) == bone for v in o.data.vertices]
        lo = Vector([min(v[i] for v in vs) for i in range(3)])
        hi = Vector([max(v[i] for v in vs) for i in range(3)])
        centres[bone] = (lo + hi) / 2.0
    bpy.ops.object.select_all(action="DESELECT")
    for o in keep:
        o.select_set(True)
    bpy.context.view_layer.objects.active = keep[0]
    bpy.ops.object.join()
    mesh = bpy.context.view_layer.objects.active
    mesh.name = mesh.data.name = "PodVehicle"
    # work in centimetres (1 Blender unit = 1 cm), so the FBX needs no unit
    # conversion: Blender otherwise writes it as a 100x scale on the armature,
    # which Unreal keeps on the root bone and applies to the physics bodies
    # (the pod then "collides" everywhere and never spawns)
    # センチメートルで作業し FBX の単位変換を無くす（さもないとルートボーンに
    # 100 倍のスケールが付き、物理形状も 100 倍になってスポーンできない）
    bpy.context.scene.unit_settings.scale_length = 0.01
    mesh.data.transform(Matrix.Scale(100.0, 4))
    centres = {b: c * 100.0 for b, c in centres.items()}
    arm = bpy.data.objects.new("Vehicle_Base", bpy.data.armatures.new("Vehicle_Base"))
    bpy.context.scene.collection.objects.link(arm)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    for bone, c in centres.items():
        b = arm.data.edit_bones.new(bone)
        b.head, b.tail = c, c + Vector((0.0, 1.0, 0.0))     # 1 cm along +y, like the Prius's
    bpy.ops.object.mode_set(mode="OBJECT")
    mesh.parent = arm
    mesh.modifiers.new("Armature", "ARMATURE").object = arm
    for bone, c in centres.items():
        print("  %-17s bone at x=%.1f y=%.1f z=%.1f cm (blender)" % (bone, c.x, c.y, c.z))
    bpy.ops.object.select_all(action="DESELECT")
    mesh.select_set(True)
    arm.select_set(True)
    bpy.ops.export_scene.fbx(filepath=os.path.join(out_dir, "PodVehicle.fbx"),
                             use_selection=True, object_types={"ARMATURE", "MESH"},
                             apply_unit_scale=True, bake_space_transform=False,
                             mesh_smooth_type="FACE", path_mode="STRIP", add_leaf_bones=False,
                             bake_anim=False, armature_nodetype="NULL",
                             use_armature_deform_only=False)
    print("PodVehicle     %d verts, %d wheel bones" % (len(mesh.data.vertices), len(centres)))


def prepare_drone(src, out_dir):
    rotor = in_group("Drone_D1_Prop")
    body(src, out_dir, "Drone", lambda o: not rotor(o), DRONE_SCALE)
    for i in (1, 2, 3, 4):   # report every hub; the blades are identical
        load(src)
        ref = model_origin([o for o in bpy.context.scene.objects if o.type == "MESH"])
        obj = keep_join("DroneRotor", in_group("Drone_D1_Prop%d" % i))
        c = centre(obj)
        report("  rotor%d" % i, obj, c * DRONE_SCALE, ref * DRONE_SCALE)
        if i == 1:
            shift(obj, c, DRONE_SCALE)
            export(os.path.join(out_dir, "DroneRotor.fbx"), obj)


def prepare_parcel(out_dir):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sx, sy, sz = PARCEL_SIZE
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.0, 0.0, sz / 2.0))
    box = bpy.context.view_layer.objects.active
    box.scale = (sx, sy, sz)
    bpy.ops.object.transform_apply(scale=True)
    box.name = box.data.name = "Parcel"
    mat = bpy.data.materials.new("M_cardboard")
    mat.use_nodes = True
    mat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = PARCEL_COLOR
    box.data.materials.append(mat)
    print("Parcel         size %.3f x %.3f x %.3f m" % PARCEL_SIZE)
    export(os.path.join(out_dir, "Parcel.fbx"), box)


if __name__ == "__main__":
    pod_src, drone_src, out_dir = sys.argv[sys.argv.index("--") + 1:][:3]
    os.makedirs(out_dir, exist_ok=True)
    prepare_pod(pod_src, out_dir)
    prepare_vehicle(pod_src, out_dir)
    prepare_drone(drone_src, out_dir)
    prepare_parcel(out_dir)
