"""Split LastMilePod.fbx (the LMP-4 pod of LastMile_Pod_3D.html) into CARLA props (Blender).

The export (README_Unreal.md next to it) has every part as its own named node,
in metres, in its parked rest pose, with the geometry already in place (all
node transforms are identity). This script makes:

  LMPVehicle.fbx   the drivable vehicle: everything that never moves and is
                   not outer shell, wheels skinned to Prius-style wheel bones
                   (root Vehicle_Base), built in centimetres like PodVehicle
  LMPShell.fbx / LMPShellStreet.fbx
                   the outer shell (panels, ribs, roof panels, livery) and the
                   street-side wall, apart from the vehicle so the X-ray /
                   cutaway / frame modes can swap or hide them
  LMP<Part>.fbx    one prop per moving part, its origin on its pivot

and lmp_parts.json: for every prop, the pivot of each instance in the pod
frame (CARLA axes: x forward, y right, z up, metres, origin on the ground
between the axles -- the HTML's origin). The runtime (delivery/lmp_rig.py)
poses parts as rest pivot + the HTML animation's offset from its rest pose.

Blender (x, y, z) arrives in CARLA as (x, -y, z), so the HTML's (x, y up, z)
is CARLA's (x, z, y): the numbers in the HTML scripts carry over directly.

Run headless:
  blender -b -P prepare_lastmile.py -- <LastMilePod.fbx> <Drone.fbx> <BoxStation.fbx> <out_dir>

LastMilePod.fbx を CARLA 用プロップに分割する（Blender）。車両本体・外殻
（X 線表示用）・可動部品ごとのプロップと、各部品の支点を lmp_parts.json に出力。
"""

import json
import os
import sys

import bpy
from mathutils import Matrix, Vector

# the HTML's X-ray mode fades these: shell panels, ribs, roof panels, livery
# HTML の X 線モードで透明になるマテリアル
SHELL_MATS = ("M_Standard_e4e8ec", "M_Standard_c9cfd5", "M_Printed_1")
SHELL_NODES = ("Pod_Body", "Pod_Roof")
STREET_WALL = "Pod_StreetSideWall"
WHEEL_BONES = {"Wheel_Front_Left": "Wheel_FrontLeft", "Wheel_Front_Right": "Wheel_FrontRight",
               "Wheel_Rear_Left": "Wheel_RearLeft", "Wheel_Rear_Right": "Wheel_RearRight"}

# prop -> (nodes whose meshes make it, pivot rule, instances: node -> pivot rule)
# pivot rules: "bc" bottom centre, "c" centre, "top" top centre, "origin" (0,0,0),
# "hinge_top" top edge centre, "hinge_left" left (-x) edge centre
POD_PARTS = {
    "LMPHatchFront": (["Roof_HatchDoor_Front"], "bc"),
    "LMPHatchRear": (["Roof_HatchDoor_Rear"], "bc"),
    "LMPJack": (["Jack_FrontLeft"], "bc",
                {"Jack_FrontRight": "bc", "Jack_FrontLeft": "bc",
                 "Jack_RearRight": "bc", "Jack_RearLeft": "bc"}),
    "LMPLiftPad": (["Lift_Pad"], "bc"),
    "LMPScissorA": (["Lift_ScissorA_0"], "c", {"Lift_ScissorA_%d" % i: "c" for i in range(6)}),
    "LMPScissorB": (["Lift_ScissorB_0"], "c", {"Lift_ScissorB_%d" % i: "c" for i in range(6)}),
    "LMPLiftPin": (["Lift_Pin_0"], "c", {"Lift_Pin_%d" % i: "c" for i in range(6)}),
    "LMPShuttleMast": (["Shuttle_Mast"], "bc"),
    "LMPShuttleCarriage": (["Shuttle_Carriage"], "bc"),
    "LMPShuttleFork1": (["Shuttle_Fork1"], "bc"),
    "LMPShuttleFork2": (["Shuttle_Fork2"], "bc"),
    "LMPPresenterTable": (["Presenter_Table"], "bc"),
    "LMPSwapperCarrier": (["Swapper_Carrier"], "bc"),
    "LMPSwapperBoom": (["Swapper_Boom"], "bc"),
    "LMPSwapperJaw": (["Swapper_Jaw"], "bc"),
    "LMPBatteryMagazine": (["Battery_Magazine"], "bc"),
    "LMPBattery": (["Battery_B"], "c", {"Battery_A": "c", "Battery_B": "c"}),
    "LMPHoistBridge": (["Hoist_Bridge"], "bc"),
    "LMPHoistGripper": (["Hoist_Gripper"], "bc"),
    "LMPTray": (["Tray_F-0.2-1.19"], "bc", {"Tray_F-0.2-1.19": "bc", "Tray_F-0.7-0.86": "bc"}),
    "LMPParcelM": (["Parcel_F-0.2-1.19_M"], "bc"),
    "LMPParcelXL": (["Parcel_F-0.7-0.86_XL"], "bc"),
    "LMPPortDoor": (["SidePort_Door"], "bc"),
    "LMPPortStage1": (["SidePort_Stage1"], "bc"),
    "LMPPortStage2": (["SidePort_Stage2"], "bc"),
    "LMPPortStage3": (["SidePort_Stage3"], "bc"),
    "LMPPortCarrier": (["SidePort_Carrier"], "bc"),
}
# things in the pod export that are props elsewhere (not part of the vehicle)
# 車両本体に含めないもの（別プロップ）
NOT_VEHICLE = tuple(n for v in POD_PARTS.values() for n in v[0]) + (
    "Jack_FrontRight", "Jack_RearRight", "Jack_RearLeft",
    "Lift_ScissorA_", "Lift_ScissorB_", "Lift_Pin_", "Battery_A", "Tray_F-0.7-0.86",
    "Drone_D1", "Drone_D2", "Drone_D3", "Drone_D4", STREET_WALL)
DRONE_PARTS = {
    "LMPDrone": (None, "origin"),            # everything but props and hook
    "LMPDroneProp": (["Drone_D1_Prop1"], "c",
                     {"Drone_D1_Prop%d" % i: "c" for i in (1, 2, 3, 4)}),
    "LMPDroneHook": (["Drone_D1_Hook"], "top"),
}
STATION_PARTS = {
    "LMPStation": (None, "origin"),          # everything but the flap and door C08
    "LMPStationFlap": (["BoxStation_IntakeFlap"], "hinge_top"),
    # the compartment the HTML uses (column 1, row 2) / HTML が使う区画
    "LMPStationDoor": (["BoxStation_Door_C08"], "hinge_left"),
}

OUT = {}


def load(path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=path)
    return [o for o in bpy.context.scene.objects if o.type == "MESH"]


def ancestors(o):
    out, p = [], o.parent
    while p is not None:
        out.append(p.name)
        p = p.parent
    return out


def under(o, node):
    """``o`` belongs to node ``node``: an exact name, or a prefix when it ends
    in '_' or '*' (``Lift_ScissorA_`` = every scissor arm A)."""
    names = ancestors(o)
    if node.endswith(("_", "*")):
        return any(n.startswith(node.rstrip("*")) for n in names)
    return node in names


PART_NODES = set()       # every node a prop is made of / プロップ元ノード全て


def own(o, node):
    """``o`` is ``node``'s own mesh: under it, but not under another part node
    nested in it (the shuttle mast does not take the carriage and forks).
    ノード自身のメッシュ（入れ子の別部品は除く）。"""
    names = ancestors(o)
    if node not in names:
        return False
    return not any(n in PART_NODES for n in names[:names.index(node)])


def mat_names(o):
    return [s.material.name for s in o.material_slots if s.material]


def is_shell(o):
    return bool(ancestors(o)[:1]) and ancestors(o)[0] in SHELL_NODES and \
        any(m.startswith(SHELL_MATS) for m in mat_names(o))


def world_verts(objs):
    for o in objs:
        mw = o.matrix_world
        for v in o.data.vertices:
            yield mw @ v.co


def bounds(objs):
    vs = list(world_verts(objs))
    lo = Vector([min(v[i] for v in vs) for i in range(3)])
    hi = Vector([max(v[i] for v in vs) for i in range(3)])
    return lo, hi


def pivot(objs, rule):
    if rule == "origin":
        return Vector((0.0, 0.0, 0.0))
    lo, hi = bounds(objs)
    c = (lo + hi) / 2.0
    return {"bc": Vector((c.x, c.y, lo.z)), "c": c, "top": Vector((c.x, c.y, hi.z)),
            "hinge_top": Vector((c.x, c.y, hi.z)), "hinge_left": Vector((lo.x, c.y, c.z))}[rule]


def carla(v):
    """Blender -> CARLA pod frame (y flips). / Blender 座標を CARLA に。"""
    return [round(v.x, 5), round(-v.y, 5), round(v.z, 5)]


def baked_copy(objs, name, origin):
    """One joined mesh object of copies of ``objs``, baked and shifted to ``origin``."""
    copies = []
    for o in objs:
        c = o.copy()
        c.data = o.data.copy()
        bpy.context.scene.collection.objects.link(c)
        c.data.transform(o.matrix_world)
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
    obj.data.transform(Matrix.Translation(-origin))
    return obj


def export_obj(obj, out_dir):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.export_scene.fbx(filepath=os.path.join(out_dir, obj.name + ".fbx"),
                             use_selection=True, object_types={"MESH"}, apply_unit_scale=True,
                             bake_space_transform=False, mesh_smooth_type="FACE",
                             path_mode="COPY", embed_textures=True)
    bpy.data.objects.remove(obj)


def make_parts(meshes, parts, out_dir, rest=None):
    """Export each prop of ``parts``; ``rest`` picks the meshes of a None entry."""
    for name, spec in parts.items():
        nodes, rule = spec[0], spec[1]
        objs = rest if nodes is None else [o for o in meshes if any(own(o, n) for n in nodes)]
        if not objs:
            raise RuntimeError("no meshes for %s (%s)" % (name, nodes))
        origin = pivot(objs, rule)
        lo, hi = bounds(objs)
        instances = {}
        for node, irule in (spec[2] if len(spec) > 2 else {(nodes or [name])[0]: rule}).items():
            inst = objs if node == (nodes or [name])[0] else [o for o in meshes if own(o, node)]
            instances[node] = carla(pivot(inst, irule))
        OUT[name] = {"size": [round(x, 4) for x in (hi - lo)], "instances": instances,
                     "materials": len({m for o in objs for m in mat_names(o)})}
        export_obj(baked_copy(objs, name, origin), out_dir)
        print("%-20s size %.3f x %.3f x %.3f  pivot %s  x%d" % (
            (name,) + tuple(hi - lo) + (carla(origin), len(instances))))


def make_vehicle(meshes, out_dir):
    """LMPVehicle.fbx: the static pod on a Prius-style armature, in centimetres.

    Built in cm (1 Blender unit = 1 cm) so the FBX needs no unit conversion,
    which Unreal would keep as a 100x scale on the root bone (see
    prepare_models.prepare_vehicle). / 車両本体（cm 単位、プリウス型骨格）。
    """
    keep = [o for o in meshes if not is_shell(o) and not any(under(o, n) for n in NOT_VEHICLE)]
    wheel_of = {}
    for o in keep:
        parent = ancestors(o)[:1]
        for bone, group in WHEEL_BONES.items():
            if parent == [group]:
                wheel_of[o] = bone
    n_mats = len({m for o in keep for m in mat_names(o)})
    centres = {}
    for bone in WHEEL_BONES:
        lo, hi = bounds([o for o in keep if wheel_of.get(o) == bone])
        centres[bone] = (lo + hi) / 2.0
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
    mesh.name = mesh.data.name = "LMPVehicle"
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
    bpy.ops.export_scene.fbx(filepath=os.path.join(out_dir, "LMPVehicle.fbx"),
                             use_selection=True, object_types={"ARMATURE", "MESH"},
                             apply_unit_scale=True, bake_space_transform=False,
                             mesh_smooth_type="FACE", path_mode="COPY", embed_textures=True,
                             add_leaf_bones=False, bake_anim=False, armature_nodetype="NULL",
                             use_armature_deform_only=False)
    OUT["LMPVehicle"] = {"wheels": {b: carla(c) for b, c in centres.items()},
                         "materials": n_mats}
    print("LMPVehicle           %d verts, %d materials, wheels %s" % (
        len(mesh.data.vertices), OUT["LMPVehicle"]["materials"],
        {b: carla(c) for b, c in centres.items()}))


def main():
    pod_src, drone_src, station_src, out_dir = sys.argv[sys.argv.index("--") + 1:][:4]
    for parts in (POD_PARTS, DRONE_PARTS, STATION_PARTS):
        for spec in parts.values():
            PART_NODES.update(spec[0] or [])
            PART_NODES.update(spec[2] if len(spec) > 2 else {})
    os.makedirs(out_dir, exist_ok=True)

    meshes = load(pod_src)
    shell = [o for o in meshes if is_shell(o)]
    street = [o for o in meshes if under(o, STREET_WALL)]
    make_parts(meshes, {"LMPShell": (None, "origin")}, out_dir, rest=shell)
    make_parts(meshes, {"LMPShellStreet": (None, "origin")}, out_dir, rest=street)
    OUT["LMPShell"]["shell_materials"] = list(SHELL_MATS)
    make_parts(meshes, POD_PARTS, out_dir)
    make_vehicle(load(pod_src), out_dir)

    meshes = load(drone_src)
    moving = [o for o in meshes if under(o, "Drone_D1_Prop*") or under(o, "Drone_D1_Hook")]
    make_parts(meshes, DRONE_PARTS, out_dir, rest=[o for o in meshes if o not in moving])

    meshes = load(station_src)
    moving = [o for o in meshes if under(o, "BoxStation_IntakeFlap") or
              under(o, "BoxStation_Door_C08")]
    make_parts(meshes, STATION_PARTS, out_dir, rest=[o for o in meshes if o not in moving])

    with open(os.path.join(out_dir, "lmp_parts.json"), "w") as f:
        json.dump(OUT, f, indent=1, sort_keys=True)
    print("wrote %d props + lmp_parts.json" % len(OUT))


if __name__ == "__main__":
    main()
