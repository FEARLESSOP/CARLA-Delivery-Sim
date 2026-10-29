"""Turn a downloaded car model into a CARLA vehicle + steering-wheel prop (Blender).

  blender -b -P prepare_car.py -- <cars.json> <car key> <out_dir>

From the car's entry in cars.json:
  <Name>Vehicle.fbx  the car on a Prius-style armature (root Vehicle_Base, four
                     wheel bones at the wheel centres, wheels skinned to them),
                     without its steering wheel, in centimetres, x forward,
                     origin on the ground between the axles
  <Name>Steer.fbx    the steering wheel alone, origin on its hub, its column
                     axis along +x, so the driving script can turn it by
                     rolling it about x (cars/car_views.py)
  <key>.json         wheel centres / radius, bounds, the steering wheel's hub
                     and column tilt, and a first guess at the driver's eye,
                     all in CARLA axes (x forward, y right, z up, metres)

Blender (x, y, z) arrives in CARLA as (x, -y, z).

ダウンロードした車モデルを CARLA 車両とハンドル用プロップに変換する（Blender）。
"""

import json
import math
import os
import re
import sys

import bpy
from mathutils import Matrix, Vector

WHEEL_BONES = ("Wheel_Front_Left", "Wheel_Front_Right", "Wheel_Rear_Left", "Wheel_Rear_Right")
FORWARD = {"+x": 0.0, "-x": 180.0, "+y": -90.0, "-y": 90.0}   # yaw about z that puts the nose on +x


def load(path, split_groups=False):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    ext = path.lower().rsplit(".", 1)[-1]
    if ext == "fbx":
        bpy.ops.import_scene.fbx(filepath=path)
    elif ext in ("glb", "gltf"):
        bpy.ops.import_scene.gltf(filepath=path)
    elif ext == "obj":
        # merged OBJs keep their parts as groups / 結合 OBJ は部品をグループで持つ
        bpy.ops.wm.obj_import(filepath=path, use_split_groups=split_groups,
                              use_split_objects=split_groups)
    else:
        raise SystemExit("unsupported model type: " + path)
    bpy.context.view_layer.update()
    # placeholder objects with no geometry (BMW's "3DWheel ...") / 空のメッシュは除く
    for o in [o for o in bpy.context.scene.objects if o.type == "MESH" and not o.data.vertices]:
        bpy.data.objects.remove(o)
    return [o for o in bpy.context.scene.objects if o.type == "MESH"]


def bake(objs, m):
    """Apply world transforms, then ``m``, into the mesh data; unparent all. / 変換を焼き込む"""
    for o in objs:
        o.data = o.data.copy()          # instanced meshes (four wheels) share data / 共有メッシュを分離
        mat = m @ o.matrix_world
        det = mat.to_3x3().determinant()
        o.data.transform(mat)
        # Inverted scales (negative determinant) invert polygon winding order; flip normals outward
        # 反転スケール（行列式が負）は面の向きが反転するため法線を反転して外向きに整える
        if det < 0:
            o.data.flip_normals()
        o.parent = None
        o.matrix_world = Matrix.Identity(4)


def bounds(objs):
    vs = [v.co for o in objs for v in o.data.vertices]
    lo = Vector((min(v[i] for v in vs) for i in range(3)))
    hi = Vector((max(v[i] for v in vs) for i in range(3)))
    return lo, hi


def matches(o, rule):
    if "name" in rule and rule["name"].lower() in o.name.lower():
        return True
    if "objects" in rule and o.name in rule["objects"]:
        return True
    mats = {m.name.lower() for m in o.data.materials if m}
    return bool("materials" in rule and mats & {m.lower() for m in rule["materials"]})


def carla(v):
    return [round(v.x, 4), round(-v.y, 4), round(v.z, 4)]


def find_steering_wheel(objs, rule, side):
    """Steering wheel merged into a bigger mesh (BMW: the interior): split the
    named objects into connected pieces and take the round, thin one in front
    of the driver's seat; returns the new object or None.
    大きなメッシュに含まれるハンドルを、連結部品に分けて形と位置で探す。"""
    lo, hi = bounds(objs)
    mid_x, mid_y, length = (lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0, hi.x - lo.x
    sign = -1.0 if side == "right" else 1.0          # Blender +y = CARLA left / Blender の +y は左
    pieces = []
    for o in [o for o in objs if any(k.lower() in o.name.lower() for k in rule["auto_in"])]:
        bpy.ops.object.select_all(action="DESELECT")
        o.select_set(True)
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.separate(type="LOOSE")
        bpy.ops.object.mode_set(mode="OBJECT")
        pieces += list(bpy.context.selected_objects)
    best = None
    for o in pieces:
        if len(o.data.vertices) < 24:
            continue
        plo, phi = bounds([o])
        d = sorted(phi - plo)
        c = (plo + phi) / 2.0
        if not (0.28 <= d[2] <= 0.5 and d[1] >= 0.75 * d[2] and d[0] <= 0.22):
            continue
        if not (0.2 <= sign * (c.y - mid_y) <= 0.65 and mid_x <= c.x <= hi.x - 0.15 * length
                and 0.5 <= c.z - lo.z <= 1.3):
            continue
        q = [(v.co - c).length for v in o.data.vertices]
        ring = sum(1 for r in q if r > 0.3 * d[2] / 2.0) / len(q)
        if best is None or ring > best[0]:
            best = (ring, o)
    return pieces, (best[1] if best else None)


def column_axis(o):
    """Steering column direction (unit, pointing forward) from the wheel's
    vertices: the normal of the rim's plane. / ハンドルのコラム軸（前向き）。"""
    vs = [v.co for v in o.data.vertices]
    c = sum(vs, Vector()) / len(vs)
    cov = [[sum((v[i] - c[i]) * (v[j] - c[j]) for v in vs) for j in range(3)] for i in range(3)]
    # smallest-variance direction by power iteration on (trace*I - cov)
    t = cov[0][0] + cov[1][1] + cov[2][2]
    a = Matrix([[t * (i == j) - cov[i][j] for j in range(3)] for i in range(3)])
    n = Vector((1.0, 0.3, 0.2))
    for _ in range(200):
        n = (a @ n).normalized()
    if n.x < 0:
        n = -n
    return n


def main():
    cfg_path, key, out_dir = sys.argv[sys.argv.index("--") + 1:][:3]
    cfg = json.load(open(cfg_path))[key]
    name = cfg["name"]
    src_path = os.path.expanduser(cfg["source"])
    if not os.path.isabs(src_path):
        src_path = os.path.join(os.path.dirname(os.path.abspath(cfg_path)), src_path)
    objs = load(src_path, cfg.get("split_groups", False))
    # names Unreal keeps as they are (no ":" etc.), so cars.json can list them
    # Unreal でそのまま残る名前にする（":" などを除く）
    for m in bpy.data.materials:
        m.name = re.sub(r"[^A-Za-z0-9_]", "_", m.name)
    # nose on +x, real size / 前を +x に、実寸に
    m = Matrix.Rotation(math.radians(FORWARD[cfg.get("forward", "+x")]), 4, "Z")
    m = Matrix.Scale(cfg.get("scale", 1.0), 4) @ m
    bake(objs, m)
    if "length_m" in cfg:
        lo, hi = bounds(objs)
        bake(objs, Matrix.Scale(cfg["length_m"] / (hi.x - lo.x), 4))
    # window panes sharing a material with lamp lenses (RX-7) get their own,
    # so only the windows turn clear / 窓とレンズが同じマテリアルなら窓だけ分ける
    glass = list(cfg.get("glass", []))
    if cfg.get("glass_objects"):
        pane = bpy.data.materials.new(name + "_WindowGlass")
        for o in objs:
            if any(k.lower() in o.name.lower() for k in cfg["glass_objects"]):
                o.data.materials.clear()
                o.data.materials.append(pane)
        glass.append(pane.name)
    wheels = [o for o in objs if matches(o, cfg["wheels"])]
    if "auto_in" in cfg["steering_wheel"]:
        pieces, found = find_steering_wheel(objs, cfg["steering_wheel"], cfg.get("drive_side", "right"))
        objs = [o for o in bpy.context.scene.objects if o.type == "MESH" and o.data.vertices]
        steer = [found] if found else []
        print("auto steering wheel:", found.name if found else None, "of", len(pieces), "pieces")
    else:
        steer = [o for o in objs if matches(o, cfg["steering_wheel"])]
    if len(steer) != 1:
        raise SystemExit("steering wheel: expected 1 object, got %s" % [o.name for o in steer])
    steer = steer[0]
    # small parts sitting on the steering wheel (horn badge, logo) turn with it
    # ハンドル上の小物（ロゴ・ホーンパッド）も一緒に回す
    slo, shi = bounds([steer])
    pad = Vector((0.02, 0.02, 0.02))
    extra = []
    for o in objs:
        if o is steer:
            continue
        olo, ohi = bounds([o])
        if all(olo[i] >= slo[i] - pad[i] and ohi[i] <= shi[i] + pad[i] for i in range(3)):
            extra.append(o)
    if extra:
        print("steering wheel + %s" % [o.name for o in extra])
        bpy.ops.object.select_all(action="DESELECT")
        for o in [steer] + extra:
            o.select_set(True)
        bpy.context.view_layer.objects.active = steer
        bpy.ops.object.join()
        objs = [o for o in objs if o not in extra]
    # wheel centres by quadrant (Blender +y = left) / 象限で前後左右を決める
    lo, hi = bounds(objs)
    mid_x, mid_y = (lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0     # models aren't always centred / 中心線
    groups = {b: [] for b in WHEEL_BONES}
    for o in wheels:
        wlo, whi = bounds([o])
        c = (wlo + whi) / 2.0
        groups["Wheel_%s_%s" % ("Front" if c.x > mid_x else "Rear", "Left" if c.y > mid_y else "Right")].append(o)
    if not all(groups.values()):
        raise SystemExit("wheels: %s" % {b: [o.name for o in g] for b, g in groups.items()})

    def centres_of(groups):
        out = {}
        for b, g in groups.items():
            wlo, whi = bounds(g)
            out[b] = (wlo + whi) / 2.0
        return out
    # what turns with each wheel (rim, tyre, disc): centred on its axle, not
    # bigger than a wheel; calipers sit off-centre and stay on the body
    # 車輪と一緒に回る部品（リム・タイヤ・ディスク）。キャリパーは車体側
    centres = centres_of(groups)
    taken = {o for g in groups.values() for o in g}
    for o in objs:
        if o in taken or o is steer:
            continue
        olo, ohi = bounds([o])
        c, size = (olo + ohi) / 2.0, max(ohi - olo)
        for b, wc in centres.items():
            if abs(c.x - wc.x) < 0.04 and abs(c.z - wc.z) < 0.04 and abs(c.y - wc.y) < 0.2 \
                    and size < 0.9:
                groups[b].append(o)
                break
    centres, radius = centres_of(groups), 0.0
    for b, g in groups.items():
        wlo, whi = bounds(g)
        radius = max(radius, (whi.z - wlo.z) / 2.0)
    # origin: on the ground, between the axles, on the centre line / 原点：車軸間の地面
    ground = min(c.z for c in centres.values()) - radius
    origin = Vector(((max(c.x for c in centres.values()) + min(c.x for c in centres.values())) / 2.0,
                     sum(c.y for c in centres.values()) / 4.0, ground))
    bake(objs, Matrix.Translation(-origin))
    for b in centres:
        centres[b] -= origin
    lo, hi = bounds(objs)

    # steering wheel prop: hub at the origin, column along +x / ハンドル：ハブ原点、コラム +x
    axis = column_axis(steer)
    slo, shi = bounds([steer])
    hub = (slo + shi) / 2.0
    pitch = math.degrees(math.atan2(axis.z, math.hypot(axis.x, axis.y)))
    yaw = math.degrees(math.atan2(axis.y, axis.x))
    to_local = (Matrix.Rotation(math.radians(pitch), 4, "Y") @
                Matrix.Rotation(math.radians(-yaw), 4, "Z") @ Matrix.Translation(-hub))
    eye = hub + Vector((-0.62, 0.0, 0.26))     # first guess; tune in car_views.json / 初期値

    def export(sel, fname, armature=None):
        bpy.ops.object.select_all(action="DESELECT")
        for o in sel:
            o.select_set(True)
        if armature:
            armature.select_set(True)
        bpy.context.view_layer.objects.active = sel[0]
        bpy.ops.export_scene.fbx(filepath=os.path.join(out_dir, fname), use_selection=True,
                                 object_types={"ARMATURE", "MESH"}, apply_unit_scale=True,
                                 bake_space_transform=False, mesh_smooth_type="FACE",
                                 path_mode="COPY", embed_textures=True, add_leaf_bones=False,
                                 bake_anim=False, armature_nodetype="NULL",
                                 use_armature_deform_only=False)

    # textures packed inside the model (glb) have no files for the FBX to
    # embed: write them out / モデル内蔵テクスチャをファイルに書き出す
    tex_dir = os.path.join(out_dir, name + "_textures")
    for img in bpy.data.images:
        if img.packed_file is not None:
            os.makedirs(tex_dir, exist_ok=True)
            safe = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in img.name)
            img.filepath_raw = os.path.join(tex_dir, safe + ".png")
            img.file_format = "PNG"
            img.save()
            img.unpack(method="REMOVE")
            img.filepath = os.path.join(tex_dir, safe + ".png")
    bpy.context.scene.unit_settings.scale_length = 0.01      # 1 unit = 1 cm / 1 単位 = 1 cm
    # steering prop / ハンドルのプロップ
    sp = steer.copy()
    sp.data = steer.data.copy()
    bpy.context.scene.collection.objects.link(sp)
    sp.data.transform(Matrix.Scale(100.0, 4) @ to_local)
    sp.name = sp.data.name = name + "Steer"
    export([sp], name + "Steer.fbx")
    bpy.data.objects.remove(sp)
    # vehicle: everything but the steering wheel / 車両：ハンドル以外
    body = [o for o in objs if o is not steer]
    for b, g in groups.items():
        for o in g:
            vg = o.vertex_groups.new(name=b)
            vg.add(list(range(len(o.data.vertices))), 1.0, "REPLACE")
    bpy.data.objects.remove(steer)
    bpy.ops.object.select_all(action="DESELECT")
    for o in body:
        o.select_set(True)
    bpy.context.view_layer.objects.active = body[0]
    bpy.ops.object.join()
    mesh = bpy.context.view_layer.objects.active
    mesh.name = mesh.data.name = name + "Vehicle"
    mesh.data.transform(Matrix.Scale(100.0, 4))
    arm = bpy.data.objects.new("Vehicle_Base", bpy.data.armatures.new("Vehicle_Base"))
    bpy.context.scene.collection.objects.link(arm)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    for b, c in centres.items():
        eb = arm.data.edit_bones.new(b)
        eb.head = c * 100.0
        eb.tail = c * 100.0 + Vector((0.0, 1.0, 0.0))
    bpy.ops.object.mode_set(mode="OBJECT")
    mesh.parent = arm
    mesh.modifiers.new("Armature", "ARMATURE").object = arm
    export([mesh], name + "Vehicle.fbx", armature=arm)

    size = hi - lo
    out = {
        "name": name, "blueprint": "vehicle.%s.%s" % (cfg["make"], cfg["model"]),
        "drive_side": cfg.get("drive_side", "right"),
        "wheels": {b: carla(c) for b, c in centres.items()},
        "wheel_radius_m": round(radius, 4), "tyre_width_m": cfg.get("tyre_width_m", 0.22),
        "bounds_extent_m": [round(size.x / 2, 4), round(size.y / 2, 4), round(size.z / 2, 4)],
        "bounds_centre_m": carla((lo + hi) / 2.0),
        "mass_kg": cfg.get("mass", 1400),
        "steering_wheel": {"prop": "static.prop." + (name + "Steer").lower(), "hub": carla(hub),
                           "pitch": round(pitch, 2), "yaw": round(-yaw, 2),
                           "diameter_m": round(max(shi - slo), 3)},
        "driver_eye": carla(eye),
        "materials": len({m.name for m in mesh.data.materials if m}),
        "glass": glass,
    }
    if "cameras" in cfg:
        out["cameras"] = cfg["cameras"]
    if "driver_eye" in cfg:
        out["driver_eye"] = cfg["driver_eye"]
    json.dump(out, open(os.path.join(out_dir, key + ".json"), "w"), indent=2)
    print("CAR " + json.dumps(out))


main()
