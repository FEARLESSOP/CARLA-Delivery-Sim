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
                     all in CARLA axes (x forward, y right, z up, metres),
                     plus the blinker / mirror slots for make_lights.py

"blinkers" / "mirrors" rules give indicator lenses <Name>_BlinkerL/_BlinkerR
(side by CARLA y) and side-mirror glass <Name>_MirrorGlass (select_faces).

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


def wheel_frame(objs, wheels, steer=None):
    """Wheel groups by quadrant, their centres, the wheel radius and the
    vehicle origin (on the ground, between the axles, on the centre line).
    車輪のグループ・中心・半径と車両原点。"""
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
    return groups, centres, radius, origin


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


def islands(o, faces, weld=0.002):
    """Connected pieces of ``faces`` (polygon indices of ``o``). Vertices closer
    than ``weld`` count as shared: OBJ parts often have split vertices.
    面を連結部品に分ける（近接頂点は共有とみなす）。"""
    key = [tuple(int(round(c / weld)) for c in v.co) for v in o.data.vertices]
    parent = {f: f for f in faces}

    def root(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    owner = {}
    for f in faces:
        for vi in o.data.polygons[f].vertices:
            k = key[vi]
            if k in owner:
                ra, rb = root(f), root(owner[k])
                if ra != rb:
                    parent[ra] = rb
            else:
                owner[k] = f
    out = {}
    for f in faces:
        out.setdefault(root(f), []).append(f)
    return list(out.values())


_pixels = {}


def face_colour(o, faces):
    """Mean base colour of ``faces``: the base-colour texture sampled at the
    face corners and centres, or the material's base colour.
    面の平均色（テクスチャを UV で読む、無ければ基本色）。"""
    me, cols = o.data, []
    uv = me.uv_layers.active
    for f in faces:
        p = me.polygons[f]
        mat = me.materials[p.material_index] if p.material_index < len(me.materials) else None
        img, base = None, (1.0, 1.0, 1.0)
        if mat and mat.use_nodes:
            for n in mat.node_tree.nodes:
                if n.type == "BSDF_PRINCIPLED":
                    sock = n.inputs["Base Color"]
                    base = tuple(sock.default_value[:3])
                    if sock.is_linked and sock.links[0].from_node.type == "TEX_IMAGE":
                        img = sock.links[0].from_node.image
                    break
        if img is None or uv is None or not img.size[0]:
            cols.append(base)
            continue
        if img.name not in _pixels:
            import numpy as np     # bundled with Blender / Blender 同梱
            px = np.empty(img.size[0] * img.size[1] * 4, dtype=np.float32)
            img.pixels.foreach_get(px)
            _pixels[img.name] = (img.size[0], img.size[1], px)
        w, h, px = _pixels[img.name]
        uvs = [uv.data[li].uv for li in p.loop_indices]
        uvs.append(sum(uvs, Vector((0.0, 0.0))) / len(uvs))
        for t in uvs:
            i = (int((t.y % 1.0) * (h - 1)) * w + int((t.x % 1.0) * (w - 1))) * 4
            cols.append(tuple(float(c) for c in px[i:i + 3]))
    return [sum(c[i] for c in cols) / len(cols) for i in range(3)]


def tinted(rgb, tint):
    """Colour test for "tint" rules / 色の判定: amber = orange-yellow lens."""
    r, g, b = rgb
    if tint == "amber":
        return r > 0.2 and 0.2 <= g / r <= 0.85 and b <= 0.5 * r
    raise SystemExit("unknown tint: %s" % tint)


def select_faces(objs, specs, what):
    """Face groups chosen by the cars.json "blinkers" / "mirrors" rules, in the
    final vehicle frame (CARLA axes, metres: x forward from mid-axle, y right,
    z up from the ground; the frame of the CAR line). Each rule:
      name / objects / materials   which objects (as for wheels; all if none)
      material  [names]            only faces with these materials
      per_face  true               judge single faces, not connected pieces
      box  {x|y|abs_y|z: [lo, hi]} the piece's (or face's) centre must be inside
      tint "amber"                 the piece's colour must be amber
      side "left"|"right"          blinkers only: force the side (else by y)
      exclude [object name parts]  skip these objects (Dawn: tail lenses stay
                                   out of the invisible "lenses")
    Returns [(object, faces, centre, rule)]; a rule matching nothing stops.
    cars.json のルールで面を選ぶ（最終座標系：x 前、y 右、z 上、m）。"""
    if isinstance(specs, dict):
        specs = [specs]
    out = []
    for spec in specs:
        want = {m.lower() for m in spec.get("material", [])}
        box, n = spec.get("box", {}), 0
        for o in objs:
            if any(k in spec for k in ("name", "objects", "materials")) and not matches(o, spec):
                continue
            if any(k.lower() in o.name.lower() for k in spec.get("exclude", [])):
                continue
            mats = o.data.materials
            faces = [p.index for p in o.data.polygons
                     if not want or (p.material_index < len(mats) and mats[p.material_index]
                                     and mats[p.material_index].name.lower() in want)]
            if not faces:
                continue
            for group in ([[f] for f in faces] if spec.get("per_face") else islands(o, faces)):
                vs = [o.data.vertices[vi].co for f in group for vi in o.data.polygons[f].vertices]
                lo_ = Vector([min(v[i] for v in vs) for i in range(3)])
                hi_ = Vector([max(v[i] for v in vs) for i in range(3)])
                c = carla((lo_ + hi_) / 2.0)
                val = {"x": c[0], "y": c[1], "abs_y": abs(c[1]), "z": c[2]}
                if not all(r[0] <= val[k] <= r[1] for k, r in box.items()):
                    continue
                if "tint" in spec and not tinted(face_colour(o, group), spec["tint"]):
                    continue
                out.append((o, group, c, spec))
                n += 1
        if not n:
            raise SystemExit("%s: rule matched no faces: %s" % (what, spec))
    return out


def set_material(o, faces, mat):
    """Give ``faces`` of ``o`` the material ``mat`` / 面にマテリアルを割り当てる"""
    if mat.name not in [m.name for m in o.data.materials if m]:
        o.data.materials.append(mat)
    idx = [i for i, m in enumerate(o.data.materials) if m and m.name == mat.name][0]
    for f in faces:
        o.data.polygons[f].material_index = idx


def flat_material(name, rgb, metallic=0.0, roughness=0.5):
    """A plain material; make_lights.py (UE) swaps it by slot name.
    単色マテリアル（UE 側の make_lights.py がスロット名で差し替える）。"""
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    mat.diffuse_color = (rgb[0], rgb[1], rgb[2], 1.0)
    bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (rgb[0], rgb[1], rgb[2], 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def drop_objects(objs, rules):
    """cars.json "drop": object rules for helper meshes that are not part of
    the car (Sketchfab's "Icosphere" around the Dawn). / 車以外の補助メッシュを除く"""
    if isinstance(rules, dict):
        rules = [rules]
    gone = [o for o in objs if any(matches(o, r) for r in rules)]
    if not gone:
        raise SystemExit("drop: rule matched no objects: %s" % rules)
    print("drop: %s" % [o.name for o in gone])
    for o in gone:
        bpy.data.objects.remove(o)
    return [o for o in bpy.context.scene.objects if o.type == "MESH"]


def split_objects(objs):
    """cars.json "split": true. A model that arrives as one mesh (S800) is cut
    into one object per material and connected piece, named
    <object>_<material>_<n>, so the wheel and steering-wheel rules can pick
    parts. / 1 メッシュのモデルをマテリアル・連結部品ごとに分ける"""
    out = []
    for o in objs:
        base = o.name
        bpy.ops.object.select_all(action="DESELECT")
        o.select_set(True)
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.separate(type="MATERIAL")
        bpy.ops.object.mode_set(mode="OBJECT")
        for part in [p for p in bpy.context.selected_objects if p.type == "MESH"]:
            bpy.ops.object.select_all(action="DESELECT")
            part.select_set(True)
            bpy.context.view_layer.objects.active = part
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_all(action="SELECT")
            bpy.ops.mesh.separate(type="LOOSE")
            bpy.ops.object.mode_set(mode="OBJECT")
            for i, q in enumerate([p for p in bpy.context.selected_objects if p.type == "MESH"]):
                used = {p.material_index for p in q.data.polygons}
                keep = [q.data.materials[k] for k in sorted(used) if k < len(q.data.materials)]
                remap = {k: j for j, k in enumerate(sorted(used))}
                idx = [remap[p.material_index] for p in q.data.polygons]
                q.data.materials.clear()
                for m in keep:
                    q.data.materials.append(m)
                for p, k in zip(q.data.polygons, idx):
                    p.material_index = k
                q.name = q.data.name = "%s_%s_%d" % (base, keep[0].name if keep and keep[0] else "none", i)
                out.append(q)
    print("split: %d objects into %d" % (len(objs), len(out)))
    return [o for o in bpy.context.scene.objects if o.type == "MESH" and o.data.vertices]


def bake_vertex_colours(objs):
    """cars.json "vertex_colour_base": true, for game rips (the FM7 Dawn):
    - materials coloured by vertex colour (the FBX/UE material import ignores
      it: white or grey car) get that colour as their base colour; a
      material shared by parts of different colours is copied once per
      colour (<material>_vc<n>);
    - a texture multiplied by the vertex colour is wired straight to Base
      Color;
    - a base-colour texture read through another UV map (atlas on UVMap.005)
      has those UVs copied into the first UV map for its faces, because UE
      samples UV channel 0; the same for a normal map on another UV map
      (dropped if the base colour texture needs channel 0).
    頂点カラー着色・別 UV のアトラスを UE で見える形に直す"""
    import numpy as np     # bundled with Blender / Blender 同梱

    def src_of(m):
        if not m or not m.use_nodes:
            return None, None
        bsdf = next((n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is None or not bsdf.inputs["Base Color"].is_linked:
            return bsdf, None
        return bsdf, bsdf.inputs["Base Color"].links[0].from_node

    def tex_uv(node):
        v = node.inputs["Vector"]
        if v.is_linked and v.links[0].from_node.type == "UVMAP":
            return v.links[0].from_node.uv_map
        return None
    # 1. atlas UVs into channel 0 / アトラス UV を第 1 UV へ
    moved, nm_moved, nm_dropped = 0, set(), set()
    for o in objs:
        me = o.data
        if len(me.uv_layers) < 2:
            continue
        for mi, m in enumerate(me.materials):
            bsdf, src = src_of(m)
            tex = None
            if src is not None and src.type == "TEX_IMAGE":
                tex = src
            elif src is not None and src.type in ("MIX", "MIX_RGB"):
                tex = next((i.links[0].from_node for i in src.inputs
                            if i.type == "RGBA" and i.is_linked
                            and i.links[0].from_node.type == "TEX_IMAGE"), None)
            name = tex_uv(tex) if tex else None
            # a normal map read through another UV map (the FM7 tyre tread
            # atlas on UVMap.006): UE reads it through channel 0 too, which
            # shows the atlas as blocks. Its UVs move into channel 0 when the
            # base colour has no texture of its own; otherwise the normal map
            # is dropped (flat beats wrong).
            # 別 UV のノーマルマップ：UV0 へ移す（基本色テクスチャと競合なら外す）
            nm = None
            if bsdf is not None and bsdf.inputs["Normal"].is_linked:
                nmap = bsdf.inputs["Normal"].links[0].from_node
                if nmap.type == "NORMAL_MAP" and nmap.inputs["Color"].is_linked:
                    ntex = nmap.inputs["Color"].links[0].from_node
                    if ntex.type == "TEX_IMAGE":
                        nm = tex_uv(ntex)
            if nm and nm in me.uv_layers and nm != me.uv_layers[0].name and nm != name:
                if tex is None:
                    name = nm
                    nm_moved.add(m.name)
                else:
                    m.node_tree.links.remove(bsdf.inputs["Normal"].links[0])
                    nm_dropped.add(m.name)
            if not name or name not in me.uv_layers or name == me.uv_layers[0].name:
                continue
            l0, lx = me.uv_layers[0], me.uv_layers[name]
            for p in me.polygons:
                if p.material_index == mi:
                    for li in p.loop_indices:
                        l0.data[li].uv = lx.data[li].uv
            moved += 1
    # 2. face colours per material / マテリアルごとの面の色
    groups = {}            # material name -> {key: [sum rgb, n]}
    keys = {}              # object name -> [key per face]
    for o in objs:
        me = o.data
        if not me.color_attributes:
            continue
        ca = me.color_attributes.active_color or me.color_attributes[0]
        col = np.empty(len(ca.data) * 4, dtype=np.float32)
        ca.data.foreach_get("color", col)
        col = col.reshape(-1, 4)[:, :3]
        ks = []
        for p in me.polygons:
            m = me.materials[p.material_index] if p.material_index < len(me.materials) else None
            idx = list(p.loop_indices) if ca.domain == "CORNER" else list(p.vertices)
            c = col[idx].mean(0)
            k = tuple(int(round(float(v) ** (1 / 2.2) * 24)) for v in c)
            ks.append(k)
            if m:
                g = groups.setdefault(m.name, {}).setdefault(k, [np.zeros(3), 0])
                g[0] += c
                g[1] += 1
        keys[o.name] = ks
    done, copies = [], 0
    for m in list(bpy.data.materials):
        if m.name not in groups:
            continue
        bsdf, src = src_of(m)
        if src is None:
            continue
        const = None
        if src.type in ("VERTEX_COLOR", "ATTRIBUTE"):
            const = np.ones(3)
        elif src.type in ("MIX", "MIX_RGB"):
            ins = [i for i in src.inputs if i.type == "RGBA"]
            feeds = [i.links[0].from_node if i.is_linked else None for i in ins]
            if not any(f is not None and f.type in ("VERTEX_COLOR", "ATTRIBUTE") for f in feeds):
                continue
            tex = [f for f in feeds if f is not None and f.type == "TEX_IMAGE"]
            if tex:
                m.node_tree.links.remove(bsdf.inputs["Base Color"].links[0])
                m.node_tree.links.new(tex[0].outputs["Color"], bsdf.inputs["Base Color"])
                m.node_tree.nodes.active = tex[0]
                done.append(m.name)
                continue
            k = [i for i, f in zip(ins, feeds) if f is None]
            const = np.array(k[0].default_value[:3]) if k else np.ones(3)
        else:
            continue
        m.node_tree.links.remove(bsdf.inputs["Base Color"].links[0])
        by_colour = sorted(groups[m.name].items(), key=lambda kv: -kv[1][1])
        mats = {}
        for i, (k, (sm, n)) in enumerate(by_colour):
            rgb = sm / n * const
            mm = m if i == 0 else m.copy()
            if i:
                mm.name = "%s_vc%d" % (m.name, i)
                copies += 1
            b = next(n_ for n_ in mm.node_tree.nodes if n_.type == "BSDF_PRINCIPLED")
            b.inputs["Base Color"].default_value = (float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0)
            mm.diffuse_color = (float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0)
            mats[k] = mm
        if len(mats) > 1:
            for o in objs:
                me = o.data
                if o.name not in keys:
                    continue
                mis = [i for i, x in enumerate(me.materials) if x and x.name == m.name]
                if not mis:
                    continue
                for p, k in zip(me.polygons, keys[o.name]):
                    if p.material_index in mis and mats[k] is not m:
                        set_material(o, [p.index], mats[k])
        done.append(m.name)
    print("vertex colours baked into %d materials (%d colour copies), atlas UVs moved for %d slots"
          % (len(done), copies, moved))
    print("normal maps on another UV: %d materials moved to UV 0, %d dropped (base colour texture needs UV 0)"
          % (len(nm_moved), len(nm_dropped)))


def _piece_groups(objs, spec, frame, only=None):
    """Pieces chosen by one rhd_convert rule: [(object, faces, centre,
    Blender centre)], the centre in CARLA axes of ``frame`` (the provisional
    vehicle frame). Pieces are connected islands, single faces
    ("per_face"), or every matched face of an object ("whole"). "uv_box"
    [u0, v0, u1, v1] keeps faces whose UV centre lies in that texture
    rectangle (a decal in an atlas). ``only`` {object name: faces} limits
    the faces (the mirrored ones). / rhd_convert の 1 ルールで選ぶ部品"""
    box, out = spec.get("box", {}), []
    uvb = spec.get("uv_box")
    for o in objs:
        if any(k in spec for k in ("name", "objects", "materials")) and not matches(o, spec):
            continue
        if any(k.lower() in o.name.lower() for k in spec.get("exclude", [])):
            continue
        faces = [p.index for p in o.data.polygons]
        if only is not None:
            faces = sorted(only.get(o.name, set()))
        if uvb:
            uv = o.data.uv_layers.active
            if uv is None:
                continue
            keep = []
            for f in faces:
                lis = o.data.polygons[f].loop_indices
                u = sum(uv.data[li].uv.x for li in lis) / len(lis)
                v = sum(uv.data[li].uv.y for li in lis) / len(lis)
                if uvb[0] <= u <= uvb[2] and uvb[1] <= v <= uvb[3]:
                    keep.append(f)
            faces = keep
        if not faces:
            continue
        groups = [faces] if spec.get("whole") else (
            [[f] for f in faces] if spec.get("per_face") else islands(o, faces))
        for group in groups:
            vs = [o.data.vertices[vi].co for f in group for vi in o.data.polygons[f].vertices]
            lo_ = Vector([min(v[i] for v in vs) for i in range(3)])
            hi_ = Vector([max(v[i] for v in vs) for i in range(3)])
            c = carla((lo_ + hi_) / 2.0 - frame)
            size = hi_ - lo_
            val = {"x": c[0], "y": c[1], "abs_y": abs(c[1]), "z": c[2],
                   "size": max(size)}
            if not all(r[0] <= val[k] <= r[1] for k, r in box.items()):
                continue
            out.append((o, group, c, (lo_ + hi_) / 2.0))
    return out


def mirror_faces(o, faces, y0=None, dy=None):
    """Mirror ``faces`` of ``o`` across the plane Blender y = ``y0`` (CARLA
    y -> -y about it), or with ``dy`` move them by dy along y. A mirror
    reverses the winding of those faces so they still face outwards (UE
    culls back faces). Vertices the faces share with the rest of the mesh are
    split off first, and every corner keeps its normal (mirrored with the
    face): glb/FBX custom split normals would otherwise come out wrong.
    面を y = y0 で鏡映、または dy だけ移動（巻き順・法線を保つ）"""
    import bmesh
    import numpy as np     # bundled with Blender / Blender 同梱
    me = o.data
    fs = set(faces)
    me.calc_normals_split()
    n = np.empty(len(me.loops) * 3, dtype=np.float32)
    me.loops.foreach_get("normal", n)
    n = n.reshape(-1, 3)
    if y0 is not None:
        for f in faces:
            for li in me.polygons[f].loop_indices:
                n[li, 1] = -n[li, 1]
    att = me.attributes.new("rhd_n", "FLOAT_VECTOR", "CORNER")
    att.data.foreach_set("vector", n.ravel())
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    sel = [bm.faces[i] for i in faces]
    for f in sel:                     # split shared corners / 共有頂点を分ける
        for v in list(f.verts):
            if any(g.index not in fs for g in v.link_faces):
                bmesh.utils.face_vert_separate(f, v)
    for v in {v for f in sel for v in f.verts}:
        if y0 is not None:
            v.co.y = 2.0 * y0 - v.co.y
        else:
            v.co.y += dy
    if y0 is not None:
        bmesh.ops.reverse_faces(bm, faces=sel)
    bm.to_mesh(me)
    bm.free()
    att = me.attributes["rhd_n"]
    n = np.empty(len(me.loops) * 3, dtype=np.float32)
    att.data.foreach_get("vector", n)
    me.attributes.remove(att)
    if hasattr(me, "use_auto_smooth"):
        me.use_auto_smooth = True
    me.normals_split_custom_set([tuple(v) for v in n.reshape(-1, 3)])
    me.update()


def flip_uv_faces(o, faces, mid=None):
    """Flip the UVs of ``faces`` horizontally about their own UV centre: the
    graphic on a mirrored piece reads the right way round again while the
    piece keeps its mirrored shape. / UV を島の中心で左右反転"""
    uv = o.data.uv_layers.active
    if uv is None:
        return
    loops = [li for f in faces for li in o.data.polygons[f].loop_indices]
    if mid is None:
        us = [uv.data[li].uv.x for li in loops]
        mid = (min(us) + max(us)) / 2.0
    for li in loops:
        uv.data[li].uv.x = 2.0 * mid - uv.data[li].uv.x


def rhd_convert(objs, spec, frame):
    """cars.json "rhd_convert": turn a left-hand-drive model into right-hand
    drive without mirroring the whole car (badges, plates and exterior text
    stay as they are). In the provisional vehicle frame (CARLA axes, the
    frame of the CAR line):
      mirror     [rules]  pieces mirrored across the centre line (y -> -y):
                          driver-shaped mouldings (dash top, cluster hood)
      move       [rules]  each rule's pieces moved across as one block
                          (block centre y -> -y), not mirrored: parts the
                          RHD car shares with the LHD one (steering wheel,
                          column, paddles, stalks, cluster, pedals: the
                          throttle stays right of the brake, text stays
                          readable); "axis_y" (CARLA y) is the block's
                          own centre line (the steering column), else the
                          middle of its bounds
      unmirror and flip_uv only look at faces that "mirror" moved (unless
      the rule says "any": true).
      unmirror   [rules]  pieces (in their NEW place) mirrored back about
                          their own centre: they keep the mirrored position
                          but read the right way round (gauges, screens,
                          badges, labelled switches)
      flip_uv    [rules]  pieces whose UVs are flipped about their own UV
                          centre: mirrored shape, readable graphic
    flip_uv with "uv_box" [u0, v0, u1, v1] (a decal's rectangle in an atlas)
    flips all chosen faces about the centre of their UV extent, so a decal
    cut into several quads reads right ("about": "island" flips each
    island about its own UV centre instead: separate icons).
    "exclude" [object name parts] skips objects (keep the exterior out of a
    cabin box); unmirror with "axis_y" mirrors back about that CARLA y
    instead of each piece's centre (clock hands about the dial centre).
    Rules: name / objects / materials, box {x|y|abs_y|z|size: [lo, hi]} on
    the piece centre, whole (the matched object is one piece), per_face
    (single faces: cuts a region out of a merged shell; mirror only, with a
    box symmetric about the centre line so the cut edges meet). A rule that
    matches nothing stops. Runs after the forward/scale bake and before the
    steering-wheel search. / 左ハンドルを右ハンドルに（必要な内装だけ鏡映）"""
    y0 = frame.y                      # centre line in Blender y / 中心線
    report, done, mirrored = [], {}, {}   # a face crosses once / 面は一度だけ移す
    for what in ("move", "mirror", "unmirror", "flip_uv"):
        rules = spec.get(what, [])
        if isinstance(rules, dict):
            rules = [rules]
        for rule in rules:
            only = None if what in ("move", "mirror") or rule.get("any") else mirrored
            pieces = _piece_groups(objs, rule, frame, only)
            if not pieces:
                raise SystemExit("rhd_convert %s: rule matched nothing: %s" % (what, rule))
            if what in ("mirror", "move"):
                pieces = [p for p in pieces
                          if not done.setdefault(p[0].name, set()).intersection(p[1])]
                for p in pieces:
                    done[p[0].name].update(p[1])
                if not pieces:
                    continue
            if what == "move":
                vs = [o.data.vertices[vi].co for o, faces, _, _ in pieces
                      for f in faces for vi in o.data.polygons[f].vertices]
                yb = (min(v.y for v in vs) + max(v.y for v in vs)) / 2.0
                if "axis_y" in rule:          # the block's own axis (CARLA y) / 軸
                    yb = y0 - rule["axis_y"]
                dy = 2.0 * (y0 - yb)
                for o in {o for o, _, _, _ in pieces}:
                    mirror_faces(o, [f for oo, faces, _, _ in pieces if oo is o for f in faces], dy=dy)
            per_obj = {}
            for o, faces, c, bc in pieces:
                per_obj.setdefault(o, []).append((faces, c, bc))
            for o, items in per_obj.items():
                if what == "mirror":
                    mirror_faces(o, [f for faces, _, _ in items for f in faces], y0)
                    mirrored.setdefault(o.name, set()).update(f for faces, _, _ in items for f in faces)
                elif what == "move":
                    pass
                elif what == "unmirror" and "axis_y" in rule:
                    mirror_faces(o, [f for faces, _, _ in items for f in faces], y0 - rule["axis_y"])
                elif what == "unmirror":
                    for faces, c, bc in items:
                        mirror_faces(o, faces, bc.y)
                elif rule.get("uv_box") and rule.get("about", "all") == "all":
                    flip_uv_faces(o, [f for faces, _, _ in items for f in faces])
                else:
                    for faces, c, bc in items:
                        flip_uv_faces(o, faces)
                nf = sum(len(f) for f, _, _ in items)
                cs = [c for _, c, _ in items]
                mean = [round(sum(v[i] for v in cs) / len(cs), 2) for i in range(3)]
                report.append((what, o.name, len(items), nf, mean))
    for what, name, n, nf, mean in report:
        print("RHD %s %s pieces=%d faces=%d centre=%s" % (what, name, n, nf, mean))
    return report


def steering_from_box(objs, rule, frame):
    """steering_wheel {"box": {...}, "in": [object name parts]}: the wheel
    is cut out of merged meshes as every connected piece whose centre lies
    in the box (CAR frame, as rhd_convert; "size" limits the piece size),
    from the objects named in "in" (all if missing); the pieces are joined
    into one object. For wheels that auto_in cannot judge by shape (SF90:
    flat-bottomed rim in three parts plus badge). / 箱でハンドル部品を切り出す"""
    spec = {"box": rule["box"]}
    names = rule.get("in")
    cut = []
    for o in [o for o in objs if not names or any(k.lower() in o.name.lower() for k in names)]:
        pieces = _piece_groups([o], spec, frame)
        if not pieces:
            continue
        faces = {f for _, fs, _, _ in pieces for f in fs}
        me = o.data
        vsel = {vi for f in faces for vi in me.polygons[f].vertices}
        for v in me.vertices:
            v.select = v.index in vsel
        for e in me.edges:
            e.select = e.vertices[0] in vsel and e.vertices[1] in vsel
        for p in me.polygons:
            p.select = p.index in faces
        bpy.ops.object.select_all(action="DESELECT")
        o.select_set(True)
        bpy.context.view_layer.objects.active = o
        before = set(bpy.data.objects)
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_mode(type="FACE")
        bpy.ops.mesh.separate(type="SELECTED")
        bpy.ops.object.mode_set(mode="OBJECT")
        cut += [x for x in bpy.data.objects if x not in before]
    if not cut:
        return None
    bpy.ops.object.select_all(action="DESELECT")
    for x in cut:
        x.select_set(True)
    bpy.context.view_layer.objects.active = cut[0]
    if len(cut) > 1:
        bpy.ops.object.join()
    w = bpy.context.view_layer.objects.active
    print("box steering wheel: %d pieces from %d objects" % (
        sum(1 for _ in cut), len(cut)))
    return w


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
    if cfg.get("drop"):
        objs = drop_objects(objs, cfg["drop"])
    if cfg.get("vertex_colour_base"):
        bake_vertex_colours(objs)
    if cfg.get("split"):
        objs = split_objects(objs)
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
    # left-hand-drive model made right-hand drive: only the cabin parts that
    # set the drive side move (ch. 13) / 左ハンドルを右ハンドルに（内装のみ）
    if cfg.get("rhd_convert"):
        frame = wheel_frame(objs, wheels)[3]
        rhd_convert(objs, cfg["rhd_convert"], frame)
    if "box" in cfg["steering_wheel"]:
        found = steering_from_box(objs, cfg["steering_wheel"], wheel_frame(objs, wheels)[3])
        objs = [o for o in bpy.context.scene.objects if o.type == "MESH" and o.data.vertices]
        steer = [found] if found else []
    elif "auto_in" in cfg["steering_wheel"]:
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
    keep_off = [k.lower() for k in cfg["steering_wheel"].get("exclude", [])]
    for o in objs:
        if o is steer or any(k in o.name.lower() for k in keep_off):
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
    groups, centres, radius, origin = wheel_frame(objs, wheels, steer)
    bake(objs, Matrix.Translation(-origin))
    for b in centres:
        centres[b] -= origin
    lo, hi = bounds(objs)

    # mirror glass and blinker lenses get their own materials; make_lights.py
    # (UE) swaps them by slot name for a mirror look and blinking amber
    # ミラーとウインカーに専用マテリアル（UE の make_lights.py が差し替える）
    pieces, light_slots = [], {"blinker_left": [], "blinker_right": [], "mirror": []}
    body_objs = [o for o in objs if o is not steer]
    if cfg.get("mirrors"):
        mat = flat_material(name + "_MirrorGlass", (0.9, 0.9, 0.9), metallic=1.0, roughness=0.05)
        for o, faces, c, spec in select_faces(body_objs, cfg["mirrors"], "mirrors"):
            set_material(o, faces, mat)
            pieces.append([mat.name, o.name, len(faces), c])
        light_slots["mirror"] = [mat.name]
    if cfg.get("blinkers"):
        mats = {s: flat_material(name + "_Blinker" + s[0].upper(), (1.0, 0.35, 0.0), roughness=0.3)
                for s in ("left", "right")}
        for o, faces, c, spec in select_faces(body_objs, cfg["blinkers"], "blinkers"):
            side = spec.get("side") or ("left" if c[1] < 0 else "right")
            set_material(o, faces, mats[side])
            pieces.append([mats[side].name, o.name, len(faces), c])
        for s in ("left", "right"):
            if not any(p[0] == mats[s].name for p in pieces):
                raise SystemExit("blinkers: nothing on the %s side" % s)
            light_slots["blinker_" + s] = [mats[s].name]
    # lamp lenses that come out opaque in UE (black / grey covers over the
    # lamp) join the invisible glass, after the blinkers took theirs
    # UE で不透明になるランプレンズ（黒・灰色）は透明ガラスに（ウインカー分の後）
    if cfg.get("lenses"):
        mat = flat_material(name + "_LensGlass", (0.9, 0.9, 0.9), roughness=0.05)
        for o, faces, c, spec in select_faces(body_objs, cfg["lenses"], "lenses"):
            set_material(o, faces, mat)
            pieces.append([mat.name, o.name, len(faces), c])
        glass.append(mat.name)
    summary = {}      # per slot, object and side: faces, centre / スロット・部品・左右ごと
    for slot, obj, nf, c in pieces:
        s = summary.setdefault((slot, obj, c[1] > 0, c[0] > 0), [0, Vector()])
        s[0] += nf
        s[1] += Vector(c) * nf
    for (slot, obj, _, _), (nf, c) in sorted(summary.items()):
        print("LIGHT %s %s faces=%d centre=%s" % (slot, obj, nf, [round(v / nf, 2) for v in c]))

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
        # slots make_lights.py swaps (blinking amber / mirror) / 差し替えるスロット
        "blinker_left": light_slots["blinker_left"],
        "blinker_right": light_slots["blinker_right"],
        "mirror": light_slots["mirror"],
        "light_pieces": len(pieces),
    }
    if "cameras" in cfg:
        out["cameras"] = cfg["cameras"]
    if "driver_eye" in cfg:
        out["driver_eye"] = cfg["driver_eye"]
    json.dump(out, open(os.path.join(out_dir, key + ".json"), "w"), indent=2)
    print("CAR " + json.dumps(out))


main()
