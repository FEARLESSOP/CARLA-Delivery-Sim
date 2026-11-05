"""Japanese maximum-speed post (最高速度, regulatory 323): our own model for the
Japan maps, used by CARLA's speed-limit actors there (guide ch. 25).

  blender -b --python osm_city/jp_sokudo.py -- <out dir> [--render]

Writes into <out dir>, per speed value V in VALUES:
  T_JpSokudo<V>.png   the plate face (drawn here, no photos),
  SM_JpSokudo<V>.fbx  white pole + 60 cm disc plate, material slots
                      0 jpk_pole (white), 1 jpk_back (grey back/rim),
                      2 jpk_face (the face, UVs 0..1 over the disc),
plus layout.json, and with --render Cycles previews (preview_*.png).
maps/ue_jp_sokudo.py imports them into /Game/<Pkg>/Sokudo and builds
BP_JpSokudo<V>, copies of CARLA's BP_SpeedLimit40 with our mesh. Names say
"Sokudo" (速度), never "sign"/"light": CARLA's importer drops meshes and
materials whose names contain those words.

The face (NPA "Traffic Lights and Road Signs", p. 7, Maximum Speed; measured
on the chart at 400 dpi): white disc, red ring from 0.67 R to 0.93 R, thin
white rim outside it, BLUE numerals 0.37 D tall and 0.49 D wide (two
digits), centred. Colours from the chart: red (236, 27, 36), blue (28, 96, 179).

Mounting = our jp_signs.py plates: white 60.5 mm pole, 60 cm plate (道路標識令
standard round size), plate bottom 1.85 m over the pavement. CARLA spawns
the actor at the OpenDRIVE signal's point at road level (signs.py: t just
past the kerb) and only raises signs to the ground on Town15, so the model
assumes the pavement 0.15 m (KERB_H) above its origin: plate bottom 2.00 m
over the origin, pole from 0.5 m below the origin.

Frame (BP local = CARLA's speed-limit BP, read in the editor): pole at the
origin, face towards FACE (+Y here; ue_jp_sokudo.py checks the stock mesh
faces the same way). Blender frame = UE (x, -y, z) in metres.

日本の最高速度標識（規制 323・白地に赤枠・青い数字）を自作する。CARLA の速度制限
アクタのメッシュだけを置き換え、速度値とトリガーは CARLA のまま。
"""

import json
import math
import os
import sys

import bpy
import bmesh
from mathutils import Matrix, Vector

VALUES = [20, 30, 40, 50, 60, 70, 80]      # 規制速度 on ordinary roads and motorways (km/h)
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
RED, BLUE, WHITE = (236, 27, 36), (28, 96, 179), (250, 250, 250)
POLE_R = 0.03025            # 60.5 mm / 白色支柱 φ60.5
KERB_H = 0.15               # pavement over the road at the signal point / 歩道の高さ
BOTTOM = 1.85               # plate bottom over the pavement (jp_signs.py) / 板下端
DIAM, PLATE_T = 0.60, 0.012
FACE = 1.0                  # +1: face towards BP +Y / 表示面の向き
PLATE_Z = KERB_H + BOTTOM + DIAM / 2
POLE_TOP = PLATE_Z + DIAM / 2 + 0.04
MATS = {"jpk_pole": (0.92, 0.92, 0.91), "jpk_back": (0.55, 0.56, 0.57), "jpk_face": (1, 1, 1)}


# -- face texture / 板面 ---------------------------------------------------------
def texture(v, S=1024):
    from PIL import Image, ImageDraw, ImageFont
    im = Image.new("RGB", (S, S), WHITE)
    d = ImageDraw.Draw(im)
    c = S / 2
    R = S / 2

    def disc(r, fill):
        d.ellipse([c - r, c - r, c + r, c + r], fill=fill)
    disc(R * 0.93, RED)
    disc(R * 0.67, WHITE)
    # numerals: drawn big, cropped to their ink, fitted into 0.49 D x 0.37 D
    # 数字: インク範囲に切り出して 0.49D x 0.37D に収める
    txt = str(v)
    f = ImageFont.truetype(FONT, 600)
    tmp = Image.new("L", (1600, 900), 0)
    ImageDraw.Draw(tmp).text((40, 40), txt, fill=255, font=f)
    ink = tmp.crop(tmp.getbbox())
    w = S * (0.49 if len(txt) == 2 else 0.56)
    h = S * 0.37
    ink = ink.resize((int(w), int(h)), Image.LANCZOS)
    im.paste(Image.new("RGB", ink.size, BLUE), (int(c - w / 2), int(c - h / 2)), ink)
    return im


# -- geometry / 形状 -------------------------------------------------------------
def ue(p):
    return Vector((p[0], -p[1], p[2]))


class Part:
    def __init__(self, name, mats):
        self.name, self.mats = name, list(mats)
        self.bm = bmesh.new()

    def _add(self, bm2, mat_of_face, uvfun=None):
        bmesh.ops.recalc_face_normals(bm2, faces=bm2.faces)
        uv2 = bm2.loops.layers.uv.verify()
        for f in bm2.faces:
            mat = mat_of_face(f)
            f.material_index = self.mats.index(mat)
            for lp in f.loops:
                co = lp.vert.co
                lp[uv2].uv = uvfun(co) if (uvfun and mat == "jpk_face") else (co.x + 0.37 * co.y, co.z + 0.61 * co.y)
        me = bpy.data.meshes.new("tmp")
        bm2.to_mesh(me)
        bm2.free()
        self.bm.from_mesh(me)
        bpy.data.meshes.remove(me)

    def cylinder_z(self, mat, z0, z1, r, seg=12, x=0.0, y=0.0):
        bm2 = bmesh.new()
        bmesh.ops.create_cone(bm2, cap_ends=True, segments=seg, radius1=r, radius2=r, depth=z1 - z0)
        bmesh.ops.translate(bm2, vec=ue((x, y, (z0 + z1) / 2)), verts=bm2.verts)
        self._add(bm2, lambda f: mat)

    def box(self, mat, lo, hi):
        bm2 = bmesh.new()
        bmesh.ops.create_cube(bm2, size=1.0)
        a, b = ue(lo), ue(hi)
        s = Vector([abs(q - p) for p, q in zip(a, b)])
        bmesh.ops.transform(bm2, matrix=Matrix.Translation((a + b) / 2) @ Matrix.Diagonal((*s, 1)), verts=bm2.verts)
        self._add(bm2, lambda f: mat)

    def plate(self, y0, y1, z, r, seg=48):
        """Disc between y0 and y1 (BP local), front cap at FACE side = face, rest = back.
        The face UVs: u to the viewer's right, v up, 0..1 over the disc."""
        bm2 = bmesh.new()
        bmesh.ops.create_cone(bm2, cap_ends=True, segments=seg, radius1=r, radius2=r, depth=abs(y1 - y0))
        rot = Matrix.Rotation(math.radians(90), 4, "X")          # cone z -> Blender -y = UE +y
        bmesh.ops.transform(bm2, matrix=Matrix.Translation(ue((0, (y0 + y1) / 2, z))) @ rot, verts=bm2.verts)
        front = ue((0, FACE, 0))
        bmesh.ops.recalc_face_normals(bm2, faces=bm2.faces)

        def mat_of(f):
            return "jpk_face" if f.normal.dot(front) > 0.99 and len(f.verts) > 4 else "jpk_back"

        # viewer facing the plate (looking along -FACE): right = +X for FACE=+Y (UE), mirrored otherwise
        sgn = 1.0 if FACE > 0 else -1.0
        self._add(bm2, mat_of, lambda co: (0.5 + sgn * co.x / (2 * r), 0.5 + (co.z - z) / (2 * r)))

    def to_object(self):
        me = bpy.data.meshes.new(self.name)
        self.bm.to_mesh(me)
        self.bm.free()
        for m in self.mats:
            me.materials.append(material(m))
        ob = bpy.data.objects.new(self.name, me)
        bpy.context.scene.collection.objects.link(ob)
        return ob


def material(name, tex=None):
    key = name if tex is None else os.path.basename(tex)
    m = bpy.data.materials.get(key)
    if m:
        return m
    m = bpy.data.materials.new(key)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*MATS[name], 1)
    p.inputs["Roughness"].default_value = 0.35 if name != "jpk_pole" else 0.45
    if tex:
        t = m.node_tree.nodes.new("ShaderNodeTexImage")
        t.image = bpy.data.images.load(tex)
        m.node_tree.links.new(t.outputs["Color"], p.inputs["Base Color"])
    return m


def build(v):
    part = Part("SM_JpSokudo%d" % v, ["jpk_pole", "jpk_back", "jpk_face"])
    part.cylinder_z("jpk_pole", -0.5, POLE_TOP, POLE_R, seg=12)
    part.cylinder_z("jpk_pole", POLE_TOP, POLE_TOP + 0.012, POLE_R + 0.004, seg=12)       # cap / 頭キャップ
    y_back = FACE * (POLE_R + 0.004)
    y_front = y_back + FACE * PLATE_T
    part.plate(min(y_back, y_front), max(y_back, y_front), PLATE_Z, DIAM / 2)
    for dz in (-0.17, 0.17):                                                             # clamp bands / 取付金具
        z = PLATE_Z + dz
        ya, yb = sorted((-FACE * (POLE_R + 0.008), y_back))
        part.box("jpk_back", (-0.045, ya, z - 0.02), (0.045, yb, z + 0.02))
    return part.to_object()


def export(obs, out):
    for ob in obs.values():
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.export_scene.fbx(filepath=os.path.join(out, ob.name + ".fbx"), use_selection=True,
                                 object_types={"MESH"}, apply_unit_scale=True, bake_space_transform=False,
                                 mesh_smooth_type="FACE")
    layout = {"values": VALUES, "meshes": sorted(o.name for o in obs.values()),
              "slots": ["jpk_pole", "jpk_back", "jpk_face"], "face": "+Y" if FACE > 0 else "-Y",
              "plate_centre_cm": [0.0, round(FACE * (POLE_R + 0.004 + PLATE_T / 2) * 100, 2), round(PLATE_Z * 100, 1)],
              "plate_diameter_cm": DIAM * 100, "plate_bottom_over_origin_cm": round((PLATE_Z - DIAM / 2) * 100, 1)}
    json.dump(layout, open(os.path.join(out, "layout.json"), "w"), indent=1)
    print("[jp_sokudo] wrote", layout["meshes"], "to", out)


def render(obs, out):
    sc = bpy.context.scene
    for i, (v, ob) in enumerate(sorted(obs.items())):
        ob.data.materials[2] = material("jpk_face", os.path.join(out, "T_JpSokudo%d.png" % v))
        ob.location = ue(((i - (len(obs) - 1) / 2) * 1.0, 0, 0))
    bpy.ops.mesh.primitive_plane_add(size=60, location=(0, 0, KERB_H))
    gm = bpy.data.materials.new("ground")
    gm.use_nodes = True
    gm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.25, 0.25, 0.24, 1)
    bpy.context.active_object.data.materials.append(gm)
    world = sc.world or bpy.data.worlds.new("w")
    sc.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.55, 0.68, 0.85, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.9
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(45), 0, math.radians(200))
    sc.collection.objects.link(sun)
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 32
    sc.cycles.use_denoising = False
    sc.render.resolution_x, sc.render.resolution_y = 960, 540
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam

    def shot(name, eye, target, lens=50):
        cam.location = ue(eye)
        cam.data.lens = lens
        cam.rotation_euler = (ue(target) - cam.location).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = os.path.join(out, "preview_%s.png" % name)
        bpy.ops.render.render(write_still=True)

    shot("row", (0, FACE * 9.0, 1.6), (0, 0, PLATE_Z - 0.4), 40)                    # all values / 全種
    shot("driver", (-3.5, FACE * 20.0, 1.25), (0, 0, PLATE_Z), 85)                 # from a car / 運転席から
    shot("side", (4.5, FACE * 2.5, 2.0), (2.0, 0, 1.6), 35)                        # mount and back / 取付
    shot("back", (0.5, -FACE * 3.0, 2.3), (0, 0, PLATE_Z), 50)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = os.path.abspath(argv[0] if argv else "build/JpSokudo")
    os.makedirs(out, exist_ok=True)
    for v in VALUES:
        texture(v).save(os.path.join(out, "T_JpSokudo%d.png" % v))
    bpy.ops.wm.read_factory_settings(use_empty=True)
    obs = {v: build(v) for v in VALUES}
    export(obs, out)
    if "--render" in argv:
        render(obs, out)


main()
