"""Japanese vehicle traffic signal (信号機): our own geometry for the Japan maps.

  blender -b --python osm_city/jp_shingo.py -- <out dir> [--render]

Writes one FBX per part into <out dir> (SM_JpShingoPole, SM_JpShingoArm,
SM_JpShingoHead, SM_JpShingoLamp) plus layout.json, and with --render
Blender previews of the whole signal in each state (green / yellow / red /
off). maps/ue_jp_shingo.py imports the parts into /Game/Japan/Shingo and
builds BP_JpShingo, a copy of CARLA's BP_TLOpenDrive_RHT with these meshes
(guide ch. 23). Names say "Shingo" (信号), never "sign"/"light": CARLA's
importer drops meshes and materials whose names contain those words.

How CARLA drives it (read from BP_TLOpenDrive_RHT, 0.9.16): the blueprint's
construction script adds one component per entry of Heads[0].Lights, puts a
dynamic instance of M_StreetlightsMaster_T10 on the mesh's material SLOT 1,
sets its LightColor by the entry's colour (green 0,1,0 / yellow 1,.745,0 /
red 1,.016,0) and files it under GreenLights / YellowLights / RedLights. On
every state change (TrafficLightComponent -> LightChanged) the graph sets
"On/Off" = 1 on the matching list and 0 on the others. So the lamp mesh needs
slot 0 = housing/hood, slot 1 = lens, and the lens UVs fill 0..1.

Frame (BP local, = CARLA's RHT model so lht_poles.py placement holds): pole
at the origin, arm along -X, head centred 6 m out, lamps facing +Y (towards
the approaching driver). From the driver's seat, left to right: green
(blue-green 青), yellow, red (道路交通法施行令 第3条; red nearest the road
centre on keep-left roads). Blender frame = UE (x, -y, z) in metres.

Sizes: LED head with 250 mm lenses (警察庁 2017 spec change, 300 -> 250 mm),
1050 x 350 mm body (thin LED type, newswitch.jp/p/10304), black hoods 200 mm
deep, grey steel pole and cantilever arm, head bottom about 5.3 m above the
road (Japanese overhead signals: 4.5 m minimum over the carriageway).

日本の車両用信号機（横型・LED・フード付き）を自作ジオメトリで生成する。
CARLA の信号ブループリントの複製に載せ、状態変化は CARLA のまま動く。
"""

import json
import math
import os
import sys

import bpy
import bmesh
from mathutils import Matrix, Vector

# ---- dimensions (m, BP local frame: x along the arm, y towards the driver, z up)
POLE_H, POLE_R0, POLE_R1 = 6.30, 0.135, 0.105        # φ270 tapering to φ210
ARM_Z, ARM_LEN, ARM_R0, ARM_R1 = 5.85, 6.55, 0.076, 0.051
HEAD_X, HEAD_Y, HEAD_Z = -6.00, 0.10, 5.52            # head centre (= Heads[0].Position)
BODY_W, BODY_H, BODY_D = 1.05, 0.35, 0.12             # slim LED body
PITCH = 0.35                                          # lamp spacing
LENS_R, HOOD_R, HOOD_T, HOOD_L = 0.125, 0.140, 0.006, 0.20
# lamp order along +x, as Heads[0].Lights: (colour, x offset)
LAMPS = [("GREEN", -PITCH), ("YELLOW", 0.0), ("RED", PITCH)]
COLOURS = {"GREEN": (0.0, 0.85, 0.55), "YELLOW": (1.0, 0.62, 0.0), "RED": (1.0, 0.03, 0.0)}
MATS = {"jps_body": (0.52, 0.53, 0.52), "jps_hood": (0.025, 0.025, 0.025), "jps_lens": (0.05, 0.05, 0.05)}


def ue(p):
    """BP-local (UE axes, metres) -> Blender coordinates."""
    return Vector((p[0], -p[1], p[2]))


class Part:
    """A mesh built from closed solids, normals made consistent per solid."""

    def __init__(self, name, mats):
        self.name, self.mats = name, list(mats)
        self.bm = bmesh.new()
        self.uv = self.bm.loops.layers.uv.new("UVMap")

    def _add(self, bm2, mat, uvfun=None):
        bmesh.ops.recalc_face_normals(bm2, faces=bm2.faces)
        uv2 = bm2.loops.layers.uv.verify()
        for f in bm2.faces:
            f.material_index = self.mats.index(mat)
            for l in f.loops:
                l[uv2].uv = uvfun(l.vert.co) if uvfun else (l.vert.co.x + 0.37 * l.vert.co.y, l.vert.co.z + 0.61 * l.vert.co.y + 0.29 * l.vert.co.x)
        me = bpy.data.meshes.new("tmp")
        bm2.to_mesh(me)
        bm2.free()
        self.bm.from_mesh(me)
        bpy.data.meshes.remove(me)

    def frustum(self, mat, a, b, r0, r1, seg=24):
        """Closed tapered tube from a to b (BP-local points)."""
        bm2 = bmesh.new()
        A, B = ue(a), ue(b)
        d = B - A
        bmesh.ops.create_cone(bm2, cap_ends=True, segments=seg, radius1=r0, radius2=r1, depth=d.length)
        rot = d.normalized().to_track_quat("Z", "Y").to_matrix().to_4x4()
        bmesh.ops.transform(bm2, matrix=Matrix.Translation((A + B) / 2) @ rot, verts=bm2.verts)
        self._add(bm2, mat)

    def box(self, mat, lo, hi):
        bm2 = bmesh.new()
        bmesh.ops.create_cube(bm2, size=1.0)
        lo_b, hi_b = ue(lo), ue(hi)
        c = (lo_b + hi_b) / 2
        s = Vector([abs(h - l) for l, h in zip(lo_b, hi_b)])
        bmesh.ops.transform(bm2, matrix=Matrix.Translation(c) @ Matrix.Diagonal((*s, 1)), verts=bm2.verts)
        self._add(bm2, mat)

    def disc(self, mat, y0, y1, r, seg=32):
        """Lens: a closed disc facing +y (BP local), UVs 0..1 across it."""
        bm2 = bmesh.new()
        bmesh.ops.create_cone(bm2, cap_ends=True, segments=seg, radius1=r, radius2=r, depth=y1 - y0)
        rot = Matrix.Rotation(math.radians(90), 4, "X")          # cone z -> Blender -y = UE +y
        bmesh.ops.transform(bm2, matrix=Matrix.Translation(ue((0, (y0 + y1) / 2, 0))) @ rot, verts=bm2.verts)
        self._add(bm2, mat, lambda co: (0.5 + co.x / (2 * r), 0.5 + co.z / (2 * r)))

    def hood(self, mat, y0, length, r, t, a0=-25.0, a1=205.0, seg=20):
        """Visor: a thin closed shell over the top of the lens, out along +y."""
        bm2 = bmesh.new()
        rings = []
        for y in (y0, y0 + length):
            ring = []
            for rr in (r, r + t):
                ring.append([bm2.verts.new(ue((rr * math.cos(math.radians(a0 + (a1 - a0) * k / seg)), y,
                                              rr * math.sin(math.radians(a0 + (a1 - a0) * k / seg)))))
                             for k in range(seg + 1)])
            rings.append(ring)
        (i0, o0), (i1, o1) = rings
        for k in range(seg):
            for q in ((i0[k], i0[k + 1], i1[k + 1], i1[k]), (o0[k], o1[k], o1[k + 1], o0[k + 1]),
                      (i0[k], o0[k], o0[k + 1], i0[k + 1]), (i1[k], i1[k + 1], o1[k + 1], o1[k])):
                bm2.faces.new(q)
        for e in (0, seg):
            bm2.faces.new((i0[e], i1[e], o1[e], o0[e]))
        self._add(bm2, mat)

    def to_object(self, origin=(0, 0, 0)):
        me = bpy.data.meshes.new(self.name)
        bmesh.ops.translate(self.bm, vec=-ue(origin), verts=self.bm.verts)
        self.bm.to_mesh(me)
        self.bm.free()
        for m in self.mats:
            me.materials.append(material(m))
        ob = bpy.data.objects.new(self.name, me)
        bpy.context.scene.collection.objects.link(ob)
        return ob


def material(name, emit=None):
    key = name if emit is None else "%s_%s" % (name, emit)
    m = bpy.data.materials.get(key)
    if m:
        return m
    m = bpy.data.materials.new(key)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*MATS[name], 1)
    p.inputs["Roughness"].default_value = 0.15 if name == "jps_lens" else 0.55
    p.inputs["Metallic"].default_value = 0.4 if name == "jps_body" else 0.0
    if emit:
        p.inputs["Emission Color"].default_value = (*COLOURS[emit], 1)
        p.inputs["Emission Strength"].default_value = 40.0
    return m


def build():
    """The four parts, each with its origin where the blueprint puts it."""
    pole = Part("SM_JpShingoPole", ["jps_body"])
    pole.frustum("jps_body", (0, 0, -0.05), (0, 0, POLE_H), POLE_R0, POLE_R1)
    pole.frustum("jps_body", (0, 0, -0.05), (0, 0, 0.30), 0.18, 0.18)          # base collar
    pole.frustum("jps_body", (0, 0, POLE_H), (0, 0, POLE_H + 0.06), 0.115, 0.06)  # cap
    pole.box("jps_body", (-0.16, -0.10, ARM_Z - 0.16), (0.0, 0.10, ARM_Z + 0.16))  # arm flange

    arm = Part("SM_JpShingoArm", ["jps_body"])
    arm.frustum("jps_body", (-0.10, 0, ARM_Z), (-ARM_LEN, 0, ARM_Z), ARM_R0, ARM_R1)
    arm.frustum("jps_body", (-0.10, 0, POLE_H - 0.10), (-2.2, 0, ARM_Z + 0.03), 0.022, 0.022)  # tie rod

    head = Part("SM_JpShingoHead", ["jps_body", "jps_hood"])
    hx, hy, hz = HEAD_X, HEAD_Y, HEAD_Z
    head.box("jps_body", (hx - BODY_W / 2, hy - BODY_D / 2, hz - BODY_H / 2),
             (hx + BODY_W / 2, hy + BODY_D / 2, hz + BODY_H / 2))
    top = ARM_Z - ARM_R1 - hz                                               # bracket up to the arm
    for sx in (-0.30, 0.30):
        head.box("jps_body", (hx + sx - 0.03, -0.06, hz + BODY_H / 2 - 0.02), (hx + sx + 0.03, hy - BODY_D / 2 + 0.01, hz + top + 0.02))
        head.box("jps_body", (hx + sx - 0.05, -0.07, ARM_Z - 0.07), (hx + sx + 0.05, 0.07, ARM_Z + 0.07))   # clamp
    head.box("jps_hood", (hx - BODY_W / 2 + 0.008, hy + BODY_D / 2, hz - BODY_H / 2 + 0.008),
             (hx + BODY_W / 2 - 0.008, hy + BODY_D / 2 + 0.004, hz + BODY_H / 2 - 0.008))   # black face

    lamp = Part("SM_JpShingoLamp", ["jps_hood", "jps_lens"])                # slot 0 hood, slot 1 lens
    yf = BODY_D / 2 + 0.004
    lamp.frustum("jps_hood", (0, yf - 0.002, 0), (0, yf + 0.010, 0), LENS_R + 0.012, LENS_R + 0.012)  # bezel ring
    lamp.disc("jps_lens", yf + 0.010, yf + 0.016, LENS_R)
    lamp.hood("jps_hood", yf, HOOD_L, HOOD_R, HOOD_T)
    return {"pole": pole.to_object(), "arm": arm.to_object(),
            "head": head.to_object((hx, hy, hz)), "lamp": lamp.to_object()}


def export(obs, out):
    for ob in obs.values():
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.export_scene.fbx(filepath=os.path.join(out, ob.name + ".fbx"), use_selection=True,
                                 object_types={"MESH"}, apply_unit_scale=True, bake_space_transform=False,
                                 mesh_smooth_type="FACE")
    layout = {"head_position_cm": [HEAD_X * 100, HEAD_Y * 100, HEAD_Z * 100],
              "lamps_cm": [{"colour": c, "x": x * 100, "y": 0.0, "z": 0.0} for c, x in LAMPS],
              "meshes": sorted(o.name for o in obs.values()),
              "lamp_slots": ["jps_hood", "jps_lens"]}
    json.dump(layout, open(os.path.join(out, "layout.json"), "w"), indent=1)
    print("[jp_shingo] wrote", layout["meshes"], "to", out)


def render(obs, out):
    """Previews: the full signal assembled as BP_JpShingo places it, per state."""
    sc = bpy.context.scene
    head = obs["head"]
    head.location = ue((HEAD_X, HEAD_Y, HEAD_Z))
    lamps = []
    for c, x in LAMPS:
        ob = obs["lamp"].copy()
        ob.data = obs["lamp"].data.copy()
        ob.location = ue((HEAD_X + x, HEAD_Y, HEAD_Z))
        sc.collection.objects.link(ob)
        lamps.append((c, ob))
    obs["lamp"].hide_render = True
    bpy.ops.mesh.primitive_plane_add(size=40, location=(-4, 0, 0))
    ground = bpy.context.active_object
    gm = bpy.data.materials.new("ground")
    gm.use_nodes = True
    gm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.08, 0.08, 0.085, 1)
    ground.data.materials.append(gm)
    world = sc.world or bpy.data.worlds.new("w")
    sc.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.55, 0.68, 0.85, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.8
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(50), 0, math.radians(30))
    sc.collection.objects.link(sun)
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 48
    sc.cycles.use_denoising = False
    sc.render.resolution_x, sc.render.resolution_y = 960, 540
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam

    def shot(name, eye, target, lens=50):
        cam.location = ue(eye)
        cam.data.lens = lens
        d = ue(target) - cam.location
        cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = os.path.join(out, "preview_%s.png" % name)
        bpy.ops.render.render(write_still=True)

    for state in ("GREEN", "YELLOW", "RED", "OFF"):
        for c, ob in lamps:
            ob.data.materials[1] = material("jps_lens", c if c == state else None)
        # driver's eye in the lane, 20 m before the head / 停止線手前の運転席から
        shot("driver_%s" % state.lower(), (HEAD_X + 0.6, 20.0, 1.25), (HEAD_X, HEAD_Y, HEAD_Z), 85)
    for c, ob in lamps:
        ob.data.materials[1] = material("jps_lens", c if c == "RED" else None)
    shot("side", (3.5, 9.0, 2.0), (-3.0, 0.0, 4.2), 28)
    shot("closeup", (HEAD_X + 1.4, 2.6, HEAD_Z - 0.6), (HEAD_X, HEAD_Y, HEAD_Z), 50)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = os.path.abspath(argv[0] if argv else "build/JpShingo")
    os.makedirs(out, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    obs = build()
    export(obs, out)
    if "--render" in argv:
        render(obs, out)


main()
