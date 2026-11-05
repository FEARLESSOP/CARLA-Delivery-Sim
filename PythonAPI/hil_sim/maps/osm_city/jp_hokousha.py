"""Japanese pedestrian signal heads (歩行者用灯器) as map objects (maps/.venv).

  python jp_hokousha.py <build/<Name>>        (after hokousha_crossings.py)
  python jp_hokousha.py --sample <out_dir>    (one head + pole, for previews/tests)

One object per signalised crossing, Terrain_JpHokousha_<Name>_<NNN> (NNN =
the crossing id of hokousha.json): a grey pole at each kerb end with a
vertical two-lamp LED head facing across the crossing. Our own geometry,
drawn from the standard look (guide ch. 26; never from photos):

* TWO rounded-square LED lamps one above the other in a grey frame: upper =
  red STANDING figure with a hat (止まれ), lower = blue-green (青) WALKING
  figure with a hat (進め, flashing before red); one lamp lit at a time; only
  the figure lights, black face around, the unlit figure faintly visible;
* two columns of 10 small square segments, both sides of the figure, the full
  face height (経過時間表示): the lit lamp's columns count down (cyan: walk
  left, red: wait left), as on the reference head;
* faces 330 mm (figure 240 mm), frames 0.40 x 0.40 m, short visors, head
  bottom 2.7 m over the pavement (2.5 m minimum, 2.7-3.2 m usual).

Materials (slots): jph_body (frame), jph_face (black face), jph_pole,
jph_stop / jph_walk (the figures), jph_count (the segments; each segment's UV
v = (i + 0.5) / 10, i = 0 at the bottom; u = 0.25 lower lamp, 0.75 upper). maps/ue_jp_hokousha.py puts our lamp
materials on the stop/walk/count slots; the rig (hil_sim/hokousha.py) lights
them at run time through the "Emissive" texture parameter. No "sign"/"light"
in any name (CARLA's importer drops those).

日本の歩行者用信号機（2 灯式・帽子の人形・経過時間表示）を横断歩道ごとの
オブジェクトとして作る。自作の形状のみ。
"""

import json
import math
import os
import sys

import mapbox_earcut as earcut
import numpy as np
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mesh import Mesh  # noqa: E402

LAMP = 0.40            # lamp frame, square / 灯器 1 面の枠
FACE = 0.33            # black face inside the frame / 黒い表示面
DEPTH = 0.16           # housing depth / 奥行き
FIG_H = 0.24           # figure height / 人形の高さ
SEGS = 10              # countdown segments per column (= hil_sim/hokousha.SEGS) / 区画数
SEG = 0.020            # segment size / 区画の大きさ
BOTTOM = 2.7           # head bottom over the pavement / 灯器下端の高さ
POLE_R = 0.05          # pole radius (φ100) / 支柱半径
GAP_POLE = 0.03        # head back to pole surface / 支柱と灯器の隙間

MATERIALS = {          # manifest spec: texture, colour, roughness, metallic, smooth
    "jph_body": [None, [150, 154, 156], 0.5, 0.3, False],
    "jph_face": [None, [8, 8, 8], 0.4, 0.0, False],
    "jph_pole": [None, [150, 154, 156], 0.5, 0.4, True],
    "jph_stop": [None, [60, 12, 10], 0.3, 0.0, False],
    "jph_walk": [None, [10, 48, 44], 0.3, 0.0, False],
    "jph_count": [None, [30, 30, 30], 0.3, 0.0, False],
}


# -- figures (unit: metres of figure height 1) / 人形（高さ 1 の単位）-----------------
def limb(x0, y0, x1, y1, w):
    """A rounded bar from (x0, y0) to (x1, y1), w wide. / 丸みのある棒"""
    ln = math.hypot(x1 - x0, y1 - y0)
    b = box(-w / 2, 0, w / 2, ln)
    b = affinity.rotate(b, math.degrees(math.atan2(y1 - y0, x1 - x0)) - 90, origin=(0, 0))
    return affinity.translate(b, x0, y0).buffer(w * 0.12, join_style=1)


def hat(cx, top):
    """Brimmed hat: crown + brim. / つば付きの帽子"""
    crown = box(cx - 0.075, top - 0.075, cx + 0.075, top).buffer(0.012, join_style=1)
    brim = box(cx - 0.125, top - 0.085, cx + 0.125, top - 0.06)
    return unary_union([crown, brim])


def standing():
    """止まれ: standing figure, arms down, feet together, hat. / 立ち止まる人形"""
    parts = [hat(0.0, 1.0), Point(0.0, 0.83).buffer(0.075),
             box(-0.11, 0.42, 0.11, 0.74).buffer(0.03, join_style=1),          # torso / 胴
             limb(-0.155, 0.72, -0.165, 0.42, 0.055), limb(0.155, 0.72, 0.165, 0.42, 0.055),  # arms / 腕
             limb(-0.05, 0.45, -0.055, 0.0, 0.075), limb(0.05, 0.45, 0.055, 0.0, 0.075)]      # legs / 脚
    return unary_union(parts).buffer(0.004).buffer(-0.002)


def walking():
    """進め: walking figure, striding to the right, hat. / 歩く人形（右向き）"""
    parts = [hat(0.04, 1.0), Point(0.04, 0.83).buffer(0.075),
             affinity.rotate(box(-0.085, 0.43, 0.085, 0.74).buffer(0.03, join_style=1), -8, origin=(0, 0.43)),
             limb(0.06, 0.70, 0.20, 0.50, 0.055), limb(0.20, 0.50, 0.25, 0.40, 0.05),         # front arm / 前の腕
             limb(-0.01, 0.70, -0.14, 0.53, 0.055), limb(-0.14, 0.53, -0.20, 0.44, 0.05),     # back arm / 後ろの腕
             limb(0.01, 0.46, 0.17, 0.22, 0.08), limb(0.17, 0.22, 0.23, 0.0, 0.07),           # front leg / 前脚
             box(0.20, 0.0, 0.30, 0.04),
             limb(-0.01, 0.46, -0.12, 0.22, 0.08), limb(-0.12, 0.22, -0.26, 0.04, 0.07)]      # back leg / 後脚
    return unary_union(parts).buffer(0.004).buffer(-0.002)


def _polys(g):
    return [g] if g.geom_type == "Polygon" else [q for q in g.geoms if q.geom_type == "Polygon"]


# -- placing a flat 2-D shape on the head face / 平面形状を灯器面に置く ----------------
class Frame(object):
    """Head frame: origin = centre of the head front at its bottom; u right (as
    seen by the walker who faces it), v up, w toward that walker."""

    def __init__(self, x, y, z, face):
        self.o = np.array([x, y, z], float)
        self.w = np.array([face[0], face[1], 0.0])
        self.w /= np.linalg.norm(self.w)
        self.v = np.array([0.0, 0.0, 1.0])
        self.u = np.cross(self.v, self.w)       # viewer's right / 見る人の右

    def p(self, u, v, w):
        return self.o + self.u * u + self.v * v + self.w * w


def flat_shape(m, mat, fr, g, w, uv=None):
    """Triangulate a 2-D (u, v) shape onto the plane at depth w, facing +w."""
    for poly in _polys(g):
        poly = orient(poly, 1.0)
        rings = [np.array(poly.exterior.coords)[:-1]] + [np.array(r.coords)[:-1] for r in poly.interiors]
        verts = np.concatenate(rings)
        ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
        idx = earcut.triangulate_float64(verts, ends).reshape(-1, 3)
        P = [fr.p(a, b, w) for a, b in verts]
        UV = [uv if uv is not None else (a, b) for a, b in verts]
        # earcut winds counter-clockwise in (u, v); u x v = w, so the face points +w / +w 向き
        m.tris(mat, P, fr.w, UV, idx[:, [0, 1, 2]])


def rounded_square(cx, cy, size, r):
    return box(cx - size / 2 + r, cy - size / 2 + r, cx + size / 2 - r, cy + size / 2 - r).buffer(r, quad_segs=4)


def prism_shape(m, mat, fr, g, w0, w1):
    """An extruded (u, v) shape from depth w0 (back) to w1 (front): front cap and side walls."""
    flat_shape(m, mat, fr, g, w1)
    for poly in _polys(g):
        ring = np.array(orient(poly, 1.0).exterior.coords)
        for (a0, b0), (a1, b1) in zip(ring[:-1], ring[1:]):
            m.quad(mat, fr.p(a0, b0, w0), fr.p(a1, b1, w0), fr.p(a1, b1, w1), fr.p(a0, b0, w1))
    # back cap facing -w / 背面
    back = Frame(0, 0, 0, (-fr.w[0], -fr.w[1]))
    back.o = fr.o
    mirrored = affinity.scale(g, xfact=-1, origin=(0, 0))
    flat_shape(m, mat, back, mirrored, -w0)


def head(m, x, y, zg, face):
    """One head on its pole at ground (x, y, zg), the lamps facing `face` (unit xy):
    two lamps one above the other, red standing figure on top, blue-green
    walking figure below, each with its countdown column on the left (only the
    lit lamp's column shows). / 支柱付きの灯器 1 基（縦 2 灯）"""
    fr = Frame(x, y, zg + BOTTOM, face)
    px, py = np.array([x, y]) - fr.w[:2] * (POLE_R + GAP_POLE)
    m.cylinder("jph_pole", px, py, zg - 0.3, zg + BOTTOM + 2 * LAMP + 0.25, POLE_R, seg=12)
    for k, (fig, mat) in enumerate(((walking(), "jph_walk"), (standing(), "jph_stop"))):
        cv = LAMP / 2 + k * LAMP          # k = 0 lower (walk), 1 upper (stop) / 0 下 1 上
        frame = rounded_square(0.0, cv, LAMP - 0.01, 0.05)
        face_g = rounded_square(0.0, cv, FACE, 0.03)
        prism_shape(m, "jph_body", fr, frame.difference(face_g), 0.0, DEPTH)
        prism_shape(m, "jph_body", fr, face_g, 0.0, DEPTH - 0.03)
        flat_shape(m, "jph_face", fr, face_g, DEPTH - 0.029)
        top = cv + LAMP / 2
        # short visor over each face / 各面の庇
        m.quad("jph_body", fr.p(-LAMP / 2 + 0.01, top - 0.012, DEPTH), fr.p(LAMP / 2 - 0.01, top - 0.012, DEPTH),
               fr.p(LAMP / 2 - 0.01, top - 0.03, DEPTH + 0.07), fr.p(-LAMP / 2 + 0.01, top - 0.03, DEPTH + 0.07))
        m.quad("jph_body", fr.p(-LAMP / 2 + 0.01, top - 0.03, DEPTH + 0.07), fr.p(LAMP / 2 - 0.01, top - 0.03, DEPTH + 0.07),
               fr.p(LAMP / 2 - 0.01, top - 0.012, DEPTH), fr.p(-LAMP / 2 + 0.01, top - 0.012, DEPTH))
        fig = fig.simplify(0.004)
        fig = affinity.translate(affinity.scale(fig, FIG_H, FIG_H, origin=(0, 0)), 0.0, cv - FIG_H / 2)
        flat_shape(m, mat, fr, fig, DEPTH - 0.027)
        # countdown columns on BOTH sides of the figure, the full face height,
        # bottom segment i = 0; UV u = 0.25 lower lamp, 0.75 upper
        # 人形の両側に面の高さいっぱいの経過時間表示（下から 0）。u = 0.25 下、0.75 上
        for side in (-1.0, 1.0):
            for i in range(SEGS):
                sv = cv - 0.135 + i * 0.03
                su = side * 0.138
                flat_shape(m, "jph_count", fr, box(su - SEG / 2, sv - SEG / 2, su + SEG / 2, sv + SEG / 2),
                           DEPTH - 0.027, uv=(0.25 + 0.5 * k, (i + 0.5) / SEGS))
    for v in (0.1, LAMP, 2 * LAMP - 0.1):
        m.box("jph_pole", *(fr.p(0, v, -GAP_POLE / 2 - POLE_R / 2)[:2]), zg + BOTTOM + v - 0.02,
              zg + BOTTOM + v + 0.02, GAP_POLE + POLE_R, 0.06, ang=math.atan2(fr.w[1], fr.w[0]))


def save(mesh_dir, objs):
    """Write the objects' npz and register them in manifest.json (replacing old ones)."""
    man_p = os.path.join(mesh_dir, "manifest.json")
    man = json.load(open(man_p)) if os.path.exists(man_p) else {"name": "", "objects": [], "materials": {}}
    for o in [o for o in man["objects"] if "_JpHokousha_" in o["name"]]:
        man["objects"].remove(o)
        try:
            os.remove(os.path.join(mesh_dir, o["name"] + ".npz"))
        except OSError:
            pass
    for on, m in objs:
        arrs = m.arrays()
        data = {}
        for mat, (V, N, UV, I) in arrs.items():
            data[mat + "|V"], data[mat + "|N"], data[mat + "|UV"], data[mat + "|I"] = V, N, UV, I
        np.savez_compressed(os.path.join(mesh_dir, on + ".npz"), **data)
        man["objects"].append({"name": on, "kind": "jp_hokousha", "materials": sorted(arrs),
                               "tris": int(sum(len(a[3]) for a in arrs.values()))})
    man["materials"].update(MATERIALS)
    json.dump(man, open(man_p, "w"), indent=0)


def build(out):
    data = json.load(open(os.path.join(out, "hokousha.json")))
    name = data["map"]
    objs = []
    for cr in data["crossings"]:
        if not cr.get("signal") or not cr.get("heads"):
            continue
        m = Mesh()
        for h in cr["heads"]:
            # hokousha.json is in CARLA's frame (y south); the meshes use the map frame (y north)
            # hokousha.json は CARLA 座標（y 南向き）、メッシュは地図座標（y 北向き）
            head(m, h["x"], -h["y"], h["z"], (h["face"][0], -h["face"][1]))
        objs.append(("Terrain_JpHokousha_%s_%03d" % (name, cr["id"]), m))
    save(os.path.join(out, "meshes"), objs)
    print("[jp_hokousha] wrote %d crossing objects, %d heads" % (len(objs), sum(len(c.get("heads", []))
          for c in data["crossings"] if c.get("signal"))))
    return objs


def sample(out_dir):
    os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)
    m = Mesh()
    head(m, 0.0, 0.0, 0.0, (0.0, -1.0))     # faces -y (south) / 南向き
    save(os.path.join(out_dir, "meshes"), [("Terrain_JpHokousha_Sample_000", m)])
    print("[jp_hokousha] sample head: %d tris" % m.ntris())
    return m


if __name__ == "__main__":
    if sys.argv[1] == "--sample":
        sample(sys.argv[2])
    else:
        build(sys.argv[1])
