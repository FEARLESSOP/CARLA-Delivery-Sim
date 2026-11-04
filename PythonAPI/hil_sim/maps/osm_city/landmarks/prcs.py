"""Pasir Ris Crest Secondary School, modelled in detail (maps/.venv).

Sources (2025-2026): Google Maps satellite, traced at zoom 20
(build/PasirRis/prcs_trace.json); Street View on Pasir Ris Street 41 and
Drive 4 (Mar 2025); the school's Traffic Management Plan (prcss.moe.edu.sg,
block names and uses); the architect's photo of the indoor sports hall
(Planar One, 2011; repainted white since). Nothing is copied as texture.

  blocks       Allegro (offices, library; "Home of the Overcomers" and the
               school name on its walls), Forte and Energico (computer labs,
               the diamond garden between them), the hall block (canteen
               on L1, hall on L2, "Bravura" on its north face), Dolce
               (science labs), a link wing; 4 storeys, white, teal glass,
               open corridors with grey railings on the courtyard side,
               hipped grey metal roofs covered in solar panels
  rotunda      the octagonal tower at the drop-off: three storeys of stacked
               curved balconies, a cantilevered canopy with a teal fascia, a
               two-tier dark green roof
  sports hall  white, open ground floor on columns, hall above with louvre
               bands, dark green hipped roof
  grounds      astro football field with a tall fence and blue windbreak,
               a low retaining wall on Pasir Ris Drive 4; the courtyard netball
               court, the red court, the outdoor basketball court; red brick
               drop-off loop round the oval "PRCS" planter, parking bays; the
               Crescendo covered walkway along the field; covered links
  gate         blue steel gates, the white guard room, the green name wall
               with the school crest; green chain-link fencing round the site

Coordinates: the map frame (x east, y north, metres).

白沙岭中学（Pasir Ris Crest Secondary School）の詳細モデル。
"""

import json
import math
import os

import numpy as np
from shapely.affinity import rotate, scale, translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from mesh import Mesh, polys

NAME = "Pasir Ris Crest Secondary School"
TAG = "PRCS"
TRACE = "prcs_trace.json"
FLOOR = 3.6
Z0 = 0.15
STOREYS = 4
EAVE = Z0 + STOREYS * FLOOR


def text_poly(text, height, font="DejaVuSans-Bold.ttf", aspect=1.0):
    """Text as polygons, ``height`` tall, centred, x reading direction. / 文字の多角形。"""
    from PIL import Image, ImageDraw, ImageFont
    from shapely.ops import unary_union as uu
    f = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/" + font, 110)
    bb = f.getbbox(text)
    im = Image.new("L", (bb[2] + 12, bb[3] + 12), 0)
    ImageDraw.Draw(im).text((6, 6), text, fill=255, font=f)
    a = np.asarray(im) > 128
    h, w = a.shape
    cells = []
    for y in range(h):
        row, x = a[y], 0
        while x < w:
            if row[x]:
                x0 = x
                while x < w and row[x]:
                    x += 1
                cells.append(box(x0, h - y - 1, x, h - y))
            x += 1
    g = uu(cells).simplify(0.8)
    minx, miny, maxx, maxy = g.bounds
    k = height / (maxy - miny)
    return scale(translate(g, -(minx + maxx) / 2, -(miny + maxy) / 2), k * aspect, k, origin=(0, 0))


class PRCS(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.drive_snapshot = L.driveways          # before build.py clears the site / 消去前の駐車通路
        self.meta = L.meta
        fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, "prcs_trace.json")))
        self.T = T

        def P(tile, px, py):
            lat, lon = T["tiles"][tile]
            cx, cy = fr.xy(lon, lat)
            return (cx + (px - T["centre"][0]) * T["mpp"], cy - (py - T["centre"][1]) * T["mpp"])
        self.P = P
        self.site = [p for p, t in L.osm.areas(lambda t: t.get("name") == NAME)][0]
        self.blocks = {}
        for k, b in T["blocks"].items():
            pts = [P(b["tile"], *q) for q in b["pts"]]
            if "extend_to" in b:
                # the hall block runs on south to the line traced on another tile / 南へ延長
                y_end = P(b["extend_to"]["tile"], 784, b["extend_to"]["y"])[1]
                (a0, a1, a2, a3) = [np.array(q) for q in pts]
                d = a3 - a0
                t = (y_end - a0[1]) / d[1]
                pts = [tuple(a0), tuple(a1), tuple(a1 + d * t), tuple(a0 + d * t)]
            self.blocks[k] = dict(poly=orient(Polygon(pts), 1.0), storeys=b["storeys"])
        r = T["rotunda"]
        self.rot_c = P(r["tile"], *r["centre"])
        self.rot_r = r["r_px"] * T["mpp"]
        poly = lambda d: Polygon([P(d["tile"], *q) for q in d["pts"]])
        self.court_c = poly(T["court_c"])
        self.red_court = poly(T["red_court"])
        self.outdoor = poly(T["outdoor_court"])
        pc = P(T["planter"]["tile"], *T["planter"]["centre"])
        self.planter = scale(Point(pc).buffer(1.0, 24), T["planter"]["r_px"][0] * T["mpp"],
                             T["planter"]["r_px"][1] * T["mpp"])
        self.links = [(LineString([P(l["tile"], *q) for q in l["pts"]]), l["w_px"] * T["mpp"]) for l in T["links"]]
        c = T["crescendo"]
        self.crescendo = LineString([P(c["tile"], *q) for q in c["pts"]])
        g = T["guardhouse"]
        gx, gy = P(g["tile"], *g["centre"])
        self.guard = (gx, gy)
        cn = T["canopy"]
        self.canopy = (P(cn["tile"], *cn["centre"]), cn["size_px"][0] * T["mpp"], cn["size_px"][1] * T["mpp"],
                       math.radians(-cn["angle"]))
        # the field: OSM's pitch inside the site / 校庭: OSM のピッチ
        self.field = [p for p, t in L.osm.areas(lambda t: t.get("leisure") == "pitch")
                      if self.site.contains(p.representative_point())][0].minimum_rotated_rectangle
        # the main gate on Pasir Ris Street 41 (Street View pano) / 正門
        self.gate = fr.xy(103.95962, 1.37344)
        self.centre = self.site.centroid
        foot = [b["poly"] for b in self.blocks.values()] + [Point(self.rot_c).buffer(self.rot_r, 8)]
        self.footprint = unary_union(foot)
        # the school's driveway and drop-off loop are in the road network (an
        # OSM service road): nothing of the school may stand on them
        # 校内の車寄せは道路網の一部: その上には何も置かない
        self.road = L.road_all.buffer(0.05)
        self.planter = self.planter.difference(L.road_all.buffer(0.3))
        if self.planter.geom_type != "Polygon":
            self.planter = max(polys(self.planter), key=lambda g: g.area)
        # the gate: where the driveway crosses the site boundary / 正門: 車路と敷地境界の交点
        span = self.site.exterior.intersection(L.road_all.buffer(0.6))
        pts = [q for g in getattr(span, "geoms", [span]) for q in getattr(g, "coords", [])]
        near = [q for q in pts if math.dist(q, self.gate) < 25.0]
        if len(near) >= 2:
            a = min(near, key=lambda q: q[0])
            b = max(near, key=lambda q: q[0])
            self.gate_span = (a, b)
            self.gate = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        else:
            self.gate_span = ((self.gate[0] - 4.0, self.gate[1]), (self.gate[0] + 4.0, self.gate[1]))
        self.fit_rotunda(L)

    def fit_rotunda(self, L, clear=1.0):
        """The road wins: the drop-off loop runs round the tower, so the tower
        (whose traced centre sat 5.9 m from the loop with a 9.5 m reach, its
        columns in the lane) moves back from the road, up to 4 m, toward the
        site and stays clear of the blocks, then shrinks if it still does not
        clear. Its canopy and eaves are cut back from the road too.
        道路優先: 塔を車寄せから離し、収まらなければ縮める。"""
        from shapely.geometry import Point as Pt
        road = L.road_all
        c = Pt(self.rot_c)
        q = road.boundary.interpolate(road.boundary.project(c))
        v = np.array([c.x - q.x, c.y - q.y])
        n = np.linalg.norm(v)
        v = v / n if n > 1e-6 else np.array([0.0, 1.0])
        others = unary_union([b["poly"] for k, b in self.blocks.items()])
        best = None
        for step in np.arange(0.0, 4.01, 0.25):
            cc = Pt(c.x + v[0] * step, c.y + v[1] * step)
            r_road = road.distance(cc) - clear
            r_blk = others.distance(cc) + 3.0          # may tuck into a block a little / 少し食い込み可
            r = min(self.rot_r, r_road, max(r_blk, 4.0))
            if best is None or r > best[1] + 0.05:
                best = (step, r)
            if r >= self.rot_r - 1e-6:
                break
        step, r = best
        self.rot_c = (c.x + v[0] * step, c.y + v[1] * step)
        self.rot_r = max(4.0, min(r, self.rot_r))
        self.road_clip = road.buffer(0.6)
        print("rotunda: moved %.2f m from the drop-off loop, radius %.2f m (traced %.2f)"
              % (step, self.rot_r, self.T["rotunda"]["r_px"] * self.T["mpp"]))
        foot = [b["poly"] for b in self.blocks.values()] + [Pt(self.rot_c).buffer(self.rot_r + 2.2, 8)]
        self.footprint = unary_union(foot)

    # -- layout / 配置
    def inner(self):
        return self.site.buffer(-0.6, join_style="mitre")

    def keepout(self):
        """Where the generic trees must not go. / 汎用の木を置かない所。"""
        return unary_union([self.footprint.buffer(3.0), self.field.buffer(5.0),
                            unary_union([self.court_c, self.outdoor, self.red_court]).buffer(1.0),
                            self.inner().intersection(box(-95, 326, 45, 400))])

    def ground_mesh(self, tile, z):
        """Paving, courts, the field and gardens inside the site. / 敷地内の地面。"""
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        T = lambda g: g.intersection(tile)
        inner = self.inner()
        hard = unary_union([self.footprint.buffer(1.5, join_style="mitre"),
                            # the north plaza and drop-off / 北側広場と車寄せ
                            inner.intersection(box(-95, 326, 45, 400)),
                            self.crescendo.buffer(2.0, cap_style="flat"),
                            unary_union([l.buffer(w / 2 + 1.0, cap_style="flat") for l, w in self.links])])
        courts = unary_union([self.court_c, self.red_court, self.outdoor, self.field.buffer(3.0, join_style="mitre")])
        hard = hard.difference(courts).difference(self.planter).intersection(inner).difference(self.road)
        m.flat("pavers_red", T(hard), z + 0.01, 2.0)
        m.flat("grass_park", T(inner.difference(hard).difference(courts).difference(self.footprint)
                               .difference(self.planter).difference(self.road)), z + 0.005, 4.0)
        m.flat("parade_red", T(self.field.buffer(3.0, join_style="mitre").difference(self.field)), z + 0.02, 3.0)
        m.flat("parade_red", T(self.red_court), z + 0.02, 3.0)
        for poly, mat in ((self.field, "pitch_astro"), (self.court_c, "court_netball"),
                          (self.outdoor, "court_bb_green")):
            if tile.contains(poly.centroid):           # once, in its own tile / 自分のタイルに 1 回だけ
                self._court(m, poly, mat, z + 0.03)
        # the oval planter with its low wall / 楕円の植え込み
        if tile.contains(self.planter.centroid):
            m.prism("concrete", self.planter, z, z + 0.45, top=False)
            m.flat("grass_park", self.planter.buffer(-0.2), z + 0.4, 2.0)
        return m

    def _court(self, m, poly, mat, z):
        c = list(orient(poly.minimum_rotated_rectangle, 1.0).exterior.coords)[:4]
        e0, e1 = math.dist(c[0], c[1]), math.dist(c[1], c[2])
        if e1 > e0:
            c = c[1:] + c[:1]
        P = [(q[0], q[1], z) for q in c]
        m.quad(mat, P[0], P[1], P[2], P[3], (0, 0), (1, 0), (1, 1), (0, 1))

    # -- buildings / 建物
    def buildings_mesh(self, tile):
        m = Mesh()
        x0, y0, x1, y1 = tile.bounds
        inside = lambda x, y: x0 <= x < x1 and y0 <= y < y1
        for k, b in self.blocks.items():
            c = b["poly"].centroid
            if inside(c.x, c.y):
                if k == "sports_hall":
                    self.sports_hall(m, b["poly"])
                elif k == "link_e":
                    self.school_block(m, b["poly"], k, plain=True)
                else:
                    self.school_block(m, b["poly"], k)
        if inside(*self.rot_c):
            self.rotunda(m)
        if inside(*self.gate):
            self.gate_area(m)
        if inside(self.centre.x, self.centre.y):
            for l, w in self.links:
                self.link(m, l, w)
            self.crescendo_walk(m)
            self.flagpoles(m)
        f = self.field.centroid
        if inside(f.x, f.y):
            self.field_fence(m)
        if inside(self.centre.x, self.centre.y):
            self.site_fence(m)
        return m

    def corridor_side(self, poly):
        """Edges whose outward normal faces the site's middle carry the open
        corridors. / 敷地中央を向く辺に共用廊下。"""
        out = set()
        for i, (a, b, L, n) in enumerate(B.edges_of(poly)):
            mid = (a + b) / 2
            v = np.array([self.centre.x, self.centre.y]) - mid
            if L > 12 and float(n @ v) > 0:
                out.add(i)
        return out

    def school_block(self, m, poly, name, plain=False):
        edges = B.edges_of(poly)
        corr = self.corridor_side(poly)
        hall = name == "hall"
        for i, (a, b, L, n) in enumerate(edges):
            f = B.Facade(m, a, b, n)
            end = L < 20 and not plain
            for k in range(STOREYS):
                z0 = Z0 + k * FLOOR
                z1 = z0 + FLOOR
                if hall and k == 0 and L > 25:
                    self.canteen_floor(f, L, z0, z1)
                elif hall and k in (1, 2) and L > 25 and i in (1, 3):
                    if k == 1:
                        self.hall_wall(f, L, z0, z0 + 2 * FLOOR)
                elif plain or end:
                    f.face("paint_white", 0.0, L, z0, z1, 0.0, 2.0)
                    f.block("paint_palegrey", 0.0, L, 0.0, 0.03, z0 + FLOOR - 0.12, z0 + FLOOR - 0.04, ends=False)
                elif i in corr:
                    self.corridor(f, L, z0, z1)
                else:
                    self.classroom_windows(f, L, z0, z1)
            # slab band at the eaves / 軒の帯
            f.block("paint_white", 0.0, L, 0.0, 0.25, EAVE - 0.6, EAVE, ends=False)
        self.hip_roof_solar(m, poly, EAVE, raised=hall)
        if name == "allegro":
            self.allegro_texts(m, poly)
        if name == "hall":
            self.wall_text(m, poly, "Bravura", side="north", height=1.6, z=Z0 + 2 * FLOOR + 1.0,
                           font="DejaVuSans-Oblique.ttf", mat="paint_navy", along=0.3)

    def corridor(self, f, L, z0, z1, depth=2.2):
        """Open corridor: slab edge, low wall, grey railing, the classroom wall
        behind with doors and windows, columns. / 開放廊下。"""
        f.face("paint_white", 0.0, L, z0, z0 + 0.3, 0.0)
        f.block("paint_white", 0.0, L, -0.15, 0.0, z0 + 0.3, z0 + 0.55, top=True, bottom=False, ends=False)
        f.block("railing", 0.0, L, -0.1, -0.05, z0 + 1.05, z0 + 1.12, ends=False)
        for u in np.arange(0.4, L, 1.2):
            f.block("railing", u - 0.03, u + 0.03, -0.1, -0.05, z0 + 0.55, z0 + 1.05, top=False, bottom=False)
        f.face("paint_palegrey", 0.0, L, z0 + 0.3, z1, -depth)
        f.hplane("concrete", 0.0, L, -depth, 0.0, z1 - 0.001, False)
        f.hplane("void_floor", 0.0, L, -depth, -0.15, z0 + 0.3, True)
        for u0, u1 in B.bays(L, 7.2, 0.6):
            mid = (u0 + u1) / 2
            f.face("door_wood", mid - 2.6, mid - 1.6, z0 + 0.3, z0 + 2.4, -depth + 0.02)
            f.face("glass_teal", mid - 1.0, mid + 2.6, z0 + 1.2, z0 + 2.6, -depth + 0.02)
        for u in np.linspace(0.25, L - 0.25, max(2, int(L // 7.2) + 1)):
            f.block("paint_white", u - 0.25, u + 0.25, -depth, 0.0, z0 + 0.55, z1, top=False, bottom=False)

    def classroom_windows(self, f, L, z0, z1):
        """Teal glass window bands with white sunshade fins. / 青緑ガラスの窓と庇。"""
        opens = [(u0 + 0.35, u1 - 0.35, z0 + 0.9, z0 + 2.8) for u0, u1 in B.bays(L, 3.6, 0.6)]
        f.wall_with_openings("paint_white", z0, z1, opens, 2.0)
        for o in opens:
            f.window(*o, "paint_white", glass="glass_teal", mullions=2)
        f.block("paint_white", 0.4, L - 0.4, 0.0, 0.7, z0 + 2.95, z0 + 3.05)          # sunshade / 庇
        f.block("paint_palegrey", 0.0, L, 0.0, 0.05, z0 + 0.05, z0 + 0.25, ends=False)

    def canteen_floor(self, f, L, z0, z1):
        """L1 canteen: open on columns, the back wall in shade. / 1 階の食堂（柱のみ）。"""
        f.face("paint_palegrey", 0.0, L, z0, z1, -4.0)
        f.hplane("concrete", 0.0, L, -4.0, 0.0, z1 - 0.001, False)
        for u in np.linspace(0.3, L - 0.3, max(2, int(L // 6.0) + 1)):
            f.block("paint_white", u - 0.3, u + 0.3, -0.6, 0.0, z0, z1, top=False, bottom=False)
        f.block("paint_white", 0.0, L, 0.0, 0.2, z1 - 0.5, z1, ends=False)

    def hall_wall(self, f, L, z0, z1):
        """L2 hall: double height, tall glazed strips between fins. / 2 層吹抜けのホール。"""
        opens = [(u0 + 0.6, u1 - 0.6, z0 + 1.2, z1 - 0.8) for u0, u1 in B.bays(L, 4.8, 0.6)]
        f.wall_with_openings("paint_white", z0, z1, opens, 2.0)
        for o in opens:
            f.window(*o, "paint_white", glass="glass_teal", mullions=1, depth=0.3)
            f.block("paint_white", o[1] + 0.05, o[1] + 0.45, 0.0, 0.6, z0 + 0.8, z1 - 0.4)

    def hip_roof_solar(self, m, poly, z, raised=False):
        """Low hipped metal roof, overhanging, covered in solar panels; the hall
        has a raised roof over its north end. / 太陽光パネルの載る寄棟屋根。"""
        r = orient(poly.minimum_rotated_rectangle, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, b, cc, d = [c[(k + i) % 4] for i in range(4)]
        ctr = (a + b + cc + d) / 4
        ax = (b - a) / np.linalg.norm(b - a)
        side = (d - a) / np.linalg.norm(d - a)
        ov = 1.0
        a2, b2, c2, d2 = (a - ax * ov - side * ov, b + ax * ov - side * ov, cc + ax * ov + side * ov,
                          d - ax * ov + side * ov)
        W = np.linalg.norm(d2 - a2)
        Lr = np.linalg.norm(b2 - a2)
        h = W / 2 * math.tan(math.radians(14))
        ins = min(W / 2, Lr / 2 - 0.1)
        r1 = (a2 + d2) / 2 + ax * ins
        r2 = (b2 + c2) / 2 - ax * ins
        P = lambda q, zz: (q[0], q[1], zz)
        m.flat("concrete", poly, z - 0.01, 2.0)
        m.quad("roof_metal", P(a2, z), P(b2, z), P(r2, z + h), P(r1, z + h))
        m.quad("roof_metal", P(c2, z), P(d2, z), P(r1, z + h), P(r2, z + h))
        m.tris("roof_metal", [P(b2, z), P(c2, z), P(r2, z + h)], (0, 0, 1), [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
        m.tris("roof_metal", [P(d2, z), P(a2, z), P(r1, z + h)], (0, 0, 1), [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
        m.quad("concrete", P(b2, z), P(a2, z), P(d2, z), P(c2, z))
        # solar panels on both long slopes, rows along the ridge / 長辺の斜面にパネル列
        for s0, s1, f0 in ((a2, b2, 1), (c2, d2, -1)):
            along = (s1 - s0) / np.linalg.norm(s1 - s0)
            up = side * f0
            for row in range(3):
                t0, t1 = 0.6 + row * 1.9, 0.6 + row * 1.9 + 1.7
                if t1 > W / 2 - 0.4:
                    break
                for k2 in range(int((Lr - 2 * (ins * 0.0) - 4.0) // 1.1)):
                    u0 = 2.0 + k2 * 1.1 + t0 * 0.9
                    u1 = u0 + 1.0
                    if u1 > Lr - 2.0 - t0 * 0.9:
                        break
                    q = [s0 + along * u + up * t for u, t in ((u0, t0), (u1, t0), (u1, t1), (u0, t1))]
                    zz = [z + t * math.tan(math.radians(14)) + 0.06 for _, t in ((0, t0), (0, t0), (0, t1), (0, t1))]
                    m.quad("solar", *[(p[0], p[1], zp) for p, zp in zip(q, zz)], (0, 0), (1, 0), (1, 1), (0, 1))
        if raised:
            # the hall's raised hip roof over its north 40 m / ホール上部の高い屋根
            north = sorted([a2, b2, c2, d2], key=lambda q: -q[1])[:2]
            hall = Polygon(list(poly.exterior.coords)).intersection(
                box(-1e3, max(q[1] for q in north) - 40, 1e3, 1e3))
            if not hall.is_empty:
                bld = B.Builder.__new__(B.Builder)
                core = hall.buffer(-2.0, join_style="mitre")
                if not core.is_empty and core.geom_type == "Polygon":
                    m.prism("paint_white", core, z, z + h + 1.6, top=False)
                    bld.hip_roof(m, core, z + h + 1.6, 3.0, "roof_metal", overhang=0.8)

    def allegro_texts(self, m, poly):
        self.wall_text(m, poly, "Home of the Overcomers", side="east", height=0.75, z=EAVE - 3.4,
                       font="DejaVuSerif-Italic.ttf", mat="paint_charcoal", along=0.5)
        self.wall_text(m, poly, "PASIR RIS CREST SECONDARY SCHOOL", side="north", height=0.7,
                       z=EAVE - 2.6, font="DejaVuSans-Bold.ttf", mat="paint_charcoal", along=0.6)
        # the crest beside the name / 校章
        a, b, L, n = self.edge_facing(poly, "north")
        f = B.Facade(m, a, b, n)
        u = L * 0.6 - 13.5
        f.block("plaque_crest", u - 0.8, u + 0.8, 0.0, 0.05, EAVE - 3.4, EAVE - 1.8, top=False, bottom=False)
        m.quad("plaque_crest", f.P(u - 0.8, 0.06, EAVE - 3.4), f.P(u + 0.8, 0.06, EAVE - 3.4),
               f.P(u + 0.8, 0.06, EAVE - 1.8), f.P(u - 0.8, 0.06, EAVE - 1.8), (0, 0), (1, 0), (1, 1), (0, 1))

    def edge_facing(self, poly, side):
        want = {"north": (0, 1), "east": (1, 0), "south": (0, -1), "west": (-1, 0)}[side]
        return max(B.edges_of(poly), key=lambda e: float(e[3] @ np.array(want)) + 0.001 * e[2])

    def wall_text(self, m, poly, text, side, height, z, font, mat, along=0.5):
        from mesh import earcut
        a, b, L, n = self.edge_facing(poly, side)
        f = B.Facade(m, a, b, n)
        g = text_poly(text, height, font)
        u = L * along
        for p in polys(g):
            p = orient(p, 1.0)
            rings = [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]
            V = np.concatenate(rings)
            ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
            I = earcut.triangulate_float64(V.astype(np.float64), ends).reshape(-1, 3)
            P3 = [f.P(u + q[0], 0.05, z + q[1]) for q in V]
            A = np.array([P3[i] for i in I.ravel()]).reshape(-1, 3, 3)
            nrm = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
            flip = (nrm[:, :2] @ n) < 0
            I = np.where(flip[:, None], I[:, ::-1], I)
            m.tris(mat, P3, np.r_[n, 0.0], V / height, I)

    def rotunda(self, m):
        """Octagonal tower: stacked balconies, a canopy with a teal fascia, a
        two-tier green roof. / 八角形の塔。"""
        cx, cy = self.rot_c
        R = self.rot_r
        oct_ = lambda r, rot=22.5: Polygon([(cx + r * math.cos(math.radians(rot + 45 * k)),
                                             cy + r * math.sin(math.radians(rot + 45 * k))) for k in range(8)])
        core = oct_(R - 2.0)
        m.prism("glass_teal", core, Z0, Z0 + 3 * FLOOR, top=False)
        for k in range(1, 4):
            z = Z0 + k * FLOOR
            ring = oct_(R + (0.6 if k == 1 else 0.0)).difference(self.road_clip)
            m.prism("paint_white", ring, z - 0.35, z, top=True, bottom=True)
            for p in polys(ring):
                rr = p.exterior
                m.ring_walls("railing", rr, z, z + 0.08, outward=True)
                pts = list(rr.coords)
                for q0, q1 in zip(pts, pts[1:]):
                    for t in np.linspace(0, 1, 7)[:-1]:
                        x, y = q0[0] + (q1[0] - q0[0]) * t, q0[1] + (q1[1] - q0[1]) * t
                        m.box("railing", x, y, z, z + 1.05, 0.05, 0.05)
                m.ring_walls("railing", rr, z + 1.0, z + 1.07, outward=True)
        # the canopy at level 2, cantilevered, teal fascia / 2 階の張り出し庇
        can = oct_(R + 2.2).difference(self.road_clip)
        z = Z0 + FLOOR + 0.5
        m.flat("paint_white", can, z, 2.0)
        m.flat("concrete", can, z - 0.25, 2.0, up=False)
        for p in polys(can):
            m.ring_walls("paint_teal", p.exterior, z - 0.85, z, outward=True)
        for k in range(8):
            ang = math.radians(22.5 + 45 * k)
            m.box("paint_white", cx + (R - 0.4) * math.cos(ang), cy + (R - 0.4) * math.sin(ang), Z0,
                  Z0 + 3 * FLOOR, 0.45, 0.45, ang)
        # two-tier roof / 二段の屋根
        zr = Z0 + 3 * FLOOR + 0.3
        # the lower roof tier reaches only as far as the road allows / 下段屋根は道路手前まで
        reach = min(R + 1.6, self.L.road_all.distance(Point(cx, cy)) - 0.6)
        self.oct_tier(m, cx, cy, max(reach, R * 0.7), R * 0.55, zr, zr + 2.2)
        m.prism("paint_white", oct_(R * 0.55), zr + 2.2, zr + 3.4, top=False)
        self.oct_tier(m, cx, cy, R * 0.75, 0.3, zr + 3.4, zr + 6.0)

    def oct_tier(self, m, cx, cy, r0, r1, z0, z1):
        for k in range(8):
            a0, a1 = math.radians(22.5 + 45 * k), math.radians(22.5 + 45 * (k + 1))
            p = lambda r, a, z: (cx + r * math.cos(a), cy + r * math.sin(a), z)
            m.quad("roof_green", p(r0, a0, z0), p(r0, a1, z0), p(r1, a1, z1), p(r1, a0, z1))
            m.quad("paint_white", p(r0, a0, z0 - 0.4), p(r0, a1, z0 - 0.4), p(r0, a1, z0), p(r0, a0, z0))
            m.quad("concrete", p(r0, a1, z0 - 0.4), p(r0, a0, z0 - 0.4), p(r0 * 0.8, a0, z0 - 0.4),
                   p(r0 * 0.8, a1, z0 - 0.4))                    # soffit, facing down / 軒裏

    def sports_hall(self, m, poly):
        """White hall raised over an open ground floor, louvre bands, dark green
        hipped roof. / 屋内運動場。"""
        z1 = Z0 + 4.5
        z2 = Z0 + 12.0
        core = poly.buffer(-1.5, join_style="mitre")
        m.prism("paint_palegrey", core, Z0, z1, top=False)
        m.flat("concrete", poly, z1, 2.0, up=False)
        for a, b, L, n in B.edges_of(poly):
            f = B.Facade(m, a, b, n)
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 6.0) + 1)):
                f.block("paint_white", u - 0.3, u + 0.3, -0.6, 0.0, Z0, z1, top=False, bottom=False)
            f.face("paint_white", 0.0, L, z1, z2, 0.0, 2.0)
            for k in range(2):
                zz = z1 + 2.0 + k * 2.8
                f.block("louvre", 1.0, L - 1.0, -0.05, 0.05, zz, zz + 1.4, top=False, bottom=False, ends=False)
            f.block("paint_white", 0.0, L, 0.0, 0.3, z1 - 0.4, z1, ends=False)
        bld = B.Builder.__new__(B.Builder)
        m.flat("concrete", poly, z2 - 0.01)
        bld.hip_roof(m, poly, z2, 4.5, "roof_green", overhang=0.9)

    def link(self, m, line, w):
        """Covered link between blocks: a two-storey bridge. / 棟をつなぐ渡り廊下。"""
        strip = line.buffer(w / 2, cap_style="flat").difference(self.footprint)
        for p in polys(strip):
            for k in (1, 2):
                z = Z0 + k * FLOOR
                m.prism("paint_white", p, z - 0.35, z, top=True, bottom=True)
            m.flat("roof_metal", p, Z0 + 3 * FLOOR + 0.3, 2.0)
            m.flat("concrete", p, Z0 + 3 * FLOOR + 0.1, 2.0, up=False)
        for k in range(int(line.length // 6) + 1):
            q = line.interpolate(min(line.length, k * 6.0))
            if not self.footprint.buffer(0.5).contains(q):
                m.box("paint_white", q.x, q.y, Z0, Z0 + 3 * FLOOR + 0.1, 0.4, 0.4)

    def crescendo_walk(self, m):
        """The Crescendo covered walkway along the field: green roof, white posts. / 屋根付き通路。"""
        # stops short of the car-park aisle it ran over / 駐車通路の手前で止める
        line = self.crescendo.difference(self.drive_snapshot.buffer(2.0))
        if line.geom_type != "LineString":
            line = max(getattr(line, "geoms", [line]), key=lambda g: g.length)
        roof = line.buffer(1.6, cap_style="flat")
        m.flat("roof_linkway", roof, Z0 + 3.0, 2.0)
        m.flat("roof_linkway", roof, Z0 + 2.9, 2.0, up=False)
        for k in range(int(line.length // 4) + 1):
            q = line.interpolate(min(line.length, k * 4.0))
            m.box("paint_white", q.x, q.y, Z0, Z0 + 2.9, 0.15, 0.15)

    def gate_area(self, m):
        """Main gate: blue gates, the guard room, the name wall with the crest. / 正門。"""
        gx, gy = self.gate
        (ax, ay), (bx, by) = self.gate_span
        tx, ty = (bx - ax), (by - ay)
        tl = math.hypot(tx, ty) or 1.0
        tx, ty = tx / tl, ty / tl
        # piers beside the driveway, the blue leaves swung back along the fence,
        # off the lane / 門柱は車路の脇、青い門扉はフェンス沿いに開く
        for s, (px, py) in ((-1, (ax, ay)), (1, (bx, by))):
            px, py = px + s * tx * 0.5, py + s * ty * 0.5
            m.box("paint_white", px, py, Z0, Z0 + 2.3, 0.6, 0.6)
            for k in range(20):
                x, y = px + s * tx * (0.4 + k * 0.15), py + s * ty * (0.4 + k * 0.15)
                m.box("gate_blue", x, y, Z0 + 0.1, Z0 + 2.0, 0.04, 0.04)
            ex, ey = px + s * tx * 1.85, py + s * ty * 1.85
            m.box("gate_blue", ex, ey, Z0 + 1.9, Z0 + 2.0, 3.0, 0.06, math.atan2(ty, tx))
            m.box("gate_blue", ex, ey, Z0 + 0.1, Z0 + 0.2, 3.0, 0.06, math.atan2(ty, tx))
        # guard room west of the gate / 守衛室
        hx, hy = ax - tx * 7.0 - 1.0, ay - ty * 7.0 - 3.0
        m.box("paint_white", hx, hy, Z0, Z0 + 2.9, 4.2, 3.4)
        m.box("concrete", hx, hy, Z0 + 2.9, Z0 + 3.1, 4.8, 4.0)
        m.box("glass_dark", hx, hy + 1.71, Z0 + 1.0, Z0 + 2.2, 1.4, 0.04)
        # name wall east of the gate: white wall, green panel with the plaque / 校名の壁
        wx, wy = bx + tx * 8.5, by + ty * 8.5 - 0.8
        m.box("paint_white", wx, wy, Z0, Z0 + 2.4, 7.0, 0.5)
        m.box("paint_green", wx, wy + 0.27, Z0 + 0.35, Z0 + 2.3, 6.6, 0.05)
        m.quad("plaque_prcs", (wx - 2.6, wy + 0.31, Z0 + 0.7), (wx + 2.6, wy + 0.31, Z0 + 0.7),
               (wx + 2.6, wy + 0.31, Z0 + 2.1), (wx - 2.6, wy + 0.31, Z0 + 2.1), (0, 0), (1, 0), (1, 1), (0, 1))
        # "PRCS" letters standing in the planter / 植え込みの PRCS 文字
        pc = self.planter.centroid
        g = text_poly("PRCS", 0.9, "DejaVuSans-Bold.ttf")
        for p in polys(g):
            q = translate(p, pc.x, pc.y - 2.0)
            m.prism("paint_palegrey", q, Z0 + 0.4, Z0 + 0.62, top=True)
        # parking bays on the red bricks either side of the drop-off / 駐車枠
        # (satellite: bays on the red bricks north of Allegro) / 衛星写真: Allegro 北側
        for k in range(9):
            x = -72.0 + k * 2.5
            m.ribbon("paint_white", [(x, 345.6), (x, 350.6)], 0.08, Z0 + 0.03)
        # canopy by the field gate / 校庭脇の屋根
        (cx, cy), lw, ww, a = self.canopy
        m.box("paint_white", cx, cy, Z0 + 2.8, Z0 + 3.0, lw, ww, a)
        for s in (-1, 1):
            m.box("metal_grey", cx + s * lw / 2.4 * math.cos(a), cy + s * lw / 2.4 * math.sin(a), Z0, Z0 + 2.8,
                  0.12, 0.12)

    def flagpoles(self, m):
        pc = self.planter.centroid
        # inside the planter, behind the letters (off the drop-off loop) / 植え込みの中
        for k in range(3):
            x, y = pc.x - 1.5 + k * 1.5, pc.y + 1.2
            if self.L.road_all.buffer(0.5).contains(Point(x, y)):
                continue
            m.cylinder("metal_grey", x, y, Z0 + 0.45, Z0 + 9.0, 0.06, seg=6, top=True)

    def field_fence(self, m):
        """Tall chain-link round the field: posts, rails, a blue windbreak, a low
        retaining wall on Pasir Ris Drive 4. / 校庭の高いフェンスと青い防風ネット。"""
        ring = self.field.buffer(3.6, join_style="mitre").exterior
        c = list(ring.coords)
        for a, b in zip(c, c[1:]):
            for zz, mat in ((Z0 + 0.6, "windbreak_blue"),):
                m.wall(mat, a, b, Z0 + 0.6, Z0 + 2.6, 0.0, 2.0)
                m.wall(mat, b, a, Z0 + 0.6, Z0 + 2.6, 0.0, 2.0)
            for zz in (Z0 + 3.6, Z0 + 5.0, Z0 + 6.0):
                m.wall("metal_grey", a, b, zz, zz + 0.05, 0.0, 1.0)
                m.wall("metal_grey", b, a, zz, zz + 0.05, 0.0, 1.0)
        for k in range(int(ring.length // 3.0)):
            q = ring.interpolate(k * 3.0)
            m.box("metal_grey", q.x, q.y, Z0, Z0 + 6.0, 0.08, 0.08)
        east = max(B.edges_of(orient(self.field.buffer(3.6, join_style="mitre"), 1.0)), key=lambda e: e[3][0])
        a, b = east[0] + east[3] * 0.4, east[1] + east[3] * 0.4
        m.wall("concrete", tuple(a), tuple(b), Z0 - 0.1, Z0 + 0.9, 0.0, 2.0)
        m.flat("concrete", LineString([tuple(a), tuple(b)]).buffer(0.15, cap_style="flat"), Z0 + 0.9, 1.0)

    def site_fence(self, m):
        """Green chain-link fence with a hedge round the site, open at the gate. / 敷地の緑の金網フェンス。"""
        ring = self.site.buffer(-0.5, join_style="mitre").exterior
        gate = Point(self.gate).buffer(5.5)
        line = ring.difference(gate).difference(self.L.road_all.buffer(0.6)).difference(self.drive_snapshot.buffer(0.6))
        for g in getattr(line, "geoms", [line]):
            if g.geom_type != "LineString" or g.length < 1.0:
                continue
            c = list(g.coords)
            for a, b in zip(c, c[1:]):
                m.wall("fence_green", a, b, Z0, Z0 + 1.8, 0.0, 1.0)
                m.wall("fence_green", b, a, Z0, Z0 + 1.8, 0.0, 1.0)
            hedge = g.offset_curve(-0.7) if g.length > 2 else None
            if hedge is not None and not hedge.is_empty and hedge.geom_type == "LineString":
                m.prism("leaves_dark", hedge.buffer(0.4, cap_style="flat"), Z0, Z0 + 1.0, top=True)


def write_textures(tex_dir):
    """The name plaque and the crest, drawn here (green panel, white board,
    the crest: black mountain with snow on a yellow disc, blue sea, orange
    ribbon). / 校名板と校章を描く。"""
    from PIL import Image, ImageDraw, ImageFont
    fb = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

    def crest(size):
        im = Image.new("RGB", (size, size), (255, 255, 255))
        d = ImageDraw.Draw(im)
        s = size
        d.ellipse([s * .05, s * .02, s * .95, s * .92], fill=(250, 220, 40), outline=(20, 20, 20), width=max(2, s // 60))
        d.polygon([(s * .2, s * .72), (s * .52, s * .14), (s * .82, s * .72)], fill=(15, 15, 15))
        d.polygon([(s * .52, s * .14), (s * .44, s * .32), (s * .5, s * .28), (s * .56, s * .4), (s * .6, s * .3)],
                  fill=(255, 255, 255))
        for k in range(3):
            y = s * (.70 + k * .04)
            d.arc([s * .12, y - s * .05, s * .88, y + s * .05], 0, 180, fill=(40, 90, 190), width=max(2, s // 40))
        d.polygon([(s * .08, s * .78), (s * .92, s * .78), (s * .86, s * .9), (s * .14, s * .9)], fill=(240, 130, 30))
        f = ImageFont.truetype(fb, max(8, s // 18))
        d.text((s * .22, s * .8), "Pursue & Realise", fill=(20, 20, 20), font=f)
        return im

    crest(512).save(os.path.join(tex_dir, "plaque_crest.png"))
    W, H = 1024, 276
    im = Image.new("RGB", (W, H), (245, 245, 240))
    d = ImageDraw.Draw(im)
    d.rectangle([4, 4, W - 5, H - 5], outline=(30, 90, 50), width=6)
    f1, f2 = ImageFont.truetype(fb, 34), ImageFont.truetype(fb, 30)
    d.text((40, 40), "SEKOLAH MENENGAH PASIR RIS CREST", fill=(20, 20, 20), font=f1)
    d.text((40, 120), "PASIR RIS CREST SECONDARY SCHOOL", fill=(20, 20, 20), font=f2)
    im.paste(crest(220), (W - 250, 28))
    im.save(os.path.join(tex_dir, "plaque_prcs.png"))
    return {"plaque_crest": ("plaque_crest", None, 0.5, 0.0, False),
            "plaque_prcs": ("plaque_prcs", None, 0.5, 0.0, False)}
