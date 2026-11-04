"""Loyang View Secondary School, modelled in detail (maps/.venv).

Sources (2025-2026): Google Maps satellite traced at zoom 20 by the Photo
Getter (build/PasirRis/lvss_photos, trace in build/PasirRis/lvss_trace.json);
Street View on Pasir Ris Street 11 and 12 (Mar 2025); photos of the gable
portal, the hall and the field (Facebook / TikTok via Google Images). The
school publishes no site plan, so block names are descriptive. Nothing is
copied as texture: colours and forms only.

  blocks       two families. The 1990 blocks by the main gate: red-brown face
               brick with cream bands, yellow window frames, an orange fascia
               carrying the school name, terracotta hip roofs, and the tall
               dark-brick stair tower with orange bands and the vertical
               Chinese name. The PRIME (2004) and later blocks: cream render,
               maroon ground-floor columns, yellow window frames, orange end
               walls and stair cores, open corridors facing the courts, flat
               roofs covered in solar panels (some terracotta hip roofs)
  portal       the maroon gable portal with the crest, a yellow frame grid and
               the name boards, at the south gable end of the terracotta
               gable block, facing the SW hard court (its position is a guess:
               the photo shows it over a paved square)
  halls        the school hall (terracotta gable, cream walls, orange end
               panels); the indoor sports hall (cream, maroon frame lines,
               yellow/orange chequer panels, clerestory louvres, open ground
               floor under a grey band, flat roof with a red inset)
  grounds      dark green astro pitch with a tall ball-stop fence, the blue
               court, the basketball court, the central netball court on red,
               the grey hard court with netball lines in the SW carpark, two
               garden courtyards with trees, covered walkways, a link bridge
  gates        main gate on St 11 (brick piers with white caps, yellow sliding
               gates, brick sign wall with the name board), back gate on St 11
               (grey steel gate, 2-storey gatehouse), east gate on St 12
               (green steel gate); green chain-link fence with a hedge

The satellite imagery is oblique: roofs appear north of their bases, so every
block outline traced on its roof is moved back by the measured lean
(lvss_trace.json "lean") times the height of that roof edge.

Coordinates: the map frame (x east, y north, metres).

洛阳中学（Loyang View Secondary School）の詳細モデル。
"""

import json
import math
import os

import numpy as np
from shapely.affinity import translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from mesh import Mesh, polys

NAME = "Loyang View Secondary School"
TAG = "LVSS"
TRACE = "lvss_trace.json"
FLOOR = 3.6
Z0 = 0.15
DEJAVU = "/usr/share/fonts/truetype/dejavu/"
CJK = "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc"
ZH_NAME = "洛阳中学"
# end-wall colours seen in Street View (default orange) / 妻壁の色（既定は橙）
ACCENT = {"long_block": "paint_cream", "middle_wing": "paint_yellow", "east_annex": "paint_yellow",
          "southwest_block": "paint_cream"}
ROAD_CLEAR = 1.6          # footprint to road: facade depth (1.0) + margin / 建物と道路の間隔


def text_poly(text, height, font, aspect=1.0):
    """Text as polygons, ``height`` tall, centred, x reading direction (font: a
    full path). / 文字の多角形。"""
    from PIL import Image, ImageDraw, ImageFont
    from shapely.affinity import scale
    f = ImageFont.truetype(font, 110)
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
    g = unary_union(cells).simplify(0.8)
    minx, miny, maxx, maxy = g.bounds
    k = height / (maxy - miny)
    return scale(translate(g, -(minx + maxx) / 2, -(miny + maxy) / 2), k * aspect, k, origin=(0, 0))


def builder():
    """A roof helper without a layout (only its roof methods are used). / 屋根用。"""
    return B.Builder.__new__(B.Builder)


def oquad(m, mat, P, centre, uv=((0, 0), (1, 0), (1, 1), (0, 1))):
    """A quad facing away from ``centre`` (a point inside the solid it bounds).
    中心から外向きの四角形。"""
    P = [np.asarray(p, np.float64) for p in P]
    n = np.cross(P[1] - P[0], P[3] - P[0])
    c = (P[0] + P[1] + P[2] + P[3]) / 4.0
    if float(n @ (c - np.asarray(centre, np.float64))) < 0:
        P = [P[1], P[0], P[3], P[2]]
        uv = (uv[1], uv[0], uv[3], uv[2])
    m.quad(mat, *P, *uv)


def mapped_flat(m, mat, poly, rect, z):
    """A horizontal polygon (any shape, holes allowed) facing up, its texture
    mapped onto the rectangle ``rect`` (4 corners, CCW: (0,0) (1,0) (1,1)
    (0,1)), so a court cut by a road keeps its markings in place.
    任意形状の水平面に、矩形基準でテクスチャを貼る（上向き）。"""
    import shapely
    o = np.asarray(rect[0], np.float64)
    eu = np.asarray(rect[1], np.float64) - o
    ev = np.asarray(rect[3], np.float64) - o
    Minv = np.linalg.inv(np.column_stack([eu, ev]))
    for p in polys(poly):
        if p.area < 1e-4:
            continue
        tris = shapely.get_parts(shapely.constrained_delaunay_triangles(p))
        if len(tris) == 0:
            continue
        C = shapely.get_coordinates(shapely.get_exterior_ring(tris)).reshape(-1, 4, 2)[:, :3, :]
        a, b, c = C[:, 0], C[:, 1], C[:, 2]
        cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        keep = np.abs(cross) > 1e-10
        C = C[keep]
        flip = cross[keep] < 0
        C[flip] = C[flip][:, ::-1]
        V = C.reshape(-1, 2)
        UV = (V - o) @ Minv.T
        m.tris(mat, np.column_stack([V, np.full(len(V), z)]), (0, 0, 1), UV, np.arange(len(V)).reshape(-1, 3))


def solid(m, mat, bot, top):
    """A closed six-sided solid from its bottom and top rings of 4 corners
    (same order), every face facing out. / 6 面体（全面外向き）。"""
    c = (np.sum(np.asarray(bot, np.float64), 0) + np.sum(np.asarray(top, np.float64), 0)) / 8.0
    oquad(m, mat, bot, c)
    oquad(m, mat, top, c)
    for i in range(4):
        j = (i + 1) % 4
        oquad(m, mat, [bot[i], bot[j], top[j], top[i]], c)


class LVSS(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.drive_snapshot = L.driveways          # before build.py clears the site / 消去前の駐車通路
        self.meta = L.meta
        fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        mx, my = T.get("mpp_x", T["mpp"]), T.get("mpp_y", T["mpp"])

        def P(tile, px, py):
            lat, lon = T["tiles"][tile]
            cx, cy = fr.xy(lon, lat)
            return (cx + (px - T["centre"][0]) * mx, cy - (py - T["centre"][1]) * my)
        self.P = P
        poly = lambda d: orient(Polygon([P(d["tile"], *q) for q in d["pts"]]), 1.0)
        a = math.radians(T["grid_deg"])
        self.ax_u = np.array([math.cos(a), math.sin(a)])
        self.ax_v = np.array([-math.sin(a), math.cos(a)])
        lean = T["lean"]
        self.lean = (lean["dx_per_m"], lean["dy_per_m"])
        self.road = L.road_all
        self.carriageway = L.carriageway
        osm_site = [p for p, t in L.osm.areas(lambda t: t.get("name") == NAME)][0]
        self.boundary = orient(Polygon([P(t, x, y) for t, x, y in T["boundary"]["pts"]]).buffer(0), 1.0)
        site = unary_union([osm_site, self.boundary])
        self.site = site if site.geom_type == "Polygon" else max(polys(site), key=lambda g: g.area)
        # blocks: roof outline as seen, moved back by the lean, cut clear of the
        # road network (roads win) / 屋根の輪郭を傾き分戻し、道路から離す
        keep = self.road.buffer(ROAD_CLEAR)
        self.blocks = {}
        self.cut = {}
        for k, b in T["blocks"].items():
            roof = poly(b)
            foot = self.unlean(roof, b["h"])
            if b.get("overhang"):                     # eaves traced: walls stand in by the overhang / 軒の出だけ内側
                foot = foot.buffer(-b["overhang"], join_style="mitre")
            q = foot.difference(keep)
            parts = polys(q)
            if not parts:
                print("landmark %s: block %s dropped (on the road network)" % (TAG, k))
                continue
            q = max(parts, key=lambda g: g.area)
            if q.area < 0.5 * foot.area:
                print("landmark %s: block %s dropped (%.0f %% of it on the road network)"
                      % (TAG, k, 100 * (1 - q.area / foot.area)))
                self.cut[k] = foot.area
                continue
            if q.area < foot.area - 0.5:
                self.cut[k] = foot.area - q.area
                print("landmark %s: %s cut back %.1f m2 from the road network" % (TAG, k, foot.area - q.area))
            q = orient(q.simplify(0.05), 1.0)
            self.blocks[k] = dict(poly=q, storeys=b["storeys"], style=b["style"], roof=b["roof"], h=b["h"],
                                  roof_seen=roof)
        self.footprint = unary_union([b["poly"] for b in self.blocks.values()])
        # ground features: as seen / 地上のものは見たまま
        self.netball = poly(T["netball_court"])
        self.red_area = poly(T["red_area"])
        self.bb = poly(T["bb_court"])
        self.blue = poly(T["blue_court"])
        self.astro = poly(T["astro"])
        self.pitch_lines = poly(T["pitch_lines"])
        self.carpark = poly(T["carpark"])
        self.parade = poly(T["parade_court"])
        self.field = self.astro
        self.courtyards = []
        for c in T["courtyards"]:
            r = poly(c)
            g = unary_union([r, self.unlean(r, c["h"])]).difference(self.footprint.buffer(0.3))
            self.courtyards.append(g)
        self.walkways = [(LineString([P(w["tile"], *q) for q in w["pts"]]), w["w_px"] * T["mpp"]) for w in T["walkways"]]
        self.links = [(LineString([P(w["tile"], *q) for q in w["pts"]]), w["w_px"] * T["mpp"], w.get("levels", 3))
                      for w in T["links"]]
        self.flags = poly(T["flagpoles"]).centroid
        self.centre = self.red_area.centroid
        self.site_c = self.boundary.centroid
        g = T["gates"]
        self.back_gate = [P(g["back"]["tile"], *q) for q in g["back"]["pts"]]
        self.east_gate = [P(g["east"]["tile"], *q) for q in g["east"]["pts"]]
        near = P(g["main"]["near"][1], *g["main"]["near"][0])
        # the main gate: where the driveway (an OSM service road) crosses the
        # boundary / 正門: 校内車路と敷地境界の交点
        span = self.boundary.exterior.intersection(self.road.buffer(0.8))
        pts = [q for gg in getattr(span, "geoms", [span]) for q in getattr(gg, "coords", [])]
        pts = [q for q in pts if math.dist(q, near) < 18.0]
        if len(pts) >= 2:
            a0 = max(pts, key=lambda q: q[1])
            b0 = min(pts, key=lambda q: q[1])
            self.gate_span = (a0, b0)
        else:
            self.gate_span = ((near[0], near[1] + 4.0), (near[0], near[1] - 4.0))
        self.gate = ((self.gate_span[0][0] + self.gate_span[1][0]) / 2, (self.gate_span[0][1] + self.gate_span[1][1]) / 2)
        self.fit_parade()

    def unlean(self, g, h):
        return translate(g, -self.lean[0] * h, -self.lean[1] * h)

    def fit_parade(self, clear=0.3):
        """The road wins: the network's driveway (an OSM service road) runs
        diagonally through the SW hard court and no free place nearby holds a
        30 x 15 m court (the strip west of the driveway is 10-12 m wide), so the
        court keeps its traced place and its paint stops at the driveway (the
        court is drawn clipped, its texture still mapped on the whole court).
        道路優先: コートの塗装は校内車路の手前で止める（位置はそのまま）。"""
        self.parade_full = self.parade
        cut = self.parade.difference(self.road.buffer(clear))
        print("landmark %s: parade court paint cut at the driveway (%.0f of %.0f m2 kept)"
              % (TAG, cut.area, self.parade.area))

    # -- what build.py needs / build.py 用
    def inner(self):
        return self.site.buffer(-0.6, join_style="mitre")

    def keepout(self):
        """Where the generic trees must not go. / 汎用の木を置かない所。"""
        return unary_union([self.footprint.buffer(3.0), self.astro.buffer(5.0),
                            unary_union([self.netball, self.red_area, self.bb, self.blue, self.parade]).buffer(1.0),
                            self.carpark.intersection(self.inner())])

    # -- ground / 地面
    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        T = lambda g: g.intersection(tile)
        inner = self.boundary.buffer(-0.3).union(self.inner())
        road = self.road.buffer(0.05)
        courts = unary_union([self.netball, self.bb, self.blue, self.astro, self.parade, self.red_area])
        paths = unary_union([self.footprint.buffer(2.2, join_style="mitre")]
                            + [l.buffer(w / 2 + 0.6, cap_style="flat") for l, w in self.walkways])
        lot = self.carpark.intersection(inner)
        # around the gatehouse and the back gate: hard standing / 裏門まわりは舗装
        lot = unary_union([lot, Polygon(self.back_gate + [self.blocks["guard_house"]["poly"].centroid.coords[0]])
                           .buffer(4.0)]) if "guard_house" in self.blocks else lot
        lot = lot.intersection(inner)
        hard = paths.difference(courts).difference(lot).intersection(inner).difference(road)
        lot = lot.difference(courts).difference(self.footprint).difference(road)
        green = inner.difference(hard).difference(lot).difference(courts).difference(self.footprint).difference(road)
        m.flat("grass_park", T(green), z + 0.005, 4.0)
        m.flat("concrete_path", T(hard.difference(self.footprint)), z + 0.01, 2.0)
        m.flat("asphalt_lot", T(lot), z + 0.01, 4.0)
        # bay lines along the south fence of the carpark / 駐車枠
        if tile.contains(self.carpark.centroid):
            self.bays(m, z + 0.016, lot)
        m.flat("parade_red", T(self.red_area.difference(self.netball).difference(self.footprint).difference(road)),
               z + 0.02, 3.0)
        lines = self.pitch_lines.buffer(1.0, join_style="mitre")      # texture lines sit 1 m in / 線は 1 m 内側
        m.flat("astro_lvss", T(self.astro.difference(lines).difference(road)), z + 0.02, 3.0)
        for g, mat in ((self.netball, "court_netball_lv"), (self.bb, "court_bb_lv"), (self.blue, "court_bb_blue"),
                       (lines, "pitch_lvss"), (self.parade, "court_parade_lv")):
            if tile.contains(g.centroid):              # once, in its own tile / 自分のタイルに 1 回だけ
                self.court_quad(m, g, mat, z + 0.03)
        return m

    def court_quad(self, m, poly, mat, z):
        """A court texture over the court's rectangle (u along its length),
        clipped where it would meet the road network. / コート（道路部分は切る）。"""
        c = list(orient(poly.minimum_rotated_rectangle, 1.0).exterior.coords)[:4]
        if math.dist(c[1], c[2]) > math.dist(c[0], c[1]):
            c = c[1:] + c[:1]
        rect = Polygon(c)
        keep = rect.difference(self.road.buffer(0.3))
        if keep.area > rect.area - 0.01:
            P = [(q[0], q[1], z) for q in c]
            m.quad(mat, P[0], P[1], P[2], P[3], (0, 0), (1, 0), (1, 1), (0, 1))
        else:
            mapped_flat(m, mat, keep, c, z)

    def bays(self, m, z, lot):
        """Parking bays along the carpark's south edge, inside the hard standing. / 駐車枠。"""
        c = self.carpark.minimum_rotated_rectangle
        e = B.edges_of(orient(c, 1.0))
        a, b, Ln, n = min(e, key=lambda q: (q[0][1] + q[1][1]))      # the southern edge / 南辺
        t = (b - a) / Ln
        for k in range(int(Ln // 2.5)):
            p0 = a + t * (k * 2.5) - n * 1.0
            p1 = p0 - n * 5.0
            seg = LineString([tuple(p0), tuple(p1)])
            if lot.buffer(-0.3).contains(seg):
                m.ribbon("paint_white", [tuple(p0), tuple(p1)], 0.1, z)

    # -- buildings / 建物
    def buildings_mesh(self, tile):
        m = Mesh()
        x0, y0, x1, y1 = tile.bounds
        inside = lambda x, y: x0 <= x < x1 and y0 <= y < y1
        for k, b in self.blocks.items():
            c = b["poly"].centroid
            if inside(c.x, c.y):
                getattr(self, "b_" + b["style"])(m, b, k)
        if inside(self.site_c.x, self.site_c.y):
            self.main_gate(m)
            self.side_gates(m)
            for l, w in self.walkways:
                self.walkway(m, l, w)
            for l, w, n in self.links:
                self.link(m, l, w, n)
            self.flagpoles(m)
            self.site_fence(m)
            self.trees(m)
        f = self.astro.centroid
        if inside(f.x, f.y):
            self.pitch_fence(m)
        return m

    # facade parts / 外壁の部品
    def corridor_edges(self, poly):
        """Edges facing the middle of the school (the courts) carry the open
        corridors. / 中央（コート）を向く辺に共用廊下。"""
        out = set()
        for i, (a, b, Ln, n) in enumerate(B.edges_of(poly)):
            v = np.array([self.centre.x, self.centre.y]) - (a + b) / 2
            if Ln > 12 and float(n @ v) > 0:
                out.add(i)
        return out

    def ground_floor(self, f, Ln, z0, z1, wall="paint_cream", col="paint_maroon", depth=1.6):
        """Ground floor set back behind maroon columns: doors and windows in the
        shade. / 1 階: 栗色の柱の奥に扉と窓。"""
        f.face(wall, 0.0, Ln, z0, z1, -depth)
        f.side(wall, 0.0, -depth, 0.0, z0, z1, 1)
        f.side(wall, Ln, -depth, 0.0, z0, z1, -1)
        f.hplane("concrete", 0.0, Ln, -depth, 0.0, z1 - 0.001, False)
        for u0, u1 in B.bays(Ln, 7.2, 0.6):
            mid = (u0 + u1) / 2
            f.face("door_wood", mid - 2.4, mid - 1.4, z0, z0 + 2.3, -depth + 0.02)
            f.face("glass_yellow_3", mid - 0.6, mid + 2.6, z0 + 0.9, z0 + 2.5, -depth + 0.02)
        for u in np.linspace(0.3, Ln - 0.3, max(2, int(Ln // 7.2) + 1)):
            f.block(col, u - 0.3, u + 0.3, -0.6, 0.0, z0, z1, top=False, bottom=False)
        f.block(wall, 0.0, Ln, 0.0, 0.12, z1 - 0.45, z1, ends=False)

    def classroom(self, f, Ln, z0, z1, wall="paint_cream", shade="paint_cream"):
        """Yellow-framed windows, a sunshade ledge over them. / 黄色い枠の窓と庇。"""
        opens = [(u0 + 0.4, u1 - 0.4, z0 + 0.9, z0 + 2.7) for u0, u1 in B.bays(Ln, 3.6, 0.6)]
        f.wall_with_openings(wall, z0, z1, opens, 2.0)
        for o in opens:
            f.window(*o, wall, glass="glass_yellow", mullions=2)
        if opens:
            f.block(shade, 0.4, Ln - 0.4, 0.0, 0.6, z0 + 2.85, z0 + 2.97)
        f.block(wall, 0.0, Ln, 0.0, 0.05, z0, z0 + 0.22, ends=False)

    def corridor(self, f, Ln, z0, z1, depth=2.2, wall="paint_cream"):
        """Open corridor: slab edge, solid cream parapet with a rail, the class
        wall behind with doors and windows, columns. / 開放廊下。"""
        f.face(wall, 0.0, Ln, z0, z0 + 0.3, 0.0)
        f.block(wall, 0.0, Ln, -0.15, 0.0, z0 + 0.3, z0 + 1.0, top=True, bottom=False, ends=False)
        f.block("railing", 0.0, Ln, -0.12, -0.04, z0 + 1.05, z0 + 1.1, ends=False)
        for u in np.arange(0.6, Ln, 1.8):
            f.block("railing", u - 0.03, u + 0.03, -0.1, -0.06, z0 + 1.0, z0 + 1.05, top=False, bottom=False)
        f.face("paint_palegrey", 0.0, Ln, z0 + 0.3, z1, -depth)
        f.side(wall, 0.0, -depth, 0.0, z0, z1, 1)
        f.side(wall, Ln, -depth, 0.0, z0, z1, -1)
        f.hplane("concrete", 0.0, Ln, -depth, 0.0, z1 - 0.001, False)
        f.hplane("void_floor", 0.0, Ln, -depth, -0.15, z0 + 0.3, True)
        for u0, u1 in B.bays(Ln, 7.2, 0.6):
            mid = (u0 + u1) / 2
            f.face("door_wood", mid - 2.6, mid - 1.6, z0 + 0.3, z0 + 2.4, -depth + 0.02)
            f.face("glass_yellow_3", mid - 1.0, mid + 2.6, z0 + 1.2, z0 + 2.6, -depth + 0.02)
        for u in np.linspace(0.25, Ln - 0.25, max(2, int(Ln // 7.2) + 1)):
            f.block(wall, u - 0.25, u + 0.25, -depth, 0.0, z0 + 1.0, z1, top=False, bottom=False)

    def end_wall(self, f, Ln, z0, z1, accent, screens=False):
        """Accent end wall (orange or yellow) with a slot window; perforated
        screen panels on the east end. / 妻壁: アクセント色と縦長の窓。"""
        mid = Ln / 2
        o = (mid - 0.7, mid + 0.7, z0 + 0.9, z0 + 2.5)
        if Ln > 3.0:
            f.wall_with_openings(accent, z0, z1, [o], 2.0)
            f.window(*o, accent, glass="glass_dark", mullions=0)
        else:
            f.face(accent, 0.0, Ln, z0, z1, 0.0, 2.0)
        if screens and Ln > 7.0:
            for u in (1.0, Ln - 3.2):
                f.block("perforated_lvss", u, u + 2.2, 0.15, 0.3, z0 + 0.6, z0 + 2.9)
                f.block("paint_cream", u - 0.1, u + 2.3, 0.0, 0.7, z0 + 2.9, z0 + 3.0)
        f.block("paint_cream", 0.0, Ln, 0.0, 0.08, z0, z0 + 0.2, ends=False)

    # roofs / 屋根
    def flat_roof(self, m, poly, z, solar=True, wall="paint_cream", coping="paint_maroon"):
        bld = builder()
        bld.flat_roof(m, poly, z, wall, parapet=1.0)
        m.ring_walls(coping, poly.exterior, z + 0.88, z + 1.0, outward=True)
        if solar:
            self.solar(m, poly.buffer(-1.4, join_style="mitre"), z + 0.25)

    def solar(self, m, area, z):
        """Rows of solar panels tilted 10 deg, along the roof's long axis. / 太陽光パネルの列。"""
        if area.is_empty:
            return
        r = orient(area.minimum_rotated_rectangle, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, b, d = c[k], c[(k + 1) % 4], c[(k + 3) % 4]
        along = (b - a) / np.linalg.norm(b - a)
        across = (d - a) / np.linalg.norm(d - a)
        W = np.linalg.norm(d - a)
        Lr = np.linalg.norm(b - a)
        slope = math.tan(math.radians(10))
        t = 0.2
        while t + 1.0 < W - 0.2:
            q = [a - along * 1 + across * t, b + along * 1 + across * t, b + along * 1 + across * (t + 1.0),
                 a - along * 1 + across * (t + 1.0)]
            strip = Polygon([tuple(p) for p in q]).intersection(area)
            base = a + across * t
            zf = (lambda bx, by, base=base: z + 0.05 + slope *
                  ((bx - base[0]) * across[0] + (by - base[1]) * across[1]))
            for p in polys(strip):
                if p.area > 1.0:
                    m.flat("solar", p, z, 1.0, zf=zf)
            t += 1.7
        _ = Lr

    def hip(self, m, poly, z, mat="roof_terracotta", h=None, overhang=0.8):
        r = poly.minimum_rotated_rectangle
        cc = list(r.exterior.coords)
        short = min(math.dist(cc[0], cc[1]), math.dist(cc[1], cc[2]))
        h = short / 2 * math.tan(math.radians(22)) if h is None else h
        if not r.buffer(overhang + 0.3).disjoint(self.road):
            self.flat_roof(m, poly, z, solar=False)
            return
        m.flat("concrete", poly, z - 0.01, 2.0)
        builder().hip_roof(m, poly, z, h, mat, overhang=overhang)

    def gable(self, m, poly, z, wall, mat="roof_terracotta", deg=17):
        r = poly.minimum_rotated_rectangle
        cc = list(r.exterior.coords)
        short = min(math.dist(cc[0], cc[1]), math.dist(cc[1], cc[2]))
        h = short / 2 * math.tan(math.radians(deg))
        m.flat("concrete", poly, z - 0.01, 2.0)
        builder().gable_roof(m, poly, z, h, mat, wall, overhang=0.8)
        return h

    # block styles / 棟の種類
    def b_cream(self, m, b, name):
        poly, st = b["poly"], b["storeys"]
        eave = Z0 + st * FLOOR
        corr = self.corridor_edges(poly)
        accent = ACCENT.get(name, "paint_orange")
        east = max(B.edges_of(poly), key=lambda e: e[3][0])
        for i, (a, bb, Ln, n) in enumerate(B.edges_of(poly)):
            f = B.Facade(m, a, bb, n)
            end = Ln < 16.0
            for k in range(st):
                z0 = Z0 + k * FLOOR
                z1 = z0 + FLOOR
                if end:
                    self.end_wall(f, Ln, z0, z1, accent, screens=(name == "long_block" and np.allclose(a, east[0])))
                elif k == 0:
                    self.ground_floor(f, Ln, z0, z1)
                elif i in corr:
                    self.corridor(f, Ln, z0, z1)
                else:
                    self.classroom(f, Ln, z0, z1)
            f.block("paint_cream", 0.0, Ln, 0.0, 0.2, eave - 0.45, eave, ends=False)
        self.roof_of(m, b, eave)
        if name == "portal_block":
            self.portal(m, poly, eave)

    def roof_of(self, m, b, eave):
        r = b["roof"]
        if r == "flat_solar":
            self.flat_roof(m, b["poly"], eave)
        elif r == "hip_terracotta":
            self.hip(m, b["poly"], eave)
        elif r == "gable_terracotta":
            self.gable(m, b["poly"], eave, "paint_cream")
        else:
            self.flat_roof(m, b["poly"], eave, solar=False)

    def b_core(self, m, b, name):
        """Stair / lift core: orange panels, cream frame. / 階段室: 橙のパネル。"""
        poly, st = b["poly"], b["storeys"]
        eave = Z0 + st * FLOOR
        east = max(B.edges_of(poly), key=lambda e: e[3][0])
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            for k in range(st):
                z0 = Z0 + k * FLOOR
                self.end_wall(f, Ln, z0, z0 + FLOOR, "paint_orange",
                              screens=name == "long_stair" and np.allclose(a, east[0]))
            f.block("paint_cream", 0.0, Ln, 0.0, 0.2, eave - 0.45, eave, ends=False)
        self.flat_roof(m, poly, eave, solar=False)

    def b_link(self, m, b, name):
        """Two-storey link wing: open covered way on columns below, corridor
        above. / 2 層の渡り棟: 1 階は柱だけの通路。"""
        poly = b["poly"]
        eave = Z0 + 2 * FLOOR
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            if Ln < 10:
                f.face("paint_cream", 0.0, Ln, Z0, eave, 0.0, 2.0)
                continue
            self.ground_floor(f, Ln, Z0, Z0 + FLOOR, depth=2.4)
            self.corridor(f, Ln, Z0 + FLOOR, eave)
            f.block("paint_cream", 0.0, Ln, 0.0, 0.2, eave - 0.45, eave, ends=False)
        self.flat_roof(m, poly, eave, solar=False)

    def b_brick(self, m, b, name):
        """The 1990 brick block by the main gate: brick with cream bands, yellow
        frames, the orange fascia with the name, terracotta hip roof.
        正門前の煉瓦の棟。"""
        poly, st = b["poly"], b["storeys"]
        eave = Z0 + st * FLOOR
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            for k in range(st):
                z0 = Z0 + k * FLOOR
                z1 = z0 + FLOOR
                opens = [(u0 + 0.6, u1 - 0.6, z0 + 1.0, z0 + 2.6) for u0, u1 in B.bays(Ln, 3.6, 0.6)]
                f.wall_with_openings("paint_brickred", z0, z1, opens, 2.0)
                for o in opens:
                    f.window(*o, "paint_brickred", glass="glass_yellow", mullions=1, depth=0.2)
                band = (z0, z0 + 0.9) if k == 0 else (z0 - 0.15, z0 + 0.3)
                f.block("paint_cream", 0.0, Ln, 0.0, 0.08, band[0] if k else z0, band[1], ends=False)
            # orange fascia under the eaves / 軒下の橙の帯
            f.block("paint_orange", -0.1, Ln + 0.1, 0.0, 0.18, eave - 1.3, eave + 0.25)
        self.hip(m, poly, eave + 0.25, h=4.0)
        # the name on the fascia facing the main gate (west) / 正門側の校名
        a, bb, Ln, n = self.edge_facing(poly, "west")
        f = B.Facade(m, a, bb, n)
        self.wall_text(m, f, Ln, "LOYANG VIEW SECONDARY SCHOOL", DEJAVU + "DejaVuSans-Bold.ttf", 0.55,
                       Ln / 2 + 1.0, eave - 0.6, "paint_navy", 0.2)
        u = Ln / 2 - 5.6
        self.shield(m, f, u, eave - 0.55, 0.5, 0.2, "crest_lvss")

    def b_tower(self, m, b, name):
        """Dark-brick stair tower: orange bands, dark glass slot, the vertical
        Chinese name, a small terracotta cap. / 濃い煉瓦の階段塔と縦書きの校名。"""
        poly = b["poly"]
        top = b["h"] - 1.0
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            o = (Ln / 2 - 0.6, Ln / 2 + 0.6, Z0 + 1.0, top - 1.5)
            f.wall_with_openings("brick_dark", Z0, top, [o] if Ln > 4 else [], 2.0)
            if Ln > 4:
                f.window(*o, "brick_dark", glass="glass_dark", mullions=0, depth=0.3)
            for k in range(1, 5):
                z = Z0 + k * FLOOR + 0.2
                f.block("paint_orange", -0.05, Ln + 0.05, 0.0, 0.12, z, z + 0.7)
            f.block("paint_orange", -0.05, Ln + 0.05, 0.0, 0.12, top - 0.6, top)
        self.flat_roof(m, poly, top, solar=False, wall="brick_dark", coping="paint_orange")
        core = poly.buffer(-1.0, join_style="mitre")
        if not core.is_empty and core.geom_type == "Polygon":
            m.prism("brick_dark", core, top, top + 1.4, top=False)
            builder().hip_roof(m, core, top + 1.4, 1.6, "roof_terracotta", overhang=0.4)
        # vertical name, beside the glass slot on the west face / 縦書きの校名
        a, bb, Ln, n = self.edge_facing(poly, "west")
        f = B.Facade(m, a, bb, n)
        for k, ch in enumerate(ZH_NAME):
            self.wall_text(m, f, Ln, ch, CJK, 0.9, Ln / 2 + 1.55, top - 3.0 - k * 1.25, "paint_cream", 0.13)

    def b_porch(self, m, b, name):
        """Low covered porch: maroon posts, terracotta gable. / 屋根付きの車寄せ通路。"""
        poly = b["poly"]
        z = Z0 + 3.0
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            if Ln > 6:
                for u in np.linspace(0.3, Ln - 0.3, max(2, int(Ln // 4.0) + 1)):
                    f.block("paint_maroon", u - 0.18, u + 0.18, -0.36, 0.0, Z0, z, top=False, bottom=False)
            f.block("paint_cream", 0.0, Ln, -0.3, 0.0, z - 0.35, z, ends=False)
        m.flat("concrete", poly, z - 0.35, 2.0, up=False)
        self.gable(m, poly, z, "paint_cream", deg=20)

    def b_small(self, m, b, name):
        poly = b["poly"]
        z = Z0 + 3.2
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face("paint_cream", 0.0, Ln, Z0, z, 0.0, 2.0)
            if Ln > 3:
                f.face("door_wood", Ln / 2 - 0.5, Ln / 2 + 0.5, Z0, Z0 + 2.2, 0.02)
        self.hip(m, poly, z, h=1.6)

    def b_gatehouse(self, m, b, name):
        """Two-storey gatehouse with single-storey wings, terracotta hip roofs. / 守衛所。"""
        poly = b["poly"]
        r = orient(poly.minimum_rotated_rectangle, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, bq, d = c[k], c[(k + 1) % 4], c[(k + 3) % 4]
        Ln = np.linalg.norm(bq - a)
        t = (bq - a) / Ln
        s = d - a
        part = lambda u0, u1: Polygon([tuple(a + t * u0), tuple(a + t * u1), tuple(a + t * u1 + s),
                                       tuple(a + t * u0 + s)]).intersection(poly)
        for u0, u1, z in ((0.0, Ln * 0.3, Z0 + 3.2), (Ln * 0.3, Ln * 0.7, Z0 + 6.4), (Ln * 0.7, Ln, Z0 + 3.2)):
            p = part(u0, u1)
            if p.is_empty or p.geom_type != "Polygon":
                continue
            p = orient(p, 1.0)
            for a2, b2, L2, n2 in B.edges_of(p):
                f = B.Facade(m, a2, b2, n2)
                fl = int(round((z - Z0) / 3.2))
                for kk in range(fl):
                    zz = Z0 + kk * 3.2
                    opens = [(u0b + 0.5, u1b - 0.5, zz + 1.0, zz + 2.2) for u0b, u1b in B.bays(L2, 2.4, 0.4)]
                    f.wall_with_openings("paint_cream", zz, zz + 3.2, opens, 2.0)
                    for o in opens:
                        f.window(*o, "paint_cream", glass="glass_yellow", mullions=1, depth=0.12)
            self.hip(m, p, z, h=1.5, overhang=0.6)

    def b_hall(self, m, b, name):
        """School hall: cream walls with orange pilasters and end panels, a
        clerestory band, terracotta gable roof (timber trusses inside).
        講堂: クリーム色の壁、橙の付柱、テラコッタの切妻屋根。"""
        poly = b["poly"]
        eave = b["h"]
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            low = [(u0 + 0.6, u1 - 0.6, Z0 + 1.0, Z0 + 2.6) for u0, u1 in B.bays(Ln, 4.8, 0.6)]
            high = [(u0 + 0.4, u1 - 0.4, eave - 3.2, eave - 1.2) for u0, u1 in B.bays(Ln, 4.8, 0.6)]
            f.wall_with_openings("paint_cream", Z0, Z0 + 3.6, low, 2.0)
            f.wall_with_openings("paint_cream", Z0 + 3.6, eave, high, 2.0)
            for o in low:
                f.window(*o, "paint_cream", glass="glass_yellow", mullions=2)
            for o in high:
                f.window(*o, "paint_cream", glass="glass_dark", mullions=0, depth=0.25)
            for u in np.linspace(0.0, Ln, max(2, int(Ln // 9.6) + 1)):
                u0, u1 = max(0.0, u - 0.4), min(Ln, u + 0.4)
                f.block("paint_orange", u0, u1, 0.0, 0.25, Z0, eave)
            f.block("paint_cream", 0.0, Ln, 0.0, 0.3, Z0 + 3.4, Z0 + 3.7, ends=False)
        self.gable(m, poly, eave, "paint_orange", deg=17)

    def b_sports_hall(self, m, b, name):
        """Indoor sports hall: open ground floor on maroon columns under a grey
        band, cream walls with maroon frame lines, yellow/orange chequer panels,
        a clerestory louvre band, flat roof with a red inset.
        屋内運動場: 1 階は柱、クリーム色の壁に栗色の枠、市松模様、高窓。"""
        poly = b["poly"]
        z1 = Z0 + 5.0
        top = Z0 + 17.5
        core = poly.buffer(-2.2, join_style="mitre")
        m.prism("paint_cream", core, Z0, z1, top=False)
        for p in polys(core):
            for a, bb, Ln, n in B.edges_of(orient(p, 1.0)):
                f = B.Facade(m, a, bb, n)
                for u0, u1 in B.bays(Ln, 6.0, 1.0):
                    f.face("glass_dark", u0 + 0.8, u1 - 0.8, Z0 + 0.3, Z0 + 3.0, 0.02)
        m.flat("concrete", poly.difference(core), z1, 2.0, up=False)
        for a, bb, Ln, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            cols = np.linspace(0.4, Ln - 0.4, max(2, int(Ln // 7.5) + 1))
            for u in cols:
                f.block("paint_maroon", u - 0.4, u + 0.4, -0.8, 0.12, Z0, top, top=False, bottom=False)
            f.block("clad_silver", 0.0, Ln, 0.0, 0.6, z1, z1 + 1.4)            # the grey band / 灰色の帯
            f.face("paint_cream", 0.0, Ln, z1 + 1.4, top, 0.0, 2.0)
            for j, (u0, u1) in enumerate(zip(cols[:-1], cols[1:])):
                if j % 2 == 0 and u1 - u0 > 3.0:
                    m.quad("chequer_lvss", f.P(u0 + 0.6, 0.03, z1 + 3.0), f.P(u1 - 0.6, 0.03, z1 + 3.0),
                           f.P(u1 - 0.6, 0.03, z1 + 7.0), f.P(u0 + 0.6, 0.03, z1 + 7.0), (0, 0), (1, 0), (1, 1), (0, 1))
                f.block("louvre", u0 + 0.4, u1 - 0.4, -0.05, 0.04, top - 3.0, top - 1.2, top=False, bottom=False,
                        ends=False)
            f.block("paint_maroon", 0.0, Ln, 0.0, 0.14, top - 3.6, top - 3.25, ends=False)
            f.block("paint_maroon", 0.0, Ln, 0.0, 0.14, top - 0.35, top, ends=False)
        self.flat_roof(m, poly, top, solar=False, coping="paint_maroon")
        inset = poly.buffer(-3.0, join_style="mitre")
        if not inset.is_empty and inset.geom_type == "Polygon":
            m.prism("paint_palegrey", inset, top, top + 0.6, top=False)
            builder().hip_roof(m, inset, top + 0.6, 1.6, "roof_terracotta", overhang=0.3)

    def portal(self, m, poly, eave):
        """The maroon gable portal on the south gable end: piers, a gable frame
        standing above the roof, the yellow frame grid on dark glass, the crest
        at the apex, the name and the Chinese name boards. Its position on this
        block is a guess (the photo shows it over a paved square).
        南の妻側の栗色の門型と校章（位置は推定）。"""
        a, bb, Ln, n = self.edge_facing(poly, "south")
        f = B.Facade(m, a, bb, n)
        apex = eave + 4.0
        out = 1.0
        for u0, u1 in ((-0.6, 0.9), (Ln - 0.9, Ln + 0.6)):
            f.block("paint_maroon", u0, u1, -0.3, out, Z0, eave + 0.8)
        # the gable: a maroon slab from the eaves to the apex, out over the wall
        # 切妻の栗色の壁（屋根より高い）
        tri = [(-0.6, eave + 0.8), (Ln + 0.6, eave + 0.8), (Ln / 2, apex + 0.8)]
        for w, flip in ((out, False), (-0.3, True)):
            P = [f.P(u, w, z) for u, z in tri]
            A = np.array(P)
            nrm = np.cross(A[1] - A[0], A[2] - A[0])
            want = n if not flip else -n
            I = [[0, 1, 2]] if float(nrm[:2] @ want) > 0 else [[0, 2, 1]]
            m.tris("paint_maroon", P, np.r_[want, 0.0], [(0, 0), (1, 0), (0.5, 1)], I)
        for (u0, z0), (u1, z1) in ((tri[0], tri[2]), (tri[2], tri[1])):
            ctr = f.P((u0 + u1) / 2, (out - 0.3) / 2, (z0 + z1) / 2 - 1.0)
            oquad(m, "paint_maroon", [f.P(u0, -0.3, z0), f.P(u1, -0.3, z1), f.P(u1, out, z1), f.P(u0, out, z0)], ctr)
        # dark glass behind a yellow frame grid / 黄色い格子とガラス
        g0, g1, gz0, gz1 = 0.9, Ln - 0.9, Z0 + 3.4, eave + 0.6
        f.face("glass_dark", g0, g1, gz0, gz1, 0.25)
        for u in np.linspace(g0, g1, 4):
            f.block("paint_yellow", u - 0.15, u + 0.15, 0.25, 0.55, gz0, gz1)
        for z in np.linspace(gz0, gz1, 4):
            f.block("paint_yellow", g0, g1, 0.25, 0.55, z - 0.15, z + 0.15)
        # name sign over the grid, crest at the apex, Chinese name below
        # 格子の上に校名、頂部に校章、下に中国語名
        hz = (g1 - g0) / 8
        m.quad("name_lvss", f.P(g0, out + 0.02, eave + 1.0), f.P(g1, out + 0.02, eave + 1.0),
               f.P(g1, out + 0.02, eave + 1.0 + hz), f.P(g0, out + 0.02, eave + 1.0 + hz),
               (0, 0), (1, 0), (1, 1), (0, 1))
        self.shield(m, f, Ln / 2, apex - 1.1, 0.95, out + 0.03, "crest_lvss")
        m.quad("zh_lvss", f.P(Ln / 2 - 2.4, 0.6, Z0 + 2.3), f.P(Ln / 2 + 2.4, 0.6, Z0 + 2.3),
               f.P(Ln / 2 + 2.4, 0.6, Z0 + 3.2), f.P(Ln / 2 - 2.4, 0.6, Z0 + 3.2), (0, 0), (1, 0), (1, 1), (0, 1))

    # small parts / 小物
    def shield(self, m, f, u, z, r, w, mat):
        """A shield-shaped crest plaque (flat top, pointed base: not a round
        traffic-sign shape), centred at (u, z), r = half width, w off the wall,
        its texture mapped on the bounding box. / 盾形の校章（道路標識の形を避ける）。"""
        outline = [(-1.0, 1.0), (1.0, 1.0), (1.0, 0.1), (0.8, -0.5), (0.4, -0.95), (0.0, -1.2), (-0.4, -0.95),
                   (-0.8, -0.5), (-1.0, 0.1)]
        P = [f.P(u, w, z)] + [f.P(u + r * x, w, z + r * y) for x, y in outline]
        UV = [(0.5, 0.55)] + [((x + 1) / 2, (y + 1.2) / 2.2) for x, y in outline]
        n = len(outline)
        I = np.array([[0, 1 + k, 1 + (k + 1) % n] for k in range(n)])
        A = np.array(P)
        nrm = np.cross(A[I[0, 1]] - A[I[0, 0]], A[I[0, 2]] - A[I[0, 0]])
        if float(nrm[:2] @ f.n) < 0:
            I = I[:, ::-1]
        m.tris(mat, P, np.r_[f.n, 0.0], UV, I)

    def disc(self, m, f, u, z, r, w, mat, seg=24):
        """A round plaque standing w off the wall, textured. / 丸い銘板。"""
        P = [f.P(u, w, z)] + [f.P(u + r * math.cos(2 * math.pi * k / seg), w, z + r * math.sin(2 * math.pi * k / seg))
                              for k in range(seg)]
        UV = [(0.5, 0.5)] + [(0.5 + 0.5 * math.cos(2 * math.pi * k / seg), 0.5 + 0.5 * math.sin(2 * math.pi * k / seg))
                             for k in range(seg)]
        I = []
        for k in range(seg):
            I += [0, 1 + k, 1 + (k + 1) % seg]
        A = np.array(P)
        nrm = np.cross(A[I[1]] - A[I[0]], A[I[2]] - A[I[0]])
        I = np.array(I).reshape(-1, 3)
        if float(nrm[:2] @ f.n) < 0:
            I = I[:, ::-1]
        m.tris(mat, P, np.r_[f.n, 0.0], UV, I)

    def edge_facing(self, poly, side):
        want = {"north": (0, 1), "east": (1, 0), "south": (0, -1), "west": (-1, 0)}[side]
        return max(B.edges_of(poly), key=lambda e: float(e[3] @ np.array(want)) + 0.001 * e[2])

    def wall_text(self, m, f, Ln, text, font, height, u, z, mat, w):
        """Raised letters on a facade, centred at (u, z), w off the wall. / 壁の文字。"""
        from mesh import earcut
        g = text_poly(text, height, font)
        for p in polys(g):
            p = orient(p, 1.0)
            rings = [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]
            V = np.concatenate(rings)
            ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
            I = earcut.triangulate_float64(V.astype(np.float64), ends).reshape(-1, 3)
            if len(I) == 0:
                continue
            P3 = [f.P(u + q[0], w, z + q[1]) for q in V]
            A = np.array([P3[i] for i in I.ravel()]).reshape(-1, 3, 3)
            nrm = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
            ok = np.linalg.norm(nrm, axis=1) > 2e-6              # drop earcut's zero-area slivers / 面積ゼロを除く
            flip = (nrm[:, :2] @ f.n) < 0
            I = np.where(flip[:, None], I[:, ::-1], I)[ok]
            m.tris(mat, P3, np.r_[f.n, 0.0], V / height, I)

    def walkway(self, m, line, w):
        """Covered walkway: green metal roof on maroon posts, cut clear of the
        buildings and roads. / 屋根付き通路。"""
        roof = line.buffer(w / 2, cap_style="flat").difference(self.footprint.buffer(0.2)).difference(
            self.road.buffer(1.0))
        z = Z0 + 2.9
        m.flat("roof_linkway", roof, z + 0.12, 2.0)
        m.flat("roof_linkway", roof, z, 2.0, up=False)
        for p in polys(roof):
            m.ring_walls("paint_cream", p.exterior, z, z + 0.12, outward=True)
        for k in range(int(line.length // 4.0) + 1):
            q = line.interpolate(min(line.length, k * 4.0))
            for s in (-1, 1):
                d = np.array(line.coords[-1]) - np.array(line.coords[0])
                d = d / np.linalg.norm(d)
                px, py = q.x - d[1] * s * (w / 2 - 0.3), q.y + d[0] * s * (w / 2 - 0.3)
                if roof.buffer(-0.1).contains(Point(px, py)):
                    m.box("paint_maroon", px, py, Z0, z, 0.15, 0.15)

    def link(self, m, line, w, levels):
        """Link bridge between blocks: floors with railings and a roof. / 連絡橋。"""
        strip = line.buffer(w / 2, cap_style="flat").difference(self.footprint)
        for p in polys(strip):
            for k in range(1, levels):
                z = Z0 + k * FLOOR
                m.prism("paint_cream", p, z - 0.35, z, top=True, bottom=True)
                m.ring_walls("paint_cream", p.exterior, z, z + 1.0, outward=True)
            m.prism("roof_metal", p, Z0 + levels * FLOOR - 0.2, Z0 + levels * FLOOR, top=True, bottom=True)
        for k in range(int(line.length // 6) + 1):
            q = line.interpolate(min(line.length, k * 6.0))
            if not self.footprint.buffer(0.5).contains(q):
                m.box("paint_maroon", q.x, q.y, Z0, Z0 + levels * FLOOR - 0.2, 0.4, 0.4)

    def main_gate(self, m):
        """Main gate on St 11: brick piers with white caps beside the driveway,
        yellow sliding gates pulled back along the fence (off the lane), the
        brick sign wall with the name board north of the gate.
        正門: 煉瓦の門柱（白い笠石）、黄色い引き戸、校名板の壁。"""
        (ax, ay), (bx, by) = self.gate_span
        tx, ty = bx - ax, by - ay
        tl = math.hypot(tx, ty) or 1.0
        tx, ty = tx / tl, ty / tl
        road = self.road.buffer(0.3)
        ang = math.atan2(ty, tx)
        nx0, ny0 = -ty, tx                                       # into the site / 敷地内向き
        if nx0 * (self.site_c.x - ax) + ny0 * (self.site_c.y - ay) < 0:
            nx0, ny0 = -nx0, -ny0
        for s, (px, py) in ((-1, (ax, ay)), (1, (bx, by))):
            for k in range(2):
                cx, cy = px + s * tx * (0.8 + k * 9.0), py + s * ty * (0.8 + k * 9.0)
                if road.intersects(Point(cx, cy).buffer(0.8)):
                    continue
                m.box("paint_brickred", cx, cy, Z0, Z0 + 2.5, 1.1, 1.1, ang)
                m.box("paint_white", cx, cy, Z0 + 2.5, Z0 + 2.7, 1.3, 1.3, ang)
                m.box("paint_white", cx, cy, Z0 + 2.7, Z0 + 2.9, 0.8, 0.8, ang)
            # the yellow sliding gate, open: lies along the fence behind the pier
            # 開いた黄色い引き戸（フェンス沿い）
            g0 = (px + s * tx * 1.6, py + s * ty * 1.6)
            g1 = (px + s * tx * 7.6, py + s * ty * 7.6)
            # both leaves slide south (Street View); the sign wall is north / 門扉は南側、北側は校名の壁
            if s > 0 and LineString([g0, g1]).distance(self.road) > 0.4:
                c = ((g0[0] + g1[0]) / 2, (g0[1] + g1[1]) / 2)
                for off in (0.0, 0.25):                  # two leaves, one behind the other / 2 枚重ね
                    cc = (c[0] + nx0 * off, c[1] + ny0 * off)
                    for zz in (Z0 + 0.15, Z0 + 1.0, Z0 + 1.85):
                        m.box("paint_yellow", cc[0], cc[1], zz, zz + 0.08, 6.0, 0.06, ang)
                for k in range(16):
                    x = g0[0] + (g1[0] - g0[0]) * k / 15
                    y = g0[1] + (g1[1] - g0[1]) * k / 15
                    m.box("paint_yellow", x, y, Z0 + 0.15, Z0 + 1.93, 0.05, 0.05)
        # the brick sign wall between the two north piers, in the fence line,
        # the name board on its street face / 北側の門柱の間の校名の壁
        nx, ny = ty, -tx
        if nx < 0:                          # into the site (east) / 敷地内向き
            nx, ny = -nx, -ny
        spot = None
        # in the fence line (the fence stops at it), set in only as far as the
        # road needs / 塀の線上（塀はここで切る）、道路にかかる時だけ内側へ
        for off in np.arange(0.0, 3.01, 0.25):
            wx, wy = ax - tx * 5.3 + nx * off, ay - ty * 5.3 + ny * off
            r = Polygon([(wx + tx * u + nx * w, wy + ty * u + ny * w) for u, w in
                         ((-3.8, -0.4), (3.8, -0.4), (3.8, 0.4), (-3.8, 0.4))])
            if r.disjoint(self.road.buffer(0.5)):
                spot = (wx, wy)
                self.sign_rect = r
                break
        if spot is not None:
            wx, wy = spot
            m.box("paint_brickred", wx, wy, Z0, Z0 + 2.2, 7.4, 0.6, ang)
            m.box("paint_white", wx, wy, Z0 + 2.2, Z0 + 2.35, 7.6, 0.75, ang)
            # the board faces the street: a viewer outside looks along (nx, ny),
            # their right is (ny, -nx); u runs left to right, so it reads right
            # 道路側の面に校名板（外から見て左から右へ読める向き）
            rx, ry = ny, -nx
            hx, hy = -nx * 0.32, -ny * 0.32
            p = lambda u, z: (wx + rx * u + hx, wy + ry * u + hy, z)
            m.quad("plaque_lvss", p(-2.6, Z0 + 0.6), p(2.6, Z0 + 0.6), p(2.6, Z0 + 2.0), p(-2.6, Z0 + 2.0),
                   (0, 0), (1, 0), (1, 1), (0, 1))

    def side_gates(self, m):
        """Back gate (grey steel, closed) and east gate (green steel). / 裏門と東門。"""
        for (g0, g1), mat, pier in ((self.back_gate, "clad_silver", "paint_cream"),
                                   (self.east_gate, "paint_green", "paint_cream")):
            ang = math.atan2(g1[1] - g0[1], g1[0] - g0[0])
            for p in (g0, g1):
                if self.road.buffer(0.3).intersects(Point(p).buffer(0.5)):
                    continue
                m.box(pier, p[0], p[1], Z0, Z0 + 2.2, 0.5, 0.5, ang)
            L = math.dist(g0, g1)
            c = ((g0[0] + g1[0]) / 2, (g0[1] + g1[1]) / 2)
            for zz in (Z0 + 0.1, Z0 + 1.05, Z0 + 1.95):
                m.box(mat, c[0], c[1], zz, zz + 0.08, L - 0.5, 0.06, ang)
            for k in range(int(L / 0.15)):
                x = g0[0] + (g1[0] - g0[0]) * (0.25 + k * 0.15) / L
                y = g0[1] + (g1[1] - g0[1]) * (0.25 + k * 0.15) / L
                if math.dist((x, y), g0) < L - 0.25:
                    m.box(mat, x, y, Z0 + 0.1, Z0 + 2.03, 0.04, 0.04)

    def flagpoles(self, m):
        c = self.flags
        for k in range(3):
            x, y = c.x - 2.0 + k * 2.0, c.y
            if self.road.buffer(1.0).contains(Point(x, y)) or self.footprint.buffer(0.5).contains(Point(x, y)):
                continue
            m.cylinder("metal_grey", x, y, Z0, Z0 + (10.0 if k == 1 else 8.5), 0.06, seg=6, top=True)
        m.box("concrete", c.x, c.y, Z0, Z0 + 0.3, 6.5, 1.2, math.atan2(self.ax_u[1], self.ax_u[0]))

    def chain_link(self, m, line, z0, z1, post=2.5, wire=0.3, rails=None, post_w=0.06):
        """See-through chain-link fence along a line: green posts, top/bottom
        (and middle) rails, sparse thin vertical wires (each a two-sided strip
        1.5 cm wide) and a few horizontal tension wires.
        透ける金網フェンス: 柱・横桟・細い針金（両面の細帯）。"""
        rails = rails if rails is not None else (z0 + 0.05, (z0 + z1) / 2, z1 - 0.05)
        Lg = line.length
        for k in range(int(Lg // post) + 1):
            q = line.interpolate(min(Lg, k * post))
            m.box("paint_green", q.x, q.y, z0, z1 + 0.06, post_w, post_w)
        c = list(line.coords)
        for a, b in zip(c, c[1:]):
            seg = math.dist(a, b)
            if seg < 1e-3:
                continue
            for zz in rails:
                m.wall("paint_green", a, b, zz - 0.025, zz + 0.025, 0.0, 1.0)
                m.wall("paint_green", b, a, zz - 0.025, zz + 0.025, 0.0, 1.0)
            for zz in np.arange(z0 + 0.4, z1 - 0.3, 0.6):          # tension wires / 張り線
                m.wall("fence_green", a, b, zz - 0.006, zz + 0.006, 0.0, 1.0)
                m.wall("fence_green", b, a, zz - 0.006, zz + 0.006, 0.0, 1.0)
            t = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
            for k in range(int(seg // wire) + 1):
                u = min(seg, k * wire)
                p0 = (a[0] + t[0] * (u - 0.0075), a[1] + t[1] * (u - 0.0075))
                p1 = (a[0] + t[0] * (u + 0.0075), a[1] + t[1] * (u + 0.0075))
                m.wall("fence_green", p0, p1, z0, z1, 0.0, 1.0)
                m.wall("fence_green", p1, p0, z0, z1, 0.0, 1.0)

    def site_fence(self, m):
        """Green chain-link fence (2.1 m, see-through: posts, rails, sparse
        wire) with a hedge inside, open at the gates, cut where it meets a road.
        緑の金網フェンス（透ける）と生垣。"""
        ring = self.boundary.exterior
        gaps = unary_union([LineString(self.gate_span).buffer(1.2), LineString(self.back_gate).buffer(0.4),
                            LineString(self.east_gate).buffer(0.4), getattr(self, "sign_rect", Polygon()).buffer(0.1)])
        line = ring.difference(gaps).difference(self.road.buffer(0.6))
        for g in getattr(line, "geoms", [line]):
            if g.geom_type != "LineString" or g.length < 1.0:
                continue
            self.chain_link(m, g, Z0, Z0 + 2.1)
            if g.length > 3:
                hedge = g.offset_curve(-0.8)
                if not hedge.is_empty and hedge.geom_type == "LineString":
                    hb = hedge.buffer(0.45, cap_style="flat").intersection(self.boundary).difference(
                        self.footprint.buffer(0.5)).difference(self.road.buffer(0.8))
                    m.prism("leaves_dark", hb, Z0, Z0 + 1.1, top=True)

    def pitch_fence(self, m):
        """Tall green ball-stop fence round the astro (8 m, see-through: posts,
        rails, sparse wire). / 人工芝の周りの高い防球フェンス（透ける）。"""
        ring = self.astro.buffer(0.6, join_style="mitre").exterior
        self.chain_link(m, LineString(ring.coords), Z0, Z0 + 8.0, post=3.0, wire=0.45,
                        rails=(Z0 + 0.05, Z0 + 2.4, Z0 + 4.8, Z0 + 7.2, Z0 + 7.95), post_w=0.1)
        # goals at both ends of the lines / ゴール
        cc = list(orient(self.pitch_lines.minimum_rotated_rectangle, 1.0).exterior.coords)[:4]
        if math.dist(cc[1], cc[2]) > math.dist(cc[0], cc[1]):
            cc = cc[1:] + cc[:1]
        for p, q in ((cc[1], cc[2]), (cc[3], cc[0])):
            mid = np.add(p, q) / 2
            d = np.subtract(q, p) / math.dist(p, q)
            for s in (-1, 1):
                gx, gy = mid + d * s * 1.5
                m.cylinder("goal_white", gx, gy, Z0, Z0 + 2.0, 0.05, seg=6)
            m.box("goal_white", mid[0], mid[1], Z0 + 1.95, Z0 + 2.05, 3.0, 0.08, math.atan2(d[1], d[0]))

    def trees(self, m):
        """Trees in the garden courtyards and along the fences (rain trees on
        the St 11 verge). / 中庭と塀沿いの木。"""
        spots = []
        for g in self.courtyards:
            for p in polys(g):
                core = p.buffer(-2.5)
                if core.is_empty:
                    continue
                x0, y0, x1, y1 = core.bounds
                for x in np.arange(x0 + 1.5, x1, 6.5):
                    for y in np.arange(y0 + 1.5, y1, 6.5):
                        if core.contains(Point(x, y)):
                            spots.append((x, y, 2.6 + 0.6 * ((x * 7 + y * 3) % 1.0)))
        ring = self.boundary.buffer(-4.5).exterior
        for k in range(int(ring.length // 11.0)):
            q = ring.interpolate(k * 11.0)
            spots.append((q.x, q.y, 3.4))
        busy = unary_union([self.footprint.buffer(2.0), self.astro.buffer(2.0), self.netball.buffer(1.0),
                            self.red_area, self.bb.buffer(1.0), self.blue.buffer(1.0), self.parade.buffer(1.5),
                            self.carpark] + [l.buffer(w / 2 + 1.0) for l, w in self.walkways])
        for x, y, r in spots:
            p = Point(x, y)
            if busy.contains(p) or self.road.distance(p) < r + 0.5:
                continue
            h = 4.0 + r
            m.cylinder("bark", x, y, Z0, h - r * 0.5, 0.18, seg=6, top=False)
            m.sphere("leaves_rain", x, y, h, r, r, r * 0.7, seg=8, rings=5)


def write_textures(tex_dir):
    """The school's drawn textures: yellow-framed glass, dark brick, the
    courts, the astro, the sports hall chequer, perforated screens, the name
    board, the crest (a drawn stand-in, not the real crest), the portal's
    name and Chinese name boards. / 学校独自のテクスチャ（描画）。"""
    import textures as X
    from PIL import Image, ImageDraw, ImageFont
    fb = DEJAVU + "DejaVuSans-Bold.ttf"
    out = {}

    def save(name, im, rough=0.8, metal=0.0):
        im.save(os.path.join(tex_dir, name + ".png"))
        out[name] = (name, None, rough, metal, False)

    for k, n in (("glass_yellow", 1), ("glass_yellow_2", 2), ("glass_yellow_3", 3)):
        save(k, X.window(256, n, frame=(232, 190, 45), glass0=(55, 80, 95), glass1=(130, 160, 170), curtain=False),
             0.1, 0.2)
    save("brick_dark", X.bricks(512, (95, 55, 45)))
    save("court_netball_lv", X.court(32, 17, "netball", (60, 140, 110)).convert("RGB"), 0.7)
    # netball thirds in red, as seen / 両端の 1/3 を赤に
    im = Image.open(os.path.join(tex_dir, "court_netball_lv.png")).convert("RGB")
    W, H = im.size
    d = ImageDraw.Draw(im)
    k = W / 32.0
    for x0, x1 in ((1 + 0.3, 1 + 30.5 / 3 - 0.3), (1 + 2 * 30.5 / 3 + 0.3, 31 - 0.3)):
        d.rectangle([x0 * k, 0.75 * k, x1 * k, H - 0.75 * k], fill=(170, 75, 65))
    lw = max(3, int(0.06 * k))
    d.rectangle([k, 0.85 * k, W - k, H - 0.85 * k], outline=(245, 245, 240), width=lw)
    for x in (1 + 30.5 / 3, 1 + 2 * 30.5 / 3):
        d.line([(x * k, 0.85 * k), (x * k, H - 0.85 * k)], fill=(245, 245, 240), width=lw)
    for cx, a0, a1 in ((k, -90, 90), (W - k, 90, 270)):
        d.arc([cx - 4.9 * k, H / 2 - 4.9 * k, cx + 4.9 * k, H / 2 + 4.9 * k], a0, a1, fill=(245, 245, 240), width=lw)
    save("court_netball_lv", im, 0.7)
    bb = X.court(30, 17, "basketball", (55, 85, 150))
    d = ImageDraw.Draw(bb)
    W, H = bb.size
    k = W / 30.0
    d.rectangle([0, 0, W, k], fill=(60, 135, 95))
    d.rectangle([0, H - k, W, H], fill=(60, 135, 95))
    d.rectangle([0, 0, k, H], fill=(60, 135, 95))
    d.rectangle([W - k, 0, W, H], fill=(60, 135, 95))
    for s in (-1, 1):
        x0, x1 = sorted((W / 2 + s * 14 * k, W / 2 + s * 8.2 * k))
        d.rectangle([x0, H / 2 - 2.45 * k, x1, H / 2 + 2.45 * k], fill=(170, 70, 60), outline=(245, 245, 240),
                    width=max(3, int(0.06 * k)))
    d.ellipse([W / 2 - 1.8 * k, H / 2 - 1.8 * k, W / 2 + 1.8 * k, H / 2 + 1.8 * k], fill=(170, 70, 60),
              outline=(245, 245, 240), width=max(3, int(0.06 * k)))
    save("court_bb_lv", bb, 0.7)
    save("court_parade_lv", X.court(32, 17, "netball", (92, 90, 86), line=(215, 212, 200)), 0.85)
    save("astro_lvss", Image.fromarray(X.tint(X.noise(512, (8, 32, 128)), (24, 60, 38), (36, 78, 50))), 0.9)
    save("pitch_lvss", X.court(52.3, 33.9, "football", (30, 70, 45), px_per_m=14), 0.9)
    # the sports hall's yellow / orange chequer panels / 屋内運動場の市松模様
    im = Image.new("RGB", (512, 512), (238, 228, 200))
    d = ImageDraw.Draw(im)
    for j in range(4):
        for i in range(8):
            if (i + j) % 2 == 0:
                d.rectangle([i * 64 + 2, j * 128 + 2, i * 64 + 61, j * 128 + 125],
                            fill=(235, 180, 60) if (i // 2 + j) % 2 else (225, 125, 50))
    save("chequer_lvss", im, 0.6)
    # perforated metal screen / 有孔パネル
    im = Image.new("RGB", (256, 256), (225, 220, 205))
    d = ImageDraw.Draw(im)
    for y in range(4, 256, 10):
        for x in range(4 + (5 if (y // 10) % 2 else 0), 256, 10):
            d.ellipse([x - 2, y - 2, x + 2, y + 2], fill=(95, 95, 90))
    save("perforated_lvss", im, 0.5, 0.3)

    def crest(s):
        """A drawn stand-in crest (not the school's artwork, and no road-sign
        shape or arrow): a maroon shield with a gold border, an open white book
        and a gold rising sun, "LVSS" on a gold scroll. Background white.
        校章の代わりの図案（盾・本・朝日。道路標識に見えない形）。"""
        im = Image.new("RGB", (s, s), (240, 238, 230))
        d = ImageDraw.Draw(im)
        # shield in (x, y up) with its point at y = 0; image rows run down / 盾（y は上向き、先端 y=0）
        d.polygon([(s * x, s * (1 - y)) for x, y in [(.05, 1.0), (.95, 1.0), (.95, .5), (.86, .27), (.68, .07),
                                                        (.5, .0), (.32, .07), (.14, .27), (.05, .5)]],
                  fill=(205, 160, 50))
        d.polygon([(s * x, s * (1 - y)) for x, y in [(.1, .96), (.9, .96), (.9, .5), (.82, .3), (.66, .12),
                                                        (.5, .06), (.34, .12), (.18, .3), (.1, .5)]],
                  fill=(120, 30, 40))
        # rising sun with rays / 朝日
        d.pieslice([s * .32, s * .14, s * .68, s * .5], 180, 360, fill=(240, 195, 60))
        for k in range(7):
            a = math.radians(195 + k * 25)
            d.line([(s * (.5 + .22 * math.cos(a)), s * (.32 + .22 * math.sin(a))),
                    (s * (.5 + .3 * math.cos(a)), s * (.32 + .3 * math.sin(a)))], fill=(240, 195, 60),
                   width=max(2, s // 60))
        # open book / 開いた本
        d.polygon([(s * .22, s * .4), (s * .5, s * .46), (s * .5, s * .66), (s * .22, s * .6)], fill=(245, 245, 240))
        d.polygon([(s * .78, s * .4), (s * .5, s * .46), (s * .5, s * .66), (s * .78, s * .6)], fill=(230, 230, 225))
        d.line([(s * .5, s * .46), (s * .5, s * .66)], fill=(120, 30, 40), width=max(2, s // 100))
        # scroll with the initials / 巻物と頭文字
        d.rectangle([s * .2, s * .7, s * .8, s * .8], fill=(205, 160, 50))
        f = ImageFont.truetype(fb, max(8, s // 13))
        tw = d.textlength("LVSS", font=f)
        d.text((s * .5 - tw / 2, s * .705), "LVSS", fill=(120, 30, 40), font=f)
        return im

    save("crest_lvss", crest(512), 0.5)
    # sign wall board: grey-blue with the names, crest on the right / 校名板
    W, H = 1024, 276
    im = Image.new("RGB", (W, H), (80, 100, 120))
    d = ImageDraw.Draw(im)
    d.rectangle([4, 4, W - 5, H - 5], outline=(225, 225, 225), width=5)
    f1, f2 = ImageFont.truetype(fb, 34), ImageFont.truetype(fb, 34)
    fz = ImageFont.truetype(CJK, 48)
    d.text((40, 30), "SEKOLAH MENENGAH LOYANG VIEW", fill=(245, 245, 245), font=f1)
    d.text((40, 95), ZH_NAME, fill=(245, 245, 245), font=fz)
    d.text((40, 190), "LOYANG VIEW SECONDARY SCHOOL", fill=(245, 245, 245), font=f2)
    im.paste(crest(220), (W - 245, 28))
    save("plaque_lvss", im, 0.5)
    W, H = 1024, 128
    im = Image.new("RGB", (W, H), (245, 245, 242))
    d = ImageDraw.Draw(im)
    d.text((40, 40), "LOYANG VIEW SECONDARY SCHOOL", fill=(30, 50, 110), font=ImageFont.truetype(fb, 46))
    save("name_lvss", im, 0.5)
    W, H = 512, 96
    im = Image.new("RGB", (W, H), (245, 242, 235))
    d = ImageDraw.Draw(im)
    d.text((110, 8), ZH_NAME, fill=(120, 30, 35), font=ImageFont.truetype(CJK, 70))
    save("zh_lvss", im, 0.5)
    return out
