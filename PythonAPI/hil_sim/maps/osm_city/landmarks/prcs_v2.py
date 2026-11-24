"""Pasir Ris Crest Secondary School (11 Pasir Ris Street 41) for the
PasirRisTampines map, modelled from scratch (maps/.venv). The older PasirRis map
keeps its own module (landmarks/prcs.py, untouched); build.py uses this one
only on PasirRisTampines (enabled()).

Sources (build/PasirRisTampines/prcs_photos, visual reference only, never
textures): z20 satellite tiles (overlap scale), OneMap 3D heights and orbits,
Street View Mar 2025 on St 41 / Dr 4, the school's Traffic Management Plans
(block names), Open House 2026 and Street Directory photos (inner court,
courtyard, lettering), the architect's 2011 sports-hall photo. Trace:
build/<Name>/prcs_trace.json (made by prcs_trace_make.py beside it). Colours:
the measured medians in description.md (the current white / grey scheme).

  A            General Office: 3 storeys (4.0 m ground storey), white, grey
               ground band, cyan 3 x 3 window grids, royal-blue cladding on the
               north face's east part, "Home of the OVERCOMERS" on the east gable,
               the school name + crest on the north face, the green-glass curtain
               stair bay (4 tiers, 3 fins) and the grey core at the west end,
               hip roof with solar rows
  tower        round drum: open ground storey, L2 corridor ring, the L3
               cantilevered balcony, a louvred drum, the lower octagonal ring
               roof and the top cap (dark green fascias), fitted off the loop
  links        3 / 4-storey open-corridor wings (white slabs, grey louvre bands,
               grey railings), Mondrian navy / red frames and a yellow panel on
               the inner-court faces, the glass stair tower at C's corner
  B            canteen (open L1) + hall: louvre bands, the dark-blue top panel,
               "Bravura" sign band, orange stair panel, hip roof with lantern
  C, D, E, F   4-storey classroom blocks: corridor sides, window sides with
               green-framed windows and grey awnings, plain end walls, hip roofs
               covered in solar rows; D's stair cores; "Energico" on F
  west end     the connector with stair landings, the 18.8 m stair tower,
               roof plant box with louvres, pipe railings
  sports hall  white box raised on charcoal columns (open 6 m ground storey),
               pilasters, slot windows, ledge band, the south box, the steel stair,
               dark green hip roof with deep eaves; covered walkway on Dr 4
  grounds      the raised astro field (retaining walls, 6 m ball fence with the
               blue windbreak, goals, steps), the blue inner court with stage and
               flagpole, the green basketball court, red-brick plaza / drive /
               car parks with bays, gardens with palms and the pavilion, the
               PRCS letters in the loop's planter, the Joule Centre and field
               store, bicycle shelter
  frontage     blue sliding gates on white piers, the white guard house, the
               green-tiled name wall with the drawn plaque, chain-link fence with
               a hedge

Roads win: footprints keep GAP off the road network (the drop-off loop and the
car-park aisles), sides move in rather than being cut (rectangles stay
rectangles), the tower is fitted, and any triangle left touching a road is
dropped and reported. Coordinates: the map frame (x east, y north, metres).

PasirRisTampines 用の白沙岭中学（Pasir Ris Crest Secondary School）を新たに作る。
旧 PasirRis 用の prcs.py はそのまま（この地図だけ enabled() で本モジュール）。
写真は参照のみ（テクスチャに使わない）。色は description.md の実測値。道路優先。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from landmarks.coral import chainlink, drop_over, glyphs_on, opening, tiled_face, two_sided_wall, wall_openings
from landmarks.prcs import text_poly
from mesh import Mesh, polys

NAME = "Pasir Ris Crest Secondary School"
TAG = "PRCS2"
TRACE = "prcs_trace.json"
MAP = "PasirRisTampines"
SITE_NAMES = (NAME,)
Z0 = 0.15                 # the raised ground / 敷地面
F = 3.6                   # storey / 階高
FA = 4.0                  # Block A / B ground storey / A・B の 1 階
GAP = 1.0                 # footprints this far off the road network / 道路網からの離れ
D_CORR = 2.4              # open corridor depth / 開放廊下の奥行き
FIELD_Z = 1.8             # the raised field above the site (description 1.5-2 m) / 校庭の嵩上げ
BANK_FOOT = 0.8           # the retaining wall at the boundary under the bank (sv06) / 法面下の擁壁
WALL = "pr2_white"


def enabled(out, meta):
    """Only on PasirRisTampines, with its trace (PasirRis keeps prcs.py).
    この地図とトレースがある時だけ（PasirRis は prcs.py）。"""
    return meta.get("name") == MAP and os.path.exists(os.path.join(out, TRACE))


# -- primitives (tested in test_prcs_v2.py: every face points the way it is seen)
# 基本形状（test_prcs_v2.py で面の向きを確認）
def hip_roof(m, a, b, c, d, z0, z1, ov, mat, fascia, soffit="concrete", ins=None):
    """Hipped roof over the CCW rectangle a b c d (a->b a long side): slopes up
    and out, hip ends out, soffit down, fascia boards out. Returns the eave
    corners and ridge ends for the solar rows. / 寄棟屋根（面は外・上向き）。"""
    a, b, c, d = (np.asarray(q, float) for q in (a, b, c, d))
    ax = (b - a) / np.linalg.norm(b - a)
    sd = (d - a) / np.linalg.norm(d - a)
    a2, b2, c2, d2 = a - (ax + sd) * ov, b + (ax - sd) * ov, c + (ax + sd) * ov, d - (ax - sd) * ov
    W = float(np.linalg.norm(d2 - a2))
    Lr = float(np.linalg.norm(b2 - a2))
    ins = min(W / 2, Lr / 2 - 0.05) if ins is None else ins
    r1 = (a2 + d2) / 2 + ax * ins
    r2 = (b2 + c2) / 2 - ax * ins
    P = lambda q, z: (q[0], q[1], z)
    m.quad(mat, P(a2, z0), P(b2, z0), P(r2, z1), P(r1, z1), (0, 0), (Lr / 3, 0), (Lr / 3, W / 6), (0, W / 6))
    m.quad(mat, P(c2, z0), P(d2, z0), P(r1, z1), P(r2, z1), (0, 0), (Lr / 3, 0), (Lr / 3, W / 6), (0, W / 6))
    m.tris(mat, [P(b2, z0), P(c2, z0), P(r2, z1)], (0, 0, 1), [(0, 0), (W / 3, 0), (W / 6, W / 6)], [0, 1, 2])
    m.tris(mat, [P(d2, z0), P(a2, z0), P(r1, z1)], (0, 0, 1), [(0, 0), (W / 3, 0), (W / 6, W / 6)], [0, 1, 2])
    m.quad(soffit, P(b2, z0), P(a2, z0), P(d2, z0), P(c2, z0))
    for p, q in ((a2, b2), (b2, c2), (c2, d2), (d2, a2)):
        m.wall(fascia, tuple(p), tuple(q), z0 - 0.3, z0 + 0.03)
    return a2, b2, c2, d2, r1, r2


def solar_rows(m, a2, b2, c2, d2, r1, r2, z0, z1, mat="pr2_solar", rows=3, margin=1.2):
    """Rows of PV panels lying 8 cm over both long slopes of a hip roof: one
    quad per row, the texture repeats per panel (1.0 x 1.7 m).
    寄棟の長辺斜面に太陽光パネルの列（1 列 1 枚の四角形、模様は 1 枚ごとに繰り返し）。"""
    W = float(np.linalg.norm(d2 - a2))
    rise = z1 - z0
    for e0, e1, top0, top1 in ((a2, b2, r1, r2), (c2, d2, r2, r1)):
        L = float(np.linalg.norm(e1 - e0))
        ax = (e1 - e0) / L
        mid0 = (e0 + e1) / 2
        upv = ((top0 + top1) / 2 - mid0)
        span = float(np.linalg.norm(upv))
        if span < 2.0:
            continue
        upv = upv / span
        ins = float(np.linalg.norm(top0 - e0 - upv * span))      # hip inset along the eave / 隅の入り
        for k in range(rows):
            t0, t1 = 0.8 + k * 1.9, 0.8 + k * 1.9 + 1.7
            if t1 > span - 0.4:
                break
            # the row ends follow the hips (they narrow toward the ridge) / 隅棟に沿って短く
            cut = ins * t1 / span + margin
            s0, s1 = cut, L - cut
            if s1 - s0 < 2.0:
                continue
            q = [e0 + ax * s + upv * t for s, t in ((s0, t0), (s1, t0), (s1, t1), (s0, t1))]
            zz = [z0 + rise * t / span + 0.08 for t in (t0, t0, t1, t1)]
            n = round(s1 - s0)
            m.quad(mat, *[(p[0], p[1], z) for p, z in zip(q, zz)], (0, 0), (n, 0), (n, 1), (0, 1))
    return W


def oct_tier(m, c, r0, r1, z0, z1, mat, fascia, soffit="concrete", n=8, rot=22.5, r_in=None):
    """One tier of an octagonal roof: frustum (r1 > 0) or pyramid (r1 = 0) faces
    up and out, a fascia band out, a soffit ring down to r_in.
    八角形屋根の 1 段（面は外・上、鼻隠しは外、軒裏は下）。"""
    cx, cy = c
    p = lambda r, a, z: (cx + r * math.cos(a), cy + r * math.sin(a), z)
    r_in = r0 * 0.6 if r_in is None else r_in
    for k in range(n):
        a0, a1 = math.radians(rot + 360.0 / n * k), math.radians(rot + 360.0 / n * (k + 1))
        if r1 > 1e-6:
            m.quad(mat, p(r0, a0, z0), p(r0, a1, z0), p(r1, a1, z1), p(r1, a0, z1))
        else:
            m.tris(mat, [p(r0, a0, z0), p(r0, a1, z0), (cx, cy, z1)], (0, 0, 1), [(0, 0), (1, 0), (.5, 1)],
                   [0, 1, 2])
        m.wall(fascia, p(r0, a0, 0)[:2], p(r0, a1, 0)[:2], z0 - 0.35, z0 + 0.02)
        m.quad(soffit, p(r0, a1, z0 - 0.35), p(r0, a0, z0 - 0.35), p(r_in, a0, z0 - 0.35), p(r_in, a1, z0 - 0.35))


def ngon(c, r, n=16, rot=0.0):
    return Polygon([(c[0] + r * math.cos(math.radians(rot + 360.0 / n * k)),
                     c[1] + r * math.sin(math.radians(rot + 360.0 / n * k))) for k in range(n)])


def rail_line(m, mat, p, q, z, h=1.05, post=1.6, w=0.05):
    """A steel railing along p->q standing on z: top and mid rails, posts.
    鋼製の手すり（笠木・中桟・支柱）。"""
    p, q = np.asarray(p, float), np.asarray(q, float)
    L = float(np.linalg.norm(q - p))
    if L < 0.2:
        return
    ang = math.atan2(q[1] - p[1], q[0] - p[0])
    mid = (p + q) / 2
    for z0, z1 in ((z + h - 0.06, z + h), (z + h * 0.5 - 0.03, z + h * 0.5 + 0.03)):
        m.box(mat, mid[0], mid[1], z0, z1, L, w, ang)
    for k in range(int(L // post) + 1):
        s = min(L, k * post + (L - post * int(L // post)) / 2)
        r = p + (q - p) * s / L
        m.box(mat, r[0], r[1], z, z + h - 0.06, w, w, ang, top=False)


def rail_ring(m, mat, ring, z, h=1.05, post=1.6):
    c = list(ring.coords)
    for a, b in zip(c, c[1:]):
        rail_line(m, mat, a, b, z, h, post)


class PRCS2(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        # the trace is in the real map's frame; a re-centred scratch frame
        # (crop_build.py) converts through lon/lat / 別の座標系なら緯度経度経由
        lon0, lat0 = T.get("frame_lonlat0", [fr.lon0, fr.lat0])
        tf = geo.Frame({"lat0": lat0, "lon0": lon0})
        same = abs(lon0 - fr.lon0) < 1e-9 and abs(lat0 - fr.lat0) < 1e-9
        self.X = (lambda q: (float(q[0]), float(q[1]))) if same else \
            (lambda q: fr.xy(*tf.lonlat(float(q[0]), float(q[1]))))
        ox, oy, ang = T["frames"]["S"]
        self.o = np.array(self.X((ox, oy)))
        a = math.radians(ang)
        self.eu, self.ev = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        site = [p for p, t in L.osm.areas(lambda t: t.get("name") in SITE_NAMES and "building" not in t)]
        self.site = unary_union(site).buffer(0)
        # roads win: road network, islands, the site's own drives (check_roads
        # --site counts them) / 道路優先: 道路網・島・敷地内の車路
        self.drive_snapshot = L.driveways
        self.parking_snapshot = L.parking
        near = self.site.buffer(5.0)
        self.road = L.road_all
        self.islands = L.islands.intersection(self.site.buffer(2.0))
        self.drives = L.driveways.intersection(near)
        self.no_go = unary_union([L.road_all, L.islands, self.drives])
        shapely.prepare(self.no_go)
        self.fit_log = []
        # the tower first (fitted off the loop), then the blocks in priority
        # order, each fitted (sides move in) and less what the earlier hold
        # 先に塔（車寄せから離す）、次に優先順の棟（辺を内側へ、重なりは先の棟）
        self.fit_tower()
        taken = Point(self.tc).buffer(self.tr["drum"], 16)
        self.blocks = {}
        for k in T["order"]:
            b = T["blocks"][k]
            uv = self.fit_rect(k, list(b["uv"]))
            if uv is None:
                continue
            p = self.rect(*uv).difference(taken)
            p = max(polys(p), key=lambda q: q.area) if polys(p) else None
            if p is None or p.area < 6.0:
                continue
            p = orient(shapely.set_precision(p, 0.001).buffer(0), 1.0)
            if p.geom_type != "Polygon":
                p = orient(max(polys(p), key=lambda q: q.area), 1.0)
            taken = taken.union(p)
            self.blocks[k] = dict(b, poly=p, uv=uv)
        self.footprint = unary_union([b["poly"] for b in self.blocks.values()]
                                     + [Point(self.tc).buffer(self.tr["drum"], 16)])
        G = T["ground"]
        self.courts = {k: (orient(self.rect(*g["uv"]), 1.0), g["mat"]) for k, g in G.items()}
        # the field: OSM's pitch in the site, raised with a margin / 校庭: OSM のピッチ（嵩上げ）
        pitches = [p for p, t in L.osm.areas(lambda t: t.get("leisure") == "pitch")
                   if self.site.contains(p.representative_point())]
        self.field = orient(max(pitches, key=lambda p: p.area).minimum_rotated_rectangle, 1.0) if pitches else None
        self.plateau = self.field.buffer(1.0, join_style="mitre").difference(self.no_go.buffer(GAP)) \
            if self.field is not None else Polygon()
        if self.plateau.geom_type != "Polygon" and not self.plateau.is_empty:
            self.plateau = max(polys(self.plateau), key=lambda q: q.area)
        # the lot: the OSM area less the streets and their pavement band, open at
        # the gates (coral's method) / 敷地: 道路と歩道帯を除く（門は開ける）
        ring = self.site.exterior
        span = ring.intersection(self.no_go.buffer(0.3))
        deep = self.no_go.intersection(self.site.buffer(-4.0))
        self.gates = [g for g in getattr(span, "geoms", [span]) if g.geom_type == "LineString"
                      and 2.5 < g.length < 14.0 and deep.distance(g) < 6.0]
        edge_roads = self.road.buffer(2.6).intersection(ring.buffer(7.0))
        for g in self.gates:
            edge_roads = edge_roads.difference(g.buffer(6.0))
        lot = self.site.difference(edge_roads)
        self.lot = max(polys(lot), key=lambda q: q.area) if polys(lot) else self.site
        self.centre = self.footprint.centroid
        # generic street lamps on the loop's planter island go (build.py
        # items_clear; the planter holds the letters and shrubs, sv04)
        # 車寄せの島の汎用街灯を除く
        self.items_clear = self.islands.buffer(0.3)
        self.items_kinds = ("lamp",)
        self.court_c = self.courts["court_blue"][0].centroid
        self._parts = None
        self._ground = None
        for line in self.fit_log:
            print("prcs_v2: " + line)

    # -- frame S / 敷地座標
    def at(self, u, v):
        q = self.o + self.eu * u + self.ev * v
        return (float(q[0]), float(q[1]))

    def uv(self, x, y):
        d = np.array([x, y]) - self.o
        return float(d @ self.eu), float(d @ self.ev)

    def rect(self, u0, u1, v0, v1):
        return Polygon([self.at(u0, v0), self.at(u1, v0), self.at(u1, v1), self.at(u0, v1)])

    def facing(self, n):
        cu, cv = float(n @ self.eu), float(n @ self.ev)
        if abs(cv) >= abs(cu):
            return "N" if cv > 0 else "S"
        return "E" if cu > 0 else "W"

    # -- roads win / 道路優先
    def fit_rect(self, k, uv, step=0.25, limit=8.0):
        """Rectangles stay rectangles: the side nearest the road network moves
        in until the block keeps GAP off it (ch. 41). / 辺を内側へ（矩形のまま）。"""
        moved = 0.0
        while self.rect(*uv).distance(self.no_go) < GAP - 1e-6:
            u0, u1, v0, v1 = uv
            segs = {0: LineString([self.at(u0, v0), self.at(u0, v1)]), 1: LineString([self.at(u1, v0), self.at(u1, v1)]),
                    2: LineString([self.at(u0, v0), self.at(u1, v0)]), 3: LineString([self.at(u0, v1), self.at(u1, v1)])}
            i = min(segs, key=lambda j: segs[j].distance(self.no_go))
            uv[i] += step if i in (0, 2) else -step
            moved += step
            if moved > limit or uv[1] - uv[0] < 2.0 or uv[3] - uv[2] < 2.0:
                self.fit_log.append("%s left out (road network)" % k)
                return None
        if moved:
            self.fit_log.append("%s: sides moved in %.2f m off the road network" % (k, moved))
        return uv

    def fit_tower(self):
        """The drop-off loop runs round the tower's north: the tower moves back
        (south, up to 2 m) and its rings shrink until they keep GAP off it.
        塔は車寄せから後退し、輪を縮めて離す。"""
        t = self.T["tower"]
        c0 = np.array(self.X(t["c"]))
        r = dict(drum=t["r_drum"], balcony=t["r_balcony"], roof=t["r_roof"], cap=t["r_cap"])
        best = None
        for back in np.arange(0.0, 2.01, 0.25):
            c = c0 - self.ev * back
            d = self.no_go.distance(Point(*c)) - GAP
            reach = min(d, r["roof"])
            if best is None or reach > best[1] + 1e-3:
                best = (back, reach, c)
            if reach >= r["roof"] - 1e-6:
                break
        back, reach, c = best
        k = min(1.0, reach / r["roof"])
        self.tc = (float(c[0]), float(c[1]))
        self.tr = {n: v * (k if n != "drum" else max(k, 0.85)) for n, v in r.items()}
        self.tr["porch"] = max(0.0, self.no_go.distance(Point(*c)) - GAP)
        self.fit_log.append("tower: moved back %.2f m, rings x%.2f (roof r %.2f m)" % (back, k, self.tr["roof"]))

    # -- layout / 配置
    def inner(self):
        return self.lot.buffer(-0.6, join_style="mitre")

    def keepout(self):
        """Where the generic trees and furniture must not go. / 汎用の木と小物を置かない所。"""
        return unary_union([self.inner(), self.footprint.buffer(3.0)])

    def tree_filter(self, trees):
        """Generic trees off the loop's planter island and the site (props.py
        plants islands without the keep-out; the planter has shrubs and the
        PRCS letters, sv04). / 車寄せの島と敷地の汎用の木を除く。"""
        clear = unary_union([self.islands.buffer(0.5), self.inner()])
        return [t for t in trees if not clear.contains(Point(t[0], t[1]))]

    # -- ground / 地面
    def zones(self):
        if self._ground is not None:
            return self._ground
        inner = self.inner().difference(self.no_go.buffer(0.05))
        foot = self.footprint
        courts = unary_union([c for c, _ in self.courts.values()]).intersection(inner).difference(foot)
        plateau = self.plateau.intersection(inner)
        red = unary_union([self.rect(*r["uv"]) for r in self.T["red"].values()]
                          + [self.parking_snapshot.intersection(self.site)])
        red = red.intersection(inner).difference(foot).difference(courts).difference(plateau)
        gardens = unary_union([self.rect(*g["uv"]) for g in self.T["gardens"].values()])
        paths = self.garden_paths()
        hard = foot.buffer(2.0, join_style="mitre").intersection(inner).difference(foot).difference(courts) \
            .difference(plateau).difference(red).difference(gardens)
        hard = hard.union(paths.intersection(inner).difference(foot))
        surround = unary_union([self.rect(*h["uv"]) for h in self.T.get("hard", {}).values()])
        hard = hard.union(surround.intersection(inner).difference(foot).difference(courts))
        red = red.difference(surround)
        red = red.difference(paths)
        grass = inner.difference(foot).difference(courts).difference(plateau).difference(red).difference(hard)
        drives = self.drives.intersection(self.lot.buffer(-0.3))
        bank = self.bank().intersection(inner).difference(foot.buffer(0.5)).difference(courts)
        red, hard, grass = red.difference(bank), hard.difference(bank), grass.difference(bank)
        self._ground = dict(courts=courts, plateau=plateau, red=red, hard=hard, grass=grass, drives=drives,
                            bank=bank)
        return self._ground

    def bank(self):
        """The planted bank on the field's road sides (N, E): from the low
        retaining wall at the boundary up to the field (sv06, sv07).
        校庭の道路側（北・東）の植栽のある法面。"""
        if self.plateau.is_empty:
            return Polygon()
        parts = []
        for a, b, L, n in B.edges_of(orient(self.plateau, 1.0)):
            if self.facing(n) in ("N", "E") and L > 3.0:
                parts.append(Polygon([tuple(a), tuple(b), tuple(b + n * 12.0), tuple(a + n * 12.0)]))
        g = unary_union(parts).difference(self.plateau).difference(self.no_go.buffer(1.0))
        return g.intersection(self.lot.buffer(-0.6, join_style="mitre"))

    def bank_z(self, X, Y):
        """Bank height: BANK_FOOT at the boundary fence, the field level at the
        plateau, linear between. / 法面の高さ（境界で擁壁天端、校庭端で校庭面）。"""
        pts = shapely.points(np.column_stack([X, Y]))
        dp = shapely.distance(self.plateau, pts)
        df = shapely.distance(self.lot.exterior, pts) - 0.6
        t = np.clip(dp / np.maximum(dp + np.maximum(df, 0.0), 1e-6), 0.0, 1.0)
        return Z0 + FIELD_Z - (FIELD_Z - BANK_FOOT) * t

    def garden_paths(self):
        """The diamond paths of the Forte / Energico courtyard and a path round
        the pavilion. / 中庭の菱形の小道。"""
        u0, u1, v0, v1 = self.T["gardens"]["courtyard_ef"]["uv"]
        um, vm = (u0 + u1) / 2, (v0 + v1) / 2
        ls = [LineString([self.at(u0, vm), self.at(um, v1), self.at(u1, vm), self.at(um, v0), self.at(u0, vm)]),
              LineString([self.at(u0, vm), self.at(u1, vm)])]
        return unary_union([g.buffer(0.8, cap_style="flat") for g in ls])

    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        Z = self.zones()
        T = lambda g: g.intersection(tile)
        m.flat("grass_park", T(Z["grass"]), z + 0.005, 4.0)
        m.flat("concrete_path", T(Z["hard"]), z + 0.01, 2.0)
        m.flat("pavers_red", T(Z["red"]), z + 0.01, 2.0)
        m.flat("asphalt_lot", T(Z["drives"]), z + 0.006, 4.0)
        m.flat("grass_park", T(Z["bank"]), z, 3.0, zf=self.bank_z)
        # the raised field: margin and the astro turf (drawn once) / 嵩上げした校庭
        zf = Z0 + FIELD_Z
        if not Z["plateau"].is_empty:
            m.flat("grass_park", T(Z["plateau"].difference(self.field)), zf, 4.0)
            if tile.contains(self.field.centroid):
                self.court_quad(m, self.field, "pr2_astro", zf + 0.01)
        for k, (c, mat) in self.courts.items():
            if tile.contains(c.centroid):              # once, in its own tile / 自分のタイルに 1 回だけ
                self.court_quad(m, c, mat, z + 0.02)
        for ln in self.bay_lines():
            if tile.contains(ln.interpolate(0.5, normalized=True)):
                m.flat("paint_white", ln.buffer(0.05, cap_style="flat"), z + 0.02, 1.0)
        return m

    def court_quad(self, m, c, mat, z):
        q = [np.array(p) for p in list(orient(c, 1.0).exterior.coords)[:4]]
        if math.dist(q[0], q[1]) < math.dist(q[1], q[2]):
            q = q[1:] + q[:1]
        m.quad(mat, *[(p[0], p[1], z) for p in q], (0, 0), (1, 0), (1, 1), (0, 1))

    def bay_lines(self):
        """Car-park bays (2.5 m) both sides of the front aisles, clear of the
        aisle, the blocks and the lot edge. / 前庭の通路両側の駐車枠。"""
        if getattr(self, "_bays", None) is not None:
            return self._bays
        out = []
        inner = self.inner()
        foot = self.footprint.buffer(0.8)
        for ua, ub in ((-72.0, -36.0), (-8.0, 34.0)):
            for u in np.arange(ua, ub + 1e-6, 2.5):
                seg = LineString([self.at(u, 44.0), self.at(u, 80.0)])
                hit = seg.intersection(self.drives)
                for g in getattr(hit, "geoms", [hit]):
                    if g.geom_type != "LineString" or g.length < 3.0:
                        continue
                    vs = sorted(self.uv(*q)[1] for q in g.coords)
                    for va, vb in ((vs[0] - 5.0, vs[0] - 0.3), (vs[-1] + 0.3, vs[-1] + 5.0)):
                        ln = LineString([self.at(u, va), self.at(u, vb)])
                        if inner.contains(ln) and not ln.intersects(foot) and ln.distance(self.no_go) > 0.2:
                            out.append(ln)
        self._bays = out
        return out

    # -- buildings / 建物
    def buildings_mesh(self, tile):
        if self._parts is None:
            self._parts = self.build_parts()
        m = Mesh()
        x0, y0, x1, y1 = tile.bounds
        for (x, y), pm in self._parts:
            if x0 <= x < x1 and y0 <= y < y1:
                m.merge(pm)
        return m

    def build_parts(self):
        parts = []
        self.drop_log = []
        if self.site.is_empty:          # crop builds that leave the school out / 敷地が切り抜き範囲外
            return parts
        guard = self.no_go.buffer(0.05)
        road_only = unary_union([self.road, self.drives]).buffer(0.3)

        def add(anchor, fn, *a, guard_=None, **k):
            pm = Mesh()
            fn(pm, *a, **k)
            nd = drop_over(pm, guard if guard_ is None else guard_)
            if nd:
                self.drop_log.append((fn.__name__, nd))
            parts.append((anchor, pm))

        for k, b in self.blocks.items():
            c = b["poly"].centroid
            add((c.x, c.y), self.block, k, b)
        add(self.tc, self.tower)
        c = self.site.centroid
        add((c.x, c.y), self.site_fence)
        add((c.x, c.y), self.trees)
        add((c.x, c.y), self.walkways)
        if self.field is not None:
            f = self.field.centroid
            add((f.x, f.y), self.field_works)
        for g in self.gates:
            q = g.interpolate(0.5, normalized=True)
            add((q.x, q.y), self.vehicle_gate, g)
        P = self.T["points"]
        add(self.X(P["guard_house"]["xy"]), self.guard_house)
        add(self.X(P["name_wall"]["xy"]), self.name_wall)
        add(self.X(P["pavilion"]["xy"]), self.pavilion)
        add(self.X(P["stage"]["xy"]), self.court_furniture)
        add(self.X(P["ne_shelter"]["xy"]), self.ne_shelter)
        # the PRCS letters stand on the loop's island: guarded by the road and
        # drives only / 車寄せの島の PRCS 文字（道路と車路だけを避ける）
        if not self.islands.is_empty:
            isl = max(polys(self.islands), key=lambda q: q.area)
            add((isl.centroid.x, isl.centroid.y), self.planter, isl, guard_=road_only)
        for nm, n in self.drop_log:
            print("prcs_v2: %s: %d triangles over the road network dropped" % (nm, n))
        self.tris = sum(pm.ntris() for _, pm in parts)
        print("prcs_v2: %d blocks, %d triangles" % (len(self.blocks), self.tris))
        return parts

    # -- facades / 外壁
    def pieces(self, k, a, b, n):
        """Runs [(u0, u1, z_cover)] of an edge: z_cover = Z0 where free, else the
        wall top of the neighbour touching it (coral.pieces). / 辺の区間と隣棟の高さ。"""
        L = float(np.linalg.norm(b - a))
        t = (b - a) / L
        others = [(bb["poly"], bb["top"]) for kk, bb in self.blocks.items() if kk != k]
        tower = Point(self.tc).buffer(self.tr["drum"] + 0.05, 16)
        step = 0.25
        us = np.arange(step / 2, L, step)
        runs = []
        for u in us:
            p = Point(*(a + t * u + n * 0.15))
            h = Z0
            for poly, tp in others:
                if poly.contains(p):
                    h = max(h, tp)
            if tower.contains(p):
                h = max(h, Z0 + 3 * F)
            u0, u1 = max(0.0, u - step / 2), min(L, u + step / 2)
            if runs and abs(runs[-1][2] - h) < 1e-6:
                runs[-1][1] = u1
            else:
                runs.append([u0, u1, h])
        return [tuple(r) for r in runs]

    def block(self, m, k, b):
        poly, top, kind = b["poly"], b["top"], b["kind"]
        special = {"tower": None, "sports_hall": self.sports_hall, "joule": self.joule, "store": self.store,
                   "shelter": self.bike_shelter, "porch": self.porch}
        if kind in special and special[kind] is not None:
            return special[kind](m, k, b)
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            side = self.facing(n)
            for u0, u1, zc in self.pieces(k, a, bb, n):
                if zc >= top - 0.05:
                    continue
                if zc > Z0 + 0.05:
                    # above a lower neighbour: plain wall / 低い隣棟の上は無地の壁
                    f.face(WALL, u0, u1, zc, top, 0.0, 3.0)
                    continue
                self.facade(f, k, kind, side, u0, u1, L, top)
        self.roof(m, k, b)
        self.signs(m, k, b)

    def facade(self, f, k, kind, side, u0, u1, L, top):
        short = L < 18.0
        corr_sides = {"E": "S", "F": "N", "C": "WE", "D": "N"}
        if kind == "classroom":
            if short:
                # west ends on St 41 blank (sv11, left part); others a stack of window pairs
                # 西端（St 41 側）は無地、他は対の窓
                pairs = () if side == "W" else ((u0 + u1) / 2,)
                return self.end_wall(f, u0, u1, Z0, 4, F, top, pairs=pairs)
            if side in corr_sides.get(k, ""):
                return self.corridor(f, u0, u1, Z0, 4, F, top, mondrian=self.on_court(f, u0, u1))
            # E's north face, west end: the royal-blue cladding seen past A's west core
            # (sv13, bearing ~165 deg; a CCW north edge runs east to west)
            # E の北面西端の青い外装（sv13）
            blue = (L - 10.5, L - 1.0) if k == "E" and side == "N" and u1 > L - 1.0 else None
            return self.windows(f, u0, u1, Z0, 4, F, top, blue=blue)
        if kind == "corridor":
            st = self.T["blocks"][k]["storeys"]
            if L < 9.0:                                     # the narrow ends only / 狭い端だけ
                return self.end_wall(f, u0, u1, Z0, st, F, top)
            zb = Z0 + (0.4 if st == 3 else 0.0)            # 3-storey links: A's floors / 3 階建ては A の床高
            return self.corridor(f, u0, u1, zb, st, F, top, mondrian=self.on_court(f, u0, u1))
        if kind == "office":
            return self.office(f, side, u0, u1, L, top)
        if kind == "hall":
            return self.hall(f, side, u0, u1, L, top)
        if kind == "core":
            return self.core(f, u0, u1, top, grey=k == "A_core")
        if kind == "curtain":
            return self.curtain(f, u0, u1, top)
        if kind == "glass_stair":
            return self.glass_stair(f, u0, u1, top)
        if kind == "endblock":
            if side == "W":
                # sv11: one stack of recessed landings at the north end, window pairs
                # under the stair tower, blank beyond (a CCW west edge runs north to south)
                # 北端に踊り場の列、階段塔の下に対の窓、他は無地
                north = u0 < 0.5
                return self.end_wall(f, u0, u1, Z0, 4, F, top, landing=1.9 if north else None,
                                     pairs=(7.6, 11.4) if north else ())
            if side == "E":
                return self.corridor(f, u0, u1, Z0, 4, F, top)
            return self.end_wall(f, u0, u1, Z0, 4, F, top)
        if kind == "stair_tower":
            return self.stair_tower_face(f, u0, u1, top, side)
        f.face(WALL, u0, u1, Z0, top, 0.0, 3.0)

    def on_court(self, f, u0, u1):
        """Does this run face the inner court (within 4 m of it)? / 内庭に面するか。"""
        mid = np.array(f.P((u0 + u1) / 2, 0.0, 0.0)[:2])
        cc = np.array([self.court_c.x, self.court_c.y])
        court = self.courts["court_blue"][0]
        return float(f.n @ (cc - mid)) > 0 and court.distance(Point(*(mid + f.n * 1.0))) < 7.0

    def windows(self, f, u0, u1, zb, nst, fh, top, bay=3.6, blue=None):
        """Classroom window side: green-framed windows with grey awnings, a
        score line per floor (sv11); optional royal-blue cladding over a run
        with louvred windows (sv13). / 教室の窓面: 緑枠の窓と灰色の庇、青い外装。"""
        for k in range(nst):
            z0, z1 = zb + k * fh, zb + (k + 1) * fh
            opens = [(c - 1.25, c + 1.25, z0 + 0.95, z0 + 2.55) for c in self.centres(u0, u1, bay)]
            if blue is not None and k > 0:
                opens = [o for o in opens if o[1] < blue[0] - 0.2 or o[0] > blue[1] + 0.2]
            wall_openings(f, WALL, u0, u1, z0, z1, opens, 3.0)
            for o in opens:
                opening(f, *o, WALL, "pr2_win", depth=0.2, sill=True)
                f.block("pr2_rail", o[0] - 0.1, o[1] + 0.1, 0.0, 0.55, o[3] + 0.12, o[3] + 0.18)
            f.block("pr2_grey", u0, u1, 0.0, 0.03, z1 - 0.05, z1 + 0.03, ends=False)
        f.face(WALL, u0, u1, zb + nst * fh, top, 0.0, 3.0)
        f.face(WALL, u0, u1, Z0 - 0.15, zb, 0.0, 3.0)
        if blue is not None:
            ba, bb = max(u0, blue[0]), min(u1, blue[1])
            if bb - ba > 2.0:
                f.block("pr2_blueclad", ba, bb, 0.0, 0.18, zb + fh, top - 0.4)
                for k in range(1, nst):
                    z0 = zb + k * fh
                    for c in self.centres(ba, bb, 3.0, 0.6):
                        f.block("pr2_grid", c - 1.1, c + 1.1, 0.18, 0.2, z0 + 1.0, z0 + 2.4, bottom=False)
                        f.block("pr2_rail", c - 1.2, c + 1.2, 0.18, 0.5, z0 + 2.45, z0 + 2.52)

    @staticmethod
    def centres(u0, u1, bay, margin=0.8):
        n = int((u1 - u0 - 2 * margin) // bay)
        if n <= 0:
            return []
        w = (u1 - u0 - 2 * margin) / n
        return [u0 + margin + w * (i + 0.5) for i in range(n)]

    def corridor(self, f, u0, u1, zb, nst, fh, top, mondrian=False):
        """Open corridors: white slab edges, grey railings, a grey louvre band
        under each slab, the classroom wall behind (doors, windows), columns;
        the recess stops 0.25 m short of the corners. / 開放廊下。"""
        a0, a1 = u0 + 0.25, u1 - 0.25
        if a1 - a0 < 1.5:
            f.face(WALL, u0, u1, Z0, top, 0.0, 3.0)
            return
        D = D_CORR
        zt = zb + nst * fh
        f.face(WALL, u0, a0, Z0 - 0.15, top, 0.0, 3.0)
        f.face(WALL, a1, u1, Z0 - 0.15, top, 0.0, 3.0)
        if zb > Z0 + 0.01:
            f.face(WALL, a0, a1, Z0 - 0.15, zb, 0.0, 3.0)
        cols = np.linspace(a0 + 0.25, a1 - 0.25, max(2, int((a1 - a0) // 7.6) + 2))
        for k in range(nst):
            z0, z1 = zb + k * fh, zb + (k + 1) * fh
            zc = z1 - 0.35                                   # ceiling / 天井
            if k == 0 and zb <= Z0 + 0.01:
                f.face(WALL, a0, a1, Z0 - 0.15, z0, 0.0, 3.0)
            else:
                f.face(WALL, a0, a1, z0 - 0.35, z0 + 0.1, 0.0, 3.0)           # slab edge / スラブ端
                f.hplane(WALL, a0, a1, -0.1, 0.0, z0 + 0.1, True)
            f.hplane("concrete", a0, a1, -D, -0.1, z0 + 0.1 if k else z0, True)          # floor / 床
            tiled_face(f, "pr2_corr", a0, a1, z0, zc, -D, 3.6, zc - z0, v0=z0)
            f.hplane("concrete", a0, a1, -D, 0.0, zc, False)
            f.side(WALL, a0, -D, 0.0, z0, zc, 1)
            f.side(WALL, a1, -D, 0.0, z0, zc, -1)
            f.face("pr2_louvre", a0, a1, zc - 0.95, zc, -0.16, 1.0)
            for j in range(4):
                zz = zc - 0.9 + j * 0.22
                f.block("pr2_rail", a0, a1, -0.14, 0.03, zz, zz + 0.08, ends=False)
            if k > 0 or zb > Z0 + 0.01:
                self.rail(f, a0, a1, z0 + 0.1)
            for u in cols:
                f.block(WALL, u - 0.22, u + 0.22, -D, -0.02, z0, zc - 0.95, top=False,
                        bottom=False)
        f.face(WALL, a0, a1, zt - 0.35, top, 0.0, 3.0)
        if mondrian:
            self.mondrian(f, a0, a1, zb, nst, fh)

    def rail(self, f, a0, a1, z, h=1.05, w=-0.08, post=1.6):
        """Grey steel railing along a facade run. / 外壁沿いの鋼製手すり。"""
        f.block("pr2_rail", a0, a1, w - 0.05, w, z + h - 0.06, z + h, ends=False)
        f.block("pr2_rail", a0, a1, w - 0.04, w, z + 0.45, z + 0.5, ends=False, top=False, bottom=False)
        for u in np.arange(a0 + 0.1, a1, post):
            f.block("pr2_rail", u - 0.025, u + 0.025, w - 0.05, w, z, z + h - 0.06, top=False, bottom=False)

    def mondrian(self, f, a0, a1, zb, nst, fh):
        """The inner court's Mondrian frames (photo): a wide navy frame round two
        bays over L2-L3, its thick top bar at the L4 floor, and a red frame round
        one bay over L3-L4 with a thick top bar at the roof; 0.7 m posts standing
        0.6 m out. / 内庭の紺の枠（2 柱間 x 2 層）と赤の枠（1 柱間 x 2 層）。"""
        L = a1 - a0
        bay = 7.6
        frames = [("pr2_navy", a0 + L * 0.12, min(2 * bay, L * 0.45), zb + fh - 0.35, zb + 3 * fh)]
        if L > 20.0:
            frames.append(("pr2_red", a0 + L * 0.62, min(bay, L * 0.28), zb + 2 * fh - 0.35, zb + nst * fh))
        for mat, u, w, zlo, zhi in frames:
            t, d = 0.7, 0.6
            f.block(mat, u, u + t, 0.0, d, zlo, zhi)                       # posts / 柱
            f.block(mat, u + w - t, u + w, 0.0, d, zlo, zhi)
            f.block(mat, u + t, u + w - t, 0.0, d, zhi - 1.2, zhi, ends=False)   # top bar / 上枠
            f.block(mat, u + t, u + w - t, 0.0, d, zlo, zlo + 0.45, ends=False)  # bottom bar / 下枠

    def end_wall(self, f, u0, u1, zb, nst, fh, top, landing=None, pairs=()):
        """End walls (sv11): white render with score lines every half storey;
        optional ONE stack of recessed stair landings (railing, louvre hood) at
        u = landing, and stacks of top-hung window pairs (green frames, grey
        awnings) on L2-L4 at the u values in pairs; blank elsewhere.
        妻壁: 半階ごとの目地、踊り場の列 1 本、対の窓（2-4 階）、他は無地。"""
        L = u1 - u0
        opens, lands = [], []
        if landing is not None and u0 + 1.5 < landing < u1 - 1.5:
            c = landing
            for k in range(nst):
                z0 = zb + k * fh
                o = (c - 1.3, c + 1.3, z0 + (0.9 if k else 0.0), z0 + fh - 0.45)
                opens.append(o)
                lands.append(o)
        wins = []
        for c in pairs:
            if not (u0 + 1.8 < c < u1 - 1.8):
                continue
            for k in range(1, nst):
                z0 = zb + k * fh
                for d in (-0.95, 0.95):
                    wins.append((c + d - 0.75, c + d + 0.75, z0 + 1.55, z0 + 2.2))
        opens = sorted(opens + wins)
        for k in range(nst):
            z0, z1 = zb + k * fh, zb + (k + 1) * fh
            ok = [o for o in opens if z0 <= o[2] < z1]
            wall_openings(f, WALL, u0, u1, z0, z1, ok, 3.0)
            for zz in (z1, z0 + fh / 2):
                f.block("pr2_grey", u0, u1, 0.0, 0.02, zz - 0.02, zz + 0.02, ends=False, bottom=False)
        f.face(WALL, u0, u1, Z0 - 0.15, zb, 0.0, 3.0)
        f.face(WALL, u0, u1, zb + nst * fh, top, 0.0, 3.0)
        for o in lands:
            d = 1.5
            f.hplane("concrete", o[0], o[1], -d, 0.0, o[2], True)
            f.hplane("concrete", o[0], o[1], -d, 0.0, o[3], False)
            f.side(WALL, o[0], -d, 0.0, o[2], o[3], 1)
            f.side(WALL, o[1], -d, 0.0, o[2], o[3], -1)
            f.face("pr2_corr", o[0], o[1], o[2], o[3], -d, 3.6)
            if o[2] > zb + 0.5:
                self.rail(f, o[0], o[1], o[2], w=-0.05, post=0.5)
            for j in range(3):
                zz = o[3] + 0.08 + j * 0.13
                f.block("pr2_rail", o[0] - 0.15, o[1] + 0.15, 0.0, 0.45, zz, zz + 0.06)
        for o in wins:
            opening(f, *o, WALL, "pr2_win", depth=0.12, sill=False)
            q = [f.P(o[0] - 0.05, 0.0, o[3] + 0.1), f.P(o[1] + 0.05, 0.0, o[3] + 0.1),
                 f.P(o[1] + 0.05, 0.4, o[3] - 0.05), f.P(o[0] - 0.05, 0.4, o[3] - 0.05)]
            f.m.quad("pr2_rail", q[3], q[2], q[1], q[0])      # sloping awning, faces up and out / 斜めの庇
            f.m.quad("pr2_rail", q[0], q[1], q[2], q[3])

    def office(self, f, side, u0, u1, L, top):
        """Block A: grey ground band with glazed doors, L2-L3 cyan 3 x 3 window
        grids; the north face's east part in royal-blue cladding; the east gable:
        two grids per floor on the right, the lettering upper left (sv15).
        A 棟: 灰色の 1 階帯、2-3 階の水色の格子窓、北面東部の青い外装、東妻の文字。"""
        z1 = Z0 + FA
        # ground storey: the grey band (description) with small windows (sv03);
        # glazed doors only on the east gable at the plaza (sv15)
        # 1 階: 灰色の帯に小窓（東妻だけガラス戸）
        base = "pr2_grey"
        if side == "E":
            opens = [(c - 1.4, c + 1.4, Z0, Z0 + 2.7) for c in self.centres(u0, u1, 6.0)]
            glass = "pr2_glassdoor"
        else:
            opens = [(c - 0.6, c + 0.6, Z0 + 1.3, Z0 + 2.5) for c in self.centres(u0, u1, 3.0)]
            glass = "pr2_win"
        wall_openings(f, base, u0, u1, Z0, z1, opens, 3.0)
        for o in opens:
            opening(f, *o, base, glass, depth=0.15, sill=side != "E")
        f.face(base, u0, u1, Z0 - 0.15, Z0, 0.0, 3.0)
        f.block(WALL, u0, u1, 0.0, 0.25, z1 - 0.3, z1)
        blue_from = 1e9      # (the royal blue seen in sv13 is on E, not A / sv13 の青は E 棟)
        if side == "E" and L > 12:
            cs = [u0 + L * 0.58, u0 + L * 0.84]
        else:
            cs = self.centres(u0, u1, 5.4)
        for k in range(2):
            z0, zz1 = z1 + k * F, z1 + (k + 1) * F
            opens = [(c - 2.2, c + 2.2, z0 + 0.6, z0 + 3.0) for c in cs if c + 2.2 < blue_from]
            wall_openings(f, WALL, u0, min(u1, blue_from), z0, zz1, opens, 3.0)
            for o in opens:
                opening(f, *o, WALL, "pr2_grid", depth=0.2, tu=1.47, sill=True)
            f.block("pr2_grey", u0, u1, 0.0, 0.03, zz1 - 0.05, zz1 + 0.03, ends=False)
        f.face(WALL, u0, u1, z1 + 2 * F, top, 0.0, 3.0)
        f.block(WALL, u0, u1, 0.0, 0.3, top - 0.45, top, ends=False)
        if side == "N" and u1 > L - 0.5:
            # the red panel on the stair core at the west end (sv03; a CCW north edge
            # runs east to west) / 西端の階段室の赤いパネル
            f.block("pr2_red", L - 4.2, L - 0.6, 0.0, 0.15, z1 + F + 0.3, top - 0.7)

    def hall(self, f, side, u0, u1, L, top):
        """Block B: the open canteen (L1), the hall above with two grey louvre
        bands, the dark-blue top panel on the north (sv05), the orange stair
        panel at the NW corner (sv15). / B 棟: 食堂、ルーバー帯、紺の上部パネル。"""
        z1 = Z0 + FA
        D = 3.0
        a0, a1 = u0 + 0.3, u1 - 0.3
        f.face(WALL, u0, a0, Z0 - 0.15, z1, 0.0, 3.0)
        f.face(WALL, a1, u1, Z0 - 0.15, z1, 0.0, 3.0)
        if a1 - a0 > 2.0:
            tiled_face(f, "pr2_canteen", a0, a1, Z0, z1 - 0.4, -D, 4.0, z1 - 0.4 - Z0, v0=Z0)
            f.hplane("concrete", a0, a1, -D, 0.0, z1 - 0.4, False)
            f.hplane("concrete", a0, a1, -D, 0.0, Z0 + 0.01, True)
            f.side(WALL, a0, -D, 0.0, Z0, z1 - 0.4, 1)
            f.side(WALL, a1, -D, 0.0, Z0, z1 - 0.4, -1)
            for u in np.linspace(a0 + 0.3, a1 - 0.3, max(2, int((a1 - a0) // 6.0) + 2)):
                f.block(WALL, u - 0.3, u + 0.3, -0.6, 0.0, Z0, z1 - 0.4, top=False, bottom=False)
            f.face(WALL, a0, a1, z1 - 0.4, z1, 0.0, 3.0)
            f.hplane(WALL, a0, a1, -0.0, 0.0, z1, True)
        f.face(WALL, u0, u1, z1, top, 0.0, 3.0)
        # the dark-blue panel over the hall level, west of the north face's centre
        # (sv05, from the NE: right of the louvres) but short of the NW corner, which
        # sv15 shows white with the Bravura band (a CCW north edge runs east to west)
        # 紺のパネルは北面中央より西（sv05）、北西隅（sv15）までは届かない
        nv = (u0 + L * 0.52, u0 + L * 0.80) if side == "N" and u0 < 0.5 else None
        if nv is not None:
            f.block("pr2_navy", nv[0], nv[1], 0.0, 0.14, z1 + 0.6, top - 0.2)
        for zz in (z1 + 1.6, z1 + 4.2):
            for a, b in (((u0 + 0.6, nv[0] - 0.3), (nv[1] + 0.3, u1 - 0.6)) if nv else ((u0 + 0.6, u1 - 0.6),)):
                if b - a > 1.0:
                    f.face("pr2_louvre", a, b, zz, zz + 1.3, -0.02, 1.0)
                    for j in range(5):
                        f.block("pr2_rail", a, b, 0.0, 0.16, zz + 0.05 + j * 0.26, zz + 0.13 + j * 0.26,
                                ends=False)
        if side == "W" and u0 < 0.5:
            # (a CCW west edge starts at its north end) / 西辺は北端から始まる
            f.block("pr2_orange", 0.2, 3.2, 0.0, 0.3, Z0, top - 1.8)

    def core(self, f, u0, u1, top, grey=False):
        """Stair cores: grey clad (A's west core) or white, a vertical strip of
        grid glazing. / 階段室: 灰色または白、縦の格子窓。"""
        mat = "pr2_grey" if grey else WALL
        L = u1 - u0
        if L > 3.0:
            c = (u0 + u1) / 2
            o = (c - 0.9, c + 0.9, Z0 + 1.0, top - 1.2)
            wall_openings(f, mat, u0, u1, Z0 - 0.15, top, [o], 3.0)
            opening(f, *o, mat, "pr2_grid", depth=0.12, tu=1.8, sill=False)
        else:
            f.face(mat, u0, u1, Z0 - 0.15, top, 0.0, 3.0)

    def curtain(self, f, u0, u1, top):
        """A's west stair bay: green-glass curtain wall, 4 tiers over 3 floors,
        three thin projecting fins (sv13). / A 西端のガラスのカーテンウォール。"""
        tiers = np.linspace(Z0, top - 0.4, 5)
        for z0, z1 in zip(tiers, tiers[1:]):
            tiled_face(f, "pr2_curtain", u0, u1, z0, z1, 0.0, 1.5, z1 - z0, v0=z0)
        for z in tiers[1:4]:
            f.block(WALL, u0 - 0.1, u1 + 0.1, 0.0, 0.7, z - 0.06, z + 0.06)
        f.face(WALL, u0, u1, top - 0.4, top, 0.0, 3.0)

    def glass_stair(self, f, u0, u1, top):
        """The inner court's glass stair tower (photo): a white box drawn as a
        Mondrian panel (black lines, red / yellow / navy fields, gridded glass)
        over its height, white slab bands and corner posts standing out.
        内庭のガラス階段塔: 黒線と赤・黄・紺の面と格子ガラスの白い箱。"""
        L = u1 - u0
        mat = "pr2_mondrian_a" if L > 6.0 else "pr2_mondrian_b"
        f.m.quad(mat, f.P(u0, 0.0, Z0), f.P(u1, 0.0, Z0), f.P(u1, 0.0, top - 0.6), f.P(u0, 0.0, top - 0.6))
        for k in range(1, 4):
            z = Z0 + k * F
            f.block(WALL, u0, u1, 0.0, 0.2, z - 0.25, z)
        for u in (u0, u1 - 0.35):
            f.block(WALL, u, u + 0.35, 0.0, 0.25, Z0, top - 0.6, bottom=False)
        f.face(WALL, u0, u1, top - 0.6, top, 0.0, 3.0)

    def stair_tower_face(self, f, u0, u1, top, side):
        """The west stair / lift tower (18.8 m): white, a vertical louvred slot.
        西の階段・昇降機塔: 白、縦のルーバー。"""
        c = (u0 + u1) / 2
        o = (c - 0.7, c + 0.7, Z0 + 1.5, top - 1.5)
        if u1 - u0 > 2.5:
            wall_openings(f, WALL, u0, u1, Z0 - 0.15, top, [o], 3.0)
            opening(f, *o, WALL, "pr2_louvre", depth=0.15, tu=1.4, sill=False)
        else:
            f.face(WALL, u0, u1, Z0 - 0.15, top, 0.0, 3.0)
        for k in range(1, 5):
            f.block("pr2_grey", u0, u1, 0.0, 0.03, Z0 + k * F - 0.04, Z0 + k * F + 0.04, ends=False)

    # -- roofs / 屋根
    def roof(self, m, k, b):
        poly, top, kind = b["poly"], b["top"], b["kind"]
        if kind in ("classroom", "office", "hall"):
            a, bb, c, d = self.rect_corners(poly)
            ridge = b.get("ridge", top + 2.5)
            main_ridge = ridge if kind != "hall" else ridge - 2.0
            ov = 0.9
            r = hip_roof(m, a, bb, c, d, top, main_ridge, ov, "pr2_roof", "pr2_fascia")
            solar_rows(m, *r, top, main_ridge, rows=8)
            if kind == "hall":
                # the raised lantern on the ridge: louvred walls, its own hip (OneMap 17 m)
                # 棟上の越屋根（ルーバー壁と寄棟）
                core = poly.buffer(-9.5, join_style="mitre")
                if core.geom_type == "Polygon" and core.area > 30:
                    ca, cb, cc, cd = self.rect_corners(core)
                    m.prism("pr2_louvre", orient(Polygon([ca, cb, cc, cd]), 1.0), main_ridge - 1.6,
                            main_ridge + 0.5, top=False)
                    hip_roof(m, ca, cb, cc, cd, main_ridge + 0.5, ridge, 0.6, "pr2_roof", "pr2_fascia")
            return
        if kind in ("corridor", "core", "endblock", "stair_tower", "glass_stair", "curtain"):
            par = 0.9 if kind in ("endblock", "core", "stair_tower") else 0.4
            self.flat_roof(m, poly, top, par)
            if kind in ("corridor", "endblock") and poly.area > 40:
                inner = poly.buffer(-0.35, join_style="mitre")
                for q in polys(inner):
                    rail_ring(m, "pr2_rail", q.exterior, top + par, h=1.0, post=2.4)
            if kind == "stair_tower":
                # two louvre panels high on its St 41 face, above the connector roof
                # (sv11) / St 41 側の上部に 2 枚のルーバー
                for a, bb, L, n in B.edges_of(poly):
                    if self.facing(n) == "W" and L > 4.0:
                        f = B.Facade(m, a, bb, n)
                        for c in (L * 0.3, L * 0.7):
                            f.face("pr2_louvre", c - 1.2, c + 1.2, top - 2.4, top - 1.2, -0.12, 1.0)
                            f.side(WALL, c - 1.2, -0.12, 0.0, top - 2.4, top - 1.2, 1)
                            f.side(WALL, c + 1.2, -0.12, 0.0, top - 2.4, top - 1.2, -1)
                            for j in range(4):
                                f.block("pr2_rail", c - 1.2, c + 1.2, -0.1, 0.02, top - 2.35 + j * 0.3,
                                        top - 2.27 + j * 0.3, ends=False)

    def rect_corners(self, poly):
        """Corners of the block's rectangle in frame S, CCW, the long side first.
        敷地座標での矩形の角（反時計回り、長辺から）。"""
        us, vs = zip(*[self.uv(*q) for q in poly.exterior.coords])
        u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
        c = [np.array(self.at(u0, v0)), np.array(self.at(u1, v0)), np.array(self.at(u1, v1)),
             np.array(self.at(u0, v1))]
        if v1 - v0 > u1 - u0:
            c = c[1:] + c[:1]
        return c

    def flat_roof(self, m, poly, z, parapet=0.9):
        """Flat roof with a parapet: outer walls out, inner walls in, cap up.
        陸屋根とパラペット。"""
        for p in polys(poly):
            inner = p.buffer(-0.25, join_style="mitre")
            m.prism(WALL, p, z, z + parapet, top=False)
            for q in polys(inner):
                m.ring_walls(WALL, q.exterior, z, z + parapet, outward=False)
                m.flat("roof_concrete", q, z + 0.01, 3.0)
            m.flat(WALL, p.difference(inner), z + parapet, 1.0)

    # -- signs and lettering / 文字
    def signs(self, m, k, b):
        poly = b["poly"]
        edges = B.edges_of(poly)
        face = lambda side: max(edges, key=lambda e: (self.facing(e[3]) == side) * 1000 + e[2])
        if k == "A":
            a, bb, L, n = face("E")
            f = B.Facade(m, a, bb, n)
            u = L * 0.28
            glyphs_on(m, f, text_poly("Home of the", 0.55, "DejaVuSerif.ttf"), u, Z0 + FA + F + 2.25, 0.06,
                      "pr2_letter", 0.55)
            glyphs_on(m, f, text_poly("OVERCOMERS", 0.85, "DejaVuSerif.ttf"), u + 0.6, Z0 + FA + F + 1.2, 0.06,
                      "pr2_letter", 0.85)
            a, bb, L, n = face("N")
            f = B.Facade(m, a, bb, n)
            z = Z0 + FA + 2 * F - 0.1
            glyphs_on(m, f, text_poly("PASIR RIS CREST SECONDARY SCHOOL", 0.5), L * 0.5, z - 0.05, 0.32,
                      "pr2_letter", 0.5)
            m.quad("pr2_crest", f.P(L * 0.5 - 9.6, 0.33, z - 0.55), f.P(L * 0.5 - 8.4, 0.33, z - 0.55),
                   f.P(L * 0.5 - 8.4, 0.33, z + 0.55), f.P(L * 0.5 - 9.6, 0.33, z + 0.55))
        elif k == "B":
            a, bb, L, n = face("N")
            f = B.Facade(m, a, bb, n)
            z = Z0 + FA + 0.2
            # the white Bravura band at the NW end (sv15) / 北西端の白い Bravura の帯
            f.block(WALL, L - 8.5, L - 1.0, 0.15, 0.3, z, z + 1.3)
            glyphs_on(m, f, text_poly("Bravura", 0.85, "DejaVuSans-Oblique.ttf"), L - 4.75, z + 0.65, 0.31,
                      "pr2_navy", 0.85)
        elif k == "F":
            a, bb, L, n = face("N")
            f = B.Facade(m, a, bb, n)
            glyphs_on(m, f, text_poly("Energico", 1.0, "DejaVuSerif-Italic.ttf"), L * 0.2, Z0 + 4 * F - 0.4,
                      0.32, "pr2_blueclad", 1.0)

    # -- the tower / 塔
    def tower(self, m):
        """Round drum: open ground storey on columns, L2 corridor ring, the L3
        cantilevered balcony, a louvred drum, the lower octagonal ring roof and
        the top cap (sv04, sv15, OneMap). / 円筒の塔: 列柱、廊下、片持ちバルコニー、二段屋根。"""
        c = self.tc
        R = self.tr
        rd, rb = R["drum"], R["balcony"]
        z2, z3 = Z0 + F + 0.4, Z0 + 2 * F + 0.4
        z_roof = self.T["tower"]["z_lower_roof"]
        z_top = self.T["tower"]["z_top"]
        # L1: glazed core, columns, the porch slab toward the loop / 1 階: ガラスの芯、列柱、車寄せの庇
        m.prism("pr2_glassdoor", ngon(c, rd - 2.6), Z0, z2 - 0.4, top=False)
        for k in range(12):
            a = 2 * math.pi * (k + 0.5) / 12
            m.cylinder(WALL, c[0] + (rd - 0.45) * math.cos(a), c[1] + (rd - 0.45) * math.sin(a), Z0, z2 - 0.4,
                       0.25, seg=8, top=False)
        # L2 slab + corridor ring + back wall / 2 階床・廊下・奥の壁
        ring2 = ngon(c, rd).difference(ngon(c, rd - 2.6))
        m.prism(WALL, ring2, z2 - 0.4, z2, top=True, bottom=True)
        tiled = ngon(c, rd - 2.6)
        for a, b, L, n in B.edges_of(orient(tiled, 1.0)):
            f = B.Facade(m, a, b, n)
            tiled_face(f, "pr2_corr", 0.0, L, z2, z3 - 0.4, 0.0, 3.6, z3 - 0.4 - z2, v0=z2)
        rail_ring(m, "pr2_rail", ngon(c, rd - 0.1).exterior, z2, post=1.4)
        for a, b, L, n in B.edges_of(orient(ngon(c, rd), 1.0)):
            f = B.Facade(m, a, b, n)
            f.block("pr2_louvre", 0.0, L, -0.12, 0.0, z3 - 1.3, z3 - 0.4, top=False, ends=False)
        # L3: the cantilevered balcony slab (grey underside) / 3 階の片持ちバルコニー
        ring3 = ngon(c, rb, 24).difference(ngon(c, rd - 2.6))
        m.prism(WALL, ring3, z3 - 0.5, z3, top=True, bottom=False)
        m.flat("concrete", ring3, z3 - 0.5, 2.0, up=False)
        rail_ring(m, "pr2_rail", ngon(c, rb - 0.12, 24).exterior, z3, post=1.4)
        # the L3 drum with its louvre band, the rods to the roof / 3 階の円筒とルーバー帯
        drum3 = ngon(c, rd - 1.6)
        m.prism(WALL, drum3, z3, z_roof - 1.1, top=False)
        m.prism("pr2_louvre", drum3, z_roof - 1.1, z_roof - 0.3, top=False)
        for k in range(8):
            a = math.radians(22.5 + 45 * k)
            r = rb - 0.4
            m.cylinder("pr2_rail", c[0] + r * math.cos(a), c[1] + r * math.sin(a), z3 + 1.0, z_roof - 0.3, 0.04,
                       seg=5, top=False)
        # the lower octagonal ring roof and the top cap (dark green fascias) / 下段の八角屋根と頂部
        r_roof, r_cap = R["roof"], R["cap"]
        oct_tier(m, c, r_roof, r_cap * 0.85, z_roof, z_roof + 1.2, "pr2_roof", "pr2_fascia", r_in=rd - 1.6)
        m.prism("pr2_louvre", ngon(c, r_cap * 0.8, 8, 22.5), z_roof + 1.2, z_top - 1.2, top=False)
        oct_tier(m, c, r_cap, 0.0, z_top - 1.2, z_top, "pr2_roof", "pr2_fascia", r_in=r_cap * 0.8)
        # the drop-off porch slab toward the loop (as far as the road allows) / 車寄せ側の庇
        rp = min(R["porch"], rd + 2.0)
        if rp > rd + 0.4:
            d = self.ev                                        # toward the loop (north) / 北へ
            cc = np.array(c)
            sl = Polygon([tuple(cc + self.eu * 4.5), tuple(cc + self.eu * 4.5 + d * rp),
                          tuple(cc - self.eu * 4.5 + d * rp), tuple(cc - self.eu * 4.5)]).difference(ngon(c, rd - 0.2))
            m.prism(WALL, sl, z2 - 0.75, z2 - 0.4, top=True, bottom=True)
            for s in (-1, 1):
                q = cc + self.eu * 4.0 * s + d * (rp - 0.4)
                m.cylinder(WALL, q[0], q[1], Z0, z2 - 0.75, 0.22, seg=8, top=False)

    # -- other buildings / その他の建物
    def sports_hall(self, m, k, b):
        """White hall box raised on charcoal columns (open 6 m ground storey),
        pilasters, slot windows, a ledge band, the south box, the west steel stair,
        dark green hip roof with deep eaves (sv08-10, 2011 photo, OneMap).
        屋内運動場: 黒い柱の上の白い箱、付柱、縦スリット窓、緑の寄棟屋根。"""
        poly, top, ridge = b["poly"], b["top"], b["ridge"]
        z1 = Z0 + 6.0
        core = poly.buffer(-2.2, join_style="mitre")
        for a, bb, L, n in B.edges_of(orient(core, 1.0)):
            f = B.Facade(m, a, bb, n)
            tiled_face(f, "pr2_hallglass", 0.0, L, Z0, z1, 0.0, 3.0, z1 - Z0, v0=Z0)
        m.flat("concrete", poly.difference(core), z1, 2.0, up=False)
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            cs = self.centres(0.0, L, 5.5, 0.0)
            for u in [0.3] + [c + 2.75 for c in cs[:-1]] + [L - 0.3]:
                f.block("pr2_charcoal", u - 0.3, u + 0.3, -0.6, 0.0, Z0, z1, top=False, bottom=False)
            f.face(WALL, 0.0, L, z1, top, 0.0, 3.0)
            f.block(WALL, 0.0, L, 0.0, 0.35, z1, z1 + 0.6)
            # slim charcoal corner columns full height (sv09) / 隅の細い黒い柱
            for u in (0.0, L - 0.35):
                f.block("pr2_charcoal", u, u + 0.35, 0.0, 0.3, z1 + 0.6, top - 0.2, bottom=False)
            if L > 30:
                # long sides: pilasters every ~11 m, a slot window per bay (2011 photo)
                # 長辺: 約 11 m ごとの付柱、柱間ごとの縦スリット
                ps = self.centres(0.0, L, 11.0, 0.0)
                for c in ps[1:]:
                    u = c - (ps[1] - ps[0]) / 2
                    f.block("pr2_charcoal", u - 0.2, u + 0.2, 0.0, 0.25, z1 + 0.6, top - 0.3, bottom=False)
                for c in ps:
                    f.block("pr2_slot", c - 0.45, c + 0.45, 0.0, 0.08, top - 6.5, top - 1.6, bottom=False)
            else:
                # the gable ends: two slots high in the middle (sv09) / 妻面: 中央上部に 2 本
                for c in (L / 2 - 1.1, L / 2 + 1.1):
                    f.block("pr2_slot", c - 0.4, c + 0.4, 0.0, 0.08, top - 5.0, top - 1.4, bottom=False)
        hip_roof(m, *self.rect_corners(poly), top, ridge, 1.8, "pr2_roofgreen", "pr2_charcoal")
        # the cantilevered south box (13.5 m) / 南面の張り出し箱
        edges = B.edges_of(poly)
        a, bb, L, n = max(edges, key=lambda e: (self.facing(e[3]) == "S") * 1000 + e[2])
        f = B.Facade(m, a, bb, n)
        s0, s1 = L * 0.12, L * 0.88
        if self.no_go.distance(LineString([f.P(s0, 2.3, 0)[:2], f.P(s1, 2.3, 0)[:2]])) > GAP:
            f.block(WALL, s0, s1, 0.0, 2.1, z1 + 1.0, 13.5)
            f.block("pr2_grid", s0 + 0.6, s1 - 0.6, 2.1, 2.14, z1 + 3.2, z1 + 5.0, top=False, bottom=False,
                    ends=False)
        # the external steel stair at the west end (2011 photo) / 西端の外階段
        a, bb, L, n = max(edges, key=lambda e: (self.facing(e[3]) == "W") * 1000 + e[2])
        f = B.Facade(m, a, bb, n)
        if self.no_go.distance(Point(*f.P(L / 2, 3.5, 0)[:2])) > GAP + 2.0:
            for i in range(3):
                z0 = Z0 + i * 2.0
                u = L * 0.25 + (i % 2) * 4.0
                du = 4.0 if i % 2 == 0 else -4.0
                q = [f.P(u, 0.4, z0), f.P(u + du, 0.4, z0 + 2.0), f.P(u + du, 1.6, z0 + 2.0), f.P(u, 1.6, z0)]
                if du < 0:
                    q = [q[1], q[0], q[3], q[2]]
                m.quad("pr2_charcoal", *q)
                m.quad("pr2_charcoal", q[1], q[0], q[3], q[2])
                rail_line(m, "pr2_charcoal", f.P(u, 1.6, 0)[:2], f.P(u + du, 1.6, 0)[:2], z0 + 1.0, 1.0, 2.0)

    def joule(self, m, k, b):
        """The Joule Centre (most likely OSM 213639019): white single storey,
        ventilation blocks, brown double doors, green metal hip roof, its own
        chain-link fence (sv12). / ジュールセンター: 白い平屋、緑の寄棟、換気口、茶色の扉。"""
        poly, top = b["poly"], b["top"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face(WALL, 0.0, L, Z0 - 0.15, top, 0.0, 3.0)
            if L > 10:
                for c in (L * 0.3, L * 0.7):
                    f.block("door_wood", c - 0.9, c + 0.9, 0.0, 0.04, Z0, Z0 + 2.4, bottom=False)
                f.block("pr2_louvre", L * 0.45, L * 0.55, 0.0, 0.1, Z0 + 2.6, Z0 + 3.4)
            else:
                f.block("pr2_louvre", L * 0.3, L * 0.7, 0.0, 0.1, Z0 + 0.4, Z0 + 1.4)
        hip_roof(m, *self.rect_corners(poly), top, b["ridge"], 0.6, "pr2_roofgreen", "pr2_fascia")
        ring = poly.buffer(1.5, join_style="mitre").exterior
        cc = list(ring.coords)
        for p, q in zip(cc, cc[1:]):
            chainlink(m, "pr2_fence", p, q, Z0, Z0 + 1.8, pitch=0.25)
        for k2 in range(int(ring.length // 3.0)):
            q = ring.interpolate(k2 * 3.0)
            m.box("pr2_fence", q.x, q.y, Z0, Z0 + 1.9, 0.06, 0.06)

    def store(self, m, k, b):
        """Field store module: white walls, roller door, green roof. / 校庭の倉庫。"""
        poly, top = b["poly"], b["top"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face(WALL, 0.0, L, Z0 - 0.15, top, 0.0, 3.0)
            if L > 4.5 and self.facing(n) == "S":
                f.block("pr2_grey", L * 0.25, L * 0.75, 0.0, 0.04, Z0, Z0 + 2.2, bottom=False)
        hip_roof(m, *self.rect_corners(poly), top, b["ridge"], 0.4, "pr2_roofgreen", "pr2_fascia")

    def bike_shelter(self, m, k, b):
        """Bicycle bay: steel posts, a green mono-pitch roof, racks. / 駐輪場。"""
        poly = b["poly"]
        a, bb, c, d = self.rect_corners(poly)
        for p in (a, bb, c, d):
            m.box("pr2_rail", p[0], p[1], Z0, Z0 + 2.5, 0.1, 0.1)
        m.flat("pr2_roofgreen", poly.buffer(0.3, join_style="mitre"), Z0 + 2.55, 2.0)
        m.flat("pr2_roofgreen", poly.buffer(0.3, join_style="mitre"), Z0 + 2.5, 2.0, up=False)
        ax = (bb - a) / np.linalg.norm(bb - a)
        for s in np.arange(0.8, np.linalg.norm(bb - a) - 0.5, 0.9):
            p = a + ax * s + (d - a) * 0.5
            m.box("pr2_rail", p[0], p[1], Z0, Z0 + 0.8, 0.04, 1.6, math.atan2(ax[1], ax[0]))

    def porch(self, m, k, b):
        """B's NW porch: white columns, a flat slab with a fascia (OneMap 5 m).
        B 北西の玄関庇。"""
        poly, top = b["poly"], b["top"]
        m.prism(WALL, poly, top - 0.45, top, top=True, bottom=True)
        for p in list(poly.buffer(-0.4, join_style="mitre").exterior.coords)[:-1]:
            m.cylinder(WALL, p[0], p[1], Z0, top - 0.45, 0.2, seg=8, top=False)

    # -- grounds / 敷地
    def field_works(self, m):
        """The raised field: retaining walls round the plateau, the 6 m ball
        fence with the blue windbreak (lower 2 m) on the road sides, 3 m on the
        others, goals, steps from the drive. / 嵩上げした校庭: 擁壁、高い防球ネット、ゴール、階段。"""
        zf = Z0 + FIELD_Z
        pl = self.plateau
        if pl.is_empty:
            return
        m.ring_walls("concrete", orient(pl, 1.0).exterior, Z0 - 0.15, zf, outward=True)
        m.flat("concrete", pl.exterior.buffer(0.12).intersection(pl), zf + 0.015, 1.0)
        ring = pl.buffer(-0.3, join_style="mitre").exterior
        cc = list(orient(Polygon(ring), 1.0).exterior.coords)
        for p, q in zip(cc, cc[1:]):
            p, q = np.array(p), np.array(q)
            n = np.array([q[1] - p[1], p[0] - q[0]])
            n /= max(np.linalg.norm(n), 1e-9)
            side = self.facing(n)
            tall = side in ("N", "E")
            h = 6.0 if tall else 3.0
            two_sided_wall(m, "pr2_windbreak", p, q, zf, zf + 2.0, gap=0.01, uvs=2.0)
            chainlink(m, "pr2_fence", p, q, zf + 2.0, zf + h, pitch=0.3, w=0.02)
            for zz in (zf + 2.0, zf + h):
                two_sided_wall(m, "pr2_fence", p, q, zz - 0.03, zz + 0.03, gap=0.04)
            L = float(np.linalg.norm(q - p))
            t = (q - p) / max(L, 1e-9)
            for s in np.arange(0.0, L, 3.0):
                r = p + t * s
                m.box("pr2_fence", r[0], r[1], zf, zf + h, 0.08, 0.08)
        # shrubs planted on the bank (sv06) / 法面の植え込み
        bank = self.zones()["bank"]
        if not bank.is_empty:
            rng = np.random.default_rng(41)
            for g in polys(bank):
                inner_ = g.buffer(-0.6)
                if inner_.is_empty:
                    continue
                x0_, y0_, x1_, y1_ = inner_.bounds
                n_ = int(inner_.area / 2.2)
                xs = rng.uniform(x0_, x1_, n_ * 3)
                ys = rng.uniform(y0_, y1_, n_ * 3)
                ok_ = shapely.contains_xy(inner_, xs, ys)
                xs, ys = xs[ok_][:n_], ys[ok_][:n_]
                zs = self.bank_z(xs, ys)
                for x_, y_, z_ in zip(xs, ys, zs):
                    r_ = rng.uniform(0.35, 0.7)
                    m.sphere("leaves_dark", x_, y_, z_ + r_ * 0.4, r_, r_ * rng.uniform(0.8, 1.2), r_ * 0.7,
                             seg=5, rings=3)
        # goals at both ends / 両端のゴール
        q = [np.array(p) for p in list(self.field.exterior.coords)[:4]]
        if math.dist(q[0], q[1]) > math.dist(q[1], q[2]):
            q = q[1:] + q[:1]
        cen = sum(q) / 4
        for p0, p1 in ((q[0], q[1]), (q[2], q[3])):
            mid = (p0 + p1) / 2
            t = (p1 - p0) / np.linalg.norm(p1 - p0)
            inw = (cen - mid) / np.linalg.norm(cen - mid)
            for s in (-3.66, 3.66):
                r = mid + t * s
                m.box("goal_white", r[0], r[1], zf, zf + 2.44, 0.12, 0.12)
            m.box("goal_white", mid[0], mid[1], zf + 2.32, zf + 2.44, 7.44, 0.12, math.atan2(t[1], t[0]))
            back = mid + inw * 1.5
            m.box("net_dark", back[0], back[1], zf, zf + 0.06, 7.3, 1.5, math.atan2(t[1], t[0]))
        # steps up from the drive on the west side (three access gates, plan)
        # 西側の 3 か所の階段（計画図の門）
        west = max(B.edges_of(orient(pl, 1.0)), key=lambda e: (self.facing(e[3]) == "W") * 1000 + e[2])
        a, b, L, n = west
        for s in (0.25, 0.5, 0.75):
            p = a + (b - a) * s + n * 1.6
            if self.no_go.distance(Point(*p).buffer(2.0)) > 0.3 and not self.footprint.intersects(Point(*p).buffer(2)):
                for i in range(6):
                    c = p - n * (0.2 + i * 0.35)
                    m.box("concrete", c[0], c[1], Z0, Z0 + FIELD_Z * (i + 1) / 6, 2.4, 0.35,
                          math.atan2(n[1], n[0]) + math.pi / 2)

    def site_fence(self, m):
        """Green chain-link on a low kerb with a clipped hedge inside (St 41,
        sv03/11/12), open at the gates; by the field on a retaining wall (sv06).
        緑の金網フェンスと生垣（門は開ける）、校庭沿いは擁壁の上。"""
        ring = max(polys(self.lot.buffer(-0.5, join_style="mitre")), key=lambda q: q.area).exterior
        line = ring.difference(self.no_go.buffer(1.0))
        for g in self.gates:
            line = line.difference(g.buffer(1.5))
        P = self.T["points"]
        for nm in ("side_gate", "back_gate"):
            q = Point(*self.X(P[nm]["xy"]))
            pq = ring.interpolate(ring.project(q))
            if pq.distance(q) < 12.0:
                line = line.difference(pq.buffer(1.2))
        # the low wall replaces the chain-link only where the bank is (+1 m); posts
        # use the same test as their segments / 法面の所（+1 m）だけ擁壁、支柱も同じ判定
        near_field = self.zones()["bank"].buffer(1.0)
        foot = self.footprint.buffer(0.6)
        # the line is split at the bank zone: long ring segments must not carry the
        # wall past it (a 40 m segment whose midpoint was on the bank walled B's car
        # park) / 法面の範囲で線を切る（長い辺の中点判定は擁壁を延ばした）
        wall_line = line.intersection(near_field)
        for g in getattr(wall_line, "geoms", [wall_line]):
            if g.geom_type != "LineString" or g.length < 0.3:
                continue
            m.prism("concrete", g.buffer(0.15, cap_style="flat"), Z0 - 0.15, Z0 + BANK_FOOT, top=True)
        fence_line = line.difference(near_field)
        for g in getattr(fence_line, "geoms", [fence_line]):
            if g.geom_type != "LineString" or g.length < 1.0:
                continue
            c = list(g.coords)
            zb = Z0 + 0.2
            for a, b in zip(c, c[1:]):
                mid = Point((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                if foot.contains(mid):
                    continue
                chainlink(m, "pr2_fence", a, b, zb, zb + 1.6, pitch=0.25)
                two_sided_wall(m, "pr2_fence", a, b, zb + 1.57, zb + 1.63, gap=0.04)
            for k in range(int(g.length // 3.0) + 1):
                q = g.interpolate(min(g.length, k * 3.0))
                if foot.contains(q):
                    continue
                m.box("pr2_fence", q.x, q.y, Z0, Z0 + 1.9, 0.06, 0.06)
            if g.length > 2:
                m.prism("concrete", g.buffer(0.12, cap_style="flat"), Z0 - 0.05, Z0 + 0.2, top=True)
                # the hedge on the inside (either offset side, kept within the lot, off
                # roads, drives and pavements) / 内側の生垣（敷地内、道路・歩道外）
                band = unary_union([g.offset_curve(d).buffer(0.45, cap_style="flat") for d in (-0.8, 0.8)])
                hp = band.intersection(self.inner()).difference(foot.buffer(0.5)) \
                    .difference(near_field).difference(self.no_go.buffer(0.5)) \
                    .difference(unary_union([cc for cc, _ in self.courts.values()]).buffer(0.3))
                m.prism("leaves_dark", hp, Z0, Z0 + 1.1, top=True)

    def vehicle_gate(self, m, g):
        """Blue steel sliding gates parked open along the fence on both sides,
        on white piers (sv04). / 青い引き戸の門（両側に開いて停まる）と白い門柱。"""
        a, b = np.array(g.coords[0]), np.array(g.coords[-1])
        L = float(np.linalg.norm(b - a))
        t = (b - a) / max(L, 1e-6)
        ang = math.atan2(t[1], t[0])
        inward = np.array([-t[1], t[0]])
        cen = np.array(self.lot.centroid.coords[0])
        if float((cen - (a + b) / 2) @ inward) < 0:
            inward = -inward
        half = min(L / 2, 4.5)
        for s, p in ((-1, a), (1, b)):
            pier = p + t * s * 0.8 + inward * 0.4
            m.box(WALL, pier[0], pier[1], Z0, Z0 + 2.4, 0.6, 0.6, ang)
            m.box(WALL, pier[0], pier[1], Z0 + 2.4, Z0 + 2.5, 0.7, 0.7, ang)
            # the leaf, parked behind the fence on its side / 扉は脇のフェンス沿い
            c0 = pier + t * s * (0.4 + half / 2) + inward * 0.35
            for zz in (Z0 + 0.12, Z0 + 1.9):
                m.box("pr2_gate", c0[0], c0[1], zz, zz + 0.1, half, 0.06, ang)
            for k in range(int(half / 0.15)):
                r = c0 + t * (-half / 2 + 0.07 + k * 0.15)
                m.box("pr2_gate", r[0], r[1], Z0 + 0.12, Z0 + 2.0, 0.035, 0.035, ang, top=False)

    def guard_house(self, m):
        """White flat-roofed guard house with a dark fascia, window and door (sv04).
        守衛所: 白、平屋根、濃色の鼻隠し。"""
        c = self.X(self.T["points"]["guard_house"]["xy"])
        poly = self.rect_at(c, 3.6, 3.4)
        if self.no_go.distance(poly) < GAP:
            return
        m.prism(WALL, poly, Z0, Z0 + 2.8, top=False)
        roof = poly.buffer(0.45, join_style="mitre")
        if self.no_go.distance(roof) > 0.3:
            m.prism("pr2_charcoal", roof, Z0 + 2.8, Z0 + 3.1, top=True, bottom=True)
        for a, b, L, n in B.edges_of(orient(poly, 1.0)):
            f = B.Facade(m, a, b, n)
            s = self.facing(n)
            if s == "N":
                f.block("glass_dark", 0.6, 2.2, 0.0, 0.03, Z0 + 1.1, Z0 + 2.2, bottom=False)
            elif s == "W":
                f.block("glass_dark", 0.8, 2.6, 0.0, 0.03, Z0 + 1.1, Z0 + 2.2, bottom=False)
            elif s == "S":
                f.block("door_wood", 1.2, 2.1, 0.0, 0.03, Z0, Z0 + 2.1, bottom=False)

    def rect_at(self, c, w, h):
        uc, vc = self.uv(*c)
        return self.rect(uc - w / 2, uc + w / 2, vc - h / 2, vc + h / 2)

    def name_wall(self, m):
        """The dark green tiled name wall with the white plaque (school names
        and a drawn crest stand-in) on a low planter (sv04). / 校名の壁。"""
        c = self.X(self.T["points"]["name_wall"]["xy"])
        wall = self.rect_at(c, 7.2, 0.5)
        if self.no_go.distance(wall) < GAP:
            return
        m.prism("pr2_tiles", wall, Z0, Z0 + 2.4, top=False, uv_scale=1.0)
        m.flat(WALL, wall, Z0 + 2.4, 1.0)
        planter = self.rect_at(self.at(*np.add(self.uv(*c), (0.0, 0.9))), 8.0, 1.1)
        m.prism("concrete", planter.difference(wall), Z0, Z0 + 0.45, top=False)
        m.flat("leaves_dark", planter.difference(wall), Z0 + 0.5, 1.0)
        a, b, L, n = max(B.edges_of(orient(wall, 1.0)), key=lambda e: (self.facing(e[3]) == "N") * 100 + e[2])
        f = B.Facade(m, a, b, n)
        m.quad("pr2_plaque", f.P(L / 2 - 2.6, 0.03, Z0 + 0.8), f.P(L / 2 + 2.6, 0.03, Z0 + 0.8),
               f.P(L / 2 + 2.6, 0.03, Z0 + 2.15), f.P(L / 2 - 2.6, 0.03, Z0 + 2.15))

    def planter(self, m, isl):
        """The loop's round planter: a low wall, shrubs, the PRCS letters (sv04).
        車寄せの丸い植え込みと PRCS の文字。"""
        p = isl.buffer(-0.35)
        if p.is_empty or p.area < 4.0:
            return
        p = max(polys(p), key=lambda q: q.area)
        ring = p.difference(p.buffer(-0.25))
        m.prism("concrete", ring, Z0, Z0 + 0.4, top=True)
        m.flat("grass_park", p.buffer(-0.25), Z0 + 0.3, 2.0)
        c = p.centroid
        # upright grey letters on the planter's gate side, facing north (sv04):
        # a front and a back face 0.15 m apart, on a low plinth
        # 門側（北）を向いて立つ灰色の文字（表と裏）
        cc = np.array([c.x, c.y])
        r = min(p.bounds[2] - p.bounds[0], p.bounds[3] - p.bounds[1]) * 0.5
        base = cc + self.ev * max(0.0, r - 1.3)
        g = text_poly("PRCS", 0.8, "DejaVuSans-Bold.ttf")
        w = (g.bounds[2] - g.bounds[0]) / 2 + 0.2
        a, b = base + self.eu * w, base - self.eu * w
        front = B.Facade(m, a, b, self.ev)
        back = B.Facade(m, b, a, -self.ev)
        glyphs_on(m, front, g, w, Z0 + 0.75, 0.08, "pr2_letter", 0.8)
        from shapely.affinity import scale as _scale
        glyphs_on(m, back, _scale(g, -1.0, 1.0, origin=(0, 0)), w, Z0 + 0.75, 0.08, "pr2_letter", 0.8)
        m.box("concrete", base[0], base[1], Z0 + 0.3, Z0 + 0.35, 2 * w, 0.4,
              math.atan2(self.eu[1], self.eu[0]))
        for k in range(5):
            a = 2 * math.pi * k / 5
            r = min(p.bounds[2] - p.bounds[0], p.bounds[3] - p.bounds[1]) * 0.3
            x, y = c.x + r * math.cos(a), c.y + r * math.sin(a) + 0.0
            m.sphere("leaves_dark", x, y, Z0 + 0.8, 0.8, 0.8, 0.6, seg=6, rings=4)

    def pavilion(self, m):
        """The green-roofed square pavilion with benches in the Forte / Energico
        courtyard (photo). / 中庭の緑屋根のあずまや。"""
        c = self.X(self.T["points"]["pavilion"]["xy"])
        poly = orient(self.rect_at(c, 4.6, 4.6), 1.0)
        for p in list(poly.exterior.coords)[:-1]:
            m.box(WALL, p[0], p[1], Z0, Z0 + 2.7, 0.25, 0.25)
        hip_roof(m, *self.rect_corners(poly), Z0 + 2.7, Z0 + 4.1, 0.4, "pr2_roofgreen", "pr2_fascia", ins=0.0)
        m.box("concrete", c[0], c[1], Z0, Z0 + 0.75, 1.4, 1.4)
        for s in (-1, 1):
            q = np.array(c) + self.ev * 1.4 * s
            m.box("concrete", q[0], q[1], Z0, Z0 + 0.45, 2.2, 0.45, math.atan2(self.eu[1], self.eu[0]))

    def court_furniture(self, m):
        """The inner court's stage with steps, the flagpole, basketball hoops on
        the outdoor court. / 内庭の舞台と旗竿、屋外コートのゴール。"""
        c = self.X(self.T["points"]["stage"]["xy"])
        st = self.rect_at(c, 10.0, 2.6)
        if not st.intersects(self.footprint.buffer(0.2)):
            m.prism("concrete", st, Z0, Z0 + 0.9, top=True)
            for i in range(3):
                q = self.rect_at(self.at(*np.add(self.uv(*c), (0.0, 1.3 + 0.35 * (i + 0.5)))), 5.0, 0.35)
                m.prism("concrete", q, Z0, Z0 + 0.9 - 0.3 * (i + 1), top=True)
        fx, fy = self.X(self.T["points"]["flagpole"]["xy"])
        m.cylinder("metal_grey", fx, fy, Z0, Z0 + 10.0, 0.07, seg=6, top=True)
        bb = self.courts.get("court_bb")
        if bb is not None:
            q = [np.array(p) for p in list(bb[0].exterior.coords)[:4]]
            if math.dist(q[0], q[1]) > math.dist(q[1], q[2]):
                q = q[1:] + q[:1]
            cen = sum(q) / 4
            for p0, p1 in ((q[0], q[1]), (q[2], q[3])):
                mid = (p0 + p1) / 2
                inw = (cen - mid) / np.linalg.norm(cen - mid)
                t = (p1 - p0) / np.linalg.norm(p1 - p0)
                pole = mid - inw * 0.6
                m.box("metal_grey", pole[0], pole[1], Z0, Z0 + 3.4, 0.15, 0.15)
                bd = mid + inw * 0.6
                m.box("backboard", bd[0], bd[1], Z0 + 2.9, Z0 + 3.95, 1.8, 0.05, math.atan2(t[1], t[0]))
                arm = (pole + bd) / 2
                m.box("metal_grey", arm[0], arm[1], Z0 + 3.3, Z0 + 3.4, 0.1, 1.2,
                      math.atan2(t[1], t[0]))

    def ne_shelter(self, m):
        """The green sloped shelter at the NE pedestrian gate (sv16). / 北東の通用門の屋根。"""
        c = self.X(self.T["points"]["ne_shelter"]["xy"])
        poly = self.rect_at(c, 4.0, 3.0)
        if self.no_go.distance(poly) < GAP:
            return
        for p in list(poly.exterior.coords)[:-1]:
            m.box("pr2_rail", p[0], p[1], Z0, Z0 + 2.6, 0.1, 0.1)
        m.flat("pr2_roofgreen", poly.buffer(0.25, join_style="mitre"), Z0 + 2.7, 2.0)
        m.flat("pr2_roofgreen", poly.buffer(0.25, join_style="mitre"), Z0 + 2.65, 2.0, up=False)

    def walkways(self, m):
        """Covered walkways (white posts, flat dark green roof) round the sports
        hall's east and south sides along Dr 4 and the access road (sv09, sv10).
        屋内運動場の東・南を回る屋根付き通路。"""
        sh = self.blocks.get("sports_hall")
        if sh is None:
            return
        u0, u1, v0, v1 = sh["uv"]
        line = LineString([self.at(u0 - 2.0, v0 - 3.2), self.at(u1 + 3.2, v0 - 3.2), self.at(u1 + 3.2, v1 + 8.0)])
        ok = self.inner().buffer(-1.4).difference(self.no_go.buffer(GAP + 1.4)).difference(self.footprint.buffer(0.4))
        part = line.intersection(ok)
        for g in getattr(part, "geoms", [part]):
            if g.geom_type != "LineString" or g.length < 4.0:
                continue
            roof = g.buffer(1.3, cap_style="flat", join_style="mitre")
            m.flat("roof_linkway", roof, Z0 + 2.9, 2.0)
            m.flat("roof_linkway", roof, Z0 + 2.75, 2.0, up=False)
            m.ring_walls("pr2_fascia", orient(roof, 1.0).exterior if roof.geom_type == "Polygon" else
                         max(polys(roof), key=lambda q: q.area).exterior, Z0 + 2.75, Z0 + 2.9)
            for k in range(int(g.length // 4.0) + 1):
                q = g.interpolate(min(g.length, k * 4.0))
                m.box(WALL, q.x, q.y, Z0, Z0 + 2.75, 0.15, 0.15)

    def trees(self, m):
        """Palms in the gardens, rain trees by the fence (imagery). / 庭のヤシ、塀沿いの雨の木。"""
        import props
        busy = unary_union([self.footprint.buffer(2.5), self.plateau.buffer(1.0)]
                           + [c.buffer(1.5) for c, _ in self.courts.values()])
        for kind, pts in (("palm", self.T["trees"]["palms"]), ("rain", self.T["trees"]["rain"])):
            for q in pts:
                x, y = self.X(q)
                p = Point(x, y)
                R = {"rain": 6.0 * 0.8, "palm": 3.2 * 0.9}[kind]
                if busy.contains(p) or not self.lot.contains(p) or self.no_go.distance(p) < R + 0.6:
                    continue
                props.Props.tree_mesh(None, m, x, y, kind, 0.8 if kind == "rain" else 0.9, False)


def write_textures(tex_dir):
    """The school's drawn textures (procedural; colours from description.md,
    nothing taken from imagery): renders, glazing, louvres, roofs, solar
    panels, the corridor and canteen walls, the name plaque and a drawn crest
    stand-in. / 学校の描画テクスチャ（description.md の色、画像は使わない）。"""
    import textures as X
    from PIL import Image, ImageDraw, ImageFont
    out = {}

    def save(name, im, rough=0.8, metal=0.0, relief=0.0):
        im = im.convert("RGB")
        im.save(os.path.join(tex_dir, name + ".png"))
        if relief > 0:
            h = np.asarray(im.convert("L"), np.float64) / 255.0
            h -= h.mean()
            X.normal_map(h / max(np.abs(h).max(), 1e-6), relief).save(os.path.join(tex_dir, name + "_n.png"))
        Image.fromarray((X.srgb_encode(np.full((8, 8), rough)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_r.png"))
        Image.fromarray((X.srgb_encode(np.full((8, 8), metal)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_m.png"))
        out[name] = (name, None, rough, metal, False)

    save(WALL, X.paint(512, (214, 213, 208), 0.06), 0.85, 0.0, 0.8)
    save("pr2_grey", X.paint(256, (150, 153, 151), 0.05), 0.8)
    save("pr2_letter", X.paint(64, (154, 155, 150), 0.02), 0.5, 0.3)
    save("pr2_louvre", X.stripes(256, (95, 100, 98), (150, 156, 153), 16), 0.6, 0.3, 1.5)
    save("pr2_rail", X.paint(64, (123, 129, 129), 0.02), 0.4, 0.6)
    save("pr2_charcoal", X.paint(64, (45, 48, 56), 0.02), 0.5, 0.4)
    save("pr2_fascia", X.paint(64, (86, 97, 83), 0.03), 0.5, 0.4)
    save("pr2_navy", X.paint(128, (18, 30, 60), 0.04), 0.7)
    save("pr2_red", X.paint(128, (150, 58, 46), 0.04), 0.7)
    save("pr2_yellow", X.paint(128, (215, 182, 70), 0.04), 0.7)
    save("pr2_orange", X.paint(128, (206, 142, 83), 0.04), 0.8)
    save("pr2_blueclad", X.stripes(256, (56, 96, 156), (64, 106, 168), 8, vertical=True), 0.5, 0.3, 0.6)
    save("pr2_gate", X.paint(64, (79, 121, 156), 0.02), 0.4, 0.5)
    save("pr2_fence", X.paint(64, (56, 72, 40), 0.03), 0.6, 0.3)
    save("pr2_windbreak", X.stripes(128, (118, 155, 180), (134, 170, 194), 32), 0.9)
    save("pr2_roof", X.stripes(256, (168, 167, 160), (186, 184, 176), 20, vertical=True), 0.5, 0.5, 1.5)
    save("pr2_roofgreen", X.stripes(256, (62, 84, 70), (74, 98, 82), 20, vertical=True), 0.5, 0.4, 1.5)

    def grid(frame, g0, g1, cols, rows, n=256, fw=7):
        y = np.linspace(0, 1, n)[:, None, None]
        g = np.array(g1)[None, None, :] * (1 - y) + np.array(g0)[None, None, :] * y
        im = Image.fromarray(np.repeat(g, n, axis=1).astype(np.uint8))
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, n - 1, n - 1], outline=frame, width=fw)
        for c in range(1, cols):
            x = int(n * c / cols)
            d.rectangle([x - fw // 2, 0, x + fw // 2, n], fill=frame)
        for r in range(1, rows):
            yy = int(n * r / rows)
            d.rectangle([0, yy - fw // 2, n, yy + fw // 2], fill=frame)
        return im

    CY0, CY1 = (110, 135, 118), (160, 182, 165)        # cyan / green tint (145,164,145) / 水色ガラス
    save("pr2_grid", grid((200, 202, 200), CY0, CY1, 1, 3), 0.1, 0.2)
    save("pr2_win", grid((50, 95, 60), CY0, CY1, 2, 1, fw=10), 0.1, 0.2)
    save("pr2_curtain", grid((70, 95, 88), (70, 110, 100), (115, 155, 145), 2, 1, fw=5), 0.08, 0.3)
    save("pr2_glassdoor", grid((60, 62, 66), (45, 60, 64), (90, 110, 112), 2, 1, fw=8), 0.1, 0.2)
    save("pr2_hallglass", grid((45, 48, 56), (40, 48, 54), (75, 88, 92), 3, 2, fw=6), 0.3, 0.3)
    save("pr2_slot", X.stripes(128, (40, 44, 50), (120, 126, 128), 20), 0.5, 0.3, 1.0)

    def mondrian(cells, W=512, H=1536):
        """The glass stair tower's face: white, thick black lines, coloured fields
        and gridded glass, as in the court photo (drawn, not traced).
        ガラス階段塔の面（黒線、色面、格子ガラス）。"""
        im = Image.new("RGB", (W, H), (232, 232, 228))
        d = ImageDraw.Draw(im)
        col = {"r": (170, 50, 40), "y": (225, 185, 50), "n": (30, 48, 95), "w": (232, 232, 228)}
        for x0, y0, x1, y1, kind in cells:
            box_ = [int(x0 * W), int(y0 * H), int(x1 * W), int(y1 * H)]
            if kind == "g":
                d.rectangle(box_, fill=(55, 70, 78))
                for i in (1, 2):
                    xx = box_[0] + (box_[2] - box_[0]) * i // 3
                    yy = box_[1] + (box_[3] - box_[1]) * i // 3
                    d.line([(xx, box_[1]), (xx, box_[3])], fill=(225, 225, 222), width=6)
                    d.line([(box_[0], yy), (box_[2], yy)], fill=(225, 225, 222), width=6)
                d.rectangle([box_[0] + 8, box_[1] + 8, box_[0] + (box_[2] - box_[0]) // 3 - 4,
                             box_[1] + (box_[3] - box_[1]) // 3 - 4], fill=(95, 120, 125))
            else:
                d.rectangle(box_, fill=col[kind])
            d.rectangle(box_, outline=(20, 20, 22), width=10)
        return im

    # (v from the top: 0 = roof) / 上から
    save("pr2_mondrian_a", mondrian([(0, 0, 0.62, 0.08, "n"), (0.62, 0, 1, 0.08, "y"),
                                     (0.05, 0.1, 0.7, 0.32, "g"), (0.05, 0.34, 0.7, 0.56, "g"),
                                     (0.05, 0.58, 0.7, 0.8, "g"), (0.72, 0.1, 1, 0.45, "w"),
                                     (0.72, 0.45, 1, 0.8, "w"), (0, 0.82, 1, 1, "w")]), 0.4, 0.1)
    save("pr2_mondrian_b", mondrian([(0, 0, 1, 0.08, "n"), (0.08, 0.1, 0.45, 0.62, "w"),
                                     (0.45, 0.1, 0.62, 0.8, "r"), (0.62, 0.1, 1, 0.45, "w"),
                                     (0.62, 0.45, 1, 0.8, "r"), (0.08, 0.62, 0.45, 0.8, "g"),
                                     (0, 0.82, 1, 1, "w")]), 0.4, 0.1)

    def lines(size, base, ln, draw, margin=None, mcol=None, noise=0.03):
        im = X.paint(max(size), base, noise).resize(size)
        d = ImageDraw.Draw(im)
        if margin is not None:
            d.rectangle([0, 0, size[0] - 1, size[1] - 1], outline=mcol, width=margin)
        draw(d, size, ln)
        return im

    def football(d, sz, ln):
        # 86 x 64.5 m at 12 px/m: touchlines, halfway, circle, boxes / サッカーの線
        W, H = sz
        k = W / 86.0
        w = max(3, int(0.12 * k))
        d.rectangle([2, 2, W - 3, H - 3], outline=ln, width=w)
        d.line([(W // 2, 0), (W // 2, H)], fill=ln, width=w)
        r = 9.15 * k
        d.ellipse([W / 2 - r, H / 2 - r, W / 2 + r, H / 2 + r], outline=ln, width=w)
        for x0, sgn in ((0, 1), (W, -1)):
            for bw, bh in ((16.5, 40.3), (5.5, 18.3)):
                xa, xb = sorted((x0, x0 + sgn * bw * k))
                d.rectangle([xa, H / 2 - bh * k / 2, xb, H / 2 + bh * k / 2], outline=ln, width=w)

    def basketball(d, sz, ln):
        # 28 x 15 m (u along the long side) / バスケットコートの線
        W, H = sz
        k = W / 28.0
        w = max(3, int(0.06 * k))
        d.rectangle([3, 3, W - 4, H - 4], outline=ln, width=w)
        d.line([(W // 2, 0), (W // 2, H)], fill=ln, width=w)
        r = 1.8 * k
        d.ellipse([W / 2 - r, H / 2 - r, W / 2 + r, H / 2 + r], outline=ln, width=w)
        for x0, sgn in ((0, 1), (W, -1)):
            xa, xb = sorted((x0, x0 + sgn * 5.8 * k))
            d.rectangle([xa, H / 2 - 2.45 * k, xb, H / 2 + 2.45 * k], outline=ln, width=w)
            R = 6.75 * k
            cx = x0 + sgn * 1.6 * k
            d.arc([cx - R, H / 2 - R, cx + R, H / 2 + R], -90 if sgn > 0 else 90, 90 if sgn > 0 else 270,
                  fill=ln, width=w)

    # the astro field (33,67,73 on the tiles), the blue inner court (32,65,108),
    # the green court with a blue margin (56,105,105) / 人工芝、青い内庭コート、緑のコート
    save("pr2_astro", lines((1032, 774), (40, 78, 74), (225, 230, 228), football, noise=0.05), 0.9)
    save("pr2_court_blue", lines((1024, 1024), (40, 72, 118), (235, 235, 235), basketball, 60, (70, 78, 90)), 0.7)
    save("pr2_court_green", lines((1024, 1024), (56, 108, 100), (235, 235, 235), basketball, 70, (45, 80, 140)), 0.7)
    # PV panels: one panel per tile (cells, a light frame) / 太陽光パネル 1 枚分
    n = 128
    im = Image.new("RGB", (n, n), (70, 76, 92))
    d = ImageDraw.Draw(im)
    for i in range(1, 6):
        d.line([(0, i * n // 6), (n, i * n // 6)], fill=(95, 102, 118), width=1)
    for i in range(1, 4):
        d.line([(i * n // 4, 0), (i * n // 4, n)], fill=(95, 102, 118), width=1)
    d.rectangle([0, 0, n - 1, n - 1], outline=(170, 172, 175), width=3)
    save("pr2_solar", im, 0.15, 0.5)
    # corridor wall: a door and a window per 3.6 m module / 廊下壁: 扉と窓
    W, H = 512, 448
    im = X.paint(512, (205, 206, 203), 0.04).resize((W, H))
    d = ImageDraw.Draw(im)
    d.rectangle([30, 140, 150, H - 1], fill=(120, 98, 78))
    d.rectangle([40, 150, 140, 250], fill=(110, 135, 125))
    d.rectangle([215, 160, 485, 330], fill=(70, 75, 78))
    d.rectangle([222, 167, 478, 323], fill=(130, 158, 145))
    d.line([(350, 160), (350, 330)], fill=(70, 75, 78), width=6)
    d.rectangle([0, H - 26, W, H], fill=(170, 172, 168))
    save("pr2_corr", im, 0.8)
    # canteen back wall: stall counters and tiles / 食堂の奥の壁（売店）
    im = X.paint(512, (190, 192, 188), 0.04).resize((512, 512))
    d = ImageDraw.Draw(im)
    d.rectangle([20, 260, 492, 470], fill=(150, 120, 90))
    d.rectangle([20, 120, 492, 250], fill=(225, 225, 220))
    d.rectangle([40, 140, 230, 230], fill=(200, 80, 60))
    d.rectangle([280, 140, 470, 230], fill=(70, 120, 170))
    save("pr2_canteen", im, 0.8)
    # dark green tiles of the name wall / 校名壁の濃緑タイル
    im = Image.new("RGB", (256, 256), (20, 44, 28))
    d = ImageDraw.Draw(im)
    for i in range(0, 256, 32):
        d.line([(0, i), (256, i)], fill=(12, 28, 18), width=2)
        d.line([(i, 0), (i, 256)], fill=(12, 28, 18), width=2)
    save("pr2_tiles", im, 0.3, 0.1)

    def crest(s):
        """A drawn stand-in for the crest (mountain on a yellow shield, blue sea),
        not the real artwork. / 校章の代わりの絵（本物ではない）。"""
        im = Image.new("RGB", (s, s), (245, 245, 240))
        d = ImageDraw.Draw(im)
        d.ellipse([s * .06, s * .04, s * .94, s * .94], fill=(240, 205, 60), outline=(30, 30, 30), width=max(2, s // 64))
        d.polygon([(s * .2, s * .7), (s * .5, s * .2), (s * .8, s * .7)], fill=(25, 25, 25))
        d.polygon([(s * .5, s * .2), (s * .43, s * .33), (s * .57, s * .33)], fill=(250, 250, 250))
        for k in range(3):
            y = s * (.72 + k * .05)
            d.arc([s * .14, y - s * .04, s * .86, y + s * .04], 0, 180, fill=(40, 90, 180), width=max(2, s // 48))
        return im

    crest(256).save(os.path.join(tex_dir, "pr2_crest.png"))
    for nm in ("pr2_crest",):
        Image.fromarray((X.srgb_encode(np.full((8, 8), 0.5)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, nm + "_r.png"))
        Image.fromarray((X.srgb_encode(np.full((8, 8), 0.0)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, nm + "_m.png"))
        out[nm] = (nm, None, 0.5, 0.0, False)
    # the name plaque: white board, four names, the crest / 校名板
    W, H = 1024, 268
    im = Image.new("RGB", (W, H), (245, 245, 240))
    d = ImageDraw.Draw(im)
    d.rectangle([4, 4, W - 5, H - 5], outline=(30, 70, 45), width=6)
    fb = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    f1, f2 = ImageFont.truetype(fb, 34), ImageFont.truetype(fb, 30)
    d.text((36, 26), "SEKOLAH MENENGAH PASIR RIS CREST", fill=(20, 20, 20), font=f1)
    cjk = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
    if os.path.exists(cjk):
        d.text((36, 80), "励志中学", fill=(20, 20, 20), font=ImageFont.truetype(cjk, 40))
    d.line([(36, 150), (560, 150)], fill=(90, 90, 90), width=3)          # (Tamil line: no font / タミル語は省略)
    d.text((36, 190), "PASIR RIS CREST SECONDARY SCHOOL", fill=(20, 20, 20), font=f2)
    im.paste(crest(210), (W - 240, 28))
    save("pr2_plaque", im, 0.5)
    return out
