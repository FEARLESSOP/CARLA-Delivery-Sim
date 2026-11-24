"""White Sands shopping mall (1 Pasir Ris Central Street 3), PasirRisTampines
(maps/.venv). One terraced 5-storey mall block beside Pasir Ris MRT station.

Sources (build/PasirRisTampines/wsm_photos, visual reference only, never
textures): the labels-off z20 tile (blocks, octagon, plant, solar rows),
OSM way 34575159 (the footprint, as the gate set it), OneMap 3-D tooltips and
its 3 m height grid (heights), Street View sv01-sv11 (facades, colours; RGB
medians in description.md). Trace: build/<Name>/wsm_trace.json (map frame,
written by wsm_trace_make.py beside it; frame S = the south face's axis).

  massing    W strip 29.5 m with AC plant, core 32.5 m round the sunk octagonal
             atrium skylight (rim 28 m, dark glazed pyramid) and a 25 m roof
             north of it (cream panel), the 34.5 m high zone (L-shaped), middle
             strip 28.5 m, a short 21 m terrace on the S edge, the NE nose 22 m,
             the E wing 23 m with solar rows and a screened plant deck (26 m);
             heights from OneMap's grid re-binned in frame S
  E / SE     the metal shell: its wall top an arch in elevation (half ellipse
             10 -> 27.5 m along the shell, sv03 / sv10) with a ribbed strip 6 m
             deep behind it, two-tone panels (dark lower half, light upper, a
             diagonal wave between), the rounded SE corner (8 m fillet), red-
             fascia shops below; white steps where it meets the S and NE walls
  linkway    the covered walkway along the E footpath (flat steel roof, square
             posts with brackets on the mall side, off the road)
  S, W       glass shopfronts under a white canopy (tensile struts on the S,
             blue glass on the W), the bulging light-blue L2 band over a grey
             metal fascia, white render with recessed horizontal lines; the
             dark-framed glazed bay (S); the blue wave band and the royal-blue
             "white sands" lettering high on the W face, a roof fin
  N, NE      ground floor set back behind round white columns (shops, the
             car-park portal "In / Carpark / Out" on a blue fascia where the
             OSM car-park drive meets the face), the blue band, grey perforated
             metal in a wave on the NE, a long ribbon window and a louvre vent;
             the NW entrance: blue fascia with white "white sands", LED screen,
             blue glass canopy, stepped roof fins; the S face's lettering high at
             its W end; glass entrance canopies (S, N, SE)

Left out (rule: nothing over a road): the NE canopy over the St 3 junction,
the height gantry over the taxi drive, the covered links north of the mall.
The MRT station and its link canopy are not part of this landmark.

Roads win: every footprint edge keeps GAP off the road network and the site's
own drives (`fit_edges`: the edge moves in, the block stays a block); any
triangle left touching them is dropped (`drop_over`).
Coordinates: the map frame (x east, y north, metres).

ホワイトサンズ（パシリス・セントラル St 3 のショッピングモール）の詳細モデル。
写真は参照のみ（テクスチャに使わない）。道路の上には何も置かない。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import nearest_points, unary_union

import buildings as B
import geo
from landmarks.coral import drop_over, glyphs_on, opening, tiled_face, wall_openings
from landmarks.dte import beam
from landmarks.prcs import text_poly
from mesh import Mesh, earcut, polys

NAME = "White Sands"
TAG = "WSM"
TRACE = "wsm_trace.json"
MAP = "PasirRisTampines"
SITE_NAMES = ()           # the OSM area is the building itself: site_area() / OSM 区画は建物のみ
Z0 = 0.15                 # the raised ground / 地盤面
GAP = 1.0                 # walls this far off the road network / 道路網からの離れ
PAR = 1.0                 # parapet height / パラペット高さ
SHOP_Z = 4.0              # shopfront glass top (street faces) / 店舗ガラスの上端
BAND = (5.3, 8.3, 9.2)    # blue band bottom, top, grey fascia top / 青帯・灰色鼻隠し
WHITE = "wsm_white"


def enabled(out, meta):
    """Only on PasirRisTampines, with its trace. / この地図とトレースがある時だけ。"""
    return meta.get("name") == MAP and os.path.exists(os.path.join(out, TRACE))


def _xform(T, meta):
    """Trace point -> this build's frame (a re-centred crop converts through
    lon/lat). / トレース座標から現在の座標系へ。"""
    fr = geo.Frame(meta)
    lon0, lat0 = T.get("frame_lonlat0", [fr.lon0, fr.lat0])
    if abs(lon0 - fr.lon0) < 1e-9 and abs(lat0 - fr.lat0) < 1e-9:
        return lambda q: (float(q[0]), float(q[1]))
    tf = geo.Frame({"lat0": lat0, "lon0": lon0})
    return lambda q: fr.xy(*tf.lonlat(float(q[0]), float(q[1])))


def site_area(L, out, T=None):
    """The area check_roads.py --site wsm tests: the footprint + 8 m.
    / 道路確認の範囲（外形 + 8 m）。"""
    T = T or json.load(open(os.path.join(out, TRACE)))
    X = _xform(T, L.meta)
    return Polygon([X(q) for q in T["footprint"]]).buffer(8.0, join_style="mitre")


# -- primitives (tested in test_wsm.py: every face points the way it is seen)
# 基本形状（test_wsm.py で面の向きを確認）
def left(t):
    return np.array([-t[1], t[0]])


def facade(m, a, b):
    """Facade on a -> b facing its right (a CCW footprint's outside).
    / a->b の右（反時計回り外形の外側）を向く外壁。"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    t = (b - a) / np.linalg.norm(b - a)
    return B.Facade(m, a, b, np.array([t[1], -t[0]]))


def fit_edges(poly, no_go, gap, step=0.25, max_move=3.0):
    """Move each edge of ``poly`` in (parallel, in steps) until it keeps ``gap``
    off ``no_go``: a block stays a block (a cut leaves notches). Returns the
    CCW polygon and each edge's move. / 辺を平行に内側へ動かして道路から離す。"""
    p = orient(poly, 1.0)
    pts = [np.array(q, float) for q in list(p.exterior.coords)[:-1]]
    n = len(pts)
    off = np.zeros(n)
    shapely.prepare(no_go)

    def build():
        lines = []
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            t = (b - a) / np.linalg.norm(b - a)
            lines.append((a + left(t) * off[i], t))
        out = []
        for i in range(n):
            (a0, t0), (a1, t1) = lines[i - 1], lines[i]
            cr = t0[0] * t1[1] - t0[1] * t1[0]
            if abs(cr) < 0.05:                    # (nearly) collinear: start of the new edge / ほぼ一直線
                out.append(a1)
            else:
                d = a1 - a0
                s = (d[0] * t1[1] - d[1] * t1[0]) / cr
                out.append(a0 + t0 * s)
        return Polygon(out)

    zone = no_go.buffer(gap - 1e-3)
    for _ in range(n * int(max_move / step) + 1):
        # one edge per step, the one running longest inside the gap: a corner
        # near the road is cleared by its near edge, its neighbour stays
        # 1 回に 1 辺（間隔内を最も長く走る辺）: 角だけ近い隣の辺は動かさない
        c = list(build().exterior.coords)
        runs = [(LineString([c[i], c[i + 1]]).intersection(zone).length, i) for i in range(n) if off[i] < max_move]
        runs = [r for r in runs if r[0] > 1e-6 or no_go.distance(LineString([c[r[1]], c[r[1] + 1]])) < gap - 1e-3]
        if not runs:
            break
        off[max(runs)[1]] += step
    q = orient(build().buffer(0), 1.0)
    if q.geom_type != "Polygon":
        q = orient(max(polys(q), key=lambda g: g.area), 1.0)
    return q, off


def clean_ring(pts, min_len=1.6, max_turn=20.0):
    """Drop ring vertices that make a short edge (< min_len) at a small turn
    (< max_turn deg): offsetting such an edge inwards turns it inside out.
    / 短い辺を作る小さな折れの頂点を除く（内側へずらすと裏返る）。"""
    pts = [np.asarray(q, float) for q in pts]
    changed = True
    while changed and len(pts) > 4:
        changed = False
        n = len(pts)
        for k in range(n):
            a, v, b = pts[k - 1], pts[k], pts[(k + 1) % n]
            l0, l1 = np.linalg.norm(v - a), np.linalg.norm(b - v)
            if min(l0, l1) >= min_len or l0 < 1e-9 or l1 < 1e-9:
                continue
            c = float((v - a) @ (b - v)) / (l0 * l1)
            if math.degrees(math.acos(max(-1.0, min(1.0, c)))) < max_turn:
                del pts[k]
                changed = True
                break
    return [tuple(q) for q in pts]


def fillet(pts, k, r, seg=8):
    """Round the convex corner pts[k] of a CCW ring with radius r. / 角を丸める。"""
    n = len(pts)
    V, A, C = np.asarray(pts[k], float), np.asarray(pts[k - 1], float), np.asarray(pts[(k + 1) % n], float)
    ti, to = (V - A) / np.linalg.norm(V - A), (C - V) / np.linalg.norm(C - V)
    phi = math.acos(max(-1.0, min(1.0, float(ti @ to))))
    if phi < 1e-3 or ti[0] * to[1] - ti[1] * to[0] <= 0:       # straight or concave / 直線・凹角は対象外
        return list(pts)
    tl = min(r * math.tan(phi / 2), 0.45 * np.linalg.norm(V - A), 0.45 * np.linalg.norm(C - V))
    r = tl / math.tan(phi / 2)
    t1 = V - ti * tl
    cen = t1 + left(ti) * r
    a0 = math.atan2(*(t1 - cen)[::-1])
    arc = [tuple(cen + r * np.array([math.cos(a0 + phi * j / seg), math.sin(a0 + phi * j / seg)]))
           for j in range(seg + 1)]
    return list(pts[:k]) + arc + list(pts[k + 1:])


def tface(f, mat, u0, u1, z0, z1, w, tu, tv, v0=0.0, uo=0.0):
    """Outward patch, texture every tu along (from u offset uo) and tv up.
    / 繰り返し模様の外向きの面（uo: 模様の起点）。"""
    if u1 - u0 < 1e-3 or z1 - z0 < 1e-3:
        return
    a, b = (u0 + uo) / tu, (u1 + uo) / tu
    f.m.quad(mat, f.P(u0, w, z0), f.P(u1, w, z0), f.P(u1, w, z1), f.P(u0, w, z1),
             (a, (z0 - v0) / tv), (b, (z0 - v0) / tv), (b, (z1 - v0) / tv), (a, (z1 - v0) / tv))


def quad_up(m, mat, a, b, c, d, uvs=1.0):
    """A horizontal quad wound to face up. / 上向きの水平四角形。"""
    P = [np.asarray(q, float) for q in (a, b, c, d)]
    if np.cross(P[1] - P[0], P[3] - P[0])[2] < 0:
        P = P[::-1]
    m.quad(mat, *P, *[(q[0] / uvs, q[1] / uvs) for q in P])


def vpoly(m, mat, P3, normal, uvs=4.0):
    """A vertical polygon (3-D points in one vertical plane) facing ``normal``.
    / 鉛直面内の多角形（normal の向き）。"""
    P3 = np.asarray(P3, float)
    nh = np.asarray(normal, float)[:2]
    nh = nh / np.linalg.norm(nh)
    s_ax = np.array([-nh[1], nh[0]])
    V2 = np.column_stack([P3[:, :2] @ s_ax, P3[:, 2]])
    if Polygon(V2).area < 1e-6:
        return
    I = earcut.triangulate_float64(V2.astype(np.float64), np.array([len(V2)], np.uint32)).reshape(-1, 3)
    if len(I) == 0:
        return
    T = P3[I]
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    flip = (nrm[:, :2] @ nh) < 0
    I = np.where(flip[:, None], I[:, ::-1], I)
    m.tris(mat, P3, np.r_[nh, 0.0], V2 / uvs, I)


def pyramid(m, mat, ring, z0, apex):
    """A pyramid over a CCW ring from z0 to an apex (x, y, z), faces up and
    out. / 角錐（上・外向き）。"""
    c = list(ring)
    for a, b in zip(c, c[1:] + c[:1]):
        P = [(a[0], a[1], z0), (b[0], b[1], z0), tuple(apex)]
        n = np.cross(np.subtract(P[1], P[0]), np.subtract(P[2], P[0]))
        if n[2] < 0:
            P = P[::-1]
            n = -n
        m.tris(mat, P, n / np.linalg.norm(n), [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])


def band(m, foot, lines, w, z0, z1, mats, bottom="wsm_soffit", top=None, uvs=3.0, taken=None):
    """A band standing w out of the footprint along ``lines`` (pieces of its
    ring), z0..z1: outer walls in z bands ``mats`` [(za, zb, mat)], the top up,
    the bottom down; corners mitred exactly (one polygon). / 外形から張り出す帯。"""
    if lines is None or lines.is_empty:
        return
    ring = foot.buffer(w, join_style="mitre").difference(foot)
    g = ring.intersection(lines.buffer(w + 0.05, cap_style="flat", join_style="mitre"))
    if taken:
        # 2 cm off the band already drawn: end caps never back to back / 既存の帯から 2 cm 離す
        g = g.difference(unary_union(taken).buffer(0.02))
    edge = foot.exterior.buffer(0.02)
    for p in polys(g):
        if p.area < 0.05:
            continue
        p = orient(p, 1.0)
        if taken is not None:
            taken.append(p)
        for za, zb, mat in mats:
            m.ring_walls(mat, p.exterior, za, zb, True, uvs,
                         skip=lambda a, b: edge.contains(Point((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)))
        m.flat(top or mats[-1][2], p, z1, uvs)
        m.flat(bottom, p, z0, uvs, up=False)


def slab(m, mat, poly, z0, z1, under=None, uvs=2.0):
    """A thin slab (canopy, fin): walls out, top up, underside down. / 薄い庇・フィン。"""
    for p in polys(poly):
        if p.area < 0.05:
            continue
        m.prism(mat, p, z0, z1, top=True, bottom=False, uv_scale=uvs)
        m.flat(under or mat, p, z0, uvs, up=False)


class WSM(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        self.X = X = _xform(T, self.meta)
        ox, oy, a = T["frames"]["S"]
        self.O = np.array(X((ox, oy)))
        a = math.radians(a)
        self.eu, self.ev = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        raw = orient(Polygon([X(q) for q in T["footprint"]]), 1.0)
        self.raw = raw
        # roads win: the network, islands and the drives near the site (check_roads
        # --site counts them) / 道路優先: 道路網・島・敷地付近の車路
        near = raw.buffer(20.0)
        self.no_go = unary_union([L.road_all.intersection(near), L.islands.intersection(near),
                                  L.driveways.intersection(near)])
        self.drives = L.driveways.intersection(raw.buffer(8.0))
        foot, self.moves = fit_edges(raw, self.no_go, GAP)
        if self.no_go.distance(foot) < GAP - 1e-3:
            foot = orient(max(polys(foot.difference(self.no_go.buffer(GAP))), key=lambda g: g.area), 1.0)
        pts = clean_ring(list(foot.exterior.coords)[:-1])
        # the rounded SE corner, then the shell's start on the south face
        # 南東の角を丸め、シェルの始点を南面に入れる
        sh = T["shell"]
        k = self.nearest_vertex(pts, X(sh["corner"]))
        pts = fillet(pts, k, sh["fillet_r"])
        pts = self.insert_at(pts, self.S2xy(sh["u0"], 0.0))
        # the shell turns the E corner and ends end_along metres up the NE face
        # シェルは東角を回り北東面を end_along m 進んで終わる
        pts = orient(Polygon(pts), 1.0).exterior.coords[:-1]
        ke = self.nearest_vertex(pts, X(sh["end"]))
        E, Nx = np.array(pts[ke]), np.array(pts[(ke + 1) % len(pts)])
        e_end = E + (Nx - E) / np.linalg.norm(Nx - E) * min(sh.get("end_along", 0.0), np.linalg.norm(Nx - E) - 0.5)
        pts = self.insert_at(list(pts), tuple(e_end))
        self.foot = orient(Polygon(pts), 1.0)
        self.ring = [np.array(q) for q in list(self.foot.exterior.coords)[:-1]]
        self.footprint = self.foot
        i0 = self.nearest_vertex(self.ring, self.S2xy(sh["u0"], 0.0))
        i1 = self.nearest_vertex(self.ring, tuple(e_end))
        self.shell_idx = self.walk(i0, i1)
        self.shell_line = LineString([self.ring[i] for i in self.shell_idx])
        rc = T["recess"]
        r0 = i1                     # the set-back starts where the shell ends / セットバックはシェルの終点から
        r1 = self.nearest_vertex(self.ring, X(rc["end"]))
        self.recess_line = LineString([self.ring[i] for i in self.walk(r0, r1)])
        strip = self.recess_line.buffer(rc["depth"], cap_style="flat", join_style="mitre").intersection(self.foot)
        g = self.foot.difference(strip)
        self.ground_floor = orient(max(polys(g), key=lambda q: q.area), 1.0)
        # blocks: rectangles of frame S clipped to the footprint, in order
        # ブロック: S 座標の矩形を外形で切る
        taken = Polygon()
        self.blocks = {}
        oc = T["octagon"]
        self.oct_c = np.array(X(oc["c"]))
        self.oct_out = self.octagon(oc["r_out"])
        self.oct_in = self.octagon(oc["r_in"])
        self.blocks["atrium"] = dict(poly=self.oct_out, top=oc["z_rim"], roof_z=oc["z_rim"], kind="atrium")
        taken = self.oct_out
        for k2, b in T["blocks"].items():
            p = orient(Polygon([X(q) for q in b["pts"]]), 1.0).intersection(self.foot).difference(taken)
            p = [q for q in polys(shapely.set_precision(p, 0.001)) if q.area > 1.0]
            if not p:
                continue
            p = orient(max(p, key=lambda q: q.area), 1.0)
            taken = taken.union(p)
            self.blocks[k2] = dict(poly=p, top=b["top"], roof_z=b["top"] - PAR, kind=b["kind"])
        for b in self.blocks.values():
            shapely.prepare(b["poly"])
        self.shell_rows()
        self.portal = self.find_portal()
        self.site = self.foot.buffer(6.0, join_style="mitre")
        self.footpaths = L.footpaths.intersection(raw.buffer(40.0))
        self.canopies = self.make_canopies()
        self.link = self.make_linkway()
        if self.link is not None:
            # generic covered walks under ours are dropped (build.py items_clear) / 汎用の屋根付き通路を除く
            self.items_clear = self.link["roof"].buffer(3.0)
            self.items_kinds = ("linkway",)
        self.centre = self.foot.centroid
        self._parts = None
        mv = ["%.2f" % v for v in self.moves if v > 0]
        print("wsm: footprint %.0f m2 (OSM %.0f), edges moved in for roads: %s"
              % (self.foot.area, raw.area, ", ".join(mv) or "none"))

    # -- frame S and the ring / S 座標と外形
    def S2xy(self, u, v):
        return tuple(self.O + self.eu * u + self.ev * v)

    def xy2S(self, p):
        d = np.asarray(p, float)[:2] - self.O
        return float(d @ self.eu), float(d @ self.ev)

    @staticmethod
    def nearest_vertex(pts, q):
        return int(np.argmin([math.dist(p, q) for p in pts]))

    @staticmethod
    def insert_at(pts, q):
        """Insert q into the ring edge nearest to it (if not on a vertex). / 最寄りの辺に点を挿入。"""
        n = len(pts)
        best = min(range(n), key=lambda i: LineString([pts[i], pts[(i + 1) % n]]).distance(Point(q)))
        if min(math.dist(pts[best], q), math.dist(pts[(best + 1) % n], q)) < 0.3:
            return list(pts)
        # projected onto that edge: a kink would mitre the shell far away / 辺上に投影（折れは遠くで留まる）
        e = LineString([pts[best], pts[(best + 1) % n]])
        q = e.interpolate(e.project(Point(q))).coords[0]
        return list(pts[:best + 1]) + [tuple(q)] + list(pts[best + 1:])

    def walk(self, i0, i1):
        n = len(self.ring)
        out = [i0]
        while out[-1] != i1:
            out.append((out[-1] + 1) % n)
        return out

    def octagon(self, r):
        """Regular octagon, flats along frame S, across-flats 2 r. / 八角形（S 座標に辺が平行）。"""
        R = r / math.cos(math.pi / 8)
        out = []
        for k in range(8):
            a = math.pi / 8 + k * math.pi / 4
            out.append(tuple(self.oct_c + R * (self.eu * math.cos(a) + self.ev * math.sin(a))))
        return orient(Polygon(out), 1.0)

    def h_at(self, p):
        """(top, roof surface, key) of the block at p; ground (0, 0, None) outside.
        / 点 p のブロックの高さ（外は地面）。"""
        for k, b in self.blocks.items():
            if b["poly"].contains(Point(p)):
                return b["top"], b["roof_z"], k
        return 0.0, 0.0, None

    def s_along(self, f, kind, s):
        """Local u on facade f of frame-S coordinate s along the face's axis
        (u for S / N / NE faces, v for W). / S 座標から外壁の u へ。"""
        e = self.ev if kind == "W" else self.eu
        te = float(f.t @ e)
        if abs(te) < 1e-6:
            return None
        return (s - float((f.a - self.O) @ e)) / te

    def kind_of(self, n, mid=None):
        """S, W (the west end only), N, NW (the nose), NE, E by the outward
        normal. / 外向き法線による面の種類。"""
        cu, cv = float(n @ self.eu), float(n @ self.ev)
        if cv < -0.9:
            return "S"
        if cu < -0.9 and (mid is None or self.xy2S(mid)[0] < 10.0):
            return "W"
        if cv > 0.95:
            return "N"
        if cv > 0:
            return "NE" if cu > 0 else "NW"
        return "E" if cu > 0 else "W"

    # -- shell / 曲面シェル
    def offset_section(self, d):
        """The shell's base line offset d inward; edges outside it stay: mitred
        on the next face, a perpendicular step where it starts mid-face.
        / シェル基線を内側へ d ずらす（隣の面で留める）。"""
        R, n = self.ring, len(self.ring)
        idx = self.shell_idx
        S = set(zip(idx, idx[1:]))

        def line(i):
            a, b = R[i], R[(i + 1) % n]
            t = (b - a) / np.linalg.norm(b - a)
            return a + left(t) * (d if (i, (i + 1) % n) in S else 0.0), t

        out = []
        for i in idx:
            (a0, t0), (a1, t1) = line((i - 1) % n), line(i)
            cr = t0[0] * t1[1] - t0[1] * t1[0]
            if abs(cr) < 0.2 and i in (idx[0], idx[-1]):
                # starts / ends in mid-face: a perpendicular step (on the face, then offset)
                # 面の途中で始まる・終わる: 直角の段（面上の点と内側の点）
                if i == idx[0]:
                    out += [a0 + t0 * float((R[i] - a0) @ t0), a1]
                else:
                    out += [a0 + t0 * float((R[i] - a0) @ t0), a1 + t1 * float((R[i] - a1) @ t1)]
            elif (i, (i + 1) % n) in S and ((i - 1) % n, i) in S:
                # both edges offset d: the mitre along the bisector (stable at small turns)
                # 両辺とも d: 二等分線上の留め（小さな折れでも安定）
                bis = left(t0) + left(t1)
                bis = bis / np.linalg.norm(bis)
                out.append(R[i] + bis * d / max(float(bis @ left(t1)), 0.2))
            elif abs(cr) < 0.2:
                out.append(a1)
            else:
                dd = a1 - a0
                out.append(a0 + t0 * (dd[0] * t1[1] - dd[1] * t1[0]) / cr)
        return out

    def shell_rows(self):
        """Base and inner rows of the shell strip and its arc lengths; the east
        roof loses the strip. / シェル帯の外側・内側の列と弧長（東の屋根から帯を除く）。"""
        sh = self.T["shell"]
        self.B = self.offset_section(0.0)
        self.I = self.offset_section(sh["depth"])
        s = [0.0, 0.0]
        for a, b in zip(self.B[1:-2], self.B[2:-1]):
            s.append(s[-1] + float(np.linalg.norm(b - a)))
        s.append(s[-1])
        self.S_arc = s
        self.S_tot = s[-1]
        zone = Polygon([tuple(p) for p in self.B] + [tuple(p) for p in self.I[::-1]]).buffer(0)
        e = self.blocks["east"]
        roof = e["poly"].difference(zone.buffer(0.01))
        e["roof_poly"] = orient(max(polys(roof), key=lambda q: q.area), 1.0)

    def z_arch(self, s):
        """Wall top of the shell at arc length s: a half ellipse z_lo..z_crown.
        / シェル壁頂の高さ（半楕円）。"""
        sh = self.T["shell"]
        x = (s - self.S_tot / 2) / (self.S_tot / 2)
        return sh["z_lo"] + (sh["z_crown"] - sh["z_lo"]) * math.sqrt(max(0.0, 1.0 - x * x))

    def shell(self, m):
        """The curved strip behind the arched wall top (light ribbed metal, up),
        its inner edge against the 23 m roof, white end steps.
        / アーチ状の壁頂の奥の曲面帯・屋根との段・両端の白い段。"""
        sh = self.T["shell"]
        top = self.blocks["east"]["top"]
        B, I, S = self.B, self.I, self.S_arc
        for j in range(1, len(B) - 2):
            Lj = S[j + 1] - S[j]
            ns = max(1, int(math.ceil(Lj / 1.5)))
            for k in range(ns):
                t0, t1 = k / ns, (k + 1) / ns
                b0, b1 = B[j] + (B[j + 1] - B[j]) * t0, B[j] + (B[j + 1] - B[j]) * t1
                i0, i1 = I[j] + (I[j + 1] - I[j]) * t0, I[j] + (I[j + 1] - I[j]) * t1
                s0, s1 = S[j] + Lj * t0, S[j] + Lj * t1
                z0, z1 = self.z_arch(s0), self.z_arch(s1)
                quad_up(m, "wsm_metal_top", (*b0, z0), (*b1, z1), (*i1, z1), (*i0, z0), 2.0)
                # the step to the flat roof: faces the roof where the strip is higher,
                # the strip where it is lower / 屋根との段（高い側から低い側へ向く）
                tdir = (i1 - i0) / max(np.linalg.norm(i1 - i0), 1e-9)
                for (pa, za), (pb, zb) in self.split_at(i0, z0, i1, z1, top):
                    want = left(tdir) if (za + zb) / 2 > top else -left(tdir)
                    q = [(*pa, top), (*pb, top), (*pb, zb), (*pa, za)]
                    nq = np.cross(np.subtract(q[1], q[0]), np.subtract(q[3], q[0]))
                    if np.linalg.norm(nq) < 1e-9:
                        nq = np.cross(np.subtract(q[2], q[1]), np.subtract(q[3], q[1]))
                    if float(nq[:2] @ want) < 0:
                        q = q[::-1]
                    m.quad(WHITE, *q)
        # white end steps where the strip starts / ends below the roof / 両端の段
        for jb, ji, jn in ((1, 1, 2), (-2, -2, -3)):
            za = self.z_arch(S[jb])
            if za >= top - 0.05:
                continue
            along = B[jn] - B[jb]
            along = along / np.linalg.norm(along)
            vpoly(m, WHITE, [(*B[jb], za), (*I[ji], za), (*I[ji], top), (*B[jb], top)], along)

    @staticmethod
    def split_at(a, za, b, zb, h):
        """[(a, za), (b, zb)] split where z crosses h. / 高さ h を横切る所で分ける。"""
        if (za - h) * (zb - h) >= 0:
            return [((a, za), (b, zb))]
        t = (h - za) / (zb - za)
        c = a + (b - a) * t
        return [((a, za), (c, h)), ((c, h), (b, zb))]

    # -- the car park portal / 駐車場の出入口
    def find_portal(self):
        """Where the OSM car-park drive meets the NE face (else the trace's u).
        / 駐車場の車路が北東面に当たる所。"""
        ne = [ln for ln in self.ring_pieces(self.recess_line) if self.kind_of(ln[2], (ln[0] + ln[1]) / 2) == "NE"]
        ne_line = unary_union([LineString([a, b]) for a, b, n in ne])
        if not self.drives.is_empty and not ne_line.is_empty:
            p, q = nearest_points(ne_line, self.drives)
            if p.distance(q) < 4.0:
                return np.array([p.x, p.y])
        u = self.T.get("carpark_u", 80.0)
        best = None
        for a, b, n in ne:
            for t in np.linspace(0, 1, 41):
                q = a + (b - a) * t
                du = abs(self.xy2S(q)[0] - u)
                if best is None or du < best[0]:
                    best = (du, q)
        return best[1] if best else None

    @staticmethod
    def ring_pieces(line):
        c = [np.array(q) for q in line.coords]
        out = []
        for a, b in zip(c, c[1:]):
            L_ = np.linalg.norm(b - a)
            if L_ > 1e-3:
                t = (b - a) / L_
                out.append((a, b, np.array([t[1], -t[0]])))
        return out

    # -- layout / 配置
    def inner(self):
        return self.foot.buffer(0.2, join_style="mitre")

    def keepout(self):
        """Where the generic trees and furniture must not go. / 汎用の木と小物を置かない所。"""
        return unary_union([self.foot.buffer(3.0)] + [c["poly"].buffer(1.0) for c in self.canopies]
                           + ([self.link["roof"].buffer(0.8)] if self.link is not None else []))

    def make_linkway(self):
        """The covered linkway along the E footpath: its roof (off the road by
        0.6 m and the walls by 1.5 m) and post spots on the path's mall side,
        never on the path or the road. / 東の歩道沿いの屋根付き通路（屋根と柱の位置）。"""
        lk = self.T.get("linkway")
        if not lk:
            return None
        a, b = np.array(self.X(lk["a"])), np.array(self.X(lk["b"]))
        line = LineString([a, b])
        roof = line.buffer(lk["w"] / 2, cap_style="flat").difference(self.L.road_all.buffer(0.6)) \
            .difference(self.foot.buffer(1.5)).difference(self.no_go.buffer(0.6))
        roof = [q for q in polys(roof) if q.area > 2.0]
        if not roof:
            return None
        roof = orient(max(roof, key=lambda q: q.area), 1.0)
        t = (b - a) / np.linalg.norm(b - a)
        side = left(t)
        if self.foot.distance(Point(a + side * 3)) > self.foot.distance(Point(a - side * 3)):
            side = -side                          # posts on the mall side / 柱はモール側
        posts = []
        for s_ in np.arange(1.0, line.length - 0.5, lk["every"]):
            p = a + t * s_ + side * lk["post_off"]
            P = Point(p)
            if roof.buffer(0.3).contains(P) and self.footpaths.distance(P) > 0.1 \
                    and self.no_go.distance(P) > 0.8:
                posts.append(p)
        return dict(lk, roof=roof, posts=posts, t=t, side=side)

    def linkway(self, m):
        """Flat white-grey steel roof, fascia and gutter, square grey posts with
        brackets (sv04). / 白灰色の平屋根・鼻隠し・樋、角柱と方杖（sv04）。"""
        lk = self.link
        if lk is None:
            return
        z = lk["z"]
        slab(m, "wsm_linkroof", lk["roof"], z, z + 0.18, under="wsm_soffit")
        for p in lk["posts"]:
            beam(m, "metal_grey", (p[0], p[1], Z0), (p[0], p[1], z), r=0.08)
            for sg in (1, -1):
                q = p - lk["side"] * sg * 0.9
                if lk["roof"].contains(Point(q)):
                    beam(m, "metal_grey", (p[0], p[1], z - 0.7), (q[0], q[1], z - 0.02), r=0.04)
        # gutter along the mall-side edge / モール側の樋
        ext = lk["roof"].exterior
        a = np.array(self.X(lk["a"])) + lk["side"] * lk["w"] / 2
        e = LineString([a, a + lk["t"] * 200]).intersection(lk["roof"].buffer(0.02))
        for g in getattr(e, "geoms", [e]):
            if g.geom_type == "LineString" and g.length > 1.0:
                c = [np.array(q) for q in g.coords]
                beam(m, "metal_grey", (*(c[0] + lk["side"] * 0.08), z - 0.05), (*(c[-1] + lk["side"] * 0.08), z - 0.05),
                     r=0.07)
        del ext

    def make_canopies(self):
        out = []
        for c in self.T["canopies"]:
            a, b = np.array(self.X(c["a"])), np.array(self.X(c["b"]))
            t = (b - a) / np.linalg.norm(b - a)
            n = np.array([t[1], -t[0]])
            # outward = away from the footprint / 外形の外側へ
            mid = (a + b) / 2
            if self.foot.contains(Point(mid + n * 0.5)):
                n = -n
            p = Polygon([a, b, b + n * c["depth"], a + n * c["depth"]]).difference(self.foot.buffer(0.02))
            p = p.difference(self.no_go.buffer(0.4))
            p = [q for q in polys(p) if q.area > 1.0]
            if p:
                out.append(dict(c, poly=orient(max(p, key=lambda q: q.area), 1.0), a=a, b=b, n=n))
        return out

    # -- ground / 地面
    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        under = self.foot.difference(self.ground_floor).difference(self.no_go.buffer(0.05))
        m.flat("wsm_tiles", under.intersection(tile), z + 0.01, 3.0)
        return m

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
        guard = self.no_go.buffer(0.05)
        c = (self.centre.x, self.centre.y)

        def add(fn, *a):
            pm = Mesh()
            fn(pm, *a)
            nd = drop_over(pm, guard)
            if nd:
                self.drop_log.append((fn.__name__, nd))
            parts.append((c, pm))

        for fn in (self.walls, self.roofs, self.shell, self.ground_storey, self.street_bands, self.canopy_parts,
                   self.features, self.roof_plant, self.linkway):
            add(fn)
        for nm, n in self.drop_log:
            print("wsm: %s: %d triangles over the road network dropped" % (nm, n))
        self.tris = sum(pm.ntris() for _, pm in parts)
        print("wsm: %d blocks, %d triangles" % (len(self.blocks), self.tris))
        return parts

    # -- walls where the height steps / 高さの変わる所の壁
    def segments(self):
        lines = unary_union([b["poly"].exterior for b in self.blocks.values()]
                            + [r for b in self.blocks.values() for r in b["poly"].interiors]
                            + [self.foot.exterior])
        for ln in getattr(lines, "geoms", [lines]):
            c = [np.array(q) for q in ln.coords]
            for a, b in zip(c, c[1:]):
                if np.linalg.norm(b - a) > 0.02:
                    yield a, b

    def walls(self, m):
        on_shell = self.shell_line.buffer(0.03)
        on_rec = self.recess_line.buffer(0.03)
        for a, b in self.segments():
            t = (b - a) / np.linalg.norm(b - a)
            n = np.array([t[1], -t[0]])
            mid = (a + b) / 2
            hr, hl = self.h_at(mid + n * 0.15), self.h_at(mid - n * 0.15)
            if abs(hr[0] - hl[0]) < 0.05:
                continue
            if hr[0] < hl[0]:
                f, low, high = facade(m, a, b), hr, hl
            else:
                f, low, high = facade(m, b, a), hl, hr
            top = high[0]
            if low[2] is not None:
                # a step between roofs: plain render / 屋上の段差: 白い壁
                tiled_face(f, WHITE, 0.0, f.L, low[1], top, 0.0, 4.4, 4.4)
                continue
            P = Point(mid)
            kind = self.kind_of(f.n, mid)
            if on_shell.contains(P):
                self.shell_wall(f, top)
            elif on_rec.contains(P):
                self.upper_wall(f, BAND[0], top, kind, recessed=True)
            else:
                self.street_wall(f, top, kind)

    def opens_for(self, f, kind):
        """Openings (u0, u1, z0, z1, glass, depth, tu) of the trace on this piece.
        / この壁片に掛かる開口。"""
        T = self.T
        out = []
        specs = []
        if kind == "S":
            g = T["glazed_bay_s"]
            specs.append((g["u0"], g["u1"], g["z0"], g["z1"], "wsm_bay", 0.3, None))
        if kind == "NE":
            r = T["ribbon_ne"]
            specs.append((r["u0"], r["u1"], r["z0"], r["z1"], "wsm_ribbon", 0.35, 1.5))
        for s0, s1, z0, z1, glass, dep, tu in specs:
            u0, u1 = self.s_along(f, kind, s0), self.s_along(f, kind, s1)
            if u0 is None:
                continue
            u0, u1 = sorted((u0, u1))
            u0, u1 = max(u0, 0.6), min(u1, f.L - 0.6)
            if u1 - u0 > 1.0:
                out.append((u0, u1, z0, z1, glass, dep, tu))
        return out

    def upper_wall(self, f, z0, top, kind, recessed=False):
        """The wall from z0 up: white render with recessed lines, the W face's
        wave band, openings. / z0 から上の壁（白い塗装・波の帯・開口）。"""
        zb = BAND[2] if kind in ("S", "W", "N") else BAND[1]
        if recessed and kind != "N":
            zb = BAND[1]
        if top <= zb:
            return                                  # (all behind the band) / 帯の裏のみ
        ops = self.opens_for(f, kind)
        if kind == "W":
            w = self.T["wave_w"]
            tiled_face(f, WHITE, 0.0, f.L, zb, w["z0"], 0.0, 4.4, 4.4)
            # the wave runs on from piece to piece (u from the face's v) / 片をまたいで波を続ける
            tface(f, "wsm_wave", 0.0, f.L, w["z0"], w["z1"], 0.0, 27.0, w["z1"] - w["z0"], v0=w["z0"],
                  uo=-self.xy2S(f.a)[1])
            zb = w["z1"]
        rows = [(u0, u1, oz0, oz1) for u0, u1, oz0, oz1, *_ in ops]
        self.wall_patch(f, zb, top, rows)
        for u0, u1, oz0, oz1, glass, dep, tu in ops:
            opening(f, u0, u1, oz0, oz1, WHITE, glass, depth=dep, tu=tu)

    @staticmethod
    def wall_patch(f, z0, z1, opens):
        """White render patch with holes; texture every 4.4 m. / 開口付きの白壁。"""
        us = sorted({0.0, f.L} | {min(max(v, 0.0), f.L) for a, b, _, _ in opens for v in (a, b)})
        for x0, x1 in zip(us, us[1:]):
            if x1 - x0 < 1e-4:
                continue
            mid = (x0 + x1) / 2
            z = z0
            for oz0, oz1 in sorted((max(oz0, z0), min(oz1, z1)) for a, b, oz0, oz1 in opens if a <= mid <= b):
                if oz0 > z:
                    tiled_face(f, WHITE, x0, x1, z, oz0, 0.0, 4.4, 4.4)
                z = max(z, oz1)
            if z1 > z:
                tiled_face(f, WHITE, x0, x1, z, z1, 0.0, 4.4, 4.4)

    def street_wall(self, f, top, kind):
        """A face on the street: shopfronts set 0.35 m in (solid ends), the
        bulkhead, then the upper wall (the band is drawn by street_bands).
        / 道路側の壁: 店舗ガラス（0.35 m 奥）・鼻隠し・上の壁。"""
        d = 0.35
        # the ground storey only where the ground floor reaches this face (the
        # NW corner is an open arcade) / 1 階は床がこの面まで来る所だけ（北西角は開放）
        cov = LineString([f.P(0, 0, 0)[:2], f.P(f.L, 0, 0)[:2]]).intersection(self.ground_floor.buffer(0.05))
        us = [float((np.array(q) - f.a) @ f.t) for g in getattr(cov, "geoms", [cov]) if not g.is_empty
              for q in g.coords] if not cov.is_empty else []
        if us:
            ua, ub = max(0.0, min(us)), min(f.L, max(us))
            L_ = ub - ua
            e = min(0.4, L_ / 4)
            if L_ > 2.0:
                tface(f, "wsm_shop", ua + e, ub - e, 0.0, SHOP_Z, -d, 9.0, SHOP_Z)
                self.mullions(f, ua + e, ub - e, SHOP_Z, -d)
                f.hplane(WHITE, ua + e, ub - e, -d, 0.0, SHOP_Z, up=False)
                f.side(WHITE, ua + e, -d, 0.0, 0.0, SHOP_Z, 1)
                f.side(WHITE, ub - e, -d, 0.0, 0.0, SHOP_Z, -1)
                tiled_face(f, WHITE, ua, ua + e, 0.0, SHOP_Z, 0.0, 4.4, 4.4)
                tiled_face(f, WHITE, ub - e, ub, 0.0, SHOP_Z, 0.0, 4.4, 4.4)
            else:
                tiled_face(f, WHITE, ua, ub, 0.0, SHOP_Z, 0.0, 4.4, 4.4)
            tface(f, "wsm_teal" if kind == "W" else "wsm_grey", ua, ub, SHOP_Z, BAND[0], 0.0, 3.0, 1.3, v0=SHOP_Z)
        self.upper_wall(f, BAND[0], top, kind)

    def shell_wall(self, f, top):
        """The shell's wall: red-fascia shops, the band (street_bands), then the
        two-tone metal up to the arched top, the dark lower half and the light
        upper half meeting in a wave. / シェルの壁: 店舗・二色のメタル（アーチ状の壁頂まで）。"""
        L_, d = f.L, 0.35
        z0 = BAND[1]
        if L_ > 1.5:
            tiled_face(f, "wsm_shop_r", 0.0, L_, 0.0, SHOP_Z + 0.4, -d, 9.0, SHOP_Z + 0.4)
            f.hplane(WHITE, 0.0, L_, -d, 0.0, SHOP_Z + 0.4, up=False)
            self.mullions(f, 0.0, L_, SHOP_Z + 0.4, -d)
        else:
            tiled_face(f, WHITE, 0.0, L_, 0.0, SHOP_Z + 0.4, 0.0, 4.4, 4.4)
        tiled_face(f, WHITE, 0.0, L_, SHOP_Z + 0.4, z0, 0.0, 4.4, 4.4)
        s0 = self.shell_line.project(Point(f.a))
        n = max(1, int(math.ceil(L_ / 1.5)))
        for k in range(n):
            u0, u1 = L_ * k / n, L_ * (k + 1) / n
            za, zb = self.z_arch(s0 + u0), self.z_arch(s0 + u1)
            f.m.quad("wsm_metal", f.P(u0, 0, z0), f.P(u1, 0, z0), f.P(u1, 0, zb), f.P(u0, 0, za),
                     ((s0 + u0) / 40, 0.0), ((s0 + u1) / 40, 0.0), ((s0 + u1) / 40, 1.0), ((s0 + u0) / 40, 1.0))
            # a thin coping along the arched top / アーチ頂部の笠木
            f.m.quad("wsm_coping", f.P(u0, 0.12, za), f.P(u1, 0.12, zb), f.P(u1, 0.0, zb + 0.12),
                     f.P(u0, 0.0, za + 0.12))
            f.m.quad("wsm_coping", f.P(u0, 0.0, za - 0.25), f.P(u1, 0.0, zb - 0.25), f.P(u1, 0.12, zb),
                     f.P(u0, 0.12, za))

    # -- the set-back ground storey / セットバックした 1 階
    def ground_storey(self, m):
        rc = self.T["recess"]
        zr = rc["z"]
        edge = self.foot.exterior.buffer(0.03)
        g = self.ground_floor
        c = [np.array(q) for q in g.exterior.coords]
        port = None
        if self.portal is not None:
            port = Point(self.portal)
        for a, b in zip(c, c[1:]):
            if np.linalg.norm(b - a) < 0.05 or edge.contains(Point((a + b) / 2)):
                continue
            f = facade(m, a, b)
            cut = None
            if port is not None and LineString([a, b]).distance(port) < rc["depth"] + 1.0:
                u = float((np.array(nearest_points(LineString([a, b]), port)[0].coords[0]) - f.a) @ f.t)
                cut = (max(0.3, u - 4.5), min(f.L - 0.3, u + 4.5))
                if cut[1] - cut[0] < 4.0:
                    cut = None
            if cut is None:
                tiled_face(f, "wsm_shop", 0.0, f.L, 0.0, zr, 0.0, 9.0, zr)
                self.mullions(f, 0.0, f.L, zr, 0.0)
                continue
            tiled_face(f, "wsm_shop", 0.0, cut[0], 0.0, zr, 0.0, 9.0, zr)
            tiled_face(f, "wsm_shop", cut[1], f.L, 0.0, zr, 0.0, 9.0, zr)
            self.portal_box(f, cut[0], cut[1], zr)
        # soffit over the set-back, columns at the face / 軒天と柱
        m.flat("wsm_soffit", self.foot.difference(g), zr, 3.0, up=False)
        L_ = self.recess_line.length
        for s in np.arange(rc["col_every"] / 2, L_, rc["col_every"]):
            p = np.array(self.recess_line.interpolate(s).coords[0])
            q = np.array(self.recess_line.interpolate(min(L_, s + 0.5)).coords[0]) - p
            if np.linalg.norm(q) < 1e-6:
                continue
            t = q / np.linalg.norm(q)
            cpt = p + left(t) * (rc["col_r"] + 0.35)
            if port is not None and port.distance(Point(cpt)) < 6.5:
                continue
            if not g.buffer(-0.2).disjoint(Point(cpt).buffer(rc["col_r"])):
                continue
            m.cylinder("wsm_col", cpt[0], cpt[1], 0.0, zr, rc["col_r"], seg=10, top=False)

    @staticmethod
    def mullions(f, u0, u1, z1, w, every=3.0):
        """Slim dark mullions proud of the shop glass and a transom at 3 m.
        / 店舗ガラスの方立と無目（ガラスより手前）。"""
        if u1 - u0 < 2.0:
            return
        n = max(1, int(round((u1 - u0) / every)))
        for k in range(1, n):
            u = u0 + (u1 - u0) * k / n
            f.block("wsm_mull", u - 0.05, u + 0.05, w, w + 0.1, 0.0, z1, top=False, bottom=False)
        f.block("wsm_mull", u0, u1, w, w + 0.08, 2.9, 3.02)

    def portal_box(self, f, u0, u1, zr):
        """The car-park portal: a dark opening 5 m deep, the blue fascia with
        "In / Carpark / Out". / 駐車場出入口（暗い開口と青い表示板）。"""
        h, dep = 4.4, 5.0
        f.face("wsm_dark", u0, u1, 0.0, h, -dep)
        f.side("wsm_dark", u0, -dep, 0.0, 0.0, h, 1)
        f.side("wsm_dark", u1, -dep, 0.0, 0.0, h, -1)
        f.hplane("wsm_dark", u0, u1, -dep, 0.0, h, up=False)
        f.hplane("wsm_dark", u0, u1, -dep, 0.0, Z0 + 0.01, up=True)
        f.face("wsm_fascia", u0, u1, h, zr, 0.0, 2.0)
        mid = (u0 + u1) / 2
        for txt, du in (("In", -3.2), ("Carpark", 0.0), ("Out", 3.2)):
            g = text_poly(txt, 0.45)
            glyphs_on(f.m, f, g, mid + du, h + 0.45, 0.03, "wsm_letter_w", 0.45)

    # -- bands on the street faces / 道路側の帯
    def street_bands(self, m):
        """The bulging light-blue L2 band with the grey fascia (S, W, N), the
        band alone on the NE and nose, royal blue on the shell; the NE faces'
        perforated metal wave. / 2 階の青い帯・灰色の鼻隠し・北東の有孔メタル。"""
        on_shell = self.shell_line.buffer(0.03)
        S_W, NE, SH = [], [], []
        for a, b in self.ring_pairs():
            mid = Point((a + b) / 2)
            t = (b - a) / np.linalg.norm(b - a)
            kind = self.kind_of(np.array([t[1], -t[0]]), (a + b) / 2)
            ln = LineString([a, b])
            if on_shell.contains(mid):
                SH.append(ln)
            elif kind in ("NE", "E", "NW"):
                NE.append(ln)
            else:
                S_W.append(ln)
        z0, z1, z2 = BAND
        taken = []
        for lines, w, zt in ((S_W, 0.6, z2), (NE, 0.6, z1), (SH, 0.5, z1)):
            for ln in lines:
                a, b = (np.array(q) for q in ln.coords)
                f = facade(m, a, b)
                for u in np.arange(1.2, f.L - 0.6, 2.4):
                    f.block("wsm_mull", u - 0.04, u + 0.04, w, w + 0.06, z0 + 0.05, zt - 0.05, top=False, bottom=False)
        band(m, self.foot, unary_union(S_W), 0.6, z0, z2, [(z0, z1, "wsm_blue"), (z1, z2, "wsm_grey")], taken=taken)
        band(m, self.foot, unary_union(NE), 0.6, z0, z1, [(z0, z1, "wsm_blue")], taken=taken)
        band(m, self.foot, unary_union(SH), 0.5, z0, z1, [(z0, z1, "wsm_blue2")], taken=taken)
        # perforated wave on the NE faces, proud 0.3 m, over the band / 有孔メタルの波
        for ln in NE:
            a, b = (np.array(q) for q in ln.coords)
            f = facade(m, a, b)
            steps = max(1, int(math.ceil(f.L / 1.5)))
            us = np.linspace(0.0, f.L, steps + 1)
            zt = [z1 + 3.6 + 2.6 * math.sin(2 * math.pi * (self.xy2S(f.P(u, 0, 0))[0] - 50.0) / 64.0) for u in us]
            for k in range(steps):
                u0, u1, t0, t1 = us[k], us[k + 1], zt[k], zt[k + 1]
                m.quad("wsm_perf", f.P(u0, 0.3, z1), f.P(u1, 0.3, z1), f.P(u1, 0.3, t1), f.P(u0, 0.3, t0),
                       (u0 / 3, z1 / 3), (u1 / 3, z1 / 3), (u1 / 3, t1 / 3), (u0 / 3, t0 / 3))
                # top edge, sloping with the wave / 波に沿う上端
                m.quad("wsm_perf", f.P(u0, 0.3, t0), f.P(u1, 0.3, t1), f.P(u1, 0.0, t1), f.P(u0, 0.0, t0))
            f.side("wsm_perf", 0.0, 0.0, 0.3, z1, zt[0], -1)
            f.side("wsm_perf", f.L, 0.0, 0.3, z1, zt[-1], 1)

    def ring_pairs(self):
        R = self.ring
        return [(R[i], R[(i + 1) % len(R)]) for i in range(len(R))]

    # -- canopies, fins, columns / 庇・フィン・柱
    def canopy_parts(self, m):
        for c in self.canopies:
            p, z = c["poly"], c["z"]
            if c["kind"] == "tensile":
                slab(m, "wsm_canopy", p, z - 0.25, z, under="wsm_canopy")
                # tensile struts: a tie from the band down to the canopy edge, a slim post
                # 張力式の庇: 帯から庇先端への吊り材と細い柱
                a, b, n = c["a"], c["b"], c["n"]
                L_ = float(np.linalg.norm(b - a))
                t = (b - a) / L_
                for s in np.arange(4.5, L_ - 2.0, 9.0):
                    base = a + t * s
                    tip = base + n * (c["depth"] - 0.3)
                    if not p.buffer(0.05).contains(Point(tip)):
                        continue
                    beam(m, "wsm_col", (base[0], base[1], BAND[1]), (tip[0], tip[1], z), r=0.05)
                    beam(m, "wsm_col", (tip[0], tip[1], Z0), (tip[0], tip[1], z - 0.25), r=0.07)
            else:
                slab(m, "wsm_glass_canopy", p, z - 0.15, z, under="wsm_glass_canopy")
                a, b, n = c["a"], c["b"], c["n"]
                L_ = float(np.linalg.norm(b - a))
                t = (b - a) / L_
                for s in np.arange(2.0, L_ - 1.0, 8.0 if L_ > 10 else max(L_ - 2.0, 1.0)):
                    tip = a + t * s + n * (c["depth"] - 0.3)
                    if p.buffer(0.05).contains(Point(tip)):
                        m.cylinder("wsm_col", tip[0], tip[1], 0.0, z - 0.15, 0.16, seg=8, top=False)
        # fins: thin white slabs cantilevered from the top of a face / フィン
        for fn in self.T["fins"]:
            for a, b in self.ring_pairs():
                t = (b - a) / np.linalg.norm(b - a)
                n = np.array([t[1], -t[0]])
                if self.kind_of(n, (a + b) / 2) != fn["edge"] or np.linalg.norm(b - a) < 3.0:
                    continue
                f = facade(m, a, b)
                u0, u1 = 0.0, f.L
                if "u0" in fn:
                    s0, s1 = self.s_along(f, fn["edge"], fn["u0"]), self.s_along(f, fn["edge"], fn["u1"])
                    if s0 is None:
                        continue
                    u0, u1 = max(0.0, min(s0, s1)), min(f.L, max(s0, s1))
                    if u1 - u0 < 1.0:
                        continue
                q = Polygon([f.P(u0, 0, 0)[:2], f.P(u1, 0, 0)[:2], f.P(u1, fn["depth"], 0)[:2],
                             f.P(u0, fn["depth"], 0)[:2]]).difference(self.foot.buffer(0.02)).difference(
                    self.no_go.buffer(0.4))
                # only in front of blocks that reach the fin's height / フィンの高さに届くブロックの前だけ
                tall = [b["poly"] for b in self.blocks.values() if b["top"] >= fn["z"] - 0.05]
                q = q.intersection(unary_union(tall).buffer(fn["depth"] + 0.1, join_style="mitre")) \
                    if tall else Polygon()
                slab(m, WHITE, q, fn["z"] - 0.35, fn["z"])

        # fins on the upper south walls (frame S lines) / 南面上部のフィン（S 座標の線）
        for fn in self.T.get("fins_s", []):
            q = Polygon([self.S2xy(fn["u0"], fn["v"]), self.S2xy(fn["u1"], fn["v"]),
                         self.S2xy(fn["u1"], fn["v"] - fn["depth"]), self.S2xy(fn["u0"], fn["v"] - fn["depth"])])
            lowr = [b["poly"] for k, b in self.blocks.items() if b["top"] < fn["z"] - 0.5]
            q = q.intersection(unary_union(lowr).buffer(-0.3)) if lowr else Polygon()
            slab(m, WHITE, q.difference(self.no_go.buffer(0.4)), fn["z"] - 0.35, fn["z"])

    # -- signage, panels, openings that are not in the wall pieces / 文字・パネル
    def edge_facade(self, m, kind):
        """Facade on the longest footprint edge of a kind. / その向きの最長の辺。"""
        best = None
        for a, b in self.ring_pairs():
            t = (b - a) / np.linalg.norm(b - a)
            if self.kind_of(np.array([t[1], -t[0]]), (a + b) / 2) == kind:
                if best is None or np.linalg.norm(b - a) > np.linalg.norm(best[1] - best[0]):
                    best = (a, b)
        return facade(m, *best) if best else None

    def features(self, m):
        T = self.T
        # W face: the royal-blue lettering / 西面の青い文字
        f = self.edge_facade(m, "W")
        if f is not None:
            lw = T["lettering_w"]
            u = self.s_along(f, "W", lw["v_mid"])
            g = text_poly(lw["text"], lw["height"])
            glyphs_on(m, f, g, u, lw["z"], 0.05, "wsm_letter_b", lw["height"])
        # S face: the same lettering high at its W end (sv09, OneMap h000) / 南面西端の高所の文字
        f = self.edge_facade(m, "S")
        ls = T.get("lettering_s")
        if f is not None and ls:
            u = self.s_along(f, "S", ls["u_mid"])
            g = text_poly(ls["text"], ls["height"])
            glyphs_on(m, f, g, u, ls["z"], 0.05, "wsm_letter_b", ls["height"])
        # N face: NW entrance fascia with white lettering, LED screen, poster
        # 北面: 北西入口の表示板・LED・ポスター
        f = self.edge_facade(m, "N")
        if f is not None:
            nf = T["nw_fascia"]
            a, b = self.s_along(f, "N", self.xy2S(self.X(nf["a"]))[0]), self.s_along(f, "N", self.xy2S(self.X(nf["b"]))[0])
            u0, u1 = max(0.0, min(a, b)), min(f.L, max(a, b))
            f.block("wsm_fascia", u0, u1, 0.0, 0.75, nf["z0"], nf["z1"])
            g = text_poly(nf["text"], 1.3)
            glyphs_on(m, f, g, (u0 + u1) / 2 + 0.8, (nf["z0"] + nf["z1"]) / 2, 0.78, "wsm_letter_w", 1.3)
            led = nf["led"]
            la, lb = self.s_along(f, "N", led["u0"]), self.s_along(f, "N", led["u1"])
            f.block("wsm_led", max(0.0, min(la, lb)), min(f.L, max(la, lb)), 0.0, 0.25, led["z0"], led["z1"])
            po = T["poster_n"]
            pa, pb = self.s_along(f, "N", po["u0"]), self.s_along(f, "N", po["u1"])
            f.block("wsm_poster", max(0.0, min(pa, pb)), min(f.L, max(pa, pb)), 0.0, 0.12, po["z0"], po["z1"])
        # NW corner column under the entrance canopy / 北西入口の柱
        nw = np.array(self.S2xy(1.2, 53.0))
        if self.foot.contains(Point(nw)):
            m.cylinder("wsm_col", nw[0], nw[1], 0.0, BAND[0], 0.45, seg=12, top=False)
        # NE face: the louvre vent / 北東面のガラリ
        f = self.edge_facade(m, "NE")
        if f is not None:
            v0, v1 = T["ribbon_ne"]["vent_u"]
            a, b = self.s_along(f, "NE", v0), self.s_along(f, "NE", v1)
            if a is not None and 0.5 < min(a, b) and max(a, b) < f.L - 0.5:
                f.block("wsm_louvre", min(a, b), max(a, b), 0.0, 0.06, 13.4, 14.2)

    # -- roofs / 屋根
    def roofs(self, m):
        for k, b in self.blocks.items():
            if k == "atrium":
                ring_out = [tuple(q) for q in list(self.oct_out.exterior.coords)[:-1]]
                m.flat("roof_concrete", self.oct_out.difference(self.oct_in), b["top"], 3.0)
                ap = (self.oct_c[0], self.oct_c[1], self.T["octagon"]["z_apex"])
                pyramid(m, "wsm_sky", [tuple(q) for q in list(self.oct_in.exterior.coords)[:-1]], b["top"], ap)
                del ring_out
                continue
            poly = b.get("roof_poly", b["poly"])
            inner = poly.buffer(-0.25, join_style="mitre")
            for q in polys(inner):
                q = orient(q, 1.0)
                m.ring_walls(WHITE, q.exterior, b["roof_z"], b["top"], outward=False, uv_scale=4.4)
                for r in q.interiors:
                    m.ring_walls(WHITE, r, b["roof_z"], b["top"], outward=True, uv_scale=4.4)
                m.flat("roof_concrete", q, b["roof_z"] + 0.005, 3.0)
            m.flat("wsm_cap", poly.difference(inner), b["top"], 1.0)

    def roof_inner(self, k):
        b = self.blocks[k]
        return b.get("roof_poly", b["poly"]).buffer(-0.8, join_style="mitre")

    def block_at(self, p):
        for k, b in self.blocks.items():
            if k != "atrium" and b["poly"].contains(Point(p)):
                return k
        return None

    def roof_plant(self, m):
        """Solar rows, the screened AC deck, lattice screens, the cream roof
        panel, plant boxes on the W strip, lift rooms. / 屋上設備。"""
        T = self.T
        X = self.X
        for g in T["solar"]:
            poly = Polygon([X(q) for q in g["pts"]])
            k = self.block_at(poly.centroid.coords[0])
            if k is None:
                continue
            area = poly.intersection(self.roof_inner(k))
            z = self.blocks[k]["roof_z"] + 0.3
            a = math.radians(g["dir"])
            d = self.eu * math.cos(a) + self.ev * math.sin(a)     # along the rows / 列の方向
            nrm = np.array([-d[1], d[0]])
            c = np.array(poly.centroid.coords[0])
            for s in np.arange(-40.0, 40.0, 2.3):
                ln = LineString([c + nrm * s - d * 60, c + nrm * s + d * 60]).intersection(area)
                for seg in getattr(ln, "geoms", [ln]):
                    if seg.geom_type != "LineString" or seg.length < 2.0:
                        continue
                    p0, p1 = (np.array(q) for q in seg.coords[:2])
                    if float((p1 - p0) @ d) < 0:
                        p0, p1 = p1, p0
                    lo, hi = -nrm * 0.85, nrm * 0.85
                    m.quad("wsm_solar", (*(p0 + lo), z), (*(p1 + lo), z), (*(p1 + hi), z + 0.3), (*(p0 + hi), z + 0.3),
                           (0, 0), (seg.length / 1.0, 0), (seg.length / 1.0, 1.7), (0, 1.7))
        # the AC deck on the E wing: louvre screen ring + unit rows / 東翼の空調機置場
        pl = T["plant"]
        poly = Polygon([X(q) for q in pl["pts"]]).intersection(self.roof_inner("east"))
        z = self.blocks["east"]["roof_z"]
        for p in polys(poly):
            p = orient(p, 1.0)
            ring = p.difference(p.buffer(-0.15, join_style="mitre"))
            m.prism("wsm_louvre", ring, z, pl["z"] - 0.5, top=True, uv_scale=2.0)
            a = math.radians(61.0)
            d = self.eu * math.cos(a) + self.ev * math.sin(a)
            nrm = np.array([-d[1], d[0]])
            c = np.array(p.centroid.coords[0])
            ins = p.buffer(-1.2)
            for s in np.arange(-16.0, 16.0, 3.4):
                ln = LineString([c + nrm * s - d * 40, c + nrm * s + d * 40]).intersection(ins)
                for seg in getattr(ln, "geoms", [ln]):
                    if seg.geom_type != "LineString" or seg.length < 3.0:
                        continue
                    for q in np.arange(1.2, seg.length - 1.0, 2.4):
                        cc = seg.interpolate(q)
                        m.box("wsm_ac", cc.x, cc.y, z, z + 2.0, 2.0, 1.5, ang=math.atan2(d[1], d[0]),
                              mat_top="wsm_ac_top")
        # lattice screens and the cream panel on the core / 格子スクリーンとクリーム色の屋根板
        for sc in T["screens"] + [dict(T["roof_panel"], cream=True)]:
            p = orient(Polygon([X(q) for q in sc["pts"]]), 1.0)
            k = self.block_at(p.centroid.coords[0])
            if k is None:
                continue
            zc = self.blocks[k]["roof_z"]
            p = p.intersection(self.roof_inner(k))
            for q in polys(p):
                if sc.get("cream"):
                    m.prism("wsm_cream", orient(q, 1.0), zc, zc + sc["h"], top=True, uv_scale=3.0)
                else:
                    ring = q.difference(q.buffer(-0.12, join_style="mitre"))
                    m.prism("wsm_lattice", ring, zc, zc + sc["h"], top=True, uv_scale=2.0)
        # plant boxes on the W strip, lift / stair rooms / 西側の設備箱・塔屋
        rng = np.random.default_rng(7)
        zw = self.blocks["west"]["roof_z"]
        wi = self.roof_inner("west").buffer(-0.8)
        boxes = []
        for _ in range(60):
            u, v = rng.uniform(1.5, 12.5), rng.uniform(8.0, 52.0)
            sx, sy = rng.uniform(1.5, 4.0), rng.uniform(1.5, 5.0)
            p = Polygon([self.S2xy(u - sx / 2, v - sy / 2), self.S2xy(u + sx / 2, v - sy / 2),
                         self.S2xy(u + sx / 2, v + sy / 2), self.S2xy(u - sx / 2, v + sy / 2)])
            if not wi.contains(p) or any(p.buffer(0.6).intersects(b) for b in boxes):
                continue
            boxes.append(p)
            m.prism("wsm_ac" if len(boxes) % 2 else "tank", orient(p, 1.0), zw, zw + rng.uniform(1.2, 2.6),
                    top=True, uv_scale=2.0, mat_top="wsm_ac_top" if len(boxes) % 2 else "tank")
            if len(boxes) >= 14:
                break
        for u, v, k in ((48.5, 50.0, "strip"), (38.0, 40.0, "core"), (57.0, 3.0, "mid")):
            p = Polygon([self.S2xy(u - 1.6, v - 2.0), self.S2xy(u + 1.6, v - 2.0), self.S2xy(u + 1.6, v + 2.0),
                         self.S2xy(u - 1.6, v + 2.0)])
            if self.roof_inner(k).contains(p):
                zz = self.blocks[k]["roof_z"]
                m.prism(WHITE, orient(p, 1.0), zz, zz + 3.0, top=True, uv_scale=4.4, mat_top="roof_concrete")


def write_textures(tex_dir):
    """The mall's drawn textures (procedural; colours from description.md,
    nothing taken from imagery). / モールの描画テクスチャ（画像は使わない）。"""
    import textures as X
    from PIL import Image, ImageDraw
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

    WH = (214, 217, 216)
    # white render, recessed lines every 1.1 m (tile 4.4 m) / 白い塗装と目地
    im = X.paint(512, WH, 0.06)
    d = ImageDraw.Draw(im)
    for k in range(4):
        y = int(k * 128)
        d.rectangle([0, y, 512, y + 3], fill=(168, 172, 172))
        d.rectangle([0, y + 4, 512, y + 5], fill=(232, 234, 233))
    save(WHITE, im, 0.85, 0.0, 1.0)
    save("wsm_cap", X.paint(128, (225, 227, 226), 0.04), 0.8)
    save("wsm_col", X.paint(128, (232, 233, 232), 0.03), 0.6)
    save("wsm_coping", X.paint(64, (198, 202, 205), 0.03), 0.4, 0.6)

    def panels(c, n=256, k=3, joint=(0.8, 0.8, 0.8), noise_=0.04):
        a = np.array(c, np.float64)[None, None, :] * (1 - noise_ + 2 * noise_ * X.noise(n))[..., None]
        im = Image.fromarray(a.clip(0, 255).astype(np.uint8))
        d = ImageDraw.Draw(im)
        jc = tuple(int(v * j) for v, j in zip(c, joint))
        for i in range(k):
            x = int(i * n / k)
            d.line([(x, 0), (x, n)], fill=jc, width=2)
        d.line([(0, n // 2), (n, n // 2)], fill=jc, width=2)
        return im

    save("wsm_blue", panels((112, 152, 206)), 0.3, 0.5, 0.6)          # light blue L2 band / 水色の帯
    save("wsm_blue2", panels((72, 108, 168)), 0.3, 0.5, 0.6)          # royal blue (shell) / 群青
    save("wsm_grey", panels((150, 153, 148), k=6), 0.4, 0.6, 0.5)
    # teal bulkhead with rounded cut-outs (W arcade) / 西側の青緑の垂れ壁
    im = Image.new("RGB", (384, 166), (40, 128, 138))
    d = ImageDraw.Draw(im)
    for x in range(0, 384, 96):
        d.rounded_rectangle([x + 14, 40, x + 82, 126], radius=30, fill=(70, 160, 168))
    save("wsm_teal", im, 0.5, 0.2)

    def shop(fascias, W=600, H=400):
        im = Image.new("RGB", (W, H), (40, 46, 52))
        d = ImageDraw.Draw(im)
        unit = W // len(fascias)
        for i, col in enumerate(fascias):
            x0 = i * unit
            d.rectangle([x0, 0, x0 + unit, 70], fill=col)                 # fascia / 看板帯
            d.rectangle([x0 + 20, 22, x0 + unit - 20, 48], fill=tuple(min(255, v + 60) for v in col))
            for x in range(x0 + 8, x0 + unit, 50):
                d.rectangle([x, 76, x + 40, H - 8], fill=(78, 98, 108))  # glazing panes / ガラス
                d.rectangle([x + 3, 80, x + 37, 200], fill=(104, 126, 134))
            d.rectangle([x0 + unit // 2 - 30, 210, x0 + unit // 2 + 30, H - 8], fill=(150, 140, 120))  # lit door
            d.rectangle([x0, 0, x0 + 6, H], fill=(205, 207, 206))         # pier / 柱
        d.rectangle([0, H - 8, W, H], fill=(120, 120, 118))
        return im

    save("wsm_shop", shop([(232, 232, 228), (60, 120, 70), (225, 225, 220)]), 0.15, 0.2)
    save("wsm_shop_r", shop([(150, 34, 30), (160, 36, 32), (232, 232, 228)]), 0.15, 0.2)
    save("wsm_soffit", X.stripes(128, (176, 178, 176), (192, 194, 192), 8, vertical=True), 0.7)
    save("wsm_canopy", X.paint(128, (236, 237, 237), 0.03), 0.6)
    save("wsm_glass_canopy", X.paint(128, (150, 192, 218), 0.03), 0.15, 0.2)
    save("wsm_dark", X.paint(64, (28, 30, 34), 0.05), 0.9)
    save("wsm_mull", X.paint(64, (62, 64, 68), 0.03), 0.4, 0.6)
    save("wsm_led", X.paint(64, (22, 28, 44), 0.05), 0.2, 0.3)
    save("wsm_cream", X.stripes(128, (222, 212, 178), (232, 224, 196), 10, vertical=True), 0.5)
    save("wsm_louvre", X.stripes(128, (190, 194, 196), (120, 124, 128), 16), 0.5, 0.4, 1.0)
    save("wsm_lattice", X.stripes(128, (228, 230, 230), (150, 156, 160), 12, vertical=True), 0.6)
    save("wsm_ac", X.stripes(128, (226, 229, 229), (200, 204, 206), 10), 0.6, 0.2)
    im = Image.new("RGB", (128, 128), (224, 227, 227))
    d = ImageDraw.Draw(im)
    d.ellipse([18, 18, 110, 110], fill=(70, 74, 78))
    d.ellipse([56, 56, 72, 72], fill=(200, 200, 200))
    save("wsm_ac_top", im, 0.6, 0.2)
    save("wsm_solar", X.solar_panel(256), 0.2, 0.4)
    save("wsm_letter_w", X.paint(64, (246, 247, 247), 0.0), 0.5)
    save("wsm_letter_b", X.paint(64, (38, 86, 168), 0.0), 0.5)
    # perforated metal: grey with a dot grid / 有孔メタル
    n = 256
    yy, xx = np.indices((n, n))
    dots = ((yy % 16 - 8) ** 2 + ((xx + (yy // 16 % 2) * 8) % 16 - 8) ** 2) < 12
    a = np.where(dots[..., None], np.array([96, 104, 112]), np.array([150, 158, 166])).astype(np.uint8)
    save("wsm_perf", Image.fromarray(a), 0.5, 0.6, 0.8)
    # two-tone metal shell: v runs 0 at the band to 1 at the arched top; dark
    # panels (76, 86, 97) below a diagonal wave near mid-height, light above
    # (tile 40 m along) / 二色のメタル（下半分が濃色、斜めの波で切り替え）
    W, H = 1024, 512
    yy, xx = np.indices((H, W))
    vv = (H - 1 - yy) / (H - 1)
    xs = xx / W
    wave = 0.5 + 0.13 * np.sin(2 * np.pi * xs) + 0.05 * np.sin(4 * np.pi * xs + 1.0)
    lo = vv < wave
    base = np.where(lo[..., None], np.array([76, 86, 97]), np.array([160, 168, 175])).astype(np.float64)
    edge = np.abs(vv - wave) < 0.012
    base[edge] = np.array([196, 200, 204])
    joint = ((xx % int(W / 40 * 1.25)) < 2) | ((yy % int(H / 12)) < 2)
    base[joint] *= 0.84
    base *= (0.96 + 0.08 * np.asarray(Image.fromarray((X.noise(512) * 255).astype(np.uint8)).resize((W, H)),
                                      np.float64)[..., None] / 255.0)
    save("wsm_metal", Image.fromarray(base.clip(0, 255).astype(np.uint8)), 0.35, 0.7, 0.6)
    save("wsm_metal_top", X.stripes(128, (150, 157, 163), (172, 178, 184), 10, vertical=True), 0.35, 0.7, 1.0)
    save("wsm_linkroof", X.stripes(128, (205, 208, 208), (222, 224, 224), 8, vertical=True), 0.5, 0.5, 0.8)
    # the W face's wave band: two blue waves on white (tile 27 m x 8.5 m) / 西面の波の帯
    W, H = 810, 255
    im = Image.new("RGB", (W, H), WH)
    d = ImageDraw.Draw(im)
    xs = np.arange(W + 1)
    for amp, off, wdt, col in ((60, 150, 44, (54, 96, 168)), (50, 92, 18, (120, 160, 210))):
        top = [(x, off - amp * math.sin(2 * math.pi * x / W)) for x in xs]
        bot = [(x, off + wdt - amp * math.sin(2 * math.pi * x / W)) for x in xs[::-1]]
        d.polygon(top + bot, fill=col)
    save("wsm_wave", im, 0.85)
    # NW fascia blue with pale blobs; also the car-park fascia / 北西入口の青い表示板
    im = Image.new("RGB", (256, 128), (50, 114, 160))
    d = ImageDraw.Draw(im)
    for x, y, r in ((30, 90, 38), (70, 110, 26), (210, 20, 30)):
        d.ellipse([x - r, y - r, x + r, y + r], fill=(120, 170, 205))
    save("wsm_fascia", im, 0.4, 0.2)
    # a generic poster panel (abstract, no brand) / 汎用のポスター（抽象）
    im = Image.new("RGB", (256, 256), (236, 220, 226))
    d = ImageDraw.Draw(im)
    d.ellipse([40, 60, 150, 170], fill=(214, 120, 150))
    d.ellipse([120, 110, 220, 210], fill=(250, 196, 120))
    d.rectangle([0, 0, 255, 255], outline=(60, 60, 64), width=8)
    save("wsm_poster", im, 0.5)
    # ribbon window: dark glass, a mullion every 1.5 m tile / 連窓
    im = Image.new("RGB", (128, 256), (50, 66, 72))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 6, 256], fill=(40, 42, 46))
    d.rectangle([0, 80, 128, 84], fill=(40, 42, 46))
    d.rectangle([10, 90, 118, 250], fill=(82, 104, 110))
    save("wsm_ribbon", im, 0.1, 0.3)
    # the glazed bay: dark frames, 3 x 4 panes / 黒枠のガラス面
    im = Image.new("RGB", (256, 384), (40, 42, 46))
    d = ImageDraw.Draw(im)
    for i in range(3):
        for j in range(4):
            d.rectangle([8 + i * 83, 8 + j * 94, 8 + i * 83 + 72, 8 + j * 94 + 84], fill=(78, 100, 112))
    save("wsm_bay", im, 0.1, 0.3)
    # skylight glazing / トップライトのガラス
    im = Image.new("RGB", (128, 128), (40, 50, 62))
    d = ImageDraw.Draw(im)
    for k in range(0, 128, 32):
        d.line([(k, 0), (k, 128)], fill=(150, 156, 160), width=3)
        d.line([(0, k), (128, k)], fill=(150, 156, 160), width=3)
    save("wsm_sky", im, 0.1, 0.4)
    # floor tiles: grey / white / charcoal / 床タイル
    rng = np.random.default_rng(3)
    im = Image.new("RGB", (256, 256))
    d = ImageDraw.Draw(im)
    cols = [(170, 172, 172), (214, 214, 210), (82, 84, 88)]
    for i in range(8):
        for j in range(8):
            d.rectangle([i * 32, j * 32, i * 32 + 31, j * 32 + 31], fill=cols[int(rng.choice(3, p=[0.5, 0.3, 0.2]))])
    save("wsm_tiles", im, 0.7)
    return out
