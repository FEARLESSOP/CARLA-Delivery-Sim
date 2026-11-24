"""Temasek Polytechnic (21 Tampines Ave 1) for the PasirRisTampines map
(maps/.venv). James Stirling, Michael Wilford & Associates with DP Architects,
1991-95; East Wing Blk 1A by Forum Architects, 2015.

Sources (build/PasirRisTampines/tp_photos, visual reference only, never
textures): the zoom-20 tiles (overlap scale), the campus's OSM outlines and
parts (they sit on the wall bases: the roofs lean 0.17 m per metre of height
toward bearing 345 deg on the tiles), OneMap 3-D heights per OSM way and its
orbits, Street View 2024-26 on Ave 1 (2010 inside), the Commons photos (main
gate 2014, courtyard 2026), the 2020 library photo, the architects' photos of
Blk 1A. Trace: build/<Name>/tp_trace.json (made by tp_trace_make.py beside it).
Colours: description.md (measured medians; blue-violet roofs 90,95,140).

  horseshoe    Blk 9: the U opening NE, 2 storeys below the plaza deck (8 m) + 5
               above, punched windows under navy sunshade bars ("T" windows),
               the courtyard face with the glazed arcade (green canopy, yellow
               wall), the recessed level-4 gallery with its glass balustrade and
               green-grey spandrel, the swept blue-violet roof rising to the outer
               edge (inner eave 30 m, outer 38.5 m), navy fascia over a white
               soffit band, the gateway (25 m wide, 3 storeys) where the lawn's
               palm axis passes, the dark glazed lower floors on the lawn side,
               plant boxes with fans on the arm ends, the name band on the E arm
  plaza        Blk 9A podium (dark glazed front, charcoal canopy, red
               "Temasek POLYTECHNIC" letters, the WELCOME fascia), the deck in
               lilac-grey pavers with turf panels and a paved star, the drum
               (8-spoke skylight), the sunken terraced garden, the saucer canopy,
               glass balustrades, the ramp + steps from the drop-off
  library      Blk 27: the cross in three tiers to 49 m (grid curtain walls in the
               re-entrant corners, the white middle, the crown with flared white
               walls and the blue glass band)
  wings        the West (Blk 10-25) and East (Blk 1-8) wings: gabled spine blocks
               with ribbon windows and grey sunshades, 8-storey slabs with T
               windows, louvre panels and red base panels, X-plan blocks with
               their hipped tops and round lanterns, angled arms ending in drum
               stair towers with blue domes
  others       Blk 1A (three tower pairs with curved-top white frames, the green
               podium roof), the round Blk 41/43/45 with balcony bands and the
               green strip, the fan-shaped convention centre (Blk 34), the
               sawtooth sports hall (Blk 29), SIT@TP (Blk 29A), tennis on the roof
               of Blk 26A, green roofs, Blk 30 with its ochre fascia, link bridges
  grounds      the great lawn (striped panels, paved paths, the palm axis, the pad),
               the planted water channel with 4 white ring basins, the stepped
               amphitheatre, courts, the stadium (track lanes, football pitch),
               pools, campus drives, the boundary wall + railing, guard houses and
               the West gate sign wall

Roads win: every footprint, the deck and the ramp keep GAP off the road network
(the site's own drives and car-park aisles included, check_roads --site tp);
links that would stand over a campus drive are cut there, the covered
footbridge over Tampines Ave 1 is left out, and any triangle still touching a
road is dropped and reported. Coordinates: the map frame (x east, y north).

PasirRisTampines 用の淡馬錫理工学院（Temasek Polytechnic）。写真は参照のみ（テクスチャに
使わない）。色は description.md の実測値。道路優先（敷地内の車路も道路扱い、Ave 1 の
歩道橋は作らない）。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import polylabel, unary_union
from shapely.strtree import STRtree

import buildings as B
import geo
from landmarks.coral import glyphs_on, opening, wall_openings
from landmarks.prcs import text_poly
from mesh import Mesh, polys

NAME = "Temasek Polytechnic"
TAG = "TP"
TRACE = "tp_trace.json"
MAP = "PasirRisTampines"
SITE_NAMES = (NAME,)
Z0 = 0.15                 # the raised ground / 敷地面
GAP = 1.0                 # footprints this far off the road network / 道路網からの離れ
ROOM = 0.4                # projections this far off it / 張り出しの離れ
WHITE = "tp_white"
NAVY = "tp_navy"
ROOF = "tp_roof"
FASCIA = "tp_fascia"
GREY = "tp_grey"


def enabled(out, meta):
    """Only on PasirRisTampines, with its trace. / この地図とトレースがある時だけ。"""
    return meta.get("name") == MAP and os.path.exists(os.path.join(out, TRACE))


# -- primitives (test_tp.py: every face points the way it is seen) ---------------
# 基本形状（test_tp.py で面の向きを確認）
def wall_top(m, mat, p0, p1, z0, za, zb, uvs=1.0):
    """A vertical face from p0 to p1 (facing right of p0->p1) whose top runs from
    za at p0 to zb at p1. / 上端が傾いた垂直面（p0->p1 の右向き）。"""
    L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if L < 1e-4 or max(za, zb) - z0 < 1e-4:
        return
    a, b = (p0[0], p0[1], z0), (p1[0], p1[1], z0)
    c, d = (p1[0], p1[1], max(zb, z0)), (p0[0], p0[1], max(za, z0))
    m.tris(mat, [a, b, c, d], np.r_[(p1[1] - p0[1]) / L, -(p1[0] - p0[0]) / L, 0.0],
           [(0, z0 / uvs), (L / uvs, z0 / uvs), (L / uvs, zb / uvs), (0, za / uvs)], [0, 1, 2, 0, 2, 3])


def sweep(s):
    """The horseshoe roof's cross profile, 0 at the inner eave, 1 at the outer
    edge: a steep rise off the courtyard eave that flattens toward the outer edge
    (the arm end in the main-gate photo; OneMap reads ~44 m over most of the ring).
    馬蹄形の屋根の断面（中庭側で急に上り、外側で平らに）。"""
    return 1.0 - (1.0 - min(max(s, 0.0), 1.0)) ** 2.2


def band(m, mat, p0, p1, za, zb, h):
    """A vertical band of height h whose top runs from za at p0 to zb at p1
    (a fascia along a sloping edge), facing right of p0->p1.
    傾いた縁に沿う一定幅の帯（p0->p1 の右向き）。"""
    L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if L < 1e-4:
        return
    m.tris(mat, [(p0[0], p0[1], za - h), (p1[0], p1[1], zb - h), (p1[0], p1[1], zb), (p0[0], p0[1], za)],
           np.r_[(p1[1] - p0[1]) / L, -(p1[0] - p0[0]) / L, 0.0], [(0, 0), (L, 0), (L, h), (0, h)], [0, 1, 2, 0, 2, 3])


def inset_ring(poly, d):
    """The CCW outer ring moved d inward with mitred corners, vertex for vertex,
    or None when an edge would flip or vanish (then no mansard).
    外周を頂点ごとに内側へ（辺が反転・消滅する時は None）。"""
    c = np.asarray(orient(poly, 1.0).exterior.coords)[:-1]
    n = len(c)
    if n < 3:
        return None
    t = np.roll(c, -1, axis=0) - c
    L = np.linalg.norm(t, axis=1)
    if np.any(L < 1e-6):
        return None
    t /= L[:, None]
    nin = np.column_stack([-t[:, 1], t[:, 0]])          # inward (left of a CCW edge) / 内向き
    out = []
    for i in range(n):
        n1, n2 = nin[i - 1], nin[i]
        den = 1.0 + float(n1 @ n2)
        if den < 0.15:
            return None
        out.append(c[i] + d * (n1 + n2) / den)
    out = np.asarray(out)
    t2 = np.roll(out, -1, axis=0) - out
    if np.any(np.sum(t2 * t, axis=1) <= 0.05 * L):
        return None
    q = Polygon(out)
    if not q.is_valid or q.area < 1.0:
        return None
    return c, out


def mansard(m, poly, z0, z1, d, mat, top=None):
    """A hipped / mansard roof on any footprint: each eave edge slopes up and in
    to the inset ring (faces up and out), a flat top. False when the footprint
    is too narrow for d. / 任意形状の寄棟（腰折れ）屋根。"""
    r = inset_ring(poly, d)
    if r is None:
        return False
    c, o = r
    n = len(c)
    for i in range(n):
        j = (i + 1) % n
        m.quad(mat, (c[i][0], c[i][1], z0), (c[j][0], c[j][1], z0), (o[j][0], o[j][1], z1),
               (o[i][0], o[i][1], z1), (0, 0), (np.linalg.norm(c[j] - c[i]) / 2, 0),
               (np.linalg.norm(c[j] - c[i]) / 2, d / 2), (0, d / 2))
    m.flat(top or mat, Polygon(o), z1, 2.0)
    return True


def dome(m, mat, c, r, z, h, seg=16, rings=4):
    """An upper half-ellipsoid dome (faces out). / 半球のドーム（外向き）。"""
    cx, cy = c
    P, N, UV, I = [], [], [], []
    for j in range(rings + 1):
        th = math.pi / 2 * j / rings
        for k in range(seg + 1):
            ph = 2 * math.pi * k / seg
            s, co = math.cos(th), math.sin(th)
            P.append((cx + r * s * math.cos(ph), cy + r * s * math.sin(ph), z + h * co))
            nn = np.array([s * math.cos(ph) / r, s * math.sin(ph) / r, co / max(h, 1e-6)])
            N.append(nn / np.linalg.norm(nn))
            UV.append((k / seg * 3, j / rings))
    for j in range(rings):
        for k in range(seg):
            a = j * (seg + 1) + k
            b = a + seg + 1
            if j == rings - 1:
                I += [a, a + 1, b]
            else:
                I += [a, a + 1, b + 1, a, b + 1, b]
    m.tris(mat, P, N, UV, I)


def drum(m, mat, c, r, z0, z1, seg=24, uvs=1.0):
    """A vertical cylinder wall (faces out), no caps. / 円筒の壁（外向き）。"""
    cx, cy = c
    for k in range(seg):
        a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
        p0 = (cx + r * math.cos(a0), cy + r * math.sin(a0))
        p1 = (cx + r * math.cos(a1), cy + r * math.sin(a1))
        m.wall(mat, p0, p1, z0, z1, u0=2 * math.pi * r * k / seg / uvs, uv_scale=uvs)


def annulus(m, mat, c, r0, r1, z, up=True, seg=24):
    """A flat ring between r0 and r1 at z (up or down). / 平らな輪。"""
    ring = Point(c).buffer(r1, seg // 4).difference(Point(c).buffer(r0, seg // 4))
    m.flat(mat, ring, z, 1.0, up=up)


def frustum(m, mat, c, r0, r1, z0, z1, seg=24, inside=False):
    """A cone frustum from (r0, z0) to (r1, z1); faces out (or in). / 円錐台。"""
    cx, cy = c
    for k in range(seg):
        a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
        q = [(cx + r0 * math.cos(a0), cy + r0 * math.sin(a0), z0), (cx + r0 * math.cos(a1), cy + r0 * math.sin(a1), z0),
             (cx + r1 * math.cos(a1), cy + r1 * math.sin(a1), z1), (cx + r1 * math.cos(a0), cy + r1 * math.sin(a0), z1)]
        if inside:
            q = q[::-1]
        m.quad(mat, *q)


def gable_at(m, poly, z0, z1, mat, wall, ang=None, ov=0.6, fascia=FASCIA, soffit="concrete"):
    """A gable roof over the footprint's rectangle, the ridge along ``ang``
    (radians, map frame) or the long side: slopes up and out, gable ends out,
    soffit down, fascia boards out. Returns the eave rectangle.
    切妻屋根（棟は ang 方向か長辺）。"""
    if ang is None:
        r = orient(poly.minimum_rotated_rectangle, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, b, cc, d = [c[(k + i) % 4] for i in range(4)]
    else:
        ax = np.array([math.cos(ang), math.sin(ang)])
        sd = np.array([-ax[1], ax[0]])
        P = np.asarray(poly.exterior.coords)
        u, v = P @ ax, P @ sd
        a, b = ax * u.min() + sd * v.min(), ax * u.max() + sd * v.min()
        cc, d = ax * u.max() + sd * v.max(), ax * u.min() + sd * v.max()
    ax = (b - a) / np.linalg.norm(b - a)
    sd = (d - a) / np.linalg.norm(d - a)
    a2, b2, c2, d2 = a - ax * ov - sd * ov, b + ax * ov - sd * ov, cc + ax * ov + sd * ov, d - ax * ov + sd * ov
    r1, r2 = (a2 + d2) / 2, (b2 + c2) / 2
    Lr = float(np.linalg.norm(b2 - a2))
    W = float(np.linalg.norm(d2 - a2))
    Q = lambda q, z: (q[0], q[1], z)
    m.quad(mat, Q(a2, z0), Q(b2, z0), Q(r2, z1), Q(r1, z1), (0, 0), (Lr / 1.2, 0), (Lr / 1.2, W / 4), (0, W / 4))
    m.quad(mat, Q(c2, z0), Q(d2, z0), Q(r1, z1), Q(r2, z1), (0, 0), (Lr / 1.2, 0), (Lr / 1.2, W / 4), (0, W / 4))
    m1, m2 = (a + d) / 2, (b + cc) / 2
    m.tris(wall, [Q(d, z0), Q(a, z0), Q(m1, z1)], np.r_[-ax, 0], [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
    m.tris(wall, [Q(b, z0), Q(cc, z0), Q(m2, z1)], np.r_[ax, 0], [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
    m.quad(soffit, Q(b2, z0), Q(a2, z0), Q(d2, z0), Q(c2, z0))
    for p, q in ((a2, b2), (c2, d2)):
        m.wall(fascia, tuple(p), tuple(q), z0 - 0.35, z0 + 0.02)
    # the fascia along the rakes (verge boards) / 妻側の破風板
    for p, q, top in ((d2, a2, r1), (b2, c2, r2)):
        for s, e, zs, ze in ((p, top, z0, z1), (top, q, z1, z0)):
            wall_top(m, fascia, tuple(s), tuple(e), min(zs, ze) - 0.35, zs + 0.02, ze + 0.02)
    return a, b, cc, d


def drop_over(m, no_go):
    """Remove every triangle touching ``no_go`` (the last guard: roads win), the
    test check_roads.py makes: vertical faces by their two lowest vertices. The
    kept triangles are re-indexed onto compacted vertices (Mesh.merge subtracts
    each chunk's lowest index: coral/dte drop_over leave unused vertices in front,
    which shifted the kept triangles onto the wrong vertices). Returns the number
    dropped. / 道路に触れる三角形を除く（頂点を詰め直す: merge が最小の添字を引くため）。"""
    shapely.prepare(no_go)
    gone = 0
    for mat, part in m.parts.items():
        V, N, UV, I, n = part
        if not V:
            continue
        Vc, Nc, Uc, Ic = np.concatenate(V), np.concatenate(N), np.concatenate(UV), np.concatenate(I)
        if len(Ic) == 0:
            continue
        T = Vc[Ic]
        tri = shapely.polygons(np.concatenate([T[:, :, :2], T[:, :1, :2]], axis=1))
        flat = np.abs(shapely.area(tri)) < 1e-6
        k = np.argsort(T[:, :, 2], axis=1, kind="stable")[:, :2]
        low = np.take_along_axis(T[:, :, :2], k[:, :, None], axis=1)
        base = shapely.linestrings(low)
        hit = np.where(flat, shapely.intersects(no_go, base), shapely.intersects(no_go, tri))
        if not hit.any():
            continue
        gone += int(hit.sum())
        keep = Ic[~hit]
        used = np.unique(keep)
        remap = np.full(len(Vc), -1, np.int64)
        remap[used] = np.arange(len(used))
        part[0], part[1], part[2], part[3] = [Vc[used]], [Nc[used]], [Uc[used]], [remap[keep]]
        part[4] = len(used)
    return gone


def circle_fit(pts):
    """Least-squares circle through points (Kasa). / 最小二乗の円。"""
    P = np.asarray(pts, float)
    A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))])
    bb = (P ** 2).sum(axis=1)
    cx, cy, k = np.linalg.lstsq(A, bb, rcond=None)[0]
    return (float(cx), float(cy)), float(math.sqrt(max(k + cx * cx + cy * cy, 1e-6)))


def arc_run(poly, max_edge=4.2, min_n=6):
    """The longest run of short edges of a ring (a drum end), fitted as a circle.
    短い辺が続く所（円筒の端部）を円で近似。"""
    c = np.asarray(orient(poly, 1.0).exterior.coords)[:-1]
    n = len(c)
    L = np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1)
    short = L < max_edge
    if short.all() or not short.any():
        return None
    start = int(np.argmin(short))           # a long edge: runs start after it / 長い辺の後から
    best, run = [], []
    for k in range(1, n + 1):
        i = (start + k) % n
        if short[i]:
            run.append(i)
        else:
            if len(run) > len(best):
                best = run
            run = []
    if len(run) > len(best):
        best = run
    if len(best) < min_n:
        return None
    pts = [c[i] for i in best] + [c[(best[-1] + 1) % n]]
    return circle_fit(pts)


def rail_run(m, mat, p, q, z, h=1.1, glass=False, post=1.8):
    """A railing p->q on z: glass panel (two-sided) with a top rail, or steel
    top + mid rails and posts. / 手すり（ガラスか鋼製）。"""
    p, q = np.asarray(p, float), np.asarray(q, float)
    L = float(np.linalg.norm(q - p))
    if L < 0.3:
        return
    ang = math.atan2(q[1] - p[1], q[0] - p[0])
    mid = (p + q) / 2
    if glass:
        n = np.array([q[1] - p[1], p[0] - q[0]]) / L * 0.004
        m.wall("glass_rail", tuple(p + n), tuple(q + n), z, z + h - 0.06)
        m.wall("glass_rail", tuple(q - n), tuple(p - n), z, z + h - 0.06)
        m.box("railing", mid[0], mid[1], z + h - 0.06, z + h, L, 0.06, ang)
        return
    for z0, z1 in ((z + h - 0.05, z + h), (z + h * 0.5 - 0.025, z + h * 0.5 + 0.025)):
        m.box(mat, mid[0], mid[1], z0, z1, L, 0.05, ang)
    for k in range(int(L // post) + 1):
        s = min(L, k * post + (L - post * int(L // post)) / 2)
        r = p + (q - p) * s / L
        m.box(mat, r[0], r[1], z, z + h - 0.05, 0.05, 0.05, ang, top=False)


def rail_panel(m, panel, rail, p, q, z, h, post=6.0):
    """A cheap steel railing: the bars drawn on a two-sided panel (6 mm apart),
    a top rail and posts every ``post`` m (boundary runs: 20 k triangles of box
    rails became ~4 k). / 安価な手すり: 縦桟は両面パネルのテクスチャ、笠木と支柱。"""
    p, q = np.asarray(p, float), np.asarray(q, float)
    L = float(np.linalg.norm(q - p))
    if L < 0.3:
        return
    t = (q - p) / L
    n = np.array([t[1], -t[0]]) * 0.003
    m.wall(panel, tuple(p + n), tuple(q + n), z, z + h - 0.05, 0.0, h)
    m.wall(panel, tuple(q - n), tuple(p - n), z, z + h - 0.05, 0.0, h)
    mid = (p + q) / 2
    ang = math.atan2(t[1], t[0])
    m.box(rail, mid[0], mid[1], z + h - 0.05, z + h, L, 0.06, ang)
    for k in range(int(L // post) + 1):
        r = p + t * min(L, k * post + (L - post * int(L // post)) / 2)
        m.box(rail, r[0], r[1], z, z + h - 0.05, 0.07, 0.07, ang, top=False)


def centres(u0, u1, bay, margin=0.8):
    n = int((u1 - u0 - 2 * margin) // bay)
    if n <= 0:
        return []
    w = (u1 - u0 - 2 * margin) / n
    return [u0 + margin + w * (i + 0.5) for i in range(n)]


def resample(line, n):
    """n + 1 points evenly along a polyline. / 折れ線を等間隔に。"""
    ls = LineString(line)
    return [np.array(ls.interpolate(ls.length * k / n).coords[0]) for k in range(n + 1)]


class TP(object):
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
        self.XP = lambda pts: Polygon([self.X(q) for q in pts]).buffer(0)
        site = [p for p, t in L.osm.areas(lambda t: t.get("name") in SITE_NAMES and "building" not in t)]
        self.site = unary_union(site).buffer(0)
        self.empty = self.site.is_empty
        if self.empty:
            # a crop build without the campus: nothing to add, nothing to clear
            # (other agents' crops copy this trace) / 敷地の無い切り出し: 何もしない
            self.blocks, self.footprint, self.deck = {}, Polygon(), Polygon()
            self._parts, self._ground, self._inner = [], None, Polygon()
            print("tp: the site is not in this build (crop): nothing built")
            return
        # roads win: road network, islands, the site's own drives and aisles
        # 道路優先: 道路網・島・敷地内の車路と通路
        near = self.site.buffer(8.0)
        self.road = L.road_all
        self.islands = L.islands.intersection(self.site.buffer(3.0))
        self.drives = L.driveways.intersection(near)
        self.parking_snapshot = L.parking.intersection(near)
        self.no_go = unary_union([L.road_all, L.islands.intersection(near), self.drives])
        shapely.prepare(self.no_go)
        self.fit_log = []
        ng = self.no_go.buffer(GAP)
        self.ng = ng
        # the lot: the OSM area less the streets and their pavement band, open at
        # the gates (coral's method) / 敷地: 道路と歩道帯を除く（門は開ける）
        ring = self.site.exterior
        span = ring.intersection(self.no_go.buffer(0.3))
        deep = self.no_go.intersection(self.site.buffer(-4.0))
        self.gates = [g for g in getattr(span, "geoms", [span]) if g.geom_type == "LineString"
                      and 2.5 < g.length < 16.0 and deep.distance(g) < 6.0]
        edge_roads = self.road.buffer(2.6).intersection(ring.buffer(9.0))
        for g in self.gates:
            edge_roads = edge_roads.difference(g.buffer(7.0))
        lot = self.site.difference(edge_roads)
        self.lot = max(polys(lot), key=lambda q: q.area) if polys(lot) else self.site
        self.blocks = {}
        self.solids = []          # (poly, base, top, key) for the wall-cover test / 壁の隠れ判定
        self.make_horseshoe()
        self.make_library()
        for k, b in T["blocks"].items():
            self.add_block(k, dict(b))
        self.make_plaza()
        # the planted channel lies just outside the OSM campus, by the canal: it
        # joins the site's ground (generic park cleared there) / 水路は敷地の外側（運河沿い）
        Lw = T["lawn"]
        a_, b_ = np.array(self.X(Lw["channel"]["a"])), np.array(self.X(Lw["channel"]["b"]))
        r_ = Lw["channel"]["r"]
        ch = LineString([tuple(a_), tuple(b_)]).buffer(Lw["channel"]["w"] / 2 - r_, cap_style="flat").buffer(r_)
        self.chan_zone = ch.buffer(1.2).difference(self.no_go.buffer(0.5))
        self._inner = None
        self.footprint = unary_union([s[0] for s in self.solids if s[1] < 1.0] + [self.deck])
        self.tree = STRtree([s[0] for s in self.solids])
        self.centre = self.footprint.centroid
        self._parts = None
        self._ground = None
        for line in self.fit_log:
            print("tp: " + line)

    # -- fitting / 道路に合わせる --------------------------------------------------
    def fit_poly(self, k, p, rect=False):
        """The footprint less the road network + GAP; the largest piece if more
        than 35 % survives. / 道路網 + GAP を除いた外形。"""
        p = p.buffer(0)
        if p.distance(self.no_go) >= GAP - 1e-6:
            return p
        q = p.difference(self.ng)
        ps = [g for g in polys(q) if g.area > 4.0]
        if not ps:
            self.fit_log.append("%s left out (road network)" % k)
            return None
        g = max(ps, key=lambda g_: g_.area)
        if g.area < 0.35 * p.area:
            self.fit_log.append("%s left out (%.0f %% left beside the road network)" % (k, 100 * g.area / p.area))
            return None
        g = orient(shapely.set_precision(g, 0.01).buffer(0), 1.0)
        if g.geom_type != "Polygon":
            g = orient(max(polys(g), key=lambda q_: q_.area), 1.0)
        self.fit_log.append("%s: cut %.1f m2 off the road network" % (k, p.area - g.area))
        return g

    def add_block(self, k, b):
        p = self.XP(b["poly"])
        if b["kind"] == "xblock":
            up = self.XP(b["upper"])
        p = self.fit_poly(k, p)
        if p is None:
            return
        b["poly"] = p
        b.setdefault("base", 0.0)
        if b["kind"] == "xblock":
            u = up.intersection(p.buffer(-0.3, join_style="mitre"))
            b["upper_poly"] = max(polys(u), key=lambda q: q.area) if polys(u) else None
        if b["kind"] == "eastwing":
            tw = []
            for t in b["towers"]:
                q = self.fit_poly(k + "_tower", self.XP(t))
                if q is not None:
                    tw.append(q)
            b["tower_polys"] = tw
        if b["kind"] in ("arm",):
            fit = arc_run(p)
            b["drum"] = fit if fit is not None and 3.0 < fit[1] < 9.0 else None
        if b["kind"] == "round":
            c, r = circle_fit(np.asarray(p.exterior.coords)[:-1])
            r = min(r, self.no_go.distance(Point(c)) - GAP)
            b["c"], b["r"] = c, r
            p = Point(c).buffer(r, 12)
            b["poly"] = p
        self.blocks[k] = b
        self.solids.append((p, b.get("base", 0.0), b["top"], k))
        if b["kind"] == "xblock" and b.get("upper_poly") is not None:
            self.solids.append((b["upper_poly"], b["top"], b["utop"], k + "_up"))
        if b["kind"] == "eastwing":
            for i, t in enumerate(b["tower_polys"]):
                self.solids.append((t, 0.0, b["ttop"], k + "_t%d" % i))

    # -- the horseshoe / 馬蹄形 ------------------------------------------------------
    def make_horseshoe(self):
        """Split the OSM U into its outer and inner chains (the arm-end edges are
        the two long jumps), resample both to pairs, trim pairs near a road.
        U 字を外周と内周に分けて対にする（道路に近い対は除く）。"""
        H = self.T["horseshoe"]
        self.hs = H
        cen = np.array(self.X(H["centre"]))
        outer = [np.array(self.X(q)) for q in H["outer"]]
        inner = [np.array(self.X(q)) for q in H["inner"]]
        N = 44
        O, I = resample(outer, N), resample(inner, N)
        # roads win: drop pairs at the arm ends that come within GAP of a road
        # 道路優先: 腕の端の対が道路に近ければ除く
        def bad(k):
            return Polygon([O[k], O[k + 1], I[k + 1], I[k]]).buffer(0).distance(self.no_go) < GAP
        a, b = 0, N
        while a < b - 4 and bad(a):
            a += 1
        while b > a + 4 and bad(b - 1):
            b -= 1
        if a or b < N:
            self.fit_log.append("horseshoe: %d pairs off the arm ends (road network)" % (a + N - b))
        self.hO, self.hI = O[a:b + 1], I[a:b + 1]
        mid = [k for k in range(len(self.hO) - 1) if bad(k)]
        if mid:
            self.fit_log.append("horseshoe: %d segments within GAP of a road (kept; drop_over guards)" % len(mid))
        poly = Polygon([tuple(q) for q in self.hO] + [tuple(q) for q in self.hI[::-1]]).buffer(0)
        self.hpoly = orient(poly, 1.0)
        # the gateway: the pair nearest the bearing from the centre / 門の位置
        br = math.radians(H["gateway"]["bearing"])
        g = cen + np.array([math.sin(br), math.cos(br)]) * 43.0
        mids = [(self.hO[k] + self.hI[k] + self.hO[k + 1] + self.hI[k + 1]) / 4 for k in range(len(self.hO) - 1)]
        dist = np.cumsum([0.0] + [np.linalg.norm(mids[k + 1] - mids[k]) for k in range(len(mids) - 1)])
        kg = int(np.argmin([np.linalg.norm(q - g) for q in mids]))
        w = H["gateway"]["width"] / 2
        self.gate_segs = {k for k in range(len(mids)) if abs(dist[k] - dist[kg]) <= w}
        self.hcen = cen
        self.solids.append((self.hpoly, 0.0, H["inner_eave"], "horseshoe"))

    # -- the library / 図書館 --------------------------------------------------------
    def make_library(self):
        """The cross re-traced from the roof parapet (OSM parts turned ~40 deg):
        arms along ang / ang + 90, the base filled with glass boxes in the corners.
        屋上から再トレースした十字（下段は入隅をガラスの箱で埋める）。"""
        Lb = self.T["library"]
        c = np.array(self.X(Lb["c"]))
        u = np.array([math.cos(math.radians(Lb["ang"])), math.sin(math.radians(Lb["ang"]))])
        v = np.array([-u[1], u[0]])
        R, a, fR = Lb["R"], Lb["a"], Lb["fill"] * Lb["R"]
        P = lambda pts: Polygon([tuple(c + u * x + v * y) for x, y in pts])
        cross = P([(R, -a), (R, a), (a, a), (a, R), (-a, R), (-a, a), (-R, a), (-R, -a), (-a, -a), (-a, -R),
                   (a, -R), (a, -a)])
        # corner boxes overlap the arms 5 cm (touching squares left slivers in the union)
        # 入隅の箱は腕に 5 cm 重ねる（接するだけだと和で隙間が残った）
        e = a - 0.05
        corners = [P([(sx * e, sy * e), (sx * fR, sy * e), (sx * fR, sy * fR), (sx * e, sy * fR)]).buffer(0)
                   for sx in (1, -1) for sy in (1, -1)]
        base = shapely.set_precision(unary_union([cross] + corners), 0.01).buffer(0)
        corners = [q.difference(cross).buffer(0).simplify(0.02) for q in corners]
        self.lib = None
        cr = self.fit_poly("library", cross)
        bs = self.fit_poly("library base", base)
        bs = bs.simplify(0.02) if bs is not None else None
        if cr is None or bs is None:
            return
        self.lib = dict(cross=orient(cr, 1.0), base=orient(bs, 1.0), c=c, u=u, v=v, R=R, a=a,
                        corners=[orient(q.intersection(bs).buffer(0).simplify(0.02), 1.0) for q in corners
                                 if q.intersection(bs).area > 4.0])
        self.libz = Lb["z"] + [Lb["top"]]
        self.solids.append((self.lib["base"], 0.0, Lb["z"][1], "library"))
        self.solids.append((self.lib["cross"], 0.0, Lb["top"], "library"))

    # -- the plaza deck / 広場デッキ -------------------------------------------------
    def make_plaza(self):
        Pz = self.T["plaza"]
        self.deck_z = Pz["deck"]
        pod = self.XP(Pz["podium"])
        # the courtyard: inside the U, closed by the podium / U の中庭
        court = Polygon([tuple(q) for q in self.hI]).buffer(0)
        deck = unary_union([pod, court]).difference(self.hpoly.buffer(0.02))
        if self.lib is not None:
            deck = deck.difference(self.lib["base"])
        deck = deck.difference(self.ng)
        deck = max(polys(deck), key=lambda q: q.area)
        self.deck = orient(shapely.set_precision(deck, 0.01).buffer(0), 1.0)
        D = Pz["drum"]
        self.drum_c, self.drum_r, self.drum_top = self.X(D["c"]), D["r"], D["top"]
        self.sunken = self.XP(Pz["sunken"]).intersection(self.deck.buffer(-1.5, join_style="mitre"))
        S = Pz["saucer"]
        self.saucer_c, self.saucer_r, self.saucer_z = self.X(S["c"]), S["r"], S["z"]
        R = Pz["ramp"]
        a, b = np.array(self.X(R["a"])), np.array(self.X(R["b"]))
        t = (b - a) / np.linalg.norm(b - a)
        nn = np.array([-t[1], t[0]])
        w = R["w"] / 2
        ramp = Polygon([tuple(a - nn * w), tuple(b - nn * w + t * 1.0), tuple(b + nn * w + t * 1.0), tuple(a + nn * w)])
        ramp = ramp.difference(self.ng).difference(self.hpoly).difference(self.deck.buffer(-0.01))
        self.ramp = max(polys(ramp), key=lambda q: q.area) if polys(ramp) else Polygon()
        self.ramp_a, self.ramp_t = a, t
        self.ramp_len = float(np.linalg.norm(b - a))
        self.solids.append((self.deck, 0.0, self.deck_z, "deck"))
        if not self.ramp.is_empty:
            self.fit_log.append("ramp: %.0f m2 (of %.0f) clear of the road network" % (self.ramp.area, 2 * w * self.ramp_len))

    # -- layout / 配置 -------------------------------------------------------------
    def inner(self):
        """The lot less the generic car parks (they stay generic: bays, lamps,
        trees) / 敷地（一般の駐車場は汎用のまま残す）。"""
        if getattr(self, "empty", False):
            return Polygon()
        if getattr(self, "_inner", None) is None:
            g = self.lot.buffer(-0.6, join_style="mitre").difference(self.parking_snapshot.buffer(0.5))
            g = unary_union([g, self.chan_zone]).difference(self.no_go.buffer(0.3))
            self._inner = g
        return self._inner

    def keepout(self):
        if self.empty:
            return Polygon()
        return unary_union([self.inner(), self.footprint.buffer(3.0)])

    def tree_filter(self, trees):
        if self.empty:
            return trees
        clear = unary_union([self.inner(), self.footprint.buffer(2.0)])
        return [t for t in trees if not clear.contains(Point(t[0], t[1]))]

    def zones(self):
        if self._ground is not None:
            return self._ground
        inner = self.inner().difference(self.no_go.buffer(0.05))
        foot = unary_union([self.footprint, self.ramp])
        Lw = self.T["lawn"]
        a, b = np.array(self.X(Lw["channel"]["a"])), np.array(self.X(Lw["channel"]["b"]))
        r = Lw["channel"]["r"]
        chan = self.chan_zone.buffer(-1.2).intersection(inner)
        self.chan = chan
        basins = unary_union([Point(self.X(q)).buffer(Lw["basin_r"], 16) for q in Lw["basins"]]).intersection(chan)
        A = Lw["amph"]
        ac = np.array(self.X(A["c"]))
        br = math.radians(A["bearing"])
        ad = np.array([math.sin(br), math.cos(br)])
        half = Point(ac).buffer(A["r"], 32).intersection(
            Polygon([tuple(ac + np.array([ad[1], -ad[0]]) * 40), tuple(ac - np.array([ad[1], -ad[0]]) * 40),
                     tuple(ac - np.array([ad[1], -ad[0]]) * 40 + ad * 40),
                     tuple(ac + np.array([ad[1], -ad[0]]) * 40 + ad * 40)]))
        self.amph = (ac, ad, A["r"], half.intersection(inner))
        pad = self.XP(Lw["pad"]).intersection(inner)
        # paths: OSM footways in the site (the axis 5 m, others 3 m) / 小道
        ways = []
        for ln, t, _ in self.L.osm.lines("highway"):
            if t.get("highway") in ("footway", "path", "pedestrian", "steps") and t.get("bridge") != "yes" \
                    and ln.intersects(self.site):
                ways.append(ln.buffer(1.5, cap_style="flat"))
        ax = LineString([self.X(q) for q in Lw["axis"]])
        ways.append(ax.buffer(2.6, cap_style="flat"))
        paths = unary_union(ways).intersection(inner).difference(foot).difference(chan)
        stripes = self.XP(Lw["stripes"]).intersection(inner).difference(paths).difference(foot)
        # courts and pitches (OSM, drawn once each) / コートとグラウンド
        self.courts = []
        for p, t in self.L.osm.areas(lambda t: t.get("leisure") in ("pitch", "track")):
            if not self.site.contains(p.representative_point()):
                continue
            self.courts.append((orient(p.buffer(0), 1.0), t.get("sport", ""), t.get("leisure")))
        court_area = unary_union([c for c, _, _ in self.courts]).intersection(inner).difference(foot) \
            if self.courts else Polygon()
        pools = unary_union([p for p, t in self.L.osm.areas(lambda t: t.get("leisure") == "swimming_pool")
                             if self.site.contains(p.representative_point())])
        pools = pools.intersection(inner)
        # the pool complex Blk 31A (OSM 882664788): its white deck round the pools / プールのデッキ
        pd = self.T.get("pool_deck")
        pool_deck = self.XP(pd).intersection(inner).difference(foot).difference(pools.buffer(0.6)) \
            if pd else Polygon()
        self.pool_deck_outline = self.XP(pd).intersection(inner) if pd else Polygon()
        drives = self.drives.intersection(self.lot.buffer(-0.3)).difference(self.parking_snapshot.buffer(0.4))
        hard = foot.buffer(2.0, join_style="mitre").intersection(inner).difference(foot)
        hard = unary_union([hard, paths, pad]).difference(court_area).difference(chan).difference(pools)
        hard = hard.difference(drives).difference(pool_deck.buffer(0.01))
        # no hairline slivers (they rasterise into walkable crumbs) / 髪の毛状の細片を除く
        hard = hard.buffer(-0.2, join_style="mitre").buffer(0.2, join_style="mitre").intersection(inner)
        grass = inner.difference(foot).difference(chan).difference(court_area).difference(hard).difference(stripes) \
            .difference(pools.buffer(0.6)).difference(drives).difference(pool_deck)
        stripes = stripes.difference(court_area).difference(hard)
        self._ground = dict(grass=grass, hard=hard, stripes=stripes, chan=chan.difference(basins), basins=basins,
                            drives=drives, pools=pools, courts=court_area, pad=pad, pool_deck=pool_deck)
        return self._ground

    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        Z = self.zones()
        T = lambda g: g.intersection(tile)
        m.flat("grass_park", T(Z["grass"]), z + 0.005, 4.0)
        m.flat("tp_lawnstripe", T(Z["stripes"]), z + 0.005, 6.0, uv_rot=math.radians(53.0))
        m.flat("concrete_path", T(Z["hard"].difference(self.walk_keep())), z + 0.01, 2.0)
        m.flat("asphalt_lot", T(Z["drives"]), z + 0.006, 4.0)
        # water above the generic grass base (0.14, drawn under the whole site),
        # the depth read from the raised copings / 水面は汎用の芝の下地より上（縁を上げて深さを見せる）
        m.flat("tp_marsh", T(Z["chan"]), z + 0.012, 5.0)
        m.flat("water", T(Z["basins"]), z + 0.02, 4.0)
        m.flat("pool_tile", T(Z["pools"]), z + 0.012, 2.0)
        m.flat("water", T(Z["pools"].buffer(-0.4)), z + 0.03, 4.0)
        for c, sport, kind in self.courts:
            if tile.contains(c.centroid):              # once, in its own tile / 自分のタイルに 1 回だけ
                self.court(m, c, sport, kind, z + 0.02)
        return m

    def court(self, m, c, sport, kind, z):
        """A court / pitch / track from its OSM area (textures per sport).
        OSM のコート・グラウンド・トラック。"""
        c = c.difference(self.footprint).intersection(self.inner())
        if c.is_empty:
            return
        if sport == "athletics" or kind == "track":
            m.flat("tp_track", c, z, 3.0)
            # lane lines: offsets of the infield edge every 1.22 m / レーンの線
            inner = None
            for p, s, k in self.courts:
                if s == "soccer" and c.buffer(1.0).contains(p.representative_point()):
                    inner = p
            if inner is not None:
                for k in range(7):
                    ln = inner.buffer(0.6 + 1.22 * k, 16).exterior.buffer(0.03).intersection(c)
                    m.flat("paint_white", ln, z + 0.006, 1.0)
            return
        mat = {"tennis": "court_tennis", "basketball": "court_bb_green", "badminton": "court_multi",
               "soccer": "tp_football"}.get(sport, "court_multi")
        if sport == "soccer":
            z += 0.01
        q = list(orient(c.minimum_rotated_rectangle, 1.0).exterior.coords)[:4]
        q = [np.array(p) for p in q]
        if math.dist(q[0], q[1]) < math.dist(q[1], q[2]):
            q = q[1:] + q[:1]
        # the court quad only where its rectangle fits the area / 長方形がはみ出す所は面だけ
        rect = Polygon([tuple(p) for p in q])
        if rect.difference(c).area < 0.03 * rect.area:
            m.quad(mat, *[(p[0], p[1], z) for p in q], (0, 0), (1, 0), (1, 1), (0, 1))
        else:
            m.flat("court_multi" if sport != "soccer" else "pitch_grass", c, z, 6.0)

    # -- buildings / 建物 ----------------------------------------------------------
    def buildings_mesh(self, tile):
        if self.empty:
            return Mesh()
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
        self.tri_count = {}

        def add(anchor, name, fn, *a, guard_=None, **k):
            pm = Mesh()
            fn(pm, *a, **k)
            nd = drop_over(pm, guard if guard_ is None else guard_)
            if nd:
                self.drop_log.append((name, nd))
            parts.append((anchor, pm))
            grp = name.split(":")[0]
            self.tri_count[grp] = self.tri_count.get(grp, 0) + pm.ntris()

        c = self.hpoly.centroid
        add((c.x, c.y), "horseshoe", self.horseshoe)
        c = self.deck.centroid
        add((c.x, c.y), "plaza", self.plaza)
        if self.lib is not None:
            add(tuple(self.lib["c"]), "library", self.library)
        for k, b in self.blocks.items():
            c = b["poly"].centroid
            add((c.x, c.y), "%s:%s" % (b["kind"], k), self.block, k, b)
        self.zones()
        c = self.chan.centroid if not self.chan.is_empty else self.site.centroid
        add((c.x, c.y), "lawn", self.lawn_works)
        c = self.site.centroid
        add((c.x, c.y), "boundary", self.boundary)
        add((c.x, c.y), "trees", self.trees)
        # the main gate's guard house stands on the drop-off island (SV h215):
        # guarded by the roads and drives only / 正門の守衛所は車寄せの島の上（道路・車路のみ避ける）
        g = Point(self.X(self.T["gates"]["main_guard"]))
        isl = [q for q in polys(self.L.islands.intersection(g.buffer(40.0))) if q.distance(g) < 15.0]
        if isl:
            road_only = unary_union([self.road, self.drives]).buffer(0.05)
            # the island with the most room near the guard point (SV h215) / 余裕の最も大きい島
            cand = [(road_only.distance(polylabel(q_, 0.1)), polylabel(q_, 0.1)) for q_ in isl]
            room, spot = max(cand, key=lambda c_: c_[0])
            # the island leaves 2.2 m round its widest point: a 2.4 x 2.0 m booth with a
            # 0.3 m roof overhang (corner reach 2.0 m) / 島の余裕 2.2 m: 2.4 x 2.0 m の守衛所
            self.fit_log.append("main gate island: %.2f m from the lanes at its widest point" % room)
            if room > 1.5:
                # 1.56 m measured: a 1.8 x 1.6 m booth, 0.15 m roof overhang (corner reach
                # 1.42 m) / 実測 1.56 m: 1.8 x 1.6 m の守衛所（屋根の出 0.15 m）
                from shapely.ops import nearest_points
                nr = nearest_points(spot, road_only)[1]
                ang = math.atan2(nr.y - spot.y, nr.x - spot.x)
                big = room > 2.05
                add((spot.x, spot.y), "guard", self.guard_house, (spot.x, spot.y), ang, w=2.4 if big else 1.8,
                    d=2.0 if big else 1.6, ov=0.3 if big else 0.15, guard_=road_only)
        for line in self.fit_log:
            if line.startswith("main gate"):
                print("tp: " + line)
        for nm, n in self.drop_log:
            print("tp: %s: %d triangles over the road network dropped" % (nm, n))
        self.tris = sum(pm.ntris() for _, pm in parts)
        print("tp: %d blocks, %d triangles (%s)" % (len(self.blocks), self.tris, ", ".join(
            "%s %d" % kv for kv in sorted(self.tri_count.items(), key=lambda kv: -kv[1]))))
        return parts

    # -- wall pieces / 壁の区間 ----------------------------------------------------
    def pieces(self, key, a, b, n, base=0.0):
        """Runs [(u0, u1, z_cover)] of an edge: the top of whatever solid touches
        the edge from outside (Z0 where free). / 辺の区間と、外側で接する立体の高さ。"""
        L = float(np.linalg.norm(b - a))
        t = (b - a) / L
        seg = LineString([tuple(a + n * 0.2), tuple(b + n * 0.2)])
        cand = [i for i in self.tree.query(seg.buffer(0.3)) if self.solids[i][3] != key]
        step = 0.5 if L > 4 else 0.25
        us = np.arange(step / 2, L, step)
        runs = []
        for u in us:
            p = Point(*(a + t * u + n * 0.2))
            hgt = Z0
            for i in cand:
                poly, b0, tp, kk = self.solids[i]
                if b0 <= base + 0.5 and poly.contains(p):
                    hgt = max(hgt, tp)
            u0, u1 = max(0.0, u - step / 2), min(L, u + step / 2)
            if runs and abs(runs[-1][2] - hgt) < 1e-6:
                runs[-1][1] = u1
            else:
                runs.append([u0, u1, hgt])
        return [tuple(r) for r in runs]

    # -- facade languages / 外壁の語彙 --------------------------------------------
    def t_windows(self, f, u0, u1, floors, bay=3.6, wall=WHITE, glass="tp_win", w=1.3, h=1.3, shade=NAVY, cs=None):
        """Stirling's punched square windows, each under a navy sunshade bar (the
        "T" windows; main-gate photo), a navy band per floor.
        四角い窓と紺の庇（T 字の窓）。"""
        for z0, z1 in floors:
            fh = z1 - z0
            oz0 = z0 + max(0.9, fh * 0.32)
            oz1 = min(oz0 + h, z1 - 0.9)
            cc = centres(u0, u1, bay) if cs is None else [c for c in cs if u0 + w / 2 + 0.1 <= c <= u1 - w / 2 - 0.1]
            opens = [(c - w / 2, c + w / 2, oz0, oz1) for c in cc]
            wall_openings(f, wall, u0, u1, z0, z1, opens, 3.0)
            for o in opens:
                opening(f, *o, wall, glass, depth=0.2, sill=False)
                cu = (o[0] + o[1]) / 2
                sw = min(bay * 0.36, cu - u0, u1 - cu)
                f.block(shade, cu - sw, cu + sw, 0.0, 0.45, oz1 + 0.25, oz1 + 0.55)

    def ribbons(self, f, u0, u1, floors, wall=WHITE, glass="tp_ribbon", shade=GREY, sh_d=0.8, hh=1.6):
        """Ribbon windows (one opening per floor, the glass repeats every 1.8 m)
        under a grey sunshade ledge (wings, Blk 28). / 連窓と灰色の庇。"""
        if u1 - u0 < 2.4:
            for z0, z1 in floors:
                f.face(wall, u0, u1, z0, z1, 0.0, 3.0)
            return
        for z0, z1 in floors:
            fh = z1 - z0
            oz0 = z0 + max(0.8, min(1.0, fh * 0.25))
            oz1 = min(oz0 + hh, z1 - 0.7)
            o = (u0 + 0.6, u1 - 0.6, oz0, oz1)
            wall_openings(f, wall, u0, u1, z0, z1, [o], 3.0)
            opening(f, *o, wall, glass, depth=0.15, tu=1.8, sill=False)
            if sh_d > 0:
                f.block(shade, u0 + 0.3, u1 - 0.3, 0.0, sh_d, oz1 + 0.15, oz1 + 0.3)

    @staticmethod
    def floors(zb, zt, n):
        n = max(1, int(n))
        fh = (zt - zb) / n
        return [(zb + i * fh, zb + (i + 1) * fh) for i in range(n)]

    def walls(self, m, key, poly, base, top, style, storeys, **kw):
        """Every edge of a block: facade where free, plain wall above a lower
        neighbour, nothing where covered. / 全ての辺: 空いている所に外壁。"""
        pieces_drawn = 0
        for a, b, L, n in B.edges_of(poly):
            f = B.Facade(m, a, b, n)
            for u0, u1, zc in self.pieces(key, a, b, n, base):
                if zc >= top - 0.05:
                    continue
                if zc > max(base, Z0) + 0.05:
                    # above a lower neighbour: plain wall / 低い隣の上は無地の壁
                    f.face(WHITE, u0, u1, zc, top, 0.0, 3.0)
                    continue
                style(f, u0, u1, L, base, top, storeys, n=n, **kw)
                pieces_drawn += 1
        return pieces_drawn

    # styles: f, u0, u1, L, base, top, storeys / 外壁の種類
    def st_ribbon(self, f, u0, u1, L, zb, zt, st, n=None, mural=False, wall=WHITE, glass="tp_ribbon"):
        fl = self.floors(zb, zt - 1.0, st)
        if mural and zb < 1.0:
            f.face("tp_mural", u0, u1, zb, fl[0][1], 0.02, 1.0)
            f.face(wall, u0, u1, zb, fl[0][1], 0.0, 3.0)
            fl = fl[1:]
        self.ribbons(f, u0, u1, fl, wall=wall, glass=glass)
        f.face(wall, u0, u1, zt - 1.0, zt, 0.0, 3.0)
        if zb > 1.0:
            # a raised block: its soffit / 持ち上げた棟の下面
            f.hplane(wall, u0, u1, -0.01, 0.0, zb, up=False)

    def st_t(self, f, u0, u1, L, zb, zt, st, n=None, red=False, vents=False):
        fl = self.floors(zb, zt - 0.8, st)
        if vents and self.is_end(f):
            # the slab end walls: blank white, louvred grey vent panels near the top,
            # a stair slot (main-gate photo) / 妻壁: 無地、上部に灰色のガラリ、階段の縦長窓
            f.face(WHITE, u0, u1, zb, zt, 0.0, 3.0)
            cu = (u0 + u1) / 2
            if u1 - u0 > 6.0:
                for z0, z1 in fl[-3:]:
                    f.block("tp_louvre", cu - 1.6, cu + 1.6, 0.0, 0.12, z0 + 0.6, z1 - 0.6)
                f.block("tp_slot", u0 + 0.9, u0 + 1.7, -0.25, 0.0, zb + 4.0, zt - 2.0, top=False, bottom=False)
            if red:
                for c in centres(u0, u1, 4.5, 1.2)[:2]:
                    f.block("tp_red", c - 1.0, c + 1.0, 0.0, 0.08, zb, zb + 3.2, bottom=False)
            return
        if red and zb < 1.0:
            f.face(WHITE, u0, u1, zb, fl[0][1], 0.0, 3.0)
            for c in centres(u0, u1, 9.0, 2.0):
                f.block("tp_red", c - 1.2, c + 1.2, 0.0, 0.08, zb, zb + 3.2, bottom=False)
            fl = fl[1:]
        self.t_windows(f, u0, u1, fl)
        f.face(WHITE, u0, u1, zt - 0.8, zt, 0.0, 3.0)

    def st_band(self, f, u0, u1, L, zb, zt, st, n=None, wall=WHITE, glass="tp_ribbon"):
        fl = self.floors(zb, zt - 0.6, st)
        self.ribbons(f, u0, u1, fl, wall=wall, glass=glass, shade=NAVY, sh_d=0.3, hh=1.3)
        f.face(wall, u0, u1, zt - 0.6, zt, 0.0, 3.0)

    def st_plain(self, f, u0, u1, L, zb, zt, st, n=None, wall=GREY):
        f.face(wall, u0, u1, zb, zt, 0.0, 3.0)
        if zt - zb > 2.6 and u1 - u0 > 3.0:
            c = (u0 + u1) / 2
            o = (c - 0.6, c + 0.6, zb, zb + 2.2)
            f.face("tp_door", o[0], o[1], o[2], o[3], 0.02, 1.0)

    # -- blocks / 棟 --------------------------------------------------------------
    def block(self, m, k, b):
        kind = b["kind"]
        fn = getattr(self, "b_" + kind, None)
        if fn is None:
            fn = self.b_school
        fn(m, k, b)

    def b_spine(self, m, k, b):
        p, top = b["poly"], b["top"]
        st = b.get("storeys", 6)
        self.walls(m, k, p, 0.0, top, self.st_ribbon, st, mural=b.get("blk") == "28")
        ridge = max(b.get("ridge") or top + 2.5, top + 1.5)
        ang = self.wing_angle(p) if b.get("blk") not in ("28", "26", "4", "8") else None
        self.pitched(m, p, top, ridge, ang=ang)

    def pitched(self, m, p, top, ridge, ang=None, ov=0.6, kmin=0.8, fallback="mansard"):
        """A gable roof on the largest rectangle inside the footprint (a road-cut
        footprint keeps its roof off the road), flat roof on what is left; a
        mansard (or flat) roof when no rectangle covers kmin of it.
        外形に収まる長方形の切妻屋根（残りは陸屋根）。"""
        r = inner_rect(p, ang)
        rp = Polygon([tuple(q) for q in r]) if r is not None else None
        if rp is None or rp.area < kmin * p.area:
            if fallback == "flat" or not mansard(m, p, top, top + 2.0, 3.0, ROOF, top="roof_metal"):
                self.flat_roof(m, p, top)
            return
        a, b_, c, d = r
        ax = (b_ - a) / np.linalg.norm(b_ - a)
        gable_at(m, rp, top, ridge, ROOF, WHITE, ang=math.atan2(ax[1], ax[0]), ov=ov)
        for g in polys(p.difference(rp.buffer(0.02))):
            if g.area > 2.0:
                self.flat_roof(m, g, top, parapet=0.5)

    def wing_angle(self, p):
        """The spine ridge runs along its wing (west 151 deg, east 163 deg from
        +x; tiles). / 背骨の棟は翼の方向。"""
        return math.radians(151.0 if p.centroid.x < -1560 + self.dx() else 163.0)

    def dx(self):
        # the real frame's x -> this frame's x (crop builds) / 実座標から作業座標へ
        return self.X((0.0, 0.0))[0]

    @staticmethod
    def rectish(p, k=0.86):
        mr = p.minimum_rotated_rectangle
        return mr.area > 0 and p.area / mr.area > k

    def is_end(self, f):
        """A slab's end wall: its normal runs across the wing (the slabs stand
        square to their wing; main-gate photo, Ave 1 SV). / 板状棟の妻壁（翼と直角）。"""
        ang = self.wing_angle(Point(*f.a))
        perp = np.array([-math.sin(ang), math.cos(ang)])
        return abs(float(f.n @ perp)) > 0.75

    def b_slab(self, m, k, b):
        p, top = b["poly"], b["top"]
        self.walls(m, k, p, 0.0, top, self.st_t, 8, red=True, vents=True)
        self.pitched(m, p, top, top + 2.2, ov=0.0)
        # the curved tops of the end walls (Ave 1 SV h207, main-gate photo): a
        # white parapet rising in a quarter curve over the gable / 妻壁の曲線の頂部
        for a, bb, L, n in B.edges_of(p):
            f = B.Facade(m, a, bb, n)
            if L > 6.0 and self.is_end(f):
                curved_parapet(m, f, L, top, 3.4)

    def b_xblock(self, m, k, b):
        p = b["poly"]
        self.walls(m, k, p, 0.0, b["top"], self.st_band, 2)
        self.flat_roof(m, p, b["top"], mat="tp_roofwhite", parapet=0.7)
        u = b.get("upper_poly")
        if u is None:
            return
        self.walls(m, k + "_up", u, b["top"], b["utop"], self.st_band, 1)
        if not mansard(m, u, b["utop"], b["ridge"] - 0.6, 3.2, ROOF, top=ROOF):
            self.flat_roof(m, u, b["utop"])
        # the round lightwell lantern in the middle (tiles) / 中央の丸い採光塔
        c = u.centroid
        if u.buffer(-3.0).contains(c):
            drum(m, "tp_curtain", (c.x, c.y), 2.6, b["ridge"] - 0.8, b["ridge"] + 0.6, seg=12)
            frustum(m, ROOF, (c.x, c.y), 3.0, 0.4, b["ridge"] + 0.6, b["ridge"] + 1.6, seg=12)

    def b_arm(self, m, k, b):
        p, top = b["poly"], b["top"]
        dr = b.get("drum")
        body = p
        if dr is not None:
            body = p.difference(Point(dr[0]).buffer(dr[1] * 0.97, 24))
            body = max(polys(body), key=lambda q: q.area) if polys(body) else p
        st = b.get("storeys", 6)
        self.walls(m, k, body, 0.0, top, self.st_ribbon, st)
        ridge = max(b.get("ridge") or top + 2.5, top + 1.5)
        self.pitched(m, body, top, ridge, kmin=0.75)
        if dr is not None:
            (cx, cy), r = dr
            zt = ridge + 1.2
            drum(m, WHITE, (cx, cy), r, 0.0, zt, seg=24, uvs=3.0)
            # vertical glass slots round the stair drum / 階段塔の縦長窓
            for j in range(6):
                a0 = 2 * math.pi * (j + 0.25) / 6
                a1 = a0 + 0.18
                p0 = (cx + (r + 0.05) * math.cos(a0), cy + (r + 0.05) * math.sin(a0))
                p1 = (cx + (r + 0.05) * math.cos(a1), cy + (r + 0.05) * math.sin(a1))
                m.wall("tp_curtain", p0, p1, 3.0, zt - 2.0)
            annulus(m, FASCIA, (cx, cy), r - 0.4, r + 0.35, zt, up=True)
            drum(m, FASCIA, (cx, cy), r + 0.35, zt - 0.5, zt, seg=24)
            dome(m, "tp_dome", (cx, cy), r - 0.4, zt, max(2.0, r * 0.45))

    def b_round(self, m, k, b):
        """Blk 41/43/45: round 10-storey blocks, white balcony bands with glass
        railings, dark window recesses, the green accent strip (sv west gate).
        円形の 10 階棟（バルコニーの帯、緑の縦帯）。"""
        c, r, top = b["c"], b["r"], b["top"]
        fh = (top - 1.2) / 10
        drum(m, "tp_balc", c, r - 1.2, 0.0, top - 1.2, seg=24, uvs=fh)
        drum(m, WHITE, c, r, top - 1.2, top, seg=24, uvs=3.0)
        for j in range(1, 10):
            z = j * fh
            annulus(m, WHITE, c, r - 1.2, r, z + 0.25, up=True)
            annulus(m, WHITE, c, r - 1.2, r, z, up=False)
            drum(m, WHITE, c, r, z, z + 0.25, seg=24)
            drum(m, "glass_rail", c, r - 0.05, z + 0.25, z + 1.2, seg=24)
        annulus(m, WHITE, c, r - 1.2, r, top - 1.2, up=False)
        m.flat("roof_concrete", Point(c).buffer(r - 0.3, 6), top - 0.2, 3.0)
        drum(m, WHITE, c, r - 0.3, top - 0.2, top, seg=24)
        # the cross-shaped core on the roof (tiles) / 屋上の十字形のコア
        for ang in (0.0, math.pi / 2):
            m.box(WHITE, c[0], c[1], top, top + 3.5, r * 1.1, 4.0, ang + 0.4)
        # green strip facing Ave 1 (ENE) / Ave 1 側の緑の縦帯
        a0 = math.radians(90 - 72)
        p = (c[0] + (r + 0.1) * math.cos(a0), c[1] + (r + 0.1) * math.sin(a0))
        m.box("tp_green", p[0], p[1], 0.0, top - 0.6, 0.5, 2.6, a0 + math.pi / 2)

    def b_school(self, m, k, b):
        p, top = b["poly"], b["top"]
        st = max(1, b.get("storeys") or int(round(top / 4.3)))
        self.walls(m, k, p, b.get("base", 0.0), top, self.st_ribbon, st, mural=b.get("blk") == "28")
        ridge = b.get("ridge") or top
        if b.get("roof", "pitched") != "flat" and self.rectish(p, 0.9) and ridge - top > 1.0:
            self.pitched(m, p, top, ridge, kmin=0.85, fallback="flat")
        else:
            self.flat_roof(m, p, top)

    def b_box(self, m, k, b):
        p, top = b["poly"], b["top"]
        st = max(1, b.get("storeys") or int(round(top / 4.2)))
        self.walls(m, k, p, 0.0, top, self.st_band, st)
        self.flat_roof(m, p, top)

    def b_shed(self, m, k, b):
        p, top = b["poly"], max(2.8, b["top"])
        self.walls(m, k, p, 0.0, top, self.st_plain, 1)
        self.flat_roof(m, p, top, parapet=0.3, mat="roof_metal")

    def b_glassbox(self, m, k, b):
        p, top = b["poly"], b["top"]
        self.walls(m, k, p, 0.0, top, self.st_curtain, 1)
        self.flat_roof(m, p, top, parapet=0.4)

    def st_curtain(self, f, u0, u1, L, zb, zt, st, n=None, glass="tp_curtain"):
        f.face(WHITE, u0, u1, zt - 0.6, zt, 0.0, 3.0)
        f.m.quad(glass, f.P(u0, 0.0, zb), f.P(u1, 0.0, zb), f.P(u1, 0.0, zt - 0.6), f.P(u0, 0.0, zt - 0.6),
                 (u0 / 1.8, zb / 3.6), (u1 / 1.8, zb / 3.6), (u1 / 1.8, (zt - 0.6) / 3.6), (u0 / 1.8, (zt - 0.6) / 3.6))

    def b_link(self, m, k, b):
        """A link bridge / raised block: glazed band between white fascias, its
        soffit; cut where it would stand over a drive. / 連絡橋（車路の上は切る）。"""
        p, z0, z1 = b["poly"], b["base"], b["top"]
        self.walls(m, k, p, z0, z1, self.st_ribbon, max(1, int(round((z1 - z0) / 4.2))))
        m.flat(WHITE, p, z0, 2.0, up=False)
        self.flat_roof(m, p, z1, parapet=0.5)

    def b_green(self, m, k, b):
        p, top = b["poly"], b["top"]
        st = max(1, b.get("storeys") or int(round(top / 4.5)))
        self.walls(m, k, p, 0.0, top, self.st_band, max(st, int(round(top / 4.5))))
        self.flat_roof(m, p, top, mat="tp_greenroof", parapet=1.0)
        c = p.centroid
        if p.buffer(-6).contains(c):
            m.box(GREY, c.x, c.y, top, top + 3.0, 10.0, 6.0, 0.3)

    def b_courts_roof(self, m, k, b):
        """Blk 26A: the open hall with tennis courts on the roof (red/green), a
        fence round them. / 屋上にテニスコートのある体育館。"""
        p, top = b["poly"], b["top"]
        self.walls(m, k, p, 0.0, top, self.st_band, 3)
        self.flat_roof(m, p, top, mat="tp_courtred", parapet=1.0)
        a, bb, c, d = gable_rect(p)
        L, W = np.linalg.norm(bb - a), np.linalg.norm(d - a)
        ax, sd = (bb - a) / L, (d - a) / W
        n = max(1, int((L - 4) // 19))
        for i in range(n):
            cu = (i + 0.5) * L / n
            ctr = a + ax * cu + sd * W / 2
            if not p.buffer(-1.5).contains(Point(*ctr)):
                continue
            hl, hw = min(L / n / 2 - 1.5, 9.0), min(W / 2 - 2.0, 17.0)
            if hl < 5 or hw < 8:
                continue
            q = [ctr + sd * hw - ax * hl, ctr - sd * hw - ax * hl, ctr - sd * hw + ax * hl, ctr + sd * hw + ax * hl]
            m.quad("court_tennis", *[(v[0], v[1], top + 0.03) for v in q], (0, 0), (1, 0), (1, 1), (0, 1))
        for a_, b_, L_, n_ in B.edges_of(p.buffer(-0.4, join_style="mitre")):
            two = np.array([n_[0], n_[1]]) * 0.004
            m.wall("tp_fence", tuple(a_ + two), tuple(b_ + two), top, top + 3.5, 0.0, 1.0)
            m.wall("tp_fence", tuple(b_ - two), tuple(a_ - two), top, top + 3.5, 0.0, 1.0)

    def b_sawtooth(self, m, k, b):
        """Blk 29: the sports hall / grandstand with a grey sawtooth roof, yellow
        and purple end bays (tiles). / のこぎり屋根の体育館。"""
        p, top, ridge = b["poly"], b["top"], b.get("ridge") or b["top"] + 2.6
        self.walls(m, k, p, 0.0, top, self.st_band, 2)
        r = inner_rect(p)
        if r is None:
            return self.flat_roof(m, p, top)
        a, bb, c, d = r
        rest = p.difference(Polygon([tuple(a), tuple(bb), tuple(c), tuple(d)]).buffer(0.02))
        for g in polys(rest):
            if g.area > 2.0:
                self.flat_roof(m, g, top, parapet=0.5)
        L, W = float(np.linalg.norm(bb - a)), float(np.linalg.norm(d - a))
        ax, sd = (bb - a) / L, (d - a) / W
        n = max(2, int(round(W / 7.5)))
        w = W / n
        Q = lambda u, v, z: (a[0] + ax[0] * u + sd[0] * v, a[1] + ax[1] * u + sd[1] * v, z)
        for i in range(n):
            v0, v1 = i * w, (i + 1) * w
            mat = "tp_yellow" if i in (0, n - 1) else ("tp_purple" if i in (1, n - 2) else "roof_metal")
            # the slope rises from v0 to v1, the glazed face drops at v1 / v0 から v1 へ上る斜面
            m.quad(mat, Q(0, v0, top), Q(L, v0, top), Q(L, v1, ridge), Q(0, v1, ridge))
            m.quad("tp_curtain", Q(L, v1, top), Q(0, v1, top), Q(0, v1, ridge), Q(L, v1, ridge),
                   (0, 0), (L / 1.8, 0), (L / 1.8, 1), (0, 1))
            m.tris(WHITE, [Q(0, v1, top), Q(0, v0, top), Q(0, v1, ridge)], (0, 0, 1), [(0, 0), (1, 0), (1, 1)], [0, 1, 2])
            m.tris(WHITE, [Q(L, v0, top), Q(L, v1, top), Q(L, v1, ridge)], (0, 0, 1), [(0, 0), (1, 0), (1, 1)], [0, 1, 2])

    def b_fan(self, m, k, b):
        """Blk 34, the convention centre: fan-shaped hall under one big sloping
        grey metal roof (high to Ave 1), the curved orange entrance canopy on
        round white columns. / 扇形の会議場（大きな片流れ屋根、橙の庇）。"""
        p, z_lo, z_hi = b["poly"], b["top"], b.get("ridge") or b["top"] + 10.0
        br = math.radians(30.0)
        d = np.array([math.sin(br), math.cos(br)])
        P = np.asarray(p.exterior.coords)
        s = P @ d
        s0, s1 = s.min(), s.max()
        zf = lambda X, Y: z_lo + (z_hi - z_lo) * ((np.asarray(X) * d[0] + np.asarray(Y) * d[1]) - s0) / max(s1 - s0, 1e-6)
        for a, bb, L, n in B.edges_of(p):
            za, zb_ = float(zf(a[0], a[1])), float(zf(bb[0], bb[1]))
            f = B.Facade(m, a, bb, n)
            wall_top(m, WHITE, tuple(a), tuple(bb), 0.0, za, zb_, 3.0)
            low = min(za, zb_)
            if L > 6:
                self.ribbons(f, 0.0, L, [(4.5, 9.0)], glass="tp_curtain", shade=GREY, sh_d=0.6, hh=3.0)
                f.face(WHITE, 0.0, L, 0.0, 4.5, 0.01, 3.0)
            band(m, FASCIA, tuple(a + n * 0.6), tuple(bb + n * 0.6), za + 0.1, zb_ + 0.1, 0.7)
        roof = p.buffer(0.6, join_style="mitre")
        m.flat("roof_metal", roof, 0.0, 3.0, zf=lambda X, Y: zf(X, Y) + 0.1)
        m.flat("concrete", roof.difference(p), 0.0, 3.0, up=False, zf=lambda X, Y: zf(X, Y) - 0.5)
        # the entrance canopy on the low side, clear of the roads / 低い側の庇（道路から離す）
        lowe = min(B.edges_of(p), key=lambda e: float((e[0] + e[1]) @ d))
        a, bb, L, n = lowe
        if L > 10:
            arc = [a + (bb - a) * t + n * (2.0 + 5.0 * math.sin(math.pi * t)) for t in np.linspace(0.15, 0.85, 9)]
            can = Polygon([tuple(a + (bb - a) * 0.15)] + [tuple(q) for q in arc] + [tuple(a + (bb - a) * 0.85)])
            can = can.difference(self.no_go.buffer(ROOM)).difference(p)
            for g in polys(can):
                m.flat("tp_orange", g, 5.0, 2.0, up=False)
                m.flat(WHITE, g, 5.5, 2.0)
                m.ring_walls(WHITE, g.exterior, 5.0, 5.5)
            for q in arc[1:-1:2]:
                if can.buffer(-0.3).contains(Point(*(q - n * 1.0))):
                    m.cylinder(WHITE, *(q - n * 1.0), 0.0, 5.0, 0.3, seg=8, top=False)

    def b_hipped(self, m, k, b):
        """Blk 30: layered white floors with balustraded balcony bands, a wide hip
        roof with the orange-ochre fascia (2020 photo). / 腰折れ屋根と黄土色の鼻隠し。"""
        p, top, ridge = b["poly"], b["top"], b.get("ridge") or b["top"] + 2.5
        self.walls(m, k, p, 0.0, top, self.st_balcony, 4)
        q = p.buffer(1.2, join_style="mitre")
        if not mansard(m, q, top, ridge, 3.5, ROOF, top="roof_metal"):
            self.flat_roof(m, p, top)
        m.flat("concrete", q.difference(p), top, 2.0, up=False)
        m.ring_walls("tp_ochre", q.exterior, top - 0.8, top + 0.02)

    def st_balcony(self, f, u0, u1, L, zb, zt, st, n=None):
        fl = self.floors(zb, zt - 0.5, st)
        self.ribbons(f, u0, u1, fl, glass="tp_ribbon", shade=WHITE, sh_d=0.9, hh=1.8)
        for z0, z1 in fl[1:]:
            f.block(WHITE, u0 + 0.3, u1 - 0.3, 0.75, 0.9, z0 + 0.15, z0 + 1.05, bottom=False)
        f.face(WHITE, u0, u1, zt - 0.5, zt, 0.0, 3.0)

    def b_pavilion(self, m, k, b):
        p, top, ridge = b["poly"], b["top"], b.get("ridge") or b["top"] + 1.5
        self.walls(m, k, p, 0.0, top, self.st_band, 1)
        q = p.buffer(0.8, join_style="mitre")
        if not mansard(m, q, top, max(ridge, top + 1.2), 3.0, ROOF):
            self.flat_roof(m, p, top)
        m.flat("concrete", q.difference(p), top, 2.0, up=False)

    def b_culinary(self, m, k, b):
        p, top, ridge = b["poly"], b["top"], b.get("ridge") or b["top"] + 3.0
        self.walls(m, k, p, 0.0, top, self.st_band, 3)
        if not mansard(m, p, top, ridge, 3.5, ROOF, top="roof_metal"):
            self.flat_roof(m, p, top)

    def b_sit(self, m, k, b):
        """SIT@TP (Blk 29A): a beige render base, the dark cantilevered box with
        vertical green-wall fins, a white spiral stair (2016 photo).
        SIT: ベージュの基壇、緑の縦フィンの暗い箱、白い螺旋階段。"""
        p, top = b["poly"], b["top"]
        z1 = 8.0
        self.walls(m, k, p, 0.0, z1, self.st_band, 2, wall="tp_beige")
        q = p.buffer(1.0, join_style="mitre").difference(self.ng)
        q = max(polys(q), key=lambda g: g.area) if polys(q) else p
        q = orient(q.buffer(0), 1.0)
        for a, bb, L, n in B.edges_of(q):
            f = B.Facade(m, a, bb, n)
            f.face("tp_darkbox", 0.0, L, z1, top, 0.0, 3.0)
            for c in centres(0.0, L, 2.4, 0.6):
                f.block("tp_green", c - 0.25, c + 0.25, 0.0, 0.5, z1 + 0.5, top - 0.5)
        m.flat("tp_darkbox", q.difference(p), z1, 2.0, up=False)
        self.flat_roof(m, q, top, parapet=0.6)
        c = np.asarray(p.exterior.coords)[0]
        cc = Point(*c)
        if self.no_go.distance(cc) > GAP + 3.0:
            m.cylinder(WHITE, c[0], c[1], 0.0, top - 4.0, 2.2, seg=12)

    def b_eastwing(self, m, k, b):
        """Blk 1A (2015): the curvy podium (lime/green striped ground floor, green
        roof, white railing) and three tower pairs: blue-grey glazing, white
        curved-top frames, grey perforated cores (photo_theplan_*).
        1A: 曲面の基壇（緑の屋上）と 3 組の塔（白い曲線の枠、灰色の穴あきパネルのコア）。"""
        p, top = b["poly"], b["top"]
        tw = b.get("tower_polys", [])
        # the podium runs under the towers (they stand on it) / 塔は基壇の上に立つ
        pod = orient(p.buffer(0), 1.0)
        twu = unary_union(tw) if tw else Polygon()
        for a, bb, L, n in B.edges_of(pod):
            f = B.Facade(m, a, bb, n)
            for u0, u1, zc in self.pieces(k, a, bb, n):
                if zc >= top - 0.05:
                    continue
                tiled_lime(f, u0, u1, Z0, 4.0)
                self.ribbons(f, u0, u1, self.floors(4.0, top - 0.8, 2), glass="tp_glass1a", shade=WHITE, sh_d=0.5)
                f.face(WHITE, u0, u1, top - 0.8, top, 0.0, 3.0)
        m.flat("tp_greenroof", pod.buffer(-0.3, join_style="mitre").difference(twu), top + 0.02, 3.0)
        for a, bb, L, n in B.edges_of(pod):
            rail_run(m, "railing", a - n * 0.3, bb - n * 0.3, top, 1.1)
        m.flat(WHITE, pod.difference(pod.buffer(-0.3, join_style="mitre")), top + 0.05, 1.0)
        # light wells (dark ellipses on the green roof) / 楕円の光井
        c = pod.centroid
        for dxy in ((-12, 3), (6, -4), (20, 2)):
            e = Point(c.x + dxy[0], c.y + dxy[1]).buffer(1.0, 12)
            from shapely.affinity import scale as _sc
            e = _sc(e, 4.0, 2.2)
            if pod.buffer(-2).contains(e) and not e.intersects(twu.buffer(1.0)):
                m.flat("tp_glassdark", e, top + 0.04, 2.0)
        zt, zf = b["ttop"], b["tframe"]
        wing = math.radians(163.0)
        ax = np.array([math.cos(wing), math.sin(wing)])
        for t in tw:
            r = orient(t.minimum_rotated_rectangle, 1.0)
            q = [np.array(v) for v in list(r.exterior.coords)[:4]]
            e0 = q[1] - q[0]
            if abs(float(e0 @ ax)) / np.linalg.norm(e0) < 0.7:
                q = q[1:] + q[:1]
            a0, a1, a2, a3 = q                     # a0->a1 along the wing / 翼方向
            Lw = float(np.linalg.norm(a1 - a0))
            ux, vx = (a1 - a0) / Lw, (a3 - a0) / np.linalg.norm(a3 - a0)
            Wd = float(np.linalg.norm(a3 - a0))
            for s0, s1, core in ((0.0, 0.4, False), (0.4, 0.6, True), (0.6, 1.0, False)):
                g = Polygon([tuple(a0 + ux * Lw * s0), tuple(a0 + ux * Lw * s1), tuple(a0 + ux * Lw * s1 + vx * Wd),
                             tuple(a0 + ux * Lw * s0 + vx * Wd)])
                g = orient(g.intersection(t).buffer(0), 1.0) if g.intersects(t) else None
                if g is None or g.geom_type != "Polygon" or g.area < 10:
                    continue
                if core:
                    g = orient(g.buffer(-1.0, join_style="mitre"), 1.0)
                    if g.is_empty or g.geom_type != "Polygon":
                        continue
                    for a, bb, L, n in B.edges_of(g):
                        B.Facade(m, a, bb, n).face("tp_perf", 0.0, L, top, zt - 0.5, 0.0, 2.0)
                    self.flat_roof(m, g, zt - 0.5, parapet=0.4)
                    continue
                for a, bb, L, n in B.edges_of(g):
                    f = B.Facade(m, a, bb, n)
                    along = abs(float((bb - a) @ ux)) / L > 0.7
                    if along:
                        # the narrow faces to Ave 1 and back: glass inside a white frame
                        # 幅の狭い面: 白い枠の中のガラス
                        f.m.quad("tp_glass1a", f.P(1.0, 0.0, top), f.P(L - 1.0, 0.0, top), f.P(L - 1.0, 0.0, zt),
                                 f.P(1.0, 0.0, zt), (0, top / 3.6), ((L - 2) / 1.8, top / 3.6),
                                 ((L - 2) / 1.8, zt / 3.6), (0, zt / 3.6))
                        f.face(WHITE, 0.0, 1.0, top, zt, 0.0, 3.0)
                        f.face(WHITE, L - 1.0, L, top, zt, 0.0, 3.0)
                        # the curved-top frame: side bands rising past the roof, one
                        # side higher, joined by an arc / 曲線の頂部を持つ枠
                        for u_a, u_b in ((0.0, 1.0), (L - 1.0, L)):
                            f.block(WHITE, u_a, u_b, 0.0, 1.2, top, zt, top=False, bottom=False)
                        arc = [(u, zt + (zf - zt) * math.sin(math.pi / 2 * (1 - u / L))) for u in np.linspace(0, L, 7)]
                        for (ua, za), (ub, zb_) in zip(arc, arc[1:]):
                            pa, pb = f.P(ua, 1.2, 0)[:2], f.P(ub, 1.2, 0)[:2]
                            wall_top(m, WHITE, pa, pb, zt - 1.0, za, zb_)
                            pa2, pb2 = f.P(ua, 0.0, 0)[:2], f.P(ub, 0.0, 0)[:2]
                            wall_top(m, WHITE, pb2, pa2, zt - 1.0, zb_, za)
                            m.quad(WHITE, (pa[0], pa[1], za), (pb[0], pb[1], zb_), (pb2[0], pb2[1], zb_),
                                   (pa2[0], pa2[1], za))
                    else:
                        f.m.quad("tp_glass1a", f.P(0.0, 0.0, top), f.P(L, 0.0, top), f.P(L, 0.0, zt),
                                 f.P(0.0, 0.0, zt), (0, top / 3.6), (L / 1.8, top / 3.6), (L / 1.8, zt / 3.6),
                                 (0, zt / 3.6))
                        for z in np.arange(top + 3.6, zt - 0.1, 3.6):
                            f.block(WHITE, 0.0, L, 0.0, 0.25, z - 0.12, z + 0.12)
                self.flat_roof(m, g, zt, parapet=0.5)

    # -- roofs / 屋根 ------------------------------------------------------------
    def flat_roof(self, m, poly, z, parapet=0.9, mat="roof_concrete", cap=WHITE):
        for p in polys(poly):
            inner = p.buffer(-0.25, join_style="mitre")
            m.prism(WHITE, p, z, z + parapet, top=False)
            for q in polys(inner):
                m.ring_walls(WHITE, q.exterior, z, z + parapet, outward=False)
                for r in q.interiors:
                    m.ring_walls(WHITE, r, z, z + parapet, outward=True)
                m.flat(mat, q, z + 0.01, 3.0)
            m.flat(cap, p.difference(inner), z + parapet, 1.0)

    # -- horseshoe / 馬蹄形 --------------------------------------------------------
    def horseshoe(self, m):
        H = self.hs
        O, I = self.hO, self.hI
        n = len(O) - 1
        zd, zi, zo = H["deck"], H["inner_eave"], H["outer_eave"]
        st = H["storey"]
        g0, g1 = H["gateway"]["z0"], H["gateway"]["z1"]
        lower = [(0.0, 4.0), (4.0, zd)]
        upper = [(zd + i * st, zd + (i + 1) * st) for i in range(5)]      # 8 .. 30
        # the outer face rises to the outer eave (OneMap 43.5-44 m): T-window rows to
        # 2 m under it / 外周は外側の軒（44 m）まで: その 2 m 下まで窓の段
        attic = [(zi + i * st, zi + (i + 1) * st) for i in range(int((zo - 2.0 - zi) // st))] or [(zi, zi + 0.01)]
        segs = range(n)
        gseg = self.gate_segs
        cen = self.hcen
        # the dark glazed lower floors facing the lawn, near the gateway (OneMap)
        # 芝生側の門付近の下層は暗いガラス
        mids = [(O[k] + O[k + 1]) / 2 for k in segs]
        dg = {k for k in segs if min(abs(k - j) for j in gseg) <= 4} if gseg else set()

        def chain(P, k, out_ref):
            """Facade of segment k (facing away from / toward the centre) and the
            window / column centres laid out along the whole chain (not per facet).
            区間 k の外壁と、鎖全体に沿って並べた窓・柱の位置。"""
            s = np.concatenate([[0.0], np.cumsum([np.linalg.norm(P[j + 1] - P[j]) for j in range(len(P) - 1)])])
            a, b = P[k], P[k + 1]
            L = float(np.linalg.norm(b - a))
            t = (b - a) / L
            nrm = np.array([t[1], -t[0]])
            flip = float(nrm @ out_ref((a + b) / 2)) < 0
            if flip:
                nrm = -nrm
                a, b = b, a

            def loc(bay, off=0.5):
                g = np.arange(bay * off, s[-1], bay)
                g = g[(g >= s[k]) & (g <= s[k + 1])] - s[k]
                return sorted(L - g if flip else g)
            return B.Facade(m, a, b, nrm), L, loc

        for k in segs:
            if np.linalg.norm(O[k + 1] - O[k]) < 0.2:
                continue
            f, L, loc = chain(O, k, lambda q: q - cen)
            cs = loc(3.6)
            if k in gseg:
                # the gateway: open between g0 and g1 / 門: g0〜g1 は開口
                f.face("tp_glassdark", 0.0, L, Z0, zd, -1.5, 1.0)
                f.face(WHITE, 0.0, L, g1, g1 + 0.4, 0.0, 3.0)
                self.t_windows(f, 0.0, L, [(z0, z1) for z0, z1 in upper if z0 >= g1 - 0.1], cs=cs)
                self.t_windows(f, 0.0, L, attic, cs=cs)
                f.face(WHITE, 0.0, L, attic[-1][1], zo, 0.0, 3.0)
                f.hplane(WHITE, 0.0, L, -1.5, 0.0, zd, up=False)
                f.hplane(WHITE, 0.0, L, -1.5, 0.0, Z0 + 0.01, up=True)
                continue
            if k in dg:
                f.face("tp_glassdark", 0.0, L, Z0, zd, -1.2, 1.0)
                f.hplane(WHITE, 0.0, L, -1.2, 0.0, zd, up=False)
                f.hplane(WHITE, 0.0, L, -1.2, 0.0, Z0 + 0.01, up=True)
                for c in loc(7.2):
                    f.block(WHITE, c - 0.4, c + 0.4, -1.2, 0.0, Z0, zd, top=False, bottom=False)
                for kk, uu in ((k - 1, 0.0), (k + 1, L)):
                    if kk not in dg and kk not in gseg:
                        f.side(WHITE, uu, -1.2, 0.0, Z0, zd, 1 if uu == 0.0 else -1)
            else:
                self.t_windows(f, 0.0, L, lower, cs=cs)
            self.t_windows(f, 0.0, L, upper, cs=cs)
            # the high outer band: an attic row of small windows, white soffit band
            # 外周の高い帯: 小窓の列と白い軒裏帯
            self.t_windows(f, 0.0, L, attic, cs=cs)
            f.face(WHITE, 0.0, L, attic[-1][1], zo, 0.0, 3.0)
        # inner (courtyard) face above the deck / 中庭側（デッキより上）
        for k in segs:
            if np.linalg.norm(I[k + 1] - I[k]) < 0.2:
                continue
            f, L, loc = chain(I, k, lambda q: cen - q)
            cs = loc(3.6)
            if k in gseg:
                f.face(WHITE, 0.0, L, g1, g1 + 0.4, 0.0, 3.0)
                self.t_windows(f, 0.0, L, [(z0, z1) for z0, z1 in upper if z0 >= g1 - 0.1 and z1 <= zi - 4.3],
                               cs=cs)
                self.gallery(f, L, zi - 4.4 if g1 <= zi - 8.8 else None, zi, posts=loc(7.2, 0.0))
                continue
            # the arcade: glazed, recessed 3 m behind square columns, the yellow wall
            # and the dark green canopy (2026 photo, photosphere) / 列柱廊
            z0, z1 = upper[0]
            f.face("tp_yellow", 0.0, L, z0, z1 - 0.6, -3.0, 2.0)
            f.face("tp_shopglass", 0.0, L, z0 + 0.1, z1 - 1.2, -2.95, 1.8)
            f.hplane(WHITE, 0.0, L, -3.0, 0.0, z1 - 0.6, up=False)
            f.face(WHITE, 0.0, L, z1 - 0.6, z1, 0.0, 3.0)
            for c in loc(7.2, 0.0):
                f.block(WHITE, max(0.0, c - 0.35), min(L, c + 0.35), -0.7, 0.0, z0, z1 - 0.6, top=False,
                        bottom=False)
            # glazing mullions, the curved dark green canopy in three facets
            # ガラスの方立てと 3 面に折れた濃緑の曲面庇
            for c in loc(1.8, 0.25):
                f.block(WHITE, max(0.0, c - 0.04), min(L, c + 0.04), -2.95, -2.85, z0 + 0.1, z1 - 1.2,
                        top=False, bottom=False)
            for (w0, w1, za, zb) in ((0.0, 0.8, z0 + 3.5, z0 + 3.35), (0.8, 1.6, z0 + 3.35, z0 + 3.1),
                                     (1.6, 2.3, z0 + 3.1, z0 + 2.75)):
                pa, pb = np.array(f.P(0.0, w0, 0)[:2]), np.array(f.P(0.0, w1, 0)[:2])
                qa, qb = np.array(f.P(L, w0, 0)[:2]), np.array(f.P(L, w1, 0)[:2])
                top = [(pa[0], pa[1], za), (qa[0], qa[1], za), (qb[0], qb[1], zb), (pb[0], pb[1], zb)]
                nq = np.cross(np.subtract(top[1], top[0]), np.subtract(top[3], top[0]))
                m.quad("tp_canopy", *(top if nq[2] > 0 else top[::-1]))
                bot = [(v[0], v[1], v[2] - 0.12) for v in top]
                nq = np.cross(np.subtract(bot[1], bot[0]), np.subtract(bot[3], bot[0]))
                m.quad("tp_soffit", *(bot if nq[2] < 0 else bot[::-1]))
            f.face("tp_canopy", 0.0, L, z0 + 2.63, z0 + 2.75, 2.3)
            self.t_windows(f, 0.0, L, upper[1:3], cs=cs)
            self.gallery(f, L, upper[3][0], upper[3][1], posts=loc(7.2, 0.0))
            self.t_windows(f, 0.0, L, [upper[4]], cs=cs)
        # the gateway's jambs and soffit / 門の側面と天井
        if gseg:
            ks = sorted(gseg)
            for k, side in ((ks[0], 1), (ks[-1] + 1, -1)):
                a, b = O[k], I[k]
                if side < 0:
                    a, b = b, a
                m.wall(WHITE, tuple(b), tuple(a), g0, g1)
            for k in ks:
                q = [O[k], O[k + 1], I[k + 1], I[k]]
                pq = orient(Polygon([tuple(v) for v in q]).buffer(0), 1.0)
                if pq.geom_type == "Polygon":
                    m.flat("tp_soffit", pq, g1, 1.0, up=False)
                    m.flat("tp_paver", pq, g0 + 0.02, 2.0)
        # arm ends: the end wall's top follows the roof sweep (main-gate photo); the
        # roof's end cap carries the navy band / 腕の端: 壁の上端は屋根の反りに沿う
        ovo, ovi = 0.8, 1.2
        zr = lambda t, D: zi + (zo - zi) * sweep((ovi + t * D) / (D + ovi + ovo))
        for k in (0, n):
            a, b = I[k], O[k]                       # inner -> outer / 内から外
            D = float(np.linalg.norm(b - a))
            tt = (O[1] - O[0]) if k == 0 else (O[n] - O[n - 1])
            out = -tt if k == 0 else tt
            ts = np.linspace(0.0, 1.0, 9)
            pts = [a + (b - a) * t for t in ts]
            zs = [zr(t, D) for t in ts]
            nn = np.array([(b - a)[1], -(b - a)[0]])
            fwd = float(nn @ out) >= 0
            for j in range(8):
                p0, p1, z_a, z_b = pts[j], pts[j + 1], zs[j], zs[j + 1]
                if fwd:
                    wall_top(m, WHITE, tuple(p0), tuple(p1), 0.0, z_a, z_b, 3.0)
                else:
                    wall_top(m, WHITE, tuple(p1), tuple(p0), 0.0, z_b, z_a, 3.0)
            # vertical stair slots and a white soffit band under the cap / 縦長の窓と白い帯
            p0, p1 = (a, b) if fwd else (b, a)
            nv = np.array([(p1 - p0)[1], -(p1 - p0)[0]]) / D
            f = B.Facade(m, p0, p1, nv)
            for c in centres(0.0, D, 3.6, 2.0)[1:-1]:
                f.block("tp_slot", c - 0.3, c + 0.3, -0.25, 0.0, 2.0, zi - 4.0, top=False, bottom=False)
            for j in range(8):
                q0, q1 = pts[j] + nv * 0.02, pts[j + 1] + nv * 0.02
                if fwd:
                    band(m, "tp_soffit", tuple(q0), tuple(q1), zs[j] - 1.15, zs[j + 1] - 1.15, 0.7)
                else:
                    band(m, "tp_soffit", tuple(q1), tuple(q0), zs[j + 1] - 1.15, zs[j] - 1.15, 0.7)
        self.swept_roof(m, O, I, zi, zo)
        # low fan boxes on the arm ends' roof sweep, 7 fans each (OneMap close h225)
        # 腕の端の屋根の上の低いファン箱（7 基）
        for k0, k1 in ((1, 4), (n - 4, n - 1)):
            sa, sb = 0.38, 0.62
            q = [I[k0] + (O[k0] - I[k0]) * sa, I[k1] + (O[k1] - I[k1]) * sa,
                 I[k1] + (O[k1] - I[k1]) * sb, I[k0] + (O[k0] - I[k0]) * sb]
            pq = orient(Polygon([tuple(v) for v in q]).buffer(0), 1.0)
            if pq.is_empty or pq.geom_type != "Polygon":
                continue
            D = float(np.linalg.norm(O[k0] - I[k0]))
            zb_, zt = zr(sa, D) - 0.3, zr(sb, D) + 0.9
            m.prism("tp_louvre", pq, zb_, zt, top=False)
            m.flat(GREY, pq, zt, 2.0)
            c0 = I[k0] + (O[k0] - I[k0]) * 0.5
            c1 = I[k1] + (O[k1] - I[k1]) * 0.5
            for j in range(7):
                q_ = c0 + (c1 - c0) * (j + 0.5) / 7
                if pq.buffer(-0.6).contains(Point(*q_)):
                    m.cylinder("tp_fan", q_[0], q_[1], zt, zt + 0.35, 0.7, seg=10)

    def gallery(self, f, L, z0, z1, posts=None):
        """The recessed level-4 gallery: glass balustrade on a green-grey
        spandrel band, the back wall with doors and windows 2.4 m in.
        4 階の奥まった回廊（ガラスの手すり、緑灰色の腰壁）。"""
        if z0 is None:
            f.face(WHITE, 0.0, L, z1 - 4.4, z1, 0.0, 3.0)
            return
        f.face("tp_spandrel", 0.0, L, z0, z0 + 0.9, 0.0, 2.0)
        f.hplane("tp_paver", 0.0, L, -2.4, 0.0, z0 + 0.9, up=True)
        f.face("tp_gallery", 0.0, L, z0 + 0.9, z1 - 0.5, -2.4, 3.6)
        f.hplane(WHITE, 0.0, L, -2.4, 0.0, z1 - 0.5, up=False)
        f.face(WHITE, 0.0, L, z1 - 0.5, z1, 0.0, 3.0)
        a, b = np.array(f.P(0.0, -0.1, 0)[:2]), np.array(f.P(L, -0.1, 0)[:2])
        rail_run(f.m, "railing", a, b, z0 + 0.9, 1.1, glass=True)
        for c in (centres(0.0, L, 7.2, 0.0) if posts is None else posts):
            f.block(WHITE, max(0.0, c - 0.3), min(L, c + 0.3), -0.6, 0.0, z0 + 0.9, z1 - 0.5, top=False,
                    bottom=False)

    def swept_roof(self, m, O, I, zi, zo, k_sub=6):
        """The swept roof: from the inner eave up a concave curve to the outer
        edge, standing seams in the texture, a navy fascia over a white soffit
        band on both eaves (courtyard photo). / 反りのある屋根（内から外へ上る）。"""
        n = len(O) - 1
        prof = [(s, zi + (zo - zi) * sweep(s)) for s in np.linspace(0, 1, k_sub + 1)]
        ovo, ovi = 0.8, 1.2
        for k in range(n):
            o0, o1, i0, i1 = O[k], O[k + 1], I[k], I[k + 1]
            # eaves stick out: inner toward the courtyard, outer away / 軒の出
            d0, d1 = (i0 - o0) / np.linalg.norm(i0 - o0), (i1 - o1) / np.linalg.norm(i1 - o1)
            o0e, o1e, i0e, i1e = o0 - d0 * ovo, o1 - d1 * ovo, i0 + d0 * ovi, i1 + d1 * ovi
            for (sa, za), (sb, zb) in zip(prof, prof[1:]):
                pa0, pa1 = i0e + (o0e - i0e) * sa, i1e + (o1e - i1e) * sa
                pb0, pb1 = i0e + (o0e - i0e) * sb, i1e + (o1e - i1e) * sb
                q = [(pa0[0], pa0[1], za), (pa1[0], pa1[1], za), (pb1[0], pb1[1], zb), (pb0[0], pb0[1], zb)]
                nn = np.cross(np.subtract(q[1], q[0]), np.subtract(q[3], q[0]))
                if nn[2] < 0:
                    q = q[::-1]
                Lk = float(np.linalg.norm(o1 - o0))
                m.quad(ROOF, *q, (0, 0), (Lk / 1.2, 0), (Lk / 1.2, 1), (0, 1))
            # fascias (outer and inner) and soffits / 鼻隠しと軒裏
            for p0, p1, z, w_in in ((o0e, o1e, zo, o0 - o0e), (i1e, i0e, zi, i1 - i1e)):
                pass
            self.fascia_seg(m, o0e, o1e, o0, o1, zo, outward=o0e - o0)
            self.fascia_seg(m, i1e, i0e, i1, i0, zi, outward=i0e - i0)
        # end caps of the roof sweep (vertical, over each arm end) / 屋根端部の面
        for k, sgn in ((0, -1), (n, 1)):
            o, i = O[k], I[k]
            d = (i - o) / np.linalg.norm(i - o)
            oe, ie = o - d * ovo, i + d * ovi
            pts = [ie + (oe - ie) * s for s, _ in prof]
            zs = [z for _, z in prof]
            tdir = (O[1] - O[0]) if k == 0 else (O[n] - O[n - 1])
            out = -tdir if k == 0 else tdir
            for j in range(len(pts) - 1):
                p0, p1 = pts[j], pts[j + 1]
                nn = np.array([(p1 - p0)[1], -(p1 - p0)[0]])
                if float(nn @ out) < 0:
                    band(m, FASCIA, tuple(p1), tuple(p0), zs[j + 1] + 0.05, zs[j] + 0.05, 1.15)
                else:
                    band(m, FASCIA, tuple(p0), tuple(p1), zs[j] + 0.05, zs[j + 1] + 0.05, 1.15)

    def fascia_seg(self, m, e0, e1, w0, w1, z, outward):
        """Fascia from e0 to e1 (eave line), soffit back to the wall w0-w1.
        鼻隠しと軒裏（外向き・下向き）。"""
        nn = np.array([(e1 - e0)[1], -(e1 - e0)[0]])
        if float(nn @ outward) < 0:
            e0, e1, w0, w1 = e1, e0, w1, w0
        m.wall(FASCIA, tuple(e0), tuple(e1), z - 1.1, z + 0.05)
        m.wall("tp_soffit", tuple(e0), tuple(e1), z - 1.8, z - 1.1)
        q = [(e0[0], e0[1], z - 1.8), (e1[0], e1[1], z - 1.8), (w1[0], w1[1], z - 1.8), (w0[0], w0[1], z - 1.8)]
        nq = np.cross(np.subtract(q[1], q[0]), np.subtract(q[3], q[0]))
        if nq[2] > 0:
            q = q[::-1]
        m.quad("tp_soffit", *q)

    # -- plaza / 広場 ------------------------------------------------------------
    def plaza(self, m):
        zd = self.deck_z
        deck = self.deck
        # podium walls (0 .. deck) where free; the drop-off front dark glazed
        # under the charcoal canopy / 基壇の壁（車寄せ側は暗いガラスと庇）
        Pz = self.T["plaza"]
        cf = LineString([self.X(q) for q in Pz["canopy_face"]])
        fronts = []
        for a, b, L, n in B.edges_of(deck):
            f = B.Facade(m, a, b, n)
            cd = np.subtract(cf.coords[-1], cf.coords[0]) / cf.length
            front = abs(float((b - a) @ cd)) / L > 0.9 and cf.distance(Point(*((a + b) / 2))) < 12.0
            for u0, u1, zc in self.pieces("deck", a, b, n):
                if zc >= zd - 0.05:
                    continue
                if front:
                    fronts.append((u1 - u0, f, u0, u1, self.podium_front(f, u0, u1, zd)))
                else:
                    self.st_band(f, u0, u1, L, 0.0, zd, 2)
                # the glass balustrade on the deck edge / デッキ縁のガラス手すり
                if u1 - u0 > 0.8:
                    rail_run(m, "railing", f.P(u0 + 0.1, -0.15, 0)[:2], f.P(u1 - 0.1, -0.15, 0)[:2], zd, 1.1,
                             glass=True)
                f.face(WHITE, u0, u1, zd, zd + 0.15, 0.0, 1.0)
        if fronts:
            # the name on the longest free run of the front / 正面の最も長い区間に校名
            Lf, f, u0, u1, dmax = max(fronts, key=lambda q: q[0])
            if Lf > 15.0:
                self.podium_letters(f, u0, u1, zd, dmax)
        # deck surface: lilac-grey pavers, turf panels, the paved star, the mural on
        # the 9A part (OneMap) / デッキ面（紫灰色の舗装、芝、星形の小道、9A の壁画）
        ds = self.deck_surfaces()
        turf, mural = ds["turf"], ds["mural"]
        m.flat("tp_turf", turf, zd + 0.17, 3.0)
        m.flat("tp_mural", mural, zd + 0.16, 24.0)
        self.deck_furniture(m, ds)
        # the drum: white bands, glazed slot band, the 8-spoke skylight roof
        # 円筒: 白い帯、ガラスの帯、8 本の放射状の天窓
        cx, cy = self.drum_c
        r, zt = self.drum_r, self.drum_top
        drum(m, WHITE, (cx, cy), r, zd + 0.15, zd + 3.0, seg=32, uvs=3.0)
        drum(m, "tp_shopglass", (cx, cy), r - 0.4, zd + 3.0, zd + 6.5, seg=32, uvs=3.0)
        annulus(m, WHITE, (cx, cy), r - 0.4, r, zd + 3.0, up=True)
        annulus(m, WHITE, (cx, cy), r - 0.4, r, zd + 6.5, up=False)
        drum(m, WHITE, (cx, cy), r, zd + 6.5, zt, seg=32, uvs=3.0)
        for z in np.arange(zd + 8.0, zt - 0.5, 2.2):
            drum(m, "tp_navy", (cx, cy), r + 0.05, z, z + 0.25, seg=32)
        m.flat("roof_concrete", Point(cx, cy).buffer(r - 0.4, 8), zt + 0.4, 3.0)
        drum(m, WHITE, (cx, cy), r, zt, zt + 0.9, seg=32)
        annulus(m, WHITE, (cx, cy), r - 0.4, r, zt + 0.9, up=True)
        drum(m, WHITE, (cx, cy), r - 0.4, zt + 0.4, zt + 0.9, seg=32) if False else None
        frustum(m, "tp_curtain", (cx, cy), 3.2, 1.2, zt + 0.4, zt + 2.2, seg=16)
        m.flat(GREY, Point(cx, cy).buffer(1.2, 4), zt + 2.2, 1.0)
        for j in range(8):
            ang = math.radians(22.5 + 45 * j)
            p0 = np.array([cx, cy]) + np.array([math.cos(ang), math.sin(ang)]) * 3.0
            p1 = np.array([cx, cy]) + np.array([math.cos(ang), math.sin(ang)]) * (r - 1.0)
            mid = (p0 + p1) / 2
            m.box("tp_curtain", mid[0], mid[1], zt + 0.4, zt + 1.1, float(np.linalg.norm(p1 - p0)), 1.2, ang)
        # the sunken terraced garden: steps down from the deck in 3 terraces
        # 沈んだ段状の庭（3 段）
        if not self.sunken.is_empty and self.sunken.geom_type == "Polygon":
            zf = self.T["plaza"]["sunken_floor"]
            levels = np.linspace(zd, zf, 4)
            cur = self.sunken
            for j in range(3):
                nxt = cur.buffer(-2.6, join_style="mitre")
                ring = cur.difference(nxt) if not nxt.is_empty else cur
                for g in polys(ring):
                    m.flat("tp_planter", g, levels[j + 1], 2.0)
                for g in polys(cur):
                    m.ring_walls("concrete", g.exterior, levels[j + 1], levels[j] + 0.15, outward=False)
                cur = nxt
                if cur.is_empty:
                    break
            if not cur.is_empty:
                m.flat("tp_planter", cur, levels[-1], 2.0)
            for a, b, L, n in B.edges_of(self.sunken):
                rail_run(m, "railing", a + n * 0.2, b + n * 0.2, zd + 0.15, 1.1, glass=True)
        # the saucer canopy on its column / 一本柱の円盤の庇
        sx, sy = self.saucer_c
        if deck.buffer(-1.0).contains(Point(sx, sy)):
            zs, rs = self.saucer_z, self.saucer_r
            m.cylinder("tp_charcoal", sx, sy, zd + 0.15, zs - 1.0, 0.35, seg=10, top=False)
            frustum(m, "tp_canopy", (sx, sy), 0.4, rs, zs - 1.2, zs, seg=24)
            frustum(m, WHITE, (sx, sy), rs, rs - 0.1, zs, zs + 0.35, seg=24)
            m.flat(WHITE, Point(sx, sy).buffer(rs - 0.1, 6), zs + 0.35, 2.0)
        # the ramp + steps from the drop-off / 車寄せからのスロープと階段
        self.ramp_and_steps(m)

    def deck_surfaces(self):
        """The deck's paving (walkable: Road_Sidewalk via walk_mesh), turf panels
        and the 9A mural, cut round the drum and the sunken garden.
        デッキの舗装（歩行面）、芝、9A の壁画。"""
        if getattr(self, "_deck_s", None) is not None:
            return self._deck_s
        Pz = self.T["plaza"]
        S = Pz["star"]
        sc = np.array(self.X(S["c"]))
        rays = unary_union([LineString([tuple(sc), tuple(sc + 80 * np.array([math.sin(math.radians(bb)),
                                                                              math.cos(math.radians(bb))]))])
                            .buffer(S["w"] / 2, cap_style="flat") for bb in S["bearings"]] + [Point(*sc).buffer(5.0)])
        court = Polygon([tuple(q) for q in self.hI]).buffer(0)
        dr = Point(self.drum_c).buffer(self.drum_r, 24)
        holes = unary_union([dr, self.sunken])
        surf = self.deck.difference(holes)
        turf = court.buffer(-6.0).difference(rays).difference(holes.buffer(2.5)).intersection(surf)
        turf = unary_union([g for g in polys(turf) if g.area > 25.0]) if polys(turf) else Polygon()
        pod = self.XP(Pz["podium"]).difference(court.buffer(2.0))
        mural = pod.buffer(-4.0, join_style="mitre").intersection(surf).difference(turf)
        paver = surf.difference(turf).difference(mural)
        self._deck_s = dict(paver=paver, turf=turf, mural=mural, rays=rays, court=court)
        return self._deck_s

    def walk_keep(self):
        """The paved pieces walkers get: those joined into >= 50 m2 (slivers round
        the blocks stay plain paving, they only split the network into crumbs).
        歩行面にする舗装（50 m2 以上につながる部分だけ。細片は普通の舗装）。"""
        if getattr(self, "_walk_keep", None) is None:
            Z = self.zones()
            ds = self.deck_surfaces()
            W = unary_union([Z["hard"], Z["pool_deck"], ds["paver"], self.ramp]).buffer(0.05)
            self._walk_keep = unary_union([g for g in polys(W) if g.area >= 50.0]) if polys(W) else Polygon()
        return self._walk_keep

    def walk_mesh(self, tile):
        """Where walkers go (build.py merges it into the tile's Road_Sidewalk): the
        campus paths, the pad, the pool deck, the plaza deck and the ramp.
        歩行者の歩く面（build.py がタイルの Road_Sidewalk に合わせる）。"""
        m = Mesh()
        if self.empty or not self.site.buffer(40.0).intersects(tile):
            return m
        Z = self.zones()
        T = lambda g: g.intersection(tile)
        z = Z0
        m.flat("concrete_path", T(Z["hard"].intersection(self.walk_keep())), z + 0.01, 2.0)
        m.flat("tp_pooldeck", T(Z["pool_deck"]), z + 0.012, 2.0)
        ds = self.deck_surfaces()
        m.flat("tp_paver", T(ds["paver"]), self.deck_z + 0.15, 2.0)
        if not self.ramp.is_empty:
            a, t, Lr, zd = self.ramp_a, self.ramp_t, self.ramp_len, self.deck_z
            zf = lambda X, Y: Z0 + (zd + 0.15 - Z0) * np.clip(((np.asarray(X) - a[0]) * t[0]
                                                               + (np.asarray(Y) - a[1]) * t[1]) / Lr, 0.0, 1.0)
            m.flat("tp_paver", T(self.ramp), 0.0, 2.0, zf=zf)
        return m

    def deck_furniture(self, m, ds):
        """The courtyard's coloured light columns along the paths, the faceted
        yellow stools, planters (2026 photo). / 中庭の色付きの光る柱、黄色い腰掛け、植木鉢。"""
        zd = self.deck_z + 0.15
        paver = ds["paver"]
        rays = ds["rays"]
        rng = np.random.default_rng(11)
        edge = ds["turf"].boundary
        n = 0
        for d in np.arange(4.0, edge.length, 9.0) if not edge.is_empty else []:
            q = edge.interpolate(d)
            p = np.array(q.coords[0])
            if not paver.buffer(-0.3).contains(Point(*p)) and not ds["turf"].buffer(-0.3).contains(Point(*p)):
                continue
            m.cylinder("tp_lampcol", p[0], p[1], zd, zd + 1.0, 0.16, seg=6, top=False)
            m.cylinder("tp_glow", p[0], p[1], zd + 1.0, zd + 3.6, 0.13, seg=6)
            n += 1
            if n > 40:
                break
        for k in range(14):
            g = ds["turf"].buffer(-1.5)
            if g.is_empty:
                break
            x0, y0, x1, y1 = g.bounds
            p = rng.uniform([x0, y0], [x1, y1])
            if g.contains(Point(*p)):
                m.cylinder("tp_bench", p[0], p[1], zd, zd + 0.45, 0.55, seg=6)

    def podium_front(self, f, u0, u1, zd):
        """Blk 9A's drop-off front: dark glazing behind columns, the charcoal canopy
        fascia with WELCOME TO TEMASEK, the red name letters on the deck edge.
        9A の正面: 暗いガラス、炭色の庇、赤い校名。"""
        f.face("tp_glassdark", u0, u1, Z0, zd - 2.0, -2.5, 1.0)
        f.hplane(WHITE, u0, u1, -2.5, 0.0, zd - 2.0, up=False)
        # mullions, entrance doors, flower pots under the canopy (2014 photo)
        # 方立て、入口の扉、庇の下の植木鉢
        for c in np.arange(u0 + 1.5, u1 - 0.5, 1.5):
            f.block(GREY, c - 0.05, c + 0.05, -2.5, -2.38, Z0, zd - 2.0, top=False, bottom=False)
        for c in centres(u0, u1, 16.0, 4.0):
            f.face("tp_door", c - 1.6, c + 1.6, Z0, Z0 + 2.6, -2.36, 1.0)
        for c in centres(u0, u1, 6.0, 2.0):
            pc = np.array(f.P(c, -0.9, 0)[:2])
            if self.no_go.distance(Point(*pc)) > 1.0:
                f.m.box("tp_pot", pc[0], pc[1], Z0, Z0 + 0.5, 0.8, 0.8, math.atan2(f.t[1], f.t[0]))
                f.m.blob("tp_flower", pc[0], pc[1], Z0 + 0.75, 0.5, 0.5, 0.35, seg=6, rings=3)
        for c in centres(u0, u1, 8.0, 0.0):
            f.block(GREY, c - 0.4, c + 0.4, -0.4, 0.4, Z0, zd - 2.0, top=False, bottom=False)
        # the canopy band: out to 3 m where the road allows / 庇（道路が許す範囲で 3 m）
        dmax = 3.0
        for d_ in (3.0, 2.0, 1.2, 0.6):
            ok = True
            for u in np.linspace(u0, u1, 6):
                if self.no_go.distance(Point(*f.P(u, d_, 0)[:2])) < ROOM:
                    ok = False
            if ok:
                dmax = d_
                break
        else:
            dmax = 0.3
        f.block("tp_charcoal", u0, u1, 0.0, dmax, zd - 2.0, zd)
        f.face(WHITE, u0, u1, zd - 2.0, zd, 0.0, 3.0)
        return dmax

    def podium_letters(self, f, u0, u1, zd, dmax):
        L = u1 - u0
        if True:
            # red "Temasek" standing on the deck edge, POLYTECHNIC and WELCOME on the
            # charcoal fascia (main-gate photo) / デッキ縁の赤い校名と庇の文字
            cu = u0 + L * 0.55
            g = text_poly("Temasek", 2.0, aspect=0.95)
            gw = g.bounds[2] - g.bounds[0]
            cu = min(max(cu, u0 + gw / 2 + 2.0), u1 - gw / 2 - 0.3)
            glyphs_on(f.m, f, g, cu, zd + 1.3, dmax - 0.3, "tp_redletter", 2.0)
            m2 = text_poly("POLYTECHNIC", 0.7, aspect=1.1)
            glyphs_on(f.m, f, m2, cu + 0.5, zd - 0.6, dmax + 0.02, WHITE, 0.7)
            w2 = text_poly("WELCOME TO TEMASEK", 0.45, aspect=1.0)
            glyphs_on(f.m, f, w2, max(u0 + 4.0, cu - gw / 2 - 1.0), zd - 1.4, dmax + 0.02, "tp_redletter", 0.45)
            # the round logo disc / 丸いロゴ
            glyphs_on(f.m, f, Point(0, 0).buffer(1.0, 16), cu - gw / 2 - 1.2, zd + 1.3, dmax - 0.3,
                      "tp_redletter", 2.0)

    def ramp_and_steps(self, m):
        if self.ramp.is_empty:
            return
        zd = self.deck_z
        a, t, Lr = self.ramp_a, self.ramp_t, self.ramp_len
        nn = np.array([-t[1], t[0]])
        zf = lambda X, Y: Z0 + (zd + 0.15 - Z0) * np.clip(((np.asarray(X) - a[0]) * t[0] + (np.asarray(Y) - a[1]) * t[1])
                                                          / Lr, 0.0, 1.0)
        # split: the podium side half as steps, the other half ramp / 基壇側は階段、反対側はスロープ
        mids = [(self.ramp.centroid.x, self.ramp.centroid.y)]
        side = np.sign(float((np.array(self.deck.centroid.coords[0]) - a) @ nn)) or 1.0
        cut = LineString([tuple(a - t * 5), tuple(a + t * (Lr + 5))]).buffer(30, single_sided=True) if side > 0 else \
            LineString([tuple(a + t * (Lr + 5)), tuple(a - t * 5)]).buffer(30, single_sided=True)
        steps = Polygon()                         # the 2010 pano: one paved ramp / 2010 年: 一面のスロープ
        slope = self.ramp
        nst = int(round((zd + 0.15 - Z0) / 0.16))
        for j in range(nst):
            s0, s1 = Lr * j / nst, Lr * (j + 1) / nst
            band = Polygon([tuple(a + t * s0 - nn * 30), tuple(a + t * s1 - nn * 30), tuple(a + t * s1 + nn * 30),
                            tuple(a + t * s0 + nn * 30)])
            g = steps.intersection(band)
            z = Z0 + (zd + 0.15 - Z0) * (j + 1) / nst
            for q in polys(g):
                m.flat("tp_paver", q, z, 2.0)
                e = LineString([tuple(a + t * s0 - nn * 30), tuple(a + t * s0 + nn * 30)]).intersection(q.buffer(1e-3))
                for ls in getattr(e, "geoms", [e]):
                    if ls.geom_type == "LineString" and ls.length > 0.2:
                        p0, p1 = np.array(ls.coords[0]), np.array(ls.coords[-1])
                        if float(np.array([(p1 - p0)[1], -(p1 - p0)[0]]) @ (-t)) < 0:
                            p0, p1 = p1, p0
                        m.wall("concrete", tuple(p0), tuple(p1), z - 0.16, z)
        # a short flight of steps beside the ramp's foot, from the drop-off (2014 photo)
        # スロープの足元の脇の短い階段
        side = 1.0 if float((np.array(self.deck.centroid.coords[0]) - a) @ nn) > 0 else -1.0
        for j in range(6):
            s0, s1 = 0.4 * j, 0.4 * (j + 1)
            p0 = a + nn * side * (self.T["plaza"]["ramp"]["w"] / 2 + 0.2)
            q = Polygon([tuple(p0 + t * s0), tuple(p0 + t * s1), tuple(p0 + t * s1 + nn * side * 3.0),
                         tuple(p0 + t * s0 + nn * side * 3.0)]).difference(self.ng).difference(self.deck)
            zz = Z0 + 0.16 * (j + 1)
            for g in polys(q):
                if g.area < 0.3:
                    continue
                m.prism("concrete", orient(g, 1.0), Z0, zz)
        # white parapet walls with square openings along both long sides, flower pots
        # 両側の白い腰壁（四角い開口）と植木鉢
        for a_, b_, L, n in B.edges_of(orient(self.ramp, 1.0)):
            if abs(float((b_ - a_) @ t)) / L < 0.8 or L < 3:
                continue
            za, zb_ = float(zf(*a_)), float(zf(*b_))
            wall_top(m, WHITE, tuple(a_), tuple(b_), min(za, zb_) - 0.3, za + 1.1, zb_ + 1.1, 3.0)
            wall_top(m, WHITE, tuple(b_ - n * 0.25), tuple(a_ - n * 0.25), min(za, zb_), zb_ + 1.1, za + 1.1, 3.0)
            f = B.Facade(m, a_, b_, n)
            for s in np.arange(1.5, L - 1.0, 2.4):
                p = a_ + (b_ - a_) * s / L
                z = float(zf(*p))
                f.face("tp_charcoal", s, s + 0.6, z + 0.35, z + 0.85, 0.01, 1.0)
            for s in np.arange(2.7, L - 1.0, 4.8):
                pc = np.array(f.P(s, -0.75, 0)[:2])
                z = float(zf(*pc))
                m.box("tp_pot", pc[0], pc[1], z, z + 0.45, 0.9, 0.45, math.atan2(f.t[1], f.t[0]))
                m.blob("tp_flower", pc[0], pc[1], z + 0.65, 0.5, 0.3, 0.3, seg=6, rings=3)
            pa, pb = a_ - n * 0.125, b_ - n * 0.125
            m.quad(WHITE, (pa[0] - n[0] * 0.125, pa[1] - n[1] * 0.125, za + 1.1),
                   (pb[0] - n[0] * 0.125, pb[1] - n[1] * 0.125, zb_ + 1.1),
                   (pb[0] + n[0] * 0.125, pb[1] + n[1] * 0.125, zb_ + 1.1),
                   (pa[0] + n[0] * 0.125, pa[1] + n[1] * 0.125, za + 1.1))

    # -- library / 図書館 --------------------------------------------------------
    def library(self, m):
        """Blk 27 library: the base with grid curtain-wall boxes in the corners (2020
        photo) and white punched arm ends, terraces on the corner boxes (tiles), the
        white cross with glazed re-entrant faces, the blue glass band, the flared
        white crown (OneMap close orbit). / 図書館: 入隅のカーテンウォールの箱と屋上テラス、
        白い十字と入隅のガラス、青いガラスの帯、外へ反る白い頂部。"""
        Lb = self.lib
        if Lb is None:
            return
        z0, z1, z2, z3, z4 = self.libz          # 0, 18, 40, 44.5, 49
        cross, base = Lb["cross"], Lb["base"]
        shapely.prepare(cross)
        # base (0 .. 18) / 下段
        for a, b, L, n in B.edges_of(base):
            f = B.Facade(m, a, b, n)
            mid = Point(*((a + b) / 2 + n * 0.05))
            corner_face = cross.exterior.distance(Point(*((a + b) / 2))) > 0.3
            for u0, u1, zc in self.pieces("library", a, b, n):
                if zc >= z1 - 0.05:
                    continue
                zb = max(Z0, zc)
                if corner_face:
                    f.face(WHITE, u0, u1, zb, max(zb, 4.0), 0.0, 3.0)
                    self.curtain(f, u0, u1, max(zb, 4.0), z1 - 0.8, "tp_curtaingrid", fins=1.8)
                    f.face(WHITE, u0, u1, z1 - 0.8, z1, 0.0, 3.0)
                else:
                    self.t_windows(f, u0, u1, self.floors(zb, z1 - 0.8, 4), bay=3.0, w=1.1, h=1.8, shade=WHITE)
                    f.face(WHITE, u0, u1, z1 - 0.8, z1, 0.0, 3.0)
                    # pilasters 0.3 m in from the corners (an end face in a corner plane
                    # faces the neighbour wall back to back) / 隅から 0.3 m 内側の付柱
                    if u1 - u0 > 3.0:
                        f.block(WHITE, u0 + 0.3, u0 + 0.9, 0.0, 0.35, zb, z1, bottom=False)
                        f.block(WHITE, u1 - 0.9, u1 - 0.3, 0.0, 0.35, zb, z1, bottom=False)
        # terraces on the corner boxes: pavers, planters, glass railings / 入隅の屋上テラス
        for q in Lb["corners"]:
            if q.is_empty or q.geom_type != "Polygon":
                continue
            m.flat("tp_paver", q, z1 + 0.05, 2.0)
            for a, b, L, n in B.edges_of(q):
                if cross.exterior.distance(Point(*((a + b) / 2))) < 0.3:
                    continue
                f = B.Facade(m, a, b, n)
                f.face(WHITE, 0.0, L, z1, z1 + 0.25, 0.0, 1.0)
                f.hplane(WHITE, 0.0, L, -0.25, 0.0, z1 + 0.25, up=True)
                rail_run(m, "railing", a - n * 0.12, b - n * 0.12, z1 + 0.25, 1.1, glass=True)
                for cu in centres(0.0, L, 3.0, 1.0):
                    pc = np.array(f.P(cu, -1.0, 0)[:2])
                    m.box("tp_planterbox", pc[0], pc[1], z1 + 0.05, z1 + 0.7, 1.6, 0.8,
                          math.atan2(f.t[1], f.t[0]))
                    m.blob("tp_planter", pc[0], pc[1], z1 + 0.95, 0.8, 0.8, 0.45, seg=6, rings=3)
        # the cross (18 .. 40): glazed re-entrant faces, white arm ends with windows
        # 十字（18〜40 m）: 入隅はガラス、腕の端は白い壁と窓
        hull = cross.convex_hull.exterior
        for a, b, L, n in B.edges_of(cross):
            f = B.Facade(m, a, b, n)
            reent = hull.distance(Point(*((a + b) / 2))) > 0.5
            for u0, u1, zc in self.pieces("library", a, b, n, base=z1):
                if zc >= z2 - 0.05:
                    continue
                zb = max(z1, zc)
                if reent:
                    self.curtain(f, u0, u1, zb, z2, "tp_curtain", fins=1.5)
                else:
                    fl = [(zz, zz + 4.4) for zz in np.arange(z1, z2 - 4.3, 4.4) if zz >= zb - 0.1]
                    f.face(WHITE, u0, u1, zb, fl[0][0] if fl else z2, 0.0, 3.0)
                    self.t_windows(f, u0, u1, fl, bay=3.6, w=1.0, h=1.9, shade=WHITE, glass="tp_glassdark")
                    if fl:
                        f.face(WHITE, u0, u1, fl[-1][1], z2, 0.0, 3.0)
            # the blue glass band (40 .. 44.5) and its white sill / 青いガラスの帯
            f.face(WHITE, 0.0, L, z2, z2 + 0.5, 0.0, 1.0)
            self.curtain(f, 0.0, L, z2 + 0.5, z3, "tp_curtain", fins=1.2)
        # the crown: flared white walls 44.5 .. 49, the roof, plant box / 反った頂部、屋上
        r = inset_ring(cross, -1.6)
        if r is not None:
            c, o = r
            nv = len(c)
            for i in range(nv):
                j = (i + 1) % nv
                m.quad(WHITE, (c[i][0], c[i][1], z3), (c[j][0], c[j][1], z3), (o[j][0], o[j][1], z4),
                       (o[i][0], o[i][1], z4))
            outer = Polygon(o)
            inner = outer.buffer(-0.35, join_style="mitre")
            m.flat("roof_concrete", cross.buffer(-0.2, join_style="mitre"), z4 - 1.2, 3.0)
            m.ring_walls(WHITE, inner.exterior, z4 - 1.2, z4, outward=False)
            m.flat(WHITE, outer.difference(inner), z4, 1.0)
            # the crown's inside face down to the roof (seen from above) / 頂部の内側
            ctr = Lb["c"]
            m.box(GREY, ctr[0], ctr[1], z4 - 1.2, z4 + 1.4, 3.2, 2.2, math.atan2(Lb["u"][1], Lb["u"][0]))
            m.box("tp_glassdark", ctr[0], ctr[1], z4 + 0.6, z4 + 1.2, 3.25, 2.25,
                  math.atan2(Lb["u"][1], Lb["u"][0]), top=False)
        else:
            self.flat_roof(m, cross, z3)

    def curtain(self, f, u0, u1, z0, z1, glass, fins=None):
        """A curtain wall: one glass quad (grid in the texture, white frames, dark
        spandrels every floor) and a mullion fin every 1.8 m. / カーテンウォール。"""
        if u1 - u0 < 0.5 or z1 - z0 < 0.5:
            return
        f.m.quad(glass, f.P(u0, 0.0, z0), f.P(u1, 0.0, z0), f.P(u1, 0.0, z1), f.P(u0, 0.0, z1),
                 (u0 / 1.8, z0 / 4.5), (u1 / 1.8, z0 / 4.5), (u1 / 1.8, z1 / 4.5), (u0 / 1.8, z1 / 4.5))
        for z in np.arange(z0 + 4.5, z1 - 0.2, 4.5):
            f.block(WHITE, u0, u1, 0.0, 0.18, z - 0.1, z + 0.1, ends=False)
        if fins:
            # white mullion fins (2020 photo) / 白い方立て
            for cu in np.arange(u0 + fins, u1 - 0.2, fins):
                f.block(WHITE, cu - 0.06, cu + 0.06, 0.0, 0.3, z0, z1, bottom=False)

    # -- lawn / 芝生 -------------------------------------------------------------
    def lawn_works(self, m):
        """The channel's white coping, the 4 ring basins, the stepped
        amphitheatre, the lawn's palm axis lamps. / 水路の縁、輪の池、円形劇場。"""
        Z = self.zones()
        ch = self.chan
        for g in polys(ch):
            m.ring_walls("concrete", g.exterior, Z0, Z0 + 0.35, outward=False)
            ring = g.buffer(0.5, join_style="round").difference(g)
            m.flat("concrete", ring, Z0 + 0.35, 2.0)
            m.ring_walls("concrete", g.buffer(0.5).exterior, Z0, Z0 + 0.35)
        # the pool complex's fence round its deck (Blk 31A) / プールのデッキの柵
        po = getattr(self, "pool_deck_outline", Polygon())
        for g in polys(po):
            for a, b, L, n in B.edges_of(g):
                seg = LineString([tuple(a), tuple(b)])
                if L > 1.0 and seg.distance(self.footprint) > 0.5 and seg.distance(self.no_go) > 0.6:
                    rail_panel(m, "tp_railgreen", "fence_green", a, b, Z0, 1.8)
        for g in polys(Z["pools"]):
            # the pool's white coping / プールの白い縁
            ring = g.buffer(0.6, join_style="mitre").difference(g)
            m.flat("tp_basin", ring, Z0 + 0.3, 2.0)
            m.ring_walls("tp_basin", g.exterior, Z0, Z0 + 0.3, outward=False)
            m.ring_walls("tp_basin", g.buffer(0.6, join_style="mitre").exterior, Z0, Z0 + 0.3)
        for q in self.T["lawn"]["basins"]:
            c = self.X(q)
            if not ch.buffer(-1.0).contains(Point(c)):
                continue
            rb = self.T["lawn"]["basin_r"]
            drum(m, "tp_basin", c, rb + 0.5, Z0, Z0 + 0.5, seg=24)
            m.ring_walls("tp_basin", Point(c).buffer(rb, 6).exterior, Z0, Z0 + 0.5, outward=False)
            annulus(m, "tp_basin", c, rb, rb + 0.5, Z0 + 0.5, up=True)
        # amphitheatre: tiers rising outward from the stage / 円形劇場: 舞台から外へ上る段
        ac, ad, R, half = self.amph
        if not half.is_empty:
            nt = 6
            r0 = 5.0
            stage = half.intersection(Point(*ac).buffer(r0, 24))
            m.flat("concrete_path", stage, Z0 + 0.3, 2.0)
            m.ring_walls("concrete", orient(stage, 1.0).exterior if stage.geom_type == "Polygon" else
                         Polygon().exterior, Z0, Z0 + 0.3) if stage.geom_type == "Polygon" else None
            for j in range(nt):
                ra, rb = r0 + (R - r0) * j / nt, r0 + (R - r0) * (j + 1) / nt
                band = half.intersection(Point(*ac).buffer(rb, 32)).difference(Point(*ac).buffer(ra, 32))
                z = Z0 + 0.4 * (j + 1)
                for g in polys(band):
                    m.flat("concrete" if j % 2 else "tp_seat", g, z, 2.0)
                    m.ring_walls("concrete", orient(g, 1.0).exterior, Z0, z)

    # -- boundary / 境界 ---------------------------------------------------------
    def boundary(self, m):
        """Ave 1 / St 96 side: low white wall with steel railings; canal side: a
        green railing; openings at the gates; guard houses; the West gate's
        white sign wall with the red name. / 境界の塀、門、守衛所、西門の校名壁。"""
        ring = orient(self.lot, 1.0).exterior
        shapely.prepare(self.no_go)
        streets = self.road.buffer(14.0)
        coords = list(ring.coords)
        # no railing in front of the West gate's sign wall / 西門の校名壁の前は柵なし
        wgp = Point(self.X(self.T["gates"]["west"]))
        for a, b in zip(coords, coords[1:]):
            seg = LineString([a, b])
            L = seg.length
            if L < 0.5:
                continue
            n_ = max(1, int(L // 18.0))
            for j in range(n_):
                p = np.array(seg.interpolate(L * j / n_).coords[0])
                q = np.array(seg.interpolate(L * (j + 1) / n_).coords[0])
                piece = LineString([tuple(p), tuple(q)])
                if piece.distance(wgp) < 16.0:
                    continue
                if piece.distance(self.no_go) < 0.6 or piece.intersects(self.footprint.buffer(0.3)):
                    # split a blocked run into 6 m pieces and keep the clear ones / 遮られた区間は 6 m に分ける
                    k_ = max(1, int(piece.length // 6.0))
                    subs = [LineString([piece.interpolate(piece.length * i / k_).coords[0],
                                        piece.interpolate(piece.length * (i + 1) / k_).coords[0]]) for i in range(k_)]
                    subs = [g for g in subs if g.distance(self.no_go) >= 0.6 and not
                            g.intersects(self.footprint.buffer(0.3))]
                else:
                    subs = [piece]
                for g in subs:
                    self.fence_piece(m, np.array(g.coords[0]), np.array(g.coords[-1]), streets)
        self.gates_and_signs(m)

    def fence_piece(self, m, p, q, streets):
        """Street side: low white wall with a steel railing; elsewhere a green
        railing (canal). / 道路側は白い腰壁と手すり、他は緑の柵。"""
        piece = LineString([tuple(p), tuple(q)])
        if streets.intersects(piece):
            two = np.array([(q - p)[1], -(q - p)[0]]) / max(np.linalg.norm(q - p), 1e-6)
            m.wall(WHITE, tuple(p + two * 0.1), tuple(q + two * 0.1), Z0 - 0.1, Z0 + 0.6, 0.0, 2.0)
            m.wall(WHITE, tuple(q - two * 0.1), tuple(p - two * 0.1), Z0 - 0.1, Z0 + 0.6, 0.0, 2.0)
            m.quad(WHITE, (*(p - two * 0.1), Z0 + 0.6), (*(q - two * 0.1), Z0 + 0.6),
                   (*(q + two * 0.1), Z0 + 0.6), (*(p + two * 0.1), Z0 + 0.6))
            rail_panel(m, "tp_railbars", "tp_railgrey", p, q, Z0 + 0.6, 1.0)
        else:
            rail_panel(m, "tp_railgreen", "fence_green", p, q, Z0, 1.4)

    def gates_and_signs(self, m):
        # guard houses at the gates (inside, beside the drive) / 門の守衛所
        self.guards = []
        for g in self.gates:
            mid = np.array(g.interpolate(0.5, normalized=True).coords[0])
            inward = np.array(self.lot.centroid.coords[0]) - mid
            inward /= np.linalg.norm(inward)
            placed = False
            for back in (6.0, 9.0, 12.0):
                for sgn in (1, -1):
                    side = np.array([-inward[1], inward[0]]) * sgn
                    for off in (5.0, 7.0, 9.0):
                        c = mid + inward * back + side * off
                        foot = Point(*c).buffer(2.6, cap_style="square")
                        if self.no_go.distance(foot) > 0.6 and self.lot.contains(foot) and \
                                not foot.intersects(self.footprint):
                            self.guard_house(m, c, math.atan2(inward[1], inward[0]))
                            self.guards.append(c)
                            placed = True
                            break
                    if placed:
                        break
                if placed:
                    break
        # the West gate's sign wall / 西門の校名壁
        wg = np.array(self.X(self.T["gates"]["west"]))
        best = None
        for g in self.gates:
            mid = np.array(g.interpolate(0.5, normalized=True).coords[0])
            d = float(np.linalg.norm(mid - wg))
            if best is None or d < best[0]:
                best = (d, g, mid)
        if best is not None and best[0] < 60:
            _, g, mid = best
            p0, p1 = np.array(g.coords[0]), np.array(g.coords[-1])
            t = (p1 - p0) / np.linalg.norm(p1 - p0)
            for sgn in (1, -1):
                end = p1 if sgn > 0 else p0
                a = end + t * sgn * 1.5
                b = end + t * sgn * 13.5
                seg = LineString([tuple(a), tuple(b)])
                if seg.distance(self.no_go) < 0.6 or not self.lot.buffer(2.0).contains(seg):
                    continue
                nrm = np.array([t[1], -t[0]])
                if float(nrm @ (np.array(self.lot.centroid.coords[0]) - mid)) > 0:
                    nrm = -nrm
                aa, bb = (a, b) if float(np.array([(b - a)[1], -(b - a)[0]]) @ nrm) > 0 else (b, a)
                f = B.Facade(m, aa, bb, nrm / np.linalg.norm(nrm))
                L = float(np.linalg.norm(bb - aa))
                f.block(WHITE, 0.0, L, -0.5, 0.0, Z0, Z0 + 2.4)
                g1 = text_poly("Temasek", 0.9, aspect=0.95)
                glyphs_on(m, f, g1, L / 2 + 0.6, Z0 + 1.55, 0.02, "tp_redletter", 0.9)
                g2 = text_poly("POLYTECHNIC", 0.35, aspect=1.1)
                glyphs_on(m, f, g2, L / 2 + 0.6, Z0 + 0.85, 0.02, "tp_charcoal", 0.35)
                glyphs_on(m, f, Point(0, 0).buffer(0.45, 12), L / 2 - 2.6, Z0 + 1.55, 0.02, "tp_redletter", 0.9)
                break

    def guard_house(self, m, c, ang, w=4.0, d=3.2, ov=0.6):
        """A white guard booth with glazing on every side and a grey pyramid roof
        (SV h215 / h207 "WEST"). / 白い守衛所（四方にガラス、灰色の方形屋根）。"""
        W_, D_ = w, d
        m.box(WHITE, c[0], c[1], Z0, Z0 + 2.7, W_, D_, ang)
        for k in range(4):
            a = ang + k * math.pi / 2
            n = np.array([math.cos(a), math.sin(a)])
            half = W_ / 2 if k % 2 == 0 else D_ / 2
            w = D_ if k % 2 == 0 else W_
            p = np.array(c) + n * (half + 0.01)
            t = np.array([-n[1], n[0]])
            m.wall("tp_shopglass", tuple(p - t * (w / 2 - 0.4)), tuple(p + t * (w / 2 - 0.4)), Z0 + 1.0, Z0 + 2.2)
        # grey pyramid roof / 灰色の方形屋根
        hx, hy = W_ / 2 + ov, D_ / 2 + ov
        R_ = lambda pts: Polygon([(c[0] + math.cos(ang) * x - math.sin(ang) * y,
                                   c[1] + math.sin(ang) * x + math.cos(ang) * y) for x, y in pts])
        q = R_(((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)))
        if not mansard(m, q, Z0 + 2.7, Z0 + 2.7 + 0.55 * min(hx, hy), min(hx, hy) - 0.05, "tp_greyroof"):
            m.flat("tp_greyroof", q, Z0 + 2.75, 1.0)
        m.flat("concrete", q.difference(R_(((-W_ / 2, -D_ / 2), (W_ / 2, -D_ / 2), (W_ / 2, D_ / 2),
                                            (-W_ / 2, D_ / 2)))), Z0 + 2.7, 1.0, up=False)

    # -- trees / 樹木 ------------------------------------------------------------
    def trees(self, m):
        """Palms along the lawn axis (2010 photo), rain trees scattered over the
        lawns and by the drives (tiles). / 軸のヤシ、芝生と車路沿いの雨の木。"""
        import props
        Z = self.zones()
        busy = unary_union([self.footprint.buffer(4.0), self.ramp.buffer(2.0), self.chan.buffer(2.0),
                            Z["courts"].buffer(3.0), Z["pools"].buffer(3.0), self.amph[3].buffer(2.0)])
        ax = LineString([self.X(q) for q in self.T["lawn"]["axis"]])
        n = 0
        for s in np.arange(6.0, ax.length - 4.0, 9.0):
            p = np.array(ax.interpolate(s).coords[0])
            t = np.array(ax.interpolate(min(s + 1, ax.length)).coords[0]) - p
            t /= np.linalg.norm(t)
            for sg in (1, -1):
                q = p + np.array([-t[1], t[0]]) * 4.6 * sg
                pt = Point(*q)
                if busy.contains(pt) or not self.inner().contains(pt) or self.no_go.distance(pt) < 3.5:
                    continue
                props.Props.tree_mesh(None, m, q[0], q[1], "palm", 1.0, False)
                n += 1
        grass = Z["grass"]
        # the great lawn stays open (tiles): no rain trees there / 大芝生は木を植えない
        great = self.XP(self.T["lawn"]["open"]).union(self.XP(self.T["lawn"]["stripes"]).buffer(4.0))
        x0, y0, x1, y1 = grass.bounds
        rng = np.random.default_rng(7)
        cnt = 0
        for x in np.arange(x0, x1, 21.0):
            for y in np.arange(y0, y1, 21.0):
                q = np.array([x, y]) + rng.uniform(-5, 5, 2)
                pt = Point(*q)
                if not grass.buffer(-3.0).contains(pt) or busy.contains(pt) or self.no_go.distance(pt) < 6.0:
                    continue
                if great.contains(pt):
                    continue
                props.Props.tree_mesh(None, m, q[0], q[1], "rain", 0.8, False)
                cnt += 1
                if cnt > 220:
                    break
            if cnt > 220:
                break
        self.n_trees = n + cnt


def inner_rect(p, ang=None, step=0.25):
    """(a, b, c, d) of a rectangle inside the footprint (within 5 cm), sides
    along ``ang`` or the oriented rectangle's long side, a->b the long side;
    None when it would keep < 50 %. / 外形に収まる長方形（無ければ None）。"""
    if ang is None:
        a, b, c, d = gable_rect(p)
        ax = (b - a) / np.linalg.norm(b - a)
    else:
        ax = np.array([math.cos(ang), math.sin(ang)])
    sd = np.array([-ax[1], ax[0]])
    P = np.asarray(p.exterior.coords)
    u, v = P @ ax, P @ sd
    bnd = [u.min(), u.max(), v.min(), v.max()]
    fit = p.buffer(0.05)
    R = lambda q: Polygon([tuple(ax * q[0] + sd * q[2]), tuple(ax * q[1] + sd * q[2]), tuple(ax * q[1] + sd * q[3]),
                           tuple(ax * q[0] + sd * q[3])])
    for _ in range(400):
        out = R(bnd).difference(fit).area
        if out < 0.01:
            break
        best = None
        for i in range(4):
            q = list(bnd)
            q[i] += step if i in (0, 2) else -step
            if q[1] - q[0] < 2 or q[3] - q[2] < 2:
                continue
            o = R(q).difference(fit).area
            gain = (out - o) / max(R(bnd).area - R(q).area, 1e-6)
            if best is None or gain > best[0]:
                best = (gain, q)
        if best is None:
            return None
        bnd = best[1]
    if R(bnd).difference(fit).area > 0.01 or R(bnd).area < 0.5 * p.area:
        return None
    a, b, c, d = (ax * bnd[0] + sd * bnd[2], ax * bnd[1] + sd * bnd[2], ax * bnd[1] + sd * bnd[3],
                  ax * bnd[0] + sd * bnd[3])
    if np.linalg.norm(b - a) < np.linalg.norm(d - a) and ang is None:
        a, b, c, d = b, c, d, a
    return [np.asarray(a), np.asarray(b), np.asarray(c), np.asarray(d)]


def curved_parapet(m, f, L, z0, rise, thick=0.4, n=8):
    """A parapet slab on facade f (u 0..L, w -thick..0) whose top falls from
    z0 + rise at u = 0 to z0 at u = L on a quarter curve: outer face out, inner
    face in, the top cap up. / 四分円の頂部を持つパラペット（外面は外、内面は内、天端は上）。"""
    us = np.linspace(0.0, L, n + 1)
    zs = [z0 + rise * math.sin(math.pi / 2 * (1 - u / L)) for u in us]
    for (ua, za), (ub, zb) in zip(zip(us, zs), zip(us[1:], zs[1:])):
        po, qo = f.P(ua, 0.0, 0)[:2], f.P(ub, 0.0, 0)[:2]
        pi, qi = f.P(ua, -thick, 0)[:2], f.P(ub, -thick, 0)[:2]
        wall_top(m, WHITE, po, qo, z0 - 0.5, za, zb)              # out / 外
        wall_top(m, WHITE, qi, pi, z0 - 0.5, zb, za)              # in / 内
        m.quad(WHITE, (po[0], po[1], za), (qo[0], qo[1], zb), (qi[0], qi[1], zb), (pi[0], pi[1], za))
    f.side(WHITE, 0.0, -thick, 0.0, z0 - 0.5, zs[0], -1)
    f.side(WHITE, L, -thick, 0.0, z0 - 0.5, zs[-1] + 0.01, 1) if zs[-1] > z0 - 0.49 else None


def gable_rect(p):
    """(a, b, c, d) of the oriented rectangle, a->b the long side. / 外接矩形。"""
    r = orient(p.minimum_rotated_rectangle, 1.0)
    c = [np.array(q) for q in list(r.exterior.coords)[:4]]
    e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
    k = 0 if e[0] >= e[1] else 1
    return [c[(k + i) % 4] for i in range(4)]


def tiled_lime(f, u0, u1, z0, z1):
    """Blk 1A's ground floor: lime / green vertical stripes (theplan photo). / 黄緑の縦縞。"""
    f.m.quad("tp_lime", f.P(u0, 0.0, z0), f.P(u1, 0.0, z0), f.P(u1, 0.0, z1), f.P(u0, 0.0, z1),
             (u0 / 2.4, 0), (u1 / 2.4, 0), (u1 / 2.4, 1), (u0 / 2.4, 1))


def write_textures(tex_dir):
    """Drawn textures (procedural; colours from description.md, nothing taken
    from imagery). / 描画テクスチャ（description.md の色、画像は使わない）。"""
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

    save(WHITE, X.paint(512, (214, 216, 219), 0.05), 0.85, 0.0, 0.6)
    save(NAVY, X.paint(128, (24, 46, 84), 0.03), 0.6, 0.2)
    save(FASCIA, X.paint(128, (28, 42, 72), 0.03), 0.5, 0.4)
    save(GREY, X.paint(128, (150, 154, 158), 0.04), 0.7, 0.2)
    save("tp_soffit", X.paint(128, (232, 233, 235), 0.02), 0.7)
    # blue-violet standing-seam metal (90,95,140; lilac from above) / 青紫の立平葺き
    save(ROOF, X.stripes(256, (86, 90, 136), (100, 104, 150), 16, vertical=True), 0.45, 0.5, 1.2)
    save("tp_greyroof", X.stripes(128, (120, 124, 130), (138, 142, 148), 8, vertical=True), 0.5, 0.4, 1.0)
    save("tp_roofwhite", X.paint(256, (205, 206, 204), 0.06), 0.8)
    save("tp_spandrel", X.paint(128, (85, 98, 90), 0.03), 0.6)
    save("tp_yellow", X.paint(128, (214, 176, 72), 0.04), 0.7)
    save("tp_purple", X.paint(128, (112, 78, 140), 0.04), 0.6, 0.3)
    save("tp_canopy", X.paint(128, (40, 72, 56), 0.03), 0.5, 0.4)
    save("tp_charcoal", X.paint(128, (44, 38, 41), 0.03), 0.5, 0.3)
    save("tp_redletter", X.paint(64, (205, 32, 42), 0.02), 0.4, 0.1)
    save("tp_red", X.paint(128, (150, 28, 44), 0.04), 0.7)
    save("tp_green", X.paint(128, (80, 112, 82), 0.05), 0.8)
    save("tp_ochre", X.paint(128, (200, 128, 54), 0.04), 0.6)
    save("tp_orange", X.paint(128, (222, 132, 62), 0.04), 0.6)
    save("tp_beige", X.paint(256, (204, 190, 160), 0.05), 0.85)
    save("tp_darkbox", X.paint(256, (52, 56, 60), 0.04), 0.6, 0.2)
    save("tp_railgrey", X.paint(64, (150, 156, 160), 0.02), 0.4, 0.6)
    save("tp_fan", X.stripes(64, (70, 74, 78), (120, 124, 128), 6), 0.5, 0.5)
    save("tp_door", X.paint(64, (110, 112, 116), 0.03), 0.5, 0.4)
    save("tp_basin", X.paint(128, (236, 236, 232), 0.03), 0.6)
    save("tp_seat", X.paint(128, (188, 186, 180), 0.05), 0.85)
    save("tp_louvre", X.stripes(128, (92, 96, 102), (132, 136, 140), 12), 0.6, 0.3, 1.2)
    save("tp_slot", X.stripes(128, (40, 52, 70), (90, 110, 130), 16), 0.2, 0.3)
    save("tp_perf", X.stripes(128, (128, 132, 136), (150, 154, 158), 4, vertical=True), 0.6, 0.4, 0.8)
    rng = np.random.default_rng(3)

    def grid(frame, g0, g1, cols, rows, n=256, fw=7, spandrel=None):
        y = np.linspace(0, 1, n)[:, None, None]
        g = np.array(g1)[None, None, :] * (1 - y) + np.array(g0)[None, None, :] * y
        im = Image.fromarray(np.repeat(g, n, axis=1).astype(np.uint8))
        d = ImageDraw.Draw(im)
        if spandrel is not None:
            d.rectangle([0, n - n // 5, n, n], fill=spandrel)
        d.rectangle([0, 0, n - 1, n - 1], outline=frame, width=fw)
        for c in range(1, cols):
            x = int(n * c / cols)
            d.rectangle([x - fw // 2, 0, x + fw // 2, n], fill=frame)
        for r in range(1, rows):
            yy = int(n * r / rows)
            d.rectangle([0, yy - fw // 2, n, yy + fw // 2], fill=frame)
        return im

    save("tp_win", grid((24, 46, 84), (34, 48, 66), (92, 112, 134), 1, 1, fw=18), 0.1, 0.2)
    save("tp_ribbon", grid((30, 52, 92), (40, 56, 72), (110, 130, 150), 1, 1, fw=10), 0.1, 0.2)
    save("tp_curtain", grid((215, 218, 222), (14, 44, 96), (60, 100, 150), 2, 2, fw=6), 0.06, 0.4)
    save("tp_curtaingrid", grid((228, 230, 232), (70, 92, 118), (140, 160, 178), 2, 3, fw=8,
                                spandrel=(40, 46, 56)), 0.08, 0.3)
    save("tp_glassdark", grid((40, 42, 46), (26, 30, 36), (60, 70, 80), 2, 1, fw=6), 0.08, 0.3)
    save("tp_shopglass", grid((60, 62, 66), (60, 70, 72), (150, 140, 110), 3, 1, fw=6), 0.1, 0.2)
    save("tp_glass1a", grid((210, 214, 218), (70, 88, 104), (128, 146, 160), 2, 1, fw=5), 0.08, 0.3)
    # gallery back wall: doors / windows behind the balustrade / 回廊の奥の壁
    im = X.paint(512, (205, 206, 203), 0.04).resize((512, 512))
    d = ImageDraw.Draw(im)
    d.rectangle([40, 120, 220, 470], fill=(60, 70, 78))
    d.rectangle([50, 130, 210, 460], fill=(120, 140, 150))
    d.rectangle([290, 160, 480, 340], fill=(60, 70, 78))
    d.rectangle([298, 168, 472, 332], fill=(130, 150, 158))
    save("tp_gallery", im, 0.7)
    # balcony wall of the round blocks: dark window recesses / 円形棟のバルコニー奥の壁
    im = X.paint(256, (200, 202, 204), 0.04).resize((256, 256))
    d = ImageDraw.Draw(im)
    d.rectangle([30, 40, 226, 200], fill=(44, 50, 56))
    d.rectangle([40, 50, 216, 190], fill=(70, 82, 92))
    save("tp_balc", im, 0.6)
    # lilac-grey pavers (158,144,165) / 紫灰色の舗装
    n = 256
    base = X.tint(X.noise(n), np.array((150, 137, 158)), np.array((168, 154, 175)))
    im = Image.fromarray(base)
    d = ImageDraw.Draw(im)
    for i in range(0, n, 32):
        d.line([(0, i), (n, i)], fill=(128, 118, 136), width=2)
        d.line([(i, 0), (i, n)], fill=(128, 118, 136), width=2)
    save("tp_paver", im, 0.8, 0.0, 1.2)
    save("tp_turf", X.grass(512, (34, 70, 40), (56, 96, 52)), 0.95)
    save("tp_planter", X.leaves(256, (36, 72, 34), (80, 120, 56)), 0.9)
    save("tp_greenroof", X.grass(512, (60, 92, 48), (96, 130, 66)), 0.95)
    save("tp_courtred", X.paint(256, (150, 66, 52), 0.05), 0.8)
    # the 9A deck mural / court pattern (OneMap colours) / 9A デッキの壁画・コート模様
    W = 1024
    im = Image.new("RGB", (W, W), (156, 150, 160))
    d = ImageDraw.Draw(im)
    cols = [(196, 60, 52), (232, 186, 64), (52, 92, 156), (60, 128, 92), (220, 220, 214)]
    for k in range(18):
        x0, y0 = rng.integers(0, W - 200, 2)
        w, h = rng.integers(80, 320, 2)
        d.rectangle([x0, y0, x0 + w, y0 + h], fill=tuple(int(v) for v in cols[k % len(cols)]))
    for k in range(10):
        x0, y0 = rng.integers(0, W - 260, 2)
        r = int(rng.integers(40, 140))
        d.ellipse([x0, y0, x0 + 2 * r, y0 + 2 * r], outline=(240, 240, 236), width=10)
    save("tp_mural", im, 0.8)
    # striped lawn panels: grass rows with gravel strips (2010 photo) / 砂利の縞の芝生
    n = 256
    g = X.tint(X.noise(n), np.array((50, 92, 40)), np.array((88, 132, 60)))
    gr = X.tint(X.noise(n, (8, 32), (0.5, 0.5)), np.array((140, 140, 136)), np.array((190, 190, 184)))
    im = g.copy()
    for i in range(n):
        if (i // 32) % 2 == 1 and (i % 32) > 22:
            im[i, :] = gr[i, :]
    save("tp_lawnstripe", Image.fromarray(im.astype(np.uint8)), 0.95)
    # the planted channel: dark water with lily pads / 睡蓮の水路
    im = Image.fromarray(X.tint(X.noise(512), np.array((40, 64, 58)), np.array((62, 88, 72))))
    d = ImageDraw.Draw(im)
    for k in range(260):
        x, y = rng.integers(0, 512, 2)
        r = int(rng.integers(5, 16))
        c = (int(rng.integers(60, 110)), int(rng.integers(110, 160)), int(rng.integers(50, 80)))
        d.ellipse([x - r, y - r, x + r, y + r], fill=c)
    save("tp_marsh", im, 0.3)
    # the stadium football pitch (lines) / サッカー場
    save("tp_football", X.court(104, 68, "football", (58, 112, 54), px_per_m=10), 0.95)
    # lime / green stripes of Blk 1A's ground floor / 1A の黄緑の縦縞
    im = Image.new("RGB", (256, 128), (230, 232, 228))
    d = ImageDraw.Draw(im)
    for x0, c in ((0, (170, 205, 70)), (64, (90, 160, 80)), (128, (205, 220, 90)), (192, (230, 232, 228))):
        d.rectangle([x0, 0, x0 + 56, 128], fill=c)
    save("tp_lime", im, 0.6)
    # rooftop tennis fence (dark mesh, see-through look) / 屋上の金網
    im = Image.new("RGB", (64, 64), (50, 60, 56))
    d = ImageDraw.Draw(im)
    for i in range(0, 64, 8):
        d.line([(i, 0), (i, 64)], fill=(110, 120, 116), width=1)
        d.line([(0, i), (64, i)], fill=(110, 120, 116), width=1)
    save("tp_fence", im, 0.6, 0.4)
    save("tp_track", X.paint(256, (176, 72, 58), 0.06), 0.85)
    for nm, bg, bar in (("tp_railbars", (205, 212, 206), (120, 126, 130)), ("tp_railgreen", (78, 120, 70), (40, 92, 52))):
        im = Image.new("RGB", (64, 64), bg)
        d = ImageDraw.Draw(im)
        for x in range(4, 64, 16):
            d.rectangle([x, 0, x + 3, 64], fill=bar)
        d.rectangle([0, 28, 64, 31], fill=bar)
        save(nm, im, 0.5, 0.4)
    save("tp_planterbox", X.paint(128, (196, 192, 184), 0.05), 0.85)
    save("tp_pooldeck", X.concrete(256, (214, 214, 208), (232, 232, 226), joints=4), 0.7)
    save("tp_lampcol", X.paint(64, (40, 42, 46), 0.02), 0.4, 0.5)
    save("tp_glow", X.stripes(64, (230, 120, 220), (120, 220, 240), 16), 0.3)
    save("tp_bench", X.paint(64, (214, 168, 92), 0.04), 0.6)
    save("tp_flower", X.leaves(128, (60, 110, 50), (220, 80, 120)), 0.9)
    save("tp_pot", X.paint(64, (180, 100, 70), 0.04), 0.8)
    save("tp_dome", X.stripes(128, (70, 96, 150), (82, 110, 166), 8, vertical=True), 0.4, 0.5, 0.8)
    return out
