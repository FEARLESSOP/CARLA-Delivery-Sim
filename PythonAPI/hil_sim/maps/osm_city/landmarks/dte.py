"""Downtown East (1 Pasir Ris Close, Singapore), modelled in detail (maps/.venv).

Sources (2025-2026, Photo Getter, build/PasirRisTampines/dte_photos): the z20
satellite tiles (scale from tile overlap), OneMap 3D views and their height
tooltips (Google Earth has no 3-D here), Street View on Pasir Ris Drive 3 and
Pasir Ris Close, the 2025 wayfinding map, DP Architects' photos, a 2025 photo
of the D'Marquee front. Outlines traced on a map-frame mosaic of the tiles
(build/PasirRisTampines/dte_trace.json, tile pixels as prcs_trace.json).
Nothing is copied as texture: every texture is drawn here.

  E!Hub        5-storey grey metal-panel box (26 m, plant 28 m) wrapped S and E
               in a dark steel exoskeleton of leaning braced frames with round
               discs; the green glass curtain wall on the east, the curved glass
               SE corner with stacked balconies, a striped awning; HVAC roof
  car park     6 open decks (21 m, the north end 12 m): slab bands, dark voids,
               columns, the steel brace frame and discs on the east, solar roof
  E!Avenue     the 2-storey solar-roofed strip (Market Square / The Alley /
               Event Square, 14 m) with its white canopy on tall slender round
               columns; the 4-storey core and the north wing (glass curtain
               wall, grey spandrel ledges, colonnade to Wild Wild Wet); the 20 m
               east box in white / grey vertical strips, canopy slab, shutters,
               louvred plant screen
  D'Marquee    the 2025 hall: orange and glazed ground floor, purple fascia,
               purple fluted band with banners, green battens with the name,
               the white wave canopy; white solar roof
  PlayHaus     the grey standing-seam hipped hall (9 / 19 m) with its lantern
  Aranda       the clubhouse (red clay-tile hip 10 / 19 m, lantern tower), the
               pool, the sports hall, the low block; the 4-storey wing with its
               many red pyramidal roofs
  loop         Playon's hipped block, its annex, the pyramid kiosk, a small
               block and the covered walk in the drop-off loop island; the
               DOWNTOWN EAST planter at the entry
  Wild Wild    pools and the lazy river (OSM), the painted pool deck, tube
  Wet          slides along the mapped slides, the slide towers, the Tornado
               funnel, the white tensile shade, kiosks with brown pyramid
               roofs, the swim school, the plant block and cooling units
  D'Resort     the V of two dark hotel wings (26-27 m) with ribbon windows,
               copper-striped cores and the logo; the chalet chains (3 storeys:
               beige ground floor, white upper band with grey fins, galvanised
               post railings on the corridors, checker end walls), sand
               courtyards; Gallop Stable and two courts
  frontage     the covered walkway along Pasir Ris Drive 3

Roads win: every footprint keeps GAP off the road network (the drop-off loop,
the D'Resort road and the hotel ramp are OSM service roads in it) and every
projection is cut back from it; a last pass drops any triangle that would
still touch it (reported). The Begonia open car park and Aranda's small car
park stay generic (inner() leaves them out).

Coordinates: the map frame (x east, y north, metres).

ダウンタウン・イースト（シンガポール、パシリス）の詳細モデル。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from landmarks.prcs import text_poly
from mesh import Mesh, polys

NAME = "Downtown East"
TAG = "DTE"
TRACE = "dte_trace.json"
MAP = "PasirRisTampines"
Z0 = 0.15                 # the raised ground every landmark stands on / 敷地面
GAP = 1.2                 # footprints this far off the road network / 道路網からの離れ
ROOM = 0.4                # projections this far off it / 張り出しの離れ
SITE_NAMES = ("Downtown East", "D’Resort @ Downtown East", "D'Resort @ Downtown East")


def enabled(out, meta):
    """Only on PasirRisTampines, with its trace. / この地図とトレースがある時だけ。"""
    return meta.get("name") == MAP and os.path.exists(os.path.join(out, TRACE))


# -- primitives (tested in test_dte.py: every face points out) / 基本形状（外向き面は test_dte で確認）
def _frame(d):
    """Two unit vectors square to d (3-D). / d に直交する 2 単位ベクトル。"""
    d = d / np.linalg.norm(d)
    ref = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    a = np.cross(d, ref)
    a /= np.linalg.norm(a)
    return a, np.cross(d, a)


def beam(m, mat, p0, p1, r=0.15, caps=False):
    """A square tube of half-width r from p0 to p1 (3-D), faces out. / 3D の角パイプ。"""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    if np.linalg.norm(d) < 1e-3:
        return
    a, b = _frame(d)
    c = [a * r + b * r, -a * r + b * r, -a * r - b * r, a * r - b * r]   # CCW about d / d 周りに反時計
    for k in range(4):
        q0, q1 = c[k], c[(k + 1) % 4]
        m.quad(mat, p0 + q0, p0 + q1, p1 + q1, p1 + q0)
    if caps:
        m.quad(mat, p1 + c[0], p1 + c[1], p1 + c[2], p1 + c[3])
        m.quad(mat, p0 + c[3], p0 + c[2], p0 + c[1], p0 + c[0])


def tube(m, mat, pts, r=0.6, seg=6):
    """A closed tube along a 3-D polyline (water slides), faces out. / 3D 折れ線に沿う管。"""
    P = [np.asarray(p, float) for p in pts]
    if len(P) < 2:
        return
    rings = []
    for i, p in enumerate(P):
        t = (P[min(i + 1, len(P) - 1)] - P[max(i - 1, 0)])
        if np.linalg.norm(t) < 1e-6:
            t = np.array([1.0, 0, 0])
        t = t / np.linalg.norm(t)
        # a stable side vector: horizontal, square to the travel / 進行に直角な水平ベクトル
        s = np.array([-t[1], t[0], 0.0])
        if np.linalg.norm(s) < 1e-6:
            s = np.array([1.0, 0, 0])
        s /= np.linalg.norm(s)
        u = np.cross(t, s)
        rings.append([p + r * (math.cos(2 * math.pi * k / seg) * s + math.sin(2 * math.pi * k / seg) * u)
                      for k in range(seg)])
    for i in range(len(P) - 1):
        for k in range(seg):
            a0, a1 = rings[i][k], rings[i][(k + 1) % seg]
            b0, b1 = rings[i + 1][k], rings[i + 1][(k + 1) % seg]
            m.quad(mat, a0, a1, b1, b0)


def disc(m, mat, rim, c, n, R, seg=16, t=0.25):
    """A vertical disc (brand disc on the frame) facing n (2-D), drawn texture on
    the front, a rim and a back. / 正面に描画テクスチャのある垂直の円盤。"""
    c = np.asarray(c, float)
    n3 = np.array([n[0], n[1], 0.0]) / math.hypot(n[0], n[1])
    s = np.array([-n3[1], n3[0], 0.0])          # the viewer's right when facing it / 正面から見て右
    up = np.array([0.0, 0.0, 1.0])
    front, back = c + n3 * t / 2, c - n3 * t / 2
    ang = [2 * math.pi * k / seg for k in range(seg)]
    F = [front] + [front + R * (math.cos(a) * s + math.sin(a) * up) for a in ang]
    UV = [(0.5, 0.5)] + [(0.5 + 0.5 * math.cos(a), 0.5 + 0.5 * math.sin(a)) for a in ang]
    I = []
    for k in range(seg):
        I += [0, 1 + k, 1 + (k + 1) % seg]
    m.tris(mat, F, n3, UV, I)
    Bk = [back] + [back + R * (math.cos(a) * s + math.sin(a) * up) for a in ang]
    Ib = []
    for k in range(seg):
        Ib += [0, 1 + (k + 1) % seg, 1 + k]
    m.tris(rim, Bk, -n3, [(0, 0)] * len(Bk), Ib)
    for k in range(seg):
        a0, a1 = F[1 + k], F[1 + (k + 1) % seg]
        b0, b1 = Bk[1 + k], Bk[1 + (k + 1) % seg]
        m.quad(rim, a0, b0, b1, a1)


def rect_axes(poly):
    """(a, b, c, d) corners of the oriented rectangle, a->b the long side. / 外接矩形の角。"""
    r = orient(poly.minimum_rotated_rectangle, 1.0)
    c = [np.array(q) for q in list(r.exterior.coords)[:4]]
    e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
    k = 0 if e[0] >= e[1] else 1
    return [c[(k + i) % 4] for i in range(4)]


def hip(m, poly, z0, z1, mat, ov=0.5, soffit="concrete", inner=None):
    """A hipped roof (z0 at the eaves, z1 at the ridge) over the footprint's
    oriented rectangle grown by ov; with ``inner`` (a, b: inner square half
    sizes) it stops at that height as a frustum with a flat top. Faces out
    and up. / 外接矩形の寄棟屋根（inner で途中まで: 台形）。"""
    a, b, c, d = rect_axes(poly)
    ctr = (a + b + c + d) / 4
    ax = (b - a) / np.linalg.norm(b - a)
    sd = (d - a) / np.linalg.norm(d - a)
    Lh = np.linalg.norm(b - a) / 2 + ov
    Wh = np.linalg.norm(d - a) / 2 + ov
    P = lambda u, v, z: (ctr[0] + ax[0] * u + sd[0] * v, ctr[1] + ax[1] * u + sd[1] * v, z)
    o = [P(-Lh, -Wh, z0), P(Lh, -Wh, z0), P(Lh, Wh, z0), P(-Lh, Wh, z0)]
    if inner is None:
        ri = max(Lh - Wh, 0.0)
        i_ = [P(-ri, 0, z1), P(ri, 0, z1), P(ri, 0, z1), P(-ri, 0, z1)]
    else:
        iu, iv, zt = inner
        i_ = [P(-iu, -iv, zt), P(iu, -iv, zt), P(iu, iv, zt), P(-iu, iv, zt)]
    for k in range(4):
        q = [o[k], o[(k + 1) % 4], i_[(k + 1) % 4], i_[k]]
        if math.dist(q[2], q[3]) < 1e-6:
            m.tris(mat, q[:3], (0, 0, 1), [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        else:
            m.quad(mat, *q)
    if inner is not None:
        m.quad(mat, i_[0], i_[1], i_[2], i_[3])
    m.quad(soffit, o[1], o[0], o[3], o[2])                  # soffit, facing down / 軒裏
    return o


def gable(m, poly, z0, z1, mat, wall, ov=0.5):
    """A gable roof over the oriented rectangle, ridge along the long side. / 切妻屋根。"""
    a, b, c, d = rect_axes(poly)
    ax = (b - a) / np.linalg.norm(b - a)
    sd = (d - a) / np.linalg.norm(d - a)
    a2, b2, c2, d2 = a - ax * ov - sd * ov, b + ax * ov - sd * ov, c + ax * ov + sd * ov, d - ax * ov + sd * ov
    r1, r2 = (a2 + d2) / 2, (b2 + c2) / 2
    P = lambda q, z: (q[0], q[1], z)
    m.quad(mat, P(a2, z0), P(b2, z0), P(r2, z1), P(r1, z1))
    m.quad(mat, P(c2, z0), P(d2, z0), P(r1, z1), P(r2, z1))
    m1, m2 = (a + d) / 2, (b + c) / 2
    m.tris(wall, [P(d, z0), P(a, z0), P(m1, z1)], np.r_[-ax, 0], [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
    m.tris(wall, [P(b, z0), P(c, z0), P(m2, z1)], np.r_[ax, 0], [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
    m.quad("concrete", P(b2, z0), P(a2, z0), P(d2, z0), P(c2, z0))


def two_sided(m, mat, a, b, c, d, gap=0.006):
    """A thin sheet seen from both sides (membranes, sails): two quads 6 mm apart. / 両面の薄い膜。"""
    a, b, c, d = (np.asarray(p, float) for p in (a, b, c, d))
    n = np.cross(b - a, d - a)
    ln = np.linalg.norm(n)
    if ln < 1e-9:
        return
    o = n / ln * gap / 2
    m.quad(mat, a + o, b + o, c + o, d + o)
    m.quad(mat, d - o, c - o, b - o, a - o)


def face_in(f, mat, u0, u1, z0, z1, w):
    """A wall patch at offset w facing INTO the building (-n). / 内向きの壁面。"""
    if u1 - u0 < 1e-3 or z1 - z0 < 1e-3:
        return
    f.m.quad(mat, f.P(u1, w, z0), f.P(u0, w, z0), f.P(u0, w, z1), f.P(u1, w, z1),
             (u1, z0), (u0, z0), (u0, z1), (u1, z1))


def grid_face(f, mat, u0, u1, z0, z1, w, tu, tv):
    """A wall patch whose texture tiles every tu metres along and tv up. / 模様の繰り返しを指定した壁面。"""
    if u1 - u0 < 1e-3 or z1 - z0 < 1e-3:
        return
    f.m.quad(mat, f.P(u0, w, z0), f.P(u1, w, z0), f.P(u1, w, z1), f.P(u0, w, z1),
             (u0 / tu, z0 / tv), (u1 / tu, z0 / tv), (u1 / tu, z1 / tv), (u0 / tu, z1 / tv))


def all_edges(poly):
    """Edges of the outer ring and the holes (a, b, L, outward normal: away
    from the material, into a courtyard for a hole). / 外周と中庭の辺。"""
    p = orient(poly, 1.0)
    out = []
    for ring, hole in [(p.exterior, False)] + [(r, True) for r in p.interiors]:
        c = list(ring.coords)
        for a, b in zip(c, c[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy)
            if L < 0.3:
                continue
            out.append((np.array(a), np.array(b), L, np.array([dy / L, -dx / L]), hole))
    return out


def largest(g):
    ps = polys(g)
    return max(ps, key=lambda q: q.area) if ps else None


def drop_over(m, no_go):
    """Remove every triangle that touches ``no_go`` (the last guard: roads win).
    Returns the number dropped. / 道路に触れる三角形を除く（最後の安全策）。"""
    shapely.prepare(no_go)
    gone = 0
    for mat, part in m.parts.items():
        V, N, UV, I, n = part
        if not V:
            continue
        Vc = np.concatenate(V)
        Nc = np.concatenate(N)
        Uc = np.concatenate(UV)
        Ic = np.concatenate(I)
        T = Vc[Ic]
        tri = shapely.polygons(np.concatenate([T[:, :, :2], T[:, :1, :2]], axis=1))
        flat = np.abs(shapely.area(tri)) < 1e-6
        # vertical faces by their base edge / 垂直面は底辺で
        base = shapely.linestrings(T[:, :2, :2])
        hit = np.where(flat, shapely.intersects(no_go, base), shapely.intersects(no_go, tri))
        if hit.any():
            gone += int(hit.sum())
            if os.environ.get("DTE_DEBUG"):
                print("  dropped %s: %d at %s" % (mat, int(hit.sum()), np.round(T[hit].mean(axis=1)[:3], 1).tolist()))
            part[0], part[1], part[2], part[3] = [Vc], [Nc], [Uc], [Ic[~hit]]
    return gone


class DTE(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        C = T["centre"]

        def P(tile, px, py):
            t = T["tiles"][tile]
            cx, cy = fr.xy(t["lon"], t["lat"])
            return (cx + (px - C[0]) * t["mpp_x"], cy - (py - C[1]) * t["mpp_y"])
        self.P = P
        pts = lambda d: [P(d["tile"], *q) for q in d["pts"]]
        # the site: OSM's Downtown East and D'Resort areas / 敷地: OSM の 2 区画
        site = [p for p, t in L.osm.areas(lambda t: t.get("name") in SITE_NAMES and "building" not in t)]
        self.site = unary_union(site).buffer(0) if site else None
        # what stays generic: the open car parks (their aisles and bays) / 汎用のまま残す駐車場
        A = T["areas"]
        self.keep_generic = unary_union([Polygon(pts(A["begonia_keep"])), Polygon(pts(A["aranda_keep"]))])
        self.drive_snapshot = L.driveways
        self.parking_snapshot = L.parking
        self.road = L.road_all
        self.hard = unary_union([L.road_all, L.islands])
        # check_roads.py --site counts the site's own drives and car-park aisles as road too
        # 敷地内の車路・駐車通路も道路扱い（check_roads --site）
        near = self.site.buffer(5.0) if self.site is not None else box(0, 0, 0, 0)
        # every driving lane near the site, raised ones too (the hotel ramp: the
        # carriageway outline leaves raised decks out; check_drive tests every lane
        # from its own xodr z) / 敷地付近の全車線（高架のホテル斜路も）
        self.lanes_near, self.raised = [], []
        near30 = near.buffer(30)
        shapely.prepare(near30)
        for ln in getattr(L, "lanes", []):
            lp = ln["pts"]
            if not any(near30.contains(Point(q["c"][0], q["c"][1])) for q in lp[::4] + lp[-1:]):
                continue
            ring = [q["l"] for q in lp] + [q["r"] for q in lp][::-1]
            pg = Polygon(ring).buffer(0) if len(ring) >= 3 else Polygon()
            if pg.is_empty:
                continue
            self.lanes_near.append(pg)
            if max(q["z"] for q in lp) > 1.0:
                self.raised.append((pg, ln))
        self.lane_area = unary_union(self.lanes_near) if self.lanes_near else Polygon()
        self.no_go = unary_union([self.hard, L.driveways.intersection(near), self.lane_area.buffer(0.4)])
        self.no_go_gap = self.no_go.buffer(GAP)
        self.room = self.no_go.buffer(ROOM)
        self.cut_log = []
        self.blocks = {}
        for k, b in T["blocks"].items():
            if "line" in b:
                ln = LineString([P(b["tile"], *q) for q in b["line"]])
                poly = ln.buffer(b["w"] / 2, cap_style="flat", join_style="mitre")
            else:
                poly = Polygon(pts(b))
            self.blocks[k] = dict(b, poly=self.fit(k, orient(poly, 1.0)), traced=poly)
        if self.site is None:
            self.site = unary_union([b["traced"] for b in self.blocks.values()]).convex_hull.buffer(20)
        # the chalet chains: centre lines, 12.5 m deep, one solid each / シャレーの連なり
        hotel = unary_union([self.blocks[k]["poly"] for k in ("hotel_w", "hotel_e")])
        self.chalets = {}
        self.chalet_line = {}
        lines = {k: LineString([P(c["tile"], *q) for q in c["line"]]) for k, c in T["chalets"].items()}
        for k, c in T["chalets"].items():
            ln = lines[k]
            if not ln.is_ring:
                # an end that meets another chain runs on into it (else a dark wedge
                # stays between a square cap and the other chain's wall) / 他の棟に接する端は延ばす
                cp = [np.array(q) for q in ln.coords]
                for end, nxt in ((0, 1), (-1, -2)):
                    near = min(lines[o].distance(Point(cp[end])) for o in lines if o != k)
                    if near < 8.0:
                        d = cp[end] - cp[nxt]
                        cp[end] = cp[end] + d / np.linalg.norm(d) * (near + 6.5)
                ln = LineString(cp)
            poly = ln.buffer(c["w"] / 2, cap_style="flat", join_style="mitre")
            if ln.is_ring:
                poly = Polygon(ln.coords).buffer(c["w"] / 2, join_style="mitre").difference(
                    Polygon(ln.coords).buffer(-c["w"] / 2, join_style="mitre"))
            self.chalets[k] = (poly, c["h"])
            self.chalet_line[k] = ln
        # one polygon per chain, the earlier chains win where they meet / 交わる所は先の棟
        done = hotel.buffer(3.0)
        for k in list(self.chalets):
            poly, h = self.chalets[k]
            poly = poly.difference(done)
            done = done.union(poly)
            g = self.fit("chalet_" + k, poly, keep_all=True)
            self.chalets[k] = (g, h)
        self.areas = {}
        for k, v in A.items():
            if isinstance(v, list):
                self.areas[k] = [Polygon(pts(q)) for q in v]
            elif len(v["pts"]) == 1:
                self.areas[k] = Point(P(v["tile"], *v["pts"][0]))
            elif k in ("strip_canopy", "link_bridge"):
                self.areas[k] = LineString(pts(v))
            else:
                self.areas[k] = Polygon(pts(v))
        foot = [b["poly"] for b in self.blocks.values() if b["poly"] is not None] + \
               [g for g, h in self.chalets.values() if g is not None and not g.is_empty]
        self.footprint = unary_union(foot)
        self.www = largest(unary_union([p for p, t in L.osm.areas(lambda t: t.get("leisure") == "water_park")
                                        if p.intersects(self.site)]))
        self.dresort = largest(unary_union([p for p, t in L.osm.areas(
            lambda t: (t.get("name") or "").startswith("D") and "Resort" in (t.get("name") or "")
            and "building" not in t)]))
        inw = lambda p: self.www is not None and self.www.buffer(5).contains(p.representative_point())
        self.pools = unary_union([p for p, t in L.osm.areas(
            lambda t: t.get("leisure") == "swimming_pool" or t.get("name") in ("Shiok River",)
            or t.get("natural") == "water") if self.site.contains(p.representative_point())])
        self.pools = self.pools.difference(self.room).difference(self.footprint.buffer(0.5))
        self.slides = [(ln, t) for ln, t, w in L.osm.lines("attraction") if t.get("attraction") == "water_slide"
                       and self.site.contains(ln.interpolate(0.5, normalized=True))]
        self.kiosks = [(p, t, w) for p, t, w in L.osm.areas_id(lambda t: "building" in t)
                       if inw(p) and not self.footprint.buffer(1.0).intersects(p)]
        self.dropped = 0
        self._parts = None
        self._ground = None
        print("dte: %d blocks, %d chalet chains, %d slides, %d kiosks/towers, cut back for roads: %s"
              % (len(self.blocks), len(self.chalets), len(self.slides), len(self.kiosks),
                 ", ".join("%s %.0f m2" % c for c in self.cut_log if c[1] > 1.0) or "none"), flush=True)

    def fit(self, name, poly, keep_all=False):
        """Roads win: the footprint keeps GAP off the road network. A traced
        rectangle stays a rectangle: each side that comes too close moves in
        (its roof and facades stay square); other shapes are cut (largest
        piece). / 道路優先: 矩形は辺ごとに後退、他は切り取り。"""
        c = list(orient(poly, 1.0).exterior.coords)[:-1] if poly.geom_type == "Polygon" else []
        rect = name.startswith("hotel") or "ridge" in self.T["blocks"].get(name, {})
        if len(c) == 4 and not keep_all and rect:
            q = [np.array(p, float) for p in c]
            for _ in range(200):
                moved = False
                for i in range(4):
                    a, b = q[i], q[(i + 1) % 4]
                    if LineString([a, b]).distance(self.no_go) < GAP:
                        d = b - a
                        nrm = np.array([d[1], -d[0]]) / max(np.linalg.norm(d), 1e-9)
                        q[i], q[(i + 1) % 4] = a - nrm * 0.25, b - nrm * 0.25
                        moved = True
                if not moved:
                    break
            r = Polygon(q)
            self.cut_log.append((name, poly.area - r.area))
            if moved or not r.is_valid or r.area < 0.3 * poly.area:
                return None
            return orient(r, 1.0)
        g = poly.difference(self.no_go_gap)
        cut = poly.area - g.area
        self.cut_log.append((name, cut))
        if keep_all:
            return MultiPolygon([q for q in polys(g) if q.area > 20.0]) if polys(g) else None
        return orient(largest(g), 1.0) if largest(g) is not None else None

    # -- layout / 配置
    def inner(self):
        return self.site.buffer(-0.6, join_style="mitre").difference(self.keep_generic)

    def keepout(self):
        """Where the generic trees and furniture must not go. / 汎用の木と小物を置かない所。"""
        # generic trees and furniture keep 7.5 m (a big crown) off every lane in and round the site,
        # raised ramp lanes included (props.py does not know raised lanes: crowns
        # stood in the hotel ramp, check_drive "blocked") / 全車線から 7.5 m（高架の斜路も）
        k = [self.footprint.buffer(3.0), self.pools.buffer(1.0), self.lane_area.buffer(7.5)]
        if self.www is not None:
            k.append(self.www)
        k += [c.buffer(1.0) for c in self.areas["courts"]]
        return unary_union(k)

    # -- ground / 地面
    def zones(self):
        if self._ground is not None:
            return self._ground
        inner = self.inner().difference(self.hard.buffer(0.05))
        foot = self.footprint
        drive = self.drive_snapshot.intersection(inner).difference(foot)
        free = inner.difference(foot).difference(drive)
        www = free.intersection(self.www) if self.www is not None else Polygon()
        pools = self.pools.intersection(free)
        dres = free.intersection(self.dresort).difference(www) if self.dresort is not None else Polygon()
        aranda = free.intersection(box(690, 1370, 800, 1700)).difference(www)
        garden = free.intersection(self.areas["loop_garden"])
        # chalet courtyards (the hex ring's holes) are sand / 中庭は砂
        sand = unary_union([Polygon(r) for g, h in self.chalets.values() if g is not None
                            for p in polys(g) for r in p.interiors]).intersection(free)
        ring = foot.buffer(3.0, join_style="mitre")
        green = unary_union([dres, aranda]).difference(ring.intersection(unary_union([dres, aranda]))) \
            .difference(sand).union(garden.buffer(-1.5))
        courts = unary_union(self.areas["courts"]).intersection(free)
        paved = free.difference(www).difference(green).difference(sand).difference(courts)
        self._ground = dict(drive=drive, www=www.difference(pools), pools=pools, green=green, sand=sand,
                            paved=paved, courts=courts)
        return self._ground

    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        Z = self.zones()
        T = lambda g: g.intersection(tile)
        m.flat("asphalt_lot", T(Z["drive"]), z + 0.002, 4.0)
        m.flat("pavers", T(Z["paved"]), z + 0.01, 2.0)
        m.flat("grass_park", T(Z["green"]), z + 0.005, 4.0)
        m.flat("dte_sand", T(Z["sand"]), z + 0.01, 3.0)
        m.flat("dte_deck", T(Z["www"]), z + 0.012, 12.0)
        m.flat("dte_pool", T(Z["pools"]), z + 0.02, 6.0)
        cop = unary_union([p.exterior.buffer(0.3) for p in polys(Z["pools"])]).intersection(
            Z["pools"].buffer(0.3)).difference(self.room)
        m.flat("concrete", T(cop), z + 0.03, 1.0)
        for c in self.areas["courts"]:
            if tile.contains(c.centroid) and c.difference(Z["courts"]).area < 1.0:
                q = [np.array(p) for p in list(orient(c, 1.0).exterior.coords)[:4]]
                if math.dist(q[0], q[1]) < math.dist(q[1], q[2]):
                    q = q[1:] + q[:1]
                m.quad("court_multi", *[(p[0], p[1], z + 0.03) for p in q], (0, 0), (1, 0), (1, 1), (0, 1))
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

        def add(anchor, fn, *a, **k):
            pm = Mesh()
            fn(pm, *a, **k)
            nd = drop_over(pm, self.no_go.buffer(0.05))
            if nd:
                self.drop_log.append((fn.__name__, nd))
            self.dropped += nd
            parts.append(((anchor.x, anchor.y) if hasattr(anchor, "x") else anchor, pm))
        bl = self.blocks
        g = lambda k: bl[k]["poly"]
        for k, fn in (("ehub", self.ehub), ("mscp", self.mscp), ("strip", self.strip), ("eav_core", self.eav_core),
                      ("eav_north", self.eav_north), ("eav_east", self.eav_east), ("dmarquee", self.dmarquee),
                      ("playhaus", self.playhaus), ("aranda_club", self.aranda_club), ("hotel_w", self.hotel),
                      ("hotel_e", self.hotel), ("gallop", self.gallop), ("swim_school", self.swim_school),
                      ("plant", self.plant)):
            if g(k) is not None:
                add(g(k).centroid, fn, g(k), bl[k])
        for k in ("aranda_porch", "aranda_pav_n", "aranda_pav_s", "playon", "kiosk"):
            if g(k) is not None:
                add(g(k).centroid, self.small_hip, g(k), bl[k], k)
        for k in ("aranda_hall",):
            if g(k) is not None:
                add(g(k).centroid, self.sports_hall, g(k), bl[k])
        for k in ("aranda_low", "playon_annex", "loop_small"):
            if g(k) is not None:
                add(g(k).centroid, self.small_flat, g(k), bl[k], k)
        for k in sorted(bl):
            if k.startswith("wing_") and g(k) is not None:
                add(g(k).centroid, self.wing, g(k), bl[k])
        allch = [(p, h, self.chalet_line[k]) for k, (geom, h) in self.chalets.items() for p in polys(geom)]
        for i, (p, h, ln) in enumerate(allch):
            others = unary_union([q for j, (q, _, _) in enumerate(allch) if j != i])
            add(p.representative_point(), self.chalet, p, h, others, ln, i)
        add(self.blocks["hotel_w"]["traced"].centroid, self.courtyard_canopy)
        add(self.blocks["hotel_e"]["traced"].centroid, self.hotel_deck)
        c = self.site.centroid
        add(self.blocks["ehub"]["poly"].centroid, self.frontage_walk)
        add(self.areas["loop_garden"].centroid, self.loop_island)
        if "link_bridge" in self.areas:
            add(self.areas["link_bridge"].centroid, self.link_bridge, self.areas["link_bridge"])
        if self.www is not None:
            add(self.www.representative_point(), self.waterpark)
        add(self.dresort.representative_point() if self.dresort is not None else c, self.trees)
        print("dte: %d parts, %d triangles, level-2 drop-off deck %.0f m2, %d own trees, %d triangles dropped "
              "over the road network (roads win)%s"
              % (len(parts), sum(pm.ntris() for _, pm in parts), getattr(self, "deck_area", 0.0),
                 getattr(self, "n_trees", 0), self.dropped,
                 (": " + ", ".join("%s %d" % d for d in self.drop_log)) if self.drop_log else ""), flush=True)
        return parts

    # ---- shared facade pieces / 共通の外壁
    def windows_row(self, f, u0, u1, z0, z1, bay, wall, glass, oz0, oz1, mull=1, margin=0.6, depth=0.15):
        """One floor of punched windows between u0 and u1. / 1 層分の窓。"""
        sub = f if (u0 == 0 and abs(u1 - f.L) < 1e-6) else \
            B.Facade(f.m, np.array(f.P(u0, 0, 0)[:2]), np.array(f.P(u1, 0, 0)[:2]), f.n)
        ops = [(a + margin * 0.5, b - margin * 0.5, z0 + oz0, z0 + oz1) for a, b in B.bays(sub.L, bay, 0.4)]
        sub.wall_with_openings(wall, z0, z1, ops, 2.0)
        for a, b, c, d in ops:
            sub.window(a, b, c, d, wall, glass=glass, mullions=mull, depth=depth)

    def ribbon(self, f, u0, u1, z0, z1, glass, wall, oz0, oz1, depth=0.3, tu=1.5):
        """A ribbon window (one recess, glass tiled every tu m) in the floor z0..z1. / 連続窓。"""
        a, b = u0 + 0.6, u1 - 0.6
        if b - a < 1.0:
            f.face(wall, u0, u1, z0, z1, 0.0, 2.0)
            return
        f.face(wall, u0, a, z0, z1, 0.0, 2.0)
        f.face(wall, b, u1, z0, z1, 0.0, 2.0)
        f.face(wall, a, b, z0, z0 + oz0, 0.0, 2.0)
        f.face(wall, a, b, z0 + oz1, z1, 0.0, 2.0)
        f.hplane(wall, a, b, -depth, 0.0, z0 + oz0, True)
        f.hplane(wall, a, b, -depth, 0.0, z0 + oz1, False)
        f.side(wall, a, -depth, 0.0, z0 + oz0, z0 + oz1, 1)
        f.side(wall, b, -depth, 0.0, z0 + oz0, z0 + oz1, -1)
        f.m.quad(glass, f.P(a, -depth, z0 + oz0), f.P(b, -depth, z0 + oz0), f.P(b, -depth, z0 + oz1),
                 f.P(a, -depth, z0 + oz1), (0, 0), ((b - a) / tu, 0), ((b - a) / tu, 1), (0, 1))

    def curtain(self, f, u0, u1, z0, z1, fh, glass="dte_curtain", ledge="dte_steel", w=0.0, tu=1.5, ends=False):
        """Glass curtain wall, one texture tile per mullion bay and floor, with a
        spandrel ledge standing 0.25 m out at every floor. / カーテンウォールと各階の庇。"""
        grid_face(f, glass, u0, u1, z0, z1, w, tu, fh)
        z = z0 + fh
        while z < z1 - 0.5:
            f.block(ledge, u0, u1, w, w + 0.25, z - 0.18, z + 0.04, ends=ends)
            z += fh

    def flat_roof(self, m, poly, z, wall, parapet=1.0, mat="roof_concrete"):
        bld = B.Builder.__new__(B.Builder)
        bld.flat_roof(m, poly, z, wall, parapet, mat)

    def solar_rows(self, m, poly, z, pitch=2.6, w=1.9, ang=0.0, inset=1.5):
        """Rows of solar panels on a flat roof (flat strips, the panel texture
        tiles per metre). / 屋上の太陽光パネル列。"""
        core = poly.buffer(-inset, join_style="mitre")
        if core.is_empty:
            return
        c = core.centroid
        rc = rotate(core, -ang, origin=c)
        x0, y0, x1, y1 = rc.bounds
        rows = []
        y = y0 + 0.3
        while y + w < y1:
            rows.append(box(x0 - 1, y, x1 + 1, y + w))
            y += pitch
        g = rotate(unary_union(rows), ang, origin=c).intersection(core)
        m.flat("solar", g, z + 0.35, 1.0)
        m.flat("metal_grey", g.buffer(-0.02), z + 0.33, 1.0, up=False)

    def plant_boxes(self, m, poly, z, seed, n=6, size=(3, 8), h=(1.5, 3.5), mat="tank"):
        """Rooftop plant boxes and ducts inside the roof (seeded). / 屋上の設備（乱数固定）。"""
        rng = np.random.default_rng(seed)
        core = poly.buffer(-3.0, join_style="mitre")
        if core.is_empty:
            return
        x0, y0, x1, y1 = core.bounds
        k = 0
        for _ in range(n * 20):
            if k >= n:
                break
            cx, cy = rng.uniform(x0, x1), rng.uniform(y0, y1)
            sx, sy = rng.uniform(*size), rng.uniform(*size)
            b_ = box(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
            if core.contains(b_):
                m.box(mat, cx, cy, z, z + rng.uniform(*h), sx, sy)
                k += 1

    def text(self, m, f, text, u, z, height, mat, font="DejaVuSans-Bold.ttf", w=0.06, aspect=1.0):
        """Letters standing w off the wall, centred at u, base z. / 壁から w 離した文字。"""
        from mesh import earcut
        g = text_poly(text, height, font, aspect)
        for p in polys(g):
            p = orient(p, 1.0)
            rings = [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]
            V = np.concatenate(rings)
            ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
            I = earcut.triangulate_float64(V.astype(np.float64), ends).reshape(-1, 3)
            P3 = [f.P(u + q[0], w, z + height / 2 + q[1]) for q in V]
            A = np.array([P3[i] for i in I.ravel()]).reshape(-1, 3, 3)
            nrm = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
            flip = (nrm[:, :2] @ f.n) < 0
            I = np.where(flip[:, None], I[:, ::-1], I)
            m.tris(mat, P3, np.r_[f.n, 0.0], V / height, I)

    def room_exo_hit(self, f, u0, u1, w0, w1):
        """Whether the box u0..u1 x w0..w1 off this face meets no_go + 0.8 m. / 道路に掛かるか。"""
        if not hasattr(self, "room_exo"):
            self.room_exo = self.no_go.buffer(0.8)
            shapely.prepare(self.room_exo)
        q = Polygon([f.P(u0, w0, 0)[:2], f.P(u1, w0, 0)[:2], f.P(u1, w1, 0)[:2], f.P(u0, w1, 0)[:2]])
        return self.room_exo.intersects(q)

    def side_of(self, n):
        return "S" if n[1] < -0.7 else "N" if n[1] > 0.7 else "E" if n[0] > 0.7 else "W" if n[0] < -0.7 else "C"

    def exo_path(self, m, chain, top, base=Z0, w_bot=1.4, leans=(3.6, 2.4, 3.0), bay=11.5, discs=(),
                 crown=2.0):
        """The steel exoskeleton wrapped along a facade chain (CCW outline points,
        S face round the corner tower to the E face): trapezoid panels narrower
        at the foot, each leaning out by its own amount at the top (a folded
        look), round tubes: frame, X bracing, a mid chord, a V of secondary
        members, round discs; a continuous crown chord above the roof.
        外骨格: 下が狭い台形パネルが各々外へ傾き、角の塔を回り込み、屋上に冠の弦材。"""
        if not hasattr(self, "room_exo"):
            self.room_exo = self.no_go.buffer(0.8)
            shapely.prepare(self.room_exo)
        line = LineString(chain)
        paths = {}

        def path(w):
            if w not in paths:
                g = line.offset_curve(-w, join_style="mitre", mitre_limit=3.0)
                paths[w] = g if g.geom_type == "LineString" else max(getattr(g, "geoms", [g]), key=lambda q: q.length)
            return paths[w]

        def P(t, w, z):
            q = path(w).interpolate(min(max(t, 0.0), 1.0), normalized=True)
            return (q.x, q.y, z)
        ok = lambda p: not self.room_exo.contains(Point(p[0], p[1]))
        Lp = path(w_bot).length
        n = max(1, int(round(Lp / bay)))
        z0, zt = base + 5.0, top
        zm = z0 + (zt - z0) * 0.5
        st = "dte_steel"
        tops = []
        for i in range(n):
            a, b = i / n, (i + 1) / n
            g = 0.9 / Lp                                   # the foot 0.9 m narrower each side / 下端は狭い
            wt = leans[i % len(leans)]
            wm = w_bot + (wt - w_bot) * 0.5
            B0, B1 = P(a + g, w_bot, z0), P(b - g, w_bot, z0)
            T0, T1 = P(a, wt, zt), P(b, wt, zt)
            M0, M1 = P(a + g / 2, wm, zm), P(b - g / 2, wm, zm)
            if not all(ok(q) for q in (B0, B1, T0, T1)):
                continue
            G0, G1 = P(a + g, w_bot, base), P(b - g, w_bot, base)
            tube(m, st, [G0, B0, M0, T0], 0.22, 8)            # posts / 柱
            tube(m, st, [G1, B1, M1, T1], 0.22, 8)
            for p0, p1 in ((B0, B1), (M0, M1), (T0, T1)):     # chords / 弦材
                tube(m, st, [p0, p1], 0.16, 8)
            for p0, p1, p2, p3 in ((B0, B1, M0, M1), (M0, M1, T0, T1)):   # X bracing / X ブレース
                tube(m, st, [p0, p3], 0.12, 6)
                tube(m, st, [p1, p2], 0.12, 6)
            C = P((a + b) / 2, w_bot, z0)                     # the V of secondary members / 副材の V
            tube(m, st, [C, T0], 0.08, 6)
            tube(m, st, [C, T1], 0.08, 6)
            tops.append((T0, T1))
        # the crown: struts and a continuous chord above the panel tops / 冠
        for T0, T1 in tops:
            for T in (T0, T1):
                tube(m, st, [T, (T[0], T[1], T[2] + crown)], 0.12, 6)
            tube(m, st, [(T0[0], T0[1], T0[2] + crown), (T1[0], T1[1], T1[2] + crown)], 0.14, 8)
        for (A0, A1), (B0, B1) in zip(tops, tops[1:]):
            tube(m, st, [(A1[0], A1[1], A1[2] + crown), (B0[0], B0[1], B0[2] + crown)], 0.14, 8)
            tube(m, st, [A1, B0], 0.14, 8)
        # discs on the panels (panel index, height, radius) / 円盤
        for k, (i, z, R) in enumerate(discs):
            if i >= n:
                continue
            t = (i + 0.5) / n
            wt = leans[i % len(leans)]
            w = w_bot + (wt - w_bot) * (z - z0) / (zt - z0) + 0.35
            q0, q1 = P(t - 0.01, w, z), P(t + 0.01, w, z)
            d = np.array([q1[0] - q0[0], q1[1] - q0[1]])
            nrm = np.array([d[1], -d[0]]) / max(np.linalg.norm(d), 1e-9)
            c = P(t, w, z)
            if ok(c) and ok((c[0] + nrm[0] * 0.5, c[1] + nrm[1] * 0.5)):
                disc(m, "dte_disc_%d" % (k % 4), "paint_white", c, nrm, R)

    def chain(self, poly, start_dir, end_dir):
        """The CCW outline from the corner furthest along start_dir to the one
        furthest along end_dir (e.g. SW -> NE: the S face, the SE corner, the E face).
        外周の一部（例: 南西角から北東角まで）。"""
        c = [np.array(q) for q in list(orient(poly, 1.0).exterior.coords)[:-1]]
        i0 = int(np.argmax([q @ np.array(start_dir) for q in c]))
        i1 = int(np.argmax([q @ np.array(end_dir) for q in c]))
        out = [c[i0]]
        i = i0
        while i != i1:
            i = (i + 1) % len(c)
            out.append(c[i])
        return [tuple(q) for q in out]

    def exo(self, m, f, L, top, w0=1.6, w1=3.0, bay=11.0, discs=(), base=Z0):
        """The steel exoskeleton along one face: posts leaning out to the top,
        chords, X bracing, round discs. / 鉄骨の外骨格（外へ傾く柱・弦材・X ブレース・円盤）。"""
        n = max(2, int(round(L / bay)) + 1)
        us = np.linspace(0.6, L - 0.6, n)
        zs = [base + 5.0, base + (top - base) * 0.55, top]
        W = lambda z: w0 + (w1 - w0) * (z - base) / (top - base)
        if not hasattr(self, "room_exo"):
            self.room_exo = self.no_go.buffer(0.8)
            shapely.prepare(self.room_exo)
        ok = lambda u, w: not self.room_exo.contains(Point(f.P(u, w, 0)[:2]))
        for u in us:
            if ok(u, W(top)):
                beam(m, "dte_steel", f.P(u, w0, base), f.P(u, W(top), top), 0.2)
        for z in zs:
            for u0, u1 in zip(us, us[1:]):
                if ok(u0, W(z)) and ok(u1, W(z)):
                    beam(m, "dte_steel", f.P(u0, W(z), z), f.P(u1, W(z), z), 0.13)
        for i, (u0, u1) in enumerate(zip(us, us[1:])):
            for za, zb in zip(zs, zs[1:]):
                if not (ok(u0, W(zb)) and ok(u1, W(zb))):
                    continue
                beam(m, "dte_steel", f.P(u0, W(za), za), f.P(u1, W(zb), zb), 0.1)
                beam(m, "dte_steel", f.P(u1, W(za), za), f.P(u0, W(zb), zb), 0.1)
        for k, (t, z, R) in enumerate(discs):
            u = 0.6 + t * (L - 1.2)
            w = W(z) + 0.35
            if ok(u, w + 0.3) and ok(u - R, w + 0.3) and ok(u + R, w + 0.3):
                disc(m, "dte_disc_%d" % (k % 4), "paint_white", f.P(u, w, z), f.n, R)

    # ---- E!Hub
    def ehub(self, m, poly, b):
        top = Z0 + b["h"]
        roof = top - 1.2
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            s = self.side_of(n) if L > 6.0 else "C"
            if s == "C":                               # the curved glass corner / 曲面ガラスの角
                self.curtain(f, 0, L, Z0, top - 1.5, 4.6, glass="dte_glass_green", ledge="paint_white", tu=1.2)
                for k in range(1, 5):
                    z = Z0 + 5.5 + (k - 1) * 4.6
                    if not self.room.intersects(Point(f.P(L / 2, 1.3, 0)[:2])):
                        f.block("paint_white", 0, L, 0, 1.2, z - 0.25, z, ends=True)
                        f.face("glass_rail", 0, L, z, z + 1.1, 1.18)
                f.face("dte_panel_grey", 0, L, top - 1.5, top, 0.0)
                continue
            # ground floor: shopfronts set back 0.6 m / 1 階: 0.6 m 後退した店舗
            f.face("dte_shopglass", 0, L, Z0, Z0 + 4.4, -0.6, 4.0)
            f.hplane("dte_panel_grey", 0, L, -0.6, 0, Z0 + 4.4, False)
            f.side("dte_panel_grey", 0, -0.6, 0, Z0, Z0 + 4.4, 1)
            f.side("dte_panel_grey", L, -0.6, 0, Z0, Z0 + 4.4, -1)
            if s in ("S", "E") and not self.room.intersects(LineString([f.P(0, 2.6, 0)[:2], f.P(L, 2.6, 0)[:2]])):
                f.block("dte_awning", 0.5, L - 0.5, 0, 2.5, Z0 + 4.2, Z0 + 4.55)
            # panels with a groove every floor / 各階に目地のあるパネル
            grid_face(f, "dte_panel_grey", 0, L, Z0 + 4.4, top, 0.0, 3.0, 4.6)
            for k in range(1, 5):
                f.block("paint_palegrey", 0, L, 0, 0.12, Z0 + 5.5 + k * 4.6 - 0.25, Z0 + 5.5 + k * 4.6, ends=False)
            if s == "E" and L > 30:
                # the green glass curtain wall, 20 m wide, 4 storeys / 緑のガラスのカーテンウォール
                u0, u1 = L / 2 - 10, L / 2 + 10
                if not self.room.intersects(LineString([f.P(u0, 0.5, 0)[:2], f.P(u1, 0.5, 0)[:2]])):
                    self.curtain(f, u0, u1, Z0 + 5.5, Z0 + 5.5 + 4 * 4.6, 4.6, glass="dte_glass_green", w=0.35,
                                 tu=1.25)
                    f.side("dte_steel", u0, 0, 0.35, Z0 + 5.5, Z0 + 5.5 + 4 * 4.6, -1)
                    f.side("dte_steel", u1, 0, 0.35, Z0 + 5.5, Z0 + 5.5 + 4 * 4.6, 1)
                    f.hplane("dte_steel", u0, u1, 0, 0.35, Z0 + 5.5 + 4 * 4.6, True)
            if s == "W" and L > 30:
                # slot windows over the service lane / 裏通り側の細長い窓
                for k in range(4):
                    z = Z0 + 6.6 + k * 4.6
                    for u0, u1 in B.bays(L, 9.0, 2.0):
                        f.face("glass_dark", u0 + 1.5, u1 - 1.5, z, z + 1.2, 0.03, 2.0)
            if s == "S":
                self.text(m, f, "E!HUB", L * 0.18, top - 4.2, 2.2, "paint_white", w=0.15)
            if s == "E" and L > 30 and not self.room_exo_hit(f, 1.0, 4.5, 0.4, 1.4):
                # the open stair tower: flights between landings in a steel cage / 屋外階段の塔
                for k in range(5):
                    z = Z0 + k * 4.6
                    f.block("concrete", 1.0, 4.5, 0.3, 1.5, z + 2.2, z + 2.4)
                    beam(m, "concrete", f.P(1.2, 0.9, z), f.P(4.3, 0.9, z + 2.3), 0.5)
                    beam(m, "concrete", f.P(4.3, 0.9, z + 2.3), f.P(1.2, 0.9, z + 4.6), 0.5)
                for uu in (1.0, 4.5):
                    beam(m, "dte_steel", f.P(uu, 1.5, Z0), f.P(uu, 1.5, top), 0.08)

        # the wrapped exoskeleton: S face, round the SE corner tower, E face / 南面から角の塔を回り東面へ
        discs = ((1, Z0 + 15.0, 1.6), (2, Z0 + 18.5, 1.3), (3, Z0 + 12.5, 2.1), (4, Z0 + 20.0, 1.5),
                 (5, Z0 + 14.0, 1.9), (7, Z0 + 17.0, 1.7), (8, Z0 + 12.0, 2.0), (10, Z0 + 19.0, 1.5))
        self.exo_path(m, self.chain(poly, (-1, -1), (1, 1)), top + 3.0, discs=discs)
        self.flat_roof(m, poly, roof, "dte_panel_grey", 1.2, "dte_roof_green")
        # ducts, plant, a solar block / ダクト・設備・太陽光パネル
        self.plant_boxes(m, poly, roof, 11, n=9, size=(3, 9), h=(1.5, 3.2), mat="tank")
        c = poly.centroid
        for k in range(5):
            y = c.y - 20 + k * 9
            m.box("tank", c.x, y, roof, roof + 1.1, poly.bounds[2] - poly.bounds[0] - 14, 1.1)
        sb = box(c.x - 14, poly.bounds[1] + 4, c.x + 12, poly.bounds[1] + 14).intersection(poly.buffer(-2))
        self.solar_rows(m, sb, roof, inset=0.2)

    # ---- multi-storey car park / 立体駐車場
    def mscp(self, m, poly, b):
        top = Z0 + b["h"]
        ny = b["north_end_y"]
        x0, y0, x1, y1 = poly.bounds
        main = poly.intersection(box(x0 - 1, y0 - 1, x1 + 1, ny))
        north = poly.intersection(box(x0 - 1, ny, x1 + 1, y1 + 1))
        for part, levels in ((main, 7), (north, 4)):
            part = largest(part)
            if part is None:
                continue
            core = part.buffer(-6.0, join_style="mitre")
            for k in range(levels):
                z = Z0 + k * 3.0
                if k > 0:
                    m.prism("dte_slab", part, z - 0.35, z, top=True, bottom=True)
                    m.ring_walls("dte_slab", part.exterior, z, z + 1.05, outward=True, uv_scale=2.0)
                    inn = part.buffer(-0.2, join_style="mitre")
                    m.ring_walls("concrete", inn.exterior, z, z + 1.05, outward=False)
                    m.flat("dte_slab", part.difference(inn), z + 1.05, 1.0)
                if k < levels - 1 and not core.is_empty:
                    m.prism("corridor_dark", core, z, z + 2.65, top=False)
                    ring = part.buffer(-0.5, join_style="mitre").exterior
                    for t in np.arange(0.0, ring.length, 8.0):
                        q = ring.interpolate(t)
                        m.box("concrete", q.x, q.y, z, z + 2.65, 0.5, 0.5)
            zt = Z0 + (levels - 1) * 3.0
            if levels == 7:
                self.solar_rows(m, part, zt + 1.0, inset=2.5)
                ring = part.buffer(-0.6, join_style="mitre").exterior
                for t in np.arange(0.0, ring.length, 18.0):
                    q = ring.interpolate(t)
                    m.cylinder("metal_grey", q.x, q.y, zt, zt + 6.0, 0.08, seg=6)
                    m.box("lamp_head", q.x, q.y, zt + 6.0, zt + 6.25, 0.7, 0.3)
        mp = largest(main)
        mp = mp if mp is not None else poly
        self.exo_path(m, self.chain(mp, (1, -1), (1, 1)), top, w_bot=1.2, leans=(2.6, 1.9, 2.3), bay=12.0,
                      discs=((1, Z0 + 12.0, 1.8), (3, Z0 + 15.0, 2.1), (6, Z0 + 11.0, 1.6)), crown=1.2)
        for a, bb, L, n in B.edges_of(mp):
            if self.side_of(n) == "E":
                f = B.Facade(m, a, bb, n)
                f.face("dte_shutter", L * 0.42, L * 0.42 + 7.0, Z0, Z0 + 2.6, 0.02, 3.0)

    # ---- E!Avenue strip, core, north wing, east box / E!Avenue
    def strip(self, m, poly, b):
        top = Z0 + b["h"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face("dte_shopglass", 0, L, Z0, Z0 + 5.5, -0.5, 4.0)
            f.hplane("paint_white", 0, L, -0.5, 0, Z0 + 5.5, False)
            f.side("paint_white", 0, -0.5, 0, Z0, Z0 + 5.5, 1)
            f.side("paint_white", L, -0.5, 0, Z0, Z0 + 5.5, -1)
            f.face("paint_white", 0, L, Z0 + 5.5, top, 0.0, 2.0)
            if n[0] < -0.5 and L > 10:
                # shop fascia boxes over the shopfronts (purple, teal, red, green) / 店の看板箱
                cols = ("paint_purple", "paint_teal", "paint_red", "paint_green", "paint_orange")
                for i, (u0, u1) in enumerate(B.bays(L, 8.0, 0.5)):
                    f.block(cols[(i * 3) % len(cols)], u0 + 1.0, u1 - 1.0, 0, 0.3, Z0 + 4.4, Z0 + 5.3)
            if L > 6:
                self.ribbon(f, 0, L, Z0 + 7.0, Z0 + 11.5, "dte_glass_ribbon", "paint_white", 0.8, 3.8, 0.25)
                f.face("paint_white", 0, L, Z0 + 5.5, Z0 + 7.0, 0.0, 2.0)
        self.flat_roof(m, poly, top - 0.9, "paint_white", 0.9)
        self.solar_rows(m, poly, top - 0.9, ang=0.0)
        # the skylight band (glass ridge) and plant boxes / 天窓の帯と設備
        sk = poly.intersection(box(800, 1524, 1000, 1531))
        for p in polys(sk):
            gable(m, p, top - 0.9, top + 0.9, "glass_dark", "paint_white", ov=0.0)
        self.plant_boxes(m, poly, top - 0.9, 23, n=6, size=(2.5, 5), h=(1.2, 2.2))
        # the white canopy along the west face on tall slender round columns
        # 西面の白い大庇（細く高い丸柱）
        ln = self.areas["strip_canopy"]
        can = ln.buffer(7.0, single_sided=True, join_style="mitre")
        others = unary_union([self.blocks[k]["poly"] for k in ("dmarquee",) if self.blocks[k]["poly"] is not None])
        can = can.difference(self.room).difference(others.buffer(0.5)).difference(poly) \
            .difference(self.drive_snapshot.buffer(0.3))
        zc = Z0 + 7.5
        for p in polys(can):
            if p.area < 8:
                continue
            m.flat("paint_white", p, zc + 0.6, 2.0)
            m.flat("dte_soffit_folded", p, zc, 3.0, up=False)
            m.ring_walls("paint_white", p.exterior, zc, zc + 0.6, outward=True)
        posts = ln.offset_curve(6.3)
        for t in np.arange(2.0, posts.length, 9.0):
            q = posts.interpolate(t)
            if can.buffer(-0.3).contains(q):
                m.cylinder("metal_grey", q.x, q.y, Z0, zc, 0.25, seg=8, top=False)

    def eav_core(self, m, poly, b):
        top = Z0 + b["h"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face("dte_shopglass", 0, L, Z0, Z0 + 4.5, -0.4, 4.0)
            f.hplane("dte_steel", 0, L, -0.4, 0, Z0 + 4.5, False)
            self.curtain(f, 0, L, Z0 + 4.5, top - 1.0, 4.4)
            f.face("paint_white", 0, L, top - 1.0, top, 0.0)
        self.flat_roof(m, poly, top - 0.8, "paint_white", 0.8)
        self.plant_boxes(m, poly, top - 0.8, 31, n=7, size=(3, 7), h=(1.5, 3.0))

    def eav_north(self, m, poly, b):
        top = Z0 + b["h"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            if n[1] > 0.2 and n[0] < 0.2:                 # faces Wild Wild Wet: the colonnade / 列柱
                f.face("dte_shopglass", 0, L, Z0, Z0 + 5.0, -3.0, 4.0)
                f.hplane("concrete", 0, L, -3.0, 0, Z0 + 5.0, False)
                f.side("paint_white", 0, -3.0, 0, Z0, Z0 + 5.0, 1)
                f.side("paint_white", L, -3.0, 0, Z0, Z0 + 5.0, -1)
                for u in np.arange(2.0, L - 1.0, 6.0):
                    q = f.P(u, -0.5, 0)
                    m.cylinder("paint_white", q[0], q[1], Z0, Z0 + 5.0, 0.3, seg=10, top=False)
            else:
                f.face("dte_shopglass", 0, L, Z0, Z0 + 5.0, -0.4, 4.0)
                f.hplane("dte_steel", 0, L, -0.4, 0, Z0 + 5.0, False)
            self.curtain(f, 0, L, Z0 + 5.0, top - 0.8, 4.3)
            f.block("paint_white", 0, L, 0, 0.5, top - 0.8, top)          # thin white roof edge / 白い軒
        self.flat_roof(m, poly, top - 0.8, "paint_white", 0.8)
        self.solar_rows(m, poly, top - 0.8, ang=-45.0)

    def eav_east(self, m, poly, b):
        top = Z0 + b["h"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            grid_face(f, "dte_strips", 0, L, Z0, top, 0.0, 4.8, top - Z0)
            s = self.side_of(n)
            if s in ("S", "E"):
                # canopy slab at 5 m, as deep as the road allows / 5 m の庇（道路まで）
                for d in (3.0, 2.0, 1.2):
                    if not self.room.intersects(LineString([f.P(0.5, d, 0)[:2], f.P(L - 0.5, d, 0)[:2]])):
                        f.block("paint_white", 0.5, L - 0.5, 0, d, Z0 + 5.0, Z0 + 5.45)
                        break
            if s == "E" and L > 20:
                f.face("dte_shutter", L * 0.3, L * 0.3 + 6.0, Z0, Z0 + 4.2, 0.03, 3.0)
                f.face("dte_shutter", L * 0.3 + 7.0, L * 0.3 + 13.0, Z0, Z0 + 4.2, 0.03, 3.0)
        self.flat_roof(m, poly, top, "dte_strips", 0.6)
        scr = poly.buffer(-1.5, join_style="mitre")
        if not scr.is_empty:
            m.ring_walls("louvre", scr.exterior, top, top + 2.2, outward=True, uv_scale=2.0)
            m.ring_walls("louvre", scr.exterior, top, top + 2.2, outward=False, uv_scale=2.0)
        self.plant_boxes(m, scr if not scr.is_empty else poly, top, 41, n=4, size=(2, 4), h=(1.0, 1.8))

    # ---- D'Marquee
    def dmarquee(self, m, poly, b):
        top = Z0 + b["h"]
        edges = B.edges_of(poly)
        front = min(edges, key=lambda e: e[3][1])          # the south face / 南面
        a, bb, L, n = front
        f = B.Facade(m, a, bb, n)
        depth = 8.0
        # the wave: low over the west third, full height in the east / 波形の大屋根の縁
        wave = lambda t: Z0 + float(np.interp(t, [0, 0.12, 0.27, 0.36, 0.52, 0.62, 1.0],
                                              [11.0, 10.3, 9.2, 9.4, 13.4, 14.0, 14.0]))
        N = 28
        us = np.linspace(0, L, N + 1)
        # ground floor: glazing, orange panels, the tile strip, doors / 1 階
        for i, (u0, u1) in enumerate(B.bays(L, 6.0, 0.0)):
            mat = "dte_orange" if i % 3 == 0 else "glass_dark"
            f.face(mat, u0 + 0.15, u1 - 0.15, Z0, Z0 + 3.8, -0.3, 2.0)
            f.block("metal_grey", u1 - 0.15, u1 + 0.15, -0.3, 0, Z0, Z0 + 3.8, top=False, bottom=False)
            if mat == "glass_dark":
                f.face("dte_tiles_multi", u0 + 0.15, u1 - 0.15, Z0 + 0.5, Z0 + 0.8, -0.27, 1.0)
        f.hplane("paint_white", 0, L, -0.3, 0, Z0 + 3.8, False)
        f.face("dte_purple", 0, L, Z0 + 3.8, Z0 + 4.0, 0.0)
        # the purple fascia canopy at 4 m / 4 m の紫の庇
        f.block("dte_purple", 0, L, 0, 1.6, Z0 + 4.0, Z0 + 4.4)
        for u0, u1 in zip(us, us[1:]):
            t0, t1 = u0 / L, u1 / L
            z0w, z1w = wave(t0), wave(t1)
            # bands whose top follows the wave's soffit line (no steps) / 帯の上端は波に沿う
            for mat, za, zb, tu, tv in (("dte_purple_fluted", Z0 + 4.4, Z0 + 7.0, 1.2, 2.6),
                                        ("dte_green_batten", Z0 + 7.0, Z0 + 10.0, 1.0, 3.0),
                                        ("paint_white", Z0 + 10.0, 99.0, 3.0, 3.0)):
                t0, t1 = min(zb, z0w - 0.55), min(zb, z1w - 0.55)
                if t0 > za + 0.01 and t1 > za + 0.01:
                    f.m.quad(mat, f.P(u0, 0.0, za), f.P(u1, 0.0, za), f.P(u1, 0.0, t1), f.P(u0, 0.0, t0),
                             (u0 / tu, za / tv), (u1 / tu, za / tv), (u1 / tu, t1 / tv), (u0 / tu, t0 / tv))
            # the white wave canopy: fascia 2 m out, soffit, roof back to the hall / 白い波形庇
            P = f.P
            m.quad("paint_white", P(u0, 2.0, z0w - 0.55), P(u1, 2.0, z1w - 0.55), P(u1, 2.0, z1w), P(u0, 2.0, z0w))
            m.quad("paint_white", P(u0, -depth, z0w), P(u0, 2.0, z0w), P(u1, 2.0, z1w), P(u1, -depth, z1w))
            m.quad("dte_soffit", P(u0, 0.0, z0w - 0.55), P(u1, 0.0, z1w - 0.55), P(u1, 2.0, z1w - 0.55),
                   P(u0, 2.0, z0w - 0.55))
            # the wall from the wave up to the hall roof behind / 奥のホール屋根まで立ち上がる壁
            if top - 0.6 > min(z0w, z1w):
                m.quad("paint_white", P(u0, -depth, z0w), P(u1, -depth, z1w), P(u1, -depth, top - 0.6),
                       P(u0, -depth, top - 0.6))
        # the canopy's open ends / 庇の端
        for u, zz, fac in ((0.0, wave(0.0), -1), (L, wave(1.0), 1)):
            f.side("paint_white", u, -depth, 2.0, zz - 0.55, zz, fac)
        # the name on the green band, two banner boxes on the purple / 名前と 2 つの看板
        gu = L * 0.6
        f.block("dte_green_batten", gu - 6.5, gu + 6.5, 0, 0.25, Z0 + 7.4, Z0 + 9.6)
        self.text(m, f, "D'MARQUEE", gu, Z0 + 7.85, 1.3, "paint_white", font="DejaVuSans.ttf", w=0.31)
        for k, uu in enumerate((L * 0.18, L * 0.5)):
            f.block("dte_purple", uu - 3.2, uu + 3.2, 0, 0.3, Z0 + 4.6, Z0 + 6.9)
            f.m.quad("dte_banner_%d" % k, f.P(uu - 3.0, 0.31, Z0 + 4.75), f.P(uu + 3.0, 0.31, Z0 + 4.75),
                     f.P(uu + 3.0, 0.31, Z0 + 6.75), f.P(uu - 3.0, 0.31, Z0 + 6.75), (0, 0), (1, 0), (1, 1), (0, 1))
        for u in np.arange(1.0, L - 0.5, 2.4):           # the row of bollards / 車止め
            q = f.P(u, 3.2, 0)
            if not self.room.contains(Point(q[:2])):
                m.cylinder("metal_grey", q[0], q[1], Z0, Z0 + 0.9, 0.08, seg=6)
        for u in (0.6, L - 0.6):
            q = f.P(u, 1.7, 0)
            m.cylinder("metal_grey", q[0], q[1], Z0, wave(u / L) - 0.55, 0.12, seg=6, top=False)
        # the other faces: white panels, the purple band / 他の面: 白いパネルと紫の帯
        for a2, b2, L2, n2 in edges:
            if a2 is front[0] or np.allclose(a2, front[0]):
                continue
            f2 = B.Facade(m, a2, b2, n2)
            fr_end = None
            if np.allclose(b2, front[0]):            # the west face ends at the front / 西面は前面で終わる
                fr_end = (L2 - depth, L2, wave(0.0))
            if np.allclose(a2, front[1]):            # the east face starts at it / 東面は前面から始まる
                fr_end = (0.0, depth, wave(1.0))
            u0, u1 = 0.0, L2
            if fr_end is not None:
                grid_face(f2, "paint_white", fr_end[0], fr_end[1], Z0 + 4.4, fr_end[2], 0.0, 3.0, 3.0)
                f2.face("glass_dark", fr_end[0], fr_end[1], Z0, Z0 + 3.8, -0.3, 2.0)
                f2.face("dte_purple", fr_end[0], fr_end[1], Z0 + 3.8, Z0 + 4.4, 0.0)
                if fr_end[0] == 0.0:
                    u0 = depth
                else:
                    u1 = L2 - depth
            grid_face(f2, "paint_white", u0, u1, Z0, top, 0.0, 3.0, 3.0)
            f2.block("dte_purple", u0, u1, 0, 0.12, Z0 + 3.8, Z0 + 4.4, ends=False)
            for d0, d1 in B.bays(u1 - u0, 14.0, 2.0)[::2]:
                f2.face("dte_shutter", u0 + d0 + 4, u0 + d0 + 8, Z0, Z0 + 3.5, 0.02, 3.0)
        hall = poly.difference(LineString([front[0], front[1]]).buffer(depth, single_sided=False, cap_style="flat"))
        hall = largest(hall)
        if hall is not None:
            self.flat_roof(m, hall, top - 0.6, "paint_white", 0.6, mat="paint_white")
            self.solar_rows(m, hall, top - 0.6, ang=0.0, inset=2.5)

    # ---- PlayHaus
    def playhaus(self, m, poly, b):
        ze, zr = Z0 + b["eave"], Z0 + b["ridge"]
        core = poly.buffer(-1.0, join_style="mitre")
        for a, bb, L, n in B.edges_of(core):
            f = B.Facade(m, a, bb, n)
            grid_face(f, "paint_charcoal", 0, L, Z0, ze, 0.0, 3.0, 3.0)
            for u0, u1 in B.bays(L, 8.0, 2.0):
                f.face("glass_dark", u0 + 1.0, u1 - 1.0, Z0, Z0 + 3.2, 0.03, 2.0)
            f.face("louvre", 1.0, L - 1.0, ze - 2.5, ze - 1.0, 0.03, 2.0)
        ov = self.overhang(core, 3.2)
        lan = b.get("lantern", 22.0) / 2
        zl = ze + 6.3
        hip(m, core, ze, zl, "dte_roof_seam", ov=ov, inner=(lan, lan, zl))
        # the lantern: louvred clerestory and its own hipped cap / 頂部の越屋根
        a, bb_, c, d = rect_axes(core)
        ctr = (a + bb_ + c + d) / 4
        ax = (bb_ - a) / np.linalg.norm(bb_ - a)
        rot = math.atan2(ax[1], ax[0])
        lanp = rotate(box(ctr[0] - lan + 1.0, ctr[1] - lan + 1.0, ctr[0] + lan - 1.0, ctr[1] + lan - 1.0),
                      rot, origin=(ctr[0], ctr[1]), use_radians=True)
        m.prism("louvre", lanp, zl, zl + 1.6, top=False, uv_scale=2.0)
        hip(m, lanp, zl + 1.6, zr, "dte_roof_seam", ov=0.9)

    def overhang(self, poly, want):
        for ov in (want, want * 0.75, want * 0.5, 0.6, 0.3, 0.0):
            if not poly.minimum_rotated_rectangle.buffer(ov + 0.1, join_style="mitre").intersects(self.room):
                return ov
        return 0.0

    # ---- Aranda Country Club / アランダ・カントリークラブ
    def aranda_club(self, m, poly, b):
        ze, zr = Z0 + b["eave"], Z0 + b["ridge"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            # ground floor arcade, two upper floors of windows / 1 階はアーケード、上 2 層に窓
            f.face("paint_cream", 0, L, Z0, Z0 + 4.0, -2.0, 2.0)
            f.hplane("paint_white", 0, L, -2.0, 0, Z0 + 4.0, False)
            f.side("paint_white", 0, -2.0, 0, Z0, Z0 + 4.0, 1)
            f.side("paint_white", L, -2.0, 0, Z0, Z0 + 4.0, -1)
            for u in np.arange(1.5, L - 1.0, 4.5):
                f.block("paint_white", u - 0.3, u + 0.3, -0.6, 0.0, Z0, Z0 + 4.0, top=False, bottom=False)
            for k in range(2):
                z0 = Z0 + 4.0 + k * 3.0
                self.windows_row(f, 0, L, z0, z0 + 3.0, 3.0, "paint_white", "glass_dark", 0.8, 2.5, mull=1)
            f.block("paint_white", 0, L, 0, 0.3, ze - 0.4, ze)
        ov = self.overhang(poly, 1.5)
        hip(m, poly, ze, zr, "roof_terracotta", ov=ov)
        # the lantern / clock tower above the ridge / 棟の上の塔
        c = poly.centroid
        zb = zr - 2.0
        for dx in (-2.6, 2.6):
            for dy in (-2.6, 2.6):
                m.box("paint_charcoal", c.x + dx, c.y + dy, zb, zb + 6.0, 0.4, 0.4)
        m.box("glass_dark", c.x, c.y, zb, zb + 5.6, 4.8, 4.8)
        m.box("paint_white", c.x, c.y, zb + 3.6, zb + 4.4, 5.6, 5.6)
        hip(m, box(c.x - 3.0, c.y - 3.0, c.x + 3.0, c.y + 3.0), zb + 6.0, zb + 8.5, "roof_terracotta", ov=0.3)

    def small_hip(self, m, poly, b, k):
        ze, zr = Z0 + b["eave"], Z0 + b["ridge"]
        wall = {"playon": "paint_charcoal", "kiosk": "glass_dark"}.get(k, "paint_white")
        roof = {"playon": "roof_grey", "kiosk": "roof_teal"}.get(k, "roof_terracotta")
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face(wall, 0, L, Z0, ze, 0.0, 2.0)
            if L > 6 and k != "kiosk":
                for u0, u1 in B.bays(L, 5.0, 0.8):
                    f.face("glass_dark", u0 + 0.6, u1 - 0.6, Z0 + 0.8, ze - 0.8, 0.03, 2.0)
        hip(m, poly, ze, zr, roof, ov=self.overhang(poly, 0.8))

    def small_flat(self, m, poly, b, k):
        top = Z0 + b["h"]
        wall = {"aranda_low": "paint_white", "playon_annex": "paint_charcoal"}.get(k, "paint_palegrey")
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face(wall, 0, L, Z0, top, 0.0, 2.0)
            for u0, u1 in B.bays(L, 4.0, 0.6):
                f.face("glass_dark", u0 + 0.5, u1 - 0.5, Z0 + 0.9, Z0 + 2.4, 0.03, 2.0)
        self.flat_roof(m, poly, top - 0.6, wall, 0.6, mat="roof_green" if k == "aranda_low" else "roof_concrete")

    def sports_hall(self, m, poly, b):
        ze, zr = Z0 + b["eave"], Z0 + b["ridge"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face("paint_white", 0, L, Z0, ze, 0.0, 2.0)
            f.face("louvre", 1.0, L - 1.0, ze - 2.2, ze - 0.8, 0.03, 2.0)
        gable(m, poly, ze, zr, "roof_metal", "paint_white", ov=self.overhang(poly, 0.6))

    def wing(self, m, poly, b):
        """A piece of the accommodation wing: cream render, 4 floors of windows,
        its own red pyramidal hipped roof. / 宿泊棟の一部。"""
        ze, zr = Z0 + b["eave"], Z0 + b["ridge"]
        nf = max(1, int(round((ze - Z0) / 3.3)))
        fh = (ze - Z0) / nf
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            for k in range(nf):
                z0 = Z0 + k * fh
                self.windows_row(f, 0, L, z0, z0 + fh, 3.4, "paint_cream", "glass_dark", 0.9, 2.5, mull=1)
            f.block("paint_white", 0, L, 0, 0.2, Z0 + fh - 0.2, Z0 + fh, ends=False)
        hip(m, poly, ze, zr, "roof_terracotta", ov=self.overhang(poly, 0.9))

    # ---- D'Resort hotel / ホテル
    def hotel(self, m, poly, b):
        top = Z0 + b["h"]
        pod = Z0 + 7.0
        nf = 6
        fh = (top - 1.4 - pod) / nf
        ra = rect_axes(poly)
        long_ax = (ra[1] - ra[0]) / np.linalg.norm(ra[1] - ra[0])
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            if abs(float(((bb - a) / L) @ long_ax)) < 0.5:
                # the end walls: copper-red and charcoal vertical stripes, a projecting
                # dark core with copper edges, the light brown parapet / 妻面: 銅色の縦縞と張り出したコア
                grid_face(f, "dte_copper", 0, L, Z0, top, 0.0, 6.0, top - Z0)
                f.face("glass_dark", L * 0.12, L * 0.3, Z0, Z0 + 3.5, -0.3, 2.0)
                c0, c1 = L * 0.45, L * 0.8
                if L > 8 and not self.room_exo_hit(f, c0, c1, 0.0, 1.0):
                    f.block("dte_hotel_clad", c0 + 0.7, c1 - 0.7, 0.0, 1.0, Z0, top + 2.0)
                    f.block("dte_copper_solid", c0, c0 + 0.7, 0.0, 1.0, Z0, top + 2.0)
                    f.block("dte_copper_solid", c1 - 0.7, c1, 0.0, 1.0, Z0, top + 2.0)
                    for k in range(6):                         # stair slot windows / 階段の細窓
                        zz = Z0 + 8.0 + k * 3.1
                        f.face("glass_dark", (c0 + c1) / 2 - 0.5, (c0 + c1) / 2 + 0.5, zz, zz + 1.8, 1.02, 2.0)
                grid_face(f, "dte_parapet_brown", 0, L, top - 1.4, top, 0.04, 3.6, 1.4)
                continue
            # podium: glazed lobby toward the drop-off, clad elsewhere / 基壇: 車寄せ側はガラス
            inward = float(n @ (np.array([930.0, 1860.0]) - (a + bb) / 2)) > 0
            if inward:
                self.curtain(f, 0, L, Z0, pod, 3.5, glass="dte_glass_ribbon", ledge="paint_woodbrown")
            else:
                grid_face(f, "dte_hotel_clad", 0, L, Z0, pod, 0.0, 3.0, 3.5)
                for u0, u1 in B.bays(L, 7.0, 1.0):
                    f.face("glass_dark", u0 + 1.0, u1 - 1.0, Z0 + 0.6, Z0 + 3.0, -0.1, 2.0)
            f.block("paint_woodbrown", 0, L, 0, 0.4, pod - 0.4, pod)
            for k in range(nf):
                z0 = pod + k * fh
                self.ribbon(f, 0, L, z0, z0 + fh, "dte_glass_ribbon", "dte_hotel_clad", 0.9, fh - 0.4, 0.3)
            grid_face(f, "dte_parapet_brown", 0, L, top - 1.4, top, 0.0, 3.6, 1.4)
            if poly is self.blocks["hotel_w"]["poly"] and not inward:
                self.text(m, f, "D'RESORT", L * 0.62, top - 4.2, 1.9, "dte_teal", w=0.08)
        self.flat_roof(m, poly, top - 0.6, "dte_parapet_brown", 0.6)
        self.solar_rows(m, poly, top - 0.6, ang=math.degrees(math.atan2(*(rect_axes(poly)[1] - rect_axes(poly)[0])[::-1])),
                        inset=3.0)
        # stair / lift cores at both ends, higher than the roof / 両端の階段・EV コア
        a, bb, c, d = rect_axes(poly)
        ax = (bb - a) / np.linalg.norm(bb - a)
        sd = (d - a) / np.linalg.norm(d - a)
        Wd = np.linalg.norm(d - a)
        for t in (4.0, np.linalg.norm(bb - a) - 4.0):
            q = a + ax * t + sd * Wd / 2
            core = rotate(box(q[0] - 3.5, q[1] - 3.0, q[0] + 3.5, q[1] + 3.0), math.atan2(ax[1], ax[0]),
                          origin=(q[0], q[1]), use_radians=True)
            m.prism("dte_copper", core, top - 0.6, top + 3.2, top=True, uv_scale=6.0)

    # ---- chalets / シャレー
    def chalet(self, m, poly, h, others=None, line=None, idx=0):
        top = Z0 + h
        nf = max(2, int(round(h / 3.8)))
        fh = (top - 0.8 - Z0) / nf
        hc = np.array([930.0, 1878.0])
        for a0, b0, L0, n, hole in all_edges(poly):
            # the parts of the edge not shared with the next chain (no wall or fascia
            # there: a fascia coplanar with the neighbour's roof rendered black)
            # 隣の棟と接しない部分だけに壁と鼻隠し（接する所は隣の屋根と同一面で黒く出た）
            pieces = [(a0, b0)]
            if others is not None and not others.is_empty:
                free = LineString([a0 + n * 0.3, b0 + n * 0.3]).difference(others.buffer(0.05))
                pieces = []
                for g in getattr(free, "geoms", [free]):
                    if g.geom_type == "LineString" and g.length > 0.8:
                        c0, c1 = np.array(g.coords[0]) - n * 0.3, np.array(g.coords[-1]) - n * 0.3
                        if float((c1 - c0) @ (b0 - a0)) < 0:
                            c0, c1 = c1, c0
                        pieces.append((c0, c1))
            for a, bb in pieces:
                L = float(np.linalg.norm(bb - a))
                f = B.Facade(m, a, bb, n)
                mid = (a + bb) / 2
                corr = hole or (L0 > 8 and float(n @ (hc - mid)) > 0)
                if L0 < 14 and not hole:                       # end walls: grey checker tiles / 妻面: 市松タイル
                    grid_face(f, "dte_checker", 0, L, Z0, top, 0.0, 2.4, 2.4)
                    continue
                for k in range(nf):
                    z0 = Z0 + k * fh
                    if corr:
                        # the external corridor: slab edge, galvanised post railing, doors / 外廊下
                        f.face("paint_white", 0, L, z0 + fh - 0.3, z0 + fh, 0.0)
                        if k > 0:
                            f.hplane("concrete", 0, L, -1.8, -0.11, z0, True)
                            f.face("dte_rail_posts", 0, L, z0, z0 + 1.1, -0.05, 1.0)
                            face_in(f, "dte_rail_posts", 0, L, z0, z0 + 1.1, -0.11)
                            f.hplane("metal_grey", 0, L, -0.11, -0.05, z0 + 1.1, True)
                            for u in np.arange(0.25, L - 0.1, 0.5):          # galvanised posts / 縦格子
                                beam(m, "metal_grey", f.P(u, 0.02, z0), f.P(u, 0.02, z0 + 1.1), 0.025)
                        face_wall = "dte_render_beige" if k == 0 else "paint_white"
                        f.face(face_wall, 0, L, z0, z0 + fh, -1.8, 2.0)
                        for u0, u1 in B.bays(L, 6.0, 0.6):
                            f.face("door_wood", u0 + 0.6, u0 + 1.6, z0, z0 + 2.2, -1.78)
                            f.face("glass_dark", u0 + 2.4, u1 - 0.6, z0 + 0.9, z0 + 2.2, -1.78, 2.0)
                        f.hplane("concrete", 0, L, -1.8, 0, z0 + fh - 0.3, False)
                        if k == 0:
                            f.hplane("dte_sand", 0, L, -1.8, 0, z0 + 0.01, True)
                        for u in np.linspace(0.3, L - 0.3, max(2, int(L // 6.0) + 1)):
                            f.block("concrete", u - 0.2, u + 0.2, -0.4, 0.0, z0, z0 + fh - 0.3, top=False, bottom=False)
                    else:
                        wall = "dte_render_beige" if k == 0 else "paint_white"
                        self.windows_row(f, 0, L, z0, z0 + fh, 4.0, wall, "glass_dark", 0.9, 2.6, mull=1)
                        if k > 0:
                            for u in np.arange(0.8, L - 0.5, 1.6):          # curved grey fins / 灰色のフィン
                                f.block("metal_grey", u - 0.05, u + 0.05, 0.0, 0.35, z0 + 0.2, z0 + fh - 0.1,
                                        bottom=False)
                if corr:
                    f.side("concrete", 0, -1.8, 0, Z0, top - 0.8, 1)
                    f.side("concrete", L, -1.8, 0, Z0, top - 0.8, -1)
        # the dark grey roof: one slab overhanging 0.6 m (no per-edge fascias: their
        # corners left dark holes), stepped 0.7 m every other block along the chain
        # 濃灰色の屋根: 0.6 m 張り出す一枚の板、棟ごとに 0.7 m の段差
        roofp = poly.buffer(0.6, join_style="mitre")
        if others is not None and not others.is_empty:
            roofp = roofp.difference(others)
        roofp = roofp.difference(self.room)
        zt = top + 0.03 * (idx % 3)                   # neighbours never coplanar / 隣と同一面にしない
        m.prism("dte_roof_dark", roofp, top - 0.8, zt, top=True, bottom=True, uv_scale=3.0)
        if line is not None and line.length > 30:
            from shapely.ops import substring
            stations = np.arange(13.0, line.length, 24.0)
            for j, (s0, s1) in enumerate(zip(stations, stations[1:])):
                if j % 2:
                    continue
                band = substring(line, s0, s1).buffer(25.0, cap_style="flat", join_style="mitre")
                piece = roofp.intersection(band).buffer(-0.4, join_style="mitre")
                for pc in polys(piece):
                    if pc.area > 20:
                        m.prism("dte_roof_dark", pc, zt, zt + 0.7, top=True, uv_scale=3.0)

    # ---- small buildings / 小さな建物
    def gallop(self, m, poly, b):
        ze, zr = Z0 + b["eave"], Z0 + b["ridge"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face("paint_white", 0, L, Z0, ze, 0.0, 2.0)
            for u0, u1 in B.bays(L, 3.6, 0.5):
                f.face("deck_wood", u0 + 0.4, u1 - 0.4, Z0, Z0 + 2.6, 0.03, 2.0)
        gable(m, poly, ze, zr, "roof_metal", "paint_white", ov=self.overhang(poly, 0.7))

    def swim_school(self, m, poly, b):
        top = Z0 + b["h"]
        core = poly.buffer(-0.8, join_style="mitre")
        for a, bb, L, n in B.edges_of(core):
            f = B.Facade(m, a, bb, n)
            f.face("paint_charcoal", 0, L, Z0, top - 0.8, 0.0, 2.0)
            for zz in (Z0 + 2.6, Z0 + 4.4):
                f.face("louvre", 0.6, L - 0.6, zz, zz + 1.0, 0.04, 2.0)
        m.prism("paint_white", poly, top - 0.8, top, top=True, bottom=True)

    def plant(self, m, poly, b):
        top = Z0 + b["h"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            f.face("concrete", 0, L, Z0, top - 3.0, 0.0, 2.0)
            f.face("louvre", 0, L, top - 3.0, top, 0.0, 2.0)
        self.flat_roof(m, poly, top - 3.0, "louvre", 0.0)
        c = poly.centroid
        for k in range(4):
            m.cylinder("tank", c.x - 12 + k * 8, c.y, top - 3.0, top - 0.5, 2.4, seg=12)

    # ---- outdoor structures / 屋外の構造物
    def walkway(self, m, line, w=3.0, z=3.2, mat="roof_linkway", posts=4.0):
        """A covered walkway along a line, cut back from roads and blocks. / 屋根付き歩道。"""
        roof = line.buffer(w / 2, cap_style="flat").difference(self.room.buffer(0.3)).difference(
            self.footprint.buffer(0.3))
        for p in polys(roof):
            if p.area < 3:
                continue
            m.flat(mat, p, Z0 + z, 2.0)
            m.flat("concrete", p, Z0 + z - 0.12, 2.0, up=False)
            m.ring_walls("metal_grey", p.exterior, Z0 + z - 0.12, Z0 + z, outward=True)
        for t in np.arange(0.5, line.length, posts):
            q = line.interpolate(t)
            if roof.buffer(-0.2).contains(q):
                m.box("metal_grey", q.x, q.y, Z0, Z0 + z - 0.12, 0.12, 0.12)

    def link_bridge(self, m, line, w=4.0, z0=6.0, z1=9.0):
        """The level-2 covered link bridge north to D'Resort: glazed sides, a
        dark roof, round piers; cut back from the road. / 2 階の屋根付き連絡橋。"""
        strip = line.buffer(w / 2, cap_style="flat").difference(self.room.buffer(0.3))
        strip = strip.difference(self.footprint.buffer(0.2))
        for p in polys(strip):
            if p.area < 10:
                continue
            m.prism("concrete", p, Z0 + z0 - 0.6, Z0 + z0, top=True, bottom=True)
            m.ring_walls("glass_dark", p.exterior, Z0 + z0, Z0 + z1 - 0.4, outward=True, uv_scale=2.0)
            m.prism("paint_charcoal", p.buffer(0.3, join_style="mitre").difference(self.room), Z0 + z1 - 0.4,
                    Z0 + z1, top=True, bottom=True)
            for t in np.arange(3.0, line.length, 12.0):
                q = line.interpolate(t)
                if p.buffer(-0.6).contains(q):
                    m.cylinder("concrete", q.x, q.y, Z0, Z0 + z0 - 0.6, 0.45, seg=10, top=False)

    def hotel_deck(self, m):
        """The level-2 drop-off between the hotel wings: a pedestrian deck at the
        raised ramp's height (xodr z + kerb) beside the lanes, never over them,
        glass rails, brown round columns below, a glass canopy on brown columns
        along the lobby side. / 2 階の車寄せ: 斜路の高さの歩行者デッキ（車線の上には掛けない）。"""
        top = [(pg, ln) for pg, ln in self.raised if max(q["z"] for q in ln["pts"]) > 5.0]
        self.deck_area = 0.0
        if not top:
            return
        zs = [q["z"] for pg, ln in top for q in ln["pts"] if q["z"] > 5.0]
        z = float(np.median(zs)) + 0.15
        wings = unary_union([self.blocks[k]["traced"] for k in ("hotel_w", "hotel_e")])
        region = wings.convex_hull.buffer(4.0)
        deck = unary_union([pg for pg, ln in top]).buffer(7.0, join_style="mitre").intersection(region)
        deck = deck.difference(self.no_go.buffer(0.1)).difference(self.footprint)
        deck = MultiPolygon([p for p in polys(deck) if p.area > 15.0]) if polys(deck) else None
        if deck is None or deck.is_empty:
            return
        self.deck_area = deck.area
        m.prism("concrete", deck, z - 0.6, z, top=False, bottom=True)
        m.flat("pavers", deck, z, 2.0)
        for p in polys(deck):
            c = list(p.exterior.coords)
            if not Polygon(c).exterior.is_ccw:
                c = c[::-1]
            for a, b in zip(c, c[1:]):
                mid = Point((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                if self.lane_area.distance(mid) > 1.5 and self.footprint.distance(mid) > 0.3:
                    m.wall("glass_rail", a, b, z, z + 1.1, 0.0, 1.0)
                    m.wall("glass_rail", b, a, z, z + 1.1, 0.0, 1.0)
            inner = p.buffer(-0.8)
            if not inner.is_empty:
                x0, y0, x1, y1 = inner.bounds
                for x in np.arange(x0, x1, 8.0):
                    for y in np.arange(y0, y1, 8.0):
                        if inner.contains(Point(x, y)):
                            m.cylinder("paint_woodbrown", x, y, Z0, z - 0.6, 0.35, seg=10, top=False)
        # the glass canopy along the lobby side / ロビー側のガラス屋根
        can = deck.intersection(self.footprint.buffer(6.0, join_style="mitre")).buffer(-0.3)
        for p in polys(can):
            if p.area < 10:
                continue
            m.flat("glass_dark", p, z + 4.6, 2.0)
            m.flat("glass_dark", p, z + 4.55, 2.0, up=False)
            m.ring_walls("paint_woodbrown", p.exterior, z + 4.2, z + 4.6, outward=True)
            ring = p.buffer(-0.4).exterior if p.buffer(-0.4).geom_type == "Polygon" else None
            if ring is not None:
                for t in np.arange(0.0, ring.length, 6.0):
                    q = ring.interpolate(t)
                    if self.footprint.distance(q) > 1.0:
                        m.cylinder("paint_woodbrown", q.x, q.y, z, z + 4.2, 0.22, seg=8, top=False)

    def courtyard_canopy(self, m):
        """The white curved canopy on thin dark columns through the chalets'
        hexagonal courtyard (DPA night photo). / 中庭を抜ける白い曲線の屋根。"""
        holes = [Polygon(r) for g, h in self.chalets.values() if g is not None
                 for p in polys(g) for r in p.interiors]
        if not holes:
            return
        yard = max(holes, key=lambda q: q.area).buffer(-2.0)
        if yard.is_empty or yard.geom_type != "Polygon":
            return
        c = yard.centroid
        x0, y0, x1, y1 = yard.bounds
        R = (x1 - x0) * 0.45
        pts = [(c.x - R + 2 * R * t, c.y + 0.35 * R * math.sin(2 * math.pi * t)) for t in np.linspace(0, 1, 25)]
        line = LineString(pts).intersection(yard)
        if line.geom_type != "LineString" or line.length < 8:
            return
        roof = line.buffer(1.8)
        m.flat("paint_white", roof, Z0 + 3.6, 2.0)
        m.flat("dte_soffit", roof, Z0 + 3.35, 2.0, up=False)
        m.ring_walls("paint_white", roof.exterior, Z0 + 3.35, Z0 + 3.6, outward=True)
        for t in np.arange(1.0, line.length, 4.0):
            q = line.interpolate(t)
            m.cylinder("paint_charcoal", q.x, q.y, Z0, Z0 + 3.35, 0.09, seg=6, top=False)

    def frontage_walk(self, m):
        """The covered walkway along Pasir Ris Drive 3 (the mapped footway in
        front of the open car park). / パシリス・ドライブ 3 沿いの屋根付き歩道。"""
        for ln, t, w in self.L.osm.lines("highway"):
            if t.get("highway") == "footway" and ln.length > 100 and \
                    ln.intersects(box(780, 1350, 940, 1395)):
                part = ln.intersection(box(775, 1340, 938, 1400))
                for g in getattr(part, "geoms", [part]):
                    if g.geom_type == "LineString" and g.length > 10:
                        self.walkway(m, g, 3.0, 3.2)

    def loop_island(self, m):
        """The drop-off loop island: a covered walk on its north side, the
        DOWNTOWN EAST planter at the entry. / 車寄せの島: 屋根付き歩道と植え込み。"""
        isl = largest(self.areas["loop_garden"].buffer(6.0).difference(self.no_go.buffer(0.5)))
        if isl is not None and isl.area > 100:
            band = isl.buffer(-0.6, join_style="mitre").difference(isl.buffer(-3.2, join_style="mitre"))
            band = band.intersection(box(-1e4, isl.bounds[3] - 4.5, 1e4, 1e4)).difference(self.footprint.buffer(0.5))
            for p in polys(band):
                if p.area < 6:
                    continue
                z = Z0 + 3.0
                m.flat("paint_white", p, z, 2.0)
                m.flat("concrete", p, z - 0.15, 2.0, up=False)
                m.ring_walls("paint_white", p.exterior, z - 0.15, z, outward=True)
                ring = p.buffer(-0.3).exterior if p.buffer(-0.3).geom_type == "Polygon" else None
                if ring is not None:
                    for t in np.arange(0.0, ring.length, 5.0):
                        q = ring.interpolate(t)
                        m.cylinder("metal_grey", q.x, q.y, Z0, z - 0.15, 0.09, seg=6, top=False)
        # the planter with the letters at the Pasir Ris Close entry / 入口の植え込みと文字
        for cx, cy in ((1012, 1660), (1012, 1668), (1010, 1652), (1008, 1676), (1014, 1680)):
            pl = box(cx - 5.0, cy - 1.2, cx + 5.0, cy + 1.2)
            if self.site.buffer(3).contains(pl) and not pl.buffer(0.5).intersects(self.no_go) \
                    and not pl.intersects(self.footprint):
                m.prism("concrete", pl, Z0, Z0 + 0.6, top=False)
                m.flat("leaves_dark", pl.buffer(-0.2), Z0 + 0.55, 2.0)
                g = text_poly("DOWNTOWN EAST", 0.55, "DejaVuSans-Bold.ttf")
                for p in polys(g):
                    m.prism("paint_white", translate(p, cx, cy), Z0 + 0.6, Z0 + 0.75, top=True)
                break

    def waterpark(self, m):
        """Wild Wild Wet: tube slides on the mapped slides, towers, the Tornado
        funnel, the tensile shade, kiosks, sails, the fence. / ウォーターパーク。"""
        cols = ["dte_slide_red", "dte_slide_yellow", "dte_slide_blue", "dte_slide_green"]
        towers = [p for p, t, w in self.kiosks if p.area > 60]
        tall = {"1117215040": 20.0, "1117209931": 12.0, "1117209901": 18.0, "1396567421": 4.5,
                "1117209936": 4.0, "1117209912": 6.0, "1117209918": 10.0}
        for k, (ln, t) in enumerate(self.slides):
            L = ln.length
            if L < 4:
                continue
            # the top is the end nearer a tower (else the first point) / 塔に近い端が上
            a, b = Point(ln.coords[0]), Point(ln.coords[-1])
            if towers:
                da = min(p.distance(a) for p in towers)
                db = min(p.distance(b) for p in towers)
                if db < da:
                    ln = LineString(list(ln.coords)[::-1])
            ztop = min(Z0 + 2.0 + L * 0.12, Z0 + 18.0)
            n = max(2, int(L / 2.0) + 1)
            pts = []
            for i in range(n):
                q = ln.interpolate(i / (n - 1), normalized=True)
                pts.append((q.x, q.y, ztop + (Z0 + 0.9 - ztop) * (i / (n - 1)) ** 0.8))
            racer = "Kraken" in (t.get("name") or "")
            tube(m, cols[k % 4], pts, r=0.45 if racer else 0.65, seg=6)
            for i in range(0, n, 3):
                x, y, z = pts[i]
                if z > Z0 + 1.6:
                    m.box("metal_grey", x, y, Z0, z - 0.6, 0.18, 0.18)
        # towers and kiosks from OSM / OSM の塔と売店
        for p, t, w in self.kiosks:
            h = tall.get(str(w))
            if h is None:
                h = 3.5 if p.area < 150 else 5.0
            if h >= 10.0:
                self.scaffold(m, p, Z0 + h)
            else:
                pp = p.minimum_rotated_rectangle if p.area < 400 else p
                m.prism("paint_cream", pp.buffer(-0.3), Z0, Z0 + h, top=False)
                if pp.geom_type == "Polygon" and len(pp.exterior.coords) == 5:
                    hip(m, pp, Z0 + h, Z0 + h + min(2.5, math.sqrt(pp.area) * 0.35), "roof_brown", ov=0.5)
                else:
                    m.flat("roof_brown", pp, Z0 + h, 2.0)
        # the Tornado funnel on its tower / トルネードの漏斗
        c = self.areas["tornado"]
        if not self.room.buffer(10).contains(c):
            for k in range(8):
                ang = k * math.pi / 4
                m.cylinder("metal_grey", c.x + 3.0 * math.cos(ang), c.y + 3.0 * math.sin(ang), Z0, Z0 + 6.5,
                           0.2, seg=6, top=False)
            self.funnel(m, c.x, c.y, Z0 + 6.5, Z0 + 14.0, 2.5, 9.0)
        # the white tensile shade: four peaks on masts / 白い膜屋根
        sh = self.areas["www_shade"]
        if not sh.intersects(self.room):
            q = [np.array(p) for p in list(orient(sh, 1.0).exterior.coords)[:4]]
            if math.dist(q[0], q[1]) < math.dist(q[1], q[2]):
                q = q[1:] + q[:1]
            a, b_, cc, d = q
            n = 4
            for i in range(n):
                p0, p1 = a + (b_ - a) * i / n, a + (b_ - a) * (i + 1) / n
                p3, p2 = d + (cc - d) * i / n, d + (cc - d) * (i + 1) / n
                pk = (p0 + p1 + p2 + p3) / 4
                ze, zp = Z0 + 4.5, Z0 + 9.0
                Pp = lambda v, z: (v[0], v[1], z)
                for e0, e1 in ((p0, p1), (p1, p2), (p2, p3), (p3, p0)):
                    two_sided(m, "dte_membrane", Pp(e0, ze), Pp(e1, ze), Pp(pk, zp), Pp(pk, zp))
                m.cylinder("metal_grey", pk[0], pk[1], Z0, zp + 0.8, 0.15, seg=6)
                for e in (p0, p3):
                    m.cylinder("metal_grey", e[0], e[1], Z0, ze, 0.1, seg=6, top=False)
            for e in (p1, p2):
                m.cylinder("metal_grey", e[0], e[1], Z0, ze, 0.1, seg=6, top=False)
        # red sail shades over the deck / 赤い帆の日除け
        rng = np.random.default_rng(7)
        deck = self.zones()["www"]
        for _ in range(60):
            if deck.is_empty:
                break
            x0, y0, x1, y1 = deck.bounds
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if not deck.buffer(-4).contains(Point(x, y)) or self.pools.buffer(2).contains(Point(x, y)):
                continue
            ang = rng.uniform(0, math.pi)
            r = 3.0
            A = (x + r * math.cos(ang), y + r * math.sin(ang), Z0 + 3.8)
            Bq = (x + r * math.cos(ang + 2.1), y + r * math.sin(ang + 2.1), Z0 + 3.0)
            Cq = (x + r * math.cos(ang + 4.2), y + r * math.sin(ang + 4.2), Z0 + 3.4)
            two_sided(m, "dte_sail", A, Bq, Cq, Cq)
            for p in (A, Bq, Cq):
                m.cylinder("metal_grey", p[0], p[1], Z0, p[2], 0.06, seg=5, top=False)
        # umbrellas over the deck (blue, red, yellow) / パラソル
        for i in range(140):
            if deck.is_empty:
                break
            x0, y0, x1, y1 = deck.bounds
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if not deck.buffer(-2.5).contains(Point(x, y)) or self.pools.buffer(1.5).contains(Point(x, y)):
                continue
            col = ("dte_slide_blue", "dte_slide_red", "dte_slide_yellow")[i % 3]
            m.cylinder("metal_grey", x, y, Z0, Z0 + 2.4, 0.04, seg=4, top=False)
            m.cylinder(col, x, y, Z0 + 2.0, Z0 + 2.6, 1.3, seg=8, r1=0.05, top=False)
        # the mesh fence along the park's road side / 道路側の金網
        if self.www is not None:
            ring = self.www.buffer(-0.5).exterior
            near = ring.intersection(self.no_go.buffer(12.0)).difference(self.room.buffer(0.5)) \
                .difference(self.footprint.buffer(1.0))
            for g in getattr(near, "geoms", [near]):
                if g.geom_type != "LineString" or g.length < 2:
                    continue
                cc_ = list(g.coords)
                for p0, p1 in zip(cc_, cc_[1:]):
                    m.wall("fence_green", p0, p1, Z0, Z0 + 2.0, 0.0, 1.0)
                    m.wall("fence_green", p1, p0, Z0, Z0 + 2.0, 0.0, 1.0)
                for t in np.arange(0.0, g.length, 2.5):
                    q = g.interpolate(t)
                    m.box("metal_grey", q.x, q.y, Z0, Z0 + 2.05, 0.06, 0.06)

    def scaffold(self, m, p, top):
        """A steel slide tower: corner posts, platforms every 3.5 m, a stair. / 鋼製の滑り台の塔。"""
        r = p.minimum_rotated_rectangle
        c = [np.array(q) for q in list(orient(r, 1.0).exterior.coords)[:4]]
        for q in c:
            beam(m, "dte_steel_blue", (q[0], q[1], Z0), (q[0], q[1], top), 0.15, caps=True)
        z = Z0 + 3.5
        while z <= top + 0.01:
            m.prism("metal_grey", r, z - 0.15, z, top=True, bottom=True)
            for i in range(4):
                a, b = c[i], c[(i + 1) % 4]
                beam(m, "dte_steel_blue", (a[0], a[1], z + 1.0), (b[0], b[1], z + 1.0), 0.05)
                beam(m, "dte_steel_blue", (a[0], a[1], z - 0.15), (b[0], b[1], z - 3.4), 0.06)
            z += 3.5
        hip(m, r, top + 2.2, top + 4.2, "dte_slide_blue", ov=0.4)
        for q in c:
            beam(m, "dte_steel_blue", (q[0], q[1], top), (q[0], q[1], top + 2.2), 0.1)

    def funnel(self, m, cx, cy, z0, z1, r0, r1, seg=20):
        """The open funnel, inside and outside faces. / 漏斗（内外の面）。"""
        for k in range(seg):
            a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
            P = lambda r, a, z: (cx + r * math.cos(a), cy + r * math.sin(a), z)
            mat = "dte_slide_red" if k % 2 else "dte_slide_yellow"
            m.quad(mat, P(r0, a0, z0), P(r0, a1, z0), P(r1, a1, z1), P(r1, a0, z1))
            m.quad(mat, P(r1, a0, z1 - 0.05), P(r1, a1, z1 - 0.05), P(r0, a1, z0 + 0.05), P(r0, a0, z0 + 0.05))
        m.cylinder("dte_slide_yellow", cx, cy, z0 - 0.3, z0, r0 + 0.2, seg=seg)

    def trees(self, m):
        """Trees in the D'Resort gardens, the loop island and round Aranda
        (simple crowns, clear of roads and blocks). / 庭の樹木。"""
        Z = self.zones()
        g = unary_union([Z["green"]]).difference(self.footprint.buffer(3.0))
        rng = np.random.default_rng(42)
        x0, y0, x1, y1 = g.bounds if not g.is_empty else (0, 0, 0, 0)
        shapely.prepare(g)
        n = 0
        for x in np.arange(x0, x1, 7.0):
            for y in np.arange(y0, y1, 7.0):
                px, py = x + rng.uniform(-3, 3), y + rng.uniform(-3, 3)
                r = rng.uniform(2.2, 3.4)
                q = Point(px, py)
                if not g.contains(q) or self.no_go.distance(q) < r + 0.6 or self.footprint.distance(q) < r:
                    continue
                h = rng.uniform(6.0, 10.0)
                m.cylinder("bark", px, py, Z0, Z0 + h * 0.55, 0.18, seg=6, top=False)
                m.sphere("leaves_rain", px, py, Z0 + h * 0.65, r, r, h * 0.3, seg=7, rings=4)
                n += 1
        self.n_trees = n


# -- textures / テクスチャ
def write_textures(tex_dir):
    """Drawn textures (procedural, colours from description.md; no imagery):
    panels, steel, curtain walls, glass, stripes, the D'Marquee bands and
    banners, standing seam, cladding, copper stripes, render, checker tiles,
    post railings, the pool deck, sand, membrane, slides, the brand discs.
    描画テクスチャ（description.md の色。画像は使わない）。"""
    import textures as X
    from PIL import Image, ImageDraw, ImageFont
    out = {}
    rs = np.random.default_rng(519599)
    fb = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

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

    def tinted(c, n=256, var=0.06, scales=(4, 16, 64)):
        return Image.fromarray(X.tint(X.noise(n, scales), np.array(c) * (1 - var), np.array(c) * (1 + var)))

    def panels(c, n=512, ku=4, kv=3, joint=0.75):
        im = tinted(c, n, 0.04)
        d = ImageDraw.Draw(im)
        jc = tuple(int(v * joint) for v in c)
        for i in range(ku + 1):
            d.line([(i * n / ku, 0), (i * n / ku, n)], fill=jc, width=3)
        for j in range(kv + 1):
            d.line([(0, j * n / kv), (n, j * n / kv)], fill=jc, width=3)
        return im

    save("dte_panel_grey", panels((175, 178, 180), ku=3, kv=2), 0.45, 0.5, 0.8)
    save("dte_steel", tinted((62, 64, 67), 64, 0.05), 0.55, 0.3)
    save("dte_steel_blue", tinted((50, 95, 165), 64, 0.05), 0.4, 0.6)
    save("dte_slab", panels((160, 160, 158), ku=2, kv=1, joint=0.85), 0.9, 0.0, 0.6)
    save("dte_roof_green", tinted((120, 135, 125), 256, 0.08), 0.85, 0.0, 0.5)
    save("dte_soffit", tinted((235, 235, 230), 64, 0.02), 0.7)

    def curtain(glass, spandrel, frame, n=256, sp=0.28):
        """One mullion bay and one floor: glass, the spandrel at the top. / 1 スパン 1 層。"""
        y = np.linspace(0, 1, n)[:, None, None]
        g = np.array(glass[1])[None, None, :] * (1 - y) + np.array(glass[0])[None, None, :] * y
        im = Image.fromarray(np.repeat(g, n, axis=1).astype(np.uint8))
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, n, int(n * sp)], fill=spandrel)
        fw = max(3, n // 40)
        d.rectangle([0, 0, n - 1, n - 1], outline=frame, width=fw)
        d.line([(0, int(n * sp)), (n, int(n * sp))], fill=frame, width=fw)
        return im

    save("dte_curtain", curtain(((60, 80, 92), (130, 155, 165)), (78, 80, 84), (95, 98, 102)), 0.1, 0.3)
    save("dte_glass_green", curtain(((80, 115, 105), (140, 170, 160)), (90, 120, 110), (180, 185, 185), sp=0.1),
         0.1, 0.3)
    rib = curtain(((35, 40, 45), (85, 95, 100)), (40, 44, 48), (60, 62, 64), sp=0.0)
    save("dte_glass_ribbon", rib, 0.08, 0.3)
    st = Image.new("RGB", (512, 512))
    d = ImageDraw.Draw(st)
    x = 0
    seq = [(235, 235, 232), (200, 201, 200), (150, 152, 152), (220, 220, 218), (175, 176, 176)]
    k = 0
    while x < 512:                                   # 0.6-1.2 m strips on a 4.8 m tile / 縦の帯
        w = int(rs.choice([64, 96, 128]))
        d.rectangle([x, 0, x + w, 512], fill=seq[k % len(seq)])
        d.line([(x, 0), (x, 512)], fill=(120, 120, 120), width=2)
        x += w
        k += 1
    save("dte_strips", st, 0.6, 0.2, 0.4)
    sh = Image.new("RGB", (256, 256), (170, 172, 172))
    d = ImageDraw.Draw(sh)
    for j in range(0, 256, 8):
        d.line([(0, j), (256, j)], fill=(120, 122, 122), width=2)
    save("dte_shutter", sh, 0.5, 0.6, 1.0)
    aw = Image.new("RGB", (256, 256), (240, 240, 235))
    d = ImageDraw.Draw(aw)
    for i in range(0, 256, 64):
        d.rectangle([i, 0, i + 32, 256], fill=(40, 120, 90))
    save("dte_awning", aw, 0.7)
    # glazed shopfronts (4 m tile): dark glass, white frames, a lit interior band,
    # a few posters / ガラスの店先（暗いガラス・白い枠・明るい店内・ポスター）
    sg = Image.new("RGB", (512, 512), (48, 58, 66))
    d = ImageDraw.Draw(sg)
    for i in range(4):
        x0 = i * 128
        d.rectangle([x0 + 6, 200, x0 + 122, 470], fill=(70 + 10 * (i % 2), 84, 92))
        d.rectangle([x0 + 10, 300, x0 + 118, 360], fill=(200, 190, 160))       # lit shelves / 店内
        if rs.random() < 0.5:
            c = tuple(int(v) for v in rs.integers(80, 240, 3))
            d.rectangle([x0 + 30, 220, x0 + 90, 290], fill=c)                 # poster / ポスター
        d.rectangle([x0, 0, x0 + 6, 512], fill=(225, 225, 222))
    d.rectangle([0, 470, 512, 512], fill=(200, 200, 196))
    d.rectangle([0, 190, 512, 200], fill=(225, 225, 222))
    save("dte_shopglass", sg, 0.1, 0.2)

    # D'Marquee bands / D'Marquee の帯
    save("dte_orange", tinted((230, 120, 30), 128, 0.04), 0.6)
    save("dte_purple", tinted((120, 60, 170), 128, 0.03), 0.5, 0.3)
    fl = np.asarray(tinted((150, 90, 190), 256, 0.03), np.float64)
    xs = np.arange(256)
    fl *= (0.82 + 0.18 * np.abs(np.sin(xs / 256 * math.pi * 8)))[None, :, None]
    save("dte_purple_fluted", Image.fromarray(fl.clip(0, 255).astype(np.uint8)), 0.45, 0.4, 1.5)
    gb = np.asarray(tinted((40, 100, 60), 256, 0.04), np.float64)
    gb *= np.where((xs % 32) < 4, 0.6, 1.0)[None, :, None]
    save("dte_green_batten", Image.fromarray(gb.clip(0, 255).astype(np.uint8)), 0.6, 0.0, 1.5)
    tm = Image.new("RGB", (512, 64))
    d = ImageDraw.Draw(tm)
    pal = [(200, 40, 50), (40, 160, 210), (250, 200, 40), (60, 170, 80), (130, 70, 180), (240, 130, 30)]
    for i in range(16):
        d.rectangle([i * 32, 0, i * 32 + 31, 64], fill=pal[int(rs.integers(0, len(pal)))])
    save("dte_tiles_multi", tm, 0.3)
    for k, (t1, t2) in enumerate((("EVENTS", "& EXHIBITIONS"), ("CREATE YOUR", "NEXT EVENT"))):
        im = Image.new("RGB", (768, 256), (70, 30, 110))
        d = ImageDraw.Draw(im)
        for i in range(12):
            d.line([(0, 256 - i * 30), (768, 120 - i * 18)], fill=(200, 60 + i * 12, 120), width=6)
        f = ImageFont.truetype(fb, 64)
        d.text((40, 40), t1, fill=(255, 255, 255), font=f)
        d.text((40, 130), t2, fill=(255, 255, 255), font=f)
        d.rectangle([2, 2, 765, 253], outline=(150, 90, 190), width=6)
        save("dte_banner_%d" % k, im, 0.4)
    # PlayHaus standing seam, hotel cladding, copper stripes, parapet panels
    seam = np.asarray(tinted((120, 125, 130), 256, 0.04), np.float64)
    seam *= np.where((xs % 32) < 3, 0.7, 1.0)[None, :, None]
    save("dte_roof_seam", Image.fromarray(seam.clip(0, 255).astype(np.uint8)), 0.5, 0.6, 1.2)
    save("dte_hotel_clad", panels((70, 60, 58), ku=2, kv=1, joint=0.8), 0.7, 0.1, 0.6)
    cp = Image.new("RGB", (512, 512), (72, 64, 62))
    d = ImageDraw.Draw(cp)
    # full-height copper-red stripes (about 45 %), a light brown one / 全高の銅色の縦縞（約 45 %）
    for x0, w, c in ((0, 70, (150, 80, 60)), (110, 40, (160, 88, 64)), (190, 24, (185, 150, 120)),
                     (250, 80, (148, 78, 58)), (380, 34, (160, 88, 64)), (450, 30, (150, 80, 60))):
        d.rectangle([x0, 0, x0 + w, 512], fill=c)
        d.line([(x0, 0), (x0, 512)], fill=(55, 48, 46), width=3)
    save("dte_copper", cp, 0.6, 0.2, 0.4)
    save("dte_copper_solid", tinted((150, 80, 60), 128, 0.05), 0.55, 0.3)
    save("dte_parapet_brown", panels((180, 150, 120), ku=2, kv=1), 0.7, 0.0, 0.5)
    save("dte_teal", tinted((30, 150, 140), 64, 0.03), 0.5)
    rd = np.asarray(tinted((44, 46, 50), 256, 0.05), np.float64)       # dark grey (OneMap) / 濃灰色
    rd *= np.where((xs % 43) < 3, 0.75, 1.0)[None, :, None]
    save("dte_roof_dark", Image.fromarray(rd.clip(0, 255).astype(np.uint8)), 0.75, 0.05, 1.0)
    # chalets: beige textured render with grey bands, checker tiles, post railing
    rb = np.asarray(tinted((205, 185, 150), 512, 0.07, (8, 32, 128)), np.float64)
    rb[:, 240:272] = np.array((150, 150, 148))
    save("dte_render_beige", Image.fromarray(rb.clip(0, 255).astype(np.uint8)), 0.9, 0.0, 0.8)
    ck = Image.new("RGB", (256, 256))
    d = ImageDraw.Draw(ck)
    for i in range(4):
        for j in range(4):
            c = [(150, 152, 154), (85, 88, 92), (200, 200, 198)][int(rs.integers(0, 3))]
            d.rectangle([i * 64, j * 64, i * 64 + 63, j * 64 + 63], fill=c)
    save("dte_checker", ck, 0.6, 0.0, 0.5)
    rp = Image.new("RGB", (256, 128), (60, 66, 60))
    d = ImageDraw.Draw(rp)
    for i in range(0, 256, 32):                      # posts every 0.125 m (texture 1 m) / 縦格子
        d.rectangle([i, 0, i + 14, 128], fill=(175, 178, 175))
    d.rectangle([0, 0, 256, 10], fill=(175, 178, 175))
    save("dte_rail_posts", rp, 0.4, 0.6)
    # the pool deck: yellow, blue, purple paint / プールサイドの塗装
    n = 1024
    a = X.noise(n, (3, 6, 12), (0.6, 0.3, 0.1))
    deck = np.zeros((n, n, 3))
    deck[a < 0.42] = (225, 190, 60)
    deck[(a >= 0.42) & (a < 0.6)] = (70, 120, 190)
    deck[a >= 0.6] = (130, 100, 175)
    deck[(a > 0.405) & (a < 0.425)] = (240, 240, 235)
    deck *= (0.94 + 0.06 * X.noise(n, (64, 256), (0.5, 0.5)))[..., None]
    save("dte_deck", Image.fromarray(deck.clip(0, 255).astype(np.uint8)), 0.7)
    save("dte_pool", Image.fromarray(X.tint(X.noise(256, (4, 16)), (40, 170, 190), (90, 210, 220))), 0.05)
    save("dte_sand", tinted((215, 195, 150), 256, 0.07, (16, 64, 256)), 0.95, 0.0, 0.8)
    save("dte_membrane", tinted((245, 245, 240), 64, 0.02), 0.6)
    save("dte_sail", tinted((200, 50, 45), 64, 0.03), 0.6)
    fs = np.asarray(tinted((235, 235, 230), 256, 0.02), np.float64)
    fs *= (0.85 + 0.15 * np.abs(((xs % 128) - 64) / 64.0))[None, :, None]
    save("dte_soffit_folded", Image.fromarray(fs.clip(0, 255).astype(np.uint8)), 0.7)
    for nm, c in (("red", (210, 50, 45)), ("yellow", (240, 195, 40)), ("blue", (40, 110, 200)),
                  ("green", (60, 170, 90))):
        save("dte_slide_" + nm, tinted(c, 64, 0.03), 0.3)
    # round discs on the exoskeleton: generic words and colours (no brand marks)
    # 外骨格の円盤（一般的な文字と色、商標は描かない）
    for k, (bg, fg, word) in enumerate((((200, 30, 40), (255, 255, 255), "FOOD"),
                                        ((250, 200, 30), (40, 40, 40), "E!HUB"),
                                        ((255, 255, 255), (200, 40, 60), "CINEMA"),
                                        ((30, 60, 140), (255, 255, 255), "FUN"))):
        im = Image.new("RGB", (256, 256), (255, 255, 255))
        d = ImageDraw.Draw(im)
        d.ellipse([10, 10, 246, 246], fill=bg)
        f = ImageFont.truetype(fb, 48 if len(word) < 5 else 36)
        bb = d.textbbox((0, 0), word, font=f)
        d.text((128 - (bb[2] - bb[0]) / 2, 128 - (bb[3] - bb[1]) / 2 - bb[1]), word, fill=fg, font=f)
        save("dte_disc_%d" % k, im, 0.5)
    return out
