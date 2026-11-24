"""White Sands Primary School (2 Pasir Ris Street 11, built 1998), PasirRisTampines
(maps/.venv). OSM has no school buildings here: every block was traced from the
z20 tiles (build/<Name>/wsps_photos, visual reference only, never textures),
heights from the OneMap 3-D height grid and tooltips, facades and colours from
Street View (St 11 2025, Dr 1 2022) and description.md (measured pixel medians).
Trace: build/<Name>/wsps_trace.json (map frame, made by wsps_trace_make.py,
roofs moved to their bases by the measured lean of the imagery).

  hall (A)      46 m barrel vault spanning NW-SE (eaves 12, crown 16 m), the NE
                gable on Pasir Ris Dr 1 with its grid of red / green / blue /
                yellow panels, louvres and the school's name; stepped terraces
                along the field
  link (B)      a higher barrel behind it (crown 21 m), classrooms on the road
  wing (G + H)  the curved outer wing and the straight SW wing as one vault
                rising inward (15.4 -> 18.6 m): punched windows with hoods,
                blue and yellow bands outside, open corridors on the courts
  inner (E)     the inner curved wing between the light wells, yellow band
  core (D)      the flat-roofed 5-storey core round light well W2, corridors on
                the court; the low round-cornered block (C) at the field corner
  front (I)     barrel along u with the white arched crown and round crest on
                St 11, royal-blue frame grid, the curved glass entrance canopy
  east (J)      4 storeys along the drive, yellow and blue bands, part open
                ground floor; NE block (K) on royal-blue pilotis, dark eaves;
                annex (L)
  grounds       artificial-turf field with goals and the tall ball fence, the
                inner netball court, the SE hard court, drop-off apron, the
                blue-and-white striped awning and grey covered walkways, the
                long white gatehouse with its green hipped roof, the name board
                wall, blue wall + grey railings on St 11, red wall + green mesh
                on Dr 1, sliding gates, trees
Roads win: strips (vaulted blocks) narrow until their walls keep GAP off the
road network (the estate road west of the hall is mapped ~5 m inside the
building), other blocks are cut, and any triangle left touching it is dropped.
Coordinates: the map frame (x east, y north, metres).

ホワイトサンズ小学校（1998 年）の詳細モデル。OSM に校舎が無いため z20 航空写真から
全棟をトレース、高さは OneMap、外観は Street View と description.md の実測色。
写真は参照のみ（テクスチャに使わない）。道路優先: ヴォールト棟は幅を詰め、他は切る。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from landmarks.coral import chainlink, drop_over, face_in, flat_roof, glyphs_on, opening, tiled_face, wall_openings
from landmarks.dte import beam, disc, gable as gable_roof, hip
from landmarks.prcs import text_poly
from mesh import Mesh, polys

NAME = "White Sands Primary School"
TAG = "WSPS"
TRACE = "wsps_trace.json"
MAP = "PasirRisTampines"
SITE_NAMES = (NAME,)
Z0 = 0.15                 # the raised ground / 敷地面
FH = 3.6                  # school storey / 階高
GAP = 1.0                 # walls this far off the road network / 道路網からの離れ
OV = 0.6                  # roof overhang / 軒の出
NJ = 10                   # vault columns across / ヴォールトの横分割
D_CORR = 2.0              # open corridor depth / 開放廊下の奥行き


def enabled(out, meta):
    """Only on PasirRisTampines, with its trace. / この地図とトレースがある時だけ。"""
    return meta.get("name") == MAP and os.path.exists(os.path.join(out, TRACE))


# -- primitives (tested in test_wsps.py: every face points the way it is seen)
# 基本形状（test_wsps.py で面の向きを確認）
def prof_z(prof, s):
    """Roof height across a vault: left eave at s=0, crown at s=sc, right eave
    at s=1, parabolic on each side. / ヴォールト断面の高さ。"""
    zl, zc, zr, sc = prof
    s = min(max(s, 0.0), 1.0)
    if s <= sc:
        x = 1.0 - s / sc if sc > 0 else 0.0
        return zl + (zc - zl) * (1.0 - x * x)
    x = (s - sc) / (1.0 - sc) if sc < 1 else 0.0
    return zr + (zc - zr) * (1.0 - x * x)


def vquad_out(m, mat, p, q, z0p, z1p, z0q, z1q, ref, uv=None):
    """A vertical quad p->q (bottom z0, top z1 at each end) facing away from the
    2-D point ref. / ref から外向きの垂直四角形。"""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    nrm = np.array([d[1], -d[0]])                    # right of p->q / p->q の右
    if float(nrm @ (0.5 * (p + q) - np.asarray(ref, float))) < 0:
        p, q, z0p, z0q, z1p, z1q = q, p, z0q, z0p, z1q, z1p
    L = float(np.linalg.norm(d))
    uv = uv or ((0, z0p), (L, z0q), (L, z1q), (0, z1p))
    m.quad(mat, (p[0], p[1], z0p), (q[0], q[1], z0q), (q[0], q[1], z1q), (p[0], p[1], z1p), *uv)


class Strip(object):
    """A vaulted block: a spine (2-D polyline) with widths left / right of it,
    mitred at the bends. Stations = spine vertices. / 帯状の棟（中心線と左右の幅）。"""

    def __init__(self, spine, wl, wr):
        self.P = [np.asarray(p, float) for p in spine]
        n = len(self.P)
        self.t, self.nl, self.k = [], [], []
        segs = [(self.P[i + 1] - self.P[i]) / np.linalg.norm(self.P[i + 1] - self.P[i]) for i in range(n - 1)]
        for i in range(n):
            if i == 0:
                t = segs[0]
            elif i == n - 1:
                t = segs[-1]
            else:
                t = segs[i - 1] + segs[i]
                t = t / np.linalg.norm(t)
            nl = np.array([-t[1], t[0]])
            sn = np.array([-segs[min(i, n - 2)][1], segs[min(i, n - 2)][0]])
            self.t.append(t)
            self.nl.append(nl)
            self.k.append(1.0 / max(0.5, float(nl @ sn)))
        self.wl = np.full(n, float(wl))
        self.wr = np.full(n, float(wr))

    @property
    def n(self):
        return len(self.P)

    def left(self, i, extra=0.0):
        return self.P[i] + self.nl[i] * self.k[i] * (self.wl[i] + extra)

    def right(self, i, extra=0.0):
        return self.P[i] - self.nl[i] * self.k[i] * (self.wr[i] + extra)

    def poly(self):
        ring = [tuple(self.right(i)) for i in range(self.n)] + [tuple(self.left(i)) for i in reversed(range(self.n))]
        return orient(Polygon(ring).buffer(0), 1.0)

    def fit(self, ng):
        """Roads win: narrow each side until the wall line keeps clear of ng
        (straight strips keep one width: rectangles stay rectangles). Returns
        the metres taken off. / 道路優先: 壁線が ng に掛からないよう幅を詰める。"""
        def clear(p, d, w):
            seg = LineString([tuple(p), tuple(p + d * w)])
            if not seg.intersects(ng):
                return w
            return max(0.0, Point(p).distance(seg.intersection(ng)) - 0.05)

        cut = 0.0
        if self.n == 2:
            L = float(np.linalg.norm(self.P[1] - self.P[0]))
            ts = np.linspace(0.0, 1.0, max(2, int(L) + 1))
            for side, d in ((self.wl, self.nl[0]), (self.wr, -self.nl[0])):
                w0 = side[0]
                w = min(clear(self.P[0] + (self.P[1] - self.P[0]) * t, d, w0) for t in ts)
                side[:] = w
                cut = max(cut, w0 - w)
            # the end caps: pull a spine end back while its cap touches ng / 端部が道路に掛かれば縮める
            for e in (0, 1):
                for _ in range(40):
                    cap = LineString([tuple(self.left(e)), tuple(self.right(e))])
                    if not cap.intersects(ng):
                        break
                    o = 1 - e
                    tt = (self.P[o] - self.P[e]) / np.linalg.norm(self.P[o] - self.P[e])
                    self.P[e] = self.P[e] + tt * 0.25
                    cut = max(cut, 0.25)
            return cut
        for i in range(self.n):
            for side, d in ((self.wl, self.nl[i] * self.k[i]), (self.wr, -self.nl[i] * self.k[i])):
                w0 = side[i]
                pts = [self.P[i]]
                if i > 0:
                    pts.append((self.P[i] + self.P[i - 1]) / 2)
                if i < self.n - 1:
                    pts.append((self.P[i] + self.P[i + 1]) / 2)
                w = min(clear(p, d, w0) for p in pts)
                side[i] = w
                cut = max(cut, w0 - w)
        return cut

    def locate(self, q):
        """(s across 0 = left .. 1 = right, distance along the spine) of a point. / 断面位置と距離。"""
        q = np.asarray(q, float)
        best, acc, s_along = None, 0.0, 0.0
        for i in range(self.n - 1):
            a, b = self.P[i], self.P[i + 1]
            ab = b - a
            L = float(np.linalg.norm(ab))
            t = min(max(float((q - a) @ ab) / (L * L), 0.0), 1.0)
            c = a + ab * t
            d = float(np.linalg.norm(q - c))
            if best is None or d < best[0]:
                nl = self.nl[i] * (1 - t) + self.nl[i + 1] * t
                nl = nl / np.linalg.norm(nl)
                wl = self.wl[i] * (1 - t) + self.wl[i + 1] * t
                wr = self.wr[i] * (1 - t) + self.wr[i + 1] * t
                off = float((q - c) @ nl)
                best = (d, (wl - off) / max(wl + wr, 1e-6), acc + L * t)
            acc += L
        return best[1], best[2]

    def edges(self):
        """CCW wall edges (a, b, side): right side forward, end cap, left side back,
        start cap. / 反時計回りの壁の辺（右側・終端・左側・始端）。"""
        out = []
        for i in range(self.n - 1):
            out.append((self.right(i), self.right(i + 1), "R", i))
        out.append((self.right(self.n - 1), self.left(self.n - 1), "E1", self.n - 1))
        for i in reversed(range(self.n - 1)):
            out.append((self.left(i + 1), self.left(i), "L", i))
        out.append((self.left(0), self.right(0), "E0", 0))
        return out


def vault(m, S, prof, mat, ext=(0.0, 0.0), ov=OV, thick=0.18, fascia="ws_fascia", soffit="ws_soffit",
          uvu=0.55):
    """Vault roof over a Strip: top faces up, overhangs at the sides (ov) and at
    the ends (ext), soffits down, fascia out. / ヴォールト屋根（上向き、軒裏は下、鼻隠しは外）。"""
    zl, zc, zr, sc = prof
    rows = []
    for i in range(S.n):
        rows.append((S.P[i], S.nl[i] * S.k[i], S.wl[i], S.wr[i]))
    if ext[0] > 0:
        p, nl, wl, wr = rows[0]
        rows.insert(0, (p - S.t[0] * ext[0], nl, wl, wr))
    if ext[1] > 0:
        p, nl, wl, wr = rows[-1]
        rows.append((p + S.t[-1] * ext[1], nl, wl, wr))
    G, Z, Ua = [], [], []
    acc = 0.0
    for r, (p, nl, wl, wr) in enumerate(rows):
        if r:
            acc += float(np.linalg.norm(p - rows[r - 1][0]))
        W = max(wl + wr, 1e-3)
        lft, rgt = p + nl * wl, p - nl * wr
        pts = [lft + nl / np.linalg.norm(nl) * ov] + [lft + (rgt - lft) * j / NJ for j in range(NJ + 1)] \
            + [rgt - nl / np.linalg.norm(nl) * ov]
        dl = min(max(2 * (zc - zl) / max(sc, 0.05) / W * ov, 0.08), 0.5)
        dr = min(max(2 * (zc - zr) / max(1 - sc, 0.05) / W * ov, 0.08), 0.5)
        zs = [zl - dl + 0.06] + [prof_z(prof, j / NJ) + 0.06 for j in range(NJ + 1)] + [zr - dr + 0.06]
        G.append(pts)
        Z.append(zs)
        Ua.append(acc)
    nr, nc = len(G), NJ + 3
    ctr = np.mean([q for row in G for q in row], axis=0)
    for r in range(nr - 1):
        for j in range(nc - 1):
            a, b = G[r][j], G[r][j + 1]
            c, d = G[r + 1][j + 1], G[r + 1][j]
            za, zb, zc_, zd = Z[r][j], Z[r][j + 1], Z[r + 1][j + 1], Z[r + 1][j]
            ua, ub = Ua[r] / uvu, Ua[r + 1] / uvu
            m.quad(mat, (*a, za), (*b, zb), (*c, zc_), (*d, zd), (ua, j / 3.0), (ua, (j + 1) / 3.0),
                   (ub, (j + 1) / 3.0), (ub, j / 3.0))
            over = j == 0 or j == nc - 2 or (ext[0] > 0 and r == 0) or (ext[1] > 0 and r == nr - 2)
            if over:
                m.quad(soffit, (*d, zd - thick), (*c, zc_ - thick), (*b, zb - thick), (*a, za - thick))
    # fascia round the edge / 外周の鼻隠し
    ring = [(r, 0) for r in range(nr)] + [(nr - 1, j) for j in range(1, nc)] + \
           [(r, nc - 1) for r in range(nr - 2, -1, -1)] + [(0, j) for j in range(nc - 2, 0, -1)] + [(0, 0)]
    for (r0, j0), (r1, j1) in zip(ring, ring[1:]):
        p, q = G[r0][j0], G[r1][j1]
        vquad_out(m, fascia, p, q, Z[r0][j0] - thick, Z[r0][j0], Z[r1][j1] - thick, Z[r1][j1], ctr)


def gable_fill(f, mat, S, prof, u0, u1, L, zfrom, side):
    """The wall above the eave line on a vault's end cap, under the profile:
    vertical strips facing out (+n). zfrom(u) = lowest z there.
    ヴォールト妻面の軒線より上（断面の下）の壁、外向き。"""
    if u1 - u0 < 0.05:
        return
    n = max(2, int(math.ceil((u1 - u0) / (L / NJ))))
    us = np.linspace(u0, u1, n + 1)
    for a, b in zip(us, us[1:]):
        # the cap runs right -> left (E1) or left -> right (E0) / 終端は右から左、始端は左から右
        sa, sb = (a / L, b / L) if side == "E0" else (1 - a / L, 1 - b / L)
        za, zb = prof_z(prof, sa), prof_z(prof, sb)
        lo_a, lo_b = zfrom(a), zfrom(b)
        if za <= lo_a + 0.01 and zb <= lo_b + 0.01:
            continue
        za, zb = max(za, lo_a), max(zb, lo_b)
        f.m.quad(mat, f.P(a, 0.0, lo_a), f.P(b, 0.0, lo_b), f.P(b, 0.0, zb), f.P(a, 0.0, za),
                 (a / 2, lo_a / 2), (b / 2, lo_b / 2), (b / 2, zb / 2), (a / 2, za / 2))


def drum_walls(m, mat, c, r, z0, z1, skip=None, seg=24, uvs=2.0):
    """Cylinder walls facing out, segments whose middle lies in ``skip`` left
    out (inside a neighbour). / 円筒の壁（外向き、隣棟の中の区間は省く）。"""
    cx, cy = c
    for k in range(seg):
        a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
        p = (cx + r * math.cos(a0), cy + r * math.sin(a0))
        q = (cx + r * math.cos(a1), cy + r * math.sin(a1))
        mid = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
        if skip is not None and skip.contains(Point(mid)):
            continue
        u0, u1 = r * a0 / uvs, r * a1 / uvs
        m.quad(mat, (p[0], p[1], z0), (q[0], q[1], z0), (q[0], q[1], z1), (p[0], p[1], z1),
               (u0, z0 / uvs), (u1, z0 / uvs), (u1, z1 / uvs), (u0, z1 / uvs))


def bars(m, mat, a, b, z0, z1, pitch=0.2, w=0.022):
    """Vertical railing bars along a->b as thin two-sided strips (seen through).
    縦格子（両面の細い帯、透けて見える）。"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    if L < 0.05:
        return
    t = (b - a) / L
    nrm = np.array([t[1], -t[0]]) * 0.004
    for s in np.arange(pitch / 2, L, pitch):
        p0, p1 = a + t * (s - w / 2), a + t * (s + w / 2)
        m.quad(mat, (*(p0 + nrm), z0), (*(p1 + nrm), z0), (*(p1 + nrm), z1), (*(p0 + nrm), z1))
        m.quad(mat, (*(p1 - nrm), z0), (*(p0 - nrm), z0), (*(p0 - nrm), z1), (*(p1 - nrm), z1))


class WSPS(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        # the trace is in the real map's frame; a re-centred scratch frame
        # converts through lon/lat / トレースは本来の地図座標（別の座標系なら緯度経度経由）
        lon0, lat0 = T.get("frame_lonlat0", [fr.lon0, fr.lat0])
        tf = geo.Frame({"lat0": lat0, "lon0": lon0})
        same = abs(lon0 - fr.lon0) < 1e-9 and abs(lat0 - fr.lat0) < 1e-9
        self.X = (lambda q: (float(q[0]), float(q[1]))) if same else \
            (lambda q: fr.xy(*tf.lonlat(float(q[0]), float(q[1]))))
        pts = lambda lst: [self.X(q) for q in lst]
        ox, oy, ang = T["frames"]["S"]
        self.frame = (self.X((ox, oy)), math.radians(ang))
        site = [p for p, t in L.osm.areas(lambda t: t.get("name") in SITE_NAMES and "building" not in t)]
        lot0 = Polygon(pts(T["lot"]))
        self.site = unary_union(site).buffer(0) if site else lot0.buffer(8.0)
        # roads win: the road network, islands and the site's own drives / 道路優先
        near = self.site.buffer(5.0)
        self.road = L.road_all
        self.no_go = unary_union([L.road_all, L.islands, L.driveways.intersection(near)])
        self.no_go_gap = self.no_go.buffer(GAP)
        self.cut_log = []
        # strips first (vaulted blocks), narrowed off the roads / ヴォールト棟（道路から幅を詰める）
        self.strips = {}
        for k, s in T["strips"].items():
            S = Strip(pts(s["spine"]), *s["w"])
            cut = S.fit(self.no_go_gap)
            if cut > 0.05:
                self.cut_log.append((k, cut, "m narrower"))
            self.strips[k] = dict(s, S=S, poly=S.poly(), prof=tuple(s["prof"]))
        self.wells = {k: orient(Polygon(pts(p)).buffer(0), 1.0) for k, p in T["wells"].items()}
        taken = unary_union([b["poly"] for b in self.strips.values()] + list(self.wells.values()))
        # the other blocks, each minus what the earlier ones hold, cut off the roads
        # 他の棟: 先の棟と重ならず、道路から切る
        self.blocks = {}
        for k in ("ne", "east", "annex", "corner", "core", "gate"):
            b = T["blocks"].get(k)
            if b is None:
                continue
            raw = orient(Polygon(pts(b["pts"])).buffer(0), 1.0)
            p = raw.difference(taken).difference(self.no_go_gap)
            p = max(polys(p), key=lambda q: q.area) if polys(p) else None
            cut = raw.difference(taken).area - (p.area if p is not None else 0.0)
            if cut > 0.5:
                self.cut_log.append((k, cut, "m2 cut"))
            if p is None or p.area < 8.0:
                continue
            p = orient(shapely.set_precision(p, 0.001).buffer(0), 1.0)
            if p.geom_type != "Polygon":
                p = orient(max(polys(p), key=lambda q: q.area), 1.0)
            taken = taken.union(p)
            self.blocks[k] = dict(b, poly=p)
        self.footprint = unary_union([b["poly"] for b in self.strips.values()]
                                     + [b["poly"] for b in self.blocks.values()])
        # the drum stair tower on the curved wing / 曲面棟の円筒階段塔
        d = T["drums"]["M"]
        self.drum_c, self.drum_r, self.drum_top = self.X(d["c"]), d["r"], d["top"]
        dd = self.no_go.distance(Point(self.drum_c))
        if dd < self.drum_r + GAP:
            self.drum_r = max(0.0, dd - GAP)
        # the curved glass entrance canopy / 曲面ガラスの玄関上屋
        cn = T["canopy"]
        cc = self.X(cn["c"])
        circ = Point(cc).buffer(cn["r"], 64)
        half = Polygon([self.at(cn["u_min"], -200), self.at(400, -200), self.at(400, 200), self.at(cn["u_min"], 200)])
        q = circ.intersection(half).difference(self.no_go.buffer(GAP)).difference(self.footprint)
        self.canopy = max(polys(q), key=lambda g: g.area) if polys(q) else None
        self.canopy_z = (cn["z_in"], cn["z_out"])
        C = T["courts"]
        self.courts = {k: (orient(Polygon(pts(c["pts"])), 1.0), c["mat"]) for k, c in C.items()}
        self.field = orient(Polygon(pts(T["field"])), 1.0)
        self.terrace = orient(Polygon(pts(T["terrace"])), 1.0).difference(self.footprint.buffer(0.05))
        self.drive = Polygon(pts(T["drive"]))
        self.parking = Polygon(pts(T["parking"]))
        # the school's lot: the traced boundary less the streets (and their kerb
        # band), the drive kept inside / 敷地: 道路を除き、構内の車路は含める
        lot = lot0.intersection(self.site.buffer(1.0)).difference(unary_union([L.road_all, L.islands]).buffer(1.2))
        lot = unary_union([lot, lot0.intersection(self.drive)])
        self.lot = max(polys(lot), key=lambda g: g.area) if polys(lot) else lot0
        self.centre = self.footprint.centroid
        self._parts = None
        self._ground = None
        for k, a, unit in self.cut_log:
            print("wsps: %s %.1f %s (road clearance)" % (k, a, unit))
        if os.environ.get("WSPS_DEBUG"):
            # opt-in: the plan in lon/lat for overlays on the tiles / 確認用に平面を緯度経度で書き出す
            ll = lambda g: shapely.transform(g, lambda c: np.array([fr.lonlat(x, y) for x, y in c]))
            near = self.site.buffer(40.0)
            dump = {"no_go": self.no_go.intersection(near), "footprint": self.footprint, "lot": self.lot,
                    "canopy": self.canopy if self.canopy is not None else Polygon()}
            dump.update({"s_" + k: v["poly"] for k, v in self.strips.items()})
            dump.update({"b_" + k: v["poly"] for k, v in self.blocks.items()})
            json.dump({k: shapely.to_wkt(ll(v), rounding_precision=8) for k, v in dump.items()},
                      open(os.environ["WSPS_DEBUG"], "w"))

    # -- frames / 座標系
    def axes(self):
        a = self.frame[1]
        return np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])

    def at(self, u, v):
        (ox, oy), _ = self.frame
        eu, ev = self.axes()
        return (ox + eu[0] * u + ev[0] * v, oy + eu[1] * u + ev[1] * v)

    def uv(self, x, y):
        (ox, oy), _ = self.frame
        eu, ev = self.axes()
        d = np.array([x - ox, y - oy])
        return float(d @ eu), float(d @ ev)

    def facing(self, n):
        """Side of the site frame a normal faces: N = +v (Dr 1), E = +u (St 11).
        法線の向き（N: Dr 1 側、E: St 11 側）。"""
        eu, ev = self.axes()
        cu, cv = float(n @ eu), float(n @ ev)
        if abs(cv) >= abs(cu):
            return "N" if cv > 0 else "S"
        return "E" if cu > 0 else "W"

    # -- heights / 高さ
    def height_at(self, k, q):
        """Roof (or parapet) height of block k over point q. / 棟 k の点 q での屋根高さ。"""
        if k in self.strips:
            s = self.strips[k]
            sa, _ = s["S"].locate(q)
            return prof_z(s["prof"], sa)
        b = self.blocks[k]
        return b.get("ridge", b["top"]) if b.get("roof") != "flat" else b["top"] + 0.9

    def wall_top(self, k):
        b = self.blocks[k]
        return b["top"]

    # -- layout / 配置
    def inner(self):
        return self.lot.buffer(-0.6, join_style="mitre")

    def keepout(self):
        """Where the generic trees and furniture must not go. / 汎用の木と小物を置かない所。"""
        return unary_union([self.inner(), self.footprint.buffer(3.0)])

    # -- ground / 地面
    def zones(self):
        if self._ground is not None:
            return self._ground
        inner = self.inner().difference(self.no_go.buffer(0.05))
        foot = self.footprint
        courts = unary_union([c for c, _ in self.courts.values()]).intersection(inner).difference(foot)
        field = self.field.intersection(inner).difference(foot).difference(self.terrace)
        parking = self.parking.intersection(inner).difference(foot)
        apron = self.drive.intersection(inner).difference(foot).difference(courts).difference(parking)
        hard = foot.buffer(2.4, join_style="mitre")
        if self.canopy is not None:
            hard = hard.union(self.canopy.buffer(0.5))
        hard = hard.intersection(inner).difference(foot).difference(courts).difference(field) \
            .difference(apron).difference(parking).difference(self.terrace)
        grass = inner.difference(foot).difference(courts).difference(field).difference(apron) \
            .difference(parking).difference(hard).difference(self.terrace)
        self._ground = dict(courts=courts, field=field, parking=parking, apron=apron, hard=hard, grass=grass)
        return self._ground

    def frame_uv(self, rect_pts, size_hint=None):
        """uvfun mapping a court rectangle (S-frame axes) to 0..1. / コートの UV。"""
        us, vs = zip(*[self.uv(*q) for q in rect_pts])
        u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
        (ox, oy), _ = self.frame
        eu, ev = self.axes()

        def f(V):
            d = V[:, :2] - np.array([ox, oy])
            return np.column_stack([(d @ eu - u0) / (u1 - u0), (d @ ev - v0) / (v1 - v0)])
        return f

    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        Z = self.zones()
        T = lambda g: g.intersection(tile)
        m.flat("grass_park", T(Z["grass"]), z + 0.005, 4.0)
        m.flat("concrete_path", T(Z["hard"]), z + 0.01, 2.0)
        m.flat("ws_pavers", T(Z["apron"]), z + 0.01, 2.0)
        m.flat("asphalt_lot", T(Z["parking"]), z + 0.008, 4.0)
        # field and courts textured in the site frame (continuous across tiles)
        # 運動場とコート（敷地座標で貼る: タイル境界で連続）
        m.flat("ws_turf", T(Z["field"]), z + 0.006, uvfun=self.frame_uv(list(self.field.exterior.coords)[:4]))
        for k, (c, mat) in self.courts.items():
            g = T(c.intersection(Z["courts"]))
            if not g.is_empty:
                m.flat(mat, g, z + 0.02, uvfun=self.frame_uv(list(c.exterior.coords)[:4]))
        # parking bay lines / 駐車区画線
        for ln in self.bay_lines():
            if tile.contains(ln.interpolate(0.5, normalized=True)):
                m.flat("paint_white", ln.buffer(0.06, cap_style="flat"), z + 0.02, 1.0)
        return m

    def bay_lines(self):
        """Staff bays along the parking strip's north edge (tiles: marked bays).
        職員駐車場の区画線。"""
        out = []
        pk = self.parking.intersection(self.inner()).difference(self.no_go.buffer(0.3))
        for u in np.arange(57.0, 75.1, 2.5):
            ln = LineString([self.at(u, -8.6), self.at(u, -13.6)])
            if pk.buffer(0.01).contains(ln):
                out.append(ln)
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
        guard = self.no_go.buffer(0.05)

        def add(anchor, fn, *a, **k):
            pm = Mesh()
            fn(pm, *a, **k)
            nd = drop_over(pm, guard)
            if nd:
                self.drop_log.append((fn.__name__, nd))
            parts.append((anchor, pm))

        for k in self.strips:
            c = self.strips[k]["poly"].centroid
            add((c.x, c.y), self.strip_block, k)
        for k in self.blocks:
            c = self.blocks[k]["poly"].centroid
            add((c.x, c.y), self.block, k)
        if self.drum_r > 1.0:
            add(self.drum_c, self.drum)
        if self.canopy is not None:
            c = self.canopy.centroid
            add((c.x, c.y), self.canopy_roof)
        c = self.site.centroid
        add((c.x, c.y), self.fences)
        add((c.x, c.y), self.walkways)
        add((c.x, c.y), self.trees)
        f = self.field.centroid
        add((f.x, f.y), self.field_kit)
        add(self.X(self.T["sign"][0]), self.name_wall)
        for nm, n in self.drop_log:
            print("wsps: %s: %d triangles over the road network dropped" % (nm, n))
        self.tris = sum(pm.ntris() for _, pm in parts)
        print("wsps: %d vaulted + %d other blocks, %d triangles" % (len(self.strips), len(self.blocks), self.tris))
        return parts

    def pieces(self, k, a, b, n):
        """Runs [(u0, u1, z_cover)] of an edge: z_cover = Z0 where it is free,
        else the roof height of the block touching it (within 0.15 m).
        辺の区間と、接する隣棟の高さ。"""
        L = float(np.linalg.norm(b - a))
        t = (b - a) / L
        others = [(kk, bb["poly"]) for kk, bb in list(self.strips.items()) + list(self.blocks.items()) if kk != k]
        step = 0.25
        us = np.arange(step / 2, L, step)
        cov = []
        for u in us:
            q = a + t * u + n * 0.15
            p = Point(*q)
            h = Z0
            for kk, poly in others:
                if poly.contains(p):
                    h = max(h, self.height_at(kk, q))
            cov.append(round(h, 2))
        runs = []
        for u, h in zip(us, cov):
            u0, u1 = max(0.0, u - step / 2), min(L, u + step / 2)
            if runs and abs(runs[-1][2] - h) < 0.3:
                runs[-1][1] = u1
                runs[-1][2] = max(runs[-1][2], h)
            else:
                runs.append([u0, u1, h])
        return [tuple(r) for r in runs]

    # -- vaulted blocks / ヴォールト棟
    def strip_block(self, m, k):
        s = self.strips[k]
        S, prof, kind = s["S"], s["prof"], s["kind"]
        zl, zc, zr, sc = prof
        ext = [0.0, 0.0]
        for a, b, side, i in S.edges():
            L = float(np.linalg.norm(b - a))
            if L < 0.05:
                continue
            t = (b - a) / L
            n = np.array([t[1], -t[0]])
            f = B.Facade(m, a, b, n)
            top = zl if side == "L" else zr if side == "R" else min(zl, zr)
            runs = self.pieces(k, a, b, n)
            for u0, u1, zcov in runs:
                if u1 - u0 < 0.05:
                    continue
                if zcov > Z0 + 0.01:
                    if top > zcov + 0.01:
                        f.face("ws_white", u0, u1, zcov, top, 0.0, 2.0)
                    continue
                self.strip_facade(f, k, kind, side, self.facing(n), u0, u1, L, top, i)
            if side in ("E0", "E1"):
                covs = [(u0, u1, zcov) for u0, u1, zcov in runs]
                zfrom = lambda u, covs=covs: max([min(zl, zr)] + [zcov for a0, a1, zcov in covs if a0 <= u <= a1])
                free = sum(u1 - u0 for u0, u1, zcov in runs if zcov < min(zl, zr) - 0.5) > 0.6 * L
                gable_fill(f, "ws_white", S, prof, 0.0, L, L, zfrom, side)
                if free:
                    ext[0 if side == "E0" else 1] = OV
                    self.gable_detail(f, k, kind, side, L, prof)
        vault(m, S, prof, "ws_roof", ext=tuple(ext))

    def strip_facade(self, f, k, kind, side, face, u0, u1, L, top, i):
        if kind == "hall":
            if side in ("E0", "E1"):
                return self.hall_gable(f, u0, u1, L, top) if face == "N" else self.plain(f, u0, u1, top)
            return self.hall_side(f, u0, u1, L, top)
        if kind == "link":
            if side in ("E0", "E1"):
                return self.plain(f, u0, u1, top, louvre=True)
            return self.class_out(f, u0, u1, L, top, 4)
        if kind == "wing":
            if side == "R":
                return self.class_out(f, u0, u1, L, top, 4)
            if side == "L":
                return self.corridor(f, u0, u1, L, 4, top)
            return self.end_wall(f, u0, u1, L, top, 4)
        if kind == "inner":
            if side in ("E0", "E1"):
                return self.end_wall(f, u0, u1, L, top, 4)
            return self.class_out(f, u0, u1, L, top, 4, yellow_only=True)
        if kind == "front":
            if side == "E1":
                return self.front_face(f, u0, u1, L, top)
            if side == "E0":
                return self.class_out(f, u0, u1, L, top, 4)
            return self.class_out(f, u0, u1, L, top, 4)
        return self.plain(f, u0, u1, top)

    def gable_detail(self, f, k, kind, side, L, prof):
        """What sits on a free vault gable above the eave: the hall's louvre band,
        the front's blue arch band and crest. / 自由な妻面の上部（ガラリ・青いアーチ帯・校章）。"""
        zl, zc, zr, sc = prof
        ze = min(zl, zr)
        if kind == "hall":
            half = L * 0.28
            tiled_face(f, "ws_louvre", L / 2 - half, L / 2 + half, ze + 1.2, ze + 2.4, 0.05, 1.5, 1.2, v0=ze + 1.2)
        if kind == "front":
            # the blue band just under the arch, then the crest at the crown / アーチ下の青い帯と校章
            n = 24
            for j in range(n):
                a, b = L * j / n, L * (j + 1) / n
                sa, sb = (a / L, b / L) if side == "E0" else (1 - a / L, 1 - b / L)
                za, zb = prof_z(prof, sa) - 0.15, prof_z(prof, sb) - 0.15
                f.m.quad("ws_blue", f.P(a, 0.12, za - 0.55), f.P(b, 0.12, zb - 0.55), f.P(b, 0.12, zb),
                         f.P(a, 0.12, za))
                f.m.quad("ws_blue", f.P(a, 0.0, za), f.P(b, 0.0, zb), f.P(b, 0.12, zb), f.P(a, 0.12, za))
            c = f.a + f.t * (L / 2) + f.n * 0.2
            disc(f.m, "ws_crest", "ws_white", (c[0], c[1], zc - 1.75), f.n, 1.05, seg=20, t=0.2)

    def plain(self, f, u0, u1, top, louvre=False):
        f.face("ws_white", u0, u1, Z0, top, 0.0, 2.0)
        if louvre and top - Z0 > 6:
            tiled_face(f, "ws_louvre", u0 + 0.4, u1 - 0.4, top - 2.6, top - 1.4, 0.04, 1.5, 1.2, v0=top - 2.6)

    @staticmethod
    def bay_centres(L, bay, margin=0.8):
        n = int((L - 2 * margin) // bay)
        if n <= 0:
            return [L / 2] if L > 2.6 else []
        w = (L - 2 * margin) / n
        return [margin + (i + 0.5) * w for i in range(n)]

    def class_out(self, f, u0, u1, L, top, storeys, yellow_only=False, zb=Z0):
        """Classroom side (SV01, SV04): white walls, a window per 3.6 m bay with
        a concrete hood, blue floor bands, the yellow band and the blue band
        under the eave. / 教室側: 白い壁、ベイごとの窓と庇、青い床帯、軒下の黄・青帯。"""
        ww = 2.5 if L > 3.0 else max(0.8, L - 0.9)
        cs = [c for c in self.bay_centres(L, FH, 0.6) if u0 + 0.2 <= c - ww / 2 and c + ww / 2 <= u1 - 0.2]
        zt = Z0 + storeys * FH
        for s in range(storeys):
            z0 = Z0 + s * FH
            z1 = z0 + FH if s < storeys - 1 else max(top, z0 + FH)
            if z1 <= zb + 0.05:
                continue
            lo = max(z0, zb)
            opens = [(c - ww / 2, c + ww / 2, z0 + 0.95, z0 + 2.45) for c in cs if z0 + 0.95 > lo + 0.05]
            wall_openings(f, "ws_white", u0, u1, lo, min(z1, top), opens, 2.0)
            for o in opens:
                opening(f, *o, "ws_white", "ws_glass", 0.15, frame=None)
            if opens:
                f.block("concrete", u0 + 0.1, u1 - 0.1, 0.0, 0.45, z0 + 2.55, z0 + 2.67, ends=True)
            if s > 0 and z0 > zb + 0.3:
                f.block("ws_blue", u0, u1, 0.0, 0.06, z0 - 0.12, z0 + 0.14, ends=False)
        if top > zt + 0.05:
            f.face("ws_white", u0, u1, zt, top, 0.0, 2.0)
        # bands under the eave: yellow over the top storey's windows, blue at the top
        # 軒下の帯: 最上階の窓上に黄、最上部に青
        zy = Z0 + (storeys - 1) * FH + 2.75
        if zy > zb + 0.3 and not yellow_only:
            f.block("ws_yellow", u0, u1, 0.0, 0.07, zy, zy + 0.75, ends=False)
        elif zy > zb + 0.3:
            f.block("ws_yellow", u0, u1, 0.0, 0.07, Z0 + (storeys - 2) * FH + 2.75, Z0 + (storeys - 2) * FH + 3.5,
                    ends=False)
        if top - 0.45 > zb:
            f.block("ws_blue", u0, u1, 0.0, 0.08, top - 0.45, top, ends=False, bottom=True, top=False)
        if zb <= Z0 + 0.01:
            f.block("concrete", u0, u1, 0.0, 0.04, Z0, Z0 + 0.35, ends=False, bottom=False)

    def corridor(self, f, u0, u1, L, storeys, top, parapets=("ws_yellow", "ws_white")):
        """Open corridors on the courts (SV04 across the field): blue slab edges,
        parapets yellow / white by bay, the corridor wall behind (doors and
        windows drawn, a real recess), blue piers. / 開放廊下（青いスラブ端、黄・白の腰壁、通し柱）。"""
        D = D_CORR
        zt = Z0 + storeys * FH
        if u0 < 0.01 and L > 1.0:
            f.face("ws_white", u0, u0 + 0.25, Z0, top, 0.0)
            u0 += 0.25
        if u1 > L - 0.01 and L > 1.0:
            f.face("ws_white", u1 - 0.25, u1, Z0, top, 0.0)
            u1 -= 0.25
        if u1 - u0 < 0.4:
            return
        g = B.Facade(f.m, f.a - f.n * D, f.a - f.n * D + f.t * f.L, f.n)
        for s in range(storeys):
            z0 = Z0 + s * FH
            z1 = z0 + FH
            tiled_face(g, "ws_corr", u0, u1, z0, z1 - 0.3, 0.0, 3.6, FH - 0.3, v0=z0)
            f.hplane("concrete", u0, u1, -D, 0.0, z0 + 0.01, True)
            f.hplane("ws_soffit", u0, u1, -D, 0.0, z1 - 0.3, False)
            f.face("ws_blue", u0, u1, z1 - 0.3, z1, 0.0)
            if s == 0:
                f.face("concrete", u0, u1, Z0 - 0.15, Z0 + 0.01, 0.0)
                continue
            mat = parapets[(int((f.a @ np.array([1.0, 0.37])) // 7) + s) % len(parapets)] if L < 6 else None
            mods = self.bay_centres(L, 7.2, 0.0) if L >= 6 else [L / 2]
            edges = [0.0] + [(mods[i] + mods[i + 1]) / 2 for i in range(len(mods) - 1)] + [L]
            for j in range(len(edges) - 1):
                a, b = max(u0, edges[j]), min(u1, edges[j + 1])
                if b - a < 0.05:
                    continue
                pm = mat or parapets[(j + s) % len(parapets)]
                f.block(pm, a, b, -0.15, 0.0, z0 + 0.01, z0 + 1.0, top=True, bottom=False, ends=False)
                face_in(f, pm, a, b, z0 + 0.01, z0 + 1.0, -0.15)
                f.block("ws_rail", a, b, -0.11, -0.04, z0 + 1.0, z0 + 1.06, ends=False, bottom=False)
        if top > zt + 0.01:
            f.face("ws_white", u0, u1, zt, top, 0.0, 2.0)
            f.block("ws_blue", u0, u1, 0.0, 0.08, top - 0.45, top, ends=False, top=False)
        if L >= 6:
            mods = self.bay_centres(L, 7.2, 0.0)
            pts = [0.6] + [(mods[i] + mods[i + 1]) / 2 for i in range(len(mods) - 1)] + [L - 0.6]
        else:
            pts = [0.3] if u0 < 0.5 else []
        for p in pts:
            if u0 + 0.25 <= p <= u1 - 0.25:
                f.block("ws_blue", p - 0.25, p + 0.25, -D, 0.06, Z0, zt, top=False, bottom=False)
        f.side("ws_white", u0, -D, 0.0, Z0, zt, 1)
        f.side("ws_white", u1, -D, 0.0, Z0, zt, -1)

    def end_wall(self, f, u0, u1, L, top, storeys):
        """Block ends: white, a blue panel, a yellow top corner, a window per
        storey. / 妻壁: 白地に青いパネル、黄色い角、各階に窓。"""
        c = L / 2
        opens = [(c - 0.7, c + 0.7, Z0 + s * FH + 1.0, Z0 + s * FH + 2.4) for s in range(storeys)
                 if u0 + 0.2 <= c - 0.7 and c + 0.7 <= u1 - 0.2 and Z0 + s * FH + 2.4 < top]
        wall_openings(f, "ws_white", u0, u1, Z0, top, opens, 2.0)
        for o in opens:
            opening(f, *o, "ws_white", "ws_glass", 0.15, frame=None)
        p0, p1 = max(u0, L * 0.12), min(u1, L * 0.3)
        if p1 > p0 + 0.3:
            f.block("ws_blue", p0, p1, 0.0, 0.06, Z0 + FH, top - 0.5, bottom=True)
        y0, y1 = max(u0, L * 0.72), min(u1, L - 0.3)
        if y1 > y0 + 0.3:
            f.block("ws_yellow", y0, y1, 0.0, 0.06, top - 3.2, top - 0.5, bottom=True, top=True)

    # -- the hall / ホール
    def hall_side(self, f, u0, u1, L, top):
        """The hall's long walls: white, two rows of high windows with blue sill
        and head bands, louvred openings low. / ホールの長辺: 白壁に 2 段の高窓と青い帯。"""
        rows = [(4.4, 5.8), (8.0, 9.6)]
        cs = [c for c in self.bay_centres(L, 4.5) if u0 + 0.2 <= c - 1.4 and c + 1.4 <= u1 - 0.2]
        opens = [(c - 1.4, c + 1.4, z0, z1) for z0, z1 in rows for c in cs]
        low = [(c - 1.2, c + 1.2, Z0 + 0.6, Z0 + 2.6) for c in cs[::2]]
        wall_openings(f, "ws_white", u0, u1, Z0, top, opens + low, 2.0)
        for o in opens:
            opening(f, *o, "ws_white", "ws_glass", 0.15, sill=False, frame=None)
        for o in low:
            opening(f, *o, "ws_white", "ws_louvre", 0.12, sill=False, frame=None)
        for z0, z1 in rows:
            f.block("ws_blue", u0, u1, 0.0, 0.08, z0 - 0.3, z0, ends=False)
        f.block("ws_blue", u0, u1, 0.0, 0.1, top - 0.4, top, ends=False, top=False)

    def hall_gable(self, f, u0, u1, L, top):
        """The NE gable on Dr 1 (SV04, Littleone photo): white, a grid of square
        panels in red / green / blue / yellow / teal, a grey louvre band below,
        the school's name in blue letters. / 北東妻面: 色の正方形パネルの格子、ガラリ帯、校名。"""
        f.face("ws_white", u0, u1, Z0, top, 0.0, 2.0)
        cols = ("ws_pan_r", "ws_pan_g", "ws_pan_b", "ws_pan_y", "ws_pan_t", "ws_pan_g", "ws_pan_y", "ws_pan_r",
                "ws_pan_b", "ws_pan_t", "ws_pan_y", "ws_pan_g")
        sz, gap = 2.9, 0.3
        n = 10
        x0 = L / 2 - n * (sz + gap) / 2
        for r, zb in enumerate((4.1, 4.1 + sz + gap)):
            for j in range(n):
                a = x0 + j * (sz + gap)
                b = a + sz
                if a < u0 + 0.3 or b > u1 - 0.3:
                    continue
                f.block(cols[(j + 3 * r) % len(cols)], a, b, 0.0, 0.08, zb, zb + sz)
        # louvre band and the entrance doors below / 下部のガラリ帯と扉
        tiled_face(f, "ws_louvre", max(u0, x0), min(u1, L - x0), Z0 + 0.4, 3.4, 0.03, 1.5, 3.0, v0=Z0 + 0.4)
        f.block("ws_blue", max(u0, x0 - 0.3), min(u1, L - x0 + 0.3), 0.0, 0.12, 3.5, 3.8, ends=True)
        if u0 < L / 2 < u1:
            g = text_poly("WHITE SANDS PRIMARY SCHOOL", 0.75, "DejaVuSans-Bold.ttf")
            glyphs_on(f.m, f, g, L / 2, top - 1.25, 0.03, "ws_blue", 0.75)

    # -- the front block / 正面棟
    def front_face(self, f, u0, u1, L, top):
        """The front on St 11 (SV02, SV06): a royal-blue frame grid (fins per
        3.6 m, floor bands) on white with glazing behind, the entrance glazing
        under the canopy, a white frieze up to the eave. / 正面: 青い格子枠とガラス、白いフリーズ。"""
        zt = Z0 + 4 * FH
        a0, a1 = u0 + 0.5, u1 - 0.5
        if a1 - a0 < 1.0:
            return self.plain(f, u0, u1, top)
        f.face("ws_white", u0, a0, Z0, top, 0.0, 2.0)
        f.face("ws_white", a1, u1, Z0, top, 0.0, 2.0)
        tiled_face(f, "ws_glass_grid", a0, a1, Z0, zt, -0.25, 3.6, FH)
        f.side("ws_white", a0, -0.25, 0.0, Z0, zt, 1)
        f.side("ws_white", a1, -0.25, 0.0, Z0, zt, -1)
        f.face("ws_white", a0, a1, zt, top, 0.0, 2.0)
        f.hplane("ws_white", a0, a1, -0.25, 0.0, zt, False)
        for s in range(1, 5):
            z = Z0 + s * FH
            f.block("ws_blue", a0, a1, -0.25, 0.35, z - 0.35, z, ends=True)
        for c in np.arange(a0 + 1.8, a1 - 0.5, 3.6):
            f.block("ws_blue", c - 0.17, c + 0.17, -0.25, 0.5, Z0, zt, top=True, bottom=False)
        f.block("ws_white", a0, a1, 0.0, 0.3, top - 0.3, top, ends=True)

    # -- other blocks / その他の棟
    def block(self, m, k):
        b = self.blocks[k]
        poly, top, kind = b["poly"], b["top"], b["kind"]
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            side = self.facing(n)
            for u0, u1, zc in self.pieces(k, a, bb, n):
                if u1 - u0 < 0.05:
                    continue
                if zc > Z0 + 0.01:
                    if top > zc + 0.01:
                        self.upper(f, kind, u0, u1, L, zc, top)
                    continue
                self.block_facade(f, k, kind, side, u0, u1, L, top)
        roof = b.get("roof")
        if roof == "gable":
            fasc = "ws_eave_dark" if kind == "ne" else "ws_fascia"
            ov = 1.0 if kind == "ne" else 0.6
            ov = min(ov, max(0.0, self.no_go.distance(poly) - 0.4))
            gable_roof(m, poly, top, b["ridge"], "ws_roof", "ws_white", ov=ov)
            if ov > 0.05:
                # the fascia round the eaves (K: a deep dark band, SV03) / 軒の鼻隠し（K は濃色の太い帯）
                dz = 0.7 if kind == "ne" else 0.35
                m.ring_walls(fasc, poly.buffer(ov, join_style="mitre").exterior, top - dz, top + 0.05)
                m.flat("ws_soffit", poly.buffer(ov, join_style="mitre").difference(poly), top - dz, 2.0, up=False)
        elif roof == "hip":
            hip(m, poly, top, b["ridge"], "ws_gate_roof", ov=min(0.6, max(0.0, self.no_go.distance(poly) - 0.3)))
        else:
            par = 0.9 if kind == "core" else 0.6
            flat_roof(m, poly, top, "ws_white", parapet=par, cap="ws_blue" if kind != "annex" else "ws_white")
            if kind in ("annex", "core"):
                self.roof_plant(m, poly, top, 2 if kind == "annex" else 4)

    def upper(self, f, kind, u0, u1, L, zc, top):
        """Above a lower neighbour: windows where a classroom face goes on, else
        plain. / 低い隣棟より上: 窓または無地。"""
        if kind in ("core", "east", "ne") and top - zc > 3.0:
            st = 5 if kind == "core" else 4
            self.class_out(f, u0, u1, L, top, st, zb=zc)
        else:
            f.face("ws_white", u0, u1, zc, top, 0.0, 2.0)

    def block_facade(self, f, k, kind, side, u0, u1, L, top):
        if kind == "core":
            return self.corridor(f, u0, u1, L, 5, top)
        if kind == "corner":
            return self.class_out(f, u0, u1, L, top, 2)
        if kind == "east":
            if side == "W" or side == "N":
                return self.corridor(f, u0, u1, L, 4, top) if L > 8 else self.end_wall(f, u0, u1, L, top, 4)
            if L < 20:
                return self.end_wall(f, u0, u1, L, top, 4)
            return self.east_face(f, u0, u1, L, top)
        if kind == "ne":
            if side in ("N", "S"):
                return self.ne_long(f, u0, u1, L, top, side)
            return self.ne_end(f, u0, u1, L, top)
        if kind == "annex":
            return self.annex_wall(f, u0, u1, L, top)
        if kind == "gatehouse":
            return self.gate_wall(f, u0, u1, L, top)
        return self.plain(f, u0, u1, top)

    def east_face(self, f, u0, u1, L, top):
        """J's face on the drive (SV02, OneMap): 4 storeys of windows, blue floor
        bands, a yellow band; the NE part of the ground floor open on blue
        columns. / J の車路側: 4 層の窓、青い帯、黄色い帯、北東側の 1 階は柱の吹放し。"""
        # which end is north-east (+v) / 北東の端
        ne_end = L if float(f.t @ self.axes()[1]) > 0 else 0.0
        # the recess stops 0.3 m short of a corner (its end face must not lie in
        # the next wall's plane) / 凹みは角の 0.3 m 手前で止める
        o0, o1 = (L * 0.55, L - 0.3) if ne_end > 0 else (0.3, L * 0.45)
        a, b = max(u0, o0), min(u1, o1)
        if b > a + 1.0:
            # the open ground floor: recessed dark wall, columns on the line / 吹放しの 1 階
            g = B.Facade(f.m, f.a - f.n * 2.4, f.a - f.n * 2.4 + f.t * L, f.n)
            tiled_face(g, "ws_corr", a, b, Z0, Z0 + FH - 0.3, 0.0, 3.6, FH - 0.3, v0=Z0)
            f.hplane("ws_soffit", a, b, -2.4, 0.0, Z0 + FH - 0.3, False)
            f.hplane("concrete", a, b, -2.4, 0.0, Z0 + 0.01, True)
            f.side("ws_white", a, -2.4, 0.0, Z0, Z0 + FH - 0.3, 1)
            f.side("ws_white", b, -2.4, 0.0, Z0, Z0 + FH - 0.3, -1)
            for c in np.arange(a + 1.2, b - 0.5, 6.0):
                q = f.a + f.t * c - f.n * 0.4
                f.m.cylinder("ws_blue", q[0], q[1], Z0, Z0 + FH - 0.3, 0.3, seg=10, top=False)
            self.class_out(f, a, b, L, top, 4, zb=Z0 + FH - 0.3)
            for c0, c1 in ((u0, a), (b, u1)):
                if c1 - c0 >= 0.5:
                    self.class_out(f, c0, c1, L, top, 4)
                elif c1 - c0 > 0.01:
                    f.face("ws_white", c0, c1, Z0, top, 0.0)
        else:
            self.class_out(f, u0, u1, L, top, 4)
        zy0 = Z0 + FH + 1.0
        f.block("ws_yellow", u0, u1, 0.0, 0.08, zy0 - 0.9, zy0 - 0.2, ends=False)

    def ne_long(self, f, u0, u1, L, top, side):
        """K's long faces (SV05): open ground floor on royal-blue round columns,
        four storeys above with yellow spandrel panels on alternate bays and
        blue floor bands. / K の長辺: 青い丸柱の吹放し、上 4 層は黄色い腰壁と青い帯。"""
        zp = Z0 + 5.4
        g = B.Facade(f.m, f.a - f.n * 3.0, f.a - f.n * 3.0 + f.t * L, f.n)
        a, b = u0 + 0.3, u1 - 0.3
        if b - a > 1.0:
            tiled_face(g, "ws_corr", a, b, Z0, zp - 0.4, 0.0, 3.6, zp - 0.4 - Z0, v0=Z0)
            f.hplane("ws_soffit", a, b, -3.0, 0.0, zp - 0.4, False)
            f.hplane("concrete", a, b, -3.0, 0.0, Z0 + 0.01, True)
            f.side("ws_white", a, -3.0, 0.0, Z0, zp - 0.4, 1)
            f.side("ws_white", b, -3.0, 0.0, Z0, zp - 0.4, -1)
            f.face("ws_white", u0, a, Z0, zp, 0.0)
            f.face("ws_white", b, u1, Z0, zp, 0.0)
            f.face("ws_blue", a, b, zp - 0.4, zp, 0.0)
            for c in np.arange(a + 1.5, b - 0.5, 6.6):
                q = f.a + f.t * c - f.n * 0.5
                f.m.cylinder("ws_blue", q[0], q[1], Z0, zp - 0.4, 0.38, seg=12, top=False)
        cs = [c for c in self.bay_centres(L, FH, 0.6) if u0 + 0.2 <= c - 1.3 and c + 1.3 <= u1 - 0.2]
        for s in range(4):
            z0 = zp + s * FH
            z1 = z0 + FH if s < 3 else top
            opens = [(c - 1.3, c + 1.3, z0 + 1.0, z0 + 2.5) for c in cs]
            wall_openings(f, "ws_white", u0, u1, z0, z1, opens, 2.0)
            for o in opens:
                opening(f, *o, "ws_white", "ws_glass", 0.15, frame=None)
            for j, c in enumerate(cs):
                if (j + s) % 2 == 0:
                    f.block("ws_yellow", c - 1.75, c + 1.75, 0.0, 0.06, z0 + 0.1, z0 + 0.95, ends=True)
            f.block("ws_blue", u0, u1, 0.0, 0.08, z0 - 0.05, z0 + 0.2, ends=False)
        # the yellow stair tower at the NW end (SV05 looks SW at the NE face: its
        # right is NW) / 北西端の黄色い階段室（SV05 は南西向き: 右が北西）
        nw = L if float(f.t @ self.axes()[0]) < 0 else 0.0
        y0, y1 = (L - 4.0, L - 0.2) if nw > 0 else (0.2, 4.0)
        y0, y1 = max(u0, y0), min(u1, y1)
        if y1 > y0 + 0.5:
            f.block("ws_yellow", y0, y1, 0.0, 0.12, zp, top, bottom=True)

    def ne_end(self, f, u0, u1, L, top):
        """K's ends (SV03): white, a yellow band, a blue vertical strip, a
        narrow window column. / K の妻: 白、黄色い帯、青い縦帯。"""
        c = L / 2
        opens = [(c - 0.6, c + 0.6, Z0 + 5.4 + s * FH + 0.9, Z0 + 5.4 + s * FH + 2.6) for s in range(4)
                 if u0 + 0.2 <= c - 0.6 and c + 0.6 <= u1 - 0.2]
        wall_openings(f, "ws_white", u0, u1, Z0, top, opens, 2.0)
        for o in opens:
            opening(f, *o, "ws_white", "ws_glass", 0.15, frame=None)
        f.block("ws_yellow", u0, u1, 0.0, 0.07, top - 4.4, top - 3.3, ends=False)
        p0, p1 = max(u0, L * 0.15), min(u1, L * 0.27)
        if p1 > p0 + 0.3:
            f.block("ws_blue", p0, p1, 0.0, 0.08, Z0 + 5.4, top - 0.3, bottom=True)
        f.block("ws_blue", u0, u1, 0.0, 0.08, Z0 + 5.0, Z0 + 5.4, ends=False)

    def annex_wall(self, f, u0, u1, L, top):
        """The service annex: white with louvre bands per storey. / 付属棟: 各階のガラリ帯。"""
        f.face("ws_white", u0, u1, Z0, top, 0.0, 2.0)
        for s in range(3):
            z = Z0 + s * FH + 1.2
            if u1 - u0 > 1.6:
                tiled_face(f, "ws_louvre", u0 + 0.6, u1 - 0.6, z, z + 1.2, 0.04, 1.5, 1.2, v0=z)
        f.block("ws_blue", u0, u1, 0.0, 0.06, top - 0.5, top, ends=False, top=False)

    def gate_wall(self, f, u0, u1, L, top):
        """The gatehouse (SV06): white walls, blue grille windows, a door.
        守衛所: 白い壁、青い格子窓、扉。"""
        cs = [c for c in self.bay_centres(L, 3.2, 0.6) if u0 + 0.2 <= c - 0.8 and c + 0.8 <= u1 - 0.2]
        opens = [(c - 0.8, c + 0.8, Z0 + 1.0, Z0 + 2.2) for c in cs]
        door = []
        if L < 7 and u1 - u0 > 2.4:
            door = [(u0 + (u1 - u0) / 2 - 0.5, u0 + (u1 - u0) / 2 + 0.5, Z0, Z0 + 2.2)]
            opens = []
        wall_openings(f, "ws_white", u0, u1, Z0, top, opens + door, 2.0)
        for o in opens:
            opening(f, *o, "ws_white", "ws_grille", 0.1, frame=None)
        for o in door:
            opening(f, *o, "ws_white", "ws_door", 0.08, sill=False, frame=None)
        f.block("ws_blue", u0, u1, 0.0, 0.04, Z0, Z0 + 0.6, ends=False, bottom=False)

    def roof_plant(self, m, poly, z, n):
        """Roof plant boxes. / 屋上設備。"""
        c = poly.centroid
        a = poly.buffer(-1.5)
        for i in range(n):
            p = Point(c.x + (i - n / 2) * 3.5, c.y + ((i % 2) - 0.5) * 2.5)
            if a.contains(p):
                m.box("tank", p.x, p.y, z, z + 1.5, 2.2, 1.6)

    # -- drum, canopy / 円筒、上屋
    def drum(self, m):
        """The W stair drum (tooltip 16 m): white, a tall glazed slot facing
        away from the wing, blue and yellow rings at the top. / 西の円筒階段塔。"""
        cx, cy = self.drum_c
        r, top = self.drum_r, self.drum_top
        wing = self.strips["wing"]["poly"] if "wing" in self.strips else None
        drum_walls(m, "ws_white", (cx, cy), r, Z0, top - 1.6, skip=wing)
        drum_walls(m, "ws_yellow", (cx, cy), r + 0.08, top - 1.6, top - 0.9, skip=wing)
        drum_walls(m, "ws_blue", (cx, cy), r + 0.08, top - 0.9, top, skip=wing)
        m.flat("roof_concrete", Point(cx, cy).buffer(r, 24), top + 0.01, 3.0)
        # glazed slot on the side away from the wing / 塔の外側の縦長ガラス
        away = 0.0
        if wing is not None:
            q = wing.exterior.interpolate(wing.exterior.project(Point(cx, cy)))
            away = math.atan2(cy - q.y, cx - q.x)
            if wing.contains(Point(cx, cy)):
                away += math.pi               # the centre stands inside the wing's outline / 中心が棟の内側
        for da in (-0.5, 0.0, 0.5):
            ang = away + da
            nrm = np.array([math.cos(ang), math.sin(ang)])
            t = np.array([-nrm[1], nrm[0]])
            c = np.array([cx, cy]) + nrm * (r + 0.03)
            a, b = c - t * 0.45, c + t * 0.45
            m.quad("ws_glass", (*a, Z0 + 1.0), (*b, Z0 + 1.0), (*b, top - 2.2), (*a, top - 2.2), (0, 0), (1, 0),
                   (1, 4), (0, 4))

    def canopy_roof(self, m):
        """The curved glass entrance canopy (OneMap 8 m): a glazed roof falling
        outward on steel ribs, a fascia, round steel columns on the arc.
        曲面ガラスの玄関上屋（鉄骨のリブ、鼻隠し、円弧上の柱）。"""
        p = self.canopy
        zi, zo = self.canopy_z
        (ox, oy), _ = self.frame
        eu = self.axes()[0]
        us = [self.uv(*q)[0] for q in p.exterior.coords]
        u_in, u_out = min(us), max(us)
        zf = lambda x, y: zi + (zo - zi) * ((x - ox) * eu[0] + (y - oy) * eu[1] - u_in) / max(u_out - u_in, 1e-3)
        m.flat("ws_canopy_glass", p, 0.0, 2.5, zf=zf)
        m.flat("ws_steel", p, 0.0, 2.5, up=False, zf=lambda x, y: zf(x, y) - 0.12)
        ring = orient(p, 1.0).exterior
        c = list(ring.coords)
        ctr = (p.centroid.x, p.centroid.y)
        for a, b in zip(c, c[1:]):
            za, zb = zf(*a), zf(*b)
            vquad_out(m, "ws_white", a, b, za - 0.45, za, zb - 0.45, zb, ctr)
        # columns on the outer arc, every ~5 m / 外周の柱
        arc = ring.difference(self.footprint.buffer(0.3))
        for g in getattr(arc, "geoms", [arc]):
            if g.geom_type != "LineString":
                continue
            for s in np.arange(1.0, g.length - 0.5, 5.0):
                q = g.interpolate(s)
                d = Point(q).buffer(0.6)
                if not p.buffer(-0.2).intersects(d):
                    continue
                inner = p.buffer(-0.5)
                qq = inner.exterior.interpolate(inner.exterior.project(q)) if not inner.is_empty else q
                m.cylinder("ws_steel", qq.x, qq.y, Z0, zf(qq.x, qq.y) - 0.45, 0.16, seg=8, top=False)

    # -- grounds / 敷地
    def fences(self, m):
        """Blue low wall + grey railings on St 11 and the estate road, red wall
        with white coping + green mesh on Dr 1 and the NE corner, the tall ball
        fence on the field, the sliding gates. / 塀と柵、球技場の高い柵、門扉。"""
        cut = unary_union([self.no_go.buffer(1.0), self.footprint.buffer(0.4),
                           LineString([self.X(q) for q in self.T["gate"]]).buffer(1.2),
                           Point(self.X(self.T["ped_gate"])).buffer(1.6),
                           LineString([self.X(q) for q in self.T["sign"]]).buffer(0.5)])
        if self.canopy is not None:
            cut = cut.union(self.canopy.buffer(0.3))
        for name, line in self.T["fences"].items():
            ln = LineString([self.X(q) for q in line])
            ln = ln.difference(cut)
            red = "red" in name
            for g in getattr(ln, "geoms", [ln]):
                if g.geom_type != "LineString" or g.length < 1.0:
                    continue
                c = list(g.coords)
                for a, b in zip(c, c[1:]):
                    seg = LineString([a, b])
                    m.prism("ws_red" if red else "ws_blue", seg.buffer(0.12, cap_style="flat"), Z0 - 0.05, Z0 + 0.6,
                            top=False)
                    m.prism("ws_white", seg.buffer(0.16, cap_style="flat"), Z0 + 0.6, Z0 + 0.7, top=True)
                    if red:
                        chainlink(m, "ws_fence_green", a, b, Z0 + 0.7, Z0 + 1.9)
                        top_c = "ws_fence_green"
                    else:
                        bars(m, "ws_rail", a, b, Z0 + 0.7, Z0 + 1.8)
                        top_c = "ws_rail"
                    ang = math.atan2(b[1] - a[1], b[0] - a[0])
                    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                    m.box(top_c, mid[0], mid[1], Z0 + 1.8 if not red else Z0 + 1.85, Z0 + 1.9,
                          math.dist(a, b), 0.05, ang)
                for k in range(int(g.length // 2.5) + 1):
                    q = g.interpolate(min(g.length, k * 2.5))
                    m.box("ws_post" if red else "ws_rail", q.x, q.y, Z0 + 0.6, Z0 + 1.95, 0.07, 0.07)
        # the ball fence along the field's NE side (SV04): 6 m green chain-link,
        # posts every 3 m with diagonal stays / 運動場北東の高い球技柵
        bl = LineString([self.X(q) for q in self.T["ball_fence"]]).difference(self.no_go.buffer(1.0))
        H = 6.0
        for g in getattr(bl, "geoms", [bl]):
            if g.geom_type != "LineString" or g.length < 2.0:
                continue
            c = list(g.coords)
            for a, b in zip(c, c[1:]):
                chainlink(m, "ws_fence_green", a, b, Z0 + 0.1, Z0 + H, pitch=0.3)
            inward = None
            for k in range(int(g.length // 3.0) + 1):
                q = g.interpolate(min(g.length, k * 3.0))
                m.box("ws_post", q.x, q.y, Z0, Z0 + H + 0.1, 0.09, 0.09)
                if k % 2 == 0:
                    q2 = g.interpolate(min(g.length, k * 3.0 + 0.1))
                    tx, ty = q2.x - q.x, q2.y - q.y
                    ln = math.hypot(tx, ty) or 1.0
                    inward = np.array([-ty, tx]) / ln
                    fc = np.array(self.field.centroid.coords[0])
                    if float((fc - np.array([q.x, q.y])) @ inward) < 0:
                        inward = -inward
                    # the stays stand on the Dr 1 side, away from the field (SV04)
                    # 支柱の控えは Dr 1 側（運動場の外）
                    foot = np.array([q.x, q.y]) - inward * 2.2
                    if self.footprint.contains(Point(*foot)) or self.no_go.distance(Point(*foot)) < 1.0:
                        continue
                    beam(m, "ws_post", (q.x, q.y, Z0 + 3.5), (foot[0], foot[1], Z0), r=0.035)
        # the sliding gates at the drive mouth, parked open along the fence
        # 車路入口の引き戸（開いて塀沿い）
        a, b = (np.array(self.X(q)) for q in self.T["gate"])
        t = (b - a) / max(np.linalg.norm(b - a), 1e-6)
        inward = np.array([-t[1], t[0]])
        cen = np.array(self.lot.centroid.coords[0])
        if float((cen - (a + b) / 2) @ inward) < 0:
            inward = -inward
        span = LineString([tuple(a), tuple(b)]).difference(self.no_go.buffer(0.6))
        ends = []
        for g in getattr(span, "geoms", [span]):
            if g.geom_type == "LineString":
                ends += [np.array(g.coords[0]), np.array(g.coords[-1])]
        if not ends:
            ends = [a, b]
        lo = min(ends, key=lambda q: float((q - a) @ t))
        hi = max(ends, key=lambda q: float((q - a) @ t))
        for p, s in ((lo, -1), (hi, 1)):
            for q0 in (p + t * s * 0.5 + inward * 0.6,):
                if self.no_go.buffer(0.3).contains(Point(*q0)):
                    continue
                m.box("ws_blue", p[0], p[1], Z0, Z0 + 2.0, 0.5, 0.5)
                ang = math.atan2(t[1], t[0])
                leaf0, leaf1 = q0, q0 + t * s * 4.5
                bars(m, "ws_rail", leaf0, leaf1, Z0 + 0.1, Z0 + 1.8, pitch=0.15)
                for zz in (Z0 + 0.1, Z0 + 1.0, Z0 + 1.8):
                    mid = (leaf0 + leaf1) / 2
                    m.box("ws_rail", mid[0], mid[1], zz, zz + 0.06, 4.5, 0.05, ang)

    def walkways(self, m):
        """Covered walkways: the grey steel one along the drive, the blue-and-white
        striped awning by the hard court, the dark green one before the hall
        gable. / 屋根付き通路（灰色の鋼製、青白の縞の天幕、緑の通路）。"""
        busy = unary_union([self.footprint.buffer(0.3), self.no_go.buffer(0.6)])
        for w in self.T["walkways"]:
            line = LineString([self.X(q) for q in w["line"]])
            roof = line.buffer(w["w"] / 2, cap_style="flat").difference(busy)
            z = Z0 + w["z"]
            for p in polys(roof):
                if p.area < 3.0:
                    continue
                if w.get("vault"):
                    # a striped barrel awning / 縞の半円筒天幕
                    sp = np.array(line.coords[0]), np.array(line.coords[-1])
                    t = (sp[1] - sp[0]) / np.linalg.norm(sp[1] - sp[0])
                    us = [float((np.array(q) - sp[0]) @ t) for q in p.exterior.coords]
                    a0, a1 = sp[0] + t * min(us), sp[0] + t * max(us)
                    S = Strip([a0, a1], w["w"] / 2, w["w"] / 2)
                    vault(m, S, (z, z + 0.9, z, 0.5), w["mat"], ov=0.0, thick=0.05, fascia="ws_rail",
                          soffit="ws_awning", uvu=1.0)
                    nl = S.nl[0]
                    for s_ in np.arange(0.4, float(np.linalg.norm(a1 - a0)), 3.0):
                        for sd in (-1, 1):
                            q = a0 + t * s_ + nl * sd * (w["w"] / 2 - 0.1)
                            m.box("ws_rail", q[0], q[1], Z0, z, 0.08, 0.08)
                    continue
                m.flat(w["mat"], p, z, 2.0)
                m.flat("ws_soffit", p, z - 0.1, 2.0, up=False)
                m.ring_walls("ws_steel", p.exterior, z - 0.1, z)
                for s_ in np.arange(0.5, line.length, 3.5):
                    q = line.interpolate(s_)
                    if p.buffer(-0.2).contains(q):
                        d = line.interpolate(min(line.length, s_ + 0.1))
                        tx, ty = d.x - q.x, d.y - q.y
                        ln = math.hypot(tx, ty) or 1.0
                        for sd in (-1, 1):
                            px, py = q.x - ty / ln * sd * (w["w"] / 2 - 0.15), q.y + tx / ln * sd * (w["w"] / 2 - 0.15)
                            m.box("ws_steel", px, py, Z0, z - 0.1, 0.08, 0.08)
        # the stepped terrace along the field (SV04): grey steps up to the hall
        # 運動場沿いの段状の観覧席
        if not self.terrace.is_empty:
            us = [self.uv(*q)[0] for q in self.terrace.exterior.coords]
            for k in range(3):
                u_hi = max(us) - k * 0.95
                cutp = Polygon([self.at(-500, -500), self.at(u_hi, -500), self.at(u_hi, 500), self.at(-500, 500)])
                g = self.terrace.intersection(cutp)
                if g.area > 1.0:
                    m.prism("ws_steps", g, Z0, Z0 + 0.45 * (k + 1), top=True)

    def field_kit(self, m):
        """Goals at both ends of the field (white posts, dark net). / ゴール。"""
        us, vs = zip(*[self.uv(*q) for q in self.field.exterior.coords])
        uc = (min(us) + max(us)) / 2
        eu, ev = self.axes()
        ang = math.atan2(eu[1], eu[0])
        for v in (min(vs) + 1.2, max(vs) - 1.2):
            x, y = self.at(uc - 1.5, v)
            for s in (-1, 1):
                q = self.at(uc - 1.5 + s * 2.5, v)
                m.cylinder("goal_white", q[0], q[1], Z0, Z0 + 2.0, 0.06, seg=6)
            m.box("goal_white", x, y, Z0 + 1.94, Z0 + 2.06, 5.0, 0.1, ang)

    def name_wall(self, m):
        """The name board wall at the St 11 corner (Yelp photo): blue base, a
        white panel with the school's names in blue and the crest (drawn).
        St 11 角の校名壁（青い腰壁、白い板に校名と校章: 描画）。"""
        a, b = (np.array(self.X(q)) for q in self.T["sign"])
        t = (b - a) / np.linalg.norm(b - a)
        n = np.array([t[1], -t[0]])
        if self.facing(n) != "E":
            a, b = b, a
            t, n = -t, -n
        L = float(np.linalg.norm(b - a))
        seg = LineString([tuple(a), tuple(b)])
        if seg.buffer(0.4).intersects(self.no_go):
            return
        m.prism("ws_blue", seg.buffer(0.25, cap_style="flat"), Z0, Z0 + 0.9, top=True)
        m.prism("ws_white", seg.buffer(0.15, cap_style="flat"), Z0 + 0.9, Z0 + 2.5, top=True)
        f = B.Facade(m, a, b, n)
        f.m.quad("ws_plaque", f.P(0.15, 0.17, Z0 + 0.98), f.P(L - 0.15, 0.17, Z0 + 0.98),
                 f.P(L - 0.15, 0.17, Z0 + 2.42), f.P(0.15, 0.17, Z0 + 2.42))

    def trees(self, m):
        """Rain trees along St 11 and Dr 1 inside the walls, trees in the light
        wells, by the gate. / 樹木（St 11・Dr 1 沿い、光庭、門脇）。"""
        import props
        T = self.T["trees"]
        spots = [(self.X(q), "rain", 0.85) for q in T["st11"]] + [(self.X(q), "angsana", 0.95) for q in T["dr1"]]
        spots += [(self.X(q), "rain", 0.6) for q in T["wells"]] + [(self.X(q), "palm", 0.8) for q in T["gate"]]
        busy = unary_union([self.footprint.buffer(1.5), self.field.buffer(0.5)]
                           + [c.buffer(1.0) for c, _ in self.courts.values()])
        for (x, y), kind, s in spots:
            p = Point(x, y)
            R = {"rain": 6.0, "angsana": 4.2, "palm": 3.2}[kind] * s
            inwell = any(w.buffer(-1.0).contains(p) for w in self.wells.values())
            if (busy.contains(p) and not inwell) or self.no_go.distance(p) < R + 0.6:
                continue
            props.Props.tree_mesh(None, m, x, y, kind, s, False)


def write_textures(tex_dir):
    """The school's drawn textures (procedural; colours from description.md,
    nothing taken from imagery): renders, glass, the corridor wall, roof
    metal, louvres, panels, courts, turf, pavers, awning stripes, the crest
    stand-in and the name board. / 学校の描画テクスチャ（description.md の色）。"""
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

    WHT, BLU, YEL = (205, 206, 208), (40, 95, 160), (200, 170, 80)
    save("ws_white", X.paint(512, WHT, 0.07), 0.85, 0.0, 1.0)
    save("ws_blue", X.paint(256, BLU, 0.05), 0.8, 0.0, 0.6)
    save("ws_yellow", X.paint(256, YEL, 0.05), 0.8, 0.0, 0.6)
    save("ws_red", X.paint(256, (123, 48, 48), 0.06), 0.85, 0.0, 0.6)
    save("ws_soffit", X.paint(128, (196, 196, 192), 0.04), 0.9)
    save("ws_fascia", X.paint(64, (150, 155, 158), 0.02), 0.5, 0.4)
    save("ws_eave_dark", X.paint(64, (70, 74, 78), 0.03), 0.5, 0.4)
    save("ws_roof", X.stripes(256, (140, 145, 149), (160, 165, 168), 18, vertical=True), 0.45, 0.6, 1.2)
    save("ws_gate_roof", X.stripes(256, (85, 115, 100), (100, 132, 116), 18, vertical=True), 0.5, 0.5, 1.2)
    save("ws_louvre", X.stripes(256, (85, 95, 96), (130, 140, 140), 14), 0.6, 0.3, 1.5)
    save("ws_rail", X.paint(64, (150, 155, 158), 0.02), 0.4, 0.6)
    save("ws_post", X.paint(64, (40, 70, 52), 0.02), 0.5, 0.5)
    save("ws_fence_green", X.paint(64, (45, 85, 60), 0.04), 0.6, 0.3)
    save("ws_steel", X.paint(64, (120, 125, 128), 0.02), 0.45, 0.6)
    save("ws_walk_grey", X.stripes(128, (138, 142, 144), (156, 160, 162), 8, vertical=True), 0.5, 0.6)
    save("ws_walk_green", X.stripes(128, (35, 70, 50), (48, 88, 64), 8, vertical=True), 0.55, 0.4)
    save("ws_steps", X.concrete(256, (150, 152, 150), (170, 172, 168)), 0.85)
    save("ws_pavers", X.bricks(512, (150, 80, 65)), 0.85, 0.0, 1.2)
    for nm, c in (("ws_pan_r", (157, 60, 62)), ("ws_pan_g", (60, 150, 115)), ("ws_pan_b", (30, 100, 160)),
                  ("ws_pan_y", (210, 175, 70)), ("ws_pan_t", (45, 150, 150))):
        save(nm, X.paint(64, c, 0.03), 0.6)

    def framed(frame, glass0, glass1, panes=2, n=256, transom=0.33):
        y = np.linspace(0, 1, n)[:, None, None]
        g = np.array(glass1)[None, None, :] * (1 - y) + np.array(glass0)[None, None, :] * y
        im = Image.fromarray(np.repeat(g, n, axis=1).astype(np.uint8))
        d = ImageDraw.Draw(im)
        fw = max(4, n // 26)
        d.rectangle([0, 0, n - 1, n - 1], outline=frame, width=fw)
        for p in range(1, panes):
            x = int(n * p / panes)
            d.rectangle([x - fw // 2, 0, x + fw // 2, n], fill=frame)
        if transom:
            yt = int(n * transom)
            d.rectangle([0, yt - fw // 2, n, yt + fw // 2], fill=frame)
        return im

    G0, G1 = (60, 92, 96), (118, 148, 150)
    save("ws_glass", framed((60, 66, 70), G0, G1, 3), 0.1, 0.2)
    save("ws_grille", framed(BLU, (50, 60, 66), (90, 105, 110), 6, transom=0), 0.3, 0.3)
    save("ws_door", framed((70, 72, 74), (120, 125, 128), (140, 145, 148), 2, transom=0.7), 0.5)
    # the front's glazing behind the blue grid: one storey x 3.6 m bay / 正面のガラス（1 層 x 1 ベイ）
    n = 256
    im = framed((205, 206, 208), (55, 85, 95), (110, 140, 145), 2, n, transom=0.0)
    d = ImageDraw.Draw(im)
    d.rectangle([0, int(n * 0.72), n, n], fill=WHT)
    d.rectangle([0, 0, n, int(n * 0.08)], fill=WHT)
    save("ws_glass_grid", im, 0.15, 0.2)
    # curved canopy glazing: green-tinted panes on a steel grid / 上屋のガラス
    im = Image.new("RGB", (n, n), (90, 130, 125))
    d = ImageDraw.Draw(im)
    for k in range(0, n, n // 4):
        d.line([(k, 0), (k, n)], fill=(110, 115, 118), width=6)
        d.line([(0, k), (n, k)], fill=(110, 115, 118), width=4)
    save("ws_canopy_glass", im, 0.1, 0.3)
    # the corridor wall: cream, a door and a window per 3.6 m module / 廊下壁
    W_, H_ = 512, 420
    im = X.paint(512, (225, 215, 185), 0.05).resize((W_, H_))
    d = ImageDraw.Draw(im)
    d.rectangle([30, 110, 150, H_ - 1], fill=(70, 95, 140))
    d.rectangle([40, 120, 140, 230], fill=(80, 110, 115))
    d.rectangle([200, 140, 490, 320], fill=(55, 60, 64))
    d.rectangle([208, 148, 482, 312], fill=(105, 135, 135))
    d.line([(345, 140), (345, 320)], fill=(55, 60, 64), width=6)
    d.rectangle([0, H_ - 24, W_, H_], fill=(185, 180, 165))
    save("ws_corr", im, 0.8)

    # courts / コート
    def court_n():
        # 33 x 31.5 m: blue-grey surround, a teal netball court 30 x 18 m with
        # orange goal circles / 青灰色の地に緑青のネットボールコートと橙の円
        k = 24
        Wc, Hc = int(33 * k), int(31.5 * k)
        im = Image.fromarray(X.tint(X.noise(max(Wc, Hc))[:Hc, :Wc], np.array((105, 115, 128)),
                                    np.array((120, 130, 142))))
        d = ImageDraw.Draw(im)
        P = lambda x, y: (x * k, Hc - y * k)
        x0, x1, y0, y1 = 2.5, 32.5, 9.0, 27.0
        d.rectangle([P(x0, y1), P(x1, y0)], fill=(60, 125, 120))
        for cx, a0, a1 in ((x0, -90, 90), (x1, 90, 270)):
            d.pieslice([P(cx - 4.9, y0 + 9 + 4.9), P(cx + 4.9, y0 + 9 - 4.9)], a0, a1, fill=(205, 110, 80))
        lc = (240, 240, 236)
        d.rectangle([P(x0, y1), P(x1, y0)], outline=lc, width=4)
        for x in (x0 + 10, x0 + 20):
            d.line([P(x, y0), P(x, y1)], fill=lc, width=4)
        d.ellipse([P(17.5 - 0.5, 18.5), P(17.5 + 0.5, 17.5)], fill=lc)
        return im
    save("ws_court_n", court_n(), 0.8, 0.0, 0.5)

    def court_o():
        # 24 x 24 m beige hard court, a white ring of four arcs and a centre disc
        # ベージュのコート、白い四分円の輪と中心の円
        k = 24
        Wc = 24 * k
        im = Image.fromarray(X.tint(X.noise(Wc), np.array((150, 140, 132)), np.array((168, 158, 150))))
        d = ImageDraw.Draw(im)
        c = (Wc * 0.38, Wc * 0.5)
        r = 3.2 * k
        for a in (20, 110, 200, 290):
            d.arc([c[0] - r, c[1] - r, c[0] + r, c[1] + r], a, a + 50, fill=(240, 240, 236), width=10)
        d.ellipse([c[0] - 0.8 * k, c[1] - 0.8 * k, c[0] + 0.8 * k, c[1] + 0.8 * k], fill=(240, 240, 236))
        return im
    save("ws_court_o", court_o(), 0.85, 0.0, 0.5)

    def turf():
        # artificial turf 56 x 51.5 m (dark teal green, tiles 2026) with a pitch
        # along v: touchlines, halfway line, centre circle, boxes / 人工芝とライン
        k = 16
        Wc, Hc = int(56 * k), int(51.5 * k)
        base = X.tint(X.noise(max(Wc, Hc))[:Hc, :Wc], np.array((38, 82, 56)), np.array((52, 100, 68)))
        im = Image.fromarray(base)
        d = ImageDraw.Draw(im)
        P = lambda x, y: (x * k, Hc - y * k)
        lc, lw = (235, 238, 232), 4
        x0, x1, y0, y1 = 4.5, 51.5, 1.5, 50.0
        d.rectangle([P(x0, y1), P(x1, y0)], outline=lc, width=lw)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        d.line([P(x0, cy), P(x1, cy)], fill=lc, width=lw)
        d.ellipse([P(cx - 5, cy + 5), P(cx + 5, cy - 5)], outline=lc, width=lw)
        for yb, sg in ((y0, 1), (y1, -1)):
            d.rectangle([P(cx - 10, max(yb, yb + sg * 9)), P(cx + 10, min(yb, yb + sg * 9))], outline=lc, width=lw)
            d.rectangle([P(cx - 4, max(yb, yb + sg * 3)), P(cx + 4, min(yb, yb + sg * 3))], outline=lc, width=lw)
        return im
    save("ws_turf", turf(), 0.9, 0.0, 0.4)
    # the awning: blue and white stripes across / 青白の縞の天幕
    im = Image.new("RGB", (256, 64), (240, 240, 238))
    d = ImageDraw.Draw(im)
    for x in range(0, 256, 64):
        d.rectangle([x, 0, x + 31, 64], fill=(45, 100, 170))
    save("ws_awning", im, 0.7)
    # the crest stand-in: a round badge in the school colours (not the real crest)
    # 校章の代用（学校の色の円形バッジ。実物ではない）
    n = 512
    im = Image.new("RGB", (n, n), WHT)
    d = ImageDraw.Draw(im)
    d.ellipse([8, 8, n - 8, n - 8], fill=(240, 240, 238), outline=BLU, width=34)
    d.ellipse([90, 90, n - 90, n - 90], fill=(60, 140, 110))
    for k2 in range(3):
        y = n * (0.5 + k2 * 0.08)
        d.arc([n * .25, y - n * .06, n * .75, y + n * .06], 0, 180, fill=(240, 240, 238), width=n // 40)
    d.ellipse([n * .42, n * .26, n * .58, n * .42], fill=YEL)
    save("ws_crest", im, 0.6)
    # the name board: the school's names (school site, OSM name) drawn, crest right
    # 校名板（学校の名称を描く、右に校章）
    im = Image.new("RGB", (1024, 320), (246, 246, 242))
    d = ImageDraw.Draw(im)
    fb = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    d.text((30, 22), "SEKOLAH RENDAH WHITE SANDS", fill=BLU, font=ImageFont.truetype(fb, 46))
    cjk = "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc"
    if os.path.exists(cjk):
        d.text((30, 88), "白沙小学", fill=BLU, font=ImageFont.truetype(cjk, 58))
    d.text((30, 228), "WHITE SANDS PRIMARY SCHOOL", fill=BLU, font=ImageFont.truetype(fb, 50))
    crest = Image.open(os.path.join(tex_dir, "ws_crest.png")).resize((190, 190))
    im.paste(crest, (812, 60))
    save("ws_plaque", im, 0.6)
    return out
