"""Coral Primary School (20 Pasir Ris Street 51), rebuilt as it stood in
2016-2018 (maps/.venv). The school merged into White Sands Primary in 2019,
was repainted white by 2025 and demolished in 2025-26: OSM and today's
imagery show a construction site, which this module replaces.

Sources (build/PasirRisTampines/coral_photos, visual reference only, never
textures): OneMap 2-D footprints (registered on the OSM HDB blocks), the
OneMap 3-D height tooltip and 8 obliques (shapes, storeys, roofs), Google
Earth historical 2018-06-11 / 2018-12-08 and Esri 2018-01-29 (roofs, courts,
ground), Street View Sep 2016 / Feb 2018 / Apr 2019 (facades, colours), the
MOE Heritage Centre court photo (curved block, lettering). Colours from the
measured pixel medians in description.md (yellow 200,185,115; blue 25,90,160;
off-white 215,216,212). Trace: build/<Name>/coral_trace.json (map frame,
made by measuring in two rotated frames, see its "sources").

  north, middle  4-storey classroom blocks, grey metal gables; the north
                 block's street face: white ground floor with green-framed
                 windows, three blue storeys with deep projecting bays and
                 yellow-framed window pairs, a yellow stair core (east) and a
                 glazed stair tower (west); open corridors on the courtyards
                 (white slabs, blue piers; blue and yellow parapets on the
                 middle block's south face), yellow corridor walls, orange soffits
                 (the yellow core is at the east end and the glazed tower at the west
                 end in Street View, the other way round from description.md)
  spine          4 storeys + yellow parapet band along the field: open
                 corridors, eight blue fins, white stair towers, glazed ends
  curved block   2 storeys of yellow spandrel bands with thin blue lines and
                 ribbon windows following the curve, the lettering upturn
                 "CORAL PRIMARY SCHOOL" with a white railing, the blue third
                 storey set back, a gable over the bar behind
  west wing      3 storeys, white, punched windows, a blue band, flat roof
  blue wings     3-storey blue wings by the hall and the field
  hall           off-white, two rows of high windows with blue sill and
                 lintel bands, the entrance side (blue columns, canopy, white
                 band with square windows, blue fascia, dark recessed top),
                 the SE gable's tall green glazing, louvre bands, the blue NE
                 corner with the yellow crest band, light grey gable roof
  entrance       yellow porch frame with the school's name, the yellow drum
                 stair tower with its blue cap ring
  grounds        red basketball court with blue keys, the grey parade square,
                 the painted strip along the spine, the field with goals and
                 bare goal mouths, red-brick drop-off apron and staff bays, the
                 store with its arched white gates, the white sliding gates
                 and yellow guard house, dark green chain-link fence on a kerb
                 with a clipped hedge, the tall ball-court fence, the covered
                 walkway along St 51, flagpole, rain trees, palms

Roads win: footprints keep GAP off the road network (the drop-off loop is an
OSM service road), and any triangle left touching it is dropped.
Coordinates: the map frame (x east, y north, metres).

珊瑚小学（Coral Primary School、2016-2018 年の姿）の詳細モデル。2019 年に統合、
2025 年に白く塗り替え、2025-26 年に解体（OSM と現在の航空写真は工事現場）。
写真は参照のみ（テクスチャに使わない）。色は description.md の実測値。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.affinity import translate
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from landmarks.prcs import text_poly
from mesh import Mesh, earcut, polys

NAME = "Coral Primary School"
TAG = "CORAL"
TRACE = "coral_trace.json"
MAP = "PasirRisTampines"
SITE_NAMES = (NAME,)
Z0 = 0.15                 # the raised ground / 敷地面
F4 = 3.5                  # classroom storey / 教室棟の階高
F3 = 3.6                  # curved block, wings / 曲面棟・翼棟の階高
GAP = 1.0                 # footprints this far off the road network / 道路網からの離れ
HALL_EAVE, HALL_RIDGE = 12.0, 17.5
D_CORR = 2.2              # open corridor depth / 開放廊下の奥行き
# window frames proud of the glass, by glass (yellow on the blue storeys, dark
# green on the white ground floor, dark elsewhere) / ガラスごとの窓枠の色
FRAMES = {"coral_glass_yf": "coral_frame_y", "coral_glass_gf": "coral_frame_g",
          "coral_glass_dk": "coral_frame_dk"}


def enabled(out, meta):
    """Only on PasirRisTampines, with its trace. / この地図とトレースがある時だけ。"""
    return meta.get("name") == MAP and os.path.exists(os.path.join(out, TRACE))


# -- primitives (tested in test_coral.py: every face points the way it is seen)
# 基本形状（test_coral.py で面の向きを確認）
def wall_openings(f, mat, u0, u1, z0, z1, opens, uvs=1.0):
    """The wall patch [u0, u1] x [z0, z1] facing out, with rectangular holes
    [(a, b, oz0, oz1)] in any arrangement (rows, stacked columns): cut into
    columns at every opening edge, each column drawn between the openings
    it holds. (A per-opening version drew full-height pieces over stacked
    windows: blank walls; test_coral checks area and overlap.)
    開口付きの壁（区間版）: 開口の縁で縦に区切り、列ごとに開口の間だけ描く。"""
    us = sorted({u0, u1} | {min(max(v, u0), u1) for a, b, _, _ in opens for v in (a, b)})
    for x0, x1 in zip(us, us[1:]):
        if x1 - x0 < 1e-4:
            continue
        mid = (x0 + x1) / 2
        z = z0
        for oz0, oz1 in sorted((max(oz0, z0), min(oz1, z1)) for a, b, oz0, oz1 in opens if a <= mid <= b):
            if oz0 > z:
                f.face(mat, x0, x1, z, oz0, 0.0, uvs)
            z = max(z, oz1)
        if z1 > z:
            f.face(mat, x0, x1, z, z1, 0.0, uvs)


def opening(f, a, b, oz0, oz1, wall, glass, depth=0.15, tu=None, sill=True, frame="auto"):
    """Reveals into the opening and the glass set back; tu: the glass texture
    repeats every tu metres (ribbon windows), else one tile per window.
    開口の見込みと奥のガラス（tu: 連窓の繰り返し幅）。"""
    if frame == "auto":
        frame = None if tu is not None else FRAMES.get(glass)
    d = -depth
    f.hplane(wall, a, b, d, 0.0, oz0, True)
    f.hplane(wall, a, b, d, 0.0, oz1, False)
    f.side(wall, a, d, 0.0, oz0, oz1, 1)
    f.side(wall, b, d, 0.0, oz0, oz1, -1)
    uv = ((0, 0), (1, 0), (1, 1), (0, 1)) if tu is None else \
        ((a / tu, 0), (b / tu, 0), (b / tu, 1), (a / tu, 1))
    f.m.quad(glass, f.P(a, d, oz0), f.P(b, d, oz0), f.P(b, d, oz1), f.P(a, d, oz1), *uv)
    if sill:
        f.hplane("concrete", a - 0.04, b + 0.04, 0.0, 0.07, oz0, True)
        f.face("concrete", a - 0.04, b + 0.04, oz0 - 0.05, oz0, 0.07)
    if frame:
        # a frame proud of the glass, inside the reveal: jambs, head, sill rail,
        # the middle mullion / 見込み内の窓枠（縦枠・上枠・下枠・中桟）
        fw, fd = 0.06, -depth + 0.06
        g = B.Facade(f.m, f.a, f.a + f.t * f.L, f.n)
        for x0, x1 in ((a, a + fw), (b - fw, b), ((a + b) / 2 - fw / 2, (a + b) / 2 + fw / 2)):
            g.block(frame, x0, x1, d, fd, oz0, oz1, top=False, bottom=False)
        for zz0, zz1 in ((oz0, oz0 + fw), (oz1 - fw, oz1)):
            g.block(frame, a + fw, b - fw, d, fd, zz0, zz1, ends=False)


def face_in(f, mat, u0, u1, z0, z1, w):
    """A wall patch at offset w facing INTO the building (-n). / 内向きの壁面。"""
    if u1 - u0 < 1e-3 or z1 - z0 < 1e-3:
        return
    f.m.quad(mat, f.P(u1, w, z0), f.P(u0, w, z0), f.P(u0, w, z1), f.P(u1, w, z1),
             (u1, z0), (u0, z0), (u0, z1), (u1, z1))


def tiled_face(f, mat, u0, u1, z0, z1, w, tu, tv, v0=0.0):
    """Outward patch whose texture repeats every tu along and tv up. / 繰り返し模様の面。"""
    if u1 - u0 < 1e-3 or z1 - z0 < 1e-3:
        return
    f.m.quad(mat, f.P(u0, w, z0), f.P(u1, w, z0), f.P(u1, w, z1), f.P(u0, w, z1),
             (u0 / tu, (z0 - v0) / tv), (u1 / tu, (z0 - v0) / tv), (u1 / tu, (z1 - v0) / tv),
             (u0 / tu, (z1 - v0) / tv))


def rect_axes(poly):
    """(a, b, c, d) corners of the oriented rectangle, a->b the long side. / 外接矩形の角。"""
    r = orient(poly.minimum_rotated_rectangle, 1.0)
    c = [np.array(q) for q in list(r.exterior.coords)[:4]]
    e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
    k = 0 if e[0] >= e[1] else 1
    return [c[(k + i) % 4] for i in range(4)]


def gable(m, poly, z0, z1, mat, wall, ov=0.5, fascia="coral_white", ribs=0.6):
    """A gable roof over the oriented rectangle (ridge along the long side):
    slopes up and out, gable triangles out, soffit down, a fascia board.
    外接矩形の切妻屋根（棟は長辺方向）。"""
    a, b, c, d = rect_axes(poly)
    ax = (b - a) / np.linalg.norm(b - a)
    sd = (d - a) / np.linalg.norm(d - a)
    a2, b2, c2, d2 = a - ax * ov - sd * ov, b + ax * ov - sd * ov, c + ax * ov + sd * ov, d - ax * ov + sd * ov
    r1, r2 = (a2 + d2) / 2, (b2 + c2) / 2
    P = lambda q, z: (q[0], q[1], z)
    m.quad(mat, P(a2, z0), P(b2, z0), P(r2, z1), P(r1, z1), (0, 0), (np.linalg.norm(b2 - a2) / 2, 0),
           (np.linalg.norm(b2 - a2) / 2, 3), (0, 3))
    m.quad(mat, P(c2, z0), P(d2, z0), P(r1, z1), P(r2, z1), (0, 0), (np.linalg.norm(b2 - a2) / 2, 0),
           (np.linalg.norm(b2 - a2) / 2, 3), (0, 3))
    if ribs:
        # standing seams on both slopes, eave to ridge / 立平葺きのはぜ（軒から棟へ）
        Lr = float(np.linalg.norm(b2 - a2))
        for e0, e1, r0 in ((a2, b2, r1), (c2, d2, r2)):
            tt = (e1 - e0) / Lr
            down = (e0 - r0) - tt * float((e0 - r0) @ tt)          # ridge -> eave, horizontal / 棟から軒
            for u in np.arange(ribs / 2, Lr, ribs):
                pe = e0 + tt * u
                pr = pe - down
                s_ = np.r_[pe, z0] - np.r_[pr, z1]
                nn = np.cross(np.r_[tt, 0.0], s_)
                nn = nn / np.linalg.norm(nn)
                if nn[2] < 0:
                    nn = -nn
                h, w = nn * 0.035, np.r_[tt, 0.0] * 0.025
                A0, A1 = np.r_[pr, z1], np.r_[pe, z0]
                for sg in (-1, 1):
                    q = [A1 + w * sg, A0 + w * sg, A0 + w * sg + h, A1 + w * sg + h]
                    # wound so each side faces away from the seam's centre line / 外向きに巻く
                    nq = np.cross(q[1] - q[0], q[3] - q[0])
                    if float(nq @ (w * sg)) > 0:
                        m.quad(mat, *q)
                    else:
                        m.quad(mat, q[1], q[0], q[3], q[2])
                m.quad(mat, A1 - w + h, A1 + w + h, A0 + w + h, A0 - w + h)
    m1, m2 = (a + d) / 2, (b + c) / 2
    m.tris(wall, [P(d, z0), P(a, z0), P(m1, z1)], np.r_[-ax, 0], [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
    m.tris(wall, [P(b, z0), P(c, z0), P(m2, z1)], np.r_[ax, 0], [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
    m.quad("concrete", P(b2, z0), P(a2, z0), P(d2, z0), P(c2, z0))
    # fascia boards on the eaves (faces out) and the grey gutter in front / 鼻隠しと軒樋
    for p, q, nrm in ((a2, b2, -sd), (c2, d2, sd)):
        m.quad(fascia, P(p, z0 - 0.25), P(q, z0 - 0.25), P(q, z0 + 0.02), P(p, z0 + 0.02))
        g = B.Facade(m, p, q, nrm)
        g.block("coral_pipe", 0.0, float(np.linalg.norm(q - p)), 0.0, 0.16, z0 - 0.2, z0 - 0.04, ends=True)
    return a, b, c, d


def flat_roof(m, poly, z, wall, parapet=0.9, mat="roof_concrete", cap="coral_white"):
    """Flat roof with a parapet: outer walls out, inner walls in, cap up. / 陸屋根とパラペット。"""
    for p in polys(poly):
        inner = p.buffer(-0.25, join_style="mitre")
        m.prism(wall, p, z, z + parapet, top=False)
        for q in polys(inner):
            m.ring_walls(wall, q.exterior, z, z + parapet, outward=False)
            m.flat(mat, q, z + 0.01, 3.0)
        m.flat(cap, p.difference(inner), z + parapet, 1.0)


def glyphs_on(m, f, g, u, z, w, mat, height):
    """Text polygons g (x along, y up, metres) flat on facade f at (u, z),
    offset w, facing out (+n). / 文字を外壁に貼る（外向き）。"""
    for p in polys(g):
        p = orient(p, 1.0)
        rings = [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]
        V = np.concatenate(rings)
        ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
        I = earcut.triangulate_float64(V.astype(np.float64), ends).reshape(-1, 3)
        if len(I) == 0:
            continue
        P3 = np.array([f.P(u + q[0], w, z + q[1]) for q in V])
        A = P3[I]
        nrm = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
        flip = (nrm[:, :2] @ f.n) < 0
        I = np.where(flip[:, None], I[:, ::-1], I)
        m.tris(mat, P3, np.r_[f.n, 0.0], V / height, I)


def text_along(m, line, text, s_mid, z, w, mat, height, font="DejaVuSans-Bold.ttf", aspect=1.0):
    """Text along a polyline (each glyph flat on the segment under its centre),
    centred at arc length s_mid, on the side the polyline's right-hand normal
    faces (CCW footprint edges: outward). / 折れ線に沿う文字（各文字は中心の区間に）。"""
    g = text_poly(text, height, font, aspect)
    pts = [np.array(q) for q in line.coords]
    seg = []
    s = 0.0
    for a, b in zip(pts, pts[1:]):
        L = float(np.linalg.norm(b - a))
        if L > 1e-6:
            seg.append((s, a, b, L))
            s += L
    for p in polys(g):
        cx = p.centroid.x
        sc = s_mid + cx
        s0, a, b, L = next((q for q in seg if q[0] <= sc < q[0] + q[3]), seg[-1] if sc >= seg[-1][0] else seg[0])
        t = (b - a) / L
        nrm = np.array([t[1], -t[0]])
        f = B.Facade(m, a, b, nrm)
        glyphs_on(m, f, translate(p, -cx, 0.0), sc - s0, z, w, mat, height)


def drop_over(m, no_go):
    """Remove every triangle touching ``no_go`` (the last guard: roads win);
    vertical faces by their base edge. Returns the number dropped.
    道路に触れる三角形を除く（最後の安全策）。"""
    shapely.prepare(no_go)
    gone = 0
    for mat, part in m.parts.items():
        V, N, UV, I, n = part
        if not V:
            continue
        Vc, Nc, Uc, Ic = np.concatenate(V), np.concatenate(N), np.concatenate(UV), np.concatenate(I)
        T = Vc[Ic]
        tri = shapely.polygons(np.concatenate([T[:, :, :2], T[:, :1, :2]], axis=1))
        flat = np.abs(shapely.area(tri)) < 1e-6
        base = shapely.linestrings(T[:, :2, :2])
        hit = np.where(flat, shapely.intersects(no_go, base), shapely.intersects(no_go, tri))
        if hit.any():
            gone += int(hit.sum())
            part[0], part[1], part[2], part[3] = [Vc], [Nc], [Uc], [Ic[~hit]]
    return gone


def two_sided_wall(m, mat, a, b, z0, z1, gap=0.006, uvs=1.0):
    """A thin vertical sheet seen from both sides (fence mesh), 6 mm apart:
    back-to-back coplanar faces render black in Cycles. / 両面の薄い壁。"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = np.linalg.norm(d)
    if L < 1e-3:
        return
    n = np.array([d[1], -d[0]]) / L * gap / 2
    m.wall(mat, tuple(a + n), tuple(b + n), z0, z1, 0.0, uvs)
    m.wall(mat, tuple(b - n), tuple(a - n), z0, z1, 0.0, uvs)


def chainlink(m, mat, a, b, z0, z1, pitch=0.22, w=0.016):
    """Chain-link mesh as thin two-sided diagonal strips (no alpha in this
    pipeline): a diamond lattice you can see through. / 斜めの細帯で作る金網（透けて見える）。"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    H = z1 - z0
    if L < 0.05 or H < 0.05:
        return
    t = (b - a) / L
    n = np.array([t[1], -t[0]])
    for sgn in (1, -1):
        # lines z = sgn * (s - s0) in the panel's (s, z) plane, clipped to it / パネル内に切る
        k0 = int(math.floor((-H if sgn > 0 else 0.0) / pitch)) - 1
        k1 = int(math.ceil((L if sgn > 0 else L + H) / pitch)) + 1
        for k in range(k0, k1):
            s0 = k * pitch
            if sgn > 0:
                sa, sb = max(0.0, s0), min(L, s0 + H)
            else:
                sa, sb = max(0.0, s0 - H), min(L, s0)
            if sb - sa < 0.02:
                continue
            za = sgn * (sa - s0) if sgn > 0 else (s0 - sa)
            zb = sgn * (sb - s0) if sgn > 0 else (s0 - sb)
            pa, pb = a + t * sa, a + t * sb
            hw = w / 2
            for side in (1, -1):
                o = n * 0.003 * side
                q = [(*(pa + o), z0 + za - hw), (*(pb + o), z0 + zb - hw), (*(pb + o), z0 + zb + hw),
                     (*(pa + o), z0 + za + hw)]
                if side > 0:
                    m.quad(mat, *q)
                else:
                    m.quad(mat, q[3], q[2], q[1], q[0])


class CORAL(object):
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
        self.frames = {}
        for k, (ox, oy, a) in T["frames"].items():
            o = self.X((ox, oy))
            self.frames[k] = (o, math.radians(a))
        site = [p for p, t in L.osm.areas(lambda t: t.get("name") in SITE_NAMES and "building" not in t)]
        pts = lambda lst: [self.X(q) for q in lst]
        raw = {k: orient(Polygon(pts(b["pts"])), 1.0) for k, b in T["blocks"].items()}
        self.site = unary_union(site).buffer(0) if site else unary_union(list(raw.values())).convex_hull.buffer(25)
        # roads win: the road network, islands and the site's own drives
        # (check_roads --site counts them) / 道路優先: 道路網・島・敷地内の車路
        self.drive_snapshot = L.driveways
        near = self.site.buffer(5.0)
        self.road = L.road_all
        self.no_go = unary_union([L.road_all, L.islands, L.driveways.intersection(near)])
        self.no_go_gap = self.no_go.buffer(GAP)
        self.cut_log = []
        # blocks: higher priority first, each loses what the earlier ones hold
        # 優先順に重なりを除く
        order = ["hall", "spine", "north", "middle", "curved", "spine_s", "link_e", "west_wing", "west_arm",
                 "hall_w", "hall_e", "hall_ne", "hall_s", "store"]
        taken = Polygon()
        self.blocks = {}
        for k in order:
            if k not in raw:
                continue
            b = T["blocks"][k]
            p = raw[k].difference(taken).difference(self.no_go_gap)
            p = max(polys(p), key=lambda q: q.area) if polys(p) else None
            cut = raw[k].difference(taken).area - (p.area if p is not None else 0.0)
            if cut > 0.5:
                self.cut_log.append((k, cut))
            if p is None or p.area < 8.0:
                continue
            p = orient(shapely.set_precision(p, 0.001).buffer(0), 1.0)
            if p.geom_type != "Polygon":
                p = orient(max(polys(p), key=lambda q: q.area), 1.0)
            taken = taken.union(p)
            self.blocks[k] = dict(b, poly=p, top=self.top_of(k, b))
        if "curved" in self.blocks:
            self.blocks["curved"]["bar"] = orient(Polygon(pts(T["blocks"]["curved"]["bar"])), 1.0)
        self.footprint = unary_union([b["poly"] for b in self.blocks.values()])
        # entrance pieces / 玄関まわり
        self.drum_c = self.X(T["drum"]["c"])
        self.drum_r = T["drum"]["r"]
        if self.no_go.distance(Point(self.drum_c)) < self.drum_r + GAP:
            self.drum_r = max(0.0, self.no_go.distance(Point(self.drum_c)) - GAP)
        self.porch = orient(Polygon(pts(T["porch"]["pts"])), 1.0).difference(self.no_go_gap)
        self.porch = max(polys(self.porch), key=lambda q: q.area) if polys(self.porch) else None
        self.flag = self.X(T["flagpole"])
        C = T["courts"]
        self.courts = {k: (orient(Polygon(pts(c["pts"])), 1.0), c["mat"]) for k, c in C.items()}
        self.field = orient(Polygon(pts(T["field"]["pts"])), 1.0)
        self.bricks = Polygon(pts(T["bricks"]["pts"]))
        self.ped_gate = self.X(T["ped_gate"])
        self.fitness = self.X(T["fitness"])
        # the vehicle gate: where the site's drive / road crosses the boundary
        # 車両門: 敷地境界を車路が横切る所
        ring = self.site.exterior
        span = ring.intersection(self.no_go.buffer(0.3))
        deep = self.no_go.intersection(self.site.buffer(-4.0))
        # a crossing is short and its road goes on into the site; long runs are
        # streets along the boundary / 横断は短く敷地内へ続く（長いのは境界沿いの道路）
        self.gates = [g for g in getattr(span, "geoms", [span]) if g.geom_type == "LineString"
                      and 2.5 < g.length < 14.0 and deep.distance(g) < 6.0]
        # the school's own lot: the OSM area less the streets and their pavement
        # band (the OSM school way runs over St 51 in places) / 学校の敷地: 道路と歩道帯を除く
        edge_roads = self.road.buffer(2.6).intersection(ring.buffer(7.0))
        for g in self.gates:
            edge_roads = edge_roads.difference(g.buffer(6.0))
        lot = self.site.difference(edge_roads)
        self.lot = max(polys(lot), key=lambda q: q.area) if polys(lot) else self.site
        self.centre = self.footprint.centroid
        self._parts = None
        self._ground = None
        for k, a in self.cut_log:
            print("coral: %s cut %.1f m2 (overlap or road clearance)" % (k, a))

    def top_of(self, k, b):
        """Wall top (eave or parapet) of a block, metres. / 壁の上端。"""
        return {"north": Z0 + 4 * F4, "middle": Z0 + 4 * F4, "spine": Z0 + 4 * F4 + 2.2,
                "curved": Z0 + 3 * F3, "west_wing": Z0 + 3 * F3 + 0.9, "west_arm": Z0 + 2 * F3 + 0.8,
                "spine_s": Z0 + 3 * F3 + 0.9, "link_e": Z0 + 3 * F3 + 0.9, "hall": HALL_EAVE,
                "hall_w": Z0 + 2 * F3 + 0.8, "hall_e": 4.6, "hall_ne": 4.6, "hall_s": 4.6,
                "store": 3.2}.get(k, Z0 + b.get("storeys", 1) * F3)

    # -- frames / 座標系
    def axes(self, fr):
        (ox, oy), a = self.frames[fr]
        return np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])

    def at(self, fr, u, v):
        (ox, oy), a = self.frames[fr]
        eu, ev = self.axes(fr)
        return (ox + eu[0] * u + ev[0] * v, oy + eu[1] * u + ev[1] * v)

    def uv(self, fr, x, y):
        (ox, oy), a = self.frames[fr]
        eu, ev = self.axes(fr)
        d = np.array([x - ox, y - oy])
        return float(d @ eu), float(d @ ev)

    def facing(self, n, fr="S"):
        eu, ev = self.axes(fr)
        cu, cv = float(n @ eu), float(n @ ev)
        if abs(cv) >= abs(cu):
            return "N" if cv > 0 else "S"
        return "E" if cu > 0 else "W"

    def ring_in(self, d):
        """The lot's boundary d metres in (its largest piece). / 敷地境界から d m 内側の輪。"""
        g = self.lot.buffer(-d, join_style="mitre")
        return max(polys(g), key=lambda q: q.area).exterior

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
        foot = unary_union([self.footprint] + ([self.porch] if self.porch is not None else []))
        courts = unary_union([c for c, _ in self.courts.values()]).intersection(inner).difference(foot)
        field = self.field.intersection(inner).difference(foot)
        hard = foot.buffer(2.4, join_style="mitre")
        bricks = self.bricks.intersection(inner).difference(foot).difference(courts)
        hard = hard.intersection(inner).difference(foot).difference(courts).difference(field).difference(bricks)
        grass = inner.difference(foot).difference(courts).difference(field).difference(bricks).difference(hard)
        mouths = []
        for c in self.goal_spots():
            mouths.append(Point(c[0]).buffer(3.5).intersection(field))
        self._ground = dict(courts=courts, field=field, bricks=bricks, hard=hard, grass=grass,
                            mouths=unary_union(mouths) if mouths else Polygon())
        return self._ground

    def goal_spots(self):
        """((x, y) of each goal mouth, unit vector across the goal). / ゴール位置と向き。"""
        a, b, c, d = rect_axes(self.field)
        out = []
        for p, q in ((a, d), (b, c)):
            mid = (p + q) / 2
            cen = (a + b + c + d) / 4
            inw = cen - mid
            inw = inw / np.linalg.norm(inw)
            out.append((tuple(mid + inw * 0.8), (q - p) / np.linalg.norm(q - p)))
        return out

    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.site.intersects(tile):
            return m
        Z = self.zones()
        T = lambda g: g.intersection(tile)
        m.flat("grass_park", T(Z["grass"]), z + 0.005, 4.0)
        m.flat("concrete_path", T(Z["hard"]), z + 0.01, 2.0)
        m.flat("coral_brick", T(Z["bricks"]), z + 0.01, 2.0)
        m.flat("grass_park", T(Z["field"].difference(Z["mouths"])), z + 0.006, 4.0)
        m.flat("soil", T(Z["mouths"]), z + 0.008, 3.0)
        for k, (c, mat) in self.courts.items():
            # once, in its own tile (copies z-fight) / 自分のタイルに 1 回だけ
            if not tile.contains(c.centroid):
                continue
            g = c.intersection(self.inner()).difference(self.no_go.buffer(0.05))
            if abs(g.area - c.area) < 0.5:
                q = [np.array(p) for p in list(orient(c, 1.0).exterior.coords)[:4]]
                if math.dist(q[0], q[1]) < math.dist(q[1], q[2]):
                    q = q[1:] + q[:1]
                m.quad(mat, *[(p[0], p[1], z + 0.02) for p in q], (0, 0), (1, 0), (1, 1), (0, 1))
            else:
                # clipped by the boundary: texture laid by the court's own axes / 切れたコート
                a, b, cc, d = rect_axes(c)
                ax, sd = (b - a) / np.linalg.norm(b - a), (d - a) / np.linalg.norm(d - a)
                La, Ld = np.linalg.norm(b - a), np.linalg.norm(d - a)
                m.flat(mat, g, z + 0.02, uvfun=lambda V, a=a, ax=ax, sd=sd, La=La, Ld=Ld:
                       np.column_stack([(V - a) @ ax / La, (V - a) @ sd / Ld]))
        # staff car-park bay lines on the bricks / 職員駐車場の区画線
        for ln in self.bay_lines():
            if tile.contains(ln.interpolate(0.5, normalized=True)):
                m.flat("paint_white", ln.buffer(0.05, cap_style="flat"), z + 0.02, 1.0)
        return m

    def bay_lines(self):
        u0, u1 = self.T["bays_u"]
        ring = self.ring_in(1.2)
        out = []
        eu, ev = self.axes("S")
        for u in np.arange(u0, u1 + 1e-6, 2.5):
            # where the S-frame line u meets the south part of the ring / 南側の境界との交点
            a, b = self.at("S", u, -95.0), self.at("S", u, -40.0)
            hit = LineString([a, b]).intersection(ring)
            ps = [q for q in getattr(hit, "geoms", [hit]) if q.geom_type == "Point"]
            if not ps:
                continue
            p = min(ps, key=lambda q: self.uv("S", q.x, q.y)[1])
            q = (p.x + ev[0] * 5.0, p.y + ev[1] * 5.0)
            ln = LineString([(p.x, p.y), q])
            if ln.distance(self.no_go) > 0.3 and not ln.intersects(self.footprint):
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

        for k, b in self.blocks.items():
            c = b["poly"].centroid
            add((c.x, c.y), self.block, k, b)
        if self.porch is not None:
            c = self.porch.centroid
            add((c.x, c.y), self.porch_frame)
        if self.drum_r > 1.0:
            add(self.drum_c, self.drum)
        c = self.site.centroid
        add((c.x, c.y), self.site_fence)
        add((c.x, c.y), self.walkway_e)
        add((c.x, c.y), self.trees)
        add(self.flag, self.flagpole)
        f = self.field.centroid
        add((f.x, f.y), self.goals)
        for g in self.gates:
            q = g.interpolate(0.5, normalized=True)
            add((q.x, q.y), self.vehicle_gate, g)
        add(self.fitness, self.fitness_corner)
        if "red_bb" in self.courts:
            c = self.courts["red_bb"][0].centroid
            add((c.x, c.y), self.hoops)
        for nm, n in self.drop_log:
            print("coral: %s: %d triangles over the road network dropped" % (nm, n))
        self.tris = sum(pm.ntris() for _, pm in parts)
        print("coral: %d blocks, %d triangles" % (len(self.blocks), self.tris))
        return parts

    # -- facades / 外壁
    def pieces(self, k, a, b, n, top):
        """Runs [(u0, u1, z_cover)] of an edge: z_cover = Z0 where it is free,
        else the wall top of the block touching it (within 0.15 m).
        辺の区間と、接する隣棟の高さ。"""
        L = float(np.linalg.norm(b - a))
        t = (b - a) / L
        others = [(kk, bb["poly"], bb["top"]) for kk, bb in self.blocks.items() if kk != k]
        step = 0.25
        us = np.arange(step / 2, L, step)
        cov = []
        for u in us:
            p = Point(*(a + t * u + n * 0.15))
            h = Z0
            for kk, poly, tp in others:
                if poly.contains(p):
                    h = max(h, tp)
            cov.append(h)
        runs = []
        for u, h in zip(us, cov):
            u0, u1 = max(0.0, u - step / 2), min(L, u + step / 2)
            if runs and abs(runs[-1][2] - h) < 1e-6:
                runs[-1][1] = u1
            else:
                runs.append([u0, u1, h])
        return [tuple(r) for r in runs]

    def block(self, m, k, b):
        poly, top = b["poly"], b["top"]
        kind = b.get("kind")
        if kind == "curved":
            return self.curved_block(m, k, b)
        if kind == "hall":
            return self.hall(m, k, b)
        base = {"classroom": "coral_white", "spine": "coral_white", "bluewing": "coral_blue",
                "westwing": "coral_white", "white": "coral_white", "annex": "coral_white",
                "store": "coral_white"}.get(kind, "coral_white")
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            side = self.facing(n, "H" if k.startswith("hall") else "S")
            for u0, u1, zc in self.pieces(k, a, bb, n, top):
                if u1 - u0 < 0.05:
                    continue
                if zc > Z0 + 0.01:
                    # covered by a neighbour up to zc: plain wall above it / 隣棟より上は無地の壁
                    if top > zc + 0.01:
                        f.face(base, u0, u1, zc, top, 0.0, 2.0)
                    continue
                self.facade(f, k, kind, side, u0, u1, L, top)
        self.pipes(m, k, b)
        z = top
        if b.get("roof") == "gable":
            if kind == "store":
                gable(m, poly, z, 5.0, "coral_roof", "coral_white", ov=0.4)
            else:
                gable(m, poly, z, z + 2.8, "coral_roof", base, ov=0.7)
            return
        # flat roofs: the parapet is part of the wall top for the wings / 陸屋根
        tall_par = kind in ("westwing", "bluewing") or k in ("west_arm", "hall_w")
        if kind == "spine":
            flat_roof(m, poly, z, "coral_yellow", parapet=0.5, cap="coral_white")
            self.spine_vents(m, poly, z)
        elif tall_par:
            flat_roof(m, poly, z - 0.9, base, parapet=0.9, cap="coral_blue" if kind == "bluewing" else "coral_white")
            if kind == "westwing":
                self.plant(m, poly, z - 0.9)
        else:
            flat_roof(m, poly, z, base, parapet=0.5)

    def pipes(self, m, k, b, every=18.0):
        """Rain-water down pipes (grey) at the free corners and every ~18 m on
        long walls. / 雨どいの竪樋（角と約 18 m ごと）。"""
        poly, top = b["poly"], b["top"]
        if top < 4.0:
            return
        for a, bb, L, n in B.edges_of(poly):
            if L < 6.0:
                continue
            t = (bb - a) / L
            us = [0.45] + list(np.arange(every, L - 3.0, every))
            runs = self.pieces(k, a, bb, n, top)
            for u in us:
                if any(r0 <= u <= r1 and zc > Z0 + 0.01 for r0, r1, zc in runs):
                    continue
                q = a + t * u + n * 0.14
                m.cylinder("coral_pipe", q[0], q[1], Z0 + 0.05, top - 0.15, 0.06, seg=6, top=False)

    def facade(self, f, k, kind, side, u0, u1, L, top):
        if kind == "classroom":
            if k == "north" and side == "N":
                return self.classroom_out(f, u0, u1, L, top)
            if side in ("N", "S"):
                pmat = ("coral_blue", "coral_yellow") if (k == "middle" and side == "S") else ("coral_white",)
                return self.corridor(f, u0, u1, L, 4, F4, top, parapets=pmat, pier="coral_blue")
            return self.end_wall(f, u0, u1, L, top, side)
        if kind == "spine":
            if side == "E":
                return self.spine_east(f, u0, u1, L, top)
            if side == "W":
                return self.corridor(f, u0, u1, L, 4, F4, Z0 + 4 * F4, parapets=("coral_white",),
                                     pier="coral_white", band=("coral_yellow", top))
            if side == "N":
                return self.stair_glass(f, u0, u1, L, top)
            return self.punched(f, u0, u1, L, top, 4, F4, "coral_white", "coral_glass_dk", bay=4.0)
        if kind == "bluewing":
            return self.blue_wing(f, u0, u1, L, top)
        if kind == "westwing":
            return self.punched(f, u0, u1, L, top, 3, F3, "coral_white", "coral_glass_dk", bay=3.2,
                                band="coral_blue" if side in ("S", "E") else None)
        if kind == "store":
            return self.store_wall(f, u0, u1, L, top)
        storeys = 2 if k in ("west_arm", "hall_w") else 1
        return self.punched(f, u0, u1, L, top, storeys, F3, "coral_white", "coral_glass_gf", bay=3.4)

    @staticmethod
    def bay_centres(L, bay, margin=0.8):
        n = int((L - 2 * margin) // bay)
        if n <= 0:
            return []
        w = (L - 2 * margin) / n
        return [margin + (i + 0.5) * w for i in range(n)]

    def punched(self, f, u0, u1, L, top, storeys, fh, wall, glass, bay=3.2, ww=1.9, wh=1.4, sill=0.95,
                band=None):
        """Storeys of punched windows on a plain wall, a plinth, an optional
        coloured band under the parapet. / 窓の並ぶ壁。"""
        cs = [c for c in self.bay_centres(L, bay) if u0 + 0.2 <= c - ww / 2 and c + ww / 2 <= u1 - 0.2]
        for s in range(storeys):
            z0 = Z0 + s * fh
            z1 = min(z0 + fh, top) if s < storeys - 1 else top
            opens = [(c - ww / 2, c + ww / 2, z0 + sill, z0 + sill + wh) for c in cs]
            wall_openings(f, wall, u0, u1, z0, z1, opens, 2.0)
            for o in opens:
                opening(f, *o, wall, glass, 0.15)
        f.block("concrete", u0, u1, 0.0, 0.04, Z0, Z0 + 0.35, ends=False, bottom=False)
        if band:
            f.block(band, u0, u1, 0.0, 0.06, top - 1.5, top - 0.6, ends=False)

    def classroom_out(self, f, u0, u1, L, top):
        """The north block's street face (SV Apr 2019): white ground floor with
        green-framed windows and a green band; blue storeys above with deep
        projecting bays (pairs of yellow-framed windows); a yellow stair core
        with a white frame at the east end, a glazed stair tower at the west.
        北棟の道路側: 白い 1 階、青い上階に張り出した窓枠ベイ、両端の階段室。"""
        zb, zt = Z0 + F4, top
        # the yellow stair core stands at the EAST end and the glazed tower at the
        # WEST end: Street View Apr 2019 h197 (yellow at the left looking SSW) and
        # h165 (glazing + yellow top corner at the near west end); description.md
        # had them the other way round. The CCW north face runs east to west.
        # 黄色い階段室は東端、ガラスの塔は西端（SV h197/h165。description.md と逆）
        if float(f.t @ self.axes("S")[0]) < 0:
            west, east = (0.0, 4.2), (L - 4.6, L)        # (names: west = yellow core, east = glazing)
        else:
            west, east = (L - 4.2, L), (0.0, 4.6)
        lo, hi = min(west[1], east[1]), max(west[0], east[0])
        # ground floor / 1 階
        cs = [c + lo for c in self.bay_centres(hi - lo, 3.0, 0.4)]
        cs = [c for c in cs if u0 + 0.2 <= c - 1.15 and c + 1.15 <= u1 - 0.2]
        a0, a1 = max(u0, lo), min(u1, hi)
        if a1 > a0:
            opens = [(c - 1.15, c + 1.15, Z0 + 0.9, Z0 + 2.6) for c in cs]
            wall_openings(f, "coral_white", a0, a1, Z0, zb, opens, 2.0)
            for o in opens:
                opening(f, *o, "coral_white", "coral_glass_gf", 0.15)
            f.block("coral_green", a0, a1, 0.0, 0.05, zb - 0.45, zb - 0.2, ends=False)
            f.block("coral_white", a0, a1, 0.0, 0.3, zb, zb + 0.2)            # slab ledge / 庇
            # upper storeys: blue wall between projecting bays / 上階: 張り出しベイの間の青い壁
            mods = [c for c in self.bay_centres(a1 - a0, 7.0, 0.5)]
            mods = [c + a0 for c in mods]
            spans = []
            for c in mods:
                b0, b1 = c - 2.8, c + 2.8
                spans.append((b0, b1))
                f.side("coral_blue", b0, 0.0, 0.65, zb + 0.2, zt, -1)
                f.side("coral_blue", b1, 0.0, 0.65, zb + 0.2, zt, 1)
                f.hplane("coral_blue", b0, b1, 0.0, 0.65, zt, True)
                f.hplane("coral_blue", b0, b1, 0.0, 0.65, zb + 0.2, False)
                ff = B.Facade(f.m, f.a + f.n * 0.65, f.a + f.n * 0.65 + f.t * L, f.n)
                opens = []
                for s in range(1, 4):
                    z0 = Z0 + s * F4
                    for d in (-1.3, 1.3):
                        opens.append((c + d - 1.1, c + d + 1.1, z0 + 1.0, z0 + 2.25))
                wall_openings(ff, "coral_blue", b0, b1, zb + 0.2, zt, opens, 2.0)
                for o in opens:
                    opening(ff, *o, "coral_blue", "coral_glass_yf", 0.2)
            u = a0
            for b0, b1 in spans:
                f.face("coral_blue", u, b0, zb + 0.2, zt, 0.0, 2.0)
                u = b1
            f.face("coral_blue", u, a1, zb + 0.2, zt, 0.0, 2.0)
            f.block("coral_white", a0, a1, 0.0, 0.25, zt - 0.4, zt, ends=False, bottom=False)
        # the yellow stair core (east end) and the glazed tower (west end) / 階段室
        w0, w1 = max(u0, west[0]), min(u1, west[1])
        if w1 > w0 + 0.5:
            opens = [(w0 + (w1 - w0) / 2 - 0.35, w0 + (w1 - w0) / 2 + 0.35, Z0 + s * F4 + 0.6, Z0 + s * F4 + 3.0)
                     for s in range(4)]
            wall_openings(f, "coral_yellow", w0, w1, Z0, zt, opens, 2.0)
            for o in opens:
                opening(f, *o, "coral_yellow", "coral_glass_dk", 0.2, sill=False)
            for p0, p1 in ((w0, w0 + 0.35), (w1 - 0.35, w1)):
                f.block("coral_white", p0, p1, 0.0, 0.3, Z0, zt + 0.4, bottom=False)
            f.block("coral_white", w0, w1, 0.0, 0.3, zt - 0.2, zt + 0.4, bottom=False)
        e0, e1 = max(u0, east[0]), min(u1, east[1])
        if e1 > e0 + 0.5:
            tiled_face(f, "coral_grid", e0 + 0.3, e1 - 0.3, Z0 + 0.4, zt - 0.3, -0.15, 1.2, 1.2)
            # white posts and rails proud of the glass; only the faces round the
            # glass, none in the neighbouring walls' planes / ガラスを囲む白枠（隣の壁面と同一面にしない）
            for p0, p1, side in ((e0, e0 + 0.3, 1), (e1 - 0.3, e1, -1)):
                f.face("coral_white", p0, p1, Z0, zt, 0.1)
                f.side("coral_white", p1 if side > 0 else p0, -0.15, 0.1, Z0 + 0.4, zt - 0.3, side)
            f.face("coral_white", e0 + 0.3, e1 - 0.3, zt - 0.3, zt, 0.1)
            f.hplane("coral_white", e0 + 0.3, e1 - 0.3, -0.15, 0.1, zt - 0.3, False)
            f.face("coral_white", e0 + 0.3, e1 - 0.3, Z0, Z0 + 0.4, 0.1)
            f.hplane("coral_white", e0 + 0.3, e1 - 0.3, -0.15, 0.1, Z0 + 0.4, True)
            f.hplane("coral_white", e0, e1, 0.0, 0.1, zt, True)

    def corridor(self, f, u0, u1, L, storeys, fh, top, parapets=("coral_white",), pier="coral_blue",
                 band=None, fins=None):
        """Open corridors: white slab edges, parapets (colours alternate per
        7 m bay), the yellow corridor wall behind (doors and windows drawn),
        orange soffits, piers through all storeys, end walls closing the
        recess. / 開放廊下（白いスラブ、色違いの腰壁、黄色い廊下壁、橙の天井、通し柱）。"""
        D = D_CORR
        zt = top
        # at a block corner the recess stops 0.25 m short (the end wall's
        # thickness): its end face must not lie in the end wall's plane
        # 角では凹みを 0.25 m 手前で止める（端壁と同一面にしない）
        if u0 < 0.01:
            f.face("coral_white", u0, u0 + 0.25, Z0, zt, 0.0)
            u0 += 0.25
        if u1 > L - 0.01:
            f.face("coral_white", u1 - 0.25, u1, Z0, zt, 0.0)
            u1 -= 0.25
        for s in range(storeys):
            z0 = Z0 + s * fh
            z1 = z0 + fh if s < storeys - 1 else zt
            # the corridor wall behind: a door and a window per 3.6 m classroom
            # module, real openings with frames / 奥の廊下壁: 3.6 m ごとに扉と窓（実際の開口）
            g = B.Facade(f.m, f.a - f.n * D, f.a - f.n * D + f.t * f.L, f.n)
            opens, doors = [], []
            for k0 in np.arange(math.floor(u0 / 3.6) * 3.6, u1, 3.6):
                d0, w0 = k0 + 0.45, k0 + 1.75
                if d0 >= u0 + 0.1 and d0 + 0.95 <= u1 - 0.1:
                    doors.append((d0, d0 + 0.95, z0 + 0.02, z0 + 2.15))
                if w0 >= u0 + 0.1 and w0 + 1.6 <= u1 - 0.1:
                    opens.append((w0, w0 + 1.6, z0 + 0.95, z0 + 2.2))
            wall_openings(g, "coral_corr_wall", u0, u1, z0, z1 - 0.3, opens + doors, 2.0)
            for o in doors:
                opening(g, *o, "coral_corr_wall", "coral_door", 0.08, sill=False, frame="coral_frame_dk")
            for o in opens:
                opening(g, *o, "coral_corr_wall", "coral_glass_dk", 0.1, sill=False)
            f.hplane("concrete", u0, u1, -D, 0.0, z0 + 0.01, True)
            f.hplane("coral_orange", u0, u1, -D, 0.0, z1 - 0.3, False)
            f.face("coral_white", u0, u1, z1 - 0.3, z1, 0.0)            # slab edge above / 上階スラブ端
            if s == 0:
                f.face("concrete", u0, u1, Z0 - 0.15, Z0 + 0.01, 0.0)
                continue
            mods = self.bay_centres(L, 7.0, 0.0)
            edges = [0.0] + [(mods[i] + mods[i + 1]) / 2 for i in range(len(mods) - 1)] + [L] if mods else [0.0, L]
            for i in range(len(edges) - 1):
                a, b = max(u0, edges[i]), min(u1, edges[i + 1])
                if b - a < 0.05:
                    continue
                mat = parapets[(i + s) % len(parapets)]
                f.block(mat, a, b, -0.15, 0.0, z0 + 0.01, z0 + 1.0, top=True, bottom=False, ends=False)
                face_in(f, mat, a, b, z0 + 0.01, z0 + 1.0, -0.15)
                # steel handrail on the parapet / 腰壁上の手摺
                f.block("railing", a, b, -0.11, -0.04, z0 + 1.0, z0 + 1.06, ends=False)
        # the roof slab band / 屋上スラブの帯
        if band is not None:
            bmat, btop = band
            f.block(bmat, u0, u1, -0.2, 0.1, zt, btop, top=False, bottom=True, ends=False)
        # piers through every storey / 通し柱
        mods = self.bay_centres(L, 7.0, 0.0)
        pts = [0.6] + [(mods[i] + mods[i + 1]) / 2 for i in range(len(mods) - 1)] + [L - 0.6] if mods else [0.6, L - 0.6]
        for p in pts:
            if u0 + 0.25 <= p <= u1 - 0.25:
                f.block(pier, p - 0.25, p + 0.25, -D, 0.06, Z0, zt, top=False, bottom=False)
        # close the recess at the run's ends / 区間端で凹みを閉じる
        f.side("coral_white", u0, -D, 0.0, Z0, zt, 1)
        f.side("coral_white", u1, -D, 0.0, Z0, zt, -1)

    def spine_east(self, f, u0, u1, L, top):
        """The spine's field face (SV Feb 2018): open corridors with white
        parapets, eight blue full-height fins, the yellow parapet band, a white
        stair tower with slot windows at the north end, a white end bay south.
        渡り棟の運動場側: 開放廊下、青い縦フィン 8 本、黄色いパラペット帯、北端の階段室。"""
        zc = Z0 + 4 * F4
        # which end is north: the edge runs along t / 北端の判定
        north_at_end = float(f.t @ self.axes("S")[1]) > 0
        st = (L - 5.0, L) if north_at_end else (0.0, 5.0)
        en = (0.0, 4.0) if north_at_end else (L - 4.0, L)
        lo, hi = (4.0, L - 5.0) if north_at_end else (5.0, L - 4.0)
        c0, c1 = max(u0, lo), min(u1, hi)
        if c1 > c0 + 1.0:
            self.corridor(f, c0, c1, L, 4, F4, zc, parapets=("coral_white",), pier="coral_white")
            # eight blue fins at about 6 m centres / 青いフィン
            n = 8
            for i in range(n):
                p = c0 + (i + 0.5) * (c1 - c0) / n
                f.block("coral_blue", p - 0.35, p + 0.35, -0.2, 0.7, Z0, zc + 0.3, top=True, bottom=False)
        # the yellow parapet band over the whole face / 黄色いパラペット帯
        f.block("coral_yellow", u0, u1, 0.0, 0.15, zc, top, top=False, bottom=True, ends=False)
        f.face("coral_yellow", u0, u1, zc, top, 0.0)
        # north stair tower (white, slot windows) and the south end bay / 階段室と南端
        for (a, b), slots in ((st, True), (en, False)):
            a, b = max(u0, a), min(u1, b)
            if b - a < 0.5:
                continue
            if slots:
                opens = [((a + b) / 2 - 0.3, (a + b) / 2 + 0.3, Z0 + s * F4 + 0.5, Z0 + s * F4 + 3.0)
                         for s in range(4)]
                wall_openings(f, "coral_white", a, b, Z0, zc, opens, 2.0)
                for o in opens:
                    opening(f, *o, "coral_white", "coral_glass_dk", 0.2, sill=False)
            else:
                self.punched(f, a, b, L, zc, 4, F4, "coral_white", "coral_glass_dk", bay=3.0, ww=1.5)

    def stair_glass(self, f, u0, u1, L, top):
        """Glazed stair-tower end: a grid of square panes in a white frame. / ガラス張りの階段室。"""
        m0, m1 = max(u0, L / 2 - 4.0), min(u1, L / 2 + 4.0)
        if m1 > m0 + 0.5:
            f.face("coral_white", u0, m0, Z0, top, 0.0, 2.0)
            f.face("coral_white", m1, u1, Z0, top, 0.0, 2.0)
            tiled_face(f, "coral_grid", m0, m1, Z0 + 0.5, top - 2.4, -0.2, 1.2, 1.2)
            f.side("coral_white", m0, -0.2, 0.0, Z0 + 0.5, top - 2.4, 1)
            f.side("coral_white", m1, -0.2, 0.0, Z0 + 0.5, top - 2.4, -1)
            f.hplane("coral_white", m0, m1, -0.2, 0.0, Z0 + 0.5, True)
            f.hplane("coral_white", m0, m1, -0.2, 0.0, top - 2.4, False)
            f.face("coral_white", m0, m1, Z0, Z0 + 0.5, 0.0)
            f.face("coral_yellow", m0, m1, top - 2.4, top, 0.0)
        else:
            f.face("coral_white", u0, u1, Z0, top, 0.0, 2.0)

    def end_wall(self, f, u0, u1, L, top, side):
        """Classroom block ends: white with a blue panel and a yellow top
        corner (SV Apr 2019 north-west), one window per storey.
        教室棟の妻壁: 白地に青いパネルと黄色い角。"""
        c = L / 2
        opens = [(c - 0.7, c + 0.7, Z0 + s * F4 + 1.0, Z0 + s * F4 + 2.4) for s in range(4)
                 if u0 + 0.2 <= c - 0.7 and c + 0.7 <= u1 - 0.2]
        wall_openings(f, "coral_white", u0, u1, Z0, top, opens, 2.0)
        for o in opens:
            opening(f, *o, "coral_white", "coral_glass_dk", 0.15)
        p0, p1 = max(u0, L * 0.18), min(u1, L * 0.42)
        if p1 > p0:
            f.block("coral_blue", p0, p1, 0.0, 0.06, Z0 + F4, top, bottom=True)
        y0, y1 = max(u0, L * 0.72), min(u1, L)
        if y1 > y0:
            f.block("coral_yellow", y0, y1, 0.0, 0.06, top - 3.2, top, bottom=True, top=True)
        f.block("concrete", u0, u1, 0.0, 0.04, Z0, Z0 + 0.35, ends=False, bottom=False)

    def blue_wing(self, f, u0, u1, L, top):
        """3-storey blue wing (MOE photo): blue render, rows of dark-framed
        windows, a white plinth band. / 青い 3 階建ての翼棟。"""
        cs = [c for c in self.bay_centres(L, 3.0) if u0 + 0.2 <= c - 1.2 and c + 1.2 <= u1 - 0.2]
        for s in range(3):
            z0 = Z0 + s * F3
            z1 = z0 + F3 if s < 2 else top
            opens = [(c - 1.2, c + 1.2, z0 + 0.9, z0 + 2.3) for c in cs]
            wall_openings(f, "coral_blue", u0, u1, z0, z1, opens, 2.0)
            for o in opens:
                opening(f, *o, "coral_blue", "coral_glass_dk", 0.15)
            if s > 0:
                f.block("coral_blue", u0, u1, 0.0, 0.12, z0 - 0.1, z0 + 0.1, ends=False)
        f.block("coral_white", u0, u1, 0.0, 0.05, Z0, Z0 + 0.6, ends=False, bottom=False)

    def store_wall(self, f, u0, u1, L, top):
        """The store / bin centre: white with dark louvred openings. / 倉庫: 白壁にガラリ。"""
        cs = [c for c in self.bay_centres(L, 2.4) if u0 + 0.2 <= c - 0.6 and c + 0.6 <= u1 - 0.2]
        opens = [(c - 0.6, c + 0.6, Z0 + 1.6, Z0 + 2.4) for c in cs]
        wall_openings(f, "coral_white", u0, u1, Z0, top, opens, 2.0)
        for o in opens:
            opening(f, *o, "coral_white", "coral_louvre", 0.1, sill=False)

    # -- the curved block / 曲面棟
    def curved_block(self, m, k, b):
        """Two storeys of yellow spandrel bands, thin blue lines and ribbon
        windows along the front (curve), the lettering upturn with a white
        railing on the second-floor slab, the blue third storey set back
        2.5 m, a grey gable over the bar behind; open corridors north.
        曲面棟: 黄色い帯と連窓の 2 層、校名の立上りと手摺、3 階は後退、奥に切妻。"""
        poly = b["poly"]
        z2 = Z0 + 2 * F3
        z3 = Z0 + 3 * F3
        eu, ev = self.axes("S")
        L3 = poly.intersection(translate(poly, ev[0] * 2.5, ev[1] * 2.5))
        L3 = max(polys(L3), key=lambda q: q.area) if polys(L3) else poly
        front = []
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            side = self.facing(n)
            is_front = side == "S"
            for u0, u1, zc in self.pieces(k, a, bb, n, z3):
                if u1 - u0 < 0.05:
                    continue
                if zc > Z0 + 0.01:
                    if z3 > zc + 0.01:
                        f.face("coral_white", u0, u1, zc, z3 if not is_front else z2, 0.0, 2.0)
                    continue
                if is_front:
                    self.ribbon_front(f, u0, u1, L, z2)
                    front.append((a, bb, n, u0, u1))
                elif side == "N":
                    self.corridor(f, u0, u1, L, 3, F3, z3, parapets=("coral_white",), pier="coral_blue")
                else:
                    self.punched(f, u0, u1, L, z3, 3, F3, "coral_white", "coral_glass_dk", bay=3.2)
        # the second-floor slab, the yellow upturn and its railing on the front
        # 2 階スラブ・黄色い立上り・手摺（前面）
        for a, bb, n, u0, u1 in front:
            f = B.Facade(m, a, bb, n)
            f.block("coral_white", u0, u1, 0.0, 0.85, z2 - 0.25, z2, top=False, bottom=True, ends=True)
            f.block("coral_yellow", u0, u1, 0.65, 0.85, z2, z2 + 1.15, top=True, bottom=False, ends=True)
            face_in(f, "coral_yellow", u0, u1, z2, z2 + 1.15, 0.65)
            f.block("coral_rail", u0, u1, 0.72, 0.78, z2 + 1.95, z2 + 2.0, ends=False)
            for u in np.arange(u0 + 0.1, u1, 1.6):
                f.block("coral_rail", u - 0.025, u + 0.025, 0.72, 0.78, z2 + 1.15, z2 + 1.95, top=False,
                        bottom=False)
        # the name on the upturn, along the curve / 立上りの校名（曲面に沿う）
        line = self.front_line(poly)
        if line is not None and line.length > 26.0:
            text_along(m, line, "CORAL PRIMARY SCHOOL", line.length * 0.5, z2 + 0.18, 0.88, "coral_blue", 0.8,
                       font="DejaVuSans-Bold.ttf", aspect=0.95)
        # the terrace in front of L3 / 3 階前のテラス
        m.flat("roof_concrete", poly.difference(L3), z2 + 0.01, 3.0)
        # the third storey: blue, punched window pairs, on the set-back front
        # 3 階: 青い壁に窓の対
        for a, bb, L, n in B.edges_of(L3):
            mid = Point(*((a + bb) / 2))
            if poly.exterior.distance(mid) < 0.05:
                continue                          # drawn with the full-height facade / 全高の外壁で描画済み
            f = B.Facade(m, a, bb, n)
            cs = [c for c in self.bay_centres(L, 3.6) if c - 1.1 > 0.2 and c + 1.1 < L - 0.2]
            opens = []
            for c in cs:
                opens += [(c - 1.05, c - 0.1, z2 + 0.95, z2 + 2.15), (c + 0.1, c + 1.05, z2 + 0.95, z2 + 2.15)]
            wall_openings(f, "coral_blue", 0.0, L, z2, z3, opens, 2.0)
            for o in opens:
                opening(f, *o, "coral_blue", "coral_glass_dk", 0.15)
            f.block("coral_white", 0.0, L, 0.0, 0.2, z3 - 0.3, z3, ends=True, bottom=True)
        # roofs: flat over L3, the gable over the bar behind / 屋根: 3 階の陸屋根と奥の切妻
        bar = b.get("bar")
        flat_roof(m, L3, z3, "coral_blue", parapet=0.3, cap="coral_white")
        if bar is not None:
            barp = bar.intersection(L3)
            barp = max(polys(barp), key=lambda q: q.area) if polys(barp) else None
            if barp is not None and barp.area > 30:
                gable(m, barp, z3 + 0.3, z3 + 3.5, "coral_roof", "coral_blue", ov=0.4)

    def front_line(self, poly):
        """The longest run of south-facing edges as one line (CCW, so its
        right side is outside). / 南向きの辺の最長の連なり。"""
        runs, cur = [], []
        for a, b, L, n in B.edges_of(poly):
            if self.facing(n) == "S" and float(n @ self.axes("S")[1]) < -0.5:
                if cur and np.allclose(cur[-1], a, atol=1e-6):
                    cur.append(b)
                else:
                    if len(cur) > 1:
                        runs.append(cur)
                    cur = [a, b]
            else:
                if len(cur) > 1:
                    runs.append(cur)
                cur = []
        if len(cur) > 1:
            runs.append(cur)
        if not runs:
            return None
        best = max(runs, key=lambda r: LineString(r).length)
        return LineString([tuple(q) for q in best])

    def ribbon_front(self, f, u0, u1, L, z2):
        """Yellow spandrels with thin blue lines and continuous ribbon windows
        (green glass, dark frames), two storeys. / 黄色い腰壁と青い細線、連窓。"""
        for s in range(2):
            z0 = Z0 + s * F3
            w0, w1 = z0 + 1.0, z0 + 2.55
            a, b = u0 + 0.15, u1 - 0.15
            if b - a > 0.5:
                f.face("coral_yellow", u0, u1, z0, w0, 0.0, 2.0)
                f.face("coral_yellow", u0, u1, w1, z0 + F3, 0.0, 2.0)
                f.face("coral_yellow", u0, a, w0, w1, 0.0)
                f.face("coral_yellow", b, u1, w0, w1, 0.0)
                opening(f, a, b, w0, w1, "coral_yellow", "coral_ribbon", 0.12, tu=1.25, sill=False)
            else:
                f.face("coral_yellow", u0, u1, z0, z0 + F3, 0.0)
            for zz in (w0 - 0.12, w1 + 0.02):
                f.block("coral_blue", u0, u1, 0.0, 0.04, zz, zz + 0.1, ends=False)

    # -- the hall / ホール
    def hall(self, m, k, b):
        poly = b["poly"]
        zt = HALL_EAVE
        eu, ev = self.axes("H")
        for a, bb, L, n in B.edges_of(poly):
            f = B.Facade(m, a, bb, n)
            side = self.facing(n, "H")
            for u0, u1, zc in self.pieces(k, a, bb, n, zt):
                if u1 - u0 < 0.05:
                    continue
                zb = zc if zc > Z0 + 0.01 else Z0
                if side == "S":
                    self.hall_entrance_side(f, u0, u1, L, zb, zt)
                else:
                    self.hall_side(f, u0, u1, L, zb, zt, side)
        a, bb, c, d = gable(m, poly, zt, HALL_RIDGE, "coral_roof_hall", "coral_white", ov=0.6)
        # louvre bands under the ridge on both gable ends / 妻面の棟下のガラリ帯
        ax = (bb - a) / np.linalg.norm(bb - a)
        for p, q, nrm in ((d, a, -ax), (bb, c, ax)):
            W = np.linalg.norm(q - p)
            mid = (p + q) / 2
            t = (q - p) / W
            z0b, z1b = zt + 2.2, zt + 3.4
            half = W / 2 * (1 - (z1b - zt) / (HALL_RIDGE - zt)) - 0.6
            if half > 1:
                e0, e1 = mid - t * half, mid + t * half
                o = nrm * 0.05
                m.quad("coral_louvre", (*(e0 + o), z0b), (*(e1 + o), z0b), (*(e1 + o), z1b), (*(e0 + o), z1b),
                       (0, 0), (2 * half / 1.5, 0), (2 * half / 1.5, 1), (0, 1))

    def hall_side(self, f, u0, u1, L, zb, zt, side):
        """Off-white walls, two rows of high windows with blue sill and lintel
        bands; the SE gable's tall green glazing and door; the blue NE corner
        with the yellow crest band. / 白い壁と 2 段の高窓（青い窓台・まぐさ帯）。"""
        rows = [(4.6, 5.9), (8.4, 9.7)]
        cs = [c for c in self.bay_centres(L, 4.2) if u0 + 0.2 <= c - 1.3 and c + 1.3 <= u1 - 0.2]
        opens = [(c - 1.3, c + 1.3, z0, z1) for z0, z1 in rows for c in cs if z0 > zb + 0.3]
        tall = []
        if side == "E" and zb <= Z0 + 0.01:
            # SE gable end: a pair of tall green glazed panels and a door / 南東妻: 縦長ガラス 2 枚と扉
            c = L * 0.62
            tall = [(c - 2.9, c - 0.4, Z0 + 0.6, 8.0), (c + 0.4, c + 2.9, Z0 + 0.6, 8.0)]
            opens = [o for o in opens if all(o[1] < t[0] - 0.3 or o[0] > t[1] + 0.3 for t in tall)]
            dc = L * 0.35
            tall.append((dc - 0.9, dc + 0.9, Z0, Z0 + 2.4))
        wall_openings(f, "coral_white", u0, u1, zb, zt, opens + tall, 2.0)
        for o in opens:
            opening(f, *o, "coral_white", "coral_glass_dk", 0.15, sill=False)
        for o in tall:
            g = "coral_door" if o[3] < 3 else "coral_glass_dk"
            opening(f, *o, "coral_white", g, 0.2, tu=1.25 if g != "coral_door" else None, sill=False)
        for z0, z1 in rows:
            if z0 > zb + 0.3:
                f.block("coral_blue", u0, u1, 0.0, 0.08, z0 - 0.3, z0, ends=False)
                f.block("coral_blue", u0, u1, 0.0, 0.08, z1, z1 + 0.25, ends=False)
        f.block("coral_white", u0, u1, 0.0, 0.12, zt - 0.35, zt, ends=False)
        if side == "N":
            # the blue NE corner with the yellow crest band (SV Feb 2018) / 青い北東角と黄色い校章帯
            north_e = float(f.t @ self.axes("H")[0]) < 0      # the edge runs towards -u: its start is east
            a, b = (0.0, 9.0) if north_e else (L - 9.0, L)
            a, b = max(u0, a), min(u1, b)
            if b > a + 1.0 and zb <= Z0 + 0.01:
                f.block("coral_blue", a, b, 0.0, 0.1, Z0, zt, top=True, bottom=False)
                f.block("coral_yellow", a, b, 0.0, 0.18, 6.4, 8.2, top=True, bottom=True)
                cu = (a + b) / 2
                f.m.quad("coral_crest", f.P(a + 0.4, 0.19, 6.5), f.P(a + 2.0, 0.19, 6.5), f.P(a + 2.0, 0.19, 8.1),
                         f.P(a + 0.4, 0.19, 8.1), (0, 0), (1, 0), (1, 1), (0, 1))
                g = text_poly("CORAL PRIMARY", 0.55, "DejaVuSans-Bold.ttf")
                glyphs_on(f.m, f, g, cu + 1.0, 7.0, 0.2, "coral_blue", 0.55)

    def hall_entrance_side(self, f, u0, u1, L, zb, zt):
        """The hall's SW flank on the drop-off (SV Sep 2016): blue square
        columns and a white canopy on the ground floor, a white band with
        small square windows, a blue fascia band, the dark recessed top storey.
        ホールの南西面: 青い角柱と白い庇、小窓の白い帯、青い鼻隠し、暗い最上階。"""
        z1, z2, z3 = 4.3, 8.0, 8.6
        if zb > z3:
            f.face("coral_white", u0, u1, zb, zt, 0.0, 2.0)
            return
        if zb < z1 - 0.1:
            cs = [c for c in self.bay_centres(L, 3.4) if u0 + 0.2 <= c - 1.2 and c + 1.2 <= u1 - 0.2]
            opens = [(c - 1.2, c + 1.2, Z0 + 0.8, Z0 + 2.9) for c in cs]
            wall_openings(f, "coral_white", u0, u1, zb, z1, opens, 2.0)
            for o in opens:
                opening(f, *o, "coral_white", "coral_glass_gf", 0.15)
            for c in self.bay_centres(L, 5.0, 0.0):
                if u0 + 0.4 < c < u1 - 0.4:
                    f.block("coral_blue", c - 0.3, c + 0.3, 1.3, 1.9, Z0, 3.7, top=False, bottom=False)
            f.block("coral_white", u0, u1, 0.0, 2.2, 3.7, 3.95, top=True, bottom=True)
        lo = max(zb, z1)
        cs = [c for c in self.bay_centres(L, 2.6) if u0 + 0.2 <= c - 0.5 and c + 0.5 <= u1 - 0.2]
        opens = [(c - 0.5, c + 0.5, 5.6, 6.6) for c in cs] if lo < 5.4 else []
        wall_openings(f, "coral_white", u0, u1, lo, z2, opens, 2.0)
        for o in opens:
            opening(f, *o, "coral_white", "coral_glass_dk", 0.12, sill=False)
        f.block("coral_blue", u0, u1, 0.0, 0.25, z2, z3, top=True, bottom=True, ends=False)
        # the dark recessed top storey (stops 0.25 m short of a corner) / 暗い後退した最上階
        if u0 < 0.01:
            f.face("coral_white", u0, u0 + 0.25, z3, zt - 0.5, 0.0)
            u0 += 0.25
        if u1 > L - 0.01:
            f.face("coral_white", u1 - 0.25, u1, z3, zt - 0.5, 0.0)
            u1 -= 0.25
        tiled_face(f, "coral_glass_dk", u0, u1, z3, zt - 0.5, -0.8, 2.0, zt - 0.5 - z3, v0=z3)
        f.hplane("coral_white", u0, u1, -0.8, 0.0, z3, True)
        f.hplane("coral_orange", u0, u1, -0.8, 0.0, zt - 0.5, False)
        f.side("coral_white", u0, -0.8, 0.0, z3, zt - 0.5, 1)
        f.side("coral_white", u1, -0.8, 0.0, z3, zt - 0.5, -1)
        f.face("coral_blue", u0, u1, zt - 0.5, zt, 0.0)

    # -- entrance / 玄関
    def porch_frame(self, m):
        """The yellow porch frame with the school's name in blue on its fascia,
        blue columns, a white flat canopy. / 黄色い玄関枠と校名、青い柱、白い庇。"""
        a, b, c, d = rect_axes(self.porch)
        hall = self.blocks.get("hall", {}).get("poly")
        # the front: the long side farther from the hall (CCW: outward on the right)
        # 正面はホールから遠い長辺（反時計回り: 右が外）
        fronts = [(a, b, d), (c, d, b)]
        if hall is not None:
            fronts.sort(key=lambda s_: -hall.distance(Point(*((s_[0] + s_[1]) / 2))))
        p0, p1, back = fronts[0]
        W = float(np.linalg.norm(p1 - p0))
        t = (p1 - p0) / W
        n = np.array([t[1], -t[0]])
        depth = abs(float((back - p0) @ n))
        f = B.Facade(m, p0, p1, n)
        for u in (0.0, W - 0.8):
            f.block("coral_yellow", u, u + 0.8, -0.8, 0.0, Z0, 4.7, top=True, bottom=False)
        f.block("coral_yellow", 0.8, W - 0.8, -0.8, 0.0, 3.6, 4.7, top=True, bottom=True, ends=False)
        f.block("coral_white", 0.8, W - 0.8, -depth - 0.2, -0.8, 3.5, 3.7, top=True, bottom=True, ends=True)
        for u in (2.4, W - 2.4):
            f.block("coral_blue", u - 0.25, u + 0.25, -depth + 0.6, -depth + 1.1, Z0, 3.5, top=False, bottom=False)
        g = text_poly("CORAL PRIMARY SCHOOL", 0.42, "DejaVuSans-Bold.ttf")
        glyphs_on(m, f, g, W / 2, 4.15, 0.03, "coral_blue", 0.42)

    def drum(self, m):
        """The yellow drum stair tower: two small windows per storey, the blue
        cap ring. / 黄色い円筒の階段塔と青い頂部リング。"""
        cx, cy = self.drum_c
        r = self.drum_r
        top = Z0 + 3 * F3 + 1.6
        m.cylinder("coral_yellow", cx, cy, Z0, top - 0.7, r, seg=20, top=False)
        m.cylinder("coral_blue", cx, cy, top - 0.7, top, r + 0.12, seg=20, top=True)
        m.flat("coral_blue", Point(cx, cy).buffer(r + 0.12, 20).difference(Point(cx, cy).buffer(r, 20)),
               top - 0.7, 1.0, up=False)
        # windows on the side away from the hall / ホールと反対側の窓
        hall = self.blocks.get("hall", {}).get("poly")
        away = 0.0
        if hall is not None:
            q = hall.exterior.interpolate(hall.exterior.project(Point(cx, cy)))
            away = math.atan2(cy - q.y, cx - q.x)
        for s in range(3):
            z0 = Z0 + s * F3 + 1.0
            for da in (-0.32, 0.32):
                ang = away + da
                nrm = np.array([math.cos(ang), math.sin(ang)])
                t = np.array([-nrm[1], nrm[0]])
                c = np.array([cx, cy]) + nrm * (r + 0.02)
                a, b = c - t * 0.35, c + t * 0.35
                m.quad("coral_glass_dk", (*a, z0), (*b, z0), (*b, z0 + 1.3), (*a, z0 + 1.3))

    # -- grounds / 敷地
    def site_fence(self, m):
        """Dark green chain-link on a low kerb with a clipped hedge inside, open
        at the gates; the tall ball-court fence along the south road; the store's
        arched white gates. / 縁石上の濃緑の金網フェンスと生垣、球技場の高いフェンス。"""
        ring = self.ring_in(0.5)
        cut = self.no_go.buffer(1.0)
        ped = Point(self.ped_gate)
        pg = ring.interpolate(ring.project(ped))
        line = ring.difference(cut).difference(Point(pg.x, pg.y).buffer(1.8))
        for g in self.gates:
            line = line.difference(g.buffer(1.5))
        u0, u1 = self.T["ball_fence_u"]
        for g in getattr(line, "geoms", [line]):
            if g.geom_type != "LineString" or g.length < 1.0:
                continue
            c = list(g.coords)
            for a, b in zip(c, c[1:]):
                mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                u, v = self.uv("S", *mid)
                tall = u0 <= u <= u1 and v < -55
                h = 4.6 if tall else 1.8
                chainlink(m, "coral_fence", a, b, Z0 + 0.2, Z0 + h)
                two_sided_wall(m, "coral_fence_post", a, b, Z0 + 0.2, Z0 + 0.26, gap=0.04)
                two_sided_wall(m, "coral_fence_post", a, b, Z0 + h - 0.05, Z0 + h + 0.01, gap=0.04)
            for k in range(int(g.length // 3.0) + 1):
                q = g.interpolate(min(g.length, k * 3.0))
                u, v = self.uv("S", q.x, q.y)
                h = 4.6 if (u0 <= u <= u1 and v < -55) else 1.8
                m.box("coral_fence_post", q.x, q.y, Z0, Z0 + h, 0.06, 0.06)
            m.prism("concrete", g.buffer(0.12, cap_style="flat"), Z0 - 0.05, Z0 + 0.2, top=True)
            hedge = g.offset_curve(-0.8) if g.length > 2 else None
            if hedge is not None and not hedge.is_empty and hedge.geom_type == "LineString":
                hp = hedge.buffer(0.4, cap_style="flat").difference(self.footprint.buffer(0.5)) \
                    .difference(unary_union([c for c, _ in self.courts.values()]).buffer(0.3))
                m.prism("leaves_dark", hp, Z0, Z0 + 1.0, top=True)
        # pedestrian gate leaves on St 51 east (white steel, open) / 歩行者門
        self.steel_leaves(m, (pg.x, pg.y), ring, 1.8, 1.6)
        # the store's arched double gates on the south road / 倉庫脇のアーチ門
        st = self.blocks.get("store")
        if st is not None:
            q = ring.interpolate(ring.project(st["poly"].centroid))
            self.steel_leaves(m, (q.x, q.y), ring, 2.0, 2.2, arch=True)

    def steel_leaves(self, m, c, ring, half, h, arch=False):
        """White steel gate leaves across an opening of 2 x half (bars, rails,
        an arched top rail). / 白い鋼製の門扉。"""
        s = ring.project(Point(c))
        a = ring.interpolate(max(0.0, s - half))
        b = ring.interpolate(min(ring.length, s + half))
        a, b = np.array([a.x, a.y]), np.array([b.x, b.y])
        L = float(np.linalg.norm(b - a))
        if L < 0.5:
            return
        t = (b - a) / L
        ang = math.atan2(t[1], t[0])
        for k in range(int(L / 0.14) + 1):
            p = a + t * min(L, k * 0.14)
            x = min(L, k * 0.14)
            hh = h + (0.35 * math.sin(math.pi * ((x % (L / 2)) / (L / 2))) if arch else 0.0)
            m.box("coral_rail", p[0], p[1], Z0 + 0.08, Z0 + hh, 0.035, 0.035)
        for zz in (Z0 + 0.1, Z0 + h * 0.5, Z0 + h):
            mid = (a + b) / 2
            m.box("coral_rail", mid[0], mid[1], zz, zz + 0.06, L, 0.05, ang)
        for p in (a, b):
            m.box("coral_white", p[0], p[1], Z0, Z0 + h + 0.4, 0.35, 0.35)

    def vehicle_gate(self, m, g):
        """White steel sliding gates parked along the fence and the yellow
        guard house (SV Sep 2016). / 白い引き戸の門と黄色い守衛所。"""
        a, b = np.array(g.coords[0]), np.array(g.coords[-1])
        if self.uv("S", *a)[0] > self.uv("S", *b)[0]:
            a, b = b, a                       # a: the west pier, the guard house beside it / a は西側
        t = (b - a) / max(np.linalg.norm(b - a), 1e-6)
        ang = math.atan2(t[1], t[0])
        inward = np.array([-t[1], t[0]])
        cen = np.array(self.lot.centroid.coords[0])
        if float((cen - (a + b) / 2) @ inward) < 0:
            inward = -inward
        for s, p in ((-1, a), (1, b)):
            p = p + t * s * 1.6
            m.box("coral_white", p[0], p[1], Z0, Z0 + 2.2, 0.45, 0.45)
            # the parked leaf along the fence, 0.4 m inside / 開いた門扉
            q0 = p + t * s * 0.3 + inward * 0.4
            for k in range(30):
                q = q0 + t * s * k * 0.15
                m.box("coral_rail", q[0], q[1], Z0 + 0.1, Z0 + 1.9, 0.035, 0.035)
            mid = q0 + t * s * 2.2
            for zz in (Z0 + 0.1, Z0 + 1.0, Z0 + 1.9):
                m.box("coral_rail", mid[0], mid[1], zz, zz + 0.06, 4.4, 0.05, ang)
        # the school's name board beside the gate, facing the road (drawn plaque)
        # 門脇の校名板（道路向き、描画テクスチャ）
        q = a - t * 2.8 + inward * 0.7
        m.box("coral_frame_dk", *(q - t * 1.2), Z0, Z0 + 2.1, 0.08, 0.08)
        m.box("coral_frame_dk", *(q + t * 1.2), Z0, Z0 + 2.1, 0.08, 0.08)
        m.box("coral_white", q[0], q[1], Z0 + 0.9, Z0 + 2.1, 2.6, 0.1, ang)
        o = -inward * 0.056
        p0, p1 = q - t * 1.25 + o, q + t * 1.25 + o
        dd = p1 - p0
        if float(np.array([dd[1], -dd[0]]) @ -inward) < 0:
            p0, p1 = p1, p0                    # the board's text reads from the road / 道路側から読める向き
        m.quad("coral_plaque", (*p0, Z0 + 0.95), (*p1, Z0 + 0.95), (*p1, Z0 + 2.05), (*p0, Z0 + 2.05))
        # guard house beside the west pier, inside / 守衛所
        hx, hy = a - t * 4.6 + inward * 3.2
        hp = Polygon([(hx + dx * t[0] + dy * inward[0], hy + dx * t[1] + dy * inward[1])
                      for dx, dy in ((-1.6, -1.4), (1.6, -1.4), (1.6, 1.4), (-1.6, 1.4))])
        if hp.distance(self.no_go) > 0.6 and not hp.intersects(self.footprint):
            hp = orient(hp, 1.0)
            for e0, e1, L, n in B.edges_of(hp):
                f = B.Facade(m, e0, e1, n)
                c = L / 2
                opens = [(c - 0.7, c + 0.7, Z0 + 1.0, Z0 + 2.1)]
                wall_openings(f, "coral_yellow", 0.0, L, Z0, Z0 + 2.8, opens, 2.0)
                opening(f, *opens[0], "coral_yellow", "coral_glass_dk", 0.1)
            m.prism("coral_white", hp.buffer(0.35, join_style="mitre"), Z0 + 2.8, Z0 + 3.05, top=True, bottom=True)

    def walkway_e(self, m):
        """The covered walkway inside the fence along St 51 east: grey metal
        roof on thin posts (SV Feb 2018). / St 51 沿いの屋根付き通路。"""
        v0, v1 = self.T["walkway_e_v"]
        ring = self.ring_in(3.2)
        zone = Polygon([self.at("S", 70, v0), self.at("S", 130, v0), self.at("S", 130, v1), self.at("S", 70, v1)])
        g = ring.intersection(zone)
        g = max([q for q in getattr(g, "geoms", [g]) if q.geom_type == "LineString"], key=lambda q: q.length,
                default=None)
        if g is None:
            return
        roof = g.buffer(1.4, cap_style="flat").difference(self.no_go.buffer(0.6)).difference(self.footprint)
        m.flat("coral_walkway", roof, Z0 + 2.75, 2.0)
        m.flat("coral_walkway", roof, Z0 + 2.7, 2.0, up=False)
        for p in polys(roof):
            m.ring_walls("coral_walkway", p.exterior, Z0 + 2.6, Z0 + 2.75)
        for k in range(int(g.length // 3.0) + 1):
            q = g.interpolate(min(g.length, k * 3.0))
            for s in (-1.1, 1.1):
                d = g.interpolate(min(g.length, k * 3.0 + 0.1))
                tx, ty = d.x - q.x, d.y - q.y
                ln = math.hypot(tx, ty) or 1.0
                px, py = q.x - ty / ln * s, q.y + tx / ln * s
                m.box("metal_grey", px, py, Z0, Z0 + 2.7, 0.08, 0.08)
        m.flat("concrete_path", g.buffer(1.3, cap_style="flat").difference(self.no_go.buffer(0.3))
               .difference(self.footprint), Z0 + 0.02, 2.0)

    def goals(self, m):
        """Football goals at both ends of the field (white posts, dark net). / ゴール。"""
        for (x, y), t in self.goal_spots():
            ang = math.atan2(t[1], t[0])
            for s in (-1, 1):
                m.cylinder("goal_white", x + t[0] * s * 2.5, y + t[1] * s * 2.5, Z0, Z0 + 2.2, 0.06, seg=6)
            m.box("goal_white", x, y, Z0 + 2.12, Z0 + 2.24, 5.0, 0.1, ang)

    def hoops(self, m):
        """Basketball posts at both ends of the red court: post, arm, white
        board facing the court, orange rim. / 赤いコートのバスケットゴール。"""
        a, b, c, d = rect_axes(self.courts["red_bb"][0])
        cen = (a + b + c + d) / 4
        for e in ((a + d) / 2, (b + c) / 2):
            inw = (cen - e) / np.linalg.norm(cen - e)
            side = np.array([-inw[1], inw[0]])
            ang = math.atan2(side[1], side[0])
            p = e + inw * 0.3
            m.box("metal_grey", p[0], p[1], Z0, Z0 + 3.4, 0.16, 0.16, ang)
            arm = e + inw * 0.85
            m.box("metal_grey", arm[0], arm[1], Z0 + 3.3, Z0 + 3.42, 0.12, 1.2, ang)
            bd = e + inw * 1.45
            m.box("backboard", bd[0], bd[1], Z0 + 2.9, Z0 + 3.95, 1.8, 0.05, ang)
            rim = e + inw * 1.75
            m.cylinder("paint_orange", rim[0], rim[1], Z0 + 3.05, Z0 + 3.08, 0.23, seg=10, top=False)

    def flagpole(self, m):
        x, y = self.flag
        m.box("concrete", x, y, Z0, Z0 + 0.3, 1.2, 1.2)
        m.cylinder("metal_grey", x, y, Z0 + 0.3, Z0 + 10.0, 0.06, seg=8, top=True)
        # the flag, both sides 6 mm apart / 国旗（両面）
        d = np.array([1.0, 0.0])
        two_sided_wall(m, "coral_flag", (x + 0.07, y), (x + 1.6, y), Z0 + 8.9, Z0 + 9.9)
        del d

    def fitness_corner(self, m):
        """Fitness corner (OSM 763918055): a few steel bars on rubber. / 健康器具。"""
        x, y = self.fitness
        m.flat("rubber", Point(x, y).buffer(3.2, 8).difference(self.footprint), Z0 + 0.02, 2.0)
        for dx, dy in ((-1.5, 0), (0, 1.2), (1.5, 0)):
            m.box("metal_grey", x + dx, y + dy, Z0, Z0 + 1.8, 0.08, 0.08)
            m.box("metal_grey", x + dx + 0.8, y + dy, Z0, Z0 + 1.8, 0.08, 0.08)
            m.box("metal_grey", x + dx + 0.4, y + dy, Z0 + 1.75, Z0 + 1.85, 0.9, 0.06)

    def plant(self, m, poly, z):
        """Roof plant boxes on the west wing. / 屋上の設備。"""
        c = poly.centroid
        for dx, dy, sx, sy in ((-3, 2, 3.0, 2.0), (3, -2, 2.0, 2.0)):
            p = Point(c.x + dx, c.y + dy)
            if poly.buffer(-1.5).contains(p):
                m.box("tank", p.x, p.y, z, z + 1.6, sx, sy)

    def spine_vents(self, m, poly, z):
        """Small vents along the spine's roof (OneMap 3-D). / 渡り棟屋上の換気口。"""
        a, b, c, d = rect_axes(poly)
        ax = (b - a) / np.linalg.norm(b - a)
        sd = (d - a) / np.linalg.norm(d - a)
        Lr = np.linalg.norm(b - a)
        for k in range(int(Lr // 8)):
            p = a + ax * (4 + k * 8) + sd * np.linalg.norm(d - a) / 2
            if poly.buffer(-1.0).contains(Point(*p)):
                m.box("tank", p[0], p[1], z, z + 0.9, 1.2, 1.2)

    def trees(self, m):
        """Rain trees along the north road and St 51 east inside the fence,
        the north courtyard row, palms by the entrance. / 樹木。"""
        import props
        spots = [(self.X(q), "rain", 0.8) for q in self.T["trees"]["courtyard"]]
        spots += [(self.X(q), "palm", 0.9) for q in self.T["trees"]["palms"]]
        ring = self.ring_in(5.0)
        for k in range(int(ring.length // 12.0)):
            q = ring.interpolate(k * 12.0 + 4.0)
            u, v = self.uv("S", q.x, q.y)
            if v > 55 or u > 86:                  # the north road and St 51 east / 北の道と東の St 51
                spots.append(((q.x, q.y), "rain" if k % 3 else "angsana", 0.9))
        busy = unary_union([self.footprint.buffer(2.5), self.field.buffer(1.0)]
                           + [c.buffer(1.5) for c, _ in self.courts.values()])
        for (x, y), kind, s in spots:
            p = Point(x, y)
            R = {"rain": 6.0, "angsana": 4.2, "palm": 3.2}[kind] * s
            if busy.contains(p) or self.no_go.distance(p) < R + 0.6:
                continue
            props.Props.tree_mesh(None, m, x, y, kind, s, False)


def write_textures(tex_dir):
    """The school's drawn textures (procedural; colours from description.md,
    nothing taken from imagery): renders, framed glass, ribbon glazing, the
    corridor wall, the grid glazing, roofs, louvres, the courts (red with blue
    keys, the parade square), the painted strip, fence, the drawn crest
    stand-in (blue, yellow, white; not the real crest), the flag.
    学校の描画テクスチャ（description.md の色、画像は使わない）。"""
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

    YEL, BLU, WHT = (200, 185, 115), (25, 90, 160), (205, 206, 202)
    save("coral_yellow", X.paint(512, YEL, 0.06), 0.85, 0.0, 1.0)
    save("coral_blue", X.paint(512, BLU, 0.06), 0.85, 0.0, 1.0)
    save("coral_white", X.paint(512, WHT, 0.07), 0.85, 0.0, 1.0)
    save("coral_green", X.paint(256, (40, 80, 60), 0.05), 0.7)
    save("coral_orange", X.paint(256, (225, 140, 60), 0.05), 0.85)

    def framed(frame, glass0, glass1, panes=2, n=256, transom=0.36):
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

    G0, G1 = (70, 100, 92), (120, 150, 140)          # green-tinted glass / 緑がかったガラス
    save("coral_glass_yf", framed((200, 170, 50), G0, G1, 2), 0.1, 0.2)
    save("coral_glass_gf", framed((40, 80, 60), G0, G1, 3), 0.1, 0.2)
    save("coral_glass_dk", framed((45, 50, 55), (40, 55, 60), (85, 110, 105), 2), 0.1, 0.2)
    save("coral_ribbon", framed((45, 50, 55), G0, G1, 1, transom=0.3), 0.1, 0.2)
    # the grid glazing of the stair towers: square panes, white edges / 階段室の格子ガラス
    n = 256
    im = Image.new("RGB", (n, n), (60, 80, 85))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, n - 1, n - 1], outline=(40, 45, 50), width=10)
    d.rectangle([12, 12, n - 13, n - 13], fill=(95, 125, 120))
    d.line([(0, n // 2), (n, n // 2)], fill=(40, 45, 50), width=6)
    save("coral_grid", im, 0.1, 0.2)
    # the corridor wall: yellow, a door and a window per 3.6 m module / 廊下壁: 扉と窓
    W, H = 512, 448
    im = X.paint(512, (225, 200, 115), 0.05).resize((W, H))
    d = ImageDraw.Draw(im)
    d.rectangle([30, 120, 150, H - 1], fill=(95, 70, 50))
    d.rectangle([38, 128, 142, 260], fill=(70, 95, 90))
    d.rectangle([210, 150, 480, 330], fill=(45, 50, 55))
    d.rectangle([218, 158, 472, 322], fill=(110, 140, 130))
    d.line([(345, 150), (345, 330)], fill=(45, 50, 55), width=6)
    d.rectangle([0, H - 30, W, H], fill=(200, 175, 100))
    save("coral_corr", im, 0.8)
    save("coral_corr_wall", X.paint(256, (225, 200, 115), 0.05), 0.85)
    save("coral_roof", X.stripes(256, (160, 160, 155), (182, 182, 176), 20, vertical=True), 0.5, 0.5, 1.5)
    save("coral_roof_hall", X.stripes(256, (142, 142, 137), (160, 160, 154), 20, vertical=True), 0.5, 0.5, 1.5)
    save("coral_louvre", X.stripes(256, (45, 48, 50), (90, 92, 95), 14), 0.6, 0.3, 1.5)
    save("coral_door", framed((60, 60, 62), (75, 60, 45), (95, 75, 55), 2, transom=0), 0.5)
    save("coral_rail", X.paint(64, (232, 232, 228), 0.02), 0.4, 0.5)
    save("coral_fence", X.paint(64, (40, 70, 50), 0.05), 0.6, 0.3)
    save("coral_fence_post", X.paint(64, (35, 60, 45), 0.03), 0.5, 0.5)
    save("coral_frame_y", X.paint(64, (200, 170, 50), 0.03), 0.5, 0.2)
    save("coral_frame_g", X.paint(64, (40, 80, 60), 0.03), 0.5, 0.2)
    save("coral_frame_dk", X.paint(64, (48, 52, 56), 0.03), 0.5, 0.3)
    save("coral_pipe", X.paint(64, (150, 152, 150), 0.03), 0.5, 0.3)
    save("coral_brick", X.bricks(512, (112, 72, 64)), 0.85, 0.0, 1.5)
    save("coral_walkway", X.stripes(128, (140, 145, 145), (158, 163, 163), 8, vertical=True), 0.5, 0.6)

    # courts / コート
    def court_bb():
        # red area 27 x 20 m, the court 24.5 x 16 m, blue keys / 赤いコートと青いキー
        k = 30
        Wc, Hc = 27 * k, 20 * k
        im = Image.fromarray(X.tint(X.noise(Wc)[:Hc, :Wc], np.array((122, 48, 40)), np.array((140, 58, 48))))
        d = ImageDraw.Draw(im)
        P = lambda x, y: (Wc / 2 + x * k, Hc / 2 - y * k)
        lw = 4
        hl, hw = 12.25, 8.0
        for s in (-1, 1):
            x0, x1 = sorted((s * hl, s * (hl - 5.8)))
            d.rectangle([P(x0, 2.45), P(x1, -2.45)], fill=(40, 80, 160))
        d.rectangle([P(-hl, hw), P(hl, -hw)], outline=(245, 245, 240), width=lw)
        d.line([P(0, -hw), P(0, hw)], fill=(245, 245, 240), width=lw)
        d.ellipse([P(-1.8, 1.8), P(1.8, -1.8)], outline=(245, 245, 240), width=lw)
        for s in (-1, 1):
            x0, x1 = sorted((s * hl, s * (hl - 5.8)))
            d.rectangle([P(x0, 2.45), P(x1, -2.45)], outline=(245, 245, 240), width=lw)
            bx = s * (hl - 1.575)
            a0, a1 = (-68, 68) if s < 0 else (112, 248)
            d.arc([P(bx - 6.75, 6.75), P(bx + 6.75, -6.75)], a0, a1, fill=(245, 245, 240), width=lw)
        return im
    save("coral_court_bb", court_bb(), 0.75, 0.0, 0.6)

    def parade():
        # grey concrete 33 x 31.5 m with two netball and volleyball courts / 灰色の広場とライン
        k = 24
        Wc, Hc = int(33.2 * k), int(31.5 * k)
        im = X.concrete(max(Wc, Hc), (150, 150, 146), (170, 170, 165), joints=0).crop((0, 0, Wc, Hc))
        d = ImageDraw.Draw(im)
        P = lambda x, y: (Wc / 2 + x * k, Hc / 2 - y * k)
        lc = (240, 240, 236)
        for cy in (-7.6, 7.6):
            d.rectangle([P(-15.25, cy + 7.0), P(15.25, cy - 7.0)], outline=lc, width=4)
            for s in (-1, 1):
                d.line([P(s * 5.08, cy - 7.0), P(s * 5.08, cy + 7.0)], fill=lc, width=4)
                d.arc([P(s * 15.25 - 4.9, cy + 4.9), P(s * 15.25 + 4.9, cy - 4.9)],
                      90 if s > 0 else -90, 270 if s > 0 else 90, fill=lc, width=4)
        d.rectangle([P(-9, 4.5), P(9, -4.5)], outline=(250, 210, 60), width=4)
        d.line([P(0, -4.5), P(0, 4.5)], fill=(250, 210, 60), width=4)
        return im
    save("coral_parade", parade(), 0.85, 0.0, 0.6)
    # the painted strip at the spine's foot: blue with red markings / 渡り棟脚元の塗装帯
    im = Image.fromarray(X.tint(X.noise(512), np.array((40, 95, 160)), np.array((60, 120, 185))))
    d = ImageDraw.Draw(im)
    for k2 in range(6):
        y = 40 + k2 * 80
        d.rectangle([60, y, 450, y + 18], fill=(185, 60, 55))
        d.ellipse([220, y + 25, 290, y + 70], outline=(240, 240, 235), width=5)
    save("coral_strip", im, 0.8)
    # the crest stand-in: a shield in the school colours / 校章の代用（学校の色の盾）
    n = 512
    im = Image.new("RGB", (n, n), YEL)
    d = ImageDraw.Draw(im)
    d.polygon([(n * .18, n * .1), (n * .82, n * .1), (n * .82, n * .55), (n * .5, n * .92), (n * .18, n * .55)],
              fill=(240, 240, 236), outline=BLU)
    for k2 in range(3):
        y = n * (0.45 + k2 * 0.1)
        d.arc([n * .22, y - n * .05, n * .78, y + n * .05], 0, 180, fill=BLU, width=n // 40)
    d.polygon([(n * .5, n * .16), (n * .56, n * .3), (n * .7, n * .3), (n * .59, n * .38), (n * .63, n * .52),
               (n * .5, n * .43), (n * .37, n * .52), (n * .41, n * .38), (n * .3, n * .3), (n * .44, n * .3)],
              fill=(230, 190, 40))
    save("coral_crest", im, 0.6)
    # the name board: the school's names (OSM name, name:ms, name:zh) and address, drawn
    # 校名板（OSM の名称と住所を描く）
    from PIL import ImageFont
    im = Image.new("RGB", (1024, 432), (245, 245, 240))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 1023, 431], outline=BLU, width=14)
    d.rectangle([14, 14, 1009, 70], fill=YEL)
    fb = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    d.text((40, 95), "CORAL PRIMARY SCHOOL", fill=BLU, font=ImageFont.truetype(fb, 66))
    d.text((40, 185), "SEKOLAH RENDAH CORAL", fill=(40, 40, 45), font=ImageFont.truetype(fb, 40))
    cjk = "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc"
    if os.path.exists(cjk):
        d.text((40, 240), "云海小学", fill=(40, 40, 45), font=ImageFont.truetype(cjk, 60))
    d.text((40, 340), "20 PASIR RIS STREET 51  S(518902)", fill=(40, 40, 45), font=ImageFont.truetype(fb, 34))
    crest = Image.open(os.path.join(tex_dir, "coral_crest.png")).resize((200, 200))
    im.paste(crest, (790, 160))
    save("coral_plaque", im, 0.6)
    # Singapore's flag (red over white, crescent and stars) / シンガポール国旗
    im = Image.new("RGB", (300, 200), (238, 36, 50))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 100, 300, 200], fill=(245, 245, 245))
    d.ellipse([25, 18, 95, 88], fill=(245, 245, 245))
    d.ellipse([42, 18, 112, 88], fill=(238, 36, 50))
    for k2 in range(5):
        a = -math.pi / 2 + k2 * 2 * math.pi / 5
        x, y = 100 + 18 * math.cos(a), 52 + 18 * math.sin(a)
        d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(245, 245, 245))
    save("coral_flag", im, 0.7)
    return out
