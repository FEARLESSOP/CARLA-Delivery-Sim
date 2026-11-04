"""Singapore-style buildings with facade depth, from OSM footprints.

Nothing of a building stands out more than FACADE_DEPTH (1.0 m) beyond its
footprint; layout.py keeps every footprint far enough from the roads for
that. Styles (from the OSM tags, floor counts and footprint):

  hdb       public housing blocks (apartments, 4+ floors): an open void
            deck on columns at the ground; the corridor side with recessed
            common corridors behind parapet walls, unit doors and windows;
            the window side with recessed windows, AC ledges, planter
            boxes and a slab ledge at every floor; end walls with the block
            number; a flat roof with the lift motor room and water tank or
            a pitched roof frame; pastel colour themes shared by a precinct
  condo     private apartments: glass-balustrade balconies, dark glazing,
            white frames, a roof crown
  carpark   multi-storey car parks: open decks between parapet bands,
            columns, some with a roof garden
  house     terraces, semi-detached and detached houses: 2-3 floors,
            pitched tiled roofs, a car porch, framed windows
  retail    shopping centres: glass shopfronts under a canopy, cladding
            bands and glazed strips above
  school    corridor blocks with coloured railings
  worship   church / temple: simplified pitched or hipped roofs
  low       anything else: a low pavilion with clerestory windows

シンガポール風の建物（奥行きのある外壁）。張り出しは建物外形から最大 1 m。
"""

import hashlib
import math
import random

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient

from mesh import Mesh, polys

FACADE_DEPTH = 1.0
FLOOR = 2.8
VOID = 3.6

HDB_THEMES = [   # (main wall, accent, second accent, roof frame) / HDB の配色
    ("paint_white", "paint_skyblue", "paint_navy", "roof_blue"),
    ("paint_cream", "paint_terracotta", "paint_brown", "roof_terracotta"),
    ("paint_white", "paint_teal", "paint_grey", "roof_teal"),
    ("paint_cream", "paint_mustard", "paint_maroon", "roof_terracotta"),
    ("paint_white", "paint_sage", "paint_green", "roof_green"),
    ("paint_palegrey", "paint_coral", "paint_white", "roof_grey"),
    ("paint_white", "paint_lavender", "paint_purple", "roof_grey"),
    ("paint_cream", "paint_peach", "paint_brown", "roof_terracotta"),
]


def seed_of(*keys):
    h = hashlib.md5(("|".join(str(k) for k in keys)).encode()).hexdigest()
    return int(h[:8], 16)


def levels_of(t, default):
    for k in ("building:levels", "levels"):
        v = t.get(k)
        if v:
            try:
                return max(1, int(round(float(v.split(";")[0]))))
            except ValueError:
                pass
    h = t.get("height")
    if h:
        try:
            return max(1, int(float(h.split()[0]) / 3.0))
        except ValueError:
            pass
    return default


def style_of(t, poly, levels):
    b = t.get("building", "yes")
    if b in ("house", "terrace", "semidetached_house", "detached", "bungalow"):
        return "house"
    if b == "parking" or t.get("parking") == "multi-storey" or t.get("amenity") == "parking":
        return "carpark"
    if b in ("retail", "commercial", "supermarket", "mall") or t.get("shop") == "mall":
        return "retail"
    if b in ("school", "kindergarten", "college", "university") or t.get("amenity") in ("school", "kindergarten"):
        return "school"
    if b in ("church", "temple", "mosque", "chapel") or t.get("amenity") == "place_of_worship":
        return "worship"
    if b in ("apartments", "residential", "yes") and levels >= 4:
        cond = t.get("condo") or t.get("residential") == "condominium" or \
            any(w in (t.get("name") or "").lower() for w in ("condo", "residences", "the ", "@"))
        return "condo" if cond else "hdb"
    if b in ("garage", "shed", "hut", "roof", "kiosk"):
        return "shed"
    return "low"


def edges_of(poly):
    """Outer ring edges, CCW: (a, b, length, outward normal). / 外周の辺。"""
    p = orient(poly, 1.0)
    c = list(p.exterior.coords)
    out = []
    for a, b in zip(c, c[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L < 0.05:
            continue
        out.append((np.array(a), np.array(b), L, np.array([dy / L, -dx / L])))
    return out


class Facade(object):
    """Geometry along one edge, in the edge's frame: u along the edge
    (0..L), w out from the wall (+ outwards), z up.
    辺に沿う外壁。u: 辺方向, w: 外向き, z: 上。"""

    def __init__(self, m, a, b, n):
        self.m, self.a = m, a
        self.L = float(np.linalg.norm(b - a))
        self.t = (b - a) / self.L
        self.n = n

    def P(self, u, w, z):
        q = self.a + self.t * u + self.n * w
        return (q[0], q[1], z)

    def face(self, mat, u0, u1, z0, z1, w=0.0, uvs=1.0):
        """A wall patch facing out, at offset w. / 外向きの壁面。"""
        if u1 - u0 < 1e-3 or z1 - z0 < 1e-3:
            return
        self.m.quad(mat, self.P(u0, w, z0), self.P(u1, w, z0), self.P(u1, w, z1), self.P(u0, w, z1),
                    (u0 / uvs, z0 / uvs), (u1 / uvs, z0 / uvs), (u1 / uvs, z1 / uvs), (u0 / uvs, z1 / uvs))

    def side(self, mat, u, w0, w1, z0, z1, facing):
        """A patch square to the wall at u, facing +t (facing=1) or -t. / 壁に直角な面。"""
        if w1 - w0 < 1e-3 or z1 - z0 < 1e-3:
            return
        if facing > 0:
            self.m.quad(mat, self.P(u, w1, z0), self.P(u, w0, z0), self.P(u, w0, z1), self.P(u, w1, z1))
        else:
            self.m.quad(mat, self.P(u, w0, z0), self.P(u, w1, z0), self.P(u, w1, z1), self.P(u, w0, z1))

    def hplane(self, mat, u0, u1, w0, w1, z, up=True):
        # t x n points down, so a->b->c->d (along t, then out along n) faces
        # down: the upward face is the reverse / t×n は下向きなので上向きは逆順
        a, b, c, d = self.P(u0, w0, z), self.P(u1, w0, z), self.P(u1, w1, z), self.P(u0, w1, z)
        if up:
            self.m.quad(mat, d, c, b, a)
        else:
            self.m.quad(mat, a, b, c, d)

    def block(self, mat, u0, u1, w0, w1, z0, z1, top=True, bottom=True, ends=True):
        """A box standing out from (or into) the wall. / 壁から張り出す箱。"""
        self.face(mat, u0, u1, z0, z1, w1)
        if top:
            self.hplane(mat, u0, u1, w0, w1, z1, True)
        if bottom:
            self.hplane(mat, u0, u1, w0, w1, z0, False)
        if ends:
            self.side(mat, u0, w0, w1, z0, z1, -1)
            self.side(mat, u1, w0, w1, z0, z1, 1)

    def window(self, u0, u1, z0, z1, wall_mat, glass="glass", frame="frame", depth=0.15, mullions=1):
        """An opening in the wall: reveals, the glass set back (its frames are
        in the glass texture, one texture tile per window), a sill ledge.
        窓: 見込み・奥のガラス（枠はテクスチャ）・窓台。"""
        d = -depth
        self.hplane(wall_mat, u0, u1, d, 0.0, z0, True)
        self.hplane(wall_mat, u0, u1, d, 0.0, z1, False)
        self.side(wall_mat, u0, d, 0.0, z0, z1, 1)
        self.side(wall_mat, u1, d, 0.0, z0, z1, -1)
        nm = glass if mullions <= 0 else glass + ("_2" if mullions == 1 else "_3")
        self.m.quad(nm, self.P(u0, d, z0), self.P(u1, d, z0), self.P(u1, d, z1), self.P(u0, d, z1),
                    (0, 0), (1, 0), (1, 1), (0, 1))
        # sill ledge: its top and front only / 窓台（上面と前面のみ）
        self.hplane("concrete", u0 - 0.05, u1 + 0.05, 0.0, 0.08, z0, True)
        self.face("concrete", u0 - 0.05, u1 + 0.05, z0 - 0.06, z0, 0.08)

    def wall_with_openings(self, mat, z0, z1, openings, uvs=1.0):
        """The wall patch between z0 and z1 with rectangular holes
        [(u0, u1, oz0, oz1)], drawn as the pieces around them. / 開口付きの壁。"""
        openings = sorted(openings)
        u = 0.0
        for (a, b, oz0, oz1) in openings:
            self.face(mat, u, a, z0, z1, 0.0, uvs)
            self.face(mat, a, b, z0, oz0, 0.0, uvs)
            self.face(mat, a, b, oz1, z1, 0.0, uvs)
            u = b
        self.face(mat, u, self.L, z0, z1, 0.0, uvs)


def bays(L, width, margin=0.6):
    n = int((L - 2 * margin) // width)
    if n <= 0:
        return []
    w = (L - 2 * margin) / n
    return [(margin + i * w, margin + (i + 1) * w) for i in range(n)]


# school accent colours where they could be found in writing (school crest /
# colours); others keep a neutral scheme. Not taken from imagery.
# 文献で確認できた学校の色（校章など）。他は中立の配色。
SCHOOL_COLOURS = {
    "dunman secondary school": ("paint_cream", "paint_brown"),
    "loyang view secondary school": ("paint_white", "paint_navy"),
    "greenview secondary school": ("paint_white", "paint_navy"),     # merged into Loyang View / 統合
}


class Builder(object):
    def __init__(self, layout, sites=None):
        self.L = layout
        self.sites = sites
        self.count = {}
        self.schools = [(p, (t.get("name") or "").lower()) for p, t in
                        layout.osm.areas(lambda t: t.get("amenity") in ("school", "kindergarten"))]
        # maps.json "road_style": "japan": Japanese styles, lots and terraces
        # (buildings_jp.py); every other map keeps the styles below
        # 日本の地図は buildings_jp の様式・敷地・造成（他の地図は従来どおり）
        self.jp = getattr(layout, "meta", {}).get("road_style") == "japan"
        self._jp = None

    def jp_styles(self):
        if self._jp is None:
            import buildings_jp
            self._jp = buildings_jp.JP(self)
        return self._jp

    def build(self, poly, t, idx):
        if self.jp:
            return self.jp_styles().build(poly, t, idx)
        levels_default = {"apartments": 12, "residential": 12, "house": 2, "terrace": 2,
                          "semidetached_house": 2, "parking": 6, "school": 4, "retail": 3, "commercial": 3}
        levels = levels_of(t, levels_default.get(t.get("building"), 2))
        st = style_of(t, poly, levels)
        self.count[st] = self.count.get(st, 0) + 1
        rng = random.Random(seed_of(idx, poly.centroid.x, poly.centroid.y))
        poly = orient(poly.simplify(0.25), 1.0)
        if poly.geom_type != "Polygon" or poly.area < 4.0:
            return None
        m = Mesh()
        fn = getattr(self, "b_" + st)
        fn(m, poly, t, levels, rng)
        return m

    def front_normal(self, poly):
        """The direction from the building to its nearest road. / 最寄りの道路の方向。"""
        c = poly.centroid
        p = self.L.carriageway.boundary.interpolate(self.L.carriageway.boundary.project(c))
        v = np.array([p.x - c.x, p.y - c.y])
        n = np.linalg.norm(v)
        return v / n if n > 1e-6 else np.array([0.0, -1.0])

    def theme(self, poly, style="hdb"):
        """(wall, accent, second accent, roof): the colours seen in Street
        View on the nearest observed block of the same kind (sitefeatures),
        else a pastel theme per 140 m precinct. Brick-red precincts get the
        red tiled roofs they have. / ストリートビューで見た最寄りの棟の配色。"""
        o = self.sites.colours(poly, style) if self.sites else None
        if o:
            wall, acc, acc2 = o["wall"], o["acc"], o["acc2"]
            roof = "roof_terracotta" if (wall == "paint_brickred" or acc2.startswith("roof_")) else \
                {"paint_royalblue": "roof_blue", "paint_teal": "roof_teal", "paint_maroon": "roof_brown"}.get(
                    acc, "roof_grey")
            if acc2.startswith("roof_"):
                acc2 = acc
            return wall, acc, acc2, roof
        c = poly.centroid
        k = seed_of("precinct", int(c.x // 140), int(c.y // 140))
        return HDB_THEMES[k % len(HDB_THEMES)]

    def school_roof(self, poly):
        """The roof colour seen on satellite for the school this building is in. / 学校の屋根色。"""
        if not self.sites:
            return None
        c = poly.representative_point()
        for area, name in self.schools:
            if area.contains(c):
                for f in self.sites.of_type("roof_colour"):
                    if f.get("school") == name:
                        return f["colour"]
        return None

    # -- roofs / 屋根
    def flat_roof(self, m, poly, z, wall, parapet=1.0, mat="roof_concrete"):
        m.flat(mat, poly.buffer(-0.25, join_style="mitre"), z, 2.0)
        ring = poly.difference(poly.buffer(-0.25, join_style="mitre"))
        m.flat(wall, ring, z + parapet, 1.0)
        for p in polys(poly.buffer(-0.25, join_style="mitre")):
            m.ring_walls(wall, p.exterior, z, z + parapet, outward=False)
        m.ring_walls(wall, poly.exterior, z, z + parapet, outward=True)

    def roof_clear(self, poly, overhang):
        """Whether a roof built on the footprint's oriented rectangle (plus its
        overhang) stays off the road. A footprint cut back from the road keeps
        its rectangle's corner over the road (Road Checker found church and
        house roofs 0.5-2.4 m over the carriageway). / 屋根が道路に掛からないか。"""
        L = getattr(self, "L", None)
        if L is None:
            return True
        r = poly.minimum_rotated_rectangle.buffer(min(overhang, FACADE_DEPTH * 0.8) + 0.1, join_style="mitre")
        return not r.intersects(L.road_all)

    def gable_roof(self, m, poly, z, pitch_h, mat, wall, overhang=0.4):
        """A pitched roof over the footprint's oriented rectangle; a flat roof if
        that would reach over a road. / 外形の外接矩形に切妻屋根（道路に掛かるなら陸屋根）。"""
        if not self.roof_clear(poly, overhang):
            self.flat_roof(m, poly, z, wall, 0.6)
            return
        r = poly.minimum_rotated_rectangle
        c = list(orient(r, 1.0).exterior.coords)[:4]
        e = [math.dist(c[i], c[(i + 1) % 4]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, b, cc, d = [np.array(c[(k + i) % 4]) for i in range(4)]
        ax = (b - a) / max(np.linalg.norm(b - a), 1e-6)
        side = (d - a) / max(np.linalg.norm(d - a), 1e-6)
        ov = min(overhang, FACADE_DEPTH * 0.8)
        a2, b2 = a - ax * ov - side * ov, b + ax * ov - side * ov
        d2, c2 = d - ax * ov + side * ov, cc + ax * ov + side * ov
        r1, r2 = (a2 + d2) / 2, (b2 + c2) / 2
        P = lambda q, zz: (q[0], q[1], zz)
        m.quad(mat, P(a2, z), P(b2, z), P(r2, z + pitch_h), P(r1, z + pitch_h))
        m.quad(mat, P(c2, z), P(d2, z), P(r1, z + pitch_h), P(r2, z + pitch_h))
        # gable ends / 妻壁
        a1, b1, c1, d1 = a, b, cc, d
        rr1, rr2 = (a1 + d1) / 2, (b1 + c1) / 2
        m.tris(wall, [P(d1, z), P(a1, z), P(rr1, z + pitch_h)], np.r_[-ax, 0], [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        m.tris(wall, [P(b1, z), P(c1, z), P(rr2, z + pitch_h)], np.r_[ax, 0], [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        # soffit / 軒裏
        m.quad("concrete", P(b2, z), P(a2, z), P(d2, z), P(c2, z))

    def hip_roof(self, m, poly, z, h, mat, overhang=0.4, wall="paint_white"):
        if not self.roof_clear(poly, overhang):
            self.flat_roof(m, poly, z, wall, 0.6)
            return
        r = poly.minimum_rotated_rectangle
        c = [np.array(q) for q in list(orient(r, 1.0).exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, b, cc, d = [c[(k + i) % 4] for i in range(4)]
        ctr = (a + b + cc + d) / 4
        ov = min(overhang, FACADE_DEPTH * 0.8)
        grow = lambda q: q + (q - ctr) / max(np.linalg.norm(q - ctr), 1e-6) * ov
        a, b, cc, d = grow(a), grow(b), grow(cc), grow(d)
        ax = (b - a) / max(np.linalg.norm(b - a), 1e-6)
        short = np.linalg.norm(d - a)
        long = np.linalg.norm(b - a)
        inset = min(short / 2, long / 2 - 0.1)
        r1, r2 = ctr - ax * (long / 2 - inset), ctr + ax * (long / 2 - inset)
        P = lambda q, zz: (q[0], q[1], zz)
        m.quad(mat, P(a, z), P(b, z), P(r2, z + h), P(r1, z + h))
        m.quad(mat, P(cc, z), P(d, z), P(r1, z + h), P(r2, z + h))
        m.tris(mat, [P(b, z), P(cc, z), P(r2, z + h)], (0, 0, 1), [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        m.tris(mat, [P(d, z), P(a, z), P(r1, z + h)], (0, 0, 1), [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        m.quad("concrete", P(b, z), P(a, z), P(d, z), P(cc, z))

    def numbers(self, m, f, text, u, z, mat, size=1.2):
        """Block number plates on an end wall (from addr:housenumber), centred
        at (u, z), standing 6 cm off the wall. / 妻壁の棟番号（壁から 6 cm）。"""
        from markings import glyphs
        from mesh import earcut
        g = glyphs(text, size, 0.8)
        if g is None:
            return
        for p in polys(g):
            p = orient(p, 1.0)                         # CCW in (u, z): faces +n / (u,z) で反時計回り
            rings = [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]
            V = np.concatenate(rings)
            ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
            I = earcut.triangulate_float64(V.astype(np.float64), ends).reshape(-1, 3)
            P3 = [f.P(u + q[0], 0.06, z + q[1]) for q in V]
            m.tris(mat, P3, np.r_[f.n, 0.0], V / size, I)

    # -- HDB / 公営住宅
    def b_hdb(self, m, poly, t, levels, rng):
        wall, acc, acc2, roofc = self.theme(poly)
        z_top = VOID + (levels - 1) * FLOOR
        front = self.front_normal(poly)
        edges = edges_of(poly)
        longest = max(e[2] for e in edges)
        corridor_dir = front if rng.random() < 0.7 else -front
        core = poly.buffer(-2.0, join_style="mitre")
        if core.is_empty or core.geom_type != "Polygon":
            core = poly.buffer(-0.8, join_style="mitre")
        # void deck: the core walls set back, columns on the footprint line / ピロティ
        if not core.is_empty:
            for c in polys(core):
                m.ring_walls(wall, c.exterior, 0.15, VOID, outward=True, uv_scale=2.0)
            m.flat("void_floor", poly, 0.16, 1.0)
            m.flat("concrete", poly, VOID - 0.02, 2.0, up=False)
        block_no = t.get("addr:housenumber")
        for a, b, L, n in edges:
            f = Facade(m, a, b, n)
            # ground floor columns along the edge / 外周の柱
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 5.0) + 1)):
                f.block(wall, u - 0.3, u + 0.3, -0.6, 0.0, 0.15, VOID, top=False, bottom=False)
            f.block(acc, 0.0, L, 0.0, 0.25, VOID - 0.45, VOID, top=True, bottom=True, ends=False)
            corridor = float(n @ corridor_dir) > 0.5 and L > 6.0
            end_wall = L < 7.0 or (L < 0.45 * longest and L < 14.0)
            for k in range(levels - 1):
                z0 = VOID + k * FLOOR
                z1 = z0 + FLOOR
                if end_wall:
                    f.face(wall, 0.0, L, z0, z1, 0.0, 2.0)
                    # accent band at every floor / 各階の帯
                    if L > 2.0:
                        f.block(acc, 0.0, L, 0.0, 0.12, z0, z0 + 0.25, ends=False)
                elif corridor:
                    self.hdb_corridor(f, L, z0, z1, wall, acc, acc2, rng)
                else:
                    self.hdb_windows(f, L, z0, z1, wall, acc, rng, k)
            if end_wall and block_no and L > 4.0 and levels >= 6 and len(block_no) <= 4:
                self.numbers(m, f, block_no, L / 2, z_top - 2.6, acc2, size=1.6)
        # roof: a pitched frame only on a compact block / 屋根: 切妻は単純な形の棟のみ
        mrr = poly.minimum_rotated_rectangle
        compact = poly.area / max(mrr.area, 1.0) > 0.82 and min(
            math.dist(*list(mrr.exterior.coords)[0:2]), math.dist(*list(mrr.exterior.coords)[1:3])) < 22.0
        if compact and rng.random() < (0.9 if roofc == "roof_terracotta" and wall == "paint_brickred" else 0.55):
            self.flat_roof(m, poly, z_top, wall, 0.3)
            r = poly.minimum_rotated_rectangle
            self.gable_roof(m, poly.buffer(-0.5, join_style="mitre") if poly.area > 60 else poly, z_top + 0.3,
                            2.2, roofc, wall, overhang=0.6)
        else:
            self.flat_roof(m, poly, z_top, wall, 1.1)
            c = poly.centroid
            if poly.contains(Point(c.x, c.y)):
                ang = math.atan2(edges[0][1][1] - edges[0][0][1], edges[0][1][0] - edges[0][0][0])
                m.box(wall, c.x, c.y, z_top, z_top + 3.6, min(8.0, math.sqrt(poly.area) * 0.35), 5.0, ang)
                m.box("tank", c.x, c.y, z_top + 3.6, z_top + 5.2, 4.0, 3.0, ang)

    def hdb_corridor(self, f, L, z0, z1, wall, acc, acc2, rng):
        D = min(1.6, FACADE_DEPTH + 0.6)
        slab = 0.25
        # the slab edge and the parapet on the facade line; the corridor behind
        # 床版の小口と腰壁、奥に共用廊下
        f.face(acc2, 0.0, L, z0, z0 + slab, 0.0)
        f.block(acc, 0.0, L, -0.15, 0.0, z0 + slab, z0 + 1.05, top=True, bottom=False, ends=False)
        f.face("corridor_dark", 0.0, L, z0 + 1.05, z1, -D)
        # the corridor ceiling and the back wall details / 廊下の天井と奥の壁
        f.hplane("concrete", 0.0, L, -D, 0.0, z1 - 0.001, False)
        for u0, u1 in bays(L, 6.0, 0.4):
            mid = (u0 + u1) / 2
            f.face("door_wood", mid - 1.4, mid - 0.5, z0 + 0.25, z0 + 2.3, -D + 0.02)
            f.face("glass_2", mid + 0.2, mid + 1.6, z0 + 1.1, z0 + 2.1, -D + 0.02, uvs=1.0)
        # columns between the bays / 柱
        for u in np.linspace(0.2, L - 0.2, max(2, int(L // 6.0) + 1)):
            f.block(wall, u - 0.2, u + 0.2, -D, 0.0, z0 + 1.05, z1, top=False, bottom=False)
        # railing on the parapet / 手すり
        f.block("railing", 0.0, L, -0.1, -0.05, z0 + 1.05, z0 + 1.1, ends=False)

    def hdb_windows(self, f, L, z0, z1, wall, acc, rng, k):
        bw = 3.2
        opens = []
        details = []
        for i, (u0, u1) in enumerate(bays(L, bw, 0.6)):
            mid = (u0 + u1) / 2
            wide = 1.9 if (i % 3) else 1.2
            opens.append((mid - wide / 2, mid + wide / 2, z0 + 0.9, z0 + 2.2))
            details.append((i, mid, wide))
        f.wall_with_openings(wall, z0, z1, opens, 2.0)
        for (a, b, oz0, oz1) in opens:
            f.window(a, b, oz0, oz1, wall, mullions=1 if b - a > 1.5 else 0)
        for i, mid, wide in details:
            if i % 3 == 1:
                # AC ledge with its louvred screen / 室外機置き場
                f.block(wall, mid + wide / 2 + 0.1, mid + wide / 2 + 1.0, 0.0, 0.55, z0 + 0.15, z0 + 0.25)
                f.block("louvre", mid + wide / 2 + 0.12, mid + wide / 2 + 0.98, 0.5, 0.55, z0 + 0.25, z0 + 0.95,
                        top=True, bottom=False)
            elif i % 3 == 2:
                # planter box below the window / プランター
                f.block(acc, mid - wide / 2 - 0.1, mid + wide / 2 + 0.1, 0.0, 0.45, z0 + 0.35, z0 + 0.85)
        # slab ledge at the floor line / 床ラインの庇
        f.block(acc if k % 2 == 0 else wall, 0.0, L, 0.0, 0.3, z0 - 0.05, z0 + 0.12, ends=False)

    # -- condo / 民間マンション
    def b_condo(self, m, poly, t, levels, rng):
        wall = rng.choice(["paint_white", "paint_palegrey", "paint_cream"])
        acc = rng.choice(["paint_charcoal", "paint_grey", "paint_woodbrown"])
        o = self.sites.colours(poly, "condo") if self.sites else None
        if o:
            wall, acc = o["wall"], o["acc"]
        z_top = 4.5 + (levels - 1) * 3.1
        m.prism("glass_dark", poly.buffer(-1.2, join_style="mitre"), 0.15, 4.5, top=False)
        m.flat("void_floor", poly, 0.16)
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            for u in np.linspace(0.4, L - 0.4, max(2, int(L // 7.0) + 1)):
                f.block(acc, u - 0.4, u + 0.4, -0.8, 0.0, 0.15, 4.5, top=False, bottom=False)
            for k in range(levels - 1):
                z0 = 4.5 + k * 3.1
                z1 = z0 + 3.1
                f.face("glass_dark", 0.0, L, z0 + 0.3, z1, -0.2)
                f.face(wall, 0.0, L, z0, z0 + 0.3, 0.0)
                if L > 5.0:
                    # balcony slab with glass balustrade / ガラス手すりのバルコニー
                    f.block(wall, 0.3, L - 0.3, 0.0, 0.95, z0, z0 + 0.2)
                    f.block("glass_rail", 0.35, L - 0.35, 0.9, 0.95, z0 + 0.2, z0 + 1.15, top=True, bottom=False)
                    for u in np.linspace(0.3, L - 0.3, max(2, int(L // 4.5) + 1)):
                        f.block(wall, u - 0.12, u + 0.12, -0.2, 0.0, z0 + 0.3, z1, top=False, bottom=False)
        self.flat_roof(m, poly, z_top, wall, 1.2)
        c = poly.centroid
        if poly.contains(c):
            m.box(acc, c.x, c.y, z_top, z_top + 4.0, min(10.0, math.sqrt(poly.area) * 0.5),
                  min(6.0, math.sqrt(poly.area) * 0.3), rng.random() * math.pi)

    # -- multi-storey car park / 立体駐車場
    def b_carpark(self, m, poly, t, levels, rng):
        wall, acc, acc2, roofc = self.theme(poly, "carpark")
        H = 3.0
        z_top = levels * H
        inner = poly.buffer(-1.0, join_style="mitre")
        m.flat("asphalt_lot", poly, 0.16, 6.0)
        for k in range(levels + 1):
            z = k * H
            if k:
                m.flat("concrete", poly, z - 0.3, 2.0, up=False)
                m.flat("asphalt_lot" if k < levels else "roof_concrete", poly, z, 6.0)
        if not inner.is_empty:
            for c in polys(inner):
                m.ring_walls("corridor_dark", c.exterior, 0.15, z_top, outward=True, uv_scale=3.0)
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            for k in range(1, levels + 1):
                z = k * H
                f.face(acc2 if k % 2 else wall, 0.0, L, z - 0.3, z, 0.0)
                f.block(acc if k % 2 else wall, 0.0, L, -0.15, 0.0, z, z + 1.0, top=True, bottom=False, ends=False)
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 8.0) + 1)):
                f.block(wall, u - 0.3, u + 0.3, -0.6, 0.0, 0.15, z_top + 1.0, top=True, bottom=False)
        if rng.random() < 0.5:
            # roof garden / 屋上緑化
            m.flat("grass_park", poly.buffer(-2.5, join_style="mitre"), z_top + 0.05, 4.0)

    # -- landed houses / 戸建て・テラスハウス
    def b_house(self, m, poly, t, levels, rng):
        wall = rng.choice(["paint_white", "paint_cream", "paint_palegrey", "paint_peach", "paint_white"])
        roof = rng.choice(["roof_terracotta", "roof_terracotta", "roof_grey", "roof_brown"])
        levels = min(levels, 3)
        z_top = 0.15 + levels * 3.2
        front = self.front_normal(poly)
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            facing = float(n @ front) > 0.6
            for k in range(levels):
                z0 = 0.15 + k * 3.2
                z1 = z0 + 3.2
                opens = []
                for u0, u1 in bays(L, 2.8, 0.5):
                    mid = (u0 + u1) / 2
                    if k == 0 and facing:
                        opens.append((mid - 1.0, mid + 1.0, z0 + 0.1, z0 + 2.4))
                    else:
                        opens.append((mid - 0.7, mid + 0.7, z0 + 1.0, z0 + 2.3))
                f.wall_with_openings(wall, z0, z1, opens, 2.0)
                for (u0, u1, oz0, oz1) in opens:
                    f.window(u0, u1, oz0, oz1, wall, glass="glass", mullions=1)
                f.block("concrete", 0.0, L, 0.0, 0.2, z1 - 0.15, z1, ends=False)
            if facing and L > 4.0:
                # car porch canopy on slim posts / カーポートの庇
                f.block("concrete", 0.3, min(L - 0.3, 5.5), 0.0, 0.95, 2.9, 3.05)
                for u in (0.35, min(L - 0.35, 5.45)):
                    f.block("frame", u - 0.06, u + 0.06, 0.85, 0.95, 0.15, 2.9, top=False, bottom=False)
        m.flat("concrete", poly, z_top - 0.01, 2.0)
        r = poly.minimum_rotated_rectangle
        w = min(math.dist(*list(r.exterior.coords)[0:2]), math.dist(*list(r.exterior.coords)[1:3]))
        if poly.area / max(r.area, 1.0) < 0.75 or w > 18.0:
            self.flat_roof(m, poly, z_top, wall, 0.6)
        elif rng.random() < 0.6:
            self.gable_roof(m, poly, z_top, max(1.2, w * 0.28), roof, wall, overhang=0.6)
        else:
            self.hip_roof(m, poly, z_top, max(1.0, w * 0.25), roof, overhang=0.6)

    # -- shopping centre / 商業施設
    def b_retail(self, m, poly, t, levels, rng):
        wall = rng.choice(["clad_silver", "paint_white", "clad_beige"])
        band = rng.choice(["paint_red", "paint_navy", "paint_teal", "paint_orange"])
        z_top = 5.0 + max(0, levels - 1) * 4.2
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            f.face("shopfront", 0.0, L, 0.15, 4.2, -0.3)
            f.hplane("concrete", 0.0, L, -0.3, 0.0, 4.2, False)
            f.block("concrete", 0.0, L, 0.0, 0.95, 3.6, 3.9)          # canopy / 庇
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 8.0) + 1)):
                f.block(wall, u - 0.3, u + 0.3, -0.3, 0.0, 0.15, 4.2, top=False, bottom=False)
            f.face(band, 0.0, L, 4.2, 5.0, 0.0)
            for k in range(1, levels):
                z0 = 5.0 + (k - 1) * 4.2
                f.face(wall, 0.0, L, z0, z0 + 2.6, 0.0)
                f.face("glass", 0.0, L, z0 + 2.6, z0 + 4.2, -0.15)
                f.hplane(wall, 0.0, L, -0.15, 0.0, z0 + 2.6, True)
                f.block(wall, 0.0, L, 0.0, 0.25, z0 + 4.0, z0 + 4.2, ends=False)
        self.flat_roof(m, poly, z_top, wall, 1.0)

    # -- school / 学校
    def b_school(self, m, poly, t, levels, rng):
        wall = rng.choice(["paint_cream", "paint_white"])
        rail = rng.choice(["paint_red", "paint_skyblue", "paint_mustard", "paint_green"])
        c = poly.representative_point()
        for area, name in self.schools:
            if name in SCHOOL_COLOURS and area.contains(c):
                wall, rail = SCHOOL_COLOURS[name]
        z_top = 0.15 + levels * 3.5
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            for k in range(levels):
                z0 = 0.15 + k * 3.5
                z1 = z0 + 3.5
                if L > 8.0 and k % 1 == 0 and (int(L) % 2 == 0):
                    f.face("corridor_dark", 0.0, L, z0 + 1.0, z1, -1.5)
                    f.hplane("concrete", 0.0, L, -1.5, 0.0, z1 - 0.001, False)
                    f.block(wall, 0.0, L, -0.15, 0.0, z0, z0 + 1.0, ends=False)
                    f.block(rail, 0.0, L, -0.1, 0.0, z0 + 1.0, z0 + 1.1, ends=False)
                    for u in np.linspace(0.2, L - 0.2, max(2, int(L // 4.0) + 1)):
                        f.block(wall, u - 0.2, u + 0.2, -1.5, 0.0, z0 + 1.0, z1, top=False, bottom=False)
                    f.hplane("concrete", 0.0, L, -1.5, 0.0, z0 + 1.0, True)
                else:
                    opens = [((u0 + u1) / 2 - 1.1, (u0 + u1) / 2 + 1.1, z0 + 1.0, z0 + 2.6) for u0, u1 in bays(L, 3.0)]
                    f.wall_with_openings(wall, z0, z1, opens, 2.0)
                    for o in opens:
                        f.window(*o, wall, mullions=2)
        roof = self.school_roof(poly)
        r = poly.minimum_rotated_rectangle
        rw = min(math.dist(*list(r.exterior.coords)[0:2]), math.dist(*list(r.exterior.coords)[1:3]))
        if roof == "roof_terracotta" and poly.area / max(r.area, 1.0) > 0.75 and rw < 24:
            m.flat("concrete", poly, z_top - 0.01)
            self.hip_roof(m, poly, z_top, max(1.5, rw * 0.22), roof, overhang=0.8)
        else:
            self.flat_roof(m, poly, z_top, wall, 1.0, mat=roof or "roof_concrete")

    # -- place of worship / 宗教施設
    def b_worship(self, m, poly, t, levels, rng):
        temple = t.get("building") == "temple" or t.get("religion") in ("buddhist", "taoist", "hindu")
        wall = "paint_red" if temple else "paint_white"
        o = self.sites.colours(poly, "worship", 80.0) if self.sites else None
        if o:
            wall = o["wall"]
        z_top = 0.15 + max(2, levels) * 3.8
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            opens = [((u0 + u1) / 2 - 0.6, (u0 + u1) / 2 + 0.6, 1.2, z_top - 1.5) for u0, u1 in bays(L, 3.5)]
            f.wall_with_openings(wall, 0.15, z_top, opens, 2.0)
            for o in opens:
                f.window(*o, wall, glass="glass_dark", mullions=1)
        m.flat("concrete", poly, z_top - 0.01)
        r = poly.minimum_rotated_rectangle
        if poly.area / max(r.area, 1.0) < 0.75:
            self.flat_roof(m, poly, z_top, wall, 0.8)
        elif temple:
            self.hip_roof(m, poly, z_top, 3.0, "roof_orange", overhang=0.9)
        else:
            self.gable_roof(m, poly, z_top, 5.0, "roof_grey", wall, overhang=0.5)

    def b_shed(self, m, poly, t, levels, rng):
        m.prism("paint_palegrey", poly, 0.15, 2.8, top=False)
        m.flat("roof_metal", poly, 2.8)

    def b_low(self, m, poly, t, levels, rng):
        wall = rng.choice(["paint_white", "paint_cream", "paint_palegrey"])
        levels = max(1, min(levels, 4))
        z_top = 0.15 + levels * 3.6
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            for k in range(levels):
                z0 = 0.15 + k * 3.6
                opens = [((u0 + u1) / 2 - 0.9, (u0 + u1) / 2 + 0.9, z0 + 0.9, z0 + 2.7) for u0, u1 in bays(L, 3.0)]
                f.wall_with_openings(wall, z0, z0 + 3.6, opens, 2.0)
                for o in opens:
                    f.window(*o, wall, mullions=1)
                f.block("concrete", 0.0, L, 0.0, 0.4, z0 + 3.3, z0 + 3.5, ends=False)
        self.flat_roof(m, poly, z_top, wall, 0.8)
