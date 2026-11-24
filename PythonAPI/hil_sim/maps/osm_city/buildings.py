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

Opt-in per map (maps.json entry, first used for PasirRisTampines; every other
map builds exactly as before, guide ch. 46):
  "buildings_geojson"  a building-shape file (OSM outlines + parts with
            height_m / base_m, HDB register floors): HDB storeys from the
            register (void deck = level 1), HDB car parks, private blocks
            (condo), parts of different heights built as separate pieces
  "building_detail"    {"storey_m", "void_deck_m": the HDB storey heights;
            "exec_storey_m" + "hdb_register_csv": executive-only blocks (register
            flat types) get their own storey;
            "facade_lod": true: walls merged over the floors (piers full
            height), facades no road can see get flush windows and no small
            details, party walls plain; "seen_m": how far a road sees a facade}

シンガポール風の建物（奥行きのある外壁）。張り出しは建物外形から最大 1 m。
地図ごとの任意設定（maps.json）: 建物形状ファイルの高さ・部分、HDB の階高、
道路から見えない面の簡略化（三角形の予算）。他の地図は従来どおり。
"""

import hashlib
import json
import math
import os
import random

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, shape
from shapely.geometry.polygon import orient

from mesh import Mesh, polys

FACADE_DEPTH = 1.0
FLOOR = 2.8
VOID = 3.6
SEEN_NEAR = 15.0          # facade_lod: a facade this near a road is always seen / これより道路に近い面は常に「見える」
PIECE_MIN = 12.0          # geojson parts: smaller pieces are left out (m2) / これより小さい部分は省く
RAISED = 4.0              # geojson parts with base_m above this are raised (crowns, bridges) / 持ち上げた部分

_OPTS = {}


def map_options(meta):
    """This map's maps.json entry (by its name), {} if none. / この地図の maps.json の項目。"""
    name = (meta or {}).get("name")
    if name not in _OPTS:
        ent = {}
        try:
            for v in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                                 "maps.json"))).values():
                if isinstance(v, dict) and v.get("name") == name:
                    ent = v
                    break
        except (OSError, ValueError):
            pass
        _OPTS[name] = ent
    return _OPTS[name]


_ABBR = {"STREET": "ST", "AVENUE": "AVE", "DRIVE": "DR", "ROAD": "RD", "CENTRAL": "CTRL", "NORTH": "NTH",
         "SOUTH": "STH", "CRESCENT": "CRES", "CLOSE": "CL", "PLACE": "PL", "TERRACE": "TER", "HEIGHTS": "HTS",
         "GROVE": "GR", "UPPER": "UPP", "JALAN": "JLN", "LORONG": "LOR", "BUKIT": "BT", "INDUSTRIAL": "IND"}


def _street(s):
    """A street name as the HDB register writes it (TAMPINES STREET 81 -> TAMPINES ST 81). / 台帳の通り名。"""
    import re
    return " ".join(_ABBR.get(w, w) for w in re.sub(r"[^A-Z0-9 ]", " ", str(s or "").upper()).split())


def _blk(s):
    return "".join(str(s or "").upper().split())


class GeoHeights(object):
    """maps.json "buildings_geojson": building outlines and parts with heights
    (properties building_id, part, height_m, base_m, levels_used, height_source,
    hdb_max_floor_lvl, hdb_multistorey_carpark, hdb_year_completed). A footprint of
    the layout (cut back from the roads) takes the record it overlaps most (>= 30 %).
    建物形状ファイル: 外形・部分・高さ。レイアウトの外形に最も重なる記録を使う。"""

    def __init__(self, path, meta, register=None):
        import geo
        from shapely.strtree import STRtree
        fr = geo.Frame(meta)
        d = json.load(open(os.path.expanduser(path)))
        recs = {}
        for f in d["features"]:
            p = f["properties"]
            g = shapely.transform(shape(f["geometry"]), lambda c: np.column_stack(fr.xy(c[:, 0], c[:, 1])))
            if not g.is_valid:
                g = g.buffer(0)
            r = recs.setdefault(p.get("building_id") or p["id"], {"geoms": [], "props": [], "parts": []})
            r["geoms"].append(g)
            r["props"].append(p)
            if p.get("part"):
                r["parts"].append((g, float(p.get("base_m") or 0.0), float(p.get("height_m") or 0.0),
                                   p.get("levels_used")))
        self.recs = list(recs.values())
        for r in self.recs:
            r["outline"] = shapely.union_all(r["geoms"]) if len(r["geoms"]) > 1 else r["geoms"][0]
            P = r["props"]
            hdb = [p for p in P if p.get("hdb_max_floor_lvl")]
            r["hdb_floors"] = max(int(p["hdb_max_floor_lvl"]) for p in hdb) if hdb else 0
            r["hdb_carpark"] = any(p.get("hdb_multistorey_carpark") for p in hdb)
            yrs = [int(p["hdb_year_completed"]) for p in hdb if str(p.get("hdb_year_completed") or "").isdigit()]
            r["year"] = min(yrs) if yrs else 0
            ex = self.exec_only(register)
            r["exec"] = bool(hdb) and any((_blk(p.get("block")), _street(p.get("street"))) in ex for p in hdb)
        self.tree = STRtree([r["outline"] for r in self.recs])

    _ex = None

    def exec_only(self, path):
        """(block, street) of the register's blocks with only executive /
        multi-generation flats (exec_sold + multigen_sold = total_dwelling_units): the
        maisonettes whose storeys measured 2.6 m (prs11). / 台帳でエグゼクティブ住戸のみの棟。"""
        if self._ex is None:
            self._ex = set()
            if path:
                import csv
                for x in csv.DictReader(open(os.path.expanduser(path))):
                    try:
                        n = int(x["total_dwelling_units"])
                        if x.get("residential") == "Y" and n > 0 and \
                                int(x["exec_sold"]) + int(x["multigen_sold"]) == n:
                            self._ex.add((_blk(x["blk_no"]), _street(x["street"])))
                    except (KeyError, ValueError):
                        pass
        return self._ex

    def match(self, poly):
        best, ba = None, 0.0
        for k in self.tree.query(poly, predicate="intersects"):
            a = self.recs[k]["outline"].intersection(poly).area
            if a > ba:
                best, ba = self.recs[k], a
        return best if best is not None and ba >= 0.3 * poly.area else None

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
    if b == "parking" or t.get("parking") == "multi-storey" or t.get("amenity") == "parking" \
            or t.get("@hdb_carpark"):
        return "carpark"
    if t.get("@hdb") is not None and b in ("apartments", "residential", "yes") and levels >= 4:
        return "hdb"            # buildings_geojson: in the HDB register / HDB 台帳にある棟
    if t.get("@private") and b in ("apartments", "residential") and levels >= 4 and t.get("addr:housenumber"):
        return "condo"          # buildings_geojson: an addressed block not in the register / 台帳に無い住棟
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

    def block(self, mat, u0, u1, w0, w1, z0, z1, top=True, bottom=True, ends=True, uvs=1.0):
        """A box standing out from (or into) the wall. / 壁から張り出す箱。"""
        self.face(mat, u0, u1, z0, z1, w1, uvs)
        if top:
            self.hplane(mat, u0, u1, w0, w1, z1, True)
        if bottom:
            self.hplane(mat, u0, u1, w0, w1, z0, False)
        if ends:
            self.side(mat, u0, w0, w1, z0, z1, -1)
            self.side(mat, u1, w0, w1, z0, z1, 1)

    def columns(self, mat, us, half, w0, w1, z0, z1, end_w1=None):
        """Columns of a recessed corridor (fronts and sides, no top/bottom). A column
        touching the edge's end gets no end face there: at a convex corner it lay in
        the next wall's plane (coplanar faces: a black strip in Cycles, z-fighting in
        UE), at a concave one inside the building. end_w1: the front of those end
        columns (closes the slit beside the next wall). / 廊下の柱。端の柱は端面なし
        （隣の壁と同一平面になり Cycles で黒、UE でちらつき）。"""
        for u in us:
            a, b = u - half, u + half
            at_end = a <= 1e-3 or b >= self.L - 1e-3
            wf = end_w1 if (at_end and end_w1 is not None) else w1
            self.face(mat, a, b, z0, z1, wf)
            if a > 1e-3:
                self.side(mat, a, w0, wf, z0, z1, -1)
            if b < self.L - 1e-3:
                self.side(mat, b, w0, wf, z0, z1, 1)

    def window(self, u0, u1, z0, z1, wall_mat, glass="glass", frame="frame", depth=0.15, mullions=1, sill_front=True):
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
        if sill_front:
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

    def grid(self, mat, rows, seen, glass="glass", mullions=1, uvs=2.0):
        """facade_lod: a wall of stacked floors rows [(z0, z1, openings)] (rows
        touching, openings [(u0, u1, oz0, oz1)]). When every floor has the same
        columns the piers run the full height (one quad each) and each column gets
        one spandrel per floor; else floor by floor. Walls seen from near a road
        (seen 2) get real windows (reveals, sill top), the others flush glass in the
        hole (no gap, no z-fight).
        階を重ねた壁: 柱型は全高 1 枚、窓は見える面だけ奥行き付き（他は面一のガラス）。"""
        if not rows:
            return
        cols = [[(a, b) for a, b, _, _ in sorted(o)] for _, _, o in rows]
        zA, zB = rows[0][0], rows[-1][1]
        if all(c == cols[0] for c in cols):
            u = 0.0
            for a, b in cols[0]:
                self.face(mat, u, a, zA, zB, 0.0, uvs)
                u = b
            self.face(mat, u, self.L, zA, zB, 0.0, uvs)
            for j, (a, b) in enumerate(cols[0]):
                zs = zA
                for _, _, o in rows:
                    oz0, oz1 = sorted(o)[j][2:]
                    self.face(mat, a, b, zs, oz0, 0.0, uvs)
                    zs = oz1
                self.face(mat, a, b, zs, zB, 0.0, uvs)
        else:
            for z0, z1, o in rows:
                self.wall_with_openings(mat, z0, z1, o, uvs)
        for _, _, o in rows:
            for a, b, oz0, oz1 in o:
                mu = mullions(b - a) if callable(mullions) else mullions
                if seen >= 2:
                    self.window(a, b, oz0, oz1, mat, glass=glass, mullions=mu, sill_front=False)
                else:
                    nm = glass if mu <= 0 else glass + ("_2" if mu == 1 else "_3")
                    self.m.quad(nm, self.P(a, 0.0, oz0), self.P(b, 0.0, oz0), self.P(b, 0.0, oz1),
                                self.P(a, 0.0, oz1), (0, 0), (1, 0), (1, 1), (0, 1))


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
        # maps.json options (opt-in; module docstring) / 地図ごとの任意設定
        meta = getattr(layout, "meta", None) or {}
        opts = {} if self.jp else map_options(meta)
        det = opts.get("building_detail") or {}
        self.FLOOR = float(det.get("storey_m", FLOOR))
        self.VOID = float(det.get("void_deck_m", VOID))
        self.lod = bool(det.get("facade_lod"))
        self.seen_m = float(det.get("seen_m", 60.0))
        self.seen_near_m = float(det.get("seen_near_m", 30.0))
        self.FLOOR_EXEC = float(det.get("exec_storey_m", self.FLOOR))
        self._floor = self.FLOOR
        self.geo = GeoHeights(opts["buildings_geojson"], meta, det.get("hdb_register_csv")) \
            if opts.get("buildings_geojson") else None
        self.made = {}            # idx -> (style, levels, height) when an option is on / 実際の様式・階数・高さ
        # colour_notes.json "palette_sg" (General Painter, opt-in by data): Street View paints with
        # weathered textures (textures.write_palette), "<paint>_base" grime on the void deck,
        # seeded theme variants ("alt") and per-kind paint lists ("kinds_sg"); maps without it
        # build as before / 観測色のパレット（任意）: 風化テクスチャ・ピロティの汚れ・配色の変化
        notes = getattr(sites, "notes", None) or {}
        self.palette = notes.get("palette_sg") or {}
        self.kinds = notes.get("kinds_sg") or {}
        self.geo_stats = {}
        self._cw = self._bld = None
        if self.lod and layout is not None:
            self._cw = layout.carriageway
            self._bld = layout.building_area
            shapely.prepare(self._cw)
            shapely.prepare(self._bld)

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
        info = self.geo.match(poly) if self.geo is not None else None
        if self.geo is not None:
            t = self.geo_tags(t, info)
        levels = levels_of(t, levels_default.get(t.get("building"), 2))
        st = style_of(t, poly, levels)
        self.count[st] = self.count.get(st, 0) + 1
        rng = random.Random(seed_of(idx, poly.centroid.x, poly.centroid.y))
        poly = orient(poly.simplify(0.25), 1.0)
        if poly.geom_type != "Polygon" or poly.area < 4.0:
            return None
        m = Mesh()
        fn = getattr(self, "b_" + st)
        pieces = self.geo_pieces(info, poly) if info is not None else None
        if pieces:
            self.build_pieces(m, pieces, st, t, idx)
            levels = max(self.piece_levels(st, top, lv) for q, base, top, lv in pieces if base <= RAISED) \
                if any(base <= RAISED for q, base, top, lv in pieces) else levels
        else:
            fn(m, poly, t, levels, rng)
        if self.geo is not None or self.lod:
            self.made[idx] = (st, levels, self.roof_z(st, levels, t) + {"hdb": 1.2, "condo": 1.2}.get(st, 1.0))
        return m

    # -- buildings_geojson (opt-in) / 建物形状ファイル（任意）
    def geo_tags(self, t, info):
        """The tags with what the shape file adds: the HDB register's storeys
        (void deck = level 1; it wins over OSM's levels, which miss the void deck on
        77 blocks), "@hdb" (year completed), "@hdb_carpark", "@private" (no register
        match). / 形状ファイルの情報を加えたタグ（HDB 台帳の階数・竣工年・駐車場）。"""
        t = dict(t)
        st = self.geo_stats
        if info is None:
            st["unmatched"] = st.get("unmatched", 0) + 1
            return t
        st["matched"] = st.get("matched", 0) + 1
        if info["hdb_floors"]:
            st["hdb"] = st.get("hdb", 0) + 1
            t["@hdb"] = info["year"]
            if info.get("exec"):
                t["@hdb_exec"] = "yes"      # executive maisonettes: their own storey / エグゼクティブ棟の階高
                st["exec"] = st.get("exec", 0) + 1
            if info["hdb_carpark"]:
                t["@hdb_carpark"] = "yes"
            if not info["hdb_carpark"] or "building:levels" not in t:
                t["building:levels"] = str(info["hdb_floors"])
        else:
            t["@private"] = "yes"
            if "building:levels" not in t and "height" not in t and \
                    not info["props"][0].get("height_source", "").startswith("default"):
                lv = info["props"][0].get("levels_used")
                if lv:
                    t["building:levels"] = str(int(round(float(lv))))
        return t

    def geo_pieces(self, info, poly):
        """The parts of a building as pieces of the (cut-back) footprint when their
        heights differ by 3 m or more or some are raised, else None (one solid).
        高さの違う部分（3 m 以上）や持ち上げた部分があれば、外形内の部分ごとに分ける。"""
        parts = info["parts"]
        ground = [top for g, base, top, lv in parts if base <= RAISED]
        if not ground or (max(ground) - min(ground) < 3.0 and len(ground) == len(parts)):
            return None
        out = []
        for g, base, top, lv in parts:
            for p in polys(g.intersection(poly)):
                p = orient(p.simplify(0.25), 1.0)
                if p.geom_type == "Polygon" and p.area >= PIECE_MIN:
                    out.append((p, base, top, lv))
        if out:
            self.geo_stats["pieces"] = self.geo_stats.get("pieces", 0) + len(out)
        return out or None

    def piece_levels(self, st, top, lv):
        if lv:
            return max(1, int(round(float(lv))))
        z = top - 1.2
        n = {"hdb": 1 + (z - self.VOID) / self.FLOOR, "condo": 1 + (z - 4.5) / 3.1, "carpark": z / 3.0,
             "retail": 1 + (z - 5.0) / 4.2, "school": z / 3.5, "house": z / 3.2}.get(st, z / 3.6)
        return max(1, int(round(n)))

    def roof_z(self, st, lv, t=None):
        """Roof (structure) height of a style with lv levels. / 様式と階数から屋上高さ。"""
        F = self.FLOOR_EXEC if t is not None and t.get("@hdb_exec") else self.FLOOR
        return {"hdb": self.VOID + (lv - 1) * F, "condo": 4.5 + (lv - 1) * 3.1, "carpark": lv * 3.0,
                "house": 0.15 + min(lv, 3) * 3.2, "retail": 5.0 + max(0, lv - 1) * 4.2,
                "school": 0.15 + lv * 3.5, "worship": 0.15 + max(2, lv) * 3.8, "shed": 2.8}.get(
            st, 0.15 + max(1, min(lv, 4)) * 3.6)

    def build_pieces(self, m, pieces, st, t, idx):
        """Each ground piece in the building's style with its own storeys (low
        podiums of a tower block as "low"); raised pieces (roof crowns, link
        bridges) as plain boxes standing on the piece under them.
        部分ごとに同じ様式で建てる（低い基壇は low）。持ち上げた部分は下の部分に載る箱。"""
        tops = []
        for q, base, top, lv in pieces:
            if base > RAISED:
                continue
            plv = self.piece_levels(st, top, lv)
            pst = "low" if st in ("hdb", "condo") and plv <= 3 else st
            getattr(self, "b_" + pst)(m, q, t, plv, random.Random(seed_of(idx, q.centroid.x, q.centroid.y)))
            tops.append((q, self.roof_z(pst, plv, t)))
        wall = self.theme(pieces[0][0])[0] if st in ("hdb", "carpark") else "paint_palegrey"
        for q, base, top, lv in pieces:
            if base <= RAISED:
                continue
            under = [z for g, z in tops if g.intersection(q).area > 0.3 * q.area]
            z0 = max(under) if under else base
            m.prism(wall, q, z0, z0 + max(1.0, top - base), top=True, bottom=not under, uv_scale=2.0)

    # -- facade_lod (opt-in) / 簡略化（任意）
    def seen(self, a, b, n):
        """How well a road sees this facade: 2 near (within SEEN_NEAR of the
        carriageway, or a ray out of it, straight or +-40 deg from up to three
        points, meets the carriageway within seen_near_m), 1 farther (within
        seen_m), 0 not at all. Always 2 without facade_lod. Window reveals need 2,
        ledges and AC units 1. / 道路からの見え方: 2 近い（窓の奥行き）, 1 遠い（庇・室外機）, 0 見えない。"""
        if self._cw is None:
            return 2
        L = float(np.linalg.norm(b - a))
        pts = [a + (b - a) * s for s in ((0.2, 0.5, 0.8) if L > 30.0 else (0.5,))]
        if any(shapely.dwithin(self._cw, Point(*p), SEEN_NEAR) for p in pts):
            return 2
        best = 0
        for ang in (0.0, 0.7, -0.7):
            c, s = math.cos(ang), math.sin(ang)
            d = np.array([c * n[0] - s * n[1], s * n[0] + c * n[1]])
            for p in pts:
                if self._cw.intersects(LineString([p + d * 0.5, p + d * self.seen_near_m])):
                    return 2
                if not best and self._cw.intersects(LineString([p + d * 0.5, p + d * self.seen_m])):
                    best = 1
        return best

    def party(self, a, b, n):
        """A wall against another building (within 0.9 m): nobody sees it.
        隣の建物に接する壁（0.9 m 以内）。"""
        if self._bld is None:
            return False
        mid = (a + b) / 2 + n * 0.9
        return bool(self._bld.intersects(Point(mid[0], mid[1])))

    def roof_box(self, m, poly, mat, cx, cy, z0, z1, sx, sy, ang, top=True):
        """A box on the roof (lift-motor room, tank, crown) kept inside the footprint:
        at the given spot when it fits (as always), else 0.3 m inside at the spot or
        the footprint's inner point, shrunk down to half; left out if it still does
        not fit (every map: a box beyond the facade breaks FACADE_DEPTH). A motor room on the
        centroid of a bent block stood 2 m out over the road (check_roads,
        2026-11-21). Returns (x, y, scale) or None.
        屋上の箱は外形の内側に収める（重心が外形外に近い棟で道路上に出た）。"""
        inner = poly.buffer(-0.3, join_style="mitre")
        if inner.is_empty:
            return None
        c, s = math.cos(ang), math.sin(ang)

        def rect(x, y, k):
            hx, hy = sx * k / 2, sy * k / 2
            return Polygon([(x + c * u - s * v, y + s * u + c * v) for u, v in
                            ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))])
        if poly.buffer(0.01).contains(rect(cx, cy, 1.0)):
            m.box(mat, cx, cy, z0, z1, sx, sy, ang, top=top)    # where it always stood / 従来どおり
            return cx, cy, 1.0
        p = inner.representative_point()
        for x, y in ((cx, cy), (p.x, p.y)):
            for k in (1.0, 0.75, 0.5):
                if inner.contains(rect(x, y, k)):
                    m.box(mat, x, y, z0, z1, sx * k, sy * k, ang, top=top)
                    return x, y, k
        return None

    def kind_paint(self, kind, key, poly, default):
        """colour_notes.json "kinds_sg": {kind: {key: [paints]}} seen in Street View for a
        building kind (house walls / roofs, retail walls / bands, school walls / rails, low
        walls): a seeded pick per building (its own stream, so the builder's rng is untouched);
        the default without the notes. / 種類ごとの観測色から建物ごとに選ぶ（無ければ従来の色）。"""
        opts = (self.kinds.get(kind) or {}).get(key)
        if not opts:
            return default
        c = poly.centroid
        return opts[seed_of(kind, key, round(c.x), round(c.y)) % len(opts)]

    def base_of(self, wall):
        """The void-deck version of a palette paint (darker foot band, splashes; v = 0..1 over
        the void deck height), else the paint itself. / ピロティ用の汚れ付き塗装。"""
        return wall + "_base" if wall in self.palette else wall

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
            if o.get("alt") and self.palette:
                # seeded variation inside the observed precinct (the main theme twice as
                # often as each alternate) / 観測地区内の配色の変化（主配色は 2 倍）
                c = poly.centroid
                opts = [(wall, acc, acc2)] * 2 + [tuple(x) for x in o["alt"]]
                wall, acc, acc2 = opts[seed_of("theme", round(c.x), round(c.y)) % len(opts)]
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

    def party_clamp(self, rect, grown):
        """Roof corners grown for the overhang, pulled back to the wall line on every side of the
        rectangle rect (CCW a, b, c, d) that another building touches (within 0.9 m): terraced
        houses' eaves ran past the shared wall into the neighbour's (coplanar soffits, flicker).
        Unchanged when no side is shared. / 隣家と接する辺は軒を出さない（軒裏の重なり）。"""
        L = getattr(self, "L", None)
        ba = getattr(L, "building_area", None)
        if ba is None or ba.is_empty:
            return grown
        if getattr(self, "_ba_prep", None) is not ba:
            shapely.prepare(ba)
            self._ba_prep = ba
        out = [np.array(g, np.float64) for g in grown]
        for i in range(4):
            p, q = rect[i], rect[(i + 1) % 4]
            t = (q - p) / max(np.linalg.norm(q - p), 1e-6)
            nrm = np.array([t[1], -t[0]])
            if not any(ba.intersects(Point(*(p + (q - p) * s + nrm * 0.9))) for s in (0.25, 0.5, 0.75)):
                continue
            for j in (i, (i + 1) % 4):
                d = float((out[j] - p) @ nrm)
                if d > 0:
                    out[j] = out[j] - nrm * d
        return out

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
        a2, b2, c2, d2 = self.party_clamp([a, b, cc, d], [a2, b2, c2, d2])
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
        a, b, cc, d = self.party_clamp([a, b, cc, d], [grow(a), grow(b), grow(cc), grow(d)])
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
        # (maps.json building_detail: executive-only blocks their own storey) / 地図ごとの階高
        VOID, FLOOR = self.VOID, (self.FLOOR_EXEC if t.get("@hdb_exec") else self.FLOOR)
        self._floor = FLOOR
        z_top = VOID + (levels - 1) * FLOOR
        front = self.front_normal(poly)
        edges = edges_of(poly)
        longest = max(e[2] for e in edges)
        corridor_dir = front if rng.random() < 0.7 else -front
        core = poly.buffer(-2.0, join_style="mitre")
        if core.is_empty or core.geom_type != "Polygon":
            core = poly.buffer(-0.8, join_style="mitre")
        # void deck: the core walls set back, columns on the footprint line / ピロティ
        base = self.base_of(wall)
        buv = VOID if base != wall else 2.0           # the grime band texture spans the void deck / 汚れは階高に合わせる
        if not core.is_empty:
            for c in polys(core):
                m.ring_walls(base, c.exterior, 0.15, VOID, outward=True, uv_scale=buv)
            m.flat("void_floor", poly, 0.16, 1.0)
            m.flat("concrete", poly, VOID - 0.02, 2.0, up=False)
        block_no = t.get("addr:housenumber")
        # corridor edges, and where two meet at a convex corner the corner square belongs to the
        # second: the first stops its floor / ceiling short (both drew it: coplanar faces, black
        # per storey in Cycles, z-fighting in UE) / 凸角で隣り合う廊下の床・天井の重なりを避ける
        is_corr = [float(n @ corridor_dir) > 0.5 and L > 6.0 and not (L < 7.0 or (L < 0.45 * longest and L < 14.0))
                   for a, b, L, n in edges]
        D_c = min(1.6, FACADE_DEPTH + 0.6)
        trims = []
        for i, (a, b, L, n) in enumerate(edges):
            j = (i + 1) % len(edges)
            ti, tj = (b - a) / L, (edges[j][1] - edges[j][0]) / edges[j][2]
            turn = float(ti[0] * tj[1] - ti[1] * tj[0])          # sin of the left turn (CCW: convex) / 左折の正弦
            trims.append(min(D_c / max(turn, 0.3), L / 2) if is_corr[i] and is_corr[j] and turn > 0.1 else 0.0)
        for (a, b, L, n), trim in zip(edges, trims):
            f = Facade(m, a, b, n)
            # ground floor columns along the edge (facade_lod: none on the short jogs
            # of a stepped outline) / 外周の柱（簡略時は段差の短い辺に置かない）
            for u in (np.linspace(0.3, L - 0.3, max(2, int(L // 5.0) + 1)) if not self.lod or L >= 2.5 else ()):
                f.block(base, u - 0.3, u + 0.3, -0.6, 0.0, 0.15, VOID, top=False, bottom=False,
                        uvs=VOID if base != wall else 1.0)
            f.block(acc, 0.0, L, 0.0, 0.25, VOID - 0.45, VOID, top=True, bottom=True, ends=False)
            corridor = float(n @ corridor_dir) > 0.5 and L > 6.0
            end_wall = L < 7.0 or (L < 0.45 * longest and L < 14.0)
            if self.lod and levels > 1:
                # walls merged over the floors; details only where a road sees them
                # 壁は階をまとめて描き、細部は道路から見える面だけ
                sn = self.seen(a, b, n)
                if end_wall:
                    f.face(wall, 0.0, L, VOID, z_top, 0.0, 2.0)
                    for k in range(levels - 1 if sn else 0):
                        if L > 2.0:
                            f.block(acc, 0.0, L, 0.0, 0.12, VOID + k * FLOOR, VOID + k * FLOOR + 0.25, ends=False)
                elif corridor:
                    self.hdb_corridor_lod(f, L, VOID, levels - 1, wall, acc, acc2, sn, trim)
                else:
                    self.hdb_windows_lod(f, L, VOID, levels - 1, wall, acc, sn)
            for k in range(0 if self.lod else levels - 1):
                z0 = VOID + k * FLOOR
                z1 = z0 + FLOOR
                if end_wall:
                    f.face(wall, 0.0, L, z0, z1, 0.0, 2.0)
                    # accent band at every floor / 各階の帯
                    if L > 2.0:
                        f.block(acc, 0.0, L, 0.0, 0.12, z0, z0 + 0.25, ends=False)
                elif corridor:
                    self.hdb_corridor(f, L, z0, z1, wall, acc, acc2, rng, trim)
                else:
                    self.hdb_windows(f, L, z0, z1, wall, acc, rng, k)
            if end_wall and block_no and L > 4.0 and levels >= 6 and len(block_no) <= 4:
                self.numbers(m, f, block_no, L / 2, z_top - 2.6, acc2, size=1.6)
        # roof: a pitched frame only on a compact block / 屋根: 切妻は単純な形の棟のみ
        mrr = poly.minimum_rotated_rectangle
        compact = poly.area / max(mrr.area, 1.0) > 0.82 and min(
            math.dist(*list(mrr.exterior.coords)[0:2]), math.dist(*list(mrr.exterior.coords)[1:3])) < 22.0
        pitch_p = 0.9 if roofc == "roof_terracotta" and wall == "paint_brickred" else 0.55
        if t.get("@hdb") and int(t["@hdb"]) >= 2000:
            pitch_p = 0.15          # buildings_geojson: blocks from 2000 on have flat roofs / 2000 年以降は陸屋根
        if compact and rng.random() < pitch_p:
            self.flat_roof(m, poly, z_top, wall, 0.3)
            r = poly.minimum_rotated_rectangle
            self.gable_roof(m, poly.buffer(-0.5, join_style="mitre") if poly.area > 60 else poly, z_top + 0.3,
                            2.2, roofc, wall, overhang=0.6)
        else:
            self.flat_roof(m, poly, z_top, wall, 1.1)
            c = poly.centroid
            if poly.contains(Point(c.x, c.y)):
                ang = math.atan2(edges[0][1][1] - edges[0][0][1], edges[0][1][0] - edges[0][0][0])
                # inside the footprint, the tank on the motor room / 外形の内側、水槽は機械室の上
                at = self.roof_box(m, poly, wall, c.x, c.y, z_top, z_top + 3.6,
                                   min(8.0, math.sqrt(poly.area) * 0.35), 5.0, ang)
                if at is not None:
                    m.box("tank", at[0], at[1], z_top + 3.6, z_top + 5.2, 4.0 * at[2], 3.0 * at[2], ang)

    def hdb_corridor(self, f, L, z0, z1, wall, acc, acc2, rng, trim=0.0):
        D = min(1.6, FACADE_DEPTH + 0.6)
        slab = 0.25
        # the slab edge and the parapet on the facade line; the corridor behind
        # 床版の小口と腰壁、奥に共用廊下
        f.face(acc2, 0.0, L, z0, z0 + slab, 0.0)
        f.block(acc, 0.0, L, -0.15, 0.0, z0 + slab, z0 + 1.05, top=True, bottom=False, ends=False)
        # the back wall down to the corridor floor, and the floor: without them a
        # view from above (drones, flyovers) looked over the parapet into the solid
        # 奥の壁を廊下の床まで、床も描く（上から見ると腰壁越しに建物の内側が見えた）
        f.face("corridor_dark", 0.0, L, z0 + slab, z1, -D)
        f.hplane("concrete", 0.0, L - trim, -D, -0.15, z0 + slab, True)
        # the corridor ceiling and the back wall details (trim: the next corridor owns the
        # corner square) / 廊下の天井と奥の壁（trim: 角は次の廊下が描く）
        f.hplane("concrete", 0.0, L - trim, -D, 0.0, z1 - 0.001, False)
        for u0, u1 in bays(L, 6.0, 0.4):
            mid = (u0 + u1) / 2
            f.face("door_wood", mid - 1.4, mid - 0.5, z0 + 0.25, z0 + 2.3, -D + 0.02)
            f.face("glass_2", mid + 0.2, mid + 1.6, z0 + 1.1, z0 + 2.1, -D + 0.02, uvs=1.0)
        # columns between the bays / 柱
        f.columns(wall, np.linspace(0.2, L - 0.2, max(2, int(L // 6.0) + 1)), 0.2, -D, 0.0, z0 + 1.05, z1)
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

    def hdb_corridor_lod(self, f, L, zA, nfl, wall, acc, acc2, seen, trim=0.0):
        """facade_lod corridor side: slab edge, parapet, dark corridor and ceiling
        per floor; the columns one block over the full height, their fronts 1 cm
        behind the parapet's inner plane (no coplanar faces); doors, windows and
        railing only where a road sees them. / 廊下側: 柱は全高 1 本、扉・手すりは見える面のみ。"""
        D = min(1.6, FACADE_DEPTH + 0.6)
        F = self._floor
        for k in range(nfl):
            z0 = zA + k * F
            z1 = z0 + F
            f.face(acc2, 0.0, L, z0, z0 + 0.25, 0.0)
            f.block(acc, 0.0, L, -0.15, 0.0, z0 + 0.25, z0 + 1.05, top=True, bottom=False, ends=False)
            f.face("corridor_dark", 0.0, L, z0 + 0.25, z1, -D)          # down to the floor / 床まで
            f.hplane("concrete", 0.0, L - trim, -D, -0.15, z0 + 0.25, True)     # corridor floor / 廊下の床
            f.hplane("concrete", 0.0, L - trim, -D, 0.0, z1 - 0.001, False)     # (trim: corner owned by the next edge)
            if seen:
                for u0, u1 in bays(L, 6.0, 0.4):
                    mid = (u0 + u1) / 2
                    f.face("door_wood", mid - 1.4, mid - 0.5, z0 + 0.25, z0 + 2.3, -D + 0.02)
                    f.face("glass_2", mid + 0.2, mid + 1.6, z0 + 1.1, z0 + 2.1, -D + 0.02, uvs=1.0)
                f.block("railing", 0.0, L, -0.1, -0.05, z0 + 1.05, z0 + 1.1, ends=False)
        # end columns: no end face, front 2 cm behind the facade (no slit by the next wall)
        # 端の柱: 端面なし、前面は外壁面の 2 cm 奥（隣の壁との隙間をふさぐ）
        f.columns(wall, np.linspace(0.2, L - 0.2, max(2, int(L // 6.0) + 1)), 0.2, -D, -0.16, zA, zA + nfl * F,
                  end_w1=-0.02)

    def hdb_windows_lod(self, f, L, zA, nfl, wall, acc, seen):
        """facade_lod window side: the same bays as hdb_windows; piers full height,
        one spandrel per window; reveals near a road (seen 2); AC ledges, planters
        and the slab ledge at every floor where a road sees it at all (seen >= 1).
        窓側: 同じ柱間、壁は全高、見込み・室外機台・プランターは見える面のみ。"""
        F = self._floor
        cols = [(i, (u0 + u1) / 2, 1.9 if (i % 3) else 1.2) for i, (u0, u1) in enumerate(bays(L, 3.2, 0.6))]
        rows = [(zA + k * F, zA + (k + 1) * F,
                 [(mid - w / 2, mid + w / 2, zA + k * F + 0.9, zA + k * F + 2.2) for i, mid, w in cols])
                for k in range(nfl)]
        f.grid(wall, rows, seen, mullions=lambda w: 1 if w > 1.5 else 0)
        for k in range(nfl):
            z0 = zA + k * F
            if seen:
                for i, mid, wide in cols:
                    if i % 3 == 1:
                        # AC ledge and louvre screen without their end faces / 室外機台（端面なし）
                        f.block(wall, mid + wide / 2 + 0.1, mid + wide / 2 + 1.0, 0.0, 0.55, z0 + 0.15, z0 + 0.25,
                                ends=False)
                        f.block("louvre", mid + wide / 2 + 0.12, mid + wide / 2 + 0.98, 0.5, 0.55, z0 + 0.25,
                                z0 + 0.95, top=True, bottom=False, ends=False)
                    elif i % 3 == 2:
                        f.block(acc, mid - wide / 2 - 0.1, mid + wide / 2 + 0.1, 0.0, 0.45, z0 + 0.35, z0 + 0.85)
            if seen:
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
            if self.lod and levels > 1:
                # one glass wall and full-height mullions (fronts 3 cm behind the
                # slab bands); balustrades only where a road sees them
                # ガラス面と方立は全高 1 枚、手すりは見える面のみ
                sn = self.seen(a, b, n)
                f.face("glass_dark", 0.0, L, 4.5, z_top, -0.2)
                if L > 5.0:
                    for u in np.linspace(0.3, L - 0.3, max(2, int(L // 4.5) + 1)):
                        f.block(wall, u - 0.12, u + 0.12, -0.2, -0.03, 4.5, z_top, top=False, bottom=False)
                for k in range(levels - 1):
                    z0 = 4.5 + k * 3.1
                    f.face(wall, 0.0, L, z0, z0 + 0.3, 0.0)
                    if L > 5.0:
                        f.block(wall, 0.3, L - 0.3, 0.0, 0.95, z0, z0 + 0.2)
                        if sn:
                            f.block("glass_rail", 0.35, L - 0.35, 0.9, 0.95, z0 + 0.2, z0 + 1.15, top=True,
                                    bottom=False)
            for k in range(0 if self.lod else levels - 1):
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
            # the roof crown inside the footprint / 屋上の箱は外形の内側
            self.roof_box(m, poly, acc, c.x, c.y, z_top, z_top + 4.0, min(10.0, math.sqrt(poly.area) * 0.5),
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
        wall, roof = self.kind_paint("house", "walls", poly, wall), self.kind_paint("house", "roofs", poly, roof)
        levels = min(levels, 3)
        z_top = 0.15 + levels * 3.2
        front = self.front_normal(poly)
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            facing = float(n @ front) > 0.6
            if self.lod:
                # party walls plain; the rest floors stacked (flush glass unseen)
                # 隣家に接する壁は無地、他は階をまとめる（見えない面は面一のガラス）
                if self.party(a, b, n):
                    f.face(wall, 0.0, L, 0.15, z_top, 0.0, 2.0)
                    continue
                rows = []
                for k in range(levels):
                    z0 = 0.15 + k * 3.2
                    rows.append((z0, z0 + 3.2, [(mid - 1.0, mid + 1.0, z0 + 0.1, z0 + 2.4) if k == 0 and facing
                                                else (mid - 0.7, mid + 0.7, z0 + 1.0, z0 + 2.3)
                                                for mid in [(u0 + u1) / 2 for u0, u1 in bays(L, 2.8, 0.5)]]))
                f.grid(wall, rows, self.seen(a, b, n), glass="glass", mullions=1)
                for k in range(levels):
                    z1 = 0.15 + (k + 1) * 3.2
                    f.block("concrete", 0.0, L, 0.0, 0.2, z1 - 0.15, z1, ends=False)
            for k in range(0 if self.lod else levels):
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
        wall, band = self.kind_paint("retail", "walls", poly, wall), self.kind_paint("retail", "bands", poly, band)
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
        wall, rail = self.kind_paint("school", "walls", poly, wall), self.kind_paint("school", "rails", poly, rail)
        c = poly.representative_point()
        for area, name in self.schools:
            if name in SCHOOL_COLOURS and area.contains(c):
                wall, rail = SCHOOL_COLOURS[name]
        z_top = 0.15 + levels * 3.5
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            if self.lod:
                # columns full height (behind the parapet), windows stacked
                # 柱は全高（腰壁の奥）、窓は階をまとめる
                sn = self.seen(a, b, n)
                if L > 8.0 and (int(L) % 2 == 0):
                    for k in range(levels):
                        z0 = 0.15 + k * 3.5
                        z1 = z0 + 3.5
                        f.face("corridor_dark", 0.0, L, z0 + 1.0, z1, -1.5)
                        f.hplane("concrete", 0.0, L, -1.5, 0.0, z1 - 0.001, False)
                        f.block(wall, 0.0, L, -0.15, 0.0, z0, z0 + 1.0, ends=False)
                        if sn:
                            f.block(rail, 0.0, L, -0.1, 0.0, z0 + 1.0, z0 + 1.1, ends=False)
                        f.hplane("concrete", 0.0, L, -1.5, 0.0, z0 + 1.0, True)
                    f.columns(wall, np.linspace(0.2, L - 0.2, max(2, int(L // 4.0) + 1)), 0.2, -1.5, -0.16,
                              0.15, z_top, end_w1=-0.02)
                else:
                    f.grid(wall, [(0.15 + k * 3.5, 0.15 + (k + 1) * 3.5,
                                   [((u0 + u1) / 2 - 1.1, (u0 + u1) / 2 + 1.1, 0.15 + k * 3.5 + 1.0,
                                     0.15 + k * 3.5 + 2.6) for u0, u1 in bays(L, 3.0)]) for k in range(levels)],
                           sn, mullions=2)
            for k in range(0 if self.lod else levels):
                z0 = 0.15 + k * 3.5
                z1 = z0 + 3.5
                if L > 8.0 and k % 1 == 0 and (int(L) % 2 == 0):
                    f.face("corridor_dark", 0.0, L, z0 + 1.0, z1, -1.5)
                    f.hplane("concrete", 0.0, L, -1.5, 0.0, z1 - 0.001, False)
                    f.block(wall, 0.0, L, -0.15, 0.0, z0, z0 + 1.0, ends=False)
                    f.block(rail, 0.0, L, -0.1, 0.0, z0 + 1.0, z0 + 1.1, ends=False)
                    f.columns(wall, np.linspace(0.2, L - 0.2, max(2, int(L // 4.0) + 1)), 0.2, -1.5, 0.0,
                              z0 + 1.0, z1)
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
    def b_worship(self, m, poly, t, levels, rng, roof_ok=None):
        """roof_ok (Japanese maps: buildings_jp free): False for the pitched
        roof's outline (its rectangle + overhang) -> a flat roof instead.
        / 勾配屋根が道路・歩道に掛かるなら陸屋根"""
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
        pitched_clear = roof_ok is None or roof_ok(r.buffer((0.9 if temple else 0.5) + 0.2, join_style=2))
        if poly.area / max(r.area, 1.0) < 0.75 or t.get("roof:shape") == "flat" or not pitched_clear:
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
        wall = self.kind_paint("low", "walls", poly, wall)
        levels = max(1, min(levels, 4))
        z_top = 0.15 + levels * 3.6
        for a, b, L, n in edges_of(poly):
            f = Facade(m, a, b, n)
            if self.lod:
                # party walls plain, floors stacked / 隣接壁は無地、階をまとめる
                if self.party(a, b, n):
                    f.face(wall, 0.0, L, 0.15, z_top, 0.0, 2.0)
                    continue
                f.grid(wall, [(0.15 + k * 3.6, 0.15 + (k + 1) * 3.6,
                               [((u0 + u1) / 2 - 0.9, (u0 + u1) / 2 + 0.9, 0.15 + k * 3.6 + 0.9, 0.15 + k * 3.6 + 2.7)
                                for u0, u1 in bays(L, 3.0)]) for k in range(levels)], self.seen(a, b, n), mullions=1)
                for k in range(levels):
                    f.block("concrete", 0.0, L, 0.0, 0.4, 0.15 + k * 3.6 + 3.3, 0.15 + k * 3.6 + 3.5, ends=False)
            for k in range(0 if self.lod else levels):
                z0 = 0.15 + k * 3.6
                opens = [((u0 + u1) / 2 - 0.9, (u0 + u1) / 2 + 0.9, z0 + 0.9, z0 + 2.7) for u0, u1 in bays(L, 3.0)]
                f.wall_with_openings(wall, z0, z0 + 3.6, opens, 2.0)
                for o in opens:
                    f.window(*o, wall, mullions=1)
                f.block("concrete", 0.0, L, 0.0, 0.4, z0 + 3.3, z0 + 3.5, ends=False)
        self.flat_roof(m, poly, z_top, wall, 0.8)
