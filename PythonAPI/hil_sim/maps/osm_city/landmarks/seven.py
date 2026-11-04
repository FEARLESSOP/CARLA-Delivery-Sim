"""7-Eleven Oita Akenokita (セブン-イレブン 大分明野北店), on the main road
south of Oita Kosen: modelled in detail (maps/.venv).

Sources (2026): the z20 satellite tile, the Google Earth 3D orbit and top
view with the written description, Street View from the main road (Feb
2026), the side street (2017) and the east lane (2024) (Photo Getter,
build/OitaKosen/seven_photos). Outlines traced on the z20 tile
(build/OitaKosen/seven_trace.json). Nothing is copied as texture: every
texture is drawn here, the brand look only as abstract stripes and blocks.

  store        single storey white box, flat grey roof inside a parapet, the
               glazed shop front on the north face (aluminium mullions, the
               lit shop seen through the glass, dark kick plates), the
               automatic door near the north-west corner under a flat canopy,
               the orange / green / red stripe band along the north face and
               round the corner onto the west face, white panel walls with
               joints every 1.8 m elsewhere (the south wall blank, downpipes),
               the back door; roof: the L-shaped solar array, AC units, hatch
  annex        the storage box east of the store
  by the door  the recycling-bin station, three vending machines on the west
               face, the large AC units at the south end of the west walkway;
               the AC units on the ground along the south wall (roof if no room)
  car park     asphalt with white U-shaped double bay lines (front row with
               the hatched aisle and the wheelchair bay, the west row, the two
               east groups), the orange loading box, wheel stops, the concrete
               walkway and the concrete yard east of the store, the painted
               edge of the main-road footway
  levels       the lot is one level platform at the main road's footway
               height (the entrance is flat); towards the side street and the
               east lane it stands on herringbone (kenchi) block retaining
               walls with concrete-block courses and a coping on top, a white
               welded-mesh fence along them; on the north boundary a block
               wall with the mesh fence
  kanban       the pole sign at the north-west corner: round white post on a
               round base, a white box with abstract brand stripes and blocks
               and the red / green / red goods panels below (faces north and
               south, towards the main road's traffic)
  pole         the utility pole with its transformer and ground cabinet at
               the south-west corner

Roads win: the lot is cut to the road network, its kerbs, paths and the
entrance driveway (``fit_store`` keeps the traced north face and shortens the
store from the south only if the side street would come nearer than 0.4 m;
with the road fix of 2026-11-02 the full traced depth fits); the store stays
0.4 m clear, projections 0.2 m, every other part 0.3 m.

Coordinates: the map frame (x east, y north, metres).

セブン-イレブン 大分明野北店の詳細モデル。
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
from landmarks import onterrain
from landmarks.kosen import fit_rect, split_tiles
from landmarks.lvss import oquad
from mesh import Mesh, polys

NAME = "セブン-イレブン 大分明野北店"
TAG = "SEVEN"
TRACE = "seven_trace.json"
SITE_WAY = "182821513"
CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
CLEAR_FOOT = 0.4         # store to roads, kerbs, paths / 建物と道路・歩道の間隔
ROOM = 0.2               # facade projections stay this far off them / 張り出しの余裕
GAP = 0.3                # the lot platform and everything else / 造成面・工作物の間隔
PAINT = 0.006            # paint over the lot (ground.LOT_PAINT_Z - RAISED_Z) / 路面標示の高さ
BAND = (3.0, 4.0)        # stripe band (one texture tile, v = z) / ストライプ帯
GLAZE = (0.45, 2.7)      # shop window: kick plate below, panel above / 窓の高さ
DOOR_TOP = 2.7
ROOF_Z = 4.55            # roof deck / 屋上面
WALL_MAT = "seven_panel"


def edge_of(poly, nx, ny):
    """The edge of a rectangle whose outward normal is nearest (nx, ny): (a, b, L, n).
    指定方向を向く辺。"""
    return max(B.edges_of(orient(poly, 1.0)), key=lambda e: e[3][0] * nx + e[3][1] * ny)


def u_divider(p_far, p_round, half=0.25, w=0.12):
    """A Japanese double bay line: two lines ``2 half`` apart joined by a
    semicircle at the aisle end (p_round). / U 字型の二重区画線。"""
    a, b = np.asarray(p_far, float), np.asarray(p_round, float)
    t = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    n = np.array([-t[1], t[0]])
    pts = [tuple(a - n * half), tuple(b - n * half)]
    for k in range(1, 12):
        ang = math.pi * k / 12
        pts.append(tuple(b + t * half * math.sin(ang) - n * half * math.cos(ang)))
    pts += [tuple(b + n * half), tuple(a + n * half)]
    return LineString(pts).buffer(w / 2, cap_style="flat", join_style="mitre")


def drop_slivers(g, w=0.1):
    """The polygons of g at least w wide somewhere (an opening by w/2 leaves
    something); the others are dropped whole, the kept ones stay unchanged.
    幅 w 未満の細片を除く（残す多角形は形を変えない）。"""
    keep = [p for p in polys(g) if not p.buffer(-w / 2).is_empty]
    return unary_union(keep) if keep else Polygon()


def wheelchair(c, size=1.3, up=(0.0, 1.0)):
    """The access pictogram (seated figure, wheel) as polygons, ``up`` = top. / 車いすの記号。"""
    u = np.asarray(up, float) / np.linalg.norm(up)
    r = np.array([u[1], -u[0]])
    s = size
    P = lambda x, y: tuple(np.asarray(c, float) + r * x * s + u * y * s)
    parts = [Point(P(-0.05, 0.38)).buffer(0.085 * s, 12),
             LineString([P(-0.05, 0.25), P(-0.05, 0.0), P(0.22, 0.0), P(0.3, -0.3)]).buffer(0.045 * s),
             LineString([P(-0.05, 0.16), P(0.15, 0.16)]).buffer(0.035 * s),
             Point(P(-0.08, -0.12)).buffer(0.27 * s, 24).difference(Point(P(-0.08, -0.12)).buffer(0.2 * s, 24))]
    return unary_union(parts)


class SEVEN(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        self.out_dir = out_dir
        self.fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        self.drape = None                         # set by build.py (terrain.Drape) / build.py が設定
        lot_seen = self.poly(T["lot"])
        near = lot_seen.buffer(40.0)
        # the road network and its paths (before build.py clears the site)
        # 道路網と歩道（build.py が敷地内を消す前の写し）
        self.road = L.road_all.intersection(near)
        self.carriageway = L.carriageway.intersection(near)
        # path slivers narrower than 0.1 m (left by a footpath cut) are not paths:
        # 0.3 m clearance round them dug pits into the lot / 幅 0.1 m 未満の歩道片は無視
        self.paths = drop_slivers(unary_union([L.footpaths, L.cycleways, L.driveways]).intersection(near))
        self.islands = L.islands.intersection(near)
        self.no_go = unary_union([self.road, self.paths, self.islands])
        self.drives = L.driveways.intersection(near)        # the entrance driveway: no fence across it / 入口の私道
        shapely.prepare(self.no_go)
        # the lot: as traced, grown west to the main-road footway of the road
        # network (the entrance is the whole frontage), cut to every road, kerb
        # and path / 敷地: トレースを幹線歩道まで西へ広げ、道路・歩道で切る
        x0, y0, x1, y1 = lot_seen.bounds
        # up to the footway band, not the verge beyond; the traced SW corner (the
        # last 3 m) stays where it is, so no lobe forms round the band's corner curve
        # 歩道帯まで広げる（南西角の 3 m はトレースのまま）
        grow = box(x0 - 1.6, y0 + 3.0, x0 + 0.5, y1)
        g = unary_union([lot_seen, grow]).difference(self.no_go)
        if T.get("corner_cut"):
            g = g.difference(self.poly(T["corner_cut"]))   # SE chamfer clear of the curb returns / 南東の隅切り
        self.site = orient(max(polys(g.buffer(0)), key=lambda p: p.area).simplify(0.02), 1.0)
        self.lot_seen = lot_seen
        self.blocks, self.cut = {}, {}
        self.fit_store()
        self.footprint = unary_union([b["poly"] for b in self.blocks.values()])
        self.kanban = self.fit_point("kanban", self.P("c", *T["kanban_post"]["pt"]), 0.75)
        self.upole = self.fit_point("utility pole", self.P("c", *T["utility_pole"]["pt"]), 0.25)
        self._built = None

    # -- trace / トレース
    def P(self, tile, px, py):
        la, lo = self.T["tiles"][tile]
        cx, cy = self.fr.xy(lo, la)
        return (cx + (px - self.T["centre"][0]) * self.T["mpp_x"], cy - (py - self.T["centre"][1]) * self.T["mpp_y"])

    def poly(self, d):
        return orient(Polygon([self.P(d["tile"], *q) for q in d["pts"]]).buffer(0), 1.0)

    def fit_store(self):
        """Store and annex: the traced rectangles cut 0.4 m clear of the road
        network, kept rectangular (fit_rect) -- the store keeps its north face
        (the shop front) and loses depth on the side-street side.
        店舗と倉庫: 道路から 0.4 m 離し矩形に収める（北面＝店先は動かさない）。"""
        keep = self.no_go.buffer(CLEAR_FOOT)
        for k in ("store", "annex"):
            d = self.T[k]
            seen = self.poly(d)
            q = seen.difference(keep).intersection(self.site)
            parts = [p for p in polys(q) if p.area > 1.0]
            if not parts:
                print("landmark %s: %s dropped (on the road network)" % (TAG, k))
                continue
            q = fit_rect(orient(max(parts, key=lambda p: p.area).simplify(0.02), 1.0))
            if q.area < seen.area - 0.3:
                self.cut[k] = seen.area - q.area
                print("landmark %s: %s cut back %.1f m2 of %.0f (roads win): %.1f x %.1f m -> %.1f x %.1f m"
                      % ((TAG, k, seen.area - q.area, seen.area) + self.dims(seen) + self.dims(q)))
            self.blocks[k] = dict(d, name=k, poly=q, seen=seen)

    def fit_point(self, what, p, r):
        """The nearest point to a traced one (within 4 m) where a thing of radius r
        stands GAP clear of the road network, on the lot; None if there is none.
        道路から離れる最寄りの位置（4 m 以内）。"""
        ok = self.site.buffer(-(GAP + r)).difference(self.no_go.buffer(GAP + r))
        q = Point(p)
        if ok.contains(q):
            return p
        if ok.is_empty or ok.distance(q) > 4.0:
            print("landmark %s: %s has no room clear of the roads, not built" % (TAG, what))
            return None
        from shapely.ops import nearest_points
        n = nearest_points(ok.buffer(-0.02), q)[0]
        print("landmark %s: %s moved %.1f m to stay clear of the road network" % (TAG, what, n.distance(q)))
        return (n.x, n.y)

    @staticmethod
    def dims(p):
        x0, y0, x1, y1 = p.bounds
        return (x1 - x0, y1 - y0)

    # -- what build.py needs / build.py 用
    def inner(self):
        return self.site

    def region(self):
        """The ground this module draws: the lot, already clear of every road,
        kerb, path and island. / 本モジュールが描く地面（道路・歩道を除いた敷地）。"""
        return self.site

    def ground_taken(self):
        return self.site

    def keepout(self):
        """No generic trees or street furniture on the lot. / 敷地内に汎用の木・街具を置かない。"""
        return self.site

    def ground_mesh(self, tile, z):
        self.build_all()
        return self._built["ground"].get(tile.bounds, Mesh())

    def buildings_mesh(self, tile):
        self.build_all()
        return self._built["bld"].get(tile.bounds, Mesh())

    def note(self):
        return "%d blocks" % len(self.blocks)

    # -- terrain / 地形
    def plan(self):
        """One level platform for the whole lot at the height of the main-road
        footway along its west edge (median), so the frontage is flat; the
        retaining walls follow from the ground falling away to the side street
        and the east lane. / 敷地全体を西端の歩道高さで一枚の造成面に。"""
        if getattr(self, "S", None) is not None:
            return self.S
        S = onterrain.Site(self.drape, self.region(), no_go=self.no_go, gap=GAP)
        x0, y0, x1, y1 = self.site.bounds
        front = self.site.intersection(box(x0, y0, x0 + 2.5, y1))
        self.z_front = S.level(front, "median") if not front.is_empty else S.level(self.site, "max")
        self.lot = S.platform("lot", self.site, z=self.z_front, kind="building")
        self.S = S
        self.z = self.lot.z
        return S

    # -- assembly / 組み立て
    def build_all(self):
        if self._built is not None:
            return
        import ground as G
        S = self.plan()
        gm, bm = Mesh(), Mesh()
        self.ground_surfaces(gm)
        S.walls(gm, mat="jp_retaining_kenchi", bank_mat="grass_park")
        self.block_courses(gm)
        st = Mesh()
        if "store" in self.blocks:
            self.store(st, self.blocks["store"])
        if "annex" in self.blocks:
            self.annex(st, self.blocks["annex"])
        self.door_things(st)
        self.ground_ac(st)
        self.lot_things(st)
        st.shift_z(self.z)
        bm.merge(st)
        self.fences(bm)
        self.kanban_pole(bm)
        self.utility_pole(bm)
        tiles = [tb for tn, tb in G.tiles(self.meta)]
        self._built = {"ground": split_tiles(gm, tiles), "bld": split_tiles(bm, tiles)}
        n = gm.ntris(), bm.ntris()
        walls = [w for w in S.walls_plan() if w[0] == "fill"]
        hmax = max([(w[6] if w[6] is not None else max(w[7], w[8])) - min(w[4], w[5]) - onterrain.FOOT
                    for w in walls] or [0.0])
        print("landmark %s: lot %.0f m2 at z %.2f (footway median), terrain %s; retaining walls up to %.1f m; "
              "ground %d, buildings %d triangles" % (TAG, self.lot.poly.area, self.z, S.report(), hmax, n[0], n[1]))
        for k, b in self.blocks.items():
            print("  %s: %.1f x %.1f m, %.1f m tall" % ((k,) + self.dims(b["poly"]) + (b["h"],)))

    # ================= ground / 地面 =================
    def ground_surfaces(self, m):
        S = self.S
        lot = self.lot.poly
        T = self.T
        st = self.blocks.get("store", {}).get("poly", Polygon())
        an = self.blocks.get("annex", {}).get("poly", Polygon())
        foot = unary_union([st, an])
        pad = self.poly(T["pad"]).intersection(lot).difference(foot.buffer(0.02))
        walk = st.buffer(T["walkway_m"]["north"], join_style="mitre").intersection(lot).difference(
            foot.buffer(0.0)).difference(pad)
        asph = lot.difference(foot).difference(pad).difference(walk)
        S.ground(m, "asphalt_lot", asph, uv_scale=6.0)
        S.ground(m, "seven_concrete", pad, uv_scale=3.0)
        S.ground(m, "concrete_path", walk, uv_scale=2.0)
        # under the store and annex: the floor slab (hidden, closes the ground) / 床下
        S.ground(m, "seven_concrete", foot.intersection(lot), uv_scale=3.0)
        # outside the platform: the footway-coloured strip at the entrance, weeds at the wall feet
        # 造成面の外: 入口の歩道色の帯、擁壁の足元の草
        free = S.free()
        x0, y0, x1, y1 = self.site.bounds
        entry = [box(x0 - 1, y0 - 1, x0 + 3.0, y1 + 1)]
        if not getattr(self, "drives", Polygon()).is_empty:
            entry.append(self.drives.buffer(1.0))        # paved where the driveway meets the lot / 私道の取付け部は舗装
        front = free.intersection(unary_union(entry))
        S.ground(m, "pavement_asphalt", front, uv_scale=3.0)
        S.ground(m, "grass", free.difference(front), uv_scale=4.0)
        self.paint(m)

    def paint(self, m):
        """Bay lines, the hatched aisle, the wheelchair symbol, the orange loading
        box, the footway edge line: paint on the lot (clipped to the platform).
        区画線・ゼブラ・車いす記号・荷捌き枠・歩道境界線。"""
        S = self.S
        Tp = self.T["paint"]
        P = lambda q: self.P("c", *q)
        lot = self.lot.poly.buffer(-0.05)
        white, orange = [], []
        # 18 bays: front row 8 (the hatched aisle is not one), front-east 3, north-east 3, west 4
        # 駐車枠 18（前列 8・東 3・北東 3・西 4）
        self.bays = []                       # (far-end centre, aisle-end centre, width) / 駐車枠
        for key in ("front_row", "front_east", "north_east"):
            d = Tp[key]
            ya, yb = d["y"]
            far, rnd = (yb, ya) if d["round"] == "y0" else (ya, yb)
            xs = d["x"]
            for x in xs:
                white.append(u_divider(P((x, far)), P((x, rnd))))
            for xa, xb in zip(xs, xs[1:]):
                if key == "front_row" and (xa, xb) == (750, 771):
                    continue                 # the hatched aisle / ゼブラ
                self.bays.append((P(((xa + xb) / 2, far)), P(((xa + xb) / 2, rnd)), abs(xb - xa) * self.T["mpp_x"]))
            if d.get("base"):
                white.append(LineString([P(q) for q in d["base"]]).buffer(0.06, cap_style="flat"))
        d = Tp["west_row"]
        xa, xb = d["x"]
        far, rnd = (xb, xa) if d["round"] == "x0" else (xa, xb)
        for y in d["y"]:
            white.append(u_divider(P((far, y)), P((rnd, y))))
        ys = [d["single"][0][1]] + d["y"]
        for ya, yb in zip(ys, ys[1:]):
            self.bays.append((P((far, (ya + yb) / 2)), P((rnd, (ya + yb) / 2)), abs(yb - ya) * self.T["mpp_y"]))
        white.append(LineString([P(q) for q in d["single"]]).buffer(0.06, cap_style="flat"))
        white.append(LineString([P(q) for q in d["base"]]).buffer(0.06, cap_style="flat"))
        # hatched aisle beside the wheelchair bay / 車いす区画横のゼブラ
        hz = self.poly(Tp["disabled_aisle"])
        white.append(hz.exterior.buffer(0.06, cap_style="flat"))
        hx0, hy0, hx1, hy1 = hz.bounds
        for yy in np.arange(hy0 + 0.45, hy1 - 0.2, 0.45):
            white.append(LineString([(hx0, yy), (hx1, yy)]).buffer(0.06, cap_style="flat").intersection(hz))
        white.append(wheelchair(P(Tp["wheelchair"]["pt"]), 1.6, up=(0.0, -1.0)))
        # orange loading box: outline and diagonal hatching / 荷捌き枠（橙）
        lb = self.poly(Tp["loading_box"])
        orange.append(lb.exterior.buffer(0.08, cap_style="flat").intersection(lb))
        bx0, by0, bx1, by1 = lb.bounds
        for k in np.arange(-(bx1 - bx0), by1 - by0, 0.9):
            orange.append(LineString([(bx0, by0 + k), (bx1, by0 + k + (bx1 - bx0))]).buffer(0.08).intersection(
                lb.buffer(-0.25)))
        # the painted edge of the footway / 歩道の境界線
        fl = self.T["footway_line"]
        white.append(LineString([P(q) for q in fl["pts"]]).buffer(0.075, cap_style="flat"))
        Wp = unary_union(white).intersection(lot)
        Op = unary_union(orange).intersection(lot).difference(Wp)
        S.ground(m, "line_white", Wp, dz=PAINT, uv_scale=1.0)
        S.ground(m, "seven_paint_orange", Op, dz=PAINT, uv_scale=1.0)

    def block_courses(self, m):
        """Concrete-block courses (0.4 m) on top of the herringbone retaining walls
        higher than 0.7 m, standing 3 cm proud (Street View south face), and one
        coping over every wall higher than 0.3 m: the strips of all wall pieces
        merged and drawn once (per-piece strips overlapped at the joints).
        擁壁上部のブロック積み（0.4 m、3 cm 出す）と笠木（一枚にまとめて描く）。"""
        strips, top = [], None
        for w in self.S.walls_plan():
            if w[0] != "fill" or w[6] is None:
                continue
            kind, a, b, n, za, zb, top = w[:7]
            h = top - max(za, zb) - onterrain.FOOT
            if h <= 0.3:
                continue
            out = 0.03 if h >= 0.7 else 0.0
            strips.append(Polygon([tuple(a - n * 0.2), tuple(b - n * 0.2), tuple(b + n * out), tuple(a + n * out)]))
            if out:
                m.wall("jp_block_wall", tuple(a + n * out), tuple(b + n * out), top - 0.4, top + 0.04, 0.0, 1.0)
        if strips:
            cap = unary_union([p.buffer(1e-4, join_style="mitre") for p in strips]).buffer(-1e-4, join_style="mitre")
            m.flat("seven_coping", cap, top + 0.04, 1.0)

    # ================= the store / 店舗 =================
    def room(self, a, b):
        """How far a facade on edge a-b may project before it nears a road or path. / 張り出し可能量。"""
        return max(0.0, self.no_go.distance(LineString([tuple(a), tuple(b)])) - ROOM)

    def room_part(self, f, u0, u1):
        """The same for the part u0..u1 of a facade (a canopy, a vending machine). / 区間ごとの張り出し可能量。"""
        return max(0.0, self.no_go.distance(LineString([f.P(u0, 0.0, 0.0)[:2], f.P(u1, 0.0, 0.0)[:2]])) - ROOM)

    def opening(self, f, u0, u1, z0, z1, frame, glass, depth=0.12, sill=True):
        """An opening with reveals in the frame colour and the glass set back. / 開口（枠色の見込み・奥のガラス）。"""
        d = -depth
        if z0 > 0.001:                       # a door's bottom reveal would lie on the floor / 床と重なる
            f.hplane(frame, u0, u1, d, 0.0, z0, True)
        f.hplane(frame, u0, u1, d, 0.0, z1, False)
        f.side(frame, u0, d, 0.0, z0, z1, 1)
        f.side(frame, u1, d, 0.0, z0, z1, -1)
        f.m.quad(glass, f.P(u0, d, z0), f.P(u1, d, z0), f.P(u1, d, z1), f.P(u0, d, z1), (0, 0), (1, 0), (1, 1), (0, 1))
        if sill:
            f.hplane(frame, u0 - 0.03, u1 + 0.03, 0.0, 0.05, z0, True)
            f.face(frame, u0 - 0.03, u1 + 0.03, z0 - 0.04, z0, 0.05)

    def shop_front(self, f, L, panes, door, wall_end):
        """A glazed shop front from u = panes[0][0] to wall_end along a face:
        kick plates under the panes, piers between, the door, panel above.
        ガラス張りの店先（腰板・方立・自動ドア・上部パネル）。"""
        ops = sorted(panes + ([door] if door else []))
        u = 0.0
        for a, b in ops:
            f.face(WALL_MAT, u, a, 0.0, DOOR_TOP, 0.0, 1.8)
            u = b
        f.face(WALL_MAT, u, wall_end, 0.0, DOOR_TOP, 0.0, 1.8)
        for a, b in panes:
            f.face("seven_skirt", a, b, 0.0, GLAZE[0], 0.0, 1.0)
            self.opening(f, a, b, GLAZE[0], GLAZE[1], "seven_alu", "seven_shop")
        if door:
            a, b = door
            self.opening(f, a, b, 0.0, DOOR_TOP, "seven_alu", "seven_door", sill=False)
            f.hplane("seven_mat", a, b, -0.12, 0.0, 0.015, True)    # clear of the floor slab / 床から離す

    def band(self, f, u0, u1, ends=(True, True), out=0.12):
        """The stripe fascia: a box 0.12 m proud with the stripes on its face. / ストライプの帯（0.12 m 出す）。"""
        z0, z1 = BAND
        f.m.quad("seven_band", f.P(u0, out, z0), f.P(u1, out, z0), f.P(u1, out, z1), f.P(u0, out, z1),
                 (u0, 0), (u1, 0), (u1, 1), (u0, 1))
        f.hplane(WALL_MAT, u0, u1, 0.0, out, z1, True)
        f.hplane("seven_white", u0, u1, 0.0, out, z0, False)
        if ends[0]:
            f.side("seven_band", u0, 0.0, out, z0, z1, -1)
        if ends[1]:
            f.side("seven_band", u1, 0.0, out, z0, z1, 1)

    def store(self, m, b):
        poly = b["poly"]
        H = b["h"]
        # north face: shop front, door near the west end, canopy / 北面: 店先・自動ドア・庇
        a, bb, L, n = edge_of(poly, 0, 1)
        f = B.Facade(m, a, bb, n)                  # u from the north-east corner / 北東角から
        s = lambda d: L - d                          # distance from the north-west corner / 北西角から
        back = min(L - 1.0, 16.1)
        panes = [(s(2.3), s(0.45))]
        w = (back - 4.5) / 6.0
        panes += [(s(4.5 + w * (k + 1)) + 0.04, s(4.5 + w * k) - 0.04) for k in range(6)]
        door = (s(4.4), s(2.4))
        self.shop_front(f, L, panes, door, L)
        # above the glazing, the back part (east) with the staff door / ガラス上部、東側の通用口
        f.face(WALL_MAT, 0.0, L, DOOR_TOP, BAND[0], 0.0, 1.8)
        f.face(WALL_MAT, 0.0, L, BAND[1], H, 0.0, 1.8)
        sd = s(18.3) if L > 19.5 else None
        if sd is not None:
            f.face("jp_door_steel", sd - 0.9, sd, 0.02, 2.05, 0.02)
        self.band(f, 0.0, L + 0.12, ends=(True, True))          # covers the corner / 角まで
        cu0, cu1 = s(5.0), s(1.8)
        dep = min(1.6, self.room_part(f, cu0, cu1))
        if dep > 0.6:
            f.block("seven_white", cu0, cu1, 0.0, dep, 2.85, 2.98, top=True, bottom=False)
            f.hplane("seven_soffit", cu0, cu1, 0.0, dep, 2.85, False)
        self.north = (a, bb, L, n)
        # west face: the shop front round the corner, then panel wall / 西面
        a, bb, Lw, n = edge_of(poly, -1, 0)
        f = B.Facade(m, a, bb, n)                  # u from the north-west corner / 北西角から
        wp = [(0.45, 2.45), (2.53, 4.5)] if Lw > 6.0 else []
        self.shop_front(f, Lw, wp, None, 4.6 if wp else 0.0)
        f.face(WALL_MAT, 4.6 if wp else 0.0, Lw, 0.0, DOOR_TOP, 0.0, 1.8)
        f.face(WALL_MAT, 0.0, Lw, DOOR_TOP, BAND[0], 0.0, 1.8)
        f.face(WALL_MAT, min(5.0, Lw), Lw, BAND[0], BAND[1], 0.0, 1.8)
        f.face(WALL_MAT, 0.0, Lw, BAND[1], H, 0.0, 1.8)
        self.band(f, 0.0, min(5.0, Lw), ends=(False, True))
        self.west = (a, bb, Lw, n)
        # south face: blank panels, downpipes / 南面: 窓無しのパネル壁と竪樋
        a, bb, Ls, n = edge_of(poly, 0, -1)
        f = B.Facade(m, a, bb, n)
        f.face(WALL_MAT, 0.0, Ls, 0.0, H, 0.0, 1.8)
        po = self.room(a, bb)
        if po > 0.16:
            for u in self.pipe_us(Ls):
                f.block("jp_pipe", u - 0.05, u + 0.05, 0.03, 0.13, 0.05, ROOF_Z + 0.2, top=True, bottom=False)
            f.block("seven_meter", Ls * 0.5, Ls * 0.5 + 0.5, 0.0, min(0.15, po - 0.02), 1.2, 1.9)
        # east face: panels, a vent / 東面
        a, bb, Le, n = edge_of(poly, 1, 0)
        f = B.Facade(m, a, bb, n)
        f.face(WALL_MAT, 0.0, Le, 0.0, H, 0.0, 1.8)
        f.block("seven_alu", Le * 0.5 - 0.3, Le * 0.5 + 0.3, 0.0, 0.08, 3.4, 3.8)
        self.roof(m, b)

    def roof(self, m, b):
        """Flat grey roof inside a 0.45 m parapet with a white coping; the
        L-shaped solar array on the west half, AC units, a roof hatch.
        陸屋根・パラペット・L 字の太陽光・室外機・点検口。"""
        poly, H = b["poly"], b["h"]
        inner = poly.buffer(-0.2, join_style="mitre")
        m.flat("seven_roof", inner, ROOF_Z, 3.0)
        for p in polys(inner):
            m.ring_walls(WALL_MAT, p.exterior, ROOF_Z, H, outward=False, uv_scale=1.8)
        m.flat("seven_white", poly.difference(inner), H, 1.0)
        x0, y0, x1, y1 = poly.bounds
        so = self.T["solar_m"]
        north = y1 - so["from_north"]
        west = x0 + so["from_west"]
        arr = unary_union([box(west, north - so["notch_depth"], west + so["length"] - so["notch_east"], north),
                           box(west, north - so["depth"], west + so["length"], north - so["notch_depth"])])
        arr = arr.intersection(inner.buffer(-0.35, join_style="mitre"))
        self.solar(m, arr)
        # AC condensers: on the ground along the south wall as in Street View when the
        # strip between the wall and the lot edge holds them, else on the roof
        # 室外機: 南壁沿いの地上（入らなければ屋上）
        self.ac_ground = self.south_ac(b)
        for k in range(0 if self.ac_ground else 5):
            cx = x0 + 6.0 + k * 1.3
            if inner.buffer(-0.5).contains(Point(cx, y0 + 0.75)):
                m.box("jp_ac_unit", cx, y0 + 0.75, ROOF_Z, ROOF_Z + 0.65, 0.85, 0.32, math.pi)
        for k in range(2):
            cx = x1 - 2.2 - k * 2.0
            if inner.buffer(-0.8).contains(Point(cx, y1 - 2.0)):
                m.box("seven_unit", cx, y1 - 2.0, ROOF_Z, ROOF_Z + 1.1, 1.3, 0.75, 0.0)
                m.box("seven_unit_top", cx, y1 - 2.0, ROOF_Z + 1.1, ROOF_Z + 1.16, 1.0, 0.6, 0.0)
        hx, hy = x1 - 2.0, y0 + 2.6
        if inner.buffer(-0.7).contains(Point(hx, hy)):
            m.box("seven_alu", hx, hy, ROOF_Z, ROOF_Z + 0.6, 0.9, 0.9)

    @staticmethod
    def pipe_us(Ls):
        """Where the downpipes stand along the south wall. / 南壁の竪樋の位置。"""
        return (0.6, Ls * 0.36, Ls * 0.68, Ls - 0.6)

    def south_ac(self, b):
        """Slots (u0, u1, w1, height) for the AC units on the ground by the south wall:
        five small condensers and a larger pair at the west end, each kept on the lot
        platform and ROOM clear of the road network; [] if fewer than four fit.
        南壁沿いの室外機の位置（造成面内・道路から離す）。"""
        lot = getattr(self, "lot", None)
        if lot is None:
            return []
        a, bb, Ls, n = edge_of(b["poly"], 0, -1)
        f = B.Facade(Mesh(), a, bb, n)
        ok = lot.poly.buffer(-0.08)
        want = [(0.85, 1.8, 0.45, 1.3), (1.9, 2.85, 0.45, 1.3)]
        want += [(3.2 + k * 1.15, 4.05 + k * 1.15, 0.38, 0.62) for k in range(3)]
        want += [(Ls * 0.68 + 0.25 + k * 1.15, Ls * 0.68 + 1.1 + k * 1.15, 0.38, 0.62) for k in range(2)]
        pipes = self.pipe_us(Ls)
        want = [w for w in want if all(u < w[0] - 0.12 or u > w[1] + 0.12 for u in pipes)]   # clear of the downpipes / 竪樋を避ける
        out = []
        for u0, u1, w1, h in want:
            if u1 > Ls - 0.3:
                continue
            q = Polygon([f.P(u0, 0.05, 0)[:2], f.P(u1, 0.05, 0)[:2], f.P(u1, w1, 0)[:2], f.P(u0, w1, 0)[:2]])
            if ok.contains(q) and self.no_go.distance(q) > ROOM:
                out.append((u0, u1, w1, h))
        return out if len(out) >= 4 else []

    def solar(self, m, area, tilt=10.0):
        """Panel rows (1.6 m deep, gaps 0.25 m) tilted to the south on low steel
        frames. / 南向きに傾けた太陽光パネルの列。"""
        if area.is_empty:
            return
        x0, y0, x1, y1 = area.bounds
        slope = math.tan(math.radians(tilt))
        y = y1
        while y - 1.6 >= y0 - 0.01:
            strip = box(x0, y - 1.6, x1, y).intersection(area)
            base = y - 1.6
            zf = (lambda xx, yy, base=base: ROOF_Z + 0.25 + slope * (yy - base))
            for p in polys(strip):
                if p.area > 1.0:
                    m.flat("solar", p, 0.0, 1.6, zf=zf)
                    m.flat("seven_alu", p, 0.0, 1.6, zf=lambda xx, yy, zf=zf: zf(xx, yy) - 0.03, up=False)
                    px0, py0, px1, py1 = p.bounds
                    for xx in np.arange(px0 + 0.3, px1, 2.0):
                        m.box("seven_alu", xx, py0 + 0.1, ROOF_Z, ROOF_Z + 0.24, 0.06, 0.06)
                        m.box("seven_alu", xx, py1 - 0.1, ROOF_Z, ROOF_Z + 0.24 + slope * 1.4, 0.06, 0.06)
            y -= 1.85

    def annex(self, m, b):
        """The storage box east of the store: panel walls, flat roof with a thin
        coping, a steel door facing east, a cooling unit on top. / 倉庫。"""
        poly, h = b["poly"], b["h"]
        m.ring_walls(WALL_MAT, poly.exterior, 0.0, h, outward=True, uv_scale=1.8)
        m.flat("seven_roof", poly, h, 3.0)
        m.prism("seven_white", poly.exterior.buffer(0.04, join_style="mitre").difference(poly.buffer(-0.04)),
                h, h + 0.12, top=True)
        a, bb, L, n = edge_of(poly, 1, 0)
        f = B.Facade(m, a, bb, n)
        if L > 2.5:
            f.face("jp_door_steel", L * 0.5 - 0.45, L * 0.5 + 0.45, 0.02, 2.0, 0.02)
        c = poly.centroid
        m.box("seven_unit", c.x, c.y + 1.5, h, h + 0.55, 1.0, 1.0)

    # ================= by the door, on the lot / 店先・駐車場 =================
    def door_things(self, m):
        """The recycling-bin station by the door, three vending machines on the
        west face, the large AC units at the south end of the west walkway.
        ゴミ箱ステーション・自販機 3 台・大型室外機。"""
        if "store" not in self.blocks:
            return
        a, bb, L, n = self.north
        f = B.Facade(m, a, bb, n)
        s = lambda d: L - d
        u0, u1 = s(7.3), s(5.3)
        dep = min(0.6, self.room_part(f, u0, u1))
        if dep > 0.4:
            f.block("seven_bins", u0, u1, 0.0, dep, 0.0, 1.05, top=False, bottom=False, ends=False)
            f.hplane("seven_alu", u0, u1, 0.0, dep, 1.05, True)
            f.side("seven_alu", u0, 0.0, dep, 0.0, 1.05, -1)
            f.side("seven_alu", u1, 0.0, dep, 0.0, 1.05, 1)
        a, bb, Lw, n = self.west
        f = B.Facade(m, a, bb, n)
        for k, mat in enumerate(("seven_vend_white", "seven_vend_red", "seven_vend_blue")):
            v0 = 5.2 + k * 0.98
            v1 = v0 + 0.95
            dep = min(0.8, self.room_part(f, v0, v1))
            if v1 > Lw - 0.3 or dep < 0.6:
                break
            f.m.quad(mat, f.P(v0, dep, 0.0), f.P(v1, dep, 0.0), f.P(v1, dep, 1.83), f.P(v0, dep, 1.83),
                     (0, 0), (1, 0), (1, 1), (0, 1))
            f.side("seven_vend_side", v0, 0.0, dep, 0.0, 1.83, -1)
            f.side("seven_vend_side", v1, 0.0, dep, 0.0, 1.83, 1)
            f.hplane("seven_vend_side", v0, v1, 0.0, dep, 1.83, True)
        # large AC units at the south end / 大型室外機
        for k in range(2):
            v1 = Lw - 0.25 - k * 1.0
            v0 = v1 - 0.92
            if v0 > 5.2 + 3 * 0.98 + 0.2 and self.room_part(f, v0, v1) > 0.45:
                f.block("jp_ac_unit", v0, v1, 0.05, 0.42, 0.1, 1.35)

    def ground_ac(self, m):
        """The AC units on the ground by the south wall (south_ac). / 南壁沿いの室外機。"""
        if "store" not in self.blocks or not getattr(self, "ac_ground", None):
            return
        a, bb, Ls, n = edge_of(self.blocks["store"]["poly"], 0, -1)
        f = B.Facade(m, a, bb, n)
        for u0, u1, w1, h in self.ac_ground:
            f.block("jp_ac_unit", u0, u1, 0.05, w1, 0.08, 0.08 + h)
            f.block("seven_alu", u0 + 0.05, u1 - 0.05, 0.08, w1 - 0.03, 0.0, 0.08, top=False, bottom=False)

    def lot_things(self, m):
        """Wheel stops in every bay (0.75 m from its far end). / 車止め。"""
        lot = self.lot.poly.buffer(-0.2)
        self.n_bays = 0
        for far, rnd, w in self.bays:
            far, rnd = np.asarray(far), np.asarray(rnd)
            t = (rnd - far) / np.linalg.norm(rnd - far)
            c = far + t * 0.75
            if not lot.contains(Point(*c)) or not lot.contains(Point(*(far + t * 3.0))):
                continue
            self.n_bays += 1
            ang = math.atan2(t[1], t[0]) + math.pi / 2
            for sgn in (-1, 1):
                q = c + np.array([math.cos(ang), math.sin(ang)]) * sgn * 0.45
                m.box("seven_carstop", q[0], q[1], 0.0, 0.11, 0.6, 0.14, ang)

    # ================= fences, kanban, pole / 柵・看板・電柱 =================
    def fence_lines(self):
        """Where the mesh fence runs: 0.15 m inside the lot edge on the south,
        east and north sides, not along the entrance (west), not where the
        store wall itself stands at the edge, gaps at the NE steps. Returns
        (north boundary runs, the rest), split segment by segment.
        フェンスの位置（入口・店舗壁際・階段を除く）、北境界とその他。"""
        lot = self.lot.poly
        ring = LineString(list(orient(lot.buffer(-0.15, join_style="mitre"), 1.0).exterior.coords))
        x0, y0, x1, y1 = self.site.bounds
        stop = [box(x0 - 1, y0 - 1, x0 + 3.5, y1 + 1), self.no_go.buffer(GAP + 0.1)]
        if not getattr(self, "drives", Polygon()).is_empty:
            stop.append(self.drives.buffer(GAP + 1.0))
        if "store" in self.blocks:
            stop.append(self.blocks["store"]["poly"].buffer(0.75))
        for p in (self.upole, self.kanban):
            if p is not None:
                stop.append(Point(p).buffer(0.8))
        g = ring.difference(unary_union(stop))
        runs = [q for q in getattr(g, "geoms", [g]) if q.geom_type == "LineString" and q.length > 1.0]
        north, rest = [], []
        for q in runs:
            c = list(q.coords)
            cur, kind = [c[0]], None
            for a, b in zip(c, c[1:]):
                k = "n" if (a[1] > y1 - 1.2 and b[1] > y1 - 1.2 and abs(b[0] - a[0]) > abs(b[1] - a[1])) else "r"
                if kind is not None and k != kind:
                    (north if kind == "n" else rest).append(LineString(cur))
                    cur = [a]
                cur.append(b)
                kind = k
            if len(cur) > 1:
                (north if kind == "n" else rest).append(LineString(cur))
        keep = lambda ls: [q for q in ls if q.length > 0.8]
        return keep(north), keep(rest)

    def fences(self, m):
        north, rest = self.fence_lines()
        z = self.z
        for q in north:
            # block wall 0.12 x 0.6 m with the mesh fence on it / ブロック塀＋メッシュフェンス
            c = [np.array(p) for p in q.coords]
            for a, b in zip(c, c[1:]):
                L = float(np.linalg.norm(b - a))
                if L < 1e-3:
                    continue
                t = (b - a) / L
                r = np.array([t[1], -t[0]]) * 0.06
                m.wall("jp_block_wall", tuple(a + r), tuple(b + r), z - 0.05, z + 0.6, 0.0, 1.0)
                m.wall("jp_block_wall", tuple(b - r), tuple(a - r), z - 0.05, z + 0.6, 0.0, 1.0)
            for p, d in ((c[0], c[1] - c[0]), (c[-1], c[-1] - c[-2])):
                t = d / max(np.linalg.norm(d), 1e-9)
                r = np.array([t[1], -t[0]]) * 0.06
                if p is c[0]:
                    m.wall("jp_block_wall", tuple(p - r), tuple(p + r), z - 0.05, z + 0.6, 0.0, 1.0)
                else:
                    m.wall("jp_block_wall", tuple(p + r), tuple(p - r), z - 0.05, z + 0.6, 0.0, 1.0)
            m.flat("seven_coping", q.buffer(0.07, cap_style="flat", join_style="mitre"), z + 0.6, 1.0)
            self.mesh_fence(m, q, z + 0.6, 1.2)
        for q in rest:
            self.mesh_fence(m, q, z + 0.02, 1.2)
        self.fence_len = sum(q.length for q in north + rest)

    def mesh_fence(self, m, line, z0, h, post=2.0, wire=0.2):
        """White welded-mesh fence: square posts, a top and a bottom rail, a fold
        rail, vertical wires (two-sided thin strips). / 白いメッシュフェンス。"""
        Lg = line.length
        posts = self.__dict__.setdefault("_posts", set())   # runs sharing an end share its post / 端の支柱は共有
        for k in range(int(Lg // post) + 1):
            q = line.interpolate(min(Lg, k * post))
            key = (round(q.x, 2), round(q.y, 2))
            if key in posts:
                continue
            posts.add(key)
            m.box("seven_fence", q.x, q.y, z0 - 0.02, z0 + h + 0.03, 0.06, 0.06)
        c = list(line.coords)
        for a, b in zip(c, c[1:]):
            seg = math.dist(a, b)
            if seg < 1e-3:
                continue
            t = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
            # the two sides of a strip 6 mm apart (rails) or 16 mm (wires): never coplanar,
            # also not where a wire crosses a rail (5 mm between their planes)
            # 両面を離す（レール ±3 mm、線 ±8 mm）
            A = lambda p, s, o=0.003: (p[0] + t[1] * o * s, p[1] - t[0] * o * s)
            for zz, hw in ((z0 + 0.06, 0.03), (z0 + h * 0.55, 0.02), (z0 + h - 0.03, 0.03)):
                m.wall("seven_fence", A(a, 1), A(b, 1), zz - hw, zz + hw, 0.0, 1.0)
                m.wall("seven_fence", A(b, -1), A(a, -1), zz - hw, zz + hw, 0.0, 1.0)
            for k in range(int(seg // wire) + 1):
                u = min(seg, k * wire)
                p0 = (a[0] + t[0] * (u - 0.006), a[1] + t[1] * (u - 0.006))
                p1 = (a[0] + t[0] * (u + 0.006), a[1] + t[1] * (u + 0.006))
                m.wall("seven_fence", A(p0, 1, 0.008), A(p1, 1, 0.008), z0, z0 + h, 0.0, 1.0)
                m.wall("seven_fence", A(p1, -1, 0.008), A(p0, -1, 0.008), z0, z0 + h, 0.0, 1.0)

    def kanban_pole(self, m):
        """The pole sign (kanban): round base, white post, a white box 1.3 x 0.45 m
        with the abstract brand face and the goods panels on its north and south
        faces. Material names avoid 'sign' (CARLA's importer drops them).
        ポール看板（素材名に sign を使わない）。"""
        if self.kanban is None:
            return
        x, y = self.kanban
        k = Mesh()
        zb = self.S.low(x, y, 0.5)
        k.cylinder("seven_coping", x, y, zb - 0.2, zb + 0.25, 0.5, seg=20)
        k.cylinder("seven_kanban_white", x, y, zb + 0.25, zb + 4.7, 0.21, seg=16, top=False)
        z0, z1, z2 = zb + 4.6, zb + 5.8, zb + 7.6
        sx, sy = 1.3, 0.45
        k.box("seven_kanban_white", x, y, z0, z2, sx, sy, 0.0, top=True, bottom=True)
        c = (x, y, (z0 + z2) / 2)
        for sgn in (1, -1):
            yy = y + sgn * (sy / 2 + 0.006)
            xa, xb = (x + sx / 2 - 0.05, x - sx / 2 + 0.05) if sgn > 0 else (x - sx / 2 + 0.05, x + sx / 2 - 0.05)
            oquad(k, "seven_kanban_brand", [(xa, yy, z1 + 0.05), (xb, yy, z1 + 0.05), (xb, yy, z2 - 0.05),
                                            (xa, yy, z2 - 0.05)], c)
            oquad(k, "seven_kanban_goods", [(xa, yy, z0 + 0.05), (xb, yy, z0 + 0.05), (xb, yy, z1 - 0.03),
                                            (xa, yy, z1 - 0.03)], c)
        m.merge(k)

    def utility_pole(self, m):
        """Concrete utility pole (12 m) with cross-arms, a pole-top transformer, the
        striped guard sleeve and a ground cabinet (Street View, SW corner).
        電柱（腕金・柱上変圧器・トラ柄カバー・地上機器）。"""
        if self.upole is None:
            return
        x, y = self.upole
        k = Mesh()
        zb = self.S.low(x, y, 0.3)
        k.cylinder("seven_pole_concrete", x, y, zb - 0.3, zb + 12.0, 0.19, seg=12, r1=0.13)
        k.cylinder("seven_tiger", x, y, zb + 0.0, zb + 1.8, 0.205, seg=12, top=False)
        # everything hangs towards the lot, away from the nearest road or path, and
        # the cross-arms stay 0.15 m off it / 道路から離れる向きに付け、腕金も 0.15 m 離す
        from shapely.ops import nearest_points
        q = nearest_points(self.no_go, Point(x, y))[0]
        v = np.array([x - q.x, y - q.y])
        v = v / max(np.linalg.norm(v), 1e-9)
        ang = math.atan2(v[1], v[0]) + math.pi / 2           # arms across v / 腕金は v に直交
        for zz, wd in ((zb + 11.2, 1.8), (zb + 10.5, 1.5)):
            half = wd / 2
            while half > 0.3 and self.no_go.distance(LineString(
                    [(x - math.cos(ang) * half, y - math.sin(ang) * half),
                     (x + math.cos(ang) * half, y + math.sin(ang) * half)])) < 0.2:
                half -= 0.05
            k.box("seven_steel", x, y, zz, zz + 0.09, 2 * half, 0.09, ang)
        tb = np.array([x, y]) + v * 0.25
        k.box("seven_steel", tb[0], tb[1], zb + 8.6, zb + 8.7, 0.5, 0.12, ang - math.pi / 2)
        tr = np.array([x, y]) + v * 0.5
        k.cylinder("seven_transformer", tr[0], tr[1], zb + 7.9, zb + 8.9, 0.27, seg=12)
        cb = np.array([x, y]) + v * 0.6
        if self.no_go.distance(Point(*cb)) > GAP + 0.5:
            k.box("seven_cabinet", cb[0], cb[1], zb, zb + 1.3, 0.55, 0.75, ang)
        m.merge(k)


def write_textures(tex_dir):
    """The store's drawn textures (procedural; colours from the photos, no
    imagery used; the brand only as abstract stripes and blocks). Each with
    roughness, metallic and normal maps. / 描画テクスチャ。"""
    import textures as X
    from PIL import Image, ImageDraw, ImageFont
    out = {}
    rs = np.random.default_rng(711)

    def save(name, im, rough=0.85, metal=0.0, relief=0.0, height=None):
        im = im.convert("RGB")
        im.save(os.path.join(tex_dir, name + ".png"))
        sq = im.size[0] == im.size[1]
        lum = lambda: (lambda a: a - a.mean())(np.asarray(im.convert("L"), np.float64) / 255.0)
        if relief > 0:
            h = height if height is not None else (X.height_of(im, 0.3) if sq else lum())
            X.normal_map(h / max(np.abs(h).max(), 1e-6), relief).save(os.path.join(tex_dir, name + "_n.png"))
        r = np.clip(rough + 0.05 * (X.height_of(im, 0.5) if sq else lum()), 0, 1)
        Image.fromarray((X.srgb_encode(np.asarray(r, np.float64)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_r.png"))
        Image.fromarray((X.srgb_encode(np.full((8, 8), metal)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_m.png"))
        out[name] = (name, None, rough, metal, False)

    def tinted(c0, c1, n=512, scales=(4, 16, 64)):
        return Image.fromarray(X.tint(X.noise(n, scales), c0, c1))

    ORANGE, GREEN, RED = (240, 120, 35), (0, 132, 95), (225, 35, 50)
    # white panel wall, vertical joints every tile (1.8 m), faint streaks / パネル壁
    n = 512
    a = np.asarray(tinted((232, 233, 230), (244, 245, 242), n, (8, 32, 128)), np.float64)
    for _ in range(10):
        x = int(rs.integers(0, n))
        a[:, x:x + int(rs.integers(2, 5))] *= 0.975
    a[:, :3] *= 0.8
    a[:, n - 2:] *= 0.88
    h = np.zeros((n, n))
    h[:, :3] = -1.0
    save("seven_panel", Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)), 0.6, 0.0, 0.8, h)
    save("seven_white", tinted((240, 240, 237), (250, 250, 248), 256), 0.55, 0.0)
    save("seven_soffit", tinted((236, 236, 232), (246, 246, 242), 256), 0.7, 0.0)
    save("seven_skirt", tinted((70, 72, 74), (92, 94, 96), 256), 0.45, 0.4)
    save("seven_alu", tinted((168, 172, 176), (196, 199, 202), 128), 0.35, 0.8)
    save("seven_coping", X.concrete(256, (178, 176, 170), (200, 198, 192), joints=2), 0.85, 0.0, 0.6)
    save("seven_concrete", X.concrete(512, (168, 166, 160), (192, 190, 184), joints=3, jc=(130, 128, 122)), 0.88, 0.0,
         0.7)
    save("seven_roof", tinted((200, 205, 201), (220, 225, 221), 512, (4, 16, 64)), 0.8, 0.0, 0.4)
    save("seven_mat", tinted((60, 62, 66), (80, 82, 86), 128), 0.95, 0.0)
    save("seven_fence", tinted((236, 236, 234), (248, 248, 246), 64, (2, 8, 32)), 0.4, 0.3)
    save("seven_carstop", tinted((215, 185, 40), (235, 205, 60), 128), 0.7, 0.0)
    save("seven_paint_orange", tinted((228, 120, 45), (240, 135, 60), 128), 0.7, 0.0)
    save("seven_steel", tinted((120, 122, 125), (150, 152, 155), 128), 0.45, 0.7)
    save("seven_pole_concrete", X.concrete(256, (170, 168, 162), (195, 193, 187)), 0.85, 0.0, 0.5)
    save("seven_transformer", tinted((150, 158, 160), (170, 178, 180), 128), 0.5, 0.5)
    save("seven_cabinet", tinted((150, 160, 150), (172, 182, 172), 128), 0.55, 0.4)
    save("seven_unit", tinted((200, 202, 198), (220, 222, 218), 128), 0.5, 0.3)
    save("seven_unit_top", tinted((60, 62, 64), (80, 82, 84), 128), 0.5, 0.3)
    save("seven_vend_side", tinted((205, 207, 210), (222, 224, 226), 128), 0.45, 0.4)
    save("seven_meter", tinted((190, 192, 194), (210, 212, 214), 128), 0.45, 0.4)
    # tiger-striped guard sleeve (yellow / black diagonals) / トラ柄
    im = Image.new("RGB", (256, 256), (235, 200, 30))
    d = ImageDraw.Draw(im)
    for k in range(-256, 512, 64):
        d.polygon([(k, 256), (k + 32, 256), (k + 288, 0), (k + 256, 0)], fill=(25, 25, 25))
    save("seven_tiger", im, 0.6, 0.0)
    # the stripe band: white with orange / green / red (v = z, one tile = 1 m)
    # ストライプ帯（白地に橙・緑・赤）
    W, Hh = 256, 512
    im = Image.new("RGB", (W, Hh), (247, 247, 245))
    d = ImageDraw.Draw(im)
    for c, y0, y1 in ((ORANGE, 0.34, 0.50), (GREEN, 0.54, 0.70), (RED, 0.74, 0.90)):
        d.rectangle([0, int(y0 * Hh), W, int(y1 * Hh)], fill=c)
    save("seven_band", im, 0.35, 0.0)
    # the shop seen through the glass: lit ceiling, shelves of goods, a poster
    # ガラス越しの店内（天井照明・商品棚・ポスター）
    W, Hh = 256, 320
    sh = Image.new("RGB", (W, Hh), (226, 228, 226))
    d = ImageDraw.Draw(sh)
    d.rectangle([0, 0, W, int(Hh * 0.22)], fill=(242, 242, 236))
    for x in range(20, W, 64):
        d.rectangle([x, int(Hh * 0.08), x + 34, int(Hh * 0.11)], fill=(255, 255, 250))
    for k in range(4):
        y = int(Hh * (0.34 + k * 0.15))
        d.rectangle([0, y, W, y + 6], fill=(150, 152, 155))
        x = 0
        while x < W:
            w = int(rs.integers(6, 16))
            col = tuple(int(v) for v in rs.integers(60, 240, 3))
            d.rectangle([x, y - int(rs.integers(16, 30)), x + w - 2, y], fill=col)
            x += w
    d.rectangle([int(W * 0.62), int(Hh * 0.24), int(W * 0.92), int(Hh * 0.46)], fill=(250, 245, 230))
    d.rectangle([int(W * 0.62), int(Hh * 0.40), int(W * 0.92), int(Hh * 0.46)], fill=ORANGE)
    a = np.asarray(sh, np.float64)
    sky = np.linspace(0.30, 0.08, Hh)[:, None, None]
    a = a * (1 - sky) + np.array([150, 175, 195]) * sky          # reflection / 映り込み
    a[:, :4] = a[:, -4:] = (175, 178, 182)
    save("seven_shop", Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)), 0.08, 0.3)
    # the automatic door: two leaves, frames, push bars, a stripe decal / 自動ドア
    dr = Image.new("RGB", (256, 320), (175, 178, 182))
    d = ImageDraw.Draw(dr)
    for x0, x1 in ((8, 124), (132, 248)):
        for yy in range(10, 300):
            t = yy / 300
            d.line([(x0, yy), (x1, yy)], fill=(int(200 - 70 * t), int(212 - 60 * t), int(215 - 55 * t)))
        d.rectangle([x0, 150, x1, 156], fill=ORANGE)
        d.rectangle([x0, 157, x1, 162], fill=GREEN)
        d.rectangle([x0, 163, x1, 168], fill=RED)
    d.rectangle([0, 300, 256, 320], fill=(165, 168, 172))
    save("seven_door", dr, 0.1, 0.4)
    # recycling-bin station: four slots with coloured labels / ゴミ箱
    bn = Image.new("RGB", (512, 256), (200, 203, 206))
    d = ImageDraw.Draw(bn)
    for k, col in enumerate(((60, 120, 200), (230, 170, 40), (60, 160, 90), (200, 60, 60))):
        x0 = 12 + k * 125
        d.rectangle([x0, 20, x0 + 110, 236], fill=(215, 218, 220), outline=(150, 152, 155), width=3)
        d.rectangle([x0 + 15, 40, x0 + 95, 75], fill=(25, 25, 28))
        d.rectangle([x0 + 15, 90, x0 + 95, 115], fill=col)
    save("seven_bins", bn, 0.45, 0.4)
    # vending machines: lit sample rows, buttons, the dark slot / 自販機
    for nm, body in (("seven_vend_white", (238, 238, 236)), ("seven_vend_red", (200, 30, 40)),
                     ("seven_vend_blue", (30, 80, 170))):
        v = Image.new("RGB", (256, 512), body)
        d = ImageDraw.Draw(v)
        d.rectangle([14, 24, 242, 300], fill=(235, 240, 245))
        for r in range(4):
            y = 40 + r * 64
            for c in range(6):
                x = 22 + c * 37
                col = tuple(int(q) for q in rs.integers(40, 230, 3))
                d.rectangle([x + 6, y, x + 26, y + 40], fill=col)
                d.rectangle([x + 10, y + 46, x + 22, y + 52], fill=(60, 200, 90))
        d.rectangle([150, 320, 230, 380], fill=(40, 40, 44))
        d.rectangle([30, 420, 226, 470], fill=(25, 25, 28))
        save(nm, v, 0.35, 0.2)
    # kanban: the brand face as abstract stripes and blocks (no logo) / 看板（抽象化、ロゴ無し）
    kb = Image.new("RGB", (260, 360), (248, 248, 246))
    d = ImageDraw.Draw(kb)
    d.rectangle([40, 40, 220, 250], fill=(248, 248, 246), outline=(225, 225, 222), width=4)
    d.rectangle([62, 70, 198, 112], fill=ORANGE)
    d.rectangle([62, 124, 198, 166], fill=GREEN)
    d.rectangle([62, 178, 198, 220], fill=RED)
    d.rectangle([20, 280, 240, 296], fill=ORANGE)
    d.rectangle([20, 302, 240, 318], fill=GREEN)
    d.rectangle([20, 324, 240, 340], fill=RED)
    save("seven_kanban_brand", kb, 0.3, 0.0)
    # the goods panels: red / green / red with white words / 取扱い品目パネル
    gp = Image.new("RGB", (390, 240), (245, 245, 243))
    d = ImageDraw.Draw(gp)
    words = ("たばこ", "酒", "ATM")
    for k, col in enumerate((RED, GREEN, RED)):
        x0 = 8 + k * 128
        d.rectangle([x0, 8, x0 + 118, 232], fill=col)
        try:
            f = ImageFont.truetype(CJK, 56 if len(words[k]) < 3 else 34)
            tw = d.textlength(words[k], font=f)
            d.text((x0 + (118 - tw) / 2, 90), words[k], fill=(255, 255, 255), font=f)
        except OSError:
            pass
    save("seven_kanban_goods", gp, 0.3, 0.0)
    save("seven_kanban_white", tinted((240, 240, 238), (250, 250, 248), 128), 0.35, 0.1)
    return out
