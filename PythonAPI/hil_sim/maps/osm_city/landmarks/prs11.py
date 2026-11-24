"""Pasir Ris Street 11 (Singapore): HDB blocks 130-137 in detail, the street's
frontage blocks and its furniture (maps/.venv; PasirRisTampines only).

Sources (2026-11-20, Photo Getter, build/PasirRisTampines/prs11_photos): the z20
satellite tiles (scale from tile overlap), OSM outlines checked on them, OneMap
3-D views and height tooltips (shape and height only), Street View on the street
and the estate roads (Mar 2025 / Oct 2022), the HDB register (storeys). Shapes in
build/PasirRisTampines/prs11_trace.json (tile pixels, as prcs_trace.json).
Nothing is copied as texture: every texture is drawn here.

  blocks 130-137  the 1993 executive-flat cluster: stepped blocks of offset
                  wings; buff/orange brick-tile courses with darker red courses,
                  a white floor band at every floor (it frames the planter
                  windows), the top storey white; window sides with planter
                  boxes, AC units, grey-framed windows; corridor sides with a
                  white slab edge, brick parapet and the recessed corridor (doors,
                  grilles, columns); end walls with the white bar strip or the
                  porthole column, portholes, red-ringed openings at the void
                  deck, green block plates, blue "1xx Pasir Ris St 11" lettering;
                  open void decks (white frame, brick piers, grey / blue roller
                  shutters), ground-floor flats at 135 and 136; flat roofs with
                  solar rows, white raised crowns over the lift cores with the
                  lift-motor rooms (+5.5 m), open crown frames, 133's pergola
                  and drop-off porch
  135A            the 4-deck car park: white slab edges, teal parapet panels and
                  orange bands, white columns, the stair core, solar canopies on
                  the roof deck, the entrance (barrier, no gantry over the road)
  137A            the precinct pavilion: red clay-tile hip on brick-and-white piers
  grounds         red clay-tile covered linkways on brick-and-white piers, the
                  east gateway (its piers; the portal over the estate road is left
                  out: roads win), herringbone-paved estate roads with double
                  yellow lines, the playground, the basketball court, the fitness
                  frame, the roundabout planter, bins, the precinct marker post,
                  the white perforated west wall with red-tile coping
  street          the frontage HDB blocks in their colour schemes (blue/white,
                  orange/white, red brick, teal/white; two car parks); the bus
                  shelters at the six stops (LTA grey, the blue-striped barrel
                  vault at 78179, the orange-striped roof at 78189) with chevron
                  bollards; green steel railings in the median at the White Sands
                  crossing and on the St 13 bend; chevron boards on the bend

Heights: HDB levels (void deck = level 1): 3.2 m void deck + 2.6 m per floor +
1.0 m parapet (7 levels 19.8 m, 8 levels 22.4 m, 9 levels 25.0 m; the trace's
"storey", chosen from the end walls in Street View and OneMap), lift-motor
rooms 5.5 m over the roof (OneMap 27 / 30 m). Roads win: footprints keep GAP off
the road network (the estate roads are OSM service roads) and a last pass drops
any triangle touching it (reported).

Coordinates: the map frame (x east, y north, metres).

パシリス・ストリート 11（シンガポール）: HDB 130-137 棟の詳細モデル、沿道の棟と道路付属物。
"""

import json
import math
import os

import numpy as np
from shapely.affinity import rotate, scale, translate
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from landmarks.dte import all_edges, beam, drop_over, gable, hip, largest, two_sided
from landmarks.prcs import text_poly
from mesh import Mesh, polys

NAME = "Pasir Ris Street 11"
TAG = "PRS11"
TRACE = "prs11_trace.json"
MAP = "PasirRisTampines"
Z0 = 0.15                 # the raised ground / 敷地面
GAP = 1.2                 # footprints this far off the road network / 道路網からの離れ
ROOM = 0.4                # projections this far off it / 張り出しの離れ
STREET = "Pasir Ris Street 11"
BRICK = "prs11_brick"
WHITE = "prs11_render"
CREAM = "prs11_cream"


def enabled(out, meta):
    """Only on PasirRisTampines, with its trace. / この地図とトレースがある時だけ。"""
    return meta.get("name") == MAP and os.path.exists(os.path.join(out, TRACE))


def _pts(T, fr, d):
    """Tile pixels -> map metres. / タイル画素 -> 地図座標。"""
    t = T["tiles"][d["tile"]]
    cx, cy = fr.xy(t["lon"], t["lat"])
    C = T["centre"]
    return [(cx + (px - C[0]) * t["mpp_x"], cy - (py - C[1]) * t["mpp_y"]) for px, py in d["pts"]]


def street_lines(L):
    """The OSM centre lines of Pasir Ris Street 11. / 通りの中心線。"""
    return [ln for ln, t, w in L.osm.lines("highway") if t.get("name") == STREET]


def site_area(L, out, T=None):
    """The area check_roads.py --site prs11 tests: the estate, the frontage blocks
    and the street's verges (no OSM area carries the name). / 道路確認の範囲。"""
    T = T or json.load(open(os.path.join(out, TRACE)))
    fr = geo.Frame(L.meta)
    est = Polygon(_pts(T, fr, T["site_outline"]))
    st = unary_union([ln.buffer(14.0) for ln in street_lines(L)])
    return unary_union([est, st]).buffer(0)


# -- primitives (tested in test_prs11.py: every face points out) / 基本形状（test_prs11 で向きを確認）
def vault(m, mat, p0, p1, r, z, seg=8, under=None):
    """A half-cylinder roof (barrel vault) over the line p0->p1 (2-D), springing
    at height z, radius r; faces up / out, the underside (``under``) faces down
    and in. / 半円筒の屋根（上面は外向き、裏面は内向き）。"""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    Ln = np.linalg.norm(d)
    if Ln < 1e-6:
        return
    t = d / Ln
    n = np.array([t[1], -t[0]])             # right of p0->p1 / 進行方向の右
    ring = []
    for k in range(seg + 1):
        a = math.pi * k / seg               # 0 = right side, pi = left side / 0 は右、pi は左
        ring.append((math.cos(a), math.sin(a)))
    for k in range(seg):
        (c0, s0), (c1, s1) = ring[k], ring[k + 1]
        a0 = (*(p0 + n * r * c0), z + r * s0)
        a1 = (*(p0 + n * r * c1), z + r * s1)
        b0 = (*(p1 + n * r * c0), z + r * s0)
        b1 = (*(p1 + n * r * c1), z + r * s1)
        u0, u1 = k / seg, (k + 1) / seg
        # outward: from the right edge up toward the left, along p0->p1 / 外向き
        m.quad(mat, b0, b1, a1, a0, (Ln, u0), (Ln, u1), (0, u1), (0, u0))
        if under:
            m.quad(under, a0, a1, b1, b0, (0, u0), (0, u1), (Ln, u1), (Ln, u0))


def bracket_window(f, a, b, z0, z1, frame, w=0.12, t=0.22):
    """A white frame standing out round a window (head, sill band, jambs): the
    floor band that steps up round the planter windows. / 窓を囲む白い枠。"""
    f.block(frame, a - t, b + t, 0.0, w, z1, z1 + t)
    f.block(frame, a - t, a, 0.0, w, z0, z1, top=False, bottom=False)
    f.block(frame, b, b + t, 0.0, w, z0, z1, top=False, bottom=False)


def texquad(f, mat, u0, u1, z0, z1, w):
    """A wall patch with the whole texture once (plates, portholes). / テクスチャ 1 枚の面。"""
    f.m.quad(mat, f.P(u0, w, z0), f.P(u1, w, z0), f.P(u1, w, z1), f.P(u0, w, z1), (0, 0), (1, 0), (1, 1), (0, 1))


def sub(f, u0, u1):
    """The part u0..u1 of a facade as its own facade. / 外壁の一部。"""
    return B.Facade(f.m, np.array(f.P(u0, 0, 0)[:2]), np.array(f.P(u1, 0, 0)[:2]), f.n)


def orect(c, size, bearing):
    """Rectangle (along, across) on the bearing (clockwise from north). / 方位に沿う矩形。"""
    a = math.radians(90 - bearing)
    co, si = math.cos(a), math.sin(a)
    L2, W2 = size[0] / 2, size[1] / 2
    return Polygon([(c[0] + co * u - si * v, c[1] + si * u + co * v)
                    for u, v in ((-L2, -W2), (L2, -W2), (L2, W2), (-L2, W2))])


def bearing_vec(b):
    a = math.radians(b)
    return np.array([math.sin(a), math.cos(a)])


class PRS11(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        S = T["storey"]
        self.VOID, self.FLOOR, self.PAR = S["void_deck"], S["floor"], S["parapet"]
        self.MOTOR, self.CROWN = S["motor_room"], S["crown"]
        pts = lambda d: _pts(T, fr, d)
        pt = lambda d: _pts(T, fr, d)[0]
        self.pts, self.pt = pts, pt
        # roads win: the road network, its islands and the estate's drives / 道路優先
        self.road = L.road_all
        self.hard = unary_union([L.road_all, L.islands])
        est = Polygon(pts(T["site_outline"])).difference(self.hard.buffer(0.05))
        self.estate = largest(est)
        near = self.estate.buffer(5.0)
        self.drive_snapshot = L.driveways
        self.foot_snapshot = L.footpaths
        self.cyc_snapshot = L.cycleways
        self.park_snapshot = L.parking
        self.no_go = unary_union([self.hard, L.driveways.intersection(near)])
        self.no_go_gap = self.no_go.buffer(GAP)
        self.room = self.no_go.buffer(ROOM)
        self.cut_log = []
        self.blocks = {}
        for k, b in T["blocks"].items():
            poly = orient(Polygon(pts(b)).buffer(0), 1.0)
            fit = self.fit(k, poly)
            if fit is None:
                continue
            d = dict(b)
            d["poly"] = orient(fit.simplify(0.15), 1.0)
            d["tower_polys"] = []
            for t in b["towers"]:
                tw = d["poly"].intersection(orect(pt(t), t["size"], b["bearing"]))
                # 0.35 m in: off the parapet's inner face (back-to-back faces render black)
                # パラペット内面から離す（背中合わせの面は黒くなる）
                tw = largest(tw.buffer(-0.35, join_style="mitre"))
                if tw is not None and tw.area > 12:
                    d["tower_polys"].append((tw, pt(t), t.get("motor")))
            if "porch" in b:
                d["porch_poly"] = Polygon(pts(b["porch"]))
            self.blocks[k] = d
        self.mscp = self.fit("135A", orient(Polygon(pts(T["135A"])), 1.0))
        self.pavilion = self.fit("137A", orient(Polygon(pts(T["137A"])), 1.0))
        foot = [b["poly"] for b in self.blocks.values()] + [g for g in (self.mscp, self.pavilion) if g is not None]
        self.cluster_foot = unary_union(foot)
        # Pasir Ris Street 11 frontage blocks (their cut-back generic outlines) / 沿道の棟
        self.frontage = []
        want = {}
        for sch, names in T["frontage"].items():
            for nm in names:
                want[nm] = sch
        for p, t in L.buildings:
            key = "%s %s" % (t.get("addr:housenumber", ""), t.get("addr:street", ""))
            if key in want and not self.estate.contains(p.representative_point()):
                g = self.fit("frontage " + key, orient(p, 1.0))
                if g is not None:
                    lv = T["frontage_levels"].get(t.get("addr:housenumber"), B.levels_of(t, 10) + 1)
                    self.frontage.append(dict(poly=orient(g.simplify(0.2), 1.0), scheme=want[key], levels=lv,
                                              no=t.get("addr:housenumber"), orig=p))
        self.footprint = unary_union([self.cluster_foot] + [f["poly"] for f in self.frontage])
        # build.py: generic buildings with their point in the site are replaced / 置換範囲
        self.site = unary_union([self.estate] + [f["orig"].buffer(0.5) for f in self.frontage]).buffer(0)
        self.linkways = [(LineString(pts(l)), l["w"], l["name"]) for l in T["linkways"]]
        self.gate_c = pt(T["gateway"])
        pg = T["playground"]
        self.play_c = pt(pg)
        self.playground = rotate(scale(Point(self.play_c).buffer(1.0, 24), pg["r"][0], pg["r"][1]),
                                 90 - pg["bearing"], origin=self.play_c).difference(self.no_go.buffer(0.6))
        self.playground = largest(self.playground)
        self.court = Polygon(pts(T["court"]))
        if self.court.intersects(self.no_go.buffer(0.5)) or not self.estate.buffer(-0.5).contains(self.court):
            self.court = None
        self.fit_c = pt(T["fitness"])
        self.fit_b = T["fitness"]["bearing"]
        self.play2 = Polygon(pts(T["play_e135A"])).difference(self.no_go.buffer(0.6)).difference(
            self.cluster_foot.buffer(1.0))
        self.round_c = pt(T["roundabout"])
        self.marker = pt(T["marker_post"])
        self.west_wall = LineString(pts(T["west_wall"]))
        self.items_clear = self.items_area()
        self.items_kinds = ("shelter", "linkway")
        self.dropped = 0
        self._parts = None
        self._ground = None
        print("prs11: %d blocks + 135A + 137A, %d frontage blocks (%s), %d linkways; cut back for roads: %s"
              % (len(self.blocks), len(self.frontage), " ".join(sorted(f["no"] for f in self.frontage)),
                 len(self.linkways),
                 ", ".join("%s %.0f m2" % c for c in self.cut_log if c[1] > 1.0) or "none"), flush=True)

    def fit(self, name, poly):
        """Roads win: the flat-roofed footprint is cut GAP off the road network
        (largest piece). / 道路優先: 道路網から GAP 離して切る。"""
        g = poly.difference(self.no_go_gap)
        self.cut_log.append((name, poly.area - g.area))
        p = largest(g)
        if p is None or p.area < 0.5 * poly.area:
            return None
        return orient(p, 1.0)

    # -- layout / 配置
    def inner(self):
        return self.estate.buffer(-0.6, join_style="mitre")

    def keepout(self):
        """Where the generic trees must not go. / 汎用の木を置かない所。"""
        k = [self.footprint.buffer(3.0), self.playground.buffer(1.5) if self.playground is not None else Polygon()]
        if self.court is not None:
            k.append(self.court.buffer(1.5))
        k += [ln.buffer(w / 2 + 1.5, cap_style="flat") for ln, w, n in self.linkways]
        k.append(self.west_wall.buffer(1.5))
        k += [Point(c).buffer(4.0) for c, a, s, ref in self.shelter_spots()]
        return unary_union(k)

    def items_area(self):
        """Generic furniture replaced here (bus shelters at the six stops, the
        covered walks of the estate): build.py drops items meeting it.
        汎用の小物を置き換える範囲（6 か所のバス停の上屋、団地の屋根付き通路）。"""
        k = [self.estate.buffer(-1.0)]
        k += [Point(c).buffer(3.5) for c, a, s, ref in self.shelter_spots()]
        return unary_union(k)

    def tree_filter(self, trees):
        """Generic street trees whose crown could sag below 4.5 m over the street:
        a tree 6.5 m or more from the carriageway keeps its natural crown height
        (props.py), and a rain tree's umbrella (R 6 m x size) reaches back over the
        road or an estate drive with its lowest leaves below 4.5 m (check_roads found
        4.46 m over a drive). Such trees in this site are left out. / 樹冠が通りの上で 4.5 m 未満に垂れる汎用の木を除く。"""
        # the area check_roads.py --site tests (site + 30 m) and a crown's reach / 確認範囲＋樹冠
        area = site_area(self.L, None, self.T).buffer(38.0)
        cw = self.L.carriageway
        # check_roads.py --site also counts the drives in and near the site as road / 敷地の車路も道路扱い
        dr = self.drive_snapshot.intersection(site_area(self.L, None, self.T).buffer(5.0))
        keep, gone = [], 0
        for t in trees:
            x, y, kind, s = t
            if kind in ("rain", "angsana") and area.contains(Point(x, y)):
                # props.tree_mesh: lowest leaves h0 + 0.55 H - 0.575 H, reach 1.09 R
                # 最下部の葉の高さと樹冠の届く距離
                R, h0, H = ((6.0, 4.6, 4.0) if kind == "rain" else (4.2, 4.6, 5.0))
                low = (h0 - 0.025 * H) * s
                d = cw.distance(Point(x, y))
                dd = min(d, dr.distance(Point(x, y))) if not dr.is_empty else d
                if d >= 6.5 - 0.01 and low < 4.6 and dd < 1.1 * R * s + 0.3:
                    gone += 1
                    continue
            keep.append(t)
        if gone:
            print("prs11: %d generic trees left out (crown would sag over the street)" % gone, flush=True)
        return keep

    def shelter_spots(self):
        """(centre, angle to the road, style, ref) of the St 11 bus shelters, placed
        as props.place_shelters does (3.2 m behind the kerb line). / バス停上屋の位置。"""
        if hasattr(self, "_spots"):
            return self._spots
        out = []
        L = self.L
        for p, t, nid in L.osm.points(lambda t: t.get("highway") == "bus_stop"):
            ref = t.get("ref")
            if ref not in self.T["bus_stops"]:
                continue
            # the kerb nearest the stop, on its off-road side (a node mapped in the lane
            # or a bus bay points the other way) / 停留所に最も近い縁石の歩道側
            rb = L.road_all.boundary
            e = rb.interpolate(rb.project(p))
            v = np.array([p.x - e.x, p.y - e.y])
            d = np.linalg.norm(v)
            if d < 1e-3:
                continue
            v /= d
            if L.road_all.contains(p):
                v = -v
            c = np.array([e.x, e.y]) + v * 3.0
            out.append(((float(c[0]), float(c[1])), math.atan2(v[1], v[0]), self.T["bus_stops"][ref], ref))
        self._spots = out
        return out

    # -- ground / 地面
    def zones(self):
        if self._ground is not None:
            return self._ground
        inner = self.inner().difference(self.hard.buffer(0.05))
        foot = self.cluster_foot
        drive = self.drive_snapshot.intersection(inner).difference(foot)
        park = self.park_snapshot.intersection(inner).difference(foot).difference(drive)
        path = unary_union([self.foot_snapshot, self.cyc_snapshot]).intersection(inner).difference(foot) \
            .difference(drive).difference(park)
        apron = foot.buffer(2.2, join_style="mitre").difference(foot).intersection(inner).difference(drive) \
            .difference(park)
        special = unary_union([g for g in (self.playground, self.court, self.play2) if g is not None])
        lw = unary_union([ln.buffer(w / 2, cap_style="flat") for ln, w, n in self.linkways]).intersection(inner) \
            .difference(drive).difference(foot)
        paved = unary_union([path, apron, lw]).difference(special)
        grass = inner.difference(foot).difference(drive).difference(park).difference(paved).difference(special)
        self._ground = dict(drive=drive, park=park, paved=paved, grass=grass, special=special)
        return self._ground

    def ground_mesh(self, tile, z):
        m = Mesh()
        if not self.estate.intersects(tile):
            return m
        Z = self.zones()
        T = lambda g: g.intersection(tile)
        m.flat("prs11_pavers", T(Z["drive"]), z + 0.01, 2.4)
        m.flat("asphalt_lot", T(Z["park"]), z + 0.008, 4.0)
        m.flat("concrete_path", T(Z["paved"]), z + 0.012, 2.0)
        m.flat("grass_park", T(Z["grass"]), z + 0.005, 4.0)
        # double yellow lines along both edges of the estate roads / 団地内道路の二重黄線
        lines = []
        for p in polys(Z["drive"]):
            for ring in [p.exterior] + list(p.interiors):
                for off in (0.3, 0.5):
                    q = Polygon(ring).buffer(-off) if ring is p.exterior else Polygon(ring).buffer(off)
                    for g in polys(q):
                        lines.append(g.exterior.buffer(0.05))
        yl = unary_union(lines).intersection(Z["drive"].buffer(-0.2)) if lines else Polygon()
        m.flat("paint_yellow", T(yl), z + 0.02, 1.0)
        if self.playground is not None and tile.contains(self.playground.centroid):
            m.flat("prs11_rubber", self.playground, z + 0.025, 3.0)
        if not self.play2.is_empty and tile.contains(self.play2.centroid):
            m.flat("prs11_rubber", self.play2, z + 0.025, 3.0)
        if self.court is not None and tile.contains(self.court.centroid):
            q = [np.array(p) for p in list(orient(self.court, 1.0).exterior.coords)[:4]]
            if math.dist(q[0], q[1]) < math.dist(q[1], q[2]):
                q = q[1:] + q[:1]
            m.quad("prs11_court", *[(p[0], p[1], z + 0.03) for p in q], (0, 0), (1, 0), (1, 1), (0, 1))
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

        def add(anchor, fn, *a, g=None, **k):
            pm = Mesh()
            fn(pm, *a, **k)
            nd = drop_over(pm, guard if g is None else g)
            if nd:
                self.drop_log.append((fn.__name__ + (" " + str(a[-1]) if a and isinstance(a[-1], str) else ""), nd))
            self.dropped += nd
            parts.append(((anchor.x, anchor.y) if hasattr(anchor, "x") else anchor, pm))
        for k, b in sorted(self.blocks.items()):
            add(b["poly"].centroid, self.hdb93, b["poly"], b, k)
        if self.mscp is not None:
            add(self.mscp.centroid, self.carpark, self.mscp, self.T["135A"])
        if self.pavilion is not None:
            add(self.pavilion.centroid, self.pavilion_137a, self.pavilion)
        for ln, w, nm in self.linkways:
            add(ln.interpolate(0.5, normalized=True), self.linkway, ln, w, nm)
        add(Point(self.gate_c), self.gateway)
        add(Point(self.play_c), self.grounds)
        add(self.west_wall.interpolate(0.5, normalized=True), self.boundary_wall)
        for f in self.frontage:
            add(f["poly"].centroid, self.frontage_block, f["poly"], f, f["no"])
        for c, ang, style, ref in self.shelter_spots():
            add(Point(c), self.shelter, c, ang, style, ref)
        # the median railings stand on the islands: only the carriageway and kerbs are off limits
        # 分離帯の柵は島の上: 車道と縁石のみ不可
        add(Point(self.pt(self.T["median_rails"][0])), self.median_rails, g=self.road.buffer(0.05))
        tot = sum(pm.ntris() for _, pm in parts)
        print("prs11: %d parts, %d triangles, %d triangles dropped over the road network (roads win)%s"
              % (len(parts), tot, self.dropped,
                 (": " + ", ".join("%s %d" % d for d in self.drop_log)) if self.drop_log else ""), flush=True)
        return parts

    # ---- HDB 1993 cluster block / 1993 年のクラスター棟
    def classify(self, poly, b):
        """Edge kinds: 'end' with its spec (named end walls), 'corridor' (long
        faces toward the estate roads), 'window', 'core' (short white strips),
        'brick' (short brick walls). / 辺の種類。"""
        edges = all_edges(poly)
        kinds = [None] * len(edges)
        cen = np.array(poly.centroid.coords[0])
        for spec in b.get("ends", []):
            v = bearing_vec(spec["bearing"])
            best, bi = -1e9, None
            for i, (a, bb, L, n, hole) in enumerate(edges):
                if hole or L < 4.0 or float(n @ v) < 0.75 or kinds[i] is not None:
                    continue
                score = float(((a + bb) / 2 - cen) @ v) + 2.0 * float(n @ v)
                if score > best:
                    best, bi = score, i
            if bi is not None:
                kinds[bi] = ("end", spec)
        drive = self.drive_snapshot
        for i, (a, bb, L, n, hole) in enumerate(edges):
            if kinds[i] is not None:
                continue
            mid = (a + bb) / 2
            if L < 4.5:
                kinds[i] = ("core", None)
            elif L < 7.5:
                kinds[i] = ("brick", None)
            else:
                q = Point(*(mid + n * 9.0))
                near_drive = drive.distance(q) < 5.0 and self.road.distance(Point(*(mid + n * 4.0))) > 6.0
                kinds[i] = ("corridor", None) if (near_drive or hole) else ("window", None)
        return edges, kinds

    def hdb93(self, m, poly, b, name):
        lv = b["levels"]
        V, F = self.VOID, self.FLOOR
        zr = Z0 + V + (lv - 1) * F
        edges, kinds = self.classify(poly, b)
        rng = np.random.default_rng(int(name[:3]))
        # void deck: floor, soffit, the walls set back round the lift lobbies and stores
        # ピロティ: 床・天井・奥の壁
        m.flat("void_floor", poly, Z0 + 0.01, 1.0)
        m.flat("concrete", poly, Z0 + V - 0.01, 2.0, up=False)
        core = poly.buffer(-2.6, join_style="mitre")
        for c in polys(core):
            for a, bb, L, n, hole in all_edges(c):
                f = B.Facade(m, a, bb, n)
                f.face(CREAM, 0.0, L, Z0, Z0 + V, 0.0, 2.0)
                for i, (u0, u1) in enumerate(B.bays(L, 4.0, 0.3)):
                    r = rng.random()
                    if r < 0.35:
                        f.face("prs11_shutter_blue" if r < 0.07 else "prs11_shutter_grey", u0 + 0.2, u1 - 0.2,
                               Z0, Z0 + 2.6, 0.02)
                    elif r < 0.55:
                        f.face("prs11_door_grey", u0 + 1.0, u0 + 2.0, Z0, Z0 + 2.2, 0.02)
        for i, (a, bb, L, n, hole) in enumerate(edges):
            f = B.Facade(m, a, bb, n)
            kind, spec = kinds[i]
            ground_flats = b.get("ground_flats") and kind == "window" and L > 10
            if kind == "end":
                self.end_wall(m, f, L, spec, b, name, zr)
                continue
            # void deck along the edge / 外周のピロティ
            shop = b.get("shops") and kind != "core" and L > 9 and float(n @ bearing_vec(40)) > 0.6
            if shop:
                # 136's shop units (the mini-mart) on its NE side: glazed fronts and a fascia
                # 136 北東側の店舗（ミニマート）: ガラスの店先と看板帯
                f.face("shopfront", 0.6, L - 0.6, Z0, Z0 + V - 0.6, -0.5, 4.0)
                f.side(CREAM, 0.6, -0.5, 0.0, Z0, Z0 + V - 0.6, 1)
                f.side(CREAM, L - 0.6, -0.5, 0.0, Z0, Z0 + V - 0.6, -1)
                f.hplane("concrete", 0.6, L - 0.6, -0.5, 0.0, Z0 + V - 0.6, False)
                f.block(WHITE, 0.6, L - 0.6, 0.0, 0.08, Z0 + V - 0.6, Z0 + V - 0.05, top=False, bottom=True)
                texquad(f, "prs11_fascia", 0.8, min(L - 0.8, 0.8 + 9.0), Z0 + V - 0.55, Z0 + V - 0.1, 0.09)
                f.block(BRICK, 0.0, 0.6, -0.6, 0.0, Z0, Z0 + V - 0.5, top=False, bottom=False)
                f.block(BRICK, L - 0.6, L, -0.6, 0.0, Z0, Z0 + V - 0.5, top=False, bottom=False)
            elif ground_flats:
                self.window_floor(f, 0.0, L, Z0, Z0 + V, BRICK, rng, low=True)
            else:
                for u in np.linspace(0.3, L - 0.3, max(2, int(L // 4.5) + 1)):
                    f.block(BRICK, u - 0.3, u + 0.3, -0.6, 0.0, Z0, Z0 + V - 0.5, top=False, bottom=False)
                f.block(WHITE, 0.0, L, -0.35, 0.05, Z0 + V - 0.55, Z0 + V, ends=False)
            for k in range(1, lv):
                z0 = Z0 + V + (k - 1) * F
                z1 = z0 + F
                top = k == lv - 1
                wall = WHITE if top else BRICK
                if kind == "core":
                    f.face(WHITE, 0.0, L, z0, z1, 0.0, 2.0)
                    if L > 1.6:
                        u = L / 2
                        f.window(u - 0.35, u + 0.35, z0 + 1.2, z0 + 1.9, WHITE, glass="prs11_win", mullions=0)
                elif kind == "brick":
                    ops = [(L / 2 - 0.35, L / 2 + 0.35, z0 + 1.2, z0 + 1.9)] if L > 3 else []
                    f.wall_with_openings(wall, z0, z1, ops, 1.2)
                    for o in ops:
                        f.window(*o, wall, glass="prs11_win", mullions=0)
                    f.block(WHITE, 0.0, L, 0.0, 0.12, z0 - 0.05, z0 + 0.25, ends=False)
                elif kind == "corridor":
                    self.corridor_floor(f, L, z0, z1, wall, top)
                elif any(float(f.n @ bearing_vec(r)) > 0.8 for r in b.get("ribbons", [])) and L > 8:
                    self.ribbon_floor(f, L, z0, z1, wall)
                else:
                    sun = any(float(f.n @ bearing_vec(s)) > 0.7 for s in b.get("sunshades", []))
                    self.window_floor(f, 0.0, L, z0, z1, wall, rng, sunshade=sun)
        # roof: parapet, solar rows, the raised white crowns and motor rooms / 屋上
        bld = B.Builder.__new__(B.Builder)
        bld.flat_roof(m, poly, zr, WHITE, self.PAR, "roof_concrete")
        towers = unary_union([t[0] for t in b["tower_polys"]]) if b["tower_polys"] else Polygon()
        solar = poly.buffer(-1.2, join_style="mitre").difference(towers.buffer(1.0))
        self.solar_rows(m, solar, zr, b["bearing"])
        for tw, c, motor in b["tower_polys"]:
            zc = zr + self.CROWN
            m.prism(WHITE, tw, zr, zc, top=False)
            bld.flat_roof(m, tw, zc, WHITE, self.PAR, "roof_concrete")
            if motor:
                mr = tw.intersection(orect(c, motor, b["bearing"]))
                mr = largest(mr.buffer(-0.3, join_style="mitre"))
                if mr is not None and mr.area > 6:
                    m.prism(WHITE, mr, zc, zr + self.MOTOR, top=True)
                    for a, bb, L, n, hole in all_edges(mr):
                        f = B.Facade(m, a, bb, n)
                        if L > 2.5:
                            f.face("louvre", 0.5, L - 0.5, zr + self.MOTOR - 1.6, zr + self.MOTOR - 0.6, 0.02)
                    rc = mr.representative_point()
                    m.box("tank", rc.x, rc.y, zr + self.MOTOR, zr + self.MOTOR + 1.2, 2.2, 1.6,
                          math.radians(90 - b["bearing"]))
        if b.get("porch_poly") is not None:
            self.porch(m, b["porch_poly"])

    def solar_rows(self, m, poly, z, bearing, pitch=2.4, w=1.7):
        """Rows of solar panels on the flat roof (strips on a low frame). / 屋上の太陽光パネル列。"""
        if poly is None or poly.is_empty:
            return
        c = poly.centroid
        ang = 90 - bearing
        rc = rotate(poly, -ang, origin=c)
        x0, y0, x1, y1 = rc.bounds
        rows = []
        y = y0 + 0.3
        while y + w < y1:
            rows.append(box(x0 - 1, y, x1 + 1, y + w))
            y += pitch
        g = rotate(unary_union(rows), ang, origin=c).intersection(poly)
        g = unary_union([p for p in polys(g) if p.area > 1.5])
        if g.is_empty:
            return
        m.flat("solar", g, z + 0.45, 1.0)
        m.flat("metal_grey", g.buffer(-0.02), z + 0.42, 1.0, up=False)
        for p in polys(g):
            m.ring_walls("metal_grey", p.exterior, z + 0.32, z + 0.45, outward=True)

    def window_floor(self, f, u0, u1, z0, z1, wall, rng, low=False, sunshade=False):
        """One floor of the window side: planter windows framed by the white band,
        narrow windows with AC units, the floor band. / 窓側 1 層分。"""
        s = sub(f, u0, u1) if (u0 > 0 or abs(u1 - f.L) > 1e-6) else f
        L = s.L
        ops, det = [], []
        for i, (a, b) in enumerate(B.bays(L, 3.2, 0.5)):
            mid = (a + b) / 2
            k = i % 3
            if k == 0:
                ops.append((mid - 1.1, mid + 1.1, z0 + 0.75, z0 + 2.25))
            elif k == 1:
                ops.append((mid - 1.2, mid - 0.2, z0 + 1.0, z0 + 2.25))
            else:
                ops.append((mid - 0.8, mid + 0.8, z0 + 0.95, z0 + 2.25))
            det.append((k, mid))
        s.wall_with_openings(wall, z0, z1, ops, 1.2)
        for (a, b, oz0, oz1), (k, mid) in zip(ops, det):
            s.window(a, b, oz0, oz1, wall, glass="prs11_win", mullions=2 if b - a > 1.8 else 1 if b - a > 1.2 else 0)
            if k == 0:
                # planter box under the wide window, the white band framing it / プランターと白枠
                s.block(WHITE, a - 0.05, b + 0.05, 0.0, 0.42, z0 + 0.3, z0 + 0.72)
                s.hplane("soil", a, b, 0.05, 0.37, z0 + 0.73, True)
                bracket_window(s, a, b, z0 + 0.25, oz1 + 0.05, WHITE)
            elif k == 1:
                # AC unit on its bracket beside the narrow window / 室外機
                s.block("prs11_ac", b + 0.25, b + 1.05, 0.05, 0.55, z0 + 1.1, z0 + 1.7)
                s.block("metal_grey", b + 0.25, b + 1.05, 0.0, 0.58, z0 + 1.04, z0 + 1.1, top=True, bottom=True)
            if sunshade:
                s.block(WHITE, a - 0.3, b + 0.3, 0.0, 0.7, oz1 + 0.06, oz1 + 0.16)
        if not low:
            s.block(WHITE, 0.0, L, 0.0, 0.12, z0 - 0.05, z0 + 0.25, ends=False)

    def ribbon_floor(self, f, L, z0, z1, wall):
        """Ribbon windows (132's St 11 face, 136's south face): 2.4 m panes between
        narrow piers, a continuous white sill ledge and head band.
        連続窓（132 の通り側、136 の南面）: 細い柱の間の窓、連続した窓台と白帯。"""
        n = max(1, int((L - 1.6) // 2.65))
        w = (L - 1.6) / n
        ops = [(0.8 + i * w + 0.12, 0.8 + (i + 1) * w - 0.12, z0 + 0.95, z0 + 2.2) for i in range(n)]
        f.wall_with_openings(wall, z0, z1, ops, 1.2)
        for o in ops:
            f.window(*o, wall, glass="prs11_win_3", mullions=0, depth=0.18)
        f.block(WHITE, 0.6, L - 0.6, 0.0, 0.35, z0 + 0.8, z0 + 0.95)
        f.block(WHITE, 0.6, L - 0.6, 0.0, 0.12, z0 + 2.2, z0 + 2.42)
        f.block(WHITE, 0.0, L, 0.0, 0.12, z0 - 0.05, z0 + 0.25, ends=False)

    def corridor_floor(self, f, L, z0, z1, wall, top, D=1.7):
        """The corridor side: white slab edge, brick parapet, the recessed common
        corridor with doors, grilles and columns. / 廊下側 1 層分。"""
        f.face(WHITE, 0.0, L, z0, z0 + 0.3, 0.0)
        f.block(WHITE if top else BRICK, 0.0, L, -0.15, 0.0, z0 + 0.3, z0 + 1.05, top=True, bottom=False,
                ends=False)
        f.face(CREAM, 0.0, L, z0 + 1.05, z1, -D, 2.0)
        f.hplane("concrete", 0.0, L, -D, 0.0, z1 - 0.001, False)
        f.hplane("void_floor", 0.0, L, -D, -0.15, z0 + 0.3, True)
        for u0, u1 in B.bays(L, 6.4, 0.4):
            mid = (u0 + u1) / 2
            f.face("door_wood", mid - 1.6, mid - 0.7, z0 + 0.3, z0 + 2.4, -D + 0.02)
            f.face("prs11_grille", mid - 0.4, mid + 1.6, z0 + 1.1, z0 + 2.2, -D + 0.02)
        for u in np.linspace(0.2, L - 0.2, max(2, int(L // 6.4) + 1)):
            f.block(WHITE, u - 0.2, u + 0.2, -D, 0.0, z0 + 1.05, z1, top=False, bottom=False)
        f.block("railing", 0.0, L, -0.1, -0.05, z0 + 1.05, z0 + 1.1, ends=False)

    def end_wall(self, m, f, L, spec, b, name, zr):
        """A named end wall: the white bar strip or porthole column between brick
        sides with white bands, portholes on the top brick storey, the void deck
        closed with the lobby, plate, shutters and red-ringed openings, the blue
        lettering high up, an open crown frame or 133's pergola.
        妻壁: 白い帯（格子模様または丸窓）、丸窓、ピロティの入口・棟番号・シャッター、文字。"""
        V, F, lv = self.VOID, self.FLOOR, b["levels"]
        motif = spec.get("motif", "brick")
        sw = {"bars": min(4.6, L * 0.4), "portholes": min(2.2, L * 0.3), "core": min(3.2, L * 0.4)}.get(motif, 0.0)
        a0, a1 = L / 2 - sw / 2, L / 2 + sw / 2
        # void deck: closed white wall with the lobby opening, or the shutter room / ピロティ
        zt = Z0 + V
        if spec.get("plate") or spec.get("shutter") or spec.get("rings"):
            lob0, lob1 = L / 2 - 1.3, L / 2 + 1.3
            f.face(WHITE, 0.0, lob0, Z0, zt, 0.0, 2.0)
            f.face(WHITE, lob1, L, Z0, zt, 0.0, 2.0)
            f.face(WHITE, lob0, lob1, Z0 + 2.7, zt, 0.0, 2.0)
            f.side(WHITE, lob0, -1.5, 0.0, Z0, Z0 + 2.7, 1)
            f.side(WHITE, lob1, -1.5, 0.0, Z0, Z0 + 2.7, -1)
            f.hplane("concrete", lob0, lob1, -1.5, 0.0, Z0 + 2.7, False)
            f.face("prs11_door_grey" if spec.get("shutter") is None else "prs11_shutter_" + spec["shutter"],
                   lob0, lob1, Z0, Z0 + 2.7, -1.5)
            for u in (0.25, L - 0.25):
                f.block(BRICK, u - 0.3, u + 0.3, 0.0, 0.1, Z0, zt - 0.5, top=False, bottom=False)
            if spec.get("rings") and L > 7.5:
                for u in (lob0 - 1.3, lob1 + 1.3):
                    texquad(f, "prs11_ring", u - 0.9, u + 0.9, Z0 + 0.6, Z0 + 2.4, 0.02)
            elif spec.get("shutter") and L > 7.5:
                texquad(f, "prs11_shutter_" + spec["shutter"], 0.6, lob0 - 0.4, Z0, Z0 + 2.6, 0.02)
            if spec.get("plate"):
                texquad(f, "prs11_plate_" + name, L / 2 - 0.45, L / 2 + 0.45, Z0 + 2.75, Z0 + 3.3, 0.06)
                f.block(WHITE, L / 2 - 0.5, L / 2 + 0.5, 0.0, 0.05, Z0 + 2.72, Z0 + 3.33, top=True, bottom=True)
            f.block(WHITE, 0.0, L, -0.1, 0.05, zt - 0.55, zt, ends=False)
        else:
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 4.5) + 1)):
                f.block(BRICK, u - 0.3, u + 0.3, -0.6, 0.0, Z0, zt - 0.5, top=False, bottom=False)
            f.block(WHITE, 0.0, L, -0.35, 0.05, zt - 0.55, zt, ends=False)
        for k in range(1, lv):
            z0 = Z0 + V + (k - 1) * F
            z1 = z0 + F
            top = k == lv - 1
            wall = WHITE if top else BRICK
            sides = [(0.0, a0), (a1, L)] if sw > 0 else [(0.0, L)]
            port_row = spec.get("portholes") and k == lv - 2
            for u0, u1 in sides:
                if u1 - u0 < 0.2:
                    continue
                if port_row and u1 - u0 > 2.6:
                    n = 3
                    us = np.linspace(u0 + 0.8, u1 - 0.8, n)
                    ops = []
                    f.face(wall, u0, u1, z0, z1, 0.0, 1.2)
                    for u in us:
                        texquad(f, "prs11_port_b", u - 0.4, u + 0.4, z0 + 1.0, z0 + 1.8, 0.03)
                elif motif == "core" and u1 - u0 > 1.5:
                    um = (u0 + u1) / 2
                    ops = [(um - 0.35, um + 0.35, z0 + 1.2, z0 + 1.9)]
                    s = sub(f, u0, u1)
                    s.wall_with_openings(wall, z0, z1, [(o[0] - u0, o[1] - u0, o[2], o[3]) for o in ops], 1.2)
                    for o in ops:
                        s.window(o[0] - u0, o[1] - u0, o[2], o[3], wall, glass="prs11_win", mullions=0)
                else:
                    f.face(wall, u0, u1, z0, z1, 0.0, 1.2)
                f.block(WHITE, u0, u1, 0.0, 0.12, z0 - 0.05, z0 + 0.25, ends=False)
            if sw > 0:
                if motif == "bars":
                    texquad(f, "prs11_bars", a0, a1, z0, z1, 0.08)
                    f.side(WHITE, a0, 0.0, 0.08, z0, z1, -1)
                    f.side(WHITE, a1, 0.0, 0.08, z0, z1, 1)
                elif motif == "portholes":
                    f.face(WHITE, a0, a1, z0, z1, 0.08, 2.0)
                    texquad(f, "prs11_port_w", L / 2 - 0.35, L / 2 + 0.35, z0 + 1.05, z0 + 1.75, 0.1)
                    f.side(WHITE, a0, 0.0, 0.08, z0, z1, -1)
                    f.side(WHITE, a1, 0.0, 0.08, z0, z1, 1)
                else:
                    f.face(WHITE, a0, a1, z0, z1, 0.0, 2.0)
                    for u in (L / 2 - 0.7, L / 2 + 0.7):
                        f.window(u - 0.3, u + 0.3, z0 + 1.2, z0 + 1.8, WHITE, glass="prs11_win", mullions=0)
        if sw > 0:
            f.hplane(WHITE, a0, a1, 0.0, 0.08, zr, True)
        if spec.get("letters"):
            self.letters(m, f, spec["letters"], L, zr - F - 1.3, 0.62)
        if spec.get("crown"):
            # open white frame over the end wall / 妻壁上の白い枠
            w = -0.5
            for u in (max(0.4, a0 - 0.6), min(L - 0.4, a1 + 0.6)):
                beam(m, WHITE, f.P(u, w, zr + self.PAR), f.P(u, w, zr + self.PAR + 2.6), 0.18)
            beam(m, WHITE, f.P(max(0.4, a0 - 0.8), w, zr + self.PAR + 2.6), f.P(min(L - 0.4, a1 + 0.8), w,
                 zr + self.PAR + 2.6), 0.2, caps=True)
            beam(m, WHITE, f.P(max(0.4, a0 - 0.6), w, zr + self.PAR + 1.4), f.P(min(L - 0.4, a1 + 0.6), w,
                 zr + self.PAR + 1.4), 0.12, caps=True)
        if spec.get("pergola"):
            # 133: white pergola beams over the top storeys at the SW end / 133 南西端のパーゴラ
            for k in range(6):
                u = 0.6 + k * (L - 1.2) / 5
                beam(m, WHITE, f.P(u, -3.5, zr + self.PAR + 0.2), f.P(u, -0.2, zr + self.PAR + 0.2), 0.12, caps=True)
            beam(m, WHITE, f.P(0.4, -0.4, zr + self.PAR + 0.35), f.P(L - 0.4, -0.4, zr + self.PAR + 0.35), 0.14,
                 caps=True)

    def letters(self, m, f, text, L, z, h):
        """Blue italic lettering with a white outline, standing off the wall, at
        most L - 1.5 m long. / 白縁の青い斜体文字。"""
        from mesh import earcut
        g = text_poly(text, h, "DejaVuSans-BoldOblique.ttf")
        wd = g.bounds[2] - g.bounds[0]
        if wd > L - 1.5:
            g = scale(g, (L - 1.5) / wd, (L - 1.5) / wd, origin=(0, 0))
        for geom, mat, w in ((g.buffer(0.05, join_style="mitre").simplify(0.01), WHITE, 0.05),
                             (g, "paint_royalblue", 0.08)):
            for p in polys(geom):
                p = orient(p, 1.0)
                rings = [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]
                Vv = np.concatenate(rings)
                ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
                I = earcut.triangulate_float64(Vv.astype(np.float64), ends).reshape(-1, 3)
                P3 = [f.P(L / 2 + q[0], w, z + h / 2 + q[1]) for q in Vv]
                A = np.array([P3[i] for i in I.ravel()]).reshape(-1, 3, 3)
                nrm = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
                flip = (nrm[:, :2] @ f.n) < 0
                I = np.where(flip[:, None], I[:, ::-1], I)
                m.tris(mat, P3, np.r_[f.n, 0.0], Vv / h, I)

    def porch(self, m, poly):
        """133's drop-off porch: an open pitched timber-look roof on two round
        columns, the crest disc. / 133 の車寄せ: 木調の切妻屋根と丸柱 2 本。"""
        poly = largest(poly.difference(self.room.buffer(0.6)))
        if poly is None or poly.area < 6:
            return
        a, b, c, d = [np.array(q) for q in list(orient(poly.minimum_rotated_rectangle, 1.0).exterior.coords)[:4]]
        far = sorted([a, b, c, d], key=lambda q: -self.cluster_foot.distance(Point(*q)))[:2]
        for q in far:
            ctr = poly.centroid
            v = np.array([ctr.x, ctr.y]) - q
            v = v / max(np.linalg.norm(v), 1e-6) * 0.5
            m.cylinder("paint_white", q[0] + v[0], q[1] + v[1], Z0, Z0 + 3.3, 0.2, seg=10, top=False)
        gable(m, poly, Z0 + 3.3, Z0 + 4.5, "deck_wood", WHITE, ov=0.0)

    # ---- 135A car park / 立体駐車場
    def carpark(self, m, poly, spec):
        H = 3.1
        nd = spec.get("decks", 4)
        ztop = Z0 + nd * H
        inner = poly.buffer(-1.2, join_style="mitre")
        m.flat("asphalt_lot", poly, Z0 + 0.01, 6.0)
        for k in range(1, nd + 1):
            z = Z0 + k * H
            m.flat("concrete", poly, z - 0.35, 2.0, up=False)
            m.flat("asphalt_lot" if k < nd else "roof_concrete", poly, z, 6.0)
        for c in polys(inner):
            m.ring_walls("corridor_dark", c.exterior, Z0, ztop, outward=True, uv_scale=3.0)
        entry = bearing_vec(spec.get("entry_bearing", 31))
        for a, b, L, n in B.edges_of(poly):
            f = B.Facade(m, a, b, n)
            ent = float(n @ entry) > 0.8
            for k in range(1, nd + 1):
                z = Z0 + k * H
                f.face(WHITE, 0.0, L, z - 0.35, z, 0.0)
                f.block("prs11_orange", 0.0, L, -0.12, 0.0, z, z + 0.25, top=False, bottom=False, ends=False)
                for u0, u1 in B.bays(L, 4.0, 0.3):
                    f.block("prs11_teal", u0 + 0.15, u1 - 0.15, -0.1, 0.0, z + 0.25, z + 1.05, top=True,
                            bottom=False)
            if not ent:
                # the ground floor: a low wall between the columns / 1 階の腰壁
                f.block(WHITE, 0.0, L, -0.15, 0.0, Z0, Z0 + 0.9, top=True, bottom=False, ends=False)
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 8.0) + 1)):
                f.block(WHITE, u - 0.3, u + 0.3, -0.5, 0.05, Z0, ztop + 1.0, top=True, bottom=False)
            if ent and L > 20:
                # entrance: barrier box and arm beside the ramp (no gantry over the road) / 入口のバー
                u = L * 0.5
                f.block("paint_yellow", u + 3.4, u + 3.8, 0.5, 0.9, Z0, Z0 + 1.1)
                f.block("paint_red", u + 0.4, u + 3.4, 0.62, 0.72, Z0 + 0.95, Z0 + 1.05)
        # parapet on the roof deck, the white stair core, solar canopies / 屋上の立ち上がり・階段室・太陽光屋根
        a, b, c, d = [np.array(q) for q in list(orient(poly.minimum_rotated_rectangle, 1.0).exterior.coords)[:4]]
        if np.linalg.norm(b - a) < np.linalg.norm(d - a):
            a, b, c, d = b, c, d, a
        ax = (b - a) / np.linalg.norm(b - a)
        sd = (d - a) / np.linalg.norm(d - a)
        W = np.linalg.norm(d - a)
        Lr = np.linalg.norm(b - a)
        core = Polygon([a + sd * 1.0, a + ax * 6.0 + sd * 1.0, a + ax * 6.0 + sd * 8.0, a + sd * 8.0])
        m.prism(WHITE, core.intersection(poly), Z0, ztop + 3.0, top=True)
        for t0 in (0.2, 0.5):
            p0 = a + ax * 9.0 + sd * (W * t0)
            strip = Polygon([p0, p0 + ax * (Lr - 12.0), p0 + ax * (Lr - 12.0) + sd * (W * 0.26),
                             p0 + sd * (W * 0.26)]).intersection(poly.buffer(-1.0))
            if strip.is_empty:
                continue
            zs = ztop + 3.2
            m.flat("solar", strip, zs, 1.0)
            m.flat("metal_grey", strip, zs - 0.08, 1.0, up=False)
            for p in polys(strip):
                m.ring_walls("paint_charcoal", p.exterior, zs - 0.3, zs, outward=True)
                q = list(p.exterior.coords)
                for k in range(0, len(q) - 1):
                    m.box("paint_charcoal", q[k][0], q[k][1], ztop, zs - 0.3, 0.25, 0.25)
            for t in np.linspace(0.1, 0.9, 6):
                pc = p0 + ax * ((Lr - 12.0) * t) + sd * (W * 0.13)
                m.box("paint_charcoal", pc[0], pc[1], ztop, zs - 0.3, 0.25, 0.25)

    # ---- 137A pavilion / 東屋
    def pavilion_137a(self, m, poly):
        sp = self.T["137A"]
        ze = Z0 + sp.get("eave", 2.9)
        zr_ = Z0 + sp.get("ridge", 5.0)
        ring = poly.buffer(-0.4, join_style="mitre").exterior
        for t in np.arange(0.0, ring.length, 2.4):
            q = ring.interpolate(t)
            m.box(BRICK, q.x, q.y, Z0, Z0 + 1.0, 0.45, 0.45)
            m.box(WHITE, q.x, q.y, Z0 + 1.0, ze, 0.4, 0.4)
        m.flat("concrete_path", poly, Z0 + 0.03, 2.0)
        for p in polys(poly.buffer(-0.2, join_style="mitre")):
            m.ring_walls(WHITE, p.exterior, ze - 0.4, ze, outward=True)
        m.flat("concrete", poly.buffer(-0.2, join_style="mitre"), ze - 0.4, 2.0, up=False)
        ov = max(0.0, min(0.6, self.room.distance(poly) - 0.2))
        hip(m, poly, ze, zr_, "prs11_claytile", ov=ov)
        c = poly.centroid
        for dx in (-2.5, 2.5):
            m.box("prs11_blue", c.x + dx, c.y, Z0 + 0.4, Z0 + 0.48, 1.8, 0.45)

    # ---- covered linkways / 屋根付き通路
    def linkway(self, m, line, w, nm):
        """Red clay-tile gable roof on brick-and-white piers along the line, in
        pieces that clear the roads and the blocks. / 赤瓦の屋根付き通路（道路・棟を避ける）。"""
        ov = 0.3
        clear = unary_union([self.room.buffer(w / 2 + ov + 0.3), self.cluster_foot.buffer(0.2)])
        pieces = line.difference(clear)
        for g in getattr(pieces, "geoms", [pieces]):
            if g.geom_type != "LineString" or g.length < 2.5:
                continue
            strip = g.buffer(w / 2, cap_style="flat")
            ze = Z0 + 2.7
            gable(m, strip, ze, ze + 0.9, "prs11_claytile", WHITE, ov=ov)
            m.flat("concrete_path", strip, Z0 + 0.03, 2.0)
            for s in (-1, 1):
                side = g.parallel_offset(w / 2 - 0.25, "left" if s > 0 else "right")
                if side.is_empty or side.geom_type != "LineString":
                    continue
                n = max(2, int(side.length // 3.2) + 1)
                for t in np.linspace(0.15, side.length - 0.15, n):
                    q = side.interpolate(t)
                    m.box(BRICK, q.x, q.y, Z0, Z0 + 0.9, 0.36, 0.36)
                    m.box(WHITE, q.x, q.y, Z0 + 0.9, ze, 0.3, 0.3)
                m.flat(WHITE, side.buffer(0.18, cap_style="flat"), ze - 0.01, 1.0)

    def gateway(self, m):
        """The east gateway: white rendered portal piers with brick bases either
        side of the estate road; the portal over the road is left out (roads win).
        東口の門: 道路の両脇の白い門柱（道路上の門は作らない）。"""
        gc = Point(self.gate_c)
        dr = self.drive_snapshot.union(self.road)
        edge = dr.boundary
        for ln, w, nm in self.linkways:
            if not nm.startswith("gateway"):
                continue
            # the pier at the road end of each gateway linkway / 道路側の門柱
            q = ln.interpolate(ln.project(gc))
            v = np.array(ln.coords[0]) - np.array(ln.coords[-1])
            v /= np.linalg.norm(v)
            for t in np.arange(0.0, 6.0, 0.25):
                p = Point(q.x + v[0] * t, q.y + v[1] * t)
                if self.room.buffer(1.0).distance(p) > 0.0:
                    for s in (-1, 1):
                        nx, ny = -v[1] * s * (w / 2 - 0.4), v[0] * s * (w / 2 - 0.4)
                        m.box(BRICK, p.x + nx, p.y + ny, Z0, Z0 + 1.2, 0.9, 0.9)
                        m.box(WHITE, p.x + nx, p.y + ny, Z0 + 1.2, Z0 + 4.4, 0.8, 0.8)
                    m.box(WHITE, p.x, p.y, Z0 + 3.6, Z0 + 4.4, 0.8, w - 0.2, math.atan2(v[1], v[0]))
                    break
        # the precinct marker post: white with red fins / 地区の標柱
        x, y = self.marker
        if self.room.distance(Point(x, y)) > 1.0:
            m.cylinder(WHITE, x, y, Z0, Z0 + 4.2, 0.35, seg=12)
            for k in range(7):
                z = Z0 + 1.6 + k * 0.35
                m.box("paint_red", x + 0.25, y, z, z + 0.12, 0.5, 0.06)

    # ---- grounds / 外構
    def grounds(self, m):
        # playground: play set, benches, the stainless railing round it / 遊具・ベンチ・手すり
        if self.playground is not None:
            c = self.playground.centroid
            m.box("paint_red", c.x - 1.5, c.y, Z0, Z0 + 1.4, 2.0, 2.0)
            m.box("paint_orange", c.x - 1.5, c.y, Z0 + 1.4, Z0 + 1.55, 2.4, 2.4)
            for dx, dy in ((-2.4, -0.9), (-2.4, 0.9), (-0.6, -0.9), (-0.6, 0.9)):
                m.cylinder("paint_yellow", c.x + dx, c.y + dy, Z0, Z0 + 2.8, 0.06, seg=6, top=False)
            hip(m, box(c.x - 2.7, c.y - 1.2, c.x - 0.3, c.y + 1.2), Z0 + 2.8, Z0 + 3.5, "paint_red", ov=0.1)
            beam(m, "paint_orange", (c.x - 0.5, c.y, Z0 + 1.45), (c.x + 2.0, c.y, Z0 + 0.3), 0.3)
            m.box("paint_green", c.x + 2.5, c.y + 1.8, Z0, Z0 + 0.6, 1.2, 0.5)
            rail = self.playground.buffer(2.5).exterior
            rail = LineString(rail.coords).difference(self.room.buffer(0.3)).difference(self.cluster_foot.buffer(0.5))
            self.railing(m, rail, "metal_grey", 1.0)
            for k in range(3):
                a = math.radians(30 + k * 25)
                bx = c.x + math.cos(a) * 7.0
                by = c.y + math.sin(a) * 4.5
                if not self.room.buffer(0.5).contains(Point(bx, by)):
                    m.box("prs11_blue", bx, by, Z0 + 0.42, Z0 + 0.5, 1.6, 0.45, a + math.pi / 2)
                    m.box("metal_grey", bx, by, Z0, Z0 + 0.42, 1.4, 0.3, a + math.pi / 2)
        # the fitness frame by the court / フィットネス器具
        x, y = self.fit_c
        a = math.radians(90 - self.fit_b)
        ca, sa = math.cos(a), math.sin(a)
        if self.room.distance(Point(x, y)) > 4.0:
            for k in range(4):
                u = -3.0 + k * 2.0
                beam(m, WHITE, (x + ca * u, y + sa * u, Z0), (x + ca * u, y + sa * u, Z0 + 2.4), 0.06)
            beam(m, WHITE, (x - ca * 3.0, y - sa * 3.0, Z0 + 2.4), (x + ca * 3.0, y + sa * 3.0, Z0 + 2.4), 0.05,
                 caps=True)
            beam(m, WHITE, (x - ca * 3.0, y - sa * 3.0, Z0 + 1.6), (x + ca * 1.0, y + sa * 1.0, Z0 + 1.6), 0.04,
                 caps=True)
        # roundabout planter: low wall, shrubs / ロータリーの植え込み
        rc = Point(self.round_c)
        r = min(4.0, self.no_go.distance(rc) - 0.5)
        if r > 1.2:
            disc = rc.buffer(r, 16)
            m.prism("concrete", disc, Z0, Z0 + 0.45, top=False)
            m.flat("soil", disc.buffer(-0.15), Z0 + 0.4, 2.0)
            m.prism("leaves_dark", rc.buffer(r * 0.55, 10), Z0 + 0.4, Z0 + 1.1, top=True)
        # bulk bins outside the refuse rooms (blue, green) / 大型ごみ容器
        for k, b in self.blocks.items():
            for spec in b.get("ends", []):
                if spec.get("shutter") == "blue":
                    v = bearing_vec(spec["bearing"])
                    p = b["poly"]
                    cen = np.array(p.centroid.coords[0])
                    far = max((np.array(q) for q in p.exterior.coords), key=lambda q: float((q - cen) @ v))
                    for j, mat in enumerate(("prs11_blue", "paint_green")):
                        q = far + v * 2.2 + np.array([-v[1], v[0]]) * (j * 1.6 - 0.8)
                        if not self.room.buffer(0.8).contains(Point(*q)) and not p.buffer(0.3).contains(Point(*q)):
                            m.box(mat, q[0], q[1], Z0, Z0 + 1.3, 1.3, 1.1, math.atan2(v[1], v[0]))

    def railing(self, m, line, mat, h=1.0, post=2.0):
        """A steel railing: posts and two rails, seen from both sides. / 手すり（両面）。"""
        for g in getattr(line, "geoms", [line]):
            if g.geom_type != "LineString" or g.length < 1.0:
                continue
            c = list(g.coords)
            for a, b in zip(c, c[1:]):
                for zz in (Z0 + h - 0.03, Z0 + h * 0.5):
                    # a closed square rail (faces back to back in one plane render black) / 角パイプの手すり
                    beam(m, mat, (a[0], a[1], zz), (b[0], b[1], zz), 0.025)
            for t in np.arange(0.0, g.length + 0.01, post):
                q = g.interpolate(t)
                m.box(mat, q.x, q.y, Z0, Z0 + h, 0.05, 0.05)

    def boundary_wall(self, m):
        """The white perforated wall with red-tile coping on the west edge. / 西側の白い透かし塀。"""
        g = self.west_wall.difference(self.room.buffer(0.3))
        for p in getattr(g, "geoms", [g]):
            if p.geom_type != "LineString":
                continue
            c = list(p.coords)
            # a 0.2 m thick wall: two faces back to back in one plane render black / 厚み 0.2 m の塀
            m.prism("prs11_perf", p.buffer(0.1, cap_style="flat"), Z0, Z0 + 1.8, top=False, uv_scale=1.8)
            m.prism("prs11_claytile", p.buffer(0.18, cap_style="flat"), Z0 + 1.8, Z0 + 1.95, top=True)
            for t in np.arange(0.0, p.length, 3.6):
                q = p.interpolate(t)
                m.box(WHITE, q.x, q.y, Z0, Z0 + 2.0, 0.35, 0.35)

    # ---- frontage blocks / 沿道の棟
    SCHEMES = {   # wall, accent, window-band texture, band / 壁・差し色・窓帯
        "bluewhite": (WHITE, "paint_royalblue", "prs11_fw_white", "paint_paleblue"),
        "orangewhite": (WHITE, "paint_orange", "prs11_fw_white", WHITE),
        "redbrick": ("paint_brickred", WHITE, "prs11_fw_brick", WHITE),
        "tealwhite": (WHITE, "paint_teal", "prs11_fw_white", WHITE),
    }

    def frontage_block(self, m, poly, f_, no):
        if f_["scheme"].startswith("mscp"):
            return self.frontage_mscp(m, poly, f_)
        wall, acc, band_tex, band = self.SCHEMES[f_["scheme"]]
        lv = f_["levels"]
        V, F = self.VOID, self.FLOOR
        zr = Z0 + V + (lv - 1) * F
        edges = B.edges_of(poly)
        sl = unary_union(street_lines(self.L))
        c = poly.centroid
        q = sl.interpolate(sl.project(c))
        to_st = np.array([q.x - c.x, q.y - c.y])
        to_st /= max(np.linalg.norm(to_st), 1e-6)
        longest = max(e[2] for e in edges)
        m.flat("void_floor", poly, Z0 + 0.01, 1.0)
        m.flat("concrete", poly, Z0 + V - 0.01, 2.0, up=False)
        for cc in polys(poly.buffer(-2.4, join_style="mitre")):
            m.ring_walls(CREAM, cc.exterior, Z0, Z0 + V, outward=True, uv_scale=2.0)
        for a, b, L, n in edges:
            f = B.Facade(m, a, b, n)
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 5.0) + 1)):
                f.block(wall if wall != "paint_brickred" else WHITE, u - 0.3, u + 0.3, -0.6, 0.0, Z0, Z0 + V,
                        top=False, bottom=False)
            f.block(acc, 0.0, L, -0.3, 0.05, Z0 + V - 0.45, Z0 + V, ends=False)
            end = L < 8.0 or (L < 0.4 * longest and L < 14.0)
            corr = not end and float(n @ to_st) < -0.5 and L > 8.0
            for k in range(1, lv):
                z0 = Z0 + V + (k - 1) * F
                z1 = z0 + F
                if end:
                    f.face(wall, 0.0, L, z0, z1, 0.0, 1.2 if wall == "paint_brickred" else 2.0)
                    if L > 3.0 and f_["scheme"] in ("bluewhite", "orangewhite", "tealwhite"):
                        texquad(f, acc, L / 2 - min(1.6, L * 0.25), L / 2 + min(1.6, L * 0.25), z0 + 0.3, z1 - 0.2,
                                0.06)
                elif corr:
                    self.corridor_floor(f, L, z0, z1, wall, False, D=1.5)
                else:
                    f.m.quad(band_tex, f.P(0, 0, z0), f.P(L, 0, z0), f.P(L, 0, z1), f.P(0, 0, z1),
                             (0, 0), (L / 3.2, 0), (L / 3.2, 1), (0, 1))
                    for i, (u0, u1) in enumerate(B.bays(L, 3.2, 0.0)):
                        if i % 2 == 1:
                            f.block("prs11_ac", u0 + 0.25, u0 + 1.0, 0.0, 0.5, z0 + 0.15, z0 + 0.7)
                        elif f_["scheme"] == "orangewhite":
                            f.block(acc, u1 - 0.18, u1, 0.0, 0.45, z0, z1, top=False, bottom=False)
                f.block(band if not end else acc if f_["scheme"] == "redbrick" else band, 0.0, L, 0.0, 0.25,
                        z0 - 0.05, z0 + 0.15, ends=False)
            if end and L > 5.0 and lv >= 6:
                bld = B.Builder.__new__(B.Builder)
                bld.numbers(m, f, no, L / 2, zr - 2.4, "paint_navy" if wall == WHITE else WHITE, size=1.4)
        bld = B.Builder.__new__(B.Builder)
        bld.flat_roof(m, poly, zr, wall if wall != "paint_brickred" else WHITE, 1.0, "roof_concrete")
        rc = poly.representative_point()
        ang = math.atan2(edges[0][1][1] - edges[0][0][1], edges[0][1][0] - edges[0][0][0])
        m.box(wall if wall != "paint_brickred" else WHITE, rc.x, rc.y, zr, zr + 3.4, 6.0, 4.0, ang)
        m.box("tank", rc.x, rc.y, zr + 3.4, zr + 4.8, 3.6, 2.6, ang)
        self.solar_rows(m, poly.buffer(-1.5, join_style="mitre").difference(rc.buffer(5.0)), zr,
                        90 - math.degrees(ang))

    def frontage_mscp(self, m, poly, f_):
        acc = "paint_peach" if f_["scheme"] == "mscp_salmon" else "paint_mustard"
        acc2 = "paint_orange" if f_["scheme"] == "mscp_salmon" else "paint_yellow"
        H = 3.0
        nd = f_["levels"]
        ztop = Z0 + nd * H
        m.flat("asphalt_lot", poly, Z0 + 0.01, 6.0)
        for k in range(1, nd + 1):
            z = Z0 + k * H
            m.flat("concrete", poly, z - 0.3, 2.0, up=False)
            m.flat("asphalt_lot" if k < nd else "roof_concrete", poly, z, 6.0)
        for cc in polys(poly.buffer(-1.0, join_style="mitre")):
            m.ring_walls("corridor_dark", cc.exterior, Z0, ztop, outward=True, uv_scale=3.0)
        for a, b, L, n in B.edges_of(poly):
            f = B.Facade(m, a, b, n)
            for k in range(1, nd + 1):
                z = Z0 + k * H
                f.face(WHITE, 0.0, L, z - 0.3, z, 0.0)
                f.block(acc if k % 2 else acc2, 0.0, L, -0.12, 0.0, z, z + 0.45, top=False, bottom=False, ends=False)
                f.block("railing", 0.0, L, -0.1, -0.05, z + 0.95, z + 1.0, ends=False)
            for u in np.linspace(0.3, L - 0.3, max(2, int(L // 8.0) + 1)):
                f.block(WHITE, u - 0.3, u + 0.3, -0.5, 0.05, Z0, ztop + 1.0, top=True, bottom=False)

    # ---- street furniture / 道路付属物
    def shelter(self, m, c, ang, style, ref):
        """A bus shelter set back from the kerb (roof clear of the road): LTA grey
        steel, the blue-striped barrel vault, or the orange-striped roof; bench,
        back panel, the stop's pole and chevron bollards. / バス停の上屋。"""
        x, y = c
        ca, sa = math.cos(ang), math.sin(ang)        # (ca, sa): away from the road / 道路から離れる向き
        tx, ty = -sa, ca
        Dw = 2.6
        # step back until the roof clears the road by ROOM, shorter if a block is in the way
        # 屋根が道路から離れるまで下げる（建物に当たれば短くする）
        found = False
        for Ls in ((24.0, 18.0, 12.0) if style == "vault_blue" else (9.0, 7.0)):
            for back in np.arange(0.0, 3.01, 0.25):
                cx, cy = x + ca * back, y + sa * back
                roof = orect((cx, cy), (Ls, Dw), 90 - math.degrees(math.atan2(ty, tx)))
                if not roof.intersects(self.room.buffer(0.2)) and not roof.intersects(self.footprint.buffer(-0.05)):
                    found = True
                    break
            if found:
                break
        if not found:
            print("prs11: shelter %s left out (no room off the road)" % ref)
            return
        P2 = lambda u, w: (cx + tx * u + ca * w, cy + ty * u + sa * w)
        zr = Z0 + 2.8
        if style == "vault_blue":
            vault(m, "prs11_vault", P2(-Ls / 2, 0.0), P2(Ls / 2, 0.0), Dw / 2, zr - 0.3, seg=8,
                  under="prs11_vault")
            for u in np.arange(-Ls / 2 + 0.6, Ls / 2, 3.0):
                for w in (-Dw / 2 + 0.25, Dw / 2 - 0.25):
                    q = P2(u, w)
                    m.cylinder("paint_orange", q[0], q[1], Z0, zr - 0.3, 0.16, seg=8, top=False)
        else:
            mat = "prs11_stripe_or" if style == "stripe_orange" else "metal_grey"
            m.box(mat, cx, cy, zr, zr + 0.15, Ls, Dw, math.atan2(ty, tx), top=True, bottom=True)
            for u in (-Ls / 2 + 0.4, 0.0, Ls / 2 - 0.4):
                q = P2(u, Dw / 2 - 0.3)
                m.box("metal_grey", q[0], q[1], Z0, zr, 0.12, 0.12)
                if style == "stripe_orange":
                    q = P2(u, -Dw / 2 + 0.3)
                    m.box("metal_grey", q[0], q[1], Z0, zr, 0.08, 0.08)
            q = P2(0.0, Dw / 2 - 0.2)
            m.box("glass_rail", q[0], q[1], Z0 + 0.4, zr - 0.3, Ls - 1.2, 0.05, math.atan2(ty, tx))
        q = P2(0.0, Dw / 2 - 0.65)
        m.box("prs11_blue", q[0], q[1], Z0 + 0.42, Z0 + 0.5, min(Ls - 2.0, 6.0), 0.45, math.atan2(ty, tx))
        m.box("metal_grey", q[0], q[1], Z0, Z0 + 0.42, min(Ls - 2.0, 6.0) - 0.4, 0.3, math.atan2(ty, tx))
        # the stop's pole with its plate at the near end / バス停標柱
        q = P2(-Ls / 2 - 0.8, -Dw / 2 + 0.6)
        if not self.room.buffer(0.3).contains(Point(q)):
            m.cylinder("metal_grey", q[0], q[1], Z0, Z0 + 2.9, 0.05, seg=6, top=True)
            m.box("prs11_stopplate", q[0], q[1], Z0 + 2.2, Z0 + 2.9, 0.6, 0.04, math.atan2(ty, tx))
        # chevron bollards toward the kerb / 縁石側のシェブロン付きボラード
        for u in (-Ls / 2 + 1.0, Ls / 2 - 1.0):
            q = P2(u, -Dw / 2 - 0.2)
            if not self.room.buffer(0.2).contains(Point(q)):
                m.cylinder("prs11_bollard", q[0], q[1], Z0, Z0 + 1.0, 0.11, seg=8, top=True)

    def median_rails(self, m):
        """Green steel railings along the narrow medians at the White Sands crossing
        and on the St 13 bend; chevron boards on the bend. / 中央分離帯の緑の柵、カーブの矢羽根板。"""
        L = self.L
        fr = geo.Frame(self.meta)
        for z in self.T["median_rails"]:
            a, b = _pts(self.T, fr, z)
            zone = LineString([a, b]).buffer(25.0)
            done = []
            for isl in polys(L.islands.intersection(zone)):
                if isl.area < 5.0:
                    continue
                # a narrow median: its outline offset in by half its width runs down the
                # middle (both long sides meet there; keep one) / 狭い分離帯の中央線
                w = 2.0 * isl.area / isl.length
                if w > 4.0:
                    continue
                mid = isl.exterior.offset_curve(-max(w / 2 - 0.05, 0.2))
                for g in getattr(mid, "geoms", [mid]):
                    g = g.intersection(isl.buffer(-0.15))
                    for q in getattr(g, "geoms", [g]):
                        if q.geom_type != "LineString" or q.length < 3.0:
                            continue
                        if any(q.buffer(0.3).contains(q.interpolate(0.5, normalized=True)) and
                               d.distance(q.interpolate(0.5, normalized=True)) < 0.3 for d in done):
                            continue
                        rest = q.difference(unary_union([d.buffer(0.3) for d in done])) if done else q
                        for r in getattr(rest, "geoms", [rest]):
                            if r.geom_type == "LineString" and r.length > 3.0:
                                self.railing(m, r, "prs11_rail_green", 1.0, 2.5)
                                done.append(r)
        cw = L.carriageway
        for c in self.T["chevrons"]:
            p = Point(_pts(self.T, fr, c)[0])
            e = cw.boundary.interpolate(cw.boundary.project(p))
            v = np.array([p.x - e.x, p.y - e.y])
            d = np.linalg.norm(v)
            if d < 1e-6:
                continue
            v /= d
            q = np.array([e.x, e.y]) + v * 1.6
            if self.room.buffer(0.5).contains(Point(*q)):
                continue
            ang = math.atan2(v[1], v[0])
            m.cylinder("metal_grey", q[0], q[1], Z0, Z0 + 2.0, 0.04, seg=6, top=True)
            m.box("metal_grey", q[0] + v[0] * 0.05, q[1] + v[1] * 0.05, Z0 + 1.3, Z0 + 2.0, 0.03, 0.9,
                  ang)
            # the board faces the road (-v) / 板は道路側を向く
            nx, ny = -v[0], -v[1]
            s = np.array([-ny, nx])
            o = q + np.array([nx, ny]) * 0.02
            pa = (*(o - s * 0.45), Z0 + 1.3)
            pb = (*(o + s * 0.45), Z0 + 1.3)
            pc = (*(o + s * 0.45), Z0 + 2.0)
            pd = (*(o - s * 0.45), Z0 + 2.0)
            m.quad("prs11_chevron", pa, pb, pc, pd, (0, 0), (1, 0), (1, 1), (0, 1))


# -- textures / テクスチャ
def write_textures(tex_dir):
    """Drawn textures (procedural, colours from description.md; no imagery):
    the brick-tile courses, render, windows with grey frames, shutters, the end
    wall bars, portholes, red-ringed openings, block plates, corridor grilles,
    car-park bands, clay tiles, herringbone pavers, play rubber, the court, the
    perforated wall, shelter roofs, bollards, chevrons, frontage window bands.
    描画テクスチャ（description.md の色。画像は使わない）。"""
    import textures as X
    from PIL import Image, ImageDraw, ImageFont
    out = {}
    rs = np.random.default_rng(130137)
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

    def brick_courses(n=480, var=1.0):
        """1.2 m square: 12 courses of 0.1 m brick-look tiles, 0.3 m long, a darker
        red course every 4th (sv09). / 0.1 m の煉瓦調タイル 12 段、4 段ごとに濃い赤。"""
        im = Image.new("RGB", (n, n), (172, 134, 110))
        d = ImageDraw.Draw(im)
        ch = n // 12
        tw = n // 4
        for r in range(12):
            dark = r % 3 == 1
            base = np.array((128, 56, 38)) if dark else np.array((176, 96, 56))
            off = tw // 2 if r % 2 else 0
            for k in range(-1, 5):
                v = rs.uniform(-1, 1) * var
                col = tuple(int(c) for c in np.clip(base * (1.0 + 0.07 * v) + (0 if dark else rs.uniform(-6, 6)),
                                                     0, 255))
                d.rectangle([k * tw + off + 2, r * ch + 2, k * tw + off + tw - 2, r * ch + ch - 2], fill=col)
        a = np.asarray(im, np.float64) * (0.94 + 0.08 * X.noise(n, (8, 32)))[..., None]
        return Image.fromarray(a.clip(0, 255).astype(np.uint8))

    save(BRICK, brick_courses(), 0.85, 0.0, 0.8)
    save(WHITE, tinted((222, 220, 212), 256, 0.04), 0.75)
    save(CREAM, tinted((206, 199, 182), 256, 0.05), 0.85)
    save("prs11_ac", tinted((205, 205, 200), 64, 0.03), 0.5, 0.2)
    save("prs11_blue", tinted((30, 80, 160), 64, 0.04), 0.5)
    save("prs11_teal", tinted((59, 93, 97), 64, 0.04), 0.6)
    save("prs11_orange", tinted((176, 124, 80), 64, 0.04), 0.6)
    save("prs11_rail_green", tinted((40, 105, 70), 64, 0.03), 0.4, 0.5)

    def window(panes, n=256):
        """One window: grey aluminium frame and mullions, a transom, dark glass. / 灰色サッシの窓。"""
        y = np.linspace(0, 1, n)[:, None, None]
        g = np.array((70, 85, 95))[None, None, :] * (1 - y) + np.array((120, 135, 145))[None, None, :] * y
        im = Image.fromarray(np.repeat(g, n, axis=1).astype(np.uint8))
        d = ImageDraw.Draw(im)
        fc = (150, 155, 155)
        fw = max(4, n // 28)
        d.rectangle([0, 0, n - 1, n - 1], outline=fc, width=fw)
        for k in range(1, panes):
            x = int(n * k / panes)
            d.rectangle([x - fw // 2, 0, x + fw // 2, n], fill=fc)
        d.rectangle([0, int(n * 0.28) - fw // 2, n, int(n * 0.28) + fw // 2], fill=fc)
        return im

    save("prs11_win", window(1), 0.15, 0.3)
    save("prs11_win_2", window(2), 0.15, 0.3)
    save("prs11_win_3", window(3), 0.15, 0.3)
    for nm, c in (("grey", (147, 146, 133)), ("blue", (12, 77, 148))):
        a = np.asarray(tinted(c, 256, 0.03), np.float64)
        xs = np.arange(256)
        a *= (0.82 + 0.18 * np.abs(np.sin(xs / 256 * math.pi * 24)))[None, :, None]
        save("prs11_shutter_" + nm, Image.fromarray(a.clip(0, 255).astype(np.uint8)), 0.5, 0.5, 1.0)
    dg = tinted((120, 124, 126), 128, 0.03)
    ImageDraw.Draw(dg).rectangle([4, 4, 123, 123], outline=(90, 92, 95), width=4)
    save("prs11_door_grey", dg, 0.5, 0.5)
    # corridor window grille: white bars over dark glass / 廊下側の格子窓
    gr = Image.new("RGB", (256, 128), (60, 70, 78))
    d = ImageDraw.Draw(gr)
    for x in range(0, 256, 16):
        d.rectangle([x, 0, x + 3, 128], fill=(225, 225, 220))
    d.rectangle([0, 0, 255, 127], outline=(225, 225, 220), width=6)
    save("prs11_grille", gr, 0.4, 0.2)
    # end-wall strip: white with two grey bars and grey squares (sv01) / 妻壁の白帯（灰色の縦線と四角）
    bw = Image.new("RGB", (460, 280), (214, 214, 206))
    d = ImageDraw.Draw(bw)
    for x in (150, 290):
        d.rectangle([x, 0, x + 26, 280], fill=(112, 114, 114))
    for x in (40, 360):                              # checker column of grey squares / 灰色の市松
        for j in range(4):
            d.rectangle([x + (j % 2) * 30, j * 70 + 10, x + (j % 2) * 30 + 30, j * 70 + 50], fill=(128, 130, 130))
    d.rectangle([186, 40, 274, 70], fill=(128, 130, 130))
    d.rectangle([186, 180, 274, 210], fill=(128, 130, 130))
    save("prs11_bars", bw, 0.7, 0.0, 0.4)

    def porthole(bg, n=128):
        im = bg.resize((n, n)) if bg is not None else Image.new("RGB", (n, n), (220, 218, 210))
        d = ImageDraw.Draw(im)
        d.ellipse([8, 8, n - 9, n - 9], fill=(230, 228, 222))
        d.ellipse([22, 22, n - 23, n - 23], fill=(60, 72, 82))
        d.line([(n // 2, 22), (n // 2, n - 23)], fill=(200, 200, 196), width=3)
        return im

    save("prs11_port_w", porthole(None), 0.4, 0.1)
    save("prs11_port_b", porthole(brick_courses().crop((0, 0, 320, 320))), 0.6, 0.0)
    rg = Image.new("RGB", (256, 256), (220, 218, 210))
    d = ImageDraw.Draw(rg)
    d.ellipse([10, 10, 245, 245], fill=(180, 62, 50))
    d.ellipse([36, 36, 219, 219], fill=(78, 76, 72))
    save("prs11_ring", rg, 0.6)
    # block plates: green, white digits, yellow strip (sv01, sv12) / 棟番号板
    for no in ("130", "131", "132", "133", "134", "135", "136", "137"):
        im = Image.new("RGB", (256, 160), (77, 102, 71))
        d = ImageDraw.Draw(im)
        d.rectangle([0, 128, 256, 160], fill=(230, 190, 40))
        f = ImageFont.truetype(fb, 92)
        bb = d.textbbox((0, 0), no, font=f)
        d.text((128 - (bb[2] - bb[0]) / 2 - bb[0], 64 - (bb[3] - bb[1]) / 2 - bb[1]), no, fill=(255, 255, 255),
               font=f)
        save("prs11_plate_" + no, im, 0.4)
    save("prs11_claytile", X.roof_tiles(512, (178, 96, 70)), 0.75, 0.0, 0.8)
    # herringbone pavers, red-grey (sv07) / 杉綾の舗装
    n = 512
    pv = Image.new("RGB", (n, n), (120, 118, 118))
    d = ImageDraw.Draw(pv)
    u = n // 16
    for i in range(-2, 18):
        for j in range(-2, 18):
            x, y = i * u * 2 + (j % 2) * u, j * u
            col = (128, 125, 127) if rs.random() < 0.7 else (132, 96, 88)
            col = tuple(int(c * rs.uniform(0.9, 1.08)) for c in col)
            if (i + j) % 2:
                d.rectangle([x + 1, y + 1, x + 2 * u - 2, y + u - 2], fill=col)
            else:
                d.rectangle([x + 1, y + 1, x + u - 2, y + 2 * u - 2], fill=col)
    save("prs11_pavers", pv, 0.85, 0.0, 0.8)
    a = X.noise(512, (4, 8, 16), (0.6, 0.3, 0.1))
    rub = np.where((a > 0.52)[..., None], np.array((175, 62, 50)), np.array((60, 130, 72))).astype(np.float64)
    rub *= (0.92 + 0.1 * X.noise(512, (64, 256)))[..., None]
    save("prs11_rubber", Image.fromarray(rub.clip(0, 255).astype(np.uint8)), 0.9)
    # the court: green surround, red court, white lines, blue boundary band / コート
    W, H = 960, 540                   # 32 x 18 m at 30 px/m: 28 x 15 m court, 2 m green round it
    ct = Image.new("RGB", (W, H), (55, 110, 85))
    d = ImageDraw.Draw(ct)
    d.rectangle([45, 30, W - 46, H - 31], fill=(40, 90, 170))
    d.rectangle([60, 45, W - 61, H - 46], fill=(168, 70, 55))
    d.rectangle([60, 45, W - 61, H - 46], outline=(240, 240, 235), width=4)
    d.line([(W // 2, 45), (W // 2, H - 46)], fill=(240, 240, 235), width=4)
    d.ellipse([W // 2 - 54, H // 2 - 54, W // 2 + 54, H // 2 + 54], outline=(240, 240, 235), width=4)
    for x0, s in ((60, 1), (W - 61, -1)):
        d.arc([x0 - 200, H // 2 - 200, x0 + 200, H // 2 + 200], -90 if s > 0 else 90, 90 if s > 0 else 270,
              fill=(240, 240, 235), width=4)
        d.rectangle([min(x0, x0 + s * 175), H // 2 - 73, max(x0, x0 + s * 175), H // 2 + 73],
                    fill=(190, 150, 60), outline=(240, 240, 235), width=4)
    save("prs11_court", ct, 0.7)
    pf = Image.new("RGB", (360, 360), (226, 224, 216))
    d = ImageDraw.Draw(pf)
    for i in range(4):
        for j in range(3):
            d.rectangle([25 + i * 85, 60 + j * 90, 75 + i * 85, 110 + j * 90], fill=(70, 80, 70))
    save("prs11_perf", pf, 0.8)
    vt = Image.new("RGB", (256, 256), (235, 236, 238))
    d = ImageDraw.Draw(vt)
    for k in range(0, 256, 64):
        d.rectangle([0, k + 20, 256, k + 34], fill=(70, 120, 200))
    save("prs11_vault", vt, 0.5, 0.2)
    so = Image.new("RGB", (256, 256), (240, 238, 232))
    d = ImageDraw.Draw(so)
    for k in range(0, 256, 64):
        d.rectangle([k, 0, k + 32, 256], fill=(225, 120, 50))
    save("prs11_stripe_or", so, 0.5, 0.2)
    bo = Image.new("RGB", (128, 128), (235, 235, 230))
    d = ImageDraw.Draw(bo)
    for k in range(4):
        x = k * 32
        d.polygon([(x, 70), (x + 16, 50), (x + 32, 70), (x + 32, 84), (x + 16, 64), (x, 84)], fill=(235, 120, 30))
    d.rectangle([0, 100, 128, 128], fill=(40, 40, 40))
    save("prs11_bollard", bo, 0.5)
    cv = Image.new("RGB", (256, 192), (240, 240, 236))
    d = ImageDraw.Draw(cv)
    for x in (30, 110, 190):
        d.polygon([(x, 20), (x + 40, 96), (x, 172), (x + 30, 172), (x + 70, 96), (x + 30, 20)], fill=(25, 25, 25))
    d.rectangle([0, 0, 255, 191], outline=(25, 25, 25), width=6)
    save("prs11_chevron", cv, 0.5)
    sp = Image.new("RGB", (128, 160), (240, 240, 236))
    d = ImageDraw.Draw(sp)
    d.rectangle([0, 0, 128, 40], fill=(40, 110, 60))
    d.rectangle([20, 60, 108, 140], outline=(40, 40, 40), width=4)
    d.text((28, 8), "BUS", fill=(255, 255, 255), font=ImageFont.truetype(fb, 22))
    save("prs11_stopplate", sp, 0.5)

    fa = Image.new("RGB", (1024, 64), (235, 235, 230))
    d = ImageDraw.Draw(fa)
    d.rectangle([0, 0, 1024, 10], fill=(200, 40, 40))
    for x in (40, 560):
        d.text((x, 16), "MINI MART  ·  PROVISION", fill=(30, 60, 140), font=ImageFont.truetype(fb, 34))
    save("prs11_fascia", fa, 0.5)

    def fw(wall_im):
        """A frontage floor, 3.2 m wide: the wall with two windows (frames and
        sills drawn). / 沿道の棟の窓帯（3.2 m × 1 層）。"""
        im = wall_im.resize((320, 280))
        d = ImageDraw.Draw(im)
        for x0, x1 in ((30, 150), (190, 290)):
            d.rectangle([x0 - 4, 96, x1 + 4, 226], fill=(205, 205, 200))
            d.rectangle([x0, 100, x1, 220], fill=(75, 92, 102))
            d.line([((x0 + x1) // 2, 100), ((x0 + x1) // 2, 220)], fill=(205, 205, 200), width=4)
            d.line([(x0, 130), (x1, 130)], fill=(205, 205, 200), width=4)
            d.rectangle([x0 - 8, 222, x1 + 8, 232], fill=(190, 190, 186))
        return im

    save("prs11_fw_white", fw(tinted((228, 226, 220), 256, 0.03)), 0.6, 0.1)
    save("prs11_fw_brick", fw(X.bricks(512, (165, 72, 52))), 0.75, 0.0)
    return out
