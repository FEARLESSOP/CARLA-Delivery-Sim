"""The ground plan: every surface of the map as a 2-D shape (maps/.venv).

All shapes come from the road logic's own lanes (lanes.json) and the OSM
data, and they never overlap, so what is drawn is exactly what the car
drives on (the collision of an imported CARLA map is its visible mesh):

  carriageway  the lanes, plus a 0.4 m gutter beyond the outermost lane
               (the kerb is never right at a lane's edge: a wheel on the
               lane edge does not touch it), gaps under 2.4 m between two
               carriageways paved over, junction corners rounded
  kerb         0.2 m wide, 0.15 m high, round the carriageway
  islands      wider medians and traffic islands: kerbed, grass
  pavements    the OSM footways (sidewalks, paths, park connectors, steps)
               and a default 1.8 m footpath behind a grass verge along
               roads OSM has no footway beside; none on the expressway
  areas        car parks, pitches, pools, playgrounds, parks, construction
  buildings    the OSM footprints, cut back so that no part of a building
               (balconies and ledges included) comes within 2.2 m of the
               carriageway, and never overlapping each other
  ground       everything else (grass), up to the map's edge

地図の全ての面を 2D 形状として作る。道路面は道路情報の車線そのもので、
面同士は重ならない（見た目＝当たり判定）。
"""

import json
import math

import numpy as np
import shapely
from shapely import set_precision
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import substring, unary_union
from shapely.validation import make_valid

import geo
from osmdata import OSM

GUTTER = 0.4             # carriageway beyond the outermost lane edge / 最外車線の外の路肩
CLOSE_GAP = 1.2          # gaps under 2x this between carriageways are paved / これ未満の隙間は舗装
CORNER_R = 4.0           # junction corner radius / 交差点の隅切り半径
KERB_W, KERB_H = 0.2, 0.15
BUILD_CLEAR = 2.2        # nothing of a building this close to the carriageway / 建物と車道の最小距離
FACADE_DEPTH = 1.0       # balconies and ledges stand out this far / バルコニー等の張り出し
VERGE, FOOTPATH = 1.0, 1.8
FOOT_W = {"footway": 1.8, "path": 1.5, "cycleway": 2.2, "steps": 1.6, "pedestrian": 4.0, "track": 2.5,
          "bridleway": 1.5}
LEVEL_H, GROUND_H = 2.8, 3.6
MEDIAN_MIN_W = 0.6       # narrower gaps between carriageways are paved / これより狭い隙間は舗装
MEDIAN_JUNCTION_CLEAR = 3.0
BAY_DEPTH, BAY_LEN, BAY_TAPER = 3.0, 15.0, 10.0     # bus bay / バス停車帯
HEAD_TAPER = 10.0        # dead-end turning head: taper length along the street / 袋小路の転回部のすり付け長
# road_fixes maps: a dead end's turning loop (roadnet.py -> turnloops.py: a
# forward bulb, centre radius 3.5-5 m); a car's front corner sweeps past the
# lane's outer edge and TM cars stray more (guide ch. 21)
# 袋小路の転回ループ（電球形）: 車体の角は車線外縁からはみ出す
DEAD_END_MARGIN = 2.0    # paved past a dead end's turning loop (turnloops.PASSES assumes it) / 袋小路の転回ループの外の舗装幅
HEAD_CLEAR = GUTTER + 2 * CLOSE_GAP + 2 * KERB_W + 0.5   # head to other roads' lanes (no gap paving) / 他道路との離れ
WALLED_MARGIN, HEAD_CLEAR_MIN = 1.2, 2.0   # meta "dead_end_walled" heads: paving kept round the loop / 擁壁側の余裕幅


def polys(g):
    if g is None or g.is_empty:
        return []
    if isinstance(g, Polygon):
        return [g]
    return [q for q in getattr(g, "geoms", []) if isinstance(q, Polygon) and not q.is_empty]


def clean(g, min_area=0.5):
    g = make_valid(g)
    return unary_union([p for p in polys(g) if p.area >= min_area])


def lane_turn(lane):
    """Heading change along a lane, degrees (0 straight, 180 a U-turn). / 車線の方向変化（度）。"""
    p = [q["c"] for q in lane["pts"]]
    if len(p) < 3:
        return 0.0
    a = math.atan2(p[1][1] - p[0][1], p[1][0] - p[0][0])
    b = math.atan2(p[-1][1] - p[-2][1], p[-1][0] - p[-2][0])
    return abs((math.degrees(b - a) + 180.0) % 360.0 - 180.0)


def lane_poly(lane):
    left = [p["l"] for p in lane["pts"]]
    right = [p["r"] for p in lane["pts"]]
    p = Polygon(left + right[::-1])
    if not p.is_valid:
        p = make_valid(p)
    return p.buffer(0.01)


class Layout(object):
    def __init__(self, meta, lanes_json, osm_path):
        self.meta = meta
        import os
        fp = os.path.join(os.path.dirname(os.path.abspath(lanes_json)), "road_features.json")
        self.features = json.load(open(fp)) if os.path.exists(fp) else {}
        self.bounds = box(-meta["half_x"], -meta["half_y"], meta["half_x"], meta["half_y"])
        data = json.load(open(lanes_json))
        self.lanes = [l for l in data["lanes"] if l["kind"] == "driving"]
        self.approaches = data["approaches"]
        self.lights = data["lights"]
        self.osm = OSM(osm_path, meta)
        self.build_roads()
        self.build_paths()
        self.build_areas()
        self.build_buildings()
        self.build_ground()
        self.landmark_footpaths()

    # -- roads / 道路
    def build_roads(self):
        lp = [lane_poly(l) for l in self.lanes]
        if self.meta.get("road_fixes"):
            # the lanes as drawn, for the road heights (terrain.Drape): a head's
            # paving beyond its lanes takes the nearest lane's height, not an
            # interpolation towards a road beside it / 標高用の元の車線形状
            self.lane_area_drawn = unary_union(lp)
            lp = self.dead_end_heads(lp)
        jp = [p for p, l in zip(lp, self.lanes) if l["junction"] >= 0]
        self.junction_lanes = unary_union(jp)
        dead = getattr(self, "dead_end_junctions", set())
        # a dead end's turning-loop lanes as drawn (props keeps lamps off) / 袋小路の転回ループ車線
        self.dead_end_lanes = unary_union([lane_poly(l) for l in self.lanes if l["junction"] in dead])
        self.lane_area = unary_union(lp)
        # islands the lanes already enclose (slip-road triangles, medians): kept
        # whatever the gap-closing and corner rounding below would pave
        # 車線が囲む島（左折専用路の三角島・分離帯）は舗装で潰さない
        base = self.lane_area.buffer(GUTTER, quad_segs=4)
        keep_islands = unary_union([Polygon(r) for p in polys(base) for r in p.interiors
                                    if Polygon(r).area > 8.0 and not Polygon(r).buffer(-0.5).is_empty])
        c = self.lane_area.buffer(GUTTER, quad_segs=4)
        c = c.buffer(CLOSE_GAP, quad_segs=4).buffer(-CLOSE_GAP, quad_segs=4)
        jz = unary_union(jp).buffer(10.0)
        corners = c.buffer(CORNER_R, quad_segs=4).buffer(-CORNER_R, quad_segs=4).intersection(jz)
        c = unary_union([c, corners])
        # the strip between a dead-end head and a road at another height stays
        # unpaved (the corner rounding bridged it: one sheet with a fold)
        # 転回部と高さの違う道路の間は舗装しない（隅切りが繋いでしまう）
        if getattr(self, "head_keepout", None) is not None and not self.head_keepout.is_empty:
            c = c.difference(self.head_keepout)
        # never pave over the lanes' own outline less than the gutter / 車線＋路肩は必ず含む
        c = unary_union([c, self.lane_area.buffer(GUTTER, quad_segs=4)]).intersection(self.bounds)
        if not keep_islands.is_empty:
            c = c.difference(keep_islands)
        out = []
        islands = []
        for p in polys(c):
            keep = []
            for ring in p.interiors:
                hole = Polygon(ring)
                if hole.area < 6.0 or hole.buffer(-0.6).is_empty:
                    continue                          # slivers: pave them / 細い隙間は舗装
                keep.append(ring)
                if hole.area < 1500.0 or hole.buffer(-6.0).is_empty:
                    islands.append(hole)              # median / traffic island / 中央分離帯・交通島
            out.append(Polygon(p.exterior, keep))
        self.carriageway = clean(unary_union(out), 1.0)
        self.medians = Polygon()
        self.bus_bays = []
        if self.meta.get("medians"):
            # separate carriageways (OSM dual carriageways): the gap between them,
            # outside junctions, is a kerbed median, not paved over
            # 分離道路の車道間の隙間（交差点外）は舗装せず縁石付きの中央分離帯にする
            open_ = self.lane_area.buffer(GUTTER, quad_segs=4)
            gap = open_.buffer(CLOSE_GAP, quad_segs=4).buffer(-CLOSE_GAP, quad_segs=4).difference(open_)
            gap = gap.difference(unary_union(jp).buffer(MEDIAN_JUNCTION_CLEAR))
            med = [q for q in polys(gap) if q.area > 15.0 and q.length ** 2 / q.area > 40.0
                   and not q.buffer(-MEDIAN_MIN_W / 2).is_empty]
            if med:
                self.medians = unary_union(med).buffer(0.05).buffer(-0.05)
                self.carriageway = clean(self.carriageway.difference(self.medians), 1.0)
                islands += polys(self.medians)
        if self.meta.get("bus_bays"):
            self.add_bus_bays(jp)
        # lanes are sampled every metre: drop the points a straight run does
        # not need (within 2 cm) / 直線部の不要な点を除く（2 cm 以内）
        self.carriageway = clean(set_precision(self.carriageway.simplify(0.02), 0.001), 1.0)
        self.kerb = clean(self.carriageway.buffer(KERB_W, quad_segs=3, join_style="mitre", mitre_limit=2.0)
                          .difference(self.carriageway).intersection(self.bounds), 0.01)
        self.road_all = unary_union([self.carriageway, self.kerb])
        self.islands = clean(unary_union(islands).difference(self.road_all), 0.5)

    def dead_end_heads(self, lp):
        """Dead ends (road_fixes maps): each dead end's turning loop (a forward
        bulb, turnloops.py; roadnet.py drops the turnarounds at every other
        junction) becomes one smooth round turning head: the convex hull of
        the loop lane buffered DEAD_END_MARGIN m and the last HEAD_TAPER m of
        the street's lanes. / 袋小路の転回ループ＋余裕幅と手前 HEAD_TAPER m の
        凸包（丸い転回部）。"""
        byj = {}
        for i, l in enumerate(self.lanes):
            if l["junction"] >= 0:
                byj.setdefault(l["junction"], []).append(i)

        turn = lane_turn

        out = list(lp)
        # landmark sites (maps.json "landmarks") keep the plain head: their blocks
        # were cut back by the bigger heads (campus, dorm) / 詳細モデル敷地内は従来の転回部
        sites = self.landmark_sites(lambda lm: True)
        lm_zone = sites.buffer(BUILD_CLEAR + FACADE_DEPTH + 0.5) if sites is not None else None
        self.heads_in_landmarks = 0
        tree = shapely.STRtree(lp)
        keepout = []
        self.heads_clipped = 0
        walled = set(self.meta.get("dead_end_walled") or [])
        self.head_rail_sides = []
        self.dead_ends = 0
        self.dead_end_junctions = set()
        self.dead_end_heads_by_j = {}
        for j, idx in byj.items():
            if not all(turn(self.lanes[i]) > 100.0 for i in idx):
                continue                     # an ordinary junction / 通常の交差点
            self.dead_end_junctions.add(j)
            head = unary_union([lp[i] for i in idx])
            near = [k for k, l in enumerate(self.lanes) if l["junction"] < 0 and lp[k].intersects(head.buffer(0.3))]
            ends = []
            for k in near:
                # the street's last HEAD_TAPER m, measured along the lane (a hull
                # over a bend paved across its inside corner) / 車線に沿って測る
                pts = self.lanes[k]["pts"]
                at_start = Point(pts[0]["c"]).distance(head) < Point(pts[-1]["c"]).distance(head)
                s_end = pts[0]["s"] if at_start else pts[-1]["s"]
                q = [p_ for p_ in pts if abs(p_["s"] - s_end) <= HEAD_TAPER]
                if len(q) >= 2:
                    ends.append(lane_poly({"pts": q}))
            body = unary_union([head] + ends)
            # convex and round: the U-turn lanes plus DEAD_END_MARGIN m (a car's
            # body sweeps past them), tapering into the street; never more than
            # that past the lanes (a hull over a bend) / 転回車線＋余裕幅の凸包（丸い転回部）
            hull = unary_union([head.buffer(DEAD_END_MARGIN, quad_segs=8)] + ends).convex_hull \
                .intersection(body.buffer(DEAD_END_MARGIN + 1.0, quad_segs=8))
            # the bigger head keeps clear of roads at other heights (two heads
            # merged into one sheet with a fold); never smaller than the plain
            # hull / 高さの違う道路とは離す（転回部が繋がり段ができた）
            mine = set(idx) | set(near)
            zh = float(np.mean([q["z"] for i in idx for q in self.lanes[i]["pts"]]))
            zone = hull.buffer(HEAD_CLEAR)

            def apart(k):
                # a road beside the head at another height (a road at the same
                # height may join it: the gap is paved level) / 高さの違う道路のみ
                zs = [q["z"] for q in self.lanes[k]["pts"] if zone.contains(Point(q["c"][:2]))]
                return bool(zs) and max(abs(z - zh) for z in zs) > 0.3
            other = [lp[k] for k in tree.query(zone) if k not in mine
                     and self.lanes[k]["junction"] != j and apart(k)]
            if other:
                ou = unary_union(other)
                # full HEAD_CLEAR: paving nearer ended in a drop with no kerb (fix3
                # review); turnloops.py --refine moves loops clear of such roads
                # 高さの違う道路から HEAD_CLEAR 離す（近いと段差・折れ）
                full = hull
                hull = hull.difference(ou.buffer(HEAD_CLEAR, quad_segs=4))
                # a loop still that near (turnloops --refine could not move it, meta
                # "dead_end_walled", or its head's taper) keeps WALLED_MARGIN m of
                # paving round its lane, down to HEAD_CLEAR_MIN m off the other road,
                # with a guardrail behind the kerb there (props.py)
                # それでも近いループ: 車線の外 WALLED_MARGIN m を残しガードレール
                keep = head.buffer(WALLED_MARGIN, quad_segs=8).difference(ou.buffer(HEAD_CLEAR_MIN, quad_segs=4))
                lost = full.intersection(keep).difference(hull)
                if j in walled or lost.area > 0.5:
                    hull = unary_union([hull, full.intersection(keep)])
                hull = hull.buffer(-1.2, quad_segs=6).buffer(1.2, quad_segs=6)    # no sharp cut corners / 角を丸める
                cut_clear = HEAD_CLEAR_MIN if (j in walled or lost.area > 0.5) else HEAD_CLEAR
                plain = body.convex_hull.intersection(body.buffer(3.0, quad_segs=4)).difference(ou.buffer(cut_clear, quad_segs=4))
                hull = unary_union([plain] + [q for q in polys(hull) if q.intersects(head)])
                self.heads_clipped += 1
                if j in walled or lost.area > 0.5:
                    self.head_rail_sides.append((j, ou))      # guardrail on that side / ガードレール
                gap = ou.buffer(HEAD_CLEAR - 0.6, quad_segs=4).intersection(hull.buffer(HEAD_CLEAR, quad_segs=4)) \
                    .difference(ou.buffer(GUTTER + 0.05, quad_segs=4)).difference(hull.buffer(GUTTER + 0.05, quad_segs=4))
                keepout.append(gap)
            if lm_zone is not None and hull.intersects(lm_zone):
                plain = body.convex_hull.intersection(body.buffer(3.0, quad_segs=4))
                rest = hull.difference(lm_zone).buffer(-1.2, quad_segs=6).buffer(1.2, quad_segs=6)
                hull = unary_union([plain] + [q for q in polys(rest) if q.intersects(head)])
                self.heads_in_landmarks += 1
            for i in idx:
                out[i] = hull
            self.dead_end_heads_by_j[j] = (hull, zh)
            self.dead_ends += 1
        self.head_keepout = unary_union(keepout) if keepout else None
        # the finished heads (props.py keeps lamps off their kerbs) / 完成した転回部
        # guardrail lines: the finished head's kerb back, where it faces the road
        # at another height (CCW: the head on the left) / ガードレール線
        self.head_rails = []
        for j, ou in self.head_rail_sides:
            hp = out[byj[j][0]]
            ring = Polygon(hp.exterior).buffer(GUTTER + KERB_W + 0.5, quad_segs=8).exterior if hp.geom_type == "Polygon" else None
            if ring is None:
                continue
            if not Polygon(ring).exterior.is_ccw:
                ring = LineString(list(ring.coords)[::-1])
            side = LineString(ring.coords).intersection(ou.buffer(HEAD_CLEAR + 1.5, quad_segs=4))
            for g in getattr(side, "geoms", [side]):
                if g.geom_type == "LineString" and g.length > 3.0:
                    self.head_rails.append((j, g))
        self.dead_end_head_area = unary_union([out[byj[j][0]] for j in self.dead_end_junctions]) \
            if self.dead_end_junctions else Polygon()
        return out

    def add_bus_bays(self, jp):
        """Bus bays at the OSM bus stops beside roads with traffic (>= 40 km/h):
        the kerb indented BAY_DEPTH m over BAY_LEN m, with BAY_TAPER m tapers,
        on the kerb side of the nearest lane outside junctions. Model only
        (the road logic has no lane there). / バス停の停車帯（縁石を後退させる）。"""
        left = self.meta.get("lefthand", False)
        kerbs = []
        for l in self.lanes:
            if l["junction"] >= 0 or (l["speed"] or 0) < 11.0:
                continue
            kerbs.append(l)
        # the kerb-side lane of each road section: keep-left lane -1 (its left
        # edge is the kerb); keep-right the last lane (its right edge)
        # 縁石側の車線: 左側通行は -1 車線の左端、右側通行は最後の車線の右端
        side = {}
        for l in kerbs:
            k = (l["road"], l["section"])
            if k not in side or (l["lane"] > side[k]["lane"] if left else l["lane"] < side[k]["lane"]):
                side[k] = l
        edges = []
        for l in side.values():
            e = [q["l"] if left else q["r"] for q in l["pts"]]
            c = [q["c"] for q in l["pts"]]
            if len(e) >= 2:
                edges.append((LineString(e), LineString(c), l))
        if not edges:
            return
        junc = unary_union(jp).buffer(5.0)
        total_len = BAY_LEN + 2 * BAY_TAPER
        bays = []
        # road_features.json in the build dir (optional): what Street View showed,
        # e.g. {"bus_bays": {"skip": ["<node id>", ...]}} for kerbside stops
        # 現地確認による例外（停車帯の無いバス停など）
        skip = set(str(v) for v in self.features.get("bus_bays", {}).get("skip", []))
        for p, t, nid in self.osm.points(lambda t: t.get("highway") == "bus_stop"):
            if not self.bounds.buffer(-20).contains(p) or str(nid) in skip:
                continue
            d, e, cl, l = min(((g.distance(p), g, c, l) for g, c, l in edges), key=lambda v: v[0])
            sc = cl.project(p)
            a, b = cl.interpolate(max(0.0, sc - 1.0)), cl.interpolate(min(cl.length, sc + 1.0))
            c0 = cl.interpolate(sc)
            cross = (b.x - a.x) * (p.y - c0.y) - (b.y - a.y) * (p.x - c0.x)     # > 0: left of travel / 左側
            if d > 10.0 or (cross > 0) != left or cl.distance(p) < 1.0:
                continue                 # on the road itself, or on the far side / 道路上・反対側
            # centred on the stop, else slid (up to 15 m) clear of junctions / 交差点を避けてずらす
            sp = e.project(p) - total_len / 2
            starts = [sp + o for o in (0, 3, -3, 6, -6, 9, -9, 12, -12, 15, -15)]
            starts = [v for v in starts if 1.0 <= v <= e.length - total_len - 1.0
                      and not e.interpolate(v).buffer(0.1).union(e.interpolate(v + total_len).buffer(0.1))
                      .intersects(junc)
                      and not shapely.intersects(junc, LineString([e.interpolate(v + u) for u in
                                                                   np.linspace(0, total_len, 8)]))]
            if not starts:
                continue                 # no room between junctions / 交差点間に余地なし
            s0 = starts[0]
            pts_in, pts_out = [], []
            for u in np.linspace(0.0, total_len, int(total_len) + 1):
                q = e.interpolate(s0 + u)
                q2 = e.interpolate(min(e.length, s0 + u + 0.5))
                q1 = e.interpolate(max(0.0, s0 + u - 0.5))
                tx, ty = q2.x - q1.x, q2.y - q1.y
                n = math.hypot(tx, ty) or 1.0
                nx, ny = -ty / n, tx / n                  # left of travel / 進行方向の左
                c = cl.interpolate(cl.project(q))
                if (q.x - c.x) * nx + (q.y - c.y) * ny < 0:
                    nx, ny = -nx, -ny                     # away from the lane / 車線と反対側へ
                dep = BAY_DEPTH * min(1.0, u / BAY_TAPER, (total_len - u) / BAY_TAPER)
                pts_in.append((q.x - nx * 0.1, q.y - ny * 0.1))
                pts_out.append((q.x + nx * (GUTTER + dep), q.y + ny * (GUTTER + dep)))
            bay = Polygon(pts_in + pts_out[::-1]).buffer(0)
            if bay.intersects(junc) or bay.is_empty:
                continue
            bays.append(bay)
            self.bus_bays.append({"stop": nid, "name": t.get("name", ""), "road": l["road"],
                                  "x": round(p.x, 1), "y": round(p.y, 1),
                                  # for the bay's markings: its outline and the lane edge
                                  # along its mouth (travel direction) / 標示用: 形状と入口の車線端
                                  "poly": bay, "edge": substring(e, s0, s0 + total_len)})
        if bays:
            self.carriageway = clean(unary_union([self.carriageway] + bays).intersection(self.bounds), 1.0)

    # -- footways / 歩道
    def build_paths(self):
        osm = self.osm
        parts, bridges, crossings = [], [], []
        covered = []
        for ln, t, wid in osm.lines("highway"):
            hw = t.get("highway")
            if hw not in FOOT_W or t.get("area") == "yes":
                continue
            if t.get("bridge") or t.get("layer") not in (None, "0"):
                bridges.append((ln, t))
                continue
            if t.get("tunnel") or t.get("indoor") == "yes":
                continue
            w = FOOT_W[hw] if t.get("footway") != "sidewalk" else 2.0
            g = ln.buffer(w / 2, cap_style="flat", quad_segs=3)
            if t.get("footway") == "crossing" or ln.intersects(self.carriageway):
                crossings.append((ln, t))
            parts.append((g, hw, t))
            if t.get("covered") == "yes":
                covered.append((ln, w))
        for p, t in osm.areas(lambda t: t.get("highway") == "pedestrian" or t.get("area:highway") in
                              ("footway", "pedestrian")):
            parts.append((p, "pedestrian", t))
        keepout = self.road_all
        self.footpaths = clean(unary_union([g for g, hw, t in parts if hw != "cycleway"]).difference(keepout), 0.3)
        self.cycleways = clean(unary_union([g for g, hw, t in parts if hw == "cycleway"])
                               .difference(keepout).difference(self.footpaths), 0.3)
        self.footbridges = bridges
        self.crossing_ways = crossings
        self.covered_ways = covered
        # default footpath where OSM maps none beside a road / OSM に歩道が無い道路の既定歩道
        motorway = unary_union([lane_poly(l) for l in self.lanes if (l["speed"] or 0) >= 24.0])
        band = self.carriageway.buffer(KERB_W + VERGE + FOOTPATH, quad_segs=3).difference(
            self.carriageway.buffer(KERB_W + VERGE, quad_segs=3))
        mapped = unary_union([self.footpaths, self.cycleways]).buffer(6.0)
        if not band.is_valid:
            band = make_valid(band)       # GEOS buffers can come out invalid / buffer が不正形状のとき
        band = band.difference(mapped).difference(motorway.buffer(12.0))
        if getattr(self, "dead_end_lanes", None) is not None and not self.dead_end_lanes.is_empty:
            band = band.difference(self.dead_end_lanes.buffer(DEAD_END_MARGIN + KERB_W + VERGE + FOOTPATH + 1.0))
        self.footpaths = clean(unary_union([self.footpaths, clean(band, 6.0)]).difference(keepout), 0.3)
        self.default_band = clean(band, 6.0)

    # -- areas / 土地利用
    def build_areas(self):
        osm = self.osm
        def area(test):
            return clean(unary_union([p for p, t in osm.areas(test)]).intersection(self.bounds), 1.0)
        self.parking = area(lambda t: t.get("amenity") == "parking" and t.get("parking") not in
                            ("multi-storey", "underground") and "building" not in t)
        # surface car parks along the OSM parking aisles: the aisle and a row of
        # bays on each side (Singapore estates map the aisles, not the lots)
        # 駐車通路の両側に駐車枠の列（シンガポールの団地は通路のみ登録）
        aisles = []
        for ln, t, wid in osm.lines("highway"):
            if t.get("service") == "parking_aisle" and ln.length > 6.0:
                aisles.append(ln)
        self.parking_aisles = aisles
        self.parking = clean(unary_union([self.parking] + [ln.buffer(8.0, cap_style="flat") for ln in aisles])
                             .intersection(self.bounds), 4.0)
        self.pools = area(lambda t: t.get("leisure") == "swimming_pool")
        self.pitches = area(lambda t: t.get("leisure") in ("pitch", "track"))
        self.playgrounds = area(lambda t: t.get("leisure") in ("playground", "fitness_station"))
        self.parks = area(lambda t: t.get("leisure") in ("park", "garden", "park_connector")
                          or t.get("landuse") in ("grass", "meadow", "recreation_ground", "village_green")
                          or t.get("natural") in ("grassland", "scrub"))
        self.forest = area(lambda t: t.get("landuse") == "forest" or t.get("natural") == "wood")
        self.construction = area(lambda t: t.get("landuse") == "construction")
        self.water = area(lambda t: t.get("natural") == "water" or t.get("waterway") == "riverbank")
        self.shelters = [p for p, t in osm.areas(lambda t: t.get("amenity") == "shelter" and "building" not in t)]
        # driveways into car parks and estates (not in the road logic) / 駐車場等への進入路
        dw = []
        for ln, t, wid in osm.lines("highway"):
            if t.get("highway") == "service" and t.get("area") != "yes":
                half = 2.6 if t.get("oneway") == "yes" else 3.0
                if "width" in t:                  # a mapped width (e.g. from maps.json osm_fixes) / 幅タグ
                    try:
                        half = float(t["width"].split()[0]) / 2
                    except ValueError:
                        pass
                dw.append(ln.buffer(half, cap_style="flat"))
        self.driveways = clean(unary_union(dw).difference(self.road_all.buffer(0.05)), 2.0)
        taken = unary_union([self.road_all, self.islands])
        self.driveways = clean(self.driveways.difference(taken), 2.0)
        # beyond the ring road there is only green (trees along the edge, a
        # noise barrier where an expressway runs) / 外周道路の外は緑地のみ
        from shapely import wkt
        inside = wkt.loads(self.meta["ring_wkt"]).buffer(self.meta["ring_w"] / 2 + KERB_W + VERGE + FOOTPATH + 0.5)
        self.inside = inside
        paved_in = inside
        if self.meta.get("ring_guardrail"):
            # a guardrail lines the ring's outer kerb (props.py): nothing paved
            # behind it (OSM stubs leaving the map and the default footpath band
            # lay as grey patches on the grass outside the rail)
            # 外周にガードレールがある地図: その外側には何も舗装しない
            probe = wkt.loads(self.meta["ring_wkt"]).exterior.interpolate(0.0)
            piece = next((p for p in polys(self.carriageway) if p.buffer(0.5).contains(probe)), None)
            if piece is not None:
                paved_in = Polygon(piece.exterior).buffer(KERB_W, quad_segs=4)
        self.paved_inside = paved_in
        for name in ("driveways", "parking", "footpaths", "cycleways", "pools", "pitches", "playgrounds",
                     "construction", "water"):
            setattr(self, name, clean(getattr(self, name).intersection(paved_in), 0.3))
        self.parks = clean(self.parks.intersection(inside), 1.0)
        # no area may reach onto a road or its kerb / どの面も道路・縁石に掛からない
        for name in ("pools", "pitches", "playgrounds", "construction", "water", "parks", "forest"):
            setattr(self, name, clean(getattr(self, name).difference(taken.buffer(0.5)), 0.5))
        self.parking = clean(self.parking.difference(taken).difference(self.driveways), 2.0)
        self.footpaths = clean(self.footpaths.difference(self.driveways).difference(self.parking), 0.3)
        self.cycleways = clean(self.cycleways.difference(self.driveways).difference(self.parking), 0.3)

    # -- buildings / 建物
    def build_buildings(self):
        osm = self.osm
        keep_off = self.carriageway.buffer(BUILD_CLEAR + FACADE_DEPTH, quad_segs=3)
        skip_ways, skip_areas = self.landmark_exclusion()
        self.landmark_areas = skip_areas
        raw = []
        self.excluded = []
        import osmfix
        added = {str(a["id"]) for a in (osmfix.fixes_for(self.meta) or {}).get("add_ways", [])}
        last = set()
        for p, t, wid in osm.areas_id(lambda t: "building" in t and t.get("building") not in ("no", "roof")
                                      and t.get("location") != "underground"):
            if not p.intersects(self.bounds):
                continue
            # landmarks (maps.json "landmarks"): left to their own module, clear
            # ground here / 詳細モデルの建物は汎用処理から外す（地面のまま残す）
            if wid in skip_ways or (skip_areas is not None and skip_areas.intersects(p.representative_point())):
                self.excluded.append(wid)
                continue
            if skip_ways or skip_areas is not None:
                t = dict(t, **{"@way": wid})
            raw.append((p, t))
            if wid in added:
                last.add(id(t))
        # largest first; buildings added by maps.json osm_fixes go last, so the
        # buildings of OSM keep their order (buildings_jp seeds by it) / 追加分は最後（既存の順序・乱数を保つ）
        raw.sort(key=lambda pt: (id(pt[1]) in last, -pt[0].area))
        taken = Polygon()
        self.buildings, self.cut_buildings, self.dropped = [], 0, 0
        ring_keep = self.ring_inner()
        for p, t in raw:
            p = make_valid(p)
            q = p.difference(keep_off).difference(taken.buffer(0.5)).intersection(ring_keep)
            q = clean(q, 8.0)
            parts = polys(q)
            if not parts or q.area < 0.35 * p.area:
                self.dropped += 1
                continue
            if q.area < p.area - 0.5:
                self.cut_buildings += 1
            q = max(parts, key=lambda g: g.area)       # one solid per building / 1 棟 1 形状
            q = q.simplify(0.15)
            self.buildings.append((q, t))
            taken = unary_union([taken, q])
        self.building_area = taken

    def landmark_exclusion(self):
        """maps.json "landmarks" of this map: [{"name", "ways": [building way ids],
        "areas": [way ids of areas whose buildings are all left out], "buffer": m}].
        Returns (set of way ids, union of the areas or None). Building Builder
        fills the list. / 詳細モデルに任せる建物（way id と敷地）。"""
        import os
        try:
            cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json")))
        except (OSError, ValueError):
            return set(), None
        ent = next((v for v in cfg.values() if isinstance(v, dict) and v.get("name") == self.meta.get("name")), {})
        ways, areas = set(), []
        for lm in ent.get("landmarks") or []:
            ways |= {str(w) for w in lm.get("ways", [])}
            for a in lm.get("areas", []):
                refs = self.osm.ways.get(str(a), (None,))[0]
                p = self.osm.poly(refs) if refs else None
                if p is not None:
                    areas.append(p.buffer(float(lm.get("buffer", 0.0))))
        return ways, (unary_union(areas) if areas else None)

    def landmark_footpaths(self):
        """Landmark sites whose maps.json entry says "footpaths": "osm" keep only
        their mapped footways (a campus's internal lanes have no pavements). Done
        last, piece by piece: every footpath polygon that does not reach into
        the site stays the very same object, so nothing outside changes.
        詳細モデル敷地（"footpaths": "osm"）内だけ既定の歩道を除く（最後に、敷地外は不変）。"""
        nb = self.landmark_sites(lambda lm: lm.get("footpaths") == "osm")
        if nb is None:
            return
        self.footpaths_before = self.footpaths
        keep = polys(self.footpaths)
        # one landmark entry at a time: a site's cut never touches the pieces of
        # another (with one entry this is exactly the single cut it always was)
        # 詳細モデルごとに切る（他の敷地の歩道片には触れない）
        for site in self.landmark_sites(lambda lm: lm.get("footpaths") == "osm", each=True):
            cut = self.default_band.intersection(site.buffer(-0.5))
            out = []
            for g in keep:
                # the cut leaves zero-width slivers along the band's edges (1-3 m
                # from the kerb): dropped / 帯の縁に残る幅ゼロの細片は捨てる
                # (hairline slivers along the site edge too: buffered by a landmark's
                # clearance they notched buildings) / 敷地縁の細片も除く
                out += [q for q in polys(g.difference(cut)) if q.area >= 0.3 and q.buffer(-0.2).area > 0] \
                    if g.intersects(cut) else [g]
            keep = out
        self.footpaths = MultiPolygon(keep)

    def landmark_sites(self, test, each=False):
        """Union of the areas of this map's maps.json "landmarks" entries that
        pass ``test`` (None if none). An entry may also give "site_lonlat":
        [[lon, lat], ...], the traced site where OSM maps only part of it (the
        7-Eleven: OSM has the shop outline, the car park is unmapped). each=True:
        a list with one union per entry instead.
        条件に合う詳細モデル敷地の和（"site_lonlat": OSM に無い敷地のトレース）。"""
        import os
        try:
            cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json")))
        except (OSError, ValueError):
            return None
        ent = next((v for v in cfg.values() if isinstance(v, dict) and v.get("name") == self.meta.get("name")), {})
        areas, per = [], []
        for lm in ent.get("landmarks") or []:
            if not test(lm):
                continue
            mine = []
            for a in lm.get("areas", []):
                refs = self.osm.ways.get(str(a), (None,))[0]
                p = self.osm.poly(refs) if refs else None
                if p is not None:
                    mine.append(p)
            if lm.get("site_lonlat"):
                fr = geo.Frame(self.meta)
                mine.append(make_valid(Polygon([fr.xy(lo, la) for lo, la in lm["site_lonlat"]])))
            areas += mine
            if mine:
                per.append(unary_union(mine))
        if each:
            return per
        return unary_union(areas) if areas else None

    def ring_inner(self):
        """Inside of the ring road's inner pavement (buildings stay inside it). / 外周道路の内側。"""
        from shapely import wkt
        m = self.meta
        return wkt.loads(m["ring_wkt"]).buffer(-(m["ring_w"] / 2 + KERB_W + VERGE + FOOTPATH + 1.0),
                                               join_style="mitre")

    # -- ground / 地面
    def build_ground(self):
        for name in ("footpaths", "cycleways", "driveways", "parking", "islands", "parks", "forest"):
            setattr(self, name, clean(getattr(self, name).simplify(0.03), 0.3))
        used = unary_union([self.road_all, self.islands, self.footpaths, self.cycleways, self.parking,
                            self.driveways, self.pools, self.pitches, self.playgrounds,
                            self.construction, self.water])
        # aprons: a paved strip round every building / 建物周りの舗装
        bld = self.building_area
        for name in ("parking", "driveways", "footpaths", "cycleways", "pitches", "playgrounds", "pools"):
            setattr(self, name, clean(getattr(self, name).difference(bld.buffer(0.3)), 0.3))
        used = unary_union([self.road_all, self.islands, self.footpaths, self.cycleways, self.parking,
                            self.driveways, self.pools, self.pitches, self.playgrounds,
                            self.construction, self.water])
        self.aprons = clean(bld.buffer(1.5, join_style="mitre").difference(bld).difference(used), 0.5)
        used = unary_union([used, self.aprons, bld])
        self.parks = clean(self.parks.difference(used), 1.0)
        self.forest = clean(self.forest.difference(used), 1.0)
        self.grass = clean(self.bounds.difference(unary_union([used, self.parks, self.forest])), 0.05)

    def summary(self):
        a = lambda g: g.area
        return ("carriageway %.0f m2, kerb %.0f m2 (%.1f km), islands %d, footpaths %.0f m2, cycleways %.0f m2, "
                "driveways %.0f m2, car parks %.0f m2, buildings %d (%d cut back from the road, %d left out)"
                % (a(self.carriageway), a(self.kerb), self.kerb.area / KERB_W / 1000, len(polys(self.islands)),
                   a(self.footpaths), a(self.cycleways), a(self.driveways), a(self.parking),
                   len(self.buildings), self.cut_buildings, self.dropped)
                + ("; medians %.0f m2 (%d), bus bays %d" % (self.medians.area, len(polys(self.medians)),
                                                          len(self.bus_bays))
                   if self.meta.get("medians") or self.meta.get("bus_bays") else ""))


NET_SKIP_SERVICE = {"parking_aisle", "drive-through", "emergency_access", "driveway"}   # as roadnet.py


def drive_remnants(L, width=1.0):
    """Thin pieces of L.driveways along a kerb that only come from the drive band of a
    service way the road logic already drives (roadnet.py keeps it): the OSM line's
    3 m band sticks out past the netconvert carriageway on bends and beside one-way
    or offset roads (0.5-0.6 m strips, OitaKosen 300 m2). Pieces under ``width`` m
    mean width (2 x area / perimeter) touching the kerb; parts other drives (car-park
    entrances) cover are kept. Used on hilly maps by ground.py (they were paved as
    lots and dropped the kerb beside them).
    走行網に入るサービス道路の帯が縁石の外にはみ出した細片（坂の地図で芝に戻す）。"""
    net, other = [], []
    for ln, t, wid in L.osm.lines("highway"):
        if t.get("highway") != "service" or t.get("area") == "yes":
            continue
        half = 2.6 if t.get("oneway") == "yes" else 3.0
        if "width" in t:
            try:
                half = float(t["width"].split()[0]) / 2
            except ValueError:
                pass
        b = ln.buffer(half, cap_style="flat")
        drives = t.get("service") not in NET_SKIP_SERVICE and t.get("access") not in ("no", "private") \
            and t.get("tunnel") != "building_passage"
        (net if drives else other).append(b)
    if not net:
        return Polygon()
    g = unary_union(net).difference(unary_union(other)).intersection(L.driveways)
    keep = [p for p in polys(g) if p.area > 1e-4 and 2 * p.area / p.length < width and p.distance(L.kerb) < 0.1]
    return unary_union(keep) if keep else Polygon()


def preview(L, path, scale=1.0):
    from PIL import Image, ImageDraw
    hx, hy = L.meta["half_x"], L.meta["half_y"]
    W, H = int(2 * hx * scale), int(2 * hy * scale)
    im = Image.new("RGB", (W, H), (90, 140, 70))
    d = ImageDraw.Draw(im)
    P = lambda c: [((x + hx) * scale, (hy - y) * scale) for x, y in c]

    def fill(g, col, outline=None):
        # a mask per shape so holes stay open / 穴を開けたまま塗るためマスクを使う
        for p in polys(g):
            xs, ys = p.bounds[0::2], p.bounds[1::2]
            mk = Image.new("L", (W, H), 0)
            md = ImageDraw.Draw(mk)
            md.polygon(P(p.exterior.coords), fill=255)
            for r in p.interiors:
                md.polygon(P(r.coords), fill=0)
            im.paste(col, (0, 0, W, H), mk)
            if outline:
                d.line(P(p.exterior.coords), fill=outline)
    fill(L.carriageway, (55, 55, 58)); fill(L.kerb, (230, 230, 230))
    fill(L.parks, (110, 165, 80)); fill(L.forest, (50, 100, 50)); fill(L.pitches, (80, 170, 90))
    fill(L.pools, (60, 150, 210)); fill(L.playgrounds, (200, 120, 80)); fill(L.construction, (160, 140, 110))
    fill(L.aprons, (190, 185, 175)); fill(L.parking, (70, 70, 72)); fill(L.driveways, (75, 75, 78))
    fill(L.footpaths, (205, 200, 190)); fill(L.cycleways, (180, 90, 70)); fill(L.islands, (100, 150, 80))
    for p, t in L.buildings:
        fill(p, (225, 215, 195), (120, 110, 100))
    im.save(path)
