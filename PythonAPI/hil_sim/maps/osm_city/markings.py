"""Road markings (Singapore practice) from the road logic's lanes (maps/.venv).

Every marking follows the lane edges of the road logic, so it is exactly
where CARLA's lanes are. Lines stop where a junction begins.

  lane lines       white broken (3 m on, 6 m off); solid over the last 30 m
                   before a signal stop line (no lane changing)
  centre line      between the two directions of a two-way road: double
                   white on main roads, broken white on estate roads
  kerb side        yellow along the kerb: double on main roads (no parking
                   at any time), single on estate roads; white edge lines on
                   the expressway
  median side      white edge line where the other direction is beyond an
                   island
  stop lines       0.3 m white across the approach at signals, behind the
                   pedestrian crossing if there is one
  give way         two broken transverse lines and the inverted triangle
                   where a minor road meets a bigger one without signals
  lane arrows      on approaches with two or more lanes, or at signals, 7 m
                   and 27 m before the stop line, showing only the turns the
                   lane's connections really make
  crossings        zebra (bars along the road, kerb to kerb) with zig-zag
                   lines on the approaches, or a signalised crossing (two
                   parallel studded lines), square to the road
  words            SLOW before zebra crossings

Each marking is (shapely polygon, colour "w"/"y").

道路標示（シンガポール式）。道路情報の車線端に沿って配置する。
"""

import math

import numpy as np
from shapely.affinity import rotate, scale, translate
from shapely.geometry import LineString, MultiLineString, Point, Polygon, box
from shapely.ops import substring, unary_union

LINE_W = 0.12
STOP_W = 0.30
DASH, GAP = 3.0, 6.0
SOLID_BEFORE = 30.0


def polys(g):
    if g is None or g.is_empty:
        return []
    if g.geom_type == "Polygon":
        return [g]
    return [q for q in getattr(g, "geoms", []) if q.geom_type == "Polygon" and not q.is_empty]


def strip(line, w):
    if line is None or line.is_empty or line.length < 0.05:
        return None
    return line.buffer(w / 2, cap_style="flat", join_style="mitre", mitre_limit=3.0)


def dashes(line, dash=DASH, gap=GAP, phase=0.0):
    out, s, L = [], phase, line.length
    while s < L:
        out.append(substring(line, s, min(L, s + dash)))
        s += dash + gap
    return [q for q in out if q.length > 0.2]


def offset(line, d):
    """Parallel line, d > 0 to the left. / 左へ d の平行線。"""
    try:
        o = line.offset_curve(d, quad_segs=4, join_style="mitre", mitre_limit=3.0)
    except Exception:
        return None
    if o.is_empty:
        return None
    if o.geom_type == "MultiLineString":
        o = max(o.geoms, key=lambda g: g.length)
    return o


class Markings(object):
    def __init__(self, layout):
        self.L = layout
        self.out = []                 # (polygon, colour) / (形状, 色)
        self.giveway = []             # (road, junction) with give-way lines / 譲れ標示のある進入路
        self.lanes = [l for l in layout.lanes if l["junction"] < 0]
        self.main = {}
        self.build_index()
        self.crossings = self.find_crossings()
        self.lane_lines()
        self.junction_marks()
        self.crossing_marks()
        self.clip()

    def add(self, g, col="w"):
        for p in polys(g):
            if p.area > 0.002:
                self.out.append((p, col))

    # -- lanes grouped per road section / 道路区間ごとの車線
    def build_index(self):
        self.sections = {}
        for l in self.lanes:
            self.sections.setdefault((l["road"], l["section"]), []).append(l)
        self.signal_roads = {lt["road"] for lt in self.L.lights}
        # lanes whose left edge (the road's centre side) has the other
        # direction right beside it / 左端の隣に対向車線がある車線
        from shapely.strtree import STRtree
        # centre-side edges of every section's last lane (keep-left) or first (keep-right)
        # 各区間の対向側の車線端
        lefthand = self.L.meta.get("lefthand", False)
        self.edges = []
        for lanes in self.sections.values():
            lanes = sorted(lanes, key=lambda l: abs(l["lane"]))
            l = lanes[-1] if lefthand else lanes[0]
            self.edges.append((LineString([p["r" if lefthand else "l"] for p in l["pts"]]), l))
        self.tree = STRtree([e for e, _ in self.edges])

    def is_main(self, l):
        """A main road: 50 km/h or more with two or more lanes each way (or 60+).
        幹線: 50 km/h 以上で片側 2 車線以上（または 60 km/h 以上）。"""
        sp = l["speed"] or 13.9
        n = len(self.sections.get((l["road"], l["section"]), [l]))
        return sp >= 16.0 or (sp >= 13.8 and n >= 2)

    def opposite(self, edge, l):
        """The innermost lane of the other direction along this lane's left edge. / 対向車線。"""
        best = None
        for i in self.tree.query(edge.buffer(0.4)):
            e2, l2 = self.edges[i]
            if l2 is l or l2["road"] == l["road"]:
                continue
            mid = edge.interpolate(0.5, normalized=True)
            if e2.distance(mid) > 0.4:
                continue
            best = l2
        return best

    # -- lane lines / 車線境界線
    def lane_lines(self):
        """netconvert gives every direction its own road with negative lanes.
        Keep-left: lane -1 runs along the kerb (its left edge) and the last
        lane's right edge faces the other direction. Keep-right: the reverse.
        左側通行: 車線 -1 の左端が縁石、最後の車線の右端が対向側。右側通行は逆。"""
        lefthand = self.L.meta.get("lefthand", False)
        for key, lanes in self.sections.items():
            lanes.sort(key=lambda l: abs(l["lane"]))
            stopline_road = lanes[0]["road"] in self.signal_roads
            n = len(lanes)
            for i, l in enumerate(lanes):
                left = LineString([p["l"] for p in l["pts"]])
                right = LineString([p["r"] for p in l["pts"]])
                if left.length < 0.5:
                    continue
                main = self.is_main(l)
                fast = (l["speed"] or 0) >= 22.0
                kerb_lane = i == 0 if lefthand else i == n - 1
                centre_lane = i == n - 1 if lefthand else i == 0
                kerb_edge, kerb_out = (left, 1.0) if lefthand else (right, -1.0)
                centre_edge, centre_in = (right, 1.0) if lefthand else (left, -1.0)
                # between lanes of the same direction / 同方向車線の間
                if i + 1 < n:
                    self.lane_divider(right, l, stopline_road)
                if kerb_lane:
                    # in the gutter between the lane edge and the kerb / 車線端と縁石の間
                    if fast:
                        self.add(strip(offset(kerb_edge, kerb_out * 0.2), 0.15))
                    elif main:
                        self.add(strip(offset(kerb_edge, kerb_out * 0.12), 0.09), "y")
                        self.add(strip(offset(kerb_edge, kerb_out * 0.30), 0.09), "y")
                    else:
                        self.add(strip(offset(kerb_edge, kerb_out * 0.18), 0.10), "y")
                if centre_lane:
                    opp = self.opposite(centre_edge, l)
                    if opp is not None:
                        if main or fast:
                            # each direction paints its half of the double line / 二重線を半分ずつ
                            self.add(strip(offset(centre_edge, centre_in * 0.10), LINE_W))
                        elif l["road"] < opp["road"]:
                            for d in dashes(centre_edge, 3.0, 4.0):
                                self.add(strip(d, LINE_W))
                    elif main or fast:
                        # median side: the other direction is beyond an island / 分離帯側
                        self.add(strip(offset(centre_edge, -centre_in * 0.2), 0.12))

    def lane_divider(self, line, l, stopline_road):
        L = line.length
        solid_from = max(0.0, L - SOLID_BEFORE) if stopline_road else L
        # lanes run with increasing s for negative ids: the junction is at the end
        # 負の車線は s 増加方向に走る: 終点側が交差点
        if l["lane"] > 0:
            line = LineString(line.coords[::-1])
        broken = substring(line, 0, solid_from) if solid_from > 0 else None
        if broken is not None and broken.length > 0.5:
            for d in dashes(broken):
                self.add(strip(d, LINE_W))
        if solid_from < L:
            self.add(strip(substring(line, solid_from, L), LINE_W))

    # -- stop / give way lines, arrows / 停止線・譲れ・矢印
    def junction_marks(self):
        appr = {}
        for a in self.L.approaches:
            appr.setdefault((a["road"], a["junction"]), []).append(a)
        lane_by = {(l["road"], l["lane"]): l for l in self.lanes}
        jroads = {}
        for (road, j), al in appr.items():
            jroads.setdefault(j, []).append((road, al))
        for (road, j), al in appr.items():
            lanes = [lane_by.get((a["road"], a["lane"])) for a in al]
            lanes = [x for x in lanes if x is not None]
            if not lanes:
                continue
            signal = road in self.signal_roads
            yaw = math.radians(al[0]["yaw"])
            f = np.array([math.cos(yaw), math.sin(yaw)])
            n = np.array([-f[1], f[0]])
            back = self.crossing_setback(al, f)
            ends = np.array([a["end"] for a in al])
            ws = [a["width"] for a in al]
            # the line across all approach lanes, set back from the junction / 全車線を横切る線
            c = ends.mean(axis=0) - f * (1.0 + back)
            half = sum(ws) / 2
            across = LineString([c - n * half, c + n * half])
            others = [x for x in jroads.get(j, []) if x[0] != road]
            minor = self.is_minor(lanes[0], [lane_by.get((a2["road"], a2["lane"]))
                                             for _, al2 in others for a2 in al2])
            if signal:
                self.add(strip(across, STOP_W))
            elif minor and others:
                self.giveway.append((road, j))
                for d in dashes(across, 0.6, 0.3):
                    self.add(strip(d, 0.2))
                for d in dashes(offset(across, 0.5) or across, 0.6, 0.3):
                    self.add(strip(d, 0.2))
                for a in al:
                    self.add(self.triangle(np.array(a["end"]) - f * (back + 8.0), yaw))
            if signal or len(al) >= 2:
                for a in al:
                    lane = lane_by.get((a["road"], a["lane"]))
                    L = LineString([p["c"] for p in lane["pts"]]).length if lane else 0
                    for dist in (7.0, 27.0):
                        if dist + 6.0 < L and a["turns"]:
                            p = np.array(a["end"]) - f * (dist + back)
                            self.add(self.arrow(p, yaw, a["turns"]))

    def is_minor(self, lane, others):
        def rank(l):
            return (l["speed"] or 13.9) + (5.0 if self.is_main(l) else 0.0)
        others = [o for o in others if o is not None]
        return bool(others) and rank(lane) + 0.5 < max(rank(o) for o in others)

    def crossing_setback(self, al, f):
        """How far the stop line must sit back so it is behind a crossing at this arm.
        交差点の入口に横断歩道があれば、停止線をその手前に。"""
        end = np.array(al[0]["end"])
        best = 0.0
        for cr in self.crossings:
            v = np.array(cr["c"]) - end
            along = -float(v @ f)                          # >0: before the junction / 交差点の手前
            side = abs(float(v @ np.array([-f[1], f[0]])))
            if -2.0 < along < 15.0 and side < 15.0:
                best = max(best, along + cr["width"] / 2 + 1.0)
        return best

    @staticmethod
    def place(shape, p, yaw):
        """Shape drawn pointing +x at the origin -> at p, pointing yaw. / 形状を配置。"""
        return translate(rotate(shape, yaw, origin=(0, 0), use_radians=True), p[0], p[1])

    def triangle(self, p, yaw):
        # give-way triangle, pointing back at the driver / 運転者側を向く逆三角形
        t = Polygon([(-2.5, -0.75), (-2.5, 0.75), (2.5, 0.0)]).difference(
            Polygon([(-2.1, -0.55), (-2.1, 0.55), (1.6, 0.0)]))
        return self.place(rotate(t, 180, origin=(0, 0)), p, yaw)

    def arrow(self, p, yaw, turns):
        shaft = box(-2.5, -0.09, 1.2, 0.09)
        head = Polygon([(1.2, -0.35), (2.5, 0.0), (1.2, 0.35)])
        parts = []
        if "straight" in turns or not ({"left", "right"} & set(turns)):
            parts += [shaft, head]
        for side in ("left", "right"):
            if side in turns:
                sgn = 1 if side == "left" else -1
                stem = box(-2.5, -0.09, 0.0, 0.09)
                bend = box(-0.09, 0.0, 0.09, 0.9) if sgn > 0 else box(-0.09, -0.9, 0.09, 0.0)
                h = Polygon([(-0.4, 0.9 * sgn), (0.4, 0.9 * sgn), (0.0, 1.8 * sgn)])
                h = rotate(h, 0, origin=(0, 0))
                parts += [stem, translate(bend, -0.6, 0), translate(h, -0.6, 0)]
        return self.place(unary_union(parts), p, yaw)

    # -- pedestrian crossings / 横断歩道
    def find_crossings(self):
        """Crossings from the OSM crossing ways and nodes, square to the road,
        kerb to kerb. / 横断歩道（道路に直交、縁石から縁石まで）。"""
        L = self.L
        cw = L.carriageway
        out = []
        cands = []
        for ln, t in L.crossing_ways:
            seg = ln.intersection(cw)
            if seg.is_empty:
                continue
            p = seg.interpolate(0.5, normalized=True) if seg.geom_type == "LineString" else seg.centroid
            cands.append((p, t))
        for p, t, nid in L.osm.points(lambda t: t.get("highway") == "crossing" or "crossing" in t
                                      and t.get("highway") in ("traffic_signals",)):
            if p.distance(cw) < 0.5 and all(p.distance(q) > 6.0 for q, _ in cands):
                cands.append((p, t))
        lane_lines = [(LineString([q["c"] for q in l["pts"]]), l) for l in self.lanes]
        for p, t in cands:
            kind = (t.get("crossing") or t.get("crossing:markings") or "")
            if kind in ("", "unmarked", "no", "informal") or t.get("crossing:markings") == "no":
                continue                         # no type given: unmarked here / 種別なしは標示なし
            signal = kind == "traffic_signals" or t.get("crossing:signals") == "yes" or \
                t.get("highway") == "traffic_signals"
            zebra = kind in ("zebra", "marked", "uncontrolled") and not signal
            near = min(lane_lines, key=lambda ll: ll[0].distance(p), default=None)
            if near is None or near[0].distance(p) > 12.0:
                continue
            s = near[0].project(p)
            a = near[0].interpolate(max(0, s - 1.0))
            b = near[0].interpolate(min(near[0].length, s + 1.0))
            yaw = math.atan2(b.y - a.y, b.x - a.x)
            f = np.array([math.cos(yaw), math.sin(yaw)])
            n = np.array([-f[1], f[0]])
            c = np.array([p.x, p.y])
            line = LineString([c - n * 30.0, c + n * 30.0])
            segs = [g for g in getattr(line.intersection(cw), "geoms", [line.intersection(cw)])
                    if g.geom_type == "LineString"]
            if zebra:
                # only the kerb-to-kerb piece it is on (a slip lane, say) / 載っている区間のみ
                segs = [min(segs, key=lambda g: g.distance(p))] if segs else []
            else:
                # across a median refuge too, but not over a whole junction / 分離帯越しまで
                segs = [g for g in segs if g.distance(p) < 8.0]
            if not segs or sum(g.length for g in segs) > 32.0:
                continue
            ends = [np.array(q) for g in segs for q in g.coords]
            ts = [float((q - c) @ n) for q in ends]
            if not (zebra or signal):
                continue
            out.append({"c": c.tolist(), "f": f.tolist(), "n": n.tolist(), "t0": min(ts), "t1": max(ts),
                        "segs": segs, "width": 4.0 if zebra else 3.6,
                        "kind": "zebra" if zebra else "signal", "yaw": yaw})
        # one crossing per spot / 同じ場所の重複を除く
        kept = []
        for cr in out:
            if all(math.hypot(cr["c"][0] - k["c"][0], cr["c"][1] - k["c"][1]) > 5.0 for k in kept):
                kept.append(cr)
        return kept

    def crossing_marks(self):
        for cr in self.crossings:
            f, n, c = np.array(cr["f"]), np.array(cr["n"]), np.array(cr["c"])
            w = cr["width"]
            area = []
            for g in cr["segs"]:
                # the crossing band along this kerb-to-kerb piece / 縁石間の帯
                area.append(g.buffer(w / 2, cap_style="flat"))
            band = unary_union(area).intersection(self.L.carriageway)
            if cr["kind"] == "zebra":
                bars = []
                t = cr["t0"] + 0.3
                while t + 0.5 < cr["t1"]:
                    q = c + n * (t + 0.25)
                    bars.append(LineString([q - f * w / 2, q + f * w / 2]).buffer(0.25, cap_style="flat"))
                    t += 1.0
                self.add(unary_union(bars).intersection(band))
                self.zigzags(cr)
                # SLOW in each lane approaching the crossing, 14 m before it,
                # read by that lane's drivers / 各進入車線の 14 m 手前に SLOW
                for lane in self.lanes:
                    ln = LineString([q["c"] for q in lane["pts"]])
                    if ln.distance(band) > 0.5:
                        continue
                    hit = ln.intersection(band)
                    if hit.is_empty:
                        continue
                    s0 = min(ln.project(Point(q)) for g in getattr(hit, "geoms", [hit]) for q in g.coords)
                    if s0 < 16.0:
                        continue
                    a, b = ln.interpolate(s0 - 15.0), ln.interpolate(s0 - 13.0)
                    yaw = math.atan2(b.y - a.y, b.x - a.x)
                    self.add(self.word("SLOW", ((a.x + b.x) / 2, (a.y + b.y) / 2), yaw))
            else:
                for sgn in (-1, 1):
                    edge = LineString([c + n * cr["t0"] + f * sgn * w / 2, c + n * cr["t1"] + f * sgn * w / 2])
                    for d in dashes(edge, 0.25, 0.25):
                        self.add(strip(d, 0.2).intersection(self.L.carriageway))

    def zigzags(self, cr):
        """White zig-zag lines along the lane lines before a zebra. / 横断歩道手前のジグザグ線。"""
        f, n, c = np.array(cr["f"]), np.array(cr["n"]), np.array(cr["c"])
        for sgn in (-1, 1):
            for t in (cr["t0"] + 0.4, cr["t1"] - 0.4):
                pts = []
                for k in range(9):
                    along = cr["width"] / 2 + 1.0 + k * 1.5
                    pts.append(c + f * sgn * along + n * (t + (0.25 if k % 2 else -0.25) * (1 if t < 0 else -1)))
                self.add(LineString(pts).buffer(0.06, join_style="mitre"))

    def word(self, text, p, yaw, band_t=None, n=None, c=None):
        """Road lettering, 1.6 m tall glyphs stretched along the road (vector from a font).
        路面文字（縦長に引き伸ばした字形）。"""
        g = glyphs(text, 2.4, 0.35)
        if g is None:
            return None
        # letters' up along the travel direction, reading left to right for the
        # driver / 文字の上が進行方向、運転者から左→右に読める
        return self.place(rotate(g, -90, origin=(0, 0)), p, yaw)

    def clip(self):
        cw = self.L.carriageway
        # merge each colour into one shape: overlapping paint at one height
        # would z-fight (flicker); white wins over yellow
        # 色ごとに結合（同じ高さの重なりはちらつく）、白を黄より優先
        white = unary_union([g for g, c in self.out if c == "w"]).intersection(cw)
        yellow = unary_union([g for g, c in self.out if c == "y"]).intersection(cw).difference(white.buffer(0.02))
        self.out = [(q.simplify(0.01), "w") for q in polys(white) if q.area > 0.002] + \
                   [(q.simplify(0.01), "y") for q in polys(yellow) if q.area > 0.002]


_GLYPHS = {}


def glyphs(text, height=2.4, aspect=0.35):
    """Text as polygons (rendered with PIL, traced from the bitmap), letters
    ``height`` tall, squeezed across by ``aspect``; centred on the origin, x
    reading direction, y up. Road words are long and narrow (read at a low
    angle). / 文字を多角形に（路面文字は縦長）。"""
    key = (text, height, aspect)
    if key in _GLYPHS:
        return _GLYPHS[key]
    from PIL import Image, ImageDraw, ImageFont
    font = None
    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                 "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"):
        try:
            font = ImageFont.truetype(path, 120)
            break
        except OSError:
            continue
    if font is None:
        _GLYPHS[key] = None
        return None
    bb = font.getbbox(text)
    im = Image.new("L", (bb[2] + 10, bb[3] + 10), 0)
    ImageDraw.Draw(im).text((5, 5), text, fill=255, font=font)
    a = np.asarray(im) > 128
    h, w = a.shape
    cells = []
    for y in range(h):
        row = a[y]
        x = 0
        while x < w:
            if row[x]:
                x0 = x
                while x < w and row[x]:
                    x += 1
                cells.append(box(x0, h - y - 1, x, h - y))
            x += 1
    g = unary_union(cells).simplify(1.0)
    minx, miny, maxx, maxy = g.bounds
    k = height / (maxy - miny)
    g = scale(translate(g, -(minx + maxx) / 2, -(miny + maxy) / 2), k * aspect, k, origin=(0, 0))
    _GLYPHS[key] = g
    return g
