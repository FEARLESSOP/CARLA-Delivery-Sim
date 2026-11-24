"""Road markings, Singapore practice (LTA), opt-in per map (maps.json "sg_lta"; maps/.venv).

Rules from LTA's Standard Details of Road Elements, chapter 8 "Road markings & signs"
(LTA/SDRE14/8 RMS1-RMS14, 2014-2019), checked against Street View in Tampines /
Pasir Ris (2024-25; guide ch. 51). Widths in metres:

  lane lines     B 0.10 white 2 m line / 4 m gap; B1 on expressways 2 m / 10 m;
                 C 4 m / 2 m over the last 8 marks (48 m) before a signal stop line
  centre line    two-way main road H: two 0.10 white lines 0.10 apart; estate road
                 E: 0.15 white 2.75 m / 2.75 m; service roads none
  kerb side      main road I: double yellow 0.10 lines 0.10 apart (no parking at
                 any time); estate road: 0.10 white edge line (Street View: no yellow
                 on Tampines St 45); slip roads / ramps: white edge line, no yellow
                 (RMS1 note 1); expressway J: 0.30 white edge line
  median side    M 0.20 white edge line on 3-lane+ divided roads, J on expressways
  stop line      J 0.30 white at signals, 2 m before the signalised crossing
                 (RMS14) or 1 m before the junction mouth (RMS9 note 3); signs.py then
                 moves each light to its line and stands the pole past it
  give way       D two broken white lines (0.15, 0.6 m / 0.4 m) where a lower class
                 meets a higher one without signals at a real crossing (not a merge)
  arrows         RMS4 shapes (5.5 m), 10, 25 and 40 m before the stop line (RMS9),
                 from the real junction connections, on 2+ lane signal approaches or
                 where the lanes' turns differ; never on expressways
  crossings      only where OSM says so: zebra (white bars 0.5 m / 0.5 m, 4 m band)
                 on main and slip roads with K zig-zag lines (0.10, 4 m legs,
                 +-0.25 m) at the kerb (0.35 m off it, RMS9 note 6) and on the centre
                 line, the kerb lines stopping there (note 5); on estate roads (Street
                 View) no zig-zags, the kerb lines run on, a hatched taper on the
                 centre line; no SLOW; signalised: A4 two rows of 0.2 m studs 0.2 m
                 apart (no bars); untyped crossings are unmarked
  bus lanes      OSM lanes:bus on the kerb lane: L 0.30 yellow line, A1 broken
                 yellow (1 m / 1 m) near junctions, "BUS" in yellow at the start
                 (RMS10; normal bus lanes, all of them here are time-limited)
  bus bays       yellow box (Street View): continuous yellow line along the bay
                 mouth and across both ends
  chevrons       RMS8: white chevrons (0.2 m, 1.8 m pitch) and outline in the paved
                 wedge where two carriageways merge or divide
  yellow box     N: 0.45 m yellow outline and 0.20 m diagonals, at junctions listed
                 in maps.json "sg_lta" "yellow_box" (no OSM tag; from imagery)

Raised (flyover) lanes are left to flyover.py (deck paint); here only the ground
parts. Each colour is unioned before meshing (no z-fighting).

シンガポール (LTA) 式の路面標示（maps.json "sg_lta" の地図のみ）。
"""

import collections
import math

import numpy as np
from shapely.affinity import rotate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import substring, unary_union
from shapely.strtree import STRtree

import geo
from markings import Markings, dashes, glyphs, offset, polys, strip
from markings_jp import MarkingsJP, tangent, xodr_roads

EXPWY, MAIN, LINK, ESTATE, NARROW = 5, 4, 3, 2, 1
CLASS_NAME = {EXPWY: "expressway", MAIN: "main", LINK: "slip/ramp", ESTATE: "estate", NARROW: "service"}
Z0 = 0.02                   # flyover.Z0: a lane point higher than this is on a deck / これより高い点は高架
LINE_W = 0.10               # B / B1 / C lane lines
CENTRE_W = 0.15             # E / F
DOUBLE_W, DOUBLE_GAP = 0.10, 0.10     # H / I
EDGE_W = 0.10               # estate edge line (Street View)
LINK_EDGE_W = 0.15
MEDIAN_W = 0.20             # M
EXPWY_W = 0.30              # J (also the stop line)
STOP_W = 0.30
STOP_BACK, STOP_PLAIN = 2.0, 1.0      # before a signalised crossing / the junction mouth
STOP_NEAR_CROSS = 20.0     # a crossing this close before the mouth: the line goes behind it / これより近い横断歩道の手前に停止線
C_DASH, C_GAP, C_MARKS = 4.0, 2.0, 8
B_DASH, B_GAP = 2.0, 4.0
B1_DASH, B1_GAP = 2.0, 10.0
E_DASH, E_GAP = 2.75, 2.75
ARROW_AT = (10.0, 25.0, 40.0)
ZEBRA_BAR, ZEBRA_GAP = 0.5, 0.5
STUD, STUD_GAP = 0.2, 0.2
ZZ_LEG, ZZ_AMP, ZZ_W, ZZ_LEGS = 4.0, 0.25, 0.10, 3
HT_LEN, HT_W, HT_PITCH = 15.0, 1.2, 1.5      # estate zebra: hatched taper on the centre line / 導流帯
GIVE_W, GIVE_DASH, GIVE_GAP, GIVE_SEP = 0.15, 0.6, 0.4, 0.10
BUS_L_W = 0.30
BOX_EDGE, BOX_DIAG = 0.45, 0.20
JUNCTION_TOTAL_MAX = 0.50   # share of a band allowed over junction lanes (check_markings) / 交差点車線上の許容率
JUNCTION_TURN_MAX = 0.20    # ... over junction lanes that turn across it / 曲がる交差点車線上の許容率
CHEV_W, CHEV_PITCH = 0.20, 1.8


def lane_poly(l):
    return Polygon([q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]).buffer(0)


def ground_runs(lanes, level="ground"):
    """Each lane's parts on one level, as lane dicts: "ground" (points with z <=
    Z0) or "deck" (z > Z0, the flyover decks); at_end marks the run that reaches
    the lane's end (it carries the approach). / 各車線の地上部分または高架部分。"""
    out = []
    for l in lanes:
        z = [q.get("z", 0.0) for q in l["pts"]]
        on = [(zz <= Z0) if level == "ground" else (zz > Z0) for zz in z]
        if all(on):
            out.append(l)
            continue
        runs, cur = [], []
        for q, ok in zip(l["pts"], on):
            if ok:
                cur.append(q)
            elif cur:
                runs.append(cur)
                cur = []
        if cur:
            runs.append(cur)
        runs = [r for r in runs if len(r) >= 2]
        runs.sort(key=lambda r: r[-1] is not l["pts"][-1])
        for r in runs:
            out.append(dict(l, pts=r, raised_part=True, at_end=r[-1] is l["pts"][-1]))
    return out


class MarkingsSG(MarkingsJP):
    """LTA markings. Reuses MarkingsJP's crossing placement (square to the lanes,
    kerb to kerb, behind the junction) and stop-line setback; everything else is
    Singapore's own. / LTA 式の標示（横断歩道の配置と停止線の後退は日本版を流用）。"""

    def __init__(self, layout, xodr, cfg=None, level="ground", cw=None):
        """level "ground": the model's roads (Road_Marking); "deck": the flyover lanes,
        clipped to ``cw`` (the deck footprint), draped by build.py on the deck mesh.
        地上の道路、または高架の車線（橋面に貼る）。"""
        self.L = layout
        self.level = level
        self.cw = cw if cw is not None else layout.carriageway
        self.cfg = cfg if isinstance(cfg, dict) else {}
        self.out, self.giveway, self.stops = [], [], []
        self.paint = collections.defaultdict(list)      # (colour, cut group) -> [polygon]
        self.elements = []
        self.count = collections.Counter()
        self.lefthand = layout.meta.get("lefthand", False)
        allj = [l for l in layout.lanes if l["junction"] < 0]
        self.lanes = ground_runs(allj, level)
        self.count["lanes (parts) on this level (%s)" % level] = len(self.lanes)
        self.build_index()
        self.classify_sg(xodr)
        self.osm_signals = [p for p, t, nid in layout.osm.points(lambda t: t.get("highway") == "traffic_signals")]
        self.osm_stops, self.jcfg, self.jartic, self.jartic_log = [], None, None, []
        self.stop_log, self.dropped_stops, self.no_stop_log = [], [], []
        self.zones = {"kerb": [], "centre": []}         # zig-zag zones: kerb / centre lines stop there
        self.bus_override = self.bus_lanes_from_imagery()
        self.crossings = self.find_crossings() if level == "ground" else []
        self.bands = unary_union([cr["band"] for cr in self.crossings]) if self.crossings else Polygon()
        self.plan_junctions()
        self.crossing_marks()
        self.lane_lines()
        self.junction_marks()
        self.bus_marks()
        self.gore_marks()
        self.yellow_boxes()
        self.clip()

    def build_index(self):
        """Markings.build_index, with the centre-side edges of EVERY part of a section's
        innermost lane (a lane split by a flyover has a part on each level).
        中央側の車線端は分割された全ての部分から。"""
        Markings.build_index(self)
        self.edges = []
        for lanes in self.sections.values():
            top = max(abs(l["lane"]) for l in lanes)
            for l in lanes:
                if abs(l["lane"]) == top and len(l["pts"]) >= 2:
                    self.edges.append((LineString([p["r" if self.lefthand else "l"] for p in l["pts"]]), l))
        self.tree = STRtree([e for e, _ in self.edges])

    # -- painting / 描画
    def add(self, g, col="w", cut="band"):
        """cut: None (never cut), "band" (stops at crossings), "kerb" / "centre"
        (also stops in zig-zag zones and, kerb lines, at bus bays).
        cut: 横断歩道で切る / ジグザグ区間・バス停でも切る。"""
        for p in polys(g):
            if p.area > 0.002:
                self.paint[(col, cut)].append(p)

    def mark(self, kind, g, col="w", cut=None, **info):
        if g is None or g.is_empty:
            return
        self.add(g, col, cut)
        e = {"kind": kind, "col": col, "geom": g}
        e.update(info)
        self.elements.append(e)
        self.count[kind] += 1

    # -- road classes / 道路の格
    def classify_sg(self, xodr):
        x = xodr_roads(xodr)
        self.rid_way, self.rid_rev, self.rid_len, self.rid_pred = x["way"], x["rev"], x["len"], x["pred"]
        self.rid_lights = x["lights"]
        # signal approaches: lights in the xodr (signs.py's final set) / 信号のある道路
        self.signal_roads = set(self.rid_lights)
        ways = self.L.osm.ways
        self.cls, self.road_cls, self.tags = {}, {}, {}
        for key, lanes in self.sections.items():
            l = lanes[0]
            t = ways.get(self.rid_way.get(l["road"], ""), (None, {}))[1]
            self.tags[l["road"]] = t
            hw = t.get("highway", "")
            n = len({abs(q["lane"]) for q in lanes})
            sp = l["speed"] or 13.9
            w = float(np.median([q["w"] for q in l["pts"]]))
            if hw in ("motorway", "trunk") and sp >= 19.0 or hw == "motorway":
                c = EXPWY
            elif hw.endswith("_link"):
                c = LINK
            elif hw in ("trunk", "primary", "secondary") or n >= 2 or l["name"] == "Ring Road":
                c = MAIN
            elif hw == "service" or t.get("lane_markings") == "no" or w < 2.95:
                c = NARROW
            else:
                c = ESTATE
            self.cls[key] = c
            self.road_cls[l["road"]] = max(self.road_cls.get(l["road"], 0), c)
        n = collections.Counter(self.cls.values())
        self.count["sections " + "/".join(CLASS_NAME[c] for c in (EXPWY, MAIN, LINK, ESTATE, NARROW))] = \
            "/".join(str(n[c]) for c in (EXPWY, MAIN, LINK, ESTATE, NARROW))

    def bus_lanes_from_imagery(self):
        """maps.json "sg_lta" "bus_lanes": [{lon, lat, where}] points on a kerb lane seen as a
        bus lane in imagery but not tagged in OSM: that lane's section gets one.
        画像で確認した OSM に無いバスレーン（縁石側車線上の点）。"""
        out = set()
        fr = geo.Frame(self.L.meta)
        for item in self.cfg.get("bus_lanes", []):
            p = Point(*fr.xy(item["lon"], item["lat"]))
            best = None
            for (road, sec), lanes in self.sections.items():
                k = min(lanes, key=lambda l: abs(l["lane"]))
                d = LineString([q["c"] for q in k["pts"]]).distance(p) if len(k["pts"]) >= 2 else 99.0
                if d <= 3.0 and (best is None or d < best[0]):
                    best = (d, (road, sec))
            if best is None:
                self.count["imagery bus lanes not matched"] += 1
            else:
                out.add(best[1])
                self.count["imagery bus lanes"] += 1
        return out

    def cls_of(self, l):
        return self.cls.get((l["road"], l["section"]), ESTATE)

    def opposite_of(self, edge, l, tol=1.3):
        """The innermost lane of the other direction along this lane's centre-side
        edge (most of its length within tol: one-way pairs with a paved gap < 2.4 m
        read as one carriageway, layout.py), or None. / 対向車線（無ければ None）。"""
        if edge.length < 1.0:
            return None
        pts = [edge.interpolate(t, normalized=True) for t in np.linspace(0.1, 0.9, 7)]
        for i in self.tree.query(edge.buffer(tol)):
            e2, l2 = self.edges[int(i)]
            if l2["road"] == l["road"]:
                continue
            if sum(1 for p in pts if e2.distance(p) <= tol) >= 4:
                return l2
        return None

    # -- crossings / 横断歩道
    def find_crossings(self):
        """Marked crossings from OSM only (untyped ones are unmarked in Singapore),
        placed by MarkingsJP.place_crossing. / OSM の種別付き横断歩道のみ。"""
        L = self.L
        cw = L.carriageway
        cands = []

        def kind_of(t):
            c, m = t.get("crossing", ""), t.get("crossing:markings", "")
            if c in ("no", "unmarked", "informal") or m == "no":
                return None
            sig = c in ("traffic_signals", "pedestrian_signals") or t.get("crossing:signals") == "yes" or \
                t.get("crossing_ref") in ("pelican", "toucan", "puffin") or t.get("highway") == "traffic_signals" \
                or (m in ("dots", "lines") and c in ("", "traffic_signals"))
            zeb = c in ("zebra", "uncontrolled", "marked") or m == "zebra" or t.get("crossing_ref") == "zebra" or \
                (m == "yes" and c != "traffic_signals")
            if sig:
                return "signal"
            return "zebra" if zeb else None

        for ln, t in L.crossing_ways:
            k = kind_of(t)
            seg = ln.intersection(cw)
            if k and not seg.is_empty:
                cands.append((seg.centroid, t, k, "osm way"))
        for p, t, nid in L.osm.points(lambda t: t.get("highway") == "crossing" or
                                      (t.get("highway") == "traffic_signals" and "crossing" in t)):
            k = kind_of(t)
            if k and p.distance(cw) < 0.5:
                cands.append((p, t, k, "osm node %s" % nid))
        fr = geo.Frame(L.meta)
        for c in self.cfg.get("crossings", []):         # from imagery / 画像で確認
            x, y = fr.xy(c["lon"], c["lat"])
            cands.append((Point(x, y), {}, "signal" if c.get("signal") else "zebra", "imagery %s" % c.get("where", "")))
        kept = []
        for p, t, k, src in cands:
            if all(p.distance(q) > 6.0 for q, _, _, _ in kept):
                kept.append((p, t, k, src))
        self.lane_ls = [(LineString([q["c"] for q in l["pts"]]), l) for l in self.lanes if len(l["pts"]) >= 2]
        self.lane_tree = STRtree([g for g, _ in self.lane_ls])
        out = []
        for p, t, k, src in kept:
            cr = self.place_crossing(p)
            if cr is None:
                self.count["crossings not placed (no ground road under the point)"] += 1
                continue
            cr["signal"] = k == "signal"
            cr["kind"], cr["src"], cr["point"] = ("signal" if k == "signal" else "zebra"), src, [p.x, p.y]
            out.append(cr)
        out = [self.to_kerb(cr) for cr in out]
        out = [cr for cr in out if cr is not None]
        out = self.merge_crossings(out)
        out = self.drop_twins(out)
        for cr in out:
            cr["kind"] = "signal" if cr["signal"] else "zebra"
        return out

    def to_kerb(self, cr, reach=8.0, shift_ok=True):
        """Singapore crossings run kerb to kerb: a band end left inside the
        carriageway (lanes + 2.5 m of MarkingsJP.band_at; flares, bays, paved gaps)
        is carried on to the kerb, up to ``reach`` m, unless that part runs over
        junction lanes or another road's lanes; a band that is still a stub
        (< 2.5 m, or mostly over a junction that is not a straight pass) is left
        out. / 横断歩道を縁石まで延ばす（交差点・他道路にかかる場合は延ばさない）。"""
        cw, jl = self.L.carriageway, self.L.junction_lanes
        c, n, f = np.array(cr["c"]), np.array(cr["n"]), np.array(cr["f"])
        w = cr["width"]
        full = LineString([c + n * (cr["t0"] - reach), c + n * (cr["t1"] + reach)]).intersection(cw)
        pieces = [g for g in getattr(full, "geoms", [full]) if g.geom_type == "LineString"]
        tt = lambda q: float((np.asarray(q) - c) @ n)
        t0, t1 = cr["t0"], cr["t1"]
        for g in pieces:
            a, b = sorted((tt(g.coords[0]), tt(g.coords[-1])))
            if b < t0 - 0.05 or a > t1 + 0.05:
                continue                                    # a different carriageway / 別の車道
            for side, new in ((0, a), (1, b)):
                old = t0 if side == 0 else t1
                if abs(new - old) < 0.3:
                    continue
                lo, hi = sorted((old, new))
                add = LineString([c + n * lo, c + n * hi]).buffer(w / 2, cap_style="flat").intersection(cw)
                if add.is_empty or add.area < 0.05:
                    continue
                if add.intersection(jl).area > 0.3 * add.area:
                    trial = dict(cr, band=unary_union([cr["band"], add]))
                    if self.turning_share(trial) > JUNCTION_TURN_MAX:
                        continue        # it would run into the junction / 交差点に入り込む
                other = [l for gl, l in self.lane_ls if gl.intersects(add) and l["road"] not in cr.get("roads", [])
                         and abs(float(tangent(gl, gl.project(Point(*c))) @ f)) < math.cos(math.radians(30.0))]
                if other:
                    continue
                if side == 0:
                    t0 = min(t0, new)
                else:
                    t1 = max(t1, new)
        if (t0, t1) != (cr["t0"], cr["t1"]):
            line = LineString([c + n * t0, c + n * t1]).intersection(cw)
            segs = [g for g in getattr(line, "geoms", [line]) if g.geom_type == "LineString" and g.length > 0.2]
            if segs:
                cr["segs"] = segs
                cr["band"] = unary_union([g.buffer(w / 2, cap_style="flat") for g in segs]).intersection(cw)
                cr["t0"], cr["t1"] = t0, t1
                cr["seg_roads"] = [cr.get("roads", [])] * len(segs)
                self.count["crossings carried to the kerb"] += 1
        span = sum(g.length for g in cr["segs"])
        edge = cw.boundary
        if span < 2.5 or cr["band"].area < 0.6 * span * w:
            self.count["crossings left out (stub)"] += 1
            return None

        def fit(q):
            """(ends at the kerb, junction share, turning share). / 縁石・交差点の割合。"""
            cq, nq = np.array(q["c"]), np.array(q["n"])
            e = max(edge.distance(Point(*(cq + nq * t))) for t in (q["t0"], q["t1"]))
            # an end on a corner (an island's tip) with road again just past it is not at a
            # kerb: walkers would step back onto the carriageway / 島の先端（先がまた車道）は縁石ではない
            if any(cw.contains(Point(*(cq + nq * (t + sg * 0.5)))) for t, sg in ((q["t0"], -1.0), (q["t1"], 1.0))):
                e = max(e, 1.0)
            tot = q["band"].intersection(jl).area / max(q["band"].area, 1e-6)
            return e, tot, self.turning_share(q)

        def good(v):
            return v[0] <= 0.35 and v[1] <= JUNCTION_TOTAL_MAX and v[2] <= JUNCTION_TURN_MAX
        here = fit(cr)
        if not shift_ok or (good(here) and self.walk_ok(cr)):
            return cr
        # an end short of the kerb, too much of it over the junction, or an end that meets no
        # footway: slide it along the road (up to 10 m) to where it spans kerb to kerb clear of
        # the junction, both ends on the walking network if possible; a band ending mid-road or
        # lying in a junction is left out (worse than none), one only missing a footway stays
        # 縁石に届かない・交差点に入り込む・歩道に繋がらない: 道路沿いに最大 10 m ずらす
        cands = []
        for k in range(1, 21):
            for sgn in (1.0, -1.0):
                try:
                    cr2 = self.band_at(c + f * sgn * 0.5 * k, f, cw, f)
                except ValueError:
                    cr2 = None      # MarkingsJP.band_at: its refined heading found no lane / 車線なし
                if cr2 is None:
                    continue
                for key in ("signal", "kind", "src", "point"):
                    cr2[key] = cr.get(key)
                cr2 = self.to_kerb(cr2, reach, shift_ok=False)
                if cr2 is not None and good(fit(cr2)):
                    # only 3 m for a footway alone (the band stays where OSM has it)
                    # 歩道だけが理由なら 3 m まで
                    if self.walk_ok(cr2) and (k <= 6 or not good(here)):
                        self.count["crossings slid onto the walking network"] += 1
                        return cr2
                    cands.append(cr2)
        if good(here):
            self.count["crossings with an end off the walking network (pavement)"] += 1
            return cr
        if cands:
            self.count["crossings slid to span kerb to kerb clear of the junction"] += 1
            return cands[0]
        self.count["crossings left out (kerb / junction)"] += 1
        self.left_out = getattr(self, "left_out", []) + [
            (cr.get("src"), [round(v, 1) for v in cr["c"]],
             "kerb gap %.1f m, junction %.0f %%, turning %.0f %%" % (here[0], 100 * here[1], 100 * here[2]))]
        return None

    def drop_twins(self, out):
        """Two parallel bands side by side (< 6 m apart along the road, sharing half
        their span): one OSM way across both carriageways and one per carriageway.
        Keep the one less over the junction. / 並んだ平行な 2 本の横断歩道は 1 本に。"""
        jl = self.L.junction_lanes
        drop = set()
        for i, a in enumerate(out):
            for k, b in enumerate(out):
                if k <= i or i in drop or k in drop:
                    continue
                fa, fb = np.array(a["f"]), np.array(b["f"])
                if abs(float(fa @ fb)) < 0.97:
                    continue
                d = float((np.array(b["c"]) - np.array(a["c"])) @ fa)
                if abs(d) > 6.0 or a["band"].distance(b["band"]) > 1.0:
                    continue
                na = np.array(a["n"])
                ta = (a["t0"], a["t1"])
                tb = sorted(float((np.array(b["c"]) + np.array(b["n"]) * t - np.array(a["c"])) @ na)
                            for t in (b["t0"], b["t1"]))
                share = max(0.0, min(ta[1], tb[1]) - max(ta[0], tb[0])) / max(1e-6, min(ta[1] - ta[0], tb[1] - tb[0]))
                if share < 0.5:
                    if abs(d) < 4.0 and share <= 0.05:
                        # two halves of one crossing, staggered: align b on a and join
                        # 互い違いの 2 つの半分: 位置を揃えて 1 本に
                        from shapely.affinity import translate as _tr
                        sh = -fa * d
                        b_band = _tr(b["band"], sh[0], sh[1]).intersection(self.L.carriageway)
                        a["band"] = unary_union([a["band"], b_band])
                        a["segs"] = a["segs"] + [_tr(g, sh[0], sh[1]) for g in b["segs"]]
                        a["seg_roads"] = a.get("seg_roads", []) + b.get("seg_roads", [[]] * len(b["segs"]))
                        a["roads"] = sorted(set(a.get("roads", [])) | set(b.get("roads", [])))
                        a["t0"], a["t1"] = min(ta[0], tb[0]), max(ta[1], tb[1])
                        a["signal"] = a["signal"] or b["signal"]
                        a["src"] += " + " + b["src"]
                        drop.add(k)
                        self.count["crossing halves joined"] += 1
                    continue
                ja = a["band"].intersection(jl).area / max(a["band"].area, 1e-6)
                jb = b["band"].intersection(jl).area / max(b["band"].area, 1e-6)
                lose = i if ja > jb else k
                keep = out[k if lose == i else i]
                keep["signal"] = keep["signal"] or out[lose]["signal"]
                drop.add(lose)
                self.count["crossings dropped (parallel twin)"] += 1
        return [cr for i, cr in enumerate(out) if i not in drop]

    def walk_ok(self, cr):
        """Both band ends meet the walking network the model builds (footpaths minus
        islands on large maps, aprons, park connectors; check_paths.py looks 0.3-0.9 m
        past the end). / 帯の両端が歩行面に接する。"""
        if not hasattr(self, "_walk"):
            import shapely as _sh
            L = self.L
            fp = L.footpaths.difference(L.islands) if L.meta.get("roadnet") else L.footpaths
            self._walk = unary_union([fp, L.aprons, L.cycleways])
            _sh.prepare(self._walk)
        c, n = np.array(cr["c"]), np.array(cr["n"])
        for t, sg in ((cr["t0"], -1.0), (cr["t1"], 1.0)):
            if not any(self._walk.distance(Point(*(c + n * (t + sg * d)))) < 0.25 for d in (0.3, 0.5, 0.7, 0.9)):
                return False
        return True

    def turning_share(self, cr):
        """Share of the band over junction lanes that turn across it (not a dead-end
        loop, not running along the road). / 帯のうち曲がる交差点車線に掛かる割合。"""
        f = np.array(cr["f"])
        dead = set(getattr(self.L, "dead_end_junctions", []) or [])
        turn = []
        for l in self.L.lanes:
            if l["junction"] < 0 or len(l["pts"]) < 2 or l["junction"] in dead:
                continue
            g = LineString([q["c"] for q in l["pts"]])
            if g.intersects(cr["band"]) and \
                    abs(float(tangent(g, g.project(cr["band"].centroid)) @ f)) < math.cos(math.radians(15.0)):
                turn.append(lane_poly(l))
        return unary_union(turn).intersection(cr["band"]).area / max(cr["band"].area, 1e-6) if turn else 0.0

    def straight_junction(self, cr):
        """The junction lanes under the band all run along the road (a junction made
        for the crossing itself, or a dead-end loop's neck): the band may lie on them.
        帯の下の交差点車線が道路方向に通り抜けるだけ（横断歩道の交差点・転回路の首）。"""
        f = np.array(cr["f"])
        dead = set(getattr(self.L, "dead_end_junctions", []) or [])
        for l in self.L.lanes:
            if l["junction"] < 0 or len(l["pts"]) < 2:
                continue
            g = LineString([q["c"] for q in l["pts"]])
            if not g.intersects(cr["band"]) or l["junction"] in dead:
                continue
            if abs(float(tangent(g, g.project(cr["band"].centroid)) @ f)) < math.cos(math.radians(15.0)):
                return False
        return True

    def crossing_marks(self):
        for k, cr in enumerate(self.crossings):
            f, n, c = np.array(cr["f"]), np.array(cr["n"]), np.array(cr["c"])
            w = cr["width"]
            if cr["signal"]:
                # A4: two rows of 0.2 m studs, 0.2 m apart, along the band edges / 信号横断歩道: 鋲状の 2 列
                studs = []
                for sgn in (-1, 1):
                    t = cr["t0"] + 0.1
                    while t + STUD <= cr["t1"] - 0.05:
                        q = c + n * (t + STUD / 2) + f * sgn * (w / 2 - STUD / 2)
                        studs.append(box(-STUD / 2, -STUD / 2, STUD / 2, STUD / 2).buffer(0))
                        studs[-1] = self.place(studs[-1], q, cr["yaw"])
                        t += STUD + STUD_GAP
                self.mark("signal_crossing", unary_union(studs).intersection(cr["band"].buffer(0.05)), crossing=k)
                continue
            bars = []
            t = cr["t0"] + 0.25
            while t + ZEBRA_BAR <= cr["t1"] - 0.2:
                q = c + n * (t + ZEBRA_BAR / 2)
                bars.append(LineString([q - f * w / 2, q + f * w / 2]).buffer(ZEBRA_BAR / 2, cap_style="flat"))
                t += ZEBRA_BAR + ZEBRA_GAP
            self.mark("zebra", unary_union(bars).intersection(cr["band"]) if bars else None, crossing=k)
            self.approach_marks(k, cr)

    def approach_marks(self, k, cr):
        """In every lane that runs into a zebra. Main roads and slip roads: K zig-zags at
        the kerb and on the centre line (RMS9), on slip roads also a transverse line 2 m
        before (Street View). Estate and service roads (Street View, Tampines St 45, Jul
        2024): no zig-zags, the kerb lines run on, the centre line opens into a hatched
        taper round the refuge (RMS8 diagonal marking). No SLOW: LTA's drawings have none
        and the street does not show it at the crossing.
        幹線・導流路: ジグザグ線。団地内道路: ジグザグ無し、中央線は導流帯（斜線）に。"""
        for lane in self.lanes:
            ln = LineString([q["c"] for q in lane["pts"]])
            if ln.length < 4.0 or ln.distance(cr["band"]) > 0.3:
                continue
            hit = ln.intersection(cr["band"])
            if hit.is_empty:
                continue
            s0 = min(ln.project(Point(q)) for g in getattr(hit, "geoms", [hit]) for q in getattr(g, "coords", []))
            if s0 < 3.0:
                continue
            ids = sorted({abs(l["lane"]) for l in self.sections.get((lane["road"], lane["section"]), [lane])})
            i = ids.index(abs(lane["lane"]))
            kerb_edge = LineString([q["l" if self.lefthand else "r"] for q in lane["pts"]])
            cen_edge = LineString([q["r" if self.lefthand else "l"] for q in lane["pts"]])
            out_k = 1.0 if self.lefthand else -1.0
            estate = self.cls_of(lane) in (ESTATE, NARROW)
            if estate:
                if i == len(ids) - 1 and self.opposite_of(cen_edge, lane) is not None:
                    self.hatch_taper(k, lane, ln, cen_edge, s0)
                continue
            z1 = s0 - 1.0
            z0 = max(0.5, z1 - ZZ_LEG * ZZ_LEGS)
            if z1 - z0 >= ZZ_LEG:
                legs = int((z1 - z0) // ZZ_LEG)
                z0 = z1 - legs * ZZ_LEG
                if i == 0:
                    # 0.35 m off the kerb face (gutter 0.4 m) / 縁石から 0.35 m
                    self.zigzag(k, lane, ln, kerb_edge, out_k * 0.05, z0, z1, "kerb")
                if i == len(ids) - 1 and self.opposite_of(cen_edge, lane) is not None:
                    self.zigzag(k, lane, ln, cen_edge, 0.0, z0, z1, "centre")
            if self.cls_of(lane) == LINK and s0 > 3.0:
                # slip roads: a continuous line 2 m before the zebra (Street View) / 導流路: 横断歩道 2 m 手前の横線
                q, fv = self.at(ln, s0 - 2.0 - 0.15)
                nv = np.array([-fv[1], fv[0]])
                wl = lane["pts"][min(len(lane["pts"]) - 1, int(s0))]["w"]
                self.mark("zebra_line", strip(LineString([q - nv * wl / 2, q + nv * wl / 2]), 0.3), crossing=k,
                          road=lane["road"], lane=lane["lane"])

    def hatch_taper(self, k, lane, ln, edge, s0):
        """Hatched taper on the centre line before an estate zebra: a wedge 0 -> HT_W wide
        over HT_LEN m ending 0.5 m before the band, outline + 45 deg hatching (RMS8, 0.15 m
        lines). / 団地内の横断歩道手前の導流帯（中央線上の斜線のくさび）。"""
        z1 = s0 - 0.5
        z0 = max(0.5, z1 - HT_LEN)
        if z1 - z0 < 5.0:
            return
        L0, L1 = ln.length, edge.length
        left, right = [], []
        for u in np.linspace(0.0, 1.0, 9):
            s = (z0 + (z1 - z0) * u) / L0 * L1
            p = edge.interpolate(s)
            tv = tangent(edge, s)
            nv = np.array([-tv[1], tv[0]])
            hw = max(0.05, HT_W / 2 * u)
            left.append(tuple(np.array([p.x, p.y]) + nv * hw))
            right.append(tuple(np.array([p.x, p.y]) - nv * hw))
        wedge = Polygon(left + right[::-1]).buffer(0)
        if wedge.is_empty:
            return
        outline = wedge.difference(wedge.buffer(-0.10, join_style="mitre"))
        pe = edge.interpolate(z1 / L0 * L1)
        te = tangent(edge, z1 / L0 * L1)
        nn = np.array([-te[1], te[0]])
        dd = (te + nn) / math.sqrt(2.0)
        hatch = []
        for j in range(int(HT_LEN / HT_PITCH) + 2):
            c0 = np.array([pe.x, pe.y]) - te * (j * HT_PITCH)
            hatch.append(LineString([tuple(c0 - dd * 3.0), tuple(c0 + dd * 3.0)]).buffer(0.075, cap_style="flat"))
        g = unary_union([outline, unary_union(hatch).intersection(wedge.buffer(-0.05))])
        self.mark("hatch", g, crossing=k, road=lane["road"], lane=lane["lane"])
        self.zones["centre"].append(wedge.buffer(0.3))

    def zigzag(self, k, lane, ln, edge, d, z0, z1, side):
        """K zig-zag along ``edge`` (offset d, + = left) between lane distances z0..z1
        (the crossing end at z1), with the 0.3 x 0.5 m end block. / K ジグザグ線。"""
        base = offset(edge, d) if abs(d) > 1e-6 else edge
        if base is None:
            return
        # lane distance -> edge distance (same parameter along the lane) / 車線上の距離を端線へ
        L0, L1 = ln.length, base.length
        k0, k1 = z0 / L0 * L1, z1 / L0 * L1
        pts = []
        legs = int(round((k1 - k0) / (ZZ_LEG / L0 * L1)))
        for j in range(legs + 1):
            s = k0 + (k1 - k0) * j / max(legs, 1)
            p = base.interpolate(s)
            tv = tangent(base, s)
            nv = np.array([-tv[1], tv[0]])
            amp = ZZ_AMP if j % 2 else -ZZ_AMP
            pts.append(np.array([p.x, p.y]) + nv * amp)
        if len(pts) < 2:
            return
        zz = LineString(pts).buffer(ZZ_W / 2, join_style="mitre", mitre_limit=5.0)
        pe = base.interpolate(k1)
        te = tangent(base, k1)
        blk = self.place(box(-0.3, -0.25, 0.0, 0.25), (pe.x, pe.y), math.atan2(te[1], te[0]))
        self.mark("zigzag", unary_union([zz, blk]), crossing=k, road=lane["road"], lane=lane["lane"], side=side)
        zone = substring(base, max(0.0, k0 - 0.5), min(L1, k1 + 0.5)).buffer(0.75, cap_style="flat")
        self.zones[side].append(zone)

    def word_fit(self, text, p, yaw, along, across):
        """Road word, letters ``along`` m in the travel direction, ``across`` m wide in
        all, reading left to right for the driver, centred on p. / 路面文字。"""
        g = glyphs(text, along, 1.0)
        if g is None:
            return None
        w = g.bounds[2] - g.bounds[0]
        from shapely.affinity import scale as _scale
        g = _scale(g, across / max(w, 1e-6), 1.0, origin=(0, 0))
        return self.place(rotate(g, -90, origin=(0, 0)), p, yaw)

    # -- junctions: what each approach gets / 交差点の進入路
    def plan_junctions(self):
        appr = collections.defaultdict(list)
        for a in self.L.approaches:
            appr[(a["road"], a["junction"])].append(a)
        lane_by = {}
        for l in self.lanes:
            lane_by.setdefault((l["road"], l["lane"]), l)       # the run that reaches the end / 終端の部分
        self.lane_by = lane_by
        at_j = collections.defaultdict(list)
        for (road, j), al in appr.items():
            at_j[j].append((road, al))
        self.plan = {}
        # lights on stub roads (< 3 m: netconvert split the approach at a small junction
        # just before the big one): the road entering that small junction is the
        # signalised approach / 短い分割道路の信号: その手前の小交差点に入る道路が信号の進入路
        stub_j = {self.rid_pred[r] for r in self.signal_roads if self.rid_len.get(r, 99.0) < 3.0 and r in self.rid_pred}
        self.stub_signal = {road for (road, j) in appr if j in stub_j and road not in self.signal_roads}
        for (road, j), al in appr.items():
            byl = collections.OrderedDict()
            for a in al:
                byl.setdefault(a["lane"], set()).update(a["turns"])
            lanes = [(lane_by.get((road, ln)), t) for ln, t in byl.items()]
            lanes = [(l, t) for l, t in lanes if l is not None and len(l["pts"]) >= 3 and l.get("at_end", True)]
            if not lanes:
                continue
            signal = road in self.signal_roads or road in self.stub_signal
            back, cross_at = self.setback([l for l, t in lanes])
            # no pole rule here: the light's s in the road logic is netconvert's placeholder
            # (4 m before the mouth); signs.py moves the light to this painted line and stands
            # the pole past it (pole_s) / 柱による後退はしない（signs.py が信号を停止線へ、柱をその先へ）
            pb = None
            mine = self.road_cls.get(road, ESTATE)
            yaw = math.radians(al[0]["yaw"])
            crossing_roads = []
            for r2, al2 in at_j[j]:
                if r2 == road:
                    continue
                d = abs((math.degrees(math.radians(al2[0]["yaw"]) - yaw) + 180.0) % 360.0 - 180.0)
                if 25.0 <= d <= 155.0:
                    crossing_roads.append(r2)
            top = max([self.road_cls.get(r, ESTATE) for r in crossing_roads], default=0)
            give = (not signal and crossing_roads and mine < top and mine != EXPWY
                    and not any(self.road_cls.get(r, 0) == EXPWY for r in crossing_roads))
            turns = [frozenset(t) for l, t in lanes]
            arrows = mine not in (EXPWY,) and len(lanes) >= 2 and (signal or len(set(turns)) > 1)
            self.plan[(road, j)] = {"lanes": lanes, "signal": signal, "give": give, "back": back,
                                    "crossing": cross_at, "arrows": arrows, "pole": pb}
        # lights at a road end with no junction (mid-block signalised crossings, the road
        # continuing straight on): a stop line there too / 交差点の無い信号（単路の押しボタン信号）
        planned = {road for (road, j) in self.plan}
        for road in self.signal_roads - planned:
            lanes = [(l, set()) for l in self.lanes if l["road"] == road and len(l["pts"]) >= 3
                     and l.get("at_end", True)]
            if not lanes:
                continue
            back, cross_at = self.setback([l for l, t in lanes])
            pb = None
            self.plan[(road, None)] = {"lanes": lanes, "signal": True, "give": False, "back": back,
                                       "crossing": cross_at, "arrows": False, "pole": pb}
            self.count["mid-block signal stop lines"] += 1
        # a signal approach too short for its line (the crossing sits on it): the line
        # goes on the roads entering the small junction before it / 停止線が入らない短い進入路: 手前の道路へ
        for (road, j), v in [kv for _ in range(4) for kv in list(self.plan.items())]:    # up to 4 hops / 最大 4 段
            if not v["signal"] or road not in self.rid_pred:
                continue
            room = min(LineString([q["c"] for q in l["pts"]]).length for l, t in v["lanes"]) - v["back"] - STOP_W
            if room >= 0.5:
                continue
            j0 = self.rid_pred[road]
            ups = [(k, w) for k, w in self.plan.items() if k[1] == j0 and k[0] != road]
            if not ups:
                continue
            extra = v["back"] - self.rid_len.get(road, 0.0)
            for k, w in ups:
                # the lights this line serves (check_markings) / この停止線が受け持つ信号の道路
                w["serves"] = sorted(set(w.get("serves", [])) | {road} | set(v.get("serves", [])))
                w["signal"], w["give"] = True, False
                w["back"] = max(w["back"], extra, STOP_PLAIN)
                w["arrows"] = self.road_cls.get(k[0], ESTATE) != EXPWY and len(w["lanes"]) >= 2
            v["signal"], v["arrows"] = False, False
            v["moved_up"] = j0
            self.count["signal lines moved upstream (short approach)"] += 1
        self.signal_back = {road: v["back"] for (road, j), v in self.plan.items() if v["signal"]}

    def setback(self, lanes):
        """MarkingsJP.setback with LTA's numbers (2 m before the crossing, 1 m
        before the mouth). / LTA の値で停止線位置。"""
        best, which = STOP_PLAIN, None
        for k, cr in enumerate(self.crossings):
            for l in lanes:
                P = np.array([q["c"] for q in l["pts"]])
                f = P[-1] - P[-3]
                f = f / (np.linalg.norm(f) or 1.0)
                ext = LineString(list(map(tuple, P)) + [tuple(P[-1] + f * 3.0)])
                hit = ext.intersection(cr["band"])
                if hit.is_empty:
                    continue
                ss = [ext.project(Point(q)) for g in getattr(hit, "geoms", [hit]) for q in g.coords]
                s_min, s_max = min(ss), max(ss)
                gap = (ext.length - 3.0) - s_min
                if gap > STOP_NEAR_CROSS or (s_min < 2.0 and (ext.length - 3.0) - s_max >= 3.0):
                    continue        # far, or at the lane's start with room after it: the crossing of the
                    #                 junction behind this road / 遠い・後ろの交差点の横断歩道（始点）
                if gap + STOP_BACK > best:
                    best, which = gap + STOP_BACK, k
        return best, which

    def junction_marks(self):
        for (road, j), v in self.plan.items():
            where = {"road": road, "junction": j, "signal": v["signal"], "back": v["back"], "crossing": v["crossing"],
                     "serves": v.get("serves", [])}
            for l, turns in v["lanes"]:
                ln = LineString([q["c"] for q in l["pts"]])
                total = ln.length
                w = l["pts"][-1]["w"]
                if v["signal"]:
                    sc = total - v["back"] - STOP_W / 2
                    if sc > STOP_W:
                        q, f = self.at(ln, sc)
                        nv = np.array([-f[1], f[0]])
                        self.mark("stop_signal", strip(LineString([q - nv * (w / 2 + 0.05), q + nv * (w / 2 + 0.05)]),
                                                       STOP_W), lane=l["lane"], s=sc, **where)
                elif v["give"]:
                    for kk, d in enumerate((0.0, GIVE_W + GIVE_SEP)):
                        sc = total - v["back"] - GIVE_W / 2 - d
                        if sc < 1.0:
                            continue
                        q, f = self.at(ln, sc)
                        nv = np.array([-f[1], f[0]])
                        across = LineString([q - nv * w / 2, q + nv * w / 2])
                        self.mark("giveway", unary_union([strip(g, GIVE_W) for g in dashes(across, GIVE_DASH, GIVE_GAP)]),
                                  lane=l["lane"], s=sc, row=kk, **where)
                if v["arrows"] and turns:
                    for dist in ARROW_AT:
                        sc = total - v["back"] - STOP_W - dist
                        if sc - 3.0 > 0.5:
                            q, f = self.at(ln, sc)
                            self.mark("arrow", self.arrow_lta(q, math.atan2(f[1], f[0]), turns), lane=l["lane"],
                                      turns=sorted(turns), s=sc, **where)
            if v["give"]:
                self.giveway.append((road, j))

    def arrow_lta(self, p, yaw, turns):
        """LTA RMS4 lane arrows (5.5 m): straight, turn, straight + turn, drawn
        pointing +x, left = +y. / LTA の矢印。"""
        parts = []
        straight = "straight" in turns or not ({"left", "right"} & set(turns))
        if straight:
            parts.append(Polygon([(-2.75, -0.15), (0.45, -0.09), (0.45, 0.09), (-2.75, 0.15)]))
            parts.append(Polygon([(0.45, -0.36), (2.75, 0.0), (0.45, 0.36)]))
        for side in ("left", "right"):
            if side not in turns:
                continue
            g = 1.0 if side == "left" else -1.0
            if straight:
                # a side head off the shaft (RMS4 third figure) / 軸から分岐する矢頭
                stem = LineString([(-1.3, 0.0), (-0.6, 0.25 * g), (0.0, 0.55 * g)]).buffer(0.08, join_style="round")
                d = np.array([0.6, 0.55 * g])
                tip = np.array([0.0, 0.55 * g]) + d / np.linalg.norm(d) * 1.3
            else:
                stem = LineString([(-2.75, 0.0), (-0.4, 0.0), (0.3, 0.3 * g), (0.8, 0.8 * g)]).buffer(
                    0.1, join_style="round")
                d = np.array([0.5, 0.5 * g])
                tip = np.array([0.8, 0.8 * g]) + d / np.linalg.norm(d) * 1.5
            dn = d / np.linalg.norm(d)
            nn = np.array([-dn[1], dn[0]])
            b = tip - dn * (1.3 if straight else 1.5)
            parts += [stem, Polygon([tuple(b + nn * 0.36), tuple(tip), tuple(b - nn * 0.36)])]
        return self.place(unary_union(parts), p, yaw)

    # -- lines along the lanes / 車線に沿う線
    def lane_lines(self):
        for key, lanes in self.sections.items():
            lanes = sorted(lanes, key=lambda l: abs(l["lane"]))
            ids = sorted({abs(l["lane"]) for l in lanes})
            n = len(ids)
            c = self.cls.get(key, ESTATE)
            t = self.tags.get(lanes[0]["road"], {})
            try:
                nbus = int(str(t.get("lanes:bus", "0")).split(";")[0])
            except ValueError:
                nbus = 0
            bus = (nbus >= 1 or key in self.bus_override) and n >= 2
            for l in lanes:
                i = ids.index(abs(l["lane"]))
                left = LineString([q["l"] for q in l["pts"]])
                right = LineString([q["r"] for q in l["pts"]])
                if left.length < 1.0:
                    continue
                kerb_edge, k_out = (left, 1.0) if self.lefthand else (right, -1.0)
                cen_edge = right if self.lefthand else left
                if i + 1 < n:
                    if bus and i == 0:
                        self.bus_lane(l, cen_edge)
                    else:
                        self.divider(l, cen_edge, c)
                if i == 0:
                    self.kerb_line(l, kerb_edge, k_out, c)
                if i == n - 1:
                    opp = self.opposite_of(cen_edge, l)
                    if opp is not None:
                        self.centre_line(l, opp, cen_edge, k_out, c)
                    else:
                        self.median_line(l, cen_edge, k_out, c, n)

    def divider(self, l, line, c):
        L = line.length
        if c == EXPWY:
            for d in dashes(line, B1_DASH, B1_GAP):
                self.add(strip(d, LINE_W), "w", "band")
            return
        back = self.signal_back.get(l["road"]) if l.get("at_end", True) else None
        c_from = L
        if back is not None:
            end = L - back - STOP_W          # the stop line's back edge / 停止線の手前側
            c_from = max(0.0, end - C_MARKS * (C_DASH + C_GAP) + C_GAP)
            s = end - C_DASH
            while s > c_from - 0.01 and s >= 0.0:
                self.add(strip(substring(line, max(0.0, s), s + C_DASH), LINE_W), "w", "band")
                s -= C_DASH + C_GAP
            c_from = max(0.0, s + C_DASH + C_GAP - B_GAP)
        if c_from > 0.5:
            for d in dashes(substring(line, 0.0, c_from), B_DASH, B_GAP):
                self.add(strip(d, LINE_W), "w", "band")

    def bus_lane(self, l, line):
        """Normal bus lane (RMS10 L / A1): 0.30 yellow line, broken near junctions,
        BUS at the start. / バスレーン。"""
        L = line.length
        brk_end, brk_start = 30.0, 12.0
        a, b = (brk_start if L > 60 else 0.0), max(0.0, L - brk_end)
        if b - a > 5.0:
            # L: the 0.30 line and a 1.2 x 0.2 m bar into the bus lane every 12 m (RMS2;
            # Street View, Tampines Ave 10) / 12 m ごとにバスレーン側へ 1.2 x 0.2 m の横棒
            parts = [strip(substring(line, a, b), BUS_L_W)]
            ks = 1.0 if self.lefthand else -1.0
            s = a + 6.0
            while s < b - 1.0:
                q, f = self.at(line, s)
                nv = np.array([-f[1], f[0]]) * ks              # toward the kerb (the bus lane) / 縁石側
                parts.append(strip(LineString([q, q + nv * 1.2]), 0.2))
                s += 12.0
            self.mark("bus_lane", unary_union(parts), "y", "band", road=l["road"], lane=l["lane"])
        for s0, s1 in ((0.0, a), (b, L)):
            if s1 - s0 > 1.0:
                for d in dashes(substring(line, s0, s1), 1.0, 1.0):
                    self.add(strip(d, LINE_W), "y", "band")
        ln = LineString([q["c"] for q in l["pts"]])
        if ln.length > 45.0:
            q, f = self.at(ln, a + 4.0)
            self.mark("bus_word", self.word_fit("BUS", q, math.atan2(f[1], f[0]), 2.7, min(2.4, l["pts"][0]["w"] - 0.6)),
                      "y", None, road=l["road"], lane=l["lane"])

    def kerb_line(self, l, edge, k_out, c):
        if c == NARROW:
            return
        if c == EXPWY:
            self.add(strip(offset(edge, k_out * 0.2), EXPWY_W), "w", "kerb")
        elif c == LINK:
            self.add(strip(offset(edge, k_out * 0.15), LINK_EDGE_W), "w", "kerb")
        elif c == MAIN and (self.level == "deck" or l.get("raised_part")):
            # flyovers and the ramps up to them: no yellow lines (RMS1 note 1) / 高架・取付部は黄線なし
            self.add(strip(offset(edge, k_out * 0.15), LINK_EDGE_W), "w", "kerb")
        elif c == MAIN and self.cfg.get("main_kerb", "double_yellow") == "double_yellow":
            for d in (0.10, 0.10 + DOUBLE_W + DOUBLE_GAP):
                self.add(strip(offset(edge, k_out * d), DOUBLE_W), "y", "kerb")
        elif self.cfg.get("estate_kerb", "white") == "white":
            self.add(strip(offset(edge, k_out * 0.15), EDGE_W), "w", "kerb")
        else:
            self.add(strip(offset(edge, k_out * 0.18), CENTRE_W), "y", "kerb")

    def centre_line(self, l, opp, edge, k_out, c):
        if c == NARROW and self.cls_of(opp) == NARROW:
            return
        if c in (MAIN, EXPWY, LINK) or self.cls_of(opp) in (MAIN, EXPWY):
            # H: each direction paints its line of the pair / 二重線の自分側
            self.add(strip(offset(edge, -k_out * (DOUBLE_GAP / 2 + DOUBLE_W / 2)), DOUBLE_W), "w", "centre")
        elif l["road"] < opp["road"]:
            for d in dashes(edge, E_DASH, E_GAP):
                self.add(strip(d, CENTRE_W), "w", "centre")

    def median_line(self, l, edge, k_out, c, n):
        if c == EXPWY:
            self.add(strip(offset(edge, -k_out * 0.2), EXPWY_W), "w", "kerb")
        elif c == LINK:
            self.add(strip(offset(edge, -k_out * 0.15), LINK_EDGE_W), "w", "kerb")
        elif n >= 3:
            self.add(strip(offset(edge, -k_out * 0.15), MEDIAN_W), "w", "kerb")

    def bw_zone(self):
        """Where kerbs get LTA's black-and-white paint (Street View: slip roads,
        traffic islands, junction corners of main roads; plain grey kerbs along
        estate roads such as Tampines St 45): ground.py paints kerb faces inside.
        白黒塗装の縁石の範囲（導流路・交通島・幹線交差点の隅）。"""
        L = self.L
        jcls = collections.defaultdict(int)
        for a in L.approaches:
            jcls[a["junction"]] = max(jcls[a["junction"]], self.road_cls.get(a["road"], ESTATE))
        parts = [lane_poly(l).buffer(0.95) for l in self.lanes if self.cls_of(l) == LINK and len(l["pts"]) >= 2]
        parts += [lane_poly(l).buffer(6.0) for l in L.lanes
                  if l["junction"] >= 0 and jcls.get(l["junction"], 0) >= MAIN and len(l["pts"]) >= 2]
        parts += [q.buffer(0.6) for q in polys(L.islands) if q.area < 600.0]
        return unary_union(parts) if parts else Polygon()

    # -- bus bays / バス停車帯
    def bus_marks(self):
        from layout import BAY_DEPTH, BAY_TAPER, GUTTER
        self.bay_zone = []
        if self.level != "ground":
            return                  # bays are cut into the ground carriageway (layout) / 停車帯は地上のみ
        sg = 1.0 if self.lefthand else -1.0
        for bay in getattr(self.L, "bus_bays", []):
            e = bay.get("edge")
            if e is None or e.length < 2 * BAY_TAPER + 4.0:
                continue
            mouth = offset(e, sg * 0.35)
            if mouth is None:
                continue
            parts = [strip(mouth, 0.15)]
            for s in (BAY_TAPER, e.length - BAY_TAPER):
                q, f = self.at(e, s)
                nv = np.array([-f[1], f[0]]) * sg
                parts.append(strip(LineString([q + nv * 0.35, q + nv * (GUTTER + BAY_DEPTH - 0.05)]), 0.15))
            self.mark("bus_bay", unary_union(parts).intersection(bay["poly"].buffer(0.45)), "y", None,
                      stop=bay.get("stop"))
            self.bay_zone.append(bay["poly"].buffer(0.3))

    # -- chevrons where carriageways merge / divide (RMS8) / 合流・分流部のゼブラ
    def gore_marks(self):
        """The paved wedge between two carriageways that meet at a narrow angle
        (layout paves gaps < 2.4 m), outside junctions: outline and chevrons
        pointing the way traffic goes. / 合流・分流部の舗装された三角地帯。"""
        L = self.L
        lp = [lane_poly(l) for l in self.lanes if len(l["pts"]) >= 2]
        if not lp:
            return
        from layout import GUTTER
        lanes_u = unary_union(lp)
        rest = self.cw.difference(lanes_u.buffer(GUTTER + 0.05)).difference(L.junction_lanes.buffer(1.0))
        bays = [b["poly"].buffer(1.0) for b in getattr(L, "bus_bays", []) if "poly" in b]
        if bays:
            rest = rest.difference(unary_union(bays))
        rest = rest.buffer(-0.15).buffer(0.15)
        ltree = STRtree([LineString([q["c"] for q in l["pts"]]) for l in self.lanes])
        lls = [l for l in self.lanes]
        for w in polys(rest):
            if w.area < 6.0:
                continue
            mr = w.minimum_rotated_rectangle
            cc = np.asarray(mr.exterior.coords)[:4]
            e0, e1 = cc[1] - cc[0], cc[2] - cc[1]
            ln, wd = max(np.linalg.norm(e0), np.linalg.norm(e1)), min(np.linalg.norm(e0), np.linalg.norm(e1))
            if ln < 15.0 or wd > 4.5 or ln / max(wd, 0.1) < 5.0:
                continue
            u = e0 / np.linalg.norm(e0) if np.linalg.norm(e0) >= np.linalg.norm(e1) else e1 / np.linalg.norm(e1)
            # the lanes either side: two different roads, nearly parallel / 両側の車線
            near = [int(i) for i in ltree.query(w.buffer(GUTTER + 0.6))]
            roads = {}
            for i in near:
                l = lls[i]
                g = LineString([q["c"] for q in l["pts"]])
                h = tangent(g, g.project(w.centroid))
                if abs(float(h @ u)) > math.cos(math.radians(25.0)):
                    roads.setdefault(l["road"], (h, l))
            if len(roads) < 2:
                continue
            h = np.mean([v[0] for v in roads.values()], axis=0)
            h = h / (np.linalg.norm(h) or 1.0)
            if any(self.cls_of(v[1]) in (NARROW,) for v in roads.values()):
                continue
            fwd = h if float(h @ u) >= 0 else -h
            # chevrons: V apex pointing forward, 45 deg arms / 進行方向を向く V 字
            ctr = np.asarray(w.centroid.coords[0])
            nn = np.array([-fwd[1], fwd[0]])
            half = ln / 2 + 2.0
            chev = []
            s = -half
            while s <= half:
                apex = ctr + fwd * s
                arm = wd + 1.0
                v = LineString([tuple(apex - fwd * arm + nn * arm), tuple(apex), tuple(apex - fwd * arm - nn * arm)])
                chev.append(v.buffer(CHEV_W / 2, join_style="mitre", mitre_limit=3.0))
                s += CHEV_PITCH
            inner = w.buffer(-0.1)
            outline = w.difference(w.buffer(-0.15, join_style="mitre"))
            g = unary_union([unary_union(chev).intersection(inner), outline])
            self.mark("chevron", g, "w", None, roads=sorted(roads))

    # -- yellow box junctions (RMS2 N) / 黄色ボックス
    def yellow_boxes(self):
        lst = self.cfg.get("yellow_box", [])
        if not lst:
            return
        fr = geo.Frame(self.L.meta)
        jl = collections.defaultdict(list)
        for l in self.L.lanes:
            if l["junction"] >= 0 and len(l["pts"]) >= 2:
                high = float(np.mean([q.get("z", 0.0) for q in l["pts"]])) > Z0
                if high == (self.level == "deck"):
                    jl[l["junction"]].append(lane_poly(l))
        for item in lst:
            x, y = fr.xy(item["lon"], item["lat"])
            p = Point(x, y)
            best = min(jl.items(), key=lambda kv: unary_union(kv[1]).distance(p), default=None)
            if best is None or unary_union(best[1]).distance(p) > 25.0:
                self.count["yellow boxes not matched to a junction"] += 1
                continue
            area = unary_union(best[1]).convex_hull.intersection(self.cw)
            area = area.difference(self.bands.buffer(0.5)).buffer(-0.3, join_style="mitre")
            area = max(polys(area), key=lambda q: q.area, default=None)
            if area is None or area.area < 20.0:
                continue
            area = area.simplify(0.5)
            outline = area.difference(area.buffer(-BOX_EDGE, join_style="mitre"))
            mr = np.asarray(area.minimum_rotated_rectangle.exterior.coords)[:4]
            diag = unary_union([LineString([mr[0], mr[2]]), LineString([mr[1], mr[3]])]).buffer(BOX_DIAG / 2)
            self.mark("yellow_box", unary_union([outline, diag.intersection(area)]), "y", None, junction=best[0])

    def clip(self):
        cw = self.cw
        bands = self.bands.buffer(0.5) if not self.bands.is_empty else Polygon()
        cutters = {None: Polygon(), "band": bands,
                   "kerb": unary_union([bands] + self.zones["kerb"] + self.bay_zone),
                   "centre": unary_union([bands] + self.zones["centre"])}
        white, yellow = [], []
        for (col, cut), ps in self.paint.items():
            g = unary_union(ps)
            if cut is not None and not cutters[cut].is_empty:
                g = g.difference(cutters[cut])
            (white if col == "w" else yellow).append(g)
        W = unary_union(white).intersection(cw)
        Y = unary_union(yellow).intersection(cw).difference(W.buffer(0.02))
        self.out = [(q.simplify(0.01), "w") for q in polys(W) if q.area > 0.002] + \
                   [(q.simplify(0.01), "y") for q in polys(Y) if q.area > 0.002]
