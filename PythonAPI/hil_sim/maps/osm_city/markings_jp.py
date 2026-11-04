"""Road markings, Japanese practice (maps.json "road_style": "japan"; maps/.venv).

Same principle as markings.py: every line follows the road logic's lane
edges, lines stop where junctions begin, all paint is unioned per colour.
What differs is the practice (Akeno, Oita: Street View and z20 satellite
in build/<Name>/road_photos, kosen_photos):

  road class    from the OSM way behind each xodr road (sumoId): MAIN (two
                or more lanes each way), TERT (tertiary and up, the ring),
                MINOR (residential / unclassified, lanes >= 2.95 m), NARROW
                (service lanes, lane_markings=no, narrower lanes)
  lane lines    white broken 5 m / 5 m, 0.15 m (lanes of one direction)
  centre line   MAIN without a median: two solid white lines (each direction
                paints its own, 0.6 m apart, as on the satellite south of the
                median); TERT with a 5.5 m+ carriageway: white broken 5/5
                where overtaking is allowed, solid YELLOW where it is banned
                (within 30 m of a junction or crossing, curves under 150 m
                radius, grades over 7 %); MINOR / NARROW: none
  edge lines    white solid 0.15 m along the kerb (MAIN, TERT, MINOR), and
                on the median side of divided roads
  stop lines    0.45 m white, 2 m behind a crossing: at signals (lights in
                the road logic or OSM traffic_signals at the junction), and
                with 止まれ where a lower-class road meets a higher one
                without signals or OSM has highway=stop (the xodr gets a
                stop sign there: signs.py)
  lane arrows   on approaches with two or more lanes, from the real
                junction connections, 5 m long, 7 m and 27 m before the line
  crossings     zebra: 0.45 m bars along the road, 0.45 m gaps, 4 m long,
                kerb to kerb; OSM crossings (untyped ones are marked in
                Japan) and road_features.json "crossings" read from imagery;
                diamonds (crossing ahead) 30 m and 50 m before unsignalised
                ones, in every approaching lane
  bus bays      yellow broken line along the bay mouth, a yellow box with
                バス停留所 in the bay

日本式の路面標示（車線端に沿う。色ごとに結合）。
"""

import collections
import math
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.affinity import rotate, scale, translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import substring, unary_union
from shapely.strtree import STRtree

import geo
import jartic
from markings import Markings, dashes, offset, polys, strip

MAIN, TERT, MINOR, NARROW = 4, 3, 2, 1
TERT_UP = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
           "secondary_link", "tertiary", "tertiary_link"}
W = 0.15                 # line width / 線幅
DASH, GAP = 5.0, 5.0
STOP_W = 0.45
STOP_BACK = 2.0
STOP_ROAD_MIN = 12.0     # a stop needs an approach road longer than this (room for line + sign) / 一時停止に必要な進入路長          # stop line this far before a crossing / 横断歩道の手前 2 m
NO_PASS = 30.0           # no overtaking this close to a junction or crossing / 追越し禁止の距離
NO_PASS_R, NO_PASS_GRADE = 150.0, 0.07
CENTRE_MIN = 5.5         # carriageway needed for a centre line / 中央線を引く最小幅員
SIGNAL_NEAR = 22.0
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


class MarkingsJP(Markings):
    def __init__(self, layout, xodr):
        self.L = layout
        self.out, self.giveway, self.stops = [], [], []
        self.cut_w, self.keep_w, self.cut_y, self.keep_y = [], [], [], []
        self.count = collections.Counter()
        self.lanes = [l for l in layout.lanes if l["junction"] < 0]
        self.lefthand = layout.meta.get("lefthand", False)
        self.build_index()
        self.classify(xodr)
        self.jtree = STRtree([lp for lp in self.junction_polys()])
        self.osm_signals = [p for p, t, nid in layout.osm.points(lambda t: t.get("highway") == "traffic_signals")]
        # OSM stop nodes: (point, node id, direction tag) / OSM の一時停止ノード
        self.osm_stops = [(p, nid, t.get("direction", "").lower())
                          for p, t, nid in layout.osm.points(lambda t: t.get("highway") == "stop")]
        self.stop_log = []
        self.dropped_stops = []
        self.no_stop_log = []
        # maps.json "jartic_d": police regulations (JARTIC type D) for stops and
        # crossings; None on every other map / 警察の交通規制データ（無ければ None）
        import os
        self.jcfg = jartic.config(layout.meta)
        self.jartic = jartic.load(layout.meta, os.path.dirname(os.path.abspath(xodr))) if self.jcfg else None
        self.jartic_log = []
        self.crossings = self.find_crossings()
        self.bands = unary_union([cr["band"] for cr in self.crossings]) if self.crossings else Polygon()
        self.lane_lines()
        self.junction_marks()
        self.crossing_marks()
        self.bus_marks()
        self.clip()

    def add(self, g, col="w", cut=True):
        """cut: a line that stops at crossings and bus bays. / 横断歩道で途切れる線。"""
        dst = (self.cut_w if cut else self.keep_w) if col == "w" else (self.cut_y if cut else self.keep_y)
        for p in polys(g):
            if p.area > 0.002:
                dst.append(p)

    # -- road classes / 道路の格
    def classify(self, xodr):
        rid_way = {}
        self.rid_rev = {}             # road runs against its OSM way (sumoId "-<way>") / OSM と逆向き
        self.rid_len, self.rid_pred = {}, {}   # xodr length, upstream junction / 長さ・上流の交差点
        for r in ET.parse(xodr).getroot().findall("road"):
            rid = int(r.get("id"))
            self.rid_len[rid] = float(r.get("length"))
            pr = r.find("link/predecessor")
            if pr is not None and pr.get("elementType") == "junction":
                self.rid_pred[rid] = int(pr.get("elementId"))
            u = r.find("userData[@code='sumoId']")
            if u is not None:
                v = u.get("value")
                rid_way[int(r.get("id"))] = v.lstrip("-").split("#")[0]
                self.rid_rev[int(r.get("id"))] = v.startswith("-")
        self.rid_way = rid_way
        ways = self.L.osm.ways
        self.cls = {}
        for key, lanes in self.sections.items():
            l = lanes[0]
            t = ways.get(rid_way.get(l["road"], ""), (None, {}))[1]
            hw = t.get("highway")
            w = float(np.median([q["w"] for q in l["pts"]]))
            try:
                nl = int(float((t.get("lanes") or "0").split(";")[0]))
            except ValueError:
                nl = 0
            if len(lanes) >= 2 or (hw in TERT_UP and nl >= 4):
                c = MAIN
            elif hw in TERT_UP or l["name"] == "Ring Road":
                c = TERT
            elif hw == "service" or t.get("lane_markings") == "no" or w < 2.95:
                c = NARROW
            else:
                c = MINOR
            self.cls[key] = c
        self.road_cls = {}
        for (road, sec), c in self.cls.items():
            self.road_cls[road] = max(self.road_cls.get(road, 0), c)
        n = collections.Counter(self.cls.values())
        self.count["sections MAIN/TERT/MINOR/NARROW"] = "%d/%d/%d/%d" % (n[MAIN], n[TERT], n[MINOR], n[NARROW])

    def junction_polys(self):
        out = []
        for l in self.L.lanes:
            if l["junction"] >= 0 and len(l["pts"]) >= 2:
                p = Polygon([q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]).buffer(0)
                if not p.is_empty:
                    out.append(p)
        return out or [Point(1e7, 1e7).buffer(1)]

    # -- lines along the lanes / 車線に沿う線
    def lane_lines(self):
        lh = self.lefthand
        for key, lanes in self.sections.items():
            lanes.sort(key=lambda l: abs(l["lane"]))
            c = self.cls[key]
            n = len(lanes)
            for i, l in enumerate(lanes):
                left = LineString([p["l"] for p in l["pts"]])
                right = LineString([p["r"] for p in l["pts"]])
                if left.length < 0.5:
                    continue
                kerb_lane = i == 0 if lh else i == n - 1
                centre_lane = i == n - 1 if lh else i == 0
                kerb_edge, kerb_out = (left, 1.0) if lh else (right, -1.0)
                centre_edge, centre_in = (right, 1.0) if lh else (left, -1.0)
                if i + 1 < n:
                    for d in dashes(right if lh else left, DASH, GAP):
                        self.add(strip(d, W))
                    self.count["lane line m"] += (right if lh else left).length
                if kerb_lane and c >= MINOR:
                    self.add(strip(offset(kerb_edge, kerb_out * 0.1), W))
                    self.count["edge line m"] += kerb_edge.length
                if not centre_lane:
                    continue
                opp = self.opposite_jp(centre_edge, l)
                if opp is None:
                    if c >= MINOR:
                        # median side of a divided road / one-way road's right side
                        # 分離帯側・一方通行の右側の外側線
                        self.add(strip(offset(centre_edge, -centre_in * 0.1), W))
                        self.count["edge line m"] += centre_edge.length
                    continue
                okey = (opp["road"], opp["section"])
                oc = self.cls.get(okey, c)
                if c == MAIN or oc == MAIN:
                    self.add(strip(offset(centre_edge, centre_in * 0.3), W))
                    self.count["centre double white m (per side)"] += centre_edge.length
                elif min(c, oc) >= TERT and l["road"] < opp["road"]:
                    width = float(np.median([q["w"] for q in l["pts"]])) + \
                        float(np.median([q["w"] for q in opp["pts"]]))
                    if width >= CENTRE_MIN:
                        self.centre_line(centre_edge, l)

    def opposite_jp(self, edge, l, tol=1.5):
        """The other direction alongside this lane's centre-side edge: the road
        whose edge stays within tol of it over most of its length (the
        mid-point test alone missed partners on curves and near junctions, and
        a two-way estate street got two 'one-way' lines down its middle).
        中央側の車線端に沿う対向車線（長さの大半で tol 以内）。"""
        pts = [edge.interpolate(t, normalized=True) for t in np.linspace(0.05, 0.95, 7)]
        best, bf = None, 0.0
        for i in self.tree.query(edge.buffer(tol)):
            e2, l2 = self.edges[i]
            if l2 is l or l2["road"] == l["road"]:
                continue
            f = sum(1 for p in pts if e2.distance(p) <= tol) / len(pts)
            if f > bf:
                best, bf = l2, f
        return best if bf >= 0.5 else None

    def centre_line(self, edge, l):
        """White broken where overtaking is allowed, yellow solid where not.
        追越し可は白破線、禁止区間は黄色実線。"""
        P = np.array([q["c"] for q in l["pts"]])
        Z = np.array([q["z"] for q in l["pts"]])
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
        ban = np.zeros(len(P), bool)
        pts = shapely.points(P)
        near_j = self.jtree.query(pts, predicate="dwithin", distance=NO_PASS)[0]
        ban[np.unique(near_j)] = True
        if not self.bands.is_empty:
            ban |= shapely.dwithin(self.bands, pts, NO_PASS)
        for k in range(len(P)):
            a = np.searchsorted(s, s[k] - 5.0)
            b = min(len(P) - 1, np.searchsorted(s, s[k] + 5.0))
            if b - a < 2 or s[b] - s[a] < 3.0:
                continue
            h0 = math.atan2(*(P[a + 1] - P[a])[::-1])
            h1 = math.atan2(*(P[b] - P[b - 1])[::-1])
            dh = abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
            if dh / (s[b] - s[a]) > 1.0 / NO_PASS_R or abs(Z[b] - Z[a]) / (s[b] - s[a]) > NO_PASS_GRADE:
                ban[k] = True
        # short allowed runs between banned ones are banned too / 短い追越し可区間は禁止に
        runs = []
        k = 0
        while k < len(P):
            j = k
            while j + 1 < len(P) and ban[j + 1] == ban[k]:
                j += 1
            runs.append([k, j, ban[k]])
            k = j + 1
        for r in runs:
            if not r[2] and s[r[1]] - s[r[0]] < 25.0 and len(runs) > 1:
                r[2] = True
        for k0, k1, b in runs:
            s0, s1 = edge.project(Point(P[k0])), edge.project(Point(P[k1]))
            if k1 + 1 < len(P):
                s1 = edge.project(Point(P[k1 + 1]))
            piece = substring(edge, s0, s1)
            if piece.length < 0.3:
                continue
            if b:
                self.add(strip(piece, W), "y")
                self.count["centre yellow m"] += piece.length
            else:
                for d in dashes(piece, DASH, GAP):
                    self.add(strip(d, W))
                self.count["centre white broken m"] += piece.length

    # -- junctions / 交差点
    def junction_marks(self):
        appr = collections.defaultdict(list)
        for a in self.L.approaches:
            appr[(a["road"], a["junction"])].append(a)
        lane_by = {(l["road"], l["lane"]): l for l in self.lanes}
        at_j = collections.defaultdict(set)
        for road, j in appr:
            at_j[j].add(road)
        sig_roads = set(self.signal_roads)
        # each OSM stop node belongs to ONE approach: on its own way, matching
        # direction, the nearest end (a way running through the junction has
        # an approach on both sides) / 一時停止ノードは最も近い 1 つの進入路のみ
        owner = {}
        for (road, j), al in appr.items():
            ls = [lane_by.get((road, a["lane"])) for a in al]
            ls = [l for l in ls if l is not None and len(l["pts"]) >= 3]
            if not ls:
                continue
            end = np.mean([l["pts"][-1]["c"] for l in ls], axis=0)
            for nid, dist in self.osm_stop_for(road, end):
                if nid not in owner or dist < owner[nid][0]:
                    owner[nid] = (dist, (road, j))
        own = {v[1]: "OSM node %s" % nid for nid, v in owner.items()}
        # stops read from imagery (road_features.json "stops": [{"x", "y", "why"}]):
        # the approach whose end is nearest the painted stop bar, within 10 m
        # 画像で確認した一時停止（停止線に最も近い進入路の終端、10 m 以内）
        for q in self.L.features.get("stops", []):
            best = None
            for (road, j), al in appr.items():
                ls = [lane_by.get((road, a["lane"])) for a in al]
                ls = [l for l in ls if l is not None and len(l["pts"]) >= 3]
                if not ls or road in sig_roads:
                    continue
                e = np.mean([l["pts"][-1]["c"] for l in ls], axis=0)
                dd = math.hypot(e[0] - q["x"], e[1] - q["y"])
                if dd < 10.0 and (best is None or dd < best[0]):
                    best = (dd, (road, j))
            if best is not None:
                own[best[1]] = "imagery (%s)" % q.get("where", "road_features stops")
        if self.jartic is not None:
            own.update(self.jartic_stops(appr, lane_by, sig_roads))
        # pass 1: what each approach gets / 各進入路の判定
        info = {}
        for (road, j), al in appr.items():
            byl = collections.OrderedDict()
            for a in al:
                byl.setdefault(a["lane"], set()).update(a["turns"])
            lanes = [(lane_by.get((road, ln)), turns) for ln, turns in byl.items()]
            lanes = [(l, t) for l, t in lanes if l is not None and len(l["pts"]) >= 3]
            if not lanes:
                continue
            end = np.mean([l["pts"][-1]["c"] for l, t in lanes], axis=0)
            # signal stop lines exactly where the road logic has lights / 信号は道路情報の信号機のみ
            info[(road, j)] = {"lanes": lanes, "end": end, "signal": road in sig_roads, "stop": False, "why": ""}
        # clusters: unsignalised junctions joined by links <= STOP_ROAD_MIN act as
        # one junction; a road ENTERING the cluster stops when its class is below
        # the highest class entering it (links inside get none); OSM stop nodes
        # stay on their own way / 交差点群: 群に入る格の低い道路に一時停止、群内の連結路は無し
        sig_j = {j for (r, j) in appr if r in sig_roads}
        parent = {j: j for (r, j) in appr}
        for r in self.rid_pred.values():
            parent.setdefault(r, r)

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        internal = set()
        for (r, j) in appr:            # all approaches: sub-metre links have no lane points / 全進入路
            j0 = self.rid_pred.get(r)
            if (self.rid_len.get(r, 99.0) <= STOP_ROAD_MIN and j0 in parent and j not in sig_j
                    and j0 not in sig_j):
                parent[find(j)] = find(j0)
                internal.add((r, j))
        entering = collections.defaultdict(list)
        for (r, j) in info:
            if (r, j) not in internal:
                entering[find(j)].append(r)
        self.clusters = {c: (sorted({j for j in parent if find(j) == c and any(jj == j for (_, jj) in appr)}),
                             sorted(set(rs)))
                         for c, rs in entering.items()}
        self.cluster_of = {j: find(j) for j in parent}
        self.internal = internal
        ns = self.L.features.get("no_stop", {})
        skip_ways = {str(w) for w in ns.get("ways", [])}
        # optional: only approaches ending within "radius" of "near" (one exit) / 特定の出口のみ
        near = ns.get("near")
        ends = {k: v["end"] for k, v in info.items()}
        for (road, j), v in info.items():
            osm_stop = own.get((road, j))
            c = find(j)
            ent = entering[c]
            top = max(self.road_cls.get(r, MINOR) for r in ent) if ent else 0
            mine = self.road_cls.get(road, MINOR)
            by_class = (road, j) not in internal and len(ent) >= 3 and mine < top
            if by_class and not v["signal"] and self.jcfg and osm_stop is None and self.public_road(road) \
                    and not self.ring_junction(j, appr):
                # JARTIC lacks this stop. The police list is incomplete (road 3756 has a
                # real 止まれ it misses), so the stop stays unless imagery shows it gone
                # ("no_stop" evidence) or "stops": "replace" / JARTIC に無い一時停止: 画像で無いと確認した物だけ外す
                ev = self.stop_evidence(v["end"])
                rec = {"road": road, "junction": j, "way": self.rid_way.get(road, "?"),
                       "x": round(float(v["end"][0]), 1), "y": round(float(v["end"][1]), 1)}
                if ev is not None or self.jcfg.get("stops") == "replace":
                    rec["what"] = "class stop removed (not in JARTIC; %s)" % (ev or "stops: replace")
                    self.count["class stops removed (evidence)"] += 1
                    by_class = False
                else:
                    rec["what"] = "class stop kept (not in JARTIC, no evidence it is gone)"
                    self.count["class stops kept, not in JARTIC"] += 1
                self.jartic_log.append(rec)
            here = near is None or math.hypot(ends[(road, j)][0] - near[0], ends[(road, j)][1] - near[1]) \
                <= ns.get("radius", 30.0)
            if self.rid_way.get(road) in skip_ways and here and osm_stop is None:
                if by_class:
                    self.no_stop_log.append((road, j, self.rid_way.get(road)))
                continue
            if not v["signal"] and (by_class or osm_stop is not None):
                v["stop"] = True
                v["why"] = osm_stop if osm_stop is not None else \
                    "class %d < %d entering cluster %s" % (mine, top, self.clusters[c][0])
        # pass 2: a stop on a short link (<= STOP_ROAD_MIN, inside a junction
        # cluster) moves upstream onto the same OSM way's road entering the
        # cluster, before its first junction; with none, it is dropped. Every
        # painted stop then has room for its stop sign (signs.py) on its road.
        # 短い連結路（交差点群の内部）の一時停止は、群に入る同じ道路へ移す（無ければ削除）
        for key in [k for k, v in info.items() if v["stop"]]:
            road, j = key
            if self.rid_len.get(road, 0.0) > STOP_ROAD_MIN:
                continue
            info[key]["stop"] = False
            cur, hops, dest = road, 0, None
            while hops < 4:
                j0 = self.rid_pred.get(cur)
                if j0 is None:
                    break
                here = [r for (r, jj) in info if jj == j0 and r != cur]
                ups = [r for r in here if self.rid_way.get(r) == self.rid_way.get(road)]
                # or, the way split at the cluster: the longest road of no higher
                # class entering it / ウェイが分かれている場合: 格が同じ以下の最長の流入路
                alt = [r for r in here if self.road_cls.get(r, MINOR) <= self.road_cls.get(road, MINOR)
                       and self.rid_len.get(r, 0.0) > STOP_ROAD_MIN]
                long_up = [r for r in ups if self.rid_len.get(r, 0.0) > STOP_ROAD_MIN]
                if long_up or (alt and not ups):
                    dest = (max(long_up or alt, key=lambda r: self.rid_len.get(r, 0.0)), j0)
                    break
                if not ups:
                    break
                cur, hops = max(ups, key=lambda r: self.rid_len.get(r, 0.0)), hops + 1
            if dest is None and hops:
                # end of the same-way chain: try the fallback there too / 連鎖の終端でも代替を試す
                j0 = self.rid_pred.get(cur)
                alt = [r for (r, jj) in info if jj == j0 and r != cur
                       and self.road_cls.get(r, MINOR) <= self.road_cls.get(road, MINOR)
                       and self.rid_len.get(r, 0.0) > STOP_ROAD_MIN] if j0 is not None else []
                if alt:
                    dest = (max(alt, key=lambda r: self.rid_len.get(r, 0.0)), j0)
            if dest is not None and not info[dest]["signal"]:
                if not info[dest]["stop"]:
                    info[dest]["stop"] = True
                    info[dest]["why"] = "%s, moved from road %d (%.1f m link)" % (
                        info[key]["why"], road, self.rid_len.get(road, 0.0))
                self.count["stops moved off short links"] += 1
            else:
                self.count["stops dropped (short link, no road outside)"] += 1
                self.dropped_stops.append((road, j, info[key]["why"]))
                if str(info[key]["why"]).startswith("JARTIC"):
                    self.jartic_log.append({
                        "what": "JARTIC stop dropped: its approach is a %.1f m link (<= %.0f m, no room for the "
                                "stop line and sign) and no longer road of the same way enters the cluster "
                                "upstream" % (self.rid_len.get(road, 0.0), STOP_ROAD_MIN),
                        "road": road, "junction": j, "why": info[key]["why"]})
        # pass 3: paint / 描画
        for (road, j), v in info.items():
            lanes, end, signal, stop = v["lanes"], v["end"], v["signal"], v["stop"]
            back = self.setback(end, lanes[0][0])
            for l, turns in lanes:
                P = np.array([q["c"] for q in l["pts"]])
                f = P[-1] - P[-3]
                f = f / (np.linalg.norm(f) or 1.0)
                nrm = np.array([-f[1], f[0]])
                yaw = math.atan2(f[1], f[0])
                w = l["pts"][-1]["w"]
                ln = LineString(P)
                if signal or stop:
                    c = P[-1] - f * (back + STOP_W / 2)
                    self.add(strip(LineString([c - nrm * (w / 2 + 0.05), c + nrm * (w / 2 + 0.05)]), STOP_W),
                             cut=False)
                    self.count["stop lines (signal)" if signal else "stop lines + 止まれ"] += 1
                if stop and ln.length > back + 10.0:
                    self.add(self.text("止まれ", P[-1] - f * (back + STOP_W + 1.5), yaw, 3.0, 1.5, 0.5), cut=False)
                if len(lanes) >= 2 and turns:
                    for dist in (7.0, 27.0):
                        if dist + back + 8.0 < ln.length:
                            p = ln.interpolate(ln.length - back - dist)
                            self.add(self.arrow_jp((p.x, p.y), yaw, turns), cut=False)
                            self.count["lane arrows"] += 1
            if stop:
                self.stops.append((road, j, back + STOP_W))
                self.stop_log.append({"junction": j, "road": road, "way": self.rid_way.get(road, "?"),
                                      "why": v["why"], "x": round(float(end[0]), 1), "y": round(float(end[1]), 1),
                                      "back": round(back + STOP_W, 2)})

    def speed_signs(self, speeds=(30, 40, 60, 70, 80, 90, 100), min_len=40.0):
        """Speed-limit signs, Japanese practice: only where a road's limit
        differs from the limit of the road a driver arrives from: for TERT and
        up the same street through the junction (same OSM way and direction),
        otherwise the highest-class road entering the junction. No repeats
        after junctions where the limit stays the same.
        [(road, km/h)] for signs.py. / 制限速度が変わる所だけに速度標識。"""
        kmh = {}
        for l in self.lanes:
            if l["speed"]:
                kmh[l["road"]] = max(kmh.get(l["road"], 0), int(round(l["speed"] * 3.6)))
        if self.jartic is not None:
            speeds = tuple(speeds) + (50,)        # CARLA has the 50 model; the JARTIC 50 line needs it / 50 も
            # JARTIC 最高速度 lines win; signs.py sets these roads' lane speed too
            # JARTIC の最高速度を優先（signs.py が車線の制限速度も合わせる）
            self.jartic_speed = {}
            for r in self.jartic:
                if r["code"] == jartic.SPEED and r["speed"] and r["kind"] == 2:
                    for road, lane in jartic.lanes_along(r["xy"], self.lanes):
                        self.jartic_speed[road] = r["speed"]
            for road, v in self.jartic_speed.items():
                if kmh.get(road) != v:
                    self.jartic_log.append({"what": "speed from JARTIC", "road": road, "was": kmh.get(road), "kmh": v})
                    self.count["roads given the JARTIC speed"] += 1
                kmh[road] = v
        into = collections.defaultdict(set)
        for a in self.L.approaches:
            into[a["junction"]].add(a["road"])
        out = []
        for road, v in kmh.items():
            j = self.rid_pred.get(road)
            if j is None or v not in speeds or self.rid_len.get(road, 0.0) <= min_len:
                continue
            way, rev = self.rid_way.get(road), self.rid_rev.get(road)
            cand = [r for r in into.get(j, ()) if r in kmh and not (self.rid_way.get(r) == way and
                                                                   self.rid_rev.get(r) != rev)]
            if not cand:
                continue
            same = [r for r in cand if self.rid_way.get(r) == way and self.rid_rev.get(r) == rev]
            if self.road_cls.get(road, MINOR) >= TERT and same:
                frm = same[0]
            else:
                frm = max(cand, key=lambda r: (self.road_cls.get(r, MINOR), self.rid_len.get(r, 0.0)))
            if kmh[frm] != v:
                out.append((road, v))
        return out

    def osm_stop_for(self, road, end):
        """OSM highway=stop nodes that may apply to this approach, with their
        distance (junction_marks keeps each node for its nearest approach only): the node lies
        on this road's own OSM way, within 15 m of the approach end, and its
        direction tag (forward / backward along the way) matches the road's
        travel direction. Distance alone also caught the through road at the
        same junction (all-way stops).
        この進入路に該当する一時停止ノード（同じ OSM ウェイ上、15 m 以内、向き一致）。"""
        way = self.rid_way.get(road)
        if way is None:
            return []
        refs = self.L.osm.ways.get(way, ([], {}))[0]
        out = []
        for p, nid, d in self.osm_stops:
            dist = p.distance(Point(*end))
            if nid not in refs or dist >= 15.0:
                continue
            rev = self.rid_rev.get(road, False)
            if (d == "forward" and rev) or (d == "backward" and not rev):
                continue
            out.append((nid, dist))
        return out

    # -- JARTIC type D (maps.json "jartic_d") / 警察の交通規制データ
    def jartic_stops(self, appr, lane_by, sig_roads):
        """{(road, junction): why} for every JARTIC 一時停止 point matched to an
        approach (its entry point and the stop). Stops on signalised approaches
        (flashing red at night) and points with no approach are logged only.
        JARTIC の一時停止を進入路へ（信号の進入路・該当なしは記録のみ）。"""
        own = {}
        for r in self.jartic:
            if r["code"] != jartic.STOP or r["kind"] != 1 or not self.in_ring(r["xy"][0]):
                continue
            P = r["xy"][0]
            for E in r["entry"] or [P]:
                k, score = jartic.match_approach(P, E, appr, lane_by)
                rec = {"what": "", "x": P[0], "y": P[1], "key": r["key"], "road": k[0] if k else None,
                       "junction": k[1] if k else None}
                if k is None:
                    rec["what"], c = "JARTIC stop: no approach in the road logic", "JARTIC stops without approach"
                elif k[0] in sig_roads:
                    rec["what"] = "JARTIC stop on a signalised approach (flashing red at night): no sign"
                    c = "JARTIC stops on signalised approaches"
                else:
                    own[k] = "JARTIC 一時停止 #%s" % jartic.short_key(r)
                    rec["what"], c = "JARTIC stop", "JARTIC stops matched"
                self.jartic_log.append(rec)
                self.count[c] += 1
        return own

    def stop_evidence(self, end, near=10.0):
        """The "no_stop" entry of the jartic_d settings ({x, y, why}: imagery shows
        no stop there) whose point is within ``near`` m of this approach end.
        画像で一時停止が無いと確認した地点（"no_stop"）。"""
        for q in (self.jcfg or {}).get("no_stop", []):
            if math.hypot(q["x"] - end[0], q["y"] - end[1]) <= near:
                return q.get("why", "imagery")
        return None

    def in_ring(self, p):
        """Inside the map's ring road (JARTIC points outside it have no road here). / 外周道路の内側か。"""
        if not hasattr(self, "_ring_in"):
            from shapely import wkt
            m = self.L.meta
            self._ring_in = wkt.loads(m["ring_wkt"]).buffer(-(m["ring_w"] / 2 + 1.0)) if "ring_wkt" in m else None
        return self._ring_in is None or self._ring_in.contains(Point(*p))

    def public_road(self, road):
        """A public road (police regulations apply): not service, not private.
        公道か（サービス道路・私道以外）。"""
        t = self.L.osm.ways.get(self.rid_way.get(road, ""), (None, {}))[1]
        return t.get("highway") in TERT_UP | {"residential", "unclassified", "living_street"} and \
            t.get("access") not in ("private", "no", "customers", "destination", "permissive")

    def ring_junction(self, j, appr):
        """A junction on the map's own ring road (no police data there). / 外周道路の交差点。"""
        if not hasattr(self, "_ring_j"):
            self._ring_j = {jj for (r, jj), al in appr.items()
                            if any("Ring Road" in a.get("exits", {}).values() for a in al)}
            self._ring_j |= {l["junction"] for l in self.L.lanes if l["junction"] >= 0 and l["name"] == "Ring Road"}
            ring_roads = {l["road"] for l in self.lanes if l["name"] == "Ring Road"}
            self._ring_j |= {jj for (r, jj) in appr if r in ring_roads}
        return j in self._ring_j

    def jartic_crossings(self, cands):
        """Crossing candidates from JARTIC 横断歩道 points: a point stands for
        横断歩道設置本数 crossings, at a junction on its legs nearest the point
        (2.5 m out from the junction), elsewhere at the point; crossings already
        found (OSM, imagery) within 30 m count toward the number.
        JARTIC の横断歩道（設置本数を交差点の脚に配置、既存分を差し引く）。"""
        jl = collections.defaultdict(list)
        for l in self.L.lanes:
            if l["junction"] >= 0:
                jl[l["junction"]] += [q["c"] for q in l["pts"]]
        jc = {j: (np.mean(v, axis=0), max(4.0, float(np.max(np.hypot(*(np.array(v) - np.mean(v, axis=0)).T)))))
              for j, v in jl.items()}
        have = [np.array([p.x, p.y]) for p, t, src in cands]
        out = []
        for r in self.jartic:
            if r["code"] != jartic.CROSSING or r["kind"] != 1 or not self.in_ring(r["xy"][0]):
                continue
            P = np.array(r["xy"][0])
            n = r["n_crossings"] or 1
            tags = {"crossing": "traffic_signals" if r["signal"] else "uncontrolled"}
            need = n - sum(1 for q in have if np.linalg.norm(q - P) < 30.0)
            rec = {"what": "JARTIC crossing", "x": float(P[0]), "y": float(P[1]), "n": n, "added": 0}
            if need <= 0:
                rec["what"] += ": all present"
                self.jartic_log.append(rec)
                continue
            near = sorted((float(np.linalg.norm(c - P)) - rad, j) for j, (c, rad) in jc.items())
            if not near or near[0][0] > 8.0:
                pts = [P] if n == 1 else []
            else:
                pts = [q for d, q in self.junction_legs(near[0][1], jc, P)]
            for q in pts:
                if need <= 0:
                    break
                if any(np.linalg.norm(q - h) < 6.0 for h in have):
                    continue
                out.append((Point(*q), tags, "JARTIC 横断歩道 #%s" % jartic.short_key(r)))
                have.append(q)
                need -= 1
                rec["added"] += 1
            if need > 0:
                rec["what"] += ": %d not placed (no free leg)" % need
            self.jartic_log.append(rec)
            self.count["JARTIC crossings added"] += rec["added"]
        return out

    def junction_legs(self, j, jc, P, out_m=2.5):
        """Crossing points on the legs of junction j (not the ring road), nearest
        P first: [(distance, point)]. / 交差点の各脚の横断歩道位置（P に近い順）。"""
        c, rad = jc[j]
        legs = []
        for l in self.lanes:
            if l["name"] == "Ring Road" or len(l["pts"]) < 3:
                continue
            C = np.array([q["c"] for q in l["pts"]])
            for mouth, nxt in ((C[-1], C[-3]), (C[0], C[2])):
                if np.linalg.norm(mouth - c) > rad + 3.0:
                    continue
                d = nxt - mouth
                d = d / (np.linalg.norm(d) or 1.0)       # outward along the leg / 脚の外向き
                legs.append((mouth, d))
        groups = []
        for mouth, d in legs:
            for g in groups:
                if float(g["d"] @ d) > 0.82 and np.linalg.norm(g["m"][0] - mouth) < 14.0:
                    g["m"].append(mouth)
                    break
            else:
                groups.append({"d": d, "m": [mouth]})
        out = []
        for g in groups:
            q = np.mean(g["m"], axis=0) + g["d"] * out_m
            out.append((float(np.linalg.norm(q - P)), q))
        return sorted(out, key=lambda t: t[0])

    def setback(self, end, lane):
        """Distance of the stop line from the lane end: 2 m behind a crossing
        at this arm. / 横断歩道があればその 2 m 手前。"""
        P = np.array([q["c"] for q in lane["pts"]])
        f = P[-1] - P[-3]
        f = f / (np.linalg.norm(f) or 1.0)
        best = 0.5
        for cr in self.crossings:
            v = np.array(cr["c"]) - end
            along = -float(v @ f)
            side = abs(float(v @ np.array([-f[1], f[0]])))
            if -3.0 < along < 15.0 and side < 15.0:
                best = max(best, along + cr["width"] / 2 + STOP_BACK)
        return best

    def arrow_jp(self, p, yaw, turns):
        """Japanese lane arrow, 5 m long, combined for the lane's turns. / 進行方向別通行区分の矢印。"""
        def head(tip, d):
            d = np.asarray(d, float) / np.linalg.norm(d)
            nn = np.array([-d[1], d[0]])
            ln = 1.3 if d[1] == 0 else 1.0
            b = np.asarray(tip) - d * ln
            return Polygon([tuple(b + nn * 0.4), tuple(tip), tuple(b - nn * 0.4)])
        parts = []
        if "straight" in turns or not ({"left", "right"} & set(turns)):
            parts += [LineString([(-2.5, 0), (1.25, 0)]).buffer(0.08, cap_style="flat"), head((2.5, 0), (1, 0))]
        for side in ("left", "right"):
            if side in turns:
                g = 1 if side == "left" else -1
                path = [(-2.5, 0), (-0.3, 0), (0.2, 0.18 * g), (0.45, 0.45 * g)]
                parts.append(LineString(path).buffer(0.08, cap_style="flat", join_style="mitre"))
                d = np.array([0.25, 0.27 * g])
                tip = np.array([0.45, 0.45 * g]) + d / np.linalg.norm(d) * 1.0
                parts.append(head(tip, d))
        return self.place(unary_union(parts), p, yaw)

    def text(self, s, p, yaw, length, width, gap):
        """Road text read by an approaching driver: each character `length`
        along the road, `width` across, its top toward travel; the first
        character furthest ahead (止 next to the stop line). p is where the
        text starts (its far end). / 路面文字（最初の字が最も前方）。"""
        out = []
        for k, ch in enumerate(s):
            g = glyph_jp(ch, length, width)
            if g is None:
                continue
            along = -(k * (length + gap) + length / 2)
            q = np.asarray(p, float) + np.array([math.cos(yaw), math.sin(yaw)]) * along
            out.append(self.place(rotate(g, -90, origin=(0, 0)), q, yaw))
        return unary_union(out) if out else None

    @staticmethod
    def diamond(p, yaw, length=3.0, width=1.5, line=0.2):
        d = Polygon([(-length / 2, 0), (0, width / 2), (length / 2, 0), (0, -width / 2)])
        ring = d.difference(d.buffer(-line, join_style="mitre"))
        return Markings.place(ring, p, yaw)

    # -- crossings / 横断歩道
    def find_crossings(self):
        L = self.L
        cw = L.carriageway
        fr = geo.Frame(L.meta)
        cands = []
        for ln, t in L.crossing_ways:
            if t.get("footway") != "crossing" or t.get("crossing") in ("no", "unmarked"):
                continue
            seg = ln.intersection(cw)
            if not seg.is_empty:
                cands.append((seg.centroid, t, "osm way"))
        for p, t, nid in L.osm.points(lambda t: t.get("highway") == "crossing" or
                                      (t.get("highway") == "traffic_signals" and "crossing" in t)):
            if t.get("crossing") in ("no", "unmarked", "informal") or t.get("crossing:markings") == "no":
                continue
            if p.distance(cw) < 0.5:
                cands.append((p, t, "osm node %s" % nid))
        for c in L.features.get("crossings", []):
            if "lat" not in c:
                continue
            x, y = fr.xy(c["lon"], c["lat"])
            cands.append((Point(x, y), {"crossing": "traffic_signals" if c.get("signal") else "uncontrolled"},
                          "imagery: %s" % c.get("where", "")))
        if self.jartic is not None:
            cands += self.jartic_crossings(cands)
        kept = []
        for p, t, src in cands:
            if all(p.distance(q) > 6.0 for q, _, _ in kept):
                kept.append((p, t, src))
        lane_lines = [(LineString([q["c"] for q in l["pts"]]), l) for l in self.lanes]
        tree = STRtree([g for g, _ in lane_lines])
        out = []
        for p, t, src in kept:
            i = tree.query_nearest(p, max_distance=12.0)
            if len(i) == 0:
                continue
            line = lane_lines[int(i[0])][0]
            s = line.project(p)
            a, b = line.interpolate(max(0, s - 1.0)), line.interpolate(min(line.length, s + 1.0))
            yaw = math.atan2(b.y - a.y, b.x - a.x)
            f = np.array([math.cos(yaw), math.sin(yaw)])
            n = np.array([-f[1], f[0]])
            c = np.array([p.x, p.y])
            g = LineString([c - n * 30.0, c + n * 30.0]).intersection(cw)
            segs = [q for q in getattr(g, "geoms", [g]) if q.geom_type == "LineString" and q.distance(p) < 10.0]
            if not segs or sum(q.length for q in segs) > 32.0:
                continue
            ts = [float((np.array(q) - c) @ n) for sg in segs for q in sg.coords]
            signal = t.get("crossing") == "traffic_signals" or t.get("crossing:signals") == "yes" or \
                t.get("highway") == "traffic_signals" or any(q.distance(p) < 20.0 for q in self.osm_signals)
            wdt = 4.0
            band = unary_union([sg.buffer(wdt / 2, cap_style="flat") for sg in segs]).intersection(cw)
            out.append({"c": c.tolist(), "f": f.tolist(), "n": n.tolist(), "t0": min(ts), "t1": max(ts),
                        "segs": segs, "width": wdt, "kind": "zebra_jp", "signal": signal, "yaw": yaw,
                        "band": band, "src": src})
        return out

    def crossing_marks(self):
        for cr in self.crossings:
            f, n, c = np.array(cr["f"]), np.array(cr["n"]), np.array(cr["c"])
            w = cr["width"]
            bars = []
            t = cr["t0"] + 0.25
            while t + 0.45 <= cr["t1"] - 0.2:
                q = c + n * (t + 0.225)
                bars.append(LineString([q - f * w / 2, q + f * w / 2]).buffer(0.225, cap_style="flat"))
                t += 0.9
            self.add(unary_union(bars).intersection(cr["band"]), cut=False)
            self.count["zebra crossings"] += 1
            if cr["signal"]:
                continue
            # diamonds in each lane approaching it, 30 m and 50 m before / 手前 30・50 m のひし形
            for lane in self.lanes:
                ln = LineString([q["c"] for q in lane["pts"]])
                if ln.distance(cr["band"]) > 0.5:
                    continue
                hit = ln.intersection(cr["band"])
                if hit.is_empty:
                    continue
                s0 = min(ln.project(Point(q)) for g in getattr(hit, "geoms", [hit]) for q in g.coords)
                for dist in (30.0, 50.0):
                    if s0 - dist < 2.0:
                        continue
                    a, b = ln.interpolate(s0 - dist - 1.0), ln.interpolate(s0 - dist + 1.0)
                    yaw = math.atan2(b.y - a.y, b.x - a.x)
                    m = ln.interpolate(s0 - dist)
                    self.add(self.diamond((m.x, m.y), yaw), cut=False)
                    self.count["diamonds"] += 1

    # -- bus bays / バス停車帯
    def bus_marks(self):
        from layout import BAY_DEPTH, BAY_TAPER, GUTTER
        for bay in getattr(self.L, "bus_bays", []):
            e = bay.get("edge")
            if e is None or e.length < 2 * BAY_TAPER + 4.0:
                continue
            for d in dashes(e, 1.5, 1.5):
                self.add(strip(d, W), "y", cut=False)
            # the box over the full-depth part, away from the lane / 奥行き一杯の部分に枠
            mid = substring(e, BAY_TAPER + 0.5, e.length - BAY_TAPER - 0.5)
            o1 = offset(mid, 0.35 if self.lefthand else -0.35)
            o2 = offset(mid, (GUTTER + BAY_DEPTH - 0.35) * (1 if self.lefthand else -1))
            if o1 is None or o2 is None:
                continue
            ring = Polygon(list(o1.coords) + list(o2.coords)[::-1]).buffer(0)
            self.add(ring.difference(ring.buffer(-W, join_style="mitre")), "y", cut=False)
            a, b = mid.interpolate(mid.length / 2 - 1.0), mid.interpolate(mid.length / 2 + 1.0)
            yaw = math.atan2(b.y - a.y, b.x - a.x)
            fwd = np.array([math.cos(yaw), math.sin(yaw)])
            ctr = np.array(ring.centroid.coords[0])
            txt = "バス停留所"
            ln_t = 1.9 * len(txt) + 0.25 * (len(txt) - 1)
            self.add(self.text(txt, ctr + fwd * ln_t / 2, yaw, 1.9, 1.4, 0.25), "y", cut=False)
            self.count["bus bays marked"] += 1
            self.bay_zone = getattr(self, "bay_zone", []) + [bay["poly"].buffer(0.3)]

    def clip(self):
        cw = self.L.carriageway
        cut = self.bands.buffer(0.5) if not self.bands.is_empty else Polygon()
        cut = unary_union([cut] + getattr(self, "bay_zone", []))
        white = unary_union([unary_union(self.cut_w).difference(cut), unary_union(self.keep_w)]).intersection(cw)
        yellow = unary_union([unary_union(self.cut_y).difference(self.bands.buffer(0.5) if not self.bands.is_empty
                                                                  else Polygon()),
                              unary_union(self.keep_y)]).intersection(cw).difference(white.buffer(0.02))
        self.out = [(q.simplify(0.01), "w") for q in polys(white) if q.area > 0.002] + \
                   [(q.simplify(0.01), "y") for q in polys(yellow) if q.area > 0.002]

    def report(self):
        return ", ".join("%s %s" % (k, ("%.0f" % v) if isinstance(v, float) else v)
                         for k, v in sorted(self.count.items()))


_GLY = {}


def glyph_jp(ch, length, width):
    """One character as polygons from Noto Sans CJK JP Bold, scaled by its em
    box to `width` across and `length` along: narrow kana keep their
    proportions instead of being blown up to the full box, and all characters
    of a word share one size (tall along the road, still legible).
    文字 1 字を多角形に（em 枠で拡大縮小、字形の比率を保つ）。"""
    key = (ch, length, width)
    if key in _GLY:
        return _GLY[key]
    from PIL import Image, ImageDraw, ImageFont
    try:
        font = ImageFont.truetype(FONT, 256, index=0)
    except OSError:
        _GLY[key] = None
        return None
    em, pad = 256, 32
    im = Image.new("L", (em + 2 * pad, em + 2 * pad), 0)
    ImageDraw.Draw(im).text((pad, pad), ch, fill=255, font=font)
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
    if not cells:
        _GLY[key] = None
        return None
    g = unary_union(cells).simplify(0.6)
    # across: centred on the ink; along: centred on the em box; scaled so a
    # kanji's ink (~0.92 em) fills the box / 横はインク中心、縦は em 中心
    minx, miny, maxx, maxy = g.bounds
    asc, desc = font.getmetrics()
    cy = h - pad - (asc + desc) / 2.0
    k = 1.0 / (0.92 * em)
    g = scale(translate(g, -(minx + maxx) / 2, -cy), width * k, length * k, origin=(0, 0))
    _GLY[key] = g
    return g
