"""Grade separation on a FLAT map: flyovers and their ramps (carla-venv).

  carla-venv/bin/python grade_sep.py <Name.xodr> <meta.json> [--out <xodr>]

roadnet.py (maps.json "roadnet" layer_elevation) runs it after netconvert.
Every road stays at z = 0 except the decks of OSM bridges that pass over
another road (roadnet.separate_layers -> meta["grade_separation"]["decks"]:
the way's line and layer) and the ramps leading to them:

  1. every road is sampled every <= STEP m (the cross-section through its
     lane centres: keep-left one-way roads lie beside the OSM line, not on
     it); samples whose lanes reach within DECK_D m of a deck line, running
     along it, are deck samples at layer x layer_elevation m;
  2. a graph of the samples (consecutive samples, road-to-road links, and
     ONE node per junction: junctions stay level) carries the height down
     from the decks: z = max(0, max over decks (H - g x path length)), one
     Dijkstra from all decks at once, g = layer_max_grade;
  3. each road gets an <elevationProfile> of straight pieces through its
     samples (grade <= g on the shortest lane; cubics with level junction
     ends overshot to 1.5 g); junction roads the junction's height. Linked
     road ends share one node, so z is continuous across every link.
  4. crossings without a junction: where a raised road crosses another with
     less than layer_elevation between them, the upper road's samples there
     become sources too (the deck is the upper one), up to MAX_LEVELS layers,
     until nothing changes; what is still short is printed.
  5. deck tails: past DECK_TAIL m beyond the outermost road a deck crosses
     (not at an end shared with another deck way) the deck may come down;
     junction plateaus: the last PLATEAU m into a raised junction, and approach
     lanes lying on its lanes up to JTIE m lower, take the junction's height.

The roads netconvert's own --osm.layer-elevation produced were unusable
(every footway/tunnel layer counted, ramps through junctions, z -13 m,
117 % steps; guide ch. 40). Prints the deck count, max grade and the
crossings' clearance; writes <dir>/grade_sep.json.

平坦な地図の立体交差: 道路を跨ぐ橋だけを持ち上げ、勾配 g 以下の取付け部を付ける。
最後に跨ぐ道路の先の橋面は下りてよい。高い交差点の手前と、交差点の車線に重なる接続車線は交差点の高さ。
"""

import argparse
import heapq
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

STEP = 4.0           # sample spacing along a road (m) / 道路の標本間隔
DECK_D = 1.5         # a sample whose lanes reach this close to a deck line (and run along it) is on the deck / 橋の線からの距離
DECK_TAIL = 20.0     # a deck holds its height this far past the outermost road it crosses / 跨ぐ道路の先の橋面保持
JTIE = 1.5           # ... or whose lanes lie on its lanes, if at most this much lower / 交差点に重なる車線の高さ差
PLATEAU = 6.0        # roads into a raised junction are level this far before it / 交差点手前の水平区間
NEAR_LINK = 120.0    # merges: overlapping roads meeting within this far along the network / 合流の距離
MAX_LEVELS = 3       # crossing fixes raise a road to at most this many layers / 交差修正の最大層数
PARALLEL = 0.85      # |cos| of the heading difference for "along" / 平行とみなす角度


def lane_ids(sec):
    ids = []
    for side in ("left", "right"):
        for ln in sec.findall("%s/lane" % side):
            if ln.get("type") == "driving":
                ids.append(int(ln.get("id")))
    return ids


def linear(s, z):
    """[(s0, a, b, 0, 0)] per interval through (s, z), flat runs merged. / 区間ごとの 1 次式"""
    out = []
    for i in range(len(s) - 1):
        h = s[i + 1] - s[i]
        if h <= 1e-6:
            continue
        p = (s[i], z[i], (z[i + 1] - z[i]) / h, 0.0, 0.0)
        if out and abs(p[2]) < 1e-9 and abs(out[-1][2]) < 1e-9 and abs(p[1] - out[-1][1]) < 1e-6:
            continue
        out.append(p)
    return out or [(0.0, float(z[0]) if len(z) else 0.0, 0.0, 0.0, 0.0)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xodr")
    ap.add_argument("meta")
    ap.add_argument("--out")
    a = ap.parse_args()
    import carla
    from shapely.geometry import LineString, Point
    from shapely.strtree import STRtree

    meta = json.load(open(a.meta))
    H1 = float(meta["layer_elevation"])
    G = float(meta.get("layer_max_grade", 8.0)) / 100.0
    decks = meta.get("grade_separation", {}).get("decks", [])
    text = open(a.xodr).read()
    m = carla.Map("grade_sep", text)
    tree = ET.ElementTree(ET.fromstring(text))
    root = tree.getroot()
    roads = {r.get("id"): r for r in root.findall("road")}
    speed = {}
    for rid, r in roads.items():
        v = [float(sp.get("max")) for sp in r.iter("speed") if sp.get("max") not in (None, "no limit")]
        speed[rid] = max(v) if v else 0.0
    dl = [LineString(d["pts"]) for d in decks]
    dh = [H1 * int(d["layer"]) for d in decks]
    # an elevated arterial (bridge=viaduct) goes over another deck it runs along or across
    # (Tampines Viaduct over the TPE) / 高架道路（viaduct）は他の橋面の上
    dvia = [bool(d.get("viaduct")) for d in decks]
    # a deck lifts only roads of its own name (unnamed ones too): Upper Changi Road East and
    # an unnamed PIE-TPE link under the Tampines Viaduct's line took the deck height
    # 橋面は同じ名前の道路だけを持ち上げる（下や脇の別の道路は除く）
    dname = [d.get("osm_name", "") for d in decks]
    dtree = STRtree(dl) if dl else None

    # graph nodes: ("r", road, i) samples, ("j", junction); union of linked road ends
    # 節点: 道路の標本と交差点（1 つ）。つながる道路端は同じ節点
    parent = {}

    def find(k):
        while parent.get(k, k) != k:
            parent[k] = parent.get(parent[k], parent[k])
            k = parent[k]
        return k

    def union(p, q):
        p, q = find(p), find(q)
        if p != q:
            parent[max(p, q)] = min(p, q)

    samples = {}            # road -> (s array, deck height per sample or -1, junction)
    lanepts = {}            # road -> per sample {lane id: (x, y)} / 標本ごとの車線中心
    cent = {}               # road -> per sample cross-section centre (x, y) or None
    nd = 0
    dways = {}
    dts = {}
    for rid, r in roads.items():
        L = float(r.get("length"))
        jn = r.get("junction", "-1")
        n = max(1, int(math.ceil(L / STEP)))
        s = np.linspace(0.0, L, n + 1)
        secs = r.findall("lanes/laneSection")
        s0s = [float(x.get("s")) for x in secs]
        hdeck = np.full(len(s), -1.0)
        dway = np.full(len(s), -1)
        dtt = np.full(len(s), -1.0)
        lp, cc = [], []
        for i, si in enumerate(s):
            k = max(0, int(np.searchsorted(s0s, min(si, L - 1e-3), side="right")) - 1)
            pts, hd = [], None
            for lid in sorted(lane_ids(secs[k])):
                w = m.get_waypoint_xodr(int(rid), lid, float(min(max(si, 0.01), L - 0.01)))
                if w is not None:
                    pts.append((lid, w))
            lp.append({lid: (w.transform.location.x, -w.transform.location.y) for lid, w in pts})
            if not pts:
                cc.append(None)
                continue
            # the carriageway's cross-section (lane centres): netconvert's keep-left
            # one-way roads lie beside the OSM line, not on it / 車線中心を結ぶ横断線
            xy = list(lp[-1].values())
            p = LineString(xy) if len(xy) > 1 else Point(xy[0])
            cc.append((p.centroid.x, p.centroid.y))
            if dtree is None or jn != "-1":
                # junction roads are never decks themselves: a turn under a viaduct running
                # along it lifted the whole junction (its roads share one node); a junction
                # on a deck rises with the deck roads around it / 交差点内道路は橋面にしない
                continue
            yaw = math.radians(-pts[0][1].transform.rotation.yaw)
            reach = DECK_D + max(w.lane_width for _, w in pts) / 2
            rname = r.get("name", "") or ""
            for j in dtree.query(p.buffer(reach)):
                if dname[j] != rname:
                    continue                          # another road under / beside the deck / 別の道路
                ln = dl[j]
                if ln.distance(p) > reach:
                    continue
                t = ln.project(p.centroid)
                q0, q1 = ln.interpolate(max(0, t - 1)), ln.interpolate(min(ln.length, t + 1))
                dx, dy = q1.x - q0.x, q1.y - q0.y
                c = abs(dx * math.cos(yaw) + dy * math.sin(yaw)) / (math.hypot(dx, dy) or 1)
                if c > PARALLEL and dh[j] >= (hd or 0.0):
                    hd = dh[j]
                    dway[i] = j
                    dtt[i] = t
            if hd:
                hdeck[i] = hd
        samples[rid] = (s, hdeck, jn)
        dways[rid] = dway
        dts[rid] = dtt
        lanepts[rid], cent[rid] = lp, cc
    # deck tails: an OSM bridge often runs on far past the last road it crosses (the TPE
    # bridge ends 190 m past Upper Changi Road North), so its ramp came down on top of the
    # link beside it; past DECK_TAIL m beyond the outermost road crossed the deck is free
    # to come down (chained deck ways keep their shared ends) / 最後に跨ぐ道路の先は下りてよい
    dcross = [[] for _ in dl]
    for rid, r in roads.items():
        if samples[rid][2] != "-1":
            continue
        cl = [c for c in cent[rid] if c is not None]
        if len(cl) < 2:
            continue
        rl = LineString(cl)
        rname = r.get("name", "") or ""
        for j in (dtree.query(rl) if dtree is not None else []):
            if dname[j] == rname:
                continue
            x = dl[j].intersection(rl)
            for g in getattr(x, "geoms", [x]):
                if not g.is_empty and g.geom_type == "Point":
                    dcross[j].append(dl[j].project(g))
    ends = [(Point(l.coords[0]), Point(l.coords[-1])) for l in dl]
    dlim = []
    for j, l in enumerate(dl):
        if not dcross[j]:
            dlim.append((-1e9, 1e9))
            continue
        shared = [any(k != j and min(e.distance(ends[k][0]), e.distance(ends[k][1])) < 2.0 for k in range(len(dl)))
                  for e in ends[j]]
        dlim.append((-1e9 if shared[0] else min(dcross[j]) - DECK_TAIL,
                     1e9 if shared[1] else max(dcross[j]) + DECK_TAIL))
    n_tail = 0
    for rid, (s, hdeck, jn) in samples.items():
        for i in range(len(s)):
            j = dways[rid][i]
            if hdeck[i] > 0 and j >= 0 and not (dlim[j][0] <= dts[rid][i] <= dlim[j][1]):
                hdeck[i] = -1.0
                n_tail += 1
        nd += int((hdeck > 0).sum())
    # links / 接続
    for rid, r in roads.items():
        s, hdeck, jn = samples[rid]
        n = len(s)
        if jn != "-1":
            for i in range(n):
                union(("r", rid, i), ("j", jn))
        lk = r.find("link")
        for tag, i in (("predecessor", 0), ("successor", n - 1)):
            e = lk.find(tag) if lk is not None else None
            if e is None:
                continue
            if e.get("elementType") == "junction":
                union(("r", rid, i), ("j", e.get("elementId")))
            elif e.get("elementId") in samples:
                o = samples[e.get("elementId")]
                union(("r", rid, i), ("r", e.get("elementId"), 0 if e.get("contactPoint") == "start" else len(o[0]) - 1))
    adj = {}
    src = {}
    for rid, (s, hdeck, jn) in samples.items():
        lp = lanepts[rid]
        for i in range(len(s)):
            k = find(("r", rid, i))
            if hdeck[i] > 0:
                src[k] = max(src.get(k, 0.0), hdeck[i])
            if i and jn == "-1":
                p = find(("r", rid, i - 1))
                if p != k:
                    # the shortest lane path between the samples (inner lanes of a tight
                    # ramp are shorter than the reference line) / 最短の車線の長さ
                    common = set(lp[i]) & set(lp[i - 1])
                    d = min([math.hypot(lp[i][q][0] - lp[i - 1][q][0], lp[i][q][1] - lp[i - 1][q][1])
                             for q in common] or [float(s[i] - s[i - 1])])
                    d = max(d, 0.05)
                    adj.setdefault(p, []).append((k, d))
                    adj.setdefault(k, []).append((p, d))

    def solve(src, cap=None):
        """z per node: max(0, max over sources (h - g * path)), never above a node's
        cap (the lower road of a crossing, held while the upper one climbs over it;
        a source keeps its own height). Processed highest first.
        高さ: 源から勾配 g で下る最大値。交差の下側は上限で押さえる（源は除く）。"""
        cap = cap or {}
        # upper bounds spread from the caps at the same grade (a held node pulls its
        # neighbours down with it, no step) / 上限も勾配 g で広げる（段差を作らない）
        ub = {}
        pq = [(c, k) for k, c in cap.items() if k not in src]
        heapq.heapify(pq)
        while pq:
            c, k = heapq.heappop(pq)
            if k in ub:
                continue
            ub[k] = c
            if c > MAX_LEVELS * H1:
                continue
            for q, w in adj.get(k, ()):
                if q not in ub and q not in src:
                    heapq.heappush(pq, (c + G * w, q))
        z = {}
        pq = [(-h, k) for k, h in src.items()]
        heapq.heapify(pq)
        while pq:
            nh, k = heapq.heappop(pq)
            h = -nh
            if k in z or h <= 0:
                continue
            z[k] = h
            for q, w in adj.get(k, ()):
                if q in z:
                    continue
                hq = h - G * w
                if q in ub:
                    hq = min(hq, ub[q])
                if hq > 0:
                    heapq.heappush(pq, (-hq, q))
        return lambda k: z.get(k, 0.0)

    # crossings without a junction: the higher road must clear the lower by H1
    # (a ramp that reaches a road below before its deck, a road bridged on a
    # segment OSM did not tag): its samples there become sources too, 3 rounds
    # 交差点の無い交差: 上の道路は下より H1 高く（足りなければその標本も高さの源に）
    from shapely.strtree import STRtree as _T
    segs, meta_s = [], []
    for rid, cc in cent.items():
        jn = samples[rid][2]
        for i in range(len(cc) - 1):
            if cc[i] is not None and cc[i + 1] is not None and cc[i] != cc[i + 1]:
                segs.append(LineString([cc[i], cc[i + 1]]))
                meta_s.append((rid, i, jn))
    stree = _T(segs)
    pairs = []
    for a_, b_ in zip(*stree.query(segs, predicate="crosses")):
        if a_ >= b_:
            continue
        ra, ia, ja = meta_s[a_]
        rb, ib, jb = meta_s[b_]
        if ra == rb or (ja != "-1" and ja == jb):
            continue
        ends = {find(("r", ra, ia)), find(("r", ra, ia + 1))} & {find(("r", rb, ib)), find(("r", rb, ib + 1))}
        if ends:
            continue                                            # linked there / そこで接続
        pairs.append((ra, ia, rb, ib, segs[a_].intersection(segs[b_])))
    # merges: two roads of one direction lying on top of each other in plan that meet
    # within NEAR_LINK m along the network (a slip road's taper) are tied to one height
    # there -- the lower takes the higher's / 合流テーパーは同じ高さに（低い方を上げる）
    def near_link(a, b):
        seen, pq = {a: 0.0}, [(0.0, a)]
        while pq:
            dd, k = heapq.heappop(pq)
            if k == b:
                return True
            if dd > seen.get(k, 1e9):
                continue
            for q, w in adj.get(k, ()):
                if dd + w <= NEAR_LINK and dd + w < seen.get(q, 1e9):
                    seen[q] = dd + w
                    heapq.heappush(pq, (dd + w, q))
        return False
    z0 = solve(src)
    merges = []
    band = []
    for k, (rid, i, jn) in enumerate(meta_s):
        lp_ = list(lanepts[rid][i].values())
        if not lp_:
            continue
        hw = (max([math.hypot(a[0] - b[0], a[1] - b[1]) for a in lp_ for b in lp_] + [0.0]) + 2.4) / 2
        band.append((k, segs[k].buffer(max(hw - 0.5, 0.5), cap_style="flat")))
    raised = [(k, g) for k, g in band if max(z0(find(("r", meta_s[k][0], meta_s[k][1]))),
                                           z0(find(("r", meta_s[k][0], meta_s[k][1] + 1)))) > 0.05]
    btree = _T([g for _, g in band])
    for k, g in raised:
        ra, ia, ja = meta_s[k]
        va = np.diff(np.array(segs[k].coords), axis=0)[0]
        for j in btree.query(g, predicate="intersects"):
            kb = band[j][0]
            rb, ib, jb = meta_s[kb]
            if rb == ra or (ja != "-1" and ja == jb) or segs[k].crosses(segs[kb]):
                continue
            vb = np.diff(np.array(segs[kb].coords), axis=0)[0]
            if float(va @ vb) <= 0.5 * np.linalg.norm(va) * np.linalg.norm(vb):
                continue                                      # not one direction / 同方向でない
            if not (g.intersects(segs[kb]) or band[j][1].intersects(segs[k])):
                continue
            if near_link(find(("r", ra, ia)), find(("r", rb, ib))):
                merges.append((ra, ia, rb, ib))
    zof = solve(src)
    # level-junction plateaus: netconvert's junction lanes reach several metres over the
    # roads that enter them, so a road still on its ramp there lay 0.1-0.5 m under the
    # (level) junction lanes on top of it; the last PLATEAU m of every road into a raised
    # junction take the junction's height / 高い交差点の手前 PLATEAU m は交差点と同じ高さ
    def plateaus():
        n = 0
        for rid, r in roads.items():
            s_, hd_, jn_ = samples[rid]
            if jn_ != "-1":
                continue
            lk = r.find("link")
            for tag, i_end in (("predecessor", 0), ("successor", len(s_) - 1)):
                e = lk.find(tag) if lk is not None else None
                if e is None or e.get("elementType") != "junction":
                    continue
                zj = zof(find(("j", e.get("elementId"))))
                if zj <= 0.05:
                    continue
                for i in range(len(s_)):
                    if abs(s_[i] - s_[i_end]) <= PLATEAU:
                        k = find(("r", rid, i))
                        if src.get(k, 0.0) < zj - 0.01:
                            src[k] = zj
                            n += 1
        return n
    # junction footprints: an approach lane whose centre lies on a raised junction's lanes
    # (netconvert's junction shapes reach past the road ends) takes the junction's height
    # when it is only a little lower (a road passing under the junction is left alone)
    # 交差点の車線に重なる接続車線は、少しだけ低ければ交差点の高さに
    from shapely.ops import unary_union as _uu
    jparts = {}
    for k, g in band:
        if meta_s[k][2] != "-1":
            jparts.setdefault(meta_s[k][2], []).append(g)
    jids = list(jparts)
    jpolys = [_uu(jparts[j]) for j in jids]
    jtree = _T(jpolys) if jpolys else None

    def jties():
        n = 0
        if jtree is None:
            return 0
        for rid, (s_, hd_, jn_) in samples.items():
            if jn_ != "-1":
                continue
            for i in range(len(s_)):
                pts_ = [Point(q) for q in lanepts[rid][i].values()]
                if not pts_:
                    continue
                k = find(("r", rid, i))
                zr = zof(k)
                for q in pts_:
                    for jj in jtree.query(q, predicate="intersects"):
                        zj = zof(find(("j", jids[jj])))
                        if zj > 0.05 and 0.01 < zj - zr < JTIE and src.get(k, 0.0) < zj - 0.01:
                            src[k] = zj
                            zr = zj
                            n += 1
        return n
    n_plat = 0
    for _ in range(3):
        added_p = plateaus() + jties()
        n_plat += added_p
        if not added_p:
            break
        zof = solve(src)
    for rnd in range(6):
        added = 0
        short = []
        for ra, ia, rb, ib, x in pairs:
            def zat(rid, i):
                s_ = samples[rid][0]
                c0, c1 = cent[rid][i], cent[rid][i + 1]
                f = Point(c0).distance(x) / max(Point(c0).distance(Point(c1)), 1e-6)
                return (1 - f) * zof(find(("r", rid, i))) + f * zof(find(("r", rid, i + 1)))
            za, zb = zat(ra, ia), zat(rb, ib)
            if max(za, zb) < 0.3:
                continue                                         # both at grade: no bridge in OSM / 両方地上
            # the deck (OSM layer) is the upper road, else the higher one / 橋面が上、なければ高い方
            da = max(samples[ra][1][ia], samples[ra][1][ia + 1])
            db = max(samples[rb][1][ib], samples[rb][1][ib + 1])
            da += 0.1 if dways[ra][ia] >= 0 and dvia[dways[ra][ia]] else 0.0
            db += 0.1 if dways[rb][ib] >= 0 and dvia[dways[rb][ib]] else 0.0
            # else the faster road (an expressway passes over a street's junction), else the higher
            # なければ速い道路（高速道路が上）、それも同じなら高い方
            spa, spb = speed.get(ra, 0.0), speed.get(rb, 0.0)
            up_a = da > db if da != db else (spa > spb if abs(spa - spb) > 1.0 else za >= zb)
            (ru, iu, zu), (rl, il, zl) = ((ra, ia, za), (rb, ib, zb)) if up_a else ((rb, ib, zb), (ra, ia, za))
            if zu - zl >= H1 - 0.05:
                continue
            short.append((ru, rl, round(zu - zl, 2)))
            want = min(zl + H1, MAX_LEVELS * H1)          # at most MAX_LEVELS layers / 最大の層数
            for i in (iu, iu + 1):
                k = find(("r", ru, i))
                if src.get(k, 0.0) < want - 0.01:
                    src[k] = want
                    added += 1
        for ra, ia, rb, ib in merges:
            za = max(zof(find(("r", ra, ia))), zof(find(("r", ra, ia + 1))))
            zb = max(zof(find(("r", rb, ib))), zof(find(("r", rb, ib + 1))))
            if abs(za - zb) <= 0.1:
                continue
            rl, il, zu = (rb, ib, za) if za > zb else (ra, ia, zb)
            for i in (il, il + 1):
                k = find(("r", rl, i))
                if src.get(k, 0.0) < zu - 0.01:
                    src[k] = zu
                    added += 1
        if not added:
            break
        zof = solve(src)
    dbg = set(os.environ.get("GRADE_SEP_DEBUG", "").split(",")) - {""}
    for ra, ia, rb, ib in merges:
        if ra in dbg or rb in dbg:
            print("debug merge %s[%d] ~ %s[%d]" % (ra, ia, rb, ib))
    for rid in dbg:
        s_, hd_, jn_ = samples[rid]
        print("debug road %s junction %s: z %s deck %s" % (rid, jn_, [round(zof(find(("r", rid, i))), 1) for i in range(len(s_))],
                                                          [round(v, 1) for v in hd_]))
    left = []
    for ra, ia, rb, ib, x in pairs:
        za = 0.5 * (zof(find(("r", ra, ia))) + zof(find(("r", ra, ia + 1))))
        zb = 0.5 * (zof(find(("r", rb, ib))) + zof(find(("r", rb, ib + 1))))
        if max(za, zb) >= 0.3 and abs(za - zb) < H1 - 0.3:
            left.append((ra, rb, round(abs(za - zb), 1), [round(x.x), round(x.y)]))
    nhigh = sum(1 for v in src.values() if v > H1 + 0.01)
    print("grade separation: %d merge tapers tied to one height, %d junction-plateau samples" % (len(merges), n_plat))
    print("grade separation: %d crossings without a junction, %d raised to clear the road below "
          "(%d rounds), %d sources above one layer; %d still short of %.1f m: %s"
          % (len(pairs), len(set((a, b) for a, b, _ in short)), rnd, nhigh, len(left), H1, left[:8]))
    nraised, zmax, gmax = 0, 0.0, 0.0
    for rid, r in roads.items():
        s, hdeck, jn = samples[rid]
        z = np.array([zof(find(("r", rid, i))) for i in range(len(s))])
        if jn != "-1":
            z[:] = zof(find(("j", jn)))
            prof = [(0.0, float(z[0]), 0.0, 0.0, 0.0)]
        else:
            # straight pieces between the samples: the grade never exceeds g (a cubic
            # with level ends at junctions overshot to 1.5 g: 13.6 %); the kinks at a
            # ramp's foot and top are 4 m apart / 標本間は直線（3 次式は勾配が 1.5 倍に）
            prof = linear(list(s), list(z))
        if z.max() > 1e-6:
            nraised += 1
            zmax = max(zmax, float(z.max()))
        ep = r.find("elevationProfile")
        if ep is None:
            ep = ET.Element("elevationProfile")
            r.insert(list(r).index(r.find("planView")) + 1, ep)
        for el in list(ep):
            ep.remove(el)
        for s0, aa, b, c, d in prof:
            ET.SubElement(ep, "elevation", s="%.4f" % s0, a="%.4f" % aa, b="%.6f" % b, c="%.8f" % c,
                          d="%.10f" % d)
    tree.write(a.out or a.xodr, encoding="utf-8", xml_declaration=True)
    for k, lst in adj.items():                   # grade along the shortest lane / 最短の車線での勾配
        for q, d in lst:
            gmax = max(gmax, abs(zof(k) - zof(q)) / d)
    rep = {"short_crossings": left, "crossings": len(pairs), "decks": len(decks), "deck_samples": nd, "raised_roads": nraised, "z_max": round(zmax, 2),
           "sample_grade_max_pct": round(100 * gmax, 2), "layer_elevation": H1, "max_grade_pct": 100 * G}
    json.dump(rep, open(os.path.join(os.path.dirname(os.path.abspath(a.meta)), "grade_sep.json"), "w"), indent=1)
    print("grade separation: %d deck-tail samples freed past the last road crossed" % n_tail)
    print("grade separation: %d deck ways, %d deck samples, %d roads raised (z <= %.1f m), max grade %.1f %%"
          % (len(decks), nd, nraised, zmax, 100 * gmax))


if __name__ == "__main__":
    main()
