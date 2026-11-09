"""Per-map corrections to the OpenStreetMap data (maps.json "osm_fixes").

Some OSM ways are drawn metres off the real road, or cross a building that
the zoom-20 imagery shows whole. They are corrected where the OSM file is
read, for the road logic (roadnet.py) and the 3-D model (osmdata.py) alike,
so both see the same streets. A map without "osm_fixes" is read unchanged.

  "osm_fixes": {
    "move_nodes": {"<node id>": [dx, dy], ...},     # metres east / north (map frame)
    "drop_ways": ["<way id>", ...],
    "split_ways": {"<way id>": ["<node id>", ...]}, # pieces: the way's own id, then <id>01, <id>02, ...
    "way_tags": {"<way id>": {"key": "value" or null}},   # null removes the tag
    "add_ways": [{"id": "<new way id>", "pts": [[x, y], ...], "tags": {...},
                  "closed": true}],           # closed: an area (ring back to the first point)
    "round_corners": {"radius": 6.0, "min_turn": 15.0}   # fillet sharp corners of drivable ways
  }

Applied in that order. Keys starting with "_" are notes. Measure against the
imagery (guide ch. 3, "Per-map OSM corrections").

round_corners: OSM draws a street corner as one vertex. netconvert turns it
into a tiny paramPoly3 (0.4-3.4 m, radius 0.8 m or a cusp) and offsets the
lanes 1.5-3 m from it, so the lane centres and edges fold back over
themselves (OitaKosen: 45 street lanes, 0.1-0.2 m steps in the drape, CARLA
waypoints zigzagging). Every vertex where exactly two drivable segments meet
(inside a way, or where one way continues another) and the way turns more
than min_turn degrees gets a circular arc of radius max(radius, the road's
width) through nodes every ~1.5 m; the tangent points stay within 45 % of a
leg shared with another rounded corner (90 % otherwise), the radius shrinks
if they must. The corner node itself moves onto the arc's middle, so ways
that share it (footways, crossings) stay connected.

道路の角（1 頂点）を円弧で丸める: netconvert の角の曲線は半径が車線のずれより小さく、
車線の中心・縁が折り返していた。

地図ごとの OSM 修正（maps.json "osm_fixes"）: ノード移動・ウェイ削除・分割・タグ・追加。
道路網（roadnet.py）と 3D モデル（osmdata.py）の両方で OSM を読む時に適用する。
"""

import json
import os
import xml.etree.ElementTree as ET

import geo

HERE = os.path.dirname(os.path.abspath(__file__))


def fixes_for(meta):
    """The "osm_fixes" of the maps.json entry whose name is meta["name"], or
    None. / 地図の osm_fixes（無ければ None）。"""
    try:
        cfg = json.load(open(os.path.join(HERE, "..", "maps.json")))
    except (OSError, ValueError):
        return None
    for v in cfg.values():
        if isinstance(v, dict) and v.get("name") == meta.get("name"):
            fx = v.get("osm_fixes") or None
            # the area features' own fixes (landmarks/akeno.py; AKENO_WIP=1: the
            # work-in-progress entry, scratch builds) / 地物の修正も加える
            af = v.get("_area_features_wip") if os.environ.get("AKENO_WIP") else v.get("area_features")
            wip = (af or {}).get("osm_fixes")
            if wip:
                fx = dict(fx or {})
                fx["drop_ways"] = list(fx.get("drop_ways", [])) + list(wip.get("drop_ways", []))
                fx["add_ways"] = list(fx.get("add_ways", [])) + list(wip.get("add_ways", []))
                for k in ("move_nodes", "way_tags"):
                    fx[k] = dict(fx.get(k, {}), **wip.get(k, {}))
            return fx
    return None


def _tag(way, k, v):
    for t in way.findall("tag"):
        if t.get("k") == k:
            if v is None:
                way.remove(t)
            else:
                t.set("v", str(v))
            return
    if v is not None:
        ET.SubElement(way, "tag", k=k, v=str(v))


def apply(root, meta, quiet=False):
    """Apply the map's OSM fixes to an OSM XML ``root`` in place. Returns the
    counts, or None when the map has none (the tree is then untouched).
    OSM の XML に修正を適用する（修正が無ければ何もしない）。"""
    fx = fixes_for(meta)
    if not fx:
        return None
    fr = geo.Frame(meta)
    n = {"moved": 0, "dropped": 0, "split": 0, "tagged": 0, "added": 0}
    nodes = {nd.get("id"): nd for nd in root.iter("node")}
    ways = {w.get("id"): w for w in root.iter("way")}
    for wid in fx.get("drop_ways", []):
        w = ways.pop(str(wid), None)
        if w is not None:
            root.remove(w)
            n["dropped"] += 1
    for nid, (dx, dy) in fx.get("move_nodes", {}).items():
        nd = nodes.get(str(nid))
        if nd is None:
            continue
        x, y = fr.xy(float(nd.get("lon")), float(nd.get("lat")))
        lon, lat = fr.lonlat(x + float(dx), y + float(dy))
        nd.set("lon", "%.9f" % lon)
        nd.set("lat", "%.9f" % lat)
        n["moved"] += 1
    for wid, at in fx.get("split_ways", {}).items():
        w = ways.get(str(wid))
        if w is None:
            continue
        at = {str(a) for a in at}
        nds = w.findall("nd")
        pieces, cur = [], []
        for i, nd in enumerate(nds):
            cur.append(nd.get("ref"))
            if nd.get("ref") in at and 0 < i < len(nds) - 1:
                pieces.append(cur)
                cur = [nd.get("ref")]
        pieces.append(cur)
        if len(pieces) < 2:
            continue
        tags = [(t.get("k"), t.get("v")) for t in w.findall("tag")]
        for nd in nds:
            w.remove(nd)
        for t in w.findall("tag"):
            w.remove(t)
        pos = list(root).index(w)
        for k, refs in enumerate(pieces):
            if k:
                pw = ET.Element("way", dict(w.attrib))
                pw.set("id", "%s%02d" % (wid, k))
                root.insert(pos + k, pw)
                ways[pw.get("id")] = pw
            else:
                pw = w
            for r in refs:
                ET.SubElement(pw, "nd", ref=r)
            for tk, tv in tags:
                ET.SubElement(pw, "tag", k=tk, v=tv)
        n["split"] += 1
    for wid, tags in fx.get("way_tags", {}).items():
        w = ways.get(str(wid))
        if w is None:
            continue
        for k, v in tags.items():
            _tag(w, k, v)
        n["tagged"] += 1
    kids = list(root)
    first_way = next((i for i, e in enumerate(kids) if e.tag == "way"), len(kids))
    for aw in fx.get("add_ways", []):
        wid = str(aw["id"])
        w = ET.Element("way", id=wid, version="1", visible="true")
        for k, (x, y) in enumerate(aw["pts"]):
            nid = "%s%02d" % (wid, k)
            lon, lat = fr.lonlat(float(x), float(y))
            root.insert(first_way, ET.Element("node", id=nid, version="1", visible="true",
                                              lat="%.9f" % lat, lon="%.9f" % lon))
            first_way += 1
            ET.SubElement(w, "nd", ref=nid)
        if aw.get("closed"):
            # an area (a building, a car park): the ring ends on its first node / 面: 最初のノードで閉じる
            ET.SubElement(w, "nd", ref="%s00" % wid)
        for k, v in aw.get("tags", {}).items():
            ET.SubElement(w, "tag", k=k, v=str(v))
        root.insert(first_way, w)
        ways[wid] = w
        n["added"] += 1
    n["rounded"] = n["merged"] = 0
    if fx.get("round_corners"):
        rc = fx["round_corners"]
        n["rounded"] = round_corners(root, fr, float(rc.get("radius", 6.0)), float(rc.get("min_turn", 15.0)))
        n["merged"] = getattr(round_corners, "merged", 0)
    if not quiet:
        print("osm fixes (maps.json osm_fixes): %(moved)d nodes moved, %(dropped)d ways dropped, "
              "%(split)d split, %(tagged)d retagged, %(added)d added, %(rounded)d corners rounded, "
              "%(merged)d merged into the junction beside them" % n)
    return n


# as roadnet.py: the ways netconvert drives / roadnet.py と同じ走行可能な道路
VEHICLE = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
           "secondary_link", "tertiary", "tertiary_link", "residential", "unclassified", "living_street",
           "service", "road"}
SKIP_SERVICE = {"parking_aisle", "drive-through", "emergency_access", "driveway"}
ARC_STEP = 1.5             # metres between the arc's nodes / 円弧のノード間隔
MERGE_LEG = 2.0            # a corner this near a junction node / way end is merged into it / これより近い角は統合
SHIFT_MAX = 2.5            # an arc leaves the OSM line by at most this (radius reduced) / 元の線からの最大ずれ
FOLD_MARGIN = 1.5          # ... but its radius stays >= half the road width + this / 半径の下限（半幅＋これ）


def _drivable(t):
    return t.get("highway") in VEHICLE and t.get("area") != "yes" and t.get("service") not in SKIP_SERVICE \
        and t.get("access") not in ("no", "private") and t.get("junction") not in ("roundabout", "circular")


def _width(t):
    try:
        if "width" in t:
            return float(t["width"].split()[0])
        if "lanes" in t:
            return 3.0 * int(float(t["lanes"].split(";")[0]))
    except ValueError:
        pass
    return 6.0




def _line_x(p, u, q, v):
    """Intersection of the lines p + s u and q + t v (None if parallel). / 2 直線の交点。"""
    den = u[0] * v[1] - u[1] * v[0]
    if abs(den) < 1e-9:
        return None
    s = ((q[0] - p[0]) * v[1] - (q[1] - p[1]) * v[0]) / den
    return (p[0] + s * u[0], p[1] + s * u[1])


def _avail(leg):
    """How much of a leg two arcs (or one, at a junction) may use: all but ARC_STEP,
    half when the leg is shorter than two steps. / 円弧が使える区間長"""
    return leg - ARC_STEP if leg >= 2 * ARC_STEP else 0.5 * leg


def _dist_poly(pt, poly):
    import math
    best = 1e9
    for a, b in zip(poly, poly[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((pt[0] - a[0]) * dx + (pt[1] - a[1]) * dy) / L2))
        best = min(best, math.hypot(pt[0] - a[0] - t * dx, pt[1] - a[1] - t * dy))
    return best


def round_corners(root, fr, radius=6.0, min_turn=15.0):
    """Fillet the sharp corners of drivable ways in place (see the module
    docstring). Consecutive corners turning the same way whose arcs would not
    fit on the leg between them become ONE arc over their combined turn,
    tangent to the outer legs (OSM draws a tight bend as 2-3 corners 1-5 m
    apart: arcs per corner stayed under the lanes' offset and still folded);
    corners turning opposite ways share their leg. Returns the number of
    corners rounded (round_corners.merged: corners merged into a junction,
    round_corners.shift: the largest move off the OSM line, m). New nodes get
    ids "77<first corner id><kkk>" (deterministic: roadnet.py and osmdata.py
    make the same ones). / 道路の角を円弧で丸める（同じ向きの近い角は 1 つの円弧に）。"""
    import math
    nodes = {nd.get("id"): nd for nd in root.iter("node")}
    xy = {k: fr.xy(float(v.get("lon")), float(v.get("lat"))) for k, v in nodes.items()}
    dways = []
    for w in root.iter("way"):
        t = {k.get("k"): k.get("v") for k in w.findall("tag")}
        if not _drivable(t):
            continue
        refs = [nd.get("ref") for nd in w.findall("nd")]
        if len(refs) < 2 or refs[0] == refs[-1] or any(r not in xy for r in refs):
            continue
        dways.append((w, t, refs))
    # incident drivable segments per node / ノードに接する走行可能な区間
    nb, tag_of = {}, {}
    for w, t, refs in dways:
        for i, r in enumerate(refs):
            for j in (i - 1, i + 1):
                if 0 <= j < len(refs):
                    nb.setdefault(r, []).append(refs[j])
            tag_of.setdefault(r, t)
    dist = lambda p, q: math.hypot(q[0] - p[0], q[1] - p[1])
    cand = {}
    for r, ns in nb.items():
        if len(ns) != 2 or ns[0] == ns[1]:
            continue
        a, p, c = xy[ns[0]], xy[r], xy[ns[1]]
        if dist(a, p) < 0.3 or dist(p, c) < 0.3:
            continue
        turn = (math.atan2(c[1] - p[1], c[0] - p[0]) - math.atan2(p[1] - a[1], p[0] - a[0]) + math.pi) \
            % (2 * math.pi) - math.pi
        if abs(math.degrees(turn)) < min_turn or abs(math.degrees(turn)) > 170.0:
            continue
        cand[r] = (ns[0], ns[1], turn)
    # a corner closer than MERGE_LEG to a junction node or a way end has no room for an
    # arc: it is merged into that node (dropped from the way; the street runs straight
    # into the junction, whose shape netconvert builds there anyway)
    # 交差点・端点に MERGE_LEG 未満の角は円弧が入らない: その節点に統合（ウェイから外す）
    merge = set()
    for r, (na, nc, turn) in cand.items():
        if (dist(xy[na], xy[r]) < MERGE_LEG and na not in cand) or (dist(xy[r], xy[nc]) < MERGE_LEG and nc not in cand):
            merge.add(r)
    for r in merge:
        del cand[r]
    Rt = {r: max(radius, min(12.0, _width(tag_of[r]))) for r in cand}
    Td = {r: Rt[r] * math.tan(abs(cand[r][2]) / 2) for r in cand}
    # clusters: same-way turns whose wanted arcs overlap on the leg between them / 角のまとまり
    par = {r: r for r in cand}

    def find(r):
        while par[r] != r:
            par[r] = par[par[r]]
            r = par[r]
        return r
    for r, (na, nc, turn) in cand.items():
        for o in (na, nc):
            if o in cand and (turn > 0) == (cand[o][2] > 0) and Td[r] + Td[o] > _avail(dist(xy[r], xy[o])):
                par[find(r)] = find(o)
    groups = {}
    for r in cand:
        groups.setdefault(find(r), []).append(r)
    clusters = []
    for g in groups.values():
        gs = set(g)
        ends = [r for r in g if any(o not in gs for o in cand[r][:2])]
        if not ends:
            continue                                         # a closed ring of corners / 角だけの環
        first = ends[0]
        prev = next(o for o in cand[first][:2] if o not in gs)
        order, cur = [], first
        while True:
            order.append(cur)
            na, nc, _ = cand[cur]
            nxt = nc if na == prev else na
            prev, cur = cur, nxt
            if cur not in gs or cur in order:
                break
        if len(order) != len(g):
            continue
        if len(order) > 1:
            a = next(o for o in cand[order[0]][:2] if o != order[1])
            c = next(o for o in cand[order[-1]][:2] if o != order[-2])
        else:
            a, c = cand[order[0]][0], cand[order[0]][1]
        P1, Pk = xy[order[0]], xy[order[-1]]
        u1 = ((P1[0] - xy[a][0]) / dist(xy[a], P1), (P1[1] - xy[a][1]) / dist(xy[a], P1))
        u2 = ((xy[c][0] - Pk[0]) / dist(Pk, xy[c]), (xy[c][1] - Pk[1]) / dist(Pk, xy[c]))
        theta = sum(cand[r][2] for r in order)
        if abs(math.degrees(theta)) > 175.0:
            continue                                         # a hairpin: left as drawn / ヘアピンはそのまま
        V = P1 if len(order) == 1 else _line_x(P1, u1, Pk, u2)
        if V is None:
            continue
        la = (V[0] - xy[a][0]) * u1[0] + (V[1] - xy[a][1]) * u1[1]
        lc = (xy[c][0] - V[0]) * u2[0] + (xy[c][1] - V[1]) * u2[1]
        if la <= 0.3 or lc <= 0.3:
            continue
        w_ = max(_width(tag_of[r]) for r in order)
        clusters.append({"order": order, "a": a, "c": c, "u1": u1, "u2": u2, "theta": theta, "V": V,
                         "la": la, "lc": lc, "Rt": max(Rt[r] for r in order), "Rneed": w_ / 2 + FOLD_MARGIN})

    def build(cl, R):
        """Arc of radius R tangent to the cluster's outer legs: its nodes (fraction, corner
        id or None), points, and the largest distance from the OSM line. / 円弧と元の線からのずれ"""
        th = abs(cl["theta"])
        T = R * math.tan(th / 2)
        u1, V = cl["u1"], cl["V"]
        A = (V[0] - u1[0] * T, V[1] - u1[1] * T)
        sgn = 1.0 if cl["theta"] > 0 else -1.0
        Cc = (A[0] - u1[1] * sgn * R, A[1] + u1[0] * sgn * R)          # centre: left / right of u1 / 中心
        a0 = math.atan2(A[1] - Cc[1], A[0] - Cc[0])
        # evenly spaced nodes (netconvert's corner curves overshot into cusps where a node
        # sat 0.4 m from the next and 1.5 m from the one before); the corner nodes take the
        # inner positions nearest their own turn, so shared ways stay connected
        # 等間隔のノード（間隔が不揃いだと netconvert の曲線が尖った）。角のノードは自分の
        # 曲がりに最も近い内側の位置へ（共有するウェイは繋がったまま）
        cum, fs = 0.0, []
        for r in cl["order"]:
            t_ = abs(cand[r][2])
            fs.append((cum + t_ / 2) / th)
            cum += t_
        m = max(2, int(math.ceil(R * th / ARC_STEP)), len(fs) + 1)
        while True:
            ks = [min(m - 1, max(1, int(round(f * m)))) for f in fs]
            if all(b > a for a, b in zip(ks, ks[1:])):
                break
            m += 1
        own = dict(zip(ks, cl["order"]))
        seq = [(i / m, own.get(i)) for i in range(m + 1)]
        pts = [(Cc[0] + R * math.cos(a0 + sgn * th * f), Cc[1] + R * math.sin(a0 + sgn * th * f)) for f, _ in seq]
        old = [xy[cl["a"]]] + [xy[r] for r in cl["order"]] + [xy[cl["c"]]]
        shift = max(max(_dist_poly(p, old) for p in pts), max(_dist_poly(xy[r], pts) for r in cl["order"]))
        return seq, pts, shift
    # the wanted radius: the target, smaller where the arc would leave the OSM line by
    # more than SHIFT_MAX, never below half the width + FOLD_MARGIN (the inner kerb keeps
    # a radius) / 目標半径（元の線から SHIFT_MAX 以内、内側の縁石が折れない半径以上）
    for cl in clusters:
        R = cl["Rt"]
        if build(cl, R)[2] > SHIFT_MAX:
            lo, hi = min(cl["Rneed"], R), R
            for _ in range(20):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if build(cl, mid)[2] <= SHIFT_MAX else (lo, mid)
            R = lo
        cl["Td"] = R * math.tan(abs(cl["theta"]) / 2)
    # each leg keeps a straight of at least ARC_STEP (half the leg if shorter than two):
    # shared with a neighbouring cluster in proportion to the wanted tangent lengths
    # (0.4 m left between two arcs: netconvert's curves overshot into a cusp)
    # 各区間に ARC_STEP 以上の直線を残し、隣の角とは按分（短い直線で netconvert の曲線が尖った）
    member = {r: k for k, cl in enumerate(clusters) for r in cl["order"]}
    plans = []
    for k, cl in enumerate(clusters):
        tan = math.tan(abs(cl["theta"]) / 2)
        for jfrac in (None, 0.9):
            # a leg to a junction node / way end may be used up to 90 % if the radius
            # would otherwise stay under the no-fold minimum (netconvert cuts the
            # junction's shape out there anyway) / 交差点側の区間は必要なら 90 % まで
            T = cl["Td"]
            for nbn, leg in ((cl["a"], cl["la"]), (cl["c"], cl["lc"])):
                o = member.get(nbn)
                if o is None:
                    T = min(T, _avail(leg) if jfrac is None else max(_avail(leg), jfrac * leg))
                else:
                    oc = clusters[o]
                    T = min(T, _avail(dist(cl["V"], oc["V"])) * cl["Td"] / (cl["Td"] + oc["Td"]))
            if T / tan >= cl["Rneed"] - 1e-6:
                break
        R = T / tan
        seq, pts, shift = build(cl, R)
        plans.append((cl, seq, pts, shift, R))
    if not plans and not merge:
        round_corners.merged, round_corners.shift = len(merge), 0.0
        return 0
    kids = list(root)
    first_way = next((i for i, e in enumerate(kids) if e.tag == "way"), len(kids))
    # a leg between two rounded corners gets the arc nodes of both: collected per leg,
    # then ordered along it (one overwrote the other: the line zigzagged back)
    # 2 つの角に挟まれた区間は両方の円弧ノードを集め、区間に沿って並べる
    legs, at_xy = {}, dict(xy)
    for cl, seq, pts, shift, R in plans:
        ids, kk = [], 0
        for (f, r), q in zip(seq, pts):
            lon, lat = fr.lonlat(*q)
            if r is not None:
                nodes[r].set("lon", "%.9f" % lon)
                nodes[r].set("lat", "%.9f" % lat)
                at_xy[r] = q
                ids.append(r)
                continue
            nid = "77%s%03d" % (cl["order"][0], kk)
            kk += 1
            root.insert(first_way, ET.Element("node", id=nid, version="1", visible="true",
                                              lat="%.9f" % lat, lon="%.9f" % lon))
            first_way += 1
            at_xy[nid] = q
            ids.append(nid)
        full = [cl["a"]] + ids + [cl["c"]]
        orig = [cl["a"]] + cl["order"] + [cl["c"]]
        pos = [full.index(r) for r in orig]
        for (u, i), (v, j) in zip(zip(orig, pos), zip(orig[1:], pos[1:])):
            legs.setdefault(frozenset((u, v)), set()).update(full[i + 1:j])
    fill = {}
    for key, ids in legs.items():
        u, v = tuple(key)
        pu, pv = at_xy[u], at_xy[v]
        d = (pv[0] - pu[0], pv[1] - pu[1])
        order = sorted(ids, key=lambda q: (at_xy[q][0] - pu[0]) * d[0] + (at_xy[q][1] - pu[1]) * d[1])
        fill[(u, v)] = order
        fill[(v, u)] = order[::-1]
    # every way rewritten in one pass from its ORIGINAL node list / 元のノード列から一度に書き直す
    for w, t, refs in dways:
        out = [refs[0]]
        for u, v in zip(refs, refs[1:]):
            out += fill.get((u, v), []) + [v]
        drop = {r for r in refs[1:-1] if r in merge}
        out = [r for r in out if r not in drop]
        if out == refs:
            continue
        for nd in w.findall("nd"):
            w.remove(nd)
        p0 = next((i for i, e in enumerate(list(w)) if e.tag == "tag"), len(list(w)))
        for k, r in enumerate(out):
            w.insert(p0 + k, ET.Element("nd", ref=r))
    round_corners.merged = len(merge)
    round_corners.shift = max([p[3] for p in plans] or [0.0])
    round_corners.shifts = sorted(((round(p[3], 2), p[0]["order"][0], round(p[4], 1)) for p in plans), reverse=True)
    return sum(len(p[0]["order"]) for p in plans)
