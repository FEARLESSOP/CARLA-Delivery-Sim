"""Road logic (OpenDRIVE) for a city map built from an OpenStreetMap file.

  carla-venv/bin/python roadnet.py <in.osm> <out_dir> <Name> [--lefthand]

Steps:
  1. The map is the OSM file's bounding box, in metres east and north of its
     centre (the same frame as the 3-D model, see geo.py).
  2. A ring road runs round the inside of the box, and every street that
     leaves the map is cut where it meets the ring and joins it at a
     T-junction (like the Oita Station map): no dead ends at the edge, and
     the whole edge can be driven. Footways, steps and paths stop short of
     the ring.
  3. Roads under construction, parking aisles, drive-throughs and
     non-vehicle ways are left out of the drivable network (the 3-D model
     still draws footways, see layout.py).
  4. SUMO netconvert builds the network (keep-left with --lefthand: every
     direction its own road, ordinary negative lanes -- never stamp
     rule="LHT" on top, that reverses every lane), with the OSM turn-lane
     tags and traffic signals.
  5. fix_xodr_signals.py makes the signals proper CARLA traffic lights.
  6. road_fixes maps: turnloops.py replaces netconvert's dead-end
     turnarounds (drawn inside out) with forward bulb loops.
  7. maps.json "roadnet" (roadnet_opts, opt-in per map): expressways kept
     (they end into the ring), the ring on land only, and grade_sep.py
     raising the bridges that cross another road (flat maps with flyovers).

Writes <out_dir>/<Name>.xodr, <out_dir>/<Name>.ring.osm and
<out_dir>/meta.json (frame, bounds, ring).

OSM から都市マップ用の OpenDRIVE を作る。外周道路を追加し、地図外の道路を切る。
"""

import argparse
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from shapely.geometry import LineString, Point, box

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geo  # noqa: E402
import osmfix  # noqa: E402

CARLA_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
NETCONVERT = os.path.join(CARLA_ROOT, "carla-venv", "bin", "netconvert")
VENV_PY = os.path.join(CARLA_ROOT, "carla-venv", "bin", "python")

EXPRESSWAY_CLEAR = 25.0   # the ring stays this far (plus the inset) from an expressway / 高速道路からの距離
RING_INSET = 10.0          # ring centre line this far inside the OSM box / 外周道路の中心線の内側距離
RING_W, RING_RC = 7.0, 15.0
RING_DROP_LANES, RING_DROP_LEN = 1, 60.0   # ring_lane_drop: lanes / length before the ring's T / 外周手前の車線数と長さ
RING_TAGS = {"highway": "tertiary", "name": "Ring Road", "lanes": "2", "width": "7.0",
             "maxspeed": "40", "oneway": "no", "ring": "yes"}
VEHICLE = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
           "secondary_link", "tertiary", "tertiary_link", "residential", "unclassified", "living_street",
           "service", "road"}
SKIP_SERVICE = {"parking_aisle", "drive-through", "emergency_access", "driveway"}
LANE_W = {"residential": 3.0, "unclassified": 3.0, "living_street": 2.8}
ROAD_W = {"motorway": 14, "trunk": 14, "primary": 12, "secondary": 10, "tertiary": 8, "residential": 6.5,
          "unclassified": 6, "service": 5, "living_street": 5}


def ring_area(meta):
    """Inside of the ring road's centre line (meta["ring_wkt"] once made). / 外周道路の内側。"""
    from shapely import wkt
    return wkt.loads(meta["ring_wkt"])


def roadnet_opts(meta):
    """The "roadnet" options of the maps.json entry named meta["name"] ({} when
    none: every other map builds exactly as before). Keys:
      keep_expressways  true: motorways stay in the map (no EXPRESSWAY_CLEAR cut);
                        their carriageways end into the ring like any street
      ring_on_land      true: the ring keeps WATER_SETBACK m off the sea
                        (natural=coastline) and water bodies; inlets and canals
                        narrower than 2 x WATER_CLOSE are bridged (flat)
      layer_elevation   m per OSM layer for the bridges that cross a lower road
                        (separate_layers -> grade_sep.py); other bridges, tunnels
                        and every other road stay at z = 0
      layer_max_grade   % ramp grade down from those decks (grade_sep.py)
      detach_crossings  true: foot-only crossing nodes leave the drivable way
                        (no stray junctions; detach_crossings)
      halve_dual_lanes  false: road_fixes keeps the tagged lane count of each
                        one-way half (Singapore tags each half correctly)
      dual_lane_w       lane width (m) of the halves without a width tag
      join_crossings    true: same-layer drivable crossings without a shared node
                        get one (join_crossings)
      connect_lane_ends true: lane_ends.py after netconvert (dead lanes merged into
                        their neighbour's connection, one-way stubs to nowhere removed)
      fill_missing_lanes true: untagged main-road ways take their same-name
                        neighbours' lane count (fill_missing_lanes)
      no_loops_in_sites true: no dead-end loop (turnloops) within 40 m of a landmark
                        site (<site>_photos/index.json bounds; landmark_photo_sites)
      ring_lane_drop    true: a one-way road wider than RING_DROP_LANES that ends
                        at the ring (an expressway carriageway) drops to that many
                        lanes RING_DROP_LEN m before it (no dead lanes at the T)
    maps.json の "roadnet": 高速道路を残す・外周道路を陸上に・立体交差の高さ。"""
    try:
        cfg = json.load(open(os.path.join(HERE, "..", "maps.json")))
    except (OSError, ValueError):
        return {}
    for v in cfg.values():
        if isinstance(v, dict) and v.get("name") == meta.get("name"):
            return {k: x for k, x in (v.get("roadnet") or {}).items() if not k.startswith("_")}
    return {}


WATER_SETBACK = 25.0       # ring centre line this far from the sea / water (ring_on_land) / 水際からの距離
WATER_CLOSE = 45.0         # water narrower than 2x this (canals, river mouths) is bridged / これより狭い水路は渡る
RING_SMOOTH = 60.0         # the land ring's bends: radius at least this / 陸上外周道路の曲率半径


def _tags(e):
    return {k.get("k"): k.get("v") for k in e.findall("tag")}


def water_area(root, xy, region):
    """The sea (box side of every natural=coastline way: land on the left, sea
    on the right) and water bodies (natural=water, riverbanks, reservoirs;
    multipolygon outers) inside ``region``. / 海（海岸線の右側）と水面。"""
    from shapely.geometry import Polygon
    from shapely.ops import linemerge, polygonize, unary_union
    from shapely.validation import make_valid
    coast, wet = [], []
    ways = {w.get("id"): w for w in root.iter("way")}

    def pts(w):
        return [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]

    def is_water(t):
        return t.get("natural") == "water" or t.get("waterway") in ("riverbank", "dock") \
            or t.get("landuse") in ("reservoir", "basin")

    for w in ways.values():
        t = _tags(w)
        p = pts(w)
        if t.get("natural") == "coastline" and len(p) >= 2:
            coast.append(LineString(p))
        elif is_water(t) and len(p) >= 4 and p[0] == p[-1]:
            wet.append(make_valid(Polygon(p)))
    for r in root.iter("relation"):
        t = _tags(r)
        if t.get("type") == "multipolygon" and is_water(t):
            outer = [LineString(pts(ways[m.get("ref")])) for m in r.findall("member")
                     if m.get("type") == "way" and m.get("role") == "outer" and m.get("ref") in ways
                     and len(pts(ways[m.get("ref")])) >= 2]
            wet += [make_valid(p) for p in polygonize(linemerge(outer))]
    sea = []
    if coast:
        cl = linemerge(coast)
        cl = list(getattr(cl, "geoms", [cl]))
        noded = unary_union([region.exterior] + [g.intersection(region) for g in cl])
        for piece in polygonize(noded):
            c = piece.representative_point()
            best = min(cl, key=lambda g: g.distance(c))
            s = best.project(c)
            a, b = best.interpolate(max(0.0, s - 1.0)), best.interpolate(min(best.length, s + 1.0))
            q = best.interpolate(s)
            if (b.x - a.x) * (c.y - q.y) - (b.y - a.y) * (c.x - q.x) < 0:     # right of the coast = sea / 右が海
                sea.append(piece)
    water = unary_union(sea + [g.buffer(0) for g in wet if not g.is_empty])
    return water.intersection(region), len(coast), len(wet)


def make_ring_area(osm_path, meta):
    """The ring road follows the OSM box, and runs along the near side of an
    expressway that crosses the map instead of meeting it at grade (the
    expressway's slip roads join the ring like any street). Rounded corners.
    maps.json "roadnet": keep_expressways keeps the whole box (the expressways
    end into the ring), ring_on_land keeps the ring off the sea and water.
    外周道路は地図の枠に沿い、地図を横切る高速道路は平面交差させず手前を通す。"""
    from shapely.ops import linemerge, split, unary_union
    opts = roadnet_opts(meta)
    fr = geo.Frame(meta)
    root = ET.parse(osm_path).getroot()
    osmfix.apply(root, meta, quiet=True)          # maps.json "osm_fixes" / 地図ごとの OSM 修正
    xy = {n.get("id"): fr.xy(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    region = box(-meta["half_x"], -meta["half_y"], meta["half_x"], meta["half_y"])
    if opts.get("ring_on_land"):
        return _land_ring(root, xy, region, meta)
    mw = []
    for w in root.iter("way"):
        t = {k.get("k"): k.get("v") for k in w.findall("tag")}
        if t.get("highway") == "motorway":
            pts = [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]
            if len(pts) >= 2:
                mw.append(LineString(pts))
    if mw and not opts.get("keep_expressways"):
        band = unary_union(mw).buffer(EXPRESSWAY_CLEAR)
        rest = region.difference(band)
        parts = list(getattr(rest, "geoms", [rest]))
        centre = Point(0, 0)
        keep = max(parts, key=lambda g: (g.contains(centre), g.area))
        meta["expressway_cut"] = round(region.area - keep.area)
        region = keep
    inner = region.buffer(-RING_INSET, join_style="mitre").buffer(-RING_RC, join_style="mitre") \
        .buffer(RING_RC, quad_segs=6)
    if inner.geom_type != "Polygon":
        inner = max(inner.geoms, key=lambda g: g.area)
    return inner.simplify(0.05)


def _land_ring(root, xy, region, meta):
    """ring_on_land: the box inset by RING_INSET, minus the water (sea and water
    bodies; narrower than 2 x WATER_CLOSE bridged) grown by WATER_SETBACK,
    smoothed to bends of >= RING_SMOOTH m; the piece holding the centre.
    陸上の外周道路: 水面（狭い水路は渡る）から離し、曲率を緩くする。"""
    from shapely.geometry import Polygon
    water, n_coast, n_wet = water_area(root, xy, region)
    land = region.difference(water)
    # canals and river mouths narrower than 2 x WATER_CLOSE: closed (the ring bridges them flat)
    # 狭い水路・河口は閉じる（外周道路は平坦な橋で渡る）
    land = land.buffer(WATER_CLOSE, quad_segs=8).buffer(-WATER_CLOSE, quad_segs=8).intersection(region)
    wet = region.difference(land)
    inner = region.buffer(-RING_INSET, join_style="mitre")
    if not wet.is_empty:
        inner = inner.difference(wet.buffer(WATER_SETBACK, quad_segs=8))
    inner = inner.buffer(-RING_SMOOTH, quad_segs=8).buffer(RING_SMOOTH, quad_segs=8)    # convex bends / 凸の角
    inner = inner.buffer(RING_SMOOTH, quad_segs=8).buffer(-RING_SMOOTH, quad_segs=8)    # concave bends / 凹の角
    inner = inner.intersection(region.buffer(-RING_INSET, join_style="mitre"))
    inner = inner.buffer(-RING_RC, join_style="mitre").buffer(RING_RC, quad_segs=6)
    parts = list(getattr(inner, "geoms", [inner]))
    keep = max(parts, key=lambda g: (g.contains(Point(0, 0)), g.area))
    keep = Polygon(keep.exterior).simplify(0.5)
    meta["ring_water"] = {"coastline_ways": n_coast, "water_bodies": n_wet, "water_m2": round(water.area),
                          "land_kept_m2": round(keep.area), "box_m2": round(region.area)}
    print("ring on land: %d coastline ways, %d water bodies, water %.2f km2; ring encloses %.2f of %.2f km2"
          % (n_coast, n_wet, water.area / 1e6, keep.area / 1e6, region.area / 1e6))
    return keep


DUAL_LANE_W = 3.0          # lane width on the halves of a dual carriageway / 分離道路の車線幅


def fix_dual_carriageways(root, xy, halve=True, lane_w=DUAL_LANE_W):
    """Dual carriageways mapped as two one-way ways that each carry the WHOLE
    road's lane count (OitaKosen: the 4-lane road past Oita Kosen as two
    one-way ways 8 m apart, each lanes=4, built as 4 + 4 overlapping lanes).
    When the two halves of n lanes each could not fit side by side
    (n x DUAL_LANE_W > their spacing), each half gets n / 2. Halves without
    a width tag get lanes x DUAL_LANE_W. Tags are written into the XML.
    Returns the pairs found.
    分離道路の両側の一方通行路に道路全体の車線数が付いている場合に半分にする。"""
    one = []
    for w in root.findall("way"):
        t = {k.get("k"): k.get("v") for k in w.findall("tag")}
        if t.get("highway") not in VEHICLE or t.get("oneway") not in ("yes", "1") or t.get("area") == "yes":
            continue
        pts = [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]
        if len(pts) >= 2 and LineString(pts).length > 20:
            one.append((w, t, LineString(pts)))
    pairs = []
    for i, (wa, ta, la) in enumerate(one):
        for wb, tb, lb in one[i + 1:]:
            if la.distance(lb) > 25 or ta.get("highway") != tb.get("highway"):
                continue
            seps, n = [], int(la.length // 5) + 1
            for k in range(n):
                sa = min(la.length, k * 5.0)
                p = la.interpolate(sa)
                q = lb.interpolate(lb.project(p))
                d = p.distance(q)
                if d > 25:
                    continue
                pa, pb = la.interpolate(min(la.length, sa + 1)), la.interpolate(max(0, sa - 1))
                sb = lb.project(q)
                qa, qb = lb.interpolate(min(lb.length, sb + 1)), lb.interpolate(max(0, sb - 1))
                da = (pa.x - pb.x, pa.y - pb.y)
                db = (qa.x - qb.x, qa.y - qb.y)
                dot = (da[0] * db[0] + da[1] * db[1]) / ((math.hypot(*da) * math.hypot(*db)) or 1)
                if dot < -0.9 and 2.0 < d:
                    seps.append(d)
            if len(seps) >= max(4, 0.5 * min(la.length, lb.length) / 5.0):
                pairs.append((wa, ta, wb, tb, sorted(seps)[len(seps) // 2]))
    fixed = 0
    for wa, ta, wb, tb, sep in pairs:
        for w, t in ((wa, ta), (wb, tb)):
            try:
                n = int(float(t.get("lanes", "0").split(";")[0]))
            except ValueError:
                n = 0
            if halve and n >= 2 and n % 2 == 0 and n * lane_w > sep and not t.get("_halved"):
                t["_halved"] = True
                for el in w.findall("tag"):
                    if el.get("k") == "lanes":
                        el.set("v", str(n // 2))
                t["lanes"] = str(n // 2)
                fixed += 1
            if "width" not in t and "lanes" in t:
                wd = "%.2f" % (lane_w * int(float(t["lanes"].split(";")[0])))
                ET.SubElement(w, "tag", k="width", v=wd)
                t["width"] = wd
    print("dual carriageways: %d pairs of one-way halves (spacing %s m), %d halves had the whole road's lane count"
          % (len(pairs), [round(p[4], 1) for p in pairs], fixed))
    return len(pairs)


JP_SPEED = {"motorway": 100, "motorway_link": 60, "trunk": 60, "trunk_link": 40, "primary": 60,
            "primary_link": 40, "secondary": 50, "secondary_link": 40, "tertiary": 40, "tertiary_link": 40}


def japan_speeds(root):
    """Japan: roads without a maxspeed tag get a class limit (netconvert would
    give an untagged tertiary 80 km/h; the legal ordinary-road limit is 60 and
    the tagged neighbours are 40); residential/unclassified 30 where narrow
    (width < 5.5 m or < 3 m a lane), else 40 (netconvert: 50). Returns {class: count}.
    日本: 制限速度の無い道路に道路種別の既定値を与える。"""
    n = {}
    for w in root.findall("way"):
        t = {k.get("k"): k.get("v") for k in w.findall("tag")}
        if "maxspeed" in t:
            continue
        v = JP_SPEED.get(t.get("highway"))
        if v is None and t.get("highway") in ("residential", "unclassified"):
            # estate streets: 30 where narrow (width < 5.5 m or lanes < 3 m), else 40
            # 生活道路: 狭い道は 30、それ以外は 40
            try:
                wd = float(t["width"].split()[0]) if "width" in t else None
                nl = int(float(t["lanes"].split(";")[0])) if "lanes" in t else \
                    (1 if t.get("oneway") in ("yes", "1", "-1") else 2)
            except ValueError:
                wd, nl = None, 2
            v = 30 if wd is not None and (wd < 5.5 or wd / max(nl, 1) < 3.0) else 40
        if v is None:
            continue
        ET.SubElement(w, "tag", k="maxspeed", v=str(v))
        n[t["highway"]] = n.get(t["highway"], 0) + 1
    print("japan speed limits on untagged roads: %s" % n)
    return n


SNAP = 4.0                 # a dead end this close to another road meets it / 行き止まりをつなぐ距離


def drivable(t):
    return t.get("highway") in VEHICLE and t.get("area") != "yes" and t.get("service") not in SKIP_SERVICE \
        and t.get("access") not in ("no", "private")


def snap_near_misses(root, xy, new_node):
    """A street whose dead end stops within SNAP m of another street (mapped
    a little short, no shared node) is extended to meet it there: the new
    node is put on the other street's line, so netconvert makes a junction
    instead of two roads overlapping at different heights. Bridges, tunnels
    and layered ways are left alone. Returns the number joined.
    他の道路の手前 SNAP m 以内で終わる行き止まりを延ばして接続する。"""
    from collections import Counter
    ways = []
    for w in root.findall("way"):
        t = {k.get("k"): k.get("v") for k in w.findall("tag")}
        if drivable(t):
            refs = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in xy]
            if len(refs) >= 2:
                ways.append((w, t, refs))
    use = Counter(r for _, _, refs in ways for r in refs)
    flat = lambda t: not t.get("bridge") and not t.get("tunnel") and t.get("layer") in (None, "0")
    joined = 0
    for w, t, refs in ways:
        if not flat(t):
            continue
        for end, nxt in ((refs[0], refs[1]), (refs[-1], refs[-2])):
            if use[end] > 1:
                continue
            p = Point(xy[end])
            best = None
            for w2, t2, r2 in ways:
                if w2 is w or not flat(t2) or end in r2:
                    continue
                ln = LineString([xy[r] for r in r2])
                d = ln.distance(p)
                if d < SNAP and (best is None or d < best[0]):
                    best = (d, w2, r2, ln)
            if best is None or best[0] < 0.05:
                continue
            d, w2, r2, ln = best
            q = ln.interpolate(ln.project(p))
            # the extension must head away from the street's own last segment / 進行方向に延ばす
            a = xy[nxt]
            if (p.x - a[0]) * (q.x - p.x) + (p.y - a[1]) * (q.y - p.y) < -0.3 * d * math.hypot(p.x - a[0], p.y - a[1]):
                continue
            nid = new_node((q.x, q.y))
            sq = ln.project(q)
            acc, k_ins = 0.0, len(r2) - 1
            for k in range(len(r2) - 1):
                seg = math.hypot(xy[r2[k + 1]][0] - xy[r2[k]][0], xy[r2[k + 1]][1] - xy[r2[k]][1])
                if acc + seg >= sq - 1e-9:
                    k_ins = k + 1
                    break
                acc += seg
            nds2 = w2.findall("nd")
            ref_at = [nd.get("ref") for nd in nds2].index(r2[k_ins])
            w2.insert(list(w2).index(nds2[ref_at]), ET.Element("nd", ref=nid))
            r2.insert(k_ins, nid)
            nds = w.findall("nd")
            if end == refs[0]:
                w.insert(list(w).index(nds[0]), ET.Element("nd", ref=nid))
                refs.insert(0, nid)
            else:
                w.insert(list(w).index(nds[-1]) + 1, ET.Element("nd", ref=nid))
                refs.append(nid)
            use[nid] = 2
            joined += 1
    print("near misses: %d dead ends extended to meet the street beside them (< %.0f m)" % (joined, SNAP))
    return joined


def lines(g):
    return [q for q in getattr(g, "geoms", [g]) if q.geom_type == "LineString" and not q.is_empty]


def _layer(t):
    try:
        n = int(float((t.get("layer") or "0").split(";")[0]))
    except ValueError:
        n = 0
    if n == 0 and t.get("bridge") not in (None, "no"):
        n = 1
    return n


def separate_layers(root, xy):
    """maps.json "roadnet" layer_elevation (flat maps with flyovers): the decks
    for grade_sep.py. Only a layered vehicle way that really passes over
    another vehicle way (no shared node) is a deck, at its layer; a road over
    a tunnel (layer < 0) is a deck at layer 1 instead (a flat map has no
    underground). Bridges over canals only, car-park tunnels and footway
    layers stay at z = 0. The OSM tags are not changed (netconvert's own
    --osm.layer-elevation is not used). Crossings at the same layer without a
    shared node are counted (netconvert makes no junction there).
    立体交差: 他の道路を跨ぐ橋だけを grade_sep.py の橋面とする（タグは変えない）。"""
    from shapely.strtree import STRtree
    ways = []
    for w in root.findall("way"):
        t = _tags(w)
        if t.get("highway") not in VEHICLE or t.get("area") == "yes" or t.get("service") in SKIP_SERVICE \
                or t.get("access") in ("no", "private"):
            continue
        refs = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in xy]
        if len(refs) >= 2:
            ways.append((w, t, refs, LineString([xy[r] for r in refs]), _layer(t)))
    tree = STRtree([v[3] for v in ways])
    decks, flat, same = {}, 0, []
    for i, (w, t, refs, ln, lay) in enumerate(ways):
        cross = []
        for j in tree.query(ln, predicate="crosses"):
            w2, t2, r2, ln2, lay2 = ways[j]
            if j != i and not set(refs) & set(r2):
                cross.append((lay2, j))
        if lay > 0 and any(c[0] < lay for c in cross):
            decks[i] = max(decks.get(i, 0), lay)
        elif lay < 0 and any(c[0] > lay for c in cross):
            for lay2, j in cross:
                if lay2 > lay:
                    decks[j] = max(decks.get(j, 0), lay2 - lay)
        elif lay != 0:
            flat += 1
        elif lay == 0:
            same += [(w.get("id"), ways[j][0].get("id")) for lay2, j in cross
                     if lay2 == 0 and w.get("id") < ways[j][0].get("id")]
    out = [{"way": ways[i][0].get("id"), "name": ways[i][1].get("name", ways[i][1].get("highway")),
            "layer": lay, "viaduct": ways[i][1].get("bridge") == "viaduct", "osm_name": ways[i][1].get("name", ""),
            "pts": [[round(x, 2), round(y, 2)] for x, y in ways[i][3].coords]}
           for i, lay in sorted(decks.items())]
    print("grade separation: %d decks pass over a road, %d layered ways stay at z = 0 (cross nothing lower), "
          "%d crossings at the same layer without a junction %s"
          % (len(out), flat, len(same), same[:8]))
    return {"decks": out, "flat_layered": flat, "same_layer_crossings": same}


DETACH_SHIFT = 1.0         # detach_crossings: the foot ways' node copy this far north / 歩道側の複製の位置
def join_crossings(root, xy, node_for):
    """maps.json "roadnet" join_crossings: two drivable ways at the same layer that
    cross without a shared node (a mapping slip: Pasir Ris Farmway 1 over Pasir
    Ris Drive 12) get one at the crossing point, so netconvert makes a junction.
    Returns the number joined. / 同じ層で節点なく交差する道路に交点の節点を加える。"""
    from shapely.strtree import STRtree
    ways = []
    for w in root.findall("way"):
        t = _tags(w)
        if not drivable(t):
            continue
        refs = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in xy]
        if len(refs) >= 2:
            ways.append((w, refs, LineString([xy[r] for r in refs]), _layer(t)))
    tree = STRtree([v[2] for v in ways])
    joined = 0

    def insert(w, refs, p, nid):
        ln = LineString([xy[r] for r in refs])
        sp = ln.project(p)
        acc = 0.0
        for k in range(len(refs) - 1):
            seg = math.hypot(xy[refs[k + 1]][0] - xy[refs[k]][0], xy[refs[k + 1]][1] - xy[refs[k]][1])
            if acc + seg >= sp - 1e-9:
                nds = [nd for nd in w.findall("nd")]
                at = [nd.get("ref") for nd in nds].index(refs[k + 1])
                w.insert(list(w).index(nds[at]), ET.Element("nd", ref=nid))
                refs.insert(k + 1, nid)
                return
            acc += seg

    for i, (w, refs, ln, lay) in enumerate(ways):
        for j in tree.query(ln, predicate="crosses"):
            if j <= i:
                continue
            w2, r2, ln2, lay2 = ways[j]
            if lay2 != lay or set(refs) & set(r2):
                continue
            x = LineString([xy[r] for r in refs]).intersection(LineString([xy[r] for r in r2]))
            for p in getattr(x, "geoms", [x]):
                if p.geom_type != "Point":
                    continue
                nid = node_for((p.x, p.y))
                insert(w, refs, p, nid)
                insert(w2, r2, p, nid)
                joined += 1
    print("crossings joined: %d same-layer drivable crossings without a shared node got one" % joined)
    return joined


MAIN_ROADS = {"motorway", "trunk", "primary", "secondary", "tertiary"}


def fill_missing_lanes(root):
    """maps.json "roadnet" fill_missing_lanes: a main-road way without a lanes
    tag (netconvert would take its type's default, e.g. 1 lane on a 3-lane
    carriageway) takes the smallest lane count of the ways with the same name,
    class and one-way status that meet its ends. Written into the XML.
    Returns the number filled. / 車線数タグの無い主要道路に隣の同名道路の車線数を付ける。"""
    from collections import defaultdict
    ways, at_end = [], defaultdict(list)
    for w in root.findall("way"):
        t = _tags(w)
        if t.get("highway") in MAIN_ROADS and t.get("name"):
            refs = [nd.get("ref") for nd in w.findall("nd")]
            if len(refs) >= 2:
                ways.append((w, t, refs))
                at_end[refs[0]].append(len(ways) - 1)
                at_end[refs[-1]].append(len(ways) - 1)
    n = 0
    for k in range(3):                          # a gap of up to 3 untagged ways / 3 本まで続く欠落
        for w, t, refs in ways:
            if "lanes" in t:
                continue
            got = []
            for e in (refs[0], refs[-1]):
                for j in at_end[e]:
                    w2, t2, _ = ways[j]
                    if w2 is not w and t2.get("lanes") and t2.get("name") == t["name"] \
                            and t2.get("highway") == t["highway"] and t2.get("oneway") == t.get("oneway"):
                        try:
                            got.append(int(float(t2["lanes"].split(";")[0])))
                        except ValueError:
                            pass
            if got:
                t["lanes"] = str(min(got))
                ET.SubElement(w, "tag", k="lanes", v=t["lanes"])
                n += 1
    left = sum(1 for _, t, _ in ways if "lanes" not in t)
    print("lanes filled on %d untagged main-road ways from their neighbours (%d still untagged)" % (n, left))
    return n


NEAR_JUNCTION = 15.0       # detach_crossings: a signalised crossing this near a junction is its own / 交差点付属とみなす距離
FOOT = {"footway", "cycleway", "path", "steps", "pedestrian", "bridleway", "corridor", "track"}


def detach_crossings(root, fresh, xy):
    """maps.json "roadnet" detach_crossings: a node where only a footway (or
    cycleway, path, steps) crosses ONE drivable way, without signals, made
    netconvert keep a junction there; a few metres from a real junction it
    left a 0.2 m road between the two whose connector runs backwards (lane
    walks that never reach their exit: 33 runaway segments on
    PasirRisTampines). The foot ways get their own copy of the node (same
    place), so the drivable way passes straight through. Only the road logic's
    OSM (.ring.osm) changes; the model reads the original crossings.
    横断歩道だけが交わる節点を歩道側で複製し、netconvert の不要な交差点を作らない。"""
    from collections import defaultdict
    use = defaultdict(lambda: [0, 0, 0])           # [drivable ways, foot ways, interior on drivable]
    sig = set()
    for n in root.iter("node"):
        t = _tags(n)
        if t.get("highway") == "traffic_signals" or t.get("crossing") == "traffic_signals" \
                or t.get("crossing:signals") == "yes" or t.get("highway") in ("stop", "give_way"):
            sig.add(n.get("id"))
    foot = []
    for w in root.findall("way"):
        t = _tags(w)
        refs = [nd.get("ref") for nd in w.findall("nd")]
        if t.get("highway") in VEHICLE and t.get("area") != "yes":
            for k, r in enumerate(refs):
                use[r][0] += 1
                if 0 < k < len(refs) - 1:
                    use[r][2] += 1
        elif t.get("highway") in FOOT:
            foot.append(w)
            for r in set(refs):
                use[r][1] += 1
    n = 0
    copies = {}
    for w in foot:
        for nd in w.findall("nd"):
            r = nd.get("ref")
            u = use.get(r)
            if u and u[0] == 1 and u[2] == 1 and r not in sig and r in xy:
                if r not in copies:
                    # 1 m off: netconvert merges nodes at the same position again
                    # 1 m ずらす（同じ位置のノードは netconvert が再び一つにする）
                    copies[r] = fresh((xy[r][0], xy[r][1] + DETACH_SHIFT))
                    n += 1
                nd.set("ref", copies[r])
    # signalised crossings (highway=crossing + crossing=traffic_signals) within
    # NEAR_JUNCTION m of a real junction are that junction's own crossings: the
    # drivable way gets a plain copy of the node (netconvert kept a TLS junction
    # there, 4 m from the cluster, with a 0.2 m road between)
    # 交差点の近くの信号付き横断歩道: 車道側は印の無い複製ノードにする
    deg = defaultdict(int)
    dways = []
    for w in root.findall("way"):
        t = _tags(w)
        if t.get("highway") in VEHICLE and t.get("area") != "yes":
            refs = [nd.get("ref") for nd in w.findall("nd")]
            dways.append(w)
            for k, r in enumerate(refs):
                deg[r] += 1 if k in (0, len(refs) - 1) else 2
    junc = [xy[r] for r, d in deg.items() if d >= 3 and r in xy]
    from shapely.geometry import MultiPoint
    jgeo = MultiPoint(junc) if junc else None
    tagged = {}
    for nd_ in root.iter("node"):
        t = _tags(nd_)
        if t.get("highway") == "crossing" and nd_.get("id") in sig:
            tagged[nd_.get("id")] = nd_
    m = 0
    plain = {}
    for w in dways:
        for nd in w.findall("nd"):
            r = nd.get("ref")
            if r in tagged and deg.get(r) == 2 and r in xy and jgeo is not None \
                    and jgeo.distance(Point(xy[r])) < NEAR_JUNCTION:
                if r not in plain:
                    plain[r] = fresh(xy[r])
                    # the tagged node stays with the foot ways, 1 m off / 印付きノードは歩道側で 1 m ずらす
                    el = tagged[r]
                    el.set("lat", "%.9f" % (float(el.get("lat")) + DETACH_SHIFT / geo.KY))
                    xy[r] = (xy[r][0], xy[r][1] + DETACH_SHIFT)
                    m += 1
                nd.set("ref", plain[r])
    print("crossings: %d foot-only crossing nodes detached from the drivable way, %d signalised crossings "
          "beside a junction made plain on it (no junction there)" % (n, m))
    return n + m


def add_ring(src, dst, meta, fixes=False):
    """Cut the OSM ways at the ring and add the ring road. With fixes (maps.json
    "road_fixes"): the dual-carriageway lane fix and near-miss snapping
    (also onto the ring) first. / 外周道路を追加（fixes: 分離道路・行き止まりの修正）。"""
    tree = ET.parse(src)
    root = tree.getroot()
    fixed = osmfix.apply(root, meta)               # maps.json "osm_fixes" / 地図ごとの OSM 修正
    if fixed:
        meta["osm_fixes"] = fixed
    fr = geo.Frame(meta)
    xy = {n.get("id"): fr.xy(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    at = {v: k for k, v in xy.items()}
    area = ring_area(meta)
    edge = area.exterior
    short = area.buffer(-(RING_W / 2 + 4.0))     # paths stop before the ring's pavement / 歩道等は手前で止める
    # new ids below every id already in the file (negative ids from an earlier
    # step, e.g. topo_buildings.py, collided and merged unrelated nodes)
    # 既存の ID より小さい番号を使う（衝突すると無関係なノードが一つになる）
    used = [int(e.get("id")) for e in root if e.get("id") and e.get("id").lstrip("-").isdigit()]
    nid = [min([0] + used) - 1]
    new_nodes = []

    def node_for(c):
        c = (c[0], c[1])
        if c not in at:
            at[c] = str(nid[0])
            nid[0] -= 1
            xy[at[c]] = c
            new_nodes.append(at[c])
        return at[c]

    if meta.get("road_style") == "japan":
        meta["speed_defaults"] = japan_speeds(root)
    if fixes:
        # maps.json "roadnet" halve_dual_lanes false: the halves carry their own lane
        # counts (Singapore), dual_lane_w their lane width / 片側ごとの車線数が正しい地図
        ro = meta.get("roadnet") or {}
        meta["dual_carriageways"] = fix_dual_carriageways(root, xy, halve=ro.get("halve_dual_lanes", True),
                                                          lane_w=float(ro.get("dual_lane_w", DUAL_LANE_W)))
        meta["near_misses"] = snap_near_misses(root, xy, node_for)
    if meta.get("layer_elevation"):
        meta["grade_separation"] = separate_layers(root, xy)
    if (meta.get("roadnet") or {}).get("join_crossings"):
        meta["crossings_joined"] = join_crossings(root, xy, node_for)
    if (meta.get("roadnet") or {}).get("fill_missing_lanes"):
        meta["lanes_filled"] = fill_missing_lanes(root)
    if (meta.get("roadnet") or {}).get("detach_crossings"):
        def fresh(c):
            k = str(nid[0])
            nid[0] -= 1
            xy[k] = c
            new_nodes.append(k)
            return k
        meta["crossings_detached"] = detach_crossings(root, fresh, xy)
    from collections import Counter
    use = Counter(nd.get("ref") for w in root.findall("way") for nd in w.findall("nd")
                  if drivable({t.get("k"): t.get("v") for t in w.findall("tag")}))
    ends, kept, cut, dropped = [], 0, 0, 0
    ring_snaps = [0]
    ring_drop = bool((meta.get("roadnet") or {}).get("ring_lane_drop"))
    ring_drops = [0]
    for way in list(root.findall("way")):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        if "highway" not in tags and "railway" not in tags:
            continue
        if tags.get("highway") == "construction" or tags.get("railway") == "construction":
            root.remove(way)
            continue
        if tags.get("highway") == "service":
            if tags.get("service") in SKIP_SERVICE or tags.get("access") in ("no", "private") \
                    or tags.get("tunnel") == "building_passage":
                tags["highway"] = "service_skip"           # not drivable for the network / 走行網から除外
            else:
                # estate access roads: netconvert's own types keep cars off
                # service roads, so pass them as slow minor streets
                # 団地の進入路: netconvert の型では乗用車不可のため低速の生活道路として渡す
                tags["highway"] = "residential"
                if "maxspeed" not in tags:
                    tags["maxspeed"] = "30"
                    ET.SubElement(way, "tag", k="maxspeed", v="30")
                tags["_lane_w"] = 2.8
            for t in way.findall("tag"):
                if t.get("k") == "highway":
                    t.set("v", tags["highway"])
        # realistic carriageway widths: estate roads are narrow (netconvert
        # would give every lane 3.4 m) / 団地道路は狭い（既定では全車線 3.4 m）
        lane_w = tags.pop("_lane_w", LANE_W.get(tags.get("highway")))
        if lane_w and "width" not in tags and "highway" in tags:
            if tags.get("oneway") in ("yes", "1", "-1"):
                n = int(float(tags.get("lanes", "1").split(";")[0]))
            else:
                n = int(float(tags.get("lanes", "2").split(";")[0]))
            ET.SubElement(way, "tag", k="width", v="%.2f" % (lane_w * max(n, 1)))
        refs = [nd.get("ref") for nd in way.findall("nd") if nd.get("ref") in xy]
        if len(refs) < 2:
            continue
        line = LineString([xy[r] for r in refs])
        drive = tags.get("highway") in VEHICLE and tags.get("area") != "yes"
        w = ROAD_W.get(tags.get("highway"), 6.0)
        clip = area if drive else short
        if line.within(clip) and (not drive or line.distance(edge) > RING_W / 2 + w / 2 + 0.5):
            kept += 1
            continue
        pieces = []
        for g in lines(line.intersection(clip)):
            if g.length < 0.5:
                continue
            if drive:
                along = g.intersection(edge.buffer(RING_W / 2 + w / 2 + 0.5)).length
                if along > 15.0 and along > 0.5 * g.length:
                    continue                      # runs on top of the ring / 外周道路と重なる
            pieces.append(g)
        root.remove(way)
        if not pieces:
            dropped += 1
            continue
        cut += 1
        for k, g in enumerate(pieces):
            if drive and fixes:
                # a dead end that stops just short of the ring is extended onto it
                # 外周道路の手前で終わる行き止まりは外周道路まで延ばす
                cs = list(g.coords)
                for i_end in (0, -1):
                    c = cs[i_end]
                    dd = edge.distance(Point(c))
                    if 0.01 <= dd < RING_W / 2 + w / 2 + 0.5 and use.get(at.get(tuple(c)), 2) <= 1:
                        q = edge.interpolate(edge.project(Point(c)))
                        if i_end == 0:
                            cs.insert(0, (q.x, q.y))
                        else:
                            cs.append((q.x, q.y))
                        ring_snaps[0] += 1
                g = LineString(cs)
            nw = ET.SubElement(root, "way", dict(way.attrib))
            if k:
                nw.set("id", str(nid[0]))
                nid[0] -= 1
            drop = None
            if drive and ring_drop and tags.get("oneway") in ("yes", "1", "-1"):
                # maps.json "roadnet" ring_lane_drop: a wide one-way road (an expressway
                # carriageway) that ends at the ring drops to RING_DROP_LANES lanes
                # RING_DROP_LEN m before it, so no lane runs into the T without a
                # connection / 外周道路に当たる多車線の一方通行路は手前で車線を減らす
                try:
                    nl = int(float(tags.get("lanes", "1").split(";")[0]))
                except ValueError:
                    nl = 1
                e0, e1 = edge.distance(Point(g.coords[0])) < 0.01, edge.distance(Point(g.coords[-1])) < 0.01
                if nl > RING_DROP_LANES and (e0 != e1) and g.length > 2 * RING_DROP_LEN:
                    from shapely.ops import substring
                    if e1:
                        main, drop = substring(g, 0, g.length - RING_DROP_LEN), substring(g, g.length - RING_DROP_LEN, g.length)
                    else:
                        drop, main = substring(g, 0, RING_DROP_LEN), substring(g, RING_DROP_LEN, g.length)
                    g = main
                    ring_drops[0] += 1
            for c in g.coords:
                ET.SubElement(nw, "nd", ref=node_for(c))
            if drop is not None:
                dw = ET.SubElement(root, "way", dict(way.attrib))
                dw.set("id", str(nid[0]))
                nid[0] -= 1
                for c in drop.coords:
                    ET.SubElement(dw, "nd", ref=node_for(c))
                for t in way.findall("tag"):
                    if t.get("k") in ("lanes", "width", "turn:lanes", "change:lanes") or t.get("k").startswith("turn:") \
                            or t.get("k").startswith("lanes:") or t.get("k").startswith("destination"):
                        continue
                    dw.append(ET.Element("tag", k=t.get("k"), v=t.get("v")))
                ET.SubElement(dw, "tag", k="lanes", v=str(RING_DROP_LANES))
                for c in (drop.coords[0], drop.coords[-1]):
                    if edge.distance(Point(c)) < 0.01 and all(math.hypot(c[0] - e[0], c[1] - e[1]) > 0.05
                                                              for e in ends):
                        ends.append((c[0], c[1]))
            for t in way.findall("tag"):
                nw.append(t)
                for c in (g.coords[0], g.coords[-1]):
                    if edge.distance(Point(c)) < 0.01 and all(math.hypot(c[0] - e[0], c[1] - e[1]) > 0.05
                                                              for e in ends):
                        ends.append((c[0], c[1]))
    pts = [(edge.project(Point(c)), tuple(c), False) for c in list(edge.coords)[:-1]]
    pts += [(edge.project(Point(q)), q, True) for q in ends]
    pts.sort(key=lambda v: (v[0], not v[2]))
    coords = []
    for s_, c, meet in pts:
        if coords and math.hypot(c[0] - coords[-1][1][0], c[1] - coords[-1][1][1]) < 0.05:
            if meet and not coords[-1][2]:
                coords[-1] = (s_, c, meet)
            continue
        coords.append((s_, c, meet))
    refs = [node_for(c) for _, c, _ in coords]
    ring = ET.SubElement(root, "way", id=str(nid[0]), version="1", visible="true")
    for r in refs + refs[:1]:
        ET.SubElement(ring, "nd", ref=r)
    for k, v in RING_TAGS.items():
        ET.SubElement(ring, "tag", k=k, v=v)
    first_way = list(root).index(root.find("way"))
    for i, n in enumerate(new_nodes):
        lon, lat = fr.lonlat(*xy[n])
        root.insert(first_way + i, ET.Element("node", id=n, version="1", visible="true",
                                              lat="%.9f" % lat, lon="%.9f" % lon))
    for r in root.findall("relation"):
        root.remove(r)
        root.append(r)
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    print("ring road: %d ways inside, %d cut at the ring, %d outside left out; %d streets join the ring "
          "(%d dead ends just short of it extended)" % (kept, cut, dropped, len(ends), ring_snaps[0]))
    if ring_drop:
        meta["ring_lane_drops"] = ring_drops[0]
        print("ring lane drops: %d wide one-way roads drop to %d lanes %.0f m before the ring"
              % (ring_drops[0], RING_DROP_LANES, RING_DROP_LEN))


def osm_buildings(osm, meta):
    """Building footprints (map frame) of an OSM file, for turnloops.py. / 建物の外形"""
    from shapely.geometry import Polygon
    from shapely.validation import make_valid
    root = ET.parse(osm).getroot()
    fr = geo.Frame(meta)
    xy = {n.get("id"): fr.xy(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    out = []
    for w in root.iter("way"):
        if not any(t.get("k") == "building" for t in w.findall("tag")):
            continue
        pts = [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]
        if len(pts) >= 4:
            p = make_valid(Polygon(pts))
            if not p.is_empty and p.area > 5.0:
                out.append(p)
    return out


def landmark_photo_sites(out_dir, meta):
    """maps.json "roadnet" no_loops_in_sites: the landmark sites of the build dir, read
    each time from every <site>_photos/index.json "bounds" (lat/lon, written by the
    photo-getter; later sites count as soon as their index exists), joined with
    maps.json "landmarks" areas. None when there is none. No dead-end loop is made
    in or near them (turnloops.fix sites). / 詳細モデル敷地（写真索引の範囲）。"""
    import glob
    from shapely.geometry import box as _box
    import turnloops
    fr = geo.Frame(meta)
    out = []
    for f in sorted(glob.glob(os.path.join(out_dir, "*_photos", "index.json"))):
        try:
            b = json.load(open(f)).get("bounds") or {}
            x0, y0 = fr.xy(float(b["west"]), float(b["south"]))
            x1, y1 = fr.xy(float(b["east"]), float(b["north"]))
            out.append(_box(x0, y0, x1, y1))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    lm = None
    try:
        lm = turnloops.landmark_sites(os.path.join(out_dir, meta["name"] + ".ring.osm"), meta)
    except (OSError, ET.ParseError):
        pass
    if lm is not None:
        out.append(lm)
    if not out:
        return None
    from shapely.ops import unary_union
    meta["loop_free_sites"] = len(out)
    print("dead-end loops: none within %.0f m of %d landmark sites (%s)"
          % (turnloops.LOOP_SITE_CLEAR, len(out), ", ".join(os.path.basename(os.path.dirname(f))
                                                            for f in glob.glob(os.path.join(out_dir, "*_photos", "index.json")))))
    return unary_union(out)


STUB_MAX = 0.5             # reversed end pieces shorter than this are cut off / これより短い逆向きの端片は除く


def drop_reversed_stubs(root):
    """netconvert sometimes ends a road with a 0.1-0.15 m line pointing BACK
    (where it cut the edge at the junction's shape): lane -1 jumps 3 m to the
    other side of the reference line in its last sample (OitaKosen roads 3483,
    4026). Such a last piece (< STUB_MAX m, heading > 90 deg off the piece
    before) is removed and the road shortened by it; lane sections, objects and
    signals past the new end are pulled back. Runs before fix_xodr_signals.py
    and elevate.py, so no s-dependent data is lost. Returns the roads fixed.
    逆向きの短い端片（交差点形状で切った残り）を除き、道路を短くする。"""
    import math as _m
    fixed = []
    for r in root.findall("road"):
        gs = r.findall("planView/geometry")
        if len(gs) < 2:
            continue
        last, prev = gs[-1], gs[-2]
        Ls = float(last.get("length"))
        if Ls >= STUB_MAX:
            continue
        # heading at the end of the piece before (line / arc / paramPoly3 end tangent)
        el = list(prev)[0]
        h0, Lp = float(prev.get("hdg")), float(prev.get("length"))
        if el.tag == "arc":
            h1 = h0 + float(el.get("curvature")) * Lp
        elif el.tag == "paramPoly3":
            bU, cU, dU = (float(el.get(k)) for k in ("bU", "cU", "dU"))
            bV, cV, dV = (float(el.get(k)) for k in ("bV", "cV", "dV"))
            p = 1.0 if el.get("pRange", "normalized") == "normalized" else Lp
            h1 = h0 + _m.atan2(bV + 2 * cV * p + 3 * dV * p * p, bU + 2 * cU * p + 3 * dU * p * p)
        else:
            h1 = h0
        if abs((float(last.get("hdg")) - h1 + _m.pi) % (2 * _m.pi) - _m.pi) < _m.pi / 2:
            continue
        r.find("planView").remove(last)
        L = float(r.get("length")) - Ls
        r.set("length", "%.8f" % L)
        lanes = r.find("lanes")
        if lanes is not None:
            for ls in lanes.findall("laneSection"):
                if float(ls.get("s")) > L - 0.05 and ls is not lanes.findall("laneSection")[0]:
                    lanes.remove(ls)
        for tag in ("objects/object", "signals/signal", "signals/signalReference"):
            for o in r.findall(tag):
                if float(o.get("s", "0")) > L:
                    o.set("s", "%.4f" % L)
        fixed.append((r.get("id"), round(Ls, 2)))
    if fixed:
        print("reversed end stubs cut off (netconvert's junction cut): %s" % fixed)
    return fixed


def netconvert(osm, xodr, meta, lefthand):
    fr = geo.Frame(meta)
    cmd = [NETCONVERT if os.path.exists(NETCONVERT) else "netconvert",
           "--osm-files", osm,
           "-o", xodr + ".net.xml",
           "--proj", fr.proj4(),
           "--offset.disable-normalization", "true",
           "--keep-edges.by-vclass", "passenger",
           "--remove-edges.by-type", "highway.service_skip,highway.construction",
           "--osm.turn-lanes", "true",
           "--default.lanewidth", "3.4",
           "--geometry.remove", "true",
           "--roundabouts.guess", "true",
           "--ramps.guess", "true",
           "--junctions.join", "true",
           "--junctions.corner-detail", "5",
           "--tls.guess-signals", "true",
           "--tls.discard-simple", "false" if meta.get("road_fixes") else "true",
           "--tls.join", "true",
           "--tls.default-type", "actuated",
           "--output.street-names", "true",
           "--output.original-names", "true",
           "--remove-edges.isolated", "true",
           "--keep-edges.components", "1",
           "--no-turnarounds.tls", "true",
           "--opendrive-output", xodr + ".raw",
           "--verbose", "false"]
    if lefthand:
        cmd[1:1] = ["--lefthand", "true"]
    if meta.get("road_fixes"):
        # no U-turn connections at ordinary junctions (dead ends keep theirs, or
        # their roads run away): a 3 m lane turned round in a ~2 m radius swings
        # its outer edge 2-3 m past the road edges, and the model paved that as
        # rectangular "ears" at junction mouths (OitaKosen 2026-11-04); cars sent
        # round them climb the kerb. TLS junctions had none already.
        # 通常の交差点では転回接続を作らない（袋小路は残す）: 縁石の「耳」の原因
        cmd[-4:-4] = ["--no-turnarounds.except-deadend", "true"]
    subprocess.check_call(cmd)
    os.remove(xodr + ".net.xml")
    tree = ET.parse(xodr + ".raw")
    root = tree.getroot()
    for el in [root.find("header")] + root.findall("road"):
        if el is not None and "rule" in el.attrib:
            del el.attrib["rule"]                  # never rule="LHT", see the docstring / LHT は付けない
    if meta.get("road_fixes"):
        meta["reversed_stubs"] = drop_reversed_stubs(root)
    tree.write(xodr, encoding="utf-8", xml_declaration=True)
    os.remove(xodr + ".raw")
    fix = os.path.join(HERE, "..", "fix_xodr_signals.py")
    subprocess.check_call([VENV_PY if os.path.exists(VENV_PY) else sys.executable, fix, xodr, xodr])
    if meta.get("road_fixes"):
        # netconvert draws the dead-end turnarounds inside out (start 2.8 m
        # beside the lane, ~150 deg reversed): forward bulb loops instead
        # 袋小路の転回接続（裏返し）を前進する電球形ループに置き換える
        import turnloops
        sites = landmark_photo_sites(os.path.dirname(xodr), meta) \
            if (meta.get("roadnet") or {}).get("no_loops_in_sites") else None
        rep, failed = turnloops.fix(xodr, xodr, buildings=osm_buildings(osm, meta), sites=sites)
        meta["dead_end_loops"] = {str(r["j"]): {k: round(r[k], 2) for k in ("R", "back", "g", "clr")} for r in rep}
        meta["dead_end_no_loop"] = failed
    if (meta.get("roadnet") or {}).get("connect_lane_ends"):
        # maps.json "roadnet": dead lanes merged into their neighbour's connection,
        # one-way stubs to nowhere removed (no dead ends for the Traffic Manager)
        # 行き止まりの車線を隣の接続へ合流させ、行き先の無い一方通行の短い道路を除く
        import lane_ends
        added, left = lane_ends.fix(xodr, xodr)
        meta["lane_ends"] = {"merged": len(added), "left": [list(q) for q in left]}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("osm")
    ap.add_argument("out_dir")
    ap.add_argument("name")
    ap.add_argument("--lefthand", action="store_true", help="keep-left traffic (Japan, Singapore, UK...)")
    ap.add_argument("--road-style", help="maps.json road_style (japan: class speed limits on untagged roads)")
    ap.add_argument("--road-fixes", action="store_true",
                    help="maps.json road_fixes: dual-carriageway lanes, near-miss snapping, medians, bus bays")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    meta = geo.frame_from_osm(args.osm)
    meta.update(name=args.name, lefthand=args.lefthand, ring_inset=RING_INSET, ring_w=RING_W,
                ring_rc=RING_RC)
    opts = roadnet_opts(meta)
    if opts:
        meta["roadnet"] = opts
    if opts.get("layer_elevation"):
        meta["layer_elevation"] = float(opts["layer_elevation"])
        meta["layer_max_grade"] = float(opts.get("layer_max_grade", 10.0))
    meta["ring_wkt"] = make_ring_area(args.osm, meta).wkt
    if args.road_fixes:
        meta["road_fixes"] = True        # also keeps mid-block signals (netconvert) / 単路部の信号も残す
    if args.road_style:
        meta["road_style"] = args.road_style
    json.dump(meta, open(os.path.join(args.out_dir, "meta.json"), "w"), indent=1)
    ring_osm = os.path.join(args.out_dir, args.name + ".ring.osm")
    add_ring(args.osm, ring_osm, meta, fixes=args.road_fixes)
    if args.road_fixes:
        # what the model builds from the road logic (layout.py): kerbed medians
        # between separate carriageways, bus bays at bus stops on through roads
        # 中央分離帯・バスベイ
        meta.update(medians=True, bus_bays=True)
        json.dump(meta, open(os.path.join(args.out_dir, "meta.json"), "w"), indent=1)
    xodr = os.path.join(args.out_dir, args.name + ".xodr")
    netconvert(ring_osm, xodr, meta, args.lefthand)
    json.dump(meta, open(os.path.join(args.out_dir, "meta.json"), "w"), indent=1)
    if meta.get("layer_elevation"):
        # flyovers: decks over other roads raised, ramps <= layer_max_grade, all else z = 0
        # 立体交差: 他の道路を跨ぐ橋面を持ち上げ、取付け部を付ける（他は z = 0）
        subprocess.check_call([VENV_PY if os.path.exists(VENV_PY) else sys.executable,
                               os.path.join(HERE, "grade_sep.py"), xodr,
                               os.path.join(args.out_dir, "meta.json")])
    print("wrote", xodr)


if __name__ == "__main__":
    main()
