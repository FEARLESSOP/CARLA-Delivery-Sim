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


def make_ring_area(osm_path, meta):
    """The ring road follows the OSM box, and runs along the near side of an
    expressway that crosses the map instead of meeting it at grade (the
    expressway's slip roads join the ring like any street). Rounded corners.
    外周道路は地図の枠に沿い、地図を横切る高速道路は平面交差させず手前を通す。"""
    from shapely.ops import linemerge, split, unary_union
    fr = geo.Frame(meta)
    root = ET.parse(osm_path).getroot()
    osmfix.apply(root, meta, quiet=True)          # maps.json "osm_fixes" / 地図ごとの OSM 修正
    xy = {n.get("id"): fr.xy(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    region = box(-meta["half_x"], -meta["half_y"], meta["half_x"], meta["half_y"])
    mw = []
    for w in root.iter("way"):
        t = {k.get("k"): k.get("v") for k in w.findall("tag")}
        if t.get("highway") == "motorway":
            pts = [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]
            if len(pts) >= 2:
                mw.append(LineString(pts))
    if mw:
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


DUAL_LANE_W = 3.0          # lane width on the halves of a dual carriageway / 分離道路の車線幅


def fix_dual_carriageways(root, xy):
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
            if n >= 2 and n % 2 == 0 and n * DUAL_LANE_W > sep and not t.get("_halved"):
                t["_halved"] = True
                for el in w.findall("tag"):
                    if el.get("k") == "lanes":
                        el.set("v", str(n // 2))
                t["lanes"] = str(n // 2)
                fixed += 1
            if "width" not in t and "lanes" in t:
                wd = "%.2f" % (DUAL_LANE_W * int(float(t["lanes"].split(";")[0])))
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
    nid = [-1]
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
        meta["dual_carriageways"] = fix_dual_carriageways(root, xy)
        meta["near_misses"] = snap_near_misses(root, xy, node_for)
    from collections import Counter
    use = Counter(nd.get("ref") for w in root.findall("way") for nd in w.findall("nd")
                  if drivable({t.get("k"): t.get("v") for t in w.findall("tag")}))
    ends, kept, cut, dropped = [], 0, 0, 0
    ring_snaps = [0]
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
            for c in g.coords:
                ET.SubElement(nw, "nd", ref=node_for(c))
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
    tree.write(xodr, encoding="utf-8", xml_declaration=True)
    os.remove(xodr + ".raw")
    fix = os.path.join(HERE, "..", "fix_xodr_signals.py")
    subprocess.check_call([VENV_PY if os.path.exists(VENV_PY) else sys.executable, fix, xodr, xodr])


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
    print("wrote", xodr)


if __name__ == "__main__":
    main()
