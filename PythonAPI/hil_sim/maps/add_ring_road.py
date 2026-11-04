"""Cut the Oita Station OSM data at the map's ring road and add the ring road.

  python add_ring_road.py <in.osm> <out.osm>

The 3D map (oita_station_map_v8) has a made-up ring road, 外周道路, round its
outside. It is not in OpenStreetMap, and the OSM streets run on past it,
out over empty ground. The road logic built from the plain OSM file
therefore had streets running off the edge of the model and no ring road.
This rebuilds the OSM file the way the map's own build does
(source/ground.py, ring_road()):
  - the ring: a square with rounded corners, centre line 423.25 m out from
    the station, corner radius 14 m, two lanes (7.5 m), tertiary, 40 km/h;
  - every street is cut where it meets the ring's centre line and joins the
    ring there, so the ring has a T-junction at each one. A street that
    would run along on top of the ring is left out, like in the model;
  - footways, steps and the railway stop short of the ring's inner pavement,
    so they do not join it;
  - everything outside the ring is gone.
Coordinates are the map's (metres east and north of the station node, the
same projection as source/geo.py).

地図の外周道路（OSM に無い）を追加し、その外側の道路を切り取る。各道路は外周道路と
T 字路で接続する。
"""

import math
import sys
import xml.etree.ElementTree as ET

from shapely.geometry import LineString, Point, box

LAT0, LON0 = 33.2326262, 131.6060258
KX = math.cos(math.radians(LAT0)) * 111319.49
KY = 111132.95

# ring road geometry, from source/ground.py / 外周道路の寸法
HALF = 384.0
RING_W, RING_SIDE, RING_RC = 7.5, 3.0, 14.0
INNER_HALF = HALF + 32.5
RING_C = INNER_HALF + RING_SIDE + RING_W / 2               # 423.25 m
RING_TAGS = {"highway": "tertiary", "name": "外周道路", "name:en": "Ring road", "lanes": "2", "width": "7.5",
             "maxspeed": "40", "oneway": "no"}
VEHICLE = {"trunk", "primary", "secondary", "tertiary", "residential", "unclassified", "living_street",
           "service", "trunk_link", "primary_link", "secondary_link", "tertiary_link", "road"}
ROAD_W = {"trunk": 22, "primary": 18, "secondary": 14, "tertiary": 10, "residential": 6, "unclassified": 6,
          "service": 4.5, "living_street": 5, "trunk_link": 8, "primary_link": 8, "secondary_link": 7}


def proj(lon, lat):
    return ((lon - LON0) * KX, (lat - LAT0) * KY)


def unproj(x, y):
    return (LON0 + x / KX, LAT0 + y / KY)


def ring_area():
    a = RING_C - RING_RC
    return box(-a, -a, a, a).buffer(RING_RC, quad_segs=6)


def lines(g):
    return [q for q in getattr(g, "geoms", [g]) if q.geom_type == "LineString" and not q.is_empty]


def main():
    src, dst = sys.argv[1:3]
    tree = ET.parse(src)
    root = tree.getroot()
    xy = {n.get("id"): proj(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    at = {v: k for k, v in xy.items()}
    area = ring_area()
    edge = area.exterior
    # paths that must not join the ring stop before its inner pavement / 歩道等は内側の歩道手前で止める
    short = area.buffer(-(RING_W / 2 + RING_SIDE + 1.0))
    next_id = [-1]
    new_nodes = []

    def node_for(c):
        c = (c[0], c[1])
        if c in at:
            return at[c]
        nid = str(next_id[0])
        next_id[0] -= 1
        at[c] = nid
        xy[nid] = c
        new_nodes.append(nid)
        return nid

    ends = []                               # meeting points on the ring / 外周道路との接続点
    kept = cut = dropped = 0
    for way in list(root.findall("way")):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        if "highway" not in tags and "railway" not in tags:
            continue
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
                    continue                         # runs on top of the ring / 外周道路と重なる
            pieces.append(g)
        root.remove(way)
        if not pieces:
            dropped += 1
            continue
        cut += 1
        for k, g in enumerate(pieces):
            nw = ET.SubElement(root, "way", dict(way.attrib))
            if k:
                nw.set("id", str(next_id[0]))
                next_id[0] -= 1
            for c in g.coords:
                ET.SubElement(nw, "nd", ref=node_for(c))
            for t in way.findall("tag"):
                nw.append(t)
            if drive:
                for c in (g.coords[0], g.coords[-1]):
                    if edge.distance(Point(c)) < 0.01 and all(
                            math.hypot(c[0] - e[0], c[1] - e[1]) > 0.05 for e in ends):
                        ends.append((c[0], c[1]))
    # the ring through every meeting point / 接続点を通る外周道路
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
    ring_refs = [node_for(c) for _, c, _ in coords]
    ring = ET.SubElement(root, "way", id=str(next_id[0]), version="1", visible="true")
    next_id[0] -= 1
    for r in ring_refs + ring_refs[:1]:
        ET.SubElement(ring, "nd", ref=r)
    for k, v in RING_TAGS.items():
        ET.SubElement(ring, "tag", k=k, v=v)
    # new nodes go before the ways (OSM order: nodes, ways, relations) / ノードはウェイより前に
    first_way = list(root).index(root.find("way"))
    for i, nid in enumerate(new_nodes):
        lon, lat = unproj(*xy[nid])
        root.insert(first_way + i, ET.Element("node", id=nid, version="1", visible="true",
                                              lat="%.9f" % lat, lon="%.9f" % lon))
    # ways go before relations / ウェイはリレーションより前に
    rels = root.findall("relation")
    for r in rels:
        root.remove(r)
        root.append(r)
    b = root.find("bounds")
    if b is not None:
        lo = unproj(-RING_C - 10, -RING_C - 10)
        hi = unproj(RING_C + 10, RING_C + 10)
        b.set("minlon", "%.7f" % lo[0]); b.set("minlat", "%.7f" % lo[1])
        b.set("maxlon", "%.7f" % hi[0]); b.set("maxlat", "%.7f" % hi[1])
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    print("ring road: %d ways inside, %d cut at the ring, %d outside left out; %d streets join the ring"
          % (kept, cut, dropped, len(ends)))


if __name__ == "__main__":
    main()
