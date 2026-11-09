"""Turn a built map into the pretend world's map file (no CARLA server needed).

Reads, for one maps.json map:
- the road logic (<build>/<Name>.xodr) through an offline carla.Map: a
  directed road graph of driving lanes with lengths and speed limits;
- buildings.csv: delivery addresses and drone obstacles (footprint + height);
- data/delivery_zones.json: pod zones, no-fly zones, meeting points, chargers.

Writes rl_delivery/maps/<Name>.json. Coordinates are CARLA's (x east, y south).

  python3 -m rl_delivery.prepare_map --map OitaKosen

ビルド済みマップから疑似環境用のマップ（道路グラフ・建物・ゾーン）を作る。
"""
import argparse
import csv
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
sys.path.insert(0, HIL)
from delivery import zones as zonelib  # noqa: E402

STEP = 4.0                # polyline spacing along a lane [m] / 点間隔
DEFAULT_KMH = 40.0


def find_build(name):
    with open(os.path.join(HIL, "maps", "maps.json")) as f:
        meta = json.load(f)
    entries = meta.get("maps", meta) if isinstance(meta, dict) else meta
    items = entries.items() if isinstance(entries, dict) else [(e.get("key"), e) for e in entries]
    for _k, e in items:
        if isinstance(e, dict) and e.get("name") == name:
            return os.path.join(HIL, "maps", "build", name)
    path = os.path.join(HIL, "maps", "build", name)
    if os.path.isdir(path):
        return path
    raise SystemExit("no build folder for %s" % name)


def road_speeds(xodr_text):
    """road id -> max lane speed [km/h] from the xodr. / 道路ごとの制限速度"""
    out = {}
    for m in re.finditer(r'<road\b[^>]*\bid="([^"]+)"(.*?)</road>', xodr_text, re.S):
        sp = [float(x) for x in re.findall(r'<speed\b[^>]*\bmax="([0-9.]+)"', m.group(2))]
        if sp:
            out[int(m.group(1)) if m.group(1).isdigit() else m.group(1)] = max(sp)
    return out


def build_graph(xodr_text, name):
    import carla
    wmap = carla.Map(name, xodr_text)
    speeds = road_speeds(xodr_text)
    key = lambda loc: (round(loc.x * 2) / 2, round(loc.y * 2) / 2)
    nodes, node_xyz, edges, lines = {}, [], [], []

    def node(loc):
        k = key(loc)
        if k not in nodes:
            nodes[k] = len(node_xyz)
            node_xyz.append((round(loc.x, 2), round(loc.y, 2), round(loc.z, 2)))
        return nodes[k]

    for a, b in wmap.get_topology():
        if a.lane_type != carla.LaneType.Driving:
            continue
        pts = [a.transform.location]
        wp = a
        guard = 0
        while guard < 2000:
            guard += 1
            nxt = wp.next(STEP)
            if not nxt:
                break
            nxt = min(nxt, key=lambda w: w.transform.location.distance(b.transform.location))
            if nxt.transform.location.distance(b.transform.location) < STEP or nxt.road_id != a.road_id \
                    and nxt.road_id != b.road_id:
                break
            pts.append(nxt.transform.location)
            wp = nxt
        pts.append(b.transform.location)
        length = sum(pts[i].distance(pts[i + 1]) for i in range(len(pts) - 1))
        if length < 0.1:
            continue
        u, v = node(pts[0]), node(pts[-1])
        kmh = speeds.get(a.road_id, DEFAULT_KMH)
        lines.append([[round(p.x, 1), round(p.y, 1)] for p in pts])
        edges.append([u, v, round(length, 2), kmh, len(lines) - 1, int(a.is_junction)])
    return wmap, node_xyz, edges, lines


def biggest_scc(n, edges):
    """Nodes of the largest strongly connected part (pods never get trapped)."""
    import networkx as nx
    g = nx.DiGraph()
    g.add_nodes_from(range(n))
    g.add_edges_from((e[0], e[1]) for e in edges)
    return max(nx.strongly_connected_components(g), key=len)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="OitaKosen")
    a = ap.parse_args()
    build = find_build(a.map)
    with open(os.path.join(build, a.map + ".xodr")) as f:
        xodr = f.read()
    _wmap, node_xyz, edges, lines = build_graph(xodr, a.map)
    keep = biggest_scc(len(node_xyz), edges)
    xy = np.array([p[:2] for p in node_xyz])
    keep_idx = np.array(sorted(keep))

    def nearest(x, y):
        d = (xy[keep_idx, 0] - x) ** 2 + (xy[keep_idx, 1] - y) ** 2
        i = int(np.argmin(d))
        return int(keep_idx[i]), float(math.sqrt(d[i]))

    buildings = []
    with open(os.path.join(build, "buildings.csv"), newline="") as f:
        for r in csv.DictReader(f):
            x, y = float(r["x_east_m"]), -float(r["y_north_m"])
            n, d = nearest(x, y)
            buildings.append({"id": r["id"], "x": round(x, 2), "y": round(y, 2),
                              "z": float(r["z_base_m"] or 0.0), "h": float(r["height_m"] or 6.0),
                              "len": float(r["length_m"] or 10.0), "wid": float(r["width_m"] or 10.0),
                              "yaw": float(r["yaw_deg"] or 0.0), "node": n, "road_m": round(d, 1),
                              "deliverable": d < 80.0})
    zd = zonelib.load(a.map)
    z = zonelib.Zones(zd)
    chargers = [nearest(c["x"], c["y"])[0] for c in (z.charge or [])]
    if not chargers:            # one per pod zone, near its middle / ゾーン中央付近に 1 か所
        for _vid, r in sorted(z.pods.items()):
            chargers.append(nearest((r["x0"] + r["x1"]) / 2.0, (r["y0"] + r["y1"]) / 2.0)[0])
    out = {"map": a.map, "nodes": node_xyz, "edges": edges, "lines": lines,
           "keep": sorted(int(i) for i in keep), "buildings": buildings, "chargers": chargers,
           "zones": z.summary(),
           "bounds": [float(xy[:, 0].min()), float(xy[:, 1].min()), float(xy[:, 0].max()), float(xy[:, 1].max())]}
    os.makedirs(os.path.join(HERE, "maps"), exist_ok=True)
    path = os.path.join(HERE, "maps", a.map + ".json")
    with open(path, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    print("%s: %d nodes (%d connected), %d lanes, %d buildings (%d deliverable), %d chargers, "
          "%d pod zones, %d no-fly -> %s" % (
              a.map, len(node_xyz), len(keep), len(edges), len(buildings),
              sum(b["deliverable"] for b in buildings), len(chargers), len(z.pods), len(z.nofly), path))


if __name__ == "__main__":
    main()
