"""Lane-level road data for the pod driving world (no CARLA server needed).

For every driving lane piece of the map's road logic (offline carla.Map from
the xodr), every 2 m: the centre point, heading, lane width, the speed limit,
and what lies beyond each edge: "kerb" (no lane: kerb / verge), "lane" (a lane
the same way) or "oncoming" (a lane the other way: the centre line). Plus
the stop lines of the 止まれ signs (type 206) and the traffic signals (1000011
/ 1000001), mapped onto the lane pieces, signals grouped by junction with a
phase by approach direction. Lane pieces use the same edge numbering as
rl_delivery/maps/<Name>.json (prepare_map.py).

  python3 -m rl_delivery.prepare_drive --map OitaKosen

ポッド運転用の車線データ（2 m ごとの中心・幅・縁の種類・停止線・信号）。
"""
import argparse
import json
import math
import os

import numpy as np

from rl_delivery.prepare_map import STEP, find_build, road_speeds

HERE = os.path.dirname(os.path.abspath(__file__))
SAMPLE = 2.0


def side(wp, nb):
    import carla
    if nb is None or nb.lane_type != carla.LaneType.Driving:
        return 0                                   # kerb / verge / 縁石
    return 1 if (nb.lane_id > 0) == (wp.lane_id > 0) else 2   # 1 same way, 2 oncoming / 対向


def main():
    import carla
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="OitaKosen")
    a = ap.parse_args()
    with open(os.path.join(HERE, "maps", a.map + ".json")) as f:
        M = json.load(f)
    with open(os.path.join(find_build(a.map), a.map + ".xodr")) as f:
        xodr = f.read()
    wmap = carla.Map(a.map, xodr)
    speeds = road_speeds(xodr)
    nodes = np.array([n[:2] for n in M["nodes"]])
    # the same topology walk as prepare_map, so edge i matches M["edges"][i]
    # prepare_map と同じ順序で辿る（辺番号を一致させる）
    lanes = []
    for wa, wb in wmap.get_topology():
        if wa.lane_type != carla.LaneType.Driving:
            continue
        seq = [wa]
        wp = wa
        guard = 0
        while guard < 2000:
            guard += 1
            nxt = wp.next(STEP)
            if not nxt:
                break
            nxt = min(nxt, key=lambda w: w.transform.location.distance(wb.transform.location))
            if nxt.transform.location.distance(wb.transform.location) < STEP or nxt.road_id != wa.road_id \
                    and nxt.road_id != wb.road_id:
                break
            seq.append(nxt)
            wp = nxt
        seq.append(wb)
        length = sum(seq[i].transform.location.distance(seq[i + 1].transform.location) for i in range(len(seq) - 1))
        if length < 0.1:
            continue
        # resample every 2 m with lane width and edge kinds / 2 m ごとに再標本化
        pts = []
        for w in seq:
            loc, yaw = w.transform.location, math.radians(w.transform.rotation.yaw)
            pts.append([round(loc.x, 2), round(loc.y, 2), round(loc.z, 2), round(yaw, 4), round(w.lane_width, 2),
                        side(w, w.get_left_lane()), side(w, w.get_right_lane())])
        lanes.append({"road": wa.road_id, "lane": wa.lane_id, "junction": int(wa.is_junction),
                      "kmh": speeds.get(wa.road_id, 40.0), "pts": pts})
    assert len(lanes) == len(M["edges"]), (len(lanes), len(M["edges"]))
    # each direction is its own road on netconvert maps, so CARLA has no "left
    # lane" across the centre line: find neighbours geometrically instead. Side
    # a = +normal (-sin h, cos h), side b = -normal; another lane centre ~1 lane
    # width away there = "lane" (same heading) or "oncoming" (opposite).
    # 対向車線は別の道路なので、幾何で隣接車線を探す
    allp = np.array([q[:4] + [i] for i, L in enumerate(lanes) for q in L["pts"]])
    cell = 4.0
    grid = {}
    for j, q in enumerate(allp):
        grid.setdefault((int(q[0] // cell), int(q[1] // cell)), []).append(j)

    def probe(x, y, h, own):
        best = None
        cx, cy = int(x // cell), int(y // cell)
        for gx in (cx - 1, cx, cx + 1):
            for gy in (cy - 1, cy, cy + 1):
                for j in grid.get((gx, gy), ()):
                    q = allp[j]
                    if int(q[4]) == own:
                        continue
                    d = math.hypot(q[0] - x, q[1] - y)
                    if d < 1.6 and (best is None or d < best[0]):
                        best = (d, q[3])
        if best is None:
            return 0
        dh = abs((best[1] - h + math.pi) % (2 * math.pi) - math.pi)
        return 1 if dh < math.pi / 2 else 2

    for i, L in enumerate(lanes):
        if L["junction"]:
            continue                                   # junction connectors overlap by design / 交差点内は除外
        for q in L["pts"]:
            x, y, h, wdt = q[0], q[1], q[3], q[4]
            nx_, ny_ = -math.sin(h), math.cos(h)
            q[5] = probe(x + nx_ * wdt, y + ny_ * wdt, h, i)
            q[6] = probe(x - nx_ * wdt, y - ny_ * wdt, h, i)
    # stop lines and signals / 停止線と信号
    index = {}
    for i, L in enumerate(lanes):
        index.setdefault(L["road"], []).append(i)
    stops, signals = [], []
    for lm in wmap.get_all_landmarks():
        if lm.type not in ("206", "1000001", "1000011"):
            continue
        for lid in (-4, -3, -2, -1, 1, 2, 3, 4):
            try:
                w = wmap.get_waypoint_xodr(lm.road_id, lid, lm.s)
            except RuntimeError:
                w = None
            if w is None or w.lane_type != carla.LaneType.Driving:
                continue
            p = np.array([w.transform.location.x, w.transform.location.y])
            best = None
            for ei in index.get(lm.road_id, []):
                if lanes[ei]["lane"] != lid:
                    continue
                P = np.array([q[:2] for q in lanes[ei]["pts"]])
                k = int(np.argmin(np.linalg.norm(P - p, axis=1)))
                d = float(np.linalg.norm(P[k] - p))
                if best is None or d < best[0]:
                    best = (d, ei, k)
            if best is None or best[0] > 6.0:
                continue
            rec = {"edge": best[1], "k": best[2], "xy": [round(float(p[0]), 2), round(float(p[1]), 2)],
                   "yaw": round(math.radians(w.transform.rotation.yaw), 3)}
            (stops if lm.type == "206" else signals).append(rec)
    # signals: one junction per cluster within 45 m, phase by approach axis / 交差点ごと・方向で位相
    groups = []
    for s in signals:
        for g in groups:
            if math.dist(g["c"], s["xy"]) < 45.0:
                g["m"].append(s)
                g["c"] = list(np.mean([q["xy"] for q in g["m"]], axis=0))
                break
        else:
            groups.append({"c": list(s["xy"]), "m": [s]})
    for gi, g in enumerate(groups):
        ref = g["m"][0]["yaw"]
        for s in g["m"]:
            s["junction"] = gi
            d = abs((s["yaw"] - ref + math.pi / 2) % math.pi - math.pi / 2)
            s["phase"] = 0 if d < math.pi / 4 else 1
    out = {"map": a.map, "lanes": lanes, "stops": stops, "signals": signals, "junctions": len(groups)}
    path = os.path.join(HERE, "maps", a.map + "_drive.json")
    with open(path, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    kinds = np.array([q[5:7] for L in lanes for q in L["pts"]]).ravel()
    print("%s: %d lanes, %d points (edges: kerb %.0f %%, lane %.0f %%, oncoming %.0f %%), %d stop lines, "
          "%d signal heads at %d junctions -> %s" % (
              a.map, len(lanes), sum(len(L["pts"]) for L in lanes), 100 * float((kinds == 0).mean()),
              100 * float((kinds == 1).mean()), 100 * float((kinds == 2).mean()), len(stops), len(signals),
              len(groups), path))


if __name__ == "__main__":
    main()
