"""Crossings for the walkers and the pedestrian signal heads (carla-venv).

  carla-venv/bin/python hokousha_crossings.py <build/<Name>> [--report]

Reads the build's crossings.pkl (markings_jp: every painted crossing, map
frame x east / y north) and <Name>.xodr, and writes build/<Name>/hokousha.json
in CARLA's frame (x, -y), which the rig reads at run time:

* every crossing: centre, the across-road axis n (t0 -> t1 = kerb to kerb), the
  along-road axis f, width, both kerb ends and their waiting zones;
* signalised ones also: the vehicle signals whose traffic drives THROUGH the
  crossing ("conflict": the approach just before it, and the straight-on
  traffic of the opposite approach leaving the junction across it), the
  junction's other signals, the length (for the 青点滅 time) and two head
  poles (one per kerb end, beside the crossing, facing across it);
* --report: which light CARLA's own walker logic (WalkerManager: the light of
  the nearest stop waypoint) would wait for at each kerb end, and whether that
  light really governs the crossing.

Pure road-logic read: nothing in the xodr changes, so no TM cache or navmesh
rebuild. The mesh (jp_hokousha.py) and the client (hil_sim/hokousha.py) read
this file.

横断歩道（歩行者と歩行者用信号のため）を CARLA 座標で書き出す。各横断歩道を
通過する車両信号（対立信号）を道路情報から求める。xodr は変更しない。
"""

import argparse
import json
import math
import os
import pickle
import sys

import carla
import numpy as np
from shapely.geometry import Point, Polygon

TL_TYPE = "1000011"
KERB_BACK = 0.9          # pole this far behind the carriageway edge (kerb 0.2 + 0.7) / 車道端から支柱
POLE_ALONG = 0.6         # pole beside the crossing: this far past its side edge / 横断歩道の側縁から
LANE_CLEAR = 0.5         # pole + housing this far from every driving lane / 走行車線からの離隔
BAND_CLEAR = 0.5         # ... and from every crossing band / 横断歩道の帯からの離隔
POST_CLEAR = 1.0         # pole this far from light poles, sign posts, other heads / 他の柱からの距離
WAIT_DEPTH = 3.0         # waiting zone depth on the pavement / 待機範囲の奥行き
STRAIGHT_DEG = 35.0      # a branch within this of the approach heading is "straight on" / 直進とみなす角度
NEAR_M = 0.0            # turning traffic (yields to walkers, 38 条) does not count / 右左折車は数えない
FAR_M = 70.0             # straight-on traffic followed this far / 直進交通をこの距離まで追う


def log(*a):
    print("[hokousha_crossings]", *a)
    sys.stdout.flush()


def cxy(p):
    """map frame (x, y north) -> CARLA (x, y south). / 地図座標 -> CARLA 座標"""
    return [float(p[0]), -float(p[1])]


def light_lanes(m):
    """{signal id: [stop waypoints]} from the xodr's traffic-light landmarks."""
    out = {}
    for lm in m.get_all_landmarks_of_type(TL_TYPE):
        wps = out.setdefault(lm.id, [])
        for a, b in lm.get_lane_validities():
            for lane in range(min(a, b), max(a, b) + 1):
                if lane == 0:
                    continue
                try:
                    wp = m.get_waypoint_xodr(lm.road_id, lane, lm.s)
                except RuntimeError:
                    wp = None
                if wp is not None and wp.lane_type == carla.LaneType.Driving:
                    if all(w.transform.location.distance(wp.transform.location) > 0.5 for w in wps):
                        wps.append(wp)
    return {k: v for k, v in out.items() if v}


def _yaw_diff(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


def traffic_points(wp, step=1.0):
    """Points the traffic from this stop waypoint drives over: every branch for
    NEAR_M, then the straight-on branches to FAR_M (CARLA frame xy)."""
    pts = []
    start_yaw = wp.transform.rotation.yaw
    front = [(wp, 0.0)]
    seen = set()
    while front:
        w, d = front.pop()
        loc = w.transform.location
        pts.append((loc.x, loc.y))
        if d >= FAR_M:
            continue
        for n in w.next(step):
            key = (n.road_id, n.section_id, n.lane_id, round(n.s, 0))
            if key in seen:
                continue
            seen.add(key)
            nd = d + step
            if nd > NEAR_M and _yaw_diff(n.transform.rotation.yaw, start_yaw) > STRAIGHT_DEG:
                continue
            front.append((n, nd))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--report", action="store_true")
    args = ap.parse_args()
    out = os.path.abspath(args.out)
    meta = json.load(open(os.path.join(out, "meta.json")))
    name = meta["name"]
    m = carla.Map(name, open(os.path.join(out, name + ".xodr")).read())
    extra = pickle.load(open(os.path.join(out, "crossings.pkl"), "rb"))
    lights = light_lanes(m)
    log("%d traffic-light signals with stop waypoints" % len(lights))
    tl_pts = {sid: [p for wp in wps for p in traffic_points(wp)] for sid, wps in lights.items()}
    stop_locs = [(sid, wp.transform.location) for sid, wps in lights.items() for wp in wps]

    def driving_at(x, y, z):
        return m.get_waypoint(carla.Location(x, y, z + 0.5), project_to_road=False,
                              lane_type=carla.LaneType.Driving) is not None

    def ground_z(x, y):
        wp = m.get_waypoint(carla.Location(x, y, 0.0))
        return wp.transform.location.z if wp is not None else 0.0

    # the xodr's phase controllers per junction (CARLA cycles them in an order of
    # its own; the client learns it at run time) / 交差点ごとの現示（順序は実行時に学習）
    import xml.etree.ElementTree as ET
    junction_phases = {}
    for ctl in ET.parse(os.path.join(out, name + ".xodr")).getroot().iter("controller"):
        sigs = [c.get("signalId") for c in ctl.iter("control")]
        if sigs:
            junction_phases.setdefault(ctl.get("id").rsplit("_p", 1)[0], []).append(sigs)
    # bands of every crossing; a crossing lying on another one (same line drawn
    # twice in OSM, parallel overlapping bands) is dropped, the bigger kept
    # 全横断歩道の帯。重なる重複（OSM の二重線など）は大きい方を残す
    raw = []
    for idx, cr in enumerate(extra.get("crossings", [])):
        c = np.array(cxy(cr["c"]))
        f = np.array(cxy(cr["f"]))
        n = np.array(cxy(cr["n"]))
        t0, t1, w = float(cr["t0"]), float(cr["t1"]), float(cr["width"])
        band = Polygon([c + n * t0 - f * w / 2, c + n * t1 - f * w / 2,
                        c + n * t1 + f * w / 2, c + n * t0 + f * w / 2])
        raw.append((idx, cr, c, f, n, t0, t1, w, band))
    dropped = set()
    for i in range(len(raw)):
        for j in range(i + 1, len(raw)):
            a, b = raw[i], raw[j]
            if a[0] in dropped or b[0] in dropped or abs(float(a[4] @ b[4])) < 0.9:
                continue
            ov = a[8].intersection(b[8]).area
            if ov > 0.3 * min(a[8].area, b[8].area):
                lose = a if a[8].area < b[8].area else b
                keep = b if lose is a else a
                dropped.add(lose[0])
                if lose[1].get("signal") and not keep[1].get("signal"):
                    keep[1]["signal"] = True
                log("crossing %d lies on crossing %d (%.1f m2 overlap): merged" % (lose[0], keep[0], ov))
    raw = [r for r in raw if r[0] not in dropped]
    bands = [r[8] for r in raw]
    # things a head pole must keep clear of: light poles, sign posts, other heads
    # 支柱が避けるもの: 信号柱・標識柱・他の灯器
    posts = [(lm.transform.location.x, lm.transform.location.y) for lm in m.get_all_landmarks_of_type(TL_TYPE)]
    try:
        for pl in json.load(open(os.path.join(out, "jp_plates.json"))).get("plates", []):
            posts.append((float(pl["x"]), -float(pl["y"])))
    except (OSError, ValueError, KeyError):
        pass
    placed = []

    def lane_gap(x, y, z):
        """Distance outside the nearest driving lane (negative = on it). / 走行車線の外側までの距離"""
        loc = carla.Location(x, y, z + 0.5)
        if m.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            return -1.0
        wp = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            return 99.0
        r = wp.transform.get_right_vector()
        d = abs((x - wp.transform.location.x) * r.x + (y - wp.transform.location.y) * r.y)
        return d - wp.lane_width / 2.0

    def head_ok(p, face, z):
        """Pole + housing footprint >= 0.5 m from every lane and band, >= 1 m from posts."""
        perp = np.array([-face[1], face[0]])
        pts = [p] + [p + face * a + perp * b for a in (0.0, 0.36) for b in (-0.21, 0.21)]
        for q in pts:
            if lane_gap(q[0], q[1], z) < LANE_CLEAR:
                return False
            if any(bd.distance(Point(q)) < BAND_CLEAR for bd in bands):
                return False
        return all(math.hypot(p[0] - x, p[1] - y) >= POST_CLEAR for x, y in posts + placed)

    crossings, report = [], []
    for idx, cr, c, f, n, t0, t1, w, band in raw:
        z = ground_z(*c)
        ends, zones = [], []
        for t, sgn in ((t0, -1.0), (t1, 1.0)):
            e = c + n * t
            ends.append([round(e[0], 2), round(e[1], 2)])
            q0, q1 = e - n * sgn * 0.5, e + n * sgn * WAIT_DEPTH
            zones.append([[round(v, 2) for v in p] for p in
                          (q0 - f * (w / 2 + 0.5), q1 - f * (w / 2 + 0.5), q1 + f * (w / 2 + 0.5), q0 + f * (w / 2 + 0.5))])
        rec = {"id": idx, "c": [round(c[0], 2), round(c[1], 2)], "z": round(z, 2),
               "n": [round(v, 5) for v in n], "f": [round(v, 5) for v in f],
               "t0": round(t0, 2), "t1": round(t1, 2), "width": w, "length": round(t1 - t0, 2),
               "ends": ends, "wait_zones": zones, "signal": bool(cr.get("signal")),
               "src": cr.get("src", "")}
        if rec["signal"]:
            hit = band.buffer(0.3)
            conflict = sorted(sid for sid, pts in tl_pts.items()
                              if any(hit.contains(Point(p)) for p in pts))
            near = sorted({sid for sid, loc in stop_locs
                           if math.hypot(loc.x - c[0], loc.y - c[1]) < 60.0})
            rec["conflict"] = conflict
            jk = next((k for k, ph in junction_phases.items() if any(sid in p for p in ph for sid in conflict)), None)
            rec["junction"] = jk
            rec["phases"] = junction_phases.get(jk, [])
            rec["junction_lights"] = near
            # head poles: on the pavement at each end, beside the crossing on the
            # side away from the junction-ish centre of the near lights; facing the
            # other end / 支柱: 各端の歩道、横断歩道の脇、対岸を向く
            heads = []
            if near:
                jc = np.mean([[loc.x, loc.y] for sid, loc in stop_locs if sid in near], axis=0)
                away = 1.0 if float((c - jc) @ f) >= 0 else -1.0
            else:
                away = 1.0
            for (t, sgn) in ((t0, -1.0), (t1, 1.0)):
                spot = None
                face = -n * sgn                   # toward the other end / 対岸へ向く
                for extra_along in (POLE_ALONG, 1.0, 1.5, 2.2, 3.0, 4.0):
                    for side in (away, -away):
                        for back in (KERB_BACK, 1.2, 1.5, 2.0, 2.5, 3.0, 3.5):
                            p = c + n * (t + sgn * back) + f * side * (w / 2 + extra_along)
                            if head_ok(p, face, z):
                                spot = p
                                break
                        if spot is not None:
                            break
                    if spot is not None:
                        break
                if spot is None:
                    log("crossing %d end t=%.1f: no clear pole spot, head left out" % (idx, t))
                    continue
                placed.append((spot[0], spot[1]))
                heads.append({"x": round(spot[0], 2), "y": round(spot[1], 2),
                              "z": round(ground_z(*spot) + 0.15, 2),
                              "face": [round(face[0], 5), round(face[1], 5)]})
            rec["heads"] = heads
            if not conflict:
                log("crossing %d: signalised but no light drives through it (kept unsignalised)" % idx)
                rec["signal"] = False
            # what CARLA's WalkerManager would pick / CARLA 標準の歩行者が見る信号
            for k, e in enumerate(ends):
                best = min(stop_locs, key=lambda s: (s[1].x - e[0]) ** 2 + (s[1].y - e[1]) ** 2) if stop_locs else None
                if best is not None:
                    d = math.hypot(best[1].x - e[0], best[1].y - e[1])
                    report.append((idx, k, best[0], round(d, 1), best[0] in conflict))
        crossings.append(rec)
    sig = [r for r in crossings if r["signal"]]
    data = {"map": name, "frame": "carla", "crossings": crossings,
            "_doc": "hokousha_crossings.py: crossings in CARLA's frame; conflict = vehicle signals driving "
                    "through the crossing. / 横断歩道（CARLA 座標）と対立する車両信号"}
    json.dump(data, open(os.path.join(out, "hokousha.json"), "w"), indent=0)
    log("wrote hokousha.json: %d crossings, %d signalised, %d head poles"
        % (len(crossings), len(sig), sum(len(r.get("heads", [])) for r in sig)))
    if True:   # always printed (cheap) / 常に表示
        ok = sum(1 for r in report if r[4])
        log("CARLA walker light check: %d/%d kerb ends would wait for a light that governs the crossing"
            % (ok, len(report)))
        for r in report:
            if not r[4]:
                log("  crossing %d end %d: nearest stop waypoint is %s (%.1f m), not a conflicting light"
                    % (r[0], r[1], r[2], r[3]))


if __name__ == "__main__":
    main()
