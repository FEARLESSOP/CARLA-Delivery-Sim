"""Live check of the 止まれ full stop (traffic_profile.py RuleKeeper, map
guide ch. 23). Read-only: spawns nothing, never touches the Traffic Manager,
never starts a server; it only waits for the ticks of the running pipeline.

  carla-venv/bin/python maps/stop_live.py [--seconds 600] [--junctions 5]
                                          [--stops 900001,900017] [--out FILE]
                                          [--role delivery]

Every passage of a vehicle over a 206 stop line (from the map's landmarks, the
same StopLines index the helper uses) is logged to a CSV: the minimum speed
with the front bumper within 3 m of the line (-1.5 .. +3 m), the longest
standstill (< 0.1 m/s) there, and where the front stood when it stopped.
At the end: per junction (the 5 busiest, or those of --stops) the passages,
how many stopped >= 2 s, the median stop duration and minimum speed, then
the same per role_name ("delivery" = the delivery vans of delivery_sim.py,
driven by their own BasicAgent + delivery/rules.py; "autopilot" = TM NPCs).
--role delivery logs the vans only.

止まれの一時停止のライブ確認（読み取りのみ。アクターも TM も触らない）。
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import carla  # noqa: E402
import traffic_profile as tp  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--seconds", type=float, default=600.0, help="wall-clock seconds")
    ap.add_argument("--junctions", type=int, default=5, help="busiest junctions in the summary")
    ap.add_argument("--stops", default="", help="landmark ids to watch only (comma list)")
    ap.add_argument("--out", default=os.path.join(HERE, "build", "stop_live.csv"))
    ap.add_argument("--role", default="", help="only vehicles with this role_name "
                    "(e.g. delivery) / この role_name の車のみ")
    a = ap.parse_args()

    cl = carla.Client(a.host, a.port)
    cl.set_timeout(20.0)
    world = cl.get_world()
    wmap = world.get_map()
    _name, prof = tp.select_profile(wmap.name, force="japan")
    lines = tp.StopLines(wmap, prof.get("stop_signs") or {}, carla.LaneType.Driving)
    watch = set(s for s in a.stops.split(",") if s)
    print("%s: %d stop lines%s" % (wmap.name, len(lines.stops),
                                   ", watching %s" % sorted(watch) if watch else ""))
    if not lines.stops:
        sys.exit("no 206 stop landmarks on this map")

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fh = open(a.out, "w", newline="")
    out = csv.writer(fh)
    out.writerow(["sim_t", "vehicle", "type", "role", "stop_id", "junction",
                  "min_kmh_at_line", "stop_s", "front_at_stop_m", "stopped_2s"])
    track = {}        # vid -> dict(stop, min_v, still0, best, front, last_d)
    info = {}         # vid -> (type_id, role)
    ids, next_ids = [], 0.0
    rows = []
    end = time.time() + a.seconds
    while time.time() < end:
        snap = world.wait_for_tick(10.0)           # synchronous world: follow its ticks
        now = snap.timestamp.elapsed_seconds
        if now >= next_ids:
            next_ids = now + 2.0
            acts = world.get_actors().filter("vehicle.*")
            ids = [v.id for v in acts
                   if not a.role or v.attributes.get("role_name", "") == a.role]
            for v in acts:
                info.setdefault(v.id, (v.type_id, v.attributes.get("role_name", ""),
                                         v.bounding_box.extent.x))
        for vid in ids:
            act = snap.find(vid)
            if act is None:
                continue
            tf = act.get_transform()
            vel = act.get_velocity()
            v = math.hypot(vel.x, vel.y)
            wp = wmap.get_waypoint(tf.location, lane_type=carla.LaneType.Driving)
            ah = lines.ahead(wp) if wp is not None else None
            tr = track.get(vid)
            if tr is not None and (ah is None or ah[0] != tr["stop"]):
                if tr["last_d"] < 1.0:                  # crossed the line: one passage / 通過
                    st = lines.stops[tr["stop"]]
                    row = [round(now, 1), vid, info.get(vid, ("?", ""))[0],
                           info.get(vid, ("?", ""))[1],
                           st["id"], st["jkey"],
                           round(tr["min_v"] * 3.6, 2) if tr["min_v"] < 1e9 else "",
                           round(tr["best"], 2), "" if tr["front"] is None else round(tr["front"], 2),
                           tr["best"] >= 2.0]
                    out.writerow(row)
                    fh.flush()
                    rows.append(row)
                    print("passage", row)
                track.pop(vid)
                tr = None
            if ah is None:
                continue
            st = lines.stops[ah[0]]
            if watch and str(st["id"]) not in watch:
                continue
            half = info[vid][2] if vid in info else 2.3
            d = ah[1] - half                              # front bumper -> line / 前端から停止線
            if d > 40.0:
                continue
            if tr is None:
                tr = track[vid] = {"stop": ah[0], "min_v": 1e9, "still0": None, "best": 0.0,
                                   "front": None, "last_d": d}
            tr["last_d"] = d
            if -1.5 <= d <= 3.0:
                tr["min_v"] = min(tr["min_v"], v)
                if v < 0.1:
                    if tr["still0"] is None:
                        tr["still0"], tr["front"] = now, d
                    tr["best"] = max(tr["best"], now - tr["still0"])
                else:
                    tr["still0"] = None
    fh.close()

    by_j = {}
    for r in rows:
        by_j.setdefault(r[5], []).append(r)
    pick = sorted(by_j, key=lambda j: -len(by_j[j]))[:a.junctions]
    print("\njunction  passages  stopped>=2s  median stop s  median min km/h  max min km/h")
    for j in pick:
        rs = by_j[j]
        mins = [r[6] for r in rs if r[6] != ""]
        print("%8s  %8d  %11d  %13.1f  %15s  %12s" % (
            j, len(rs), sum(1 for r in rs if r[9]), statistics.median([r[7] for r in rs]),
            "%.1f" % statistics.median(mins) if mins else "-", "%.1f" % max(mins) if mins else "-"))
    by_role = {}
    for r in rows:
        by_role.setdefault(r[3] or "-", []).append(r)
    print("\nrole        passages  stopped>=2s  max min km/h")
    for role, rs in sorted(by_role.items()):
        mins = [r[6] for r in rs if r[6] != ""]
        print("%-10s  %8d  %11d  %12s" % (role, len(rs), sum(1 for r in rs if r[9]),
                                         "%.1f" % max(mins) if mins else "-"))
    print("all: %d passages, %d stopped >= 2 s; CSV %s"
          % (len(rows), sum(1 for r in rows if r[9]), a.out))


if __name__ == "__main__":
    main()
