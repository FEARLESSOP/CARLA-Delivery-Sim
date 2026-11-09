"""Signal stop lines vs the light poles (carla-venv, offline; guide ch. 32).

  python check_stop_poles.py <build dir>

For every painted signal stop line (markings.pkl, build.py) the light of its
road, located the way CARLA places it (carla.Map landmark transform: pole =
positionInertial, else reference line at s, offset t), must stand AFTER the
line for the driver (the driver stops before the pole, Japanese practice: the
head hangs over the lanes beyond the line). Prints, per line, the along-lane
distance line -> pole and the crossing order (line, crossing, pole), CARLA x/y.
Exit 1 when a pole stands before its line.

信号の停止線と信号柱の順序（運転者から見て 停止線 -> 柱）を確認する。
"""

import json
import math
import os
import pickle
import sys

import carla
import numpy as np
from shapely.geometry import LineString, Point

TL_TYPE = "1000011"


def main():
    out = os.path.abspath(sys.argv[1])
    meta = json.load(open(os.path.join(out, "meta.json")))
    name = meta["name"]
    m = carla.Map(name, open(os.path.join(out, name + ".xodr")).read())
    els = pickle.load(open(os.path.join(out, "markings.pkl"), "rb"))["elements"]
    lanes = {(l["road"], l["lane"]): l for l in json.load(open(os.path.join(out, "lanes.json")))["lanes"]}
    poles = {}
    for lm in m.get_all_landmarks_of_type(TL_TYPE):
        loc = lm.transform.location
        poles.setdefault(lm.road_id, []).append((lm.id, np.array([loc.x, -loc.y])))   # map frame / 地図座標
    bad, n = 0, 0
    for e in els:
        if e["kind"] != "stop_signal":
            continue
        l = lanes.get((e["road"], e["lane"]))
        if l is None or e["road"] not in poles:
            continue
        ln = LineString([q["c"] for q in l["pts"]])
        ctr = e["geom"].centroid
        s_line = ln.project(ctr) + 0.225
        for sid, p in poles[e["road"]]:
            n += 1
            # the pole projected on the lane, beyond its end: along the end heading / 柱の車線上の位置
            P = np.asarray(ln.coords)
            fe = (P[-1] - P[-2]) / (np.linalg.norm(P[-1] - P[-2]) or 1.0)
            ext = LineString(list(map(tuple, P)) + [tuple(P[-1] + fe * 30.0)])
            s_pole = ext.project(Point(*p))
            d = s_pole - s_line
            cr = e.get("crossing")
            order = "line %.1f m -> pole" % d
            if cr is not None:
                order = "line -> crossing %d (%.1f m back) -> pole %.1f m after the line" % (cr, e.get("back", 0), d)
            ok = d >= 0.0
            bad += 0 if ok else 1
            print("%s tl %s road %d lane %d line CARLA (%.1f, %.1f) pole CARLA (%.1f, %.1f): %s"
                  % ("ok  " if ok else "FAIL", sid, e["road"], e["lane"], ctr.x, -ctr.y, p[0], -p[1], order))
    print("stop-pole check: %d line/pole pairs, %d poles before their line: %s" % (n, bad, "PASS" if not bad else "FAIL"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
