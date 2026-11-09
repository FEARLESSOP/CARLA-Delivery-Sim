"""Gate: no driving lane runs into the kept model's walls (maps.json "core", maps/.venv).

  maps/.venv/bin/python check_keep_walls.py <build dir> [--clear 0.3] [--json out.json] [--warn]

The embedded model's kept objects (the v8 JR_Oita_City station and Railway,
core "keep") are solid in CARLA: their faces at wheel height (0.12-2.6 m over
the model's ground, core/keep_walls.npz, written by fit_core_widths.py) are
walls, columns and viaduct piers. check_drive cannot see them (they are not
in the npz ground), and fit_core_widths only narrows lanes down to the width
floors. This check measures every driving lane of lanes.json (street and
junction lanes, so the lane interior and both wheel tracks) against those
faces in plan view and lists every lane closer than --clear (default 0.3 m)
by road id, lane, junction and the wall's position (map frame x east / y
north, and CARLA x / y = -y), with the wall's height range. Exit 1 when any
lane is listed (make_osm_city.sh stops the chain), unless --warn.
Fix offenders in the road logic: maps.json osm_fixes (drop / split the OSM
way, never a dead-end stub), guide ch. 35. No core: nothing to do.

残すモデル（駅・高架）の車輪の高さの面から 0.3 m 以内を通る走行車線を道路 ID と位置で
列挙し、あれば終了コード 1（make_osm_city.sh が止まる）。
"""

import argparse
import json
import os
import sys

import numpy as np
import shapely
from shapely.geometry import Polygon


def wall_geoms(bd, sx):
    """The kept faces at wheel height in the map frame (x scaled by core x_scale):
    triangles as polygons, faces seen edge-on (vertical walls) as lines, and each
    face's height range. / 残す面（平面図: 鉛直面は線）と高さ範囲"""
    W = np.load(os.path.join(bd, "core", "keep_walls.npz"))["walls"].astype(np.float64)
    W[:, :, 0] *= sx
    ar = np.abs((W[:, 1, 0] - W[:, 0, 0]) * (W[:, 2, 1] - W[:, 0, 1])
                - (W[:, 2, 0] - W[:, 0, 0]) * (W[:, 1, 1] - W[:, 0, 1]))
    G = np.where(ar > 1e-6, shapely.polygons(np.concatenate([W[:, :, :2], W[:, :1, :2]], 1)),
                 shapely.linestrings(W[:, :, :2]))
    return shapely.make_valid(G), W[:, :, 2].min(1), W[:, :, 2].max(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("build")
    ap.add_argument("--clear", type=float, default=0.3, help="least clearance (m) / 最小離れ")
    ap.add_argument("--json")
    ap.add_argument("--warn", action="store_true", help="report only, exit 0 / 報告のみ")
    a = ap.parse_args()
    bd = os.path.abspath(a.build)
    meta = json.load(open(os.path.join(bd, "meta.json")))
    core = meta.get("core")
    if not core:
        print("[check_keep_walls] no core in meta.json: nothing to do")
        return 0
    if not os.path.exists(os.path.join(bd, "core", "keep_walls.npz")):
        print("[check_keep_walls] core/keep_walls.npz missing (fit_core_widths.py writes it)")
        return 0 if a.warn else 1
    G, z0, z1 = wall_geoms(bd, float(core.get("x_scale", 1.0)))
    tree = shapely.STRtree(G)
    lanes = json.load(open(os.path.join(bd, "lanes.json")))["lanes"]
    rows, n = [], 0
    for l in lanes:
        if l["kind"] != "driving" or len(l["pts"]) < 2:
            continue
        p = Polygon([tuple(q["l"]) for q in l["pts"]] + [tuple(q["r"]) for q in l["pts"]][::-1]).buffer(0)
        n += 1
        idx = [i for i in tree.query(p.buffer(a.clear)) if p.distance(G[i]) < a.clear]
        if not idx:
            continue
        d = min(p.distance(G[i]) for i in idx)
        near = shapely.union_all([G[i] for i in idx])
        inside = near.intersection(p)
        at = (inside if not inside.is_empty else near).centroid
        rows.append(dict(road=l["road"], lane=l["lane"], junction=l["junction"], clear=round(d, 2),
                         inside_m=round(inside.length + inside.area, 2), faces=len(idx),
                         map_xy=[round(at.x, 1), round(at.y, 1)], carla_xy=[round(at.x, 1), round(-at.y, 1)],
                         wall_z=[round(float(z0[idx].min()), 2), round(float(z1[idx].max()), 2)]))
    rows.sort(key=lambda r: (r["clear"], -r["inside_m"]))
    if a.json:
        json.dump(dict(clear=a.clear, lanes=n, offenders=rows), open(a.json, "w"), indent=1)
    if not rows:
        print("[check_keep_walls] %d driving lanes, none within %.2f m of the kept model's walls (core/keep_walls.npz)"
              % (n, a.clear))
        return 0
    print("[check_keep_walls] FAIL: %d driving lanes within %.2f m of the kept model's walls / 残す壁に近い車線:"
          % (len(rows), a.clear))
    for r in rows:
        print("  road %(road)d lane %(lane)d junction %(junction)d: clearance %(clear).2f m, %(inside_m).1f m of wall "
              "inside the lane, at map (%(mx).1f, %(my).1f) = CARLA (%(cx).1f, %(cy).1f), wall z %(z0).1f-%(z1).1f m"
              % dict(r, mx=r["map_xy"][0], my=r["map_xy"][1], cx=r["carla_xy"][0], cy=r["carla_xy"][1],
                     z0=r["wall_z"][0], z1=r["wall_z"][1]))
    print("  fix in the road logic: maps.json osm_fixes (drop / split the way), guide ch. 35")
    return 0 if a.warn else 1


if __name__ == "__main__":
    sys.exit(main())
