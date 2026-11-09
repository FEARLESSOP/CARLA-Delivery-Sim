#!/usr/bin/env python3
"""Drivability check of a built map, offline (maps/.venv: numpy, shapely, scipy).

Compares the BUILT ground meshes (build/<Name>/meshes/*.npz) with the road
logic (lanes.json) along every driving lane, the way a car's wheels meet them:

  steps     the top surface jumps > STEP m between samples 1 m apart (a kerb
            or slab edge across the lane: the car bumps up or drops)
  above     the top surface stands > ABOVE m over the lane's xodr z (the car
            drives up onto something: slab, pavement, wrong tile height)
  below     the top surface is > BELOW m under the xodr z (the car drops; or
            a hole with grass far below)
  holes     no up-facing ground triangle under the wheel at all
  sheets    two up-facing surfaces > 0.25 m apart under one point (an
            invisible floor the car can land on)
  tilt      left and right wheel tracks differ > TILT m (one wheel on a kerb)
  blocked   mesh vertices of anything inside the lane 0.12-2.6 m over the
            road (dividers, kerbs, posts, walls, foliage in the lane)
  tight     junction lanes whose centre-line radius < R_MIN m
  terrain   road z - terrain field (the hills must stay: a road that lost its
            elevation shows as a big negative/positive here)

Usage: check_drive.py <build dir> [--json out.json] [--top N]
Prints the worst spots in CARLA coordinates (x east, y south) for live checks.

完成した地図の走行性チェック（サーバー不要）: 車線に沿って路面の段差・浮き・穴・
二重面・片輪の傾き・車線内の障害物・急カーブ・地形との差を調べる。
"""
import argparse
import collections
import json
import math
import os
import sys

import numpy as np
import shapely
from shapely import STRtree
from shapely.geometry import LineString, Point, Polygon
from scipy.spatial import cKDTree

STEP, ABOVE, BELOW, TILT, R_MIN = 0.06, 0.08, 0.10, 0.12, 4.5
SHEET_GAP = 0.25
GROUND = ("Road_Road", "Road_Curb", "Road_Sidewalk", "Road_Grass", "Terrain_Lots", "Terrain_Sites",
          "Terrain_Courts")
NOT_OBSTACLE = ("Road_Road", "Road_Marking")      # the road itself and its paint / 路面と塗装
# carriageway pieces kept in another object: the 路側帯 strips (pavements.py) are
# road surface moved into Road_Sidewalk for CARLA's tag and the navmesh / 別オブジェクトの路面
ROAD_SURFACE_MATS = ("asphalt_rosoku",)
LEAVES = ("Leaves", "Trees")                       # crowns: reported apart (4.5 m rule is check_roads')


def family(fn):
    return fn.rsplit("_", 2)[0]


def load_meshes(mdir):
    """{family: [(material, V, I)]} from the npz tiles. / npz タイルを読む。"""
    out = collections.defaultdict(list)
    for fn in sorted(os.listdir(mdir)):
        if not fn.endswith(".npz"):
            continue
        z = np.load(os.path.join(mdir, fn))
        mats = sorted({k.split("|")[0] for k in z.files})
        for m in mats:
            V, I = z.get(m + "|V"), z.get(m + "|I")
            if V is None or I is None or len(I) == 0:
                continue
            out[family(fn)].append((m, V.astype(np.float64), I.astype(np.int64)))
    return out


def up_tris(meshes, fams):
    """Up-facing triangles (N,3,3) of the given families. / 上向き三角形。"""
    tris, src = [], []
    for f in fams:
        for m, V, I in meshes.get(f, []):
            T = V[I]
            n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
            up = n[:, 2] > 0.3 * np.linalg.norm(n, axis=1).clip(1e-9)
            tris.append(T[up])
            src += [(f, m)] * int(up.sum())
    return (np.concatenate(tris) if tris else np.zeros((0, 3, 3))), src


def surface_z(tris, pts):
    """For each point: sorted list of z of the up-facing triangles under it.
    各点の真下にある上向き三角形の z（昇順）。"""
    polys = shapely.polygons(tris[:, :, :2])
    tree = STRtree(polys)
    P = shapely.points(pts[:, :2])
    pi, ti = tree.query(P, predicate="intersects")
    out = [[] for _ in range(len(pts))]
    if len(pi):
        T = tris[ti]
        x, y = pts[pi, 0], pts[pi, 1]
        (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = ((T[:, 0, 0], T[:, 0, 1], T[:, 0, 2]),
                                                     (T[:, 1, 0], T[:, 1, 1], T[:, 1, 2]),
                                                     (T[:, 2, 0], T[:, 2, 1], T[:, 2, 2]))
        det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        det = np.where(np.abs(det) < 1e-12, 1e-12, det)
        a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det
        b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det
        c = 1.0 - a - b
        z = a * z1 + b * z2 + c * z3
        for i, zz in zip(pi, z):
            out[i].append(float(zz))
    return [sorted(v) for v in out]


def carla(p):
    return (round(float(p[0]), 1), round(-float(p[1]), 1))


def lane_samples(lane, inset=0.35):
    """Centre and the two wheel tracks of a lane. / 中心と左右の轍。"""
    pts = lane["pts"]
    C = np.array([p["c"] for p in pts]); Lp = np.array([p["l"] for p in pts]); R = np.array([p["r"] for p in pts])
    Z = np.array([p["z"] for p in pts]); S = np.array([p["s"] for p in pts])
    w = np.array([p["w"] for p in pts]).clip(0.8)
    f = (inset / (w / 2.0))[:, None]          # fraction of the half width / 半幅に対する割合
    L2 = Lp + (C - Lp) * f
    R2 = R + (C - R) * f
    return C, L2, R2, Z, S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build")
    ap.add_argument("--json", default=None)
    ap.add_argument("--top", type=int, default=25)
    a = ap.parse_args()
    bd = a.build
    L = json.load(open(os.path.join(bd, "lanes.json")))
    lanes = [l for l in L["lanes"] if l["kind"] == "driving" and len(l["pts"]) >= 2]
    meshes = load_meshes(os.path.join(bd, "meshes"))
    print("meshes:", {f: len(v) for f, v in meshes.items()})
    ground, gsrc = up_tris(meshes, GROUND)
    print("ground triangles (up-facing):", len(ground))

    # -- samples / サンプル点
    allp, owner = [], []
    for li, l in enumerate(lanes):
        C, Lw, Rw, Z, S = lane_samples(l)
        for k, A in enumerate((C, Lw, Rw)):
            allp.append(np.column_stack([A, Z]))
            owner += [(li, k, i) for i in range(len(A))]
    P = np.concatenate(allp)
    print("lane samples:", len(P))
    zs = surface_z(ground, P)

    rep = collections.defaultdict(list)
    top = np.full(len(P), np.nan)
    for i, v in enumerate(zs):
        if not v:
            rep["holes"].append((owner[i], P[i]))
            continue
        top[i] = v[-1]
        if len(v) > 1 and v[-1] - v[0] > SHEET_GAP:
            rep["sheets"].append((owner[i], P[i], round(v[-1] - v[0], 2)))
        d = v[-1] - P[i, 2]
        if d > ABOVE:
            rep["above"].append((owner[i], P[i], round(d, 2)))
        elif d < -BELOW:
            rep["below"].append((owner[i], P[i], round(d, 2)))
    # steps along each track, tilt across / 進行方向の段差と左右差
    idx = {}
    for n, o in enumerate(owner):
        idx[o] = n
    for li, l in enumerate(lanes):
        npts = len(l["pts"])
        for k in range(3):
            for i in range(1, npts):
                n0, n1 = idx[(li, k, i - 1)], idx[(li, k, i)]
                if np.isnan(top[n0]) or np.isnan(top[n1]):
                    continue
                ds = max(l["pts"][i]["s"] - l["pts"][i - 1]["s"], 0.05)
                dz = top[n1] - top[n0]
                dzx = (l["pts"][i]["z"] - l["pts"][i - 1]["z"])      # the road logic's own slope
                if abs(dz - dzx) > STEP and ds <= 1.5:
                    rep["steps"].append(((li, k, i), P[n1], round(dz - dzx, 2)))
        for i in range(npts):
            nl, nr = idx[(li, 1, i)], idx[(li, 2, i)]
            if np.isnan(top[nl]) or np.isnan(top[nr]):
                continue
            if abs(top[nl] - top[nr]) > TILT:
                rep["tilt"].append(((li, 0, i), P[idx[(li, 0, i)]], round(top[nl] - top[nr], 2)))

    # -- obstacles in lanes / 車線内の障害物
    lane_polys, lane_ids = [], []
    for li, l in enumerate(lanes):
        pts = l["pts"]
        ring = [p["l"] for p in pts] + [p["r"] for p in pts][::-1]
        pg = Polygon(ring)
        if not pg.is_valid:
            pg = pg.buffer(0)
        pg = pg.buffer(-0.2)
        if pg.is_empty:
            continue
        lane_polys.append(pg); lane_ids.append(li)
    ltree = STRtree(lane_polys)
    ctrees = [cKDTree(np.array([p["c"] for p in l["pts"]])) for l in lanes]
    for f, lst in meshes.items():
        if f in NOT_OBSTACLE:
            continue
        for m, V, I in lst:
            if len(V) == 0 or m in ROAD_SURFACE_MATS:
                continue
            pi, lj = ltree.query(shapely.points(V[:, :2]), predicate="within")
            for vi, j in zip(pi, lj):
                li = lane_ids[j]
                d, ii = ctrees[li].query(V[vi, :2])
                zr = lanes[li]["pts"][ii]["z"]
                h = V[vi, 2] - zr
                if 0.12 < h < 2.6:
                    rep["blocked"].append(((li, 0, ii), V[vi], round(h, 2), f + ":" + m))
    # -- tight junction lanes / 急な交差点車線
    for li, l in enumerate(lanes):
        if l["junction"] < 0:
            continue
        pts = l["pts"]
        worst = None
        for i in range(1, len(pts)):
            ds = pts[i]["s"] - pts[i - 1]["s"]
            dy = abs((pts[i]["yaw"] - pts[i - 1]["yaw"] + 180.0) % 360.0 - 180.0)
            if ds > 0.05 and dy > 0.5:
                r = ds / math.radians(dy)
                if worst is None or r < worst[0]:
                    worst = (r, i)
        if worst and worst[0] < R_MIN:
            rep["tight"].append(((li, 0, worst[1]), np.array(pts[worst[1]]["c"] + [pts[worst[1]]["z"]]),
                                 round(worst[0], 1), round(pts[worst[1]]["w"], 1)))
    # -- terrain / 地形との差
    tf = os.path.join(bd, "terrain_field.npz")
    terr = None
    if os.path.exists(tf):
        z = np.load(tf)
        Zf, x0, y0, st = z["Z"], float(z["x0"]), float(z["y0"]), float(z["step"])
        C = np.concatenate([np.array([p["c"] for p in l["pts"]]) for l in lanes])
        Zr = np.concatenate([np.array([p["z"] for p in l["pts"]]) for l in lanes])
        ix = np.clip(((C[:, 0] - x0) / st).astype(int), 0, Zf.shape[1] - 1)
        iy = np.clip(((C[:, 1] - y0) / st).astype(int), 0, Zf.shape[0] - 1)
        diff = Zr - Zf[iy, ix]
        terr = {"road_z_min": round(float(Zr.min()), 1), "road_z_max": round(float(Zr.max()), 1),
                "road_minus_terrain_p5_p50_p95": [round(float(np.percentile(diff, q)), 2) for q in (5, 50, 95)],
                "abs_over_4m": int((np.abs(diff) > 4.0).sum()), "n": int(len(diff))}

    # -- cluster and print / まとめ
    def cluster(items, key=lambda it: abs(it[2]) if len(it) > 2 and isinstance(it[2], (int, float)) else 1.0):
        """Group hits within 6 m; keep the worst of each group. / 6 m 以内をまとめる。"""
        groups = []
        for it in sorted(items, key=key, reverse=True):
            p = it[1]
            for g in groups:
                if abs(g["p"][0] - p[0]) < 6.0 and abs(g["p"][1] - p[1]) < 6.0:
                    g["n"] += 1
                    break
            else:
                li = it[0][0]
                groups.append({"p": p, "n": 1, "worst": it, "lane": (lanes[li]["road"], lanes[li]["lane"]),
                               "junction": lanes[li]["junction"], "name": lanes[li].get("name") or ""})
        return groups

    summary, out = {}, {}
    for kind in ("steps", "above", "below", "holes", "sheets", "tilt", "blocked", "tight"):
        g = cluster(rep[kind])
        summary[kind] = {"hits": len(rep[kind]), "spots": len(g)}
        out[kind] = [{"carla_xy": carla(x["p"]), "z": round(float(x["p"][2]), 1), "n": x["n"],
                      "value": (x["worst"][2] if len(x["worst"]) > 2 else None),
                      "what": (x["worst"][3] if len(x["worst"]) > 3 else None),
                      "road_lane": x["lane"], "junction": x["junction"], "name": x["name"]} for x in g]
    if a.json:
        json.dump({"summary": summary, "terrain": terr, "spots": out}, open(a.json, "w"), indent=1, default=float)
    print("\n== summary (hits / distinct spots) ==")
    for k, v in summary.items():
        print("  %-8s %6d hits  %4d spots" % (k, v["hits"], v["spots"]))
    if terr:
        print("  terrain  road z %.1f..%.1f m; road-terrain p5/p50/p95 %s; |diff|>4 m at %d/%d samples" % (
            terr["road_z_min"], terr["road_z_max"], terr["road_minus_terrain_p5_p50_p95"], terr["abs_over_4m"], terr["n"]))
    for kind in ("steps", "above", "below", "holes", "sheets", "tilt", "blocked", "tight"):
        g = out[kind][:a.top]
        if not g:
            continue
        print("\n-- %s (worst %d of %d spots; CARLA x,y) --" % (kind, len(g), len(out[kind])))
        for x in g:
            print("  %-16s z %5.1f  n=%-4d %-7s %-28s road %s lane %s j%s %s" % (
                str(x["carla_xy"]), x["z"], x["n"], x["value"] if x["value"] is not None else "",
                str(x["what"] or "")[:28], x["road_lane"][0], x["road_lane"][1], x["junction"], x["name"][:24]))
    if a.json:
        print("\nwritten", a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
