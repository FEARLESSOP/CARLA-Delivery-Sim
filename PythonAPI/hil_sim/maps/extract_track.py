"""Track centreline, width, race direction and start pose from a circuit model.

  python extract_track.py <maps.json> <map key> <out_dir>      (needs scipy, scikit-image)

From the map's entry in maps.json:
  track_material   the racing surface: one clean closed ribbon. Its skeleton
                   is the centreline (rasterised at 0.5 m, spurs pruned, the
                   loop walked, smoothed with a closed spline, ~1 m spacing).
  road_materials   all the asphalt: stepping out from the centreline across
                   it gives the real edges, so the centre is re-centred and
                   the width measured (capped at 12 m a side, run-off excluded).
  start_line, pole the chequered line and the pole grid slot, in model
                   coordinates (found in a top-down render). The race goes
                   from the pole across the line; the centreline is turned to
                   run that way and to start at the line.

Writes <out_dir>/track.json (offset + points [x, y, z, width] in model axes)
and stores the CARLA start pose in maps.json. The triangles come from
Blender (export_tris), run by this script.

サーキットのモデルからセンターライン・幅・走行方向・スタート位置を求める。
"""

import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
from scipy import ndimage
from scipy.interpolate import LinearNDInterpolator, splev, splprep
from skimage.draw import polygon
from skimage.morphology import closing, disk, remove_small_holes, skeletonize

EXPORT = r'''
import bpy, sys, numpy as np
src, out = sys.argv[sys.argv.index("--") + 1:][:2]
mats = sys.argv[sys.argv.index("--") + 3:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=src) if src.lower().endswith((".glb", ".gltf")) else bpy.ops.import_scene.fbx(filepath=src)
for o in [o for o in bpy.context.scene.objects if o.type == "MESH"]:
    me = o.data; me.calc_loop_triangles(); M = np.array(o.matrix_world)
    co = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get("co", co); co = co.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
    nt = len(me.loop_triangles); tv = np.zeros(nt * 3, dtype=np.int64); me.loop_triangles.foreach_get("vertices", tv)
    tm = np.zeros(nt, dtype=np.int64); me.loop_triangles.foreach_get("material_index", tm)
    names = [m.name if m else "" for m in me.materials]
    for m in mats:
        if m in names:
            f = "%s_%s.npy" % (out, m); old = np.load(f) if __import__("os").path.exists(f) else np.zeros((0, 3, 3), np.float32)
            np.save(f, np.concatenate([old, co[tv.reshape(-1, 3)[tm == names.index(m)]].astype(np.float32)]))
'''


def rasterise(tris, x0, y0, g, shape):
    mask = np.zeros(shape, bool)
    for t in tris:
        rr, cc = polygon((t[:, 1] - y0) / g, (t[:, 0] - x0) / g, shape=shape)
        mask[rr, cc] = True
    return mask


def centreline(tris):
    g = 0.5
    x0, y0 = tris[:, :, 0].min() - 20, tris[:, :, 1].min() - 20
    shape = (int((tris[:, :, 1].max() + 20 - y0) / g), int((tris[:, :, 0].max() + 20 - x0) / g))
    mask = closing(rasterise(tris, x0, y0, g, shape), disk(3))
    mask = remove_small_holes(mask, 2000)
    lab, n = ndimage.label(mask)
    mask = lab == (1 + int(np.argmax(ndimage.sum(mask, lab, range(1, n + 1)))))
    sk = skeletonize(mask)
    k = np.ones((3, 3), int)
    k[1, 1] = 0
    for _ in range(400):                                  # prune spurs / ひげを除去
        ends = sk & (ndimage.convolve(sk.astype(int), k, mode="constant") == 1)
        if not ends.any():
            break
        sk[ends] = False
    ys, xs = np.nonzero(sk)
    pts = set(zip(ys.tolist(), xs.tolist()))

    # Build skeleton graph to find the primary closed circuit loop
    # スケルトングラフを構築し、最長の閉路（メインコース）を探索
    path = None
    try:
        import networkx as nx
        G = nx.Graph()
        for y, x in pts:
            G.add_node((y, x))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if (dy or dx) and (y + dy, x + dx) in pts:
                        G.add_edge((y, x), (y + dy, x + dx), weight=math.hypot(dy, dx))
        cycles = sorted(nx.cycle_basis(G), key=len, reverse=True)
        if cycles and len(cycles[0]) > 50:
            path = cycles[0]
    except Exception as exc:
        print("networkx cycle search fallback: %s" % exc)

    if path is None:
        cur = (ys[0], xs[0])
        path, seen = [cur], {cur}
        while True:                                           # walk the loop / ループを辿る
            nxt = None
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    q = (cur[0] + dy, cur[1] + dx)
                    if q != cur and q in pts and q not in seen and (nxt is None or dy == 0 or dx == 0):
                        nxt = q
            if nxt is None:
                break
            path.append(nxt)
            seen.add(nxt)
            cur = nxt
        if len(path) < 0.95 * len(pts):
            raise SystemExit("the track surface is not one clean loop (%d of %d skeleton px walked)"
                             % (len(path), len(pts)))
    p = np.array(path, float)
    X, Y = x0 + p[:, 1] * g, y0 + p[:, 0] * g
    # Subsample dense skeletons before spline fitting for smoothness / スプライン平滑化用に適度に間引き
    step = max(1, len(X) // 3000)
    X_sub, Y_sub = X[::step], Y[::step]
    tck, _ = splprep([X_sub, Y_sub], s=len(X_sub) * 0.6, per=1)
    n_samples = int(np.sum(np.hypot(np.diff(np.append(X_sub, X_sub[0])), np.diff(np.append(Y_sub, Y_sub[0])))))
    xs, ys = splev(np.linspace(0, 1, n_samples, endpoint=False), tck)
    return np.c_[xs, ys]


def recentre(C, road):
    """Real edges across all the asphalt: true centre and width. / 実際の端から中心と幅。"""
    g = 0.25
    x0, y0 = C[:, 0].min() - 40, C[:, 1].min() - 40
    shape = (int((C[:, 1].max() + 40 - y0) / g), int((C[:, 0].max() + 40 - x0) / g))
    mask = closing(rasterise(road, x0, y0, g, shape), disk(4))
    d = np.roll(C, -1, 0) - np.roll(C, 1, 0)
    d /= np.linalg.norm(d, axis=1)[:, None]
    nl = np.c_[-d[:, 1], d[:, 0]]

    def reach(p, n):
        for k in range(1, 160):
            q = p + n * k * g
            i, j = int((q[1] - y0) / g), int((q[0] - x0) / g)
            if not (0 <= i < shape[0] and 0 <= j < shape[1]) or not mask[i, j]:
                return (k - 1) * g
        return 40.0
    L = np.minimum([reach(p, n) for p, n in zip(C, nl)], 12.0)
    R = np.minimum([reach(p, -n) for p, n in zip(C, nl)], 12.0)
    off = ndimage.uniform_filter1d(ndimage.median_filter((L - R) / 2, 31, mode="wrap"), 41, mode="wrap")
    wid = ndimage.uniform_filter1d(ndimage.median_filter(L + R, 31, mode="wrap"), 41, mode="wrap")
    return C + nl * off[:, None], wid


def main():
    cfg_path, key, out = sys.argv[1:4]
    cfgs = json.load(open(cfg_path))
    cfg = cfgs[key]
    os.makedirs(out, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        script = os.path.join(tmp, "export.py")
        open(script, "w").write(EXPORT)
    track_mats = [cfg["track_material"]] if isinstance(cfg["track_material"], str) else list(cfg["track_material"])
    road_mats = [cfg["road_materials"]] if isinstance(cfg["road_materials"], str) else list(cfg["road_materials"])
    mats = sorted(set(track_mats) | set(road_mats))
    with tempfile.TemporaryDirectory() as tmp:
        script = os.path.join(tmp, "export.py")
        open(script, "w").write(EXPORT)
        src_path = os.path.expanduser(cfg["source"])
        subprocess.run([os.environ.get("BLENDER", "blender"), "-b", "-P", script, "--",
                        src_path, os.path.join(tmp, "tris")] + mats,
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        tris = {m: np.load(os.path.join(tmp, "tris_%s.npy" % m)) for m in mats if os.path.exists(os.path.join(tmp, "tris_%s.npy" % m))}
    track = np.concatenate([tris[m] for m in track_mats if m in tris])
    C = centreline(track)
    C, wid = recentre(C, np.concatenate([tris[m] for m in road_mats if m in tris]))
    V = track.reshape(-1, 3)
    z = LinearNDInterpolator(V[:, :2], V[:, 2])(C)
    z = ndimage.uniform_filter1d(np.where(np.isnan(z), np.nanmedian(z), z), 15, mode="wrap")
    tck, _ = splprep([C[:, 0], C[:, 1], z, wid], s=len(C) * 0.3, per=1)
    n = int(np.sum(np.hypot(np.diff(C[:, 0]), np.diff(C[:, 1]))))
    P = np.array(splev(np.linspace(0, 1, n, endpoint=False), tck)).T
    # race direction and start at the line / 走行方向とスタートライン始点
    sl, pole = np.array(cfg["start_line"]), np.array(cfg["pole"])
    i = int(np.argmin(np.hypot(*(P[:, :2] - sl).T)))
    j = int(np.argmin(np.hypot(*(P[:, :2] - pole).T)))
    if (i - j) % n > n // 2:                      # line must come right after the pole / ポールの直後が線
        P = P[::-1]
        i, j = n - 1 - i, n - 1 - j
    P = np.roll(P, -i, axis=0)
    j = (j - i) % n
    off = [-round(float((P[:, 0].min() + P[:, 0].max()) / 2), 2),
           -round(float((P[:, 1].min() + P[:, 1].max()) / 2), 2), -round(float(P[0, 2]), 2)]
    head = math.degrees(math.atan2(P[(j + 1) % n, 1] - P[j, 1], P[(j + 1) % n, 0] - P[j, 0]))
    cfg["start"] = {"x": round(float(pole[0] + off[0]), 2), "y": round(float(-(pole[1] + off[1])), 2),
                    "z": round(float(P[j, 2] + off[2] + 0.5), 2), "yaw": round(-head, 1)}
    json.dump({"offset": off, "points": [[round(float(v), 3) for v in p] for p in P]},
              open(os.path.join(out, "track.json"), "w"))
    json.dump(cfgs, open(cfg_path, "w"), indent=2)
    print("lap %.0f m, width %.1f-%.1f m (median %.1f), start %s"
          % (n, P[:, 3].min(), P[:, 3].max(), np.median(P[:, 3]), cfg["start"]))


if __name__ == "__main__":
    main()
