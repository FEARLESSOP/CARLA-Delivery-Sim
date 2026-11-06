"""Street-lamp light positions (街灯) from a built map's meshes.

  maps/.venv/bin/python osm_city/gaito.py <build dir>     -> <build dir>/gaito.json

props.m_lamp gives every lamp a 0.7 x 0.3 x 0.2 m ``lamp_head`` box at the
end of its arm. This finds each head in the Terrain_Street meshes (the
0.7 x 0.3 m top faces of that material, vertices welded to the mm; the
meshes have no shared vertices) so the lights can be placed without
rebuilding the map. Output, in CARLA's frame (x east, y south, metres):
``{"heads": [[x, y, z_bottom, yaw_deg], ...]}`` with yaw along the arm.
build_map.sh step 2g spawns BP_JpGaito (CARLA's street-light BP with our
glowing plate, guide ch. 28) at each.

街灯の灯具位置をメッシュから取り出す（lamp_head の 0.7 x 0.3 m 上面）。
"""

import collections
import glob
import json
import math
import os
import sys

import numpy as np

HEAD_L, HEAD_W, HEAD_H = 0.7, 0.3, 0.2      # props.m_lamp's head box / 灯具の箱


def heads_in(V, N, I, tol=0.02):
    """[(centre_top (map frame), long-axis angle)] of the lamp-head tops in one
    mesh part. / 1 メッシュ中の灯具上面"""
    top = I[N[I[:, 0], 2] > 0.99]
    keys = np.round(V * 1000.0).astype(np.int64)
    par = {}

    def find(a):
        par.setdefault(a, a)
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a
    for t in top:
        ks = [tuple(keys[i]) for i in t]
        r = find(ks[0])
        for k in ks[1:]:
            par[find(k)] = r
    groups = collections.defaultdict(set)
    for t in top:
        for i in t:
            groups[find(tuple(keys[i]))].add(tuple(keys[i]))
    out = []
    for ks in groups.values():
        if len(ks) != 4:
            continue
        P = np.array(sorted(ks), dtype=float) / 1000.0
        d = np.linalg.norm(P[1:] - P[0], axis=1)
        o = np.argsort(d)
        if abs(d[o[0]] - HEAD_W) < tol and abs(d[o[1]] - HEAD_L) < tol:
            q = P[1:][o[1]] - P[0]
            out.append((P.mean(axis=0), math.atan2(q[1], q[0])))
    return out


def find_heads(build):
    heads = []
    for f in sorted(glob.glob(os.path.join(build, "meshes", "Terrain_Street_*.npz"))):
        d = np.load(f)
        if "lamp_head|V" in d.files:
            heads += heads_in(d["lamp_head|V"], d["lamp_head|N"], d["lamp_head|I"])
    rows = []
    for c, ang in heads:
        yaw = (-math.degrees(ang)) % 180.0           # map -> CARLA (y flipped); a box has no front
        rows.append([round(float(c[0]), 3), round(-float(c[1]), 3), round(float(c[2]) - HEAD_H, 3), round(yaw, 2)])
    return rows


def self_test():
    # one head, rotated 30 deg, top at z 10: two triangles, unshared vertices
    a = math.radians(30.0)
    ux, uy = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    c = np.array([5.0, 7.0])
    corners = [c + sx * HEAD_L / 2 * ux + sy * HEAD_W / 2 * uy for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    V = np.array([[*corners[i], 10.0] for i in (0, 1, 2, 0, 2, 3)] + [[0, 0, 9.8]] * 3)
    N = np.array([[0, 0, 1.0]] * 6 + [[1.0, 0, 0]] * 3)
    I = np.arange(9).reshape(3, 3)
    h = heads_in(V, N, I)
    assert len(h) == 1 and np.allclose(h[0][0], [5, 7, 10]) and abs(math.degrees(h[0][1]) % 180 - 30) < 0.01, h
    print("gaito self-test OK / 自己テスト OK")


def main():
    if sys.argv[1:] == ["--test"]:
        self_test()
        return
    build = sys.argv[1]
    rows = find_heads(build)
    json.dump({"heads": rows, "frame": "carla", "note": "x, y, z of the head's underside (m), yaw (deg)"},
              open(os.path.join(build, "gaito.json"), "w"))
    print("[gaito] %d lamp heads -> %s" % (len(rows), os.path.join(build, "gaito.json")))


if __name__ == "__main__":
    main()
