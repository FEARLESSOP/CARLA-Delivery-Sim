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

maps.json "gaito_lit" (opt-in per map, by meta.json's name; maps without it light
every head as before): only heads within ``reach_m`` of a non-junction lane whose
OSM way (xodr sumoId -> <Name>.ring.osm) has one of ``highways`` (or the Ring Road
with ``ring``), at least ``spacing_m`` apart (grown 10 % at a time until <= ``max``),
get a light; the rest stay unlit lamp meshes (PasirRisTampines: 13.6 k heads).

街灯の灯具位置をメッシュから取り出す（lamp_head の 0.7 x 0.3 m 上面）。
maps.json "gaito_lit": 幹線沿いの灯具のみ点灯（間隔・上限付き）、他は消灯のまま。
"""

import collections
import glob
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

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


def lit_options(build):
    """maps.json "gaito_lit" of this build's map (first entry of its name), or None.
    / この地図の点灯条件。"""
    try:
        name = json.load(open(os.path.join(build, "meta.json")))["name"]
        maps = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json")))
    except (OSError, ValueError, KeyError):
        return None
    for v in maps.values():
        if isinstance(v, dict) and v.get("name") == name:
            return v.get("gaito_lit")
    return None


def main_lanes(build, opt):
    """CARLA-frame (x, y) samples of the non-junction lanes whose OSM way is in
    opt["highways"] (or named Ring Road), and of all other non-junction lanes.
    / 幹線の車線点と、その他の車線点（CARLA 座標）。"""
    name = json.load(open(os.path.join(build, "meta.json")))["name"]
    hw = {}
    for _, el in ET.iterparse(os.path.join(build, name + ".ring.osm")):
        if el.tag == "way":
            t = {c.get("k"): c.get("v") for c in el.findall("tag")}
            if "highway" in t:
                hw[el.get("id")] = t["highway"]
            el.clear()
        elif el.tag == "node":
            el.clear()
    way = {}
    for r in ET.parse(os.path.join(build, name + ".xodr")).getroot().findall("road"):
        u = r.find("userData[@code='sumoId']")
        if u is not None:
            way[int(r.get("id"))] = u.get("value").lstrip("-").split("#")[0]
    keep = set(opt.get("highways", ()))
    main, other = [], []
    for l in json.load(open(os.path.join(build, "lanes.json")))["lanes"]:
        if l.get("junction", -1) != -1:
            continue
        lit = hw.get(way.get(l["road"], ""), "") in keep or (opt.get("ring") and l.get("name") == "Ring Road")
        (main if lit else other).extend((p["c"][0], -p["c"][1]) for p in l["pts"])
    return np.array(main, float).reshape(-1, 2), np.array(other, float).reshape(-1, 2)


def spaced(P, d):
    """Indices of a greedy subset of points P at least d apart (grid hash). / 間隔 d 以上の部分集合。"""
    grid, out = {}, []
    for i, (x, y) in enumerate(P):
        gx, gy = int(math.floor(x / d)), int(math.floor(y / d))
        if any((x - P[j][0]) ** 2 + (y - P[j][1]) ** 2 < d * d
               for a in (-1, 0, 1) for b in (-1, 0, 1) for j in grid.get((gx + a, gy + b), ())):
            continue
        grid.setdefault((gx, gy), []).append(i)
        out.append(i)
    return out


def lit_heads(build, rows, opt):
    """The heads that get a light under opt (see the module doc). / 点灯する灯具。"""
    from scipy.spatial import cKDTree
    main, other = main_lanes(build, opt)
    if not len(rows) or not len(main):
        return []
    H = np.array([[r[0], r[1]] for r in rows])
    dm, _ = cKDTree(main).query(H)
    do = cKDTree(other).query(H)[0] if len(other) else np.full(len(H), np.inf)
    reach = float(opt.get("reach_m", 12.0))
    cand = [i for i in np.lexsort((H[:, 1], H[:, 0])) if dm[i] <= reach and dm[i] <= do[i]]
    d, cap = float(opt.get("spacing_m", 30.0)), int(opt.get("max", 3000))
    while True:
        pick = spaced(H[cand], d)
        if len(pick) <= cap:
            break
        d *= 1.1
    print("[gaito] lit: %d candidates by main roads, spacing %.1f m -> %d" % (len(cand), d, len(pick)))
    return [rows[cand[i]] for i in pick]


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
    opt = lit_options(build)
    if opt:
        # maps.json "gaito_lit": lights only by the main roads / 幹線沿いのみ点灯
        n_all = len(rows)
        rows = lit_heads(build, rows, opt)
        json.dump({"heads": rows, "frame": "carla", "note": "x, y, z of the head's underside (m), yaw (deg)",
                   "all_heads": n_all, "unlit": n_all - len(rows)},
                  open(os.path.join(build, "gaito.json"), "w"))
        print("[gaito] %d lamp heads lit of %d -> %s" % (len(rows), n_all, os.path.join(build, "gaito.json")))
        return
    json.dump({"heads": rows, "frame": "carla", "note": "x, y, z of the head's underside (m), yaw (deg)"},
              open(os.path.join(build, "gaito.json"), "w"))
    print("[gaito] %d lamp heads -> %s" % (len(rows), os.path.join(build, "gaito.json")))


if __name__ == "__main__":
    main()
