#!/usr/bin/env python3
"""Pavement connectivity check of a built map, offline (maps/.venv), guide ch. 31.

  check_paths.py <build dir> [--res 0.25] [--png out.png]

Rasterises what the walker navmesh will call walkable from the BUILT meshes:
Road_Sidewalk up-facing triangles (pavements, footways, 路側帯 strips), plus the
crosswalk bands and sidewalk aprons of nav_areas.json (pavements.py; without it
the crossings.pkl bands), and reports the connected pieces (walkers only route
inside one), the pieces under 50 m2, and every crossing whose two ends do not
both touch a walkable surface. Exit 1 if a crossing end is off the network.

歩行面（歩道・路側帯・横断帯）の連結成分と横断歩道の両端を調べる。
"""
import argparse
import json
import os
import pickle
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
from shapely import wkt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build")
    ap.add_argument("--res", type=float, default=0.25)
    ap.add_argument("--png")
    a = ap.parse_args()
    bd, res = a.build, a.res
    meta = json.load(open(os.path.join(bd, "meta.json")))
    hx, hy = meta["half_x"] + 5, meta["half_y"] + 5
    x0, y0 = -hx, -hy
    W, H = int(2 * hx / res), int(2 * hy / res)
    px = lambda x, y: ((x - x0) / res, (y - y0) / res)

    def raster_tris(fams):
        img = Image.new("1", (W, H), 0)
        dr = ImageDraw.Draw(img)
        for fn in sorted(os.listdir(os.path.join(bd, "meshes"))):
            if not fn.endswith(".npz") or fn.rsplit("_", 2)[0] not in fams:
                continue
            z = np.load(os.path.join(bd, "meshes", fn))
            for k in z.files:
                if not k.endswith("|I"):
                    continue
                V, I = z[k[:-2] + "|V"], z[k]
                T = V[I]
                n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
                up = n[:, 2] > 0.3 * np.linalg.norm(n, axis=1).clip(1e-9)
                for t in T[up]:
                    dr.polygon([px(t[i, 0], t[i, 1]) for i in range(3)], fill=1)
        return np.array(img, dtype=bool)

    def raster_polys(geoms):
        img = Image.new("1", (W, H), 0)
        dr = ImageDraw.Draw(img)
        for g in geoms:
            for p in getattr(g, "geoms", [g]):
                if p.geom_type != "Polygon" or p.is_empty:
                    continue
                dr.polygon([px(x, y) for x, y in p.exterior.coords], fill=1)
                for r in p.interiors:
                    dr.polygon([px(x, y) for x, y in r.coords], fill=0)
        return np.array(img, dtype=bool)

    side = raster_tris(("Road_Sidewalk",))
    extra = pickle.load(open(os.path.join(bd, "crossings.pkl"), "rb"))
    cr = extra.get("crossings", [])
    nf = os.path.join(bd, "nav_areas.json")
    if os.path.exists(nf):
        A = json.load(open(nf))
        cross = raster_polys([wkt.loads(h) for h in A.get("crosswalk", [])])
        apron = raster_polys([wkt.loads(h) for h in A.get("sidewalk", [])])
    else:
        cross = ndimage.binary_dilation(raster_polys([c["band"] for c in cr]), iterations=2)
        apron = np.zeros_like(side)
    walk = side | cross | apron
    lab, n = ndimage.label(walk, structure=np.ones((3, 3)))
    sizes = ndimage.sum(walk, lab, range(1, n + 1)) * res * res
    order = np.argsort(sizes)[::-1]
    print("walkable %.0f m2 (sidewalk meshes %.0f m2), crossings %d" % (walk.sum() * res * res, side.sum() * res * res,
                                                                       len(cr)))
    print("connected pieces: %d (largest %s m2)" % (n, [int(sizes[i]) for i in order[:6]]))
    print("pieces < 50 m2: %d, 50-500 m2: %d, >= 500 m2: %d" % ((sizes < 50).sum(), ((sizes >= 50) & (sizes < 500)).sum(),
                                                               (sizes >= 500).sum()))
    small = [i for i in order if sizes[i] < 500]
    if small:
        cm = ndimage.center_of_mass(walk, lab, [i + 1 for i in small])
        print("pieces < 500 m2 (m2 at CARLA x,y):", [(int(sizes[i]), round(c[1] * res + x0), round(-(c[0] * res + y0)))
                                                     for i, c in zip(small, cm)][:30])
    bad = 0
    walk_s = side | apron
    for c in cr:
        cc, nn = np.array(c["c"]), np.array(c["n"])
        for t, sg in ((c["t0"], -1), (c["t1"], 1)):
            hit = False
            # (just inside the kerb too: a 路側帯 strip lies inside the carriageway) / 路側帯は車道内
            for d in (-0.5, -0.3, 0.3, 0.5, 0.7, 0.9):
                q = cc + nn * (t + sg * d)
                i, j = int((q[1] - y0) / res), int((q[0] - x0) / res)
                if 0 <= i < H and 0 <= j < W and walk_s[max(0, i - 1):i + 2, max(0, j - 1):j + 2].any():
                    hit = True
                    break
            if not hit:
                bad += 1
                q = cc + nn * t
                print("  crossing end off the network: CARLA (%.1f, %.1f) %s" % (q[0], -q[1], c.get("src", "")))
    print("crossing ends off the network: %d of %d" % (bad, 2 * len(cr)))
    if a.png:
        rgb = np.zeros((H, W, 3), np.uint8) + np.array([40, 70, 40], np.uint8)
        big = lab == (order[0] + 1)
        rgb[walk] = (255, 60, 60)
        rgb[big] = (240, 240, 240)
        Image.fromarray(rgb[::-1]).resize((W // 4, H // 4)).save(a.png)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
