"""Height map of a built map for the pretend world: ground + everything on it.

Rasterises the map builder's own meshes (maps/build/<Name>/meshes/*.npz) on a
2 m grid in CARLA coordinates (x east, y south):
- ground: the highest walkable/drivable surface (roads, kerbs, pavements,
  grass, lots, courts, landmark grounds) per cell;
- top: the highest solid thing per cell (buildings, tree trunks and canopies,
  poles, lamps, signals, sign plates, fences), never lower than the ground.
A drone below ``top`` in a cell has hit something. Triangles are sampled every
<= 0.7 m so roofs and canopies fill their whole area.

  python3 -m rl_delivery.prepare_heights --map OitaKosen

建物・樹木・柱などの高さ地図（2 m 格子、CARLA 座標）。ドローンの衝突判定に使う。
"""
import argparse
import glob
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
CELL = 2.0
GROUND = ("Road_Road", "Road_Curb", "Road_Sidewalk", "Road_Grass", "Road_Marking", "Terrain_Lots",
          "Terrain_Courts", "Terrain_Sites", "Terrain_KOSEN", "Terrain_AKENO", "Terrain_DORM", "Terrain_SEVEN")
SOLID = ("SM_", "Terrain_Trees", "Leaves", "Terrain_Street", "Terrain_JpPlates", "Terrain_JpHokousha")


def tri_points(V, I, spacing=0.7):
    """Points covering every triangle (vertices + barycentric samples). / 三角形を点で覆う"""
    T = V[I.reshape(-1, 3)]
    e = np.maximum(np.linalg.norm(T[:, 1] - T[:, 0], axis=1), np.linalg.norm(T[:, 2] - T[:, 0], axis=1))
    e = np.maximum(e, np.linalg.norm(T[:, 2] - T[:, 1], axis=1))
    n = np.clip(np.ceil(e / spacing).astype(int), 1, 200)
    out = [V]
    for k in np.unique(n):
        if k <= 1:
            continue
        sel = T[n == k]
        a, b = np.meshgrid(np.arange(k + 1), np.arange(k + 1))
        m = a + b <= k
        u, v = a[m] / k, b[m] / k
        w = 1.0 - u - v
        P = (sel[:, None, 0] * w[None, :, None] + sel[:, None, 1] * u[None, :, None]
             + sel[:, None, 2] * v[None, :, None])
        out.append(P.reshape(-1, 3))
    return np.concatenate(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="OitaKosen")
    a = ap.parse_args()
    mdir = os.path.join(HIL, "maps", "build", a.map, "meshes")
    with open(os.path.join(HERE, "maps", a.map + ".json")) as f:
        bounds = json.load(f)["bounds"]
    x0, y0 = bounds[0] - 60, bounds[1] - 60
    nx_, ny_ = int((bounds[2] + 60 - x0) / CELL) + 1, int((bounds[3] + 60 - y0) / CELL) + 1
    ground = np.full((nx_, ny_), -1e3, np.float32)
    top = np.full((nx_, ny_), -1e3, np.float32)
    counts = {"ground": 0, "solid": 0}
    for path in sorted(glob.glob(os.path.join(mdir, "*.npz"))):
        name = os.path.basename(path)
        is_g = name.startswith(GROUND)
        is_s = name.startswith("SM_") or any(k in name for k in SOLID[1:])
        if not (is_g or is_s):
            continue
        d = np.load(path)
        for key in d.files:
            if not key.endswith("|V"):
                continue
            V = d[key].astype(np.float32)
            I = d[key[:-2] + "|I"]
            P = tri_points(V, I)
            ix = ((P[:, 0] - x0) / CELL).astype(int)
            iy = ((-P[:, 1] - y0) / CELL).astype(int)          # mesh y is north / メッシュの y は北向き
            ok = (ix >= 0) & (ix < nx_) & (iy >= 0) & (iy < ny_)
            grid = ground if is_g else top
            np.maximum.at(grid, (ix[ok], iy[ok]), P[ok, 2])
            counts["ground" if is_g else "solid"] += int(ok.sum())
    # fill ground gaps from neighbours, then top >= ground / 地面の穴を埋める
    gmask = ground > -1e2
    while (~gmask).any():
        g2 = ground.copy()
        for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            sh = np.roll(np.roll(ground, sx, 0), sy, 1)
            m2 = np.roll(np.roll(gmask, sx, 0), sy, 1)
            fill = ~gmask & m2
            g2[fill] = sh[fill]
        newmask = g2 > -1e2
        if newmask.sum() == gmask.sum():
            break
        ground, gmask = g2, newmask
    top = np.maximum(top, ground)
    out = os.path.join(HERE, "maps", a.map + "_heights.npz")
    np.savez_compressed(out, x0=x0, y0=y0, cell=CELL, ground=ground, top=top)
    above = top - ground
    print("%s: %dx%d cells (%.0f m), %d ground / %d solid samples, %.1f %% of cells have something > 2 m, "
          "tallest %.1f m above ground -> %s" % (a.map, nx_, ny_, CELL, counts["ground"], counts["solid"],
                                                100 * float((above > 2).mean()), float(above.max()), out))


if __name__ == "__main__":
    main()
