"""Top-down surface grid of a built map: which ground is free (plain lawn)
and which is taken (roads, markings, kerbs, pavements, house lots, courts,
pitches, car parks, building and landmark footprints, street furniture, tree
trunks). The delivery sim uses it to put charging stations only on free
ground (delivery/charge_sites.py); the road logic alone cannot tell a lawn
from a pavement or a car park.

  maps/.venv/bin/python osm_city/surface_grid.py build/<Name> [--res 0.5]

Reads build/<Name>/meshes/*.npz (per-material V/I arrays, map frame: x east,
y north) and writes build/<Name>/surface_grid.npz:
  free   uint8 [rows, cols]  1 = plain lawn, nothing else on it (taken cells
         grown by one cell)
  kind   uint8 [rows, cols]  what covers the cell (KINDS index; 0 = nothing)
  x0, y0, res               map-frame origin of cell (0, 0) (its south-west
                            corner) and the cell size; row = (y - y0) / res
CARLA's frame is x east, y SOUTH: look a CARLA point up at (x, -y).

地図の地表グリッド（芝だけの空き地か、道路・歩道・区画・コート等で塞がるか）。
"""

import argparse
import glob
import os

import numpy as np
from PIL import Image, ImageDraw

# what covers a cell (later entries win where they overlap) / セルを覆うもの
KINDS = ["none", "lawn", "lot", "court", "site", "walk", "kerb", "road", "marking", "street",
         "tree", "building"]
FREE_MATS = ("grass", "grass_park")


def kind_of(obj_kind, mat):
    """KINDS name for a mesh object kind + material (None: ignore it)."""
    if any(s in obj_kind for s in ("Leaves", "JpPlates", "JpHokousha")):
        return None                           # crowns and small signs: the raycast sees them / 樹冠は光線で確認
    if obj_kind == "Road_Grass":
        return "lawn" if mat in FREE_MATS else "lot"
    if obj_kind == "Road_Road":
        return "road"
    if obj_kind == "Road_Marking":
        return "marking"
    if obj_kind == "Road_Curb":
        return "kerb"
    if obj_kind == "Road_Sidewalk":
        return "walk"
    if obj_kind == "Terrain_Lots":
        return "lot"
    if obj_kind == "Terrain_Courts":
        return "court"
    if obj_kind == "Terrain_Sites":
        return "site"
    if obj_kind == "Terrain_Street":
        return "street"
    if obj_kind == "Terrain_Trees":
        return "tree"
    return "building"                         # SM_Blocks, SM_<landmark>, Terrain_<landmark> / 建物・ランドマーク


def objects(meshes_dir):
    """(object kind, material, V, I) for every material block of every mesh file."""
    for path in sorted(glob.glob(os.path.join(meshes_dir, "*.npz"))):
        base = os.path.basename(path)[:-4]
        okind = base.rsplit("_", 2)[0] if base.count("_") >= 2 else base
        okind = "_".join(okind.split("_")[:2]) if okind.startswith(("Road_", "Terrain_", "SM_")) else okind
        d = np.load(path)
        for key in d.files:
            if key.endswith("|V"):
                mat = key[:-2]
                yield okind, mat, d[key], d[mat + "|I"]


def build(bd, res=0.5):
    items = list(objects(os.path.join(bd, "meshes")))
    # the grid covers the road network + 40 m (the lawn skirt beyond the ring
    # cannot be reached) / 道路網 + 40 m の範囲（外周の芝は対象外）
    roads = [v for k, _, v, _ in items if k == "Road_Road"] or [v for _, _, v, _ in items]
    xs = np.concatenate([v[:, 0] for v in roads])
    ys = np.concatenate([v[:, 1] for v in roads])
    x0, y0 = float(np.floor(xs.min())) - 40.0, float(np.floor(ys.min())) - 40.0
    cols = int(np.ceil((xs.max() + 40.0 - x0) / res))
    rows = int(np.ceil((ys.max() + 40.0 - y0) / res))
    img = Image.new("L", (cols, rows), 0)
    draw = ImageDraw.Draw(img)
    order = {k: i for i, k in enumerate(KINDS)}
    groups = {}
    for okind, mat, V, I in items:
        k = kind_of(okind, mat)
        if k is not None:
            groups.setdefault(k, []).append((V, I))
    counts = {}
    for k in sorted(groups, key=lambda k: order[k]):          # later kinds paint over / 後の種類が上書き
        n = 0
        for V, I in groups[k]:
            px = (V[:, 0] - x0) / res
            py = (V[:, 1] - y0) / res
            for a, b, c in I:
                draw.polygon([(px[a], py[a]), (px[b], py[b]), (px[c], py[c])], fill=order[k])
                n += 1
        counts[k] = n
    kind = np.array(img, dtype=np.uint8)
    # poles and tree trunks are thinner than a cell and may not fill one:
    # every cell a vertex of theirs falls in is taken / 細いポールや幹は頂点のセルも塞ぐ
    for k in ("street", "tree"):
        for V, _ in groups.get(k, ()):
            c = ((V[:, 0] - x0) / res).astype(int)
            r = ((V[:, 1] - y0) / res).astype(int)
            ok = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
            r, c = r[ok], c[ok]
            kind[r, c] = np.maximum(kind[r, c], order[k])
    taken = kind > order["lawn"]
    grown = taken.copy()                       # one cell of margin round anything taken / 1 セルの余白
    grown[1:, :] |= taken[:-1, :]
    grown[:-1, :] |= taken[1:, :]
    grown[:, 1:] |= taken[:, :-1]
    grown[:, :-1] |= taken[:, 1:]
    free = ((kind == order["lawn"]) & ~grown).astype(np.uint8)
    out = os.path.join(bd, "surface_grid.npz")
    np.savez_compressed(out, free=free, kind=kind, x0=x0, y0=y0, res=res, kinds=np.array(KINDS))
    return out, kind.shape, counts, float(free.mean())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("build_dir")
    ap.add_argument("--res", type=float, default=0.5)
    a = ap.parse_args()
    out, shape, counts, share = build(a.build_dir, a.res)
    print("surface grid %s: %d x %d cells of %.2f m, free (plain lawn) %.1f %%" % (
        out, shape[1], shape[0], a.res, 100 * share))
    print("  triangles per kind: " + ", ".join("%s %d" % kv for kv in sorted(counts.items())))


if __name__ == "__main__":
    main()
