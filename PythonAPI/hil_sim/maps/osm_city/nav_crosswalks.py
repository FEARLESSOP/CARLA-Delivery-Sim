"""Crosswalk areas from the zebra paint: line_white pieces ~0.45 x 4 m, grouped, hulled.
  python nav_crosswalks.py mesh_dir out.npz
横断歩道の範囲を白線（ゼブラ）から作る。"""
import glob, os, sys
import numpy as np
import shapely
from shapely.geometry import Polygon, MultiPoint
from shapely.ops import unary_union
from shapely.geometry.polygon import orient
from shapely import STRtree
mesh_dir, out = sys.argv[1], sys.argv[2]
tris, zs = [], []
for f in glob.glob(os.path.join(mesh_dir, "Road_Marking*.npz")):
    d = np.load(f)
    if "line_white|V" not in d.files:
        continue
    V, I = d["line_white|V"], d["line_white|I"]
    for t in V[I]:
        p = Polygon(t[:, :2])
        if p.area > 1e-4:
            tris.append(p.buffer(0.01))
    zs.append(V)
pieces = list(getattr(u := unary_union(tris), "geoms", [u]))
bars = []
for p in pieces:
    r = p.minimum_rotated_rectangle
    c = np.array(r.exterior.coords)[:4]
    a, b = np.linalg.norm(c[1] - c[0]), np.linalg.norm(c[2] - c[1])
    lo, hi = min(a, b), max(a, b)
    if 0.3 < lo < 0.6 and 2.5 < hi < 4.5 and p.area > 0.7 * r.area:
        bars.append(p)
groups = list(getattr(g := unary_union([b.buffer(0.35) for b in bars]), "geoms", [g]))
V = np.concatenate(zs)
tree = STRtree([shapely.Point(v[:2]) for v in V])
hulls = []
for g in groups:
    inside = [b for b in bars if b.intersects(g)]
    if len(inside) < 3:
        continue
    h = orient(MultiPoint([q for b in inside for q in b.exterior.coords]).convex_hull.buffer(0.2, join_style=2), 1.0)
    hulls.append(h)
HV, HI, off = [], [], 0
for h in hulls:
    ring = np.array(h.exterior.coords)[:-1]
    z = []
    for q in ring:
        i = tree.nearest(shapely.Point(q))
        z.append(V[i, 2] + 0.005)
    HV.append(np.c_[ring, z])
    c = np.r_[ring.mean(0), np.mean(z)]
    HV.append(c[None])
    n = len(ring)
    HI += [[off + n, off + k, off + (k + 1) % n] for k in range(n)]   # fan, ccw from above
    off += n + 1
np.savez(out, V=np.concatenate(HV), I=np.array(HI), hulls=np.array([h.wkt for h in hulls]))
print(len(bars), "bars", len(hulls), "crosswalks")
