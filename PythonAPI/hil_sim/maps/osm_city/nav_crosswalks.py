"""Navmesh areas over the road for nav_obj.py (guide ch. 7, 31).
  python nav_crosswalks.py mesh_dir out.npz

With <build>/nav_areas.json (pavements.py, Japanese maps): the crossing bands
widened over the kerb (crosswalk), the dropped-kerb aprons across estate junction
mouths and the kerb connectors (sidewalk), and the park / playground lawns that
stay grass; faces on the built road surface (z from the nearest road vertex).
Otherwise: crosswalk hulls from the zebra paint (line_white pieces ~0.45 x 4 m,
grouped, hulled), as before.
npz: V, I, hulls (crosswalk faces), SV, SI (sidewalk faces), lawns (wkt list).
横断歩道・交差点口の帯・芝生（公園）をナビメッシュ用に作る。"""
import glob, json, os, sys
import numpy as np
import shapely
from shapely import wkt
from shapely.geometry import Polygon, MultiPoint
from shapely.ops import unary_union
from shapely.geometry.polygon import orient
from shapely import STRtree
mesh_dir, out = sys.argv[1], sys.argv[2]
areas_f = os.path.join(os.path.dirname(os.path.abspath(mesh_dir.rstrip("/"))), "nav_areas.json")


def surface(fams, mats=None):
    """Vertices of the built surfaces (z source). / 高さの元になる頂点。"""
    V = []
    for f in glob.glob(os.path.join(mesh_dir, "*.npz")):
        if os.path.basename(f).rsplit("_", 2)[0] not in fams:
            continue
        d = np.load(f)
        V += [d[k] for k in d.files if k.endswith("|V") and (mats is None or k[:-2] in mats)]
    return np.concatenate(V) if V else np.zeros((0, 3))


def faces(polys, V, tree, lift):
    """Constrained-Delaunay faces of polygons, z = nearest surface vertex + lift. / 面を作る。"""
    FV, FI, off = [], [], 0
    for p in polys:
        for q in getattr(p, "geoms", [p]):
            if q.geom_type != "Polygon" or q.area < 0.05:
                continue
            q = shapely.segmentize(q, 1.0)
            tris = shapely.get_parts(shapely.constrained_delaunay_triangles(q))
            if not len(tris):
                continue
            C = shapely.get_coordinates(shapely.get_exterior_ring(tris)).reshape(-1, 4, 2)[:, :3]
            a, b, c = C[:, 0], C[:, 1], C[:, 2]
            cr = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
            C = C[np.abs(cr) > 1e-9]
            cr = cr[np.abs(cr) > 1e-9]
            C[cr < 0] = C[cr < 0][:, ::-1]                       # ccw from above / 上から反時計回り
            P = C.reshape(-1, 2)
            idx = tree.query_nearest(shapely.points(P[:, 0], P[:, 1]), all_matches=False)[1]
            FV.append(np.c_[P, V[idx, 2] + lift])
            FI.append(np.arange(len(P)).reshape(-1, 3) + off)
            off += len(P)
    if not FV:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    return np.concatenate(FV), np.concatenate(FI)


if os.path.exists(areas_f):
    A = json.load(open(areas_f))
    V = surface(("Road_Road", "Road_Sidewalk"))
    tree = STRtree(shapely.points(V[:, 0], V[:, 1]))
    cw = [wkt.loads(h) for h in A.get("crosswalk", [])]
    sw = [wkt.loads(h) for h in A.get("sidewalk", [])]
    HV, HI = faces(cw, V, tree, 0.005)
    SV, SI = faces(sw, V, tree, -0.02)          # (just under: a kerb top beside it is still marked)
    np.savez(out, V=HV, I=HI, hulls=np.array([h.wkt for h in cw]), SV=SV, SI=SI,
             lawns=np.array(A.get("lawns", [])))
    print(len(cw), "crosswalks", len(sw), "sidewalk aprons", len(A.get("lawns", [])), "lawns (nav_areas.json)")
    sys.exit(0)

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
