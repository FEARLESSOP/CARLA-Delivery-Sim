"""Terrain height for hilly maps: the smoothed ground field and the drape (maps/.venv).

  python terrain.py <build dir> [--glb terrain.glb] [--sigma 9] [--step 2]

A map has terrain when its build dir holds the terrain files (maps.json
field `terrain_glb` names the source):
  terrain_topo.npz    the ground TIN out of the GLB: V (topo_x, topo_y, h),
                      F (triangles), B (building centroids: x, y, hmin, hmax,
                      area) -- made here with Blender when missing
  terrain_align.json  topo = osm_from_SW_corner + osm_sw_to_topo (metres) --
                      made here (building centroids, KD-tree) when missing
and this script writes
  terrain_field.npz   H on a regular grid in the map frame (x east, y north,
                      metres from the OSM box centre), Gaussian-smoothed,
                      shifted so the lowest point inside the map is 0.

Flat maps have none of these files: `Terrain.load` returns None and every
caller keeps its flat path unchanged.

`Drape` (model build) gives the height of the ground at any point from the
ELEVATED ROAD LOGIC on and near the roads (lanes.json z, the same numbers
CARLA's waypoints use) blending to the terrain field 0.6-12.6 m from the
carriageway: road(x, y) inside the carriageway, terrain far from it.

地形の高さ場（平滑化）と、道路情報の標高に合わせて地面を沿わせる関数。
"""

import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np

FIELD = "terrain_field.npz"
TOPO = "terrain_topo.npz"
ALIGN = "terrain_align.json"
BLEND_IN, BLEND_W = 0.6, 12.0      # ground = road within 0.6 m of the carriageway, terrain 12 m further / 道路→地形
RES = 0.5                          # drape raster / 沿わせ用ラスタ
OFFSET_SIGMA, OFFSET_RAMP = 3.0, 2.4   # road offset: smooth average beyond 0.6 + 2.4 m / 道路差の平滑化


class Field(object):
    """A height grid with bilinear lookup, clamped at its edge. / 格子の高さ（双線形補間）。"""

    def __init__(self, x0, y0, step, Z):
        self.x0, self.y0, self.step = float(x0), float(y0), float(step)
        self.Z = np.asarray(Z, np.float64)

    def __call__(self, x, y):
        x = np.asarray(x, np.float64)
        y = np.asarray(y, np.float64)
        ny, nx = self.Z.shape
        fx = np.clip((x - self.x0) / self.step, 0, nx - 1.000001)
        fy = np.clip((y - self.y0) / self.step, 0, ny - 1.000001)
        i, j = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
        u, v = fx - i, fy - j
        Z = self.Z
        return ((Z[j, i] * (1 - u) + Z[j, i + 1] * u) * (1 - v) + (Z[j + 1, i] * (1 - u) + Z[j + 1, i + 1] * u) * v)

    def grad(self, x, y, h=1.0):
        return ((self(x + h, y) - self(x - h, y)) / (2 * h), (self(x, y + h) - self(x, y - h)) / (2 * h))


class Terrain(Field):
    @staticmethod
    def load(out_dir):
        p = os.path.join(out_dir, FIELD)
        if not os.path.exists(p):
            return None
        d = np.load(p)
        t = Terrain(d["x0"], d["y0"], d["step"], d["Z"])
        t.z0 = float(d["z0"])
        return t


# -- making the field / 高さ場を作る
BLENDER_DUMP = r'''
import bpy, sys, numpy as np
src, out = sys.argv[sys.argv.index("--") + 1:][:2]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=src)
g = [o for o in bpy.context.scene.objects if o.type == "MESH" and o.name.startswith("TPX_Ground")][0]
mw = g.matrix_world
V = np.array([tuple(mw @ v.co) for v in g.data.vertices])
F = np.array([tuple(p.vertices) for p in g.data.polygons if len(p.vertices) == 3])
bc = []
for o in bpy.context.scene.objects:
    if o.type == "MESH" and o.name.startswith("TPX_Buildings_"):
        P = np.array([tuple(o.matrix_world @ v.co) for v in o.data.vertices])
        bc.append([P[:, 0].mean(), P[:, 2].mean(), -P[:, 1].min(), -P[:, 1].max(), np.ptp(P[:, 0]) * np.ptp(P[:, 2])])
# glTF Z-up export seen in Blender: topo x = x, topo y = z, height = -y / 高さは -y
np.savez(out, V=np.c_[V[:, 0], V[:, 2], -V[:, 1]], F=F, B=np.array(bc))
print("DUMP", V.shape, F.shape, len(bc))
'''


def topo_name(k):
    """terrain_topo.npz for the first export, terrain_topo_1.npz, ... for add-ons."""
    return TOPO if k == 0 else TOPO.replace(".npz", "_%d.npz" % k)


def dump_glb(glb, out_dir, k=0):
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
        fh.write(BLENDER_DUMP)
    try:
        subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", fh.name, "--",
                               glb, os.path.join(out_dir, topo_name(k))], stdout=subprocess.DEVNULL)
    finally:
        os.remove(fh.name)


def osm_building_centroids(osm_path, meta):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from osmdata import OSM
    o = OSM(osm_path, meta)
    return np.array([(p.centroid.x, p.centroid.y) for p, t in o.areas(lambda t: "building" in t)])


def guess_shift(meta, export_sw, margin=(11.84, 14.70)):
    """First guess of the shift from an export's SW corner (lat, lon of the
    box drawn on topoexport.com): topoexport puts its origin ``margin``
    metres west/south of the drawn box (measured on OitaKosen's main export).
    / エクスポートの南西角から位置合わせの初期値。"""
    import geo
    fr = geo.Frame(meta)
    ex, ey = fr.xy(export_sw[1], export_sw[0])
    return np.array([-(ex + meta["half_x"]) + margin[0], -(ey + meta["half_y"]) + margin[1]])


def align(topo_B, osm_c, meta, search=60.0, centre=(0.0, 0.0)):
    """Shift topo = osm_from_SW + s: grid search on building centroids around
    ``centre`` (+-search), then a least-squares refinement on the pairs within
    2 m. Search near a geographic guess (guess_shift): a wide blind search
    finds false matches on a regular estate grid. / 建物重心の照合で位置合わせ。"""
    from scipy.spatial import cKDTree
    A = osm_c + np.array([meta["half_x"], meta["half_y"]])
    T = cKDTree(topo_B[:, :2])
    best = (0, None)
    for sx in np.arange(centre[0] - search, centre[0] + search + 0.1, 1.0):
        for sy in np.arange(centre[1] - search, centre[1] + search + 0.1, 1.0):
            n = int((T.query(A + (sx, sy), distance_upper_bound=1.5)[0] < 1.5).sum())
            if n > best[0]:
                best = (n, np.array([sx, sy]))
    s = best[1]
    for _ in range(3):
        d, i = T.query(A + s, distance_upper_bound=2.0)
        ok = d < 2.0
        s = (topo_B[i[ok], :2] - A[ok]).mean(axis=0)
    return s, int(ok.sum())


def make_field(out_dir, osm_path=None, glb=None, sigma=9.0, step=2.0, export_sw=None):
    """glb: one path or a list (the main export first, add-ons after; each
    covers part of the map, e.g. a strip added later). Each export is aligned
    to OSM on its own; an add-on's heights are shifted to agree with the main
    export where they overlap. / 複数の地形エクスポートを位置合わせして統合。"""
    from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
    from scipy.ndimage import gaussian_filter
    meta = json.load(open(os.path.join(out_dir, "meta.json")))
    glbs = list(glb) if isinstance(glb, (list, tuple)) else ([glb] if glb else [])
    n = max(len(glbs), 1)
    while os.path.exists(os.path.join(out_dir, topo_name(n))):
        n += 1
    ap = os.path.join(out_dir, ALIGN)
    old = json.load(open(ap)) if os.path.exists(ap) else {}
    shifts = old.get("exports") or ([{"osm_sw_to_topo": old["osm_sw_to_topo"]}] if "osm_sw_to_topo" in old else [])
    osm_c = None
    Ps, Hs = [], []
    for k in range(n):
        if not os.path.exists(os.path.join(out_dir, topo_name(k))):
            if k >= len(glbs):
                sys.exit("no %s and no --glb for export %d" % (topo_name(k), k))
            dump_glb(glbs[k], out_dir, k)
        topo = np.load(os.path.join(out_dir, topo_name(k)))
        if k >= len(shifts):
            if not osm_path:
                sys.exit("no %s entry for export %d and no OSM to align against" % (ALIGN, k))
            if osm_c is None:
                osm_c = osm_building_centroids(osm_path, meta)
            sw = export_sw[k] if export_sw and k < len(export_sw) else None
            c0 = guess_shift(meta, sw) if sw else np.zeros(2)
            sft, pairs = align(topo["B"], osm_c, meta, search=20.0 if sw else 60.0, centre=c0)
            shifts.append({"osm_sw_to_topo": [round(float(sft[0]), 2), round(float(sft[1]), 2)], "pairs": pairs,
                           "guess": [round(float(c0[0]), 2), round(float(c0[1]), 2)] if sw else None,
                           "glb": glbs[k] if k < len(glbs) else None})
            print("terrain export %d aligned: shift %s (guess %s), %d building pairs within 2 m"
                  % (k, shifts[-1]["osm_sw_to_topo"], shifts[-1]["guess"], pairs))
        sh = np.array(shifts[k]["osm_sw_to_topo"], np.float64)
        V = topo["V"]
        # topo -> map frame: map = topo - shift - (half_x, half_y) / 地形座標 -> 地図座標
        P = V[:, :2] - sh - np.array([meta["half_x"], meta["half_y"]])
        H = V[:, 2].astype(np.float64)
        if k > 0:
            # vertical agreement with what is already there / 既存の地形と高さを合わせる
            prev = LinearNDInterpolator(np.concatenate(Ps), np.concatenate(Hs))(P[:, 0], P[:, 1])
            ok = ~np.isnan(prev)
            dz = float(np.median(prev[ok] - H[ok])) if ok.sum() > 20 else 0.0
            H = H + dz
            shifts[k]["dz"] = round(dz, 2)
            shifts[k]["overlap_vertices"] = int(ok.sum())
            # where both cover the ground, keep the main export / 重なりは主エクスポートを優先
            P, H = P[~ok], H[~ok]
            print("terrain add-on %d: shift %s, %d vertices overlap (dz %.2f m), %d added"
                  % (k, shifts[k]["osm_sw_to_topo"], int(ok.sum()), dz, len(H)))
        Ps.append(P)
        Hs.append(H)
    json.dump({"osm_sw_to_topo": shifts[0]["osm_sw_to_topo"], "exports": shifts,
               "method": "each GLB vs OSM building centroids, grid search + mean of pairs within 2 m; "
                         "add-ons shifted vertically to the main export in the overlap"},
              open(ap, "w"), indent=1)
    P = np.concatenate(Ps)
    V = np.c_[P, np.concatenate(Hs)]
    hx, hy = meta["half_x"], meta["half_y"]
    m = 6 * sigma
    xs = np.arange(math.floor((-hx - m) / step) * step, hx + m + step, step)
    ys = np.arange(math.floor((-hy - m) / step) * step, hy + m + step, step)
    GX, GY = np.meshgrid(xs, ys)
    Z = LinearNDInterpolator(V[:, :2], V[:, 2])(GX, GY)
    bad = np.isnan(Z)
    if bad.any():
        Z[bad] = NearestNDInterpolator(V[:, :2], V[:, 2])(GX[bad], GY[bad])
    Z = gaussian_filter(Z, sigma / step, mode="nearest")
    inside = (np.abs(GX) <= hx) & (np.abs(GY) <= hy)
    z0 = float(Z[inside].min())
    Z -= z0
    np.savez_compressed(os.path.join(out_dir, FIELD), x0=xs[0], y0=ys[0], step=step, Z=Z, z0=z0)
    t = Terrain(xs[0], ys[0], step, Z)
    gx, gy = np.gradient(Z, step)
    g = np.hypot(gx, gy)[inside]
    print("terrain field: %d x %d @ %.1f m, sigma %.1f m, heights 0..%.1f m (topo %.1f m = 0), "
          "ground slope median %.1f%%, 95%% %.1f%%, max %.1f%%"
          % (len(xs), len(ys), step, sigma, Z[inside].max(), z0, 100 * np.median(g), 100 * np.percentile(g, 95),
             100 * g.max()))
    return t


# -- draping the model / モデルを地形に沿わせる
def _raster(geom, x0, y0, nx, ny, res):
    """Boolean mask of a shapely (multi)polygon on the grid (cell centres). / ポリゴンのラスタ化。"""
    from PIL import Image, ImageDraw
    from mesh import polys
    im = Image.new("1", (nx, ny), 0)
    dr = ImageDraw.Draw(im)
    tr = lambda cs: [((x - x0) / res - 0.5, (y - y0) / res - 0.5) for x, y in cs]
    for p in polys(geom):
        dr.polygon(tr(p.exterior.coords), fill=1)
        for r in p.interiors:
            dr.polygon(tr(r.coords), fill=0)
    return np.array(im, bool)


class Drape(object):
    """z of the ground (and of the road surface) at any point of the map.
    The road heights come from lanes.json (the elevated road logic): every
    driving lane sample (centre and both edges, laterally level) is
    interpolated linearly inside the lanes, carried outward to the nearest
    lane value over gutter, bays and kerb, then blended to the terrain.
    地面・路面の高さ。路面は lanes.json の標高（CARLA のウェイポイントと同じ）。"""

    def __init__(self, terrain, L, res=RES):
        from scipy.interpolate import LinearNDInterpolator
        from scipy.ndimage import distance_transform_edt
        self.T = terrain
        hx, hy = L.meta["half_x"], L.meta["half_y"]
        m = BLEND_IN + BLEND_W + 4.0
        x0, y0 = -hx - m, -hy - m
        nx, ny = int(math.ceil((2 * hx + 2 * m) / res)), int(math.ceil((2 * hy + 2 * m) / res))
        xs = x0 + (np.arange(nx) + 0.5) * res
        ys = y0 + (np.arange(ny) + 0.5) * res
        GX, GY = np.meshgrid(xs, ys)
        # lane samples, snapped to 0.25 m and averaged (overlapping junction lanes)
        # 車線の標本点（0.25 m に丸めて平均）
        pts = []
        for l in L.lanes:
            for q in l["pts"]:
                for k in ("c", "l", "r"):
                    pts.append((q[k][0], q[k][1], q["z"]))
        A = np.array(pts)
        key = np.round(A[:, :2] / 0.25).astype(np.int64)
        uk, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.ravel()
        cnt = np.bincount(inv)
        P = np.column_stack([np.bincount(inv, A[:, 0]) / cnt, np.bincount(inv, A[:, 1]) / cnt])
        zz = np.bincount(inv, A[:, 2]) / cnt
        # one cell inside the lane outline: PIL fills edge cells, and there the
        # Delaunay of the samples can reach far (a sliver to another road)
        # 車線の輪郭より 1 セル内側（境界では遠くの点と結ぶ細い三角形がある）
        from scipy.ndimage import binary_erosion
        # the lanes as drawn (dead-end heads pave beyond them: their extra area
        # takes the nearest lane value, else it tilted towards a road beside it)
        # 描かれた車線（袋小路の転回部の外側は最寄り車線の値）
        lane_m = binary_erosion(_raster(getattr(L, "lane_area_drawn", L.lane_area), x0, y0, nx, ny, res))
        R = np.full(GX.shape, np.nan)
        R[lane_m] = LinearNDInterpolator(P, zz)(GX[lane_m], GY[lane_m])
        for j, (hull, zh) in getattr(L, "dead_end_heads_by_j", {}).items():
            hm = _raster(hull.buffer(0.35, quad_segs=4), x0, y0, nx, ny, res)
            R[hm] = zh
        have = ~np.isnan(R)
        # beyond the lanes: the nearest lane value (roads are laterally level)
        # 車線の外: 最寄りの車線の値（横断方向は水平）
        idx = distance_transform_edt(~have, return_distances=False, return_indices=True)
        R = R[idx[0], idx[1]]
        cw = _raster(L.carriageway, x0, y0, nx, ny, res)
        d = distance_transform_edt(~cw) * res
        w = np.clip(1.0 - (d - BLEND_IN) / BLEND_W, 0.0, 1.0)
        w = w * w * (3 - 2 * w)
        H = terrain(GX, GY)
        # the road's offset from the terrain, carried outward: the nearest lane
        # next to the carriageway, a smooth (normalised Gaussian) average of the
        # lanes around further out -- the nearest value alone jumps where two
        # roads at different heights are equally near (grass flapping 2 m over
        # paths between a road on fill and one in a cutting)
        # 道路と地形の差を外へ広げる: 車道際は最寄りの車線、離れると周囲の車線の滑らかな平均
        from scipy.ndimage import gaussian_filter
        O_near = R - H
        Rl = np.where(have, R, 0.0)
        num = gaussian_filter(np.where(have, Rl - H, 0.0), OFFSET_SIGMA / res, mode="nearest")
        den = gaussian_filter(have.astype(np.float64), OFFSET_SIGMA / res, mode="nearest")
        O_ext = np.where(den > 1e-9, num / np.maximum(den, 1e-9), O_near)
        t = np.clip((d - BLEND_IN) / OFFSET_RAMP, 0.0, 1.0)
        t = t * t * (3 - 2 * t)
        O = (1 - t) * O_near + t * O_ext
        self.road_raster = Field(x0 + res / 2, y0 + res / 2, res, R)
        self.field = Field(x0 + res / 2, y0 + res / 2, res, H + w * O)
        dz = (R - H)[cw]
        self.cut_fill = (float(dz.min()), float(dz.max())) if dz.size else (0.0, 0.0)

    def z(self, x, y):
        """Ground / road-surface z (the base the flat model's z is measured from). / 地面の高さ。"""
        return self.field(x, y)

    __call__ = z

    def low(self, x, y, r):
        """Lowest ground within r of a point (a rigid thing's foot). / 半径 r 内の最低地盤。"""
        a = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        xs = np.concatenate([[x], x + r * np.cos(a)])
        ys = np.concatenate([[y], y + r * np.sin(a)])
        return float(self.z(xs, ys).min())

    def footprint(self, poly, step=2.0):
        """Lowest and highest ground under a footprint. / 敷地の最低・最高地盤。"""
        import shapely
        b = shapely.segmentize(poly, step)
        c = shapely.get_coordinates(b)
        c = np.vstack([c, [[poly.centroid.x, poly.centroid.y]]])
        z = self.z(c[:, 0], c[:, 1])
        return float(z.min()), float(z.max())


def drape_for(out_dir, L):
    """The map's Drape, or None for a flat map. / 平坦な地図なら None。"""
    t = Terrain.load(out_dir)
    return Drape(t, L) if t is not None else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--glb", nargs="*", help="terrain GLB(s): the main export, then add-ons")
    ap.add_argument("--sw", nargs="*", default=[], help="each export's drawn SW corner as lat,lon (alignment guess)")
    ap.add_argument("--osm")
    ap.add_argument("--sigma", type=float, default=9.0)
    ap.add_argument("--step", type=float, default=2.0)
    a = ap.parse_args()
    make_field(a.out, a.osm, a.glb, a.sigma, a.step,
               [tuple(float(v) for v in q.split(",")) for q in a.sw] or None)


if __name__ == "__main__":
    main()
