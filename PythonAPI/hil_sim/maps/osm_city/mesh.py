"""Triangle meshes per material, and the shapes the map is made of.

Frame: x east, y north, z up, metres. Every face is wound counter-clockwise
seen from outside (its normal side).

材質ごとの三角形メッシュと基本形状。
"""

import math

import mapbox_earcut as earcut
import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient


def polys(g):
    if g is None or g.is_empty:
        return []
    if g.geom_type == "Polygon":
        return [g]
    return [q for q in getattr(g, "geoms", []) if q.geom_type == "Polygon" and not q.is_empty]


def grid_pieces(p, g):
    """A polygon cut along a global grid of g-metre lines (two levels: 16 g
    blocks, then g cells). Every mesh cut on the same grid splits a shared
    edge at the same points, so draped neighbours meet without cracks.
    全体共通の g m 格子で多角形を切る（隣り合う面の継ぎ目が一致する）。"""
    x0, y0, x1, y1 = p.bounds
    out = []
    B = 16 * g
    bx = np.arange(math.floor(x0 / B) * B, x1, B)
    by = np.arange(math.floor(y0 / B) * B, y1, B)
    BX, BY = np.meshgrid(bx, by)
    blocks = shapely.box(BX.ravel(), BY.ravel(), BX.ravel() + B, BY.ravel() + B)
    shapely.prepare(p)
    for blk, q in zip(blocks, shapely.intersection(p, blocks)):
        if q.is_empty or q.area < 1e-8:
            continue
        cx0, cy0, cx1, cy1 = q.bounds
        cx = np.arange(math.floor(cx0 / g) * g, cx1, g)
        cy = np.arange(math.floor(cy0 / g) * g, cy1, g)
        CX, CY = np.meshgrid(cx, cy)
        cells = shapely.box(CX.ravel(), CY.ravel(), CX.ravel() + g, CY.ravel() + g)
        out += [c for c in shapely.intersection(q, cells) if not c.is_empty]
    return [pp for c in out for pp in polys(c) if pp.area > 1e-8]


class Mesh(object):
    def __init__(self):
        self.parts = {}            # material -> [V, N, UV, I] lists / 材質 -> 配列
        # terrain (hilly maps only; flat maps leave these unset and nothing changes)
        # 地形（起伏のある地図のみ。平坦な地図では未設定で何も変わらない）
        self.zoff = 0.0            # rigid things: added to every z (set at the thing's anchor) / 剛体のずらし
        self.zfun = None           # draped things: z += zfun(x, y) per vertex / 頂点ごとに地形に沿わせる
        self.dense = None          # with zfun: cut faces on this grid (m) so they can bend / 曲げるための分割

    def _get(self, mat):
        if mat not in self.parts:
            self.parts[mat] = [[], [], [], [], 0]
        return self.parts[mat]

    def tris(self, mat, P, N, UV, I, raw=False):
        P = np.asarray(P, np.float64).reshape(-1, 3)
        if len(P) == 0 or len(I) == 0:
            return
        if not raw and (self.zfun is not None or self.zoff):
            P = P.copy()
            P[:, 2] += self.zfun(P[:, 0], P[:, 1]) if self.zfun is not None else self.zoff
        N = np.asarray(N, np.float64)
        if N.ndim == 1:
            N = np.tile(N, (len(P), 1))
        part = self._get(mat)
        part[0].append(P)
        part[1].append(N)
        part[2].append(np.asarray(UV, np.float64).reshape(-1, 2))
        part[3].append(np.asarray(I, np.int64).reshape(-1, 3) + part[4])
        part[4] += len(P)

    def merge(self, other):
        for mat, (V, N, UV, I, n) in other.parts.items():
            for v, nn, uv, ii in zip(V, N, UV, I):
                self.tris(mat, v, nn, uv, ii - (ii.min() if len(ii) else 0) + 0, raw=True)

    def shift_z(self, dz):
        """Move everything up by dz (a rigid thing onto the ground). / 全体を dz 上げる。"""
        for part in self.parts.values():
            part[0] = [v + np.array([0.0, 0.0, dz]) for v in part[0]]

    def ntris(self):
        return sum(sum(len(i) for i in p[3]) for p in self.parts.values())

    def arrays(self):
        out = {}
        for mat, (V, N, UV, I, n) in self.parts.items():
            if not V:
                continue
            out[mat] = (np.concatenate(V).astype(np.float32), np.concatenate(N).astype(np.float32),
                        np.concatenate(UV).astype(np.float32), np.concatenate(I).astype(np.int32))
        return out

    # -- shapes / 形状
    def quad(self, mat, a, b, c, d, uva=(0, 0), uvb=(1, 0), uvc=(1, 1), uvd=(0, 1)):
        a, b, c, d = (np.asarray(p, np.float64) for p in (a, b, c, d))
        if self.dense and self.zfun is not None:
            # long quads bend with the ground: bilinear sub-quads / 長い四角形は分割して沿わせる
            nu = int(math.ceil(max(np.hypot(*(b - a)[:2]), np.hypot(*(c - d)[:2])) / self.dense - 1e-9))
            nv = int(math.ceil(max(np.hypot(*(d - a)[:2]), np.hypot(*(c - b)[:2])) / self.dense - 1e-9))
            if nu > 1 or nv > 1:
                nu, nv = max(nu, 1), max(nv, 1)
                ua, ub, uc, ud = (np.asarray(t, np.float64) for t in (uva, uvb, uvc, uvd))
                bl = lambda A, B, C, D, u, v: (A * (1 - u) + B * u) * (1 - v) + (D * (1 - u) + C * u) * v
                dense, self.dense = self.dense, None
                for i in range(nu):
                    for j in range(nv):
                        u0, u1, v0, v1 = i / nu, (i + 1) / nu, j / nv, (j + 1) / nv
                        self.quad(mat, bl(a, b, c, d, u0, v0), bl(a, b, c, d, u1, v0), bl(a, b, c, d, u1, v1),
                                  bl(a, b, c, d, u0, v1), bl(ua, ub, uc, ud, u0, v0), bl(ua, ub, uc, ud, u1, v0),
                                  bl(ua, ub, uc, ud, u1, v1), bl(ua, ub, uc, ud, u0, v1))
                self.dense = dense
                return
        n = np.cross(b - a, d - a)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            n = np.cross(c - b, d - b)
            ln = np.linalg.norm(n)
            if ln < 1e-9:
                return
        self.tris(mat, [a, b, c, d], n / ln, [uva, uvb, uvc, uvd], [0, 1, 2, 0, 2, 3])

    def flat(self, mat, poly, z, uv_scale=1.0, up=True, zf=None, uv_rot=0.0, uvfun=None):
        """Triangulated horizontal polygon with holes; z may be a function of (x, y);
        uvfun(V) -> UV per vertex replaces the world-coordinate UVs (strips whose
        texture must run along them). / uvfun: 頂点ごとの UV（帯に沿う模様）。
        Constrained Delaunay (GEOS): the triangles follow the polygon's edges
        and holes exactly. earcut spilled triangles across holes on large
        shapes (a grass base reaching over the road edge -- a floating wheel).
        穴あき水平多角形（GEOS の制約付きドロネー: 穴をまたぐ三角形が出ない）。"""
        for p in polys(poly):
            if p.area < 1e-6:
                continue
            if self.dense and self.zfun is not None:
                pieces = grid_pieces(p, self.dense)
                if not pieces:
                    continue
                tris = shapely.get_parts(shapely.constrained_delaunay_triangles(np.array(pieces, dtype=object)))
            else:
                tris = shapely.get_parts(shapely.constrained_delaunay_triangles(p))
            if len(tris) == 0:
                continue
            C = shapely.get_coordinates(shapely.get_exterior_ring(tris)).reshape(-1, 4, 2)[:, :3, :]
            # every triangle facing up / 全三角形を上向きに
            a, b, c = C[:, 0], C[:, 1], C[:, 2]
            cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
            keep = np.abs(cross) > 1e-10
            C = C[keep]
            flip = cross[keep] < 0
            C[flip] = C[flip][:, ::-1]
            if not up:
                C = C[:, ::-1]
            V = C.reshape(-1, 2)
            I = np.arange(len(V)).reshape(-1, 3)
            Z = zf(V[:, 0], V[:, 1]) if zf is not None else np.full(len(V), z)
            P = np.column_stack([V, Z])
            Nf = (0, 0, 1 if up else -1)
            if self.zfun is not None:
                # draped: each triangle's own normal (vertices are not shared) / 傾いた面の法線
                P[:, 2] += self.zfun(V[:, 0], V[:, 1])
                T3 = P.reshape(-1, 3, 3)
                n = np.cross(T3[:, 1] - T3[:, 0], T3[:, 2] - T3[:, 0])
                n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
                Nf = np.repeat(n, 3, axis=0)
            if uvfun is not None:
                U = np.asarray(uvfun(V), np.float64).reshape(-1, 2)
            elif uv_rot:
                cr, sr = math.cos(uv_rot), math.sin(uv_rot)
                U = np.column_stack([V[:, 0] * cr + V[:, 1] * sr, -V[:, 0] * sr + V[:, 1] * cr]) / uv_scale
            else:
                U = V / uv_scale
            self.tris(mat, P, Nf, U, I, raw=self.zfun is not None)

    def wall(self, mat, p0, p1, z0, z1, u0=0.0, uv_scale=1.0, v0=None):
        """Vertical quad from p0 to p1, facing right of p0->p1. / p0->p1 の右向きの垂直面。"""
        if z1 - z0 < 1e-4:
            return
        L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if L < 1e-4:
            return
        if self.dense and self.zfun is not None and L > self.dense:
            # a long wall bends with the ground: pieces / 長い壁は分割して沿わせる
            n = int(math.ceil(L / self.dense))
            for k in range(n):
                a = (p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n)
                b = (p0[0] + (p1[0] - p0[0]) * (k + 1) / n, p0[1] + (p1[1] - p0[1]) * (k + 1) / n)
                self.wall(mat, a, b, z0, z1, u0 + L * k / n / uv_scale, uv_scale, v0)
            return
        u1 = u0 + L / uv_scale
        va = (z0 if v0 is None else v0) / uv_scale
        vb = va + (z1 - z0) / uv_scale
        self.quad(mat, (p0[0], p0[1], z0), (p1[0], p1[1], z0), (p1[0], p1[1], z1), (p0[0], p0[1], z1),
                  (u0, va), (u1, va), (u1, vb), (u0, vb))

    def ring_walls(self, mat, ring, z0, z1, outward=True, uv_scale=1.0, skip=None):
        """Walls along a closed ring (CCW ring + outward -> faces outside). / 閉じた輪に沿う壁。"""
        c = list(ring.coords)
        if not Polygon(c).exterior.is_ccw:
            c = c[::-1]
        if not outward:
            c = c[::-1]
        u = 0.0
        for a, b in zip(c, c[1:]):
            if skip is not None and skip(a, b):
                u += math.hypot(b[0] - a[0], b[1] - a[1]) / uv_scale
                continue
            # CCW ring: outside is on the right of a->b / 反時計回りなら外側は右
            self.wall(mat, a, b, z0, z1, u, uv_scale)
            u += math.hypot(b[0] - a[0], b[1] - a[1]) / uv_scale

    def prism(self, mat, poly, z0, z1, top=True, bottom=False, uv_scale=1.0, mat_top=None, mat_wall=None):
        for p in polys(poly):
            self.ring_walls(mat_wall or mat, p.exterior, z0, z1, True, uv_scale)
            for r in p.interiors:
                self.ring_walls(mat_wall or mat, r, z0, z1, False, uv_scale)
            if top:
                self.flat(mat_top or mat, p, z1, uv_scale)
            if bottom:
                self.flat(mat_top or mat, p, z0, uv_scale, up=False)

    def box(self, mat, cx, cy, z0, z1, sx, sy, ang=0.0, uv_scale=1.0, top=True, bottom=False, mat_top=None):
        c, s = math.cos(ang), math.sin(ang)
        hx, hy = sx / 2, sy / 2
        pts = [(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
        self.prism(mat, Polygon(pts), z0, z1, top=top, bottom=bottom, uv_scale=uv_scale, mat_top=mat_top)

    def cylinder(self, mat, cx, cy, z0, z1, r, seg=10, top=True, r1=None):
        r1 = r if r1 is None else r1
        P, N, UV, I = [], [], [], []
        for k in range(seg + 1):
            a = 2 * math.pi * k / seg
            ca, sa = math.cos(a), math.sin(a)
            P += [(cx + r * ca, cy + r * sa, z0), (cx + r1 * ca, cy + r1 * sa, z1)]
            N += [(ca, sa, 0), (ca, sa, 0)]
            UV += [(k / seg, 0), (k / seg, 1)]
        for k in range(seg):
            i = 2 * k
            I += [i, i + 2, i + 3, i, i + 3, i + 1]
        self.tris(mat, P, N, UV, I)
        if top and r1 > 0:
            self.flat(mat, Polygon([(cx + r1 * math.cos(2 * math.pi * k / seg),
                                     cy + r1 * math.sin(2 * math.pi * k / seg)) for k in range(seg)]), z1)

    def sphere(self, mat, cx, cy, cz, rx, ry, rz, seg=8, rings=5, jitter=None):
        P, N, UV, I = [], [], [], []
        for j in range(rings + 1):
            th = math.pi * j / rings
            for k in range(seg + 1):
                ph = 2 * math.pi * k / seg
                n = (math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th))
                f = 1.0 if jitter is None else jitter(j, k % seg)
                P.append((cx + rx * n[0] * f, cy + ry * n[1] * f, cz + rz * n[2] * f))
                N.append(n)
                UV.append((k / seg, j / rings))
        for j in range(rings):
            for k in range(seg):
                a = j * (seg + 1) + k
                b = a + seg + 1
                I += [a, b, a + 1, a + 1, b, b + 1]
        self.tris(mat, P, N, UV, I)

    def ribbon(self, mat, pts, w, z, uv_scale=1.0):
        """A flat strip of width w along a polyline. / 折れ線に沿う帯。"""
        from shapely.geometry import LineString
        self.flat(mat, LineString(pts).buffer(w / 2, cap_style="flat", join_style="mitre"), z, uv_scale)
