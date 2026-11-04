"""Landmarks on terrain: level platforms, retaining walls and banks, draped
ground (maps/.venv). Shared by every landmark module (kosen, the dorm, the
7-Eleven...). On a flat map (no terrain.Drape) everything sits at z = base
and no walls are drawn.

    S = Site(drape, region, no_go=roads_and_paths)
    p = S.platform("lab1", footprint, how="max", apron=1.5)      # a building
    S.platform("field", field_poly, how="median", bank=True)    # a court / field
    S.ground(m, "grass_park", S.free().difference(paved))         # draped / flat
    S.ground(m, "asphalt_lot", paved)
    S.walls(m)                                                   # retaining walls, banks
    bm = Mesh(); ...build the block at local z (0 = platform top)...
    bm.shift_z(p.z)

Rules it keeps (guide ch. 12-13): each platform is level, at the highest
ground under its footprint (or the median, or a given z); platforms never
overlap (a later one is cut by the earlier ones); between two platforms only
the higher draws the wall, down to 0.3 m under the lower; a fill wall faces
out from 0.3 m under the outside ground up to the platform; a cut wall faces
in from 0.3 m under the platform up to the bank (+5 cm); with bank=True a
fill higher than ``bank_min`` becomes a planted slope (grade 1:1.5) that
stays inside the region, any drop left at the region edge gets a wall. The
ground left over (``free()``) is draped on the terrain at base (0.15 m over
the road surface, like every other ground layer) on the global 2.5 m grid,
so it meets the map's own ground without cracks.

地形の上の詳細モデル: 水平な造成面・擁壁・法面・地面の張り付け（共通ヘルパー）。
"""

import math

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

from mesh import Mesh, polys

BASE = 0.15              # ground layers stand this far over the road surface / 地面層の高さ（路面基準）
DENSE = 2.5              # the map's ground grid (ground.DENSE) / 地面の分割格子
FOOT = 0.3               # walls reach this far under the lower surface / 擁壁の根入れ
SEG = 1.5                # wall pieces / 擁壁の分割長
GRADE = 1.5              # bank: horizontal per vertical / 法面勾配（水平/鉛直）
STEP = 0.03              # level differences below this draw nothing / これ未満の段差は描かない


class Platform(object):
    def __init__(self, name, poly, z, kind, bank):
        self.name, self.poly, self.z, self.kind, self.bank = name, poly, z, kind, bank

    def __repr__(self):
        return "Platform(%s, z=%.2f, %.0f m2)" % (self.name, self.z, self.poly.area)


class Site(object):
    def __init__(self, drape, region, no_go=None, base=BASE, gap=0.3, dense=DENSE):
        """drape: terrain.Drape or None (flat map); region: what the landmark owns
        (its ground is drawn here); no_go: polygons no platform may cover (roads,
        kerbs, paths, islands), kept ``gap`` m clear.
        drape: 地形（平坦なら None）、region: 担当範囲、no_go: 造成しない所。"""
        self.D = drape
        self.base = base
        self.region = region
        self.no_go = no_go.buffer(gap) if no_go is not None and not no_go.is_empty else Polygon()
        self.dense = dense
        self.items = []
        self.banks = []          # (polygon footprint, quads) per bank / 法面
        self._walls = None

    # -- heights / 高さ
    def ground_z(self, x, y):
        """The natural ground surface (draped layers' height). / 自然地盤（地面層）の高さ。"""
        x = np.asarray(x, np.float64)
        if self.D is None:
            return np.full(x.shape, self.base)
        return self.D.z(x, np.asarray(y, np.float64)) + self.base

    def _samples(self, poly, step=1.5):
        b = shapely.segmentize(poly, step)
        c = shapely.get_coordinates(b)
        # and an interior grid / 内部の格子
        x0, y0, x1, y1 = poly.bounds
        gx, gy = np.meshgrid(np.arange(x0, x1, 3.0) + 1.5, np.arange(y0, y1, 3.0) + 1.5)
        g = np.column_stack([gx.ravel(), gy.ravel()])
        if len(g):
            g = g[shapely.contains_xy(poly, g[:, 0], g[:, 1])]
            c = np.vstack([c, g]) if len(g) else c
        return c

    def level(self, poly, how="max"):
        """The platform level for a footprint: highest ("max"), lowest ("min") or
        median ground under it. / 造成高さ（最高・最低・中央値）。"""
        c = self._samples(poly)
        z = self.ground_z(c[:, 0], c[:, 1])
        return float({"max": np.max, "min": np.min, "median": np.median}[how](z))

    def drop(self, poly):
        """Ground range under a footprint (highest - lowest). / 敷地内の高低差。"""
        c = self._samples(poly)
        z = self.ground_z(c[:, 0], c[:, 1])
        return float(z.max() - z.min())

    # -- platforms / 造成面
    def platform(self, name, poly, z=None, how="max", apron=0.0, kind="building", bank=False, keep=None):
        """A level platform: ``poly`` (a building footprint, a court) grown by
        ``apron`` (paved margin), kept inside the region, off the no-go areas and
        off earlier platforms; at ``z`` or the level of ``poly`` (``how``). The
        footprint itself (``keep``, default ``poly``) is never cut by the apron
        rules, only by no-go. Returns the Platform (z = its top surface).
        水平な造成面を作る（建物・コート）。"""
        if z is None:
            z = self.level(poly, how)
        core = poly if keep is None else keep
        g = poly.buffer(apron, join_style="mitre") if apron > 0 else poly
        g = g.intersection(self.region.buffer(0.01)).difference(self.no_go)
        taken = unary_union([p.poly for p in self.items]) if self.items else Polygon()
        g = unary_union([g, core.intersection(self.region.buffer(0.01)).difference(self.no_go)]).difference(taken)
        g = _clean(g)
        if g.is_empty:
            return None
        p = Platform(name, g, z, kind, bank)
        self.items.append(p)
        self._walls = None
        return p

    def steps(self, poly, max_drop=1.2, min_len=8.0, how="max"):
        """A long footprint on a slope cut across its long axis into pieces whose
        ground range stays under ``max_drop`` (a stepped building): [(piece, z)].
        斜面の長い建物を長手方向に分割（段状）。"""
        r = orient(poly.minimum_rotated_rectangle, 1.0)
        c = np.array(r.exterior.coords[:4])
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = int(np.argmax(e))
        a, b = c[k], c[(k + 1) % 4]
        L = e[k]
        t = (b - a) / L
        n = np.array([-t[1], t[0]])
        for parts in range(1, max(1, int(L // min_len)) + 1):
            out = []
            for i in range(parts):
                u0, u1 = L * i / parts, L * (i + 1) / parts
                cut = Polygon([tuple(a + t * u0 - n * 1e3), tuple(a + t * u1 - n * 1e3), tuple(a + t * u1 + n * 1e3),
                               tuple(a + t * u0 + n * 1e3)])
                q = _clean(poly.intersection(cut))
                if not q.is_empty:
                    out.append(q)
            if all(self.drop(q) <= max_drop for q in out) or parts == max(1, int(L // min_len)):
                return [(q, self.level(q, how)) for q in out]
        return [(poly, self.level(poly, how))]

    def at(self, x, y):
        """The platform holding a point, or None. / 点を含む造成面。"""
        p = Point(x, y)
        for it in self.items:
            if it.poly.covers(p):
                return it
        return None

    def surface_z(self, x, y):
        """Finished ground: the platform's level inside a platform, the draped
        ground elsewhere (vectorised). / 仕上がり地盤（造成面または地形）。"""
        x = np.atleast_1d(np.asarray(x, np.float64))
        y = np.atleast_1d(np.asarray(y, np.float64))
        z = self.ground_z(x, y)
        for it in self.items:
            inside = shapely.covers(it.poly, shapely.points(np.column_stack([x, y])))
            z = np.where(inside, it.z, z)
        for fp, quads in self.banks:
            inside = shapely.contains_xy(fp, x, y)
            if inside.any():
                z = np.where(inside, np.maximum(z, self._bank_z(quads, x, y)), z)
        return z

    def low(self, x, y, r):
        """Lowest finished ground within r of a point (a rigid thing's foot). / 半径 r 内の最低地盤。"""
        a = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        return float(self.surface_z(np.r_[x, x + r * np.cos(a)], np.r_[y, y + r * np.sin(a)]).min())

    def free(self):
        """The region outside every platform and bank (draped ground). / 造成面・法面以外の範囲。"""
        self.walls_plan()
        taken = unary_union([p.poly for p in self.items] + [fp for fp, q in self.banks])
        return _clean(self.region.difference(taken))

    # -- meshes / メッシュ
    def draped(self, dz=0.0):
        """An empty Mesh whose faces follow the natural ground + base + dz. / 地形に沿うメッシュ。"""
        m = Mesh()
        if self.D is not None:
            m.zfun = lambda x, y, dz=dz: self.D.z(x, y) + dz
            m.dense = self.dense
        return m

    def ground(self, m, mat, geom, dz=0.0, uv_scale=4.0):
        """Draw a ground surface: flat on each platform it covers, draped on the
        rest; dz lifts it (paint, courts). / 地面を描く（造成面は水平、他は地形に沿う）。"""
        if geom is None or geom.is_empty:
            return
        self.walls_plan()
        rest = geom
        for it in self.items:
            part = _clean(geom.intersection(it.poly), 1e-4)
            if not part.is_empty:
                m.flat(mat, part, it.z + dz, uv_scale)
                rest = rest.difference(it.poly)
        for fp, quads in self.banks:
            rest = rest.difference(fp)
        rest = _clean(rest, 1e-4)
        if rest.is_empty:
            return
        d = self.draped(dz)
        d.flat(mat, rest, self.base, uv_scale)
        m.merge(d)

    def walls_plan(self):
        """Work out every wall and bank once (after the last platform). / 擁壁・法面の計画。"""
        if self._walls is not None:
            return self._walls
        walls, self.banks, self.bank_ends = [], [], []
        order = sorted(self.items, key=lambda p: -p.z)
        for it in order:
            for g in polys(it.poly):
                g = orient(g, 1.0)
                for ring in [g.exterior] + list(g.interiors):
                    walls += self._ring_walls(it, ring)
        self._walls = walls
        return walls

    def _neighbour(self, me, x, y):
        """z of what lies just outside a platform edge: another platform, a bank or
        the ground; and whether it is a platform. / 縁の外側の高さ。"""
        for it in self.items:
            if it is not me:
                if getattr(it, "_pb", None) is None:
                    it._pb = it.poly.buffer(0.02)
                    shapely.prepare(it._pb)
                if shapely.contains_xy(it._pb, x, y):
                    return it.z, True
        return float(self.ground_z([x], [y])[0]), False

    def _ring_walls(self, it, ring):
        out = []
        c = shapely.get_coordinates(shapely.segmentize(LineString(ring.coords), SEG))
        # CCW exterior / CW interior (orient): the outside is on the right / 外側は右
        for a, b in zip(c[:-1], c[1:]):
            L = math.dist(a, b)
            if L < 1e-3:
                continue
            t = (b - a) / L
            n = np.array([t[1], -t[0]])              # right of a->b = outside / 右が外
            mid = (a + b) / 2 + n * 0.06
            zn, plat = self._neighbour(it, mid[0], mid[1])
            if it.z > zn + STEP:
                # fill: the outside is lower / 盛土: 外が低い
                if plat:
                    out.append(("fill", a, b, n, zn - FOOT, zn - FOOT, it.z))
                    continue
                ga = float(self.ground_z([a[0] + n[0] * 0.06], [a[1] + n[1] * 0.06])[0])
                gb = float(self.ground_z([b[0] + n[0] * 0.06], [b[1] + n[1] * 0.06])[0])
                out.append(("fill", a, b, n, ga - FOOT, gb - FOOT, it.z))
            elif it.z < zn - STEP and not plat:
                # cut: the outside ground is higher, the wall faces in / 切土: 内向きの擁壁
                ga = float(self.ground_z([a[0] + n[0] * 0.06], [a[1] + n[1] * 0.06])[0])
                gb = float(self.ground_z([b[0] + n[0] * 0.06], [b[1] + n[1] * 0.06])[0])
                out.append(("cut", a, b, n, it.z - FOOT, ga + 0.05, gb + 0.05))
        if it.bank:
            out = self._banked(it, out)
        return out

    def _banked(self, it, walls, bank_min=1.2, grade=GRADE):
        """Fill walls higher than bank_min become a slope down to the ground,
        inside the region; what the region edge leaves gets a wall. / 高い盛土は法面に。"""
        keep, quads, fps = [], [], []
        ends = {}
        allowed = self.region.difference(self.no_go).difference(
            unary_union([p.poly for p in self.items if p is not it]))
        for w in walls:
            kind, a, b, n = w[:4]
            if kind != "fill" or w[6] - max(w[4], w[5]) - FOOT < bank_min or self._neighbour(
                    it, *((a + b) / 2 + n * 0.06))[1]:
                keep.append(w)
                continue
            toes = []
            for p in (a, b):
                d, zt, wall_h = self._toe(it.z, p, n, grade, allowed)
                toes.append((p + n * d, zt, wall_h))
            (ta, za, ha), (tb, zb, hb) = toes
            q = [(a[0], a[1], it.z), (b[0], b[1], it.z), (tb[0], tb[1], zb), (ta[0], ta[1], za)]
            fp = Polygon([(x, y) for x, y, z in q]).buffer(0)
            # a slope only where it lies wholly inside the region and reaches the
            # ground; elsewhere the vertical wall stays (nothing outside the site,
            # no ground layer over another) / 法面は範囲内で地盤まで届く所だけ
            if ha > STEP or hb > STEP or fp.is_empty or not fp.within(allowed.buffer(0.02)):
                keep.append(w)
                continue
            quads.append(q)
            fps.append(fp)
            for p, t, zt in ((a, ta, za), (b, tb, zb)):
                ends.setdefault((round(float(p[0]), 3), round(float(p[1]), 3)), []).append((p, t, zt))
        # close every bank end: where two banked pieces meet, a triangle over the
        # wedge between their toes; where a bank stops beside a kept wall (or the
        # region edge), a vertical end wall from 0.3 m under the ground up to the
        # bank surface (open ends showed as see-through triangles)
        # 法面の端を閉じる: 隣の法面との楔は三角形で、壁の隣では鉛直の妻壁で
        for k, lst in ends.items():
            p = lst[0][0]
            if len(lst) >= 2:
                (_, t1, z1), (_, t2, z2) = lst[0], lst[1]
                tri = [(p[0], p[1], it.z), (t1[0], t1[1], z1), (t2[0], t2[1], z2), (t2[0], t2[1], z2)]
                if Polygon([(x, y) for x, y, z in tri[:3]]).area > 1e-4:
                    quads.append(tri)
                    fps.append(Polygon([(x, y) for x, y, z in tri[:3]]).buffer(0))
            else:
                _, t, zt = lst[0]
                gp = float(self.ground_z([p[0]], [p[1]])[0])
                gt = float(self.ground_z([t[0]], [t[1]])[0])
                self.bank_ends.append((p, it.z, gp - FOOT, t, zt, gt - FOOT))
        if quads:
            self.banks.append((_clean(unary_union(fps).buffer(0.01)), quads))
        return keep

    def _toe(self, z, p, n, grade, allowed):
        """Where a slope from (p, z) going out along n meets the ground, or the
        edge of the allowed area. Returns (distance, z there, wall left). / 法尻。"""
        d = 0.0
        lim = 40.0
        hit = LineString([tuple(p + n * 0.05), tuple(p + n * lim)]).intersection(allowed.boundary)
        if not hit.is_empty:
            pts = shapely.get_coordinates(hit)
            lim = min(lim, float(np.min(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1]))))
        while d < lim:
            d2 = min(lim, d + 0.5)
            q = p + n * d2
            zs = z - d2 / grade
            if zs <= float(self.ground_z([q[0]], [q[1]])[0]):
                # interpolate the crossing / 交点を補間
                return d2, float(self.ground_z([q[0]], [q[1]])[0]), 0.0
            d = d2
        q = p + n * lim
        zs = z - lim / grade
        g = float(self.ground_z([q[0]], [q[1]])[0])
        return lim, zs, max(0.0, zs - g)

    def _bank_z(self, quads, x, y):
        """Height of the bank surfaces at points (max over the quads covering them). / 法面の高さ。"""
        z = np.full(len(x), -1e9)
        for q in quads:
            P = np.array(q)
            for tri in ((0, 1, 2), (0, 2, 3)):
                A, B, C = P[list(tri)]
                v0, v1 = B[:2] - A[:2], C[:2] - A[:2]
                den = v0[0] * v1[1] - v1[0] * v0[1]
                if abs(den) < 1e-9:
                    continue
                px, py = x - A[0], y - A[1]
                s = (px * v1[1] - v1[0] * py) / den
                t = (v0[0] * py - px * v0[1]) / den
                ok = (s >= -1e-6) & (t >= -1e-6) & (s + t <= 1 + 1e-6)
                z = np.where(ok, np.maximum(z, A[2] + s * (B[2] - A[2]) + t * (C[2] - A[2])), z)
        return z

    def walls(self, m, mat="retaining_concrete", bank_mat="grass_park", cap_mat=None, uv_scale=2.0):
        """Draw the retaining walls and banks. Each wall end runs from 0.3 m under
        the lowest to the highest finished surface found within 0.12 m of it
        (platforms, banks, ground), so corners and joints neither stand proud nor
        leave slots. / 擁壁と法面を描く（両端の上下は周囲の仕上がり面から）。"""
        W = self.walls_plan()
        if W:
            ends = np.array([[w[1], w[2]] for w in W]).reshape(-1, 2)
            ang = np.linspace(0, 2 * np.pi, 8, endpoint=False)
            off = np.vstack([[0.0, 0.0], 0.12 * np.c_[np.cos(ang), np.sin(ang)]])
            P = (ends[:, None, :] + off[None, :, :]).reshape(-1, 2)
            Z = self.surface_z(P[:, 0], P[:, 1]).reshape(len(ends), len(off))
            hi, lo = Z.max(axis=1).reshape(-1, 2), Z.min(axis=1).reshape(-1, 2)
            for k, w in enumerate(W):
                kind, a, b, n = w[:4]
                (ta, tb), (za, zb) = hi[k], lo[k] - FOOT
                if kind == "fill":
                    _wall(m, mat, a, b, za, zb, ta, tb, uv_scale)
                    if cap_mat:
                        _cap(m, cap_mat, a, b, n, ta, tb)
                else:
                    # facing in: drawn from b to a (right of b->a is inside) / 内向き
                    _wall(m, mat, b, a, zb, za, tb + 0.03, ta + 0.03, uv_scale)
        for fp, quads in self.banks:
            for q in quads:
                P = [np.array(v, np.float64) for v in q]
                if np.allclose(P[2], P[3]):
                    # a wedge triangle, facing up / 楔の三角形（上向き）
                    tri = P[:3]
                    nrm = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                    if nrm[2] < 0:
                        tri = [tri[0], tri[2], tri[1]]
                        nrm = -nrm
                    m.tris(bank_mat, [tuple(v) for v in tri], nrm / max(np.linalg.norm(nrm), 1e-12),
                           [(0, 0), (1, 0), (0, 1)], [0, 1, 2], raw=True)
                    continue
                nrm = np.cross(P[1] - P[0], P[3] - P[0])
                if nrm[2] < 0:
                    # reversed keeping the P0-P2 diagonal that surface_z uses
                    # (another diagonal left the bank and the ground model apart)
                    # surface_z と同じ対角線のまま反転
                    P = [P[0], P[3], P[2], P[1]]
                m.quad(bank_mat, *[tuple(v) for v in P], (0, 0), (1, 0), (1, 1), (0, 1))
        for p, zp, bp, t, zt, bt in getattr(self, "bank_ends", []):
            # both faces of the end wall, 1 cm apart / 妻壁の両面（1 cm 離す）
            L = math.dist(p, t)
            if L < 1e-3:
                continue
            d = (np.asarray(t) - np.asarray(p)) / L
            side = np.array([-d[1], d[0]]) * 0.005
            # the face on the left of p->t faces left: drawn t->p; the right one p->t
            # 左側の面は左向き（t->p で描く）、右側は p->t
            _wall(m, mat, t + side, p + side, bt, bp, zt, zp, uv_scale)
            _wall(m, mat, p - side, t - side, bp, bt, zp, zt, uv_scale)

    def report(self):
        """Lines for the build log. / ビルドログ用。"""
        w = self.walls_plan()
        hs = []
        for x in w:
            if x[0] == "fill":
                top = x[6] if x[6] is not None else max(x[7], x[8])
                hs.append(top - min(x[4], x[5]) - FOOT)
            else:
                hs.append(max(x[5], x[6]) - 0.05 - x[4] - FOOT)
        return ("%d platforms, %d wall pieces (highest %.1f m), %d banks"
                % (len(self.items), len(w), max(hs) if hs else 0.0, len(self.banks)))


def _wall(m, mat, a, b, za, zb, ta, tb, uv_scale):
    """A vertical wall piece from a to b facing right of a->b, bottom za/zb, top
    ta/tb at the two ends. / 右向きの鉛直壁（両端で上下端が違う）。"""
    if max(ta - za, tb - zb) < 1e-3:
        return
    L = math.dist(a, b)
    P = [(a[0], a[1], za), (b[0], b[1], zb), (b[0], b[1], tb), (a[0], a[1], ta)]
    uv = [(0, za / uv_scale), (L / uv_scale, zb / uv_scale), (L / uv_scale, tb / uv_scale), (0, ta / uv_scale)]
    t = np.array([b[0] - a[0], b[1] - a[1], 0.0]) / max(L, 1e-9)
    n = np.array([t[1], -t[0], 0.0])
    m.tris(mat, P, n, uv, [0, 1, 2, 0, 2, 3], raw=True)


def _cap(m, mat, a, b, n, ta, tb, w=0.2):
    """Coping on a fill wall: a strip just inside the edge, facing up. / 擁壁の笠木。"""
    a2, b2 = a - n * w, b - n * w
    P = [(a2[0], a2[1], ta + 0.04), (b2[0], b2[1], tb + 0.04), (b[0], b[1], tb + 0.04), (a[0], a[1], ta + 0.04)]
    m.tris(mat, P, (0, 0, 1), [(0, 0), (1, 0), (1, 1), (0, 1)], [0, 2, 1, 0, 3, 2], raw=True)


def _clean(g, min_area=0.05):
    ps = [p for p in polys(g.buffer(0) if not g.is_empty else g) if p.area >= min_area]
    return unary_union(ps) if ps else Polygon()
