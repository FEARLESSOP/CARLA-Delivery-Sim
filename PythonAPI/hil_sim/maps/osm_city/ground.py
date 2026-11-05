"""Meshes of the ground: roads, kerbs, markings, pavements, grass, lots.

Heights: the carriageway at 0; kerb 0.15 m, a vertical step at the
carriageway's edge (the edge is 0.4 m beyond the outermost lane, so a
wheel on the lane never meets it); pavements and other raised surfaces at
0.15 m on a continuous base at 0.14 m (no cracks between surfaces); paint
8 mm on the road. Where a driveway or a pedestrian crossing meets the
road the kerb is dropped: a ramp from road level up to the pavement.

The collision of an imported CARLA map is its visible mesh, so what is
drawn here is exactly what the wheels touch.

Objects are cut into tiles and named for CARLA's semantic tags:
Road_Road_* (carriageway), Road_Curb_*, Road_Marking_*, Road_Sidewalk_*,
Road_Grass_*, Terrain_* (car parks, pitches, pools).

地面のメッシュ（車道・縁石・標示・歩道・芝・駐車場）。見た目＝当たり判定。
"""

import math

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from mesh import Mesh, polys

KERB_H = 0.15
BASE_Z = 0.14
RAISED_Z = 0.15
PAINT_Z = 0.008
LOT_PAINT_Z = RAISED_Z + 0.006
TILE = 250.0
DENSE = 2.5            # hilly maps: ground faces cut on this grid to follow the terrain / 地形に沿わせる格子
APRON_GRID = 50.0      # hilly maps: the apron beyond the map's edge, cut on this grid / 地図外の地面の格子


def tiles(meta):
    hx, hy = meta["half_x"], meta["half_y"]
    nx, ny = int(math.ceil(2 * hx / TILE)), int(math.ceil(2 * hy / TILE))
    for i in range(nx):
        for j in range(ny):
            yield "%d%d" % (i, j), box(-hx + i * TILE, -hy + j * TILE, min(hx, -hx + (i + 1) * TILE),
                                       min(hy, -hy + (j + 1) * TILE))


def bay_lines(L):
    """Parking bay lines along the aisles, both sides. / 駐車通路の両側の駐車枠線。"""
    out = []
    for ln in L.parking_aisles:
        n = int(ln.length // 2.5)
        for k in range(1, n):
            s = k * 2.5
            a, b = ln.interpolate(max(0, s - 0.3)), ln.interpolate(min(ln.length, s + 0.3))
            dx, dy = b.x - a.x, b.y - a.y
            d = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / d, dx / d
            p = ln.interpolate(s)
            for sgn in (-1, 1):
                q0 = (p.x + nx * sgn * 3.2, p.y + ny * sgn * 3.2)
                q1 = (p.x + nx * sgn * 8.0, p.y + ny * sgn * 8.0)
                out.append(LineString([q0, q1]).buffer(0.05, cap_style="flat"))
    return unary_union(out).intersection(L.parking) if out else Polygon()


class RoadSurface(object):
    """The triangles of a built (draped) road mesh, to lay paint on exactly.
    出来上がった路面メッシュの三角形（標示をその面にぴったり載せる）。"""

    def __init__(self, road, mats=None):
        T = []
        for mat, (V, N, UV, I, n) in road.parts.items():
            if V and (mats is None or mat in mats):
                VV = np.concatenate(V)
                T.append(VV[np.concatenate(I)])
        self.T = np.concatenate(T) if T else np.zeros((0, 3, 3))
        if len(self.T):
            # horizontal-ish faces only (no walls: pools, steps) / 壁面は除く
            nz = np.cross(self.T[:, 1] - self.T[:, 0], self.T[:, 2] - self.T[:, 0])
            ln = np.linalg.norm(nz, axis=1)
            self.T = self.T[(ln > 1e-12) & (np.abs(nz[:, 2]) > 0.5 * np.maximum(ln, 1e-12))]
        self.polys = shapely.polygons(self.T[:, :, :2]) if len(self.T) else np.array([], dtype=object)
        self.tree = shapely.STRtree(self.polys)
        # plane z = a x + b y + c of every triangle / 各三角形の平面
        a, b, c = self.T[:, 0], self.T[:, 1], self.T[:, 2]
        n = np.cross(b - a, c - a)
        nz = np.where(np.abs(n[:, 2]) < 1e-12, 1e-12, n[:, 2])
        self.pa, self.pb = -n[:, 0] / nz, -n[:, 1] / nz
        self.pc = a[:, 2] - self.pa * a[:, 0] - self.pb * a[:, 1]
        nn = n / np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
        self.n = np.where(nn[:, 2:3] < 0, -nn, nn)

    def paint(self, mesh, mat, geom, dz, uv_scale=1.0):
        """Triangles of geom cut by the road triangles, dz above each one's plane.
        標示形状を路面三角形で切り、その平面の dz 上に置く。"""
        if not len(self.T):
            return
        for p in polys(geom):
            idx = self.tree.query(p, predicate="intersects")
            if len(idx) == 0:
                continue
            pieces = shapely.intersection(self.polys[idx], p)
            P, own = [], []
            for k, q in zip(idx, pieces):
                for pp in polys(q):
                    if pp.area < 1e-7:
                        continue
                    tris = shapely.get_parts(shapely.constrained_delaunay_triangles(pp))
                    if len(tris):
                        C = shapely.get_coordinates(shapely.get_exterior_ring(tris)).reshape(-1, 4, 2)[:, :3]
                        P.append(C)
                        own.append(np.full(len(C), k))
            if not P:
                continue
            C = np.concatenate(P)
            own = np.concatenate(own)
            a, b, c = C[:, 0], C[:, 1], C[:, 2]
            cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
            keep = np.abs(cross) > 1e-10
            C, own, cross = C[keep], own[keep], cross[keep]
            C[cross < 0] = C[cross < 0][:, ::-1]                 # facing up / 上向き
            V = C.reshape(-1, 2)
            o3 = np.repeat(own, 3)
            Z = self.pa[o3] * V[:, 0] + self.pb[o3] * V[:, 1] + self.pc[o3] + dz
            mesh.tris(mat, np.column_stack([V, Z]), self.n[o3], V / uv_scale,
                      np.arange(len(V)).reshape(-1, 3), raw=True)


class Ground(object):
    def __init__(self, L, marks, crossings, drape=None, surfaces=None):
        self.L = L
        # maps.json "road_style": "japan": Japanese surfaces (surfaces_jp.py cuts the
        # carriageway into asphalt pieces, textures_jp.py), plain concrete kerbs,
        # asphalt footways / 日本式: 車道の材質分割・無塗装の縁石・アスファルト歩道
        self.jp = L.meta.get("road_style") == "japan"
        self.surf = surfaces
        self.drape = drape          # terrain.Drape on hilly maps, None on flat ones / 起伏のある地図のみ
        self.driveways = L.driveways
        if drape is not None:
            # hilly maps: no drive-band slivers of roads the road logic already drives along
            # the kerb (they were paved as lots and dropped the kerb): grass instead
            # 坂の地図: 走行網の道路の帯の細片（縁石沿い）は舗装せず芝に
            from layout import drive_remnants
            rem = drive_remnants(L)
            if not rem.is_empty:
                self.driveways = unary_union([p for p in polys(L.driveways.difference(rem.buffer(0.01)))
                                              if p.area >= 0.05])
            taken = unary_union([L.carriageway, L.pools, L.kerb, L.footpaths, L.aprons, L.cycleways, L.islands,
                                 L.parks, L.forest, L.parking, self.driveways, L.pitches, L.playgrounds,
                                 L.construction] + ([L.terraces] if getattr(L, "terraces", None) is not None else []))
            # (terraces: Japanese building lots, flat with retaining walls -- buildings_jp)
            # 造成した敷地の下には地面を張らない
            self.base_draped = L.bounds.difference(taken)
            # and no two raised layers on top of each other: grass islands (small
            # blocks inside the roads), pitches, playgrounds give way to the paths
            # 起伏あり: 島・競技場・遊び場は歩道と重ねない（歩道を優先）
            side = unary_union([L.footpaths, L.aprons, L.cycleways])
            self.under = {n: getattr(L, n).difference(side) for n in ("islands", "pitches", "playgrounds",
                                                                     "construction")}
        self.marks = marks
        self.crossings = crossings
        cw = L.carriageway
        # dropped kerbs at driveway mouths and crossing ends / 進入路・横断歩道の縁石切り下げ
        bands = []
        for cr in crossings:
            for g in cr["segs"]:
                bands.append(g.buffer(cr["width"] / 2 + 0.3, cap_style="flat"))
        self.ramps = unary_union([L.kerb.intersection(self.driveways.buffer(0.35)),
                                  L.kerb.intersection(unary_union(bands))]).buffer(0)
        self.ramps = unary_union([p for p in polys(self.ramps) if p.area > 0.05])
        self.kerb_flat = L.kerb.difference(self.ramps)
        self.edge = cw.boundary
        self.bays = bay_lines(L)
        # every segment of the kerb face, turned to face the road, ramps left out
        # 縁石の立ち上がりの全線分（車道向き、切り下げ部を除く）
        segs = []
        for p in polys(cw):
            for ring, outward in [(p.exterior, False)] + [(r, True) for r in p.interiors]:
                c = list(ring.coords)
                if not Polygon(c).exterior.is_ccw:
                    c = c[::-1]
                if not outward:
                    c = c[::-1]
                segs += list(zip(c, c[1:]))
        a = np.array([q[0] for q in segs])
        b = np.array([q[1] for q in segs])
        mid = (a + b) / 2
        inramp = shapely.contains_xy(self.ramps.buffer(0.05), mid[:, 0], mid[:, 1]) if not self.ramps.is_empty \
            else np.zeros(len(mid), bool)
        if drape is not None and not self.ramps.is_empty:
            # hilly maps: the face is cut where a ramp starts, not dropped for the whole
            # segment whose middle is in it (simplified kerb lines run 10-20 m straight:
            # the kerb top stood 0.15 m over the road with nothing under its edge)
            # 坂の地図: 切り下げ部だけ立ち上がりを除く（線分ごと消すと縁石の下が開いた）
            rb = self.ramps.buffer(0.05)
            near = shapely.intersects(shapely.linestrings(np.stack([a, b], 1)), rb)
            pa, pb = [a[~near]], [b[~near]]
            for p, q in zip(a[near], b[near]):
                for g in shapely.get_parts(LineString([p, q]).difference(rb)):
                    if g.geom_type == "LineString" and g.length > 0.02:
                        c = np.asarray(g.coords)
                        if np.dot(c[-1] - c[0], np.asarray(q) - np.asarray(p)) < 0:
                            c = c[::-1]                       # keep facing the road / 向きを保つ
                        pa.append(c[:1]); pb.append(c[-1:])
            a, b = np.concatenate(pa), np.concatenate(pb)
            mid = (a + b) / 2
            inramp = np.zeros(len(mid), bool)
        self.face = (a[~inramp], b[~inramp], mid[~inramp])
        # main roads have black-and-white painted kerbs (Street View): face and
        # top, painted along the kerb / 幹線道路の縁石は白黒塗装（立ち上がりと天端）
        from shapely.geometry import Polygon as _P
        main = [_P([q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]).buffer(0)
                for l in L.lanes if (l["speed"] or 0) >= 13.8 and l["junction"] < 0]
        main = unary_union(main).buffer(0.95) if main else _P()
        fm = self.face[2]
        self.painted = shapely.contains_xy(main, fm[:, 0], fm[:, 1]) if not main.is_empty else np.zeros(len(fm), bool)
        if self.jp:
            self.painted = np.zeros(len(fm), bool)            # Japan: plain concrete / 日本: 無塗装
        band = L.kerb.intersection(main)
        self.road_in = L.carriageway.buffer(-0.02)
        if drape is None and not self.jp:
            self.kerb_flat = self.kerb_flat.difference(band.buffer(0.01))
        # draped: the plain kerb top stays whole under the painted strips (no
        # grass base below to fill what the strips miss at corners); the paint
        # lies 5 mm above it / 起伏あり: 塗装帯の下にも縁石天端を残し、塗装は 5 mm 上
        self.paint_lift = 0.005 if drape is not None else 0.0

    def fitted_faces(self, kerb, mat, uv, a, b, tile, top, low):
        """Kerb faces of one tile (draped): each face piece inside the tile, split at the
        kerb top's and the road's own vertices on it, top = kerb surface, bottom = road
        surface at every split point (Drape + offsets where a surface is missing).
        坂の地図の縁石の立ち上がり: 縁石天端と路面の頂点で分割し、両面の高さに合わせる。"""
        segs = shapely.linestrings(np.stack([a, b], 1))
        hit = np.nonzero(shapely.intersects(segs, tile))[0]
        if not len(hit):
            return
        V = np.concatenate([top.T.reshape(-1, 3), low.T.reshape(-1, 3)]) if len(top.T) or len(low.T) \
            else np.zeros((0, 3))
        vt = shapely.STRtree(shapely.points(V[:, :2])) if len(V) else None

        def zat(surf, P):
            out = np.full(len(P), np.nan)
            if not len(surf.T):
                return out
            pi, ti = surf.tree.query(shapely.points(P), predicate="intersects")
            if len(pi):
                z = surf.pa[ti] * P[pi, 0] + surf.pb[ti] * P[pi, 1] + surf.pc[ti]
                np.fmax.at(out, pi, z)
            return out

        zf, dense = kerb.zfun, kerb.dense
        kerb.zfun = kerb.dense = None
        try:
            for i in hit:
                g = segs[i].intersection(tile)
                for piece in shapely.get_parts(g):
                    if piece.geom_type != "LineString" or piece.length < 0.01:
                        continue
                    c = np.asarray(piece.coords)
                    p, q = c[0], c[-1]
                    if np.dot(q - p, b[i] - a[i]) < 0:
                        p, q = q, p
                    d = q - p
                    ln = float(np.hypot(*d))
                    u = d / ln
                    ts = list(np.linspace(0.0, 1.0, int(math.ceil(ln / DENSE)) + 1))
                    if vt is not None:
                        near = vt.query(piece.buffer(0.004))
                        if len(near):
                            w = V[near, :2] - p
                            t = w @ u / ln
                            off = np.abs(w[:, 0] * u[1] - w[:, 1] * u[0])
                            ts += list(t[(off < 0.004) & (t > 0) & (t < 1)])
                    ts = np.unique(np.round(ts, 6))
                    P = p[None] + ts[:, None] * d[None]
                    nr = np.array([u[1], -u[0]])                  # right of p -> q: the road / 右 = 車道
                    zt = zat(top, P - 0.01 * nr)
                    zb = zat(low, P + 0.01 * nr)
                    base = self.drape.z(P[:, 0], P[:, 1])
                    zt = np.where(np.isfinite(zt), zt, base + KERB_H)
                    zb = np.where(np.isfinite(zb), zb, base)
                    zb = np.minimum(zb, zt - 0.02)
                    s0 = float(np.dot(p, u))
                    for k in range(len(P) - 1):
                        A, B = P[k], P[k + 1]
                        ua, ub = (s0 + ts[k] * ln) / uv, (s0 + ts[k + 1] * ln) / uv
                        kerb.quad(mat, (A[0], A[1], zb[k]), (B[0], B[1], zb[k + 1]), (B[0], B[1], zt[k + 1]),
                                  (A[0], A[1], zt[k]), (ua, 0.0), (ub, 0.0), (ub, KERB_H / uv), (ua, KERB_H / uv))
        finally:
            kerb.zfun, kerb.dense = zf, dense

    def ramp_z(self, x, y):
        d = shapely.distance(self.edge, shapely.points(np.column_stack([x, y])))
        return np.clip(d / 0.2, 0.0, 1.0) * KERB_H

    def layer(self, name):
        """A layout surface, cut clear of the paths on hilly maps. / 起伏ありでは歩道を除いた面。"""
        return self.under[name] if self.drape is not None else getattr(self.L, name)

    def mesh(self):
        """A ground mesh: draped over the terrain on hilly maps. / 地面メッシュ（起伏があれば沿わせる）。"""
        m = Mesh()
        if self.drape is not None:
            m.zfun, m.dense = self.drape.z, DENSE
        return m

    def build(self, tile):
        """All ground meshes of one tile: {object name: Mesh}. / タイル内の地面メッシュ。"""
        L = self.L
        Mesh = self.mesh
        T = lambda g: g.intersection(tile) if not g.is_empty else g
        out = {}
        road = Mesh()
        if self.surf:
            # faces sharing edges (surfaces_jp.coverage), clipped one by one / 面ごとに切り取る
            for mat, faces in self.surf.items():
                sel = faces[shapely.intersects(faces, tile)] if len(faces) else faces
                if len(sel):
                    parts = shapely.get_parts(shapely.intersection(sel, tile))
                    road.flat(mat, shapely.geometrycollections(parts), 0.0, uv_scale=5.0)
        else:
            road.flat("asphalt", T(L.carriageway), 0.0, uv_scale=6.0)
        out["Road_Road"] = road

        kmat, fmat, fuv = ("kerb_jp", "kerb_face_jp", 1.2) if self.jp else ("kerb", "kerb", 1.0)
        kerb = Mesh()
        kerb.flat(kmat, T(self.kerb_flat), KERB_H, uv_scale=1.0)
        rp = T(self.ramps)
        if not rp.is_empty:
            kerb.flat(kmat, rp, 0.0, uv_scale=1.0, zf=self.ramp_z)
        # the kerb face, along the carriageway edge, facing the road / 車道に向いた縁石の立ち上がり
        a, b, mid = self.face
        x0, y0, x1, y1 = tile.bounds
        sel = (mid[:, 0] >= x0) & (mid[:, 0] < x1) & (mid[:, 1] >= y0) & (mid[:, 1] < y1)
        if self.drape is not None:
            # hilly maps: the face is cut to the tile and takes its heights from the built
            # kerb top and road beside it (Drape sampled at the face's own points stood up
            # to 9 cm off them: slits under the kerb's edge)
            # 坂の地図: 立ち上がりの高さは隣の縁石天端と路面から（ずれて隙間が開いた）
            self.fitted_faces(kerb, fmat, fuv, a, b, tile, RoadSurface(kerb, {kmat}), RoadSurface(road))
            sel = np.zeros(len(a), bool)
        for p, q, painted in zip(a[sel], b[sel], self.painted[sel]):
            kerb.wall("kerb_bw" if painted else fmat, p, q, 0.0, KERB_H, 0.0, 2.0 if painted else fuv)
            if painted:
                # the painted top: a strip along this face, away from the road / 天端の帯
                d = np.asarray(q) - np.asarray(p)
                ln = float(np.hypot(*d)) or 1.0
                n = np.array([-d[1], d[0]]) / ln * 0.2          # left of p->q = away from the road / 道路と反対側
                corners = [np.asarray(p) + n, np.asarray(q) + n, (np.asarray(p) + np.asarray(q)) / 2 + n * 0.5]
                if any(self.road_in.contains(Point(*c)) for c in corners):
                    continue                                   # a tight corner: leave it plain / 急な角は塗らない
                u1 = ln / 2.0
                zt = KERB_H + self.paint_lift
                kerb.quad("kerb_bw", (p[0], p[1], zt), (q[0], q[1], zt),
                          (q[0] + n[0], q[1] + n[1], zt), (p[0] + n[0], p[1] + n[1], zt),
                          (0, 0), (u1, 0), (u1, 0.1), (0, 0.1))          # facing up / 上向き
        out["Road_Curb"] = kerb

        paint = Mesh()
        pw, py = ("line_white", "line_yellow") if self.jp else ("paint_white", "paint_yellow")
        on_road = RoadSurface(road) if self.drape is not None else None
        for g, col in self.marks:
            if g.intersects(tile):
                if on_road is not None:
                    # draped: the paint is cut from the road's own triangles, so it
                    # lies exactly PAINT_Z over them (sampling the terrain at its
                    # own vertices left it up to 3 cm under the road on bends)
                    # 起伏あり: 路面の三角形で切り抜き、その面のちょうど PAINT_Z 上に置く
                    on_road.paint(paint, pw if col == "w" else py, T(g), PAINT_Z, uv_scale=1.0)
                else:
                    paint.flat(pw if col == "w" else py, T(g), PAINT_Z, uv_scale=1.0)
        lots = self.lots_mesh(T)
        bays = T(self.bays)
        if not bays.is_empty:
            if self.drape is not None:
                # draped: on the car park's own triangles, like the road paint
                # 起伏あり: 駐車場の三角形の上に（路面標示と同様）
                RoadSurface(lots, {"asphalt_lot", "asphalt_estate"}).paint(
                    paint, pw, bays, LOT_PAINT_Z - RAISED_Z, uv_scale=1.0)
            else:
                paint.flat(pw, bays, LOT_PAINT_Z, uv_scale=1.0)
        out["Road_Marking"] = paint

        side = Mesh()
        if self.jp:
            # Akeno footways are asphalt behind concrete kerbs (Street View) / 明野の歩道はアスファルト
            side.flat("pavement_asphalt", T(L.footpaths), RAISED_Z, uv_scale=3.0)
            side.flat("concrete_path", T(L.aprons), RAISED_Z, uv_scale=2.0)
            side.flat("pavement_asphalt", T(L.cycleways), RAISED_Z, uv_scale=3.0)
        else:
            side.flat("pavers", T(L.footpaths), RAISED_Z, uv_scale=2.0)
            side.flat("concrete_path", T(L.aprons), RAISED_Z, uv_scale=2.0)
            side.flat("park_connector", T(L.cycleways), RAISED_Z, uv_scale=3.0)
        out["Road_Sidewalk"] = side

        grass = Mesh()
        if self.drape is None:
            base = T(L.bounds.difference(L.carriageway).difference(L.pools))
            grass.flat("grass", base, BASE_Z, uv_scale=4.0)
        else:
            # draped: the grass only where no other surface is, at their height --
            # two layers bent over the same slope on different triangles do not
            # stay 1 cm apart (grass came up through the paths)
            # 起伏あり: 芝は他の面の無い所だけ、同じ高さで（重ねると斜面で突き抜ける）
            grass.flat("grass", T(self.base_draped), RAISED_Z, uv_scale=4.0)
        grass.flat("grass", T(self.layer("islands")), RAISED_Z, uv_scale=4.0)
        grass.flat("grass_park", T(L.parks), RAISED_Z, uv_scale=4.0)
        grass.flat("grass_park", T(L.forest), RAISED_Z, uv_scale=4.0)
        # beyond the map's edge: a plain apron so the world does not end at a
        # cliff on the horizon / 地図の外の地面（地平線で途切れないように）
        if tile.touches(L.bounds.exterior) or tile.boundary.intersects(L.bounds.exterior):
            apron = tile.buffer(1500.0, join_style="mitre").difference(L.bounds)
            ox0, oy0, ox1, oy1 = L.bounds.bounds
            mine = Polygon()
            for q in polys(apron):
                # each edge tile owns the part of the apron outside its own edge
                # 各端タイルは自分の外側の部分を受け持つ
                tx0, ty0, tx1, ty1 = tile.bounds
                cx0 = tx0 if tx0 > ox0 + 1 else -1e6
                cx1 = tx1 if tx1 < ox1 - 1 else 1e6
                cy0 = ty0 if ty0 > oy0 + 1 else -1e6
                cy1 = ty1 if ty1 < oy1 - 1 else 1e6
                mine = mine.union(q.intersection(box(cx0, cy0, cx1, cy1)))
            if self.drape is not None:
                # far beyond the map: its edge cut to meet the draped ground, the
                # rest on a 50 m grid. One fan of 1.5 km x 2.5 m slivers from the
                # edge (2026-11-04) came out in CARLA as an invisible collision
                # sheet over the map's SW corner, 64 x 72 m at the outer terrain's
                # height (84 m, 4 m over the roads), not in the FBX: no slivers.
                # 地図の外: 縁は地面に合わせ、残りは 50 m 格子（細長い三角形を作らない）
                if not mine.is_empty:
                    mine = shapely.segmentize(mine, DENSE)
                    grass.dense = APRON_GRID
                    grass.flat("grass_park", mine, RAISED_Z, uv_scale=8.0)
                grass.dense = None
            else:
                grass.flat("grass_park", mine, BASE_Z, uv_scale=8.0)
        out["Road_Grass"] = grass

        out["Terrain_Lots"] = lots
        return {k: v for k, v in out.items() if v.ntris()}

    def lots_mesh(self, T):
        """Car parks, drives, pitches, playgrounds, pools of one tile. / 駐車場等。"""
        L = self.L
        lots = self.mesh()
        if self.jp:
            # car parks and private drives: old asphalt / 駐車場・私道: 古いアスファルト
            lots.flat("asphalt_estate", T(L.parking), RAISED_Z, uv_scale=5.0)
            lots.flat("asphalt_estate", T(self.driveways), RAISED_Z, uv_scale=5.0)
        else:
            lots.flat("asphalt_lot", T(L.parking), RAISED_Z, uv_scale=6.0)
            lots.flat("pavers_red", T(self.driveways), RAISED_Z, uv_scale=2.0)      # red brick drop-offs / 赤レンガ
        lots.flat("pitch", T(self.layer("pitches")), RAISED_Z, uv_scale=8.0)
        lots.flat("rubber", T(self.layer("playgrounds")), RAISED_Z, uv_scale=3.0)
        lots.flat("soil", T(self.layer("construction")), RAISED_Z, uv_scale=4.0)
        for p in polys(T(L.pools)):
            # a pool: coping, tiled walls, water / プール
            lots.flat("water", p.buffer(-0.4), 0.0, uv_scale=4.0)
            lots.flat("concrete_path", p.difference(p.buffer(-0.4)), RAISED_Z + 0.02, uv_scale=1.0)
            inner = p.buffer(-0.4)
            for q in polys(inner):
                for ring, outward in [(q.exterior, False)] + [(r, True) for r in q.interiors]:
                    lots.ring_walls("pool_tile", ring, 0.0, RAISED_Z + 0.02, outward=outward, uv_scale=1.0)
        return lots
