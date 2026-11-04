"""Trees and street furniture (maps/.venv).

  trees      rain trees and angsanas (wide crowns), royal palms; along the
             verges between kerb and footpath, on wide medians, scattered
             over estate lawns and parks, dense beyond the ring road. No
             trunk on a carriageway, path or driveway; crowns over a road
             start above 4.5 m.
  lamps      grey lamp posts along the roads, on the verge
  shelters   bus shelters at the OSM bus stops, on the footpath beside the kerb
  linkways   covered walkways over the footways OSM tags covered=yes
  barrier    a noise barrier where the ring road runs beside the expressway
  beacons    Belisha beacons (amber globes) at the ends of zebra crossings

Signs and traffic lights are CARLA's own, made from the road logic.

街路樹とストリートファニチャー。標識と信号は CARLA が道路情報から生成する。
"""

import math
import random

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

from mesh import Mesh, polys
import jtrees

CLEAR_OVER_ROAD = 4.5
JT_ROOM = 0.8             # seasonal trees: crown edge to the carriageway / 樹冠と車道の間隔


class Props(object):
    def __init__(self, L, crossings, seed=7, keepout=None):
        self.L = L
        self.crossings = crossings
        self.drape = None         # terrain.Drape on hilly maps (set by build.py) / 起伏のある地図のみ
        self.rng = random.Random(seed)
        self.trees = []           # (x, y, kind, size) / 樹木
        # Japan-style maps: every tree drawn as a seasonal Japanese species
        # (jtrees.py) at the same spots; build.py sets it / 日本式は季節の樹木で描く
        self.jtrees = False
        self.items = []           # (kind, data) / 設置物
        self.no_trunk = unary_union([L.road_all, L.driveways, getattr(L, "reserved", Polygon()), L.parking, L.footpaths.buffer(0.3),
                                     L.cycleways.buffer(0.3), L.building_area.buffer(2.5), L.pools.buffer(1.0),
                                     L.pitches, L.playgrounds.buffer(0.5)] + ([keepout] if keepout is not None else []))
        self.place_street_trees()
        self.place_lawn_trees()
        self.place_edge_trees()
        self.place_lamps()
        self.place_shelters()
        self.place_linkways()
        self.place_barrier()
        self.place_beacons()

    def ok(self, x, y, r=0.6):
        return not self.no_trunk.intersects(Point(x, y).buffer(r)) and self.L.bounds.contains(Point(x, y))

    def far_enough(self, x, y, d):
        return all((x - tx) ** 2 + (y - ty) ** 2 > d * d for tx, ty, _, _ in self.near(x, y))

    def near(self, x, y):
        k = (int(x // 20), int(y // 20))
        out = []
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                out += self.grid.get((k[0] + i, k[1] + j), [])
        return out

    def add_tree(self, x, y, kind, size):
        self.trees.append((x, y, kind, size))
        self.grid.setdefault((int(x // 20), int(y // 20)), []).append((x, y, kind, size))

    # -- trees / 樹木
    def place_street_trees(self):
        self.grid = {}
        L = self.L
        verge = L.carriageway.buffer(0.2 + 0.9, quad_segs=2)
        main_roads = unary_union([Polygon([q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]).buffer(0)
                                  for l in L.lanes if (l["speed"] or 0) >= 13.0 and l["junction"] < 0])
        for p in polys(verge):
            for ring in [p.exterior] + list(p.interiors):
                n = int(ring.length // 9.0)
                for k in range(n):
                    q = ring.interpolate(k * 9.0 + self.rng.uniform(-1, 1))
                    if not self.ok(q.x, q.y, 0.5):
                        continue
                    big = main_roads.distance(q) < 6.0
                    kind = "palm" if big and self.rng.random() < 0.12 else \
                        ("rain" if self.rng.random() < 0.55 else "angsana")
                    self.add_tree(q.x, q.y, kind, self.rng.uniform(0.85, 1.2) * (1.15 if big else 0.95))
        for p in polys(L.islands):
            if p.buffer(-1.2).is_empty:
                continue
            spine = p.buffer(-1.2)
            for q in polys(spine):
                n = int(q.exterior.length // 2 // 10.0)
                for k in range(n):
                    pt = q.exterior.interpolate(k * 10.0)
                    if self.L.carriageway.distance(pt) > 1.2:
                        self.add_tree(pt.x, pt.y, "palm" if self.rng.random() < 0.3 else "angsana", 1.0)

    def place_lawn_trees(self):
        L = self.L
        lawns = unary_union([L.grass, L.parks]).intersection(L.inside)
        for p in polys(lawns):
            if p.area < 40.0:
                continue
            minx, miny, maxx, maxy = p.bounds
            n = int(p.area / 260.0) + 1
            for _ in range(n * 3):
                x, y = self.rng.uniform(minx, maxx), self.rng.uniform(miny, maxy)
                if not p.contains(Point(x, y)) or not self.ok(x, y, 1.0) or not self.far_enough(x, y, 8.0):
                    continue
                kind = self.rng.choice(["rain", "angsana", "angsana", "palm", "frangipani"])
                self.add_tree(x, y, kind, self.rng.uniform(0.7, 1.2))
                n -= 1
                if n <= 0:
                    break
        for p in polys(L.forest):
            minx, miny, maxx, maxy = p.bounds
            for _ in range(int(p.area / 50.0)):
                x, y = self.rng.uniform(minx, maxx), self.rng.uniform(miny, maxy)
                if p.contains(Point(x, y)) and self.ok(x, y) and self.far_enough(x, y, 5.0):
                    self.add_tree(x, y, "angsana", self.rng.uniform(0.9, 1.3))

    def place_edge_trees(self):
        L = self.L
        out = L.bounds.difference(L.inside.buffer(1.0))
        for p in polys(out):
            minx, miny, maxx, maxy = p.bounds
            for _ in range(int(p.area / 45.0)):
                x, y = self.rng.uniform(minx, maxx), self.rng.uniform(miny, maxy)
                if p.contains(Point(x, y)) and self.ok(x, y) and self.far_enough(x, y, 5.5):
                    self.add_tree(x, y, self.rng.choice(["rain", "angsana", "angsana"]), self.rng.uniform(0.9, 1.4))

    # -- furniture / 設置物
    def place_lamps(self):
        L = self.L
        ring_line = L.carriageway.buffer(0.2 + 0.45, quad_segs=2)
        for p in polys(ring_line):
            for ring in [p.exterior] + list(p.interiors):
                if ring.length < 30:
                    continue
                for k in range(int(ring.length // 32.0)):
                    q = ring.interpolate(k * 32.0 + 4.0)
                    if L.driveways.buffer(1.0).contains(q) or L.footpaths.contains(q) or \
                            getattr(L, "reserved", Polygon()).buffer(1.0).contains(q):
                        continue
                    # the arm reaches over the road: toward the nearest carriageway edge
                    # 腕は最寄りの車道側へ
                    e = L.carriageway.boundary.interpolate(L.carriageway.boundary.project(q))
                    ang = math.atan2(e.y - q.y, e.x - q.x)
                    self.items.append(("lamp", (q.x, q.y, ang)))

    def place_shelters(self):
        L = self.L
        for p, t, nid in L.osm.points(lambda t: t.get("highway") == "bus_stop"):
            if not L.inside.contains(p):
                continue
            e = L.carriageway.boundary.interpolate(L.carriageway.boundary.project(p))
            v = np.array([p.x - e.x, p.y - e.y])
            d = np.linalg.norm(v)
            if d < 1e-3:
                # node on the road: step out to the kerb side / 道路上のノードは縁石側へ
                continue
            v /= d
            c = np.array([e.x, e.y]) + v * 3.2
            if L.carriageway.distance(Point(c)) < 2.0 or L.building_area.distance(Point(c)) < 1.5:
                continue
            ang = math.atan2(v[1], v[0])        # facing the road is -v / 道路を向く
            self.items.append(("shelter", (c[0], c[1], ang)))

    def place_linkways(self):
        for ln, w in self.L.covered_ways:
            g = ln.difference(self.L.road_all.buffer(1.6))      # roof half-width 1.3 m + 0.3 / 屋根幅の半分＋余裕
            for part in getattr(g, "geoms", [g]):
                if part.geom_type == "LineString" and part.length > 3.0:
                    self.items.append(("linkway", part))

    def place_barrier(self):
        L = self.L
        if not L.meta.get("expressway_cut"):
            return
        from shapely import wkt
        ring = wkt.loads(L.meta["ring_wkt"]).buffer(L.meta["ring_w"] / 2 + 0.2 + 0.9 + 1.8 + 0.3).exterior
        box_ = L.bounds.buffer(-(L.meta["ring_inset"] + 12.0))
        part = ring.difference(box_)
        for g in getattr(part, "geoms", [part]):
            g2 = g.difference(make_valid(L.road_all.buffer(1.0)))     # never across a road / 道路を横切らない
            for h in getattr(g2, "geoms", [g2]):
                if h.geom_type == "LineString" and h.length > 20:
                    self.items.append(("barrier", h))

    def place_beacons(self):
        for cr in self.crossings:
            if cr["kind"] != "zebra":
                continue
            c, n = np.array(cr["c"]), np.array(cr["n"])
            for t in (cr["t0"] - 0.6, cr["t1"] + 0.6):
                q = c + n * t
                self.items.append(("beacon", (q[0], q[1])))

    # -- meshes / メッシュ
    def tree_mesh(self, m, x, y, kind, s, overroad, lift=0.0):
        rng = random.Random(int(x * 1000) ^ int(y * 1000))
        if kind == "palm":
            h = 9.0 * s
            m.cylinder("bark_palm", x, y, 0.1, h, 0.22 * s, seg=6, top=False, r1=0.17 * s)
            for k in range(9):
                a = 2 * math.pi * k / 9 + rng.random() * 0.3
                tip = (x + math.cos(a) * 3.2 * s, y + math.sin(a) * 3.2 * s, h - 1.3 * s)
                side = (-math.sin(a) * 0.5 * s, math.cos(a) * 0.5 * s)
                mid = (x + math.cos(a) * 1.5 * s, y + math.sin(a) * 1.5 * s, h + 0.4 * s)
                base = (x, y, h)
                for sg in (1, -1):
                    q = (mid[0] + side[0] * sg, mid[1] + side[1] * sg, mid[2])
                    m.tris("leaves_palm", [base, q, tip], (0, 0, 1), [(0, 0), (1, 0.5), (0, 1)], [0, 1, 2])
                    m.tris("leaves_palm", [base, tip, q], (0, 0, -1), [(0, 0), (0, 1), (1, 0.5)], [0, 1, 2])
            return
        if kind == "frangipani":
            h0, R, H = 1.4 * s, 2.2 * s, 2.0 * s
        elif kind == "rain":
            h0, R, H = 4.6 * s, 6.0 * s, 4.0 * s           # wide umbrella crown / 傘状の樹冠
        else:
            h0, R, H = 4.6 * s, 4.2 * s, 5.0 * s
        if overroad:
            # (+ lift: a tree on a bank above a road in a cutting) / 切土の上の木は嵩上げ
            # the lift is on top of the clearance height the check in build() used
            # (max(h0, 4.9 + lift) under-lifted big crowns) / 嵩上げは判定時の高さに加える
            h0 = max(h0, CLEAR_OVER_ROAD + 0.4) + lift        # crowns sag below h0 / 樹冠の下垂分
            R = min(R, 4.0)
        m.cylinder("bark", x, y, 0.1, h0 + H * 0.3, 0.28 * s, seg=6, top=False, r1=0.18 * s)
        leaves = "leaves_" + ("rain" if kind == "rain" else "dark" if kind == "angsana" else "pale")
        for k in range(3):
            a = 2 * math.pi * k / 4 + rng.random()
            ox, oy = math.cos(a) * R * 0.38, math.sin(a) * R * 0.38
            cz = h0 + H * (0.55 + 0.15 * rng.random())
            j = lambda jj, kk, _r=rng: 0.85 + 0.3 * _r.random()
            m.sphere(leaves, x + ox, y + oy, cz, R * 0.62, R * 0.62, H * 0.5, seg=7, rings=4, jitter=j)


    def build(self, tile):
        m = Mesh()
        road = self.L.carriageway
        x0, y0, x1, y1 = tile.bounds
        D = self.drape
        road_t = near_t = None                  # the road near this tile (terrain check) / タイル付近の道路
        leaves = {sp: Mesh() for sp in jtrees.KIND} if self.jtrees else {}
        for x, y, kind, s in self.trees:
            if x0 <= x < x1 and y0 <= y < y1:
                dist = road.distance(Point(x, y))
                if self.jtrees and dist < 1.0:
                    continue        # trunk on the carriageway (a road off the map's edge) / 幹が車道上
                over = dist < 6.5
                lift = 0.0
                if D is not None:
                    m.zoff = D.low(x, y, 0.5)           # rigid, at its foot / 根元の高さで剛体移動
                    # crown bottom vs the highest road surface under the crown
                    # (a tree on a bank over a road in a cutting) / 樹冠の下端と樹冠下の路面
                    if self.jtrees:
                        sp, ctx, lite = self.jt_species(x, y, kind, over)
                        R, h0, H = jtrees.dims(sp, s, over, CLEAR_OVER_ROAD + 0.4)
                        if not over:
                            R = min(R, max(1.0, dist - JT_ROOM))
                    else:
                        R, h0, H = {"palm": (3.2, 7.7, 0.0), "frangipani": (2.2, 1.4, 2.0),
                                    "rain": (6.0, 4.6, 4.0)}.get(kind, (4.2, 4.6, 5.0))
                        R, h0, H = R * s, h0 * s, H * s
                    if over:
                        R, h0 = min(R, 4.0), max(h0, CLEAR_OVER_ROAD + 0.4)
                    reach = 1.1 * R
                    if dist < reach:
                        a = np.linspace(0, 2 * np.pi, 12, endpoint=False)
                        xs = np.concatenate([[x], x + reach * np.cos(a), x + reach / 2 * np.cos(a)])
                        ys = np.concatenate([[y], y + reach * np.sin(a), y + reach / 2 * np.sin(a)])
                        # a seasonal tree at the foot of a bank under a raised road: its
                        # limbs climb into the road's clearance / 盛土の下の木は枝が道路に届く
                        if self.jtrees and float(self.drape.z(xs, ys).max()) - m.zoff > h0 - 1.0:
                            continue
                        # samples within 2 m of the road count too: a crown triangle that
                        # reaches over the road also hangs over the bank beside it, and
                        # check_roads.py measures each corner from the ground under it
                        # 道路から 2 m 以内の標本も（樹冠の三角形は道路脇の法面にも掛かる）
                        if road_t is None:
                            road_t = road.intersection(tile.buffer(12.0))
                            near_t = road_t.buffer(2.0)
                            shapely.prepare(near_t)
                        inr = shapely.contains_xy(near_t, xs, ys)
                        # and the corners of the road under the crown: a kerb corner can
                        # reach in between the samples / 樹冠下の道路の角も（標本点の間に入る角）
                        cut = near_t.intersection(Point(x, y).buffer(reach, 3))
                        cs = np.array([c for g in getattr(cut, "geoms", [cut]) if g.geom_type == "Polygon"
                                       for c in g.exterior.coords] or np.zeros((0, 2)))
                        if inr.any() or len(cs):
                            zx = np.concatenate([xs[inr], cs[:, 0]]) if len(cs) else xs[inr]
                            zy = np.concatenate([ys[inr], cs[:, 1]]) if len(cs) else ys[inr]
                            need = float(self.drape.z(zx, zy).max()) + CLEAR_OVER_ROAD + 0.1 * H \
                                - (m.zoff + h0)
                            if need > (1.0 if over else 0.0):
                                continue                 # it would hang over the road / 道路に垂れ下がる
                            lift = max(0.0, need)
                if self.jtrees:
                    self.jt_mesh(m, leaves, x, y, kind, s, over, lift, dist)
                else:
                    self.tree_mesh(m, x, y, kind, s, over, lift)
        m.zoff = 0.0
        f = Mesh()
        for kind, d in self.items:
            if kind in ("lamp", "shelter", "beacon"):
                if not (x0 <= d[0] < x1 and y0 <= d[1] < y1):
                    continue
                if D is not None:
                    f.zoff = D.low(d[0], d[1], 3.6 if kind == "shelter" else 0.3)
                getattr(self, "m_" + kind)(f, *d)
                f.zoff = 0.0
            else:
                mid = d.interpolate(0.5, normalized=True)
                if x0 <= mid.x < x1 and y0 <= mid.y < y1:
                    if D is not None:                    # long things follow the ground / 長い物は地形に沿う
                        f.zfun, f.dense = D.z, 2.5
                    getattr(self, "m_" + kind)(f, d)
                    f.zfun, f.dense = None, None
        out = {"Terrain_Trees": m, "Terrain_Street": f}
        for sp, lm in leaves.items():
            out[jtrees.KIND[sp]] = lm
        return out

    def jt_species(self, x, y, kind, over):
        """(species, context, lite) for a tree of the generic layout: street trees
        near the carriageway, woods (forest, outside the ring) lite, the rest park.
        汎用配置の木の樹種・場所・簡略の別。"""
        # a species asked for by name ("camphor", "camphor_lite": tree rows of
        # landmarks/akeno.py); the generic kinds (rain, angsana, ...) are not species
        # 樹種名で指定された木（樹列）。汎用の種類名は樹種ではない
        sp = kind[:-5] if kind.endswith("_lite") else kind
        if sp in jtrees.FORM:
            return sp, "street" if over else "park", kind.endswith("_lite")
        if not hasattr(self, "_jt_wood"):
            self._jt_wood = unary_union([self.L.forest, self.L.bounds.difference(self.L.inside.buffer(1.0))])
            shapely.prepare(self._jt_wood)
        if shapely.contains_xy(self._jt_wood, x, y):
            ctx = "wood"
        elif over or self.L.carriageway.distance(Point(x, y)) < 6.5:
            ctx = "street"
        else:
            ctx = "park"
        return jtrees.pick(x, y, ctx), ctx, ctx == "wood"

    def jt_mesh(self, m, leaves, x, y, kind, s, over, lift, dist=99.0):
        """A seasonal tree: bark into the trees mesh, leaves into the species'
        own mesh, both at the tree's foot (m.zoff). / 季節の樹木（樹皮と種別の葉）。"""
        sp, ctx, lite = self.jt_species(x, y, kind, over)
        lo, hi = jtrees.HEIGHT[sp]
        hgt = (lo + hi) / 2 * s
        clear = (CLEAR_OVER_ROAD + 0.4 + lift) if over else None
        # beside a road the limbs and crown stay off it: they start low, under the
        # crown, unlike the old sphere crowns / 道路脇では枝も樹冠も道路に出さない
        crown = 4.0 if over else max(1.0, dist - JT_ROOM)
        lm = leaves[sp]
        lm.zoff = m.zoff
        jtrees.tree(m, lm, x, y, sp, seed=int(x * 100) * 31 + int(y * 100), height=hgt, crown=crown,
                    clear=clear, lite=lite, z0=0.1)
        lm.zoff = 0.0

    def m_lamp(self, m, x, y, ang):
        m.cylinder("metal_grey", x, y, 0.1, 10.0, 0.11, seg=6, top=False, r1=0.07)
        c, s = math.cos(ang), math.sin(ang)
        m.box("metal_grey", x + c * 1.1, y + s * 1.1, 9.85, 9.95, 2.3, 0.08, ang)
        m.box("lamp_head", x + c * 2.2, y + s * 2.2, 9.75, 9.95, 0.7, 0.3, ang)

    def m_shelter(self, m, x, y, ang):
        c, s = math.cos(ang), math.sin(ang)
        tx, ty = -s, c                          # along the kerb / 縁石方向
        # LTA standard: grey steel frame; some shelters now carry a green roof
        # LTA 標準: 灰色の鋼製フレーム、一部は屋上緑化
        m.box("metal_grey", x, y, 2.6, 2.75, 2.4, 7.0, ang, top=True, bottom=True)
        if (int(x * 7) + int(y * 3)) % 3 == 0:
            m.box("grass_park", x, y, 2.75, 2.85, 2.2, 6.8, ang, top=True, bottom=False)
        for u in (-3.2, 0.0, 3.2):
            m.box("metal_grey", x + c * 0.9 + tx * u, y + s * 0.9 + ty * u, 0.15, 2.6, 0.12, 0.12)
        m.box("glass_rail", x + c * 1.0, y + s * 1.0, 0.4, 2.3, 0.05, 6.4, ang)
        m.box("paint_navy", x + c * 0.5, y + s * 0.5, 0.15, 0.55, 0.4, 3.5, ang)

    def m_linkway(self, m, line):
        roof = line.buffer(1.3, cap_style="flat", join_style="mitre")
        m.flat("roof_linkway", roof, 2.75, 2.0)
        m.flat("roof_linkway", roof, 2.65, 2.0, up=False)
        for k in range(int(line.length // 4.0) + 1):
            q = line.interpolate(min(line.length, k * 4.0))
            m.box("metal_grey", q.x, q.y, 0.15, 2.65, 0.1, 0.1)

    def m_barrier(self, m, line):
        c = list(line.coords)
        for a, b in zip(c, c[1:]):
            m.wall("barrier", a, b, 0.14, 4.0, 0.0, 3.0)
            m.wall("barrier", b, a, 0.14, 4.0, 0.0, 3.0)
        for k in range(int(line.length // 5.0) + 1):
            q = line.interpolate(min(line.length, k * 5.0))
            m.box("metal_grey", q.x, q.y, 0.14, 4.2, 0.2, 0.2)

    def m_beacon(self, m, x, y):
        m.cylinder("pole_stripes", x, y, 0.15, 2.6, 0.06, seg=6, top=True)
        m.sphere("beacon_amber", x, y, 2.85, 0.25, 0.25, 0.25, seg=8, rings=5)
