"""Flyovers in the 3-D model of a FLAT map (maps/.venv).

Used by build.py when meta.json has "layer_elevation" (maps.json "roadnet",
grade_sep.py raised the bridges and their ramps in the road logic). The flat
ground stays as it is; the raised lanes (lanes.json z > Z0) get their own
structure on top, made from the lanes:

  deck surface   each connected group of raised lanes (lanes meeting with
                 |dz| < 1 m) is one polygon (lanes + GUTTER m), triangulated
                 on a 2 m grid, every vertex at the z of the nearest lane
                 centre point (lanes are laterally level, as CARLA's are):
                 the wheels touch exactly the road logic (check_drive)
  markings       the lanes' own xodr marks (lanes.json ml / mr: solid, broken
                 3 / 6 m, white / yellow), 1 cm over the deck
  parapets       0.25 m concrete walls, 1.0 m high, round the deck where it is
                 more than PARA_Z above the ground
  embankment     where the deck is lower than OPEN_Z and no road runs under
                 it: retaining walls from the ground up to the deck edge
  open spans     where the deck is OPEN_Z or higher, or a lower road runs
                 under it: a SLAB m thick slab (underside, fascia) on piers
                 every PIER_STEP m that keep clear of every lower lane

Kinds: Road_RoadDeck (CARLA tag Road), Road_MarkingDeck (RoadLine),
Terrain_Flyover (walls, parapets, slab, piers). The ground under an open span
keeps whatever was there (the lower roads, and the raised road's own flat
surface: cosmetic).

平坦な地図の立体交差の3Dモデル: 持ち上げた車線から橋面・標示・高欄・擁壁・桁・橋脚を作る。
"""

import math

import numpy as np
import shapely
from scipy.spatial import cKDTree
from shapely.geometry import LineString, Polygon
from shapely.ops import substring, unary_union

from mesh import Mesh, polys

Z0 = 0.02            # a lane higher than this is raised / これより高い車線は持ち上げ
GUTTER = 0.5         # deck beyond the outer lane edge / 外側車線の外の路肩
PARA_Z = 0.3         # parapets where the deck is this high / 高欄を付ける高さ
PARA_W, PARA_H = 0.25, 1.0
OPEN_Z = 4.5         # higher than this: slab on piers / これより高ければ桁と橋脚
SLAB = 1.0           # slab thickness / 桁の厚さ
PIER_STEP = 24.0     # pier spacing along a raised road / 橋脚の間隔
PIER_L, PIER_WMAX = 1.6, 4.0
TREE_CLEAR = 6.0     # props keep trees / lamps this far off a deck (build.py keepout) / 樹木・街灯の離隔
GRID = 2.0           # deck triangles on this grid (they follow the ramps) / 橋面の分割
PAINT = 0.01
DASH, GAP, LINE_W = 3.0, 6.0, 0.12


class Flyovers(object):
    def __init__(self, L):
        lanes = [l for l in L.lanes if l.get("kind", "driving") == "driving" and len(l["pts"]) >= 2]
        raised = [l for l in lanes if max(p.get("z", 0.0) for p in l["pts"]) > Z0]
        self.n_lanes = len(raised)
        self.decks = []
        if not raised:
            self.lower = Polygon()
            return
        # chunks of 4 m along each raised lane / 車線を 4 m ごとに区切る
        chunks = []                   # (polygon, zmin, zmax, lane index)
        low_parts = []                # the at-grade parts of raised lanes / 持ち上げ車線の地上部分
        for li, l in enumerate(raised):
            P = l["pts"]
            for i in range(0, len(P) - 1, 4):
                q = P[i:i + 5]
                if len(q) < 2:
                    continue
                zs = [p.get("z", 0.0) for p in q]
                pg = Polygon([p["l"] for p in q] + [p["r"] for p in q][::-1]).buffer(0)
                if max(zs) <= Z0:
                    if not pg.is_empty:
                        low_parts.append(pg)              # its part at grade / 地上の部分
                    continue
                if pg.is_empty:
                    continue
                chunks.append((pg, min(zs), max(zs), li))
        # connected groups at one height / 同じ高さでつながる塊
        geoms = [c[0].buffer(GUTTER + 0.1) for c in chunks]
        tree = shapely.STRtree(geoms)
        parent = list(range(len(chunks)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        a_, b_ = tree.query(geoms, predicate="intersects")
        for i, j in zip(a_, b_):
            if i < j:
                zi = 0.5 * (chunks[i][1] + chunks[i][2])
                zj = 0.5 * (chunks[j][1] + chunks[j][2])
                if abs(zi - zj) < 1.0 or chunks[i][3] == chunks[j][3]:
                    pi, pj = find(i), find(j)
                    if pi != pj:
                        parent[pj] = pi
        groups = {}
        for i in range(len(chunks)):
            groups.setdefault(find(i), []).append(i)
        # the lanes at grade (what must stay clear under a span) / 地上の車線（桁下に残す）
        low = [Polygon([p["l"] for p in l["pts"]] + [p["r"] for p in l["pts"]][::-1]).buffer(0)
               for l in lanes if max(p.get("z", 0.0) for p in l["pts"]) <= Z0]
        self.lower = unary_union([g for g in low if not g.is_empty] + low_parts)
        lower_prep = self.lower.buffer(1.0)
        for gi, idx in groups.items():
            fp = unary_union([chunks[i][0] for i in idx]).buffer(GUTTER, join_style="mitre", mitre_limit=2.0)
            fp = fp.simplify(0.02)
            lis = sorted({chunks[i][3] for i in idx})
            C, Z = [], []
            for li in lis:
                P = raised[li]["pts"]
                for p, q in zip(P, P[1:]):
                    n = max(1, int(math.ceil(math.hypot(q["c"][0] - p["c"][0], q["c"][1] - p["c"][1]) / 0.25)))
                    for k in range(n):
                        f = k / n
                        C.append((p["c"][0] + f * (q["c"][0] - p["c"][0]), p["c"][1] + f * (q["c"][1] - p["c"][1])))
                        Z.append(p.get("z", 0.0) + f * (q.get("z", 0.0) - p.get("z", 0.0)))
                C.append(tuple(P[-1]["c"]))
                Z.append(P[-1].get("z", 0.0))
            kd = cKDTree(np.array(C))
            Z = np.array(Z)
            zf = (lambda kd_, Z_: (lambda x, y: Z_[kd_.query(np.column_stack([np.atleast_1d(x), np.atleast_1d(y)]))[1]]))(kd, Z)
            # the parapets take the height of the nearest lane that has them: at a crossing the
            # nearest lane centre was a ramp foot at ~0 m and the parapet top fell from 7.5 m to
            # 1 m over the road below (check_drive blocked) / 高欄は高欄のある車線の高さで
            hp = Z > PARA_Z
            zp = zf if not hp.any() or hp.all() else \
                (lambda kd_, Z_: (lambda x, y: Z_[kd_.query(np.column_stack([np.atleast_1d(x), np.atleast_1d(y)]))[1]]))(
                    cKDTree(np.array(C)[hp]), Z[hp])
            high = unary_union([chunks[i][0] for i in idx if chunks[i][1] >= OPEN_Z]).buffer(GUTTER + 1.0)
            under = fp.intersection(lower_prep)
            # a low deck over / beside a lane at grade (a merge's ramp foot): neither slab nor
            # walls there (a slab 1 m over the lower road: check_roads) / 低い橋面の下に桁も壁も作らない
            low = unary_union([chunks[i][0] for i in idx if chunks[i][1] < 3.6]).buffer(GUTTER + 0.5)
            lowunder = fp.intersection(under.buffer(2.0)).intersection(low)
            opn = fp.intersection(unary_union([high, under.buffer(2.0).difference(lowunder)]))
            solid = fp.difference(opn).difference(lowunder)
            para = unary_union([chunks[i][0] for i in idx if chunks[i][2] > PARA_Z]).buffer(GUTTER + 1.0)
            pband = fp.buffer(PARA_W, join_style="mitre", mitre_limit=2.0).difference(fp).intersection(para)
            ctree = shapely.STRtree([chunks[i][0] for i in idx])
            # junctions on the deck are level: one flat fill over their lanes (+ gutter)
            # covers what folded turnaround lanes leave open (D'Resort drop-off head)
            # 橋面上の交差点は水平: 車線＋路肩を一枚で覆う（折れた転回車線の穴を塞ぐ）
            jfill = unary_union([chunks[i][0] for i in idx if raised[chunks[i][3]]["junction"] >= 0])
            jfill = jfill.buffer(GUTTER, join_style="mitre", mitre_limit=2.0).intersection(fp) \
                if not jfill.is_empty else Polygon()
            self.decks.append({"fp": fp, "zf": zf, "zp": zp, "open": opn, "solid": solid, "para": pband,
                               "lanes": [raised[li] for li in lis], "ctree": ctree, "jfill": jfill})
        self.footprint = unary_union([d["fp"] for d in self.decks])
        # every raised lane piece too: a pier must not stand in a lane passing under its
        # deck (a hotel ramp under the viaduct; check_drive "blocked") / 桁下を通る持ち上げ車線も避ける
        self.chunks = chunks
        self.chunk_tree = shapely.STRtree([c[0] for c in chunks])
        # the carriageway at grade (build.py's Layout): no wall or parapet stands on it
        # (check_roads: a ramp-foot wall reached over Upper Changi Road East) / 地上の車道に壁を立てない
        cw = getattr(L, "carriageway", None)
        self.ground_road = cw.difference(self.footprint.buffer(0.3)) if cw is not None else None
        if self.ground_road is not None:
            self.ground_road_m = self.ground_road.buffer(0.1)
            shapely.prepare(self.ground_road)
        # every lane piece near a deck (at grade too), for the conflict test: deck extras
        # (gutters, fills, parapets) never lie 0.1-2.7 m over or under another lane
        # 橋面の付属物（路肩・充填・高欄）は他の車線の 0.1-2.7 m 上下に置かない
        near = self.footprint.buffer(10.0)
        shapely.prepare(near)
        oc = []
        for l in lanes:
            P = l["pts"]
            for i in range(0, len(P) - 1, 2):
                q = P[i:i + 3]
                if len(q) < 2 or not near.intersects(shapely.Point(q[0]["c"])):
                    continue
                pg = Polygon([p["l"] for p in q] + [p["r"] for p in q][::-1]).buffer(0)
                if not pg.is_empty:
                    oc.append((pg, float(np.mean([p.get("z", 0.0) for p in q]))))
        self.other = oc
        self.other_tree = shapely.STRtree([c[0] for c in oc]) if oc else None
        self.piers = self._piers()

    def _piers(self):
        out = []
        for d in self.decks:
            if d["open"].is_empty:
                continue
            seen = {}
            for l in d["lanes"]:
                seen.setdefault(l["road"], []).append(l)
            for road, ls in seen.items():
                mid = sorted(ls, key=lambda l: abs(l["lane"]))[len(ls) // 2]
                P = mid["pts"]
                width = sum(np.median([p["w"] for p in l["pts"]]) for l in ls)
                C = LineString([p["c"] for p in P])
                s = PIER_STEP / 2
                while s < C.length:
                    p = C.interpolate(s)
                    q = C.interpolate(min(C.length, s + 1.0))
                    s += PIER_STEP
                    z = float(d["zf"](p.x, p.y)[0])
                    if z < OPEN_Z or not d["open"].contains(p):
                        continue
                    ang = math.atan2(q.y - p.y, q.x - p.x)
                    wcross = min(max(width - 1.0, 1.2), PIER_WMAX)
                    c, sn = math.cos(ang), math.sin(ang)
                    hx, hy = PIER_L / 2, wcross / 2
                    # centred across the road's lanes: the middle lane's centre shifted half way
                    # 道路の車線の中央に置く
                    foot = Polygon([(p.x + c * x - sn * y, p.y + sn * x + c * y)
                                    for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))])
                    fb = foot.buffer(0.8)
                    if fb.intersects(self.lower):
                        continue                          # a lower road under it / 下の道路を塞がない
                    if any(self.chunks[k][1] < z - 1.0 for k in self.chunk_tree.query(fb, predicate="intersects")):
                        continue                          # a raised lane lower than this deck / 低い持ち上げ車線
                    out.append((foot, z - SLAB))
        return out

    def conflicts(self, geom, z, lo=0.1, hi=2.7):
        """Lane pieces under / over ``geom`` whose z differs from z (a number or a
        function of x, y at the piece's centre) by lo..hi m. / 高さが lo..hi m 違う車線片"""
        if self.other_tree is None:
            return []
        out = []
        for k in self.other_tree.query(geom, predicate="intersects"):
            pg, zc = self.other[k]
            c = pg.centroid
            zs = z(c.x, c.y) if callable(z) else z
            if lo < abs(float(np.atleast_1d(zs)[0]) - zc) < hi:
                out.append(pg)
        return out

    def _walls(self, m, d, ring, outward, tile, bottom, top, mat="concrete"):
        """Walls along a ring, split <= GRID m, bottom/top functions of (x, y). / 輪に沿う壁"""
        c = list(ring.coords)
        if not Polygon(c).exterior.is_ccw:
            c = c[::-1]
        if not outward:
            c = c[::-1]
        for a, b in zip(c, c[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if L < 1e-3:
                continue
            n = max(1, int(math.ceil(L / GRID)))
            for k in range(n):
                p0 = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                p1 = (a[0] + (b[0] - a[0]) * (k + 1) / n, a[1] + (b[1] - a[1]) * (k + 1) / n)
                mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
                if not (tile.bounds[0] <= mx < tile.bounds[2] and tile.bounds[1] <= my < tile.bounds[3]):
                    continue
                b0, b1 = bottom(*p0), bottom(*p1)
                t0, t1 = top(*p0), top(*p1)
                if abs(t1 - t0) > 1.0:
                    continue                  # lanes of another height meet here: no slanted sheet / 段差
                mid = shapely.Point(mx, my)
                if self.ground_road is not None and self.ground_road.distance(LineString([p0, p1])) < 0.1:
                    continue                  # on the carriageway at grade / 地上の車道の上
                if self.other_tree is not None and any(
                        min(b0, b1) - 0.1 < self.other[k][1] < max(t0, t1) - 0.12
                        for k in self.other_tree.query(mid, predicate="within")):
                    continue                  # it would stand in another lane / 他の車線の中
                b0 = b1 = min(b0, b1)         # vertical faces, no slanted bottoms / 底は水平
                if max(t0 - b0, t1 - b1) < 0.01:
                    continue
                m.quad(mat, (p0[0], p0[1], b0), (p1[0], p1[1], b1), (p1[0], p1[1], t1), (p0[0], p0[1], t0),
                       (0, b0), (L / n, b1), (L / n, t1), (0, t0))

    def _surface(self, m, d, tile):
        """The deck surface: each raised lane as its own strip between its edges, at its
        own z (laterally level, exactly the road logic), and a GUTTER strip beyond an
        edge with no raised lane outside it. Overlapping lanes (junctions, merges) lie
        at one height there: the same asphalt with world UVs looks the same.
        橋面: 車線ごとの帯（その車線の高さ）、外側の縁には路肩の帯。"""
        x0, y0, x1, y1 = tile.bounds
        def ext(p, q, dist):
            """p moved dist m away from q (lanes.json leaves ~1 cm between roads). / 端を延ばす"""
            v = np.array(p["c"]) - np.array(q["c"])
            n = np.linalg.norm(v)
            if n < 1e-6:
                return p
            v = v / n * dist
            return dict(p, c=list(np.array(p["c"]) + v), l=list(np.array(p["l"]) + v), r=list(np.array(p["r"]) + v))
        for l in d["lanes"]:
            P = list(l["pts"])
            if len(P) >= 2:
                P[0], P[-1] = ext(P[0], P[1], 0.05), ext(P[-1], P[-2], 0.05)
            for a, b in zip(P, P[1:]):
                za, zb = a.get("z", 0.0), b.get("z", 0.0)
                if max(za, zb) <= Z0:
                    continue
                mx, my = (a["c"][0] + b["c"][0]) / 2, (a["c"][1] + b["c"][1]) / 2
                if not (x0 <= mx < x1 and y0 <= my < y1):
                    continue
                quads = [((a["r"], za), (b["r"], zb), (b["l"], zb), (a["l"], za))]
                for side, other in (("l", "r"), ("r", "l")):
                    ea, eb = np.array(a[side]), np.array(b[side])
                    oa = ea - np.array(a[other])
                    ob = eb - np.array(b[other])
                    na, nb_ = np.linalg.norm(oa), np.linalg.norm(ob)
                    if na < 1e-3 or nb_ < 1e-3:
                        continue
                    oa, ob = oa / na, ob / nb_
                    probe = shapely.Point(*((ea + eb) / 2 + (oa + ob) / 2 * 0.6))
                    if len(d["ctree"].query(probe, predicate="intersects")):
                        continue                          # another raised lane outside / 外にも車線
                    gq = Polygon([tuple(ea), tuple(eb), tuple(eb + ob * GUTTER), tuple(ea + oa * GUTTER)])
                    if gq.is_valid and self.conflicts(gq.buffer(-0.05), 0.5 * (za + zb)):
                        continue                          # over / under another lane / 他の車線の上下
                    quads.append(((tuple(ea), za), (tuple(eb), zb), (tuple(eb + ob * GUTTER), zb),
                                  (tuple(ea + oa * GUTTER), za)))
                for q in quads:
                    V = [(p[0][0], p[0][1], p[1]) for p in q]
                    n = (V[1][0] - V[0][0]) * (V[3][1] - V[0][1]) - (V[1][1] - V[0][1]) * (V[3][0] - V[0][0])
                    if n < 0:
                        V = V[::-1]
                    m.quad("asphalt", V[0], V[1], V[2], V[3], *[(v[0] / 2.0, v[1] / 2.0) for v in V])

    def build(self, tile):
        """{kind: Mesh} of the flyovers in one ground tile. / タイル内のメッシュ"""
        if not self.decks:
            return {}
        road, paint, struct = Mesh(), Mesh(), Mesh()
        for m in (paint,):
            m.dense = GRID
            m.zfun = lambda x, y: np.zeros(len(np.atleast_1d(x)))
        for d in self.decks:
            if not d["fp"].buffer(PARA_W + 0.1).intersects(tile):
                continue
            zf = d["zf"]
            z1 = lambda x, y, zf=zf: float(zf(x, y)[0])
            self._surface(road, d, tile)
            # sharp bends: the inner edge of a lane folds back (a strip twists, holes):
            # the bend's hull filled flat at the lane's z / 急な曲がりは内側の縁が折り返す: 凸包で塞ぐ
            kinks = []
            for l in d["lanes"]:
                P = l["pts"]
                for i in range(1, len(P) - 1):
                    dy = abs((P[i + 1]["yaw"] - P[i - 1]["yaw"] + 180.0) % 360.0 - 180.0)
                    if dy > 8.0 and P[i].get("z", 0.0) > Z0:
                        q = P[max(0, i - 2):i + 3]
                        hull = shapely.MultiPoint([p_[k] for p_ in q for k in ("l", "r")]).convex_hull
                        if hull.geom_type == "Polygon":
                            kinks.append(hull)
            if kinks:
                kf = unary_union(kinks).intersection(tile)
                if not kf.is_empty:
                    cf = self.conflicts(kf, zf)
                    if cf:
                        kf = kf.difference(unary_union(cf).buffer(0.1))
                if not kf.is_empty:
                    road.flat("asphalt", kf, 0.0, uv_scale=2.0, zf=lambda x, y, zf=zf: zf(x, y))
            jf = d["jfill"].intersection(tile) if not d["jfill"].is_empty else Polygon()
            if not jf.is_empty:
                cf = self.conflicts(jf, zf)
                if cf:
                    jf = jf.difference(unary_union(cf).buffer(0.1))
            if not jf.is_empty:
                road.flat("asphalt", jf, 0.0, uv_scale=2.0, zf=lambda x, y, zf=zf: zf(x, y))
            # markings from the lanes' own marks / 車線の区画線
            white, yellow = [], []
            for l in d["lanes"]:
                P = l["pts"]
                for side, key in (("l", "ml"), ("r", "mr")):
                    run = []
                    for p in P + [None]:
                        mk = p.get(key) if p is not None else None
                        ok = p is not None and p.get("z", 0.0) > Z0 and mk and mk[0] not in ("NONE", "None", "none")
                        if ok and (not run or run[-1][1] == mk):
                            run.append((p[side], mk))
                            continue
                        if len(run) >= 2:
                            line = LineString([q[0] for q in run])
                            typ, col = run[0][1][0].lower(), (run[0][1][1] or "white").lower()
                            if "broken" in typ:
                                pieces, s = [], 0.0
                                while s < line.length:
                                    pieces.append(substring(line, s, min(line.length, s + DASH)))
                                    s += DASH + GAP
                            else:
                                pieces = [line]
                            g = [q.buffer(LINE_W / 2, cap_style="flat") for q in pieces if q.length > 0.05]
                            (yellow if "yellow" in col else white).extend(g)
                        run = [(p[side], mk)] if ok else []
            wpoly = unary_union(white).intersection(d["fp"]).intersection(tile) if white else Polygon()
            ypoly = unary_union(yellow).difference(wpoly.buffer(0.02)).intersection(d["fp"]).intersection(tile) \
                if yellow else Polygon()
            paint.flat("paint_white", wpoly, 0.0, zf=lambda x, y, zf=zf: zf(x, y) + PAINT)
            paint.flat("paint_yellow", ypoly, 0.0, zf=lambda x, y, zf=zf: zf(x, y) + PAINT)
            # parapets: walls round the band and its top / 高欄
            opn = d["open"]
            ob = opn.buffer(0.6)
            shapely.prepare(ob)
            zp = d["zp"]
            zp1 = lambda x, y, zp=zp: float(zp(x, y)[0])
            bottom_para = lambda x, y, ob=ob: (max(0.0, zp1(x, y) - SLAB)
                                               if ob.contains(shapely.Point(x, y)) else 0.0)
            for pg in polys(d["para"]):
                self._walls(struct, d, pg.exterior, True, tile, bottom_para, lambda x, y: zp1(x, y) + PARA_H)
                for r in pg.interiors:
                    self._walls(struct, d, r, False, tile, bottom_para, lambda x, y: zp1(x, y) + PARA_H)
            top = d["para"].intersection(tile)
            if not top.is_empty:
                cf = self.conflicts(top, zp)
                if cf:
                    top = top.difference(unary_union(cf).buffer(0.1))
                if self.ground_road is not None:
                    top = top.difference(self.ground_road_m)      # (a margin: touching counts) / 余裕
            if not top.is_empty:
                tm = Mesh()
                tm.dense, tm.zfun = GRID, (lambda x, y: np.zeros(len(np.atleast_1d(x))))
                tm.flat("concrete", top, 0.0, zf=lambda x, y, zp=zp: zp(x, y) + PARA_H)
                struct.merge(tm)
            # embankment walls round the solid part (under the parapet line) / 擁壁
            for pg in polys(d["solid"]):
                for r, out in [(pg.exterior, True)] + [(q, False) for q in pg.interiors]:
                    self._walls(struct, d, r, out, tile, lambda x, y: 0.0, lambda x, y: max(0.0, z1(x, y) - 0.005))
            # open spans: slab underside and fascia / 桁の下面と側面
            sp = opn.intersection(tile)
            if not sp.is_empty:
                um = Mesh()
                um.dense, um.zfun = GRID, (lambda x, y: np.zeros(len(np.atleast_1d(x))))
                um.flat("concrete", sp, 0.0, up=False, zf=lambda x, y, zf=zf: np.maximum(zf(x, y) - SLAB, 0.0))
                struct.merge(um)
        for foot, ztop in self.piers:
            c = foot.centroid
            if tile.bounds[0] <= c.x < tile.bounds[2] and tile.bounds[1] <= c.y < tile.bounds[3]:
                struct.prism("concrete", foot, 0.0, ztop, top=False)
        return {"Road_RoadDeck": road, "Road_MarkingDeck": paint, "Terrain_Flyover": struct}

    def summary(self):
        if not self.decks:
            return "flyovers: none"
        area = sum(d["fp"].area for d in self.decks)
        op = sum(d["open"].area for d in self.decks)
        return ("flyovers: %d raised lanes in %d decks, %.0f m2 (%.0f m2 open spans on %d piers, %.0f m2 embankment)"
                % (self.n_lanes, len(self.decks), area, op, len(self.piers), area - op))
