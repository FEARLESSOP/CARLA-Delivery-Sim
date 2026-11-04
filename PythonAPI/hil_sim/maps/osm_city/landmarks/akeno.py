"""Akeno roadside features OSM does not have (maps.json "area_features"),
on the main road between the 7-Eleven and Oita Kosen (maps/.venv).

The OSM data there is out of date (z20 satellite 2026, Google Earth 3D, the
user's photo of 2026-11-04; build/OitaKosen/seven_area_photos). This module
draws what stands there now, from the maps.json entry, in the onterrain way
(guide ch. 15): its own draped ground, platforms with retaining walls, nothing
on or over the road network.

  greenhouses   ranges of multi-span vinyl houses (ridge-and-furrow roofs,
                grey-white frames, translucent cladding drawn as texture), one
                level platform per block at the highest ground under it
                (concrete retaining wall down to the lowest), soil tracks
                between the blocks ("farm_ground")
  tree_rows     rows of dense evergreen trees (camphor, クスノキ) along the
                main road's west kerb and the windbreak between the ranges;
                added to the generic tree list (build.py), so they take the
                seasons and the road-clearance rules of every other tree
  sand_pitch    a sand football ground on its OSM pitch outline (no sport
                tag in OSM): sand on a level platform (median ground, banks),
                white line markings, goals, the tall ball-stop net of steel
                poles along the given sides of the platform edge, chain-link
                along the rest
  car_parks     an asphalt car park with Japanese U-shaped bay lines

Every entry is in lon/lat (WGS84). Nothing here is a building of OSM; the
buildings that replace OSM's out-of-date ones go through maps.json
"osm_fixes" and buildings_jp (General Builder).

明野の幹線沿い（セブン-イレブン〜大分高専）の OSM に無い地物: ビニールハウス群・
防風樹列・砂のグラウンド（防球ネット）・駐車場。maps.json "area_features"。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import geo
from buildings_jp import face_out
from landmarks import onterrain
from landmarks.kosen import back, fit_rect, split_tiles
from landmarks.seven import drop_slivers, u_divider
from mesh import Mesh, polys

NAME = "明野 幹線沿い (ハウス群・第2グラウンド・駐車場)"
TAG = "AKENO"
TRACE = None
KEY = "area_features"
GAP = 0.3                # platforms and things to roads, kerbs, paths / 道路・歩道との間隔
CLEAR = 0.4              # greenhouse walls to them / ハウスの壁と道路の間隔
PAINT = 0.006            # line paint over the ground / 標示の高さ
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def config(meta):
    """The maps.json "area_features" of this map, or None. / 地図の area_features。"""
    try:
        cfg = json.load(open(os.path.join(HERE, "..", "maps.json")))
    except (OSError, ValueError):
        return None
    for v in cfg.values():
        if isinstance(v, dict) and v.get("name") == meta.get("name"):
            # AKENO_WIP=1: the work-in-progress entry (scratch builds only) / 作業中の設定（試験ビルドのみ）
            return (v.get("_" + KEY + "_wip") if os.environ.get("AKENO_WIP") else v.get(KEY)) or None
    return None


def enabled(out_dir, meta):
    return config(meta) is not None


def greenhouse(m, o, ax, A, B, eave=3.0, rise=1.0, span=4.0, kick=0.35):
    """A multi-span greenhouse on the rectangle o + s ax + r ay (s 0..A along
    the ridges, r 0..B across, ay left of ax), floor at z 0: side and end walls
    (a concrete kick strip, film above), gable ends, two roof slopes per span.
    Every face points out of the volume. / 連棟ハウス（外向きの面のみ）。"""
    o, ax = np.asarray(o, float), np.asarray(ax, float) / np.linalg.norm(ax)
    ay = np.array([-ax[1], ax[0]])
    n = max(1, int(round(B / span)))
    w = B / n
    P = lambda s, r, z: (o[0] + ax[0] * s + ay[0] * r, o[1] + ax[1] * s + ay[1] * r, z)
    X, Y = (ax[0], ax[1], 0.0), (ay[0], ay[1], 0.0)
    nX, nY = (-ax[0], -ax[1], 0.0), (-ay[0], -ay[1], 0.0)
    for z0, z1, mat in ((0.0, kick, "akeno_gh_kick"), (kick, eave, "akeno_gh_wall")):
        face_out(m, mat, [P(0, 0, z0), P(A, 0, z0), P(A, 0, z1), P(0, 0, z1)], nY, 3.0)
        face_out(m, mat, [P(0, B, z0), P(A, B, z0), P(A, B, z1), P(0, B, z1)], Y, 3.0)
        face_out(m, mat, [P(0, 0, z0), P(0, B, z0), P(0, B, z1), P(0, 0, z1)], nX, 3.0)
        face_out(m, mat, [P(A, 0, z0), P(A, B, z0), P(A, B, z1), P(A, 0, z1)], X, 3.0)
    for k in range(n):
        r0, r1, rm = k * w, (k + 1) * w, (k + 0.5) * w
        for s, out in ((0.0, nX), (A, X)):
            face_out(m, "akeno_gh_wall", [P(s, r0, eave), P(s, r1, eave), P(s, rm, eave + rise)], out, 3.0)
        zr = eave + rise
        face_out(m, "akeno_gh_roof", [P(0, r0, eave), P(A, r0, eave), P(A, rm, zr), P(0, rm, zr)], (0.0, 0.0, 1.0), 3.0)
        face_out(m, "akeno_gh_roof", [P(0, rm, zr), P(A, rm, zr), P(A, r1, eave), P(0, r1, eave)], (0.0, 0.0, 1.0), 3.0)
    return n


def pitch_lines(c, axis, length, width, lw=0.12):
    """Football pitch markings (FIFA layout scaled to the field) as polygons.
    サッカーのライン（多角形）。"""
    cx, cy = c
    ca, sa = math.cos(axis), math.sin(axis)
    T = lambda u, v: (cx + ca * u - sa * v, cy + sa * u + ca * v)
    hl, hw = length / 2, width / 2
    L = lambda pts: LineString([T(*p) for p in pts]).buffer(lw / 2, cap_style="flat", join_style="mitre")
    parts = [L([(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)]), L([(0, -hw), (0, hw)])]
    ring = lambda r, a0=0.0, a1=2 * math.pi, du=0.0: LineString(
        [T(du + r * math.cos(a), r * math.sin(a)) for a in np.linspace(a0, a1, 40)]).buffer(lw / 2, cap_style="flat")
    parts.append(ring(9.15))
    parts.append(Point(T(0, 0)).buffer(0.15, 8))
    for s in (-1, 1):
        g = hl * s
        pb, ga = min(16.5, length * 0.16), min(5.5, length * 0.06)
        parts.append(L([(g, -20.16), (g - s * pb, -20.16), (g - s * pb, 20.16), (g, 20.16)]))
        parts.append(L([(g, -9.16), (g - s * ga, -9.16), (g - s * ga, 9.16), (g, 9.16)]))
        spot = g - s * 11.0
        parts.append(Point(T(spot, 0)).buffer(0.12, 8))
        # the arc outside the penalty area / ペナルティアーク
        a = math.acos(min(1.0, (pb - 11.0) / 9.15))
        a0 = (math.pi - a, math.pi + a) if s > 0 else (-a, a)
        parts.append(ring(9.15, a0[0], a0[1], spot))
    return unary_union(parts)


class AKENO(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        self.out_dir = out_dir
        self.fr = geo.Frame(self.meta)
        self.cfg = config(self.meta) or {}
        self.drape = None                      # set by build.py / build.py が設定
        P = lambda q: self.fr.xy(float(q[0]), float(q[1]))
        self.P = P
        PG = lambda pts: orient(Polygon([P(q) for q in pts]).buffer(0), 1.0)
        # the road network and its paths (copied before build.py clears the site)
        # 道路網と歩道（build.py が敷地を消す前の写し）
        self.road = L.road_all
        self.carriageway = L.carriageway
        self.paths = drop_slivers(unary_union([L.footpaths, L.cycleways, L.driveways]))
        self.no_go = unary_union([self.road, self.paths, L.islands])
        shapely.prepare(self.no_go)
        keep = self.no_go.buffer(CLEAR)
        self.blocks = {}
        # greenhouses / ハウス
        self.houses = []
        for g in self.cfg.get("greenhouses", []):
            seen = PG(g["lonlat"])
            q = seen.difference(keep).difference(L.building_area.buffer(0.5))
            parts = [p for p in polys(q) if p.area > 20.0]
            if not parts:
                print("landmark %s: greenhouse %s dropped (on the road network)" % (TAG, g.get("id")))
                continue
            q = max(parts, key=lambda p: p.area)
            if q.area < seen.area - 0.5:
                q = fit_rect(orient(q.simplify(0.02), 1.0))
                print("landmark %s: greenhouse %s cut back %.0f m2 (roads win)" % (TAG, g.get("id"), seen.area - q.area))
            if q.area < 0.3 * seen.area:
                continue
            self.houses.append(dict(g, poly=orient(q, 1.0)))
            self.blocks["gh_" + str(g.get("id"))] = q
        farm = unary_union([PG(p) for p in self.cfg.get("farm_ground", [])] + [h["poly"] for h in self.houses])
        self.farm = drop_slivers(farm.difference(self.no_go.buffer(GAP)), 0.3)
        # the sand pitch / 砂のグラウンド
        self.pitches = []
        for d in self.cfg.get("sand_pitch", []):
            refs = L.osm.ways.get(str(d["way"]), (None,))[0]
            p = L.osm.poly(refs) if refs else None
            if p is None:
                continue
            g = drop_slivers(p.difference(self.no_go.buffer(GAP)), 0.3)
            if g.is_empty:
                continue
            g = max(polys(g), key=lambda q: q.area)
            self.pitches.append(dict(d, poly=g))
            self.blocks["pitch_" + str(d["way"])] = g
        # car parks / 駐車場
        self.carparks = []
        for d in self.cfg.get("car_parks", []):
            g = drop_slivers(PG(d["lonlat"]).difference(self.no_go).difference(L.building_area.buffer(0.3)), 0.3)
            if not g.is_empty:
                self.carparks.append(dict(d, poly=g))
                self.blocks["carpark_%d" % len(self.carparks)] = g
        self.site = orient(unary_union([self.farm] + [p["poly"] for p in self.pitches] +
                                       [c["poly"] for c in self.carparks]).buffer(0), 1.0) \
            if self.blocks else Polygon()
        self.footprint = unary_union([h["poly"] for h in self.houses]) if self.houses else Polygon()
        self.tree_list, self.tree_clear = self.plan_trees()
        self._built = None

    # -- what build.py needs / build.py 用
    def inner(self):
        return self.site

    def region(self):
        return self.site

    def ground_taken(self):
        return self.site

    def keepout(self):
        """No generic trees or street furniture on the features. / 汎用の木・街具を置かない。"""
        return self.site

    def lot_clear(self):
        """Building lots (buildings_jp) keep off the features. / 一般建物の敷地はここを避ける。"""
        return self.site.buffer(0.3)

    def trees(self):
        """(x, y, kind, size) for props.Props.trees: the tree rows. / 樹列（汎用樹木リストへ）。"""
        return list(self.tree_list)

    def ground_mesh(self, tile, z):
        self.build_all()
        return self._built["ground"].get(tile.bounds, Mesh())

    def buildings_mesh(self, tile):
        self.build_all()
        return self._built["bld"].get(tile.bounds, Mesh())

    # -- trees / 樹列
    def plan_trees(self):
        """Trees every ``spacing`` m along each row (seeded jitter), each at least
        ``road_room`` m from the carriageway (its crown then stays off the road:
        props.py shrinks it to the room left), off paths, buildings and the other
        features. / 樹列: 車道から road_room 以上、歩道・建物・他の地物を避ける。"""
        out, clear = [], []
        L = self.L
        bad = unary_union([self.no_go.buffer(0.8), L.building_area.buffer(1.5), self.site.buffer(0.5)])
        shapely.prepare(bad)
        for k, d in enumerate(self.cfg.get("tree_rows", [])):
            line = LineString([self.P(q) for q in d["lonlat"]])
            sp = float(d.get("spacing", 3.5))
            room = float(d.get("road_room", 6.6))
            kind = d.get("species", "camphor") + ("_lite" if d.get("lite") else "")
            s0 = float(d.get("height", 7.0)) / 11.0          # camphor: 11 m at size 1 / クスノキ基準 11 m
            rs = np.random.default_rng(907 + k)
            n = 0
            for i in range(int(line.length // sp) + 1):
                q = line.interpolate(min(line.length, i * sp + float(rs.uniform(-0.4, 0.4))))
                lat = float(d.get("lateral", 0.3))
                x, y = q.x + float(rs.uniform(-1, 1)) * lat, q.y + float(rs.uniform(-1, 1)) * lat
                if self.carriageway.distance(Point(x, y)) < room or bad.contains(Point(x, y)) \
                        or not L.inside.contains(Point(x, y)):
                    continue
                out.append((x, y, kind, s0 * float(rs.uniform(0.9, 1.1))))
                n += 1
            clear.append(line.buffer(max(2.5, sp)))
            print("landmark %s: tree row %s: %d trees" % (TAG, d.get("id", k), n))
        return out, (unary_union(clear) if clear else Polygon())

    # -- terrain / 地形
    def plan(self):
        if getattr(self, "S", None) is not None:
            return self.S
        S = onterrain.Site(self.drape, self.region(), no_go=self.no_go, gap=GAP)
        for h in self.houses:
            # a building: the highest ground under it, a wall down to the lowest
            # 建物: 最高地盤、最低地盤まで擁壁
            h["plat"] = S.platform("gh_%s" % h.get("id"), h["poly"], how="max", kind="building")
        for p in self.pitches:
            core = p["poly"].buffer(-0.6, join_style="mitre")
            p["plat"] = S.platform("pitch", core, how="median", kind="court", bank=True) if not core.is_empty else None
        self.S = S
        return S

    def follow(self):
        m = Mesh()
        m.zfun = lambda x, y: self.S.surface_z(x, y)
        m.dense = 2.0
        return m

    def rigid(self, x, y, r=0.2):
        m = Mesh()
        m.zoff = self.S.low(x, y, r)
        return m

    # -- assembly / 組み立て
    def build_all(self):
        if self._built is not None:
            return
        import ground as G
        S = self.plan()
        gm, bm = Mesh(), Mesh()
        # ground: platforms flat, the rest draped / 地面
        paved = unary_union([c["poly"] for c in self.carparks]) if self.carparks else Polygon()
        S.ground(gm, "soil", self.farm.difference(paved), uv_scale=4.0)
        for p in self.pitches:
            plat = p.get("plat")
            if plat is not None:
                S.ground(gm, "akeno_sand", plat.poly, uv_scale=6.0)
                ln = p.get("lines")
                if ln:
                    lines = pitch_lines(self.P(ln["centre"]), math.radians(90.0 - float(ln.get("axis_deg", 0.0))),
                                        float(ln["length"]), float(ln["width"]))
                    S.ground(gm, "line_white", lines.intersection(plat.poly.buffer(-0.2)), dz=PAINT, uv_scale=1.0)
            rest = S.free().intersection(p["poly"])
            S.ground(gm, "grass", rest, uv_scale=4.0)
        for c in self.carparks:
            S.ground(gm, "asphalt_lot", c["poly"], uv_scale=6.0)
            S.ground(gm, "line_white", self.bays(c), dz=PAINT, uv_scale=1.0)
        S.walls(gm, mat="jp_retaining", bank_mat="grass_park")
        nspan = 0
        for h in self.houses:
            r = orient(h["poly"].minimum_rotated_rectangle, 1.0)
            c = np.array(r.exterior.coords[:4])
            e = [c[(i + 1) % 4] - c[i] for i in range(4)]
            want = np.array([0.0, 1.0]) if h.get("ridge", "ns") == "ns" else np.array([1.0, 0.0])
            k = max(range(4), key=lambda i: abs(float(e[i] @ want)) / np.linalg.norm(e[i]))
            o, ax = c[k], e[k]
            A = float(np.linalg.norm(ax))
            B = float(np.linalg.norm(e[(k + 1) % 4]))
            hm = Mesh()
            nspan += greenhouse(hm, o, ax, A, B, eave=float(h.get("eave", 3.0)), rise=float(h.get("rise", 1.0)),
                                span=float(h.get("span", 4.0)))
            hm.shift_z(h["plat"].z if h.get("plat") is not None else S.level(h["poly"]))
            bm.merge(hm)
        nets = 0.0
        for p in self.pitches:
            nets += self.pitch_things(bm, p)
        tiles = [tb for tn, tb in G.tiles(self.meta)]
        self._built = {"ground": split_tiles(gm, tiles), "bld": split_tiles(bm, tiles)}
        print("landmark %s: %d greenhouse blocks (%d spans, %.0f m2), farm ground %.0f m2, pitches %d, car parks %d "
              "(%d bays), ball-stop net %.0f m; terrain %s; ground %d, buildings %d triangles"
              % (TAG, len(self.houses), nspan, sum(h["poly"].area for h in self.houses), self.farm.area,
                 len(self.pitches), len(self.carparks), getattr(self, "nbays", 0), nets, S.report(),
                 gm.ntris(), bm.ntris()))

    def bays(self, c):
        """U-shaped double bay lines along each row: a row is the aisle-side
        edge (two lon/lat points) and the side the bays go to. / 駐車枠。"""
        out = []
        for r in c.get("rows", []):
            a, b = np.array(self.P(r["lonlat"][0])), np.array(self.P(r["lonlat"][1]))
            Lr = float(np.linalg.norm(b - a))
            t = (b - a) / Lr
            n = np.array([-t[1], t[0]]) * (1.0 if r.get("side", "left") == "left" else -1.0)
            w, dpt = float(r.get("width", 2.5)), float(r.get("depth", 5.0))
            for k in range(int(Lr // w) + 1):
                p = a + t * k * w
                out.append(u_divider(tuple(p + n * dpt), tuple(p + n * 0.5)))
            self.nbays = getattr(self, "nbays", 0) + int(Lr // w)
        g = unary_union(out) if out else Polygon()
        return g.intersection(c["poly"].buffer(-0.15))

    def pitch_things(self, m, p):
        """Goals, the tall ball-stop net and the chain-link round the platform.
        ゴール・防球ネット・金網。"""
        plat = p.get("plat")
        ln = p.get("lines")
        if plat is not None and ln:
            c = self.P(ln["centre"])
            axis = math.radians(90.0 - float(ln.get("axis_deg", 0.0)))
            ca, sa = math.cos(axis), math.sin(axis)
            for s in (-1, 1):
                gx, gy = c[0] + ca * s * float(ln["length"]) / 2, c[1] + sa * s * float(ln["length"]) / 2
                if not plat.poly.buffer(-0.5).contains(Point(gx, gy)):
                    continue
                r = Mesh()
                r.zoff = plat.z
                for e in (-1, 1):
                    r.cylinder("goal_white", gx - sa * e * 3.66, gy + ca * e * 3.66, 0.0, 2.44, 0.06, seg=6)
                    # the back frame / 後ろの枠
                    bx, by = gx + ca * s * 1.5 - sa * e * 3.66, gy + sa * s * 1.5 + ca * e * 3.66
                    r.cylinder("goal_white", bx, by, 0.0, 1.2, 0.04, seg=6)
                r.box("goal_white", gx, gy, 2.38, 2.5, 0.1, 7.44, axis)
                r.box("goal_white", gx + ca * s * 1.5, gy + sa * s * 1.5, 0.0, 0.06, 0.06, 7.32, axis)
                m.merge(r)
        # along the platform edge, 0.3 m in (on top of its retaining wall): the
        # OSM fence way lies in the footway band on the south side
        # 造成面の縁から 0.3 m 内側（OSM の柵は南側で歩道帯の中）
        if plat is None:
            return 0.0
        edge = max(polys(plat.poly.buffer(-0.3, join_style="mitre")), key=lambda q: q.area, default=None)
        if edge is None:
            return 0.0
        f = LineString(list(orient(edge, 1.0).exterior.coords))
        stop = unary_union([self.no_go.buffer(0.4), self.footprint.buffer(0.3)])
        g = f.difference(stop)
        cen = p["poly"].centroid
        sides = set(p.get("net_sides", ["w", "s"]))
        hn = float(p.get("net_h", 12.0))
        high, low = [], []
        for q in getattr(g, "geoms", [g]):
            if q.geom_type != "LineString" or q.length < 0.8:
                continue
            cs = list(q.coords)
            cur, kind = [cs[0]], None
            for a, b in zip(cs, cs[1:]):
                mx, my = (a[0] + b[0]) / 2 - cen.x, (a[1] + b[1]) / 2 - cen.y
                dx, dy = b[0] - a[0], b[1] - a[1]
                vert = abs(dy) > abs(dx)
                side = ("w" if mx < 0 else "e") if vert else ("s" if my < 0 else "n")
                k = side in sides
                if kind is not None and k != kind:
                    (high if kind else low).append(LineString(cur))
                    cur = [a]
                cur.append(b)
                kind = k
            if len(cur) > 1:
                (high if kind else low).append(LineString(cur))
        tot = 0.0
        for q in high:
            if q.length > 0.8:
                self.net(m, q, hn, post=9.0, wire=0.8, rails=(0.05, 2.0, hn / 2, hn - 0.05), pr=0.11)
                tot += q.length
        for q in low:
            if q.length > 0.8:
                self.net(m, q, 2.0, post=2.5, wire=0.3, pr=0.035)
        return tot

    def net(self, m, line, h, post=2.5, wire=0.3, rails=None, pr=0.035):
        """A see-through net / chain-link: round steel posts, rails, thin
        strands and wires (two-sided strips 8 mm apart, no alpha). / 透ける網。"""
        rails = rails if rails is not None else (0.05, h / 2, h - 0.05)
        Lg = line.length
        nseg = max(1, int(math.ceil(Lg / post)))
        for k in range(nseg + 1):
            q = line.interpolate(Lg * k / nseg)
            r = self.rigid(q.x, q.y, 0.1)
            r.cylinder("akeno_steel", q.x, q.y, -0.3, h + 0.05, pr, seg=8 if pr > 0.06 else 6)
            m.merge(r)
        fm = self.follow()
        c = list(line.coords)
        for a, b in zip(c, c[1:]):
            seg = math.dist(a, b)
            if seg < 1e-3:
                continue
            for zz in rails:
                fm.wall("akeno_steel", a, b, zz - 0.025, zz + 0.025, 0.0, 1.0)
                fm.wall("akeno_steel", *back(a, b, 0.03), zz - 0.025, zz + 0.025, 0.0, 1.0)
            for zz in np.arange(0.4, h - 0.3, 0.6 if h < 6 else 1.0):
                fm.wall("akeno_net", a, b, zz - 0.007, zz + 0.007, 0.0, 1.0)
                fm.wall("akeno_net", *back(a, b, 0.008), zz - 0.007, zz + 0.007, 0.0, 1.0)
            t = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
            dense, fm.dense = fm.dense, None
            for k in range(int(seg // wire) + 1):
                u = min(seg, k * wire)
                p0 = (a[0] + t[0] * (u - 0.008), a[1] + t[1] * (u - 0.008))
                p1 = (a[0] + t[0] * (u + 0.008), a[1] + t[1] * (u + 0.008))
                zb = float(self.S.surface_z([0.5 * (p0[0] + p1[0])], [0.5 * (p0[1] + p1[1])])[0])
                fm.zfun = None
                fm.wall("akeno_net", p0, p1, zb, zb + h, 0.0, 1.0)
                fm.wall("akeno_net", *back(p0, p1, 0.008), zb, zb + h, 0.0, 1.0)
                fm.zfun = lambda x, y: self.S.surface_z(x, y)
            fm.dense = dense
        m.merge(fm)


def write_textures(tex_dir):
    """Drawn textures (procedural; colours from the z20 tiles and the user's
    photo, no imagery used): sand, greenhouse film (roof and wall, steel bars
    every 1 m / 1.5 m drawn in), the concrete kick strip, steel, net.
    描画テクスチャ（砂・ハウスのフィルムと骨組・腰コンクリート・鋼・網）。"""
    import textures as X
    from PIL import Image
    out = {}

    def save(name, im, rough=0.85, metal=0.0, relief=0.0):
        im = im.convert("RGB")
        im.save(os.path.join(tex_dir, name + ".png"))
        if relief > 0:
            h = X.height_of(im, 0.3)
            X.normal_map(h / max(np.abs(h).max(), 1e-6), relief).save(os.path.join(tex_dir, name + "_n.png"))
        r = np.clip(rough + 0.05 * X.height_of(im, 0.5), 0, 1)
        Image.fromarray((X.srgb_encode(np.asarray(r, np.float64)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_r.png"))
        Image.fromarray((X.srgb_encode(np.full((8, 8), metal)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_m.png"))
        out[name] = (name, None, rough, metal, False)

    def tinted(c0, c1, n=512, scales=(4, 16, 64)):
        return np.asarray(Image.fromarray(X.tint(X.noise(n, scales), c0, c1)), np.float64)

    # sand pitch (beige, raked streaks) / 砂のグラウンド
    a = tinted((150, 124, 92), (178, 152, 116), 512, (16, 64, 256))
    save("akeno_sand", Image.fromarray(a.clip(0, 255).astype(np.uint8)), 0.95, 0.0, 1.0)
    # greenhouse film: one texture tile = 3 x 3 m (uv 3.0), bars every 1 m across
    # and 1.5 m along, a darker purlin band; the film greyish white, faintly
    # see-through-looking (darker plants inside, mottled) / ハウスのフィルム
    n = 384
    film = tinted((138, 150, 152), (176, 186, 186), n, (8, 32, 96))
    plants = tinted((90, 112, 96), (150, 168, 150), n, (4, 16))
    film = film * 0.72 + plants * 0.28
    bar = np.array([204, 206, 204], np.float64)
    for k in range(3):
        x = int(k * n / 3)
        film[:, max(0, x - 2):x + 2] = bar
    for k in range(2):
        y = int(k * n / 2)
        film[max(0, y - 2):y + 2, :] = bar
    save("akeno_gh_roof", Image.fromarray(film.clip(0, 255).astype(np.uint8)), 0.3, 0.0, 0.3)
    wall = tinted((132, 144, 144), (170, 180, 178), n, (8, 32, 96)) * 0.7 + plants * 0.3
    for k in range(3):
        x = int(k * n / 3)
        wall[:, max(0, x - 3):x + 3] = bar
    for y in (int(n * 0.05), int(n * 0.55)):
        wall[max(0, y - 2):y + 2, :] = bar * 0.92
    save("akeno_gh_wall", Image.fromarray(wall.clip(0, 255).astype(np.uint8)), 0.35, 0.0, 0.3)
    kick = tinted((150, 150, 144), (178, 176, 168), 256, (8, 32))
    save("akeno_gh_kick", Image.fromarray(kick.clip(0, 255).astype(np.uint8)), 0.9, 0.0, 0.6)
    save("akeno_steel", Image.fromarray(tinted((150, 154, 156), (178, 182, 184), 128, (4, 16)).clip(0, 255)
                                        .astype(np.uint8)), 0.45, 0.7)
    save("akeno_net", Image.fromarray(tinted((52, 70, 58), (78, 96, 82), 64, (2, 8)).clip(0, 255)
                                      .astype(np.uint8)), 0.7, 0.0)
    return out
