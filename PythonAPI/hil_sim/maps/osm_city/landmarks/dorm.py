"""Oita Kosen dormitory (大分高専 明野寮), Akeno, Oita: modelled in detail
(maps/.venv). Across the main road from the college gate.

Sources (2026): the z20 satellite tiles, Google Earth 3D orbits and the
written description, Street View at the gate (2026-02) and from the south and
east (2024-07), the dorm's own web page photos and the College Bulletin 2026
building table (Photo Getter, build/OitaKosen/dorm_photos). Outlines traced on
a map-frame mosaic of the z20 tiles (build/OitaKosen/dorm_trace.json); storeys
from Street View / Google Earth; colours from the description. Nothing is
copied as texture: every texture is drawn here.

  A, B, C, D   3-storey RC dorm blocks: off-white render, beige spandrel
               panels between projecting columns, aluminium sashes; balconies
               with solid parapets, partitions, AC units and laundry poles on
               the south / south-east faces of A, B and D; C's open corridor on
               the north; flat entrance canopies with glass doors (the 第一寮
               photo); stair windows on the end walls; flat roofs (grey-green,
               teal, green) with parapets, copings and stair penthouses; B's
               rooftop stair hall and west escape stair; D in two halves offset
               6.5 m with the stair hall at the joint
  E            dorm 5: grey-beige tile, 4 storeys of punched windows, a lower
               level exposed on the east slope, the stacked escape stair at the
               east end, the small gabled porch and AC units along the south
  office 23    single storey L, white render, thin projecting roof slab, mint
               roof, glass entrance facing the forecourt, red post box
  dining/bath  four single-storey parts at two heights: the bath wing with two
               solar arrays, the high central hall, the kitchen's skylight
               roof, the dining hall; the white boiler chimney; the covered
               walkway (brown steel frame, translucent roof)
  small        electrical room (RC1), laundry house, store, soccer clubroom,
               bicycle shelters, the store north-west of D
  site         the gate: the white tiled planter wall with the name plate
               between the two entries, white tiled walls with clipped shrubs,
               the brown louvre fence and the grey precast panel wall along the
               main road; grey panel wall and white mesh fence on the south and
               east; asphalt forecourt and lanes, paving round the blocks,
               kitchen gardens with white pipe frames, trees from the imagery
  terrain      each block (complex) on its own level platform (landmarks/
               onterrain.py), A at its median ground (the site falls 3 m along
               it), E's east end a storey lower; retaining walls and banks

Roads win: every footprint is cut 0.4 m clear of the road network, its kerbs,
the mapped footways, drives and car parks; facade projections shrink per edge
to stay 0.2 m clear; fences, walls, posts and trees are kept off them too.
Material names never contain "sign" or "light" (CARLA's importer drops them).

Coordinates: the map frame (x east, y north, metres).

大分高専 明野寮の詳細モデル。
"""

import json
import math
import os

import numpy as np
import shapely
from shapely.affinity import translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as B
import geo
from landmarks import onterrain
from landmarks.kosen import _solid, fit_rect, split_tiles
from landmarks.lvss import oquad, text_poly
from mesh import Mesh, polys
import jtrees

NAME = "大分高専 明野寮"
TAG = "DORM"
TRACE = "dorm_trace.json"
SITE_WAY = "184178243"
CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
CLEAR_FOOT = 0.4         # footprint to roads, kerbs, paths / 建物と道路・歩道の間隔
ROOM = 0.2               # facade projections stay this far off them / 張り出しの余裕
TREE_CLEAR = 0.3         # tree crowns to roads and paths / 樹冠と道路の間隔
APRON = 1.2              # paved platform margin round a building / 建物周りの舗装
GROUP_DROP = 2.0         # joined blocks share a platform unless their ground differs more / 一体の造成の上限
BAY = 3.6                # dorm room width / 寮室の幅
BALC = 1.2               # balcony / corridor depth (inside the footprint) / バルコニーの奥行き

ROOFS = {"greygreen": "dorm_roof_greygreen", "teal": "dorm_roof_teal", "green": "dorm_roof_green",
         "mint": "dorm_roof_mint", "lightgrey": "dorm_roof_palegrey", "skylight": "dorm_roof_toplit",
         "grey": "dorm_roof_grey", "metal": "dorm_roof_metal", "white": "dorm_roof_white",
         "greymetal": "dorm_roof_greymetal", "brown": "dorm_roof_brown", None: "dorm_roof_grey"}
RECT_STYLES = ("bikeshed", "shed", "shed_corr", "clubroom", "small_rc", "ext_stair_box", "roof_room")
SIDES = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0), "NE": (0.7071, 0.7071), "NW": (-0.7071, 0.7071),
         "SE": (0.7071, -0.7071), "SW": (-0.7071, -0.7071)}


def edges(poly):
    return B.edges_of(orient(poly, 1.0))


def drop_narrow(g, w=0.1):
    """g without its parts under w wide (a morphological opening by w/2, mitred so
    corners stay); pieces vanish whole if nowhere w wide. / 幅 w 未満の部分を除く。"""
    if g.is_empty:
        return g
    o = g.buffer(-w / 2, join_style="mitre").buffer(w / 2, join_style="mitre")
    return onterrain._clean(o.intersection(g), 0.01)


def axes(poly):
    """The oriented rectangle of a polygon: corner a, unit long axis, unit short
    axis (to the left), long and short lengths. / 外接矩形の軸。"""
    r = orient(poly.minimum_rotated_rectangle, 1.0)
    c = [np.array(q) for q in list(r.exterior.coords)[:4]]
    e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
    k = 0 if e[0] >= e[1] else 1
    a, b, d = c[k], c[(k + 1) % 4], c[(k + 3) % 4]
    return a, (b - a) / e[k], (d - a) / e[(k + 1) % 4], e[k], e[(k + 1) % 4]


class DORM(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        self.out_dir = out_dir
        self.fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        self.drape = None                         # set by build.py (terrain.Drape) / build.py が設定
        G = T["grounds"]
        gp = lambda d: Polygon([self.P(d["tile"], *q) for q in d["pts"]]).buffer(0)
        self.site_osm = next(p for p, t, w in L.osm.areas_id(lambda t: t.get("landuse") == "residential")
                             if w == SITE_WAY)
        # what the dorm owns: the OSM site, the strip to the main road's footway,
        # B and C across the site's south-east edge, the clubroom's corner
        # 寮の範囲: OSM 敷地 + 幹線歩道までの帯 + B・C の南側 + 部室
        self.site = onterrain._clean(unary_union([self.site_osm] + [gp(G[k]) for k in
                                                                    ("ext_west", "ext_south", "ext_c", "ext_club")]))
        near = self.site.buffer(40.0)
        self.road = L.road_all.intersection(near)
        self.carriageway = L.carriageway.intersection(near)
        # paths without the parts under 0.1 m wide (tongues a footpath cut leaves, e.g.
        # 0.01 m by B's east end): their clearance dug slots into the platforms
        # 幅 0.1 m 未満の歩道の舌片を除く（造成面に溝を掘った）
        self.paths = drop_narrow(unary_union([L.footpaths, L.cycleways, L.driveways, L.parking]).intersection(near))
        self.islands = L.islands.intersection(self.site.buffer(5.0))
        self.no_go = unary_union([self.road, self.paths])
        shapely.prepare(self.no_go)
        # generic buildings near the site (their lots are added in region()) / 周辺の汎用建物
        self.generic = unary_union([p for p, t in L.buildings if p.intersects(near)]) if L.buildings else Polygon()
        lean = T["lean"]
        self.lean = (lean["dx_per_m"], lean["dy_per_m"])
        keep = unary_union([self.no_go.buffer(CLEAR_FOOT), self.generic.buffer(0.5)])
        self.blocks, self.cut = {}, {}
        for k, b in T["blocks"].items():
            roof = orient(Polygon([self.P(b["tile"], *q) for q in b["pts"]]).buffer(0), 1.0)
            # the roof outline as seen, moved back by the lean of its top / 屋根の輪郭を傾き分戻す
            hl = b.get("lean_h", b.get("h", 2.6) + (T["blocks"][b["on"]]["h"] if b.get("style") == "roof_room" else 0.0))
            foot = translate(roof, -self.lean[0] * hl, -self.lean[1] * hl)
            # and inside the dorm's own ground (its platform is clipped to it) / 寮の範囲内に
            q = foot.difference(keep).intersection(self.site.buffer(-0.7, join_style="mitre"))
            parts = [p for p in polys(q) if p.area > 1.0]
            if not parts or max(p.area for p in parts) < 0.35 * foot.area:
                print("landmark %s: %s dropped (on the road network or a path)" % (TAG, k))
                self.cut[k] = foot.area
                continue
            q = max(parts, key=lambda g: g.area)
            if q.area < foot.area - 0.5:
                self.cut[k] = foot.area - q.area
                print("landmark %s: %s cut back %.1f m2 of %.0f (roads / paths win)" % (TAG, k, foot.area - q.area,
                                                                                     foot.area))
            q = orient(q.simplify(0.05), 1.0)
            if b["style"] in RECT_STYLES:
                q = fit_rect(q)
                rr = q.minimum_rotated_rectangle.exterior.coords
                if min(math.dist(rr[0], rr[1]), math.dist(rr[1], rr[2])) < 1.8:
                    print("landmark %s: %s dropped (what the roads leave is under 1.8 m wide)" % (TAG, k))
                    self.cut[k] = foot.area
                    continue
            d = dict(b)
            d.update(poly=q, roof_seen=roof, name=k)
            self.blocks[k] = d
        # roof rooms stay on their block's roof / 屋上の塔屋は親の屋根の上に
        for k, b in list(self.blocks.items()):
            if b.get("style") == "roof_room":
                par = self.blocks.get(b["on"])
                if par is None:
                    del self.blocks[k]
                    continue
                b["poly"] = orient(onterrain._clean(b["poly"].intersection(par["poly"].buffer(-1.0, join_style="mitre"))),
                                   1.0)
                if b["poly"].is_empty or b["poly"].geom_type != "Polygon":
                    del self.blocks[k]
        # touching blocks must not overlap (their copings doubled) / 接する棟は重ねない
        taken = Polygon()
        for k in sorted(self.blocks, key=lambda k: -self.blocks[k]["poly"].area):
            b = self.blocks[k]
            if b.get("style") == "roof_room":
                continue
            if b["poly"].distance(taken) < 0.002:
                # 2 mm clear of the earlier blocks, then near-collinear vertices removed
                # (the cut left 1 cm edge pieces whose parapets and copings doubled)
                # 先の棟から 2 mm 離し、ほぼ一直線の頂点を除く（1 cm の辺で笠木が二重に）
                q = [g for g in polys(b["poly"].difference(taken.buffer(0.002, join_style="mitre"))) if g.area > 1.0]
                if q:
                    b["poly"] = orient(max(q, key=lambda g: g.area).simplify(0.002), 1.0)
            taken = unary_union([taken, b["poly"]])
        ground_blocks = [b["poly"] for b in self.blocks.values() if b.get("style") != "roof_room"]
        self.footprint = unary_union(ground_blocks)
        shapely.prepare(self.footprint)
        self.ground = {k: gp(v) for k, v in G.items() if "pts" in v}
        self.gate = G["gate"]
        self.trees_seen = [(self.P(t, x, y), r) for t, x, y, r in T["trees"]]
        self._built = None
        self._region = None

    def P(self, tile, px, py):
        la, lo = self.T["tiles"][tile]
        cx, cy = self.fr.xy(lo, la)
        return (cx + (px - self.T["centre"][0]) * self.T["mpp_x"], cy - (py - self.T["centre"][1]) * self.T["mpp_y"])

    # -- what build.py needs / build.py 用
    def inner(self):
        """Where the generic ground layers are cleared: the dorm's ground inside its
        0.6 m edge, except paths outside the OSM site (the main road's footway
        stays generic). / 汎用の地面を消す範囲（敷地外の歩道は残す）。"""
        out = self.paths.difference(self.site_osm.buffer(-0.6, join_style="mitre"))
        return onterrain._clean(self.site.buffer(-0.6, join_style="mitre").difference(out.buffer(0.05)))

    def region(self):
        """The ground this module draws: inner() without the road network, its
        islands, and the generic buildings' lots. / 本モジュールが描く地面。"""
        if self._region is None:
            r = self.inner().difference(self.L.road_all.buffer(0.01)).difference(self.L.islands.buffer(0.01))
            lots = getattr(self.L, "terraces", None)
            obst = [self.generic.buffer(0.5)]
            if lots is not None and not lots.is_empty:
                obst.append(lots.intersection(self.site.buffer(5.0)).buffer(0.3))
            r = r.difference(unary_union(obst))
            self._region = onterrain._clean(r, 0.5)
        return self._region

    def ground_taken(self):
        """Left out of the map's draped grass base (build.py adds it to L.terraces). / 汎用の芝を張らない範囲。"""
        return self.region()

    def keepout(self):
        """No generic trees or furniture on the dorm's ground: it plants its own. / 汎用の木は置かない。"""
        return self.region().buffer(0.5)

    def ground_mesh(self, tile, z):
        self.build_all()
        return self._built["ground"].get(tile.bounds, Mesh())

    def buildings_mesh(self, tile):
        self.build_all()
        return self._built["bld"].get(tile.bounds, Mesh())

    def extra_meshes(self, tile):
        """Each tree species' leaves as objects of their own (seasons, foliage
        material). / 樹種ごとの葉（季節・植生マテリアル用）。"""
        self.build_all()
        return {jtrees.KIND[sp]: d.get(tile.bounds, Mesh()) for sp, d in self._built["leaves"].items()}

    def note(self):
        return "%d blocks" % len(self.blocks)

    # -- terrain / 地形
    def plan(self):
        """Platforms: each complex of touching blocks on one level (its highest
        ground, A at its median); E's east end (the slope) a storey lower. / 造成計画。"""
        if getattr(self, "S", None) is not None:
            return self.S
        S = onterrain.Site(self.drape, self.region(), no_go=unary_union([self.no_go, self.L.islands]), gap=0.3)
        self.S = S
        self.zb = {}
        self.low = {}
        rest = [k for k, b in self.blocks.items() if b.get("style") != "roof_room"]
        groups, seen = [], set()
        for k in sorted(rest, key=lambda k: -self.blocks[k]["poly"].area):
            if k in seen:
                continue
            grp, todo = [], [k]
            seen.add(k)
            while todo:
                a = todo.pop()
                grp.append(a)
                for o in rest:
                    if o not in seen and self.blocks[a]["poly"].distance(self.blocks[o]["poly"]) < 0.6:
                        seen.add(o)
                        todo.append(o)
            groups.append(grp)
        self.groups = []
        for grp in groups:
            u = unary_union([self.blocks[k]["poly"] for k in grp])
            if any(self.blocks[k].get("lower") for k in grp):
                self.plan_lower(S, grp, u)
                continue
            one = any(self.blocks[k].get("one_platform") for k in grp)
            # the dining/bath parts share one floor: steps between them left slots where
            # their aprons met / 食堂・浴場は一つの床（段差の継ぎ目に穴が出た）
            subs = [[k] for k in grp] if len(grp) > 1 and S.drop(u) > GROUP_DROP and not one else [grp]
            for sg in subs:
                u = unary_union([self.blocks[k]["poly"] for k in sg])
                how = next((self.blocks[k]["level"] for k in sg if self.blocks[k].get("level")), "max")
                p = S.platform("+".join(sg), u, how=how, apron=APRON)
                z = p.z if p is not None else S.level(u, how)
                for k in sg:
                    self.zb[k] = z
                self.groups.append((sg, z, S.drop(u)))
        for k, b in self.blocks.items():
            if b.get("style") == "roof_room":
                par = self.blocks[b["on"]]
                self.zb[k] = self.zb[b["on"]] + self.top(par)
        return S

    def plan_lower(self, S, grp, u):
        """A block with a lower level on the slope (E): the platform at its highest
        ground, and where the ground falls more than 2 m a second platform one
        storey lower with that storey exposed. / 斜面側に下階が出る棟。"""
        z_up = S.level(u, "max")
        fl = self.T["floor_m"]
        a, ax, sd, Ln, W = axes(u)
        # ground along the long axis / 長手方向の地盤
        us = np.linspace(0.0, Ln, int(Ln // 1.0) + 2)
        mid = [a + ax * t + sd * (W / 2) for t in us]
        g = S.ground_z([q[0] for q in mid], [q[1] for q in mid])
        low_end = 1 if g[-1] < g[0] else 0
        cut_u = None
        order = range(len(us)) if low_end == 1 else range(len(us) - 1, -1, -1)
        for i in order:
            if g[i] < z_up - 2.0:
                cut_u = us[i] - (1.0 if low_end == 1 else -1.0)
                break
        if cut_u is None or cut_u <= 2.0 or cut_u >= Ln - 2.0:
            p = S.platform("+".join(grp), u, how="max", apron=APRON)
            for k in grp:
                self.zb[k] = p.z if p is not None else z_up
            self.groups.append((grp, z_up, S.drop(u)))
            return
        big = 1e3
        if low_end == 1:
            half = Polygon([tuple(a + ax * cut_u - sd * big), tuple(a + ax * big - sd * big),
                            tuple(a + ax * big + sd * big), tuple(a + ax * cut_u + sd * big)])
        else:
            half = Polygon([tuple(a - ax * big - sd * big), tuple(a + ax * cut_u - sd * big),
                            tuple(a + ax * cut_u + sd * big), tuple(a - ax * big + sd * big)])
        lowp = onterrain._clean(u.intersection(half))
        upp = onterrain._clean(u.difference(half))
        # the two platforms meet on the split line, aprons included / 分割線で接する
        p2 = S.platform("+".join(grp) + "_low", lowp.buffer(APRON, join_style="mitre").intersection(half), z=z_up - fl,
                        keep=lowp)
        p1 = S.platform("+".join(grp), upp.buffer(APRON, join_style="mitre").difference(half), z=z_up, keep=upp)
        for k in grp:
            self.zb[k] = z_up
            self.low[k] = lowp
        self.groups.append((grp, z_up, S.drop(u)))
        self.groups.append(([k + "_low" for k in grp], z_up - fl, S.drop(lowp)))
        _ = p1, p2

    # -- assembly / 組み立て
    def build_all(self):
        if self._built is not None:
            return
        import ground as G
        S = self.plan()
        gm, bm = Mesh(), Mesh()
        self.ground_surfaces(gm)
        S.walls(gm, mat="jp_retaining", bank_mat="grass_park")
        for k, b in self.blocks.items():
            m = Mesh()
            getattr(self, "b_" + b["style"])(m, b)
            m.shift_z(self.zb[k])
            bm.merge(m)
        self.leaves = jtrees.leafsets()
        self.site_things(bm)
        tiles = [tb for tn, tb in G.tiles(self.meta)]
        self._built = {"ground": split_tiles(gm, tiles), "bld": split_tiles(bm, tiles),
                       "leaves": {sp: split_tiles(lm, tiles) for sp, lm in self.leaves.items()}}
        n = gm.ntris(), bm.ntris()
        print("landmark %s: terrain %s; ground %d, buildings %d triangles" % (TAG, S.report(), n[0], n[1]))
        for sg, z, dr in self.groups:
            print("  platform %s: z %.2f, ground under it varies %.1f m" % ("+".join(sg)[:60], z, dr))
        for k, b in self.blocks.items():
            if b.get("storeys"):
                a, ax, sd, Ln, W = axes(b["poly"])
                print("  block %-8s %5.1f x %4.1f m, %d storeys%s, %.0f m2" % (
                    k, Ln, W, b["storeys"], " + lower level" if k in self.low else "", b["poly"].area))

    # -- ground / 地面
    def ground_surfaces(self, m):
        S = self.S
        reg = self.region()
        bplat = unary_union([p.poly for p in S.items if p.kind == "building"])
        foot = self.footprint
        gr = self.ground
        gardens = unary_union([gr["garden_c"], gr["garden_d"]]).intersection(reg)
        paved = unary_union([gr[k] for k in ("forecourt", "lane_n", "lane_b", "yard_e", "path_e")])
        paths = self.paths.intersection(reg)                     # mapped footways inside / 敷地内の歩道
        paved = onterrain._clean(paved.intersection(reg).difference(gardens).difference(paths))
        self.paved = paved
        apron = onterrain._clean(bplat.difference(foot))
        S.ground(m, "dorm_paving", apron, uv_scale=2.0)
        S.ground(m, "dorm_asphalt", paved.difference(bplat), uv_scale=5.0)
        S.ground(m, "concrete_path", paths.difference(bplat), uv_scale=2.0)
        S.ground(m, "soil", gardens.difference(bplat), uv_scale=3.0)
        S.ground(m, "grass", reg.difference(paved).difference(paths).difference(bplat).difference(gardens),
                 uv_scale=4.0)

    # ================= blocks / 棟 =================
    def room(self, a, b, n):
        """How far a facade on edge a-b may project before it nears a road or path. / 張り出し可能量。"""
        seg = LineString([tuple(a), tuple(b)])
        d = self.no_go.distance(seg)
        _ = n
        return max(0.0, d - ROOM)

    def top(self, b):
        st = b.get("style")
        if st in ("dorm", "dorm_hall", "dorm_tile", "ext_stair_box"):
            return b["storeys"] * self.T["floor_m"]
        return b["h"] - 0.6

    def covered(self, b, a, bb, n):
        """Parts of an edge hidden by a neighbouring block: [(u0, u1, height of
        the neighbour)]. / 隣の棟に隠れる区間。"""
        L = float(np.linalg.norm(bb - a))
        t = (bb - a) / L
        out = []
        for k, o in self.blocks.items():
            if o is b or o.get("style") in ("roof_room", "ext_stair_box"):
                continue
            band = o["poly"].buffer(0.12)
            seg = LineString([tuple(a + n * 0.06), tuple(bb + n * 0.06)])
            if not band.intersects(seg):
                continue
            ins = seg.intersection(band)
            for g in getattr(ins, "geoms", [ins]):
                if g.geom_type != "LineString" or g.length < 0.3:
                    continue
                us = [float(np.dot(np.array(q) - (a + n * 0.06), t)) for q in g.coords]
                hz = self.top(o) + 0.6 * (o.get("style") not in ("dorm", "dorm_hall", "dorm_tile")) \
                    + self.zb.get(k, 0.0) - self.zb.get(b["name"], 0.0)
                out.append((max(0.0, min(us)), min(L, max(us)), hz))
        return out

    def facade_pieces(self, b):
        """Every edge of a block, split where neighbours cover it: (a, b, L, n,
        z_from, (u_off, L_full)) -- draw the facade above z_from only. / 外壁の区間。"""
        out = []
        for a, bb, L, n in edges(b["poly"]):
            cov = sorted(self.covered(b, a, bb, n))
            cuts = [(0.0, L, -1e3)]
            for u0, u1, hz in cov:
                new = []
                for c0, c1, zf in cuts:
                    if u1 <= c0 or u0 >= c1:
                        new.append((c0, c1, zf))
                        continue
                    if u0 > c0:
                        new.append((c0, u0, zf))
                    new.append((max(c0, u0), min(c1, u1), max(zf, hz)))
                    if u1 < c1:
                        new.append((u1, c1, zf))
                cuts = new
            t = (bb - a) / L
            for c0, c1, zf in cuts:
                if c1 - c0 < 0.2:
                    continue
                out.append((a + t * c0, a + t * c1, c1 - c0, n, zf, (c0, L)))
        return out

    def side_of(self, b, key, n):
        s = b.get(key)
        if not s:
            return False
        v = np.array(SIDES[s], np.float64)
        return float(n @ v) > 0.6

    # ---- RC dorm blocks A-D / RC の寮棟
    def b_dorm(self, m, b):
        st, fl = b["storeys"], self.T["floor_m"]
        top = st * fl
        lowp = self.low.get(b["name"])
        for a, bb, L, n, zf, (u_off, Lfull) in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            po = self.room(a, bb, n)
            end = Lfull < 13.0
            balc = self.side_of(b, "balcony", n) and Lfull > 12 and L > 4.0
            corr = self.side_of(b, "corridor", n) and Lfull > 12 and L > 4.0
            entr = self.side_of(b, "entrance", n) and L > 4.0 and zf < 0.5
            ent_u = self.entrance_u(L, Lfull, u_off) if entr and not end and not balc and not corr else None
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                if end:
                    self.end_wall(f, L, z0, z1, k, stair=k < st, entr=entr and k == 0, po=po)
                elif balc:
                    self.balconies(f, L, z0, z1, k)
                elif corr:
                    self.corridor(f, L, z0, z1, k, po)
                else:
                    self.windows(f, L, z0, z1, k, po, south=n[1] < -0.3, ent_u=ent_u)
            if top > zf:
                f.face("dorm_render", 0.0, L, max(zf, top - 0.001), top, 0.0)
            if ent_u is not None:
                self.canopy(f, ent_u, po)
            if end and self.side_of(b, "ext_stair", n) and zf < 0.5:
                self.escape_stair(m, f, L, st, fl, po)
            if end and self.side_of(b, "end_balconies", n) and zf < 0.5:
                self.end_balconies(f, L, st, fl, po)
            if lowp is not None:
                self.lower_storey(f, a, bb, n, lowp, fl, "dorm_render")
            self.downpipes(f, L, top, po, end)
        self.flat_roof(m, b, top, parapet=0.7)

    def entrance_u(self, L, Lfull, u_off):
        """Where the entrance bay is on this piece (about a third along the full
        edge), or None if it falls outside. / 玄関の位置。"""
        u = Lfull * 0.33 - u_off
        bays = B.bays(L, BAY, 0.4)
        if not bays or u < 1.0 or u > L - 1.0:
            return None
        i = min(range(len(bays)), key=lambda j: abs((bays[j][0] + bays[j][1]) / 2 - u))
        return (bays[i][0] + bays[i][1]) / 2

    def windows(self, f, L, z0, z1, k, po, south=False, ent_u=None, wall="dorm_render"):
        """Room windows between projecting columns: a beige spandrel panel under
        each sash, set back; a sunshade slab over the ground floor on the south.
        柱の間の窓と腰壁パネル、南面 1 階の庇。"""
        bays = B.bays(L, BAY, 0.4)
        ops = []
        for u0, u1 in bays:
            mid = (u0 + u1) / 2
            if ent_u is not None and k == 0 and abs(mid - ent_u) < 0.1:
                ops.append((u0 + 0.3, u1 - 0.3, z0 + 0.02, z0 + 2.45, "door"))
            else:
                ops.append((u0 + 0.35, u1 - 0.35, z0 + 0.12, z0 + 2.4, "win"))
        f.wall_with_openings(wall, z0, z1, [o[:4] for o in ops], 3.0)
        for u0, u1, oz0, oz1, kind in ops:
            d = -0.16
            f.hplane(wall, u0, u1, d, 0.0, oz1, False)
            f.side(wall, u0, d, 0.0, oz0, oz1, 1)
            f.side(wall, u1, d, 0.0, oz0, oz1, -1)
            if kind == "door":
                f.hplane("dorm_paving", u0, u1, d, 0.0, oz0, True)
                f.m.quad("dorm_glassdoor", f.P(u0, d, oz0), f.P(u1, d, oz0), f.P(u1, d, oz1), f.P(u0, d, oz1))
                continue
            zs = z0 + 0.9
            f.m.quad("dorm_spandrel", f.P(u0, -0.06, oz0), f.P(u1, -0.06, oz0), f.P(u1, -0.06, zs), f.P(u0, -0.06, zs),
                     (0, 0), ((u1 - u0) / 1.5, 0), ((u1 - u0) / 1.5, 0.5), (0, 0.5))
            f.hplane(wall, u0, u1, d, 0.0, oz0, True)
            f.hplane("dorm_white", u0, u1, d, -0.06, zs, True)              # sill / 窓台
            f.m.quad("dorm_sash_2", f.P(u0, d, zs), f.P(u1, d, zs), f.P(u1, d, oz1), f.P(u0, d, oz1))
        cp = min(0.3, po)
        if cp > 0.05:
            for u in [0.0] + [u1 for u0, u1 in bays[:-1]] + [L]:
                u0, u1 = max(0.0, u - 0.22), min(L, u + 0.22)
                f.block(wall, u0, u1, 0.0, cp, z0, z1, top=False, bottom=False)
        if south and k == 0 and po > 0.4:
            f.block("dorm_white", 0.2, L - 0.2, 0.0, min(0.7, po), z0 + 2.62, z0 + 2.74)

    def balconies(self, f, L, z0, z1, k, wall="dorm_render"):
        """Balconies recessed into the footprint: solid parapet panels, partitions
        per room, slab soffits, sliding doors, an AC unit and a laundry pole each.
        バルコニー（腰壁・隔て板・掃き出し窓・室外機・物干し）。"""
        d, e = BALC, 0.25
        f.face(wall, 0.0, e, z0, z1, 0.0)
        f.face(wall, L - e, L, z0, z1, 0.0)
        f.side(wall, e, -d, 0.0, z0, z1, 1)
        f.side(wall, L - e, -d, 0.0, z0, z1, -1)
        f.face(wall, e, L - e, z1 - 0.25, z1, 0.0)                                  # slab edge / スラブ端
        f.hplane("dorm_soffit", e, L - e, -d, 0.0, z1 - 0.25, False)
        fz = z0 + (0.05 if k > 0 else 0.15)
        f.hplane("dorm_floor", e, L - e, -d, -0.14, fz, True)
        # parapet: front, top, back / 腰壁
        f.face("dorm_spandrel", e, L - e, z0, z0 + 1.1, 0.0, 1.5)
        f.hplane("dorm_white", e, L - e, -0.14, 0.0, z0 + 1.1, True)
        f.m.quad(wall, f.P(L - e, -0.14, fz), f.P(e, -0.14, fz), f.P(e, -0.14, z0 + 1.1), f.P(L - e, -0.14, z0 + 1.1))
        # the room wall behind: sliding doors per room / 奥の壁と掃き出し窓
        g = B.Facade(f.m, f.a + f.t * e - f.n * d, f.a + f.t * (L - e) - f.n * d, f.n)
        Lg = L - 2 * e
        bays = B.bays(Lg, BAY, 0.0)
        ops = [(u0 + 0.45, u1 - 0.45, fz, z0 + 2.2) for u0, u1 in bays]
        g.wall_with_openings(wall, fz, z1 - 0.25, ops, 3.0)
        for o in ops:
            g.hplane(wall, o[0], o[1], -0.12, 0.0, o[3], False)
            g.side(wall, o[0], -0.12, 0.0, o[2], o[3], 1)
            g.side(wall, o[1], -0.12, 0.0, o[2], o[3], -1)
            g.m.quad("dorm_balcony_door", g.P(o[0], -0.12, o[2]), g.P(o[1], -0.12, o[2]), g.P(o[1], -0.12, o[3]),
                     g.P(o[0], -0.12, o[3]))
        # partitions (fire screens) between rooms: thin boards / 隔て板
        for i, (u0, u1) in enumerate(bays):
            if i > 0:
                g.block("dorm_partition", u0 - 0.03, u0 + 0.03, 0.0, d - 0.16, fz, z1 - 0.25, top=False, bottom=False)
            # AC unit on the floor by the partition, a laundry pole under the soffit
            # 室外機と物干し竿
            ua = u0 + 0.55
            g.block("jp_ac_unit", ua, ua + 0.8, 0.05, 0.35, fz, fz + 0.62, top=True, bottom=False)
            g.block("dorm_steel", u0 + 0.3, u1 - 0.3, d - 0.62, d - 0.58, z0 + 2.15, z0 + 2.19, top=True, bottom=True,
                    ends=True)

    def corridor(self, f, L, z0, z1, k, po, wall="dorm_render"):
        """Open corridor (C, north): solid parapet, the room wall behind with a
        steel door and a frosted window per room, slab soffit, columns. / 開放廊下。"""
        d, e = BALC + 0.4, 0.25
        f.face(wall, 0.0, e, z0, z1, 0.0)
        f.face(wall, L - e, L, z0, z1, 0.0)
        f.side(wall, e, -d, 0.0, z0, z1, 1)
        f.side(wall, L - e, -d, 0.0, z0, z1, -1)
        f.face(wall, e, L - e, z1 - 0.3, z1, 0.0)
        f.hplane("dorm_soffit", e, L - e, -d, 0.0, z1 - 0.3, False)
        if k > 0:
            f.face("dorm_spandrel", e, L - e, z0, z0 + 1.1, 0.0, 1.5)
            f.hplane("dorm_white", e, L - e, -0.14, 0.0, z0 + 1.1, True)
            f.m.quad(wall, f.P(L - e, -0.14, z0), f.P(e, -0.14, z0), f.P(e, -0.14, z0 + 1.1), f.P(L - e, -0.14, z0 + 1.1))
            f.hplane("dorm_floor", e, L - e, -d, -0.14, z0 + 0.02, True)
        else:
            f.hplane("dorm_floor", e, L - e, -d, 0.0, z0 + 0.02, True)
        f.face(wall, e, L - e, z0, z1 - 0.3, -d)
        for u0, u1 in B.bays(L - 2 * e, BAY, 0.2):
            u0, u1 = u0 + e, u1 + e
            f.face("jp_door_steel", u0 + 0.3, u0 + 1.15, z0 + 0.02, z0 + 2.05, -d + 0.02)
            f.face("dorm_frosted", u0 + 1.6, u1 - 0.35, z0 + 1.05, z0 + 2.15, -d + 0.02)
        cp = min(0.25, po)
        for u in np.linspace(e + 0.3, L - e - 0.3, max(2, int(L // 7.2) + 1)):
            f.block(wall, u - 0.25, u + 0.25, -d + 0.4 if k > 0 else -d, cp, z0 + (1.1 if k > 0 else 0.0), z1 - 0.3,
                    top=False, bottom=False)

    def end_wall(self, f, L, z0, z1, k, stair=True, entr=False, po=0.0, wall="dorm_render"):
        """End wall: a tall stair window per floor; an entrance door on the ground
        floor when the entrance faces this way. / 妻壁: 階段の窓、玄関。"""
        mid = L / 2
        ops = []
        if L > 3.0 and stair:
            ops.append((mid - 0.75, mid + 0.75, z0 + 1.0, z0 + 2.7))
        if entr and L > 4.0:
            ops = [(mid - 1.0, mid + 1.0, z0 + 0.02, z0 + 2.4)]
        f.wall_with_openings(wall, z0, z1, ops, 3.0)
        for o in ops:
            if o[2] < z0 + 0.1:
                d = -0.16
                f.hplane(wall, o[0], o[1], d, 0.0, o[3], False)
                f.side(wall, o[0], d, 0.0, o[2], o[3], 1)
                f.side(wall, o[1], d, 0.0, o[2], o[3], -1)
                f.hplane("dorm_paving", o[0], o[1], d, 0.0, o[2], True)
                f.m.quad("dorm_glassdoor", f.P(o[0], d, o[2]), f.P(o[1], d, o[2]), f.P(o[1], d, o[3]), f.P(o[0], d, o[3]))
                if po > 1.2:
                    self.canopy(f, mid, po, width=3.2)
            else:
                f.window(*o, wall, glass="dorm_sash", mullions=1, depth=0.14)

    def canopy(self, f, u, po, width=3.8):
        """Flat entrance canopy: a cantilevered slab with a white fascia (the
        第一寮 photo), a step. / 玄関の片持ち庇。"""
        dep = min(2.2, po - 0.05)
        if dep < 0.6:
            return
        u0, u1 = max(0.0, u - width / 2), min(f.L, u + width / 2)
        f.block("dorm_white", u0, u1, 0.0, dep, 2.62, 2.92)
        f.face("dorm_canopy_edge", u0, u1, 2.62, 2.92, dep + 0.01)
        f.block("dorm_paving", u0 + 0.2, u1 - 0.2, 0.0, min(dep, 1.4), 0.0, 0.12, top=True, bottom=False)

    def escape_stair(self, m, f, L, st, fl, po):
        """B's west escape stair: RC landings at each floor and half-floor with
        solid parapets, flights between them, a column. / 西端の屋外避難階段。"""
        dep = min(1.9, po - 0.05)
        if dep < 1.0:
            return
        u0, u1 = L * 0.18, L * 0.82
        um = (u0 + u1) / 2
        for k in range(1, 2 * st):
            z = k * fl / 2
            full = k % 2 == 0
            a0, a1 = (u0, u1) if full else ((u0, um) if k % 4 == 1 else (um, u1))
            f.block("dorm_render", u0, u1, 0.0, dep, z - 0.2, z)                       # landing slab / 踊り場
            # parapets stand on the slab: no bottom faces (back to back with its top)
            # 手摺壁はスラブの上（下面なし）
            f.block("dorm_spandrel", u0, u1, dep - 0.12, dep, z, z + 1.1, bottom=False)     # front parapet / 手摺壁
            for uu in (u0, u1 - 0.12):
                f.block("dorm_spandrel", uu, uu + 0.12, 0.0, dep - 0.12, z, z + 1.1, bottom=False)
            # flight up to the next landing (an inclined slab) / 階段（傾斜スラブ）
            if k < 2 * st - 1:
                z2 = z + fl / 2
                fa, fb = (u0 + 0.15, um - 0.05) if k % 2 else (um + 0.05, u1 - 0.15)
                w0, w1 = 0.15, dep - 0.2
                if k % 2:
                    bot = [f.P(fb, w0, z), f.P(fb, w1, z), f.P(fa, w1, z2 - 0.2), f.P(fa, w0, z2 - 0.2)]
                else:
                    bot = [f.P(fa, w0, z), f.P(fa, w1, z), f.P(fb, w1, z2 - 0.2), f.P(fb, w0, z2 - 0.2)]
                topq = [(p[0], p[1], p[2] + 0.18) for p in bot]
                _solid(m, "dorm_floor", bot, topq)
            _ = a0, a1, full
        # corner columns 2 cm proud of the parapets and slabs on every face (never coplanar)
        # 隅柱は手摺壁・スラブより各面 2 cm 外へ
        zc = (2 * st - 1) * fl / 2 - 0.2
        f.block("dorm_render", u0 - 0.02, u0 + 0.35, dep - 0.35, dep + 0.02, 0.0, zc, top=False, bottom=False)
        f.block("dorm_render", u1 - 0.35, u1 + 0.02, dep - 0.35, dep + 0.02, 0.0, zc, top=False, bottom=False)
        f.hplane("dorm_render", u0 - 0.02, u0 + 0.35, dep - 0.35, dep + 0.02, zc + 0.02, True)
        f.hplane("dorm_render", u1 - 0.35, u1 + 0.02, dep - 0.35, dep + 0.02, zc + 0.02, True)

    def end_balconies(self, f, L, st, fl, po):
        """Stacked balconies on D's north-east end (GE): slabs with solid
        parapets on each upper floor. / 北東端の積み重なったバルコニー。"""
        dep = min(1.2, po - 0.05)
        if dep < 0.6:
            return
        u0, u1 = 0.6, L - 0.6
        for k in range(1, st):
            z = k * fl
            f.block("dorm_render", u0, u1, 0.0, dep, z - 0.18, z)
            f.block("dorm_spandrel", u0, u1, dep - 0.12, dep, z, z + 1.1, bottom=False)
            for uu in (u0, u1 - 0.12):
                f.block("dorm_spandrel", uu, uu + 0.12, 0.0, dep - 0.12, z, z + 1.1, bottom=False)

    def downpipes(self, f, L, top, po, end):
        if po < 0.15 or L < 4.0:
            return
        for u in ((0.45, L - 0.55) if not end else (0.45,)):
            f.block("jp_pipe", u, u + 0.1, 0.02, 0.12, 0.0, top + 0.5, top=False, bottom=False)

    def lower_storey(self, f, a, bb, n, lowp, fl, wall, punched=False):
        """The storey below the floor where the block stands over its lower
        platform (E's east end): windows on the exposed faces. / 斜面側の下階。"""
        seg = LineString([tuple(a), tuple(bb)])
        ins = seg.intersection(lowp.buffer(0.05))
        L = float(np.linalg.norm(bb - a))
        t = (bb - a) / L
        for g in getattr(ins, "geoms", [ins]):
            if g.geom_type != "LineString" or g.length < 0.8:
                continue
            us = sorted(float(np.dot(np.array(q) - a, t)) for q in g.coords)
            u0, u1 = max(0.0, us[0]), min(L, us[-1])
            sub = B.Facade(f.m, a + t * u0, a + t * u1, n)
            Ls = u1 - u0
            z0, z1 = -fl, 0.0
            ops = []
            if Ls > 2.0:
                w = 1.0 if punched else 1.6
                for c0, c1 in B.bays(Ls, 1.95 if punched else BAY, 0.4):
                    c = (c0 + c1) / 2
                    ops.append((c - w / 2, c + w / 2, z0 + 0.9, z0 + 2.3))
            sub.wall_with_openings(wall, z0, z1, ops, 2.0 if punched else 3.0)
            for o in ops:
                sub.window(*o, wall, glass="dorm_sash", mullions=1, depth=0.14)

    def flat_roof(self, m, b, z, parapet=0.7, mat=None, things=True):
        """Flat roof inside a parapet with a white coping; stair penthouses where
        traced, a TV mast, vent pipes. / 陸屋根・パラペット・塔屋・アンテナ。"""
        poly = b["poly"]
        mat = mat or ROOFS.get(b.get("roof"), "dorm_roof_grey")
        inner = poly.buffer(-0.2, join_style="mitre")
        m.flat(mat, inner, z + 0.02, 3.0)
        for p in polys(inner):
            m.ring_walls("dorm_render", p.exterior, z, z + parapet, outward=False, uv_scale=3.0)
        m.ring_walls("dorm_render", poly.exterior, z, z + parapet, outward=True, uv_scale=3.0)
        m.flat("dorm_white", poly.difference(inner), z + parapet, 1.0)
        if not things or inner.is_empty:
            return
        a, ax, sd, Ln, W = axes(inner)
        ang = math.atan2(ax[1], ax[0])
        dx, dy = -self.lean[0] * (b["h"] + 3.0), -self.lean[1] * (b["h"] + 3.0)
        for q in b.get("penthouse", []):
            c = np.array([q[0] + dx, q[1] + dy])
            hb = box(-1.6, -1.6, 1.6, 1.6)
            ok = inner.buffer(-0.4)
            if not ok.contains(Point(c)):
                # pull it inside along the short axis / 短辺方向に寄せる
                cc = np.array(ok.centroid.coords[0]) if not ok.is_empty else c
                c = c + sd * float(np.dot(cc - c, sd))
                if not ok.contains(Point(c)):
                    continue
            _ = hb
            m.box("dorm_render", c[0], c[1], z, z + 2.7, 3.2, 3.0, ang)
            m.box("dorm_white", c[0], c[1], z + 2.7, z + 2.85, 3.4, 3.2, ang)
            pf = c - sd * 1.51
            # on the -sd face (right of the long axis), facing out / -sd 側の面に外向き
            m.quad("jp_door_steel", (pf[0] - ax[0] * 0.45, pf[1] - ax[1] * 0.45, z + 0.02),
                   (pf[0] + ax[0] * 0.45, pf[1] + ax[1] * 0.45, z + 0.02),
                   (pf[0] + ax[0] * 0.45, pf[1] + ax[1] * 0.45, z + 2.0),
                   (pf[0] - ax[0] * 0.45, pf[1] - ax[1] * 0.45, z + 2.0))
        h = sum(ord(ch) for ch in b["name"])
        # vent pipes and a TV mast / 換気管とアンテナ
        for i in range(max(1, int(Ln // 12))):
            q = a + ax * (Ln * (i + 0.5) / max(1, int(Ln // 12))) + sd * (W * 0.7)
            if inner.buffer(-0.4).contains(Point(q)):
                m.cylinder("dorm_steel", q[0], q[1], z, z + 0.9, 0.08, seg=6)
        if h % 2 == 0 or b["name"] == "C":
            q = a + ax * (Ln * 0.62) + sd * (W * 0.5)
            if inner.buffer(-0.6).contains(Point(q)):
                m.box("dorm_steel", q[0], q[1], z, z + 4.5, 0.06, 0.06)
                for zz, w in ((z + 3.6, 1.6), (z + 4.2, 1.1)):
                    m.box("dorm_steel", q[0], q[1], zz, zz + 0.04, w, 0.04, ang + 0.5)

    def b_dorm_hall(self, m, b):
        """D's stair hall at the joint: tall stair windows, the penthouse. / D の階段ホール。"""
        st, fl = b["storeys"], self.T["floor_m"]
        top = st * fl
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            po = self.room(a, bb, n)
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                if L > 5.0:
                    ops = [(L / 2 - 1.6, L / 2 + 1.6, z0 + 0.9, z0 + 2.6)]
                    f.wall_with_openings("dorm_render", z0, z1, ops, 3.0)
                    for o in ops:
                        f.window(*o, "dorm_render", glass="dorm_sash", mullions=2, depth=0.14)
                else:
                    self.end_wall(f, L, z0, z1, k)
            if top > zf:
                f.face("dorm_render", 0.0, L, max(zf, top - 0.001), top, 0.0)
            if zf < 0.5 and n[0] < -0.3 and L > 5.0:
                self.canopy(f, L / 2, po, width=3.2)
        self.flat_roof(m, b, top, parapet=0.7)

    # ---- E (dorm 5) / 第5寮
    def b_dorm_tile(self, m, b):
        """Grey-beige tile, punched windows on a 1.95 m grid, the north stair
        bump with tall windows, AC units along the south base, the gabled porch,
        the lower level over the low platform. / タイル張り 4 階、下階、切妻の玄関。"""
        st, fl = b["storeys"], self.T["floor_m"]
        top = st * fl
        wall = "dorm_tile_greige"
        lowp = self.low.get(b["name"])
        for a, bb, L, n, zf, (u_off, Lfull) in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            po = self.room(a, bb, n)
            short = Lfull < 9.0
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                ops = []
                if short and L > 3.0:
                    ops = [(L / 2 - 0.5, L / 2 + 0.5, z0 + 0.6, z0 + 2.6)]
                elif L > 2.4:
                    for c0, c1 in B.bays(L, 1.95, 0.35):
                        c = (c0 + c1) / 2
                        ops.append((c - 0.55, c + 0.55, z0 + 0.95, z0 + 2.25))
                f.wall_with_openings(wall, z0, z1, ops, 2.0)
                for o in ops:
                    f.window(*o, wall, glass="dorm_sash", mullions=1, depth=0.15)
                if k > 0:
                    # stops 3 cm short of the corners (bands of two edges overlapped at
                    # reflex corners) / 入隅で重ならないよう端を詰める
                    f.block("dorm_white", 0.03, L - 0.03, 0.0, 0.03, z0 - 0.06, z0 + 0.06, top=True, bottom=True,
                            ends=False)
            if top > zf:
                f.face(wall, 0.0, L, max(zf, top - 0.001), top, 0.0)
            if lowp is not None:
                self.lower_storey(f, a, bb, n, lowp, fl, wall, punched=True)
            # AC units along the south base, one per pair of bays / 南面の室外機
            pc = self.porch_point(b)
            if n[1] < -0.7 and po > 0.8 and zf < 0.5 and L > 6:
                for c0, c1 in B.bays(L, 3.9, 0.6):
                    ua = (c0 + c1) / 2
                    pt = Point(*(a + (bb - a) / L * ua + n * 0.5))
                    if lowp is not None and lowp.buffer(0.5).contains(pt):
                        continue
                    if pc is not None and pc.distance(pt) < 2.6:
                        continue
                    f.block("jp_ac_unit", ua - 0.4, ua + 0.4, 0.12, 0.42, 0.0, 0.62, top=True, bottom=False)
            self.downpipes(f, L, top, po, short)
        if b.get("porch"):
            self.gable_porch(m, b)
        self.flat_roof(m, b, top, parapet=0.8)

    def porch_point(self, b):
        """The traced porch point moved back by the lean. / 玄関位置（傾き補正）。"""
        if not b.get("porch"):
            return None
        return Point(b["porch"][0] - self.lean[0] * b["h"], b["porch"][1] - self.lean[1] * b["h"])

    def gable_porch(self, m, b):
        """The small gabled entrance porch of dorm 5 (photo): two posts, a gable
        roof, glass doors, steps. / 第5寮の切妻の玄関ポーチ。"""
        pc = self.porch_point(b)
        best = None
        for a, bb, L, n in edges(b["poly"]):
            if n[1] < -0.7:
                d = LineString([tuple(a), tuple(bb)]).distance(pc)
                if best is None or d < best[0]:
                    best = (d, a, bb, L, n)
        if best is None:
            return
        _, a, bb, L, n = best
        f = B.Facade(m, a, bb, n)
        t = (bb - a) / L
        u = float(np.clip(np.dot(np.array(pc.coords[0]) - a, t), 1.8, L - 1.8))
        po = self.room(a, bb, n)
        dep = min(2.6, po - 0.05)
        f.face("dorm_glassdoor", u - 1.0, u + 1.0, 0.02, 2.3, 0.02)
        if dep < 1.0:
            return
        w = 1.6
        for s in (-1, 1):
            f.block("dorm_white", u + s * w - 0.1, u + s * w + 0.1, dep - 0.3, dep - 0.1, 0.0, 2.5, top=False,
                    bottom=False)
        f.block("dorm_paving", u - w, u + w, 0.0, dep, 0.0, 0.15, top=True, bottom=False)
        ze, zr = 2.5, 3.4
        P = lambda uu, ww, zz: f.P(uu, ww, zz)
        ov = 0.25
        # two slopes, gable triangle at the front, soffits / 屋根面・妻・軒裏
        # (t x n points down: the order below makes the slopes face up) / 上向きになる順序
        m.quad("jp_kawara_ibushi", P(u - w - ov, 0.0, ze), P(u - w - ov, dep + ov, ze), P(u, dep + ov, zr), P(u, 0.0, zr))
        m.quad("jp_kawara_ibushi", P(u + w + ov, dep + ov, ze), P(u + w + ov, 0.0, ze), P(u, 0.0, zr), P(u, dep + ov, zr))
        m.quad("jp_soffit", P(u - w - ov, dep + ov, ze - 0.05), P(u - w - ov, 0.0, ze - 0.05), P(u, 0.0, zr - 0.05),
               P(u, dep + ov, zr - 0.05))
        m.quad("jp_soffit", P(u + w + ov, 0.0, ze - 0.05), P(u + w + ov, dep + ov, ze - 0.05), P(u, dep + ov, zr - 0.05),
               P(u, 0.0, zr - 0.05))
        tri = [P(u - w, dep, ze), P(u + w, dep, ze), P(u, dep, zr - 0.05)]
        nf = np.r_[n, 0.0]
        m.tris("dorm_white", tri, nf, [(0, 0), (1, 0), (0.5, 0.4)], [0, 1, 2])
        f.block("dorm_white", u - w, u + w, dep - 0.3, dep, ze - 0.3, ze, top=False, bottom=True)

    def b_ext_stair_box(self, m, b):
        """E's east escape stair: an open frame of landing slabs with solid
        parapets on the three outer sides, posts, flights, a roof slab; it starts
        on the lower platform. / 東端の屋外階段（下階から）。"""
        poly = b["poly"]
        fl = self.T["floor_m"]
        k0 = -1 if b["name"] in self.low or (b.get("on") and b["on"] in self.low) else 0
        st = b["storeys"]
        a, ax, sd, Ln, W = axes(poly)
        par = self.blocks.get(b.get("on"))
        # the side touching the parent stays open / 親棟側は開放
        P2 = lambda u, v: a + ax * u + sd * v
        for k in range(k0, st + 1):
            z = k * fl
            slab = Polygon([tuple(P2(0, 0)), tuple(P2(Ln, 0)), tuple(P2(Ln, W)), tuple(P2(0, W))])
            if k == k0:
                # the ground slab it stands on (its platform has no ground under the
                # footprint) / 最下段の土間（足元に地面が無い）
                m.prism("dorm_floor", slab, z - 0.3, z + 0.02, top=True)
                continue
            m.prism("dorm_render", slab, z - 0.2, z, top=True, bottom=True)
            if k == st:
                continue
            strips = []
            for e0, e1 in ((P2(0, 0), P2(Ln, 0)), (P2(Ln, 0), P2(Ln, W)), (P2(Ln, W), P2(0, W)), (P2(0, W), P2(0, 0))):
                mid = (e0 + e1) / 2
                if par is not None and par["poly"].distance(Point(mid)) < 0.4:
                    continue
                strips.append(LineString([tuple(e0), tuple(e1)]).buffer(0.07, cap_style="flat", join_style="mitre"))
            # one solid for the parapet run (overlapping strips doubled their tops) / 手摺壁は一体に
            if strips:
                m.prism("dorm_spandrel", unary_union(strips).buffer(0), z, z + 1.1, top=True)
        # corner posts 2 cm proud of the slab edges (never coplanar) / 隅柱はスラブ端より 2 cm 外
        for u, v in ((0.15, 0.15), (Ln - 0.15, 0.15), (Ln - 0.15, W - 0.15), (0.15, W - 0.15)):
            q = P2(u + (-0.01 if u < Ln / 2 else 0.01), v + (-0.01 if v < W / 2 else 0.01))
            m.box("dorm_render", q[0], q[1], k0 * fl + 0.02, st * fl - 0.21, 0.34, 0.34, math.atan2(ax[1], ax[0]))
        for k in range(k0, st):
            z, z2 = k * fl, (k + 1) * fl
            if Ln > W:
                bot = [P2(0.4, 0.3), P2(0.4, W - 0.3), P2(Ln - 0.4, W - 0.3), P2(Ln - 0.4, 0.3)]
            else:
                bot = [P2(0.3, 0.4), P2(Ln - 0.3, 0.4), P2(Ln - 0.3, W - 0.4), P2(0.3, W - 0.4)]
            lo = [(bot[0][0], bot[0][1], z), (bot[1][0], bot[1][1], z), (bot[2][0], bot[2][1], z2 - 0.2),
                  (bot[3][0], bot[3][1], z2 - 0.2)]
            hi = [(p[0], p[1], p[2] + 0.18) for p in lo]
            _solid(m, "dorm_floor", lo, hi)

    def b_roof_room(self, m, b):
        """B's rooftop stair hall: a one-storey room with a window band, flat roof.
        B の屋上の階段室。"""
        h = 3.0
        for a, bb, L, n in edges(b["poly"]):
            f = B.Facade(m, a, bb, n)
            ops = [(c0 + 0.3, c1 - 0.3, 1.1, 2.1) for c0, c1 in B.bays(L, 1.8, 0.6)] if L > 3 else []
            f.wall_with_openings("dorm_render", 0.0, h, ops, 3.0)
            for o in ops:
                f.window(*o, "dorm_render", glass="dorm_sash", mullions=1, depth=0.12)
        self.flat_roof(m, dict(b, roof="teal"), h, parapet=0.5, things=False)

    # ---- office / 寮務室
    def b_office(self, m, b):
        """Accommodation office: white render, windows in bays, the glass entrance
        on the face toward the gate with steps and the red post box, a thin
        roof slab projecting all round (the photo), mint roof. / 寮務室。"""
        poly = b["poly"]
        h = 3.3
        # the longest face toward the gate (west); the L's inner edge is parallel / 西向きで最長の面
        fe = min(edges(poly), key=lambda e: round(e[3][0], 1) - 0.01 * e[2])
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            front = np.allclose(a, fe[0]) and np.allclose(bb, fe[1])
            ops = []
            if L > 3.0:
                ops = [(c0 + 0.45, c1 - 0.45, 0.9, 2.4) for c0, c1 in B.bays(L, 3.0, 0.5)]
            if front:
                ops = [o for o in ops if abs((o[0] + o[1]) / 2 - L / 2) > 2.4]
            f.wall_with_openings("dorm_white_wall", 0.0, h, ops + ([(L / 2 - 1.5, L / 2 + 1.5, 0.15, 2.5)] if front else []),
                                 3.0)
            for o in ops:
                f.window(*o, "dorm_white_wall", glass="dorm_sash", mullions=1, depth=0.12)
            if front:
                d = -0.2
                u0, u1 = L / 2 - 1.5, L / 2 + 1.5
                f.hplane("dorm_white_wall", u0, u1, d, 0.0, 2.5, False)
                f.side("dorm_white_wall", u0, d, 0.0, 0.15, 2.5, 1)
                f.side("dorm_white_wall", u1, d, 0.0, 0.15, 2.5, -1)
                f.hplane("dorm_tile_red", u0, u1, d, 0.0, 0.15, True)
                f.m.quad("dorm_glassdoor", f.P(u0, d, 0.15), f.P(u1, d, 0.15), f.P(u1, d, 2.5), f.P(u0, d, 2.5))
                po = self.room(a, bb, n)
                if po > 1.0:
                    # steps of red tile (photo) / 赤タイルの階段
                    f.block("dorm_tile_red", u0 - 0.6, u1 + 0.6, 0.0, min(1.6, po - 0.1), 0.0, 0.15, top=True,
                            bottom=False)
                    f.block("dorm_tile_red", u0 - 0.6, u1 + 0.6, 0.0, min(0.8, po - 0.1), 0.15, 0.3, top=True,
                            bottom=False)
                    pb = f.P(u1 + 1.0, 0.35, 0.0)
                    m.box("dorm_postbox", pb[0], pb[1], 0.0, 1.35, 0.45, 0.4, math.atan2(n[1], n[0]))
        # the thin roof slab projecting round the walls / 張り出した屋根スラブ
        room = min(self.room(*e[:2], e[3]) for e in edges(poly))
        ov = max(0.0, min(0.7, room - 0.05))
        slab = poly.buffer(ov, join_style="mitre") if ov > 0.02 else poly
        m.ring_walls("dorm_white", slab.exterior, h, h + 0.35, outward=True)
        m.flat("dorm_soffit", slab, h, 2.0, up=False)
        inner = slab.buffer(-0.15, join_style="mitre")
        m.flat(ROOFS["mint"], inner, h + 0.3, 3.0)
        m.flat("dorm_white", slab.difference(inner), h + 0.45, 1.0)
        for p in polys(inner):
            m.ring_walls("dorm_white", p.exterior, h + 0.3, h + 0.45, outward=False)
        m.ring_walls("dorm_white", slab.exterior, h + 0.35, h + 0.45, outward=True)
        # AC units on the roof / 屋上の室外機
        c = poly.representative_point()
        m.box("jp_ac_unit", c.x, c.y, h + 0.3, h + 1.0, 0.9, 0.4)

    # ---- dining / bath / 食堂・浴場
    def b_hall(self, m, b):
        """A single-storey part of the dining/bath complex: light grey render,
        high windows, steel doors, flat roof with a parapet; solar arrays on the
        bath wing, a skylight roof on the kitchen. / 食堂・浴場の各部。"""
        poly = b["poly"]
        h = b["h"] - 0.6
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            if zf >= h or L < 1.0:
                continue
            z0 = max(0.0, zf)
            ops = []
            if L > 4.0:
                ops = [(c0 + 0.4, c1 - 0.4, max(z0 + 0.3, 1.4), h - 0.8) for c0, c1 in B.bays(L, 3.2, 0.6)]
                ops = [o for o in ops if o[3] - o[2] > 0.6]
            doors = []
            if zf < 0.5 and L > 7.0:
                mid = len(ops) // 2
                if ops:
                    doors = [ops.pop(mid)]
            f.wall_with_openings("dorm_render_grey", z0, h, ops + [(d[0], d[1], 0.02, 2.2) for d in doors], 3.0)
            for o in ops:
                f.window(*o, "dorm_render_grey", glass="dorm_sash", mullions=2, depth=0.12)
            for d in doors:
                f.face("jp_door_steel", d[0], d[1], 0.02, 2.2, -0.05)
                f.side("dorm_render_grey", d[0], -0.05, 0.0, 0.02, 2.2, 1)
                f.side("dorm_render_grey", d[1], -0.05, 0.0, 0.02, 2.2, -1)
                f.hplane("dorm_render_grey", d[0], d[1], -0.05, 0.0, 2.2, False)
        roof = ROOFS.get(b.get("roof"), "dorm_roof_palegrey")
        inner = poly.buffer(-0.18, join_style="mitre")
        m.flat(roof, inner, h + 0.02, 4.0 if roof != "dorm_roof_toplit" else 1.0)
        for p in polys(inner):
            m.ring_walls("dorm_render_grey", p.exterior, h, h + 0.5, outward=False, uv_scale=3.0)
        m.ring_walls("dorm_render_grey", poly.exterior, h, h + 0.5, outward=True, uv_scale=3.0)
        m.flat("dorm_white", poly.difference(inner), h + 0.5, 1.0)
        if inner.is_empty:
            return
        a, ax, sd, Ln, W = axes(inner)
        ang = math.atan2(ax[1], ax[0])
        if b.get("solar"):
            # two arrays of three rows tilted south (GE, the tile) / 太陽光 2 群
            for s0 in (0.12, 0.56):
                self.solar(m, inner, h + 0.3, a + ax * (Ln * s0) + sd * (W * 0.12), ax, sd, Ln * 0.36, W * 0.55)
        # roof units / 屋上設備
        hh = sum(ord(ch) for ch in b["name"])
        for i in range(2 + hh % 3):
            q = a + ax * (Ln * (0.2 + 0.25 * i)) + sd * (W * (0.75 if i % 2 else 0.35))
            if inner.buffer(-0.8).contains(Point(q)) and not b.get("solar"):
                m.box("jp_ac_unit", q[0], q[1], h + 0.02, h + 0.9, 1.2, 0.6, ang)
        if b["name"] == "hall_n":
            # louvred vent boxes on the high hall / 換気塔
            c = inner.representative_point()
            m.box("dorm_louvre", c.x, c.y, h, h + 1.4, 2.2, 1.6, ang)
            m.box("dorm_white", c.x, c.y, h + 1.4, h + 1.5, 2.5, 1.9, ang)

    def solar(self, m, area, z, origin, ax, sd, Ln, Wd, tilt=12.0):
        """Rows of panels tilted to the south side on low frames. / 太陽光パネル。"""
        slope = math.tan(math.radians(tilt))
        v = 0.0
        while v + 1.4 <= Wd:
            q = [origin + sd * v, origin + ax * Ln + sd * v, origin + ax * Ln + sd * (v + 1.4), origin + sd * (v + 1.4)]
            strip = Polygon([tuple(p) for p in q]).intersection(area.buffer(-0.4))
            base = origin + sd * (v + 1.4) if sd[1] > 0 else origin + sd * v
            zf = (lambda x, y, base=base: z + slope * np.abs((x - base[0]) * sd[0] + (y - base[1]) * sd[1]))
            for p in polys(strip):
                if p.area > 1.0:
                    m.flat("solar", p, z, 1.0, zf=zf)
            v += 1.9

    # ---- small buildings / 小さな建物
    def low_walls(self, m, b, h, wall="dorm_white_wall", door=True, shutter=False, louvre=False):
        """Walls of a small single-storey building: a door, a window or two. / 小屋の外壁。"""
        best = max(edges(b["poly"]), key=lambda e: e[2])
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            if zf >= h:
                continue
            ops = []
            if L > 3.2:
                ops = [(c0 + 0.5, c1 - 0.5, 1.0, min(h - 0.4, 2.0)) for c0, c1 in B.bays(L, 3.0, 0.8)][:4]
            first = np.allclose(a, best[0]) and door and L > 2.4
            if first and ops:
                ops = ops[1:]
            f.wall_with_openings(wall, max(0.0, zf), h, ops, 2.0)
            for o in ops:
                if louvre:
                    f.face("dorm_louvre", o[0], o[1], o[2], o[3], -0.05)
                    f.side(wall, o[0], -0.05, 0.0, o[2], o[3], 1)
                    f.side(wall, o[1], -0.05, 0.0, o[2], o[3], -1)
                    f.hplane(wall, o[0], o[1], -0.05, 0.0, o[3], False)
                    f.hplane(wall, o[0], o[1], -0.05, 0.0, o[2], True)
                else:
                    f.window(*o, wall, glass="dorm_sash", mullions=1, depth=0.1)
            if first:
                if shutter and L > 5:
                    f.face("jp_shutter", 0.6, min(L - 0.6, 3.6), 0.02, min(h - 0.4, 2.6), 0.02)
                else:
                    f.face("jp_door_steel", 0.7, 1.6 if not louvre else 2.5, 0.02, 2.05, 0.02)

    def b_small_rc(self, m, b):
        """Electrical room (RC1): white render, steel double door, louvres, flat
        grey roof with a parapet. / 電気室。"""
        h = b["h"] - 0.5
        self.low_walls(m, b, h, wall="dorm_render", louvre=True)
        poly = b["poly"]
        inner = poly.buffer(-0.15, join_style="mitre")
        m.flat(ROOFS["grey"], inner, h + 0.02, 3.0)
        m.ring_walls("dorm_render", inner.exterior, h, h + 0.45, outward=False)
        m.ring_walls("dorm_render", poly.exterior, h, h + 0.45, outward=True)
        m.flat("dorm_white", poly.difference(inner), h + 0.45, 1.0)

    def b_shed(self, m, b):
        h = b["h"] - 0.3
        self.low_walls(m, b, h, shutter=b["poly"].area > 50)
        poly = b["poly"]
        m.flat(ROOFS.get(b.get("roof"), "dorm_roof_grey"), poly, h + 0.25, 3.0)
        m.ring_walls("dorm_white", poly.exterior, h, h + 0.25, outward=True)

    def b_shed_corr(self, m, b):
        """Laundry house: white walls, a low corrugated mono-pitch roof with eaves.
        洗濯棟（波板の片流れ屋根）。"""
        h = b["h"] - 0.5
        self.low_walls(m, b, h)
        self.mono_roof(m, b["poly"], h, 0.5, ROOFS.get(b.get("roof"), "dorm_roof_greymetal"), 0.3)

    def b_clubroom(self, m, b):
        """Soccer clubroom: prefab, white metal siding, a door and windows, a
        mono-pitch galvalume roof. / サッカー部部室（プレハブ）。"""
        h = 2.6
        self.low_walls(m, b, h, wall="jp_metal_white")
        self.mono_roof(m, b["poly"], h, 0.35, "jp_roof_galvalume", 0.25)

    def mono_roof(self, m, poly, h, rise, mat, ov):
        """A mono-pitch roof over a rectangle's long axis, eaves ov (clipped by
        the room), gable infill triangles. / 片流れ屋根。"""
        poly = fit_rect(poly)
        a, ax, sd, Ln, W = axes(poly)
        room = min(self.room(*e[:2], e[3]) for e in edges(poly))
        ov = max(0.05, min(ov, room - 0.05))
        P3 = lambda u, v, z: (*(a + ax * u + sd * v), z)
        slope = rise / W
        zl, zh = h - ov * slope, h + rise + ov * slope
        m.quad(mat, P3(-ov, -ov, zl), P3(Ln + ov, -ov, zl), P3(Ln + ov, W + ov, zh), P3(-ov, W + ov, zh),
               (0, 0), ((Ln + 2 * ov) / 2, 0), ((Ln + 2 * ov) / 2, (W + 2 * ov) / 2), (0, (W + 2 * ov) / 2))
        m.quad("jp_soffit", P3(Ln + ov, -ov, zl - 0.04), P3(-ov, -ov, zl - 0.04), P3(-ov, W + ov, zh - 0.04),
               P3(Ln + ov, W + ov, zh - 0.04))
        m.tris("dorm_white_wall", [P3(0, 0, h), P3(0, W, h), P3(0, W, h + rise)], np.r_[-ax, 0.0],
               [(0, 0), (1, 0), (1, 0.3)], [0, 2, 1])
        m.tris("dorm_white_wall", [P3(Ln, 0, h), P3(Ln, W, h), P3(Ln, W, h + rise)], np.r_[ax, 0.0],
               [(0, 0), (1, 0), (1, 0.3)], [0, 1, 2])
        m.wall("dorm_white_wall", tuple(a + sd * W + ax * Ln), tuple(a + sd * W), h, h + rise, 0.0, 2.0)

    def b_bikeshed(self, m, b):
        """Bicycle shelter: steel posts, a mono-pitch roof, open sides, two rack
        rails. / 自転車置場。"""
        poly = b["poly"]
        mat = ROOFS.get(b.get("roof"), "dorm_roof_white")
        a, ax, sd, Ln, W = axes(poly)
        lo, hi = 2.2, 2.5
        P3 = lambda u, v, z: (*(a + ax * u + sd * v), z)
        m.quad(mat, P3(0, 0, lo), P3(Ln, 0, lo), P3(Ln, W, hi), P3(0, W, hi), (0, 0), (Ln / 2, 0), (Ln / 2, W / 2),
               (0, W / 2))
        m.quad("jp_soffit", P3(Ln, 0, lo - 0.04), P3(0, 0, lo - 0.04), P3(0, W, hi - 0.04), P3(Ln, W, hi - 0.04))
        ang = math.atan2(ax[1], ax[0])
        for u in np.linspace(0.3, Ln - 0.3, max(2, int(Ln // 3.0) + 1)):
            for v, zz in ((0.3, lo), (W - 0.3, hi)):
                q = a + ax * u + sd * v
                m.box("dorm_steel", q[0], q[1], 0.0, zz - 0.04, 0.08, 0.08, ang)
        for v in (W * 0.35, W * 0.7):
            q0, q1 = a + sd * v + ax * 0.5, a + sd * v + ax * (Ln - 0.5)
            m.wall("dorm_steel", tuple(q0), tuple(q1), 0.3, 0.36, 0.0, 1.0)
            m.wall("dorm_steel", tuple(q1), tuple(q0), 0.3, 0.36, 0.0, 1.0)
        m.flat("dorm_paving", poly, 0.03, 2.0)

    # ================= site things / 敷地の物 =================
    def site_things(self, m):
        self.gate_and_frontage(m)
        self.fences(m)
        self.chimney(m)
        self.walkway(m)
        self.garden_frames(m)
        self.trees(m)

    def follow(self, dense=2.0):
        """A Mesh that follows the finished ground (fences, walls). / 地盤に沿うメッシュ。"""
        m = Mesh()
        m.zfun = lambda x, y: self.S.surface_z(x, y)
        m.dense = dense
        return m

    def rigid(self, x, y, r=0.3):
        m = Mesh()
        m.zoff = self.S.low(x, y, r)
        return m

    def clear_line(self, line, clear=0.35, extra=None):
        """A line cut clear of roads, paths and buildings. / 道路・歩道・建物を避けた線。"""
        stop = unary_union([self.no_go.buffer(clear), self.footprint.buffer(0.4)] + ([extra] if extra is not None else []))
        g = line.difference(stop)
        return [q for q in getattr(g, "geoms", [g]) if q.geom_type == "LineString" and q.length > 0.8]

    def wall_run(self, m, line, h, mat, top_mat, thick=0.2, hedge=False, caps=(True, True)):
        """A solid wall along a line: both faces, the top; clipped shrubs behind
        (inside) if asked. / 塀（両面・天端、内側に生垣）。"""
        fm = self.follow()
        c = list(line.coords)
        for a, b in zip(c, c[1:]):
            L = math.dist(a, b)
            if L < 1e-3:
                continue
            t = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
            n = (t[1], -t[0])
            o = thick / 2
            a1, b1 = (a[0] + n[0] * o, a[1] + n[1] * o), (b[0] + n[0] * o, b[1] + n[1] * o)
            a2, b2 = (a[0] - n[0] * o, a[1] - n[1] * o), (b[0] - n[0] * o, b[1] - n[1] * o)
            fm.wall(mat, a1, b1, -0.3, h, 0.0, 1.0)              # faces right / 右向き
            fm.wall(mat, b2, a2, -0.3, h, 0.0, 1.0)              # faces left / 左向き
            fm.flat(top_mat, Polygon([a1, b1, b2, a2]).buffer(0), h, 1.0)
        # end caps where the wall stops (the gate gaps) / 端部の小口
        if len(c) >= 2:
            for p0, p1, want in ((c[0], c[1], caps[0]), (c[-1], c[-2], caps[1])):
                L = math.dist(p0, p1)
                if not want:
                    continue
                if L < 1e-3:
                    continue
                t = ((p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L)
                n = (t[1], -t[0])
                o = thick / 2
                q1, q2 = (p0[0] + n[0] * o, p0[1] + n[1] * o), (p0[0] - n[0] * o, p0[1] - n[1] * o)
                # facing away from the wall (-t): q2 -> q1 (right of it is -t) / 外向き
                fm2 = self.follow()
                fm2.wall(mat, q2, q1, -0.3, h, 0.0, 1.0)
                m.merge(fm2)
        m.merge(fm)
        if hedge:
            hm = self.follow()
            g = line.buffer(0.45, cap_style="flat", single_sided=False).difference(line.buffer(0.12, cap_style="flat"))
            g = onterrain._clean(g.intersection(self.region()).difference(self.footprint.buffer(0.3))
                                 .difference(self.no_go.buffer(0.3)))
            # only the side away from the road: keep pieces whose centre is farther from the road / 道路と反対側
            keep = [p for p in polys(g) if self.road.distance(p.centroid) > self.road.distance(line) + 0.1]
            for p in keep:
                hm.prism("jp_hedge", p, -0.2, h + 0.25, top=True)
            m.merge(hm)

    def gate_and_frontage(self, m):
        """The main-road frontage (Street View 2026-02): two entries split by the
        white tiled planter wall with the name plate, white tiled walls with
        clipped shrubs, the brown louvre fence and the grey precast panel wall in
        front of A. / 幹線沿い: 植栽桝と銘板・白タイル塀・ルーバー柵・PC 板塀。"""
        g = self.gate
        x = g["x"]
        stop = self.no_go.buffer(0.35)
        reg = self.region()
        # where the dorm's own ground starts at the frontage / 敷地の西端
        xs = []
        for y in np.arange(-216.0, -128.0, 2.0):
            ln = LineString([(x - 4.0, y), (x + 6.0, y)]).intersection(reg)
            if not ln.is_empty:
                xs.append(ln.bounds[0])
        x = max(x, float(np.median(xs)) + 0.35) if xs else x
        self.front_x = x
        px0, py0, px1, py1 = g["planter"]
        planter = box(max(px0, x - 0.1), py1, px1, py0).difference(stop)
        for p in polys(planter):
            if p.area < 2.0:
                continue
            r = self.rigid(p.centroid.x, p.centroid.y, 1.5)
            ring = p.difference(p.buffer(-0.22, join_style="mitre"))
            r.prism("dorm_white_tile", ring, -0.3, 1.0, top=True)
            r.flat("soil", p.buffer(-0.22, join_style="mitre"), 0.75, 1.5)
            hb = p.buffer(-0.35, join_style="mitre")
            if not hb.is_empty:
                r.prism("jp_hedge", hb, 0.75, 1.35, top=True)
            # name plate on the west face / 西面の銘板
            x0, y0, x1, y1 = p.bounds
            w = min(2.4, (y1 - y0) - 0.6)
            if w > 0.8:
                yc = (y0 + y1) / 2
                r.quad("dorm_plate", (x0 - 0.015, yc + w / 2, 0.35), (x0 - 0.015, yc - w / 2, 0.35),
                       (x0 - 0.015, yc - w / 2, 0.35 + w * 0.22), (x0 - 0.015, yc + w / 2, 0.35 + w * 0.22))
            m.merge(r)
        nw, sw = g["north_wall"], g["south_wall"]
        # caps only at free ends, not where the next run continues (back-to-back caps)
        # 小口は自由端のみ（続く塀との間には付けない）
        for (ya, yb), kind, caps in ((nw, "tile", (True, True)), ((sw[0], sw[0] - 4.0), "tile", (True, False)),
                                     ((sw[0] - 4.0, sw[1]), "louvre", (False, False)), (g["panel_wall"], "panel", None)):
            line = LineString([(x, ya), (x, yb)])
            for q in self.clear_line(line, 0.35):
                if kind == "tile":
                    self.wall_run(m, q, 1.0, "dorm_white_tile", "dorm_white", thick=0.22, hedge=True, caps=caps)
                elif kind == "louvre":
                    self.wall_run(m, q, 0.45, "dorm_white_tile", "dorm_white", thick=0.22, caps=caps)
                    self.louvre_fence(m, q)
                else:
                    self.panel_wall(m, q, 2.0)

    def slab_run(self, m, line, z0, z1, mat, thick, uv=1.0):
        """A thin solid along a line: two faces thick apart (never back to back
        on one plane: Cycles renders such pairs black), the top. / 厚みのある板。"""
        fm = self.follow()
        c = list(line.coords)
        o = thick / 2
        for a, b in zip(c, c[1:]):
            L = math.dist(a, b)
            if L < 1e-3:
                continue
            n = ((b[1] - a[1]) / L, -(b[0] - a[0]) / L)
            a1, b1 = (a[0] + n[0] * o, a[1] + n[1] * o), (b[0] + n[0] * o, b[1] + n[1] * o)
            a2, b2 = (a[0] - n[0] * o, a[1] - n[1] * o), (b[0] - n[0] * o, b[1] - n[1] * o)
            fm.wall(mat, a1, b1, z0, z1, 0.0, uv)
            fm.wall(mat, b2, a2, z0, z1, 0.0, uv)
            fm.flat(mat, Polygon([a1, b1, b2, a2]).buffer(0), z1, 1.0)
        m.merge(fm)

    def louvre_fence(self, m, line):
        """Brown aluminium louvre fence on the low wall. / 茶色のルーバー柵。"""
        self.slab_run(m, line, 0.45, 1.6, "dorm_louvre_brown", 0.05)
        Lg = line.length
        for k in range(int(Lg // 2.0) + 1):
            q = line.interpolate(min(Lg, k * 2.0))
            r = self.rigid(q.x, q.y, 0.1)
            r.box("dorm_steel_brown", q.x, q.y, 0.4, 1.65, 0.08, 0.08)
            m.merge(r)

    def panel_wall(self, m, line, h):
        """Grey precast concrete panel wall: H posts every 2 m, panels between
        (both faces), a cap. / PC 板塀。"""
        self.slab_run(m, line, -0.3, h, "dorm_panel_grey", 0.1, uv=2.0)
        Lg = line.length
        for k in range(int(Lg // 2.0) + 1):
            q = line.interpolate(min(Lg, k * 2.0))
            r = self.rigid(q.x, q.y, 0.15)
            r.box("dorm_panel_grey", q.x, q.y, -0.3, h + 0.08, 0.25, 0.25)
            m.merge(r)

    def mesh_fence(self, m, line, h=1.8, post=2.0, mat="dorm_mesh_white", post_mat="dorm_steel_white"):
        """Welded mesh fence on a low kerb (south and east, Street View 2024-07):
        posts, rails, thin vertical wires (two-sided strips). / メッシュフェンス。"""
        Lg = line.length
        for k in range(int(Lg // post) + 1):
            q = line.interpolate(min(Lg, k * post))
            r = self.rigid(q.x, q.y, 0.1)
            r.box(post_mat, q.x, q.y, -0.2, h + 0.05, 0.06, 0.06)
            m.merge(r)
        fm = self.follow()
        # the kerb's top as one polygon for the run (per-segment tops overlapped at
        # the bends) / 縁石の天端は一枚
        fm.flat("dorm_plinth", line.buffer(0.06, cap_style="flat", join_style="mitre"), 0.15, 1.0)
        c = list(line.coords)
        for a, b in zip(c, c[1:]):
            seg = math.dist(a, b)
            if seg < 1e-3:
                continue
            nn = ((b[1] - a[1]) / seg * 0.06, -(b[0] - a[0]) / seg * 0.06)
            fm.wall("dorm_plinth", (a[0] + nn[0], a[1] + nn[1]), (b[0] + nn[0], b[1] + nn[1]), -0.2, 0.15, 0.0, 1.0)
            fm.wall("dorm_plinth", (b[0] - nn[0], b[1] - nn[1]), (a[0] - nn[0], a[1] - nn[1]), -0.2, 0.15, 0.0, 1.0)
            for zz in (0.25, h - 0.05):
                q = ((b[1] - a[1]) / seg * 0.02, -(b[0] - a[0]) / seg * 0.02)
                fm.wall(post_mat, (a[0] + q[0], a[1] + q[1]), (b[0] + q[0], b[1] + q[1]), zz - 0.03, zz + 0.03, 0.0, 1.0)
                fm.wall(post_mat, (b[0] - q[0], b[1] - q[1]), (a[0] - q[0], a[1] - q[1]), zz - 0.03, zz + 0.03, 0.0, 1.0)
            t = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
            dense, fm.dense = fm.dense, None
            zfun = fm.zfun
            for k in range(int(seg // 0.2) + 1):
                u = min(seg, k * 0.2)
                p0 = (a[0] + t[0] * (u - 0.006), a[1] + t[1] * (u - 0.006))
                p1 = (a[0] + t[0] * (u + 0.006), a[1] + t[1] * (u + 0.006))
                zb = float(self.S.surface_z([0.5 * (p0[0] + p1[0])], [0.5 * (p0[1] + p1[1])])[0])
                fm.zfun = None
                o = (t[1] * 0.006, -t[0] * 0.006)                    # the two faces 12 mm apart / 両面を離す
                fm.wall(mat, (p0[0] + o[0], p0[1] + o[1]), (p1[0] + o[0], p1[1] + o[1]), zb + 0.15, zb + h, 0.0, 1.0)
                fm.wall(mat, (p1[0] - o[0], p1[1] - o[1]), (p0[0] - o[0], p0[1] - o[1]), zb + 0.15, zb + h, 0.0, 1.0)
                fm.zfun = zfun
            fm.dense = dense
        m.merge(fm)

    def fences(self, m):
        """The site boundary away from the main road: the grey panel wall south of
        A and B, white mesh fence on the rest (south-east, the company car park
        side); gaps where roads and paths cross. / 敷地境界の塀と柵。"""
        ring = self.site.buffer(-0.8, join_style="mitre")
        ring = max(polys(ring), key=lambda p: p.area).exterior
        x = getattr(self, "front_x", self.gate["x"])
        front = box(x - 3.0, -230.0, x + 1.6, -120.0)                     # the main-road side has its own / 幹線側は別
        north = box(-50.0, -132.0, 160.0, -110.0)                         # open to the north lane / 北は開放
        south_b = Polygon([(16.0, -222.0), (82.0, -199.0), (82.0, -192.0), (16.0, -214.0)])
        n_panel = n_mesh = 0.0
        for g in self.clear_line(ring, 0.4, extra=unary_union([front, north])):
            pb = g.intersection(south_b)
            rest = g.difference(south_b.buffer(0.01))
            for q in getattr(pb, "geoms", [pb]):
                if q.geom_type == "LineString" and q.length > 1.0:
                    self.panel_wall(m, q, 2.0)
                    n_panel += q.length
            for q in getattr(rest, "geoms", [rest]):
                if q.geom_type == "LineString" and q.length > 1.0:
                    self.mesh_fence(m, q)
                    n_mesh += q.length
        print("landmark %s: boundary panel wall %.0f m, mesh fence %.0f m" % (TAG, n_panel, n_mesh))

    def chimney(self, m):
        """The white boiler chimney by the bath wing (GE, ~10 m). / 浴場のボイラー煙突。"""
        if "bath" not in self.blocks:
            return
        poly = self.blocks["bath"]["poly"]
        a, ax, sd, Ln, W = axes(poly)
        # just outside the wing's north-west corner, clear of everything / 北西角の外
        cands = [Point(75.6, -135.4 - self.lean[1] * 10.0)]
        cands += [Point(*(np.array(poly.exterior.coords[i]) + d)) for i in range(len(poly.exterior.coords) - 1)
                  for d in ((-1.0, 0.6), (-1.0, -0.6), (0.8, 0.8))]
        stop = unary_union([self.no_go.buffer(1.0), self.footprint.buffer(0.15)])
        reg = self.region().buffer(-0.6)
        for c in cands:
            if reg.contains(c) and not stop.contains(c) and stop.distance(c) > 0.55:
                r = self.rigid(c.x, c.y, 0.5)
                r.cylinder("dorm_white", c.x, c.y, -0.3, 10.0, 0.42, seg=14, top=False)
                r.cylinder("dorm_steel", c.x, c.y, 10.0, 10.35, 0.46, seg=14, top=True)
                r.box("dorm_render_grey", c.x, c.y, -0.3, 0.6, 1.3, 1.3)
                m.merge(r)
                return

    def walkway(self, m):
        """Covered walkway along the west side of the bath and dining wings (the
        dorm's photo): brown steel posts and beams, a translucent roof. / 渡り廊下。"""
        line = LineString([(73.2, -140.6), (77.0, -147.6), (80.2, -155.5)])
        stop = unary_union([self.no_go.buffer(0.4), self.footprint.buffer(0.35)])
        for g in polys(line.buffer(1.0, cap_style="flat", join_style="mitre").difference(stop)):
            if g.area < 4.0:
                continue
            core = g.buffer(-0.05)
            r = self.rigid(g.centroid.x, g.centroid.y, 2.0)
            z = 2.6
            r.flat("dorm_polycarb", core, z + 0.12, 2.0)
            r.flat("dorm_polycarb", core, z + 0.1, 2.0, up=False)
            r.ring_walls("dorm_steel_brown", core.exterior, z - 0.12, z + 0.12, outward=True)
            for q in self.clear_points(g.exterior, 3.0):
                r.box("dorm_steel_brown", q.x, q.y, 0.0, z, 0.1, 0.1)
            m.merge(r)

    def clear_points(self, ring, step):
        Lg = ring.length
        out = []
        for k in range(int(Lg // step)):
            q = ring.interpolate(k * step + 0.3)
            out.append(q)
        return out

    def garden_frames(self, m):
        """Kitchen gardens (the tile): soil beds with low white pipe frames. / 菜園の白いパイプ枠。"""
        for k in ("garden_c", "garden_d"):
            g = onterrain._clean(self.ground[k].intersection(self.region()).buffer(-0.4)
                                 .difference(self.footprint.buffer(0.5)))
            for p in polys(g):
                if p.area < 6.0:
                    continue
                a, ax, sd, Ln, W = axes(p)
                ang = math.atan2(ax[1], ax[0])
                for i in range(max(2, int(Ln // 1.8) + 1)):
                    u = 0.3 + (Ln - 0.6) * i / max(1, int(Ln // 1.8))
                    for v in (0.3, W - 0.3):
                        q = a + ax * u + sd * v
                        if p.contains(Point(q)):
                            r = self.rigid(q[0], q[1], 0.05)
                            r.box("dorm_steel_white", q[0], q[1], -0.2, 1.2, 0.04, 0.04, ang)
                            m.merge(r)
                fm = self.follow()
                for v in (0.3, W - 0.3):
                    q0, q1 = a + sd * v + ax * 0.3, a + sd * v + ax * (Ln - 0.3)
                    fm.wall("dorm_steel_white", tuple(q0), tuple(q1), 1.17, 1.21, 0.0, 1.0)
                    fm.wall("dorm_steel_white", tuple(q1), tuple(q0), 1.17, 1.21, 0.0, 1.0)
                m.merge(fm)

    def trees(self, m):
        """Trees where the imagery shows canopy (dorm_trace.json), off the
        buildings and paved areas; crowns kept off roads, paths and walls. / 樹木。"""
        S = self.S
        busy = unary_union([self.footprint.buffer(1.3), self.paved.buffer(0.3)]
                           + [p.poly.buffer(-0.3) for p in S.items if p.kind == "building"])
        shapely.prepare(busy)
        region = self.region()
        shapely.prepare(region)
        n = 0
        for (x, y), r in self.trees_seen:
            if not shapely.contains_xy(region, x, y) or shapely.contains_xy(busy, x, y):
                continue
            d = float(shapely.distance(self.no_go, Point(x, y)))
            r2 = min(r, d - TREE_CLEAR, float(self.footprint.distance(Point(x, y))) + 0.6)
            if r2 < 1.1:
                continue
            # a seasonal Japanese tree (jtrees.py), leaves in their species' mesh
            # 季節の樹木（葉は樹種ごとのメッシュ）
            tm = self.rigid(x, y, 0.4)
            jtrees.landmark_tree(tm, self.leaves, self.rigid, x, y, r2, "park")
            m.merge(tm)
            n += 1
        print("landmark %s: %d trees" % (TAG, n))


def write_textures(tex_dir):
    """The dorm's drawn textures (procedural, from the colours in the Google
    Earth description; no imagery used): renders, spandrels, tile, roofs,
    sashes, balcony doors, partitions, gate tiles, panel wall, louvres, mesh,
    the name plate, the post box. Each with roughness / metallic maps and a
    normal map from its own height. / 寮の描画テクスチャ。"""
    import textures as X
    from PIL import Image, ImageDraw, ImageFont
    out = {}
    rs = np.random.default_rng(2313)

    def noise(n, scales=(4, 16, 64), amps=(0.5, 0.3, 0.2)):
        return X.noise(n, scales, amps)

    def save(name, im, rough=0.85, metal=0.0, relief=0.0, height=None):
        assert "sign" not in name and "light" not in name
        im = im.convert("RGB")
        im.save(os.path.join(tex_dir, name + ".png"))
        n = im.size[0]
        sq = im.size[0] == im.size[1]
        lum = lambda: (lambda a: a - a.mean())(np.asarray(im.convert("L"), np.float64) / 255.0)
        if relief > 0:
            h = height if height is not None else (X.height_of(im, 0.3) if sq else lum())
            X.normal_map(h / max(np.abs(h).max(), 1e-6), relief).save(os.path.join(tex_dir, name + "_n.png"))
        r = np.clip(rough + 0.05 * ((X.height_of(im, 0.5) if sq else lum()) if n <= 1024 else 0), 0, 1)
        Image.fromarray((X.srgb_encode(np.asarray(r, np.float64)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_r.png"))
        Image.fromarray((X.srgb_encode(np.full((8, 8), metal)) * 255).astype(np.uint8)).save(
            os.path.join(tex_dir, name + "_m.png"))
        out[name] = (name, None, rough, metal, False)

    def tinted(c0, c1, n=512, scales=(4, 16, 64)):
        return Image.fromarray(X.tint(noise(n, scales), c0, c1))

    def render(c, n=512, joints=0, streak=True, base=True):
        """Painted render, faint rain streaks, a darker base band (UV v = height,
        3 m tile). / 塗装（雨だれ・裾の汚れ）。"""
        a = np.asarray(tinted(np.array(c) * 0.94, np.array(c) * 1.03, n, (8, 32, 128)), np.float64)
        if streak:
            for _ in range(22):
                x = int(rs.integers(0, n))
                w = int(rs.integers(2, 6))
                ln = int(rs.integers(n // 8, n // 3))
                y0 = int(rs.integers(0, n - ln))
                fade = np.linspace(0.0, 1.0, ln)[:, None, None] * 0.08
                a[n - (y0 + ln):n - y0, x:x + w] *= (1 - fade[::-1])
        if base:
            a[int(n * 0.93):, :] *= 0.9
        if joints:
            for k in range(1, joints):
                a[int(n * k / joints) - 1:int(n * k / joints) + 1, :] *= 0.85
        return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))

    save("dorm_render", render((238, 232, 218)), 0.88, 0.0, 0.6)
    save("dorm_render_grey", render((214, 214, 210)), 0.88, 0.0, 0.6)
    save("dorm_white_wall", render((236, 236, 232)), 0.85, 0.0, 0.4)
    save("dorm_spandrel", render((216, 200, 168), joints=2, base=False), 0.85, 0.0, 0.8)
    save("dorm_white", render((242, 241, 236), streak=False, base=False), 0.8, 0.0, 0.3)
    save("dorm_canopy_edge", render((235, 233, 226), streak=False, base=False), 0.8, 0.0, 0.3)
    save("dorm_soffit", render((206, 203, 196), streak=False, base=False), 0.9, 0.0, 0.2)
    save("dorm_floor", X.concrete(256, (150, 148, 140), (175, 172, 165), joints=4, jc=(120, 118, 110)), 0.9, 0.0, 0.6)
    save("dorm_plinth", X.concrete(256, (165, 163, 155), (190, 188, 180), joints=2), 0.9, 0.0, 0.6)
    save("dorm_partition", tinted((222, 220, 212), (236, 234, 228), 128), 0.6, 0.1, 0.0)

    def tiles(c, n=512, k=16, kv=None, grout=(200, 195, 185), jitter=0.0):
        kv = kv or k * 2
        base = np.asarray(tinted(np.array(c) * 0.9, np.array(c) * 1.06, n, (n // 32, n // 8, n // 2)), np.float64)
        if jitter > 0:
            # each tile a little lighter or darker / タイルごとの色むら
            for i in range(kv):
                for j in range(k + 1):
                    x0 = int((j + (0.5 if i % 2 else 0) - 1) * n / k)
                    f = 1.0 + rs.uniform(-jitter, jitter)
                    base[int(i * n / kv):int((i + 1) * n / kv), max(0, x0):max(0, int(x0 + n / k))] *= f
        im = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8))
        d = ImageDraw.Draw(im)
        for i in range(kv + 1):
            d.line([(0, i * n / kv), (n, i * n / kv)], fill=grout, width=2)
        for j in range(k + 1):
            for i in range(kv):
                x = (j + (0.5 if i % 2 else 0)) * n / k
                d.line([(x, i * n / kv), (x, (i + 1) * n / kv)], fill=grout, width=2)
        return im

    save("dorm_tile_greige", tiles((170, 164, 152), k=8, kv=24, grout=(205, 202, 194), jitter=0.06), 0.75, 0.0, 1.2)
    save("dorm_white_tile", tiles((238, 238, 234), k=6, kv=6, grout=(190, 190, 186)), 0.5, 0.0, 0.8)
    save("dorm_tile_red", tiles((150, 70, 55), k=4, kv=4, grout=(120, 105, 95)), 0.8, 0.0, 0.8)
    pw = tiles((168, 168, 162), k=1, kv=4, grout=(130, 130, 126))
    save("dorm_panel_grey", pw, 0.9, 0.0, 0.9)

    def corrugated(c, n=256, k=32, vertical=True):
        a = np.asarray(tinted(np.array(c) * 0.95, np.array(c) * 1.03, n, (4, 16, 64)), np.float64)
        wave = 0.9 + 0.1 * np.sin(np.linspace(0, 2 * np.pi * k, n, endpoint=False))
        a *= (wave[None, :, None] if vertical else wave[:, None, None])
        h = np.sin(np.linspace(0, 2 * np.pi * k, n, endpoint=False))
        h = np.tile(h[None, :], (n, 1)) if vertical else np.tile(h[:, None], (1, n))
        return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)), h

    for name, c, rough, metal in (("dorm_roof_greymetal", (150, 152, 152), 0.5, 0.5),
                                  ("dorm_roof_white", (226, 228, 226), 0.5, 0.3),
                                  ("dorm_roof_metal", (165, 168, 168), 0.45, 0.5),
                                  ("dorm_roof_brown", (150, 128, 105), 0.6, 0.3)):
        im, h = corrugated(c)
        save(name, im, rough, metal, 1.5, h)
    # the description's colours (GE textures), a shade darker: renders lift them / GE の色をやや暗く
    for name, c in (("dorm_roof_greygreen", (118, 126, 120)), ("dorm_roof_teal", (96, 146, 154)),
                    ("dorm_roof_green", (110, 148, 130)), ("dorm_roof_mint", (124, 178, 160)),
                    ("dorm_roof_palegrey", (160, 159, 168)), ("dorm_roof_grey", (132, 134, 132))):
        # waterproof coating with seams and stains / 防水塗装（継ぎ目と汚れ）
        a = np.asarray(tinted(np.array(c) * 0.88, np.array(c) * 1.04, 512, (4, 16, 64)), np.float64)
        for k in range(0, 512, 128):
            a[k:k + 2, :] *= 0.92
            a[:, k:k + 2] *= 0.95
        save(name, Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)), 0.8, 0.0, 0.5)
    # kitchen roof: grey sheet with translucent skylight strips / 採光帯付きの屋根
    im, h = corrugated((160, 162, 165), n=256, k=24)
    a = np.asarray(im, np.float64)
    for x0 in range(0, 256, 64):
        a[:, x0 + 20:x0 + 34] = a[:, x0 + 20:x0 + 34] * 0.4 + np.array([215, 225, 228]) * 0.6
    save("dorm_roof_toplit", Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)), 0.4, 0.2, 1.0, h)

    def sash(panes, n=256, transom=True, curtain=0.5):
        """Aluminium sash: silver frames, bluish glass with sky reflection,
        curtains in some panes. / アルミサッシ。"""
        im = Image.new("RGB", (n, n), (186, 189, 191))
        d = ImageDraw.Draw(im)
        fw = max(3, n // 40)
        ty = int(n * 0.28) if transom else fw
        g0, g1 = np.array([66, 88, 104]), np.array([150, 172, 186])
        for i in range(panes):
            x0, x1 = int(i * n / panes) + fw, int((i + 1) * n / panes) - fw
            spans = ((fw, ty - fw // 2), (ty + fw // 2, n - fw)) if transom else ((fw, n - fw),)
            for y0, y1 in spans:
                for yy in range(y0, y1):
                    t = (yy - y0) / max(1, y1 - y0)
                    col = tuple(int(v) for v in g1 * (1 - t) + g0 * t)
                    d.line([(x0, yy), (x1, yy)], fill=col)
            if rs.random() < curtain:
                cx0 = x0 + int((x1 - x0) * rs.uniform(0.0, 0.4))
                col = [(228, 224, 206), (210, 220, 228), (232, 214, 200)][int(rs.integers(0, 3))]
                d.rectangle([cx0, ty + fw, cx0 + int((x1 - x0) * 0.4), n - fw - 2], fill=col)
        return im

    save("dorm_sash", sash(1), 0.15, 0.6)
    save("dorm_sash_2", sash(2), 0.15, 0.6)
    save("dorm_sash_3", sash(3), 0.15, 0.6)
    save("dorm_glassdoor", sash(2, transom=True, curtain=0.0), 0.12, 0.6)
    save("dorm_balcony_door", sash(2, transom=False, curtain=0.7), 0.15, 0.6)
    fr_ = sash(2, curtain=0.0)
    a = np.asarray(fr_, np.float64)
    a = a * 0.3 + np.array([205, 208, 205]) * 0.7                    # frosted / すりガラス
    save("dorm_frosted", Image.fromarray(a.astype(np.uint8)), 0.3, 0.3)
    save("dorm_louvre", X.stripes(256, (120, 124, 124), (200, 203, 200), 16), 0.5, 0.5, 0.8)
    lb = X.stripes(256, (92, 66, 48), (128, 96, 70), 12)
    save("dorm_louvre_brown", lb, 0.5, 0.4, 0.8)
    save("dorm_steel", tinted((150, 152, 155), (175, 178, 180), 128), 0.4, 0.7)
    save("dorm_steel_white", tinted((228, 228, 226), (244, 244, 242), 128), 0.4, 0.3)
    save("dorm_steel_brown", tinted((90, 64, 46), (112, 82, 60), 128), 0.45, 0.5)
    save("dorm_mesh_white", tinted((232, 232, 230), (246, 246, 244), 64, (2, 8, 32)), 0.5, 0.3)
    pc = tinted((214, 222, 214), (232, 238, 232), 256)
    d = ImageDraw.Draw(pc)
    for x0 in range(0, 256, 32):
        d.line([(x0, 0), (x0, 255)], fill=(150, 156, 150), width=2)
    save("dorm_polycarb", pc, 0.25, 0.0, 0.3)
    save("dorm_asphalt", tinted((88, 88, 90), (116, 116, 118), 512, (16, 64, 256)), 0.88, 0.0, 0.6)
    save("dorm_paving", tiles((188, 176, 158), n=512, k=8, kv=16, grout=(150, 140, 125)), 0.85, 0.0, 1.0)
    save("dorm_postbox", tinted((190, 28, 26), (210, 40, 36), 64, (2, 8, 32)), 0.4, 0.2)
    # the name plate on the planter wall / 植栽桝の銘板
    W, H = 1024, 228
    im = Image.new("RGB", (W, H), (72, 74, 74))
    d = ImageDraw.Draw(im)
    text = "大分工業高等専門学校 明野寮"
    try:
        size = 110
        f = ImageFont.truetype(CJK, size)
        while d.textlength(text, font=f) > W * 0.92:
            size -= 4
            f = ImageFont.truetype(CJK, size)
        tw = d.textlength(text, font=f)
        d.text(((W - tw) / 2, (H - size * 1.25) / 2), text, fill=(238, 232, 214), font=f)
    except OSError:
        pass
    save("dorm_plate", im, 0.3, 0.0)
    _ = text_poly, oquad
    return out
