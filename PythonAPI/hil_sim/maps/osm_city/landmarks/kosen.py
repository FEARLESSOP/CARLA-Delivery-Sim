"""Oita Kosen campus (大分工業高等専門学校, NIT Oita College), Akeno, Oita:
modelled in detail (maps/.venv).

Sources (2026): the z20 satellite tiles, Google Earth 3D orbits and the
written description, Street View at the gate and along the main road, the
College Bulletin 2026 campus map and building table (Photo Getter,
build/OitaKosen/kosen_photos). Outlines traced on the z20 tiles
(build/OitaKosen/kosen_trace.json); storeys from the official table; forms,
heights and colours from the Google Earth description. Nothing is copied
as texture: every texture is drawn here.

  admin 6/7    the E-plan: west spine, wings 6 and 7 (7 with the solar array
               and the entrance porch facing the forecourt), the general
               education blocks north and west, the red hipped annex, the
               2-storey glazed entrance hall; cream render, ribbon windows
               between projecting columns, open corridors to the courtyard,
               mint roofs with penthouses and AC units
  24 / 25      the grey 4-storey tower with the louvred stair strip and the
               white rooftop screen carrying blue 「大分高専」 facing east; the
               orange-brown tiled 4-storey blocks with paired windows
  labs 1-3     3-storey cream bars, open corridors (solid beige parapets
               between columns), the 3-storey open corridor (渡り廊下), the
               semicircular stair tower, the east stair core of 2
  4, 5, 9      single-storey RC blocks by the road (table RC1), 9 a shed
  13-15        the single-storey workshops, the rust sawtooth roof
  8, 12        the library (RC4, 4 x 3.3 m), the student centre
  16, 10, 11   the STK hall (pale corrugated gable, high window bands, teal
               annex), the training camp hall and lean-to, the staff hall
               with its grey tiled hip roof, the club rooms
  17           gymnasium 2: light-blue gable, five ridge ventilators,
               clerestory band
  grounds      athletic ground 21 on its own terrace (sand, the dark baseball
               infield, the blue running straight, the tall ball-stop net),
               the tennis terrace 18 (six clay courts, chain-link), the pool
               19 with its deck, the martial arts hall 20; the forecourt
               parterre and the monument (on the road network's islands)
  gate         tan masonry wing walls, the grey stone pier with the name
               plate, a sliding gate leaf; white tubular fence on a low wall
               along the main road, chain-link elsewhere; the guard house,
               bicycle shelters, stores, garages; trees where the imagery
               shows canopy
  terrain      each building complex on its own level platform (landmarks/
               onterrain.py), courts and the field flat on theirs, retaining
               walls and planted banks where they step

Roads win: every footprint is cut 0.4 m clear of the road network, its
kerbs, the mapped footways and drives; facade projections shrink per edge
to stay 0.2 m clear; everything else (fences, posts, nets, trees) is kept
off them too.

Coordinates: the map frame (x east, y north, metres).

大分工業高等専門学校の詳細モデル。
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
from landmarks.lvss import mapped_flat, oquad, text_poly
from mesh import Mesh, polys
import jtrees

NAME = "大分工業高等専門学校"
TAG = "KOSEN"
TRACE = "kosen_trace.json"
SITE_WAY = "113350012"
CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
CLEAR_FOOT = 0.4         # footprint to roads, kerbs, paths / 建物と道路・歩道の間隔
ROOM = 0.2               # facade projections stay this far off them / 張り出しの余裕
TREE_CLEAR = 0.3         # tree crowns to roads and paths / 樹冠と道路の間隔
APRON = 1.2              # paved platform margin round a building / 建物周りの舗装
GROUP_DROP = 2.0         # joined blocks share a platform unless their ground differs more / 一体の造成の上限

# roof material per trace "roof" / 屋根の材質
ROOFS = {"mint": "kosen_roof_mint", "teal": "kosen_roof_teal", "dark": "kosen_roof_dark",
         "library": "kosen_roof_library", "greygreen": "kosen_roof_greygreen", "blue": "kosen_roof_blue",
         "white": "kosen_roof_white", "green": "kosen_roof_green", "brown": "kosen_roof_rust",
         "greymetal": "kosen_roof_greymetal", None: "kosen_roof_grey"}


ADMIN_PINK = ("adm_north", "adm_spine", "wing6", "wing7", "adm_west")   # salmon-beige render / サーモンベージュ
RECT_STYLES = ("gym", "budo", "bikeshed", "sawtooth", "annex_red", "canopy", "guard", "shed")


def fit_rect(poly, step=0.1):
    """The footprint's oriented rectangle, its sides moved in until it lies inside
    the (cut back) footprint: pitched roofs and posts are built on it.
    外接矩形を外形内に収まるまで縮める（屋根・柱はこの矩形に建てる）。"""
    r = orient(poly.minimum_rotated_rectangle, 1.0)
    c = [np.array(q) for q in list(r.exterior.coords)[:4]]
    ok = poly.buffer(0.02)
    for _ in range(200):
        rect = Polygon([tuple(q) for q in c])
        out = rect.difference(ok).area
        if out < 1e-4:
            return orient(rect, 1.0)
        best = None
        for i in range(4):
            j = (i + 1) % 4
            nrm = c[(i + 3) % 4] - c[i]
            nrm = nrm / max(np.linalg.norm(nrm), 1e-9)
            t = [q.copy() for q in c]
            t[i] = t[i] + nrm * step
            t[j] = t[j] + nrm * step
            tp = Polygon([tuple(q) for q in t])
            # the side that clears most outside area per area given up (a long
            # side moved in loses far more roof than a slanted end needs)
            # 失う面積あたり最も外側を減らす辺を動かす
            o = -(out - tp.difference(ok).area) / max(rect.area - tp.area, 1e-9)
            if best is None or o < best[0]:
                best = (o, t)
        c = best[1]
    return poly


def grid_wall(f, mat, z0, z1, openings, uvs=1.0):
    """The wall patch z0..z1 with any number of rectangular openings (rows,
    columns, overlapping or not), drawn once: the patch is cut on the grid of
    every opening edge and each run of solid cells in a row is one quad (no
    doubled faces; Facade.wall_with_openings handles a single row only).
    開口の格子で壁を分割（重なりなし、複数段の窓に対応）。"""
    ops = [(max(0.0, a), min(f.L, b), max(z0, c), min(z1, d)) for a, b, c, d in openings]
    ops = [o for o in ops if o[1] - o[0] > 1e-3 and o[3] - o[2] > 1e-3]
    us = sorted({0.0, f.L} | {o[0] for o in ops} | {o[1] for o in ops})
    zs = sorted({z0, z1} | {o[2] for o in ops} | {o[3] for o in ops})
    for za, zb in zip(zs, zs[1:]):
        if zb - za < 1e-4:
            continue
        zm = 0.5 * (za + zb)
        run = None
        for ua, ub in zip(us, us[1:]):
            um = 0.5 * (ua + ub)
            hole = any(o[0] <= um <= o[1] and o[2] <= zm <= o[3] for o in ops)
            if not hole and run is None:
                run = ua
            if hole and run is not None:
                f.face(mat, run, ua, za, zb, 0.0, uvs)
                run = None
        if run is not None:
            f.face(mat, run, f.L, za, zb, 0.0, uvs)


def band_window(f, u0, u1, z0, z1, wall, glass="kosen_sash_2", pane=3.0, depth=0.12):
    """A continuous window band: one recess with reveals and a sill, the glass in
    panes of about ``pane`` m (the frames are in the glass texture). The wall
    around it must leave the whole band open. / 連続した高窓帯。"""
    f.hplane(wall, u0, u1, -depth, 0.0, z0, True)
    f.hplane(wall, u0, u1, -depth, 0.0, z1, False)
    f.side(wall, u0, -depth, 0.0, z0, z1, 1)
    f.side(wall, u1, -depth, 0.0, z0, z1, -1)
    n = max(1, int(round((u1 - u0) / pane)))
    w = (u1 - u0) / n
    for i in range(n):
        a = u0 + i * w
        f.m.quad(glass, f.P(a, -depth, z0), f.P(a + w, -depth, z0), f.P(a + w, -depth, z1), f.P(a, -depth, z1),
                 (0, 0), (1, 0), (1, 1), (0, 1))
    f.hplane("concrete", u0 - 0.05, u1 + 0.05, 0.0, 0.08, z0, True)
    f.face("concrete", u0 - 0.05, u1 + 0.05, z0 - 0.06, z0, 0.08)


def back(a, b, d):
    """The reverse of segment a-b moved d metres to its new right (behind the
    front face): a two-sided strip's back never shares the front's plane (twin
    faces in one plane render black in Cycles). / 裏面を少しずらす。"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = max(np.hypot(*(b - a)), 1e-9)
    n = np.array([-(b - a)[1], (b - a)[0]]) / L        # left of a->b = right of b->a / 裏面の外向き
    return tuple(b + n * d), tuple(a + n * d)


def edges(poly):
    return B.edges_of(orient(poly, 1.0))


def ring_line(poly):
    return LineString(list(orient(poly, 1.0).exterior.coords))


def prism_o(m, mat, poly, z0, z1, top=True, wall=None, uv=2.0):
    """Walls out and a top facing up over any polygon. / 外向きの側面と上面。"""
    m.prism(wall or mat, poly, z0, z1, top=False, uv_scale=uv)
    if top:
        m.flat(mat, poly, z1, uv)


class KOSEN(object):
    def __init__(self, L, out_dir):
        self.L = L
        self.meta = L.meta
        self.out_dir = out_dir
        self.fr = geo.Frame(self.meta)
        T = json.load(open(os.path.join(out_dir, TRACE)))
        self.T = T
        self.drape = None                         # set by build.py (terrain.Drape) / build.py が設定
        self.site = next(p for p, t, w in L.osm.areas_id(lambda t: "amenity" in t) if w == SITE_WAY)
        # the road network and the paths before build.py clears the site
        # 道路網と歩道（build.py が敷地内を消す前の写し）
        near = self.site.buffer(40.0)
        self.road = L.road_all.intersection(near)
        self.carriageway = L.carriageway.intersection(near)
        self.paths = unary_union([L.footpaths, L.cycleways, L.driveways]).intersection(near)
        self.islands = L.islands.intersection(self.site.buffer(5.0))
        self.no_go = unary_union([self.road, self.paths])
        shapely.prepare(self.no_go)
        lean = T["lean"]
        self.lean = (lean["dx_per_m"], lean["dy_per_m"])
        # blocks: roof outline as seen, moved back by the lean, cut clear of the
        # roads and paths (roads win) / 屋根の輪郭を傾き分戻し、道路・歩道から離す
        keep = self.no_go.buffer(CLEAR_FOOT)
        self.blocks, self.cut = {}, {}
        for k, b in T["blocks"].items():
            if b["style"] == "stair_round":
                c = self.P(b["tile"], *b["centre"])
                half = Point(c).buffer(b["r_m"], 24).intersection(box(c[0] - 50, c[1] - 50, c[0], c[1] + 50))
                roof = unary_union([half, Polygon([self.P(b["tile"], *q) for q in b["box"]])])
            else:
                roof = Polygon([self.P(b["tile"], *q) for q in b["pts"]])
            roof = orient(roof.buffer(0), 1.0)
            foot = translate(roof, -self.lean[0] * b["h"], -self.lean[1] * b["h"])
            q = foot.difference(keep)
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
                q = fit_rect(q)                   # roofs built on a rectangle stay on the footprint / 矩形に収める
                rr = q.minimum_rotated_rectangle.exterior.coords
                if min(math.dist(rr[0], rr[1]), math.dist(rr[1], rr[2])) < 2.0:
                    print("landmark %s: %s dropped (what the roads leave is under 2 m wide)" % (TAG, k))
                    self.cut[k] = foot.area
                    continue
            d = dict(b)
            d.update(poly=q, roof_seen=roof, name=k)
            self.blocks[k] = d
        self.footprint = unary_union([b["poly"] for b in self.blocks.values()])
        shapely.prepare(self.footprint)
        # ground shapes: as seen / 地上の形は見たまま
        G = T["grounds"]
        gp = lambda d: Polygon([self.P(d["tile"], *q) for q in d["pts"]]).buffer(0)
        self.field = gp(G["field21"])
        self.infield = Point(self.P(G["infield"]["tile"], *G["infield"]["pts"][0])).buffer(G["infield"]["r_m"], 48)
        t = G["track"]
        self.track_line = LineString([self.P(t["tile"], *q) for q in t["pts"]])
        self.track_w = t["w_m"]
        self.track2 = (self.track_line.parallel_offset(t["second_offset"], "right"), t["second_w"])
        self.tennis = gp(G["tennis18"])
        self.pool = gp(G["pool19"])
        self.pool_deck = gp(G["pool_deck"])
        self.parterre = gp(G["parterre"])
        self.monument = self.P(G["monument"]["tile"], *G["monument"]["pts"][0])
        self.courtyard = gp(G["courtyard67"])
        self.gas_yard = gp(G["gas_yard"])
        n = G["baseball_net"]
        self.net_line = LineString([self.P(n["tile"], *q) for q in n["pts"]])
        self.net_h = n["h_m"]
        cp = G["carpark_se"]
        self.carpark = gp(cp)
        self.carpark_rows = cp["bays"]
        self.yard = gp(G["yard_sw"]) if "yard_sw" in G else Polygon()
        ls = G["lot_sw"]
        self.lot_line = LineString([self.P(ls["tile"], *q) for q in ls["pts"]])
        self.lot = self.lot_line.buffer(ls["w_m"] / 2, cap_style="flat")
        gh = G["greenhouse"]
        self.greenhouse = (self.P(gh["tile"], *gh["pts"][0]), gh["len_m"], gh["w_m"], math.radians(gh["axis_deg"]))
        self.farms = [gp(G[k]) for k in ("farm1", "farm2")]
        self.woods = unary_union([Polygon([self.P(w["tile"], *q) for q in w["pts"]]).buffer(0) for w in T.get("woods", [])])
        self.trees_seen = [(self.P(t, x, y), r) for t, x, y, r in T["trees"]]
        self.centre = self.site.centroid
        self._gate()
        self._built = None

    def P(self, tile, px, py):
        la, lo = self.T["tiles"][tile]
        cx, cy = self.fr.xy(lo, la)
        return (cx + (px - self.T["centre"][0]) * self.T["mpp_x"], cy - (py - self.T["centre"][1]) * self.T["mpp_y"])

    def _gate(self):
        """The main gate: where the gate road (OSM way 181723924, road 4349 at the
        大分高専前 lights) crosses the site's east boundary. / 正門の位置。"""
        ring = self.site.exterior
        hit = ring.intersection(self.road.buffer(0.3))
        best = None
        for g in polys(hit.buffer(0.01)) if not hit.is_empty else []:
            c = g.centroid
            d = math.hypot(c.x + 8.0, c.y + 157.0)
            if best is None or d < best[0]:
                best = (d, g)
        if best is not None and best[0] < 20:
            g = best[1]
            x0, y0, x1, y1 = g.bounds
            self.gate_span = ((0.5 * (x0 + x1), y1), (0.5 * (x0 + x1), y0))       # north, south ends / 北端・南端
        else:
            self.gate_span = ((-7.5, -150.5), (-7.5, -163.5))
        self.gate = ((self.gate_span[0][0] + self.gate_span[1][0]) / 2, (self.gate_span[0][1] + self.gate_span[1][1]) / 2)

    # -- what build.py needs / build.py 用
    def inner(self):
        return self.site.buffer(-0.6, join_style="mitre")

    def region(self):
        """The ground this module draws: the site inside its 0.6 m edge, without the
        road network and its islands (ground.py draws those). / 本モジュールが描く地面。"""
        if getattr(self, "_region", None) is None:
            self._region = onterrain._clean(self.inner().difference(self.L.road_all.buffer(0.01)).difference(
                self.L.islands.buffer(0.01)))
        return self._region

    def ground_taken(self):
        """Left out of the map's draped grass base (build.py adds it to L.terraces). / 汎用の芝を張らない範囲。"""
        return self.region()

    def keepout(self):
        """No generic trees anywhere on the campus: this module plants its own. / 汎用の木は置かない。"""
        return self.site.buffer(1.0)

    def ground_mesh(self, tile, z):
        self.build_all()
        return self._built["ground"].get(tile.bounds, Mesh())

    def buildings_mesh(self, tile):
        self.build_all()
        return self._built["bld"].get(tile.bounds, Mesh())

    def extra_meshes(self, tile):
        """Objects of their own: each tree species' leaves (the season swap finds
        them by name, the import step gives them the foliage material).
        独立オブジェクト: 樹種ごとの葉。"""
        self.build_all()
        return {jtrees.KIND[sp]: d.get(tile.bounds, Mesh()) for sp, d in self._built["leaves"].items()}

    # -- terrain / 地形
    def plan(self):
        """Platforms: each building complex (blocks touching each other) on one
        level at its highest ground, the field, tennis terrace and pool deck on
        theirs. Blocks standing on the road network's islands (10) keep the
        island's grass and get a plinth instead. / 造成計画。"""
        if getattr(self, "S", None) is not None:
            return self.S
        S = onterrain.Site(self.drape, self.region(), no_go=unary_union([self.no_go, self.L.islands]), gap=0.3)
        self.S = S
        self.zb = {}
        on_island = {k for k, b in self.blocks.items() if b["poly"].intersection(self.L.islands).area > 0.3 * b["poly"].area}
        rest = [k for k in self.blocks if k not in on_island]
        # complexes: blocks within 0.6 m of each other / 0.6 m 以内で接する棟は一体
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
            if len(grp) > 1 and S.drop(u) > GROUP_DROP:
                subs = [[k] for k in grp]
            else:
                subs = [grp]
            for sg in subs:
                u = unary_union([self.blocks[k]["poly"] for k in sg])
                p = S.platform("+".join(sg), u, how="max", apron=APRON)
                z = p.z if p is not None else S.level(u)
                for k in sg:
                    self.zb[k] = z
                self.groups.append((sg, z, S.drop(u)))
        for k in on_island:
            self.zb[k] = S.level(self.blocks[k]["poly"], "max")
        # courts and the field / コートとグラウンド
        self.p_field = S.platform("field21", self.field.difference(self.footprint.buffer(1.0)), how="median",
                                  kind="court", bank=True)
        self.p_tennis = S.platform("tennis18", self.tennis, how="median", kind="court")
        self.p_pool = S.platform("pool19", self.pool_deck, how="max", kind="court")
        return S

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
            if b["poly"].intersection(self.L.islands).area > 0.3 * b["poly"].area:
                self.plinth(bm, b["poly"], self.zb[k])
        self.leaves, self.species = jtrees.leafsets(), {}
        self.site_things(bm)
        tiles = [tb for tn, tb in G.tiles(self.meta)]
        self._built = {"ground": split_tiles(gm, tiles), "bld": split_tiles(bm, tiles),
                       "leaves": {sp: split_tiles(lm, tiles) for sp, lm in self.leaves.items()}}
        n = sum(len(i) for p in gm.parts.values() for i in p[3]), sum(len(i) for p in bm.parts.values() for i in p[3])
        print("landmark %s: terrain %s; ground %d, buildings %d triangles" % (TAG, S.report(), n[0], n[1]))
        for sg, z, dr in self.groups:
            if len(sg) > 1 or dr > 0.8:
                print("  platform %s: z %.2f, ground under it varies %.1f m" % ("+".join(sg)[:60], z, dr))

    def plinth(self, m, poly, z):
        """A concrete plinth from under the lowest ground to the floor (blocks on
        the road network's islands). / 島の上の建物の基壇。"""
        lo = self.S.level(poly, "min") - 0.3
        ring = poly.buffer(0.08, join_style="mitre")
        m.prism("kosen_plinth", ring, lo, z + 0.02, top=False)

    # -- ground / 地面
    def ground_surfaces(self, m):
        S = self.S
        reg = self.region()
        courts = unary_union([p.poly for p in (self.p_field, self.p_tennis, self.p_pool) if p is not None])
        bplat = unary_union([p.poly for p in S.items if p.kind == "building"])
        foot = self.footprint
        # paved: the building platforms (aprons) and the yards between the blocks;
        # planted: courtyards, the garden by the library, banks, the wooded edges
        # 舗装: 建物の造成面と棟の間の構内、緑地: 中庭・図書館脇の庭・法面・林
        dev = foot.buffer(9.0, join_style="mitre").buffer(-4.0, join_style="mitre")
        gardens = unary_union([self.courtyard.difference(foot.buffer(0.6)),
                               box(-170.0, -117.0, -150.0, -100.5), box(-150.0, -117.0, -124.0, -102.0)])
        # the mapped footways and drives the blocks are cut for: drawn as paths
        # (Agent Checker: 593 m2 of them were lawn) / 建物を切った歩道は舗装で描く
        paths = onterrain._clean(self.paths.intersection(reg).difference(foot), 0.05)
        # car parks: south of 3 and the SW lot "P" / 駐車場
        lots = onterrain._clean(unary_union([self.carpark, self.lot, self.yard]).intersection(reg).difference(foot)
                                .difference(paths).difference(courts), 0.5)
        farm = onterrain._clean(unary_union(self.farms).intersection(reg).difference(foot).difference(paths)
                                .difference(lots), 0.5)
        yard = onterrain._clean(dev.intersection(reg).difference(gardens).difference(courts).difference(paths)
                                .difference(lots).difference(farm))
        apron = onterrain._clean(bplat.difference(foot).difference(paths).difference(lots))
        S.ground(m, "concrete_path", paths, uv_scale=2.0)
        S.ground(m, "asphalt_lot", lots, uv_scale=5.0)
        S.ground(m, "soil", farm, uv_scale=3.0)
        S.ground(m, "kosen_paving", apron, uv_scale=2.0)
        S.ground(m, "kosen_asphalt", yard.difference(bplat), uv_scale=5.0)
        S.ground(m, "grass", reg.difference(yard).difference(bplat).difference(courts).difference(paths)
                 .difference(lots).difference(farm), uv_scale=4.0)
        self.bays(m, lots)
        # the field: sand, the dark infield, the blue running straight / グラウンド
        if self.p_field is not None:
            f = self.p_field.poly
            inf = self.infield.intersection(f)
            tr = unary_union([self.track_line.buffer(self.track_w / 2, cap_style="flat"),
                              self.track2[0].buffer(self.track2[1] / 2, cap_style="flat")]).intersection(f.buffer(-0.5))
            S.ground(m, "kosen_sand", f.difference(inf).difference(tr), uv_scale=6.0)
            S.ground(m, "kosen_infield", inf.difference(tr), uv_scale=5.0)
            S.ground(m, "kosen_track", tr, uv_scale=1.0)
        # tennis: six clay courts on the terrace / テニスコート 6 面
        if self.p_tennis is not None:
            t = self.p_tennis.poly
            courts6 = self.tennis_courts()
            S.ground(m, "kosen_clay", t.difference(unary_union([c for c, r in courts6])), uv_scale=5.0)
            for c, rect in courts6:
                mapped_flat(m, "kosen_tennis", c.intersection(t), rect, self.p_tennis.z + 0.0)
        # pool: deck, water 0.35 m down, tiled walls / プール
        if self.p_pool is not None:
            d = self.p_pool.poly
            water = self.pool.intersection(d.buffer(-0.3))
            S.ground(m, "kosen_deck", d.difference(water), uv_scale=3.0)
            zw = self.p_pool.z - 0.35
            m.flat("kosen_pool_water", water, zw, 4.0)
            for p in polys(water):
                m.ring_walls("pool_tile", p.exterior, zw - 0.2, self.p_pool.z, outward=False, uv_scale=1.0)
            # lane ropes / コースロープ
            r = water.minimum_rotated_rectangle
            cc = list(orient(r, 1.0).exterior.coords)[:4]
            if math.dist(cc[1], cc[2]) > math.dist(cc[0], cc[1]):
                cc = cc[1:] + cc[:1]
            a, b, c2, dd = [np.array(q) for q in cc]
            for k in range(1, 7):
                p0 = a + (dd - a) * k / 7
                p1 = b + (c2 - b) * k / 7
                m.ribbon("kosen_rope", [tuple(p0), tuple(p1)], 0.08, zw + 0.02)

    def bays(self, m, lots):
        """Painted bay lines (white, 10 cm) in the car parks: two rows south of 3,
        one row along the SW lot; only where the lines lie on the lot. / 駐車枠。"""
        S = self.S
        lines = []
        x0, y0, x1, y1 = self.carpark.bounds
        for ya, yb in self.carpark_rows:
            x = x0 + 1.0
            while x <= x1 - 1.0:
                lines.append(LineString([(x, ya), (x, yb)]))
                x += 2.5
        Ln = self.lot_line.length
        d = np.subtract(self.lot_line.coords[-1], self.lot_line.coords[0]) / Ln
        nrm = np.array([-d[1], d[0]])
        u = 1.5
        while u <= Ln - 1.5:
            p = np.add(self.lot_line.coords[0], d * u) + nrm * 0.3
            lines.append(LineString([tuple(p), tuple(p + nrm * 5.0)]))
            u += 2.5
        g = unary_union([l.buffer(0.05, cap_style="flat") for l in lines if not l.is_empty]).intersection(
            lots.buffer(-0.2))
        S.ground(m, "paint_white", onterrain._clean(g, 0.01), dz=0.015, uv_scale=1.0)

    def tennis_courts(self):
        """Six doubles courts (23.77 x 10.97 m, long axis north-south) in a row on
        the terrace; returns (court polygon with run-off, texture rectangle).
        テニスコート 6 面（南北方向）。"""
        x0, y0, x1, y1 = self.tennis.bounds
        n = self.T["grounds"]["tennis18"].get("courts", 6)
        w = (x1 - x0 - 2.0) / n
        out = []
        cy = (y0 + y1) / 2
        for i in range(n):
            cx = x0 + 1.0 + w * (i + 0.5)
            hw, hl = min(w / 2 - 0.3, 8.5), min((y1 - y0) / 2 - 0.5, 17.5)
            rect = [(cx - hw, cy - hl), (cx + hw, cy - hl), (cx + hw, cy + hl), (cx - hw, cy + hl)]
            out.append((Polygon(rect), rect))
        return out

    # -- tile split / タイル分け
    def note(self):
        return "%d blocks" % len(self.blocks)

    # ================= blocks / 棟 =================
    def room(self, a, b, n):
        """How far a facade on edge a-b may project before it nears a road or path. / 張り出し可能量。"""
        seg = LineString([tuple(a), tuple(b)])
        band = seg.buffer(1.5, cap_style="flat", single_sided=True) if False else None
        d = self.no_go.distance(seg)
        _ = band, n
        return max(0.0, d - ROOM)

    def covered(self, b, a, bb, n):
        """Parts of an edge hidden by a neighbouring block: [(u0, u1, height of
        the neighbour)], the rest free. / 隣の棟に隠れる区間。"""
        L = float(np.linalg.norm(bb - a))
        t = (bb - a) / L
        out = []
        for k, o in self.blocks.items():
            if o is b:
                continue
            # only a neighbour that really touches hides the edge (a 0.3 m test left
            # see-through slots where two blocks stood a little apart)
            # 実際に接する隣棟だけが壁を隠す（離れた棟の間に隙間が透けた）
            band = o["poly"].buffer(0.06)
            seg = LineString([tuple(a + n * 0.04), tuple(bb + n * 0.04)])
            if not band.intersects(seg):
                continue
            ins = seg.intersection(band)
            for g in getattr(ins, "geoms", [ins]):
                if g.geom_type != "LineString" or g.length < 0.3:
                    continue
                us = [float(np.dot(np.array(q) - (a + n * 0.04), t)) for q in g.coords]
                hz = self.block_top(o) + self.zb.get(k, 0.0) - self.zb.get(b["name"], 0.0)
                out.append((max(0.0, min(us)), min(L, max(us)), hz))
        return out

    def block_top(self, b):
        st = b["style"]
        if st in ("gym",):
            return b.get("eave", 9.0)
        if st in ("rc", "tile", "grey", "library"):
            return b["storeys"] * b.get("floor", self.T["floor_m"])
        return b["h"]                    # low blocks: their real height / 低い棟は実際の高さ

    def facade_pieces(self, b):
        """Every edge of a block, split where neighbours cover it: (a, b, L, n,
        z_from) -- draw the facade above z_from only. / 外壁の区間と描き始め高さ。"""
        out = []
        for a, bb, L, n in edges(b["poly"]):
            cov = sorted(self.covered(b, a, bb, n))
            cuts = [(0.0, L, 0.0)]
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

    # ---- RC school blocks / RC 校舎
    def b_rc(self, m, b):
        st = b["storeys"]
        fl = b.get("floor", self.T["floor_m"])
        top = st * fl
        side = {"S": (0, -1), "N": (0, 1)}.get(b.get("corridor"))
        # the admin complex is salmon-beige with white columns and eaves (user's
        # photo); the rest cream / 管理棟群はサーモンベージュに白い柱と軒
        pink = b["name"] in ADMIN_PINK
        wall = "kosen_render_pink" if pink else "kosen_render"
        col = "kosen_white" if pink else None
        for a, bb, L, n, zf, (u_off, Lfull) in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            po = min(0.6, self.room(a, bb, n))
            corridor = side is not None and float(n @ np.array(side)) > 0.7 and Lfull > 12
            end = Lfull < 14.0
            south = n[1] < -0.7
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                if end:
                    self.rc_end(f, L, z0, z1, k, wall=wall)
                elif corridor:
                    self.rc_corridor(f, L, z0, z1, k, po, u_off, wall=wall)
                else:
                    self.rc_windows(f, L, z0, z1, k, po, south, u_off, wall=wall, col=col)
            if top > zf:
                f.face(wall, 0.0, L, max(zf, top - 0.001), top, 0.0)
            if pink and po > 0.15 and top > zf and not end:
                # the white eave slab round the roof (photo) / 屋上の白い軒
                f.block("kosen_white", 0.0, L, 0.0, min(0.6, po), top - 0.05, top + 0.35)
        self.flat_roof(m, b, top, wall=wall)
        if b.get("porch"):
            self.porch(m, b)

    def rc_windows(self, f, L, z0, z1, k, po, south, u_off=0.0, wall="kosen_render", glass="kosen_sash", col=None):
        """Ribbon windows between projecting columns, spandrel bands, a slab
        sunshade on south faces. / 柱の間の連窓・腰壁・南面の庇。"""
        bays = B.bays(L, 3.8, 0.35)
        ops = [(u0 + 0.28, u1 - 0.28, z0 + 0.95, z0 + 2.95) for u0, u1 in bays]
        grid_wall(f, wall, z0, z1, ops, 3.0)
        for o in ops:
            f.window(*o, wall, glass=glass, mullions=2, depth=0.16)
        f.block("kosen_spandrel", 0.0, L, 0.0, 0.03, z0 + 0.12, z0 + 0.88, top=False, bottom=False, ends=False)
        cp = min(0.35, po)
        if cp > 0.05:
            for u in [0.0] + [u1 for u0, u1 in bays[:-1]] + [L]:
                u0, u1 = max(0.0, u - 0.25), min(L, u + 0.25)
                f.block(col or wall, u0, u1, 0.0, cp, z0, z1, top=False, bottom=False)
        if south and po > 0.3:
            f.block(wall, 0.3, L - 0.3, 0.0, min(0.6, po), z0 + 3.02, z0 + 3.14)

    def rc_corridor(self, f, L, z0, z1, k, po, u_off=0.0, depth=2.0, wall="kosen_render"):
        """Open corridor: solid beige parapet panels between columns, the class
        wall behind with doors and sash windows, slab soffit. / 開放廊下。"""
        e = 0.25                                       # end walls close the corridor / 端は妻壁で閉じる
        f.face(wall, 0.0, e, z0, z1, 0.0)
        f.face(wall, L - e, L, z0, z1, 0.0)
        f.side(wall, e, -depth, 0.0, z0, z1, 1)
        f.side(wall, L - e, -depth, 0.0, z0, z1, -1)
        f.face(wall, e, L - e, z1 - 0.32, z1, 0.0)                       # slab edge / スラブ端
        f.hplane("kosen_soffit", e, L - e, -depth, 0.0, z1 - 0.32, False)
        if k > 0:
            f.block("kosen_spandrel", e, L - e, -0.14, 0.0, z0, z0 + 1.1, top=False, bottom=False, ends=False)
            f.hplane("kosen_white", e, L - e, -0.14, 0.0, z0 + 1.1, True)
            # back of the parapet, facing the corridor / 腰壁の裏（廊下側）
            f.m.quad(wall, f.P(L - e, -0.14, z0), f.P(e, -0.14, z0), f.P(e, -0.14, z0 + 1.1), f.P(L - e, -0.14, z0 + 1.1))
            f.hplane("kosen_floor", e, L - e, -depth, -0.14, z0 + 0.02, True)
        else:
            f.hplane("kosen_floor", e, L - e, -depth, 0.0, z0 + 0.02, True)
        f.face(wall, e, L - e, z0, z1 - 0.32, -depth)
        for u0, u1 in B.bays(L, 7.6, 0.4):
            mid = (u0 + u1) / 2
            f.face("jp_door_steel", mid - 3.0, mid - 2.1, z0 + 0.02, z0 + 2.1, -depth + 0.02)
            f.face("kosen_sash_2", mid - 1.6, mid + 3.2, z0 + 1.0, z0 + 2.7, -depth + 0.02)
            f.face("kosen_transom", mid - 3.0, mid + 3.2, z0 + 2.75, z0 + 3.1, -depth + 0.02)
        cp = min(0.3, po)
        for u in np.linspace(0.5, L - 0.5, max(2, int(L // 3.8) + 1)):
            f.block(wall, u - 0.25, u + 0.25, -depth + 0.4 if k > 0 else -depth, cp, z0, z1 - 0.32,
                    top=False, bottom=False)

    def rc_end(self, f, L, z0, z1, k, wall="kosen_render"):
        """End wall: a stair window per floor, a drain pipe. / 妻壁: 階段の窓と雨樋。"""
        mid = L / 2
        if L > 3.0:
            o = (mid - 0.8, mid + 0.8, z0 + 1.2, z0 + 2.7)
            grid_wall(f, wall, z0, z1, [o], 3.0)
            f.window(*o, wall, glass="kosen_sash", mullions=1, depth=0.14)
        else:
            f.face(wall, 0.0, L, z0, z1, 0.0, 3.0)
        if L > 4.0:
            f.block("jp_pipe", 0.4, 0.5, 0.0, 0.1, z0, z1, top=False, bottom=False)

    def flat_roof(self, m, b, z, parapet=0.9, mat=None, things=True, wall="kosen_render"):
        """Flat roof inside a parapet with a white coping; stair penthouse, AC
        units, solar panels as described. / 陸屋根・パラペット・塔屋・室外機・太陽光。"""
        poly = b["poly"]
        mat = mat or ROOFS.get(b.get("roof"), "kosen_roof_grey")
        inner = poly.buffer(-0.22, join_style="mitre")
        m.flat(mat, inner, z + 0.02, 3.0)
        for p in polys(inner):
            m.ring_walls(wall, p.exterior, z, z + parapet, outward=False, uv_scale=3.0)
        m.ring_walls(wall, poly.exterior, z, z + parapet, outward=True, uv_scale=3.0)
        m.flat("kosen_white", poly.difference(inner), z + parapet, 1.0)
        if not things or inner.is_empty:
            return
        r = orient(inner.minimum_rotated_rectangle, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, bb, d = c[k], c[(k + 1) % 4], c[(k + 3) % 4]
        Lr, W = e[k], e[(k + 1) % 4]
        ax, sd = (bb - a) / Lr, (d - a) / W
        ang = math.atan2(ax[1], ax[0])
        P = lambda u, v: a + ax * u + sd * v
        h = sum(ord(ch) for ch in b["name"])
        # stair penthouse(s) at the ends of long blocks / 塔屋
        if Lr > 18 and W > 6 and b["storeys"] >= 2:
            for u in ((3.2,) if Lr < 40 else (3.2, Lr - 3.2)):
                q = P(u, W * 0.35)
                if inner.buffer(-0.3).contains(Point(q)):
                    m.box(wall, q[0], q[1], z, z + 2.9, 4.2, 3.4, ang)
                    m.box("kosen_white", q[0], q[1], z + 2.9, z + 3.05, 4.5, 3.7, ang)
                    # the door on the -sd face, wound to face out (-sd) / 塔屋の扉（外向き）
                    m.quad("jp_door_steel", *[(qq[0], qq[1], zz) for qq, zz in
                                              ((q - ax * 0.45 - sd * 1.74, z), (q + ax * 0.45 - sd * 1.74, z),
                                               (q + ax * 0.45 - sd * 1.74, z + 2.0),
                                               (q - ax * 0.45 - sd * 1.74, z + 2.0))])
        # AC units in a row / 室外機
        if Lr > 10 and b["storeys"] >= 2:
            n_ac = int(min(10, Lr // 6))
            for i in range(n_ac):
                u = Lr * (0.25 + 0.5 * ((i + 0.5) / n_ac))
                q = P(u, W * 0.8)
                if inner.buffer(-0.6).contains(Point(q)):
                    m.box("jp_ac_unit", q[0], q[1], z + 0.02, z + 0.95, 0.9, 0.4, ang + (h % 2) * math.pi)
        sol = b.get("solar")
        if sol == "array":
            # the long array on 7 (GE: almost its full length, ~5 m wide, on the
            # north half) / 7 の長い太陽光アレイ
            self.solar(m, inner, z + 0.25, P(2.0, W * 0.1), ax, sd, Lr - 4.0, min(5.2, W * 0.45))
        elif isinstance(sol, int):
            for i in range(sol):
                u = Lr * (i + 0.5) / sol
                self.solar(m, inner, z + 0.25, P(u - 2.5, W * 0.15), ax, sd, 5.0, 3.6)

    def solar(self, m, area, z, origin, ax, sd, Ln, Wd, tilt=10.0):
        """Rows of panels tilted to the south side on low steel frames. / 太陽光パネル。"""
        slope = math.tan(math.radians(tilt))
        v = 0.0
        while v + 1.6 <= Wd:
            q = [origin + ax * 0 + sd * v, origin + ax * Ln + sd * v, origin + ax * Ln + sd * (v + 1.6),
                 origin + sd * (v + 1.6)]
            strip = Polygon([tuple(p) for p in q]).intersection(area.buffer(-0.3))
            base = origin + sd * v
            zf = (lambda x, y, base=base: z + slope * np.abs((x - base[0]) * sd[0] + (y - base[1]) * sd[1]))
            for p in polys(strip):
                if p.area > 1.0:
                    m.flat("solar", p, z, 1.0, zf=zf)
            v += 1.75

    def porch(self, m, b):
        """The entrance porch on the forecourt face: a flat canopy on two columns,
        glazed doors, steps. / 正面玄関のポーチ。"""
        a, bb, L, n = max(edges(b["poly"]), key=lambda e: -e[3][1] + 0.001 * e[2])   # the south face / 南面
        f = B.Facade(m, a, bb, n)
        po = self.room(a, bb, n)
        mid = L * 0.5
        dep = min(3.0, po)
        f.face("kosen_curtain", mid - 2.4, mid + 2.4, 0.0, 2.6, 0.02, 6.0)
        if dep > 0.8:
            # dark grey-green flat canopy (photo) / 濃い灰緑の庇
            f.block("kosen_canopy_dark", mid - 3.4, mid + 3.4, 0.0, dep, 3.0, 3.3)
            for s in (-1, 1):
                f.block("kosen_white", mid + s * 3.0 - 0.18, mid + s * 3.0 + 0.18, dep - 0.45, dep - 0.09, 0.0, 3.0)
            f.block("kosen_paving", mid - 3.0, mid + 3.0, 0.0, min(dep, 2.0), 0.0, 0.15, ends=True)
            # steel handrails either side of the entrance / 玄関両脇の手すり
            for s in (-1, 1):
                u = mid + s * 3.25
                for w in np.linspace(0.3, min(dep, 2.0) - 0.1, 3):
                    f.block("kosen_steel", u - 0.025, u + 0.025, w - 0.025, w + 0.025, 0.15, 1.0)
                f.block("kosen_steel", u - 0.03, u + 0.03, 0.25, min(dep, 2.0) - 0.05, 0.95, 1.0)

    def b_rc_low(self, m, b):
        """Single- or two-storey RC block: cream, windows in bays, flat roof. / 低層 RC 棟。"""
        st = b["storeys"]
        hgt = b["h"] - 0.6
        fl = hgt / st
        for a, bb, L, n, zf, (u_off, Lfull) in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            po = min(0.4, self.room(a, bb, n))
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                if L < 3.0:
                    f.face("kosen_render", 0.0, L, z0, z1, 0.0, 3.0)
                    continue
                ops = [(u0 + 0.5, u1 - 0.5, z0 + 0.9, min(z1 - 0.5, z0 + 2.6)) for u0, u1 in B.bays(L, 3.6, 0.4)]
                if k == 0 and L > 8:
                    # a door in the middle bay / 中央に扉
                    mid = len(ops) // 2
                    u0, u1 = ops[mid][0], ops[mid][1]
                    ops[mid] = (u0, u1, z0 + 0.02, z0 + 2.3)
                grid_wall(f, "kosen_render", z0, z1, ops, 3.0)
                for o in ops:
                    if o[2] < z0 + 0.1:
                        f.window(*o, "kosen_render", glass="kosen_glassdoor", mullions=0, depth=0.12)
                    else:
                        f.window(*o, "kosen_render", glass="kosen_sash", mullions=1, depth=0.12)
                if po > 0.25 and n[1] < -0.5:
                    f.block("kosen_render", 0.2, L - 0.2, 0.0, min(0.5, po), z0 + 2.75, z0 + 2.85)
        self.flat_roof(m, b, hgt, parapet=0.6, things=False)

    def b_rc_core(self, m, b):
        """A stair core: narrow windows stacked, a little higher than the block. / 階段室。"""
        top = b["h"] - 0.9
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            k = 0
            while (k + 1) * 3.8 <= top + 0.01:
                z0, z1 = k * 3.8, (k + 1) * 3.8
                if z1 > zf + 0.05:
                    self.rc_end(f, L, z0, z1, k)
                k += 1
            if top > k * 3.8 and top > zf:
                f.face("kosen_render", 0.0, L, max(zf, k * 3.8), top, 0.0, 3.0)
        self.flat_roof(m, b, top, things=False)

    # ---- 24 / 25 / 総合研究棟・専攻科棟
    def b_tile(self, m, b):
        """Orange-brown tiled 4-storey block: windows in pairs with white sills,
        a ribbon on the 3rd floor of the south face, an external stair with a
        vertical glazed strip on the north side. / 茶色タイル 4 階建て。"""
        st, fl = b["storeys"], self.T["floor_m"]
        top = st * fl
        for a, bb, L, n, zf, (u_off, Lfull) in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                if n[1] < -0.7 and k == 2 and L > 10:
                    ops = [(u0 + 0.2, u1 - 0.2, z0 + 0.9, z0 + 2.7) for u0, u1 in B.bays(L, 3.6, 0.6)]
                    mull = 2
                else:
                    ops = []
                    for u0, u1 in B.bays(L, 3.6, 0.6):
                        c = (u0 + u1) / 2
                        ops += [(c - 1.3, c - 0.15, z0 + 1.0, z0 + 2.6), (c + 0.15, c + 1.3, z0 + 1.0, z0 + 2.6)]
                    mull = 1
                if n[1] > 0.7 and L > 8 and b["name"] == "r25":
                    # the north stair's glazed strip / 北側階段の縦長ガラス
                    ops = [o for o in ops if o[1] < L - 3.6]
                grid_wall(f, "kosen_tile_brown", z0, z1, ops, 2.0)
                for o in ops:
                    f.window(*o, "kosen_tile_brown", glass="kosen_sash", mullions=mull, depth=0.18)
                f.block("kosen_white", 0.0, L, 0.0, 0.04, z0 - 0.08 if k else 0.0, z0 + 0.08 if k else 0.3,
                        top=False, bottom=False, ends=False)
            if n[1] > 0.7 and L > 8 and b["name"] == "r25" and zf < 1.0:
                f.face("kosen_curtain", L - 3.2, L - 0.6, 0.6, top - 0.6, 0.03, 6.0)
            if top > zf:
                f.face("kosen_tile_brown", 0.0, L, top - 0.001, top, 0.0)
        self.flat_roof(m, b, top, parapet=1.0, mat="kosen_roof_teal")
        if b["name"] == "r25":
            # a row of 4 large AC units on the roof (GE) / 大型室外機 4 台
            c = b["poly"].centroid
            for i in range(4):
                m.box("jp_ac_unit", c.x - 3.3 + i * 2.2, c.y + 4.0, top + 0.02, top + 1.6, 1.8, 1.0)

    def b_grey(self, m, b):
        """The grey tower of 24: light grey panels, punched windows, the tall
        glazed stair/hall strip with horizontal louvres on the east face, and the
        white rooftop screen (AC plant inside) with blue 「大分高専」 facing east.
        総合研究棟の灰色の塔と屋上の「大分高専」。"""
        st, fl = b["storeys"], self.T["floor_m"]
        top = st * fl
        glass_u = None                    # the east curtain wall's span, for the sign / 看板位置用
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            east = n[0] > 0.7
            if east:
                # user's photo (2026-11-04): the south ~56 % of the east face is a light
                # curtain wall (1.4 m grid, full height), the rest a grey panel grid
                # of the same module / 東面: 南側 56 % は明るいカーテンウォール、残りは灰色パネル
                gs = 0.56 * L
                g0, g1 = (0.0, gs) if a[1] < bb[1] else (L - gs, L)
                p0, p1 = (gs, L) if a[1] < bb[1] else (0.0, L - gs)
                z0 = max(0.0, zf)
                po = min(0.3, self.room(a, bb, n))
                f.face("kosen_curtain_pale", g0, g1, z0, top, po, 5.6)
                f.side("kosen_panel_grey", g0, 0.0, po, z0, top, -1)
                f.side("kosen_panel_grey", g1, 0.0, po, z0, top, 1)
                f.hplane("kosen_white", g0, g1, 0.0, po, top, True)
                f.face("kosen_panel_grey", p0, p1, z0, top, 0.0, 5.6)
                glass_u = (f.P(g0, 0, 0), f.P(g1, 0, 0))
                continue
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                ops = [(u0 + 0.5, u1 - 0.5, z0 + 1.0, z0 + 2.5) for u0, u1 in B.bays(L, 3.2, 0.5)]
                grid_wall(f, "kosen_panel_grey", z0, z1, ops, 2.0)
                for o in ops:
                    f.window(*o, "kosen_panel_grey", glass="kosen_sash", mullions=1, depth=0.15)
            if top > zf:
                f.face("kosen_panel_grey", 0.0, L, top - 0.001, top, 0.0)
        self.flat_roof(m, b, top, parapet=1.0, mat="kosen_roof_teal", things=False)
        # the rooftop screen box: ~7.5 x 9 m, 3 m high, white, open top with AC
        # plant inside; the letters on its east face / 屋上の白い目隠し（中に室外機）
        poly = b["poly"]
        x0, y0, x1, y1 = poly.bounds
        # GE: about 10 x 3 m, 3 m high, set back from the east parapet / 東の笠木から下げた約 10 x 3 m
        cy = (y0 + y1) / 2
        if glass_u is not None:
            # photo: the sign stands over the curtain wall, not the panel grid / 看板はガラス面の上
            cy = (glass_u[0][1] + glass_u[1][1]) / 2
        bx = box(x1 - 5.0, cy - 4.5, x1 - 2.0, cy + 4.5).intersection(poly.buffer(-1.2))
        if not bx.is_empty:
            z = top + 0.02
            m.ring_walls("kosen_white", bx.exterior, z, z + 3.0, outward=True, uv_scale=2.0)
            m.ring_walls("kosen_white", bx.buffer(-0.12, join_style="mitre").exterior, z, z + 3.0, outward=False,
                         uv_scale=2.0)
            m.flat("kosen_white", bx.difference(bx.buffer(-0.12)), z + 3.0, 1.0)
            bx0, by0, bx1, by1 = bx.bounds
            for j in range(3):
                m.box("jp_ac_unit", (bx0 + bx1) / 2, by0 + 2.0 + j * 3.0, z, z + 1.5, 1.4, 1.0)
            # 「大分高専」, each ~2.2 m, dark blue, on the east face / 東面の文字
            f = B.Facade(m, np.array([bx1, by0]), np.array([bx1, by1]), np.array([1.0, 0.0]))
            self.letters(m, f, by1 - by0, "大分高専", 2.0, z + 0.5, "kosen_letter_blue", 0.07)
            # the grey plant penthouse standing above the sign box (photo) / 看板の上の灰色の塔屋
            m.box("kosen_panel_grey", (bx0 + bx1) / 2 - 0.5, (by0 + by1) / 2, z + 3.0, z + 5.2, 2.6, 4.2)

    def letters(self, m, f, L, text, height, z, mat, depth):
        """Raised letters on a wall: the face 'depth' out, their sides back to the
        wall (all facing out). / 壁面の浮き文字（外向き）。"""
        from mesh import earcut
        g = text_poly(text, height, CJK, aspect=1.0)
        gx0, gy0, gx1, gy1 = g.bounds
        u = L / 2
        for p in polys(g):
            p = orient(p, 1.0)
            rings = [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]
            V = np.concatenate(rings)
            ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
            I = earcut.triangulate_float64(V.astype(np.float64), ends).reshape(-1, 3)
            P3 = [f.P(u + q[0], depth, z + q[1] - gy0) for q in V]
            A = np.array([P3[i] for i in I.ravel()]).reshape(-1, 3, 3)
            nrm = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
            flip = (nrm[:, :2] @ f.n) < 0
            I = np.where(flip[:, None], I[:, ::-1], I)
            m.tris(mat, P3, np.r_[f.n, 0.0], V / height, I)
            for r in [p.exterior] + list(p.interiors):
                cs = list(r.coords)
                for q0, q1 in zip(cs, cs[1:]):
                    a3 = [f.P(u + q0[0], 0.0, z + q0[1] - gy0), f.P(u + q1[0], 0.0, z + q1[1] - gy0),
                          f.P(u + q1[0], depth, z + q1[1] - gy0), f.P(u + q0[0], depth, z + q0[1] - gy0)]
                    c3 = np.array(f.P(u + (q0[0] + q1[0]) / 2, -1.0, z + (q0[1] + q1[1]) / 2 - gy0))
                    # side faces away from the glyph's inside / 側面は字の外向き
                    mid2 = np.array([(q0[0] + q1[0]) / 2, (q0[1] + q1[1]) / 2])
                    tan = np.array([q1[0] - q0[0], q1[1] - q0[1]])
                    out2 = np.array([tan[1], -tan[0]])
                    probe = Point(*(mid2 + out2 / max(np.linalg.norm(out2), 1e-9) * 0.01))
                    sgn = -1.0 if p.contains(probe) else 1.0
                    cen = np.array(f.P(u + mid2[0] - sgn * out2[0] * 0.01, depth / 2,
                                       z + mid2[1] - sgn * out2[1] * 0.01 - gy0))
                    _ = c3
                    oquad(m, mat, a3, cen)

    def b_stair_white(self, m, b):
        """White stair tower between 24 and 25, slit windows. / 白い階段塔。"""
        top = b["h"] - 0.9
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            if L > 2.5:
                ops = [(L / 2 - 0.35, L / 2 + 0.35, z + 0.8, z + 3.0) for z in np.arange(0.0, top - 2.0, 3.8)
                       if z + 3.0 > zf]
                grid_wall(f, "kosen_white", max(0.0, zf), top, ops, 2.0)
                for o in ops:
                    f.window(*o, "kosen_white", glass="kosen_sash", mullions=0, depth=0.12)
            else:
                f.face("kosen_white", 0.0, L, max(0.0, zf), top, 0.0)
        self.flat_roof(m, b, top, parapet=0.5, mat="kosen_roof_grey", things=False)

    def b_glass_hall(self, m, b):
        """The 2-storey entrance hall (user's photo from the forecourt): salmon
        render frame; on the long faces a dark glazed centre in three bays between
        white pilasters, a perforated pink block screen either side; a pink frieze
        under a white cornice; glass doors. Short faces: render round one screen.
        2 層の玄関ホール: 中央の濃いガラス 3 スパン・白い柱・両脇の穴あきブロック。"""
        top = b["h"] - 0.6
        wall = "kosen_render_pink"
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            if zf >= top:
                continue
            z0 = max(0.0, zf)
            po = min(0.35, self.room(a, bb, n))
            fr = top - 0.9                                     # under the frieze / フリーズの下端
            if L >= 8.0:
                s = L * 0.21                                   # screen width each side / 両脇の幅
                parts = [("screen", 0.0, s), ("glass", s, L - s), ("screen", L - s, L)]
            else:
                parts = [("screen", 0.0, L)]
            for kind, u0, u1 in parts:
                if kind == "screen":
                    e = 0.35 if u1 - u0 > 1.2 else 0.1
                    f.face(wall, u0, u0 + e, z0, fr, 0.0, 3.0)
                    f.face(wall, u1 - e, u1, z0, fr, 0.0, 3.0)
                    f.face(wall, u0 + e, u1 - e, z0, max(z0, 0.6), 0.0, 3.0)      # plain base / 腰
                    f.face("kosen_screen", u0 + e, u1 - e, max(z0, 0.6), fr, -0.08, 2.0)
                    f.side(wall, u0 + e, -0.08, 0.0, max(z0, 0.6), fr, 1)
                    f.side(wall, u1 - e, -0.08, 0.0, max(z0, 0.6), fr, -1)
                    f.hplane(wall, u0 + e, u1 - e, -0.08, 0.0, fr, False)
                    f.hplane(wall, u0 + e, u1 - e, -0.08, 0.0, max(z0, 0.6), True)
                    continue
                # three dark glazed bays between four white pilasters / 白い柱 4 本の間に濃いガラス
                f.face("kosen_curtain", u0, u1, z0, fr, -0.15, 6.0)
                f.hplane("kosen_white", u0, u1, -0.15, 0.0, fr, False)
                f.side(wall, u0, -0.15, 0.0, z0, fr, 1)
                f.side(wall, u1, -0.15, 0.0, z0, fr, -1)
                pw = min(0.3, po) if po > 0.05 else 0.0
                for u in np.linspace(u0, u1, 4):
                    p0, p1 = max(u0, u - 0.3), min(u1, u + 0.3)
                    f.block("kosen_white", p0, p1, -0.15, pw, z0, fr, top=True, bottom=False)
                if zf < 0.5:
                    mid = (u0 + u1) / 2
                    f.face("kosen_glassdoor", mid - 1.5, mid + 1.5, 0.02, 2.4, -0.13)
            f.face(wall, 0.0, L, fr, top, 0.0, 3.0)                             # frieze / フリーズ
            if po > 0.1:
                f.block("kosen_white", 0.0, L, 0.0, po, top - 0.2, top + 0.1, top=True, bottom=True)
        self.flat_roof(m, b, top, parapet=0.6, mat="kosen_roof_grey", things=False, wall=wall)

    def b_link3(self, m, b):
        """The 3-storey open corridor (渡り廊下) joining 1, 2 and 3: slabs, solid
        parapets both sides, columns, a roof slab. / 3 層の渡り廊下。"""
        poly = b["poly"]
        fl = self.T["floor_m"]
        top = b["storeys"] * fl
        for k in range(1, b["storeys"] + 1):
            z = k * fl
            m.prism("kosen_render", poly, z - 0.3, z, top=False)
            m.flat("kosen_floor" if k < b["storeys"] else "kosen_roof_teal", poly, z, 2.0)
            m.flat("kosen_soffit", poly, z - 0.3, 2.0, up=False)
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            if L < 6:
                continue
            f = B.Facade(m, a, bb, n)
            for k in range(1, b["storeys"]):
                z0 = k * fl
                if z0 < zf:
                    continue
                f.block("kosen_spandrel", 0.0, L, -0.15, 0.0, z0, z0 + 1.1, top=True, bottom=False, ends=False)
                f.face("kosen_render", 0.0, L, z0, z0 + 1.1, -0.15)
            for u in np.linspace(0.2, L - 0.2, max(2, int(L // 4.5) + 1)):
                f.block("kosen_render", u - 0.2, u + 0.2, -0.45, -0.05, 0.0, top - 0.3, top=False, bottom=False)
        m.ring_walls("kosen_render", poly.exterior, top, top + 0.6, outward=True)
        m.flat("kosen_white", poly.difference(poly.buffer(-0.15)), top + 0.6, 1.0)

    def b_stair_round(self, m, b):
        """The semicircular stair tower at the west end of 2: rounded walls with
        window bands, a rounded white parapet. / 2 の西端の半円形階段室。"""
        poly = b["poly"]
        top = b["storeys"] * self.T["floor_m"]
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            if zf >= top:
                continue
            for k in range(b["storeys"]):
                z0 = k * 3.8
                if z0 + 3.8 <= zf:
                    continue
                f.face("kosen_render", 0.0, L, z0, z0 + 1.6, 0.0, 3.0)
                f.face("kosen_sash", 0.0, L, z0 + 1.6, z0 + 2.9, -0.08) if L < 3 else \
                    f.face("kosen_sash_2", 0.0, L, z0 + 1.6, z0 + 2.9, -0.08)
                f.face("kosen_render", 0.0, L, z0 + 2.9, z0 + 3.8, 0.0, 3.0)
                f.hplane("kosen_render", 0.0, L, -0.08, 0.0, z0 + 1.6, True)
                f.hplane("kosen_render", 0.0, L, -0.08, 0.0, z0 + 2.9, False)
        self.flat_roof(m, b, top, parapet=1.0, mat="kosen_roof_teal", things=False)

    # ---- 8 library / 図書館
    def b_library(self, m, b):
        """Library: light grey-white render, punched windows in pairs, larger
        windows on the 2nd floor, dark grey-green roof, two white penthouses. / 図書館。"""
        st, fl = b["storeys"], b.get("floor", 3.3)
        top = st * fl
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            for k in range(st):
                z0, z1 = k * fl, (k + 1) * fl
                if z1 <= zf + 0.05:
                    continue
                ops = []
                for u0, u1 in B.bays(L, 4.0, 0.8):
                    c = (u0 + u1) / 2
                    if k == 1:
                        ops.append((c - 1.6, c + 1.6, z0 + 0.7, z0 + 2.7))
                    else:
                        ops += [(c - 1.45, c - 0.2, z0 + 0.95, z0 + 2.35), (c + 0.2, c + 1.45, z0 + 0.95, z0 + 2.35)]
                if k == 0 and n[1] > 0.7 and L > 12:
                    ops = [o for o in ops if abs((o[0] + o[1]) / 2 - L / 2) > 3.0]
                    f.face("kosen_glassdoor", L / 2 - 1.8, L / 2 + 1.8, 0.02, 2.5, -0.12)
                grid_wall(f, "kosen_render_grey", z0, z1, ops, 3.0)
                for o in ops:
                    f.window(*o, "kosen_render_grey", glass="kosen_sash", mullions=1, depth=0.16)
            if top > zf:
                f.face("kosen_render_grey", 0.0, L, top - 0.001, top, 0.0)
        self.flat_roof(m, b, top, parapet=0.9, mat="kosen_roof_library", things=False)
        c = b["poly"].centroid
        for dx in (-9.0, 6.0):
            q = Point(c.x + dx, c.y + 1.0)
            if b["poly"].buffer(-3.0).contains(q):
                m.box("kosen_white", q.x, q.y, top, top + 3.0, 5.0, 4.0)
                m.box("kosen_roof_library", q.x, q.y, top + 3.0, top + 3.12, 5.3, 4.3)

    # ---- gyms / 体育館
    def b_gym(self, m, b):
        """A steel gymnasium: walls with a door/window base, the clerestory band
        under the eaves, a shallow gable roof (light blue on 17, pale grey-green on
        16) with ridge ventilators (17) or a ridge vent strip (16). / 体育館。"""
        poly = b["poly"]
        eave, ridge = b.get("eave", 9.0), b.get("ridge_h", 12.0)
        wall = "kosen_wall_gym" if b["name"] == "gym17" else "kosen_wall_stk"
        roof = ROOFS.get(b.get("roof"), "kosen_roof_gym") if b.get("roof") not in ("gym_blue", "stk") else \
            {"gym_blue": "kosen_roof_gym", "stk": "kosen_roof_stk"}[b["roof"]]
        r = orient(poly.minimum_rotated_rectangle, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, bb, cc, d = [c[(k + i) % 4] for i in range(4)]       # a-b one long side / a-b が長辺
        for ea, eb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, ea, eb, n)
            long_side = L > 0.6 * max(e)
            z_top = eave
            if zf >= z_top:
                continue
            if long_side:
                # lower wall: doors and small windows; the high window band / 下部の扉・小窓、上部の高窓帯
                shut = [((u0 + u1) / 2 - 1.5, (u0 + u1) / 2 + 1.5) for u0, u1 in B.bays(L, 12.0, 2.0)] \
                    if zf < 0.5 else []
                ops = [(u0 + 0.4, u1 - 0.4, 1.0, 2.2) for u0, u1 in B.bays(L, 4.0, 1.0)
                       if all(u1 - 0.4 < s0 - 0.2 or u0 + 0.4 > s1 + 0.2 for s0, s1 in shut)]
                hi = (0.6, L - 0.6, eave - 2.6, eave - 0.6) if L > 3.0 else None
                grid_wall(f, wall, max(0.0, zf), z_top, ops + ([hi] if hi and hi[2] >= zf else []), 2.0)
                for o in ops:
                    f.window(*o, wall, glass="kosen_sash", mullions=1, depth=0.12)
                if hi and hi[2] >= zf:
                    band_window(f, *hi, wall, glass="kosen_sash_2", pane=3.0, depth=0.12)
                for s0, s1 in shut:
                    f.face("jp_shutter", s0, s1, 0.02, 2.6, 0.02)
                f.block("kosen_white", 0.0, L, 0.0, 0.05, 2.9, 3.05, top=True, bottom=True, ends=False)
            else:
                # gable end: windows grid, louvres / 妻面: 窓の格子とガラリ
                ops = []
                for u0, u1 in B.bays(L, 3.2, 1.0):
                    ops.append((u0 + 0.3, u1 - 0.3, 1.0, 2.4))
                    ops.append((u0 + 0.3, u1 - 0.3, eave - 3.6, eave - 1.4))
                grid_wall(f, wall, max(0.0, zf), eave, ops, 2.0)
                for o in ops:
                    f.window(*o, wall, glass="kosen_sash", mullions=1, depth=0.12)
                if zf < 0.5 and L > 10:
                    f.face("kosen_glassdoor", L / 2 - 1.6, L / 2 + 1.6, 0.02, 2.5, 0.02)
        # the gable over the long axis / 長手方向の切妻
        ax = (bb - a) / e[k]
        sd = (d - a) / np.linalg.norm(d - a)
        room = min(self.room(*ed[:2], ed[3]) for ed in edges(poly))
        ov = max(0.15, min(0.7, room - 0.05))
        a2, b2 = a - ax * ov - sd * ov, bb + ax * ov - sd * ov
        d2, c2 = d - ax * ov + sd * ov, cc + ax * ov + sd * ov
        r1, r2 = (a2 + d2) / 2, (b2 + c2) / 2
        rise = ridge - eave
        W = np.linalg.norm(d - a)
        zl = eave - ov * rise / (W / 2)                      # the eave line out at the overhang / 軒先
        P = lambda q, zz: (q[0], q[1], zz)
        Lr = np.linalg.norm(b2 - a2)
        sl = math.hypot(W / 2 + ov, rise + eave - zl)
        m.quad(roof, P(a2, zl), P(b2, zl), P(r2, ridge), P(r1, ridge), (0, 0), (Lr / 2, 0), (Lr / 2, sl / 2), (0, sl / 2))
        m.quad(roof, P(c2, zl), P(d2, zl), P(r1, ridge), P(r2, ridge), (0, 0), (Lr / 2, 0), (Lr / 2, sl / 2), (0, sl / 2))
        m.quad("kosen_soffit", P(b2, zl), P(a2, zl), P(r1, ridge - 0.15), P(r2, ridge - 0.15))
        m.quad("kosen_soffit", P(d2, zl), P(c2, zl), P(r2, ridge - 0.15), P(r1, ridge - 0.15))
        # gable triangles over the end walls / 妻壁の三角
        ra, rb = (a + d) / 2, (bb + cc) / 2
        m.tris(wall, [P(d, eave), P(a, eave), P(ra, ridge)], np.r_[-ax, 0], [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        m.tris(wall, [P(bb, eave), P(cc, eave), P(rb, ridge)], np.r_[ax, 0], [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        # fascia boards / 鼻隠し
        for p0, p1 in ((a2, b2), (c2, d2)):
            m.wall("kosen_white", tuple(p0), tuple(p1), zl - 0.25, zl, 0.0, 1.0)
        # ridge: ventilators (17) or a vent strip (16) / 棟: 換気塔または換気棟
        nv = b.get("vents", 0)
        if nv:
            rm = (r1 + r2) / 2
            ru = (r2 - r1) / max(np.linalg.norm(r2 - r1), 1e-6)
            for i in range(nv):
                q = rm + ru * (i - (nv - 1) / 2.0) * b.get("vent_step", 8.4)      # GE: 4 at ~8.4 m / 換気塔 4 基
                m.cylinder("kosen_white", q[0], q[1], ridge - 0.1, ridge + 0.7, 0.35, seg=10, top=False)
                m.cylinder("kosen_white", q[0], q[1], ridge + 0.7, ridge + 1.05, 0.75, seg=12, top=True, r1=0.3)
        else:
            u = (r2 - r1) / max(np.linalg.norm(r2 - r1), 1e-6)
            nrm = np.array([-u[1], u[0]])
            q0, q1 = r1 + u * 2.0, r2 - u * 2.0
            for s in (-1, 1):
                m.quad("kosen_louvre", P(q1 + nrm * s * 0.6, ridge - 0.02), P(q0 + nrm * s * 0.6, ridge - 0.02),
                       P(q0 + nrm * s * 0.6, ridge + 0.6), P(q1 + nrm * s * 0.6, ridge + 0.6)) if s > 0 else \
                    m.quad("kosen_louvre", P(q0 + nrm * s * 0.6, ridge - 0.02), P(q1 + nrm * s * 0.6, ridge - 0.02),
                           P(q1 + nrm * s * 0.6, ridge + 0.6), P(q0 + nrm * s * 0.6, ridge + 0.6))
            m.quad(roof, P(q0 - nrm * 0.8, ridge + 0.6), P(q1 - nrm * 0.8, ridge + 0.6), P(q1 + nrm * 0.8, ridge + 0.6),
                   P(q0 + nrm * 0.8, ridge + 0.6))

    def b_budo(self, m, b):
        """Martial arts hall 20: white walls, high windows, a low grey gable roof
        (ridge east-west). / 武道場。"""
        b2 = dict(b, eave=4.6, ridge_h=6.4, vents=0, roof="greymetal")
        poly = b["poly"]
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            ops = [(u0 + 0.3, u1 - 0.3, 2.6, 4.0) for u0, u1 in B.bays(L, 2.4, 0.6)] if L > 4 else []
            low = [(u0 + 0.5, u1 - 0.5, 0.4, 1.0) for u0, u1 in B.bays(L, 3.6, 1.0)] if L > 6 else []
            grid_wall(f, "kosen_white_wall", 0.0, 4.6, ops + low, 2.0)
            for o in ops + low:
                f.window(*o, "kosen_white_wall", glass="kosen_sash", mullions=1, depth=0.1)
        self.gable(m, poly, 4.6, 1.8, "kosen_roof_greymetal", "kosen_white_wall", 0.5)
        _ = b2

    def gable(self, m, poly, eave, rise, roof, wall, ov):
        """A plain gable roof over a rectangle's long axis (eaves ov, clipped by the
        room the edges allow). / 単純な切妻屋根。"""
        poly = fit_rect(poly)
        r = orient(poly, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, bb, cc, d = [c[(k + i) % 4] for i in range(4)]
        ax = (bb - a) / e[k]
        sd = (d - a) / np.linalg.norm(d - a)
        room = min(self.room(*ed[:2], ed[3]) for ed in edges(poly))
        ov = max(0.1, min(ov, room - 0.05))
        W = np.linalg.norm(d - a)
        zl = eave - ov * rise / (W / 2)
        a2, b2 = a - ax * ov - sd * ov, bb + ax * ov - sd * ov
        d2, c2 = d - ax * ov + sd * ov, cc + ax * ov + sd * ov
        r1, r2 = (a2 + d2) / 2, (b2 + c2) / 2
        P = lambda q, zz: (q[0], q[1], zz)
        z1 = eave + rise
        m.quad(roof, P(a2, zl), P(b2, zl), P(r2, z1), P(r1, z1))
        m.quad(roof, P(c2, zl), P(d2, zl), P(r1, z1), P(r2, z1))
        m.quad("kosen_soffit", P(b2, zl), P(a2, zl), P(r1, z1 - 0.12), P(r2, z1 - 0.12))
        m.quad("kosen_soffit", P(d2, zl), P(c2, zl), P(r2, z1 - 0.12), P(r1, z1 - 0.12))
        ra, rb = (a + d) / 2, (bb + cc) / 2
        m.tris(wall, [P(d, eave), P(a, eave), P(ra, z1)], np.r_[-ax, 0], [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])
        m.tris(wall, [P(bb, eave), P(cc, eave), P(rb, z1)], np.r_[ax, 0], [(0, 0), (1, 0), (0.5, 1)], [0, 1, 2])

    def hip(self, m, poly, eave, rise, roof, ov):
        """A hipped roof over a rectangle (eaves ov). / 寄棟屋根。"""
        poly = fit_rect(poly)
        r = orient(poly, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, bb, cc, d = [c[(k + i) % 4] for i in range(4)]
        room = min(self.room(*ed[:2], ed[3]) for ed in edges(poly))
        ov = max(0.1, min(ov, room - 0.05))
        ax = (bb - a) / e[k]
        sd = (d - a) / np.linalg.norm(d - a)
        a2, b2 = a - ax * ov - sd * ov, bb + ax * ov - sd * ov
        d2, c2 = d - ax * ov + sd * ov, cc + ax * ov + sd * ov
        W = np.linalg.norm(d2 - a2)
        Lr = np.linalg.norm(b2 - a2)
        ins = min(W / 2, Lr / 2 - 0.05)
        r1 = (a2 + d2) / 2 + ax * ins
        r2 = (b2 + c2) / 2 - ax * ins
        zl = eave - ov * rise / (W / 2)
        z1 = eave + rise
        P = lambda q, zz: (q[0], q[1], zz)
        m.quad(roof, P(a2, zl), P(b2, zl), P(r2, z1), P(r1, z1))
        m.quad(roof, P(c2, zl), P(d2, zl), P(r1, z1), P(r2, z1))
        m.tris(roof, [P(b2, zl), P(c2, zl), P(r2, z1)], (0, 0, 1), [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
        m.tris(roof, [P(d2, zl), P(a2, zl), P(r1, z1)], (0, 0, 1), [(0, 0), (1, 0), (.5, 1)], [0, 1, 2])
        m.quad("kosen_soffit", P(b2, zl), P(a2, zl), P(d2, zl), P(c2, zl))

    def low_walls(self, m, b, h, wall="kosen_white_wall", door=True, shutter=False):
        """Walls of a small single-storey building: a door, a window or two. / 小屋の外壁。"""
        best = max(edges(b["poly"]), key=lambda e: e[2])
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            ops = []
            if L > 3.2:
                ops = [(u0 + 0.5, u1 - 0.5, 0.95, min(h - 0.4, 2.1)) for u0, u1 in B.bays(L, 3.0, 0.8)][:4]
            first = np.allclose(a, best[0]) and door and L > 2.4
            if first and ops:
                ops = ops[1:] if len(ops) > 1 else []
            grid_wall(f, wall, max(0.0, zf), h, ops, 2.0)
            for o in ops:
                f.window(*o, wall, glass="kosen_sash", mullions=1, depth=0.1)
            if first:
                if shutter and L > 5:
                    f.face("jp_shutter", 0.6, min(L - 0.6, 3.8), 0.02, min(h - 0.4, 2.8), 0.02)
                else:
                    f.face("jp_door_steel", 0.7, 1.6, 0.02, 2.0, 0.02)

    def b_shed(self, m, b):
        """Small single-storey store / shed: white walls, flat roof with a thin
        parapet, or a mono-pitch metal roof for the long ones. / 小さな倉庫・小屋。"""
        h = b["h"] - 0.3
        self.low_walls(m, b, h, shutter=b["poly"].area > 60)
        poly = b["poly"]
        mat = ROOFS.get(b.get("roof"), "kosen_roof_grey")
        m.flat(mat, poly, h + 0.25, 3.0)
        # the roof edge band only where no neighbour covers the edge (a ring drew
        # it over the touching shed's wall) / 隣棟に接しない辺だけに笠木帯
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            if zf < h + 0.25:
                B.Facade(m, a, bb, n).face("kosen_white", 0.0, L, max(h, zf), h + 0.25, 0.0)

    def b_guard(self, m, b):
        """Guard house: white walls, wide windows to the gate, a dark flat roof
        with a thin eave. / 守衛室。"""
        poly = b["poly"]
        h = 2.8
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            if n[0] > 0.5 or n[1] > 0.5:
                ops = [(0.5, L - 0.5, 0.95, 2.3)] if L > 1.5 else []
            else:
                ops = [(L / 2 - 0.5, L / 2 + 0.5, 0.02, 2.0)] if L > 2 else []
            grid_wall(f, "kosen_white_wall", 0.0, h, ops, 2.0)
            for o in ops:
                f.window(*o, "kosen_white_wall", glass="kosen_glassdoor" if o[2] < 0.1 else "kosen_sash",
                         mullions=1 if o[2] > 0.1 else 0, depth=0.1)
        room = min(self.room(*ed[:2], ed[3]) for ed in edges(poly))
        ov = max(0.0, min(0.5, room - 0.05))
        top = poly.buffer(ov, join_style="mitre") if ov > 0.02 else poly
        prism_o(m, "kosen_roof_dark", top, h, h + 0.22, wall="kosen_white")
        m.flat("kosen_soffit", top, h, 2.0, up=False)

    def b_bikeshed(self, m, b):
        """Bicycle shelter: steel posts, a mono-pitch corrugated roof, open sides,
        a rack rail. / 自転車置場。"""
        poly = b["poly"]
        mat = ROOFS.get(b.get("roof"), "kosen_roof_blue")
        r = orient(poly.minimum_rotated_rectangle, 1.0)
        c = [np.array(q) for q in list(r.exterior.coords)[:4]]
        e = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        k = 0 if e[0] >= e[1] else 1
        a, bb, cc, d = [c[(k + i) % 4] for i in range(4)]
        Ln = e[k]
        ax = (bb - a) / Ln
        sd = (d - a) / np.linalg.norm(d - a)
        W = np.linalg.norm(d - a)
        lo, hi = 2.2, 2.6
        P = lambda q, zz: (q[0], q[1], zz)
        m.quad(mat, P(a, lo), P(bb, lo), P(cc, hi), P(d, hi), (0, 0), (Ln / 2, 0), (Ln / 2, W / 2), (0, W / 2))
        m.quad("kosen_soffit", P(bb, lo - 0.05), P(a, lo - 0.05), P(d, hi - 0.05), P(cc, hi - 0.05))
        for u in np.linspace(0.3, Ln - 0.3, max(2, int(Ln // 3.0) + 1)):
            for v, zz in ((0.4, lo), (W - 0.4, hi)):
                q = a + ax * u + sd * v
                m.box("kosen_steel", q[0], q[1], 0.0, zz - 0.05, 0.09, 0.09)
        q0, q1 = a + sd * (W * 0.5) + ax * 0.5, bb + sd * (W * 0.5) - ax * 0.5
        m.wall("kosen_steel", tuple(q0), tuple(q1), 0.25, 0.32, 0.0, 1.0)
        m.wall("kosen_steel", tuple(q1), tuple(q0), 0.25, 0.32, 0.0, 1.0)

    def b_workshop(self, m, b):
        """Single-storey workshop: cream walls with high windows, roll-up shutters
        on the west service road, flat teal roof. / 実習工場。"""
        h = b["h"] - 0.6
        for a, bb, L, n, zf, _ in self.facade_pieces(b):
            f = B.Facade(m, a, bb, n)
            if zf >= h or L < 1.0:
                continue
            ops = [(u0 + 0.4, u1 - 0.4, 2.2, h - 0.7) for u0, u1 in B.bays(L, 3.6, 0.6)] if L > 4 else []
            grid_wall(f, "kosen_render", max(0.0, zf), h, ops, 3.0)
            for o in ops:
                f.window(*o, "kosen_render", glass="kosen_sash", mullions=2, depth=0.12)
            if n[0] < -0.7 and L > 8 and zf < 0.5:
                for u0, u1 in B.bays(L, 9.0, 1.0)[:3]:
                    um = (u0 + u1) / 2
                    f.face("jp_shutter", um - 1.8, um + 1.8, 0.02, 1.95, 0.02)
        self.flat_roof(m, b, h, parapet=0.6, things=False)

    def b_sawtooth(self, m, b):
        """The machine shop's sawtooth roof: rust-brown slopes, glazed verticals
        facing north. / のこぎり屋根（北向きの採光面）。"""
        poly = b["poly"]
        h = 4.6
        self.low_walls(m, b, h, wall="kosen_render", door=False)
        x0, y0, x1, y1 = poly.bounds
        n = max(2, int(round((y1 - y0) / 3.0)))
        dy = (y1 - y0) / n
        for i in range(n):
            ya, yb = y0 + i * dy, y0 + (i + 1) * dy
            P0, P1 = (x0, ya, h), (x1, ya, h)
            Q0, Q1 = (x0, yb, h + 1.6), (x1, yb, h + 1.6)
            m.quad("kosen_roof_rust", P0, P1, Q1, Q0, (0, 0), ((x1 - x0) / 2, 0), ((x1 - x0) / 2, 1), (0, 1))
            m.quad("kosen_sash_2", (x1, yb, h), (x0, yb, h), (x0, yb, h + 1.6), (x1, yb, h + 1.6),
                   (0, 0), ((x1 - x0) / 2.5, 0), ((x1 - x0) / 2.5, 1), (0, 1))
            for xx, sgn in ((x0, -1), (x1, 1)):
                pts = [(xx, ya, h), (xx, yb, h), (xx, yb, h + 1.6)]
                if sgn < 0:
                    pts = pts[::-1]
                m.tris("kosen_render", pts, (sgn, 0, 0), [(0, 0), (1, 0), (1, 1)], [0, 1, 2])

    def b_canopy(self, m, b):
        """A covered way: a thin roof slab on slender posts. / 渡り屋根。"""
        poly = b["poly"]
        h = 2.8
        prism_o(m, "kosen_roof_grey", poly, h, h + 0.15, wall="kosen_white")
        m.flat("kosen_soffit", poly, h, 2.0, up=False)
        r = orient(poly.minimum_rotated_rectangle, 1.0)
        line = LineString([r.exterior.coords[0], r.exterior.coords[1]]) \
            if math.dist(*r.exterior.coords[:2]) > math.dist(*r.exterior.coords[1:3]) else \
            LineString([r.exterior.coords[1], r.exterior.coords[2]])
        cen = r.centroid
        mid = line.interpolate(0.5, normalized=True)
        off = (cen.x - mid.x, cen.y - mid.y)
        for k in range(int(line.length // 3.6) + 1):
            q = line.interpolate(min(line.length, 0.3 + k * 3.6))
            m.box("kosen_steel", q.x + off[0], q.y + off[1], 0.0, h, 0.1, 0.1)

    def b_annex_red(self, m, b):
        """Single-storey annex west of the spine: cream walls with windows, a
        red-brown hipped tile roof. / 赤い寄棟屋根の平屋。"""
        h = 3.3
        self.low_walls(m, b, h, wall="kosen_render")
        m.flat("kosen_soffit", b["poly"], h, 2.0, up=False)
        self.hip(m, b["poly"], h, 1.9, "kosen_roof_redtile", 0.55)

    def b_hiphouse(self, m, b):
        """Staff hall 11: house-like, white walls, grey tiled hip roof over its
        two wings. / 職員会館（灰色瓦の寄棟）。"""
        poly = b["poly"]
        h = 3.0
        self.low_walls(m, b, h)
        x0, y0, x1, y1 = poly.bounds
        cut = y0 + (y1 - y0) * 0.57
        wings = [box(x0, y0, x1, cut).intersection(poly), poly.difference(box(x0, y0, x1, cut))]
        roofed = []
        for w in wings:
            for p in polys(onterrain._clean(w, 1.0)):
                r = fit_rect(p)
                rr = r.exterior.coords
                if r.area > 4.0 and min(math.dist(rr[0], rr[1]), math.dist(rr[1], rr[2])) > 2.5:
                    self.hip(m, r, h, 1.6, "jp_kawara_ibushi", 0.5)
                    roofed.append(r)
        # whatever no hip covers gets a flat roof behind a low parapet, so every
        # part of the footprint is roofed / 寄棟で覆えない部分は陸屋根
        rest = onterrain._clean(poly.difference(unary_union(roofed).buffer(-0.01) if roofed else Polygon()), 0.2)
        for p in polys(rest):
            m.flat("kosen_roof_grey", p, h + 0.02, 3.0)
            m.flat("kosen_soffit", p, h - 0.01, 2.0, up=False)
            m.ring_walls("kosen_white_wall", p.exterior, h, h + 0.35, outward=True)
            m.flat("kosen_white", p.difference(p.buffer(-0.12)), h + 0.35, 1.0)

    # ================= site things / 敷地の物 =================
    def site_things(self, m):
        S = self.S
        self.fences(m)
        self.gate_area(m)
        self.ball_net(m)
        self.tennis_fence(m)
        self.pool_fence(m)
        self.parterre_and_monument(m)
        self.gas(m)
        self.greenhouse_mesh(m)
        self.goals(m)
        self.trees(m)
        _ = S

    def follow(self):
        """A Mesh that follows the finished ground (fences, nets, kerbs). / 地盤に沿うメッシュ。"""
        m = Mesh()
        m.zfun = lambda x, y: self.S.surface_z(x, y)
        m.dense = 2.0
        return m

    def rigid(self, x, y, r=0.3):
        m = Mesh()
        m.zoff = self.S.low(x, y, r)
        return m

    def clear_line(self, line, clear=0.3, extra=None):
        """A line cut clear of roads, paths and buildings. / 道路・歩道・建物を避けた線。"""
        stop = unary_union([self.no_go.buffer(clear), self.footprint.buffer(0.3)] + ([extra] if extra is not None else []))
        g = line.difference(stop)
        return [q for q in getattr(g, "geoms", [g]) if q.geom_type == "LineString" and q.length > 0.8]

    def chain_link(self, m, line, h, post=2.5, wire=0.3, mat="kosen_net", post_mat="kosen_steel", rails=None):
        """See-through chain-link / ball-stop net: posts, rails, thin wires (two-
        sided strips). / 透ける金網・防球ネット。"""
        rails = rails if rails is not None else (0.05, h / 2, h - 0.05)
        Lg = line.length
        for k in range(int(Lg // post) + 1):
            q = line.interpolate(min(Lg, k * post))
            r = self.rigid(q.x, q.y, 0.1)
            r.box(post_mat, q.x, q.y, -0.2, h + 0.06, 0.07 if h < 6 else 0.16, 0.07 if h < 6 else 0.16)
            m.merge(r)
        fm = self.follow()
        c = list(line.coords)
        for a, b in zip(c, c[1:]):
            seg = math.dist(a, b)
            if seg < 1e-3:
                continue
            for zz in rails:
                fm.wall(post_mat, a, b, zz - 0.025, zz + 0.025, 0.0, 1.0)
                fm.wall(post_mat, *back(a, b, 0.03), zz - 0.025, zz + 0.025, 0.0, 1.0)
            for zz in np.arange(0.4, h - 0.3, 0.6 if h < 6 else 1.0):
                fm.wall(mat, a, b, zz - 0.007, zz + 0.007, 0.0, 1.0)
                fm.wall(mat, *back(a, b, 0.008), zz - 0.007, zz + 0.007, 0.0, 1.0)
            t = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
            dense, fm.dense = fm.dense, None
            for k in range(int(seg // wire) + 1):
                u = min(seg, k * wire)
                p0 = (a[0] + t[0] * (u - 0.008), a[1] + t[1] * (u - 0.008))
                p1 = (a[0] + t[0] * (u + 0.008), a[1] + t[1] * (u + 0.008))
                zb = float(self.S.surface_z([0.5 * (p0[0] + p1[0])], [0.5 * (p0[1] + p1[1])])[0])
                fm.zfun = None
                fm.wall(mat, (p0[0], p0[1]), (p1[0], p1[1]), zb, zb + h, 0.0, 1.0)
                fm.wall(mat, *back((p0[0], p0[1]), (p1[0], p1[1]), 0.008), zb, zb + h, 0.0, 1.0)
                fm.zfun = lambda x, y: self.S.surface_z(x, y)
            fm.dense = dense
        m.merge(fm)

    def fences(self, m):
        """White tubular fence on a low wall along the main road (Street View),
        chain-link round the rest of the site, gaps at the gates. / 敷地の柵。"""
        ring = self.site.buffer(-0.8, join_style="mitre").exterior
        main = LineString([(-12.0, -40.0), (-12.0, -340.0)]).buffer(9.0)          # the main-road side / 幹線側
        for g in self.clear_line(ring, 0.4):
            on_main = g.intersection(main)
            rest = g.difference(main.buffer(0.01))
            for q in getattr(on_main, "geoms", [on_main]):
                if q.geom_type == "LineString" and q.length > 1.0:
                    self.tube_fence(m, q)
            for q in getattr(rest, "geoms", [rest]):
                if q.geom_type == "LineString" and q.length > 1.0:
                    self.chain_link(m, q, 1.8, wire=0.35)

    def tube_fence(self, m, line):
        """Low concrete wall (0.45 m) with a white tubular rail fence on it. / 白いパイプ柵と腰壁。"""
        fm = self.follow()
        c = list(line.coords)
        # the low wall as one closed solid with mitred corners (two offset faces
        # per segment left a V slot at every corner) / 腰壁は一体の立体（角に隙間を残さない）
        wall = line.buffer(0.18, cap_style="flat", join_style="mitre", single_sided=True)
        for p in polys(wall):
            fm.ring_walls("kosen_plinth", p.exterior, -0.2, 0.45, outward=True)
            fm.flat("kosen_plinth", p, 0.45, 1.0)
        for a, b in zip(c, c[1:]):
            for zz in (0.95, 1.45):
                fm.wall("kosen_white_rail", a, b, zz - 0.03, zz + 0.03, 0.0, 1.0)
                fm.wall("kosen_white_rail", *back(a, b, 0.05), zz - 0.03, zz + 0.03, 0.0, 1.0)
        m.merge(fm)
        Lg = line.length
        for k in range(int(Lg // 2.0) + 1):
            q = line.interpolate(min(Lg, k * 2.0))
            r = self.rigid(q.x, q.y, 0.1)
            r.box("kosen_white_rail", q.x, q.y, 0.45, 1.5, 0.05, 0.05)
            m.merge(r)

    def gate_area(self, m):
        """Main gate (Street View): tan masonry wing walls either side of the gate
        road, the grey stone pier with the name plate on the island between the
        lanes when there is one, a sliding gate leaf parked behind the south wall.
        正門: 袖壁・門柱（銘板）・引き戸。"""
        (nx, ny), (sx, sy) = self.gate_span
        x = 0.5 * (nx + sx)
        stop = self.no_go.buffer(0.35)
        walls = []
        for y0, y1 in ((ny + 0.4, ny + 7.0), (sy - 7.0, sy - 0.4)):
            wall = box(x - 0.9, min(y0, y1), x - 0.3, max(y0, y1)).difference(stop)
            for p in polys(wall):
                if p.area < 0.5:
                    continue
                walls.append(p)
                r = self.rigid(p.centroid.x, p.centroid.y, 1.5)
                r.prism("kosen_masonry", p, -0.3, 1.6, top=False)
                r.flat("kosen_stone", p.buffer(0.04, join_style="mitre"), 1.66, 1.0)
                r.prism("kosen_stone", p.buffer(0.04, join_style="mitre"), 1.6, 1.66, top=False)
                m.merge(r)
        # the stone pier with the name plate: on an island inside the gate road
        # 門柱（銘板）: 車路の間の島の上
        isl = [g for g in polys(self.L.islands) if g.distance(Point(x - 3.0, 0.5 * (ny + sy))) < 6.0]
        for g in isl:
            core = g.buffer(-0.4)
            if core.is_empty:
                continue
            c = core.centroid
            w = min(4.5, (core.bounds[3] - core.bounds[1]) - 0.2)
            dpt = min(0.6, (core.bounds[2] - core.bounds[0]) - 0.1)
            if w < 1.0 or dpt < 0.3:
                continue
            r = self.rigid(c.x, c.y, 1.0)
            r.box("kosen_stone", c.x, c.y, -0.3, 1.7, dpt, w)
            r.quad("kosen_plate", (c.x + dpt / 2 + 0.01, c.y - w * 0.4, 0.7), (c.x + dpt / 2 + 0.01, c.y + w * 0.4, 0.7),
                   (c.x + dpt / 2 + 0.01, c.y + w * 0.4, 1.4), (c.x + dpt / 2 + 0.01, c.y - w * 0.4, 1.4))
            m.merge(r)
            print("landmark %s: gate pier on the island at (%.1f, %.1f)" % (TAG, c.x, c.y))
            break
        else:
            # no island: the name plate on the east face of the longest wing wall,
            # 2 cm proud / 島が無ければ袖壁の東面に銘板
            if walls:
                p = max(walls, key=lambda g: g.area)
                x0, y0, x1, y1 = p.bounds
                w = min(2.8, (y1 - y0) - 0.5)
                yc = (y0 + y1) / 2
                xf = x1 + 0.02

                def fits(w):
                    a, b = Point(xf, yc - w / 2), Point(xf, yc + w / 2)
                    return (p.distance(a) < 0.04 and p.distance(b) < 0.04 and
                            not LineString([(xf, yc - w / 2), (xf, yc + w / 2)]).buffer(0.03).intersects(stop))
                while w > 0.8 and not fits(w):
                    w -= 0.2
                if w > 0.8:
                    r = self.rigid(p.centroid.x, yc, 1.5)
                    r.quad("kosen_plate", (xf, yc - w / 2, 0.6), (xf, yc + w / 2, 0.6), (xf, yc + w / 2, 0.6 + w * 0.25),
                           (xf, yc - w / 2, 0.6 + w * 0.25))
                    m.merge(r)
        # the sliding gate leaf, parked behind the south wing wall / 引き戸（南袖壁の裏）
        y0 = sy - 1.0
        leaf = box(x - 1.4, y0 - 6.0, x - 1.25, y0)
        if not leaf.intersects(stop) and not leaf.intersects(self.footprint):
            r = self.rigid(x - 1.3, y0 - 3.0, 3.0)
            for zz in (0.12, 1.45):
                r.box("kosen_steel", x - 1.32, y0 - 3.0, zz, zz + 0.06, 0.06, 6.0)
            for k in range(25):
                r.box("kosen_steel", x - 1.32, y0 - 5.9 + k * 0.24, 0.15, 1.45, 0.03, 0.03)
            m.merge(r)

    def ball_net(self, m):
        """The tall green ball-stop net behind the infield, along the field's east
        and north edges (GE: steel posts ~12-15 m). / 防球ネット。"""
        for g in self.clear_line(self.net_line, 0.4):
            self.chain_link(m, g, self.net_h, post=6.0, wire=0.8, mat="kosen_net",
                            rails=(0.05, 3.0, 6.0, 9.0, self.net_h - 0.05))

    def tennis_fence(self, m):
        if self.p_tennis is None:
            return
        ring = LineString(list(orient(self.tennis.buffer(-0.4, join_style="mitre"), 1.0).exterior.coords))
        for g in self.clear_line(ring, 0.4):
            self.chain_link(m, g, 4.0, post=3.0, wire=0.35)
        # nets / ネット
        for c, rect in self.tennis_courts():
            a, b, cc, d = [np.array(q) for q in rect]
            p0, p1 = (a + d) / 2, (b + cc) / 2
            r = Mesh()
            r.zoff = self.p_tennis.z
            r.wall("kosen_net", tuple(p0), tuple(p1), 0.0, 0.92, 0.0, 1.0)
            r.wall("kosen_net", *back(p0, p1, 0.01), 0.0, 0.92, 0.0, 1.0)
            for q in (p0, p1):
                r.box("kosen_steel", q[0], q[1], 0.0, 1.07, 0.08, 0.08)
            m.merge(r)

    def pool_fence(self, m):
        if self.p_pool is None:
            return
        ring = LineString(list(orient(self.pool_deck.buffer(-0.3, join_style="mitre"), 1.0).exterior.coords))
        for g in self.clear_line(ring, 0.4):
            self.chain_link(m, g, 2.0, wire=0.3)

    def goals(self, m):
        """Football goals at both ends of the field (GE). / サッカーゴール。"""
        if self.p_field is None:
            return
        f = self.p_field.poly
        for (x, y) in ((-318.0, -150.0), (-178.0, -232.0)):
            if not f.buffer(-2.0).contains(Point(x, y)):
                continue
            r = Mesh()
            r.zoff = self.p_field.z
            ang = math.atan2(-232.0 + 150.0, -178.0 + 318.0) + math.pi / 2
            c, s = math.cos(ang), math.sin(ang)
            for k in (-1, 1):
                r.cylinder("kosen_white_rail", x + c * 3.66 * k, y + s * 3.66 * k, 0.0, 2.44, 0.06, seg=6)
            r.box("kosen_white_rail", x, y, 2.38, 2.5, 7.4, 0.1, ang)
            m.merge(r)

    def parterre_and_monument(self, m):
        """The forecourt parterre (low clipped hedges, a dark turf panel, topiary)
        and the monument (a dark bronze abstract sculpture on a stone plinth); both
        stand on the road network's islands, kept 0.4 m off the kerbs. / 前庭の花壇と記念碑。"""
        isl = self.L.islands
        par = [g for g in polys(isl) if g.intersects(self.parterre)]
        for g in par:
            core = g.buffer(-0.5, join_style="mitre")
            if core.is_empty:
                continue
            fm = Mesh()
            if self.drape is not None:
                fm.zfun = lambda x, y: self.drape.z(x, y) + onterrain.BASE
                fm.dense = 2.0
            for p in polys(core):
                ringm = p.difference(p.buffer(-0.45, join_style="mitre"))
                fm.prism("jp_hedge", ringm, 0.0, 0.55, top=True)
                x0, y0, x1, y1 = p.bounds
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                panel = box(cx - (x1 - x0) * 0.18, cy - (y1 - y0) * 0.22, cx + (x1 - x0) * 0.18, cy + (y1 - y0) * 0.22)
                fm.flat("kosen_turf", panel.intersection(p.buffer(-1.0)), 0.02, 2.0)
                for dx in (-0.36, -0.24, 0.24, 0.36):
                    hb = box(cx + (x1 - x0) * dx - 0.3, y0 + 1.2, cx + (x1 - x0) * dx + 0.3, y1 - 1.2).intersection(
                        p.buffer(-0.8))
                    if not hb.is_empty:
                        fm.prism("jp_hedge", hb, 0.0, 0.45, top=True)
            m.merge(fm)
            for dx in (-0.42, 0.42):
                c = g.centroid
                q = Point(c.x + (g.bounds[2] - g.bounds[0]) * dx, c.y)
                if g.buffer(-1.2).contains(q):
                    r = self.rigid(q.x, q.y, 0.6)
                    r.sphere("leaves_dark", q.x, q.y, 1.0, 0.9, 0.9, 0.8, seg=8, rings=5)
                    m.merge(r)
        mx, my = self.monument
        mon = [g for g in polys(isl) if g.distance(Point(mx, my)) < 3.0]
        for g in mon:
            core = g.buffer(-0.6)
            if core.is_empty:
                continue
            c = core.centroid
            self.monument_mesh(m, g, c)
            break

    # the monument from the user's photo (2026-11-04, looking west from the gate
    # drive): blade outlines in photo pixels, 0.02 m/px, u = (px - 400), z = (535 - py);
    # each (outline, depth offset). / 記念碑（利用者の写真から、1 px = 0.02 m）
    MONUMENT_BLADES = (
        (((185, 528), (265, 432), (292, 446), (228, 528)), -0.25),                       # low left strut
        (((300, 528), (300, 420), (320, 380), (318, 300), (335, 265), (352, 300), (345, 400),
          (356, 460), (345, 528)), 0.0),                                                 # tall left blade
        (((358, 528), (358, 420), (370, 320), (378, 290), (387, 320), (382, 400), (392, 528)), 0.3),
        (((368, 528), (372, 395), (400, 365), (428, 392), (420, 528)), -0.3),            # the gear's fan
        (((405, 528), (410, 400), (418, 365), (430, 385), (425, 528)), 0.25),
        (((430, 528), (440, 450), (458, 405), (466, 420), (455, 528)), 0.0),
        (((428, 470), (446, 458), (606, 518), (600, 530), (450, 530)), -0.2),            # long right strut
    )

    def monument_mesh(self, m, g, c):
        """Grey concrete abstract sculpture (~5.4 m high, ~8.4 m wide): vertical
        blades extruded 0.4 m, at staggered depths, a toothed gear disc on the
        fan, on a long low plinth with a dark plate on its front; facing east,
        spread north-south along the island; hedges behind, box shrubs in front.
        Scaled down to fit the island 0.6 m off its kerbs.
        灰色コンクリートの抽象彫刻・歯車・長い基壇・銘板。"""
        from mesh import earcut
        # forecourt axes from the parterre (east and north) / 前庭の軸
        pc = list(orient(self.parterre.minimum_rotated_rectangle, 1.0).exterior.coords)
        es = sorted([(np.array(pc[i + 1]) - np.array(pc[i])) for i in range(4)], key=lambda v: -np.linalg.norm(v))
        ex = es[0] / np.linalg.norm(es[0])
        if ex[0] < 0:
            ex = -ex
        nt = np.array([-ex[1], ex[0]])                  # north along the plinth / 基壇方向（北）
        core = g.buffer(-0.6)
        span = LineString([(c.x - nt[0] * 30, c.y - nt[1] * 30), (c.x + nt[0] * 30, c.y + nt[1] * 30)]).intersection(core)
        segs = [s for s in getattr(span, "geoms", [span]) if not s.is_empty]
        avail = min(segs, key=lambda s: s.distance(c)).length if segs else 0.0
        if avail < 3.0:
            print("landmark %s: monument island too small (%.1f m)" % (TAG, avail))
            return
        k = min(1.0, avail / 13.2)
        cen = np.array([c.x, c.y])
        r = self.rigid(c.x, c.y, 2.0)
        # plinth 13 x 1.4 x 0.3 m / 基壇
        pl = Polygon([tuple(cen + nt * u + ex * w) for u, w in ((-6.6 * k, -0.7), (6.6 * k, -0.7), (6.6 * k, 0.7),
                                                                (-6.6 * k, 0.7))])
        pl = pl.intersection(core) if not core.contains(pl) else pl
        r.prism("kosen_concrete", pl, -0.2, 0.3, top=True)
        # the dark plate, sloped on the plinth's front edge (photo: right of centre) / 銘板
        u0, u1 = 2.9 * k, 5.0 * k
        q = lambda u, w, z: (cen[0] + nt[0] * u + ex[0] * w, cen[1] + nt[1] * u + ex[1] * w, z)
        oquad(r, "kosen_plate", [q(u0, 0.72, 0.3), q(u1, 0.72, 0.3), q(u1, 0.45, 0.48), q(u0, 0.45, 0.48)],
              q((u0 + u1) / 2, -0.5, 0.0))
        s = 0.02 * k

        def slab(mat, outline, d0, d1, z0):
            p = orient(Polygon(outline), 1.0)
            V = np.asarray(p.exterior.coords)[:-1]
            I = earcut.triangulate_float64(V.astype(np.float64), np.array([len(V)], np.uint32)).reshape(-1, 3)
            for d, sg in ((d1, 1.0), (d0, -1.0)):
                P3 = [q(v[0], d, z0 + v[1]) for v in V]
                A = np.array([P3[i] for i in I.ravel()]).reshape(-1, 3, 3)
                nr = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
                flip = (nr[:, :2] @ (ex * sg)) < 0
                r.tris(mat, P3, np.r_[ex * sg, 0.0], V / 2.0, np.where(flip[:, None], I[:, ::-1], I))
            inside = np.array(p.representative_point().coords[0])
            for v0, v1 in zip(V, np.roll(V, -1, 0)):
                mid = (v0 + v1) / 2
                ctr = q(mid[0] + (inside[0] - mid[0]) * 0.02, (d0 + d1) / 2, z0 + mid[1] + (inside[1] - mid[1]) * 0.02)
                oquad(r, mat, [q(v0[0], d0, z0 + v0[1]), q(v1[0], d0, z0 + v1[1]), q(v1[0], d1, z0 + v1[1]),
                               q(v0[0], d1, z0 + v0[1])], ctr)

        for outline, dw in self.MONUMENT_BLADES:
            pts = [((px - 400) * s, (535 - py) * s) for px, py in outline]
            slab("kosen_concrete", pts, dw * k - 0.2, dw * k + 0.2, 0.25)
        # the gear: 12 teeth, r 0.32 m, proud of the fan's front / 歯車（12 歯）
        gu, gz = (390 - 400) * s, (535 - 415) * s + 0.25
        teeth = []
        for i in range(24):
            a = math.pi * 2 * i / 24
            rr = (0.34 if i % 2 == 0 else 0.27) * k
            teeth.append((gu + math.cos(a) * rr, gz + math.sin(a) * rr))
        slab("kosen_concrete", [(u, z) for u, z in teeth], -0.3 * k + 0.2, -0.3 * k + 0.3, 0.0)
        m.merge(r)
        # hedge band behind (west), box shrubs at the front corners (photo) / 後ろの生垣・前の玉散らし
        if self.drape is not None:
            back = Polygon([tuple(cen + nt * u + ex * w) for u, w in ((-30, -30), (30, -30), (30, -1.4), (-30, -1.4))])
            ring = g.buffer(-0.5).difference(g.buffer(-1.4)).intersection(back)
            hm = Mesh()
            hm.zfun = lambda x, y: self.drape.z(x, y) + onterrain.BASE
            hm.dense = 2.0
            if not ring.is_empty:
                hm.prism("jp_hedge", ring, 0.0, 0.9, top=True)
            for u, w in ((-5.6, 2.2), (-4.4, 2.6), (5.8, 2.3)):
                sb = box(-0.55, -0.55, 0.55, 0.55)
                sp = Polygon([tuple(cen + nt * (u * k + x) + ex * (w + y)) for x, y in sb.exterior.coords[:-1]])
                if g.buffer(-0.5).contains(sp) and not sp.intersects(pl):
                    hm.prism("jp_hedge", sp, 0.0, 0.7, top=True)
            m.merge(hm)

    def gas(self, m):
        """The plant yard east of the stores: white gas cylinders/tanks behind a
        low wall. / ガスボンベ置場。"""
        g = self.gas_yard.difference(self.no_go.buffer(0.4)).difference(self.footprint.buffer(0.5))
        if g.is_empty:
            return
        x0, y0, x1, y1 = g.bounds
        for i in range(3):
            x, y = x0 + 2.0 + i * 3.2, (y0 + y1) / 2
            if g.buffer(-1.3).contains(Point(x, y)):
                r = self.rigid(x, y, 1.2)
                r.cylinder("kosen_white", x, y, 0.0, 3.2, 1.0, seg=12, top=True)
                m.merge(r)

    def greenhouse_mesh(self, m):
        """The campus farm's plastic greenhouse (imagery: white tunnel ~26 x 6 m):
        steel hoops every 1.5 m, a translucent white skin over a half-round
        section, framed ends with a door. / ビニールハウス。"""
        (cx, cy), Ln, W, ang = self.greenhouse
        d = np.array([math.cos(ang), math.sin(ang)])
        nrm = np.array([-d[1], d[0]])
        foot = Polygon([tuple(np.array([cx, cy]) + d * u * Ln / 2 + nrm * v * W / 2)
                        for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        if foot.intersects(self.no_go.buffer(0.4)) or foot.intersects(self.footprint.buffer(0.5)):
            return
        r = self.rigid(cx, cy, Ln / 2)
        seg = 10
        prof = [(math.cos(math.pi * k / seg) * W / 2, 0.9 + math.sin(math.pi * k / seg) * 2.1) for k in range(seg + 1)]
        c0, c1 = np.array([cx, cy]) - d * Ln / 2, np.array([cx, cy]) + d * Ln / 2
        for (v0, z0), (v1, z1) in zip(prof, prof[1:]):
            a0, a1 = c0 + nrm * v0, c0 + nrm * v1
            b0, b1 = c1 + nrm * v0, c1 + nrm * v1
            # outward: profile runs from +nrm over the top to -nrm / 外向き
            oquad(r, "kosen_poly", [(a0[0], a0[1], z0), (b0[0], b0[1], z0), (b1[0], b1[1], z1), (a1[0], a1[1], z1)],
                  (cx, cy, 0.5))
        for s_ in (-1, 1):
            for v in (-W / 2, W / 2):
                p = np.array([cx, cy]) + d * s_ * Ln / 2 + nrm * v
                q0 = (p[0], p[1], 0.0)
                q1 = (p[0] + d[0] * -s_ * Ln, p[1] + d[1] * -s_ * Ln, 0.0)
                if s_ < 0:
                    oquad(r, "kosen_poly", [q0, q1, (q1[0], q1[1], 0.9), (q0[0], q0[1], 0.9)], (cx, cy, 0.5))
            end = c0 if s_ < 0 else c1
            pts = [(end[0] + nrm[0] * v, end[1] + nrm[1] * v, z) for v, z in prof]
            pts = pts + [(end[0] - nrm[0] * W / 2, end[1] - nrm[1] * W / 2, 0.0),
                         (end[0] + nrm[0] * W / 2, end[1] + nrm[1] * W / 2, 0.0)]
            ctr = (end[0], end[1], 1.2)
            for i in range(len(pts)):
                tri = [ctr, pts[i], pts[(i + 1) % len(pts)]]
                nn = np.cross(np.subtract(tri[1], tri[0]), np.subtract(tri[2], tri[0]))
                if float(nn[:2] @ (d * s_)) < 0:
                    tri = [tri[0], tri[2], tri[1]]
                r.tris("kosen_poly", tri, np.r_[d * s_, 0.0], [(0.5, 0.5), (0, 0), (1, 0)], [0, 1, 2])
            dp = end + d * s_ * 0.02
            r.quad("jp_door_steel", *[(dp[0] + nrm[0] * v * s_, dp[1] + nrm[1] * v * s_, z) for v, z in
                                      ((-0.6, 0.0), (0.6, 0.0), (0.6, 2.0), (-0.6, 2.0))])
        for k in range(int(Ln // 1.5) + 1):
            p = c0 + d * min(Ln, k * 1.5)
            for (v0, z0), (v1, z1) in zip(prof, prof[1:]):
                a0, a1 = p + nrm * v0, p + nrm * v1
                r.box("kosen_steel", (a0[0] + a1[0]) / 2, (a0[1] + a1[1]) / 2, min(z0, z1), max(z0, z1) + 0.04,
                      0.04, 0.04)
        m.merge(r)

    def trees(self, m):
        """Trees where the imagery shows canopy (kosen_trace.json), kept off the
        buildings, courts, platforms of the courts, roads and paths (crowns too). / 樹木。"""
        S = self.S
        busy = unary_union([self.footprint.buffer(1.5), self.field.buffer(1.5), self.tennis.buffer(1.0),
                            self.carpark, self.lot, self.yard, unary_union(self.farms),
                            self.pool_deck.buffer(1.0), self.parterre.buffer(0.5)]
                           + [p.poly.buffer(0.3) for p in S.items if p.kind == "court"])
        shapely.prepare(busy)
        region = self.region()
        shapely.prepare(region)
        n = 0
        for (x, y), r in self.trees_seen:
            if not shapely.contains_xy(region, x, y) or shapely.contains_xy(busy, x, y):
                continue
            d = float(shapely.distance(self.no_go, Point(x, y)))
            r2 = min(r, d - TREE_CLEAR)
            if r2 < 1.4:
                continue
            # a seasonal Japanese tree (jtrees.py), leaves in their species' mesh
            # 季節の樹木（葉は樹種ごとのメッシュ）
            tm = self.rigid(x, y, 0.4)
            sp = jtrees.landmark_tree(tm, self.leaves, self.rigid, x, y, r2, "park")
            self.species[sp] = self.species.get(sp, 0) + 1
            m.merge(tm)
            n += 1
        # woods: wherever the tiles show canopy, densely (4.2 m jittered grid)
        # 林: 画像で樹冠のある所に密に植える
        wood = onterrain._clean(self.woods.intersection(region).difference(busy), 5.0)
        shapely.prepare(wood)
        x0, y0, x1, y1 = wood.bounds if not wood.is_empty else (0, 0, -1, -1)
        rs = np.random.default_rng(1963)
        planted = [(x, y) for (x, y), r in self.trees_seen]
        for y in np.arange(y0 + 2.1, y1, 4.2):
            for x in np.arange(x0 + 2.1, x1, 4.2):
                xx, yy = x + rs.uniform(-1.4, 1.4), y + rs.uniform(-1.4, 1.4)
                r = float(rs.uniform(2.3, 3.4))
                if not shapely.contains_xy(wood, xx, yy):
                    continue
                d = float(shapely.distance(self.no_go, Point(xx, yy)))
                r2 = min(r, d - TREE_CLEAR)
                if r2 < 1.6:
                    continue
                tm = self.rigid(xx, yy, 0.4)
                sp = jtrees.landmark_tree(tm, self.leaves, self.rigid, xx, yy, r2 * 1.05, "wood", lite=True)
                self.species[sp] = self.species.get(sp, 0) + 1
                m.merge(tm)
                n += 1
        self.wood_area = wood.area
        print("landmark %s: %d trees %s (woods %.0f m2)" % (TAG, n, dict(sorted(self.species.items())), wood.area))


def _solid(m, mat, bot, top):
    c = (np.sum(np.asarray(bot, np.float64), 0) + np.sum(np.asarray(top, np.float64), 0)) / 8.0
    oquad(m, mat, bot, c)
    oquad(m, mat, top, c)
    for i in range(4):
        j = (i + 1) % 4
        oquad(m, mat, [bot[i], bot[j], top[j], top[i]], c)


def split_tiles(mesh, tiles):
    """One Mesh per map tile (by triangle centre), keyed by the tile's bounds. / タイルごとに分ける。"""
    out = {}
    arrs = mesh.arrays()
    keys = [tb.bounds for tb in tiles]
    for mat, (V, N, UV, I) in arrs.items():
        C = V[I].mean(axis=1)
        idx = np.full(len(I), -1)
        for k, (x0, y0, x1, y1) in enumerate(keys):
            sel = (C[:, 0] >= x0) & (C[:, 0] < x1) & (C[:, 1] >= y0) & (C[:, 1] < y1) & (idx < 0)
            idx[sel] = k
        idx[idx < 0] = 0
        for k in np.unique(idx):
            sub = I[idx == k]
            used, inv = np.unique(sub.ravel(), return_inverse=True)
            mm = out.setdefault(keys[k], Mesh())
            mm.tris(mat, V[used], N[used], UV[used], inv.reshape(-1, 3), raw=True)
    return out


def write_textures(tex_dir):
    """The campus's drawn textures (procedural, from the colours in the Google
    Earth description; no imagery used): render, tiles, panels, roofs, sash
    windows, curtain wall, louvres, sand, infield, track, clay courts, pool,
    masonry, stone, the name plate. Each with roughness and metallic maps and a
    normal map from its own height. / キャンパスの描画テクスチャ。"""
    import textures as X
    from PIL import Image, ImageDraw, ImageFont
    out = {}
    rs = np.random.default_rng(1666)

    def noise(n, scales=(4, 16, 64), amps=(0.5, 0.3, 0.2)):
        return X.noise(n, scales, amps)

    def save(name, im, rough=0.85, metal=0.0, relief=0.0, height=None):
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

    def render(c, n=512, joints=0, streak=True):
        """Painted render with faint rain streaks and a darker base band (UV v = height, 3 m tile).
        塗装モルタル（雨だれ・裾の汚れ）。"""
        a = np.asarray(tinted(np.array(c) * 0.94, np.array(c) * 1.03, n, (8, 32, 128)), np.float64)
        if streak:
            for _ in range(18):
                x = int(rs.integers(0, n))
                w = int(rs.integers(2, 6))
                ln = int(rs.integers(n // 8, n // 3))
                y0 = int(rs.integers(0, n - ln))
                fade = np.linspace(0.0, 1.0, ln)[:, None, None] * 0.07
                a[n - (y0 + ln):n - y0, x:x + w] *= (1 - fade[::-1])
        a[int(n * 0.93):, :] *= 0.9                    # dirt at the bottom of each 3 m tile / 裾
        if joints:
            for k in range(1, joints):
                a[int(n * k / joints) - 1:int(n * k / joints) + 1, :] *= 0.85
        return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))

    save("kosen_render", render((228, 218, 198)), 0.88, 0.0, 0.6)
    save("kosen_render_grey", render((215, 215, 210)), 0.88, 0.0, 0.6)
    # the admin complex: salmon-beige render (user's photo of the forecourt, 2026-11-04)
    # 管理棟群: サーモンベージュ（利用者の写真）
    save("kosen_render_pink", render((212, 184, 168)), 0.88, 0.0, 0.6)
    # the entrance hall's perforated block screens: pink blocks, dark square holes,
    # 6 x 6 per 2 m tile (0.33 m pitch) / 玄関ホールの穴あきブロック
    sc = np.asarray(render((206, 178, 162), streak=False), np.float64)
    pitch = 512 // 6
    for i in range(6):
        for j in range(6):
            x0, y0 = i * pitch + pitch // 4, j * pitch + pitch // 4
            sc[y0:y0 + pitch // 2, x0:x0 + pitch // 2] = (62, 56, 52)
    save("kosen_screen", Image.fromarray(np.clip(sc, 0, 255).astype(np.uint8)), 0.9, 0.0, 1.0)
    save("kosen_concrete", tinted((138, 135, 124), (168, 164, 152), 256, (4, 16, 64)), 0.9, 0.0, 0.8)
    save("kosen_canopy_dark", tinted((88, 92, 88), (104, 108, 104), 128, (4, 16, 64)), 0.6, 0.2)
    save("kosen_spandrel", render((215, 205, 180), joints=2), 0.85, 0.0, 0.8)
    save("kosen_white", render((238, 236, 230), streak=False), 0.8, 0.0, 0.3)
    save("kosen_white_wall", render((232, 232, 228)), 0.85, 0.0, 0.4)
    save("kosen_soffit", render((205, 202, 195), streak=False), 0.9, 0.0, 0.2)
    save("kosen_floor", X.concrete(256, (150, 148, 140), (175, 172, 165), joints=4, jc=(120, 118, 110)), 0.9, 0.0, 0.6)
    save("kosen_plinth", X.concrete(256, (165, 163, 155), (190, 188, 180), joints=2), 0.9, 0.0, 0.6)

    def tiles(c, n=512, k=16, kv=None, grout=(200, 195, 185)):
        kv = kv or k * 2
        base = tinted(np.array(c) * 0.9, np.array(c) * 1.06, n, (n // 32, n // 8, n // 2))
        d = ImageDraw.Draw(base)
        for i in range(kv + 1):
            d.line([(0, i * n / kv), (n, i * n / kv)], fill=grout, width=2)
        for j in range(k + 1):
            for i in range(kv):
                x = (j + (0.5 if i % 2 else 0)) * n / k
                d.line([(x, i * n / kv), (x, (i + 1) * n / kv)], fill=grout, width=2)
        return base

    save("kosen_tile_brown", tiles((196, 118, 92), grout=(185, 160, 145)), 0.75, 0.0, 1.2)
    save("kosen_panel_grey", tiles((194, 196, 193), k=4, kv=4, grout=(160, 162, 160)), 0.6, 0.0, 0.8)
    save("kosen_masonry", tiles((185, 150, 115), k=6, kv=8, grout=(150, 125, 100)), 0.9, 0.0, 1.5)
    save("kosen_stone", tinted((140, 140, 136), (175, 175, 170), 256, (4, 16, 64)), 0.7, 0.0, 0.8)

    def corrugated(c, n=256, k=32, vertical=True):
        a = np.asarray(tinted(np.array(c) * 0.95, np.array(c) * 1.03, n, (4, 16, 64)), np.float64)
        wave = 0.9 + 0.1 * np.sin(np.linspace(0, 2 * np.pi * k, n, endpoint=False))
        a *= (wave[None, :, None] if vertical else wave[:, None, None])
        h = np.sin(np.linspace(0, 2 * np.pi * k, n, endpoint=False))
        h = np.tile(h[None, :], (n, 1)) if vertical else np.tile(h[:, None], (1, n))
        return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)), h

    # roofs: corrugated u across the slope -> ribs along the slope (v) / 屋根
    for name, c, rough, metal in (("kosen_roof_gym", (135, 180, 205), 0.45, 0.5),
                                  ("kosen_roof_stk", (210, 220, 213), 0.45, 0.5),
                                  ("kosen_roof_greymetal", (170, 170, 170), 0.5, 0.5),
                                  ("kosen_roof_blue", (140, 185, 215), 0.5, 0.4),
                                  ("kosen_roof_white", (225, 228, 228), 0.5, 0.3),
                                  ("kosen_roof_green", (95, 140, 105), 0.5, 0.3),
                                  ("kosen_roof_rust", (150, 110, 90), 0.6, 0.3)):
        im, h = corrugated(c)
        save(name, im, rough, metal, 1.5, h)
    for name, c in (("kosen_roof_mint", (92, 165, 146)), ("kosen_roof_teal", (76, 148, 164)),
                    ("kosen_roof_library", (88, 120, 130)), ("kosen_roof_dark", (92, 98, 102)),
                    ("kosen_roof_greygreen", (140, 160, 150)), ("kosen_roof_grey", (150, 152, 150))):
        # waterproof coating with seams and stains / 防水塗装（継ぎ目と汚れ）
        a = np.asarray(tinted(np.array(c) * 0.88, np.array(c) * 1.04, 512, (4, 16, 64)), np.float64)
        for k in range(0, 512, 128):
            a[k:k + 2, :] *= 0.92
        save(name, Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)), 0.8, 0.0, 0.5)
    save("kosen_roof_redtile", X.roof_tiles(512, (195, 150, 140)), 0.7, 0.0, 1.0)
    # gym walls: light grey metal siding, ribs vertical (u along the wall) / 体育館の金属外壁
    for name, c in (("kosen_wall_gym", (200, 205, 205)), ("kosen_wall_stk", (185, 190, 190))):
        im, h = corrugated(c, k=48)
        save(name, im, 0.5, 0.3, 0.8, h)

    def sash(panes, n=256, transoms=1):
        """Aluminium sash: silver frames, a transom, bluish glass with sky
        reflection and a curtain or two. / アルミサッシ（無目・反射・カーテン）。"""
        im = Image.new("RGB", (n, n), (185, 188, 190))
        d = ImageDraw.Draw(im)
        fw = max(3, n // 40)
        ty = int(n * 0.3)
        g0, g1 = np.array([70, 92, 108]), np.array([150, 172, 185])
        for i in range(panes):
            x0, x1 = int(i * n / panes) + fw, int((i + 1) * n / panes) - fw
            for y0, y1 in ((fw, ty - fw // 2), (ty + fw // 2, n - fw)):
                for yy in range(y0, y1):
                    t = (yy - y0) / max(1, y1 - y0)
                    col = tuple(int(v) for v in g1 * (1 - t) + g0 * t)
                    d.line([(x0, yy), (x1, yy)], fill=col)
            if rs.random() < 0.45:
                cx0 = x0 + int((x1 - x0) * rs.uniform(0.0, 0.4))
                d.rectangle([cx0, ty + fw, cx0 + int((x1 - x0) * 0.35), n - fw - 2], fill=(225, 222, 205))
        return im

    save("kosen_sash", sash(1), 0.15, 0.6)
    save("kosen_sash_2", sash(2), 0.15, 0.6)
    save("kosen_sash_3", sash(3), 0.15, 0.6)
    save("kosen_transom", sash(4), 0.15, 0.6)
    save("kosen_glassdoor", sash(2), 0.12, 0.6)
    cw = Image.new("RGB", (256, 256), (40, 52, 62))
    d = ImageDraw.Draw(cw)
    for k in range(0, 256, 64):
        d.rectangle([k, 0, k + 5, 255], fill=(235, 235, 232))
        d.rectangle([0, k, 255, k + 4], fill=(235, 235, 232))
    for yy in range(256):
        if yy % 64 > 5:
            t = (yy % 64) / 64.0
            d.line([(6, yy), (63, yy)], fill=(int(45 + 50 * (1 - t)), int(58 + 55 * (1 - t)), int(70 + 60 * (1 - t))))
    cw = Image.fromarray(np.tile(np.asarray(cw)[:, :64], (1, 4, 1)))
    save("kosen_curtain", cw, 0.08, 0.7)
    # tower 24's east curtain wall: light grey-blue glass, white grid, 4 x 4
    # panes per tile (photo) / 24 塔の明るいカーテンウォール
    cl = Image.new("RGB", (256, 256), (228, 230, 230))
    d = ImageDraw.Draw(cl)
    for i in range(4):
        for j in range(4):
            x0, y0 = i * 64 + 4, j * 64 + 4
            g = rs.uniform(-12, 12)
            for yy in range(y0, y0 + 56):
                t = (yy - y0) / 56.0
                d.line([(x0, yy), (x0 + 55, yy)],
                       fill=(int(165 + g - 25 * t), int(178 + g - 22 * t), int(182 + g - 18 * t)))
    save("kosen_curtain_pale", cl, 0.1, 0.6)
    lv = Image.new("RGB", (256, 256), (70, 82, 90))
    d = ImageDraw.Draw(lv)
    for yy in range(0, 256, 16):
        d.rectangle([0, yy, 255, yy + 5], fill=(200, 202, 200))
    for xx in range(0, 256, 64):
        d.rectangle([xx, 0, xx + 3, 255], fill=(170, 172, 170))
    save("kosen_louvre_glass", lv, 0.2, 0.5)
    save("kosen_louvre", X.stripes(256, (130, 135, 135), (205, 208, 205), 16), 0.5, 0.5, 0.8)
    save("kosen_steel", tinted((150, 152, 155), (175, 178, 180), 128, (4, 16, 64)), 0.4, 0.7)
    save("kosen_white_rail", tinted((232, 232, 230), (245, 245, 243), 128, (4, 16, 64)), 0.4, 0.3)
    save("kosen_net", tinted((40, 95, 60), (60, 120, 80), 64, (2, 8, 32)), 0.6, 0.0)
    save("kosen_rope", X.stripes(64, (230, 60, 50), (240, 240, 235), 8), 0.6, 0.0)
    # greenhouse film: milky white, faint hoop shadows / ビニールハウスのフィルム
    pf = tinted((205, 212, 210), (232, 236, 234), 256, (4, 16, 64))
    dd = ImageDraw.Draw(pf)
    for xx in range(0, 256, 32):
        dd.rectangle([xx, 0, xx + 2, 255], fill=(175, 182, 180))
    save("kosen_poly", pf, 0.3, 0.0)
    save("kosen_bark", tinted((70, 58, 45), (100, 85, 65), 128, (4, 16, 64)), 0.95, 0.0, 1.0)
    save("kosen_bronze", tinted((55, 48, 38), (95, 80, 60), 256, (4, 16, 64)), 0.45, 0.6, 0.8)
    save("kosen_letter_blue", tinted((25, 55, 135), (35, 70, 155), 64, (2, 8, 32)), 0.5, 0.0)
    # ground / 地面
    save("kosen_asphalt", tinted((88, 88, 90), (118, 118, 120), 512, (16, 64, 256)), 0.88, 0.0, 0.6)
    save("kosen_paving", tiles((195, 175, 150), n=512, k=8, kv=16, grout=(160, 145, 125)), 0.85, 0.0, 1.0)
    save("kosen_sand", tinted((158, 132, 100), (182, 156, 122), 512, (16, 64, 256)), 0.95, 0.0, 1.2)
    save("kosen_infield", tinted((98, 84, 76), (122, 104, 94), 512, (16, 64, 256)), 0.95, 0.0, 1.2)
    save("kosen_clay", tinted((165, 120, 90), (185, 140, 108), 512, (16, 64, 256)), 0.95, 0.0, 1.0)
    save("kosen_deck", tinted((90, 92, 94), (115, 117, 119), 256, (8, 32, 128)), 0.85, 0.0, 0.6)
    save("kosen_turf", tinted((35, 75, 40), (55, 100, 55), 256, (8, 32, 128)), 0.95, 0.0, 1.0)
    save("kosen_pool_water", tinted((95, 165, 180), (125, 190, 200), 256, (4, 16, 64)), 0.05, 0.0, 0.4)
    # the running straight: blue rubber, white lane lines along u / 走路（青いゴム、白線）
    tr = tinted((120, 140, 175), (145, 165, 195), 256, (8, 32, 128))
    d = ImageDraw.Draw(tr)
    for k in range(5):
        y = int(k * 255 / 4)
        d.rectangle([0, max(0, y - 2), 255, min(255, y + 2)], fill=(238, 238, 235))
    save("kosen_track", tr, 0.85, 0.0, 1.0)
    # a clay tennis court with white lines, 17 x 35 m incl. run-off / クレーコート
    W, H = 340, 700
    k = W / 17.0
    im = tinted((165, 120, 90), (185, 140, 108), 512, (16, 64, 256)).resize((W, H))
    d = ImageDraw.Draw(im)
    lw = max(2, int(0.05 * k))
    cx, cy = W / 2, H / 2
    hw, hl, sw = 10.97 / 2 * k, 23.77 / 2 * k, 8.23 / 2 * k
    d.rectangle([cx - hw, cy - hl, cx + hw, cy + hl], outline=(240, 238, 230), width=lw)
    for s in (-1, 1):
        d.line([(cx + s * sw, cy - hl), (cx + s * sw, cy + hl)], fill=(240, 238, 230), width=lw)
        d.line([(cx - sw, cy + s * 6.4 * k), (cx + sw, cy + s * 6.4 * k)], fill=(240, 238, 230), width=lw)
    d.line([(cx, cy - 6.4 * k), (cx, cy + 6.4 * k)], fill=(240, 238, 230), width=lw)
    save("kosen_tennis", im, 0.95, 0.0, 0.8)
    # the name plate: 大分工業高等専門学校 on polished stone / 校名の銘板
    W, H = 1024, 256
    im = Image.new("RGB", (W, H), (90, 92, 92))
    d = ImageDraw.Draw(im)
    try:
        size = 120
        f = ImageFont.truetype(CJK, size)
        while d.textlength("大分工業高等専門学校", font=f) > W * 0.9:
            size -= 4
            f = ImageFont.truetype(CJK, size)
        tw = d.textlength("大分工業高等専門学校", font=f)
        d.text(((W - tw) / 2, (H - size * 1.2) / 2), "大分工業高等専門学校", fill=(235, 230, 215), font=f)
    except OSError:
        pass
    save("kosen_plate", im, 0.3, 0.0)
    return out
