"""Courts, pitches, school fences, playgrounds and direction boards (maps/.venv).

From the OSM sports pitches and what was traced on satellite imagery
(sitefeatures.py):
  courts     basketball (blue, green or red, as seen), tennis, multi-use;
             each a surface with its own line-marking texture laid once
             over the court, hoops / net posts
  pitches    artificial-turf football pitches (traced) and grass fields
             (OSM soccer), with line markings and goals
  parade     school parade squares (red), bare earth fields
  fences     green steel railings round every school site, with gaps where
             a footway or driveway enters (gates)
  playground Park Aquaria's ship playground on round rubber pads, and its
             roofed basketball court
  boards     Singapore direction boards (green, white text and arrows) on
             main-road approaches to signalised junctions, naming the street
             each turn leads into (from the road logic's connections), on
             the kerb side 50 m before the stop line

Nothing here is placed on a carriageway, a kerb or a building.

コート・グラウンド・学校の柵・遊具・方面案内板。
"""

import math
import os

import numpy as np
import shapely
import shapely.errors
from shapely.affinity import rotate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from mesh import Mesh, polys

COURT_SIZE = {"basketball": (28.0, 15.0), "tennis": (23.8, 11.0), "badminton": (13.4, 6.1)}


def rect(cx, cy, length, width, axis):
    return rotate(box(cx - length / 2, cy - width / 2, cx + length / 2, cy + width / 2), axis,
                  origin=(cx, cy), use_radians=True)


class Extras(object):
    def __init__(self, L, sites, lanes, approaches):
        self.L = L
        self.sites = sites
        self.drape = None         # terrain.Drape on hilly maps (set by build.py) / 起伏のある地図のみ
        self.courts = []          # dict(kind, mat, cx, cy, length, width, axis, poly, hoops) / コート
        self.fences = []
        self.boards = []          # dict(x, y, yaw, lines, tex) / 案内板
        self.ships = []
        self.roofs = []
        self.lanes = lanes
        self.approaches = approaches
        self.collect_courts()
        self.collect_fences()
        self.collect_playgrounds()
        self.collect_boards()

    # -- courts and pitches / コート・グラウンド
    def free(self, poly, slack=0.3):
        L = self.L
        return not poly.intersects(L.road_all.buffer(0.5)) and \
            poly.intersection(L.building_area).area < slack * poly.area

    def add_court(self, kind, mat, cx, cy, length, width, axis, hoops=None, src="osm"):
        # a traced outline a little too big for its spot shrinks until it fits
        # (tracing error is a few metres) / 少し大きすぎる輪郭は収まるまで縮める
        for k in range(5):
            poly = rect(cx, cy, length, width, axis)
            if self.free(poly):
                break
            length, width = length * 0.93, width * 0.93
        if not self.L.inside.contains(poly.centroid) or not self.free(poly):
            return False
        if any(c["poly"].intersection(poly).area > 0.3 * min(c["poly"].area, poly.area) for c in self.courts):
            return False
        self.courts.append(dict(kind=kind, mat=mat, cx=cx, cy=cy, length=length, width=width, axis=axis,
                                poly=poly, hoops=hoops if hoops is not None else kind, src=src))
        return True

    def collect_courts(self):
        S = self.sites
        if S:
            for f in S.of_type("court_basketball", "pitch_astro", "parade_square", "field_dirt", "sheltered_court"):
                x, y = f["xy"]
                kind = {"court_basketball": "basketball", "pitch_astro": "football", "parade_square": "parade",
                        "field_dirt": "dirt", "sheltered_court": "basketball"}[f["type"]]
                colour = (f.get("colour") or "court_blue").replace("court_", "")
                mat = {"basketball": "court_bb_" + colour, "football": "pitch_astro", "parade": "parade_red",
                       "dirt": "soil"}[kind]
                length, width = f["length"], f["width"]
                if kind == "basketball":
                    length, width = max(length, 26.0), max(width, 14.0)
                self.add_court(kind, mat, x, y, length, width, f["axis"], src="traced")
        # OSM pitches by sport, where nothing traced covers them / 未トレースの OSM コート
        for p, t in self.L.osm.areas(lambda t: t.get("leisure") == "pitch"):
            r = p.minimum_rotated_rectangle
            c = list(r.exterior.coords)
            e0, e1 = math.dist(c[0], c[1]), math.dist(c[1], c[2])
            axis = math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]) if e0 >= e1 else \
                math.atan2(c[2][1] - c[1][1], c[2][0] - c[1][0])
            length, width = max(e0, e1), min(e0, e1)
            cx, cy = r.centroid.x, r.centroid.y
            sport = t.get("sport", "multi").split(";")[0]
            if sport == "soccer":
                self.add_court("football", "pitch_grass" if length * width > 2500 else "pitch_astro",
                               cx, cy, length, width, axis)
            elif sport == "basketball":
                self.add_court("basketball", "court_bb_blue", cx, cy, max(length, 26), max(width, 14), axis)
            elif sport == "tennis":
                self.add_court("tennis", "court_tennis", cx, cy, max(length, 34), max(width, 16), axis)
            else:
                self.add_court("multi", "court_multi", cx, cy, max(length, 15), max(width, 8), axis, hoops="none")

    def court_area(self):
        return unary_union([c["poly"] for c in self.courts]) if self.courts else Polygon()

    def apply_to_layout(self):
        """Take the courts out of every other ground surface. / 他の地面からコートを除く。"""
        from layout import clean
        L, area = self.L, self.court_area().buffer(0.05)
        if area.is_empty:
            return
        for name in ("parking", "driveways", "footpaths", "cycleways", "aprons", "parks", "forest", "pitches",
                     "playgrounds", "grass"):
            setattr(L, name, clean(getattr(L, name).difference(area), 0.05))

    # -- school fences / 学校の柵
    def collect_fences(self):
        L = self.L

        def union(gs):
            # GEOS "side location conflict" on near-coincident valid edges (OitaStation j104,
            # 2026-11-09): the same union on a 1 mm grid / 1 mm グリッドで再試行
            try:
                return unary_union(gs)
            except shapely.errors.GEOSException:
                return shapely.union_all([shapely.make_valid(g) for g in gs], grid_size=0.001)
        gaps = union([L.driveways, L.parking, L.footpaths.buffer(0.2), L.cycleways]).buffer(0.3)
        keepout = union([L.road_all.buffer(0.6), L.building_area.buffer(0.4), self.court_area()])
        for p, t in L.osm.areas(lambda t: t.get("amenity") in ("school", "kindergarten")):
            ring = p.buffer(-0.6, join_style="mitre")
            for q in polys(ring):
                line = q.exterior.intersection(L.inside).difference(keepout).difference(gaps)
                for g in getattr(line, "geoms", [line]):
                    if g.geom_type == "LineString" and g.length > 2.0:
                        self.fences.append(g)

    # -- playgrounds / 遊び場
    def collect_playgrounds(self):
        S = self.sites
        if not S:
            return
        for f in S.of_type("ship_playground", "play_pad"):
            x, y = f["xy"]
            r = f["poly"].area ** 0.5 / math.sqrt(math.pi)
            pad = Point(x, y).buffer(r, quad_segs=8)
            if not self.free(pad, 0.05):
                continue
            self.courts.append(dict(kind="pad", mat="play_rubber", cx=x, cy=y, length=2 * r, width=2 * r, axis=0.0,
                                    poly=pad, hoops="none", src="traced"))
            if f["type"] == "ship_playground":
                self.ships.append((x, y, r))
        for f in S.of_type("sheltered_court"):
            if any(c["src"] == "traced" and c["poly"].contains(Point(*f["xy"])) for c in self.courts):
                self.roofs.append(f)

    # -- direction boards / 方面案内板
    def collect_boards(self):
        L = self.L
        sig = {lt["road"] for lt in L.lights}
        lane_by = {(l["road"], l["lane"]): l for l in self.lanes if l["junction"] < 0}
        by_road = {}
        for a in self.approaches:
            by_road.setdefault(a["road"], []).append(a)
        for road, al in by_road.items():
            if road not in sig or len(al) < 2:
                continue
            kerb = lane_by.get((road, -1))
            if kerb is None or (kerb["speed"] or 0) < 13.8:
                continue
            here = (kerb["name"] or "").strip()
            exits = {}
            for a in al:
                for turn, name in a.get("exits", {}).items():
                    if turn in ("left", "straight", "right") and name:
                        exits.setdefault(turn, name)
            lines = [(t, exits[t]) for t in ("left", "straight", "right") if t in exits]
            if len(lines) < 2 or all(n == here for _, n in lines):
                continue
            pts = kerb["pts"]
            ln = LineString([p["l"] for p in pts])
            if ln.length < 30.0:
                continue
            s = max(ln.length - 50.0, ln.length * 0.4)
            q = ln.interpolate(s)
            q2 = ln.interpolate(min(ln.length, s + 1.0))
            yaw = math.atan2(q2.y - q.y, q2.x - q.x)
            # out beyond the kerb on the kerb (left) side / 縁石の外（左側）
            nx, ny = -math.sin(yaw), math.cos(yaw)
            x, y = q.x + nx * 1.6, q.y + ny * 1.6
            foot = Point(x, y).buffer(1.8)
            if foot.intersects(L.carriageway) or foot.intersects(L.building_area):
                x, y = q.x + nx * 2.4, q.y + ny * 2.4
                if Point(x, y).buffer(1.8).intersects(L.carriageway):
                    continue
            if any(math.hypot(b["x"] - x, b["y"] - y) < 25 for b in self.boards):
                continue
            self.boards.append(dict(x=x, y=y, yaw=yaw, lines=lines, tex="dirboard_%02d" % len(self.boards)))

    def write_board_textures(self, tex_dir):
        """One texture per board: green, white border, arrow and street per line.
        案内板ごとのテクスチャ（緑地に白字・矢印）。"""
        from PIL import Image, ImageDraw, ImageFont
        font = None
        for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                     "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"):
            if os.path.exists(path):
                font = ImageFont.truetype(path, 44)
                break
        for b in self.boards:
            W, H = 1024, 120 + 110 * len(b["lines"])
            im = Image.new("RGB", (W, H), (0, 116, 74))
            d = ImageDraw.Draw(im)
            d.rectangle([8, 8, W - 9, H - 9], outline=(255, 255, 255), width=8)
            y = 60
            for turn, name in b["lines"]:
                cx, cy = 90, y + 40
                if turn == "straight":
                    d.polygon([(cx, cy - 38), (cx - 26, cy - 8), (cx - 10, cy - 8), (cx - 10, cy + 36),
                               (cx + 10, cy + 36), (cx + 10, cy - 8), (cx + 26, cy - 8)], fill="white")
                else:
                    s = -1 if turn == "left" else 1
                    d.polygon([(cx + s * 40, cy), (cx + s * 8, cy - 26), (cx + s * 8, cy - 10), (cx - s * 36, cy - 10),
                               (cx - s * 36, cy + 10), (cx + s * 8, cy + 10), (cx + s * 8, cy + 26)], fill="white")
                d.text((170, y + 12), name, fill="white", font=font)
                y += 110
            im.save(os.path.join(tex_dir, b["tex"] + ".png"))
        return {b["tex"]: (b["tex"], None, 0.4, 0.0, False) for b in self.boards}

    # -- meshes / メッシュ
    def ground_mesh(self, tile, z):
        m = Mesh()
        if self.drape is not None:
            m.zfun, m.dense = self.drape.z, 2.5          # courts lie on the ground / 地面に沿う
        for c in self.courts:
            if not tile.contains(Point(c["cx"], c["cy"])):
                continue
            if c["kind"] == "pad":
                m.flat(c["mat"], c["poly"], z + 0.03, 2.0)
                continue
            ca, sa = math.cos(c["axis"]), math.sin(c["axis"])
            hl, hw = c["length"] / 2, c["width"] / 2
            P = lambda u, v: (c["cx"] + ca * u - sa * v, c["cy"] + sa * u + ca * v, z + 0.02)
            # the court's own UVs: its markings laid once over it / コート全体に 1 枚
            m.quad(c["mat"], P(-hl, -hw), P(hl, -hw), P(hl, hw), P(-hl, hw), (0, 0), (1, 0), (1, 1), (0, 1))
        return m

    def props_mesh(self, tile):
        m = Mesh()
        D = self.drape
        # hilly maps: posts, nets, fences, boards follow the ground vertex by
        # vertex (vertical edges stay vertical); a ship is moved whole
        # 起伏のある地図: 柱・網・柵・案内板は頂点ごとに地面に沿わせ、船の遊具は剛体移動
        if D is not None:
            m.zfun, m.dense = D.z, 2.5
        x0, y0, x1, y1 = tile.bounds
        inside = lambda x, y: x0 <= x < x1 and y0 <= y < y1
        for c in self.courts:
            if not inside(c["cx"], c["cy"]):
                continue
            ca, sa = math.cos(c["axis"]), math.sin(c["axis"])
            at = lambda u, v: (c["cx"] + ca * u - sa * v, c["cy"] + sa * u + ca * v)
            if c["hoops"] == "basketball":
                for s in (-1, 1):
                    px, py = at(s * (c["length"] / 2 + 0.6), 0)
                    m.cylinder("metal_grey", px, py, 0.15, 3.05, 0.08, seg=6, top=False)
                    bx, by = at(s * (c["length"] / 2 - 0.9), 0)
                    m.box("backboard", bx, by, 2.9, 3.95, 0.05, 1.8, c["axis"])
            elif c["kind"] == "football":
                for s in (-1, 1):
                    gx, gy = at(s * c["length"] / 2, 0)
                    w = 7.3 if c["length"] > 60 else 3.0
                    h = 2.44 if c["length"] > 60 else 2.0
                    for e in (-1, 1):
                        px, py = at(s * c["length"] / 2, e * w / 2)
                        m.cylinder("goal_white", px, py, 0.15, 0.15 + h, 0.06, seg=6, top=False)
                    m.box("goal_white", gx, gy, 0.15 + h - 0.06, 0.15 + h + 0.06, 0.12, w, c["axis"])
            elif c["kind"] == "tennis":
                for e in (-1, 1):
                    px, py = at(0, e * 6.4)
                    m.cylinder("metal_grey", px, py, 0.15, 1.22, 0.04, seg=6, top=True)
                p0, p1 = at(0, -6.4), at(0, 6.4)
                m.wall("net_dark", p0, p1, 0.15, 1.07, 0.0, 1.0)
                m.wall("net_dark", p1, p0, 0.15, 1.07, 0.0, 1.0)
        for g in self.fences:
            mid = g.interpolate(0.5, normalized=True)
            if inside(mid.x, mid.y):
                self.m_fence(m, g)
        for x, y, r in self.ships:
            if inside(x, y):
                if D is not None:
                    m.zfun, m.dense, m.zoff = None, None, D.low(x, y, r)
                self.m_ship(m, x, y, r)
                if D is not None:
                    m.zfun, m.dense, m.zoff = D.z, 2.5, 0.0
        for f in self.roofs:
            if inside(*f["xy"]):
                self.m_shelter_roof(m, f)
        for b in self.boards:
            if inside(b["x"], b["y"]):
                self.m_board(m, b)
        return m

    def m_fence(self, m, line, h=1.8):
        """Green steel railing: posts, two rails, flat bars every 0.3 m. / 緑の鋼製フェンス。"""
        coords = list(line.coords)
        for a, b in zip(coords, coords[1:]):
            m.wall("fence_green", a, b, h - 0.08, h, 0.0, 1.0)
            m.wall("fence_green", b, a, h - 0.08, h, 0.0, 1.0)
            m.wall("fence_green", a, b, 0.25, 0.33, 0.0, 1.0)
            m.wall("fence_green", b, a, 0.25, 0.33, 0.0, 1.0)
        n = int(line.length // 0.3)
        for k in range(n + 1):
            p = line.interpolate(min(line.length, k * 0.3))
            q = line.interpolate(min(line.length, k * 0.3 + 0.02))
            d = math.hypot(q.x - p.x, q.y - p.y) or 1.0
            tx, ty = (q.x - p.x) / d * 0.02, (q.y - p.y) / d * 0.02
            a, b = (p.x - tx, p.y - ty), (p.x + tx, p.y + ty)
            m.wall("fence_green", a, b, 0.15, h, 0.0, 1.0)
            m.wall("fence_green", b, a, 0.15, h, 0.0, 1.0)
            if k % 8 == 0:
                m.box("fence_green", p.x, p.y, 0.15, h + 0.05, 0.08, 0.08)

    def m_ship(self, m, x, y, r):
        """A ship-shaped play structure: hull, deck, a tower with a pointed roof,
        a slide and a mast. / 船の形の遊具。"""
        L, W = r * 1.5, r * 0.7
        hull = Polygon([(-L / 2, -W / 2), (L / 3, -W / 2), (L / 2, 0), (L / 3, W / 2), (-L / 2, W / 2)])
        hull = rotate(hull, 30, origin=(0, 0)).buffer(0)
        from shapely.affinity import translate
        hull = translate(hull, x, y)
        m.prism("hull_white", hull, 0.18, 1.2, top=False)
        m.flat("deck_wood", hull.buffer(-0.12), 1.0, 1.0)
        m.ring_walls("hull_red", hull.exterior, 0.18, 0.5, outward=True)
        m.box("deck_wood", x, y, 1.0, 3.0, 1.6, 1.6, math.radians(30), top=True)
        m.cylinder("roof_blue", x, y, 3.0, 4.6, 1.2, seg=6, top=False, r1=0.05)
        m.cylinder("metal_grey", x - 0.3 * L * math.cos(math.radians(30)), y - 0.3 * L * math.sin(math.radians(30)),
                   1.0, 5.5, 0.08, seg=6, top=True)
        sx, sy = x + 1.6 * math.cos(math.radians(120)), y + 1.6 * math.sin(math.radians(120))
        ang = math.radians(120)
        P = lambda u, v, z: (sx + math.cos(ang) * u - math.sin(ang) * v, sy + math.sin(ang) * u + math.cos(ang) * v, z)
        m.quad("slide_yellow", P(0, -0.35, 2.0), P(0, 0.35, 2.0), P(3.0, 0.35, 0.3), P(3.0, -0.35, 0.3))
        m.quad("slide_yellow", P(3.0, -0.35, 0.28), P(3.0, 0.35, 0.28), P(0, 0.35, 1.98), P(0, -0.35, 1.98))

    def m_shelter_roof(self, m, f):
        """The roof over Park Aquaria's basketball court, on columns. / 屋根付きコートの屋根。"""
        x, y = f["xy"]
        p = f["poly"]
        m.flat("roof_metal", p, 7.0, 3.0)
        m.flat("roof_metal", p, 6.8, 3.0, up=False)
        for q in list(p.exterior.coords)[:-1]:
            cx, cy = x + (q[0] - x) * 0.95, y + (q[1] - y) * 0.95
            m.box("metal_grey", cx, cy, 0.15, 6.8, 0.35, 0.35)

    def m_board(self, m, b):
        """Direction board on two posts, facing the oncoming traffic. / 2 本柱の案内板。"""
        yaw = b["yaw"]
        rows = len(b["lines"])
        W, H = 3.6, 0.42 + 0.39 * rows
        tx, ty = -math.sin(yaw), math.cos(yaw)          # across the road (left) / 道路を横切る向き
        fx, fy = math.cos(yaw), math.sin(yaw)           # traffic direction / 進行方向
        z0 = 2.3
        a = (b["x"] - tx * W / 2, b["y"] - ty * W / 2)
        c = (b["x"] + tx * W / 2, b["y"] + ty * W / 2)
        # front face toward the drivers (they come from -f): reading left to right from the road side
        # 前面は運転者側（-f）へ
        front = lambda p: (p[0] - fx * 0.04, p[1] - fy * 0.04)
        fa, fc = front(c), front(a)
        m.quad(b["tex"], (fa[0], fa[1], z0), (fc[0], fc[1], z0), (fc[0], fc[1], z0 + H),
               (fa[0], fa[1], z0 + H), (0, 0), (1, 0), (1, 1), (0, 1))
        m.wall("metal_grey", a, c, z0, z0 + H, 0.0, 1.0)
        for e in (-0.42, 0.42):
            px, py = b["x"] + tx * W * e, b["y"] + ty * W * e
            m.box("metal_grey", px + fx * 0.05, py + fy * 0.05, 0.15, z0 + H, 0.1, 0.1)
