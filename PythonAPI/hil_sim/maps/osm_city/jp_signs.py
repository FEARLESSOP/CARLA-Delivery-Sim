"""Japanese sign plates CARLA has no model for, as posts in the map (maps/.venv).

  python jp_signs.py <osm> <out_dir>      (after build.py and signs.py)

Only for maps with maps.json (or the build's meta.json) "jartic_d" (police
regulations, JARTIC type D; guide ch. 18); every other map is left untouched.
CARLA spawns its own speed (274) and give-way (205) signs from the xodr; its
stop sign is the octagonal US STOP, so on these maps signs.py names the 206
signals "Stencil_STOP" (CARLA spawns no model, the traffic manager still
stops) and this file stands the Japanese 止まれ there. Plates, drawn from the
standard designs of 道路標識令 (never photos):

  止まれ (330-A)             every stop in crossings.pkl (JARTIC 一時停止 and the
                             painted 止まれ), before the stop line
  指定方向外進行禁止 (311)   JARTIC points: arrows from the allowed directions,
                             an auxiliary plate with the hours when limited
  横断歩道 (407-A)           both ends of every unsignalised crossing, two-faced
  一方通行 (326-A)           JARTIC one-way lines, at the start of each road,
                             along the road (two-faced, arrow with the traffic)
  駐車禁止 (316)             JARTIC no-parking lines: each road's start, then
                             every 150 m
  自転車歩道通行可 (325-3)   JARTIC lines: each road's start

White 60.5 mm poles, plates 0.6 m (止まれ 0.8 m), bottom edge 1.85 m over
the pavement, the post 0.6 m behind the carriageway edge (0.4 m behind the
kerb) on the kerb side (keep-left: the left of travel), plate facing the
traffic it is for. Posts within 2.5 m facing the same way share one pole
(plates stacked). Objects Terrain_JpPlates_<Name>_<tile>, materials
jp_plate_* (CARLA's importer drops names with "sign" or "light"). Writes
build/<Name>/jp_plates.json (every plate, for checks).

日本の標識板（CARLA に無いもの）を支柱付きで地図に置く。JARTIC 規制データの地図のみ。
"""

import collections
import json
import math
import os
import pickle
import sys

import numpy as np
from shapely.geometry import Point

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jartic  # noqa: E402
from mesh import Mesh  # noqa: E402

POLE_R = 0.03025           # 60.5 mm white steel pole / 白色支柱 φ60.5
BACK = 0.6                 # post behind the carriageway edge (gutter 0.4 + kerb 0.2 + 0.4) / 車道端からの距離
BOTTOM = 1.85              # lowest plate edge over the pavement (1.8 m min) / 板下端の高さ
GAP = 0.05
LANE_CAP = 2.0             # lane ends extended this far for the clearance test (carla clamps s) / 端の延長
LANE_CLEAR = 0.3           # every post this far from any xodr driving lane (junction lanes too) / 全車線からの離れ
PLATE_T = 0.012
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
# shape, width, height (m) / 形・幅・高さ
SIZES = {"tomare": ("tri", 0.8, 0.8 * math.sqrt(3) / 2), "round": ("disc", 0.6, 0.6),
         "crossing": ("rect", 0.6, 0.6), "oneway": ("rect", 0.8, 0.27), "aux": ("rect", 0.5, 0.2)}
RED, BLUE, WHITE, BLACK = (200, 22, 30), (0, 82, 160), (250, 250, 250), (20, 20, 20)


# -- textures (standard designs of 道路標識令, drawn) / 標識の図柄 ----------------------
def _font(px):
    from PIL import ImageFont
    return ImageFont.truetype(FONT, px)


def _arrow(d, pts, w, head, fill):
    """Polyline arrow with a head at the last point. / 矢印。"""
    d.line(pts, fill=fill, width=w, joint="curve")
    (x0, y0), (x1, y1) = pts[-2], pts[-1]
    a = math.atan2(y1 - y0, x1 - x0)
    ca, sa = math.cos(a), math.sin(a)
    tip = (x1 + ca * head, y1 + sa * head)
    d.polygon([tip, (x1 - sa * head, y1 + ca * head), (x1 + sa * head, y1 - ca * head)], fill=fill)


def texture(kind, S=1024):
    """One plate face, S x S (the mesh maps its outline onto the square). / 板面 1 枚。"""
    from PIL import Image, ImageDraw
    im = Image.new("RGB", (S, S), (128, 128, 128))
    d = ImageDraw.Draw(im)
    c, R = S / 2, S / 2
    if kind == "tomare":                                   # 330-A 一時停止
        im.paste(WHITE, (0, 0, S, S))
        h = S * math.sqrt(3) / 2
        # the mesh maps the triangle's box: top edge at v=1, apex at v=0 / 上辺 v=1・頂点 v=0
        top, bot = 0, S
        b = S * 0.06
        d.polygon([(b * 1.7, top + b), (S - b * 1.7, top + b), (c, bot - b * 2.0)], fill=RED)
        f = _font(int(S * 0.15))
        d.text((c, S * 0.3), "止まれ", fill=WHITE, font=f, anchor="mm")
        del h
    elif kind.startswith("noturn_") or kind in ("noparking", "bicycle"):
        im.paste(WHITE, (0, 0, S, S))
        if kind == "noparking":                            # 316 駐車禁止
            d.ellipse([S * 0.02, S * 0.02, S * 0.98, S * 0.98], fill=RED)
            d.ellipse([S * 0.13, S * 0.13, S * 0.87, S * 0.87], fill=BLUE)
            k = S * 0.33
            d.line([(c - k * 0.95, c - k * 0.95), (c + k * 0.95, c + k * 0.95)], fill=RED, width=int(S * 0.1))
        else:
            d.ellipse([S * 0.03, S * 0.03, S * 0.97, S * 0.97], fill=BLUE)
            if kind == "bicycle":                          # 325-3 普通自転車歩道通行可
                w = int(S * 0.035)
                for x in (c - S * 0.17, c + S * 0.17):
                    d.ellipse([x - S * 0.12, c + S * 0.02, x + S * 0.12, c + S * 0.26], outline=WHITE, width=w)
                fr = [(c - S * 0.17, c + S * 0.14), (c - S * 0.03, c - S * 0.04), (c + S * 0.13, c - S * 0.04),
                      (c + S * 0.17, c + S * 0.14)]
                d.line(fr, fill=WHITE, width=w)
                d.line([(c - S * 0.17, c + S * 0.14), (c + S * 0.03, c + S * 0.14), (c - S * 0.03, c - S * 0.04)],
                       fill=WHITE, width=w)
                d.line([(c + S * 0.03, c + S * 0.14), (c + S * 0.13, c - S * 0.04)], fill=WHITE, width=w)
                # the rider / 人
                d.ellipse([c - S * 0.05, c - S * 0.34, c + S * 0.05, c - S * 0.24], fill=WHITE)
                d.line([(c, c - S * 0.23), (c - S * 0.03, c - S * 0.06), (c + S * 0.03, c + S * 0.12)],
                       fill=WHITE, width=int(S * 0.05))
                d.line([(c, c - S * 0.18), (c + S * 0.12, c - S * 0.08)], fill=WHITE, width=int(S * 0.04))
            else:                                          # 311 指定方向外進行禁止
                turns = kind.split("_")[1]
                w, hd = int(S * 0.09), S * 0.11
                base, mid = (c, S * 0.8), (c, c + S * 0.02)
                if turns == "s":
                    _arrow(d, [base, (c, S * 0.27)], w, hd, WHITE)
                if "l" in turns and "s" in turns:
                    _arrow(d, [base, (c, S * 0.27)], w, hd, WHITE)
                    _arrow(d, [mid, (S * 0.27, mid[1])], w, hd, WHITE)
                if "r" in turns and "s" in turns:
                    _arrow(d, [base, (c, S * 0.27)], w, hd, WHITE)
                    _arrow(d, [mid, (S * 0.73, mid[1])], w, hd, WHITE)
                if turns in ("l", "lr"):
                    _arrow(d, [base, mid, (S * 0.27, mid[1])], w, hd, WHITE)
                if turns in ("r", "lr"):
                    _arrow(d, [base, mid, (S * 0.73, mid[1])], w, hd, WHITE)
    elif kind == "crossing":                               # 407-A 横断歩道
        im.paste(BLUE, (0, 0, S, S))
        d.rectangle([S * 0.03, S * 0.03, S * 0.97, S * 0.97], outline=WHITE, width=int(S * 0.02))
        d.polygon([(c, S * 0.1), (S * 0.08, S * 0.88), (S * 0.92, S * 0.88)], fill=WHITE)
        for i in range(5):                                 # zebra / しま模様
            x = S * (0.3 + i * 0.09)
            d.polygon([(x, S * 0.84), (x + S * 0.05, S * 0.84), (x + S * 0.03, S * 0.77), (x - S * 0.02, S * 0.77)],
                      fill=BLACK)
        w = int(S * 0.045)
        d.ellipse([c - S * 0.05, S * 0.33, c + S * 0.05, S * 0.43], fill=BLACK)
        d.line([(c, S * 0.44), (c - S * 0.01, S * 0.6)], fill=BLACK, width=w)
        d.line([(c - S * 0.01, S * 0.6), (c - S * 0.09, S * 0.74)], fill=BLACK, width=w)
        d.line([(c - S * 0.01, S * 0.6), (c + S * 0.08, S * 0.74)], fill=BLACK, width=w)
        d.line([(c - S * 0.08, S * 0.56), (c, S * 0.47), (c + S * 0.09, S * 0.55)], fill=BLACK, width=int(S * 0.035))
    elif kind.startswith("oneway") or kind.startswith("aux_"):
        # drawn at the plate's own aspect, then squeezed into the square the UVs span
        # 板の縦横比で描いてから正方形へ（UV が正方形を張るため）
        shape, w, h = size_of(kind)
        W, H = S, int(S * h / w)
        im = Image.new("RGB", (W, H), BLUE if kind.startswith("oneway") else WHITE)
        d = ImageDraw.Draw(im)
        if kind.startswith("oneway"):                      # 326-A 一方通行
            d.rectangle([H * 0.04, H * 0.04, W - H * 0.04, H * 0.96], outline=WHITE, width=int(H * 0.04))
            y = H * 0.68
            pts = [(W * 0.08, y), (W * 0.8, y)]
            if kind.endswith("_l"):
                pts = [(W * 0.92, y), (W * 0.2, y)]
            _arrow(d, pts, int(H * 0.16), H * 0.2, WHITE)
            d.text((W / 2, H * 0.3), "一 方 通 行", fill=WHITE, font=_font(int(H * 0.3)), anchor="mm")
        else:                                              # 補助標識 (hours) / 時間
            d.rectangle([H * 0.04, H * 0.04, W - H * 0.04, H * 0.96], outline=BLACK, width=int(H * 0.04))
            t = kind[4:]
            txt = "%d:%s-%d:%s" % (int(t[:2]), t[2:4], int(t[5:7]), t[7:9])
            d.text((W / 2, H / 2), txt, fill=BLACK, font=_font(int(H * 0.5)), anchor="mm")
        im = im.resize((S, S), Image.LANCZOS)
    elif kind == "back":
        rng = np.random.default_rng(3)
        a = np.clip(rng.normal(150, 6, (S // 4, S // 4)), 0, 255).astype(np.uint8)
        im = Image.fromarray(a).resize((S, S)).convert("RGB")
    elif kind == "pole":
        rng = np.random.default_rng(5)
        a = np.clip(rng.normal(236, 4, (S // 8, S // 8)), 0, 255).astype(np.uint8)
        im = Image.fromarray(a).resize((S, S)).convert("RGB")
    return im.resize((S // 2, S // 2), Image.LANCZOS)


def size_of(kind):
    if kind == "tomare":
        return SIZES["tomare"]
    if kind == "crossing":
        return SIZES["crossing"]
    if kind.startswith("oneway"):
        return SIZES["oneway"]
    if kind.startswith("aux_"):
        return SIZES["aux"]
    return SIZES["round"]


# -- geometry / 形状 -----------------------------------------------------------------
def outline(shape, w, h, n=28):
    """Plate outline in (u right, v up) about its centre, counter-clockwise, and
    UVs (the texture's square over the outline's box). / 板の外形と UV。"""
    if shape == "disc":
        pts = [(w / 2 * math.cos(2 * math.pi * i / n), h / 2 * math.sin(2 * math.pi * i / n)) for i in range(n)]
    elif shape == "tri":                    # inverted: apex down / 逆三角形
        pts = [(0.0, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    else:
        pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    uv = [(0.5 + u / w, 0.5 + v / h) for u, v in pts]
    return pts, uv


def plate_mesh(m, mat, back_mat, centre, nf, shape, w, h, two_faced=False, back_face=None):
    """A plate PLATE_T thick: textured front facing nf, back (grey, or the same
    face mirrored when two-faced), edge strip. / 板（前面・背面・縁）。"""
    nf = np.array([nf[0], nf[1], 0.0])
    nf /= np.linalg.norm(nf)
    rt = np.array([-nf[1], nf[0], 0.0])            # the viewer's right / 見る人の右
    up = np.array([0.0, 0.0, 1.0])
    pts, uv = outline(shape, w, h)
    C = np.asarray(centre, float)
    F = [C + nf * PLATE_T / 2 + rt * u + up * v for u, v in pts]
    B = [C - nf * PLATE_T / 2 + rt * u + up * v for u, v in pts]
    n = len(pts)
    fan = [i for k in range(1, n - 1) for i in (0, k, k + 1)]
    m.tris(mat, F, nf, uv, fan)
    # back: reversed winding; two-faced plates read correctly from behind (u mirrored)
    # 背面: 逆回り。両面板は裏から見て正しく読める（u 反転）
    m.tris((back_face or mat) if two_faced else back_mat, B, -nf, [(1 - a, b) for a, b in uv] if two_faced else uv,
           [i for k in range(1, n - 1) for i in (0, k + 1, k)])
    for i in range(n):
        j = (i + 1) % n
        a, b, c2, d = F[i], B[i], B[j], F[j]
        e = (np.array(pts[j]) - np.array(pts[i]))
        on = rt * e[1] - up * e[0]
        on = on / (np.linalg.norm(on) or 1.0)
        # wound so the face looks outward / 外向きに巻く
        m.tris(back_mat, [a, b, c2, d], on, [(0, 0), (0, 1), (1, 1), (1, 0)],
               [0, 1, 2, 0, 2, 3] if np.cross(b - a, d - a) @ on >= 0 else [0, 2, 1, 0, 3, 2])


# -- placement / 配置 ----------------------------------------------------------------
class Placer(object):
    def __init__(self, out, osm):
        import layout
        import terrain
        self.out = out
        self.meta = json.load(open(os.path.join(out, "meta.json")))
        self.meta["road_style"] = "japan"
        self.cfg = jartic.config(self.meta)
        self.rows = jartic.load(self.meta, out) or []
        data = json.load(open(os.path.join(out, "lanes.json")))
        self.lanes = [l for l in data["lanes"] if l["kind"] == "driving"]
        self.lane_by = {(l["road"], l["lane"]): l for l in self.lanes}
        self.appr = collections.defaultdict(list)
        for a in data["approaches"]:
            self.appr[(a["road"], a["junction"])].append(a)
        self.road_lanes = collections.defaultdict(list)
        for l in self.lanes:
            if l["junction"] < 0:
                self.road_lanes[l["road"]].append(l)
        self.extra = pickle.load(open(os.path.join(out, "crossings.pkl"), "rb"))
        self.L = layout.Layout(self.meta, os.path.join(out, "lanes.json"), osm)
        self.D = terrain.drape_for(out, self.L)
        self.cw = self.L.carriageway
        # every driving lane of the road logic, junction lanes included: posts keep clear
        # 道路情報の全走行車線（交差点内を含む）: 支柱はここから離す
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        # carla.Map's get_waypoint(project_to_road=False) clamps s at a lane's ends and
        # tests only the lateral offset, so a point just past an end counts as on the lane:
        # each lane is extended LANE_CAP m past both ends along its end direction
        # carla は車線端を越えた点も車線上とみなすため、両端を LANE_CAP 延長する
        polys = []
        for l in self.lanes:
            P = l["pts"]
            if len(P) < 2:
                continue
            g = Polygon([q["l"] for q in P] + [q["r"] for q in P][::-1]).buffer(0)
            if not g.is_empty:
                polys.append(g)
            for e, o in ((P[-1], P[-2]), (P[0], P[1])):
                d = np.array(e["c"]) - np.array(o["c"])
                d = d / (np.linalg.norm(d) or 1.0) * LANE_CAP
                el, er = np.array(e["l"]), np.array(e["r"])
                cap = Polygon([el, er, er + d, el + d]).buffer(0)
                if not cap.is_empty:
                    polys.append(cap)
        self.lane_area = unary_union(polys)
        self.stop_posts = {}       # road -> post xy of its 止まれ / 止まれ支柱の位置
        self.merged = []
        self.left = self.meta.get("lefthand", False)
        self.posts = []            # dict(x, y, yaw_face, plates=[(kind, two_faced, src)]) / 支柱
        self.dropped = []
        self.spaced = []

    def kerb_lane(self, road, lanes=None):
        """The kerb-side lane of a road (keep-left: lane -1, its left edge). / 路肩側の車線。"""
        ls = lanes if lanes is not None else self.road_lanes.get(road, [])
        ls = [l for l in ls if len(l["pts"]) >= 3]
        if not ls:
            return None
        return max(ls, key=lambda l: l["lane"])

    def at_s(self, lane, s):
        """Point, travel direction and kerb-side normal on a lane at distance s
        from its start (s < 0: from its end). / 車線上の点・進行方向・路肩側の法線。"""
        C = np.array([q["c"] for q in lane["pts"]])
        seg = np.hypot(*np.diff(C, axis=0).T)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        if s < 0:
            s = cum[-1] + s
        s = float(np.clip(s, 0.0, cum[-1]))
        i = int(np.clip(np.searchsorted(cum, s) - 1, 0, len(C) - 2))
        t = (s - cum[i]) / (seg[i] or 1.0)
        p = C[i] + (C[i + 1] - C[i]) * t
        f = C[i + 1] - C[i]
        f = f / (np.linalg.norm(f) or 1.0)
        side = 1.0 if self.left else -1.0
        kn = np.array([-f[1], f[0]]) * side                     # toward the kerb / 路肩側
        half = lane["pts"][min(i, len(lane["pts"]) - 1)]["w"] / 2
        return p, f, kn, half, cum[-1]

    def kerb_spot(self, lane, s, slide=-1.0):
        """Post position BACK m behind the carriageway edge, beside the lane at
        s; slid along the lane (``slide`` per step) while the spot is on the
        road, a building or not clear. Returns (xy, f) or (None, why).
        車道端から BACK の位置（道路上・建物内なら車線に沿ってずらす）。"""
        for k in range(9):
            p, f, kn, half, length = self.at_s(lane, s + slide * k)
            # the carriageway edge on this side: walk out from the lane edge / 車道端
            q = p + kn * half
            for _ in range(40):
                if not self.cw.contains(Point(*(q + kn * 0.05))):
                    break
                q = q + kn * 0.05
            spot = q + kn * BACK
            pt = Point(*spot)
            d = self.cw.distance(pt)
            if self.cw.contains(pt) or d < BACK - 0.15:
                continue
            if self.L.building_area.buffer(0.3).contains(pt) or not self.lane_clear(pt):
                continue
            return spot, f
        return None, "no clear spot beside lane %s/%s" % (lane["road"], lane["lane"])

    def lane_clear(self, pt):
        return self.lane_area.distance(pt) >= LANE_CLEAR + POLE_R

    def add(self, kind, spot, face, src, two_faced=False, stack=True):
        """A plate at spot facing ``face`` (unit xy): onto a post within 2.5 m
        facing the same way, else a new post. / 近くの同じ向きの支柱に載せる。"""
        face = np.asarray(face, float) / (np.linalg.norm(face) or 1.0)
        if stack:
            for p in self.posts:
                if np.hypot(p["x"] - spot[0], p["y"] - spot[1]) < 2.5 and float(p["face"] @ face) > 0.87 \
                        and len(p["plates"]) < 3:
                    if any(k == kind for k, _, _ in p["plates"]):
                        # the same plate for the same traffic is already on this post
                        # 同じ交通向けの同じ板がこの支柱に既にある
                        first = next(sr for k, _, sr in p["plates"] if k == kind)
                        self.merged.append({"kind": kind, "src": src, "why": "same plate for the same traffic "
                                            "already on the post at (%.1f, %.1f): %s" % (p["x"], p["y"], first)})
                        return p
                    p["plates"].append((kind, two_faced, src))
                    return p
        p = {"x": float(spot[0]), "y": float(spot[1]), "face": face, "plates": [(kind, two_faced, src)]}
        self.posts.append(p)
        return p

    # each kind / 種類ごと
    def stops(self):
        for road, j, back in self.extra.get("stop", []):
            lanes = [self.lane_by.get((road, a["lane"])) for a in self.appr.get((road, j), [])]
            lane = self.kerb_lane(road, [l for l in lanes if l is not None])
            if lane is None:
                self.dropped.append(("tomare", road, j, "no lane"))
                continue
            spot, f = self.kerb_spot(lane, -(back + 1.0))
            if spot is None:
                self.dropped.append(("tomare", road, j, f))
                continue
            self.add("tomare", spot, -f, "stop road %d junction %d" % (road, j))
            self.stop_posts[road] = spot

    def no_turns(self):
        for r in self.rows:
            if r["code"] != jartic.NO_TURN or r["kind"] != 1 or not r["entry"] or not r["toward"]:
                continue
            P, E = r["xy"][0], r["entry"][0]
            k, score = jartic.match_approach(P, E, self.appr, self.lane_by)
            if k is None:
                self.dropped.append(("noturn", P, "no approach"))
                continue
            turns = {jartic.turn_of(P, E, T) for T in r["toward"]}
            code = "".join(c for c, t in (("s", "straight"), ("l", "left"), ("r", "right")) if t in turns)
            lanes = [self.lane_by.get((k[0], a["lane"])) for a in self.appr[k]]
            lane = self.kerb_lane(k[0], [l for l in lanes if l is not None])
            stop = {(rr, jj): b for rr, jj, b in self.extra.get("stop", [])}.get(k, 2.0)
            spot, f = self.kerb_spot(lane, -(stop + 1.0))
            if spot is None:
                self.dropped.append(("noturn", P, f))
                continue
            p = self.add("noturn_" + code, spot, -f, "JARTIC #%s road %d" % (jartic.short_key(r), k[0]))
            if r["time"]:
                p["plates"].append(("aux_" + r["time"].replace("-", "_"), False, "hours"))

    def crossings(self):
        for cr in self.extra.get("crossings", []):
            if cr.get("signal"):
                continue                 # signalised: the plate may be left out (設置基準) / 信号付きは省略可
            c, f, n = np.array(cr["c"]), np.array(cr["f"]), np.array(cr["n"])
            for t, sgn in ((cr["t1"], 1.0), (cr["t0"], -1.0)):
                # traffic with this kerb on its left travels d (keep-left) / この縁石を左に見る交通の向き
                d = np.array([n[1], -n[0]]) * sgn * (1.0 if self.left else -1.0)
                spot = None
                # before the crossing, at its edge, beside it, after it (junction corners) / 手前・端・横・先
                hw = cr["width"] / 2
                for along in (hw + 0.6, hw + 0.2, 0.0, -(hw + 0.6), hw + 1.5, hw + 2.5, -(hw + 1.5), -(hw + 2.5)):
                    q = c + n * (t + sgn * (BACK - 0.3)) - d * along
                    for k in range(20):
                        pt = Point(*q)
                        dist = self.cw.distance(pt)
                        if not self.cw.contains(pt) and dist >= BACK - 0.15 and self.lane_clear(pt):
                            break
                        q = q + n * sgn * 0.1
                    if not self.cw.contains(pt) and BACK - 0.15 <= dist <= 1.6 and self.lane_clear(pt) and \
                            not self.L.building_area.buffer(0.3).contains(pt):
                        spot = q
                        break
                if spot is None:
                    self.dropped.append(("crossing", [round(float(v), 1) for v in c], "no clear kerb spot"))
                    continue
                self.add("crossing", spot, -d, "crossing %s" % cr.get("src", ""), two_faced=True)

    def along(self, code, kind, every=None, first=6.0, min_len=25.0, two_faced=False, parallel=False, space=0.0):
        """Plates for a line regulation: at each covered road's start (and every
        ``every`` m). / 線規制: 各道路の始点（と一定間隔）。"""
        seen = set()
        for r in self.rows:
            if r["code"] != code or r["kind"] != 2:
                continue
            for road, ln in jartic.lanes_along(r["xy"], self.lanes):
                if road in seen:
                    continue
                seen.add(road)
                lane = self.kerb_lane(road)
                if lane is None:
                    continue
                p, f, kn, half, length = self.at_s(lane, 0.0)
                if length < min_len:
                    continue
                ss = [first] + (list(np.arange(first + every, length - 15.0, every)) if every else [])
                for s in ss:
                    spot, f = self.kerb_spot(lane, s, slide=1.0)
                    if spot is None:
                        self.dropped.append((kind, road, s, f))
                        continue
                    k = kind
                    face = -f
                    if space and any(q["kind"] == kind and float(q["face"] @ -f) > 0.8 and
                                     np.hypot(q["x"] - spot[0], q["y"] - spot[1]) < space for q in self.spaced):
                        # a plate for the same traffic within ``space`` m: not repeated after every side street
                        # 同じ交通向けの板が近くにある: 脇道ごとには繰り返さない
                        continue
                    if space:
                        self.spaced.append({"kind": kind, "face": -f, "x": spot[0], "y": spot[1]})
                    if parallel:
                        # along the road, readable from the junction: arrow with the traffic
                        # 道路に平行（交差点から読める向き、矢印は進行方向）
                        _, ff, kn, _, _ = self.at_s(lane, s)
                        face = -kn
                        k = kind + ("" if self.left else "_l")
                    self.add(k, spot, face, "JARTIC #%s road %d" % (jartic.short_key(r), road), two_faced=two_faced,
                             stack=not parallel)

    def place(self):
        self.stops()
        self.no_turns()
        self.crossings()
        self.along(jartic.ONEWAY, "oneway", first=4.0, min_len=15.0, two_faced=True, parallel=True)
        self.along(jartic.NO_PARKING, "noparking", every=150.0, space=120.0)
        self.along(jartic.BICYCLE_PAVEMENT, "bicycle", first=10.0, min_len=30.0, space=120.0)
        return self

    # -- meshes / メッシュ
    def ground_z(self, x, y):
        if self.D is not None:
            return float(np.asarray(self.D.z(np.array([x]), np.array([y]))).ravel()[0]) + 0.15
        return 0.15

    def build(self):
        import ground
        name = self.meta["name"]
        mesh_dir = os.path.join(self.out, "meshes")
        tiles = list(ground.tiles(self.meta))
        meshes = collections.defaultdict(Mesh)
        rec = []
        back_kinds = set()
        for p in self.posts:
            x, y = p["x"], p["y"]
            tn = next((t for t, tb in tiles if tb.contains(Point(x, y))), tiles[0][0])
            m = meshes[tn]
            zg = self.ground_z(x, y)
            # plates top to bottom: the main plate first, the auxiliary below / 本標識が上、補助が下
            sizes = [size_of(k) for k, _, _ in p["plates"]]
            z = zg + BOTTOM + sum(h for _, _, h in sizes) + GAP * (len(sizes) - 1)
            top = z
            nf = p["face"]
            for (k, two, src), (shape, w, h) in zip(p["plates"], sizes):
                cz = z - h / 2
                ctr = (x + nf[0] * (POLE_R + PLATE_T), y + nf[1] * (POLE_R + PLATE_T), cz)
                # a one-way plate's back shows the arrow the other way round (same direction on the road)
                # 一方通行の裏面は逆向きの図柄（道路上では同じ向き）
                bf = None
                if k.startswith("oneway"):
                    bf = "jp_plate_oneway" if k.endswith("_l") else "jp_plate_oneway_l"
                    back_kinds.add(bf[9:])
                plate_mesh(m, "jp_plate_" + k, "jp_plate_back", ctr, nf, shape, w, h, two_faced=two, back_face=bf)
                rec.append({"kind": k, "x": round(x, 2), "y": round(y, 2), "z": round(cz, 2),
                            "ground": round(zg, 2), "face_deg": round(math.degrees(math.atan2(nf[1], nf[0])), 1),
                            "w": w, "h": h, "src": src})
                z -= h + GAP
            m.cylinder("jp_plate_pole", x, y, zg - 0.5, top + 0.04, POLE_R, seg=10)
        man_p = os.path.join(mesh_dir, "manifest.json")
        man = json.load(open(man_p))
        for o in [o for o in man["objects"] if o["name"].startswith("Terrain_JpPlates_")]:
            man["objects"].remove(o)
            try:
                os.remove(os.path.join(mesh_dir, o["name"] + ".npz"))
            except OSError:
                pass
        for tn, m in sorted(meshes.items()):
            arrs = m.arrays()
            if not arrs:
                continue
            on = "Terrain_JpPlates_%s_%s" % (name, tn)
            data = {}
            for mat, (V, N, UV, I) in arrs.items():
                data[mat + "|V"], data[mat + "|N"], data[mat + "|UV"], data[mat + "|I"] = V, N, UV, I
            np.savez_compressed(os.path.join(mesh_dir, on + ".npz"), **data)
            man["objects"].append({"name": on, "kind": "jp_plates", "materials": sorted(arrs),
                                   "tris": int(sum(len(a[3]) for a in arrs.values()))})
        kinds = sorted({r["kind"] for r in rec} | back_kinds) + ["back", "pole"]
        tex_dir = os.path.join(self.out, "textures")
        for k in kinds:
            texture(k).save(os.path.join(tex_dir, "jp_plate_%s.png" % k))
            man["materials"]["jp_plate_" + k] = ["jp_plate_" + k, None, 0.35 if k != "pole" else 0.45,
                                                 0.0, k == "pole"]
        json.dump(man, open(man_p, "w"), indent=0)
        json.dump({"posts": len(self.posts), "plates": rec, "dropped": [list(map(str, d)) for d in self.dropped],
                   "merged": self.merged, "stop_landmarks": getattr(self, "moved", {}),
                   # what markings_jp did with each JARTIC row (matched, dropped and why)
                   # markings_jp での JARTIC 各行の扱い（理由付き）
                   "jartic_log": self.extra.get("jartic_log", [])},
                  open(os.path.join(self.out, "jp_plates.json"), "w"), ensure_ascii=False, indent=0, default=str)
        return rec

    def move_stop_landmarks(self):
        """Put each Stencil_STOP 206 signal at its 止まれ post: CARLA does not move
        a "Stencil_STOP" off the lanes (MapBuilder), so at the old t it stood on
        junction lanes. s, t from the kerb lane's left edge (the reference line
        on netconvert's keep-left roads) plus the road's laneOffset.
        Stencil_STOP の 206 を止まれ支柱の位置へ（CARLA は車線外へ動かさないため）。"""
        import xml.etree.ElementTree as ET
        xodr = os.path.join(self.out, self.meta["name"] + ".xodr")
        tree = ET.parse(xodr)
        self.moved = {}
        for road in tree.getroot().findall("road"):
            rid = int(road.get("id"))
            if rid not in self.stop_posts:
                continue
            sigs = [x for x in road.iter("signal") if x.get("type") == "206" and x.get("name") == "Stencil_STOP"]
            lane = self.kerb_lane(rid)
            if not sigs or lane is None:
                continue
            E = np.array([q["l"] if self.left else q["r"] for q in lane["pts"]])
            S = np.array([q["s"] for q in lane["pts"]])
            p = np.asarray(self.stop_posts[rid], float)
            d, i = jartic._dist_poly(p, E)
            a, b = E[i], E[min(i + 1, len(E) - 1)]
            ab = b - a
            t_ = float(np.clip(((p - a) @ ab) / max(ab @ ab, 1e-9), 0.0, 1.0))
            s_ = float(S[i] + (S[min(i + 1, len(S) - 1)] - S[i]) * t_)
            side = ab[0] * (p - a)[1] - ab[1] * (p - a)[0]          # > 0: left of travel / 進行方向の左
            off = 0.0
            lo = road.find("lanes/laneOffset")
            if lo is not None:
                off = float(lo.get("a", 0.0))
            t = off + (d if side > 0 else -d)
            for sg in sigs:
                sg.set("s", "%.2f" % min(max(s_, 0.05), float(road.get("length")) - 0.05))
                sg.set("t", "%.2f" % t)
            self.moved[rid] = [round(s_, 2), round(t, 2)]
        tree.write(xodr, encoding="utf-8", xml_declaration=True)
        return len(self.moved)


def check(placer, rec):
    """Offline checks: posts off the carriageway, 0.3-2.0 m behind its edge, on
    the kerb side, plates facing their traffic, nothing low over a lane.
    確認: 車道外・車道端から 0.3-2.0 m・路肩側・向き・車道上に低い物が無い。"""
    import shapely
    bad = []
    for r in rec:
        pt = Point(r["x"], r["y"])
        dl = placer.lane_area.distance(pt)
        if dl < LANE_CLEAR:
            bad.append(("post %.2f m from an xodr driving lane (< %.1f)" % (dl, LANE_CLEAR), r))
        d = placer.cw.distance(pt)
        if placer.cw.contains(pt) or not 0.3 <= d <= 2.0:
            bad.append(("distance %.2f m" % d, r))
        half = r["w"] / 2
        if r["z"] - r["h"] / 2 - r["ground"] < 1.8 - 1e-6:
            bad.append(("low plate", r))
        if d < half and r["z"] - r["h"] / 2 - r["ground"] < 4.5:
            bad.append(("plate over the carriageway below 4.5 m", r))
    mesh_dir = os.path.join(placer.out, "meshes")
    road = placer.cw.buffer(-0.02)
    over = 0
    for f in os.listdir(mesh_dir):
        if f.startswith("Terrain_JpPlates_"):
            d = np.load(os.path.join(mesh_dir, f))
            for k in d.files:
                if k.endswith("|V"):
                    over += int(shapely.contains_xy(road, d[k][:, 0], d[k][:, 1]).sum())
    return bad, over


def main():
    osm, out = sys.argv[1:3]
    meta = json.load(open(os.path.join(out, "meta.json")))
    cfg = jartic.config(meta)
    if cfg is None or not cfg.get("plates", True):
        print("jp_signs: map has no \"jartic_d\": nothing to do")
        return
    P = Placer(out, osm).place()
    n_moved = P.move_stop_landmarks()
    rec = P.build()
    n = collections.Counter(r["kind"] for r in rec)
    print("jp_signs: %d posts, %d plates: %s" % (len(P.posts), len(rec), ", ".join("%s %d" % kv for kv in sorted(n.items()))))
    print("jp_signs: %d Stencil_STOP landmarks moved to their 止まれ posts" % n_moved)
    for d in P.dropped:
        print("  dropped: %s" % (d,))
    for m in P.merged:
        print("  merged: %s" % (m,))
    bad, over = check(P, rec)
    for why, r in bad:
        print("  CHECK %s: %s" % (why, r))
    print("check: %d plate problems, %d plate vertices over the carriageway (must be 0)" % (len(bad), over))


if __name__ == "__main__":
    main()
