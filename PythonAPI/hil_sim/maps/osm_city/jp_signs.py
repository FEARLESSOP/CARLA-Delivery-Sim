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
                             (NPA design: blue pentagon, white figure)
  一方通行 (326-A)           JARTIC one-way lines, at the start of each road,
                             along the road (two-faced, arrow only, 80 x 40 cm)
  車両進入禁止 (303)         the exit end of each one-way road segment of those
                             lines, facing the junction (wrong-way driver's left)
  駐車禁止 (316)             JARTIC no-parking lines: each road's start, then
                             every 150 m
  自転車歩道通行可 (325-3)   JARTIC lines: each road's start (adult, child and
                             bicycle, no rider)

Faces follow the NPA chart "Traffic Lights and Road Signs" (guide ch. 18).
White 60.5 mm poles, plates 0.6 m (止まれ 0.8 m), bottom edge 1.85 m over
the pavement, the post 0.6 m behind the carriageway edge (0.4 m behind the
kerb) on the kerb side (keep-left: the left of travel), plate facing the
traffic it is for. Posts within 2.5 m facing the same way share one pole
(plates stacked). Objects Terrain_JpPlates_<Name>_<tile>, materials
jp_plate_* (CARLA's importer drops names with "sign" or "light"). Writes
build/<Name>/jp_plates.json (every plate, for checks).

Street View checks (build/<Name>/verified_plates.json, optional): 一方通行 /
進入禁止 plates seen on roads the JARTIC data has no row for, each with its
evidence image; placed by the same rules as the JARTIC ones, only on roads the
road logic runs one way. Entries under "none" record ends checked with no plate.

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
BACK_NEAR = (0.5, 0.45)    # nearer spots tried when a structure blocks BACK (check: >= 0.3 m) / 縁石寄りの候補
BOTTOM = 1.85              # lowest plate edge over the pavement (1.8 m min) / 板下端の高さ
GAP = 0.05
LANE_CAP = 2.0             # lane ends extended this far for the clearance test (carla clamps s) / 端の延長
GUARD_CLEAR = 0.35         # posts this far behind the ring guardrail's beam (its posts 0.04-0.18) / ガードレール背後
LANE_CLEAR = 0.3           # every post this far from any xodr driving lane (junction lanes too) / 全車線からの離れ
PLATE_T = 0.012
SIGNAL_CLEAR = 0.8         # a post nearer a traffic-signal pole than this goes onto that pole / 信号柱に近い板は信号柱に付ける
SIGNAL_R = 0.14            # signal pole radius at plate height (jp_shingo.py: φ270 tapering to φ210) / 信号柱の半径
STOP_STEPS = 3             # a 止まれ post slides at most 2 m (1 m steps) from 1 m before its line / 止まれの移動上限
STOP_TOL = 3.05            # every Stencil_STOP 206 within this many m before its stop line (check) / 停止線からの許容
SAME_LINE = 15.0           # plates this far sideways of a stepped-on plate's road are other traffic / 別の交通
VERIFIED = "verified_plates.json"   # Street View verified plates in the build folder / 現地確認済みの標識
VERIFIED_MATCH = 10.0      # a verified plate's road end within this many m of its "xy" / 道路端の照合距離
CORE_CLEAR = 0.3           # embedded model: post this far off its asphalt (and the plate's half width + 5 cm) / モデルの車道から
CORE_REACH = 3.0           # ... a post on it moves at most this far (nearest free spot, same side) / 移動上限
CORE_STEP = 0.1            # ... candidate spacing / 候補の間隔
POST_GAP = 0.6             # a moved post keeps this far from every other post / 他の支柱との間隔
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
# shape, width, height (m) / 形・幅・高さ
# 道路標識令 standard sizes (NPA "Traffic Lights and Road Signs" designs): round 60 cm,
# 止まれ 80 cm side, 407 pentagon 60 cm, 一方通行 326-A 80 x 40 cm, auxiliary 50 x 20 cm
# 標準寸法: 円形 60・止まれ一辺 80・横断歩道（五角形）60・一方通行 80x40・補助 50x20 cm
SIZES = {"tomare": ("tri", 0.8, 0.8 * math.sqrt(3) / 2), "round": ("disc", 0.6, 0.6),
         "crossing": ("pent", 0.6, 0.6), "oneway": ("rect", 0.8, 0.4), "aux": ("rect", 0.5, 0.2)}
PENT_SHOULDER = 0.25       # 407 plate: vertical sides up to 25 % of its height / 五角形の肩の高さ
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


def _curve(p0, p1, p2, n=10):
    """Quadratic Bezier points p0 -> p2 bent toward p1. / 2 次ベジェの点列。"""
    return [((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0],
             (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1]) for t in np.linspace(0, 1, n)]


def _walkers(d, S, c):
    """Adult and child hand in hand, upper half of a round plate (325-3). / 大人と子供。"""
    w = int(S * 0.05)
    ax, cx = c - S * 0.07, c + S * 0.08
    d.ellipse([ax - S * 0.04, S * 0.14, ax + S * 0.04, S * 0.22], fill=WHITE)
    d.line([(ax, S * 0.22), (ax, S * 0.4)], fill=WHITE, width=int(S * 0.075))
    d.line([(ax - S * 0.02, S * 0.39), (ax - S * 0.03, S * 0.53)], fill=WHITE, width=w)
    d.line([(ax + S * 0.02, S * 0.39), (ax + S * 0.03, S * 0.53)], fill=WHITE, width=w)
    d.line([(ax - S * 0.035, S * 0.24), (ax - S * 0.05, S * 0.37)], fill=WHITE, width=int(S * 0.03))
    d.line([(ax + S * 0.035, S * 0.24), (cx - S * 0.03, S * 0.36)], fill=WHITE, width=int(S * 0.03))
    d.ellipse([cx - S * 0.028, S * 0.28, cx + S * 0.028, S * 0.336], fill=WHITE)
    d.line([(cx, S * 0.335), (cx, S * 0.44)], fill=WHITE, width=int(S * 0.05))
    d.line([(cx - S * 0.012, S * 0.43), (cx - S * 0.02, S * 0.53)], fill=WHITE, width=int(S * 0.03))
    d.line([(cx + S * 0.012, S * 0.43), (cx + S * 0.02, S * 0.53)], fill=WHITE, width=int(S * 0.03))
    d.line([(cx + S * 0.02, S * 0.35), (cx + S * 0.05, S * 0.42)], fill=WHITE, width=int(S * 0.022))


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
        # NPA design: a thin white rim; 止まれ about half the plate wide, high in the triangle
        # 細い白縁・文字は板幅の約半分
        b = S * 0.03
        d.polygon([(b * 1.7, top + b), (S - b * 1.7, top + b), (c, bot - b * 2.0)], fill=RED)
        f = _font(int(S * 0.175))
        d.text((c, S * 0.27), "止まれ", fill=WHITE, font=f, anchor="mm")
        del h
    elif kind.startswith("noturn_") or kind in ("noparking", "bicycle", "noentry"):
        im.paste(WHITE, (0, 0, S, S))
        if kind == "noentry":                              # 303 車両進入禁止 (one-way exits)
            d.ellipse([S * 0.02, S * 0.02, S * 0.98, S * 0.98], fill=RED)
            d.rectangle([S * 0.13, c - S * 0.09, S * 0.87, c + S * 0.09], fill=WHITE)
        elif kind == "noparking":                          # 316 駐車禁止
            d.ellipse([S * 0.02, S * 0.02, S * 0.98, S * 0.98], fill=RED)
            d.ellipse([S * 0.13, S * 0.13, S * 0.87, S * 0.87], fill=BLUE)
            k = S * 0.33
            d.line([(c - k * 0.95, c - k * 0.95), (c + k * 0.95, c + k * 0.95)], fill=RED, width=int(S * 0.1))
        else:
            d.ellipse([S * 0.03, S * 0.03, S * 0.97, S * 0.97], fill=BLUE)
            if kind == "bicycle":                          # 325-3 普通自転車歩道通行可
                # NPA "Standard Bicycles and Pedestrians Only": adult and child, bicycle below
                # 大人と子供（上）・自転車（下、人は乗らない）
                _walkers(d, S, c)
                w = int(S * 0.028)
                y0, r = S * 0.72, S * 0.085
                xl, xr = c - S * 0.15, c + S * 0.15
                for x in (xl, xr):
                    d.ellipse([x - r, y0 - r, x + r, y0 + r], outline=WHITE, width=w)
                hub = (c - S * 0.02, y0)
                seat, head = (c - S * 0.07, y0 - S * 0.12), (c + S * 0.1, y0 - S * 0.13)
                d.line([(xl, y0), hub, (xr - S * 0.01, y0 - S * 0.11), (c - S * 0.06, y0 - S * 0.11), (xl, y0)],
                       fill=WHITE, width=w, joint="curve")
                d.line([hub, seat], fill=WHITE, width=w)
                d.line([(xr, y0), head], fill=WHITE, width=w)
                d.line([(seat[0] - S * 0.04, seat[1]), (seat[0] + S * 0.03, seat[1])], fill=WHITE, width=w)
                d.line([head, (head[0] - S * 0.04, head[1] - S * 0.02)], fill=WHITE, width=w)
            else:                                          # 311 指定方向外進行禁止
                turns = kind.split("_")[1]
                w, hd = int(S * 0.085), S * 0.1
                base, top = (c, S * 0.82), (c, S * 0.25)
                if "s" in turns:
                    _arrow(d, [base, top], w, hd, WHITE)
                for t, sx in (("l", -1.0), ("r", 1.0)):
                    if t not in turns:
                        continue
                    if "s" in turns:                   # branch off the stem / 幹から分岐
                        _arrow(d, _curve((c, S * 0.62), (c, S * 0.48), (c + sx * S * 0.27, S * 0.46)), w, hd, WHITE)
                    elif turns == "lr":                # fork / 二股
                        d.line([base, (c, S * 0.6)], fill=WHITE, width=w)
                        _arrow(d, _curve((c, S * 0.62), (c, S * 0.44), (c + sx * S * 0.27, S * 0.42)), w, hd, WHITE)
                    else:                              # single turn: stem up, round corner / 単独の曲がり
                        _arrow(d, [base] + _curve((c, S * 0.62), (c, S * 0.40), (c + sx * S * 0.27, S * 0.40)),
                               w, hd, WHITE)
    elif kind == "crossing":                               # 407-A 横断歩道 (NPA design)
        # blue pentagon plate (the mesh is the outline), thin white rim, white walking
        # figure over the crossing bars / 青い五角形・白縁・白い歩行者と横断帯
        im.paste(BLUE, (0, 0, S, S))
        from shapely.geometry import Polygon as _Poly
        pts, uv = outline("pent", 1.0, 1.0)
        rim = list(_Poly([(u * S, (1 - v) * S) for u, v in uv]).buffer(-S * 0.04, join_style=2).exterior.coords)
        d.line(rim, fill=WHITE, width=int(S * 0.022), joint="curve")
        d.rectangle([S * 0.2, S * 0.69, S * 0.33, S * 0.725], fill=WHITE)
        d.rectangle([S * 0.67, S * 0.69, S * 0.8, S * 0.725], fill=WHITE)
        d.rectangle([S * 0.17, S * 0.83, S * 0.83, S * 0.865], fill=WHITE)
        w = int(S * 0.06)
        d.ellipse([c - S * 0.045, S * 0.25, c + S * 0.045, S * 0.34], fill=WHITE)    # head / 頭
        d.rectangle([c - S * 0.05, S * 0.235, c + S * 0.05, S * 0.255], fill=WHITE)   # hat brim / 帽子
        d.rectangle([c - S * 0.03, S * 0.2, c + S * 0.03, S * 0.24], fill=WHITE)
        hip = (c - S * 0.02, S * 0.56)
        d.line([(c, S * 0.36), hip], fill=WHITE, width=int(S * 0.08))                 # body / 胴
        d.line([hip, (c - S * 0.1, S * 0.69), (c - S * 0.15, S * 0.78)], fill=WHITE, width=w, joint="curve")
        d.line([hip, (c + S * 0.06, S * 0.67), (c + S * 0.13, S * 0.77)], fill=WHITE, width=w, joint="curve")
        d.line([(c - S * 0.01, S * 0.39), (c - S * 0.08, S * 0.5), (c - S * 0.09, S * 0.57)], fill=WHITE,
               width=int(S * 0.04), joint="curve")
        d.line([(c + S * 0.01, S * 0.39), (c + S * 0.08, S * 0.48), (c + S * 0.12, S * 0.53)], fill=WHITE,
               width=int(S * 0.04), joint="curve")
    elif kind.startswith("oneway") or kind.startswith("aux_"):
        # drawn at the plate's own aspect, then squeezed into the square the UVs span
        # 板の縦横比で描いてから正方形へ（UV が正方形を張るため）
        shape, w, h = size_of(kind)
        W, H = S, int(S * h / w)
        im = Image.new("RGB", (W, H), BLUE if kind.startswith("oneway") else WHITE)
        d = ImageDraw.Draw(im)
        if kind.startswith("oneway"):                      # 326-A 一方通行
            # NPA design: white rim, one big white arrow, no lettering / 白縁・白矢印のみ（文字なし）
            d.rectangle([H * 0.06, H * 0.06, W - H * 0.06, H * 0.94], outline=WHITE, width=int(H * 0.045))
            y = H * 0.5
            pts = [(W * 0.86, y), (W * 0.36, y)]
            if not kind.endswith("_l"):
                pts = [(W * 0.14, y), (W * 0.64, y)]
            _arrow(d, pts, int(H * 0.2), H * 0.3, WHITE)
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
    elif shape == "pent":                   # 407 plate: apex up, vertical sides at the bottom / 五角形
        ys = -h / 2 + h * PENT_SHOULDER
        pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, ys), (w * 0.03, h / 2), (-w * 0.03, h / 2), (-w / 2, ys)]
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


# -- OpenDRIVE reference line, as libcarla evaluates it / 参照線（libcarla と同じ評価） ----------
SIGNAL_SHIFT = 0.25        # libcarla puts a signal this far forward along its offset line / 信号の前方移動量


def ref_point(road, s):
    """x, y, heading of an xodr <road>'s reference line at s: line, arc,
    paramPoly3 (s -> p through the arc length of samples ~0.3 m apart, as
    libcarla's GeometryParamPoly3). / 参照線上の点と方位。"""
    geos = road.findall("planView/geometry")
    g = geos[0]
    for gg in geos:
        if float(gg.get("s")) <= s + 1e-9:
            g = gg
    s0, x0, y0, h0, L = (float(g.get(k)) for k in ("s", "x", "y", "hdg", "length"))
    ds = min(max(s - s0, 0.0), L)
    k = g[0]
    if k.tag == "arc" and abs(float(k.get("curvature"))) > 1e-12:
        c = float(k.get("curvature"))
        h = h0 + c * ds
        return x0 + (math.sin(h) - math.sin(h0)) / c, y0 - (math.cos(h) - math.cos(h0)) / c, h
    if k.tag == "paramPoly3":
        a = [float(k.get(n)) for n in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")]
        pmax = 1.0 if k.get("pRange", "normalized") == "normalized" else L
        P = np.linspace(0.0, pmax, int(math.ceil(L / 0.3)) + 1)
        U = a[0] + a[1] * P + a[2] * P ** 2 + a[3] * P ** 3
        V = a[4] + a[5] * P + a[6] * P ** 2 + a[7] * P ** 3
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(U), np.diff(V)))])
        p = float(np.interp(ds, cum, P))
        u = a[0] + a[1] * p + a[2] * p ** 2 + a[3] * p ** 3
        v = a[4] + a[5] * p + a[6] * p ** 2 + a[7] * p ** 3
        hh = math.atan2(a[5] + 2 * a[6] * p + 3 * a[7] * p ** 2, a[1] + 2 * a[2] * p + 3 * a[3] * p ** 2)
        ch, sh = math.cos(h0), math.sin(h0)
        return x0 + u * ch - v * sh, y0 + u * sh + v * ch, h0 + hh
    return x0 + ds * math.cos(h0), y0 + ds * math.sin(h0), h0       # line (spirals: not used by netconvert here)


def signal_xy(road, s, t, shift=SIGNAL_SHIFT, step=0.005):
    """Where CARLA puts a signal without positionInertial: (s, t) on the road,
    then ``shift`` m forward along the line at offset t. / CARLA の信号位置。"""
    def at(ss):
        x, y, h = ref_point(road, ss)
        return x - math.sin(h) * t, y + math.cos(h) * t
    ss, acc = s, 0.0
    prev = at(ss)
    while acc < shift:
        ss += step
        q = at(ss)
        acc += math.hypot(q[0] - prev[0], q[1] - prev[1])
        prev = q
    return prev


def tri_box(v, h):
    """True if triangle v (3 x 3, box frame) meets the box |x| <= h (separating
    axes: box faces, triangle normal, 9 edge cross products). / 三角形と箱の交差。"""
    if (v.min(0) > h).any() or (v.max(0) < -h).any():
        return False
    e = [v[1] - v[0], v[2] - v[1], v[0] - v[2]]
    n = np.cross(e[0], e[1])
    if abs(n @ v[0]) > np.abs(n) @ h:
        return False
    for ed in e:
        for ax in np.eye(3):
            a = np.cross(ax, ed)
            if not a.any():
                continue
            pr = v @ a
            if pr.min() > np.abs(a) @ h or pr.max() < -(np.abs(a) @ h):
                return False
    return True


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
        # the ring guardrail (props.py, maps.json "ring_guardrail"): posts stand
        # behind it, never in the beam / 外周ガードレール: 支柱はその後ろに立てる
        import props
        self.meta["ring_guardrail"] = bool(_map_entry(self.meta["name"]).get("ring_guardrail"))
        self.L.meta["ring_guardrail"] = self.meta["ring_guardrail"]
        self.guard = props.guard_line(self.L)
        self.guard_moved = 0
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
        # embedded model (maps.json "core"): its asphalt is wider than the road logic's
        # lanes (shoulders, junction corner aprons); posts keep off it too, and the kerb
        # the 0.3-2.0 m check measures from is its edge / 埋め込みモデルの車道アスファルト
        import core_merge
        self.core_road = core_merge.core_asphalt(out, self.meta)
        if self.core_road is not None and self.core_road.is_empty:
            self.core_road = None
        self.kerb_edge = self.cw if self.core_road is None else unary_union([self.cw, self.core_road])
        self.core_moved, self.core_dropped = [], []
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

    def kerb_spot(self, lane, s, slide=-1.0, kind=None, face=None, steps=9):
        """Post position BACK m behind the carriageway edge, beside the lane at
        s; slid along the lane (``slide`` per step) while the spot is on the
        road, a building or not clear. Returns (xy, f) or (None, why).
        車道端から BACK の位置（道路上・建物内なら車線に沿ってずらす）。"""
        seen = set()
        for k in range(steps):
            p, f, kn, half, length = self.at_s(lane, s + slide * k)
            # the carriageway edge on this side: walk out from the lane edge / 車道端
            q = p + kn * half
            for _ in range(40):
                if not self.cw.contains(Point(*(q + kn * 0.05))):
                    break
                q = q + kn * 0.05
            # with a clash test: nearer the kerb (BACK_NEAR) before sliding on, for a
            # wall or fence just behind the pavement / 塀が近いときは縁石寄りも試す
            for b in ((BACK,) + BACK_NEAR if kind else (BACK,)):
                spot = q + kn * b
                pt = Point(*spot)
                d = self.cw.distance(pt)
                if self.cw.contains(pt) or d < b - 0.15 or (kind and d < size_of(kind)[1] / 2 + 0.05):
                    continue                       # the plate's edge stays off the carriageway / 板端は車道外
                if self.L.building_area.buffer(0.3).contains(pt) or not self.lane_clear(pt):
                    continue
                if kind:
                    hit = self.clashes(self.behind_guard(spot, count=False), face(f, kn), kind)
                    if hit:
                        seen.update(hit)
                        continue
                return spot, f
        return None, "no clear spot beside lane %s/%s" % (lane["road"], lane["lane"]) + \
            (" (structures in the way: %s)" % ", ".join(sorted(seen)) if seen else "")

    def far_spot(self, lane, s, slide=-1.0, kind=None, face=None, steps=9):
        """Like kerb_spot, on the far side of the road (keep-left: the right of
        travel, the side a wrong-way driver has on the left). / 反対側の路端。"""
        for k in range(steps):
            p, f, kn, half, length = self.at_s(lane, s + slide * k)
            kn = -kn
            q = p + kn * half
            for _ in range(200):                  # over the other lanes of the road / 他の車線を越える
                if not self.cw.contains(Point(*(q + kn * 0.05))):
                    break
                q = q + kn * 0.05
            spot = q + kn * BACK
            pt = Point(*spot)
            if self.cw.contains(pt) or self.cw.distance(pt) < BACK - 0.15:
                continue
            if self.L.building_area.buffer(0.3).contains(pt) or not self.lane_clear(pt):
                continue
            if kind and self.clashes(self.behind_guard(spot, count=False), face(f, kn), kind):
                continue
            return spot, f
        return None, "no clear spot on the far side of lane %s/%s" % (lane["road"], lane["lane"]) + \
            (" (structures in the way)" if kind else "")

    def lane_clear(self, pt):
        return self.lane_area.distance(pt) >= LANE_CLEAR + POLE_R

    def add(self, kind, spot, face, src, two_faced=False, stack=True):
        """A plate at spot facing ``face`` (unit xy): onto a post within 2.5 m
        facing the same way, else a new post. / 近くの同じ向きの支柱に載せる。"""
        face = np.asarray(face, float) / (np.linalg.norm(face) or 1.0)
        spot = self.behind_guard(spot)
        # a post beside a traffic-signal pole is hidden by it: the plate goes onto
        # the signal pole itself (common Japanese practice) / 信号柱の陰になる板は信号柱に付ける
        pole = self.signal_pole_near(spot)
        if pole is not None:
            spot = np.array(pole[1:3])
            stack = True
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
        if pole is not None:
            p["signal"] = pole[0]
        self.posts.append(p)
        return p

    def signal_poles(self):
        """(id, x, y) of every traffic-signal pole, where CARLA spawns it: the
        xodr's dynamic signals at their positionInertial, else the reference
        line's point at s offset by t, then moved SIGNAL_SHIFT m forward along
        that offset line (libcarla does; fitted on carla.Map, 24 signals within
        0.01 m; check_signal_poles.py checks it). / 信号柱の位置（CARLA と同じ計算）。"""
        if getattr(self, "_signals", None) is not None:
            return self._signals
        import xml.etree.ElementTree as ET
        self._signals = []
        xodr = os.path.join(self.out, self.meta["name"] + ".xodr")
        for road in ET.parse(xodr).getroot().findall("road"):
            for sg in road.iter("signal"):
                if sg.get("dynamic") != "yes":
                    continue
                pi = sg.find("positionInertial")
                if pi is not None:
                    self._signals.append((sg.get("id"), float(pi.get("x")), float(pi.get("y"))))
                    continue
                x, y = signal_xy(road, float(sg.get("s")), float(sg.get("t")))
                self._signals.append((sg.get("id"), x, y))
        return self._signals

    def signal_pole_near(self, spot, r=SIGNAL_CLEAR):
        """The nearest signal pole within r m of spot, or None. / 近くの信号柱。"""
        best = None
        for sp in self.signal_poles():
            d = float(np.hypot(sp[1] - spot[0], sp[2] - spot[1]))
            if d < r and (best is None or d < best[0]):
                best = (d, sp)
        return best[1] if best else None

    def behind_guard(self, spot, count=True):
        """A post within GUARD_CLEAR m of the ring guardrail's beam line goes
        GUARD_CLEAR m behind it (away from the road; past the rail's posts).
        ガードレールに近い支柱はその後ろへ。"""
        if self.guard is None:
            return spot
        pt = Point(float(spot[0]), float(spot[1]))
        if self.guard.distance(pt) >= GUARD_CLEAR:
            return spot
        on = self.guard.interpolate(self.guard.project(pt))
        q = self.cw.boundary.interpolate(self.cw.boundary.project(on))
        out = np.array([on.x - q.x, on.y - q.y])
        out /= np.linalg.norm(out) or 1.0
        self.guard_moved += 1 if count else 0
        return np.array([on.x, on.y]) + out * GUARD_CLEAR

    # each kind / 種類ごと
    def stops(self):
        for road, j, back in self.extra.get("stop", []):
            lanes = [self.lane_by.get((road, a["lane"])) for a in self.appr.get((road, j), [])]
            lane = self.kerb_lane(road, [l for l in lanes if l is not None])
            if lane is None:
                self.dropped.append(("tomare", road, j, "no lane"))
                continue
            side = ""
            # the post stays within STOP_TOL m of the stop line (drivers, and the TM hold
            # in traffic_profile.py, read the 206 landmark's s) / 停止線から STOP_TOL 以内
            spot, f = self.kerb_spot(lane, -(back + 1.0), kind="tomare", face=lambda f, kn: -f, steps=STOP_STEPS)
            if spot is None:
                # structures on the kerb side: the far side of a one-way road (no traffic
                # the other way to confuse) / 路肩側が塞がる一方通行路: 反対側
                if len({l["lane"] > 0 for l in self.road_lanes.get(road, [])}) == 1:
                    inner = min((l for l in self.road_lanes[road] if len(l["pts"]) >= 3), key=lambda l: l["lane"])
                    spot2, f2 = self.far_spot(inner, -(back + 1.0), kind="tomare", face=lambda f, kn: -f,
                                              steps=STOP_STEPS)
                    if spot2 is not None:
                        spot, f, side = spot2, f2, " (far side, kerb side blocked)"
            if spot is None:
                self.dropped.append(("tomare", road, j, f))
                # no plate fits (walls): the Stencil_STOP 206 still goes beside the stop line
                # 板は置けない（塀）: Stencil_STOP は停止線の脇へ
                q, _ = self.kerb_spot(lane, -(back + 1.0), steps=STOP_STEPS)
                if q is not None:
                    self.stop_posts[road] = q             # else signs.py's s / t stay / 無ければ signs.py の位置のまま
                continue
            post = self.add("tomare", spot, -f, "stop road %d junction %d%s" % (road, j, side))
            # the landmark goes where the post is (moved behind a guardrail) / 支柱の実位置
            self.stop_posts[road] = np.array([post["x"], post["y"]]) if post is not None else spot
            if post is not None:
                # the stop line (lane centre there, travel direction): a post moved off the
                # model's asphalt stays within STOP_TOL of it / 停止線: 移動しても STOP_TOL 以内
                pl, fl, _, _, _ = self.at_s(lane, -back)
                post.setdefault("stops", []).append((road, pl, fl))

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
            if lane is None:
                self.dropped.append(("noturn", P, "no kerb lane on the approach"))   # 路肩側の車線なし
                continue
            stop = {(rr, jj): b for rr, jj, b in self.extra.get("stop", [])}.get(k, 2.0)
            spot, f = self.kerb_spot(lane, -(stop + 1.0), kind="noturn_" + code, face=lambda f, kn: -f)
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
                            not self.L.building_area.buffer(0.3).contains(pt) and not self.clashes(q, -d, "crossing"):
                        spot = q
                        break
                if spot is None:
                    self.dropped.append(("crossing", [round(float(v), 1) for v in c], "no clear kerb spot"))
                    continue
                self.add("crossing", spot, -d, "crossing %s" % cr.get("src", ""), two_faced=True)

    def no_entry(self):
        """303 車両進入禁止 at the exit end of every one-way road segment along a
        JARTIC 一方通行 line (NPA: "exit of a one-way street"), facing the junction
        it leaves into, on the wrong-way driver's left (the far side), else on
        the kerb side. Only roads the road logic also runs one way (no road the
        other way along the line). / 一方通行の出口に進入禁止（逆走車の左側）。"""
        done = set()
        for r in self.rows:
            if r["code"] != jartic.ONEWAY or r["kind"] != 2:
                continue
            along = jartic.lanes_along(r["xy"], self.lanes)
            dirs = {}
            for road, ln in along:
                lane = self.kerb_lane(road)
                if lane is None:
                    continue
                C = np.array([q["c"] for q in lane["pts"]])
                dirs[road] = (lane, C[-1] - C[0])
            if not dirs:
                continue
            ref = max(dirs.values(), key=lambda v: np.linalg.norm(v[1]))[1]
            if any(float(v[1] @ ref) < 0 for v in dirs.values()):
                self.dropped.append(("noentry", "JARTIC #%s" % jartic.short_key(r),
                                     "roads both ways along the line: two-way in the road logic"))
                continue
            for road, (lane, _) in sorted(dirs.items()):
                if road in done:
                    continue
                done.add(road)
                _, f, _, _, length = self.at_s(lane, -0.01)
                if length < 15.0:
                    continue
                # only where the segment ends at a junction (a way in for wrong-way drivers)
                # 交差点で終わる区間のみ（逆走で進入できる所）
                if not any(a["road"] == road for al in self.appr.values() for a in al):
                    self.dropped.append(("noentry", road, "segment does not end at a junction"))
                    continue
                self.noentry_at(road, lane, "JARTIC #%s one-way road %d exit" % (jartic.short_key(r), road))

    def noentry_at(self, road, lane, src):
        """One 303 at the exit end of one-way road ``road`` (kerb lane ``lane``),
        facing the junction: far side, else kerb side, clear of modelled
        structures (sheds, walls, canopies); else just past the lane end in the
        junction mouth (far side first), where such plates stand when the road
        ends between buildings; one per junction mouth. Returns the post, or
        None. / 一方通行路の出口に 303 を 1 枚。"""
        # the lane farthest from the kerb (netconvert keep-left roads: the most negative id)
        # 路肩から最も遠い車線
        inner = min((l for l in self.road_lanes[road] if len(l["pts"]) >= 3), key=lambda l: l["lane"])
        fwd = lambda f, kn: f                                       # noqa: E731
        spot, f = self.far_spot(inner, -3.0, kind="noentry", face=fwd)
        where = "far side"
        if spot is None:
            spot, f = self.kerb_spot(lane, -3.0, kind="noentry", face=fwd)
            where = "kerb side"
        if spot is None:
            spot, f, where = None, "no spot clear of structures at the exit of road %d" % road, None
            for ln, far, w in ((inner, True, "far side"), (lane, False, "kerb side")):
                for q, ff in self.mouth_spots(ln, far):
                    if not self.clashes(q, ff, "noentry"):
                        spot, f, where = q, ff, w + ", junction mouth"
                        break
                if spot is not None:
                    break
        if spot is None:
            self.dropped.append(("noentry", road, f))
            return None
        near = [q for q in self.posts if any(k == "noentry" for k, _, _ in q["plates"])
                and np.hypot(q["x"] - spot[0], q["y"] - spot[1]) < 8.0 and float(q["face"] @ f) > 0.8]
        if near:
            # parallel one-way roads into the same junction: one plate serves both
            # 同じ交差点へ並ぶ一方通行路: 1 枚で兼ねる
            self.merged.append({"kind": "noentry", "src": "road %d" % road, "why": "plate at (%.1f, %.1f) "
                                "for the same junction mouth" % (near[0]["x"], near[0]["y"])})
            return None
        return self.add("noentry", spot, f, "%s (%s)" % (src, where), stack=False)

    def mouth_spots(self, lane, far, steps=12, step=0.5):
        """Post spots just past the end of ``lane`` (into its junction mouth),
        BACK m outside the carriageway on the far / kerb side, clear of lanes
        and buildings. Yields (xy, travel direction). / 車線端の先（交差点の口）の候補。"""
        p, f, kn, half, _ = self.at_s(lane, -0.01)
        n = -kn if far else kn
        for k in range(1, steps + 1):
            q = p + f * step * k + n * half
            for _ in range(200):
                if not self.cw.contains(Point(*(q + n * 0.05))):
                    break
                q = q + n * 0.05
            spot = q + n * BACK
            pt = Point(*spot)
            if self.cw.contains(pt) or not BACK - 0.15 <= self.cw.distance(pt) <= 2.0:
                continue
            if self.L.building_area.buffer(0.3).contains(pt) or not self.lane_clear(pt):
                continue
            yield spot, f

    def clashes(self, spot, face, kind, z_top=None):
        """Names of modelled meshes (not plates, not big terrain triangles) that
        reach into the post + plate envelope: the plate's width, 0.05 m behind
        the pole to 0.1 m in front, from 0.3 m over the ground to the plate top
        (``z_top`` for a stack); exact triangle-box test. / 支柱と板の範囲に入る構造物（屋根・柱・塀など）。"""
        _, w, h = size_of(kind)
        zg = self.ground_z(*spot)
        return self._clash_env(spot, face, w, zg + 0.3, zg + BOTTOM - 0.05,
                               z_top if z_top is not None else zg + BOTTOM + h + 0.05)

    def _structures(self):
        if getattr(self, "_st", None) is None:
            from scipy.spatial import cKDTree
            mesh_dir = os.path.join(self.out, "meshes")
            tris, nm = [], []
            for fn in sorted(os.listdir(mesh_dir)):
                if not fn.endswith(".npz") or fn.startswith("Terrain_JpPlates_"):
                    continue
                d = np.load(os.path.join(mesh_dir, fn))
                for k in d.files:
                    if k.endswith("|V"):
                        T = d[k][d[k[:-2] + "|I"].reshape(-1, 3)].astype(float)
                        a, b = T.min(1), T.max(1)
                        keep = np.hypot(*(b[:, :2] - a[:, :2]).T) < 20.0
                        tris.append(T[keep]), nm.extend([k[:-2]] * int(keep.sum()))
            T = np.concatenate(tris)
            mn, mx = T.min(1), T.max(1)
            nz = None
            self._st = (T, mn, mx, nz, nm, cKDTree((mn[:, :2] + mx[:, :2]) / 2))
        return self._st

    def _clash_env(self, spot, face, w, z0, zb, z1):
        """Triangles meeting the pole (0.1 m round it, z0..zb) or the plate box
        (its width, 0.05 m behind the pole to 0.1 m in front, zb..z1): a post at
        the foot of a bank is fine, a plate in a fence is not. / 支柱部と板部の包絡箱。"""
        T, mn, mx, nz, nm, tree = self._structures()
        face = np.asarray(face, float) / (np.linalg.norm(face) or 1.0)
        along = np.array([-face[1], face[0]])
        c = np.asarray(spot, float)
        hw = w / 2 + 0.05
        # the envelope as an oriented box: along the plate, 0.05 m behind the pole to
        # 0.1 m in front, z0..z1 / 包絡箱
        axes = np.array([[along[0], along[1], 0.0], [face[0], face[1], 0.0], [0.0, 0.0, 1.0]])
        boxes = [(np.array([*(c + face * 0.025), (zb + z1) / 2]), np.array([hw, 0.075, (z1 - zb) / 2])),
                 (np.array([c[0], c[1], (z0 + zb) / 2]), np.array([0.1, 0.1, (zb - z0) / 2]))]
        hits = set()
        for i in tree.query_ball_point(c, 12.0):
            if mx[i, 2] < z0 or mn[i, 2] > z1 or nm[i] in hits:
                continue
            if any(tri_box(((T[i] - ctr) @ axes.T), half) for ctr, half in boxes):
                hits.add(nm[i])
        return sorted(hits)

    def clash_rec(self, r):
        """clashes() for a placed plate record (jp_plates.json). / 配置済み板の干渉。"""
        a = math.radians(r["face_deg"])
        x, y = r["x"], r["y"]
        if r.get("signal"):                       # on a signal pole: the plate stands in front of it / 信号柱の前
            return []
        return self._clash_env((x, y), (math.cos(a), math.sin(a)), r["w"], r["ground"] + 0.3,
                               r["z"] - r["h"] / 2 - 0.05, r["z"] + r["h"] / 2)

    def oneway_at(self, road, lane, src, first=4.0):
        """One 326-A near the start of one-way road ``road``, along the road
        (readable from the junction, arrow with the traffic), two-faced, as the
        JARTIC lines get it. Returns the post, or None. / 一方通行路の始点に 326-A。"""
        spot, f = self.kerb_spot(lane, first, slide=1.0, kind="oneway", face=lambda f, kn: -kn)
        if spot is None:
            self.dropped.append(("oneway", road, first, f))
            return None
        _, ff, kn, _, _ = self.at_s(lane, first)
        return self.add("oneway" + ("" if self.left else "_l"), spot, -kn, src, two_faced=True, stack=False)

    def verified(self):
        """Plates seen in Street View on roads the JARTIC data lacks
        (<out>/verified_plates.json "plates": kind oneway / noentry, end entry /
        exit, xy = that road end in the map frame, evidence = image path). The
        road is the one whose end lies within VERIFIED_MATCH m of xy; it must run
        one way in the road logic (else dropped: the plate would contradict the
        traffic) and, for a 303, end at a junction. "none" = ends checked, no
        plate (only logged). / 現地確認済みの一方通行・進入禁止を同じ規則で置く。"""
        self.verified_log = []
        path = os.path.join(self.out, VERIFIED)
        if not os.path.exists(path):
            return
        data = json.load(open(path, encoding="utf-8"))
        for v in data.get("plates", []):
            kind, end = v["kind"], v.get("end", "exit" if v["kind"] == "noentry" else "entry")
            xy = np.asarray(v["xy"], float)
            best = None
            for road, ls in self.road_lanes.items():
                lane = self.kerb_lane(road, ls)
                if lane is None:
                    continue
                for l in ls:                       # any lane's end (a two-way road is dropped below)
                    if len(l["pts"]) < 2:
                        continue
                    c = np.asarray(l["pts"][-1 if end == "exit" else 0]["c"], float)
                    d = float(np.hypot(*(c - xy)))
                    if d <= VERIFIED_MATCH and (best is None or d < best[0]):
                        best = (d, road, lane)
            log = {"kind": kind, "end": end, "xy": v["xy"], "evidence": v.get("evidence", "")}
            if best is None:
                log["result"] = "dropped: no road end within %.0f m" % VERIFIED_MATCH
                self.verified_log.append(log)
                continue
            _, road, lane = best
            log["road"] = road
            if v.get("road") is not None and int(v["road"]) != road:
                log["note"] = "road id %s in the file, %d in this road logic" % (v["road"], road)
            if len({l["lane"] > 0 for l in self.road_lanes[road]}) != 1:
                log["result"] = "dropped: road %d is two-way in the road logic" % road
                self.verified_log.append(log)
                continue
            src = "Street View %s road %d %s (%s)" % (v.get("date", ""), road, end, v.get("evidence", ""))
            if kind == "noentry":
                if not any(a["road"] == road for al in self.appr.values() for a in al):
                    log["result"] = "dropped: road %d does not end at a junction" % road
                    self.verified_log.append(log)
                    continue
                post = self.noentry_at(road, lane, src)
            elif kind == "oneway":
                post = self.oneway_at(road, lane, src)
            else:
                log["result"] = "dropped: unknown kind %s" % kind
                self.verified_log.append(log)
                continue
            log["result"] = ("placed at (%.2f, %.2f)" % (post["x"], post["y"])) if post is not None else \
                "not placed (see dropped / merged)"
            self.verified_log.append(log)
        for v in data.get("unresolved", []):
            # a real plate whose meaning or spot is not pinned down, or which would
            # contradict the road logic's lanes: logged, never placed / 未解決: 置かない
            self.verified_log.append({"kind": v.get("kind"), "end": v.get("end"), "xy": v.get("post_xy", v.get("xy")),
                                      "evidence": v.get("evidence", ""), "result": "unresolved, not placed: "
                                      + v.get("seen", "")})
        for v in data.get("none", []):
            self.verified_log.append({"kind": v.get("kind"), "end": v.get("end"), "xy": v.get("xy"),
                                      "evidence": v.get("evidence", ""), "result": "no plate in Street View: "
                                      + v.get("seen", "")})

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
                    fc = (lambda f, kn: -kn) if parallel else (lambda f, kn: -f)
                    spot, f = self.kerb_spot(lane, s, slide=1.0, kind=kind, face=fc)
                    s_used = s
                    why0 = f
                    if spot is None and s == first:
                        # blocked (a fence along the road's start): step on along the road,
                        # which the regulation covers whole, up to the next plate's s
                        # 塀で塞がる: 規制区間内で次の板の手前まで先へ
                        end = ss[1] if len(ss) > 1 else length - 5.0
                        for s2 in np.arange(s + 9.0, end, 9.0):
                            spot, f = self.kerb_spot(lane, float(s2), slide=1.0, kind=kind, face=fc)
                            if spot is not None:
                                s_used = float(s2)
                                break
                        if spot is None:
                            f = "%s; nothing clear further along the road (%.0f m) either" % (why0, length)
                    if spot is None:
                        self.dropped.append((kind, road, s, f))
                        continue
                    if s_used != s:
                        self.moved_on = getattr(self, "moved_on", []) + [(kind, road, round(s_used, 1))]
                    k = kind
                    face = -f
                    # a plate stepped on along its road counts only plates on its own line
                    # (within SAME_LINE m sideways) as the same traffic / 先へ移した板は同じ線上の板のみ数える
                    near_line = (lambda q: True) if s_used == s else \
                        (lambda q: abs(f[0] * (q["y"] - spot[1]) - f[1] * (q["x"] - spot[0])) < SAME_LINE)
                    if space and any(q["kind"] == kind and float(q["face"] @ -f) > 0.8 and near_line(q) and
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
        self.no_entry()
        self.verified()
        self.along(jartic.NO_PARKING, "noparking", every=150.0, space=120.0)
        self.along(jartic.BICYCLE_PAVEMENT, "bicycle", first=10.0, min_len=30.0, space=120.0)
        self.settle_core()
        return self

    def core_clear(self, post):
        """How far a post must stand off the embedded model's asphalt: CORE_CLEAR,
        or its widest plate's half width + 5 cm (plate edge off the road).
        / モデルの車道からの必要距離"""
        return max([CORE_CLEAR] + [size_of(k)[1] / 2 + 0.05 for k, _, _ in post["plates"]])

    def core_spots(self, x, y, reach=CORE_REACH, step=CORE_STEP):
        """Candidate spots round (x, y) up to ``reach`` m, nearest first (rings
        every ``step`` m, points ``step`` m apart on each). / 近い順の候補点"""
        out = []
        for r in np.arange(step, reach + 1e-9, step):
            n = max(8, int(math.ceil(2 * math.pi * r / step)))
            a = np.arange(n) * 2 * math.pi / n
            out.append(np.stack([x + r * np.cos(a), y + r * np.sin(a)], 1))
        return np.concatenate(out)

    def settle_core(self):
        """Embedded model only: a post (not one on a signal pole) on its asphalt
        or nearer it than core_clear() moves to the nearest spot within
        CORE_REACH m that is off that asphalt and the layout carriageway by
        core_clear(), within 2.0 m of the kerb (their edge), clear of every
        driving lane, building and modelled structure, POST_GAP m from other
        posts, on the same side of the road (the move crosses no driving lane),
        and for a 止まれ 0-2.95 m before its stop line; the plates keep their
        facing. No such spot: the post is left out (self.core_dropped).
        Off asphalt this does nothing, so maps without a core are unchanged.
        モデルのアスファルト上の支柱を最寄りの歩道・縁石上へ（無理なら外す）。"""
        import shapely
        from shapely.geometry import LineString
        if self.core_road is None:
            return 0, 0
        shapely.prepare(self.core_road)
        shapely.prepare(self.cw)
        shapely.prepare(self.lane_area)
        shapely.prepare(self.kerb_edge)
        bld = self.L.building_area.buffer(0.3)
        shapely.prepare(bld)
        keep = []
        for p in self.posts:
            if p.get("signal"):
                keep.append(p)                       # on a CARLA signal pole: it stays there / 信号柱付きはそのまま
                continue
            clear = self.core_clear(p)
            if self.core_road.distance(Point(p["x"], p["y"])) >= clear:
                keep.append(p)
                continue
            Q = self.core_spots(p["x"], p["y"])
            ok = shapely.distance(self.core_road, shapely.points(Q)) >= clear
            ok &= ~shapely.contains_xy(self.cw, Q[:, 0], Q[:, 1])
            ok[ok] = shapely.distance(self.cw, shapely.points(Q[ok])) >= clear
            ok[ok] = shapely.distance(self.kerb_edge, shapely.points(Q[ok])) <= 2.0
            ok[ok] = shapely.distance(self.lane_area, shapely.points(Q[ok])) >= LANE_CLEAR + POLE_R
            ok[ok] = ~shapely.contains_xy(bld, Q[ok, 0], Q[ok, 1])
            others = np.array([[o["x"], o["y"]] for o in self.posts if o is not p] or [[1e9, 1e9]])
            for i in np.flatnonzero(ok):
                q = Q[i]
                for road, pl, fl in p.get("stops", []):
                    if not 0.0 <= float((pl - q) @ fl) <= STOP_TOL - 0.1:
                        break
                else:
                    if np.hypot(*(others - q).T).min() < POST_GAP:
                        continue
                    if shapely.intersects(self.lane_area, LineString([(p["x"], p["y"]), tuple(q)])):
                        continue                     # across a lane: the other side of a road / 車線の向こう側
                    top = self.ground_z(*q) + BOTTOM + sum(size_of(k)[2] for k, _, _ in p["plates"]) + 0.05
                    if any(self.clashes(q, p["face"], k, z_top=top) for k, _, _ in p["plates"]):
                        continue
                    break
            else:
                self.core_dropped.append({"x": round(p["x"], 2), "y": round(p["y"], 2), "kinds": [k for k, _, _ in p[
                    "plates"]], "src": [s for _, _, s in p["plates"]], "why": "on the embedded model's asphalt, "
                    "no free kerb spot within %.1f m" % CORE_REACH})
                for k, _, s in p["plates"]:
                    self.dropped.append((k, s, "on the embedded model's asphalt, no free kerb spot within %.1f m"
                                         % CORE_REACH))
                continue
            self.core_moved.append({"from": [round(p["x"], 2), round(p["y"], 2)],
                                    "to": [round(float(q[0]), 2), round(float(q[1]), 2)],
                                    "dist": round(float(np.hypot(q[0] - p["x"], q[1] - p["y"])), 2),
                                    "kinds": [k for k, _, _ in p["plates"]]})
            for road, _, _ in p.get("stops", []):
                if road in self.stop_posts and np.allclose(self.stop_posts[road], (p["x"], p["y"])):
                    self.stop_posts[road] = np.array(q, float)    # the 206 follows its post / 206 も移す
            p["x"], p["y"] = float(q[0]), float(q[1])
            keep.append(p)
        # a dropped 止まれ post: its Stencil_STOP 206 (no mesh) keeps that spot by the line
        # 外した止まれの 206（描画なし）は停止線脇の元の位置のまま
        self.posts = keep
        return len(self.core_moved), len(self.core_dropped)

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
            rr = SIGNAL_R if p.get("signal") else POLE_R       # on a signal pole: in front of it / 信号柱の前面
            for (k, two, src), (shape, w, h) in zip(p["plates"], sizes):
                cz = z - h / 2
                ctr = (x + nf[0] * (rr + PLATE_T), y + nf[1] * (rr + PLATE_T), cz)
                # a one-way plate's back shows the arrow the other way round (same direction on the road)
                # 一方通行の裏面は逆向きの図柄（道路上では同じ向き）
                bf = None
                if k.startswith("oneway"):
                    bf = "jp_plate_oneway" if k.endswith("_l") else "jp_plate_oneway_l"
                    back_kinds.add(bf[9:])
                plate_mesh(m, "jp_plate_" + k, "jp_plate_back", ctr, nf, shape, w, h, two_faced=two, back_face=bf)
                rec.append({"kind": k, "x": round(x, 2), "y": round(y, 2), "z": round(cz, 2),
                            "ground": round(zg, 2), "face_deg": round(math.degrees(math.atan2(nf[1], nf[0])), 1),
                            "w": w, "h": h, "src": src, **({"signal": p["signal"]} if p.get("signal") else {})})
                z -= h + GAP
            if not p.get("signal"):                        # a signal pole carries it: no white pole / 信号柱に付ける
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
                   "jartic_log": self.extra.get("jartic_log", []),
                   # Street View verified plates (verified_plates.json) / 現地確認済みの標識
                   "verified_log": getattr(self, "verified_log", []),
                   # embedded model: posts moved off / left out for its asphalt / モデルの車道から移動・除外
                   "core_moved": getattr(self, "core_moved", []), "core_dropped": getattr(self, "core_dropped", []),
                   # signal poles as computed here (check_signal_poles.py compares carla.Map) / 信号柱位置
                   "signal_poles": [[i, round(x, 3), round(y, 3)] for i, x, y in self.signal_poles()]},
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


def _map_entry(name):
    """maps.json entry of the map called ``name`` ({} if none). / maps.json の項目。"""
    try:
        for v in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json"))).values():
            if isinstance(v, dict) and v.get("name") == name:
                return v
    except (OSError, ValueError):
        pass
    return {}


def stop_offsets(xodr, items):
    """For every Stencil_STOP 206 of a stopped approach: (road, its s, the stop
    line's s, metres before the line). ``items``: (road, s at the approach lane's
    start, s at its end, back = the line's distance from that end).
    Stencil_STOP と停止線の距離。"""
    import xml.etree.ElementTree as ET
    root = ET.parse(xodr).getroot() if os.path.exists(xodr) else ET.fromstring(xodr)
    sig = {}
    for road in root.findall("road"):
        for x in road.iter("signal"):
            if x.get("type") == "206" and x.get("name") == "Stencil_STOP":
                sig.setdefault(int(road.get("id")), []).append(float(x.get("s")))
    out = []
    for road, s0, s1, back in items:
        fwd = s1 >= s0
        line = s1 - back if fwd else s1 + back
        for sv in sig.get(road, []):
            out.append((road, sv, line, (line - sv) if fwd else (sv - line)))
    return out


def stop_items(placer):
    """stop_offsets() items from crossings.pkl stops and the approach lanes. / 照合用の停止線。"""
    items = []
    for road, j, back in placer.extra.get("stop", []):
        lanes = [placer.lane_by.get((road, a["lane"])) for a in placer.appr.get((road, j), [])]
        lane = placer.kerb_lane(road, [l for l in lanes if l is not None])
        if lane is not None:
            items.append((road, lane["pts"][0]["s"], lane["pts"][-1]["s"], back))
    return items


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
        # the kerb: the carriageway's edge, in an embedded model its asphalt's / 縁石 = 車道（モデルのアスファルト）の端
        kerb = getattr(placer, "kerb_edge", placer.cw)
        d = kerb.distance(pt)
        if kerb.contains(pt) or not 0.3 <= d <= 2.0:
            bad.append(("distance %.2f m" % d, r))
        core = getattr(placer, "core_road", None)
        if core is not None and not r.get("signal") and core.distance(pt) < max(CORE_CLEAR, r["w"] / 2 + 0.05) - 0.01:
            bad.append(("post %.2f m from the embedded model's asphalt" % core.distance(pt), r))
        half = r["w"] / 2
        if r["z"] - r["h"] / 2 - r["ground"] < 1.8 - 1e-6:
            bad.append(("low plate", r))
        if d < half and r["z"] - r["h"] / 2 - r["ground"] < 4.5:
            bad.append(("plate over the carriageway below 4.5 m", r))
        sp = placer.signal_pole_near((r["x"], r["y"]))
        if sp is not None and r.get("signal") != sp[0]:
            bad.append(("post %.2f m from signal pole %s (< %.1f)" % (math.hypot(sp[1] - r["x"], sp[2] - r["y"]),
                                                                    sp[0], SIGNAL_CLEAR), r))
        hit = placer.clash_rec(r)
        if hit:
            bad.append(("post / plate into modelled structures %s" % hit, r))
        if placer.guard is not None and placer.guard.distance(pt) < 0.2 + POLE_R:
            bad.append(("post %.2f m from the guardrail beam (< 0.2)" % placer.guard.distance(pt), r))
    xodr = os.path.join(placer.out, placer.meta["name"] + ".xodr")
    for road_id, sv, line, d in stop_offsets(xodr, stop_items(placer)):
        if not -0.5 <= d <= STOP_TOL:
            bad.append(("Stencil_STOP on road %d at s %.2f, %.2f m before its stop line (s %.2f; allowed -0.5..%.2f)"
                        % (road_id, sv, d, line, STOP_TOL), {}))
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
    on_sig = sorted({(p["signal"], round(p["x"], 1), round(p["y"], 1)) for p in P.posts if p.get("signal")})
    print("jp_signs: %d posts on traffic-signal poles: %s" % (len(on_sig), on_sig))
    print("jp_signs: %d Stencil_STOP landmarks moved to their 止まれ posts" % n_moved)
    if P.guard is not None:
        print("jp_signs: %d posts moved behind the ring guardrail" % P.guard_moved)
    if P.core_road is not None:
        print("jp_signs: %d posts moved off the embedded model's asphalt (median %.2f m, max %.2f m), %d left out"
              % (len(P.core_moved), float(np.median([m["dist"] for m in P.core_moved] or [0])),
                 max([m["dist"] for m in P.core_moved] or [0]), len(P.core_dropped)))
    for d in P.dropped:
        print("  dropped: %s" % (d,))
    for m in P.merged:
        print("  merged: %s" % (m,))
    for m in getattr(P, "moved_on", []):
        print("  stepped on along the road (structures at its start): %s road %d at s %.1f m" % m)
    for v in getattr(P, "verified_log", []):
        print("  verified: %s %s road %s: %s" % (v["kind"], v["end"], v.get("road", "-"), v["result"]))
    bad, over = check(P, rec)
    for why, r in bad:
        print("  CHECK %s: %s" % (why, r))
    print("check: %d plate problems, %d plate vertices over the carriageway (must be 0)" % (len(bad), over))


if __name__ == "__main__":
    main()
