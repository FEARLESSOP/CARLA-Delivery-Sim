"""Japanese shop fronts (店先): entrance doors on the generated shops and the
props in front of them, on every Japan map (maps.json "road_style": "japan").

Two halves:

* doors (called by buildings_jp while it draws a shop / office podium front):
  the shopfront glazing is cut into tenant units (one shop per 5-9 m), each
  with its own use (eatery, cafe, konbini, retail; OSM amenity/shop points
  inside the footprint win, else a seeded draw) and its own entrance: an
  automatic glass door (自動ドア), a single glass door, or a wooden lattice
  sliding door (格子戸) under a noren (暖簾) for eateries. Doors and noren stay
  inside the 0.3 m shopfront recess (never over a footway). Every door is
  recorded (it.doors); build.py writes them to <build>/shopfronts.json.
* props (``python shopfront_jp.py <build dir>``, after core_merge.py): in front
  of the recorded doors, seeded per door (stable across rebuilds):
  自動販売機 (single or a pair, back to the wall beside the door or at the
  facade end, the recycling box beside them), A型看板 / スタンド看板 (lit white
  or the dark wooden stand seen at Oita station doorways) by eatery
  and cafe doors, 鉢植え and プランター by doors, now and then のぼり旗.
  Each prop stands on the real ground under it (lot terrace or pavement,
  read from the built meshes; the model's ground inside a "core"), off the
  carriageway, kerb, 路側帯, tactile paving, crossings, drives and the road
  logic (+0.3 m), clear of every built object up to its own height (trees,
  lamps, plate and signal poles, pedestrian heads, walls, stairs, AC units;
  poles +0.4 m), of CARLA's signal poles, bus stops (3 m) and door openings
  (doors 1.5 m out), and leaves the footway walkable (2 m clear in front of
  it, or the footway width - 0.6 m where narrower). Objects
  Terrain_JpShopfront_<Name>_<tile> (kind "jp_shopfront"), own textures.

No "sign" / "light" in any name (CARLA's importer drops those): kanban,
jihanki, hachi, nobori.

日本の店先: 生成した店舗の入口（自動ドア・格子戸と暖簾）と、その前の自販機・
A 型看板・スタンド看板・鉢植え・プランター・のぼり旗。
"""

import json
import math
import os
import random
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mesh import Mesh  # noqa: E402

KIND = "jp_shopfront"
PREFIX = "Terrain_JpShopfront_"

# -- materials: texture, colour, roughness, metallic, smooth (manifest spec) / 材質
DOOR_MATS = {
    "jp_door_auto": 0.08, "jp_door_glass1": 0.08, "jp_door_koshi": 0.55,
    "jp_noren_navy": 0.8, "jp_noren_red": 0.8, "jp_noren_white": 0.8, "jp_noren_brown": 0.8,
    "jp_noren_rod": 0.6,
}
BRANDS = ("red", "blue", "white", "green")
NOBORI = ("red", "yellow", "blue", "green", "white")
PROP_MATS = {
    **{"jihanki_front_" + b: 0.15 for b in BRANDS},
    **{"jihanki_body_" + b: 0.35 for b in BRANDS},
    **{"jihanki_bin_" + b: 0.4 for b in BRANDS},
    "kanban_a_chalk": 0.75, "kanban_a_menu": 0.3, "kanban_frame": 0.6, "kanban_stand": 0.2, "kanban_stand_dark": 0.5,
    "kanban_stand_body": 0.4, "hachi_terracotta": 0.8, "hachi_glaze_navy": 0.2, "hachi_glaze_brown": 0.25,
    "hachi_plastic": 0.5, "hachi_leaves": 0.7, "hachi_flowers": 0.7, "hachi_soil": 0.95,
    **{"nobori_" + c: 0.75 for c in NOBORI},
    "nobori_pole": 0.35, "nobori_base": 0.5,
}
METAL = {"nobori_pole": 0.6, "jp_noren_rod": 0.0}

# prop sizes (m) / 小物の寸法
JIHANKI = (1.0, 0.72, 1.83)          # width, depth, height (standard 1,030 x 720 x 1,830)
BIN = (0.42, 0.38, 0.92)             # recycling box / リサイクルボックス
GAP = 0.06                           # back to the wall / 壁との隙間
WALK_CLEAR = 2.0                     # footway kept clear in front of a prop / 歩道の有効幅
WALK_NARROW = 0.6                    # narrower footways: width - this / 狭い歩道
POLE_CLEAR = 0.4                     # trees, lamps, plates, signal poles / 柱類との離隔
ROAD_CLEAR = 0.3                     # carriageway, kerb, tactile, drives / 車道等との離隔
DOOR_OUT = 1.5                       # door opening kept free this far out / 出入口の前
BUS_CLEAR = 3.0                      # bus stops / バス停
RES = 0.05                           # local raster (m) / 局所ラスタ


# ============================================================ textures / テクスチャ
def _rng(name):
    import hashlib
    return np.random.default_rng(int(hashlib.md5(name.encode()).hexdigest()[:8], 16))


def _noise(h, w, rng, sigma=3.0):
    a = rng.standard_normal((h, w))
    img = Image.fromarray(((a - a.min()) / (np.ptp(a) + 1e-9) * 255).astype(np.uint8))
    a = np.asarray(img.filter(ImageFilter.GaussianBlur(sigma)), np.float64) / 255.0
    return (a - a.mean()) / (a.std() + 1e-9)


def _glyphs(dr, x0, y0, size, n, col, rng, vertical=True, width=None):
    """n kana/kanji-like characters drawn as strokes (abstract, own lettering).
    抽象的な文字（自作の筆画）。"""
    width = width or max(1, int(size / 7))
    for k in range(n):
        cx = x0 + (0 if vertical else k * size * 1.05)
        cy = y0 + (k * size * 1.05 if vertical else 0)
        for _ in range(int(rng.integers(2, 5))):
            kind = rng.integers(0, 3)
            ax, ay = cx + rng.uniform(0.1, 0.9) * size, cy + rng.uniform(0.1, 0.9) * size
            if kind == 0:
                bx, by = ax + rng.uniform(-0.5, 0.5) * size, ay
            elif kind == 1:
                bx, by = ax, ay + rng.uniform(-0.5, 0.5) * size
            else:
                bx, by = ax + rng.uniform(-0.4, 0.4) * size, ay + rng.uniform(-0.4, 0.4) * size
            bx = min(max(bx, cx + 0.05 * size), cx + 0.95 * size)
            by = min(max(by, cy + 0.05 * size), cy + 0.95 * size)
            dr.line([(ax, ay), (bx, by)], fill=col, width=width)


def _grime(img, rng, amount=0.12):
    a = np.asarray(img, np.float64)
    h, w = a.shape[:2]
    n = _noise(h, w, rng, 6.0)
    foot = np.clip((np.arange(h)[:, None] - 0.8 * h) / (0.2 * h), 0, 1)
    a = a * (1 - amount * (0.5 + 0.5 * n)[..., None] * 0.4 - amount * foot[..., None])
    return Image.fromarray(a.clip(0, 255).astype(np.uint8))


BRAND_COL = {"red": (196, 24, 32), "blue": (24, 82, 168), "white": (236, 236, 232), "green": (30, 128, 72)}
BRAND_ACC = {"red": (250, 250, 250), "blue": (250, 250, 250), "white": (40, 110, 190), "green": (250, 240, 200)}


def tex_jihanki_front(brand, W=320, H=586):
    """Front of a drinks machine (1.0 x 1.83 m; image top = machine top): lit
    display window with three rows of cans and bottles, price strips and
    buttons, brand panel, coin / bill unit, product outlet.
    自販機の正面（照明付きの見本棚・ボタン・硬貨投入口・取出口）。"""
    rng = _rng("jihanki" + brand)
    ppm = W / 1.0
    body, acc = BRAND_COL[brand], BRAND_ACC[brand]
    im = Image.new("RGB", (W, H), body)
    dr = ImageDraw.Draw(im)
    Z = lambda z: int(H - z * ppm)          # metres from the bottom -> row / 下からの高さ -> 行
    X = lambda x: int(x * ppm)
    # display window 1.05..1.74 m, x 0.05..0.95 / 見本窓
    dr.rectangle([X(0.04), Z(1.76), X(0.96), Z(1.02)], fill=(70, 72, 76))
    dr.rectangle([X(0.06), Z(1.74), X(0.94), Z(1.04)], fill=(246, 248, 250))
    palette = [(200, 30, 30), (30, 60, 160), (240, 200, 40), (40, 140, 60), (230, 120, 30), (120, 60, 30),
               (240, 240, 240), (20, 20, 20), (150, 200, 230), (200, 80, 140)]
    for r, (z0, z1) in enumerate(((1.52, 1.72), (1.30, 1.50), (1.08, 1.28))):
        n = 9 if r < 2 else 7
        for k in range(n):
            x0 = 0.08 + k * (0.84 / n)
            wd = 0.84 / n * 0.72
            c = palette[int(rng.integers(0, len(palette)))]
            bottle = rng.random() < (0.5 if r == 2 else 0.25)
            top = z1 - 0.02 if bottle else z0 + 0.12
            dr.rectangle([X(x0), Z(top), X(x0 + wd), Z(z0 + 0.04)], fill=c)
            dr.rectangle([X(x0), Z(top - 0.03), X(x0 + wd), Z(top - 0.05)], fill=(250, 250, 250))
            if bottle:
                dr.rectangle([X(x0 + wd * 0.3), Z(top + 0.02), X(x0 + wd * 0.7), Z(top - 0.01)], fill=(230, 230, 230))
            # price strip and button / 価格表示とボタン
            dr.rectangle([X(x0), Z(z0 + 0.03), X(x0 + wd), Z(z0)], fill=(20, 20, 20))
            dr.rectangle([X(x0 + wd * 0.25), Z(z0 + 0.025), X(x0 + wd * 0.75), Z(z0 + 0.005)], fill=(60, 140, 240))
    # brand panel with lettering / ブランド面
    dr.rectangle([X(0.06), Z(0.98), X(0.68), Z(0.50)], fill=tuple(int(c * 0.85) for c in body))
    _glyphs(dr, X(0.10), Z(0.92), 0.11 * ppm, 4, acc, rng, vertical=False, width=4)
    dr.rectangle([X(0.10), Z(0.66), X(0.62), Z(0.62)], fill=acc)
    # coin / bill unit / 硬貨・紙幣投入部
    dr.rectangle([X(0.72), Z(1.00), X(0.94), Z(0.44)], fill=(48, 50, 54))
    dr.rectangle([X(0.76), Z(0.94), X(0.90), Z(0.88)], fill=(40, 160, 90))          # display / 表示
    dr.rectangle([X(0.80), Z(0.82), X(0.86), Z(0.74)], fill=(180, 180, 176))        # coin slot / 硬貨
    dr.rectangle([X(0.76), Z(0.68), X(0.90), Z(0.65)], fill=(20, 20, 20))           # bill slot / 紙幣
    dr.rectangle([X(0.78), Z(0.56), X(0.88), Z(0.48)], fill=(120, 120, 118))        # return / 返却口
    # product outlet / 取出口
    dr.rectangle([X(0.10), Z(0.40), X(0.70), Z(0.14)], fill=(30, 30, 32))
    dr.rectangle([X(0.12), Z(0.38), X(0.68), Z(0.24)], fill=(90, 92, 96))
    dr.rectangle([X(0.0), Z(0.08), X(1.0), Z(0.0)], fill=(50, 50, 52))               # plinth / 台座
    return _grime(im, rng, 0.06)


def tex_jihanki_body(brand, W=128, H=232):
    """Sides and top of the machine: brand colour, a stripe, grime at the foot.
    側面・天面（ブランド色・帯・足元の汚れ）。"""
    rng = _rng("jhbody" + brand)
    im = Image.new("RGB", (W, H), BRAND_COL[brand])
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, int(H * 0.30), W, int(H * 0.36)], fill=BRAND_ACC[brand])
    dr.rectangle([0, H - int(H * 0.045), W, H], fill=(50, 50, 52))
    return _grime(im, rng, 0.1)


def tex_bin(brand, W=128, H=280):
    """Recycling box front (0.42 x 0.92 m): two round holes, a label.
    リサイクルボックス（投入口 2 つ・表示）。"""
    rng = _rng("bin" + brand)
    col = BRAND_COL[brand] if brand != "white" else (40, 110, 190)
    im = Image.new("RGB", (W, H), col)
    dr = ImageDraw.Draw(im)
    dr.rectangle([8, 40, W - 8, 120], fill=(244, 244, 240))
    _glyphs(dr, 14, 52, 22, 4, (40, 40, 40), rng, vertical=False, width=2)
    for cx in (W * 0.3, W * 0.7):
        dr.ellipse([cx - 16, 8, cx + 16, 36], fill=(20, 20, 22))
    dr.rectangle([0, H - 12, W, H], fill=(60, 60, 60))
    return _grime(im, rng, 0.1)


def tex_kanban_a(kind, W=200, H=320):
    """A-frame board face (0.5 x 0.8 m): chalkboard menu or printed menu.
    A 型看板の面（黒板メニュー・印刷メニュー）。"""
    rng = _rng("kanban_a" + kind)
    if kind == "chalk":
        im = Image.new("RGB", (W, H), (34, 44, 38))
        dr = ImageDraw.Draw(im)
        for row in range(7):
            col = [(240, 240, 230), (250, 220, 90), (240, 150, 170)][row % 3]
            _glyphs(dr, 14, 18 + row * 42, 18, int(rng.integers(4, 8)), col, rng, vertical=False, width=2)
        a = np.asarray(im, np.float64) + 10 * _noise(H, W, rng, 2.0)[..., None]
        return Image.fromarray(a.clip(0, 255).astype(np.uint8))
    im = Image.new("RGB", (W, H), (248, 246, 238))
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, 0, W, 54], fill=(200, 36, 30))
    _glyphs(dr, 18, 10, 32, 4, (255, 255, 255), rng, vertical=False, width=4)
    for k in range(3):
        y = 66 + k * 82
        food = [(200, 140, 60), (220, 180, 90), (170, 60, 40), (90, 140, 60)][int(rng.integers(0, 4))]
        dr.rectangle([12, y, 92, y + 66], fill=food)
        dr.ellipse([22, y + 8, 82, y + 58], fill=tuple(min(255, c + 40) for c in food))
        _glyphs(dr, 102, y + 6, 16, 4, (40, 40, 40), rng, vertical=False, width=2)
        _glyphs(dr, 102, y + 36, 16, 3, (200, 36, 30), rng, vertical=False, width=3)
    return im


def tex_kanban_stand(W=180, H=400, dark=False):
    """Stand board face (0.45 x 1.0 m): lit white panel with a coloured head and
    menu, or (dark) the wooden stand seen by Oita station doorways: dark brown,
    a big white vertical name. スタンド看板（白い発光面、または焦茶に白の縦書き）。"""
    rng = _rng("kanban_stand" + ("_dark" if dark else ""))
    if dark:
        im = Image.new("RGB", (W, H), (58, 36, 26))
        dr = ImageDraw.Draw(im)
        dr.rectangle([8, 8, W - 8, H - 8], outline=(150, 110, 60), width=4)
        _glyphs(dr, W / 2 - 34, 30, 64, 5, (250, 248, 240), rng, vertical=True, width=7)
        dr.rectangle([W - 40, 20, W - 24, 70], fill=(200, 40, 30))
        return im
    im = Image.new("RGB", (W, H), (252, 252, 248))
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, 0, W, 90], fill=(220, 120, 20))
    _glyphs(dr, W / 2 - 22, 6, 40, 2, (255, 255, 255), rng, vertical=True, width=5)
    _glyphs(dr, 26, 110, 30, 8, (30, 30, 30), rng, vertical=True, width=4)
    _glyphs(dr, 100, 110, 30, 6, (200, 30, 30), rng, vertical=True, width=4)
    dr.rectangle([0, H - 8, W, H], fill=(60, 60, 60))
    return im


def tex_nobori(col, W=90, H=360):
    """Banner (0.45 x 1.8 m): ground colour, a column of big glyphs, header band,
    pole loops (乳) on the left edge. のぼり旗（地色・縦の大きな文字・乳）。"""
    rng = _rng("nobori" + col)
    ground = {"red": (210, 30, 30), "yellow": (250, 210, 30), "blue": (30, 90, 190), "green": (40, 140, 60),
              "white": (245, 245, 240)}[col]
    ink = {"red": (255, 255, 255), "yellow": (200, 30, 30), "blue": (255, 255, 255), "green": (255, 255, 255),
           "white": (200, 30, 30)}[col]
    im = Image.new("RGB", (W, H), ground)
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, 0, W, 26], fill=ink)
    _glyphs(dr, 18, 34, 56, 5, ink, rng, vertical=True, width=7)
    for k in range(8):
        y = 10 + k * 45
        dr.rectangle([0, y, 7, y + 12], fill=tuple(int(c * 0.7) for c in ground))
    return im


def tex_pot(kind, W=128, H=64):
    rng = _rng("pot" + kind)
    col = {"terracotta": (176, 92, 54), "glaze_navy": (36, 50, 92), "glaze_brown": (92, 58, 36),
           "plastic": (90, 96, 92)}[kind]
    a = np.ones((H, W, 3)) * col
    a *= (1 + 0.06 * _noise(H, W, rng, 2.0))[..., None]
    a[:10] *= 1.15                                             # rim band / 縁
    if kind.startswith("glaze"):
        a[10:, :] *= (1 + 0.25 * np.clip(np.sin(np.arange(W) / 5.0), 0, 1))[None, :, None]
    return Image.fromarray(a.clip(0, 255).astype(np.uint8))


def tex_leaves(flowers=False, W=256, H=256):
    """Leaf cluster (opaque, no alpha: UE takes no cut-outs here); flowers dotted.
    葉の塊（不透明）。花は点で。"""
    rng = _rng("leaves" + str(flowers))
    a = np.zeros((H, W, 3))
    base = np.array((44, 86, 38.0))
    a[:] = base * 0.6
    im = Image.fromarray(a.astype(np.uint8))
    dr = ImageDraw.Draw(im)
    for _ in range(900):
        x, y = rng.uniform(0, W), rng.uniform(0, H)
        r = rng.uniform(4, 10)
        g = base * rng.uniform(0.7, 1.5)
        dr.ellipse([x - r, y - r * 0.5, x + r, y + r * 0.5], fill=tuple(int(v) for v in g.clip(0, 255)))
    if flowers:
        for _ in range(260):
            x, y = rng.uniform(0, W), rng.uniform(0, H)
            c = [(230, 60, 80), (250, 200, 40), (250, 250, 250), (200, 90, 200), (250, 130, 40)][int(rng.integers(0, 5))]
            dr.ellipse([x - 4, y - 4, x + 4, y + 4], fill=c)
    return im


def tex_door(kind, W=256, H=320):
    """Entrance doors (image = the whole opening): 自動ドア (two glass leaves,
    aluminium frame, sticker band), single glass door with a bar handle,
    格子戸 (wooden lattice over frosted paper glass). 入口の扉。"""
    rng = _rng("door" + kind)
    if kind == "koshi":
        im = Image.new("RGB", (W, H), (226, 220, 200))
        dr = ImageDraw.Draw(im)
        wood = (92, 60, 36)
        for x in range(0, W, 14):
            dr.rectangle([x, 0, x + 4, H], fill=wood)
        for y in (0, H // 3, 2 * H // 3, H - 10):
            dr.rectangle([0, y, W, y + 10], fill=wood)
        dr.rectangle([0, 0, 10, H], fill=wood)
        dr.rectangle([W - 10, 0, W, H], fill=wood)
        dr.rectangle([W // 2 - 6, 0, W // 2 + 6, H], fill=wood)
        return im
    a = np.zeros((H, W, 3))
    t = np.linspace(0, 1, H)[:, None]
    a[:] = (np.array((60, 70, 78.0)) * (1 - t[..., None]) + np.array((110, 124, 132.0)) * t[..., None])
    a[int(H * 0.45):int(H * 0.75)] += 25 * _noise(int(H * 0.75) - int(H * 0.45), W, rng, 4.0)[..., None]
    im = Image.fromarray(a.clip(0, 255).astype(np.uint8))
    dr = ImageDraw.Draw(im)
    fr = (190, 192, 194)
    dr.rectangle([0, 0, W, 10], fill=fr)
    dr.rectangle([0, H - 12, W, H], fill=fr)
    dr.rectangle([0, 0, 8, H], fill=fr)
    dr.rectangle([W - 8, 0, W, H], fill=fr)
    if kind == "auto":
        dr.rectangle([W // 2 - 5, 0, W // 2 + 5, H], fill=fr)
        y = int(H * 0.52)
        for x0, x1 in ((14, W // 2 - 10), (W // 2 + 10, W - 14)):
            dr.rectangle([x0, y, x1, y + 16], fill=(230, 40, 40))
            _glyphs(dr, x0 + 8, y + 2, 12, 4, (255, 255, 255), rng, vertical=False, width=2)
    else:
        dr.rectangle([W - 40, int(H * 0.35), W - 30, int(H * 0.65)], fill=(210, 210, 210))
    return im


def tex_noren(col, W=256, H=128):
    rng = _rng("noren" + col)
    ground = {"navy": (30, 40, 80), "red": (170, 30, 30), "white": (236, 232, 222), "brown": (90, 60, 40)}[col]
    ink = (250, 250, 248) if col != "white" else (30, 40, 80)
    im = Image.new("RGB", (W, H), ground)
    dr = ImageDraw.Draw(im)
    dr.ellipse([W / 2 - 34, 14, W / 2 + 34, 82], outline=ink, width=6)
    _glyphs(dr, W / 2 - 18, 28, 36, 1, ink, rng, width=6)
    _glyphs(dr, 18, 20, 30, 2, ink, rng, width=5)
    _glyphs(dr, W - 50, 20, 30, 2, ink, rng, width=5)
    return im


def _plain(col, rng_name, W=64, H=64, var=0.05):
    rng = _rng(rng_name)
    a = np.ones((H, W, 3)) * np.array(col, np.float64)
    a *= (1 + var * _noise(H, W, rng, 2.0))[..., None]
    return Image.fromarray(a.clip(0, 255).astype(np.uint8))


def _surface_maps(d, name, rough, metal):
    from textures import srgb_encode
    Image.fromarray((srgb_encode(np.full((8, 8), rough)) * 255).astype(np.uint8)).save(os.path.join(d, name + "_r.png"))
    Image.fromarray((srgb_encode(np.full((8, 8), metal)) * 255).astype(np.uint8)).save(os.path.join(d, name + "_m.png"))


def _write(d, imgs, rough):
    os.makedirs(d, exist_ok=True)
    out = {}
    for name, img in imgs.items():
        img.save(os.path.join(d, name + ".png"))
        _surface_maps(d, name, rough[name], METAL.get(name, 0.0))
        out[name] = [name, None, rough[name], METAL.get(name, 0.0), False]
    return out


def write_door_textures(d):
    """Door and noren textures (building mesh materials, build.py registers
    them). Returns {material: manifest spec}. / 扉・暖簾のテクスチャ。"""
    imgs = {"jp_door_auto": tex_door("auto"), "jp_door_glass1": tex_door("glass", 128, 280),
            "jp_door_koshi": tex_door("koshi"), "jp_noren_rod": _plain((60, 40, 26), "rod")}
    for c in ("navy", "red", "white", "brown"):
        imgs["jp_noren_" + c] = tex_noren(c)
    return _write(d, imgs, DOOR_MATS)


def write_prop_textures(d):
    """Prop textures. Returns {material: manifest spec}. / 小物のテクスチャ。"""
    imgs = {}
    for b in BRANDS:
        imgs["jihanki_front_" + b] = tex_jihanki_front(b)
        imgs["jihanki_body_" + b] = tex_jihanki_body(b)
        imgs["jihanki_bin_" + b] = tex_bin(b)
    imgs["kanban_a_chalk"] = tex_kanban_a("chalk")
    imgs["kanban_a_menu"] = tex_kanban_a("menu")
    imgs["kanban_frame"] = _plain((120, 84, 50), "kframe")
    imgs["kanban_stand"] = tex_kanban_stand()
    imgs["kanban_stand_dark"] = tex_kanban_stand(dark=True)
    imgs["kanban_stand_body"] = _plain((70, 70, 72), "kstand")
    for k in ("terracotta", "glaze_navy", "glaze_brown", "plastic"):
        imgs["hachi_" + k] = tex_pot(k)
    imgs["hachi_leaves"] = tex_leaves(False)
    imgs["hachi_flowers"] = tex_leaves(True)
    imgs["hachi_soil"] = _plain((70, 52, 38), "soil", var=0.15)
    for c in NOBORI:
        imgs["nobori_" + c] = tex_nobori(c)
    imgs["nobori_pole"] = _plain((214, 216, 218), "npole")
    imgs["nobori_base"] = _plain((40, 40, 42), "nbase")
    return _write(d, imgs, PROP_MATS)


# ============================================================ geometry / 形状
class Frame(object):
    """A prop's frame: origin o (back centre on the ground), t along the wall,
    n out from it, ground z0. local (x, y, z) -> world o + t x + n y, z0 + z.
    (t, n, z) is left-handed for a CCW footprint (n = t turned clockwise):
    every face is wound from an explicit outward vector, never from the
    local order. / 小物の座標系（外向きベクトルで面の向きを決める）。"""

    def __init__(self, o, t, n, z0):
        self.o = np.asarray(o, np.float64)
        self.t = np.asarray(t, np.float64) / np.linalg.norm(t)
        self.n = np.asarray(n, np.float64) / np.linalg.norm(n)
        self.z0 = float(z0)

    def p(self, x, y, z):
        q = self.o + self.t * x + self.n * y
        return np.array([q[0], q[1], self.z0 + z])

    def v(self, x, y, z):
        q = self.t * x + self.n * y
        return np.array([q[0], q[1], z])


def oquad(m, mat, A, B, C, D, out, uv=((0, 0), (1, 0), (1, 1), (0, 1))):
    """Quad A B C D wound so its normal points along ``out`` (UVs follow their
    vertices). / out 側を向く四角形。"""
    A, B, C, D = (np.asarray(p, np.float64) for p in (A, B, C, D))
    n = np.cross(B - A, D - A)
    if np.linalg.norm(n) < 1e-12:
        n = np.cross(C - B, D - B)
    if np.dot(n, out) < 0:
        A, B, C, D = D, C, B, A
        uv = uv[::-1]
    m.quad(mat, A, B, C, D, *uv)


def vface(m, F, mat, x0, x1, y, z0, z1, out_y, uv=(0, 0, 1, 1)):
    """Vertical face across the frame at local y, from x0 to x1, facing +n
    (out_y > 0) or -n; the texture reads unmirrored from that side.
    フレーム横断の鉛直面（見る側から正しい向き）。"""
    N = F.v(0, out_y, 0)
    right = np.cross(-N, (0, 0, 1))             # the viewer's right / 見る人の右
    xa, xb = (x0, x1) if np.dot(F.v(1, 0, 0), right) > 0 else (x1, x0)
    u0, v0, u1, v1 = uv
    oquad(m, mat, F.p(xa, y, z0), F.p(xb, y, z0), F.p(xb, y, z1), F.p(xa, y, z1), N,
          ((u0, v0), (u1, v0), (u1, v1), (u0, v1)))


def sface(m, F, mat, x, y0, y1, z0, z1, out_x, uv=(0, 0, 1, 1)):
    """Vertical face along the frame's n at local x, facing +t (out_x > 0) or
    -t. / 奥行き方向の鉛直面。"""
    N = F.v(out_x, 0, 0)
    right = np.cross(-N, (0, 0, 1))
    ya, yb = (y0, y1) if np.dot(F.v(0, 1, 0), right) > 0 else (y1, y0)
    u0, v0, u1, v1 = uv
    oquad(m, mat, F.p(x, ya, z0), F.p(x, yb, z0), F.p(x, yb, z1), F.p(x, ya, z1), N,
          ((u0, v0), (u1, v0), (u1, v1), (u0, v1)))


def hface(m, F, mat, x0, x1, y0, y1, z, up=True, uv=(0, 0, 1, 1)):
    u0, v0, u1, v1 = uv
    oquad(m, mat, F.p(x0, y0, z), F.p(x1, y0, z), F.p(x1, y1, z), F.p(x0, y1, z), (0, 0, 1 if up else -1),
          ((u0, v0), (u1, v0), (u1, v1), (u0, v1)))


def lbox(m, F, mats, x0, x1, y0, y1, z0, z1, uvs=None, bottom=False):
    """Box in the frame; mats / uvs: {front, back, left, right, top} (front =
    +n). Walls reach 2 cm under z0 (no gap on a slightly sloping ground).
    フレーム内の箱（壁は z0 の 2 cm 下まで）。"""
    uvs = uvs or {}
    get = lambda k: mats.get(k, mats.get("all"))
    zb = z0 - 0.02
    vface(m, F, get("front"), x0, x1, y1, zb, z1, 1, uvs.get("front", (0, 0, 1, 1)))
    vface(m, F, get("back"), x0, x1, y0, zb, z1, -1, uvs.get("back", (0, 0, 1, 1)))
    sface(m, F, get("right"), x1, y0, y1, zb, z1, 1, uvs.get("right", (0, 0, 1, 1)))
    sface(m, F, get("left"), x0, y0, y1, zb, z1, -1, uvs.get("left", (0, 0, 1, 1)))
    hface(m, F, get("top"), x0, x1, y0, y1, z1, True, uvs.get("top", (0, 0, 1, 1)))
    if bottom:
        hface(m, F, get("top"), x0, x1, y0, y1, z0, False)


def frustum(m, F, mat, cx, cy, z0, z1, r0, r1, seg=10, top_mat=None, top_z=None):
    """Tapered round pot (outward side faces, soil disc on top). / 鉢（外向き側面・土の面）。"""
    c = F.p(cx, cy, 0)
    for k in range(seg):
        a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
        d0, d1 = np.array([math.cos(a0), math.sin(a0), 0]), np.array([math.cos(a1), math.sin(a1), 0])
        A = c + d0 * r0 + (0, 0, z0 - 0.02)
        B = c + d1 * r0 + (0, 0, z0 - 0.02)
        C = c + d1 * r1 + (0, 0, z1)
        D = c + d0 * r1 + (0, 0, z1)
        oquad(m, mat, A, B, C, D, (d0 + d1) / 2, ((k / seg, 0), ((k + 1) / seg, 0), ((k + 1) / seg, 1), (k / seg, 1)))
    if top_mat:
        zt = z1 - 0.03 if top_z is None else top_z
        rr = r1 * 0.95
        P = [c + np.array([math.cos(2 * math.pi * k / seg) * rr, math.sin(2 * math.pi * k / seg) * rr, zt])
             for k in range(seg)]
        for k in range(1, seg - 1):
            A, B, C = P[0], P[k], P[k + 1]
            n = np.cross(B - A, C - A)
            if n[2] < 0:
                B, C = C, B
            m.tris(top_mat, [A, B, C], (0, 0, 1), [(0.5, 0.5), (1, 0.5), (0.5, 1)], [0, 1, 2])
        # rim inside (seen from above) / 縁の内側
        for k in range(seg):
            a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
            d0, d1 = np.array([math.cos(a0), math.sin(a0), 0]), np.array([math.cos(a1), math.sin(a1), 0])
            oquad(m, mat, c + d0 * r1 + (0, 0, z1), c + d1 * r1 + (0, 0, z1), c + d1 * rr + (0, 0, zt),
                  c + d0 * rr + (0, 0, zt), -(d0 + d1) / 2 + np.array([0, 0, 0.3]))


def blob(m, F, mat, cx, cy, cz, rx, ry, rz, rng, seg=7, rings=4, lump=0.18):
    """Lumpy ellipsoid of foliage, wound outward. / 葉の塊（外向き）。"""
    c = F.p(cx, cy, cz)
    J = [[1.0 + lump * (rng.random() - 0.5) * 2 for _ in range(seg)] for _ in range(rings + 1)]
    P = {}
    for j in range(rings + 1):
        th = math.pi * j / rings
        for k in range(seg):
            ph = 2 * math.pi * k / seg
            f = J[j][k] if 0 < j < rings else 1.0
            d = np.array([math.sin(th) * math.cos(ph) * rx, math.sin(th) * math.sin(ph) * ry, math.cos(th) * rz]) * f
            P[(j, k)] = c + d
    for j in range(rings):
        for k in range(seg):
            A, B, C, D = P[(j, k)], P[(j, k + 1 if k + 1 < seg else 0)], P[(j + 1, k + 1 if k + 1 < seg else 0)], P[(j + 1, k)]
            mid = (A + B + C + D) / 4
            uv = ((k / seg, j / rings), ((k + 1) / seg, j / rings), ((k + 1) / seg, (j + 1) / rings), (k / seg, (j + 1) / rings))
            if j == 0 or j == rings - 1:
                # pole rows: one corner is the pole twice -> a triangle / 極の列は三角形
                tri = [A, C, D] if j == 0 else [A, B, C]
                n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                if np.dot(n, mid - c) < 0:
                    tri = [tri[0], tri[2], tri[1]]
                    n = -n
                m.tris(mat, tri, n / (np.linalg.norm(n) + 1e-12), uv[:3], [0, 1, 2])
            else:
                oquad(m, mat, A, B, C, D, mid - c, uv)


# -- props / 小物 ----------------------------------------------------------------
def jihanki(m, F, x, brand):
    """A drinks machine, back to the wall at local x (centre). Front display
    face 3 cm proud of the cabinet (a frame round the lit window). / 自販機。"""
    W, D, H = JIHANKI
    body = "jihanki_body_" + brand
    y0 = GAP
    lbox(m, F, {"all": body, "front": "jihanki_front_" + brand}, x - W / 2, x + W / 2, y0, y0 + D, 0.0, H)
    # the lit window stands 3 cm proud, its reveal in the body colour / 見本窓を 3 cm 前へ
    z0, z1 = 1.02, 1.76
    ya, yb = y0 + D, y0 + D + 0.03
    vface(m, F, "jihanki_front_" + brand, x - W / 2 + 0.04, x + W / 2 - 0.04, yb, z0, z1, 1,
          (0.04, z0 / H, 0.96, z1 / H))
    sface(m, F, body, x - W / 2 + 0.04, ya, yb, z0, z1, -1)
    sface(m, F, body, x + W / 2 - 0.04, ya, yb, z0, z1, 1)
    hface(m, F, body, x - W / 2 + 0.04, x + W / 2 - 0.04, ya, yb, z1, True)
    hface(m, F, body, x - W / 2 + 0.04, x + W / 2 - 0.04, ya, yb, z0, False)


def kago(m, F, x, brand):
    """Recycling box beside a machine. / リサイクルボックス。"""
    W, D, H = BIN
    lbox(m, F, {"all": "jihanki_body_" + brand, "front": "jihanki_bin_" + brand},
         x - W / 2, x + W / 2, GAP, GAP + D, 0.0, H)


def kanban_a(m, F, x, y, kind):
    """A-frame menu board standing free (faces along +n and -n): two panels
    leaning together, wooden edges. / A 型看板。"""
    W, Hh, spread = 0.52, 0.92, 0.26
    mat = "kanban_a_" + kind
    for s in (1, -1):
        yb, yt = y + s * spread, y + s * 0.03
        A, B = F.p(x - W / 2, yb, 0.0), F.p(x + W / 2, yb, 0.0)
        C, D = F.p(x + W / 2, yt, Hh), F.p(x - W / 2, yt, Hh)
        out = F.v(0, s, 0) + np.array([0, 0, 0.4])
        right = np.cross(-F.v(0, s, 0), (0, 0, 1))
        uv = ((0, 0), (1, 0), (1, 1), (0, 1)) if np.dot(F.v(1, 0, 0), right) > 0 else ((1, 0), (0, 0), (0, 1), (1, 1))
        oquad(m, mat, A, B, C, D, out, uv)
        oquad(m, "kanban_frame", A + F.v(0, -s * 0.025, 0), B + F.v(0, -s * 0.025, 0),
              C + F.v(0, -s * 0.025, 0), D + F.v(0, -s * 0.025, 0), -out)
        for xe in (x - W / 2, x + W / 2):            # wooden edges / 木の縁
            sgn = 1 if xe > x else -1
            oquad(m, "kanban_frame", F.p(xe, yb, 0.0), F.p(xe, yb - s * 0.025, 0.0), F.p(xe, yt - s * 0.025, Hh),
                  F.p(xe, yt, Hh), F.v(sgn, 0, 0))
    hface(m, F, "kanban_frame", x - W / 2, x + W / 2, y - 0.035, y + 0.035, Hh + 0.01, True)


def kanban_stand(m, F, x, y, dark=False):
    """Stand board: a box 0.45 x 0.14 x 1.0 m on a 0.1 m base, faces +n / -n
    (lit white, or dark wood). スタンド看板（両面・台付き）。"""
    W, Dp, H0, H1 = 0.45, 0.14, 0.12, 1.15
    face = "kanban_stand_dark" if dark else "kanban_stand"
    lbox(m, F, {"all": "kanban_stand_body"}, x - 0.28, x + 0.28, y - 0.2, y + 0.2, 0.0, H0)
    lbox(m, F, {"all": "kanban_stand_body", "front": face, "back": face},
         x - W / 2, x + W / 2, y - Dp / 2, y + Dp / 2, H0, H1)


def hachi(m, F, x, y, rng, big=False):
    """A potted plant: a tapered pot (terracotta / glazed / plastic) with a
    shrub, a tall plant or flowers. / 鉢植え。"""
    kind = ["terracotta", "glaze_navy", "glaze_brown", "plastic"][int(rng.integers(0, 4))]
    r0 = (0.17 if big else 0.12) * rng.uniform(0.9, 1.15)
    r1 = r0 * 1.3
    h = (0.42 if big else 0.30) * rng.uniform(0.9, 1.15)
    frustum(m, F, "hachi_" + kind, x, y, 0.0, h, r0, r1, seg=10, top_mat="hachi_soil")
    plant = rng.random()
    if plant < 0.45:                                 # round shrub / 丸い低木
        blob(m, F, "hachi_leaves", x, y, h + r1 * 0.9, r1 * 1.25, r1 * 1.25, r1 * 1.0, rng)
    elif plant < 0.75:                               # tall plant / 背の高い観葉植物
        hh = min(rng.uniform(0.5, 0.9) * (1.3 if big else 1.0), (1.55 - h) / 1.25)   # top <= 1.55 m / 高さ 1.55 m 以下
        blob(m, F, "hachi_leaves", x, y, h + hh * 0.55, r1 * 1.1, r1 * 1.1, hh * 0.55, rng, lump=0.25)
    else:                                            # flowers / 花
        blob(m, F, "hachi_flowers", x, y, h + 0.08, r1 * 1.1, r1 * 1.1, 0.14, rng, rings=3)
    return r1 * 1.3, h + 1.0


def planter(m, F, x, y, rng, L=0.75):
    """A rectangular planter (プランター) along the wall with flowers or a low
    hedge. / プランター。"""
    D, H = 0.26, 0.28
    kind = ["terracotta", "plastic", "glaze_brown"][int(rng.integers(0, 3))]
    lbox(m, F, {"all": "hachi_" + kind, "top": "hachi_soil"}, x - L / 2, x + L / 2, y - D / 2, y + D / 2, 0.0, H)
    mat = "hachi_flowers" if rng.random() < 0.6 else "hachi_leaves"
    n = max(1, int(round(L / 0.35)))
    for k in range(n):
        xx = x - L / 2 + (k + 0.5) * L / n
        blob(m, F, mat, xx, y, H + 0.07, L / n * 0.6, D * 0.55, 0.16, rng, rings=3)


def nobori(m, F, x, y, col, side=1):
    """Banner on a white pole with a top bar, the cloth on the ``side`` (+t /
    -t) of the pole, on a black water-filled base. Cloth two-sided (faces +n
    and -n). / のぼり旗（白い竿・横棒・注水台）。"""
    m.cylinder("nobori_base", *F.p(x, y, 0)[:2], F.z0 - 0.02, F.z0 + 0.14, 0.2, seg=8, r1=0.15)
    m.cylinder("nobori_pole", *F.p(x, y, 0)[:2], F.z0 + 0.14, F.z0 + 2.75, 0.017, seg=6)
    xa, xb = (x + 0.03, x + 0.48) if side > 0 else (x - 0.48, x - 0.03)
    z0, z1 = 0.85, 2.65
    # the two sides 6 mm apart (coplanar back-to-back faces render black) / 両面は 6 mm 離す
    vface(m, F, "nobori_" + col, xa, xb, y + 0.003, z0, z1, 1)
    vface(m, F, "nobori_" + col, xa, xb, y - 0.003, z0, z1, -1)
    lbox(m, F, {"all": "nobori_pole"}, min(xa, xb) - 0.02, max(xa, xb), y - 0.012, y + 0.012, z1, z1 + 0.025)


# ============================================================ doors (buildings_jp) / 入口
USES = ("eatery", "cafe", "konbini", "retail")
EAT = {"restaurant", "fast_food", "bar", "pub", "izakaya", "food_court", "biergarten"}


def _poi_index(J):
    """OSM amenity / shop points of the map, once per JP (for tenant uses).
    地図の店舗・飲食店の点（用途に使う）。"""
    if getattr(J, "_sf_poi", None) is None:
        import shapely
        pts, uses = [], []
        L = getattr(J, "L", None)
        if L is not None and getattr(L, "osm", None) is not None:
            for p, t, _ in L.osm.points(lambda t: t.get("amenity") in EAT | {"cafe"} or "shop" in t):
                am, sh = t.get("amenity"), t.get("shop")
                use = ("eatery" if am in EAT else "cafe" if am == "cafe" else
                       "konbini" if sh == "convenience" else "retail")
                pts.append(p)
                uses.append(use)
        J._sf_poi = (shapely.STRtree(pts) if pts else None, pts, uses)
    return J._sf_poi


def _units(L, rng):
    """Tenant units along a shopfront of length L (one shop per 5-9 m).
    間口ごとのテナント区画。"""
    n = max(1, int(L // rng.uniform(5.5, 9.0)))
    if L < 9.0:
        n = 1
    w = L / n
    return [(k * w, (k + 1) * w) for k in range(n)]


def shop_front(J, m, f, it, idx, ei, u_lo, u_hi, zg, recess, wall, door_h=2.7):
    """Doors on one shopfront (buildings_jp b_shop / b_office podium): units,
    a pier between units, one entrance per unit, records in it.doors.
    Local z (the building is shifted by it.zb later). / 店舗正面の区画と入口。"""
    from buildings import seed_of
    rng = np.random.default_rng(seed_of("jpsf_door", idx, ei, round(float(it.poly.centroid.x), 1),
                                        round(float(it.poly.centroid.y), 1)))
    tree, pts, uses = _poi_index(J)
    poi = []
    if tree is not None:
        for k in tree.query(it.poly.buffer(1.0), predicate="intersects"):
            poi.append(uses[int(k)])
    doors = getattr(it, "doors", None)
    if doors is None:
        it.doors = doors = []
    units = _units(u_hi - u_lo, rng)
    for k, (a, b) in enumerate(units):
        a, b = a + u_lo, b + u_lo
        if k > 0:                                    # pier between tenants / 区画の間の柱
            f.block(wall, a - 0.15, a + 0.15, -recess, 0.0, zg, zg + door_h, top=False, bottom=False)
        if poi:
            use = poi[k % len(poi)]
        else:
            use = USES[int(rng.choice(4, p=[0.38, 0.12, 0.0, 0.50]))]
        kind = "koshi" if use == "eatery" and rng.random() < 0.55 else (
            "auto" if (b - a) >= 4.0 and (use == "konbini" or rng.random() < 0.6) else "glass1")
        dw = {"auto": 1.8, "koshi": 1.7, "glass1": 1.0}[kind]
        if b - a < dw + 1.0:
            continue
        pos = rng.random()
        if use == "konbini" or pos < 0.4:
            du = (a + b) / 2
        else:
            du = a + 0.6 + dw / 2 if pos < 0.7 else b - 0.6 - dw / 2
        door_unit(m, f, du, dw, zg, recess, kind, use, rng)
        c = f.P(du, 0.0, 0.0)
        doors.append({"i": -1 if idx is None else int(idx), "edge": int(ei), "x": round(float(c[0]), 3), "y": round(float(c[1]), 3),
                      "t": [round(float(f.t[0]), 5), round(float(f.t[1]), 5)],
                      "n": [round(float(f.n[0]), 5), round(float(f.n[1]), 5)],
                      "u": round(float(du), 3), "dw": dw, "ua": round(float(a), 3), "ub": round(float(b), 3),
                      "L": round(float(f.L), 3), "zg": round(float(zg), 3), "zb": round(float(getattr(it, "zb", 0.0)), 3),
                      "kind": kind, "use": use, "style": it.style})


def record_door(it, idx, ei, f, du, dw, zg, use):
    """A door buildings_jp draws itself (public halls, office entrances).
    buildings_jp 自身が描く入口の記録。"""
    doors = getattr(it, "doors", None)
    if doors is None:
        it.doors = doors = []
    c = f.P(du, 0.0, 0.0)
    doors.append({"i": -1 if idx is None else int(idx), "edge": int(ei), "x": round(float(c[0]), 3), "y": round(float(c[1]), 3),
                  "t": [round(float(f.t[0]), 5), round(float(f.t[1]), 5)],
                  "n": [round(float(f.n[0]), 5), round(float(f.n[1]), 5)],
                  "u": round(float(du), 3), "dw": round(float(dw), 3), "ua": 0.0, "ub": round(float(f.L), 3),
                  "L": round(float(f.L), 3), "zg": round(float(zg), 3), "zb": round(float(getattr(it, "zb", 0.0)), 3),
                  "kind": "own", "use": use, "style": it.style})


def _fquad(m, f, mat, u0, u1, z0, z1, w, out, uv=((0, 0), (1, 0), (1, 1), (0, 1))):
    """A facade-plane quad at offset w facing +n (out > 0) or -n; u grows to the
    viewer's right from outside, so the texture reads unmirrored from +n; from
    -n the UVs are mirrored back. / 外壁面の四角形。"""
    A, B, C, D = (np.array(f.P(u0, w, z0)), np.array(f.P(u1, w, z0)), np.array(f.P(u1, w, z1)),
                  np.array(f.P(u0, w, z1)))
    N = np.array([f.n[0], f.n[1], 0.0]) * (1 if out > 0 else -1)
    if out < 0:
        A, B, C, D = B, A, D, C
    oquad(m, mat, A, B, C, D, N, uv)


def door_unit(m, f, du, dw, zg, recess, kind, use, rng):
    """The entrance in the shopfront recess: leaf face 6 cm proud of the glass,
    aluminium (or wooden) jambs and head; a noren over a wooden door. Nothing
    reaches past the facade plane (w <= 0). / 入口（凹みの中のみ）。"""
    u0, u1 = du - dw / 2, du + dw / 2
    hd = {"auto": 2.3, "koshi": 2.0, "glass1": 2.2}[kind]
    wl = -recess + 0.06
    mat = {"auto": "jp_door_auto", "koshi": "jp_door_koshi", "glass1": "jp_door_glass1"}[kind]
    _fquad(m, f, mat, u0, u1, zg, zg + hd, wl, 1)
    fr = "jp_noren_rod" if kind == "koshi" else "jp_alu"
    f.block(fr, u0 - 0.06, u0, -recess, -recess + 0.12, zg, zg + hd + 0.06, top=True, bottom=False)
    f.block(fr, u1, u1 + 0.06, -recess, -recess + 0.12, zg, zg + hd + 0.06, top=True, bottom=False)
    f.block(fr, u0, u1, -recess, -recess + 0.12, zg + hd, zg + hd + 0.06, top=True, bottom=True, ends=False)
    if kind == "koshi" and recess >= 0.25:
        # noren on a rod just inside the recess, panels with slits / 暖簾（切れ目入り）
        col = ["navy", "navy", "red", "white", "brown"][int(rng.integers(0, 5))]
        wn, z1 = -0.08, zg + hd - 0.02
        z0 = z1 - (0.75 if rng.random() < 0.6 else 1.1)
        na, nb = u0 - 0.08, u1 + 0.08
        f.block("jp_noren_rod", na - 0.05, nb + 0.05, wn - 0.02, wn + 0.02, z1, z1 + 0.04, top=True, bottom=True)
        n = 3 if dw < 1.5 else 4
        pw = (nb - na) / n
        for k in range(n):
            a, b = na + k * pw + 0.012, na + (k + 1) * pw - 0.012
            uv = ((k / n, 0), ((k + 1) / n, 0), ((k + 1) / n, 1), (k / n, 1))
            _fquad(m, f, "jp_noren_" + col, a, b, z0, z1, wn, 1, uv)
            _fquad(m, f, "jp_noren_" + col, a, b, z0, z1, wn - 0.005, -1, uv)


# ============================================================ placement / 配置
# ground classes under a prop (built meshes; the model's ground in a core)
# 小物の下の地面の種類
NONE, WALK, TACTILE, LOT, ROAD, KERB, MARK, ROSOKU, DRIVE, GRASS, NOGO, PARK = range(12)
STAND = (WALK, LOT, PARK)                    # a prop may stand on these / 置ける地面
WALKABLE = (WALK, TACTILE, LOT, PARK)        # people walk here / 歩ける地面
STRICT = (TACTILE, ROAD, KERB, MARK, ROSOKU, DRIVE)    # + ROAD_CLEAR / 離隔を取る地面
POLE_KINDS = ("Terrain_Street", "Terrain_Trees", "Terrain_JpPlates", "Terrain_JpHokousha", "Core_Pole")
DOOR_MATS_ALL = ("jp_door", "jp_door_steel", "jp_shutter", "jp_glass_dark", "jp_door_auto", "jp_door_glass1",
                 "jp_door_koshi")
PARK_DEPTH = 1.1                             # on a car-park apron: only this close to the wall / 駐車場では壁際のみ
CORE_GROUND = {"M_tactile": TACTILE, "M_kerb": KERB, "M_grass": GRASS, "M_gravel": NOGO, "M_granite": NOGO,
               "M_lot": DRIVE, "M_plot": LOT, "M_subway_tile": NOGO}


def _okind(base):
    k = base.rsplit("_", 2)[0] if base.count("_") >= 2 else base
    return "_".join(k.split("_")[:2]) if k.startswith(("Road_", "Terrain_", "SM_")) else k


def ground_class(okind, mat):
    """Ground class of a built mesh's top face, or None (an obstacle). / 地面の種類。"""
    if okind == "Road_Road":
        return ROAD
    if okind == "Road_Curb":
        return KERB
    if okind == "Road_Marking":
        return MARK
    if okind == "Road_Sidewalk":
        return TACTILE if mat in ("pave_guide", "pave_warn") else ROSOKU if mat == "asphalt_rosoku" else WALK
    if okind == "Road_Grass":
        return GRASS
    if okind == "Terrain_Lots":
        return WALK if mat == "concrete_path" else PARK if mat == "asphalt_estate" else NOGO
    if okind == "Terrain_Courts":
        return NOGO
    if okind == "SM_Blocks" and mat == "jp_lot":
        return LOT
    return None


class Scene(object):
    """Triangles near the doors: ground (top faces, class) and obstacles (zmin,
    pole / door flags), from the built meshes and, in a core, the model's
    ground and furniture. / 入口付近の三角形（地面と障害物）。"""

    def __init__(self, bd, doors, D=None, core=None, log=print):
        self.bd = bd
        P = np.array([[d["x"], d["y"]] for d in doors]) if doors else np.zeros((0, 2))
        self.cell = 4.0
        if len(P):
            self.x0, self.y0 = P.min(0) - 16.0
            nx, ny = (((P.max(0) + 16.0) - (self.x0, self.y0)) / self.cell).astype(int) + 1
            near = np.zeros((ny, nx), bool)
            ci = ((P - (self.x0, self.y0)) / self.cell).astype(int)
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    near[np.clip(ci[:, 1] + dy, 0, ny - 1), np.clip(ci[:, 0] + dx, 0, nx - 1)] = True
            self.near = near
        else:
            self.near = np.zeros((1, 1), bool)
            self.x0 = self.y0 = 0.0
        self.S = np.zeros(np.array(self.near.shape) + 1)
        self.S[1:, 1:] = np.cumsum(np.cumsum(self.near, 0), 1)
        G, GC, O, OF = [], [], [], []
        import glob
        md = os.path.join(bd, "meshes")
        man = json.load(open(os.path.join(md, "manifest.json")))
        names = [o["name"] for o in man["objects"] if not o["name"].startswith(PREFIX)]
        for nm in names:
            ok = _okind(nm)
            d = np.load(os.path.join(md, nm + ".npz"))
            for key in d.files:
                if not key.endswith("|V"):
                    continue
                mat = key[:-2]
                V, I = d[key], d[mat + "|I"]
                if not len(I):
                    continue
                T = V[I].astype(np.float64)
                T = T[self._keep(T)]
                if not len(T):
                    continue
                gc = ground_class(ok, mat)
                if gc is not None:
                    nz = np.abs(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])[:, 2])
                    ar = np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
                    top = nz > 0.3 * np.maximum(ar, 1e-12)
                    if top.any():
                        G.append(T[top])
                        GC.append(np.full(top.sum(), gc, np.int8))
                    if ok == "SM_Blocks" and (~top).any():
                        O.append(T[~top])
                        OF.append(np.zeros((~top).sum(), np.int8))
                    continue
                if ok.endswith("Leaves") or "Leaves" in ok:
                    zg = D.z(T[:, :, 0].mean(1), T[:, :, 1].mean(1)) if D is not None else 0.0
                    T = T[T[:, :, 2].min(1) < zg + 3.5]
                    if not len(T):
                        continue
                flag = 1 if ok in POLE_KINDS else 0
                if ok == "SM_Blocks" and mat in DOOR_MATS_ALL:
                    flag = 2
                O.append(T)
                OF.append(np.full(len(T), flag, np.int8))
        if core is not None:
            for T, gc in core.ground():
                T = T[self._keep(T)]
                if len(T):
                    G.append(T)
                    GC.append(np.full(len(T), gc, np.int8))
            for T, flags in core.obstacles():
                k = self._keep(T)
                if k.any():
                    O.append(T[k])
                    OF.append(np.asarray(flags, np.int8)[k])
        self.G = np.concatenate(G) if G else np.zeros((0, 3, 3))
        self.GC = np.concatenate(GC) if GC else np.zeros(0, np.int8)
        self.O = np.concatenate(O) if O else np.zeros((0, 3, 3))
        self.OF = np.concatenate(OF) if OF else np.zeros(0, np.int8)
        self.Gb = np.concatenate([self.G[:, :, :2].min(1), self.G[:, :, :2].max(1)], 1)
        self.Ob = np.concatenate([self.O[:, :, :2].min(1), self.O[:, :, :2].max(1)], 1)
        log("[shopfront] scene: %d ground and %d obstacle triangles near %d doors" % (len(self.G), len(self.O), len(P)))

    def _keep(self, T):
        """Triangles whose bbox meets a cell near a door. / 入口付近の三角形。"""
        lo = np.floor((T[:, :, :2].min(1) - (self.x0, self.y0)) / self.cell).astype(int)
        hi = np.floor((T[:, :, :2].max(1) - (self.x0, self.y0)) / self.cell).astype(int)
        ny, nx = self.near.shape
        lo = np.clip(lo, 0, [nx - 1, ny - 1])
        hi = np.clip(hi, 0, [nx - 1, ny - 1])
        S = self.S
        s = S[hi[:, 1] + 1, hi[:, 0] + 1] - S[lo[:, 1], hi[:, 0] + 1] - S[hi[:, 1] + 1, lo[:, 0]] + S[lo[:, 1], lo[:, 0]]
        inside = ((T[:, :, :2].max(1) >= (self.x0, self.y0)).all(1)
                  & (T[:, :, :2].min(1) <= (self.x0 + nx * self.cell, self.y0 + ny * self.cell)).all(1))
        return (s > 0) & inside

    def subset(self, bb):
        x0, y0, x1, y1 = bb
        g = (self.Gb[:, 2] >= x0) & (self.Gb[:, 0] <= x1) & (self.Gb[:, 3] >= y0) & (self.Gb[:, 1] <= y1)
        o = (self.Ob[:, 2] >= x0) & (self.Ob[:, 0] <= x1) & (self.Ob[:, 3] >= y0) & (self.Ob[:, 1] <= y1)
        return np.nonzero(g)[0], np.nonzero(o)[0]


class Core(object):
    """The embedded model's ground classes (core_layers.npz, x-scaled, lifted)
    and its kept furniture / trees / station (core/core_obstacles.npz, made once
    with Blender, the v8 shop-front props and bicycles left out as in the
    JOIN). / 埋め込みモデルの地面と障害物。"""

    def __init__(self, bd, meta):
        from core_merge import core_level, core_rect
        self.bd, self.meta = bd, meta
        self.r = core_rect(meta)
        self.sx = float(meta["core"].get("x_scale", 1.0))
        self.z = core_level(bd)

    def ground(self):
        p = os.path.join(self.bd, "core_layers.npz")
        if not os.path.exists(p):
            return
        d = np.load(p)
        x0, y0, x1, y1 = self.r
        for layer in ("Roads", "Road_Markings", "Ground"):
            if layer not in d.files:
                continue
            T, M, names = d[layer].astype(np.float64), d[layer + "_mat"], d[layer + "_names"]
            T[:, :, 0] *= self.sx
            T[:, :, 2] += self.z
            c = T.mean(1)
            ins = (c[:, 0] > x0) & (c[:, 0] < x1) & (c[:, 1] > y0) & (c[:, 1] < y1)
            for mi in np.unique(M):
                nm = str(names[mi]) if mi < len(names) else ""
                if layer == "Roads":
                    gc = ROAD
                elif layer == "Road_Markings":
                    gc = TACTILE if nm == "M_tactile" else MARK
                else:
                    gc = CORE_GROUND.get(nm, WALK)
                sel = ins & (M == mi)
                if sel.any():
                    yield T[sel], gc

    def obstacles(self):
        p = os.path.join(self.bd, "core", "core_obstacles.npz")
        if not os.path.exists(p):
            p = make_core_obstacles(self.bd, self.meta)
        if p is None:
            return
        d = np.load(p)
        T = d["T"].astype(np.float64)
        T[:, :, 2] += self.z
        yield T, d["F"].astype(np.int8)


CORE_DUMP = r'''
import bpy, sys, numpy as np
from mathutils import Matrix
a = sys.argv[sys.argv.index("--") + 1:]
src, out, SX, here, bd = a[0], a[1], float(a[2]), a[3], a[4]
X0, Y0, X1, Y1 = (float(v) for v in a[5:9])
sys.path.insert(0, here)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=src)
S = Matrix.Diagonal((SX, 1.0, 1.0, 1.0))
LAYERS = {"Street_Furniture": 1, "Trees": 1, "Cars": 0, "JR_Oita_City": 0, "Railway": 0, "Trains": 0}
objs = [o for o in bpy.data.objects if o.type == "MESH" and o.name in LAYERS]
for o in objs:
    o.data = o.data.copy()
    o.data.transform(S @ o.matrix_world)
    o.matrix_parent_inverse = Matrix.Identity(4)
    o.matrix_basis = Matrix.Identity(4)
    o.parent = None
bpy.context.view_layer.update()
import core_props
core_props.drop_bicycles(objs, log=lambda *x: None)
core_props.drop_shopfront(objs, bd, log=lambda *x: None)
T, F = [], []
for o in objs:
    V = np.array([v.co[:] for v in o.data.vertices])
    tris = []
    for p in o.data.polygons:
        vs = list(p.vertices)
        for k in range(1, len(vs) - 1):
            tris.append((vs[0], vs[k], vs[k + 1]))
    if not tris:
        continue
    t = V[np.array(tris)]
    c = t.mean(1)
    t = t[(c[:, 0] > X0 - 5) & (c[:, 0] < X1 + 5) & (c[:, 1] > Y0 - 5) & (c[:, 1] < Y1 + 5) & (t[:, :, 2].min(1) < 3.5)]
    T.append(t.astype(np.float32)); F.append(np.full(len(t), LAYERS[o.name], np.int8))
np.savez_compressed(out, T=np.concatenate(T), F=np.concatenate(F))
print("[shopfront] core obstacles: %d triangles" % sum(len(t) for t in T))
'''


def make_core_obstacles(bd, meta):
    """The model's furniture, trees, buses, station and railway below 3.5 m
    (x-scaled, unlifted) -> core/core_obstacles.npz, cached on the model and
    core_props.py. / モデルの障害物を一度だけ書き出す。"""
    import hashlib
    import subprocess
    import tempfile
    core = meta.get("core") or {}
    src = os.path.expanduser(core.get("fbx", ""))
    if not os.path.exists(src):
        return None
    import core_props
    from core_merge import core_rect
    cd = os.path.join(bd, "core")
    os.makedirs(cd, exist_ok=True)
    core_props.ensure_walls(bd, src, float(core.get("x_scale", 1.0)))
    out = os.path.join(cd, "core_obstacles.npz")
    stamp = os.path.join(cd, "core_obstacles.json")
    want = {"src": src, "mtime": os.path.getmtime(src),
            "code": hashlib.sha1((CORE_DUMP + open(core_props.__file__).read()).encode()).hexdigest()}
    if os.path.exists(out) and os.path.exists(stamp) and json.load(open(stamp)) == want:
        return out
    with tempfile.TemporaryDirectory() as td:
        sp = os.path.join(td, "dump.py")
        open(sp, "w").write(CORE_DUMP)
        r = subprocess.run([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--", src, out,
                            str(core.get("x_scale", 1.0)), HERE, bd] + [str(v) for v in core_rect(meta)],
                           capture_output=True, text=True)
    if r.returncode or not os.path.exists(out):
        print("[shopfront] core obstacle dump failed:\n" + r.stdout[-2000:] + r.stderr[-2000:])
        return None
    json.dump(want, open(stamp, "w"))
    return out


class Window(object):
    """Local rasters (RES m) in a door's facade frame: s along the wall from
    the door centre, w out from the wall. / 入口ごとの局所ラスタ。"""

    def __init__(self, sc, door, zf, s_rng, w_rng=(-0.4, 6.4), extra=None):
        from scipy import ndimage
        self.c = np.array([door["x"], door["y"]])
        self.t = np.array(door["t"], np.float64)
        self.n = np.array(door["n"], np.float64)
        self.s0, self.s1 = s_rng
        self.w0, self.w1 = w_rng
        self.nu = int(round((self.s1 - self.s0) / RES))
        self.nw = int(round((self.w1 - self.w0) / RES))
        self.zf = zf
        corners = np.array([self.world(s, w) for s in s_rng for w in w_rng])
        bb = (*corners.min(0), *corners.max(0))
        gi, oi = sc.subset(bb)
        size = (self.nu, self.nw)
        # ground: triangle ids painted lowest first (the top surface wins) / 地面: 上の面が勝つ
        G = sc.G[gi]
        self.GT, self.GCl = G, sc.GC[gi]
        gid = Image.new("I", size, -1)
        dg = ImageDraw.Draw(gid)
        for k in np.argsort(G[:, :, 2].mean(1)):
            dg.polygon([tuple(q) for q in self.pix(G[k, :, :2])], fill=int(k))
        self.gid = np.asarray(gid, np.int64).copy()
        self.cls = np.where(self.gid >= 0, self.GCl[np.clip(self.gid, 0, None)] if len(G) else 0, NONE)
        # obstacles: the lowest point of anything over each pixel (cm over zf - 1 m)
        # 障害物: 各画素の上にある物の最下点
        O, OF = sc.O[oi], sc.OF[oi]
        keep = O[:, :, 2].max(1) > zf - 0.6
        O, OF = O[keep], OF[keep]
        occ = Image.new("I", size, 10 ** 6)
        do = ImageDraw.Draw(occ)
        pole = Image.new("L", size, 0)
        dp = ImageDraw.Draw(pole)
        dimg = Image.new("L", size, 0)
        dd = ImageDraw.Draw(dimg)
        zmin = O[:, :, 2].min(1)
        for k in np.argsort(-zmin):
            pts = [tuple(q) for q in self.pix(O[k, :, :2])]
            v = int(max(0, (zmin[k] - zf + 1.0) * 100))
            do.polygon(pts, fill=v, outline=v)
            if OF[k] == 1:
                dp.polygon(pts, fill=255, outline=255)
            elif OF[k] == 2 and zmin[k] < zf + 0.8:
                dd.polygon(pts, fill=255, outline=255)
        self.occ = np.asarray(occ, np.float64) / 100.0 + zf - 1.0
        for x, y, r, kind in (extra or ()):
            q = self.pix(np.array([[x, y]]))[0]
            rr = r / RES
            (dp if kind == "pole" else dd).ellipse([q[0] - rr, q[1] - rr, q[0] + rr, q[1] + rr], fill=255)
        pole = np.asarray(pole) > 0
        doors = np.asarray(dimg) > 0
        strict = np.isin(self.cls, STRICT) | getattr(self, "_road", False)
        self.strict0 = strict
        self.pole_d = ndimage.binary_dilation(pole, iterations=int(round(POLE_CLEAR / RES))) if pole.any() else pole
        self.door_d = ndimage.binary_dilation(doors, iterations=int(round(1.2 / RES))) if doors.any() else doors
        self.taken = np.zeros_like(pole)

    def world(self, s, w):
        return self.c + self.t * s + self.n * w

    def pix(self, P):
        d = P - self.c
        return np.column_stack([((d @ self.t) - self.s0) / RES, ((d @ self.n) - self.w0) / RES])

    def add_road(self, polys):
        """Forbidden areas from the layout (road logic: carriageway + kerb, drives,
        cycleways), drawn in, then every strict area dilated. / 道路情報の範囲。"""
        from scipy import ndimage
        img = Image.new("L", (self.nu, self.nw), 0)
        dr = ImageDraw.Draw(img)
        for p in polys:
            for ring in [p.exterior] + list(p.interiors):
                pts = [tuple(q) for q in self.pix(np.asarray(ring.coords)[:, :2])]
                if len(pts) >= 3:
                    dr.polygon(pts, fill=255 if ring is p.exterior else 0)
        road = np.asarray(img) > 0
        strict = self.strict0 | road
        self.strict_d = ndimage.binary_dilation(strict, iterations=int(round(ROAD_CLEAR / RES))) if strict.any() else strict

    def sl(self, s0, s1, w0, w1):
        i0 = max(0, int(math.floor((s0 - self.s0) / RES)))
        i1 = min(self.nu, int(math.ceil((s1 - self.s0) / RES)))
        j0 = max(0, int(math.floor((w0 - self.w0) / RES)))
        j1 = min(self.nw, int(math.ceil((w1 - self.w0) / RES)))
        return j0, j1, i0, i1

    def zat(self, s, w):
        """Exact ground z at (s, w) from the top triangle there, or None. / 地面の高さ。"""
        i = int((s - self.s0) / RES)
        j = int((w - self.w0) / RES)
        if not (0 <= i < self.nu and 0 <= j < self.nw):
            return None
        k = self.gid[j, i]
        if k < 0:
            return None
        A, B, C = self.GT[k]
        p = self.world(s, w)
        v0, v1, v2 = B[:2] - A[:2], C[:2] - A[:2], p - A[:2]
        den = v0[0] * v1[1] - v1[0] * v0[1]
        if abs(den) < 1e-12:
            return float(max(A[2], B[2], C[2]))
        b = (v2[0] * v1[1] - v1[0] * v2[1]) / den
        c = (v0[0] * v2[1] - v2[0] * v0[1]) / den
        return float(A[2] + b * (B[2] - A[2]) + c * (C[2] - A[2]))

    def test(self, s0, s1, w0, w1, H, why):
        """Can a prop stand on [s0, s1] x [w0, w1] (H tall)? Returns its base z
        and the clear footway in front (m, inf if it stands off the footway),
        or None (the reason counted in ``why``). / 置けるか（基準高さ・前の歩道幅）。"""
        j0, j1, i0, i1 = self.sl(s0, s1, w0, w1)
        if j1 <= j0 or i1 <= i0 or i0 == 0 and s0 < self.s0 or i1 == self.nu and s1 > self.s1:
            why["window"] = why.get("window", 0) + 1
            return None
        cl = self.cls[j0:j1, i0:i1]
        if not np.isin(cl, STAND).all():
            why["ground"] = why.get("ground", 0) + 1
            return None
        if (cl == PARK).any() and w1 > PARK_DEPTH:
            why["car park"] = why.get("car park", 0) + 1
            return None
        if self.strict_d[j0:j1, i0:i1].any():
            why["road/kerb/tactile"] = why.get("road/kerb/tactile", 0) + 1
            return None
        if self.pole_d[j0:j1, i0:i1].any():
            why["pole/tree"] = why.get("pole/tree", 0) + 1
            return None
        if self.door_d[j0:j1, i0:i1].any():
            why["door"] = why.get("door", 0) + 1
            return None
        if self.taken[j0:j1, i0:i1].any():
            why["taken"] = why.get("taken", 0) + 1
            return None
        zs = [self.zat(s, w) for s in np.linspace(s0 + 0.01, s1 - 0.01, 3) for w in np.linspace(w0 + 0.01, w1 - 0.01, 3)]
        if any(z is None for z in zs):
            why["ground"] = why.get("ground", 0) + 1
            return None
        zb = max(zs)
        if zb - min(zs) > 0.06 or abs(zb - self.zf) > 0.6:
            why["slope/step"] = why.get("slope/step", 0) + 1
            return None
        # anything built over the footprint below the prop's top (the facade at
        # w = 0 lies outside: props keep GAP off it) / 足元から高さまでに物が無いこと
        jj0, jj1, ii0, ii1 = self.sl(s0 - RES, s1 + RES, max(w0, GAP), w1 + RES)
        if (self.occ[jj0:jj1, ii0:ii1] < zb + H + 0.05).any():
            why["object"] = why.get("object", 0) + 1
            return None
        # the footway in front: WALK_CLEAR clear, or its width - WALK_NARROW / 歩道の有効幅
        clear = math.inf
        if np.isin(cl, (WALK, TACTILE)).any():
            j_fac = int(round((0.0 - self.w0) / RES))
            for i in range(i0, i1):
                col = self.cls[:, i]
                run = 0
                ok = True
                for j in range(j1, self.nw):
                    if col[j] not in WALKABLE or self.occ[j, i] < zb + 2.0 or self.pole_d[j, i] and col[j] != LOT:
                        ok = False
                        break
                    run += 1
                c = run * RES if not ok else math.inf
                width = 0
                for j in range(j_fac, self.nw):
                    if col[j] in (WALK, TACTILE):
                        width += 1
                    elif col[j] in (ROAD, KERB, MARK, ROSOKU, DRIVE):
                        break
                W = width * RES
                need = WALK_CLEAR if W >= WALK_CLEAR + WALK_NARROW else max(W - WALK_NARROW, 0.0)
                if c < need - 1e-6:
                    why["footway"] = why.get("footway", 0) + 1
                    return None
                clear = min(clear, c)
        return zb, clear

    def take(self, s0, s1, w0, w1, pad=0.08):
        j0, j1, i0, i1 = self.sl(s0 - pad, s1 + pad, w0 - pad, w1 + pad)
        self.taken[j0:j1, i0:i1] = True

    def mark(self, polys):
        """Props already placed (other doors) as taken. / 既に置いた小物。"""
        img = Image.new("L", (self.nu, self.nw), 0)
        dr = ImageDraw.Draw(img)
        for p in polys:
            dr.polygon([tuple(q) for q in self.pix(np.asarray(p.exterior.coords)[:, :2])], fill=255, outline=255)
        self.taken |= np.asarray(img) > 0


# -- what goes in front of a door / 入口ごとの小物 --------------------------------
def plan_door(door, rng):
    """The props a door gets (seeded; not every shop gets everything).
    入口ごとの小物（全部の店に全部は置かない）。"""
    use = door["use"]
    p = []
    if rng.random() < 0.12:
        return p                                         # a bare front / 何も無い店先
    if use == "konbini":
        if rng.random() < 0.85:
            p.append(("jihanki", 2 if rng.random() < 0.7 else 1))
        if rng.random() < 0.45:
            p.append(("nobori", int(rng.integers(1, 3))))
        if rng.random() < 0.2:
            p.append(("kanban_a", 1))
    elif use == "eatery":
        r = rng.random()
        if r < 0.45:
            p.append(("kanban_a", 1))
        elif r < 0.75:
            p.append(("kanban_stand", 1))
        if rng.random() < 0.55:
            p.append(("hachi", int(rng.integers(1, 3))))
        if rng.random() < 0.25:
            p.append(("nobori", 2 if rng.random() < 0.3 else 1))
        if rng.random() < 0.15:
            p.append(("jihanki", 1))
    elif use == "cafe":
        if rng.random() < 0.8:
            p.append(("kanban_a", 1))
        if rng.random() < 0.7:
            p.append(("hachi", 2))
        if rng.random() < 0.3:
            p.append(("planter", 1))
    elif use == "retail":
        if rng.random() < 0.45:
            p.append(("hachi" if rng.random() < 0.6 else "planter", int(rng.integers(1, 3))))
        if rng.random() < 0.25:
            p.append(("jihanki", 1 if rng.random() < 0.75 else 2))
        if rng.random() < 0.1:
            p.append(("nobori", 1))
        if rng.random() < 0.2:
            p.append(("kanban_a" if rng.random() < 0.6 else "kanban_stand", 1))
    elif use == "office":
        if rng.random() < 0.5:
            p.append(("planter_big", 2))
        if rng.random() < 0.1:
            p.append(("jihanki", 1))
    elif use == "public":
        if rng.random() < 0.4:
            p.append(("planter_big", 2))
    return p


def _sides(rng):
    return (1, -1) if rng.random() < 0.5 else (-1, 1)


def place_door(win, door, rng, why, s_lim):
    """Positions for one door's props in its window (facade frame). Returns
    [(kind, args, (s0, s1, w0, w1), zb, clear)]. / 入口の小物の位置。"""
    hw = door["dw"] / 2 + 0.35                         # the opening + 0.35 m / 開口 + 0.35 m
    lo, hi = s_lim
    out = []

    def fits(s0, s1, w0, w1, H):
        if s0 < lo or s1 > hi:
            why["facade end"] = why.get("facade end", 0) + 1
            return None
        if w0 < DOOR_OUT and s1 > -hw and s0 < hw:
            why["door"] = why.get("door", 0) + 1
            return None
        return win.test(s0, s1, w0, w1, H, why)

    def put(kind, args, box, r):
        out.append((kind, args, box, r[0], r[1]))
        win.take(*box)

    # pots and planters hug the door, boards stand beside it, machines and
    # banners go farther along / 鉢は入口の脇、看板はその隣、自販機・のぼりは先
    order = ("hachi", "planter", "planter_big", "kanban_a", "kanban_stand", "jihanki", "nobori")
    for kind, n in sorted(plan_door(door, rng), key=lambda kn: order.index(kn[0])):
        if kind == "jihanki":
            brand = BRANDS[int(rng.integers(0, len(BRANDS)))]
            W = JIHANKI[0] * n + BIN[0] + 0.04
            D = GAP + JIHANKI[1] + 0.03
            done = False
            for dist in np.arange(0.3, 8.0, 0.25):
                for sg in _sides(rng):
                    # the bin on the side away from the door / ゴミ箱は入口から遠い側
                    s0 = hw + dist if sg > 0 else -hw - dist - W
                    r = fits(s0, s0 + W, GAP, D, JIHANKI[2])
                    if r:
                        xs = [s0 + JIHANKI[0] * (k + 0.5) for k in range(n)] if sg > 0 else \
                             [s0 + BIN[0] + 0.04 + JIHANKI[0] * (k + 0.5) for k in range(n)]
                        xb = s0 + W - BIN[0] / 2 if sg > 0 else s0 + BIN[0] / 2
                        put("jihanki", (xs, xb, brand), (s0, s0 + W, GAP, D), r)
                        done = True
                        break
                if done:
                    break
        elif kind in ("kanban_a", "kanban_stand"):
            d = 0.58 if kind == "kanban_a" else 0.46
            dep = 0.58 if kind == "kanban_a" else 0.42
            H = 0.97 if kind == "kanban_a" else 1.17
            sty = "chalk" if rng.random() < 0.55 else "menu"
            done = False
            for wc in (0.55, 0.75, 0.4, 0.95):
                for dist in (0.35, 0.6, 0.9, 1.3):
                    for sg in _sides(rng):
                        sc = sg * (hw + dist + d / 2)
                        r = fits(sc - d / 2, sc + d / 2, wc - dep / 2, wc + dep / 2, H)
                        if r:
                            put(kind, (sc, wc, sty), (sc - d / 2, sc + d / 2, wc - dep / 2, wc + dep / 2), r)
                            done = True
                            break
                    if done:
                        break
                if done:
                    break
        elif kind in ("hachi",):
            big = rng.random() < 0.35
            sides = (1, -1) if n == 2 else _sides(rng)[:1]
            for sg in sides:
                rr = 0.3 if big else 0.22
                for dist in (0.05, 0.3, 0.6):
                    sc = sg * (hw + dist + rr)
                    r = fits(sc - rr, sc + rr, GAP, GAP + 2 * rr, 1.6)
                    if r:
                        put("hachi", (sc, GAP + rr, big, int(rng.integers(0, 1 << 30))),
                            (sc - rr, sc + rr, GAP, GAP + 2 * rr), r)
                        break
        elif kind in ("planter", "planter_big"):
            Lp = 1.1 if kind == "planter_big" else float(rng.choice([0.6, 0.75, 0.9]))
            sides = (1, -1) if n == 2 else _sides(rng)[:1]
            for sg in sides:
                for dist in (0.1, 0.5, 1.0):
                    sc = sg * (hw + dist + Lp / 2)
                    r = fits(sc - Lp / 2, sc + Lp / 2, GAP, GAP + 0.3, 0.75)
                    if r:
                        put("planter", (sc, GAP + 0.13, Lp, int(rng.integers(0, 1 << 30))),
                            (sc - Lp / 2, sc + Lp / 2, GAP, GAP + 0.3), r)
                        break
        elif kind == "nobori":
            col = NOBORI[int(rng.integers(0, len(NOBORI)))]
            placed = 0
            for dist in np.arange(0.5, 6.0, 0.4):
                for sg in _sides(rng):
                    for wc in (0.6, 0.9, 1.3):
                        sp = sg * (hw + dist)
                        side = 1 if sg > 0 else -1
                        s0, s1 = (sp - 0.21, sp + 0.52) if side > 0 else (sp - 0.52, sp + 0.21)
                        r = fits(s0, s1, wc - 0.21, wc + 0.21, 2.85)
                        if r:
                            put("nobori", (sp, wc, col, side), (s0, s1, wc - 0.21, wc + 0.21), r)
                            placed += 1
                            break
                    if placed >= n:
                        break
                if placed >= n:
                    break
    return out


def draw(m, F, kind, args, rng):
    """One placed prop into mesh m (frame F at the door, base z set). / 小物を描く。"""
    if kind == "jihanki":
        xs, xb, brand = args
        for x in xs:
            jihanki(m, F, x, brand)
        kago(m, F, xb, brand)
    elif kind == "kanban_a":
        sc, wc, sty = args
        kanban_a(m, F, sc, wc, sty)
    elif kind == "kanban_stand":
        sc, wc, sty = args
        kanban_stand(m, F, sc, wc, dark=sty == "chalk")
    elif kind == "hachi":
        sc, wc, big, seed = args
        hachi(m, F, sc, wc, np.random.default_rng(seed), big)
    elif kind == "planter":
        sc, wc, Lp, seed = args
        planter(m, F, sc, wc, np.random.default_rng(seed), Lp)
    elif kind == "nobori":
        sp, wc, col, side = args
        nobori(m, F, sp, wc, col, side)


def _map_osm(name):
    for v in json.load(open(os.path.join(HERE, "..", "maps.json"))).values():
        if isinstance(v, dict) and v.get("name") == name:
            return v
    return {}


def signal_spots(bd, name):
    """(x, y) of every signal CARLA spawns from the xodr (lights, stop and speed
    posts): their poles are not in the meshes. / CARLA が置く信号・標識の位置。"""
    import xml.etree.ElementTree as ET
    from jp_signs import signal_xy
    out = []
    xodr = os.path.join(bd, name + ".xodr")
    if not os.path.exists(xodr):
        return out
    for road in ET.parse(xodr).getroot().findall("road"):
        for sg in road.iter("signal"):
            pi = sg.find("positionInertial")
            try:
                if pi is not None:
                    out.append((float(pi.get("x")), float(pi.get("y"))))
                else:
                    out.append(signal_xy(road, float(sg.get("s")), float(sg.get("t"))))
            except (TypeError, ValueError):
                continue
    return out


def build(bd, log=print):
    """Place and write the props of a built map (after core_merge.py).
    組み立て済みの地図に店先の小物を置いて書き出す。"""
    import shapely
    from shapely.geometry import Polygon
    import layout
    import terrain
    from buildings import seed_of
    meta = json.load(open(os.path.join(bd, "meta.json")))
    name = meta["name"]
    ent = _map_osm(name)
    md = os.path.join(bd, "meshes")
    man_p = os.path.join(md, "manifest.json")
    man = json.load(open(man_p))
    old = [o for o in man["objects"] if o["name"].startswith(PREFIX)]
    for o in old:
        man["objects"].remove(o)
        try:
            os.remove(os.path.join(md, o["name"] + ".npz"))
        except OSError:
            pass
    sp = os.path.join(bd, "shopfronts.json")
    if ent.get("road_style") != "japan" or not os.path.exists(sp) or ent.get("shopfront") is False:
        json.dump(man, open(man_p, "w"), indent=0)
        log("[shopfront] not a Japan map or no shopfronts.json: nothing placed")
        return {}
    doors = json.load(open(sp))["doors"]
    osm = os.path.expanduser(ent["osm"])
    L = layout.Layout(meta, os.path.join(bd, "lanes.json"), osm)
    D = terrain.drape_for(bd, L)
    core = Core(bd, meta) if meta.get("core") else None
    sc = Scene(bd, doors, D, core, log)
    from layout import polys as _polys
    forb = [g for g in _polys(L.road_all)] + [g for nm in ("driveways", "cycleways")
                                              for g in _polys(getattr(L, nm, None))]
    ftree = shapely.STRtree(forb)
    extra = [(x, y, POLE_CLEAR / 2 + 0.15, "pole") for x, y in signal_spots(bd, name)]
    extra += [(p.x, p.y, BUS_CLEAR, "door") for p, t, _ in
              L.osm.points(lambda t: t.get("highway") == "bus_stop" or t.get("public_transport") == "platform")]
    xtree = shapely.STRtree([shapely.Point(e[0], e[1]) for e in extra]) if extra else None
    from core_merge import core_rect, in_rect
    rect = core_rect(meta) if meta.get("core") else None
    import ground
    tiles = list(ground.tiles(meta))
    meshes = {}
    placed_polys = []
    ptree_dirty = True
    why = {}
    recs = []
    stats = {}
    for d in sorted(doors, key=lambda d: (d["i"], d["edge"], d["u"])):
        zf = d["zb"] + d["zg"]
        lo, hi = -d["u"] + 0.1, d["L"] - d["u"] - 0.1        # the building's own facade / 自分の外壁内
        s_rng = (max(lo, -7.0) - 0.5, min(hi, 7.0) + 0.5)
        if s_rng[1] - s_rng[0] < 1.5:
            continue
        c = np.array([d["x"], d["y"]])
        near_x = [extra[int(k)] for k in xtree.query(shapely.Point(*c).buffer(12.0))] if xtree is not None else []
        win = Window(sc, d, zf, s_rng, extra=near_x)
        box = shapely.Point(*c).buffer(10.0)
        win.add_road([forb[int(k)] for k in ftree.query(box, predicate="intersects")])
        win.mark([p for p in placed_polys if p.distance(shapely.Point(*c)) < 12.0])
        rng = np.random.default_rng(seed_of("jpsf_props", d["i"], d["edge"], round(d["u"], 1),
                                            round(d["x"], 1), round(d["y"], 1)))
        items = place_door(win, d, rng, why, (lo, hi))
        for kind, args, (s0, s1, w0, w1), zb, clear in items:
            F = Frame(c, win.t, win.n, zb)
            pc = c + win.t * (s0 + s1) / 2 + win.n * (w0 + w1) / 2
            tn = next((tn for tn, tb in tiles if tb.contains(shapely.Point(*pc))), "00")
            m = meshes.setdefault(tn, Mesh())
            draw(m, F, kind, args, rng)
            poly = Polygon([tuple(win.world(s, w)) for s, w in ((s0, w0), (s1, w0), (s1, w1), (s0, w1))])
            placed_polys.append(poly)
            in_core = bool(rect is not None and in_rect(pc[0], pc[1], rect))
            n_units = len(args[0]) if kind == "jihanki" else 1
            key = kind + ("_pair" if kind == "jihanki" and n_units == 2 else "")
            st = stats.setdefault(key, [0, 0])
            st[1 if in_core else 0] += 1
            recs.append({"kind": key, "x": round(float(pc[0]), 2), "y": round(float(pc[1]), 2), "z": round(zb, 3),
                         "face_deg": round(math.degrees(math.atan2(win.n[1], win.n[0])), 1), "use": d["use"],
                         "door": [d["i"], d["edge"], d["u"]], "core": in_core,
                         "clear": None if math.isinf(clear) else round(clear, 2),
                         "box": [round(v, 3) for v in (s0, s1, w0, w1)]})
    for tn, m in sorted(meshes.items()):
        arrs = m.arrays()
        if not arrs:
            continue
        on = PREFIX + "%s_%s" % (name, tn)
        data = {}
        for mat, (V, N, UV, I) in arrs.items():
            data[mat + "|V"], data[mat + "|N"], data[mat + "|UV"], data[mat + "|I"] = V, N, UV, I
        np.savez_compressed(os.path.join(md, on + ".npz"), **data)
        man["objects"].append({"name": on, "kind": KIND, "materials": sorted(arrs),
                               "tris": int(sum(len(a[3]) for a in arrs.values()))})
    man["materials"].update(write_prop_textures(os.path.join(bd, "textures")))
    bad = [o["name"] for o in man["objects"] if o["name"].startswith(PREFIX)
           for n in [o["name"]] + o["materials"] if "sign" in n.lower() or "light" in n.lower()]
    assert not bad, bad
    json.dump(man, open(man_p, "w"), indent=0)
    clears = [r["clear"] for r in recs if r["clear"] is not None]
    tris = sum(o["tris"] for o in man["objects"] if o["name"].startswith(PREFIX))
    summary = {"doors": len(doors), "doors_by_use": {}, "props": stats, "rejections": why, "triangles": tris,
               "on_footway": len(clears), "footway_clear_min": min(clears) if clears else None,
               "footway_clear_median": float(np.median(clears)) if clears else None}
    for d in doors:
        summary["doors_by_use"][d["use"]] = summary["doors_by_use"].get(d["use"], 0) + 1
    json.dump({"summary": summary, "props": recs}, open(os.path.join(bd, "shopfront_props.json"), "w"), indent=0)
    log("[shopfront] %d doors (%s): %s; %d triangles" % (
        len(doors), ", ".join("%s %d" % kv for kv in sorted(summary["doors_by_use"].items())),
        ", ".join("%s %d (core %d)" % (k, v[0] + v[1], v[1]) for k, v in sorted(stats.items())), tris))
    log("[shopfront] footway in front of the %d props standing on it: min %s m, median %s m clear"
        % (len(clears), summary["footway_clear_min"], summary["footway_clear_median"]))
    log("[shopfront] tries rejected: %s" % ", ".join("%s %d" % kv for kv in sorted(why.items(), key=lambda kv: -kv[1])))
    return summary


def sample(out_dir):
    """Every prop in a row (and a door unit on a dummy front) for previews and
    tests: meshes/ + textures/ + manifest. / 全小物の見本。"""
    from buildings_jp import Facade
    md = os.path.join(out_dir, "meshes")
    td = os.path.join(out_dir, "textures")
    os.makedirs(md, exist_ok=True)
    m = Mesh()
    F = Frame((0.0, 0.0), (1.0, 0.0), (0.0, -1.0), 0.0)
    rng = np.random.default_rng(7)
    jihanki(m, F, -4.0, "red")
    jihanki(m, F, -3.0, "blue")
    kago(m, F, -2.25, "blue")
    kanban_a(m, F, -1.0, 0.6, "chalk")
    kanban_stand(m, F, 0.0, 0.5)
    hachi(m, F, 0.9, 0.3, rng)
    hachi(m, F, 1.6, 0.35, rng, True)
    planter(m, F, 2.7, 0.2, rng)
    nobori(m, F, 4.0, 0.7, "red", 1)
    wall = Mesh()
    f = Facade(wall, np.array([-6.0, 0.0]), np.array([7.0, 0.0]), np.array([0.0, -1.0]))
    door_unit(wall, f, -5.0, 1.8, 0.0, 0.3, "auto", "retail", rng)
    door_unit(wall, f, 5.5, 1.7, 0.0, 0.3, "koshi", "eatery", rng)
    man = {"name": "Sample", "objects": [], "materials": {}}
    for on, mm in ((PREFIX + "Sample_00", m), ("SM_ShopDoors_Sample_00", wall)):
        arrs = mm.arrays()
        data = {}
        for mat, (V, N, UV, I) in arrs.items():
            data[mat + "|V"], data[mat + "|N"], data[mat + "|UV"], data[mat + "|I"] = V, N, UV, I
        np.savez_compressed(os.path.join(md, on + ".npz"), **data)
        man["objects"].append({"name": on, "kind": KIND, "materials": sorted(arrs),
                               "tris": int(sum(len(a[3]) for a in arrs.values()))})
    man["materials"].update(write_prop_textures(td))
    man["materials"].update(write_door_textures(td))
    man["materials"]["jp_alu"] = [None, [190, 192, 194], 0.4, 0.6, False]
    json.dump(man, open(os.path.join(md, "manifest.json"), "w"), indent=0)
    return m, wall


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--sample":
        a, b = sample(sys.argv[2])
        print("[shopfront] sample: props %d tris, doors %d tris" % (a.ntris(), b.ntris()))
    else:
        build(os.path.abspath(sys.argv[1]))
