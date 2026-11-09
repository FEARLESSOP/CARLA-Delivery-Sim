"""Japanese road surfaces and road paint (maps/.venv), for maps.json
"road_style": "japan". Procedural only, nothing taken from photos.

Every texture comes with its own height field, from which the normal map
(<tex>_n.png, DirectX green) and the roughness map (<mat>_r.png, stored
sRGB-encoded: chapter 2) are made, so stones, cracks and joints have relief
and their own roughness. Tiles (metres per texture repeat, the uv_scale
ground.py uses): asphalt 5 m, pavements 3 m, kerb top 1 m, kerb face 1.2 m
(two 0.6 m blocks), paint 1 m.

  asphalt_jp        dense-graded asphalt of the main roads: fine aggregate
                    (albedo ~0.05-0.08, as dark as Street View's asphalt)
  asphalt_wheel     the wheel paths: darker, polished (smoother)
  asphalt_estate    old estate streets: oxidised (lighter, greyer), ravelled
  asphalt_crack     their edges: alligator and edge cracking
  asphalt_patch     repairs and utility trenches: new, black, fine
  pavement_asphalt  footways (Akeno pavements are asphalt, Street View)
  kerb_jp           concrete kerb top; kerb_face_jp its face, 0.6 m blocks,
                    grime at road level
  line_white / line_yellow  road paint with wear and glass-bead grain
                    (Japanese yellow is orange-yellow)
  pave_block        インターロッキング: light grey 0.1 x 0.2 m concrete blocks in
                    stretcher bond, 3 mm sand joints, chamfered edges, per-block
                    tone (2 m tile) -- the road pavements (pavements.py, ch. 31)
  pave_guide        線状ブロック: JIS T 9251 guide block, 4 bars along u (0.3 m
                    tile, UVs along the strip); pave_warn 点状ブロック: 5 x 5 dots
  asphalt_rosoku    the 路側帯 strip of the estate streets: the cracked-edge
                    asphalt texture, its own material so the strip can be its own
                    (Sidewalk) object

日本式の路面・縁石・路面標示のテクスチャ（手続き生成、高さ場から法線と粗さ）。
"""

import os

import numpy as np
from PIL import Image, ImageDraw

from textures import normal_map, srgb_encode

# name: (texture, colour, roughness, metallic, smooth) as textures.MATERIALS / 材質表
MATERIALS = {
    "asphalt_jp": ("asphalt_jp", None, 0.86, 0.0, False),
    "asphalt_wheel": ("asphalt_wheel", None, 0.72, 0.0, False),
    "asphalt_estate": ("asphalt_estate", None, 0.9, 0.0, False),
    "asphalt_crack": ("asphalt_crack", None, 0.9, 0.0, False),
    "asphalt_patch": ("asphalt_patch", None, 0.8, 0.0, False),
    "pavement_asphalt": ("pavement_asphalt", None, 0.88, 0.0, False),
    "kerb_jp": ("kerb_jp", None, 0.85, 0.0, False),
    "kerb_face_jp": ("kerb_face_jp", None, 0.85, 0.0, False),
    "line_white": ("line_white", None, 0.55, 0.0, False),
    "line_yellow": ("line_yellow", None, 0.58, 0.0, False),
    "pave_block": ("pave_block", None, 0.8, 0.0, False),
    "pave_guide": ("pave_guide", None, 0.62, 0.0, False),
    "pave_warn": ("pave_warn", None, 0.62, 0.0, False),
    "asphalt_rosoku": ("asphalt_crack", None, 0.9, 0.0, False),
}
# the general buildings' materials (buildings_jp, General Builder) / 一般建物の材質
from buildings_jp import MATERIALS as _BUILDING_MATERIALS  # noqa: E402
from textures_jp_bld import MATERIALS as _PAINTED  # noqa: E402
MATERIALS.update(_BUILDING_MATERIALS)
MATERIALS.update(_PAINTED)          # textured, with their pattern variants / 柄違いを含む
# generic school / worship blocks use corridor_dark (a plain colour, which never
# reaches Unreal): Japanese maps give it the dark recess texture
# 汎用の学校・宗教施設の corridor_dark に暗部テクスチャを使う
MATERIALS["corridor_dark"] = ("jp_dark", None, 0.9, 0.0, False)
# metres per texture repeat (ground.py's uv_scale) / テクスチャ 1 枚の大きさ
TILE = {"asphalt": 5.0, "pavement": 3.0, "kerb": 1.0, "kerb_face": 1.2, "paint": 1.0, "pave_block": 2.0,
        "tactile": 0.3}


def smooth(n, sigma, rng):
    """Tileable smooth noise in [-1, 1]: white noise blurred by a periodic
    Gaussian of sigma pixels (FFT). / タイル可能な滑らかなノイズ。"""
    w = rng.standard_normal((n, n))
    k = np.fft.fftfreq(n)
    g = np.exp(-2 * (np.pi * sigma) ** 2 * (k[:, None] ** 2 + k[None, :] ** 2))
    a = np.real(np.fft.ifft2(np.fft.fft2(w) * g))
    a -= a.mean()
    return a / max(np.abs(a).max(), 1e-9)


def stones(n, size, density, rng):
    """Aggregate: (mask 0..1 of stone tops, per-stone brightness). / 骨材。"""
    s = smooth(n, size, rng)
    thr = np.quantile(s, 1.0 - density)
    m = np.clip((s - thr) / max(1e-6, (s.max() - thr) * 0.35), 0.0, 1.0)
    bright = 0.75 + 0.5 * (0.5 + 0.5 * smooth(n, size * 1.6, rng))
    return m, bright


def asphalt(n, rng, binder, stone, density=0.32, size=0.9, tone=0.10, pits=0.006, polish=0.0,
            rough=(0.9, 0.06)):
    """Asphalt: binder, aggregate tops, voids, broad tone variation.
    Returns (rgb, height, roughness). / アスファルト（バインダ・骨材・空隙・色むら）。"""
    low = 0.6 * smooth(n, n / 10, rng) + 0.4 * smooth(n, n / 40, rng)
    m, br = stones(n, size, density, rng)
    m *= (1.0 - polish)
    u = rng.random((n, n))
    pit = (u < pits).astype(np.float64)
    pit = np.clip(pit + 0.5 * (smooth(n, 0.7, rng) > 0.93), 0, 1)
    b = np.array(binder, np.float64)
    st = np.array(stone, np.float64)
    col = b[None, None] * (1 - m[..., None]) + st[None, None] * br[..., None] * m[..., None]
    col *= (1.0 + tone * low)[..., None]
    col *= (1.0 - 0.45 * pit)[..., None]
    col += 6.0 * smooth(n, 0.6, rng)[..., None]           # grain / 粒
    h = 0.9 * m - 1.4 * pit + 0.15 * smooth(n, 0.8, rng)
    r = rough[0] + rough[1] * (0.6 * low - 0.5 * m + 0.8 * pit)
    return col, h, r


def cracks(n, rng, cells=14, width=1.6, cover=0.55):
    """Alligator and edge cracking: borders of wobbly Voronoi cells (wrapped,
    so the tile repeats), kept where a smooth mask says so. Returns 0..1.
    亀甲状・縁部のひび割れ（タイル可能なボロノイ境界）。"""
    pts = rng.random((cells * cells // 2, 2)) * n
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float64)
    wob = 9.0
    xx += wob * smooth(n, n / 24, rng)
    yy += wob * smooth(n, n / 24, rng)
    d1 = np.full((n, n), 1e9)
    d2 = np.full((n, n), 1e9)
    for px, py in pts:
        dx = np.abs(xx - px)
        dy = np.abs(yy - py)
        dx = np.minimum(dx, n - dx)
        dy = np.minimum(dy, n - dy)
        d = np.sqrt(dx * dx + dy * dy)
        nd2 = np.where(d < d1, d1, np.minimum(d2, d))
        d1 = np.minimum(d1, d)
        d2 = nd2
    edge = np.clip(1.0 - (d2 - d1) / width, 0.0, 1.0)
    mask = smooth(n, n / 12, rng) > (1.0 - 2.0 * cover)
    # a few long longitudinal cracks too / 縦方向の長いひび
    im = Image.new("L", (n, n), 0)
    dr = ImageDraw.Draw(im)
    for k in range(3):
        x = rng.uniform(0, n)
        pts_l = []
        for y in np.linspace(0, n, 40):
            x += rng.normal(0, 3.0)
            pts_l.append((x % n, y))
        for (x0, y0), (x1, y1) in zip(pts_l, pts_l[1:]):
            if abs(x1 - x0) < n / 2:
                for ox in (-n, 0, n):
                    dr.line([(x0 + ox, y0), (x1 + ox, y1)], fill=255, width=2)
    lon = np.asarray(im, np.float64) / 255.0
    return np.clip(edge * mask + lon, 0.0, 1.0)


def concrete(n, rng, c0, c1, pores=0.004):
    low = 0.6 * smooth(n, n / 8, rng) + 0.4 * smooth(n, n / 32, rng)
    f = 0.5 + 0.5 * low
    c0, c1 = np.array(c0, np.float64), np.array(c1, np.float64)
    col = c0[None, None] + (c1 - c0)[None, None] * f[..., None]
    pore = (rng.random((n, n)) < pores).astype(np.float64)
    col *= (1.0 - 0.35 * pore)[..., None]
    col += 5.0 * smooth(n, 0.7, rng)[..., None]
    # grime blotches / 汚れ
    g = np.clip(smooth(n, n / 16, rng) - 0.3, 0, 1)
    col *= (1.0 - 0.18 * g)[..., None]
    h = 0.3 * smooth(n, 1.2, rng) - 1.0 * pore + 0.2 * low
    r = 0.85 + 0.05 * low + 0.05 * pore
    return col, h, r


def paint(n, rng, c, wear=0.10):
    """Road paint: thick film with glass-bead grain, worn spots showing the
    asphalt. / 路面標示の塗料（ガラスビーズの粒、摩耗）。"""
    # fine chips (1-2 cm), denser where a broad mask says so / 細かい剥がれ
    worn_m = smooth(n, 2.5, rng) + 0.9 * smooth(n, n / 10, rng)
    worn = (worn_m > np.quantile(worn_m, 1.0 - wear)).astype(np.float64)
    beads = (rng.random((n, n)) < 0.03).astype(np.float64)
    c = np.array(c, np.float64)
    col = c[None, None] * (0.94 + 0.06 * smooth(n, 4.0, rng))[..., None]
    col = col * (1 - worn[..., None]) + np.array([70.0, 70.0, 72.0])[None, None] * worn[..., None]
    col *= (1.0 - 0.06 * smooth(n, n / 6, rng) ** 2)[..., None]      # dirt / 汚れ
    h = 0.6 * (1 - worn) + 0.3 * beads + 0.2 * smooth(n, 0.8, rng)
    r = 0.55 + 0.25 * worn - 0.15 * beads
    return col, h, r


def blocks(n, rng, tile=2.0, bw=0.2, bh=0.1, joint=0.003, chamfer=0.006, base=(178, 176, 170)):
    """Interlocking concrete block paving in stretcher bond. Returns (rgb, height
    in mm, roughness). / インターロッキングブロック（馬目地）。"""
    mpp = tile / n
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float64) * mpp
    row = np.floor(yy / bh).astype(int)
    xo = xx + (row % 2) * (bw / 2)
    colm = np.floor(xo / bw).astype(int)
    fx = xo - colm * bw
    fy = yy - row * bh
    # distance to the block edge (joint at the edges) / 目地までの距離
    dedge = np.minimum(np.minimum(fx, bw - fx), np.minimum(fy, bh - fy))
    top = np.clip((dedge - joint / 2) / chamfer, 0.0, 1.0)
    top = top * top * (3 - 2 * top)                      # chamfer / 面取り
    h = 4.0 * top - 1.0                                  # mm: joint 4 mm down / 目地は 4 mm 低い
    # per-block tone and tint, a few darker blocks / ブロックごとの色むら
    nr, nc = int(tile / bh) + 2, int(tile / bw) + 2
    tone = rng.uniform(0.93, 1.06, (nr, nc))
    tint = rng.uniform(-2.0, 2.0, (nr, nc, 1)) * np.array([1.0, 0.9, 0.7]) + rng.uniform(-1.5, 1.5, (nr, nc, 3))
    dark = (rng.random((nr, nc)) < 0.06) * rng.uniform(0.12, 0.2, (nr, nc))
    tone = tone - dark
    # wrap the block index so the tile repeats (period: 2 rows, 1 block) / 繰り返し
    ri = row % int(round(tile / bh))
    ci = colm % int(round(tile / bw))
    t = tone[ri, ci]
    b = np.array(base, np.float64)
    col = (b[None, None] + tint[ri, ci]) * t[..., None]
    col *= (0.9 + 0.1 * top)[..., None]                   # chamfer shade / 面取りの陰
    col = col * (1 - (1 - top))[..., None] + np.array([120.0, 112.0, 100.0])[None, None] * (1 - top)[..., None]
    grain = smooth(n, 1.0, rng)
    col += 7.0 * grain[..., None]
    col *= (1.0 + 0.06 * smooth(n, n / 6, rng))[..., None]   # broad weathering / 広い汚れ
    h = h + 0.25 * grain + 0.3 * smooth(n, 2.5, rng) * top
    r = 0.78 + 0.03 * grain - 0.2 * (1 - top) + 0.04 * (1 - t)
    r = np.where(top < 0.5, 0.95, r)
    return col, h, r


def tactile(n, rng, kind, c=(240, 200, 30)):
    """JIS T 9251 tactile paving block (0.3 m tile): "guide" = 4 bars along x (the
    strip direction), "warn" = 5 x 5 dots; 5 mm high with sloped sides.
    点字ブロック（線状・点状）。"""
    mm = 300.0 / n
    yy, xx = (np.mgrid[0:n, 0:n].astype(np.float64) + 0.5) * mm
    if kind == "guide":
        cy = np.array([37.5, 112.5, 187.5, 262.5])
        dy = np.min(np.abs(yy[..., None] - cy[None, None]), axis=2)
        dx = np.clip(np.maximum(15.0 - xx, xx - 285.0), 0.0, None)
        d = np.sqrt(dy * dy + dx * dx)
        prof = np.clip((14.0 - d) / 5.5, 0.0, 1.0)         # 17 mm top, 28 mm foot / 上面 17 mm 底 28 mm
    else:
        cs = np.array([30.0, 90.0, 150.0, 210.0, 270.0])
        dx = np.min(np.abs(xx[..., None] - cs[None, None]), axis=2)
        dy = np.min(np.abs(yy[..., None] - cs[None, None]), axis=2)
        d = np.sqrt(dx * dx + dy * dy)
        prof = np.clip((11.0 - d) / 5.0, 0.0, 1.0)         # 12 mm top, 22 mm foot / 上面 12 mm 底 22 mm
    h = 5.0 * prof
    edge = np.minimum(np.minimum(xx, 300 - xx), np.minimum(yy, 300 - yy))
    joint = np.clip(1.0 - edge / 2.0, 0.0, 1.0)            # 2 mm joint at the block edge / 目地
    h -= 3.0 * joint
    cc = np.array(c, np.float64)
    col = cc[None, None] * (0.96 + 0.08 * smooth(n, 3.0, rng))[..., None]
    col *= (1.0 - 0.05 * smooth(n, n / 5, rng) ** 2)[..., None]
    col *= (0.86 + 0.18 * prof)[..., None]                  # bar tops brighter than the base / 突起の上面
    col = col * (1 - joint[..., None]) + np.array([90.0, 85.0, 70.0])[None, None] * joint[..., None]
    r = 0.6 + 0.04 * smooth(n, 2.0, rng) - 0.1 * prof + 0.3 * joint
    return col, h, r


def save(d, name, col, h, r, strength, mat=None):
    Image.fromarray(np.clip(col, 0, 255).astype(np.uint8)).save(os.path.join(d, name + ".png"))
    h = h - h.mean()
    normal_map(h / max(np.abs(h).max(), 1e-6), strength).save(os.path.join(d, name + "_n.png"))
    r = r if r.shape[0] <= 512 else np.asarray(Image.fromarray(r.astype(np.float32)).resize((512, 512)))
    Image.fromarray((srgb_encode(r) * 255).astype(np.uint8)).save(os.path.join(d, (mat or name) + "_r.png"))
    Image.fromarray((srgb_encode(np.zeros((8, 8))) * 255).astype(np.uint8)).save(
        os.path.join(d, (mat or name) + "_m.png"))


def write(d):
    rng = np.random.default_rng(81)
    n = 1024
    # main roads: mid grey, fine aggregate (5 m tile -> 4.9 mm/px) / 幹線: 中間の灰色
    col, h, r = asphalt(n, rng, (42, 42, 44), (84, 83, 80), rough=(0.86, 0.06))
    save(d, "asphalt_jp", col, h, r, 1.6)
    # wheel paths: darker, aggregate polished flat, smoother / 轍部: 暗く滑らか
    col, h, r = asphalt(n, rng, (37, 37, 39), (72, 71, 69), polish=0.35, tone=0.08, rough=(0.74, 0.05))
    save(d, "asphalt_wheel", col, h, r, 1.2)
    # old estate asphalt: oxidised, lighter, ravelled / 古い生活道路: 退色・骨材露出
    col, h, r = asphalt(n, rng, (54, 53, 51), (96, 93, 88), density=0.40, size=1.1, tone=0.14,
                        pits=0.012, rough=(0.9, 0.05))
    save(d, "asphalt_estate", col, h, r, 1.9)
    # its edges, cracked / 縁部のひび割れ
    k = cracks(n, rng)
    col2 = col * (1.0 - 0.62 * k[..., None])
    save(d, "asphalt_crack", col2, h - 2.2 * k, r + 0.06 * k, 2.0)
    # patches and trench reinstatements: new black asphalt / 補修: 新しい黒い舗装
    col, h, r = asphalt(n, rng, (34, 34, 36), (66, 65, 63), density=0.28, size=0.8, tone=0.05,
                        pits=0.002, rough=(0.82, 0.04))
    save(d, "asphalt_patch", col, h, r, 1.3)
    # footways: fine dense asphalt (3 m tile) / 歩道: 細粒アスファルト
    col, h, r = asphalt(n, rng, (52, 52, 53), (92, 90, 87), density=0.30, size=0.7, tone=0.12,
                        rough=(0.88, 0.05))
    save(d, "pavement_asphalt", col, h, r, 1.5)
    # kerb top: a shade darker and warmer than the block paving beside it, so the
    # kerb line reads / 縁石天端（歩道のブロックより少し暗く）
    col, h, r = concrete(512, rng, (146, 143, 136), (176, 173, 165))
    save(d, "kerb_jp", col, h, r, 1.6)
    # kerb face: 0.6 m blocks on a 1.2 m tile, grime at road level (image
    # bottom = v 0 = the road; the 0.15 m face uses the lowest 1/8)
    # 縁石の立ち上がり: 0.6 m ブロック、路面際の汚れ
    col, h, r = concrete(512, rng, (158, 156, 148), (186, 184, 176))
    yy = np.arange(512)[:, None] / 511.0                  # 0 top .. 1 bottom / 上 0 下 1
    grime = np.clip((yy - 0.9) / 0.1, 0, 1) ** 1.5
    col *= (1.0 - 0.45 * grime)[..., None]
    for x in (0, 256):
        for dx in range(-2, 3):
            col[:, (x + dx) % 512] *= 0.62
            h[:, (x + dx) % 512] -= 1.2
    save(d, "kerb_face_jp", col, h, r + 0.04 * grime, 1.6)
    col, h, r = paint(512, rng, (236, 236, 230), wear=0.05)
    save(d, "line_white", col, h, r, 1.4)
    col, h, r = paint(512, rng, (232, 168, 34), wear=0.06)
    save(d, "line_yellow", col, h, r, 1.4)
    # pavements (pavements.py, ch. 31): block paving 2 m tile, tactile blocks 0.3 m
    # 歩道: インターロッキング（2 m）、点字ブロック（0.3 m）
    col, h, r = blocks(1024, rng)
    save(d, "pave_block", col, h, r, 2.2)
    col, h, r = tactile(256, rng, "guide")
    save(d, "pave_guide", col, h, r, 2.0)
    col, h, r = tactile(256, rng, "warn")
    save(d, "pave_warn", col, h, r, 2.0)
    # 路側帯: the cracked-edge asphalt under its own material name / 路側帯の材質
    for suf in ("_r.png", "_m.png"):
        Image.open(os.path.join(d, "asphalt_crack" + suf)).save(os.path.join(d, "asphalt_rosoku" + suf))
    # the general buildings' textures (General Painter, own random streams) / 一般建物のテクスチャ
    import textures_jp_bld
    textures_jp_bld.write(d)
