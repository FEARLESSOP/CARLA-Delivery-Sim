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
TILE = {"asphalt": 5.0, "pavement": 3.0, "kerb": 1.0, "kerb_face": 1.2, "paint": 1.0}


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
    # kerb top / 縁石天端
    col, h, r = concrete(512, rng, (160, 158, 150), (190, 188, 180))
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
    # the general buildings' textures (General Painter, own random streams) / 一般建物のテクスチャ
    import textures_jp_bld
    textures_jp_bld.write(d)
