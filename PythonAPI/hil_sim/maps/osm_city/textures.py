"""Procedural, tileable textures and the material table (maps/.venv).

Every texture is generated here (noise, joints, paint, glass), none is
copied or traced from photos or map services.

  python textures.py <out_dir>       writes <out_dir>/textures/*.png

MATERIALS: name -> (texture or None, base colour if no texture, roughness,
metallic, smooth shading). Names avoid "sign" and "light" (CARLA's
importer drops meshes and materials with those in the name).

手続き生成のタイル状テクスチャと材質表。写真や地図サービスは使わない。
"""

import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

RNG = np.random.default_rng(11)


def noise(n, scales=(4, 16, 64), amps=(0.5, 0.3, 0.2)):
    """Tileable value noise in [0, 1]. / タイル可能なノイズ。"""
    out = np.zeros((n, n))
    for s, a in zip(scales, amps):
        g = RNG.random((s, s))
        im = Image.fromarray((g * 255).astype(np.uint8)).resize((n, n), Image.BICUBIC)
        g = np.asarray(im, np.float64) / 255.0
        # wrap: blend with a half-shifted copy so edges meet / 端をつなぐ
        g = 0.5 * (g + np.roll(np.roll(g, n // 2, 0), n // 2, 1))
        out += a * g
    out -= out.min()
    return out / max(out.max(), 1e-6)


def tint(n01, c0, c1):
    c0, c1 = np.array(c0, np.float64), np.array(c1, np.float64)
    return (c0 + (c1 - c0) * n01[..., None]).clip(0, 255).astype(np.uint8)


def speckle(img, density, c, rng=RNG):
    a = np.asarray(img).copy()
    n = a.shape[0]
    k = int(n * n * density)
    ys, xs = rng.integers(0, n, k), rng.integers(0, n, k)
    a[ys, xs] = c
    return Image.fromarray(a)


def asphalt(n=1024):
    base = tint(noise(n, (8, 32, 128, 512), (0.25, 0.25, 0.25, 0.25)), (40, 40, 42), (62, 62, 64))
    im = speckle(Image.fromarray(base), 0.02, (100, 100, 100))
    im = speckle(im, 0.01, (35, 35, 36))
    return im.filter(ImageFilter.GaussianBlur(0.6))


def concrete(n=512, c0=(170, 170, 165), c1=(200, 200, 195), joints=0, jc=(120, 120, 118)):
    im = Image.fromarray(tint(noise(n, (4, 16, 64, 256), (0.3, 0.3, 0.2, 0.2)), c0, c1))
    d = ImageDraw.Draw(im)
    for k in range(joints):
        x = int(k * n / joints)
        d.line([(x, 0), (x, n)], fill=jc, width=3)
    return im


def pavers(n=1024):
    """Interlocking rectangular pavers, red-brown and grey. / インターロッキング。"""
    im = Image.new("RGB", (n, n))
    d = ImageDraw.Draw(im)
    bw, bh = n // 10, n // 20                  # 0.2 x 0.1 m on a 2 m tile / 2 m に 0.2x0.1 m
    rows = n // bh
    for r in range(rows):
        off = (bw // 2) if r % 2 else 0
        for c in range(-1, n // bw + 1):
            x = c * bw + off
            v = RNG.random()
            col = (150 + int(30 * v), 82 + int(20 * v), 70 + int(15 * v)) if RNG.random() < 0.75 else \
                (150 + int(25 * v), 148 + int(25 * v), 140 + int(25 * v))
            d.rectangle([x + 2, r * bh + 2, x + bw - 2, r * bh + bh - 2], fill=col)
    a = np.asarray(im, np.float64) * (0.85 + 0.3 * noise(n)[..., None])
    return Image.fromarray(a.clip(0, 255).astype(np.uint8))


def grass(n=1024, c0=(58, 96, 38), c1=(98, 140, 58)):
    a = tint(noise(n, (8, 32, 128, 512), (0.2, 0.3, 0.25, 0.25)), c0, c1)
    im = speckle(Image.fromarray(a), 0.03, (120, 160, 70))
    return im.filter(ImageFilter.GaussianBlur(0.5))


def paint(n=512, c=(240, 238, 230), stain=0.10):
    """Painted render with faint rain streaks, as on Singapore estates. / 雨だれのある塗装壁。"""
    nn = noise(n, (4, 16, 64), (0.4, 0.4, 0.2))
    streak = np.asarray(Image.fromarray((RNG.random((8, 64)) * 255).astype(np.uint8)).resize((n, n), Image.BICUBIC),
                        np.float64) / 255.0
    k = 1.0 - stain * (0.6 * nn + 0.4 * streak)
    a = np.array(c, np.float64)[None, None, :] * k[..., None]
    return Image.fromarray(a.clip(0, 255).astype(np.uint8))


def window(n=256, panes=2, frame=(225, 225, 220), glass0=(70, 95, 110), glass1=(140, 165, 175), curtain=True):
    """One window, 0..1 in u and v: frame, mullions, a transom, glass. / 窓 1 枚分。"""
    y = np.linspace(0, 1, n)[:, None]
    g = np.array(glass1)[None, None, :] * (1 - y[..., None]) + np.array(glass0)[None, None, :] * y[..., None]
    g = np.repeat(g, n, axis=1)
    im = Image.fromarray(g.astype(np.uint8))
    d = ImageDraw.Draw(im)
    if curtain:
        for p in range(panes):
            if RNG.random() < 0.5:
                x0 = int(n * p / panes)
                d.rectangle([x0, int(n * 0.1), x0 + int(n / panes * RNG.uniform(0.2, 0.6)), n],
                            fill=tuple(int(v) for v in RNG.integers(150, 230, 3)))
    fw = max(4, n // 28)
    d.rectangle([0, 0, n - 1, n - 1], outline=frame, width=fw)
    for p in range(1, panes):
        x = int(n * p / panes)
        d.rectangle([x - fw // 2, 0, x + fw // 2, n], fill=frame)
    yt = int(n * 0.36)
    d.rectangle([0, yt - fw // 2, n, yt + fw // 2], fill=frame)
    return im


def roof_tiles(n=512, c=(170, 80, 55)):
    im = Image.fromarray(tint(noise(n), np.array(c) * 0.85, np.array(c) * 1.1))
    d = ImageDraw.Draw(im)
    for r in range(16):
        yy = int(r * n / 16)
        d.line([(0, yy), (n, yy)], fill=tuple(int(v * 0.6) for v in c), width=3)
        for k in range(16):
            xx = int((k + (0.5 if r % 2 else 0)) * n / 16)
            d.line([(xx, yy), (xx, yy + n // 16)], fill=tuple(int(v * 0.75) for v in c), width=1)
    return im


def stripes(n=256, c0=(30, 30, 30), c1=(240, 240, 240), k=8, vertical=False):
    im = Image.new("RGB", (n, n), c0)
    d = ImageDraw.Draw(im)
    for i in range(0, k, 2):
        a, b = int(i * n / k), int((i + 1) * n / k)
        d.rectangle([0, a, n, b] if not vertical else [a, 0, b, n], fill=c1)
    return im


def leaves(n=512, c0=(30, 70, 25), c1=(80, 130, 50)):
    a = tint(noise(n, (16, 64, 256), (0.3, 0.4, 0.3)), c0, c1)
    return speckle(Image.fromarray(a), 0.05, tuple(int(v * 0.6) for v in c0))


def shopfront(n=512):
    im = Image.new("RGB", (n, n), (60, 70, 80))
    d = ImageDraw.Draw(im)
    for k in range(4):
        x0 = k * n // 4
        col = tuple(int(v) for v in RNG.integers(120, 240, 3))
        d.rectangle([x0 + 8, n // 3, x0 + n // 4 - 8, n], fill=col)
        d.rectangle([x0 + 8, int(n * 0.12), x0 + n // 4 - 8, int(n * 0.26)], fill=tuple(int(v) for v in RNG.integers(30, 255, 3)))
        d.rectangle([x0, 0, x0 + 6, n], fill=(200, 200, 200))
    return im


PAINTS = {
    "paint_white": (240, 238, 232), "paint_cream": (238, 228, 200), "paint_palegrey": (205, 207, 205),
    "paint_skyblue": (110, 170, 215), "paint_navy": (40, 60, 105), "paint_terracotta": (190, 100, 70),
    "paint_brown": (110, 75, 55), "paint_teal": (50, 150, 150), "paint_grey": (120, 122, 125),
    "paint_mustard": (215, 175, 60), "paint_maroon": (120, 40, 45), "paint_sage": (150, 180, 140),
    "paint_green": (60, 120, 70), "paint_coral": (235, 125, 105), "paint_lavender": (175, 160, 205),
    "paint_purple": (100, 70, 130), "paint_peach": (240, 195, 160), "paint_charcoal": (60, 62, 66),
    "paint_woodbrown": (130, 90, 60), "paint_red": (190, 45, 40), "paint_orange": (230, 130, 40),
    "clad_silver": (190, 192, 195), "clad_beige": (215, 200, 175),
    # colours seen in Street View (Pasir Ris) / ストリートビューで見た色
    "paint_royalblue": (35, 85, 185), "paint_paleblue": (175, 205, 230), "paint_rust": (185, 90, 50),
    "paint_periwinkle": (140, 155, 220), "paint_lime": (165, 205, 80), "paint_ochre": (205, 150, 60),
    "paint_yellow": (235, 200, 70), "paint_brickred": (165, 72, 52),
}
ROOFS = {"roof_blue": (60, 100, 160), "roof_terracotta": (175, 85, 55), "roof_teal": (40, 130, 130),
         "roof_green": (60, 110, 70), "roof_grey": (95, 98, 102), "roof_brown": (110, 70, 50),
         "roof_orange": (215, 110, 35), "roof_purple": (135, 105, 175)}

def court(length, width, kind, base, line=(245, 245, 240), px_per_m=36):
    """A court or pitch with its line markings, the court's length along u.
    コート・グラウンドのライン。"""
    W, H = int(length * px_per_m), int(width * px_per_m)
    k = px_per_m
    im = Image.fromarray(tint(noise(max(W, H))[:H, :W], np.array(base) * 0.92, np.array(base) * 1.06))
    d = ImageDraw.Draw(im)
    lw = max(3, int(0.06 * k))
    P = lambda x, y: (W / 2 + x * k, H / 2 - y * k)
    def rect(x0, y0, x1, y1):
        d.rectangle([P(x0, y1), P(x1, y0)], outline=line, width=lw)
    def circ(cx, cy, r, a0=0, a1=360):
        (x0, y0), (x1, y1) = P(cx - r, cy + r), P(cx + r, cy - r)
        d.arc([x0, y0, x1, y1], a0, a1, fill=line, width=lw)
    if kind == "basketball":
        hl, hw = 14.0, 7.5
        rect(-hl, -hw, hl, hw)
        d.line([P(0, -hw), P(0, hw)], fill=line, width=lw)
        circ(0, 0, 1.8)
        for s in (-1, 1):
            bx = s * (hl - 1.575)
            x0, x1 = sorted((s * hl, s * (hl - 5.8)))
            rect(x0, -2.45, x1, 2.45)
            circ(s * (hl - 5.8), 0, 1.8)
            a0, a1 = (90 - 68, 90 + 68) if s < 0 else (270 - 68, 270 + 68)
            a0, a1 = (-68, 68) if s < 0 else (180 - 68, 180 + 68)
            circ(bx, 0, 6.75, a0, a1)
            for e in (-1, 1):
                d.line([P(s * hl, e * 6.6), P(s * (hl - 3.0), e * 6.6)], fill=line, width=lw)
    elif kind == "tennis":
        hl, hw, hs = 11.885, 5.485, 4.115
        rect(-hl, -hw, hl, hw)
        for e in (-1, 1):
            d.line([P(-hl, e * hs), P(hl, e * hs)], fill=line, width=lw)
        for s in (-1, 1):
            d.line([P(s * 6.4, -hs), P(s * 6.4, hs)], fill=line, width=lw)
        d.line([P(-6.4, 0), P(6.4, 0)], fill=line, width=lw)
    elif kind == "football":
        hl, hw = length / 2 - 1.0, width / 2 - 1.0
        rect(-hl, -hw, hl, hw)
        d.line([P(0, -hw), P(0, hw)], fill=line, width=lw)
        circ(0, 0, min(9.15, hw * 0.3))
        pa_d, pa_w = min(16.5, hl * 0.3), min(40.3, hw * 1.2) / 2
        for s in (-1, 1):
            x0, x1 = sorted((s * hl, s * (hl - pa_d)))
            rect(x0, -pa_w, x1, pa_w)
            x0, x1 = sorted((s * hl, s * (hl - pa_d * 0.35)))
            rect(x0, -pa_w * 0.45, x1, pa_w * 0.45)
    elif kind == "netball":
        hl, hw = 15.25, 7.625
        rect(-hl, -hw, hl, hw)
        for s in (-1, 1):
            d.line([P(s * hl / 3, -hw), P(s * hl / 3, hw)], fill=line, width=lw)
            circ(s * hl, 0, 4.9, 90 if s > 0 else -90, 270 if s > 0 else 90)
        circ(0, 0, 0.45)
    elif kind == "multi":
        rect(-length / 2 + 0.5, -width / 2 + 0.5, length / 2 - 0.5, width / 2 - 0.5)
        d.line([P(0, -width / 2 + 0.5), P(0, width / 2 - 0.5)], fill=line, width=lw)
    return im


def bricks(n=512, c=(160, 70, 50)):
    """Brick facing: running bond, light joints. / レンガ張り。"""
    im = Image.new("RGB", (n, n), (200, 190, 175))
    d = ImageDraw.Draw(im)
    bh, bw = n // 32, n // 8
    for r in range(32):
        off = bw // 2 if r % 2 else 0
        for k in range(-1, 9):
            v = RNG.random()
            col = tuple(int(ch * (0.85 + 0.3 * v)) for ch in c)
            d.rectangle([k * bw + off + 1, r * bh + 1, k * bw + off + bw - 2, r * bh + bh - 2], fill=col)
    return im


def solar_panel(n=256):
    """A photovoltaic panel: dark blue cells, silver frame and gaps. / 太陽光パネル。"""
    yy, xx = np.indices((n, n))
    cell = (yy % 32 > 2) & (xx % 43 > 2) & (yy > 4) & (yy < n - 5) & (xx > 4) & (xx < n - 5)
    a = np.where(cell[..., None], np.array([28, 42, 78]), np.array([175, 180, 188])).astype(np.float64)
    a *= (0.92 + 0.12 * noise(n))[..., None]
    return Image.fromarray(a.clip(0, 255).astype(np.uint8))


def kerb_bw(n=256):
    """Black and white kerb paint, 1 m each (texture tile = 2 m). / 白黒の縁石。"""
    a = np.zeros((n, n, 3), np.uint8)
    a[:, : n // 2] = (235, 235, 230)
    a[:, n // 2:] = (30, 30, 30)
    return Image.fromarray((a * (0.9 + 0.15 * noise(n)[..., None])).clip(0, 255).astype(np.uint8))


# name: (texture file, colour, roughness, metallic, smooth) / 材質表
MATERIALS = {
    "asphalt": ("asphalt", None, 0.88, 0.0, False),
    "asphalt_lot": ("asphalt_lot", None, 0.9, 0.0, False),
    "kerb": ("kerb", None, 0.8, 0.0, False),
    "paint_white": (None, (235, 235, 230), 0.55, 0.0, False),
    "paint_yellow": (None, (230, 185, 30), 0.6, 0.0, False),
    "pavers": ("pavers", None, 0.85, 0.0, False),
    "concrete_path": ("concrete_path", None, 0.85, 0.0, False),
    "park_connector": ("park_connector", None, 0.85, 0.0, False),
    "grass": ("grass", None, 0.95, 0.0, False),
    "grass_park": ("grass_park", None, 0.95, 0.0, False),
    "soil": ("soil", None, 0.95, 0.0, False),
    "pitch": ("pitch", None, 0.9, 0.0, False),
    "rubber": ("rubber", None, 0.9, 0.0, False),
    "water": (None, (60, 150, 190), 0.05, 0.0, False),
    "pool_tile": ("pool_tile", None, 0.3, 0.0, False),
    "concrete": ("concrete", None, 0.85, 0.0, False),
    "roof_concrete": ("roof_concrete", None, 0.9, 0.0, False),
    "roof_metal": ("roof_metal", None, 0.5, 0.6, False),
    "roof_linkway": ("roof_linkway", None, 0.5, 0.3, False),
    "void_floor": ("void_floor", None, 0.4, 0.0, False),
    "corridor_dark": (None, (55, 55, 58), 0.9, 0.0, False),
    "door_wood": (None, (120, 80, 50), 0.6, 0.0, False),
    "railing": (None, (90, 95, 100), 0.4, 0.7, False),
    "louvre": ("louvre", None, 0.6, 0.0, False),
    "tank": (None, (215, 215, 210), 0.7, 0.0, False),
    "frame": (None, (215, 215, 210), 0.4, 0.4, False),
    "glass": ("glass", None, 0.08, 0.2, False),
    "glass_2": ("glass_2", None, 0.08, 0.2, False),
    "glass_3": ("glass_3", None, 0.08, 0.2, False),
    "glass_dark": ("glass_dark", None, 0.05, 0.3, False),
    "glass_dark_2": ("glass_dark", None, 0.05, 0.3, False),
    "glass_dark_3": ("glass_dark", None, 0.05, 0.3, False),
    "glass_rail": (None, (170, 205, 205), 0.05, 0.1, False),
    "shopfront": ("shopfront", None, 0.15, 0.1, False),
    "bark": (None, (85, 70, 55), 0.9, 0.0, True),
    "bark_palm": (None, (140, 130, 115), 0.9, 0.0, True),
    "leaves_rain": ("leaves_rain", None, 0.8, 0.0, True),
    "leaves_dark": ("leaves_dark", None, 0.8, 0.0, True),
    "leaves_pale": ("leaves_pale", None, 0.8, 0.0, True),
    "leaves_palm": ("leaves_palm", None, 0.8, 0.0, False),
    "metal_grey": (None, (125, 130, 135), 0.4, 0.7, True),
    "lamp_head": (None, (60, 62, 66), 0.4, 0.5, False),
    "barrier": ("barrier", None, 0.6, 0.2, False),
    "pole_stripes": ("pole_stripes", None, 0.5, 0.0, True),
    "beacon_amber": (None, (255, 170, 30), 0.3, 0.0, True),
}
MATERIALS.update({
    "court_bb_blue": ("court_bb_blue", None, 0.7, 0.0, False), "court_bb_green": ("court_bb_green", None, 0.7, 0.0, False),
    "court_bb_red": ("court_bb_red", None, 0.7, 0.0, False), "court_tennis": ("court_tennis", None, 0.7, 0.0, False),
    "court_multi": ("court_multi", None, 0.7, 0.0, False), "pitch_astro": ("pitch_astro", None, 0.9, 0.0, False),
    "pitch_grass": ("pitch_grass", None, 0.95, 0.0, False), "parade_red": ("parade_red", None, 0.8, 0.0, False),
    "play_rubber": ("play_rubber", None, 0.85, 0.0, False), "backboard": (None, (235, 235, 235), 0.3, 0.0, False),
    "goal_white": (None, (240, 240, 240), 0.4, 0.2, True), "net_dark": (None, (40, 40, 40), 0.8, 0.0, False),
    "fence_green": (None, (40, 105, 60), 0.5, 0.4, False), "hull_white": (None, (240, 240, 240), 0.5, 0.0, False),
    "hull_red": (None, (200, 50, 45), 0.5, 0.0, False), "deck_wood": ("deck_wood", None, 0.8, 0.0, False),
    "slide_yellow": (None, (240, 200, 40), 0.3, 0.0, False), "kerb_bw": ("kerb_bw", None, 0.7, 0.0, False),
    "pavers_red": ("pavers_red", None, 0.85, 0.0, False),
    "court_netball": ("court_netball", None, 0.7, 0.0, False), "glass_teal": ("glass_teal", None, 0.08, 0.2, False),
    "glass_teal_2": ("glass_teal", None, 0.08, 0.2, False), "glass_teal_3": ("glass_teal", None, 0.08, 0.2, False),
    "solar": ("solar", None, 0.15, 0.4, False), "windbreak_blue": ("windbreak_blue", None, 0.8, 0.0, False),
    "gate_blue": (None, (40, 95, 175), 0.4, 0.5, False),
})
for k, c in PAINTS.items():
    MATERIALS[k] = (k, None, 0.8, 0.0, False)
for k, c in ROOFS.items():
    MATERIALS[k] = (k, None, 0.7, 0.0, False)


# surface relief: (normal strength, roughness variation, fine-noise mix) per
# material; anything not listed is smooth paint-like / 表面の凹凸と粗さのばらつき
SURFACE = {
    "asphalt": (2.2, 0.08, 0.95), "asphalt_lot": (2.0, 0.08, 0.95), "kerb": (1.5, 0.06, 0.6),
    "pavers": (4.0, 0.08, 0.2), "concrete_path": (1.5, 0.06, 0.6), "park_connector": (1.8, 0.06, 0.9),
    "grass": (7.0, 0.04, 0.8), "grass_park": (7.0, 0.04, 0.8), "soil": (5.0, 0.05, 0.7),
    "pitch": (5.0, 0.04, 0.8), "rubber": (3.0, 0.05, 0.6), "pool_tile": (2.0, 0.05, 0.1),
    "concrete": (2.0, 0.06, 0.4), "roof_concrete": (2.5, 0.06, 0.5), "roof_metal": (2.0, 0.08, 0.1),
    "roof_linkway": (1.5, 0.08, 0.1), "void_floor": (1.5, 0.05, 0.2), "louvre": (3.0, 0.05, 0.0),
    "barrier": (2.0, 0.05, 0.3), "leaves_rain": (5.0, 0.05, 0.7), "leaves_dark": (5.0, 0.05, 0.7),
    "leaves_pale": (5.0, 0.05, 0.7), "leaves_palm": (4.0, 0.05, 0.6), "bark": (5.0, 0.04, 0.7),
    "bark_palm": (4.0, 0.04, 0.6),
}
for _k in list(PAINTS) + list(ROOFS):
    SURFACE.setdefault(_k, (3.0, 0.06, 0.3) if _k in ROOFS else (1.2, 0.05, 0.5))


def height_of(img, fine_mix):
    """A height map from the colour (darker = lower: joints, gaps) plus fine noise.
    色（暗い所が低い: 目地・隙間）と細かいノイズから高さ。"""
    a = np.asarray(img.convert("L"), np.float64) / 255.0
    n = a.shape[0]
    # grain only: a few centimetres on these tiles (2-6 m) / 数 cm の粒だけ
    fine = noise(n, (n // 4, n // 2, n), (0.3, 0.4, 0.3)) if n >= 16 else np.zeros_like(a)
    h = (1 - fine_mix) * a + fine_mix * fine
    return h - h.mean()


def normal_map(h, strength):
    """Tangent-space normal map from a height map, DirectX green (what Unreal
    expects). / 高さから法線マップ（Unreal 用に DirectX 形式）。"""
    dx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * strength
    dy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * strength
    nz = np.ones_like(h)
    ln = np.sqrt(dx * dx + dy * dy + nz * nz)
    nx, ny, nz = -dx / ln, dy / ln, nz / ln      # image rows run down: +dy is DirectX green / 行は下向き
    rgb = np.stack([nx, ny, nz], -1) * 0.5 + 0.5
    return Image.fromarray((rgb * 255).clip(0, 255).astype(np.uint8))


def srgb_encode(v):
    """Unreal reads these maps as sRGB: store v^(1/2.2) so it gets v back. / sRGB 補正。"""
    return np.clip(v, 0, 1) ** (1 / 2.2)


def write_surface_maps(d, tex, mats=None):
    """<texture>_n.png (normal) and <material>_r.png (roughness), <material>_m.png
    (metallic) for every material. / 各材質の法線・粗さ・金属度マップ。"""
    made = {}
    for name, (tx, col, rough, metal, smooth) in (MATERIALS if mats is None else mats).items():
        strength, var, fine = SURFACE.get(name, (0.0, 0.03, 0.0))
        if tx and tx in tex:
            img = tex[tx]
            n = min(img.size[0], 512)
            if n != img.size[0]:
                img = img.resize((n, n), Image.BILINEAR)
            h = height_of(img, fine)
            if tx not in made and strength > 0:
                normal_map(h / max(np.abs(h).max(), 1e-6), strength).save(os.path.join(d, tx + "_n.png"))
                made[tx] = True
            r = rough + var * (h / max(np.abs(h).max(), 1e-6))
        else:
            r = np.full((8, 8), rough)
        Image.fromarray((srgb_encode(r) * 255).astype(np.uint8)).save(os.path.join(d, name + "_r.png"))
        Image.fromarray((srgb_encode(np.full((8, 8), metal)) * 255).astype(np.uint8)).save(
            os.path.join(d, name + "_m.png"))


def materials(style=None):
    """The material table for a map's road style (maps.json "road_style").
    地図の道路様式に応じた材質表。"""
    m = dict(MATERIALS)
    if style == "japan":
        import textures_jp
        m.update(textures_jp.MATERIALS)
    return m


def write_all(out_dir, style=None):
    d = os.path.join(out_dir, "textures")
    os.makedirs(d, exist_ok=True)
    tex = {
        "asphalt": asphalt(),
        "asphalt_lot": asphalt().point(lambda v: min(255, int(v * 1.08))),
        "kerb": concrete(512, (175, 175, 170), (205, 205, 200), joints=1),
        "pavers": pavers(),
        "concrete_path": concrete(512, (180, 178, 170), (205, 203, 195), joints=1),
        "park_connector": Image.fromarray(tint(noise(512), (150, 70, 55), (180, 95, 75))),
        "grass": grass(),
        "grass_park": grass(1024, (52, 105, 40), (95, 150, 60)),
        "soil": Image.fromarray(tint(noise(512), (120, 95, 70), (160, 130, 95))),
        "pitch": grass(512, (50, 120, 55), (70, 145, 70)),
        "rubber": Image.fromarray(tint(noise(512, (32, 128, 512)), (160, 70, 55), (60, 130, 80))),
        "pool_tile": concrete(256, (120, 190, 220), (150, 210, 235), joints=8, jc=(230, 240, 245)),
        "concrete": concrete(512),
        "roof_concrete": concrete(512, (130, 128, 122), (170, 168, 160)),
        "roof_metal": stripes(256, (150, 155, 160), (175, 180, 185), 24, vertical=True),
        "roof_linkway": stripes(256, (70, 110, 120), (90, 130, 140), 16, vertical=True),
        "void_floor": concrete(256, (165, 140, 120), (190, 165, 140), joints=4, jc=(140, 120, 100)),
        "louvre": stripes(256, (150, 150, 145), (215, 215, 210), 16),
        "glass": window(256, 1),
        "glass_2": window(256, 2),
        "glass_3": window(256, 3),
        "glass_dark": Image.fromarray(tint(noise(256, (2, 8)), (30, 45, 60), (60, 80, 95))),
        "shopfront": shopfront(),
        "leaves_rain": leaves(512, (35, 80, 28), (85, 140, 55)),
        "leaves_dark": leaves(512, (25, 60, 22), (60, 105, 40)),
        "leaves_pale": leaves(512, (60, 110, 45), (130, 170, 80)),
        "leaves_palm": leaves(256, (50, 95, 35), (110, 150, 60)),
        "barrier": concrete(512, (140, 160, 150), (165, 185, 175), joints=2),
        "pole_stripes": stripes(128, (20, 20, 20), (240, 240, 240), 10),
    }
    tex.update({
        "court_bb_blue": court(28, 15, "basketball", (45, 95, 160)),
        "court_bb_green": court(28, 15, "basketball", (50, 120, 85)),
        "court_bb_red": court(28, 15, "basketball", (165, 70, 55)),
        "court_tennis": court(34, 16, "tennis", (60, 120, 90)),
        "court_multi": court(20, 10, "multi", (70, 120, 100)),
        "pitch_astro": court(100, 64, "football", (55, 135, 65), px_per_m=12),
        "pitch_grass": court(100, 64, "football", (75, 130, 55), px_per_m=12),
        "parade_red": Image.fromarray(tint(noise(512), (150, 60, 50), (175, 80, 65))),
        "play_rubber": Image.fromarray(tint(noise(512, (16, 64, 256)), (40, 110, 160), (70, 150, 110))),
        "deck_wood": stripes(256, (140, 95, 60), (165, 115, 75), 12, vertical=True),
        "kerb_bw": kerb_bw(),
        "pavers_red": bricks(512, (150, 70, 55)),
        "court_netball": court(32, 17, "netball", (60, 70, 140)),
        "glass_teal": window(256, 3, frame=(205, 210, 210), glass0=(40, 110, 110), glass1=(110, 175, 170), curtain=False),
        "solar": solar_panel(),
        "windbreak_blue": Image.fromarray(tint(noise(256, (8, 32, 128)), (40, 100, 170), (70, 130, 200))),
    })
    for k, c in PAINTS.items():
        tex[k] = paint(512, c, 0.08 if k.startswith("paint") else 0.04)
    tex["paint_brickred"] = bricks(512, (165, 72, 52))      # brick facing, not paint / レンガ張り
    for k, c in ROOFS.items():
        tex[k] = roof_tiles(512, c)
    for k, im in tex.items():
        im.save(os.path.join(d, k + ".png"))
    write_surface_maps(d, tex)
    if style == "japan":
        # Japanese road surfaces and paint (own generator and random stream:
        # the textures above stay exactly as they were) / 日本の路面と標示
        import textures_jp
        textures_jp.write(d)
    return d


if __name__ == "__main__":
    print(write_all(sys.argv[1]))
