"""Japanese general buildings' textures and materials (maps/.venv), for maps.json
"road_style": "japan" (General Painter, guide ch. 6 and 13). Procedural only:
Street View photos gave the colours (colour_notes.json "palette_jp"), never
pixels.

Every texture is drawn from its own height field (metres), from which the
normal map (<tex>_n.png, DirectX green) and the roughness map (<mat>_r.png,
sRGB-encoded, ch. 2) are made; glass and metal frames also get a metallic
map. Arrays are built z-up (row 0 = v 0 = the bottom of the image) and
flipped when saved.

UVs. buildings_jp gives jp_* faces metric UVs (u along the wall, v = local
height above the terrace; roofs: u along the eave, v up the slope) and
finish() divides them by UV_TILE (metres per repeat), so a wall texture's
lowest rows are the wall's foot: the darker base band and the rain streaks
under sills sit where they belong. PER_FACE materials (doors, shutters, AC
units, frosted glass, entrance glazing) map one texture onto one face.

  walls     ceramic siding (board / stone / tile / vertical board) and
            render with rain streaks, 3 x 9 m tile, 170 px/m
  concrete  danchi render with floor joints, 4 x 16 m, 128 px/m
  tiles     45 mm mosaic in three shades (mansions), 3 x 6 m, 256 px/m
  roofs     kawara (ibushi, black, brown, blue, terracotta), colonial slate,
            standing-seam steel, galvalume; 4 x 4 m, 256 px/m
  lots      ishigaki, cast concrete with tie holes, kenchi block, CB block
            wall with a cap course, terrace concrete
  details   painted steel with rust, rails, aluminium, AC unit, doors,
            roll-up shutter, frosted and clear window glass, sills, soffit,
            gutters, kanban bands with abstract lettering, awnings, shopfront

日本の一般建物のテクスチャと材質（手続き生成。色は Street View の観察から）。
"""

import json
import math
import os
import random

import numpy as np
from PIL import Image, ImageDraw

from textures import normal_map, srgb_encode

# -- colours (sRGB) observed in Street View (colour_notes.json "palette_jp"
# overrides these; ch. 5) / 色（Street View の観察値。colour_notes.json が優先）
PALETTE = {
    "white": (232, 230, 222), "ivory": (228, 219, 198), "beige": (206, 190, 164), "grey": (178, 178, 173),
    "darkgrey": (98, 100, 103), "brown": (120, 99, 84), "mocha": (172, 154, 132), "navy": (66, 76, 94),
    "concrete_cream": (218, 212, 198), "concrete_white": (214, 211, 205),
    "tile_white": (224, 220, 210), "tile_beige": (192, 180, 162), "tile_brown": (134, 116, 104),
    "tile_darkbrown": (104, 88, 76),
    "kawara_ibushi": (92, 96, 102), "kawara_black": (44, 46, 50), "kawara_brown": (104, 74, 58),
    "kawara_blue": (62, 84, 108), "kawara_terracotta": (162, 92, 64), "slate": (60, 58, 57),
    "metal_roof": (74, 76, 79), "metal_roof_brown": (92, 72, 58), "galvalume": (176, 179, 180),
    "siding_metal": (186, 186, 182), "kenchi": (136, 128, 116), "retaining": (160, 158, 150),
    "ishigaki": (132, 126, 116), "block": (152, 150, 144), "foundation": (152, 151, 146),
    "lot": (166, 164, 157), "shutter": (178, 181, 184), "metal_white": (224, 222, 214),
    "metal_dark": (62, 60, 58), "alu": (176, 179, 181),
    # estate samples (photo-getter, estate_photos/colour_samples.json) / 住宅地の観察
    "paleblue": (176, 196, 210), "blue": (96, 128, 170), "brick_red": (150, 86, 66),
    "twotone_brick": (122, 94, 78), "slate_grey": (100, 100, 99), "slate_brown": (88, 62, 54),
    "brick_beige": (188, 160, 140),
    # city-centre offices (OitaStation core Street View, photos/core_C1..C5, 2026-11-08): precast /
    # stone panel cladding and curtain-wall glass tints / 駅前の事務所: パネル外装・カーテンウォール
    "panel_white": (216, 216, 210), "panel_grey": (176, 177, 174), "panel_darkgrey": (92, 95, 98),
    "panel_beige": (198, 188, 170), "panel_brown": (150, 100, 84),
    "curtain_blue": (78, 110, 138), "curtain_green": (84, 120, 112), "curtain_dark": (52, 60, 68),
    "curtain_silver": (140, 150, 158), "louvre": (168, 172, 174),
}

# wall colour -> pattern variants (the plain name is ceramic board siding)
# 外壁色ごとの柄（無印は窯業系サイディングの板柄）
WALL_VARIANTS = {
    "white": ["", "_render", "_stone", "_twotone"], "ivory": ["", "_render", "_tile", "_twotone"],
    "beige": ["", "_stone", "_tile", "_render", "_twotone"],
    "grey": ["", "_render", "_stone"], "darkgrey": ["", "_vboard"], "brown": ["", "_tile", "_stone"],
    "mocha": ["", "_tile", "_render"], "navy": ["", "_vboard"],
    "paleblue": ["", "_render"], "blue": ["", "_tile"], "brick_red": [""],
}
# the plain name's pattern where it is not board siding / 無印が板柄でない色
BASE_PATTERN = {"brick_red": "_tile"}
TWOTONE_Z = 3.45        # ground-floor top of a house (0.6 m floor + 2.85 m): brick below, siding above / 1 階上端

# metres per texture repeat (u, v) for metric UVs / テクスチャ 1 枚の大きさ (m)
UV_TILE = {}
# materials whose mesh UVs are already divided (m.flat(..., 2.0)) / 既に縮尺済みの UV
UV_PRE = {"jp_lot": 2.0, "jp_roof_flat": 2.0}
# one texture per face, and the UV spot of a plain part for small faces
# 面ごとに 1 枚のテクスチャ（小さな面は無地の部分の UV）
PER_FACE = {"jp_glass_office": (0.5, 0.5), "jp_door": (0.5, 0.15), "jp_door_steel": (0.5, 0.25), "jp_shutter": (0.5, 0.5),
            "jp_ac_unit": (0.92, 0.5), "jp_glass_frosted": (0.5, 0.5), "jp_glass_dark": (0.5, 0.5)}
FULL_ALWAYS = {"jp_ac_unit"}
MATERIALS = {}          # name: (texture, None, roughness, metallic, smooth) / 材質表
RENAME = {"concrete": "jp_sill", "roof_concrete": "jp_roof_flat", "shopfront": "jp_shopfront",
          "glass_dark": "jp_glass_dark", "glass_dark_2": "jp_glass_dark"}


def _mat(name, rough, metal=0.0, tile=None):
    MATERIALS[name] = (name, None, rough, metal, False)
    if tile:
        UV_TILE[name] = tile


# -- the material table (filled at import so the manifest knows them) / 材質表
for _c in WALL_VARIANTS:
    for _v in WALL_VARIANTS[_c]:
        _mat("jp_wall_%s%s" % (_c, _v), 0.8, 0.0, (3.0, 9.0))
for _k in ("concrete_cream", "concrete_white"):
    _mat("jp_" + _k, 0.88, 0.0, (4.0, 16.0))
for _k in ("tile_white", "tile_beige", "tile_brown", "tile_darkbrown"):
    _mat("jp_" + _k, 0.45, 0.0, (3.0, 6.0))
for _k, _r, _m in (("kawara_ibushi", 0.55, 0.0), ("kawara_black", 0.3, 0.0), ("kawara_brown", 0.32, 0.0),
                   ("kawara_blue", 0.3, 0.0), ("kawara_terracotta", 0.4, 0.0), ("roof_slate", 0.68, 0.0),
                   ("roof_metal", 0.45, 0.15), ("roof_metal_brown", 0.45, 0.15), ("roof_galvalume", 0.35, 0.75)):
    _mat("jp_" + _k, _r, _m, (4.0, 4.0))
_mat("jp_metal_siding", 0.4, 0.6, (4.0, 4.0))
_mat("jp_ridge", 0.5, 0.0, (1.0, 1.0))
_mat("jp_ridge_metal", 0.45, 0.2, (1.0, 1.0))           # slate and metal roofs' ridge cap / スレート・金属屋根の棟
_mat("jp_roof_slate_grey", 0.66, 0.0, (4.0, 4.0))
_mat("jp_roof_slate_brown", 0.66, 0.0, (4.0, 4.0))
_mat("jp_soffit", 0.85, 0.0, (2.0, 2.0))
_mat("jp_gutter", 0.5, 0.0, (1.0, 1.0))
_mat("jp_pipe", 0.55, 0.0, (1.0, 1.0))
_mat("jp_foundation", 0.88, 0.0, (4.0, 4.0))
_mat("jp_corridor", 0.8, 0.0, (2.0, 2.0))
_mat("jp_dark", 0.9, 0.0, (2.0, 2.0))
_mat("jp_number", 0.6, 0.0, (1.0, 1.0))
_mat("jp_lot", 0.88, 0.0, (6.0, 6.0))
_mat("jp_roof_flat", 0.85, 0.0, (6.0, 6.0))
_mat("jp_ishigaki", 0.9, 0.0, (4.0, 8.0))
_mat("jp_retaining", 0.88, 0.0, (4.0, 8.0))
_mat("jp_retaining_kenchi", 0.88, 0.0, (4.0, 8.0))
_mat("jp_block_wall", 0.9, 0.0, (4.0, 2.0))
_mat("jp_block_wall_brick", 0.85, 0.0, (4.0, 2.0))      # brick-tile plinth walls of newer estates / レンガタイル塀
_mat("jp_hedge", 0.9, 0.0, (2.0, 2.0))
_mat("jp_metal_white", 0.5, 0.0, (2.0, 2.0))
_mat("jp_metal_dark", 0.5, 0.0, (2.0, 2.0))
_mat("jp_alu", 0.35, 0.8, (1.0, 1.0))
_mat("jp_sill", 0.4, 0.7, (1.0, 1.0))
for _k in ("alu", "white", "dark"):
    _mat("jp_rail_" + _k, 0.45, 0.6 if _k == "alu" else 0.0, (2.0, 2.0))
for _k in ("white", "red", "blue", "green", "yellow"):
    _mat("jp_kanban_" + _k, 0.4, 0.0, (4.0, 4.0))
for _k in ("red", "green", "brown"):
    _mat("jp_awning_" + _k, 0.75, 0.0, (2.0, 2.0))
_mat("jp_shopfront", 0.1, 0.0, (3.6, 3.0))
for _k in PER_FACE:
    _mat(_k, {"jp_door": 0.55, "jp_door_steel": 0.45, "jp_shutter": 0.45, "jp_ac_unit": 0.55,
              "jp_glass_frosted": 0.25, "jp_glass_dark": 0.05, "jp_glass_office": 0.05}[_k],
         {"jp_shutter": 0.5, "jp_glass_dark": 0.2}.get(_k, 0.0))
for _p in ("1", "2", "3"):
    for _v in ("a", "b"):
        _mat("jp_glass_%s%s" % (_p, _v), 0.05, 0.0)
# city-centre offices, hotels, car parks (buildings_jp b_office / b_carpark): wall UVs metric
# and anchored (v = local height), floor lines at FLOOR_Z0 + k FLOOR_M (the first upper floor
# of an office at 4.8 m), so joints, spandrels and streaks sit on the floors
# 駅前の事務所・ホテル・立体駐車場（UV は高さ基準、床線 = FLOOR_Z0 + k FLOOR_M）
FLOOR_M, FLOOR_Z0 = 3.6, 1.2
PANELS = ("white", "grey", "darkgrey", "beige", "brown")
CURTAINS = ("blue", "green", "dark", "silver")
for _k in PANELS:
    _mat("jp_panel_" + _k, 0.62, 0.0, (3.6, 7.2))
for _k in CURTAINS:
    _mat("jp_curtain_" + _k, 0.06, 0.0, (7.2, 7.2))
for _v in ("a", "b"):
    _mat("jp_ribbon_" + _v, 0.06, 0.0, (6.0, 7.2))
_mat("jp_louvre", 0.4, 0.5, (2.0, 2.0))


# ---------------------------------------------------------------- helpers
def sm(h, w, sigma, rng, sx=None):
    """Tileable smooth noise in [-1, 1] (periodic Gaussian blur of white noise;
    sx: a different blur along x, in pixels). / タイル可能な滑らかなノイズ。"""
    a = rng.standard_normal((h, w))
    ky = np.fft.fftfreq(h)[:, None]
    kx = np.fft.fftfreq(w)[None, :]
    sx = sigma if sx is None else sx
    g = np.exp(-2 * np.pi ** 2 * ((sx * kx) ** 2 + (sigma * ky) ** 2))
    a = np.real(np.fft.ifft2(np.fft.fft2(a) * g))
    a -= a.mean()
    return a / max(np.abs(a).max(), 1e-9)


def grid(h, w, ppm):
    """(x, z) in metres of each pixel centre, z-up. / 画素中心の座標 (m)。"""
    z, x = np.mgrid[0:h, 0:w].astype(np.float64)
    return (x + 0.5) / ppm, (z + 0.5) / ppm


def groove(d, width):
    """0..1 groove profile for a distance d (m) from a joint line. / 目地の断面。"""
    return np.clip(1.0 - np.abs(d) / width, 0.0, 1.0)


def dist_to_lines(t, period, offset=0.0):
    """Signed distance (m) to the nearest of lines every ``period``. / 周期線までの距離。"""
    q = (t - offset) / period
    return (q - np.round(q)) * period


def streaks(h, w, ppm, rng, n, starts, length=(0.4, 1.6), width=(0.015, 0.1), amount=(0.3, 1.0)):
    """Rain streaks: n vertical stains hanging from heights ``starts`` (m,
    jittered) or anywhere, fading downward; wraps in x. Returns 0..1.
    雨だれ（窓台などの高さから下へ薄れる縦じみ）。"""
    out = np.zeros((h, w))
    jit = sm(h, w, 1.5, rng, sx=0.6 * ppm)
    for _ in range(n):
        x0 = rng.uniform(0, w)
        z0 = (rng.choice(starts) + rng.normal(0, 0.03)) if starts and rng.random() < 0.8 else rng.uniform(0, h / ppm)
        L = rng.uniform(*length)
        wd = rng.uniform(*width) * ppm
        r1 = int(np.clip(z0 * ppm, 1, h))
        r0 = int(np.clip((z0 - L) * ppm, 0, h - 1))
        if r1 <= r0:
            continue
        cols = np.arange(int(x0 - 2.5 * wd) - 1, int(x0 + 2.5 * wd) + 2)
        prof = np.exp(-((cols - x0) / max(wd, 0.7)) ** 2)
        rows = np.arange(r0, r1)
        below = (r1 - rows) / ppm                    # metres under the start / 起点からの下がり
        fade = np.clip(below / 0.05, 0, 1) * np.exp(-below / (0.45 * L))
        a = rng.uniform(*amount)
        out[rows[:, None], cols[None, :] % w] += a * fade[:, None] * prof[None, :]
    out *= (0.75 + 0.25 * jit)
    return np.clip(out, 0, 1)


def base_band(h, w, ppm, rng, z0, z1, amount):
    """The darker band at a wall's foot (splash, dust): full at z0, gone at
    z1, a ragged top. / 壁の足元の汚れ帯。"""
    _, z = grid(h, w, ppm)
    edge = z1 + 0.12 * (z1 - z0) * sm(h, w, 0.3 * ppm, rng, sx=0.4 * ppm)
    t = np.clip((edge - z) / max(z1 - z0, 1e-6), 0, 1) ** 1.6
    t[z < z0 - 0.02] *= 0.6
    return amount * t


def stain(col, mask, dirt=(78, 72, 64)):
    """Blend toward a dirt colour by mask (0..1). / 汚れ色へ寄せる。"""
    d = np.array(dirt, np.float64)
    return col * (1 - mask[..., None]) + d[None, None] * mask[..., None]


def colour_field(c, tone):
    """A base colour times (1 + tone). / 基本色 × (1 + 色むら)。"""
    return np.array(c, np.float64)[None, None] * (1.0 + tone)[..., None]


def save(d, name, col, h_m, rough, ppm, exag=2.0, metal=None, r_max=512):
    """Write <name>.png, <name>_n.png, <name>_r.png, <name>_m.png from z-up
    arrays (height in metres). / z 上向きの配列から各マップを書く。"""
    col = np.flipud(col)
    h_m = np.flipud(h_m)
    rough = np.flipud(rough)
    Image.fromarray(np.clip(col, 0, 255).astype(np.uint8)).save(os.path.join(d, name + ".png"))
    # slope from the height: textures.normal_map takes the central difference
    # times strength, so pass h / (2 px) / ... / 高さ (m) から勾配
    normal_map(h_m * ppm / 2.0 * exag, 1.0).save(os.path.join(d, name + "_n.png"))
    r = np.clip(rough, 0.02, 1.0)
    hh, ww = r.shape
    s = max(1, int(math.ceil(max(hh, ww) / r_max)))
    if s > 1:
        r = np.asarray(Image.fromarray(r.astype(np.float32)).resize((ww // s, hh // s), Image.BILINEAR))
    Image.fromarray((srgb_encode(r) * 255).astype(np.uint8)).save(os.path.join(d, name + "_r.png"))
    if metal is None:
        mimg = np.full((8, 8), MATERIALS[name][3])
    else:
        mimg = np.flipud(metal)
        if s > 1:
            mimg = np.asarray(Image.fromarray(mimg.astype(np.float32)).resize((ww // s, hh // s), Image.BILINEAR))
    Image.fromarray((srgb_encode(mimg) * 255).astype(np.uint8)).save(os.path.join(d, name + "_m.png"))


# ---------------------------------------------------------------- walls
SILLS = [1.55, 2.2, 4.35, 5.05, 6.15]     # sill / opening heights of 2-storey houses and apaato (m) / 窓台の高さ


def wall(c, pattern, rng, W=512, H=1536, ppm=512 / 3.0):
    """Ceramic siding (board / stone / tile / vboard) or render on a 3 x 9 m
    tile: panels 0.45 m high with staggered caulked joints, weathering.
    窯業系サイディング（板・石・タイル・縦張り）または吹付け、汚れ付き。"""
    X, Z = grid(H, W, ppm)
    h = np.zeros((H, W))
    tone = 0.025 * sm(H, W, 0.8 * ppm, rng) + 0.012 * sm(H, W, 3.0, rng)
    grain = sm(H, W, 0.7, rng)
    rough = np.full((H, W), 0.78)
    if pattern != "_render":
        row = np.floor(Z / 0.45).astype(int)
        # caulked vertical joints, staggered per panel row (period 3 m) / 縦目地（段ごとにずらす）
        off = (row * 1.37) % 3.0
        dv = (X - off) % 3.0
        dv = np.minimum(dv, 3.0 - dv)
        jv = groove(dv, 0.006)
        jh = groove(dist_to_lines(Z, 0.45), 0.005)
        h -= 0.004 * np.maximum(jv, jh)
        tone -= 0.06 * np.maximum(jv, jh)
        rough += 0.08 * np.maximum(jv, jh)
    if pattern == "":
        # two boards per panel, V grooves, a faint wood-like grain / 板柄（V 溝、木目調）
        g = groove(dist_to_lines(Z, 0.225), 0.008)
        h -= 0.003 * g
        tone -= 0.05 * g
        h += 0.0008 * sm(H, W, 1.0, rng, sx=18.0)
        tone += 0.012 * sm(H, W, 1.2, rng, sx=25.0)
        board = np.floor(Z / 0.225).astype(int)
        tone += 0.015 * np.sin(board * 2.3 + 1.0)
    elif pattern == "_vboard":
        # vertical boards / ribs every 0.2 m (modern dark houses) / 縦張り
        g = groove(dist_to_lines(X, 0.2), 0.01)
        h -= 0.005 * g
        tone -= 0.08 * g
        rough -= 0.08
    elif pattern == "_stone":
        # cut stone: three courses per panel, stones 0.2-0.6 m, bevelled / 石積み調
        crs = np.floor(Z / 0.15).astype(int)
        lens = np.array([rng.uniform(0.2, 0.6) for _ in range(64)])
        cuts = np.cumsum(lens)
        ncrs = int(H / ppm / 0.15) + 1
        offs = np.array([rng.uniform(0, 3.0) for _ in range(ncrs)])
        xs = (X + offs[np.clip(crs, 0, ncrs - 1)]) % 3.0
        cuts = cuts[cuts < 3.0]
        cuts = np.r_[0.0, cuts, 3.0]
        idx = np.searchsorted(cuts, xs) - 1
        dx = np.minimum(xs - cuts[idx], cuts[idx + 1] - xs)
        dz = np.abs(dist_to_lines(Z, 0.15))
        e = np.minimum(dx, dz)
        bev = np.clip(e / 0.012, 0, 1)
        h += 0.008 * bev + 0.002 * sm(H, W, 2.0, rng)
        sid = (idx * 7 + crs * 13) % 17
        tone += 0.06 * (sid / 16.0 - 0.5) - 0.12 * (1 - bev)
        rough += 0.06 * (1 - bev)
    elif pattern == "_tile":
        # brick tiles 0.24 x 0.065, 1 cm joints / タイル調
        crs = np.floor(Z / 0.075).astype(int)
        xs = (X + (crs % 2) * 0.125) % 0.25
        dx = np.minimum(xs, 0.25 - xs)
        dz = np.abs(dist_to_lines(Z, 0.075))
        j = np.clip(1 - np.minimum(dx, dz) / 0.006, 0, 1)
        h -= 0.003 * j
        bid = (np.floor((X + (crs % 2) * 0.125) / 0.25).astype(int) * 31 + crs * 17) % 23
        tone += 0.07 * (bid / 22.0 - 0.5) - 0.15 * j
        rough += 0.05 * j
    else:
        # render (吹付け): stipple, mottling, a few hairline cracks / 吹付け
        h += 0.0015 * grain + 0.0008 * sm(H, W, 0.5, rng)
        tone += 0.02 * sm(H, W, 0.3 * ppm, rng)
        rough += 0.1
    col = colour_field(c, tone + 0.02 * grain)
    if pattern == "_twotone":
        # ready-built two-tone: brick-pattern siding on the ground floor, a
        # trim at TWOTONE_Z (the estate samples) / 1 階はレンガ調、境に見切り
        lo, hlo, rlo, _ = wall(PALETTE["twotone_brick"], "_tile", rng, W, H, ppm)
        low = Z < TWOTONE_Z
        col[low] = lo[low]
        h[low] = hlo[low]
        rough[low] = rlo[low]
        trim = np.abs(Z - TWOTONE_Z) < 0.03
        col[trim] = (236, 234, 228)
        h[trim] += 0.006
    # weathering: base band, streaks under sills, a touch under the eaves
    # 汚れ: 足元の帯、窓台の下の雨だれ
    heavy = pattern == "_render"
    band = base_band(H, W, ppm, rng, 0.15, 0.9, 0.13 if heavy else 0.09)
    st = streaks(H, W, ppm, rng, 90 if heavy else 45, SILLS, amount=(0.08, 0.24) if heavy else (0.04, 0.12))
    col = stain(col, np.clip(band + st, 0, 0.6), (92, 86, 76))
    rough += 0.05 * band
    return col, h, rough, ppm


def concrete_render(c, rng, W=512, H=2048, ppm=128.0, floors=(0.5, 2.75)):
    """Danchi render: stipple, floor joints every 2.75 m, streaks under every
    slab, a stained foot. / 団地の吹付け（打継ぎ目地・雨だれ）。"""
    X, Z = grid(H, W, ppm)
    tone = 0.03 * sm(H, W, 0.6 * ppm, rng) + 0.02 * sm(H, W, 2.0, rng)
    h = 0.0015 * sm(H, W, 0.8, rng)
    j = groove(dist_to_lines(Z, floors[1], floors[0]), 0.012)
    h -= 0.006 * j
    tone -= 0.08 * j
    col = colour_field(c, tone)
    starts = [floors[0] + floors[1] * k - 0.02 for k in range(1, 7)]
    st = streaks(H, W, ppm, rng, 140, starts, length=(0.5, 2.4), width=(0.02, 0.15), amount=(0.06, 0.22))
    band = base_band(H, W, ppm, rng, 0.15, 1.1, 0.16)
    col = stain(col, np.clip(st + band, 0, 0.6), (96, 92, 84))
    rough = 0.86 + 0.04 * j + 0.03 * band
    return col, h, rough, ppm


def mosaic(c, rng, W=768, H=1536, ppm=256.0):
    """45 mm mosaic tiles (5 mm joints) in three shades, faint streaks under
    the 3 m floors. / 45 二丁モザイクタイル（3 色混ぜ）。"""
    X, Z = grid(H, W, ppm)
    p = 0.05
    ix, iz = np.floor(X / p).astype(int), np.floor(Z / p).astype(int)
    dx, dz = np.abs((X % p) - p / 2), np.abs((Z % p) - p / 2)
    j = np.clip((np.maximum(dx, dz) - (p / 2 - 0.004)) / 0.002, 0, 1)
    nx, nz = int(round(W / ppm / p)), int(round(H / ppm / p))
    shade = np.array([rng.choice([-0.07, 0.0, 0.0, 0.05, 0.1]) + rng.normal(0, 0.015) for _ in range(nx * nz)])
    s = shade[(iz % nz) * nx + (ix % nx)]
    tone = s - 0.16 * j + 0.01 * sm(H, W, 0.5 * ppm, rng)
    col = colour_field(c, tone)
    starts = [0.55 + 3.0 * k - 0.2 for k in range(1, 3)]
    st = streaks(H, W, ppm, rng, 40, starts, length=(0.3, 1.2), amount=(0.04, 0.12))
    col = stain(col, st, (90, 86, 80))
    h = -0.003 * j + 0.0006 * sm(H, W, 1.0, rng)
    rough = 0.32 + 0.45 * j + 0.05 * st
    return col, h, rough, ppm


# ---------------------------------------------------------------- roofs
def kawara(c, rng, glazed, W=1024, ppm=256.0, lichen=0.0):
    """J-shaped kawara (sangawara): 13 waves per 4 m across the slope,
    0.25 m courses, each tile's lip casting a shadow on the course below,
    per-tile colour (ibushi varies most). / 桟瓦（波・段・軒先の影・1 枚ごとの色むら）。"""
    H = W
    X, Z = grid(H, W, ppm)
    pw, pc = 4.0 / 13, 0.25
    ph = X / pw * 2 * np.pi
    wave = 0.5 + 0.5 * np.cos(ph)              # 1 at the crest / 山で 1
    t = (Z % pc) / pc                           # 0 at the lip (down-slope) .. 1 / 軒先側 0
    h = 0.03 * wave + 0.012 * (1 - t)
    shadow = np.clip((t - 0.86) / 0.14, 0, 1) ** 2     # under the next lip / 上段の重なりの影
    h -= 0.01 * shadow
    ix, iz = np.floor(X / pw).astype(int), np.floor(Z / pc).astype(int)
    n = 13 * 16
    var = 0.035 if glazed else 0.08
    tv = np.array([rng.normal(0, var) for _ in range(n)])
    tone = tv[(iz % 16) * 13 + (ix % 13)] + 0.18 * (wave - 0.5) - 0.35 * shadow
    tone += (0.015 if glazed else 0.04) * sm(H, W, 1.5, rng)
    col = colour_field(c, tone)
    rough = np.full((H, W), 0.3 if glazed else 0.55) + 0.15 * shadow
    if glazed:
        rough -= 0.08 * wave
    # weathering: lichen spots and dark streaks down the slope / 地衣類・黒い筋
    if lichen > 0:
        sp = np.clip(sm(H, W, 3.0, rng) - 0.55, 0, 1) * 2.5 * np.clip(sm(H, W, 40.0, rng) + 0.3, 0, 1)
        col = stain(col, np.clip(lichen * sp, 0, 0.6), (150, 152, 128))
    dk = np.clip(sm(H, W, 30.0, rng, sx=3.0) - 0.4, 0, 1) * (1 - wave)
    col = stain(col, 0.35 * dk, (40, 40, 38))
    rough += 0.1 * dk
    return col, h, rough, ppm


def slate(c, rng, W=1024, ppm=256.0):
    """Colonial slate: 1.0 m shingles, 0.182 m exposure, half-staggered, cut
    slits, faded patches, moss at the lower edges. / コロニアル（化粧スレート）。"""
    H = W
    X, Z = grid(H, W, ppm)
    pc = 4.0 / 22
    crs = np.floor(Z / pc).astype(int)
    xs = (X + (crs % 2) * 0.5) % 1.0
    t = (Z % pc) / pc
    slit = groove(np.minimum(xs, 1.0 - xs), 0.004)
    lip = np.clip((0.12 - t) / 0.12, 0, 1)
    h = 0.006 * t - 0.004 * slit
    sid = (np.floor((X + (crs % 2) * 0.5)).astype(int) * 7 + crs * 5) % 11
    tone = 0.05 * (sid / 10.0 - 0.5) - 0.25 * slit - 0.2 * lip ** 2 + 0.05 * sm(H, W, 60.0, rng)
    col = colour_field(c, tone + 0.02 * sm(H, W, 1.0, rng))
    fade = np.clip(sm(H, W, 80.0, rng), 0, 1) * 0.25
    col = stain(col, fade, (120, 118, 112))
    moss = np.clip(sm(H, W, 2.0, rng) - 0.3, 0, 1) * lip * np.clip(sm(H, W, 50.0, rng), 0, 1) * 1.5
    col = stain(col, np.clip(moss, 0, 0.5), (74, 84, 52))
    rough = 0.66 + 0.1 * slit + 0.05 * fade
    return col, h, rough, ppm


def metal_roof(c, rng, kind, W=1024, ppm=256.0):
    """Standing seam (seams down the slope every 0.444 m), galvalume, or
    corrugated siding (76 mm waves). / 縦葺き鋼板・ガルバリウム・波板。"""
    H = W
    X, Z = grid(H, W, ppm)
    if kind == "corrugated":
        p = 4.0 / 52
        h = 0.009 * np.cos(X / p * 2 * np.pi)
        tone = 0.06 * np.cos(X / p * 2 * np.pi)
        seam = np.zeros_like(X)
    else:
        p = 4.0 / 9
        d = np.abs(dist_to_lines(X, p))
        seam = groove(d, 0.012)
        h = 0.025 * seam + 0.0015 * sm(H, W, 0.15 * ppm, rng, sx=0.05 * ppm)       # oil-canning / ゆがみ
        tone = 0.12 * seam - 0.08 * groove(d - 0.016, 0.006)
    tone = tone + 0.025 * sm(H, W, 0.5 * ppm, rng)
    if kind == "galvalume":
        tone += 0.05 * sm(H, W, 6.0, rng)          # spangle / スパングル
    col = colour_field(c, tone)
    st = streaks(H, W, ppm, rng, 30, [], length=(0.8, 3.0), width=(0.03, 0.2), amount=(0.05, 0.15))
    col = stain(col, st, (70, 66, 60) if kind != "corrugated" else (120, 84, 56))
    rough = np.full((H, W), 0.4) + 0.1 * st
    return col, h, rough, ppm


# ---------------------------------------------------------------- walls on the lot
def voronoi_stones(H, W, ppm, rng, size, jitter=0.35):
    """Irregular stones: wrapped Voronoi cells (centres on a jittered grid),
    returns (edge distance m, cell id). / 不定形の石（タイル可能なボロノイ）。"""
    nx, nz = max(2, int(round(W / ppm / size))), max(2, int(round(H / ppm / (size * 0.75))))
    cx = ((np.arange(nx)[None, :] + 0.5 + rng.uniform(-jitter, jitter, (nz, nx))) * W / nx).ravel()
    cz = ((np.arange(nz)[:, None] + 0.5 + rng.uniform(-jitter, jitter, (nz, nx))) * H / nz).ravel()
    zz, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    wob = sm(H, W, 6.0, rng).astype(np.float32) * (0.06 * ppm)
    xx += wob
    zz += np.roll(wob, H // 3, 0)
    d1 = np.full((H, W), 1e9, np.float32)
    d2 = np.full((H, W), 1e9, np.float32)
    idd = np.zeros((H, W), np.int32)
    for k, (px, pz) in enumerate(zip(cx, cz)):
        dx = np.abs(xx - px)
        dz = np.abs(zz - pz)
        dx = np.minimum(dx, W - dx)
        dz = np.minimum(dz, H - dz) * 1.25
        d = np.sqrt(dx * dx + dz * dz)
        closer = d < d1
        d2 = np.where(closer, d1, np.minimum(d2, d))
        idd = np.where(closer, k, idd)
        d1 = np.where(closer, d, d1)
    return (d2 - d1) / 2.0 / ppm, idd


def ishigaki(c, rng, W=1024, H=2048, ppm=256.0):
    """Battered stone wall: irregular stones ~0.45 m, deep dark joints, moss
    in the joints and on the lowest stones, stains. / 石積み（ごつごつした石、暗い目地、苔）。"""
    X, Z = grid(H, W, ppm)
    e, idd = voronoi_stones(H, W, ppm, rng, 0.45)
    face = np.clip(e / 0.03, 0, 1)
    dome = np.sqrt(np.clip(e / 0.12, 0, 1))
    h = 0.05 * dome + 0.006 * sm(H, W, 3.0, rng) + 0.003 * sm(H, W, 1.0, rng)
    sv = np.array([rng.normal(0, 0.09) for _ in range(idd.max() + 1)])
    hue = np.array([rng.uniform(-1, 1) for _ in range(idd.max() + 1)])
    tone = sv[idd] - 0.55 * (1 - face) + 0.05 * sm(H, W, 2.0, rng) + 0.12 * (dome - 0.7)
    col = colour_field(c, tone)
    col[..., 0] *= 1 + 0.04 * hue[idd]
    col[..., 2] *= 1 - 0.04 * hue[idd]
    # moss: in the joints everywhere a little, on the stones near the foot / 苔: 目地と足元
    foot = np.clip((1.2 - Z) / 1.0, 0, 1) ** 1.2
    blot = np.clip(sm(H, W, 0.25 * ppm, rng) + 0.2, 0, 1)
    moss = np.clip(0.5 * (1 - face) * blot + 0.65 * foot * blot * (0.4 + 0.6 * (1 - dome)), 0, 0.8)
    col = stain(col, moss, (62, 78, 40))
    st = streaks(H, W, ppm, rng, 40, [], length=(0.5, 2.5), width=(0.05, 0.3), amount=(0.08, 0.25))
    col = stain(col, st, (58, 56, 50))
    rough = 0.82 + 0.1 * (1 - face) + 0.05 * moss
    return col, h, rough, ppm


def retaining(c, rng, W=512, H=1024, ppm=128.0):
    """Cast-in-place retaining wall: formwork panels 0.9 x 1.8 m, tie-hole
    plugs on a 0.6 x 0.45 m grid, efflorescence, a dark mossy foot.
    現場打ち擁壁（型枠の継ぎ目、セパ穴、白華、苔の足元）。"""
    X, Z = grid(H, W, ppm)
    tone = 0.04 * sm(H, W, 0.5 * ppm, rng) + 0.02 * sm(H, W, 2.0, rng)
    pj = np.maximum(groove(dist_to_lines(X, 1.0), 0.004), groove(dist_to_lines(Z, 2.0), 0.004))
    # panel tone differences / 型枠ごとの色の違い
    pid = (np.floor(X / 1.0).astype(int) * 3 + np.floor(Z / 2.0).astype(int) * 5) % 7
    tone += 0.03 * (pid / 6.0 - 0.5) - 0.05 * pj
    hx = np.abs(((X - 0.25) % 0.5) - 0.25)
    hz = np.abs(((Z - 0.3) % 0.5) - 0.25)
    hole = np.clip(1 - np.sqrt(hx ** 2 + hz ** 2) / 0.016, 0, 1)
    tone -= 0.18 * (hole > 0)
    h = 0.002 * sm(H, W, 1.0, rng) - 0.002 * pj - 0.004 * (hole > 0)
    col = colour_field(c, tone)
    eff = streaks(H, W, ppm, rng, 25, [], length=(0.5, 2.0), width=(0.03, 0.15), amount=(0.1, 0.3))
    col = stain(col, eff, (214, 212, 204))
    rust = np.zeros((H, W))
    for zz in np.arange(0.3, H / ppm, 0.5):
        rust += streaks(H, W, ppm, rng, 3, [zz], length=(0.05, 0.25), width=(0.004, 0.01), amount=(0.2, 0.4))
    dk = streaks(H, W, ppm, rng, 60, [], length=(0.6, 3.0), width=(0.03, 0.2), amount=(0.08, 0.25))
    col = stain(col, np.clip(rust, 0, 0.5) * 0.6, (128, 92, 66))
    col = stain(col, dk, (70, 68, 62))
    foot = base_band(H, W, ppm, rng, 0.0, 0.9, 0.45) * np.clip(0.5 + sm(H, W, 0.2 * ppm, rng), 0, 1)
    col = stain(col, foot, (64, 70, 48))
    rough = 0.86 + 0.04 * foot + 0.03 * pj
    return col, h, rough, ppm


def kenchi(c, rng, W=1024, H=2048, ppm=256.0):
    """Kenchi-block retaining wall (Street View, Akeno lanes): 0.35 m square
    blocks laid diagonally, bevelled faces, PVC weep holes every ~2 m, moss
    at the foot. / 間知ブロック積み（斜めの格子、水抜き穴）。"""
    X, Z = grid(H, W, ppm)
    s = 0.35 * math.sqrt(2.0)
    a, b = (X + Z) / s, (X - Z) / s
    # tile so the diagonal grid wraps on the 4 x 8 m tile: snap s / 斜め格子がタイルするように
    n = round(4.0 / s)
    s = 4.0 / n
    a, b = (X + Z) / s, (X - Z) / s
    da, db = np.abs(a - np.round(a)), np.abs(b - np.round(b))
    e = np.minimum(da, db) * s / math.sqrt(2)
    # a 4 cm bevel round a rough split face; the bevel catches less light, so
    # the blocks read as blocks, not as a thin lattice / 面取りと割肌
    face = np.clip((e - 0.006) / 0.04, 0, 1)
    dome = np.clip(e / 0.09, 0, 1)
    h = 0.035 * face + 0.01 * dome + 0.004 * sm(H, W, 1.5, rng) + 0.002 * sm(H, W, 4.0, rng)
    bid = (np.floor(a).astype(int) * 7 + np.floor(b).astype(int) * 11) % 13
    tone = (0.12 * (bid / 12.0 - 0.5) - 0.32 * (1 - face) ** 1.5 + 0.07 * sm(H, W, 2.5, rng)
            + 0.04 * sm(H, W, 0.3 * ppm, rng))
    col = colour_field(c, tone)
    # weep holes (75 mm PVC) at 0.9 m and every 2 m along / 水抜き穴
    hole = np.zeros((H, W))
    for zz in (0.9, 2.9, 4.9, 6.9):
        for xx in np.arange(0.5 + (zz % 2.0) * 0.5, 4.0, 2.0):
            dx = np.minimum(np.abs(X - xx), 4.0 - np.abs(X - xx))
            hole = np.maximum(hole, np.clip(1 - np.sqrt(dx ** 2 + (Z - zz) ** 2) / 0.045, 0, 1))
    ring = (hole > 0) & (hole < 0.35)
    col[hole >= 0.35] = (34, 34, 34)
    col[ring] = (190, 190, 186)
    h -= 0.04 * (hole >= 0.35)
    under = streaks(H, W, ppm, rng, 20, [0.9 - 0.05, 2.9 - 0.05, 4.85], length=(0.3, 1.0),
                    width=(0.02, 0.05), amount=(0.3, 0.6))
    dk = streaks(H, W, ppm, rng, 40, [], length=(0.5, 2.5), width=(0.05, 0.3), amount=(0.08, 0.22))
    col = stain(col, np.clip(under + dk, 0, 0.6), (64, 62, 56))
    foot = base_band(H, W, ppm, rng, 0.0, 0.8, 0.45) * np.clip(0.5 + sm(H, W, 0.2 * ppm, rng), 0, 1)
    col = stain(col, foot, (64, 72, 46))
    rough = 0.84 + 0.08 * (1 - face)
    return col, h, rough, ppm


def block_wall(c, rng, W=1024, H=512, ppm=256.0, bw=0.4, bh=0.2):
    """Concrete block wall (CB): 390 x 190 blocks with 10 mm joints from the
    terrace (z 0.15), running bond, a smooth cap course at 1.15-1.35 m,
    efflorescence and a dirty foot. / CB 塀（390x190、笠木、白華）。"""
    X, Z = grid(H, W, ppm)
    z0 = 0.15
    crs = np.floor((Z - z0) / bh).astype(int)
    xs = (X + (crs % 2) * bw / 2) % bw
    dx = np.minimum(xs, bw - xs)
    dz = np.abs(((Z - z0) % bh) - bh / 2)
    j = np.maximum(groove(dx, 0.006), groove(bh / 2 - dz, 0.006))
    cap = (Z > 1.15) & (Z < 1.35)
    j[cap] = groove(np.minimum(np.abs(Z - 1.15), np.abs(Z - 1.35)), 0.006)[cap]
    bid = (np.floor((X + (crs % 2) * bw / 2) / bw).astype(int) * 13 + crs * 7) % 11
    tone = 0.05 * (bid / 10.0 - 0.5) - 0.22 * j + 0.03 * sm(H, W, 1.2, rng) + 0.03 * sm(H, W, 0.4 * ppm, rng)
    tone[cap] += 0.06
    h = -0.005 * j + 0.0015 * sm(H, W, 1.0, rng)
    h[cap] += 0.002
    col = colour_field(c, tone)
    eff = streaks(H, W, ppm, rng, 30, [1.15, 0.95, 0.75], length=(0.1, 0.6), width=(0.01, 0.08), amount=(0.1, 0.3))
    col = stain(col, eff, (210, 208, 200))
    foot = base_band(H, W, ppm, rng, 0.15, 0.55, 0.3)
    dk = streaks(H, W, ppm, rng, 40, [1.15], length=(0.2, 1.0), width=(0.02, 0.15), amount=(0.08, 0.22))
    col = stain(col, np.clip(foot + dk, 0, 0.6), (70, 70, 62))
    rough = 0.88 + 0.06 * j - 0.05 * cap
    return col, h, rough, ppm


def plain_concrete(c, rng, W=1024, H=1024, ppm=256.0, grooves=None, splash=None, tyre=False):
    """Trowelled concrete (foundation, terrace, flat roof): mottling, pores,
    optional grooves every ``grooves`` m and a dirty foot. / 金ごて仕上げのコンクリート。"""
    X, Z = grid(H, W, ppm)
    tone = 0.04 * sm(H, W, 0.4 * ppm, rng) + 0.025 * sm(H, W, 2.0, rng)
    pore = (rng.random((H, W)) < 0.004)
    tone -= 0.2 * pore
    h = 0.001 * sm(H, W, 1.0, rng) - 0.002 * pore
    rough = np.full((H, W), 0.86)
    if grooves:
        j = np.maximum(groove(dist_to_lines(X, grooves), 0.008), groove(dist_to_lines(Z, grooves), 0.008))
        tone -= 0.3 * j
        h -= 0.006 * j
    col = colour_field(c, tone)
    blot = np.clip(sm(H, W, 0.3 * ppm, rng) - 0.25, 0, 1)
    col = stain(col, 0.25 * blot, (96, 94, 88))
    if splash:
        col = stain(col, base_band(H, W, ppm, rng, splash[0], splash[1], 0.35), (88, 80, 70))
    if tyre:
        t = np.clip(sm(H, W, 12.0, rng, sx=0.6 * ppm) - 0.4, 0, 1)
        col = stain(col, 0.4 * t, (60, 58, 56))
    return col, h, rough, ppm


def painted_steel(c, rng, W=512, ppm=256.0, rust=0.5, ribs=None, bars=None):
    """Painted steel (stairs, railings, meter boxes): slight orange-peel,
    chipped paint with rust spots and rust streaks; ribs / bars vertical.
    塗装鋼板（錆の点と筋）。"""
    H = W
    X, Z = grid(H, W, ppm)
    h = 0.0004 * sm(H, W, 1.5, rng)
    tone = 0.02 * sm(H, W, 0.3 * ppm, rng)
    if ribs:
        g = 0.5 + 0.5 * np.cos(X / ribs * 2 * np.pi)
        h += 0.01 * g
        tone += 0.1 * (g - 0.5)
    if bars:
        d = np.abs(dist_to_lines(X, bars))
        bar = (d < 0.012).astype(np.float64)
        h += 0.02 * bar
        tone += -0.45 * (1 - bar) + 0.05 * bar
    col = colour_field(c, tone)
    spots = np.clip(sm(H, W, 1.5, rng) - (0.85 - 0.2 * rust), 0, 1) * 5.0
    st = streaks(H, W, ppm, rng, int(20 * rust), [], length=(0.1, 0.6), width=(0.005, 0.03), amount=(0.2, 0.5))
    rm = np.clip(spots + st * rust, 0, 0.8) * rust
    col = stain(col, rm, (132, 76, 44))
    rough = 0.45 + 0.35 * rm
    return col, h, rough, ppm


def alu(c, rng, W=256, ppm=256.0):
    """Anodised aluminium, brushed horizontally. / アルミ（ヘアライン）。"""
    H = W
    tone = 0.04 * sm(H, W, 0.5, rng, sx=40.0) + 0.02 * sm(H, W, 8.0, rng)
    col = colour_field(c, tone)
    return col, 0.0002 * sm(H, W, 0.5, rng, sx=30.0), 0.32 + 0.05 * sm(H, W, 4.0, rng), ppm


def plain(c, rng, W=128, ppm=128.0, var=0.02, rough=0.6, grad=0.0):
    H = W
    _, Z = grid(H, W, ppm)
    tone = var * sm(H, W, 0.25 * W, rng) + 0.01 * sm(H, W, 1.0, rng) + grad * (Z / Z.max() - 0.5)
    return colour_field(c, tone), 0.0003 * sm(H, W, 1.0, rng), np.full((H, W), rough), ppm


def soffit(c, rng, W=512, ppm=256.0):
    """Fibre-cement soffit boards: joints every 1 m, a perforated vent strip.
    軒天ボード（継ぎ目、有孔の換気帯）。"""
    H = W
    X, Z = grid(H, W, ppm)
    j = groove(dist_to_lines(X, 1.0), 0.004)
    hx, hz = (X % 0.025) - 0.0125, (Z % 0.025) - 0.0125
    perf = ((np.abs(Z % 1.0 - 0.15) < 0.06) & (hx ** 2 + hz ** 2 < 0.004 ** 2)).astype(np.float64)
    tone = -0.15 * j - 0.25 * perf + 0.02 * sm(H, W, 0.3 * ppm, rng)
    col = stain(colour_field(c, tone), 0.1 * np.clip(sm(H, W, 0.4 * ppm, rng), 0, 1))
    return col, -0.002 * j - 0.003 * perf, 0.85 + 0.0 * j, ppm


def ridge(c, rng, W=256, ppm=256.0):
    """Noshi ridge tiles: stacked 5 cm layers. / のし瓦の段。"""
    H = W
    X, Z = grid(H, W, ppm)
    t = (Z % 0.05) / 0.05
    tone = 0.1 * t - 0.25 * (t < 0.12) + 0.04 * sm(H, W, 3.0, rng)
    return colour_field(c, tone), 0.005 * t, 0.5 + 0.1 * (t < 0.12), ppm


def hedge(rng, W=512, ppm=256.0):
    """A clipped hedge: dense leaves with relief. / 生垣（刈り込み）。"""
    H = W
    leaf = sm(H, W, 2.0, rng) + 0.5 * sm(H, W, 6.0, rng)
    hole = np.clip(-leaf - 0.6, 0, 1)
    c0, c1 = np.array((34, 66, 30.0)), np.array((92, 132, 58.0))
    t = np.clip(0.5 + 0.5 * leaf, 0, 1)[..., None]
    col = c0 * (1 - t) + c1 * t
    col *= (1 - 0.7 * hole)[..., None]
    return col, 0.03 * leaf, 0.85 + 0.0 * leaf, ppm


def kanban(c, rng, ink, W=512, ppm=128.0):
    """A shop's sign band (4 x 4 m tile, the band at z 3.1-3.75 m as
    buildings_jp draws it): panel colour, a pale edge strip, abstract
    blocky lettering (not real words). / 看板帯（抽象的な文字形）。"""
    H = W
    im = Image.new("RGB", (W, H), tuple(int(v) for v in c))
    dr = ImageDraw.Draw(im)
    z2y = lambda z: H - 1 - int(z * ppm)
    y0, y1 = z2y(3.68), z2y(3.17)
    dr.rectangle([0, z2y(3.75), W, z2y(3.70)], fill=tuple(int(min(255, v * 1.15 + 20)) for v in c))
    dr.rectangle([0, z2y(3.14), W, z2y(3.10)], fill=tuple(int(v * 0.7) for v in c))
    x = int(0.3 * ppm)
    cell = int(0.42 * ppm)
    while x < W - cell:
        if rng.random() < 0.15:
            x += cell // 2
            continue
        g = cell - 6
        # a glyph: a few strokes in a square cell / 字形: 正方形内の数画
        for _ in range(rng.randint(3, 6)):
            if rng.random() < 0.5:
                yy = rng.randint(y0, y1 - 4)
                xa = x + rng.randint(0, g // 2)
                dr.rectangle([xa, yy, xa + rng.randint(g // 3, g), yy + 4], fill=ink)
            else:
                xx = x + rng.randint(0, g - 4)
                ya = rng.randint(y0, (y0 + y1) // 2)
                dr.rectangle([xx, ya, xx + 4, ya + rng.randint((y1 - y0) // 3, y1 - y0) // 1], fill=ink)
        x += cell
    col = np.flipud(np.asarray(im, np.float64)).copy()
    nrng = np.random.default_rng(rng.randint(0, 1 << 30))
    col *= (1 + 0.02 * sm(H, W, 6.0, nrng))[..., None]
    ink_m = (np.abs(col - np.array(ink, np.float64)).sum(-1) < 40).astype(np.float64)
    return col, 0.001 * ink_m, 0.4 - 0.05 * ink_m + 0.0 * col[..., 0], ppm


def awning(c, rng, striped, W=256, ppm=128.0):
    """Canvas awning: weave, vertical stripes (down the slope) if striped,
    dirt along the stripes. / テント地（織り目、縦縞）。"""
    H = W
    X, Z = grid(H, W, ppm)
    col = colour_field(c, 0.03 * sm(H, W, 0.5, rng) + 0.03 * sm(H, W, 0.3 * ppm, rng))
    if striped:
        s = ((X / 0.25).astype(int) % 2 == 1)
        col[s] = (232, 228, 218)
    st = streaks(H, W, ppm, rng, 15, [], length=(0.3, 1.5), width=(0.02, 0.1), amount=(0.05, 0.15))
    col = stain(col, st, (70, 64, 58))
    return col, 0.0008 * sm(H, W, 0.6, rng), 0.75 + 0.0 * X, ppm


def shopfront(rng, W=576, H=480, ppm=160.0):
    """Shopfront glazing (3.6 x 3.0 m tile, v = height): kick plate, aluminium
    mullions every 1.2 m and a transom, a dim interior with shelves and
    posters, the sky's reflection. / 店舗のガラス面（框・方立・店内・映り込み）。"""
    X, Z = grid(H, W, ppm)
    t = np.clip((Z - 0.45) / 2.4, 0, 1)
    col = np.zeros((H, W, 3))
    sky = np.array((150, 168, 182.0))
    inside = np.array((70, 72, 74.0))
    col[:] = inside * (1 - 0.6 * t[..., None]) + sky * 0.0
    # shelves and goods / 棚と商品
    shelf = ((Z > 0.5) & (Z < 1.7))
    goods = sm(H, W, 3.0, rng, sx=6.0)
    gcol = np.stack([150 + 60 * goods, 120 + 50 * np.roll(goods, 37, 1), 100 + 60 * np.roll(goods, 91, 0)], -1)
    col[shelf] = gcol[shelf] * 0.6
    for zz in (0.5, 0.9, 1.3, 1.7):
        col[np.abs(Z - zz) < 0.015] = (190, 190, 186)
    # posters / ポスター
    for k in range(3):
        x0 = rng.uniform(0.1, 3.2)
        z0 = rng.uniform(1.75, 2.0)
        m = (X > x0) & (X < x0 + 0.35) & (Z > z0) & (Z < z0 + 0.5)
        col[m] = rng.choice([(220, 60, 50), (240, 210, 60), (240, 240, 236), (60, 120, 200)])
    refl = (0.25 + 0.35 * t)[..., None]
    col = col * (1 - refl) + sky[None, None] * refl
    frame = np.zeros((H, W), bool)
    frame |= np.abs(dist_to_lines(X, 1.2)) < 0.03
    frame |= (np.abs(Z - 2.35) < 0.03) | (np.abs(Z - 0.45) < 0.03) | (Z > 2.82)
    kick = Z < 0.45
    col[frame] = (178, 180, 182)
    col[kick] = (120, 122, 124)
    met = (frame | kick).astype(np.float64) * 0.7
    rough = np.where(frame | kick, 0.35, 0.05)
    return col, 0.004 * frame, rough, ppm, met


# ---------------------------------------------------------------- per-face objects
def ac_unit(rng, W=512, H=384):
    """AC outdoor unit front (0.8 x 0.6 m): fan grille on the left, louvred
    right panel, a label, grime at the foot. / エアコン室外機の正面。"""
    ppm = W / 0.8
    X, Z = grid(H, W, ppm)
    c = np.array((226, 224, 214.0))
    col = np.ones((H, W, 3)) * c
    h = np.zeros((H, W))
    cx, cz, r = 0.3, 0.3, 0.22
    d = np.sqrt((X - cx) ** 2 + (Z - cz) ** 2)
    disc = d < r
    rings = (np.abs(((d + 0.006) % 0.022) - 0.011) < 0.0025) & disc
    spokes = disc & (np.abs(np.sin(np.arctan2(Z - cz, X - cx) * 2)) < 0.05)
    col[disc] = (40, 42, 44)
    col[rings | spokes] = (200, 200, 194)
    h[disc] -= 0.02
    h[rings | spokes] += 0.012
    lou = (X > 0.58) & (X < 0.76) & (Z > 0.08) & (Z < 0.52)
    sl = lou & ((Z * 60) % 1 < 0.35)
    col[sl] = (150, 150, 146)
    h[sl] -= 0.004
    lab = (X > 0.6) & (X < 0.74) & (Z > 0.44) & (Z < 0.5)
    col[lab] = (90, 120, 170)
    col *= (1 - 0.25 * np.clip((0.12 - Z) / 0.12, 0, 1))[..., None]
    nrng = np.random.default_rng(rng.randint(0, 1 << 30))
    col = stain(col, streaks(H, W, ppm, nrng, 10, [0.6], length=(0.1, 0.4), width=(0.01, 0.04),
                             amount=(0.05, 0.12)))
    return col, h, np.where(disc, 0.7, 0.5), ppm


def door(rng, steel, W=256, H=512):
    """Front door (1.0 x 2.05 m): wood-look panel with grooves, a glass slit
    and a lever handle; or an apaato steel door with a frame, peephole, mail
    slot and number plate. / 玄関ドア（木目調／鋼製）。"""
    ppm = W / 1.0
    X, Z = grid(H, W, ppm)
    nr = np.random.default_rng(rng.randint(0, 1 << 30))
    if steel:
        c = np.array(rng.choice([(160, 150, 134), (176, 172, 164), (120, 110, 98)]), np.float64)
        col = np.ones((H, W, 3)) * c * (1 + 0.02 * sm(H, W, 20.0, nr))[..., None]
        h = 0.0005 * sm(H, W, 2.0, nr)
        fr = (X < 0.06) | (X > 0.94) | (Z > 1.97)
        col[fr] = (90, 90, 92)
        h[fr] += 0.01
        slot = (X > 0.35) & (X < 0.65) & (np.abs(Z - 1.0) < 0.025)
        col[slot] = (60, 60, 60)
        peep = (X - 0.5) ** 2 + (Z - 1.5) ** 2 < 0.012 ** 2
        col[peep] = (30, 30, 30)
        plate = (X > 0.4) & (X < 0.6) & (Z > 1.72) & (Z < 1.8)
        col[plate] = (210, 206, 190)
    else:
        c = np.array(rng.choice([(112, 82, 58), (84, 62, 46), (150, 118, 86), (60, 58, 56)]), np.float64)
        grain = sm(H, W, 1.0, nr, sx=0.3)
        col = np.ones((H, W, 3)) * c * (1 + 0.06 * grain)[..., None]
        g = groove(dist_to_lines(X, 0.12), 0.005)
        col *= (1 - 0.25 * g)[..., None]
        h = -0.003 * g
        slit = (X > 0.12) & (X < 0.2) & (Z > 0.3) & (Z < 1.8)
        col[slit] = (150, 166, 176)
        fr = (X < 0.04) | (X > 0.96) | (Z > 2.0)
        col[fr] = (70, 70, 72)
    hd = (X > 0.82) & (X < 0.9) & (np.abs(Z - 1.0) < 0.02)
    col[hd] = (200, 200, 200)
    h = h + 0.01 * hd
    col = stain(col, base_band(H, W, ppm, nr, 0.0, 0.3, 0.2))
    return col, h, np.where(hd, 0.3, 0.5 if steel else 0.55), ppm


def shutter(rng, W=512, H=512):
    """Roll-up shutter: horizontal slats (40 per face), a bottom bar with a
    handle, dirt and a little rust at the foot. / シャッター（スラット、座板）。"""
    ppm = W / 2.5
    X, Z = grid(H, W, ppm)
    nr = np.random.default_rng(rng.randint(0, 1 << 30))
    t = (np.arange(H)[:, None] / H * 40) % 1.0 + 0 * X
    prof = np.sin(t * np.pi)
    c = np.array(PALETTE["shutter"], np.float64)
    col = np.ones((H, W, 3)) * c * (1 + 0.1 * (prof - 0.6) - 0.25 * (t < 0.08))[..., None]
    h = 0.006 * prof
    bar = np.arange(H)[:, None] / H < 0.035 + 0 * X
    col[bar] = (110, 112, 114)
    hd = bar & (np.abs(X - 1.25) < 0.08)
    col[hd] = (60, 60, 60)
    col = stain(col, base_band(H, W, ppm, nr, 0.0, 0.5, 0.25), (100, 90, 76))
    st = streaks(H, W, ppm, nr, 20, [], length=(0.2, 1.2), width=(0.01, 0.05), amount=(0.05, 0.15))
    col = stain(col, st, (128, 92, 66))
    return col, h, 0.45 + 0.2 * (t < 0.08), ppm


def frosted(rng, W=256, H=512):
    """Figured (frosted) glass in an aluminium frame: a fine dimpled pattern,
    a brighter centre. / 型板ガラス（アルミ枠）。"""
    nr = np.random.default_rng(rng.randint(0, 1 << 30))
    u = (np.arange(W)[None, :] + 0.5) / W
    v = (np.arange(H)[:, None] + 0.5) / H
    dimple = sm(H, W, 1.2, nr)
    base = np.array((196, 206, 212.0))
    col = np.ones((H, W, 3)) * base * (1 + 0.05 * dimple + 0.04 * (1 - np.abs(u - 0.5) * 2) * 0)[..., None]
    col *= (0.92 + 0.08 * np.sin(np.pi * u) * np.sin(np.pi * v))[..., None]
    fr = (u < 0.04) | (u > 0.96) | (v < 0.03) | (v > 0.97)
    fr = np.broadcast_to(fr, dimple.shape)
    col[fr] = (172, 175, 178)
    met = fr.astype(np.float64) * 0.7
    return col, 0.0015 * dimple * (~fr) + 0.003 * fr, np.where(fr, 0.35, 0.2), W / 0.8, met


def window(rng, panes, variant, W=512, H=512):
    """Window glass (one texture per window): aluminium sash (silver in 'a',
    bronze in 'b') with sliding panes, the sky reflected (bright at the top,
    readable at a distance), lace or drawn curtains, a dark room.
    窓ガラス（アルミサッシ、引き違い、空の映り込み、レース・カーテン）。"""
    nr = np.random.default_rng(rng.randint(0, 1 << 30))
    u = (np.arange(W)[None, :] + 0.5) / W + 0 * np.zeros((H, 1))
    v = (np.arange(H)[:, None] + 0.5) / H + 0 * np.zeros((1, W))
    sky_top, sky_bot = np.array((176, 196, 212.0)), np.array((92, 108, 120.0))
    col = sky_bot * (1 - v[..., None]) + sky_top * v[..., None]
    room = np.array((46, 48, 52.0))
    # interior seen through: curtains / 室内: カーテン
    k = max(1, panes)
    pane = np.minimum((u * k).astype(int), k - 1)
    pu = u * k - pane
    inner = np.ones((H, W)) * 0.0
    for p in range(k):
        m = pane == p
        r = rng.random()
        if r < (0.55 if variant == "a" else 0.25):         # lace / レース
            fold = 0.5 + 0.5 * np.sin(pu * 40 + nr.uniform(0, 6))
            lace = (0.72 + 0.12 * fold)[..., None] * np.array((236, 234, 226.0))
            col = np.where(m[..., None], 0.45 * col + 0.55 * lace * 0.75, col)
        elif r < 0.8:                                       # drawn curtain / カーテン
            cc = np.array(rng.choice([(170, 150, 120), (120, 130, 150), (200, 190, 170), (110, 90, 80)]), np.float64)
            edge = rng.uniform(0.3, 0.9)
            fold = 0.8 + 0.2 * np.sin(pu * 60)
            mm = m & (pu < edge)
            col = np.where(mm[..., None], 0.4 * col + 0.6 * cc * fold[..., None] * 0.7, col)
            mm2 = m & (pu >= edge)
            col = np.where(mm2[..., None], 0.55 * col + 0.45 * room, col)
        else:                                               # a dark room / 暗い室内
            col = np.where(m[..., None], 0.55 * col + 0.45 * room, col)
    col *= (1 + 0.03 * sm(H, W, 30.0, nr))[..., None]
    # sash / サッシ
    fc = np.array((182, 184, 186.0)) if variant == "a" else np.array((70, 60, 52.0))
    fw = 0.035
    fr = (u < fw) | (u > 1 - fw) | (v < fw * 1.3) | (v > 1 - fw)
    for p in range(1, k):
        fr |= np.abs(u - p / k) < 0.022
    col[fr] = fc
    met = fr.astype(np.float64) * (0.75 if variant == "a" else 0.3)
    return col, 0.004 * fr, np.where(fr, 0.38, 0.04), W / 1.6, met


def glass_dark(rng, W=512, H=410):
    """Entrance glazing (3.0 x 2.4 m): two automatic door leaves and side
    lights in dark frames, reflections. / エントランスのガラス（自動ドア）。"""
    u = (np.arange(W)[None, :] + 0.5) / W + np.zeros((H, 1))
    v = (np.arange(H)[:, None] + 0.5) / H + np.zeros((1, W))
    col = np.array((40, 50, 60.0)) * (1 - v[..., None]) + np.array((120, 140, 156.0)) * v[..., None]
    nr = np.random.default_rng(rng.randint(0, 1 << 30))
    col *= (1 + 0.04 * sm(H, W, 20.0, nr))[..., None]
    fr = (u < 0.02) | (u > 0.98) | (v > 0.96) | (v < 0.02)
    for x in (0.25, 0.5, 0.75):
        fr |= np.abs(u - x) < 0.012
    fr |= (np.abs(v - 0.85) < 0.012)
    col[fr] = (60, 60, 62)
    return col, 0.004 * fr, np.where(fr, 0.4, 0.04), W / 3.0, fr * 0.6


# ---------------------------------------------------------------- city-centre offices
def floor_d(Z):
    """Height (m) above the storey's floor line (office module). / 階の床線からの高さ。"""
    return (Z - FLOOR_Z0) % FLOOR_M


def panel(c, rng, W=576, H=1152, ppm=160.0):
    """Precast / stone panel cladding on a 3.6 x 7.2 m tile: panels 1.8 m wide,
    horizontal joints at the floor line and 1.2 m above it, per-panel tone,
    granite speckle, dirt hanging from every joint (no foot band: the panels
    climb whole towers; the podium carries the base band). / PC・石パネル外装。"""
    X, Z = grid(H, W, ppm)
    fd = floor_d(Z)
    jv = groove(dist_to_lines(X, 1.8), 0.007)
    jh = np.maximum(groove(fd, 0.007), groove(fd - 1.2, 0.007))
    j = np.maximum(jv, jh)
    ix = np.floor(X / 1.8).astype(int)
    iz = np.floor((Z - FLOOR_Z0) / 1.2).astype(int)
    pv = np.array([rng.normal(0, 0.025) for _ in range(64)])
    tone = pv[(ix * 7 + iz * 3) % 64] + 0.02 * sm(H, W, 0.5 * ppm, rng)
    speck = rng.random((H, W))
    tone += 0.05 * (speck < 0.03) - 0.05 * (speck > 0.97) + 0.012 * sm(H, W, 0.8, rng)
    tone -= 0.18 * j
    col = colour_field(c, tone)
    starts = [FLOOR_Z0 + FLOOR_M * k + o for k in range(-1, 3) for o in (0.0, 1.2)]
    st = streaks(H, W, ppm, rng, 70, [z for z in starts if 0 < z < H / ppm], length=(0.4, 1.8),
                 width=(0.02, 0.12), amount=(0.05, 0.16))
    col = stain(col, st, (92, 88, 82))
    h = -0.005 * j + 0.0008 * sm(H, W, 1.0, rng)
    rough = 0.6 + 0.12 * j + 0.04 * st
    return col, h, rough, ppm


def curtain(c, rng, W=720, H=720, ppm=100.0):
    """Curtain-wall glass on a 7.2 x 7.2 m tile (two storeys, four 1.8 m
    panels): a 1.2 m spandrel at each floor line (opaque, darker), vision
    glass reflecting the sky with a per-panel tilt, some ceiling lights and
    lowered blinds behind, aluminium mullions and transoms (metallic).
    カーテンウォール（スパンドレル、映り込み、方立・無目）。"""
    X, Z = grid(H, W, ppm)
    fd = floor_d(Z)
    c = np.array(c, np.float64)
    sky = np.array((178, 196, 212.0))
    ix = np.floor(X / 1.8).astype(int)
    iz = np.floor((Z - FLOOR_Z0) / FLOOR_M).astype(int)
    tilt = np.array([rng.normal(0, 0.07) for _ in range(32)])[(ix * 5 + iz * 3) % 32]
    t = np.clip((fd - 0.9) / 2.7, 0, 1)
    refl = (0.32 + 0.3 * t + tilt)[..., None]
    col = c[None, None] * (1 - refl) + sky[None, None] * refl * (0.55 + 0.45 * c[None, None] / 255.0)
    # the floor behind: blinds part-way down, a band of ceiling lights / 室内: ブラインド・照明
    blind = np.array([rng.uniform(0.0, 1.0) for _ in range(32)])[(ix * 11 + iz * 7) % 32]
    bl = (fd > 3.3 - 2.4 * blind) & (fd < 3.3)
    slat = (np.abs(dist_to_lines(Z, 0.05)) < 0.01) & bl
    col[bl] = col[bl] * 0.82 + np.array((200, 200, 196.0)) * 0.18
    col[slat] *= 0.9
    lit = (np.abs(fd - 3.2) < 0.04) & (np.array([rng.random() < 0.5 for _ in range(32)])[(ix * 3 + iz) % 32])
    col[lit] = col[lit] * 0.6 + 120
    spand = (fd < 0.9) | (fd > 3.3)
    col[spand] = c * 0.62 * (1 + 0.04 * sm(H, W, 3.0, rng))[spand][..., None] + 10
    frame = (np.abs(dist_to_lines(X, 1.8)) < 0.035) | (np.abs(fd - 0.9) < 0.03) | (np.abs(fd - 3.3) < 0.03) | \
        (np.abs(dist_to_lines(fd, FLOOR_M)) < 0.03)
    col[frame] = (150, 154, 158)
    h = 0.012 * frame + 0.0 * X
    rough = np.where(frame, 0.35, np.where(spand, 0.18, 0.04))
    met = frame * 0.8 + spand * 0.1
    return col, h, rough, ppm, met


def ribbon(rng, variant, W=600, H=720, ppm=100.0):
    """Ribbon-window glass on a 6.0 x 7.2 m tile: the strip from 1.0 to 3.0 m
    above each floor line, mullions every 1.5 m, venetian blinds at random
    heights, the sky reflected; silver ('a') or bronze ('b') frames.
    水平連続窓のガラス（方立 1.5 m、ブラインド、映り込み）。"""
    X, Z = grid(H, W, ppm)
    fd = floor_d(Z)
    t = np.clip((fd - 1.0) / 2.0, 0, 1)
    nr = np.random.default_rng(rng.randint(0, 1 << 30))
    top, bot = (np.array((176, 194, 208.0)), np.array((72, 86, 98.0))) if variant == "a" else \
        (np.array((140, 150, 156.0)), np.array((46, 52, 58.0)))
    col = bot[None, None] * (1 - t[..., None]) + top[None, None] * t[..., None]
    ix = np.floor(X / 1.5).astype(int)
    iz = np.floor((Z - FLOOR_Z0) / FLOOR_M).astype(int)
    blind = np.array([rng.uniform(0.0, 1.0) for _ in range(40)])[(ix * 7 + iz * 13) % 40]
    bl = fd > 3.0 - 2.0 * blind
    col[bl] = col[bl] * 0.55 + np.array((214, 212, 204.0)) * 0.45
    col[bl & (np.abs(dist_to_lines(Z, 0.04)) < 0.008)] *= 0.88
    col *= (1 + 0.03 * sm(H, W, 20.0, nr))[..., None]
    fc = (178, 180, 182) if variant == "a" else (78, 66, 56)
    frame = (np.abs(dist_to_lines(X, 1.5)) < 0.03) | (np.abs(fd - 1.0) < 0.035) | (np.abs(fd - 3.0) < 0.035)
    col[frame] = fc
    return col, 0.006 * frame, np.where(frame, 0.35, 0.04), ppm, frame * (0.8 if variant == "a" else 0.4)


def glass_office(rng, W=320, H=288):
    """A punched office / hotel window (one texture per window): an aluminium
    frame with a centre mullion, venetian blinds part-way down, a dim floor
    with a ceiling light, the sky reflected. / 事務所の窓（ブラインド・照明）。"""
    nr = np.random.default_rng(rng.randint(0, 1 << 30))
    u = (np.arange(W)[None, :] + 0.5) / W + np.zeros((H, 1))
    v = (np.arange(H)[:, None] + 0.5) / H + np.zeros((1, W))
    col = np.array((70, 82, 92.0)) * (1 - v[..., None]) + np.array((168, 186, 200.0)) * v[..., None]
    for half in (0, 1):
        m = (u < 0.5) if half == 0 else (u >= 0.5)
        b = rng.uniform(0.2, 1.0)
        bl = m & (v > 1 - 0.85 * b)
        col[bl] = col[bl] * 0.5 + np.array((216, 214, 206.0)) * 0.5
        col[bl & ((v * 40) % 1.0 < 0.18)] *= 0.86
    col[np.abs(v - 0.9) < 0.012] += 40
    col *= (1 + 0.03 * sm(H, W, 25.0, nr))[..., None]
    fr = (u < 0.03) | (u > 0.97) | (v < 0.04) | (v > 0.96) | (np.abs(u - 0.5) < 0.012)
    col[fr] = (172, 175, 178)
    return col, 0.004 * fr, np.where(fr, 0.35, 0.04), W / 1.8, fr * 0.75


def louvre(c, rng, W=512, ppm=256.0):
    """Horizontal aluminium louvres every 0.1 m (car-park decks, rooftop plant
    screens): lit slat tops, dark gaps showing the deck behind.
    横ルーバー（立体駐車場・屋上目隠し）。"""
    H = W
    X, Z = grid(H, W, ppm)
    t = (Z % 0.1) / 0.1
    slat = t > 0.3
    tone = np.where(slat, 0.15 * (t - 0.65), -0.7) + 0.02 * sm(H, W, 0.3 * ppm, rng)
    col = colour_field(c, tone)
    col = stain(col, 0.3 * streaks(H, W, ppm, rng, 25, [], length=(0.3, 1.5), amount=(0.1, 0.3)), (96, 92, 86))
    h = 0.02 * np.sin(np.clip(t - 0.3, 0, 0.7) / 0.7 * np.pi)
    return col, h, np.where(slat, 0.4, 0.8), ppm, slat * 0.6


# ---------------------------------------------------------------- writer
def load_palette(d):
    """PALETTE updated with colour_notes.json "palette_jp" (the build dir is
    the textures dir's parent). / colour_notes.json の色で更新。"""
    p = dict(PALETTE)
    f = os.path.join(os.path.dirname(os.path.abspath(d)), "colour_notes.json")
    try:
        for k, v in json.load(open(f)).get("palette_jp", {}).items():
            if k in p and isinstance(v, (list, tuple)) and len(v) == 3:
                p[k] = tuple(int(x) for x in v)
    except (OSError, ValueError):
        pass
    return p


def write(d):
    """Every building texture, from its own random streams. / 全テクスチャを書く。"""
    P = load_palette(d)
    out = {}

    def rs(name):
        h = sum((i + 1) * ord(ch) for i, ch in enumerate(name))       # stable across runs / 実行ごとに不変
        return np.random.default_rng(5000 + h), random.Random(9000 + h)

    def put(name, res, exag=2.0):
        col, h, r, ppm = res[:4]
        met = res[4] if len(res) > 4 else None
        save(d, name, col, h, r, ppm, exag, met)
        out[name] = True

    for c, vs in WALL_VARIANTS.items():
        for v in vs:
            n = "jp_wall_%s%s" % (c, v)
            put(n, wall(P[c], v or BASE_PATTERN.get(c, ""), rs(n)[0]))
    for k in ("concrete_cream", "concrete_white"):
        put("jp_" + k, concrete_render(P[k], rs(k)[0]))
    for k in ("tile_white", "tile_beige", "tile_brown", "tile_darkbrown"):
        put("jp_" + k, mosaic(P[k], rs(k)[0]))
    put("jp_kawara_ibushi", kawara(P["kawara_ibushi"], rs("ib")[0], False, lichen=0.35))
    for k in ("black", "brown", "blue", "terracotta"):
        put("jp_kawara_" + k, kawara(P["kawara_" + k], rs(k)[0], True, lichen=0.1))
    put("jp_roof_slate", slate(P["slate"], rs("slate")[0]))
    put("jp_roof_slate_grey", slate(P["slate_grey"], rs("slateg")[0]))
    put("jp_roof_slate_brown", slate(P["slate_brown"], rs("slateb")[0]))
    put("jp_roof_metal", metal_roof(P["metal_roof"], rs("mr")[0], "seam"))
    put("jp_roof_metal_brown", metal_roof(P["metal_roof_brown"], rs("mrb")[0], "seam"))
    put("jp_roof_galvalume", metal_roof(P["galvalume"], rs("gv")[0], "galvalume"))
    put("jp_metal_siding", metal_roof(P["siding_metal"], rs("ms")[0], "corrugated"))
    put("jp_ridge", ridge(P["kawara_ibushi"], rs("ridge")[0]))
    put("jp_ridge_metal", alu((70, 70, 70), rs("ridgem")[0], 256, 256.0))
    put("jp_soffit", soffit((226, 223, 214), rs("soffit")[0]))
    put("jp_gutter", plain((200, 198, 190), rs("gutter")[0], grad=0.1, rough=0.5))
    put("jp_pipe", plain((214, 211, 202), rs("pipe")[0], grad=0.05, rough=0.55))
    put("jp_foundation", plain_concrete(P["foundation"], rs("fd")[0], 1024, 1024, 256.0, splash=(0.13, 0.45)),
        exag=1.5)
    put("jp_corridor", plain_concrete((146, 150, 146), rs("cor")[0], 512, 512, 256.0, grooves=0.5))
    put("jp_dark", plain((48, 48, 50), rs("dark")[0], 256, 128.0, var=0.08, rough=0.9))
    put("jp_number", plain((52, 66, 98), rs("num")[0], 64, 64.0))
    put("jp_lot", plain_concrete(P["lot"], rs("lot")[0], 768, 768, 128.0, grooves=3.0, tyre=True))
    put("jp_roof_flat", plain_concrete((140, 140, 136), rs("rf")[0], 768, 768, 128.0, grooves=3.0))
    put("jp_ishigaki", ishigaki(P["ishigaki"], rs("ishi")[0]), exag=1.2)
    put("jp_retaining", retaining(P["retaining"], rs("ret")[0]))
    put("jp_retaining_kenchi", kenchi(P["kenchi"], rs("ken")[0]), exag=1.2)
    put("jp_block_wall", block_wall(P["block"], rs("cb")[0]))
    put("jp_block_wall_brick", block_wall(P["brick_beige"], rs("cbb")[0], bw=0.24, bh=0.075))
    put("jp_hedge", hedge(rs("hedge")[0]), exag=0.6)
    put("jp_metal_white", painted_steel(P["metal_white"], rs("mw")[0], rust=0.6))
    put("jp_metal_dark", painted_steel(P["metal_dark"], rs("md")[0], rust=0.5))
    put("jp_alu", alu(P["alu"], rs("alu")[0]))
    put("jp_sill", alu((190, 192, 192), rs("sill")[0], 128, 128.0))
    put("jp_rail_alu", painted_steel(P["alu"], rs("ra")[0], 256, 128.0, rust=0.0, bars=0.11))
    put("jp_rail_white", painted_steel(P["metal_white"], rs("rw")[0], 256, 128.0, rust=0.5, ribs=0.1))
    put("jp_rail_dark", painted_steel((84, 78, 72), rs("rd")[0], 256, 128.0, rust=0.4, ribs=0.1))
    for k, c, ink in (("white", (238, 238, 232), (40, 60, 120)), ("red", (190, 44, 38), (250, 250, 246)),
                      ("blue", (38, 84, 158), (250, 250, 246)), ("green", (36, 118, 68), (250, 250, 246)),
                      ("yellow", (236, 196, 48), (40, 40, 40))):
        put("jp_kanban_" + k, kanban(c, rs(k + "kb")[1], ink))
    put("jp_awning_red", awning((170, 40, 38), rs("awr")[0], True))
    put("jp_awning_green", awning((46, 100, 62), rs("awg")[0], True))
    put("jp_awning_brown", awning((110, 74, 52), rs("awb")[0], False))
    put("jp_shopfront", shopfront(rs("sf")[0]))
    put("jp_ac_unit", ac_unit(rs("ac")[1]))
    put("jp_door", door(rs("door")[1], False))
    put("jp_door_steel", door(rs("doors")[1], True))
    put("jp_shutter", shutter(rs("sh")[1]))
    put("jp_glass_frosted", frosted(rs("fg")[1]))
    put("jp_glass_dark", glass_dark(rs("gd")[1]))
    put("jp_glass_office", glass_office(rs("go")[1]))
    for k in PANELS:
        put("jp_panel_" + k, panel(P["panel_" + k], rs("pn" + k)[0]), exag=1.5)
    for k in CURTAINS:
        put("jp_curtain_" + k, curtain(P["curtain_" + k], rs("cw" + k)[0]), exag=1.0)
    for v in ("a", "b"):
        put("jp_ribbon_" + v, ribbon(rs("rb" + v)[1], v), exag=1.0)
    put("jp_louvre", louvre(P["louvre"], rs("lv")[0]), exag=1.0)
    for p in (1, 2, 3):
        for v in ("a", "b"):
            put("jp_glass_%d%s" % (p, v), window(rs("w%d%s" % (p, v))[1], p, v))
    missing = sorted(set(MATERIALS) - set(out))
    if missing:
        raise RuntimeError("building materials without a texture: %s" % missing)
    return out


# ---------------------------------------------------------------- per building
def _rename(m, old, new):
    """Move material ``old``'s triangles to ``new`` (merging). / 材質名の付け替え。"""
    if old not in m.parts or old == new:
        return
    V, N, UV, I, n = m.parts.pop(old)
    for v, nn, uv, ii in zip(V, N, UV, I):
        m.tris(new, v, nn, uv, ii - (ii.min() if len(ii) else 0), raw=True)


def finish(m, seed, style):
    """After a building is built: per-building pattern variants (wall siding
    pattern, window curtains, kenchi or cast retaining walls, terracotta or
    galvalume roofs), generic names to Japanese ones, and UVs divided by the
    texture size with a random u offset so neighbours' stains differ.
    建物ごとの柄の選択・材質名の置換・UV の縮尺（汚れの位置を建物ごとにずらす）。"""
    rng = random.Random(seed)
    for mat in sorted(m.parts):
        if mat.startswith("jp_wall_") and mat[8:] in WALL_VARIANTS:
            # board siding three times as likely as each other pattern (estate
            # samples: ~20 % render, mostly siding); two-tone only on houses
            # 板柄を多めに（観察: 吹付けは約 2 割）、ツートンは戸建てのみ
            vs = [x for x in WALL_VARIANTS[mat[8:]] if style == "house" or x != "_twotone"]
            _rename(m, mat, mat + rng.choices(vs, [3.0] + [1.0] * (len(vs) - 1))[0])
    gv = rng.choice("ab")
    for old, new in (("glass", "jp_glass_1"), ("glass_2", "jp_glass_2"), ("glass_3", "jp_glass_3")):
        _rename(m, old, new + gv)
    for old, new in RENAME.items():
        _rename(m, old, new)
    if rng.random() < 0.45:
        _rename(m, "jp_retaining", "jp_retaining_kenchi")
    if rng.random() < 0.35:
        _rename(m, "jp_kawara_brown", "jp_kawara_terracotta")
    if style in ("shed", "apaato") and rng.random() < 0.5:
        _rename(m, "jp_roof_metal", "jp_roof_galvalume")
    if any(k.startswith(("jp_roof_slate", "jp_roof_metal", "jp_roof_galvalume")) for k in m.parts):
        _rename(m, "jp_ridge", "jp_ridge_metal")
    if style == "house" and rng.random() < 0.35:
        _rename(m, "jp_block_wall", "jp_block_wall_brick")
    for mat, part in m.parts.items():
        if mat not in UV_TILE:
            continue
        sx, sy = UV_TILE[mat]
        pre = UV_PRE.get(mat, 1.0)
        du = rng.random()
        anchored = mat.startswith(("jp_wall", "jp_tile", "jp_block", "jp_foundation", "jp_kanban", "jp_shopfront",
                                   "jp_concrete", "jp_ishigaki", "jp_retaining", "jp_panel", "jp_curtain",
                                   "jp_ribbon"))
        dv = 0.0 if anchored else rng.random()
        part[2] = [np.column_stack([uv[:, 0] * pre / sx + du, uv[:, 1] * pre / sy + dv]) for uv in part[2]]
