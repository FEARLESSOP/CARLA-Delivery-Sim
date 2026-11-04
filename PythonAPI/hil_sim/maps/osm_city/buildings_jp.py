"""Japanese general buildings (maps.json "road_style": "japan"), guide ch. 10 and 13.

Builder (buildings.py) hands every building of a Japanese map to JP. The
styles, from OSM tags, levels, footprint size/shape and the neighbourhood:

  house    detached house: foundation band, 2 floors (the second smaller:
           set back with a lean-to roof, or an L plan with a one-storey
           wing), hipped or gabled kawara roof with 0.6-0.9 m eaves and a
           ridge cap, entrance canopy, shutter boxes, balcony, AC units with
           duct covers, drain pipes; block wall and gate round the lot
  apaato   2-storey rental block: open upper corridor (1.2 m) with a solid
           railing, doors in a row, external steel stair, balconies and AC
           units at the back, low gable metal roof
  danchi   4-5 storey walk-up (company housing 社宅/寮, prefectural 県営,
           団地): stair cores with landings, balconies with laundry poles,
           block number on the end wall, flat roof with stair heads
  mansion  mid-rise flats: tile cladding, 1.0-1.5 m slab balconies across
           the front with partitions and AC units, open corridors behind a
           parapet, drain pipes, entrance canopy, lift-motor room on the roof
  shop     shops, offices, restaurants, public halls: shopfront, sign band,
           awning or canopy, rooms above, flat or hipped roof
  shed     sheds, garages (roll-up shutter), warehouses (gable metal roof)
  school / worship   the generic styles (buildings.py)

On terrain every building stands on a terrace (its lot): flat at the
highest ground under the footprint, the ground cut away beneath it, and a
retaining wall round it: battered stone ishigaki for houses where the drop
is large, concrete elsewhere; up to the natural ground where the lot is
cut into the slope. Lots are Voronoi cells of the footprints, set back from
every road, path, car park and landmark site. Nothing projects within
BUILD_CLEAR of a carriageway (each eave, balcony, stair and canopy is tested
and shrunk or dropped). Seeded variation; neighbours never share wall,
roof colour and roof form.

日本の一般建物（戸建て・アパート・団地・マンション・店舗・倉庫）。敷地は
最高地盤で平らに造成し、石積み（戸建て）またはコンクリートの擁壁で囲む。
"""

import math
import os
import random

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import buildings as BD
from buildings import bays, edges_of, levels_of, seed_of
from mesh import polys
from textures_jp_bld import FULL_ALWAYS, PER_FACE, RENAME as _TEX_RENAME

PATH_CLEAR = 0.1         # projections keep this off footways, drives, car parks / 歩道等からの離れ
FOOT_CLEAR = 0.4         # footprints keep this off them (AC units, sills, gutters stand out <= 0.35 m) / 外壁の離れ
LOT_PATH_CLEAR = 0.6     # lots keep this off them (battered stone toes) / 敷地の離れ
RAISED_Z = 0.15          # ground.RAISED_Z: the ground / pavement surface / 地面の高さ
BUILD_CLEAR = 2.2        # layout.BUILD_CLEAR / 建物と車道の最小距離
REACH = 1.5              # longest projection (balconies) / 最大の張り出し
GUTTER = 0.12            # eaves gutter beyond the fascia / 軒樋

# name: (rgb, roughness, metallic, kind) -- a plain texture each, so that the
# colour reaches Unreal (plain values do not, ch. 2); General Painter replaces
# them / 材質（単色テクスチャ。塗装は General Painter）
_M = {
    # walls / 外壁
    "jp_wall_white": ((238, 236, 228), 0.8, 0.0, "wall"), "jp_wall_ivory": ((234, 226, 205), 0.8, 0.0, "wall"),
    "jp_wall_beige": ((212, 194, 165), 0.8, 0.0, "wall"), "jp_wall_grey": ((168, 168, 164), 0.8, 0.0, "wall"),
    "jp_wall_darkgrey": ((92, 94, 97), 0.8, 0.0, "wall"), "jp_wall_brown": ((118, 88, 68), 0.8, 0.0, "wall"),
    "jp_wall_mocha": ((163, 138, 113), 0.8, 0.0, "wall"), "jp_wall_navy": ((62, 72, 92), 0.8, 0.0, "wall"),
    "jp_tile_white": ((230, 226, 216), 0.5, 0.0, "tile"), "jp_tile_beige": ((204, 184, 150), 0.5, 0.0, "tile"),
    "jp_tile_brown": ((124, 86, 62), 0.5, 0.0, "tile"), "jp_tile_darkbrown": ((80, 60, 48), 0.5, 0.0, "tile"),
    "jp_concrete_cream": ((222, 214, 194), 0.85, 0.0, "wall"), "jp_concrete_white": ((232, 231, 224), 0.85, 0.0, "wall"),
    "jp_foundation": ((150, 150, 146), 0.9, 0.0, "concrete"),
    # roofs / 屋根
    "jp_kawara_ibushi": ((84, 88, 94), 0.45, 0.1, "roof"), "jp_kawara_black": ((46, 48, 53), 0.45, 0.1, "roof"),
    "jp_kawara_brown": ((108, 76, 58), 0.5, 0.0, "roof"), "jp_kawara_blue": ((68, 88, 108), 0.4, 0.1, "roof"),
    "jp_roof_metal": ((72, 74, 77), 0.45, 0.6, "metalroof"), "jp_roof_metal_brown": ((92, 70, 56), 0.45, 0.6, "metalroof"),
    "jp_roof_slate": ((58, 55, 53), 0.6, 0.0, "roof"), "jp_ridge": ((60, 62, 66), 0.5, 0.1, "plain"),
    "jp_soffit": ((226, 223, 214), 0.8, 0.0, "plain"), "jp_gutter": ((196, 196, 192), 0.5, 0.3, "plain"),
    # details / 付属物
    "jp_metal_white": ((226, 226, 220), 0.5, 0.3, "plain"), "jp_metal_dark": ((62, 64, 67), 0.45, 0.6, "plain"),
    "jp_alu": ((176, 179, 181), 0.35, 0.7, "plain"), "jp_ac_unit": ((226, 226, 218), 0.6, 0.1, "plain"),
    "jp_pipe": ((212, 210, 202), 0.6, 0.1, "plain"), "jp_door": ((112, 86, 62), 0.6, 0.0, "plain"),
    "jp_door_steel": ((150, 142, 128), 0.5, 0.4, "plain"), "jp_shutter": ((166, 169, 171), 0.5, 0.5, "plain"),
    "jp_glass_frosted": ((200, 210, 214), 0.2, 0.1, "plain"), "jp_corridor": ((150, 150, 146), 0.85, 0.0, "concrete"),
    "jp_number": ((52, 66, 98), 0.6, 0.0, "plain"), "jp_metal_siding": ((170, 174, 176), 0.5, 0.5, "metalroof"),
    # lots and retaining walls / 敷地・擁壁
    "jp_lot": ((172, 170, 163), 0.9, 0.0, "concrete"), "jp_ishigaki": ((128, 123, 113), 0.9, 0.0, "stone"),
    "jp_retaining": ((164, 162, 154), 0.9, 0.0, "concrete"), "jp_block_wall": ((184, 181, 171), 0.9, 0.0, "block"),
    "jp_hedge": ((54, 94, 44), 0.9, 0.0, "plain"), "jp_dark": ((50, 50, 52), 0.9, 0.0, "plain"),
    # shops / 店舗
    # sign bands: "kanban", since CARLA's importer drops names with "sign" (ch. 2) / 看板帯
    "jp_kanban_white": ((240, 240, 236), 0.4, 0.0, "plain"), "jp_kanban_red": ((196, 46, 40), 0.4, 0.0, "plain"),
    "jp_kanban_blue": ((40, 86, 160), 0.4, 0.0, "plain"), "jp_kanban_green": ((40, 122, 70), 0.4, 0.0, "plain"),
    "jp_kanban_yellow": ((236, 196, 50), 0.4, 0.0, "plain"),
    "jp_rail_alu": ((176, 179, 181), 0.4, 0.6, "plain"), "jp_rail_white": ((226, 226, 220), 0.45, 0.0, "plain"),
    "jp_rail_dark": ((84, 78, 72), 0.45, 0.0, "plain"), "jp_awning_red": ((170, 40, 38), 0.7, 0.0, "plain"),
    "jp_awning_green": ((46, 100, 62), 0.7, 0.0, "plain"), "jp_awning_brown": ((110, 74, 52), 0.7, 0.0, "plain"),
}
MATERIALS = {k: (k, None, v[1], v[2], False) for k, v in _M.items()}

HOUSE_WALLS = ["jp_wall_white", "jp_wall_ivory", "jp_wall_beige", "jp_wall_grey", "jp_wall_darkgrey",
               "jp_wall_brown", "jp_wall_mocha", "jp_wall_navy", "jp_wall_white", "jp_wall_ivory"]
HOUSE_ROOFS = ["jp_kawara_ibushi", "jp_kawara_ibushi", "jp_kawara_black", "jp_kawara_brown", "jp_kawara_blue",
               "jp_roof_metal", "jp_roof_slate", "jp_roof_metal_brown"]
TILES = ["jp_tile_white", "jp_tile_beige", "jp_tile_brown", "jp_tile_darkbrown"]
SIGNS = ["jp_kanban_white", "jp_kanban_red", "jp_kanban_blue", "jp_kanban_green", "jp_kanban_yellow"]
AWNINGS = ["jp_awning_red", "jp_awning_green", "jp_awning_brown"]
# General Painter's mixes (Akeno, Street View 2017-2026 and the brief: mostly
# white, cream, beige, light grey and brown siding under dark roofs; ch. 13)
# 明野の配色（白・クリーム・ベージュ・薄灰・茶のサイディングに暗い屋根）
AKENO_HOUSE = [("jp_wall_white", 0.22), ("jp_wall_ivory", 0.2), ("jp_wall_beige", 0.2), ("jp_wall_grey", 0.14),
               ("jp_wall_brown", 0.1), ("jp_wall_mocha", 0.08), ("jp_wall_darkgrey", 0.04), ("jp_wall_navy", 0.02)]
AKENO_KAWARA = [("jp_kawara_ibushi", 0.5), ("jp_kawara_black", 0.22), ("jp_kawara_brown", 0.14),
                ("jp_kawara_blue", 0.14)]
AKENO_APAATO = [("jp_wall_beige", 0.3), ("jp_wall_ivory", 0.25), ("jp_wall_white", 0.15), ("jp_wall_grey", 0.15),
                ("jp_wall_mocha", 0.1), ("jp_wall_brown", 0.05), ("jp_wall_blue", 0.05)]
DANCHI_ACC = [("jp_concrete_white", 0.45), ("jp_wall_grey", 0.2), ("jp_wall_beige", 0.2), ("jp_wall_mocha", 0.15)]
AKENO_KAWARA += [("jp_roof_slate_grey", 0.3), ("jp_roof_slate", 0.2), ("jp_roof_slate_brown", 0.15)]
OBS_R, OBS_SHARE = 120.0, 0.35       # an observation colours its style within 120 m, 35 % of them / 観察の影響範囲
PITCHED = ("jp_kawara", "jp_roof_slate")     # same pitch: interchangeable by paint / 同じ勾配の屋根
METAL = ("jp_roof_metal",)                   # lower pitch: only among themselves / 低勾配の金属屋根


def area_of(x, y):
    """Photo-getter's estate areas (colour_notes.json areas_jp). / 住宅地の区域。"""
    if y < -150:
        return "SE" if x > 0 else "S/SW"
    if x > 150:
        return "NE/E"
    if x < -150:
        return "W/NW"
    return "N-centre"
DANCHI_WORDS = ("社宅", "寮", "県営", "市営", "団地", "宿舎", "NCC", "公社", "住宅")
SOUTH = np.array([0.0, -1.0])


def write_textures(d):
    """A plain, slightly noisy texture per material (plus roughness / metallic
    maps), from its own random stream. / 材質ごとの単色テクスチャ。"""
    from PIL import Image
    from textures import write_surface_maps
    rng = np.random.default_rng(1013)
    tex = {}
    n = 64
    for k, (c, r, mtl, kind) in _M.items():
        a = np.ones((n, n, 3)) * np.array(c, np.float64)
        a *= (0.96 + 0.06 * rng.random((n, n)))[..., None]
        if kind in ("roof", "metalroof"):            # tile courses / ribs / 瓦の段・リブ
            a[(np.arange(n) % 16) < 2] *= 0.75
        elif kind in ("stone", "block", "tile"):     # joints / 目地
            yy, xx = np.mgrid[0:n, 0:n]
            step = 16 if kind != "tile" else 8
            a[((yy % step) < 1) | (((xx + (yy // step % 2) * step // 2) % (2 * step)) < 1)] *= 0.7
        tex[k] = Image.fromarray(a.clip(0, 255).astype(np.uint8))
        tex[k].save(os.path.join(d, k + ".png"))
    write_surface_maps(d, tex, MATERIALS)


class Facade(BD.Facade):
    """buildings.Facade with UVs for General Painter's textures (ch. 13): every
    face metric (reveals and sides (w, z), soffits and slabs (u, w)), so v is
    the height above the terrace; doors, shutters, AC units and frosted glass
    (PER_FACE) get one texture per face and their small faces a plain spot.
    塗装用の UV を持つ外壁（v = 造成面からの高さ。扉・シャッター等は面ごとに 1 枚）。"""

    def _spot(self, mat):
        return PER_FACE.get(_TEX_RENAME.get(mat, mat))

    def face(self, mat, u0, u1, z0, z1, w=0.0, uvs=1.0):
        spot = self._spot(mat)
        if spot is None:
            return BD.Facade.face(self, mat, u0, u1, z0, z1, w, uvs)
        if u1 - u0 < 1e-3 or z1 - z0 < 1e-3:
            return
        if z1 - z0 >= 0.5 or _TEX_RENAME.get(mat, mat) in FULL_ALWAYS:
            uv = ((0, 0), (1, 0), (1, 1), (0, 1))
        else:
            uv = (spot,) * 4
        self.m.quad(mat, self.P(u0, w, z0), self.P(u1, w, z0), self.P(u1, w, z1), self.P(u0, w, z1), *uv)

    def side(self, mat, u, w0, w1, z0, z1, facing):
        if w1 - w0 < 1e-3 or z1 - z0 < 1e-3:
            return
        spot = self._spot(mat)
        if facing > 0:
            uv = ((-w1, z0), (-w0, z0), (-w0, z1), (-w1, z1)) if spot is None else (spot,) * 4
            self.m.quad(mat, self.P(u, w1, z0), self.P(u, w0, z0), self.P(u, w0, z1), self.P(u, w1, z1), *uv)
        else:
            uv = ((w0, z0), (w1, z0), (w1, z1), (w0, z1)) if spot is None else (spot,) * 4
            self.m.quad(mat, self.P(u, w0, z0), self.P(u, w1, z0), self.P(u, w1, z1), self.P(u, w0, z1), *uv)

    def hplane(self, mat, u0, u1, w0, w1, z, up=True):
        a, b, c, d = self.P(u0, w0, z), self.P(u1, w0, z), self.P(u1, w1, z), self.P(u0, w1, z)
        spot = self._spot(mat)
        ua, ub, uc, ud = ((u0, w0), (u1, w0), (u1, w1), (u0, w1)) if spot is None else (spot,) * 4
        if up:
            self.m.quad(mat, d, c, b, a, ud, uc, ub, ua)
        else:
            self.m.quad(mat, a, b, c, d, ua, ub, uc, ud)


# -- geometry helpers / 形状の補助
def face_out(m, mat, pts, out, uv=1.0):
    """A planar convex polygon (3D points) wound so its normal points along
    ``out`` (UE culls back faces). / 法線が out 側を向く平面多角形。"""
    P = np.asarray(pts, np.float64)
    n = np.cross(P[1] - P[0], P[2] - P[0])
    if np.linalg.norm(n) < 1e-9 and len(P) > 3:
        n = np.cross(P[2] - P[0], P[3] - P[0])
    ln = np.linalg.norm(n)
    if ln < 1e-9:
        return
    n /= ln
    if float(n @ np.asarray(out, np.float64)) < 0:
        P = P[::-1]
        n = -n
    # uv: metres in the face's plane, u level (along the eave / wall), v up
    # the slope, from the world origin so neighbouring faces join and v of a
    # wall is its height (textures_jp_bld) / 面内の m 単位（u 水平、v 斜面の上方向）
    if abs(n[2]) < 0.999:
        e1 = np.cross((0.0, 0.0, 1.0), n)
        e1 /= np.linalg.norm(e1)
    else:
        e1 = np.array([1.0, 0.0, 0.0])
    e2 = np.cross(n, e1)
    UV = np.column_stack([P @ e1, P @ e2]) / uv
    I = [k for j in range(1, len(P) - 1) for k in (0, j, j + 1)]
    m.tris(mat, P, n, UV, I)


def two_sided(m, mat, pts):
    """A thin plate seen from both sides (stair stringers, treads). / 両面の板。"""
    P = np.asarray(pts, np.float64)
    n = np.cross(P[1] - P[0], P[2] - P[0])
    face_out(m, mat, P, n)
    face_out(m, mat, P, -n)


class Rect(object):
    """An oriented rectangle: origin o, axis ax (long side A), ay to its left
    (side B); (s, r) -> o + s ax + r ay. / 向きのある矩形。"""

    def __init__(self, o, ax, A, B):
        self.o, self.ax = np.asarray(o, np.float64), np.asarray(ax, np.float64)
        self.ay = np.array([-self.ax[1], self.ax[0]])
        self.A, self.B = float(A), float(B)

    @classmethod
    def of(cls, poly):
        r = poly.minimum_rotated_rectangle
        c = [np.array(q) for q in list(orient(r, 1.0).exterior.coords)[:4]]
        e0, e1 = np.linalg.norm(c[1] - c[0]), np.linalg.norm(c[2] - c[1])
        if e0 >= e1:
            return cls(c[0], (c[1] - c[0]) / max(e0, 1e-9), e0, e1)
        return cls(c[1], (c[2] - c[1]) / max(e1, 1e-9), e1, e0)

    @classmethod
    def toward(cls, corners, up):
        """The rectangle through 4 corners whose ay points along ``up``. / ay が up を向く矩形。"""
        up = np.asarray(up, np.float64) / np.linalg.norm(up)
        ax = np.array([up[1], -up[0]])
        C = np.asarray(corners, np.float64)
        s, r = C @ ax, C @ up
        o = ax * s.min() + up * r.min()
        return cls(o, ax, s.max() - s.min(), r.max() - r.min())

    def xy(self, s, r):
        return self.o + self.ax * s + self.ay * r

    def P(self, s, r, z):
        q = self.xy(s, r)
        return (q[0], q[1], z)

    def poly(self, s0=0.0, s1=None, r0=0.0, r1=None):
        s1 = self.A if s1 is None else s1
        r1 = self.B if r1 is None else r1
        return Polygon([tuple(self.xy(s0, r0)), tuple(self.xy(s1, r0)), tuple(self.xy(s1, r1)), tuple(self.xy(s0, r1))])

    def local(self, xy):
        d = np.asarray(xy, np.float64) - self.o
        return d @ self.ax, d @ self.ay

    def sub(self, s0, s1, r0, r1, up=None):
        c = [self.xy(s0, r0), self.xy(s1, r0), self.xy(s1, r1), self.xy(s0, r1)]
        return Rect.toward(c, self.ay if up is None else up)

    def out(self, s, r, up=0.0):
        """Outward direction (horizontal + up) at a local point. / 外向き。"""
        v = self.ax * (s - self.A / 2) / max(self.A, 1e-6) + self.ay * (r - self.B / 2) / max(self.B, 1e-6)
        return (v[0], v[1], up)


def roof_solar(m, R, zt, where="yes", margin=1.3, depth=1.0, tilt=12.0, pitch=1.7, share=0.45):
    """Rows of solar panels on a flat roof (rectangle R, deck at zt) facing
    south, tilted ``tilt`` degrees, on the south half; ``where`` "west" / "east"
    puts them on that ``share`` of the roof's length, "yes" across it (minus the
    middle, where the lift-motor room stands). Top faces up ("solar", the
    map's panel texture), backs down and north (aluminium); returns the number
    of rows. / 陸屋根の太陽光パネル（南向き、南半分）。"""
    if R is None or R.A < 2 * margin + 2.0 or R.B < 2 * margin + depth:
        return 0
    south = float(R.ay @ SOUTH) > 0                      # +r runs south / +r が南
    w0 = float(R.xy(0.0, 0.0)[0]) < float(R.xy(R.A, 0.0)[0])     # s = 0 at the west end / s=0 が西端
    if where in ("west", "east"):
        at_low = (where == "west") == w0
        spans = [(margin, R.A * share)] if at_low else [(R.A * (1 - share), R.A - margin)]
    else:
        spans = [(margin, R.A / 2 - 2.5), (R.A / 2 + 2.5, R.A - margin)]
    zr = depth * math.sin(math.radians(tilt))
    dr = depth * math.cos(math.radians(tilt))
    n = 0
    mid = R.B / 2
    sgn = 1.0 if south else -1.0                          # r towards the south / 南向きの r
    k = 0
    while True:
        # each row from its high (north) edge, the first just south of the middle
        # 各列の北端（高い側）から。最初の列は中央のすぐ南
        r_n = mid + sgn * (0.4 + k * pitch)
        r_s = r_n + sgn * dr                              # the low (south) edge / 南端（低い側）
        k += 1
        if not (margin - 1e-6 <= r_s <= R.B - margin + 1e-6):
            break
        for s0, s1 in spans:
            if s1 - s0 < 1.0:
                continue
            a, b = zt + 0.35, zt + 0.35 + zr
            top = [R.P(s0, r_s, a), R.P(s1, r_s, a), R.P(s1, r_n, b), R.P(s0, r_n, b)]
            face_out(m, "solar", top, (sgn * R.ay[0] * 0.2, sgn * R.ay[1] * 0.2, 1.0), 1.0)
            low = [(x, y, z - 0.04) for x, y, z in top]
            face_out(m, "jp_alu", low, (0.0, 0.0, -1.0), 1.0)
            # the high (north) edge and the frame legs / 北側の縁と脚
            face_out(m, "jp_alu", [R.P(s0, r_n, zt), R.P(s1, r_n, zt), R.P(s1, r_n, b - 0.04),
                                           R.P(s0, r_n, b - 0.04)], (-sgn * R.ay[0], -sgn * R.ay[1], 0.0), 1.0)
            n += 1
    return n


def roof_hip(m, R, zw, pitch, ov, mat, th=0.15, cap=True):
    """Hipped roof slab over R (bottom plane through the wall line at zw),
    eaves ov, fascia, gutters, ridge cap. / 寄棟屋根（軒・鼻隠し・雨樋・棟）。"""
    A, B = R.A, R.B
    sa, sb = min(B / 2, A / 2), max(A - B / 2, A / 2)
    ze, zr = zw - ov * pitch, zw + B / 2 * pitch
    F = [(-ov, -ov), (A + ov, -ov), (A + ov, B + ov), (-ov, B + ov)]
    Ra, Rb = (sa, B / 2), (sb, B / 2)
    planes = [([F[0], F[1], Rb, Ra], -R.ay), ([F[2], F[3], Ra, Rb], R.ay),
              ([F[1], F[2], Rb], R.ax), ([F[3], F[0], Ra], -R.ax)]
    for pts, d in planes:
        z = lambda q: zr if q in (Ra, Rb) else ze
        top = [R.P(q[0], q[1], z(q) + th) for q in pts]
        bot = [R.P(q[0], q[1], z(q)) for q in pts]
        face_out(m, mat, top, (d[0], d[1], 1.0))
        face_out(m, "jp_soffit", bot, (-d[0], -d[1], -1.0))
    for k in range(4):
        a, b = F[k], F[(k + 1) % 4]
        d = [-R.ay, R.ax, R.ay, -R.ax][k]
        face_out(m, mat, [R.P(a[0], a[1], ze), R.P(b[0], b[1], ze), R.P(b[0], b[1], ze + th), R.P(a[0], a[1], ze + th)],
                 (d[0], d[1], 0.0))
    gutter_ring(m, R, -ov, A + ov, -ov, B + ov, ze, (True, True, True, True))
    if cap and sb - sa > 0.3:
        ridge_cap(m, R, sa, sb, B / 2, zr + th)


def roof_gable(m, R, zw, pitch, ov, mat, wall, th=0.15, cap=True):
    """Gabled roof slab over R, ridge along s, eaves ov, verges 0.75 ov,
    gable walls, barge boards, gutters. / 切妻屋根（妻壁・破風・雨樋）。"""
    A, B = R.A, R.B
    ovv = max(0.25, 0.75 * ov)
    ze, zr = zw - ov * pitch, zw + B / 2 * pitch
    for r0, r1, d in ((-ov, B / 2, -R.ay), (B + ov, B / 2, R.ay)):
        zz = lambda r: zr if r == B / 2 else ze
        q = [(-ovv, r0), (A + ovv, r0), (A + ovv, r1), (-ovv, r1)]
        face_out(m, mat, [R.P(s, r, zz(r) + th) for s, r in q], (d[0], d[1], 1.0))
        face_out(m, "jp_soffit", [R.P(s, r, zz(r)) for s, r in q], (-d[0], -d[1], -1.0))
        # fascia / 鼻隠し
        face_out(m, mat, [R.P(-ovv, r0, ze), R.P(A + ovv, r0, ze), R.P(A + ovv, r0, ze + th), R.P(-ovv, r0, ze + th)],
                 (d[0], d[1], 0.0))
        # barge boards at both verges / 破風
        for s, e in ((-ovv, -R.ax), (A + ovv, R.ax)):
            face_out(m, mat, [R.P(s, r0, ze), R.P(s, r1, zr), R.P(s, r1, zr + th), R.P(s, r0, ze + th)],
                     (e[0], e[1], 0.0))
    # gable walls up to the roof's underside / 妻壁
    for s, e in ((0.0, -R.ax), (A, R.ax)):
        face_out(m, wall, [R.P(s, 0, zw), R.P(s, B, zw), R.P(s, B / 2, zr)], (e[0], e[1], 0.0))
    gutter_ring(m, R, -ovv, A + ovv, -ov, B + ov, ze, (True, False, True, False))
    if cap:
        ridge_cap(m, R, -ovv, A + ovv, B / 2, zr + th)


def roof_mono(m, R, zhi, pitch, ov, mat, high_ov=0.0, th=0.12):
    """Lean-to roof over R falling toward r = 0 from zhi at r = B (against a
    wall, or with high_ov beyond it on a shed). / 片流れ屋根（下屋・物置）。"""
    A, B = R.A, R.B
    zb = lambda r: zhi - (B - r) * pitch
    q = [(-ov, -ov), (A + ov, -ov), (A + ov, B + high_ov), (-ov, B + high_ov)]
    face_out(m, mat, [R.P(s, r, zb(r) + th) for s, r in q], (-R.ay[0] * pitch, -R.ay[1] * pitch, 1.0))
    face_out(m, "jp_soffit", [R.P(s, r, zb(r)) for s, r in q], (R.ay[0] * pitch, R.ay[1] * pitch, -1.0))
    edges = [(q[0], q[1], -R.ay), (q[1], q[2], R.ax), (q[3], q[0], -R.ax)] + ([(q[2], q[3], R.ay)] if high_ov else [])
    for a, b, d in edges:
        face_out(m, mat, [R.P(a[0], a[1], zb(a[1])), R.P(b[0], b[1], zb(b[1])), R.P(b[0], b[1], zb(b[1]) + th),
                          R.P(a[0], a[1], zb(a[1]) + th)], (d[0], d[1], 0.0))
    gutter_ring(m, R, -ov, A + ov, -ov, B, zb(-ov), (True, False, False, False))


def roof_gable_ps(m, R, zw, pitch, ovs, mat, wall, th=0.15, cap=True):
    """Gabled roof over R (ridge along s) with an eave of its own on every
    side: ovs = (r0, s1, r1, s0); a side next to a road, footway or neighbour
    gets 0-0.1 m, the others their full eave. / 辺ごとの軒の出の切妻屋根。"""
    A, B = R.A, R.B
    o0, o1, o2, o3 = ovs
    v0, v1 = max(0.05, o3), max(0.05, o1)          # verges / けらば
    zr = zw + B / 2 * pitch
    for r0, oe, d, k in ((-o0, o0, -R.ay, 0), (B + o2, o2, R.ay, 2)):
        ze = zw - oe * pitch
        zz = lambda r: zr if r == B / 2 else ze
        q = [(-v0, r0), (A + v1, r0), (A + v1, B / 2), (-v0, B / 2)]
        face_out(m, mat, [R.P(s_, r, zz(r) + th) for s_, r in q], (d[0], d[1], 1.0))
        face_out(m, "jp_soffit", [R.P(s_, r, zz(r)) for s_, r in q], (-d[0], -d[1], -1.0))
        face_out(m, mat, [R.P(-v0, r0, ze), R.P(A + v1, r0, ze), R.P(A + v1, r0, ze + th), R.P(-v0, r0, ze + th)],
                 (d[0], d[1], 0.0))
        for s_, e in ((-v0, -R.ax), (A + v1, R.ax)):
            face_out(m, mat, [R.P(s_, r0, ze), R.P(s_, B / 2, zr), R.P(s_, B / 2, zr + th), R.P(s_, r0, ze + th)],
                     (e[0], e[1], 0.0))
        if oe >= 0.15:
            gutter_ring(m, R, -v0, A + v1, r0, r0, ze, tuple(j == k for j in range(4)))
    for s_, e in ((0.0, -R.ax), (A, R.ax)):
        face_out(m, wall, [R.P(s_, 0, zw), R.P(s_, B, zw), R.P(s_, B / 2, zr)], (e[0], e[1], 0.0))
    if cap:
        ridge_cap(m, R, -v0, A + v1, B / 2, zr + th)


def roof_mono_ps(m, R, zw, pitch, ovs, mat, wall, th=0.12):
    """Mono-pitch roof (片流れ) over R rising from the low side r = 0 (wall top
    zw) to r = B, an eave of its own on every side (r0, s1, r1, s0); side and
    high walls up to its underside. / 片流れ屋根（辺ごとの軒の出、妻壁と高い側の壁）。"""
    A, B = R.A, R.B
    o0, o1, o2, o3 = ovs
    zb = lambda r: zw + r * pitch
    q = [(-o3, -o0), (A + o1, -o0), (A + o1, B + o2), (-o3, B + o2)]
    face_out(m, mat, [R.P(s_, r, zb(r) + th) for s_, r in q], (-R.ay[0] * pitch, -R.ay[1] * pitch, 1.0))
    face_out(m, "jp_soffit", [R.P(s_, r, zb(r)) for s_, r in q], (R.ay[0] * pitch, R.ay[1] * pitch, -1.0))
    for a, b, d in ((q[0], q[1], -R.ay), (q[1], q[2], R.ax), (q[2], q[3], R.ay), (q[3], q[0], -R.ax)):
        face_out(m, mat, [R.P(a[0], a[1], zb(a[1])), R.P(b[0], b[1], zb(b[1])), R.P(b[0], b[1], zb(b[1]) + th),
                          R.P(a[0], a[1], zb(a[1]) + th)], (d[0], d[1], 0.0))
    for s_, e in ((0.0, -R.ax), (A, R.ax)):
        face_out(m, wall, [R.P(s_, 0, zw), R.P(s_, B, zw), R.P(s_, B, zb(B))], (e[0], e[1], 0.0))
    face_out(m, wall, [R.P(0, B, zw), R.P(A, B, zw), R.P(A, B, zb(B)), R.P(0, B, zb(B))], (R.ay[0], R.ay[1], 0.0))
    if o0 >= 0.15:
        gutter_ring(m, R, -o3, A + o1, -o0, -o0, zb(-o0), (True, False, False, False))


def gutter_ring(m, R, s0, s1, r0, r1, z, sides):
    """Eaves gutters (a 0.12 m box) along the chosen sides (r0, s1, r1, s0). / 軒樋。"""
    g = GUTTER
    for k, on in enumerate(sides):
        if not on:
            continue
        if k in (0, 2):
            r = r0 if k == 0 else r1
            d = -R.ay if k == 0 else R.ay
            a, b = R.xy(s0, r), R.xy(s1, r)
        else:
            s = s1 if k == 1 else s0
            d = R.ax if k == 1 else -R.ax
            a, b = R.xy(s, r0), R.xy(s, r1)
        a2, b2 = a + d * g, b + d * g
        P = lambda q, zz: (q[0], q[1], zz)
        face_out(m, "jp_gutter", [P(a2, z - 0.1), P(b2, z - 0.1), P(b2, z + 0.02), P(a2, z + 0.02)], (d[0], d[1], 0))
        face_out(m, "jp_gutter", [P(a, z - 0.1), P(b, z - 0.1), P(b2, z - 0.1), P(a2, z - 0.1)], (0, 0, -1))


def ridge_cap(m, R, s0, s1, r, z):
    """Ridge tiles (noshi): a 0.3 x 0.2 m bar along the ridge. / 棟瓦。"""
    w = 0.15
    c = [R.xy(s0, r - w), R.xy(s1, r - w), R.xy(s1, r + w), R.xy(s0, r + w)]
    P = lambda q, zz: (q[0], q[1], zz)
    face_out(m, "jp_ridge", [P(q, z + 0.2) for q in c], (0, 0, 1))
    for (a, b), d in zip(((c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])), (-R.ay, R.ax, R.ay, -R.ax)):
        face_out(m, "jp_ridge", [P(a, z - 0.05), P(b, z - 0.05), P(b, z + 0.2), P(a, z + 0.2)], (d[0], d[1], 0))


def ac_unit(f, u, z, w0=0.05):
    """An AC outdoor unit (0.8 x 0.3 x 0.6) on two feet. / エアコン室外機。"""
    f.block("jp_ac_unit", u, u + 0.8, w0, w0 + 0.3, z + 0.08, z + 0.68, top=True, bottom=False)
    for uu in (u + 0.08, u + 0.66):
        f.block("jp_metal_dark", uu, uu + 0.06, w0 + 0.02, w0 + 0.28, z, z + 0.08, top=False, bottom=False)


def pipe(f, u, z0, z1, w0=0.03, mat="jp_pipe", wd=0.09):
    """A vertical pipe / duct cover on the wall. / 縦樋・配管カバー。"""
    f.block(mat, u, u + wd, w0, w0 + wd, z0, z1, top=False, bottom=False)


def stair(m, f, u_b, run, w0, w1, z_b, z_t, mat="jp_metal_white", open_side=1):
    """An open steel stair along a facade, rising toward +u from (u_b, z_b) to
    (u_b + run, z_t): treads, two stringers, a handrail. / 鉄骨の外階段。"""
    n = max(4, int(round((z_t - z_b) / 0.2)))
    g = run / n
    rise = (z_t - z_b) / n
    for k in range(n - 1):
        z = z_b + (k + 1) * rise
        u0, u1 = u_b + k * g, u_b + (k + 1) * g + 0.03
        f.hplane(mat, u0, u1, w0, w1, z, True)
        f.hplane(mat, u0, u1, w0, w1, z - 0.04, False)
        f.side(mat, u0, w0, w1, z - 0.04, z, -1)
    u_t = u_b + run
    for w in (w0, w1):
        two_sided(m, mat, [f.P(u_b, w, z_b - 0.25), f.P(u_t, w, z_t - 0.25), f.P(u_t, w, z_t + 0.02),
                           f.P(u_b, w, z_b + 0.02)])
    # handrail on the open side / 手すり（外側）
    wr = w1 if open_side > 0 else w0
    two_sided(m, mat, [f.P(u_b, wr, z_b + 0.85), f.P(u_t, wr, z_t + 0.85), f.P(u_t, wr, z_t + 0.95),
                       f.P(u_b, wr, z_b + 0.95)])
    for uu in (u_b + 0.05, u_b + run * 0.5):
        zz = z_b + (uu - u_b) / run * (z_t - z_b)
        pw = (wr - 0.05, wr) if open_side > 0 else (wr, wr + 0.05)
        f.block(mat, uu, uu + 0.05, pw[0], pw[1], zz, zz + 0.9, top=False, bottom=False)


class Item(object):
    """One building's plan. / 1 棟の計画。"""
    pass


class JP(object):
    def __init__(self, B):
        self.B = B
        self.L = getattr(B, "L", None)
        self.D = getattr(B, "drape", None)
        self.items = None
        self.cur = None
        import collections
        self.stats = collections.Counter()     # roof outcomes for the build report / 屋根の集計

    # ------------------------------------------------------------ planning
    def style_of(self, t, poly, levels_tag):
        b = t.get("building", "yes")
        a = poly.area
        name = t.get("name") or ""
        am = t.get("amenity")
        if b in ("school", "kindergarten", "college", "university") or am in ("school", "kindergarten"):
            return "school"
        if b in ("church", "temple", "shrine", "chapel") or am == "place_of_worship":
            return "worship"
        if b in ("garage", "garages", "shed", "hut", "kiosk", "carport", "industrial", "warehouse", "barn",
                 "storage_tank", "service"):
            return "shed"
        if b in ("apartments", "dormitory", "residential"):
            if any(w in name for w in DANCHI_WORDS) or b == "dormitory":
                return "danchi"
            if levels_tag is not None:
                return "apaato" if levels_tag <= 2 else "mansion"
            return "apaato" if a < 450 else "mansion"
        if b in ("retail", "commercial", "office", "supermarket", "shop") or t.get("shop") or t.get("office") \
                and b != "house":
            return "shop"
        if am in ("restaurant", "pub", "cafe", "bank", "pharmacy", "post_office", "community_centre", "dentist",
                  "clinic", "doctors", "prep_school", "public_bath", "fast_food", "bar"):
            return "shop"
        if b in ("house", "detached", "semidetached_house", "terrace", "bungalow"):
            return "shed" if a < 22 else "house"
        # building=yes and the rest: by size / その他は面積で
        if a < 30:
            return "shed"
        if a < 220 and not name:
            return "house"
        if a > 500 and levels_tag is None:
            return "shed"
        return "shop"

    def overrides(self):
        """maps.json "building_levels": {way id: storeys} -- seen in Street View
        where OSM has none. / ストリートビューで確かめた階数。"""
        if getattr(self, "_lv_over", None) is None:
            import json
            self._lv_over = {}
            try:
                cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json")))
                name = self.L.meta.get("name") if self.L is not None else None
                ent = next((v for v in cfg.values() if isinstance(v, dict) and v.get("name") == name), {})
                self._lv_over = {str(k): int(v) for k, v in (ent.get("building_levels") or {}).items()
                                 if not str(k).startswith("_")}
            except (OSError, ValueError):
                pass
        return self._lv_over

    def levels(self, st, t, poly, rng):
        lv = self.overrides().get(str(t.get("@way"))) or levels_of(t, 0) or None
        if st == "house":
            return min(lv or (1 if poly.area < 45 else 2), 3)
        if st == "apaato":
            return min(lv or 2, 2)
        if st == "danchi":
            return lv or (5 if poly.area > 600 else 4)
        if st == "mansion":
            return max(3, lv or (5 if poly.area < 900 else 7))
        if st == "shop":
            return min(lv or (1 if poly.area > 250 else 2), 4)
        if st == "shed":
            return 1
        return lv or 2

    def free(self, g, i=None):
        """True if g keeps BUILD_CLEAR off every carriageway (and off the kerb)
        and off other buildings' footprints. / 道路・他棟に掛からないか。"""
        if self.L is None:
            return True
        i = self.cur if i is None else i
        if self.no_go.intersects(g):
            return False
        for j in self.tree.query(g, predicate="intersects"):
            if int(j) != i:
                return False
        return True

    def f_free(self, f, u0, u1, w0, w1):
        q = Polygon([f.P(u0, w0, 0)[:2], f.P(u1, w0, 0)[:2], f.P(u1, w1, 0)[:2], f.P(u0, w1, 0)[:2]])
        return self.free(q)

    def depth(self, f, u0, u1, want, w0=0.0, low=0.4):
        """The deepest projection <= want (down to low) clear of roads and
        neighbours, or 0. / 道路・隣棟に掛からない最大の張り出し。"""
        d = want
        while d >= low - 1e-9:
            if self.f_free(f, u0, u1, w0, d + 0.05):
                return d
            d -= 0.2
        return 0.0

    def plan(self):
        L, B = self.L, self.B
        self.items = {}
        if L is None:
            return
        # paths: footways, cycleways, drives and car parks -- nothing of a
        # building over them either (user's rule 1 covers footways)
        # 歩道・自転車道・私道・駐車場の上にも建物の部分を出さない
        self.paths = unary_union([L.footpaths, L.cycleways, L.driveways, L.parking])
        self.no_go = unary_union([L.carriageway.buffer(BUILD_CLEAR, quad_segs=3), L.road_all.buffer(0.6),
                                  self.paths.buffer(PATH_CLEAR, quad_segs=3)])
        shapely.prepare(self.no_go)
        # hard ground the terrace walls' toes keep off / 擁壁の根元が避ける面
        self.hard = unary_union([L.road_all, self.paths])
        shapely.prepare(self.hard)
        self.foot_off = self.paths.buffer(FOOT_CLEAR, quad_segs=3)
        shapely.prepare(self.foot_off)
        raw = []
        for i, (poly, t) in enumerate(L.buildings):
            p = orient(poly.simplify(0.25), 1.0)
            if p.geom_type != "Polygon" or p.area < 4.0:
                continue
            raw.append((i, p, t))
        self.keep_off = L.carriageway.buffer(BUILD_CLEAR + BD.FACADE_DEPTH, quad_segs=3)
        shapely.prepare(self.keep_off)
        # STRtree returning building indices / 建物番号を返す空間索引
        self.tree = _Tree(shapely.STRtree([p for _, p, _ in raw]), np.array([i for i, _, _ in raw]))
        for i, p, t in raw:
            self.cur = i
            rng = random.Random(seed_of("jp", i, p.centroid.x, p.centroid.y))
            lv_tag = levels_of(t, 0) or None
            it = Item()
            it.i, it.t, it.raw = i, t, p
            it.style = self.style_of(t, p, lv_tag)
            it.poly, it.shape, it.R, it.R2 = self.regularise(p, it.style, i)
            if it.style in ("apaato", "danchi", "mansion") and min(it.R.A, it.R.B) < 6.5 and \
                    max(it.R.A, it.R.B) > 5 * min(it.R.A, it.R.B):
                # a thin strip left after cutting a block back from the road: a
                # one-storey row (storage / garages), not a slab of flats
                # 道路から切り戻した細長い残り: 平屋の棟（物置・車庫）
                it.style = "shed"
            it.levels = self.levels(it.style, t, p, rng)
            self.items[i] = it
        self.lots()
        self.variants()
        self.terraces()

    def regularise(self, p, st, i):
        """A near-rectangular footprint becomes its rectangle (inset up to 0.6 m
        where the full one would touch the road clearance or a neighbour); a
        house with one corner notch becomes two rectangles (L plan); other
        compact houses keep their outline under a hip roof on the rectangle
        ("near"). / 矩形・L 字に整える（それ以外のまとまった形は寄棟を矩形に）。"""
        R = Rect.of(p)
        ratio = p.area / max(R.A * R.B, 1e-6)
        ok = lambda q: self.free(q.buffer(-0.03, join_style="mitre"), i) and \
            (self.L is None or not (self.keep_off.intersects(q.buffer(-0.03, join_style="mitre")) or
                                    self.foot_off.intersects(q)))
        if self.L is not None and self.foot_off.intersects(p):
            # an outline on (or too near) a footway, drive or car park: cut back
            # 歩道・私道・駐車場に近い外形は切り戻す
            q = [g for g in polys(p.difference(self.paths.buffer(FOOT_CLEAR + 0.1, quad_segs=3))) if g.area > 4.0]
            if not q:
                return orient(p.buffer(-0.5, join_style="mitre"), 1.0) if p.buffer(-0.5).area > 4 else p, \
                    "free", R, None
            p = orient(max(q, key=lambda g: g.area).simplify(0.05), 1.0)
            R = Rect.of(p)
            ratio = p.area / max(R.A * R.B, 1e-6)
        if st in ("house", "apaato", "danchi", "mansion", "shop", "shed") and ratio > 0.86:
            for d in (0.0, 0.15, 0.3, 0.45, 0.6):
                if min(R.A, R.B) - 2 * d < 2.0:
                    break
                Rd = R.sub(d, R.A - d, d, R.B - d) if d else R
                Rd = Rect.of(Rd.poly()) if d else R
                q = Rd.poly()
                if q.symmetric_difference(p).area < 0.2 * p.area and ok(q):
                    return orient(q, 1.0), "rect", Rd, None
        if st == "house" and 0.5 < ratio <= 0.88 and min(R.A, R.B) > 5.0:
            lr = self.l_plan(p, R)
            if lr is not None:
                q = orient(unary_union([lr[0].poly(), lr[1].poly()]).buffer(0.01, join_style="mitre")
                           .buffer(-0.01, join_style="mitre"), 1.0)
                if q.geom_type == "Polygon" and q.symmetric_difference(p).area < 0.2 * p.area and ok(q):
                    return q, "L", lr[0], lr[1]
        if st == "house" and ratio > 0.7:
            return p, "near", R, None
        return p, "free", R, None

    def l_plan(self, p, R):
        """Two rectangles (main, wing) for an L footprint, or None. / L 字の 2 矩形。"""
        notch = R.poly().difference(p.buffer(0.05, join_style="mitre"))
        parts = sorted(polys(notch), key=lambda g: -g.area)
        if not parts or parts[0].area < 0.08 * R.A * R.B:
            return None
        s, r = R.local(np.asarray(parts[0].exterior.coords))
        ns0, ns1, nr0, nr1 = s.min(), s.max(), r.min(), r.max()
        tol = 0.6
        at_s0, at_s1 = ns0 < tol, ns1 > R.A - tol
        at_r0, at_r1 = nr0 < tol, nr1 > R.B - tol
        if not ((at_s0 or at_s1) and (at_r0 or at_r1)) or (at_s0 and at_s1) or (at_r0 and at_r1):
            return None
        if at_r1:
            main = R.sub(0, R.A, 0, nr0)
            wing = R.sub(ns1, R.A, nr0, R.B) if at_s0 else R.sub(0, ns0, nr0, R.B)
        else:
            main = R.sub(0, R.A, nr1, R.B)
            wing = R.sub(ns1, R.A, 0, nr1) if at_s0 else R.sub(0, ns0, 0, nr1)
        main, wing = (main, wing) if main.A * main.B >= wing.A * wing.B else (wing, main)
        if min(main.A, main.B) < 3.5 or min(wing.A, wing.B) < 2.4:
            return None
        return Rect.of(main.poly()), Rect.of(wing.poly())

    def lots(self):
        """Each building's lot: its Voronoi cell (of footprint boundary points)
        within a style setback, off roads, paths, car parks, courts and
        landmark sites. / 敷地: 外形点のボロノイ領域を後退距離で切る。"""
        L = self.L
        pts, own = [], []
        for i, it in self.items.items():
            c = shapely.get_coordinates(shapely.segmentize(it.poly.exterior, 1.5))[:-1]
            pts.append(c)
            own += [i] * len(c)
        P = np.round(np.vstack(pts), 2)
        P, first = np.unique(P, axis=0, return_index=True)
        own = np.asarray(own)[first]
        cells = shapely.get_parts(shapely.voronoi_polygons(shapely.multipoints(P), extend_to=L.bounds.buffer(100),
                                                           ordered=True))
        by = {}
        for c, o in zip(cells, own):
            by.setdefault(int(o), []).append(c)
        keep = getattr(self.B, "keep_clear", None)
        obst = [L.road_all.buffer(0.8), L.islands.buffer(0.2), self.paths.buffer(LOT_PATH_CLEAR), L.pools.buffer(0.3),
                L.pitches.buffer(0.2),
                L.playgrounds.buffer(0.2), L.water.buffer(0.3), getattr(L, "reserved", Polygon())]
        if getattr(L, "landmark_areas", None) is not None:
            obst.append(L.landmark_areas.buffer(1.0))
        if keep is not None:
            obst.append(keep)
        self.obst = unary_union([g for g in obst if g is not None and not g.is_empty])
        shapely.prepare(self.obst)
        setback = {"house": 2.2, "apaato": 2.2, "danchi": 3.0, "mansion": 2.5, "shop": 1.2, "shed": 0.4,
                   "school": 1.5, "worship": 1.5}
        for i, it in self.items.items():
            it.cell = shapely.union_all(by.get(i, [])).buffer(0)
            it.setback = setback.get(it.style, 1.5)

    def lot_of(self, it, s):
        g = it.poly.buffer(s, join_style="mitre").intersection(it.cell).difference(self.obst)
        g = g.buffer(-0.25, join_style="mitre").buffer(0.25, join_style="mitre")
        g = unary_union([g, it.poly])
        part = max(polys(g), key=lambda q: q.intersection(it.poly).area) if polys(g) else it.poly
        part = Polygon(part.exterior).simplify(0.1)
        return orient(part, 1.0) if part.is_valid and part.contains(it.poly.representative_point()) else it.poly

    def terraces(self):
        """Flat lot terraces on the highest ground under each footprint; the
        lot shrinks where the cut or fill beyond the footprint grows too big.
        敷地を最高地盤で造成（切土・盛土が大きすぎる所は敷地を縮める）。"""
        L, D = self.L, self.D
        for i, it in self.items.items():
            if D is not None:
                zlo, zhi = D.footprint(it.poly)
            else:
                zlo = zhi = 0.0
            it.zb, it.drop = zhi, zhi - zlo
            s = it.setback
            for _ in range(4):
                lot = self.lot_of(it, s)
                if D is None:
                    break
                c = shapely.get_coordinates(shapely.segmentize(lot.exterior, 1.0))
                g = D.z(c[:, 0], c[:, 1])
                fill, cut = zhi - g.min(), g.max() - zhi
                if fill <= it.drop + 2.0 and cut <= 1.6:
                    break
                s *= 0.5
                if s < 0.3:
                    g = unary_union([it.poly.buffer(0.15, join_style="mitre").difference(self.obst), it.poly])
                    lot = orient(max(polys(g), key=lambda q: q.area), 1.0) if polys(g) else it.poly
                    break
            # filling holes can swallow a footway the lot surrounds: cut again
            # 穴埋めで囲んだ歩道を含めないよう再度切る
            g = unary_union([lot.difference(self.obst), it.poly]).buffer(0)
            pp = [q for q in polys(g) if q.intersects(it.poly)]
            it.lot = orient(max(pp, key=lambda q: q.intersection(it.poly).area), 1.0) if pp else it.poly
        self.unoverlap()
        lots = [it.lot for it in self.items.values()]
        self.lot_ids = np.array(list(self.items.keys()))
        self.lot_tree = shapely.STRtree(lots)
        from layout import clean
        T = unary_union(lots + [it.poly for it in self.items.values()])
        L.terraces = T
        for nm in ("islands", "parks", "forest", "pitches", "playgrounds", "construction", "grass"):
            if hasattr(L, nm):
                setattr(L, nm, clean(getattr(L, nm).difference(T), 0.05))
        # the terraces replace the paved aprons round the buildings: the strips
        # left between a lot and a path become grass
        # 建物周りの舗装は造成面に置き換え（残りの細片は芝）
        L.aprons = Polygon()

    def unoverlap(self):
        """Terraces never overlap: each lot loses the other footprints (+0.1 m)
        and the lots already settled (lower index), then keeps the part on its
        own footprint. / 造成面を重ねない（他棟の外形と先に決まった敷地を除く）。"""
        ids = sorted(self.items)
        fps = shapely.STRtree([self.items[i].poly for i in ids])
        settled = []
        st_ids = []
        for n_, i in enumerate(ids):
            it = self.items[i]
            lot = it.lot
            near = [ids[k] for k in fps.query(lot.buffer(0.2), predicate="intersects") if ids[k] != i]
            cut = [self.items[j].poly.buffer(0.1, join_style="mitre") for j in near]
            if settled:
                tr = shapely.STRtree(settled)
                cut += [settled[k] for k in tr.query(lot, predicate="intersects")]
            if cut:
                g = lot.difference(unary_union(cut))
                g = unary_union([g, it.poly]).buffer(0)
                pp = [q for q in polys(g) if q.intersects(it.poly)]
                g = max(pp, key=lambda q: q.intersection(it.poly).area) if pp else it.poly
                if settled:
                    g = g.difference(unary_union([settled[k] for k in tr.query(g, predicate="intersects")]))
                    pp = [q for q in polys(g) if q.intersects(it.poly)]
                    g = max(pp, key=lambda q: q.area) if pp else it.poly
                it.lot = orient(g if g.geom_type == "Polygon" else it.poly, 1.0)
            settled.append(it.lot)
            st_ids.append(i)
        # close slivers: each lot grows up to 0.3 m into free ground (never into
        # paths, roads, other footprints or lots), so lots that nearly touch
        # share an exact edge and their walls meet / 細い隙間を埋め、隣接する敷地の境界を一致させる
        tree = shapely.STRtree([self.items[i].lot for i in ids])
        for n_, i in enumerate(ids):
            it = self.items[i]
            # (old outlines up to 0.6 m apart can both grow into the same strip) / 両側から伸びる隙間も見る
            near = [k for k in tree.query(it.lot.buffer(0.8), predicate="intersects") if ids[k] != i]
            others = unary_union([self.items[ids[k]].lot for k in near] +
                                 [self.items[ids[k]].poly.buffer(0.1, join_style="mitre") for k in near])
            grow = it.lot.buffer(0.3, join_style="mitre").difference(others).difference(self.obst)
            g = unary_union([it.lot, grow.intersection(it.lot.buffer(0.3, join_style="mitre"))]).buffer(0)
            # an opening removes needles where two grown outlines zig-zag along one
            # line (duplicate walls there flickered); never larger than before
            # 2 つの敷地が同じ線上でジグザグする針状部分を除く（重複擁壁のちらつき）
            g = unary_union([g.buffer(-0.02, quad_segs=2).buffer(0.02, quad_segs=2).intersection(g), it.poly])
            pp = [q for q in polys(g) if q.intersects(it.poly)]
            if pp:
                it.lot = orient(clean_ring(max(pp, key=lambda q: q.area), it.poly), 1.0)

    def variants(self):
        """Seeded looks; a house never shares wall colour, roof colour and roof
        form with a neighbour within 25 m. / 隣家と同じ外観にしない。"""
        ids = sorted(self.items)
        C = np.array([[self.items[i].poly.centroid.x, self.items[i].poly.centroid.y] for i in ids])
        done = {}
        for n_, i in enumerate(ids):
            it = self.items[i]
            rng = random.Random(seed_of("jpvar", i, round(C[n_, 0], 1), round(C[n_, 1], 1)))
            d = np.hypot(C[:n_, 0] - C[n_, 0], C[:n_, 1] - C[n_, 1])
            near = [done[ids[k]] for k in np.nonzero(d < 25.0)[0]]
            for _ in range(12):
                v = self.pick(it, rng)
                key = (v["wall"], v.get("roof"), v.get("kind"))
                if all((n["wall"], n.get("roof"), n.get("kind")) != key for n in near):
                    break
            it.var = v
            done[i] = v
        self.paint()

    def paint(self):
        """General Painter's colours: each building's wall (and its roof within
        the same family, so the geometry stays the same) from the Street View
        observations of its neighbourhood (colour_notes.json "obs", nearest of
        its style within OBS_R) or else Akeno's mix, with seeded variation;
        never the same wall and roof as a neighbour within 25 m. Own random
        stream: shapes and the build's other choices are untouched.
        塗装: 近くの Street View 観察（同じ様式）または明野の配色から外壁・屋根色を選ぶ。"""
        obs = [o for o in (getattr(self.B.sites, "obs", None) or []) if str(o.get("wall", "")).startswith("jp_")]
        self.areas = (getattr(self.B.sites, "notes", None) or {}).get("areas_jp", {})
        ids = sorted(self.items)
        C = np.array([[self.items[i].poly.centroid.x, self.items[i].poly.centroid.y] for i in ids]) \
            if ids else np.zeros((0, 2))
        done = {}
        for n_, i in enumerate(ids):
            it = self.items[i]
            v = it.var
            rng = random.Random(seed_of("jppaint", i, round(C[n_, 0], 1), round(C[n_, 1], 1)))
            near_o = None
            bd = OBS_R
            for o in obs:
                if o.get("style") != it.style:
                    continue
                d = math.hypot(o["xy"][0] - C[n_, 0], o["xy"][1] - C[n_, 1])
                if d < bd:
                    near_o, bd = o, d
            d = np.hypot(C[:n_, 0] - C[n_, 0], C[:n_, 1] - C[n_, 1])
            near = [done[ids[k]] for k in np.nonzero(d < 25.0)[0]]
            for _ in range(8):
                w, r, a = self.paint_pick(it, v, rng, near_o)
                if all((w, r) != q for q in near):
                    break
            if w:
                v["wall"] = w
            if r:
                v["roof"] = r
            if a:
                v["acc"] = a
            done[i] = (v.get("wall"), v.get("roof"))
        # colours fixed by the building's own tags (maps.json osm_fixes, from the
        # user's photo: it wins over the mixes, ch. 17) / タグで指定した配色（利用者の写真）
        for i in ids:
            it = self.items[i]
            for k in ("wall", "acc", "roof", "bal"):
                if it.t.get("jp:" + k):
                    it.var[k] = it.t["jp:" + k]

    def paint_pick(self, it, v, rng, o):
        """(wall, roof, accent) for one building, None to keep. / 1 棟の配色。"""
        st = it.style
        wpick = lambda mix: rng.choices([k for k, _ in mix], [q for _, q in mix])[0]
        use_o = o is not None and (rng.random() < OBS_SHARE or
                                   math.hypot(o["xy"][0] - it.poly.centroid.x, o["xy"][1] - it.poly.centroid.y) < 15)
        if st == "house":
            # the area's mix (estate samples) unless a nearby observation is taken;
            # roofs only within their family: kawara <-> slate (same pitch), metal
            # <-> metal / 区域の配色。屋根は同じ勾配の系統内（瓦⇔スレート、金属⇔金属）
            A = self.areas.get(area_of(it.poly.centroid.x, it.poly.centroid.y), {})
            mix = lambda key, dflt: list(A[key].items()) if A.get(key) else dflt
            w = o["wall"] if use_o else wpick(mix("house_walls", AKENO_HOUSE))
            r = None
            ro = str(o.get("roof") or "") if use_o else ""
            if v["roof"].startswith(PITCHED):
                r = ro if ro.startswith(PITCHED) else wpick(mix("house_roofs_pitched", AKENO_KAWARA))
            elif v["roof"].startswith(METAL):
                r = ro if ro.startswith(METAL) else wpick(mix("house_roofs_metal",
                                                              [("jp_roof_metal", 0.6), ("jp_roof_metal_brown", 0.4)]))
            return w, r, None
        if st == "apaato":
            r = None
            ro = str(o.get("roof") or "") if use_o else ""
            if ro and v["roof"].startswith(PITCHED) and ro.startswith(PITCHED) or \
                    ro and v["roof"].startswith(METAL) and ro.startswith(METAL):
                r = ro
            return (o["wall"] if use_o else wpick(AKENO_APAATO)), r, None
        if st == "danchi":
            w = o["wall"] if use_o else rng.choice(["jp_concrete_white", "jp_concrete_cream"])
            a = o.get("acc") if use_o and o.get("acc") else wpick(DANCHI_ACC)
            return w, None, a
        if st == "mansion":
            return (o["wall"] if use_o else None), None, (o.get("acc") if use_o else None)
        if st == "shop" and use_o:
            return o["wall"], None, None
        return None, None, None

    def pick(self, it, rng):
        st = it.style
        v = {"rng_seed": rng.random()}
        if st == "house":
            # roofs: about 10 % metal (Street View), the rest kawara / slate; forms
            # hip 45 %, gable 45 %, mono-pitch 10 % (modern houses, mostly metal)
            # 屋根: 金属約 1 割、残りは瓦・スレート。寄棟・切妻・片流れ
            metal = rng.random() < 0.10
            roof = rng.choice(["jp_roof_metal", "jp_roof_metal_brown"]) if metal else \
                rng.choice([r for r in HOUSE_ROOFS if not r.startswith(METAL)])
            fr = rng.random()
            kind = "mono" if (fr < 0.06 or (metal and fr < 0.5)) else ("hip" if fr < 0.53 else "gable")
            v.update(wall=rng.choice(HOUSE_WALLS), roof=roof, kind=kind, ov=rng.choice([0.6, 0.75, 0.9]),
                     setback=rng.random() < 0.6, lotwall=rng.choice(["block", "block", "block_fence", "hedge", "low"]),
                     balcony=rng.random() < 0.7, trim=rng.choice(["jp_wall_white", "jp_wall_darkgrey", "jp_alu"]))
            v["pitch"] = 0.3 if v["roof"].startswith("jp_roof_metal") else rng.choice([0.4, 0.45, 0.5])
        elif st == "apaato":
            v.update(wall=rng.choice(["jp_wall_beige", "jp_wall_white", "jp_wall_ivory", "jp_wall_grey",
                                      "jp_wall_mocha", "jp_wall_navy"]),
                     roof=rng.choice(["jp_roof_metal", "jp_roof_metal_brown", "jp_roof_slate", "jp_kawara_ibushi"]),
                     kind="gable", rail=rng.choice(["jp_rail_alu", "jp_rail_white", "jp_rail_dark"]),
                     stair_end=rng.random() < 0.5, lotwall=rng.choice(["block", "block_fence", "low"]))
        elif st == "danchi":
            v.update(wall=rng.choice(["jp_concrete_cream", "jp_concrete_white", "jp_wall_ivory", "jp_wall_beige"]),
                     acc=rng.choice(["jp_wall_brown", "jp_wall_grey", "jp_concrete_white", "jp_wall_mocha"]))
        elif st == "mansion":
            v.update(wall=rng.choice(TILES), acc=rng.choice(["jp_tile_white", "jp_tile_brown", "jp_wall_grey",
                                                             "jp_concrete_white"]),
                     bal=rng.choice(["solid", "solid", "glass", "rail"]))
        elif st == "shop":
            v.update(wall=rng.choice(["jp_wall_white", "jp_wall_beige", "jp_wall_grey", "jp_tile_beige",
                                      "jp_wall_ivory", "jp_tile_brown"]),
                     sign=rng.choice(SIGNS), awning=rng.choice(AWNINGS + [None, None]),
                     roof=rng.choice(["flat", "flat", "flat", "hip"]), rmat=rng.choice(HOUSE_ROOFS[:5]))
        elif st == "shed":
            v.update(wall=rng.choice(["jp_metal_siding", "jp_metal_siding", "jp_wall_grey", "jp_wall_beige"]),
                     roof=rng.choice(["jp_roof_metal", "jp_roof_metal_brown", "jp_metal_siding"]))
        else:
            v.update(wall="jp_wall_white")
        return v

    # ------------------------------------------------------------ building
    def build(self, poly, t, idx):
        """The mesh of building idx (local z: the terrace top is RAISED_Z), or
        None. / 建物 idx のメッシュ（局所 z、造成面 = RAISED_Z）。"""
        from mesh import Mesh
        if self.items is None:
            self.plan()
        it = self.items.get(idx)
        if it is None:
            return None
        self.cur = idx
        rng = random.Random(seed_of("jpb", idx, it.poly.centroid.x, it.poly.centroid.y))
        m = Mesh()
        if it.style == "school":
            self.B.b_school(m, it.poly, t, it.levels, rng)
            it.height = it.levels * 3.5 + 1.0
        elif it.style == "worship":
            self.B.b_worship(m, it.poly, t, it.levels, rng)
            it.height = max(2, it.levels) * 3.8 + 4.0
        else:
            getattr(self, "b_" + it.style)(m, it, rng)
        self.site(m, it)
        # General Painter: pattern variants, Japanese material names, UVs to the
        # texture size (textures_jp_bld) / 柄の選択・材質名・UV の縮尺
        import textures_jp_bld
        textures_jp_bld.finish(m, seed_of("jpfinish", idx, round(it.poly.centroid.x, 1),
                                          round(it.poly.centroid.y, 1)), it.style)
        self.B.count[it.style] = self.B.count.get(it.style, 0) + 1
        return m

    def front(self, poly):
        return self.B.front_normal(poly) if self.L is not None else np.array([0.0, -1.0])

    # -- walls with openings / 開口付きの壁
    def wall_ops(self, f, z0, z1, wall, ops, frame_mat="jp_alu"):
        """ops: [(u0, u1, oz0, oz1, kind)] kind win / sweep / frosted / door / dark.
        開口: 窓・掃き出し窓・型板ガラス・玄関・暗部。"""
        f.wall_with_openings(wall, z0, z1, [o[:4] for o in ops], 1.0)
        for u0, u1, oz0, oz1, kind in ops:
            if kind == "door":
                d = 0.25
                f.hplane(wall, u0, u1, -d, 0.0, oz1, False)
                f.side(wall, u0, -d, 0.0, oz0, oz1, 1)
                f.side(wall, u1, -d, 0.0, oz0, oz1, -1)
                f.face("jp_door", u0, u1, oz0, oz1, -d)
                f.hplane("jp_foundation", u0, u1, -d, 0.0, oz0, True)
            elif kind == "dark":
                f.face("jp_dark", u0, u1, oz0, oz1, -0.6)
                f.hplane(wall, u0, u1, -0.6, 0.0, oz1, False)
                f.hplane("jp_corridor", u0, u1, -0.6, 0.0, oz0, True)
                f.side(wall, u0, -0.6, 0.0, oz0, oz1, 1)
                f.side(wall, u1, -0.6, 0.0, oz0, oz1, -1)
            else:
                glass = "jp_glass_frosted" if kind == "frosted" else "glass"
                f.window(u0, u1, oz0, oz1, wall, glass=glass, mullions=0 if (u1 - u0) < 0.9 or kind == "frosted"
                         else 1, depth=0.12)
                if kind == "frosted" and oz1 - oz0 < 1.3:
                    # window grille / 面格子
                    for k in range(1, 4):
                        uu = u0 + (u1 - u0) * k / 4
                        f.block(frame_mat, uu - 0.015, uu + 0.015, 0.02, 0.06, oz0, oz1, top=True, bottom=True,
                                ends=False)

    def house_ops(self, L, z0, z1, ground, frontish, garden, rng, door=True):
        ops = []
        bs = bays(L, 3.0, 0.45)
        if not bs:
            return ops
        door_bay = (0 if rng.random() < 0.5 else len(bs) - 1) if (ground and frontish and door) else -1
        for k, (a, b) in enumerate(bs):
            mid = (a + b) / 2
            if k == door_bay:
                ops.append((mid - 0.5, mid + 0.5, z0, z0 + 2.05, "door"))
            elif ground and garden and k % 2 == 0 and b - a > 2.0:
                ops.append((mid - 0.85, mid + 0.85, z0 + 0.05, z0 + 2.0, "sweep"))
            else:
                w = rng.choice([0.6, 1.2, 1.65, 1.65])
                w = min(w, b - a - 0.4)
                if w < 0.5:
                    continue
                kind = "frosted" if (w < 0.7 and not frontish) else "win"
                top = z0 + (2.0 if w > 1.0 else 1.85)
                ops.append((mid - w / 2, mid + w / 2, top - (1.1 if w > 1.0 else 0.6), top, kind))
        return ops

    # -- styles / 様式
    def b_house(self, m, it, rng):
        v = it.var
        wall, roof, trim = v["wall"], v["roof"], v["trim"]
        z0 = RAISED_Z
        zf = z0 + 0.45                     # floor level over the foundation / 基礎の上の床
        H = 2.85
        lv = it.levels
        front = self.front(it.poly)
        floors, roofs = [], []
        if it.shape == "rect" and lv >= 2 and v["setback"] and min(it.R.A, it.R.B) > 6.5:
            R = it.R
            # the second floor set back from the side facing the road or south: a lean-to below
            # 2 階は道路側か南側を後退させ、下屋を掛ける
            sides = [(0, -R.ay), (1, R.ax), (2, R.ay), (3, -R.ax)]
            pref = lambda d: float(np.asarray(d) @ front) + 0.5 * float(np.asarray(d) @ SOUTH) + rng.random() * 0.3
            k, d = max(sides, key=lambda q: pref(q[1]))
            span = R.B if k in (0, 2) else R.A
            cut = min(max(1.8, span * rng.uniform(0.25, 0.38)), span - 4.5)
            if cut >= 1.5:
                if k == 0:
                    up, strip = R.sub(0, R.A, cut, R.B), R.sub(0, R.A, 0, cut, up=R.ay)
                elif k == 2:
                    up, strip = R.sub(0, R.A, 0, R.B - cut), R.sub(0, R.A, R.B - cut, R.B, up=-R.ay)
                elif k == 1:
                    up, strip = R.sub(0, R.A - cut, 0, R.B), R.sub(R.A - cut, R.A, 0, R.B, up=-R.ax)
                else:
                    up, strip = R.sub(cut, R.A, 0, R.B), R.sub(0, cut, 0, R.B, up=R.ax)
                up = Rect.of(up.poly())
                floors = [(it.poly, zf, zf + H)] + [(orient(up.poly(), 1.0), zf + H * j, zf + H * (j + 1))
                                                     for j in range(1, lv)]
                roofs = [("main", up, zf + H * lv), ("lean", strip, zf + H)]
        if not floors and it.shape == "L":
            R1, R2 = it.R, it.R2
            if lv >= 2:
                floors = [(it.poly, zf, zf + H)] + [(orient(R1.poly(), 1.0), zf + H * j, zf + H * (j + 1))
                                                     for j in range(1, lv)]
                roofs = [("main", R1, zf + H * lv), ("wing", R2, zf + H)]
            else:
                floors = [(it.poly, zf, zf + H)]
                roofs = [("main", R1, zf + H), ("wing", R2, zf + H)]
        if not floors:
            floors = [(it.poly, zf, zf + H * j + H) for j in range(1)] + \
                     [(it.poly, zf + H * j, zf + H * (j + 1)) for j in range(1, lv)]
            roofs = [("main", it.R, zf + H * lv)] if it.shape == "rect" else \
                [("near", it.R, zf + H * lv)] if it.shape == "near" else [("free", it.R, zf + H * lv)]
        # walls, floor by floor / 階ごとの外壁
        balc_done = False
        for fi, (fp, za, zb) in enumerate(floors):
            for a, b, L, n in edges_of(fp):
                f = Facade(m, a, b, n)
                frontish = float(n @ front) > 0.55
                garden = float(n @ SOUTH) > 0.4 and not frontish
                if fi == 0:
                    f.block("jp_foundation", 0.0, L, 0.0, 0.03, z0 - 0.02, zf, top=True, bottom=False, ends=False)
                if L < 1.2:
                    f.face(wall, 0.0, L, za, zb, 0.0)
                    continue
                ops = self.house_ops(L, za, zb, fi == 0, frontish, garden, rng)
                balc = None
                if fi == 1 and v["balcony"] and not balc_done and (garden or frontish) and L > 3.6:
                    w = min(L - 0.8, rng.uniform(2.7, 4.5))
                    u0 = 0.4 if rng.random() < 0.5 else L - 0.4 - w
                    dd = self.depth(f, u0, u0 + w, 0.9, low=0.7)
                    if dd > 0:
                        balc = (u0, u0 + w, dd)
                        balc_done = True
                        ops = [o for o in ops if o[1] < u0 - 0.1 or o[0] > u0 + w + 0.1]
                        mid = u0 + w / 2
                        ops.append((mid - 0.85, mid + 0.85, za + 0.05, za + 2.0, "sweep"))
                ops.sort()
                self.wall_ops(f, za, zb, wall, ops)
                for u0, u1, oz0, oz1, kind in ops:
                    if kind == "door":
                        dd = self.depth(f, u0 - 0.35, u1 + 0.35, 0.75, low=0.4)
                        if dd > 0:          # entrance canopy / 玄関庇
                            f.block(trim, u0 - 0.35, u1 + 0.35, 0.0, dd, oz1 + 0.25, oz1 + 0.33)
                    elif kind == "sweep" or (kind == "win" and u1 - u0 > 1.5 and fi == 0):
                        # shutter box / シャッターボックス
                        f.block("jp_alu", u0 - 0.05, u1 + 0.05, 0.0, 0.18, oz1 + 0.02, oz1 + 0.25, bottom=True)
                if balc:
                    u0, u1, dd = balc
                    f.block("jp_foundation", u0, u1, 0.0, dd, za - 0.15, za + 0.05)
                    pm = rng.choice([wall, trim, "jp_rail_alu"])
                    f.block(pm, u0, u1, dd - 0.08, dd, za + 0.05, za + 1.05, bottom=False)
                    for uu in (u0, u1 - 0.08):
                        f.block(pm, uu, uu + 0.08, 0.0, dd - 0.08, za + 0.05, za + 1.05, bottom=False, ends=True)
                # floor band between storeys / 水切り
                if fi >= 1:
                    f.block(trim, 0.0, L, 0.0, 0.04, za - 0.06, za + 0.04, ends=False)
                if fi == 0 and not frontish and L > 3.5 and rng.random() < 0.6:
                    u = rng.uniform(0.4, L - 1.3)
                    if not any(o[0] - 0.2 < u + 0.8 and o[1] + 0.2 > u for o in ops if o[4] in ("door", "sweep")):
                        ac_unit(f, u, z0)
                        pipe(f, u + 0.85, z0 + 0.4, za + 2.3, mat="jp_pipe", wd=0.1)
        # roofs / 屋根
        H_top = zf + H * lv
        for kind, R, zw in roofs:
            self.house_roof(m, it, kind, R, zw, v, wall, floors)
        # drain pipes at two corners of the main block / 縦樋
        fp = floors[-1][0]
        e = sorted(edges_of(fp), key=lambda q: -q[2])[:2]
        for a, b, L, n in e:
            f = Facade(m, a, b, n)
            pipe(f, 0.12, z0, floors[-1][2] - 0.1, mat="jp_gutter", wd=0.08)
        it.height = H_top + min(it.R.A, it.R.B) / 2 * v["pitch"]

    def house_roof(self, m, it, kind, R, zw, v, wall, floors):
        """A house roof: main, wing, near-rectangular or irregular plan (on its
        rectangle, inset where needed), lean-to. Each side gets its own eave:
        0-0.1 m (or an inset) only where it meets a road, footway or neighbour.
        Hip when every side keeps >= 0.3 m, else gable or mono-pitch with
        per-side eaves; flat only when no rectangle covers the plan.
        住宅の屋根: 辺ごとの軒の出（道路・歩道・隣棟側だけ短く）。陸屋根は最後の手段。"""
        pitch = v["pitch"]
        if kind == "lean":
            ov = self.roof_ov(R, v["ov"], high=False)
            if ov is None:
                face_out(m, "jp_foundation", [R.P(0, 0, zw), R.P(R.A, 0, zw), R.P(R.A, R.B, zw), R.P(0, R.B, zw)],
                         (0, 0, 1))
                return
            zhi = zw + min(0.75, R.B * 0.32)
            roof_mono(m, R, zhi, (zhi - zw) / R.B, ov, v["roof"])
            return
        fp = floors[-1][0] if kind in ("main", "near", "free") else R.poly()
        if R is None:
            R = Rect.of(fp)
        fit = self.fit_sides(R, v["ov"], fp, 0.5 if kind == "free" else 0.6)
        if fit is None:
            self.stats["house roof flat"] += 1
            self.B.flat_roof(m, fp, zw, wall, 0.5, mat="roof_concrete")
            return
        R, ovs, inset = fit
        if inset:
            # the plan outside the roof rectangle: a flat roof / 屋根矩形の外は陸屋根
            # flush with the walls and the roof rectangle (a 2 cm step left slits)
            # 外壁・屋根矩形と同じ高さ（2 cm の段差が隙間になった）
            for q in polys(fp.difference(R.poly())):
                if q.area > 0.05:
                    m.flat("roof_concrete", q, zw, 2.0)
        face_out(m, "jp_foundation", [R.P(0, 0, zw), R.P(R.A, 0, zw), R.P(R.A, R.B, zw), R.P(0, R.B, zw)], (0, 0, 1))
        form = v["kind"]
        if form == "hip" and min(ovs) >= 0.2:
            roof_hip(m, R, zw, pitch, min(ovs), v["roof"])
        elif form == "mono":
            # low (gutter) side: the long side with the deeper eave, i.e. away from the path
            # 低い側（雨樋）は軒の深い長辺＝歩道・道路の反対側
            if ovs[2] > ovs[0]:
                R = Rect.toward([R.xy(0, 0), R.xy(R.A, 0), R.xy(R.A, R.B), R.xy(0, R.B)], -R.ay)
                ovs = (ovs[2], ovs[3], ovs[0], ovs[1])
            roof_mono_ps(m, R, zw, min(pitch, 0.25), ovs, v["roof"], wall)
        else:
            if max(ovs[0], ovs[2]) < 0.2 and min(ovs[1], ovs[3]) >= 0.3 and R.A < 1.5 * R.B:
                # ridge across: the deep eaves on the short sides / 棟を短辺方向に
                R = Rect.toward([R.xy(0, 0), R.xy(R.A, 0), R.xy(R.A, R.B), R.xy(0, R.B)], R.ax)
                ovs = (ovs[3], ovs[0], ovs[1], ovs[2])
            roof_gable_ps(m, R, zw, pitch, ovs, v["roof"], wall)
            form = "gable"
        self.stats["house roof %s" % form] += 1
        if v["roof"].startswith(METAL):
            self.stats["house roof metal"] += 1

    def fit_sides(self, R, want, fp, cov):
        """(rectangle, eaves (r0, s1, r1, s0), inset?) for a roof over R: a side
        whose eave strip meets a road, footway or neighbour steps its eave down
        (want ... 0.1, 0), then is inset 0.3 m at a time; None if the rectangle
        would cover less than cov of the plan. / 辺ごとに軒を縮め、だめなら後退。"""
        ladder = [o for o in (want, 0.75, 0.6, 0.45, 0.3, 0.2, 0.1, 0.0) if o <= want]
        idx = [0, 0, 0, 0]
        ins = [0.0, 0.0, 0.0, 0.0]
        for _ in range(48):
            ov = [ladder[i] for i in idx]
            g = [o + (GUTTER + 0.05 if o >= 0.15 else 0.05) for o in ov]
            s0, s1, r0, r1 = ins[3], R.A - ins[1], ins[0], R.B - ins[2]
            if s1 - s0 < 3.0 or r1 - r0 < 3.0:
                return None
            strips = [R.poly(s0 - g[3], s1 + g[1], r0 - g[0], r0 + 0.3), R.poly(s1 - 0.3, s1 + g[1], r0 - g[0], r1 + g[2]),
                      R.poly(s0 - g[3], s1 + g[1], r1 - 0.3, r1 + g[2]), R.poly(s0 - g[3], s0 + 0.3, r0 - g[0], r1 + g[2])]
            bad = [k for k, q in enumerate(strips) if not self.free(q)]
            if not bad:
                if self.free(R.poly(s0 - g[3], s1 + g[1], r0 - g[0], r1 + g[2])):
                    inset = any(v > 0 for v in ins)
                    Rn = R.sub(s0, s1, r0, r1) if inset else R
                    if Rn.poly().intersection(fp).area < cov * fp.area:
                        return None
                    return Rn, tuple(ov), inset or fp.difference(Rn.poly().buffer(0.05)).area > 0.5
                bad = [0, 1, 2, 3]
            for k in bad:
                if idx[k] < len(ladder) - 1:
                    idx[k] += 1
                else:
                    ins[k] += 0.3
        return None

    def roof_ov(self, R, want, high=True):
        """The deepest eave <= want whose roof (plus gutter) keeps off roads and
        neighbours; None if not even 0.25 m does. / 道路・隣棟に掛からない軒の出。"""
        for ov in (want, 0.75, 0.6, 0.45, 0.3, 0.2, 0.1):
            if ov > want:
                continue
            g = ov + GUTTER + 0.05
            if self.free(R.poly(-g, R.A + g, -g, R.B + (g if high else 0.0))):
                return ov
        return None

    def b_apaato(self, m, it, rng):
        v = it.var
        wall, rail = v["wall"], v["rail"]
        z0 = RAISED_Z
        zg = z0 + 0.3
        H = 2.8
        z1, z2 = zg + H, zg + 2 * H
        lv = it.levels
        front = self.front(it.poly)
        es = edges_of(it.poly)
        longest = max(e[2] for e in es)
        cand = [e for e in es if e[2] > 0.6 * longest]
        # the corridor side needs square corners: a recess on an edge with an
        # acute corner runs out through the neighbouring wall
        # 廊下側は直角の辺のみ（鋭角の角では後退した壁が外へ抜ける）
        cand = [e for e in cand if square_ends(it.poly, e)] or [None]
        ce = max(cand, key=lambda e: float(e[3] @ front)) if cand[0] is not None else (None,)
        D = 1.2
        for a, b, L, n in es:
            f = Facade(m, a, b, n)
            f.block("jp_foundation", 0.0, L, 0.0, 0.03, z0 - 0.02, zg, top=True, bottom=False, ends=False)
            if (a is ce[0]) and lv >= 1:
                self.apaato_corridor(m, f, L, D, z0, zg, z1, z2, lv, wall, rail, v, rng)
                continue
            back = (float(n @ ce[3]) < -0.6 if ce[0] is not None else float(n @ SOUTH) > 0.6) and L > 5.0
            units = bays(L, 4.5, 0.3)
            for k in range(lv):
                za, zb = zg + k * H, zg + (k + 1) * H
                if back:
                    ops = []
                    for u0, u1 in units:
                        mid = (u0 + u1) / 2
                        ops.append((mid - 0.85, mid + 0.85, za + 0.05, za + 2.0, "sweep"))
                    self.wall_ops(f, za, zb, wall, ops)
                    for u0, u1 in units:
                        mid = (u0 + u1) / 2
                        if k == 0:
                            ac_unit(f, u1 - 1.0, z0)
                        else:
                            dd = self.depth(f, u0 + 0.1, u1 - 0.1, 0.9, low=0.6)
                            if dd > 0:
                                f.block("jp_foundation", u0 + 0.1, u1 - 0.1, 0.0, dd, za - 0.15, za + 0.05)
                                f.block(rail, u0 + 0.1, u1 - 0.1, dd - 0.05, dd, za + 0.05, za + 1.05, bottom=False)
                                ac_unit(f, u1 - 1.0, za + 0.05, w0=0.1)
                        f.block("jp_alu", mid - 0.9, mid + 0.9, 0.0, 0.18, za + 2.02, za + 2.25, bottom=True)
                else:
                    ops = [((u0 + u1) / 2 - 0.4, (u0 + u1) / 2 + 0.4, za + 1.0, za + 1.9, "frosted")
                           for u0, u1 in bays(L, 3.5, 0.6)]
                    self.wall_ops(f, za, zb, wall, ops)
                if k >= 1:
                    f.block(v["rail"], 0.0, L, 0.0, 0.04, za - 0.06, za + 0.04, ends=False)
        self.gable_or_flat(m, it, z2, 0.3, 0.6, v["roof"], wall)
        it.height = z2 + min(it.R.A, it.R.B) / 2 * 0.3 + 0.3

    def apaato_corridor(self, m, f, L, D, z0, zg, z1, z2, lv, wall, rail, v, rng):
        """The corridor side: walls set back D, slab, railing, doors, the
        external stair. / 共用廊下側（後退した壁・床版・手すり・扉・外階段）。"""
        n_units = max(2, int(round((L - 1.5) / 4.6)))
        run = 3.6
        sdepth = self.depth(f, 0.0, L, 1.0, low=0.9)
        st = sdepth > 0 and L > run + 3.0
        if v["stair_end"]:
            u_land0, u_land1 = L - 1.3, L - 0.15
        else:
            u_land0, u_land1 = 0.15, 1.3
        H = z1 - zg
        for k in range(lv):
            za, zb = zg + k * H, zg + (k + 1) * H
            f.face(wall, 0.0, L, za, zb, -D)
            usable = (1.4, L - 1.4) if st else (0.2, L - 0.2)
            w = (usable[1] - usable[0]) / n_units
            for j in range(n_units):
                u = usable[0] + j * w
                f.face("jp_door_steel", u + 0.3, u + 1.15, za + 0.05, za + 2.05, -D + 0.02)
                f.face("jp_glass_frosted", u + 1.6, u + 2.4, za + 1.0, za + 1.9, -D + 0.02)
                for kk in range(1, 4):
                    uu = u + 1.6 + 0.2 * kk
                    f.block("jp_alu", uu - 0.015, uu + 0.015, -D + 0.02, -D + 0.07, za + 1.0, za + 1.9,
                            ends=False)
                f.block("jp_metal_white", u + 2.6, u + 3.0, -D, -D + 0.15, za + 1.2, za + 1.75)   # meter box / メーター
            # the corridor floor (ground: a walkway; upper: the slab) / 廊下床
            if k == 0:
                f.block("jp_corridor", 0.0, L, -D, 0.0, z0, za, top=True, bottom=False, ends=False)
            else:
                f.block("jp_corridor", 0.0, L, -D, 0.0, za - 0.2, za, top=True, bottom=True, ends=False)
                r0, r1 = (0.1, u_land0) if (st and v["stair_end"]) else ((u_land1 if st else 0.1), L - 0.1)
                f.block(rail, r0, r1, -0.06, 0.0, za, za + 1.1, top=True, bottom=False)
            # the corridor's ceiling / 廊下の天井
            f.hplane("jp_soffit", 0.0, L, -D, 0.0, zb - 0.001, False)
        # columns at the corridor ends / 端部の柱
        for u in (0.0, L - 0.25):
            f.block(wall, u, u + 0.25, -0.25, 0.0, z0, z2, top=False, bottom=False)
        if st:
            d = sdepth
            if v["stair_end"]:
                ub = u_land0 - run
                stair(m, f, ub, run, 0.08, d, z0, z1)
            else:
                # mirrored: rise toward -u / 逆向き
                ub = u_land1
                fm = Facade(m, np.asarray(f.P(L, 0, 0)[:2]), np.asarray(f.P(0, 0, 0)[:2]), -f.n)
                stair(m, fm, L - (ub + run), run, -d, -0.08, z0, z1, open_side=-1)
            # landing and its posts / 踊り場と柱
            f.block("jp_metal_white", u_land0, u_land1, -0.02, d, z1 - 0.15, z1, top=True, bottom=True)
            f.block(rail, u_land0, u_land1, d - 0.05, d, z1, z1 + 1.1, bottom=False)
            for uu in (u_land0, u_land1 - 0.1):
                f.block("jp_metal_white", uu, uu + 0.1, d - 0.12, d - 0.02, z0, z1 - 0.15, top=False, bottom=False)

    def gable_or_flat(self, m, it, zw, pitch, want, mat, wall):
        R = it.R
        ov = self.roof_ov(R, want) if it.shape == "rect" else None
        face_out(m, "jp_foundation", [R.P(0, 0, zw), R.P(R.A, 0, zw), R.P(R.A, R.B, zw), R.P(0, R.B, zw)],
                 (0, 0, 1)) if ov is not None else None
        if ov is None:
            self.B.flat_roof(m, it.poly, zw, wall, 0.6, mat="roof_concrete")
        else:
            roof_gable(m, R, zw, pitch, ov, mat, wall)

    def b_danchi(self, m, it, rng):
        v = it.var
        wall, acc = v["wall"], v["acc"]
        z0 = RAISED_Z
        zg = z0 + 0.35
        H = 2.75
        lv = it.levels
        zt = zg + lv * H
        es = edges_of(it.poly)
        longest = max(e[2] for e in es)
        south_e = max(es, key=lambda e: float(e[3] @ SOUTH) * (e[2] > 0.5 * longest))
        num = BD_number(it.t.get("name") or "")
        for a, b, L, n in es:
            f = Facade(m, a, b, n)
            f.block("jp_foundation", 0.0, L, 0.0, 0.03, z0 - 0.02, zg, top=True, bottom=False, ends=False)
            sdot = float(n @ south_e[3])
            if sdot > 0.8 and L > 8.0:
                self.danchi_balconies(m, f, L, zg, H, lv, wall, acc, rng)
            elif sdot < -0.8 and L > 8.0:
                self.danchi_stairs(m, f, L, z0, zg, H, lv, zt, wall, acc, rng)
            else:
                for k in range(lv):
                    za = zg + k * H
                    ops = [((u0 + u1) / 2 - 0.45, (u0 + u1) / 2 + 0.45, za + 1.0, za + 2.0, "frosted")
                           for u0, u1 in bays(L, 4.5, 1.2)] if L > 6 else []
                    self.wall_ops(f, za, za + H, wall, ops)
                if num and L > 6.0:
                    self.B.numbers(m, f, num, L / 2, zt - 2.0, "jp_number", size=1.5)
        self.B.flat_roof(m, it.poly, zt, wall, 0.6, mat="roof_concrete")
        it.height = zt + 1.5

    def danchi_balconies(self, m, f, L, zg, H, lv, wall, acc, rng):
        units = bays(L, 5.8, 0.4)
        dd = self.depth(f, 0.0, L, 1.1, low=0.8)
        for k in range(lv):
            za = zg + k * H
            ops = []
            for u0, u1 in units:
                mid = (u0 + u1) / 2
                ops += [(mid - 1.6, mid + 0.2, za + 0.05, za + 2.0, "sweep"), (mid + 0.6, mid + 1.8, za + 1.0, za + 2.0,
                                                                              "win")]
            self.wall_ops(f, za, za + H, wall, ops)
            if dd <= 0:
                continue
            f.block(acc, 0.0, L, 0.0, dd, za - 0.15, za, top=True, bottom=True)
            f.block(wall, 0.0, L, dd - 0.12, dd, za, za + 0.5, top=True, bottom=False, ends=True)
            f.block("jp_metal_white", 0.0, L, dd - 0.06, dd - 0.02, za + 0.5, za + 1.1, top=True, bottom=True,
                    ends=True)
            for u0, u1 in units:
                f.block(wall, u1 - 0.04, u1 + 0.04, 0.0, dd - 0.12, za, za + 2.2, top=True, bottom=False)
                f.block("jp_metal_dark", u0 + 0.3, u1 - 0.3, dd - 0.35, dd - 0.31, za + 1.85, za + 1.89, ends=True)
                ac_unit(f, u1 - 1.0, za, w0=0.1)
            # drain pipe at the balcony ends / 縦樋
        if dd > 0:
            for u in (0.15, L - 0.25):
                pipe(f, u, zg - 0.3, zg + lv * H, w0=dd - 0.25, mat="jp_pipe")

    def danchi_stairs(self, m, f, L, z0, zg, H, lv, zt, wall, acc, rng):
        nc = max(1, int(round(L / 12.0)))
        cores = [L * (j + 0.5) / nc for j in range(nc)]
        cw = 2.6
        dd = 0.3
        for k in range(lv):
            za = zg + k * H
            ops = []
            for j in range(nc):
                a, b = j * L / nc, (j + 1) * L / nc
                for u in (a + (b - a) * 0.17, b - (b - a) * 0.17):
                    if all(abs(u - c) > cw / 2 + 0.9 for c in cores):
                        ops.append((u - 0.6, u + 0.6, za + 1.0, za + 2.0, "frosted"))
            self.wall_ops(f, za, za + H, wall, ops)
        for c in cores:
            u0, u1 = c - cw / 2, c + cw / 2
            f.block(acc, u0, u1, 0.0, dd, z0, zt + 1.0, top=True, bottom=False)
            # entrance at the foot, landing slots above (dark openings in front) / 入口と踊り場の開口
            f.face("jp_dark", u0 + 0.5, u1 - 0.5, z0, zg + 2.1, dd + 0.02)
            ca = self.depth(f, u0, u1, 0.8, w0=dd, low=0.4)
            if ca > 0:
                f.block("jp_foundation", u0 - 0.1, u1 + 0.1, dd, dd + ca, zg + 2.2, zg + 2.32)
            for k in range(lv):
                zl = zg + k * H + H * 0.5 + 0.9
                if zl + 1.1 < zt:
                    f.face("jp_dark", u0 + 0.35, u1 - 0.35, zl, zl + 1.1, dd + 0.02)
                    f.block(wall, u0 + 0.3, u1 - 0.3, dd, dd + 0.08, zl - 0.1, zl + 0.05)
        if cores:
            # elevated water tank on one stair head / 高架水槽
            P = f.P(cores[0], -1.6, 0)
            ang = math.atan2(f.t[1], f.t[0])
            m.box("jp_concrete_white", P[0], P[1], zt + 1.0, zt + 2.6, 2.4, 2.0, ang)

    def b_mansion(self, m, it, rng):
        v = it.var
        wall, acc = v["wall"], v["acc"]
        z0 = RAISED_Z
        zg = z0 + 0.4
        H = 3.0
        lv = it.levels
        zt = zg + lv * H
        es = edges_of(it.poly)
        longest = max(e[2] for e in es)
        bal_e = max(es, key=lambda e: float(e[3] @ SOUTH) * e[2] + 0.01 * e[2])
        front = self.front(it.poly)
        ent = max(es, key=lambda e: float(e[3] @ front) * min(e[2], 10.0))
        for a, b, L, n in es:
            f = Facade(m, a, b, n)
            f.block("jp_foundation", 0.0, L, 0.0, 0.03, z0 - 0.02, zg, top=True, bottom=False, ends=False)
            sdot = float(n @ bal_e[3])
            if sdot > 0.8 and L > 6.0:
                self.mansion_balconies(m, f, L, zg, H, lv, wall, acc, v, rng)
            elif sdot < -0.8 and L > 8.0 and lv >= 3 and square_ends(it.poly, (a, b, L, n)):
                self.mansion_corridor(m, f, L, zg, H, lv, wall, acc, rng)
            elif it.t.get("jp:gable") == "blank":
                # a blank end wall (tag jp:gable=blank, as photographed): plain, in the
                # accent colour, from the foundation to the roof; 3 cm proud, so it is not
                # coplanar with the open corridor's end faces (black in Cycles)
                # 窓の無い妻壁（タグ指定）。開放廊下の端面と同一平面にしない（3 cm 外）
                f.face(acc, 0.0, L, zg, zg + lv * H, 0.03)
            else:
                for k in range(lv):
                    za = zg + k * H
                    ops = [((u0 + u1) / 2 - 0.6, (u0 + u1) / 2 + 0.6, za + 0.9, za + 2.2, "win")
                           for u0, u1 in bays(L, 3.6, 0.8)]
                    self.wall_ops(f, za, za + H, wall, ops)
                    f.block(acc, 0.0, L, 0.0, 0.05, za - 0.05, za + 0.1, ends=False)
            if a is ent[0] and L > 5.0:
                # entrance canopy / エントランスの庇
                u = L / 2
                ca = self.depth(f, u - 2.0, u + 2.0, 1.5, low=0.6)
                if ca > 0:
                    f.block(acc, u - 2.0, u + 2.0, 0.0, ca, zg + 2.6, zg + 2.85, bottom=True)
                f.face("glass_dark", u - 1.5, u + 1.5, zg, zg + 2.4, 0.05)
        self.B.flat_roof(m, it.poly, zt, wall, 1.0, mat="roof_concrete")
        if it.t.get("roof:solar"):
            roof_solar(m, it.R, zt, it.t["roof:solar"])
        c = it.poly.centroid
        if lv >= 4 and it.poly.buffer(-2.5).contains(c):
            ang = math.atan2(es[0][1][1] - es[0][0][1], es[0][1][0] - es[0][0][0])
            m.box(acc, c.x, c.y, zt, zt + 3.2, 3.6, 3.0, ang)        # lift-motor room / エレベーター機械室
        it.height = zt + (4.2 if lv >= 4 else 1.0)

    def mansion_balconies(self, m, f, L, zg, H, lv, wall, acc, v, rng):
        units = bays(L, 6.4, 0.3)
        dd = self.depth(f, 0.0, L, 1.4, low=1.0)
        for k in range(lv):
            za = zg + k * H
            ops = []
            for u0, u1 in units:
                mid = (u0 + u1) / 2
                ops += [(u0 + 0.5, mid - 0.2, za + 0.05, za + 2.2, "sweep"), (mid + 0.4, u1 - 0.5, za + 0.05,
                                                                              za + 2.2, "sweep")]
            self.wall_ops(f, za, za + H, wall, ops)
            if dd <= 0 or k == 0:
                continue
            f.block(acc, 0.0, L, 0.0, dd, za - 0.2, za, top=True, bottom=True)
            pm = {"solid": wall, "glass": "jp_glass_frosted", "rail": "jp_rail_alu"}[v["bal"]]
            f.block(pm, 0.0, L, dd - 0.1, dd, za, za + 1.1, top=True, bottom=False)
            for u0, u1 in units:
                f.block("jp_glass_frosted", u1 - 0.03, u1 + 0.03, 0.0, dd - 0.1, za, za + 2.0, top=True)
                ac_unit(f, u1 - 1.0, za, w0=0.15)
        if dd > 0:
            for u in (0.2, L - 0.3):
                pipe(f, u, zg, zg + lv * H, w0=dd - 0.3)

    def mansion_corridor(self, m, f, L, zg, H, lv, wall, acc, rng):
        D = 1.3
        for k in range(lv):
            za, zb = zg + k * H, zg + (k + 1) * H
            if k == 0:
                ops = [((u0 + u1) / 2 - 0.5, (u0 + u1) / 2 + 0.5, za + 1.0, za + 2.0, "frosted")
                       for u0, u1 in bays(L, 4.0, 0.8)]
                self.wall_ops(f, za, zb, wall, ops)
                continue
            f.face(acc, 0.0, L, za - 0.2, za, 0.0)
            f.block(wall, 0.0, L, -0.15, 0.0, za, za + 1.15, top=True, bottom=False, ends=False)
            f.face(wall, 0.0, L, za + 1.15, zb - 0.2, -D)
            f.hplane("jp_soffit", 0.0, L, -D, 0.0, zb - 0.2, False)
            f.hplane("jp_corridor", 0.0, L, -D, -0.15, za, True)
            for u0, u1 in bays(L, 6.4, 0.4):
                mid = (u0 + u1) / 2
                f.face("jp_door_steel", mid - 1.4, mid - 0.5, za + 1.15, za + 2.1, -D + 0.02)
                f.face("jp_glass_frosted", mid + 0.2, mid + 1.4, za + 1.3, za + 2.0, -D + 0.02)
            for u in np.linspace(0.2, L - 0.2, max(2, int(L // 6.4) + 1)):
                f.block(wall, u - 0.2, u + 0.2, -D, 0.0, za + 1.15, zb - 0.2, top=False, bottom=False)

    def b_shop(self, m, it, rng):
        v = it.var
        wall = v["wall"]
        z0 = RAISED_Z
        zg = z0 + 0.1
        lv = it.levels
        H0, H = 3.6, 2.9
        zt = zg + H0 + (lv - 1) * H
        front = self.front(it.poly)
        public = it.t.get("amenity") in ("community_centre", "post_office", "bank", "public_bath")
        for a, b, L, n in edges_of(it.poly):
            f = Facade(m, a, b, n)
            fr = float(n @ front) > 0.55 and L > 3.0
            if fr and not public:
                f.face("shopfront", 0.0, L, zg, zg + 2.7, -0.3)
                f.hplane(wall, 0.0, L, -0.3, 0.0, zg + 2.7, False)
                f.hplane("jp_foundation", 0.0, L, -0.3, 0.0, zg, True)
                for u in (0.0, L - 0.3):
                    f.block(wall, u, u + 0.3, -0.3, 0.0, zg, zg + 2.7, top=False, bottom=False)
                f.face(wall, 0.0, L, z0, zg, 0.0)
                f.face(wall, 0.0, L, zg + 2.7, zg + H0, 0.0)
                f.block(v["sign"], 0.2, L - 0.2, 0.0, 0.15, zg + 2.85, zg + 3.5)      # sign band / 看板
                if v["awning"]:
                    dd = self.depth(f, 0.3, L - 0.3, 1.0, low=0.6)
                    if dd > 0:
                        two_sided(m, v["awning"], [f.P(0.3, 0.0, zg + 2.75), f.P(L - 0.3, 0.0, zg + 2.75),
                                                   f.P(L - 0.3, dd, zg + 2.3), f.P(0.3, dd, zg + 2.3)])
                else:
                    f.block("jp_shutter", 0.3, L - 0.3, 0.0, 0.25, zg + 2.62, zg + 2.82, bottom=True)
            else:
                ops = [((u0 + u1) / 2 - 0.7, (u0 + u1) / 2 + 0.7, zg + 1.0, zg + 2.3, "win")
                       for u0, u1 in bays(L, 3.6, 0.6)]
                if fr and public and ops:
                    u0, u1 = ops[len(ops) // 2][:2]
                    ops[len(ops) // 2] = (u0 - 0.2, u1 + 0.2, zg, zg + 2.3, "door")
                self.wall_ops(f, z0, zg + H0, wall, ops)
            for k in range(1, lv):
                za = zg + H0 + (k - 1) * H
                ops = [((u0 + u1) / 2 - 0.8, (u0 + u1) / 2 + 0.8, za + 0.9, za + 2.1, "win")
                       for u0, u1 in bays(L, 3.2, 0.5)]
                self.wall_ops(f, za, za + H, wall, ops)
                f.block("jp_foundation", 0.0, L, 0.0, 0.05, za - 0.08, za + 0.05, ends=False)
                if fr and k == lv - 1 and rng.random() < 0.5 and L > 3.5:
                    pipe(f, 0.15, z0, zt, mat="jp_gutter")
        if v["roof"] == "hip" and it.shape == "rect" and min(it.R.A, it.R.B) < 16:
            ov = self.roof_ov(it.R, 0.6)
            if ov is not None:
                R = it.R
                face_out(m, "jp_foundation", [R.P(0, 0, zt), R.P(R.A, 0, zt), R.P(R.A, R.B, zt), R.P(0, R.B, zt)],
                         (0, 0, 1))
                roof_hip(m, R, zt, 0.4, ov, v["rmat"])
                it.height = zt + R.B / 2 * 0.4
                return
        self.B.flat_roof(m, it.poly, zt, wall, 0.8, mat="roof_concrete")
        it.height = zt + 0.8

    def b_shed(self, m, it, rng):
        if it.t.get("roof:shape") == "flat" and it.t.get("height"):
            return self.b_shed_flat(m, it)
        v = it.var
        z0 = RAISED_Z
        big = it.poly.area > 120 and min(it.R.A, it.R.B) > 8.0
        H = 5.5 if big else 2.4
        front = self.front(it.poly)
        es = edges_of(it.poly)
        for a, b, L, n in es:
            f = Facade(m, a, b, n)
            f.face(v["wall"], 0.0, L, z0, z0 + H, 0.0)
            if float(n @ front) > 0.55 and L > 2.6:
                w = min(L - 0.8, 4.0 if big else 2.4)
                u = (L - w) / 2
                f.face("jp_shutter", u, u + w, z0, z0 + min(H - 0.4, 3.8 if big else 2.0), 0.03)
        if it.shape == "rect":
            R = it.R
            if big:
                ov = self.roof_ov(R, 0.5)
                if ov is not None:
                    face_out(m, "jp_foundation", [R.P(0, 0, z0 + H), R.P(R.A, 0, z0 + H), R.P(R.A, R.B, z0 + H),
                                                  R.P(0, R.B, z0 + H)], (0, 0, 1))
                    roof_gable(m, R, z0 + H, 0.15, ov, v["roof"], v["wall"])
                    it.height = z0 + H + R.B / 2 * 0.15
                    return
            else:
                ov = self.roof_ov(R, 0.3)
                if ov is not None:
                    zhi = z0 + H + R.B * 0.12
                    face_out(m, "jp_foundation", [R.P(0, 0, z0 + H), R.P(R.A, 0, z0 + H), R.P(R.A, R.B, z0 + H),
                                                  R.P(0, R.B, z0 + H)], (0, 0, 1))
                    # back wall up to the high side / 高い側の壁
                    roof_mono(m, R, zhi, 0.12, ov, v["roof"], high_ov=ov)
                    for s, e in ((0.0, -R.ax), (R.A, R.ax)):
                        face_out(m, v["wall"], [R.P(s, 0, z0 + H), R.P(s, R.B, z0 + H), R.P(s, R.B, zhi)],
                                 (e[0], e[1], 0))
                    face_out(m, v["wall"], [R.P(0, R.B, z0 + H), R.P(R.A, R.B, z0 + H), R.P(R.A, R.B, zhi),
                                            R.P(0, R.B, zhi)], (R.ay[0], R.ay[1], 0))
                    it.height = zhi
                    return
        m.flat("jp_metal_siding", it.poly, z0 + H, 2.0)
        it.height = z0 + H

    def b_shed_flat(self, m, it):
        """A garage row / store with a flat roof (tags roof:shape=flat, height=m):
        closed walls, a coping, roll-up shutters every 3 m on the side(s) named
        by jp:doors (n/s/e/w), none elsewhere. / 陸屋根の車庫棟（シャッターは jp:doors 側のみ）。"""
        v = it.var
        z0 = RAISED_Z
        H = max(2.2, float(str(it.t["height"]).split()[0]))
        doors = {"n": (0.0, 1.0), "s": (0.0, -1.0), "e": (1.0, 0.0), "w": (-1.0, 0.0)}
        want = [np.array(doors[c]) for c in str(it.t.get("jp:doors", ""))if c in doors]
        for a, b, L, n in edges_of(it.poly):
            f = Facade(m, a, b, n)
            f.face(v["wall"], 0.0, L, z0, z0 + H, 0.0)
            f.block(v.get("acc", v["wall"]), 0.0, L, 0.0, 0.06, z0 + H - 0.25, z0 + H + 0.12, top=True, ends=False)
            if any(float(n @ d) > 0.7 for d in want):
                for u0, u1 in bays(L, 3.0, 0.4):
                    f.face("jp_shutter", u0 + 0.25, u1 - 0.25, z0, z0 + min(H - 0.5, 2.3), 0.03)
        m.flat("roof_concrete", it.poly, z0 + H + 0.1, 2.0)
        it.height = z0 + H + 0.12

    # ------------------------------------------------------------ site
    def site(self, m, it):
        """The lot terrace: its top (around the footprint), retaining walls down
        (or up) to the natural ground, the lot wall and gate. Local z (the
        building is shifted by it.zb afterwards). / 造成面・擁壁・塀と門。"""
        top = RAISED_Z
        lot = it.lot
        mat_top = "jp_lot"
        m.flat(mat_top, lot.difference(it.poly.buffer(-0.05, join_style="mitre")), top, 2.0)
        if self.D is None and self.L is None:
            return
        house = it.style in ("house", "apaato")
        ring = list(orient(lot, 1.0).exterior.coords)
        # vertices every <= 1.5 m / 1.5 m ごとの頂点
        pts = []
        for a, b in zip(ring, ring[1:]):
            if math.dist(a, b) < 0.03:
                continue                # (a tiny segment: its normal would fold the batter) / 微小な辺は飛ばす
            n = max(1, int(math.ceil(math.dist(a, b) / 1.5)))
            for k in range(n):
                pts.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
        P = np.array(pts)
        N = len(P)
        if N < 3:
            return
        # outward vertex normals (mitred) / 外向きの頂点法線
        E = np.roll(P, -1, axis=0) - P
        En = E / np.maximum(np.linalg.norm(E, axis=1), 1e-9)[:, None]
        On = np.column_stack([En[:, 1], -En[:, 0]])
        Vn = On + np.roll(On, 1, axis=0)
        Vn /= np.maximum(np.linalg.norm(Vn, axis=1), 1e-9)[:, None]
        cosh = np.maximum((Vn * On).sum(axis=1), 0.35)
        Vn = Vn / cosh[:, None]
        g = (self.D.z(P[:, 0], P[:, 1]) if self.D is not None else np.zeros(N)) - it.zb + RAISED_Z
        # a neighbouring terrace across the boundary (its top, local) / 隣の造成面
        nb = np.full(N, np.nan)
        nbj = np.full(N, -1)
        # per vertex too (the cut wall's top follows what is really beside it) and
        # any lot within 0.3 m of a segment (its cut wall then stands 5 cm inside)
        # 頂点ごとの隣の高さ、0.3 m 以内の敷地（切土擁壁を 5 cm 内側へ）
        nbv = np.full(N, np.nan)
        near = np.zeros(N, bool)
        nbm = np.full(N, np.nan)
        if self.L is not None:
            for a_, b_ in zip(*self.lot_tree.query(shapely.points(P), predicate="dwithin", distance=0.1)):
                j = int(self.lot_ids[b_])
                if j != it.i and j in self.items:
                    nbv[a_] = np.nanmax([nbv[a_], self.items[j].zb - it.zb + RAISED_Z])
            # a lot beside the segment's middle / 辺の中点の横の敷地
            for a_, b_ in zip(*self.lot_tree.query(shapely.points((P + np.roll(P, -1, axis=0)) / 2),
                                                   predicate="dwithin", distance=0.1)):
                j = int(self.lot_ids[b_])
                if j != it.i and j in self.items:
                    nbm[a_] = np.nanmax([nbm[a_], self.items[j].zb - it.zb + RAISED_Z])
            segs = shapely.linestrings(np.stack([P, np.roll(P, -1, axis=0)], axis=1))
            for a_, b_ in zip(*self.lot_tree.query(segs, predicate="dwithin", distance=0.3)):
                if int(self.lot_ids[b_]) != it.i:
                    near[a_] = True
        mids = shapely.points((P + np.roll(P, -1, axis=0)) / 2 + On * 0.05)   # adjacent lots only (a gap needs its own walls) / 隣接する敷地のみ
        hit = self.lot_tree.query(mids, predicate="within") if self.L is not None else np.zeros((2, 0), int)
        for a_, b_ in zip(*hit):
            j = int(self.lot_ids[b_])
            if j != it.i and j in self.items:
                nb[a_] = self.items[j].zb - it.zb + RAISED_Z
                nbj[a_] = j
        # houses: stone (ishigaki) where the terrace stands more than 1 m over the ground
        # 戸建て: 1 m を超える盛土は石積み
        stone = it.style == "house" and float(np.max(top - np.where(np.isnan(nb), g, nb))) > 1.0
        it.wall_max = float(max(0.0, np.max(top - g)))
        it.stone = stone
        wall_mat = "jp_ishigaki" if stone else "jp_retaining"
        # the stone batter never reaches a road, path, drive or car park
        # 石積みの勾配の根元は道路・歩道等に出さない
        room = np.full(N, 0.5)
        if self.L is not None:
            room = np.clip(shapely.distance(self.hard, shapely.points(P)) - 0.15, 0.0, 0.5)
        # texture u: distance along the lot edge, continuous round the ring / 周長方向の u
        S = np.r_[0.0, np.cumsum(np.linalg.norm(E, axis=1))]
        # batter per vertex (shared by both segments, so no cracks): none at a
        # reflex corner (the two toes would open a wedge) or next to another
        # terrace / 勾配は頂点ごと（入隅・隣接造成面の横では垂直）
        reflex = (np.roll(En, 1, axis=0)[:, 0] * En[:, 1] - np.roll(En, 1, axis=0)[:, 1] * En[:, 0]) < -1e-6
        adjnb = ~np.isnan(nb) | ~np.isnan(np.roll(nb, 1))
        if self.L is not None:
            # and none near another lot (two battered walls would cross in the wedge between)
            # 他の敷地の近く（0.6 m 以内）でも垂直（くさび状の隙間で勾配面が交差する）
            pa, pb = self.lot_tree.query(shapely.points(P), predicate="dwithin", distance=0.6)
            for a_, b_ in zip(pa, pb):
                if int(self.lot_ids[b_]) != it.i:
                    adjnb[a_] = True
        vlen = np.maximum(np.linalg.norm(Vn, axis=1), 1.0)
        batv = np.where(stone & ~reflex & ~adjnb, np.minimum(0.2 * np.clip(top - g, 0, None), room / vlen), 0.0)
        FOOT = 0.4                     # walls reach this far under the ground / 根入れ
        for k in range(N):
            k2 = (k + 1) % N
            a, b = P[k], P[k2]
            sa, sb = S[k], S[k + 1]
            nbk = nb[k]
            ga, gb = g[k], g[k2]
            # (always both, from the natural ground too: a neighbour found at the
            # segment's middle may leave a gap at its ends; where the lots share
            # the edge the extra wall lies hidden under the neighbour's terrace)
            # 隣接していても自然地盤まで閉じる（端に隙間が残ることがある）
            # fill: from under the lower ground (or the lower terrace) up to the terrace
            # 盛土側: 低い地面（または低い隣の造成面）の下から造成面まで
            lowa = ga if np.isnan(nbk) else min(ga, nbk)
            lowb = gb if np.isnan(nbk) else min(gb, nbk)
            if max(top - lowa, top - lowb) > 0.02:
                A0 = a + Vn[k] * batv[k]
                B0 = b + Vn[k2] * batv[k2]
                if self.D is not None:
                    za = float(self.D.z(A0[0], A0[1])) - it.zb + RAISED_Z
                    zb_ = float(self.D.z(B0[0], B0[1])) - it.zb + RAISED_Z
                    if not np.isnan(nbk):
                        za, zb_ = min(za, nbk), min(zb_, nbk)
                else:
                    za, zb_ = lowa, lowb
                za, zb_ = min(za - FOOT, top - 0.05), min(zb_ - FOOT, top - 0.05)
                # v: height above the wall's foot, so stains and moss follow the ground / v: 擁壁の根元からの高さ
                m.quad(wall_mat, (A0[0], A0[1], za), (B0[0], B0[1], zb_), (b[0], b[1], top), (a[0], a[1], top),
                       (sa, 0.0), (sb, 0.0), (sb, top - zb_), (sa, top - za))
            # cut: the bank (or a higher terrace) above, held back and facing this
            # terrace; next to a terrace it stands 5 cm inside, so it never shares
            # the plane of that terrace's own fill wall
            # 切土側: 上の地面（または高い隣の造成面）を押さえる擁壁（隣接時は 5 cm 内側）
            # up to the neighbouring terrace where one is beside this vertex (the
            # ground there is cut away), else up to the natural ground
            # 隣の造成面がある頂点ではその高さまで（地面は削られている）、無ければ自然地盤まで
            along = not np.isnan(nbm[k])
            hia = ga if np.isnan(nbv[k]) else (nbv[k] if along else max(ga, nbv[k]))
            hib = gb if np.isnan(nbv[k2]) else (nbv[k2] if along else max(gb, nbv[k2]))
            if max(hia - top, hib - top) > 0.02:
                inset = near[k] or not np.isnan(nbk)
                ai, bi = (a - On[k] * 0.05, b - On[k] * 0.05) if inset else (a, b)
                ta, tb = max(hia, top) + 0.04, max(hib, top) + 0.04
                m.quad("jp_retaining", (bi[0], bi[1], top - 0.05), (ai[0], ai[1], top - 0.05),
                       (ai[0], ai[1], ta), (bi[0], bi[1], tb),
                       (-sb, 0.0), (-sa, 0.0), (-sa, ta + 0.05 - top), (-sb, tb + 0.05 - top))
        if house:
            self.lot_wall(m, it, P, g, nb, nbj, top)

    def lot_wall(self, m, it, P, g, nb, nbj, top):
        """Block wall (or hedge / low wall with a fence) round a house lot, a
        gap with piers and a gate leaf where it faces the road. / 塀と門。"""
        v = it.var
        kind = v.get("lotwall", "block")
        h = {"block": 1.2, "block_fence": 0.6, "hedge": 1.1, "low": 0.5}[kind]
        mat = "jp_hedge" if kind == "hedge" else "jp_block_wall"
        lot = it.lot
        N = len(P)
        keep = []
        others = []
        if self.L is not None:
            for j in self.tree.query(lot.buffer(1.0), predicate="intersects"):
                j = int(j)
                if j != it.i and j in self.items:
                    others.append(self.items[j].poly)
        near = unary_union([it.poly] + others).buffer(0.7) if others else it.poly.buffer(0.7)
        for k in range(N):
            k2 = (k + 1) % N
            if g[k] > top + 0.25 or g[k2] > top + 0.25:
                continue                       # cut side: the retaining wall is there / 切土側
            if not np.isnan(nb[k]) and (nb[k] > top + 0.02 or (abs(nb[k] - top) <= 0.02 and nbj[k] < it.i)):
                continue                       # the higher neighbour's wall (one wall per boundary) / 境界に塀は 1 枚
            keep.append(LineString([P[k], P[k2]]))
        if not keep:
            return
        line = shapely.line_merge(unary_union(keep)).difference(near)
        # the gate: on the side nearest the road / 門: 道路に最も近い辺
        if self.L is not None:
            c = it.poly.centroid
            q = self.L.carriageway.boundary.interpolate(self.L.carriageway.boundary.project(c))
            gp = lot.exterior.interpolate(lot.exterior.project(q))
            gw = v.get("gate_w") or (4.5 if it.style == "house" else 3.0)
            gap = gp.buffer(gw / 2)
            line = line.difference(gap)
        inset = []
        for s in getattr(line, "geoms", [line]):
            if s.geom_type != "LineString" or s.length < 0.6:
                continue
            inset.append(s)
        for s in inset:
            # the wall stands just inside the lot (0.12 m thick) / 敷地の内側に立てる
            band = s.buffer(0.14, cap_style="flat", join_style="mitre").intersection(
                lot.buffer(-0.02, join_style="mitre"))
            if band.is_empty:
                continue
            m.prism(mat, band, top - 0.02, top + h, top=True)
            if kind == "block_fence":
                m.prism("jp_alu", s.buffer(0.1, cap_style="flat").intersection(lot.buffer(-0.06))
                        .difference(lot.buffer(-0.1)), top + h, top + h + 0.6, top=True)
        if self.L is not None:
            ends = []
            for s in inset:
                for e in (s.coords[0], s.coords[-1]):
                    if Point(e).distance(gp) < gw / 2 + 0.3:
                        ends.append(e)
            for e in ends:
                m.box("jp_block_wall" if kind != "hedge" else "jp_foundation", e[0], e[1], top - 0.02,
                      top + max(h, 1.0) + 0.15, 0.35, 0.35, 0.0)
            if len(ends) == 2 and it.style == "house":
                # a pedestrian gate leaf next to one pier / 片側の門扉
                a, b = np.array(ends[0]), np.array(ends[1])
                d = (b - a) / max(np.linalg.norm(b - a), 1e-6)
                p0, p1 = a + d * 0.2, a + d * 1.1
                leaf = LineString([p0, p1]).buffer(0.025, cap_style="flat")
                if lot.buffer(-0.01).contains(leaf):        # never across a corner of the lot / 敷地の角を横切らない
                    m.prism("jp_alu", leaf, top, top + 1.1, top=True)


def clean_ring(p, keep):
    """p without vertices closer than 5 cm to the previous one (tiny segments
    turn the mitred wall normals round and fold the stone batter back on
    itself: duplicate, flickering walls), still holding ``keep``.
    5 cm 未満の短い辺を除く（擁壁の勾配が折り返して重複するため）。"""
    c = np.asarray(p.exterior.coords)[:-1]
    keep_pts = [c[0]]
    for v in c[1:]:
        if np.hypot(*(v - keep_pts[-1])) >= 0.05:
            keep_pts.append(v)
    if len(keep_pts) > 1 and np.hypot(*(keep_pts[-1] - keep_pts[0])) < 0.05:
        keep_pts.pop()
    if len(keep_pts) < 3:
        return p
    q = Polygon(keep_pts, [r for r in p.interiors])       # holes (an enclosed footway) kept / 穴は残す
    q = q if q.is_valid else q.buffer(0)
    q = unary_union([q, keep]).buffer(0)
    parts = [g for g in polys(q) if g.intersects(keep)]
    return max(parts, key=lambda g: g.area) if parts else p


def square_ends(poly, e, tol=0.2):
    """True if both corners of edge e of poly are near right angles. / 辺の両端が直角に近いか。"""
    a, b = np.asarray(e[0]), np.asarray(e[1])
    t = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    c = np.asarray(orient(poly, 1.0).exterior.coords)[:-1]
    ok = 0
    for q, sgn in ((a, -1), (b, 1)):
        k = int(np.argmin(np.hypot(*(c - q).T)))
        o = c[(k + sgn) % len(c)] - c[k]
        if np.linalg.norm(o) < 1e-6:
            return False
        if abs(float(o @ t)) / np.linalg.norm(o) < tol:
            ok += 1
    return ok == 2


class _Tree(object):
    """STRtree returning building indices. / 建物番号を返す空間索引。"""

    def __init__(self, tree, ids):
        self.t, self.ids = tree, ids

    def query(self, g, predicate=None):
        r = self.t.query(g, predicate=predicate)
        return self.ids[r]


def BD_number(name):
    """The block number painted on a danchi end wall, from its name (共400 ->
    400, 県営87-2A-1 -> 2A-1). / 団地の妻壁の棟番号。"""
    import re
    toks = re.findall(r"[0-9A-Za-z]+(?:-[0-9A-Za-z]+)*", name)
    if not toks:
        return None
    t = toks[-1]
    if len(t) > 5:
        t = t.split("-", 1)[-1] if "-" in t else t[-4:]
    return t if 0 < len(t) <= 5 else None
