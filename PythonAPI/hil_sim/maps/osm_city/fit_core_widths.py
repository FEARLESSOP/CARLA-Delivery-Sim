"""Embedded model (maps.json "core"): narrow the road logic's lanes to the
model's asphalt inside the core (carla-venv).

  carla-venv/bin/python fit_core_widths.py <build dir> [--xodr in.xodr] [--out out.xodr]
                                           [--dry-run] [--report out.json]

Defaults: <build>/<Name>.xodr in place, report <build>/core/fit_widths.json.
Run after roadnet / terrain / elevate (make_osm_city.sh does it when
meta.json has "core"), then lht_poles.py and lanes.py again.

Why: one road logic (roadnet.py, netconvert 3.0-3.4 m lanes) covers the whole
map, but inside the core the user's own model (v8) draws the asphalt. Where
its streets are narrower, the lanes ran onto its kerbs and pavements and its
buildings stood over the carriageway (user's rules 1 and 2). Guide ch. 35.

Lane layout (netconvert --lefthand, guide ch. 3 and lht_poles.py): every
direction is its own road, lanes at t in [-W, 0], the KERB at t = 0 (the
reference line), the other direction beyond t = -W. A narrowed street road
gets its reference line moved c m sideways (t = 0 stays the kerb: lht_poles,
signs.py, speed_spot, jp_signs measure t from it, and libcarla places a
signal by the reference line, not by a laneOffset) and every lane width x f.
  - the two directions of a street (each the other's partner beside
    t = -W): fitted into the model's street together; the shared centre edge
    stays unless the OSM line is off the model's street centre, then it moves
    by as little as fits (where even the narrowest lanes do not fit, the
    overflow is split between the two kerbs);
  - a two-way road without a matched partner: the centre edge stays, the
    kerb edge moves in;
  - one-way roads (nothing beside them, or a median): both edges may move in.
Per road (non-junction, samples inside the core rectangle): at every 1 m of
s the v8 asphalt (Roads + Road_Markings of core_layers.npz, x scaled by core
"x_scale", 0.1 m raster, gaps < 0.3 m closed) is followed along the lanes'
normal; the robust extent is the P_LOW-th percentile of the samples, less
MARGIN. Notches (the model's street narrower than any lane over a stretch:
a plot reaching into it) are left out of the percentile when they are under
half the samples, and reported. Constant widths: all lanes of a road get the
same factor, one value inside the core; a road crossing the core edge goes
back to its own width over TAPER m outside (smoothstep, so it keeps its
marking class outside; markings_jp classes it by the outside width), unless
its outside run is too short to taper, then it keeps the core width (floored
to 2.95 m, markings_jp's MINOR, when within MINOR_SLACK). Lanes never get narrower than
MIN_LANE (2+ lanes) / MIN_LANE_1 (one-lane roads: the core's two-way side
streets are 4.0-4.8 m of asphalt, 2.0 m a direction, as real 生活道路);
roads that would need less are reported (the model has less road there).
Only narrowing, never widening (except at the seam, below).
Seam (edge_profiles, FIT_SEAM=0 turns it off): where a road crosses the core
edge its lanes at the edge are centred on the model's asphalt band (1.5 m
inside) and widened up to it, never past the road's own widths, at most 1.5 m
sideways, the two directions as one street (a partner that does not cross,
or traffic beside a road with no matched partner: the centre edge stays; a
band much wider than the street, e.g. the other carriageway: no move); the
lanes blend from the core fit over D_in and back to the OSM road over L_out
(>= 20 m, >= 22.5 x the move: smoothstep slope <= 1:15); a run too short to
blend keeps the edge target to its end.
Junction roads fed by / feeding a changed road get the same lane edges at
their ends (laneOffset and width factor blended along s with a smoothstep),
so lane centres do not jump at junction borders. Nothing else changes.

埋め込みモデル（v8）の範囲内で、道路情報の車線幅を v8 のアスファルトに収まるよう狭める。
上下線は一緒に v8 の道路幅へ収め（中央の縁は必要な分だけ動かす）、縁石側（t = 0）は
参照線ごと内側へ寄せる（t = 0 は縁石のまま）。一方通行は両側を寄せる。交差点内の接続道路も両端の変化に
合わせて滑らかに変える。
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

RES = 0.1            # asphalt raster (m) / ラスタ解像度
CLOSE_PX = 3         # close gaps < 0.3 m in the asphalt (triangle seams) / 継ぎ目の隙間を埋める
STEP_S = 1.0         # samples along s / s 方向の間隔
STEP_T = 0.05        # march along the normal / 法線方向の刻み
P_LOW = 5.0          # robust extent: this percentile of the samples / 低いパーセンタイル
MARGIN = 0.10        # lane edge this far inside the asphalt edge / 余裕
MIN_LANE = 2.5       # narrowest lane of a road with 2+ lanes / 2 車線以上の道路の最小車線幅
MIN_LANE_1 = float(os.environ.get("FIT_MIN_LANE_1", 2.0))   # one-lane roads (narrow streets) / 1 車線の道路
MIN_SAMPLES = 3      # fewer samples inside the core: leave the road / サンプル不足は変更しない
PARTNER_FRAC = 0.5   # two-way when the other direction is beside t = -W at this share of samples
SEAM = os.environ.get("FIT_SEAM", "1") != "0"   # seam targets at the core edge / 境界の合わせ込み
TAPER = 20.0         # a road crossing the core edge widens back over this outside it / 境界外の漸変長
SEAM_WIDE = 3.0      # a band this much wider than the street holds more than it / これ以上広い帯は別物を含む
SEAM_MOVE = 1.5      # largest sideways move of a street at the seam / 境界での最大横移動
SEAM_PROBE_AT = (3.0, 4.5, 6.0, 8.0)   # band also measured this far inside (bulge test) / 内側でも測る
SEAM_BULGE = 0.5     # band at the edge this much wider than inside: a bulge / これ以上広いと膨らみ
SLOPE_K = 22.5       # blend length >= this x the lateral move (smoothstep peak slope 1:15) / 横移動の 22.5 倍以上
TAPER_END = 2.0      # and keeps this much straight width before its end / 端の手前の一定幅区間
MINOR_W = 2.95       # markings_jp: one-lane roads from this lane width are MINOR (edge lines) / MINOR の幅
MINOR_SLACK = 0.15   # floored to it outside the core when within this / この差以内なら床上げ
REFIT_TOL = 0.02     # offset paramPoly3 within this of the exact offset curve / 平行移動の近似誤差
EDGE_IN = 1.0        # samples within this of the core edge are not used / 境界付近は使わない
WALL_BAND = (0.12, 2.6)   # kept model faces at this height over its ground are walls (check_drive's band) / 壁の高さ
WALL_CLEAR = 0.5     # lanes keep this far from them (a hard limit down to the width floors; check_keep_walls.py
                     # fails the chain under 0.3 m) / 車線の離れ（幅の下限まで厳守、0.3 m 未満は check_keep_walls.py で停止）


# ---------------------------------------------------------------- v8 asphalt
def core_layers(bd, core):
    """core_layers.npz (the model's Roads / Road_Markings / Ground triangles,
    model frame), dumped with Blender as core_merge.py does when missing.
    / モデルの地面三角形（無ければ core_merge と同じく Blender で書き出す）"""
    cache = os.path.join(bd, "core_layers.npz")
    fbx = os.path.expanduser(core["fbx"])
    if not os.path.exists(cache) or os.path.getmtime(cache) < os.path.getmtime(fbx):
        import core_merge
        with tempfile.TemporaryDirectory() as td:
            sp = os.path.join(td, "dump.py")
            open(sp, "w").write(core_merge.DUMP)
            subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--",
                                   fbx, cache], stdout=subprocess.DEVNULL)
    return np.load(cache)


WALL_DUMP = r'''
import bpy, bmesh, sys, numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
lo, hi = float(a[2]), float(a[3])
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=a[0])
bpy.context.view_layer.update()
out = []
for o in bpy.data.objects:
    if o.type != "MESH" or not any(n in o.name for n in a[4:]):
        continue
    bm = bmesh.new(); bm.from_mesh(o.data); bm.transform(o.matrix_world)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    for f in bm.faces:
        z = [v.co.z for v in f.verts]
        if min(z) < hi and max(z) > lo:
            out.append([tuple(v.co) for v in f.verts])
    bm.free()
np.savez_compressed(a[1], walls=np.array(out, np.float32).reshape(-1, 3, 3))
print("[fit_core_widths] kept model walls dumped:", len(out))
'''


def keep_walls(bd, core):
    """Faces of the model objects the core keeps (the station, the railway: maps.json
    core "keep") at wheel height (WALL_BAND over its ground, z 0), model frame;
    cached in core/keep_walls.npz. / 残す建物の車輪の高さの面（駅の壁・柱）"""
    names = list(core.get("keep", ("JR_Oita_City", "Railway")))
    fbx = os.path.expanduser(core["fbx"])
    cache = os.path.join(bd, "core", "keep_walls.npz")
    if not os.path.exists(cache) or os.path.getmtime(cache) < os.path.getmtime(fbx):
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        with tempfile.TemporaryDirectory() as td:
            sp = os.path.join(td, "walls.py")
            open(sp, "w").write(WALL_DUMP)
            subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--",
                                   fbx, cache, str(WALL_BAND[0]), str(WALL_BAND[1])] + names, stdout=subprocess.DEVNULL)
    return np.load(cache)["walls"]


class Asphalt:
    """The model's asphalt as a boolean raster in the map frame (x east, y north).
    walls: the kept station / railway faces at wheel height, taken off the asphalt
    with WALL_CLEAR m round them (a station wall stood 0.3 m inside lane 4021).
    / モデルのアスファルト（地図座標のラスタ）。残す建物の壁の周り WALL_CLEAR m は除く"""

    def __init__(self, layers, rect, sx, walls=None):
        self.x0, self.y1 = rect[0] - 10.0, rect[3] + 10.0
        self.nx = int((rect[2] - rect[0] + 20.0) / RES) + 1
        self.ny = int((rect[3] - rect[1] + 20.0) / RES) + 1
        img = Image.new("L", (self.nx, self.ny), 0)
        dr = ImageDraw.Draw(img)
        for name in ("Roads", "Road_Markings"):
            if name not in layers.files:
                continue
            for t in layers[name]:
                dr.polygon([((p[0] * sx - self.x0) / RES, (self.y1 - p[1]) / RES) for p in t], fill=255)
        k = 2 * CLOSE_PX + 1
        img = img.filter(ImageFilter.MaxFilter(k)).filter(ImageFilter.MinFilter(k))
        # free: everything but the kept walls (+ WALL_CLEAR), for the hard wall limit
        # free: 残す壁（+ WALL_CLEAR）以外すべて（壁の厳密な制限用）
        free = Image.new("L", (self.nx, self.ny), 255)
        if walls is not None and len(walls):
            wpx = max(1, int(round(2 * WALL_CLEAR / RES)))
            for im in (img, free):
                dr = ImageDraw.Draw(im)
                for t in walls:
                    q = [((p[0] * sx - self.x0) / RES, (self.y1 - p[1]) / RES) for p in t]
                    dr.line(q + [q[0]], fill=0, width=wpx, joint="curve")
                    dr.polygon(q, fill=0)
        self.m = np.asarray(img) > 127
        self.free = np.asarray(free) > 127

    def on(self, x, y, grid=None):
        g = self.m if grid is None else grid
        c = np.floor((x - self.x0) / RES).astype(int)
        r = np.floor((self.y1 - y) / RES).astype(int)
        ok = (c >= 0) & (c < self.nx) & (r >= 0) & (r < self.ny)
        out = np.zeros(np.shape(x), bool) if grid is None else np.ones(np.shape(x), bool)
        out[ok] = g[r[ok], c[ok]]
        return out

    def extent(self, P, n, dmax, grid=None):
        """For points P (k, 2) and unit directions n (k, 2): the distance to the
        first cell off the asphalt (0 when P itself is off); grid=self.free: to
        the first cell within WALL_CLEAR of a kept wall. / 縁（または壁）までの距離"""
        d = np.arange(0.0, dmax + STEP_T, STEP_T)
        X = P[:, None, 0] + n[:, None, 0] * d[None, :]
        Y = P[:, None, 1] + n[:, None, 1] * d[None, :]
        on = self.on(X, Y, grid)
        off = ~on
        first = np.where(off.any(axis=1), off.argmax(axis=1), len(d))
        return np.where(first >= len(d), dmax, d[np.minimum(first, len(d) - 1)])


# ---------------------------------------------------------------- xodr helpers
def poly(el, ds):
    return (float(el.get("a", 0)) + float(el.get("b", 0)) * ds + float(el.get("c", 0)) * ds ** 2
            + float(el.get("d", 0)) * ds ** 3)


def piecewise(recs, key, s):
    """Value of a list of OpenDRIVE polynomial records (key: "s" or "sOffset") at s."""
    cur = None
    for r in recs:
        if float(r.get(key)) <= s + 1e-9 or cur is None:
            cur = r
    return poly(cur, s - float(cur.get(key))) if cur is not None else 0.0


def right_lanes(road):
    """The driving lanes of the road's (single) lane section, -1, -2, ...
    (junction roads keep them under <left> with negative ids: netconvert).
    / 車線（負の id、交差点内は <left> の下にある）"""
    ls = road.find("lanes/laneSection")
    out = [l for l in ls.iter("lane") if int(l.get("id")) < 0]
    return sorted(out, key=lambda l: -int(l.get("id")))


def lane_width(lane, ds):
    return piecewise(lane.findall("width"), "sOffset", ds)


def above(lanes, lid, ds):
    """Sum of the widths of the lanes between the reference and lane lid. / 上側の車線幅の和"""
    return sum(lane_width(l, ds) for l in lanes if int(l.get("id")) > lid)


def fit_cubic(f, s0, s1):
    """Cubic a + b ds + c ds^2 + d ds^3 through f at 4 points of [s0, s1]."""
    L = s1 - s0
    u = np.array([0.0, 1 / 3, 2 / 3, 1.0]) * L
    A = np.vander(u, 4, increasing=True)
    return np.linalg.solve(A, np.array([f(s0 + v) for v in u]))


# ---------------------------------------------------------------- measuring
def measure(m, road, asp, rect, carla):
    """Samples of one street road inside the core: two-way share, and the
    asphalt beside the lanes. / 一本の道路の計測"""
    rid = int(road.get("id"))
    L = float(road.get("length"))
    lanes = right_lanes(road)
    n = len(lanes)
    last = int(lanes[-1].get("id"))
    S = np.arange(0.5, max(L - 0.5, 0.5) + 1e-6, STEP_S)
    # plus both ends (wall limit only: a pier at a junction mouth sat past the last sample)
    # 両端も（壁の制限のみ: 交差点口の橋脚が最後のサンプルより先にあった）
    ends = [0.02, L - 0.02] if L > 1.0 else []
    rows, wrows = [], []
    for s in list(S) + ends:
        w = m.get_waypoint_xodr(rid, -1, float(s))
        wl = m.get_waypoint_xodr(rid, last, float(s))
        if w is None or wl is None:
            continue
        tf = w.transform
        rv = tf.get_right_vector()
        r = np.array([rv.x, -rv.y])                 # CARLA y south -> map y north / 南向き -> 北向き
        r /= max(np.linalg.norm(r), 1e-9)
        c1 = np.array([tf.location.x, -tf.location.y])
        top = c1 - r * w.lane_width / 2                # t = 0 (kerb on two-way roads) / t = 0
        cl = np.array([wl.transform.location.x, -wl.transform.location.y])
        W = float(np.dot(cl - top, r) + wl.lane_width / 2)
        bot = top + r * W                              # t = -W
        if not all(rect[0] + EDGE_IN < p[0] < rect[2] - EDGE_IN and rect[1] + EDGE_IN < p[1] < rect[3] - EDGE_IN
                   for p in (top, bot)):
            continue
        # the other direction beside t = -W? / t = -W の外側に対向車線があるか
        q = bot + r * 0.5
        wp = m.get_waypoint(carla.Location(x=float(q[0]), y=float(-q[1]), z=tf.location.z),
                            project_to_road=False, lane_type=carla.LaneType.Driving)
        fwd = tf.get_forward_vector()
        partner = None
        if wp is not None and wp.road_id != rid:
            f2 = wp.transform.get_forward_vector()
            if (fwd.x * f2.x + fwd.y * f2.y) < -0.5:
                partner = wp.road_id
        (wrows if s in ends else rows).append((s, top, bot, r, W, partner))
    if not rows:
        return None
    wrows = rows + wrows
    top = np.array([x[1] for x in rows]); bot = np.array([x[2] for x in rows])
    R = np.array([x[3] for x in rows]); W = np.array([x[4] for x in rows])
    hits = [x[5] for x in rows if x[5] is not None]
    two = len(hits) / len(rows)
    mate = max(set(hits), key=hits.count) if hits else None
    dmax = float(W.max()) + 3.0
    # from t = -W outward (towards the kerb) / 中央側の縁から縁石側へ
    out_from_bot = asp.extent(bot, -R, dmax)
    mid = (top + bot) / 2
    up = asp.extent(mid, -R, dmax)                     # mid -> +t side / 中央から +t 側
    down = asp.extent(mid, R, dmax)                    # mid -> -t side / 中央から -t 側
    # kept walls (station, railway piers) are a hard limit: the smallest sample, not the
    # percentile (a 1.5 m pier is a "notch" the percentile skips; lanes ran into two)
    # 残す壁・橋脚は厳密な制限: パーセンタイルではなく最小値（橋脚は切欠き扱いで無視されていた）
    wb = np.array([x[2] for x in wrows]); wR = np.array([x[3] for x in wrows])
    wm = (np.array([x[1] for x in wrows]) + wb) / 2
    wall = dict(out=float(asp.extent(wb, -wR, dmax, asp.free).min()),
                up=float(asp.extent(wm, -wR, dmax, asp.free).min()),
                down=float(asp.extent(wm, wR, dmax, asp.free).min()))
    return dict(rid=rid, n=n, L=L, W=float(np.median(W)), two=two, mate=mate, k=len(rows),
                out_from_bot=out_from_bot, up=up, down=down, Wk=W, wall=wall,
                bot_on=asp.on(bot[:, 0], bot[:, 1]), mid_on=asp.on(mid[:, 0], mid[:, 1]))


def min_w(n):
    """Narrowest total width of n lanes. / n 車線の最小幅"""
    return n * (MIN_LANE_1 if n == 1 else MIN_LANE)


def robust(v, floor):
    """P_LOW-th percentile of the samples at least ``floor`` (a notch where the
    model's street is narrower than any lane can be, e.g. a plot reaching
    into it, cannot be fitted: it is reported, not used); the share left out.
    / 低いパーセンタイル（どの車線も入らない切欠きは除外して報告）"""
    keep = v >= floor
    if keep.sum() < max(MIN_SAMPLES, 0.5 * len(v)):
        keep = np.ones(len(v), bool)
    return float(np.percentile(v[keep], P_LOW)), float(1.0 - keep.mean())


def plan_one(meas):
    """One road on its own: (hi_t, lo_t) of its lanes (old lanes: 0, -W).
    two-way without a matched partner: the centre-side edge stays, the kerb
    edge moves in; one-way: both edges may move in.
    / 単独の道路: 対面通行は中央側固定、一方通行は両側"""
    W, n = meas["W"], meas["n"]
    if meas["two"] >= PARTNER_FRAC:
        a, notch = robust(meas["out_from_bot"], min_w(n))
        need = min(a - MARGIN, meas["wall"]["out"])
        Wn = min(W, max(need, min_w(n)))
        return dict(hi=-W + Wn, lo=-W, need=need, notch=notch, kind="two-way")
    up, nu = robust(meas["up"], min_w(n) / 2)
    dn, nd = robust(meas["down"], min_w(n) / 2)
    hi = min(0.0, -W / 2 + min(up - MARGIN, meas["wall"]["up"]))
    lo = max(-W, -W / 2 - min(dn - MARGIN, meas["wall"]["down"]))
    need = hi - lo
    if need < min_w(n):                             # keep the minimum round the middle of what is there
        mid = (hi + lo) / 2
        half = min(min_w(n), W) / 2
        mid = min(max(mid, -W + half), -half)
        hi, lo = mid + half, mid - half
    return dict(hi=hi, lo=lo, need=need, notch=max(nu, nd), kind="one-way")


def plan_pair(A, B):
    """Two directions of one street (each the other's partner beside t = -W):
    both are fitted into the model's street together, the shared centre edge
    may move (where the OSM line is off the model's street centre), by as
    little as possible. u: across the street from the shared edge towards
    A's kerb. / 同じ道路の上下線を一緒に収める（中央の縁も最小限動かす）"""
    WA, WB, nA, nB = A["W"], B["W"], A["n"], B["n"]
    a, na = robust(A["out_from_bot"], 1.0)
    b, nb = robust(B["out_from_bot"], 1.0)
    a = min(a - MARGIN, A["wall"]["out"])           # kept walls: hard limit / 壁は厳密
    b = min(b - MARGIN, B["wall"]["out"])
    if a >= WA and b >= WB:
        return None, None
    if a + b >= WA + WB:
        WA2, WB2 = WA, WB
        sh = min(max(0.0, WB - b), a - WA)
    else:
        k = (a + b) / (WA + WB)
        WA2 = min(WA, max(k * WA, min_w(nA)))
        WB2 = min(WB, max(k * WB, min_w(nB)))
        lo_s, hi_s = WB2 - b, a - WA2
        sh = (lo_s + hi_s) / 2 if lo_s > hi_s else min(max(0.0, lo_s), hi_s)
    pa = dict(hi=sh + WA2 - WA, lo=sh - WA, need=a + b - (WB2 if a + b >= WA + WB else WB * (a + b) / (WA + WB)),
              notch=na, kind="pair", shift=sh)
    pb = dict(hi=WB2 - WB - sh, lo=-sh - WB, need=a + b - pa["need"], notch=nb, kind="pair", shift=-sh)
    return pa, pb


def finish(meas, p):
    """(offset c, factor f, ...) from a fitted (hi, lo), or None when nothing
    changes. c: the shift of the centre lane (t = 0), f: the width factor.
    / 参照線の移動量と幅の倍率"""
    W, n = meas["W"], meas["n"]
    Wn = p["hi"] - p["lo"]
    if abs(W - Wn) < 0.05 and abs(p["hi"]) < 0.05:
        return None
    short = p["need"] < min_w(n) - 1e-6 or p["notch"] > 0
    return dict(c=p["hi"], f=Wn / W, kind=p["kind"], W=W, Wn=Wn, need=p["need"], notch=p["notch"],
                shift=p.get("shift", 0.0), short=short)


# ---------------------------------------------------------------- writing
def _cubic_xy(A, p):
    return (A[0] + A[1] * p + A[2] * p ** 2 + A[3] * p ** 3, A[4] + A[5] * p + A[6] * p ** 2 + A[7] * p ** 3)


def offset_geometry(g, cfun, s0):
    """Move one planView geometry (old road s from s0) sideways by cfun(s) m
    (left positive); returns (the geometry elements now standing for it, max
    refit error). A line with a constant offset stays an exact line; anything
    else (a paramPoly3, or a line inside a taper) becomes cubic Hermite pieces
    through the exact offset curve, split until within REFIT_TOL.
    / 平面線形を cfun(s) m 横へ移す（一定なら直線は厳密、他は 3 次 Hermite 片で近似）"""
    x0, y0, h0, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
    ch, sh = math.cos(h0), math.sin(h0)
    el = list(g)[0]
    cs = [cfun(s0 + L * q) for q in (0.0, 0.25, 0.5, 0.75, 1.0)]
    const = max(cs) - min(cs) < 1e-6
    if el.tag == "line" and const:
        g.set("x", "%.8f" % (x0 - sh * cs[0]))
        g.set("y", "%.8f" % (y0 + ch * cs[0]))
        return [g], 0.0
    if el.tag == "line":
        A = np.array([0.0, L, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    elif el.tag == "paramPoly3":
        A = np.array([float(el.get(k)) for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")])
        if el.get("pRange", "normalized") != "normalized":           # arcLength: p in [0, L] -> [0, 1]
            A[[1, 5]] *= L
            A[[2, 6]] *= L ** 2
            A[[3, 7]] *= L ** 3
    else:
        raise ValueError("fit_core_widths: planView %s not handled" % el.tag)
    P = lambda p: np.array(_cubic_xy(A, p))
    d1 = lambda p: np.array([A[1] + 2 * A[2] * p + 3 * A[3] * p ** 2, A[5] + 2 * A[6] * p + 3 * A[7] * p ** 2])
    # p -> old s by arc length / p から弧長 s へ
    pt = np.linspace(0.0, 1.0, 401)
    xy = np.array([P(q) for q in pt])
    st = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    st *= L / st[-1] if st[-1] > 0 else 1.0

    def off(p):
        t = d1(p)
        n = np.array([-t[1], t[0]]) / max(np.hypot(*t), 1e-12)
        return P(p) + cfun(s0 + float(np.interp(p, pt, st))) * n

    def tan(p, h=1e-4):
        a_, b_ = max(p - h, 0.0), min(p + h, 1.0)
        return (off(b_) - off(a_)) / (b_ - a_)

    def piece(pa, pb):
        """Hermite of the offset curve over p in [pa, pb], origin at its start."""
        P0, P1 = off(pa), off(pb)
        T0, T1 = tan(pa) * (pb - pa), tan(pb) * (pb - pa)
        cc = 3 * (P1 - P0) - 2 * T0 - T1
        dd = 2 * (P0 - P1) + T0 + T1
        B = [0.0, T0[0], cc[0], dd[0], 0.0, T0[1], cc[1], dd[1]]
        Q = np.array([_cubic_xy(B, q) for q in np.linspace(0.0, 1.0, 101)]) + P0
        E = np.array([off(pa + (pb - pa) * q) for q in np.linspace(0.0, 1.0, 301)])
        err = float(np.max(np.min(np.hypot(Q[:, None, 0] - E[None, :, 0], Q[:, None, 1] - E[None, :, 1]), axis=1)))
        return P0, B, float(np.sum(np.hypot(np.diff(Q[:, 0]), np.diff(Q[:, 1])))), err
    for k in (1, 2, 4, 8, 16):                          # split until the refit is within 2 cm / 2 cm 以内まで分割
        parts = [piece(i / k, (i + 1) / k) for i in range(k)]
        err = max(q[3] for q in parts)
        if err < REFIT_TOL:
            break
    out = []
    for P0, B, Ln, _ in parts:
        gi = ET.Element("geometry", dict(g.attrib))
        gi.set("x", "%.8f" % (x0 + ch * P0[0] - sh * P0[1]))
        gi.set("y", "%.8f" % (y0 + sh * P0[0] + ch * P0[1]))
        gi.set("length", "%.8f" % Ln)
        attrs = {kk: "%.8f" % v for kk, v in zip(("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV"), B)}
        attrs["pRange"] = "normalized"
        ET.SubElement(gi, "paramPoly3", attrs)
        out.append(gi)
    return out, err


def profile(m, road, rect):
    """g(s) in [0, 1] along a street road: 1 where its lanes are inside the core
    rectangle, easing to 0 over TAPER m outside it (smoothstep), so a road
    crossing the core edge is narrowed only inside and keeps its width (and
    its marking class, markings_jp) outside. An outside run shorter than
    TAPER + TAPER_END keeps the core's width. None when nothing tapers.
    / コア内 1、外側 TAPER m で 0 へ（境界を越える道路は外側で元の幅）"""
    rid, L = int(road.get("id")), float(road.get("length"))
    last = min(int(l.get("id")) for l in right_lanes(road))
    S = np.arange(0.0, L + 1e-6, 0.5)
    S[-1] = min(S[-1], L - 1e-3)
    ins = np.zeros(len(S), bool)
    for i, s in enumerate(S):
        w = m.get_waypoint_xodr(rid, last, float(max(s, 1e-3)))
        if w is None:
            continue
        x, y = w.transform.location.x, -w.transform.location.y
        ins[i] = rect[0] < x < rect[2] and rect[1] < y < rect[3]
    if ins.all() or not ins.any():
        return None, 0.0
    k = np.nonzero(ins)[0]
    s_in0, s_in1 = S[k[0]], S[k[-1]]
    G = np.ones(len(S))
    # an outside run shorter than the taper keeps the core's width (no kink at
    # its junction: the taper ends with zero slope, never at a road end)
    # 外側の区間がテーパーより短ければコアの幅のまま（交差点境界で折れない）
    if s_in0 >= TAPER + TAPER_END:
        u = np.clip(1.0 - (s_in0 - S) / TAPER, 0.0, 1.0)
        G = np.where(S < s_in0, u * u * (3 - 2 * u), G)
    if L - s_in1 >= TAPER + TAPER_END:
        u = np.clip(1.0 - (S - s_in1) / TAPER, 0.0, 1.0)
        G = np.where(S > s_in1, u * u * (3 - 2 * u), G)
    out_len = 0.5 * float((~ins).sum())
    if (G == 1.0).all():
        return None, out_len
    return (lambda s: float(np.interp(s, S, G))), out_len


def _smooth(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3 - 2 * u)


def inside_mask(m, road, rect, step=0.5):
    """S grid and which samples of the road's lane set lie inside the core. / コア内のサンプル"""
    rid, L = int(road.get("id")), float(road.get("length"))
    last = min(int(l.get("id")) for l in right_lanes(road))
    S = np.arange(0.0, L + 1e-6, step)
    S[-1] = min(S[-1], L - 1e-3)
    ins = np.zeros(len(S), bool)
    for i, sv in enumerate(S):
        w = m.get_waypoint_xodr(rid, last, float(max(sv, 1e-3)))
        if w is not None:
            x, y = w.transform.location.x, -w.transform.location.y
            ins[i] = rect[0] < x < rect[2] and rect[1] < y < rect[3]
    return S, ins


def section(m, rid, s):
    """(kerb point t = 0, unit left normal, old lane widths sum W) of a road at s (map frame)."""
    lanes = sorted({w for w in range(-1, -10, -1) if m.get_waypoint_xodr(rid, w, float(s)) is not None}, reverse=True)
    if not lanes:
        return None
    w1 = m.get_waypoint_xodr(rid, -1, float(s))
    wl = m.get_waypoint_xodr(rid, lanes[-1], float(s))
    tf = w1.transform
    rv = tf.get_right_vector()
    r = np.array([rv.x, -rv.y]); r /= max(np.linalg.norm(r), 1e-9)
    top = np.array([tf.location.x, -tf.location.y]) - r * w1.lane_width / 2
    cl = np.array([wl.transform.location.x, -wl.transform.location.y])
    W = float(np.dot(cl - top, r) + wl.lane_width / 2)
    return top, -r, W


def place(a, b, tot_f, tot_o, mid, mate, half, A_hi, A_lo, WA):
    """The street's (lo, hi) edges in t that edge_profiles would target on the band
    [a, b] (same rules as there). / 帯 [a, b] での目標の両端"""
    if half:
        return A_lo, min(max(A_hi, b - MARGIN), A_lo + WA)
    T = min(max(tot_f, b - a - 2 * MARGIN), tot_o)
    mc = (a + b) / 2
    if mate is None or b - a > tot_o + SEAM_WIDE:
        mc = min(max(mid, a + MARGIN + T / 2), b - MARGIN - T / 2) if b - a - 2 * MARGIN >= T else mid
    mc = mid + float(np.clip(mc - mid, -SEAM_MOVE, SEAM_MOVE))
    return mc - T / 2, mc + T / 2


def edge_profiles(m, roads, M, changes, partner, asp, rect, report):
    """Seam targets (guide ch. 35): where a street road crosses the core edge,
    its lanes there are centred on the model's asphalt band and widened up to
    it (never past the road's own widths), blending from the core fit over
    D_in m inside and back to the OSM road over L_out m outside (smoothstep,
    lateral slope <= ~1:15). An outside or inside run too short for its
    blend keeps the edge target to its end (no blend ends in a junction).
    The two directions of a street share one target. Sets changes[rid]["cf"].
    / 境界で道路をモデルのアスファルト帯の中心に合わせ、幅も帯まで広げる（内外で滑らかに）"""
    S_in, targets, done = {}, {}, set()
    for rid in M:
        road = roads[rid]
        S, ins = inside_mask(m, road, rect)
        if ins.all() or not ins.any():
            continue
        S_in[rid] = (S, ins)
    fitcf = {}
    for rid in S_in:
        p = changes.get(rid)
        fitcf[rid] = (p["c"], p["f"]) if p else (0.0, 1.0)
    for rid, (S, ins) in S_in.items():
        if rid in done:
            continue
        mate = partner.get(rid)
        k = np.nonzero(ins)[0]
        cross = []
        if k[0] > 0:
            cross.append((S[k[0]], +1))                    # inside lies at larger s / 内側は s の大きい側
        if k[-1] < len(S) - 1:
            cross.append((S[k[-1]], -1))
        for se, sg in cross:
            sm = float(np.clip(se + sg * 1.5, 0.01, float(roads[rid].get("length")) - 0.01))
            sec = section(m, rid, sm)
            if sec is None:
                continue
            top, nl, WA = sec
            cA, fA = fitcf[rid]
            A_hi, A_lo = cA, cA - fA * WA
            if mate in S_in or mate in M:
                cB, fB = fitcf.get(mate) or ((changes[mate]["c"], changes[mate]["f"]) if mate in changes else (0.0, 1.0))
                # the partner's lane widths (old) at the matching place / 対向側の元の幅
                WB = float(M[mate]["W"])
                B_lo_A = -WA - WB - cB                        # B's kerb in A's t / A 座標での B の縁石
                B_hi_A = B_lo_A + fB * WB
                lo_f, hi_f = B_lo_A, A_hi
                tot_f, tot_o = fA * WA + fB * WB, WA + WB
            else:
                WB = 0.0
                lo_f, hi_f = A_lo, A_hi
                tot_f, tot_o = fA * WA, WA
            # its partner does not cross here, or traffic beside it with no matched
            # partner: the centre-side edge stays / 対向が境界を越えない・対向不明: 中央側は固定
            half = (mate is not None and mate not in S_in) or (mate is None and M[rid]["two"] > 0.05)
            mid = (lo_f + hi_f) / 2
            P = (top + nl * mid)[None, :]
            if not asp.on(P[:, 0], P[:, 1])[0]:
                report["seam_skipped"].append([rid, "street middle off the model's asphalt"])
                continue
            up = float(asp.extent(P, nl[None, :], 25.0)[0])
            dn = float(asp.extent(P, -nl[None, :], 25.0)[0])
            a, b = mid - dn, mid + up
            # a kerb bulge at the edge (W edge (-384, 233): 9 m of asphalt at the edge, 6 m
            # from 4.5 m in) pulled the street onto the bulge's kerb further in: where the
            # band at the edge is > SEAM_BULGE wider than its narrowest section within
            # SEAM_PROBE_AT and the target would leave that inner envelope, the envelope
            # (each side's least extent) is the band / 境界の膨らみに引かれる場合は内側の最狭部を帯とする
            Lr, ups, dns = float(roads[rid].get("length")), [], []
            for dk in SEAM_PROBE_AT:
                sk = se + sg * dk
                if not 0.01 < sk < Lr - 0.01 or not ins[min(int(np.searchsorted(S, sk)), len(S) - 1)]:
                    continue
                sec_k = section(m, rid, sk)
                if sec_k is None:
                    continue
                Pk = (sec_k[0] + sec_k[1] * mid)[None, :]
                if asp.on(Pk[:, 0], Pk[:, 1])[0]:
                    ups.append(float(asp.extent(Pk, sec_k[1][None, :], 25.0)[0]))
                    dns.append(float(asp.extent(Pk, -sec_k[1][None, :], 25.0)[0]))
            if ups and up + dn - min(u + d for u, d in zip(ups, dns)) > SEAM_BULGE:
                ea, eb = mid - min([dn] + dns), mid + min([up] + ups)
                lo_t, hi_t = place(a, b, tot_f, tot_o, mid, mate, half, A_hi, A_lo, WA)
                if lo_t < ea + MARGIN - 0.05 or hi_t > eb - MARGIN + 0.05:
                    report["seam_bulge"].append(dict(road=rid, s=round(float(se), 1), band_edge=round(b - a, 2),
                                                     band_inner=round(eb - ea, 2)))
                    a, b = ea, eb
            if half:
                # the shared centre edge stays (the other direction is not refitted
                # here): only the kerb side widens up to the band / 中央側は固定、縁石側のみ
                hiA = min(max(A_hi, b - MARGIN), A_lo + WA)
                tgt = {rid: (hiA, (hiA - A_lo) / WA)}
                T, mc = hiA - A_lo, mid
                xc = top + nl * mid
                targets.setdefault(rid, []).append((se, sg) + tgt[rid])
                report["seam"].append(dict(road=rid, mate=mate, s=round(float(se), 1), band=round(b - a, 2),
                                           street_fit=round(tot_f, 2), street_new=round(T, 2), centre_move=0.0,
                                           half=True))
                continue
            T = min(max(tot_f, b - a - 2 * MARGIN), tot_o)
            mc = (a + b) / 2
            if mate is None or b - a > tot_o + SEAM_WIDE:
                # the band holds more than this street (the other carriageway, a
                # plaza): keep the street where it is, only widen inside the band
                # 帯がこの道路より広い（反対車線・広場）: 位置はそのまま、幅だけ
                mc = min(max(mid, a + MARGIN + T / 2), b - MARGIN - T / 2) if b - a - 2 * MARGIN >= T else mid
            mc = mid + float(np.clip(mc - mid, -SEAM_MOVE, SEAM_MOVE))
            hi = mc + T / 2
            TA = T * (fA * WA) / tot_f
            tgt = {rid: (hi, TA / WA)}
            if mate in S_in:
                TB = T - TA
                kerbB = (-WA - WB) - (mc - T / 2)
                tgt[mate] = (kerbB, TB / WB)
            xc = top + nl * mid
            for r_, (ce, fe) in tgt.items():
                if r_ == rid:
                    targets.setdefault(r_, []).append((se, sg, ce, fe))
                else:
                    # the partner's crossing nearest this one / 対向側の最寄りの境界交差
                    S2, ins2 = S_in[r_]
                    k2 = np.nonzero(ins2)[0]
                    cands = []
                    if k2[0] > 0:
                        cands.append((S2[k2[0]], +1))
                    if k2[-1] < len(S2) - 1:
                        cands.append((S2[k2[-1]], -1))
                    best = None
                    for s2, g2 in cands:
                        sc = section(m, r_, float(np.clip(s2 + g2 * 1.5, 0.01, float(roads[r_].get("length")) - 0.01)))
                        if sc is None:
                            continue
                        d = float(np.linalg.norm(sc[0] - xc))
                        if best is None or d < best[0]:
                            best = (d, s2, g2)
                    if best is not None and best[0] < 20.0:
                        targets.setdefault(r_, []).append((best[1], best[2], ce, fe))
            report["seam"].append(dict(road=rid, mate=mate, s=round(float(se), 1), band=round(b - a, 2),
                                       street_fit=round(tot_f, 2), street_new=round(T, 2),
                                       centre_move=round(mc - mid, 2)))
        done.add(rid)
        if mate is not None:
            done.add(mate)
    for rid, tl in targets.items():
        S, ins = S_in[rid]
        L = float(roads[rid].get("length"))
        W = float(M[rid]["W"])
        c0, f0 = fitcf[rid]
        C = np.full(len(S), c0)
        F = np.full(len(S), f0)
        outside_set = np.zeros(len(S), bool)
        for se, sg, ce, fe in tl:
            d = sg * (S - se)                                # > 0 inside / 内側で正
            din = max(abs(ce - c0), abs((ce - fe * W) - (c0 - f0 * W)))
            dout = max(abs(ce), abs(ce - fe * W + W))
            D_in = max(TAPER, SLOPE_K * din)
            L_out = max(TAPER, SLOPE_K * dout)
            R_in = float((ins & (d >= 0)).sum()) * (S[1] - S[0])
            R_out = (se if sg > 0 else L - se)
            if R_in >= D_in + TAPER_END:
                w = _smooth(1.0 - d / D_in) * (d >= 0)
            else:
                w = (d >= 0).astype(float)
            C = np.where(d >= 0, C + (ce - c0) * w, C)
            F = np.where(d >= 0, F + (fe - f0) * w, F)
            if R_out >= L_out + TAPER_END:
                wo = _smooth(1.0 - (-d) / L_out)
            else:
                wo = np.ones(len(S))
            C = np.where(d < 0, ce * wo, C)
            F = np.where(d < 0, 1.0 + (fe - 1.0) * wo, F)
            outside_set |= d < 0
        Sc, Cc, Fc = S.copy(), C.copy(), F.copy()
        p = changes.get(rid)
        if p is None:
            p = dict(c=0.0, f=1.0, kind="seam", W=W, Wn=W, need=W, notch=0.0, shift=0.0, short=False, g=None)
            changes[rid] = p
        p["cf"] = ((lambda s, Sc=Sc, Cc=Cc: float(np.interp(s, Sc, Cc))),
                   (lambda s, Sc=Sc, Fc=Fc: float(np.interp(s, Sc, Fc))))
        p["seam"] = True



def set_street(road, c, f, g=None, cf=None):
    """A street road: its reference line moved c m to the left (so t = 0
    stays on the kerb: lht_poles, signs.py, speed_spot and jp_signs measure
    their t from there; libcarla places a signal by the reference line, not
    the laneOffset) and every lane width x f; with a profile g(s) (a road
    crossing the core edge) by c g(s) and 1 + (f - 1) g(s). A curved road's
    length changes: elevation, signal and object s are scaled with it.
    Returns the largest paramPoly3 refit error (m).
    / 参照線を左へ c m 移す（t = 0 を縁石に保つ）、車線幅を f 倍（境界を越える道路は g(s) で漸変）"""
    gg = g or (lambda s: 1.0)
    cfun = lambda s: c * gg(s)
    ffun = lambda s: 1.0 + (f - 1.0) * gg(s)
    if cf is not None:                                   # seam profiles (edge_profiles) / 境界の形状
        cfun, ffun = cf
    L0 = float(road.get("length"))
    s_acc, err = 0.0, 0.0
    pv = road.find("planView")
    geoms = []
    for g_ in list(pv.findall("geometry")):
        gs, e = offset_geometry(g_, cfun, float(g_.get("s")))
        geoms += gs
        err = max(err, e)
    for g_ in list(pv):
        pv.remove(g_)
    for g_ in geoms:
        g_.set("s", "%.8f" % s_acc)
        pv.append(g_)
        s_acc += float(g_.get("length"))
    r = s_acc / L0 if L0 > 0 else 1.0
    road.set("length", "%.8f" % s_acc)
    if abs(r - 1.0) > 1e-9:
        for e in road.findall("elevationProfile/elevation") + road.findall("lateralProfile/superelevation"):
            e.set("s", "%.6f" % (float(e.get("s")) * r))
            e.set("b", "%.10f" % (float(e.get("b", 0)) / r))
            e.set("c", "%.12f" % (float(e.get("c", 0)) / r ** 2))
            e.set("d", "%.14f" % (float(e.get("d", 0)) / r ** 3))
        for e in list(road.iter("signal")) + list(road.iter("object")) + list(road.iter("signalReference")):
            if e.get("s") is not None:
                e.set("s", "%.4f" % min(float(e.get("s")) * r, s_acc))
        for u in road.iter("userData"):
            if u.get("code") == "pole_s":
                u.set("value", "%.4f" % (float(u.get("value")) * r))
    for l in right_lanes(road):
        recs = list(l.findall("width"))
        if g is None and cf is None:
            for w in recs:
                for k in "abcd":
                    w.set(k, "%.6g" % (float(w.get(k, 0)) * f))
            continue
        wf = lambda so, recs=recs: piecewise(recs, "sOffset", so)
        at = list(l).index(recs[0])
        for w in recs:
            l.remove(w)
        n = max(1, int(math.ceil(s_acc / 2.0)))
        E = np.linspace(0.0, s_acc, n + 1)
        for i, (a_, b_) in enumerate(zip(E[:-1], E[1:])):
            co = fit_cubic(lambda sn: wf(sn / r) * ffun(sn / r), a_, b_)
            l.insert(at + i, ET.Element("width", sOffset="%.6f" % a_, a="%.6f" % co[0], b="%.8f" % co[1],
                                        c="%.8f" % co[2], d="%.8f" % co[3]))
    return err


def delta_end(road, ch, lid, at_end):
    """Shift of lane lid's left edge (t) at the road's end/start after change ch
    (c, f, profile g), and the width factor there. / 道路端での車線左縁の移動量"""
    if ch is None:
        return 0.0, 1.0
    L = float(road.get("length"))
    lanes = right_lanes(road)
    S = above(lanes, lid, L if at_end else 0.0)
    se = L if at_end else 0.0
    if ch.get("cf"):
        c, f = ch["cf"][0](se), ch["cf"][1](se)
    else:
        k = ch["g"](se) if ch.get("g") else 1.0
        c, f = ch["c"] * k, 1.0 + (ch["f"] - 1.0) * k
    return c + (1.0 - f) * S, f


def set_connector(road, d0, f0, d1, f1, j0, j1):
    """Junction road: its lane edges follow the changed roads at both ends,
    blended with a smoothstep. d: the shift of the left edge of the conn lane
    linked at that end (j0 / j1), f: the width factor there.
    / 交差点内の接続道路: 両端の変化を滑らかにつなぐ"""
    L = float(road.get("length"))
    lanes = right_lanes(road)
    lanes_el = road.find("lanes")
    los = lanes_el.findall("laneOffset")
    old_off = [(float(x.get("s")), x) for x in los]
    S0 = above(lanes, j0, 0.0)
    S1 = above(lanes, j1, L)
    c0 = d0 + (f0 - 1.0) * S0
    c1 = d1 + (f1 - 1.0) * S1
    g = lambda s: (lambda u: 3 * u * u - 2 * u ** 3)(min(max(s / L, 0.0), 1.0)) if L > 1e-6 else 1.0
    fac = lambda s: f0 + (f1 - f0) * g(s)
    off_old = lambda s: piecewise([x for _, x in old_off], "s", s) if old_off else 0.0
    off_new = lambda s: off_old(s) + c0 + (c1 - c0) * g(s)
    npc = max(1, int(math.ceil(L / 2.0)))              # ~2 m cubic pieces / 約 2 m ごとの 3 次式
    edges = np.linspace(0.0, L, npc + 1)
    # the old width records, read before they are replaced / 置き換える前の幅
    old_w = {l.get("id"): (lambda recs: (lambda s: piecewise(recs, "sOffset", s)))(list(l.findall("width")))
             for l in lanes}
    for x in los:
        lanes_el.remove(x)
    pos = 0
    for a, b in zip(edges[:-1], edges[1:]):
        co = fit_cubic(off_new, a, b)
        lanes_el.insert(pos, ET.Element("laneOffset", s="%.6f" % a, a="%.6f" % co[0], b="%.8f" % co[1],
                                        c="%.8f" % co[2], d="%.8f" % co[3]))
        pos += 1
    for l in lanes:
        wf = old_w[l.get("id")]
        recs = l.findall("width")
        at = list(l).index(recs[0])
        for w in recs:
            l.remove(w)
        for i, (a, b) in enumerate(zip(edges[:-1], edges[1:])):
            co = fit_cubic(lambda s: wf(s) * fac(s), a, b)
            l.insert(at + i, ET.Element("width", sOffset="%.6f" % a, a="%.6f" % co[0], b="%.8f" % co[1],
                                        c="%.8f" % co[2], d="%.8f" % co[3]))


def linked(road, which):
    """(road id, contact, conn lane id, linked lane id) of the first conn lane
    with a <which> link. / 最初にリンクを持つ車線"""
    el = road.find("link/" + which)
    if el is None or el.get("elementType") != "road":
        return None
    for l in right_lanes(road):
        x = l.find("link/" + which)
        if x is not None:
            return int(el.get("elementId")), el.get("contactPoint"), int(l.get("id")), int(x.get("id"))
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("build")
    ap.add_argument("--xodr")
    ap.add_argument("--out")
    ap.add_argument("--report")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    bd = os.path.abspath(a.build)
    meta = json.load(open(os.path.join(bd, "meta.json")))
    core = meta.get("core")
    if not core:
        print("[fit_core_widths] no core in meta.json: nothing to do")
        return
    import carla
    from core_merge import core_rect
    rect = core_rect(meta)
    xodr = a.xodr or os.path.join(bd, meta["name"] + ".xodr")
    out = a.out or xodr
    rep_path = os.path.abspath(a.report or os.path.join(bd, "core", "fit_widths.json"))
    asp = Asphalt(core_layers(bd, core), rect, float(core.get("x_scale", 1.0)), walls=keep_walls(bd, core))
    text = open(xodr).read()
    m = carla.Map("fit", text)
    tree = ET.ElementTree(ET.fromstring(text))
    root = tree.getroot()
    roads = {int(r.get("id")): r for r in root.findall("road")}
    changes, report = {}, {"roads": [], "short": [], "skipped": []}
    M = {}
    for rid, road in roads.items():
        if road.get("junction") != "-1" or not right_lanes(road):
            continue
        meas = measure(m, road, asp, rect, carla)
        if meas is None:
            continue
        if meas["k"] < MIN_SAMPLES:
            report["skipped"].append([rid, meas["k"]])
            continue
        M[rid] = meas
    plans = {}
    for rid, meas in M.items():
        mate = meas["mate"]
        if rid in plans or meas["two"] < PARTNER_FRAC or mate not in M:
            continue
        if M[mate]["mate"] == rid and M[mate]["two"] >= PARTNER_FRAC:
            pa, pb = plan_pair(meas, M[mate])
            plans[rid], plans[mate] = pa or "same", pb or "same"
    for rid, meas in M.items():
        if rid not in plans:
            plans[rid] = plan_one(meas)
    for rid, meas in M.items():
        p = plans[rid]
        p = finish(meas, p) if isinstance(p, dict) else None
        if p is None:
            continue
        p["g"], out_len = profile(m, roads[rid], rect)   # crossing the core edge: taper outside / 境界外は漸変
        n = meas["n"]
        if p["g"] is None and out_len >= 5.0 and n == 1 and p["W"] >= MINOR_W - 1e-6 \
                and MINOR_W - MINOR_SLACK <= p["Wn"] < MINOR_W:
            # a short run outside keeps its marking class (markings_jp MINOR >= 2.95 m)
            # 外側の短い区間: 区画線の格（MINOR）を保つ
            dW = MINOR_W - p["Wn"]
            p["c"] += dW
            p["Wn"] = MINOR_W
            p["f"] = p["Wn"] / p["W"]
        changes[rid] = p
        row = dict(road=rid, lanes=meas["n"], kind=p["kind"], W=round(p["W"], 2), W_new=round(p["Wn"], 2),
                   lane_w=round(p["Wn"] / meas["n"], 2), offset=round(p["c"], 2), centre_shift=round(p["shift"], 2),
                   samples=meas["k"], need=round(p["need"], 2), notch=round(p["notch"], 2),
                   tapered=p["g"] is not None)
        report["roads"].append(row)
        if p["short"]:
            report["short"].append(row)
    # seam: centre and widen the lanes on the model's asphalt where roads cross the core edge
    # 境界: 道路をモデルのアスファルト帯へ合わせる
    partner = {}
    for rid, meas in M.items():
        mt = meas["mate"]
        if mt in M and M[mt]["mate"] == rid and meas["two"] >= PARTNER_FRAC and M[mt]["two"] >= PARTNER_FRAC:
            partner[rid] = mt
    report["seam"], report["seam_skipped"], report["seam_bulge"] = [], [], []
    if SEAM:
        edge_profiles(m, roads, M, changes, partner, asp, rect, report)
    # junction roads next to changed roads / 変更した道路につながる交差点内の道路
    nconn = 0
    for rid, road in roads.items():
        if road.get("junction") == "-1" or not right_lanes(road):
            continue
        pr, su = linked(road, "predecessor"), linked(road, "successor")
        if pr is None or su is None:
            continue
        if pr[0] not in changes and su[0] not in changes:
            continue
        if pr[1] != "end" or su[1] != "start":
            report["skipped"].append([rid, "contact %s/%s" % (pr[1], su[1])])
            continue
        d0, f0 = delta_end(roads[pr[0]], changes.get(pr[0]), pr[3], True)
        d1, f1 = delta_end(roads[su[0]], changes.get(su[0]), su[3], False)
        if not a.dry_run:
            set_connector(road, d0, f0, d1, f1, pr[2], su[2])
        nconn += 1
    if not a.dry_run:
        perr = 0.0
        for rid, p in changes.items():
            perr = max(perr, set_street(roads[rid], p["c"], p["f"], p.get("g"), p.get("cf")))
        report["planview_refit_max_err"] = round(perr, 4)
        tree.write(out, encoding="utf-8", xml_declaration=True)
        carla.Map("check", open(out).read())           # still loads / 読み込めること
    rs = report["roads"]
    report["summary"] = dict(roads_changed=len(rs), junction_roads_changed=nconn,
                             pair=sum(r["kind"] == "pair" for r in rs),
                             two_way=sum(r["kind"] == "two-way" for r in rs),
                             one_way=sum(r["kind"] == "one-way" for r in rs),
                             short=len(report["short"]), min_lane=MIN_LANE, min_lane_1=MIN_LANE_1, p_low=P_LOW, margin=MARGIN)
    os.makedirs(os.path.dirname(rep_path), exist_ok=True)
    json.dump(report, open(rep_path, "w"), indent=1)
    narrowed = [r["W"] - r["W_new"] for r in rs]
    print("[fit_core_widths] %d roads in the core narrowed to the model's asphalt (%d paired directions, %d two-way "
          "alone: kerb edge in, %d one-way), by median %.2f m, max %.2f m; %d junction roads follow; %d need lanes "
          "< %.1f m (one-lane roads %.1f m) or have notches somewhere (reported: the model has less road there)%s"
          % (len(rs), report["summary"]["pair"], report["summary"]["two_way"], report["summary"]["one_way"],
             float(np.median(narrowed)) if narrowed else 0.0, max(narrowed) if narrowed else 0.0, nconn,
             len(report["short"]), MIN_LANE, MIN_LANE_1, " [dry run]" if a.dry_run else ""))
    if report["short"]:
        print("[fit_core_widths]   short: " + ", ".join("%d (%d lanes, %.1f m, notch %.0f%%)" % (r["road"], r["lanes"], r["need"], 100 * r["notch"])
                                                for r in report["short"][:20]))


if __name__ == "__main__":
    main()
