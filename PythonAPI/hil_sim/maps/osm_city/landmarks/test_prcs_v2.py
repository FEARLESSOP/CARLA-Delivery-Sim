"""Tests for the Pasir Ris Crest (PasirRisTampines) primitives and blocks: every
face points the way it is seen (UE culls back faces), names CARLA keeps
(maps/.venv).

  .venv/bin/python osm_city/landmarks/test_prcs_v2.py

白沙岭中学（新地図用）の基本形状と棟の面の向き、名前を確認する。
"""

import os
import sys

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from landmarks import prcs_v2 as P  # noqa: E402
from landmarks.test_coral import check, tris  # noqa: E402
from mesh import Mesh  # noqa: E402


def up_and_out(C, N, centre):
    """Every normal points up and away from the centre (roof faces). / 上・外向き。"""
    out = (C[:, :2] - np.asarray(centre)[None, :])
    return np.all(N[:, 2] > 0.05) and np.all(np.sum(N[:, :2] * out, axis=1) >= -1e-6)


def fake(blocks):
    """A PRCS2 without a layout: frame S = the map frame, nothing near.
    レイアウト無しの PRCS2（S 座標 = 地図座標、道路は遠く）。"""
    s = P.PRCS2.__new__(P.PRCS2)
    s.o, s.eu, s.ev = np.zeros(2), np.array([1.0, 0.0]), np.array([0.0, 1.0])
    s.X = lambda q: (float(q[0]), float(q[1]))
    s.no_go = box(1000, 1000, 1001, 1001)
    s.tc, s.tr = (500.0, 500.0), dict(drum=7.0, balcony=8.6, roof=9.2, cap=4.6, porch=10.0)
    s.T = {"blocks": {k: {"storeys": b["storeys"]} for k, b in blocks.items()},
           "tower": {"z_lower_roof": 12.3, "z_top": 15.3}}
    s.blocks = blocks
    s.courts = {"court_blue": (box(-200, -200, -180, -180), "court_bb_blue")}
    s.court_c = Point(-190, -190)
    return s


def main():
    good = True
    # hip roof: slopes and hips up and out, soffit down, fascia out / 寄棟: 上外向き・軒裏下・鼻隠し外
    m = Mesh()
    a, b, c, d = [np.array(q) for q in ((0, 0), (30, 0), (30, 14), (0, 14))]
    r = P.hip_roof(m, a, b, c, d, 14.5, 16.5, 0.9, "roof", "fascia", "soffit")
    C, N, _ = tris(m, "roof")
    good &= check("hip roof faces up and out (%d)" % len(C), up_and_out(C, N, (15, 7)))
    C, N, _ = tris(m, "soffit")
    good &= check("hip soffit faces down", np.all(N[:, 2] < -0.99))
    C, N, _ = tris(m, "fascia")
    out = C[:, :2] - np.array([15, 7])
    good &= check("hip fascia faces out", np.all(np.abs(N[:, 2]) < 1e-6) and np.all(np.sum(N[:, :2] * out, 1) > 0))
    # solar rows: up, over the slope, inside the roof / 太陽光パネル: 上向き、斜面の上
    m2 = Mesh()
    P.solar_rows(m2, *r, 14.5, 16.5)
    C, N, _ = tris(m2, "pr2_solar")
    inside = all(Polygon([r[0], r[1], r[2], r[3]]).contains(Point(q[0], q[1])) for q in C)
    good &= check("solar rows face up, on the roof (%d)" % len(C), len(C) > 0 and np.all(N[:, 2] > 0.9) and inside)
    # octagonal tiers: frustum and pyramid up and out, fascia out, soffit down / 八角屋根
    m = Mesh()
    P.oct_tier(m, (0, 0), 9.0, 4.0, 12.0, 13.2, "roof", "fascia", "soffit")
    P.oct_tier(m, (0, 0), 4.6, 0.0, 14.0, 15.3, "roof", "fascia", "soffit")
    C, N, _ = tris(m, "roof")
    good &= check("octagonal tiers face up and out (%d)" % len(C), up_and_out(C, N, (0, 0)))
    C, N, _ = tris(m, "fascia")
    good &= check("octagonal fascia faces out", np.all(np.sum(N[:, :2] * C[:, :2], 1) > 0))
    C, N, _ = tris(m, "soffit")
    good &= check("octagonal soffit faces down", np.all(N[:, 2] < -0.99))
    # railing: closed boxes (every face out of its own box) / 手すり: 閉じた箱
    m = Mesh()
    P.rail_line(m, "rail", (0, 0), (10, 0), 4.0)
    C, N, _ = tris(m, "rail")
    good &= check("railing has posts and rails (%d)" % len(C), len(C) > 40)

    # every kind of block: each vertical face on its footprint edge points out,
    # roofs and slabs on top face up / 全種類の棟: 外周の鉛直面は外向き
    kinds = [("E", "classroom", 4, P.Z0 + 4 * P.F, box(0, 0, 50, 16)),
             ("D", "classroom", 4, P.Z0 + 4 * P.F, box(0, -60, 50, -44)),
             ("A_link", "corridor", 3, P.Z0 + P.FA + 2 * P.F, box(0, 40, 30, 46)),
             ("A", "office", 3, P.Z0 + P.FA + 2 * P.F, box(0, 60, 40, 80)),
             ("B", "hall", 3, 11.5, box(60, 0, 93, 38)),
             ("A_core", "core", 3, 12.6, box(-20, 60, -13, 67)),
             ("A_glass", "curtain", 3, 11.6, box(-30, 60, -27.6, 66)),
             ("glass_stair", "glass_stair", 4, 16.5, box(-30, 0, -25, 7.5)),
             ("connector", "endblock", 4, 15.45, box(-60, 0, -54, 30)),
             ("stair_tower", "stair_tower", 5, 18.8, box(-70, 0, -65, 7)),
             ("joule", "joule", 1, 4.0, box(-90, 0, -83, 13.5)),
             ("store_w", "store", 1, 2.6, box(-100, 0, -94, 5)),
             ("sports_hall", "sports_hall", 1, 19.65, box(100, 0, 137, 24.4))]
    blocks = {}
    for k, kind, st, top, poly in kinds:
        blocks[k] = dict(poly=orient(poly, 1.0), kind=kind, storeys=st, top=top, ridge=top + 2.5,
                         uv=list(poly.bounds[::2] + poly.bounds[1::2]))
    s = fake(blocks)
    names = set()
    for k, kind, st, top, poly in kinds:
        m = Mesh()
        s.block(m, k, blocks[k])
        C, N, M = tris(m)
        names |= set(M)
        ring = poly.exterior
        bad = 0
        n_on = 0
        for cc, n in zip(C, N):
            if abs(n[2]) > 0.1 or cc[2] > top - 0.01:
                continue
            p = Point(cc[0], cc[1])
            if ring.distance(p) > 0.005:
                continue
            n_on += 1
            q = Point(cc[0] + n[0] * 0.05, cc[1] + n[1] * 0.05)
            if poly.contains(q):
                bad += 1
        good &= check("%s (%s): %d vertical faces on the footprint all point out" % (k, kind, n_on),
                      n_on > 0 and bad == 0)
    m = Mesh()
    s.tower(m)
    C, N, M = tris(m, "pr2_roof")
    good &= check("tower roofs face up and out (%d)" % len(C), up_and_out(C, N, s.tc))
    names |= set(tris(m)[2])
    bad = sorted(n for n in names if "sign" in n.lower() or "light" in n.lower())
    good &= check("no material name CARLA's importer drops (%d materials)" % len(names), not bad)
    print("ALL PASS" if good else "SOME FAILED")
    sys.exit(0 if good else 1)


if __name__ == "__main__":
    main()
