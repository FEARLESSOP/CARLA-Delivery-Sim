"""Tests for the Pasir Ris Street 11 primitives: every face points the way it is
seen (UE culls back faces), names CARLA keeps (maps/.venv).

  .venv/bin/python osm_city/landmarks/test_prs11.py

パシリス・ストリート 11 用の基本形状の面の向きと名前を確認する。
"""

import math
import os
import sys
import tempfile

import numpy as np
from shapely.geometry import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import buildings as B  # noqa: E402
from landmarks import prs11  # noqa: E402
from mesh import Mesh  # noqa: E402


def tris(m, mat=None):
    """(centroids, winding normals) of every triangle. / 三角形の重心と巻き方向の法線。"""
    C, N = [], []
    for k, (V, _, _, I, _) in m.parts.items():
        if mat is not None and k != mat:
            continue
        Vc, Ic = np.concatenate(V), np.concatenate(I)
        T = Vc[Ic]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        keep = np.linalg.norm(n, axis=1) > 1e-10
        C.append(T[keep].mean(axis=1))
        N.append(n[keep] / np.linalg.norm(n[keep], axis=1)[:, None])
    return np.concatenate(C), np.concatenate(N)


def check(name, ok):
    print(("PASS " if ok else "FAIL ") + name)
    return ok


def main():
    good = True
    # barrel vault along +x: the roof faces away from its axis, the underside toward it
    # ヴォールト: 上面は軸から外向き、裏面は軸向き
    for p0, p1 in (((0, 0), (10, 0)), ((3, 4), (-2, 9))):
        m = Mesh()
        prs11.vault(m, "top", p0, p1, 1.3, 2.5, seg=8, under="under")
        ax = np.array(p1, float) - np.array(p0, float)
        ax /= np.linalg.norm(ax)
        for mat, sign in (("top", 1), ("under", -1)):
            C, N = tris(m, mat)
            rel = C - np.array([p0[0], p0[1], 2.5])
            rel[:, :2] -= np.outer(rel[:, :2] @ ax, ax)          # off the axis / 軸からの成分
            good &= check("vault %s faces %s (%d tris)" % (mat, "out" if sign > 0 else "in", len(C)),
                          bool(np.all(sign * np.sum(rel * N, 1) > 0)))
        C, N = tris(m, "top")
        good &= check("vault top above the springing", bool(np.all(C[:, 2] >= 2.5)))
    # the white frame round a window: every face points away from the wall or sideways out
    # 窓の白枠: すべての面が壁から外向きか横向き
    m = Mesh()
    f = B.Facade(m, np.array([0.0, 0.0]), np.array([6.0, 0.0]), np.array([0.0, -1.0]))
    prs11.bracket_window(f, 2.0, 4.0, 1.0, 2.5, "frame")
    C, N = tris(m)
    good &= check("window frame never faces into the wall (%d tris)" % len(C), bool(np.all(N[:, 1] < 0.5)))
    tops = N[:, 2] > 0.5
    good &= check("window frame head has an upward top", bool(tops.any()))
    # a texture quad (plate, porthole) faces out from the wall / 板・丸窓は壁の外を向く
    for n in ((0.0, -1.0), (1.0, 0.0), (0.6, 0.8)):
        m = Mesh()
        nn = np.array(n)
        a = np.array([0.0, 0.0])
        b = a + np.array([nn[1], -nn[0]]) * -5.0
        f = B.Facade(m, a, b, nn)
        prs11.texquad(f, "plate", 1.0, 2.0, 3.0, 3.5, 0.06)
        C, N = tris(m)
        good &= check("plate faces the wall's normal %s" % (n,), bool(np.all(N[:, :2] @ nn > 0.99)))
    # the lettering faces out and is not mirrored (reads left to right from in front)
    # 文字は外向きで鏡文字でない
    m = Mesh()
    a, b, nn = np.array([0.0, 0.0]), np.array([10.0, 0.0]), np.array([0.0, -1.0])
    f = B.Facade(m, a, b, nn)
    obj = prs11.PRS11.__new__(prs11.PRS11)
    obj.letters(m, f, "L", 10.0, 5.0, 0.6)
    C, N = tris(m, "paint_royalblue")
    good &= check("lettering faces out (%d tris)" % len(C), bool(np.all(N[:, 1] < -0.99)))
    # an "L" seen from in front (looking +y: right is +x): its foot runs right of the stem
    # 正面から見た "L": 下の横棒は縦棒の右へ伸びる
    Vb = np.concatenate(m.parts["paint_royalblue"][0])
    right = Vb[np.argmax(Vb[:, 0])]
    good &= check("lettering not mirrored", bool(right[2] < Vb[:, 2].min() + 0.15))
    # a whole cluster block: walls face out, roofs up (a small L-shaped test block)
    # クラスター棟全体: 壁は外向き、屋根は上向き
    poly = Polygon([(0, 0), (30, 0), (30, 12), (14, 12), (14, 24), (0, 24)])
    m = Mesh()
    obj = prs11.PRS11.__new__(prs11.PRS11)
    obj.VOID, obj.FLOOR, obj.PAR, obj.MOTOR, obj.CROWN = 3.2, 2.8, 1.0, 5.5, 1.8
    from shapely.geometry import box
    obj.drive_snapshot = box(-20, -20, -15, -15)
    obj.road = box(100, 100, 101, 101)
    obj.room = box(100, 100, 101, 101)
    obj.cluster_foot = poly
    b = dict(levels=7, bearing=90, ends=[dict(bearing=90, motif="bars", plate=True, rings=True, letters="130 Pasir",
                                              portholes=True, crown=True)],
             tower_polys=[(Polygon([(1, 13), (13, 13), (13, 23), (1, 23)]), (7, 18), (5, 5))])
    obj.hdb93(m, poly, b, "130")
    C, N = tris(m)
    cen = np.array([12.0, 10.0])
    walls = np.abs(N[:, 2]) < 0.1
    # outer walls at the footprint line face away from the block; the void deck's inner walls
    # are set back; test the facade line only / 外周線上の壁のみ判定
    from shapely.geometry import Point
    ring = poly.exterior
    on = np.array([ring.distance(Point(c[0], c[1])) < 0.02 for c in C]) & walls
    outward = []
    for c, n in zip(C[on], N[on]):
        q = np.array(c[:2]) + n[:2] * 0.05
        outward.append(not poly.contains(Point(q[0], q[1])))
    good &= check("block walls on the footprint line face out (%d)" % len(outward), all(outward))
    for mat in ("roof_concrete", "solar"):
        Cr, Nr = tris(m, mat)
        good &= check("%s faces up (%d)" % (mat, len(Cr)), bool(np.all(Nr[:, 2] > 0.99)))
    good &= check("block triangles %d (< 40 k)" % len(C), len(C) < 40000)
    # texture names CARLA keeps (no "sign" / "light") / CARLA が捨てない名前
    with tempfile.TemporaryDirectory() as d:
        mats = prs11.write_textures(d)
    bad = [k for k in mats if "sign" in k.lower() or "light" in k.lower()]
    good &= check("%d texture names without sign/light" % len(mats), not bad)
    print("ALL PASS" if good else "SOME FAILED")
    sys.exit(0 if good else 1)


if __name__ == "__main__":
    main()
