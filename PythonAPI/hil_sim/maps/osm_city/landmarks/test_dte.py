"""Tests for the Downtown East primitives: every face points the way it is
seen (UE culls back faces), texture names CARLA keeps (maps/.venv).

  .venv/bin/python osm_city/landmarks/test_dte.py

ダウンタウン・イースト用の基本形状の面の向きを確認する。
"""

import math
import os
import sys
import tempfile

import numpy as np
from shapely.geometry import Polygon, box

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import buildings as B  # noqa: E402
from landmarks import dte  # noqa: E402
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
    # a closed square tube: every face away from its axis / 角パイプ: 全面が外向き
    m = Mesh()
    p0, p1 = np.array([1.0, 2.0, 0.5]), np.array([4.0, 3.0, 6.0])
    dte.beam(m, "x", p0, p1, 0.2, caps=True)
    C, N = tris(m)
    ctr = (p0 + p1) / 2
    good &= check("beam faces out (%d tris)" % len(C), len(C) == 12 and bool(np.all(np.sum((C - ctr) * N, 1) > 0)))
    # a tube along a bent 3-D line: every face away from the line / 管: 全面が軸から外向き
    m = Mesh()
    pts = [(0, 0, 5), (5, 0, 4), (8, 4, 3), (8, 9, 1)]
    dte.tube(m, "x", pts, 0.6, 6)
    C, N = tris(m)
    from shapely.geometry import LineString
    ln = LineString([p[:2] for p in pts])
    out = []
    for c, n in zip(C, N):
        q = ln.interpolate(ln.project(LineString([c[:2], c[:2]]).centroid))
        # the axis point at that station (z interpolated along the line) / 軸上の点
        s = ln.project(q) / ln.length
        st = np.cumsum([0] + [math.dist(a[:2], b[:2]) for a, b in zip(pts, pts[1:])]) / ln.length
        z = np.interp(s, st, [p[2] for p in pts])
        out.append(np.dot(c - np.array([q.x, q.y, z]), n) > 0)
    good &= check("tube faces out (%d tris)" % len(C), all(out))
    # a disc facing east: front +x, back -x, rim outward / 円盤: 正面は東
    m = Mesh()
    dte.disc(m, "front", "rim", (0, 0, 10), (1, 0), 2.0, seg=12)
    C, N = tris(m, "front")
    good &= check("disc front faces its normal", bool(np.all(N[:, 0] > 0.99)))
    C, N = tris(m, "rim")
    side = np.abs(N[:, 0]) < 0.5
    rad = C[side] - np.array([0, 0, 10])
    rad[:, 0] = 0
    good &= check("disc back faces away, rim outward",
                  bool(np.all(N[~side][:, 0] < -0.99)) and bool(np.all(np.sum(rad * N[side], 1) > 0)))
    # the disc's text reads left to right from in front (u along the viewer's right)
    # 正面から見て文字が左から右へ
    m = Mesh()
    dte.disc(m, "front", "rim", (0, 0, 10), (1, 0), 2.0, seg=12)
    V, _, UV, I, _ = m.parts["front"]
    V, UV = np.concatenate(V), np.concatenate(UV)
    k_right = np.argmax(V[:, 1])                 # facing +x the viewer's right is +y / 右は +y
    good &= check("disc texture not mirrored", UV[k_right, 0] > 0.9)
    # hipped roofs (plain and with a lantern frustum): roof up and out, soffit down
    # 寄棟（頂部台形あり・なし）: 屋根は上・外向き、軒裏は下向き
    for inner in (None, (5.0, 5.0, 16.0)):
        m = Mesh()
        poly = Polygon([(0, 0), (30, 4), (27, 24), (-3, 20)])
        dte.hip(m, poly, 10.0, 19.0, "roof", ov=1.0, soffit="soff", inner=inner)
        C, N = tris(m, "roof")
        c = np.array([poly.centroid.x, poly.centroid.y])
        ok = bool(np.all(N[:, 2] > 0)) and bool(np.all(np.sum((C[:, :2] - c) * N[:, :2], 1) >= -1e-6))
        Cs, Ns = tris(m, "soff")
        good &= check("hip roof up/out, soffit down (inner=%s)" % (inner,), ok and bool(np.all(Ns[:, 2] < -0.99)))
    # gable roof: slopes up, gable walls out / 切妻: 屋根は上、妻壁は外
    m = Mesh()
    poly = box(0, 0, 20, 10)
    dte.gable(m, poly, 5.0, 8.0, "roof", "wall", ov=0.5)
    C, N = tris(m, "roof")
    Cw, Nw = tris(m, "wall")
    good &= check("gable roof up, gable walls out", bool(np.all(N[:, 2] > 0)) and
                  bool(np.all(np.sum((Cw[:, :2] - np.array([10, 5])) * Nw[:, :2], 1) > 0)))
    # a two-sided sheet: one face each way / 両面の膜
    m = Mesh()
    dte.two_sided(m, "mem", (0, 0, 4), (6, 0, 4), (3, 3, 9), (3, 3, 9))
    C, N = tris(m)
    good &= check("two-sided sheet faces both ways", len(N) == 2 and float(N[0] @ N[1]) < -0.99)
    # the funnel: outside faces out, inside faces in / 漏斗: 外面は外、内面は内
    obj = dte.DTE.__new__(dte.DTE)
    m = Mesh()
    obj.funnel(m, 0.0, 0.0, 6.0, 14.0, 2.5, 9.0, seg=12)
    C, N = tris(m, "dte_slide_red")
    rad = C[:, :2] / np.linalg.norm(C[:, :2], axis=1)[:, None]
    d = np.sum(rad * N[:, :2], 1)
    good &= check("funnel outside out, inside in", int((d > 0).sum()) == int((d < 0).sum()) and len(d) > 0)
    # face_in faces into the building / face_in は内向き
    m = Mesh()
    f = B.Facade(m, np.array([0.0, 0.0]), np.array([10.0, 0.0]), np.array([0.0, -1.0]))
    dte.face_in(f, "rail", 0, 10, 0, 1, -0.1)
    C, N = tris(m)
    good &= check("face_in faces -n", bool(np.all(N[:, 1] > 0.99)))
    # the ribbon window and the curtain wall: everything that is not a reveal faces out
    # 連続窓とカーテンウォール: 見込み以外は外向き
    m = Mesh()
    f = B.Facade(m, np.array([0.0, 0.0]), np.array([20.0, 0.0]), np.array([0.0, -1.0]))
    obj.ribbon(f, 0, 20, 0, 3.2, "glass", "wall", 0.9, 2.8, 0.3)
    obj.curtain(f, 0, 20, 3.2, 12.0, 4.4)
    C, N = tris(m, "glass")
    Cw, Nw = tris(m, "wall")
    vertical = np.abs(Nw[:, 2]) < 0.5
    along = np.abs(Nw[:, 0]) > 0.99                   # reveal sides / 見込みの側面
    sill_head_ok = bool(np.all((Nw[~vertical][:, 2] > 0) == (Cw[~vertical][:, 2] < 1.5)))
    good &= check("ribbon window glass and walls face out, sill up, head down",
                  bool(np.all(N[:, 1] < -0.99)) and bool(np.all(Nw[vertical & ~along][:, 1] < -0.99)) and sill_head_ok)
    Cc, Nc = tris(m, "dte_curtain")
    good &= check("curtain wall faces out", bool(np.all(Nc[:, 1] < -0.99)))
    # texture names CARLA keeps (no "sign" / "light") and every file written
    # CARLA が捨てない名前（sign / light を含まない）、全ファイル出力
    with tempfile.TemporaryDirectory() as d:
        mats = dte.write_textures(d)
        names_ok = all("sign" not in k.lower() and "light" not in k.lower() for k in mats)
        files_ok = all(os.path.exists(os.path.join(d, k + ".png")) for k in mats)
    good &= check("textures: %d materials, names kept, files written" % len(mats), names_ok and files_ok)
    print("ALL PASS" if good else "SOME FAILED")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
