"""Tests for the White Sands Primary School primitives: every face points the
way it is seen (UE culls back faces), vault strips stay off roads, names CARLA
keeps (maps/.venv).

  .venv/bin/python osm_city/landmarks/test_wsps.py

ホワイトサンズ小学校の基本形状: 面の向き、道路からの幅詰め、材質名を確認する。
"""

import os
import sys
import tempfile

import numpy as np
import shapely
from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import buildings as B  # noqa: E402
from landmarks import wsps  # noqa: E402
from landmarks.test_coral import check, tris  # noqa: E402
from mesh import Mesh  # noqa: E402


def site_obj(strips=None, blocks=None):
    """A bare WSPS with the frame axis-aligned (u = x, v = y). / 座標軸を揃えた最小の WSPS。"""
    obj = wsps.WSPS.__new__(wsps.WSPS)
    obj.frame = ((0.0, 0.0), 0.0)
    obj.strips = strips or {}
    obj.blocks = blocks or {}
    obj.no_go = Polygon([(500, 500), (501, 500), (501, 501)])
    return obj


def faces_out(m, poly, tol=0.004):
    """(number of vertical faces on the footprint, all of them face out). / 外周上の面が外向きか。"""
    C, N, _ = tris(m)
    bd = shapely.distance(shapely.points(C[:, :2]), poly.exterior)
    on = (bd < tol) & (np.abs(N[:, 2]) < 0.01)
    out = ~shapely.contains_xy(poly, C[on, 0] + N[on, 0] * 0.05, C[on, 1] + N[on, 1] * 0.05)
    return int(on.sum()), bool(np.all(out))


def main():
    good = True
    prof = (12.0, 16.0, 13.0, 0.4)
    good &= check("profile: eaves and crown", abs(wsps.prof_z(prof, 0) - 12) < 1e-9
                  and abs(wsps.prof_z(prof, 0.4) - 16) < 1e-9 and abs(wsps.prof_z(prof, 1) - 13) < 1e-9)

    # a straight strip: CCW footprint, edges with outward normals / 直線の帯: 反時計回り、辺の法線は外向き
    S = wsps.Strip([(0, 0), (30, 0)], 8.0, 6.0)
    p = S.poly()
    good &= check("strip footprint CCW, 30 x 14", p.exterior.is_ccw and abs(p.area - 420) < 1e-6)
    ok = True
    for a, b, side, i in S.edges():
        t = (b - a) / np.linalg.norm(b - a)
        n = np.array([t[1], -t[0]])
        mid = (a + b) / 2 + n * 0.05
        ok &= not p.contains(Point(*mid))
    good &= check("strip edges: right normal points outside", ok)
    # roads win: a road along the left side narrows only that side, one width
    # 道路優先: 左側の道路で左の幅だけ一様に詰める
    road = box(-5, 6.0, 40, 15).buffer(1.0)
    cut = S.fit(road)
    good &= check("fit: left side narrowed to clear the road (wl %.2f, cut %.2f)" % (S.wl[0], cut),
                  abs(S.wl[0] - S.wl[1]) < 1e-9 and 4.8 < S.wl[0] < 5.0 and S.wr[0] == 6.0
                  and not S.poly().intersects(road))
    # a curved strip (quarter circle) keeps its stations' widths / 円弧の帯
    ang = np.radians(np.arange(180, 271, 5))
    C = wsps.Strip([(30 * np.cos(a), 30 * np.sin(a)) for a in ang], 7.5, 7.5)
    good &= check("curved strip footprint valid and CCW", C.poly().is_valid and C.poly().exterior.is_ccw)
    s0, _ = C.locate((-30 + 7.0, 0.0))
    s1, _ = C.locate((-30 - 7.0, 0.0))
    good &= check("locate: left edge near s=0, right near s=1 (%.2f, %.2f)" % (s0, s1), s0 < 0.1 and s1 > 0.9)

    # vault roof: top up, soffits down, fascia out / ヴォールト: 上面は上、軒裏は下、鼻隠しは外
    m = Mesh()
    S = wsps.Strip([(0, 0), (30, 0)], 8.0, 6.0)
    wsps.vault(m, S, prof, "roof", ext=(0.6, 0.6), fascia="fas", soffit="sof")
    Cr, Nr, _ = tris(m, "roof")
    good &= check("vault top faces up (%d)" % len(Cr), len(Cr) > 20 and np.all(Nr[:, 2] > 0.2))
    Cs, Ns, _ = tris(m, "sof")
    good &= check("vault soffits face down (%d)" % len(Cs), len(Cs) > 4 and np.all(Ns[:, 2] < -0.2))
    Cf, Nf, _ = tris(m, "fas")
    ctr = np.array([15.0, 1.0])
    good &= check("vault fascia faces out (%d)" % len(Cf), np.all(np.sum((Cf[:, :2] - ctr) * Nf[:, :2], axis=1) > 0))
    good &= check("vault reaches the crown", abs(Cr[:, 2].max() - 16.06) < 0.3)
    m = Mesh()
    wsps.vault(m, C, (15.4, 18.6, 17.0, 0.7), "roof")
    Cr, Nr, _ = tris(m, "roof")
    good &= check("curved vault top faces up (%d)" % len(Cr), np.all(Nr[:, 2] > 0.1))

    # gable fill on an end cap faces out and follows the profile / 妻面の上部は外向き
    m = Mesh()
    a, b = S.right(1), S.left(1)
    L = float(np.linalg.norm(b - a))
    t = (b - a) / L
    f = B.Facade(m, a, b, np.array([t[1], -t[0]]))
    wsps.gable_fill(f, "gw", S, prof, 0.0, L, L, lambda u: 12.0, "E1")
    Cg, Ng, _ = tris(m, "gw")
    good &= check("gable fill faces out (+x) (%d)" % len(Cg), len(Cg) > 4 and np.all(Ng[:, 0] > 0.99))
    good &= check("gable fill under the profile", Cg[:, 2].max() < 16.0 and Cg[:, 2].min() > 11.99)

    # drum walls out, skipping a neighbour / 円筒は外向き、隣棟の中は省く
    m = Mesh()
    wsps.drum_walls(m, "dr", (0.0, 0.0), 5.0, 0.0, 10.0, skip=box(0, -10, 10, 10))
    Cd, Nd, _ = tris(m, "dr")
    good &= check("drum walls face out, none inside the neighbour",
                  np.all(np.sum(Cd[:, :2] * Nd[:, :2], axis=1) > 0) and np.all(Cd[:, 0] < 0.5))

    # railing bars two-sided, vquad_out away from its reference / 縦格子は両面、vquad_out は外向き
    m = Mesh()
    wsps.bars(m, "bar", (0, 0), (5, 0), 0.7, 1.8)
    Cb, Nb, _ = tris(m, "bar")
    good &= check("bars two-sided (%d tris)" % len(Cb), (Nb[:, 1] < -0.9).sum() == (Nb[:, 1] > 0.9).sum() > 20)
    m = Mesh()
    wsps.vquad_out(m, "q", (0, 0), (4, 0), 0, 1, 0, 1, ref=(2, 3))
    wsps.vquad_out(m, "q", (4, 0), (0, 0), 0, 1, 0, 1, ref=(2, 3))
    Cq, Nq, _ = tris(m, "q")
    good &= check("vquad_out faces away from the reference", np.all(Nq[:, 1] < -0.99))

    # whole vaulted blocks and the other block kinds: faces on the footprint face
    # out, roofs up / ヴォールト棟と他の棟: 外周上の面は外向き、屋根は上向き
    strips = {}
    for k, kind, spine, w, pr in (("hall", "hall", [(0, 40), (0, 10)], (23.5, 23.5), (12, 16, 12, 0.5)),
                                  ("wing", "wing", [(-100 + 30 * np.cos(a), 30 * np.sin(a)) for a in ang],
                                   (7.5, 7.5), (17.0, 18.6, 15.4, 0.3)),
                                  ("front", "front", [(60, -30), (76, -30)], (21.5, 21.5), (16.2, 20.2, 16.2, 0.5))):
        St = wsps.Strip(spine, *w)
        strips[k] = dict(S=St, poly=St.poly(), prof=pr, kind=kind)
    blocks = {"ne": dict(poly=orient(box(100, 30, 138, 55), 1.0), top=20.0, ridge=21.0, kind="ne", roof="gable"),
              "east": dict(poly=orient(box(100, -20, 118, 18), 1.0), top=14.6, ridge=16.0, kind="east",
                           roof="gable"),
              "core": dict(poly=orient(box(140, -40, 176, -10), 1.0), top=19.0, kind="core", roof="flat"),
              "corner": dict(poly=orient(box(180, -10, 194, -2), 1.0), top=8.6, kind="corner", roof="flat"),
              "annex": dict(poly=orient(box(200, 0, 207, 18), 1.0), top=13.0, kind="annex", roof="flat"),
              "gate": dict(poly=orient(box(210, 0, 215, 15), 1.0), top=3.3, ridge=4.9, kind="gatehouse",
                           roof="hip")}
    obj = site_obj(strips, blocks)
    for k in strips:
        m = Mesh()
        obj.strip_block(m, k)
        n, ok = faces_out(m, strips[k]["poly"])
        good &= check("%s: %d faces on the footprint face out (%d tris)" % (k, n, m.ntris()), n > 10 and ok)
        Cr, Nr, _ = tris(m, "ws_roof")
        good &= check("%s: roof faces up" % k, len(Cr) and np.all(Nr[:, 2] > 0.1))
        C2, N2, M2 = tris(m, "ws_glass")
        if len(C2):
            good &= check("%s: windows face out" % k,
                          not np.any(shapely.contains_xy(strips[k]["poly"].buffer(-0.5), C2[:, 0] + N2[:, 0] * 0.6,
                                                         C2[:, 1] + N2[:, 1] * 0.6) &
                                     ~shapely.contains_xy(strips[k]["poly"].buffer(-0.5), C2[:, 0], C2[:, 1])))
    for k in blocks:
        m = Mesh()
        obj.block(m, k)
        n, ok = faces_out(m, blocks[k]["poly"])
        good &= check("%s: %d faces on the footprint face out (%d tris)" % (k, n, m.ntris()), n > 4 and ok)
        C, N, _ = tris(m)
        roofs = (C[:, 2] > blocks[k]["top"] + 0.5) & (np.abs(N[:, 2]) > 0.3)
        good &= check("%s: roof faces up" % k, np.all(N[roofs, 2] > 0))

    # stacked windows on the classroom face: no wall face over any opening
    # 教室側の縦に重なる窓: 開口の上に壁が無い
    m = Mesh()
    f = B.Facade(m, np.array([0.0, 0.0]), np.array([18.0, 0.0]), np.array([0.0, -1.0]))
    obj.class_out(f, 0.0, 18.0, 18.0, 15.4, 4)
    Cw, Nw, _ = tris(m, "ws_white")
    Cg2, _, _ = tris(m, "ws_glass")
    flat = (np.abs(Cw[:, 1]) < 1e-6) & (Nw[:, 1] < -0.99)
    over = 0
    for c in Cg2:
        over += int(np.any(flat & (np.abs(Cw[:, 0] - c[0]) < 1.1) & (np.abs(Cw[:, 2] - c[2]) < 0.6)))
    good &= check("classroom face: %d window glass, no wall face over them" % (len(Cg2) // 2), len(Cg2) >= 16
                  and over == 0)

    d = tempfile.mkdtemp()
    mats = wsps.write_textures(d)
    bad = [k for k in mats if "light" in k.lower() or "sign" in k.lower()]
    good &= check("texture names CARLA keeps (%d)" % len(mats), not bad)
    print("ALL PASS" if good else "SOME FAILED")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
