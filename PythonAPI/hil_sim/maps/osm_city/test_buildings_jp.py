"""Face-direction tests for the Japanese building primitives (buildings_jp).

  maps/.venv/bin/python osm_city/test_buildings_jp.py

UE culls back faces, so every face must point out of the solid it bounds
(Blender renders hide a wrong winding). Each test checks the winding normal
(not the stored one) of every triangle. / 面の向き（UE は裏面を描かない）の試験。
"""

import os
import random
import sys

import numpy as np
from shapely.geometry import Polygon, box

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import buildings  # noqa: E402
import buildings_jp as J  # noqa: E402
from mesh import Mesh  # noqa: E402


def tris(m, mats=None):
    """(material, centroid, winding normal) for every triangle. / 全三角形。"""
    out = []
    for mat, (V, N, UV, I) in m.arrays().items():
        if mats and mat not in mats:
            continue
        T = V[I].astype(np.float64)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(n, axis=1)
        keep = ln > 1e-9
        n = n[keep] / ln[keep, None]
        stored = N[I[:, 0]][keep]
        assert (np.sum(n * stored, axis=1) > 0.99).all(), "%s: stored normal disagrees with winding" % mat
        out += [(mat, c, nn) for c, nn in zip(T[keep].mean(axis=1), n)]
    return out


def check(name, cond):
    print("%-58s %s" % (name, "ok" if cond else "FAIL"))
    assert cond, name


def test_roofs():
    R = J.Rect.of(box(0, 0, 10, 7))
    for kind in ("hip", "gable"):
        m = Mesh()
        if kind == "hip":
            J.roof_hip(m, R, 6.0, 0.45, 0.75, "jp_kawara_ibushi")
        else:
            J.roof_gable(m, R, 6.0, 0.45, 0.75, "jp_kawara_ibushi", "jp_wall_white")
        T = tris(m)
        ctr = np.array([5.0, 3.5])
        roof_tops = [(c, n) for mat, c, n in T if mat == "jp_kawara_ibushi" and abs(n[2]) > 0.1]
        check("%s roof: every sloping roof face looks up" % kind, all(n[2] > 0 for c, n in roof_tops))
        check("%s roof: every soffit face looks down" % kind,
              all(n[2] < 0 for mat, c, n in T if mat == "jp_soffit"))
        vert = [(c, n) for mat, c, n in T if abs(n[2]) < 0.05]
        check("%s roof: fascia, barge, gutter, ridge, gable faces look out" % kind,
              all(np.dot(n[:2], c[:2] - ctr) > 0 or mat == "jp_ridge" and abs(c[1] - 3.5) < 0.5
                  for (mat, c, n) in [(mm, cc, nn) for mm, cc, nn in T if abs(nn[2]) < 0.05]))
        check("%s roof: ridge cap faces look away from the ridge" % kind,
              all((n[2] > 0.9) or np.dot(n[:2], c[:2] - np.array([c[0] if abs(n[0]) < 0.5 else 5.0, 3.5])) > 0
                  for mat, c, n in T if mat == "jp_ridge"))
        check("%s roof: eaves reach 0.75 m (+ gutter) and no further" % kind,
              all(-0.75 - J.GUTTER - 1e-6 <= c[1] <= 7.75 + J.GUTTER + 1e-6 for mat, c, n in T))
    R = J.Rect.of(box(0, 0, 10, 7))
    ctr = np.array([5.0, 3.5])
    for name, fn in (("per-side gable", J.roof_gable_ps), ("mono-pitch", J.roof_mono_ps)):
        m = Mesh()
        fn(m, R, 6.0, 0.4 if fn is J.roof_gable_ps else 0.25, (0.0, 0.75, 0.9, 0.1), "jp_kawara_ibushi",
           "jp_wall_white")
        T = tris(m)
        check("%s roof: sloping faces up, soffits down" % name,
              all(n[2] > 0 for mat, c, n in T if mat == "jp_kawara_ibushi" and abs(n[2]) > 0.1)
              and all(n[2] < 0 for mat, c, n in T if mat == "jp_soffit"))
        check("%s roof: vertical faces look out" % name,
              all(np.dot(n[:2], c[:2] - ctr) > 0 for mat, c, n in T if abs(n[2]) < 0.05 and mat != "jp_ridge"))
        V = np.concatenate([v[0] for v in m.arrays().values()])
        check("%s roof: each side keeps its own eave (0 m at y=0, 0.1 m at x=0)" % name,
              V[:, 1].min() >= -1e-6 and V[:, 0].min() >= -0.1 - 1e-6 and V[:, 1].max() > 7.8)
    m = Mesh()
    J.roof_mono(m, J.Rect.of(box(0, 0, 8, 2)).sub(0, 8, 0, 2), 4.0, 0.3, 0.6, "jp_kawara_ibushi")
    T = tris(m)
    check("lean-to: top looks up, soffit down",
          all(n[2] > 0 for mat, c, n in T if mat == "jp_kawara_ibushi" and abs(n[2]) > 0.1)
          and all(n[2] < 0 for mat, c, n in T if mat == "jp_soffit"))


def test_rect_toward():
    R = J.Rect.of(box(0, 0, 10, 6))
    for up in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        S = J.Rect.toward([R.xy(0, 0), R.xy(10, 0), R.xy(10, 2), R.xy(0, 2)], up)
        check("Rect.toward %s: ay points that way, corners kept" % (up,),
              np.allclose(S.ay, np.array(up, float) / np.linalg.norm(up)) and
              abs(S.A * S.B - 20.0) < 1e-6)


def test_stair_and_boxes():
    m = Mesh()
    f = buildings.Facade(m, np.array([0.0, 0.0]), np.array([10.0, 0.0]), np.array([0.0, -1.0]))
    J.stair(m, f, 2.0, 3.6, 0.08, 1.0, 0.15, 3.0)
    T = tris(m)
    tops = [(c, n) for mat, c, n in T if abs(n[2]) > 0.9]
    check("stair: treads have a top (up) and a bottom (down) face",
          sum(1 for c, n in tops if n[2] > 0) == sum(1 for c, n in tops if n[2] < 0) > 0)
    risers = [n for mat, c, n in T if abs(n[0]) > 0.9]
    check("stair: tread nosings face down the stair (-u); posts both ways",
          sum(1 for n in risers if n[0] < 0) >= 13)
    m = Mesh()
    f = buildings.Facade(m, np.array([0.0, 0.0]), np.array([10.0, 0.0]), np.array([0.0, -1.0]))
    J.ac_unit(f, 2.0, 0.15)
    T = tris(m, {"jp_ac_unit"})
    ctr = np.array([2.4, -0.2, 0.45])
    check("AC unit: every face looks out of the box", all(np.dot(n, c - ctr) > 0 for mat, c, n in T))


class FakeDrape(object):
    """Ground falling 0.3 m per metre toward -y. / -y 方向に下る地面。"""

    def z(self, x, y):
        return 0.3 * np.asarray(y, float)

    def footprint(self, poly, step=2.0):
        b = poly.bounds
        return 0.3 * b[1], 0.3 * b[3]


def fake_jp():
    B = buildings.Builder.__new__(buildings.Builder)
    B.L, B.sites, B.count, B.schools, B.jp, B._jp = None, None, {}, [], True, None
    jp = J.JP(B)
    jp.items = {}
    return B, jp


def item(jp, poly, style, lot):
    it = J.Item()
    it.i, it.t, it.raw, it.style = 0, {}, poly, style
    it.poly, it.shape, it.R, it.R2 = jp.regularise(poly, style, 0)
    it.levels = 2
    it.var = jp.pick(it, random.Random(3))
    it.lot = lot
    if jp.D is not None:
        it.zb = 0.3 * poly.bounds[3]
        it.drop = 0.3 * (poly.bounds[3] - poly.bounds[1])
    else:
        it.zb, it.drop = 0.0, 0.0
    jp.items[0] = it
    return it


def test_retaining_walls():
    B, jp = fake_jp()
    jp.D = FakeDrape()
    poly = box(0, 0, 10, 8)
    lot = Polygon([(-3, -3), (13, -3), (13, 11), (-3, 11)])
    it = item(jp, poly, "house", lot)
    m = Mesh()
    jp.site(m, it)
    T = tris(m, {"jp_ishigaki", "jp_retaining"})
    ctr = np.array([5.0, 4.0])
    fill = [(mat, c, n) for mat, c, n in T if c[2] < J.RAISED_Z - 0.05 and mat == "jp_ishigaki"]
    cut = [(mat, c, n) for mat, c, n in T if mat == "jp_retaining"]
    check("house on a slope: stone (ishigaki) fill walls", len(fill) > 0)
    check("fill walls face out of the terrace (and lean back)",
          all(np.dot(n[:2], c[:2] - ctr) > 0 and n[2] >= -1e-6 for mat, c, n in fill))
    check("cut walls (bank above the terrace) face into it",
          len(cut) > 0 and all(np.dot(n[:2], c[:2] - ctr) < 0 for mat, c, n in cut))
    V = m.arrays()["jp_ishigaki"][0]
    low = V[(V[:, 1] < -2.9) & (V[:, 2] < J.RAISED_Z - 0.1)]
    check("the downhill fill wall reaches 0.3 m below the ground",
          len(low) > 0 and (low[:, 2] <= 0.3 * low[:, 1] - it.zb + J.RAISED_Z - 0.29).all())
    top = [n for mat, c, n in tris(m, {"jp_lot"})]
    check("terrace top looks up", len(top) > 0 and all(n[2] > 0.99 for n in top))


def test_styles_smoke():
    """Every style builds on a rectangle and an L plan; walls look out of the
    footprint (horizontal faces on the footprint line). / 各様式が作れること。"""
    for style, poly in (("house", box(0, 0, 10, 8)), ("house", Polygon([(0, 0), (12, 0), (12, 6), (6, 6), (6, 10),
                                                                         (0, 10)])),
                        ("apaato", box(0, 0, 16, 9)), ("danchi", box(0, 0, 48, 9)),
                        ("mansion", box(0, 0, 30, 12)), ("shop", box(0, 0, 9, 12)), ("shed", box(0, 0, 3, 2.5)),
                        ("shed", box(0, 0, 20, 12)), ("office:punched", box(0, 0, 24, 16)),
                        ("office:ribbon", box(0, 0, 24, 16)), ("office:curtain", box(0, 0, 24, 16)),
                        ("office:hotel", box(0, 0, 14, 30)), ("carpark", box(0, 0, 30, 20))):
        B, jp = fake_jp()
        style, _, front = style.partition(":")
        it = item(jp, poly, style, poly.buffer(2.0, join_style="mitre"))
        it.levels = {"danchi": 5, "mansion": 6, "office": 9, "carpark": 4}.get(style, it.levels)
        if front:
            it.var.update(front=front, podium="shop" if front != "hotel" else "office")
        m = Mesh()
        getattr(jp, "b_" + style)(m, it, random.Random(1))
        jp.site(m, it)
        T = tris(m)
        # faces lying on the footprint line, vertical: must look out / 外形線上の鉛直面は外向き
        ring = it.poly.exterior
        from shapely.geometry import Point
        bad = 0
        n_on = 0
        for mat, c, n in T:
            if abs(n[2]) > 0.05 or mat in ("jp_lot",):
                continue
            if ring.distance(Point(c[0], c[1])) < 0.005:
                n_on += 1
                q = Point(c[0] + 0.05 * n[0], c[1] + 0.05 * n[1])
                if it.poly.buffer(-0.001).contains(q):
                    bad += 1
        check("%s (%s): %d tris, %d wall faces on the outline, %d face in" % (style, it.shape, len(T), n_on, bad),
              bad == 0 and n_on > 0)


if __name__ == "__main__":
    test_rect_toward()
    test_roofs()
    test_stair_and_boxes()
    test_retaining_walls()
    test_styles_smoke()
    print("all passed")
