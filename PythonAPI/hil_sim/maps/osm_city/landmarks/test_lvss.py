"""Face-direction tests for the Loyang View landmark's geometry (maps/.venv).

  .venv/bin/python osm_city/landmarks/test_lvss.py      (or pytest)

UE culls back faces, and Blender renders hide inverted ones, so every new
primitive is checked by its winding (not by the normals it is given):
walls face out, roofs, courts and panels face up, plaques and letters face
away from their wall, closed solids face away from their centre.

洛阳中学モデルの面の向きのテスト（UE は裏面を描かない）。
"""

import math
import os
import sys

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import buildings as B  # noqa: E402
from landmarks import lvss  # noqa: E402
from mesh import Mesh  # noqa: E402


def tris(m, mats=None):
    """(material, triangle corners (n,3,3), winding normals (n,3)) per material. / 巻き順の法線。"""
    out = []
    for mat, (V, N, UV, I) in m.arrays().items():
        if mats is not None and mat not in mats:
            continue
        T = V[I].astype(np.float64)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(n, axis=1)
        ok = ln > 1e-9
        out.append((mat, T[ok], n[ok] / ln[ok, None]))
    return out


def fake():
    """An LVSS without a layout: no roads nearby, the courts to the south. / 道路なしの仮オブジェクト。"""
    o = lvss.LVSS.__new__(lvss.LVSS)
    o.road = box(1e4, 1e4, 1e4 + 1, 1e4 + 1)
    o.carriageway = o.road
    o.centre = Point(15.0, -40.0)
    o.footprint = Polygon()
    return o


def test_mapped_flat_faces_up_and_maps_the_rectangle():
    m = Mesh()
    rect = [(0, 0), (30, 0), (30, 15), (0, 15)]
    cut = Polygon(rect).difference(Point(30, 15).buffer(6.0))          # a court cut by a road / 道路で欠けたコート
    lvss.mapped_flat(m, "c", cut, rect, 0.2)
    (mat, T, n), = tris(m)
    assert (n[:, 2] > 0.999).all()
    V, N, UV, I = m.arrays()["c"]
    k = int(np.argmin(np.hypot(V[:, 0], V[:, 1])))
    assert np.allclose(UV[k], (0, 0), atol=1e-6)
    k = int(np.argmin(np.hypot(V[:, 0] - 30, V[:, 1])))
    assert np.allclose(UV[k], (1, 0), atol=1e-6)


def test_solid_and_oquad_face_out():
    m = Mesh()
    bot = [(0, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0)]
    top = [(0.5, 0, 3), (2.5, 0, 3), (2.5, 1, 3), (0.5, 1, 3)]
    lvss.solid(m, "s", bot, top)
    (mat, T, n), = tris(m)
    c = np.vstack([bot, top]).mean(axis=0)
    assert len(T) == 12
    assert ((T.mean(axis=1) - c) * n).sum(axis=1).min() > 0
    m = Mesh()
    lvss.oquad(m, "q", [(0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0)], (0.5, 1.0, 0.5))
    (mat, T, n), = tris(m)
    assert (n[:, 1] < -0.999).all()


def test_disc_and_letters_face_away_from_the_wall():
    o = fake()
    for nrm in ((0, -1), (1, 0), (0.6, 0.8)):
        n = np.array(nrm, float)
        n /= np.linalg.norm(n)
        t = np.array([-n[1], n[0]])               # CCW edge: outward normal on the right / 外向きは右
        a = np.zeros(2)
        f = B.Facade(Mesh(), a, a + t * 10, n)
        assert np.allclose(f.n, n)
        o.disc(f.m, f, 5.0, 3.0, 0.8, 0.1, "crest")
        o.shield(f.m, f, 2.0, 3.0, 0.8, 0.1, "crest_shield")
        o.wall_text(f.m, f, 10.0, "LV 8", lvss.DEJAVU + "DejaVuSans-Bold.ttf", 0.6, 5.0, 2.0, "txt", 0.1)
        o.wall_text(f.m, f, 10.0, lvss.ZH_NAME[0], lvss.CJK, 0.9, 5.0, 4.0, "txt", 0.1)
        for mat, T, nn in tris(f.m):
            assert (nn[:, :2] @ n).min() > 0.999, mat


def test_solar_rows_face_up():
    o = fake()
    m = Mesh()
    o.solar(m, Polygon([(0, 0), (30, 8), (27, 18), (-3, 10)]), 15.0)
    (mat, T, n), = tris(m)
    assert mat == "solar" and len(T) > 10 and (n[:, 2] > 0.9).all()
    zs = T[:, :, 2]
    assert zs.min() > 15.0 and zs.max() < 15.3


def check_block(o, style, poly, storeys=4, roof="flat_solar", name="test_block", h=15.0, min_walls=10):
    """Main walls (on the footprint outline) face out; roofs, panels and courts
    face up. / 外壁は外向き、屋根は上向き。"""
    poly = orient(poly, 1.0)
    o.footprint = poly
    b = dict(poly=poly, storeys=storeys, style=style, roof=roof, h=h, roof_seen=poly)
    m = Mesh()
    getattr(o, "b_" + style)(m, b, name)
    assert m.ntris() > 20
    ring = poly.exterior
    corners = [Point(q) for q in list(ring.coords)[:-1]]
    ctr = np.array(poly.centroid.coords[0])
    walls = 0
    for mat, T, n in tris(m):
        cen = T.mean(axis=1)
        horiz = np.abs(n[:, 2]) < 0.05
        for c, nn in zip(cen[horiz], n[horiz]):
            pc = Point(c[0], c[1])
            # the returns of recesses at the corners lie on the next edge's
            # line, facing into the recess: skip the 2.6 m round each corner
            # 角の凹部の側面は隣の辺の線上にある: 角から 2.6 m は除外
            if ring.distance(pc) < 0.02 and min(q.distance(pc) for q in corners) > 2.6:
                # outward = away from the polygon, measured just off the face / 面のすぐ外が外側
                p_out = Point(c[0] + nn[0] * 0.05, c[1] + nn[1] * 0.05)
                p_in = Point(c[0] - nn[0] * 0.05, c[1] - nn[1] * 0.05)
                if poly.contains(p_in) and not poly.contains(p_out):
                    walls += 1
                else:
                    raise AssertionError("%s: %s wall faces in at %s" % (style, mat, c))
        if mat.startswith("roof_") or mat == "solar":
            up = n[:, 2]
            assert (up > -0.05).all(), "%s: %s faces down" % (style, mat)
    assert walls >= min_walls, style
    _ = ctr


def test_block_styles_face_out():
    o = fake()
    rect = Polygon([(0, 0), (40, 0), (40, 12), (0, 12)])
    turned = Polygon([(0, 0), (38.4, -11.2), (41.8, 0.3), (3.4, 11.5)])        # the -16.3 deg grid / 傾いた格子
    for poly in (rect, turned):
        check_block(o, "cream", poly)
        check_block(o, "cream", poly, roof="hip_terracotta")
        check_block(o, "cream", poly, storeys=3, roof="gable_terracotta", name="portal_block")
        check_block(o, "brick", poly, storeys=3, roof="hip_terracotta")
        check_block(o, "link", poly, storeys=2, roof="flat")
        check_block(o, "hall", poly, roof="gable_terracotta", h=9.6)
        check_block(o, "sports_hall", poly, h=18.5)
    small = Polygon([(0, 0), (6, 0), (6, 6.3), (0, 6.3)])
    check_block(o, "tower", small, storeys=5, roof="flat", h=19.0, min_walls=4)
    check_block(o, "core", small, roof="flat", min_walls=4)
    check_block(o, "small", Polygon([(0, 0), (8.4, 0), (8.4, 11), (0, 11)]), storeys=1, roof="hip_terracotta",
                min_walls=2)
    check_block(o, "porch", Polygon([(0, 0), (35, 0), (35, 4), (0, 4)]), storeys=1, roof="gable_terracotta",
                min_walls=0)
    check_block(o, "gatehouse", Polygon([(0, 0), (21, 0), (21, 6), (0, 6)]), storeys=2, roof="hip_terracotta")


def test_portal_gable_faces():
    """The portal's gable slab: front faces out of the wall, back faces in. / 門型の妻壁。"""
    o = fake()
    poly = orient(Polygon([(0, 0), (10, 0), (10, 36), (0, 36)]), 1.0)
    m = Mesh()
    o.portal(m, poly, 10.95)
    south = np.array([0.0, -1.0])
    found = 0
    for mat, T, n in tris(m, {"paint_maroon"}):
        cen = T.mean(axis=1)
        sel = (cen[:, 2] > 11.9) & (np.abs(n[:, 2]) < 0.05) & (np.abs(n[:, :2] @ south) > 0.99)
        for c, nn in zip(cen[sel], n[sel]):
            if abs(c[1] - (-1.0)) < 0.01:              # front plane (1 m out) / 前面
                assert nn @ np.r_[south, 0] > 0
                found += 1
            elif abs(c[1] - 0.3) < 0.01:               # back plane (0.3 m in) / 背面
                assert nn @ np.r_[south, 0] < 0
                found += 1
    assert found >= 2
    for mat, T, n in tris(m, {"name_lvss", "zh_lvss", "crest_lvss", "glass_dark"}):
        assert (n[:, 1] < -0.99).all(), mat



def test_chain_link_is_see_through_and_two_sided():
    """Fence wires are thin strips facing both ways; most of the panel is open.
    金網は細い両面の帯で、大部分は透ける。"""
    from shapely.geometry import LineString
    o = fake()
    m = Mesh()
    o.chain_link(m, LineString([(0, 0), (10, 0)]), 0.15, 2.25)
    covered = 0.0
    for mat, T, n in tris(m, {"fence_green"}):
        area = np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1) / 2
        side = n[:, 1]
        assert abs(area[side > 0.99].sum() - area[side < -0.99].sum()) < 1e-6
        covered += area[side > 0.99].sum()
    assert covered < 0.15 * 10 * 2.1


def test_gate_board_faces_the_street():
    """The main gate's name board faces out of the site (the street). / 校名板は道路側を向く。"""
    o = fake()
    o.gate_span = ((0.0, 4.0), (0.3, -4.0))          # gate on the west fence, site to the east / 西の塀の門
    o.site_c = Point(60.0, 0.0)
    m = Mesh()
    o.main_gate(m)
    (mat, T, n), = tris(m, {"plaque_lvss"})
    assert (n[:, 0] < -0.99).all()


if __name__ == "__main__":
    for k, f in sorted(globals().items()):
        if k.startswith("test_") and callable(f):
            f()
            print("ok", k)
