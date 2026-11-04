"""Face-direction tests for the 7-Eleven landmark's primitives (UE culls back
faces; Blender renders hide that). Run: maps/.venv/bin/python
osm_city/landmarks/test_seven.py

セブン-イレブンのモデル部品の面の向きの試験。
"""
import os
import sys

import numpy as np
from shapely.geometry import Point, box

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from landmarks import onterrain, seven  # noqa: E402
from mesh import Mesh  # noqa: E402
import buildings as B  # noqa: E402

TRACE = {"solar_m": {"from_west": 4.2, "from_north": 2.9, "length": 12.4, "depth": 6.2, "notch_east": 1.6,
                     "notch_depth": 3.1}, "walkway_m": {"north": 1.0, "west": 1.0}}


def stub(store=box(0, 0, 20.7, 9.7), lot=box(-25, -12, 45, 30)):
    k = seven.SEVEN.__new__(seven.SEVEN)
    k.no_go = box(1000, 1000, 1001, 1001)
    k.T = dict(TRACE)
    k.blocks = {"store": {"name": "store", "poly": store, "h": 5.0},
                "annex": {"name": "annex", "poly": box(21.1, 1.5, 22.9, 8.7), "h": 2.7}}
    k.site = lot
    k.S = onterrain.Site(None, lot)
    k.lot = k.S.platform("lot", lot, z=0.15)
    k.z = 0.15
    k.drape = None
    k.kanban, k.upole = (-20.0, 25.0), (-20.0, -8.0)
    return k


def tri(m):
    out = {}
    for mat, (V, N, UV, I) in m.arrays().items():
        T = V[I].astype(np.float64)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(n, axis=1)
        out[mat] = (T[ln > 1e-9], n[ln > 1e-9] / ln[ln > 1e-9, None])
    return out


def away(T, n, c):
    """Share of triangles facing away from point c (2-D or 3-D). / 点 c から外向きの割合。"""
    c = np.asarray(c, np.float64)
    d = T.mean(axis=1)[:, :len(c)] - c
    return float(np.mean(np.einsum("ij,ij->i", n[:, :len(c)], d) > -1e-9))


def test_store_walls_out_roof_up():
    k = stub()
    m = Mesh()
    k.store(m, k.blocks["store"])
    p = k.blocks["store"]["poly"]
    T, n = tri(m)["seven_panel"]
    cen = T.mean(axis=1)
    vert = np.abs(n[:, 2]) < 1e-6
    on_wall = np.array([p.exterior.distance(Point(c[0], c[1])) < 0.03 for c in cen]) & vert & (cen[:, 2] < 4.5)
    assert on_wall.sum() > 20
    assert away(T[on_wall], n[on_wall], (10.35, 4.85)) == 1.0          # facades face out / 外壁は外向き
    for mat in ("seven_roof", "seven_white"):
        T, n = tri(m)[mat]
        flat = np.abs(n[:, 2]) > 0.5
        under = np.abs(T.mean(axis=1)[:, 2] - seven.BAND[0]) < 1e-6   # the band's underside / 帯の下面
        assert (n[flat & ~under][:, 2] > 0).all(), mat                # roof, copings, canopy top up / 上向き
        assert (n[flat & under][:, 2] < 0).all(), mat                 # band underside down / 帯の下面は下向き
    T, n = tri(m)["seven_soffit"]
    assert (n[:, 2] < -0.99).all()                                     # canopy underside down / 庇裏は下向き
    T, n = tri(m)["seven_band"]
    front = np.abs(n[:, 1] - 1.0) < 1e-6
    assert front.sum() >= 2 and (T[front].mean(axis=1)[:, 1] > 9.7).all()   # north band faces north / 北向き
    west = np.abs(n[:, 0] + 1.0) < 1e-6
    assert west.sum() >= 2 and (T[west].mean(axis=1)[:, 0] < 0).all()       # west band faces west / 西向き
    T, n = tri(m)["seven_shop"]
    assert (n[:, 1] > 0.99).sum() > 10 and ((n[:, 1] > 0.99) | (n[:, 0] < -0.99)).all()   # glass faces out
    T, n = tri(m)["seven_alu"]
    # window reveals face into the opening: none faces into the wall body / 枠の見込みは開口側
    cen = T.mean(axis=1)
    inside = np.array([p.buffer(-0.2).contains(Point(c[0], c[1])) for c in cen]) & (cen[:, 2] < 4.4)
    assert not inside.any()
    T, n = tri(m)["solar"]
    assert (n[:, 2] > 0.9).all()                                       # panels face up / パネルは上向き


def test_band_corner_closed_and_out():
    m = Mesh()
    f = B.Facade(m, np.array([10.0, 5.0]), np.array([0.0, 5.0]), np.array([0.0, 1.0]))
    stub().band(f, 0.0, 10.12, ends=(True, True))
    T, n = tri(m)["seven_band"]
    assert away(T, n, (5.0, 5.0)) == 1.0
    T, n = tri(m)["seven_panel"]
    assert (n[:, 2] > 0.99).all()                                      # band top up / 上面は上向き
    T, n = tri(m)["seven_white"]
    assert (n[:, 2] < -0.99).all()                                     # band underside down / 下面は下向き


def test_opening_reveals_face_inward():
    m = Mesh()
    f = B.Facade(m, np.array([0.0, 0.0]), np.array([4.0, 0.0]), np.array([0.0, -1.0]))
    stub().opening(f, 1.0, 3.0, 0.5, 2.5, "fr", "gl", sill=False)
    T, n = tri(m)["gl"]
    assert (n[:, 1] < -0.99).all()                                     # glass faces out (south) / ガラスは外向き
    T, n = tri(m)["fr"]
    c = np.array([2.0, -0.06, 1.5])
    assert (np.einsum("ij,ij->i", n, c - T.mean(axis=1)) > 0).all()    # reveals face the opening / 見込みは開口向き


def test_block_courses_and_caps():
    lot = box(0, 0, 10, 10)
    k = stub(lot=lot)

    class D(object):                                                   # the ground falls 2 m to the east / 東へ下がる
        def z(self, x, y):
            return -0.2 * np.asarray(x, np.float64)
    k.S = onterrain.Site(D(), lot.buffer(3.0))
    k.lot = k.S.platform("lot", lot, z=0.15)
    k.z = 0.15
    m = Mesh()
    k.block_courses(m)
    T, n = tri(m)["jp_block_wall"]
    assert len(T) > 4 and away(T, n, (5.0, 5.0)) == 1.0               # courses face out / 外向き
    T, n = tri(m)["seven_coping"]
    assert len(T) > 4 and (n[:, 2] > 0.99).all()                      # copings face up / 笠木は上向き


def test_fence_two_sided_and_kanban_out():
    k = stub()
    m = Mesh()
    from shapely.geometry import LineString
    k.mesh_fence(m, LineString([(0, 0), (6, 0)]), 0.15, 1.2)
    T, n = tri(m)["seven_fence"]
    vert = np.abs(n[:, 2]) < 1e-6
    ny = n[vert][:, 1]
    assert (ny > 0.99).sum() == (ny < -0.99).sum() > 0                 # every strip both ways / 両面
    m = Mesh()
    k.kanban_pole(m)
    x, y = k.kanban
    for mat in ("seven_kanban_brand", "seven_kanban_goods"):
        T, n = tri(m)[mat]
        assert away(T, n, (x, y)) == 1.0 and (np.abs(n[:, 1]) > 0.99).all(), mat   # faces N and S / 南北向き
    T, n = tri(m)["seven_kanban_white"]
    box_part = T.mean(axis=1)[:, 2] > 4.6
    assert away(T[box_part], n[box_part], (x, y, 6.1)) == 1.0


def test_vending_bins_stops_pole_out():
    k = stub()
    m = Mesh()
    k.store(m, k.blocks["store"])                                     # sets the faces / 面の情報
    m = Mesh()
    k.door_things(m)
    for mat in ("seven_vend_white", "seven_vend_red", "seven_vend_blue"):
        T, n = tri(m)[mat]
        assert (n[:, 0] < -0.99).all(), mat                           # machine fronts face west / 西向き
    T, n = tri(m)["seven_bins"]
    assert (n[:, 1] > 0.99).all()                                     # bin fronts face north / 北向き
    k.bays = [((5.0, 12.0), (5.0, 16.0), 2.5)]
    k.lot = k.S.platform("lot2", box(-25, -12, 45, 30), z=0.15) or k.lot
    m = Mesh()
    k.lot_things(m)
    T, n = tri(m)["seven_carstop"]
    assert len(T) == 2 * 10                                           # two stops, 10 triangles each / 2 個
    for sgn in (-1, 1):
        one = (T.mean(axis=1)[:, 0] - 5.0) * sgn > 0
        assert away(T[one], n[one], (5.0 + sgn * 0.45, 12.75, 0.055)) == 1.0   # each closed, outward / 外向き
    m = Mesh()
    k.utility_pole(m)
    T, n = tri(m)["seven_pole_concrete"]
    x, y = k.upole
    side = np.abs(n[:, 2]) < 0.5
    assert away(T[side], n[side], (x, y)) == 1.0


def test_boundary_block_wall_closed_and_out():
    k = stub(lot=box(0, 0, 30, 20))
    k.site = box(0, 0, 30, 20)
    k.blocks = {}
    k.kanban = k.upole = None
    m = Mesh()
    k.fences(m)
    T, n = tri(m)["jp_block_wall"]
    cen = T.mean(axis=1)
    north = cen[:, 1] > 19.0
    assert north.sum() > 4
    # the north wall: 0.12 m thick, both faces outward from its centre line y = 19.85 / 両面とも外向き
    yc = 20.0 - 0.15
    side = np.abs(n[:, 1]) > 0.99
    assert (np.sign(n[north & side][:, 1]) == np.sign(cen[north & side][:, 1] - yc)).all()
    assert (np.abs(np.abs(cen[north & side][:, 1] - yc) - 0.06) < 1e-6).all()
    T, n = tri(m)["seven_coping"]
    assert (n[:, 2] > 0.99).all()


def test_slivers_leave_no_pits():
    """10 mm footway slivers in the lot must not punch holes into the platform. / 細片で造成面に穴が開かない。"""
    from shapely.ops import unary_union
    lot = box(0, 0, 60, 30)
    band = box(-2, 0, -0.2, 30)                                       # a real footway beside the lot / 本物の歩道
    slivers = unary_union([box(50, 10, 51.4, 10.01), box(20, 25, 20.6, 25.012), box(5, 2, 5.01, 3.3)])
    no_go = seven.drop_slivers(unary_union([band, slivers]))
    assert no_go.equals(band)                                         # slivers dropped, the band unchanged / 歩道はそのまま
    S = onterrain.Site(None, lot.difference(no_go), no_go=no_go, gap=seven.GAP)
    p = S.platform("lot", lot, z=0.15)
    assert all(len(q.interiors) == 0 for q in onterrain.polys(p.poly)) and p.poly.area > 1796.9  # only the 0.1 m strip by the band / 歩道際のみ


def test_coping_once_and_door_reveal():
    lot = box(0, 0, 10, 10)
    k = stub(lot=lot)

    class D(object):
        def z(self, x, y):
            return -0.3 * np.asarray(x, np.float64)
    k.S = onterrain.Site(D(), lot.buffer(3.0))
    k.lot = k.S.platform("lot", lot, z=0.15)
    m = Mesh()
    k.block_courses(m)
    T, n = tri(m)["seven_coping"]
    from shapely.geometry import Polygon
    ps = [Polygon(t[:, :2]) for t in T]
    tot = sum(p.area for p in ps)
    from shapely.ops import unary_union
    assert abs(unary_union(ps).area - tot) < 1e-6                     # no two coping triangles overlap / 重なり無し
    m = Mesh()
    f = B.Facade(m, np.array([0.0, 0.0]), np.array([4.0, 0.0]), np.array([0.0, -1.0]))
    k.opening(f, 1.0, 3.0, 0.0, 2.7, "fr", "gl", sill=False)
    T, n = tri(m)["fr"]
    assert not (np.abs(T[:, :, 2]).max(axis=1) < 1e-9).any()          # no reveal on the floor / 床面に枠無し


def test_ac_units_on_the_ground_by_the_south_wall():
    k = stub(lot=box(-25, -1.3, 45, 30))                             # 1.3 m strip south of the store / 南の帯
    m = Mesh()
    k.store(m, k.blocks["store"])
    assert len(k.ac_ground) >= 4
    T, n = tri(m).get("jp_ac_unit", (np.zeros((0, 3, 3)), np.zeros((0, 3))))
    assert len(T) == 0 or (T.mean(axis=1)[:, 2] < 4.5).all()          # none left on the roof / 屋上に無い
    m = Mesh()
    k.ground_ac(m)
    T, n = tri(m)["jp_ac_unit"]
    cen = T.mean(axis=1)
    assert (cen[:, 1] < 0).all() and (cen[:, 1] > -1.3).all()         # between the wall and the lot edge / 壁と敷地端の間
    for u0, u1, w1, h in k.ac_ground:                                   # every unit closed and outward / 外向き
        c = np.array([(u0 + u1) / 2, -(0.05 + w1) / 2, 0.08 + h / 2])     # u runs east from the SW corner
        one = np.abs(cen[:, 0] - c[0]) < (u1 - u0) / 2 + 1e-6
        assert away(T[one], n[one], c) == 1.0
    k2 = stub(lot=box(-25, -0.3, 45, 30))                            # no room: roof / 余地無し: 屋上
    m = Mesh()
    k2.store(m, k2.blocks["store"])
    T, n = tri(m)["jp_ac_unit"]
    assert not k2.ac_ground and (T.mean(axis=1)[:, 2] > 4.5).all()


def test_u_divider_and_symbol():
    g = seven.u_divider((0, 0), (0, 4.4))
    assert g.is_valid and 0.9 < g.area < 1.4 and g.bounds[3] > 4.6   # two 0.12 m lines + the bend / 二重線
    w = seven.wheelchair((0, 0), 1.3)
    assert w.is_valid and w.area > 0.2


if __name__ == "__main__":
    for k, f in sorted(globals().items()):
        if k.startswith("test_"):
            f()
            print("ok", k)
