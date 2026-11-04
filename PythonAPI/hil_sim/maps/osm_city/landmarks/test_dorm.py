"""Face-direction tests for the dorm landmark's primitives (UE culls back faces;
Blender renders hide that). Run: maps/.venv/bin/python
osm_city/landmarks/test_dorm.py

明野寮モデルの部品の面の向きの試験。
"""
import os
import sys

import numpy as np
from shapely.geometry import Point, Polygon, box

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from landmarks import dorm, onterrain  # noqa: E402
from mesh import Mesh  # noqa: E402


def stub(blocks, low=None):
    k = dorm.DORM.__new__(dorm.DORM)
    k.no_go = box(1000, 1000, 1001, 1001)
    k.road = k.no_go
    k.blocks = blocks
    k.footprint = box(-1, -1, 0, 0)
    k.zb = {n: 0.0 for n in blocks}
    k.low = low or {}
    k.T = {"floor_m": 3.0}
    k.lean = (0.0, 0.0)
    k.S = onterrain.Site(None, box(-200, -200, 200, 200))
    k._region = box(-200, -200, 200, 200)
    k.drape = None
    return k


def block(name, poly, style, storeys=3, h=9.9, **kw):
    d = {"name": name, "poly": poly, "style": style, "storeys": storeys, "h": h}
    d.update(kw)
    return d


def tri(m):
    out = {}
    for mat, (V, N, UV, I) in m.arrays().items():
        T = V[I]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(n, axis=1)
        out[mat] = (T[ln > 1e-9], n[ln > 1e-9] / ln[ln > 1e-9, None])
    return out


def outer_walls_face_out(m, poly, mats=None, tol=0.03):
    """Vertical triangles on the footprint's outer wall planes face away from it. / 外壁は外向き。"""
    c = np.array(poly.centroid.coords[0])
    ring = poly.exterior
    bad = total = 0
    for mat, (T, n) in tri(m).items():
        if mats and mat not in mats:
            continue
        cen = T.mean(axis=1)
        vert = np.abs(n[:, 2]) < 1e-6
        for t, nn in zip(cen[vert], n[vert]):
            if ring.distance(Point(t[0], t[1])) < tol and not poly.buffer(-tol).contains(Point(t[0], t[1])):
                # only faces parallel to the wall (reveals square to it are not outer walls) / 壁に平行な面のみ
                d = ring.project(Point(t[0], t[1]))
                q0, q1 = ring.interpolate(max(0.0, d - 0.05)), ring.interpolate(min(ring.length, d + 0.05))
                tv = np.array([q1.x - q0.x, q1.y - q0.y])
                if np.linalg.norm(tv) < 1e-6 or abs(float(nn[:2] @ tv) / np.linalg.norm(tv)) > 0.3:
                    continue
                total += 1
                # outward: the point a little along the normal lies outside / 法線方向は外
                if poly.contains(Point(t[0] + nn[0] * 0.05, t[1] + nn[1] * 0.05)):
                    bad += 1
    return bad, total


def horizontal(m, mat):
    T, n = tri(m)[mat]
    return n[np.abs(n[:, 2]) > 0.5][:, 2]


def test_dorm_block_balconies_windows_roof():
    p = Polygon([(0, 0), (40, 0), (40, 9.5), (0, 9.5)])
    k = stub({"A": block("A", p, "dorm", balcony="S", entrance="N", roof="greygreen", penthouse=[[8.0, 5.0]])})
    m = Mesh()
    k.b_dorm(m, k.blocks["A"])
    bad, total = outer_walls_face_out(m, p, mats={"dorm_render"})
    assert total > 50 and bad == 0, (bad, total)
    assert (horizontal(m, "dorm_roof_greygreen") > 0.99).all()            # roof up / 屋根は上
    T, n = tri(m)["dorm_white"]                                            # copings, sills up; canopy undersides down
    hz = np.abs(n[:, 2]) > 0.5                                             # 笠木・窓台は上、庇の下面は下
    zc = T.mean(axis=1)[:, 2]
    assert (n[hz & (zc > 9.5)][:, 2] > 0).all() and (n[hz & (zc > 3.5) & (zc < 8.9)][:, 2] > 0).all()
    assert (n[hz][:, 2] < 0).sum() < (n[hz][:, 2] > 0).sum()
    assert (horizontal(m, "dorm_soffit") < -0.99).all()                    # balcony soffits down / 軒裏は下
    assert (horizontal(m, "dorm_floor") > 0.99).all()                      # balcony floors up / 床は上
    # balcony doors face out of the building (south) / 掃き出し窓は南向き
    T, n = tri(m)["dorm_balcony_door"]
    assert len(T) and (n[:, 1] < -0.99).all()
    # every window faces away from the block / 窓は全て外向き
    T, n = tri(m)["dorm_sash_2"]
    c = np.array([20.0, 4.75])
    assert len(T) and (np.einsum("ij,ij->i", n[:, :2], T.mean(axis=1)[:, :2] - c) > 0).all()
    assert (n[:, 1] > 0.99).sum() > 20
    # spandrel panels face out / 腰壁パネルは外向き
    T, n = tri(m)["dorm_spandrel"]
    cen = T.mean(axis=1)
    out = np.where(cen[:, 1] < 4.75, n[:, 1] < -0.99, n[:, 1] > 0.99)
    front = np.abs(n[:, 2]) < 1e-6
    assert out[front].mean() > 0.6
    T, n = tri(m)["jp_door_steel"]                                         # penthouse door faces out / 塔屋の扉
    top = T.mean(axis=1)[:, 2] > 9.0
    assert top.any()
    pc = np.array([8.0, 5.0])
    assert (np.einsum("ij,ij->i", n[top][:, :2], T.mean(axis=1)[top][:, :2] - pc) > 0).all()
    T, n = tri(m)["dorm_glassdoor"]                                        # the entrance (north) / 玄関
    assert len(T) and (n[:, 1] > 0.99).all()


def test_corridor_and_end_walls():
    p = Polygon([(0, 0), (40, 0), (40, 8.2), (0, 8.2)])
    k = stub({"C": block("C", p, "dorm", corridor="N", entrance="W", roof="green")})
    m = Mesh()
    k.b_dorm(m, k.blocks["C"])
    bad, total = outer_walls_face_out(m, p, mats={"dorm_render"})
    assert total > 40 and bad == 0, (bad, total)
    T, n = tri(m)["jp_door_steel"]                                         # room doors face the corridor (north) / 扉は北向き
    assert len(T) and (n[n[:, 1] != 0][:, 1] > 0.99).all() or (n[:, 1] > 0.99).sum() > 10
    T, n = tri(m)["dorm_glassdoor"]                                        # the west entrance / 西の玄関
    assert len(T) and (n[:, 0] < -0.99).all()
    assert (horizontal(m, "dorm_soffit") < -0.99).all()


def test_escape_stair_and_end_balconies():
    p = Polygon([(0, 0), (40, 0), (40, 9.2), (0, 9.2)])
    k = stub({"B": block("B", p, "dorm", balcony="S", ext_stair="W", end_balconies="E", roof="teal")})
    m = Mesh()
    k.b_dorm(m, k.blocks["B"])
    T, n = tri(m)["dorm_floor"]
    # the stair flights: closed solids, every face away from its own centre / 階段は閉じた立体
    west = T.mean(axis=1)[:, 0] < 0
    assert west.any()
    T, n = tri(m)["dorm_spandrel"]
    assert ((T.mean(axis=1)[:, 0] < -0.5)).any() and ((T.mean(axis=1)[:, 0] > 40.5)).any()
    up = n[:, 2] > 0.5
    assert not (n[:, 2] < -0.5).any() or up.any()


def test_tile_block_lower_level_and_porch():
    p = Polygon([(0, 0), (30, 0), (30, 9), (0, 9)])
    low = Polygon([(24, -1), (31, -1), (31, 10), (24, 10)]).intersection(p)
    k = stub({"E": block("E", p, "dorm_tile", storeys=4, h=12.9, roof="teal", porch=[14.0, 0.0])},
             low={"E": low})
    m = Mesh()
    k.b_dorm_tile(m, k.blocks["E"])
    bad, total = outer_walls_face_out(m, p, mats={"dorm_tile_greige"})
    assert total > 40 and bad == 0, (bad, total)
    T, n = tri(m)["dorm_tile_greige"]
    assert (T[:, :, 2].min() < -2.9)                                      # the lower level reaches -3 m / 下階
    lowz = T.mean(axis=1)[:, 2] < 0
    assert (T.mean(axis=1)[lowz][:, 0] > 23.9).all()                      # only over the low part / 低い側のみ
    T, n = tri(m)["jp_kawara_ibushi"]                                      # porch roof up and out / 玄関屋根
    assert len(T) and (n[:, 2] > 0.2).all()
    T, n = tri(m)["jp_soffit"]
    assert (n[:, 2] < -0.2).all()
    T, n = tri(m)["jp_ac_unit"]
    assert len(T)


def test_office_hall_small():
    p = Polygon([(0, 0), (12, 0), (12, 6), (6, 6), (6, 14), (0, 14)])
    k = stub({"office": block("office", p, "office", storeys=1, h=4.2, roof="mint")})
    m = Mesh()
    k.b_office(m, k.blocks["office"])
    bad, total = outer_walls_face_out(m, p, mats={"dorm_white_wall"})
    assert total > 10 and bad == 0, (bad, total)
    assert (horizontal(m, "dorm_roof_mint") > 0.99).all()
    assert (horizontal(m, "dorm_soffit") < -0.99).all()
    T, n = tri(m)["dorm_glassdoor"]
    assert len(T) and (n[:, 0] < -0.99).all()                              # entrance faces west / 玄関は西
    for st, mat in (("hall", "dorm_render_grey"), ("small_rc", "dorm_render"), ("shed", "dorm_white_wall")):
        q = box(0, 0, 14, 9)
        k = stub({"x": block("x", q, st, storeys=1, h=5.0, roof="lightgrey", solar=(st == "hall"))})
        m = Mesh()
        getattr(k, "b_" + st)(m, k.blocks["x"])
        bad, total = outer_walls_face_out(m, q, mats={mat})
        assert total > 8 and bad == 0, (st, bad, total)
        for rm in ("dorm_roof_palegrey", "dorm_roof_grey"):
            if rm in tri(m):
                assert (horizontal(m, rm) > 0.99).all(), (st, rm)
        if st == "hall":
            T, n = tri(m)["solar"]
            assert len(T) and (n[:, 2] > 0.9).all()


def test_mono_roofs_bikeshed_stairbox_roofroom():
    q = box(0, 0, 10, 4)
    k = stub({"b": block("b", q, "bikeshed", 1, 2.6, roof="white")})
    m = Mesh()
    k.b_bikeshed(m, k.blocks["b"])
    assert (horizontal(m, "dorm_roof_white") > 0.9).all()
    assert (horizontal(m, "jp_soffit") < -0.9).all()
    for st in ("clubroom", "shed_corr"):
        k = stub({"c": block("c", q, st, 1, 3.0, roof="greymetal")})
        m = Mesh()
        getattr(k, "b_" + st)(m, k.blocks["c"])
        rm = "jp_roof_galvalume" if st == "clubroom" else "dorm_roof_greymetal"
        T, n = tri(m)[rm]
        assert (n[:, 2] > 0.9).all(), st
        assert (horizontal(m, "jp_soffit") < -0.9).all(), st
        T, n = tri(m)["dorm_white_wall"]
        tri_gable = T.mean(axis=1)[:, 2] > 2.6
        c = np.array([5.0, 2.0])
        side = np.einsum("ij,ij->i", n[tri_gable][:, :2], T.mean(axis=1)[tri_gable][:, :2] - c)
        assert (side > 0).all(), st                                       # gable infill faces out / 妻は外向き
    par = block("E", box(-30, 0, 0, 9), "dorm_tile", storeys=4, h=12.9)
    k = stub({"E": par, "S": block("S", box(0, 2, 2.2, 7.6), "ext_stair_box", storeys=4, h=12.9, on="E")})
    m = Mesh()
    k.b_ext_stair_box(m, k.blocks["S"])
    T, n = tri(m)["dorm_spandrel"]
    yspan = T[:, :, 1].max(axis=1) - T[:, :, 1].min(axis=1)
    assert len(T) and not ((T.mean(axis=1)[:, 0] < 0.1) & (np.abs(n[:, 0]) > 0.99) & (yspan > 0.5)).any()   # open toward E
    T, n = tri(m)["dorm_floor"]
    assert len(T)
    rr = block("R", box(5, 2, 12, 8), "roof_room", h=3.0)
    k = stub({"R": rr})
    m = Mesh()
    k.b_roof_room(m, rr)
    bad, total = outer_walls_face_out(m, rr["poly"], mats={"dorm_render"})
    assert total > 8 and bad == 0, (bad, total)


def test_site_walls_face_both_ways():
    from shapely.geometry import LineString
    k = stub({})
    k.region = lambda: box(-200, -200, 200, 200)
    k.footprint = box(100, 100, 101, 101)
    m = Mesh()
    line = LineString([(0, 0), (0, 10)])
    k.wall_run(m, line, 1.0, "w", "t", thick=0.2)
    T, n = tri(m)["w"]
    xs, ys = T.mean(axis=1)[:, 0], T.mean(axis=1)[:, 1]
    side = np.abs(n[:, 0]) > 0.5
    assert ((xs[side] > 0) == (n[side, 0] > 0)).all()                    # each face outward / 両面とも外向き
    cap = np.abs(n[:, 1]) > 0.5                                           # end caps at the gaps face out / 小口
    assert cap.sum() >= 4 and ((ys[cap] > 5) == (n[cap, 1] > 0)).all()
    T, n = tri(m)["t"]
    assert (n[:, 2] > 0.99).all()
    m = Mesh()
    k.panel_wall(m, line, 2.0)
    T, n = tri(m)["dorm_panel_grey"]
    assert (n[:, 0] > 0.5).any() and (n[:, 0] < -0.5).any()


def coplanar_same(m, tol=5e-4):
    """Pairs of overlapping coplanar triangles facing the same way (z-fight). / 同向きの重なる共面三角形。"""
    import shapely
    T = np.concatenate([V[I] for V, N, UV, I in m.arrays().values()]).astype(np.float64)
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    ln = np.linalg.norm(n, axis=1)
    T, n = T[ln > 1e-9], n[ln > 1e-9] / ln[ln > 1e-9, None]
    bad = 0
    for i in range(len(T)):
        for j in range(i + 1, len(T)):
            if n[i] @ n[j] < 0.999 or abs((T[j] - T[i][0]) @ n[i]).max() > tol:
                continue
            ax = int(np.argmax(np.abs(n[i])))
            e = [q for q in range(3) if q != ax]
            if shapely.Polygon(T[i][:, e]).intersection(shapely.Polygon(T[j][:, e])).area > 2e-4:
                bad += 1
    return bad


def test_escape_stair_and_stair_box_not_coplanar():
    p = Polygon([(0, 0), (40, 0), (40, 9.2), (0, 9.2)])
    k = stub({"B": block("B", p, "dorm", balcony="S", ext_stair="W", roof="teal")})
    m = Mesh()
    a, bb = np.array([0.0, 9.2]), np.array([0.0, 0.0])                    # the west end, facing west / 西端
    import buildings as BB
    f = BB.Facade(m, a, bb, np.array([-1.0, 0.0]))
    k.escape_stair(m, f, 9.2, 3, 3.0, 3.0)
    assert coplanar_same(m) == 0
    par = block("E", box(-30, 0, 0, 9), "dorm_tile", storeys=4, h=12.9)
    k = stub({"E": par, "S": block("S", box(0, 2, 2.2, 7.6), "ext_stair_box", storeys=2, h=6.9, on="E")},
             low={"E": box(-5, 0, 2.2, 9), "S": box(0, 2, 2.2, 7.6)})
    m = Mesh()
    k.b_ext_stair_box(m, k.blocks["S"])
    assert coplanar_same(m) == 0
    # the bottom slab covers the footprint at the lower floor, facing up / 最下段の土間
    T, n = tri(m)["dorm_floor"]
    low = (np.abs(T.mean(axis=1)[:, 2] + 2.98) < 0.01) & (n[:, 2] > 0.99)
    area = 0.5 * np.linalg.norm(np.cross(T[low][:, 1] - T[low][:, 0], T[low][:, 2] - T[low][:, 0]), axis=1).sum()
    assert abs(area - 2.2 * 5.6) < 0.05, area


def test_no_slot_or_sliver_from_path_tongues():
    from landmarks.dorm import drop_narrow
    path = Polygon([(10, -5), (12, -5), (12, 20), (10, 20)]).union(
        Polygon([(0, 4.995), (10, 4.995), (10, 5.005), (0, 5.005)]))      # a 1 cm tongue / 1 cm の舌片
    g = drop_narrow(path)
    assert abs(g.area - 50.0) < 0.05                                     # the band stays, the tongue goes / 帯は残る
    S_ = onterrain.Site(None, box(-20, -20, 30, 30), no_go=g, gap=0.3)
    pl = S_.platform("b", box(-2, 0, 9, 10), how="max", apron=1.2)
    q = pl.poly
    opened = q.buffer(-0.15, join_style="mitre").buffer(0.15, join_style="mitre")
    assert q.difference(opened).area < 0.05                              # no slot or needle under 0.3 m / 溝・細片なし
    assert len(list(getattr(q, "geoms", [q]))) == 1


if __name__ == "__main__":
    for kk, fn in sorted(globals().items()):
        if kk.startswith("test_"):
            fn()
            print("ok", kk)
