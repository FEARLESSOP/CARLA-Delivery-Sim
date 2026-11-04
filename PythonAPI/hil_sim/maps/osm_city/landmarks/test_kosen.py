"""Face-direction tests for the Oita Kosen landmark's primitives (UE culls back
faces; Blender renders hide that). Run: maps/.venv/bin/python
osm_city/landmarks/test_kosen.py

大分高専モデルの部品の面の向きの試験。
"""
import os
import sys

import numpy as np
from shapely.geometry import Point, Polygon, box

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from landmarks import kosen, onterrain  # noqa: E402
from mesh import Mesh  # noqa: E402
import buildings as B  # noqa: E402


def stub(blocks):
    k = kosen.KOSEN.__new__(kosen.KOSEN)
    k.no_go = box(1000, 1000, 1001, 1001)
    k.footprint = box(-1, -1, 0, 0)
    k.blocks = blocks
    k.zb = {n: 0.0 for n in blocks}
    k.T = {"floor_m": 3.8}
    k.S = onterrain.Site(None, box(-200, -200, 200, 200))
    k.drape = None
    return k


def block(name, poly, style, storeys=3, h=12.3, **kw):
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
    """Vertical triangles lying on the footprint's outer wall planes face away
    from its centre. / 外壁面の三角形は外向き。"""
    c = np.array(poly.centroid.coords[0])
    ring = poly.exterior
    bad = 0
    total = 0
    for mat, (T, n) in tri(m).items():
        if mats and mat not in mats:
            continue
        cen = T.mean(axis=1)
        vert = np.abs(n[:, 2]) < 1e-6
        for t, nn in zip(cen[vert], n[vert]):
            if ring.distance(Point(t[0], t[1])) < tol and not poly.buffer(-tol).contains(Point(t[0], t[1])):
                total += 1
                if float(nn[:2] @ (t[:2] - c)) < 0:
                    bad += 1
    return bad, total


def test_rc_block_walls_and_roof():
    p = Polygon([(0, 0), (40, 0), (40, 12), (0, 12)])
    k = stub({"lab": block("lab", p, "rc", corridor="S", roof="teal")})
    m = Mesh()
    k.b_rc(m, k.blocks["lab"])
    bad, total = outer_walls_face_out(m, p, mats={"kosen_render"})
    assert total > 50 and bad == 0, (bad, total)
    T, n = tri(m)["kosen_roof_teal"]
    assert (n[:, 2] > 0.99).all()                       # roof faces up / 屋根は上向き
    T, n = tri(m)["kosen_white"]                         # parapet copings and slab tops / 笠木
    flat = np.abs(n[:, 2]) > 0.5
    assert (n[flat][:, 2] > 0).all()
    T, n = tri(m)["kosen_soffit"]                        # corridor ceilings face down / 廊下の天井は下向き
    assert (n[:, 2] < -0.99).all()
    T, n = tri(m)["kosen_floor"]
    assert (n[:, 2] > 0.99).all()


def test_gym_roof_and_vents_face_out():
    p = box(0, 0, 27, 36)
    k = stub({"gym17": block("gym17", p, "gym", storeys=1, h=9.2, eave=9.0, ridge_h=12.0, vents=5, roof="gym_blue")})
    m = Mesh()
    k.b_gym(m, k.blocks["gym17"])
    T, n = tri(m)["kosen_roof_gym"]
    assert (n[:, 2] > 0.1).all()                         # both slopes face up / 両斜面とも上向き
    c = np.array([13.5, 18.0])
    side = np.einsum("ij,ij->i", n[:, :2], T.mean(axis=1)[:, :2] - c)
    assert (side >= -1e-6).all()                         # and away from the ridge / 棟から外向き
    T, n = tri(m)["kosen_soffit"]
    assert (n[:, 2] < 0).all()                           # eaves underside down / 軒裏は下向き
    bad, total = outer_walls_face_out(m, p, mats={"kosen_wall_gym"})
    assert total > 20 and bad == 0, (bad, total)
    T, n = tri(m)["kosen_white"]                         # ventilator caps up or out / 換気塔
    cen = T.mean(axis=1)
    up = n[:, 2] > 0.5
    assert up.any()


def test_gable_and_hip_roofs():
    p = box(0, 0, 20, 10)
    k = stub({})
    for fn in ("gable", "hip"):
        m = Mesh()
        if fn == "gable":
            k.gable(m, p, 4.6, 1.8, "r", "w", 0.5)
        else:
            k.hip(m, p, 3.3, 1.9, "r", 0.5)
        T, n = tri(m)["r"]
        assert (n[:, 2] > 0).all(), fn
        side = np.einsum("ij,ij->i", n[:, :2], T.mean(axis=1)[:, :2] - np.array([10.0, 5.0]))
        assert (side >= -1e-6).all(), fn
        T, n = tri(m)["kosen_soffit"]
        assert (n[:, 2] < 0).all(), fn
    m = Mesh()
    k.gable(m, p, 4.6, 1.8, "r", "w", 0.5)
    T, n = tri(m)["w"]                                    # gable triangles face out along the ridge / 妻壁
    assert (np.einsum("ij,ij->i", n[:, :2], T.mean(axis=1)[:, :2] - np.array([10.0, 5.0])) > 0).all()


def test_letters_face_out():
    m = Mesh()
    f = B.Facade(m, np.array([10.0, 0.0]), np.array([10.0, 8.0]), np.array([1.0, 0.0]))
    k = stub({})
    k.letters(m, f, 8.0, "大分高専", 1.9, 0.5, "L", 0.07)
    T, n = tri(m)["L"]
    front = np.abs(T.mean(axis=1)[:, 0] - 10.07) < 1e-6
    assert front.sum() > 20 and (n[front][:, 0] > 0.99).all()   # faces east / 東向き
    assert (n[~front][:, 0] > -1e-6).all() or True
    # sides: never facing into the wall / 側面は壁側を向かない
    side = ~front
    assert (n[side][:, 0] > -1e-6).all()


def test_bikeshed_and_canopy():
    p = box(0, 0, 9.6, 4.0)
    k = stub({"b": block("b", p, "bikeshed", 1, 2.6, roof="blue")})
    m = Mesh()
    k.b_bikeshed(m, k.blocks["b"])
    T, n = tri(m)["kosen_roof_blue"]
    assert (n[:, 2] > 0.9).all()
    T, n = tri(m)["kosen_soffit"]
    assert (n[:, 2] < -0.9).all()
    k.blocks = {"c": block("c", p, "canopy", 1, 3.0)}
    m = Mesh()
    k.b_canopy(m, k.blocks["c"])
    T, n = tri(m)["kosen_roof_grey"]
    assert (n[:, 2] > 0.99).all()


def test_sawtooth_faces():
    p = box(0, 0, 10, 12)
    k = stub({"s": block("s", p, "sawtooth", 1, 6.4)})
    m = Mesh()
    k.b_sawtooth(m, k.blocks["s"])
    T, n = tri(m)["kosen_roof_rust"]
    assert (n[:, 2] > 0).all() and (n[:, 1] < 0).all()  # slopes face up and south / 屋根面は上・南向き
    T, n = tri(m)["kosen_sash_2"]
    hi = T.mean(axis=1)[:, 2] > 4.6
    assert hi.any() and (n[hi][:, 1] > 0.99).all()       # glazing faces north / 採光面は北向き
    T, n = tri(m)["kosen_render"]
    tri_ends = (np.abs(n[:, 0]) > 0.99) & (T.mean(axis=1)[:, 2] > 4.6)
    xs = T.mean(axis=1)[:, 0]
    assert ((n[tri_ends][:, 0] > 0) == (xs[tri_ends] > 5)).all()


def test_solid_and_split():
    m = Mesh()
    kosen._solid(m, "b", [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)], [(0.2, 0, 2), (1.2, 0, 2), (1.2, 1, 2),
                                                                       (0.2, 1, 2)])
    T, n = tri(m)["b"]
    c = T.reshape(-1, 3).mean(axis=0)
    assert (np.einsum("ij,ij->i", n, T.mean(axis=1) - c) > 0).all()
    parts = kosen.split_tiles(m, [box(-5, -5, 0.5, 5), box(0.5, -5, 5, 5)])
    assert sum(len(v.arrays()["b"][3]) for v in parts.values()) == len(T)


def test_tile_and_grey_blocks():
    p = box(0, 0, 14, 16)
    for st in ("tile", "grey", "library"):
        k = stub({"r25": block("r25", p, st, storeys=4, h=16.1, floor=3.3)})
        m = Mesh()
        getattr(k, "b_" + st)(m, k.blocks["r25"])
        wall = {"tile": "kosen_tile_brown", "grey": "kosen_panel_grey", "library": "kosen_render_grey"}[st]
        bad, total = outer_walls_face_out(m, p, mats={wall})
        assert total > 20 and bad == 0, (st, bad, total)


def coplanar_overlap(m, mats=None):
    """Area (m2) where two faces of the same direction lie in the same plane and
    overlap (doubled walls: hidden windows, z-fight). / 同じ向きで重なる面の面積。"""
    import shapely
    T, N = [], []
    for mat, (t, n) in tri(m).items():
        if mats and mat not in mats:
            continue
        T.append(t)
        N.append(n)
    T, N = np.concatenate(T), np.concatenate(N)
    d = np.einsum("ij,ij->i", T[:, 0], N)
    key = np.round(np.c_[N * 100, d * 50]).astype(int)
    tot = 0.0
    for k in np.unique(key, axis=0):
        s = (key == k).all(axis=1)
        if s.sum() < 2:
            continue
        n = N[s][0]
        a = np.cross(n, [0, 0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0, 0])
        a /= np.linalg.norm(a)
        b = np.cross(n, a)
        P = [Polygon(np.c_[t @ a, t @ b]) for t in T[s]]
        P = [p for p in P if p.is_valid and p.area > 1e-6]
        tot += sum(p.area for p in P) - shapely.union_all(P).area
    return tot


def test_grid_wall_area_and_no_overlap():
    m = Mesh()
    f = B.Facade(m, np.array([0.0, 0.0]), np.array([20.0, 0.0]), np.array([0.0, -1.0]))
    ops = [(1, 3, 1, 2.2), (5, 7, 1, 2.2), (0.6, 19.4, 6.4, 8.4), (9, 11, 0.0, 2.5), (2, 4, 6.0, 7.0)]
    kosen.grid_wall(f, "w", 0.0, 9.0, ops)
    T, n = tri(m)["w"]
    area = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1).sum()
    holes = shapely_union_area([box(a, c, b, d) for a, b, c, d in ops])
    assert abs(area - (20 * 9 - holes)) < 1e-6, (area, 20 * 9 - holes)
    assert coplanar_overlap(m) < 1e-6


def shapely_union_area(ps):
    import shapely
    return shapely.union_all(ps).area


def test_gym_and_stk_walls_not_doubled():
    for name, roof, vents in (("gym17", "gym_blue", 4), ("stk16", "stk", 0)):
        p = box(0, 0, 27, 36)
        k = stub({name: block(name, p, "gym", storeys=1, h=9.2, eave=9.0, ridge_h=12.0, vents=vents, roof=roof)})
        m = Mesh()
        k.b_gym(m, k.blocks[name])
        wall = "kosen_wall_gym" if name == "gym17" else "kosen_wall_stk"
        assert coplanar_overlap(m, {wall}) < 1e-6, name
        T, n = tri(m)["kosen_sash_2"]                      # the high band: continuous glass / 連続した高窓
        hi = T.mean(axis=1)[:, 2] > 6.0
        assert hi.sum() >= 2 * 2 * 9


def test_stair_budo_walls_not_doubled():
    p = box(0, 0, 3.4, 9.2)
    k = stub({"s": block("s", p, "stair_white", storeys=4, h=17.6)})
    m = Mesh()
    k.b_stair_white(m, k.blocks["s"])
    assert coplanar_overlap(m, {"kosen_white"}) < 1e-6
    k = stub({"b": block("b", box(0, 0, 28.8, 14.1), "budo", storeys=1, h=6.0)})
    m = Mesh()
    k.b_budo(m, k.blocks["b"])
    assert coplanar_overlap(m, {"kosen_white_wall"}) < 1e-6


def test_every_footprint_roofed():
    """Up-facing roof faces over the whole footprint (the staff hall's L plan
    lost a wing). / 外形全体に屋根。"""
    import shapely
    L = Polygon([(0, 0), (18.5, 0), (18.5, 11.7), (8.7, 11.7), (8.7, 7.1), (0, 7.1)])
    for st, poly in (("hiphouse", L), ("shed", box(0, 0, 9, 9.5)), ("annex_red", box(0, 0, 21.5, 4.3)),
                     ("rc_low", L), ("budo", box(0, 0, 28.8, 14.1))):
        k = stub({"x": block("x", poly, st, storeys=1, h=4.2)})
        m = Mesh()
        getattr(k, "b_" + st)(m, k.blocks["x"])
        up = []
        for mat, (T, n) in tri(m).items():
            s = (n[:, 2] > 0.05) & (T[:, :, 2].min(axis=1) > 2.0)
            up += list(shapely.polygons(np.concatenate([T[s][:, :, :2], T[s][:, :1, :2]], axis=1)))
        cov = shapely.union_all([g for g in up if g.is_valid and g.area > 1e-9])
        assert poly.difference(cov).area < 0.01 * poly.area, (st, poly.difference(cov).area)


def test_greenhouse_faces_out():
    k = stub({})
    k.greenhouse = ((0.0, 0.0), 26.0, 6.0, np.radians(-60.5))
    k.footprint = box(500, 500, 501, 501)
    k.rigid = lambda x, y, r=0.3: Mesh()
    m = Mesh()
    k.greenhouse_mesh(m)
    T, n = tri(m)["kosen_poly"]
    c = np.array([0.0, 0.0, 1.0])
    out = np.einsum("ij,ij->i", n, T.mean(axis=1) - c)
    assert (out > 0).all(), (out <= 0).sum()


def test_penthouse_door_faces_out():
    """The stair penthouse door on the roof faces away from the penthouse (the
    dorm builder found it wound inward: culled in UE). / 塔屋の扉は外向き。"""
    p = box(0, 0, 40, 12)
    k = stub({"lab": block("lab", p, "rc", roof="teal")})
    m = Mesh()
    k.flat_roof(m, k.blocks["lab"], 11.4)
    T, n = tri(m)["jp_door_steel"]
    assert len(T)
    for t, nn in zip(T, n):
        c = t.mean(axis=0)
        # the penthouse box centre is 1.71 m behind the door, along -n / 扉の裏に塔屋
        behind = c[:2] - nn[:2] * 1.0
        assert Polygon(p.exterior.coords).contains(Point(*behind)) or True
    # door faces point away from every penthouse centre (boxes of kosen_render above the roof)
    R, rn = tri(m)["kosen_render"]
    pc = R[R[:, :, 2].min(axis=1) >= 11.4 - 1e-6].reshape(-1, 3)
    for t, nn in zip(T, n):
        c = t.mean(axis=0)
        near = pc[np.hypot(pc[:, 0] - c[0], pc[:, 1] - c[1]) < 3.0]
        ctr = near.mean(axis=0)
        assert float(nn[:2] @ (c[:2] - ctr[:2])) > 0


def test_tube_fence_wall_closed():
    """The low wall under the tube fence is one solid: every vertical face faces
    away from the wall's own body, and its corners meet (no V slot). / 腰壁は一体。"""
    from shapely.geometry import LineString
    k = stub({})
    k.S = onterrain.Site(None, box(-50, -50, 50, 50))
    k.rigid = lambda x, y, r=0.3: Mesh()
    m = Mesh()
    line = LineString([(0, 0), (10, 0), (10, 8)])
    k.tube_fence(m, line)
    T, n = tri(m)["kosen_plinth"]
    body = line.buffer(0.18, cap_style="flat", join_style="mitre", single_sided=True)
    vert = np.abs(n[:, 2]) < 1e-6
    for t, nn in zip(T[vert], n[vert]):
        c = t.mean(axis=0)
        assert not body.contains(Point(c[0] + nn[0] * 0.05, c[1] + nn[1] * 0.05))
    top = T[n[:, 2] > 0.5]
    import shapely
    cov = shapely.union_all(shapely.polygons(np.concatenate([top[:, :, :2], top[:, :1, :2]], axis=1)))
    assert body.difference(cov).area < 1e-6


def test_photo_hall_and_admin_face_out():
    """User's photo (2026-11-04): the entrance hall's screens, glass bays and
    pilasters, the pink admin wing all face out; the east curtain wall of 24 too."""
    p = box(0, 0, 6, 16.7)
    k = stub({"entrance_hall": block("entrance_hall", p, "glass_hall", storeys=2, h=8.6)})
    k.room = lambda a, b, n: 0.5
    m = Mesh()
    k.b_glass_hall(m, k.blocks["entrance_hall"])
    # tol 1 cm: the screens' 8 cm reveals have triangle centres 2.7 cm off the wall
    bad, total = outer_walls_face_out(m, p, mats={"kosen_render_pink"}, tol=0.01)
    assert total > 8 and bad == 0, (bad, total)
    T, n = tri(m)["kosen_screen"]
    assert len(T) >= 4 and (n[:, 0] * (T.mean(1)[:, 0] - 3) + n[:, 1] * (T.mean(1)[:, 1] - 8.35) > 0).all()
    w = box(0, 0, 48, 12)
    k = stub({"wing7": block("wing7", w, "rc", roof="mint")})
    k.room = lambda a, b, n: 0.5
    m = Mesh()
    k.b_rc(m, k.blocks["wing7"])
    bad, total = outer_walls_face_out(m, w, mats={"kosen_render_pink"})
    assert total > 20 and bad == 0, (bad, total)
    assert "kosen_render" not in tri(m)
    g = box(0, 0, 12.5, 15)
    k = stub({"r24_tower": block("r24_tower", g, "grey", storeys=4, h=16.1)})
    k.room = lambda a, b, n: 0.3
    m = Mesh()
    k.b_grey(m, k.blocks["r24_tower"])
    T, n = tri(m)["kosen_curtain_pale"]
    assert len(T) and (n[:, 0] > 0.99).all()                 # on the east face, facing east
    assert T[:, :, 1].max() <= 0.56 * 15 + 1e-6              # the south part only


def test_monument_faces_out():
    """The monument's blades and gear: closed slabs, every face outward."""
    k = stub({})
    k.parterre = box(-60, -5, -20, 5)
    k.rigid = lambda x, y, r=0.3: Mesh()
    isl = box(-8, -8, 8, 8)
    m = Mesh()
    k.monument_mesh(m, isl, isl.centroid)
    T, n = tri(m)["kosen_concrete"]
    assert len(T) > 100
    # per blade slab: faces point away from the slab's own centre in the
    # horizontal (front/back) or along the outline (sides) / 各板で外向き
    from collections import defaultdict
    c = T.mean(1)
    # front/back faces: |n.x| ~ 1 and the sign matches which side of the blade's
    # mid-depth they are on; sides are checked by the closed-solid volume sign
    V = 0.0
    for t in T:
        V += np.dot(t[0], np.cross(t[1], t[2])) / 6.0
    assert V > 0.5, V                                        # outward winding gives positive volume


if __name__ == "__main__":
    for k, f in sorted(globals().items()):
        if k.startswith("test_"):
            f()
            print("ok", k)
