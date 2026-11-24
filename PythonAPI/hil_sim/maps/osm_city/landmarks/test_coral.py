"""Tests for the Coral Primary School primitives: every face points the way it
is seen (UE culls back faces), names CARLA keeps (maps/.venv).

  .venv/bin/python osm_city/landmarks/test_coral.py

珊瑚小学の基本形状の面の向きと名前を確認する。
"""

import os
import sys
import tempfile

import numpy as np
from shapely.geometry import LineString, Polygon, box
from shapely.geometry.polygon import orient

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import buildings as B  # noqa: E402
from landmarks import coral  # noqa: E402
from mesh import Mesh  # noqa: E402


def tris(m, mat=None):
    """(centroids, unit winding normals, materials) of every triangle. / 三角形の重心・法線・材質。"""
    C, N, M = [], [], []
    for k, (V, _, _, I, _) in m.parts.items():
        if mat is not None and k != mat or not V:
            continue
        Vc, Ic = np.concatenate(V), np.concatenate(I)
        if len(Ic) == 0:
            continue
        T = Vc[Ic]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        keep = np.linalg.norm(n, axis=1) > 1e-10
        C.append(T[keep].mean(axis=1))
        N.append(n[keep] / np.linalg.norm(n[keep], axis=1)[:, None])
        M += [k] * int(keep.sum())
    if not C:
        return np.zeros((0, 3)), np.zeros((0, 3)), []
    return np.concatenate(C), np.concatenate(N), M


def check(name, ok):
    print(("PASS " if ok else "FAIL ") + name)
    return bool(ok)


def facade():
    """A facade along +x with its outside towards -y (n = (0, -1)). / x 方向の外壁（外は -y）。"""
    m = Mesh()
    return m, B.Facade(m, np.array([0.0, 0.0]), np.array([20.0, 0.0]), np.array([0.0, -1.0]))


def main():
    good = True
    # wall with openings: the wall faces out, reveals face into the hole, glass out
    # 開口付きの壁: 壁は外向き、見込みは開口の内向き、ガラスは外向き
    m, f = facade()
    opens = [(2.0, 4.0, 1.0, 2.5), (6.0, 8.0, 1.0, 2.5)]
    coral.wall_openings(f, "wall", 0.0, 10.0, 0.0, 3.5, opens)
    for o in opens:
        coral.opening(f, *o, "rev", "glass", 0.15, sill=False)
    C, N, _ = tris(m, "wall")
    good &= check("wall faces out (%d)" % len(C), np.all(N[:, 1] < -0.99))
    area = sum(0.5 * np.linalg.norm(np.cross(*(lambda T: (T[1] - T[0], T[2] - T[0]))(t)))
               for t in np.concatenate(m.parts["wall"][0])[np.concatenate(m.parts["wall"][3])])
    good &= check("wall area = gross - openings (%.2f)" % area, abs(area - (35.0 - 6.0)) < 1e-6)
    C, N, _ = tris(m, "glass")
    good &= check("glass faces out", np.all(N[:, 1] < -0.99))
    # stacked openings (a column of windows sharing one span, plus a row): wall
    # area = gross - openings and no wall face over any opening
    # 縦に重なる開口: 壁面積 = 全体 - 開口、開口の上に壁が無い
    m, f = facade()
    opens = [(2.0, 4.0, z, z + 1.2) for z in (1.0, 4.5, 8.0)] + [(6.0, 9.0, 1.0, 2.5), (6.5, 8.5, 5.0, 6.0)]
    coral.wall_openings(f, "wall", 0.0, 12.0, 0.0, 10.5, opens)
    C, N, _ = tris(m, "wall")
    V = np.concatenate(m.parts["wall"][0])[np.concatenate(m.parts["wall"][3])]
    area = float(np.sum(0.5 * np.linalg.norm(np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]), axis=1)))
    gross = 12.0 * 10.5 - sum((b - a) * (z1 - z0) for a, b, z0, z1 in opens)
    inside = [any(a + 1e-6 < c[0] < b - 1e-6 and z0 + 1e-6 < c[2] < z1 - 1e-6 for a, b, z0, z1 in opens) for c in C]
    good &= check("stacked openings: wall area %.2f = %.2f, none over a window" % (area, gross),
                  abs(area - gross) < 1e-6 and not any(inside) and np.all(N[:, 1] < -0.99))
    # window frames inside the reveal face out of the frame bars / 窓枠
    m, f = facade()
    coral.opening(f, 2.0, 4.0, 1.0, 2.5, "wall", "coral_glass_yf", 0.2)
    C, N, _ = tris(m, "coral_frame_y")
    good &= check("window frame present and inside the reveal (%d tris)" % len(C),
                  len(C) > 20 and np.all((C[:, 1] > -0.001) & (C[:, 1] < 0.2 + 1e-6)))
    C, N, _ = tris(m, "rev")
    ok = True
    for c, n in zip(C, N):
        o = next(o for o in opens if o[0] - 0.01 <= c[0] <= o[1] + 0.01)
        mid = np.array([(o[0] + o[1]) / 2, c[1], (o[2] + o[3]) / 2])
        ok &= float(n @ (mid - c)) > 0
    good &= check("reveals face into the opening (%d)" % len(C), ok)

    # open corridor: back wall out, soffit down, floor up, end caps into the recess,
    # parapet front out and its back towards the corridor
    # 開放廊下: 奥壁は外向き、天井は下、床は上、端壁は凹みの内側、腰壁の表裏
    obj = coral.CORAL.__new__(coral.CORAL)
    m, f = facade()
    obj.corridor(f, 3.0, 17.0, 20.0, 3, 3.5, 10.65, parapets=("pa", "pb"), pier="pier")
    C, N, _ = tris(m, "coral_corr_wall")
    wallf = np.abs(N[:, 1]) > 0.5                     # (the rest are door / window reveals) / 残りは見込み
    good &= check("corridor wall faces out", wallf.sum() > 10 and np.all(N[wallf, 1] < -0.99))
    C, N, _ = tris(m, "coral_orange")
    good &= check("corridor soffit faces down", len(C) and np.all(N[:, 2] < -0.99))
    C, N, _ = tris(m, "concrete")
    flo = C[:, 2] > 0.5
    good &= check("corridor floors face up", np.all(N[flo, 2] > 0.99))
    C, N, _ = tris(m, "coral_white")
    caps = np.abs(N[:, 0]) > 0.99
    ok = np.all((N[caps, 0] > 0) == (C[caps, 0] < 10.0))
    good &= check("recess end walls face into the recess (%d)" % int(caps.sum()), caps.any() and ok)
    C, N, _ = tris(m, "pa")
    front = C[:, 1] < 0.07
    good &= check("parapet front out, back in", np.all(N[front & (np.abs(N[:, 1]) > 0.5), 1] < 0)
                  and np.all(N[~front & (np.abs(N[:, 1]) > 0.5), 1] > 0))

    # gable roof: slopes up and away from the ridge, gable ends out, soffit down
    # 切妻: 斜面は上向きで棟から外、妻は外、軒裏は下
    m = Mesh()
    poly = orient(box(0, 0, 30, 12), 1.0)
    coral.gable(m, poly, 10.0, 13.0, "roof", "wall", ov=0.5)
    C, N, _ = tris(m, "roof")
    slope = np.abs(N[:, 2]) > 0.5
    good &= check("gable slopes and seam tops up and out", np.all(N[slope, 2] > 0.1)
                  and np.all(N[slope, 1] * (C[slope, 1] - 6.0) > 0))
    # the seam's centre line: from each slope's own eave corner (x -0.5 south, 30.5 north)
    # はぜの中心線（各斜面の軒の角から）
    xs, south = C[~slope, 0], C[~slope, 1] < 6.0
    xc = np.where(south, -0.2 + np.round((xs + 0.2) / 0.6) * 0.6, 30.2 - np.round((30.2 - xs) / 0.6) * 0.6)
    good &= check("standing seam sides face out of the seam (%d)" % int((~slope).sum()),
                  (~slope).sum() > 100 and np.all(N[~slope, 0] * (C[~slope, 0] - xc) > 0))
    C, N, _ = tris(m, "wall")
    good &= check("gable ends face out", np.all(N[:, 0] * (C[:, 0] - 15.0) > 0))
    C, N, _ = tris(m, "concrete")
    good &= check("eaves soffit faces down", np.all(N[:, 2] < -0.99))
    C, N, _ = tris(m, "coral_white")
    good &= check("fascia faces out", np.all(N[:, 1] * (C[:, 1] - 6.0) > 0))

    # flat roof with parapet / パラペット付き陸屋根
    m = Mesh()
    coral.flat_roof(m, poly, 10.0, "par", parapet=0.9, mat="roofc", cap="cap")
    C, N, _ = tris(m, "par")
    ctr = np.array([15.0, 6.0])
    import shapely
    outer = ~shapely.contains_xy(poly.buffer(-0.1), C[:, 0], C[:, 1])
    radial = np.sum((C[:, :2] - ctr) * N[:, :2], axis=1)
    good &= check("parapet outer walls out, inner walls in",
                  np.all(radial[outer] > 0) and np.all(radial[~outer] < 0))
    C, N, _ = tris(m, "roofc")
    C2, N2, _ = tris(m, "cap")
    good &= check("roof and parapet cap face up", np.all(N[:, 2] > 0.99) and np.all(N2[:, 2] > 0.99))

    # chain-link: the same number of strips each way, front +n, back -n
    # 金網: 表裏同数（表は +n、裏は -n）
    m = Mesh()
    coral.chainlink(m, "mesh", (0, 0), (6, 0), 0.2, 2.0)
    C, N, _ = tris(m, "mesh")
    f_ = N[:, 1] < -0.9
    b_ = N[:, 1] > 0.9
    good &= check("chain-link two-sided (%d tris)" % len(C), f_.sum() == b_.sum() and f_.sum() + b_.sum() == len(C)
                  and np.all(C[f_, 1] < 0) and np.all(C[b_, 1] > 0))
    good &= check("chain-link inside its panel", np.all((C[:, 0] >= -1e-6) & (C[:, 0] <= 6 + 1e-6)
                                                        & (C[:, 2] > 0.18) & (C[:, 2] < 2.02)))

    # text along a curve faces out of the curve (CCW edges: outside on the right)
    # 曲線に沿う文字は外向き
    m = Mesh()
    ang = np.linspace(-2.2, -0.9, 12)                 # the south arc of a CCW circle, west to east / 南の円弧
    line = LineString([(40 * np.cos(a), 40 * np.sin(a)) for a in ang])
    coral.text_along(m, line, "CORAL PRIMARY SCHOOL", line.length / 2, 7.5, 0.88, "txt", 0.8)
    C, N, _ = tris(m, "txt")
    radial = np.sum(C[:, :2] * N[:, :2], axis=1) / np.linalg.norm(C[:, :2], axis=1)
    good &= check("curved lettering faces out (%d tris)" % len(C), len(C) > 50 and np.all(radial > 0.95))
    good &= check("curved lettering on the wall line", np.all(np.abs(np.linalg.norm(C[:, :2], axis=1) - 40) < 1.6))

    # the drum: cylinder and windows out / 円筒の階段塔
    obj = coral.CORAL.__new__(coral.CORAL)
    obj.drum_c, obj.drum_r = (5.0, 5.0), 2.6
    obj.blocks = {"hall": {"poly": orient(box(8, 0, 30, 10), 1.0)}}
    m = Mesh()
    obj.drum(m)
    C, N, _ = tris(m)
    side = np.abs(N[:, 2]) < 0.5
    radial = np.sum((C[side, :2] - np.array([5.0, 5.0])) * N[side, :2], axis=1)
    good &= check("drum walls and windows face out", np.all(radial > 0))
    C, N, _ = tris(m, "coral_glass_dk")
    good &= check("drum windows on the side away from the hall", np.all(C[:, 0] < 5.0))

    # a whole classroom block: faces on the footprint face out, the roof up
    # 教室棟全体: 外周上の面は外向き
    obj = coral.CORAL.__new__(coral.CORAL)
    obj.frames = {"S": ((0.0, 0.0), 0.0), "H": ((0.0, 0.0), 0.0)}
    p = orient(box(0, 0, 50, 14), 1.0)
    obj.blocks = {"north": dict(poly=p, top=14.15, kind="classroom", roof="gable"),
                  "middle": dict(poly=orient(box(0, -40, 50, -26), 1.0), top=14.15, kind="classroom", roof="gable")}
    for k in ("north", "middle"):
        m = Mesh()
        obj.block(m, k, obj.blocks[k])
        C, N, Ms = tris(m)
        pb = obj.blocks[k]["poly"]
        bd = shapely.distance(shapely.points(C[:, :2]), pb.exterior)
        on = (bd < 0.004) & (np.abs(N[:, 2]) < 0.01)
        outward = ~shapely.contains_xy(pb, C[on, 0] + N[on, 0] * 0.05, C[on, 1] + N[on, 1] * 0.05)
        good &= check("%s block: %d faces on the footprint all face out (%d tris)" % (k, int(on.sum()), len(C)),
                      on.sum() > 20 and np.all(outward))

    # every other block kind, side by side as on the site / 他の棟の種類も同様に
    curved = orient(Polygon([(0, -60), (50, -60), (50, -66), (45, -70), (25, -73), (5, -70), (0, -66)]), 1.0)
    obj.blocks = {
        "spine": dict(poly=orient(box(50, -60, 64, 14), 1.0), top=16.35, kind="spine", roof="flat"),
        "curved": dict(poly=curved, top=10.95, kind="curved", roof="gable", bar=orient(box(0, -66, 50, -60), 1.0)),
        "west_wing": dict(poly=orient(box(-20, -80, 0, -62), 1.0), top=11.85, kind="westwing", roof="flat"),
        "spine_s": dict(poly=orient(box(64, -20, 76, -8), 1.0), top=11.85, kind="bluewing", roof="flat"),
        "hall": dict(poly=orient(box(70, -80, 120, -46), 1.0), top=12.0, kind="hall", roof="hall"),
        "store": dict(poly=orient(box(-40, -90, -32, -75), 1.0), top=3.2, kind="store", roof="gable"),
    }
    for k in obj.blocks:
        m = Mesh()
        obj.block(m, k, obj.blocks[k])
        C, N, Ms = tris(m)
        pb = obj.blocks[k]["poly"]
        bd = shapely.distance(shapely.points(C[:, :2]), pb.exterior)
        on = (bd < 0.004) & (np.abs(N[:, 2]) < 0.01)
        outward = ~shapely.contains_xy(pb, C[on, 0] + N[on, 0] * 0.05, C[on, 1] + N[on, 1] * 0.05)
        good &= check("%s: %d faces on the footprint all face out (%d tris)" % (k, int(on.sum()), len(C)),
                      on.sum() > 4 and np.all(outward))
        roofs = (C[:, 2] > obj.blocks[k]["top"] + 0.6) & (np.abs(N[:, 2]) > 0.3)
        good &= check("%s: roof faces up" % k, np.all(N[roofs, 2] > 0))

    # every material name the textures write is one CARLA keeps / 材質名に light / sign を含まない
    d = tempfile.mkdtemp()
    mats = coral.write_textures(d)
    bad = [k for k in mats if "light" in k.lower() or "sign" in k.lower()]
    good &= check("texture names CARLA keeps (%d)" % len(mats), not bad)
    print("ALL PASS" if good else "SOME FAILED")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
