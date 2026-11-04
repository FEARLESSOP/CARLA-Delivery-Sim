"""Tests for the landmark-on-terrain helper (onterrain.py): platforms level and
apart, retaining walls facing the right way (UE culls back faces), banks
facing up, ground meeting the platforms. Run: maps/.venv/bin/python -m pytest
osm_city/landmarks/test_onterrain.py  (or plain python).

地形ヘルパーの試験（面の向き・造成面・擁壁・法面）。
"""
import os
import sys

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onterrain  # noqa: E402
from mesh import Mesh  # noqa: E402


class Slope(object):
    """A plane rising 0.1 m per metre to the east. / 東へ 10 % 上る平面。"""

    def z(self, x, y):
        return 0.1 * np.asarray(x, np.float64) + 0.0 * np.asarray(y, np.float64)


def tris(m, mat):
    V, N, UV, I = m.arrays()[mat]
    T = V[I]
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    return T, n


def site():
    return onterrain.Site(Slope(), box(-50, -50, 50, 50), no_go=box(40, -50, 50, 50))


def test_platform_level_and_apart():
    S = site()
    a = S.platform("a", box(-10, -5, 10, 5), how="max", apron=1.0)
    b = S.platform("b", box(9, -5, 20, 5), how="max", apron=1.0)
    assert abs(a.z - (0.1 * 10 + 0.15)) < 0.05          # highest ground under the footprint
    assert a.poly.intersection(b.poly).area < 1e-6       # never overlapping / 重ならない
    c = S.platform("c", box(35, -5, 45, 5))              # cut by the no-go area / 立入禁止域で切る
    assert c.poly.bounds[2] <= 40 - 0.3 + 1e-6


def test_fill_walls_face_out_cut_walls_face_in():
    S = site()
    p = S.platform("p", box(-10, -5, 10, 5), how="median")
    m = Mesh()
    S.walls(m)
    T, n = tris(m, "retaining_concrete")
    c = np.array([0.0, 0.0])
    mid = T.mean(axis=1)[:, :2]
    out = np.einsum("ij,ij->i", n[:, :2], mid - c)
    west = mid[:, 0] < -9.9            # ground lower than the platform there: fill / 西は盛土
    east = mid[:, 0] > 9.9             # higher: cut / 東は切土
    assert west.any() and east.any()
    assert (out[west] > 0).all()       # fill walls face out / 盛土の擁壁は外向き
    assert (out[east] < 0).all()       # cut walls face in / 切土の擁壁は内向き
    assert np.allclose(n[:, 2], 0.0, atol=1e-9)
    # the fill wall reaches under the outside ground; the cut wall over the bank
    # 盛土擁壁は外の地盤の下まで、切土擁壁は背面の地盤の上まで
    zw = T[west][:, :, 2]
    assert zw.min() < S.ground_z([-10.0], [0.0])[0] - 0.25 and abs(zw.max() - p.z) < 1e-6
    ze = T[east][:, :, 2]
    assert ze.max() >= S.ground_z([10.0], [0.0])[0]


def test_higher_platform_draws_the_shared_wall():
    S = site()
    S.platform("lo", box(-10, -5, 0, 5), z=0.5)
    S.platform("hi", box(0, -5, 10, 5), z=2.0)
    m = Mesh()
    S.walls(m)
    T, n = tris(m, "retaining_concrete")
    mid = T.mean(axis=1)
    shared = np.abs(mid[:, 0]) < 1e-6
    assert shared.any()
    assert (n[shared][:, 0] < 0).all()                      # faces the lower one (west) / 低い側を向く
    assert T[shared][:, :, 2].min() <= 0.5 - onterrain.FOOT + 1e-6       # reaches under the lower one / 低い側の下まで
    assert abs(T[shared][:, :, 2].max() - 2.0) < 1e-6                    # top flush with the higher / 高い側と面一


def test_bank_faces_up_and_stays_in_region():
    S = onterrain.Site(Slope(), box(-60, -50, 50, 50))
    S.platform("field", box(-20, -10, 20, 10), z=3.0, bank=True)
    m = Mesh()
    S.walls(m)
    assert S.banks, "a high fill on the west should become a bank"
    T, n = tris(m, "grass_park")
    assert (n[:, 2] > 0).all()                               # bank faces up / 法面は上向き
    for fp, q in S.banks:
        assert fp.within(S.region.buffer(0.05))
    # on the bank the finished ground is above the natural ground / 法面上は自然地盤より高い
    x, y = -21.0, 0.0
    assert S.surface_z([x], [y])[0] > S.ground_z([x], [y])[0]


def test_ground_flat_on_platform_draped_elsewhere():
    S = site()
    p = S.platform("p", box(-10, -5, 10, 5), how="max")
    m = Mesh()
    S.ground(m, "grass_park", box(-20, -8, 20, 8))
    T, n = tris(m, "grass_park")
    assert (n[:, 2] > 0).all()                               # all facing up / 全て上向き
    c = T.mean(axis=1)
    on = (np.abs(c[:, 0]) < 9.5) & (np.abs(c[:, 1]) < 4.5)
    assert np.allclose(c[on][:, 2], p.z)
    off = np.abs(c[:, 0]) > 11
    g = S.ground_z(c[off][:, 0], c[off][:, 1])
    assert np.abs(c[off][:, 2] - g).max() < 0.15             # draped (linear within a cell) / 地形に沿う


def test_wall_tops_flush_no_slots():
    """Every wall end is flush (within 6 cm) with the higher finished surface
    beside it and reaches below the lower one (no lips, no see-through slots).
    擁壁の天端は高い側と面一、下端は低い側より下。"""
    S = site()
    S.platform("a", box(-10, -5, 10, 5), how="median")
    S.platform("b", box(10, -5, 18, 5), how="max")
    m = Mesh()
    S.walls(m)
    T, n = tris(m, "retaining_concrete")
    V = T.reshape(-1, 3)
    ang = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    for x, y, z in V[::7]:
        zz = S.surface_z(np.r_[x, x + 0.12 * np.cos(ang)], np.r_[y, y + 0.12 * np.sin(ang)])
        assert z <= zz.max() + 0.06 + 1e-6 and z >= zz.min() - onterrain.FOOT - 1e-6


def test_bank_only_inside_region():
    S = onterrain.Site(Slope(), box(-24, -50, 50, 50))
    S.platform("field", box(-20, -10, 20, 10), z=3.0, bank=True)
    m = Mesh()
    S.walls(m)
    for fp, q in S.banks:
        assert fp.within(S.region.buffer(0.05))
    T, n = tris(m, "retaining_concrete")
    assert (T[:, :, 0] >= -24.0 - 1e-6).all()                # walls inside too / 擁壁も範囲内


def test_platform_stays_in_region():
    S = onterrain.Site(Slope(), box(-30, -30, 30, 30))
    p = S.platform("f", box(-40, -10, 10, 10), how="median", bank=True)
    assert p.poly.within(S.region.buffer(0.02))
    m = Mesh()
    S.walls(m)
    T, n = tris(m, "retaining_concrete")
    assert (T[:, :, 0] >= -30.0 - 0.02).all()


def step_faces(m, surf, box_, h=0.3, tol=0.15):
    """Every finished-ground step over ``tol`` between neighbouring cells of an
    h-metre grid must have a face across it (a horizontal ray at mid-height from
    the low cell to the high one hits a triangle). Returns (steps, open).
    段差ごとに面があるか（低い側から水平に光線を飛ばす）。"""
    T = np.concatenate([np.asarray(V)[np.asarray(I)] for V, N, UV, I in m.arrays().values()]).astype(float)
    x0, y0, x1, y1 = box_
    xs, ys = np.arange(x0, x1, h) + 0.0137, np.arange(y0, y1, h) + 0.0071
    GX, GY = np.meshgrid(xs, ys)
    Z = surf(GX.ravel(), GY.ravel()).reshape(GX.shape)
    e1, e2 = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]

    def hit(o, d, L):
        p = np.cross(d, e2)
        det = (e1 * p).sum(1)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
        s = o - T[:, 0]
        u = (s * p).sum(1) * inv
        q = np.cross(s, e1)
        v = (q * d).sum(1) * inv
        t = (e2 * q).sum(1) * inv
        return (ok & (u >= -1e-6) & (v >= -1e-6) & (u + v <= 1 + 1e-6) & (t > 0) & (t < L)).any()
    n = bad = 0
    for dj, di in ((0, 1), (1, 0)):
        za, zb = Z[:Z.shape[0] - dj, :Z.shape[1] - di], Z[dj:, di:]
        for i, j in zip(*np.nonzero(np.abs(zb - za) > tol)):
            a = np.array([GX[i, j], GY[i, j]])
            b = np.array([GX[i + dj, j + di], GY[i + dj, j + di]])
            z1, z2 = Z[i, j], Z[i + dj, j + di]
            lo, hi = (a, b) if z1 < z2 else (b, a)
            n += 1
            bad += not hit(np.r_[lo, (z1 + z2) / 2], np.r_[(hi - lo) / h, 0.0], h)
    return n, bad


def test_bank_ends_closed():
    """A bank that stops beside a kept wall (or meets the next bank at a corner)
    leaves no open step (Agent Checker found see-through triangles). / 法面の端に隙間なし。"""
    S = onterrain.Site(Slope(), box(-36, -40, 40, 40))
    S.platform("field", Polygon([(-25, -12), (35, -12), (35, 12), (-15, 14)]), z=3.0, bank=True)
    m = Mesh()
    S.walls(m)
    S.ground(m, "g", S.free())
    S.ground(m, "g", unary_union([p.poly for p in S.items]))
    assert S.banks and S.bank_ends
    n, bad = step_faces(m, S.surface_z, (-35, -20, 38, 20))
    assert n > 50 and bad == 0, (n, bad)


def test_cap_faces_up():
    m = Mesh()
    onterrain._cap(m, "c", np.array([0.0, 0.0]), np.array([4.0, 0.0]), np.array([0.0, -1.0]), 1.0, 1.0)
    T, n = tris(m, "c")
    assert (n[:, 2] > 0).all()


def test_flat_map_has_no_walls():
    S = onterrain.Site(None, box(-50, -50, 50, 50))
    p = S.platform("p", box(-10, -5, 10, 5))
    assert abs(p.z - onterrain.BASE) < 1e-9
    m = Mesh()
    S.walls(m)
    assert not m.arrays()


def test_steps_split_long_footprint_on_slope():
    S = site()
    parts = S.steps(box(-35, -5, 35, 5), max_drop=1.6)
    assert len(parts) > 1
    assert all(S.drop(q) <= 1.6 + 1e-6 for q, z in parts)
    assert parts[0][1] < parts[-1][1]                        # rising to the east / 東へ上る


if __name__ == "__main__":
    for k, f in sorted(globals().items()):
        if k.startswith("test_"):
            f()
            print("ok", k)
