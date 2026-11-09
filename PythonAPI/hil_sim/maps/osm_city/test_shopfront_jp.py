"""Tests for shopfront_jp.py: faces point outward (UE culls back faces),
textures read unmirrored, props stand on their base, doors stay inside the
shopfront recess, and the placement rules (ground classes, road / tactile
clearance, poles, door openings, the walkable footway).

  maps/.venv/bin/python osm_city/test_shopfront_jp.py      (or with pytest)

店先の小物のテスト（面の向き・接地・入口の凹み・配置規則）。
"""
import math
import os
import sys

import numpy as np

try:
    import pytest
except ImportError:                                  # the venv has no pytest: a small stand-in / 代用
    class _Approx(object):
        def __init__(self, v, rel=None, abs=None):
            self.v, self.rel, self.abs = v, rel, abs

        def __eq__(self, o):
            tol = max(self.abs or 0.0, (self.rel or (1e-6 if self.abs is None else 0.0)) * abs(self.v))
            return abs(o - self.v) <= tol

    class _Mark(object):
        @staticmethod
        def parametrize(name, values):
            def deco(fn):
                fn._params = (name, values)
                return fn
            return deco

    class pytest(object):                            # noqa: N801
        mark = _Mark()

        @staticmethod
        def approx(v, rel=None, abs=None):
            return _Approx(v, rel, abs)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shopfront_jp as S  # noqa: E402
from mesh import Mesh  # noqa: E402

ANGLES = (0.0, 37.0, 200.0, 291.0)


def frame(deg, z0=1.25, o=(3.0, -2.0)):
    a = math.radians(deg)
    t = np.array([math.cos(a), math.sin(a)])
    n = np.array([t[1], -t[0]])                      # CCW footprint: n = t turned clockwise / 時計回り
    return S.Frame(o, t, n, z0)


def tris(m):
    out = []
    for mat, (V, N, UV, I) in m.arrays().items():
        for f in I:
            P = V[f].astype(np.float64)
            out.append((mat, P, N[f[0]].astype(np.float64), UV[f].astype(np.float64)))
    return out


def wind_normal(P):
    n = np.cross(P[1] - P[0], P[2] - P[0])
    return n / (np.linalg.norm(n) + 1e-12)


def signed_volume(m, origin):
    """Divergence theorem about a point on the open bottom plane: > 0 and about
    the solid's volume when every face points out. / 外向きなら正の体積。"""
    v = 0.0
    for _, P, _, _ in tris(m):
        c = P.mean(0) - origin
        n = np.cross(P[1] - P[0], P[2] - P[0]) / 2.0
        v += float(c @ n) / 3.0
    return v


@pytest.mark.parametrize("deg", ANGLES)
def test_winding_matches_normals(deg):
    F = frame(deg)
    m = Mesh()
    S.jihanki(m, F, 0.0, "red")
    S.kago(m, F, 0.8, "red")
    S.kanban_a(m, F, 2.0, 0.6, "chalk")
    S.kanban_stand(m, F, 3.0, 0.5)
    S.hachi(m, F, 4.0, 0.3, np.random.default_rng(1))
    S.planter(m, F, 5.0, 0.2, np.random.default_rng(2))
    S.nobori(m, F, 6.0, 0.7, "red", 1)
    for mat, P, N, _ in tris(m):
        if np.linalg.norm(np.cross(P[1] - P[0], P[2] - P[0])) < 1e-9:
            continue
        assert wind_normal(P) @ N > 0.5, mat          # lumpy blobs: quads are not planar / 非平面の四角形


@pytest.mark.parametrize("deg", ANGLES)
def test_box_faces_out(deg):
    F = frame(deg)
    m = Mesh()
    S.lbox(m, F, {"all": "x"}, -0.5, 0.5, 0.1, 0.8, 0.0, 1.8)
    c = F.p(0.0, 0.45, 0.9)
    for _, P, _, _ in tris(m):
        assert wind_normal(P) @ (P.mean(0) - c) > 0
    vol = signed_volume(m, F.p(0.0, 0.45, -0.02))
    assert vol == pytest.approx(1.0 * 0.7 * 1.82, rel=0.01)


@pytest.mark.parametrize("deg", ANGLES)
def test_props_positive_volume(deg):
    """Every closed-ish prop has a positive signed volume near its real one
    (an inverted face would subtract twice). / 小物の符号付き体積が正。"""
    F = frame(deg)
    m = Mesh()
    S.jihanki(m, F, 0.0, "blue")
    v = signed_volume(m, F.p(0.0, S.GAP + 0.36, -0.02))
    W, D, H = S.JIHANKI
    A = 0.92 * 0.74                                  # the lit window, 3 cm proud, open at its back / 見本窓
    assert v == pytest.approx(W * D * (H + 0.02) + A * 0.03 + A * (D / 2) / 3, rel=0.01)
    m = Mesh()
    S.frustum(m, F, "pot", 0.0, 0.5, 0.0, 0.3, 0.12, 0.16, top_mat="soil")
    for mat, P, _, _ in tris(m):
        c = F.p(0.0, 0.5, P.mean(0)[2] - F.z0)
        n = wind_normal(P)
        if mat == "soil":
            assert n[2] > 0.99
        elif P[:, 2].min() < F.z0:                  # outer side faces (from the foot) / 外側面
            assert n @ (P.mean(0) - c) > 0
        else:                                        # the rim's inner band faces in and up / 縁の内側
            assert n @ (P.mean(0) - c) < 0 and n[2] > 0
    m = Mesh()
    S.blob(m, F, "leaf", 0.0, 0.5, 0.6, 0.3, 0.3, 0.2, np.random.default_rng(3))
    c = F.p(0.0, 0.5, 0.6)
    for _, P, _, _ in tris(m):
        assert wind_normal(P) @ (P.mean(0) - c) > 0


@pytest.mark.parametrize("deg", ANGLES)
def test_textures_unmirrored(deg):
    """Seen from the side a face points to, texture u grows to the viewer's
    right (the frame is left-handed). / 見る側から右へ u が増える。"""
    F = frame(deg)
    for out in (1, -1):
        m = Mesh()
        S.vface(m, F, "x", -0.5, 0.5, 0.3, 0.0, 1.0, out)
        N = F.v(0, out, 0)
        right = np.cross(-N, (0, 0, 1))
        for _, P, _, UV in tris(m):
            assert wind_normal(P) @ N > 0.99
            k0, k1 = np.argmin(UV[:, 0]), np.argmax(UV[:, 0])
            if UV[k1, 0] - UV[k0, 0] > 0.5:
                assert (P[k1] - P[k0]) @ right > 0
    for out in (1, -1):
        m = Mesh()
        S.sface(m, F, "x", 0.4, 0.0, 0.7, 0.0, 1.0, out)
        N = F.v(out, 0, 0)
        right = np.cross(-N, (0, 0, 1))
        for _, P, _, UV in tris(m):
            assert wind_normal(P) @ N > 0.99
            k0, k1 = np.argmin(UV[:, 0]), np.argmax(UV[:, 0])
            if UV[k1, 0] - UV[k0, 0] > 0.5:
                assert (P[k1] - P[k0]) @ right > 0


@pytest.mark.parametrize("deg", ANGLES)
def test_two_sided_panels(deg):
    """A-board, banner: as many faces look along +n as along -n. / 両面。"""
    F = frame(deg)
    for build in (lambda m: S.kanban_a(m, F, 0.0, 0.6, "menu"), lambda m: S.nobori(m, F, 0.0, 0.7, "blue", -1)):
        m = Mesh()
        build(m)
        mats = {}
        for mat, P, _, _ in tris(m):
            if mat.startswith(("kanban_a_", "nobori_")) and mat not in ("nobori_pole", "nobori_base"):
                s = np.sign(wind_normal(P) @ F.v(0, 1, 0))
                mats.setdefault(mat, []).append(s)
        for mat, ss in mats.items():
            assert ss.count(1.0) == ss.count(-1.0) > 0, mat


@pytest.mark.parametrize("deg", ANGLES)
def test_ground_contact_and_heights(deg):
    """Feet 2 cm into the ground, nothing lower; tops within the heights the
    placement tests the air for. / 接地（2 cm 埋め込み）と高さ。"""
    F = frame(deg, z0=4.0)
    cases = [(lambda m: S.jihanki(m, F, 0, "white"), S.JIHANKI[2] + 0.001),
             (lambda m: S.kago(m, F, 0, "white"), S.BIN[2] + 0.001),
             (lambda m: S.kanban_a(m, F, 0, 0.6, "chalk"), 0.97),
             (lambda m: S.kanban_stand(m, F, 0, 0.5), 1.17),
             *[(lambda m, k=k: S.hachi(m, F, 0, 0.3, np.random.default_rng(k), k % 2 == 0), 1.6) for k in range(40)],
             (lambda m: S.planter(m, F, 0, 0.2, np.random.default_rng(6), 1.1), 0.75),
             (lambda m: S.nobori(m, F, 0, 0.7, "green", 1), 2.85)]
    for build, top in cases:
        m = Mesh()
        build(m)
        Z = np.concatenate([V[:, 2] for V, _, _, _ in m.arrays().values()])
        assert Z.min() >= F.z0 - 0.021
        assert Z.min() <= F.z0 + 1e-6
        assert Z.max() <= F.z0 + top


def test_door_unit_in_recess():
    """Door leaf, frame and noren stay behind the facade plane (w <= 0), the
    leaf faces out. / 入口は外壁面より内側。"""
    from buildings_jp import Facade
    for kind in ("auto", "koshi", "glass1"):
        m = Mesh()
        a, b = np.array([2.0, 1.0]), np.array([10.0, 7.0])
        L = np.linalg.norm(b - a)
        t = (b - a) / L
        n = np.array([t[1], -t[0]])
        f = Facade(m, a, b, n)
        S.door_unit(m, f, 4.0, 1.8, 0.25, 0.3, kind, "eatery", np.random.default_rng(4))
        leaf = {"auto": "jp_door_auto", "koshi": "jp_door_koshi", "glass1": "jp_door_glass1"}[kind]
        for mat, P, _, _ in tris(m):
            w = (P[:, :2] - a) @ n
            assert w.max() <= 1e-6, (kind, mat)
            if mat == leaf:
                assert wind_normal(P) @ np.array([n[0], n[1], 0]) > 0.99
        nor = [np.sign(wind_normal(P) @ np.array([n[0], n[1], 0])) for mat, P, _, _ in tris(m)
               if mat.startswith("jp_noren_") and mat != "jp_noren_rod"]
        if kind == "koshi":
            assert nor and nor.count(1.0) == nor.count(-1.0)


# -- placement rules on a synthetic street / 合成した通りで配置規則 ----------------
def scene(walk_w=3.0, lot_w=0.0, tree=None):
    """A straight facade along x (n = -y), a lot strip, a footway walk_w wide
    (tactile strip 0.3 m at 1.5 m), kerb 0.15 m, road. / 合成シーン。"""
    sc = object.__new__(S.Scene)
    G, GC = [], []

    def strip(w0, w1, cls, z):
        x0, x1 = -20.0, 20.0
        A, B, C, D = (x0, -w0, z), (x1, -w0, z), (x1, -w1, z), (x0, -w1, z)
        G.extend([[A, B, C], [A, C, D]])
        GC.extend([cls, cls])
    z = 2.0
    if lot_w:
        strip(0.0, lot_w, S.LOT, z + 0.12)
    strip(lot_w, lot_w + walk_w, S.WALK, z)
    strip(lot_w + 1.5, lot_w + 1.8, S.TACTILE, z + 0.001)
    strip(lot_w + walk_w, lot_w + walk_w + 0.15, S.KERB, z)
    strip(lot_w + walk_w + 0.15, 12.0, S.ROAD, z - 0.15)
    O, OF = [], []
    # the facade wall itself (vertical, at w = 0) / 外壁
    O.extend([[(-20, 0.0, z), (20, 0.0, z), (20, 0.0, z + 8)]])
    OF.append(0)
    if tree is not None:
        x, w = tree
        for k in range(8):
            a0, a1 = 2 * math.pi * k / 8, 2 * math.pi * (k + 1) / 8
            P0 = (x + 0.15 * math.cos(a0), -w + 0.15 * math.sin(a0), z)
            P1 = (x + 0.15 * math.cos(a1), -w + 0.15 * math.sin(a1), z)
            O.append([P0, P1, (P1[0], P1[1], z + 4)])
            OF.append(1)
    sc.G = np.array(G, np.float64)
    sc.GC = np.array(GC, np.int8)
    sc.O = np.array(O, np.float64)
    sc.OF = np.array(OF, np.int8)
    sc.Gb = np.concatenate([sc.G[:, :, :2].min(1), sc.G[:, :, :2].max(1)], 1)
    sc.Ob = np.concatenate([sc.O[:, :, :2].min(1), sc.O[:, :, :2].max(1)], 1)
    return sc, z


def door(use="eatery"):
    return {"x": 0.0, "y": 0.0, "t": [1.0, 0.0], "n": [0.0, -1.0], "u": 10.0, "dw": 1.7, "L": 20.0,
            "zg": 0.1, "zb": 1.9, "use": use, "i": 1, "edge": 0}


def run(sc, z, plan, use="eatery", seed=0):
    d = door(use)
    win = S.Window(sc, d, z + (0.12 if (sc.GC == S.LOT).any() else 0.0), (-7.5, 7.5))
    win.add_road([])
    orig = S.plan_door
    S.plan_door = lambda door, rng: plan
    try:
        why = {}
        out = S.place_door(win, d, np.random.default_rng(seed), why, (-9.9, 9.9))
    finally:
        S.plan_door = orig
    return out, why


def test_props_off_door_tactile_and_kerb():
    sc, z = scene(walk_w=3.5)
    out, why = run(sc, z, [("jihanki", 2), ("kanban_a", 1), ("hachi", 2), ("nobori", 2)])
    kinds = [o[0] for o in out]
    assert "jihanki" in kinds and "kanban_a" in kinds and kinds.count("hachi") == 2
    hw = 1.7 / 2 + 0.35
    for kind, args, (s0, s1, w0, w1), zb, clear in out:
        assert not (w0 < S.DOOR_OUT and s1 > -hw and s0 < hw), kind          # door opening / 入口前
        assert w1 <= 1.5 - S.ROAD_CLEAR + 1e-6, kind                       # tactile at 1.5 m / 点字ブロック
        assert zb == pytest.approx(z, abs=0.01)
        assert clear >= S.WALK_CLEAR or math.isinf(clear)


def test_narrow_footway_keeps_width_minus_06():
    sc, z = scene(walk_w=1.6)
    sc.GC[sc.GC == S.TACTILE] = S.WALK                                     # no tactile here / 点字なし
    out, why = run(sc, z, [("jihanki", 1), ("hachi", 2), ("planter", 1)])
    for kind, args, (s0, s1, w0, w1), zb, clear in out:
        assert 1.6 - w1 >= 1.6 - S.WALK_NARROW - 1e-6, kind                 # depth <= 0.6 m / 奥行き 0.6 m 以下
    assert "jihanki" not in [o[0] for o in out]                             # 0.81 m deep: no room / 置けない
    assert why.get("footway", 0) > 0


def test_lot_props_need_no_footway_check():
    sc, z = scene(walk_w=1.2, lot_w=1.2)
    out, why = run(sc, z, [("jihanki", 1)], use="retail")
    assert out and out[0][0] == "jihanki"
    assert math.isinf(out[0][4])
    assert out[0][3] == pytest.approx(z + 0.12, abs=0.01)                  # on the lot terrace / 敷地の上


def test_pole_clearance():
    sc, z = scene(walk_w=3.5, tree=(2.6, 0.5))
    out, why = run(sc, z, [("hachi", 2), ("kanban_a", 1), ("jihanki", 1)])
    for kind, args, (s0, s1, w0, w1), zb, clear in out:
        dx = max(s0 - 2.6, 0, 2.6 - s1)
        dw = max(w0 - 0.5, 0, 0.5 - w1)
        assert math.hypot(dx, dw) >= S.POLE_CLEAR + 0.15 - 0.06, kind


def test_no_prop_on_road_or_kerb():
    sc, z = scene(walk_w=0.0)                                               # no footway at all / 歩道なし
    out, why = run(sc, z, [("hachi", 2), ("jihanki", 1), ("kanban_a", 1)])
    assert out == []


def test_textures_written(tmp_path):
    spec = S.write_prop_textures(str(tmp_path))
    spec.update(S.write_door_textures(str(tmp_path)))
    for name, v in spec.items():
        assert "sign" not in name.lower() and "light" not in name.lower()
        assert os.path.exists(os.path.join(tmp_path, v[0] + ".png"))
        assert os.path.exists(os.path.join(tmp_path, name + "_r.png"))


if __name__ == "__main__":
    import inspect
    import tempfile
    n = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        args = inspect.signature(fn).parameters
        runs = [{fn._params[0]: v} for v in fn._params[1]] if hasattr(fn, "_params") else [{}]
        for kw in runs:
            if "tmp_path" in args:
                with tempfile.TemporaryDirectory() as td:
                    fn(tmp_path=td, **kw)
            else:
                fn(**kw)
            n += 1
    print("all %d passed" % n)
