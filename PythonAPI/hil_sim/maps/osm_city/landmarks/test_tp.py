"""Tests for the Temasek Polytechnic primitives and blocks: every face points the
way it is seen (UE culls back faces), names CARLA keeps (maps/.venv).

  .venv/bin/python osm_city/landmarks/test_tp.py

淡馬錫理工学院の基本形状と棟の面の向き、名前を確認する。
"""

import math
import os
import sys

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.strtree import STRtree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from landmarks import tp as P  # noqa: E402
from landmarks.test_coral import check, tris  # noqa: E402
from mesh import Mesh  # noqa: E402


def outward(C, N, centre, tol=-1e-6):
    """Horizontal components point away from the centre. / 水平成分が中心から外向き。"""
    d = C[:, :2] - np.asarray(centre)[None, :]
    return bool(np.all(np.sum(N[:, :2] * d, axis=1) >= tol))


def fake(blocks):
    """A TP without a layout: nothing near, the blocks as solids.
    レイアウト無しの TP（道路は遠く、棟だけ）。"""
    s = P.TP.__new__(P.TP)
    s.X = lambda q: (float(q[0]), float(q[1]))
    s.no_go = box(5000, 5000, 5001, 5001)
    s.ng = s.no_go.buffer(1.0)
    s.blocks = blocks
    s.solids = [(b["poly"], b.get("base", 0.0), b["top"], k) for k, b in blocks.items()]
    s.tree = STRtree([q[0] for q in s.solids])
    return s


def main():
    good = True
    # wall_top: faces right of p0->p1, the top slopes / 上端の傾いた壁: p0->p1 の右向き
    m = Mesh()
    P.wall_top(m, "w", (0, 0), (10, 0), 0.0, 3.0, 6.0)
    C, N, _ = tris(m, "w")
    good &= check("wall_top faces right of p0->p1 (-y)", np.allclose(N, [0, -1, 0], atol=1e-6))
    good &= check("wall_top top runs 3 -> 6", abs(np.asarray(m.parts["w"][0][0])[:, 2].max() - 6.0) < 1e-9)
    # mansard on an X-shaped footprint: slopes up and out, flat top up / X 字形の寄棟: 上・外向き
    x = Polygon([(-4, -12), (4, -12), (4, -4), (12, -4), (12, 4), (4, 4), (4, 12), (-4, 12), (-4, 4), (-12, 4),
                 (-12, -4), (-4, -4)])
    m = Mesh()
    ok = P.mansard(m, x, 10.0, 12.0, 1.5, "roof", top="top")
    C, N, _ = tris(m, "roof")
    good &= check("mansard built on an X plan (%d slopes)" % len(C), ok and len(C) == 24)
    # each slope's normal points out of its own eave edge: up and away from the inset ring
    inner = Polygon(P.inset_ring(x, 1.5)[1])
    away = [inner.exterior.distance(Point(*(c[:2] + n[:2] * 0.3))) > inner.exterior.distance(Point(*c[:2]))
            or not inner.contains(Point(*(c[:2] + n[:2] * 0.3))) for c, n in zip(C, N)]
    good &= check("mansard slopes face up and out", np.all(N[:, 2] > 0.3) and all(away))
    C, N, _ = tris(m, "top")
    good &= check("mansard top faces up", len(C) > 0 and np.all(N[:, 2] > 0.99))
    good &= check("mansard refuses a footprint too narrow", not P.mansard(Mesh(), box(0, 0, 20, 2), 0, 1, 1.5, "r"))
    # dome and drum: out; annulus up / down; frustum out (in) / ドーム・円筒は外向き
    m = Mesh()
    P.dome(m, "d", (0, 0), 5.0, 10.0, 2.5)
    P.drum(m, "c", (0, 0), 5.0, 0.0, 10.0)
    C, N, _ = tris(m, "d")
    good &= check("dome faces up and out", outward(C, N, (0, 0)) and np.all(N[:, 2] > -1e-6))
    C, N, _ = tris(m, "c")
    good &= check("drum faces out", outward(C, N, (0, 0), 1e-3) and np.allclose(N[:, 2], 0, atol=1e-9))
    m = Mesh()
    P.annulus(m, "u", (0, 0), 3.0, 4.0, 1.0, up=True)
    P.annulus(m, "dn", (0, 0), 3.0, 4.0, 0.5, up=False)
    good &= check("annulus up / down", np.all(tris(m, "u")[1][:, 2] > 0.99) and np.all(tris(m, "dn")[1][:, 2] < -0.99))
    m = Mesh()
    P.frustum(m, "o", (0, 0), 6.0, 1.0, 0.0, 2.0)
    P.frustum(m, "i", (0, 0), 0.5, 6.0, 0.0, 1.0)
    C, N, _ = tris(m, "o")
    good &= check("frustum (roof) faces up and out", outward(C, N, (0, 0)) and np.all(N[:, 2] > 0))
    C, N, _ = tris(m, "i")
    good &= check("saucer underside faces down and out", np.all(N[:, 2] < 0) and outward(C, N, (0, 0)))
    # gable_at with a ridge angle: slopes up and out, gable ends out, soffit down
    # 切妻（棟の向き指定）: 斜面は上外、妻は外、軒裏は下
    for ang in (None, math.radians(30.0)):
        m = Mesh()
        p = Polygon([(0, 0), (30, 0), (30, 12), (0, 12)])
        P.gable_at(m, p, 20.0, 23.0, "roof", "gw", ang=ang, fascia="fa", soffit="so")
        C, N, _ = tris(m, "roof")
        good &= check("gable (%s) slopes up and out" % ang, np.all(N[:, 2] > 0.1) and outward(C, N, (15, 6)))
        C, N, _ = tris(m, "gw")
        good &= check("gable (%s) ends face out" % ang, outward(C, N, (15, 6), 1e-3))
        good &= check("gable (%s) soffit faces down" % ang, np.all(tris(m, "so")[1][:, 2] < -0.99))
        C, N, _ = tris(m, "fa")
        good &= check("gable (%s) fascia / verges face out" % ang, outward(C, N, (15, 6), 1e-3))
    # circle fit and arc run (drum ends of the arms) / 円の当てはめ
    pts = [(10 + 4 * math.cos(a), 5 + 4 * math.sin(a)) for a in np.linspace(0, math.pi, 9)]
    c, r = P.circle_fit(pts)
    good &= check("circle fit", abs(c[0] - 10) < 1e-6 and abs(c[1] - 5) < 1e-6 and abs(r - 4) < 1e-6)
    arm = Polygon([(-30, -4), (10, -4)] + pts[::-1][:0] + [(10 + 4 * math.cos(a), 0 + 4 * math.sin(a))
                                                            for a in np.linspace(-math.pi / 2, math.pi / 2, 10)][1:-1]
                  + [(10, 4), (-30, 4)])
    fit = P.arc_run(arm)
    good &= check("arc run finds the drum end", fit is not None and abs(fit[1] - 4) < 0.2)
    # swept roof and fascias: the roof faces up, fascias out, soffits down
    # 反り屋根: 屋根は上、鼻隠しは外、軒裏は下
    t = fake({})
    O = [np.array([50 * math.cos(a), 50 * math.sin(a)]) for a in np.linspace(math.pi, 2 * math.pi, 9)]
    I = [np.array([33 * math.cos(a), 33 * math.sin(a)]) for a in np.linspace(math.pi, 2 * math.pi, 9)]
    m = Mesh()
    t.swept_roof(m, O, I, 30.0, 38.5)
    C, N, _ = tris(m, P.ROOF)
    good &= check("swept roof faces up (%d)" % len(C), np.all(N[:, 2] > 0.2))
    C, N, _ = tris(m, "tp_soffit")
    vert = np.abs(N[:, 2]) < 1e-6
    good &= check("roof soffits face down", np.all(N[~vert][:, 2] < -0.99))
    r_ = np.linalg.norm(C[:, :2], axis=1)
    outer = r_ > 41.5
    good &= check("soffit bands face out of the ring / into the court",
                  np.all(np.sum(N[vert & outer][:, :2] * C[vert & outer][:, :2], 1) > 0)
                  and np.all(np.sum(N[vert & ~outer][:, :2] * C[vert & ~outer][:, :2], 1) < 0))
    # a spine block: every wall face on the footprint edges points out
    # 背骨の棟: 外周上の壁は全て外向き
    p = Polygon([(0, 0), (30, 0), (30, 14), (0, 14)])
    b = dict(kind="spine", blk="10", poly=p, top=28.0, ridge=33.0, storeys=6)
    t = fake({"s": b})
    t.wing_angle = lambda q: 0.0
    m = Mesh()
    t.block(m, "s", b)
    C, N, M = tris(m)
    on = np.array([p.exterior.distance(Point(*c[:2])) < 0.02 for c in C]) & (np.abs(N[:, 2]) < 1e-6)
    good &= check("spine walls on the footprint face out (%d)" % on.sum(), outward(C[on], N[on], (15, 7), 1e-6))
    good &= check("spine has ribbon glass and sunshades", "tp_ribbon" in M and P.GREY in M)
    # a slab: T windows (navy bars) / 板状棟: T 字の窓
    b2 = dict(kind="slab", blk="11", poly=Polygon([(0, 0), (14, 0), (14, 30), (0, 30)]), top=37.0, storeys=8)
    t = fake({"x": b2})
    m = Mesh()
    t.block(m, "x", b2)
    C, N, M = tris(m)
    on = np.array([b2["poly"].exterior.distance(Point(*c[:2])) < 0.02 for c in C]) & (np.abs(N[:, 2]) < 1e-6)
    good &= check("slab walls on the footprint face out (%d)" % on.sum(), outward(C[on], N[on], (7, 15), 1e-6))
    good &= check("slab has T windows, navy bars, louvres, red panels",
                  all(k in M for k in ("tp_win", P.NAVY, "tp_louvre", "tp_red")))
    # covered neighbour: no facade where another block touches up to the top
    # 隣の棟が全高で接する所に外壁を描かない
    a = dict(kind="box", blk="", poly=box(0, 0, 10, 10), top=12.0, storeys=3)
    c2 = dict(kind="box", blk="", poly=box(10, 0, 20, 10), top=12.0, storeys=3)
    t = fake({"a": a, "c": c2})
    m = Mesh()
    t.block(m, "a", a)
    C, N, _ = tris(m)
    shared = (np.abs(C[:, 0] - 10.0) < 0.05) & (np.abs(N[:, 0] - 1.0) < 1e-6) & (C[:, 2] < 12.0)
    good &= check("no wall on the shared edge", not shared.any())
    # the horseshoe: outer walls face out, courtyard walls face in, the gateway open
    # 馬蹄形: 外周の壁は外向き、中庭側は内向き、門は開口
    t = fake({})
    t.hs = dict(deck=8.0, storey=4.4, inner_eave=30.0, outer_eave=38.5, plant_top=44.0,
                gateway=dict(z0=8.0, z1=21.2))
    t.hO = [np.array([50 * math.cos(a), 50 * math.sin(a)]) for a in np.linspace(math.pi, 2 * math.pi, 25)]
    t.hI = [np.array([33 * math.cos(a), 33 * math.sin(a)]) for a in np.linspace(math.pi, 2 * math.pi, 25)]
    t.hcen = np.zeros(2)
    t.gate_segs = {11, 12}
    m = Mesh()
    t.horseshoe(m)
    C, N, M = tris(m)
    r_ = np.linalg.norm(C[:, :2], axis=1)
    vert = np.abs(N[:, 2]) < 1e-6
    rad = np.sum(N[:, :2] * C[:, :2], 1) / np.maximum(r_, 1e-9)
    on_out = vert & (np.abs(r_ - 50.0 * math.cos(math.pi / 48)) < 0.6) & (C[:, 1] < -5)
    on_in = vert & (np.abs(r_ - 33.0 * math.cos(math.pi / 48)) < 0.4) & (C[:, 1] < -5)
    # faces in the wall's plane (|radial| > 0.5; reveals and bar ends are sideways) / 壁面と平行な面
    on_out &= np.abs(rad) > 0.5
    on_in &= (np.abs(rad) > 0.5) & ~np.isin(np.array(M), ["glass_rail", "railing"])   # two-sided by design
    good &= check("horseshoe outer walls face out (%d)" % on_out.sum(), on_out.sum() > 100 and np.all(rad[on_out] > 0.9))
    good &= check("horseshoe courtyard walls face in (%d)" % on_in.sum(), on_in.sum() > 100 and np.all(rad[on_in] < -0.9))
    gate_mid = t.hO[12]
    ang = math.atan2(gate_mid[1], gate_mid[0])
    near = (np.abs(np.arctan2(C[:, 1], C[:, 0]) - ang) < 0.05) & (C[:, 2] > 10) & (C[:, 2] < 19) & on_out
    good &= check("gateway open between deck and 21.2 m", not near.any())
    good &= check("horseshoe has T windows, gallery, arcade, swept roof",
                  all(k in M for k in ("tp_win", "tp_gallery", "tp_yellow", P.ROOF, "glass_rail")))
    # the library: the crown flares out (faces out and down), base walls out
    # 図書館: 頂部は外・下向き、下段の壁は外向き
    t = fake({})
    t.T = {"library": dict(c=[0.0, 0.0], ang=17.0, R=18.4, a=6.15, fill=0.72, z=[0.0, 18.0, 40.0, 44.5], top=49.0)}
    t.fit_log = []
    t.make_library()
    t.tree = STRtree([q[0] for q in t.solids])
    sq = t.lib["base"]
    m = Mesh()
    t.library(m)
    C, N, M = tris(m, P.WHITE)
    crown = (C[:, 2] > 44.6) & (C[:, 2] < 48.9) & (np.abs(N[:, 2]) > 0.05) & (np.abs(N[:, 2]) < 0.99)
    good &= check("library crown flares out (%d)" % crown.sum(),
                  crown.sum() >= 12 and outward(C[crown], N[crown], (0, 0)) and np.all(N[crown][:, 2] < 0))
    C, N, M = tris(m)
    M = np.array(M)
    good &= check("library has grid curtain walls, terraces, glass band",
                  all(k in M for k in ("tp_curtaingrid", "tp_curtain", "tp_planterbox", "glass_rail")))
    sel = M != "glass_rail"
    C, N = C[sel], N[sel]
    walls = (np.abs(N[:, 2]) < 1e-6) & (C[:, 2] < 17.0) & np.array(
        [sq.exterior.distance(Point(*c[:2])) < 0.02 for c in C])
    good &= check("library base walls face out (%d)" % walls.sum(), outward(C[walls], N[walls], (0, 0), 1e-6))
    # drop_over: the kept triangles keep their own vertices after Mesh.merge
    # 道路に触れる三角形を除いた後も、残りの三角形の頂点がずれない
    m = Mesh()
    m.quad("q", (0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0))
    m.quad("q", (10, 0, 0), (11, 0, 0), (11, 1, 0), (10, 1, 0))
    n = P.drop_over(m, box(-0.5, -0.5, 1.5, 1.5))
    m2 = Mesh()
    m2.merge(m)
    V = np.concatenate(m2.parts["q"][0])[np.concatenate(m2.parts["q"][3])]
    good &= check("drop_over drops 2, the kept ones stay at x 10-11", n == 2 and V[:, :, 0].min() >= 10.0)
    # inner rectangle of a road-cut footprint stays inside it / 道路で切った外形の内側の長方形
    cut = Polygon([(0, 0), (30, 0), (30, 9), (24, 14), (0, 14)])
    r = P.inner_rect(cut)
    rp = Polygon([tuple(q) for q in r])
    good &= check("inner_rect inside the cut footprint (%.0f of %.0f m2)" % (rp.area, cut.area),
                  rp.difference(cut.buffer(0.06)).area < 0.01 and rp.area > 0.7 * cut.area)
    # curved parapet: outer face out, inner face in, the cap up / 曲線のパラペット
    m = Mesh()
    f = P.B.Facade(m, np.array([0.0, 0.0]), np.array([12.0, 0.0]), np.array([0.0, -1.0]))
    P.curved_parapet(m, f, 12.0, 30.0, 3.4)
    C, N, _ = tris(m, P.WHITE)
    vert = np.abs(N[:, 2]) < 1e-6
    outer = vert & (np.abs(C[:, 1]) < 0.01)
    inner = vert & (np.abs(C[:, 1] - 0.4) < 0.01)
    good &= check("curved parapet: outer out, inner in, cap up",
                  np.all(N[outer][:, 1] < -0.99) and np.all(N[inner][:, 1] > 0.99) and np.all(N[~vert][:, 2] > 0))
    # Blk 1A: tower glass faces out of its tower, the curved frame caps up
    # 1A: 塔のガラスは外向き、曲線の枠の天端は上向き
    ang = math.radians(163.0)
    ax_, sd_ = np.array([math.cos(ang), math.sin(ang)]), np.array([-math.sin(ang), math.cos(ang)])
    tw = Polygon([tuple(ax_ * u + sd_ * v) for u, v in ((0, 0), (28, 0), (28, 26), (0, 26))])
    pod = tw.buffer(15.0, join_style="mitre")
    b3 = dict(kind="eastwing", blk="1A", poly=pod, top=12.1, tower_polys=[tw], ttop=33.8, tframe=37.8)
    t = fake({"1A": b3})
    m = Mesh()
    t.block(m, "1A", b3)
    C, N, M = tris(m)
    M = np.array(M)
    caps = (M == P.WHITE) & (np.abs(N[:, 2]) > 0.5) & (C[:, 2] > 33.9)
    good &= check("1A frame caps face up (%d)" % caps.sum(), caps.sum() > 8 and np.all(N[caps][:, 2] > 0))
    gl = (M == "tp_glass1a") & (C[:, 2] > 12.2)
    subs = [Polygon([tuple(ax_ * u + sd_ * v) for u, v in ((u0, 0), (u1, 0), (u1, 26), (u0, 26))])
            for u0, u1 in ((0, 28 * 0.4), (28 * 0.6, 28))]
    ahead = [not any(q.contains(Point(*(c[:2] + n[:2] * 0.3))) for q in subs) for c, n in zip(C[gl], N[gl])]
    behind = [any(q.contains(Point(*(c[:2] - n[:2] * 0.3))) for q in subs) for c, n in zip(C[gl], N[gl])]
    good &= check("1A tower glass faces out (%d)" % gl.sum(), gl.sum() > 4 and all(ahead) and all(behind))
    # band along a slope: faces right of p0->p1, constant height / 傾いた帯
    m = Mesh()
    P.band(m, "b", (0, 0), (10, 0), 30.0, 38.0, 1.2)
    V = np.concatenate(m.parts["b"][0])
    good &= check("band faces right, 1.2 m high all along",
                  np.allclose(tris(m, "b")[1], [0, -1, 0], atol=1e-6) and abs(V[:, 2].min() - 28.8) < 1e-9)
    # a crop build without the campus: nothing built, nothing cleared, no crash
    # 敷地を含まない切り出し: 何も作らず、落ちない
    import json

    class _O(object):
        def areas(self, f):
            return iter([])

    class _L(object):
        pass
    bd = os.path.join(os.path.dirname(HERE), "..", "build", "PasirRisTampines")
    if os.path.exists(os.path.join(bd, P.TRACE)):
        lo = _L()
        lo.meta, lo.osm = json.load(open(os.path.join(bd, "meta.json"))), _O()
        e = P.TP(lo, bd)
        good &= check("no site in the build: empty, no crash",
                      e.empty and e.inner().is_empty and e.buildings_mesh(box(0, 0, 9, 9)).ntris() == 0
                      and e.tree_filter([(1, 2)]) == [(1, 2)])
    # names CARLA keeps (no "light" / "sign") / CARLA が捨てない名前
    import tempfile
    d = tempfile.mkdtemp()
    names = list(P.write_textures(d))
    good &= check("texture names without light/sign (%d)" % len(names),
                  all("light" not in n and "sign" not in n for n in names))
    print("ALL PASS" if good else "SOME FAILED")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
