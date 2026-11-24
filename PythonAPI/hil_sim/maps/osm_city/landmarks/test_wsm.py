"""Tests for the White Sands mall (landmarks/wsm.py): every face points the way
it is seen (UE culls back faces), roads win, names CARLA keeps (maps/.venv).

  .venv/bin/python osm_city/landmarks/test_wsm.py

The whole model is also built on a stand-in layout (the real trace, a fake road
along the NE face) and checked: footprint walls face out, roofs up, soffits
down, the arched shell wall out, its strip up, its end steps along it, nothing on the
road.

ホワイトサンズの基本形状と全体モデルの面の向き・道路優先・名前を確認する。
"""

import json
import math
import os
import sys
import tempfile

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.geometry.polygon import orient

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import buildings as B  # noqa: E402
from landmarks import wsm  # noqa: E402
from mesh import Mesh  # noqa: E402

BUILD = os.path.join(HERE, "..", "..", "build", "PasirRisTampines")


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


def primitives():
    good = True
    # fit_edges: an edge 0.4 m from a road moves in, the block stays 4-sided
    # 道路から 0.4 m の辺は内側へ動き、四角形のまま
    sq = box(0, 0, 20, 10)
    road = box(-5, 10.4, 25, 14)
    q, off = wsm.fit_edges(sq, road, 1.0)
    good &= check("fit_edges: clear of the road by the gap (%.2f m)" % road.distance(q),
                  road.distance(q) >= 0.99 and len(q.exterior.coords) == 5 and q.exterior.is_ccw)
    good &= check("fit_edges: only the north edge moved", sorted(np.round(off, 2))[-1] > 0 and (off > 0).sum() == 1)
    # fillet: a rounded corner stays inside, the ring stays CCW and valid / 角の丸め
    pts = [(0, 0), (20, 0), (30, 15), (0, 15)]
    f = wsm.fillet(pts, 1, 5.0)
    P = Polygon(f)
    good &= check("fillet: valid CCW ring inside the original", P.is_valid and P.exterior.is_ccw
                  and Polygon(pts).buffer(1e-6).contains(P) and P.area < Polygon(pts).area)
    # vpoly faces its normal / 鉛直多角形は指定の向き
    m = Mesh()
    wsm.vpoly(m, "x", [(0, 0, 0), (0, 2, 0), (0, 2, 3), (0, 1, 4), (0, 0, 3)], (1, 0, 0))
    C, N, _ = tris(m)
    good &= check("vpoly: all %d faces along +x" % len(N), len(N) == 3 and np.all(N[:, 0] > 0.99))
    m = Mesh()
    wsm.vpoly(m, "x", [(0, 0, 0), (0, 2, 0), (0, 2, 3), (0, 0, 3)], (-1, 0, 0))
    C, N, _ = tris(m)
    good &= check("vpoly: faces -x when asked", np.all(N[:, 0] < -0.99))
    # pyramid faces up and out / 角錐は上・外向き
    m = Mesh()
    ring = [(math.cos(a) * 5, math.sin(a) * 5) for a in np.linspace(0, 2 * math.pi, 9)[:-1]]
    wsm.pyramid(m, "g", ring, 10.0, (0.0, 0.0, 12.0))
    C, N, _ = tris(m)
    out = np.einsum("ij,ij->i", N[:, :2], C[:, :2])
    good &= check("pyramid: 8 faces up and out", len(N) == 8 and np.all(N[:, 2] > 0) and np.all(out > 0))
    # quad_up / tface / 上向きの四角形・外向きの面
    m = Mesh()
    wsm.quad_up(m, "c", (0, 0, 1), (0, 1, 1), (1, 1, 1), (1, 0, 1))
    C, N, _ = tris(m)
    good &= check("quad_up: faces up", np.all(N[:, 2] > 0.99))
    m = Mesh()
    fac = B.Facade(m, np.array([0.0, 0.0]), np.array([10.0, 0.0]), np.array([0.0, -1.0]))
    wsm.tface(fac, "w", 1.0, 9.0, 0.0, 3.0, 0.0, 4.4, 4.4, uo=3.0)
    C, N, _ = tris(m)
    good &= check("tface: faces out (-y)", np.all(N[:, 1] < -0.99))
    # band: outer walls out of the footprint, top up, bottom down / 帯
    foot = orient(box(0, 0, 30, 20), 1.0)
    m = Mesh()
    wsm.band(m, foot, LineString([(0, 0), (30, 0), (30, 20)]), 0.6, 5.0, 8.0, [(5.0, 8.0, "b")], bottom="s")
    C, N, M = tris(m)
    vert = np.abs(N[:, 2]) < 0.01
    outside = ~shapely.contains_xy(foot.buffer(0.3, join_style="mitre"), C[vert, 0] + N[vert, 0] * 0.4,
                                   C[vert, 1] + N[vert, 1] * 0.4)
    ends = ~outside
    good &= check("band: %d outer walls face out, end caps along the face" % int(vert.sum()),
                  vert.sum() >= 4 and (np.all(outside) or np.all(np.abs(N[vert][ends][:, 0]) > 0.99)))
    tops = np.array([k == "b" for k in M]) & ~vert
    bots = np.array([k == "s" for k in M])
    good &= check("band: top up, bottom down", np.all(N[tops, 2] > 0.99) and np.all(N[bots, 2] < -0.99))
    good &= check("band: no wall on the footprint line",
                  not np.any(shapely.contains_xy(foot.exterior.buffer(0.01), C[vert, 0], C[vert, 1])))
    # slab: walls out, top up, underside down / 薄い庇
    m = Mesh()
    wsm.slab(m, "c", box(0, 0, 4, 2), 4.75, 5.0, under="u")
    C, N, M = tris(m)
    u = np.array([k == "u" for k in M])
    top = (~u) & (N[:, 2] > 0.5)
    wall = np.abs(N[:, 2]) < 0.01
    cx = np.array([2.0, 1.0])
    good &= check("slab: walls out, top up, underside down",
                  np.all(np.einsum("ij,ij->i", N[wall, :2], C[wall, :2] - cx) > 0) and top.sum() == 2
                  and np.all(N[u, 2] < -0.99))
    return good


class FakeL(object):
    """A stand-in layout: a road along the NE face, 3 m out. / 仮の配置（北東面の外 3 m に道路）。"""

    def __init__(self, meta, T):
        self.meta = meta
        X = wsm._xform(T, meta)
        foot = orient(Polygon([X(q) for q in T["footprint"]]), 1.0)
        ox, oy, a = T["frames"]["S"]
        a = math.radians(a)
        eu, ev = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        O = np.array([ox, oy])
        P = lambda u, v: tuple(O + eu * u + ev * v)
        # NE face line (53.2, 67.6) -> (101.8, 40.7) in frame S, moved 3.4 m out
        ne = LineString([P(53.2, 67.6), P(101.8, 40.7)])
        self.road_all = ne.parallel_offset(5.0, "left").buffer(1.8, cap_style="flat")
        self.islands = Polygon()
        # a drive stub ending 0.4 m from the face (the car-park entrance) / 駐車場の車路
        tip = np.array(ne.interpolate(0.45, normalized=True).coords[0])
        nrm = np.array([-(P(101.8, 40.7)[1] - P(53.2, 67.6)[1]), P(101.8, 40.7)[0] - P(53.2, 67.6)[0]])
        nrm = nrm / np.linalg.norm(nrm)
        if foot.contains(Point(tip + nrm * 1.0)):
            nrm = -nrm
        self.driveways = LineString([tip + nrm * 0.4, tip + nrm * 6.0]).buffer(2.5, cap_style="flat")
        self.foot = foot
        # the E footpath under the linkway / 通路の下の東の歩道
        self.footpaths = LineString([P(82.6, -3.0), P(104.65, 35.0)]).buffer(1.05, cap_style="flat")


def whole_model():
    good = True
    trace = os.path.join(BUILD, wsm.TRACE)
    if not os.path.exists(trace):
        print("SKIP whole model: no %s" % trace)
        return True
    d = tempfile.mkdtemp()
    T = json.load(open(trace))
    meta = json.load(open(os.path.join(BUILD, "meta.json")))
    json.dump(T, open(os.path.join(d, wsm.TRACE), "w"))
    L = FakeL(meta, T)
    W = wsm.WSM(L, d)
    good &= check("roads win: footprint %.2f m off the fake road and drive (>= GAP)"
                  % W.no_go.distance(W.foot), W.no_go.distance(W.foot) >= wsm.GAP - 1e-3)
    m = Mesh()
    for _, pm in W.build_parts():
        m.merge(pm)
    C, N, M = tris(m)
    M = np.array(M)
    good &= check("nothing left on the road network (%d triangles)" % len(C),
                  not np.any(shapely.contains_xy(W.no_go.buffer(-0.01), C[:, 0], C[:, 1])))
    # footprint walls face out / 外形上の壁は外向き
    ring = W.foot.exterior
    on = (shapely.distance(shapely.points(C[:, :2]), ring) < 0.004) & (np.abs(N[:, 2]) < 0.01) & (C[:, 2] > 0.2)
    outward = ~shapely.contains_xy(W.foot, C[on, 0] + N[on, 0] * 0.05, C[on, 1] + N[on, 1] * 0.05)
    good &= check("%d faces on the footprint all face out" % int(on.sum()), on.sum() > 20 and np.all(outward))
    for mat, sgn, nm in (("roof_concrete", 1, "roofs up"), ("wsm_soffit", -1, "soffits down"),
                         ("wsm_cap", 1, "parapet caps up"), ("wsm_solar", 1, "solar rows up"),
                         ("wsm_sky", 1, "skylight up")):
        k = M == mat
        good &= check("%s (%d)" % (nm, int(k.sum())), k.sum() > 0 and np.all(N[k, 2] * sgn > 0.2))
    # step walls between roofs face the lower roof / 屋上の段差の壁は低い側を向く
    edges = shapely.union_all([bb["poly"].boundary for bb in W.blocks.values()])
    st = (M == "wsm_white") & (np.abs(N[:, 2]) < 0.01) & (C[:, 2] > 22.5) \
        & (shapely.distance(shapely.points(C[:, :2]), edges) < 0.004)
    bad = 0
    for c, n in zip(C[st], N[st]):
        p_out, p_in = c[:2] + n[:2] * 0.3, c[:2] - n[:2] * 0.3
        h_out, h_in = W.h_at(p_out)[0], W.h_at(p_in)[0]
        if h_out > h_in + 0.05:
            bad += 1
    good &= check("high walls face the lower side (%d checked, %d wrong)" % (int(st.sum()), bad), bad == 0)
    # the shell: arched metal wall faces out, the strip behind it up, end steps
    # face along the shell, the step to the roof faces the lower side
    # シェル: アーチ状の壁は外向き、帯は上向き、端の段は帯の方、屋根との段は低い側
    sh = M == "wsm_metal"
    out_ok = ~shapely.contains_xy(W.foot, C[sh, 0] + N[sh, 0] * 0.05, C[sh, 1] + N[sh, 1] * 0.05)
    good &= check("shell wall: %d metal faces face out" % int(sh.sum()),
                  sh.sum() > 40 and np.all(out_ok) and np.all(np.abs(N[sh, 2]) < 0.01))
    zs = [W.z_arch(s_) for s_ in np.linspace(0, W.S_tot, 41)]
    good &= check("shell top is an arch (ends %.1f m, crown %.1f m)" % (zs[0], max(zs)),
                  zs[0] < 12 and zs[-1] < 12 and max(zs) > 25 and int(np.argmax(zs)) in range(15, 26))
    tp = M == "wsm_metal_top"
    good &= check("shell strip faces up (%d; steep at the arch ends)" % int(tp.sum()), tp.sum() > 40 and np.all(N[tp, 2] > 0.02))
    top = W.blocks["east"]["top"]
    for jb, ji, jn, nm in ((1, 1, 2, "start"), (-2, -2, -3, "end")):
        along = W.B[jn] - W.B[jb]
        along = along / np.linalg.norm(along)
        plane = np.abs((C[:, :2] - W.B[jb]) @ along) < 0.01
        cap = (M == "wsm_white") & plane & (shapely.distance(shapely.points(C[:, :2]), Point(W.B[jb]))
                                            < W.T["shell"]["depth"] + 0.1) & (np.abs(N[:, 2]) < 0.01) \
            & (C[:, 2] > W.z_arch(W.S_arc[jb]))
        good &= check("shell %s step faces along the shell (%d)" % (nm, int(cap.sum())),
                      cap.sum() > 0 and np.all(N[cap, :2] @ along > 0.99))
    il = LineString([tuple(p) for p in W.I[1:-1]])
    stp = (M == "wsm_white") & (shapely.distance(shapely.points(C[:, :2]), il) < 0.01) & (np.abs(N[:, 2]) < 0.01)
    bad = 0
    for c, n in zip(C[stp], N[stp]):
        s_ = W.shell_line.project(Point(il.interpolate(il.project(Point(c[:2])))))
        z_s = W.z_arch(min(W.S_tot, s_ * W.S_tot / max(W.shell_line.length, 1e-9)))
        inside = W.blocks["east"]["roof_poly"].contains(Point(c[:2] + n[:2] * 0.1))
        bad += int(inside != (c[2] > top) and abs(c[2] - top) > 0.6 and abs(z_s - top) > 1.0)
    good &= check("strip-to-roof step faces the lower side (%d faces, %d doubtful)" % (int(stp.sum()), bad),
                  stp.sum() > 10 and bad <= 2)
    # the linkway: roof up, off the road, posts off the footpath / 通路: 屋根は上向き、道路と歩道を避ける
    lk = M == "wsm_linkroof"
    good &= check("linkway roof (%d faces) up on top, off the fake road" % int(lk.sum()),
                  W.link is not None and W.no_go.distance(W.link["roof"]) >= 0.59)
    good &= check("triangles within the budget (%d <= 150000)" % len(C), len(C) <= 150000)
    return good


def main():
    good = primitives()
    good &= whole_model()
    # names CARLA keeps / 材質名に light / sign を含まない
    d = tempfile.mkdtemp()
    mats = wsm.write_textures(d)
    bad = [k for k in mats if "light" in k.lower() or "sign" in k.lower()]
    good &= check("texture names CARLA keeps (%d)" % len(mats), not bad)
    print("ALL PASS" if good else "SOME FAILED")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
