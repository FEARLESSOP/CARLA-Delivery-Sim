"""Road elevation from the terrain field into the road logic (maps/.venv: numpy, scipy).

  python elevate.py <build dir>     (after roadnet.py [fix_xodr_signals] / lht_poles and a
                                     first lanes.py; then run lanes.py again)

Every road of <Name>.xodr gets an <elevationProfile> (one Hermite cubic per
~5 m: value and slope at both ends) from the smoothed terrain
(terrain_field.npz, terrain.py). The heights are solved once for the whole
network, so everything that meets agrees exactly:

  - each street is sampled on its CENTRE LINE (netconvert keep-left: every
    direction is its own road with its reference line on its kerb; two
    directions sampled at their kerbs would differ by the cross-fall of
    the hillside), and the two directions of a street are tied together;
  - junctions are level (OpenDRIVE lanes are laterally level and CARLA has
    no superelevation, so crossing connecting roads only agree on a level
    junction): one height per junction, every connecting road at it, the
    roads leaving it start level (slope 0);
  - between junctions a least-squares fit to the terrain with a curvature
    penalty (wavelengths below ~SMOOTH_WAVE m are smoothed out: the
    terrain's 20 m TIN and terraced lots are not the road), roads that
    continue each other share their end point and slope.
  - exact where the model's road surface meets two lanes (2026-11-07, guide
    ch. 30): a junction's lanes held over the roads beside them, the two
    directions of a street on shared stations and the second one written as
    dense samples of the first one's profile, streets that touch tied; the
    grade penalty measures each interval at its tightest point (on a bend the
    inside lane is steeper than the street).
Nothing is smoothed separately per road after that: linked roads share
end heights and slopes by construction.

Prints the grades (flag > 12 %), and writes elevation_report.json.
Re-running replaces the profile. lanes.json (any previous run: only its
x, y are used) gives the lane edges for the centre lines.

道路情報の各道路に地形から縦断線形（標高）を書き込む。網全体で一度に解く:
道路は中心線で標本化、交差点は水平、接続点で高さと勾配が一致。
"""

import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SEG = 5.0              # metres per cubic / 3 次式 1 本の長さ
FLAG = 0.12            # grades above this are reported / 報告する勾配
SMOOTH_WAVE = 60.0     # wavelengths shorter than this are smoothed out / これより短い起伏は平滑化
W_TIE, W_FLAT, W_JUNC = 30.0, 10.0, 4.0
GRADE_SOFT, W_GRADE, GRADE_ITER = 0.15, 200, 12   # grades above this are pushed down / これを超える勾配を抑える
W_CORE = 100.0         # stations inside an embedded model held at its level / 埋め込みモデル内の測点の重み


def load_field(out_dir):
    from terrain import Terrain
    return Terrain.load(out_dir)


# -- the reference line / 参照線
def geom_sampler(g):
    """Function s_local -> (x, y, hdg) for one planView geometry. / 平面線形 1 要素。"""
    x0, y0, h0, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
    el = list(g)[0]
    c, s = math.cos(h0), math.sin(h0)
    if el.tag == "line":
        return lambda t: (x0 + c * t, y0 + s * t, h0)
    if el.tag == "arc":
        k = float(el.get("curvature"))
        if abs(k) < 1e-12:
            return lambda t: (x0 + c * t, y0 + s * t, h0)
        return lambda t: (x0 + (math.sin(h0 + k * t) - s) / k, y0 - (math.cos(h0 + k * t) - c) / k, h0 + k * t)
    n = 400                                  # paramPoly3, poly3, spiral: by arc length / 弧長で表に
    if el.tag == "paramPoly3":
        A = [float(el.get(k)) for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV")]
        p = np.linspace(0, 1.0 if el.get("pRange", "normalized") == "normalized" else L, n)
        u = A[0] + A[1] * p + A[2] * p ** 2 + A[3] * p ** 3
        v = A[4] + A[5] * p + A[6] * p ** 2 + A[7] * p ** 3
        du, dv = A[1] + 2 * A[2] * p + 3 * A[3] * p ** 2, A[5] + 2 * A[6] * p + 3 * A[7] * p ** 2
    elif el.tag == "poly3":
        a, b, cc, d = (float(el.get(k)) for k in "abcd")
        u = np.linspace(0, L, n)
        v = a + b * u + cc * u ** 2 + d * u ** 3
        du, dv = np.ones(n), b + 2 * cc * u + 3 * d * u ** 2
    else:
        k0, k1 = float(el.get("curvStart")), float(el.get("curvEnd"))
        t = np.linspace(0, L, n)
        hh = k0 * t + (k1 - k0) * t ** 2 / (2 * L)
        du, dv = np.cos(hh), np.sin(hh)
        u = np.concatenate([[0], np.cumsum((du[1:] + du[:-1]) / 2 * np.diff(t))])
        v = np.concatenate([[0], np.cumsum((dv[1:] + dv[:-1]) / 2 * np.diff(t))])
    S = np.concatenate([[0], np.cumsum(np.hypot(np.diff(u), np.diff(v)))])
    S *= L / S[-1] if S[-1] > 0 else 1.0
    X, Y, Hd = x0 + c * u - s * v, y0 + s * u + c * v, np.unwrap(h0 + np.arctan2(dv, du))
    return lambda t: (float(np.interp(t, S, X)), float(np.interp(t, S, Y)), float(np.interp(t, S, Hd)))


def road_sampler(road):
    gs = road.findall("planView/geometry")
    starts = [float(g.get("s")) for g in gs]
    fns = [geom_sampler(g) for g in gs]

    def at(sv):
        k = max(0, int(np.searchsorted(starts, sv, side="right")) - 1)
        return fns[k](sv - starts[k])
    return at


def hermite(S, z, m):
    """[(s, a, b, c, d)] through values z and slopes m at stations S. / エルミート 3 次式。"""
    out = []
    for i in range(len(S) - 1):
        ds = S[i + 1] - S[i]
        h0, h1, m0, m1 = z[i], z[i + 1], m[i], m[i + 1]
        if ds < 1e-6:
            continue
        out.append((S[i], h0, m0, (3 * (h1 - h0) / ds - 2 * m0 - m1) / ds, (2 * (h0 - h1) / ds + m0 + m1) / ds ** 2))
    return out or [(0.0, z[0], 0.0, 0.0, 0.0)]


DENSE_S = 1.0          # the other direction's profile: one cubic per metre / 対向車線の縦断は 1 m ごと


def prof_eval(prof, s):
    """z of an elevation profile [(s, a, b, c, d)] at stations s. / 縦断の高さ"""
    P = np.array(prof, np.float64)
    k = np.clip(np.searchsorted(P[:, 0], s, side="right") - 1, 0, len(P) - 1)
    q = P[k]
    ds = s - q[:, 0]
    return q[:, 1] + q[:, 2] * ds + q[:, 3] * ds ** 2 + q[:, 4] * ds ** 3


def _centre(cpos, rid, S):
    Ks, P = cpos[rid]
    return np.interp(S, Ks, P[:, 0]), np.interp(S, Ks, P[:, 1])


def _project(cpos, rid, X, Y):
    """(s on rid's centre line, distance) of points. / 中心線への射影"""
    import shapely
    Ks, P = cpos[rid]
    if len(P) < 2:
        return np.full(len(X), -1.0), np.full(len(X), 1e9)
    line = shapely.LineString(P)
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    mp = shapely.points(np.column_stack([X, Y]))
    return np.interp(shapely.line_locate_point(line, mp), arc, Ks), shapely.distance(line, mp)


def grade_of(prof, L):
    g = 0.0
    for k, (s0, a, b, c, d) in enumerate(prof):
        s1 = prof[k + 1][0] if k + 1 < len(prof) else L
        t = np.linspace(0, max(s1 - s0, 0.0), 6)
        g = max(g, float(np.abs(b + 2 * c * t + 3 * d * t * t).max()))
    return g


def centre_lines(lanes_json):
    """Per road: s, kerb point, offside point, heading (from lanes.json). / 道路ごとの縁石・中央側の線。"""
    data = json.load(open(lanes_json))
    per = {}
    for l in data["lanes"]:
        per.setdefault(l["road"], {}).setdefault(l["section"], []).append(l)
    out = {}
    for rid, secs in per.items():
        S, K, O, Y = [], [], [], []
        for si in sorted(secs):
            ls = secs[si]
            kerb = max(ls, key=lambda l: l["lane"])           # lane -1: its left edge is the kerb / 縁石側
            off = min(ls, key=lambda l: l["lane"])            # last lane: its right edge faces the other way
            ko = {round(q["s"], 3): q for q in off["pts"]}
            for q in kerb["pts"]:
                o = ko.get(round(q["s"], 3))
                if o is None:
                    continue
                S.append(q["s"])
                K.append(q["l"])
                O.append(o["r"])
                Y.append(math.radians(q["yaw"]))
        if len(S) >= 2:
            i = np.argsort(S)
            out[rid] = (np.array(S)[i], np.array(K)[i], np.array(O)[i], np.array(Y)[i])
    return out


class UF(object):
    def __init__(self):
        self.p = {}

    def find(self, a):
        self.p.setdefault(a, a)
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        self.p[self.find(a)] = self.find(b)


REACH = 0.5            # = terrain.LANE_EXT: the model's road surface averages lanes this far past their edge / 舗装の高さが及ぶ幅
W_CONTACT = 300.0      # two streets touching away from a junction / 交差点外で接する道路の結び
REACH_OTHER = 0.9      # REACH + the wheel-track inset (0.35 m) + margin: another junction's lanes / 他の交差点の判定幅
MERGE_ST = 1.5         # regular stations this near an inserted one are dropped / 挿入した測点の近くの測点は除く
PAIR_D = 1.0           # a station of one direction pairs with the other's projection within this / 上下線の測点を対にする距離
END_GAP = 0.3          # no inserted station this near a road end / 端点の近くには測点を挿入しない


def mates_of(partner, rid):
    """The roads paired with rid as the other direction (either way). / 対向車線の道路"""
    return {p for o, p in partner.items() if o == rid} | {o for o, p in partner.items() if p == rid}


def _lane_poly(l):
    import shapely
    ring = [q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]
    if len(ring) < 4:
        return None
    return shapely.make_valid(shapely.polygons(np.array(ring, np.float64)))


def solve(root, H, cl, wave=SMOOTH_WAVE, lanes=None, core=None):
    """The network's heights (see the module docstring). Exact where the model's
    road surface meets two lanes (2026-11-07; soft ties left 0.1-0.4 m between them,
    check_drive.py, guide ch. 30):
      - a junction's lanes reach REACH m (the drape's lane extension) over a road
        leaving it: that stretch of the road is held at the junction's height, with
        a station exactly at its end (likewise another junction's lanes over a road);
      - the two directions of a street (netconvert keep-left: two roads) share
        stations: one direction's stations are the projections of the other's, and
        each such pair is ONE unknown, so both get the same profile.
    道路網の標高。交差点の車線が掛かる区間は交差点の高さ、上下線は測点を共有（同じ未知数）。"""
    import scipy.sparse as sp
    import shapely
    from scipy.sparse.linalg import spsolve
    from scipy.spatial import cKDTree
    roads = {r.get("id"): r for r in root.findall("road")}
    free = {rid: r for rid, r in roads.items() if r.get("junction", "-1") == "-1"}
    st = {}
    two_way = {}
    offs = []
    for rid, r in free.items():
        c = cl.get(int(rid))
        if c is not None:
            offs.append((rid, c))
    # two-way streets: another road's offside edge runs right beside, the other way
    # 対向車線: 別の道路の中央側の縁がすぐ横を逆向きに走る
    allp = np.vstack([c[2] for _, c in offs])
    ally = np.concatenate([c[3] for _, c in offs])
    owner = np.concatenate([[i] * len(c[0]) for i, (_, c) in enumerate(offs)])
    tree = cKDTree(allp)
    partner = {}
    for i, (rid, c) in enumerate(offs):
        d, j = tree.query(c[2], k=6, distance_upper_bound=1.2)
        hits = {}
        for row_d, row_j, y in zip(d, j, c[3]):
            for dd, jj in zip(row_d, row_j):
                if jj >= len(owner) or owner[jj] == i:
                    continue
                if math.cos(ally[jj] - y) < -0.8:
                    hits[owner[jj]] = hits.get(owner[jj], 0) + 1
                    break
        if hits:
            best = max(hits, key=hits.get)
            if hits[best] >= 0.3 * len(c[0]) or hits[best] >= 8:
                partner[rid] = offs[best][0]
        two_way[rid] = bool(hits) and max(hits.values()) >= min(8, 0.3 * len(c[0]))
    # the street's centre line per road (s -> x, y) / 道路ごとの中心線
    cpos = {}
    for rid, r in free.items():
        L = float(r.get("length"))
        c = cl.get(int(rid))
        if c is not None:
            Ks, Kk, Ko, _ = c
            t = 1.0 if two_way[rid] else 0.5            # the street's centre / 道路の中心
            cpos[rid] = (np.asarray(Ks, np.float64), Kk + (Ko - Kk) * t)
        else:
            at = road_sampler(r)
            Ss = np.linspace(0.0, L, max(2, int(L) + 1))
            cpos[rid] = (Ss, np.array([at(v)[:2] for v in Ss]))

    def pos(rid, S):
        Ks, P = cpos[rid]
        return np.interp(S, Ks, P[:, 0]), np.interp(S, Ks, P[:, 1])
    # junction centres: mean of the field over their connecting roads / 交差点の高さの目安
    jpts = {}
    for rid, r in roads.items():
        j = r.get("junction", "-1")
        if j == "-1":
            continue
        at = road_sampler(r)
        L = float(r.get("length"))
        for v in np.linspace(0, L, max(2, int(L / 2) + 1)):
            jpts.setdefault(j, []).append(at(v)[:2])
    uf = UF()
    # junctions whose connecting lanes overlap each other (two junctions a few
    # metres apart) share one height / 接続車線が重なる交差点同士は同じ高さ
    nmerge = 0
    jl, rl = {}, {}
    if lanes is not None:
        jp_ = []
        for l in lanes:
            if l["junction"] >= 0:
                for q in l["pts"]:
                    for k_ in ("c", "l", "r"):
                        jp_.append((q[k_][0], q[k_][1], l["junction"]))
                p = _lane_poly(l)
                if p is not None:
                    jl.setdefault(str(l["junction"]), []).append(p)
            elif str(l["road"]) in free:
                for q in l["pts"]:
                    for k_ in ("c", "l", "r"):
                        rl.setdefault(str(l["road"]), []).append((q["s"], q[k_][0], q[k_][1]))
        J = np.array(jp_)
        if len(J):
            for a, b in cKDTree(J[:, :2]).query_pairs(0.6, output_type="ndarray"):
                ja, jb = str(int(J[a, 2])), str(int(J[b, 2]))
                if ja != jb and uf.find(("j", ja)) != uf.find(("j", jb)):
                    uf.union(("j", ja), ("j", jb))
                    nmerge += 1
    # the area each junction's lanes reach (merged junctions together) / 交差点の車線が及ぶ範囲
    groups = {}
    for j, ps in jl.items():
        groups.setdefault(uf.find(("j", j)), []).extend(ps)
    gunion = {g: shapely.union_all(ps) for g, ps in groups.items()}
    gpoly = {g: shapely.buffer(u, REACH) for g, u in gunion.items()}
    # another junction's lanes beside a road (not at its end, e.g. a dead-end loop next to
    # a street): the road's WHEEL TRACK must be clear of their reach, so its edge samples
    # are tested REACH_OTHER out (j137 / road 4073 stayed 0.08 m apart at 0.9 m)
    # 端以外で接する他の交差点: 車輪の轍まで届く範囲で判定
    gfar = {g: shapely.buffer(u, REACH_OTHER) for g, u in gunion.items()}
    gname = {g: next(j for j in jl if uf.find(("j", j)) == g) for g in groups}
    gkeys = list(gpoly)
    gtree = shapely.STRtree([gfar[g] for g in gkeys])
    # per road: stretches held at a junction's height, and the stations inserted at their ends
    # 道路ごとに交差点の高さに保つ区間と、その端に入れる測点
    holds, ins = {}, {}
    for rid, r in free.items():
        L = float(r.get("length"))
        holds[rid], ins[rid] = [], set()
        smp = rl.get(rid)
        if not smp:
            continue
        A = np.array(smp)
        lk = r.find("link")
        ends = {}
        for tag in ("predecessor", "successor"):
            e = lk.find(tag) if lk is not None else None
            if e is not None and e.get("elementType") == "junction":
                ends[uf.find(("j", e.get("elementId")))] = tag
        pts = shapely.points(A[:, 1:3])
        for gi in set(gtree.query(shapely.multipoints(A[:, 1:3]))):
            g = gkeys[gi]
            inside = shapely.contains(gpoly[g] if g in ends else gfar[g], pts)
            if not inside.any():
                continue
            s_in = A[inside, 0]
            if g in ends:
                # the stretch from the junction end that its lanes reach without a break
                # (all of a short road between junctions, whose heights then merge)
                # 交差点端から途切れずに届く区間（交差点間の短い道路は全体）
                dist = np.sort(s_in if ends[g] == "predecessor" else L - s_in)
                if not len(dist) or dist[0] > 1.5:
                    continue
                brk = np.nonzero(np.diff(dist) > 1.5)[0]
                reach = float(dist[brk[0]] if len(brk) else dist[-1])
                s0, s1 = (0.0, reach) if ends[g] == "predecessor" else (L - reach, L)
            else:
                s0, s1 = float(s_in.min()), float(s_in.max())     # another junction's lanes over the road / 他の交差点
            holds[rid].append((s0, s1, gname[g]))
            for sv in (s0, s1):
                if END_GAP < sv < L - END_GAP:
                    ins[rid].add(round(sv, 4))
    # two different streets whose lanes come within REACH of each other away from a junction
    # (OSM ways drawn over each other: OitaKosen 188560912 / 129675076 overlap 57 m): the
    # contact stretch is tied to the other street's height / 交差点外で接する別の道路は高さを結ぶ
    rpoly = {}
    if lanes is not None:
        for l in lanes:
            if l["junction"] < 0 and str(l["road"]) in free:
                p = _lane_poly(l)
                if p is not None:
                    rpoly.setdefault(str(l["road"]), []).append(p)
    rids = [k for k in rpoly]
    rbuf = [shapely.buffer(shapely.union_all(rpoly[k]), REACH) for k in rids]
    rtree = shapely.STRtree(rbuf)
    contacts = {}
    for rid, r in free.items():
        smp = rl.get(rid)
        if not smp:
            continue
        A = np.array(smp)
        L = float(r.get("length"))
        linked = {e.get("elementId") for e in (r.find("link") if r.find("link") is not None else [])
                  if e.get("elementType") == "road"}
        pts = shapely.points(A[:, 1:3])
        for oi in set(rtree.query(shapely.multipoints(A[:, 1:3]))):
            o = rids[oi]
            if o == rid or o in mates_of(partner, rid) or o in linked or rid in {
                    e.get("elementId") for e in (free[o].find("link") if free[o].find("link") is not None else [])
                    if e.get("elementType") == "road"}:
                continue
            inside = shapely.contains(rbuf[oi], pts)
            if inside.sum() < 2:
                continue
            s_in = np.sort(A[inside, 0])
            # contiguous pieces / 連続区間ごと
            cut = np.nonzero(np.diff(s_in) > 1.5)[0]
            for seg_ in np.split(s_in, cut + 1):
                s0, s1 = float(seg_[0]), float(seg_[-1])
                contacts.setdefault(rid, []).append((s0, s1, o))
                for sv in (s0, s1):
                    if END_GAP < sv < L - END_GAP:
                        ins[rid].add(round(sv, 4))
    # stations: every <= SEG m, the hold ends inserted; a road with a lower-id partner
    # (the other direction) takes that partner's stations, projected / 測点（上下線は共有）
    pair_of = {}
    mates = {}
    for a_, b_ in partner.items():
        mates.setdefault(a_, set()).add(b_)
        mates.setdefault(b_, set()).add(a_)

    def project(rid, X, Y):
        """(s on rid's centre line, distance) of points. / 中心線への射影"""
        Ks, P = cpos[rid]
        if len(P) < 2:
            return np.full(len(X), -1.0), np.full(len(X), 1e9)
        line = shapely.LineString(P)
        arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
        mp = shapely.points(np.column_stack([X, Y]))
        return np.interp(shapely.line_locate_point(line, mp), arc, Ks), shapely.distance(line, mp)
    for rid in sorted(free, key=int):
        r = free[rid]
        L = float(r.get("length"))
        n = max(1, int(math.ceil(L / SEG - 1e-6)))
        base = list(np.linspace(0.0, L, n + 1))
        # where the other direction ends (at its junction's height) this one needs a
        # station too: the two directions end metres apart on a bend
        # 対向車線の端（交差点の高さ）の位置にも測点（カーブでは端が数 m ずれる）
        for o in mates.get(rid, ()):
            if o not in cpos or int(o) < int(rid):
                continue
            Lo = float(free[o].get("length"))
            # its ends and its hold boundaries: the two get the same station set
            # 対向車線の端と保持区間の端（測点の組を同じに）
            xo, yo = pos(o, np.array([0.0, Lo] + sorted(ins.get(o, ()))))
            sv, dv = project(rid, xo, yo)
            for s_, d_ in zip(sv, dv):
                if d_ < PAIR_D and END_GAP < s_ < L - END_GAP:
                    ins[rid].add(round(float(s_), 4))
        insd = sorted(ins[rid])
        mine = [0.0, L] + insd
        # every opposite road with a lower id that runs beside this one is a master: its
        # stations are projected here (one direction may be split where the other is not)
        # 番号の小さい対向車線（複数可）の測点を射影する
        proj, near = [], []
        for master in sorted((o for o in mates.get(rid, ()) if int(o) < int(rid) and o in st), key=int):
            Sm, xm, ym = st[master]
            sv, dv = project(rid, xm, ym)
            for km, (s_, d_) in enumerate(zip(sv, dv)):
                if d_ >= PAIR_D or s_ < -0.5 or s_ > L + 0.5:
                    continue
                m_ = min(mine, key=lambda v: abs(v - s_))
                if abs(m_ - s_) <= 0.5:
                    near.append((m_, master, km))   # pairs with this road's own end / hold station / 自分の測点と対に
                elif END_GAP < s_ < L - END_GAP:
                    proj.append((float(s_), master, km))
        # two projections within MERGE_ST of each other (two masters overlapping): the first
        # 近すぎる射影は最初のものだけ
        proj.sort()
        thin = []
        for q in proj:
            if not thin or q[0] - thin[-1][0] >= 0.5:
                thin.append(q)
        proj = thin
        S = set(mine) | {q[0] for q in proj}
        fixed = sorted(S)
        if proj or near:
            # the other direction's stations cover this road: only gaps it leaves are
            # filled (an extra station gave the two cubic profiles different intervals;
            # on a bend the outer road's s runs longer, so a covered gap can exceed SEG)
            # 対向車線の測点で覆われる所は追加しない（区間が違うと 3 次式がずれた）
            paired = {round(q[0], 4) for q in proj + near}
            for f0, f1 in zip(fixed, fixed[1:]):
                if round(f0, 4) in paired and round(f1, 4) in paired:
                    continue
                if f1 - f0 > 1.5 * SEG:
                    k = int(math.ceil((f1 - f0) / SEG - 1e-6))
                    S.update(np.linspace(f0, f1, k + 1)[1:-1])
        else:
            for b in base:
                if min(abs(b - f) for f in fixed) >= MERGE_ST:
                    S.add(b)
        S = np.array(sorted(S))
        keep = np.concatenate([[True], np.diff(S) > 1e-3])
        S = S[keep]
        x, y = pos(rid, S)
        st[rid] = (S, x, y)
        for sv, master, km in proj + near:
            k = int(np.argmin(np.abs(S - sv)))
            if (rid, k) not in pair_of:
                pair_of[(rid, k)] = (master, km)
    solve.mates, solve.cpos = mates, cpos             # main() writes the profiles from them / 縦断の書き出し用
    # unknowns: road stations, merged at shared ends; one per junction / 未知数
    for rid, r in free.items():
        n = len(st[rid][0]) - 1
        lk = r.find("link")
        for tag, k in (("predecessor", 0), ("successor", n)):
            e = lk.find(tag) if lk is not None else None
            if e is None:
                continue
            if e.get("elementType") == "junction":
                uf.union(("r", rid, k), ("j", e.get("elementId")))
            elif e.get("elementId") in free:
                o = e.get("elementId")
                ko = 0 if e.get("contactPoint") == "start" else len(st[o][0]) - 1
                uf.union(("r", rid, k), ("r", o, ko))
    nlevel = 0
    for rid, hs in holds.items():
        S = st[rid][0]
        for s0, s1, j in hs:
            for k in np.nonzero((S >= s0 - 1e-6) & (S <= s1 + 1e-6))[0]:
                if uf.find(("r", rid, int(k))) != uf.find(("j", j)):
                    uf.union(("r", rid, int(k)), ("j", j))
                    nlevel += 1
    npair = 0
    for (rid, k), (o, ko) in pair_of.items():
        if uf.find(("r", rid, k)) != uf.find(("r", o, ko)):
            uf.union(("r", rid, k), ("r", o, ko))
            npair += 1
    print("junction overlaps: %d junctions merged with an overlapping neighbour, %d road stations held "
          "at the height of the junction lanes beside or over them; %d stations shared by the two "
          "directions of a street" % (nmerge, nlevel, npair))
    keys = {}

    def var(key):
        r = uf.find(key)
        if r not in keys:
            keys[r] = len(keys)
        return keys[r]
    rows, cols, vals, rhs = [], [], [], []

    def row(entries, b, w):
        i = len(rhs)
        for c_, v in entries:
            rows.append(i)
            cols.append(c_)
            vals.append(v * w)
        rhs.append(b * w)
    lam = (wave / (2 * math.pi)) ** 4
    for rid in free:
        S, x, y = st[rid]
        h = H(x, y)
        n = len(S) - 1
        ix = [var(("r", rid, k)) for k in range(n + 1)]
        dS = np.diff(S) if n else np.array([1.0])
        for k in range(n + 1):
            w = ((dS[k - 1] if k > 0 else 0.0) + (dS[k] if k < n else 0.0)) / 2 if n else 1.0
            row([(ix[k], 1.0)], h[k], math.sqrt(max(w, 1e-3)))
        for k in range(1, n):
            # second derivative on unequal intervals / 不等間隔の 2 階差分
            h0, h1 = dS[k - 1], dS[k]
            row([(ix[k - 1], 2.0 / (h0 * (h0 + h1))), (ix[k], -2.0 / (h0 * h1)), (ix[k + 1], 2.0 / (h1 * (h0 + h1)))],
                0.0, math.sqrt(lam * (h0 + h1) / 2))
        lk = free[rid].find("link")
        for tag, k, k2 in (("predecessor", 0, 1), ("successor", n, n - 1)):
            e = lk.find(tag) if lk is not None else None
            if n and e is not None and e.get("elementType") == "junction":
                row([(ix[k], 1.0), (ix[k2], -1.0)], 0.0, W_FLAT / abs(S[k] - S[k2]))   # leave the junction level / 水平に出る
    for j, pts in jpts.items():
        p = np.array(pts)
        row([(var(("j", j)), 1.0)], float(H(p[:, 0], p[:, 1]).mean()), W_JUNC * math.sqrt(SEG))
    if core is not None:
        # an embedded flat model (meta "core"): every station and junction inside its
        # square at its level, so the road logic = its surface there (the curvature
        # penalty had pulled roads 0.1 m off it near the edge: check_drive steps)
        # 埋め込みモデルの範囲内は全測点・交差点をその高さに（境界近くで 0.1 m ずれた）
        (cx0, cy0, cx1, cy1), cz = core
        ncore = 0
        for rid in free:
            S, x, y = st[rid]
            for k in np.nonzero((x > cx0) & (x < cx1) & (y > cy0) & (y < cy1))[0]:
                row([(var(("r", rid, int(k))), 1.0)], cz, W_CORE)
                ncore += 1
        for j, pts in jpts.items():
            p = np.array(pts)
            if ((p[:, 0] > cx0) & (p[:, 0] < cx1) & (p[:, 1] > cy0) & (p[:, 1] < cy1)).all():
                row([(var(("j", j)), 1.0)], cz, W_CORE)
        print("embedded model: %d stations held at its level %.2f m" % (ncore, cz))
    # the rest of a street's two directions (no shared station there): tied softly
    # 上下線の残り（共有測点の無い所）は弱く結ぶ
    ntie = 0
    for rid, oth in partner.items():
        if oth not in st:
            continue
        S, x, y = st[rid]
        So, xo, yo = st[oth]
        if len(So) < 2:
            continue
        seg_a = np.column_stack([xo[:-1], yo[:-1]])
        seg_b = np.column_stack([xo[1:], yo[1:]])
        for k in range(len(S)):
            p = np.array([x[k], y[k]])
            ab = seg_b - seg_a
            t = np.clip(((p - seg_a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-9), 0, 1)
            q = seg_a + ab * t[:, None]
            d = np.hypot(*(q - p).T)
            m = int(np.argmin(d))
            if d[m] > 1.5:
                continue
            a, b = var(("r", oth, m)), var(("r", oth, m + 1))
            me = var(("r", rid, k))
            if me in (a, b):
                continue
            row([(me, 1.0), (a, -(1 - t[m])), (b, -t[m])], 0.0, W_TIE)
            ntie += 1
    # streets in contact away from junctions: strong ties at every station of the contact
    # stretch to the other street's height there / 接する別の道路: 接触区間の各測点を強く結ぶ
    ncontact = 0
    for rid, cs in contacts.items():
        S, x, y = st[rid]
        for s0, s1, o in cs:
            So, xo, yo = st[o]
            if len(So) < 2:
                continue
            ks = np.nonzero((S >= s0 - 1e-6) & (S <= s1 + 1e-6))[0]
            if not len(ks):
                continue
            so, do = _project(cpos, o, x[ks], y[ks])
            for k, sv in zip(ks, so):
                m = int(np.clip(np.searchsorted(So, sv) - 1, 0, len(So) - 2))
                t = float(np.clip((sv - So[m]) / max(So[m + 1] - So[m], 1e-9), 0.0, 1.0))
                a, b_ = var(("r", o, m)), var(("r", o, m + 1))
                me = var(("r", rid, int(k)))
                if me in (a, b_):
                    continue
                row([(me, 1.0), (a, -(1 - t)), (b_, -t)], 0.0, W_CONTACT)
                ncontact += 1
    print("streets in contact away from junctions: %d roads, %d station ties" % (len(contacts), ncontact))
    A = sp.csr_matrix((vals, (rows, cols)), shape=(len(rhs), len(keys)))
    b = np.array(rhs)
    # grade limit: reweighted penalty on segments steeper than GRADE_SOFT
    # 勾配の上限: 急な区間に重みを足して解き直す
    # on a bend a road's s runs slower than the street's centre line on the inside: the
    # height change over an interval is concentrated where the street turns, and the lane
    # there is steeper than the interval's mean grade (x3 on a 4.5 m bend after the two
    # directions shared one profile). Each interval's grade is measured over its effective
    # length at its tightest point / カーブ内側では s が詰まり局所勾配が大きい: 最も詰まる所の実効長で測る
    seg = []
    for rid in free:
        S = st[rid][0]
        Ks, P = cpos[rid]
        for k in range(len(S) - 1):
            a, b_ = var(("r", rid, k)), var(("r", rid, k + 1))
            d = S[k + 1] - S[k]
            if a == b_ or d <= 1e-6:
                continue
            ss = np.linspace(S[k], S[k + 1], max(3, int(d / 0.5) + 1))
            X, Y = np.interp(ss, Ks, P[:, 0]), np.interp(ss, Ks, P[:, 1])
            du = np.hypot(np.diff(X), np.diff(Y)) / np.diff(ss)        # street centre per road s / 中心線の速さ
            ratio = float(du.max() / max(du.mean(), 1e-6)) if len(du) else 1.0
            seg.append((a, b_, d, d / max(1.0, ratio)))
    ia = np.array([a for a, _, _, _ in seg])
    ib = np.array([b_ for _, b_, _, _ in seg])
    dd = np.array([d for _, _, d, _ in seg])
    de = np.array([e for _, _, _, e in seg])
    D = sp.csr_matrix((np.concatenate([-1 / de, 1 / de]), (np.tile(np.arange(len(seg)), 2), np.concatenate([ia, ib]))),
                      shape=(len(seg), len(keys)))
    N0, r0 = (A.T @ A).tocsc(), A.T @ b
    wg = np.zeros(len(seg))
    for it in range(GRADE_ITER):
        z = spsolve(N0 + (D.T @ sp.diags(wg * dd) @ D).tocsc(), r0)
        g = np.abs(D @ z)
        over = g > GRADE_SOFT
        if not over.any():
            break
        wg = np.where(over, np.maximum(wg, W_GRADE) * (g / GRADE_SOFT) ** 2, wg)
    zj = {j: z[var(("j", j))] for j in set(jpts) | {r.get("junction") for r in roads.values()} - {"-1", None}}
    zr = {rid: np.array([z[var(("r", rid, k))] for k in range(len(st[rid][0]))]) for rid in free}
    return st, zr, zj, two_way, ntie


def crossings_without_junction(st, zr, roads=None):
    """Centre lines of two roads (outside junctions) that cross away from their
    ends: an overpass or a mapping error. [(road, road, x, y, dz)]
    交差点なしで交差する道路（立体交差かデータの誤り）。"""
    from shapely.geometry import LineString
    from shapely.strtree import STRtree
    ids = [r for r in st if len(st[r][0]) >= 2]
    ls = [LineString(np.column_stack([st[r][1], st[r][2]])) for r in ids]
    out = []
    tr = STRtree(ls)
    def ends(rid):
        lk = roads[rid].find("link") if roads else None
        return frozenset((e.get("elementType"), e.get("elementId")) for e in (lk if lk is not None else []))
    for i, j in zip(*tr.query(ls, predicate="crosses")):
        if i >= j:
            continue
        if roads and ends(ids[i]) == ends(ids[j]):
            continue                     # the two directions of one street (a hairpin) / 上下線（ヘアピン）
        x = ls[i].intersection(ls[j])
        for p in getattr(x, "geoms", [x]):
            if p.geom_type != "Point":
                continue
            si, sj = ls[i].project(p), ls[j].project(p)
            if min(si, ls[i].length - si, sj, ls[j].length - sj) < 3.0:
                continue
            ti = np.subtract(ls[i].interpolate(si + 1).coords[0], ls[i].interpolate(si - 1).coords[0])
            tj = np.subtract(ls[j].interpolate(sj + 1).coords[0], ls[j].interpolate(sj - 1).coords[0])
            if abs(np.dot(ti, tj)) / (np.linalg.norm(ti) * np.linalg.norm(tj) + 1e-9) > 0.85:
                continue                 # the two directions of one street / 同じ道路の上下線
            zi = np.interp(si / max(ls[i].length, 1e-9), np.linspace(0, 1, len(zr[ids[i]])), zr[ids[i]])
            zj = np.interp(sj / max(ls[j].length, 1e-9), np.linspace(0, 1, len(zr[ids[j]])), zr[ids[j]])
            out.append((ids[i], ids[j], round(p.x), round(p.y), round(float(zi - zj), 2)))
    return out


def main():
    out = sys.argv[1]
    meta = json.load(open(os.path.join(out, "meta.json")))
    xodr = os.path.join(out, meta["name"] + ".xodr")
    H = load_field(out)
    if H is None:
        sys.exit("no terrain_field.npz in %s (flat map): run terrain.py first" % out)
    tree = ET.parse(xodr)
    root = tree.getroot()
    cl = centre_lines(os.path.join(out, "lanes.json"))
    core = None
    if meta.get("core"):
        from core_merge import core_rect
        tf = np.load(os.path.join(out, "terrain_field.npz"))
        if "core_z" in tf.files and not np.isnan(float(tf["core_z"])):
            core = (core_rect(meta), float(tf["core_z"]))
    st, zr, zj, two_way, ntie = solve(root, H, cl, lanes=json.load(open(os.path.join(out, "lanes.json")))["lanes"],
                                      core=core)
    roads = {r.get("id"): r for r in root.findall("road")}
    # slopes: central differences; level at junctions; shared at road links
    # 勾配: 中心差分、交差点で 0、道路同士の接続で共有
    slope = {}
    for rid, z in zr.items():
        S = st[rid][0]
        m = np.zeros(len(S))
        if len(S) > 2:
            m[1:-1] = (z[2:] - z[:-2]) / (S[2:] - S[:-2])
        if len(S) > 1:
            m[0] = (z[1] - z[0]) / (S[1] - S[0])
            m[-1] = (z[-1] - z[-2]) / (S[-1] - S[-2])
        slope[rid] = m
    ends = {}
    for rid in zr:
        lk = roads[rid].find("link")
        for tag, k in (("predecessor", 0), ("successor", -1)):
            e = lk.find(tag) if lk is not None else None
            if e is None:
                continue
            if e.get("elementType") == "junction":
                slope[rid][k] = 0.0
            elif e.get("elementId") in zr:
                o = e.get("elementId")
                ko = 0 if e.get("contactPoint") == "start" else -1
                # slope away from the shared point, both sides / 接続点から離れる向きの勾配
                a = slope[rid][k] * (1 if k == 0 else -1)
                b = slope[o][ko] * (1 if ko == 0 else -1)
                g = (a - b) / 2
                ends[(rid, k)] = g * (1 if k == 0 else -1)
                ends[(o, ko)] = -g * (1 if ko == 0 else -1)
    for (rid, k), v in ends.items():
        slope[rid][k] = v
    # beside a stretch held level (at a junction's height) the slope is 0: a cubic with a
    # central-difference slope there bulged out of the level stretch by centimetres
    # 水平に保った区間の両端は勾配 0（中心差分の勾配だと 3 次式が水平区間から膨らんだ）
    for rid, z in zr.items():
        flat = np.abs(np.diff(z)) < 1e-6
        for k in range(len(z)):
            if (k > 0 and flat[k - 1]) or (k < len(z) - 1 and flat[k]):
                slope[rid][k] = 0.0
    profs, grades = {}, []
    mates, cpos = getattr(solve, "mates", {}), getattr(solve, "cpos", {})
    nfollow = 0
    # lower ids first: a street's other direction is written from the profile of the road
    # it pairs with (its master), as dense samples at the projections of its own centre
    # points -- each road's cubic runs in its own s, and on a bend the two directions'
    # s differ (inside / outside of the curve): the same stations still gave 0.1 m apart
    # 番号の小さい道路から: 対向車線は相手の縦断を自分の中心線の射影点で密に標本化して書く
    for rid, r in sorted(roads.items(), key=lambda kv: int(kv[0])):
        L = float(r.get("length"))
        if rid in zr:
            prof = hermite(st[rid][0], zr[rid], slope[rid])
            masters = [o for o in mates.get(rid, ()) if int(o) < int(rid) and o in profs and o in cpos]
            if masters and rid in cpos:
                s = np.unique(np.r_[np.arange(0.0, L, DENSE_S), L])
                X, Y = _centre(cpos, rid, s)
                z = prof_eval(prof, s)
                cov = np.zeros(len(s), bool)
                for m in sorted(masters, key=int):
                    sm, dm = _project(cpos, m, X, Y)
                    Lm = float(roads[m].get("length"))
                    ok = (dm < PAIR_D) & (sm > -1e-3) & (sm < Lm + 1e-3) & ~cov
                    if ok.any():
                        z[ok] = prof_eval(profs[m], np.clip(sm[ok], 0.0, Lm))
                        cov |= ok
                if cov.any():
                    m_ = np.gradient(z, s)
                    lk = r.find("link")
                    for tag, k in (("predecessor", 0), ("successor", -1)):
                        e = lk.find(tag) if lk is not None else None
                        if e is not None and e.get("elementType") == "junction":
                            m_[k] = 0.0                            # level junctions / 交差点は水平
                    flat = np.abs(np.diff(z)) < 1e-6
                    m_[np.r_[flat, False] | np.r_[False, flat]] = 0.0
                    prof = hermite(s, z, m_)
                    nfollow += 1
        else:
            prof = [(0.0, float(zj.get(r.get("junction"), 0.0)), 0.0, 0.0, 0.0)]
        ep = r.find("elevationProfile")
        if ep is None:
            ep = ET.Element("elevationProfile")
            r.insert(list(r).index(r.find("planView")) + 1, ep)
        for el in list(ep):
            ep.remove(el)
        for s0, a, b, c, d in prof:
            ET.SubElement(ep, "elevation", s="%.4f" % s0, a="%.4f" % a, b="%.6f" % b, c="%.8f" % c, d="%.10f" % d)
        profs[rid] = prof
        grades.append((grade_of(prof, L), rid, r.get("junction"), r.get("name", ""), L))
    tree.write(xodr, encoding="utf-8", xml_declaration=True)
    cross = crossings_without_junction(st, zr, roads)
    grades.sort(reverse=True)
    G = np.array([g for g, *_ in grades])
    allz = [p[1] for pr in profs.values() for p in pr]
    # how far the roads sit from the terrain (cut < 0 < fill) / 地形との差（切土・盛土）
    dz = np.concatenate([zr[rid] - H(st[rid][1], st[rid][2]) for rid in zr])
    print("elevated %d roads (%d junctions level, %d two-way roads, %d written from the other direction's "
          "profile, %d soft ties left): z %.1f..%.1f m; road - terrain %.1f..%.1f m (95%% within %.1f m)"
          % (len(profs), len(zj), sum(two_way.values()), nfollow, ntie, min(allz), max(allz), dz.min(), dz.max(),
             np.percentile(np.abs(dz), 95)))
    print("max grade %.1f%%, median %.1f%%; > %d%%: %d roads (%.0f m)"
          % (100 * G.max(), 100 * float(np.median(G)), FLAG * 100, int((G > FLAG).sum()),
             sum(L for g, _, _, _, L in grades if g > FLAG)))
    print("grade separation: %d roads cross without a junction%s" % (len(cross), "" if not cross else
          " -- NOT lifted (needs the bridge step): %s" % cross[:6]))
    for g, rid, j, name, L in grades[:10]:
        print("  road %s%s %s %.0f m: %.1f%%" % (rid, " (junction %s)" % j if j != "-1" else "", name, L, 100 * g))
    json.dump({"max_grade": float(G.max()), "over_flag": int((G > FLAG).sum()),
               "road_minus_terrain": [float(dz.min()), float(dz.max())],
               "steepest": [(rid, name, round(L), round(100 * g, 1)) for g, rid, j, name, L in grades[:40]]},
              open(os.path.join(out, "elevation_report.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
