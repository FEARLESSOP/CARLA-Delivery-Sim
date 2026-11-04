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


def solve(root, H, cl, wave=SMOOTH_WAVE, lanes=None):
    import scipy.sparse as sp
    from scipy.sparse.linalg import spsolve
    from scipy.spatial import cKDTree
    roads = {r.get("id"): r for r in root.findall("road")}
    free = {rid: r for rid, r in roads.items() if r.get("junction", "-1") == "-1"}
    # stations and centre points / 測点と中心点
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
    for rid, r in free.items():
        L = float(r.get("length"))
        n = max(1, int(math.ceil(L / SEG - 1e-6)))
        S = np.linspace(0.0, L, n + 1)
        c = cl.get(int(rid))
        if c is not None:
            Ks, Kk, Ko, _ = c
            t = 1.0 if two_way[rid] else 0.5            # the street's centre / 道路の中心
            P = Kk + (Ko - Kk) * t
            x = np.interp(S, Ks, P[:, 0])
            y = np.interp(S, Ks, P[:, 1])
        else:
            at = road_sampler(r)
            Q = np.array([at(v) for v in S])
            x, y = Q[:, 0], Q[:, 1]
        st[rid] = (S, x, y)
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
    # unknowns: road stations, merged at shared ends; one per junction / 未知数
    uf = UF()
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
    # junctions whose connecting lanes overlap each other (two junctions a few
    # metres apart) share one height / 接続車線が重なる交差点同士は同じ高さ
    nmerge = 0
    J = R_ = None
    if lanes is not None:
        jp_, rp_ = [], []
        for l in lanes:
            for q in l["pts"]:
                for k_ in ("c", "l", "r"):
                    if l["junction"] >= 0:
                        jp_.append((q[k_][0], q[k_][1], l["junction"]))
                    elif str(l["road"]) in free:
                        rp_.append((q[k_][0], q[k_][1], l["road"], q["s"]))
        J, R_ = np.array(jp_), np.array(rp_)
        if len(J):
            for a, b in cKDTree(J[:, :2]).query_pairs(0.6, output_type="ndarray"):
                ja, jb = str(int(J[a, 2])), str(int(J[b, 2]))
                if ja != jb and uf.find(("j", ja)) != uf.find(("j", jb)):
                    uf.union(("j", ja), ("j", jb))
                    nmerge += 1
    nlevel = 0
    if J is not None and len(J) and len(R_):
        hits = cKDTree(R_[:, :2]).query_ball_point(J[:, :2], 0.6)
        reach, spots = {}, set()
        for jrow, hh in zip(J, hits):
            for h in hh:
                rid, sv, j = str(int(R_[h, 2])), R_[h, 3], str(int(jrow[2]))
                lk = free[rid].find("link")
                L_ = float(free[rid].get("length"))
                linked = False
                for tag in ("predecessor", "successor"):
                    e = lk.find(tag) if lk is not None else None
                    if e is not None and e.get("elementType") == "junction" and e.get("elementId") == j:
                        dist = sv if tag == "predecessor" else L_ - sv
                        if dist < 0.5 * L_:
                            key = (rid, tag, j)
                            reach[key] = max(reach.get(key, 0.0), dist)
                            linked = True
                if not linked:                   # another junction's lane over this road / 他の交差点の車線
                    S = st[rid][0]
                    k = int(np.argmin(np.abs(S - sv)))
                    spots.add((rid, k, j))
        for (rid, tag, j), dist in reach.items():
            S = st[rid][0]
            L_ = S[-1]
            for k in range(len(S)):
                dk = S[k] if tag == "predecessor" else L_ - S[k]
                if 0 < dk <= dist + 1.0:
                    spots.add((rid, k, j))
        for rid, k, j in spots:          # the same unknown: exact / 同じ未知数（厳密に一致）
            if uf.find(("r", rid, k)) != uf.find(("j", j)):
                uf.union(("r", rid, k), ("j", j))
                nlevel += 1
    print("junction overlaps: %d junctions merged with an overlapping neighbour, %d road stations held "
          "at the height of the junction lanes over them" % (nmerge, nlevel))
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
        ds = S[1] - S[0] if n else 1.0
        ix = [var(("r", rid, k)) for k in range(n + 1)]
        for k in range(n + 1):
            row([(ix[k], 1.0)], h[k], math.sqrt(ds))
        for k in range(1, n):
            row([(ix[k - 1], 1.0), (ix[k], -2.0), (ix[k + 1], 1.0)], 0.0, math.sqrt(lam * ds) / ds ** 2)
        lk = free[rid].find("link")
        for tag, k, k2 in (("predecessor", 0, 1), ("successor", n, n - 1)):
            e = lk.find(tag) if lk is not None else None
            if n and e is not None and e.get("elementType") == "junction":
                row([(ix[k], 1.0), (ix[k2], -1.0)], 0.0, W_FLAT / ds)      # leave the junction level / 水平に出る
    for j, pts in jpts.items():
        p = np.array(pts)
        row([(var(("j", j)), 1.0)], float(H(p[:, 0], p[:, 1]).mean()), W_JUNC * math.sqrt(SEG))
    # where a junction's connecting lanes lie over the first metres of a road
    # leaving it (netconvert's wide junction shapes), that stretch of the road
    # is at the junction's height too: cars on either lane meet one surface
    # 交差点の接続車線が道路の始まりに重なる区間は交差点と同じ高さ
    # the two directions of a street at one height / 上下線を同じ高さに
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
    A = sp.csr_matrix((vals, (rows, cols)), shape=(len(rhs), len(keys)))
    b = np.array(rhs)
    # grade limit: reweighted penalty on segments steeper than GRADE_SOFT
    # 勾配の上限: 急な区間に重みを足して解き直す
    seg = [(var(("r", rid, k)), var(("r", rid, k + 1)), st[rid][0][k + 1] - st[rid][0][k])
           for rid in free for k in range(len(st[rid][0]) - 1)]
    seg = [(a, b_, d) for a, b_, d in seg if a != b_ and d > 1e-6]
    ia = np.array([a for a, _, _ in seg])
    ib = np.array([b_ for _, b_, _ in seg])
    dd = np.array([d for _, _, d in seg])
    D = sp.csr_matrix((np.concatenate([-1 / dd, 1 / dd]), (np.tile(np.arange(len(seg)), 2), np.concatenate([ia, ib]))),
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
    st, zr, zj, two_way, ntie = solve(root, H, cl, lanes=json.load(open(os.path.join(out, "lanes.json")))["lanes"])
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
    profs, grades = {}, []
    for rid, r in roads.items():
        L = float(r.get("length"))
        if rid in zr:
            prof = hermite(st[rid][0], zr[rid], slope[rid])
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
    print("elevated %d roads (%d junctions level, %d two-way roads, %d ties): z %.1f..%.1f m; "
          "road - terrain %.1f..%.1f m (95%% within %.1f m)"
          % (len(profs), len(zj), sum(two_way.values()), ntie, min(allz), max(allz), dz.min(), dz.max(),
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
