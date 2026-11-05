"""Forward turning loops at dead ends (road_fixes maps; carla-venv).

  carla-venv/bin/python turnloops.py <Name.xodr> [<out.xodr>]   (roadnet.py runs it)

netconvert keeps a U-turn connection at every dead end
(`--no-turnarounds.except-deadend`), but draws it inside out: its reference
line starts half a lane beside the lane and the lane another half lane
further, so the 3 m lane, turned in a ~1.5 m radius, starts 2.8 m sideways,
heading ~150 deg reversed, and loops back over the street (OitaKosen live,
2026-11-05: 83 dead ends, TM cars drove into the head's kerb).

Each dead-end connector's plan view is replaced by a forward "light bulb"
loop, G1 continuous: a swing arc away from the turn (radius r1, tangent to
the incoming lane's end), the loop round a centre C (radius R, the turn's
direction) and a swing arc back (r2, tangent to the outgoing lane's start).
Each swing arc is the circle tangent to the lane's line at its end that
touches the loop circle from outside, so C and R fix the whole path. C is
searched in front of the mouth (symmetric first, then shifted sideways
towards the open side), the loop may start up to MAX_BACK m back in the
street (the street is shortened; bent netconvert tails go whole), and R
from 5 m down: the first loop whose lane keeps CLR_GOOD m off every other
lane and clear of the buildings (PASSES) wins, else the one with the most
room (>= CLR_MIN). The loop is written as three <arc>s of the reference
line (lane -1 lies on its right: the reference line is the centre line
offset w/2 to the left). Junction, connection and lane records, widths and
speeds are kept; elevate.py gives it the junction's (level) height.

袋小路の転回接続（netconvert は裏返しに描く）を、前進する電球形のループ
（振り出し円弧 + 転回円 + 戻り円弧、接線連続）に置き換える。
"""

import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.geometry import LineString

SWING_MIN = 4.6                 # swing arcs at least this: a car's rear axle turns >= ~4.1 m, and CARLA steps
                                # waypoints along the reference line (r - w/2): 1 m there stays < 20 deg / 振り出し円弧の最小半径
CLR_GOOD = 2.5                  # loop lane edge to other lanes: paving margin + gutter / 他車線との離れ（目標）
CLR_MIN = 1.0                   # never closer (elevate.py merges lanes within 0.6 m) / 最小離れ
STEP = 0.5
APART_DZ = 0.3                  # a lane this much higher or lower is "at another height" (layout.py) / 高さの違う車線
CLR_APART = 2.0 + 3.7 + 0.2     # loop lane edge to it: head margin + layout.HEAD_CLEAR (+ slack) / その車線との離れ
KINK_LEN, KINK_DEG = 6.0, 8.0    # no bend sharper than this per 0.5 m within this of the loop / 折れの判定
MAX_BACK = 16                   # the loop may start this far back in the street (m) / ループ開始の最大後退
# search passes: (building clearance of the loop lane edge, radii, smallest swing arc)
# 探索の段階: (建物との離れ, 半径, 振り出し円弧の最小半径)
PASSES = ((5.2, (5.0, 4.5, 4.0), SWING_MIN),   # margin 2.0 + building clearance 2.2 + facade 1.0: nothing cut
          (2.5, (5.0, 4.5, 4.0), SWING_MIN),   # buildings cut back, not dropped / 建物は削るが残る
          (None, (5.0, 4.5, 4.0), SWING_MIN),  # the best building clearance / 建物との離れ最大
          (None, (3.5,), SWING_MIN),
          (None, (5.0, 4.5, 4.0, 3.5), 3.0))


def _wp_pose(m, rid, lid, s):
    w = m.get_waypoint_xodr(int(rid), int(lid), s)
    if w is None:
        return None
    t = w.transform
    return np.array([t.location.x, -t.location.y]), math.radians(-t.rotation.yaw), w.lane_width


def _nrm(h, k):
    """Unit normal on side k of heading h (k=+1 left, -1 right). / 進行方向の左(+1)右(-1)の法線"""
    return np.array([-math.sin(h), math.cos(h)]) * k


def _ang(u, v, k):
    """Angle from vector u to v turning in direction k (+1 CCW), in [0, 2 pi). / u から v への回転角"""
    a = math.atan2(u[0] * v[1] - u[1] * v[0], u @ v) * k
    return a % (2 * math.pi)


def solve_loop(P0, h0, P1, h1, C, R, km, swing_min):
    """Arcs (centre, radius, start angle point, sweep, turn sign) of the bulb loop
    round C, or None. km: the loop's turn sign (-1 clockwise). / ループの円弧列"""
    ks = -km
    arcs = []
    swings = []
    for P, h in ((P0, h0), (P1, h1)):
        n = _nrm(h, ks)
        d = P - C
        den = 2 * (d @ n - R)
        if den >= -1e-9:
            return None
        r = (R * R - d @ d) / den
        if not (swing_min <= r <= 4 * R + 10):
            return None
        L = P + r * n
        T = L + r * (C - L) / np.linalg.norm(C - L)
        swings.append((L, r, T))
    (L1, r1, T1), (L2, r2, T2) = swings
    a1 = _ang(P0 - L1, T1 - L1, ks)
    th = _ang(T1 - C, T2 - C, km)
    a2 = _ang(T2 - L2, P1 - L2, ks)
    if a1 > math.radians(120) or a2 > math.radians(120) or not (math.radians(180) < th < math.radians(345)):
        return None
    arcs = [(P0, h0, ks / r1, r1 * a1), (T1, h0 + ks * a1, km / R, R * th),
            (T2, h0 + ks * a1 + km * th, ks / r2, r2 * a2)]
    return arcs


def sample(arcs, step=STEP):
    """Centre-line points (x, y, hdg) every ~step m along the arcs. / 中心線の点列"""
    out = []
    for P, h, k, L in arcs:
        n = max(2, int(math.ceil(L / step)) + 1)
        for t in np.linspace(0, L, n):
            out.append((P[0] + (math.sin(h + k * t) - math.sin(h)) / k,
                        P[1] - (math.cos(h + k * t) - math.cos(h)) / k, h + k * t))
    return np.array(out)


def lane_polys(m, root, skip, piece=None):
    """Every driving lane's outline from the waypoints, except roads in skip:
    (road, outline, lowest z, highest z), in pieces of `piece` m when given (a
    hillside lane changes height along its length). / 全車線の外形（と高さ）"""
    polys = []
    for road in root.iter("road"):
        rid = int(road.get("id"))
        if rid in skip:
            continue
        L = float(road.get("length"))
        lids = {int(l.get("id")) for l in road.iter("lane") if l.get("type") == "driving"}
        for lid in lids:
            n = max(2, int(math.ceil(L)) + 1)
            pts, ws, zs = [], [], []
            for s in np.linspace(0.01, L - 0.01, n):
                w = m.get_waypoint_xodr(rid, lid, float(s))
                if w is not None:
                    t = w.transform
                    pts.append((t.location.x, -t.location.y))
                    ws.append(w.lane_width)
                    zs.append(t.location.z)
            if len(pts) < 2:
                continue
            k = len(pts) if not piece else max(2, int(piece) + 1)
            for i in range(0, len(pts) - 1, k - 1):
                q = pts[i:i + k]
                if len(q) >= 2:
                    polys.append((rid, LineString(q).buffer(max(ws[i:i + k]) / 2, quad_segs=4),
                                  min(zs[i:i + k]), max(zs[i:i + k])))
    return polys


def clearance(pts, w, obst):
    """Smallest gap from the loop lane (centre points, width w) to the obstacles. / 他車線までの最小離れ"""
    if obst is None:
        return 99.0
    return float(np.min(shapely.distance(shapely.points(pts[:, :2]), obst))) - w / 2


def candidates(P0, h0, P1, h1, R, swing_min):
    """Loops of radius R between two lane ends, nearest first. / 半径 R のループ候補"""
    d = P1 - P0
    lat = -d[0] * math.sin(h0) + d[1] * math.cos(h0)       # + left / 左が正
    km = -1 if lat < 0 else 1                              # keep-left: the loop turns right / 左側通行は右回り
    M = (P0 + P1) / 2
    fw = np.array([math.cos(h0), math.sin(h0)])
    sd = _nrm(h0, km)                                      # towards the turn's side / 転回側
    out = []
    for g in np.arange(-6.0, 6.01, 0.5):
        for f in np.arange(R * 0.5, R + 12.0, 0.25):
            arcs = solve_loop(P0, h0, P1, h1, M + fw * f + sd * g, R, km, swing_min)
            if arcs is not None:
                out.append((f + 0.8 * abs(g), f, g, arcs))
                break                                      # the nearest f for this g / この g で最短の f
    return sorted(out, key=lambda c: c[0])


def plan_loop(ends, w, obst, bld=None):
    """Search the set-back (how far the loop starts before the junction), R and C
    for one dead end. ends(back) -> (P0, h0, P1, h1). PASSES in order: a pass
    with a building clearance takes the first loop (big radius first, then the
    least set-back) whose lane keeps CLR_GOOD m off other lanes and that far off
    the buildings; a pass without one (None) takes the loop with the best
    building clearance, radius and set-back. Last resort: the most room
    (>= CLR_MIN m). Returns (arcs, info). / 1 つの袋小路のループを探す"""
    fallback = None
    for p, (bmin, Rs, swing_min) in enumerate(PASSES):
        if bld is None and bmin is not None and bmin > 0:
            continue
        order = sorted(((R, b) for R in Rs for b in ends.backs),
                       key=lambda rb: (0 if rb[0] >= 4.5 else 1 if rb[0] >= 4.0 else 2, rb[1], -rb[0]))
        best = None
        for R, back in order:
            P0, h0, P1, h1 = ends(back)
            for cost, f, g, arcs in candidates(P0, h0, P1, h1, R, swing_min):
                pts = sample(arcs)
                c = clearance(pts, w, obst)
                info = dict(R=R, f=float(f), g=float(g), back=back, clr=c, swing_min=swing_min, tier=p)
                if c < CLR_GOOD:
                    score = R + min(c, CLR_GOOD) - 0.1 * back
                    if c >= CLR_MIN and (fallback is None or score > fallback[0]):
                        fallback = (score, arcs, dict(info, bld=clearance(pts, w, bld)))
                    continue
                cb = clearance(pts, w, bld)
                info["bld"] = cb
                if bmin is not None:
                    if cb >= bmin:
                        return arcs, info
                    continue
                score = min(cb, PASSES[0][0]) + 0.5 * R - 0.1 * back - 0.05 * cost
                if best is None or score > best[0]:
                    best = (score, arcs, info)
        if best is not None:
            return best[1], best[2]
    if fallback is not None:
        return fallback[1], fallback[2]
    return None, None


def write_road(road, arcs, w):
    """Replace the road's plan view by the loop's reference-line arcs. / 参照線の円弧を書く"""
    pv = road.find("planView")
    for g in list(pv):
        pv.remove(g)
    s = 0.0
    for P, h, k, L in arcs:
        r = 1.0 / k                                         # signed centre-line radius / 符号付き半径
        rr = r - w / 2                                      # the reference line, w/2 to the left / 参照線
        kr = 1.0 / rr
        ref = P + _nrm(h, 1) * (w / 2)
        Lr = L * rr / r
        g = ET.SubElement(pv, "geometry", s="%.8f" % s, x="%.8f" % ref[0], y="%.8f" % ref[1],
                          hdg="%.8f" % math.atan2(math.sin(h), math.cos(h)), length="%.8f" % Lr)
        ET.SubElement(g, "arc", curvature="%.10f" % kr)
        s += Lr
    road.set("length", "%.8f" % s)
    return s


def _arcs_of(road, w):
    """The centre-line arcs (start, hdg, curvature, length) of a loop written by
    write_road. / 書いたループの中心線円弧"""
    out = []
    for g in road.findall("planView/geometry"):
        kr = float(g.find("arc").get("curvature"))
        rr = 1.0 / kr
        r = rr + w / 2
        h = float(g.get("hdg"))
        P = np.array([float(g.get("x")), float(g.get("y"))]) - _nrm(h, 1) * (w / 2)
        out.append((P, h, 1.0 / r, float(g.get("length")) * r / rr))
    return out


def _z_at(road, sv):
    """Road height at s from its elevationProfile. / 縦断線形の高さ"""
    recs = road.findall("elevationProfile/elevation")
    e = [r for r in recs if float(r.get("s")) <= sv + 1e-6][-1] if recs else None
    if e is None:
        return 0.0
    ds = sv - float(e.get("s"))
    a, b, c, d = (float(e.get(k)) for k in "abcd")
    return a + b * ds + c * ds * ds + d * ds ** 3


def _rebase(el, ds):
    """Shift a cubic record (a, b, c, d) by ds along s. / 3 次式の原点をずらす"""
    a, b, c, d = (float(el.get(k)) for k in "abcd")
    el.set("a", "%.8f" % (a + b * ds + c * ds * ds + d * ds ** 3))
    el.set("b", "%.8f" % (b + 2 * c * ds + 3 * d * ds * ds))
    el.set("c", "%.8f" % (c + 3 * d * ds))


def _cut_at(road, back, at_end):
    """Where a cut `back` m from the dead end falls: (index of the element it
    falls in, metres into that element from its start) or None when that
    element is not a straight line (and the cut is not on an element border).
    / 切断点の位置（直線要素内か要素境界のみ可）"""
    gs = road.findall("planView/geometry")
    L = float(road.get("length"))
    sc = L - back if at_end else back
    for i, g in enumerate(gs):
        s0, gl = float(g.get("s")), float(g.get("length"))
        if s0 - 1e-6 <= sc <= s0 + gl + 1e-6:
            t = sc - s0
            if t < 1e-3 or t > gl - 1e-3 or list(g)[0].tag == "line":
                return i, min(max(t, 0.0), gl)
    return None


def can_cut(road, back, at_end):
    """A street can lose `back` m at the dead end: the cut falls on a straight
    line or an element border (netconvert bends the last metres of a dead-end
    street towards its turnaround: those elements go whole), one lane
    section, constant widths, no signal or object in the part cut off.
    / 袋小路側を back m 短くできるか"""
    L = float(road.get("length"))
    if L < back + 2.0 or _cut_at(road, back, at_end) is None:
        return False
    if len(road.findall("lanes/laneSection")) != 1:
        return False
    if any(float(x.get(k, 0)) != 0.0 for x in road.iter("width") for k in "bcd"):
        return False
    for el in list(road.iter("signal")) + list(road.iter("object")):
        s = float(el.get("s"))
        if (at_end and s > L - back - 0.5) or (not at_end and s < back + 0.5):
            return False
    return True


def cut(road, back, at_end):
    """Shorten a street by `back` m at its dead end (see can_cut). / 袋小路側を短くする"""
    if back <= 0:
        return
    L = float(road.get("length"))
    i, t = _cut_at(road, back, at_end)
    pv = road.find("planView")
    gs = road.findall("planView/geometry")
    road.set("length", "%.8f" % (L - back))
    if at_end:
        for g in gs[i + 1:]:
            pv.remove(g)
        if t < 1e-3 and i > 0:
            pv.remove(gs[i])
        else:
            gs[i].set("length", "%.8f" % t)
        rest = road.findall("planView/geometry")
        road.set("length", "%.8f" % (float(rest[-1].get("s")) + float(rest[-1].get("length"))))
        for tag in ("elevationProfile", "lateralProfile"):
            par = road.find(tag)
            for el in list(par) if par is not None else []:
                if float(el.get("s")) >= L - back - 1e-6:
                    par.remove(el)
        return
    for g in gs[:i]:
        pv.remove(g)
    g = gs[i]
    gl = float(g.get("length"))
    if t > gl - 1e-3:
        pv.remove(g)
    elif t > 0:
        h = float(g.get("hdg"))
        g.set("x", "%.8f" % (float(g.get("x")) + t * math.cos(h)))
        g.set("y", "%.8f" % (float(g.get("y")) + t * math.sin(h)))
        g.set("length", "%.8f" % (gl - t))
        g.set("s", "%.8f" % back)
    acc = 0.0                       # elements back to back from s = 0 (CARLA needs one at 0) / s=0 から連続
    for x in road.findall("planView/geometry"):
        x.set("s", "%.8f" % acc)
        acc += float(x.get("length"))
    road.set("length", "%.8f" % acc)
    for el in list(road.iter("signal")) + list(road.iter("object")):
        el.set("s", "%.8f" % (float(el.get("s")) - back))
    for tag in ("elevationProfile", "lateralProfile"):
        par = road.find(tag)
        if par is None:
            continue
        recs = list(par)
        for j, el in enumerate(recs):
            s0 = float(el.get("s"))
            s1 = float(recs[j + 1].get("s")) if j + 1 < len(recs) else 1e18
            if s1 <= back:
                par.remove(el)
            elif s0 < back:
                _rebase(el, back - s0)
                el.set("s", "0")
            else:
                el.set("s", "%.8f" % (s0 - back))


def kinked(m, rid, s_from, s_to):
    """Largest lane heading change per 0.5 m between s_from and s_to (deg): the
    bent tails netconvert gives dead-end streets. / 車線の折れ（0.5 m あたりの最大方位変化）"""
    n = max(2, int(abs(s_to - s_from) / 0.5) + 1)
    ys = []
    for s in np.linspace(s_from, s_to, n):
        p = _wp_pose(m, rid, -1, float(s))
        if p is not None:
            ys.append(p[1])
    return max([abs((math.degrees(b - a) + 180) % 360 - 180) for a, b in zip(ys, ys[1:])] or [0.0])


def fix(xodr_in, xodr_out, log=print, buildings=None, refine=False, sites=None):
    """buildings: footprints (map frame) the loops should keep clear of. refine
    (after elevate.py, the xodr has heights): only the loops whose head would
    come within layout.HEAD_CLEAR of a lane at another height (> APART_DZ) are
    planned again from where they are, that lane counted CLR_APART - CLR_GOOD m
    nearer (the head ends in a drop otherwise); the rest are kept.
    / buildings: 避ける建物。refine（標高付与後）: 高さの違う車線に近い転回部だけ
    再計画する（その車線は CLR_APART m 離す）。"""
    import carla
    text = open(xodr_in).read()
    tree_x = ET.ElementTree(ET.fromstring(text))
    root = tree_x.getroot()
    m = carla.Map("loops", text)
    roads = {r.get("id"): r for r in root.findall("road")}
    dead = []
    for j in root.findall("junction"):
        cons = j.findall("connection")
        if not cons:
            continue
        items = []
        for c in cons:
            cr = roads[c.get("connectingRoad")]
            L = float(cr.get("length"))
            for ll in c.findall("laneLink"):
                a = _wp_pose(m, cr.get("id"), ll.get("to"), 0.01)
                b = _wp_pose(m, cr.get("id"), ll.get("to"), L - 0.01)
                if a is None or b is None:
                    break
                dh = abs((math.degrees(b[1] - a[1]) + 180) % 360 - 180)
                items.append((c, ll, dh))
        if items and all(dh > 100 for _, _, dh in items) and len(items) == 1:
            dead.append((j, items[0][0], items[0][1]))
    plans = []
    for j, c, ll in dead:
        cr = roads[c.get("connectingRoad")]
        lk = cr.find("link")
        pr, su = lk.find("predecessor"), lk.find("successor")
        lto = next((int(x.get("id")) for x in cr.iter("lane") if int(x.get("id")) != 0), 0)
        lout = cr.find(".//lane[@id='-1']/link/successor")
        if pr.get("contactPoint") != "end" or su.get("contactPoint") != "start" or lto != -1 \
                or ll.get("from") != "-1" or lout is None or lout.get("id") != "-1":
            log("turnloops: junction %s: not a plain one-lane turnaround, kept" % j.get("id"))
            continue
        plans.append((j, cr, roads[pr.get("elementId")], roads[su.get("elementId")]))
    polys = lane_polys(m, root, set() if refine else {int(cr.get("id")) for _, cr, _, _ in plans},
                       piece=10.0 if refine else None)
    tree = shapely.STRtree([p[1] for p in polys])
    btree = shapely.STRtree(buildings) if buildings else None
    report, failed = [], []
    for j, cr, inc, out in plans:
        own = {int(cr.get("id")), int(inc.get("id")), int(out.get("id"))}
        w = float(cr.find(".//lane[@id='-1']/width").get("a"))
        Li, Lo = float(inc.get("length")), float(out.get("length"))

        def ends(back, inc=inc, out=out, Li=Li):
            a = _wp_pose(m, inc.get("id"), -1, Li - back - 0.001)
            b = _wp_pose(m, out.get("id"), -1, back + 0.001)
            return a[0], a[1], b[0], b[1]
        # set-backs: whole metres and the element borders near the dead end;
        # none that leaves a bent tail before the loop / 後退量: 整数 m と要素境界（折れが残らないこと）
        borders = {round(Li - float(g.get("s")), 3) for g in inc.findall("planView/geometry")} | \
                  {round(float(g.get("s")), 3) for g in out.findall("planView/geometry")}
        backs = sorted({float(b) for b in range(0, MAX_BACK + 1)} | {b for b in borders if 0 < b <= MAX_BACK + 3})
        ends.backs = [b for b in backs if (b == 0 or (can_cut(inc, b, True) and can_cut(out, b, False)))
                      and kinked(m, inc.get("id"), max(0.01, Li - b - KINK_LEN), Li - b - 0.01) <= KINK_DEG
                      and kinked(m, out.get("id"), b + 0.01, min(Lo - 0.01, b + KINK_LEN)) <= KINK_DEG]
        if not ends.backs:
            ends.backs = [0.0]
        P0 = ends(0)[0]
        idx = [i for i in tree.query(shapely.Point(P0).buffer(45.0)) if polys[i][0] not in own]
        if refine:
            # lanes at another height count CLR_APART - CLR_GOOD m nearer / 高さの違う車線は近いとみなす
            zh = float(cr.find("elevationProfile/elevation").get("a"))
            apart = [i for i in idx if max(abs(polys[i][2] - zh), abs(polys[i][3] - zh)) > APART_DZ]
            near = [polys[i][1] for i in idx if i not in set(apart)] + \
                [polys[i][1].buffer(CLR_APART - CLR_GOOD, quad_segs=4) for i in apart]
        else:
            near = [polys[i][1] for i in idx]
        obst = shapely.union_all(near) if near else None
        if obst is not None:
            shapely.prepare(obst)
        if refine:
            now = sample([(np.array(a), b, c, d) for a, b, c, d in _arcs_of(cr, w)])
            c_now = clearance(now, w, obst)
            if c_now >= CLR_GOOD:
                continue                     # clear of roads at other heights: kept / そのまま
            if sites is not None and sites.distance(shapely.Point(P0)) < 3.0:
                failed.append(int(j.get("id")))   # landmark sites keep their heads: guardrail / 敷地内はそのまま（ガードレール）
                continue
            log("turnloops: junction %s: %.1f m short of the clearance to a road at another height"
                % (j.get("id"), CLR_GOOD - c_now))
        bl = None
        if btree is not None:
            nb = [buildings[i] for i in btree.query(shapely.Point(P0).buffer(45.0))]
            if nb:
                bl = shapely.union_all(nb)
                shapely.prepare(bl)
        arcs, info = plan_loop(ends, w, obst, bl)
        if refine and (arcs is None or info["clr"] < CLR_GOOD):
            failed.append(int(j.get("id")))      # kept as it is: layout.py walls that side / そのまま（擁壁）
            continue
        if arcs is None:
            failed.append(int(j.get("id")))
            continue
        cut(inc, info["back"], True)
        cut(out, info["back"], False)
        L = write_road(cr, arcs, w)
        if refine:
            # the loop from the street's new end height to the other's (streets
            # slope: a level loop left a step after the cut) / 切った端の高さに合わせる
            za, zb = _z_at(inc, float(inc.get("length"))), _z_at(out, 0.0)
            ep = cr.find("elevationProfile")
            for e in list(ep):
                ep.remove(e)
            ET.SubElement(ep, "elevation", s="0", a="%.4f" % za, b="%.8f" % ((zb - za) / L), c="0", d="0")
        info.update(j=int(j.get("id")), road=int(cr.get("id")), length=L, inc=int(inc.get("id")),
                    out=int(out.get("id")))
        report.append(info)
    tree_x.write(xodr_out, encoding="utf-8", xml_declaration=True)
    Rs = [r["R"] for r in report]
    tight = [r for r in report if r["clr"] < CLR_GOOD]
    log("turnloops: %d dead-end loops (R %s), %d shifted sideways, %d set back into the street "
        "(max %.1f m), %d with < %.1f m to another lane: %s"
        % (len(report), {k: Rs.count(k) for k in sorted(set(Rs), reverse=True)},
           sum(1 for r in report if abs(r["g"]) > 0.01), sum(1 for r in report if r["back"] > 0),
           max([r["back"] for r in report] or [0]), len(tight), CLR_GOOD,
           ", ".join("j%d %.1f m" % (r["j"], r["clr"]) for r in tight)))
    if buildings:
        tiers = [r["tier"] for r in report]
        log("turnloops: clear of buildings by >= %.1f m: %d, >= %.1f m: %d, closer: %d (min %.1f m)"
            % (PASSES[0][0], tiers.count(0), PASSES[1][0], tiers.count(1),
               len(tiers) - tiers.count(0) - tiers.count(1), min([r["bld"] for r in report] or [99])))
    if failed:
        log(("turnloops: no clearer loop at junctions %s: kept, guardrail on that side (layout/props)" if refine
             else "turnloops: no loop fits at junctions %s: netconvert's turnaround kept") % failed)
    return report, failed


def landmark_sites(osm, meta):
    """Union of the map's maps.json "landmarks" areas (OSM ways, site_lonlat), or
    None: their heads stay as they are (layout.py keeps the plain head there).
    詳細モデル敷地（転回部はそのまま）"""
    import json
    import os
    from shapely.geometry import Polygon
    import geo
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json")))
    ent = next((v for v in cfg.values() if isinstance(v, dict) and v.get("name") == meta.get("name")), {})
    fr = geo.Frame(meta)
    root = ET.parse(osm).getroot()
    xy = {n.get("id"): fr.xy(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.iter("way")}
    out = []
    for lm in ent.get("landmarks") or []:
        for a in lm.get("areas", []):
            pts = [xy[r] for r in ways.get(str(a), []) if r in xy]
            if len(pts) >= 4:
                out.append(shapely.make_valid(Polygon(pts)))
        if lm.get("site_lonlat"):
            out.append(shapely.make_valid(Polygon([fr.xy(lo, la) for lo, la in lm["site_lonlat"]])))
    return shapely.union_all(out) if out else None


if __name__ == "__main__":
    import json
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    refine = "--refine" in sys.argv
    bl = sites = None
    if refine:
        import glob
        import os
        d = os.path.dirname(os.path.abspath(args[0]))
        ring = glob.glob(os.path.join(d, "*.ring.osm"))
        if ring and os.path.exists(os.path.join(d, "meta.json")):
            import roadnet
            meta = json.load(open(os.path.join(d, "meta.json")))
            bl = roadnet.osm_buildings(ring[0], meta)
            sites = landmark_sites(ring[0], meta)
    rep, bad = fix(args[0], args[1] if len(args) > 1 else args[0], refine=refine, buildings=bl, sites=sites)
    for _ in range(3 if refine else 0):
        # a moved loop can come near another head: again until nothing moves
        # 動かしたループが別の転回部に近づくことがある: 動かなくなるまで繰り返す
        if not rep:
            break
        r2, bad = fix(args[1] if len(args) > 1 else args[0], args[1] if len(args) > 1 else args[0],
                      refine=True, buildings=bl, sites=sites)
        rep = rep + r2 if r2 else rep
        if not r2:
            break
    if refine:
        # the heads layout.py walls on the near side / layout.py が擁壁を付ける転回部
        import os
        mp = os.path.join(os.path.dirname(os.path.abspath(args[0])), "meta.json")
        if os.path.exists(mp):
            meta = json.load(open(mp))
            meta["dead_end_walled"] = bad
            meta.setdefault("dead_end_refined", {}).update(
                {str(r["j"]): {k: round(r[k], 2) for k in ("R", "back", "g", "clr")} for r in rep})
            json.dump(meta, open(mp, "w"), indent=1)
    elif len(args) > 2:
        json.dump({"loops": rep, "failed": bad}, open(args[2], "w"), indent=0)
