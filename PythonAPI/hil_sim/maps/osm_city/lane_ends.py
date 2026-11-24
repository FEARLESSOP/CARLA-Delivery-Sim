"""Lanes that end at a junction without a connection get one (carla-venv).

  carla-venv/bin/python lane_ends.py <in.xodr> <out.xodr>

netconvert gives a dropped lane (4 lanes into 3, an exit lane whose slip road
it did not connect, a wide carriageway ending at a narrow street) no laneLink
in the junction: the lane runs into nothing, the Traffic Manager's cars stop
there, and the route graph has a sink (PasirRisTampines: ~160 of them).

For every driving lane that reaches a junction with no connection, the
connection of the nearest connected lane of the same road is followed to its
target lane, and a new connecting road (one lane, a cubic from the dead lane's
end to that lane's start, width blended) merges the dead lane into it. The new
road copies the neighbour's connecting road (lane type, speed, marks), joins
the junction's connections and links like netconvert's own. Lanes whose road
has no connection at all in the junction are reported, not changed.
roadnet.py runs it when maps.json "roadnet" has connect_lane_ends (before
grade_sep.py, which gives the new roads the junction's height).

交差点で接続の無い車線（車線減少・接続の無い出口車線）に、隣の車線の接続先へ
合流する接続道路を加える。
"""

import copy
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def _end_waypoint(m, rid, lid, L, at_end):
    s = max(0.0, L - 0.01) if at_end else 0.01
    return m.get_waypoint_xodr(int(rid), int(lid), float(min(max(s, 0.0), L)))


def _pose(w):
    loc = w.transform.location
    return np.array([loc.x, -loc.y]), math.radians(-w.transform.rotation.yaw), w.lane_width


def _cubic(P0, h0, P1, h1):
    """paramPoly3 (normalized) from P0 heading h0 to P1 heading h1, in P0's frame. / 3 次曲線"""
    d = np.linalg.norm(P1 - P0)
    k = d                                           # tangent length / 接線の長さ
    c, s = math.cos(h0), math.sin(h0)
    R = np.array([[c, s], [-s, c]])                 # world -> local / 世界 -> 局所
    q1 = R @ (P1 - P0)
    t0 = np.array([k, 0.0])
    t1 = R @ (k * np.array([math.cos(h1), math.sin(h1)]))
    # Hermite: p(t) = a + b t + c t^2 + d t^3 / エルミート
    a = np.zeros(2)
    b = t0
    cc = 3 * q1 - 2 * t0 - t1
    dd = -2 * q1 + t0 + t1
    ts = np.linspace(0, 1, 101)
    pts = a[None] + b[None] * ts[:, None] + cc[None] * ts[:, None] ** 2 + dd[None] * ts[:, None] ** 3
    length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
    return (b, cc, dd), length


STUB_MAX = 150.0         # a one-way road shorter than this that leads nowhere is removed / 行き止まりの一方通行の最大長


def prune(root, roads, juncs):
    """One-way roads that lead nowhere (no successor or no connection onward, no turnaround: a short exit
    whose continuation is a car-park aisle or private, a carriageway cut at the
    map's edge) and are shorter than STUB_MAX are removed with the connecting
    roads into them, repeatedly. Returns the removed road ids.
    行き先の無い短い一方通行路（と、そこへの接続道路）を取り除く。"""
    out = []
    while True:
        dead = []
        for rid, r in roads.items():
            if r.get("junction", "-1") != "-1" or float(r.get("length")) >= STUB_MAX:
                continue
            ids = [int(ln.get("id")) for ln in r.iter("lane") if ln.get("type") == "driving"]
            if not ids or any(i > 0 for i in ids):
                continue
            sl = r.find("link/successor")
            if sl is None:
                dead.append(rid)
            elif sl.get("elementType") == "junction" and sl.get("elementId") in juncs and \
                    not any(c.get("incomingRoad") == rid for c in juncs[sl.get("elementId")].findall("connection")):
                dead.append(rid)                        # its junction leads it nowhere / 交差点で行き先が無い
        if not dead:
            return out
        for rid in dead:
            r = roads.pop(rid)
            root.remove(r)
            out.append(rid)
            for J in juncs.values():
                for c in list(J.findall("connection")):
                    C = roads.get(c.get("connectingRoad"))
                    sl = C.find("link/successor") if C is not None else None
                    if c.get("incomingRoad") == rid or (sl is not None and sl.get("elementId") == rid):
                        J.remove(c)
                        if C is not None and c.get("incomingRoad") != rid and C.get("id") in roads:
                            roads.pop(C.get("id"))
                            root.remove(C)


STUB_LIGHT = 2.0         # a traffic light on a road shorter than this moves upstream / これより短い道路の信号は手前へ


def lights_off_stubs(root, roads, juncs, log=print):
    """Traffic lights that netconvert / fix_xodr_signals left on a stub road (0.2-2 m
    between two junctions: a lane-count node just before the signalised junction) move
    to the end of the road that feeds the stub (the straightest connection into it),
    2 m before its end; ids and controllers stay. Returns the number moved.
    交差点間の短い道路（0.2-2 m）にある信号を、その手前の道路の終端へ移す。"""
    import math as _m
    moved = 0
    for rid, r in list(roads.items()):
        if r.get("junction", "-1") != "-1" or float(r.get("length")) >= STUB_LIGHT:
            continue
        sigs = r.findall("signals/signal")
        if not sigs:
            continue
        pl = r.find("link/predecessor")
        if pl is None or pl.get("elementType") != "junction" or pl.get("elementId") not in juncs:
            continue
        best = None
        h_stub = float(r.find("planView/geometry").get("hdg"))
        for c in juncs[pl.get("elementId")].findall("connection"):
            C = roads.get(c.get("connectingRoad"))
            sl = C.find("link/successor") if C is not None else None
            if sl is None or sl.get("elementId") != rid:
                continue
            R = roads.get(c.get("incomingRoad"))
            if R is None or R.get("junction", "-1") != "-1" or float(R.get("length")) < STUB_LIGHT + 2.0:
                continue
            g = R.findall("planView/geometry")[-1]
            turn = abs((float(g.get("hdg")) - h_stub + _m.pi) % (2 * _m.pi) - _m.pi)
            if best is None or turn < best[0]:
                best = (turn, R)
        if best is None:
            continue
        R = best[1]
        L = float(R.get("length"))
        sg_r = R.find("signals")
        if sg_r is None:
            sg_r = ET.SubElement(R, "signals")
        for sg in sigs:
            r.find("signals").remove(sg)
            sg.set("s", "%.4f" % max(0.0, L - 2.0))
            sg_r.append(sg)
            moved += 1
    log("lights off stub roads: %d moved to the road feeding the stub" % moved)
    return moved


def fix(src, dst, log=print):
    import carla
    text = open(src).read()
    m = carla.Map("lane_ends", text)
    tree = ET.ElementTree(ET.fromstring(text))
    root = tree.getroot()
    roads = {r.get("id"): r for r in root.findall("road")}
    juncs = {j.get("id"): j for j in root.findall("junction")}
    next_id = max(int(k) for k in roads) + 1
    added, skipped = [], []
    pruned = prune(root, roads, juncs)
    lights_off_stubs(root, roads, juncs, log)
    for rid, r in list(roads.items()):
        if r.get("junction", "-1") != "-1":
            continue
        L = float(r.get("length"))
        lk = r.find("link")
        secs = r.findall("lanes/laneSection")
        for tag, sec, at_end in (("successor", secs[-1], True), ("predecessor", secs[0], False)):
            e = lk.find(tag) if lk is not None else None
            if e is None or e.get("elementType") != "junction" or e.get("elementId") not in juncs:
                continue
            J = juncs[e.get("elementId")]
            # lanes that drive INTO the junction at this end: negative lanes at the end, positive at the start
            # この端で交差点に入る車線: 終端は負、始端は正
            ids = [int(ln.get("id")) for ln in sec.iter("lane")
                   if ln.get("type") == "driving" and int(ln.get("id")) != 0 and (int(ln.get("id")) < 0) == at_end]
            if not ids:
                continue
            conns = [c for c in J.findall("connection") if c.get("incomingRoad") == rid]
            linked = {}
            for c in conns:
                for ll in c.findall("laneLink"):
                    linked.setdefault(int(ll.get("from")), []).append((c, int(ll.get("to"))))
            if not linked:
                skipped.append((rid, J.get("id"), "no connection at all"))
                continue
            for lid in ids:
                if lid in linked:
                    continue
                nb = min(linked, key=lambda q: (abs(q - lid), -abs(q)))
                w0 = _end_waypoint(m, rid, lid, L, at_end)
                if w0 is None:
                    skipped.append((rid, J.get("id"), "lane %d has no waypoint" % lid))
                    continue
                P0, h0, wd0 = _pose(w0)
                # the neighbour's connection that keeps straightest / 隣の接続のうち最も直進するもの
                best = None
                for c, to in linked[nb]:
                    C = roads.get(c.get("connectingRoad"))
                    if C is None:
                        continue
                    cl = C.find("link/successor")
                    if cl is None or cl.get("elementType") != "road" or cl.get("elementId") not in roads:
                        continue
                    T = roads[cl.get("elementId")]
                    tlane = None
                    for ln in C.iter("lane"):
                        if int(ln.get("id")) == to:
                            sl = ln.find("link/successor")
                            tlane = int(sl.get("id")) if sl is not None else None
                    if tlane is None:
                        continue
                    t_start = cl.get("contactPoint") != "end"
                    w1 = _end_waypoint(m, T.get("id"), tlane, float(T.get("length")), not t_start)
                    if w1 is None:
                        continue
                    P1, h1, wd1 = _pose(w1)
                    turn = abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
                    if best is None or turn < best[0]:
                        best = (turn, c, to, C, T, tlane, t_start, P1, h1, wd1)
                if best is None:
                    skipped.append((rid, J.get("id"), "lane %d: neighbour %d leads nowhere" % (lid, nb)))
                    continue
                turn, c, to, C, T, tlane, t_start, P1, h1, wd1 = best
                # reference line = the lane's left edge (lane -1 lies right of it) / 基準線は車線の左端
                n0 = np.array([-math.sin(h0), math.cos(h0)])
                n1 = np.array([-math.sin(h1), math.cos(h1)])
                R0, R1 = P0 + n0 * wd0 / 2, P1 + n1 * wd1 / 2
                (b, cc, dd), length = _cubic(R0, h0, R1, h1)
                if length < 0.5 or np.linalg.norm(P1 - P0) > 40.0:
                    skipped.append((rid, J.get("id"), "lane %d: target %.1f m away" % (lid, np.linalg.norm(P1 - P0))))
                    continue
                N = copy.deepcopy(C)
                nid = str(next_id)
                next_id += 1
                N.set("id", nid)
                N.set("name", ":lane_end_%s_%d" % (rid, -lid if lid < 0 else lid))
                N.set("length", "%.8f" % length)
                pl = N.find("link/predecessor")
                pl.set("elementId", rid)
                pl.set("contactPoint", "end" if at_end else "start")
                pv = N.find("planView")
                for g in list(pv):
                    pv.remove(g)
                g = ET.SubElement(pv, "geometry", s="0.00000000", x="%.8f" % R0[0], y="%.8f" % R0[1],
                                  hdg="%.8f" % h0, length="%.8f" % length)
                ET.SubElement(g, "paramPoly3", aU="0", bU="%.8f" % b[0], cU="%.8f" % cc[0], dU="%.8f" % dd[0],
                              aV="0", bV="%.8f" % b[1], cV="%.8f" % cc[1], dV="%.8f" % dd[1], pRange="normalized")
                for ls in N.findall("lanes/laneSection"):
                    for side in list(ls):
                        for ln in list(side):
                            lnid = int(ln.get("id"))
                            if lnid == 0:
                                continue
                            if lnid != to:
                                side.remove(ln)
                                continue
                            ln.set("id", "-1")
                            lkk = ln.find("link")
                            for el in list(lkk):
                                lkk.remove(el)
                            ET.SubElement(lkk, "predecessor", id=str(lid))
                            ET.SubElement(lkk, "successor", id=str(tlane))
                            for wdt in ln.findall("width"):
                                ln.remove(wdt)
                            wdt = ET.Element("width", sOffset="0", a="%.2f" % wd0, b="%.6f" % ((wd1 - wd0) / length),
                                             c="0.00", d="0.00")
                            ln.insert(list(ln).index(lkk) + 1, wdt)
                for tagn in ("signals", "objects"):
                    sub = N.find(tagn)
                    if sub is not None:
                        for el in list(sub):
                            sub.remove(el)
                root.insert(list(root).index(C) + 1, N)
                roads[nid] = N
                cid = max([int(x.get("id")) for x in J.findall("connection")] + [0]) + 1
                cn = ET.SubElement(J, "connection", id=str(cid), incomingRoad=rid, connectingRoad=nid,
                                   contactPoint="start")
                ET.SubElement(cn, "laneLink", **{"from": str(lid), "to": "-1"})
                added.append((rid, lid, J.get("id"), nid, round(math.degrees(turn), 1),
                              round(float(np.linalg.norm(P1 - P0)), 1)))
    # signals on removed roads: their controller entries go too (a control naming a
    # missing signal breaks CARLA's light groups) / 除いた道路の信号を制御器からも外す
    alive = {sg.get("id") for r in roads.values() for sg in r.findall("signals/signal")}
    for ctl in list(root.findall("controller")):
        for cc in list(ctl.findall("control")):
            if cc.get("signalId") not in alive:
                ctl.remove(cc)
        if not ctl.findall("control"):
            root.remove(ctl)
            for J in juncs.values():
                for jc in list(J.findall("controller")):
                    if jc.get("id") == ctl.get("id"):
                        J.remove(jc)
    for r in roads.values():                     # signalReferences to removed signals / 参照も
        for sr in list(r.findall("signals/signalReference")):
            if sr.get("id") not in alive:
                r.find("signals").remove(sr)
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    log("lane ends: %d one-way stubs that led nowhere removed (and their connecting roads); "
        "%d dead lanes merged into their neighbour's connection by a new connecting road; %d left (%s)"
        % (len(pruned), len(added), len(skipped), skipped[:6]))
    return added, skipped


if __name__ == "__main__":
    fix(sys.argv[1], sys.argv[2])
