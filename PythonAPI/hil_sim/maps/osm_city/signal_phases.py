"""Signal phases for netconvert's traffic lights (called by signs.py; guide ch. 18).

netconvert writes ONE OpenDRIVE <controller> per signalised junction cluster,
holding every light of it. CARLA makes one UTrafficLightController per xodr
controller and runs a junction's controllers one after another
(TrafficLightManager.cpp, TrafficLightGroup.cpp): with a single controller
every approach shows green at the same time. This splits each such
controller into phase controllers by approach axis: lights whose approach
headings are parallel or opposite (within 35 degrees, modulo 180) share a
phase, as in a two-phase Japanese signal (main road, then the side road).
Within an axis, approaches whose paths through the junction cross (from the
junction lanes in lanes.json) are split further: two approaches share a phase
only if no path of one crosses a path of the other, except the turning
conflicts of OPPOSITE approaches that Japanese signals allow (a right turn
yielding to oncoming traffic; merges at the exit). A diagonal leg on a
"north-south" axis, or a staggered junction, gets its own phase that way.
``conflicts()`` lists any crossing pair left in one phase (tests, checks).
Each junction that listed the old controller lists the phases instead.
Duplicate controllers of the same id (netconvert writes one per junction of a
joined cluster) are merged. Rerunning first merges earlier "_p<n>" phases
back, so it always recomputes from the current rules.

hil_sim/signal_timing.py then gives each phase its green / yellow / red.

信号の現示: 交差点ごとに 1 つのコントローラ（全方向同時に青）を、進入方向の軸ごとの
現示コントローラに分ける（対向する進入路は同じ現示）。
"""

import collections
import json
import math
import re
import xml.etree.ElementTree as ET

AXIS_TOL = 35.0
STUB = 3.0           # roads shorter than this join two junctions of one cluster / 交差点群の連結路
OPPOSITE = 150.0     # approaches this far apart in heading are opposite (skewed legs are not) / 対向とみなす角度


def approach_yaw(lanes_json):
    """{road: heading (deg, map frame) at its end} from lanes.json approaches. / 進入路の向き。"""
    data = json.load(open(lanes_json))
    acc = collections.defaultdict(list)
    for a in data["approaches"]:
        acc[a["road"]].append(math.radians(a["yaw"]))
    out = {}
    for r, v in acc.items():
        out[r] = math.degrees(math.atan2(sum(math.sin(x) for x in v), sum(math.cos(x) for x in v)))
    for l in data["lanes"]:                     # roads with no approach entry: their last point / 予備
        if l["road"] not in out and l["junction"] < 0 and len(l["pts"]) >= 2:
            out[l["road"]] = l["pts"][-1]["yaw"]
    return out


def axis_diff(a, b):
    d = abs((a - b) % 180.0)
    return min(d, 180.0 - d)


class Paths(object):
    """Each approach road's paths through its junction (junction lane centres
    starting at the road's lane ends) and their turn. / 進入路ごとの交差点内の経路。"""

    def __init__(self, lanes_json):
        data = json.load(open(lanes_json)) if isinstance(lanes_json, str) else lanes_json
        self.jl = [l for l in data["lanes"] if l.get("junction", -1) >= 0 and len(l.get("pts", [])) >= 2]
        self.ends = collections.defaultdict(list)
        self.stubs = []                 # (start, road) of roads under STUB m (joined clusters) / 短い連結路
        for l in data["lanes"]:
            if l.get("junction", -1) < 0 and len(l.get("pts", [])) >= 2:
                self.ends[l["road"]].append((l["pts"][-2]["c"], l["pts"][-1]["c"]))
                C = [q["c"] for q in l["pts"]]
                if sum(math.dist(C[i], C[i + 1]) for i in range(len(C) - 1)) < STUB:
                    self.stubs.append((C[0], l["road"]))
        self.cache = {}

    def _follow(self, P, chain, hops):
        """Continue a path over a stub road into the next junction of a joined
        cluster: [(polyline, roads passed)]. / 短い連結路を越えて次の交差点へ。"""
        e = P[-1]
        nxt = [r for c, r in self.stubs if math.dist(c, e) < 0.8 and r not in chain]
        if not nxt or hops >= 3:
            return [(P, chain)]
        out = []
        for r in nxt:
            for a, e2 in self.ends.get(r, []):
                for l in self.jl:
                    if math.dist(l["pts"][0]["c"], e2) <= 0.8:
                        out += self._follow(P + [tuple(q["c"]) for q in l["pts"]], chain + (r,), hops + 1)
        return out or [(P, chain)]

    def feeds(self, ra, rb):
        """ra's straight path runs straight onto stub road rb (the next light of
        the same movement in a two-stage junction). / ra の直進が次の連結路 rb に続く。"""
        for P, turn, h, ch in self.of(ra):
            if len(ch) >= 2 and ch[1] == rb and turn == "straight":
                for Q, t2, h2, c2 in self.of(rb):
                    if abs((h - h2 + 180.0) % 360.0 - 180.0) < 35.0:
                        return True
        return False

    def of(self, road):
        """[(polyline, turn, heading, roads passed)] / [(折れ線, 進行方向, 方位, 通過道路)]"""
        if road in self.cache:
            return self.cache[road]
        out = []
        for a, e in self.ends.get(road, []):
            h0 = math.atan2(e[1] - a[1], e[0] - a[0])
            for l in self.jl:
                c0 = l["pts"][0]["c"]
                if math.hypot(c0[0] - e[0], c0[1] - e[1]) > 0.8:
                    continue
                for P, chain in self._follow([tuple(e)] + [tuple(q["c"]) for q in l["pts"]], (road,), 0):
                    b, c = P[-2], P[-1]
                    d = math.degrees((math.atan2(c[1] - b[1], c[0] - b[0]) - h0 + math.pi) % (2 * math.pi)
                                     - math.pi)
                    # U-turns and loops through a joined cluster are not movements / 転回・周回は除く
                    if abs(d) > 150.0 or any(math.dist(q, P[0]) < 4.0 for q in P[len(P) // 2:]):
                        continue
                    turn = "straight" if abs(d) < 35.0 else ("left" if d > 0 else "right")
                    out.append((P, turn, math.degrees(h0), chain))
        self.cache[road] = out
        return out


def _segx(p, q, a, b):
    cr = lambda o, u, v: (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])
    d1, d2, d3, d4 = cr(a, b, p), cr(a, b, q), cr(p, q, a), cr(p, q, b)
    if (d1 == 0 and d2 == 0) or (d3 == 0 and d4 == 0):
        return False                      # collinear (overlapping lanes) / 同一直線上
    return d1 * d2 <= 0 and d3 * d4 <= 0   # touching at a vertex counts / 頂点で接するのも交差


def _crossing(P, Q):
    """Where polylines P and Q cross (not a merge at their exits), or None. / 交差点（出口での合流は除く）。"""
    for i in range(len(P) - 1):
        for k in range(len(Q) - 1):
            if _segx(P[i], P[i + 1], Q[k], Q[k + 1]):
                if math.dist(P[-1], Q[-1]) < 1.5 and math.dist(P[i], P[-1]) < 6.0:
                    continue
                return P[i]
    return None


def conflict(paths, ra, rb):
    """True when approach roads ra and rb may not be green together: some path
    of one crosses a path of the other, apart from opposite approaches' right
    turns (they yield; keep-left) / 同時に青にできない（対向の右折は許容）。"""
    # two-stage junction: when one light's straight runs on to the next light of the
    # same carriageway (feeds), the paths through that next light are one movement
    # 二段の交差点: 同じ車道の次の信号へ直進で続く場合、その先の経路は同じ流れ
    fab, fba = paths.feeds(ra, rb), paths.feeds(rb, ra)
    for P, ta, ha, ca in paths.of(ra):
        if fab and rb in ca:
            continue
        for Q, tb, hb, cb in paths.of(rb):
            if fba and ra in cb:
                continue
            if _crossing(P, Q) is None:
                continue
            opposite = abs((ha - hb + 180.0) % 360.0 - 180.0) > OPPOSITE
            if opposite and "right" in (ta, tb):
                continue
            return True
    return False


def phases_of(signals, sig_road, yaw, paths=None):
    """Lights grouped by approach axis, then split so no two conflicting
    approaches share a phase: [[signal id, ...], ...]. / 軸ごと・衝突しない組。"""
    groups = []
    for sid in signals:
        y = yaw.get(sig_road.get(sid))
        if y is None:
            continue
        for g in groups:
            if axis_diff(g["axis"], y) < AXIS_TOL:
                g["ids"].append(sid)
                break
        else:
            groups.append({"axis": y % 180.0, "ids": [sid]})
    out = []
    for g in groups:
        if paths is None:
            out.append(g["ids"])
            continue
        # greedy colouring by approach road (lights of one road stay together) / 道路単位で塗り分け
        roads = sorted({sig_road[s] for s in g["ids"]}, key=lambda r: -sum(1 for s in g["ids"] if sig_road[s] == r))
        sub = []
        for r in roads:
            for grp in sub:
                if not any(conflict(paths, r, o) for o in grp):
                    grp.append(r)
                    break
            else:
                sub.append([r])
        for grp in sub:
            out.append([s for s in g["ids"] if sig_road[s] in grp])
    if paths is not None:
        # merge phases across axes when nothing in them conflicts (fewer phases, shorter waits)
        # 衝突の無い現示どうしは軸をまたいでまとめる
        merged = True
        while merged:
            merged = False
            out.sort(key=len)
            for i in range(len(out)):
                for k in range(len(out) - 1, i, -1):
                    ra = {sig_road[x] for x in out[i]}
                    rb = {sig_road[x] for x in out[k]}
                    # only roads whose paths are known (no data: keep the axis split) / 経路不明なら分けたまま
                    if all(paths.of(x) for x in ra | rb) and \
                            not any(conflict(paths, a, b) for a in ra for b in rb):
                        out[k] = out[k] + out[i]
                        del out[i]
                        merged = True
                        break
                if merged:
                    break
    # the phase with the most lights first (usually the main road) / 信号の多い現示から
    out.sort(key=lambda ids: -len(ids))
    return out


def conflicts(root, lanes_json):
    """Crossing approach pairs that share a controller: [(controller, signal a,
    signal b)]. Empty when the phases are right. / 同じ現示内の衝突する組。"""
    paths = Paths(lanes_json)
    sig_road = {s.get("id"): int(road.get("id")) for road in root.findall("road") for s in road.iter("signal")
                if s.get("dynamic") == "yes"}
    bad = []
    for c in root.findall("controller"):
        ids = [x.get("signalId") for x in c.findall("control") if x.get("signalId") in sig_road]
        for i in range(len(ids)):
            for k in range(i + 1, len(ids)):
                ra, rb = sig_road[ids[i]], sig_road[ids[k]]
                if ra != rb and conflict(paths, ra, rb):
                    bad.append((c.get("id"), ids[i], ids[k]))
    return bad


def merge_phases(root):
    """Undo an earlier split: "<id>_p<n>" controllers back into one "<id>". / 以前の分割を戻す。"""
    base = collections.OrderedDict()
    for c in list(root.findall("controller")):
        m = re.match(r"(.+)_p\d+$", c.get("id", ""))
        if not m:
            continue
        b = m.group(1)
        if b not in base:
            nc = ET.Element("controller", id=b)
            nc.tail = c.tail
            root.insert(list(root).index(c), nc)
            base[b] = nc
        for x in c.findall("control"):
            ET.SubElement(base[b], "control", signalId=x.get("signalId"))
        root.remove(c)
    for j in root.findall("junction"):
        put = set()
        for jc in list(j.findall("controller")):
            m = re.match(r"(.+)_p\d+$", jc.get("id", ""))
            if not m:
                continue
            if m.group(1) not in put:
                e = ET.Element("controller", id=m.group(1), type=jc.get("type", "0"), sequence="0")
                e.tail = jc.tail
                j.insert(list(j).index(jc), e)
                put.add(m.group(1))
            j.remove(jc)


def split(root, lanes_json):
    """Split the controllers of ``root`` (an xodr ElementTree root) in place.
    Returns {controller id: [[signal ids of phase 1], ...]} for those split.
    xodr のコントローラを現示ごとに分ける（その場で変更）。"""
    yaw = approach_yaw(lanes_json)
    paths = Paths(lanes_json)
    sig_road = {}
    for road in root.findall("road"):
        for s in road.iter("signal"):
            if s.get("dynamic") == "yes":
                sig_road[s.get("id")] = int(road.get("id"))
    merge_phases(root)
    # lights on stub roads (< STUB m between two junctions of a joined cluster) stop
    # cars inside the junction box (gridlock): they are removed, the cluster is
    # governed by the lights where traffic enters it
    # 交差点群内の短い連結路の信号は交差点内で車を止める（渋滞）: 削除し、流入部の信号で制御
    self_removed = []
    for road in root.findall("road"):
        lk = road.find("link")
        pre = lk.find("predecessor") if lk is not None else None
        suc = lk.find("successor") if lk is not None else None
        if float(road.get("length", 99)) >= STUB or pre is None or suc is None or \
                pre.get("elementType") != "junction" or suc.get("elementType") != "junction":
            continue
        sigs = road.find("signals")
        for sg in list(sigs.findall("signal")) if sigs is not None else []:
            if sg.get("dynamic") == "yes":
                sigs.remove(sg)
                sig_road.pop(sg.get("id"), None)
                self_removed.append((sg.get("id"), int(road.get("id"))))
    gone = {sid for sid, _ in self_removed}
    for c in root.findall("controller"):
        for x in list(c.findall("control")):
            if x.get("signalId") in gone:
                c.remove(x)
    split.removed = self_removed
    seen, done = {}, {}
    for c in list(root.findall("controller")):
        cid = c.get("id")
        if cid in seen:                         # duplicate of a joined cluster / 重複
            root.remove(c)
            continue
        seen[cid] = c
        ids = [x.get("signalId") for x in c.findall("control")]
        ph = phases_of(ids, sig_road, yaw, paths)
        if len(ph) < 2:
            continue
        pos = list(root).index(c)
        root.remove(c)
        for k, grp in enumerate(ph):
            nc = ET.Element("controller", id="%s_p%d" % (cid, k + 1), name="phase %d" % (k + 1),
                            sequence=str(k))
            for sid in grp:
                ET.SubElement(nc, "control", signalId=sid)
            nc.tail = c.tail
            root.insert(pos + k, nc)
        done[cid] = ph
    for j in root.findall("junction"):
        for jc in list(j.findall("controller")):
            ph = done.get(jc.get("id"))
            if ph is None:
                continue
            pos = list(j).index(jc)
            j.remove(jc)
            for k in range(len(ph)):
                e = ET.Element("controller", id="%s_p%d" % (jc.get("id"), k + 1), type=jc.get("type", "0"),
                               sequence=str(k))
                e.tail = jc.tail
                j.insert(pos + k, e)
    return done
