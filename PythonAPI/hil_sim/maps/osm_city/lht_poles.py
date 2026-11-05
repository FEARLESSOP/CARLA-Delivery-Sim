"""Traffic-light poles for keep-left road logic (carla-venv).

  python lht_poles.py <in.xodr> <out.xodr>

CARLA spawns its right-hand-traffic light model (BP_TLOpenDrive_RHT, on
Japan maps our BP_JpShingo, same frame): the pole stands on the RIGHT of the
lanes it controls and the arm reaches LEFT over them, the head ARM_REACH
(6 m) from the pole. It only uses its left-hand model on roads marked
rule="LHT", and that flag also reverses every lane of netconvert's keep-left
roads, so it cannot be used. With the pole on the left kerb (where
fix_xodr_signals puts it) the arm hangs over the pavement, away from the road.

So each light moves to the right of its approach (netconvert keep-left
roads: lanes at t in [-W, 0], the kerb at t = 0, the other direction
beyond t = -W). The preferred spot:
  - a median beside the approach: the middle of the median
  - the other direction right beside it: the far pavement, across the
    opposing lanes (the arm reaches back over the road)
  - nothing within 4 m (a one-way road, a wide median): just past the
    right-hand kerb
Then the spot must pass two checks, else the nearest t that does is used
(map guide ch. 24):
  - head over the lanes: the head centre (ARM_REACH left of the pole) lies
    at least HEAD_IN inside the approach's lanes, so the 1.05 m head hangs
    over them (a one-lane road with the pole at the kerb put it 1.3 m past
    the left edge);
  - clear of CARLA's own fix: MapBuilder::CheckSignalsOnRoads moves every
    signal that lies within 0.7 lane widths of a driving/shoulder lane
    centre (any road) and gives it THAT lane's heading, which turned heads
    180 deg (other direction) or sideways (a junction road).
Where no t passes both (a narrow strip between two roads, a junction road
just past the kerb), the pole goes to the off-lane spot with the head over
the lanes that is furthest from every lane edge (at least POLE_EDGE), and
the signal gets a <positionInertial> with that spot and the approach's
heading: CARLA then skips its fix for it. Its z is the road's, so run this
again after elevate.py (make_osm_city.sh does).
Only signal t values (and positionInertial) change; when no id needs shortening the file is
rewritten textually, so controllers, phases and everything else stay
byte-identical.

左側通行の道路情報で、信号柱を進入車線の右側（中央分離帯・対向側の歩道）に移す。
CARLA の右側通行用モデルは腕が柱の左へ 6 m 伸びるため、灯器が車線の上に来て、
かつ CARLA の自動移動（車線中心から 0.7 車線幅以内なら移動・向き変更）に
掛からない t を選ぶ。
"""

import math
import re
import sys
import xml.etree.ElementTree as ET

import carla

GUTTER, KERB, CLEAR = 0.4, 0.2, 0.7
ARM_REACH = 6.0       # pole to head centre (jp_shingo.py HEAD_X; CARLA's RHT model is close) / 柱から灯器中心
HEAD_IN = 0.5         # head centre at least this far inside the lanes / 灯器中心は車線内側へ
CARLA_FIX = 0.7       # CheckSignalsOnRoads: moved when nearer than 0.7 lane widths / CARLA の自動移動しきい値
FIX_MARGIN = 0.25
POLE_EDGE = 0.25      # inertial spot: at least this far off every lane edge / 車線端からの余裕
SEARCH = 12.0         # how far right of the approach a pole may go / 探索範囲
DRIVE_SHOULDER = carla.LaneType.Driving | carla.LaneType.Shoulder


def pole_clear(m, loc, fwd):
    """True when CARLA keeps the signal where it is (and keeps its heading)."""
    p = carla.Location(loc.x + fwd.x * 0.25, loc.y + fwd.y * 0.25, loc.z)   # TL forward shift / 前方 0.25 m
    q = m.get_waypoint(p, project_to_road=True, lane_type=DRIVE_SHOULDER)
    if q is None:
        return True
    return q.transform.location.distance(p) >= CARLA_FIX * q.lane_width + FIX_MARGIN


def main():
    src, dst = sys.argv[1:3]
    text = open(src).read()
    m = carla.Map("lht", text)
    root = ET.fromstring(text)
    moved = {"median": 0, "far pavement": 0, "right kerb": 0, "kept": 0}
    refit, stuck, pinned = [], [], []
    new_t, inertial = {}, {}
    for road in root.findall("road"):
        rid = int(road.get("id"))
        for sig in road.iter("signal"):
            if sig.get("dynamic") != "yes":
                continue
            s = float(sig.get("s"))
            lanes = sorted({int(l.get("id")) for l in road.iter("lane") if int(l.get("id")) < 0})
            if not lanes:
                continue
            ws = [m.get_waypoint_xodr(rid, lid, s) for lid in lanes]
            ws = [w for w in ws if w is not None]
            if not ws:
                moved["kept"] += 1
                continue
            W = sum(w.lane_width for w in ws)
            inner = min(ws, key=lambda w: w.lane_id)         # centre-side lane / 中央側の車線
            tr = inner.transform
            r = tr.get_right_vector()
            f = tr.get_forward_vector()
            L = tr.location
            half = inner.lane_width / 2

            def at(d):
                return carla.Location(L.x + r.x * d, L.y + r.y * d, L.z)

            def lane_at(d):
                q = m.get_waypoint(at(d), project_to_road=False, lane_type=carla.LaneType.Driving)
                return q if q is not None and q.road_id != rid else None

            gap = None
            for k in range(1, 41):
                if lane_at(half + 0.1 * k) is not None:
                    gap = 0.1 * k
                    break
            if gap is not None and gap >= 1.0:
                t, kind = -(W + gap / 2), "median"
            elif gap is not None:
                # across the other direction: walk to its far edge / 対向車線の外側まで
                far = gap
                while far < 30 and lane_at(half + far + 0.1) is not None:
                    far += 0.1
                t, kind = -(W + far + GUTTER + KERB + CLEAR), "far pavement"
            else:
                t, kind = -(W + GUTTER + KERB + CLEAR), "right kerb"

            # d: from the inner lane centre to the pole / 中央側車線の中心から柱まで
            def ok(tt, head_in):
                head = -tt - ARM_REACH                        # head centre, metres right of t = 0
                if not (head_in <= head <= W - head_in):
                    return False
                d = -tt - W + half
                if m.get_waypoint(at(d), project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
                    return False                              # pole on a driving lane / 走行車線上は不可
                return pole_clear(m, at(d), f)

            if not ok(t, HEAD_IN):
                cands = [-(W + 0.1 * k) for k in range(1, int(SEARCH * 10) + 1)]
                best = None
                for head_in in (HEAD_IN, 0.0):                # then "just at the edge" / 次は車線端まで
                    good = [c for c in cands if ok(c, head_in)]
                    if good:
                        best = min(good, key=lambda c: abs(c - t))
                        break
                if best is None:
                    # pin it with positionInertial / 慣性座標で固定
                    best, edge = None, POLE_EDGE
                    for head_in in (HEAD_IN, 0.0):
                        for c in cands:
                            head = -c - ARM_REACH
                            if not (head_in <= head <= W - head_in):
                                continue
                            p = at(-c - W + half)
                            if m.get_waypoint(p, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
                                continue
                            q = m.get_waypoint(p, project_to_road=True, lane_type=carla.LaneType.Driving)
                            e = q.transform.location.distance(p) - q.lane_width / 2
                            if e >= edge:
                                best, edge = c, e
                        if best is not None:
                            break
                    if best is None:
                        # as before: the preferred spot if it is off the lanes
                        # 従来どおり: 走行車線外なら優先位置
                        stuck.append(sig.get("id"))
                        if m.get_waypoint(at(-t - W + half), project_to_road=False,
                                          lane_type=carla.LaneType.Driving) is not None:
                            moved["kept"] += 1
                            continue
                        sig.set("t", "%.3f" % t)
                        new_t[sig.get("id")] = sig.get("t")
                        moved[kind] += 1
                        continue
                    p = at(-best - W + half)
                    inertial[sig.get("id")] = (p.x, -p.y, p.z, -math.radians(tr.rotation.yaw))
                    pinned.append("%s t=%.2f (%.2f m off the lanes)" % (sig.get("id"), best, edge))
                refit.append("%s %.2f->%.2f" % (sig.get("id"), t, best))
                t = best
            sig.set("t", "%.3f" % t)
            new_t[sig.get("id")] = sig.get("t")
            moved[kind] += 1
    # CARLA cuts signal ids at 32 characters (netconvert's cluster ids are
    # longer), so the API cannot match those lights to their lanes: rename
    # every signal with a short id, and every reference to it
    # CARLA は信号 ID を 32 文字で切るため短い ID に付け替える
    ids = {}
    for sig in root.iter("signal"):
        old = sig.get("id")
        if len(old) > 24:
            ids[old] = "tl%d" % (len(ids) + 1)
            sig.set("id", ids[old])
    for el in list(root.iter("control")) + list(root.iter("signalReference")):
        for key in ("signalId", "id"):
            if el.get(key) in ids:
                el.set(key, ids[el.get(key)])
    def pos_xml(sid):
        x, y, z, h = inertial[sid]
        return '<positionInertial x="%.4f" y="%.4f" z="%.4f" hdg="%.6f" pitch="0" roll="0"/>' % (x, y, z, h)

    if ids:
        print("signal ids shortened: %d" % len(ids))
        for sig in root.iter("signal"):
            for old in sig.findall("positionInertial"):
                sig.remove(old)
            sid = sig.get("id")
            back = {v: k for k, v in ids.items()}.get(sid, sid)
            if back in inertial:
                sig.insert(0, ET.fromstring(pos_xml(back)))
        ET.ElementTree(root).write(dst, encoding="utf-8", xml_declaration=True)
    else:
        # only the t values change; the rest of the file stays as it was
        # t 値だけを書き換え、他はそのまま
        def sub(mo):
            tag = mo.group(0)
            sid = re.search(r'\bid="([^"]*)"', tag)
            if sid is None or sid.group(1) not in new_t:
                return tag
            return re.sub(r'\bt="[^"]*"', 't="%s"' % new_t[sid.group(1)], tag, count=1)
        text = re.sub(r'<signal\s[^>]*dynamic="yes"[^>]*>', sub, text)
        # positionInertial: drop old ones, add the new ones right after the tag
        # 古い positionInertial を消し、新しいものを入れる
        text = re.sub(r'\n[ \t]*<positionInertial\b[^>]*/>', '', text)

        def pin(mo):
            sid = re.search(r'\bid="([^"]*)"', mo.group(0)).group(1)
            if sid not in inertial:
                return mo.group(0)
            ind = mo.group(1)
            return mo.group(0) + "\n" + ind + "    " + pos_xml(sid)
        text = re.sub(r'\n([ \t]*)<signal\s[^>]*dynamic="yes"[^>]*[^/]>', pin, text)
        open(dst, "w").write(text)
    print("traffic-light poles to the right of their lanes: %s" % moved)
    if refit:
        print("moved so the head hangs over the lanes / clear of CARLA's fix: %s" % ", ".join(refit))
    if pinned:
        print("pinned with positionInertial: %s" % ", ".join(pinned))
    if stuck:
        print("WARNING no spot with the head over the lanes: %s" % ", ".join(stuck))


if __name__ == "__main__":
    main()
