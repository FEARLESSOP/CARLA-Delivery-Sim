"""Traffic-light poles for keep-left road logic (carla-venv).

  python lht_poles.py <in.xodr> <out.xodr>

CARLA spawns its right-hand-traffic light (BP_TLOpenDrive_RHT): the pole
stands on the RIGHT of the lanes it controls and the arm reaches LEFT over
them. It only uses its left-hand model on roads marked rule="LHT", and that
flag also reverses every lane of netconvert's keep-left roads, so it cannot
be used. With the pole on the left kerb (where fix_xodr_signals puts it)
the arm hangs over the pavement, away from the road.

So each light moves to the right of its approach (netconvert keep-left
roads: lanes at t in [-W, 0], the kerb at t = 0, the other direction
beyond t = -W):
  - a median beside the approach: the middle of the median
  - the other direction right beside it: the far pavement, across the
    opposing lanes (the arm reaches back over the road)
  - nothing within 4 m (a one-way road, a wide median): just past the
    right-hand kerb
A new spot that falls on a driving lane is not used.

左側通行の道路情報で、信号柱を進入車線の右側（中央分離帯・対向側の歩道）に移す。
CARLA の右側通行用モデルは腕が柱の左へ伸びるため。
"""

import sys
import xml.etree.ElementTree as ET

import carla

GUTTER, KERB, CLEAR = 0.4, 0.2, 0.7


def main():
    src, dst = sys.argv[1:3]
    text = open(src).read()
    m = carla.Map("lht", text)
    root = ET.fromstring(text)
    moved = {"median": 0, "far pavement": 0, "right kerb": 0, "kept": 0}
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
            L = tr.location

            def lane_at(d):
                p = carla.Location(L.x + r.x * d, L.y + r.y * d, 0.0)
                q = m.get_waypoint(p, project_to_road=False, lane_type=carla.LaneType.Driving)
                return q if q is not None and q.road_id != rid else None

            half = inner.lane_width / 2
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
            # the spot must be off every driving lane / 走行車線上は不可
            d = -t - W + half
            p = carla.Location(L.x + r.x * d, L.y + r.y * d, 0.0)
            if m.get_waypoint(p, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
                moved["kept"] += 1
                continue
            sig.set("t", "%.3f" % t)
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
    if ids:
        print("signal ids shortened: %d" % len(ids))
    ET.ElementTree(root).write(dst, encoding="utf-8", xml_declaration=True)
    print("traffic-light poles to the right of their lanes: %s" % moved)


if __name__ == "__main__":
    main()
