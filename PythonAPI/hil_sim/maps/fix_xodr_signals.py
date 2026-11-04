"""Make netconvert's traffic signals work as CARLA traffic lights.

  python fix_xodr_signals.py <in.xodr> <out.xodr>

SUMO netconvert writes OpenDRIVE signals that CARLA turns into broken
traffic lights:
  - zOffset="5" (netconvert's head height): CARLA uses it for the base of
    the pole, so every light floats 5 m up -> 0;
  - one signal per lane, each in the middle of its lane: several poles
    standing in the road -> one signal per approach (same road, s and
    orientation), valid for all its lanes, on the kerb past the outermost
    lane, set back from the junction so it is clear of the turning lanes;
  - controllers that no junction lists: CARLA groups lights by the
    controllers of a junction, so every light had no group (asking a light
    for its group crashed the server, and the traffic manager could not tie
    lights to junctions) -> each controller is added to the junction its
    signals face (the successor junction of a signal near its road's end,
    the predecessor of one near its start).

netconvert の信号を CARLA の信号機として正しく動くよう修正する（高さ・車線ごとの
重複・交差点との対応付け）。
"""

import sys
import xml.etree.ElementTree as ET

KERB_M = 0.6           # pole this far past the outermost lane / 最外車線からの距離
SETBACK_M = 4.0        # and this far back from the junction, out of its turning lanes / 交差点から下げる距離


def lane_widths(road, s, side):
    """Total width of the lanes on ``side`` (-1 right, +1 left) at s. / 片側の車線幅の合計。"""
    secs = road.find("lanes").findall("laneSection")
    sec = [x for x in secs if float(x.get("s")) <= s + 1e-6][-1]
    ds = s - float(sec.get("s"))
    part = sec.find("right" if side < 0 else "left")
    total = 0.0
    if part is None:
        return total
    for lane in part.findall("lane"):
        if lane.get("type") not in ("driving", "bidirectional", "entry", "exit", "onRamp", "offRamp"):
            continue
        ws = [w for w in lane.findall("width") if float(w.get("sOffset")) <= ds + 1e-6] or lane.findall("width")[:1]
        if not ws:
            continue
        w = ws[-1]
        d = ds - float(w.get("sOffset"))
        total += sum(float(w.get(k)) * d ** i for i, k in enumerate("abcd"))
    return total


def main():
    src, dst = sys.argv[1:3]
    tree = ET.parse(src)
    root = tree.getroot()
    roads = {r.get("id"): r for r in root.findall("road")}
    junctions = {j.get("id"): j for j in root.findall("junction")}
    sig_road, merged, removed = {}, 0, 0
    for rid, road in roads.items():
        sigs = road.find("signals")
        if sigs is None:
            continue
        groups = {}
        for s in list(sigs.findall("signal")):
            key = (round(float(s.get("s")), 2), s.get("orientation"))
            groups.setdefault(key, []).append(s)
        for (s_val, orient), group in groups.items():
            keep = group[0]
            lanes = []
            for g in group:
                for v in g.findall("validity"):
                    lanes += [int(v.get("fromLane")), int(v.get("toLane"))]
            for g in group[1:]:                       # one pole per approach / 進入路ごとに 1 本
                sigs.remove(g)
                removed += 1
            for v in keep.findall("validity"):
                keep.remove(v)
            if lanes:
                ET.SubElement(keep, "validity", fromLane=str(min(lanes)), toLane=str(max(lanes)))
            side = -1 if (lanes and max(lanes) < 0) or (not lanes and float(keep.get("t")) < 0) else 1
            length = float(road.get("length"))
            s_val = (max(s_val - SETBACK_M, length / 2) if s_val > length / 2
                     else min(s_val + SETBACK_M, length / 2))
            keep.set("s", "%.4f" % s_val)
            keep.set("t", "%.3f" % (side * (lane_widths(road, s_val, side) + KERB_M)))
            keep.set("zOffset", "0")
            merged += 1
            sig_road[keep.get("id")] = (rid, s_val)
            for g in group[1:]:
                sig_road[g.get("id")] = (rid, s_val)
    # junction <- controller / 交差点に制御器を登録
    added = 0
    removed_ids = set()
    for c in root.findall("controller"):
        js = set()
        for ctl in list(c.findall("control")):
            sid = ctl.get("signalId")
            if sid not in sig_road:
                continue
            rid, s_val = sig_road[sid]
            road = roads[rid]
            link = road.find("link")
            near_end = s_val > float(road.get("length")) / 2
            el = link.find("successor" if near_end else "predecessor") if link is not None else None
            if el is not None and el.get("elementType") == "junction":
                js.add(el.get("elementId"))
        # controls must point at signals that still exist / 残った信号だけを参照
        alive = {s.get("id") for r in roads.values() for s in r.iter("signal")}
        for ctl in list(c.findall("control")):
            if ctl.get("signalId") not in alive:
                c.remove(ctl)
                removed_ids.add(ctl.get("signalId"))
        for jid in js:
            j = junctions.get(jid)
            if j is not None and not any(x.get("id") == c.get("id") for x in j.findall("controller")):
                ET.SubElement(j, "controller", id=c.get("id"), type="0", sequence="0")
                added += 1
    print("signals: %d approaches kept, %d per-lane duplicates removed; %d junction-controller links added"
          % (merged, removed, added))
    refine(root, roads)
    tree.write(dst, encoding="utf-8", xml_declaration=True)


def refine(root, roads):
    """Check each pole with CARLA's own parser: one standing in another road's
    lane goes to the other edge of its road (netconvert makes each direction
    its own road, so the "outer" side can be the opposite carriageway), one
    still in a junction's turning lane is set further back. Skipped without
    the carla module. / CARLA で配置を確認し、他の車線上の柱を移す。"""
    try:
        import carla
    except ImportError:
        print("(carla module not found: pole placement not checked)")
        return
    sig = {s.get("id"): (r, s) for r in roads.values() for s in r.iter("signal")}
    for rnd in range(4):
        m = carla.Map("check", ET.tostring(root, encoding="unicode"))
        moved = 0
        for lm in m.get_all_landmarks():
            if not lm.is_dynamic or lm.id not in sig:
                continue
            wp = m.get_waypoint(lm.transform.location, project_to_road=False,
                                lane_type=carla.LaneType.Driving)
            if wp is None:
                continue
            road, el = sig[lm.id]
            s_val, t = float(el.get("s")), float(el.get("t"))
            length = float(road.get("length"))
            if wp.is_junction:                       # further from the junction / さらに下げる
                s_val = (max(s_val - 3.0, length / 2) if s_val > length / 2
                         else min(s_val + 3.0, length / 2))
                el.set("s", "%.4f" % s_val)
            else:                                    # the other edge of its road / 道路の反対側の端へ
                side = 1 if t < 0 else -1
                el.set("t", "%.3f" % (side * (lane_widths(road, s_val, side) + KERB_M)))
            moved += 1
        print("placement check %d: %d poles moved" % (rnd + 1, moved))
        if not moved:
            return


if __name__ == "__main__":
    main()
