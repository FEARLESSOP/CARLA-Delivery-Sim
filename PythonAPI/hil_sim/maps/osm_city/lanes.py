"""Lane geometry out of the road logic, for the 3-D model (carla-venv).

  carla-venv/bin/python lanes.py <Name.xodr> <out lanes.json>

The model's carriageways, kerbs and markings are built from these, so the
road surface is exactly where CARLA's lanes are (the Oita Station model was
made separately from its road logic, and they disagree by up to metres).

For every lane of every lane section (OpenDRIVE frame: x east, y NORTH):
  road, section, lane id, junction id (-1 outside junctions), kind
  (driving / sidewalk / ...), the centre line, left and right edges
  (sampled every <=1 m), the lane width, and the lane markings on each edge
  (type, colour) per sample.
And for every lane that ends at a junction: the junction it enters, and
which way each of its connections turns (for lane arrows and stop lines).

道路情報から車線の形・区画線・交差点への接続を書き出す（3D モデル用）。
"""

import json
import math
import sys
import xml.etree.ElementTree as ET

import carla

STEP = 1.0


def flipy(loc):
    return [round(loc.x, 3), round(-loc.y, 3)]     # CARLA y south -> north / 南向き -> 北向き


def main():
    src, dst = sys.argv[1:3]
    text = open(src).read()
    m = carla.Map("city", text)
    root = ET.fromstring(text)
    lanes = []
    speed = {}
    names = {}
    for road in root.iter("road"):
        rid = int(road.get("id"))
        names[rid] = road.get("name", "")
        for t in road.iter("type"):
            sp = t.find("speed")
            if sp is not None:
                speed[rid] = float(sp.get("max"))
        secs = list(road.iter("laneSection"))
        starts = [float(x.get("s")) for x in secs]
        ends = starts[1:] + [float(road.get("length"))]
        for si, (sec, s0, s1) in enumerate(zip(secs, starts, ends)):
            if s1 - s0 < 0.02:
                continue
            n = max(1, int(math.ceil((s1 - s0) / STEP)))
            ss = [s0 + 0.005 + (s1 - s0 - 0.01) * i / n for i in range(n + 1)]
            for lane in sec.iter("lane"):
                lid = int(lane.get("id"))
                if lid == 0:
                    continue
                kind = lane.get("type")
                sp_el = lane.find("speed")                 # netconvert: the limit per lane / 車線ごとの制限速度
                lane_speed = float(sp_el.get("max")) if sp_el is not None else speed.get(rid)
                pts = []
                for sv in ss:
                    w = m.get_waypoint_xodr(rid, lid, sv)
                    if w is None or w.lane_id != lid:
                        continue
                    t = w.transform
                    r = t.get_right_vector()
                    hw = w.lane_width / 2
                    c = t.location
                    lm, rm = w.left_lane_marking, w.right_lane_marking
                    pts.append({
                        "s": round(sv, 3), "c": flipy(c),
                        "l": flipy(carla.Location(c.x - r.x * hw, c.y - r.y * hw, 0)),
                        "r": flipy(carla.Location(c.x + r.x * hw, c.y + r.y * hw, 0)),
                        "w": round(w.lane_width, 3), "z": round(c.z, 3),
                        "ml": [str(lm.type), str(lm.color)] if lm else ["NONE", "Standard"],
                        "mr": [str(rm.type), str(rm.color)] if rm else ["NONE", "Standard"],
                        "yaw": round(-t.rotation.yaw, 2)})
                if len(pts) < 2:
                    continue
                w0 = m.get_waypoint_xodr(rid, lid, ss[len(ss) // 2]) or m.get_waypoint_xodr(rid, lid, ss[0])
                lanes.append({"road": rid, "section": si, "lane": lid, "kind": kind,
                              "junction": w0.junction_id if w0 is not None and w0.is_junction else -1,
                              "speed": lane_speed, "name": names[rid], "pts": pts})
    # approaches: driving lanes entering a junction, with the turns they lead to
    # 交差点に入る車線と、その先の曲がり方
    approaches = []
    for a, b in m.get_topology():
        # (start, end) of each lane; an approach is a lane outside a junction
        # whose end leads into one / 交差点の手前で終わる車線
        if a.is_junction or a.lane_type != carla.LaneType.Driving:
            continue
        end = b
        ahead = end.next(1.0)
        if not any(q.is_junction for q in ahead):
            continue
        jid = next(q.junction_id for q in ahead if q.is_junction)
        turns = set()
        exits = {}
        for nxt in ahead:
            if not nxt.is_junction:
                continue
            # follow the connection through the junction / 交差点内の接続を辿る
            q, steps = nxt, 0
            while q.is_junction and steps < 200:
                nn = q.next(1.0)
                if not nn:
                    break
                q = nn[0]
                steps += 1
            dyaw = (q.transform.rotation.yaw - end.transform.rotation.yaw + 540.0) % 360.0 - 180.0
            # CARLA yaw grows clockwise seen from above: + is a right turn / CARLA は時計回りが正
            turn = ("straight" if abs(dyaw) < 30 else "right" if dyaw > 0 and dyaw < 150
                    else "left" if dyaw < 0 and dyaw > -150 else "uturn")
            turns.add(turn)
            # the street this turn leads into, for direction boards / 曲がった先の道路名（案内板用）
            if names.get(q.road_id):
                exits.setdefault(turn, names[q.road_id])
        approaches.append({"road": a.road_id, "lane": a.lane_id, "junction": jid,
                           "end": flipy(end.transform.location), "yaw": round(-end.transform.rotation.yaw, 2),
                           "width": round(end.lane_width, 3), "turns": sorted(turns), "exits": exits})
    lights = []
    for lm in m.get_all_landmarks():
        if lm.is_dynamic:
            lights.append({"id": lm.id, "road": lm.road_id, "pos": flipy(lm.transform.location)})
    junctions = {}
    for w in m.generate_waypoints(2.0):
        if w.is_junction:
            j = w.get_junction()
            bb = j.bounding_box
            junctions[j.id] = {"c": flipy(bb.location), "ext": [round(bb.extent.x, 2), round(bb.extent.y, 2)]}
    json.dump({"lanes": lanes, "approaches": approaches, "lights": lights, "junctions": junctions},
              open(dst, "w"))
    print("lanes %d (driving %d), approaches %d, junctions %d, traffic lights %d"
          % (len(lanes), sum(l["kind"] == "driving" for l in lanes), len(approaches), len(junctions), len(lights)))


if __name__ == "__main__":
    main()
