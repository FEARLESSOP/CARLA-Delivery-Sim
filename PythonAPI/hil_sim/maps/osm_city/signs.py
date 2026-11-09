"""Road signs into the road logic, for CARLA to spawn its own sign models.

  python signs.py <out_dir>          (after build.py; edits <Name>.xodr)

CARLA makes its own sign props from OpenDRIVE signals of three types: give
way (205), stop (206) and speed limit (274, 30-120 km/h). Singapore
practice:
  speed limit  where the limit is not the default 50 km/h (the 30 km/h
               estate roads, 40, 60, 70 km/h roads), just after the junction
               a road leaves, on the kerb side
  give way     at the minor approaches that have the give-way lines
               (markings.py), on the kerb side before the line
  stop         Japanese practice (markings_jp.py): at the approaches painted
               with a stop line and 止まれ (crossings.pkl "stop")
netconvert's keep-left roads have only negative lanes, with the reference
line along the kerb, so t > 0 is beyond the kerb.

maps.json "jartic_d" (Japan, police data, guide ch. 18): signal controllers
split into phases (signal_phases.py; "phases": false keeps them); roads along JARTIC
最高速度 lines get that lane speed (crossings.pkl "speed_set"), and stops are
named "Stencil_STOP": CARLA then spawns no octagonal STOP model for them
(TrafficLightManager skips that name) while the traffic manager still stops
at the 206 landmark; jp_signs.py stands the Japanese 止まれ plate there.

道路情報に標識を追加する（CARLA が自前の標識モデルを生成する）。
"""

import os
import pickle
import sys
import xml.etree.ElementTree as ET

KERB_T = 1.6           # 0.4 m gutter + 0.2 m kerb + 1 m verge / 路肩＋縁石＋植樹帯
SPEEDS = {30, 40, 60, 70, 80, 90, 100}
HERE = os.path.dirname(os.path.abspath(__file__))
# the painted stop line at a signal (markings_jp.setback): 2 m before a crossing band the
# approach runs through within 10 m of its end, else 1 m before the junction edge; 0.45 m wide
# 信号の停止線（markings_jp と同じ規則）
STOP_BACK, STOP_PLAIN, STOP_NEAR_CROSS, STOP_W = 2.0, 1.0, 10.0, 0.45
STOP_BEHIND = 0.5      # the light's stop waypoint this far behind the line's back edge / 停止線の手前
POLE_PAST = 1.5        # the pole at least this far past the line's front edge / 柱は停止線の先


def stop_setback(lanes, crossings):
    """Distance of the stop line's front edge from the approach's lane end (as
    markings_jp.setback): bands the lanes, extended 3 m into the junction, really
    cross. / 停止線の前端と車線端の距離"""
    import numpy as np
    from shapely.geometry import LineString, Point
    best = STOP_PLAIN
    for cr in crossings:
        band = cr.get("band")
        if band is None:
            continue
        for l in lanes:
            P = np.array([q["c"] for q in l["pts"]])
            if len(P) < 3:
                continue
            f = P[-1] - P[-3]
            f = f / (np.linalg.norm(f) or 1.0)
            ext = LineString(list(map(tuple, P)) + [tuple(P[-1] + f * 3.0)])
            hit = ext.intersection(band)
            if hit.is_empty:
                continue
            s_min = min(ext.project(Point(q)) for g in getattr(hit, "geoms", [hit]) for q in g.coords)
            gap = (ext.length - 3.0) - s_min
            if gap > STOP_NEAR_CROSS:
                continue
            best = max(best, gap + STOP_BACK)
    return best


def light_stops(root, crossings, lanes_json):
    """Each traffic light (netconvert put it 4 m before the junction: CARLA's stop
    waypoint landed ~2.5 m past the painted line, on the crossing) moves along its
    road so the stop waypoint is STOP_BEHIND m behind the painted stop line. Keep-left
    netconvert roads drive towards +s, the junction at s = length. Returns the number
    moved. / 信号の s を停止線の手前へ（CARLA の停止位置が横断歩道上だった）。"""
    import json
    data = json.load(open(lanes_json))
    by_road = {}
    for l in data["lanes"]:
        if l["kind"] == "driving" and l["junction"] < 0:
            by_road.setdefault(l["road"], []).append(l)
    moved, log = 0, []
    for road in root.findall("road"):
        if road.get("junction", "-1") != "-1":
            continue
        sigs = [s for s in road.iter("signal") if s.get("dynamic") == "yes"]
        if not sigs:
            continue
        L = float(road.get("length"))
        lanes = by_road.get(int(road.get("id")), [])
        if not lanes:
            continue
        last = max(l["section"] for l in lanes)
        lanes = [l for l in lanes if l["section"] == last]
        back = stop_setback(lanes, crossings)
        s_new = max(0.5, L - back - STOP_W - STOP_BEHIND)
        for s in sigs:
            if float(s.get("s")) < L / 2:
                continue                       # faces the start of its road (not on keep-left netconvert roads) / 始点向き
            log.append((road.get("id"), round(float(s.get("s")), 2), round(s_new, 2), round(back, 2)))
            # the pole stays where netconvert stood it (the junction corner, past
            # the line and the crossing): lht_poles pins it there / 柱は元の位置（角）に固定
            # ...but never before the line: on an approach without a crossing the
            # line is 1 m from the junction edge and netconvert's pole 4 m, so it
            # goes POLE_PAST m past the line's front edge (towards the corner)
            # 停止線より手前にはしない（横断歩道の無い進入路は柱を停止線の先へ）
            pole_at = min(L - 0.1, max(float(s.get("s")), L - back + POLE_PAST))
            if not any(u.get("code") == "pole_s" for u in s.findall("userData")):
                ET.SubElement(s, "userData", code="pole_s", value="%.4f" % pole_at)
            s.set("s", "%.4f" % s_new)
            moved += 1
    print("lights at the painted stop line: %d moved (road, old s, new s, line from the junction): %s"
          % (moved, log[:40]))
    return moved


def main():
    out = sys.argv[1]
    import json
    meta = json.load(open(os.path.join(out, "meta.json")))
    xodr = os.path.join(out, meta["name"] + ".xodr")
    extra = pickle.load(open(os.path.join(out, "crossings.pkl"), "rb"))
    giveway = {r for r, j in extra.get("giveway", [])}
    # (road, junction, stop line distance from the road's end) / 停止線の位置
    stops = {q[0]: (q[2] if len(q) > 2 else 2.0) for q in extra.get("stop", [])}
    # Japanese style: markings_jp decides the speed signs (only where the limit
    # changes) / 日本式: 速度標識は制限が変わる所のみ（markings_jp が決める）
    speed_at = dict(extra["speed"]) if "speed" in extra else None
    speed_set = extra.get("speed_set") or {}       # JARTIC lane speeds / JARTIC の制限速度
    stop_name = "Stencil_STOP" if "jartic_log" in extra else "stop"
    n_set = 0
    tree = ET.parse(xodr)
    root = tree.getroot()
    lefthand = meta.get("lefthand", False)
    t_kerb = KERB_T if lefthand else -KERB_T
    n_speed = n_give = n_stop = n_nospot = 0
    spots = None               # speed_spot.Spots, built on the first speed post / 最初の速度標識で作る
    sid = 900000
    for road in root.findall("road"):
        if road.get("junction", "-1") != "-1":
            continue
        rid = int(road.get("id"))
        length = float(road.get("length"))
        sigs = road.find("signals")
        if sigs is None:
            sigs = ET.SubElement(road, "signals")
        for s in list(sigs.findall("signal")):
            if s.get("type") in ("205", "206", "274"):
                sigs.remove(s)                     # rerun: replace ours / 再実行時は置き換え
        link = road.find("link")
        pred = link.find("predecessor") if link is not None else None
        after_junction = pred is not None and pred.get("elementType") == "junction"
        if rid in speed_set:
            for sp in road.iter("speed"):
                if sp.get("max") is not None and abs(float(sp.get("max")) * 3.6 - speed_set[rid]) > 0.5:
                    sp.set("max", "%.2f" % (speed_set[rid] / 3.6))
                    n_set += 1
        lane = road.find("lanes/laneSection/right/lane")
        sp = lane.find("speed") if lane is not None else None
        kmh = round(float(sp.get("max")) * 3.6) if sp is not None else None
        if speed_at is not None:
            post = rid in speed_at
            kmh = speed_at.get(rid, kmh)
        else:
            post = after_junction and kmh in SPEEDS and length > 40.0
        t_speed = t_kerb
        if post:
            # the free kerb: never in another road's lane (speed_spot.py, guide ch. 25)
            # 空いている側の歩道（他の道路の車線に置かない）
            if spots is None:
                import speed_spot
                spots = speed_spot.Spots(out, meta)
            t_speed, side, why = spots.choose(rid, 8.0, KERB_T)
            if t_speed is None:
                post = False
                n_nospot += 1
                print("signs: speed %d km/h on road %d left out, no free kerb (%s)" % (kmh, rid, why))
            elif side != "reference kerb":
                print("signs: speed %d km/h on road %d at t=%.2f (%s; %s)" % (kmh, rid, t_speed, side, why))
        if post:
            sid += 1
            ET.SubElement(sigs, "signal", s="8.0", t="%.2f" % t_speed, id=str(sid), name="speed_%d" % kmh,
                          dynamic="no", orientation="+", zOffset="0", country="OpenDRIVE", type="274",
                          subtype=str(kmh), value=str(kmh), unit="km/h", height="2.2", width="0.6")
            n_speed += 1
        if rid in giveway and length > 12.0:
            sid += 1
            ET.SubElement(sigs, "signal", s="%.2f" % max(1.0, length - 4.0), t="%.2f" % t_kerb, id=str(sid),
                          name="give_way", dynamic="no", orientation="+", zOffset="0", country="OpenDRIVE",
                          type="205", subtype="-1", value="-1", height="2.2", width="0.8")
            n_give += 1
        if rid in stops:          # markings_jp only paints stops on roads > 12 m: one sign per stop / 1 対 1
            sid += 1
            ET.SubElement(sigs, "signal", s="%.2f" % max(0.5, length - stops[rid] - 1.5), t="%.2f" % t_kerb,
                          id=str(sid), name=stop_name, dynamic="no", orientation="+", zOffset="0", country="OpenDRIVE",
                          type="206", subtype="-1", value="-1", height="2.2", width="0.8")
            n_stop += 1
        if not len(sigs):
            road.remove(sigs)
    import jartic
    jc = jartic.config(meta)
    if jc and jc.get("phases", True):
        # one controller per junction makes every approach green at once: phases by axis
        # 交差点に 1 つのコントローラだと全方向同時に青: 軸ごとの現示に分ける
        import signal_phases
        done = signal_phases.split(root, os.path.join(out, "lanes.json"))
        if signal_phases.split.removed:
            print("signal phases: lights on stub roads removed (they stopped cars inside the junction): %s"
                  % ", ".join("%s road %d" % q for q in signal_phases.split.removed))
        print("signal phases: %d junction controllers split: %s" % (
            len(done), ", ".join("%s %s" % (k[-12:], "/".join(str(len(g)) for g in v)) for k, v in done.items())))
    moved = light_stops(root, extra.get("crossings", []), os.path.join(out, "lanes.json"))
    tree.write(xodr, encoding="utf-8", xml_declaration=True)
    if moved and meta.get("lefthand"):
        # the pole's keep-left side is chosen for its s: lht_poles again, then the lanes
        # (lanes.json also loses the lights signal_phases removed) / 柱の位置と車線を再計算
        import subprocess
        py = os.path.join(HERE, "..", "..", "..", "..", "carla-venv", "bin", "python")
        py = py if os.path.exists(py) else sys.executable
        subprocess.check_call([py, os.path.join(HERE, "lht_poles.py"), xodr, xodr], stdout=subprocess.DEVNULL)
    if jc and jc.get("phases", True) or moved:
        import subprocess
        py = os.path.join(HERE, "..", "..", "..", "..", "carla-venv", "bin", "python")
        py = py if os.path.exists(py) else sys.executable
        subprocess.check_call([py, os.path.join(HERE, "lanes.py"), xodr, os.path.join(out, "lanes.json")])
    if n_nospot:
        print("signs: %d speed posts left out (no free kerb)" % n_nospot)
    print("signs: %d speed limit, %d give way" % (n_speed, n_give) + (", %d stop" % n_stop if stops else "")
          + (" (named %s), %d lane speeds set from JARTIC" % (stop_name, n_set) if speed_set or "jartic_log" in extra
             else ""))


if __name__ == "__main__":
    main()
