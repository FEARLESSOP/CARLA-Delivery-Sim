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
    tree.write(xodr, encoding="utf-8", xml_declaration=True)
    if n_nospot:
        print("signs: %d speed posts left out (no free kerb)" % n_nospot)
    print("signs: %d speed limit, %d give way" % (n_speed, n_give) + (", %d stop" % n_stop if stops else "")
          + (" (named %s), %d lane speeds set from JARTIC" % (stop_name, n_set) if speed_set or "jartic_log" in extra
             else ""))


if __name__ == "__main__":
    main()
