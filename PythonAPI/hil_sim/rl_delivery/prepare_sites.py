"""Charging stations and parcel lockers for the pretend world, from a live run.

delivery_sim places them in CARLA (delivery/charge_sites.py: LastMile charge
stations on free lawn, each with one pod bay + plug, a hub bay and a 3-pad
drone deck; delivery/boxes.py: the shared parcel lockers beside the kerb) and
exports their positions to data/delivery_live/map.json. This copies them for
one map into rl_delivery/maps/<Name>_sites.json, with delivery_sim's charging
and battery defaults, so the fleet world uses the same sites and numbers.
Run a delivery session with --charge-stations N once to (re)place them.

  python3 -m rl_delivery.prepare_sites --map OitaKosen

ライブ実行で置かれた充電ステーションとロッカーを疑似環境に写す。
"""
import argparse
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
LIVE = os.path.join(HIL, "data", "delivery_live", "map.json")

# delivery_sim.py defaults / delivery_sim の既定値
DEFAULTS = {
    "pod_kwh": 10.0, "drone_wh": 500.0, "station_kw": 50.0, "pod_charge_to": 95.0,
    "pod_drain_km": 5.0, "pod_idle_drain": 0.02, "pod_speed_kmh": 30.0,
    "charge_fast": 1.5, "charge_slow": 0.08, "standby_drones": 2, "drones_per_pod": 4,
    "drone_speed": 12.0,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="OitaKosen")
    a = ap.parse_args()
    with open(LIVE) as f:
        live = json.load(f)
    if not str(live.get("map", "")).endswith("/" + a.map):
        raise SystemExit("data/delivery_live/map.json is from %s; run delivery_sim on %s with "
                         "--charge-stations first" % (live.get("map"), a.map))
    pads = [tuple(p[:2]) for p in live.get("station", [])]
    chargers = [tuple(p[:2]) for p in live.get("chargers", [])]
    # chargers come in pairs per station (pod bay, hub bay); the deck pads are the
    # 3 pads nearest each station, the rest are overflow pads on the pavement
    # 充電器はステーションごとに 2 基（ポッド・ハブ）、甲板パッドは最寄り 3 個
    stations, used = [], set()
    for i in range(0, len(chargers) - 1, 2):
        pod_bay, hub_bay = chargers[i], chargers[i + 1]
        cx, cy = (pod_bay[0] + hub_bay[0]) / 2.0, (pod_bay[1] + hub_bay[1]) / 2.0
        near = sorted((j for j in range(len(pads)) if j not in used),
                      key=lambda j: math.dist(pads[j], (cx, cy)))[:3]
        near = [j for j in near if math.dist(pads[j], (cx, cy)) < 25.0]
        used.update(near)
        stations.append({"id": len(stations), "pod_bay": pod_bay, "hub_bay": hub_bay,
                         "deck": [pads[j] for j in near]})
    extra = [pads[j] for j in range(len(pads)) if j not in used]
    out = {"map": a.map, "stations": stations, "extra_pads": extra,
           "lockers": [tuple(p[:2]) for p in live.get("boxes", [])], "params": DEFAULTS}
    os.makedirs(os.path.join(HERE, "maps"), exist_ok=True)
    path = os.path.join(HERE, "maps", a.map + "_sites.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    print("%s: %d charging stations (%s deck pads), %d overflow pads, %d lockers -> %s" % (
        a.map, len(stations), "/".join(str(len(s["deck"])) for s in stations), len(extra),
        len(out["lockers"]), path))


if __name__ == "__main__":
    main()
