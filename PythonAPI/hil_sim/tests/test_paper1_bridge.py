"""Paper 1 CARLA bridge, practice side (rl_delivery/paper1/snapshot.py), without CARLA:
a practice day is turned into snapshots at every recovery decision, rebuilt by the
bridge, and B0 / B2 must choose the same as in the original world; B4 and B5 must
answer from a rebuilt snapshot. / CARLA なしでブリッジの練習世界側を確認する。

  ../../carla-venv/bin/python tests/test_paper1_bridge.py
"""
import math
import os
import sys
import time
import warnings

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
warnings.filterwarnings("ignore")

from rl_delivery import env_fleet as F                       # noqa: E402
from rl_delivery.paper1 import world as W1                   # noqa: E402
from rl_delivery.paper1.evaluate import hover_rule           # noqa: E402
from rl_delivery.paper1.methods import make_method           # noqa: E402
from rl_delivery.paper1.snapshot import SnapshotPlanner, checkpoint_for   # noqa: E402


def snapshot_from_world(w, d):
    """What the CARLA side would send, read from a practice world. / 練習世界からスナップショット"""
    pod = w.pods[0]
    near_st = lambda xy: min(range(len(w.stations)), key=lambda i: math.dist(w.stations[i]["xy"], xy))
    drones = []
    for o in w.drones:
        s = {"key": o["id"], "pos": [float(v) for v in o["pos"]], "batt": float(o["batt"])}
        if o is d:
            s["state"] = "dropped"
        elif o["state"] == F.DOCKED:
            s["state"] = "docked"
        elif o["state"] == F.FLY and o.get("task") == "deliver":
            s.update(state="deliver", target=[float(v) for v in o["leg"][3]], pos=[float(v) for v in w._draw_pos(o)])
        elif o["state"] == F.FLY and o.get("task") in ("meet", "return"):
            s.update(state="return", pos=[float(v) for v in w._draw_pos(o)])
        elif o["state"] == F.FLY and o.get("task") == "station":
            s.update(state="to_station", station=near_st(o["leg"][3]), pos=[float(v) for v in w._draw_pos(o)])
        elif o["state"] == F.HOVER:
            s["state"] = "hover"
        elif o["state"] in (W1.STN_CHARGE, W1.STN_QUEUE):
            s.update(state="charging" if o["state"] == W1.STN_CHARGE else "queue", station=o["station"])
        else:
            s["state"] = "down"
        drones.append(s)
    nxt = w._next_stop(pod)
    return {"t": w.t, "wind": list(w.wind_hist),
            "pod": {"pos": [float(v) for v in pod["pos"]], "batt": float(pod["batt"]),
                    "driving_to": [float(v) for v in w.xy[pod["drive"]["to"]]] if pod["drive"] else None,
                    "eta_s": pod["drive"]["t_left"] if pod["drive"] else None,
                    "meet": [float(v) for v in pod["meet"]] if pod["meet"] is not None else None,
                    "next_stop": [float(v) for v in w.xy[nxt]] if nxt is not None else None},
            "drones": drones,
            "orders": [{"xy": [float(v) for v in o["xy"]], "age_s": w.t - o["t0"], "kind": "pod" if o["kind"] == "f2f" else "drone"}
                       for o in w.orders.values() if o["state"] == "pod"]}


# ---- CARLA side with stand-in objects (delivery/paper1_bridge.py) / CARLA 側（代役オブジェクト）----
class _Obj(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _fake_pod():
    stops = [_Obj(kind="drone", location=(60.0, -300.0, 0.0)), _Obj(kind="face_to_face", location=(160.0, -260.0, 0.0)),
             _Obj(kind="drone", location=(250.0, -200.0, 0.0))]
    pads = [(489.0, -441.0, 1.0), (-122.0, -30.0, 1.0), (109.0, 59.0, 1.0)]
    station = _Obj(pads=pads, taken={})
    logs = []
    fleet = _Obj(station=station, grounded=False, charge_trips=0, plan_wind=lambda: 3.0,
                 cruise_z=lambda a, b: 40.0, _free_pad=lambda near: min(
                     (i for i in range(len(pads)) if i not in station.taken),
                     key=lambda i: math.dist(pads[i][:2], near[:2]), default=None),
                 bm=_Obj(transfer_cost=lambda a, b, z, h, v, w: 0.02 * math.dist(a[:2], b[:2])),
                 log=lambda *a, **k: logs.append(("fleet",) + a))
    sky = _Obj(drones=fleet, check=_Obj(flight_z=lambda a, b: 40.0))
    van = _Obj(vid=1, lmp=True, state="drone_out", stops=stops, meets={}, lmp_units=[], launched=set(),
               pending=[_Obj(id=7, kind="drone", drop=(90.0, -350.0, 0.0), created_t=100.0)],
               energy=_Obj(level=70.0), sky=sky, fleet=_Obj(zones=None), released=[],
               car_pos=lambda: (60.0, -300.0, 0.0), log=lambda *a, **k: logs.append(("van",) + a))
    van.release_drone = lambda u: van.released.append(u)

    def unit(i, status, fstate, pos, batt):
        u = _Obj(id=i, van=van, status=status, flight=_Obj(state=fstate, meet=stops[0].location, meet_z=40.0),
                 battery=_Obj(level=batt), pos=pos, h_speed=12.0, v_speed=4.0, order=None, pad_target=None)
        u.world_pos = lambda u=u: u.pos

        def start_transfer(fn, z, u=u):
            u.status, u.van, u.target = "to_station", None, fn()
        u.start_transfer = start_transfer
        return u
    us = [unit(1, "delivering", "ascend", (90.0, -350.0, 30.0), 64.0), unit(2, "docked", None, (60.0, -300.0, 2.0), 90.0),
          unit(3, "delivering", "cruise", (40.0, -250.0, 40.0), 75.0), unit(4, "docked", None, (60.0, -300.0, 2.0), 55.0)]
    us[2].order = _Obj(drop=(10.0, -200.0, 0.0))
    van.lmp_units = us
    van.meets = {id(stops[0]): {"stop": stops[0], "units": [us[0], us[2]]}}
    return van, us, logs


def carla_side():
    from delivery.paper1_bridge import Paper1RecoveryBridge
    fails = 0
    br = Paper1RecoveryBridge("B0", 0, log=lambda *a: None)
    try:
        van, us, logs = _fake_pod()
        snap = br.snapshot(us[0], van, 900.0, (3.0, 4.0))
        st = [d["state"] for d in snap["drones"]]
        ok = st == ["dropped", "docked", "deliver", "docked"] and snap["pod"]["next_stop"] == [160.0, -260.0] \
            and len(snap["orders"]) == 1 and abs(snap["wind"][0] - 5.0) < 1e-9
        fails += not ok
        print("CARLA snapshot (states, next stop, orders, wind): %s" % ("OK" if ok else "FAIL %s" % snap))
        job = br.on_drop(us[0], 900.0, (3.0, 4.0))
        t0 = time.time()
        while not job["future"].done() and time.time() - t0 < 120:
            time.sleep(0.2)
        done = br.poll(us[0], job, 901.0)
        ok = done and br.history and br.history[-1]["choice"] in ("pod", "pod_next", "station")
        fails += not ok
        print("worker process answers (B0): %s -> %s, applied %s (%.1f s incl. start-up)" % (
            "OK" if ok else "FAIL", br.history[-1]["choice"] if br.history else None,
            br.history[-1].get("applied") if br.history else None, time.time() - t0))
        # each choice carried out on a fresh pod / 各選択の実行
        van, us, logs = _fake_pod()
        br.apply(us[0], {"van": van, "t": 900.0}, {"kind": "pod_next", "xy": [160.0, -260.0]}, 901.0)
        moved = us[0] in van.meets[id(van.stops[1])]["units"] and us[0] not in van.meets[id(van.stops[0])]["units"]
        ok = tuple(us[0].flight.meet) == van.stops[1].location and moved
        fails += not ok
        print("next stop: meeting point and the pod's receive list moved: %s" % ("OK" if ok else "FAIL"))
        van, us, logs = _fake_pod()
        br.apply(us[0], {"van": van, "t": 900.0}, {"kind": "station", "station": 2, "xy": [109.0, 59.0]}, 901.0)
        ok = us[0].status == "to_station" and us[0].pad_target == 2 \
            and van.sky.drones.station.taken.get(2) == 1 and us[0] in van.released \
            and all(us[0] not in m["units"] for m in van.meets.values())
        fails += not ok
        print("charger: pad reserved, drone released, pod no longer waits: %s" % ("OK" if ok else "FAIL"))
        van, us, logs = _fake_pod()
        us[0].battery.level = 6.0
        n = br.counts["skipped"]
        br.apply(us[0], {"van": van, "t": 900.0}, {"kind": "station", "station": 0, "xy": [489.0, -441.0]}, 901.0)
        us[0].flight.state = "land"
        br.apply(us[0], {"van": van, "t": 900.0}, {"kind": "pod_next", "xy": [160.0, -260.0]}, 901.0)
        ok = br.counts["skipped"] == n + 2 and us[0].pad_target is None and tuple(us[0].flight.meet) == van.stops[0].location
        fails += not ok
        print("unreachable / too-late answers are skipped: %s" % ("OK" if ok else "FAIL"))
    finally:
        br.close()
    return fails


def main():
    fails = 0
    for name in ("B0", "B2"):
        ref = make_method(name, checkpoint_for(name, 0), world="v2")
        planner = SnapshotPlanner(name, 0, zone=1)
        w = W1.Paper1World(scenario="S2_rush_hour", world="v2", area=[1], zone=1)
        w.reset(seed=1003)
        same = n = 0
        while True:
            p = w.advance()
            if p is None or n >= 25:
                break
            if p[0] == "pod":
                w.pod_act(p[1], w.pod_rule(p[1]))
                continue
            d, kind = p[1], p[2]
            if kind == F.D_AFTER:
                opt, _ = ref.decide(w, d)
                want = ("pod" if opt[1] == 0 else "pod_next") if opt[0] == "pod" else "station"
                got = planner.decide(snapshot_from_world(w, d))
                same += (got["kind"] == want and (want != "station" or got["station"] == opt[1]))
                n += 1
                w.recover(d, opt)
            elif kind == F.D_HOVER:
                o = hover_rule(w, d)
                if o is None:
                    d["ask"] = None
                else:
                    w.recover(d, o, by_method=False)
            else:
                w.drone_act(d, kind, 1)
        ok = n >= 10 and same >= 0.8 * n
        fails += not ok
        print("%s: rebuilt snapshots choose like the original world %d/%d: %s" % (name, same, n, "OK" if ok else "FAIL"))
    # B4 / B5 answer from a rebuilt snapshot / B4・B5 が答える
    w = W1.Paper1World(scenario="S1_normal", world="v2", area=[1], zone=1)
    w.reset(seed=1001)
    snap = None
    while snap is None:
        p = w.advance()
        if p[0] == "pod":
            w.pod_act(p[1], w.pod_rule(p[1]))
        elif p[2] == F.D_AFTER:
            snap = snapshot_from_world(w, p[1])
        else:
            w.drone_act(p[1], p[2], 1)
    for name in ("B4", "B5"):
        t0 = time.time()
        a = SnapshotPlanner(name, 0, zone=1).decide(snap)
        ok = a["kind"] in ("pod", "pod_next", "station") and len(a["xy"]) == 2
        fails += not ok
        print("%s answers from a snapshot: %s -> %s (%.1f s)" % (name, "OK" if ok else "FAIL", a["kind"], time.time() - t0))
    fails += carla_side()
    print("ALL PAPER 1 BRIDGE TESTS PASSED" if not fails else "%d FAILED" % fails)
    return fails


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
