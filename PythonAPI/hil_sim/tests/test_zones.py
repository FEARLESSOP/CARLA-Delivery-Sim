#!/usr/bin/env python3
"""Pod delivery zones and no-fly zones (delivery/zones.py and their use in
planning, the drone flights, the battery estimates and delivery_sim's Fleet).
No CARLA server.

  ../../carla-venv/bin/python tests/test_zones.py

配送ゾーンと飛行禁止区域の試験（サーバー不要）。
"""
import os
import random
import sys
import tempfile
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

from delivery import zones, drone, planning  # noqa: E402
from delivery.battery import BatteryModel  # noqa: E402
from delivery.planning import Building, DRONE, FACE_TO_FACE  # noqa: E402
import delivery_sim as ds  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


R = lambda x0, y0, x1, y1: {"x0": x0, "y0": y0, "x1": x1, "y1": y1}
NF = R(0, -20, 50, 20)

print("=== file / ファイル ===")
tmp = os.path.join(tempfile.mkdtemp(prefix="zones_test_"), "delivery_zones.json")
check("map key from the full name", zones.map_key("Japan/Maps/OitaKosen/OitaKosen") == "OitaKosen")
saved = zones.save("Japan/Maps/OitaKosen/OitaKosen", {
    "pods": {"1": R(100, 50, 0, 0), "2": R(0, 0, 3, 3), "x": R(0, 0, 50, 50), "120": R(0, 0, 50, 50)},
    "nofly": [dict(NF, id="a", name="  school  "), R(0, 0, 1, 1), "junk", dict(NF, id="a")]}, tmp)
check("corners sorted", saved["pods"]["1"] == R(0, 0, 100, 50), saved["pods"])
check("tiny, bad-id and out-of-range pods dropped", set(saved["pods"]) == {"1"}, saved["pods"])
check("no-fly: bad ones dropped, ids unique, name trimmed",
      [n["id"] for n in saved["nofly"]] == ["a", "a_"] and saved["nofly"][0]["name"] == "school",
      saved["nofly"])
check("load returns what was saved", zones.load("OitaKosen", tmp) == saved)
check("other maps are empty", zones.load("Town03", tmp) == {"pods": {}, "nofly": [], "meet": {}, "dist": {}, "charge": []})

print("=== airspace / 空域 ===")
air = zones.Airspace([NF])
p = air.path((-100, 0), (150, 0))
check("a leg through the zone turns at its corners", len(p) == 3 and p[-1] == (150, 0), p)
check("no turn point inside the zone (margin kept)",
      all(not zones.inside(NF, q, zones.MARGIN) for q in p[:-1]), p)
check("a clear leg stays straight", air.path((-100, -100), (150, -100)) == [(150, -100)])
check("detour longer than the straight line", air.length((-100, 0), (150, 0)) > 250.0)
check("a leg starting inside the zone leaves straight", air.path((25, 0), (150, 0)) == [(150, 0)])
two = zones.Airspace([NF, R(80, -60, 120, 60)])
pts = two.path((-100, 0), (200, 0))
here, hit = (-100, 0), False
for q in pts:
    for k in range(1, 101):
        x = here[0] + (q[0] - here[0]) * k / 100.0
        y = here[1] + (q[1] - here[1]) * k / 100.0
        hit = hit or any(zones.inside(r, (x, y)) for r in two.zones)
    here = q
check("two zones: the path touches neither", not hit, pts)

print("=== flights / 飛行 ===")
zones.set_active(zones.Airspace([NF]))


def fly(f, n=6000, dt=0.05):
    inside, evs = 0, []
    for _ in range(n):
        evs += [e for e, _ in f.update(dt)]
        if zones.inside(NF, f.pos):
            inside += 1
        if f.done:
            break
    return inside, evs


f = drone.Transfer((-60, 0, 0), lambda: (120, 0, 0), 30)
ins, evs = fly(f)
check("transfer goes round, never inside", ins == 0 and evs == ["arrived"], (ins, evs))
f = drone.Sortie(lambda: (-60, 5, 0), [(120, 0, 0)], leg_z=[30])
ins, evs = fly(f)
check("sortie out and back round the zone", ins == 0 and "delivered" in evs and "landed" in evs,
      (ins, evs))
f = drone.Mission((-60, 0, 0), (120, 0, 0), 30, (-60, 10, 0), 30)
ins, evs = fly(f, 3000)
f.clear(lambda: (-60, 10, 0))
ins2, evs2 = fly(f)
check("mission: drop, meet and land without entering",
      ins + ins2 == 0 and "delivered" in evs and "landed" in evs2, (ins, ins2, evs, evs2))
f = drone.Transfer((-60, 0, 0), lambda: (120, 0, 0), 30)
for _ in range(60):
    f.update(0.05)
zones.set_active(zones.Airspace([NF, R(-40, -80, -30, 80)]))
ins, evs = fly(f)
check("a zone drawn mid-flight is avoided too", evs == ["arrived"], evs)
bm = BatteryModel()
zones.set_active(None)
straight = bm.transfer_cost((-60, 0, 0), (120, 0, 0), 30)
zones.set_active(zones.Airspace([NF]))
around = bm.transfer_cost((-60, 0, 0), (120, 0, 0), 30)
check("battery estimate counts the detour", around > straight, (straight, around))
zones.set_active(None)

print("=== orders / 注文 ===")
rng = random.Random(3)
blds = [Building(i, "b%d" % i, (x, y, 0.0), (x, y + 5, 0.0))
        for i, (x, y) in enumerate((rng.uniform(-300, 300), rng.uniform(-300, 300)) for _ in range(300))]
zone1 = R(-300, -300, 0, 300)
gen = planning.OrderGenerator(blds, random.Random(1), min_batch=5, max_batch=5, drone_prob=1.0)
gen.area, gen.area_key = (lambda p: zones.inside(zone1, p)), 1
gen.no_drone = lambda p: zones.inside(R(-300, -300, -150, 300), p)
batch = sum((gen.make_batch(0.0) for _ in range(10)), [])
check("every order inside the pod's zone", batch and all(zones.inside(zone1, o.drop) for o in batch))
check("orders tagged with the pod", all(o.zone == 1 for o in batch))
check("no drone order in the no-fly zone",
      all(not (o.kind == DRONE and o.drop[0] < -150) for o in batch),
      [(o.kind, round(o.drop[0])) for o in batch if o.drop[0] < -150][:5])
gen.area = lambda p: False
check("an empty zone gives no batch (counted)", gen.make_batch(0.0) == [] and gen.empty_area == 1)

d = planning.Dispatcher()
b1 = [planning.Order(1, FACE_TO_FACE, blds[0], 0.0, 1, zone=1)]
b2 = [planning.Order(2, FACE_TO_FACE, blds[1], 0.0, 2, zone=2)]
d.add(b1)
d.add(b2)
out = d.assign([(2, (0, 0, 0))], can=lambda b, v: b[0].zone == v)
check("dispatcher skips a batch the free pod may not take", [(v, b[0].id) for v, b in out] == [(2, 2)])
check("the skipped batch keeps its place", [b[0].id for b in d.queue] == [1])

print("=== fleet / 配車 ===")
LOG = []
fl = ds.Fleet.__new__(ds.Fleet)
fl.vans = [types.SimpleNamespace(vid=i) for i in (1, 2, 3)]
fl.dispatcher, fl.gen, fl.rng = planning.Dispatcher(), gen, random.Random(5)
fl.log = lambda t, ev, *a, **k: LOG.append((ev, k.get("detail", "")))
fl.cancelled = 0
fl.zones = zones.Zones({"pods": {"1": R(-300, -300, 0, 0), "2": R(0, 0, 300, 300)}})
inz = lambda zone: next(b for b in blds if zones.inside(zone, b.drop))
out_b = next(b for b in blds if b.drop[0] < 0 < b.drop[1])          # outside both zones
mk = lambda b, i: [planning.Order(i, FACE_TO_FACE, b, 0.0, i)]
check("pod 1 takes its zone", fl._can(mk(inz(R(-300, -300, 0, 0)), 1), 1))
check("pod 2 may not take pod 1's zone", not fl._can(mk(inz(R(-300, -300, 0, 0)), 1), 2))
check("outside every zone: the pod without a zone", fl._can(mk(out_b, 2), 3) and not fl._can(mk(out_b, 2), 1))
ready = [types.SimpleNamespace(vid=2)]
fl._set_area(0.0, ready)
check("next batch goes to the idle pod's zone", gen.area_key == 2 and fl._hungry, gen.area_key)
check("the area test matches pod 2's zone", gen.area((10, 10, 0)) and not gen.area((-10, -10, 0)))
fl.dispatcher.add(mk(out_b, 3))
fl.zones = zones.Zones({"pods": {"1": R(-300, -300, 0, 0), "2": R(0, 0, 300, 300),
                                 "3": R(-300, 0, -200, 100)}})
fl._cancel_orphans(0.0)
check("every pod zoned: orders outside them all are cancelled",
      not fl.dispatcher.queue and fl.cancelled == 1 and LOG[-1][0] == "orders_cancelled", LOG[-1:])
fl.zones = zones.Zones({})
fl._set_area(0.0, ready)
check("no zones: old behaviour (any building, any pod)", gen.area is None and fl._can(mk(out_b, 4), 1))

print("=== meeting points / 合流地点 ===")
e = zones.clean({"meet": {"1": [{"x": 10, "y": 20}, {"x": "bad"}, {"x": 1e9, "y": 0}], "0": [{"x": 1, "y": 1}],
                          "2": [{"x": i, "y": 0} for i in range(40)]}})
check("bad / out-of-range points and pods dropped", e["meet"].get("1") == [{"x": 10.0, "y": 20.0}]
      and "0" not in e["meet"], e["meet"].get("1"))
check("at most 24 points per pod", len(e["meet"]["2"]) == zones.MAX_MEET)
zs = zones.Zones({"meet": {"1": [{"x": 25, "y": 0}, {"x": 200, "y": 0}]}, "nofly": [NF]})
check("a meeting point in a no-fly zone is ignored", zs.meet_of(1) == [(200.0, 0.0)], zs.meet_of(1))
check("summary carries the points", zs.summary()["meet"]["1"][1] == {"x": 200.0, "y": 0.0})

HUBS = [(float(x), float(y), 0.0) for x in range(-300, 301, 4) for y in (-2.0, 2.0)]
LOGV = []
van = types.SimpleNamespace(
    vid=1, hubs=HUBS, args=types.SimpleNamespace(hub_radius=120.0), energy=types.SimpleNamespace(level=100.0),
    car_pos=lambda: (0.0, 0.0, 0.0), road_dist=planning.dist2d, batch_need=lambda stops: 0.0,
    log=lambda t, ev, *a, **k: LOGV.append(ev), drive_to_next=lambda t: None,
    fleet=types.SimpleNamespace(zones=zones.Zones({"meet": {"1": [{"x": -151, "y": 1}]}})))
near = [Building(900 + i, "n", (x, 30.0, 0.0), (x, 3.0, 0.0)) for i, x in enumerate((60.0, 80.0))]
dorders = [planning.Order(900 + i, DRONE, b, 0.0, 9) for i, b in enumerate(near)]
ds.Van.assign(van, dorders, 0.0)
hub = next(s for s in van.stops if s.kind == DRONE).location
check("the drone stop is the pod's meeting point (snapped to a kerb point)",
      hub == (-152.0, 2.0, 0.0), hub)
van.fleet.zones = zones.Zones({})
ds.Van.assign(van, [planning.Order(950, DRONE, near[0], 0.0, 9)], 0.0)
hub = next(s for s in van.stops if s.kind == DRONE).location
check("no meeting points: the stop is chosen near the drops", abs(hub[0] - 60.0) < 10, hub)

import delivery.lmp_van as lv  # noqa: E402
a, b = planning.Stop(DRONE, (-152.0, 2.0, 0.0), []), planning.Stop(FACE_TO_FACE, (100.0, 2.0, 0.0), [])
pod = types.SimpleNamespace(vid=1, stops=[a, b], fleet=types.SimpleNamespace(zones=zones.Zones({})))
check("no meeting points: drones meet at the next face-to-face stop", lv.LmpVanMixin.meet_point(pod, a) is b)
pod.fleet.zones = zones.Zones({"meet": {"1": [{"x": -151, "y": 1}]}})
check("with meeting points: not at a stop that is not one", lv.LmpVanMixin.meet_point(pod, a) is a)
pod.fleet.zones = zones.Zones({"meet": {"1": [{"x": -151, "y": 1}, {"x": 105, "y": 0}]}})
check("with meeting points: at the next stop when it is one (within 15 m)",
      lv.LmpVanMixin.meet_point(pod, a) is b)

print("=== charging stations / 充電ステーション ===")
cc = zones.clean({"charge": [{"x": 10.04, "y": -5, "yaw": 270}, {"x": "bad"}, {"x": 1, "y": 2, "yaw": "n"},
                             {"x": float("nan"), "y": 0}] + [{"x": i, "y": i} for i in range(20)]})["charge"]
check("charge spots: bad ones dropped, yaw wrapped, optional", cc[0] == {"x": 10.0, "y": -5.0, "yaw": -90.0}
      and cc[1] == {"x": 1.0, "y": 2.0}, cc[:2])
check("at most MAX_CHARGE spots", len(cc) <= zones.MAX_CHARGE, len(cc))
check("Zones keeps them and summary() returns them",
      zones.Zones({"charge": [{"x": 3, "y": 4}]}).summary()["charge"] == [{"x": 3.0, "y": 4.0}])
check("old files without charge still load", zones.clean({"pods": {}})["charge"] == [])

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
