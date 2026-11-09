#!/usr/bin/env python3
"""LastMile Hub (LMH-3) with Mini Pods: dispatch (no-fly zones -> hubs), the
hub timeline, kerb-side paths, the pod state machine (front drop, delivery
box, obstacle stop + recovery, faults + rescue, docking, meeting point),
distribution / meeting points and the published state. No CARLA server.

  ../../carla-venv/bin/python tests/test_lmh.py

LastMile ハブとミニポッドの試験（サーバー不要）。
"""
import math
import os
import random
import sys
import time
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

from delivery import lmh_rig as L, zones, planning  # noqa: E402
from delivery import minipod_sensors as MS  # noqa: E402
from delivery.planning import Building, DRONE, FACE_TO_FACE  # noqa: E402
import delivery_sim as ds  # noqa: E402
from delivery import lmh_hub  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


R = lambda x0, y0, x1, y1: {"x0": x0, "y0": y0, "x1": x1, "y1": y1}
NF = R(100, -50, 200, 50)
B = lambda i, x, y, box=None: Building(i, "b%d" % i, (x, y, 0.0), (x, y + 6.0, 0.0), box=box)
O = lambda i, b, kind=DRONE: planning.Order(i, kind, b, 0.0, i)


# ------------------------------------------------------------------ dispatch
print("=== no-fly orders go to the hubs / 飛行禁止区域の注文はハブへ ===")


def fleet(vans, entry):
    fl = ds.Fleet.__new__(ds.Fleet)
    fl.vans = vans
    fl.dispatcher, fl.rng = planning.Dispatcher(), random.Random(3)
    fl.gen = planning.OrderGenerator([B(i, x, 0) for i, x in enumerate(range(-300, 300, 20))], random.Random(1))
    fl.log = lambda *a, **k: None
    fl.cancelled = 0
    fl.zones = zones.Zones(entry)
    return fl


V = lambda vid, lmh=False: types.SimpleNamespace(vid=vid, lmh=lmh)
inside, outside = [O(1, B(1, 150, 0))], [O(2, B(2, -100, 0))]
fl = fleet([V(1), V(2), V(3, True)], {"nofly": [NF]})
check("default: a hub without a zone has the no-fly zones as its area", fl.area_of(3) == "nofly"
      and fl.area_of(1) == "any", (fl.area_of(3), fl.area_of(1)))
check("no-fly batch -> the hub only", fl._can(inside, 3) and not fl._can(inside, 1) and not fl._can(inside, 2))
check("other batches -> the drone pods, not the default hub", fl._can(outside, 1) and not fl._can(outside, 3))
mixed = [O(3, B(3, 150, 10)), O(4, B(4, -50, 0))]
parts = fl._split_nofly(mixed)
check("a mixed batch is split: no-fly part for the hub", len(parts) == 2 and [o.id for o in parts[0]] == [3]
      and [o.id for o in parts[1]] == [4], [[o.id for o in p] for p in parts])
ready = [V(3, True)]
fl._set_area(0.0, ready)
check("idle default hub: next batch made inside the no-fly zone", fl.gen.area_key == "nofly"
      and fl.gen.area((150, 0, 0)) and not fl.gen.area((0, 0, 0)), fl.gen.area_key)
check("...and drone kinds allowed there (ground orders for the pods)", fl.gen.no_drone is None)
fl.gen.area_key = None
fl.zones = zones.Zones({"nofly": [NF], "pods": {"3": R(-300, -300, 0, 300)}})
check("the user's zone wins: hub 3 uses its zone", fl.area_of(3) == "zone" and fl._can(outside, 3))
check("no-fly orders no hub covers: old way (face to face by a pod)", fl._can(inside, 1) and not fl._can(inside, 3))
fl.zones = zones.Zones({"nofly": [NF], "pods": {"3": R(90, -60, 210, 60), "1": R(-300, -300, 0, 300)}})
check("a hub whose zone holds the no-fly orders takes them", fl._can(inside, 3) and not fl._can(inside, 2))
fl = fleet([V(1), V(2), V(3, True), V(4, True)], {"nofly": [NF], "pods": {"3": R(-300, -300, 0, 300)}})
check("zoned hub + a default hub: no-fly orders to the default one", fl._can(inside, 4) and not fl._can(inside, 3)
      and fl.area_of(4) == "nofly")
fl = fleet([V(1), V(2)], {"nofly": [NF]})
check("no hubs: old behaviour (any pod, face to face)", fl._can(inside, 1) and fl._can(inside, 2)
      and fl._split_nofly(mixed) == [mixed])
fl = fleet([V(1), V(3, True)], {})
check("no no-fly zones: the hub takes any order", fl.area_of(3) == "any" and fl._can(outside, 3) and fl._can(outside, 1))

LOGV = []
fake = types.SimpleNamespace(vid=3, fleet=types.SimpleNamespace(nofly=lambda p: zones.inside(NF, p)),
                             log=lambda t, ev, *a, **k: LOGV.append((ev, k.get("detail", ""))),
                             lmh_assign=lambda b, t: [o.kind for o in b])
kinds = ds.LmhVan.assign(fake, [O(5, B(5, 150, 0)), O(6, B(6, 20, 0), FACE_TO_FACE)], 0.0)
check("hub assign: every order a ground delivery", kinds == [FACE_TO_FACE, FACE_TO_FACE], kinds)
check("event nofly_minipod logged", LOGV and LOGV[0][0] == "nofly_minipod" and "hub 3" in LOGV[0][1], LOGV)

# the distribution point for a no-fly batch: on the road, may be inside the zone
HUBS = [(float(x), float(y), 0.0) for x in range(-300, 301, 4) for y in (-2.0, 2.0)]
LOGA = []
hubv = types.SimpleNamespace(
    vid=3, hubs=HUBS, power=L.PodPower(), car_pos=lambda: (0.0, 0.0, 0.0), road_dist=planning.dist2d,
    batch_need=lambda stops: 0.0, energy=types.SimpleNamespace(level=100.0),
    log=lambda t, ev, *a, **k: LOGA.append(ev), drive_to_next=lambda t: None,
    mission=lambda o: {"kind": "front"},
    fleet=types.SimpleNamespace(zones=zones.Zones({"nofly": [NF]})))
hubv._log_dist = lambda t, s, d: lmh_hub.LmhVanMixin._log_dist(hubv, t, s, d)
lmh_hub.LmhVanMixin.lmh_assign(hubv, [O(7, B(7, 150, 30)), O(8, B(8, 170, 20))], 0.0)
dp = hubv.stops[0].location
check("auto distribution point near the no-fly orders (inside the zone is fine)",
      zones.inside(NF, dp) and len(hubv.stops) == 1 and "hub_dist" in LOGA, dp)

# ------------------------------------------------------------------ zones dist
print("=== distribution / meeting points / 配布・合流地点 ===")
e = zones.clean({"dist": {"3": [{"x": 1, "y": 2}] + [{"x": i, "y": 0} for i in range(40)] + [{"x": "bad"}],
                          "0": [{"x": 1, "y": 1}]}})
check("dist validated like meet (<= 24, bad dropped, ids 1..99)", len(e["dist"]["3"]) == zones.MAX_DIST
      and "0" not in e["dist"] and e["dist"]["3"][0] == {"x": 1.0, "y": 2.0})
zs = zones.Zones({"dist": {"3": [{"x": 150, "y": 0}]}, "meet": {"3": [{"x": 160, "y": 0}]}, "nofly": [NF]})
check("dist_of / meet_all ignore no-fly zones (pods drive)", zs.dist_of(3) == [(150.0, 0.0)]
      and zs.meet_all(3) == [(160.0, 0.0)] and zs.meet_of(3) == [])
check("summary carries dist", zs.summary()["dist"]["3"] == [{"x": 150.0, "y": 0.0}])
hubv.fleet.zones = zones.Zones({"dist": {"3": [{"x": -201, "y": 1}, {"x": 61, "y": 3}]},
                                "meet": {"3": [{"x": 81, "y": 1}, {"x": -250, "y": 0}]}})
lmh_hub.LmhVanMixin.lmh_assign(hubv, [O(9, B(9, 70, 30)), O(10, B(10, 90, 25))], 0.0)
s0 = hubv.stops[0]
check("with dist points: parks at the best one, snapped to a kerb point", s0.location == (60.0, 2.0, 0.0), s0.location)
check("with meet points: pods return to the best one", s0.meet_pt == (80.0, 2.0, 0.0), s0.meet_pt)
far = [O(11, B(11, 280, 0)), O(12, B(12, -280, 0))]
g = L.plan_groups(far, HUBS, 700.0 / 1.4 / 2.0)
check("orders out of pod range from one point: two stops", len(g) == 2, [(p, [o.id for o in os]) for p, os in g])
check("choose_meet: none reachable -> None", L.choose_meet(far, [(0.0, 0.0, 0.0)], 100.0) is None)

# ------------------------------------------------------------------ geometry
print("=== timeline and parts / タイムラインと部品 ===")
check("phases as the HTML (scan 0-6, divert, lift, load, release, deploy)",
      [L.work_phase(t) for t in (1, 7, 13, 17, 23, 28, 40)] ==
      ["scan", "divert", "lift", "load", "release", "deploy", "pods_out"])
m = L.HubMech()
outs = m.schedule_load(0.0, [0, 1, 2])
check("roll-out times 27.4 / 27.7 / 30.0", [round(outs[i], 1) for i in range(3)] == [27.4, 27.7, 30.0])
check("gate 3 fires first, then 2, then 1", m.gate[2].at(7.0) > 0.9 and m.gate[1].at(7.0) < 0.1
      and m.gate[0].at(8.8) > 0.9)
check("lift down at 15.5 s, up again at 22 s", m.lift[0].at(15.5) > 0.99 and m.lift[0].at(22.5) < 0.01)
check("plug out, pins out, door down by 25.2 s", m.value("Bay_1_ChargePlug", 23.0) > 0.99
      and m.value("Bay_1_LockPins", 23.5) > 0.99 and m.door[0].at(25.3) > 0.99)
parts = L.load_parts()
check("lmh_parts.json read (asset build) with 3 bays", len(parts["hub"]["bays"]) == 3 and
      parts["hub"]["bays"][0]["ramp_foot"][1] == -2.0, parts.get("_stub"))
door = next(p for p in parts["hub"]["parts"] if p["name"] == "Bay_1_RampDoor")
pose = L.motion_pose(door, 1.0)
check("ramp door rotates about x (roll) to fold down", abs(pose[5] - door["motion"]["amount"]) < 1e-6, pose)
lift = next(p for p in parts["hub"]["parts"] if p["name"] == "Lift_1_Carriage")
check("lift slides 0.72 m down", abs(L.motion_pose(lift, 1.0)[2] - (lift["loc"][2] - 0.72)) < 1e-6)
sh = next(p for p in parts["minipod"]["parts"] if p["name"] == "Shutter")
check("shutter (scale) hidden when rolled up, shown when shut", L.motion_pose(sh, 1.0) is None
      and L.motion_pose(sh, 0.0) is not None)
flag = next(p for p in parts["minipod"]["parts"] if p["name"] == "Flag")
check("flag hidden stowed, raised 0.517 m", L.motion_pose(flag, 0.0) is None and
      abs(L.motion_pose(flag, 1.0)[2] - flag["loc"][2] - 0.517) < 1e-6)

print("=== kerb-side paths / 路肩側の経路 ===")
p = L.kerb_point(0, 0, 0, 0.0, 3.0, keep_left=True, inset=0.4)
check("keep-left: kerb on the left (-y for heading +x), 0.4 m inside the edge", abs(p[1] + 1.1) < 1e-9, p)
p = L.kerb_point(0, 0, 0, 90.0, 3.0, keep_left=False, inset=0.4)
check("keep-right heading +y: kerb on the right (-x)", abs(p[0] + 1.1) < 1e-9, p)
wps = [(x, 0.0, 0.0, 0.0, 3.5) for x in range(0, 20, 2)]
kp = L.kerb_path(wps)
check("kerb path offset along the lane", all(abs(q[1] + 1.35) < 1e-9 for q in kp) and len(kp) == 10)
grid = lambda n: [((n[0] + dx, n[1] + dy), (n[0] + dx, n[1] + dy), 1.0)
                  for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)) if (n[0] + dx, n[1] + dy) not in WALL]
WALL = {(5, y) for y in range(-5, 6)}
path = L.astar(((0, 0), (0, 0)), grid, lambda n: n == (10, 0), lambda n: math.hypot(10 - n[0], n[1]))
check("A* goes round a wall", path and path[-1] == (10, 0) and all(q not in WALL for q in path), len(path or []))
j = L.join_hub([(-1.0, -1.1, 0), (1.0, -1.1, 0), (5.0, -1.1, 0), (9.0, -1.1, 0)], (0.0, 0.0), 0.0, (0.66, -2.0, 0.15))
check("join_hub: starts at the ramp foot, runs clear of the hub body", j[0] == (0.66, -2.0, 0.15)
      and all(abs(q[0]) > L.HUB_HALF_L or abs(q[1]) > 1.5 for q in j[1:]), j[:3])
ob = [("walker", 2.0, 0.0, 0.0, 0.3, 0.3)]
check("obstacle within 2.5 m ahead blocks", L.blocked(L.ahead_points((0, 0, 0), [(10, 0, 0)], 0), ob) is not None)
check("walker 1.5 m to the side does not", L.blocked(L.ahead_points((0, 0, 0), [(10, 0, 0)], 0),
                                                     [("walker", 2.0, 1.5, 0, 0.3, 0.3)]) is None)
check("car box beside the path blocks only when it overlaps", L.blocked(
    L.ahead_points((0, 0, 0), [(10, 0, 0)], 0), [("vehicle", 2.0, 1.1, 0.0, 2.3, 0.9)]) is not None and L.blocked(
    L.ahead_points((0, 0, 0), [(10, 0, 0)], 0), [("vehicle", 2.0, 2.0, 0.0, 2.3, 0.9)]) is None)


# ------------------------------------------------------------------ the crew
print("=== the hub's crew: a full cycle / 1 サイクル ===")


class Env(object):
    def __init__(self):
        self.crew, self.obs, self.park, self.pose, self.t = None, [], True, (0.0, 0.0, 0.0, 0.0), 0.0
        self.routes = 0

    def hub_pose(self):
        return self.pose

    def parked(self):
        return self.park

    def bay(self, n):
        return L.bay_geometry(L.load_parts(), n)

    def ground(self, x, y, z):
        return z

    def route(self, pod, a, b):
        self.routes += 1
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d / 2))
        return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, 0.0) for k in range(1, n + 1)]

    def mission(self, o):
        d = o.drop
        if o.building.box:
            return {"kind": L.ORDER_BOX, "stop": (d[0], d[1] + 1.05, 0.0), "face_yaw": 90.0,
                    "back": L.BOX_BACK, "drop": d, "box": o.building.box}
        return {"kind": L.ORDER_FRONT, "stop": (d[0], d[1] + L.FRONT_GAP, 0.0), "face_yaw": 90.0,
                "back": 0.0, "drop": d, "box": None}

    def blocked(self, pod, pts):
        return L.blocked(pts, self.obs)

    def slow(self, pod):
        return False

    def door(self, hub, bay):
        return self.crew.mech.door[bay - 1].at(self.t)


def run(crew, env, t0, t1, dt=0.05, stop=None):
    evs, t = [], t0
    while t < t1:
        env.t = t
        for e in crew.step(t, dt, env):
            evs.append((round(t, 1), e["ev"], getattr(e.get("pod"), "id", None), e))
        t += dt
        if stop and stop(t):
            break
    return evs, t


def new_crew(hz=None):
    pods = [L.MiniPod(101 + i, 4, i + 1, power=L.PodPower(), hazards=hz) for i in range(3)]
    crew = L.HubCrew(4, pods, L.PodPower(), hz)
    env = Env()
    env.crew = crew
    return crew, env


crew, env = new_crew()
o_front, o_box = O(20, B(20, 30, -6)), O(21, B(21, -40, -6, box=7))
crew.start(0.0, [o_front, o_box])
evs, tend = run(crew, env, 0.0, 300.0, stop=lambda t: t > 40 and not crew.busy())
names = [e[1] for e in evs]
pods = {e[2]: e for e in evs}
check("two pods out at 27.4 / 27.7 s", [e[0] for e in evs if e[1] == "pod_out"] == [27.4, 27.7],
      [e[0] for e in evs if e[1] == "pod_out"])
hov = [e for e in evs if e[1] == "pod_handover"]
check("front drop + delivery box handed over", len(hov) == 2 and {bool(e[3].get("box")) for e in hov} == {True, False})
check("all pods docked again, doors shut, crew idle", names.count("pod_docked") == 2 and not crew.busy()
      and all(p.status == L.DOCKED for p in crew.pods), (round(tend, 1), [p.status for p in crew.pods]))
p1 = crew.pods[0]
check("docked pod sits in its bay facing the door", abs(p1.x + 0.66) < 1e-6 and abs(p1.y + 0.31) < 1e-6
      and abs(p1.yaw + 90.0) < 1e-6, (p1.x, p1.y, p1.yaw))
check("trips and km counted", p1.trips == 1 and p1.km > 0.05, (p1.trips, p1.km))
box_pod = next(p for p in crew.pods if p.trips and p.id != hov[0][2]) if len(hov) == 2 else None
check("the box pod reversed up to the box (stop then 0.39 m back)", any(e[1] == "pod_door" and e[3].get("box")
                                                                        for e in evs))

print("=== handover motion / 受け渡しの動き ===")
pod = L.MiniPod(150, 4, 1)
pod.status, pod.mission, pod.order = L.TO_DOOR, env.mission(o_front), o_front
pod.x, pod.y, pod.yaw = 30.0, -5.4, -90.0
pod.moves = []
env2 = Env()
env2.crew = crew
seen, t = [], 0.0
for _ in range(200):
    seen += [e["ev"] for e in pod.step(t, 0.05, env2)]
    if t > 0.3 and t < 0.31 + 0.05:
        yaw_mid = pod.yaw
    t += 0.05
check("front drop: turns its rear to the place first", abs(pod.yaw - 90.0) < 1.5 or "done" in seen, pod.yaw)
check("shutter rolled up during the push, shut after", max(pod.shutter.at(x / 10.0) for x in range(0, 100)) > 0.99
      and pod.shutter.at(t) < 0.01)
check("events at_door -> handover -> done", [e for e in seen if e in ("at_door", "handover", "done")] ==
      ["at_door", "handover", "done"], seen)
pod = L.MiniPod(151, 4, 1)
pod.status, pod.mission, pod.order = L.TO_DOOR, env.mission(o_box), o_box
pod.x, pod.y, pod.z, pod.yaw = -40.0, -4.95, 0.0, 90.0
pod.moves = []
t, flaps = 0.0, []
for _ in range(200):
    pod.step(t, 0.05, env2)
    flaps.append(pod.flap(t))
    t += 0.05
check("box: reversed 0.39 m to the intake", abs(pod.y - (-4.95 - 0.39)) < 0.02, round(pod.y, 3))
check("box: the intake flap opens and closes", max(flaps) > 0.99 and flaps[-1] < 0.01)

print("=== obstacle stop and recovery / 障害物で停止・回復 ===")
pod = L.MiniPod(160, 4, 1)
pod.status, pod.x, pod.y = L.TO_DOOR, 0.0, 0.0
pod.mission = {"kind": "front", "stop": (40, 0, 0), "face_yaw": 0.0, "back": 0.0, "drop": (40, 0, 0)}
pod.set_path([(float(x), 0.0, 0.0) for x in range(2, 42, 2)])
env3 = Env()
env3.obs = [("walker", 3.0, 0.0, 0.0, 0.3, 0.3)]
t, evs3 = 0.0, []
for _ in range(40):
    evs3 += pod.step(t, 0.05, env3)
    t += 0.05
check("stops for a walker 2.5 m ahead", pod.speed == 0.0 and pod.x < 1.0, (pod.x, pod.speed))
env3.obs = []
for _ in range(20):
    pod.step(t, 0.05, env3)
    t += 0.05
check("drives on when clear (no recovery needed)", pod.x > 0.5 and pod.status == L.TO_DOOR, pod.x)
env3.obs = [("vehicle", pod.x + 2.5, 0.0, 0.0, 2.2, 0.9)]
kinds = []
for _ in range(int(80 / 0.05)):
    kinds += [e["ev"] for e in pod.step(t, 0.05, env3)]
    env3.obs = [("vehicle", 6.0, 0.0, 0.0, 2.2, 0.9)] if pod.x < 9.0 else []
    t += 0.05
    if "stuck" in kinds:
        break
check("blocked for good: backs up and re-plans 3 times, then skips ahead (logged)",
      kinds.count("stuck_try") == 3 and kinds.count("stuck") == 1 and pod.x >= 7.5, (kinds, round(pod.x, 1)))
check("status back to to_door after the skip", pod.status == L.TO_DOOR, pod.status)

print("=== faults, rescue, recovery / 故障・救援・回収 ===")
hz = L.PodHazards(random_on=False, recover_s=40.0, fail_timeout=200.0)
crew, env = new_crew(hz)
o1 = O(30, B(30, 30, -6))
crew.start(0.0, [o1])
evs, t = run(crew, env, 0.0, 35.0)
p = next(q for q in crew.pods if q.order is o1)
p.force_fault(t, "motor")
check("fault: pod crashed, damage on that wheel", p.status == L.CRASHED and p.damage and
      any(k.startswith("Wheel") for k in p.damage["parts"]), p.damage)
evs, t = run(crew, env, t, 400.0, stop=lambda t: t > 60 and not crew.busy())
names = [e[1] for e in evs]
check("another pod took the parcel over and delivered it", "pod_rescue" in names and "pod_handover" in names
      and not any(e[1] == "order_failed" for e in evs), names)
check("the crashed pod came back to its bay (service crew)", "pod_recovered" in names and p.status == L.DOCKED)
hz = L.PodHazards(random_on=False, recover_s=500.0, fail_timeout=30.0)
crew, env = new_crew(hz)
o2 = O(31, B(31, 30, -6))
crew.start(0.0, [o2])
run(crew, env, 0.0, 35.0)
for q in crew.pods:
    if q.order is not o2:
        q.status = L.CRASHED          # no pod free to take it over / 救援できるポッドなし
        q.down_t = 35.0
next(q for q in crew.pods if q.order is o2).force_fault(35.0, "lidar")
evs, t = run(crew, env, 35.0, 120.0)
check("no pod to take it over: the order fails after the timeout",
      any(e[1] == "order_failed" and e[3]["order"] is o2 for e in evs))

print("=== meeting point / 合流地点 ===")
crew, env = new_crew()
o3 = O(40, B(40, 30, -6))
crew.start(0.0, [o3], meet=(60.0, 0.0, 0.0))
run(crew, env, 0.0, 34.0)
check("hub may leave once the pod is out and the doors are shut", crew.can_leave(34.0) and crew.pods_out())
env.park = False
crew.hub_leaving(34.0, env, (60.0, 0.0, 0.0))
evs, t = run(crew, env, 34.0, 120.0, stop=lambda t: any(p.status == L.WAITING for p in crew.pods))
check("pod delivers, goes to the meeting point and waits for the hub",
      any(e[1] == "pod_handover" for e in evs) and any(p.status == L.WAITING for p in crew.pods))
env.park, env.pose = True, (60.0, 1.0, 0.0, 0.0)
evs, t = run(crew, env, t, 260.0, stop=lambda t: not crew.busy())
check("hub parked there: the pod docks into its bay", any(e[1] == "pod_docked" for e in evs) and not crew.busy()
      and abs(crew.pods[0].x - 59.34) < 0.01, [(p.status, round(p.x, 2)) for p in crew.pods])

print("=== charging and range / 充電と航続 ===")
crew, env = new_crew()
crew.pods[0].battery = 50.0
run(crew, env, 0.0, 10.0)
check("docked pods charge from the hub (drawn energy counted)", crew.pods[0].battery > 52.0 and crew.hub_draw > 0)
crew, env = new_crew()
o4 = O(41, B(41, 900, -6))
crew.start(0.0, [o4])
evs, t = run(crew, env, 0.0, 2.0)
check("beyond pod range: deferred to another stop", o4 in crew.deferred and any(e[1] == "deferred" for e in evs))
for q in crew.pods:
    q.battery = 15.0
o5 = O(42, B(42, 30, -6))
crew.start(2.0, [o5])
run(crew, env, 2.0, 3.0)
check("pods too low: the order waits for them to charge (not deferred)", o5 in crew.queue and o5 not in crew.deferred)

print("=== published state / 公開状態 ===")
geo = types.SimpleNamespace(latlon=lambda x, y: (33.2, 131.6), heading=lambda y: 0.0)
pods18 = [L.MiniPod(101 + i, 4 + i // 3, i % 3 + 1) for i in range(18)]
suites = [MS.MiniPodSuite(p.id, geo) for p in pods18]
for p in pods18:
    p.status, p.speed, p.order = L.TO_DOOR, 1.6, o1
    p.mission = env.mission(o1)
    p.path, p.i = [(float(k), 0.0, 0.0) for k in range(50)], 0
t0 = time.perf_counter()
for rep in range(5):
    out = []
    for p, su in zip(pods18, suites):
        d = p.state(10.0 + rep)
        d["actor"] = None
        d["sensors"] = su.update(10.0 + rep, {"status": p.status, "x": p.x, "y": p.y, "z": 0.0, "yaw": p.yaw,
                                              "speed": p.speed, "battery": p.battery, "odo_l": rep, "odo_r": rep,
                                              "lid": 0, "shutter": 0, "flag": 1, "km": 0.1, "payload_kg": 2.4,
                                              "fail": None, "ranges": {"front": 3.0}, "near": None, "hub_d": 50.0})
        d["damage"] = MS.damage(p.damage)
        out.append(d)
ms = (time.perf_counter() - t0) * 1000.0 / 5
check("18 pods' state + sensors < 2 ms per publish", ms < 2.0, "%.2f ms" % ms)
d = out[0]
want = {"id", "hub", "bay", "status", "x", "y", "z", "yaw", "speed", "battery", "order", "dest", "lid", "shutter",
        "flag", "actor", "trips", "km", "fail", "path", "sensors", "damage"}
check("minipods[] keys as the dashboard expects", want <= set(d), want - set(d))
check("ids unique from 101, path <= 30 points", len({x["id"] for x in out}) == 18 and out[0]["id"] == 101
      and len(d["path"]) == 30)
s = d["sensors"]
grp = {"perception", "navigation", "drive", "power", "payload", "comms"}
check("sensors: every entry has label/value/unit/health/group/part",
      all(set(v) == {"label", "value", "unit", "health", "group", "part"} and v["group"] in grp and
          v["health"] in ("ok", "warn", "fail", "off") for v in s.values()))
need = {"lidar", "cam_front", "cam_rear", "cam_left", "cam_right", "cam_down_front", "cam_lid", "ultrasonic",
        "bumper", "gnss", "imu_acc", "wheel1", "wheel6", "motor1", "batt_soc", "batt_v", "batt_i", "batt_t",
        "batt_range", "lid", "shutter", "flag", "lte", "latency", "estop"}
check("the suite: lidar, 6 cameras, ultrasonic, bumper, GNSS, IMU, wheels, motors, battery, lid, flag, LTE, e-stop",
      need <= set(s), need - set(s))
pf = L.MiniPod(199, 4, 1)
pf.force_fault(0.0, "lidar")
su = MS.MiniPodSuite(199)
sf = su.update(0.0, {"status": pf.status, "x": 0, "y": 0, "z": 0, "yaw": 0, "speed": 0, "battery": 50,
                     "odo_l": 0, "odo_r": 0, "fail": pf.fail})
check("lidar fault shows as fail, damage block has parts + why", sf["lidar"]["health"] == "fail"
      and MS.damage(pf.damage)["parts"] == {"lidar": "broken"} and MS.damage(pf.damage)["why"])
check("van entry fields for a hub", set(["kind", "pods_docked", "dist", "meet", "phase"]) <=
      set(lmh_hub.LmhVanMixin.lmh_van_state(types.SimpleNamespace(
          stops=[], crew=new_crew()[0], real_hub=False))))

print("=== parcel locker (mini.js Apartment B robot port) ===")
import carla  # noqa: E402
from delivery import boxes as BX  # noqa: E402
yaw = 30.0
ltf = carla.Transform(carla.Location(10.0, 20.0, 0.15), carla.Rotation(yaw=yaw))
fake = types.SimpleNamespace(boxes={5: types.SimpleNamespace(site=types.SimpleNamespace(yaw=yaw), ltf=ltf)},
                             has_locker=lambda b: b == 5)
fake.locker_point = lambda b, local: BX.BoxStations.locker_point(fake, b, local)
lm = lmh_hub.LmhVanMixin.mission(types.SimpleNamespace(boxes=fake),
                                 types.SimpleNamespace(building=types.SimpleNamespace(box=5)))
port = BX.BoxStations.locker_point(fake, 5, BX.LOCKER["port"][:2] + [0.0])
fwd = (-math.sin(math.radians(yaw)), math.cos(math.radians(yaw)))          # locker front (+y)
lock_d = math.hypot(lm["drop"][0] - port.x, lm["drop"][1] - port.y)
stop_d = math.hypot(lm["stop"][0] - lm["drop"][0], lm["stop"][1] - lm["drop"][1])
check("locker mission: box kind, locker flag", lm["kind"] == L.ORDER_BOX and lm["locker"] and lm["box"] == 5)
check("pod rear docks at the port face (centre POD_HL + 1 cm out)", abs(lock_d - (L.POD_HL + 0.01)) < 1e-3, round(lock_d, 3))
check("reverses BOX_BACK (0.39 m, mini.js) from its stop", abs(stop_d - L.BOX_BACK) < 1e-3, round(stop_d, 3))
check("pod faces away from the locker (rear to the port)",
      abs(math.cos(math.radians(lm["face_yaw"])) - fwd[0]) < 1e-6 and abs(math.sin(math.radians(lm["face_yaw"])) - fwd[1]) < 1e-6)
fl = BX.BoxStations._flap_tf(1.0)
bot = carla.Location(0.0, 0.0, -0.3)
fl.transform(bot)
check("open flap swings its bottom inward (-y, into the locker)", bot.y < fl.location.y - 0.2, round(bot.y - fl.location.y, 3))
check("port at Mini Pod bay height (parcel 0.31-0.59 m fits 0.29-0.65 m)",
      0.40 < BX.LOCKER["port"][2] < 0.55, BX.LOCKER["port"])
pk = L.MiniPod(198, 4, 2)
pk.mission = {"kind": L.ORDER_BOX, "locker": True}
pk.handover_t = 0.0
check("flap times as mini.js (open by 0.5 s, shut after 2.8 s)",
      pk.flap(0.6) > 0.99 and pk.flap(2.0) > 0.99 and pk.flap(2.9) < 0.01, [round(pk.flap(x), 2) for x in (0.6, 2.0, 2.9)])

print("=== charging station: pods on the pads / 充電ステーションのパッド ===")
from delivery import charge_sites as CS  # noqa: E402


class FakeStations(object):
    """charge_sites.Stations without CARLA: the pad poses / lane as it gives
    them, pad() calls recorded. / CARLA 無しの Stations（pad 呼び出しを記録）。"""

    def __init__(self, frames):
        self.lay = CS.layout()
        self.sites = [{"frame": f} for f in frames]
        self.calls, self.t = [], 0.0

    def pad_poses(self, sid):
        fr = self.sites[sid - 1]["frame"]
        return [fr.world(x, y) + (fr.z + CS.FLOOR_LIFT + z, CS._wrap(fr.yaw + yaw))
                for x, y, z, yaw in self.lay["pod_poses"]]

    def pad_lane(self, sid):
        fr, pl = self.sites[sid - 1]["frame"], self.lay["pad_lane"]
        return {"y_local": pl["y"], "far": fr.world(pl["far_x"], pl["y"]), "reverse_m": pl["reverse_m"]}

    def pad(self, sid, i, pins_u, plug_u):
        self.calls.append((self.t, sid, i, pins_u, plug_u))


class FakeHub(Env, lmh_hub.LmhVanMixin):
    """The van side (LmhVanMixin) on the test Env (hub pose, bays, doors)."""

    def __init__(self, frame, levels):
        Env.__init__(self)
        self.crew, _ = new_crew()
        for p, lv in zip(self.crew.pods, levels):
            p.battery = lv
        self.pose = (frame.x, frame.y, frame.z + 0.1, frame.yaw)    # parked on bay 04 / ベイ 04
        self.args = types.SimpleNamespace(minipod_kwh=1.0)          # pad_kw: default 3 kW
        self.stops, self.real_hub, self.LOG = [], False, []
        self.log = lambda t, ev, *a, **k: self.LOG.append((round(t, 2), ev, k.get("detail", "")))


def local(fr, p):
    a, l = L._frame((fr.x, fr.y), fr.yaw, p)
    return a, l


def pod_box(fr, p):
    """The pod's footprint corners in station-local terms."""
    x, y = local(fr, (p.x, p.y))
    r = math.radians(p.yaw - fr.yaw)
    c, s = math.cos(r), math.sin(r)
    return [(x + dx * c - dy * s, y + dx * s + dy * c) for dx, dy in
            ((L.POD_HL, L.POD_HW), (L.POD_HL, -L.POD_HW), (-L.POD_HL, -L.POD_HW), (-L.POD_HL, L.POD_HW))]


def _proj(pts, ax):
    v = [p[0] * ax[0] + p[1] * ax[1] for p in pts]
    return min(v), max(v)


def overlap(a, b):
    """Two convex quads overlap (separating axis test)."""
    for q in (a, b):
        for i in range(4):
            e = (q[(i + 1) % 4][0] - q[i][0], q[(i + 1) % 4][1] - q[i][1])
            ax = (-e[1], e[0])
            (a0, a1), (b0, b1) = _proj(a, ax), _proj(b, ax)
            if a1 <= b0 or b1 <= a0:
                return False
    return True


def seg_d(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    u = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy or 1.0)))
    return math.hypot(a[0] + u * dx - p[0], a[1] + u * dy - p[1])


def box_d(c, q):
    """Distance from point ``c`` to quad ``q`` (0 inside)."""
    if overlap(q, [(c[0] - 1e-6, c[1] - 1e-6), (c[0] + 1e-6, c[1] - 1e-6), (c[0] + 1e-6, c[1] + 1e-6),
                   (c[0] - 1e-6, c[1] + 1e-6)]):
        return 0.0
    return min(seg_d(c, q[i], q[(i + 1) % 4]) for i in range(4))


PLUG_C, PLUG_R = (2.83, -0.46), 0.7
HUB_BOX = [(-1.63, -0.75), (1.63, -0.75), (1.63, 0.75), (-1.63, 0.75)]
CABINET = [(3.45, -1.91), (4.09, -1.91), (4.09, -0.89), (3.45, -0.89)]     # bay 04 DC cabinet (GLB) / 充電盤
fr = CS.Frame(120.0, -40.0, 0.5, 35.0)
stn = FakeStations([CS.Frame(0.0, 0.0, 0.0, 0.0), fr])                      # site 2 / 2 番目
hub = FakeHub(fr, [60.0, 70.0, 80.0])
crew = hub.crew
pads_w = stn.pad_poses(2)
BAYS_L = {p.bay: local(fr, crew.bay_world(p, hub)[2])[0] for p in crew.pods}
seen, bad, worst = set(), [], {"plug": 9.0, "hub": 9.0, "cab": 9.0}


def tick(t, dt=0.05):
    hub.t, stn.t = t, t
    for e in crew.step(t, dt, hub):
        hub._crew_event(t, e)
    hub.station_pods_step(t, dt)
    boxes = []
    for p in crew.pods:
        seen.add(p.status)
        if p.status in L.STATION:
            q = pod_box(fr, p)
            x, y = local(fr, (p.x, p.y))
            on_ramp = abs(x - BAYS_L[p.bay]) < 0.05 and y > -2.05
            worst["plug"] = min(worst["plug"], box_d(PLUG_C, q))
            worst["cab"] = min(worst["cab"], 0.0 if overlap(q, CABINET) else 1.0)
            if not on_ramp:
                worst["hub"] = min(worst["hub"], 0.0 if overlap(q, HUB_BOX) else 1.0)
            boxes.append((p, q))
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            if overlap(boxes[i][1], boxes[j][1]):
                bad.append((round(t, 2), boxes[i][0].id, boxes[j][0].id))


t, DT = 0.0, 0.05
hub.station_pods_begin(t, stn, 2)
check("begin: every pod under 90 % sent to a pad, bay 3 -> M1, bay 2 -> M2, bay 1 -> M3",
      {p.bay: p.pad for p in crew.pods} == {3: 1, 2: 2, 1: 3} and all(p.status == L.TO_PAD for p in crew.pods),
      [(p.bay, p.pad, p.status) for p in crew.pods])
check("event pod_to_pad logged for each", sum(1 for e in hub.LOG if e[1] == "pod_to_pad") == 3, hub.LOG[:3])
st = hub.lmh_van_state()
check("van entry: station id and 3 pads (pod, level, phase 'in')", st["station"] == 2 and len(st["pads"]) == 3
      and all(set(d) == {"pod", "level", "phase"} and d["phase"] == "in" for d in st["pads"]), st["pads"])
while t < 90.0 and not all(d["phase"] == "charging" for d in st["pads"]):
    t += DT
    tick(t)
    st = hub.lmh_van_state()
check("all three on their pads and charging (< 60 s)", t < 60.0 and all(p.status == L.ON_PAD for p in crew.pods),
      (round(t, 1), [p.status for p in crew.pods]))
errs = []
for p in crew.pods:
    x, y, z, yaw = pads_w[p.pad - 1]
    errs.append((round(math.hypot(p.x - x, p.y - y), 4), round(abs(L._wrap(p.yaw - yaw)), 3)))
check("pods on the pad poses (< 5 cm, yaw < 2 deg)", all(d < 0.05 and a < 2.0 for d, a in errs), errs)
check("pad pose: nose +y of the station", all(abs(L._wrap(p.yaw - fr.yaw - 90.0)) < 2.0 for p in crew.pods))
check("hub doors shut behind them", crew.mech.doors_shut(t), [round(d.at(t), 2) for d in crew.mech.door])
names = [e[1] for e in hub.LOG]
check("event pod_on_pad for each", names.count("pod_on_pad") == 3)
check("statuses to_pad / on_pad published in minipods[]", {L.TO_PAD, L.ON_PAD} <= seen and
      {d["status"] for d in [p.state(t) for p in crew.pods]} == {"on_pad"})


def pad_series(i):
    return [(c[0], c[3], c[4]) for c in stn.calls if c[2] == i and c[1] == 2]


ok_in = True
for i in range(3):
    ser = pad_series(i)
    pins_t = next((c[0] for c in ser if c[1] >= 0.999), None)
    plug_t = next((c[0] for c in ser if c[2] > 0.001), None)
    ok_in = ok_in and pins_t is not None and plug_t is not None and pins_t <= plug_t + 1e-9 and ser[-1][2] >= 0.999
check("pad pins all the way in before the plug moves, then the plug in", ok_in)
rate = 3.0 * 0.9 / 1.0 * 100.0 / 3600.0
b0 = [p.battery for p in crew.pods]
for k in range(200):
    t += DT
    tick(t)
gain = [p.battery - b for p, b in zip(crew.pods, b0)]
check("charging at pad_kw * 0.9 / minipod_kwh (0.075 %/s at 3 kW, 1 kWh)",
      all(abs(g - rate * 200 * DT) < 1e-6 for g in gain), [round(g, 4) for g in gain])
hub.args.pad_kw = 6.0
b1 = crew.pods[0].battery
t += DT
tick(t)
check("pad_kw from args when set", abs(crew.pods[0].battery - b1 - 2 * rate * DT) < 1e-9)
hub.args.pad_kw = 3.0
st = hub.lmh_van_state()
check("van entry pads: pod ids, levels, phase charging", all(d["phase"] == "charging" and d["pod"] in
                                                             (101, 102, 103) and d["level"] > 60 for d in st["pads"]))
check("no deliveries from the charging bay", (crew.start(t, [O(90, B(90, 30, -6))]) or True) and
      not any(e["ev"] == "cycle" for e in crew.step(t, DT, hub)) and crew.cycle is None)
crew.queue = []
full = crew.pods[1]                                  # bay 2 on M2 nearly full / ほぼ満充電
full.battery = L.PAD_FULL - 0.01
t0 = t
while t < t0 + 60.0 and full.status != L.DOCKED:
    t += DT
    tick(t)
check("a full pod leaves its pad alone and docks; the others keep charging",
      full.status == L.DOCKED and [p.status for p in crew.pods].count(L.ON_PAD) == 2 and
      [e for e in hub.LOG if e[1] == "pod_off_pad" and "full" in e[2]], (round(t - t0, 1), [p.status for p in crew.pods]))
check("...back in its bay facing the door", abs(full.yaw - L._wrap(fr.yaw - 90.0)) < 1e-6 and
      math.hypot(full.x - crew.bay_world(full, hub)[0][0], full.y - crew.bay_world(full, hub)[0][1]) < 1e-6)
check("not done while pods are on the pads", not hub.station_pods_done(t))
hub.station_pods_recall(t)
t0 = t
while t < t0 + 120.0 and not hub.station_pods_done(t):
    t += DT
    tick(t)
check("recall: the rest unplug and come back; done() true", hub.station_pods_done(t) and
      all(p.status == L.DOCKED for p in crew.pods) and crew.mech.doors_shut(t), (round(t - t0, 1), [p.status for p in crew.pods]))
names = [e[1] for e in hub.LOG]
check("events: 3 pod_off_pad, 3 pod_docked", names.count("pod_off_pad") == 3 and names.count("pod_docked") == 3)
check("from_pad status seen (docking for the last metres)", {L.FROM_PAD, L.DOCKING} <= seen, seen)
ok_out = True
for i in range(3):
    ser = pad_series(i)
    top = max(k for k, c in enumerate(ser) if c[2] >= 0.999)
    plug_out = next((c[0] for c in ser[top:] if c[2] <= 0.001), None)
    pins_drop = next((c[0] for c in ser[top:] if c[1] < 0.999), None)
    ok_out = ok_out and plug_out is not None and pins_drop is not None and plug_out <= pins_drop + 1e-9 \
        and ser[-1][1] <= 0.001 and ser[-1][2] <= 0.001
check("leaving: plug out first, then the pins; pads at rest", ok_out)
check("docked pods sit in their bays, pins and plugs in", all(crew.mech.pins[i].at(t) > 0.99 and
                                                              crew.mech.plug[i].at(t) > 0.99 for i in range(3)))
check("session over: station null, pads free, crew idle", hub.lmh_van_state()["station"] is None and
      all(d["phase"] == "free" for d in hub.lmh_van_state()["pads"]) and not crew.busy())
check("route clear of the bay 04 plug-unit base (r 0.7 m at (2.83, -0.46))", worst["plug"] > 0.0,
      "closest %.2f m" % worst["plug"])
check("route clear of the bay 04 hub footprint (except on the ramp)", worst["hub"] > 0.0)
check("route clear of bay 04's DC cabinet", worst["cab"] > 0.0)
check("pods never overlap each other in the station", not bad, bad[:5])
rl = [local(fr, q) for q in L.pad_route((fr.x, fr.y, fr.z, fr.yaw), fr.world(0.66, -2.0) + (0.0,),
                                        fr.world(5.08, 1.0), 1.0)]
check("route: hub-side lane y -2.55, U-turn on the floor (x < 9.98 - pod), pad lane y 1.0",
      abs(rl[0][1] - L.STN_LANE_Y) < 1e-6 and max(a for a, _ in rl) + L.POD_HL < 9.98 and
      abs(rl[-2][1] - 1.0) < 1e-6 and abs(rl[-1][0] - 5.08) < 1e-6, [(round(a, 2), round(b, 2)) for a, b in rl])

hub = FakeHub(fr, [60.0, 98.0, 70.0])
hub.station_pods_begin(0.0, stn, 2)
check("full pods (>= PAD_START) stay docked; two go: bay 3 -> M1, bay 1 -> M2",
      [(p.bay, p.status, p.pad) for p in hub.crew.pods] == [(1, L.TO_PAD, 2), (2, L.DOCKED, None), (3, L.TO_PAD, 1)],
      [(p.bay, p.status, p.pad) for p in hub.crew.pods])
hub.station_pods_recall(0.2)
check("recall before they rolled out: they stay in their bays, done after the doors", all(
    p.status == L.DOCKED for p in hub.crew.pods) and hub.crew.station["trips"] == {})
t = 0.2
while t < 10.0 and not hub.station_pods_done(t):
    t += DT
    hub.t = t
    hub.crew.step(t, DT, hub)
    hub.station_pods_step(t, DT)
check("...done() true once the doors are shut", hub.station_pods_done(t) and hub.crew.station is None, round(t, 1))
check("statuses: old ones unchanged, to_pad / on_pad / from_pad added", L.STATUSES[:10] ==
      ("docked", "loading", "rolling_out", "to_door", "handover", "returning", "docking", "waiting", "crashed", "stuck")
      and L.STATUSES[10:] == ("to_pad", "on_pad", "from_pad"))

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
