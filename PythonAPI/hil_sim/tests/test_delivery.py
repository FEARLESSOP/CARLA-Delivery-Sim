"""Headless test of the delivery planner and the kinematic drone sortie.

Exercises delivery/planning.py and delivery/drone.py on synthetic buildings
and road points. No CARLA server needed.
"""
import itertools
import math
import types
import os
import random
import sys

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

from delivery import planning
from delivery.planning import (Building, Order, OrderGenerator, DRONE, FACE_TO_FACE,
                               best_tour, choose_hub, plan_batch, tour_length, dist2d)
from delivery.drone import Sortie

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


# a 600 m x 600 m grid town: buildings every 50 m, roads every 10 m along x/y lines
# 600 m 四方の格子状の街
BUILDINGS = [Building(i + 1, "b%d" % i, (x, y, 0.0), (x, y + 6.0, 0.0))
             for i, (x, y) in enumerate(itertools.product(range(0, 600, 50), repeat=2))]
ROAD = [(float(x), float(y), 0.0) for x in range(0, 600, 10) for y in range(0, 600, 100)] + \
       [(float(x), float(y), 0.0) for x in range(0, 600, 100) for y in range(0, 600, 10)]


print("order generation / 注文生成")
gen = OrderGenerator(BUILDINGS, random.Random(7), min_gap=20, max_gap=90)
gen.schedule_from(0.0)
check("first batch is not immediate by default", gen.next_t >= 20.0, "next_t=%.1f" % gen.next_t)
check("nothing before next_t", gen.poll(gen.next_t - 0.1) == [])
sizes, gaps, kinds, last_t = [], [], set(), gen.next_t
for _ in range(300):
    t = gen.next_t
    batch = gen.poll(t)
    sizes.append(len(batch))
    gaps.append(gen.next_t - t)
    kinds.update(o.kind for o in batch)
    ids = [o.building.id for o in batch]
    if len(set(ids)) != len(ids):
        check("no building twice in a batch", False, str(ids))
    drones = [o for o in batch if o.kind == DRONE]
    spread = max([dist2d(a.drop, b.drop) for a in drones for b in drones] or [0])
    if spread > 2 * gen.drone_radius + 1e-6:
        check("drone orders clustered", False, "spread=%.0f" % spread)
    for o in batch:
        gen.release(o)
check("batch size always 3..5", set(sizes) <= {3, 4, 5}, str(sorted(set(sizes))))
check("all of 3, 4 and 5 occur", set(sizes) == {3, 4, 5})
check("gap within 20..90 s", 20 <= min(gaps) and max(gaps) <= 90,
      "%.1f..%.1f" % (min(gaps), max(gaps)))
check("both order kinds occur", kinds == {DRONE, FACE_TO_FACE})
check("busy buildings released", not gen.busy_buildings)

g2 = OrderGenerator(BUILDINGS, random.Random(1))
g2.schedule_from(5.0, immediate=True)
check("--first-now releases at once", len(g2.poll(5.0)) >= 3)
b = g2.make_batch(6.0)
check("busy buildings are not reused while pending",
      not ({o.building.id for o in b} & {o.building.id for o in g2.poll(1e9)}))


print("drone tour / ドローン巡回")
pts = [(100.0, 0, 0), (0, 100.0, 0), (100.0, 100.0, 0)]
order, length = best_tour((0, 0, 0), pts)
check("square tour is optimal", abs(length - 400.0) < 1e-6, "len=%.1f" % length)
rng = random.Random(3)
ok = True
for _ in range(50):
    ps = [(rng.uniform(0, 300), rng.uniform(0, 300), 0) for _ in range(5)]
    o, L = best_tour((0, 0, 0), ps)
    brute = min(tour_length((0, 0, 0), ps, p) for p in itertools.permutations(range(5)))
    ok &= abs(L - brute) < 1e-6
check("exact tour matches brute force (5 pts x50)", ok)
ps = [(rng.uniform(0, 300), rng.uniform(0, 300), 0) for _ in range(12)]
o, L = best_tour((0, 0, 0), ps)
check("heuristic tour visits all 12 once", sorted(o) == list(range(12)))
nn_len = tour_length((0, 0, 0), ps, list(range(12)))
check("heuristic beats arbitrary order", L <= nn_len, "%.0f <= %.0f" % (L, nn_len))


print("hub choice / ハブ選定")
drops = [(300.0, 250.0, 0), (350.0, 300.0, 0), (250.0, 330.0, 0)]
hub, visit, L = choose_hub(drops, ROAD)
c = planning.centroid(drops)
check("hub is a road point", hub in ROAD)
check("hub is near the centroid", dist2d(hub, c) < 60, "%.1f m" % dist2d(hub, c))
check("hub tour is the best among candidates",
      all(best_tour(r, drops)[1] >= L - 1e-6 for r in ROAD if dist2d(r, c) <= 120))
far_hub = min(ROAD, key=lambda r: dist2d(r, (0, 0, 0)))
check("central hub beats a corner hub", L < best_tour(far_hub, drops)[1])
_, _, L1 = choose_hub([(320.0, 320.0, 0)], ROAD)
nearest = min(dist2d(r, (320.0, 320.0, 0)) for r in ROAD)
check("single drone order: hub is its nearest road point", abs(L1 - 2 * nearest) < 1e-6,
      "tour %.1f m" % L1)


print("batch plan / バッチ計画")
mk = lambda i, k, b: Order(i, k, BUILDINGS[b], 0.0, 1)
batch = [mk(1, FACE_TO_FACE, 3), mk(2, DRONE, 60), mk(3, DRONE, 61),
         mk(4, FACE_TO_FACE, 100), mk(5, DRONE, 72)]
stops = plan_batch(batch, (0, 0, 0), ROAD, dist2d)
kinds_ = [s.kind for s in stops]
check("one stop per face-to-face order + one hub", sorted(kinds_) ==
      [DRONE, FACE_TO_FACE, FACE_TO_FACE], str(kinds_))
hub_stop = [s for s in stops if s.kind == DRONE][0]
check("hub stop carries every drone order",
      sorted(o.id for o in hub_stop.drone_order) == [2, 3, 5])
check("f2f stop parks at the building curb",
      all(s.location == s.orders[0].building.curb for s in stops if s.kind == FACE_TO_FACE))
path = lambda ss: sum(dist2d(a, b) for a, b in zip([(0, 0, 0)] + [s.location for s in ss],
                                                    [s.location for s in ss]))
check("stop order is shortest open path",
      all(path(stops) <= path(list(p)) + 1e-6 for p in itertools.permutations(stops)))
only_f2f = plan_batch([mk(1, FACE_TO_FACE, 3), mk(2, FACE_TO_FACE, 9),
                       mk(3, FACE_TO_FACE, 50)], (0, 0, 0), ROAD, dist2d)
check("all-f2f batch has no hub", all(s.kind == FACE_TO_FACE for s in only_f2f))


print("drone sortie / ドローン出撃")
home = [(0.0, 0.0, 2.0)]
s = Sortie(lambda: home[0], [(100.0, 0, 0), (100.0, 100.0, 0)], cruise_alt=40.0,
           h_speed=10.0, v_speed=4.0, hover_s=2.0, drop_height=3.0, reload_s=5.0)
events, t, max_z, low, back, loaded = [], 0.0, 0.0, {}, [], set()
while not s.done and t < 900:
    for e in s.update(0.05):
        events.append((round(t, 2), e))
        if e[0] == "delivered":
            low[e[1]] = s.pos
        if e[0] == "reloading":
            back.append(s.pos)
    loaded.add((s.state, s.loaded))
    if t > 40:
        home[0] = (5.0, 0.0, 2.0)   # car creeps forward while the drone is out
    max_z = max(max_z, s.pos[2])
    t += 0.05
check("sortie finishes", s.done, "t=%.1f" % t)
check("events: deliver, back in the van to reload, deliver, land",
      [e for _, e in events] == [("delivered", 0), ("reloading", 1), ("delivered", 1),
                                 ("landed", None)], str(events))
check("reloads inside the van, at its current spot",
      len(back) == 1 and dist2d(back[0], home[0]) < 1e-6 and abs(back[0][2] - 2.0) < 1e-6,
      str(back))
check("cruises at launch z + cruise_alt", abs(max_z - 42.0) < 1e-6, "max z %.2f" % max_z)
check("hovers drop_height above the parcel spot",
      all(abs(p[2] - 3.0) < 1e-6 and dist2d(p, d) < 1e-6
          for (i, p), d in zip(sorted(low.items()), [(100.0, 0, 0), (100.0, 100.0, 0)])))
check("lands in the car's current spot", dist2d(s.pos, home[0]) < 1e-6 and
      abs(s.pos[2] - 2.0) < 1e-6, str(s.pos))
check("carries a parcel only on the way out",
      all(l == (st in ("takeoff", "cruise", "descend", "drop")) for st, l in loaded))
# trip 1: up 40, out 100, down 39, up 39 (the car moves to x=5 at t=40, during
# this climb), back 95, down 40; trip 2: up 40, out hypot(95, 100), down 39,
# up 39, back the same, down 40
trip2 = math.hypot(95, 100)
expect = (40 + 100 + 39 * 2 + 95 + 40) + (40 + trip2 + 39 * 2 + trip2 + 40)
check("flown distance plausible", abs(s.distance - expect) < 1.0,
      "%.1f vs %.1f" % (s.distance, expect))
empty = Sortie(lambda: (0, 0, 0), [], cruise_alt=10, v_speed=5)
check("sortie with no drops is done at once", empty.done and empty.update(0.1) == [])


print("per-leg altitude / 区間ごとの高度")
from delivery.planning import Obstacle, leg_altitude, sortie_altitudes
A, B = (0.0, 0.0, 0.0), (100.0, 0.0, 0.0)
z, w = leg_altitude(A, B, [], margin=8, min_alt=15)
check("open sky: fly at the minimum", z == 15.0 and w is None, "z=%.1f" % z)
tower = Obstacle(50, 0, 5, 5, 0, 60.0)
z, w = leg_altitude(A, B, [tower], margin=8, min_alt=15)
check("tower on the path: clear it by the margin", z == 68.0 and w is tower, "z=%.1f" % z)
z, _ = leg_altitude(A, B, [Obstacle(50, 30, 5, 5, 0, 60.0)], corridor=5, margin=8, min_alt=15)
check("tower 25 m off the path is ignored", z == 15.0, "z=%.1f" % z)
z, _ = leg_altitude(A, B, [Obstacle(50, 9, 5, 5, 0, 30.0)], corridor=5, margin=8, min_alt=15)
check("building 4 m off the path counts (corridor 5 m)", z == 38.0, "z=%.1f" % z)
z, _ = leg_altitude(A, B, [Obstacle(150, 0, 5, 5, 0, 60.0)], margin=8, min_alt=15)
check("tower beyond the end of the leg is ignored", z == 15.0, "z=%.1f" % z)
# long thin building 20 m off the path: rotated 45 deg its corner comes within
# ~4.5 m of the path; unrotated its near edge stays 18 m away
slab = Obstacle(50, 20, 20, 2, 45, 40.0)
z, _ = leg_altitude(A, B, [slab], corridor=5, margin=8, min_alt=15)
check("rotated footprint is respected", z == 48.0, "z=%.1f" % z)
slab_flat = Obstacle(50, 20, 20, 2, 0, 40.0)
z, _ = leg_altitude(A, B, [slab_flat], corridor=5, margin=8, min_alt=15)
check("same slab unrotated does not reach the path", z == 15.0, "z=%.1f" % z)
z, _ = leg_altitude((0, 0, 5.0), (100, 0, 1.0), [], min_alt=15)
check("minimum is measured from the higher endpoint", z == 20.0, "z=%.1f" % z)
low, high = Obstacle(50, 0, 5, 5, 0, 10.0), Obstacle(100, 100, 5, 5, 0, 55.0)
legs = sortie_altitudes((0, 0, 2.0), [(100.0, 0, 0), (100.0, 200.0, 0)], [low, high],
                        margin=8, min_alt=15)
# trip 1 passes the low block (10 m); trip 2's straight line from home to
# (100, 200) passes (100, 100)? no: it passes (50, 100) -- clear of both
check("each round trip gets its own height", [round(v) for v in legs] == [18, 17],
      str([round(v, 1) for v in legs]))

home = [(0.0, 0.0, 2.0)]
s = Sortie(lambda: home[0], [(100.0, 0, 0), (100.0, 100.0, 0)], leg_z=[20.0, 50.0],
           h_speed=10.0, v_speed=5.0, hover_s=1.0, reload_s=1.0)
seen, t = {}, 0.0
while not s.done and t < 900:
    s.update(0.05)
    seen.setdefault((s.target_i, s.state), []).append(s.pos[2])
    t += 0.05
check("sortie with per-trip heights finishes", s.done)
check("trip 1 flies out and back at 20 m",
      all(abs(z - 20.0) < 1e-6 for z in seen[(0, "cruise")] + seen[(0, "return")]))
check("trip 2 flies out and back at 50 m",
      all(abs(z - 50.0) < 1e-6 for z in seen[(1, "cruise")] + seen[(1, "return")]))
check("takes off no higher than its trip", max(seen[(0, "takeoff")]) <= 20.0 + 1e-6)
try:
    Sortie(lambda: (0, 0, 0), [(1, 0, 0)], leg_z=[10.0, 20.0])
    check("wrong number of trip heights is rejected", False)
except ValueError:
    check("wrong number of trip heights is rejected", True)


# a tower right beside the start (the delivery building) vs one the path crosses
beside_start = Obstacle(0, 7, 2, 2, 0, 47.0)       # facade 5 m beside the start point
z, _ = leg_altitude(A, B, [beside_start], corridor=5, margin=8, min_alt=15)
check("tower beside the drop point is not flown over", z == 15.0, "z=%.1f" % z)
z, _ = leg_altitude((0, 0, 0), (5, 0, 0), [beside_start], corridor=5, margin=8, min_alt=15)
check("5 m hop beside a tower stays low", z == 15.0, "z=%.1f" % z)
across = Obstacle(2, 0, 1, 3, 0, 47.0)             # the path runs through it
z, _ = leg_altitude(A, B, [across], corridor=5, margin=8, min_alt=15)
check("tower the path crosses near the end is still cleared", z == 55.0, "z=%.1f" % z)
z, _ = leg_altitude(A, B, [], margin=8, min_alt=15, surface=40.0)
check("a hill under the leg (surface 40 m) is cleared by the margin", z == 48.0,
      "z=%.1f" % z)
legs = sortie_altitudes((0, 0, 2.0), [(100.0, 0, 0), (0.0, 100.0, 0)], [],
                        surface_fn=lambda a, b: 30.0 if b[0] == 100.0 else None,
                        margin=8, min_alt=15)
check("surface_fn is applied per trip", [round(v) for v in legs] == [38, 17],
      str([round(v, 1) for v in legs]))


print("multi-van dispatch / 複数台の配車")
from delivery.planning import Dispatcher
d = Dispatcher()
mkb = lambda bid, *idx: [Order(100 * bid + k, FACE_TO_FACE, BUILDINGS[i], 0.0, bid)
                         for k, i in enumerate(idx)]
west, east = mkb(1, 0, 1, 2), mkb(2, 140, 141, 142)      # x~0 vs x~550
d.add(west)
d.add(east)
out = d.assign([(1, (560.0, 0.0, 0.0)), (2, (0.0, 0.0, 0.0))])
check("oldest batch goes to the nearest free van", out[0] == (2, west), str(out[0][0]))
check("next batch goes to the remaining van", out[1] == (1, east), str(out[1][0]))
check("assigned orders record their van", all(o.van == 2 for o in west) and
      all(o.van == 1 for o in east))
check("queue is empty after assigning", d.queue == [] and d.orders == [])
d.add(mkb(3, 5, 6, 7))
check("no free van: the batch waits", d.assign([]) == [] and len(d.queue) == 1)
d.add(mkb(4, 8, 9, 10))
out = d.assign([(1, (0.0, 0.0, 0.0))])
check("one free van takes only one batch", len(out) == 1 and len(d.queue) == 1)

# fleet simulation: 3 vans, random batches over time, each van busy for a while
rng2 = random.Random(5)
gen3 = OrderGenerator(BUILDINGS, random.Random(9), min_gap=5, max_gap=20)
gen3.schedule_from(0.0, immediate=True)
disp = Dispatcher()
busy_until = {1: 0.0, 2: 0.0, 3: 0.0}
owner, current, t = {}, {}, 0.0
dup = False
while t < 3000:
    disp.add(gen3.poll(t))
    for vid in list(current):
        if t >= busy_until[vid]:
            for o in current.pop(vid):
                gen3.release(o)
    free = [(v, (rng2.uniform(0, 600), rng2.uniform(0, 600), 0)) for v in busy_until
            if v not in current]
    for vid, b in disp.assign(free):
        for o in b:
            dup |= o.id in owner
            owner[o.id] = vid
        current[vid] = b
        busy_until[vid] = t + rng2.uniform(100, 300)
    t += 1.0
vans_used = set(owner.values())
check("fleet sim: no order is ever given to two vans", not dup and len(owner) > 50,
      "%d orders" % len(owner))
check("fleet sim: all three vans got work", vans_used == {1, 2, 3}, str(vans_used))


small = OrderGenerator(BUILDINGS[:4], random.Random(2), min_batch=3, max_batch=3)
first = small.make_batch(0.0)
second = small.make_batch(1.0)        # only 1 building left free
third = small.make_batch(2.0)         # none left
check("generator: partial batch when few buildings are free", len(second) == 1,
      "%d orders" % len(second))
check("generator: no free building -> batch skipped, not a crash",
      third == [] and small.skipped == 1)


print("pod and drone moving parts / ポッドとドローンの可動部品")
from delivery import rig
up, down = rig.jack_poses(0.0), rig.jack_poses(1.0)
check("four jacks", sorted(up) == ["front_left", "front_right", "rear_left", "rear_right"])
check("jacks retracted are up, lowered are on the ground",
      all(abs(up[k][2] - rig.JACK_STROKE) < 1e-9 and abs(down[k][2]) < 1e-9 for k in up))
check("jacks only move vertically", all(up[k][:2] == down[k][:2] for k in up))
shut, open_ = rig.hatch_poses(0.0), rig.hatch_poses(1.0)
check("hatch shut is flat", all(shut[k][3:] == (0.0, 0.0, 0.0) for k in shut))


def free_edge_z(hinge_pose, reach):
    """z of the door's free edge: the panel reaches ``reach`` m along x."""
    pitch = math.radians(hinge_pose[3])
    return hinge_pose[2] + reach * math.sin(pitch)   # x' = r cos p, z' = r sin p


check("both doors lift their free edge when opening",
      free_edge_z(open_["front"], -0.61) > rig.ROOF_Z + 0.5 and
      free_edge_z(open_["rear"], 0.61) > rig.ROOF_Z + 0.5)
check("hatch opens past vertical, leaning away from the hole",
      abs(open_["front"][3]) > 90 and abs(open_["rear"][3]) > 90)
check("eased motion starts and ends at rest", rig.smooth(0.0) == 0.0 and rig.smooth(1.0) == 1.0
      and rig.smooth(0.5) == 0.5 and rig.smooth(1.7) == 1.0)
r0, r1 = rig.rotor_poses(0.0), rig.rotor_poses(30.0)
check("rotors stay on their hubs and turn", all(a[:3] == b[:3] for a, b in zip(r0, r1)) and
      [round(p[4]) for p in r1] == [30, 330, 330, 30])

ident = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
p = rig.to_parent(*(ident + ((3.0, 4.0, 5.0), 30.0)))
check("to_parent with an unrotated parent is a plain offset",
      all(abs(a - b) < 1e-9 for a, b in zip(p, (3, 4, 5, 0, 30, 0))), str(p))
c90 = ((10.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))  # parent yaw 90
p = rig.to_parent(*(c90 + ((10.0, 5.0, 1.0), 90.0)))
check("to_parent with a parent turned 90 deg", all(abs(a - b) < 1e-9 for a, b in
                                                   zip(p, (5, 0, 1, 0, 0, 0))), str(p))
th = math.radians(10)       # parent pitched 10 deg nose-up (parked on a hill)
hill = ((0.0, 0.0, 0.0), (math.cos(th), 0.0, math.sin(th)), (0.0, 1.0, 0.0),
        (-math.sin(th), 0.0, math.cos(th)))
p = rig.to_parent(*(hill + ((0.0, 0.0, 10.0), 0.0)))
check("on a hill the body is pitched back to level", abs(p[3] + 10.0) < 1e-6 and
      abs(p[4]) < 1e-6 and abs(p[5]) < 1e-6, str([round(v, 3) for v in p]))



print("wind / 風")
import random as _random
from delivery.wind import Wind, Drift
w = Wind(_random.Random(1), base_speed=3.0, base_dir=0.0, gust_rate=0.0)
wx, wy = w.vector(0.0)
check("calm-ish base wind blows towards its direction", 2.5 < wx < 4.0 and abs(wy) < 1e-9,
      "%.2f, %.2f" % (wx, wy))
g = w.gust(10.0, 8.0, 90.0, 4.0)
check("a gust peaks mid-way and is gone after its duration",
      abs(w.vector(12.0)[1] - 8.0) < 1e-6 and abs(w.vector(14.5)[1]) < 1e-9)
check("wind state reports the gust", w.state(12.0)["gust"] and w.state(12.0)["gust_peak"] == 8.0)
w.calm()
check("calm stops everything", w.vector(12.0) == (0.0, 0.0))
w2 = Wind(_random.Random(2), base_speed=0.0, gust_rate=60.0)
for k in range(1200):
    w2.step(k * 0.05, 0.05)
check("random gusts happen at the asked rate (60/min for 60 s)", 30 <= len(w2.history) <= 90,
      "%d gusts" % len(w2.history))


def peak_drift(gust, cap=11.0, base=3.0):
    d, wind, t, peak = Drift(cap), Wind(_random.Random(0), base, 0.0, 0.0), 0.0, 0.0
    wind.gust(1.0, gust, 0.0, 6.0)
    while t < 20.0:
        d.step(0.05, wind.vector(t))
        peak, t = max(peak, d.offset), t + 0.05
    return peak, d.offset


small, after = peak_drift(4.0)
check("a gust the drone can fight shoves it ~1 m and it recovers (turbulence aside)",
      0.3 < small < 2.0 and after < 0.4, "peak %.2f m, after %.2f m" % (small, after))
big, _ = peak_drift(12.0)
check("a gust beyond the drone's limit blows it several metres", big > 5.0, "%.1f m" % big)
d = Drift(11.0)
for _ in range(100):
    d.step(0.05, (20.0, 0.0))
check("steady wind above the limit keeps pushing (saturated)", d.saturated and d.offset > 20)
free, held = Drift(11.0), Drift(11.0)
for k in range(40):     # a sudden 9 m/s gust: within the limit, but it lags
    wnd = (0.0, 0.0) if k < 2 else (9.0, 0.0)
    free.step(0.05, wnd)
    held.step(0.05, wnd, assist=True)
check("landing assist holds the path through a sudden gust",
      held.offset < 0.3 and free.offset > 2 * held.offset,
      "assisted %.2f m vs %.2f m" % (held.offset, free.offset))


print("battery / バッテリー")
from delivery.battery import Battery, BatteryModel
bm = BatteryModel(hover=0.25, payload=0.2, climb=0.1, wind_k=0.5, wind_ref=10.0,
                  slow=0.1, fast=1.0, reserve=15.0)
check("a parcel costs 20 % more", abs(bm.rate(True) - 0.30) < 1e-9)
check("wind costs more: 10 m/s adds half", abs(bm.rate(False, False, 10.0) - 0.375) < 1e-9)
check("climbing adds its own rate", abs(bm.rate(False, True) - 0.35) < 1e-9)
home, drop = (0.0, 0.0, 2.0), (120.0, 0.0, 0.0)
c = bm.trip_cost(home, drop, 22.0, 12.0, 4.0, 4.0, 3.0, 0.0)
# out: 120 m/12 + (22-3)/4 down at 0.30, climb 20/4 at 0.40, hover 4 s at 0.30
# back: 120/12 + 20/4 down at 0.25, climb 19/4 at 0.35
expect = (10 + 4.75) * 0.30 + 5 * 0.40 + 4 * 0.30 + (10 + 5) * 0.25 + 4.75 * 0.35
check("trip cost adds up its legs", abs(c - expect) < 1e-6, "%.3f vs %.3f" % (c, expect))
check("windy trips cost more", bm.trip_cost(home, drop, 22.0, wind=12.0) > c * 1.5)
check("an empty transfer costs less than a delivery trip",
      bm.transfer_cost(home, drop, 22.0) < c)
b = Battery(50.0)
b.drain(10.0, 1.0)
b.charge(1000.0, 1.0)
check("battery drains and charges, capped at 100", b.level == 100.0)
b.drain(500.0, 1.0)
check("battery bottoms out at 0 and is dead", b.level == 0.0 and b.dead)
check("time to charge in the van", abs(bm.wait_to_reach(40.0, 50.0, 0.1) - 100.0) < 1e-9
      and bm.wait_to_reach(60.0, 50.0, 0.1) == 0.0)


print("transfer flight / 移動飛行")
from delivery.drone import Transfer
target = [(50.0, 0.0, 0.0)]
tr = Transfer((0.0, 0.0, 1.5), lambda: target[0], 20.0, h_speed=10.0, v_speed=5.0)
ev, t, top = [], 0.0, 0.0
while not tr.done and t < 120:
    ev += tr.update(0.05)
    top = max(top, tr.pos[2])
    t += 0.05
    if t > 5:
        target[0] = (52.0, 1.0, 0.0)    # the target moved (a van creeping forward)
check("transfer lands on the target where it is now", tr.done and ev == [("arrived", None)]
      and abs(tr.pos[0] - 52.0) < 1e-6 and abs(tr.pos[1] - 1.0) < 1e-6, str(tr.pos))
check("transfer cruises at its height and carries nothing", abs(top - 20.0) < 1e-6 and
      not tr.loaded)



print("order rate, backlog and pod battery / 注文頻度・ポッド電池")
import random as _r2
from delivery.planning import OrderGenerator as _OG, Dispatcher as _D
one = _OG(BUILDINGS, _r2.Random(5), min_gap=20.0, max_gap=90.0, fleet_size=1)
four = _OG(BUILDINGS, _r2.Random(5), min_gap=20.0, max_gap=90.0, fleet_size=4)
g1, g4 = [], []
for _ in range(200):
    one.schedule_from(0.0); four.schedule_from(0.0)
    g1.append(one.next_t); g4.append(four.next_t)
check("4 vans: batches come 4x as often", abs(sum(g1) / sum(g4) - 4.0) < 1e-9 and max(g4) <= 22.5)
four.schedule_from(100.0)
four.prompt(100.0, 3.0)
check("prompt brings the next batch forward", four.next_t == 103.0)
four.prompt(100.0, 10.0)
check("prompt never pushes a batch later", four.next_t == 103.0)
d = _D()
a = [Order(1, FACE_TO_FACE, BUILDINGS[0], 0.0, 1)]
b = [Order(2, FACE_TO_FACE, BUILDINGS[1], 0.0, 2)]
d.add(a)
b[0].van = 3
d.push_front(b)
check("a returned batch is served first, unowned", d.queue[0] == b and b[0].van is None)
from delivery.battery import PodEnergyModel
pem = PodEnergyModel(per_km=5.0, idle=0.02)
check("pod energy: driving + systems", abs(pem.cost(2.0, 300.0) - 16.0) < 1e-9)
from delivery.station import EvChargers
ev = EvChargers([(0, 0, 0), (100, 0, 0)])
van_a, van_b, van_c = (types.SimpleNamespace(vid=k) for k in (1, 2, 3))
check("nearest free charger is claimed", ev.claim(van_a, near=(90, 0, 0)) == 1)
check("then the other one", ev.claim(van_b, near=(90, 0, 0)) == 0)
check("none left: None", ev.claim(van_c) is None)
ev.release(van_a)
check("released charger is free again", ev.claim(van_c, near=(0, 0, 0)) == 1)



print("charging yard / 充電ヤード")
from delivery import yard as Y
yd = Y.yard_for("Carla/Maps/Town05")
check("Town05 has a charging yard, other maps don't",
      yd is not None and Y.yard_for("Carla/Maps/Town03") is None)
for bay, (bx, by) in enumerate(yd["bays"]):
    pin = Y.path_in(yd, bay, (35.3, 47.5))
    pout = Y.path_out(yd, bay)
    jump = max(math.hypot(b[0] - a[0], b[1] - a[1]) for p_ in (pin, pout)
               for a, b in zip(p_, p_[1:]))
    check("bay %d: path in ends in the bay facing +x" % (bay + 1),
          abs(pin[-1][0] - bx) < 1e-6 and abs(pin[-1][1] - by) < 1e-6 and pin[-1][2] % 360 == 0)
    check("bay %d: path out ends on the south road heading -y" % (bay + 1),
          abs(pout[-1][0] - yd["exit"][0]) < 1e-6 and abs(pout[-1][1] - yd["exit"][1]) < 1e-6
          and abs((pout[-1][2] - 270.0 + 180.0) % 360.0 - 180.0) < 1e-6)
    check("bay %d: no jumps along either path" % (bay + 1), jump < 0.16, "%.3f m" % jump)
    piv = [q for q in pout if abs(q[0] - yd["pivot"][0]) < 1e-9 and abs(q[1] - yd["lane_y"]) < 1e-9]
    check("bay %d: turns on the spot through 180 deg" % (bay + 1),
          len(piv) > 10 and abs(piv[-1][2] - piv[0][2]) > 170.0)
    # the turning circle (2.55 m) stays clear of the other bay / 旋回円は隣のベイに掛からない
    other = yd["bays"][1 - bay]
    check("bay %d: turning circle clear of the other bay" % (bay + 1),
          abs(other[1] - yd["lane_y"]) - 1.1 > 2.55)
mv = Y.Mover(Y.path_in(yd, 0, yd["entry"]))
steps = 0
while not mv.done and steps < 100000:
    mv.step(0.05)
    steps += 1
check("the mover finishes the path in a sensible time", mv.done and 10 < steps * 0.05 < 60,
      "%.0f s" % (steps * 0.05))



print("pod side port / サイドポート")
door0, door1 = rig.side_door_pose(0.0), rig.side_door_pose(1.0)
check("side door shut sits on its pivot", door0[:3] == rig.SIDE_DOOR)
check("side door open: out of the skin and 0.40 m forward",
      abs(door1[0] - (rig.SIDE_DOOR[0] + 0.40)) < 1e-9 and door1[1] < rig.SIDE_DOOR[1])
check("side door steps out before it slides", rig.side_door_pose(0.2)[0] == rig.SIDE_DOOR[0]
      and rig.side_door_pose(0.2)[1] < rig.SIDE_DOOR[1])
arm0, arm1 = rig.slide_poses(0.0), rig.slide_poses(1.0)
check("arm stowed: every part on its pivot",
      all(arm0[k][:3] == rig.SLIDE_PARTS[k] for k in arm0))
reach = [round(rig.SLIDE_PARTS[k][1] - arm1[k][1], 3) for k in ("stage1", "stage2", "stage3", "carrier")]
check("arm out: each part reaches 0.45 m further than the last", reach == [0.45, 0.9, 1.35, 1.8],
      str(reach))
check("arm out: the tray is clear of the pod's side (y < -1.1)", arm1["carrier"][1] + 0.25 < -1.1)


print()
if FAILS:
    print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
    sys.exit(1)
print("ALL PASSED")
