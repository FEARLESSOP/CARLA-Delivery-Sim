"""The LastMile pod's flow (delivery/lmp_van.py) with stand-in objects.

No CARLA server: the van, pod, drones and sky are small fakes with what
LmpVanMixin touches. One batch: a drone stop with 2 orders, a face-to-face
stop, then another drone stop. Checked:
  - both drones launch, one after the other, without waiting for returns,
    and the pod leaves while they are out
  - a parcel reaches its drone only when the hook animation ends
  - at the face-to-face stop: box station first, then the drones circling
    overhead land one at a time and are craned onto bays
  - the next drone is loaded while the pod drives to the next drone stop
  - drones and packs charging in the pod drain the pod's battery
LastMile ポッドの配達の流れをスタブで試験（サーバ不要）。
"""
import os
import sys
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import carla   # noqa: F401  (lmp_van -> backends imports it)
from delivery import lmp_rig as R
from delivery.lmp_van import LmpVanMixin
from delivery.battery import Battery, BatteryModel, PodEnergyModel
from delivery.planning import DRONE, FACE_TO_FACE
from delivery.station import DroneFleet

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


class Flight(object):
    def __init__(self):
        self.holding, self.cleared = False, None

    def clear(self, fn):
        self.cleared = fn


class Unit(object):
    def __init__(self, uid, level):
        self.id, self.battery, self.status, self.van, self.note = uid, Battery(level), "docked", None, ""
        self.flight, self.parcel, self.pos = None, False, (0, 0, 0)
        self.h_speed = self.v_speed = 10.0

    def dock_into(self, van):
        self.van, self.status, self.flight = van, "docked", None

    def _hang(self):
        self.parcel = True

    def start_mission(self, van, order, hover, z, meet, meet_z):
        self.status, self.flight, self.order, self.meet = "delivering", Flight(), order, meet


class Pod(object):
    def __init__(self):
        self.mech, self.pad = R.Mech(), (0.0, 0.0, 3.2)

    def slot_location(self, slot, lift=R.LIFT_MIN):
        return R.H((R.SLOTS[slot], R.slot_top(slot, lift), R.DRONE_Z))

    def apply(self, mech=None, drones=None):
        self.mech = mech or self.mech

    def _world(self, p):
        return types.SimpleNamespace(x=p[0], y=p[1], z=p[2])


def stop(kind, x, orders):
    return types.SimpleNamespace(kind=kind, location=(x, 0.0, 0.0), orders=orders,
                                 drone_order=orders if kind == DRONE else None, batch=1)


class Van(LmpVanMixin):
    def __init__(self, units, stops):
        self.pod, self.drone, self.incoming, self.vid = Pod(), None, None, 1
        self.args = types.SimpleNamespace(lmp_speed=1.0, drone_wait_max=600.0, max_charge_wait=30.0,
                                          drone_speed=12.0, climb_speed=4.0, hover_s=4.0,
                                          meet_hold_s=90.0)
        self.energy = Battery(80.0)
        self.sky = types.SimpleNamespace(
            bm=BatteryModel(), check=types.SimpleNamespace(flight_z=lambda a, b: 30.0),
            drones=types.SimpleNamespace(grounded=False, pod_draw=PodEnergyModel().drone_ratio,
                                         _wind=0.0, plan_wind=lambda: 3.0,
                                         request=lambda v, n: None,
                                         send_to_station=lambda u, t: False))
        self.lmp_init()
        self.events, self.stages_done, self.finished, self._said = [], 0, [], set()
        self.dock, self.state = (0, 0, 0), "driving"
        self.stops = stops
        self.pending = [o for s in stops for o in s.orders]
        for u in units:
            self.accept_drone(u)

    # what the mixin calls on delivery_sim.Van / Van 側
    def log(self, t, event, *a, **k):
        self.events.append((round(t, 1), event))

    def hold(self):
        pass

    def roof(self):
        return (0.0, 0.0, 3.2)

    def car_pos(self):
        return (0.0, 0.0, 0.0)

    def _world(self, p):
        return tuple(p) if not hasattr(p, "x") else (p.x, p.y, p.z)

    def trip_plan(self, order):
        return ((order.id * 50.0, 30.0, 0.0), 30.0, 0.0)

    def _end_drone_work(self, t):
        pass

    def _next_stage(self, t):
        self.stages_done += 1

    def _say(self, t, key, event, **kw):
        self.events.append((round(t, 1), event))

    def fail_order(self, t, order, reason):
        self.events.append((round(t, 1), "failed:" + reason))

    def finish_order(self, t, order, pos):
        self.finished.append(order)
        self.pending.remove(order)


O = [types.SimpleNamespace(id=i, batch=1) for i in range(1, 6)]
us = [Unit(i + 1, lvl) for i, lvl in enumerate((95, 90, 85, 80))]
stops = [stop(DRONE, 100.0, [O[0], O[1]]), stop(FACE_TO_FACE, 300.0, [O[2]]),
         stop(DRONE, 500.0, [O[3], O[4]])]
v = Van(us, stops)
t, dt = 0.0, 0.1


def run(seconds, until=None, work=None):
    global t
    for _ in range(int(seconds / dt)):
        v.lmp_tick(t, dt)
        if work is not None:
            work(t, dt)
        t += dt
        if until is not None and until():
            return True
    return False


print("driving to the first drone stop: the first drone is loaded on the way")
parcel_early = []
def watch(*_):
    if v._loading and v._loading[0].parcel and v.anim.t["m"] < R.HOOK_DONE:
        parcel_early.append(round(v.anim.t["m"], 2))
run(60, until=lambda: v.loaded is not None, work=watch)
check("a drone is ready on the lift before arriving", v.loaded is not None and v.loaded[1] is O[0],
      "t=%.1f" % t)
check("the parcel reached the drone only when the hook animation ended", not parcel_early,
      str(parcel_early[:3]))
check("loading took the HTML's time (retrieve..presenter down, %.1f s)"
      % (R.WINDOWS["load"][2] - R.WINDOWS["load"][1]),
      abs(t - (R.WINDOWS["load"][2] - R.WINDOWS["load"][1])) < 0.5, "%.1f s" % t)

print("drone stop: launch both, don't wait")
v.state = "lmp_drone"
v.lmp_begin_drone(t)
launched = []
def hub(tt, d):
    v.lmp_drone_work(tt, d)
    for u in us:
        if u.status == "delivering" and u not in launched:
            launched.append(u)
run(300, until=lambda: v.stages_done == 1, work=hub)
check("two drones launched from the stop", len(launched) == 2, str([u.id for u in launched]))
check("the pod left the stop with both still out",
      v.stages_done == 1 and all(u.status == "delivering" for u in launched), "t=%.1f" % t)
check("they meet the pod at the next (face-to-face) stop",
      all(u.meet == stops[1].location for u in launched))
check("roof closed and lift down before leaving", v.anim.roof_closed() and v.anim.lift_down())
t_left = t

print("driving to the face-to-face stop: lift kept free for the drones")
v.stops.pop(0)
v.state, v.stages_done = "driving", 0
run(40)
check("no drone loaded while drones are due at the next stop", v.loaded is None and v._loading is None)

print("face-to-face stop, then the meeting point")
v.state = "lmp_direct"
v.lmp_begin_direct(t)
for u in launched:                       # they delivered and now circle overhead / 上空で待機
    u.flight.holding = True
landed_t = []
def f2f(tt, d):
    v.lmp_direct_work(tt, d)
    for u in launched:
        if u.flight is not None and u.flight.cleared is not None and u.status == "delivering":
            landed_t.append((u.id, round(tt, 1), v.anim.lift_up()))
            u.status = "docked"
            u.battery.level -= 30.0           # what the trip used / 飛行で消費
            v.lmp_on_drone_event(tt, u, "trip_done")
run(400, until=lambda: v.stages_done == 1, work=f2f)
check("the box-station parcel was handed over", O[2] in v.finished)
check("both drones landed, one at a time, on the raised lift",
      len(landed_t) == 2 and all(up for _, _, up in landed_t) and landed_t[1][1] - landed_t[0][1] > 5,
      str(landed_t))
check("landings came after the box station", all(x[1] > t_left for x in landed_t))
check("both drones are back on bays, the lift free",
      all(v.slots[v.lmp_units.index(u)] not in (None, R.LAUNCH) for u in launched)
      and v.drone is None, str(v.slots))
check("roof closed before driving on", v.anim.roof_closed() and v.anim.lift_down())

print("driving to the next drone stop: the next drone loads on the way")
v.stops.pop(0)
v.state, v.stages_done = "driving", 0
run(60, until=lambda: v.loaded is not None)
check("a drone is loaded with order #4 while driving", v.loaded is not None and v.loaded[1] is O[3],
      "t=%.1f" % t)
check("it is one that did not just fly", v.loaded[0] not in launched if v.loaded else False)

print("pod battery")
e0 = v.energy.level
fleet = DroneFleet([], types.SimpleNamespace(pads=[]), BatteryModel(), None, None,
                   pod_draw=v.sky.drones.pod_draw)
for u in us:
    u.battery.level = 50.0
v.packs = [50.0, None, 50.0, 50.0]
for _ in range(600):                     # 60 s of 4 drones + 3 packs charging / 充電 60 秒
    for u in us:
        u.battery.charge(0.1, fleet.bm.slow)
        v.energy.drain(0.1, fleet.bm.slow * fleet.pod_draw)
    v.pack_step(0.1)
check("charging 4 drones + 3 packs for a minute costs the pod ~1-3 %",
      1.0 < e0 - v.energy.level < 3.0, "%.2f %%" % (e0 - v.energy.level))

print("meeting at the drone stop itself: all launches go first")
us2 = [Unit(i + 11, 95.0) for i in range(4)]
v = Van(us2, [stop(DRONE, 100.0, [O[0], O[1], O[3]])])      # no face-to-face next / 次は対面なし
v.state = "lmp_drone"
v.lmp_begin_drone(t)
out = []
def hub2(tt, d):
    v.lmp_drone_work(tt, d)
    for u in us2:
        if u.status == "delivering" and u not in out:
            out.append(u)
run(400, until=lambda: len(out) == 3, work=hub2)
check("three launches before any drone is back", len(out) == 3 and all(u.meet == (100.0, 0.0, 0.0) for u in out),
      "%d launched" % len(out))

print("the drone's mission: deliver, meet, hold, land")
from delivery.drone import Mission, HOLD, DONE
f = Mission((0, 0, 3.2), (120, 40, 0.0), 30.0, (300, 0, 0.0), 25.0)
ev, t_hold, pad = [], None, (301.0, 1.0, 3.2)
for k in range(4000):
    for e, _ in f.update(0.1):
        ev.append((e, round(k * 0.1, 1)))
    if f.state == HOLD and t_hold is None:
        t_hold = k * 0.1
    if t_hold is not None and k * 0.1 > t_hold + 20 and f.land_fn is None:
        f.clear(lambda: pad)               # the pod is ready 20 s later / 20 秒後に許可
    if f.done:
        break
names = [e for e, _ in ev]
check("delivered, then at the meeting point, then landed", names == ["delivered", "at_meet", "landed"],
      str(ev))
check("it held overhead until cleared (~20 s)", 19.0 < f.hold_s < 21.5, "%.1f s" % f.hold_s)
check("landed on the pad", f.done and max(abs(a - b) for a, b in zip(f.pos, pad)) < 1e-6, str(f.pos))
bm = BatteryModel()
far = bm.mission_cost((0, 0, 3), (100, 0, 0), (1500, 0, 0), 30, 30)
near = bm.mission_cost((0, 0, 3), (100, 0, 0), (100, 0, 0), 30, 30)
check("a farther meeting point costs more battery", far > near, "%.1f vs %.1f %%" % (far, near))

print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
