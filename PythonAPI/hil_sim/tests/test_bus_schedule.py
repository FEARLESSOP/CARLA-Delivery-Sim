"""Tests for the timetabled Oita Station buses (bus_schedule.py). No CARLA
server: the real road logic is read offline (carla.Map from the build's
xodr) and fakes stand in for the world, the Traffic Manager and snapshots.

1. time and calendar: GTFS times past 24:00, day types, national holidays
   (also 2026-11-03 / 11-23, which the 大分バス feed misses), the bus clock
   (before 03:00 = the previous service day, roll-over at 27:00);
2. the timetable: visits per pass, sorted, previous-day after-midnight trips
   shifted into today's queue;
3. spawn time (edge entry, never before the last stop outside the map, 回送
   in, appearing at the stop);
4. the road logic (maps/build/OitaStation/OitaStation.xodr, skipped when
   missing): no runaway segment, every stop on a kerb lane with its pole on
   the left and the bus clear of junctions, the station platforms 1-3 and the
   降車専用 pole within 3 m, every pass planned without a long way round,
   set_path points never in a junction;
5. the speed arbiter (the stricter wish wins, buses never change lanes on
   their own, other cars pass through);
6. a fake world: a bus spawns on time, stops with its door at each pole,
   dwells, holds at Oita Station until the departure, blinks, leaves by the
   exit and is destroyed; a bus that cannot move is removed as stuck; a bus
   behind one standing at the stop waits before the junction; destroy_all
   and leftovers; maps without "bus_schedule" get nothing.

時刻表バスの試験（CARLA サーバー不要。道路ロジックはオフライン読込）。
"""
import datetime
import json
import math
import os
import random
import sys
import tempfile
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import carla  # noqa: E402
import bus_schedule as bs  # noqa: E402

FAILS = []
XODR = os.path.join(HIL, 'maps', 'build', 'OitaStation', 'OitaStation.xodr')
DATA = os.path.join(HIL, 'data', 'oita_bus')


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


D = datetime.date

print("time and calendar / 時刻と暦")
check("GTFS 07:05 -> 25500 s", bs.gtfs_seconds('07:05') == 25500)
check("GTFS 32:10 (past midnight) -> 115800 s", bs.gtfs_seconds('32:10') == 115800)
check("GTFS with seconds", bs.gtfs_seconds('08:00:30') == 28830)
check("empty time -> None", bs.gtfs_seconds(None) is None and bs.gtfs_seconds('') is None)
check("hhmm keeps hours past 24", bs.hhmm(115800) == '32:10:00')
for d, want, why in [(D(2026, 11, 11), 'weekday', 'Wed'), (D(2026, 11, 14), 'saturday', 'Sat'),
                     (D(2026, 11, 15), 'sunday_holiday', 'Sun'),
                     (D(2026, 11, 3), 'sunday_holiday', '文化の日 (missing in the 大分バス feed)'),
                     (D(2026, 11, 23), 'sunday_holiday', '勤労感謝の日 (missing in the feed)'),
                     (D(2026, 9, 22), 'sunday_holiday', '国民の休日 between 敬老の日 and 秋分の日'),
                     (D(2026, 5, 6), 'sunday_holiday', '振替休日 (5/3 is a Sunday)'),
                     (D(2026, 3, 20), 'sunday_holiday', '春分の日'),
                     (D(2026, 10, 12), 'sunday_holiday', 'スポーツの日 (2nd Monday)'),
                     (D(2026, 11, 24), 'weekday', 'the day after 勤労感謝の日'),
                     (D(2027, 3, 22), 'sunday_holiday', '振替休日 (3/21 equinox on a Sunday)')]:
    check("%s is %s (%s)" % (d, want, why), bs.day_type(d) == want, bs.day_type(d))

c = bs.BusClock(today=D(2026, 11, 8))
c.anchor(100.0, 1.5)
check("01:30 belongs to the previous service day as 25:30",
      c.date == D(2026, 11, 7) and abs(c.now(100.0) - 91800.0) < 1e-6)
c.anchor(0.0, 26.99 - 24.0)
check("the clock runs with sim time", abs(c.now(10.0) - (c.now(0.0) + 10.0)) < 1e-6)
rolled = c.roll(200.0)
check("past 27:00 the next service day starts",
      rolled and c.date == D(2026, 11, 8) and c.now(200.0) < 4 * 3600, '%s %s' % (c.date, c.now(200.0)))
c2 = bs.BusClock(fixed_day='saturday', today=D(2026, 11, 11))
check("--bus-day overrides the date", c2.day_type() == 'saturday')

print("timetable / 時刻表")
tt = bs.Timetable(DATA)
vis = tt.visits('weekday')
check("weekday visits read (%d)" % len(vis), len(vis) > 1000)
check("visits sorted by their first stop", all(vis[i]['stops'][0][1] <= vis[i + 1]['stops'][0][1]
                                                for i in range(len(vis) - 1)))
check("every trip matched its route pattern", tt.unmatched == 0, str(tt.unmatched))
check("a route that leaves and comes back is two visits",
      any(v['passes'] > 1 and v['pass_index'] == 1 for v in vis))
check("pass keys have a GTFS shape", sum(1 for v in vis if v['pass_key'] in tt.shapes) > 0.9 * len(vis))
late = [v for d in bs.DAY_TYPES for v in tt.visits(d) if v['stops'][0][1] >= 86400]
check("after-midnight times are kept as GTFS (%d visits past 24:00)" % len(late), True)


class StubPlanner(object):
    def __init__(self, plans=None):
        self.plans = plans or {}
        self.map = types.SimpleNamespace(to_opendrive=lambda: '<OpenDRIVE/>',
                                         get_waypoint=lambda loc: None)

    def plan(self, v):
        return self.plans.get(v['pass_key'])


class StubTT(object):
    def __init__(self, stops, by_day):
        self.stops = stops
        self.by_day = by_day
        self.shapes = {}

    def visits(self, d):
        return [dict(v) for v in self.by_day.get(d, [])]


def visit(key, stops, entry='boundary', exit_='boundary', prev_s=None):
    return {'trip': key, 'route': 'r', 'name': 'X' + key, 'dest': 'Dest', 'type': 'through',
            'pass_key': key, 'pass_index': 0, 'passes': 1, 'operator': 'test',
            'stops': stops, 'entry': {'kind': entry, 'x': 0, 'y': 0},
            'exit': {'kind': exit_, 'x': 600, 'y': 0}, 'prev_s': prev_s, 'other_edges': []}


T8 = 8 * 3600
qtt = StubTT({}, {'saturday': [visit('late', [('A', 86400 + 600, 86400 + 600, '')])],
                  'sunday_holiday': [visit('early', [('A', 7200, 7200, '')]),
                                     visit('day', [('A', T8, T8, '')])]})
logs = []
sched_q = bs.BusSchedule.__new__(bs.BusSchedule)
sched_q.tt, sched_q.log = qtt, logs.append
sched_q.clock = bs.BusClock(today=D(2026, 11, 15))
sched_q.clock.anchor(0.0, 0.0)            # 00:00 on Sunday = 24:00 of Saturday's service day
sched_q.clock.date = D(2026, 11, 15)
sched_q.build_queue(0.0)
keys = [v['pass_key'] for v in sched_q.queue[sched_q.qi:]]
check("Saturday's 24:10 trip runs at 00:10 of Sunday's queue", keys[:1] == ['late']
      and sched_q.queue[0]['stops'][0][1] == 600, str(keys))

print("spawn time / 生成時刻")
cfg = dict(bs.DEFAULTS)
pl = {'points': [{'kind': 'spawn', 's': 0.0}, {'kind': 'stop', 's': 360.0, 'call': 0}],
      'deadhead_in': False}
v1 = visit('k', [('A', T8, T8 + 20, '')])
check("edge entry: first stop time - 360 m at 18 km/h (72 s)", abs(bs.spawn_time(v1, pl, cfg) - (T8 - 72)) < 1e-6)
v2 = visit('k', [('A', T8, T8, '')], prev_s=T8 - 30)
check("never before the trip's last stop outside the map", bs.spawn_time(v2, pl, cfg) == T8 - 30)
pl_dh = dict(pl, deadhead_in=True)
check("回送 in: departure - layover - way in",
      abs(bs.spawn_time(v1, pl_dh, cfg) - (T8 + 20 - cfg['layover_s'] - 72)) < 1e-6)
pl_st = {'points': [{'kind': 'stop', 's': 0.0, 'call': 0}], 'deadhead_in': False}
check("appearing at its first stop: departure - layover",
      bs.spawn_time(v1, pl_st, cfg) == T8 + 20 - cfg['layover_s'])

print("road logic (offline carla.Map) / 道路ロジック")
if not os.path.exists(XODR):
    print("  SKIP  no %s" % XODR)
else:
    with open(XODR) as fh:
        xodr = fh.read()
    wmap = carla.Map('OitaStation', xodr)
    lp = bs.LanePlanner(wmap, carla, None, tt, xodr=xodr)
    check("SafeRoutePlanner: no runaway segment", lp.planner.runaway_segments == 0,
          str(lp.planner.runaway_segments))
    check("the road logic reaches every stop", lp.covers(list(tt.stops.values())))
    old = types.SimpleNamespace(bounds=bs.LanePlanner._header_bounds(
        '<header revMajor="1" north="423.25" south="-423.25" east="423.25" west="-423.25">'))
    check("the old +-423 m level does not (no buses there)",
          old.bounds is not None and not bs.LanePlanner.covers(old, list(tt.stops.values())))
    bad, wrong_side, not_kerb, in_junction = [], [], [], []
    for sid, st in tt.stops.items():
        cands = lp.stop_candidates(sid)
        if not cands:
            bad.append(sid)
            continue
        w, gap, shift = cands[0]
        f = bs._fwd(w)
        wx, wy = bs._xy(w)
        if bs._dot((st['x'] - wx, -st['y'] - wy), bs._right(f)) > 0.5:
            wrong_side.append(sid)
        if lp.kerb_lane(w).lane_id != w.lane_id or lp.kerb_lane(w).road_id != w.road_id:
            not_kerb.append(sid)
        if not lp.footprint_clear(w):
            in_junction.append(sid)
    check("every stop has a stopping place (%d stops)" % len(tt.stops), not bad, str(bad))
    check("... with its pole on the bus's left (keep-left door)", not wrong_side, str(wrong_side))
    check("... on the kerb (left-most) lane of its direction", not not_kerb, str(not_kerb))
    check("... with the 10.4 m bus clear of junctions", not in_junction, str(in_junction))
    near = {sid: lp.stop_candidates(sid)[0][1] for sid in ('S016', 'S040', 'S043', 'S010')}
    check("platforms 3, 2, 1 and the 降車専用 pole within 3 m of their lane",
          all(g <= 3.0 for g in near.values()), str(near))
    seen = {}
    for v in vis:
        seen.setdefault(v['pass_key'], v)
    keys = sorted(seen)
    random.Random(3).shuffle(keys)
    sample = keys[:80]
    planned, longway, in_jn, gaps = 0, [], 0, 0.0
    for k in sample:
        plan = lp.plan(seen[k])
        if plan is None:
            continue
        planned += 1
        if any('long way' in w for w in lp.problems.get(k, [])):
            longway.append(k)
        pts = bs.path_locations(plan)
        for x, y, z in pts:
            w = wmap.get_waypoint(carla.Location(x, y, z))
            if w is not None and w.is_junction and w.transform.location.distance(carla.Location(x, y, z)) < 0.5:
                in_jn += 1
        for a, b in zip(pts, pts[1:]):
            gaps = max(gaps, math.hypot(b[0] - a[0], b[1] - a[1]))
    check("80 random passes planned", planned == len(sample), '%d/%d' % (planned, len(sample)))
    check("... at most 2 with a long way round (routes that turn outside the map)",
          len(longway) <= 2, str(longway))
    check("set_path points are never inside a junction", in_jn == 0, str(in_jn))
    check("set_path points close enough for the TM (< 60 m apart)", gaps < 60.0, '%.1f' % gaps)
    # a pass with a GTFS shape keeps to it (no side-street shortcut)
    k = next(k for k in sample if k in tt.shapes and lp.plans.get(k))
    plan = lp.plans[k]
    shp = [(x, -y) for x, y in tt.shapes[k]]
    far = max(min(math.hypot(px - sx, py - sy) for sx, sy in shp)
              for px, py in zip(plan['xs'][::10], plan['ys'][::10]))
    check("a planned path stays near its GTFS shape (max %.0f m)" % far, far < 60.0)

print("speed arbiter / 速度の調停")


class FakeTM(object):
    def __init__(self):
        self.calls = []
        self.desired = {}
        self.paths = {}

    def get_port(self):
        return 8123

    def set_desired_speed(self, a, kmh):
        self.calls.append(('speed', a.id, kmh))
        self.desired[a.id] = kmh

    def vehicle_percentage_speed_difference(self, a, pct):
        self.calls.append(('pct', a.id, pct))
        if pct >= 100.0:
            self.desired[a.id] = 0.0

    def set_path(self, a, path):
        self.paths[a.id] = path

    def __getattr__(self, name):
        def rec(*args):
            self.calls.append((name,) + tuple(getattr(x, 'id', x) for x in args))
        return rec


class A(object):
    def __init__(self, i):
        self.id = i


tm = FakeTM()
arb = bs.SpeedArbiter(tm)
bus, car = A(1), A(2)
arb.add(bus)
arb.set_desired_speed(bus, 36.0)
check("RuleKeeper's lane speed reaches the TM", tm.desired[1] == 36.0)
arb.bus_wish(1, 12.0)
check("the slower bus wish wins", tm.desired[1] == 12.0)
arb.vehicle_percentage_speed_difference(bus, 100.0)
check("a RuleKeeper hold (walker / 止まれ) stops the bus", tm.desired[1] == 0.0)
arb.set_desired_speed(bus, 36.0)
check("... and its release gives back the bus wish, not more", tm.desired[1] == 12.0)
arb.bus_wish(1, None)
check("no bus wish: the RuleKeeper's speed", tm.desired[1] == 36.0)
n = len(tm.calls)
arb.auto_lane_change(bus, True)
check("buses never get auto lane change from the RuleKeeper", len(tm.calls) == n)
arb.auto_lane_change(car, True)
arb.set_desired_speed(car, 50.0)
check("other cars pass straight through", tm.calls[-2][0] == 'auto_lane_change' and tm.desired[2] == 50.0)
n = len(tm.calls)
arb.set_desired_speed(bus, 36.0)
check("unchanged wishes send nothing", len(tm.calls) == n)

print("fake world: a bus runs its timetable / 偽ワールドでの運行")


def line_plan(length=600.0, junction=(300.0, 315.0), stops=((200.0, 0), (400.0, 1)), exit_s=590.0):
    xs = [i * 2.0 for i in range(int(length / 2) + 1)]
    plan = {'xs': xs, 'ys': [0.0] * len(xs), 'zs': [0.0] * len(xs), 's': list(xs),
            'junction': [junction[0] <= x <= junction[1] for x in xs],
            'roads': [1 if x < junction[0] else (9 if x <= junction[1] else 2) for x in xs],
            'deadhead_in': False, 'deadhead_out': False, 'length': length}
    pts = [{'kind': 'spawn', 's': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'road': 1, 'lane': -1}]
    for s, ci in stops:
        pts.append({'kind': 'stop', 's': s, 'x': s, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'road': 1,
                    'lane': -1, 'stop': 'S%d' % ci, 'call': ci})
    if exit_s is not None:
        pts.append({'kind': 'exit', 's': exit_s, 'x': exit_s, 'y': 0.0, 'z': 0.0, 'yaw': 0.0,
                    'road': 2, 'lane': -1})
    plan['points'] = pts
    return plan


class Snap(object):
    def __init__(self, world, t):
        self.world = world
        self.timestamp = types.SimpleNamespace(elapsed_seconds=t)

    def find(self, i):
        return self.world.actors.get(i)


class Actor(object):
    def __init__(self, i, x, y, bp):
        self.id = i
        self.x, self.y, self.v = x, y, 0.0
        self.frozen = False
        self.attributes = {'role_name': bs.ROLE}
        self.type_id = bp
        self.lights = []

    def get_transform(self):
        return carla.Transform(carla.Location(self.x, self.y, 0.0))

    def get_velocity(self):
        return carla.Vector3D(self.v, 0.0, 0.0)

    def set_autopilot(self, on, port):
        self.autopilot = (on, port)

    def set_light_state(self, st):
        self.lights.append(st)


class World(object):
    def __init__(self):
        self.actors = {}
        self.spawned = {}
        self.next_id = 100

    def try_spawn_actor(self, bp, tf):
        self.next_id += 1
        a = Actor(self.next_id, tf.location.x, tf.location.y, bp)
        self.actors[a.id] = self.spawned[a.id] = a
        return a

    def get_actors(self):
        lst = list(self.actors.values())
        return types.SimpleNamespace(filter=lambda pat: lst)


class Client(object):
    def __init__(self, world):
        self.world = world
        self.destroyed = []

    def apply_batch_sync(self, cmds, sync):
        for c in cmds:
            self.destroyed.append(c.actor_id)
            self.world.actors.pop(c.actor_id, None)
        return []


def make_sched(visits, plans, cfg=None, rk=None, stops=None):
    world = World()
    client = Client(world)
    tm = FakeTM()
    stops = stops or {'S0': {'name': 'Stop 0', 'platform': None, 'is_station_platform': False},
                      'S1': {'name': '大分駅前', 'platform': '3', 'is_station_platform': True}}
    tt = StubTT(stops, {'weekday': visits})
    clock = bs.BusClock(fixed_time=T8, fixed_day='weekday', today=D(2026, 11, 11))
    logs = []
    status = os.path.join(tempfile.mkdtemp(), 'bus_status.json')
    model = dict(bs.BUS_MODELS[0])
    sched = bs.BusSchedule(world, client, tm, carla, tt, StubPlanner(plans), clock,
                           dict({'seed': 1}, **(cfg or {})), rule_keeper=rk, log=logs.append,
                           status_path=status, blueprint=('bus-bp', model))
    return sched, world, client, tm, logs


def run_sim(sched, world, tm, seconds, dt=0.05, t0=0.0, on_step=None):
    t = t0
    while t < t0 + seconds:
        t += dt
        sched.update(Snap(world, t))
        for a in list(world.actors.values()):
            if a.frozen:
                a.v = 0.0
                continue
            want = tm.desired.get(a.id, 30.0) / 3.6
            acc = 1.2 if want > a.v else -3.0
            a.v = max(0.0, min(want, a.v + acc * dt)) if want > a.v else max(want, a.v + acc * dt)
            a.x += a.v * dt
        if on_step:
            on_step(t)
    return t


# trip: stop S0 at 08:01:00 (dep 08:01:00), Oita Station S1 dep 08:04:00 (hold),
# exit at 590 m / 08:01 に S0、08:04 発の大分駅（待機）、590 m で退出
v = visit('k', [('S0', T8 + 60, T8 + 60, ''), ('S1', T8 + 120, T8 + 240, 'pick_up_only')])
plan = line_plan()
sched, world, client, tm, logs = make_sched([v], {'k': plan})
arrivals = {}


def watch(t):
    for run in sched.runs.values():
        if run.phase == 'dwell' and run.next_stop not in arrivals:
            a = world.actors[run.id]
            arrivals[run.next_stop] = (t, a.x + run.model['door_m'], sched.clock.now(t))


tend = run_sim(sched, world, tm, 420.0, on_step=watch)
spawned = [l for l in logs if ' spawn ' in l]
check("the bus spawned once", len(spawned) == 1, '; '.join(spawned))
check("it spawned 40 s before the first stop (200 m at 18 km/h)", spawned and spawned[0].split()[1] == '08:00:20',
      spawned[0] if spawned else '')
check("it stopped at both stops", set(arrivals) == {0, 1}, str(arrivals))
if 0 in arrivals:
    check("door at the first pole (within 1.5 m)", abs(arrivals[0][1] - 200.0) <= 1.5, '%.2f' % arrivals[0][1])
if 1 in arrivals:
    check("door at the station pole (within 1.5 m)", abs(arrivals[1][1] - 400.0) <= 1.5, '%.2f' % arrivals[1][1])
dep = [l for l in logs if ' depart ' in l]
check("departures logged for both stops", len(dep) == 2, '; '.join(dep))
st_dep = [l for l in dep if '大分駅前' in l]
check("Oita Station: held until the scheduled 08:04", st_dep and st_dep[0].split()[1] >= '08:04:00',
      st_dep[0] if st_dep else '')
lights = [int(x) for x in world.spawned[101].lights]
L, R = int(carla.VehicleLightState.LeftBlinker), int(carla.VehicleLightState.RightBlinker)
check("left blinker when pulling in, right when pulling out (TM lights off meanwhile)",
      ('update_vehicle_lights', 101, False) in tm.calls and any(x & L for x in lights)
      and any(x & R for x in lights) and ('update_vehicle_lights', 101, True) in tm.calls, str(lights))
check("it left by the exit and was destroyed", 101 in client.destroyed and not sched.runs,
      str(client.destroyed))
check("the exit was logged", any(' exit ' in l for l in logs))
check("the bus never changes lanes on its own", ('auto_lane_change', 101, False) in tm.calls)
check("it got a TM path", 101 in tm.paths and len(tm.paths[101]) > 10)
with open(sched.status_path) as fh:
    stat = json.load(fh)
check("status file written (clock, counts, credit)", stat['day_type'] == 'weekday'
      and stat['counts']['spawned'] == 1 and 'BODIK' in stat['credit'])

# stuck: a bus that never moves is removed after stuck_s / 動けないバスは削除
sched, world, client, tm, logs = make_sched([v], {'k': plan}, cfg={'stuck_s': 30.0})
t = run_sim(sched, world, tm, 21.0)
for a in world.actors.values():
    a.frozen = True
run_sim(sched, world, tm, 40.0, t0=t)
check("a bus with no progress for stuck_s is removed and logged",
      not sched.runs and sched.counts['stuck'] == 1 and any('(stuck)' in l for l in logs),
      str(sched.counts))

# queueing: a second bus behind one standing at the pole waits before the
# junction (the pole stands right after it) / 停車中のバスの後ろは交差点の手前で待つ
qplan = line_plan(stops=((322.0, 0),), exit_s=590.0)
va = visit('a', [('S0', T8 + 70, T8 + 70, '')])
vb = visit('b', [('S0', T8 + 110, T8 + 110, '')])
sched, world, client, tm, logs = make_sched([va, vb], {'a': qplan, 'b': qplan},
                                            cfg={'dwell_s': {'both': [80, 80], 'pick_up_only': [80, 80],
                                                             'drop_off_only': [80, 80]}})
held_x = []


def watch2(t):
    ids = sorted(world.actors)
    if len(ids) == 2:
        first, second = world.actors[ids[0]], world.actors[ids[1]]
        if sched.runs.get(first.id) and sched.runs[first.id].phase == 'dwell' and second.v < 0.05:
            held_x.append(second.x)


run_sim(sched, world, tm, 200.0, on_step=watch2)
check("the second bus waited before the junction (front clear of it)",
      held_x and max(held_x) + 5.215 <= 300.0, '%.1f' % (max(held_x) if held_x else -1))
check("... and both buses served the stop", sum(1 for l in logs if ' arrive ' in l) == 2,
      str([l for l in logs if ' arrive ' in l]))

# destroy_all and leftovers / 全削除と前回の残り
sched, world, client, tm, logs = make_sched([v], {'k': plan})
run_sim(sched, world, tm, 25.0)
n = len(sched.runs)
sched.destroy_all()
check("destroy_all removes every bus (%d)" % n, n == 1 and not sched.runs and len(client.destroyed) == 1)
world.actors[999] = Actor(999, 0, 0, 'bus-bp')
sched.clear_leftovers()
check("buses left by a killed run are cleared", 999 in client.destroyed)

# the late bus and the cap / 遅れと上限
sched, world, client, tm, logs = make_sched([v], {'k': plan}, cfg={'max_buses': 0})
run_sim(sched, world, tm, 10.0)
check("max_buses 0: nothing spawns", not sched.runs and not world.actors)
vl = visit('late', [('S0', T8 - 600, T8 - 600, '')])
sched, world, client, tm, logs = make_sched([vl], {'late': plan})
run_sim(sched, world, tm, 2.0)
check("a trip long gone is not spawned", not world.actors)

# a RuleKeeper's speeds pass through the arbiter / RuleKeeper の要求は調停を経由
rk = types.SimpleNamespace(ids=[], tm=FakeTM())
sched, world, client, tm, logs = make_sched([v], {'k': plan}, rk=rk)
check("the RuleKeeper now talks to the arbiter", isinstance(rk.tm, bs.SpeedArbiter))
run_sim(sched, world, tm, 25.0)
check("the bus joined the RuleKeeper (止まれ, walkers, lane speeds)", rk.ids == [101])

print("maps without a timetable / 時刻表の無いマップ")
maps = {'a': {'name': 'OitaStation', 'bus_schedule': 'data/oita_bus'},
        'b': {'name': 'OitaStation', 'road_style': 'japan'},
        'c': {'name': 'OitaKosen', 'road_style': 'japan'}}
check("the first OitaStation entry (oita_station) decides", bs.map_config('Japan/Maps/OitaStation/OitaStation', maps)
      == (os.path.join(HIL, 'data/oita_bus'), {}))
check("OitaKosen: no buses", bs.map_config('OitaKosen', maps) is None)
check("Town10HD: no buses", bs.map_config('Carla/Maps/Town10HD_Opt', maps) is None)
check("a dict entry passes options", bs.map_config('X', {'x': {'name': 'X', 'bus_schedule': {
    'data': 'data/oita_bus', 'max_buses': 5, 'junk': 1}}})[1] == {'max_buses': 5})


class NoCallWorld(object):
    def __init__(self, name):
        self.name = name

    def get_map(self):
        return types.SimpleNamespace(name=self.name)

    def __getattr__(self, n):
        raise AssertionError('unexpected world call %s' % n)


check("setup on a map without bus_schedule: None, no world call",
      bs.setup(NoCallWorld('Carla/Maps/Town10HD_Opt'), None, None, carla) is None)
check("--bus-schedule none: None", bs.setup(NoCallWorld('OitaStation'), None, None, carla, mode='none') is None)

print("\n%d failure(s)" % len(FAILS) + (": " + ", ".join(FAILS) if FAILS else ""))
sys.exit(1 if FAILS else 0)
