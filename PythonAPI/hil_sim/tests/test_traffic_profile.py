"""Tests for the country traffic profiles (traffic_profile.py,
traffic_profiles.json, generate_traffic.setup_traffic_profile). No CARLA
server: fakes stand in for the world, map, Traffic Manager and snapshots.

1. selection: Japanese maps (road_style japan / keep_left in Japan) get
   "japan"; Pasir Ris (keep-left Singapore), the circuits and CARLA's towns
   get nothing; --traffic-profile none / a name; a maps.json override;
2. on any other map setup_traffic_profile makes NO Traffic Manager call and
   returns no helper (traffic exactly as before);
3. on a Japanese map: the static settings per vehicle, blinkers on;
4. xodr lane speeds (m/s -> km/h, sections, units);
5. RuleKeeper: lane speed x factor (cyclists capped, fallback), no lane
   change near a junction, stop for a walker on the road ahead, release when
   the walker has gone, timeout + cooldown, walkers off the road ignored,
   destroyed cars dropped;
6. 止まれ: stop lines from the 206 landmarks, a car slows, stops with its
   front at the line, stands 2 s, goes; opposite stops first come first
   served (no deadlock); waits for a priority-road car; a blocked junction
   releases after max_hold_s; non-Japanese maps make no landmark query.

国別交通プロファイルの試験（CARLA サーバー不要）。
"""
import os
import sys
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import traffic_profile as tp  # noqa: E402
import generate_traffic as gt  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


# ---------------------------------------------------------------- fakes
class V3(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Rot(object):
    def __init__(self, yaw=0.0):
        self.yaw = yaw


class Tf(object):
    def __init__(self, x, y, yaw=0.0):
        self.location = V3(x, y)
        self.rotation = Rot(yaw)


class ActSnap(object):
    def __init__(self, tf, vel):
        self.tf, self.vel = tf, vel

    def get_transform(self):
        return self.tf

    def get_velocity(self):
        return self.vel


class Stamp(object):
    def __init__(self, t):
        self.elapsed_seconds = t


class Snapshot(object):
    def __init__(self, t, actors):
        self.timestamp = Stamp(t)
        self.actors = actors      # id -> ActSnap

    def find(self, aid):
        return self.actors.get(aid)


class TM(object):
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)

        def rec(*a):
            self.calls.append((name,) + a)
        return rec

    def named(self, name):
        return [c for c in self.calls if c[0] == name]


class Actor(object):
    def __init__(self, aid, base_type='car', type_id='vehicle.x.y'):
        self.id = aid
        self.attributes = {'base_type': base_type}
        self.type_id = type_id

    def __repr__(self):
        return 'A%d' % self.id


class ActorList(list):
    def filter(self, pattern):
        pre = pattern.rstrip('*')
        return ActorList(a for a in self if a.type_id.startswith(pre))


class WP(object):
    def __init__(self, road=1, section=0, lane=-1, junction=False, ahead=()):
        self.road_id, self.section_id, self.lane_id = road, section, lane
        self.is_junction = junction
        self.ahead = ahead      # list of (distance, is_junction)

    def next(self, d):
        return [WP(junction=j) for (dist, j) in self.ahead if abs(dist - d) < 1e-6]


class Map(object):
    """Road along x; |y| < 7 is carriageway. The lane waypoint comes from a
    function of the location so tests can place junctions."""

    def __init__(self, name, wp_fn=None):
        self.name = name
        self.wp_fn = wp_fn or (lambda loc: WP())

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        if not project_to_road and abs(loc.y) >= 7.0:
            return None
        return self.wp_fn(loc)

    def to_opendrive(self):
        return XODR


class World(object):
    def __init__(self, wmap, actors):
        self.map = wmap
        self.actors = ActorList(actors)

    def get_map(self):
        return self.map

    def get_actors(self, ids=None):
        if ids is None:
            return self.actors
        return ActorList(a for a in self.actors if a.id in ids)

    def get_actor(self, aid):
        for a in self.actors:
            if a.id == aid:
                return a
        return None


XODR = """<OpenDRIVE><header/>
<road id="1" junction="-1"><lanes>
 <laneSection s="0"><right><lane id="-1" type="driving"><speed sOffset="0" max="11.11"/></lane>
                          <lane id="-2" type="driving"><speed sOffset="0" max="40" unit="km/h"/></lane></right></laneSection>
 <laneSection s="50"><right><lane id="-1" type="driving"><speed sOffset="0" max="8.33"/></lane></right></laneSection>
</lanes></road>
<road id="7" junction="3"><lanes><laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection></lanes></road>
</OpenDRIVE>"""

# ---------------------------------------------------------------- 1. selection
print("1. selection / 選択")
for full, want in (("Japan/Maps/OitaKosen/OitaKosen", "japan"),
                   ("OitaKosen", "japan"),
                   ("Japan/Maps/OitaStation/OitaStation", "japan"),
                   ("Singapore/Maps/PasirRis/PasirRis", None),
                   ("Circuits/Maps/MarinaBay/MarinaBay", None),
                   ("Carla/Maps/Town10HD_Opt", None),
                   ("Carla/Maps/Town02", None),
                   ("", None)):
    name, prof = tp.select_profile(full)
    check("%s -> %s" % (full or "<empty>", want), name == want and (prof is None) == (want is None),
          "got %s" % name)
check("force none on OitaKosen", tp.select_profile("OitaKosen", force="none") == (None, None))
check("force japan on Town02", tp.select_profile("Town02", force="japan")[0] == "japan")
check("force unknown -> default", tp.select_profile("OitaKosen", force="xx") == (None, None))
maps = {"a": {"name": "OitaKosen", "road_style": "japan", "traffic_profile": "none"},
        "b": {"name": "Foo", "package": "Japan", "keep_left": False},
        "c": {"name": "Bar", "package": "Japan", "keep_left": True}}
check("maps.json traffic_profile none overrides road_style",
      tp.select_profile("OitaKosen", maps=maps)[0] is None)
check("Japan package without keep_left -> default", tp.select_profile("Foo", maps=maps)[0] is None)
check("Japan package + keep_left -> japan", tp.select_profile("Bar", maps=maps)[0] == "japan")
check("missing profiles file -> default",
      tp.select_profile("OitaKosen", maps=maps, config={}) == (None, None))

# ---------------------------------------------------------------- 2. other maps untouched
print("2. other maps: no Traffic Manager call / 他のマップは従来どおり")
for mname in ("Carla/Maps/Town03", "Singapore/Maps/PasirRis/PasirRis", "Circuits/Maps/MarinaBay/MarinaBay"):
    tm = TM()
    world = World(Map(mname), [Actor(1), Actor(2, 'bicycle')])
    helper = gt.setup_traffic_profile(world, tm, [1, 2], 'auto')
    check("%s: no helper, 0 TM calls" % mname, helper is None and tm.calls == [], str(tm.calls))
tm = TM()
helper = gt.setup_traffic_profile(World(Map("Japan/Maps/OitaKosen/OitaKosen"), [Actor(1)]),
                                  tm, [1], 'none')
check("OitaKosen with --traffic-profile none: 0 TM calls", helper is None and tm.calls == [])

# ---------------------------------------------------------------- 3. Japanese map
print("3. Japanese map: static settings / 日本のマップ")
tm = TM()
world = World(Map("Japan/Maps/OitaKosen/OitaKosen"), [Actor(1), Actor(2, 'bicycle')])
helper = gt.setup_traffic_profile(world, tm, [1, 2], 'auto')
check("helper made", isinstance(helper, tp.RuleKeeper))
for meth in ('ignore_lights_percentage', 'ignore_signs_percentage', 'ignore_walkers_percentage',
             'ignore_vehicles_percentage'):
    calls = tm.named(meth)
    check("%s 0 for both cars" % meth, len(calls) == 2 and all(c[2] == 0.0 for c in calls))
check("following gap 3.5 m", [c[2] for c in tm.named('distance_to_leading_vehicle')] == [3.5, 3.5])
check("keep-slow-lane rule 0 (would push to the right = overtaking lane)",
      [c[2] for c in tm.named('keep_slow_lane_rule_percentage')] == [0.0, 0.0])
check("blinkers / lights by TM", [c[2] for c in tm.named('update_vehicle_lights')] == [True, True])
check("cyclist known to the helper", helper.cyclists == {2})
tm = TM()
helper = gt.setup_traffic_profile(world, tm, [1, 2], 'auto', run_helper=False)
check("--driver-profile: statics only, no helper", helper is None and tm.named('ignore_lights_percentage'))

# ---------------------------------------------------------------- 4. xodr speeds
print("4. xodr lane speeds / 車線の最高速度")
by_sec, by_lane = tp.lane_speeds_from_xodr(XODR)
check("m/s -> km/h", abs(by_sec[(1, 0, -1)] - 40.0) < 0.1)
check("unit km/h", abs(by_sec[(1, 0, -2)] - 40.0) < 1e-6)
check("second section", abs(by_sec[(1, 1, -1)] - 30.0) < 0.1)
check("lane without speed absent", (7, 0, -1) not in by_sec)
check("broken xodr -> empty", tp.lane_speeds_from_xodr("<nope") == ({}, {}))

# ---------------------------------------------------------------- 5. RuleKeeper
print("5. RuleKeeper / 補助")
profile = tp.select_profile("OitaKosen")[1]


def make(wp_fn=None, walkers=()):
    acts = [Actor(1), Actor(2, 'bicycle'), Actor(3)] + \
           [Actor(100 + i, type_id='walker.pedestrian.0001') for i in range(len(walkers))]
    world = World(Map("Japan/Maps/OitaKosen/OitaKosen", wp_fn), acts)
    tm = TM()
    rk = tp.RuleKeeper(world, world.map, tm, [1, 2, 3], profile, cyclist_ids=[2])
    return rk, tm


def snap(t, cars, walkers=()):
    d = {}
    for vid, (x, y, yaw, v) in cars.items():
        d[vid] = ActSnap(Tf(x, y, yaw), V3(v, 0.0))
    for i, (x, y) in enumerate(walkers):
        d[100 + i] = ActSnap(Tf(x, y), V3())
    return Snapshot(t, d)


# speeds: car 1 on lane (1,0,-1) 40 km/h, car 2 cyclist there, car 3 on a lane without speed
def wp_speed(loc):
    if loc.x > 500:
        return WP(road=7, lane=-1)
    return WP(road=1, section=0, lane=-1)


rk, tm = make(wp_speed)
rk.update(snap(0.0, {1: (0, 0, 0, 10), 2: (100, 0, 0, 4), 3: (600, 0, 0, 10)}))
ds = {c[1].id: c[2] for c in tm.named('set_desired_speed')}
check("car: lane 40 km/h x 0.9 = 36", abs(ds.get(1, 0) - 36.0) < 0.1, str(ds))
check("cyclist capped at 18 km/h", abs(ds.get(2, 0) - 18.0) < 1e-6)
check("no lane speed -> fallback 30 x 0.9", abs(ds.get(3, 0) - 27.0) < 1e-6)
n = len(tm.named('set_desired_speed'))
rk.update(snap(0.1, {1: (0, 0, 0, 10), 2: (100, 0, 0, 4), 3: (600, 0, 0, 10)}))
check("nothing before period_s", len(tm.named('set_desired_speed')) == n)
rk.update(snap(0.6, {1: (0, 0, 0, 10), 2: (100, 0, 0, 4), 3: (600, 0, 0, 10)}))
check("unchanged speed not resent", len(tm.named('set_desired_speed')) == n)


# lane change: junction 20 m ahead of car 1 only
def wp_junction(loc):
    return WP(ahead=[(20.0, True)] if loc.x < 50 else [(20.0, False)])


rk, tm = make(wp_junction)
rk.update(snap(0.0, {1: (0, 0, 0, 10), 2: (100, 0, 0, 4), 3: (200, 0, 0, 10)}))
lc = {c[1].id: c[2] for c in tm.named('auto_lane_change')}
check("no lane change 20 m before a junction", lc.get(1) is False, str(lc))
check("lane change allowed elsewhere", lc.get(3) is True)
rk.update(snap(1.0, {1: (300, 0, 0, 10), 2: (100, 0, 0, 4), 3: (200, 0, 0, 10)}))
lc = [c for c in tm.named('auto_lane_change') if c[1].id == 1]
check("lane change back on past the junction", lc[-1][2] is True and len(lc) == 2)

# walkers: one on the road 10 m ahead of car 1 (on the far half), one on the pavement
rk, tm = make(walkers=[(10, 3), (8, 9)])
rk.update(snap(0.0, {1: (0, 0, 0, 8), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, 3), (8, 9)]))
holds = [c for c in tm.named('vehicle_percentage_speed_difference')]
check("stop for a walker on the road ahead (far half too)",
      [(c[1].id, c[2]) for c in holds] == [(1, 100.0)], str(holds))
rk.update(snap(0.2, {1: (2, 0, 0, 2), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, -1), (8, 9)]))
check("held while the walker crosses", 1 in rk.held and
      len(tm.named('vehicle_percentage_speed_difference')) == 1)
before = len(tm.named('set_desired_speed'))
rk.update(snap(0.4, {1: (2, 0, 0, 0), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, -9), (8, 9)]))
check("not released at once (release_s)", 1 in rk.held)
rk.update(snap(1.6, {1: (2, 0, 0, 0), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, -9), (8, 9)]))
rel = [c for c in tm.named('set_desired_speed')[before:] if c[1].id == 1]
check("released once the walker has left the road", 1 not in rk.held and rel and abs(rel[-1][2] - 36.0) < 0.1,
      str(rel))

rk, tm = make(walkers=[(-10, 0)])
rk.update(snap(0.0, {1: (0, 0, 0, 8), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(-10, 0)]))
check("walker behind: no stop", not tm.named('vehicle_percentage_speed_difference'))
rk, tm = make(walkers=[(40, 0)])
rk.update(snap(0.0, {1: (0, 0, 0, 8), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(40, 0)]))
check("walker beyond look-ahead (8 m/s x 2.5 + 4 = 24 m): no stop",
      not tm.named('vehicle_percentage_speed_difference'))
rk, tm = make(walkers=[(10, 0)])
rk.update(snap(0.0, {1: (0, 0, 90, 8), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, 0)]))
check("heading respected (car facing +y, walker at +x)",
      not tm.named('vehicle_percentage_speed_difference'))

# a walker standing still on the road = stuck navmesh agent: ignored after stuck_s
rk, tm = make(walkers=[(10, 0)])
for i in range(0, 120):
    t = i * 0.2
    rk.update(snap(t, {1: (0, 0, 0, 0), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, 0)]))
    if 1 not in rk.held and t > 1.0:
        break
check("still walker: released after stuck_s + release_s", 1 not in rk.held and 8.5 < t < 10.0, "t=%.1f" % t)
rk.update(snap(t + 7.0, {1: (0, 0, 0, 0), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, 0)]))
check("stuck walker stays ignored", 1 not in rk.held)

# timeout: a walker moving about on the road ahead for ever
rk, tm = make(walkers=[(10, 0)])
for i in range(0, 120):
    t = i * 0.2
    wy = 0.5 if i % 2 else -0.5
    rk.update(snap(t, {1: (0, 0, 0, 0), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, wy)]))
    if 1 not in rk.held and t > 1.0:
        break
check("hold released after max_hold_s", 1 not in rk.held and 15.0 < t < 16.0, "t=%.1f" % t)
n = len(tm.named('vehicle_percentage_speed_difference'))
rk.update(snap(t + 2.0, {1: (0, 0, 0, 0), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, 0.5)]))
check("cooldown: no new hold right away", len(tm.named('vehicle_percentage_speed_difference')) == n)
rk.update(snap(t + 6.0, {1: (0, 0, 0, 0), 2: (100, 0, 0, 4), 3: (200, 0, 0, 8)}, [(10, -0.5)]))
check("held again after the cooldown", len(tm.named('vehicle_percentage_speed_difference')) == n + 1)

rk, tm = make()
rk.update(snap(0.0, {1: (0, 0, 0, 8), 3: (200, 0, 0, 8)}))
check("destroyed car dropped", rk.ids == [1, 3])

# ---------------------------------------------------------------- 6. 止まれ
print("6. 止まれ: full stop at the stop line / 一時停止")
# A cross junction (id 20, box x 100..115, |y| < 7.5). Road 10 runs east
# (y = -1.75, s = x) and road 11 west (y = +1.75, s = 215 - x); each has a
# 206 stop at s = 98, the line 2 m before the junction. Road 30 (x = 107.5,
# along y) is the priority road without a stop.


class Loc(object):
    def __init__(self, x, y):
        self.x, self.y, self.z = x, y, 0.0


class LTf(object):
    def __init__(self, x, y):
        self.location = Loc(x, y)


class BBox(object):
    location = Loc(107.5, 0.0)
    extent = Loc(7.5, 7.5)


class Junction(object):
    id = 20
    bounding_box = BBox()


ROADS = {  # road -> (x(s), y, length, next road, prev road)
    10: (lambda s: s, -1.75, 100.0), 11: (lambda s: 215.0 - s, 1.75, 100.0),
    500: (lambda s: 100.0 + s, -1.75, 15.0), 501: (lambda s: 115.0 - s, 1.75, 15.0),
    12: (lambda s: 115.0 + s, -1.75, 500.0), 13: (lambda s: 100.0 - s, 1.75, 500.0),
    30: (lambda s: 107.5, None, 1000.0)}
NEXT = {10: 500, 500: 12, 11: 501, 501: 13}
PREV = {v: k for k, v in NEXT.items()}


class LWP(object):
    lane_type = 'Driving'
    section_id = 0

    def __init__(self, road, s, lane=-1):
        self.road_id, self.lane_id, self.s = road, lane, s
        self.is_junction = road in (500, 501)
        fx, y, _l = ROADS[road]
        self.transform = LTf(fx(s), y if y is not None else s - 500.0)

    def _at(self, road, s):
        length = ROADS[road][2]
        if s > length:
            return [LWP(NEXT[road], s - length)] if road in NEXT else []
        if s < 0:
            if road not in PREV:
                return []
            pr = PREV[road]
            return [LWP(pr, ROADS[pr][2] + s)]
        return [LWP(road, s)]

    def next(self, d):
        return self._at(self.road_id, self.s + d)

    def previous(self, d):
        return self._at(self.road_id, self.s - d)

    def get_junction(self):
        return Junction()


class LM(object):
    def __init__(self, road, s, lid):
        self.road_id, self.s, self.id = road, s, lid
        self.orientation = 'LandmarkOrientation.Positive'

    def get_lane_validities(self):
        return []


class LineMap(Map):
    def __init__(self):
        Map.__init__(self, "Japan/Maps/OitaKosen/OitaKosen")
        self.landmark_calls = 0

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        if abs(loc.x - 107.5) < 7.5 and abs(loc.y) < 7.5:
            return LWP(500, loc.x - 100.0) if loc.y < 0 else LWP(501, 115.0 - loc.x)
        if abs(loc.x - 107.5) < 3.0:
            return LWP(30, loc.y + 500.0)
        if loc.y < 0:
            return LWP(10, loc.x) if loc.x < 100 else LWP(12, loc.x - 115.0)
        return LWP(11, 215.0 - loc.x) if loc.x > 115 else LWP(13, 100.0 - loc.x)

    def get_waypoint_xodr(self, road, lane, s):
        return LWP(road, s) if road in (10, 11) and lane == -1 else None

    def get_all_landmarks_of_type(self, kind):
        self.landmark_calls += 1
        return [LM(10, 98.0, '900001'), LM(11, 98.0, '900002')] if kind == '206' else []


XODR_LINE = """<OpenDRIVE><road id="10"><lanes><laneSection s="0"><right>
<lane id="-1" type="driving"><speed sOffset="0" max="50" unit="km/h"/></lane></right></laneSection></lanes></road>
<road id="11"><lanes><laneSection s="0"><right>
<lane id="-1" type="driving"><speed sOffset="0" max="50" unit="km/h"/></lane></right></laneSection></lanes></road>
</OpenDRIVE>"""


class Sim(object):
    """Cars moving along their path; the TM is modelled as: target 0 after
    percentage_speed_difference 100, else the last desired speed; braking
    <= 5 m/s^2 (TM brake 0.7), throttle <= 2.5 m/s^2."""
    DIRS = {'E': (1.0, 0.0, -1.75), 'W': (-1.0, 0.0, 1.75), 'N': (0.0, 1.0, 107.5)}

    def __init__(self, cars, ids=None, parked=()):
        self.cars = cars                       # vid -> [dir, along, v]
        self.parked = dict(parked)             # vid -> (x, y)
        acts = [Actor(v) for v in list(cars) + list(self.parked)]
        self.map = LineMap()
        self.world = World(self.map, acts)
        self.tm = TM()
        self.rk = tp.RuleKeeper(self.world, self.map, self.tm, ids or list(cars), profile,
                                xodr=XODR_LINE)
        self.t = 0.0
        self.log = {v: [] for v in cars}       # (t, front, v, released?)

    def xy(self, vid):
        d, a, _v = self.cars[vid]
        dx, dy, off = self.DIRS[d]
        return (a, off) if d == 'E' else (215.0 - a, off) if d == 'W' else (off, a)

    def target(self, vid, default):
        for c in reversed(self.tm.calls):
            if len(c) > 1 and getattr(c[1], 'id', None) == vid:
                if c[0] == 'vehicle_percentage_speed_difference':
                    return 0.0
                if c[0] == 'set_desired_speed':
                    return c[2] / 3.6
        return default

    def run(self, seconds, dt=0.05, stop_when=None):
        end = self.t + seconds
        while self.t < end:
            acts = {}
            for vid, (d, a, v) in self.cars.items():
                x, y = self.xy(vid)
                dx, dy, _o = self.DIRS[d]
                yaw = math.degrees(math.atan2(dy, dx))
                acts[vid] = ActSnap(Tf(x, y, yaw), V3(dx * v, dy * v))
            for vid, (x, y) in self.parked.items():
                acts[vid] = ActSnap(Tf(x, y), V3())
            self.rk.update(Snapshot(self.t, acts))
            for vid, car in self.cars.items():
                tgt = self.target(vid, car[2])
                dv = max(-5.0 * dt, min(2.5 * dt, tgt - car[2]))
                car[2] = max(0.0, car[2] + dv)
                car[1] += car[2] * dt
                st = self.rk.stop_state.get(vid)
                self.log[vid].append((self.t, car[1] + 2.3, car[2], st['mode'] if st else None))
            self.t += dt
            if stop_when and stop_when(self):
                return


import math  # noqa: E402

check("Japan profile has stop_signs on", (profile.get('stop_signs') or {}).get('enabled') is True)
sim = Sim({1: ['E', 40.0, 45 / 3.6]})
check("two stop lines found from the 206 landmarks", len(sim.rk.stop_lines.stops) == 2,
      str(sim.rk.stop_lines.stops))
check("stop's junction = the box after the line",
      sim.rk.stop_lines.stops[0]['jkey'] == 20 and abs(sim.rk.stop_lines.stops[0]['r'] - 10.6) < 0.1)
i, dd = sim.rk.stop_lines.ahead(LWP(10, 70.0))
check("ahead: 28 m to the line", i == 0 and abs(dd - 28.0) < 1e-6, "%s %.2f" % (i, dd))
check("crossing road (no stop): nothing ahead", sim.rk.stop_lines.ahead(LWP(30, 480.0)) is None)
check("wrong-way lane of the stop road: nothing", sim.rk.stop_lines.ahead(LWP(10, 50.0, lane=1)) is None)
sim.run(30.0)
lg = sim.log[1]
stopped = [r for r in lg if r[2] < 0.1 and r[3] in ('hold', 'wait')]
front = stopped[0][1] if stopped else None
check("car stops at the line (front bumper within 1 m before, not past)",
      front is not None and 97.0 <= front <= 98.05, "front at %s" % front)
v_line = min(r[2] for r in lg if 92.0 <= r[1] <= 98.5)
check("slows down before the line (no 35 km/h dash)",
      all(r[2] < 6.0 for r in lg if r[1] > 90.0 and r[3] == 'approach'), "min %.2f" % v_line)
rel = next((r for r in lg if r[3] == 'done' or (r[3] is None and r[1] > 99)), None)
still = [r for r in lg if r[2] < 0.1 and rel and r[0] < rel[0]]
check("held still >= 2 s before going", rel is not None and still and
      rel[0] - still[0][0] >= 2.0 - 1e-6, "rel %s still %s" % (rel, still[:1]))
ds1 = [c for c in sim.tm.named('set_desired_speed') if c[1].id == 1]
hold_i = [k for k, c in enumerate(sim.tm.calls) if c[0] == 'vehicle_percentage_speed_difference']
after = [c for c in sim.tm.calls[hold_i[-1]:] if c[0] == 'set_desired_speed'] if hold_i else []
check("released with set_desired_speed back to the lane target (45 km/h)",
      after and abs(after[0][2] - 45.0) < 0.1, str(after[:2]))
check("approach slows with set_desired_speed below the lane target",
      any(c[2] < 20.0 for c in ds1), str(ds1[:4]))
check("drove on through the junction", sim.cars[1][1] > 130.0, "at %.1f" % sim.cars[1][1])
check("stop counted", sim.rk.stop_count['stops'] == 1 and sim.rk.stop_count['missed'] == 0,
      str(sim.rk.stop_count))
check("state cleared once through the junction", 1 not in sim.rk.stop_state)

# two cars at opposite stops: first come, first served, no deadlock
sim = Sim({1: ['E', 50.0, 45 / 3.6], 2: ['W', 46.0, 45 / 3.6]})
sim.run(40.0)
go = {v: next((r[0] for r in sim.log[v] if r[3] == 'done'), None) for v in (1, 2)}
check("both cars go (no deadlock)", None not in go.values(), str(go))
check("first come first served: car 1 (nearer) first", go[1] is not None and go[2] is not None
      and go[1] < go[2], str(go))
a_at = [r for r in sim.log[1] if abs(r[0] - (go[2] or 0)) < 0.026]
check("car 2 goes only when car 1 has left the junction and >= 2.5 s later",
      go[2] - go[1] >= 2.5 and a_at and a_at[0][1] - 2.3 > 115.0, "a %s" % a_at)
check("no hold timed out", sim.rk.stop_count['timeouts'] == 0, str(sim.rk.stop_count))

# a waiting car destroyed (TM idle clean-up) must not block the queue
# (both wait behind a car parked in the junction, then car 1 and it vanish)
sim = Sim({1: ['E', 50.0, 45 / 3.6], 2: ['W', 46.0, 45 / 3.6]}, parked={4: (107.5, 0.0)})
sim.run(40.0, stop_when=lambda s: (s.rk.stop_state.get(2) or {}).get('mode') == 'wait'
        and (s.rk.stop_state.get(1) or {}).get('mode') == 'wait')
both = sim.rk.stop_state[1]['mode'] == sim.rk.stop_state[2]['mode'] == 'wait'
del sim.cars[1]
sim.parked.clear()
t_del = sim.t
sim.run(5.0)
go2 = next((r[0] for r in sim.log[2] if r[3] == 'done'), None)
check("destroyed waiting car dropped, the other goes at once (no timeout)",
      both and 1 not in sim.rk.stop_state and go2 is not None and go2 - t_del < 1.0
      and sim.rk.stop_count['timeouts'] == 0, "both %s go2 %s t_del %.1f" % (both, go2, t_del))

# priority road: wait for the car coming along road 30
sim = Sim({1: ['E', 60.0, 45 / 3.6], 3: ['N', -60.0, 8.0]})
sim.run(40.0)
go1 = next((r[0] for r in sim.log[1] if r[3] == 'done'), None)
c_out = next((r[0] for r in sim.log[3] if r[1] > 7.5 + 2.3 + 2.3), None)
stop1 = next((r[0] for r in sim.log[1] if r[3] == 'wait'), None)
check("waits for the priority car, goes after it has passed",
      go1 is not None and c_out is not None and stop1 is not None and stop1 < c_out < go1,
      "stop %s, car3 out %s, go %s" % (stop1, c_out, go1))
check("car on a lane without a stop is never held",
      not [c for c in sim.tm.calls if c[0] == 'vehicle_percentage_speed_difference' and c[1].id == 3])

# a car parked in the junction: released by max_hold_s, never stuck
sim = Sim({1: ['E', 70.0, 45 / 3.6]}, parked={4: (107.5, 0.0)})
sim.run(30.0)
hold0 = next((r[0] for r in sim.log[1] if r[3] == 'hold'), None)
go1 = next((r[0] for r in sim.log[1] if r[3] == 'done'), None)
check("blocked junction: released after max_hold_s (12 s)",
      hold0 is not None and go1 is not None and 12.0 <= go1 - hold0 <= 12.5 and
      sim.rk.stop_count['timeouts'] == 1, "hold %s go %s" % (hold0, go1))

# walker release while held at a stop keeps the stop hold
sim = Sim({1: ['E', 70.0, 45 / 3.6]})
sim.run(30.0, stop_when=lambda s: (s.rk.stop_state.get(1) or {}).get('mode') == 'hold')
sim.rk._release(1)
check("walker release during a 止まれ hold re-applies the stop",
      sim.tm.calls[-1][0] == 'vehicle_percentage_speed_difference' and sim.tm.calls[-1][2] == 100.0)

# two helpers in separate processes (delivery vans vs generate_traffic): each
# sees the other's car held at the line as foreign traffic. With the junction
# box (+2.5 m) reaching over both lines (as at 7 of 74 OitaKosen lines), the
# old order (box check first) blocked both until max_hold_s.
# 別プロセスの補助同士: 相手が停止線で止めている車で詰まらない
class Sim2(Sim):
    def __init__(self, cars):
        Sim.__init__(self, cars, ids=[1])
        self.rk2 = tp.RuleKeeper(self.world, self.map, self.tm, [2], profile, xodr=XODR_LINE)
        for k in (self.rk, self.rk2):
            for st in k.stop_lines.stops:
                st['ex'] = 13.0              # box +2.5 m reaches x 92..123 / 停止線に掛かる箱
        orig = self.rk.update

        def both(snap):
            orig(snap)
            self.rk2.update(snap)
        self.rk.update = both


def still_to_go(log, modes=None):
    """(first standstill at the line, release) of one car / 停止開始と発進の時刻"""
    st = next((r[0] for r in log if r[2] < 0.1), None)
    go = next((r[0] for r in log if r[3] == 'done'), None)
    return st, go


sim = Sim2({1: ['E', 50.0, 45 / 3.6], 2: ['W', 46.0, 45 / 3.6]})
sim.run(40.0)
s1, g1 = still_to_go(sim.log[1])
check("two helpers, box over both lines: car 1 goes 2 s after stopping, no timeout",
      None not in (s1, g1) and 2.0 - 1e-6 <= g1 - s1 < 3.0 and
      sim.rk.stop_count['timeouts'] == 0 and sim.rk2.stop_count['timeouts'] == 0,
      "car1 still %s go %s, timeouts %s/%s" % (s1, g1, sim.rk.stop_count['timeouts'],
                                                sim.rk2.stop_count['timeouts']))
check("two helpers: car 2 released too, no timeout", sim.cars[2][1] > 130.0 and
      sim.rk2.stop_count['stops'] == 1, "car2 at %.1f" % sim.cars[2][1])


def foreign(at_line_m=None):
    prof = dict(profile)
    if at_line_m is not None:
        prof['stop_signs'] = dict(profile['stop_signs'], at_line_m=at_line_m)
    sm = Sim({2: ['W', 60.0, 45 / 3.6]}, ids=[2], parked={6: (98.0, -1.75)})
    sm.rk = tp.RuleKeeper(sm.world, sm.map, sm.tm, [2], prof, xodr=XODR_LINE)
    for st in sm.rk.stop_lines.stops:
        st['ex'] = 13.0
    sm.run(30.0)
    return sm


# a foreign car standing at the other line (centre at it, inside the box)
sim = foreign()
s2, g2 = still_to_go(sim.log[2])
check("foreign car held at the other line (inside the box) does not block the release",
      None not in (s2, g2) and g2 - s2 < 3.0 and sim.rk.stop_count['timeouts'] == 0,
      "still %s go %s" % (s2, g2))
sim = foreign(at_line_m=1e9)                 # old order: box check decides / 旧順序
check("control: without the at-line exemption the same car blocks until max_hold_s",
      sim.rk.stop_count['timeouts'] == 1, str(sim.rk.stop_count))

# helper switched off (release_all, also after an error in update): every car
# held at a 止まれ, slowed on its approach or held for a walker gets its lane
# target back, none stays at target 0 / 補助停止時に全車の速度を戻す
sim = Sim({1: ['E', 70.0, 45 / 3.6], 2: ['W', 60.0, 45 / 3.6], 5: ['N', -80.0, 8.0]})
sim.run(30.0, stop_when=lambda s: (s.rk.stop_state.get(1) or {}).get('mode') == 'hold')
sim.rk.held[5] = [sim.t, sim.t]           # a walker hold on car 5
sim.tm.vehicle_percentage_speed_difference(sim.world.get_actor(5), 100.0)
modes = {v: (sim.rk.stop_state.get(v) or {}).get('mode') for v in (1, 2)}
sim.rk.release_all()
check("release_all: 止まれ hold, approach and walker hold all back to a desired speed",
      modes[1] == 'hold' and modes[2] in ('approach', 'hold') and
      all(sim.target(v, None) not in (None, 0.0) for v in (1, 2, 5))
      and not sim.rk.stop_state and not sim.rk.held,
      "modes %s targets %s" % (modes, [sim.target(v, None) for v in (1, 2, 5)]))
src = open(os.path.join(HIL, 'generate_traffic.py')).read()
blk = src[src.index('rule_keeper.update(world.get_snapshot())'):][:600]
check("generate_traffic: helper error calls release_all before dropping the helper",
      blk.index('release_all()') < blk.index('rule_keeper = None'))

# off: profile without stop_signs, and the plain fake map, make no landmark query
prof_off = dict(profile)
prof_off.pop('stop_signs', None)
m = LineMap()
rk_off = tp.RuleKeeper(World(m, [Actor(1)]), m, TM(), [1], prof_off, xodr=XODR_LINE)
check("profile without stop_signs: no landmark query, no helper state",
      m.landmark_calls == 0 and rk_off.stop_lines is None)
rk, tm = make()
check("map without 206 landmarks: stop helper off", rk.stop_lines is None)


class BadMap(LineMap):
    def get_all_landmarks_of_type(self, kind):
        raise ValueError("broken landmarks")


m = BadMap()
rk_bad = tp.RuleKeeper(World(m, [Actor(1)]), m, TM(), [1], profile, xodr=XODR_LINE)
check("landmark error: stop helper off, the rest of the helper still made",
      rk_bad.stop_lines is None and rk_bad.yield_cfg is not None)
for mname in ("Carla/Maps/Town03", "Singapore/Maps/PasirRis/PasirRis"):
    tm = TM()
    m = LineMap()
    m.name = mname
    helper = gt.setup_traffic_profile(World(m, [Actor(1)]), tm, [1], 'auto')
    check("%s: no helper, no landmark query, 0 TM calls" % mname,
          helper is None and m.landmark_calls == 0 and tm.calls == [])

print("curve speed (the 100 km/h ring's corners) / カーブの速度")


class CWP(object):
    """A waypoint on a path of yaw angles every 5 m. / 5 m ごとの向きの列。"""
    def __init__(self, yaws, i=0):
        self.yaws, self.i = yaws, i
        self.transform = types.SimpleNamespace(rotation=types.SimpleNamespace(yaw=yaws[i]))

    def next(self, d):
        return [CWP(self.yaws, self.i + 1)] if self.i + 1 < len(self.yaws) else []


cc = {"lat_accel": 3.0, "decel": 2.5, "lookahead_m": 80, "step_m": 5}
straight = CWP([0.0] * 40)
check("straight road: the posted 90 stays", tp.curve_kmh(straight, 90.0, cc) == 90.0)
# a 15 m-radius 90 deg corner starting 10 m ahead: 19.1 deg of yaw per 5 m / 半径 15 m の角
corner = CWP([0.0, 0.0, 0.0] + [19.1 * k for k in range(1, 6)] + [90.0] * 30)
v_near = tp.curve_kmh(corner, 90.0, cc)
check("a 15 m corner 10 m ahead: well under 40 km/h", v_near < 40.0, "%.1f" % v_near)
check("... the same corner far ahead allows more now",
      tp.curve_kmh(CWP([0.0] * 12 + [19.1 * k for k in range(1, 6)] + [90.0] * 30), 90.0, cc) > v_near)
check("stand-in waypoints without a transform: no change", tp.curve_kmh(object(), 50.0, cc) == 50.0)

print("\n%d failure(s)" % len(FAILS) + (": " + ", ".join(FAILS) if FAILS else ""))
sys.exit(1 if FAILS else 0)
