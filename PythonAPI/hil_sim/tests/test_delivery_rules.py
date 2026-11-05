"""Tests for the delivery vans' Japanese rules (delivery/rules.py, the
delivery_sim Fleet/Van hooks). No CARLA server: fakes stand in for the
world, map, snapshots and the vans' BasicAgent.

1. a van at 30 km/h towards a 止まれ (206) slows, stops with its front at
   the line, stands >= 2 s, goes on; never faster than its own --speed;
2. delivery stops: a van parked for work is dropped from the RuleKeeper
   (state and gate cleared, nothing "releases" it); back on the road it
   stops at the line again, also when it parked right at the line;
3. lane speed: the van's target = min(its --speed, lane limit x 0.9);
4. a walker on the road ahead stops the van, released when gone;
5. Van.step (DRIVING) brakes on the gate's stop; Fleet.set_rules(None) on
   other maps touches no van; make() on Town03 / Pasir Ris = None, no
   landmark query.

配送車の日本の交通ルールの試験（CARLA サーバー不要）。
"""
import math
import os
import sys
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import traffic_profile as tp  # noqa: E402
from delivery import rules as R  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


_name, PROFILE = tp.select_profile("Japan/Maps/OitaKosen/OitaKosen", force="japan")


# ---------------------------------------------------------------- fakes
class Loc(object):
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


class Rot(object):
    def __init__(self, yaw=0.0):
        self.yaw = yaw


class Tf(object):
    def __init__(self, x, y, yaw=0.0):
        self.location = Loc(x, y)
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
        self.actors = actors

    def find(self, aid):
        return self.actors.get(aid)


class Extent(object):
    def __init__(self, x):
        self.x = x


class BB(object):
    def __init__(self, half):
        self.extent = Extent(half)


class Actor(object):
    def __init__(self, aid, type_id='vehicle.mercedes.sprinter', half=2.9):
        self.id, self.type_id = aid, type_id
        self.attributes = {'role_name': 'delivery'}
        self.bounding_box = BB(half)


class ActorList(list):
    def filter(self, pattern):
        pre = pattern.rstrip('*')
        return ActorList(a for a in self if a.type_id.startswith(pre))


class World(object):
    def __init__(self, wmap, actors):
        self.map, self.actors = wmap, ActorList(actors)

    def get_map(self):
        return self.map

    def get_actors(self, ids=None):
        return self.actors if ids is None else ActorList(a for a in self.actors if a.id in ids)

    def get_actor(self, aid):
        return next((a for a in self.actors if a.id == aid), None)


# One road east (y = -1.75, s = x) with a 206 stop at s = 98, junction 100..115
class BBox(object):
    location = Loc(107.5, 0.0)
    extent = Loc(7.5, 7.5)


class Junction(object):
    id = 20
    bounding_box = BBox()


ROADS = {10: (100.0, 500), 500: (15.0, 12), 12: (500.0, None)}
X0 = {10: 0.0, 500: 100.0, 12: 115.0}
PREV = {500: 10, 12: 500}


class LWP(object):
    lane_type = 'Driving'
    section_id = 0
    lane_id = -1

    def __init__(self, road, s):
        self.road_id, self.s = road, s
        self.is_junction = road == 500
        self.transform = Tf(X0[road] + s, -1.75)

    def _at(self, road, s):
        length, nxt = ROADS[road]
        if s > length:
            return [LWP(nxt, s - length)] if nxt else []
        if s < 0:
            pr = PREV.get(road)
            return [LWP(pr, ROADS[pr][0] + s)] if pr else []
        return [LWP(road, s)]

    def next(self, d):
        return self._at(self.road_id, self.s + d)

    def previous(self, d):
        return self._at(self.road_id, self.s - d)

    def get_junction(self):
        return Junction()


class LM(object):
    road_id, s, id = 10, 98.0, '900001'
    orientation = 'LandmarkOrientation.Positive'

    def get_lane_validities(self):
        return []


class LineMap(object):
    def __init__(self, name="Japan/Maps/OitaKosen/OitaKosen"):
        self.name = name
        self.landmark_calls = 0

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        if abs(loc.y) >= 7.0:
            return None
        if loc.x < 100:
            return LWP(10, loc.x)
        return LWP(500, loc.x - 100.0) if loc.x < 115 else LWP(12, loc.x - 115.0)

    def get_waypoint_xodr(self, road, lane, s):
        return LWP(road, s) if road == 10 and lane == -1 else None

    def get_all_landmarks_of_type(self, kind):
        self.landmark_calls += 1
        return [LM()] if kind == '206' else []

    def to_opendrive(self):
        return xodr(50)


def xodr(kmh):
    return ('<OpenDRIVE>' + ''.join(
        '<road id="%d"><lanes><laneSection s="0"><right><lane id="-1" type="driving">'
        '<speed sOffset="0" max="%d" unit="km/h"/></lane></right></laneSection></lanes></road>'
        % (r, kmh) for r in (10, 500, 12)) + '</OpenDRIVE>')


class Agent(object):
    """BasicAgent stand-in: target speed (km/h), control from a P controller."""

    def __init__(self, kmh):
        self.target = kmh
        self.v = 0.0
        self.calls = []

    def set_target_speed(self, kmh):
        self.calls.append(kmh)
        self.target = kmh

    def run_step(self):
        c = types.SimpleNamespace(throttle=0.0, brake=0.0, steer=0.0, hand_brake=False)
        err = self.target / 3.6 - self.v
        if err > 0:
            c.throttle = min(1.0, 0.5 * err)
        else:
            c.brake = min(1.0, -0.3 * err)
        return c


class VanSim(object):
    """One van on road 10: its own agent, RuleKeeper via VanRules."""

    def __init__(self, x0, base=30.0, lane_kmh=50, walkers=()):
        self.map = LineMap()
        self.van = Actor(7)
        self.walkers = [Actor(100 + i, 'walker.pedestrian.0001') for i in range(len(walkers))]
        self.wpos = list(walkers)
        self.world = World(self.map, [self.van] + self.walkers)
        self.rules = R.VanRules(self.world, self.map, PROFILE, xodr=xodr(lane_kmh))
        self.base = base
        self.agent = Agent(base)
        self.x, self.t = x0, 0.0
        self.agent.v = base / 3.6
        self.driving = True
        self.log = []                      # (t, front, v, mode)

    def front(self):
        return self.x + self.van.bounding_box.extent.x

    def step(self, dt=0.05):
        acts = {7: ActSnap(Tf(self.x, -1.75), Loc(self.agent.v, 0.0))}
        for w, (wx, wy) in zip(self.walkers, self.wpos):
            if wx is not None:
                acts[w.id] = ActSnap(Tf(wx, wy), Loc(0.0, 0.0))
        self.rules.update(Snapshot(self.t, acts), [7] if self.driving else [])
        if self.driving:
            c = self.rules.apply(7, self.agent, self.base, self.agent.run_step())
            acc = 2.5 * c.throttle - 7.0 * c.brake
        else:
            acc = -7.0                     # the van's own hold() / 自前の停止
        self.agent.v = max(0.0, self.agent.v + acc * dt)
        self.x += self.agent.v * dt
        st = self.rules.rk.stop_state.get(7)
        self.log.append((self.t, self.front(), self.agent.v, st['mode'] if st else None))
        self.t += dt

    def run(self, seconds, until=None):
        end = self.t + seconds
        while self.t < end:
            self.step()
            if until and until(self):
                return


# ---------------------------------------------------------------- 1. 止まれ
print("1. van stops at 止まれ / 配送車の一時停止")
sim = VanSim(40.0)
check("stop line read from the 206 landmark", sim.rules.rk.stop_lines is not None
      and len(sim.rules.rk.stop_lines.stops) == 1)
sim.run(40.0)
lg = sim.log
still = [r for r in lg if r[2] < 0.1 and r[3] in ('hold', 'wait')]
front = still[0][1] if still else None
check("van stops with the front at the line (97..98.05 m)",
      front is not None and 97.0 <= front <= 98.05, "front %s" % front)
rel = next((r for r in lg if r[3] == 'done'), None)
check("held still >= 2 s before going",
      rel is not None and still and rel[0] - still[0][0] >= 2.0 - 1e-6,
      "still %s rel %s" % (still[:1], rel))
check("never faster than its own --speed (30 km/h)",
      max(r[2] for r in lg) <= 30 / 3.6 + 0.05, "%.2f" % max(r[2] for r in lg))
check("drove on through the junction", sim.x > 130.0, "x %.1f" % sim.x)
check("target back to 30 km/h after the stop", sim.agent.target == 30.0, str(sim.agent.target))
check("summary counts the stop", sim.rules.summary()['stop_line_stops'] == 1,
      str(sim.rules.summary()))

# ---------------------------------------------------------------- 2. delivery stops
print("2. no conflict with delivery stops / 配送停車との干渉なし")
sim = VanSim(40.0)
sim.run(30.0, until=lambda s: (s.rules.rk.stop_state.get(7) or {}).get('mode') == 'hold')
check("in a 止まれ hold", sim.rules.stopped(7))
sim.driving = False                        # arrives at a delivery stop / 配送停車
sim.run(20.0)
check("parked van dropped: no 止まれ state, gate cleared",
      7 not in sim.rules.rk.stop_state and not sim.rules.stopped(7)
      and 7 not in sim.rules.gate.kmh)
check("the RuleKeeper never ran a parked van (no hold, no timeout counted)",
      sim.rules.rk.stop_count['timeouts'] == 0 and sim.rules.rk.ids == [])
t_back = sim.t
sim.driving = True                         # drives on, right at the line / 停止線で再出発
sim.run(20.0)
back = [r for r in sim.log if r[0] >= t_back]
st_back = [r for r in back if r[2] < 0.1 and r[3] in ('hold', 'wait')]
go_back = next((r for r in back if r[3] == 'done'), None)
check("after the delivery stop it stands 2 s at the line again, then goes",
      st_back and go_back and go_back[0] - st_back[0][0] >= 2.0 - 1e-6 and sim.x > 110.0,
      "still %s go %s x %.1f" % (st_back[:1], go_back, sim.x))

# a delivery stop before the line, far back: drives on and stops at the line
sim = VanSim(20.0)
sim.run(3.0)
sim.driving = False
sim.run(10.0)
sim.driving = True
sim.run(40.0)
st2 = [r for r in sim.log if r[2] < 0.1 and r[3] in ('hold', 'wait')]
check("delivery stop 70 m before the line: still stops at the line",
      st2 and 97.0 <= st2[0][1] <= 98.05 and sim.x > 130.0,
      "front %s" % (st2[0][1] if st2 else None))

# ---------------------------------------------------------------- 3. lane speed
print("3. lane speed / 車線の最高速度")
sim = VanSim(150.0, lane_kmh=20)
sim.run(3.0)
check("lane 20 km/h: van target 18 km/h (x 0.9)", abs(sim.agent.target - 18.0) < 0.01,
      str(sim.agent.target))
sim = VanSim(150.0, lane_kmh=60)
sim.run(3.0)
check("lane 60 km/h: van keeps its own 30 km/h (cap, never faster)",
      sim.agent.target == 30.0, str(sim.agent.target))

# ---------------------------------------------------------------- 4. walker
print("4. walker on the road / 車道の歩行者")
sim = VanSim(150.0, walkers=[(162.0, 1.0)])
sim.run(4.0)
check("walker on the road ahead: van stopped", sim.agent.v < 0.1 and sim.rules.stopped(7),
      "v %.2f" % sim.agent.v)
sim.wpos[0] = (None, None)
sim.run(4.0)
check("walker gone: van drives on", sim.agent.v > 3.0 and not sim.rules.stopped(7),
      "v %.2f" % sim.agent.v)

# ---------------------------------------------------------------- 5. delivery_sim hooks
print("5. delivery_sim hooks / delivery_sim の組込み")
import delivery_sim as ds  # noqa: E402

check("--traffic-profile defaults to auto",
      ds.parse_args(["--map", ""]).traffic_profile == "auto")
gate_rules = types.SimpleNamespace(applied=[])


def fake_apply(vid, agent, base, control):
    gate_rules.applied.append((vid, base))
    control.throttle, control.brake = 0.0, 1.0
    return control


gate_rules.apply = fake_apply
sent = []
stop = types.SimpleNamespace(kind="drone", location=(500.0, 0.0, 0.0), orders=[], batch=1)
van = types.SimpleNamespace(
    lmp=False, dropped=[], state=ds.DRIVING, stops=[stop],
    energy=types.SimpleNamespace(level=90.0), pem=types.SimpleNamespace(critical=5.0),
    args=types.SimpleNamespace(arrive_radius=8.0, speed=30.0),
    agent=types.SimpleNamespace(done=lambda: False,
                                run_step=lambda: types.SimpleNamespace(
                                    throttle=0.8, brake=0.0, steer=0.1, hand_brake=False)),
    car=types.SimpleNamespace(id=7, apply_control=sent.append),
    car_pos=lambda: (0.0, 0.0, 0.0), energy_step=lambda t, dt: None,
    guard=lambda t, dt, c: None, rules=gate_rules)
ds.Van.step(van, 1.0, 0.05)
check("Van.step DRIVING: the rules' stop reaches apply_control (brake, steer kept)",
      sent and sent[-1].brake == 1.0 and sent[-1].throttle == 0.0 and sent[-1].steer == 0.1
      and gate_rules.applied == [(7, 30.0)], str(vars(sent[-1]) if sent else None))
van.rules = None
ds.Van.step(van, 1.1, 0.05)
check("Van.step without rules: the agent's control unchanged",
      sent[-1].throttle == 0.8 and sent[-1].brake == 0.0)
speeds = []
fleet = types.SimpleNamespace(rules=None, vans=[types.SimpleNamespace(
    rules=None, agent=types.SimpleNamespace(set_target_speed=speeds.append))],
    args=types.SimpleNamespace(speed=30.0))
ds.Fleet.set_rules(fleet, None)
check("other maps: set_rules(None) touches no van agent", speeds == [] and
      fleet.vans[0].rules is None)
for mname in ("Carla/Maps/Town03", "Singapore/Maps/PasirRis/PasirRis"):
    m = LineMap(mname)
    check("%s: make() = None, no landmark query" % mname,
          R.make(World(m, []), m) is None and m.landmark_calls == 0)
m = LineMap()
check("OitaKosen: make() builds the van rules", R.make(World(m, []), m) is not None
      and m.landmark_calls == 1)

print("\n%d failure(s)" % len(FAILS) + (": " + ", ".join(FAILS) if FAILS else ""))
sys.exit(1 if FAILS else 0)
