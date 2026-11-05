"""Tests for the Japanese pedestrian signals (hokousha.py) and the waiting-walker
stop of the Japan traffic profile. No CARLA server: fakes stand in for the
traffic lights (a fake junction cycling its phases like CARLA's group),
walkers, controllers, world and snapshots.

1. the phase: WALK while the conflicting phase is red, FLASH for the 青点滅
   time, STOP for the all-red gap and while the conflicting phase runs;
2. the order CARLA cycles the phases in is learned (3-phase junction): walk
   lasts through the non-conflicting phases once known; before, safe;
3. countdown steps (10) go down in walk and in wait; texel channels;
4. heads: one RPC per change, grouped, from the worker thread, never per tick;
5. walkers: held at the kerb when not WALK (standing or heading in), never
   on the crossing, not when walking past, released at WALK with own speed;
   overlapping zones: only the crossing the walker heads into counts;
6. setup_ped_signals: no call at all on non-Japan maps;
7. RuleKeeper: stops a car for a walker waiting at an unsignalised crossing.

歩行者用信号の試験（CARLA サーバー不要）。
"""
import os
import sys
import time

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import hokousha as hk  # noqa: E402
import traffic_profile as tp  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail else ""))
    if not cond:
        FAILS.append(name)


class V3(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Tf(object):
    def __init__(self, x, y, yaw=0.0):
        self.location = V3(x, y)
        self.rotation = type('R', (), {'yaw': yaw})()


class FakeTL(object):
    def __init__(self, sid, g=10.0, y=3.0, r=2.0):
        self.sid, self.g, self.y, self.r = sid, g, y, r
        self.state, self.elapsed = 'Red', 0.0

    def get_opendrive_id(self):
        return self.sid

    def get_state(self):
        return 'TrafficLightState.' + self.state

    def get_elapsed_time(self):
        return self.elapsed

    def get_green_time(self):
        return self.g

    def get_yellow_time(self):
        return self.y

    def get_red_time(self):
        return self.r


class FakeGroup(object):
    """CARLA's group: controllers in turn green -> yellow -> red(all-red), others red with clock 0."""

    def __init__(self, phases, order):
        self.phases, self.order = phases, order
        self.i, self.t = 0, 0.0
        self.set()

    def set(self):
        for k, ph in enumerate(self.phases):
            for tl in ph:
                tl.state, tl.elapsed = 'Red', 0.0
        ph = self.phases[self.order[self.i]]
        g, y = ph[0].g, ph[0].y
        for tl in ph:
            if self.t < g:
                tl.state, tl.elapsed = 'Green', self.t
            elif self.t < g + y:
                tl.state, tl.elapsed = 'Yellow', self.t - g
            else:
                tl.state, tl.elapsed = 'Red', self.t - g - y

    def advance(self, dt):
        self.t += dt
        ph = self.phases[self.order[self.i]][0]
        if self.t >= ph.g + ph.y + ph.r:
            self.t -= ph.g + ph.y + ph.r
            self.i = (self.i + 1) % len(self.order)
        self.set()


class ActorList(list):
    def filter(self, pat):
        return ActorList(a for a in self if pat == 'traffic.traffic_light' and isinstance(a, FakeTL))


class FakeWorld(object):
    def __init__(self, tls, names=()):
        self.tls, self.names, self.calls = tls, list(names), []

    def get_actors(self, ids=None):
        return ActorList(self.tls)

    def get_names_of_all_objects(self):
        return self.names

    def apply_color_texture_to_objects(self, names, param, tex):
        self.calls.append((sorted(names), tex.texel))


class FakeCarla(object):
    class MaterialParameter(object):
        Emissive = 'Emissive'

    class Color(object):
        def __init__(self, r, g, b, a):
            self.v = (r, g, b, a)

    class TextureColor(object):
        def __init__(self, w, h):
            self.texel = None

        def set(self, x, y, c):
            self.texel = c.v


class Walker(object):
    def __init__(self, x, y, vx=0.0, vy=0.0):
        self.tf, self.vel = Tf(x, y), V3(vx, vy)

    def get_transform(self):
        return self.tf

    def get_velocity(self):
        return self.vel


class Con(object):
    def __init__(self):
        self.speeds = []

    def set_max_speed(self, v):
        self.speeds.append(v)


class Snap(object):
    def __init__(self, t, actors):
        self.timestamp = type('T', (), {'elapsed_seconds': t})()
        self.actors = actors

    def find(self, i):
        return self.actors.get(i)


def crossing(cid, conflict, phases, length=8.0, signal=True):
    # a crossing across a road running along x, at x = 0; kerbs at y = -4 / +4
    # x 方向の道路を横切る横断歩道（x = 0、縁石 y = ±4）
    return {"id": cid, "c": [0.0, 0.0], "z": 0.0, "n": [0.0, 1.0], "f": [1.0, 0.0],
            "t0": -length / 2, "t1": length / 2, "width": 4.0, "length": length,
            "ends": [[0, -length / 2], [0, length / 2]],
            "wait_zones": [[[-2.5, -length / 2 + 0.5], [-2.5, -length / 2 - 3], [2.5, -length / 2 - 3], [2.5, -length / 2 + 0.5]],
                           [[-2.5, length / 2 - 0.5], [-2.5, length / 2 + 3], [2.5, length / 2 + 3], [2.5, length / 2 - 0.5]]],
            "signal": signal, "conflict": conflict, "junction": "J", "phases": phases}


def run(ps, grp, t0, t1, dt=0.2, actors=None):
    seq, t = [], t0
    while t < t1 - 1e-9:
        ps.update(Snap(t, actors or {}))
        c = ps.crossings[0]
        seq.append((round(t, 1), c.phase, int(-(-c.frac * 8 // 1))))
        grp.advance(dt)
        t += dt
    return seq


print("1. phase from a 2-phase junction / 2 現示の交差点")
a, b = FakeTL('a'), FakeTL('b')
grp = FakeGroup([[a], [b]], order=[1, 0])          # starts with the parallel phase b / 並行方向から
w = FakeWorld([a, b])
data = {"map": "T", "crossings": [crossing(0, ['a'], [['a'], ['b']])]}
ps = hk.PedSignals(w, data, [], carla_mod=None, send_heads=False)
check("one crossing, one junction", len(ps.crossings) == 1 and len(ps.junctions) == 1)
seq = run(ps, grp, 0.0, 30.0)
ph = {t: p for t, p, _ in seq}
F = hk.flash_time(8.0)
check("flash time = whole 8 m at 1.0 m/s", F == 8.0 and hk.flash_time(2.0) == 4.0 and hk.flash_time(30) == 10.0, F)
check("walk at the start of b green", ph[0.0] == 'walk', ph[0.0])
check("walk until T = all-red + F (t < 5)", ph[4.8] == 'walk', ph[4.8])
check("flash from t = 5 (青点滅)", ph[5.4] == 'flash' and ph[12.6] == 'flash', (ph[5.4], ph[12.6]))
check("stop in the all-red gap (t > 13)", ph[13.4] == 'stop' and ph[14.8] == 'stop', (ph[13.4], ph[14.8]))
check("stop while the conflicting phase runs", all(ph[t] == 'stop' for t in (15.0, 20.0, 29.8)))
check("walk again when b returns (t = 30 cycle)", run(ps, grp, 30.0, 31.0)[2][1] == 'walk')
seq2 = run(ps, grp, 31.0, 60.0)
walk_steps = [s for _, p, s in seq2 if p == 'walk']
check("walk countdown goes down", walk_steps == sorted(walk_steps, reverse=True) and walk_steps[0] >= 6,
      walk_steps[:3] + walk_steps[-3:])
stop_steps = [s for t, p, s in seq2 if p == 'stop' and t >= 45.0]
check("wait countdown goes down", stop_steps == sorted(stop_steps, reverse=True) and stop_steps[-1] <= 2,
      stop_steps[:3] + stop_steps[-3:])

print("1b. short side phase between conflicting ones: no 2 s walk / 短い現示では青にしない")
# the 6-phase cluster under the police timing: main 20 s, then conflicting c1, a short
# non-conflicting b (6.4 s green), conflicting c2 / 警察の時間: 主現示の後に短い現示
a, c1, b, c2 = FakeTL('a', g=20.0), FakeTL('c1', g=6.4), FakeTL('b', g=6.4), FakeTL('c2', g=6.4)
grp = FakeGroup([[a], [c1], [b], [c2]], order=[0, 1, 2, 3])
data = {"map": "T", "crossings": [crossing(0, ['c1', 'c2'], [['a'], ['c1'], ['b'], ['c2']])]}
ps = hk.PedSignals(FakeWorld([a, c1, b, c2]), data, [], carla_mod=None, send_heads=False)
seq = run(ps, grp, 0.0, 2 * 70.2)
main = 25.0 + 11.4                     # c1 ends, b starts / b の開始
during_b = [p for t, p, _ in seq if main <= t < main + 11.4] + \
           [p for t, p, _ in seq if 70.2 + main <= t < 70.2 + main + 11.4]
check("no walk while only the short side phase runs", set(during_b) == {'stop'}, sorted(set(during_b)))
check("walk with the main phase", any(p == 'walk' for t, p, _ in seq if 70.2 <= t < 80.0))
check("a walk already running still ends in a flash", [p for t, p, _ in seq if 70.2 <= t < 95.2].count('flash') > 0)

print("2. 3-phase junction: order learned / 3 現示: 順序の学習")
a, b, c = FakeTL('a'), FakeTL('b'), FakeTL('c')
grp = FakeGroup([[a], [b], [c]], order=[0, 1, 2])
w = FakeWorld([a, b, c])
ps = hk.PedSignals(w, {"map": "T", "crossings": [crossing(0, ['a'], [['a'], ['b'], ['c']])]}, [],
                   send_heads=False)
first = {t: p for t, p, _ in run(ps, grp, 0.0, 45.0)}
check("before the order is known: b's end assumed to end the walk (safe)", first[26.0] in ('flash', 'stop'),
      first[26.0])
later = {t: p for t, p, _ in run(ps, grp, 45.0, 90.0)}
check("order known: walk runs through b and c (t = 60..79)", later[61.0] == 'walk' and later[78.0] == 'walk',
      (later[61.0], later[78.0]))
check("flash before a's green, then all-red", later[85.0] == 'flash' and later[88.6] == 'stop',
      (later[85.0], later[88.6]))
check("learned b -> c -> a", ps.junctions[0].nxt == {0: 1, 1: 2, 2: 0}, ps.junctions[0].nxt)

print("3. texel channels / テクセル")
cr = ps.crossings[0]
cr.phase = 'walk'
check("walk: G, A = steps / 10", cr.texel(6) == (0, 255, 0, 153), cr.texel(6))
cr.phase = 'flash'
check("flash: G + B, no column", cr.texel(0) == (0, 255, 255, 0))
cr.phase = 'stop'
check("stop: R, full red column", cr.texel(10) == (255, 0, 0, 255))

print("4. heads: RPC only on change, from the worker / 灯器: 変化時のみ")
a, b = FakeTL('a'), FakeTL('b')
grp = FakeGroup([[a], [b]], order=[1, 0])
w = FakeWorld([a, b], names=['Terrain_JpHokousha_T_000_1', 'Terrain_JpHokousha_T_0001', 'SM_Other'])
ps = hk.PedSignals(w, {"map": "T", "crossings": [crossing(0, ['a'], [['a'], ['b']])]}, [], carla_mod=FakeCarla)
check("head object found by name (not the 4-digit one)", ps.crossings[0].names == ['Terrain_JpHokousha_T_000_1'],
      ps.crossings[0].names)
for k in range(150):               # 30 s, 150 ticks / 30 秒
    ps.update(Snap(k * 0.2, {}))
    grp.advance(0.2)
    time.sleep(0.002)
time.sleep(0.2)
texels = [t for _, t in w.calls]
check("far fewer RPCs than ticks", 0 < len(w.calls) < 40, len(w.calls))
check("walk, flash, stop all shown", any(t[1] == 255 and t[2] == 0 for t in texels) and
      any(t[2] == 255 for t in texels) and any(t[0] == 255 for t in texels))
check("every RPC names the head", all(n == ['Terrain_JpHokousha_T_000_1'] for n, _ in w.calls))
ps.release_all()

print("5. walkers / 歩行者")
a, b = FakeTL('a'), FakeTL('b')
grp = FakeGroup([[a], [b]], order=[0, 1])           # conflicting phase first: STOP / 対立方向から
w = FakeWorld([a, b])
cons = {i: Con() for i in range(1, 6)}
walkers = [(i, cons[i], 1.3) for i in range(1, 6)]
ps = hk.PedSignals(w, {"map": "T", "crossings": [crossing(0, ['a'], [['a'], ['b']])]}, walkers, send_heads=False)
actors = {1: Walker(0.0, -5.0),                 # standing at the kerb / 縁で立っている
          2: Walker(0.5, -5.5, 0.0, 1.2),       # walking into the crossing / 横断歩道へ向かう
          3: Walker(0.0, 0.0, 0.0, 1.2),        # on the crossing / 横断中
          4: Walker(-2.0, -5.0, 1.3, 0.0),      # walking past along the kerb / 縁を通り過ぎる
          5: Walker(20.0, 20.0)}                # elsewhere / 別の場所
ps.update(Snap(0.0, actors))
check("standing walker held (speed 0)", cons[1].speeds == [0.0], cons[1].speeds)
check("walker heading in held", cons[2].speeds == [0.0], cons[2].speeds)
check("walker on the crossing not held (finish crossing)", cons[3].speeds == [], cons[3].speeds)
check("walker passing along the kerb not held", cons[4].speeds == [], cons[4].speeds)
check("walker elsewhere untouched", cons[5].speeds == [])
t = 0.0
while ps.crossings[0].phase != 'walk' and t < 40:
    grp.advance(0.2)
    t += 0.2
    ps.update(Snap(t, actors))
check("released at WALK with its own speed", cons[1].speeds == [0.0, 1.3] and cons[2].speeds == [0.0, 1.3],
      (cons[1].speeds, cons[2].speeds, round(t, 1)))
while ps.crossings[0].phase != 'flash' and t < 80:
    grp.advance(0.2)
    t += 0.2
    ps.update(Snap(t, actors))
check("no new start on flashing green (held again)", cons[1].speeds[-1] == 0.0, cons[1].speeds)
ps.release_all()
check("release_all restores speed", cons[1].speeds[-1] == 1.3)

print("5b. overlapping zones: only the crossing the walker heads into / 重なる待機範囲")
# crossing 1 runs across x (kerbs y = +-4) at x = 0; crossing 2 runs along y at y = -7,
# spanning x -4..4: its band lies inside crossing 1's lower waiting zone (y -4.5..-7).
# 横断歩道 2 の帯が横断歩道 1 の待機範囲に重なる
a, b = FakeTL('a'), FakeTL('b')
grp = FakeGroup([[a], [b]], order=[0, 1])           # a running: crossing 1 STOP, crossing 2 WALK
w = FakeWorld([a, b])
c2 = crossing(1, ['b'], [['a'], ['b']])
c2.update({"c": [0.0, -7.0], "n": [1.0, 0.0], "f": [0.0, 1.0],
           "ends": [[-4.0, -7.0], [4.0, -7.0]],
           "wait_zones": [[[-3.5, -9.5], [-7.0, -9.5], [-7.0, -4.5], [-3.5, -4.5]],
                          [[3.5, -9.5], [7.0, -9.5], [7.0, -4.5], [3.5, -4.5]]]})
cons = {i: Con() for i in range(1, 5)}
ps = hk.PedSignals(w, {"map": "T", "crossings": [crossing(0, ['a'], [['a'], ['b']]), c2]},
                   [(i, cons[i], 1.2) for i in cons], send_heads=False)
ps.update(Snap(0.0, {}))
check("crossing 1 stop, crossing 2 walk", [c.phase for c in ps.crossings] == ['stop', 'walk'],
      [c.phase for c in ps.crossings])
acts = {1: Walker(1.0, -7.0, 1.2, 0.0),     # on crossing 2's band, inside crossing 1's zone / 2 を横断中
        2: Walker(1.0, -7.0, 0.0, 0.0),     # standing on crossing 2's band / 2 の帯で静止
        3: Walker(0.0, -4.7, 0.0, 1.2),     # heading into crossing 1 (stop) / 1 へ向かう
        4: Walker(-5.0, -6.0, 1.2, 0.0)}    # heading into crossing 2 (walk), in both zones / 2 へ向かう
ps.next_t = None
ps.update(Snap(0.2, acts))
check("walker on another crossing's band never held", cons[1].speeds == [] and cons[2].speeds == [],
      (cons[1].speeds, cons[2].speeds))
check("walker heading into the stopped crossing held", cons[3].speeds == [0.0], cons[3].speeds)
check("walker heading into the walking crossing not held", cons[4].speeds == [], cons[4].speeds)

print("6. Japan only / 日本のマップのみ")
import generate_traffic as gt  # noqa: E402


class NoWorld(object):
    def get_map(self):
        return type('M', (), {'name': 'Carla/Maps/Town02'})()

    def __getattr__(self, k):
        raise AssertionError('world.%s called on a non-Japan map' % k)


check("non-Japan map: None, no world call", gt.setup_ped_signals(NoWorld(), [{'id': 1}], [None, None], [1.0]) is None)

print("7. RuleKeeper stops for a walker waiting at an unsignalised crossing / 待つ歩行者に停止")


class TM(object):
    def __init__(self):
        self.calls = []

    def vehicle_percentage_speed_difference(self, a, p):
        self.calls.append(('stop', p))

    def set_desired_speed(self, a, v):
        self.calls.append(('speed', v))

    def auto_lane_change(self, a, v):
        pass


class Map(object):
    name = 'X'

    def get_waypoint(self, loc, **kw):
        # carriageway: |y| < 4 / 車道 |y| < 4
        return object() if kw.get('project_to_road') is False and abs(loc.y) < 4.0 else (None if kw.get('project_to_road') is False else object())


class W2(object):
    def __init__(self, acts):
        self.acts = acts

    def get_actors(self, ids=None):
        class L(list):
            def filter(self, pat):
                return self
        return L(type('A', (), {'id': i})() for i in self.acts if i >= 100)

    def get_actor(self, i):
        return object()


cfg = {'period_s': 0.2, 'yield_to_walkers': {'enabled': True, 'lookahead_s': 2.5, 'min_lookahead_m': 12,
                                             'lateral_m': 4.5, 'release_s': 1.0, 'max_hold_s': 15, 'stuck_s': 8}}
tm = TM()
car = Walker(-10.0, -1.75, 0.0, 0.0)      # a car 10 m before the crossing, heading +x / 横断歩道の 10 m 手前
car.tf = Tf(-10.0, -1.75, 0.0)
acts = {1: car, 100: Walker(0.0, -5.0)}   # a walker standing at the kerb / 縁に立つ歩行者
rk = tp.RuleKeeper(W2(acts), Map(), tm, [1], cfg)
rk.crossings = [crossing(7, [], [], signal=False)]
rk.update(Snap(0.0, acts))
check("car stopped for the waiting walker", ('stop', 100.0) in tm.calls, tm.calls)
acts[100] = Walker(0.0, -12.0)           # walker gone from the kerb / 縁から離れた
for k in range(1, 12):
    rk.update(Snap(k * 0.2, acts))
check("released after the walker left", tm.calls[-1][0] == 'speed', tm.calls[-1])
tm2 = TM()
acts2 = {1: car, 100: Walker(-2.0, -5.0, 1.3, 0.0)}   # walking past / 通り過ぎる
rk2 = tp.RuleKeeper(W2(acts2), Map(), tm2, [1], cfg)
rk2.crossings = [crossing(7, [], [], signal=False)]
rk2.update(Snap(0.0, acts2))
check("no stop for a walker passing along the kerb", ('stop', 100.0) not in tm2.calls, tm2.calls)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
