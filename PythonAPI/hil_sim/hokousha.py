"""Japanese pedestrian signals (歩行者用信号) on Japan maps: one pedestrian
phase per signalised crossing, derived from the junction's vehicle signals,
shown on the heads and obeyed by the walkers.

generate_traffic.py makes a `PedSignals` once the walkers are started (Japan
profile maps only, and only if the map's build has hokousha.json from
maps/osm_city/hokousha_crossings.py) and calls `update(snapshot)` every tick.

The phase (the SAME value drives the head and the walkers):
  * the crossing's conflicting vehicle phases (the xodr controllers whose
    traffic drives straight through it) are known from hokousha.json; the
    order CARLA cycles a junction's phases in is learned at run time
    (CARLA registers them in its own order, not the xodr's);
  * STOP while a conflicting phase is active (green, yellow or its all-red);
  * otherwise T = time until the next conflicting phase turns green
    (from TrafficLight state / elapsed / green / yellow / red times):
      T > ALL_RED + F       WALK   (青, the cyan column counts down)
      ALL_RED < T <= +F     FLASH  (青点滅: do not start, finish crossing)
      T <= ALL_RED          STOP   (赤, all-red gap before the vehicles go)
    F = the 青点滅 time: the whole crossing at 1.0 m/s, 4..10 s.
    Before the order is known, the next phase is assumed to conflict (safe).
  * the red columns show the wait left, the cyan ones the walk left (10 steps).

Heads: World.apply_color_texture_to_objects(names, MaterialParameter.Emissive,
1x1 texel R=stop G=walk B=flash A=countdown), sent from a worker thread only
when a crossing's (state, steps) changes; the 1 Hz blink is in the material.

Walkers (CARLA's WalkerAIController, navigation in this client): a walker in
a kerb waiting zone that heads into (or stands at) the crossing while the
phase is not WALK is held with set_max_speed(0) and released with its own
speed when WALK starts; walkers already on the crossing are never held
(Japanese rule on flashing green: finish crossing); only the crossing the
walker heads into counts (zones of nearby crossings overlap). CARLA's own logic still
runs underneath (it waits for the nearest light's red and a clear road).

Cheap: client-side snapshot reads only (TrafficLight getters read the
episode snapshot), work every 0.2 s of sim time, RPCs off the tick thread.

日本の歩行者用信号: 交差点の車両信号から横断歩道ごとの歩行者現示を求め、灯器
の表示と歩行者の待機の両方に同じ値を使う。
"""

import json
import math
import os
import queue
import re
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
WALK, FLASH, STOP = 'walk', 'flash', 'stop'
ALL_RED = 2.0            # pedestrian red this long before the vehicles' green / 全赤
MIN_WALK = 5.0           # no walk shorter than this (short side phases of a long cycle) / 最短の青
SEGS = 10          # countdown segments per column (= jp_hokousha.SEGS) / 区画数
G_STATES = ('Green',)
Y_STATES = ('Yellow',)


def data_path(map_name):
    short = (map_name or '').rsplit('/', 1)[-1]
    return os.path.join(HERE, 'maps', 'build', short, 'hokousha.json')


def load(map_name):
    """hokousha.json of the loaded map, or None. / マップの横断歩道データ"""
    try:
        with open(data_path(map_name)) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def flash_time(length_m):
    """青点滅 time: the whole crossing at 1.0 m/s (someone who starts on the
    last walk second can finish), 4 to 10 s. / 青点滅の時間（全長 / 1.0 m/s）"""
    return max(4.0, min(10.0, float(length_m or 8.0) / 1.0))


def _state_name(st):
    return str(st).rsplit('.', 1)[-1]


def in_quad(x, y, quad):
    """Point in a convex quad (any winding). / 凸四角形の内外判定"""
    sign = 0
    for i in range(4):
        x0, y0 = quad[i]
        x1, y1 = quad[(i + 1) % 4]
        c = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0)
        if c != 0:
            s = 1 if c > 0 else -1
            if sign and s != sign:
                return False
            sign = s
    return True


class Junction(object):
    """A junction's phases (lists of TrafficLight actors) and the cycle order
    learned from what CARLA does. / 交差点の現示と、観測で学ぶ順序"""

    def __init__(self, phases):
        self.phases = phases          # [[tl, ...], ...]
        self.nxt = {}
        self.active = None
        self.t = 0.0                  # time into the active phase's cycle / 現示内の経過
        self.last_now = None

    def durations(self, k):
        tl = self.phases[k][0]
        return tl.get_green_time(), tl.get_yellow_time(), tl.get_red_time()

    def total(self, k):
        return sum(self.durations(k))

    def update(self, now):
        found = None
        for k, lights in enumerate(self.phases):
            st = _state_name(lights[0].get_state())
            if st in G_STATES:
                found = (k, lights[0].get_elapsed_time())
                break
            if st in Y_STATES:
                found = (k, self.durations(k)[0] + lights[0].get_elapsed_time())
                break
        if found is None:
            # all red: the phase in its all-red part is the one whose clock runs
            # 全赤: 時計が進んでいる現示が全赤中
            for k, lights in enumerate(self.phases):
                e = lights[0].get_elapsed_time()
                if e > 1e-3:
                    g, y, _r = self.durations(k)
                    found = (k, g + y + e)
                    break
        if found is None:
            if self.active is not None and self.last_now is not None:
                self.t += max(0.0, now - self.last_now)
        else:
            k, t = found
            if self.active is not None and k != self.active:
                self.nxt[self.active] = k
            self.active, self.t = k, t
        self.last_now = now

    def until_green(self, X):
        """Seconds until a phase in X turns green; None if the order is not
        known yet past the active phase. / X の現示が青になるまでの秒数"""
        if self.active is None:
            return None
        if self.active in X:
            return 0.0
        rem = max(0.0, self.total(self.active) - self.t)
        p = self.nxt.get(self.active)
        for _ in range(len(self.phases)):
            if p is None:
                return None
            if p in X:
                return rem
            rem += self.total(p)
            p = self.nxt.get(p)
        return None

    def until_free(self, X):
        """Seconds until no phase in X is active (the walk can start)."""
        if self.active is None or self.active not in X:
            return 0.0
        rem = max(0.0, self.total(self.active) - self.t)
        p = self.nxt.get(self.active)
        for _ in range(len(self.phases)):
            if p is None or p not in X:
                return rem
            rem += self.total(p)
            p = self.nxt.get(p)
        return rem


class Crossing(object):
    def __init__(self, rec, junction, X):
        self.id = rec['id']
        self.rec = rec
        self.j = junction
        self.X = X
        self.F = flash_time(rec.get('length'))
        n = rec['n']
        self.zones = [(rec['wait_zones'][0], (n[0], n[1])), (rec['wait_zones'][1], (-n[0], -n[1]))]
        c, f, w = rec['c'], rec['f'], rec['width']
        t0, t1 = rec['t0'], rec['t1']
        p = lambda t, a: (c[0] + n[0] * t + f[0] * a, c[1] + n[1] * t + f[1] * a)
        self.band = [p(t0, -w / 2), p(t1, -w / 2), p(t1, w / 2), p(t0, w / 2)]
        self.phase, self.frac = STOP, 1.0
        self.span = None              # (phase, length) for the countdown / 残り表示の全長
        self.names = []

    def compute(self):
        """(phase, remaining s or None). / 現示と残り時間"""
        j = self.j
        if j.active is None:
            return STOP, None
        if j.active in self.X:
            return STOP, j.until_free(self.X)
        T = j.until_green(self.X)
        if T is None:
            T = max(0.0, j.total(j.active) - j.t)    # assume the next phase conflicts / 安全側
        if T > ALL_RED + self.F:
            if self.phase == STOP and T - ALL_RED - self.F + j.t < MIN_WALK:
                # a short non-conflicting phase between conflicting ones: stay red
                # (the whole walk it would give, so a late first sample still walks)
                # 対立現示の間の短い現示: 青にしない（青の全長で判定）
                return STOP, T + self._conflict_run()
            return WALK, T - ALL_RED - self.F
        if T > ALL_RED and self.phase != STOP:       # flash only after a walk / 青の後だけ点滅
            return FLASH, T - ALL_RED
        return STOP, T + self._conflict_run()

    def _conflict_run(self):
        """Length of the conflicting phases that come next (the wait after T)."""
        j = self.j
        p = j.nxt.get(j.active)
        run, guard = 0.0, 0
        while p is not None and guard < len(j.phases):
            if p in self.X:
                run += j.total(p)
            elif run > 0.0:
                break
            p = j.nxt.get(p)
            guard += 1
        return run

    def update(self):
        phase, rem = self.compute()
        if self.span is None or self.span[0] != phase or (rem is not None and rem > self.span[1] + 0.5):
            self.span = (phase, rem)
        if phase == FLASH:
            frac = 0.0
        elif rem is None or not self.span[1]:
            frac = 1.0
        else:
            frac = max(0.0, min(1.0, rem / self.span[1]))
        self.phase, self.frac = phase, frac
        return phase, int(math.ceil(frac * SEGS - 1e-6))

    def texel(self, steps):
        r = 255 if self.phase == STOP else 0
        g = 255 if self.phase in (WALK, FLASH) else 0
        b = 255 if self.phase == FLASH else 0
        a = int(round(255 * steps / float(SEGS)))
        return (r, g, b, a)


class HeadSender(threading.Thread):
    """Sends head texels off the tick thread, latest value per crossing.
    / ティック外で灯器の表示を送る（横断歩道ごとに最新値のみ）"""

    def __init__(self, world, carla_mod):
        threading.Thread.__init__(self, name='hokousha-heads')
        self.daemon = True
        self.world, self.carla = world, carla_mod
        self.pending = {}
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.sent = 0
        self.errors = 0
        self.alive = True

    def push(self, cid, names, texel):
        with self.lock:
            self.pending[cid] = (names, texel)
        self.wake.set()

    def run(self):
        while self.alive:
            self.wake.wait(1.0)
            self.wake.clear()
            with self.lock:
                batch, self.pending = self.pending, {}
            groups = {}
            for cid, (names, texel) in batch.items():
                groups.setdefault(texel, []).extend(names)
            for texel, names in groups.items():
                try:
                    tex = self.carla.TextureColor(1, 1)
                    tex.set(0, 0, self.carla.Color(*texel))
                    self.world.apply_color_texture_to_objects(names, self.carla.MaterialParameter.Emissive, tex)
                    self.sent += 1
                except Exception as exc:       # never kill the rig / リグを止めない
                    self.errors += 1
                    if self.errors in (1, 10, 100):
                        print('[hokousha] head update failed (%d): %s / 灯器の更新失敗' % (self.errors, exc))


class PedSignals(object):
    """See the module doc. `walkers`: [(walker_id, controller_actor, speed)]."""

    def __init__(self, world, data, walkers, carla_mod=None, period_s=0.2, send_heads=True):
        self.world = world
        self.period = period_s
        self.next_t = None
        self.walkers = list(walkers)
        self.held = {}                 # walker id -> (controller, speed, since, crossing) / 待機中
        self.holds = 0
        tls = {}
        for tl in world.get_actors().filter('traffic.traffic_light'):
            try:
                tls[tl.get_opendrive_id()] = tl
            except Exception:
                pass
        juncs = {}
        self.crossings = []
        for rec in data.get('crossings', []):
            if not rec.get('signal') or not rec.get('phases'):
                continue
            key = rec.get('junction')
            if key not in juncs:
                phases = [[tls[s] for s in ph if s in tls] for ph in rec['phases']]
                if not all(phases):
                    continue
                juncs[key] = Junction(phases)
            j = juncs[key]
            X = {k for k, ph in enumerate(rec['phases']) if set(ph) & set(rec.get('conflict', []))}
            if X and len(X) < len(j.phases):
                self.crossings.append(Crossing(rec, j, X))
        self.junctions = list(juncs.values())
        self.sender = None
        if send_heads and carla_mod is not None:
            pat = re.compile(r'JpHokousha_%s_(\d{3})(?!\d)' % re.escape(data.get('map', '')))
            by_id = {c.id: c for c in self.crossings}
            try:
                for nm in world.get_names_of_all_objects():
                    mt = pat.search(nm)
                    if mt and int(mt.group(1)) in by_id:
                        by_id[int(mt.group(1))].names.append(nm)
            except Exception as exc:
                print('[hokousha] no object names: %s' % exc)
            if any(c.names for c in self.crossings):
                self.sender = HeadSender(world, carla_mod)
                self.sender.start()
        self.shown = {}

    def describe(self):
        return ('[hokousha] %d signalised crossings at %d junctions, %d with heads, %d walkers '
                '/ 歩行者用信号 %d か所' % (len(self.crossings), len(self.junctions),
                                         sum(1 for c in self.crossings if c.names), len(self.walkers),
                                         len(self.crossings)))

    def update(self, snapshot):
        now = snapshot.timestamp.elapsed_seconds
        if self.next_t is not None and now < self.next_t:
            return
        self.next_t = now + self.period
        for j in self.junctions:
            j.update(now)
        for c in self.crossings:
            phase, steps = c.update()
            if self.sender is not None and c.names and self.shown.get(c.id) != (phase, steps):
                self.shown[c.id] = (phase, steps)
                self.sender.push(c.id, c.names, c.texel(steps))
        self._walkers(now, snapshot)

    def _target(self, x, y, vx, vy, wid):
        """The one crossing this walker is about to enter, or None.
        Anyone on ANY crossing band is crossing (never held, whatever that
        crossing's phase). Otherwise, among the kerb zones it stands in, the
        crossing it heads into (best velocity match); standing still: the one
        it was held for, else the nearest kerb end.
        / この歩行者が入ろうとしている横断歩道（1 つ）。横断中は対象外"""
        for c in self.crossings:
            if in_quad(x, y, c.band):
                return None
        sp = math.hypot(vx, vy)
        best, best_score = None, None
        for c in self.crossings:
            for k, (zone, into) in enumerate(c.zones):
                if not in_quad(x, y, zone):
                    continue
                if sp >= 0.2:
                    dot = (vx * into[0] + vy * into[1]) / sp
                    if dot <= 0.3:
                        continue
                    score = dot
                else:
                    h = self.held.get(wid)
                    if h is not None and h[3] is c:
                        return c
                    e = c.rec['ends'][k]
                    score = -math.hypot(x - e[0], y - e[1])
                if best_score is None or score > best_score:
                    best, best_score = c, score
        return best

    def _walkers(self, now, snapshot):
        for wid, con, speed in self.walkers:
            act = snapshot.find(wid)
            if act is None:
                self.held.pop(wid, None)
                continue
            loc = act.get_transform().location
            vel = act.get_velocity()
            c = self._target(loc.x, loc.y, vel.x, vel.y, wid)
            hold = c is not None and c.phase != WALK
            h = self.held.get(wid)
            if hold and h is None:
                self.held[wid] = (con, speed, now, c)
                self.holds += 1
                con.set_max_speed(0.0)
            elif not hold and h is not None:
                del self.held[wid]
                con.set_max_speed(float(speed))

    def release_all(self):
        for wid, (con, speed, _t, _c) in list(self.held.items()):
            try:
                con.set_max_speed(float(speed))
            except Exception:
                pass
        self.held.clear()
        if self.sender is not None:
            self.sender.alive = False


def waiting_crossings(data, walkers_xy_vel, signalised=False):
    """Unsignalised crossings (or all, with signalised=True) with a walker in a
    kerb waiting zone heading into the crossing or standing there.
    `walkers_xy_vel`: [(x, y, vx, vy)]. Returns the crossing records.
    / 歩行者が待っている（信号の無い）横断歩道"""
    out = []
    for rec in (data or {}).get('crossings', []):
        if rec.get('signal') and not signalised:
            continue
        n = rec['n']
        for zone, into in ((rec['wait_zones'][0], (n[0], n[1])), (rec['wait_zones'][1], (-n[0], -n[1]))):
            hit = False
            for x, y, vx, vy in walkers_xy_vel:
                if in_quad(x, y, zone):
                    sp = math.hypot(vx, vy)
                    if sp < 0.2 or (vx * into[0] + vy * into[1]) > 0.3 * sp:
                        hit = True
                        break
            if hit:
                out.append(rec)
                break
    return out


def crossing_points(rec, step=2.0):
    """Points along the crossing's centre line, kerb to kerb (CARLA frame)."""
    c, n = rec['c'], rec['n']
    t, pts = rec['t0'], []
    while t <= rec['t1'] + 1e-6:
        pts.append((c[0] + n[0] * t, c[1] + n[1] * t))
        t += step
    return pts
