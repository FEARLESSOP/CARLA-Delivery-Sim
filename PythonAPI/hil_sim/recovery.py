"""Getting stuck AIs moving again: NPC cars, delivery pods, the hero on
autopilot (reverse out of a wedge) and walkers (a new route, then a move).

  StuckWatch     per actor: moved less than D m over the last T s?
  ReverseOut     a reverse manoeuvre for a car: back up BACK_M m with the
                 steering turning the nose towards its lane heading, then a
                 short brake; returns the control to apply each tick
  Unsticker      NPC cars of the Traffic Manager: a stuck car (not held by a
                 rule, not queued behind another car, not at a red light) is
                 taken off autopilot, reversed out, handed back; after TRIES
                 failed tries it is moved a few metres on along its lane
  WalkerRefresh  walkers: a walker still for a long time (not held at a
                 signalised crossing) gets a new destination; still stuck ->
                 moved to a random navmesh point

The pure parts (StuckWatch, ReverseOut.steer_for) have no CARLA dependency
(tests/test_recovery.py). / 動けなくなった車・ポッド・歩行者の復帰処理。
"""
import collections
import math
import random

BACK_M = 3.5             # reverse this far / 後退距離 [m]
BACK_MAX_S = 5.0         # or this long / 最長後退時間
BRAKE_S = 0.6
TRIES = 3                # manoeuvres before the car is moved / 移動までの回数
TRY_WINDOW_S = 180.0


class StuckWatch(object):
    """Rolling positions per actor; stuck = moved < dist in window seconds."""

    def __init__(self, window=20.0, dist=1.0):
        self.window, self.dist = window, dist
        self.hist = collections.defaultdict(collections.deque)

    def forget(self, aid):
        self.hist.pop(aid, None)

    def stuck(self, aid, x, y, t):
        h = self.hist[aid]
        h.append((t, x, y))
        while h and h[0][0] < t - self.window - 1.0:
            h.popleft()
        if not h or h[0][0] > t - self.window:
            return False                        # not watched long enough / 観測不足
        t0, x0, y0 = h[0]
        moved = max(math.hypot(px - x0, py - y0) for _, px, py in h)
        return moved < self.dist


class ReverseOut(object):
    """One reverse manoeuvre. ``step(dt, x, y, yaw_deg, lane_yaw_deg, lateral)``
    returns (throttle, brake, steer, reverse) or None when finished.
    ``lateral`` = car's offset from the lane centre, + = right of it (m)."""

    def __init__(self, x, y, back_m=BACK_M):
        self.x0, self.y0, self.back_m = x, y, back_m
        self.t = 0.0
        self.phase = "back"
        self.brake_t = 0.0

    @staticmethod
    def steer_for(yaw_deg, lane_yaw_deg, lateral):
        """Steer while reversing: the nose turns towards the lane heading and
        the car eases back towards the lane centre. / 後退中の操舵。"""
        e = (lane_yaw_deg - yaw_deg + 180.0) % 360.0 - 180.0      # + = nose must turn left
        # reversing with steer s turns the nose the opposite way to driving
        # forward: steer right to swing the nose left / 後退では逆に切る
        s = -e / 30.0 + lateral * 0.08
        return max(-0.6, min(0.6, s))

    def step(self, dt, x, y, yaw_deg, lane_yaw_deg, lateral):
        self.t += dt
        if self.phase == "back":
            if math.hypot(x - self.x0, y - self.y0) >= self.back_m or self.t > BACK_MAX_S:
                self.phase = "brake"
            else:
                return 0.4, 0.0, self.steer_for(yaw_deg, lane_yaw_deg, lateral), True
        if self.phase == "brake":
            self.brake_t += dt
            if self.brake_t < BRAKE_S:
                return 0.0, 0.7, 0.0, False
            return None
        return None


# ---------------------------------------------------------------- CARLA side
def _lane_frame(wmap, loc, carla):
    """(lane yaw deg, lateral offset m, waypoint) of the driving lane under loc."""
    wp = wmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return 0.0, 0.0, None
    tf = wp.transform
    r = tf.get_right_vector()
    lat = (loc.x - tf.location.x) * r.x + (loc.y - tf.location.y) * r.y
    return tf.rotation.yaw, lat, wp


class Unsticker(object):
    """Traffic Manager cars. ``update(now, dt, cars, exempt)``: ``cars`` =
    [(id, transform, velocity)], ``exempt(id)`` True while a rule holds the
    car. Returns the number of manoeuvres started. / TM 車両の復帰。"""

    def __init__(self, world, wmap, tm_port, log=print, window=20.0):
        import carla
        self.carla, self.world, self.map, self.tm_port = carla, world, wmap, tm_port
        self.log = log
        self.watch = StuckWatch(window, 1.0)
        self.active = {}          # id -> ReverseOut
        self.tries = collections.defaultdict(list)
        self.lights = None
        self.count = {"reversed": 0, "moved": 0}

    def _stop_points(self):
        if self.lights is None:
            self.lights = []
            for tl in self.world.get_actors().filter("traffic.traffic_light*"):
                try:
                    self.lights += [w.transform.location for w in tl.get_stop_waypoints()]
                except RuntimeError:
                    pass
        return self.lights

    def _queued(self, aid, tf, cars):
        """Another car within 9 m ahead in roughly the same heading: a queue."""
        f = tf.get_forward_vector()
        for oid, otf, _ in cars:
            if oid == aid:
                continue
            dx, dy = otf.location.x - tf.location.x, otf.location.y - tf.location.y
            ahead = dx * f.x + dy * f.y
            side = abs(-dx * f.y + dy * f.x)
            if 0.0 < ahead < 9.0 and side < 2.0:
                return True
        return False

    def update(self, now, dt, cars, exempt=lambda aid: False):
        started = 0
        for aid, tf, vel in cars:
            L = tf.location
            if abs(L.x) < 0.01 and abs(L.y) < 0.01 and abs(L.z) < 0.01:
                continue                  # no valid position this tick / 位置が無効
            act = None
            if aid in self.active:
                act = self.world.get_actor(aid)
                if act is None:
                    del self.active[aid]
                    continue
                yaw, lat, _ = _lane_frame(self.map, L, self.carla)
                out = self.active[aid].step(dt, L.x, L.y, tf.rotation.yaw, yaw, lat)
                if out is None:
                    act.apply_control(self.carla.VehicleControl())
                    act.set_autopilot(True, self.tm_port)
                    del self.active[aid]
                    self.watch.forget(aid)
                else:
                    thr, brk, st, rev = out
                    act.apply_control(self.carla.VehicleControl(throttle=thr, brake=brk, steer=st, reverse=rev))
                continue
            if not self.watch.stuck(aid, L.x, L.y, now):
                continue
            if exempt(aid) or self._queued(aid, tf, cars):
                self.watch.forget(aid)
                continue
            if any(L.distance(p) < 12.0 for p in self._stop_points()):
                self.watch.forget(aid)
                continue
            act = self.world.get_actor(aid)
            if act is None:
                continue
            tl = self.tries[aid] = [t for t in self.tries[aid] if now - t < TRY_WINDOW_S]
            if len(tl) >= TRIES:
                # move it on along its lane / 車線に沿って先へ移す
                _, _, wp = _lane_frame(self.map, L, self.carla)
                nxt = wp.next(12.0) if wp is not None else []
                if nxt:
                    t2 = nxt[0].transform
                    t2.location.z += 0.3
                    act.set_transform(t2)
                    self.count["moved"] += 1
                    self.log("[recovery] car %d moved 12 m on after %d failed reverses" % (aid, TRIES))
                self.tries[aid] = []
                self.watch.forget(aid)
                continue
            tl.append(now)
            act.set_autopilot(False, self.tm_port)
            self.active[aid] = ReverseOut(L.x, L.y)
            self.count["reversed"] += 1
            started += 1
            self.log("[recovery] car %d stuck at (%.1f, %.1f): reversing out" % (aid, L.x, L.y))
        return started


class WalkerRefresh(object):
    """Walkers that stopped moving. ``update(now, walkers, held_ids)`` with
    ``walkers`` = [(walker actor, controller actor)]. / 歩行者の復帰。"""

    def __init__(self, world, log=print, window=90.0):
        self.world, self.log = world, log
        self.watch = StuckWatch(window, 0.5)
        self.tries = collections.defaultdict(int)
        self.count = {"rerouted": 0, "moved": 0}

    def update(self, now, walkers, held_ids=(), snapshot=None):
        for wk, con in walkers:
            s = snapshot.find(wk.id) if snapshot is not None else None
            if snapshot is not None and s is None:
                continue                  # gone (hit by a car, destroyed) / 消えた
            L = s.get_transform().location if s is not None else wk.get_location()
            if abs(L.x) < 0.01 and abs(L.y) < 0.01 and abs(L.z) < 0.01:
                continue
            if wk.id in held_ids:
                self.watch.forget(wk.id)
                continue
            if not self.watch.stuck(wk.id, L.x, L.y, now):
                continue
            self.watch.forget(wk.id)
            self.tries[wk.id] += 1
            dest = self.world.get_random_location_from_navigation()
            if dest is None:
                continue
            if self.tries[wk.id] <= 2:
                try:
                    con.go_to_location(dest)
                    self.count["rerouted"] += 1
                except RuntimeError:
                    pass
            else:
                try:
                    wk.set_location(dest)
                    con.go_to_location(self.world.get_random_location_from_navigation() or dest)
                    self.count["moved"] += 1
                    self.tries[wk.id] = 0
                    self.log("[recovery] walker %d moved to a new pavement point" % wk.id)
                except RuntimeError:
                    pass
