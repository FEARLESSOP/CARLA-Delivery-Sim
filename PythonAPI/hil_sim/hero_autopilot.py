"""Smooth autopilot for your own car (P in the driving window).

CARLA's Traffic Manager drove the hero before. On these maps it capped the car
at 30 km/h (it reads the speed from signs, and CARLA cannot read them on
keep-left roads), and its speed controller works the pedals on/off. This
client-side driver replaces it:

  - route: CARLA's LocalPlanner (roams, picks a random way at junctions, like
    the Traffic Manager); steering: pure pursuit on that route with a
    speed-dependent look-ahead, low-passed and rate limited (``Lateral``),
    never past the steer the G923 itself reaches (0.5), so the real wheel
    and the car's own wheel can show it 1:1;
  - speed: the lane's own limit (lane_limits.py: 100 on the ring, the real
    limits elsewhere), lowered ahead of a lower limit, a bend or a turn, a red
    or yellow light, the car in front and pedestrians on the path, each as
    "the speed from which a firm but comfortable deceleration still makes it";
    the turning lanes inside a junction carry no limit of their own (only
    the bend's sideways grip counts);
  - pedals: a speed loop gives a wanted acceleration (jerk limited), which
    becomes throttle or brake through a feed-forward plus integral term,
    with a coast band between the two and rate limits on both pedals, so it
    never pumps. The car's automatic gearbox shifts.

``Pilot.step(dt)`` returns the carla.VehicleControl to apply; the pure maths
(``Longitudinal``, ``curve_speeds``, ``stop_speed``) runs without CARLA
(tests/test_hero_autopilot.py).

自車用のなめらかな自動運転（運転ウインドウの P）。速度は車線の制限速度（カーブ・
曲がり角・信号・前車・歩行者の手前で減速）、ペダルは加速度指令を躍度制限し、
アクセルとブレーキの間に惰行帯を設けて変化率も制限するため、踏み直しをしない。
"""

import math

V_MIN_STOP = 0.4          # m/s: below this and wanting 0, hold the brake / 停止保持
V_STOP_TARGET = 0.6       # m/s: a target this low means stop, not creep / これ以下は停止扱い


def stop_speed(dist, decel, v_end=0.0):
    """Highest speed now from which ``decel`` still gets to ``v_end`` within
    ``dist`` metres. / dist m 先で v_end にできる現在の最高速度。"""
    return math.sqrt(max(0.0, v_end * v_end + 2.0 * decel * max(0.0, dist)))


def curve_speeds(points, lat_accel=3.6, decel=3.0, window=6.0):
    """Speed limits from the bends of a path: ``points`` = [(s, x, y, yaw_deg)]
    along it (s from the car). Returns the lowest "speed now" any bend asks
    for (inf on a straight). / 経路上のカーブから現在許される最高速度。"""
    best = float("inf")
    j = 0
    for i in range(len(points)):
        s0 = points[i][0]
        while j < len(points) and points[j][0] - s0 < window:
            j += 1
        if j >= len(points):
            break
        ds = points[j][0] - s0
        if ds <= 0.5:
            continue
        dyaw = abs((points[j][3] - points[i][3] + 180.0) % 360.0 - 180.0)
        k = math.radians(dyaw) / ds                  # curvature 1/m / 曲率
        if k < 2e-3:
            continue
        v_bend = math.sqrt(lat_accel / k)
        best = min(best, stop_speed(s0, decel, v_bend))
    return best


class Longitudinal(object):
    """Target speed -> throttle / brake without pumping.

    a_ref = k_v * (v_target - v) plus the rate the target falls at (a planned
    stop's deceleration as feed-forward, so it does not lag behind the
    profile), clamped to [-brake_max, accel_max] and jerk limited; pedal = a_ref / full-scale + drag + integral of the
    acceleration error. Pedal > 0 is throttle, below -coast is brake, the band
    between is coasting (engine braking). Both pedals are rate limited.
    目標速度からアクセル・ブレーキへ（加速度指令・躍度制限・惰行帯・変化率制限）。
    """

    def __init__(self, accel_max=3.4, brake_max=5.0, k_v=1.2, jerk_up=4.5, jerk_down=7.0,
                 throttle_full=4.5, brake_full=9.0, ki=0.25, coast=0.06,
                 throttle_rate=1.6, throttle_rate_down=1.2, brake_rate_up=2.5, brake_rate_down=3.0):
        self.accel_max, self.brake_max, self.k_v = accel_max, brake_max, k_v
        self.jerk_up, self.jerk_down = jerk_up, jerk_down
        self.throttle_full, self.brake_full, self.ki, self.coast = throttle_full, brake_full, ki, coast
        self.throttle_rate, self.throttle_rate_down = throttle_rate, throttle_rate_down
        self.brake_rate_up, self.brake_rate_down = brake_rate_up, brake_rate_down
        self.reset()

    def reset(self):
        self.a_ref = 0.0
        self.integral = 0.0
        self.throttle = self.brake = 0.0
        self.a_meas = 0.0
        self.a_ff = 0.0
        self._vt_prev = None

    def step(self, dt, v, v_target, a_meas=None, urgent=False):
        """One control step; returns (throttle, brake). ``urgent`` lifts the
        braking limits (something close ahead). / 1 ステップ分のペダル。"""
        dt = max(dt, 1e-3)
        bmax = 8.0 if urgent else self.brake_max
        # the target's own fall rate (filtered): a planned stop asks for exactly
        # that deceleration / 目標速度の低下率（計画停止の減速度）を前もって加える
        if self._vt_prev is not None and v_target < v + 3.0:
            rate = max(-bmax, min(0.0, (v_target - self._vt_prev) / dt))
        else:
            rate = 0.0
        self._vt_prev = v_target
        self.a_ff += (rate - self.a_ff) * min(1.0, dt / 0.25)
        want = max(-bmax, min(self.accel_max, self.a_ff + self.k_v * (v_target - v)))
        stopping = v_target < V_STOP_TARGET
        if stopping and v < 2.0:
            want = min(want, -1.5)          # come to rest firmly / しっかり停止
        jd = 20.0 if urgent else self.jerk_down
        self.a_ref += max(-jd * dt, min(self.jerk_up * dt, want - self.a_ref))
        if a_meas is not None:
            self.a_meas += (a_meas - self.a_meas) * min(1.0, dt / 0.3)
            err = self.a_ref - self.a_meas
            # small authority: in a tight turn the tyres scrub speed, which
            # must not read as "more throttle" / 旋回中の減速でアクセルを足さない
            if v > v_target + 0.3 and err > 0.0:
                err = 0.0
            self.integral = max(-0.12, min(0.12, self.integral + self.ki * err * dt))
        drag = 0.10 + 0.00022 * v * v if v > 0.5 else 0.0     # rolling + air / 走行抵抗
        if self.a_ref >= 0.0:
            u = self.a_ref / self.throttle_full + drag + self.integral
        else:
            u = self.a_ref / self.brake_full + min(drag, 0.05) + min(self.integral, 0.0)
        if stopping and v < V_MIN_STOP:
            u = -0.35                       # hold at a stop / 停止保持
            self.integral = 0.0
        thr = max(0.0, min(1.0, u))
        brk = max(0.0, min(1.0, (-u - self.coast) / (1.0 - self.coast))) if u < -self.coast else 0.0
        self.throttle += max(-self.throttle_rate_down * dt, min(self.throttle_rate * dt, thr - self.throttle))
        up = (6.0 if urgent else self.brake_rate_up) * dt
        self.brake += max(-self.brake_rate_down * dt, min(up, brk - self.brake))
        if self.brake > 0.02:
            self.throttle = max(0.0, self.throttle - self.throttle_rate_down * dt)
        return round(self.throttle, 4), round(self.brake, 4)


def wheel_for_steer(steer, gain=1.83, curve=0.5, clamp=1.0):
    """The G923 position (-1..1) that gives CARLA ``steer`` with the driving
    window's mapping steer = 0.5 * clamp(gain * tan(curve * x)).
    CARLA の steer を出すハンドル位置（運転ウインドウの式の逆）。"""
    s = max(-clamp, min(clamp, 2.0 * steer))
    return max(-1.0, min(1.0, math.atan(s / gain) / curve))


class Lateral(object):
    """Pure pursuit: aim at the route point ``look`` metres ahead (4 m + 0.3 s
    of travel), turn the front wheels by atan(2 L sin(a) / look), then
    low-pass and rate limit the command. / 純追従による操舵（平滑・変化率制限）。"""

    def __init__(self, wheelbase=2.7, max_angle_deg=70.0, max_steer=0.5, tau=0.12, rate=0.9):
        self.L, self.max_angle = wheelbase, math.radians(max_angle_deg)
        self.max_steer, self.tau, self.rate = max_steer, tau, rate
        self.steer = 0.0
        self.k = 0.0

    @staticmethod
    def look(v):
        return max(5.0, min(28.0, 4.0 + 0.3 * v))

    def step(self, dt, x, y, yaw_deg, v, path, ratio=1.0):
        """``path`` = [(s, px, py, ...)] ahead of the car, s along the route
        from the point nearest it; ``ratio`` = how much of the commanded angle
        CARLA really gives at this speed. Sets ``self.k`` (the curvature the
        car is steering for)."""
        ld = self.look(v)
        tgt = None
        for p in path:
            if p[0] >= ld:
                tgt = p
                break
        if tgt is None and path:
            tgt = path[-1]
        want = self.steer
        self.k = 0.0
        if tgt is not None:
            yaw = math.radians(yaw_deg)
            dx, dy = tgt[1] - x, tgt[2] - y
            a = math.atan2(dy, dx) - yaw
            a = (a + math.pi) % (2 * math.pi) - math.pi
            dist = max(math.hypot(dx, dy), 1.0)
            self.k = abs(2.0 * math.sin(a) / dist)
            delta = math.atan2(2.0 * self.L * math.sin(a), dist)
            want = delta / (self.max_angle * max(ratio, 0.2))
        want = max(-self.max_steer, min(self.max_steer, want))
        k = min(1.0, dt / self.tau)
        lp = self.steer + (want - self.steer) * k
        step = self.rate * dt
        self.steer += max(-step, min(step, lp - self.steer))
        return self.steer


def pick_branch(turns, rng, straight_weight=4.0, uturn_deg=135.0):
    """Index of the junction branch to take, from each branch's heading change
    (deg, signed). U-turns are never taken; going straight is ``straight_weight``
    times as likely as a turn (keeps the car on the main road, at speed).
    交差点で進む枝（U ターンは選ばない、直進を優先）。"""
    ok = [i for i, t in enumerate(turns) if abs(t) < uturn_deg]
    if not ok:
        return min(range(len(turns)), key=lambda i: abs(turns[i]))
    w = [straight_weight if abs(turns[i]) < 30.0 else 1.0 for i in ok]
    return rng.choices(ok, w)[0]


# --------------------------------------------------------------- CARLA side
class Pilot(object):
    """Drives ``vehicle``. Build it when autopilot goes on, call ``step(dt)``
    every frame and apply the returned control. / 自車を運転する。"""

    LOOK_MIN = 45.0           # m of path always checked / 常に確認する距離
    COMFORT = 3.0             # m/s^2 planned deceleration / 計画減速度
    GAP = 5.0                 # m kept to the car in front at rest / 停止時の車間
    HEADWAY = 1.4             # s of following distance / 車間時間

    def __init__(self, vehicle, wmap, speed_fn, dt=0.05):
        import random
        import carla
        from agents.navigation.local_planner import LocalPlanner, RoadOption
        self.carla = carla
        self.vehicle, self.wmap, self.speed_fn = vehicle, wmap, speed_fn
        self.world = vehicle.get_world()
        rng = random.Random()

        class RoamPlanner(LocalPlanner):
            """LocalPlanner whose random junction choice never takes a U-turn
            (netconvert adds turnaround lanes) and prefers going straight.
            U ターンを選ばず直進を優先する LocalPlanner。"""

            def _compute_next_waypoints(self, k=1):
                q = self._waypoints_queue
                for _ in range(min(q.maxlen - len(q), k)):
                    last = q[-1][0]
                    nxt = list(last.next(self._sampling_radius))
                    if not nxt:
                        break
                    if len(nxt) == 1:
                        q.append((nxt[0], RoadOption.LANEFOLLOW))
                        continue
                    turns = []
                    for wp in nxt:
                        end = wp
                        for _ in range(30):              # to the junction's exit / 出口まで
                            more = end.next(2.0)
                            if not more:
                                break
                            end = more[0]
                            if not end.is_junction:
                                break
                        turns.append((end.transform.rotation.yaw - last.transform.rotation.yaw
                                      + 180.0) % 360.0 - 180.0)
                    i = pick_branch(turns, rng)
                    opt = (RoadOption.STRAIGHT if abs(turns[i]) < 30.0 else
                           RoadOption.LEFT if turns[i] < 0 else RoadOption.RIGHT)
                    q.append((nxt[i], opt))

        self.lp = RoamPlanner(vehicle, opt_dict={
            "dt": dt, "target_speed": 30.0, "sampling_radius": 2.0,
            "lateral_control_dict": {"K_P": 1.4, "K_I": 0.05, "K_D": 0.25, "dt": dt},
            "max_throttle": 1.0, "max_brake": 1.0, "max_steering": 0.5,
            "base_min_distance": 3.0, "distance_ratio": 0.2}, map_inst=wmap)
        self.lon = Longitudinal()
        # the car's geometry and CARLA's speed-dependent steering curve
        # 車の寸法と CARLA の速度感応ステアリング
        wb, ang, self._curve = 2.7, 70.0, []
        try:
            pc = vehicle.get_physics_control()
            w = pc.wheels
            ang = max(w[0].max_steer_angle, 1.0)
            a, b = w[0].position, w[2].position        # front-left, rear-left (cm)
            wb = max(1.5, math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2) / 100.0)
            self._curve = sorted((p.x, p.y) for p in pc.steering_curve)
        except (RuntimeError, AttributeError, IndexError):
            pass
        self.lat = Lateral(wheelbase=wb, max_angle_deg=ang)
        self._road_kmh = None
        # 止まれ: the map's 206 stop lines (traffic_profile.StopLines) / 一時停止
        self.stops = None
        try:
            import traffic_profile
            sl = traffic_profile.StopLines(wmap, {"approach_m": 40.0})
            self.stops = sl if sl.stops else None
        except Exception:
            self.stops = None
        self._stop = None            # {"i", "t_still", "done"} / 停止中の状態
        self._stop_done = {}         # stop index -> sim time passed / 通過済み
        # wedged: no progress with throttle on -> reverse out / 動けない: 後退
        import recovery
        self._rec = recovery
        self._reverse = None
        self._no_progress = 0.0
        self._reverses = []
        self.half_len = vehicle.bounding_box.extent.x
        self._actors_t = -1e9
        self._vehicles, self._walkers, self._lights = [], [], []
        self._stops = {}                 # light id -> [(x, y)] stop points / 停止線
        self._limit_cache = {}
        self._t = 0.0
        self.target_kmh = 0.0
        self.reason = ""

    # -- the world around / 周囲
    def _refresh(self):
        if self._t - self._actors_t < 1.0:
            return
        self._actors_t = self._t
        acts = self.world.get_actors()
        self._vehicles = [a for a in acts.filter("vehicle.*") if a.id != self.vehicle.id]
        self._walkers = list(acts.filter("walker.pedestrian.*"))
        lights = list(acts.filter("traffic.traffic_light*"))
        if len(lights) != len(self._lights):
            self._stops = {}
        self._lights = lights
        for tl in lights:
            if tl.id not in self._stops:
                try:
                    self._stops[tl.id] = [(w.transform.location.x, w.transform.location.y, w.road_id,
                                           w.lane_id) for w in tl.get_stop_waypoints()]
                except RuntimeError:
                    self._stops[tl.id] = []

    def _ratio(self, kmh):
        """CARLA's steering curve at this speed (1 = full angle)."""
        c = self._curve
        if not c:
            return 1.0
        if kmh <= c[0][0]:
            return c[0][1]
        for (x0, y0), (x1, y1) in zip(c, c[1:]):
            if kmh <= x1:
                return y0 + (y1 - y0) * (kmh - x0) / max(x1 - x0, 1e-6)
        return c[-1][1]

    def _lane_kmh(self, wp):
        key = (wp.road_id, wp.section_id, wp.lane_id)
        if key not in self._limit_cache:
            self._limit_cache[key] = self.speed_fn(wp)
        return self._limit_cache[key]

    def _path(self, loc, look):
        """[(s, x, y, yaw, wp)] along the planned route, s from the route point
        nearest the car. Route points the car has already passed are dropped
        (else a missed turn leaves them in the plan and the car circles back).
        計画経路（通過済みの点は捨てる）。"""
        q = self.lp.get_plan()
        best, bi = 1e18, 0
        for i, (wp, _) in enumerate(q):
            if i > 60:
                break
            L = wp.transform.location
            d = (L.x - loc.x) ** 2 + (L.y - loc.y) ** 2
            if d < best:
                best, bi = d, i
        for _ in range(min(bi, len(q) - 2)):
            q.popleft()
        out, s, px, py = [], 0.0, None, None
        for wp, _ in q:
            L = wp.transform.location
            if px is not None:
                s += math.hypot(L.x - px, L.y - py)
            px, py = L.x, L.y
            out.append((s, L.x, L.y, wp.transform.rotation.yaw, wp))
            if s > look:
                break
        return out

    @staticmethod
    def _along(path, x, y, lateral):
        """s of the path point nearest (x, y) when within ``lateral`` m, else None."""
        best, bs = lateral * lateral, None
        for s, px, py, _, _ in path:
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best:
                best, bs = d, s
        return bs

    def step(self, dt):
        self._t += dt
        self._refresh()
        tf = self.vehicle.get_transform()
        loc = tf.location
        vel = self.vehicle.get_velocity()
        v = math.hypot(vel.x, vel.y)
        acc = self.vehicle.get_acceleration()
        fwd = tf.get_forward_vector()
        a_long = acc.x * fwd.x + acc.y * fwd.y
        self.lp.run_step()                        # keeps the route ahead / 経路を更新
        look = max(self.LOOK_MIN, v * v / (2.0 * self.COMFORT) + 25.0)
        path = self._path(loc, look)
        steer = self.lat.step(dt, loc.x, loc.y, tf.rotation.yaw, v, path, self._ratio(v * 3.6))
        here = self.wmap.get_waypoint(loc)
        # inside a junction the road's limit carries on (the turning lanes'
        # own low limits are not used: the bend decides) / 交差点内は道路の制限を継続
        if here is not None and not here.is_junction:
            self._road_kmh = self._lane_kmh(here)
        kmh = self._road_kmh or (self._lane_kmh(here) if here is not None else 30.0)
        target, reason, urgent = kmh / 3.6, "limit", False
        # lower limits ahead / 先の低い制限速度
        last = None
        for s, _, _, _, wp in path[::5]:
            key = (wp.road_id, wp.section_id, wp.lane_id)
            if key == last or wp.is_junction:
                continue
            last = key
            lim = self._lane_kmh(wp) / 3.6
            vt = stop_speed(s - 5.0, self.COMFORT, lim)
            if vt < target:
                target, reason = vt, "limit ahead"
        vt = curve_speeds([p[:4] for p in path], decel=self.COMFORT)
        if vt < target:
            target, reason = vt, "bend"
        if self.lat.k > 0.02:            # turning now: as fast as grip allows / 旋回中の上限
            vt = math.sqrt(3.6 / self.lat.k)
            if vt < target:
                target, reason = vt, "turn"
        front = self.half_len
        # red / yellow lights on the path / 経路上の赤・黄信号
        TL = self.carla.TrafficLightState
        for tl in self._lights:
            st = tl.state
            if st not in (TL.Red, TL.Yellow):
                continue
            for x, y, road, lane in self._stops.get(tl.id, ()):
                s = self._along(path, x, y, 2.5)
                if s is None or s < front - 1.0:
                    continue
                d = s - front - 1.5
                if st == TL.Yellow and d < v * v / (2.0 * 3.0):
                    continue                       # too late to stop: go / 止まれない
                vt = stop_speed(d, self.COMFORT + 0.5)
                if vt < target:
                    target, reason = vt, "red light" if st == TL.Red else "yellow light"
                    urgent = urgent or d < 8.0
        # cars and pedestrians on the path / 経路上の車・歩行者
        for a, lat, gap in [(a, 2.0, self.GAP) for a in self._vehicles] + \
                           [(w, 2.6, 4.0) for w in self._walkers]:
            al = a.get_location()
            if abs(al.x - loc.x) > look + 10 or abs(al.y - loc.y) > look + 10:
                continue
            s = self._along(path, al.x, al.y, lat)
            if s is None:
                continue
            other = getattr(a, "bounding_box", None)
            d = s - front - (other.extent.x if other is not None else 0.5)
            if d < -1.0:
                continue
            av = a.get_velocity()
            v_other = max(0.0, av.x * math.cos(math.radians(tf.rotation.yaw)) +
                          av.y * math.sin(math.radians(tf.rotation.yaw)))
            want_gap = gap + self.HEADWAY * v
            vt = stop_speed(d - want_gap, self.COMFORT + 0.5, v_other) if d > want_gap else \
                min(v_other, max(0.0, (d - gap) / max(self.HEADWAY, 0.1)))
            if vt < target:
                target = vt
                reason = "pedestrian" if a.type_id.startswith("walker") else "car ahead"
                urgent = urgent or d < gap + 0.6 * v
        # 止まれ: stop at the line, stand 2 s, then go / 停止線で 2 秒止まって進む
        if self.stops is not None and here is not None:
            hit = self.stops.ahead(here)
            if hit is not None:
                i, d = hit
                if self._t - self._stop_done.get(i, -1e9) > 20.0:
                    st = self._stop
                    if st is None or st["i"] != i:
                        st = self._stop = {"i": i, "still": 0.0}
                    d_line = d - self.half_len - 0.3
                    if d_line > 0.5 or v > 0.3:
                        vt = stop_speed(d_line, self.COMFORT)
                        if vt < target:
                            target, reason = vt, "stop line"
                        if d_line <= 0.5 and v <= 0.3:
                            st["still"] += dt
                    else:
                        st["still"] += dt
                        target, reason = 0.0, "stop line"
                    if st["still"] >= 2.0:
                        self._stop_done[i] = self._t
                        self._stop = None
        # reverse out when wedged (throttle on, no progress) / 動けなければ後退
        if self._reverse is not None:
            ly, lat, _ = self._rec._lane_frame(self.wmap, loc, self.carla)
            o = self._reverse.step(dt, loc.x, loc.y, tf.rotation.yaw, ly, lat)
            if o is not None:
                thr, brk, st_, rev = o
                self.target_kmh, self.reason = 0.0, "reversing out"
                return self.carla.VehicleControl(throttle=thr, brake=brk, steer=st_, reverse=rev)
            self._reverse = None
            self.lon.reset()
            self.lat.steer = 0.0
        if target > 1.0 and v < 0.15 and self.lon.throttle > 0.25:
            self._no_progress += dt
        else:
            self._no_progress = 0.0
        self._reverses = [t for t in self._reverses if self._t - t < 120.0]
        if self._no_progress > 6.0 and len(self._reverses) < 3:
            self._reverses.append(self._t)
            self._no_progress = 0.0
            self._reverse = self._rec.ReverseOut(loc.x, loc.y, back_m=3.0)
        self.target_kmh, self.reason = target * 3.6, reason
        thr, brk = self.lon.step(dt, v, target, a_long, urgent)
        out = self.carla.VehicleControl()
        out.steer = steer
        out.throttle, out.brake = thr, brk
        out.hand_brake, out.reverse, out.manual_gear_shift = False, False, False
        return out
