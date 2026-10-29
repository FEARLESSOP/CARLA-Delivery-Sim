"""Gym environment over CARLA for the HIL study (docs/RL_DESIGN.md §2–§3).

The agent drives an `ego` vehicle (role_name='ego') along a random route.
Observation, action, episode rules and rewards follow the design doc; the
reward thresholds are the SAME driver-alert rules the human sees
(rl/rules.py), so human feedback and agent training agree on what "bad
driving" is. The measurement math (lane offset, TTC/THW cone, body-frame
neighbours) is ported from dashboard.py's telemetry worker.

Usage / 使い方:
    from rl.carla_env import CarlaHILEnv
    env = CarlaHILEnv(host="127.0.0.1", port=2000)
    obs, info = env.reset()
    obs, r, term, trunc, info = env.step([steer, accel])

Run CARLA first (or the pipeline with --no-traffic). Training uses
synchronous mode at 20 Hz; pass sync=False only when a human is driving
live in the same world (HIL co-simulation).

HIL研究用のCARLA Gym環境（docs/RL_DESIGN.md §2–§3）。エージェントは
`ego`（role_name='ego'）をランダム経路に沿って運転する。報酬のしきい値は
人間ドライバーが受け取る警告ルール（rl/rules.py）と同一。車線オフセット・
TTC/THW・車体座標系の近傍車両などの計算は dashboard.py から移植。
学習時は同期モード20Hz。人間が同一ワールドで運転するHIL共走評価時のみ
sync=False を指定する。
"""

import glob
import math
import os
import random
import sys
import weakref

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover
    raise ImportError(
        "gymnasium is required: pip install gymnasium / "
        "gymnasium が必要です: pip install gymnasium")

# CARLA egg + agents package (GlobalRoutePlanner) live one level up.
# CARLA egg と agents パッケージ（GlobalRoutePlanner）は1階層上にある。
_HIL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PYAPI_DIR = os.path.dirname(_HIL_DIR)
try:
    sys.path.append(glob.glob(os.path.join(
        _PYAPI_DIR, 'carla', 'dist', 'carla-*%d.%d-%s.egg' % (
            sys.version_info.major, sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass
sys.path.append(os.path.join(_PYAPI_DIR, 'carla'))

import carla  # noqa: E402
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402

from rl.rules import REWARD_RULES, step_reward  # noqa: E402

K_NEIGHBORS = 6        # nearest vehicles observed / 観測する近傍車両数
N_WAYPOINTS = 10       # route look-ahead points / 経路先読み点数
WP_STRIDE_M = 3.0      # spacing of look-ahead points (m) / 先読み点の間隔
OBS_DIM = 10 + K_NEIGHBORS * 4 + N_WAYPOINTS * 2
FIXED_DT = 0.05        # 20 Hz synchronous tick / 同期Tick
EPISODE_TIMEOUT_S = 90.0
ROUTE_MIN_M, ROUTE_MAX_M = 300.0, 1000.0
ROUTE_DONE_M = 5.0     # distance to goal that counts as completion / 完走判定距離


def _yaw_to_forward_right(yaw_deg):
    """(forward, right) unit 2D vectors for a CARLA yaw. dashboard.pyと同一。"""
    yr = math.radians(yaw_deg)
    return ((math.cos(yr), math.sin(yr)),
            (math.cos(yr - math.pi / 2.0), math.sin(yr - math.pi / 2.0)))


def _front_obstacle(ego, others, max_dist=60.0, half_cone_deg=20.0):
    """Nearest vehicle ahead inside a cone -> (dist, ttc). dashboard.py移植."""
    t = ego.get_transform()
    (fx, fy), _ = _yaw_to_forward_right(t.rotation.yaw)
    hv = ego.get_velocity()
    hx, hy = t.location.x, t.location.y
    h_fwd = hv.x * fx + hv.y * fy
    best_d, best_ttc = max_dist, -1.0
    cos_lim = math.cos(math.radians(half_cone_deg))
    for a in others:
        al = a.get_location()
        dx, dy = al.x - hx, al.y - hy
        d = math.hypot(dx, dy)
        if d < 0.5 or d > max_dist:
            continue
        if (dx * fx + dy * fy) / d < cos_lim:
            continue
        av = a.get_velocity()
        closing = h_fwd - (av.x * fx + av.y * fy)
        if d < best_d:
            best_d = d
            best_ttc = (d / closing) if closing > 0.5 else -1.0
    return (best_d if best_d < max_dist else -1.0), best_ttc


class EgoSensing(object):
    """Per-tick measurements + observation vector for one vehicle.

    Shared by the Gym env (training) and agent_drive.py (deployment) so
    a trained policy sees identical features in both.
    1台の車両のTick毎計測値と観測ベクトル。Gym環境（学習）と
    agent_drive.py（実走）で共有し、学習時と同じ特徴量を保証する。
    """

    def __init__(self, world, vehicle, route):
        self.world = world
        self.map = world.get_map()
        self.vehicle = vehicle
        self.route = route or []            # [(waypoint, option), ...]
        # Cumulative route distance for progress reward.
        # 前進報酬用の経路累積距離。
        self.cumdist = [0.0]
        for i in range(1, len(self.route)):
            a = self.route[i - 1][0].transform.location
            b = self.route[i][0].transform.location
            self.cumdist.append(self.cumdist[-1] + a.distance(b))
        self.route_idx = 0
        # Histories for wide-window differentiation (see measure()).
        # 広い窓での微分用の履歴（measure() 参照）。
        self.speed_hist = []                # m/s along forward / 前方速度
        self.accel_hist = []
        self.last_red_tl = None

    # ------------------------------------------------------------------
    def _advance_route_index(self, loc):
        """Move route index to the nearest point in a forward window.

        経路インデックスを前方窓内の最近傍点まで進める（後退はしない）。
        """
        best_i, best_d = self.route_idx, float('inf')
        end = min(len(self.route), self.route_idx + 40)
        for i in range(self.route_idx, end):
            d = self.route[i][0].transform.location.distance(loc)
            if d < best_d:
                best_i, best_d = i, d
        self.route_idx = best_i
        return best_d

    def measure(self, dt):
        """Compute one tick of measurements. Returns (m, obs_vector).

        1Tick分の計測値を計算し、(計測辞書, 観測ベクトル) を返す。
        """
        v = self.vehicle
        t = v.get_transform()
        loc = t.location
        (fx, fy), (rx, ry) = _yaw_to_forward_right(t.rotation.yaw)
        vel = v.get_velocity()
        speed_ms = vel.x * fx + vel.y * fy          # forward speed / 前方速度
        speed_kmh = 3.6 * math.hypot(vel.x, vel.y)

        # Longitudinal accel + jerk, differentiated over a 4-tick (0.2 s)
        # window. A 1-tick difference at 20 Hz turns the physics-substep
        # speed oscillation into an alternating fake accel (and x20 again
        # into fake jerk of ~±10 m/s^3 at constant speed); the wide window
        # cancels the alternation while keeping ~5 Hz of real bandwidth,
        # plenty for a comfort metric.
        # 縦加速度とジャークは4Tick（0.2秒）窓で微分する。20Hzの1Tick差分は
        # 物理サブステップ由来の速度振動を交互符号の偽加速度に変え（定速でも
        # 偽ジャーク約±10 m/s^3）、広い窓なら振動が相殺される。実帯域は
        # 約5Hz残り、乗り心地指標には十分。
        W = 4
        self.speed_hist.append(speed_ms)
        if len(self.speed_hist) > W + 1:
            self.speed_hist.pop(0)
        if len(self.speed_hist) > W:
            lon_accel = (self.speed_hist[-1] - self.speed_hist[0]) / (W * dt)
        else:
            lon_accel = 0.0
        self.accel_hist.append(lon_accel)
        if len(self.accel_hist) > W + 1:
            self.accel_hist.pop(0)
        if len(self.accel_hist) > W:
            jerk = (self.accel_hist[-1] - self.accel_hist[0]) / (W * dt)
        else:
            jerk = 0.0
        ang = v.get_angular_velocity()
        yaw_rate = math.radians(ang.z)
        lat_accel = speed_ms * yaw_rate             # v * omega approximation

        # Lane geometry — same projection as dashboard.py.
        # 車線幾何 — dashboard.py と同じ射影。
        wp = self.map.get_waypoint(loc, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        wp_off = self.map.get_waypoint(loc, project_to_road=False)
        on_road = wp_off is not None
        if wp is not None:
            lane_width = float(wp.lane_width)
            wl = wp.transform.location
            rv = wp.transform.get_right_vector()
            lane_offset = ((loc.x - wl.x) * rv.x + (loc.y - wl.y) * rv.y)
            hd = math.radians(t.rotation.yaw - wp.transform.rotation.yaw)
            heading_err = math.atan2(math.sin(hd), math.cos(hd))
        else:
            lane_width, lane_offset, heading_err = 0.0, 0.0, 0.0

        # Distance to next junction along the lane (capped 50 m).
        # 車線に沿った次の交差点までの距離（上限50m）。
        dist_junction = 50.0
        if wp is not None:
            probe, walked = wp, 0.0
            for _ in range(10):
                if probe.is_junction:
                    dist_junction = walked
                    break
                nxt = probe.next(5.0)
                if not nxt:
                    break
                probe = nxt[0]
                walked += 5.0

        # Traffic light state ahead. / 前方の信号状態。
        tl_red, tl_yellow = 0.0, 0.0
        tl = v.get_traffic_light() if v.is_at_traffic_light() else None
        if tl is not None:
            st = str(tl.get_state()).split('.')[-1]
            tl_red = 1.0 if st == 'Red' else 0.0
            tl_yellow = 1.0 if st == 'Yellow' else 0.0

        # Red-light violation: left a red light while moving (dashboard.py
        # logic). 信号無視：赤信号のまま動いて通過（dashboard.py と同じ判定）。
        red_light = False
        if tl is not None and tl_red > 0.5:
            self.last_red_tl = tl
        elif self.last_red_tl is not None:
            try:
                exit_state = str(self.last_red_tl.get_state()).split('.')[-1]
            except Exception:
                exit_state = 'None'
            if exit_state == 'Red' and speed_ms > 2.0:
                red_light = True
            self.last_red_tl = None

        # Neighbours in body frame (K nearest). / 車体座標系の近傍K台。
        others = [a for a in self.world.get_actors().filter('vehicle.*')
                  if a.id != v.id]
        neigh = []
        for a in others:
            al = a.get_location()
            dx, dy = al.x - loc.x, al.y - loc.y
            if abs(dx) > 80 or abs(dy) > 80:
                continue
            bx = dx * fx + dy * fy
            by = dx * rx + dy * ry
            av = a.get_velocity()
            rel_speed = (av.x * fx + av.y * fy) - speed_ms
            ah = math.radians(a.get_transform().rotation.yaw
                              - t.rotation.yaw)
            neigh.append((math.hypot(bx, by), bx, by, rel_speed,
                          math.atan2(math.sin(ah), math.cos(ah))))
        neigh.sort(key=lambda n: n[0])
        neigh = neigh[:K_NEIGHBORS]

        front_d, ttc = _front_obstacle(v, others)
        thw = (front_d / speed_ms) if (0.0 < front_d < 60.0
                                       and speed_ms > 0.5) else -1.0

        # Route progress + look-ahead waypoints in body frame.
        # 経路前進量と車体座標系の先読みウェイポイント。
        progress_m, route_dist_left = 0.0, 0.0
        route_pts = []
        if self.route:
            prev_cd = self.cumdist[self.route_idx]
            self._advance_route_index(loc)
            progress_m = max(0.0, self.cumdist[self.route_idx] - prev_cd)
            route_dist_left = self.cumdist[-1] - self.cumdist[self.route_idx]
            stride = max(1, int(WP_STRIDE_M / max(0.1, (
                self.cumdist[-1] / max(1, len(self.route) - 1)))))
            i = self.route_idx
            for _ in range(N_WAYPOINTS):
                i = min(i + stride, len(self.route) - 1)
                wl2 = self.route[i][0].transform.location
                dx, dy = wl2.x - loc.x, wl2.y - loc.y
                route_pts.append((dx * fx + dy * fy, dx * rx + dy * ry))
        while len(route_pts) < N_WAYPOINTS:
            route_pts.append((0.0, 0.0))

        speed_limit = float(v.get_speed_limit() or 0.0)

        m = {
            'speed_kmh': speed_kmh, 'speed_ms': speed_ms,
            'speed_limit_kmh': speed_limit,
            'lon_accel': lon_accel, 'lat_accel': lat_accel, 'jerk': jerk,
            'yaw_rate': yaw_rate,
            'lane_offset': lane_offset, 'lane_width': lane_width,
            'heading_err': heading_err, 'on_road': on_road,
            'dist_junction': dist_junction,
            'tl_red': tl_red, 'tl_yellow': tl_yellow,
            'red_light_event': red_light,
            'front_distance': front_d, 'ttc': ttc, 'thw': thw,
            'progress_m': progress_m, 'route_dist_left': route_dist_left,
        }

        # Observation vector (normalized). / 観測ベクトル（正規化済み）。
        obs = [
            speed_ms / 30.0, lon_accel / 10.0, lat_accel / 10.0,
            yaw_rate / 2.0, lane_offset / 2.0, lane_width / 4.0,
            heading_err / math.pi, dist_junction / 50.0,
            speed_limit / 100.0, tl_red + 0.5 * tl_yellow,
        ]
        for k in range(K_NEIGHBORS):
            if k < len(neigh):
                _, bx, by, rs, ah = neigh[k]
                obs += [bx / 50.0, by / 50.0, rs / 30.0, ah / math.pi]
            else:
                obs += [2.0, 2.0, 0.0, 0.0]   # "far away" filler / 「遠方」埋め
        for px, py in route_pts:
            obs += [px / 50.0, py / 50.0]
        return m, np.asarray(obs, dtype=np.float32)


class CarlaHILEnv(gym.Env):
    """Gym env: ego AV on a random route among (human-like) traffic.

    ランダム経路上のego車を運転するGym環境。周囲交通（人間らしい
    ドライバー）は外部で生成する（generate_traffic.py や将来のシナリオ
    ライブラリ）。この環境自体はego車と経路のみを管理する。
    Surrounding traffic is spawned externally (generate_traffic.py or the
    scenario library); the env only manages the ego vehicle and its route.
    """

    metadata = {'render_modes': []}

    def __init__(self, host='127.0.0.1', port=2000,
                 vehicle_filter='vehicle.tesla.model3',
                 sync=True, seed=None):
        super().__init__()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(OBS_DIM,), dtype=np.float32)
        # [steer, accel]; accel >= 0 -> throttle, < 0 -> brake.
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.client = carla.Client(host, port)
        self.client.set_timeout(20.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.sync = sync
        self._orig_settings = self.world.get_settings()
        if sync:
            s = self.world.get_settings()
            s.synchronous_mode = True
            s.fixed_delta_seconds = FIXED_DT
            self.world.apply_settings(s)
        self.dt = FIXED_DT if sync else 0.05

        self.rng = random.Random(seed)
        self.planner = GlobalRoutePlanner(self.map, sampling_resolution=2.0)
        self.vehicle_filter = vehicle_filter

        self.ego = None
        self.sensors = []
        self._collided = False
        self._lane_events = []      # ['Solid'|'Broken', ...] this step
        self.sensing = None
        self._sim_t0 = 0.0
        self._off_road_s = 0.0
        self._was_on_road = True
        self._hard_brake_cool = 0.0

    # ------------------------------------------------------------------
    def _destroy_actors(self):
        for s in self.sensors:
            try:
                s.stop()
                s.destroy()
            except Exception:
                pass
        self.sensors = []
        if self.ego is not None:
            try:
                self.ego.destroy()
            except Exception:
                pass
        self.ego = None

    def _spawn_ego(self):
        bp_lib = self.world.get_blueprint_library()
        bps = bp_lib.filter(self.vehicle_filter) or bp_lib.filter('vehicle.*')
        bp = self.rng.choice(list(bps))
        bp.set_attribute('role_name', 'ego')
        spawn_points = self.map.get_spawn_points()
        self.rng.shuffle(spawn_points)
        for sp in spawn_points:
            ego = self.world.try_spawn_actor(bp, sp)
            if ego is not None:
                return ego, sp
        raise RuntimeError('no free spawn point for ego / '
                           'ego車のスポーン地点が空いていない')

    def _make_route(self, start_loc):
        """Random route of ROUTE_MIN..MAX metres. ランダム経路を生成する。"""
        spawn_points = self.map.get_spawn_points()
        route = None
        for _ in range(20):
            goal = self.rng.choice(spawn_points).location
            d = start_loc.distance(goal)
            if d < ROUTE_MIN_M * 0.5:
                continue
            route = self.planner.trace_route(start_loc, goal)
            length = sum(
                route[i - 1][0].transform.location.distance(
                    route[i][0].transform.location)
                for i in range(1, len(route)))
            if ROUTE_MIN_M <= length <= ROUTE_MAX_M:
                return route
        if route is None:   # tiny map: accept any goal / 狭いマップでは妥協
            goal = self.rng.choice(spawn_points).location
            route = self.planner.trace_route(start_loc, goal)
        # Fallback: accept whatever we got last. / 最後の経路で妥協する。
        return route

    def _attach_sensors(self):
        bp_lib = self.world.get_blueprint_library()
        me = weakref.ref(self)

        coll = self.world.spawn_actor(
            bp_lib.find('sensor.other.collision'),
            carla.Transform(), attach_to=self.ego)

        def on_coll(_ev, ref=me):
            env = ref()
            if env is not None:
                env._collided = True
        coll.listen(on_coll)
        self.sensors.append(coll)

        lane = self.world.spawn_actor(
            bp_lib.find('sensor.other.lane_invasion'),
            carla.Transform(), attach_to=self.ego)

        def on_lane(ev, ref=me):
            env = ref()
            if env is None:
                return
            for mk in ev.crossed_lane_markings:
                name = str(mk.type)
                env._lane_events.append(
                    'Solid' if 'Solid' in name else 'Broken')
        lane.listen(on_lane)
        self.sensors.append(lane)

    # ------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)
        self._destroy_actors()
        self.ego, sp = self._spawn_ego()
        route = self._make_route(sp.location)
        self._attach_sensors()
        self._collided = False
        self._lane_events = []
        self._off_road_s = 0.0
        self._was_on_road = True
        self._hard_brake_cool = 0.0
        self.sensing = EgoSensing(self.world, self.ego, route)
        self._tick()
        self._sim_t0 = self._sim_time()
        m, obs = self.sensing.measure(self.dt)
        return obs, {'measurements': m}

    def _tick(self):
        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def _sim_time(self):
        return float(self.world.get_snapshot().timestamp.elapsed_seconds)

    def step(self, action):
        steer = float(np.clip(action[0], -1.0, 1.0))
        accel = float(np.clip(action[1], -1.0, 1.0))
        ctrl = carla.VehicleControl(
            throttle=max(0.0, accel), brake=max(0.0, -accel), steer=steer)
        self.ego.apply_control(ctrl)
        self._tick()

        m, obs = self.sensing.measure(self.dt)

        # Discrete events this step. / このステップの離散イベント。
        lane_ev = self._lane_events
        self._lane_events = []
        left_road = self._was_on_road and not m['on_road']
        self._was_on_road = m['on_road']
        if not m['on_road']:
            self._off_road_s += self.dt
        else:
            self._off_road_s = 0.0
        hard_brake = False
        self._hard_brake_cool = max(0.0, self._hard_brake_cool - self.dt)
        if (ctrl.brake > REWARD_RULES['hard_brake_brake']
                and m['lon_accel'] < -REWARD_RULES['hard_brake_decel']
                and self._hard_brake_cool <= 0.0):
            hard_brake = True
            self._hard_brake_cool = 1.0   # refractory / 再発火抑制

        route_done = (self.sensing.route
                      and m['route_dist_left'] < ROUTE_DONE_M)
        ev = {
            'collision': self._collided,
            'red_light': m['red_light_event'],
            'left_road': left_road,
            'crossed_solid': 'Solid' in lane_ev,
            'crossed_broken': ('Broken' in lane_ev
                               and 'Solid' not in lane_ev),
            'hard_brake': hard_brake,
            'route_done': route_done,
        }
        reward, breakdown = step_reward(m, ev)

        elapsed = self._sim_time() - self._sim_t0
        terminated = bool(
            self._collided
            or self._off_road_s > REWARD_RULES['off_road_end_s']
            or route_done)
        truncated = bool(elapsed > EPISODE_TIMEOUT_S) and not terminated

        info = {'measurements': m, 'events': ev,
                'reward_breakdown': breakdown, 'elapsed_s': elapsed}
        return obs, reward, terminated, truncated, info

    def close(self):
        self._destroy_actors()
        try:
            self.world.apply_settings(self._orig_settings)
        except Exception:
            pass
