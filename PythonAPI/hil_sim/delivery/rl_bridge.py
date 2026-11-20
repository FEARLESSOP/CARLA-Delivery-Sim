"""Reinforcement learning bridge for CARLA 0.9.16 (LastMile delivery rig).

Bridges the trained brains from rl_delivery/ into live CARLA delivery_sim:
- Drone navigation (RecurrentPPO, 54-obs): pure neural flight & landing on pod roof
- Pod driving (SAC, 30-obs): pure neural vehicle steering & throttle/brake
- Fleet coordinator (MaskablePPO): fleet dispatch decisions

Observation parity is strictly maintained with the pretend training worlds
(rl_delivery/env_nav.py, env_drive.py, env_fleet.py).
"""
import glob
import json
import math
import os
import random
import sys

import numpy as np

try:
    # The brains are tiny (one obs per call): torch's default thread pool
    # (one per core) costs ~3x more per call than one thread and fights CARLA
    # for the CPU, which slowed the delivery loop to ~4 Hz and made the drones
    # jump. / 小さなネットワークを 1 件ずつ推論するので 1 スレッドが最速。
    # 既定のスレッドプールはループを ~4 Hz に落としドローンがカクついた。
    import torch
    torch.set_num_threads(1)
except Exception:                  # noqa: BLE001  (no torch: bridges unused)
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
HIL_SIM = os.path.dirname(HERE)
RL_DIR = os.path.join(HIL_SIM, "rl_delivery")

# Add rl_delivery to path if needed
if HIL_SIM not in sys.path:
    sys.path.insert(0, HIL_SIM)

from rl_delivery.drone_dyn import Drone, MAX_TILT
from rl_delivery.env_flight import load_map
from rl_delivery.env_nav import Heights, rot, RAYS, RANGE, NOFLY_DIRS, NOFLY_RANGE
from rl_delivery.wind import BuildingGrid, Wind
from delivery.drone import Mission, ASCEND, DONE, HOLD, TAKEOFF, TO_MEET  # noqa: E402  (LastMile missions)

# State constants matching delivery/drone.py
TAKEOFF = "takeoff"
CRUISE = "cruise"
DESCEND = "descend"
DROP = "drop"
ASCEND = "ascend"
RETURN = "return"
LAND = "land"
DONE = "done"
OUTBOUND = (TAKEOFF, CRUISE, DESCEND, DROP)


def resolve_checkpoint(brain, path_or_kw):
    """Find the checkpoint path given a path or keyword ('latest', 'auto')."""
    if not path_or_kw:
        return None
    if path_or_kw not in ("latest", "auto"):
        if os.path.exists(path_or_kw):
            return os.path.abspath(path_or_kw)
        p = os.path.join(HIL_SIM, path_or_kw)
        if os.path.exists(p):
            return os.path.abspath(p)
        raise FileNotFoundError("Checkpoint not found: %s" % path_or_kw)
    ckpt_dir = os.path.join(RL_DIR, "checkpoints", brain)
    zips = sorted(glob.glob(os.path.join(ckpt_dir, "step_*.zip")))
    if not zips:
        raise FileNotFoundError("No checkpoints found in %s" % ckpt_dir)
    return os.path.abspath(zips[-1])


class RLNavFlight(object):
    """Drone flight controlled by a trained RecurrentPPO neural policy.

    Replaces Sortie/Mission splines in delivery/backends.py while maintaining
    the exact same duck-typed interface expected by DroneUnit._fly():
    - attributes: pos, state, done, loaded, holding, drops
    - methods: update(dt), heading()
    """

    def __init__(self, model, heights, map_data, home_fn, hover_point,
                 drop_height=3.0, leg_z=None, rng=None, wind_level=0.0):
        self.is_rl = True
        self.model = model
        self.H = heights
        self.m = map_data
        self.home_fn = home_fn
        self.drops = [hover_point]
        self.drop_height = drop_height
        self.rng = rng or np.random.default_rng()

        ang = np.arange(RAYS) * 2 * math.pi / RAYS
        self.ray_dirs = np.stack([np.cos(ang), np.sin(ang)], 1)
        self.ray_d = np.linspace(2.0, RANGE, 20)
        ang8 = np.arange(NOFLY_DIRS) * 2 * math.pi / NOFLY_DIRS
        self.nf_dirs = np.stack([np.cos(ang8), np.sin(ang8)], 1)
        self.nofly = self.m.get("zones", {}).get("nofly", [])

        # Initial drone state starting at the pod dock/roof
        start_pos = tuple(home_fn())
        self.pos = start_pos
        self.drone = Drone(start_pos, yaw=0.0)

        # Heights at drop and start/meet
        self.gd = float(self.H.ground_at(hover_point[0], hover_point[1]))
        self.gm = float(self.H.ground_at(start_pos[0], start_pos[1]))

        # Target goals
        # Phase 0: drop hover point at ground + 7.0m
        self.drop_goal = np.array([hover_point[0], hover_point[1], self.gd + 7.0], dtype=float)
        # Phase 1: pod roof pad
        self.meet_goal = np.array([start_pos[0], start_pos[1], start_pos[2]], dtype=float)

        self.phase = 0  # 0: outbound to drop, 1: inbound to pod roof
        self._update_goal()

        self.wind = Wind(self.rng, level=wind_level)
        self.state = TAKEOFF
        self.lstm_state = None
        self.episode_start = np.ones((1,), bool)
        self.last_act = np.zeros(4, dtype=np.float32)
        self._timer = 0.0
        self.n = 0
        self.holding = False
        self._crashed = None

    @property
    def done(self):
        return self.state == DONE

    @property
    def loaded(self):
        return self.phase == 0

    @property
    def roll_deg(self):
        return math.degrees(self.drone.roll)

    @property
    def pitch_deg(self):
        return math.degrees(self.drone.pitch)

    @property
    def yaw_deg(self):
        return math.degrees(self.drone.yaw)

    def heading(self):
        return self.yaw_deg

    def _in_nofly(self, p, grow=0.0):
        for r in self.nofly:
            if (r["x0"] - grow <= p[0] <= r["x1"] + grow and
                    r["y0"] - grow <= p[1] <= r["y1"] + grow):
                return True
        return False

    def _ranges(self, p, dz=0.0):
        X = p[0] + self.ray_dirs[:, 0:1] * self.ray_d[None, :]
        Y = p[1] + self.ray_dirs[:, 1:2] * self.ray_d[None, :]
        hit = self.H.top_at(X, Y) > p[2] + dz
        first = np.where(hit.any(1), hit.argmax(1), len(self.ray_d))
        return np.where(first < len(self.ray_d),
                        self.ray_d[np.minimum(first, len(self.ray_d) - 1)], RANGE)

    def _nofly_dist(self, p):
        out = np.full(NOFLY_DIRS, NOFLY_RANGE)
        steps = np.linspace(5.0, NOFLY_RANGE, 30)
        for k, d in enumerate(self.nf_dirs):
            for s in steps:
                if self._in_nofly((p[0] + d[0] * s, p[1] + d[1] * s)):
                    out[k] = s
                    break
        return out

    def _update_goal(self):
        dr_p = self.drone.p
        if self.phase == 0:
            self.goal = self.drop_goal
        else:
            # Dynamically update meet goal in case pod moved or settled
            h = self.home_fn()
            self.meet_goal = np.array([h[0], h[1], h[2]], dtype=float)
            target = self.meet_goal
            hd = float(np.linalg.norm(dr_p[:2] - target[:2]))
            z_cruise = max(self.gd, self.gm) + 40.0
            z_final = self.meet_goal[2]
            R_CONE = 25.0
            if hd >= R_CONE:
                eff_z = z_cruise
            else:
                eff_z = z_final + (z_cruise - z_final) * (hd / R_CONE)
            self.goal = np.array([target[0], target[1], eff_z], dtype=float)

    def set_recovery_target(self, target_fn):
        """Update inbound recovery target function (pod roof or station pad)."""
        self.home_fn = target_fn
        h = tuple(target_fn())
        self.gm = float(self.H.ground_at(h[0], h[1]))
        self.meet_goal = np.array([h[0], h[1], h[2]], dtype=float)
        if self.phase == 1:
            self._update_goal()

    def _obs(self):
        p, v, acc, roll, pitch, yaw = self.drone.sensors(self.rng)
        gp = self.goal - p
        g_here = float(self.H.ground_at(p[0], p[1]))
        t_here = float(self.H.top_at(p[0], p[1]))
        rng = self._ranges(self.drone.p)
        rng_low = self._ranges(self.drone.p, -5.0)[::2]
        nfd = self._nofly_dist(self.drone.p)
        dist = float(np.linalg.norm(gp[:2]))
        o = np.concatenate([
            rot(yaw, gp) / 100.0,
            rot(yaw, v) / 15.0,
            rot(yaw, acc) / 10.0,
            [math.sin(roll) / math.sin(MAX_TILT), math.sin(pitch) / math.sin(MAX_TILT)],
            [min(dist, 1000.0) / 1000.0, math.atan2(*rot(yaw, gp)[1::-1]) / math.pi],
            [(p[2] - g_here) / 50.0, (p[2] - t_here) / 20.0, (self.goal[2] - p[2]) / 30.0],
            np.roll(rng, -int(round(yaw / (2 * math.pi / RAYS)))) / RANGE,
            np.roll(rng_low, -int(round(yaw / (2 * math.pi / 8)))) / RANGE,
            np.roll(nfd, -int(round(yaw / (2 * math.pi / NOFLY_DIRS)))) / NOFLY_RANGE,
            self.last_act,
            [self.drone.battery / 100.0],
            [float(self.phase)]
        ])
        return np.clip(o, -10, 10).astype(np.float32)

    TRAIN_DT = 0.05     # the step the nav brain was trained with (env_nav.DT) / 学習時の刻み

    def update(self, dt):
        """Advance the RL flight by ``dt`` s in steps of at most TRAIN_DT, so a
        slow or uneven client loop never feeds the policy and the dynamics a
        bigger step than it was trained on. / 学習時の刻み (0.05 s) 以下に分割
        して進める（ループが遅くても大きな刻みを与えない）。"""
        events = []
        left = max(float(dt), 0.0)
        while left > 1e-6 and self.state != DONE:
            h = min(left, self.TRAIN_DT)
            events += self._update_once(h)
            left -= h
        return events

    def _update_once(self, dt):
        """One step of at most TRAIN_DT s. / 1 ステップ分進める。"""
        events = []
        if self.state == DONE:
            return events

        # Handle parcel winch hover dwell at drop
        if self.state == DROP:
            self._timer -= dt
            # Hover in place while lowering
            w = np.zeros(3)
            self.drone.step(np.zeros(4), w, dt)
            self.pos = tuple(self.drone.p)
            if self._timer <= 0.0:
                events.append(("delivered", 0))
                self.phase = 1
                self.state = ASCEND
                self._update_goal()
            return events

        # Step wind model
        agl = max(self.drone.p[2] - float(self.H.ground_at(self.drone.p[0], self.drone.p[1])), 0.5)
        self.wind.step(dt, agl)
        w = self.wind.at(self.drone.p[0], self.drone.p[1], agl)

        # Compute observation & predict action
        self._update_goal()
        obs = self._obs()
        act, self.lstm_state = self.model.predict(
            obs, state=self.lstm_state, episode_start=self.episode_start, deterministic=True
        )
        self.episode_start = np.zeros((1,), bool)
        self.last_act = act.copy()

        # Step drone physics
        self.drone.step(act, w, dt)
        self.pos = tuple(self.drone.p)
        self.n += 1

        # Check hazards / crash conditions
        top = float(self.H.top_at(self.drone.p[0], self.drone.p[1]))
        if self.drone.p[2] < top + 0.2 and self.n > 10:
            events.append(("crashed", "obstacle" if top > float(self.H.ground_at(self.drone.p[0], self.drone.p[1])) + 0.5 else "ground"))
            self.state = DONE
            return events
        if self._in_nofly(self.drone.p):
            events.append(("crashed", "nofly"))
            self.state = DONE
            return events
        if self.drone.battery <= 0:
            events.append(("crashed", "battery_depleted"))
            self.state = DONE
            return events

        # Goal progress & transitions
        hd_drop = float(np.linalg.norm(self.drop_goal[:2] - self.drone.p[:2]))
        hd_meet = float(np.linalg.norm(self.meet_goal[:2] - self.drone.p[:2]))
        tilt_deg = max(abs(self.drone.roll), abs(self.drone.pitch)) * 180.0 / math.pi

        if self.phase == 0:
            # Flying outbound to drop
            if hd_drop > 25.0:
                self.state = CRUISE
            elif hd_drop > 4.0:
                self.state = DESCEND
            elif abs(self.drop_goal[2] - self.drone.p[2]) < 3.0 and float(np.linalg.norm(self.drone.v)) < 1.8:
                # Arrived within drop threshold -> initiate hover dwell & winch lower
                self.state = DROP
                self._timer = 3.0
        else:
            # Flying inbound back to pod roof
            if hd_meet > 25.0:
                self.state = RETURN
            elif hd_meet > 4.0:
                self.state = LAND
            else:
                self.state = LAND
                # Touchdown check matching env_nav:
                # within 1.3 m of centre, vertical delta < 0.6 m, sinking < 1.5 m/s, drift < 1.2 m/s, tilt <= 12 deg
                landed = (hd_meet < 1.3 and self.drone.p[2] - self.meet_goal[2] < 0.6
                          and self.drone.v[2] > -1.5 and float(np.linalg.norm(self.drone.v[:2])) < 1.2
                          and tilt_deg <= 12.0)
                if landed:
                    events.append(("landed", None))
                    self.state = DONE

        return events


class _MeetLeg(RLNavFlight):
    """RLNavFlight whose inbound leg aims at the mission's meeting point (read every
    step, so a moved meeting point is followed) instead of a landing pad.
    帰路の目標を合流地点（毎ステップ読み直す）にした RL 飛行。"""

    def __init__(self, nav, start, drop, mission, rng):
        self.mission = mission                      # read by _update_goal during __init__
        super().__init__(nav.model, nav.heights, nav.map_data, lambda: tuple(start), drop,
                         rng=rng, wind_level=nav.wind_level)

    def _update_goal(self):
        if self.phase == 0:
            self.goal = self.drop_goal
        else:
            m = self.mission
            self.meet_goal = np.array([m.meet[0], m.meet[1], m.meet_z], dtype=float)
            self.goal = self.meet_goal.copy()


class RLMission(Mission):
    """A LastMile pod's mission (drone.Mission) with the nav brain flying the legs it
    was trained for: take-off, out to the drop, the drop, and on to the pod's meeting
    point (Mission.meet, so a moved meeting point is followed). Holding there and
    landing on the pod after it clears the drone stay scripted (Mission's own code):
    the brain was never trained to land on a moving pod. ``is_rl`` is True until the
    drone holds at the meeting point; during the RL legs the brain's drone model drains
    the battery, afterwards CARLA's battery model goes on from the same level.

    LastMile ポッドの配達を RL 脳で飛ぶ：離陸・投下地点まで・投下・合流地点まで。
    合流地点での待機と、許可後のポッドへの着陸はスクリプトのまま（移動するポッドへの着陸は未学習）。
    """
    MEET_R, MEET_CLEAR = 8.0, 3.0                    # at the meeting point: within 8 m, 3 m above everything
    BRAIN_DRAIN = 0.25                               # %/s at hover in the brain's drone model (drone_dyn)
    CLEAR_R, CLEAR_M = 15.0, 6.0                     # vertical take-off: above everything within 15 m, + 6 m

    def __init__(self, nav, start, drop, cruise_z, meet, meet_z, battery=100.0, rng=None, drain=None, **kw):
        Mission.__init__(self, start, drop, cruise_z, meet, meet_z, **kw)
        self.H = nav.heights
        # the sim's own battery use (BatteryModel.hover, %/s) over the brain's model, so a
        # "typical" 25 min battery is not drained at the brain's 7 min rate
        # 実機側の電池消費に合わせる（学習時の 7 分電池の速さで減らさない）
        self.drain_scale = (float(drain) / self.BRAIN_DRAIN) if drain else 1.0
        self.leg = _MeetLeg(nav, start, drop, self, rng or np.random.default_rng())
        self.leg.drone.battery = float(battery)
        self.rl = True
        # a pod often parks beside trees or walls; the brain learned to start in the open
        # (3 m clear), so the take-off is a scripted vertical climb clear of everything
        # around, then the brain takes over / ポッドは木や壁の横に止まることが多いので、
        # 周囲より高くまで垂直に上昇してから RL 脳に渡す
        xs = start[0] + np.linspace(-self.CLEAR_R, self.CLEAR_R, 13)
        ys = start[1] + np.linspace(-self.CLEAR_R, self.CLEAR_R, 13)
        X, Y = np.meshgrid(xs, ys)
        top = float(np.max(nav.heights.top_at(X.ravel(), Y.ravel())))
        self.clear_z = max(start[2] + 5.0, top + self.CLEAR_M)
        self.climbing = True

    @property
    def is_rl(self):
        return self.rl

    @property
    def drone(self):
        return self.leg.drone

    @property
    def yaw_deg(self):
        return self.leg.yaw_deg

    def heading(self):
        return self.leg.yaw_deg if self.rl else Mission.heading(self)

    def update(self, dt):
        if not self.rl:
            return Mission.update(self, dt)
        if self.climbing:                            # scripted vertical take-off / 垂直離陸
            self.state = TAKEOFF
            done = self._step((self.pos[0], self.pos[1], self.clear_z), self.v_speed, dt)
            d = self.leg.drone
            d.p = np.array(self.pos, float)
            d.v = np.zeros(3)
            self.leg.pos = tuple(self.pos)
            if done:
                self.climbing = False                # the brain flies from here / ここから RL 脳
                self.leg.n = 11                      # past the start-up grace steps / 起動猶予を過ぎた扱い
            return []
        out = []
        b0 = self.leg.drone.battery
        evs = self.leg.update(dt)
        self.leg.drone.battery = b0 - (b0 - self.leg.drone.battery) * self.drain_scale
        for ev, data in evs:
            if ev == "delivered":
                out.append((ev, data))
            elif ev == "crashed":
                out.append((ev, data))
                self.state = DONE
                return out
        self.pos = tuple(self.leg.pos)
        self.distance = getattr(self, "distance", 0.0)
        leg = self.leg
        if leg.phase == 0:
            self.state = leg.state                   # takeoff / cruise / descend / drop
            return out
        p = leg.drone.p
        hd = math.hypot(p[0] - self.meet[0], p[1] - self.meet[1])
        # the brain approaches a goal as it learned to approach a landing pad (lower than
        # meet_z), so "arrived" = close and safely clear of everything below
        # 学習した着陸接近のため meet_z より低く止まるので、近くて安全な高さなら到着とする
        clear = p[2] > float(self.H.top_at(p[0], p[1])) + self.MEET_CLEAR
        if (hd < self.MEET_R and clear) or leg.state == DONE:
            # at the meeting point: Mission holds there and lands when cleared
            # 合流地点に到着：以降は Mission が待機し、許可後に着陸
            self.rl = False
            self.state = HOLD
            out.append(("at_meet", None))
        else:
            self.state = ASCEND if p[2] < self.meet_z - 8.0 and hd > 25.0 and leg.n < 400 else TO_MEET
        return out


class NavBridge(object):
    """Bridge manager for neural drone navigation."""

    def __init__(self, checkpoint_path, map_name="OitaKosen", wind_level=3.0):
        from sb3_contrib import RecurrentPPO
        resolved = resolve_checkpoint("nav", checkpoint_path)
        print("[NavBridge] Loading checkpoint:", resolved)
        self.model = RecurrentPPO.load(resolved)
        self.map_name = map_name
        self.heights = Heights(map_name)
        self.map_data = load_map(map_name)
        self.wind_level = wind_level
        self.rng = np.random.default_rng()

    def create_mission(self, start, hover_point, cruise_z, meet, meet_z, battery=100.0, drain=None, **kw):
        """An RLMission for a LastMile pod's drone (RL legs, scripted hold and landing).
        LastMile ポッドのドローン用 RLMission。"""
        return RLMission(self, start, hover_point, cruise_z, meet, meet_z, battery=battery, rng=self.rng,
                         drain=drain, **kw)

    def create_flight(self, van, hover_point, drop_height=3.0):
        """Create an RLNavFlight instance for a sortie."""
        return RLNavFlight(
            model=self.model,
            heights=self.heights,
            map_data=self.map_data,
            home_fn=van.dock_pos,
            hover_point=hover_point,
            drop_height=drop_height,
            rng=self.rng,
            wind_level=self.wind_level
        )


class DrivePolicy(object):
    """The trained pod-driving brain and the lane data it was trained on
    (rl_delivery/maps/<Map>_drive.json), loaded once and shared by every pod's
    DriveBridge. / 学習済み運転ブレインと学習時の車線データ（全ポッドで共有）。"""

    def __init__(self, checkpoint_path, map_name="OitaKosen"):
        from stable_baselines3 import SAC
        resolved = resolve_checkpoint("drive", checkpoint_path)
        print("[DriveBridge] Loading checkpoint:", resolved)
        self.model = SAC.load(resolved, device="cpu")
        self.map_name = map_name
        with open(os.path.join(RL_DIR, "maps", map_name + "_drive.json")) as f:
            D = json.load(f)
        self.lanes = D["lanes"]
        self.stops = {}
        for s in D["stops"]:
            self.stops.setdefault(s["edge"], []).append(("stop", s["k"], None))
        for s in D["signals"]:
            self.stops.setdefault(s["edge"], []).append(("signal", s["k"], (s["junction"], s["phase"])))
        with open(os.path.join(RL_DIR, "maps", map_name + ".json")) as f:
            M = json.load(f)
        # lane piece i = M["edges"][i] (prepare_drive): successors for route matching / 後続の車線片
        by_u = {}
        for i, e in enumerate(M["edges"]):
            by_u.setdefault(e[0], []).append(i)
        self.succ = [by_u.get(e[1], []) for e in M["edges"]]
        self.sib = [by_u.get(e[0], []) for e in M["edges"]]          # same start (forks, duplicates) / 同じ始点
        self.by_rl = {}
        self.grid = {}
        for i, L in enumerate(self.lanes):
            self.by_rl.setdefault((L["road"], L["lane"]), []).append(i)
            for p in L["pts"]:
                self.grid.setdefault((int(p[0] // 5), int(p[1] // 5)), []).append(i)
        self.actuator = dict(DRIVE_ACTUATOR)
        try:                              # measured in CARLA (see DRIVE_ACTUATOR) / CARLA で計測した値
            with open(os.path.join(RL_DIR, "maps", "drive_actuator.json")) as f:
                self.actuator.update(json.load(f))
        except (OSError, ValueError):
            pass
        self._veh, self._veh_t, self._tls = [], -1e9, None

    def lane_dist2(self, i, x, y):
        P = np.asarray(self.lanes[i]["pts"])[:, :2]
        return float(((P[:, 0] - x) ** 2 + (P[:, 1] - y) ** 2).min())

    def lane_at(self, x, y, road=None, lane=None, cand=None):
        """The lane piece a point is on: among ``cand`` (or the same road / lane
        id, or the lanes nearby) the nearest, within 3 m. / 点が乗る車線片"""
        if cand is None:
            cand = self.by_rl.get((road, lane)) if road is not None else None
        if not cand:
            cand = set()
            cx, cy = int(x // 5), int(y // 5)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    cand.update(self.grid.get((cx + dx, cy + dy), ()))
        best, bd = None, 9.0
        for i in cand:
            d = self.lane_dist2(i, x, y)
            if d < bd - 1e-6:
                best, bd = i, d
        return best

    def vehicles(self, world, t):
        """All vehicles, re-listed once a second (a fresh list costs a server
        wait). / 車両の一覧（1 秒ごとに更新）"""
        if world is not None and t - self._veh_t >= 1.0:
            self._veh_t = t
            self._veh = list(world.get_actors().filter("vehicle.*"))
        return self._veh

    def lights(self, world):
        """(traffic light, [stop line points]) once per run. / 信号と停止線位置"""
        if self._tls is None and world is not None:
            self._tls = []
            for tl in world.get_actors().filter("traffic.traffic_light"):
                try:
                    pts = [(w.transform.location.x, w.transform.location.y) for w in tl.get_stop_waypoints()]
                except RuntimeError:
                    pts = []
                self._tls.append((tl, pts))
        return self._tls or []


# The brain asks for an acceleration (env_drive: +-3 m/s2); CARLA wants pedal
# positions. A pedal that gives full acceleration / full braking on the pod,
# so the request maps linearly (a motor model, not a driving rule). Defaults
# until measured on the LMP-4 in CARLA; a measurement goes in
# rl_delivery/maps/drive_actuator.json. / 要求加速度をペダル量へ（車両モデル）。
DRIVE_ACTUATOR = {"accel_full": 3.0, "brake_full": 8.0}
DRIVE_DT = 0.1                      # env_drive.DT: the brain decides 10 times a second / 10 Hz で判断
DRIVE_MAX_STEER, DRIVE_STEER_RATE = 0.6, 1.2
DRIVE_MAX_ACC, DRIVE_MAX_BRAKE = 3.0, 3.0
DRIVE_POD_MAX = 30.0 / 3.6
DRIVE_LENGTH = 3.6
DRIVE_AHEAD = (5.0, 10.0, 20.0, 30.0, 40.0)


class DriveBridge(object):
    """One pod's driving brain in CARLA. Builds exactly the observation of
    rl_delivery/env_drive.py from the same lane data: route points, edge kinds
    (kerb / same-way lane / oncoming), real speed limits, junction flags,
    stop lines and signals (colour from CARLA's real lights), the car ahead in
    the pod's lane; decides at the trained 10 Hz and holds the control
    between decisions. One instance per pod (2026-11-13: one shared instance
    made every pod drive by the last pod's route).
    1 台のポッドの運転ブレイン。env_drive と同じ車線データから同じ観測を作り、
    学習時の 10 Hz で判断する。ポッドごとに 1 つ（共有すると他車の経路で走っていた）。"""

    def __init__(self, checkpoint_path="latest", map_name="OitaKosen", policy=None):
        self.policy = policy or DrivePolicy(checkpoint_path, map_name)
        self.model = self.policy.model
        self.map_name = map_name
        self.P = None
        self.S = None
        self.marks = []
        self.k = 0
        self.s = 0.0
        self.last_act = np.zeros(2, dtype=np.float32)
        self.steer = 0.0
        self.done = False
        self.edges = []
        self._t_dec = None
        self._t_sim = 0.0
        self._ctrl = None
        self._max_wheel = None
        self._bound = False

    # -- route / 経路
    def build_from_waypoints(self, waypoints):
        """Route from CARLA waypoints (the planner's plan): each mapped to its
        lane piece of the training data, then built like env_drive._build.
        経路の各点を学習データの車線片に対応させ、env_drive と同じ方法で作る。"""
        if not waypoints:
            return
        pol = self.policy
        edges, locs = [], []
        for item in waypoints:
            wp = item[0] if isinstance(item, (tuple, list)) else item
            loc = wp.transform.location if hasattr(wp, "transform") else wp
            locs.append(loc)
            road, lane = getattr(wp, "road_id", None), getattr(wp, "lane_id", None)
            li = None
            if edges:
                # follow the lane graph: stay on this piece while it is the closest,
                # else move to a successor (one or two pieces on) / 車線網に沿って進む
                cur = edges[-1]
                nxt = list(pol.sib[cur]) + list(pol.succ[cur]) + [j for n in pol.succ[cur] for j in pol.succ[n]]
                if road is not None:
                    nxt = [j for j in nxt if (pol.lanes[j]["road"], pol.lanes[j]["lane"]) == (road, lane)] or nxt
                if pol.lane_dist2(cur, loc.x, loc.y) < 0.01:
                    li = cur                              # still on it (joints belong to both) / 継ぎ目は現在の片
                else:
                    li = pol.lane_at(loc.x, loc.y, cand=nxt) if nxt else None
                    if li is not None and li in pol.sib[cur]:
                        edges[-1] = li                    # the other branch of a fork / 分岐のもう一方
                        continue
                    if li is not None and li not in pol.succ[cur]:
                        mid = [n for n in pol.succ[cur] if li in pol.succ[n]]
                        if mid:
                            edges.append(mid[0])          # a short piece the points skipped / 飛ばした短い片
            if li is None:
                li = pol.lane_at(loc.x, loc.y, road, lane)
            if li is not None and (not edges or edges[-1] != li):
                edges.append(li)
        self.k, self.s, self.done = 0, 0.0, False
        self.steer, self.last_act = 0.0, np.zeros(2, dtype=np.float32)
        self._t_dec, self._ctrl, self._bound = None, None, False
        if edges:
            self.build_from_edges(edges)
        else:                                       # off the lane data: plain line / 車線データ外
            self._build_plain(locs)

    def build_from_edges(self, edges):
        """env_drive._build, line for line. / env_drive._build と同じ"""
        lanes, stops = self.policy.lanes, self.policy.stops
        pts, s_acc, marks = [], [], []
        s = 0.0
        for ei in edges:
            L = lanes[ei]
            base = len(pts)
            for q in (L["pts"] if not pts else L["pts"][1:]):
                if pts:
                    s += math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1])
                pts.append(q[:7] + [min(L["kmh"], 60.0), L["junction"]])
                s_acc.append(s)
            for kind, k, arg in stops.get(ei, ()):
                idx = base + max(0, k - (1 if base else 0))
                if idx < len(pts):
                    marks.append([s_acc[idx], kind, arg, False, False, None])  # s, kind, signal, stopped, passed, light
        self._resample(pts, s_acc)
        self.marks = sorted(marks, key=lambda m: m[0])
        self.edges = list(edges)

    def _build_plain(self, locs):
        pts, s_acc, s = [], [], 0.0
        for loc in locs:
            if pts:
                s += math.hypot(loc.x - pts[-1][0], loc.y - pts[-1][1])
            pts.append([loc.x, loc.y, 0.0, 0.0, 3.5, 0, 0, 30.0, 0])
            s_acc.append(s)
        if len(pts) >= 2 and s_acc[-1] >= 1.0:
            self._resample(pts, s_acc)
        self.marks, self.edges = [], []

    def _resample(self, pts, s_acc):
        raw = np.array([[q[0], q[1], q[3], q[4], q[5], q[6], q[7], q[8]] for q in pts], float)
        S0 = np.array(s_acc)
        keep = np.concatenate([[True], np.diff(S0) > 0.05])
        raw, S0 = raw[keep], S0[keep]
        S = np.arange(0.0, S0[-1], 1.0)
        idx = np.clip(np.searchsorted(S0, S, side="right") - 1, 0, len(S0) - 1)
        x = np.interp(S, S0, raw[:, 0])
        y = np.interp(S, S0, raw[:, 1])
        hd = np.arctan2(np.gradient(y), np.gradient(x))
        self.P = np.column_stack([x, y, hd, raw[idx, 3], raw[idx, 4], raw[idx, 5], raw[idx, 6], raw[idx, 7]])
        self.S = S

    # -- geometry (env_drive) / 幾何
    def _project(self, x, y):
        if self.P is None or len(self.P) == 0:
            return 0.0, 0.0, np.zeros(8)
        lo, hi = max(0, self.k - 5), min(len(self.P), self.k + 30)
        P = self.P[lo:hi]
        d2 = (P[:, 0] - x) ** 2 + (P[:, 1] - y) ** 2
        j = int(np.argmin(d2))
        self.k = lo + j
        q = self.P[self.k]
        h = q[2]
        nx_, ny_ = -math.sin(h), math.cos(h)
        off = (x - q[0]) * nx_ + (y - q[1]) * ny_
        along = (x - q[0]) * math.cos(h) + (y - q[1]) * math.sin(h)
        return self.S[self.k] + along, off, q

    def _at(self, s):
        k = int(np.clip(np.searchsorted(self.S, s), 0, len(self.S) - 1))
        return self.P[k]

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _next_mark(self):
        for m in self.marks:
            if not m[4] and m[0] > self.s - 1.0:
                return m
        return None

    # -- live inputs / 実車の入力
    def _bind(self, car, world):
        """Once per route: the pod's real wheel angle, and the CARLA light of
        each signal stop line. / 実車の最大舵角と各信号停止線の信号機"""
        self._bound = True
        if self._max_wheel is None:
            try:
                wheels = car.get_physics_control().wheels
                self._max_wheel = math.radians(max(w.max_steer_angle for w in wheels[:2]))
            except Exception:                        # mock / no physics: env's own angle / モック
                self._max_wheel = DRIVE_MAX_STEER
        for m in self.marks:
            if m[1] != "signal":
                continue
            p = self._at(m[0])
            best, bd = None, 6.0
            for tl, pts in self.policy.lights(world):
                for (x, y) in pts:
                    d = math.hypot(x - p[0], y - p[1])
                    if d < bd:
                        best, bd = tl, d
            m[5] = best

    @staticmethod
    def _light(m):
        """0 green / 1 yellow / 2 red from the bound CARLA light (green if none).
        / 信号の色（対応する信号機が無ければ青）"""
        tl = m[5]
        if tl is None:
            return 0
        try:
            import carla
            st = tl.get_state()
            return 2 if st == carla.TrafficLightState.Red else 1 if st == carla.TrafficLightState.Yellow else 0
        except Exception:
            return 0

    def _lead(self, car, world, x, y):
        """Gap to and speed of the nearest car ahead on the pod's own route
        (within 1.8 m of the lane centre), like env_drive's route cars.
        自分の経路上（車線中心から 1.8 m 以内）の最も近い前方車両。"""
        lead, lv = 99.0, None
        hi = min(len(self.P), self.k + 70)
        P = self.P[self.k:hi]
        if len(P) < 2:
            return lead, lv
        for a in self.policy.vehicles(world, self._t_sim):
            if a.id == car.id:
                continue
            loc = a.get_location()
            if math.hypot(loc.x - x, loc.y - y) > 75.0:
                continue
            d2 = (P[:, 0] - loc.x) ** 2 + (P[:, 1] - loc.y) ** 2
            j = int(np.argmin(d2))
            h = P[j, 2]
            lat = (loc.x - P[j, 0]) * -math.sin(h) + (loc.y - P[j, 1]) * math.cos(h)
            gap = self.S[self.k + j] - self.s - DRIVE_LENGTH
            if abs(lat) < 1.8 and -DRIVE_LENGTH < gap < lead:
                v = a.get_velocity()
                lead, lv = gap, v.x * math.cos(h) + v.y * math.sin(h)
        return lead, lv

    # -- one step / 1 ステップ
    def observe(self, car, world=None):
        """The 30-number observation of env_drive._obs for the pod now, after
        updating the stop-line flags like env_drive.step. / env_drive と同じ観測"""
        tf = car.get_transform()
        vel = car.get_velocity()
        v = math.hypot(vel.x, vel.y)
        yaw = math.radians(tf.rotation.yaw)
        s_old = self.s
        self.s, off, q = self._project(tf.location.x, tf.location.y)
        for m in self.marks:                         # stop lines (env_drive.step) / 停止線
            if m[4]:
                continue
            if 0.0 <= m[0] - self.s <= 4.0 and v < 0.3:
                m[3] = True
            if s_old < m[0] <= self.s:
                m[4] = True
        herr = self._wrap(yaw - q[2])
        curv = [self._wrap(self._at(self.s + d)[2] - q[2]) for d in DRIVE_AHEAD]
        m = self._next_mark()
        if m is not None:
            mdist = m[0] - self.s
            mk = [1.0, 0.0, 0, 0, 0] if m[1] == "stop" else [0.0, 1.0] + [float(self._light(m) == c) for c in range(3)]
            mobs = [min(mdist, 60.0) / 60.0] + mk + [float(m[3])]
        else:
            mobs = [1.0, 0, 0, 0, 0, 0, 0]
        lead, lv = self._lead(car, world, tf.location.x, tf.location.y)
        if lv is None:
            lv = v
        ka, kb = int(q[4]), int(q[5])
        o = [v / DRIVE_POD_MAX, self.steer / DRIVE_MAX_STEER, off / 2.0, herr, q[3] / 4.0,
             float(ka == 0), float(ka == 1), float(ka == 2), float(kb == 0), float(kb == 1), float(kb == 2),
             float(q[7])] + curv + mobs + [min(lead, 60.0) / 60.0, (v - lv) / 10.0, q[6] / 60.0,
                                           min(self.S[-1] - self.s, 800.0) / 800.0] + list(self.last_act)
        return np.clip(np.array(o[:30], np.float32), -10, 10), v

    def run_step(self, car_actor, world=None):
        """A carla.VehicleControl: a new decision every DRIVE_DT of sim time,
        the last one held in between. / 10 Hz で判断し、間は直前の操作を保持"""
        import carla
        if self.P is None or len(self.P) == 0:
            return carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
        if world is not None:
            self._t_sim = world.get_snapshot().timestamp.elapsed_seconds
        else:
            self._t_sim += DRIVE_DT                  # no world (tests): every call decides / テスト用
        if self._ctrl is not None and self._t_dec is not None and self._t_sim - self._t_dec < DRIVE_DT - 1e-3:
            return self._ctrl
        self._t_dec = self._t_sim
        if not self._bound:
            self._bind(car_actor, world)
        obs, v = self.observe(car_actor, world)
        if self.s >= self.S[-1] - 3.0:
            self.done = True
            self._ctrl = carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
            return self._ctrl
        act = self.model.predict(obs, deterministic=True)[0]
        a = np.clip(np.asarray(act, float), -1, 1)
        self.last_act = a.astype(np.float32)
        target = a[0] * DRIVE_MAX_STEER                   # env_drive.step / 同じ舵の動き
        self.steer += float(np.clip(target - self.steer, -DRIVE_STEER_RATE * DRIVE_DT, DRIVE_STEER_RATE * DRIVE_DT))
        acc = a[1] * (DRIVE_MAX_ACC if a[1] > 0 else DRIVE_MAX_BRAKE)
        if acc > 0 and v >= DRIVE_POD_MAX:
            acc = 0.0                                     # env caps speed at the pod's top speed / 最高速
        act_cfg = self.policy.actuator
        throttle = float(np.clip(acc / act_cfg["accel_full"], 0.0, 1.0)) if acc > 0 else 0.0
        brake = float(np.clip(-acc / act_cfg["brake_full"], 0.0, 1.0)) if acc < 0 else 0.0
        steer = float(np.clip(self.steer / (self._max_wheel or DRIVE_MAX_STEER), -1.0, 1.0))
        self._ctrl = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
        return self._ctrl


class FleetRecoveryBridge(object):
    """Bridge manager for FA-RAMPPO dynamic aerial recovery and pad reservations."""

    def __init__(
        self,
        checkpoint_path="",
        baseline="b4_fa_ramppo",
        map_name="OitaKosen",
        horizon_events=4,
        num_scenarios=8,
        beta=0.5,
    ):
        from delivery.feasibility_engine import FeasibilityEngine
        from delivery.reservations import PadReservationManager
        from rl_delivery.ramppo import FARAMPPOPlanner, BaselineType

        self.map_name = map_name
        self.pad_manager = PadReservationManager()
        self.feasibility = FeasibilityEngine(pad_manager=self.pad_manager)

        baseline_map = {
            "b0": BaselineType.B0_HEURISTIC,
            "heuristic": BaselineType.B0_HEURISTIC,
            "b1": BaselineType.B1_STANDARD_PPO,
            "standard": BaselineType.B1_STANDARD_PPO,
            "b2": BaselineType.B2_MASKABLE_PPO,
            "maskable": BaselineType.B2_MASKABLE_PPO,
            "b3": BaselineType.B3_FORECAST_PPO,
            "forecast": BaselineType.B3_FORECAST_PPO,
            "b4": BaselineType.B4_FA_RAMPPO,
            "ramppo": BaselineType.B4_FA_RAMPPO,
            "b4_fa_ramppo": BaselineType.B4_FA_RAMPPO,
            "b5": BaselineType.B5_ROLLOUT_HEURISTIC,
            "rollout": BaselineType.B5_ROLLOUT_HEURISTIC,
        }
        self.baseline = baseline_map.get(str(baseline).lower(), BaselineType.B4_FA_RAMPPO)

        self.planner = FARAMPPOPlanner(
            feasibility_engine=self.feasibility,
            horizon_events=horizon_events,
            num_scenarios=num_scenarios,
            beta_policy_weight=beta,
        )

        self.drone_policy = None
        if checkpoint_path:
            try:
                from sb3_contrib import MaskablePPO
                resolved = resolve_checkpoint("fleet_drone", checkpoint_path)
                print("[FleetRecoveryBridge] Loading fleet drone policy:", resolved)
                self.drone_policy = MaskablePPO.load(resolved)
            except Exception as e:
                print("[FleetRecoveryBridge] Warning: could not load fleet drone policy:", e)

        self.decision_history = []

    def evaluate_recovery(
        self,
        drone_unit,
        van,
        station_pads,
        fleet_vans=None,
        current_wind=(0.0, 0.0),
        current_t=0.0,
    ):
        """
        Evaluate candidate recovery targets (pod vs fixed station pads) and return chosen target.
        """
        from delivery.feasibility_engine import DroneRecoveryQuery, RecoveryTarget
        from delivery.compact_simulator import CompactFleetState, SimDrone, SimHub, SimStation, DroneState

        d_pos = drone_unit.pos if hasattr(drone_unit, "pos") else (0.0, 0.0, 10.0)
        batt_wh = float(getattr(drone_unit.battery, "level", 100.0) if hasattr(drone_unit, "battery") else 100.0)
        d_id = f"drone_{drone_unit.id}" if hasattr(drone_unit, "id") else "drone_1"

        query = DroneRecoveryQuery(
            drone_id=d_id,
            position=(float(d_pos[0]), float(d_pos[1]), float(d_pos[2])),
            battery_wh=batt_wh,
            wind_vector=(float(current_wind[0]), float(current_wind[1])),
            current_time_s=float(current_t),
        )

        targets = []
        # Target 0: Mobile Pod (van)
        if van is not None:
            v_roof = van.roof() if hasattr(van, "roof") else (van.dock_pos() if hasattr(van, "dock_pos") else (0.0, 0.0, 2.0))
            is_stopped = bool(getattr(van, "status", "") in ("spot", "wait", "parked", "loading", "docked"))
            q_len = len(van.meets.get(id(van), {}).get("units", [])) if hasattr(van, "meets") else 0
            v_res_id = f"pod_pad_{van.vid}" if hasattr(van, "vid") else "hub_roof_pad"
            if not self.pad_manager.has_resource(v_res_id):
                self.pad_manager.register_resource(v_res_id, capacity=1)
            targets.append(RecoveryTarget(
                target_id=f"pod_{getattr(van, 'vid', 1)}",
                target_type="hub",
                position=(float(v_roof[0]), float(v_roof[1]), float(v_roof[2])),
                resource_id=v_res_id,
                is_stopped_at_bay=is_stopped,
                bays_available=max(0, 1 - q_len),
            ))

        # Target 1..K: Station pads
        for idx, pad in enumerate(station_pads):
            pos = pad if isinstance(pad, (tuple, list)) else getattr(pad, "location", (0.0, 0.0, 0.0))
            s_res_id = f"station_pad_{idx}"
            if not self.pad_manager.has_resource(s_res_id):
                self.pad_manager.register_resource(s_res_id, capacity=1)
            targets.append(RecoveryTarget(
                target_id=f"station_pad_{idx}",
                target_type="station",
                position=(float(pos[0]), float(pos[1]), float(pos[2])),
                resource_id=s_res_id,
                is_stopped_at_bay=True,
                bays_available=1,
            ))

        if not targets:
            return None, None

        hub_pos = targets[0].position if targets[0].target_type == "hub" else (0.0, 0.0, 2.0)
        hub_stopped = targets[0].is_stopped_at_bay if targets[0].target_type == "hub" else True
        sim_hub = SimHub(
            hub_id=targets[0].target_id if targets[0].target_type == "hub" else "hub_1",
            position=hub_pos,
            is_stopped=hub_stopped,
            route_stops=[hub_pos],
        )

        sim_drones = {
            d_id: SimDrone(
                drone_id=d_id,
                position=(float(d_pos[0]), float(d_pos[1]), float(d_pos[2])),
                battery_wh=batt_wh,
                state=DroneState.OUTBOUND_DELIVERY,
            )
        }

        sim_stations = {}
        for idx, pad in enumerate(station_pads):
            pos = pad if isinstance(pad, (tuple, list)) else getattr(pad, "location", (0.0, 0.0, 0.0))
            sim_stations[f"station_pad_{idx}"] = SimStation(
                f"station_pad_{idx}",
                (float(pos[0]), float(pos[1]), float(pos[2])),
                capacity=1,
            )

        fleet_state = CompactFleetState(
            current_time_s=float(current_t),
            hub=sim_hub,
            drones=sim_drones,
            stations=sim_stations,
            pad_manager=self.pad_manager,
            wind_speed_ms=float(math.hypot(current_wind[0], current_wind[1])),
        )

        prior_fn = None
        if self.drone_policy is not None:
            def _prior(obs):
                try:
                    act, _ = self.drone_policy.predict(obs, deterministic=False)
                    probs = np.ones(len(targets), dtype=np.float32)
                    if act == 0 and len(targets) > 0:
                        probs[0] = 3.0
                    elif act == 1 and len(targets) > 1:
                        probs[1:] = 2.0
                    return probs
                except Exception:
                    return np.ones(len(targets), dtype=np.float32)
            prior_fn = _prior

        decision = self.planner.select_recovery_action(
            query=query,
            targets=targets,
            fleet_state=fleet_state,
            baseline=self.baseline,
            policy_prior_fn=prior_fn,
            commit_reservation=True,
        )

        chosen_target = targets[decision.selected_action_idx]
        self.decision_history.append({
            "time": float(current_t),
            "drone_id": d_id,
            "chosen_target_id": chosen_target.target_id,
            "target_type": chosen_target.target_type,
            "feasible": decision.feasible,
            "q_value": decision.q_value,
            "latency_ms": decision.decision_latency_ms,
            "booking_id": decision.booking_id,
        })

        return chosen_target, decision

