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


class DriveBridge(object):
    """Bridge manager for neural pod driving (SAC, continuous steer & throttle/brake)."""

    def __init__(self, checkpoint_path, map_name="OitaKosen"):
        from stable_baselines3 import SAC
        resolved = resolve_checkpoint("drive", checkpoint_path)
        print("[DriveBridge] Loading checkpoint:", resolved)
        self.model = SAC.load(resolved)
        self.map_name = map_name

        with open(os.path.join(RL_DIR, "maps", map_name + "_drive.json")) as f:
            D = json.load(f)
        with open(os.path.join(RL_DIR, "maps", map_name + ".json")) as f:
            M = json.load(f)

        self.lanes = D["lanes"]
        self.stops = {}
        for s in D["stops"]:
            self.stops.setdefault(s["edge"], []).append(("stop", s["k"], None))
        for s in D["signals"]:
            self.stops.setdefault(s["edge"], []).append(("signal", s["k"], (s["junction"], s["phase"])))
        self.n_junctions = D["junctions"]

        self.P = None
        self.S = None
        self.marks = []
        self.k = 0
        self.s = 0.0
        self.last_act = np.zeros(2, dtype=np.float32)
        self.steer = 0.0
        self.done = False

    def build_from_waypoints(self, waypoints):
        """Build continuous centerline, curvature, and marks from CARLA waypoints."""
        if not waypoints:
            return
        pts, s_acc = [], []
        s = 0.0
        for i, item in enumerate(waypoints):
            wp = item[0] if isinstance(item, (tuple, list)) else item
            loc = wp.transform.location if hasattr(wp, "transform") else wp
            yaw = math.radians(wp.transform.rotation.yaw) if hasattr(wp, "transform") else 0.0
            lw = wp.lane_width if hasattr(wp, "lane_width") else 3.5
            is_junc = int(wp.is_junction) if hasattr(wp, "is_junction") else 0
            if pts:
                s += math.hypot(loc.x - pts[-1][0], loc.y - pts[-1][1])
            # [x, y, z, yaw, lane_width, ka, kb, speed_limit, junction]
            pts.append([loc.x, loc.y, loc.z, yaw, lw, 0, 0, 40.0, is_junc])
            s_acc.append(s)

        if len(pts) < 2:
            return

        raw = np.array([[q[0], q[1], q[3], q[4], q[5], q[6], q[7], q[8]] for q in pts])
        S0 = np.array(s_acc)
        keep = np.concatenate([[True], np.diff(S0) > 0.05])
        raw, S0 = raw[keep], S0[keep]
        if S0[-1] < 1.0:
            return
        S = np.arange(0.0, S0[-1], 1.0)
        idx = np.clip(np.searchsorted(S0, S, side="right") - 1, 0, len(S0) - 1)
        x = np.interp(S, S0, raw[:, 0])
        y = np.interp(S, S0, raw[:, 1])
        hd = np.arctan2(np.gradient(y), np.gradient(x))
        self.P = np.column_stack([x, y, hd, raw[idx, 3], raw[idx, 4], raw[idx, 5], raw[idx, 6], raw[idx, 7]])
        self.S = S
        self.k = 0
        self.s = 0.0
        self.done = False

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

    def _wrap(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def run_step(self, car_actor, world=None):
        """Execute one neural driving step and return a carla.VehicleControl."""
        import carla

        if self.P is None or len(self.P) == 0:
            return carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)

        tf = car_actor.get_transform()
        vel = car_actor.get_velocity()
        v_current = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        yaw_current = math.radians(tf.rotation.yaw)

        self.s, off, q = self._project(tf.location.x, tf.location.y)
        if self.s >= self.S[-1] - 3.0:
            self.done = True
            return carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)

        herr = self._wrap(yaw_current - q[2])
        AHEAD = (5.0, 10.0, 20.0, 30.0, 40.0)
        curv = [self._wrap(self._at(self.s + d)[2] - q[2]) for d in AHEAD]

        # Stop lines & signals
        mobs = [1.0, 0, 0, 0, 0, 0, 0]  # default clear

        # Traffic vehicles query
        lead = 99.0
        lv = v_current
        if world is not None:
            actors = world.get_actors().filter("vehicle.*")
            for a in actors:
                if a.id == car_actor.id:
                    continue
                loc = a.get_location()
                d_euclid = loc.distance(tf.location)
                if d_euclid < 60.0:
                    along = (loc.x - tf.location.x) * math.cos(yaw_current) + (loc.y - tf.location.y) * math.sin(yaw_current)
                    if 0.5 < along < lead:
                        lead = along
                        a_vel = a.get_velocity()
                        lv = math.sqrt(a_vel.x ** 2 + a_vel.y ** 2 + a_vel.z ** 2)

        ka, kb = int(q[4]), int(q[5])
        POD_MAX = 30.0 / 3.6
        MAX_STEER = 0.6
        o = [
            v_current / POD_MAX,
            self.steer / MAX_STEER,
            off / 2.0,
            herr,
            q[3] / 4.0,
            float(ka == 0), float(ka == 1), float(ka == 2),
            float(kb == 0), float(kb == 1), float(kb == 2),
            float(q[7])
        ] + curv + mobs + [
            min(lead, 60.0) / 60.0,
            (v_current - lv) / 10.0,
            q[6] / 60.0,
            min(self.S[-1] - self.s, 800.0) / 800.0
        ] + list(self.last_act)

        obs = np.clip(np.array(o[:30], np.float32), -10, 10)
        act = self.model.predict(obs, deterministic=True)[0]
        self.last_act = act.copy()

        # Update steering with steer rate limit (0.6 rad max, 1.2 rad/s)
        DT = 0.1
        target_steer = act[0] * MAX_STEER
        self.steer += float(np.clip(target_steer - self.steer, -1.2 * DT, 1.2 * DT))

        steer_norm = float(np.clip(self.steer / MAX_STEER, -1.0, 1.0))
        throttle = float(np.clip(max(act[1], 0.0), 0.0, 1.0))
        brake = float(np.clip(max(-act[1], 0.0), 0.0, 1.0))

        return carla.VehicleControl(steer=steer_norm, throttle=throttle, brake=brake)


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

