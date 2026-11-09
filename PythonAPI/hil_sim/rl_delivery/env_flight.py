"""Flight brain environment: follow a 3-D path in hidden wind.

Each episode is a random delivery-like flight over the map: take-off point,
3-6 waypoints at 15-60 m with diagonal climbs and descents, final hover over
the drop. The drone is controlled 20 times a second (roll, pitch, thrust,
yaw rate). The brain sees only sensor-like values relative to the path; the
wind (wind.py) is never in the observation, so it has to feel it.

No-fly zones are enforced by a safety guard outside the brain: if the drone
would be inside one within 1.5 s, the guard takes over (brakes and turns
away), and the brain is penalised for every guard step.

飛行ブレイン用の環境。風は観測せず、経路追従を学ぶ。飛行禁止区域はガードで保証。
"""
import json
import math
import os

import gymnasium as gym
import numpy as np

from rl_delivery.drone_dyn import Drone, MAX_TILT
from rl_delivery.wind import BuildingGrid, Wind

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 0.05
CRUISE = 10.0             # path speed [m/s] / 巡航速度
MAX_DEV = 15.0            # off the path this far ends the episode / 逸脱上限
GUARD_LOOK = 1.5          # s ahead / ガードの先読み
GUARD_MARGIN = 10.0       # m / ガードの余裕


def load_map(name):
    with open(os.path.join(HERE, "maps", name + ".json")) as f:
        return json.load(f)


def rot(yaw, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]])


class FlightEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, map_name="OitaKosen", wind_level=1.0, seed=None, max_s=150.0):
        self.m = load_map(map_name)
        self.bounds = self.m["bounds"]
        self.grid = BuildingGrid(self.m["buildings"], self.bounds)
        self.nofly = self.m["zones"]["nofly"]
        self.wind_level = wind_level
        self.max_steps = int(max_s / DT)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (24,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (4,), np.float32)
        self.rng = np.random.default_rng(seed)
        self.log = None          # set a list to record every step (watch / KPIs)

    # -- path / 経路
    def _path(self):
        x0, y0, x1, y1 = self.bounds
        for _ in range(100):
            p = [np.array([self.rng.uniform(x0 + 50, x1 - 50), self.rng.uniform(y0 + 50, y1 - 50), 2.0])]
            p.append(p[0] + np.array([0, 0, self.rng.uniform(15, 40)]))         # climb out / 離陸
            for _k in range(self.rng.integers(2, 5)):
                ang = self.rng.uniform(0, 2 * math.pi)
                d = self.rng.uniform(60, 220)
                z = float(np.clip(p[-1][2] + self.rng.uniform(-15, 20), 15, 60))
                p.append(p[-1] + np.array([d * math.cos(ang), d * math.sin(ang), z - p[-1][2]]))
            last = p[-1].copy()
            last[2] = self.rng.uniform(6, 10)                                  # drop hover / 投下高度
            p.append(last)
            pts = np.array(p)
            if (pts[:, 0].min() > x0 and pts[:, 0].max() < x1 and pts[:, 1].min() > y0
                    and pts[:, 1].max() < y1 and not self._crosses_nofly(pts)):
                return pts
        return pts

    def _crosses_nofly(self, pts, grow=GUARD_MARGIN + 15.0):
        """Any point every 5 m along the path near a no-fly zone? / 経路が禁止区域に近いか"""
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(2, int(np.linalg.norm(b - a) / 5.0))
            for u in np.linspace(0, 1, n):
                if self._in_nofly(a + u * (b - a), grow):
                    return True
        return False

    def _in_nofly(self, q, grow):
        return any(r["x0"] - grow <= q[0] <= r["x1"] + grow and r["y0"] - grow <= q[1] <= r["y1"] + grow
                   for r in self.nofly)

    def _project(self, q):
        """(distance, closest point, tangent, s along path, segment) on the
        current leg or the next two; never goes back, and at a corner the
        later leg wins. / 現在と次の区間だけで最近点を探す（後戻りしない）"""
        best = None
        last = len(self.path) - 2
        s_acc = sum(self.seg_len[:self.seg])
        for i in range(self.seg, min(self.seg + 2, last) + 1):
            a, b = self.path[i], self.path[i + 1]
            ab = b - a
            L = self.seg_len[i] or 1e-6
            u = float(np.clip(np.dot(q - a, ab) / L ** 2, 0, 1))
            c = a + u * ab
            d = float(np.linalg.norm(q - c))
            if best is None or d <= best[0] + 0.5:
                best = (d, c, ab / L, s_acc + u * L, i)
            s_acc += L
        return best

    def _ahead(self, s, dist):
        s_t = min(s + dist, self.total)
        acc = 0.0
        for i in range(len(self.path) - 1):
            L = self.seg_len[i]
            if acc + L >= s_t:
                return self.path[i] + (s_t - acc) / L * (self.path[i + 1] - self.path[i])
            acc += L
        return self.path[-1]

    def v_ref(self, s):
        """Target speed at path position s: brake ahead of each corner and
        the drop (a = 2.5 m/s2). / 角と投下点の手前で減速する目標速度"""
        v = CRUISE
        acc = 0.0
        for i in range(len(self.path) - 1):
            acc += self.seg_len[i]
            d = acc - s
            if d < 0:
                continue
            if i + 1 >= len(self.path) - 1:
                vc = 0.0
            else:
                a = self.path[i + 1] - self.path[i]
                b = self.path[i + 2] - self.path[i + 1]
                cosang = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
                vc = CRUISE * max(0.15, (1.0 + cosang) / 2.0)
            v = min(v, math.sqrt(vc ** 2 + 2 * 2.5 * d))
        return v

    # -- gym
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.path = self._path()
        self.seg_len = [float(np.linalg.norm(self.path[i + 1] - self.path[i])) for i in range(len(self.path) - 1)]
        self.total = float(sum(self.seg_len))
        self.seg = 0
        self.drone = Drone(self.path[0])
        self.wind = Wind(self.rng, level=self.wind_level, grid=self.grid)
        self.n = 0
        self.s_best = 0.0
        self.last_act = np.zeros(4)
        self.guard_steps = 0
        self.stats = {"xtrack": [], "energy": 0.0, "guard": 0, "reached": False, "crash": False}
        return self._obs(), {}

    def _obs(self):
        p, v, acc, roll, pitch, yaw = self.drone.sensors(self.rng)
        d, c, tan, s, i = self._project(p)
        self.seg = max(self.seg, i)
        look = self._ahead(s, 15.0)
        remain = self.total - s
        o = np.concatenate([
            rot(yaw, c - p) / 10.0, rot(yaw, tan), rot(yaw, look - p) / 20.0,
            rot(yaw, v) / 15.0, rot(yaw, acc) / 10.0,
            [roll / MAX_TILT, pitch / MAX_TILT, min(remain, 200.0) / 200.0, (p[2] - c[2]) / 10.0],
            self.last_act, [self.drone.battery / 100.0]])
        self._cache = (p, c, tan, s, remain, d)
        return o.astype(np.float32)

    def _guard(self, act):
        """Override the brain if a no-fly zone is 1.5 s ahead. / 飛行禁止の先読みガード"""
        dr = self.drone
        ahead = dr.p + dr.v * GUARD_LOOK
        if not self._in_nofly(ahead, GUARD_MARGIN):
            return act, False
        # brake: tilt against the ground velocity, hold height / 減速して高度維持
        vb = rot(dr.yaw, dr.v)
        cmd = np.array([np.clip(vb[1] / 6.0, -1, 1), np.clip(-vb[0] / 6.0, -1, 1),
                        np.clip(-dr.v[2] * 0.3, -1, 1), 0.0])
        return cmd, True

    def step(self, action):
        act = np.asarray(action, float)
        cmd, guarded = self._guard(act)
        dr = self.drone
        agl = max(dr.p[2], 0.5)
        self.wind.step(DT, agl)
        w = self.wind.at(dr.p[0], dr.p[1], agl)
        dr.step(cmd, w, DT)
        self.n += 1
        obs = self._obs()
        p, c, tan, s, remain, dev = self._cache
        true_dev = float(np.linalg.norm(dr.p - c))
        prog = max(0.0, s - self.s_best)
        self.s_best = max(self.s_best, s)
        speed_err = abs(float(np.dot(dr.v, tan)) - self.v_ref(s))
        r = (0.1 * prog - 0.04 * true_dev - 0.004 * true_dev ** 2 - 0.01 * speed_err
             - 0.005 * float(np.sum(np.square(act - self.last_act))) - 0.002 * (dr.thr - 1.0) ** 2)
        self.last_act = act
        self.stats["xtrack"].append(true_dev)
        if guarded:
            self.guard_steps += 1
            self.stats["guard"] += 1
            r -= 1.0
        term = trunc = False
        if remain < 2.0 and np.linalg.norm(dr.v) < 1.0 and true_dev < 2.0:
            r += 20.0
            term = True
            self.stats["reached"] = True
        elif dr.p[2] < 0.3 or true_dev > MAX_DEV or dr.battery <= 0:
            r -= 30.0
            term = True
            self.stats["crash"] = dr.p[2] < 0.3
        elif self.n >= self.max_steps:
            trunc = True
        if self.log is not None:
            self.log.append({"t": round(self.n * DT, 2), "p": dr.p.round(2).tolist(), "c": c.round(2).tolist(),
                             "wind": w.round(2).tolist(), "cmd": np.round(cmd, 3).tolist(),
                             "dev": round(true_dev, 2), "guard": guarded})
        info = {}
        if term or trunc:
            x = self.stats["xtrack"]
            info = {"kpi": {"reached": self.stats["reached"], "crash": self.stats["crash"],
                            "xtrack_mean": float(np.mean(x)), "xtrack_max": float(np.max(x)),
                            "guard_steps": self.stats["guard"], "battery_used": 100.0 - dr.battery,
                            "time_s": self.n * DT, "path_m": self.total}}
        return obs, float(r), term, trunc, info


class PidPilot(object):
    """Classic cascaded PID autopilot (the baseline the RL pilot must beat).
    古典的な PID 自動操縦（RL の比較対象）。"""

    def __init__(self, env):
        self.env = env
        self.i = np.zeros(3)

    def act(self, obs):
        p, c, tan, s, remain, _ = self.env._cache
        dr = self.env.drone
        v_ref = tan * self.env.v_ref(s) + (c - p) * 0.8
        ev = v_ref - dr.v
        self.i = np.clip(self.i + ev * DT, -5, 5)
        a_ref = 1.2 * ev + 0.3 * self.i
        ab = rot(dr.yaw, a_ref)
        return np.array([np.clip(-ab[1] / (9.81 * math.tan(MAX_TILT)), -1, 1),
                         np.clip(ab[0] / (9.81 * math.tan(MAX_TILT)), -1, 1),
                         np.clip(a_ref[2] / 9.81, -1, 1), 0.0])
