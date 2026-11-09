"""Drone navigation brain: find its own way to the drop in hidden wind.

Pure RL (user rule): no route is given and nothing steers or vetoes for the
brain. Each episode the drone takes off from a pod roof somewhere in a pod
zone and must reach a delivery drop (hover 4-10 m above the ground there,
almost stopped). Everything solid is real: buildings, tree trunks and
canopies, poles, lamps, signs (prepare_heights.py, 2 m cells). Touching any of
it is a crash; entering a no-fly zone ends the flight too. The wind
(wind.py) is never in the observation.

What the brain senses (like a real delivery drone):
- noisy GPS/IMU: velocity, acceleration, attitude, battery;
- the drop as a vector from itself; height above ground and above whatever
  is right below it (downward rangefinder);
- 16 range readings around it at its own height, out to 40 m (a small
  LiDAR ring), and 8 readings 5 m lower (what it would descend onto);
- distances to the known no-fly zones in 8 directions (a map, not a guard).
Control: roll, pitch, thrust, yaw rate (drone_dyn.py), 20 Hz.

ドローン航法ブレイン：経路は与えず、センサーで建物・樹木・飛行禁止を避けて投下点へ。
"""
import math
import os

import gymnasium as gym
import numpy as np

from rl_delivery.drone_dyn import Drone, MAX_TILT
from rl_delivery.env_flight import load_map
from rl_delivery.wind import BuildingGrid, Wind

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 0.05
RAYS = 16
RANGE = 40.0
NOFLY_DIRS = 8
NOFLY_RANGE = 150.0
MAX_S = 240.0
CEILING_AGL = 150.0
PROG_K = 0.3              # reward per metre closer / 1 m 近づくごとの報酬
NEAR_BONUS, NEAR_SCALE = 25.0, 15.0   # extra pull in the last metres / 最後の数十 m の引き
CIRCLE_COST = 0.02        # per (m/s sideways round the drop) per step / 周回のコスト       # Japan's drone ceiling above ground (航空法) / 飛行高度の上限


class Heights(object):
    def __init__(self, map_name):
        h = np.load(os.path.join(HERE, "maps", map_name + "_heights.npz"))
        self.x0, self.y0, self.c = float(h["x0"]), float(h["y0"]), float(h["cell"])
        self.ground, self.top = h["ground"], h["top"]
        self.nx, self.ny = self.top.shape

    def idx(self, x, y):
        ix = np.clip(((np.asarray(x) - self.x0) / self.c).astype(int), 0, self.nx - 1)
        iy = np.clip(((np.asarray(y) - self.y0) / self.c).astype(int), 0, self.ny - 1)
        return ix, iy

    def top_at(self, x, y):
        return self.top[self.idx(x, y)]

    def ground_at(self, x, y):
        return self.ground[self.idx(x, y)]


def rot(yaw, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]])


class NavEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, map_name="OitaKosen", wind_level=0.0, seed=None):
        self.m = load_map(map_name)
        self.H = Heights(map_name)
        self.grid = BuildingGrid(self.m["buildings"], self.m["bounds"])
        self.nofly = self.m["zones"]["nofly"]
        self.zones = self.m["zones"]["pods"]
        self.wind_level = wind_level
        self.rng = np.random.default_rng(seed)
        ang = np.arange(RAYS) * 2 * math.pi / RAYS
        self.ray_dirs = np.stack([np.cos(ang), np.sin(ang)], 1)
        self.ray_d = np.linspace(2.0, RANGE, 20)
        ang8 = np.arange(NOFLY_DIRS) * 2 * math.pi / NOFLY_DIRS
        self.nf_dirs = np.stack([np.cos(ang8), np.sin(ang8)], 1)
        n_obs = 3 + 3 + 3 + 2 + 2 + 3 + RAYS + 8 + NOFLY_DIRS + 4 + 1
        self.observation_space = gym.spaces.Box(-10.0, 10.0, (n_obs,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (4,), np.float32)
        self.drops = [b for b in self.m["buildings"] if b["deliverable"]]
        self.log = None

    # -- world queries / 世界の問い合わせ
    def _in_nofly(self, p, grow=0.0):
        return any(r["x0"] - grow <= p[0] <= r["x1"] + grow and r["y0"] - grow <= p[1] <= r["y1"] + grow
                   for r in self.nofly)

    def _zone_of(self, p):
        for k, r in self.zones.items():
            if r["x0"] <= p[0] <= r["x1"] and r["y0"] <= p[1] <= r["y1"]:
                return k
        return None

    def _open_sky(self, p, r):
        """Nothing above 1.5 m within r metres (no canopy overhead). / 上空が開いているか"""
        xs = np.arange(-r, r + 0.1, 2.0)
        X, Y = np.meshgrid(p[0] + xs, p[1] + xs)
        return bool((self.H.top_at(X, Y) - self.H.ground_at(X, Y) < 1.5).all())

    def _phi(self, d):
        """Closeness score: rises as d falls, steeply near the drop. / 近さの得点"""
        return -PROG_K * d + NEAR_BONUS * math.exp(-d / NEAR_SCALE)

    def _ranges(self, p, dz=0.0):
        """Free distance along each ray at height p.z + dz. / 各方向の空き距離"""
        X = p[0] + self.ray_dirs[:, 0:1] * self.ray_d[None, :]
        Y = p[1] + self.ray_dirs[:, 1:2] * self.ray_d[None, :]
        hit = self.H.top_at(X, Y) > p[2] + dz
        first = np.where(hit.any(1), hit.argmax(1), len(self.ray_d))
        return np.where(first < len(self.ray_d), self.ray_d[np.minimum(first, len(self.ray_d) - 1)], RANGE)

    def _nofly_dist(self, p):
        out = np.full(NOFLY_DIRS, NOFLY_RANGE)
        steps = np.linspace(5.0, NOFLY_RANGE, 30)
        for k, d in enumerate(self.nf_dirs):
            for s in steps:
                if self._in_nofly((p[0] + d[0] * s, p[1] + d[1] * s)):
                    out[k] = s
                    break
        return out

    # -- episodes / エピソード
    def _pick(self):
        """Take-off spot on a road in a pod zone, and a drop in the same zone
        (or a nearby one), both clear of no-fly zones. / 離陸地点と投下点"""
        keep = self.m["keep"]
        for _ in range(200):
            n = self.m["nodes"][keep[int(self.rng.integers(len(keep)))]]
            start = np.array(n[:2], float)
            z = self._zone_of(start)
            if z is None or self._in_nofly(start, 20.0) or not self._open_sky(start, 3.0):
                continue
            r = self.zones[z]
            cand = [b for b in self.drops if r["x0"] <= b["x"] <= r["x1"] and r["y0"] <= b["y"] <= r["y1"]
                    and not self._in_nofly((b["x"], b["y"]), 15.0)
                    and 30.0 < math.dist((b["x"], b["y"]), start) < self.max_dist]
            if cand:
                b = cand[int(self.rng.integers(len(cand)))]
                # the drop: just outside the building, on the side facing the start / 建物の手前
                v = start - np.array([b["x"], b["y"]])
                v /= np.linalg.norm(v) + 1e-9
                drop = np.array([b["x"], b["y"]]) + v * (max(b["len"], b["wid"]) / 2.0 + 3.0)
                if not self._open_sky(drop, 2.0):
                    continue
                return start, drop, z
        return start, start + np.array([80.0, 0.0]), z

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.max_dist = float(getattr(self, "curriculum_dist", 600.0))
        start, drop, self.zone = self._pick()
        g0 = float(self.H.ground_at(start[0], start[1]))
        self.drone = Drone([start[0], start[1], g0 + 2.6])            # on the pod roof / ポッド屋根
        gd = float(self.H.ground_at(drop[0], drop[1]))
        self.goal = np.array([drop[0], drop[1], gd + 7.0])
        self.wind = Wind(self.rng, level=self.wind_level, grid=self.grid)
        self.n = 0
        self.d_best = float(np.linalg.norm(self.goal - self.drone.p))
        self.d0 = self.d_best
        self.phi_last = self._phi(self.d_best)
        self.last_act = np.zeros(4)
        self.stats = {"reached": False, "crash": None, "min_clear": RANGE, "energy": 0.0}
        self.trail = []
        return self._obs(), {}

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
            rot(yaw, gp) / 100.0, rot(yaw, v) / 15.0, rot(yaw, acc) / 10.0,
            [math.sin(roll) / math.sin(MAX_TILT), math.sin(pitch) / math.sin(MAX_TILT)],
            [min(dist, 1000.0) / 1000.0, math.atan2(*rot(yaw, gp)[1::-1]) / math.pi],
            [(p[2] - g_here) / 50.0, (p[2] - t_here) / 20.0, (self.goal[2] - p[2]) / 30.0],
            np.roll(rng, -int(round(yaw / (2 * math.pi / RAYS)))) / RANGE,
            np.roll(rng_low, -int(round(yaw / (2 * math.pi / 8)))) / RANGE,
            np.roll(nfd, -int(round(yaw / (2 * math.pi / NOFLY_DIRS)))) / NOFLY_RANGE,
            self.last_act, [self.drone.battery / 100.0]])
        self._clear = float(min(rng.min(), max(p[2] - t_here, 0.0) + 0.0))
        return np.clip(o, -10, 10).astype(np.float32)

    def step(self, action):
        act = np.asarray(action, float)
        dr = self.drone
        agl = max(dr.p[2] - float(self.H.ground_at(dr.p[0], dr.p[1])), 0.5)
        self.wind.step(DT, agl)
        w = self.wind.at(dr.p[0], dr.p[1], agl)
        dr.step(act, w, DT)
        self.n += 1
        obs = self._obs()
        d = float(np.linalg.norm(self.goal - dr.p))
        r = 0.0
        # closer = more reward, away = the same loss; steeper in the last ~15 m
        # (potential-based, so hovering anywhere earns nothing) / 近いほど報酬（離れると同じだけ減る）
        phi = self._phi(d)
        r += phi - self.phi_last
        self.phi_last = phi
        self.d_best = min(self.d_best, d)
        # circling the drop: sideways speed round it within 30 m costs / 投下点の周回にコスト
        hv = self.goal[:2] - dr.p[:2]
        hd = float(np.linalg.norm(hv))
        if hd < 30.0 and hd > 0.5:
            radial = hv / hd
            tang = dr.v[:2] - radial * float(np.dot(dr.v[:2], radial))
            r -= CIRCLE_COST * float(np.linalg.norm(tang)) * (1.0 - hd / 30.0 + 0.3)
        r -= 0.005                                                      # time / 時間
        r -= 0.003 * (dr.thr - 1.0) ** 2 + 0.003 * float(np.sum(np.square(act - self.last_act)))
        if self._clear < 3.0:
            r -= 0.02 * (3.0 - self._clear)                             # too close / 近すぎ
        agl_now = dr.p[2] - float(self.H.ground_at(dr.p[0], dr.p[1]))
        if agl_now > 60.0:
            r -= 0.002 * (agl_now - 60.0)                               # needlessly high / 高すぎ
        r -= 0.02 * max(0.0, float(self.drone.battery_drop))            # battery is a cost / 電池消費
        self.stats["min_clear"] = min(self.stats["min_clear"], self._clear)
        self.last_act = act
        term = trunc = False
        top = float(self.H.top_at(dr.p[0], dr.p[1]))
        if dr.p[2] < top + 0.2 and self.n > 10:
            self.stats["crash"] = "obstacle" if top > float(self.H.ground_at(dr.p[0], dr.p[1])) + 0.5 else "ground"
        elif self._in_nofly(dr.p):
            self.stats["crash"] = "nofly"
        elif dr.p[2] - float(self.H.ground_at(dr.p[0], dr.p[1])) > CEILING_AGL:
            self.stats["crash"] = "ceiling"
        elif dr.battery <= 0:
            self.stats["crash"] = "battery"
        if self.stats["crash"]:
            r -= 50.0
            term = True
        elif float(np.linalg.norm(self.goal[:2] - dr.p[:2])) < 2.5 and abs(self.goal[2] - dr.p[2]) < 3.0 \
                and float(np.linalg.norm(dr.v)) < 1.5:
            r += 100.0 + 20.0 * dr.battery / 100.0
            term = True
            self.stats["reached"] = True
        elif self.n * DT >= MAX_S:
            # not delivering is a failure too (no safe "hover until time is up") / 時間切れも失敗
            r -= 50.0
            term = True
            self.stats["crash"] = "timeout"
        if self.log is not None and self.n % 2 == 0:
            self.log.append({"t": round(self.n * DT, 2), "p": dr.p.round(2).tolist(), "wind": w.round(2).tolist(),
                             "cmd": np.round(act, 3).tolist(), "clear": round(self._clear, 1)})
        info = {}
        if term or trunc:
            info = {"kpi": {"reached": self.stats["reached"], "crash": self.stats["crash"] or "",
                            "time_s": self.n * DT, "battery_used": 100.0 - dr.battery,
                            "min_clear": self.stats["min_clear"], "start_dist": self.d0,
                            "end_dist": d}}
        return obs, float(r), term, trunc, info


class ScriptedPilot(object):
    """Baseline only (never in control of a trained agent): climb to 45 m above
    ground, fly straight at the drop, descend. No obstacle avoidance.
    比較用の単純な操縦（上昇・直進・降下、回避なし）。"""

    def __init__(self, env):
        self.env = env

    def act(self, obs):
        e = self.env
        d = e.drone
        g = e.goal - d.p
        gnd = float(e.H.ground_at(d.p[0], d.p[1]))
        hd = float(np.linalg.norm(g[:2]))
        tz = gnd + 45.0 if hd > 15 else e.goal[2]
        vref = np.zeros(3)
        if d.p[2] > gnd + 30 or hd < 15:
            vref[:2] = g[:2] / max(hd, 1e-3) * min(10.0, 0.4 * hd)
        vref[2] = float(np.clip(tz - d.p[2], -3, 4))
        a = 1.2 * (vref - d.v)
        c, s = math.cos(d.yaw), math.sin(d.yaw)
        ab = (c * a[0] + s * a[1], -s * a[0] + c * a[1])
        k = 9.81 * math.tan(MAX_TILT)
        return np.array([np.clip(-ab[1] / k, -1, 1), np.clip(ab[0] / k, -1, 1), np.clip(a[2] / 9.81, -1, 1), 0.0])
