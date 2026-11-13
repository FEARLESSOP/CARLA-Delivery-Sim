"""Pod driving brain environment: drive a route lawfully, never touch a kerb.

The pod (LMP-4 size, kinematic bicycle model) drives a random route through
OitaKosen's real lanes (prepare_drive.py). The brain steers, accelerates and
brakes 10 times a second; nothing drives for it.
World rules, all scored (pure RL):
- kerb: the pod's side past a kerb-side lane edge = kerb strike (ends the run);
- centre line: crossing into the oncoming lane costs every step;
- 止まれ: the pod must come to a full stop (< 0.3 m/s) within 4 m before the
  stop line before crossing it, else a violation;
- signals: crossing a signal's stop line on red is a violation (cycle 60 s,
  two phases by approach direction, random offset per junction);
- speed: over the lane limit costs; other cars ahead on the route stop at
  red lights and stop signs and follow each other (IDM); hitting one ends it;
- progress along the route pays, reaching the end pays more, time costs.
What the brain senses: speed, steering, its offset and heading error to the
lane, lane width and what lies beyond each edge (kerb / lane / oncoming),
the road's bends ahead, the next stop line (stop sign or signal + light
colour, whether it already stopped), the car ahead (gap, closing speed), the
speed limit and the distance to the route's end.

ポッド運転ブレインの環境（縁石・止まれ・信号・速度・前方車両）。
"""
import json
import math
import os

import gymnasium as gym
import networkx as nx
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 0.1
WHEELBASE, HALF_W, LENGTH = 2.6, 0.8, 3.6
MAX_STEER, STEER_RATE = 0.6, 1.2          # rad, rad/s / 最大舵角と舵角速度
# symmetric pedal: with 2.5 forward / 6.0 brake, random exploration braked more than it
# accelerated and the pod never got moving / 対称のペダル（非対称だと探索で発進しない）
MAX_ACC, MAX_BRAKE = 3.0, 3.0
POD_MAX = 30.0 / 3.6                       # delivery_sim --speed / ポッド最高速
CYCLE, GREEN, YELLOW = 60.0, 27.0, 3.0
AHEAD = (5.0, 10.0, 20.0, 30.0, 40.0)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class DriveEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, map_name="OitaKosen", seed=None):
        with open(os.path.join(HERE, "maps", map_name + "_drive.json")) as f:
            D = json.load(f)
        with open(os.path.join(HERE, "maps", map_name + ".json")) as f:
            M = json.load(f)
        self.lanes, self.M = D["lanes"], M
        self.stops = {}
        for s in D["stops"]:
            self.stops.setdefault(s["edge"], []).append(("stop", s["k"], None))
        for s in D["signals"]:
            self.stops.setdefault(s["edge"], []).append(("signal", s["k"], (s["junction"], s["phase"])))
        self.n_junctions = D["junctions"]
        g = nx.DiGraph()
        self.edge_of = {}
        for i, (u, v, L, _kmh, _li, _j) in enumerate(M["edges"]):
            if not g.has_edge(u, v) or L < g[u][v]["w"]:
                g.add_edge(u, v, w=L)
                self.edge_of[(u, v)] = i
        self.g = g
        self.out = {}
        for i, (u, v, *_r) in enumerate(M["edges"]):
            self.out.setdefault(u, []).append(i)
        self.rng = np.random.default_rng(seed)
        self.route_len = 300.0
        self.traffic = 3
        self.observation_space = gym.spaces.Box(-10.0, 10.0, (30,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
        self.log = None

    # -- route / 経路
    def _route(self):
        E = self.M["edges"]
        for _ in range(100):
            e0 = int(self.rng.integers(len(E)))
            if self.lanes[e0]["junction"] or E[e0][2] < 20:
                continue
            u0 = E[e0][1]
            try:
                dist = nx.single_source_dijkstra_path_length(self.g, u0, cutoff=self.route_len * 1.3, weight="w")
            except nx.NetworkXError:
                continue
            cand = [n for n, d in dist.items() if self.route_len * 0.7 < d < self.route_len * 1.3]
            if not cand:
                continue
            goal = int(self.rng.choice(cand))
            nodes = nx.shortest_path(self.g, u0, goal, weight="w")
            edges = [e0] + [self.edge_of[(a, b)] for a, b in zip(nodes[:-1], nodes[1:])]
            return edges
        return [e0]

    def _build(self, edges):
        pts, s_acc, marks = [], [], []
        s = 0.0
        for ei in edges:
            L = self.lanes[ei]
            base = len(pts)
            for q in (L["pts"] if not pts else L["pts"][1:]):
                if pts:
                    s += math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1])
                pts.append(q[:7] + [min(L["kmh"], 60.0), L["junction"]])
                s_acc.append(s)
            for kind, k, arg in self.stops.get(ei, ()):
                idx = base + max(0, k - (1 if base else 0))
                if idx < len(pts):
                    marks.append([s_acc[idx], kind, arg, False, False])   # s, kind, signal, stopped, passed
        # resample every 1 m; headings from the line itself (the xodr headings
        # are off at piece ends) / 1 m ごとに再標本化、方位は線から
        raw = np.array([[q[0], q[1], q[3], q[4], q[5], q[6], q[7], q[8]] for q in pts])
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
        self.marks = sorted(marks, key=lambda m: m[0])

    # -- geometry / 幾何
    def _project(self, x, y):
        lo, hi = max(0, self.k - 5), min(len(self.P), self.k + 30)
        P = self.P[lo:hi]
        d2 = (P[:, 0] - x) ** 2 + (P[:, 1] - y) ** 2
        j = int(np.argmin(d2))
        self.k = lo + j
        q = self.P[self.k]
        h = q[2]
        nx_, ny_ = -math.sin(h), math.cos(h)
        off = (x - q[0]) * nx_ + (y - q[1]) * ny_            # + = side a / a 側が正
        along = (x - q[0]) * math.cos(h) + (y - q[1]) * math.sin(h)
        return self.S[self.k] + along, off, q

    def _signal(self, j, phase):
        t = (self.t + self.sig_off[j]) % CYCLE
        t = (t - phase * CYCLE / 2.0) % CYCLE
        return 0 if t < GREEN else (1 if t < GREEN + YELLOW else 2)   # green / yellow / red

    def _at(self, s):
        k = int(np.clip(np.searchsorted(self.S, s), 0, len(self.S) - 1))
        return self.P[k]

    # -- traffic / 交通
    def _cars_step(self):
        for c in self.cars:
            gap = 1e9
            for o in self.cars:
                if o is not c and 0 < o["s"] - c["s"] < gap:
                    gap = o["s"] - c["s"] - LENGTH
            if 0 < self.s - c["s"] < gap + LENGTH:
                gap = self.s - c["s"] - LENGTH
            for m in self.marks:                               # cars obey signals / stop signs
                if 0 < m[0] - c["s"] < 30:
                    if m[1] == "signal" and self._signal(*m[2]) > 0:
                        gap = min(gap, m[0] - c["s"] - 1.0)
                    elif m[1] == "stop" and not c.get("stopped_at") == m[0]:
                        gap = min(gap, m[0] - c["s"] - 1.0)
                        if c["v"] < 0.2 and m[0] - c["s"] < 3:
                            c["stopped_at"] = m[0]
            v0 = c["v0"]
            a = 1.5 * (1 - (c["v"] / v0) ** 4 - ((2.0 + c["v"] * 1.5) / max(gap, 0.5)) ** 2)
            c["v"] = max(0.0, c["v"] + max(a, -6.0) * DT)
            c["s"] += c["v"] * DT

    # -- gym
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._build(self._route())
        q = self.P[0]
        self.x, self.y, self.yaw = q[0], q[1], q[2]
        self.v, self.steer = 0.0, 0.0
        self.k, self.t = 0, 0.0
        self.s, self.off, _ = self._project(self.x, self.y)
        self.s_best = self.s
        self.sig_off = self.rng.uniform(0, CYCLE, max(1, self.n_junctions))
        self.cars = []
        for _ in range(int(self.rng.integers(0, self.traffic + 1))):
            s0 = float(self.rng.uniform(25.0, max(30.0, self.S[-1] - 20.0)))
            if all(abs(s0 - c["s"]) > 15 for c in self.cars):
                self.cars.append({"s": s0, "v": float(self.rng.uniform(3, 8)), "v0": float(self.rng.uniform(6, 11))})
        self.last = np.zeros(2)
        self.clear_speeds = []
        self.k_stats = {"kerb": 0, "stop_violation": 0, "red_violation": 0, "centre_s": 0.0, "speeding_s": 0.0,
                        "collision": 0, "reached": False, "end": "", "stalls": 0, "p10_speed": 0.0}
        self.stopped_unprompted_t = 0.0
        self.limit_t = self.S[-1] / 2.0 + 90.0
        return self._obs(), {}

    def _next_mark(self):
        for m in self.marks:
            if not m[4] and m[0] > self.s - 1.0:
                return m
        return None

    def _obs(self):
        q = self.P[self.k]
        herr = wrap(self.yaw - q[2])
        curv = [wrap(self._at(self.s + d)[2] - q[2]) for d in AHEAD]
        m = self._next_mark()
        if m is not None:
            mdist = m[0] - self.s
            mk = [1.0, 0.0, 0, 0, 0] if m[1] == "stop" else [0.0, 1.0] + [float(self._signal(*m[2]) == c) for c in range(3)]
            mobs = [min(mdist, 60.0) / 60.0] + mk + [float(m[3])]
        else:
            mobs = [1.0, 0, 0, 0, 0, 0, 0]
        lead = min([c["s"] - self.s - LENGTH for c in self.cars if c["s"] > self.s] or [99.0])
        lv = min([(c["s"], c["v"]) for c in self.cars if c["s"] > self.s] or [(0.0, self.v)])[1]
        ka, kb = int(q[4]), int(q[5])
        o = [self.v / POD_MAX, self.steer / MAX_STEER, self.off / 2.0, herr, q[3] / 4.0,
             float(ka == 0), float(ka == 1), float(ka == 2), float(kb == 0), float(kb == 1), float(kb == 2),
             float(q[7])] + curv + mobs + [min(lead, 60.0) / 60.0, (self.v - lv) / 10.0, q[6] / 60.0,
                                           min(self.S[-1] - self.s, 800.0) / 800.0] + list(self.last)
        return np.clip(np.array(o[:30], np.float32), -10, 10)

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1)
        target = a[0] * MAX_STEER
        self.steer += float(np.clip(target - self.steer, -STEER_RATE * DT, STEER_RATE * DT))
        acc = a[1] * (MAX_ACC if a[1] > 0 else MAX_BRAKE)
        self.v = float(np.clip(self.v + acc * DT, 0.0, POD_MAX))
        self.yaw = wrap(self.yaw + self.v / WHEELBASE * math.tan(self.steer) * DT)
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT
        self.t += DT
        self._cars_step()
        s_old = self.s
        self.s, self.off, q = self._project(self.x, self.y)
        r = 0.0
        if self.s > self.s_best:
            r += 0.3 * (self.s - self.s_best)                       # progress (0.1 let it crawl) / 前進
            self.s_best = self.s
        r -= 0.06                                                   # time step penalty (strictly > max r_vel +0.05 to kill micro-crawls)
        r -= 0.02 * abs(self.off) + 0.01 * float(np.sum(np.square(a - self.last)))
        self.last = a
        limit = q[6] / 3.6
        if self.v > limit + 0.5:
            r -= 0.05 * (self.v - limit)
            self.k_stats["speeding_s"] += DT
        # Road context requirements (stop lines, signals, lead vehicles) / 道路状況の確認
        m = self._next_mark()
        # Must only holds if within legal stopping window (0..4m for stop sign, 0..12m for red/yellow signal)
        must = m is not None and (
            (m[1] == "stop" and 0.0 <= m[0] - self.s <= 4.0 and not m[3]) or
            (m[1] == "signal" and 0.0 <= m[0] - self.s <= 12.0 and self._signal(*m[2]) > 0)
        )
        lead = min([c["s"] - self.s - LENGTH for c in self.cars if c["s"] > self.s] or [99.0])

        # Positive forward velocity reward when road is clear (tied to projected lane tangent velocity) / 前進方向の速度報酬
        target_v = min(limit, POD_MAX)
        dpsi = wrap(self.yaw - q[2])
        proj_v = self.v * math.cos(dpsi)
        if not must and lead > 10.0 and proj_v > 0.1:
            r += 0.05 * min(proj_v / max(target_v, 1.0), 1.0)
            self.clear_speeds.append(self.v)

        # Unprompted standstill handling: penalty & early abort to break brake-lock local minimum
        # 理由のない停止のペナルティとタイムアウト（膠着ローカルミニマムの解消）
        term = False
        if self.v < 0.3:
            if not must and lead > 10.0:
                self.stopped_unprompted_t += DT
                r -= 0.25                                             # substantial penalty (-2.5 / s)
                if self.stopped_unprompted_t > 5.0:
                    self.k_stats["stalls"] = self.k_stats.get("stalls", 0) + 1
                    self.k_stats["end"] = "stalled"
                    r -= 35.0
                    term = True
            else:
                self.stopped_unprompted_t = max(0.0, self.stopped_unprompted_t - DT)
        else:
            self.stopped_unprompted_t = max(0.0, self.stopped_unprompted_t - DT)

        half = q[3] / 2.0
        # which edge the pod's side is past / どちらの縁を越えたか
        over_a = self.off + HALF_W - half
        over_b = -self.off + HALF_W - half
        if not term and not q[7]:
            if (over_a > 0.25 and int(q[4]) == 0) or (over_b > 0.25 and int(q[5]) == 0):
                self.k_stats["kerb"] += 1
                self.k_stats["end"] = "kerb"
                r -= 40.0
                term = True
            elif (over_a > 0.2 and int(q[4]) == 2) or (over_b > 0.2 and int(q[5]) == 2):
                r -= 0.5                                              # over the centre line / 中央線越え
                self.k_stats["centre_s"] += DT
        if abs(self.off) > 6.0 and not term:
            self.k_stats["end"] = "off_route"
            r -= 40.0
            term = True
        # stop lines / 停止線
        for m in self.marks:
            if m[4]:
                continue
            if 0.0 <= m[0] - self.s <= 4.0 and self.v < 0.3:
                m[3] = True
            if s_old < m[0] <= self.s:
                m[4] = True
                if m[1] == "stop" and not m[3]:
                    self.k_stats["stop_violation"] += 1
                    r -= 25.0
                elif m[1] == "signal" and self._signal(*m[2]) == 2:
                    self.k_stats["red_violation"] += 1
                    self.k_stats["end"] = "red_violation"
                    r -= 40.0
                    term = True
        for c in self.cars:
            if abs(c["s"] - self.s) < LENGTH and abs(self.off) < 1.5:
                self.k_stats["collision"] += 1
                self.k_stats["end"] = "collision"
                r -= 50.0
                term = True
                break
        if not term and self.S[-1] - self.s < 3.0:
            r += 30.0
            self.k_stats["reached"] = True
            self.k_stats["end"] = "reached"
            term = True
        if not term and self.t > self.limit_t:
            r -= 40.0                                                   # as bad as a kerb strike / 縁石と同等
            self.k_stats["end"] = "timeout"
            term = True
        if self.log is not None and int(round(self.t / DT)) % 2 == 0:
            self.log.append({"t": round(self.t, 1), "p": [round(self.x, 2), round(self.y, 2), round(self.yaw, 3)],
                             "v": round(self.v, 2), "off": round(self.off, 2),
                             "cars": [round(float(c["s"]), 1) for c in self.cars],
                             "lights": [self._signal(*m[2]) if m[1] == "signal" else -1 for m in self.marks]})
        info = {}
        if term:
            if self.clear_speeds:
                self.k_stats["p10_speed"] = float(np.percentile(self.clear_speeds, 10))
            else:
                self.k_stats["p10_speed"] = float(self.v)
            info = {"kpi": dict(self.k_stats, length=float(self.S[-1]), progress=float(self.s_best),
                                time_s=self.t)}
        return self._obs(), float(r), term, False, info


class ScriptedDriver(object):
    """Baseline only: pure pursuit on the lane centre, speed for bends, stops at
    stop lines and red lights, follows the car ahead. 比較用の単純な運転。"""

    def __init__(self, env):
        self.e = env

    def act(self, obs):
        e = self.e
        tgt = e._at(e.s + max(4.0, e.v * 1.2))
        dx, dy = tgt[0] - e.x, tgt[1] - e.y
        alpha = wrap(math.atan2(dy, dx) - e.yaw)
        steer = math.atan2(2 * WHEELBASE * math.sin(alpha), max(math.hypot(dx, dy), 1.0))
        bend = max(abs(wrap(e._at(e.s + d)[2] - e.P[e.k][2])) for d in (10.0, 20.0))
        vt = min(e.P[e.k][6] / 3.6, POD_MAX, max(2.5, 8.0 - 8.0 * bend))
        m = e._next_mark()
        if m is not None:
            d = m[0] - e.s
            if (m[1] == "stop" and not m[3]) or (m[1] == "signal" and e._signal(*m[2]) > 0):
                vt = min(vt, max(0.0, math.sqrt(max(0.0, 2 * 2.0 * (d - 1.5)))))
        lead = min([c["s"] - e.s - LENGTH for c in e.cars if c["s"] > e.s] or [99.0])
        vt = min(vt, max(0.0, (lead - 3.0) * 0.6))
        acc = np.clip((vt - e.v) * 0.8, -1, 1)
        return np.array([np.clip(steer / MAX_STEER, -1, 1), acc])
