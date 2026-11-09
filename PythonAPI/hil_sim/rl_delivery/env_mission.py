"""Drone mission brain environment: one pod zone, its pod and 4 drones.

The brain makes the operator decisions a delivery company makes for its
drones, one at a time, whenever a drone needs one:
- a drone in the pod with an order: launch at low / medium / high cruise
  height, or wait 30 s (wind, rain, battery, or the pod about to move);
- a drone that dropped its parcel: fly back to which meeting point (the pod's
  current spot or one of its next two stops).

It sees what an operator could know: battery, order distance and age, the
pod's next stops, rain, and the wind measured at the pod (an anemometer near
the ground, noisy). It never sees the wind up in the air or future gusts.

Flights use a quick stand-in for the flight brain (option B, docs plan):
the route goes round the no-fly zones (delivery.zones.Airspace) and stays in
the pod's zone except to pass through; time, battery and crash risk follow
from length, height, the hidden wind along the way (wind.py), rain and the
building heights. When the flight brain is trained it replaces this stand-in.

The pod drives a rule-based round of face-to-face stops in its zone (the pod
brain comes later), and only enters other zones to pass through.

ミッションブレインの環境（1 ゾーン・ポッド 1 台・ドローン 4 機）。飛行は簡易推定。
"""
import math

import gymnasium as gym
import networkx as nx
import numpy as np

from delivery.zones import Airspace
from rl_delivery.env_flight import load_map
from rl_delivery.wind import BuildingGrid, Wind, PROFILE

DT = 5.0                  # sim step between events [s] / イベント間の刻み
EP_S = 3600.0             # one hour per episode / 1 エピソード 1 時間
N_DRONES = 4
CRUISE = 10.0             # m/s airspeed / 対気速度
HEIGHTS = (20.0, 35.0, 50.0)   # low / medium / high cruise above ground / 巡航高度
HOVER_PCT_S = 0.25        # battery %/s (drone_dyn) / 電池消費
POD_SPEED = 8.0           # m/s average on the road graph (calibrated in step 3) / ポッド平均速度
STOP_S = 60.0             # face-to-face stop / 対面停車時間
WAIT_S = 30.0
CRASH_COST = 150.0        # a lost drone + parcel (was 40: it paid to risk 1 crash in 4) / 墜落のコスト
SWAP_S = 60.0             # battery swap in the pod (LMP-4 crane) / ポッド内の電池交換
SWAP_BELOW = 60.0         # swap when lower than this / この値未満なら交換
REPAIR_S = 900.0          # a crashed drone is replaced after this / 墜落機の交換まで
RAIN_GROUND = 20.0        # precipitation at which flying is dangerous / 危険な雨量

# actions / 行動
LAUNCH_LOW, LAUNCH_MID, LAUNCH_HIGH, WAIT, RET_HERE, RET_NEXT, RET_NEXT2 = range(7)
# drone states / ドローンの状態
IN_POD, OUT, RETURNING, DOWN = 0, 1, 2, 3


class MissionEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, map_name="OitaKosen", pod=None, seed=None, order_gap=(60.0, 180.0)):
        self.m = load_map(map_name)
        self.rng = np.random.default_rng(seed)
        self.pod_choice = pod
        self.order_gap = order_gap
        self.grid = BuildingGrid(self.m["buildings"], self.m["bounds"])
        self.air = Airspace(self.m["zones"]["nofly"])
        g = nx.DiGraph()
        self.edge_line = {}
        for u, v, L, kmh, li, _j in self.m["edges"]:
            if not g.has_edge(u, v) or L < g[u][v]["w"]:
                g.add_edge(u, v, w=L)
                self.edge_line[(u, v)] = li
        self.g = g
        self.xy = np.array([n[:2] for n in self.m["nodes"]])
        self._sp = {}
        self.observation_space = gym.spaces.Box(-10.0, 10.0, (24,), np.float32)
        self.action_space = gym.spaces.Discrete(7)
        self.log = None

    # -- helpers / 補助
    def _route_len(self, a, b):
        if (a, b) not in self._sp:
            try:
                self._sp[(a, b)] = nx.shortest_path_length(self.g, a, b, weight="w")
            except nx.NetworkXNoPath:
                self._sp[(a, b)] = 1e6
        return self._sp[(a, b)]

    def _inside(self, p, r):
        return r["x0"] <= p[0] <= r["x1"] and r["y0"] <= p[1] <= r["y1"]

    def _air_path(self, a, b):
        pts, here = [(a[0], a[1])], (a[0], a[1])
        for p in self.air.path(a, b):
            pts.append(p)
        return pts

    def _wind_measured(self):
        """Anemometer at the pod: 10 m mean + turbulence, noisy. / 地上の風速計"""
        w = self.wind.at(self.pod_pos[0], self.pod_pos[1], 10.0)
        return float(np.hypot(w[0], w[1]) + self.rng.normal(0, 0.4))

    # -- the stand-in flight / 簡易飛行
    def _fly(self, a, b, height, payload):
        """Fly a -> b at a cruise height: (seconds, battery %, crash reason or
        None, path pts). Wind is sampled along the way from the hidden model.
        / a→b の飛行時間・電池・墜落理由"""
        pts = self._air_path(a, b)
        t = height / 3.0 * 2.0                        # climb + descent at 3 m/s / 上昇と降下
        batt = HOVER_PCT_S * t * 1.15
        crash = None
        peak = 0.0
        wsim = self.wind
        for p, q in zip(pts[:-1], pts[1:]):
            seg = math.dist(p, q)
            if seg < 1e-3:
                continue
            ux, uy = (q[0] - p[0]) / seg, (q[1] - p[1]) / seg
            n = max(1, int(seg / 50.0))
            for k in range(n):
                x, y = p[0] + ux * seg * (k + 0.5) / n, p[1] + uy * seg * (k + 0.5) / n
                w = wsim.at(x, y, height)
                head = w[0] * ux + w[1] * uy
                cross = -w[0] * uy + w[1] * ux
                gs = math.sqrt(max(CRUISE ** 2 - cross ** 2, 4.0)) + head   # ground speed / 対地速度
                gs = max(gs, 1.5)
                dt = seg / n / gs
                t += dt
                ws = math.hypot(w[0], w[1])
                peak = max(peak, ws)
                batt += HOVER_PCT_S * (1.0 + 0.2 * payload + 0.5 * (ws / 10.0) ** 2) * dt
                # buildings: a tall one under a low cruise / 低高度での建物
                if self.grid.top(x, y) > height - 5.0 and crash is None and self.rng.random() < 0.15:
                    crash = "collision"
        # wind and rain risk / 風と雨のリスク
        p_wind = 1.0 / (1.0 + math.exp(-(peak - 15.5) / 0.8)) * 0.6
        p_rain = 0.25 if self.rain >= RAIN_GROUND else 0.0
        if crash is None and self.rng.random() < p_wind + p_rain + 0.001:
            crash = "rain" if p_rain and self.rng.random() < p_rain / (p_wind + p_rain + 1e-9) else "wind"
        return t, batt, crash, pts, peak

    # -- pod round / ポッドの巡回
    def _new_stop(self):
        z = self.zone
        cand = [i for i in self.zone_nodes if i != self.pod_node]
        return int(self.rng.choice(cand))

    def _advance_pod(self, dt):
        self.pod_t_left -= dt
        if self.pod_t_left > 0:
            if self.pod_moving and self.pod_leg_len > 0:
                f = 1.0 - max(0.0, self.pod_t_left) / self.pod_leg_t
                self.pod_pos = self._along(f)         # along the road route / 道路に沿って
            return
        if self.pod_moving:                           # arrived at the stop / 停車地に到着
            self.pod_node = self.stops.pop(0)
            self.pod_pos = self.xy[self.pod_node].copy()
            self.stops.append(self._new_stop())
            self.pod_moving = False
            self.pod_t_left = STOP_S
        else:                                         # leave for the next stop / 次へ出発
            self.pod_from = self.pod_node
            L = self._route_len(self.pod_node, self.stops[0])
            self.pod_leg_len = L
            self._leg_poly(self.pod_node, self.stops[0])
            self.pod_leg_t = self.pod_t_left = max(10.0, L / POD_SPEED)
            self.pod_moving = True

    def _leg_poly(self, a, b):
        """The road polyline of the pod's leg (lane shapes from the map). / 区間の道路形状"""
        try:
            nodes = nx.shortest_path(self.g, a, b, weight="w")
        except nx.NetworkXNoPath:
            nodes = [a, b]
        pts = [self.xy[nodes[0]]]
        for u, v in zip(nodes[:-1], nodes[1:]):
            li = self.edge_line.get((u, v))
            line = self.m["lines"][li] if li is not None else [self.xy[u], self.xy[v]]
            pts += [np.asarray(q, float) for q in line[1:]]
        P = np.array(pts)
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
        self.leg = (P, np.concatenate([[0.0], np.cumsum(seg)]))

    def _along(self, f):
        P, cum = self.leg
        d = f * cum[-1]
        i = int(np.clip(np.searchsorted(cum, d) - 1, 0, len(P) - 2))
        u = (d - cum[i]) / max(cum[i + 1] - cum[i], 1e-6)
        return P[i] + (P[i + 1] - P[i]) * u

    def _pod_eta(self, k):
        """Seconds until the pod is at its k-th next stop (0 = the next). / 到着までの秒数"""
        t = self.pod_t_left if self.pod_moving else self.pod_t_left + self._route_len(self.pod_node, self.stops[0]) / POD_SPEED
        here = self.stops[0]
        for j in range(1, k + 1):
            t += STOP_S + self._route_len(here, self.stops[j]) / POD_SPEED
            here = self.stops[j]
        return t

    # -- gym
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        pods = self.m["zones"]["pods"]
        vid = self.pod_choice or self.rng.choice(sorted(pods))
        self.vid = str(vid)
        self.zone = pods[self.vid]
        keep = set(self.m["keep"])
        self.zone_nodes = [i for i in keep if self._inside(self.xy[i], self.zone)
                           and not self.air.blocked(self.xy[i], 0.0)]
        self.addresses = [b for b in self.m["buildings"] if b["deliverable"] and self._inside((b["x"], b["y"]), self.zone)
                          and not self.air.blocked((b["x"], b["y"]), 10.0)]
        self.wind = Wind(self.rng, level=1.0, grid=self.grid)
        self.rain, self.rain_target = 0.0, 0.0
        self.t = 0.0
        self.pod_node = int(self.rng.choice(self.zone_nodes))
        self.pod_pos = self.xy[self.pod_node].copy()
        self.stops = [self._new_stop() for _ in range(3)]
        self.pod_moving, self.pod_t_left, self.pod_leg_len, self.pod_leg_t, self.pod_from = False, STOP_S, 0.0, 1.0, self.pod_node
        self.drones = [{"state": IN_POD, "batt": 100.0, "order": None, "t_back": 0.0, "pos": self.pod_pos.copy(),
                        "meet": None, "wait": 0.0, "id": k} for k in range(N_DRONES)]
        self.orders = []
        self.next_order = self.rng.uniform(0, 20)
        self.oid = 0
        self.wind_hist = []
        self.k = {"delivered": 0, "lead": [], "crash": 0, "crash_why": {}, "trips": 0, "landing": [],
                  "hold_s": 0.0, "meet_wait": [], "chases": 0, "nofly": 0}
        self.pending = None
        self._landings_seen = 0
        return self._advance(), {}

    def _spawn_orders(self):
        while self.t >= self.next_order:
            b = self.addresses[int(self.rng.integers(len(self.addresses)))]
            self.orders.append({"id": self.oid, "x": b["x"], "y": b["y"], "t0": self.t})
            self.oid += 1
            self.next_order += self.rng.uniform(*self.order_gap)

    def _weather(self, dt):
        self.wind.step(dt, 30.0)
        if self.rng.random() < dt / 1800.0:
            self.rain_target = self.rng.choice([0.0, 0.0, 10.0, 30.0, 60.0])
        self.rain += (self.rain_target - self.rain) * min(1.0, dt / 120.0)
        self.wind_hist = (self.wind_hist + [self._wind_measured()])[-12:]

    def _need(self):
        """The next drone that needs a decision, and which kind. / 判断待ちの機体"""
        for d in self.drones:
            if d["state"] == IN_POD and not self.pod_moving and d["wait"] <= 0 and d["batt"] > 25.0:
                if d["order"] is None and self.orders:
                    d["order"] = self.orders.pop(0)
                if d["order"] is not None:
                    return d, "launch"
            if d["state"] == OUT and d["t_back"] <= 0:
                return d, "return"
        return None, None

    def _advance(self):
        """Run the world until a decision is needed (or the episode ends)."""
        while self.t < EP_S:
            d, kind = self._need()
            if d is not None:
                self.pending = (d, kind)
                return self._obs()
            self.t += DT
            self._weather(DT)
            self._advance_pod(DT)
            self._spawn_orders()
            if any(dd["state"] == IN_POD and dd["order"] is not None for dd in self.drones) and not self.pod_moving:
                pass
            for dd in self.drones:
                if dd["state"] == IN_POD:
                    dd["wait"] -= DT
                    if dd.get("batt_after") and dd["wait"] <= 0:
                        dd["batt"], dd["batt_after"] = dd.pop("batt_after"), None
                    dd["pos"] = self.pod_pos.copy()
                    if dd["batt"] < SWAP_BELOW:                         # battery swap / 電池交換
                        dd["swap"] = dd.get("swap", 0.0) + DT
                        if dd["swap"] >= SWAP_S:
                            dd["batt"], dd["swap"] = 100.0, 0.0
                        dd["wait"] = max(dd["wait"], DT)
                    if dd["order"] is not None and dd["wait"] > 0:
                        self.k["hold_s"] += DT
                elif dd["state"] == OUT:
                    dd["t_back"] -= DT
                elif dd["state"] == RETURNING:
                    dd["t_back"] -= DT
                    if dd["t_back"] <= 0:
                        # holds at the meeting point until the pod is there / 合流点で待機
                        at = np.linalg.norm(self.pod_pos - dd["meet_xy"]) < 5.0 and not self.pod_moving
                        if at:
                            self.k["meet_wait"].append(-dd["t_back"])
                            dd["state"], dd["pos"] = IN_POD, self.pod_pos.copy()
                            self.k["landing"].append(dd["batt"])
                        elif not self._pod_will_be_at(dd["meet_xy"]):
                            # the pod has left: fly on to its next stop / ポッドが去った：次の停車地へ
                            tgt = self.xy[self.stops[0]]
                            t, b, crash, _pts, _pk = self._fly(dd["pos"], tgt, dd.get("height", 35.0), 0.0)
                            dd["batt"] -= b
                            self.k["chases"] += 1
                            if crash or dd["batt"] <= 0:
                                self._crash(dd, crash or "battery_chase")
                            else:
                                dd["leg"] = (self.t, t, np.array(dd["pos"], float), np.array(tgt, float))
                                dd["meet_xy"], dd["pos"], dd["t_back"] = tgt, tgt, t
                        else:
                            dd["batt"] -= HOVER_PCT_S * DT
                            if dd["batt"] <= 0:
                                self._crash(dd, "battery_hover")
                elif dd["state"] == DOWN:
                    dd["t_back"] -= DT
                    if dd["t_back"] <= 0:                               # replacement drone / 代替機
                        dd.update(state=IN_POD, batt=100.0, pos=self.pod_pos.copy(), wait=0.0)
            self._log()
        self.pending = None
        return self._obs()

    def _pod_will_be_at(self, p):
        """Is p the pod's current stop or one of its next stops? / ポッドが来る地点か"""
        cands = [self.pod_node] if not self.pod_moving else []
        cands += self.stops
        return any(np.linalg.norm(self.xy[n] - p) < 5.0 for n in cands)

    def _crash(self, d, why):
        d["state"], d["t_back"] = DOWN, REPAIR_S
        self.k["crash"] += 1
        self.k["crash_why"][why] = self.k["crash_why"].get(why, 0) + 1
        if d["order"] is not None:
            self.orders.insert(0, d["order"])           # the parcel goes back in the queue / 注文は戻す
            d["order"] = None
        self.down_penalty = True
        self._pending_cost = getattr(self, "_pending_cost", 0.0) + CRASH_COST

    def _meet(self, which):
        """Meeting point for a return choice: the pod's spot now / next / next+1."""
        if which == RET_HERE:
            return self.pod_node, 0.0 if not self.pod_moving else self._pod_eta(0)
        k = 0 if which == RET_NEXT else 1
        return self.stops[k], self._pod_eta(k)

    def action_masks(self):
        m = np.zeros(7, bool)
        if self.pending is None:
            m[WAIT] = True
            return m
        d, kind = self.pending
        if kind == "launch":
            m[[LAUNCH_LOW, LAUNCH_MID, LAUNCH_HIGH, WAIT]] = True
        else:
            m[[RET_NEXT, RET_NEXT2]] = True
            m[RET_HERE] = not self.pod_moving
        return m

    def step(self, action):
        r = 0.0
        self.down_penalty = False
        if self.pending is not None:
            d, kind = self.pending
            a = int(action)
            if kind == "launch" and a in (LAUNCH_LOW, LAUNCH_MID, LAUNCH_HIGH):
                o = d["order"]
                h = HEIGHTS[a]
                t, b, crash, pts, peak = self._fly(self.pod_pos, (o["x"], o["y"]), h, 1.0)
                self.k["trips"] += 1
                d["batt"] -= b + 3.0                 # + the drop itself / 投下分
                d["route"] = pts
                if crash or d["batt"] <= 0:
                    self._crash(d, crash or "battery_out")
                else:
                    d["state"], d["t_back"], d["height"] = OUT, t + 20.0, h
                    d["leg"] = (self.t, t, np.array(self.pod_pos, float), np.array([o["x"], o["y"]], float))
                    d["pos"] = np.array([o["x"], o["y"]])
                    lead = self.t + t + 20.0 - o["t0"]
                    self.k["delivered"] += 1
                    self.k["lead"].append(lead)
                    r += 10.0 - 0.004 * lead
                    d["order"] = None
            elif kind == "launch":
                d["wait"] = WAIT_S
                if d["batt"] < 90.0:                 # waiting also swaps a used battery / 待機中に電池交換
                    d["wait"] = max(WAIT_S, SWAP_S)
                    d["batt_after"] = 100.0
                r -= 0.05
            else:                                     # return choice / 帰還先
                node, eta = self._meet(a if a in (RET_HERE, RET_NEXT, RET_NEXT2) else RET_NEXT)
                tgt = self.xy[node]
                t, b, crash, pts, peak = self._fly(d["pos"], tgt, d.get("height", 35.0), 0.0)
                d["batt"] -= b
                if crash or d["batt"] <= 0:
                    self._crash(d, crash or "battery_return")
                else:
                    d["state"], d["meet_xy"] = RETURNING, tgt
                    d["leg"] = (self.t, t, np.array(d["pos"], float), np.array(tgt, float))
                    d["t_back"] = t
                    d["pos"] = tgt
                    early = max(0.0, eta - t)            # time it will hover waiting / 待機時間
                    late = max(0.0, t - eta - STOP_S)    # the pod may have left / ポッドが去る
                    r -= 0.01 * early + 0.02 * late
                    if d["batt"] - HOVER_PCT_S * early < 15.0:
                        r -= 5.0
        obs = self._advance()
        r -= getattr(self, "_pending_cost", 0.0)
        self._pending_cost = 0.0
        # queue pressure + low-battery landings / 待ち注文と低電池着陸
        r -= 0.002 * len(self.orders)
        # landing low: growing cost below 20 % (counted once per landing) / 低電池着陸のコスト
        while self._landings_seen < len(self.k["landing"]):
            lv = self.k["landing"][self._landings_seen]
            self._landings_seen += 1
            if lv < 20.0:
                r -= 2.0 * (20.0 - lv)
        term = self.t >= EP_S
        info = {}
        if term:
            info = {"kpi": self.kpi()}
        return obs, float(r), term, False, info

    def _obs(self):
        if self.pending is None:
            return np.zeros(24, np.float32)
        d, kind = self.pending
        ws = self.wind_hist or [0.0]
        o = d["order"] if kind == "launch" else None
        if o is not None:
            dist = math.dist(self.pod_pos, (o["x"], o["y"]))
            vec = ((o["x"] - self.pod_pos[0]) / 1000.0, (o["y"] - self.pod_pos[1]) / 1000.0)
            age = (self.t - o["t0"]) / 600.0
        else:
            dist, vec, age = 0.0, (0.0, 0.0), 0.0
        here = (o["x"], o["y"]) if o is not None else d["pos"]     # from the drop / 投下点から
        meets = []
        for k in range(2):
            p = self.xy[self.stops[k]]
            meets += [math.dist(here, p) / 1000.0, self._pod_eta(k) / 600.0]
        busy = sum(dd["state"] in (OUT, RETURNING) for dd in self.drones) / N_DRONES
        down = sum(dd["state"] == DOWN for dd in self.drones) / N_DRONES
        v = [1.0 if kind == "launch" else 0.0, d["batt"] / 100.0, dist / 1000.0, vec[0], vec[1], age,
             float(np.mean(ws)) / 10.0, float(np.std(ws)) / 3.0, float(np.max(ws)) / 10.0, self.rain / 50.0,
             len(self.orders) / 10.0, busy, down, float(self.pod_moving), self.pod_t_left / 300.0,
             math.dist(here, self.pod_pos) / 1000.0] + meets + [self.t / EP_S,
             float(np.mean([dd["batt"] for dd in self.drones])) / 100.0, 0.0, 0.0]
        return np.clip(np.array(v[:24], np.float32), -10, 10)

    def _draw_pos(self, dd):
        """Where to draw a drone: along its flight leg while flying. / 描画位置"""
        p = dd["pos"]
        if dd["state"] == IN_POD:
            p = self.pod_pos
        elif dd.get("leg") is not None:
            t0, dur, a, b = dd["leg"]
            f = min(1.0, max(0.0, (self.t - t0) / max(dur, 1e-3)))
            p = a + (b - a) * f
        return round(float(p[0]), 1), round(float(p[1]), 1)

    def _log(self):
        if self.log is None:
            return
        self.log.append({"t": self.t, "pod": [round(float(self.pod_pos[0]), 1), round(float(self.pod_pos[1]), 1)],
                         "moving": self.pod_moving,
                         "drones": [list(self._draw_pos(dd)) + [dd["state"], round(dd["batt"], 1)]
                                    for dd in self.drones],
                         "orders": [[round(o["x"], 1), round(o["y"], 1)] for o in self.orders],
                         "wind": round(float(np.mean(self.wind_hist or [0])), 1), "rain": round(self.rain, 1)})

    def kpi(self):
        k = self.k
        lead = k["lead"]
        return {"deliveries": k["delivered"], "lead_mean_s": float(np.mean(lead)) if lead else None,
                "lead_p95_s": float(np.percentile(lead, 95)) if lead else None,
                "crashes": k["crash"], "crash_why": dict(k["crash_why"]),
                "crashes_per_100": 100.0 * k["crash"] / k["trips"] if k["trips"] else 0.0,
                "landing_min": float(min(k["landing"])) if k["landing"] else None,
                "landing_mean": float(np.mean(k["landing"])) if k["landing"] else None,
                "hold_s": k["hold_s"], "meet_wait_mean_s": float(np.mean(k["meet_wait"])) if k["meet_wait"] else None,
                "queue_end": len(self.orders), "chases": k["chases"], "nofly_breaches": 0, "drops_outside_zone": 0}


class RuleMission(object):
    """Today's rules (delivery_sim): launch at once at medium height unless
    the wind measured at the pod is over 10 m/s or it rains >= 20; return to
    the pod's next stop. The score to beat. / 現行ルール（比較対象）"""

    def __init__(self, env):
        self.env = env

    def act(self, obs):
        d, kind = self.env.pending if self.env.pending else (None, None)
        if kind == "launch":
            if obs[6] * 10.0 > 10.0 or self.env.rain >= RAIN_GROUND:
                return WAIT
            # enough battery for out + back + 3 min hover + 15 % reserve? / 往復に足りるか
            o = d["order"]
            env = self.env
            dist = math.dist(env.pod_pos, (o["x"], o["y"]))
            back = min(math.dist((o["x"], o["y"]), env.xy[n]) for n in env.stops[:2])
            w = max(obs[6] * 10.0, 0.0) * PROFILE              # mean wind at the pod, scaled to 30 m / 高度換算
            k = 1.2 + 0.5 * (w / 10.0) ** 2                    # power factor in wind / 風での消費係数
            slow = CRUISE / max(CRUISE - 0.7 * w, 2.0)         # headwind on one leg / 向かい風
            need = HOVER_PCT_S * k * ((dist + back) / CRUISE * slow + 2 * 23.0 + 120.0) * 1.2 + 3.0 + 15.0
            if d["batt"] < need:
                return WAIT
            return LAUNCH_MID
        # the next stop if the battery reaches it, else the nearest choice / 届くなら次の停車地
        env, m = self.env, self.env.action_masks()
        opts = [a for a in (RET_NEXT, RET_HERE, RET_NEXT2) if m[a]]
        dist = {a: math.dist(d["pos"], env.xy[env._meet(a)[0]]) for a in opts}
        if d["batt"] - HOVER_PCT_S * (dist[RET_NEXT] / CRUISE * 1.4 + 30.0) - 10.0 > 0:
            return RET_NEXT
        return min(opts, key=lambda a: dist[a])
