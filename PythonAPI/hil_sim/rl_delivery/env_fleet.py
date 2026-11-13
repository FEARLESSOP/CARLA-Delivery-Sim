"""The fleet world: 3 pods (one per zone), 4 drones each, standby drones at the
charging stations, parcel lockers, EV batteries, crashes, recovery.

The user's delivery flow is the world's rules; every decision in it belongs to
a brain (pure RL, nothing scripted):
- pod brain (one per pod, shared): where to stop for a drone round (it has to
  find the middle of the drone deliveries, under open sky), how many drones to
  launch and which meeting point to give them, which locker to drive to for a
  face-to-face (F2F) drop, when to go to the meeting point, when to call a
  standby drone in for a tired one, when to go and charge, or wait;
- drone brain (one per drone, shared, standby drones too): cruise height,
  meeting point or home to the charging station after a drop, keep hovering or
  go home while waiting for the pod, accept or decline a standby task.

World rules (what happens, not what to choose):
- a pod carries its zone's parcels; drones launch only while it stands still;
  launching under a tree canopy can crash the drone at take-off;
- an F2F drop takes 60 s at a locker; drones land in the pod only at the pod
  (hovering at the meeting point until it is there);
- drones charge in the pod from the pod's battery (0.08 %/s) or fast on a
  station deck pad (1.5 %/s); the pod charges at a station's pod bay (50 kW);
- driving, idling and charging drones drain the pod (10 kWh); a flat pod is
  stranded, a flat drone falls;
- a standby swap: a standby drone flies from its station to the pod, the tired
  drone flies to the station and becomes a standby;
- a crash before the drop leaves the parcel at the crash site: a standby drone
  flies there, picks it up, delivers it and joins the pod; a crash after the
  drop is replaced by a standby; a human collects the wreck after 30 min and
  it is a standby again after a repair (2 h);
- pods only enter other zones to pass through to their own or to a charger.
Flights use a quick stand-in (route round no-fly zones, time / battery / crash
risk from length, height, hidden wind, rain and the height map) until the
navigation brain flies them.

Two gym views of one world: PodView (the pod brain acts; drones follow
``drone_policy``) and DroneView (the drone brain acts; pods follow
``pod_policy``), so the two brains train in turns against each other.

フリート世界（ポッド 3・ドローン 4×3・待機機・ロッカー・電池・墜落と回収）。
"""
import json
import math
import os

import gymnasium as gym
import networkx as nx
import numpy as np

from delivery.zones import Airspace
from rl_delivery.env_flight import load_map
from rl_delivery.env_nav import Heights
from rl_delivery.wind import BuildingGrid, PROFILE, Wind

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 5.0
EP_S = 3 * 3600.0
N_PER_POD = 4
HEIGHTS = (20.0, 35.0, 50.0)
STOP_S = 60.0             # F2F drop at a locker / ロッカーでの投函
LAND_S = 15.0             # a drone docking into the pod / 収容
PICK_S = 60.0             # picking a parcel up at a crash site / 回収
WRECK_S = 1800.0          # a human collects a wreck after this / 回収まで
REPAIR_S = 7200.0         # then it is a standby again / 修理後に待機機
WAIT_S = 30.0
RAIN_GROUND = 20.0
HOVER_PCT_S = 0.25
CRASH_COST = 150.0
K_SPOT, K_LOCK, K_MEET, K_STN = 6, 4, 3, 3

# drone states / ドローンの状態
DOCKED, FLY, HOVER, STANDBY, DOWN, REPAIR = "docked", "fly", "hover", "standby", "down", "repair"
# pod action layout / ポッドの行動
A_SPOT = 0                                   # 0..5 drive to a dispatch spot
A_LAUNCH = A_SPOT + K_SPOT                   # 6..17 launch n=1..4 drones x meeting point 0..2
A_LOCK = A_LAUNCH + N_PER_POD * K_MEET       # 18..21 drive to a locker
A_GOMEET = A_LOCK + K_LOCK                   # 22 drive to the meeting point
A_CHARGE = A_GOMEET + 1                      # 23..25 drive to a station and charge
A_SWAP = A_CHARGE + K_STN                    # 26 call a standby for the most tired docked drone
A_WAIT = A_SWAP + 1                          # 27
N_POD_ACT = A_WAIT + 1
# drone decisions (one Discrete(3) space, masked per kind) / ドローンの判断
D_LAUNCH, D_AFTER, D_HOVER, D_TASK = 0, 1, 2, 3
POD_OBS, DRONE_OBS = 96, 16


class Fleet(object):
    """The world itself (no gym API). / 世界そのもの"""

    def __init__(self, map_name="OitaKosen", seed=None, order_gap=(90.0, 210.0), drone_prob=0.6):
        self.m = load_map(map_name)
        with open(os.path.join(HERE, "maps", map_name + "_sites.json")) as f:
            self.sites = json.load(f)
        self.P = self.sites["params"]
        self.H = Heights(map_name)
        self.grid = BuildingGrid(self.m["buildings"], self.m["bounds"])
        self.air = Airspace(self.m["zones"]["nofly"])
        self.zones = {int(k): r for k, r in self.m["zones"]["pods"].items()}
        g = nx.DiGraph()
        for u, v, L, _kmh, _li, _j in self.m["edges"]:
            if not g.has_edge(u, v) or L < g[u][v]["w"]:
                g.add_edge(u, v, w=L)
        self.g = g
        self.xy = np.array([n[:2] for n in self.m["nodes"]])
        keep = np.array(self.m["keep"])
        self.keep = keep
        self._sp = {}
        self.order_gap, self.drone_prob = order_gap, drone_prob
        # orders per hour for the whole fleet: random per episode in this range
        # (domain randomisation: quiet to overloaded), or fixed for tests (rate_fixed)
        # 注文数/時（フリート全体）：エピソードごとに無作為、テストでは固定
        self.rate_range = (30.0, 90.0)
        self.rate_fixed = None
        self.rate = 60.0
        self.rng = np.random.default_rng(seed)
        # the stations' pod bays and lockers snapped to the road graph / 道路網へ
        self.stations = []
        for s in self.sites["stations"]:
            self.stations.append({"id": s["id"], "xy": np.array(s["pod_bay"]), "node": self._near(s["pod_bay"]),
                                  "pads": [np.array(p) for p in s["deck"]], "bay": None})
        self.lockers = [{"xy": np.array(p), "node": self._near(p), "zone": self._zone(p)}
                        for p in self.sites["lockers"]]
        self.addr = {z: [b for b in self.m["buildings"] if b["deliverable"]
                         and self._inside((b["x"], b["y"]), r) and not self.air.blocked((b["x"], b["y"]), 10.0)]
                     for z, r in self.zones.items()}
        self.zone_nodes = {z: [int(i) for i in keep if self._inside(self.xy[i], r)
                               and not self.air.blocked(self.xy[i], 0.0)] for z, r in self.zones.items()}
        self.canopy = {}
        self.log = None

    # -- helpers / 補助
    def _inside(self, p, r):
        return r["x0"] <= p[0] <= r["x1"] and r["y0"] <= p[1] <= r["y1"]

    def _zone(self, p):
        for z, r in self.zones.items():
            if self._inside(p, r):
                return z
        return None

    def _near(self, p, nodes=None):
        idx = self.keep if nodes is None else np.array(nodes)
        d = (self.xy[idx, 0] - p[0]) ** 2 + (self.xy[idx, 1] - p[1]) ** 2
        return int(idx[int(np.argmin(d))])

    def route(self, a, b):
        """Road distance a -> b [m]; one Dijkstra per start, cached. / 道路距離"""
        if a == b:
            return 0.0
        row = self._sp.get(a)
        if row is None:
            row = self._sp[a] = nx.single_source_dijkstra_path_length(self.g, a, weight="w")
        return row.get(b, 1e5)

    def open_sky(self, node):
        """Fraction of the 5 m round a road spot with nothing above 2 m (1 = clear).
        / 道路地点の上空の開け具合"""
        if node not in self.canopy:
            p = self.xy[node]
            xs = np.arange(-4.0, 4.1, 2.0)
            X, Y = np.meshgrid(p[0] + xs, p[1] + xs)
            self.canopy[node] = float((self.H.top_at(X, Y) - self.H.ground_at(X, Y) < 2.0).mean())
        return self.canopy[node]

    def pod_speed(self):
        return self.P["pod_speed_kmh"] / 3.6

    # -- the stand-in flight / 簡易飛行
    def fly(self, a, b, height, payload):
        """(seconds, battery %, crash or None). / 飛行時間・電池・墜落"""
        pts = [(a[0], a[1])] + list(self.air.path(a, b))
        t = height / 3.0 * 2.0
        batt = HOVER_PCT_S * t * 1.15
        crash, peak = None, 0.0
        for p, q in zip(pts[:-1], pts[1:]):
            seg = math.dist(p, q)
            if seg < 1e-3:
                continue
            ux, uy = (q[0] - p[0]) / seg, (q[1] - p[1]) / seg
            n = max(1, int(seg / 60.0))
            for k in range(n):
                x, y = p[0] + ux * seg * (k + 0.5) / n, p[1] + uy * seg * (k + 0.5) / n
                w = self.wind.at(x, y, height)
                head = w[0] * ux + w[1] * uy
                cross = -w[0] * uy + w[1] * ux
                gs = max(math.sqrt(max(self.P["drone_speed"] ** 2 - cross ** 2, 4.0)) + head, 1.5)
                dt = seg / n / gs
                t += dt
                ws = math.hypot(w[0], w[1])
                peak = max(peak, ws)
                batt += HOVER_PCT_S * (1.0 + 0.2 * payload + 0.5 * (ws / 10.0) ** 2) * dt
                if crash is None and float(self.H.top_at(x, y) - self.H.ground_at(x, y)) > height - 4.0 \
                        and self.rng.random() < 0.15:
                    crash = "collision"
        p_wind = 1.0 / (1.0 + math.exp(-(peak - 15.5) / 0.8)) * 0.6
        p_rain = 0.25 if self.rain >= RAIN_GROUND else 0.0
        if crash is None and self.rng.random() < p_wind + p_rain + 0.001:
            crash = "rain" if p_rain and self.rng.random() < p_rain / (p_wind + p_rain + 1e-9) else "wind"
        return t, batt, crash

    # -- episode / エピソード
    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.rate = float(self.rate_fixed if self.rate_fixed else self.rng.uniform(*self.rate_range))
        mean_gap = 3600.0 * len(self.zones) / self.rate        # per zone / ゾーンあたり
        self.order_gap = (0.5 * mean_gap, 1.5 * mean_gap)
        self.t = 0.0
        self.wind = Wind(self.rng, level=1.0, grid=self.grid)
        self.rain = self.rain_target = 0.0
        self.wind_hist = []
        self.oid = 0
        self.orders = {}
        self.next_order = {z: self.rng.uniform(0, 30) for z in self.zones}
        self.pods, self.drones = [], []
        for z in sorted(self.zones):
            n = int(self.rng.choice(self.zone_nodes[z]))
            pod = {"id": len(self.pods), "zone": z, "node": n, "pos": self.xy[n].copy(),
                   "batt": float(self.rng.uniform(70, 100)), "drive": None, "dwell": 0.0, "charging": None,
                   "meet": None, "meet_node": None, "spot_cands": [], "lock_cands": [], "meet_cands": [],
                   "busy_until": 0.0, "stranded": False, "km": 0.0, "kwh": 0.0}
            self.pods.append(pod)
            for _k in range(N_PER_POD):
                self._new_drone(pod=pod)
        n_sb = int(self.P["standby_drones"])
        for k in range(n_sb):
            st = self.stations[k % len(self.stations)]
            self._new_drone(station=st)
        self.decisions = []
        self.requests = []
        self.events = []
        self.k = {"delivered_f2f": 0, "delivered_drone": 0, "lead": [], "crash": 0, "crash_why": {},
                  "flights": 0, "landing": [], "hover_s": 0.0, "swaps": 0, "recoveries": 0,
                  "replacements": 0, "stranded": 0, "pod_kwh": 0.0, "pod_km": 0.0, "charges": 0,
                  "parcels_lost": 0, "takeoff_crash": 0, "station_visits": 0}
        self.reward = 0.0
        return self

    def _new_drone(self, pod=None, station=None):
        d = {"id": len(self.drones), "pod": pod["id"] if pod else None, "state": DOCKED if pod else STANDBY,
             "batt": 100.0 if pod else float(self.rng.uniform(80, 100)), "parcel": None, "task": None,
             "eta": 0.0, "pos": (pod["pos"] if pod else station["pads"][0]).copy(), "leg": None,
             "station": None if pod else station["id"], "height": 35.0, "hover_t": 0.0, "timer": 0.0,
             "asked": False}
        self.drones.append(d)
        return d

    # -- orders / 注文
    def _spawn_orders(self):
        for z in self.zones:
            while self.t >= self.next_order[z]:
                kind = "drone" if self.rng.random() < self.drone_prob else "f2f"
                b = self.addr[z][int(self.rng.integers(len(self.addr[z])))]
                o = {"id": self.oid, "zone": z, "kind": kind, "xy": np.array([b["x"], b["y"]]), "t0": self.t,
                     "state": "pod", "pod": z - 1, "locker": None, "site": None}
                if kind == "f2f":
                    lk = [i for i, L in enumerate(self.lockers) if L["zone"] == z]
                    if lk:
                        o["locker"] = min(lk, key=lambda i: math.dist(self.lockers[i]["xy"], o["xy"]))
                    else:
                        o["kind"] = "drone"
                self.orders[o["id"]] = o
                self.oid += 1
                self.next_order[z] += self.rng.uniform(*self.order_gap)          # gap per zone / ゾーンごとの間隔

    def pod_orders(self, pod, kind):
        return [o for o in self.orders.values() if o["pod"] == pod["id"] and o["state"] == "pod" and o["kind"] == kind]

    # -- weather / 天候
    def _weather(self):
        self.wind.step(DT, 30.0)
        if self.rng.random() < DT / 2400.0:
            self.rain_target = float(self.rng.choice([0.0, 0.0, 0.0, 10.0, 30.0, 60.0]))
        self.rain += (self.rain_target - self.rain) * min(1.0, DT / 120.0)
        p = self.pods[0]["pos"]
        w = self.wind.at(p[0], p[1], 10.0)
        self.wind_hist = (self.wind_hist + [float(np.hypot(w[0], w[1]) + self.rng.normal(0, 0.4))])[-12:]

    # -- pods / ポッド
    def _drive(self, pod, node, why, arg=None):
        L = self.route(pod["node"], node)
        dur = max(5.0, L / self.pod_speed())
        pod["drive"] = {"to": node, "t_left": dur, "dur": dur, "from": pod["pos"].copy(), "len": L,
                        "why": why, "arg": arg}

    def _pod_step(self, pod):
        if pod["stranded"]:
            return
        P = self.P
        if pod["drive"] is not None:
            dv = pod["drive"]
            step = min(DT, dv["t_left"])
            dv["t_left"] -= DT
            km = dv["len"] / 1000.0 * step / dv["dur"]
            pod["batt"] -= P["pod_drain_km"] * km
            pod["km"] += km
            self.k["pod_km"] += km
            f = 1.0 - max(0.0, dv["t_left"]) / dv["dur"]
            pod["pos"] = dv["from"] + (self.xy[dv["to"]] - dv["from"]) * f
            if dv["t_left"] <= 0:
                pod["node"], pod["pos"], pod["drive"] = dv["to"], self.xy[dv["to"]].copy(), None
                if dv["why"] == "locker":
                    pod["dwell"] = STOP_S
                    pod["drop_locker"] = dv["arg"]
                elif dv["why"] == "charge":
                    st = self.stations[dv["arg"]]
                    if st["bay"] is None:
                        st["bay"] = pod["id"]
                        pod["charging"] = st["id"]
                        self.k["charges"] += 1
                        if pod["batt"] < 35.0:
                            # Scaled docking reward: only rewarded when arriving with low battery (< 35%)
                            self.reward += 40.0 * min(max(0.0, 35.0 - pod["batt"]) / 25.0, 1.0)
                    else:
                        pod["queued"] = st["id"]
        pod["batt"] -= P["pod_idle_drain"] * DT
        # a low pod battery costs, more the lower it gets (the warning before a
        # stranding, which alone came too late to learn from) / 低電池は段階的にコスト
        if pod["batt"] < 25.0:
            # quadratic penalty for dangerously low battery
            self.reward -= 0.1 * ((25.0 - pod["batt"]) / 5.0) ** 2 * (DT / 5.0)
        if pod.get("queued") is not None:
            st = self.stations[pod["queued"]]
            if st["bay"] is None:
                st["bay"], pod["charging"], pod["queued"] = pod["id"], st["id"], None
                self.k["charges"] += 1
                if pod["batt"] < 35.0:
                    self.reward += 40.0 * min(max(0.0, 35.0 - pod["batt"]) / 25.0, 1.0)
        if pod["charging"] is not None:
            rate = P["station_kw"] / P["pod_kwh"] * 100.0 / 3600.0 * DT
            pod["batt"] = min(P["pod_charge_to"], pod["batt"] + rate)
        if pod["dwell"] > 0:
            pod["dwell"] -= DT
            if pod["dwell"] <= 0 and pod.get("drop_locker") is not None:
                for o in self.pod_orders(pod, "f2f"):
                    if o["locker"] == pod["drop_locker"]:
                        self._delivered(o)
                pod["drop_locker"] = None
        # drones charging inside the pod draw on its battery / ポッド内充電は車両電池から
        for d in self.drones:
            if d["pod"] == pod["id"] and d["state"] == DOCKED and d["batt"] < 100.0:
                add = min(P["charge_slow"] * DT, 100.0 - d["batt"])
                d["batt"] += add
                # drone-% -> Wh -> pod-%, 90 % charger efficiency / 効率 90 %
                pod["batt"] -= add * P["drone_wh"] / (P["pod_kwh"] * 1000.0) / 0.9
        if pod["batt"] <= 0.0:
            pod["batt"], pod["stranded"] = 0.0, True
            self.k["stranded"] += 1
            self.reward -= 2500.0                      # heavy penalty: wipes out gains from ~50 deliveries / 電欠脱落ペナルティ
            self._event("pod_stranded", pod=pod["id"])

    def pod_free(self, pod):
        """Can this pod take a decision now? / 判断可能か"""
        return (not pod["stranded"] and pod["drive"] is None and pod["dwell"] <= 0
                and pod.get("queued") is None and self.t >= pod["busy_until"]
                and not (pod["charging"] is not None and pod["batt"] < self.P["pod_charge_to"] - 0.5
                         and not pod.get("leave_charge")))

    # -- drones / ドローン
    def _flight(self, d, target, height, payload, kind, then):
        t, b, crash = self.fly(d["pos"], target, height, payload)
        d["leg"] = (self.t, t, np.array(d["pos"], float), np.array(target, float))
        d["state"], d["eta"], d["task"], d["pending_batt"], d["pending_crash"] = FLY, t, kind, b, crash
        d["then"] = then
        d["height"] = height
        self.k["flights"] += 1

    def _arrive(self, d):
        d["batt"] -= d.pop("pending_batt", 0.0)
        crash = d.pop("pending_crash", None)
        target = d["leg"][3]
        if crash or d["batt"] <= 0:
            # the crash happens somewhere along the leg / 区間の途中で墜落
            f = float(self.rng.uniform(0.2, 0.9))
            d["pos"] = d["leg"][2] + (target - d["leg"][2]) * f
            self._crash(d, crash or "battery")
            return
        d["pos"] = target.copy()
        then = d.pop("then", None)
        if then:
            then(d)

    def _crash(self, d, why):
        self.k["crash"] += 1
        self.k["crash_why"][why] = self.k["crash_why"].get(why, 0) + 1
        self.reward -= CRASH_COST
        self._event("crash", drone=d["id"], why=why)
        o = self.orders.get(d["parcel"]) if d["parcel"] is not None else None
        if o is not None:
            # the parcel lies at the crash site: a standby must recover it / 荷物は墜落地点
            o["state"], o["site"] = "site", d["pos"].copy()
            self._request("recover", pod=self.pods[o["pod"]], order=o)
        elif d["pod"] is not None:
            self._request("replace", pod=self.pods[d["pod"]])
        d.update(state=DOWN, timer=WRECK_S, parcel=None, task=None, leg=None, home=d["pod"], pod=None)

    def _request(self, kind, pod, order=None, tired=None):
        self.requests.append({"kind": kind, "pod": pod["id"], "order": order["id"] if order else None,
                              "tired": tired, "t": self.t, "declined": set()})

    def _delivered(self, o):
        o["state"] = "done"
        lead = self.t - o["t0"]
        self.k["delivered_f2f" if o["kind"] == "f2f" else "delivered_drone"] += 1
        self.k["lead"].append(lead)
        self.reward += 20.0 - 0.004 * lead
        self._event("delivered", order=o["id"], kind=o["kind"])

    def _land_in_pod(self, d, pod):
        d.update(state=DOCKED, pod=pod["id"], pos=pod["pos"].copy(), leg=None, task=None, hover_t=0.0)
        self.k["landing"].append(d["batt"])
        if d["batt"] < 20.0:
            self.reward -= 2.0 * (20.0 - d["batt"])

    def _to_station(self, d):
        st = min(self.stations, key=lambda s: math.dist(s["xy"], d["pos"]))
        d["pod"] = None

        def at_station(dd, st=st):
            dd.update(state=STANDBY, station=st["id"], task=None, leg=None)
            self.k["station_visits"] += 1
            self.k["landing"].append(dd["batt"])
            if dd["batt"] < 20.0:
                self.reward -= 2.0 * (20.0 - dd["batt"])
        self._flight(d, st["pads"][0], d["height"], 0.0, "station", at_station)

    def _drone_step(self, d):
        if d["state"] == FLY:
            d["eta"] -= DT
            if d["eta"] <= 0:
                self._arrive(d)
        elif d["state"] == HOVER:
            d["batt"] -= HOVER_PCT_S * DT
            d["hover_t"] += DT
            self.k["hover_s"] += DT
            self.reward -= 0.02 * DT
            pod = self.pods[d["pod"]] if d["pod"] is not None else None
            if pod is not None and pod["drive"] is None and math.dist(pod["pos"], d["pos"]) < 8.0:
                d["eta"] = LAND_S
                self._land_in_pod(d, pod)
            elif d["batt"] <= 0:
                self._crash(d, "battery")
        elif d["state"] == STANDBY:
            d["batt"] = min(100.0, d["batt"] + self.P["charge_fast"] * DT)
        elif d["state"] == DOWN:
            d["timer"] -= DT
            if d["timer"] <= 0:
                d.update(state=REPAIR, timer=REPAIR_S)
        elif d["state"] == REPAIR:
            d["timer"] -= DT
            if d["timer"] <= 0:
                st = self.stations[d["id"] % len(self.stations)]
                d.update(state=STANDBY, batt=100.0, station=st["id"], pos=st["pads"][0].copy())
        elif d["state"] == DOCKED and d["pod"] is not None:
            d["pos"] = self.pods[d["pod"]]["pos"].copy()

    # -- standby tasks / 待機機の仕事
    def _task_target(self, req):
        if req["kind"] == "recover":
            return self.orders[req["order"]]["site"]
        return self.pods[req["pod"]]["pos"]

    def _start_task(self, d, req):
        pod = self.pods[req["pod"]]
        if req["kind"] == "recover":
            o = self.orders[req["order"]]

            def at_site(dd, o=o, pod=pod):
                dd["parcel"] = o["id"]
                o["state"] = "drone"
                dd["eta"] = PICK_S
                self.k["recoveries"] += 1
                self.reward += 5.0

                def dropped(d3, o=o, pod=pod):
                    self._delivered(o)
                    d3["parcel"] = None
                    d3["pod"] = pod["id"]
                    self._go_meet(d3, pod)
                self._flight(dd, o["xy"], dd["height"], 1.0, "deliver", dropped)
            d["pod"] = None
            self._flight(d, o["site"], 35.0, 0.0, "recover", at_site)
        else:
            def at_pod(dd, pod=pod, req=req):
                dd["pod"] = pod["id"]
                if pod["drive"] is None and math.dist(dd["pos"], pod["pos"]) < 8.0:
                    self._land_in_pod(dd, pod)
                    if req["kind"] == "swap" and req["tired"] is not None:
                        tired = self.drones[req["tired"]]
                        if tired["state"] == DOCKED and tired["pod"] == pod["id"]:
                            self.k["swaps"] += 1
                            self._to_station(tired)
                    else:
                        self.k["replacements"] += 1
                    self.reward += 3.0
                else:
                    dd.update(state=HOVER, hover_t=0.0)     # wait for the pod / ポッド待ち
            self._flight(d, pod["pos"], 35.0, 0.0, req["kind"], at_pod)
        d["station"] = None

    def _go_meet(self, d, pod):
        tgt = pod["meet"] if pod["meet"] is not None else pod["pos"]

        def at_meet(dd, pod=pod):
            dd.update(state=HOVER, hover_t=0.0)
        self._flight(d, tgt, d["height"], 0.0, "meet", at_meet)

    # -- decisions / 判断
    def pending(self):
        """Next decision: ("pod", pod) or ("drone", drone, kind) or None. / 次の判断"""
        for d in self.drones:
            if d.get("ask") is not None:
                return ("drone", d, d["ask"])
        for req in self.requests:
            if req.get("retry_t", 0.0) > self.t:
                continue
            cands = [d for d in self.drones if d["state"] == STANDBY and d["id"] not in req["declined"]]
            if cands:
                best = max(cands, key=lambda d: d["batt"] - 0.02 * math.dist(d["pos"], self._task_target(req)))
                best["ask"], best["req"] = D_TASK, req
                return ("drone", best, D_TASK)
        for pod in self.pods:
            if self.pod_free(pod) and pod.get("decided_t", -1) < self.t:
                return ("pod", pod)
        return None

    def advance(self):
        """Run the world until a decision is needed; returns it (or None at the end)."""
        while self.t < EP_S:
            p = self.pending()
            if p is not None:
                return p
            self.t += DT
            self._weather()
            self._spawn_orders()
            # every waiting parcel costs, minute by minute; escalating SLA penalty for starving orders
            unfulfilled = [o for o in self.orders.values() if o["state"] != "done"]
            self.reward -= 0.05 * len(unfulfilled) * DT / 60.0
            if unfulfilled:
                ages = [self.t - o["t0"] for o in unfulfilled]
                max_age = max(ages)
                self.k["max_order_age"] = max(self.k.get("max_order_age", 0.0), max_age)
                expired = [age for age in ages if age > 1800.0]
                self.k["expired_orders"] = max(self.k.get("expired_orders", 0), len(expired))
                if expired:
                    starv_pen = 0.05 * sum(((age - 1800.0) / 600.0) ** 2 for age in expired) * (DT / 60.0)
                    self.reward -= min(starv_pen, 5.0)
            for pod in self.pods:
                self._pod_step(pod)
                pod["decided_t"] = -1
            for d in self.drones:
                self._drone_step(d)
                if d["state"] == HOVER and d["hover_t"] > 0 and int(d["hover_t"]) % 30 == 0:
                    d["ask"] = D_HOVER
            self._log()
        return None

    # -- the pod brain's options (candidates) / ポッドの選択肢
    def pod_options(self, pod):
        """The options of this decision: built once per pod and moment, so what the
        brain sees and what it may choose match. / この判断の選択肢（1 回だけ作る）"""
        key = (self.t, len(self.orders), sum(o["state"] == "done" for o in self.orders.values()))
        if pod.get("opt_key") == key:
            return pod["spot_cands"], pod["lock_cands"], pod["meet_cands"]
        pod["opt_key"] = key
        z = pod["zone"]
        dro = self.pod_orders(pod, "drone")
        if dro:
            c = np.mean([o["xy"] for o in dro], axis=0)
            nodes = sorted(self.zone_nodes[z], key=lambda i: math.dist(self.xy[i], c))
            spots = [nodes[0]]
            for r in (40.0, 80.0, 140.0, 220.0, 320.0):
                ring = [i for i in nodes if r - 25.0 < math.dist(self.xy[i], c) < r + 25.0]
                if ring:
                    spots.append(ring[int(self.rng.integers(len(ring)))])
            spots = (spots + [pod["node"]] * K_SPOT)[:K_SPOT]
        else:
            spots = [pod["node"]] * K_SPOT
        f2f = self.pod_orders(pod, "f2f")
        lks = sorted({o["locker"] for o in f2f}, key=lambda i: self.route(pod["node"], self.lockers[i]["node"]))
        lks = (lks + [None] * K_LOCK)[:K_LOCK]
        here = pod["node"]
        flying = [d for d in self.drones if d["pod"] == pod["id"] and d["state"] in (FLY, HOVER)]
        if dro:
            c = np.mean([o["xy"] for o in dro], axis=0)
        elif flying:
            c = np.mean([d["leg"][3] if d["leg"] is not None else d["pos"] for d in flying], axis=0)
        else:
            c = pod["pos"]
        mid = self._near(c, self.zone_nodes[z])
        nxt = self.lockers[lks[0]]["node"] if lks[0] is not None else mid
        meets = [here, mid, self._near((self.xy[mid] + self.xy[nxt]) / 2.0, self.zone_nodes[z])]
        pod["spot_cands"], pod["lock_cands"], pod["meet_cands"] = spots, lks, meets
        return spots, lks, meets

    def pod_mask(self, pod):
        spots, lks, meets = self.pod_options(pod)
        m = np.zeros(N_POD_ACT, bool)
        docked = [d for d in self.drones if d["pod"] == pod["id"] and d["state"] == DOCKED]
        n_dro = len(self.pod_orders(pod, "drone"))
        if n_dro:
            m[A_SPOT:A_SPOT + K_SPOT] = True
            for n in range(1, min(len(docked), n_dro) + 1):
                m[A_LAUNCH + (n - 1) * K_MEET:A_LAUNCH + n * K_MEET] = True
        for j, L in enumerate(lks):
            m[A_LOCK + j] = L is not None
        out = [d for d in self.drones if d["pod"] == pod["id"] and d["state"] in (FLY, HOVER)]
        m[A_GOMEET] = bool(out) and pod["meet_node"] is not None
        n_stn = min(K_STN, len(self.stations))
        m[A_CHARGE:A_CHARGE + n_stn] = pod["batt"] < 50.0   # mask charging if battery >= 50% (anti-docking loop)
        # Swap requires docked drone, standby available, 300s cooldown, max 4 swaps/episode
        m[A_SWAP] = (bool(docked) and any(d["state"] == STANDBY for d in self.drones)
                     and (self.t - pod.get("last_swap_t", -999.0) > 300.0)
                     and (pod.get("swaps_used", 0) < 4))
        m[A_WAIT] = True

        # Battery-protective action masking / 電池切れ防止のアクションマスク
        if pod["batt"] < 25.0:
            # Stop launching new drones or driving out to delivery spots
            m[A_SPOT:A_SPOT + K_SPOT] = False
            m[A_LAUNCH:A_LOCK] = False
        if pod["batt"] < 15.0:
            # Below 15%: forbid lockers and swaps as well; only charge or meet active airborne drones
            m[A_LOCK:A_GOMEET] = False
            m[A_SWAP] = False
            if not out:
                m[A_GOMEET] = False
                # If no airborne drones and chargers are reachable, disallow passive waiting
                if any(m[A_CHARGE:A_CHARGE + n_stn]):
                    m[A_WAIT] = False

        # Fallback safety: at least one action must always be legal for MaskablePPO
        if not np.any(m):
            m[A_WAIT] = True
        return m

    def pod_act(self, pod, a):
        a = int(a)
        pod["decided_t"] = self.t
        spots, lks, meets = self.pod_options(pod)
        if pod["charging"] is not None and a != A_WAIT:
            self.stations[pod["charging"]]["bay"] = None          # leave the bay / ベイを出る
            pod["charging"] = None
        if A_SPOT <= a < A_LAUNCH:
            self._drive(pod, spots[a - A_SPOT], "spot")
        elif A_LAUNCH <= a < A_LOCK:
            n, mk = (a - A_LAUNCH) // K_MEET + 1, (a - A_LAUNCH) % K_MEET
            pod["meet_node"] = meets[mk]
            pod["meet"] = self.xy[meets[mk]].copy()
            docked = [d for d in self.drones if d["pod"] == pod["id"] and d["state"] == DOCKED]
            docked.sort(key=lambda d: -d["batt"])
            orders = sorted(self.pod_orders(pod, "drone"), key=lambda o: math.dist(o["xy"], pod["pos"]))
            sky = self.open_sky(pod["node"])
            for d, o in zip(docked[:n], orders):
                d["parcel"], o["state"] = o["id"], "drone"
                d["pos"] = pod["pos"].copy()
                if self.rng.random() < (1.0 - sky) * 0.6:            # canopy over the pod / 樹冠
                    self.k["takeoff_crash"] += 1
                    self._crash(d, "takeoff")
                    continue
                d["ask"] = D_LAUNCH                                  # the drone picks its height
            pod["busy_until"] = self.t + 10.0
        elif A_LOCK <= a < A_GOMEET:
            L = lks[a - A_LOCK]
            self._drive(pod, self.lockers[L]["node"], "locker", L)
        elif a == A_GOMEET:
            if pod["meet_node"] == pod["node"]:
                pod["busy_until"] = self.t + WAIT_S              # already there: wait / 到着済み
            else:
                self._drive(pod, pod["meet_node"], "meet")
        elif A_CHARGE <= a < A_SWAP:
            s = a - A_CHARGE
            if s < len(self.stations):
                self._drive(pod, self.stations[s]["node"], "charge", s)
        elif a == A_SWAP:
            pod["swaps_used"] = pod.get("swaps_used", 0) + 1
            pod["last_swap_t"] = self.t
            self.reward -= 10.0                                  # swap logistics overhead penalty
            self.k["swaps"] += 1
            docked = [d for d in self.drones if d["pod"] == pod["id"] and d["state"] == DOCKED]
            tired = min(docked, key=lambda d: d["batt"])
            self._request("swap", pod, tired=tired["id"])
            pod["busy_until"] = self.t + WAIT_S
        else:
            pod["busy_until"] = self.t + WAIT_S
            if pod["charging"] is None:
                # waiting while there is work it could do costs much more: per decision
                # a 30 s wait otherwise looked cheaper than minutes-long drives
                # 仕事があるのに待つと高コスト（判断ごとの報酬では待機が安く見えた）
                docked = any(d["pod"] == pod["id"] and d["state"] == DOCKED for d in self.drones)
                work = (docked and self.pod_orders(pod, "drone")) or self.pod_orders(pod, "f2f") \
                    or pod["batt"] < 30.0
                self.reward -= 3.0 if work else 0.2

    def drone_mask(self, d, kind):
        m = np.zeros(3, bool)
        if kind == D_LAUNCH:
            m[:] = True
        elif kind == D_AFTER:
            # Paper 1 Hard Feasibility Masking (FA-RAMPPO):
            # a=0: Recover to LMP-4 pod
            # a=1: Divert to fixed charging station
            pod = self.pods[d["pod"]] if d["pod"] is not None else None
            m[1] = True  # Station charger fallback
            if pod is not None:
                tgt = pod["meet"] if pod["meet"] is not None else self.xy[pod["node"]]
                dist_m = float(np.hypot(d["pos"][0] - tgt[0], d["pos"][1] - tgt[1]))
                # Energy needed plus 15% reserve
                need_batt = (dist_m / self.P["drone_speed"]) * 0.25 + 15.0
                pod_stopped = (pod["state"] in ("spot", "wait"))
                if d["batt"] >= need_batt and (pod_stopped or getattr(self, "recovery_mode", "stopped_bay") == "moving_roof"):
                    m[0] = True
        else:
            m[:2] = True
        return m

    def drone_act(self, d, kind, a):
        a = int(a)
        d["ask"] = None
        if kind == D_LAUNCH:
            o = self.orders[d["parcel"]]
            pod = self.pods[d["pod"]]

            def dropped(dd, o=o, pod=pod):
                self._delivered(o)
                dd["parcel"] = None
                dd["ask"] = D_AFTER
            self._flight(d, o["xy"], HEIGHTS[a], 1.0, "deliver", dropped)
        elif kind == D_AFTER:
            pod = self.pods[d["pod"]] if d["pod"] is not None else None
            if a == 0 and pod is not None:
                self._go_meet(d, pod)
            else:
                self._to_station(d)
                if pod is not None:
                    self._request("replace", pod)
        elif kind == D_HOVER:
            if a == 1:
                pod = self.pods[d["pod"]] if d["pod"] is not None else None
                self._to_station(d)
                if pod is not None:
                    self._request("replace", pod)
        elif kind == D_TASK:
            req = d.pop("req")
            if a == 0:
                self.requests.remove(req)
                self._start_task(d, req)
            else:
                req["declined"].add(d["id"])
                if all(dd["id"] in req["declined"] for dd in self.drones if dd["state"] == STANDBY):
                    req["declined"].clear()                          # ask again in 60 s / 60 秒後に再依頼
                    req["retry_t"] = self.t + 60.0

    # -- observations / 観測
    def _wind_obs(self):
        ws = self.wind_hist or [0.0]
        return [float(np.mean(ws)) / 10.0, float(np.std(ws)) / 3.0, float(np.max(ws)) / 10.0, self.rain / 50.0]

    def pod_obs(self, pod):
        spots, lks, meets = self.pod_options(pod)
        P = self.P
        mine = [d for d in self.drones if d["pod"] == pod["id"]]
        docked = sorted([d["batt"] for d in mine if d["state"] == DOCKED], reverse=True)
        hover = [d for d in mine if d["state"] == HOVER]
        flying = [d for d in mine if d["state"] == FLY]
        dro, f2f = self.pod_orders(pod, "drone"), self.pod_orders(pod, "f2f")
        v = [pod["batt"] / 100.0, float(pod["charging"] is not None), len(dro) / 8.0, len(f2f) / 8.0,
             (max([self.t - o["t0"] for o in dro + f2f]) / 1800.0) if dro + f2f else 0.0,
             len(docked) / 4.0] + (docked + [0.0] * 4)[:4]
        v[6:10] = [b / 100.0 for b in v[6:10]]
        v += [len(flying) / 4.0, len(hover) / 4.0, min([d["batt"] for d in hover], default=100.0) / 100.0,
              max([d["hover_t"] for d in hover], default=0.0) / 600.0,
              min([d["eta"] for d in flying], default=0.0) / 600.0,
              sum(d["state"] == STANDBY for d in self.drones) / 4.0,
              float(any(r["pod"] == pod["id"] for r in self.requests))]
        v += self._wind_obs()
        sp = self.pod_speed()
        for n in spots:
            v += [self.route(pod["node"], n) / sp / 600.0,
                  (float(np.mean([math.dist(self.xy[n], o["xy"]) for o in dro])) / 500.0) if dro else 0.0,
                  (float(np.max([math.dist(self.xy[n], o["xy"]) for o in dro])) / 500.0) if dro else 0.0,
                  self.open_sky(n)]
        for L in lks:
            if L is None:
                v += [0.0, 0.0, 0.0]
            else:
                ords = [o for o in f2f if o["locker"] == L]
                v += [self.route(pod["node"], self.lockers[L]["node"]) / sp / 600.0, len(ords) / 4.0,
                      max([self.t - o["t0"] for o in ords], default=0.0) / 1800.0]
        for n in meets:
            v += [self.route(pod["node"], n) / sp / 600.0,
                  (float(np.mean([math.dist(self.xy[n], d["leg"][3]) for d in flying if d["leg"] is not None]))
                   / 500.0) if flying else 0.0,
                  self.open_sky(n)]
        for s in range(K_STN):
            if s < len(self.stations):
                st = self.stations[s]
                v += [self.route(pod["node"], st["node"]) / sp / 600.0, float(st["bay"] is None),
                      float(self._zone(st["xy"]) == pod["zone"])]
            else:
                v += [0.0, 0.0, 0.0]
        v += [self.rate / 100.0]
        v += [float(pod["meet_node"] is not None),
              (self.route(pod["node"], pod["meet_node"]) / sp / 600.0) if pod["meet_node"] is not None else 0.0,
              self.open_sky(pod["node"]), self.t / EP_S]
        out = np.zeros(POD_OBS, np.float32)
        out[:min(len(v), POD_OBS)] = np.clip(np.array(v[:POD_OBS], np.float32), -10, 10)
        return out

    def drone_obs(self, d, kind):
        pod = self.pods[d["pod"]] if d["pod"] is not None else None
        tgt = d["pos"]
        if kind == D_LAUNCH and d["parcel"] is not None:
            tgt = self.orders[d["parcel"]]["xy"]
        elif kind == D_TASK:
            tgt = self._task_target(d["req"])
        elif pod is not None and pod["meet"] is not None:
            tgt = pod["meet"]
        st = min(self.stations, key=lambda s: math.dist(s["xy"], d["pos"]))
        eta_pod = 0.0
        if pod is not None and pod["meet_node"] is not None:
            eta_pod = (pod["drive"]["t_left"] if pod["drive"] else
                       self.route(pod["node"], pod["meet_node"]) / self.pod_speed()) / 600.0
        v = [float(kind == k) for k in range(4)] + [
            d["batt"] / 100.0, math.dist(d["pos"], tgt) / 1000.0, math.dist(d["pos"], st["xy"]) / 1000.0,
            math.dist(tgt, st["xy"]) / 1000.0, eta_pod, d["hover_t"] / 600.0,
            float(pod is not None and pod["drive"] is not None)] + self._wind_obs() + [self.t / EP_S]
        return np.clip(np.array(v[:DRONE_OBS], np.float32), -10, 10)

    # -- log, KPIs / 記録と KPI
    def _event(self, what, **kw):
        if self.log is not None:
            self.events.append(dict(kw, t=self.t, what=what))

    def _draw_pos(self, d):
        if d["state"] == FLY and d["leg"] is not None:
            t0, dur, a, b = d["leg"]
            f = min(1.0, max(0.0, (self.t - t0) / max(dur, 1e-3)))
            return a + (b - a) * f
        if d["state"] == DOCKED and d["pod"] is not None:
            return self.pods[d["pod"]]["pos"]
        return d["pos"]

    def _log(self):
        if self.log is None:
            return
        self.log.append({
            "t": self.t,
            "pods": [[round(float(p["pos"][0]), 1), round(float(p["pos"][1]), 1), round(p["batt"], 1),
                      int(p["charging"] is not None), int(p["stranded"]),
                      None if p["meet"] is None else [round(float(p["meet"][0]), 1), round(float(p["meet"][1]), 1)]]
                     for p in self.pods],
            "drones": [[round(float(q[0]), 1), round(float(q[1]), 1), d["state"], round(d["batt"], 1),
                        d["pod"], int(d["parcel"] is not None)]
                       for d, q in ((d, self._draw_pos(d)) for d in self.drones)],
            "orders": [[round(float(o["xy"][0]), 1), round(float(o["xy"][1]), 1), o["kind"][0], o["state"][0]]
                       for o in self.orders.values() if o["state"] != "done"],
            "wind": round(float(np.mean(self.wind_hist or [0])), 1), "rain": round(self.rain, 1)})

    def kpi(self):
        k = self.k
        hours = max(self.t, 1.0) / 3600.0
        lead = k["lead"]
        return {"hours": round(hours, 2), "order_rate": self.rate,
                "deliveries_per_h": (k["delivered_f2f"] + k["delivered_drone"]) / hours,
                "f2f_per_h": k["delivered_f2f"] / hours, "drone_per_h": k["delivered_drone"] / hours,
                "lead_mean_s": float(np.mean(lead)) if lead else None,
                "crashes_per_100": 100.0 * k["crash"] / max(k["flights"], 1), "crash_why": dict(k["crash_why"]),
                "landing_min": float(min(k["landing"])) if k["landing"] else None,
                "hover_s_per_h": k["hover_s"] / hours, "swaps": k["swaps"], "recoveries": k["recoveries"],
                "replacements": k["replacements"], "stranded": k["stranded"], "charges": k["charges"],
                "pod_km": k["pod_km"], "takeoff_crash": k["takeoff_crash"],
                "max_order_age_s": float(k.get("max_order_age", 0.0)),
                "expired_orders": int(k.get("expired_orders", 0)),
                "waiting": sum(o["state"] != "done" for o in self.orders.values()),
                "parcels_at_crash_sites": sum(o["state"] == "site" for o in self.orders.values())}


class _View(gym.Env):
    """One brain's view of the fleet world; the other brain is ``other``.
    片方のブレインから見た世界（もう片方は other が判断）"""
    metadata = {"render_modes": []}

    def __init__(self, me, map_name="OitaKosen", seed=None, other=None, other_dir=None):
        self.me = me
        self.W = Fleet(map_name, seed=seed)
        self.other = other            # callable(obs, mask) -> action, or None = rule baseline
        # other_dir: the other brain's checkpoint folder; its newest model is
        # loaded at every reset (training in turns) / 相手の最新モデルを毎回読む
        self.other_dir, self._other_file = other_dir, None
        n_obs, n_act = (POD_OBS, N_POD_ACT) if me == "pod" else (DRONE_OBS, 3)
        self.observation_space = gym.spaces.Box(-10.0, 10.0, (n_obs,), np.float32)
        self.action_space = gym.spaces.Discrete(n_act)
        self.cur = None

    def _rule(self, p):
        from rl_delivery.fleet_rules import rule_pod, rule_drone
        return rule_pod(self.W, p[1]) if p[0] == "pod" else rule_drone(self.W, p[1], p[2])

    def _next(self):
        """Advance, answering the other brain's decisions, to my next decision."""
        W = self.W
        while True:
            p = W.advance()
            if p is None:
                return None
            if p[0] == self.me:
                return p
            if self.other is not None:
                if p[0] == "pod":
                    a = self.other(W.pod_obs(p[1]), W.pod_mask(p[1]))
                else:
                    a = self.other(W.drone_obs(p[1], p[2]), W.drone_mask(p[1], p[2]))
            else:
                a = self._rule(p)
            if p[0] == "pod":
                W.pod_act(p[1], a)
            else:
                W.drone_act(p[1], p[2], a)

    def _obs(self):
        if self.cur is None:
            return np.zeros(self.observation_space.shape, np.float32)
        p = self.cur
        return self.W.pod_obs(p[1]) if p[0] == "pod" else self.W.drone_obs(p[1], p[2])

    def action_masks(self):
        if self.cur is None:
            m = np.zeros(self.action_space.n, bool)
            m[-1] = True
            return m
        p = self.cur
        return self.W.pod_mask(p[1]) if p[0] == "pod" else self.W.drone_mask(p[1], p[2])

    def _load_other(self):
        import glob
        files = glob.glob(os.path.join(self.other_dir, "step_*.zip"))
        if not files:
            self.other = None
            return
        newest = max(files, key=lambda p: int(os.path.basename(p)[5:-4]))
        if newest != self._other_file:
            from sb3_contrib import MaskablePPO
            model = MaskablePPO.load(newest, device="cpu")
            self.other = lambda o, m, model=model: int(model.predict(o, action_masks=m, deterministic=False)[0])
            self._other_file = newest

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.other_dir:
            try:
                self._load_other()
            except Exception:
                self.other = None
        self.W.reset(seed=seed if seed is not None else int(np.random.randint(1 << 30)))
        self.W.requests = []
        self.cur = self._next()
        return self._obs(), {}

    def step(self, action):
        W = self.W
        W.reward = 0.0
        if self.cur is not None:
            p = self.cur
            if p[0] == "pod":
                W.pod_act(p[1], action)
            else:
                W.drone_act(p[1], p[2], action)
        self.cur = self._next()
        r = W.reward
        term = self.cur is None
        info = {"kpi": W.kpi()} if term else {}
        return self._obs(), float(r), term, False, info


class PodView(_View):
    def __init__(self, map_name="OitaKosen", seed=None, other=None, other_dir=None):
        super().__init__("pod", map_name, seed, other, other_dir)


class DroneView(_View):
    def __init__(self, map_name="OitaKosen", seed=None, other=None, other_dir=None):
        super().__init__("drone", map_name, seed, other, other_dir)
