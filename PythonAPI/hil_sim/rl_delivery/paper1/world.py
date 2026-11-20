"""Paper 1 execution world: one LMP-4, four drones, fixed chargers (OitaKosen).

A subclass of the fleet world (env_fleet.Fleet) narrowed to the scope of the
Paper 1 guide (docs/PAPER1_BASELINES.md): customers are served only by drones,
one LMP-4 mobile hub carries the parcel stock and recovers drones, fixed
charging stations give energy only. The decision under test is the drone's
**recovery choice after a drop** (go to the pod's meeting point, or charger
A / B / C); everything else follows frozen rules (fleet_rules.rule_pod for the
pod, a fixed launch height, a fixed hover rule), the same for every method.

What this adds to the fleet world:
- charger capacity and queues (each station has len(deck) pads);
- the empty flight back from a charger to the pod after charging;
- one landing at a time on the pod's roof pad (LAND_S each);
- Paper 1 scenarios S1..S7 (order rate, traffic, pod delays, a station outage,
  high wind, burst demand, held-out customers); rain is off (it grounded and
  crashed a quarter of flights at random, which would swamp the comparison);
- measurements: drop-off -> ready time, recovery failures, pad / charger
  conflicts, waiting drone-minutes, energy, empty-return distance, on-time at
  15 / 30 / 60 min.
Estimates used by decision rules (estimate()) are deterministic and never
draw from the world's random generator, so a method's thinking cannot change
the world's randomness (paired seeds stay paired).

Paper 1 の実行用世界：LMP-4 1 台・ドローン 4 機・固定充電器（大分高専）。艦隊世界を
Paper 1 の範囲に絞り、評価対象はドロップ後の回収先の選択だけ（他は固定ルール）。
充電器の容量と待ち行列、充電後の空荷帰還、屋根パッドの着陸順、シナリオ S1〜S7、
評価指標を追加。判断用の見積りは乱数を使わない（対応のあるシードを保つ）。
"""
import math

import numpy as np

from rl_delivery import env_fleet as F
from rl_delivery.fleet_rules import rule_pod
from rl_delivery.wind import Wind

READY_PCT = 45.0          # "ready": docked in the pod with this much battery (fleet rule's launch level) / 準備完了
RESERVE_PCT = 15.0        # battery that must be left on arrival (the existing mask) / 到着時の予備
STATION_CHARGE_TO = 90.0  # charge on a station deck to this, then fly back / ステーションでの充電目標
EST_MARGIN = 1.2          # safety factor on estimated flight energy / 見積り余裕
ON_TIME_S = (900.0, 1800.0, 3600.0)   # on-time thresholds reported together / 時間内の基準
PAPER1_ZONE = 1           # the LMP-4's zone: 891 addresses, ~1 km x 0.6 km / LMP-4 の担当ゾーン

# Paper 1 scenarios (docs/PAPER1_BASELINES.md section 5.1). 16 orders/h = the
# rate measured in the CARLA delivery sim (47/h for 3 pods) per pod / 1 ポッドあたりの注文数
# Paper 1 のシナリオ
SCENARIOS = {
    "S1_normal": {"rate": 16.0},
    "S2_rush_hour": {"rate": 16.0, "traffic": 1.6},
    "S3_pod_delay": {"rate": 16.0, "delay_p": 0.3, "delay_s": (60.0, 300.0)},
    "S4_station_outage": {"rate": 16.0, "outage": (3600.0, 7200.0)},
    "S5_high_wind": {"rate": 16.0, "wind_mean": 15.0, "wind_gust": 18.0},
    "S6_burst_returns": {"rate": 16.0, "burst": 6},
    "S7_unseen_customers": {"rate": 16.0, "addresses": "heldout"},
}

STN_CHARGE, STN_QUEUE = "stn_charge", "stn_queue"      # drone states at a station / ステーションでの状態

# Hardware profiles. "v1" = the fleet world's placeholders (deck charge 0->100 % in
# ~67 s, in-pod ~21 min, ~6.7 min of hover on a full battery). "typical" = typical
# published values for small delivery drones (user choice 2026-11-16): 500 Wh, ~25 min
# of hover, station fast charge 2C (30 min empty -> full), in-pod charge 1C (60 min;
# the pod feeds 4 drones from its 10 kWh battery).
# 機体プロファイル：v1 は艦隊世界の仮の値、typical は一般的な配送ドローンの値。
PROFILES = {
    "v1": {},
    "typical": {"charge_fast": 100.0 / 1800.0, "charge_slow": 100.0 / 3600.0, "hover_pct_s": 100.0 / 1500.0},
}

# World versions (docs/PAPER1_BASELINES.md 3b). "v1" = the original Paper 1 world
# (drone-only customers, the pod waits for its drones: the recovery choice did not
# matter). "v2" (2026-11-16, user go-ahead) = the world where it matters: the pod
# delivers 40 % of orders itself at lockers, typical hardware, 1 pad per station,
# zones 1 + 3, 25 orders/h; the recovery choice includes "meet the pod at its next
# stop"; the frozen pod rule stops locker trips and charges when its battery is low.
# 世界のバージョン：v1 は元の世界、v2 は回収判断が結果を左右する世界（ポッド配達あり等）。
WORLDS = {
    "v1": {},
    "v2": {"pads": 1, "profile": "typical", "area": [1, 3], "rate": 25.0, "pod_share": 0.4,
           "meet_choice": True, "pod_fix": True},
}
POD_LOW = 40.0          # v2 pod rule: below this the pod gathers its drones and charges / 低電池しきい値


def path_len(world, a, b):
    """Flight distance a -> b round the no-fly zones [m]. / 飛行距離"""
    pts = [(a[0], a[1])] + list(world.air.path(a, b))
    return sum(math.dist(p, q) for p, q in zip(pts[:-1], pts[1:]))


class Paper1World(F.Fleet):
    """The fleet world with Paper 1's scope, queues and measurements.
    Paper 1 の範囲・待ち行列・評価指標を持つ艦隊世界。"""

    def __init__(self, map_name="OitaKosen", seed=None, scenario="S1_normal", zone=PAPER1_ZONE, pads=None,
                 profile="v1", area=None, rate=None, meet=None, pod_share=0.0, world="v1",
                 meet_choice=None, pod_fix=None):
        pre = WORLDS[world]                                     # explicit arguments win / 明示の引数が優先
        pads = pads if pads is not None else pre.get("pads")
        profile = pre.get("profile", profile) if profile == "v1" else profile
        area = area or pre.get("area")
        rate = rate or pre.get("rate")
        pod_share = pod_share or pre.get("pod_share", 0.0)
        super().__init__(map_name, seed=seed, drone_prob=1.0)
        self.world_name = world
        self.meet_choice = pre.get("meet_choice", False) if meet_choice is None else bool(meet_choice)
        self.pod_fix = pre.get("pod_fix", False) if pod_fix is None else bool(pod_fix)
        if scenario not in SCENARIOS:
            raise ValueError("unknown scenario %r / 不明なシナリオ" % scenario)
        self.scenario, self.sc, self.zone_id = scenario, SCENARIOS[scenario], zone
        # usable charging pads per station: None = every deck pad of the map (3); 1 = the
        # guide's pad-count ablation, where chargers fill and queue / 局あたりの使用パッド数
        self.pads = pads
        prof = PROFILES[profile]
        self.profile = profile
        self.P = dict(self.P)                                   # this world's copy / この世界用の写し
        for k in ("charge_fast", "charge_slow"):
            if k in prof:
                self.P[k] = prof[k]
        self.hover_pct = prof.get("hover_pct_s", F.HOVER_PCT_S)
        # customers: a fixed split of the zone's addresses, training / evaluation (S1-S6)
        # vs held out (S7) / 顧客：ゾーン内住所を固定分割（S7 は未使用側）
        # area: zones served by the one hub (default: its own zone). Customers come
        # from all of them and the pod may drive and stop anywhere in them.
        # area：1 台のハブが担当するゾーン（既定は自ゾーン）。顧客もポッドの停車地点も全域。
        self.area = list(area) if area else [zone]
        self.rate_override = rate
        # meet: diagnostic override of the pod rule's meeting point at a launch:
        # None = the rule (option 1, "middle"); 0 here, 1 middle, 2 towards the next
        # stop; "best" = the option with the shortest total flight back for the
        # launched drones. 合流地点の診断用上書き（None は既定ルール）。
        self.meet_fixed = meet
        # pod_share: fraction of orders the pod delivers itself at a locker stop
        # (face-to-face), so it has its own work while its drones are out. 0 =
        # drone-only customers (the original Paper 1 world).
        # pod_share：ポッド自身がロッカーで渡す注文の割合（0 は従来のドローン専用）。
        self.pod_share = float(pod_share or 0.0)
        if len(self.area) > 1:
            self.zone_nodes[zone] = sorted({n for z in self.area for n in self.zone_nodes[z]})
        addr = sorted([b for z in self.area for b in self.addr[z]],
                      key=lambda b: (round(b["x"], 1), round(b["y"], 1)))
        self.addr_main = addr[0::2]
        self.addr_heldout = addr[1::2]

    # -- episode / エピソード
    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        sc = self.sc
        self.rate = float(self.rate_override or sc["rate"])
        self.t = 0.0
        self.wind = Wind(self.rng, max_mean=sc.get("wind_mean", 12.0), max_gust=sc.get("wind_gust", 15.0),
                         level=1.0, grid=self.grid)
        self.rain = self.rain_target = 0.0
        self.wind_hist = []
        self.oid = 0
        self.orders = {}
        self.zones_active = [self.zone_id]
        self.next_order = {self.zone_id: float(self.rng.uniform(0, 30))}
        mean_gap = 3600.0 / self.rate
        self.order_gap = (0.5 * mean_gap, 1.5 * mean_gap)
        self.pods, self.drones = [], []
        z = self.zone_id
        n = int(self.rng.choice(self.zone_nodes[z]))
        pod = {"id": 0, "zone": z, "node": n, "pos": self.xy[n].copy(),
               "batt": float(self.rng.uniform(70, 100)), "drive": None, "dwell": 0.0, "charging": None,
               "meet": None, "meet_node": None, "spot_cands": [], "lock_cands": [], "meet_cands": [],
               "busy_until": 0.0, "stranded": False, "km": 0.0, "kwh": 0.0, "pad_free_t": 0.0}
        self.pods.append(pod)
        for _k in range(F.N_PER_POD):
            self._new_drone(pod=pod)
        for st in self.stations:
            st["slots"] = [None] * (min(len(st["pads"]), self.pads) if self.pads else len(st["pads"]))
            st["queue"] = []
            st["bay"] = None
        self.outage_station = None
        if "outage" in sc:
            # the station nearest the zone's centre goes out / ゾーン中心に最も近い局が停止
            r = self.zones[z]
            c = ((r["x0"] + r["x1"]) / 2.0, (r["y0"] + r["y1"]) / 2.0)
            self.outage_station = min(range(len(self.stations)), key=lambda i: math.dist(self.stations[i]["xy"], c))
        self.decisions, self.requests, self.events = [], [], []
        self.k = {"delivered_f2f": 0, "delivered_drone": 0, "lead": [], "crash": 0, "crash_why": {},
                  "flights": 0, "landing": [], "hover_s": 0.0, "swaps": 0, "recoveries": 0,
                  "replacements": 0, "stranded": 0, "pod_kwh": 0.0, "pod_km": 0.0, "charges": 0,
                  "parcels_lost": 0, "takeoff_crash": 0, "station_visits": 0}
        self.p1 = {"ready": [], "fail": 0, "fail_why": {}, "pad_conflicts": 0, "charger_conflicts": 0,
                   "wait_s": 0.0, "energy_pct": 0.0, "empty_m": 0.0, "recoveries": 0, "to_pod": 0,
                   "to_station": 0, "fallbacks": 0, "hover_rule_station": 0, "ready_events": []}
        self.reward = 0.0
        return self

    # -- scenario hooks / シナリオ
    def station_out(self, i, t=None):
        """Is station i out of service at time t (now by default)? / 停止中か"""
        if self.outage_station is None or i != self.outage_station:
            return False
        t0, t1 = self.sc["outage"]
        t = self.t if t is None else t
        return t0 <= t < t1

    def _spawn_orders(self):
        z = self.zone_id
        addr = self.addr_heldout if self.sc.get("addresses") == "heldout" else self.addr_main
        burst = int(self.sc.get("burst", 1))
        while self.t >= self.next_order[z]:
            for _b in range(burst):
                b = addr[int(self.rng.integers(len(addr)))]
                o = {"id": self.oid, "zone": z, "kind": "drone", "xy": np.array([b["x"], b["y"]]), "t0": self.t,
                     "state": "pod", "pod": 0, "locker": None, "site": None}
                if self.pod_share and self.rng.random() < self.pod_share:
                    lk = [i for i, L in enumerate(self.lockers) if L["zone"] in self.area]
                    if lk:
                        o["kind"] = "f2f"
                        o["locker"] = min(lk, key=lambda i: math.dist(self.lockers[i]["xy"], o["xy"]))
                self.orders[o["id"]] = o
                self.oid += 1
            # a burst of n orders comes n times less often (same mean rate) / 同じ平均注文数
            self.next_order[z] += burst * float(self.rng.uniform(*self.order_gap))

    def _weather(self):
        self.wind.step(F.DT, 30.0)
        p = self.pods[0]["pos"]
        w = self.wind.at(p[0], p[1], 10.0)
        self.wind_hist = (self.wind_hist + [float(np.hypot(w[0], w[1]) + self.rng.normal(0, 0.4))])[-12:]

    def _drive(self, pod, node, why, arg=None):
        super()._drive(pod, node, why, arg)
        dv = pod["drive"]
        f = float(self.sc.get("traffic", 1.0))
        extra = 0.0
        if "delay_p" in self.sc and self.rng.random() < self.sc["delay_p"]:
            extra = float(self.rng.uniform(*self.sc["delay_s"]))
        dv["dur"] = dv["dur"] * f + extra
        dv["t_left"] = dv["dur"]

    def pod_speed_eff(self):
        return self.pod_speed() / float(self.sc.get("traffic", 1.0))

    def pod_act(self, pod, a):
        a = int(a)
        if self.meet_fixed is not None and F.A_LAUNCH <= a < F.A_LOCK:
            n = (a - F.A_LAUNCH) // F.K_MEET + 1
            if self.meet_fixed == "best":
                _s, _l, meets = self.pod_options(pod)
                orders = sorted(self.pod_orders(pod, "drone"), key=lambda o: math.dist(o["xy"], pod["pos"]))[:n]
                mk = min(range(F.K_MEET), key=lambda k: (sum(math.dist(o["xy"], self.xy[meets[k]])
                                                             for o in orders), k))
            else:
                mk = int(self.meet_fixed)
            a = F.A_LAUNCH + (n - 1) * F.K_MEET + mk
        return super().pod_act(pod, a)

    def fly(self, a, b, height, payload):
        """The fleet world's flight, with this profile's energy use (every battery
        term there is proportional to the hover rate). / プロファイルの消費電力で飛行"""
        t, batt, crash = super().fly(a, b, height, payload)
        return t, batt * (self.hover_pct / F.HOVER_PCT_S), crash

    # -- measurements on flights / 飛行の計測
    def _flight(self, d, target, height, payload, kind, then):
        super()._flight(d, target, height, payload, kind, then)
        self.p1["energy_pct"] += d["pending_batt"]
        if kind in ("meet", "station", "return"):
            self.p1["empty_m"] += path_len(self, d["pos"], target)

    def _crash(self, d, why):
        if d.get("p1_drop_t") is not None:
            # lost between its drop and being ready again: a recovery failure / 回収失敗
            self.p1["fail"] += 1
            self.p1["fail_why"][why] = self.p1["fail_why"].get(why, 0) + 1
            d["p1_drop_t"] = None
        if d["state"] in (STN_CHARGE, STN_QUEUE):
            self._leave_station(d)
        super()._crash(d, why)

    # -- decisions / 判断
    def pending(self):
        p = super().pending()
        if p is not None and p[0] == "drone" and p[2] == F.D_AFTER and p[1].get("p1_drop_t") is None:
            p[1]["p1_drop_t"] = self.t                    # its parcel was just dropped / 投下時刻
        return p

    def options(self, d):
        """Recovery candidates: the pod (meeting point) and every charger; in v2 also
        ("pod", 1) = move the pod's meeting point to its next stop.
        回収先の候補：ポッド（合流地点）と各充電ステーション。v2 では「次の停車地で合流」も。"""
        pods = [("pod", 0), ("pod", 1)] if self.meet_choice else [("pod", 0)]
        return pods + [("station", i) for i in range(len(self.stations))]

    def _next_stop(self, pod):
        """The pod's next locker stop that is not already its meeting point (node), or
        None. / ポッドの次のロッカー停車地（合流地点以外）"""
        ovr = getattr(self, "next_stop_override", None)
        if ovr is not None:                     # the live pod's planned next stop (CARLA bridge) / 実機の次の停車地
            return ovr if ovr != pod["meet_node"] else None
        _s, lks, _m = self.pod_options(pod)
        for L in lks:
            if L is not None and self.lockers[L]["node"] != pod["meet_node"]:
                return self.lockers[L]["node"]
        return None

    def _redirect(self, d, pod):
        """Send a drone that is heading to / waiting at the pod to its new meeting
        point. A flight in progress pays for the part already flown; the rest of its
        energy is refunded. / 合流地点の変更：飛行中の機は飛んだ分だけ消費して向かい直す"""
        if d["state"] == F.FLY:
            t0, dur, a, b = d["leg"]
            f = min(1.0, max(0.0, (self.t - t0) / max(dur, 1e-6)))
            pend = d.pop("pending_batt", 0.0)
            d["batt"] -= pend * f
            self.p1["energy_pct"] -= pend * (1.0 - f)
            self.p1["empty_m"] -= path_len(self, a, b) * (1.0 - f)
            d["pos"] = a + (b - a) * f
            d.pop("pending_crash", None)
            d.pop("then", None)
        self._go_meet(d, pod)

    def pod_rule(self, pod):
        """The frozen pod rule (fleet_rules.rule_pod). v2 adds the charging fix: below
        POD_LOW with drones out, no more locker trips (gather them at the meeting
        point); below POD_LOW with all drones home, drive to the nearest charger.
        固定ポッドルール。v2 では低電池時にロッカー巡回をやめてドローンを集め、充電に行く。"""
        a = rule_pod(self, pod)
        if not self.pod_fix or pod["stranded"] or pod["charging"] is not None:
            return a
        # thresholds: POD_LOW in the practice world; the CARLA bridge sets the live pod's
        # own rules (keep working with drones out down to its critical level, charge
        # when idle below its low level) / しきい値（CARLA ブリッジは実機ポッドの規則を設定）
        low_out = getattr(self, "pod_low_out", None) or POD_LOW
        low_home = getattr(self, "pod_low_home", None) or POD_LOW
        out = [d for d in self.drones if d["pod"] == pod["id"] and d["state"] in (F.FLY, F.HOVER)]
        if pod["batt"] >= (low_out if out else low_home):
            return a
        if out:
            if F.A_LOCK <= a < F.A_GOMEET or F.A_SPOT <= a < F.A_LOCK:
                return F.A_GOMEET if self.pod_mask(pod)[F.A_GOMEET] else F.A_WAIT
            return a
        s = min(range(len(self.stations)), key=lambda i: self.route(pod["node"], self.stations[i]["node"]))
        return F.A_CHARGE + s

    def _pod_target(self, pod):
        return pod["meet"] if pod["meet"] is not None else pod["pos"]

    def _flight_est(self, a, b, height):
        """Deterministic time [s] and battery [%] for a flight (no wind draws).
        飛行時間と電池の決定論的見積り（乱数なし）。"""
        L = path_len(self, a, b)
        ws = float(np.mean(self.wind_hist)) if self.wind_hist else 0.0
        t = height / 3.0 * 2.0 + L / self.P["drone_speed"]
        batt = self.hover_pct * t * (1.0 + 0.5 * (ws / 10.0) ** 2) * 1.15
        return t, batt, L

    def estimate(self, d, opt):
        """What one recovery option would cost, from the state now: feasibility
        (with the reason), and the estimated time until the drone is ready
        again [s]. Used by B0 and logged for every method.
        回収先 1 つの見積り：実行可能か（理由付き）と、再び準備完了になるまでの時間。"""
        kind, i = opt
        h = d["height"]
        pod = self.pods[0]
        name = ("pod" if i == 0 else "pod_next") if kind == "pod" else "station%d" % i
        out = {"option": name, "feasible": True, "reason": ""}
        if kind == "pod":
            if pod["stranded"]:
                out.update(feasible=False, reason="pod stranded")
                return out
            if i == 1:
                n = self._next_stop(pod)
                if n is None:
                    out.update(feasible=False, reason="no next stop")
                    return out
                tgt = self.xy[n]
                t_fly, b_fly, L = self._flight_est(d["pos"], tgt, h)
                if pod["drive"] is not None:
                    pod_eta = pod["drive"]["t_left"] + self.route(pod["drive"]["to"], n) / self.pod_speed_eff()
                else:
                    pod_eta = self.route(pod["node"], n) / self.pod_speed_eff()
            else:
                tgt = self._pod_target(pod)
                t_fly, b_fly, L = self._flight_est(d["pos"], tgt, h)
                if pod["drive"] is not None:
                    pod_eta = pod["drive"]["t_left"]
                elif pod["meet_node"] is not None and pod["meet_node"] != pod["node"]:
                    pod_eta = self.route(pod["node"], pod["meet_node"]) / self.pod_speed_eff()
                else:
                    pod_eta = 0.0
            wait = max(0.0, pod_eta - t_fly)
            ahead = sum(1 for o in self.drones if o is not d and o["pod"] == pod["id"]
                        and (o["state"] == F.HOVER or (o["state"] == F.FLY and o.get("task") in ("meet", "return")
                                                        and o["eta"] < t_fly)))
            queue = ahead * F.LAND_S
            b_arr = d["batt"] - b_fly * EST_MARGIN - self.hover_pct * (wait + queue)
            charge = max(0.0, READY_PCT - b_arr) / self.P["charge_slow"]
            ready = t_fly + wait + queue + F.LAND_S + charge
            out.update(flight_s=t_fly, flight_m=L, wait_s=wait + queue, charge_s=charge, return_s=0.0,
                       batt_arrive=b_arr, ready_s=ready)
            if b_arr < RESERVE_PCT:
                out.update(feasible=False, reason="battery reserve")
            return out
        st = self.stations[i]
        t_fly, b_fly, L = self._flight_est(d["pos"], st["pads"][0], h)
        b_arr = d["batt"] - b_fly * EST_MARGIN
        if self.station_out(i) or self.station_out(i, self.t + t_fly):
            out.update(feasible=False, reason="station out of service")
        # expected wait for a deck pad: none free -> the soonest one to finish, plus
        # the queue ahead / 空きパッドが無ければ最短の充電終了＋待ち行列
        busy = [o for o in st["slots"] if o is not None]
        if len(busy) < len(st["slots"]) and not st["queue"]:
            wait = 0.0
        else:
            left = sorted(max(0.0, STATION_CHARGE_TO - self.drones[o]["batt"]) / self.P["charge_fast"]
                          for o in busy)
            per = (STATION_CHARGE_TO - 30.0) / self.P["charge_fast"]
            wait = (left[0] if left else 0.0) + len(st["queue"]) * per / max(1, len(st["slots"]))
            wait = max(0.0, wait - t_fly)
        charge = max(0.0, STATION_CHARGE_TO - max(b_arr, 0.0)) / self.P["charge_fast"]
        t_ret, _b_ret, L_ret = self._flight_est(st["pads"][0], self._pod_target(pod), h)
        ready = t_fly + wait + charge + t_ret + F.LAND_S
        out.update(flight_s=t_fly, flight_m=L, wait_s=wait, charge_s=charge, return_s=t_ret, return_m=L_ret,
                   batt_arrive=b_arr, ready_s=ready)
        if out["feasible"] and b_arr < RESERVE_PCT:
            out.update(feasible=False, reason="battery reserve")
        return out

    def recover(self, d, opt, by_method=True):
        """Carry out a recovery choice; ``by_method`` False for the frozen hover
        rule (not counted as a method decision). / 回収先の選択を実行する"""
        d["ask"] = None
        kind, i = opt
        if by_method:
            self.p1["recoveries"] += 1
            self.p1["to_pod" if kind == "pod" else "to_station"] += 1
        else:
            self.p1["hover_rule_station"] += 1
        pod = self.pods[0]
        d["home"] = pod["id"]
        if kind == "pod":
            d["pod"] = pod["id"]
            n = self._next_stop(pod) if (i == 1 and self.meet_choice) else None
            if n is not None:
                # move the pod's meeting point; drones already on their way follow it
                # 合流地点を移し、向かっている機も追従させる
                pod["meet_node"], pod["meet"] = n, self.xy[n].copy()
                if by_method:
                    self.p1["to_next"] = self.p1.get("to_next", 0) + 1
                for o in self.drones:
                    if o is not d and o["pod"] == pod["id"] and (
                            o["state"] == F.HOVER or (o["state"] == F.FLY and o.get("task") in ("meet", "return"))):
                        self._redirect(o, pod)
            self._go_meet(d, pod)
            return
        d["pod"] = None
        st = self.stations[i]

        def at_station(dd, i=i):
            self.k["station_visits"] += 1
            self.k["landing"].append(dd["batt"])
            self._join_station(dd, i)
        self._flight(d, st["pads"][0], d["height"], 0.0, "station", at_station)

    # -- stations: pads, queue, charge, fly back / ステーション
    def _join_station(self, d, i):
        st = self.stations[i]
        d["station"] = i
        d["leg"] = None
        if None in st["slots"]:
            st["slots"][st["slots"].index(None)] = d["id"]
            d["state"] = STN_CHARGE
        else:
            st["queue"].append(d["id"])
            d["state"] = STN_QUEUE
            self.p1["charger_conflicts"] += 1

    def _leave_station(self, d):
        i = d.get("station")
        if i is None:
            return
        st = self.stations[i]
        if d["id"] in st["slots"]:
            st["slots"][st["slots"].index(d["id"])] = None
            if st["queue"]:
                nxt = self.drones[st["queue"].pop(0)]
                st["slots"][st["slots"].index(None)] = nxt["id"]
                nxt["state"] = STN_CHARGE
        elif d["id"] in st["queue"]:
            st["queue"].remove(d["id"])
        d["station"] = None

    def _fly_home(self, d):
        pod = self.pods[d["home"]]
        self._leave_station(d)
        d["pod"] = pod["id"]

        def at_meet(dd):
            dd.update(state=F.HOVER, hover_t=0.0)
        self._flight(d, self._pod_target(pod), d["height"], 0.0, "return", at_meet)

    def _drone_step(self, d):
        if d["state"] == STN_CHARGE:
            if self.station_out(d["station"]):
                return                                           # no power during an outage / 停電中
            d["batt"] = min(100.0, d["batt"] + self.P["charge_fast"] * F.DT)
            if d["batt"] >= STATION_CHARGE_TO and not self.pods[d["home"]]["stranded"]:
                self._fly_home(d)
            return
        if d["state"] == STN_QUEUE:
            self.p1["wait_s"] += F.DT
            return
        if d["state"] == F.HOVER:
            # one landing at a time on the roof pad / 屋根パッドへは 1 機ずつ
            d["batt"] -= self.hover_pct * F.DT
            self.p1["energy_pct"] += self.hover_pct * F.DT
            d["hover_t"] += F.DT
            self.k["hover_s"] += F.DT
            self.p1["wait_s"] += F.DT
            pod = self.pods[d["pod"]] if d["pod"] is not None else None
            if pod is not None and pod["drive"] is None and math.dist(pod["pos"], d["pos"]) < 8.0:
                if self.t >= pod.get("pad_free_t", 0.0):
                    pod["pad_free_t"] = self.t + F.LAND_S
                    d["eta"] = F.LAND_S
                    self._land_in_pod(d, pod)
                elif not d.get("p1_pad_wait"):
                    d["p1_pad_wait"] = True
                    self.p1["pad_conflicts"] += 1
            elif d["batt"] <= 0:
                self._crash(d, "battery")
            return
        if d["state"] == F.REPAIR:
            # a repaired drone rejoins its pod (Paper 1 keeps four drones; the fleet
            # world's standby system is not part of it) / 修理後はポッドに戻る
            d["timer"] -= F.DT
            if d["timer"] <= 0:
                pod = self.pods[0]
                st = min(self.stations, key=lambda s_: math.dist(s_["xy"], pod["pos"]))
                d.update(state=F.STANDBY, batt=100.0, station=None, pos=st["pads"][0].copy(), home=pod["id"])
                self._fly_home(d)
            return
        super()._drone_step(d)

    def open_sky(self, node):
        """Launch safety is assumed in Paper 1 (launching is not the decision
        under test): no canopy take-off crashes. The random draw in pod_act
        still happens, so the world's random sequence is unchanged in shape.
        Paper 1 では離陸の安全を前提とする（樹冠による離陸墜落なし）。"""
        return 1.0

    def _land_in_pod(self, d, pod):
        d["p1_pad_wait"] = False
        super()._land_in_pod(d, pod)

    def _log(self):
        # ready check once per step (after charging) / 毎ステップ準備完了を確認
        for d in self.drones:
            if (d.get("p1_drop_t") is not None and d["state"] == F.DOCKED and d["pod"] is not None
                    and d["batt"] >= READY_PCT):
                self.p1["ready"].append(self.t - d["p1_drop_t"])
                self.p1["ready_events"].append((d["id"], d["p1_drop_t"], self.t))
                d["p1_drop_t"] = None
        super()._log()

    # -- results / 結果
    def p1_kpi(self):
        k, p = self.k, self.p1
        hours = max(self.t, 1.0) / 3600.0
        lead = np.array(k["lead"]) if k["lead"] else np.zeros(0)
        ready = np.array(p["ready"]) if p["ready"] else np.zeros(0)
        n_drone = k["delivered_drone"]
        n_del = n_drone + k["delivered_f2f"]              # every parcel (drone + pod) / 全配達
        out = {"sim_hours": round(hours, 3), "orders": self.oid, "delivered": n_del,
               "delivered_per_h": n_del / hours, "drone_per_h": n_drone / hours,
               "pod_per_h": k["delivered_f2f"] / hours,
               "lead_mean_s": float(lead.mean()) if lead.size else None,
               "lead_p95_s": float(np.percentile(lead, 95)) if lead.size else None,
               "ready_n": int(ready.size),
               "ready_time_mean_s": float(ready.mean()) if ready.size else None,
               "ready_time_p95_s": float(np.percentile(ready, 95)) if ready.size else None,
               "recoveries": p["recoveries"], "to_pod": p["to_pod"], "to_station": p["to_station"],
               "to_next": p.get("to_next", 0),
               "recovery_failures": p["fail"], "recovery_fail_why": dict(p["fail_why"]),
               "recovery_failure_rate": p["fail"] / max(p["recoveries"], 1),
               "pad_conflicts": p["pad_conflicts"], "charger_conflicts": p["charger_conflicts"],
               "wait_drone_min": p["wait_s"] / 60.0,
               "wh_per_parcel": p["energy_pct"] / 100.0 * self.P["drone_wh"] / max(n_drone, 1),
               "empty_return_km": p["empty_m"] / 1000.0,
               "crashes": k["crash"], "crash_why": dict(k["crash_why"]), "flights": k["flights"],
               "pod_stranded": k["stranded"], "waiting_orders_end": sum(o["state"] != "done" for o in
                                                                        self.orders.values()),
               "fallbacks": p["fallbacks"], "hover_rule_station": p["hover_rule_station"],
               "infeasible_picks": p.get("infeasible_picks", 0)}
        for s in ON_TIME_S:
            n = int((lead <= s).sum()) if lead.size else 0
            out["ontime%d_per_h" % int(s / 60)] = n / hours
        return out
