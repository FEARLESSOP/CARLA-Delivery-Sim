"""Practice side of the Paper 1 CARLA bridge: rebuild one pod's situation from a
snapshot of the live simulation and ask a recovery method (B0-B5) where the drone
that just dropped its parcel should go. No CARLA import, so it is tested offline
(tests/test_paper1_bridge.py); the CARLA side is delivery/paper1_bridge.py.

Snapshot (CARLA coordinates: x east, y south, metres; times s; batteries %):

    {"t": sim seconds, "wind": [recent wind speeds m/s] (or one number),
     "pod": {"pos": [x, y], "batt": %, "driving_to": [x, y] | None, "eta_s": s | None,
             "meet": [x, y] | None, "next_stop": [x, y] | None,
             "rules": {"low": %, "critical": %} | None,           # the pod's own charging rules
             "rate_per_h": orders/h | None, "pod_share": 0-1 | None}, # its workload (new orders)
     "drones": [{"key": any, "state": "dropped" | "docked" | "deliver" | "return" | "hover" |
                 "to_station" | "charging" | "queue" | "down",
                 "pos": [x, y], "batt": %, "target": [x, y] (deliver), "station": i (station states)}],
     "orders": [{"xy": [x, y], "age_s": s, "kind": "drone" | "pod"}],   # waiting in / for the pod
     "stations": [{"xy": [x, y], "out": bool}]}                          # optional

Exactly one drone has state "dropped" (the decision). Up to 4 drones per pod are
used (Paper 1's hub); the practice world is the v2 world for the pod's zone.
Answer: {"kind": "pod" | "pod_next" | "station", "station": i | None, "xy": [x, y], "info": {...}}
("pod" = back to the pod's current meeting point, "pod_next" = meet the pod at its next stop).

Paper 1 の CARLA ブリッジ（練習世界側）：実機シミュレーションの 1 台のポッドの状況から練習世界を
組み立て、回収手法（B0〜B5）に投下直後のドローンの行き先を聞く。CARLA は読み込まない。
"""
import math
import os

import numpy as np

from rl_delivery import env_fleet as F
from rl_delivery.paper1 import world as W1

CKPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "checkpoints", "paper1")
STATE_ROOM_S = 2400.0          # keep the practice clock this far from its 3 h end / 練習世界の終了まで残す時間


def checkpoint_for(method, seed, world="v2"):
    """The trained network a method needs (B4 runs on B2), or None. / 必要な学習済みモデル"""
    if method not in ("B1", "B2", "B3", "B4"):
        return None
    net = "B2" if method == "B4" else method
    sub = ["v2"] if world == "v2" else []
    path = os.path.join(CKPT, *sub, net, "s%d" % int(seed), "final.zip")
    if not os.path.exists(path):
        raise FileNotFoundError("no checkpoint %s / チェックポイントがありません" % path)
    return path


class SnapshotPlanner(object):
    """One per pod: keeps the method and a practice world for that pod's zone.
    ポッドごと：手法とそのゾーンの練習世界を保持する。"""

    def __init__(self, method="B4", ckpt_seed=0, zone=1, world="v2", rate=None, scenario="S1_normal", seed=0):
        from rl_delivery.paper1.methods import make_method
        self.method_name, self.zone, self.world_name = method, int(zone), world
        self.method = make_method(method, checkpoint_for(method, ckpt_seed, world), world=world)
        self.w = W1.Paper1World(scenario=scenario, world=world, area=[self.zone], zone=self.zone, rate=rate)
        self.seed = int(seed)
        self.n = 0

    # -- building the world / 世界を組み立てる
    def _node(self, xy):
        w = self.w
        nodes = w.zone_nodes[w.zone_id]
        return min(nodes, key=lambda i: (w.xy[i][0] - xy[0]) ** 2 + (w.xy[i][1] - xy[1]) ** 2)

    def _station(self, xy):
        return min(range(len(self.w.stations)), key=lambda i: math.dist(self.w.stations[i]["xy"], xy))

    def build(self, snap):
        """The practice world set to this snapshot; returns (world, deciding drone).
        スナップショットに合わせた練習世界と判断するドローン。"""
        w = self.w
        self.n += 1
        # the live pod's workload: its order rate and the share it delivers itself
        # (the imagined futures spawn new orders at this rate) / 実機ポッドの仕事量
        ps0 = snap.get("pod", {})
        if ps0.get("rate_per_h"):
            w.rate_override = float(ps0["rate_per_h"])
        if ps0.get("pod_share") is not None:
            w.pod_share = float(ps0["pod_share"])
        w.reset(seed=self.seed * 100003 + self.n)
        t = float(np.clip(float(snap.get("t", 0.0)) % F.EP_S, 60.0, F.EP_S - STATE_ROOM_S))
        w.t = t
        wind = snap.get("wind", 0.0)
        w.wind_hist = [float(v) for v in (wind if isinstance(wind, (list, tuple)) else [wind])][-12:] or [0.0]
        burst = int(w.sc.get("burst", 1))
        for z in list(w.next_order):
            w.next_order[z] = t + burst * float(w.rng.uniform(*w.order_gap))
        pod = w.pods[0]
        ps = snap["pod"]
        pod["node"] = self._node(ps["pos"])
        pod["pos"] = np.array(ps["pos"][:2], float)
        pod["batt"] = float(ps.get("batt", 80.0))
        pod.update(drive=None, dwell=0.0, charging=None, stranded=False, busy_until=t, pad_free_t=0.0)
        meet = ps.get("meet")
        pod["meet_node"] = self._node(meet) if meet is not None else pod["node"]
        pod["meet"] = w.xy[pod["meet_node"]].copy()
        if ps.get("driving_to") is not None:
            w._drive(pod, self._node(ps["driving_to"]), "meet")
            if ps.get("eta_s") is not None:
                pod["drive"]["t_left"] = pod["drive"]["dur"] = max(5.0, float(ps["eta_s"]))
        nxt = ps.get("next_stop")
        w.next_stop_override = self._node(nxt) if nxt is not None else None
        rules = ps.get("rules") or {}                      # the live pod's charging rules / 実機の充電規則
        w.pod_low_out = rules.get("critical")
        w.pod_low_home = rules.get("low")
        # orders waiting in / for this pod / ポッドの待ち注文
        w.orders, w.oid = {}, 0
        lk = [i for i, L in enumerate(w.lockers) if L["zone"] in w.area]
        for o in snap.get("orders", []):
            xy = np.array(o["xy"][:2], float)
            oo = {"id": w.oid, "zone": w.zone_id, "kind": "drone", "xy": xy, "t0": t - float(o.get("age_s", 0.0)),
                  "state": "pod", "pod": 0, "locker": None, "site": None}
            if o.get("kind") == "pod" and lk:
                oo["kind"], oo["locker"] = "f2f", min(lk, key=lambda i: math.dist(w.lockers[i]["xy"], xy))
            w.orders[w.oid] = oo
            w.oid += 1
        for st in w.stations:
            st["slots"] = [None] * len(st["slots"])
            st["queue"] = []
        out = [self._station(s["xy"]) for s in snap.get("stations", []) if s.get("out")]
        if out:
            w.sc = dict(w.sc, outage=(t - 1.0, t + 4 * 3600.0))
            w.outage_station = out[0]
        # drones / ドローン
        deciding = None
        sd = list(snap.get("drones", []))[:len(w.drones)]
        for d, s in zip(w.drones, sd):
            d.update(batt=float(s.get("batt", 80.0)), pos=np.array(s["pos"][:2], float), pod=0, home=0, station=None,
                     parcel=None, ask=None, leg=None, task=None, hover_t=0.0, p1_drop_t=None, height=35.0)
            st = s.get("state", "docked")
            if st == "dropped":
                d["state"], d["p1_drop_t"] = F.HOVER, t
                deciding = d
            elif st == "docked":
                d["state"], d["pos"] = F.DOCKED, pod["pos"].copy()
            elif st == "deliver":
                o = {"id": w.oid, "zone": w.zone_id, "kind": "drone", "xy": np.array((s.get("target") or s["pos"])[:2], float),
                     "t0": t, "state": "drone", "pod": 0, "locker": None, "site": None}
                w.orders[w.oid] = o
                w.oid += 1
                d["parcel"] = o["id"]
                w.drone_act(d, F.D_LAUNCH, 1)
            elif st == "return":
                d["p1_drop_t"] = t
                w._go_meet(d, pod)
            elif st == "hover":
                d["state"], d["p1_drop_t"] = F.HOVER, t
            elif st == "to_station":
                d["p1_drop_t"] = t
                w.recover(d, ("station", self._station(s.get("station_xy") or s.get("target") or s["pos"])
                              if s.get("station") is None else int(s["station"])), by_method=False)
            elif st in ("charging", "queue"):
                i = int(s["station"]) if s.get("station") is not None else self._station(s["pos"])
                d["p1_drop_t"] = t
                d["pos"] = np.array(w.stations[i]["pads"][0], float)
                w._join_station(d, i)
            else:                                          # down, lost, unknown / 故障など
                d.update(state=F.DOWN, pod=None, home=0)
        for d in w.drones[len(sd):]:                       # fewer than 4 drones / 4 機未満
            d.update(state=F.DOWN, pod=None, ask=None)
        w.p1.update(recoveries=0, to_pod=0, to_station=0, fallbacks=0, hover_rule_station=0)
        if deciding is None:
            raise ValueError("snapshot has no drone with state 'dropped' / 判断するドローンがない")
        return w, deciding

    # -- the decision / 判断
    def decide(self, snap):
        w, d = self.build(snap)
        opt, info = self.method.decide(w, d)
        kind, i = opt
        if kind == "pod":
            nxt = w._next_stop(w.pods[0]) if i == 1 else None
            if i == 1 and nxt is not None:
                return {"kind": "pod_next", "station": None, "xy": [float(v) for v in w.xy[nxt]], "info": info}
            return {"kind": "pod", "station": None, "xy": [float(v) for v in w._pod_target(w.pods[0])], "info": info}
        st = w.stations[i]
        return {"kind": "station", "station": int(i), "xy": [float(v) for v in st["pads"][0]], "info": info}
