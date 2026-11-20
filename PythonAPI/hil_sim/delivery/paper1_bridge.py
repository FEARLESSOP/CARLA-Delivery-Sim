"""Paper 1 recovery methods (B0-B5) inside the CARLA delivery simulation.

At each parcel drop of a LastMile pod's drone (DroneUnit "delivered" event,
delivery/backends.py), the bridge snapshots that pod's situation (the pod, its four
drones, waiting orders, the drones' meeting point and the pod's next stop, wind),
sends it to a worker process that rebuilds a practice world and asks the chosen
method (rl_delivery/paper1/snapshot.py), and applies the answer when it arrives:

- "pod"      : nothing changes (the drone flies to the pod's meeting point as usual);
- "pod_next" : the drone's meeting point moves to the pod's next stop (Mission.meet),
               and the pod's list of drones to receive moves with it (van.meets);
- "station"  : a free pad at the chosen charging station is reserved and the drone
               flies there (like DroneFleet.send_to_station, but to that station).

Applied one tick after the drop, so the delivery is recorded first (the old hook
sent drones away inside the event loop and the order was never finished). Answers
come from a separate process, so B4 / B5's rollouts never stall the CARLA loop; an
answer is skipped when it comes too late, the drone cannot reach the target with
its real battery, or the target is gone. Every decision is logged.
No CARLA import here: tests/test_paper1_bridge.py drives it with stand-in objects.

Paper 1 の回収手法（B0〜B5）を CARLA の配達シミュレーションで動かす。投下時にポッドの状況を
スナップショットにし、別プロセスで練習世界を組み立てて手法に聞き、次の tick 以降に答えを反映する
（ポッドへ・次の停車地で合流・指定の充電所へ）。届かない・遅すぎる答えは使わない。
"""
import math
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor

DELIVERING = "delivering"                              # delivery/backends.py status
DRIVING = "driving"                                    # delivery_sim.py Van.state
FACE_TO_FACE, CHARGE = "face_to_face", "charge"        # delivery/planning.py stop kinds
AFTER_DROP = ("ascend", "to_meet", "hold")             # Mission states after the drop
TIMEOUT_S = 25.0                                       # answers later than this are dropped (sim s)
METHODS = ("B0", "POD", "B1", "B2", "B3", "B4", "B5")

_planners = {}                                         # worker process: one planner per zone


def _decide(job):
    """Worker process: rebuild the practice world and ask the method. / 別プロセスで判断"""
    import warnings
    warnings.filterwarnings("ignore")
    from rl_delivery.paper1.snapshot import SnapshotPlanner
    key = (job["method"], job["seed"], job["zone"])
    if key not in _planners:
        _planners[key] = SnapshotPlanner(job["method"], job["seed"], zone=job["zone"])
    ans = _planners[key].decide(job["snap"])
    info = ans.pop("info", {}) or {}
    ans["predicted_ready_s"] = info.get("predicted_ready_s")
    ans["q"] = info.get("q")
    ans["rollout_ms"] = info.get("rollout_ms")
    # what the method saw, for diagnosis (paper1_decisions.json) / 診断用に判断材料も返す
    ans["seen"] = {k: info.get(k) for k in ("candidates", "mask", "mask_reasons", "score_ready_s", "prior",
                                            "rollout_set", "policy_choice", "agrees_with_policy", "timed_out")}
    return ans


def _warm(method, seed, zones):
    """Worker process: build every pod zone's practice world before the first drop.
    最初の投下の前に各ゾーンの練習世界を作っておく。"""
    from rl_delivery.paper1.snapshot import SnapshotPlanner
    for z in zones:
        if (method, seed, z) not in _planners:
            _planners[(method, seed, z)] = SnapshotPlanner(method, seed, zone=z)
    return len(_planners)


def _xy(p):
    return [float(p[0]), float(p[1])]


class Paper1RecoveryBridge(object):
    """One per delivery_sim run; DroneUnit calls on_drop() and poll(). / 実行ごとに 1 つ"""
    paper1 = True

    def __init__(self, method="B4", ckpt_seed=0, log=print, workers=1):
        if method not in METHODS:
            raise ValueError("unknown Paper 1 method %r / 不明な手法" % method)
        self.method, self.seed, self.log = method, int(ckpt_seed), log
        self.pool = ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn"))
        self.pool.submit(_warm, self.method, self.seed, (1, 2, 3))      # load while CARLA starts / 起動中に準備
        self.history = []
        self.counts = {"asked": 0, "pod": 0, "pod_next": 0, "station": 0, "skipped": 0, "failed": 0}

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)

    # -- snapshot of one pod / ポッドのスナップショット
    @staticmethod
    def zone_of(van):
        vid = int(getattr(van, "vid", 1) or 1)
        return vid if vid in (1, 2, 3) else 1

    @staticmethod
    def meet_stop(van, unit):
        for m in getattr(van, "meets", {}).values():
            if unit in m["units"]:
                return m["stop"]
        return None

    @staticmethod
    def next_stop(van, stop):
        stops = list(getattr(van, "stops", []) or [])
        if stop is None or stop not in stops:
            return None
        i = stops.index(stop)
        nxt = stops[i + 1] if i + 1 < len(stops) else None
        if nxt is None or nxt.kind == CHARGE:
            return None
        zs = getattr(getattr(van, "fleet", None), "zones", None)
        if zs is not None and getattr(zs, "air", None) is not None and zs.air.blocked(nxt.location):
            return None                                 # never wait over a no-fly zone / 区域上では待たない
        return nxt

    @staticmethod
    def rate_of(van, t):
        n = len(getattr(van, "delivered", []) or []) + len(getattr(van, "failed", []) or []) + \
            len(getattr(van, "pending", []) or [])
        if t < 600.0 or n == 0:
            return 20.0
        return max(5.0, min(40.0, n / (t / 3600.0)))

    def snapshot(self, unit, van, t, wind):
        stop = self.meet_stop(van, unit)
        nxt = self.next_stop(van, stop)
        car = van.car_pos()
        driving = getattr(van, "state", None) == DRIVING and getattr(van, "stops", None)
        drv = van.stops[0].location if driving else None
        drones = []
        for u in [x for x in getattr(van, "lmp_units", []) if x is not None]:
            s = {"key": u.id, "pos": _xy(u.world_pos() if hasattr(u, "world_pos") else u.pos),
                 "batt": float(u.battery.level)}
            f = getattr(u, "flight", None)
            fs = getattr(f, "state", None)
            if u is unit:
                s["state"] = "dropped"
            elif u.status in ("docked", "reloading"):
                s["state"] = "docked"
            elif u.status == DELIVERING and fs in ("takeoff", "cruise", "descend", "drop"):
                s["state"] = "deliver"
                s["target"] = _xy(u.order.drop) if getattr(u, "order", None) is not None else s["pos"]
            elif u.status == DELIVERING and fs == "hold":
                s["state"] = "hover"
            elif u.status in (DELIVERING, "to_van"):
                s["state"] = "return"
            else:
                s["state"] = "down"
            drones.append(s)
        launched = getattr(van, "launched", set())
        orders = []
        for o in getattr(van, "pending", []) or []:
            if o.id in launched:
                continue
            drop = getattr(o, "drop", None) or getattr(getattr(o, "building", None), "drop", None)
            if drop is None:
                continue
            orders.append({"xy": _xy(drop), "age_s": max(0.0, t - float(getattr(o, "created_t", t))),
                           "kind": "drone" if o.kind == "drone" else "pod"})
        return {"t": float(t), "wind": [math.hypot(float(wind[0]), float(wind[1]))] if len(wind) >= 2 else [0.0],
                "pod": {"pos": _xy(car), "batt": float(getattr(getattr(van, "energy", None), "level", 80.0)),
                        "driving_to": _xy(drv) if drv is not None else None,
                        "eta_s": (math.dist(_xy(car), _xy(drv)) * 1.3 / 8.0) if drv is not None else None,
                        "meet": _xy(stop.location) if stop is not None else None,
                        "next_stop": _xy(nxt.location) if nxt is not None else None,
                        # the pod's own charging rules (PodEnergyModel): it keeps working
                        # with drones out down to "critical", charges when idle below "low"
                        # ポッド自身の充電規則
                        "rules": {"low": float(getattr(getattr(van, "pem", None), "low", 30.0)),
                                  "critical": float(getattr(getattr(van, "pem", None), "critical", 8.0))},
                        # its workload: orders it received per hour so far (20/h for the first
                        # 10 min) and the share it delivers itself / このポッドの仕事量
                        "rate_per_h": self.rate_of(van, t),
                        "pod_share": 1.0 - float(getattr(getattr(van, "args", None), "drone_prob", 0.5))},
                "drones": drones, "orders": orders}

    # -- the hooks / フック
    def on_drop(self, unit, t, wind):
        """Called at the "delivered" event; returns a pending job (or None).
        投下時に呼ばれ、判断待ちの仕事を返す。"""
        van = unit.van
        if van is None or not getattr(van, "lmp", False) or self.meet_stop(van, unit) is None:
            return None
        snap = self.snapshot(unit, van, t, wind)
        self.counts["asked"] += 1
        fut = self.pool.submit(_decide, {"method": self.method, "seed": self.seed, "zone": self.zone_of(van),
                                         "snap": snap})
        return {"future": fut, "t": float(t), "van": van, "snap": snap}

    def poll(self, unit, job, t):
        """Called every tick while a job is pending; True when finished (applied,
        skipped or failed). / 毎 tick 呼ばれ、終われば True。"""
        fut = job["future"]
        if not fut.done():
            if t - job["t"] > TIMEOUT_S:
                fut.cancel()
                self._skip(unit, job, t, "no answer in %.0f s" % TIMEOUT_S)
                return True
            return False
        try:
            ans = fut.result()
        except Exception as e:                           # keep the default behaviour / 既定の動作のまま
            self.counts["failed"] += 1
            self._note(job["van"], unit, t, "decision failed, the sim's rule is kept: %s" % str(e)[:160])
            return True
        self.apply(unit, job, ans, t)
        return True

    def _note(self, van, unit, t, text):
        """Every decision in the delivery log (events.csv) and the console. / 全判断を記録"""
        msg = "D%s: %s [Paper 1 %s]" % (unit.id, text, self.method)
        try:
            van.log(t, "paper1_decision", pos=unit.pos, detail=msg)
        except Exception:
            pass
        self.log("[paper1] t=%.0f pod %s %s" % (t, getattr(van, "vid", "?"), msg))

    def _skip(self, unit, job, t, why, ans=None):
        self.counts["skipped"] += 1
        self.history.append({"t": t, "drone": unit.id, "van": getattr(job["van"], "vid", None),
                             "choice": ans["kind"] if ans else None, "applied": False, "why": why,
                             "snapshot": job.get("snap"), "seen": (ans or {}).get("seen"), "q": (ans or {}).get("q")})
        self._note(job["van"], unit, t, "chose %s, not used (%s)" % (ans["kind"] if ans else "nothing", why))

    def apply(self, unit, job, ans, t):
        van = job["van"]
        f = getattr(unit, "flight", None)
        rec = {"t": t, "drone": unit.id, "van": getattr(van, "vid", None), "choice": ans["kind"],
               "station": ans.get("station"), "delay_s": round(t - job["t"], 2),
               "predicted_ready_s": ans.get("predicted_ready_s"), "applied": False,
               "q": ans.get("q"), "seen": ans.get("seen"), "snapshot": job.get("snap")}
        if unit.van is not van or unit.status != DELIVERING or getattr(f, "state", None) not in AFTER_DROP:
            return self._skip(unit, job, t, "drone moved on (%s)" % getattr(f, "state", unit.status), ans)
        if ans["kind"] == "pod":
            rec["applied"] = True
        elif ans["kind"] == "pod_next":
            rec["applied"] = self._to_next(unit, van, t)
        elif ans["kind"] == "station":
            rec["applied"] = self._to_station(unit, van, ans["xy"], t)
        if not rec["applied"]:
            return self._skip(unit, job, t, "not reachable now", ans)
        self.counts[ans["kind"]] += 1
        self.history.append(rec)
        pr = ans.get("predicted_ready_s")
        where = {"pod": "back to the pod's meeting point", "pod_next": "meet the pod at its next stop",
                 "station": "charge at station %s" % ans.get("station")}[ans["kind"]]
        self._note(van, unit, t, "%s (decided in %.1f s%s)" % (
            where, rec["delay_s"], ", predicted ready in %.0f s" % pr if pr == pr and pr is not None else ""))

    # -- carrying out a choice / 選択の実行
    def _to_next(self, unit, van, t):
        stop = self.meet_stop(van, unit)
        nxt = self.next_stop(van, stop)
        fleet = van.sky.drones
        if nxt is None:
            return False
        meet = tuple(nxt.location)
        src = unit.world_pos()
        z = van.sky.check.flight_z(src, meet)
        cost = fleet.bm.transfer_cost(src, meet, z, unit.h_speed, unit.v_speed, fleet.plan_wind())
        if unit.battery.level < cost + 15.0:            # reach it and still wait a while / 待つ余裕を残す
            return False
        unit.flight.meet = meet
        unit.flight.meet_z = max(z, meet[2] + 15.0)
        for m in van.meets.values():
            if unit in m["units"]:
                m["units"].remove(unit)
        van.meets.setdefault(id(nxt), {"stop": nxt, "units": []})["units"].append(unit)
        van.log(t, "drone_recovery", pos=unit.pos,
                detail="D%d meets the pod at its next stop (%s) [Paper 1 %s]" % (unit.id, nxt.kind, self.method))
        return True

    def _to_station(self, unit, van, xy, t):
        fleet = van.sky.drones
        if getattr(fleet, "grounded", False):
            return False
        pad = fleet._free_pad((xy[0], xy[1], 0.0))
        if pad is None:
            return False
        src = unit.world_pos()
        dst = fleet.station.pads[pad]
        z = fleet.cruise_z(src, dst)
        cost = fleet.bm.transfer_cost(src, dst, z, unit.h_speed, unit.v_speed, fleet.plan_wind())
        if unit.battery.level < cost + 5.0:             # land with 5 % to spare (as the sim does) / 5 % 残して着陸
            return False
        fleet.station.taken[pad] = unit.id
        unit.pad_target = pad
        unit.start_transfer(lambda: dst, z)
        van.release_drone(unit)
        for m in van.meets.values():                    # the pod must not wait for it / ポッドが待たないように
            if unit in m["units"]:
                m["units"].remove(unit)
        fleet.charge_trips = getattr(fleet, "charge_trips", 0) + 1
        fleet.log(t, "drone_to_station", van=van.vid, pos=src,
                  detail="D%d at %.0f%% to charge at pad %d [Paper 1 %s]" % (unit.id, unit.battery.level, pad + 1, self.method))
        return True

    def summary(self):
        return dict(self.counts, method=self.method, seed=self.seed)
