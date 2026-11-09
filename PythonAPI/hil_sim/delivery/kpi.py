"""KPI tracker for the LastMile pod and the drones (docs/DELIVERY_KPI.md).

Samples the fleet once a second (state times, energy, zone and no-fly checks)
and turns the run into the KPI block of summary.json (``summary["kpi"]``).
Mini Pods / hubs are not covered yet. It only reads the fleet; nothing here
changes what the vans or drones do. The RL baseline and every RL checkpoint
test are scored with these numbers.

ポッドとドローンの KPI を 1 秒ごとに集計し、summary.json の kpi に書く。
"""

import math

# van states that belong to a stop (arrival -> departure) / 停車中の状態
STOP_STATES = ("stopping", "jacks_down", "hatch_open", "docking", "handover", "drone_out",
               "hatch_close", "jacks_up", "side_open", "arm_out", "parcel_drop", "arm_in",
               "side_close")
CHARGE_STATES = ("ev_charging", "waiting_charger", "queued", "yard_in", "yard_out")
# drone statuses: in the air vs on the ground / 飛行中と地上
AIR = ("delivering", "to_station", "to_van", "falling")
GROUND = ("docked", "reloading", "charging", "standby")


def _pct(xs, q):
    if not xs:
        return None
    xs = sorted(xs)
    k = (len(xs) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def _mean(xs):
    return sum(xs) / len(xs) if xs else None


def _r(x, n=2):
    return None if x is None else round(x, n)


class KpiTracker(object):
    PERIOD = 1.0                 # sample every sim second / 1 秒ごと

    def __init__(self, fleet):
        self.fleet = fleet
        self.next_t = 0.0
        self.last_t = None
        self.vans = {}           # vid -> dict / 車ごと
        self.drones = {}         # drone id -> dict / 機体ごと
        self.weather_hold_s = 0.0
        self.ready_samples = []  # share of the fleet ready to fly / 出発可能な割合

    def _van(self, vid):
        return self.vans.setdefault(vid, {
            "state_s": {}, "state": None, "stop_t": None, "stops": 0, "stop_s": [],
            "level": None, "drain_pct": 0.0, "out_of_zone_working_s": 0.0,
            "transit_other_zone_s": 0.0})

    def _drone(self, uid):
        return self.drones.setdefault(uid, {
            "status": None, "air_t": None, "flights": [], "landing_battery": [],
            "nofly_samples": 0, "hold_s": 0.0})

    def sample(self, t):
        """Call every tick after fleet.step; does real work once a second."""
        if t < self.next_t:
            return
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t, self.next_t = t, t + self.PERIOD
        f = self.fleet
        zones = getattr(f, "zones", None)
        for v in f.vans:
            if getattr(v, "lmh", False):
                continue                                 # hubs / Mini Pods later / 後で
            k = self._van(v.vid)
            st = v.state
            k["state_s"][st] = k["state_s"].get(st, 0.0) + dt
            if st in STOP_STATES and k["state"] not in STOP_STATES:
                k["stops"] += 1
                k["stop_t"] = t
            if st not in STOP_STATES and k["state"] in STOP_STATES and k["stop_t"] is not None:
                k["stop_s"].append(t - k["stop_t"])
                k["stop_t"] = None
            k["state"] = st
            lvl = v.energy.level
            if k["level"] is not None and lvl < k["level"]:
                k["drain_pct"] += k["level"] - lvl
            k["level"] = lvl
            zone = zones.zone_of(v.vid) if zones is not None else None
            if zone is not None and dt > 0:
                try:
                    p = v.car.get_location()
                    inside = zone["x0"] <= p.x <= zone["x1"] and zone["y0"] <= p.y <= zone["y1"]
                except RuntimeError:
                    inside = True
                if not inside:
                    # transit to its zone or to a charger is allowed / 移動と充電は許可
                    if st in STOP_STATES:
                        k["out_of_zone_working_s"] += dt
                    else:
                        k["transit_other_zone_s"] += dt
        units = getattr(getattr(f.sky, "drones", None), "units", []) or []
        air = zones.air if zones is not None else None
        grounded = bool(getattr(f.sky.drones, "grounded", False)) if units else False
        if grounded:
            self.weather_hold_s += dt
        ready = 0
        for u in units:
            d = self._drone(u.id)
            s = u.status
            if s in AIR and d["status"] not in AIR:
                d["air_t"] = t
            if s in GROUND and d["status"] in AIR:
                if d["air_t"] is not None:
                    d["flights"].append(t - d["air_t"])
                d["landing_battery"].append(u.battery.level)
                d["air_t"] = None
            if s in AIR and air:
                try:
                    if air.blocked(u.pos, grow=0.0):
                        d["nofly_samples"] += 1
                except TypeError:
                    if air.blocked(u.pos):
                        d["nofly_samples"] += 1
            if s in ("docked", "standby") and u.battery.level > 50.0:
                ready += 1
            d["status"] = s
        if units:
            self.ready_samples.append(ready / float(len(units)))

    # -- the KPI block / KPI まとめ
    def result(self, t):
        f = self.fleet
        hours = max(t, 1e-6) / 3600.0
        pods = [v for v in f.vans if not getattr(v, "lmh", False)]
        pod_kwh = getattr(getattr(f.sky, "pem", None), "pod_kwh", 10.0)
        zones = getattr(f, "zones", None)

        def lead(kind):
            return [o.delivered_t - o.created_t for v in pods for o in v.delivered if o.kind == kind]

        f2f, drn = lead("face_to_face"), lead("drone")
        f2f_failed = sum(o.kind == "face_to_face" for v in pods for o in v.failed)
        drn_failed = sum(o.kind == "drone" for v in pods for o in v.failed)
        per_pod = []
        for v in pods:
            k = self._van(v.vid)
            busy = sum(s for st, s in k["state_s"].items() if st != "idle")
            total = sum(k["state_s"].values()) or 1e-6
            charging = sum(k["state_s"].get(st, 0.0) for st in CHARGE_STATES)
            n_del = sum(o.kind == "face_to_face" for o in v.delivered)
            kwh = k["drain_pct"] / 100.0 * pod_kwh
            per_pod.append({
                "pod": v.vid, "deliveries": n_del,
                "utilisation": _r(busy / total, 3), "charging_share": _r(charging / total, 3),
                "stops": k["stops"], "mean_stop_s": _r(_mean(k["stop_s"]), 1),
                "kwh_used": _r(kwh, 3), "kwh_per_delivery": _r(kwh / n_del, 3) if n_del else None,
                "km_per_delivery": _r(v.pod_km / n_del, 3) if n_del else None,
                "out_of_zone_working_s": _r(k["out_of_zone_working_s"], 1),
                "transit_other_zone_s": _r(k["transit_other_zone_s"], 1)})
        flights = [x for d in self.drones.values() for x in d["flights"]]
        land = [x for d in self.drones.values() for x in d["landing_battery"]]
        drops_out = 0
        if zones is not None:
            for v in pods:
                z = zones.zone_of(v.vid)
                if z is None:
                    continue
                for o in v.delivered:
                    if o.kind == "drone":
                        x, y = o.drop[0], o.drop[1]
                        if not (z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"]):
                            drops_out += 1
        ds = f.sky.drones.summary() if getattr(f.sky, "drones", None) is not None else {}
        trips = ds.get("trips", 0) or 0
        return {
            "sim_hours": _r(hours, 3),
            "pod": {
                "deliveries": len(f2f), "per_hour": _r(len(f2f) / hours, 2),
                "success_rate": _r(len(f2f) / (len(f2f) + f2f_failed), 3) if (f2f or f2f_failed) else None,
                "lead_time_mean_s": _r(_mean(f2f), 1), "lead_time_p95_s": _r(_pct(f2f, 0.95), 1),
                "stranded": sum(v.state == "stranded" for v in pods),
                "out_of_zone_working_s": _r(sum(p["out_of_zone_working_s"] for p in per_pod), 1),
                "per_pod": per_pod},
            "drone": {
                "deliveries": len(drn), "per_hour": _r(len(drn) / hours, 2),
                "success_rate": _r(len(drn) / (len(drn) + drn_failed), 3) if (drn or drn_failed) else None,
                "lead_time_mean_s": _r(_mean(drn), 1), "lead_time_p95_s": _r(_pct(drn, 0.95), 1),
                "flight_time_mean_s": _r(_mean(flights), 1),
                "landing_battery_min": _r(min(land) if land else None, 1),
                "landing_battery_mean": _r(_mean(land), 1),
                "crashes_per_100_trips": _r(100.0 * ds.get("crashes", 0) / trips, 2) if trips else None,
                "weather_hold_s": _r(self.weather_hold_s, 1),
                "fleet_ready_share": _r(_mean(self.ready_samples), 3),
                "nofly_breach_samples": sum(d["nofly_samples"] for d in self.drones.values()),
                "drops_outside_zone": drops_out},
        }
