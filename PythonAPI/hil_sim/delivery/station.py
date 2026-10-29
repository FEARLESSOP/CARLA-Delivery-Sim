"""The charge station and the pool of drones (one per van plus standbys).

Drones are not tied to a van for good. Each van starts with one; the rest
wait on charge-station pads as standbys. When a van's drone can't make its
next delivery and charging in the van would take too long, the drone flies
to the station and the van asks for a standby. The fleet sends the station
drone with the most charge that can reach the van and still make the
delivery; a drone that went to charge becomes a standby itself once it has
charge again, ready for whichever van needs one next ("swap back when
needed"). Crashed drones are recovered after a while and return to the
station with a part-charged battery.

充電ステーションとドローンの共用プール（車ごとに 1 機＋待機機）。車の機体が
次の配達に足りず、車内充電では時間がかかりすぎるとき、機体はステーションへ
飛び、車は待機機を要請する。プールは車まで飛べて配達もこなせる最も充電の
多い待機機を送る。充電に行った機体は充電後に待機機となる。墜落機は一定時間後
に回収され、一部充電の状態でステーションに戻る。
"""

import math

from delivery.backends import (IN_VAN, AT_STATION, CHARGING, STANDBY, CRASHED,
                               TO_STATION, TO_VAN, DELIVERING)


class ChargeStation(object):
    """Landing pads (world x, y, z) where drones fast-charge. / 急速充電パッド。"""

    def __init__(self, pads):
        self.pads = [tuple(p) for p in pads]
        self.taken = {}             # pad index -> drone id (parked or on its way)

    @property
    def location(self):
        n = float(len(self.pads))
        return tuple(sum(p[i] for p in self.pads) / n for i in range(3))

    def free_pad(self):
        return next((i for i in range(len(self.pads)) if i not in self.taken), None)

    def release(self, unit):
        for i, uid in list(self.taken.items()):
            if uid == unit.id:
                del self.taken[i]


class EvChargers(object):
    """Road-side EV chargers for the pods, one pod per charger.

    ``spots`` are road points (x, y, z). ポッド用の路側 EV 充電器（1 基 1 台）。
    """

    def __init__(self, spots):
        self.spots = [tuple(p) for p in spots]
        self.taken = {}             # charger index -> van id

    def claim(self, van, near=None):
        """A free charger for ``van`` (the nearest to ``near``), or None."""
        free = [i for i in range(len(self.spots)) if i not in self.taken]
        if not free:
            return None
        if near is not None:
            free.sort(key=lambda i: math.hypot(self.spots[i][0] - near[0],
                                               self.spots[i][1] - near[1]))
        self.taken[free[0]] = van.vid
        return free[0]

    def release(self, van):
        for i, vid in list(self.taken.items()):
            if vid == van.vid:
                del self.taken[i]


class DroneFleet(object):
    """Charging, swaps, standby dispatch and recovery for every drone.

    ``cruise_z(a, b)`` gives a safe height for a straight flight a -> b; ``log``
    is the run's event log. 全機の充電・交換・待機機派遣・回収を管理する。
    """

    def __init__(self, units, station, battery_model, cruise_z, log, recover_s=90.0,
                 repair_level=50.0, wind_margin=3.0, wind_hold=10.0, pod_draw=0.02):
        self.units, self.station, self.bm = units, station, battery_model
        self.cruise_z, self.log = cruise_z, log
        self.recover_s, self.repair_level = recover_s, repair_level
        self.wind_margin = wind_margin
        self.wind_hold = wind_hold  # no take-offs above this wind (0 = never hold)
        self.pod_draw = pod_draw    # pod-% used per drone-% charged in a pod
        self.requests = {}          # van -> % it needs for its next delivery
        self.swaps = self.charge_trips = self.repairs = 0
        self._wind = 0.0

    # -- setup / 初期配置
    def start(self, vans):
        """One drone per van (four per LastMile pod, on its deck); the rest
        wait at the station. / 車ごとに 1 機（LastMile ポッドは 4 機）、残りは待機。"""
        pool = iter(self.units)
        used = 0
        for van in vans:
            for _ in range(4 if getattr(van, "lmp", False) else 1):
                u = next(pool, None)
                if u is None:
                    break
                van.accept_drone(u)
                used += 1
        for u in self.units[used:]:
            pad = self.station.free_pad()
            self.station.taken[pad] = u.id
            u.park(pad, self.station.pads[pad], STANDBY)

    @property
    def grounded(self):
        """Wind too strong to send a drone up (it would be blown into things).

        Drones already in the air carry on; nothing new takes off.
        強風で新たな離陸を禁止（飛行中の機体はそのまま）。
        """
        return self.wind_hold > 0 and self._wind > self.wind_hold

    def plan_wind(self):
        """Wind speed to plan energy for: what blows now, plus a margin."""
        return self._wind + self.wind_margin

    # -- requests from vans / 車からの要請
    def request(self, van, need):
        self.requests[van] = need

    def cancel(self, van):
        self.requests.pop(van, None)

    def send_to_station(self, unit, t):
        """Fly ``unit`` to a free pad if it can make it. / 届くならステーションへ。"""
        pad = self.station.free_pad()
        if pad is None or self.grounded:
            return False
        src, dst = unit.world_pos(), self.station.pads[pad]
        z = self.cruise_z(src, dst)
        cost = self.bm.transfer_cost(src, dst, z, unit.h_speed, unit.v_speed,
                                     self.plan_wind())
        if unit.battery.level < cost + 5.0:     # land with 5 % to spare
            return False
        van = unit.van
        self.station.taken[pad] = unit.id
        unit.pad_target = pad
        unit.start_transfer(lambda: dst, z)
        if van is not None:
            van.release_drone(unit)
        self.charge_trips += 1
        self.log(t, "drone_to_station", van=van.vid if van else None,
                 pos=src, detail="D%d at %.0f%%, needs %.0f%% to get there"
                 % (unit.id, unit.battery.level, cost))
        return True

    def recall(self, unit, t):
        """A drone flying to a van that no longer wants it goes back to charge."""
        if unit.status == TO_VAN and not self.send_to_station(unit, t):
            unit.note = "no free pad"

    # -- per tick / 毎 tick
    def step(self, t, dt, wind):
        self._wind = math.hypot(*wind)
        for u in self.units:
            if u.status in IN_VAN:
                # trickle-charged from the pod's own battery (not from a flat one)
                # ポッドの電池から低速充電（ポッドが空なら充電しない）
                pod = getattr(u.van, "energy", None)
                if u.battery.level < 100.0 and (pod is None or pod.level > 1.0):
                    u.battery.charge(dt, self.bm.slow)
                    if pod is not None:
                        pod.drain(dt, self.bm.slow * self.pod_draw)
            elif u.status in AT_STATION:
                u.battery.charge(dt, self.bm.fast)
                u.status = STANDBY if u.battery.level >= 99.5 else CHARGING
            elif u.status == CRASHED and t - u.down_t > self.recover_s:
                self._repair(u, t)
            elif u.status == DELIVERING and getattr(u.flight, "holding", False):
                self._check_hold(u, t)
            was = u.status
            for ev in u.update(t, dt, wind):
                self._route(u, was, ev, t)
            u.collect_parcels(t)
        self._serve(t)

    def _check_hold(self, u, t):
        """A drone circling its meeting point goes to charge before it can't.
        合流地点で待つ機体は、充電所へ飛べなくなる前に向かう。"""
        pad = self.station.free_pad()
        if pad is None:
            return
        dst = self.station.pads[pad]
        cost = self.bm.transfer_cost(u.pos, dst, self.cruise_z(u.pos, dst), u.h_speed,
                                     u.v_speed, self.plan_wind())
        if u.battery.level < cost + 8.0:
            van, waited = u.van, u.flight.hold_s
            if self.send_to_station(u, t):
                self.log(t, "drone_divert", van=van.vid if van else None, pos=u.pos,
                         detail="D%d waited %.0f s at the meeting point, %.0f%% left: to charge"
                         % (u.id, waited, u.battery.level))

    def _route(self, u, was, ev, t):
        kind, data = ev
        if kind == "arrived" and was == TO_STATION:
            pad = u.pad_target
            u.park(pad, self.station.pads[pad])
            self.log(t, "drone_charging", pos=u.pos,
                     detail="D%d on pad %d at %.0f%%" % (u.id, pad + 1, u.battery.level))
        elif kind == "arrived" and was == TO_VAN:
            van = u.van
            van.accept_drone(u)
            van.incoming = None
            self.swaps += 1
            self.log(t, "drone_arrived", van=van.vid, pos=van.roof(),
                     detail="D%d at %.0f%%" % (u.id, u.battery.level))
        elif kind == "crashed":
            if was == TO_STATION:
                self.station.release(u)
            self.log(t, "drone_crash", van=u.van.vid if u.van else None, pos=u.pos,
                     detail="D%d: %s (battery %.0f%%, wind %.1f m/s%s)"
                     % (u.id, data, u.battery.level, self._wind,
                        ", " + u.crash_detail if data.startswith("collision") else ""))
            if u.van is not None:
                u.van.on_drone_event(t, u, was, ev)
        elif u.van is not None:
            u.van.on_drone_event(t, u, was, ev)

    def _serve(self, t):
        """Send standbys to vans that are waiting for one. / 待機機を派遣。"""
        if self.grounded:
            return
        for van, need in list(self.requests.items()):
            if van.drone is not None or van.incoming is not None:
                del self.requests[van]
                continue
            dest = van.dock_pos()
            best = None
            for u in self.units:
                if u.status not in AT_STATION:
                    continue
                z = self.cruise_z(u.pos, dest)
                cost = self.bm.transfer_cost(u.pos, dest, z, u.h_speed, u.v_speed,
                                             self.plan_wind())
                if u.battery.level >= need + cost and (
                        best is None or u.battery.level > best[0].battery.level):
                    best = (u, z)
            if best is None:
                continue
            u, z = best
            self.station.release(u)
            u.start_transfer(van.dock_pos, z, to_van=van)
            van.incoming = u
            del self.requests[van]
            self.log(t, "standby_sent", van=van.vid, pos=u.pos,
                     detail="D%d at %.0f%% (van needs %.0f%%)" % (u.id, u.battery.level, need))

    def _repair(self, u, t):
        pad = self.station.free_pad()
        if pad is None:
            return
        self.station.taken[pad] = u.id
        reason = u.fail_reason
        u.repair(pad, self.station.pads[pad], self.repair_level)
        self.repairs += 1
        self.log(t, "drone_repaired", pos=u.pos,
                 detail="D%d back at the station after %s, battery %.0f%%"
                 % (u.id, reason, u.battery.level))

    # -- reporting / 集計
    def counts(self):
        c = {}
        for u in self.units:
            c[u.status] = c.get(u.status, 0) + 1
        return c

    def summary(self):
        return {"drones": len(self.units), "swaps": self.swaps,
                "charge_trips": self.charge_trips, "repairs": self.repairs,
                "crashes": sum(u.crashes for u in self.units),
                "trips": sum(u.trips for u in self.units),
                "drone_km": round(sum(u.km for u in self.units), 3),
                "status": self.counts()}

