"""Drone battery: drain in flight, charge in the van or at the station (no carla).

Levels are percent. Flying drains ``hover`` %/s, more with a parcel on the
hook, more when climbing, and more in wind (the motors work harder to hold
position: the extra grows with the square of the wind speed). Parked in a
van the drone trickle-charges; on a charge-station pad it fast-charges.

``trip_cost`` / ``transfer_cost`` estimate what a flight will use before it
starts, so a van can decide whether its drone can make the next delivery,
should wait and charge a little, or must be swapped.

ドローンのバッテリー（carla 非依存）。単位は %。飛行中は ``hover`` %/s を
消費し、荷物・上昇・風（風速の 2 乗で増える）で増加。車内では低速充電、
充電ステーションでは急速充電。飛行前に消費量を見積もり、次の配達が可能か・
少し充電して待つか・交換が必要かを判断する。
"""

import math


class BatteryModel(object):
    """Rates shared by every drone. / 全機共通のレート。"""

    def __init__(self, hover=0.25, payload=0.20, climb=0.12, wind_k=0.5, wind_ref=10.0,
                 slow=0.08, fast=1.5, reserve=15.0):
        self.hover = hover          # %/s level flight / 水平飛行 [%/s]
        self.payload = payload      # +20 % with a parcel / 荷物ありで +20 %
        self.climb = climb          # extra %/s while climbing / 上昇中の追加 [%/s]
        self.wind_k, self.wind_ref = wind_k, wind_ref
        self.slow = slow            # %/s in the van / 車内充電 [%/s]
        self.fast = fast            # %/s at the station / ステーション充電 [%/s]
        self.reserve = reserve      # never plan below this / 計画上の下限 [%]

    def rate(self, loaded=False, climbing=False, wind=0.0):
        """Drain in %/s for one flight condition."""
        r = self.hover * (1.0 + (self.payload if loaded else 0.0))
        r *= 1.0 + self.wind_k * (wind / self.wind_ref) ** 2
        return r + (self.climb if climbing else 0.0)

    def _leg(self, horiz, up, down, h_speed, v_speed, loaded, wind):
        t_flat = horiz / h_speed + down / v_speed
        t_up = up / v_speed
        return (t_flat * self.rate(loaded, False, wind) +
                t_up * self.rate(loaded, True, wind))

    def trip_cost(self, home, drop, cruise_z, h_speed=12.0, v_speed=4.0, hover_s=4.0,
                  drop_height=3.0, wind=0.0):
        """% used by one round trip home -> drop -> home at ``cruise_z``.

        Outbound with the parcel, back without. ``wind`` is the speed to plan
        for (the current wind plus a margin).
        往路は荷物あり、復路はなし。``wind`` は計画用の風速（現在＋余裕）。
        """
        d = math.hypot(drop[0] - home[0], drop[1] - home[1])
        hover_z = drop[2] + drop_height
        out = self._leg(d, cruise_z - home[2], cruise_z - hover_z, h_speed, v_speed,
                        True, wind)
        out += hover_s * self.rate(True, False, wind)
        back = self._leg(d, cruise_z - hover_z, cruise_z - home[2], h_speed, v_speed,
                         False, wind)
        return out + back

    def mission_cost(self, start, drop, meet, cruise_z, meet_z, h_speed=12.0, v_speed=4.0,
                     hover_s=4.0, drop_height=3.0, hold_s=60.0, wind=0.0):
        """% for a pod Mission: start -> drop (loaded), then empty to the
        meeting point, ``hold_s`` circling there, and down onto the pad.
        ポッドからの配達（合流地点へ戻る）に必要な %。"""
        d1 = math.hypot(drop[0] - start[0], drop[1] - start[1])
        hover_z = drop[2] + drop_height
        out = self._leg(d1, cruise_z - start[2], cruise_z - hover_z, h_speed, v_speed, True, wind)
        out += hover_s * self.rate(True, False, wind)
        d2 = math.hypot(meet[0] - drop[0], meet[1] - drop[1])
        back = self._leg(d2, meet_z - hover_z, meet_z - meet[2], h_speed, v_speed, False, wind)
        return out + back + hold_s * self.rate(False, False, wind)

    def transfer_cost(self, a, b, cruise_z, h_speed=12.0, v_speed=4.0, wind=0.0):
        """% used flying empty from ``a`` up to ``cruise_z``, across, down to ``b``."""
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        return self._leg(d, cruise_z - a[2], cruise_z - b[2], h_speed, v_speed, False, wind)

    def wait_to_reach(self, level, need, rate):
        """Seconds of charging at ``rate`` %/s to get from ``level`` to ``need``."""
        return max(need - level, 0.0) / rate if rate > 0 else float("inf")


class Battery(object):
    def __init__(self, level=100.0):
        self.level = float(level)

    @property
    def dead(self):
        return self.level <= 0.0

    def drain(self, dt, rate):
        self.level = max(self.level - rate * dt, 0.0)

    def charge(self, dt, rate):
        self.level = min(self.level + rate * dt, 100.0)


class PodEnergyModel(object):
    """The delivery pod's EV battery, in percent. / 配送ポッドの EV バッテリー [%]。

    Driving costs ``per_km``; the pod's own systems (computer, sensors,
    climate) cost ``idle`` %/s even when parked; charging its drone draws a
    little too (``drone_ratio`` pod-% per drone-%: the drone pack is tiny next
    to the pod's: ``drone_wh`` / ``pod_kwh``, plus charger losses; the battery
    magazine's packs too). At an EV charger it gains ``charge`` %/s.
    走行は ``per_km``、車載システムは停車中も ``idle`` %/s、ドローンへの充電も
    わずかに消費。EV 充電器では ``charge`` %/s 充電。
    """

    def __init__(self, per_km=5.0, idle=0.02, charge=1.0, low=30.0, critical=8.0,
                 target=95.0, reserve=10.0, drone_ratio=None, pod_kwh=10.0, drone_wh=500.0,
                 charger_eff=0.9):
        self.per_km, self.idle, self.charge = per_km, idle, charge
        self.low = low              # below this an idle pod goes to charge
        self.critical = critical    # below this a busy pod gives its work back
        self.target = target        # charge up to this / 充電目標
        self.reserve = reserve      # never plan a batch that ends below this
        # pod-% per drone-% charged from the pod: the two pack sizes and the
        # charger's losses / ドローン 1 % 充電あたりのポッド消費 %（容量比と損失）
        self.pod_kwh, self.drone_wh = pod_kwh, drone_wh
        self.drone_ratio = (drone_ratio if drone_ratio is not None else
                            drone_wh / (pod_kwh * 1000.0) / charger_eff)

    def cost(self, km, seconds):
        """% for driving ``km`` over ``seconds`` (systems included)."""
        return self.per_km * km + self.idle * seconds
