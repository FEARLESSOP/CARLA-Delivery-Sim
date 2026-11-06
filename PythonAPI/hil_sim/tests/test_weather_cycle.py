#!/usr/bin/env python3
"""Offline tests: the day/night cycle and weather (weather_cycle.py) and rain
grounding the delivery drones (delivery/station.DroneFleet). No server.

  ../../carla-venv/bin/python tests/test_weather_cycle.py

昼夜サイクル・天気と、雨によるドローンの墜落・離陸禁止のオフラインテスト。
"""

import os
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
sys.path.insert(0, HIL)

import carla   # noqa: F401,E402  (station -> backends imports it)
import weather_cycle as wc  # noqa: E402
from delivery.battery import Battery, BatteryModel  # noqa: E402
from delivery.backends import DELIVERING, TO_VAN, DOCKED, STANDBY  # noqa: E402
from delivery.station import DroneFleet  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print("  %s  %s%s" % ("PASS" if cond else "FAIL", name, (" | %s" % (detail,)) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


print("1. cycle self-test / 自己テスト")
try:
    wc.self_test()
    check("weather_cycle.self_test", True)
except AssertionError as exc:
    check("weather_cycle.self_test", False, repr(exc))

print("2. settings file / 設定ファイル")
d = tempfile.mkdtemp()
p = os.path.join(d, "weather_cycle.json")
s = wc.load_settings(p)
check("defaults: 5 min a day, day/night and dynamic weather on",
      s["day_minutes"] == 5.0 and s["day_night"] and s["dynamic"] and s["force"] == "auto")
s.update(day_minutes=0.1, rain_chance=3, force="snow", junk=1)
wc.save_settings(s, p)
s = wc.load_settings(p)
check("clamped and cleaned on load", s["day_minutes"] == 1.0 and s["rain_chance"] == 1.0
      and s["force"] == "auto" and "junk" not in s, s)

print("3. night is dark, noon is bright (Oita, today's date) / 夜と昼")
c = wc.Cycle(hours=0.0)
check("midnight sun below the horizon", c.status(wc.DEFAULTS)["night"])
c = wc.Cycle(hours=12.5)
check("noon sun above it", not c.status(wc.DEFAULTS)["night"])


class Unit(object):
    def __init__(self, i, status):
        self.id, self.status, self.van, self.pos = i, status, None, (0.0, 0.0, 30.0)
        self.battery = Battery(60.0)
        self.crash_detail, self.reason, self.updates, self.flight = "", None, 0, None

    def _crash(self, t, reason):
        self.status, self.reason = "falling", reason

    def update(self, t, dt, wind):
        self.updates += 1
        return []

    def collect_parcels(self, t):
        pass


print("4. rain grounds the drones / 雨でドローン停止")
log = []
units = [Unit(1, DELIVERING), Unit(2, TO_VAN), Unit(3, DOCKED), Unit(4, STANDBY)]
station = types.SimpleNamespace(pads=[], release=lambda u: None)
fleet = DroneFleet(units, station, BatteryModel(), None, lambda *a, **k: log.append((a, k)),
                   rain_ground=20.0)
fleet.rain = 10.0
fleet.step(0.0, 0.1, (0.0, 0.0))
check("light drizzle (10 < 20): nothing grounded, nothing falls",
      not fleet.grounded and all(u.reason is None for u in units))
fleet.rain = 35.0
check("rain: take-offs held, reason says rain", fleet.grounded and fleet.raining
      and fleet.ground_reason().startswith("rain"), fleet.ground_reason())
fleet.step(1.0, 0.1, (0.0, 0.0))
check("every drone in the air comes down (rain)", units[0].reason == "rain" and units[1].reason == "rain")
check("drones in a pod or at the station stay put", units[2].reason is None and units[3].reason is None)
crashes = [a for a, k in log if a[1] == "drone_crash"]
check("each fall is logged as a crash", len(crashes) == 2, crashes)
fleet.rain = 0.0
check("rain over: drones may fly again", not fleet.grounded)
fleet.rain_ground = 0.0
fleet.rain = 100.0
check("rain_ground 0: rain never grounds", not fleet.grounded)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
