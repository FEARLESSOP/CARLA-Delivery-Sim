#!/usr/bin/env python3
"""The Drones page's sensor suite (delivery/drone_sensors.py), its damage model
and its glue in delivery/backends.py (budgeted raycasts, the snapshot of a
DroneUnit).
No CARLA server.

  ../../carla-venv/bin/python tests/test_drone_sensors.py

ドローンページのセンサー計算の試験（サーバー不要）。
"""
import math
import os
import sys
import time
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

from delivery import drone_sensors as ds  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


print("=== geo reference / 地理参照 ===")
# OitaKosen as CARLA 0.9.16 reports it (+y is north on this map)
geo = ds.GeoRef((33.234355, 131.651875, 0.0), (33.2343545382023, 131.66261909661046, 0.0),
                (33.243341747540214, 131.651875, 0.0))
lat, lon = geo.latlon(1000.0, 0.0)
check("affine map reproduces CARLA's sample", abs(lon - 131.66261909661046) < 1e-9 and abs(lat - 33.2343545) < 1e-7)
check("+x is east: heading 90", abs(geo.heading(0.0) - 90.0) < 0.5, geo.heading(0.0))
check("+y is north on this map: heading 0", min(geo.heading(90.0), 360 - geo.heading(90.0)) < 0.5, geo.heading(90.0))
fb = ds.GeoRef((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
check("no geo reference: Oita fallback", fb.fallback and abs(fb.lat0 - 33.2384) < 1e-6)
check("fallback keeps CARLA's +y = south", abs(fb.heading(90.0) - 180.0) < 0.5, fb.heading(90.0))


def snap(**kw):
    s = {"status": "delivering", "phase": "cruise", "pos": (0.0, 0.0, 60.0), "yaw": 0.0,
         "battery": 80.0, "drain": 0.3, "reserve": 15.0, "loaded": True, "order": 7,
         "wind": (3.0, 0.0), "w_est": (2.5, 0.0), "drift": 0.4, "saturated": False,
         "max_wind": 11.0, "ground_z": 0.0, "obstacles": {"front": None, "rear": None},
         "home": (-300.0, 0.0, 0.0), "fail": None, "parcel_lost": False, "drop": None,
         "drop_h": None, "parcel_wind": 12.0, "rain": 0.0, "body": True}
    s.update(kw)
    return s


print("\n=== in the pod / ポッド内 ===")
su = ds.SensorSuite(3, geo)
r = su.update(0.0, snap(status="docked", phase=None, pos=(5.0, 5.0, 3.0), loaded=False,
                         order=None, ground_z=3.0, obstacles={}, home=(5.0, 5.0, 3.0)))
check("GNSS has no fix under the roof (off, not a failure)", r["gnss"]["fix"] == "none" and r["gnss"]["health"] == "off")
check("datalink wired", r["link"]["mode"] == "wired")
check("motors stopped", r["motors"]["rpm"] == [0, 0, 0, 0] and r["motors"]["health"] == "off")
check("IMU reads -1 g at rest", abs(r["imu"]["az"] + 9.81) < 0.2, r["imu"]["az"])
check("cameras not available in the pod", r["cams"]["available"] is False)
check("parachute disarmed", r["parachute"]["state"] == "disarmed")
check("overall health ok", r["health"] in ("ok", "warn"), r["health"])

print("\n=== cruise / 巡航 ===")
su = ds.SensorSuite(1, geo)
x = 0.0
for i in range(10):
    t = i * 0.5
    x += 6.0
    r = su.update(t, snap(pos=(x, 0.0, 60.0)))
check("velocity 12 m/s from successive publishes", abs(r["gnss"]["speed"] - 12.0) < 0.3, r["gnss"]["speed"])
check("RTK fixed in open sky, cm accuracy", r["gnss"]["fix"] == "rtk_fixed" and r["gnss"]["acc_h"] == 0.02)
check("hover-ish RPM 5000-8000", all(5000 < v < 8000 for v in r["motors"]["rpm"]), r["motors"]["rpm"])
check("battery current 10-30 A, 6S voltage", 10 < r["battery"]["current"] < 30 and 21 < r["battery"]["voltage"] < 25.2,
      (r["battery"]["current"], r["battery"]["voltage"]))
check("time left = (soc - reserve) / drain", abs(r["battery"]["left_s"] - (80 - 15) / 0.3) < 2, r["battery"]["left_s"])
check("accelerometer ~ -T/m on the body z", -11.5 < r["imu"]["az"] < -9.0, r["imu"]["az"])
check("nose down into a headwind-free cruise with tailwind airspeed 9", abs(r["air"]["airspeed"] - 9.0) < 0.3,
      r["air"]["airspeed"])
check("height above ground 60 m: lidar out of range (40 m)", r["rangefinder"]["agl"] is None and
      r["rangefinder"]["ground"] == 60.0)
check("optical flow off above 30 m", r["flow"]["vx"] is None)
check("parachute armed", r["parachute"]["state"] == "armed")
check("Remote ID broadcasting", r["remote_id"]["broadcast"] and r["remote_id"]["health"] == "ok")
check("link RSSI falls with distance", r["link"]["rssi"] < -60, r["link"]["rssi"])
check("payload on the load cell", 5.0 < r["winch"]["load"] < 20.0 and r["winch"]["state"] == "stowed",
      r["winch"])
check("heading 90 (east)", abs(r["mag"]["heading"] - 90) < 3, r["mag"]["heading"])
r = su.update(5.5, snap(pos=(x + 6.0, 0.0, 60.0), obstacles={"front": 3.5, "rear": None}))
check("obstacle 3.5 m ahead: warn, GNSS float", r["obstacle"]["health"] == "warn" and r["gnss"]["fix"] == "rtk_float")

print("\n=== winch drop / ウインチ投下 ===")
su = ds.SensorSuite(2, geo)
su.update(0.0, snap(phase="drop", pos=(0.0, 0.0, 13.0), ground_z=10.0, drop=0.0, drop_h=3.0))
r = su.update(0.5, snap(phase="drop", pos=(0.0, 0.0, 13.0), ground_z=10.0, drop=0.5, drop_h=3.0))
check("tether half way down, lowering, parcel weight on it",
      abs(r["winch"]["tether"] - 1.4) < 0.01 and r["winch"]["state"] == "lowering" and r["winch"]["load"] > 5,
      r["winch"])
check("tether paying out at ~2.8 m/s (clamped 2)", r["winch"]["speed"] > 1.0, r["winch"]["speed"])
check("lidar sees 3 m", abs(r["rangefinder"]["agl"] - 3.0) < 0.1, r["rangefinder"]["agl"])
check("optical flow valid low down", r["flow"]["quality"] > 200)
check("parachute too low to open below 12 m", r["parachute"]["state"] == "low")
r = su.update(1.0, snap(phase="drop", pos=(0.0, 0.0, 13.0), ground_z=10.0, drop=1.0, drop_h=3.0))
check("touched down: released, load gone", r["winch"]["state"] == "released" and r["winch"]["load"] == 0.0)
r = su.update(1.5, snap(phase="drop", pos=(0.0, 0.0, 13.0), ground_z=10.0, drop=0.5, drop_h=3.0,
                         wind=(14.0, 0.0), parcel_lost=True))
check("parcel torn off in a gust: winch fail", r["winch"]["health"] == "fail")

print("\n=== failures / 故障 ===")
su = ds.SensorSuite(4, geo)
su.update(0.0, snap())
r = su.update(0.5, snap(status="falling", phase=None, fail="motor_failure", ground_z=0.0))
check("free fall: accelerometer ~0, IMU fail", abs(r["imu"]["az"]) < 2 and r["imu"]["health"] == "fail")
check("motor failure: one motor dead", r["motors"]["health"] == "fail" and r["motors"]["dead"] == 1 and
      r["motors"]["rpm"][0] == 0, r["motors"])
check("parachute deployed (armed above 12 m)", r["parachute"]["state"] == "deployed")
r = su.update(3.0, snap(status="crashed", phase=None, fail="motor_failure", pos=(0.0, 0.0, 0.0), ground_z=0.0))
check("crashed: lying on its side, 1 g on the IMU", abs(math.sqrt(r["imu"]["ax"] ** 2 + r["imu"]["ay"] ** 2 +
                                                                 r["imu"]["az"] ** 2) - 9.81) < 0.3)
check("overall health fail", r["health"] == "fail")
su = ds.SensorSuite(5, geo)
r = su.update(0.0, snap(saturated=True, wind=(13.0, 0.0)))
check("wind beyond the drone: air fail", r["air"]["health"] == "fail")
r = su.update(0.5, snap(battery=10.0))
check("below the reserve: battery warn", r["battery"]["health"] == "warn")
check("parcel weights stay in the M class", all(0.6 <= ds.parcel_kg(i) <= 1.9 for i in range(200)))
check("pressure at sea level", abs(ds.pressure_hpa(0.0) - 1013.25) < 1e-6 and ds.pressure_hpa(100) < 1002)

print("\n=== budget / 予算 ===")
suites = [ds.SensorSuite(i, geo) for i in range(12)]
for k, su in enumerate(suites):
    su.update(0.0, snap(pos=(k * 10.0, 0.0, 50.0)))
t0 = time.perf_counter()
N = 20
for n in range(1, N + 1):
    for k, su in enumerate(suites):
        su.update(n * 0.5, snap(pos=(k * 10.0 + n * 6.0, 0.0, 50.0), drop=0.5 if k % 3 == 0 else None,
                                drop_h=3.0))
ms = (time.perf_counter() - t0) * 1000.0 / N
check("12 drones in < 2 ms per publish (maths)", ms < 2.0, "%.3f ms" % ms)

print("\n=== raycast probe + snapshot (fake world) / レイ予算 ===")
import carla  # noqa: E402
from delivery import backends  # noqa: E402
from delivery.battery import Battery, BatteryModel  # noqa: E402
from delivery.wind import Drift  # noqa: E402


class FakeWorld(object):
    def __init__(self, cost=0.0003):
        self.n, self.cost = 0, cost

    def cast_ray(self, a, b):
        self.n += 1
        end = time.perf_counter() + self.cost
        while time.perf_counter() < end:
            pass
        if b.z < a.z - 50:      # downward: ground at z 0 / 下向き：地面 z=0
            return [types.SimpleNamespace(location=carla.Location(a.x, a.y, 0.0))]
        d = math.hypot(b.x - a.x, b.y - a.y)      # a wall 9.5 m past the ray start / 壁
        return [types.SimpleNamespace(location=carla.Location(a.x + (b.x - a.x) / d * 9.5,
                                                              a.y + (b.y - a.y) / d * 9.5, a.z))]


def unit(i, status="delivering"):
    return types.SimpleNamespace(id=i, status=status, pos=(i * 5.0, 0.0, 40.0), yaw=0.0)


w = FakeWorld()
probe = backends.SensorProbe(w, budget_s=0.001)
units = [unit(i) for i in range(12)] + [unit(99, "docked")]
t0 = time.perf_counter()
probe.run(units)
ms = (time.perf_counter() - t0) * 1000.0
check("probe stays within its budget (+1 ray)", ms < 1.0 + 0.5, "%.2f ms, %d rays" % (ms, w.n))
check("docked drones are not probed", 99 not in probe.cache)
for _ in range(40):
    probe.run(units)
check("round-robin reaches every flying drone", all(probe.get(i).get("ground") == 0.0 for i in range(12)))
check("front / rear distance from the drone (ray starts 0.5 m out)",
      all(abs((probe.get(i).get(k) or 0) - 10.0) < 0.01 for i in range(12) for k in ("front", "rear")),
      probe.get(0))

f = types.SimpleNamespace(state="drop", loaded=True, _timer=2.0, hover_s=4.0)
car = types.SimpleNamespace(get_location=lambda: carla.Location(-10.0, 0.0, 0.0))
u = types.SimpleNamespace(id=0, flight=f, status="delivering", world_pos=lambda: (0.0, 0.0, 13.0), yaw=0.0,
                          parcel_lost=False, bm=BatteryModel(), battery=Battery(70.0), _ground_z=None,
                          van=types.SimpleNamespace(car=car), order=types.SimpleNamespace(id=9, drop=(0, 0, 10.0)),
                          drift=Drift(11.0), fail_reason=None, hz=backends.Hazards(), body=object())
s = backends.sensor_snapshot(u, (2.0, 0.0), probe, (0, 0, 0))
check("snapshot: winch half way at half the hover", abs(s["drop"] - 0.5) < 1e-6 and abs(s["drop_h"] - 3.0) < 1e-6, s["drop"])
check("snapshot: home is the pod car", s["home"] == (-10.0, 0.0, 0.0))
check("snapshot: loaded drain > hover drain", s["drain"] > BatteryModel().hover)
r = ds.SensorSuite(0, geo).update(0.0, s)
check("suite accepts the real snapshot", r["winch"]["state"] == "lowering" and r["battery"]["soc"] == 70.0)

print("\n=== delivery_sim Fleet glue / Fleet の接続 ===")
import delivery_sim  # noqa: E402
from delivery.wind import Wind  # noqa: E402
import random  # noqa: E402
u.status, u.flight = "docked", None
u.world_pos = lambda: (1.0, 2.0, 3.0)
sky = types.SimpleNamespace(drones=types.SimpleNamespace(units=[u], rain=0.0,
                                                         station=types.SimpleNamespace(location=(0, 0, 0))),
                            wind=Wind(random.Random(1)))
fleet = types.SimpleNamespace(sky=sky, probe=probe, suites={}, geo=geo, sensor_ms=0.0)
out = delivery_sim.Fleet.sensor_readings(fleet, 1.0)
check("Fleet.sensor_readings: one entry per drone, timed", set(out) == {0} and fleet.sensor_ms > 0, fleet.sensor_ms)
check("docked drone: GNSS off", out[0]["gnss"]["health"] == "off")

print("\n=== damage model / 損傷モデル ===")
# a collision while flying forward and to the right: the front-right corner
# (motor 1) takes it / 前右へ飛行中に衝突：前右（モーター 1）
su = ds.SensorSuite(6, geo)
su.update(0.0, snap(pos=(0.0, 0.0, 40.0), ground_z=0.0))
r = su.update(0.5, snap(pos=(5.0, 3.0, 40.0), ground_z=0.0))
check("whole drone: no damage", r["damage"] is None)
r = su.update(1.0, snap(status="falling", phase=None, fail="collision:Buildings", pos=(5.0, 3.0, 39.0),
                         ground_z=0.0, detail="Buildings at z 40.0, during cruise", crashes=1))
dm = r["damage"]["parts"]
check("collision ahead-right: rotor 1 and arm 1 broken",
      dm.get("rotor1", {}).get("level") == "broken" and dm.get("arm1", {}).get("level") == "broken", dm)
check("collision: impact side named, why = collision", r["damage"]["impact"] == "front_right" and
      dm["rotor1"]["why"] == "collision" and r["damage"]["cause"] == "collision", r["damage"])
check("frontal blow: FPV and front stereo damaged, neighbour rotor 3 damaged",
      dm.get("cam_fpv", {}).get("level") == "damaged" and dm.get("cam_front") and
      dm.get("rotor3", {}).get("level") == "damaged", sorted(dm))
check("crash detail carried", r["damage"].get("detail", "").startswith("Buildings"))
check("broken rotor: motors fail, overall fail", r["motors"]["health"] == "fail" and r["health"] == "fail")
check("broken list", r["damage"]["broken"] == ["arm1", "rotor1"], r["damage"]["broken"])
r = su.update(4.0, snap(status="crashed", phase=None, fail="collision:Buildings", pos=(5.0, 3.0, 0.0),
                         ground_z=0.0, crashes=1))
dm = r["damage"]["parts"]
check("fell ~39 m (parachute deployed): impact parts still broken, landing side only damaged",
      dm["rotor1"]["level"] == "broken" and dm.get("rotor4", {}).get("level") == "damaged" and
      dm.get("gear", {}).get("level") == "damaged" and r["damage"]["fall_h"] > 30, (sorted(dm), r["damage"]["fall_h"]))
r = su.update(10.0, snap(status="charging", phase=None, fail=None, pos=(0.0, 0.0, 0.0), ground_z=0.0,
                          loaded=False, order=None, obstacles={}))
check("repaired at the station: damage cleared", r["damage"] is None and su.crash is None)

dmod = ds.damage_model
d = dmod(1, "falling", "collision:Static", impact=(-6.0, 0.5))
check("blow from behind (moving backwards, slightly right): rear-right rotor 4 + rear camera",
      d["parts"]["rotor4"]["level"] == "broken" and "cam_rear" in d["parts"] and d["impact"] == "rear_right", d)
d = dmod(1, "falling", "collision:Static", impact=(0.2, -7.0))
check("side blow to the left: front-left corner, antennas damaged",
      d["parts"]["arm3"]["level"] == "broken" and "link" in d["parts"], sorted(d["parts"]))
a = dmod(3, "falling", "collision:Terrain", impact=None, crashes=2)
b = dmod(3, "falling", "collision:Terrain", impact=None, crashes=2)
check("unknown direction: deterministic per drone and crash", a == b and len(a["broken"]) == 2, a["broken"])
d = dmod(2, "falling", "motor_failure", dead_motor=2)
check("motor failure: that motor and its ESC broken, its rotor damaged",
      d["parts"]["motor3"]["level"] == "broken" and d["parts"]["esc3"]["level"] == "broken" and
      d["parts"]["rotor3"]["level"] == "damaged" and d["cause"] == "motor_failure", d)
d = dmod(2, "crashed", "battery_depleted", fall_h=1.0)
check("flat battery, short fall: only the battery", list(d["parts"]) == ["battery"] and
      d["parts"]["battery"]["level"] == "broken", d)
d = dmod(2, "crashed", "battery_depleted", fall_h=25.0)
check("hard landing without a parachute: right rotors, skids, shell broken",
      all(d["parts"][p]["level"] == "broken" for p in ("rotor1", "rotor4", "gear", "shell")), sorted(d["parts"]))
d = dmod(2, "crashed", "blown_off_course", fall_h=25.0, chute="deployed")
check("parachute landing: damaged, nothing broken; cause wind",
      d["broken"] == [] and d["cause"] == "wind" and d["parts"]["rotor1"]["level"] == "damaged", d)
d = dmod(2, "delivering", None, parcel_lost=True)
check("parcel torn off: hook broken, winch damaged, still flying",
      d["parts"]["hook"] == {"level": "broken", "why": "tether_snapped"} and
      d["parts"]["winch"]["level"] == "damaged" and d["cause"] == "parcel_lost", d)
check("whole and flying: None", dmod(2, "delivering", None) is None and dmod(2, "docked", None) is None)
t0 = time.perf_counter()
for _ in range(1000):
    dmod(5, "crashed", "collision:Terrain", impact=(3.0, 1.0), fall_h=20.0)
per = (time.perf_counter() - t0)  # 1000 calls: seconds = ms per call / 1000 回の秒数 = 1 回の ms
check("damage model is cheap (< 0.05 ms a call)", per < 0.05, "%.4f ms" % per)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
