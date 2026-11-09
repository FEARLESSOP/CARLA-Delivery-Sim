"""KPI tracker check with fake vans and drones (no CARLA). / KPI 集計の確認"""
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from delivery.kpi import KpiTracker  # noqa: E402
from delivery.zones import Zones  # noqa: E402

fails = []


def check(name, ok, info=""):
    print("  %s  %s %s" % ("PASS" if ok else "FAIL", name, info))
    if not ok:
        fails.append(name)


zones = Zones({"pods": {"1": {"x0": 0, "y0": 0, "x1": 100, "y1": 100}},
               "nofly": [{"x0": 200, "y0": 0, "x1": 300, "y1": 100}]})
pos = {"p": NS(x=50.0, y=50.0)}
van = NS(vid=1, state="idle", energy=NS(level=80.0), pod_km=2.0, delivered=[], failed=[],
         car=NS(get_location=lambda: pos["p"]))
drone = NS(id=7, status="docked", battery=NS(level=90.0), pos=(50.0, 50.0, 0.0))
drones = NS(units=[drone], grounded=False, summary=lambda: {"trips": 2, "crashes": 0})
fleet = NS(vans=[van], zones=zones, sky=NS(drones=drones, pem=NS(pod_kwh=10.0)))
k = KpiTracker(fleet)

script = [  # t, van state, van level, van pos, drone status, drone pos, battery
    (0, "idle", 80, (50, 50), "docked", (50, 50, 0), 90),
    (1, "driving", 79, (50, 50), "docked", (50, 50, 0), 90),
    (2, "stopping", 78, (150, 50), "delivering", (250, 50, 30), 85),   # working out of zone, drone in no-fly
    (3, "hatch_open", 78, (150, 50), "delivering", (60, 50, 30), 80),
    (4, "driving", 77, (150, 50), "docked", (50, 50, 0), 70),           # transit out of zone
    (5, "idle", 77, (50, 50), "docked", (50, 50, 0), 70),
]
for t, st, lvl, p, ds, dp, bat in script:
    van.state, van.energy.level, pos["p"] = st, lvl, NS(x=p[0], y=p[1])
    drone.status, drone.pos, drone.battery.level = ds, dp, bat
    k.sample(float(t))
van.delivered = [NS(kind="face_to_face", created_t=0.0, delivered_t=4.0, drop=(10, 10, 0)),
                 NS(kind="drone", created_t=0.0, delivered_t=3.0, drop=(250, 50, 0))]
r = k.result(5.0)
pp = r["pod"]["per_pod"][0]
check("one stop of 2 s", pp["stops"] == 1 and pp["mean_stop_s"] == 2.0, pp)
check("energy 3 % of 10 kWh", pp["kwh_used"] == 0.3, pp["kwh_used"])
check("working out of zone 2 s", pp["out_of_zone_working_s"] == 2.0, pp)
check("transit out of zone counted apart", pp["transit_other_zone_s"] == 1.0, pp)
check("drone flight 2 s, landed at 70 %", r["drone"]["flight_time_mean_s"] == 2.0
      and r["drone"]["landing_battery_min"] == 70.0, r["drone"])
check("no-fly sample caught", r["drone"]["nofly_breach_samples"] == 1)
check("drop outside zone caught", r["drone"]["drops_outside_zone"] == 1)
check("lead times by kind", r["pod"]["lead_time_mean_s"] == 4.0 and r["drone"]["lead_time_mean_s"] == 3.0)
print("\nALL PASSED" if not fails else "\nFAILED: %s" % fails)
sys.exit(1 if fails else 0)
