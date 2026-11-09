"""Headless test of cars/drivetrain.py and the cars.json drivetrain blocks.

No UE, no CARLA. Checks every block expands (gears falling, at most 8 torque
points, no autobox hunting), the emit file for the build, and that the
computed top speeds stay near the real ones (car_guide ch. 15).
cars/drivetrain.py と cars.json の drivetrain の試験（UE・CARLA 不要）。
"""
import json
import os
import sys
import tempfile

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HIL, "cars"))

import drivetrain as dt

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


CARS = os.path.join(HIL, "cars", "cars.json")
cars = dt.load_cars(CARS)
views = dt.load_cars(os.path.join(HIL, "cars", "car_views.json"))

print("every block")
for key, e in cars.items():
    if key.startswith("_") or "drivetrain" not in e:
        continue
    d = dt.expand(e["drivetrain"])
    g = d["gears"]
    ok_shift = all(g[i]["down"] < g[i - 1]["up"] * g[i]["ratio"] / g[i - 1]["ratio"] for i in range(1, len(g)))
    check("%s: %d gears, shifts don't hunt" % (key, len(g)), ok_shift)
    check("%s: reverse negative, curve <= 8 points" % key, d["reverse"] < 0 and len(d["torque_curve"]) <= 8)
    r = dt._radius(e, views)       # built radius, else the entry's dry-run radius / 半径
    t100, top = dt.simulate(d, e["mass"], r)
    real = e["drivetrain"]["_real"]["top_kmh"]
    check("%s: top %.0f km/h within 15 %% of real %d" % (key, top, real), abs(top - real) <= 0.15 * real)

print("validation")
bad = {"gears": [3.0, 3.5], "final": 4.0, "max_rpm": 7000, "torque_curve": [[1000, 100], [7000, 150]]}
try:
    dt.expand(bad)
    check("rising ratios rejected", False)
except ValueError:
    check("rising ratios rejected", True)
hunt = dict(bad, gears=[3.0, 2.0], shift_up=0.6, shift_down=[0.0, 0.5])
try:
    dt.expand(hunt)
    check("hunting shift points rejected", False)
except ValueError:
    check("hunting shift points rejected", True)

print("emit")
with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, "x.json")
    dt.main(["--emit", CARS, "rx7", out])
    d = json.load(open(out))
    check("rx7 emitted with 5 gears", len(d["gears"]) == 5 and d["differential"] == "LIMITED_SLIP_REAR_DRIVE")
    no = dict(cars)
    no["plain"] = {"make": "x", "model": "y"}
    p = os.path.join(tmp, "cars.json")
    json.dump(no, open(p, "w"))
    dt.main(["--emit", p, "plain", out])
    check("no block: no file (Prius drivetrain kept)", not os.path.exists(out))

print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
