#!/usr/bin/env python3
"""Heavy / NPC vehicles (car_guide ch. 16): npc_only cars kept out of parking,
random traffic and the car choices; build_cars.sh passes the heavy-vehicle
options (base_type, brakes, physics box, centre of mass, no steering prop)
only for cars that set them. No CARLA, no UE.

  ../../carla-venv/bin/python tests/test_heavy_npc.py

大型・NPC 車両（ガイド 16 章）の試験（CARLA・UE 不要）。
"""
import json
import os
import re
import subprocess
import sys
import tempfile

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
sys.path.insert(0, os.path.join(HIL, "cars"))

import own_cars  # noqa: E402
import parked_cars as pc  # noqa: E402
import drivetrain as dt  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


print("=== own_cars ===")
tmp = tempfile.mkdtemp()
path = os.path.join(tmp, "cars.json")
json.dump({"_doc": "notes", "a": {"make": "mazda", "model": "rx7"},
           "bus": {"make": "oitabus", "model": "erga", "npc_only": True},
           "broken": {"name": "no make"}}, open(path, "w"))
check("user cars without npc_only", own_cars.user_car_ids(path) == {"vehicle.mazda.rx7"},
      own_cars.user_car_ids(path))
check("npc=True includes the bus", own_cars.user_car_ids(path, npc=True) == {"vehicle.mazda.rx7", "vehicle.oitabus.erga"})
check("npc_only ids", own_cars.npc_only_ids(path) == {"vehicle.oitabus.erga"})
check("missing file: empty", own_cars.load(os.path.join(tmp, "none.json")) == {})

print("=== the real cars.json ===")
real_npc = own_cars.npc_only_ids()
check("the Oita bus is npc_only", "vehicle.oitabus.erga" in real_npc, real_npc)
check("parked_cars: the bus is not a user car to park", "vehicle.oitabus.erga" not in pc.user_car_ids())
check("parked_cars: other user cars still park", "vehicle.mazda.rx7" in pc.user_car_ids())
ids = ["vehicle.toyota.prius", "vehicle.oitabus.erga", "vehicle.mazda.rx7"]
base = {"vehicle.toyota.prius": "car", "vehicle.oitabus.erga": "car", "vehicle.mazda.rx7": "car"}
pool = pc.pool_ids(ids, base, {"vehicle.mazda.rx7"}, npc={"vehicle.oitabus.erga"})
check("pool: npc_only never parks, even as base_type car", pool == ["vehicle.mazda.rx7", "vehicle.toyota.prius"], pool)
check("pool: old 3-argument call unchanged", "vehicle.oitabus.erga" in pc.pool_ids(ids, base, set()))

src = open(os.path.join(HIL, "launch_options.py")).read()
check("launcher catalog skips npc_only cars", 'not c.get("npc_only")' in src)
gt = open(os.path.join(HIL, "generate_traffic.py")).read()
check("random traffic skips npc_only (unless --filterv names it)",
      "own_cars.npc_only_ids() - {args.filterv}" in gt)
g923 = open(os.path.join(HIL, "manual_control_steeringwheel G923.py")).read()
check("hero cycle skips npc_only (unless the filter names it)",
      "own_cars.npc_only_ids() - {self._actor_filter}" in g923)

print("=== drivetrain radius ===")
e = {"make": "x", "model": "y", "wheel_radius_m": 0.4867}
check("entry radius used before a build", dt._radius(e, {}) == 0.4867)
check("built radius wins", dt._radius(e, {"vehicle.x.y": {"wheel_radius_m": 0.5}}) == 0.5)
check("default 0.31", dt._radius({"make": "x", "model": "y"}, {}) == 0.31)

print("=== build_cars.sh step 4 options ===")
sh = open(os.path.join(HIL, "cars", "build_cars.sh")).read()
m = re.search(r'VEHSTAGE/\$k\.json" <<\'PYEOF\'\n(.*?)\nPYEOF', sh, re.S)
check("step 4 snippet found", m is not None)
snippet = os.path.join(tmp, "step4.py")
open(snippet, "w").write(m.group(1) if m else "")
car = {"name": "MazdaRX7", "blueprint": "vehicle.mazda.rx7", "wheel_radius_m": 0.3167, "tyre_width_m": 0.235,
       "mass_kg": 1280, "bounds_extent_m": [2.152, 0.958, 0.608], "bounds_centre_m": [0, 0, 0.608],
       "steering_wheel": {"prop": "static.prop.mazdarx7steer"}}
bus = {"name": "OitaBusErga", "blueprint": "vehicle.oitabus.erga", "wheel_radius_m": 0.4867, "tyre_width_m": 0.275,
       "mass_kg": 9660, "bounds_extent_m": [5.384, 1.437, 1.54], "bounds_centre_m": [-0.08, 0.05, 1.54],
       "base_type": "bus", "npc_only": True, "brake_torque_nm": 7000, "handbrake_torque_nm": 7000,
       "com_height_m": 1.1, "collision_box": {"x": [-5.44, 4.95], "y": [-1.235, 1.235], "z": [0.35, 3.0]}}


def env_of(c):
    p = os.path.join(tmp, c["name"] + ".json")
    json.dump(c, open(p, "w"))
    out = subprocess.run([sys.executable, snippet, p], capture_output=True, text=True).stdout
    return dict(kv.split("=", 1) for kv in out.split())


ec, eb = env_of(car), env_of(bus)
check("a car: as before (base car, no brakes / body / COM, steering prop)",
      ec.get("BASE_TYPE") == "car" and ec.get("BRAKE") == "" and ec.get("BODY") == "" and ec.get("COMZ") == ""
      and ec.get("STEER") == "1" and ec.get("MASS") == "1280", ec)
check("the bus: base bus, 7000 Nm, no steering prop", eb.get("BASE_TYPE") == "bus" and eb.get("BRAKE") == "7000"
      and eb.get("HBRAKE") == "7000" and eb.get("STEER") == "0", eb)
body = [float(v) for v in eb.get("BODY", "").split(",")] if eb.get("BODY") else []
check("the bus: body box centre and half sizes in cm",
      len(body) == 6 and abs(body[0] + 24.5) < 0.1 and abs(body[2] - 167.5) < 0.1 and abs(body[3] - 519.5) < 0.1
      and abs(body[4] - 123.5) < 0.1 and abs(body[5] - 132.5) < 0.1, body)
check("the bus: centre of mass 110 cm", eb.get("COMZ") == "110.0", eb.get("COMZ"))
check("the cook list skips a missing steering prop", '[ "$STEER" = 0 ] ||' in sh)
check("the build fails if brakes / body were asked for but not applied",
      "brakes not applied" in sh and "physics body not applied" in sh)
bv = open(os.path.join(HIL, "delivery", "assets", "build_vehicle.py")).read()
check("build_vehicle: base_type from the env, default car",
      'os.environ.get("VEH_BASE_TYPE") or "car"' in bv and '"base_type", BASE_TYPE' in bv)

print()
print("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS))
sys.exit(1 if FAILS else 0)
