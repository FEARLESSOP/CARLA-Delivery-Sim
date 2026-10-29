"""Own cars' driver view data (cars/car_views.py), no CARLA server needed.

自作車両の視点データ（サーバ不要）。
"""
import json
import os
import sys
import tempfile

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HIL, "cars"))
import carla        # noqa: E402,F401
import car_views    # noqa: E402

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)

tmp = tempfile.mkdtemp()
car_views.PATH = os.path.join(tmp, "car_views.json")
car_views.CARLA_PATH = os.path.join(tmp, "carla_views.json")
json.dump({"vehicle.tesla.model3": {"driver_eye": [-0.13, -0.45, 1.18], "drive_side": "left"}},
          open(car_views.CARLA_PATH, "w"))
view = {"name": "MazdaRX7", "driver_eye": [-0.56, 0.395, 0.984],
        "steering_wheel": {"prop": "static.prop.mazdarx7steer", "hub": [0.06, 0.395, 0.724],
                           "pitch": -15.4, "yaw": 0.1}}
json.dump({"vehicle.mazda.rx7": view}, open(car_views.PATH, "w"))

t = car_views.load("vehicle.tesla.model3")
check("a CARLA car comes from carla_views.json, left seat, no steering-wheel prop",
      t is not None and t["driver_eye"][1] < 0 and "steering_wheel" not in t)
check("an unknown car has no entry", car_views.load("vehicle.nope.nope") is None)
v = car_views.load("vehicle.mazda.rx7")
check("an own car loads", v is not None and v["name"] == "MazdaRX7")
e = car_views.eye_location(v)
check("right-hand drive: the eye sits right of the centre line (y > 0)", e.y > 0.2)
check("the eye is behind the steering wheel hub", e.x < v["steering_wheel"]["hub"][0])
check("saving a nudged eye works",
      car_views.save_eye("vehicle.mazda.rx7", carla.Location(e.x + 0.02, e.y, e.z - 0.04)))
v2 = car_views.load("vehicle.mazda.rx7")
check("the nudge is stored and marked as tuned",
      abs(v2["driver_eye"][0] - (e.x + 0.02)) < 1e-6 and v2.get("eye_tuned") is True,
      str(v2["driver_eye"]))
check("saving for an unknown car does nothing",
      not car_views.save_eye("vehicle.nope.nope", e))
check("a CARLA car's nudge goes to carla_views.json, own cars untouched",
      car_views.save_eye("vehicle.tesla.model3", carla.Location(-0.1, -0.45, 1.2))
      and json.load(open(car_views.CARLA_PATH))["vehicle.tesla.model3"]["eye_tuned"]
      and "vehicle.tesla.model3" not in json.load(open(car_views.PATH)))
check("no temp file left behind", not os.path.exists(car_views.PATH + ".tmp"))

print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
