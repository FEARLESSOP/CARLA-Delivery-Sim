"""Headless test of vehicle_tuning.py: settings file, scopes and scaling.

Uses a real carla.VehiclePhysicsControl (the carla module must import); no
CARLA server needed. The effect on a driving car was measured live: brakes
x2.5 cut the e-tron's stop from 52 km/h from 18.5 m to 12.0 m, power x2
doubled its speed after 1.5 s of full throttle.
vehicle_tuning.py の試験（設定ファイル・適用範囲・倍率）。サーバ不要。
"""
import os
import sys
import tempfile

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import carla
import vehicle_tuning as vt

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


def blueprint_pc():
    """The e-tron's figures as CARLA reports them. / e-tron の既定値。"""
    pc = carla.VehiclePhysicsControl()
    pc.torque_curve = [carla.Vector2D(0, 749), carla.Vector2D(4000, 749), carla.Vector2D(9000, 338)]
    pc.max_rpm, pc.final_ratio, pc.drag_coefficient = 9000.0, 9.204, 0.3
    pc.forward_gears = [carla.GearPhysicsControl(1.0, 0.5, 0.65)]
    wheels = []
    for hb in (0.0, 0.0, 2000.0, 2000.0):
        w = carla.WheelPhysicsControl()
        w.max_brake_torque, w.max_handbrake_torque, w.radius, w.tire_friction = 1000.0, hb, 35.0, 3.5
        wheels.append(w)
    pc.wheels = wheels
    return pc


print("settings")
path = os.path.join(tempfile.mkdtemp(), "vehicle_tuning.json")
check("missing file gives the defaults", vt.load(path) == vt.DEFAULTS)
saved = vt.save({"brake_scale": 99, "engine_all": 0.7, "junk": 1}, path)
check("save clamps and drops unknown keys",
      saved == {"brake_scale": 4.0, "engine_all": 1}, str(saved))
check("defaults are inside their ranges",
      all(lo <= vt.DEFAULTS[k] <= hi for k, (lo, hi) in vt.LIMITS.items()))

print("scopes")
v = dict(vt.DEFAULTS, brakes_all=1, engine_all=0)
check("hero gets everything", vt.scales(v, True)["power_scale"] == v["power_scale"]
      and vt.scales(v, True)["brake_scale"] == v["brake_scale"])
check("traffic gets brakes only", vt.scales(v, False)["brake_scale"] == v["brake_scale"]
      and vt.scales(v, False)["power_scale"] == 1.0)
v["brakes_all"] = 0
check("brakes for my car only leave traffic stock", vt.scales(v, False) == vt.NEUTRAL)

print("scaling")
base = blueprint_pc()
top0 = vt.top_speed_kmh(base)
check("e-tron top speed estimate is plausible", 120 < top0 < 140, "%.0f km/h" % top0)
sc = dict(vt.NEUTRAL, brake_scale=1.5, handbrake_scale=2.0, power_scale=1.3,
          top_speed_scale=1.25, drag_scale=0.5)
FAC = vt._read_factory(blueprint_pc())
pc = vt.tune(blueprint_pc(), sc, FAC, tire_friction=2.6)
check("brake torque scaled on every wheel", [w.max_brake_torque for w in pc.wheels] == [1500.0] * 4)
check("handbrake scaled", [w.max_handbrake_torque for w in pc.wheels] == [0.0, 0.0, 4000.0, 4000.0])
check("tyre grip set", all(abs(w.tire_friction - 2.6) < 1e-6 for w in pc.wheels))
check("torque curve scaled up", abs(max(p.y for p in pc.torque_curve) - 749 * 1.3) < 0.5)
check("rev range stretched", abs(pc.max_rpm - 9000 * 1.25) < 1e-3
      and abs(pc.torque_curve[-1].x - 9000 * 1.25) < 1e-3)
check("top speed grows by the factor", abs(vt.top_speed_kmh(pc) / top0 - 1.25) < 1e-3)
check("drag scaled", abs(pc.drag_coefficient - 0.15) < 1e-6)
pc = vt.tune(blueprint_pc(), vt.NEUTRAL, FAC)
check("neutral changes nothing", pc.wheels[0].max_brake_torque == 1000.0 and pc.max_rpm == 9000.0
      and pc.wheels[0].tire_friction == 3.5)

# CARLA reads back what was applied last: tuning that again must not compound
# (the bug: power x2 then x0.5 gave x1, so lowering a slider did nothing)
# 前回適用値に再適用しても累積しないこと
print("no compounding")
up = vt.tune(blueprint_pc(), dict(vt.NEUTRAL, power_scale=2.0, brake_scale=2.0,
                                  top_speed_scale=1.5, drag_scale=0.5), FAC, tire_friction=2.0)
down = vt.tune(up, dict(vt.NEUTRAL, power_scale=0.5, brake_scale=0.5,
                        top_speed_scale=0.8, drag_scale=1.5), FAC)
check("power x2 then x0.5 is x0.5", abs(max(p.y for p in down.torque_curve) - 749 * 0.5) < 0.5)
check("brakes x2 then x0.5 is x0.5", down.wheels[0].max_brake_torque == 500.0)
check("top speed x1.5 then x0.8 is x0.8", abs(down.max_rpm - 9000 * 0.8) < 1e-3
      and abs(down.torque_curve[-1].x - 9000 * 0.8) < 1e-3)
check("drag x0.5 then x1.5 is x1.5", abs(down.drag_coefficient - 0.45) < 1e-6)
check("grip goes back to factory when not given", down.wheels[0].tire_friction == 3.5)
back = vt.tune(down, vt.NEUTRAL, FAC)
check("back to stock is stock", back.wheels[0].max_brake_torque == 1000.0 and back.max_rpm == 9000.0
      and abs(max(p.y for p in back.torque_curve) - 749) < 0.5)

print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
