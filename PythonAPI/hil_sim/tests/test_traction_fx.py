"""When traction_fx.py calls a tyre sliding (no CARLA server needed).

The readings are the ones measured on this rig (Charger, see traction_fx.py).
traction_fx.py の滑り判定（サーバ不要）。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from traction_fx import intensity   # noqa: E402

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)

kmh = 1 / 3.6
check("gripping turns (rear-axle slip under 7 deg, TM p99 5.5) leave no marks",
      all(intensity(40 * kmh, a) == 0.0 for a in (0, 2, 5.5, 6.7, -6.7)))
check("a drift or spin (12-28 deg at the rear axle) marks the road",
      all(intensity(35 * kmh, a) > 0.0 for a in (12, 20, 28, -20)))
check("a bigger slide marks harder", intensity(35 * kmh, 28) > intensity(35 * kmh, 12))
check("autopilot-style noisy slip ratio (0.3-0.5) without a hard pedal leaves no marks",
      all(intensity(30 * kmh, 1.0, ls, 2.0, False) == 0.0 for ls in (0.31, 0.44, 0.5)))
check("a full-throttle launch / locked wheel with the slip held marks the road",
      intensity(20 * kmh, 0, 0.39, 0.4, True) > 0.0 and intensity(50 * kmh, 0, 0.98, 0.5, True) == 1.0)
check("... but not a one-tick spike", intensity(20 * kmh, 0, 0.8, 0.05, True) == 0.0)
check("firm braking (slip 0.17-0.23) does not", intensity(50 * kmh, 0, 0.23, 1.0, True) == 0.0)
check("nothing below 5 km/h (noisy readings)", intensity(4 * kmh, 40, 1.0, 1.0, True) == 0.0)

class V(object):
    def __init__(self, x, y):
        self.x, self.y = x, y
from traction_fx import rear_slip_deg  # noqa: E402
fwd = V(1.0, 0.0)
check("straight ahead: no rear slip", abs(rear_slip_deg(V(10.0, 0.0), fwd, 0.0, -1.2)) < 1e-6)
# gripping turn: R = 8 m about a point beside the rear axle; the rear axle moves along the heading
# グリップした旋回: 後軸は向きの方向に進む
v_r, R, xr = 4.0, 8.0, -1.23
w_deg = math.degrees(v_r / R)
v_c = V(v_r, v_r / R * (-xr))           # velocity at the centre (origin) for that turn / 中心の速度
check("tight gripping turn: rear-axle slip ~0 (centre slips ~9 deg)",
      abs(rear_slip_deg(v_c, fwd, w_deg, xr)) < 0.5
      and math.degrees(math.atan2(v_c.y, v_c.x)) > 8.0, rear_slip_deg(v_c, fwd, w_deg, xr))
check("sliding sideways: the rear axle's slip shows it", abs(rear_slip_deg(V(8.0, 3.0), fwd, 0.0, -1.2)) > 15.0)
check("intensity stays within 0..1", 0.0 <= intensity(90 * kmh, 90, 5.0) <= 1.0)

print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
