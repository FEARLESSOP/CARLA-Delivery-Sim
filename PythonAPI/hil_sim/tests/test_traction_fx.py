"""When traction_fx.py calls a tyre sliding (no CARLA server needed).

The readings are the ones measured on this rig (Charger, see traction_fx.py).
traction_fx.py の滑り判定（サーバ不要）。
"""
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
check("normal 40 km/h cornering (2-6 deg) leaves no marks",
      all(intensity(40 * kmh, lat, 0.05) == 0.0 for lat in (2, 4, 6, -6)))
check("a handbrake drift (12-28 deg) marks the road",
      all(intensity(35 * kmh, lat, 0.1) > 0.0 for lat in (12, 20, 28, -20)))
check("a bigger slide marks harder",
      intensity(35 * kmh, 28, 0.0) > intensity(35 * kmh, 12, 0.0))
check("wheelspin and a locked wheel mark the road",
      intensity(20 * kmh, 0, 0.39) > 0.0 and intensity(50 * kmh, 0, 0.98) == 1.0)
check("firm braking (0.17-0.23) does not", intensity(50 * kmh, 0, 0.23) == 0.0)
check("nothing below 5 km/h (noisy readings)", intensity(4 * kmh, 40, 1.0) == 0.0)
check("intensity stays within 0..1", 0.0 <= intensity(90 * kmh, 90, 5.0) <= 1.0)

print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
