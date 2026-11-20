"""Shown drone motion (delivery/drone_motion.py) without CARLA: cruise lean, braking,
sideways bank, limited yaw rate, snap, and the straight-down camera mount.
表示用のドローンの動き（CARLA なし）。

  ../../carla-venv/bin/python tests/test_drone_motion.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from delivery.drone_motion import VisualPose, nadir_mount, YAW_RATE_MAX, TILT_MAX  # noqa: E402

DT = 0.05
fails = 0


def check(name, ok, info=""):
    global fails
    fails += not ok
    print("%-58s %s %s" % (name, "OK" if ok else "FAIL", info))


# the flight logic now speeds up and brakes (delivery/drone.py _ramp_step) / 加速・減速する飛行
from delivery.drone import _ramp_step  # noqa: E402


class _F(object):
    def __init__(self, pos):
        self.pos, self._v = tuple(pos), 0.0


def fly(f, m, target, speed, steps, yaw=0.0):
    out = []
    for i in range(steps):
        f.pos, arrived = _ramp_step(f, target, speed, DT)
        out.append((f.pos, m.update(DT, f.pos, yaw), f._v))
        if arrived and i > 5:
            for _ in range(60):
                out.append((f.pos, m.update(DT, f.pos, yaw), f._v))
            break
    return out


# 1. 500 m east at 12 m/s cruise: ramps up, leans forward, brakes, stops exactly / 加速・前傾・減速・停止
f, m = _F((0, 0, 50)), VisualPose(); m.reset((0, 0, 50), 0.0)
run = fly(f, m, (500, 0, 50), 12.0, 4000)
speeds = [v for _, _, v in run]
pitches = [pose[2] for _, pose, _ in run]
cruise = [pose[2] for (p, pose, v) in run if abs(v - 12.0) < 1e-6]
check("1 flight speeds up gradually (<= 3 m/s^2)", max(b - a for a, b in zip(speeds, speeds[1:])) <= 3.0 * DT + 1e-9)
check("  flight stops exactly on the target", abs(run[-1][0][0] - 500.0) < 1e-6 and speeds[-1] == 0.0)
check("  cruise: steady forward lean (nose down)", cruise and -18 < cruise[-1] < -11, "pitch %.1f deg" % (cruise[-1] if cruise else 0))
check("  body is drawn at the flight's own position", all(abs(pose[0][0] - p[0]) < 1e-9 for p, pose, _ in run))
check("  tilt eases in and out (no jump in one step)", max(abs(b - a) for a, b in zip(pitches, pitches[1:])) < 3.0,
      "max %.2f deg/step" % max(abs(b - a) for a, b in zip(pitches, pitches[1:])))
check("2 braking: nose up, then level when stopped", max(pitches[len(pitches) // 2:]) > 3.0 and abs(pitches[-1]) < 1.0,
      "max %+.1f, end %.2f" % (max(pitches[len(pitches) // 2:]), pitches[-1]))
# 3. sideways to the right (CARLA +y with yaw 0): right side down (roll > 0) / 横移動で横に傾く
f, m = _F((0, 0, 50)), VisualPose(); m.reset((0, 0, 50), 0.0)
run = fly(f, m, (0, 300, 50), 6.0, 2000)
mid = run[len(run) // 2][1]
check("3 sideways right: banks right (roll > 0), no pitch", mid[3] > 1.0 and abs(mid[2]) < 1.0, "roll %.1f pitch %.1f" % (mid[3], mid[2]))
# 4. yaw step of 180 deg: limited rate, takes seconds / 旋回速度に上限
m = VisualPose(); m.reset((0, 0, 50), 0.0)
prev, rmax, t = 0.0, 0.0, 0.0
while t < 10:
    p, yaw, pitch, roll = m.update(DT, (0, 0, 50), 179.0)
    rmax = max(rmax, abs(((yaw - prev + 180) % 360) - 180) / DT); prev = yaw; t += DT
    if abs(((yaw - 179 + 180) % 360) - 180) < 1.0:
        break
check("4 yaw 180 deg: rate never above the limit", rmax <= YAW_RATE_MAX + 1e-6, "max %.0f deg/s" % rmax)
check("  yaw 180 deg: takes a few seconds, no snap", 2.0 < t < 6.0, "%.1f s" % t)
# 5. teleport (repair, respawn): jumps / 瞬間移動
p, *_ = m.update(DT, (500, 0, 50), 179.0)
check("5 teleport > 25 m: jumps there", abs(p[0] - 500) < 1e-6)
# 6. tilt never beyond the limit even when the position jumps around / 傾きの上限
m = VisualPose(); m.reset((0, 0, 50), 0.0)
worst = 0.0
for i in range(400):
    p, yaw, pitch, roll = m.update(DT, ((i // 20) % 2 * 20.0, 0, 50), 0.0)
    worst = max(worst, abs(pitch), abs(roll))
check("6 tilt stays within the limit", worst <= TILT_MAX + 1e-6, "max %.1f deg" % worst)
# 7. straight-down camera: always pitch -90, offset turns with the heading only / 真下カメラ
(x0, y0, z0), rot0 = nadir_mount((10, 20, 30), 0.0, (0.25, 0.25, -0.05))
(x9, y9, z9), rot9 = nadir_mount((10, 20, 30), 90.0, (0.25, 0.25, -0.05))
check("7 down camera: straight down whatever the heading", rot0[0] == -90.0 and rot9[0] == -90.0 and rot0[2] == rot9[2] == 0.0)
check("  down camera: offset turns with the heading", abs(x0 - 10.25) < 1e-9 and abs(y0 - 20.25) < 1e-9
      and abs(x9 - 9.75) < 1e-9 and abs(y9 - 20.25) < 1e-9 and abs(z0 - 29.95) < 1e-9)
print("ALL DRONE MOTION TESTS PASSED" if not fails else "%d FAILED" % fails)
sys.exit(1 if fails else 0)
