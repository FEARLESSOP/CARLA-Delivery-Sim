#!/usr/bin/env python3
"""The hero's smooth autopilot (hero_autopilot.py): speed planning and pedals,
driven against a simple car model. No CARLA.

  ../../carla-venv/bin/python tests/test_hero_autopilot.py

自車の自動運転（速度計画とペダル）の試験。サーバー不要。
"""
import math
import os
import sys

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import hero_autopilot as ha  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


class Car(object):
    """Point-mass car: full throttle 4 m/s^2 minus drag, full brake 9 m/s^2."""

    def __init__(self, v=0.0):
        self.v, self.a = v, 0.0

    def step(self, thr, brk, dt):
        drag = (0.10 + 0.00022 * self.v * self.v) * 4.5 if self.v > 0.1 else 0.0
        a = thr * 4.5 - brk * 9.0 - drag
        self.v = max(0.0, self.v + a * dt)
        self.a = a
        return self.v


def drive(target_fn, v0=0.0, secs=60.0, dt=0.05):
    lon, car = ha.Longitudinal(), Car(v0)
    log = []
    for i in range(int(secs / dt)):
        t = i * dt
        thr, brk = lon.step(dt, car.v, target_fn(t, car.v), car.a)
        car.step(thr, brk, dt)
        log.append((t, car.v, thr, brk, car.a))
    return log


def flips(log):
    """Throttle <-> brake switches. / アクセルとブレーキの切替回数。"""
    n, last = 0, None
    for _, _, thr, brk, _ in log:
        cur = "t" if thr > 0.02 else ("b" if brk > 0.02 else last)
        if last is not None and cur != last:
            n += 1
        last = cur
    return n


print("=== speed planning / 速度計画 ===")
check("stop_speed: 0 at the line", ha.stop_speed(0.0, 2.0) == 0.0)
check("stop_speed: 50 m at 2 m/s^2 = 14.1 m/s", abs(ha.stop_speed(50.0, 2.0) - math.sqrt(200)) < 1e-9)
straight = [(s, s, 0.0, 0.0) for s in range(0, 200, 2)]
check("straight: no bend limit", ha.curve_speeds(straight) == float("inf"))
# a 90-degree turn of radius 12 m starting 60 m ahead
turn = [(float(s), s, 0.0, 0.0) for s in range(0, 60, 2)]
r, s0 = 12.0, 60.0
for k in range(1, 20):
    ang = (math.pi / 2) * k / 19
    turn.append((s0 + r * ang, 0, 0, math.degrees(ang)))
v_turn = math.sqrt(3.6 * r)
got = ha.curve_speeds(turn)
check("turn 60 m ahead: slows only as needed", v_turn < got < ha.stop_speed(60.0, 3.0, v_turn) + 0.5,
      (round(got, 2), round(v_turn, 2)))
at = ha.curve_speeds([(p[0] - 60, p[1], p[2], p[3]) for p in turn[30:]])
check("at the turn: about sqrt(a_lat * r)", abs(at - v_turn) < 1.5, (round(at, 2), round(v_turn, 2)))

print("=== pedals / ペダル ===")
log = drive(lambda t, v: 100 / 3.6, secs=60)
vend = log[-1][1] * 3.6
check("reaches the 100 km/h limit and holds it", abs(vend - 100) < 2.0, round(vend, 1))
check("never over the limit by more than 2 km/h", max(v for _, v, _, _, _ in log) * 3.6 < 102.0)
check("no throttle/brake pumping while cruising", flips(log) <= 1, flips(log))
th = [x[2] for x in log]
check("throttle changes slowly (rate limited both ways)",
      max(abs(th[i + 1] - th[i]) for i in range(len(th) - 1)) <= 1.6 * 0.05 + 1e-6)
jerk = [abs(log[i + 1][4] - log[i][4]) / 0.05 for i in range(len(log) - 1)]
check("acceleration builds smoothly (jerk mostly under 8 m/s^3)",
      sorted(jerk)[int(0.98 * len(jerk))] < 8.0, round(sorted(jerk)[int(0.98 * len(jerk))], 2))
check("quick: 0-60 km/h under 7 s",
      next(t for t, v, _, _, _ in log if v * 3.6 >= 60) < 7.0,
      round(next(t for t, v, _, _, _ in log if v * 3.6 >= 60), 1))

# a red light 120 m ahead at 60 km/h: plan to stop, then green after 25 s
pos = {"x": 0.0}


def light(t, v):
    pos["x"] += v * 0.05
    if t > 25.0:
        return 60 / 3.6
    return min(60 / 3.6, ha.stop_speed(120.0 - pos["x"] - 1.5, 2.5))


log = drive(light, v0=60 / 3.6, secs=45)
check("stops before the line", pos["x"] - sum(v * 0.05 for t, v, _, _, _ in log if t > 25.0) < 120.0,
      round(pos["x"] - sum(v * 0.05 for t, v, _, _, _ in log if t > 25.0), 1))
stopped = [x for x in log if 18 < x[0] < 25]
check("stands still at the red", stopped and max(x[1] for x in stopped) < 0.2,
      max(x[1] for x in stopped) if stopped else None)
dec = [x for x in log if x[0] < 25]
check("braking is smooth (no pumping on the way in)", flips(dec) <= 1, flips(dec))
check("brake never above 0.6 for a planned stop", max(x[3] for x in dec) < 0.6,
      round(max(x[3] for x in dec), 2))
check("pulls away again on green", log[-1][1] * 3.6 > 45, round(log[-1][1] * 3.6, 1))

# limit drops 100 -> 40 at t = 10 s (a junction turn ahead)
log = drive(lambda t, v: 100 / 3.6 if t < 10 else 40 / 3.6, v0=100 / 3.6, secs=30)
check("slows 100 -> 40 without pumping (brake once, then throttle)", flips(log) <= 2 and abs(log[-1][1] * 3.6 - 40) < 2,
      (flips(log), round(log[-1][1] * 3.6, 1)))
lon = ha.Longitudinal()
thr, brk = 0.0, 0.0
for _ in range(10):
    thr, brk = lon.step(0.05, 15.0, 0.0, -1.0, urgent=True)
check("urgent: brakes hard quickly", brk > 0.5, brk)

print("=== steering / 操舵 ===")
check("wheel for steer: 0 -> centre, 0.5 -> full lock", ha.wheel_for_steer(0.0) == 0.0
      and abs(ha.wheel_for_steer(0.5) - 1.0) < 0.01 and ha.wheel_for_steer(-0.9) == -1.0)
x = 0.3
check("wheel for steer inverts the driving window's mapping",
      abs(ha.wheel_for_steer(0.5 * 1.83 * math.tan(0.5 * x)) - x) < 1e-9)
# a car on a straight lane 1.5 m left of the line: steers back, no overshoot
lat, px, py, yaw = ha.Lateral(), 0.0, -1.5, 0.0
path_pts = [(s, s, 0.0) for s in range(0, 300, 2)]
ys, steers = [], []
for i in range(400):
    v = 15.0
    pts = [(p[0] - px, p[1], p[2]) for p in path_pts if p[1] > px]
    st = lat.step(0.05, px, py, yaw, v, pts)
    delta = st * math.radians(70.0)
    yaw += math.degrees(v / 2.7 * math.tan(delta) * 0.05)
    px += v * math.cos(math.radians(yaw)) * 0.05
    py += v * math.sin(math.radians(yaw)) * 0.05
    ys.append(py)
    steers.append(st)
check("pure pursuit settles on the line", abs(ys[-1]) < 0.1, round(ys[-1], 3))
check("no overshoot past the line by more than 0.3 m", max(ys) < 0.3, round(max(ys), 3))
check("steering rate limited (<= 0.9/s)",
      max(abs(steers[i + 1] - steers[i]) for i in range(len(steers) - 1)) <= 0.9 * 0.05 + 1e-9)
check("steering stays within the wheel's 0.5", max(abs(s) for s in steers) <= 0.5)

# 90-degree corner: no premature turn-in, tracks arc without mounting kerb
# 90度交差点: 手前で曲がり始めて縁石に乗らない、内輪差考慮でインカット抑制
corner_pts = []
for x_pt in range(0, 31, 2):
    corner_pts.append((float(x_pt), float(x_pt), 0.0, 0.0))
R_corner = 8.0
s_corner_start = 30.0
for k in range(1, 16):
    ang = (math.pi / 2.0) * (k / 15.0)
    wx = s_corner_start + R_corner * math.sin(ang)
    wy = R_corner * (1.0 - math.cos(ang))
    corner_pts.append((s_corner_start + R_corner * ang, wx, wy, math.degrees(ang)))
for y_pt in range(9, 30, 2):
    corner_pts.append((s_corner_start + R_corner * (math.pi / 2.0) + (y_pt - 8.0), s_corner_start + R_corner, float(y_pt), 90.0))

lat_c = ha.Lateral(wheelbase=2.7, max_angle_deg=70.0, max_steer=0.85, tau=0.08, rate=2.4)
c_x, c_y, c_yaw, c_v = 0.0, 0.0, 0.0, 4.2
corner_log = []
for _ in range(250):
    q = [p for p in corner_pts if (p[1] - c_x) * math.cos(math.radians(c_yaw)) + (p[2] - c_y) * math.sin(math.radians(c_yaw)) >= -0.5
         and (p[1] - c_x) ** 2 + (p[2] - c_y) ** 2 >= 3.5]
    if not q:
        break
    ahead = []
    px_c, py_c = c_x, c_y
    s_c = 0.0
    for p in q:
        s_c += math.hypot(p[1] - px_c, p[2] - py_c)
        px_c, py_c = p[1], p[2]
        ahead.append((s_c, p[1], p[2], p[3]))
        if s_c > 30.0:
            break
    st = lat_c.step(0.05, c_x, c_y, c_yaw, c_v, ahead)
    delta = st * math.radians(70.0)
    c_yaw += math.degrees(c_v / 2.7 * math.tan(delta) * 0.05)
    c_x += c_v * math.cos(math.radians(c_yaw)) * 0.05
    c_y += c_v * math.sin(math.radians(c_yaw)) * 0.05
    corner_log.append((c_x, c_y, c_yaw, st))

entry_pts = [p for p in corner_log if 28.5 <= p[0] <= 30.5]
y_at_entry = entry_pts[-1][1] if entry_pts else 0.0
check("corner approach: stays in lane prior to entry (<= 0.20 m offset)",
      abs(y_at_entry) <= 0.20, round(y_at_entry, 3))
in_turn = [p for p in corner_log if p[0] >= 30.0 and p[1] <= 8.5]
max_cut = max(8.0 - math.hypot(p[0] - 30.0, p[1] - 8.0) for p in in_turn) if in_turn else 0.0
check("inside 8m turn: tracks arc without apex cutting (<= 0.25 m cut)",
      max_cut <= 0.25, round(max_cut, 3))

import random  # noqa: E402
rng = random.Random(1)
picks = [ha.pick_branch([0.0, -90.0, 178.0], rng) for _ in range(2000)]
check("junction: never a U-turn", 2 not in picks)
check("junction: straight about 4x as likely as a turn", 3.0 < picks.count(0) / max(1, picks.count(1)) < 5.5,
      round(picks.count(0) / max(1, picks.count(1)), 2))
check("junction: only a U-turn left -> the gentlest branch", ha.pick_branch([170.0, -160.0], rng) == 1)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
