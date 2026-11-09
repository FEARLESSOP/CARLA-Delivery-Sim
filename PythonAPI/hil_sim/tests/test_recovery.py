#!/usr/bin/env python3
"""recovery.py: stuck detection and the reverse-out manoeuvre on a bicycle
model. No CARLA.

  ../../carla-venv/bin/python tests/test_recovery.py
"""
import math
import os
import sys

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import recovery as rc  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


print("=== stuck watch ===")
w = rc.StuckWatch(window=20.0, dist=1.0)
for t in range(0, 15):
    check("not stuck before the window is full (t=%d)" % t, not w.stuck(1, 0.0, 0.0, float(t))) if t == 14 else w.stuck(1, 0.0, 0.0, float(t))
for t in range(15, 22):
    r = w.stuck(1, 0.0, 0.0, float(t))
check("stuck after 20 s without moving", r)
w2 = rc.StuckWatch(20.0, 1.0)
for t in range(0, 25):
    r2 = w2.stuck(2, 0.3 * t, 0.0, float(t))
check("a moving car is never stuck", not r2)
w.forget(1)
check("forget clears the history", not w.stuck(1, 0.0, 0.0, 30.0))

print("=== reverse out (bicycle model) ===")
# car wedged 1.2 m right of its lane, nose 35 deg left of the lane heading
x, y, yaw, v = 0.0, 1.2, 35.0, 0.0
m = rc.ReverseOut(x, y)
L, dt = 2.7, 0.05
for i in range(400):
    o = m.step(dt, x, y, yaw, 0.0, y)
    if o is None:
        break
    thr, brk, steer, rev = o
    a = (thr * 3.0 - brk * 8.0) if rev else -(thr * 3.0) if False else (thr * 3.0 - brk * 8.0)
    v = max(0.0, v + a * dt) if rev or brk else v
    if not rev and brk:
        v = max(0.0, v - 8.0 * dt)
    d = (-v if rev else v) * dt
    yaw += math.degrees(d / L * math.tan(steer * math.radians(70)))
    x += d * math.cos(math.radians(yaw))
    y += d * math.sin(math.radians(yaw))
back = math.hypot(x, y - 1.2)
check("finished (returned None)", o is None)
check("backed up about BACK_M m", rc.BACK_M - 0.5 <= back <= rc.BACK_M + 1.5, round(back, 2))
check("moved backwards (x decreased)", x < -2.0, round(x, 2))
check("nose turned towards the lane heading", abs(yaw) < 35.0, round(yaw, 1))
check("steer within limits", all(abs(rc.ReverseOut.steer_for(a_, 0.0, l_)) <= 0.6 for a_ in (-90, 0, 90) for l_ in (-3, 0, 3)))
check("nose yawed + off the lane -> steer + while reversing (swings it back)", rc.ReverseOut.steer_for(20.0, 0.0, 0.0) > 0)
check("nose yawed - off the lane -> steer - while reversing", rc.ReverseOut.steer_for(-20.0, 0.0, 0.0) < 0)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
