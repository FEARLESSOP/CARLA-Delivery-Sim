#!/usr/bin/env python3
"""Three-screen driving window (triple_screen.py): monitor detection and the
camera plan. No CARLA, no display needed.

  ../../carla-venv/bin/python tests/test_triple_screen.py

3 画面表示（モニター検出とカメラ構成）の試験。
"""
import os
import sys

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import triple_screen as ts  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


# this rig: two monitors on the Intel GPU, one on the NVIDIA (reverse PRIME)
RIG = """Monitors: 3
 0: +*DP-1 1920/698x1080/393+0+0  DP-1
 1: +HDMI-2 1920/698x1080/393+1920+0  HDMI-2
 2: +HDMI-1-0 1920/698x1080/393+3840+0  HDMI-1-0
"""
m = ts.monitors(RIG)
check("xrandr monitors parsed", m == [(0, 0, 0, 1920, 1080), (1, 1920, 0, 1920, 1080),
                                      (2, 3840, 0, 1920, 1080)], m)
check("three in a row, left to right", [x[0] for x in ts.pick_three(m)] == [0, 1, 2])
shuffled = """Monitors: 3
 0: +*HDMI-2 1920/698x1080/393+1920+0  HDMI-2
 1: +DP-1 1920/698x1080/393+0+0  DP-1
 2: +HDMI-1-0 1920/698x1080/393+3840+0  HDMI-1-0
"""
check("ordered by position, not by index", [x[0] for x in ts.pick_three(ts.monitors(shuffled))] == [1, 0, 2])
check("two monitors: no triple", ts.pick_three(m[:2]) is None)
gap = [(0, 0, 0, 1920, 1080), (1, 2000, 0, 1920, 1080), (2, 3920, 0, 1920, 1080)]
check("not touching: no triple", ts.pick_three(gap) is None)
mixed = [(0, 0, 0, 1920, 1080), (1, 1920, 0, 2560, 1440), (2, 4480, 0, 1920, 1080)]
check("different sizes: no triple", ts.pick_three(mixed) is None)

a = ts.plan("angled", 150, 1920, 1080)
check("angled: three 1080p cameras of 50 deg", [c[:3] for c in a] == [(1920, 1080, 50.0)] * 3)
check("angled: yawed -50 / 0 / +50, side by side",
      [c[3] for c in a] == [-50.0, 0.0, 50.0] and [c[4] for c in a] == [0, 1920, 3840])
f = ts.plan("flat", 150, 1920, 1080)
check("flat: one 5760x1080 camera across", f == [(5760, 1080, 150.0, 0.0, 0)], f)
check("flat: a flat projection stays under 180 deg", ts.plan("flat", 210, 1920, 1080)[0][2] == 170.0)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
