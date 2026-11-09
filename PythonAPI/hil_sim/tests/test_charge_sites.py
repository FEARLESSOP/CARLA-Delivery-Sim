#!/usr/bin/env python3
"""charge_sites.py: station frame beside a lane, bay entries, drive-in paths,
plug travel. No CARLA.

  ../../carla-venv/bin/python tests/test_charge_sites.py
"""
import math
import os
import sys

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

from delivery import charge_sites as cs  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


lay = cs.layout()
print("=== layout ===")
check("both bays and plug units", set(lay["bays"]) == {"lmp", "lmh"} and set(lay["plug_units"]) == {"lmp", "lmh"})
check("footprint about 14 x 9.8 m", 13.0 < lay["bbox_max"][0] - lay["bbox_min"][0] < 15.0
      and 9.0 < lay["bbox_max"][1] - lay["bbox_min"][1] < 10.5)

print("=== placement beside a keep-left lane (heading +x, kerb on the left = -y) ===")
kerb = 1.9
fr = cs.place(100.0, 50.0, 0.0, kerb, +1, lay)
check("bays point away from the lane (station yaw -90 = towards -y)", abs(cs._wrap(fr.yaw + 90.0)) < 1e-6, fr.yaw)
corners = fr.corners(lay)
near = max(c[1] for c in corners)
check("open edge STATION_GAP behind the kerb (y = 50 - kerb - gap)",
      abs(near - (50.0 - kerb - cs.STATION_GAP)) < 1e-6, round(near, 3))
check("whole footprint on the kerb side", all(c[1] <= 50.0 - kerb for c in corners))
for kind in ("lmp", "lmh"):
    b = cs.bay_frame(fr, lay, kind, 0.0, kerb)
    check("%s entry on the lane centre line" % kind, abs(b["entry"][1] - 50.0) < 1e-6, b["entry"])
    check("%s bay straight in from its entry (same x)" % kind, abs(b["bay"][0] - b["entry"][0]) < 1e-6)
    check("%s bay inside the footprint" % kind, min(c[1] for c in corners) < b["bay"][1] < near)
ents = [cs.bay_frame(fr, lay, k, 0.0, kerb)["entry"][0] for k in ("lmp", "lmh")]
check("bays centred on the lane point", abs(sum(ents) / 2 - 100.0) < 1e-6, ents)
fr_r = cs.place(100.0, 50.0, 0.0, kerb, -1, lay)
check("keep-right: footprint on the right (+y)", all(c[1] >= 50.0 + kerb for c in fr_r.corners(lay)))
fa = cs.place(100.0, 50.0, 0.0, kerb, +1, lay, along=7.0)
check("along slides it with the lane", abs(fa.x - fr.x - 7.0) < 1e-6)

print("=== paths ===")
b = cs.bay_frame(fr, lay, "lmh", 0.0, kerb)
pin = cs.path_in(b, (b["entry"][0] - 1.0, 50.0, 0.0))
check("path in ends in the bay, facing it", math.hypot(pin[-1][0] - b["bay"][0], pin[-1][1] - b["bay"][1]) < 0.11
      and abs(cs._wrap(pin[-1][2] - b["bay_yaw"])) < 1.0, pin[-1])
check("turns the short way (90 deg, not 270)", all(-91.0 <= p[2] <= 1.0 for p in pin), (min(p[2] for p in pin), max(p[2] for p in pin)))
check("points ~10 cm apart", all(math.hypot(q[0] - p[0], q[1] - p[1]) < 0.12 for p, q in zip(pin, pin[1:])))
pout = cs.path_out(b)
check("path out reverses (keeps the bay heading) then turns onto the lane",
      abs(cs._wrap(pout[0][2] - b["bay_yaw"])) < 1.0 and abs(cs._wrap(pout[-1][2])) < 1.0
      and math.hypot(pout[-1][0] - b["entry"][0], pout[-1][1] - b["entry"][1]) < 0.11)

print("=== plug ===")
xs = [cs.plug_offsets(u / 10.0) for u in range(11)]
check("boom travel 0 -> -0.375 m, monotonic", abs(xs[0]) < 1e-9 and abs(xs[-1] + cs.PLUG_REACH) < 1e-9
      and all(b2 <= a2 for a2, b2 in zip(xs, xs[1:])), [round(x, 3) for x in xs])
check("last 50 mm slower than the first part", abs(xs[10] - xs[7]) < abs(xs[7] - xs[4]))

print("=== charge rate (battery.charge_rate) ===")
from delivery.battery import charge_rate  # noqa: E402
def mins(kw, kwh, a=75.0, b=98.0):
    t, lv = 0.0, a
    while lv < b:
        lv += charge_rate(lv, kw, kwh); t += 1.0
    return t / 60.0
check("50 kW into 10 kWh, 75 -> 98 %: minutes, not seconds", 3.0 < mins(50, 10) < 7.0, round(mins(50, 10), 1))
check("roadside 22 kW slower than the station's 50 kW", mins(22, 10) > mins(50, 10) * 1.8)
check("slows above 80 % (CC-CV)", charge_rate(95, 50, 10) < 0.5 * charge_rate(50, 50, 10))
check("bigger pack takes longer", mins(50, 20) > mins(50, 10) * 1.8)

print("=== waiting bay (sideways, off the lane) ===")
q = {"entry": (100.0, 50.0), "bay": (100.0, 45.0), "bay_yaw": 0.0, "entry_yaw": 0.0}
qi = cs.queue_path_in(q, (95.0, 50.0, 0.0))
check("drives along the lane to the point beside the bay, then slides onto it",
      abs(qi[-1][0] - 100.0) < 0.11 and abs(qi[-1][1] - 45.0) < 0.11 and all(abs(p[2]) < 1e-6 for p in qi))
qo = cs.queue_path_out(q)
check("slides back onto the lane keeping its heading", abs(qo[-1][1] - 50.0) < 0.11 and all(abs(p[2]) < 1e-6 for p in qo))

print("=== footpath round the station ===")
ring = cs.ring_local(lay, (3.6, 9.6))
x0, x1 = lay["entrance_x"], lay["bbox_max"][0]
check("starts and ends at the front (pavement side), round the back",
      ring[0][0] < x0 and ring[-1][0] < x0 and ring[1][0] > x1 and ring[2][0] > x1)
check("outside the station and its waiting bay", ring[0][1] < lay["bbox_min"][1] and ring[-1][1] > 9.6)

print("=== footpath laying (segments + corners) ===")
import types  # noqa: E402
placed = []
class _Loc(object):
    def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z
class _Rot(object):
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0): self.pitch, self.yaw, self.roll = pitch, yaw, roll
fake_carla = types.SimpleNamespace(Location=_Loc, Rotation=_Rot,
                                   Transform=lambda loc, rot: (loc, rot))
st = cs.Stations.__new__(cs.Stations)
st.carla = fake_carla
st.world = types.SimpleNamespace(try_spawn_actor=lambda bp, tf: placed.append((bp, tf)) or object())
st._ground_at = lambda wx, wy, fr: 0.0
st.lay = {"walk": {"segment": 2.0}}
lib = types.SimpleNamespace(filter=lambda name: [name], find=lambda name: name)
frl = cs.Frame(0.0, 0.0, 0.0, 0.0)
pts = cs.ring_local(lay, None)
half = cs.RING_W / 2
runs = [math.hypot(b[0] - a[0], b[1] - a[1]) - (half if i > 0 else 0) - (half if i + 1 < len(pts) - 1 else 0)
        for i, (a, b) in enumerate(zip(pts, pts[1:]))]
check("ring: every straight run a whole number of 2 m segments",
      all(abs(r / 2.0 - round(r / 2.0)) < 1e-6 for r in runs), [round(r, 3) for r in runs])
st._walk(lib, frl, pts)
segs = [(tf[0].x, tf[0].y) for bp, tf in placed if bp == cs.WALK_BP]
corners = [(tf[0].x, tf[0].y, tf[1].yaw) for bp, tf in placed if bp == cs.WALK_CORNER_BP]
check("a corner at each turn", len(corners) == 2, corners)
check("pieces = total run / 2 m (nothing overlaps)", len(segs) == int(round(sum(runs) / 2.0)), (len(segs), sum(runs)))
side0 = sorted(x for x, y in segs if abs(y - pts[0][1]) < 1e-6)
check("first side: end to end, exactly 2 m apart, from the pavement end to the corner",
      abs(side0[0] - (pts[0][0] + 1.0)) < 1e-6 and all(abs(b - a - 2.0) < 1e-6 for a, b in zip(side0, side0[1:]))
      and abs(side0[-1] + 1.0 - (pts[1][0] - half)) < 1e-6, side0)
check("on flat ground the pieces lie flat (no pitch / roll)",
      all(abs(tf[1].pitch) < 1e-9 and abs(tf[1].roll) < 1e-9 for _, tf in placed))
# the corner nearest -y: the path comes along +x and turns to +y with the
# station inside: kerbs on its +x and -y edges -> local +x points -y (yaw -90)
c0 = min(corners, key=lambda c: c[1])
rx, ry = math.cos(math.radians(c0[2])), math.sin(math.radians(c0[2]))
check("corner kerbs on the two outside edges", abs(rx) < 1e-6 and abs(ry + 1.0) < 1e-6, round(c0[2], 1))

placed.clear()
st._ground_at = lambda wx, wy, fr: 0.1 * wx                     # rising 10 % towards +x / 東へ 10 % 上り
st._walk(lib, frl, [(0.0, 0.0), (6.0, 0.0)])
pp = sorted([tf for bp, tf in placed if bp == cs.WALK_BP], key=lambda tf: tf[0].x)
check("on a slope the pieces pitch with it (atan 0.1)",
      all(abs(tf[1].pitch - math.degrees(math.atan(0.1))) < 0.01 for tf in pp), round(pp[0][1].pitch, 2))
ends = [(tf[0].z - math.sin(math.radians(tf[1].pitch)), tf[0].z + math.sin(math.radians(tf[1].pitch))) for tf in pp]
check("neighbours meet at the same height at every joint (one surface)",
      all(abs(a[1] - b[0]) < 2e-3 for a, b in zip(ends, ends[1:])), [(round(a[1], 3), round(b[0], 3)) for a, b in zip(ends, ends[1:])])

placed.clear()
bump = lambda wx, wy, fr=None: 0.25 * math.exp(-((wx - 3.0) ** 2 + wy ** 2) / 0.5)   # a hump mid-piece / 盛り上がり
st._ground_at = bump
st._walk(lib, frl, [(0.0, 0.0), (6.0, 0.0)])
worst = 0.0
for bp_, tf in placed:
    p_, z0 = math.radians(tf[1].pitch), tf[0].z - 0.01
    for t in [x / 10.0 - 1.0 for x in range(21)]:
        for o in (-0.8, 0.0, 0.8):
            x = tf[0].x + t * math.cos(p_)
            zp = z0 + t * math.sin(p_)
            worst = max(worst, bump(x, o) - zp)
check("a hump under the path never pokes through (pieces lifted over it)", worst < 0.02, round(worst, 3))

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
