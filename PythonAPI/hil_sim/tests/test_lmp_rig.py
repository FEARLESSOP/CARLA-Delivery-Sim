"""delivery/lmp_rig.py against LastMile_Pod_3D.html itself.

tests/data/lmp_html_truth.json was recorded from the HTML page: its own
applyTime / applyDirect run every 0.37 s, and the positions of the shuttle,
lift, doors, hoist, drones, trays, parcels, station and customer written down.
The port must put every mechanism where the HTML has it at the same time.
No CARLA server needed.
HTML ページ自身から記録した部品位置と、移植版の機構状態を比較する。
"""
import json
import math
import os
import sys

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

from delivery import lmp_rig as R

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


TRUTH = json.load(open(os.path.join(HIL, "tests", "data", "lmp_html_truth.json")))
TOL = 2e-3


def close(a, b, tol=TOL):
    if a is None or b is None:
        return a is None and b is None
    a, b = (a if isinstance(a, (list, tuple)) else [a]), (b if isinstance(b, (list, tuple)) else [b])
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def compare(scenario, fn, fields):
    worst = {}
    for rec in TRUTH[scenario]:
        m = fn(rec["t"])
        for name, ours in fields.items():
            a, b = ours(m), rec[name] if not callable(name) else None
            err = 0.0 if close(a, b) else (
                max(abs(x - y) for x, y in zip(a if isinstance(a, (list, tuple)) else [a],
                                               b if isinstance(b, (list, tuple)) else [b]))
                if a is not None and b is not None else 99.0)
            if err > worst.get(name, (0.0, 0))[0]:
                worst[name] = (err, rec["t"])
    for name in fields:
        err, t = worst.get(name, (0.0, 0))
        check("%s: %s" % (scenario, name), err <= TOL, "worst %.4f at t=%.2f" % (err, t))


door_x = lambda m: [R.LX + s * (0.3 + 0.64 * R.smooth(m.doors)) for s in (-1, 1)]
DRONE_FIELDS = {
    "shuttle": lambda m: m.shuttle,
    "pres": lambda m: m.presenter,
    "lift": lambda m: m.lift,
    "doorF": door_x,
    "hoist": lambda m: m.hoist[:2],
    "mag": lambda m: m.swap[2],
    "jack": lambda m: R.lerp(0.26, 0.0, R.smooth(m.jacks)),
    "tray": lambda m: list(m.tray_m),
}


def drone_fields_full():
    f = dict(DRONE_FIELDS)
    f["drones"] = None
    return f


print("drone delivery vs the HTML")
compare("drone", lambda t: R.drone_cycle(t), DRONE_FIELDS)

# drones: positions and flight attitude / 機体の位置と姿勢
worst = (0.0, 0)
for rec in TRUTH["drone"]:
    m = R.drone_cycle(rec["t"])
    for i, (x, y, z, ry, rz) in enumerate(rec["drones"] if len(rec["drones"]) == 4 else []):
        (px, py, pz), yaw, roll = m.drone_pose(i)
        err = max(abs(px - x), abs(py - y), abs(pz - z), abs(yaw - ry), abs(roll - rz))
        if err > worst[0]:
            worst = (err, rec["t"])
check("drone: four drones (pads, hoist, flight)", worst[0] <= 0.02, "worst %.4f at t=%.2f" % worst)

worst = (0.0, 0)
for rec in TRUTH["drone"]:
    m = R.drone_cycle(rec["t"])
    want = rec["parcel"]
    if m.parcel_m_on == "pod":
        got = list(m.parcel_m)
    elif m.parcel_m_on == "drone":
        (dx, dy, dz), _, _ = m.drone_pose(m.launch)
        got = [dx, dy + m.parcel_m[1], dz]
    else:
        got = None
    if want is None or got is None:
        err = 0.0 if want is None and got is None else 99.0
    else:
        err = max(abs(a - b) for a, b in zip(got, want))
    if err > worst[0] and not (m.parcel_m_on == "drone" and err < 0.05):
        worst = (err, rec["t"])
check("drone: parcel (tray, hook, cradle)", worst[0] <= 0.01, "worst %.4f at t=%.2f" % worst)

print("direct delivery vs the HTML")
DIRECT_FIELDS = {
    "shuttle": lambda m: m.shuttle,
    "jack": lambda m: R.lerp(0.26, 0.0, R.smooth(m.jacks)),
    "port": lambda m: [-R.smooth(m.port[0]) * R.SLIDE_MAX, R.PORT_Y + 0.09 + 0.5 * R.smooth(m.port[1])],
    "dtray": lambda m: list(m.tray_xl),
    "flap": lambda m: -1.3 * R.smooth(m.flap),
    "cell": lambda m: 1.6 * R.smooth(m.cell_door),
}
compare("direct", lambda t: R.direct_cycle(t), DIRECT_FIELDS)
worst = (0.0, 0)
for rec in TRUTH["direct"]:
    m = R.direct_cycle(rec["t"])
    want = rec["dparcel"]
    if m.parcel_xl_on == "pod":
        got = list(m.parcel_xl)
    elif m.parcel_xl_on == "person":
        got = list(R.person_point(m.person, R.HANDS))
    else:
        got = None
    err = (0.0 if want is None and got is None else 99.0) if want is None or got is None else \
        max(abs(a - b) for a, b in zip(got, want))
    if err > worst[0]:
        worst = (err, rec["t"])
check("direct: XL parcel (tray, belt, compartment, hands)", worst[0] <= 0.01,
      "worst %.4f at t=%.2f" % worst)
worst = (0.0, 0)
for rec in TRUTH["direct"]:
    m = R.direct_cycle(rec["t"])
    want, got = rec["person"], (list(m.person[0]) + [m.person[1]]) if m.person else None
    err = (0.0 if want is None and got is None else 99.0) if want is None or got is None else \
        max(abs(a - b) for a, b in zip(got, want))
    if err > worst[0]:
        worst = (err, rec["t"])
check("direct: customer walk", worst[0] <= 0.01, "worst %.4f at t=%.2f" % worst)

print("poses and axes")
rest = R.part_poses(R.REST)
off = max(abs(a - b) for (prop, node), pose in rest.items()
          for a, b in zip(pose[:3], (lambda p: (p[0], p[1], p[2]))(
              R.parts()[prop]["instances"][node] if node else
              next(iter(R.parts()[prop]["instances"].values())))))
check("rest pose = the model's pivots", off < 1e-6, "%.2g" % off)
check("rotator: three yaw +a is CARLA yaw -a", close(R.rotator(0, math.radians(30), 0), (0, -30, 0), 1e-6),
      str(R.rotator(0, math.radians(30), 0)))
check("rotator: three z +a is CARLA pitch +a", close(R.rotator(0, 0, math.radians(20)), (20, 0, 0), 1e-6),
      str(R.rotator(0, 0, math.radians(20))))
check("H swaps y and z", R.H((1, 2, 3)) == (1, 3, 2))
m = R.drone_cycle(R.at("lift", 5.9))
p = R.part_poses(m)
check("lift up: pad 1.22 m higher", close(p[("LMPLiftPad", None)][2] - rest[("LMPLiftPad", None)][2],
                                          R.LIFT_MAX - R.LIFT_MIN, 1e-6))
check("roof open: front door 0.64 m forward",
      close(p[("LMPHatchFront", None)][0] - rest[("LMPHatchFront", None)][0], 0.64, 1e-6))
m = R.direct_cycle(R.at("d_transfer", 4.0) - R.S["d_drive"])
p = R.part_poses(m)
check("slide out: carrier 1.45 m to the left (-y)",
      close(p[("LMPPortCarrier", None)][1] - rest[("LMPPortCarrier", None)][1], -1.45, 1e-6))
check("combined timeline is %d s" % R.COMBINED_TOTAL, R.COMBINED_TOTAL == 121)

print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
