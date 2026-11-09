"""Real drivetrains for your own cars: the cars.json ``drivetrain`` block.

Plain Python (no UE, no CARLA). Used by
  - build_cars.sh: ``drivetrain.py --emit cars.json <key> <out.json>`` writes
    the expanded values build_vehicle.py puts into the vehicle blueprint
    (gears, final drive, reverse, max RPM, MOI, damping, autobox, clutch,
    drag, differential); nothing is written for a car without a block;
  - you: ``drivetrain.py cars.json [key ...]`` prints, per car, the gear /
    speed table and a rough 0-100 km/h and top-speed estimate (car_guide
    ch. 15).
The torque curve can't be set from UE Python (FRuntimeFloatCurve's keys are
not exposed): vehicle_tuning.py applies it at runtime from cars.json.

自作車両の実車ドライブトレイン（cars.json の drivetrain）。build_cars.sh が
展開値を build_vehicle.py に渡し、単体実行ではギア別速度表と概算性能を表示する。
トルクカーブは UE Python から設定できないため vehicle_tuning.py が実行時に適用。
"""

import json
import math
import os
import sys

DIFFERENTIALS = {"rear": "LIMITED_SLIP_REAR_DRIVE", "front": "LIMITED_SLIP_FRONT_DRIVE",
                 "4wd": "LIMITED_SLIP_4W"}
PHYSX_MAX_CURVE_POINTS = 8      # PxVehicleEngineData::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES
PRIUS_CHASSIS_W_CM = 180.0      # UE only uses width x height (drag area)
AIR = 1.25                      # kg/m^3, the value UE4's PhysX vehicle uses


def load_cars(path):
    with open(os.path.expanduser(path)) as f:
        return json.load(f)


def _per_gear(value, n, name):
    vals = list(value) if isinstance(value, (list, tuple)) else [value] * n
    if len(vals) != n:
        raise ValueError("%s: %d values for %d gears" % (name, len(vals), n))
    return [float(v) for v in vals]


def expand(block):
    """cars.json ``drivetrain`` -> the values the blueprint gets.

    Shift ratios are fractions of max RPM (PhysX autobox: up when engine
    RPM / max > up of the current gear, down when < down). Without an
    explicit ``shift_down``, each gear's down point is 80 % of the RPM the
    engine lands on after the upshift into it, so the box never hunts.
    Damping defaults scale CARLA's (0.15 / 2.0 / 0.35) to the engine size:
    full throttle 10 % of peak torque at max RPM, engine braking 2.0 x
    peak/400 Nm (capped at CARLA's 2.0).
    """
    gears = [float(g) for g in block["gears"]]
    n = len(gears)
    if any(b >= a for a, b in zip(gears, gears[1:])):
        raise ValueError("gears must get taller (ratios falling): %s" % gears)
    curve = [[float(r), float(t)] for r, t in block["torque_curve"]]
    if len(curve) > PHYSX_MAX_CURVE_POINTS:
        raise ValueError("torque_curve: PhysX keeps only %d points" % PHYSX_MAX_CURVE_POINTS)
    max_rpm = float(block["max_rpm"])
    up = _per_gear(block.get("shift_up", 0.85), n, "shift_up")
    if n == 1:
        up, down = [0.0], [0.0]
    elif "shift_down" in block:
        down = _per_gear(block["shift_down"], n, "shift_down")
    else:
        down = [0.0] + [round(0.8 * up[i - 1] * gears[i] / gears[i - 1], 3) for i in range(1, n)]
    for i in range(1, n):
        landed = up[i - 1] * gears[i] / gears[i - 1]
        if down[i] >= landed:
            raise ValueError("gear %d: down %.2f >= %.2f reached after the upshift (hunting)"
                             % (i + 1, down[i], landed))
    peak = max(t for _, t in curve)
    omega = max_rpm * math.pi / 30.0
    k = min(1.0, peak / 400.0)
    damping = block.get("damping") or [round(min(0.15, max(0.005, 0.10 * peak / omega)), 4),
                                       round(2.0 * k, 3), round(0.35 * k, 3)]
    drive = block.get("drive", "rear")
    area = float(block.get("frontal_area_m2", PRIUS_CHASSIS_W_CM * 147.0 / 1e4))
    return {
        "gears": [{"ratio": g, "down": d, "up": u} for g, d, u in zip(gears, down, up)],
        "final": float(block["final"]),
        "reverse": -abs(float(block.get("reverse", gears[0]))),
        "max_rpm": max_rpm,
        "torque_curve": curve,
        "moi": float(block.get("moi", 0.5)),
        "damping": [float(x) for x in damping],
        "autobox": bool(block.get("autobox", True)),
        "switch_time": float(block.get("switch_time", 0.3)),
        "latency": float(block.get("autobox_latency", 0.5)),
        "clutch": float(block.get("clutch", 10.0)),
        "neutral_up": 0.15,
        "drag": float(block.get("drag", 0.3)),
        "chassis_cm": [PRIUS_CHASSIS_W_CM, round(area * 1e4 / PRIUS_CHASSIS_W_CM, 1)],
        "differential": DIFFERENTIALS[drive],
        "front_rear_split": float(block.get("front_rear_split", 0.45)),
    }


def blueprint(entry):
    return "vehicle.%s.%s" % (entry["make"], entry["model"])


# -- offline sanity check / 机上チェック ----------------------------------------

def torque_at(curve, rpm):
    if rpm <= curve[0][0]:
        return curve[0][1]
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if rpm <= x1:
            return y0 + (y1 - y0) * (rpm - x0) / max(1e-6, x1 - x0)
    return curve[-1][1]         # PhysX holds the last value / 最後の値を保持


def kmh_at(rpm, overall, radius):
    return rpm / overall * math.pi / 30.0 * radius * 3.6


def simulate(d, mass, radius, mu=1.1, dt=0.005, t_end=90.0):
    """Rough straight-line run at full throttle with the autobox: returns
    (0-100 km/h s or None, top km/h). Drive force limited by grip on the
    driven axle below 120 km/h (60 %, 4wd all), engine inertia reflected through the
    gearing, aero drag, switch time without drive. No rolling resistance
    (PhysX has none either)."""
    g = [x["ratio"] for x in d["gears"]]
    up = [x["up"] for x in d["gears"]]
    share = 1.0 if d["differential"].endswith("4W") else 0.6
    cda = d["drag"] * d["chassis_cm"][0] * d["chassis_cm"][1] / 1e4
    peak_rpm = max(d["torque_curve"], key=lambda p: p[1])[0]
    v, t, gear, shifting, t100 = 0.1, 0.0, 0, 0.0, None
    vmax = 0.0
    while t < t_end:
        overall = g[gear] * d["final"]
        rpm = v / radius * overall * 30.0 / math.pi
        if gear == 0:
            rpm = max(rpm, min(peak_rpm, 0.6 * d["max_rpm"]))   # clutch slip at launch
        rpm = min(rpm, d["max_rpm"])
        omega = rpm * math.pi / 30.0
        if shifting > 0:
            force, shifting = 0.0, shifting - dt
        else:
            te = torque_at(d["torque_curve"], rpm) - d["damping"][0] * omega
            force = max(te, 0.0) * overall / radius
            if v < 120 / 3.6:       # launch: grip-limited; later aero/power-limited
                force = min(force, mu * mass * 9.81 * share)
            if rpm >= d["max_rpm"] - 1:
                force = min(force, cda * 0.5 * AIR * v * v)     # on the limiter / リミッター
        m_eff = mass + d["moi"] * overall ** 2 / radius ** 2
        v += (force - 0.5 * AIR * cda * v * v) / m_eff * dt
        t += dt
        vmax = max(vmax, v)
        if t100 is None and v >= 100 / 3.6:
            t100 = t
        if (gear < len(g) - 1 and up[gear] > 0 and rpm >= up[gear] * d["max_rpm"]
                and shifting <= 0):
            gear, shifting = gear + 1, d["switch_time"]
    return t100, vmax * 3.6


def table(key, entry, radius):
    d = expand(entry["drivetrain"])
    rows = ["| gear | ratio | overall | down/up rpm | km/h at upshift | km/h at %d rpm |" % d["max_rpm"],
            "|---|---|---|---|---|---|"]
    for i, x in enumerate(d["gears"]):
        overall = x["ratio"] * d["final"]
        up_rpm = x["up"] * d["max_rpm"]
        rows.append("| %d | %.3f | %.2f | %s / %s | %s | %.0f |" % (
            i + 1, x["ratio"], overall,
            "%.0f" % (x["down"] * d["max_rpm"]) if x["down"] else "-",
            "%.0f" % up_rpm if x["up"] else "-",
            "%.0f" % kmh_at(up_rpm, overall, radius) if x["up"] else "-",
            kmh_at(d["max_rpm"], overall, radius)))
    t100, top = simulate(d, entry["mass"], radius)
    rows.append("")
    rows.append("Wheel radius %.3f m, mass %d kg, Cd %.2f x %.2f m2, MOI %.2f, damping %s, "
                "switch %.2f s. Estimate: 0-100 km/h %s s, top %.0f km/h." % (
                    radius, entry["mass"], d["drag"], d["chassis_cm"][0] * d["chassis_cm"][1] / 1e4,
                    d["moi"], d["damping"], d["switch_time"],
                    "%.1f" % t100 if t100 else "> 90", top))
    return "\n".join(rows)


def _radius(entry, views):
    """Wheel radius: the built car's (car_views.json), else the entry's
    ``wheel_radius_m`` (the prepare dry-run's measured radius, for a car not
    built yet), else 0.31 m. / 車輪半径（ビルド済み→エントリ→0.31 m）"""
    v = views.get(blueprint(entry), {})
    return float(v.get("wheel_radius_m", entry.get("wheel_radius_m", 0.31)))


def main(argv):
    if argv and argv[0] == "--emit":
        cars_json, key, out = argv[1:4]
        entry = load_cars(cars_json)[key]
        if "drivetrain" in entry:
            with open(out, "w") as f:
                json.dump(expand(entry["drivetrain"]), f, indent=1)
            print("drivetrain: %s -> %s" % (key, out))
        elif os.path.exists(out):
            os.remove(out)
        return 0
    cars_json = argv[0] if argv else os.path.join(os.path.dirname(os.path.abspath(__file__)), "cars.json")
    cars = load_cars(cars_json)
    views_path = os.path.join(os.path.dirname(os.path.abspath(cars_json)), "car_views.json")
    views = load_cars(views_path) if os.path.exists(views_path) else {}
    for key in (argv[1:] or [k for k in cars if not k.startswith("_")]):
        entry = cars[key]
        if "drivetrain" not in entry:
            print("## %s: no drivetrain block (Prius drivetrain)\n" % key)
            continue
        print("## %s (%s, %s)\n" % (key, blueprint(entry), entry["drivetrain"].get("transmission", "")))
        print(table(key, entry, _radius(entry, views)) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
