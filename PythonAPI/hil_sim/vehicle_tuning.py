"""Vehicle performance settings: brakes, engine power, top speed, drag.

Shared by dashboard.py (its Vehicle tuning card, and the thread that applies
the settings to the cars) and carla_ffb.py (which sets the hero's tyre grip
in the same physics write). Values live in vehicle_tuning.json.

Every setting is a multiplier on the vehicle's own factory value, so each
car keeps its character: brakes x1.5 is 1500 Nm on the e-tron (1000 Nm) and
1200 Nm on the delivery pod (800 Nm). CARLA's get_physics_control reports
what was applied last, so scaling that would compound (x2 then x0.5 = x1);
the factory values come instead from a probe car of the same model, spawned
out of sight for a moment, and are cached in data/vehicle_factory.json.

Applying physics control rebuilds the car's physics and stops it dead, so
``apply`` sends the car's current velocity in the same batch; the car dips
by ~0.4 m/s and carries on.

車両性能の設定（ブレーキ・エンジン出力・最高速・空気抵抗）。各値は車両の
工場出荷値に対する倍率。get_physics_control は前回適用した値を返すため、それに
掛けると倍率が累積する。工場出荷値は同じ車種の検査用車両を見えない所に一瞬
生成して読み、data/vehicle_factory.json に保存する。物理制御の適用は車を止めて
しまうため、同じバッチで現在の速度を与え直す。
"""

import json
import os

import carla

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TUNING_PATH = os.path.join(BASE_DIR, "vehicle_tuning.json")
STATUS_PATH = os.path.join(BASE_DIR, "data", "vehicle_tuning_status.json")
FACTORY_PATH = os.path.join(BASE_DIR, "data", "vehicle_factory.json")
PROBE_ROLE = "vt_probe"         # the factory-value probe car / 検査用車両

# (key, default, min, max, step, group, English label, Japanese label, help)
PARAMS = [
    ("brake_scale", 1.5, 0.5, 4.0, 0.05, "brakes", "Brake power", "ブレーキ力",
     "Multiplies each wheel's brake torque. Measured on the e-tron from 52 km/h: "
     "x1 stops in 18.5 m, x1.5 in 15 m, x2.5 in 12 m."),
    ("handbrake_scale", 1.0, 0.5, 4.0, 0.05, "brakes", "Handbrake power", "サイドブレーキ力", ""),
    ("brakes_all", 1, 0, 1, 1, "brakes", "Brakes: all vehicles (1) or my car only (0)",
     "ブレーキ: 全車両 (1) / 自車のみ (0)", "All = traffic and the delivery pods too."),
    ("power_scale", 1.3, 0.5, 3.0, 0.05, "engine", "Engine power", "エンジン出力",
     "Multiplies the torque curve: harder acceleration."),
    ("top_speed_scale", 1.25, 0.7, 2.5, 0.05, "engine", "Top speed", "最高速",
     "Stretches the rev range (max RPM): the top speed grows by this factor."),
    ("drag_scale", 1.0, 0.2, 2.0, 0.05, "engine", "Air drag", "空気抵抗",
     "Lower = less drag at speed (higher top speed, slower coasting down)."),
    ("engine_all", 0, 0, 1, 1, "engine", "Engine: all vehicles (1) or my car only (0)",
     "エンジン: 全車両 (1) / 自車のみ (0)", ""),
]
GROUPS = [("brakes", "Brakes", "ブレーキ"), ("engine", "Engine & speed", "エンジンと速度")]

DEFAULTS = {p[0]: p[1] for p in PARAMS}
LIMITS = {p[0]: (p[2], p[3]) for p in PARAMS}
SWITCHES = ("brakes_all", "engine_all")
NEUTRAL = {"brake_scale": 1.0, "handbrake_scale": 1.0, "power_scale": 1.0,
           "top_speed_scale": 1.0, "drag_scale": 1.0}


def clamp_value(key, value):
    lo, hi = LIMITS[key]
    v = min(max(float(value), lo), hi)
    return int(round(v)) if key in SWITCHES else v


def load(path=TUNING_PATH):
    out = dict(DEFAULTS)
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return out
    for k, v in data.items():
        if k in out:
            try:
                out[k] = clamp_value(k, v)
            except (TypeError, ValueError):
                pass
    return out


def save(values, path=TUNING_PATH):
    clean = {k: clamp_value(k, v) for k, v in values.items() if k in DEFAULTS}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(clean, f, indent=1, sort_keys=True)
    os.replace(tmp, path)
    return clean


def scales(values, is_hero):
    """The multipliers that apply to one car. / 1 台に適用する倍率。"""
    out = dict(NEUTRAL)
    if is_hero or values["brakes_all"]:
        out["brake_scale"] = values["brake_scale"]
        out["handbrake_scale"] = values["handbrake_scale"]
    if is_hero or values["engine_all"]:
        out["power_scale"] = values["power_scale"]
        out["top_speed_scale"] = values["top_speed_scale"]
        out["drag_scale"] = values["drag_scale"]
    return out


def _read_factory(pc):
    """The fields ``tune`` sets, as plain data. / tune が設定する項目。"""
    return {"torque_curve": [[p.x, p.y] for p in pc.torque_curve],
            "max_rpm": pc.max_rpm, "drag": pc.drag_coefficient,
            "brake": [w.max_brake_torque for w in pc.wheels],
            "handbrake": [w.max_handbrake_torque for w in pc.wheels],
            "grip": [w.tire_friction for w in pc.wheels]}


_FACTORY = {}


def factory(world, type_id, path=FACTORY_PATH):
    """Factory physics of ``type_id``: cached, else read from a probe car
    spawned 500 m below the map (physics off, destroyed at once; ~50 ms).
    工場出荷値。キャッシュに無ければ地下 500 m に検査用車両を一瞬生成して読む。
    """
    if type_id in _FACTORY:
        return _FACTORY[type_id]
    try:
        with open(path) as f:
            _FACTORY.update(json.load(f))
    except (OSError, ValueError):
        pass
    if type_id not in _FACTORY:
        bp = world.get_blueprint_library().find(type_id)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", PROBE_ROLE)
        probe = None
        for dx in range(5):                 # another probe may be there / 重複回避
            probe = world.try_spawn_actor(bp, carla.Transform(carla.Location(dx * 20.0, 0.0, -500.0)))
            if probe is not None:
                break
        if probe is None:
            raise RuntimeError("could not spawn a probe %s" % type_id)
        try:
            probe.set_simulate_physics(False)
            _FACTORY[type_id] = _read_factory(probe.get_physics_control())
        finally:
            probe.destroy()
        try:
            tmp = path + ".tmp.%d" % os.getpid()
            with open(tmp, "w") as f:
                json.dump(_FACTORY, f, indent=1)
            os.replace(tmp, path)
        except OSError:
            pass
    return _FACTORY[type_id]


def tune(pc, sc, fac, tire_friction=None):
    """Set the tuned fields of ``pc`` to factory values ``fac`` times ``sc``.

    Every field is set from the factory value, never from ``pc`` (which holds
    the last applied values). Wheels and curve points come back as copies,
    so the lists are rebuilt and set again.
    pc の値ではなく必ず工場出荷値 × 倍率で設定する（累積しない）。
    """
    wheels = pc.wheels
    for i, w in enumerate(wheels):
        w.max_brake_torque = fac["brake"][i] * sc["brake_scale"]
        w.max_handbrake_torque = fac["handbrake"][i] * sc["handbrake_scale"]
        w.tire_friction = fac["grip"][i] if tire_friction is None else tire_friction
    pc.wheels = wheels
    k = sc["top_speed_scale"]
    pc.torque_curve = [carla.Vector2D(x * k, y * sc["power_scale"]) for x, y in fac["torque_curve"]]
    pc.max_rpm = fac["max_rpm"] * k
    pc.drag_coefficient = fac["drag"] * sc["drag_scale"]
    return pc


def top_speed_kmh(pc, max_rpm=None):
    """Rough drivetrain top speed: max RPM in top gear at the wheel radius
    (drag can cap it lower). / 最高速の概算（空気抵抗でより低くなり得る）。"""
    gears = [g.ratio for g in pc.forward_gears] or [1.0]
    radius = max(w.radius for w in pc.wheels) / 100.0        # cm -> m
    wheel_rpm = (max_rpm or pc.max_rpm) / (min(gears) * pc.final_ratio)
    return wheel_rpm / 60.0 * 2.0 * 3.141592653589793 * radius * 3.6


def apply(client, actor, sc, tire_friction=None):
    """Apply factory x ``sc`` physics, keeping the car's speed. / 速度を保って適用。

    Without a client the commands go one by one (same order, same tick).
    """
    fac = factory(actor.get_world(), actor.type_id)
    pc = tune(actor.get_physics_control(), sc, fac, tire_friction)
    if client is None:
        v, av = actor.get_velocity(), actor.get_angular_velocity()
        actor.apply_physics_control(pc)
        actor.set_target_velocity(v)
        actor.set_target_angular_velocity(av)
        return pc
    client.apply_batch_sync([
        carla.command.ApplyVehiclePhysicsControl(actor.id, pc),
        carla.command.ApplyTargetVelocity(actor.id, actor.get_velocity()),
        carla.command.ApplyTargetAngularVelocity(actor.id, actor.get_angular_velocity())])
    return pc


def write_status(status, path=STATUS_PATH):
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(status, f)
        os.replace(tmp, path)
    except OSError:
        pass


def read_status(path=STATUS_PATH):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None
