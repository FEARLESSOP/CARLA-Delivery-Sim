"""Launch options of the rig: one schema for the launcher, the dashboard and
the web UI.

Each option maps to a run_pipeline.py flag. ``live`` says what happens when
it changes while the simulation runs:

- ``live``    applied at once (traffic and pods restart, CARLA keeps running)
- ``map``     a map switch (the clients restart, CARLA keeps running)
- ``restart`` the whole simulation restarts (CARLA too)

  python3 launch_options.py      prints the schema and the saved options

起動オプションの定義（ランチャー・ダッシュボード・Web UI 共通）。
"""

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CARLA_ROOT = os.path.dirname(os.path.dirname(HERE))
SAVED_PATH = os.path.join(HERE, "launch_options.json")      # last options (git-ignored) / 前回の値

# key, flag, type, default, group, label, help, live, extra
# type: int | float | bool (store_true flag) | nbool (store_true "--no-x": on = flag absent) |
#       choice | str | map | vehicle
SCHEMA = [
    # world / ワールド
    {"key": "map", "flag": "--map", "type": "map", "default": "OitaKosen", "group": "world",
     "label": "Map", "help": "the map loaded at start", "live": "map"},
    {"key": "vehicles", "flag": "--vehicles", "type": "int", "default": 10, "min": 0, "max": 150,
     "group": "traffic", "label": "Cars", "help": "NPC cars driving on their own", "live": "live"},
    {"key": "walkers", "flag": "--walkers", "type": "int", "default": 10, "min": 0, "max": 200,
     "group": "traffic", "label": "Pedestrians", "help": "walkers on the pavements", "live": "live"},
    {"key": "cyclists", "flag": "--cyclists", "type": "int", "default": 3, "min": 0, "max": 40,
     "group": "traffic", "label": "Cyclists", "help": "bicycles with riders, on top of the cars", "live": "live"},
    {"key": "traffic", "flag": "--no-traffic", "type": "nbool", "default": True, "group": "traffic",
     "label": "NPC traffic", "help": "cars, walkers and cyclists (off: an empty world)", "live": "live"},
    {"key": "delivery", "flag": "--delivery", "type": "int", "default": 3, "min": 0, "max": 10,
     "group": "delivery", "label": "Delivery pods", "help": "car + drone delivery vans (0 = none)", "live": "live"},
    {"key": "drone_cams", "flag": "--drone-cams", "type": "bool", "default": True, "group": "delivery",
     "label": "Drone camera wall", "help": "a window with one camera per flying drone", "live": "live"},
    # driving / 運転
    {"key": "start_vehicle", "flag": "--start-vehicle", "type": "vehicle", "default": "vehicle.mazda.rx7",
     "group": "driving", "label": "Your car", "help": "the car you start in (Backspace cycles in game)",
     "live": "live"},
    {"key": "ffb", "flag": "--ffb", "type": "bool", "default": True, "group": "driving",
     "label": "Force feedback", "help": "G923 wheel force feedback", "live": "live"},
    {"key": "keyboard", "flag": "--keyboard", "type": "bool", "default": False, "group": "driving",
     "label": "Keyboard driving", "help": "W/A/S/D instead of the wheel (automatic without a wheel)",
     "live": "restart"},
    {"key": "autopilot", "flag": "--autopilot", "type": "bool", "default": False, "group": "driving",
     "label": "Start on autopilot", "help": "your car drives itself until you press P", "live": "restart"},
    # environment / 環境
    {"key": "weather_cycle", "flag": "--no-weather-cycle", "type": "nbool", "default": True,
     "group": "environment", "label": "Day/night + weather", "help": "24 h in 5 min, rain sometimes",
     "live": "live"},
    {"key": "traction_fx", "flag": "--no-traction-fx", "type": "nbool", "default": True,
     "group": "environment", "label": "Skid marks", "help": "marks behind sliding tyres", "live": "live"},
    # display / 表示
    {"key": "control_res", "flag": "--res", "type": "choice", "default": "1080p",
     "choices": ["720p", "1080p", "1440p", "4k"], "group": "display", "label": "Driving window",
     "help": "resolution of your view", "live": "restart"},
    {"key": "fov", "flag": "--fov", "type": "float", "default": 100.0, "min": 60, "max": 130,
     "group": "display", "label": "Field of view", "help": "degrees", "live": "restart"},
    {"key": "fps", "flag": "--fps", "type": "int", "default": 60, "min": 20, "max": 144,
     "group": "display", "label": "Frame cap", "help": "driving window FPS limit", "live": "restart"},
    {"key": "sensor_viewer", "flag": "--no-sensor-viewer", "type": "nbool", "default": False,
     "group": "display", "label": "Sensor viewer", "help": "camera / LiDAR / depth window", "live": "live"},
    # advanced / 詳細
    {"key": "hybrid", "flag": "--no-hybrid", "type": "nbool", "default": True, "group": "advanced",
     "label": "Hybrid physics", "help": "full physics only near you (much less CPU; off on keep-left maps)",
     "live": "restart"},
    {"key": "gpu", "flag": "--gpu", "type": "choice", "default": "auto", "choices": ["auto", "nvidia", "off"],
     "group": "advanced", "label": "NVIDIA offload", "help": "render CARLA on the dGPU", "live": "restart"},
    {"key": "carla_args", "flag": "--carla-args", "type": "str", "default": "", "group": "advanced",
     "label": "CARLA arguments", "help": 'e.g. "-quality-level=Low"', "live": "restart"},
    {"key": "delivery_args", "flag": "--delivery-args", "type": "str", "default": "", "group": "advanced",
     "label": "Delivery options", "help": 'e.g. "--drone-prob 0.7"', "live": "restart"},
    {"key": "log", "flag": "--no-log", "type": "nbool", "default": True, "group": "advanced",
     "label": "Session logging", "help": "telemetry CSV of every drive", "live": "restart"},
]
BY_KEY = {o["key"]: o for o in SCHEMA}
GROUPS = [("world", "World"), ("traffic", "Traffic"), ("delivery", "Delivery"), ("driving", "Driving"),
          ("environment", "Environment"), ("display", "Display"), ("advanced", "Advanced")]


def defaults():
    return {o["key"]: o["default"] for o in SCHEMA}


def clean(opts):
    """Known keys only, each coerced to its type and range. / 型と範囲を整える。"""
    out = defaults()
    for k, v in (opts or {}).items():
        o = BY_KEY.get(k)
        if o is None or v is None:
            continue
        t = o["type"]
        try:
            if t == "int":
                v = int(round(float(v)))
            elif t == "float":
                v = float(v)
            elif t in ("bool", "nbool"):
                v = bool(v) if not isinstance(v, str) else v.lower() in ("1", "true", "yes", "on")
            elif t == "choice":
                v = str(v) if str(v) in o["choices"] else o["default"]
            else:
                v = str(v)
        except (TypeError, ValueError):
            continue
        if "min" in o and isinstance(v, (int, float)):
            v = max(o["min"], min(o["max"], v))
        out[k] = v
    return out


def to_argv(opts):
    """run_pipeline.py arguments for these options. / run_pipeline.py の引数。"""
    o, argv = clean(opts), []
    for s in SCHEMA:
        v, t = o[s["key"]], s["type"]
        if t == "bool":
            if v:
                argv.append(s["flag"])
        elif t == "nbool":
            if not v:
                argv.append(s["flag"])
        elif t in ("str", "map", "vehicle") and v == "":
            continue
        else:
            argv += [s["flag"], str(v)]
    return argv


def load_saved(path=SAVED_PATH):
    try:
        with open(path) as fh:
            return clean(json.load(fh))
    except (OSError, ValueError):
        return defaults()


def save(opts, path=SAVED_PATH):
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(clean(opts), fh, indent=2)
    os.replace(tmp, path)


def catalog():
    """Maps and cars to pick from. / 選べるマップと車。"""
    import map_switch
    maps = map_switch.build_cycle(map_switch.installed_maps(os.path.join(CARLA_ROOT, "CarlaUE4", "Content")))
    own_maps = {}
    try:
        for m in json.load(open(os.path.join(HERE, "maps", "maps.json"))).values():
            if isinstance(m, dict) and m.get("name"):
                own_maps[m["name"]] = m.get("package", "")
    except (OSError, ValueError):
        pass
    cars = []
    try:
        for k, c in json.load(open(os.path.join(HERE, "cars", "cars.json"))).items():
            if isinstance(c, dict) and c.get("make"):
                cars.append({"id": "vehicle.%s.%s" % (c["make"], c["model"]), "name": c.get("name", k),
                             "own": True})
    except (OSError, ValueError):
        pass
    try:
        for k in json.load(open(os.path.join(HERE, "cars", "carla_views.json"))):
            if k.startswith("vehicle."):
                cars.append({"id": k, "name": k.split(".", 1)[1].replace(".", " "), "own": False})
    except (OSError, ValueError):
        pass
    return {"maps": [{"name": m, "own": m in own_maps, "package": own_maps.get(m, "Carla")} for m in maps],
            "cars": cars}


def self_test():
    d = defaults()
    a = to_argv(d)
    assert "--map" in a and a[a.index("--vehicles") + 1] == "10" and "--ffb" in a and "--no-sensor-viewer" in a
    assert "--no-traffic" not in a and "--keyboard" not in a and "--carla-args" not in a
    c = clean({"vehicles": "300", "ffb": "false", "control_res": "8k", "junk": 1, "traffic": False})
    assert c["vehicles"] == 150 and c["ffb"] is False and c["control_res"] == "1080p" and "junk" not in c
    assert "--no-traffic" in to_argv(c) and "--ffb" not in to_argv(c)
    print("launch_options self-test OK")


if __name__ == "__main__":
    import sys
    if sys.argv[1:] == ["--test"]:
        self_test()
    else:
        print(json.dumps({"schema": SCHEMA, "saved": load_saved()}, indent=1)[:3000])
