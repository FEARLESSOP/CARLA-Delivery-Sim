"""Per-car driver view and turning steering wheel for your own cars.

cars/build_cars.sh writes car_views.json: for every car you built, keyed by
its blueprint id, the driver's eye point and the steering wheel's hub and
column tilt (CARLA axes: x forward, y right, z up, metres, origin on the
ground between the axles). The driving script uses it to

  - put the driver-eye camera at that car's own seat (right-hand drive),
    instead of guessing from the bounding box;
  - spawn the car's steering wheel as a prop on its hub and turn it 1:1
    with the real wheel (SteeringWheel.set every frame).

carla_views.json holds the measured driver eye for each of CARLA's own
vehicles (no steering-wheel prop: theirs is part of the body mesh).

The eye can be nudged live (Ctrl+I/K forward/back, Ctrl+J/L left/right,
Ctrl+U/O down/up); each nudge is saved back to car_views.json.

自作車両ごとのドライバー視点と、実ハンドルに合わせて回るハンドル。
"""

import json
import os

import carla

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(HERE, "car_views.json")              # your own cars / 自作車両
CARLA_PATH = os.path.join(HERE, "carla_views.json")      # CARLA's vehicles / CARLA 車両


def _read(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _file_of(type_id):
    for path in (PATH, CARLA_PATH):
        if type_id in _read(path):
            return path
    return None


def load(type_id):
    """The car's entry (own cars first, then CARLA's), or None. / 車両の設定。"""
    path = _file_of(type_id)
    return _read(path).get(type_id) if path else None


def eye_location(view):
    return carla.Location(*view["driver_eye"])


def save_eye(type_id, eye):
    """Store a tuned driver eye (kept across rebuilds). / 調整した視点を保存。"""
    path = _file_of(type_id)
    if path is None:
        return False
    views = _read(path)
    views[type_id]["driver_eye"] = [round(eye.x, 3), round(eye.y, 3), round(eye.z, 3)]
    views[type_id]["eye_tuned"] = True
    # Keep the first camera view synced if custom cameras exist / カスタムカメラ定義がある場合は第1視点も連動
    if "cameras" in views[type_id] and views[type_id]["cameras"]:
        views[type_id]["cameras"][0]["loc"] = views[type_id]["driver_eye"]
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(views, f, indent=2)
    os.replace(tmp, path)
    return True


class SteeringWheel(object):
    """The car's steering wheel, attached on its hub, rolled about its column.

    ``set(deg)``: the real wheel's angle, positive = turned right (clockwise
    as the driver sees it). Only sent when it changed by 0.2 deg or more.
    車両のハンドル（ハブに取り付け、コラム軸まわりに回す）。
    """

    def __init__(self, world, car, view):
        sw = view["steering_wheel"]
        self.hub = carla.Location(*sw["hub"])
        self.pitch, self.yaw = sw["pitch"], sw.get("yaw", 0.0)
        self.actor, self._last = None, None
        bps = world.get_blueprint_library().filter(sw["prop"])
        if bps:
            self.actor = world.spawn_actor(bps[0], self._tf(0.0), attach_to=car)

    def _tf(self, deg):
        return carla.Transform(self.hub, carla.Rotation(pitch=self.pitch, yaw=self.yaw, roll=deg))

    def set(self, deg):
        if self.actor is None or (self._last is not None and abs(deg - self._last) < 0.2):
            return
        self._last = deg
        self.actor.set_transform(self._tf(deg))

    def destroy(self):
        if self.actor is not None:
            try:
                self.actor.destroy()
            except RuntimeError:
                pass
        self.actor = None
