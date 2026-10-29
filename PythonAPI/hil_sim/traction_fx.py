#!/usr/bin/env python3
"""traction_fx.py - skid marks behind the hero's sliding tyres.

Runs next to the driving window and only reads the world (it never ticks it):
  ../../carla-venv/bin/python traction_fx.py      (run_pipeline.py starts it)

Each tick the hero's per-wheel slip comes from Vehicle.get_telemetry_data().
A tyre is sliding when the car moves faster than 5 km/h and its slip angle
is over 9 deg or its slip ratio over 0.3 (measured on this rig: normal
cornering stays at 2-6 deg, a handbrake drift reaches 12-28 deg, a hard
launch or a locked wheel 0.3-1.0). How far past that sets the intensity.

Skid marks are dark streaks on the road from each sliding tyre's contact
patch (debug lines, wider when sliding harder), fading after --mark-life.

タイヤが滑った時に路面にスキッドマークを描く。ワールドを読むだけで tick は
しない。滑りの判定はテレメトリのスリップ角・スリップ率。
"""

import argparse
import math
import time

import carla

SPEED_MIN = 5.0 / 3.6        # below this the slip readings are noise / 低速はノイズ
LAT_SLIP, LAT_SPAN = 9.0, 12.0
LONG_SLIP, LONG_SPAN = 0.3, 0.4
MARK_COLOR = carla.Color(0, 0, 0)    # debug lines are self-lit: even 22 glows white / 黒以外は発光する
MARK_Z = 0.03


def intensity(speed, lat, long_):
    """0 = gripping, up to 1 = full slide. / 0 はグリップ、1 は大きく滑っている。"""
    if speed < SPEED_MIN:
        return 0.0
    k = max((abs(lat) - LAT_SLIP) / LAT_SPAN, (abs(long_) - LONG_SLIP) / LONG_SPAN)
    return 0.0 if k <= 0.0 else min(1.0, 0.15 + k)


def find_hero(world, role="hero"):
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == role:
            return a
    return None


def wheel_offsets(car):
    """Contact patch of each wheel (FL, FR, RL, RR, ...) in the car's frame. / 各輪の接地点。"""
    tf = car.get_transform()
    inv = tf.get_inverse_matrix()
    bb = car.bounding_box
    ground = bb.location.z - bb.extent.z
    out = []
    for w in car.get_physics_control().wheels:
        p = w.position / 100.0                 # world, cm -> m / ワールド座標 cm
        x = inv[0][0] * p.x + inv[0][1] * p.y + inv[0][2] * p.z + inv[0][3]
        y = inv[1][0] * p.x + inv[1][1] * p.y + inv[1][2] * p.z + inv[1][3]
        out.append(carla.Location(x, y, ground + MARK_Z))
    return out


class SkidMarks(object):
    def __init__(self, world, args):
        self.world, self.args = world, args
        self.car = self.offsets = None
        self.last = []                   # last contact point while sliding / 前回の接地点

    # -- hero / hero 車
    def attach(self, car):
        self.car = car
        self.offsets = wheel_offsets(car) if car is not None else None
        self.last = [None] * len(self.offsets or ())       # trucks have 6+ wheels / トラックは 6 輪以上
        if car is not None:
            print("[traction_fx] hero %s (id %d)" % (car.type_id, car.id))

    def alive(self):
        return self.car is not None and self.car.is_alive

    # -- one tick / 1 tick
    def step(self):
        if not self.alive():
            self.attach(find_hero(self.world, self.args.role))
            if self.car is None:
                return
        try:
            tel = self.car.get_telemetry_data()
            tf = self.car.get_transform()
            vel = self.car.get_velocity()
        except RuntimeError:
            self.car = None
            return
        speed = math.sqrt(vel.x * vel.x + vel.y * vel.y)
        wheels = tel.wheels
        for i, off in enumerate(self.offsets[:len(wheels)]):
            w = wheels[i]
            k = intensity(speed, w.lat_slip, w.long_slip)
            if k <= 0.0:
                self.last[i] = None
                continue
            here = tf.transform(carla.Location(off.x, off.y, off.z))   # transforms in place / 引数を書き換える
            if self.last[i] is not None and self.last[i].distance(here) < 3.0:   # not across a teleport
                self.world.debug.draw_line(
                    self.last[i], here, thickness=0.12 + 0.1 * k, color=MARK_COLOR,
                    life_time=self.args.mark_life)
            self.last[i] = here


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--mark-life", type=float, default=25.0,
                    help="seconds a skid mark stays on the road")
    ap.add_argument("--role", default="hero", help="role_name of the car to watch")
    args = ap.parse_args()

    while True:
        try:
            client = carla.Client(args.host, args.port)
            client.set_timeout(10.0)
            world = client.get_world()
            fx = SkidMarks(world, args)
            print("[traction_fx] running")
            map_id = world.get_map().name
            while True:
                snap = world.wait_for_tick(10.0)
                fx.step()
                if fx.car is None and snap.frame % 200 == 0 and \
                        client.get_world().get_map().name != map_id:
                    break                                  # map changed / マップ変更
        except KeyboardInterrupt:
            return
        except RuntimeError as e:
            print("[traction_fx] %s -> reconnecting / 再接続" % e)
            time.sleep(2.0)


if __name__ == "__main__":
    main()
