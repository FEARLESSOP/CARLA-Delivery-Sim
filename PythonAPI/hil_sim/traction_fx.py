#!/usr/bin/env python3
"""traction_fx.py - skid marks behind the hero's sliding tyres.

Runs next to the driving window and only reads the world (it never ticks it):
  ../../carla-venv/bin/python traction_fx.py      (run_pipeline.py starts it)

A slide is judged from what the whole car does, not from the per-tyre slip
readings alone: CARLA's tyre slip (Vehicle.get_telemetry_data()) jumps to
0.3-0.5 on a gripping car whenever throttle and brake change every tick, as
the autopilot (TM) does, which drew skid marks behind a car on autopilot
(measured 2026-11-06: TM-driven Tesla and RX-7, median slip ratio 0.31-0.33
at 30 km/h on light throttle).

- Sideways: the slip angle at the rear axle (velocity there vs heading, from
  the body's velocity and yaw rate). A gripping car stays under ~7 deg even in
  tight junction turns (TM: p99 5.5, max 6.7); a drift or spin goes past 10.
  Above 8 deg all tyres mark, harder the further past.
- Wheelspin / locked wheels: a tyre's slip ratio over 0.3 for 0.3 s, and only
  with a hard input (throttle >= 0.9, brake >= 0.9 or the handbrake): the
  autopilot never gets there, a full-throttle launch or an emergency stop does.
Nothing below 5 km/h (noisy readings).

Each tyre's mark is a band of thin lines side by side across the tyre's
width, not one thick line: CARLA's debug lines are round tubes (a thick one
lay on the road like a black hose, stepped at every joint) and only pure
black does not glow. A light slide draws a few lines, a hard one all of them;
each line lives a random 35-100 % of --mark-life, so a mark thins out and
fades instead of vanishing at once, and short random gaps make it patchy
like rubber on asphalt.

タイヤが滑った時に路面にスキッドマークを描く。ワールドを読むだけで tick は
しない。滑りの判定はテレメトリのスリップ角・スリップ率。
"""

import argparse
import math
import random
import time

import carla

SPEED_MIN = 5.0 / 3.6        # below this the slip readings are noise / 低速はノイズ
REAR_SLIP, REAR_SPAN = 8.0, 12.0     # rear-axle slip angle (deg): sliding sideways / 横滑り
LONG_SLIP, LONG_SPAN = 0.3, 0.4      # tyre slip ratio: wheelspin / lock / 空転・ロック
LONG_HOLD = 0.3                      # ... held this long (s) / この時間続いたら
HARD_PEDAL = 0.9                     # ... with throttle or brake at least this / 強い踏み込み
MARK_COLOR = carla.Color(0, 0, 0)    # debug lines are self-lit: even 22 glows white / 黒以外は発光する
MARK_Z = 0.02                        # just above the road / 路面のすぐ上
TYRE_W = 0.22                        # mark width (m) / 跡の幅
STRIPES = 7                          # thin lines across it / 幅方向の細線の数
STRIPE_T = 0.018                     # each line's thickness (m): flat from any angle / 細線の太さ
FADE_IN = 2.0                        # a mark builds up over its first metres / 跡の出始めの長さ
MIN_STEP = 0.25                      # draw once the tyre moved this far (m) / この距離進んだら描く
LIFE_MIN = 0.35                      # a line lives 35-100 % of --mark-life: the mark fades / 寿命の幅


def stripes(k, rng):
    """Lateral offsets (m, across the mark) of the lines drawn for intensity k:
    the middle ones always, more towards the edges the harder the slide, a few
    skipped at random (patchy). / 強さ k で描く細線の横位置。"""
    n = max(2, min(STRIPES, int(round(2 + k * (STRIPES - 2)))))
    order = sorted(range(STRIPES), key=lambda j: abs(j - (STRIPES - 1) / 2.0))[:n]
    keep = 0.75 + 0.25 * k
    return [(j - (STRIPES - 1) / 2.0) * TYRE_W / (STRIPES - 1) for j in sorted(order) if rng.random() < keep]


def intensity(speed, rear_slip, long_=0.0, long_held=0.0, hard=False):
    """0 = gripping, up to 1 = full slide. ``rear_slip``: rear-axle slip angle
    (deg); ``long_``: this tyre's slip ratio, ``long_held``: seconds it has been
    over LONG_SLIP; ``hard``: a hard pedal or the handbrake.
    0 はグリップ、1 は大きく滑っている。"""
    if speed < SPEED_MIN:
        return 0.0
    k = (abs(rear_slip) - REAR_SLIP) / REAR_SPAN
    if hard and long_held >= LONG_HOLD:
        k = max(k, (abs(long_) - LONG_SLIP) / LONG_SPAN)
    return 0.0 if k <= 0.0 else min(1.0, 0.15 + k)


def rear_slip_deg(vel, fwd, yaw_rate_deg, rear_x):
    """Slip angle at the rear axle (``rear_x`` m along the car, negative):
    the body's velocity plus the yaw rate's share there, against the heading.
    UE axes: yaw turns x towards y, so a point at p moves by w x p.
    後軸の横滑り角（車体速度とヨーレートから）。"""
    w = math.radians(yaw_rate_deg)
    px, py = rear_x * fwd.x, rear_x * fwd.y
    vx, vy = vel.x - w * py, vel.y + w * px
    if math.hypot(vx, vy) < 0.5:
        return 0.0
    return math.degrees(math.atan2(-fwd.y * vx + fwd.x * vy, fwd.x * vx + fwd.y * vy))


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
        self.held = [0.0] * len(self.offsets or ())        # s each tyre's slip ratio is over / 空転の継続時間
        self.run = [0.0] * len(self.offsets or ())         # m of the current mark / 現在の跡の長さ
        self.rng = getattr(self, "rng", None) or random.Random()
        rear = [o.x for o in (self.offsets or ()) if o.x < 0.0]
        self.rear_x = sum(rear) / len(rear) if rear else 0.0
        if car is not None:
            print("[traction_fx] hero %s (id %d)" % (car.type_id, car.id))

    def alive(self):
        return self.car is not None and self.car.is_alive

    # -- one tick / 1 tick
    def step(self, dt=0.05):
        if not self.alive():
            self.attach(find_hero(self.world, self.args.role))
            if self.car is None:
                return
        try:
            tel = self.car.get_telemetry_data()
            tf = self.car.get_transform()
            vel = self.car.get_velocity()
            yaw_rate = self.car.get_angular_velocity().z
            ctl = self.car.get_control()
        except RuntimeError:
            self.car = None
            return
        speed = math.sqrt(vel.x * vel.x + vel.y * vel.y)
        rear = rear_slip_deg(vel, tf.get_forward_vector(), yaw_rate, self.rear_x)
        hard = ctl.throttle >= HARD_PEDAL or ctl.brake >= HARD_PEDAL or ctl.hand_brake
        wheels = tel.wheels
        for i, off in enumerate(self.offsets[:len(wheels)]):
            w = wheels[i]
            self.held[i] = self.held[i] + dt if abs(w.long_slip) > LONG_SLIP else 0.0
            k = intensity(speed, rear, w.long_slip, self.held[i], hard)
            if k <= 0.0:
                self.last[i] = None
                continue
            here = tf.transform(carla.Location(off.x, off.y, off.z))   # transforms in place / 引数を書き換える
            a = self.last[i]
            if a is None or a.distance(here) >= 3.0:          # a new mark, or across a teleport / 新しい跡
                self.last[i], self.run[i] = here, 0.0
                continue
            step = a.distance(here)
            if step < MIN_STEP:
                continue
            self.run[i] += step
            self.draw_band(a, here, k * min(1.0, 0.3 + self.run[i] / FADE_IN))
            self.last[i] = here

    def draw_band(self, a, b, k):
        """The mark from a to b: thin lines across it, perpendicular to the way
        the tyre moved. / a から b への跡（移動方向に直交する細線の束）。"""
        dx, dy = b.x - a.x, b.y - a.y
        n = math.hypot(dx, dy) or 1.0
        px, py = -dy / n, dx / n
        life = self.args.mark_life
        for off in stripes(k, self.rng):
            self.world.debug.draw_line(
                carla.Location(a.x + px * off, a.y + py * off, a.z),
                carla.Location(b.x + px * off, b.y + py * off, b.z),
                thickness=STRIPE_T, color=MARK_COLOR,
                life_time=life * self.rng.uniform(LIFE_MIN, 1.0))


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
                fx.step(snap.timestamp.delta_seconds or 0.05)
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
