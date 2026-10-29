"""Drone backends: how the drone actually exists in the world.

``kinematic``  plain CARLA (works on the stock 0.9.16 package). Every drone is
               a :class:`DroneUnit`: it flies a :class:`delivery.drone.Sortie`
               (one delivery round trip) or a :class:`delivery.drone.Transfer`
               (to / from the charge station), drifts in the wind, drains its
               battery, and can fail (collision, flat battery, motor failure,
               blown off course, parcel lost). It is drawn with the artist's
               props (built by delivery/assets/build_models.sh) or, without
               them, with debug lines.
``airsim``     CarlaAir (https://github.com/louiszengCN/CarlaAir), which runs
               CARLA 0.9.16 + AirSim 1.8.1 in one UE4.26 process. Kept for a
               later port; it has no battery / fleet model.

ドローンのバックエンド。``kinematic`` は素の CARLA で、各機は
:class:`DroneUnit`：配達往復（Sortie）または充電ステーションとの移動
（Transfer）を飛行し、風で流され、バッテリーを消費し、故障しうる（衝突・
電池切れ・モーター故障・ロスト・荷物落下）。
"""

import collections
import math
import threading
import time

import carla

from delivery import lmp_rig, rig
from delivery.battery import Battery
from delivery.drone import (Sortie, Transfer, Mission, TAKEOFF, ASCEND, LAND, DESCEND, DROP)
from delivery.wind import Drift

# the artist's drone, its rotor and a parcel that fits it (no collision)
# アーティスト製ドローン・ローターと、それに合う荷物（衝突なし）
DRONE_PROP = "static.prop.deliverydrone"
ROTOR_PROP = "static.prop.dronerotor"
PARCEL_PROP = "static.prop.parcel"
# the LastMile pod's drone (build_lastmile.sh): props, tether hook, parcel in the
# belly cradle / LastMile ポッドのドローン（ローター・フック・腹部の荷物）
LMP_DRONE, LMP_ROTOR, LMP_HOOK = ("static.prop.lmpdrone", "static.prop.lmpdroneprop",
                                  "static.prop.lmpdronehook")
LMP_PARCEL = "static.prop.lmpparcelm"
LMP_CRADLE_Z = lmp_rig.HOOK_Y - lmp_rig.BAIL - lmp_rig.PARCEL_M_H   # parcel bottom / 荷物の底
DRONE_HOOK_Z = 0.15         # tether hook above the model's origin [m]
# without the models the doorstep parcel is a stock box / モデル無し時は標準の箱
PARCEL_BLUEPRINTS = ("static.prop.box01", "static.prop.box02", "static.prop.box03")

# measured on Town10HD by CarlaAir (guide/COORDINATE_SYSTEMS.md); other maps
# need their own offsets -- see --airsim-offset / --airsim-calibrate
# CarlaAir が Town10HD で計測した値。他マップでは要再計測。
TOWN10HD_OFFSET = (172.20, -183.86, 27.45)

PARCEL_PICKUP_S = 60.0      # doorstep parcel is "collected" after this [s] / 受け取りまでの時間

# drawn quadcopter, used when the drone model is not installed.
# Debug lines render as glowing emitters: anything but pure black blooms to
# white on camera, so the frame is (0, 0, 0) and only the nav lights glow.
# ドローンモデル未導入時に毎フレーム描くクアッドコプター。デバッグ線は発光体
# として描かれ、純黒以外はカメラ上で白く光るため、機体は (0, 0, 0)。
ARM = 1.1                   # centre -> motor [m]
BLADE = 0.55                # half a rotor blade [m]
FRAME = carla.Color(0, 0, 0)
ROTOR = carla.Color(0, 0, 0)
NAV_PORT, NAV_STBD = carla.Color(255, 0, 0), carla.Color(0, 255, 0)
TRAIL = carla.Color(0, 160, 255)            # delivery flights / 配達飛行
TRAIL_TRANSFER = carla.Color(120, 255, 60)  # to / from the charge station / 充電移動
# the flight trail is drawn this many frames behind the drone: a glowing line
# that reaches the drone drowns the 0.77 m model in its bloom
# 軌跡はこのフレーム数だけ遅らせて描く（機体まで届く発光線は機体を覆い隠す）
TRAIL_LAG = 30
# The "tooltips": markers, labels and flight trails drawn in the CARLA window.
# Off by default; the dashboard (Settings > Delivery sim) or --tooltips turns
# them on. Tethers and drawn drones are part of the picture and always show.
# 「ツールチップ」（マーカー・ラベル・飛行軌跡）。既定はオフ。
OVERLAYS = {"on": False}

# what a flying drone can hit. Vehicles are left out: the pod's hidden car sits
# right under the drone's dock. The ground only counts away from take-off and
# landing (pads and the pod floor are on / near it).
# 飛行中に衝突しうるもの。車両は除外（ポッドの隠れた車が格納位置の真下にある）。
# 地面は離着陸時以外のみ。
OBSTACLE_HIT = ("Buildings", "Walls", "Fences", "Poles", "TrafficLight", "TrafficSigns",
                "Vegetation", "Bridge", "GuardRail", "Static", "Other")
GROUND_HIT = ("Roads", "Sidewalks", "Ground", "Terrain", "RoadLines")
DRONE_RADIUS = 0.4          # half the 0.77 m span / 機体の半幅 [m]
GRAVITY = 9.81

# DroneUnit.status / 機体の状態
DOCKED, RELOADING, DELIVERING = "docked", "reloading", "delivering"
TO_STATION, TO_VAN = "to_station", "to_van"
CHARGING, STANDBY = "charging", "standby"
FALLING, CRASHED = "falling", "crashed"
IN_VAN = (DOCKED, RELOADING)
AT_STATION = (CHARGING, STANDBY)
FLYING = (DELIVERING, TO_STATION, TO_VAN)


def solid_labels(names):
    return {getattr(carla.CityObjectLabel, n) for n in names
            if hasattr(carla.CityObjectLabel, n)}


def draw_quadcopter(debug, pos, yaw, spin, life):
    """Draw an X-frame quadcopter with spinning two-blade rotors.

    回転する 2 枚ブレードのローターを持つ X 型クアッドコプターを描く。
    """
    c = carla.Location(*pos)
    rad = math.radians(yaw)

    def at(fwd, right, up=0.0):
        return c + carla.Location(fwd * math.cos(rad) - right * math.sin(rad),
                                  fwd * math.sin(rad) + right * math.cos(rad), up)

    debug.draw_box(carla.BoundingBox(c, carla.Vector3D(0.42, 0.24, 0.1)),
                   carla.Rotation(yaw=yaw), thickness=0.14, color=FRAME, life_time=life)
    k = ARM / math.sqrt(2.0)
    motors = [at(k, k), at(-k, -k), at(k, -k), at(-k, k)]
    debug.draw_line(motors[0], motors[1], thickness=0.09, color=FRAME, life_time=life)
    debug.draw_line(motors[2], motors[3], thickness=0.09, color=FRAME, life_time=life)
    for i, m in enumerate(motors):
        a = math.radians(spin * (1 if i < 2 else -1) + 45 * i)
        d = carla.Location(BLADE * math.cos(a), BLADE * math.sin(a), 0.0)
        hub = m + carla.Location(z=0.14)
        debug.draw_line(hub + d, hub - d, thickness=0.05, color=ROTOR, life_time=life)
        debug.draw_point(m + carla.Location(z=0.07), size=0.13, color=FRAME, life_time=life)
    for side in (-0.3, 0.3):    # landing skids / 着陸脚
        debug.draw_line(at(0.45, side, -0.32), at(-0.45, side, -0.32), thickness=0.05,
                        color=FRAME, life_time=life)
    # nav lights on the front arms: red port (left), green starboard (right)
    # 前方アームの航法灯：左舷 赤、右舷 緑
    debug.draw_point(at(k, -k, 0.02), size=0.06, color=NAV_PORT, life_time=life)
    debug.draw_point(at(k, k, 0.02), size=0.06, color=NAV_STBD, life_time=life)


def _vec(v):
    return (v.x, v.y, v.z)


def _tf(pose):
    x, y, z, pitch, yaw, roll = pose
    return carla.Transform(carla.Location(x, y, z),
                           carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))


class Hazards(object):
    """What can go wrong in flight, and how often. / 飛行中の故障要因と頻度。

    Collisions, a flat battery and being blown off course always count.
    ``random`` switches the chance failures (motor, parcel torn off) on/off.
    衝突・電池切れ・ロストは常に有効。``random`` は確率的な故障の有無。
    """

    def __init__(self, random=True, motor_per_hour=1.0, parcel_wind=12.0,
                 parcel_loss_per_s=0.35, lost_drift=6.0):
        self.random = random
        self.motor_per_hour = motor_per_hour    # motor failures per flight hour
        self.parcel_wind = parcel_wind          # above this the tether can snap [m/s]
        self.parcel_loss_per_s = parcel_loss_per_s
        self.lost_drift = lost_drift            # pushed this far off: control lost [m]


class DroneUnit(object):
    """One physical drone: battery, flight, wind drift, failures and its props.

    Where its props hang: on a van (docked, reloading, delivering -- the van is
    parked while it delivers, so the flight is an offset from the van), or
    free in the world (charge station, flying to / from it, crashed). CARLA
    can't detach an actor, so moving between the two respawns the props.
    機体の小道具の所在：車に取り付け（格納・積込・配達中。配達中は車が停車
    しているので車からの相対で飛ぶ）、またはワールドに独立（ステーション・
    移動中・墜落）。CARLA は取り外しができないため、切替時に再生成する。
    """

    def __init__(self, uid, world, client, rng, battery_model, hazards, max_wind=11.0,
                 h_speed=12.0, v_speed=4.0, hover_s=4.0, drop_height=3.0, model=True,
                 kind="pod"):
        self.id = uid
        self.world, self.client, self.rng = world, client, rng
        self.bm, self.hz = battery_model, hazards
        self.h_speed, self.v_speed = h_speed, v_speed
        self.hover_s, self.drop_height = hover_s, drop_height
        self.battery = Battery(100.0)
        self.drift = Drift(max_wind)
        self.status = STANDBY
        self.note = ""              # what it is waiting for, for the dashboard
        self.van = None             # the van it serves / 担当車
        self.pad = None             # charge-station pad index / 充電パッド番号
        self.pad_target = None      # pad it is flying to / 向かっている充電パッド
        self.flight = None
        self.order = None           # the order it is carrying / 運搬中の注文
        self.parcel_lost = False
        self.pos, self.yaw = (0.0, 0.0, 0.0), 0.0
        self.fail_reason, self.down_t = None, None
        self.crash_detail = ""     # where / when it hit something
        self.trips = self.crashes = 0
        self.km = 0.0
        self.body, self.rotors, self.parcel, self.hook = None, [], None, None
        self.delivered = []         # doorstep / lost parcels with pickup times
        self._attached = None       # the car the props hang on, else free
        self._frame = None          # parked van's frame during a delivery
        self._start_z = 0.0
        self._spin = 0.0
        self._trail = collections.deque()
        self._fall_v, self._ground_z = 0.0, None
        self._drawn_parcel = False
        self._solid = solid_labels(OBSTACLE_HIT)
        self._ground = solid_labels(GROUND_HIT)
        lib = world.get_blueprint_library()
        have = lambda b: bool(lib.filter(b))
        # "lmp": the LastMile pod's drone, else the artist's earlier drone
        # "lmp" は LastMile ポッドのドローン、それ以外は以前のドローン
        self.lmp = kind == "lmp" and model and all(have(b) for b in (LMP_DRONE, LMP_ROTOR,
                                                                     LMP_HOOK, LMP_PARCEL))
        self.model = self.lmp or bool(model and all(have(b) for b in (DRONE_PROP, ROTOR_PROP,
                                                                      PARCEL_PROP)))
        if self.lmp:
            self._drone_bp, self._rotor_bp = lib.find(LMP_DRONE), lib.find(LMP_ROTOR)
            self._parcel_bp, self._hook_bp = lib.find(LMP_PARCEL), lib.find(LMP_HOOK)
        elif self.model:
            self._drone_bp, self._rotor_bp = lib.find(DRONE_PROP), lib.find(ROTOR_PROP)
            self._parcel_bp = lib.find(PARCEL_PROP)
        else:
            self._parcel_bp = next((lib.filter(b)[0] for b in PARCEL_BLUEPRINTS
                                    if lib.filter(b)), None)
        self.blueprint_id = "%s + %s parcel" % (
            (LMP_DRONE if self.lmp else DRONE_PROP) + " with spinning rotors"
            if self.model else "drawn quadcopter",
            self._parcel_bp.id if self._parcel_bp else "no")

    # -- status / 状態
    @property
    def flying(self):
        return self.status in FLYING

    @property
    def available(self):
        return self.status not in (FALLING, CRASHED)

    def world_pos(self):
        if self.status in IN_VAN and self.van is not None:
            return self.van.unit_pos(self)
        return self.pos

    def docked_in(self, pod):
        """In ``pod``, on a pad (not flying, not hanging off a van elsewhere)."""
        return (self.status in IN_VAN and self.van is not None and self.van.pod is pod
                and self._attached is not None)

    def _rotor_poses(self, spin):
        if self.lmp:
            return lmp_rig.prop_poses(spin)
        return rig.rotor_poses(spin)

    # -- props / 小道具
    def _clear_props(self):
        for a in self.rotors + [self.parcel, self.hook, self.body]:
            if a is not None:
                a.destroy()
        self.body, self.rotors, self.parcel, self.hook = None, [], None, None
        self._drawn_parcel = False

    def _spawn_props(self, tf, parent=None):
        self._clear_props()
        self._attached = parent
        if not self.model:
            return
        if parent is not None:
            self.body = self.world.spawn_actor(self._drone_bp, tf, attach_to=parent)
        else:
            self.body = self.world.spawn_actor(self._drone_bp, tf)
        self.rotors = [self.world.spawn_actor(self._rotor_bp, _tf(p), attach_to=self.body)
                       for p in self._rotor_poses(0.0)]
        if self.lmp:
            self.hook = self.world.spawn_actor(self._hook_bp, _tf(lmp_rig.hook_pose(0.0)),
                                               attach_to=self.body)

    def _free_here(self):
        """Props loose in the world at the current position (if they were on a van)."""
        if self._attached is not None or (self.model and self.body is None):
            self._spawn_props(carla.Transform(carla.Location(*self.pos),
                                              carla.Rotation(yaw=self.yaw)))

    def _hang(self):
        if not self.model:
            self._drawn_parcel = True
        elif self.parcel is None and self.body is not None:
            z = LMP_CRADLE_Z if self.lmp else -rig.PARCEL_BELOW
            self.parcel = self.world.try_spawn_actor(
                self._parcel_bp, carla.Transform(carla.Location(z=z)), attach_to=self.body)

    def _unhang(self):
        self._drawn_parcel = False
        if self.parcel is not None:
            self.parcel.destroy()
            self.parcel = None

    def _drop_parcel_at(self, x, y, z, t):
        if self._parcel_bp is None:
            return
        if not self.model:
            z += 0.3            # stock box: origin at its centre / 標準の箱は中心原点
        box = self.world.try_spawn_actor(self._parcel_bp,
                                         carla.Transform(carla.Location(x, y, z)))
        if box is not None:
            box.set_simulate_physics(False)
            self.delivered.append((box, t + PARCEL_PICKUP_S))

    def collect_parcels(self, t):
        """Remove doorstep parcels once the customer has had time to take them."""
        for box, due in [d for d in self.delivered if d[1] <= t]:
            box.destroy()
        self.delivered = [d for d in self.delivered if d[1] > t]

    # -- placement / 配置
    def dock_into(self, van):
        """Park inside ``van`` (props attached to it). / 車内に格納。"""
        self.van, self.pad = van, None
        d = van.dock_for(self)
        self._spawn_props(carla.Transform(carla.Location(d.x, d.y, d.z)), parent=van.car)
        self.status, self.note, self.flight = DOCKED, "", None
        self.pos = van.unit_pos(self)
        self.drift.reset()

    def park(self, pad, pos, status=CHARGING):
        """Sit on charge-station pad ``pad`` at world ``pos``. / 充電パッドに駐機。"""
        self.van, self.pad, self.pos = None, pad, tuple(pos)
        self._spawn_props(carla.Transform(carla.Location(*pos), carla.Rotation(yaw=self.yaw)))
        self.status, self.note, self.flight = status, "", None
        self.drift.reset()

    # -- missions / 任務
    def start_trip(self, van, order, hover_point, cruise_z):
        """One delivery round trip from ``van`` (parked, hatch open)."""
        self.order, self.parcel_lost = order, False
        self.flight = Sortie(van.dock_pos, [hover_point], drop_height=self.drop_height,
                             h_speed=self.h_speed, v_speed=self.v_speed,
                             hover_s=self.hover_s, reload_s=0.0, leg_z=[cruise_z])
        tf = van.car.get_transform()
        self._frame = (_vec(tf.location), _vec(tf.get_forward_vector()),
                       _vec(tf.get_right_vector()), _vec(tf.get_up_vector()))
        self.yaw = tf.rotation.yaw
        self.pos = self._start = self.flight.pos
        self._start_z = self.pos[2]
        self.status, self.note = DELIVERING, "order #%d" % order.id
        self.trips += 1
        self.drift.reset()
        self._hang()

    def start_mission(self, van, order, hover_point, cruise_z, meet, meet_z):
        """Launch from ``van``'s raised pad: deliver ``order``, then fly to
        ``meet`` and wait for the pod (drone.Mission). The props go free in the
        world: the pod may drive off while the drone is out.
        ポッドから発進して配達し、合流地点へ。機体は独立（ポッドは移動できる）。"""
        start = van.unit_pos(self)
        self.order, self.parcel_lost = order, False
        self.flight = Mission(start, hover_point, cruise_z, meet, meet_z, h_speed=self.h_speed,
                              v_speed=self.v_speed, hover_s=self.hover_s,
                              drop_height=self.drop_height)
        self.pos, self._start_z = tuple(start), start[2]
        self.yaw = van.car.get_transform().rotation.yaw
        self._free_here()           # same pose, now free of the car / 車から独立
        self._hang()
        self.status, self.note = DELIVERING, "order #%d" % order.id
        self.trips += 1
        self.drift.reset()
        self._trail.clear()

    def start_transfer(self, target_fn, cruise_z, to_van=None):
        """Fly empty to ``target_fn()``: the station (``to_van`` None) or a van."""
        self.pos = self.world_pos()
        self._free_here()
        self.flight = Transfer(self.pos, target_fn, cruise_z, self.h_speed, self.v_speed)
        self._start_z = self.pos[2]
        if to_van is None:
            self.van, self.status, self.note = None, TO_STATION, "to charge"
        else:
            self.van, self.status, self.note = to_van, TO_VAN, "to van %d" % to_van.vid
        self.pad = None
        self.drift.reset()

    def repair(self, pad, pos, level):
        """Back in service at the station with a fresh but part-charged battery."""
        self.battery.level = level
        self.fail_reason = self.down_t = None
        self.park(pad, pos, CHARGING)

    # -- per tick / 毎 tick
    def update(self, t, dt, wind):
        if self.status in FLYING:
            return self._fly(t, dt, wind)
        if self.status == FALLING:
            self._fall(t, dt)
        return []

    def _fly(self, t, dt, wind):
        f = self.flight
        events = []
        for ev, _ in f.update(dt):
            if ev == "delivered":
                if self.parcel_lost:
                    events.append(("parcel_lost", self.order))
                else:
                    self._unhang()
                    # left under where it hovered (the doorstep, or the
                    # nearest clear spot to it) / ホバリング地点の真下に置く
                    h, d = f.drops[0], self.order.drop
                    self._drop_parcel_at(h[0], h[1], d[2], t)
                    events.append(("delivered", self.order))
            elif ev == "landed":
                events.append(("trip_done", self.order))
            elif ev == "arrived":
                events.append(("arrived", None))
            elif ev == "at_meet":
                events.append(("at_meet", None))
        if getattr(f, "holding", False) and self.van is not None:
            self.note = "waiting for pod %d" % self.van.vid
        wspeed = math.hypot(*wind)
        assist = f.state == LAND or f.done or (f.state == TAKEOFF and
                                               f.pos[2] - self._start_z < 2.5)
        d = self.drift.step(dt, wind, assist=assist)
        actual = (f.pos[0] + d[0], f.pos[1] + d[1], f.pos[2])
        self.battery.drain(dt, self.bm.rate(f.loaded, f.state in (TAKEOFF, ASCEND), wspeed))
        crash = self._hazard(dt, f, actual)
        self.km += math.dist(actual, self.pos) / 1000.0
        self.pos = actual
        h = f.heading()
        if h is not None:
            self.yaw = h
        if crash is not None:
            events.append(("crashed", crash))
            self._crash(t, crash)
            return events
        if (f.state == DROP and self.hz.random and not self.parcel_lost and
                wspeed > self.hz.parcel_wind and
                self.rng.random() < self.hz.parcel_loss_per_s * dt):
            # the gust tears the parcel off the tether; it lands downwind
            # 突風で荷物が吊りひもから外れ、風下に落ちる
            self.parcel_lost = True
            self._unhang()
            g = self._ground_below(actual)
            self._drop_parcel_at(actual[0] + wind[0] * 0.4, actual[1] + wind[1] * 0.4, g, t)
        if f.done and self.status == DELIVERING:
            self.status, self.note, self.flight = DOCKED, "", None
            self.drift.reset()
        self._render(f)
        self._draw_trail(f)
        return events

    def _hazard(self, dt, f, actual):
        """The reason this tick ends the flight, or None. / 墜落理由（無ければ None）。"""
        if self.battery.dead:
            return "battery_depleted"
        mx, my, mz = (actual[i] - self.pos[i] for i in range(3))
        horiz = math.hypot(mx, my)
        if horiz + abs(mz) > 1e-4:
            # one ray along this tick's motion, reaching a body radius ahead
            # この tick の移動に沿い、機体半径だけ先まで伸ばしたレイ
            ex, ey = ((mx / horiz * DRONE_RADIUS, my / horiz * DRONE_RADIUS)
                      if horiz > 1e-4 else (0.0, 0.0))
            a = carla.Location(self.pos[0], self.pos[1], self.pos[2] + 0.2)
            b = carla.Location(actual[0] + ex, actual[1] + ey, actual[2] + 0.2)
            solid = self._solid
            if f.state not in (TAKEOFF, LAND, DESCEND):
                solid = solid | self._ground
            for h in self.world.cast_ray(a, b):
                if h.label in solid:
                    self.crash_detail = "%s at z %.1f, during %s" % (
                        h.label, h.location.z, f.state)
                    return "collision:%s" % str(h.label)
        if self.drift.offset > self.hz.lost_drift:
            return "blown_off_course"
        if (self.hz.random and
                self.rng.random() < self.hz.motor_per_hour / 3600.0 * dt):
            return "motor_failure"
        return None

    def _ground_below(self, p):
        hits = self.world.cast_ray(carla.Location(p[0], p[1], p[2]),
                                   carla.Location(p[0], p[1], p[2] - 600.0))
        return hits[0].location.z if hits else p[2] - 30.0

    def _crash(self, t, reason):
        self.status, self.fail_reason, self.note = FALLING, reason, reason
        self.crashes += 1
        self.flight, self.order = None, None
        had_parcel = self.parcel is not None or self._drawn_parcel
        self._free_here()
        self._unhang()
        self._ground_z = self._ground_below(self.pos)
        self._fall_v = 0.0
        if had_parcel:
            self._drop_parcel_at(self.pos[0] + 0.6, self.pos[1], self._ground_z, t)

    def _fall(self, t, dt):
        self._fall_v += GRAVITY * dt
        z = self.pos[2] - self._fall_v * dt
        if z <= self._ground_z:
            z = self._ground_z
            self.status, self.down_t = CRASHED, t
        self.pos = (self.pos[0], self.pos[1], z)
        if self.model and self.body is not None:
            tilt = 70.0 if self.status == CRASHED else 25.0
            self.client.apply_batch([carla.command.ApplyTransform(self.body.id, carla.Transform(
                carla.Location(*self.pos), carla.Rotation(pitch=12.0, yaw=self.yaw, roll=tilt)))])
        elif not self.model:
            draw_quadcopter(self.world.debug, self.pos, self.yaw, 0.0, 0.1)

    # -- drawing / 描画
    def _render(self, f):
        if not self.model:
            self._spin = (self._spin + 47.0) % 360.0
            draw_quadcopter(self.world.debug, self.pos, self.yaw, self._spin, 0.1)
            if self._drawn_parcel:
                c = carla.Location(self.pos[0], self.pos[1], self.pos[2] - 1.0)
                self.world.debug.draw_box(carla.BoundingBox(c, carla.Vector3D(0.3, 0.3, 0.25)),
                                          carla.Rotation(yaw=self.yaw), thickness=0.1,
                                          color=FRAME, life_time=0.1)
            return
        if self.body is None:
            return
        if self._attached is not None:
            if f.done:          # exactly back on the dock / 格納位置に正確に戻す
                d = self.van.dock_for(self)
                tf = carla.Transform(carla.Location(d.x, d.y, d.z))
            else:
                tf = _tf(rig.to_parent(*(self._frame + (self.pos, self.yaw))))
        else:
            tf = carla.Transform(carla.Location(*self.pos), carla.Rotation(yaw=self.yaw))
        cmds = [carla.command.ApplyTransform(self.body.id, tf)]
        if not f.done:      # rotors idle once landed / 着地後は停止
            self._spin += rig.ROTOR_STEP_DEG
            cmds += [carla.command.ApplyTransform(r.id, _tf(p))
                     for r, p in zip(self.rotors, self._rotor_poses(self._spin))]
        if self.lmp:
            self._lower_parcel(f, cmds)
        self.client.apply_batch(cmds)
        if self.parcel is not None and not self.lmp:     # the tether / 吊りひも
            x, y, z = self.pos
            self.world.debug.draw_line(
                carla.Location(x, y, z + DRONE_HOOK_Z),
                carla.Location(x, y, z - rig.PARCEL_BELOW + rig.PARCEL_HEIGHT),
                thickness=0.01, color=FRAME, life_time=0.075)

    def _lower_parcel(self, f, cmds):
        """LastMile drone: over the drop hover, the winch lowers the parcel on
        its tether to the ground (the HTML's tethered drop-off); it rides in
        the belly cradle otherwise. / 投下ホバリング中に荷物をひもで地面まで下ろす。"""
        if self.parcel is None:
            return
        z = LMP_CRADLE_Z
        if f.state == DROP and self.order is not None:
            u = lmp_rig.smooth(1.0 - getattr(f, "_timer", 0.0) / max(f.hover_s, 1e-3))
            z = lmp_rig.lerp(LMP_CRADLE_Z, -(self.pos[2] - self.order.drop[2]), u)
        cmds.append(carla.command.ApplyTransform(self.parcel.id, carla.Transform(carla.Location(z=z))))
        hook = z + lmp_rig.PARCEL_M_H + lmp_rig.BAIL
        if f.state == DROP:
            x, y, zz = self.pos
            self.world.debug.draw_line(carla.Location(x, y, zz + lmp_rig.WINCH_Y),
                                       carla.Location(x, y, zz + hook),
                                       thickness=0.008, color=FRAME, life_time=0.075)
        if self.hook is not None:
            cmds.append(carla.command.ApplyTransform(
                self.hook.id, _tf(lmp_rig.hook_pose(max(lmp_rig.HOOK_Y - hook, 0.0)))))

    def _draw_trail(self, f):
        self._trail.append(self.pos)
        if len(self._trail) > TRAIL_LAG and not OVERLAYS["on"]:
            self._trail.popleft()
        elif len(self._trail) > TRAIL_LAG:
            a = self._trail.popleft()
            color = TRAIL if self.status == DELIVERING else TRAIL_TRANSFER
            self.world.debug.draw_line(carla.Location(*a), carla.Location(*self._trail[0]),
                                       thickness=0.03, color=color, life_time=30.0)
        if f.done:
            self._trail.clear()

    def draw_idle(self, life):
        """Drawn drones at the station / in a stopped van (no props to show)."""
        if not self.model and self.status not in FLYING + (FALLING,):
            p = self.world_pos()
            draw_quadcopter(self.world.debug, (p[0], p[1], p[2] + 0.35), self.yaw, 0.0, life)

    def destroy(self):
        for box, _ in self.delivered:
            box.destroy()
        self.delivered = []
        self._clear_props()


class AirSimDrone(object):
    """Real AirSim multirotor inside CarlaAir. / CarlaAir 内の AirSim 機体。

    UNVERIFIED on this machine: CarlaAir is not installed here. The calls are
    the standard AirSim 1.8.1 API that CarlaAir's README uses.
    本機では未検証（CarlaAir 未インストール）。AirSim 1.8.1 標準 API を使用。
    """

    def __init__(self, offset, host="127.0.0.1", port=41451, vehicle_name="",
                 cruise_alt=40.0, h_speed=12.0, v_speed=4.0, hover_s=4.0,
                 drop_height=3.0, reload_s=5.0):
        import airsim  # only needed for this backend / このバックエンドのみ必要
        self.airsim = airsim
        self.client = airsim.MultirotorClient(ip=host, port=port)
        self.client.confirmConnection()
        self.name = vehicle_name
        self.offset = offset
        self.cruise_alt, self.h_speed, self.v_speed = cruise_alt, h_speed, v_speed
        self.hover_s, self.drop_height, self.reload_s = hover_s, drop_height, reload_s
        self.client.enableApiControl(True, vehicle_name)
        self.client.armDisarm(True, vehicle_name)
        self._thread = None
        self._events = []
        self._lock = threading.Lock()
        self.distance = 0.0
        self.error = None

    # CARLA world (z up) <-> AirSim NED (z down), CarlaAir COORDINATE_SYSTEMS.md
    def to_ned(self, p):
        ox, oy, oz = self.offset
        return (p[0] + ox, p[1] + oy, -p[2] + oz)

    def to_carla(self, n):
        ox, oy, oz = self.offset
        return (n[0] - ox, n[1] - oy, oz - n[2])

    @property
    def position(self):
        p = self.client.simGetVehiclePose(self.name).position
        return self.to_carla((p.x_val, p.y_val, p.z_val))

    @property
    def flying(self):
        return self._thread is not None and self._thread.is_alive()

    def dock(self, pad, yaw=0.0):
        """Teleport onto the car roof (only while landed). / 着陸中のみ屋根へ移動。"""
        if self.flying:
            return
        a = self.airsim
        n = self.to_ned((pad[0], pad[1], pad[2] + 0.3))
        pose = a.Pose(a.Vector3r(*n), a.to_quaternion(0.0, 0.0, math.radians(yaw)))
        self.client.simSetVehiclePose(pose, True, self.name)

    def launch(self, home_fn, drops, leg_z=None):
        self._thread = threading.Thread(target=self._fly,
                                        args=(home_fn, list(drops), leg_z), daemon=True)
        self._thread.start()

    def _fly(self, home_fn, drops, leg_z=None):
        """One round trip per drop, like :class:`delivery.drone.Sortie`."""
        c, name = self.client, self.name
        try:
            home = home_fn()
            if leg_z is None:
                leg_z = [home[2] + self.cruise_alt] * len(drops)
            # per-trip CARLA z -> NED z / 往復ごとの z を NED に変換
            ned_z = [self.to_ned((0, 0, z))[2] for z in leg_z]
            last = self.position
            for i, d in enumerate(drops):
                if i:
                    with self._lock:
                        self._events.append(("reloading", i))
                    time.sleep(self.reload_s)
                c.takeoffAsync(vehicle_name=name).join()
                c.moveToZAsync(ned_z[i], self.v_speed, vehicle_name=name).join()
                x, y, _ = self.to_ned(d)
                c.moveToPositionAsync(x, y, ned_z[i], self.h_speed, vehicle_name=name).join()
                c.moveToZAsync(self.to_ned((0, 0, d[2] + self.drop_height))[2],
                               self.v_speed, vehicle_name=name).join()
                time.sleep(self.hover_s)
                with self._lock:
                    self._events.append(("delivered", i))
                c.moveToZAsync(ned_z[i], self.v_speed, vehicle_name=name).join()
                h = home_fn()
                x, y, _ = self.to_ned(h)
                c.moveToPositionAsync(x, y, ned_z[i], self.h_speed, vehicle_name=name).join()
                h = home_fn()
                x, y, z = self.to_ned((h[0], h[1], h[2] + 1.0))
                c.moveToPositionAsync(x, y, z, self.v_speed, vehicle_name=name).join()
                c.landAsync(vehicle_name=name).join()
                now = self.position
                self.distance += math.dist(last, now)
                last = now
        except Exception as e:  # surfaced by update() / update() で報告
            self.error = e
        with self._lock:
            self._events.append(("landed", None))

    def update(self, dt):
        with self._lock:
            ev, self._events = self._events, []
        return ev

    def destroy(self):
        try:
            self.client.armDisarm(False, self.name)
            self.client.enableApiControl(False, self.name)
        except Exception:
            pass


def calibrate_airsim_offset(world, air_client, vehicle_name=""):
    """Offsets from a drone visible to BOTH clients (CarlaAir's own method).

    Looks for the AirSim drone among CARLA actors; returns (ox, oy, oz) or None.
    両クライアントから見えるドローンの位置差からオフセットを求める。
    """
    for a in world.get_actors():
        tid = a.type_id.lower()
        if any(k in tid for k in ("drone", "airsim", "multirotor", "uav")):
            loc = a.get_location()
            p = air_client.simGetVehiclePose(vehicle_name).position
            return (p.x_val - loc.x, p.y_val - loc.y, p.z_val + loc.z)
    return None
