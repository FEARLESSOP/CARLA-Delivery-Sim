"""Live sensor readings of a delivery drone, from what the sim knows (no carla import).

The sim flies each drone kinematically (delivery/drone.py) and knows its
position, heading, flight phase, battery level, the wind and the controller's
wind estimate (delivery/wind.py), what it carries and where its pod is. This
module turns that into what the drone's own sensors would report, so the
dashboard's Drones page can show a real-looking sensor suite: plausible
values, units, noise and a health flag per sensor (ok / warn / fail / off).
It runs at the live-state publish rate (2 Hz) and costs ~0.05 ms per drone.

The suite fits THIS drone: the LastMile LMP drone, a ~3 kg quad-X with a
0.97 m rotor-tip span (0.67 m motor to motor), a belly winch with a tether
hook and a hot-swap battery. It is what small delivery drones carry today:

=================  ==========================================================
gnss               dual-band GNSS with RTK (+ the magnetometer) in the puck on
                   top. FlyCart 30: RTK, +-10 cm hover with RTK [1].
mag                3-axis magnetometer (compass), in the GNSS puck.
imu                flight controller IMU (accelerometer + gyro), inside the
                   fuselage; reads ~-T/m along the body z axis, ~0 in free fall.
baro               barometer on the flight controller: pressure altitude.
rangefinder        downward lidar rangefinder, 0.2-40 m (the black puck under
                   the nose of the model); FlyCart 30 lists altitude detection
                   1.5-200 m with its radar [1].
flow               downward optical-flow camera (velocity over ground, low
                   altitude); Wing's patents describe a downward camera for
                   visual navigation / positioning [4].
obstacle           front and rear stereo vision pairs (+ the downward lidar):
                   FlyCart 30 senses obstacles with front / rear radars and
                   binocular vision, 90 x 106 deg FOV [1]; Amazon MK30 scans
                   with a series of cameras [5].
air                airspeed / wind ESTIMATE (no pitot on small multirotors: the
                   EKF estimates wind from tilt and drift).
battery            smart battery (BMS): 6S Li-ion, 10 Ah, voltage, current,
                   temperature, state of charge, time left. Hot-swap like the
                   FlyCart 30's [1].
motors             4 x motor + ESC telemetry: RPM, current, temperature.
winch              winch, tether length and load cell. FlyCart 30 winch: 20 m
                   cable, 0.8 m/s [1]; Wing estimates tether tension from
                   motor current / force feedback [4]; Zipline P2 lowers a
                   droid on a tether [2].
link               C2 datalink to the pod (5.8 GHz) + LTE backup: RSSI, SNR,
                   latency, packet loss.
remote_id          Remote ID broadcast (mandatory in Japan since 2022-06 and
                   under FAA Part 89) + ADS-B In receiver (FlyCart 30 has one
                   [1]; Zipline uses acoustic detect-and-avoid instead [2]).
parachute          ballistic parachute (orange canister on top), as on the
                   Matternet M2, tested to ASTM F3322 [3] and the FlyCart 30 [1].
=================  ==========================================================

Cameras (dashboard "Open cameras"): forward FPV gimbal camera (the black ball
on the nose of the model), downward delivery camera (watches the winch drop;
Wing's patents use a downward camera to confirm the package [4]), and the
front and rear obstacle stereo pairs (left eye shown) [1].

Sources:
  [1] DJI FlyCart 30 specs / user manual: https://www.dji.com/flycart-30 ,
      https://www.bhphotovideo.com/lit_files/1134768.pdf
  [2] Zipline acoustic detect-and-avoid, P2 tether droid:
      https://dronelife.com/2022/06/08/zipline-reveals-acoustic-based-detect-and-avoid-technology/ ,
      https://aopa.org/news-and-media/all-news/2023/april/04/autonomy-at-your-doorstep
  [3] Matternet M2 parachute (Fruity Chutes, ASTM F3322-18):
      https://dronedj.com/2022/11/11/fruity-drone-parachute-recovery-system/ ,
      https://www.post.ch/companycars/-/media/post/ueber-uns/medienmitteilungen/2017/drohnen/spezifikationen-matternet-m2.pdf
  [4] Wing patents: downward camera, tether tension, winch sensing:
      https://patents.google.com/patent/US10647427 ,
      https://patents.google.com/patent/US10689113
  [5] Amazon MK30 detect-and-avoid:
      https://www.aboutamazon.com/news/transportation/amazon-prime-air-drone-delivery-mk30-photos

配送ドローンのセンサー値をシミュレーションの状態から計算する（carla 非依存）。
位置・方位・飛行フェーズ・電池残量・風と制御側の風推定・荷物・ポッドの位置から、
実機のセンサー（GNSS/RTK・磁気・IMU・気圧・下向き LiDAR・オプティカルフロー・
障害物ステレオ・風推定・電池 BMS・モーター/ESC・ウインチ/ロードセル・データ
リンク・リモート ID/ADS-B・パラシュート）が出す値・単位・ノイズ・健全性を作る。
"""

import math
import random

G = 9.81
RHO = 1.225                     # air density [kg/m^3]
MASS = 3.2                      # empty drone with battery [kg] / 機体質量
PROP_R = 0.15                   # rotor radius [m] (0.17 x 0.88)
DISC_A = 4 * math.pi * PROP_R ** 2
FIG_MERIT = 0.65                # rotor figure of merit x motor/ESC efficiency
AVIONICS_W = 25.0               # flight controller, radios, cameras [W]
CDA = 0.06                      # drag area [m^2]
K_THRUST = 2.33e-7              # N per rpm^2 (hover ~5800 rpm) / 推力係数
RPM_MAX = 9500.0
CELLS, PACK_AH, PACK_R = 6, 10.0, 0.045     # 6S Li-ion, 10 Ah, pack resistance [ohm]
BATT_C, BATT_TAU = 900.0, 600.0              # heat capacity [J/K], cooling time [s]
AMBIENT_C = 20.0
RANGE_MAX = 40.0                # downward lidar [m]
FLOW_MAX = 30.0                 # optical flow works below this [m]
OBST_MAX = 30.0                 # stereo depth range [m]
LINK_F_MHZ, LINK_TX_DBM, LINK_GAIN = 5800.0, 20.0, 6.0
NOISE_DBM = -95.0
CHUTE_MIN_AGL = 12.0            # below this a parachute can't open in time [m]
PARCEL_KG = (0.6, 1.9)          # LastMile parcel M (the drone's size) / 荷物 M の重さ
# Li-ion cell open-circuit voltage by state of charge / 充電率ごとのセル開放電圧
OCV = ((0, 3.0), (5, 3.3), (10, 3.45), (20, 3.55), (40, 3.66), (60, 3.8),
       (80, 3.96), (100, 4.18))

IN_POD = ("docked", "reloading")
AT_STATION = ("charging", "standby")
FLYING = ("delivering", "to_station", "to_van")
DOWN = ("falling", "crashed")

# Cameras on the drone, in the drone's frame (CARLA axes: x nose, y right, z up,
# metres from the bottom of the skids), as placed on the model (make_drone_svg.py).
# 機体のカメラ（機体座標、スキッド下端原点）
CAMERAS = {
    "fpv": {"x": 0.215, "y": 0.0, "z": 0.189, "pitch": -12.0, "yaw": 0.0, "fov": 90.0,
            "w": 640, "h": 360},
    "down": {"x": 0.128, "y": 0.042, "z": 0.172, "pitch": -90.0, "yaw": 0.0, "fov": 100.0,
             "w": 480, "h": 360},
    "front": {"x": 0.19, "y": -0.032, "z": 0.282, "pitch": -5.0, "yaw": 0.0, "fov": 110.0,
              "w": 480, "h": 270},
    "rear": {"x": -0.19, "y": 0.032, "z": 0.282, "pitch": -5.0, "yaw": 180.0, "fov": 110.0,
             "w": 480, "h": 270},
}
CAMERA_ORDER = ("fpv", "down", "front", "rear")
CAMERA_FPS = 10.0

OK, WARN, FAIL, OFF = "ok", "warn", "fail", "off"
_RANK = {OFF: 0, OK: 1, WARN: 2, FAIL: 3}


def worst(*flags):
    return max(flags, key=lambda f: _RANK.get(f, 0)) if flags else OK


def clamp(v, a, b):
    return a if v < a else b if v > b else v


def interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


def parcel_kg(order_id):
    """A stable parcel weight for an order (the sim has none). / 注文ごとの荷物重量。"""
    if order_id is None:
        return 0.0
    u = ((int(order_id) * 2654435761) % 1000) / 1000.0
    return round(PARCEL_KG[0] + u * (PARCEL_KG[1] - PARCEL_KG[0]), 2)


def pressure_hpa(alt_m):
    """ISA pressure at ``alt_m`` above sea level. / 標準大気の気圧。"""
    return 1013.25 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588


class GeoRef(object):
    """World metres -> latitude / longitude, as CARLA's own geolocation does.

    Built from three samples of ``Map.transform_to_geolocation`` (origin,
    +1000 m x, +1000 m y); over a city the mapping is affine to millimetres.
    Without a map geo reference (all zero) it uses ``fallback`` (Oita).
    3 点のジオロケーションからアフィン変換を作る（地理参照が無ければ大分）。
    """

    FALLBACK = (33.2384, 131.6125, 0.0)

    def __init__(self, origin=None, at_x=None, at_y=None, d=1000.0):
        if not origin or (abs(origin[0]) < 1e-9 and abs(origin[1]) < 1e-9):
            lat0, lon0, alt0 = self.FALLBACK
            k_lat = 1.0 / 111320.0
            k_lon = 1.0 / (111320.0 * math.cos(math.radians(lat0)))
            # CARLA's convention: +x east, +y south / CARLA 既定：+x 東・+y 南
            self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
            self.m = (0.0, -k_lat, k_lon, 0.0)
            self.fallback = True
        else:
            self.lat0, self.lon0, self.alt0 = origin
            self.m = ((at_x[0] - origin[0]) / d, (at_y[0] - origin[0]) / d,
                      (at_x[1] - origin[1]) / d, (at_y[1] - origin[1]) / d)
            self.fallback = False
        lat_x, lat_y, lon_x, lon_y = self.m
        kn = 111320.0
        ke = 111320.0 * math.cos(math.radians(self.lat0))
        # metres north / east per world metre x and y / 世界座標 1 m あたりの北・東 [m]
        self._ne = (lat_x * kn, lat_y * kn, lon_x * ke, lon_y * ke)

    def latlon(self, x, y):
        lat_x, lat_y, lon_x, lon_y = self.m
        return (self.lat0 + lat_x * x + lat_y * y, self.lon0 + lon_x * x + lon_y * y)

    def heading(self, yaw_deg):
        """Compass heading (0 = north, clockwise) of a CARLA yaw. / 方位角。"""
        cx, cy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        nx, ny, ex, ey = self._ne
        return math.degrees(math.atan2(ex * cx + ey * cy, nx * cx + ny * cy)) % 360.0

    def to_dict(self):
        return {"lat0": self.lat0, "lon0": self.lon0, "fallback": self.fallback}


def link_rssi(dist_m):
    """5.8 GHz free-space link budget [dBm]. / 5.8 GHz の自由空間リンク。"""
    d = max(dist_m, 1.0)
    fspl = 20.0 * math.log10(d) + 20.0 * math.log10(LINK_F_MHZ) - 27.55
    return LINK_TX_DBM + LINK_GAIN - fspl


# ---- damage model / 損傷モデル -------------------------------------------------
# Which parts of the airframe a failure breaks, for the Drones page's 3D model
# (part ids = the page's model: rotor1..4, arm1..4, motor1..4, esc1..4, shell,
# battery, winch, hook, gear, gnss, cam_fpv, cam_front, cam_rear, link,
# parachute). Deterministic: the same crash always breaks the same parts.
# Motor numbers: PX4 quad-X, (front +, right +) corner of each.
# 故障で壊れる部品（ページの 3D モデルの部品 ID）。同じ墜落は常に同じ結果。
BROKEN, DAMAGED = "broken", "damaged"
CORNER = {1: (1, 1), 2: (-1, -1), 3: (1, -1), 4: (-1, 1)}
CORNER_NAME = {1: "front_right", 2: "rear_left", 3: "front_left", 4: "rear_right"}
GROUND_ROTORS_H = 2.0       # a fall from higher breaks the rotors it lands on [m]
GROUND_GEAR_H = 5.0         # ... and the skids / スキッド
GROUND_SHELL_H = 8.0        # ... and cracks the shell / 外殻のひび


def corner_of(fwd, right):
    """Motor number of the (front, right) corner a direction points at."""
    s = (1 if fwd >= 0 else -1, 1 if right >= 0 else -1)
    return next(k for k, v in CORNER.items() if v == s)


def damage_model(uid, status, fail, parcel_lost=False, impact=None, fall_h=0.0,
                 dead_motor=None, crashes=0, chute=None):
    """Broken / damaged parts of one drone, or None when it is whole.

    ``impact``: the drone-frame direction (forward, right) it was moving when
    it hit something (None: unknown, a fixed corner per drone and crash is
    used). ``fall_h``: how far it fell to the ground (m, 0 while still in the
    air). ``chute``: the parachute state ("deployed" softens the landing).
    衝突方向・落下高さ・故障理由から壊れた部品を返す（無傷なら None）。
    """
    fail = fail or ""
    down = status in DOWN
    parts = {}

    def hit(part, level, why):
        cur = parts.get(part)
        if cur is None or (cur["level"] == DAMAGED and level == BROKEN):
            parts[part] = {"level": level, "why": why}

    cause, side = None, None
    if down and fail.startswith("collision"):
        cause = "collision"
        if impact is not None and math.hypot(*impact) > 0.3:
            fwd, right = impact
        else:   # unknown direction: a fixed corner per drone and crash / 方向不明
            k = 1 + (uid * 7 + crashes * 3) % 4
            fwd, right = CORNER[k]
        c = corner_of(fwd, right)
        side = CORNER_NAME[c]
        hit("rotor%d" % c, BROKEN, "collision")
        hit("arm%d" % c, BROKEN, "collision")
        hit("motor%d" % c, DAMAGED, "collision")
        # the neighbour on the face that took the blow / 衝突面のもう一方
        if abs(fwd) >= abs(right):
            n = corner_of(fwd, -right)
            hit("cam_fpv" if fwd > 0 else "cam_rear", DAMAGED, "collision")
            if fwd > 0:
                hit("cam_front", DAMAGED, "collision")
        else:
            n = corner_of(-fwd, right)
            hit("link", DAMAGED, "collision")
        hit("rotor%d" % n, DAMAGED, "collision")
        hit("shell", DAMAGED, "collision")
    elif down and fail == "motor_failure":
        cause = "motor_failure"
        m = (dead_motor if dead_motor is not None else uid % 4) + 1
        hit("motor%d" % m, BROKEN, "motor_failure")
        hit("esc%d" % m, BROKEN, "motor_failure")
        hit("rotor%d" % m, DAMAGED, "motor_failure")
    elif down and fail == "battery_depleted":
        cause = "battery_depleted"
        hit("battery", BROKEN, "battery_depleted")
    elif down and fail == "blown_off_course":
        cause = "wind"
    elif down:
        cause = fail or "unknown"
    if status == "crashed" and fall_h > GROUND_ROTORS_H:
        # backends.py lays a crashed drone on its right side (roll 70): the
        # right-hand rotors take the landing / 右側を下に倒れる：右のローターが接地
        soft = chute == "deployed"
        for m in (1, 4):
            hit("rotor%d" % m, DAMAGED if soft else BROKEN, "ground")
        if not soft:
            hit("arm4", DAMAGED, "ground")
        if fall_h > GROUND_GEAR_H:
            hit("gear", DAMAGED if soft else BROKEN, "ground")
        if fall_h > GROUND_SHELL_H and not soft:
            hit("shell", BROKEN, "ground")
            hit("gnss", DAMAGED, "ground")
    if parcel_lost:
        cause = cause or "parcel_lost"
        hit("hook", BROKEN, "tether_snapped")
        hit("winch", DAMAGED, "tether_snapped")
    if not parts and cause is None:
        return None
    return {"parts": parts, "cause": cause, "impact": side,
            "fall_h": round(fall_h, 1), "broken": sorted(k for k, v in parts.items()
                                                          if v["level"] == BROKEN)}


def rotor_power(thrust_n, climb=0.0):
    """Electrical power for ``thrust_n`` (momentum theory) + climb + avionics [W]."""
    induced = thrust_n ** 1.5 / math.sqrt(2.0 * RHO * DISC_A)
    return induced / FIG_MERIT + max(climb, 0.0) * thrust_n / 0.8 + AVIONICS_W


class SensorSuite(object):
    """One drone's sensors. ``update(t, s)`` -> the "sensors" dict of the live state.

    ``s`` (a dict) is what the sim knows about the drone this publish:
      status, phase, pos (x, y, z), yaw, battery (%), drain (%/s now),
      reserve (%), loaded, order, wind (wx, wy), w_est (or None), drift (m),
      saturated, max_wind, ground_z (or None), obstacles {"front", "rear"}
      (m or None), home (x, y, z) or None, fail (reason or None),
      parcel_lost, drop (0..1 winch lowering or None), drop_h (m above
      the drop point), parcel_wind (m/s), rain (0..100), detail (what it
      hit, or ""), crashes (count). The result also carries "damage"
      (damage_model, None while the drone is whole).
    1 機分のセンサー。``s`` はシミュレーションが知っている状態。
    """

    def __init__(self, uid, geo, seed=None):
        self.id = uid
        self.geo = geo
        self.rng = random.Random(uid * 7919 if seed is None else seed)
        self.t = None
        self.pos = None
        self.vel = (0.0, 0.0, 0.0)
        self.acc = (0.0, 0.0, 0.0)
        self.yaw = None
        self.yaw_rate = 0.0
        self.tilt = (0.0, 0.0)              # roll, pitch [deg]
        self.batt_c = AMBIENT_C + 4.0
        self.motor_c = [AMBIENT_C] * 4
        self.tether = 0.0
        self.chute = "disarmed"
        self.agl = None
        self.dead_motor = uid % 4           # the one a motor failure takes / 故障するモーター
        self.baro_bias = self.rng.uniform(-0.6, 0.6)
        self.last = None
        # damage memory: last airborne drone-frame velocity, and the crash it
        # had (impact direction, height the fall started at) / 損傷の記憶
        self.air_v = None
        self.crash = None

    # -- helpers
    def _n(self, sigma):
        return self.rng.gauss(0.0, sigma)

    def _kinematics(self, t, pos, yaw):
        """Velocity / acceleration from successive publishes (low-pass)."""
        if self.t is not None and t == self.t:
            return 0.0                      # same instant again: keep the estimate / 同時刻は維持
        if self.t is None or t < self.t or self.pos is None:
            self.t, self.pos, self.yaw = t, pos, yaw
            self.vel, self.acc, self.yaw_rate = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
            return 0.0
        dt = t - self.t
        v = tuple((pos[i] - self.pos[i]) / dt for i in range(3))
        if math.dist(pos, self.pos) > 60.0 * dt + 5.0:     # a teleport (re-docked) / 瞬間移動
            v = (0.0, 0.0, 0.0)
        k = clamp(dt / 0.6, 0.0, 1.0)
        a = tuple((v[i] - self.vel[i]) / dt for i in range(3))
        self.acc = tuple(self.acc[i] + (clamp(a[i], -15.0, 15.0) - self.acc[i]) * k for i in range(3))
        self.vel = v
        dy = ((yaw - self.yaw + 180.0) % 360.0) - 180.0 if self.yaw is not None else 0.0
        self.yaw_rate = self.yaw_rate + (dy / dt - self.yaw_rate) * k
        self.t, self.pos, self.yaw = t, pos, yaw
        return dt

    # -- the suite
    def update(self, t, s):
        st = s.get("status", "")
        pos = tuple(s.get("pos", (0.0, 0.0, 0.0)))
        yaw = float(s.get("yaw", 0.0))
        dt = self._kinematics(t, pos, yaw)
        in_pod, station = st in IN_POD, st in AT_STATION
        flying, falling, crashed = st in FLYING, st == "falling", st == "crashed"
        parked = in_pod or station
        fail = s.get("fail") or ""
        vx, vy, vz = self.vel
        ax, ay, az = self.acc
        cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        v_fwd, v_right = vx * cy + vy * sy, -vx * sy + vy * cy
        wind = tuple(s.get("wind", (0.0, 0.0)))
        wspeed = math.hypot(*wind)
        payload = parcel_kg(s.get("order")) if s.get("loaded") else 0.0
        mass = MASS + payload

        # height above ground from the downward ray / 下向きレイから対地高度
        gz = s.get("ground_z")
        agl = None if gz is None else max(pos[2] - gz, 0.0)
        if parked:
            agl = 0.0
        self.agl = agl

        # forces: gravity + acceleration + drag of the airspeed / 必要推力
        air = (vx - wind[0], vy - wind[1])
        airspeed = math.hypot(*air)
        drag = 0.5 * RHO * CDA * airspeed
        fx = mass * ax + drag * air[0]
        fy = mass * ay + drag * air[1]
        fz = mass * (az + G)
        if not flying:
            fx = fy = fz = 0.0
        thrust = math.sqrt(fx * fx + fy * fy + fz * fz)
        f_fwd, f_right = fx * cy + fy * sy, -fx * sy + fy * cy
        if flying and fz > 1e-3:
            pitch = -math.degrees(math.atan2(f_fwd, fz))       # nose down = negative
            roll = math.degrees(math.atan2(f_right, fz))
        elif crashed:
            pitch, roll = 12.0, 70.0                            # how backends.py lays it / 墜落姿勢
        elif falling:
            pitch, roll = 12.0, 25.0
        else:
            pitch = roll = 0.0
        p_rate = (pitch - self.tilt[1]) / dt if dt else 0.0
        r_rate = (roll - self.tilt[0]) / dt if dt else 0.0
        self.tilt = (roll, pitch)

        out = {}
        # ---- motors + ESC / モーター・ESC
        rpm, cur, temp, thr = [0.0] * 4, [0.0] * 4, [], [0.0] * 4
        motor_fail = fail == "motor_failure" and (falling or crashed)
        if flying:
            per = thrust / 4.0
            # battery on the right flank: right motors work ~2 % harder; pitching
            # shifts thrust fore / aft / 右側の電池で右モーター +2 %、ピッチで前後差
            k_pitch = clamp(-p_rate * 0.004, -0.08, 0.08)
            k_roll = clamp(r_rate * 0.004, -0.08, 0.08)
            # PX4 quad-X: 1 FR, 2 RL, 3 FL, 4 RR / 機体番号
            sides = ((1, 1), (-1, -1), (1, -1), (-1, 1))        # (front +, right +)
            for i, (fwd, right) in enumerate(sides):
                ti = per * (1.0 + 0.02 * right + k_pitch * fwd + k_roll * right + self._n(0.006))
                rpm[i] = math.sqrt(max(ti, 0.0) / K_THRUST)
                thr[i] = clamp(rpm[i] / RPM_MAX, 0.0, 1.0)
                cur[i] = rotor_power(ti * 4.0, vz) / 4.0 / 22.2
        for i in range(4):
            if motor_fail and i == self.dead_motor:
                rpm[i] = thr[i] = cur[i] = 0.0
            heat = 18.0 + 30.0 * thr[i] ** 2 if flying else 0.0
            target = AMBIENT_C + heat
            self.motor_c[i] += (target - self.motor_c[i]) * clamp(dt / 90.0, 0.0, 1.0)
            temp.append(round(self.motor_c[i], 1))
        # ---- damage (only computed while something is wrong) / 損傷
        if flying:
            self.air_v = (v_fwd, v_right)
            self.crash = None
        if falling or crashed:
            if self.crash is None:
                self.crash = {"impact": self.air_v, "z0": pos[2]}
            fall_h = max(self.crash["z0"] - gz, 0.0) if crashed and gz is not None else 0.0
        else:
            self.crash, fall_h = None, 0.0
        damage = None
        if falling or crashed or s.get("parcel_lost"):
            damage = damage_model(self.id, st, fail, s.get("parcel_lost"),
                                  self.crash["impact"] if self.crash else None, fall_h,
                                  self.dead_motor, int(s.get("crashes") or 0), self.chute)
            if damage is not None and s.get("detail") and (falling or crashed):
                damage["detail"] = s["detail"]
        prop_broken = bool(damage) and any(
            p.startswith(("rotor", "arm", "motor", "esc")) and v["level"] == BROKEN
            for p, v in damage["parts"].items())
        m_health = (FAIL if motor_fail or prop_broken else
                    WARN if max(temp) > 85.0 or max(thr) > 0.92
                    else OK if flying else OFF)
        out["motors"] = {"rpm": [round(r) for r in rpm], "current": [round(c, 1) for c in cur],
                         "temp": temp, "throttle": [round(x * 100) for x in thr],
                         "dead": self.dead_motor + 1 if motor_fail else None, "health": m_health}

        # ---- battery (BMS) / 電池
        soc = clamp(float(s.get("battery", 0.0)), 0.0, 100.0)
        ocv = interp(OCV, soc) * CELLS
        if flying:
            power = rotor_power(thrust, vz)
            current = power / max(ocv - 0.5, 10.0)
        elif st == "charging":
            current = -PACK_AH * 1.0                            # 1C fast charge / 急速充電
        elif in_pod and soc < 99.5:
            current = -2.0                                      # trickle in the pod / 低速充電
        else:
            current = 0.4 if not (crashed and fail == "battery_depleted") else 0.0
        volt = ocv - current * PACK_R
        heat = current * current * PACK_R * 6.0
        self.batt_c += (heat / BATT_C - (self.batt_c - AMBIENT_C) / BATT_TAU) * dt
        drain = float(s.get("drain") or 0.0)
        reserve = float(s.get("reserve", 15.0))
        left = (soc - reserve) / drain if flying and drain > 1e-6 else None
        b_health = (FAIL if soc <= 0.5 or fail == "battery_depleted" else
                    WARN if soc < reserve or self.batt_c > 60.0 else OK)
        out["battery"] = {"soc": round(soc, 1), "voltage": round(volt, 2),
                          "cell": round(volt / CELLS, 3), "current": round(current, 1),
                          "power": round(volt * current), "temp": round(self.batt_c, 1),
                          "left_s": None if left is None else round(max(left, 0.0)),
                          "capacity_ah": PACK_AH, "cells": CELLS,
                          "charging": current < 0, "health": b_health}

        # ---- GNSS (RTK) / GNSS
        obst = s.get("obstacles") or {}
        near = min([d for d in (obst.get("front"), obst.get("rear")) if d is not None] or [99.0])
        if in_pod:
            fix, sats, hdop = "none", self.rng.randint(0, 3), 99.9
        elif station:
            fix, sats, hdop = "rtk_fixed", 22 + int(2 * math.sin(t / 40.0 + self.id)), 0.6
        elif crashed or falling:
            fix, sats, hdop = "3d", 9 + self.rng.randint(0, 3), 2.4
        elif (agl is not None and agl < 4.0) or near < 8.0:
            fix, sats, hdop = "rtk_float", 16 + int(3 * math.sin(t / 30.0 + self.id)), 1.3
        else:
            fix, sats, hdop = "rtk_fixed", 21 + int(3 * math.sin(t / 30.0 + self.id)), 0.7
        acc_h = {"rtk_fixed": 0.02, "rtk_float": 0.35, "3d": 1.8, "none": None}[fix]
        lat, lon = self.geo.latlon(pos[0] + (self._n(acc_h) if acc_h else 0.0),
                                   pos[1] + (self._n(acc_h) if acc_h else 0.0))
        alt = self.geo.alt0 + pos[2] + (self._n(acc_h * 1.5) if acc_h else 0.0)
        out["gnss"] = {"lat": round(lat, 7), "lon": round(lon, 7), "alt": round(alt, 2),
                       "fix": fix, "sats": sats, "hdop": round(hdop + abs(self._n(0.05)), 2),
                       "acc_h": acc_h, "speed": round(math.hypot(vx, vy), 2),
                       "health": (OFF if in_pod else FAIL if fix == "none" else
                                  WARN if fix == "3d" else OK)}

        # ---- magnetometer / 磁気
        heading = self.geo.heading(yaw)
        interference = (in_pod or (s.get("home") is not None and flying and
                                   math.dist(pos, s["home"]) < 3.0))
        field = 46.5 + (14.0 if interference else 0.0) + self._n(0.3)
        out["mag"] = {"heading": round((heading + self._n(0.6)) % 360.0, 1),
                      "field": round(field, 1), "interference": interference,
                      "health": OFF if in_pod else WARN if interference or crashed else OK}

        # ---- IMU / IMU
        if falling:
            spec = (self._n(0.3), self._n(0.3), self._n(0.4))          # free fall / 自由落下
        elif flying:
            spec = (-drag * (air[0] * cy + air[1] * sy) / mass + self._n(0.15),
                    -drag * (-air[0] * sy + air[1] * cy) / mass + self._n(0.15),
                    -thrust / mass + self._n(0.2))
        else:
            # at rest: -1 g along the body z, tipped over when crashed / 静止時は -1 g
            r, p = math.radians(roll), math.radians(pitch)
            spec = (G * math.sin(p) + self._n(0.02), -G * math.sin(r) * math.cos(p) + self._n(0.02),
                    -G * math.cos(r) * math.cos(p) + self._n(0.02))
        vibe = (0.25 + 2.2 * (sum(thr) / 4.0) ** 2 + self._n(0.05)) if flying else abs(self._n(0.02))
        if motor_fail:
            vibe += 4.0
        out["imu"] = {"ax": round(spec[0], 2), "ay": round(spec[1], 2), "az": round(spec[2], 2),
                      "gx": round(r_rate + self._n(0.3), 1), "gy": round(p_rate + self._n(0.3), 1),
                      "gz": round(self.yaw_rate + self._n(0.3), 1),
                      "roll": round(roll, 1), "pitch": round(pitch, 1), "vibe": round(max(vibe, 0.0), 2),
                      "health": FAIL if falling else WARN if crashed or vibe > 3.0 else OK}

        # ---- barometer / 気圧
        gust = clamp((wspeed - 8.0) * 0.08, 0.0, 1.0)
        b_alt = alt + self.baro_bias + 0.4 * math.sin(t / 50.0 + self.id) + self._n(0.12 + gust * 0.4)
        out["baro"] = {"alt": round(b_alt, 2), "pressure": round(pressure_hpa(b_alt), 2),
                       "vs": round(vz + self._n(0.1), 2), "temp": round(AMBIENT_C + 12.0 + self._n(0.1), 1),
                       "health": WARN if gust > 0.5 else OK}

        # ---- downward lidar rangefinder / 下向き LiDAR
        valid = agl is not None and 0.2 <= agl <= RANGE_MAX and not in_pod
        out["rangefinder"] = {"agl": round(agl + self._n(0.02), 2) if valid else None,
                              "ground": None if agl is None else round(agl, 1),
                              "max": RANGE_MAX, "in_range": valid,
                              "health": (OFF if in_pod else WARN if crashed else OK)}

        # ---- optical flow / オプティカルフロー
        if flying and agl is not None and agl < FLOW_MAX:
            q = int(clamp(255.0 * (1.0 - agl / FLOW_MAX) - (s.get("rain") or 0.0), 20.0, 255.0))
            flow = {"vx": round(v_fwd + self._n(0.05), 2), "vy": round(v_right + self._n(0.05), 2),
                    "quality": q, "health": OK if q > 60 else WARN}
        else:
            flow = {"vx": None, "vy": None, "quality": 0,
                    "health": WARN if crashed else OFF}       # above its range: idle / 範囲外は停止
        out["flow"] = flow

        # ---- obstacle sensing (stereo pairs + downward lidar) / 障害物検知
        front, rear = obst.get("front"), obst.get("rear")
        down = out["rangefinder"]["agl"]
        dists = [d for d in (front, rear) if d is not None]
        nearest = min(dists) if dists else None
        o_health = (FAIL if crashed and fail.startswith("collision") else
                    OFF if in_pod else WARN if nearest is not None and nearest < 5.0 else OK)
        out["obstacle"] = {"front": None if front is None else round(front, 1),
                           "rear": None if rear is None else round(rear, 1),
                           "down": down, "nearest": None if nearest is None else round(nearest, 1),
                           "max": OBST_MAX, "health": o_health}

        # ---- airspeed / wind estimate / 対気速度・風推定
        w_est = s.get("w_est")
        est = w_est if w_est is not None else wind
        max_wind = float(s.get("max_wind") or 11.0)
        out["air"] = {"airspeed": round(airspeed, 1) if flying else 0.0,
                      "wind": round(wspeed, 1),
                      "wind_dir": round(math.degrees(math.atan2(wind[1], wind[0])) % 360.0),
                      "est": round(math.hypot(*est), 1),
                      "est_dir": round(math.degrees(math.atan2(est[1], est[0])) % 360.0),
                      "drift": round(float(s.get("drift") or 0.0), 2), "limit": max_wind,
                      "saturated": bool(s.get("saturated")),
                      "health": (FAIL if s.get("saturated") else
                                 WARN if wspeed > 0.8 * max_wind else OK)}

        # ---- winch + tether + load cell / ウインチ・ロードセル
        drop = s.get("drop")
        drop_h = float(s.get("drop_h") or 0.0)
        prev = self.tether
        if drop is not None:
            self.tether = clamp(drop, 0.0, 1.0) * max(drop_h - 0.2, 0.0)
            touched = drop >= 0.98
            state = "released" if touched else "lowering"
            load = 0.0 if touched or s.get("parcel_lost") else payload * (G + self._n(0.4))
        else:
            self.tether = 0.0
            state = "stowed" if payload > 0 else "idle"
            load = payload * (G + az) if payload > 0 and flying else payload * G
        speed = (self.tether - prev) / dt if dt else 0.0
        if self.tether == 0.0 and prev > 0.0:
            state, speed = "retracted", 0.0
        out["winch"] = {"state": state, "tether": round(self.tether, 2),
                        "speed": round(clamp(speed, -2.0, 2.0), 2),
                        "load": round(max(load, 0.0), 1), "payload": payload,
                        "max_tether": 20.0,
                        "health": (FAIL if s.get("parcel_lost") else
                                   WARN if drop is not None and wspeed > float(s.get("parcel_wind") or 12.0)
                                   else OK)}

        # ---- datalink / データリンク
        home = s.get("home")
        dist = math.dist(pos, home) if home is not None else None
        if in_pod:
            link = {"mode": "wired", "rssi": None, "snr": None, "latency": 2, "loss": 0.0,
                    "dist": 0.0, "lte": None, "health": OK}
        else:
            rssi = link_rssi(dist if dist is not None else 500.0) + self._n(1.5)
            if crashed:
                rssi -= 18.0                       # antenna on the ground / 地上のアンテナ
            loss = clamp((-85.0 - rssi) * 4.0, 0.0, 100.0)
            lte = -68.0 - 0.08 * (agl or 0.0) + self._n(2.0)
            link = {"mode": "c2" if rssi > NOISE_DBM + 5 else "lte", "rssi": round(rssi, 1),
                    "snr": round(rssi - NOISE_DBM, 1),
                    "latency": round(18.0 + (dist or 0.0) * 0.004 + abs(self._n(3.0)) + loss * 0.5),
                    "loss": round(loss, 1), "dist": None if dist is None else round(dist),
                    "lte": round(lte, 1),
                    "health": FAIL if rssi < NOISE_DBM else WARN if rssi < -82.0 else OK}
        out["link"] = link

        # ---- Remote ID + ADS-B In / リモート ID・ADS-B
        out["remote_id"] = {"broadcast": flying or falling, "serial": "LMP-%04d" % (1000 + self.id),
                            "traffic": 0, "nearest": None,
                            "health": OK if flying or falling else OFF}

        # ---- parachute / パラシュート
        if flying:
            self.chute = "armed" if agl is None or agl >= CHUTE_MIN_AGL else "low"
        elif falling or crashed:
            if self.chute in ("armed", "low"):
                self.chute = "deployed" if self.chute == "armed" else "too_low"
        else:
            self.chute = "disarmed"
        out["parachute"] = {"state": self.chute, "min_agl": CHUTE_MIN_AGL,
                            "health": (FAIL if self.chute == "too_low" else
                                       WARN if self.chute in ("deployed", "low") else
                                       OK if self.chute == "armed" else OFF)}

        # ---- cameras / カメラ
        out["cams"] = {"available": not in_pod and s.get("body", True),
                       "list": list(CAMERA_ORDER), "fps": CAMERA_FPS}
        out["health"] = worst(*[v["health"] for v in out.values()
                                if isinstance(v, dict) and "health" in v])
        if damage is not None and damage["broken"]:
            out["health"] = FAIL
        out["damage"] = damage
        out["t"] = round(t, 1)
        self.last = out
        return out
