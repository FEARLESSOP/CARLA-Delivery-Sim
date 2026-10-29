#!/usr/bin/env python3
"""
carla_ffb.py - physically based force feedback for CARLA (Ubuntu, Logitech G923 / G29)
              CARLA 用の物理ベースのフォースフィードバック (Ubuntu, Logitech G923 / G29)

Runs NEXT TO the driving window (it only reads the world, never ticks it):
運転ウィンドウと並行して動かす (ワールドを読むだけで tick はしない):
  1) python3 run_pipeline.py            (or CarlaUE4.sh + the G923 script)
  2) ../../carla-venv/bin/python carla_ffb.py [--strength 1.0] [--verbose]

What the wheel feels, and where it comes from / ハンドルに伝わる力とその出所:
  - Self-aligning torque of the front tyres (Fiala brush tyre model): the
    front slip angle is computed from CARLA's real velocity / yaw rate and the
    live wheel angle. The wheel loads up with cornering force, then goes
    LIGHT just before the front tyres let go (pneumatic trail collapses).
    When the rear steps out the torque pulls the wheel into the slide, i.e.
    the wheel countersteers by itself like a real car.
    前輪のセルフアライニングトルク (Fiala ブラシモデル)。CARLA の実速度・ヨー
    レートと現在のハンドル角から前輪スリップ角を計算する。横力に応じて重くなり、
    グリップ限界の手前で軽くなる。リアが滑るとハンドルが自然にカウンター方向へ
    引かれる。
  - Weight transfer: heavier under braking, lighter under power; braking
    near the grip limit eats into lateral grip (friction ellipse).
    荷重移動: ブレーキで重く、加速で軽い。限界ブレーキ中は横グリップが減る。
  - Low speed: kingpin centering + tyre scrub friction, so the wheel is
    stiff when parked and frees up as the car rolls.
    低速: キングピンによる復元力とタイヤの据え切り摩擦 (停車中は重い)。
  - Kerbs: each front wheel's position is checked against the OpenDRIVE lane
    type under it. Hitting / leaving a sidewalk, border or median yanks the
    wheel toward that side, followed by a short suspension ring.
    縁石: 各前輪の下のレーン種別を OpenDRIVE で判定し、歩道・縁石・中央分離帯へ
    乗り上げる / 降りる瞬間にその側へハンドルを取られる。
  - Road surface: bumps from the chassis' vertical acceleration, plus
    texture that depends on the surface (asphalt / shoulder / sidewalk /
    off-road) and tyre-scrub buzz when the fronts are sliding.
    路面: 車体の上下加速度からの突き上げ、路面種別ごとの振動、前輪が滑っている
    時のスクラブ振動。
  - Collisions: jolt scaled by the collision impulse.
    衝突: 衝撃量に比例した衝撃。
  - Steering lock: a soft end stop where CARLA's steering saturates.
    ステアリングロック: CARLA の舵角が飽和する位置にエンドストップ。

Every tuning value below is live-editable from the dashboard's Force Feedback
tab: they live in ffb_tuning.json (see ffb_settings.py), which this script
re-reads twice a second. The default preset feels like Forza Horizon: a
speed-dependent centre spring, a centre damper so the wheel settles instead
of overshooting, a smooth minimum force and a torque slew limit.
以下の調整値はすべてダッシュボードの Force Feedback タブから変更できる
(ffb_tuning.json、毎秒 2 回読み直す)。既定値は Forza Horizon 風。

Damping and friction run as hardware effects inside the wheel driver
(~1 kHz, from the wheel's own encoder) instead of a noisy Python derivative.
With new-lg4ff, their strength is also scaled by
/sys/class/hidraw/<dev>/device/damper_level (default 30 %).
ダンパーと摩擦はホイールドライバ内のハードウェア効果 (~1 kHz) で処理する。
new-lg4ff ではその強さが sysfs の damper_level (既定 30 %) でも倍率がかかる。

Requires: pip install evdev   (and CARLA PythonAPI matching your server version)
必要: pip install evdev (サーバと同じバージョンの CARLA PythonAPI)
Permissions: sudo usermod -aG input $USER   (then log out / in)
権限: sudo usermod -aG input $USER (その後ログインし直す)
"""
import argparse
import configparser
import math
import os
import random
import subprocess
import sys
import glob
import threading
import time

# Add CARLA egg to python path
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from evdev import InputDevice, ecodes, ff, list_devices

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)
import ffb_settings
import vehicle_tuning

# ---------------- tuning / 調整 ----------------
# The live values are in T (from ffb_tuning.json, defaults in ffb_settings.py);
# the constants below are only the fixed ones and older references.
# 実際に使う値は T (ffb_tuning.json)。以下の定数は固定値と旧値の参考。
HZ = 250                 # FFB loop rate / FFB 更新周期 [Hz]
STRENGTH = 1.0           # master gain (--strength) / 全体の強さ
INVERT = True            # flip if the wheel pushes the wrong way / 向きが逆なら反転

# Tyre + steering geometry (front axle) / タイヤとステアリング形状 (前軸)
MU = 1.0                 # nominal tyre-road friction / 路面摩擦係数
SLIP_PEAK_DEG = 11.0     # front slip angle where the tyre is fully sliding (grip peaks ~0.6x this) / 完全に滑るスリップ角
CONTACT_HALF_LEN = 0.10  # contact patch half length [m] -> pneumatic trail ~ 1/3 of it / 接地長の半分
CASTER_TRAIL = 0.010     # mechanical trail [m]; lower = lighter at the limit / キャスタートレール
CG_HEIGHT = 0.55         # centre of gravity height [m] for weight transfer / 重心高
TAU_FULL_NM = 260.0      # kingpin torque that maps to 100 % motor torque / 100% 出力に相当するキングピントルク
KPI_NM = 70.0            # kingpin-inclination centering at full lock [Nm] / キングピン傾角による復元トルク
SLIP_BLEND_KMH = (2.0, 9.0)  # fade the slip model in over this speed range / スリップモデルを有効にする速度域

# Hardware effects (0..1) / ハードウェア効果
FRICTION_PARK = 0.55     # tyre scrub when stopped / 停車時の据え切り摩擦
FRICTION_PARK_KMH = 6.0  # ...decays with this speed constant / この速度で減衰
FRICTION_COLUMN = 0.05   # steering column friction, always on / コラム摩擦 (常時)
DAMPER_BASE = 0.12       # steering damping / ダンピング
DAMPER_PER_KMH = 0.0020  # extra damping per km/h (tyre relaxation) / 速度比例のダンピング
DAMPER_MAX = 0.45

# Road feel / 路面の感触
KERB_JOLT = 0.45         # kerb strike strength / 縁石の衝撃
BUMP_THRESH = 3.0        # vertical accel jump [m/s^2] that counts as a bump / 突き上げ判定
BUMP_GAIN = 0.025        # per m/s^2 above threshold / 閾値超過 1 m/s^2 あたり
TEXTURE = {              # surface -> vibration amplitude at 60 km/h / 60 km/h 時の路面振動
    "road": 0.012, "shoulder": 0.035, "raised": 0.05, "off": 0.11,
}
SCRUB_BUZZ = 0.07        # vibration while the fronts are sliding / 前輪スライド時の振動
CRASH_DIV = 4000.0       # impulse -> strength divisor (lower = stronger) / 衝撃量の割り数
CRASH_GAIN = 0.9

# Output stage / 出力段
MIN_FORCE = 0.04         # covers the motor's dead band near zero / 0 付近の不感帯補正
SOFT_KNEE = 0.80         # compress torque above this instead of clipping / これ以上は圧縮
END_STOP_GAIN = 25.0     # end-stop spring past CARLA's steering lock / ロック以降のバネ
STALE_SEC = 0.5          # world stopped ticking -> fade dynamic forces / ワールド停止時に減衰

MAKE_CAR_HARDER = True   # lower tire grip so the car really is harder to control
TIRE_FRICTION = 2.6      # default is ~3.5
# -----------------------------------------------

T = ffb_settings.load()      # live tuning, updated in place / 現在の調整値


class TuningWatcher:
    """Re-read ffb_tuning.json when it changes. 変更時に再読込。"""

    def __init__(self):
        self.mtime = self._mtime()

    @staticmethod
    def _mtime():
        try:
            return os.path.getmtime(ffb_settings.TUNING_PATH)
        except OSError:
            return None

    def poll(self):
        m = self._mtime()
        if m == self.mtime:
            return False
        self.mtime = m
        T.update(ffb_settings.load())
        print("[ffb] tuning reloaded / 調整値を再読込")
        return True


RAISED = {carla.LaneType.Sidewalk, carla.LaneType.Border, carla.LaneType.Median}
SHOULDER = {carla.LaneType.Shoulder, carla.LaneType.Restricted}


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def smoothstep(e0, e1, x):
    t = clamp((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


# ---------------------------------------------------------------- wheel ----

def find_wheel():
    for path in list_devices():
        d = InputDevice(path)
        name = d.name.lower()
        if any(k in name for k in ("g923", "g29", "driving force", "logitech")):
            if ecodes.EV_FF in d.capabilities():
                print(f"[ffb] using {d.name} at {path}")
                return d
    raise SystemExit("[ffb] No Logitech wheel with force feedback found. "
                     "Check `fftest` and that you are in the 'input' group.\n"
                     "[ffb] FFB 対応の Logitech ホイールが見つかりません。"
                     "`fftest` と 'input' グループを確認してください。")


def _condition_effect(kind):
    return ff.Effect(kind, -1, 0, ff.Trigger(0, 0), ff.Replay(0, 0),
                     ff.EffectType(ff_condition_effect=(ff.Condition * 2)()))


class Wheel:
    def __init__(self, dev):
        self.dev = dev
        dev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)    # we do our own centering
        dev.write(ecodes.EV_FF, ecodes.FF_GAIN, 0xFFFF)
        self.eff = ff.Effect(
            ecodes.FF_CONSTANT, -1, 0x4000,
            ff.Trigger(0, 0), ff.Replay(0, 0),
            ff.EffectType(ff_constant_effect=ff.Constant(0, ff.Envelope(0, 0, 0, 0))),
        )
        self.eff.id = dev.upload_effect(self.eff)
        dev.write(ecodes.EV_FF, self.eff.id, 1)
        ai = dev.absinfo(ecodes.ABS_X)
        self.xmin, self.xmax = ai.min, ai.max

        # Damper / friction run in the driver, fed by the wheel's encoder.
        # ダンパーと摩擦はドライバ側で計算 (ホイールのエンコーダを使用)。
        supported = set(dev.capabilities().get(ecodes.EV_FF, []))
        self.cond = {}
        for kind in (ecodes.FF_DAMPER, ecodes.FF_FRICTION):
            if kind not in supported:
                print(f"[ffb] driver lacks {ecodes.FF[kind]}; that effect is skipped "
                      f"/ ドライバが {ecodes.FF[kind]} 非対応のため省略")
                continue
            e = _condition_effect(kind)
            e.id = dev.upload_effect(e)
            dev.write(ecodes.EV_FF, e.id, 1)
            self.cond[kind] = [e, -1.0]

    def angle(self):
        """Steering position -1 (left) .. +1 (right)."""
        v = self.dev.absinfo(ecodes.ABS_X).value
        return (v - self.xmin) / (self.xmax - self.xmin) * 2.0 - 1.0

    def force(self, level):
        """Constant force -1..1, positive pushes right (re-upload with same id = update)."""
        level = max(-1.0, min(1.0, -level if T["invert"] else level))
        self.eff.u.ff_constant_effect.level = int(level * 32767)
        self.dev.upload_effect(self.eff)

    def condition(self, kind, coeff):
        """Set a hardware damper / friction coefficient 0..1 (only re-sent on change)."""
        slot = self.cond.get(kind)
        if slot is None or abs(slot[1] - coeff) < 0.01:
            return
        e, slot[1] = slot[0], coeff
        c = int(clamp(coeff, 0.0, 1.0) * 32767)
        for axis in e.u.ff_condition_effect:
            axis.right_saturation = axis.left_saturation = 0xFFFF
            axis.right_coeff = axis.left_coeff = c
            axis.deadband = 0
            axis.center = 0
        self.dev.upload_effect(e)

    def close(self):
        try:
            self.force(0)
            self.dev.erase_effect(self.eff.id)
            for e, _ in self.cond.values():
                self.dev.erase_effect(e.id)
            self.dev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0x6000)
        except OSError:
            pass


class SteeringMap:
    """Wheel position -> CARLA steer, mirroring the G923 script's
    steer = 0.5 * clamp(gain * tan(curve * x)) with wheel_config.ini values.
    ハンドル位置 -> CARLA の steer。G923 スクリプトと同じ式・設定値を使う。"""

    def __init__(self):
        p = configparser.ConfigParser()
        p.read(os.path.join(BASE_DIR, "wheel_config.ini"))

        def f(key, default):
            try:
                return p.getfloat("Sensitivity", key)
            except (configparser.Error, ValueError):
                return default

        self.gain = f("steer_gain", 1.83)
        self.curve = f("steer_curve", 0.5)
        self.clamp = f("steer_clamp", 1.0)
        # Wheel position where CARLA's steering saturates, if inside the range.
        # CARLA の舵角が飽和するハンドル位置 (可動域内にある場合)。
        if self.gain * math.tan(self.curve) > self.clamp:
            self.x_lock = math.atan(self.clamp / self.gain) / self.curve
        else:
            self.x_lock = None

    def steer(self, x):
        s = self.gain * math.tan(self.curve * clamp(x, -1.0, 1.0))
        return 0.5 * clamp(s, -self.clamp, self.clamp)


# ------------------------------------------------------------ car state ----

class Car:
    """Everything that comes from the simulator, refreshed once per world tick
    in a background thread. The FFB loop only reads `self.state`.
    シミュレータ由来の情報。tick ごとにバックグラウンドで更新し、FFB ループは
    `self.state` を読むだけ。"""

    def __init__(self, world, hero, steering):
        self.world, self.hero, self.steering = world, hero, steering
        self.map = world.get_map()
        self.events = []                    # (kind, amplitude, direction)
        self.lock = threading.Lock()
        self.alive = True
        self.state = None
        self._snap = None
        self._tick = threading.Event()
        self._surface = [None, None]
        self._az = None
        self._ratio = 1.0
        self._geometry()

        bp = world.get_blueprint_library()
        self.col = world.spawn_actor(bp.find("sensor.other.collision"),
                                     carla.Transform(), attach_to=hero)
        self.col.listen(self._on_collision)
        self._cb = world.on_tick(self._on_tick)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_tire_friction(self):
        """Apply T["tire_friction"] to the hero's tyres, together with the
        dashboard's vehicle tuning (brakes, power, top speed): one physics
        write holds both, so neither undoes the other. Keeps the car's speed.
        タイヤグリップを車両チューニングと一緒に適用 (互いに上書きしない)。"""
        pc = self.hero.get_physics_control()            # blueprint values / 既定値
        if MAKE_CAR_HARDER:
            vehicle_tuning.apply(None, self.hero,
                                 vehicle_tuning.scales(vehicle_tuning.load(), True),
                                 T["tire_friction"])
        self.tire_friction = T["tire_friction"]
        return pc

    def _geometry(self):
        pc = self.set_tire_friction()
        self.mass = pc.mass if pc.mass > 100 else 1500.0
        self.max_steer = math.radians(max(w.max_steer_angle for w in pc.wheels[:2]) or 70.0)

        # Wheel positions come back in world cm; turn them into body-frame m.
        # 車輪位置はワールド座標 [cm] なので車体座標 [m] に変換する。
        tf = self.hero.get_transform()
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()
        local = []
        for w in pc.wheels[:4]:
            d = w.position / 100.0 - tf.location
            local.append((d.x * fwd.x + d.y * fwd.y + d.z * fwd.z,
                          d.x * right.x + d.y * right.y + d.z * right.z))
        x_front = (local[0][0] + local[1][0]) / 2
        x_rear = (local[2][0] + local[3][0]) / 2
        com = pc.center_of_mass.x
        self.a, self.b = x_front - com, com - x_rear
        if not (1.5 < x_front - x_rear < 5.0 and self.a > 0.3 and self.b > 0.3):
            self.a, self.b = 1.3, 1.5               # sane fallback / 既定値
            local[0], local[1] = (1.3, -0.8), (1.3, 0.8)
        self.front_wheels = [(x, y) for x, y in local[:2]]
        print(f"[ffb] {self.hero.type_id}: mass {self.mass:.0f} kg, "
              f"CG->front {self.a:.2f} m, CG->rear {self.b:.2f} m, "
              f"max steer {math.degrees(self.max_steer):.0f} deg")

    def _on_tick(self, snap):
        self._snap = snap
        self._tick.set()

    def _on_collision(self, ev):
        i = ev.normal_impulse
        mag = math.sqrt(i.x ** 2 + i.y ** 2 + i.z ** 2)
        tf = self.hero.get_transform()
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()
        side = i.x * right.x + i.y * right.y
        front = i.x * fwd.x + i.y * fwd.y
        amp = T["crash_gain"] * min(1.0, mag / T["crash_div"])
        # A side hit shoves the wheels away from the impact; a head-on hit
        # wrenches them unpredictably. 側面衝突は衝撃と反対側へ、正面は不定方向。
        direction = math.copysign(1.0, side) if abs(side) > abs(front) else random.choice((-1, 1))
        self._push(("crash", amp, direction))
        print(f"[ffb] CRASH impulse={mag:.0f} strength={amp:.2f}")

    def _push(self, ev):
        with self.lock:
            self.events.append(ev)

    def take_events(self):
        with self.lock:
            ev, self.events = self.events, []
        return ev

    def _surface_at(self, loc):
        wp = self.map.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
        if wp is None:
            return "off"
        if wp.lane_type in RAISED:
            return "raised"
        if wp.lane_type in SHOULDER:
            return "shoulder"
        return "road"

    def _run(self):
        while self.alive:
            if not self._tick.wait(1.0):
                continue
            self._tick.clear()
            snap = self._snap
            a = snap.find(self.hero.id) if snap is not None else None
            if a is None:
                self.alive = False          # hero destroyed / hero が消えた
                break
            try:
                self._update(snap, a)
            except RuntimeError as e:       # transient RPC hiccup / 一時的な RPC エラー
                print(f"[ffb] state update failed: {e}")

    def _update(self, snap, a):
        tf = a.get_transform()
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()
        v, acc, w = a.get_velocity(), a.get_acceleration(), a.get_angular_velocity()
        vx = v.x * fwd.x + v.y * fwd.y + v.z * fwd.z
        vy = v.x * right.x + v.y * right.y + v.z * right.z
        ax = acc.x * fwd.x + acc.y * fwd.y + acc.z * fwd.z
        yaw_rate = math.radians(w.z)        # CARLA: deg/s, + = turning right
        kmh = 3.6 * math.hypot(vx, vy)

        # How far CARLA actually turns the front wheels for a given steer
        # command (its speed-sensitive steering curve shrinks it at speed).
        # 実際の前輪舵角と指令値の比 (速度感応ステアリングで高速時に小さくなる)。
        ctrl = self.hero.get_control()
        if abs(ctrl.steer) > 0.05:
            meas = (abs(self.hero.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)) +
                    abs(self.hero.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel))) / 2
            ratio = math.radians(meas) / (abs(ctrl.steer) * self.max_steer)
            if 0.1 < ratio < 1.5:
                self._ratio += 0.3 * (ratio - self._ratio)

        # Kerb strikes: lane type under each front tyre. 各前輪の下のレーン種別。
        loc = tf.location
        for i, (lx, ly) in enumerate(self.front_wheels):
            p = carla.Location(loc.x + fwd.x * lx + right.x * ly,
                               loc.y + fwd.y * lx + right.y * ly, loc.z)
            s = self._surface_at(p)
            prev, self._surface[i] = self._surface[i], s
            if prev is not None and s != prev and "raised" in (s, prev) and kmh > 1.0:
                amp = T["kerb_jolt"] * clamp(kmh / 30.0, 0.3, 1.0)
                self._push(("kerb", amp, 1.0 if ly > 0 else -1.0))

        # Bumps: jumps in vertical acceleration. 上下加速度の急変 = 突き上げ。
        if self._az is not None:
            jump = abs(acc.z - self._az)
            if jump > T["bump_thresh"] and kmh > 3:
                self._push(("bump", min(0.5, T["bump_gain"] * (jump - T["bump_thresh"])), 0.0))
        self._az = acc.z

        self.state = {
            "t": time.monotonic(), "vx": vx, "vy": vy, "ax": ax, "r": yaw_rate,
            "kmh": kmh, "ratio": self._ratio,
            "surface": max(self._surface, key=list(TEXTURE).index),
        }

    def close(self):
        self.alive = False
        try:
            self.world.remove_on_tick(self._cb)
        except RuntimeError:
            pass
        try:
            self.col.stop()
            self.col.destroy()
        except RuntimeError:
            pass


# ---------------------------------------------------------------- model ----

def front_axle_torque(car, st, delta):
    """Kingpin torque [Nm] of the front axle, + = pushes the wheel right.
    Fiala brush tyre: lateral force saturates at SLIP_PEAK_DEG while the
    aligning moment peaks earlier and falls to zero -> the wheel goes light.
    前軸のキングピントルク [Nm] (+ = 右へ押す)。Fiala モデルでは横力が飽和する前に
    アライニングモーメントがピークを迎えて 0 に落ちる -> ハンドルが軽くなる。
    Returns (torque, saturation 0..1)."""
    g = 9.81
    L = car.a + car.b
    vx, vyf = st["vx"], st["vy"] + car.a * st["r"]   # lateral speed at the front axle

    # Slip of the tyre against its own rolling direction, valid in reverse too.
    # タイヤの転がり方向に対するスリップ (後退時も有効)。
    along = vx * math.cos(delta) + vyf * math.sin(delta)
    across = vx * math.sin(delta) - vyf * math.cos(delta)
    tan_a = across / max(abs(along), 0.5)

    mu = T["mu"]
    fz = car.mass * (g * car.b - st["ax"] * T["cg_height"]) / L
    fz = max(fz, 0.2 * car.mass * g * car.b / L)
    # Braking / traction uses up part of the friction circle. 摩擦円。
    mu_lat = mu * math.sqrt(max(0.05, 1.0 - (st["ax"] / (mu * g)) ** 2))

    u = min(1.0, abs(tan_a) / math.tan(math.radians(T["slip_peak_deg"])) * mu / mu_lat)
    fy = mu_lat * fz * (3 * u - 3 * u * u + u ** 3)
    mz = mu_lat * fz * T["contact_half_len"] * u * (1 - u) ** 3
    if along < 0:
        mz = 0.0        # pneumatic trail flips in reverse; keep only caster / 後退時は除外
    # pneumatic (tyre) and mechanical (caster) trail, each with its own scale
    # ニューマチックトレールとメカニカルトレールを個別に倍率調整
    torque = -math.copysign(T["pneumatic_trail"] * mz +
                            T["mechanical_trail"] * fy * T["caster_trail"], tan_a)
    return torque, u


class Kick:
    """Decaying yank plus a short suspension ring. 減衰する衝撃 + 短い振動。"""

    def __init__(self, amp, direction, tau, ring_hz, ring_amp):
        self.t0 = time.monotonic()
        self.amp, self.dir, self.tau = amp, direction, tau
        self.hz, self.ring = ring_hz, ring_amp
        if direction == 0.0:
            self.dir = random.choice((-1.0, 1.0)) * 0.3

    def __call__(self, now):
        age = now - self.t0
        if age > 6 * self.tau:
            return None
        env = math.exp(-age / self.tau)
        return self.amp * env * (self.dir + self.ring * math.sin(2 * math.pi * self.hz * age))


KICKS = {   # kind -> (tau [s], ring Hz, ring amplitude)
    "kerb": (0.07, 13.0, 0.8),
    "bump": (0.05, 16.0, 1.2),
    "crash": (0.15, 22.0, 0.7),
}


def output_stage(f):
    knee = T["soft_knee"]
    a = abs(f)
    if a > knee and knee < 1.0:
        a = knee + (1 - knee) * math.tanh((a - knee) / (1 - knee))
    f = math.copysign(a, f)
    # Dead-band compensation, smooth through zero: blending the minimum force
    # in over min_force_blend (it used to be 0.02, a near-step that made the
    # wheel notch and chatter as it crossed centre).
    # 不感帯補正。min_force_blend の幅でなめらかに加える (以前の 0.02 は
    # ほぼ段差で、センター通過時にカクつきの原因だった)。
    mf = T["min_force"]
    return clamp(f * (1 - mf) + mf * math.tanh(f / T["min_force_blend"]), -1.0, 1.0)


def centre_spring(s_norm, kmh):
    """Forza-style centre spring, -1..1 of full force (+ = pushes right).

    ``s_norm`` is the steering position, -1 .. +1 at full lock. Strength
    grows with speed, from spring_standstill to full at spring_full_kmh.
    速度とともに強くなるセンタースプリング。``s_norm`` はフルロックで ±1。
    """
    k = T["spring_standstill"] + (1.0 - T["spring_standstill"]) * \
        smoothstep(0.0, T["spring_full_kmh"], kmh)
    shaped = math.copysign(abs(s_norm) ** T["spring_linearity"], s_norm)
    return -T["center_spring"] * k * shaped


def return_damper(s_norm, rate):
    """Brakes the wheel while it swings back toward centre, -1..1 of full
    force; ``rate`` is the wheel speed in full-lock units per second. Steering
    away from centre isn't damped, so turning in stays light. The weight
    fades over center_zone either side of centre, so there is no step.
    センターへ戻る動きだけを減衰させる (切り込みは軽いまま)。センター前後
    center_zone で滑らかに切り替える。
    """
    if rate == 0.0:
        return 0.0
    toward = -math.copysign(1.0, rate) * s_norm / T["center_zone"]
    return -T["return_damper"] * rate * smoothstep(-1.0, 1.0, toward)


def centre_damper(s_norm):
    """Extra damping close to centre, fading out over center_zone, so the
    wheel settles there instead of overshooting. センター付近の追加ダンピング。"""
    return T["center_damper"] * (1.0 - smoothstep(0.0, T["center_zone"], abs(s_norm)))


# ----------------------------------------------------------------- main ----

def find_hero(world):
    t0, said = time.time(), False
    while True:
        for a in world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name") == "hero":
                return a
        if time.time() - t0 > 2 and not said:
            print("[ffb] waiting for 'hero' vehicle... / 'hero' 車両を待機中...")
            said = True
        time.sleep(0.5)


def drive(wheel, car, steering, strength, verbose):
    """Run the FFB loop until the hero disappears. hero が消えるまで FFB を回す。"""
    kicks = []
    noise = 0.0
    filt = None
    steer_f = 0.0                   # slew-limited steering torque / 変化率制限後のトルク
    rate, s_prev = 0.0, None        # wheel speed [full lock/s] / ハンドル角速度
    last_log = last_poll = last_status = 0.0
    watcher = TuningWatcher()
    t_prev = time.monotonic()
    while car.alive:
        now = time.monotonic()
        dt = max(1e-3, now - t_prev)
        t_prev = now

        # Settings from the dashboard, twice a second. ダッシュボードの設定 (毎秒 2 回)。
        if now - last_poll > 0.5:
            last_poll = now
            if watcher.poll() and T["tire_friction"] != car.tire_friction:
                try:
                    car.set_tire_friction()
                except RuntimeError as e:
                    print(f"[ffb] tyre grip not applied: {e}")
        gain = strength * T["strength"]

        st = car.state
        x = wheel.angle()
        if st is None:
            wheel.force(0.0)
            time.sleep(1.0 / HZ)
            continue

        # Smooth the 20 Hz simulator state up to the FFB rate.
        # 20 Hz のシミュレータ状態を FFB 周期まで平滑化。
        k = 1.0 - math.exp(-dt / (T["smoothing_ms"] / 1000.0))
        if filt is None:
            filt = dict(st)
        for key in ("vx", "vy", "ax", "r", "kmh", "ratio"):
            filt[key] += k * (st[key] - filt[key])
        kmh = filt["kmh"]
        fresh = 1.0 - smoothstep(STALE_SEC, 2 * STALE_SEC, now - st["t"])

        # Live front-wheel angle from the wheel itself (no simulator latency).
        # 前輪舵角はハンドル位置から直接求める (シミュレータの遅延なし)。
        s_cmd = steering.steer(x)
        delta = s_cmd * car.max_steer * filt["ratio"]

        # 1) tyres + centre spring / タイヤ + センタースプリング
        s_norm = s_cmd * 2.0                            # s_cmd is +-0.5 at full lock
        tau_slip, sat = front_axle_torque(car, filt, delta)
        w = smoothstep(T["slip_blend_lo"], max(T["slip_blend_hi"], T["slip_blend_lo"] + 0.1), kmh)
        tau = w * tau_slip - T["kpi_nm"] * s_norm
        target = fresh * (tau / T["torque_full_nm"] + centre_spring(s_norm, kmh))
        # Slew limit: the steering torque glides to its new value instead of
        # stepping, which is what makes the return to centre feel smooth.
        # 変化率制限: トルクが段差なく新しい値へ移るため、センターへの戻りが滑らか。
        step = T["torque_slew"] * dt
        steer_f += clamp(target - steer_f, -step, step)
        # Return damper from the wheel's own encoder (12 ms low-pass), applied
        # directly: damping must not lag. エンコーダ由来のリターンダンパー。
        if s_prev is not None:
            rate += (1.0 - math.exp(-dt / 0.012)) * ((s_norm - s_prev) / dt - rate)
        s_prev = s_norm
        f = steer_f + return_damper(s_norm, rate)

        # 2) road texture + scrub buzz / 路面振動 + スクラブ振動
        noise += (1.0 - math.exp(-dt * 2 * math.pi * 25)) * (random.uniform(-1, 1) - noise)
        feel = T["road_feel"] if st["surface"] == "road" else T["offroad_feel"]
        tex = feel * TEXTURE[st["surface"]] * min(1.5, kmh / 60.0)
        tex += T["scrub_buzz"] * smoothstep(0.8, 1.0, sat) * min(1.0, kmh / 40.0)
        f += fresh * tex * noise * 3.0                   # low-passed noise is ~1/3 amplitude

        # 3) kerbs, bumps, crashes / 縁石・突き上げ・衝突
        for kind, amp, direction in car.take_events():
            kicks.append(Kick(amp, direction, *KICKS[kind]))
        alive = []
        for kick in kicks:
            v = kick(now)
            if v is not None:
                f += v
                alive.append(kick)
        kicks = alive

        f *= gain

        # 4) steering lock end stop / ロック位置のエンドストップ
        if steering.x_lock is not None and abs(x) > steering.x_lock:
            f -= math.copysign(T["end_stop_gain"] * (abs(x) - steering.x_lock), x)

        out = output_stage(f)
        wheel.force(out)

        # Hardware damper / friction follow speed, plus the centre damper that
        # settles the wheel on centre. 速度に追従するダンパー・摩擦 + センターダンパー。
        damper = gain * (min(T["damper_max"], T["damper_base"] + T["damper_per_kmh"] * kmh) +
                         centre_damper(s_norm))
        friction = gain * (T["friction_column"] +
                           T["friction_park"] * math.exp(-kmh / T["friction_park_kmh"]))
        wheel.condition(ecodes.FF_DAMPER, damper)
        wheel.condition(ecodes.FF_FRICTION, friction)

        if now - last_status > 0.2:
            last_status = now
            ffb_settings.write_status({
                "hero": car.hero.type_id, "kmh": round(kmh, 1), "wheel": round(x, 3),
                "steer": round(s_norm, 3), "front_sat": round(sat, 3),
                "torque": round(f, 3), "output": round(out, 3),
                "damper": round(min(damper, 1.0), 3), "friction": round(min(friction, 1.0), 3),
                "surface": st["surface"], "tire_friction": car.tire_friction,
                "tuning_mtime": watcher.mtime})

        if verbose and now - last_log > 0.5:
            last_log = now
            print(f"[ffb] {kmh:5.1f} km/h  delta {math.degrees(delta):+5.1f} deg  "
                  f"front sat {sat:4.2f}  torque {f:+5.2f}  surface {st['surface']:8s}")

        time.sleep(max(0.0, 1.0 / HZ - (time.monotonic() - now)))


def supervise():
    """Run the FFB in a child process and restart it when it dies.
    A server restart or map change makes libcarla abort() from one of its own
    threads ("terminate called ... TimeoutException"), which Python cannot
    catch, so the only robust recovery is a fresh process.
    FFB を子プロセスで動かし、終了したら再起動する。サーバ再起動やマップ変更で
    libcarla が内部スレッドから abort() し Python では捕捉できないため。"""
    cmd = [sys.executable, "-u", os.path.abspath(__file__), "--child"] + sys.argv[1:]
    try:
        while True:
            rc = subprocess.call(cmd)
            if rc == 0:
                return
            print(f"[ffb] FFB process exited ({rc}); CARLA restarting? retrying in 3 s "
                  f"/ FFB プロセス終了 ({rc})。3 秒後に再接続")
            time.sleep(3)
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--strength", type=float, default=STRENGTH,
                    help="overall force multiplier, on top of the dashboard's "
                         "'Force feedback scale' / 全体の強さ (default %(default)s)")
    ap.add_argument("--verbose", action="store_true",
                    help="print speed / slip / torque twice a second / 状態を表示")
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if not args.child:
        supervise()
        return

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    steering = SteeringMap()
    wheel = Wheel(find_wheel())
    print("[ffb] running - Ctrl+C to stop / 実行中 - Ctrl+C で終了")

    try:
        while True:
            wheel.force(0.0)
            hero = find_hero(world)
            print(f"\n[ffb] attached to {hero.type_id} (id {hero.id}) / hero に接続")
            car = Car(world, hero, steering)
            try:
                drive(wheel, car, steering, args.strength, args.verbose)
            finally:
                car.close()
            print("[ffb] hero gone, waiting for a new one / hero 消失、再接続を待機")
    except KeyboardInterrupt:
        pass
    finally:
        wheel.close()
        print("[ffb] stopped / 停止")


if __name__ == "__main__":
    main()
