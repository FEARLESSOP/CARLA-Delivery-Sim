"""How a delivery drone is SHOWN: a realistic quadcopter motion on top of the sim's
flight logic (no carla import; tests/test_drone_motion.py).

The flight logic (delivery/drone.py) now speeds up and brakes like a multirotor, but
turns its heading instantly and never tilts. This layer draws the body like a real
quad-X at the flight's own position: it turns with a limited yaw rate and yaw
acceleration, and tilts into its motion: nose down when it speeds up forwards (and a
steady forward lean in cruise, against the air drag), nose up when it brakes or flies
backwards, and banks sideways when it moves sideways. Tilts are eased with a short
time constant. Only the drawing uses it; positions for deliveries, landings, crash
checks and batteries stay the logical ones.

配送ドローンの「見せ方」：飛行ロジックの位置・向きに、実機のクアッドコプターらしい動きを
重ねる。加速度と旋回速度に上限をつけて追従し、進行方向へ傾く（加速で前傾、巡航中も空気抵抗で
わずかに前傾、減速・後退で後傾、横移動で横に傾く）。描画だけに使い、配達・着陸・衝突判定・
電池は論理上の位置のまま。
"""
import math

G = 9.81
OMEGA = 3.0            # follow stiffness [rad/s]: ~0.5 s to settle / 追従の速さ
ACC_MAX = 4.0          # horizontal acceleration limit [m/s^2] / 水平加速度の上限
ACC_Z_MAX = 3.0        # vertical acceleration limit [m/s^2] / 鉛直加速度の上限
SNAP_M = 25.0          # further than this from the logic: jump (teleports, repairs) / これ以上離れたら瞬間移動
YAW_RATE_MAX = 90.0    # deg/s / 最大旋回速度
YAW_ACC_MAX = 180.0    # deg/s^2 / 最大旋回加速度
YAW_GAIN = 2.0         # 1/s: wanted yaw rate per degree of error / 誤差 1 度あたりの目標旋回速度
DRAG_K = 0.0183        # lean against drag: 15 deg at 12 m/s cruise / 巡航 12 m/s で約 15 度の前傾
TILT_MAX = 25.0        # deg / 最大傾斜
TILT_TAU = 0.25        # s: easing of the tilt / 傾きの時定数


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class VisualPose(object):
    """Shown pose of one drone. ``update(dt, pos, yaw)`` -> (pos, yaw, pitch, roll),
    degrees in CARLA's convention (pitch > 0 nose up, roll > 0 right side down,
    yaw clockwise from +x). The position is the flight's own (the flight now speeds
    up and brakes, delivery/drone.py ``_ramp_step``), so the body never lags; the tilt
    comes from its velocity and acceleration, the heading turns at a limited rate.
    1 機の表示姿勢。位置は飛行ロジックのまま（遅れなし）、傾きは速度と加速度から、向きは旋回速度に上限。"""
    VEL_TAU, ACC_TAU = 0.15, 0.2           # s: smoothing of the measured velocity / acceleration

    def __init__(self):
        self.p = None

    def reset(self, pos, yaw):
        """At rest at ``pos`` / ``yaw`` (in the pod, a repair...). / 静止状態に合わせる"""
        self.p = [float(pos[0]), float(pos[1]), float(pos[2])]
        self.v = [0.0, 0.0, 0.0]
        self.a = [0.0, 0.0, 0.0]
        self.yaw = float(yaw)
        self.yaw_rate = 0.0
        self.pitch = self.roll = 0.0

    def update(self, dt, pos, yaw):
        if self.p is None or dt <= 0.0:
            if self.p is None:
                self.reset(pos, yaw)
            return self.pose()
        dt = min(float(dt), 0.2)
        step = [float(pos[i]) - self.p[i] for i in range(3)]
        if math.sqrt(sum(e * e for e in step)) > SNAP_M:
            self.reset(pos, yaw)
            return self.pose()
        kv = 1.0 - math.exp(-dt / self.VEL_TAU)
        ka = 1.0 - math.exp(-dt / self.ACC_TAU)
        for i in range(3):
            v_new = self.v[i] + (step[i] / dt - self.v[i]) * kv
            a_meas = (v_new - self.v[i]) / dt
            self.a[i] += (_clamp(a_meas, -3.0 * ACC_MAX, 3.0 * ACC_MAX) - self.a[i]) * ka
            self.v[i] = v_new
            self.p[i] = float(pos[i])
        # yaw: limited rate and acceleration / 向き：旋回速度と加速度に上限
        want = _clamp(wrap180(float(yaw) - self.yaw) * YAW_GAIN, -YAW_RATE_MAX, YAW_RATE_MAX)
        self.yaw_rate += _clamp(want - self.yaw_rate, -YAW_ACC_MAX * dt, YAW_ACC_MAX * dt)
        self.yaw = wrap180(self.yaw + self.yaw_rate * dt)
        # tilt into the motion, in the body frame / 機体座標で進行方向へ傾く
        y = math.radians(self.yaw)
        fx, fy = math.cos(y), math.sin(y)                  # nose / 機首
        rx, ry = -math.sin(y), math.cos(y)                 # right side (CARLA: y south) / 右
        vf = self.v[0] * fx + self.v[1] * fy
        vr = self.v[0] * rx + self.v[1] * ry
        af = self.a[0] * fx + self.a[1] * fy + DRAG_K * vf * abs(vf)
        ar = self.a[0] * rx + self.a[1] * ry + DRAG_K * vr * abs(vr)
        pitch_t = _clamp(-math.degrees(math.atan2(af, G)), -TILT_MAX, TILT_MAX)
        roll_t = _clamp(math.degrees(math.atan2(ar, G)), -TILT_MAX, TILT_MAX)
        k = 1.0 - math.exp(-dt / TILT_TAU)
        self.pitch += (pitch_t - self.pitch) * k
        self.roll += (roll_t - self.roll) * k
        return self.pose()

    def pose(self):
        return tuple(self.p), self.yaw, self.pitch, self.roll


def nadir_mount(body_xyz, body_yaw, offset):
    """World pose of a camera that always looks straight down: it rides at ``offset``
    (x forward, y right, z up, metres, body frame) turned with the body's heading
    only, so the drone's pitch and roll never tilt it. Returns ((x, y, z), (pitch,
    yaw, roll)). / 常に真下を向くカメラの位置と向き（機体の向きだけに追従し、傾きは無視）。"""
    y = math.radians(body_yaw)
    ox, oy, oz = offset
    x = body_xyz[0] + ox * math.cos(y) - oy * math.sin(y)
    yy = body_xyz[1] + ox * math.sin(y) + oy * math.cos(y)
    return (x, yy, body_xyz[2] + oz), (-90.0, float(body_yaw), 0.0)
