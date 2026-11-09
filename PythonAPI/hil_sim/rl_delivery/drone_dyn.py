"""Quadcopter physics for the flight brain (pretend world and CARLA test).

A rigid body with thrust along its tilted up-axis, gravity and quadratic
drag on the air-relative velocity (so wind pushes it). The motors and the
attitude loop react with lag, so the drone can't change instantly: that is
where overshoot after a gust comes from.

Commands (each -1..1): roll, pitch (tilt up to MAX_TILT), thrust (0..2x
hover), yaw rate. The brain only gets what real sensors give (noisy GPS
position/velocity, IMU acceleration, attitude); the wind is never passed in.

クアッドコプターの物理（推力・重力・空気抵抗・応答遅れ）。風は観測に含めない。
"""
import math

import numpy as np

G = 9.81
MAX_TILT = math.radians(30.0)
DRAG = 0.018              # per unit mass, terminal ~16 m/s at 25 deg / 空気抵抗
TAU_ATT, TAU_THR = 0.15, 0.10     # attitude / thrust response lag [s] / 応答遅れ
MAX_YAW_RATE = math.radians(90.0)
HOVER_PCT_S = 0.25        # battery %/s at hover (delivery.battery.BatteryModel.hover)


class Drone(object):
    def __init__(self, pos, yaw=0.0):
        self.p = np.array(pos, float)
        self.v = np.zeros(3)
        self.roll = self.pitch = 0.0
        self.yaw = yaw
        self.thr = 1.0           # thrust as a multiple of hover / ホバー比の推力
        self.acc = np.zeros(3)
        self.battery = 100.0
        self.battery_drop = 0.0

    def step(self, cmd, wind, dt):
        r_c = float(np.clip(cmd[0], -1, 1)) * MAX_TILT
        p_c = float(np.clip(cmd[1], -1, 1)) * MAX_TILT
        t_c = 1.0 + float(np.clip(cmd[2], -1, 1))
        y_c = float(np.clip(cmd[3], -1, 1)) * MAX_YAW_RATE
        a, b = dt / (TAU_ATT + dt), dt / (TAU_THR + dt)
        self.roll += (r_c - self.roll) * a
        self.pitch += (p_c - self.pitch) * a
        self.thr += (t_c - self.thr) * b
        self.yaw = (self.yaw + y_c * dt + math.pi) % (2 * math.pi) - math.pi
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        # body up-axis in the world (pitch forward tilts toward +x body) / 機体の上方向
        up = np.array([cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr])
        rel = self.v - wind
        drag = -DRAG * np.linalg.norm(rel) * rel
        self.acc = up * self.thr * G + np.array([0.0, 0.0, -G]) + drag
        self.v += self.acc * dt
        self.p += self.v * dt
        self.battery_drop = HOVER_PCT_S * self.thr ** 1.5 * dt
        self.battery -= self.battery_drop
        return self.acc

    def sensors(self, rng):
        """What a real drone knows: noisy GPS, IMU, attitude. / 実機のセンサー相当"""
        return (self.p + rng.normal(0, 0.3, 3), self.v + rng.normal(0, 0.1, 3),
                self.acc + rng.normal(0, 0.3, 3), self.roll, self.pitch, self.yaw)
