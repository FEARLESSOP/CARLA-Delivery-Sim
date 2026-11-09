"""Live sensor readings of a Mini Delivery Pod, from what the sim knows (no carla import).

The sim drives each Mini Pod kinematically (delivery/lmh_rig.MiniPod) and
knows its position, heading, speed, wheel odometry, battery, lid / shutter /
flag, what it carries, the obstacles around it and the hub it belongs to. This
module turns that into what the pod's own sensors would report, so the
dashboard can show a real-looking suite (value, unit, health ok / warn / fail
/ off, group, the model part it sits on). ~0.05 ms per pod at 2 Hz.

The suite follows today's sidewalk delivery robots:

================  ===========================================================
lidar             360 deg lidar on the mast (Serve Robotics fits an Ouster
                  OS1 / REV7 digital lidar to every robot [2]; ZMP DeliRo
                  carries a lidar on top for map matching [4]).
cam_*             6 cameras: front, rear, left, right, front-down (kerb and
                  step detection) and the lid / handover camera. Starship
                  robots carry ~9-12 cameras incl. stereo / time-of-flight [1].
ultrasonic        ultrasonic ring (4 front, 2 rear) for the near field, as on
                  Starship robots [1] (less affected by light than cameras).
bumper            contact bumper switch (DeliRo lists bumper sensors [4]).
gnss              GNSS with RTK corrections (Starship: GPS/GNSS + IMU [1]).
imu               6-axis IMU: acceleration, yaw rate, tilt.
wheel1..6         wheel encoders (skid-steer 6-wheel drive, Starship-style).
motor1..6         hub-motor currents, motor_temp the hottest winding.
batt_*            battery pack (BMS): SoC, voltage, current, temperature,
                  range left.
lid / shutter     lid lock and rear roller shutter; flag = the telescopic
                  safety flag; payload = load cell under the floor belt.
lte / latency     LTE / 5G link to the remote operator (Japan's remote-operated
                  small vehicles are watched remotely [3]; HAKOBO / DeliRo run
                  one operator for several robots).
estop             emergency stop (button on the body + remote stop).
================  ===========================================================

Speed: 1.6 m/s, below Japan's 6 km/h limit for 遠隔操作型小型車 (Road Traffic
Act as amended, in force 2023-04-01: max 6 km/h, 120 x 70 x 120 cm, they use
pavements and 路側帯 like pedestrians, notified to the prefectural police) [3].

Sources:
  [1] Starship robot sensors: https://wevolver.com/specs/starship-technologies-starship-robot ,
      https://www.therobotreport.com/neural-networks-delivery-robots/ ,
      https://www.arrow.com/en/research-and-events/articles/the-top-sensors-used-in-autonomous-delivery-robots
  [2] Ouster x Serve Robotics lidar: https://www.businesswire.com/news/home/20220120005327/en/ ,
      https://prnewswire.com/news-releases/serve-robotics-announces-expanded-los-angeles-delivery-extended-lidar-supply-agreement-302184458.html
  [3] METI, 自動配送ロボット / 遠隔操作型小型車 (2023-04-01):
      https://www.meti.go.jp/policy/economy/distribution/deliveryrobot/230124_gaiyou.pdf
  [4] ZMP DeliRo (lidar on top, stereo camera, bumpers, 6 km/h):
      https://monoist.itmedia.co.jp/mn/articles/2008/13/news038.html ,
      https://www.directindustry.com/prod/zmp/product-177677-2330075.html
  Also: Kiwibot (cameras + remote supervisors), Amazon Scout (6-wheel
  sidewalk robot), Panasonic HAKOBO (remote monitoring, Fujisawa).

ミニ配送ポッドのセンサー値（実機相当の値・単位・健全性）。サーバー不要。
"""

import math

from delivery.lmh_rig import WHEEL_R, WHEEL_Y, CRASHED, DOCKED, LOADING, HANDOVER

CAMS = (("cam_front", "Front camera"), ("cam_rear", "Rear camera"), ("cam_left", "Left camera"),
        ("cam_right", "Right camera"), ("cam_down_front", "Front-down camera"),
        ("cam_lid", "Lid / handover camera"))
PACK_V = (39.0, 54.6)        # 13S Li-ion empty..full / 電圧範囲
MASS = 45.0                  # kg empty / 空車質量


def _s(label, value, unit, health, group, part):
    return {"label": label, "value": value, "unit": unit, "health": health, "group": group, "part": part}


class MiniPodSuite(object):
    """Sensors of one pod; ``update(t, snap)`` -> {key: entry}.

    ``snap``: status, x, y, z, yaw, speed, battery, odo_l, odo_r, lid,
    shutter, flag, payload_kg, fail, ranges {front, rear, left, right} (m or
    None, from the rate-limited probe), near (closest walker / car, m or
    None), blocked (bool), hub_d (m to the hub), estop (bool), docked_plug.
    1 台分のセンサー。"""

    def __init__(self, pid, geo=None):
        self.pid, self.geo = pid, geo
        self._last = None            # (t, speed, yaw, odo_l, odo_r, z) / 前回値
        self.temp = 25.0

    def update(self, t, s):
        st = s.get("status")
        fail = s.get("fail") or ""
        crashed = st == CRASHED
        home = st in (DOCKED, LOADING)
        v, yaw = float(s.get("speed", 0.0)), float(s.get("yaw", 0.0))
        dt, acc, rate, tilt = 0.0, 0.0, 0.0, 0.0
        vl = vr = v
        if self._last is not None:
            t0, v0, y0, l0, r0, z0 = self._last
            dt = t - t0
            if dt > 1e-3:
                acc = (v - v0) / dt
                rate = ((yaw - y0 + 180.0) % 360.0 - 180.0) / dt
                vl, vr = (s["odo_l"] - l0) / dt, (s["odo_r"] - r0) / dt
                dist = math.hypot(v * dt, 1e-6)
                tilt = math.degrees(math.atan2(s["z"] - z0, max(dist, 0.05))) if v > 0.05 else 0.0
        self._last = (t, v, yaw, s.get("odo_l", 0.0), s.get("odo_r", 0.0), s.get("z", 0.0))
        out = {}
        h = lambda bad, warn=False: "fail" if bad else ("warn" if warn else "ok")
        # perception / 認識
        rng = s.get("ranges") or {}
        vals = [r for r in rng.values() if r is not None]
        near = s.get("near")
        lidar_min = min(vals + ([near] if near is not None else [])) if (vals or near is not None) else 30.0
        lidar_bad = fail.startswith("lidar")
        out["lidar"] = _s("360° lidar (nearest return)", "—" if lidar_bad else round(min(lidar_min, 30.0), 2),
                          "m", "fail" if lidar_bad else ("off" if home else "ok"), "perception", "lidar")
        hit = fail.startswith("collision")
        for key, label in CAMS:
            if key == "cam_lid":
                on = st == HANDOVER or s.get("lid", 0) > 0.05 or s.get("shutter", 0) > 0.05
            else:
                on = not home
            bad = hit and key in ("cam_front", "cam_down_front")
            out[key] = _s(label, 0 if (bad or not on) else 30, "fps",
                          "fail" if bad else ("ok" if on else "off"), "perception", key)
        us = rng.get("front")
        us = near if us is None else (us if near is None else min(us, near))
        out["ultrasonic"] = _s("Ultrasonic ring (min)", round(min(us, 4.0), 2) if us is not None else 4.0, "m",
                               "off" if home else h(hit, us is not None and us < 0.6), "perception", "ultrasonic")
        out["bumper"] = _s("Bumper switch", "pressed" if hit else "free", "", h(hit), "perception", "body")
        # navigation / 測位
        x, y = s.get("x", 0.0), s.get("y", 0.0)
        if self.geo is not None:
            lat, lon = self.geo.latlon(x, y)
            out["gnss_pos"] = _s("GNSS position", "%.6f, %.6f" % (lat, lon), "°", "ok", "navigation", "gnss")
            out["heading"] = _s("Heading", round(self.geo.heading(yaw), 1), "°", "ok", "navigation", "gnss")
        acc_m = 0.03 if not home else 0.8          # RTK fixed outside, float in the bay / ベイ内は float
        out["gnss"] = _s("GNSS / RTK accuracy", acc_m, "m", "warn" if home else "ok", "navigation", "gnss")
        out["imu_acc"] = _s("IMU acceleration", round(acc, 2), "m/s²", h(False, abs(acc) > 3.0), "navigation", "body")
        out["imu_rate"] = _s("IMU yaw rate", round(rate, 1), "°/s", "ok", "navigation", "body")
        out["tilt"] = _s("Tilt", round(tilt, 1), "°", h(abs(tilt) > 15.0, abs(tilt) > 8.0), "navigation", "body")
        out["odometer"] = _s("Odometer", round(s.get("km", 0.0), 3), "km", "ok", "navigation", "body")
        # drive / 駆動
        motor_bad = fail.startswith("motor")
        bad_n = int(fail.split(":")[1]) if motor_bad and ":" in fail else 0
        # power for rolling + accelerating + electronics (W) / 必要電力
        p_drive = (MASS + s.get("payload_kg", 0.0)) * (9.81 * 0.03 * abs(v) + max(acc, 0.0) * abs(v))
        p_turn = 25.0 * abs(rate) / 160.0
        amps_total = 0.0
        temps = []
        for k in range(6):
            side_v = vl if k < 3 else vr
            rpm = side_v / (2.0 * math.pi * WHEEL_R) * 60.0
            dead = motor_bad and bad_n == k + 1
            i = 0.0 if (dead or home) else 0.25 + (p_drive + p_turn) / 6.0 / 48.0 * (1.0 + 0.3 * abs(rate) / 160.0)
            amps_total += i
            temps.append(30.0 + 4.0 * i + (25.0 if dead else 0.0))
            out["wheel%d" % (k + 1)] = _s("Wheel %d encoder" % (k + 1), 0 if dead else round(rpm),
                                          "rpm", h(dead), "drive", "Wheel%d" % (k + 1))
            out["motor%d" % (k + 1)] = _s("Motor %d current" % (k + 1), round(i, 2), "A", h(dead),
                                          "drive", "Wheel%d" % (k + 1))
        out["motor_temp"] = _s("Hottest motor", round(max(temps), 1), "°C",
                               h(max(temps) > 70.0, max(temps) > 50.0), "drive", "body")
        estop = bool(s.get("estop")) or crashed
        out["estop"] = _s("Emergency stop", "pressed" if estop else "armed", "", "warn" if estop else "ok",
                          "drive", "body")
        # power / 電源
        soc = max(0.0, min(100.0, float(s.get("battery", 0.0))))
        volt = PACK_V[0] + (PACK_V[1] - PACK_V[0]) * (soc / 100.0) ** 0.8
        amps = amps_total + (0.0 if home else 1.2)          # electronics + sensors / 電装
        if home and s.get("docked_plug", True) and soc < 100.0:
            amps = -6.0                                     # charging from the hub / 充電中
        self.temp += ((25.0 + 1.5 * abs(amps)) - self.temp) * min(dt / 60.0, 1.0) if dt > 0 else 0.0
        rng_km = max(soc - float(s.get("reserve", 15.0)), 0.0) / max(float(s.get("per_km", 5.0)), 1e-6)
        out["batt_soc"] = _s("Battery SoC", round(soc, 1), "%", h(soc < 5.0, soc < 25.0), "power", "battery")
        out["batt_v"] = _s("Pack voltage", round(volt, 1), "V", h(volt < 41.0), "power", "battery")
        out["batt_i"] = _s("Pack current", round(amps, 2), "A", "ok", "power", "battery")
        out["batt_t"] = _s("Pack temperature", round(self.temp, 1), "°C", h(self.temp > 55.0, self.temp > 45.0),
                           "power", "battery")
        out["batt_range"] = _s("Range left", round(rng_km, 2), "km", h(rng_km < 0.2, rng_km < 0.8), "power", "battery")
        # payload / 荷物
        lid, sh, fl = s.get("lid", 0.0), s.get("shutter", 0.0), s.get("flag", 0.0)
        out["lid"] = _s("Lid lock", "open" if lid > 0.05 else "locked", "", "ok", "payload", "Lid")
        out["shutter"] = _s("Rear shutter", round(sh * 100), "% open", "ok", "payload", "Shutter")
        out["flag"] = _s("Safety flag", round(fl * 100), "% up", "warn" if (not home and fl < 0.5 and v > 0.1)
                         else "ok", "payload", "Flag")
        out["payload"] = _s("Load cell", round(s.get("payload_kg", 0.0), 1), "kg", "ok", "payload", "body")
        # comms / 通信
        d = float(s.get("hub_d", 0.0))
        rssi = -55.0 - 10.0 * math.log10(max(d, 1.0)) * 1.2
        lat = 38.0 + d * 0.01
        out["lte"] = _s("LTE / 5G signal", round(rssi), "dBm", h(rssi < -110.0, rssi < -95.0), "comms", "body")
        out["latency"] = _s("Operator link latency", round(lat), "ms", h(lat > 300.0, lat > 120.0), "comms", "body")
        return out


def damage(pod_damage):
    """The published "damage" block (or None). / 損傷の公開値。"""
    if not pod_damage or not pod_damage.get("parts"):
        return None
    return {"parts": dict(pod_damage["parts"]), "why": pod_damage.get("why", "")}
