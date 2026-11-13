#!/usr/bin/env python
"""Live telemetry dashboard for the CARLA hero vehicle.

Connects to a running CARLA simulator, looks for the actor whose role_name
is 'hero', attaches GNSS / IMU / collision sensors to it, and serves a
small Flask web page with live vehicle state.

Run:
    python dashboard.py --host 127.0.0.1 --port 2000 --dashboard-port 5000

Then open http://localhost:5000 in a browser.

CARLA のヒーロー車両向けライブテレメトリダッシュボード。

起動中の CARLA シミュレータに接続し、role_name が 'hero' のアクターを探して
GNSS / IMU / 衝突センサーを取り付け、車両状態をライブ表示する小さな
Flask ウェブページを提供する。

実行方法:
    python dashboard.py --host 127.0.0.1 --port 2000 --dashboard-port 5000

その後、ブラウザで http://localhost:5000 を開く。
"""

import argparse
import csv
import glob
import json
import math
import os
import signal as _signal     # not "signal": a route below is called that / ルート名と衝突回避
import socket
import subprocess
import sys
import threading
import time
from collections import deque

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from flask import Flask, Response, jsonify, request, send_file, send_from_directory

from delivery import live as delivery_live
from delivery import zones as delivery_zones
import drone_windows
import minipod_windows
import ffb_settings
import keybinds
import seasons
import vehicle_tuning
import weather_cycle
import road_overlay
from lane_limits import LaneLimits
import windows_control
import launch_options
import pipeline_control


STATE_LOCK = threading.Lock()
STATE = {
    "connected": False,
    "hero_found": False,
    "map": "",
    "speed_kmh": 0.0,
    "speed_limit_kmh": 0.0,
    "speed_delta_kmh": 0.0,
    "throttle": 0.0,
    "brake": 0.0,
    "steer": 0.0,
    "reverse": False,
    "hand_brake": False,
    "gear": 0,
    "location": {"x": 0.0, "y": 0.0, "z": 0.0},
    "heading_deg": 0.0,
    "compass_deg": 0.0,
    "gnss": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
    "imu": {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0},
    # Derived dynamics / 派生的な車両運動量
    "lat_accel": 0.0,
    "lon_accel": 0.0,
    "vert_accel": 0.0,
    "yaw_rate_deg": 0.0,
    "jerk": 0.0,
    "g_force": 0.0,
    # Safety / environment / 安全・環境関連
    "traffic_light": "None",
    "at_traffic_light": False,
    "front_distance": -1.0,
    "ttc": -1.0,
    "hard_brake_count": 0,
    "collision_count": 0,
    # Lane / road / 車線・道路情報
    "on_road": False,
    "lane_width": 0.0,
    "lane_offset": 0.0,   # signed: + = right of lane centre / 符号付き：+ は車線中心より右側
    "lane_id": 0,
    "road_id": 0,
    "at_junction": False,
    # Driver-performance metrics (human-in-the-loop research) / 運転パフォーマンス指標（HIL研究用）
    "thw": -1.0,                  # time headway to lead vehicle (s) / 先行車への車間時間（秒）
    "sdlp": 0.0,                  # std dev of lateral lane position, 60 s (m)
    # SDLP：直近60秒の車線内横位置の標準偏差（m）
    "steer_reversal_rate": 0.0,   # steering reversals per minute / 1分あたりのステアリング反転回数
    "lane_invasions": 0,
    "red_light_violations": 0,
    "speed_compliance": 100.0,    # % of moving time at/below limit / 制限速度以下で走行した時間の割合（%）
    "jerk_rms": 0.0,
    "events": [],                 # recent driving events (newest first) / 最近の運転イベント（新しい順）
    "log_file": "",
    # Trip / 走行記録
    "trip_km": 0.0,
    "trip_sec": 0.0,
    "avg_kmh": 0.0,
    # Collision / 衝突
    "collision_peak": 0.0,
    "collision_recent": [],
    "collision_hits": [],   # [{age, f, r, i, o}] impact points, vehicle frame
                            # 車両ローカル座標系での衝突位置（前方f・右方r、-1..1）
    # Context / 周辺状況
    "nearby_vehicles": 0,
    "nearby_list": [],      # [{fx, rx, d, yaw, kmh, kind}, ...] relative to hero / ヒーロー車基準の相対値
    # All sensors attached to the hero in the sim (any script)
    # シミュレーション内でヒーロー車に取り付けられた全センサー（他スクリプト含む）
    "sensors": [],          # [{type, f, r}] mounting position in vehicle frame / 車両座標系での取付位置
    "ultrasonic_m": -1.0,   # front obstacle (ultrasonic-style) sensor distance / 前方障害物（超音波型）センサー距離
    "bb": {"x": 2.4, "y": 1.0},  # hero bounding-box half extents (m) / 自車バウンディングボックス半径（m）
    "trail": [],
    "timestamp": 0.0,
    # Weather / environment / 天候・環境
    "weather": {
        "cloudiness": 0.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 0.0,
        "sun_azimuth": 0.0,
        "sun_altitude": 0.0,
        "fog_density": 0.0,
        "wetness": 0.0,
    },
    # Autonomous-driving indicators / 自動運転関連の指標
    "abs_active": False,      # approximate: hard brake + high slip signature
    # 近似値：急ブレーキ＋大スリップの兆候
    "tcs_active": False,      # approximate: high throttle + low accel / 近似値：高スロットル＋低加速度
    "stability": 1.0,         # 0..1, 1 = stable / 0〜1、1 = 安定
    "slip_angle_deg": 0.0,    # angle between heading and velocity / 車体方位と速度ベクトルの角度
    "curvature": 0.0,         # 1/m of current trajectory / 現在の軌跡の曲率（1/m）
    # Vehicle physical / 車両の物理特性
    "vehicle_type": "",
    "num_wheels": 4,
    "mass_kg": 0.0,
    "engine_rpm": 0.0,
    # Turn signal / ウインカー
    "turn_signal": "off",
    # Simulation meta / シミュレーションのメタ情報
    "sim_fps": 0.0,
    "sim_time_sec": 0.0,
}

TRAIL = deque(maxlen=400)
GNSS_DATA = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
IMU_DATA = {"ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0, "compass": 0.0}
# Front obstacle ("ultrasonic") sensor; stale after 0.6 s without an event.
# 前方障害物（超音波）センサー。0.6秒イベントが無ければ無効扱い。
OBST_DATA = {"t": 0.0, "d": -1.0}

# Turn-signal state, shared with the G923 wheel script via turn_signal.json
# (wheel L3 = left, R3 = right by default, see keybinds.json; the dashboard
# buttons POST /signal). "hazard" = both blinkers (not a lane-change signal).
# While a signal is on, crossing a broken lane marking counts as a lane
# change instead of a lane invasion.
# ウインカー状態。turn_signal.json を介して G923 スクリプトと共有
# （ホイールのボタン11=左・10=右、ダッシュボードのボタンは /signal へPOST）。
# 点灯中に破線を跨いだ場合は車線逸脱ではなく車線変更として扱う。
SIGNAL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "turn_signal.json")
SIGNAL = {"side": "off", "mtime": 0.0}


def _load_signal():
    """Pick up turn-signal toggles written by the wheel script.

    ホイールスクリプトが書き込んだウインカー切替を取り込む。
    """
    try:
        mtime = os.path.getmtime(SIGNAL_PATH)
    except OSError:
        return
    if mtime == SIGNAL["mtime"]:
        return
    SIGNAL["mtime"] = mtime
    try:
        with open(SIGNAL_PATH) as fh:
            side = json.load(fh).get("side", "off")
        SIGNAL["side"] = side if side in ("left", "right", "hazard") else "off"
    except Exception:
        pass


def _save_signal(side):
    SIGNAL["side"] = side
    try:
        tmp = SIGNAL_PATH + ".tmp"
        with open(tmp, "w") as fh:
            json.dump({"side": side}, fh)
        os.replace(tmp, SIGNAL_PATH)
        SIGNAL["mtime"] = os.path.getmtime(SIGNAL_PATH)
    except Exception:
        pass
COLL_EVENTS = deque(maxlen=200)
# Distinct crash impact points for the vehicle diagram / 車両図用の衝突位置
COLL_HITS = deque(maxlen=20)
EVENTS = deque(maxlen=80)  # (wall_time, type, message) / （実時刻, 種別, メッセージ）

# CSV session log (one per dashboard run), stored under
# <log-root>/<YYYY-MM-DD>/<HH-MM-SS>/telemetry.csv with a summary.json
# next to it that is refreshed every few seconds.
# CSV セッションログ（ダッシュボード実行ごとに1つ）。
# <log-root>/<YYYY-MM-DD>/<HH-MM-SS>/telemetry.csv に保存され、
# 隣に置かれる summary.json は数秒ごとに更新される。
LOG = {"fh": None, "writer": None, "path": "", "dir": "", "root": "",
       "summary_path": "", "last_flush": 0.0, "last_summary": 0.0,
       "date": "", "time": ""}
LOG_COLS = [
    "wall_time", "sim_time", "x", "y", "z", "speed_kmh", "speed_limit_kmh",
    "throttle", "brake", "steer", "gear", "lon_accel", "lat_accel", "jerk",
    "yaw_rate_deg", "slip_angle_deg", "lane_offset", "lane_width", "on_road",
    "front_distance", "ttc", "thw", "sdlp", "steer_reversal_rate",
    "hard_brake_count", "lane_invasions", "red_light_violations",
    "speed_compliance", "collision_peak", "collision_count",
]


def add_event(etype, msg):
    EVENTS.append((time.time(), etype, msg))

# Per-hero derived state (reset on hero change). / ヒーロー車ごとの派生状態（車両変更時にリセット）
DRV_STATE = {}


def _reset_drv_state():
    DRV_STATE.clear()
    DRV_STATE.update({
        "prev_accel": None,
        "prev_t": None,
        "last_pos": None,
        "trip_m": 0.0,
        "hard_brake_count": 0,
        "in_hard_brake": False,
        "trip_start": None,
        # Driver-performance state / 運転パフォーマンス関連の状態
        "lane_invasions": 0,
        "red_light_violations": 0,
        "collision_count": 0,
        "last_coll_t": None,
        "last_red_tl": None,          # traffic-light actor while at a light / 信号待ち中の信号機アクター
        "moving_time": 0.0,
        "speeding_time": 0.0,
        "last_loop_t": None,
        "offset_hist": deque(maxlen=1200),   # (t, signed lane offset) / （時刻, 符号付き車線オフセット）
        "jerk_hist": deque(maxlen=300),
        "steer_prev": None,
        "steer_dir": 0,
        "steer_extreme": 0.0,
        "reversal_times": deque(maxlen=600),
    })
    EVENTS.clear()


_reset_drv_state()


def find_hero(world):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == "hero":
            return actor
    return None


def attach_sensors(world, hero):
    bp_lib = world.get_blueprint_library()
    sensors = []

    gnss_bp = bp_lib.find("sensor.other.gnss")
    gnss = world.spawn_actor(
        gnss_bp,
        carla.Transform(carla.Location(x=1.0, z=2.8)),
        attach_to=hero,
    )
    gnss.listen(lambda ev: GNSS_DATA.update(
        {"lat": ev.latitude, "lon": ev.longitude}))
    sensors.append(gnss)

    imu_bp = bp_lib.find("sensor.other.imu")
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=hero)

    def on_imu(ev):
        IMU_DATA.update({
            "ax": ev.accelerometer.x,
            "ay": ev.accelerometer.y,
            "az": ev.accelerometer.z,
            "gx": ev.gyroscope.x,
            "gy": ev.gyroscope.y,
            "gz": ev.gyroscope.z,
            "compass": math.degrees(ev.compass),
        })

    imu.listen(on_imu)
    sensors.append(imu)

    coll_bp = bp_lib.find("sensor.other.collision")
    coll = world.spawn_actor(coll_bp, carla.Transform(), attach_to=hero)

    def on_coll(ev):
        i = ev.normal_impulse
        intensity = math.sqrt(i.x * i.x + i.y * i.y + i.z * i.z)
        now = time.time()
        COLL_EVENTS.append((now, intensity))
        # A single crash produces a burst of contact events; count a new
        # crash only after 2 s without contact.
        # 1回の衝突では接触イベントが連続して発生するため、接触が
        # 2秒間途切れた後にのみ新しい衝突としてカウントする。
        last = DRV_STATE.get("last_coll_t")
        DRV_STATE["last_coll_t"] = now
        if last is None or now - last > 2.0:
            DRV_STATE["collision_count"] = DRV_STATE.get(
                "collision_count", 0) + 1
            try:
                other = ev.other_actor.type_id
            except Exception:
                other = "unknown"
            # Impact point in the vehicle frame (forward f, right r),
            # normalised by the bounding-box half extents to -1..1 so the
            # UI can pin it on the car outline.  Prefer the other actor's
            # position; fall back to the impulse direction for walls etc.
            # 車両ローカル座標系での衝突位置（前方f・右方r）。バウンディング
            # ボックス半径で -1..1 に正規化。相手アクターの位置を優先し、
            # 壁などではインパルス方向から推定する。
            fn, rn = 0.0, 0.0
            try:
                ht = hero.get_transform()
                fwd, right = _yaw_to_forward_right(ht.rotation.yaw)
                ext = hero.bounding_box.extent
                dx = dy = 0.0
                try:
                    ol = ev.other_actor.get_transform().location
                    dx = ol.x - ht.location.x
                    dy = ol.y - ht.location.y
                except Exception:
                    pass
                if abs(dx) < 1e-3 and abs(dy) < 1e-3 and intensity > 0:
                    dx, dy = -i.x / intensity, -i.y / intensity
                fu = (dx * fwd[0] + dy * fwd[1]) / max(ext.x, 0.1)
                ru = (dx * right[0] + dy * right[1]) / max(ext.y, 0.1)
                m = max(abs(fu), abs(ru), 1.0)
                fn, rn = fu / m, ru / m
            except Exception:
                pass
            COLL_HITS.append({
                "t": now, "f": round(fn, 3), "r": round(rn, 3),
                "i": intensity,
                "o": other.split(".", 1)[-1] if other else "unknown"})
            add_event("collision",
                      "with %s (impulse %.0f)" % (other, intensity))

    coll.listen(on_coll)
    sensors.append(coll)

    lane_bp = bp_lib.find("sensor.other.lane_invasion")
    lane = world.spawn_actor(lane_bp, carla.Transform(), attach_to=hero)

    def on_lane(ev):
        kinds = sorted(set(str(m.type).split(".")[-1]
                           for m in ev.crossed_lane_markings))
        # With the turn signal on, crossing broken markings is a legal,
        # signalled lane change — don't count it as a violation.
        # ウインカー点灯中の破線跨ぎは合法な車線変更なので違反に数えない。
        legal = (SIGNAL["side"] in ("left", "right") and kinds
                 and all(k in ("Broken", "BrokenBroken") for k in kinds))
        if legal:
            add_event("lane_change",
                      "signalled lane change (%s)" % SIGNAL["side"])
        else:
            DRV_STATE["lane_invasions"] = DRV_STATE.get(
                "lane_invasions", 0) + 1
            add_event("lane", "crossed %s" % ", ".join(kinds))

    lane.listen(on_lane)
    sensors.append(lane)

    # Ultrasonic-style front obstacle sensor (ray-cast proximity probe).
    # 超音波型の前方障害物センサー（レイキャストによる近接プローブ）。
    try:
        obs_bp = bp_lib.find("sensor.other.obstacle")
        obs_bp.set_attribute("distance", "20")
        obs_bp.set_attribute("hit_radius", "0.4")
        obs_bp.set_attribute("only_dynamics", "false")
        ext = hero.bounding_box.extent
        obs = world.spawn_actor(
            obs_bp,
            carla.Transform(carla.Location(x=float(ext.x) + 0.3, z=0.6)),
            attach_to=hero,
        )

        def on_obs(ev):
            OBST_DATA.update({"t": time.time(), "d": float(ev.distance)})

        obs.listen(on_obs)
        sensors.append(obs)
    except Exception as exc:
        print("[dashboard] obstacle sensor unavailable: %s" % exc,
              file=sys.stderr)

    return sensors


def _classify_actor(actor):
    """Rough class of a nearby actor: walker / bike / truck / car.

    周辺アクターの大まかな分類：walker（歩行者）/ bike（二輪）/
    truck（トラック等）/ car（乗用車）。
    """
    tid = actor.type_id
    if tid.startswith("walker."):
        return "walker"
    attrs = actor.attributes
    base = attrs.get("base_type", "")
    if base in ("bicycle", "motorcycle") or \
            attrs.get("number_of_wheels", "4") == "2":
        return "bike"
    if base in ("truck", "van", "bus"):
        return "truck"
    return "car"


def _yaw_to_forward_right(yaw_deg):
    """Return (forward, right) unit 2D vectors for given yaw (degrees, CARLA).

    指定ヨー角（度、CARLA座標系）に対する（前方, 右方）の2D単位ベクトルを返す。
    """
    yr = math.radians(yaw_deg)
    fx, fy = math.cos(yr), math.sin(yr)
    # CARLA/UE4 is left-handed with X forward, Y RIGHT, Z up, so the right
    # vector is forward rotated +90° about Z (matches get_right_vector()).
    # CARLA/UE4 は左手系（X前方・Y右・Z上）なので、右方向ベクトルは
    # forward を Z 軸まわりに +90度 回転したもの（get_right_vector() と一致）。
    rx, ry = math.cos(yr + math.pi / 2.0), math.sin(yr + math.pi / 2.0)
    return (fx, fy), (rx, ry)


def _front_obstacle(hero, others, max_dist=60.0, half_cone_deg=20.0):
    """Nearest actor ahead of hero inside a cone. Return (dist, ttc, actor).

    `others` carries walkers as well as vehicles -- a pedestrian stepping out
    is exactly the front obstacle a driving study wants TTC for.

    ヒーロー車前方のコーン内で最も近いアクターを探し、(距離, TTC, アクター)
    を返す。`others` には車両だけでなく歩行者も含む。飛び出してくる歩行者
    こそ、運転研究で TTC を取りたい前方障害物である。
    """
    t = hero.get_transform()
    (fx, fy), _ = _yaw_to_forward_right(t.rotation.yaw)
    hv = hero.get_velocity()
    hx, hy = t.location.x, t.location.y
    h_fwd_speed = hv.x * fx + hv.y * fy  # m/s along forward / 前方方向の速度（m/s）
    best = (max_dist, -1.0, None)
    cos_lim = math.cos(math.radians(half_cone_deg))
    for a in others:
        al = a.get_location()
        dx, dy = al.x - hx, al.y - hy
        d = math.hypot(dx, dy)
        if d < 0.5 or d > max_dist:
            continue
        # angle check / 角度の確認
        cos_a = (dx * fx + dy * fy) / d
        if cos_a < cos_lim:
            continue
        av = a.get_velocity()
        other_fwd_speed = av.x * fx + av.y * fy
        closing = h_fwd_speed - other_fwd_speed  # m/s positive = approaching / 正なら接近中（m/s）
        ttc = (d / closing) if closing > 0.5 else -1.0
        if d < best[0]:
            best = (d, ttc, a)
    return best


def _open_log(log_dir):
    """Open a session log under <log_dir>/<YYYY-MM-DD>/<HH-MM-SS>/.

    <log_dir>/<YYYY-MM-DD>/<HH-MM-SS>/ 配下にセッションログを開く。
    """
    try:
        day = time.strftime("%Y-%m-%d")
        tod = time.strftime("%H-%M-%S")
        folder = os.path.join(log_dir, day, tod)
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "telemetry.csv")
        fh = open(path, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(LOG_COLS)
        # Flush the header immediately. Rows are flushed only every 2 s, and
        # a session that dies before the first flush -- which is what a
        # CarlaUE4 GPU crash plus the pipeline's SIGKILL sweep looks like --
        # otherwise leaves a 0-byte telemetry.csv with no header, so the run
        # cannot even be identified afterwards.
        # ヘッダは即座にフラッシュする。行のフラッシュは2秒ごとのため、
        # 最初のフラッシュ前にセッションが死ぬと（CarlaUE4 の GPU クラッシュと
        # パイプラインの SIGKILL 掃除がまさにこれ）ヘッダすら無い0バイトの
        # telemetry.csv が残り、後から実行の識別すらできなくなる。
        fh.flush()
        LOG.update({"fh": fh, "writer": writer, "path": path,
                    "dir": folder, "root": log_dir,
                    "summary_path": os.path.join(folder, "summary.json"),
                    "date": day, "time": tod})
        print("[dashboard] logging session to %s" % path)
    except Exception as exc:
        print("[dashboard] could not open log: %s" % exc, file=sys.stderr)


def _write_summary(now):
    """Refresh summary.json for the running session (called every ~5 s).

    実行中セッションの summary.json を更新する（約5秒ごとに呼ばれる）。
    """
    if not LOG.get("summary_path"):
        return
    if now - LOG.get("last_summary", 0.0) < 5.0:
        return
    LOG["last_summary"] = now
    with STATE_LOCK:
        summary = {
            "date": LOG["date"],
            "time": LOG["time"],
            "map": STATE["map"],
            "vehicle": STATE["vehicle_type"],
            "crashes": STATE["collision_count"],
            "out_of_lane": STATE["lane_invasions"],
            "red_light_violations": STATE["red_light_violations"],
            "hard_brakes": STATE["hard_brake_count"],
            "trip_km": round(STATE["trip_km"], 3),
            "trip_sec": round(STATE["trip_sec"], 1),
            "avg_kmh": round(STATE["avg_kmh"], 1),
            "speed_compliance": round(STATE["speed_compliance"], 1),
            "sdlp": round(STATE["sdlp"], 3),
        }
    try:
        tmp = LOG["summary_path"] + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(summary, fh, indent=1)
        os.replace(tmp, LOG["summary_path"])
    except Exception:
        pass


def _rpc_port_open(host, port, timeout=0.5):
    """True if something is accepting TCP on the simulator's RPC port.

    Python's socket calls release the GIL; the carla client's RPC calls do
    not. That difference is the whole point of this probe -- see
    _wait_for_server below.

    シミュレータのRPCポートがTCP接続を受け付けていれば True。

    Python の socket 呼び出しは GIL を解放するが、carla クライアントの RPC
    呼び出しは解放しない。この差こそが、下の _wait_for_server が存在する
    理由である。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            return s.connect_ex((host, port)) == 0
    except OSError:
        return False


def _wait_for_server(host, port):
    """Block until the CARLA RPC port answers, without holding the GIL.

    Every carla client call is a C++ RPC that keeps the GIL for its whole
    duration, timeout included. So a single `client.get_world()` against a
    simulator that is not up freezes *the entire process* for the client
    timeout -- including the Flask thread, and including the main thread
    sitting in `Thread.start()`. The dashboard would then never bind its
    port at all and the browser got "connection refused" rather than a page
    saying "disconnected". Probing with a plain socket first keeps that
    waiting in GIL-releasing code, so the web UI comes up and stays up
    whether or not the simulator is there.

    CARLA の RPC ポートが応答するまで、GIL を保持せずに待機する。

    carla クライアントの呼び出しはすべて C++ の RPC であり、タイムアウトを
    含む実行中ずっと GIL を保持する。そのため、起動していないシミュレータに
    対する `client.get_world()` 1回で、Flask スレッドも `Thread.start()` で
    待機中のメインスレッドも含めた*プロセス全体*がクライアントタイムアウトの
    間フリーズする。結果としてダッシュボードはポートを bind すらできず、
    ブラウザには「切断」表示ではなく接続拒否が返っていた。先に素の socket で
    確認することで待機を GIL 解放側のコードに閉じ込め、シミュレータの有無に
    関わらず Web UI が起動し、動き続ける。
    """
    if _rpc_port_open(host, port):
        return
    with STATE_LOCK:
        STATE["connected"] = False
        STATE["hero_found"] = False
    print("[dashboard] waiting for CARLA at %s:%d "
          "(web UI is already serving)" % (host, port))
    while not _rpc_port_open(host, port):
        time.sleep(1.0)
    print("[dashboard] CARLA RPC port is up")


LANE_LIMITS = LaneLimits()


def telemetry_worker(host, port, log_dir=None):
    # 5 s rather than 10 s: while the simulator is reachable but wedged
    # (a synchronous world with no tick master), this timeout is how long a
    # single RPC holds the GIL and so how long the web UI stops answering.
    # The unreachable case is handled by _wait_for_server, not by this.
    # 10秒ではなく5秒。シミュレータには到達できるが停止している場合
    # （ティックマスタ不在の同期ワールド）、この時間だけ1回の RPC が GIL を
    # 保持し、その間 Web UI が応答しなくなる。到達不能な場合はこの値ではなく
    # _wait_for_server が処理する。
    _wait_for_server(host, port)
    client = carla.Client(host, port)
    client.set_timeout(5.0)
    if log_dir:
        _open_log(log_dir)
    current_hero_id = None
    current_map = ""
    sensors = []
    last_fps_t = time.time()
    frame_count = 0
    sim_fps = 0.0
    carla_map = None
    map_name = ""
    actor_cache = None
    next_actor_scan = 0.0

    # world.get_map() parses the town's OpenDRIVE description. It was called
    # twice per pass of this 10 Hz loop; the map only changes when the town
    # is reloaded, which also invalidates the hero, so fetch it there.
    # world.get_map() は町の OpenDRIVE 定義を解析する。この10Hzループで
    # 毎回2度呼ばれていたが、マップが変わるのは町の再読み込み時だけで、
    # そのとき hero も無効になるため、そちらで取得する。
    while True:
        try:
            world = client.get_world()
            if carla_map is None:
                carla_map = world.get_map()
                map_name = carla_map.name.split("/")[-1]
            hero = find_hero(world)

            # Rough wall-clock fps for the telemetry loop.
            # テレメトリループの実時間ベースの概算FPS。
            frame_count += 1
            tnow = time.time()
            if tnow - last_fps_t >= 1.0:
                sim_fps = frame_count / (tnow - last_fps_t)
                frame_count = 0
                last_fps_t = tnow

            if hero is None:
                with STATE_LOCK:
                    STATE["connected"] = True
                    STATE["hero_found"] = False
                    STATE["map"] = map_name
                for s in sensors:
                    try:
                        s.destroy()
                    except Exception:
                        pass
                sensors = []
                current_hero_id = None
                carla_map = None      # town may be reloading / 町の再読込中かも
                _reset_drv_state()
                time.sleep(0.5)
                continue

            if hero.id != current_hero_id or map_name != current_map:
                actor_cache = None
                carla_map = world.get_map()
                map_name = carla_map.name.split("/")[-1]
                for s in sensors:
                    try:
                        s.destroy()
                    except Exception:
                        pass
                sensors = attach_sensors(world, hero)
                current_hero_id = hero.id
                current_map = map_name
                TRAIL.clear()
                COLL_EVENTS.clear()
                COLL_HITS.clear()
                _reset_drv_state()

            # Turn signals: pick up toggles (wheel script or /signal) and
            # drive the hero's blinker lights accordingly.
            # ウインカー：切替（ホイールまたは /signal）を取り込み、
            # 車両のブリンカーライトに反映する。
            _load_signal()
            if DRV_STATE.get("signal_applied") != SIGNAL["side"]:
                DRV_STATE["signal_applied"] = SIGNAL["side"]
                try:
                    blink = (int(carla.VehicleLightState.LeftBlinker)
                             | int(carla.VehicleLightState.RightBlinker))
                    ls = int(hero.get_light_state()) & ~blink
                    if SIGNAL["side"] in ("left", "hazard"):
                        ls |= int(carla.VehicleLightState.LeftBlinker)
                    if SIGNAL["side"] in ("right", "hazard"):
                        ls |= int(carla.VehicleLightState.RightBlinker)
                    hero.set_light_state(carla.VehicleLightState(ls))
                except Exception:
                    pass

            t = hero.get_transform()
            v = hero.get_velocity()
            a_vec = hero.get_acceleration()
            c = hero.get_control()
            speed_ms = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            speed = 3.6 * speed_ms
            TRAIL.append([t.location.x, t.location.y])

            # Body-frame decomposition of accel & velocity. / 加速度と速度を車体座標系に分解。
            (fx, fy), (rx, ry) = _yaw_to_forward_right(t.rotation.yaw)
            lon_accel = a_vec.x * fx + a_vec.y * fy
            lat_accel = a_vec.x * rx + a_vec.y * ry
            vert_accel = a_vec.z
            g_force = math.sqrt(lon_accel ** 2 + lat_accel ** 2) / 9.81

            # Slip angle: angle between velocity vector and heading.
            # スリップ角：速度ベクトルと車体方位のなす角度。
            if speed_ms > 0.5:
                v_heading = math.degrees(math.atan2(v.y, v.x))
                slip = ((v_heading - t.rotation.yaw + 540.0) % 360.0) - 180.0
            else:
                slip = 0.0

            # Jerk. / 加加速度（ジャーク）。
            now = time.time()
            jerk = 0.0
            if DRV_STATE["prev_accel"] is not None and DRV_STATE["prev_t"] is not None:
                dt = max(1e-3, now - DRV_STATE["prev_t"])
                jerk = (lon_accel - DRV_STATE["prev_accel"]) / dt
            DRV_STATE["prev_accel"] = lon_accel
            DRV_STATE["prev_t"] = now

            # Trip distance + time. / 走行距離と走行時間。
            if DRV_STATE["trip_start"] is None:
                DRV_STATE["trip_start"] = now
            if DRV_STATE["last_pos"] is not None:
                lp = DRV_STATE["last_pos"]
                DRV_STATE["trip_m"] += math.hypot(
                    t.location.x - lp[0], t.location.y - lp[1])
            DRV_STATE["last_pos"] = (t.location.x, t.location.y)
            trip_km = DRV_STATE["trip_m"] / 1000.0
            trip_sec = now - DRV_STATE["trip_start"]
            avg_kmh = (trip_km / (trip_sec / 3600.0)) if trip_sec > 1.0 else 0.0

            # Hard brake detection (rising edge at brake > 0.7 & decel > 4 m/s²).
            # 急ブレーキ検出（brake > 0.7 かつ減速度 > 4 m/s² の立ち上がりエッジ）。
            hard_now = (c.brake > 0.7 and lon_accel < -4.0)
            if hard_now and not DRV_STATE["in_hard_brake"]:
                DRV_STATE["hard_brake_count"] += 1
                add_event("hard_brake",
                          "decel %.1f m/s² at %.0f km/h" % (-lon_accel, speed))
            DRV_STATE["in_hard_brake"] = hard_now

            # Curvature from yaw rate and speed. / ヨーレートと速度から曲率を算出。
            yaw_rate = IMU_DATA.get("gz", 0.0)  # rad/s / 単位：rad/s
            curvature = (yaw_rate / speed_ms) if speed_ms > 1.0 else 0.0

            # Speed limit + traffic light: on your own maps the lane's posted
            # limit (lane_limits.py: no sign marks the legal 60 / 30), else
            # CARLA's last 274 sign. / 制限速度（自作マップは車線の値）と信号。
            try:
                lane_kmh = LANE_LIMITS.limit(carla_map, hero)
                speed_limit = float(lane_kmh) if lane_kmh else float(hero.get_speed_limit())
            except Exception:
                speed_limit = 0.0
            try:
                at_tl = bool(hero.is_at_traffic_light())
                tl_state = "None"
                tl_actor = None
                if at_tl:
                    tl_actor = hero.get_traffic_light()
                    if tl_actor is not None:
                        tl_state = str(tl_actor.get_state()).split(".")[-1]
            except Exception:
                at_tl, tl_state, tl_actor = False, "None", None

            # Red-light violation: we were inside a light's trigger volume and
            # left it while the light is still red and the car is moving.
            # 信号無視：信号のトリガー領域内にいて、信号がまだ赤のまま
            # 走行状態で領域を出た場合に違反とみなす。
            if at_tl and tl_actor is not None:
                DRV_STATE["last_red_tl"] = tl_actor
            elif DRV_STATE.get("last_red_tl") is not None:
                try:
                    exit_state = str(
                        DRV_STATE["last_red_tl"].get_state()).split(".")[-1]
                except Exception:
                    exit_state = "None"
                if exit_state == "Red" and speed_ms > 2.0:
                    DRV_STATE["red_light_violations"] += 1
                    add_event("red_light",
                              "ran red light at %.0f km/h" % speed)
                DRV_STATE["last_red_tl"] = None

            # Lane / waypoint info. / 車線・ウェイポイント情報。
            try:
                wp = carla_map.get_waypoint(
                    t.location, project_to_road=True,
                    lane_type=carla.LaneType.Driving)
                wp_off = carla_map.get_waypoint(
                    t.location, project_to_road=False)
                on_road = wp_off is not None
                lane_width = float(wp.lane_width) if wp else 0.0
                lane_id = int(wp.lane_id) if wp else 0
                road_id = int(wp.road_id) if wp else 0
                at_junction = bool(wp.is_junction) if wp else False
                if wp is not None:
                    # Signed lateral offset: project the vector from lane
                    # centre to the vehicle onto the lane's right vector.
                    # 符号付き横方向オフセット：車線中心から車両への
                    # ベクトルを車線の右方向ベクトルに射影する。
                    wl = wp.transform.location
                    rv = wp.transform.get_right_vector()
                    lane_offset = ((t.location.x - wl.x) * rv.x
                                   + (t.location.y - wl.y) * rv.y)
                else:
                    lane_offset = 0.0
            except Exception:
                on_road, lane_width, lane_offset = False, 0.0, 0.0
                lane_id, road_id, at_junction = 0, 0, False

            # --- Driver-performance metrics (HIL research) ---------------
            # SDLP: std dev of signed lane position over the last 60 s,
            # only while driving on-road at a meaningful speed.
            # --- 運転パフォーマンス指標（HIL研究用） ---------------
            # SDLP：直近60秒の符号付き車線内位置の標準偏差。
            # 道路上を十分な速度で走行中のみ算出する。
            if on_road and speed_ms > 2.0:
                DRV_STATE["offset_hist"].append((now, lane_offset))
            recent_off = [o for ts, o in DRV_STATE["offset_hist"]
                          if now - ts <= 60.0]
            if len(recent_off) >= 10:
                mo = sum(recent_off) / len(recent_off)
                sdlp = math.sqrt(
                    sum((o - mo) ** 2 for o in recent_off) / len(recent_off))
            else:
                sdlp = 0.0

            # Steering reversal rate: direction changes of the steering
            # signal larger than a small gap, per minute (rolling 60 s).
            # ステアリング反転率：一定のギャップを超えるステアリング信号の
            # 方向転換の1分あたり回数（直近60秒のローリング集計）。
            st = c.steer
            prev_st = DRV_STATE["steer_prev"]
            if prev_st is not None and abs(st - prev_st) > 1e-4:
                dirn = 1 if st > prev_st else -1
                if DRV_STATE["steer_dir"] == 0:
                    DRV_STATE["steer_dir"] = dirn
                    DRV_STATE["steer_extreme"] = prev_st
                elif dirn != DRV_STATE["steer_dir"]:
                    # ~0.01 of full lock ≈ a few degrees of wheel angle.
                    # フルロックの約0.01 ≈ ハンドル角にして数度。
                    if abs(prev_st - DRV_STATE["steer_extreme"]) >= 0.01:
                        DRV_STATE["reversal_times"].append(now)
                    DRV_STATE["steer_dir"] = dirn
                    DRV_STATE["steer_extreme"] = prev_st
            DRV_STATE["steer_prev"] = st
            srr = float(len([ts for ts in DRV_STATE["reversal_times"]
                             if now - ts <= 60.0]))

            # Jerk RMS (rolling window). / ジャークRMS（ローリング窓）。
            DRV_STATE["jerk_hist"].append(jerk)
            jh = DRV_STATE["jerk_hist"]
            jerk_rms = math.sqrt(sum(j * j for j in jh) / len(jh)) if jh else 0.0

            # Speed compliance: % of moving time at/below the limit (+3 km/h).
            # 速度遵守率：走行時間のうち制限速度（+3 km/h）以下だった割合（%）。
            last_lt = DRV_STATE["last_loop_t"]
            DRV_STATE["last_loop_t"] = now
            if (last_lt is not None and now - last_lt < 1.0
                    and speed > 1.0 and speed_limit > 0):
                DRV_STATE["moving_time"] += now - last_lt
                if speed > speed_limit + 3.0:
                    DRV_STATE["speeding_time"] += now - last_lt
            if DRV_STATE["moving_time"] > 1.0:
                speed_compliance = 100.0 * (
                    1.0 - DRV_STATE["speeding_time"] / DRV_STATE["moving_time"])
            else:
                speed_compliance = 100.0

            # Weather. / 天候。
            try:
                w = world.get_weather()
                weather = {
                    "cloudiness": float(w.cloudiness),
                    "precipitation": float(w.precipitation),
                    "precipitation_deposits": float(w.precipitation_deposits),
                    "wind_intensity": float(w.wind_intensity),
                    "sun_azimuth": float(w.sun_azimuth_angle),
                    "sun_altitude": float(w.sun_altitude_angle),
                    "fog_density": float(getattr(w, "fog_density", 0.0)),
                    "wetness": float(getattr(w, "wetness", 0.0)),
                }
            except Exception:
                weather = dict(STATE["weather"])

            # Nearby vehicles / walkers + front obstacle / TTC.
            # 周辺車両・歩行者・前方障害物・TTC。
            #
            # world.get_actors() is an RPC that rebuilds a Python object for
            # every actor in the town; with 60 vehicles + 15 walkers that was
            # ~750 constructions a second at this loop's 10 Hz. The roster
            # only changes when traffic spawns or despawns, while the actor
            # objects themselves keep reading live poses out of the current
            # world snapshot -- so rescan at 1 Hz and reuse the handles.
            # world.get_actors() は町の全アクター分の Python オブジェクトを
            # 作り直すRPCで、車60台＋歩行者15人ではこの10Hzループで毎秒
            # 約750個を生成していた。名簿が変わるのは交通の出入り時だけで、
            # アクターオブジェクト自体は最新のワールドスナップショットから
            # 姿勢を読むため、走査は1Hzにして参照を使い回す。
            if actor_cache is None or now >= next_actor_scan:
                next_actor_scan = now + 1.0
                all_actors = world.get_actors()
                actor_cache = (
                    all_actors,
                    [a for a in all_actors.filter("vehicle.*")
                     if a.id != hero.id]
                    + list(all_actors.filter("walker.pedestrian.*")))
            all_actors, moving_actors = actor_cache

            nearby_list = []
            for a in moving_actors:
                try:
                    al = a.get_location()
                except RuntimeError:
                    # Despawned since the last scan / 前回の走査以降に消滅
                    continue
                dx, dy = al.x - t.location.x, al.y - t.location.y
                if abs(dx) > 80 or abs(dy) > 80:
                    continue
                d = math.hypot(dx, dy)
                if d > 80.0:
                    continue
                av = a.get_velocity()
                # project into hero body frame for mini-map / ミニマップ用にヒーロー車体座標系へ射影
                bx = dx * fx + dy * fy       # forward / 前方
                by = dx * rx + dy * ry       # right / 右方
                try:
                    ayaw = a.get_transform().rotation.yaw
                except Exception:
                    ayaw = t.rotation.yaw
                rel_yaw = ((ayaw - t.rotation.yaw + 540.0) % 360.0) - 180.0
                nearby_list.append({
                    "fx": round(bx, 2),
                    "rx": round(by, 2),
                    "d": round(d, 1),
                    "yaw": round(rel_yaw, 1),
                    "kmh": round(3.6 * math.hypot(av.x, av.y), 1),
                    "kind": _classify_actor(a),
                })
            nearby_list.sort(key=lambda v: v["d"])

            # All sensors attached to the hero (this script or any other).
            # ヒーロー車に取り付けられた全センサー（本スクリプト・他スクリプト問わず）。
            hero_sensors = []
            try:
                for a in all_actors.filter("sensor.*"):
                    par = a.parent
                    if par is None or par.id != hero.id:
                        continue
                    sl = a.get_transform().location
                    sdx, sdy = sl.x - t.location.x, sl.y - t.location.y
                    hero_sensors.append({
                        "type": a.type_id,
                        "f": round(sdx * fx + sdy * fy, 2),
                        "r": round(sdx * rx + sdy * ry, 2),
                    })
                hero_sensors.sort(key=lambda x: (-x["f"], x["r"], x["type"]))
            except Exception:
                pass

            # Ultrasonic reading is stale after 0.6 s without an event.
            # 超音波の測定値はイベントが0.6秒無ければ無効とする。
            ultra = OBST_DATA["d"] if (time.time() - OBST_DATA["t"]) < 0.6 \
                else -1.0

            try:
                ext = hero.bounding_box.extent
                bb = {"x": round(max(float(ext.x), 0.5), 2),
                      "y": round(max(float(ext.y), 0.3), 2)}
            except Exception:
                bb = dict(STATE["bb"])

            front_d, ttc, _fa = _front_obstacle(hero, moving_actors)
            # Time headway: gap to lead vehicle divided by own speed.
            # 車間時間（THW）：先行車との車間距離を自車速度で割った値。
            thw = (front_d / speed_ms) if (0.0 < front_d < 60.0
                                           and speed_ms > 0.5) else -1.0

            # Stability proxy: penalize high slip, high lat accel, low speed irrelevant.
            # 安定性の代理指標：大きなスリップや横加速度を減点。低速時は実質無関係。
            stability = max(0.0, 1.0 - min(1.0,
                abs(slip) / 20.0 + abs(lat_accel) / 8.0))
            abs_active = c.brake > 0.6 and abs(lat_accel) > 3.0
            tcs_active = c.throttle > 0.7 and speed_ms < 2.0 and lon_accel < 1.0

            # Vehicle physical properties. / 車両の物理特性。
            try:
                phys = hero.get_physics_control()
                mass_kg = float(phys.mass)
                num_wheels = len(phys.wheels)
            except Exception:
                mass_kg, num_wheels = 0.0, 4
            try:
                sim_time = float(world.get_snapshot().timestamp.elapsed_seconds)
            except Exception:
                sim_time = 0.0

            recent = [i for ts, i in COLL_EVENTS if now - ts < 10.0]
            peak = max(recent) if recent else 0.0

            events_out = [
                {"t": time.strftime("%H:%M:%S", time.localtime(ts)),
                 "type": et, "msg": msg}
                for ts, et, msg in list(EVENTS)[-25:]][::-1]

            with STATE_LOCK:
                STATE.update({
                    "connected": True,
                    "hero_found": True,
                    "map": map_name,
                    "speed_kmh": speed,
                    "speed_limit_kmh": speed_limit,
                    "speed_delta_kmh": speed - speed_limit if speed_limit > 0 else 0.0,
                    "throttle": c.throttle,
                    "brake": c.brake,
                    "steer": c.steer,
                    "reverse": c.reverse,
                    "hand_brake": c.hand_brake,
                    "gear": c.gear,
                    "location": {"x": t.location.x,
                                 "y": t.location.y,
                                 "z": t.location.z},
                    "heading_deg": t.rotation.yaw,
                    "compass_deg": IMU_DATA["compass"],
                    "gnss": dict(GNSS_DATA),
                    "imu": {k: IMU_DATA[k] for k in
                            ("ax", "ay", "az", "gx", "gy", "gz")},
                    "lat_accel": lat_accel,
                    "lon_accel": lon_accel,
                    "vert_accel": vert_accel,
                    "yaw_rate_deg": math.degrees(yaw_rate),
                    "jerk": jerk,
                    "g_force": g_force,
                    "slip_angle_deg": slip,
                    "curvature": curvature,
                    "traffic_light": tl_state,
                    "at_traffic_light": at_tl,
                    "front_distance": front_d if front_d < 60.0 else -1.0,
                    "ttc": ttc,
                    "thw": thw,
                    "sdlp": sdlp,
                    "steer_reversal_rate": srr,
                    "lane_invasions": DRV_STATE["lane_invasions"],
                    "red_light_violations": DRV_STATE["red_light_violations"],
                    "speed_compliance": speed_compliance,
                    "jerk_rms": jerk_rms,
                    "events": events_out,
                    "log_file": LOG["path"],
                    "hard_brake_count": DRV_STATE["hard_brake_count"],
                    "collision_count": DRV_STATE["collision_count"],
                    "on_road": on_road,
                    "lane_width": lane_width,
                    "lane_offset": lane_offset,
                    "lane_id": lane_id,
                    "road_id": road_id,
                    "at_junction": at_junction,
                    "trip_km": trip_km,
                    "trip_sec": trip_sec,
                    "avg_kmh": avg_kmh,
                    "collision_peak": peak,
                    "collision_recent": recent[-60:],
                    "collision_hits": [
                        {"age": round(now - h["t"], 1), "f": h["f"],
                         "r": h["r"], "i": round(h["i"]), "o": h["o"]}
                        for h in list(COLL_HITS) if now - h["t"] < 120.0],
                    # Count of what the minimap actually shows: actors
                    # within 80 m. The chip is labelled "Nearby" /
                    # 「周辺車両」, so the town-wide total would be
                    # both useless and near-constant.
                    # ミニマップに実際に表示される範囲（80m以内）の
                    # 台数。チップの表記は "Nearby"／「周辺車両」で
                    # あり、町全体の総数ではほぼ一定で意味を成さない。
                    "nearby_vehicles": len(nearby_list),
                    "nearby_list": nearby_list[:50],
                    "sensors": hero_sensors[:24],
                    "ultrasonic_m": round(ultra, 2),
                    "bb": bb,
                    "turn_signal": SIGNAL["side"],
                    "trail": list(TRAIL),
                    "weather": weather,
                    "abs_active": abs_active,
                    "tcs_active": tcs_active,
                    "stability": stability,
                    "vehicle_type": hero.type_id,
                    "num_wheels": num_wheels,
                    "mass_kg": mass_kg,
                    "sim_fps": sim_fps,
                    "sim_time_sec": sim_time,
                    "timestamp": now,
                })

            if LOG["writer"] is not None:
                try:
                    LOG["writer"].writerow([
                        round(now, 3), round(sim_time, 3),
                        round(t.location.x, 2), round(t.location.y, 2),
                        round(t.location.z, 2),
                        round(speed, 2), round(speed_limit, 1),
                        round(c.throttle, 3), round(c.brake, 3),
                        round(c.steer, 4), c.gear,
                        round(lon_accel, 3), round(lat_accel, 3),
                        round(jerk, 3),
                        round(math.degrees(yaw_rate), 2), round(slip, 2),
                        round(lane_offset, 3), round(lane_width, 2),
                        int(on_road),
                        round(front_d, 2) if front_d < 60.0 else -1,
                        round(ttc, 2), round(thw, 2), round(sdlp, 3), srr,
                        DRV_STATE["hard_brake_count"],
                        DRV_STATE["lane_invasions"],
                        DRV_STATE["red_light_violations"],
                        round(speed_compliance, 1), round(peak, 1),
                        DRV_STATE["collision_count"]])
                    if now - LOG["last_flush"] > 2.0:
                        LOG["fh"].flush()
                        LOG["last_flush"] = now
                except Exception:
                    pass
            _write_summary(now)
        except Exception as exc:
            with STATE_LOCK:
                STATE["connected"] = False
                STATE["hero_found"] = False
            print("[dashboard] telemetry worker error: %s" % exc, file=sys.stderr)
            time.sleep(1.0)
            # If the simulator went away (a GPU crash takes CarlaUE4 with it),
            # retrying straight into get_world() would burn the client timeout
            # with the GIL held, once per second, and the page would go dead
            # instead of showing "disconnected". Wait on the socket instead,
            # and rebuild the client so it reconnects cleanly.
            # シミュレータが消えた場合（GPUクラッシュは CarlaUE4 ごと落とす）、
            # そのまま get_world() を再試行すると毎秒クライアントタイムアウト
            # 分だけ GIL を保持し続け、「切断」表示どころかページ自体が無反応に
            # なる。代わりに socket で待ち、クライアントを作り直して確実に
            # 再接続する。
            if not _rpc_port_open(host, port):
                _wait_for_server(host, port)
                client = carla.Client(host, port)
                client.set_timeout(5.0)
                carla_map = None
                current_hero_id = None
                sensors = []
            continue

        time.sleep(0.1)


app = Flask(__name__)


@app.route("/telemetry")
def telemetry():
    with STATE_LOCK:
        return jsonify(STATE)


# Delivery drones: delivery_sim.py writes its live state to files in this
# folder and reads wind commands appended to commands.jsonl (delivery/live.py).
# 配送ドローン：delivery_sim.py がこのフォルダに状態を書き出し、
# commands.jsonl に追記された風コマンドを読む（delivery/live.py）。
DELIVERY_DIR = delivery_live.DEFAULT_DIR
DELIVERY_STALE_S = 5.0      # no update for this long: the sim isn't running
WIND_LIMITS = {"speed": (0.0, 40.0), "duration": (0.5, 60.0), "value": (0.0, 20.0)}


@app.route("/delivery/state")
def delivery_state():
    s = delivery_live.read_json(os.path.join(DELIVERY_DIR, delivery_live.STATE))
    if not s:
        return jsonify({"available": False, "running": False})
    s["available"] = True
    s["age"] = round(time.time() - s.get("wall_time", 0.0), 1)
    if s["age"] > DELIVERY_STALE_S:
        s["running"] = False
    return jsonify(s)


@app.route("/delivery/map")
def delivery_map():
    m = delivery_live.read_json(os.path.join(DELIVERY_DIR, delivery_live.MAP))
    if not m:
        return jsonify({"error": "no map yet"}), 404
    return jsonify(m)


@app.route("/delivery/zones", methods=["GET", "POST"])
def delivery_zones_route():
    """Pod delivery zones and no-fly zones of a map (data/delivery_zones.json;
    delivery_sim.py picks changes up within a second). GET ?map=<name> (default:
    the delivery sim's map); POST {"map", "pods": {"<pod>": rect}, "nofly": [rect],
    "meet": {"<pod>": [{"x", "y"}]}, "dist": {"<hub>": [{"x", "y"}]},
    "charge": [{"x", "y"[, "yaw"]}]} (LastMile charging-station spots).
    配送ゾーン・飛行禁止区域・充電ステーション地点の取得と保存。"""
    if request.method == "POST":
        d = request.get_json(force=True, silent=True) or {}
        name = delivery_zones.map_key(d.get("map"))
        if not name:
            return jsonify({"ok": False, "error": "no map"}), 400
        saved = delivery_zones.load(name)
        entry = delivery_zones.save(name, {"pods": d.get("pods"), "nofly": d.get("nofly"),
                                           "meet": d.get("meet"),
                                           # an editor that sends no "dist" / "charge"
                                           # keeps the saved ones / 送られない項目は保存済みを維持
                                           "dist": d["dist"] if "dist" in d else saved.get("dist"),
                                           "charge": d["charge"] if "charge" in d else
                                           saved.get("charge")})
        return jsonify(dict(entry, ok=True, map=name))
    name = request.args.get("map")
    if not name:
        s = delivery_live.read_json(os.path.join(DELIVERY_DIR, delivery_live.STATE)) or {}
        name = s.get("map")
    name = delivery_zones.map_key(name)
    if not name:
        return jsonify({"ok": False, "error": "no map", "pods": {}, "nofly": []})
    return jsonify(dict(delivery_zones.load(name), ok=True, map=name))


@app.route("/delivery/pod", methods=["POST"])
def delivery_pod():
    """Queue a LastMile pod command for delivery_sim: the shell view, or the
    showcase (take a pod over, scrub / play its animation, release it)."""
    d = request.get_json(force=True, silent=True) or {}
    cmd = d.get("cmd")
    clean = {"cmd": cmd}
    try:
        if cmd == "shell":
            if d.get("mode") not in ("solid", "cutaway", "xray", "frame"):
                raise ValueError("unknown shell mode")
            clean["mode"] = d["mode"]
        elif cmd == "pod_takeover":
            clean.update(van=int(d["van"]), t=max(float(d.get("t", 0.0)), 0.0),
                         speed=min(max(float(d.get("speed", 1.0)), 0.1), 4.0))
        elif cmd == "pod_time":
            clean["t"] = max(float(d["t"]), 0.0)
        elif cmd == "tooltips":
            clean["on"] = bool(d.get("on"))
        elif cmd == "pod_play":
            clean.update(playing=bool(d.get("playing", True)),
                         speed=min(max(float(d.get("speed", 1.0)), 0.1), 4.0))
        elif cmd != "pod_release":
            raise ValueError("unknown command")
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    delivery_live.send_command(DELIVERY_DIR, clean)
    return jsonify({"ok": True, "sent": clean})


@app.route("/delivery/wind", methods=["POST"])
def delivery_wind():
    """Queue a wind command for delivery_sim: gust, wind, calm or gust_rate."""
    d = request.get_json(force=True, silent=True) or {}
    cmd = d.get("cmd")
    if cmd not in ("gust", "wind", "calm", "gust_rate"):
        return jsonify({"ok": False, "error": "unknown command"}), 400
    clean = {"cmd": cmd}
    try:
        for k, (lo, hi) in WIND_LIMITS.items():
            if d.get(k) is not None:
                clean[k] = min(max(float(d[k]), lo), hi)
        if d.get("dir") is not None:
            clean["dir"] = float(d["dir"]) % 360.0
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "bad number"}), 400
    delivery_live.send_command(DELIVERY_DIR, clean)
    return jsonify({"ok": True, "sent": clean})


@app.route("/delivery/charge", methods=["POST"])
def delivery_charge():
    """Send one delivery vehicle (LMP-4 pod or LastMile Hub) to a charging
    station: delivery_sim marks it charge_asap and it goes the next time it
    is free. Body {"van": 1..99}. / 配送車両を充電へ向かわせる"""
    d = request.get_json(force=True, silent=True)
    van = d.get("van") if isinstance(d, dict) else None
    if isinstance(van, bool) or not isinstance(van, int) or not 1 <= van <= 99:
        return jsonify({"ok": False, "error": "van must be an integer 1..99"}), 400
    clean = {"cmd": "charge", "van": van}
    delivery_live.send_command(DELIVERY_DIR, clean)
    return jsonify({"ok": True, "sent": clean})


# Drones page: "Open CARLA view" = a native CARLA window for one drone
# (drone_window.py: a chase camera following the drone body + its onboard
# cameras), started detached by drone_windows.py. Nothing is streamed here.
# GET  /drones/window                     {windows: [{drone, pid, running}]}
# POST /drones/window {drone, action}     action "open" (default; a second open
#                                         brings it forward or replaces it) | "close"
# ドローンページ：1 機の CARLA ウィンドウ（drone_window.py）を開く・閉じる
DRONE_WIN = {"host": "127.0.0.1", "port": 2000}


@app.route("/drones/window", methods=["GET", "POST"])
def drones_window_route():
    if request.method == "POST":
        d = request.get_json(force=True, silent=True) or {}
        try:
            drone = int(d.get("drone"))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "no drone"}), 400
        try:
            if d.get("action") == "close":
                act, info = drone_windows.close_window(drone), {}
            else:
                act, info = drone_windows.open_window(drone, DRONE_WIN["host"], DRONE_WIN["port"])
        except OSError as e:
            return jsonify({"ok": False, "error": str(e),
                            "windows": drone_windows.list_windows()}), 500
        ok = act != "too_many"
        return jsonify(dict(info, ok=ok, action=act, windows=drone_windows.list_windows(),
                            **({} if ok else {"error": "at most %d CARLA views at once"
                                              % drone_windows.MAX_WINDOWS})))
    return jsonify({"ok": True, "windows": drone_windows.list_windows()})


# Mini Pods page: "Open CARLA view" = a native CARLA window for one LastMile
# mini pod (minipod_window.py: a chase camera behind the pod body + its six
# onboard cameras; on the hub vehicle while docked), started detached by
# minipod_windows.py. Same host / port as the drone windows.
# GET  /minipods/window                {windows: [{id, pid, running}]}
# POST /minipods/window {id, action}   action "open" (default; a second open
#                                      brings it forward or replaces it) | "close"
# ミニポッドページ：1 台の CARLA ウィンドウ（minipod_window.py）を開く・閉じる
@app.route("/minipods/window", methods=["GET", "POST"])
def minipods_window_route():
    if request.method == "POST":
        d = request.get_json(force=True, silent=True) or {}
        try:
            pod = int(d.get("id"))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "no mini pod id"}), 400
        try:
            if d.get("action") == "close":
                act, info = minipod_windows.close_window(pod), {}
            else:
                act, info = minipod_windows.open_window(pod, DRONE_WIN["host"], DRONE_WIN["port"])
        except OSError as e:
            return jsonify({"ok": False, "error": str(e),
                            "windows": minipod_windows.list_windows()}), 500
        ok = act != "too_many"
        return jsonify(dict(info, ok=ok, action=act, windows=minipod_windows.list_windows(),
                            **({} if ok else {"error": "at most %d mini pod CARLA views at once"
                                              % minipod_windows.MAX_WINDOWS})))
    return jsonify({"ok": True, "windows": minipod_windows.list_windows()})


# Pedal tuning shared with the G923 control script, which watches this
# file's mtime and applies changes live while driving.
# G923 制御スクリプトと共有するペダル調整設定。同スクリプトがこの
# ファイルの更新時刻を監視し、運転中に変更をライブ適用する。
TUNING_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "pedal_tuning.json")
TUNING_DEFAULTS = {"throttle_max": 0.8, "throttle_curve": 1.8,
                   "brake_max": 1.0, "brake_curve": 2.2}
TUNING_LIMITS = {"throttle_max": (0.1, 1.0), "throttle_curve": (0.5, 3.0),
                 "brake_max": (0.1, 1.0), "brake_curve": (0.5, 3.0)}


@app.route("/tuning", methods=["GET", "POST"])
def tuning():
    data = dict(TUNING_DEFAULTS)
    try:
        with open(TUNING_PATH) as fh:
            saved = json.load(fh)
        data.update({k: float(saved[k]) for k in TUNING_DEFAULTS
                     if k in saved})
    except Exception:
        pass
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        for k, (lo, hi) in TUNING_LIMITS.items():
            if k in body:
                try:
                    data[k] = max(lo, min(hi, float(body[k])))
                except (TypeError, ValueError):
                    pass
        try:
            tmp = TUNING_PATH + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(data, fh, indent=1)
            os.replace(tmp, TUNING_PATH)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    return jsonify(data)


# Force-feedback settings shared with carla_ffb.py, which re-reads
# ffb_tuning.json twice a second (schema and defaults in ffb_settings.py).
# carla_ffb.py と共有する FFB 設定（ffb_tuning.json を毎秒 2 回読み直す）。
@app.route("/ffb", methods=["GET", "POST"])
def ffb():
    values = ffb_settings.load()
    error = None
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        if body.get("preset"):
            chosen = ffb_settings.preset(str(body["preset"]))
            if chosen is not None:
                values = chosen
        for k, v in (body.get("values") or {}).items():
            if k in values:
                try:
                    values[k] = ffb_settings.clamp_value(k, v)
                except (TypeError, ValueError):
                    pass
        try:
            values = ffb_settings.save(values)
            if body.get("save_preset") is not None:
                try:
                    ffb_settings.save_preset(body["save_preset"], values)
                except ValueError as exc:
                    error = str(exc)
            if body.get("delete_preset"):
                ffb_settings.delete_preset(str(body["delete_preset"]))
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    status = ffb_settings.read_status()
    if status is not None:
        status["age"] = round(time.time() - status.get("wall_time", 0), 1)
    return jsonify({
        "values": values, "defaults": ffb_settings.DEFAULTS,
        "params": [{"key": p[0], "min": p[2], "max": p[3], "step": p[4], "group": p[5],
                    "en": p[6], "ja": p[7], "help": p[8]} for p in ffb_settings.PARAMS],
        "groups": [{"key": g[0], "en": g[1], "ja": g[2]} for g in ffb_settings.GROUPS],
        "presets": list(ffb_settings.PRESETS),
        "user_presets": sorted(ffb_settings.user_presets()), "error": error,
        "active": next((n for n in list(ffb_settings.PRESETS) + sorted(ffb_settings.user_presets())
                        if ffb_settings.preset(n) == values), None),
        "status": status})


# Vehicle performance (brakes, power, top speed) shared with carla_ffb.py; the
# thread below applies vehicle_tuning.json to the cars.
# 車両性能の設定。下のスレッドが vehicle_tuning.json を各車に適用する。
@app.route("/vehicle_tuning", methods=["GET", "POST"])
def vehicle_tuning_route():
    values = vehicle_tuning.load()
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        if body.get("reset"):
            values = dict(vehicle_tuning.DEFAULTS)
        if body.get("neutral"):
            values.update(vehicle_tuning.NEUTRAL)
        for k, v in (body.get("values") or {}).items():
            if k in values:
                try:
                    values[k] = vehicle_tuning.clamp_value(k, v)
                except (TypeError, ValueError):
                    pass
        try:
            values = vehicle_tuning.save(values)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    return jsonify({
        "values": values, "defaults": vehicle_tuning.DEFAULTS,
        "params": [{"key": p[0], "min": p[2], "max": p[3], "step": p[4], "group": p[5],
                    "en": p[6], "ja": p[7], "help": p[8]} for p in vehicle_tuning.PARAMS],
        "groups": [{"key": g[0], "en": g[1], "ja": g[2]} for g in vehicle_tuning.GROUPS],
        "status": vehicle_tuning.read_status()})


# Tree seasons and wind (seasons.py): the choice lives in season.json; the
# worker below applies it, and again whenever the map is (re)loaded.
# 樹木の季節と風（seasons.py）。選択は season.json、下のワーカーが適用する。
SEASON_STATUS = {"applied": None, "result": None, "pending": True}


@app.route("/season", methods=["GET", "POST"])
def season_route():
    s = seasons.load_state()
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        if body.get("season") in seasons.SEASONS + ("auto",):
            s["season"] = body["season"]
        if body.get("wind") is not None:
            try:
                s["wind"] = max(0.0, min(100.0, float(body["wind"])))
            except (TypeError, ValueError):
                pass
        try:
            seasons.save_state(s)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
        SEASON_STATUS["pending"] = True
    return jsonify({"season": s["season"], "wind": s["wind"],
                    "by_date": seasons.season_for(),
                    "seasons": [{"key": k, "en": seasons.LABELS[k][0], "ja": seasons.LABELS[k][1]}
                                for k in ("auto",) + seasons.SEASONS],
                    "result": SEASON_STATUS["result"], "pending": SEASON_STATUS["pending"]})


def season_worker(host, port):
    """Apply season.json when it changes, when the map (re)loads (a new world
    id: the swapped texture is gone) and at midnight in 'auto'.
    season.json の変更時・マップ再読込時・日付変更時に適用する。"""
    last = None
    client = None
    while True:
        time.sleep(1.0)
        try:
            if client is None:
                # no CARLA calls while its port is closed: a failed connect
                # holds the GIL for the whole timeout and freezes every page
                # CARLA が落ちている間は呼ばない（GIL を握りページ全体が止まる）
                if not _rpc_port_open(host, port):
                    time.sleep(4.0)
                    continue
                client = carla.Client(host, port)
                client.set_timeout(20.0)
            world = client.get_world()
            s = seasons.load_state()
            eff = seasons.season_for() if s["season"] == "auto" else s["season"]
            key = (world.id, s["season"], eff, round(float(s["wind"]), 1))
            if key == last and not SEASON_STATUS["pending"]:
                continue
            if last is not None and key[0] != last[0]:
                time.sleep(3.0)                   # let a fresh map settle / 読込直後は少し待つ
            r = seasons.apply(world, s["season"], s["wind"])
            r["at"] = time.strftime("%H:%M:%S")
            SEASON_STATUS.update({"result": r, "pending": False})
            last = key
        except Exception as exc:
            SEASON_STATUS["result"] = {"ok": False, "error": str(exc), "at": time.strftime("%H:%M:%S")}
            client = None
            time.sleep(5.0)


def vehicle_tuning_worker(host, port):
    """Keep every car's physics in line with vehicle_tuning.json.

    Applies when the settings change (once they've been still for 0.8 s, so
    dragging a slider doesn't re-apply every step) and to each new car.
    Pods in a scripted move (physics off) wait until they drive again. The
    hero's tyre grip comes from the FFB settings while carla_ffb.py runs.
    A car's state is unknown when first seen (it may carry an older tuning
    from before a dashboard restart), so every car gets one apply then.
    設定変更時（0.8 秒変化が無くなってから）と新しい車に適用する。物理オフの
    ポッドは走行再開まで待つ。自車のタイヤグリップは FFB 設定に従う。
    """
    applied = {}            # actor id -> what it has / 適用済みの内容
    shape = {}              # type id -> physics control (for gears, wheels)
    client = world = None
    mtime, changed_at = None, 0.0
    while True:
        try:
            if world is None:
                if not _rpc_port_open(host, port):        # see season_worker / 同上
                    time.sleep(3.0)
                    continue
                client = carla.Client(host, port)
                client.set_timeout(5.0)
                world = client.get_world()
                applied.clear()
            world.wait_for_tick(5.0)
            try:
                m = os.path.getmtime(vehicle_tuning.TUNING_PATH)
            except OSError:
                m = None
            if m != mtime:
                mtime, changed_at = m, time.time()
            if time.time() - changed_at < 0.8:
                time.sleep(0.2)
                continue
            values = vehicle_tuning.load()
            ffb = ffb_settings.read_status()
            ffb_grip = (ffb_settings.load()["tire_friction"]
                        if ffb and time.time() - ffb.get("wall_time", 0) < 5 else None)
            state = delivery_live.read_json(
                os.path.join(DELIVERY_DIR, delivery_live.STATE)) or {}
            vans = {v.get("actor"): v for v in state.get("vans", []) if "actor" in v}
            seen, cars = set(), []
            for a in world.get_actors().filter("vehicle.*"):
                if a.attributes.get("role_name") == vehicle_tuning.PROBE_ROLE:
                    continue
                seen.add(a.id)
                is_hero = a.attributes.get("role_name") == "hero"
                sc = vehicle_tuning.scales(values, is_hero)
                grip = ffb_grip if is_hero else None
                want = (tuple(sorted(sc.items())), grip)
                van = vans.get(a.id)
                waiting = a.attributes.get("role_name") == "delivery" and \
                    (van is None or van.get("kinematic"))
                fac = vehicle_tuning.factory(world, a.type_id)
                have = applied.get(a.id)        # None: unknown / 未知
                if want != have and not waiting:
                    vehicle_tuning.apply(client, a, sc, grip)
                    applied[a.id] = have = want
                now = dict(have[0]) if have else vehicle_tuning.NEUTRAL
                if a.type_id not in shape:      # gears, wheel size: never tuned / 不変
                    shape[a.type_id] = a.get_physics_control()
                brake0, torque0 = fac["brake"][0], max(y for _, y in fac["torque_curve"])
                top0 = vehicle_tuning.top_speed_kmh(shape[a.type_id], fac["max_rpm"])
                cars.append({
                    "id": a.id, "type": a.type_id, "hero": is_hero,
                    "applied": have == want, "waiting": waiting and want != have,
                    "brake_nm": round(brake0 * now["brake_scale"]), "brake_base": round(brake0),
                    "torque_nm": round(torque0 * now["power_scale"]), "torque_base": round(torque0),
                    "top_kmh": round(top0 * now["top_speed_scale"]), "top_base": round(top0),
                    "grip": grip})
            for gone in set(applied) - seen:
                del applied[gone]
            cars.sort(key=lambda c: (not c["hero"], c["type"], c["id"]))
            vehicle_tuning.write_status({"wall_time": time.time(), "cars": cars})
            time.sleep(0.5)
        except Exception as exc:        # server gone / RPC timeout: reconnect
            print("[dashboard] vehicle tuning: %s" % exc)
            world = None
            time.sleep(3.0)


def _shutdown_worker():
    """Stop the whole simulation stack and free its ports.

    First kill the manual-control window: when launched via run_pipeline.py
    its supervisor then terminates every child cleanly (traffic, viewers,
    CarlaUE4, and this dashboard) and all ports are released. If we are
    still alive after that grace period (standalone use), kill CARLA
    directly and exit this process too.

    シミュレーション一式を停止しポートを解放する。まず運転ウィンドウを
    終了させる：run_pipeline.py 経由なら監視プロセスが全ての子（交通・
    ビューア・CarlaUE4・本ダッシュボード）をクリーンに終了させ、ポートも
    解放される。猶予時間を過ぎても生きていれば（単体起動の場合）、CARLA を
    直接終了させ、本プロセスも終了する。
    """
    time.sleep(0.3)   # let the HTTP response flush / レスポンス送信を待つ
    for pat in ("manual_control_steeringwheel", "sensor_viewer.py",
                "generate_traffic.py", "agent_drive.py"):
        try:
            subprocess.call(["pkill", "-f", pat])
        except Exception:
            pass
    # Grace period for run_pipeline's terminate_all (it will kill us too).
    # run_pipeline の terminate_all の猶予（本プロセスも終了させてくれる）。
    time.sleep(6.0)
    # Still alive -> standalone use: kill CARLA ourselves. UE4 often
    # ignores SIGTERM, so escalate to SIGKILL and verify port 2000 is
    # actually released.
    # まだ生きている＝単体起動：CARLA を自ら終了させる。UE4 は SIGTERM を
    # 無視しがちなので SIGKILL へ段階的に強め、ポート2000の解放を確認する。
    for pat in ("CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
        try:
            subprocess.call(["pkill", "-f", pat])
        except Exception:
            pass
    time.sleep(3.0)
    for pat in ("CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
        try:
            subprocess.call(["pkill", "-9", "-f", pat])
        except Exception:
            pass
    for _ in range(10):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("127.0.0.1", 2000)) != 0:
                print("[dashboard] port 2000 released")
                break
        time.sleep(0.5)
    print("[dashboard] shutdown requested from the web UI - exiting")
    os._exit(0)


# Optional windows of the pipeline (sensor viewer, drone cameras): run_pipeline
# writes pipeline_status.json about once a second; a click here writes
# pipeline_control.json, which it carries out (windows_control.py).
# パイプラインの任意ウィンドウ（センサービューア・ドローンカメラ）：状態は
# run_pipeline が約 1 秒ごとに pipeline_status.json へ書き、ここでのクリックは
# pipeline_control.json に書かれてパイプラインが実行する（windows_control.py）。
@app.route("/daycycle", methods=["GET", "POST"])
def daycycle_route():
    """Day/night cycle and dynamic weather (weather_cycle.py reads the file
    live). / 昼夜サイクルと天気の設定（weather_cycle.py が随時読む）。"""
    s = weather_cycle.load_settings()
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        for k in ("day_night", "paused", "dynamic"):
            if k in body:
                s[k] = bool(body[k])
        for k, lo, hi in (("day_minutes", 1.0, 1440.0), ("rain_chance", 0.0, 1.0),
                          ("lamp_lumens", 5000.0, 500000.0)):
            if body.get(k) is not None:
                try:
                    s[k] = max(lo, min(hi, float(body[k])))
                except (TypeError, ValueError):
                    pass
        if body.get("force") in ("auto",) + tuple(weather_cycle.STATES):
            s["force"] = body["force"]
        if body.get("lamps") in ("auto", "on", "off"):
            s["lamps"] = body["lamps"]
        hhmm = str(body.get("clock") or "")
        if len(hhmm) == 5 and hhmm[2] == ":" and hhmm.replace(":", "").isdigit():
            seq = (s.get("clock_cmd") or {}).get("seq", 0) if isinstance(s.get("clock_cmd"), dict) else 0
            s["clock_cmd"] = {"hhmm": hhmm, "seq": int(seq) + 1}
        try:
            weather_cycle.save_settings(s)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    try:
        with open(weather_cycle.STATUS_PATH) as fh:
            st = json.load(fh)
        st["live"] = time.time() - float(st.get("t", 0)) < 5.0
    except (OSError, ValueError):
        st = {"live": False}
    return jsonify({"settings": {k: v for k, v in s.items() if k != "clock_cmd"}, "status": st,
                    "states": list(weather_cycle.STATES)})


# Road logic overlay (road_overlay.py, O key in the driving window): the
# settings live in data/road_overlay.json, the helper reports its state in
# data/road_overlay_status.json. / 道路ロジック表示の設定と状態
@app.route("/road_overlay", methods=["GET", "POST"])
def road_overlay_route():
    """GET: settings, helper status, layers and legend. POST {on, spectator,
    radius, max_shapes, refresh, detail_radius, brightness, layers: {name: bool}}.
    / 設定・状態・凡例の取得と変更"""
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        try:
            road_overlay.update_control({k: body[k] for k in ("on", "spectator", "layers") + tuple(road_overlay.LIMITS)
                                         if k in body})
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    return jsonify({"settings": road_overlay.load_control(), "status": road_overlay.read_status(),
                    "layers": road_overlay.layer_info(), "legend": road_overlay.legend(),
                    "limits": road_overlay.LIMITS})


# RL training (rl_delivery/): trainer status + progress, start/stop, watch-mode
# recordings. The trainer runs as its own background process (not a pipeline
# child), so it keeps going while CARLA is stopped. / 強化学習の状態・開始/停止・ウォッチ
RL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rl_delivery")
RL_RES = os.path.join(RL_DIR, "results")
RL_PY = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                     "carla-venv", "bin", "python")
_rl_proc = {"p": None}


def _rl_trainer_alive():
    p = _rl_proc["p"]
    if p is not None and p.poll() is None:
        return True
    try:
        st = json.load(open(os.path.join(RL_RES, "train_state.json")))
        return bool(st.get("running")) and time.time() - st.get("t", 0) < 600
    except (OSError, ValueError):
        return False


def _rl_brain_running(brain):
    """Is a trainer process for this brain running? (ps, not the status file:
    the fleet trainer writes it only between rounds) / 学習プロセスの有無"""
    try:
        out = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return False
    return any("rl_delivery.train" in line and "--brain %s" % brain in line for line in out.splitlines())


@app.route("/rl/state")
def rl_state():
    """Trainer status, progress rows and checkpoints of one brain. / 学習の状態"""
    brain = request.args.get("brain", "flight")
    try:
        st = json.load(open(os.path.join(RL_RES, "train_state.json" if brain == "flight"
                                         else "train_state_%s.json" % brain)))
    except (OSError, ValueError):
        st = {}
    rows = []
    try:
        with open(os.path.join(RL_RES, "%s_progress.csv" % brain), newline="") as f:
            def num(v):
                try:
                    return float(v)
                except (TypeError, ValueError):
                    return None if v in ("", None) else v
            rows = [{k: num(v) for k, v in r.items()} for r in csv.DictReader(f)]
    except OSError:
        pass
    ck = sorted(glob.glob(os.path.join(RL_DIR, "checkpoints", "fleet_pod" if brain == "fleet" else brain,
                                       "step_*.zip")))
    ck_drone = sorted(glob.glob(os.path.join(RL_DIR, "checkpoints", "fleet_drone", "step_*.zip"))) \
        if brain == "fleet" else []
    alive = _rl_brain_running(brain)
    rule = None
    try:
        rule = json.load(open(os.path.join(RL_RES, "fleet_rule_baseline.json" if brain == "fleet"
                                           else "mission_rule_baseline.json")))
    except (OSError, ValueError):
        pass
    return jsonify({"state": st, "alive": alive, "progress": rows, "rule": rule,
                    "checkpoints": [os.path.basename(c) for c in ck],
                    "drone_checkpoints": [os.path.basename(c) for c in ck_drone]})


@app.route("/rl/train", methods=["POST"])
def rl_train():
    """{action: start|stop, brain, hours, resume}. / 学習の開始・停止"""
    body = request.get_json(silent=True) or {}
    if body.get("action") == "stop":
        b = str(body.get("brain", "flight"))
        open(os.path.join(RL_RES, "STOP" if b == "flight" else "STOP_" + os.path.basename(b)), "w").close()      # the trainer saves and exits / 保存して終了
        return jsonify({"ok": True})
    if _rl_brain_running(str(body.get("brain", "flight"))):
        return jsonify({"error": "a trainer is already running"}), 409
    cmd = [RL_PY, "-W", "ignore", "-m", "rl_delivery.train", "--brain", str(body.get("brain", "flight")),
           "--hours", str(float(body.get("hours", 3)))]
    if body.get("resume"):
        cmd.append("--resume")
    os.makedirs(RL_RES, exist_ok=True)
    log = open(os.path.join(RL_RES, "train_%s.log" % body.get("brain", "flight")), "a")
    _rl_proc["p"] = subprocess.Popen(cmd, cwd=os.path.dirname(RL_DIR), stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
    return jsonify({"ok": True, "pid": _rl_proc["p"].pid})


@app.route("/rl/watch", methods=["POST"])
def rl_watch():
    """Record one episode {brain, checkpoint (file|latest|pid), wind, seed} and
    return it (a few seconds). / 1 エピソードを記録して返す"""
    body = request.get_json(silent=True) or {}
    out = os.path.join(RL_RES, "watch", "_last.json")
    ck = str(body.get("checkpoint", "latest"))
    brain = str(body.get("brain", "flight"))
    if ck not in ("latest", "pid", "rule", "scripted"):
        ck = os.path.join(RL_DIR, "checkpoints", "fleet_pod" if brain == "fleet" else brain, os.path.basename(ck))
    dck = str(body.get("drone_checkpoint", "latest"))
    if dck not in ("latest", "rule"):
        dck = os.path.join(RL_DIR, "checkpoints", "fleet_drone", os.path.basename(dck))
    cmd = [RL_PY, "-W", "ignore", "-m", "rl_delivery.watch", "--brain", str(body.get("brain", "flight")),
           "--checkpoint", ck, "--wind", str(float(body.get("wind", 1.0))),
           "--seed", str(int(body.get("seed", 7))), "--out", out, "--drone-checkpoint", dck]
    try:
        r = subprocess.run(cmd, cwd=os.path.dirname(RL_DIR), capture_output=True, text=True, timeout=180)
    except subprocess.TimeoutExpired:
        return jsonify({"error": "timeout"}), 504
    if r.returncode != 0:
        return jsonify({"error": (r.stderr or r.stdout)[-400:]}), 500
    with open(out) as f:
        return Response(f.read(), mimetype="application/json")


# Timetabled buses (bus_schedule.py, run by generate_traffic on OitaStation):
# its status file, read-only. / 時刻表バスの状態（読み取りのみ）
BUS_STATUS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "bus_status.json")
BUS_CREDIT = ("出典：大分県オープンデータカタログ（BODIK）「大分バスGTFSデータ」「大分交通GTFSデータ」"
              "「高速バスGTFSデータ」（CC BY 4.0、大分県）を加工して作成; road names © OpenStreetMap contributors")


@app.route("/buses")
def buses_route():
    """Buses on the map and the next departures per Oita Station platform;
    live = the file is fresh (written every 2 s). / 路線バスの状態"""
    try:
        with open(BUS_STATUS, encoding="utf-8") as fh:
            st = json.load(fh)
        st["live"] = time.time() - float(st.get("t", 0)) < 10.0
    except (OSError, ValueError):
        st = {"live": False}
    st.setdefault("credit", BUS_CREDIT)
    return jsonify(st)


@app.route("/windows", methods=["GET", "POST"])
def windows_route():
    """GET: every optional window with running / pid / available. POST
    {"window": "drone_cams", "action": "open"|"close"} queues the request
    (409 when no pipeline is running). / 任意ウィンドウの状態取得・開閉依頼。"""
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        st = windows_control.read_status()
        if not st["live"]:
            st["error"] = "run_pipeline.py is not running"
            return jsonify(st), 409
        try:
            windows_control.write_request(body.get("window"), body.get("action"))
        except ValueError as exc:
            st["error"] = str(exc)
            return jsonify(st), 400
        except OSError as exc:
            st["error"] = str(exc)
            return jsonify(st), 500
        st["queued"] = {"window": body.get("window"), "action": body.get("action")}
        return jsonify(st)
    return jsonify(windows_control.read_status())


@app.route("/shutdown", methods=["POST"])
def shutdown():
    threading.Thread(target=_shutdown_worker, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/signal", methods=["GET", "POST"])
def signal():
    """Toggle / read the turn signal. POST {"side": "left"|"right"} toggles
    that side (same side again switches it off).

    ウインカーの切替・取得。POST {"side": "left"|"right"} でその側を
    トグルする（同じ側をもう一度送ると消灯）。
    """
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        side = body.get("side")
        if side in ("left", "right", "hazard"):
            _save_signal("off" if SIGNAL["side"] == side else side)
        elif side == "off":
            _save_signal("off")
    return jsonify({"side": SIGNAL["side"]})


# Key / wheel-button bindings shared with the driving window (keybinds.py):
# keybinds.json holds the bindings, g923_buttons.json the wheel's button names,
# wheel_last.json the last wheel press (for press-to-bind), keybind_learn.json
# tells the driving window to ignore wheel presses while we are listening.
# 運転ウインドウと共有するキー・ホイール割り当て（keybinds.py）。
def _keybinds_payload(bindings=None):
    import importlib
    importlib.reload(keybinds)
    bindings = bindings if bindings is not None else keybinds.load()
    lab = keybinds.load_labels()
    return {
        "actions": keybinds.ACTIONS,
        "groups": [{"key": g[0], "en": g[1], "ja": g[2]} for g in keybinds.GROUPS],
        "bindings": bindings,
        "defaults": keybinds.defaults(),
        "conflicts": keybinds.conflicts(bindings),
        "labels": lab["labels"],
        "confirmed": lab["confirmed"],
        "label_ja": keybinds.LABEL_JA,
        "stock_labels": keybinds.DEFAULT_LABELS,
        "stock_order": list(keybinds.DEFAULT_LABELS.values()),
        "known_keys": sorted(keybinds.KNOWN_KEYS),
    }


@app.route("/keybinds", methods=["GET", "POST"])
def keybinds_route():
    """GET: actions, bindings, defaults, conflicts, wheel names.
    POST {"bindings": {...}} saves (cleaned); {"reset": true} restores the
    defaults. / 割り当ての取得・保存・既定値に戻す。"""
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        try:
            if body.get("reset"):
                saved = keybinds.save(keybinds.defaults())
            else:
                saved = keybinds.save(keybinds.clean(body.get("bindings") or {}))
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
        return jsonify(_keybinds_payload(saved))
    return jsonify(_keybinds_payload())


@app.route("/keybinds/wheel")
def keybinds_wheel():
    """The last wheel button / D-pad press reported by the driving window,
    with its name. / 運転ウインドウが報告した最後のホイール入力。"""
    last = keybinds.read_wheel_last()
    if last is None:
        return jsonify({"last": None, "now": time.time()})
    labels = keybinds.load_labels(create=False)["labels"]
    last["label"] = labels.get(last["spec"])     # None = unknown / 未登録
    return jsonify({"last": last, "now": time.time(),
                    "learning": keybinds.learning_until() > time.time()})


@app.route("/keybinds/learn", methods=["POST"])
def keybinds_learn():
    """POST {"seconds": 30} asks the driving window to only report wheel
    presses for that long; {"seconds": 0} ends it. / 識別モードの開始・終了。"""
    body = request.get_json(silent=True) or {}
    try:
        secs = max(0.0, min(120.0, float(body.get("seconds", 0))))
        until = keybinds.set_learning(secs)
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500
    return jsonify({"until": until, "now": time.time()})


@app.route("/keybinds/label", methods=["POST"])
def keybinds_label():
    """Name a wheel input: {"spec": "button:3", "label": "Triangle"};
    {"spec": ..., "confirm": true} marks the current name right;
    {"reset": true} restores the stock G923 names. / ボタン名の修正・確認。"""
    body = request.get_json(silent=True) or {}
    try:
        if body.get("reset"):
            keybinds.save_labels({"labels": dict(keybinds.DEFAULT_LABELS), "confirmed": []})
        elif body.get("confirm"):
            keybinds.confirm_label(body.get("spec"))
        else:
            keybinds.set_label(body.get("spec"), body.get("label", ""))
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500
    return jsonify(_keybinds_payload())


@app.route("/sessions")
def sessions():
    """List saved sessions: [{date, time, ...summary}, ...] newest first.

    保存済みセッションの一覧を返す：[{date, time, ...summary}, ...]（新しい順）。
    """
    root = LOG.get("root") or "logs"
    out = []
    try:
        for day in sorted(os.listdir(root), reverse=True):
            day_dir = os.path.join(root, day)
            if not os.path.isdir(day_dir):
                continue
            for tod in sorted(os.listdir(day_dir), reverse=True):
                sess_dir = os.path.join(day_dir, tod)
                spath = os.path.join(sess_dir, "summary.json")
                if not os.path.isfile(spath):
                    continue
                try:
                    with open(spath) as fh:
                        s = json.load(fh)
                except Exception:
                    continue
                s["date"] = s.get("date", day)
                s["time"] = s.get("time", tod)
                s["current"] = (sess_dir == LOG.get("dir"))
                out.append(s)
    except OSError:
        pass
    return jsonify(out[:200])


@app.route("/download")
def download():
    from flask import send_file
    if not LOG["path"]:
        return Response("no session log active (run without --no-log)",
                        status=404, mimetype="text/plain")
    try:
        LOG["fh"].flush()
    except Exception:
        pass
    return send_file(os.path.abspath(LOG["path"]), as_attachment=True)


# -- launcher and live control (launcher.py, pipeline_control.py) ----------
# ランチャーと動作中の操作
@app.route("/api/launcher", methods=["GET", "POST"])
def launcher_route():
    """GET: what runs, the option schema, maps and cars, the saved options.
    POST {action: launch|stop|relaunch, options}. / 起動状態・設定・起動依頼。"""
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        a = body.get("action")
        if a not in pipeline_control.LAUNCH_ACTIONS:
            return jsonify({"error": "unknown action"}), 400
        ls = pipeline_control.read_json(pipeline_control.LAUNCHER_STATE_PATH) or {}
        if a == "stop" and ls.get("phase") in (None, "off", "idle"):
            # a pipeline started from the command line: ask it directly
            # コマンドラインから起動したパイプラインには直接依頼する
            pipeline_control.request(pipeline_control.CONTROL_PATH, "stop")
        else:
            opts = launch_options.clean(body.get("options")) if body.get("options") else None
            if opts:
                launch_options.save(opts)
            pipeline_control.request(pipeline_control.LAUNCH_PATH, a, options=opts)
    ls = pipeline_control.read_json(pipeline_control.LAUNCHER_STATE_PATH) or {}
    ps = pipeline_control.read_json(pipeline_control.STATE_PATH) or {}
    now = time.time()
    launcher_live = ls.get("phase") not in (None, "off") and now - float(ls.get("t", 0)) < 5.0
    pipeline_live = ps.get("phase") in ("running", "switching") and now - float(ps.get("t", 0)) < 5.0
    if not hasattr(launcher_route, "catalog"):
        launcher_route.catalog = launch_options.catalog()
    return jsonify({"launcher": ls if launcher_live else None, "pipeline": ps if pipeline_live else None,
                    "schema": launch_options.SCHEMA, "groups": launch_options.GROUPS,
                    "saved": launch_options.load_saved(), "catalog": launcher_route.catalog})


@app.route("/api/control", methods=["POST"])
def control_route():
    """Live change of the running simulation. / 動作中の変更。"""
    body = request.get_json(silent=True) or {}
    a = body.pop("action", None)
    if a not in pipeline_control.ACTIONS:
        return jsonify({"error": "unknown action"}), 400
    req = pipeline_control.request(pipeline_control.CONTROL_PATH, a, **body)
    return jsonify({"ok": True, "seq": req["seq"]})


@app.route("/api/pipeline/log")
def pipeline_log_route():
    """The last lines of the launched pipeline's output. / 起動したパイプラインの出力。"""
    n = max(10, min(2000, int(request.args.get("n", 200))))
    try:
        with open(pipeline_control.LOG_PATH, "rb") as fh:
            fh.seek(0, 2)
            size = fh.tell()
            fh.seek(max(0, size - 200 * n))
            lines = fh.read().decode("utf-8", "replace").splitlines()[-n:]
    except OSError:
        lines = []
    return jsonify({"lines": lines})


WEB_DIST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web", "dist")


@app.route("/")
def index():
    if os.path.exists(os.path.join(WEB_DIST, "index.html")):
        return send_file(os.path.join(WEB_DIST, "index.html"))
    return Response(NO_BUILD, mimetype="text/html")


@app.route("/favicon.svg")
def web_favicon():
    return send_from_directory(WEB_DIST, "favicon.svg")


@app.route("/assets/<path:name>")
def web_assets(name):
    """The built React app (web/dist, `npm run build` in web/). / React のビルド成果物。"""
    return send_from_directory(os.path.join(WEB_DIST, "assets"), name, max_age=31536000)


@app.route("/models/<path:name>")
def web_models(name):
    """3D models the app loads by URL (web/public/models -> web/dist/models:
    lmh_minipod.glb for the Mini Pods page). Revalidated, since the asset
    build may replace a model under the same name. / 3D モデル。"""
    return send_from_directory(os.path.join(WEB_DIST, "models"), name, max_age=0)


# The page itself is the React app in web/ (built into web/dist by
# `npm run build`, web/DEV.md). This is shown only when no build exists.
# 画面は web/ の React アプリ（ビルド前のみ以下を表示）
NO_BUILD = """<!doctype html><meta charset="utf-8"><title>CARLA HIL</title>
<body style="background:#0b1120;color:#e2e8f0;font:15px system-ui;padding:40px">
<h2>The dashboard UI is not built yet</h2>
<p>Build it once (Node.js in ~/.local/node):</p>
<pre style="background:#151e31;padding:12px;border-radius:8px">export PATH=~/.local/node/bin:$PATH
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/web &amp;&amp; npm install &amp;&amp; npm run build</pre>
<p>The JSON routes (/telemetry, /api/launcher, ...) work already.</p></body>"""


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--dashboard-port", type=int, default=5000)
    p.add_argument("--log-dir",
                   default=os.path.join(
                       os.path.dirname(os.path.abspath(__file__)), "data"),
                   help="Directory for CSV session logs "
                        "(default: <script dir>/data)")
    p.add_argument("--no-log", action="store_true",
                   help="Disable CSV session logging")
    p.add_argument("--delivery-live-dir", default=delivery_live.DEFAULT_DIR,
                   help="delivery_sim.py's live state folder (Drones tab)")
    args = p.parse_args()
    global DELIVERY_DIR
    DELIVERY_DIR = args.delivery_live_dir
    DRONE_WIN.update(host=args.host, port=args.port)
    # SIGTERM (the launcher stopping us) exits cleanly; the drone windows are
    # their own processes and close themselves (Esc / their close button)
    # SIGTERM で正常終了。ドローン窓は独立プロセス
    _signal.signal(_signal.SIGTERM, lambda *_: sys.exit(0))

    log_dir = None if args.no_log else args.log_dir
    t = threading.Thread(
        target=telemetry_worker, args=(args.host, args.port, log_dir),
        daemon=True)
    t.start()
    threading.Thread(target=vehicle_tuning_worker, args=(args.host, args.port),
                     daemon=True).start()
    threading.Thread(target=season_worker, args=(args.host, args.port),
                     daemon=True).start()

    print("[dashboard] serving http://0.0.0.0:%d/" % args.dashboard_port)
    app.run(host="0.0.0.0", port=args.dashboard_port,
            debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
